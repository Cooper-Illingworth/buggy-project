//TODO:
/*
DEBUG and test PID(and tune PIDfollow)
silver challenge reporting:
report distance to object. TO DEBUG
speed of buggy. TO DEBUG
PID contorl mode. TO DEBUG
and also the reference speed for some reason?
*/

#include <WiFiS3.h>

//wifi name and password
char ssid[] = "groupW7";
char pass[] = "test1234";
WiFiServer server(5200);

//pin initialisation
//motor pins
const int RFpin = 5, RBpin = 6, LFpin = 10, LBpin = 11;
//IR sensor pins
const int LEYE = 12, REYE = 13;
//ultrasonic sensor pins
const int trig = 9, echo = 8;
//hall effect sensor pins(interrupt pins)
const int LHALL = 2, RHALL = 3;

//global variables
//utrasonic sensor variables
const int stopDist = 5;     //distance in cm the robot stops before an obstical
const int followDist = 15;  // distance in cm the orbot follows an obstical(for us with PID)
float duration, distance;   //output variables for the ultra sonic sensor calculations/function
//IR sensor variable for the signal the line its following will appear as, can change depending colours used
const bool line = LOW;
//motor and wheel encoder variables
int PWM;                   //8-bit PWM signal for motors
const float wheeld = 6.5;  //diameter of the wheels
const int hallc = 8;       //amount of state changes per encoder rotation
float velocity;            // output variable for motor velocity
//WiFi variables
char reader;    //input variable for reading a charcter over WiFi
String report;  //temporary holder for reports before converting to char and wireless send
int reportFreq = 1000;
//PID variables
int setpoint = 15;                                  //starting setpoint for velocity in cm/s
float output;                                       //variables for PID output
const float kp_s = .08, ki_s = .002, kd_s = .002;  //PIDspd gains for proportional, integral, and derivative
const float kp_f = .002, ki_f = .004, kd_f = .08;  //PIDflw gains for proportional, integral, and derivative

//initialising other global variables, !!!RECCOMEND DO NOT TOUCH!!!
bool firstConnect = true;  //variable for the first wifi connection to the client
//variabls for keeping the arduino running or paused as well as object detection
bool run = false, objFollow = false, objDetect = false;
//variabls for wheel encoder
volatile int lencoder = 0, rencoder = 0;
int travelDist = 0, travelDistP = 0;  //tracking distance traveled in cm
//report timers
unsigned long repTime = 0, repTimep1 = 0, repTimep2 = 0;
//tracking time for current, previous, and elapsed times
unsigned long PIDtc = 0, PIDtp = 0, PIDte;
//errors for proportional, integral, derivitate, and last proportional
double s_perror = 0, s_ierror = 0, s_derror = 0, s_lerror = 0;
double f_perror = 0, f_ierror = 0, f_derror = 0, f_lerror = 0;


void setup() {
  Serial.begin(9600);

  //pin initialisation
  pinMode(RFpin, OUTPUT);
  pinMode(RBpin, OUTPUT);
  pinMode(LFpin, OUTPUT);
  pinMode(LBpin, OUTPUT);

  pinMode(LEYE, INPUT);
  pinMode(REYE, INPUT);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  pinMode(LHALL, INPUT_PULLUP);  //PULLUP is a resistor within the arduino, needed for the encoders to function properly
  pinMode(RHALL, INPUT_PULLUP);

  //initialises interrupts
  attachInterrupt(digitalPinToInterrupt(LHALL), lrev, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RHALL), rrev, CHANGE);

  //wifi activation
  WiFi.beginAP(ssid, pass);
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address:");
  Serial.println(ip);
  server.begin();
}


void loop() {
  //basic delay for timing vital actions: wifi, movement, etc.
  //anything needing a longer delay should be done with a timer to keep vital actions fast
  delay(10);

  //calls WiFi events
  wifiPoll();

  //general run loop handling movement, distance polling, and line detection
  if (run) {

    //events for when run is true
    distPoll();
    if (distance > stopDist && distance != 0) {
      objDetect = false;

      //moved here to prevent velocity error
      //general calculations for traveldistance and velocity
      travelDist = ((lencoder + rencoder) / (float)(2 * hallc)) * (wheeld * 3.1415);
      velocity = (travelDist - travelDistP) / (float)(PIDte / 1000.0);
      travelDistP = travelDist;
      //Serial.println(velocity);  //DEBUG

      //call PID controllers depending on distance from an object
      if (distance > (followDist + 15) || distance == 0) {
        PWM = PIDspd(velocity);
        objFollow = false;
      } else if (distance <= (followDist + 15) && distance != 0) {
        PWM = PIDflw(distance);
        objFollow = true;
      }

      //basic movement and line following logic
      if (digitalRead(LEYE) != line && digitalRead(REYE) != line) {
        forward();
      }
      if (digitalRead(LEYE) == line) {
        bankl();
      }
      if (digitalRead(REYE) == line) {
        bankr();
      }
    } else {
      //stop for when an object is detected
      stop();
      objDetect = true;
      objFollow = false;
      velocity = 0;
      //stop counter for the Integral to prevent total error issues?
      PIDtp = millis();
    }

  } else {
    //stop for when the buggy is told to stop
    stop();
    velocity = 0;
    //stop counter for the Integral to prevent total error issues?
    PIDtp = millis();
  }
}


//PID controllers
//PID for speed control
int PIDspd(int input) {
  //setsup current and elapsed time
  PIDtc = millis();
  PIDte = PIDtc - PIDtp;

  //calculates p, i, and d, terms
  s_perror = setpoint - input;
  s_ierror += s_perror * PIDte;
  s_derror = (s_perror - s_lerror) / PIDte;

  //calculates output
  output = kp_s * s_perror + ki_s * s_ierror + kd_s * s_derror;

  //sets previous variables
  s_lerror = s_perror;
  PIDtp = PIDtc;

  return output;
}
//PID for speed control
int PIDflw(int input) {
  //setsup current and elapsed time
  PIDtc = millis();
  PIDte = PIDtc - PIDtp;

  //calculates p, i, and d, terms
  f_perror = -(followDist - input);
  f_ierror += f_perror * PIDte;
  if (f_perror < 0) f_ierror = 0; 
  f_derror = (f_perror - f_lerror) / PIDte;

  //calculates output
  output = kp_f * f_perror + ki_f * f_ierror + kd_f * f_derror;

  //sets previous variables
  f_lerror = f_perror;
  PIDtp = PIDtc;
  Serial.println(output);

  return output;
}

//general wifi function for sending and recieving data
void wifiPoll() {
  //wifi polling, reconnecting, and notification on first connect
  WiFiClient client = server.available();
  if (client.connected() && firstConnect) {
    client.write("Hello! Arduino Connected!\n");
    Serial.println("connected!");
    firstConnect = false;
  }

  //reads for start 'x' and stop 'y' signals wirelessly
  //uses bool variables to have a constant on or off signal
  if (client.available()) {
    reader = client.read();
    //Serial.println(reader); //DEBUG
  }
  if (reader == 'x') {
    run = true;
  } else if (reader == 'y') {
    run = false;
  } else if (reader <= 30 && reader >= 5) {
    setpoint = reader;
  }

  //reporting and timer for distance travelled and object detection
  repTime = millis();
  if (repTime - repTimep1 > reportFreq) {
    //create custom report based on boolean factors

    //reports the mode the buggy is currently in
    //uses short codes for effeciency
    if (run && objFollow) {
      //follow object and clearence from object
      report = "m:f";
      report = report + "d:" + String(distance) + ":d";
    } else if (run && !objFollow && !objDetect) {
      //maintain speed
      report = "m:c";
    } else if (run && objDetect) {
      //pause for object
      report = "m:p";
      //report = report + "c:" + String(distance) + ":c";
    } else if (!run) {
      //stopped
      report = "m:s";
    }

    //reports the speed and distance travelled by the buggy
    report = report + "v:" + String(velocity) + ":v";
    report = report + "t:" + String(travelDist) + ":t";

    //send report
    client.write(report.c_str());

    //resets timer
    repTimep1 = millis();
  }
  //!!!CAUTION!!! might not be nesesary because of the GUI data which reports more often, if it is needed add "&& !objDetect" back to previous if statement
  /*
  else if (repTime - repTimep2 > reportFreq && objDetect) {
    //report for only distance traveled AND object detected
    client.write("Mode: Halt");
    //resets both timers to prevent over reporting
    repTimep1 = millis();
    repTimep2 = millis();
  }
  */
}
//function for polling distance using the ultrasonic sensor
void distPoll() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  //takes the pulse return time and converts to a distance in cm
  duration = pulseIn(echo, HIGH, 200000);  //timeout delay in microseconds
  distance = duration / 58;
}

//wheel encoder interrupt functions for tracking wheel revolutions
void lrev() {
  lencoder++;
}
void rrev() {
  rencoder++;
}

//movement functions to be called in main code to reduce the length of loop()
void forward() {
  //moves the buggy forward
  analogWrite(RFpin, PWM);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, PWM * 0.80);  // 0.80 allows to move straight, as the left motor is more powerful then the right
  analogWrite(LBpin, 0);
}
void bankr() {
  //simple right turning
  analogWrite(RFpin, 0);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, PWM);
  analogWrite(LBpin, 0);
}
void bankl() {
  //simple left turning
  analogWrite(RFpin, PWM);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, 0);
  analogWrite(LBpin, 0);
}
void stop() {
  //simple stop function that removes power to the motors
  analogWrite(RFpin, 0);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, 0);
  analogWrite(LBpin, 0);
}
