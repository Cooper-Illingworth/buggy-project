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
//PID variables
int setpoint = 30;                            //starting setpoint for velocity in cm/s
float output;                                 //variables for PID output
const float kp = .01, ki = .0005, kd = .001;  //PID gains for proportional, integral, and derivative
//!!!UNSURE!!! do we need to get new variables and retune for following PID?

//initialising other global variables, !!!RECCOMEND DO NOT TOUCH!!!
bool firstConnect = true;  //variable for the first wifi connection to the client
//variabls for keeping the arduino running or paused as well as object detection
bool run = false, objDetect = false;
//variabls for wheel encoder
volatile int lencoder = 0, rencoder = 0;
int travelDist = 0, travelDistP = 0;  //tracking distance traveled in cm
//report timers
unsigned long repTime = 0, repTimep1 = 0, repTimep2;
//tracking time for current, previous, and elapsed times
unsigned long PIDtc = 0, PIDtp = 0, PIDte;
//errors for proportional, integral, derivitate, and last proportional
double p_error = 0, i_error = 0, d_error = 0, l_error = 0;


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

  //general calculations for traveldistance and velocity used for PID and reporting
  travelDist = ((lencoder + rencoder) / (float)(2 * hallc)) * (wheeld * 3.1415);
  velocity = (travelDist - travelDistP) / (float)(PIDte / 1000.0);
  //Serial.println(velocity);
  //Serial.println(travelDist - travelDistP);
  travelDistP = travelDist;

  //general run loop handling movement, distance polling, and line detection
  if (run) {

    //events for when run is true
    distPoll();
    if (distance > stopDist && distance != 0) {
      PWM = PIDspd(velocity);
      objDetect = false;
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
      objDetect = true;
      stop();
      //stop counter for the Integral to prevent total error issues?
      PIDtp = millis();
    }

  } else {

    //stop for when the buggy is told to stop
    stop();
    //stop counter for the Integral to prevent total error issues?
    PIDtp = millis();
  }
}

//PID for speed control
int PIDspd(int input) {
  //setsup current and elapsed time
  PIDtc = millis();
  PIDte = PIDtc - PIDtp;

  //calculates p, i, and d, terms
  p_error = setpoint - input;
  i_error += p_error * PIDte;
  d_error = (p_error - l_error) / PIDte;

  //calculates output
  output = kp * p_error + ki * i_error + kd * d_error;

  //sets previous variables
  l_error = p_error;
  PIDtp = PIDtc;

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
    reader = client.read();  //PROBLEM LINE
    //Serial.println(reader);
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
  if (repTime - repTimep1 > 5000 && !objDetect) {
    //report for only distance traveled
    report = String(travelDist) + " cm traveled. ";
    client.write(report.c_str());
    //resets timer
    repTimep1 = millis();
  } else if (repTime - repTimep1 > 5000 && objDetect) {
    //report for only distance traveled AND object detected
    client.write("object detected!");
    //resets both timers to prevent over reporting
    repTimep1 = millis();
    repTimep2 = millis();
  }
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
