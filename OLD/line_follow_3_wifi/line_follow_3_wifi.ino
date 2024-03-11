#include <WiFiS3.h>

//wifi name and password
char ssid[] = "groupW7";
char pass[] = "test1234";
WiFiServer server(5200);

//pin initialisation
//motor pins
const int RFpin = 5;
const int RBpin = 6;
const int LFpin = 10;
const int LBpin = 11;
//IR sensor pins
const int LEYE = 12;
const int REYE = 13;
//ultrasonic sensor pins
const int trig = 9;
const int echo = 8;

//global variables
//speed is 100% of max power you want the motors to operate at
const int speed = 45;
//distance in cm the robot stops before an obstical
const int stopDist = 30;
//variable for telling the robot what signal will be the line, can change depending on the tape and background colours
const bool line = LOW;

//initialising other global variables, reccomend do not change
const int PWM = speed * 2.55;  //converts speed to 8-bit PWM signal
bool firstConnect = true;      //variable for the first wifi connection to the client
float duration, distance;      //variables for thhe ultra sonic sensor


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

  //wifi activation
  WiFi.beginAP(ssid, pass);
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address:");
  Serial.println(ip);
  server.begin();
}

bool run = false;
bool pause = true;
void loop() {
  //basic delay for timing vital actions: wifi, movement, etc.
  //anything needing a longer delay should be done with a timer so as not to slow down vital actions
  delay(10);

  //wifi polling, reconnecting, and notification on first connect
  WiFiClient client = server.available();
  if (client.connected() && firstConnect) {
    client.write("Hello! Arduino Connected!");
    //Serial.println("connected!");
    firstConnect = false;
  }

  if (client.read() == 'x') {
    run = true;
    pause = false;
  } else if (client.read() == 'y') {
    run = false;
    pause = true;
  }

  //movement polling: line checking, wireless start/stop signals, distanc etc
  if (run) {
    distPoll();
    if (distance > stopDist) {
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
      stop();
    }
  } else if (stop) {
    stop();
  }

  //continue here
  //TODO:
  //setup hall effect sensors
  //report things to processing
}

//functions for any sensors which need more than one digitalRead/analogRead
void distPoll() {
  //function for polling distance using the ultrasonic sensor
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  //takes the pulse return time and converts to a distance in cm
  duration = pulseIn(echo, HIGH);
  distance = duration / 58;
}

//movement functions to be called in main code to reduce the length of loop()
//many of these are fine as is but could benifit from some tuning
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
void estop() {
  //emergency stop function, stops quicker then simple stop
  //only to be used if the robot is in immediate danger of hitting something to prevent damage
  //it stops quicker by running the motors in reverse momentarily before cutting off power to motors
  analogWrite(RFpin, 0);
  analogWrite(RBpin, PWM);
  analogWrite(LFpin, 0);
  analogWrite(LBpin, PWM);
  delay(100);
  analogWrite(RFpin, 0);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, 0);
  analogWrite(LBpin, 0);
}