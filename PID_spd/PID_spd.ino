//PID setup file for constant speed

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
//speed is 100% of max power you want the motors to operate at
const int speed = 52;
//distance in cm the robot stops before an obstical
const int stopDist = 20;
//variable for telling the robot what signal will be the line, can change depending on the tape and background colours
const bool line = LOW;
//wheel encoder variables
const float wheeld = 6.5;  //diameter of the wheels
const int hallc = 8;       //amount of state changes per encoder rotation

//initialising other global variables, reccomend do not change
const int PWM = speed * 2.55;  //converts speed to 8-bit PWM signal
bool firstConnect = true;      //variable for the first wifi connection to the client
float duration, distance;      //variables for the ultra sonic sensor
//variabls for keeping the arduino running or paused as well as object detection
bool run = false, pause = true, objDetect = false;
char reader;
//variabls for wheel encoder
volatile int lencoder = 0, rencoder = 0;
int travelDist = 0;  // distance traveled in cm
//report timers
unsigned long repTime1c = 0, repTime1p = 0;
//variable for temporary holding report strings before converting to char and sending to processing
String report;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
