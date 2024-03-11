#include <WiFiS3.h>

char ssid[] = "groupW7";
char pass[] = "498";
WiFiServer server(5200);


//pin initialisation
const int RFpin = 5;
const int RBpin = 6;
const int LFpin = 10;
const int LBpin = 11;

const int LEYE = 12;
const int REYE = 13;

const int trig = 9;
const int echo = 8;

//global variables
//speed is 100% of max power you want the motors to operate at.
const int speed = 45;
//distance in cm the robot stops before an obstical
const int stopDist = 30;

const int PWM = speed * 2.55;  //converts speed to 8-bit PWM signal
const bool line = LOW;
float duration, distance;

void setup() {
  Serial.begin(9600);

  pinMode(RFpin, OUTPUT);
  pinMode(RBpin, OUTPUT);
  pinMode(LFpin, OUTPUT);
  pinMode(LBpin, OUTPUT);

  pinMode(LEYE, INPUT);
  pinMode(REYE, INPUT);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  WiFi.beginAP(ssid, pass);
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address:");
  Serial.println(ip);
  server.begin();
}

void loop() {

  delay(10);

  WiFiClient client = server.available();
  if (client.connected()) {
    client.write("Hello! Arduino Connected!");
    Serial.println("connected!");
    client.stop();
  }

  if (client.read() == 'x') {
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
  } else if (client.read() == 'y') {
    delay(1000);
  }
}

void distPoll() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH);
  distance = duration / 58;
}

//many of these are fine as is but could benifit from some tuning
void forward() {
  analogWrite(RFpin, PWM);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, PWM * 0.80);
  analogWrite(LBpin, 0);
}
void back() {
  analogWrite(RFpin, 0);
  analogWrite(RBpin, PWM);
  analogWrite(LFpin, 0);
  analogWrite(LBpin, PWM);
}
void stop() {
  analogWrite(RFpin, 0);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, 0);
  analogWrite(LBpin, 0);
}
void estop() {
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

void bankr() {
  analogWrite(RFpin, 0);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, PWM);
  analogWrite(LBpin, 0);
}
void bankl() {
  analogWrite(RFpin, PWM);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, 0);
  analogWrite(LBpin, 0);
}

void pivotr() {
  analogWrite(RFpin, 0);
  analogWrite(RBpin, PWM);
  analogWrite(LFpin, PWM);
  analogWrite(LBpin, 0);
}
void pivotl() {
  analogWrite(RFpin, PWM);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, 0);
  analogWrite(LBpin, PWM);
}