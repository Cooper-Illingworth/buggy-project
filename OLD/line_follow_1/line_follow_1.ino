//pin initialisation
const int RFpin = 5;
const int RBpin = 6;
const int LFpin = 10;
const int LBpin = 11;
const int LEYE = 12;
const int REYE = 13;

//global variables
//speed is 100% of max power you want the motors to operate at.
const int speed = 80;
const int PWM = speed * 2.55;  //converts speed to 8-bit PWM signal
const bool line = LOW;
//float duration, distance;

void setup() {
  Serial.begin(9600);

  pinMode(RFpin, OUTPUT);
  pinMode(RBpin, OUTPUT);
  pinMode(LFpin, OUTPUT);
  pinMode(LBpin, OUTPUT);

  pinMode(LEYE, INPUT);
  pinMode(REYE, INPUT);

}

void loop() {

  delay(50);
    if(digitalRead(LEYE) != line && digitalRead(REYE) != line ){
    forward();
  }
  if(digitalRead(LEYE) == line){
    bankl();
  }
  if(digitalRead(REYE) == line ){
    bankr();
  }

}


//many of these are fine as is but could benifit from some tuning
void forward() {
  analogWrite(RFpin, PWM);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, PWM*0.90);
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