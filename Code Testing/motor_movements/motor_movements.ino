const int RFpin = 5;
const int RBpin = 6;
const int LFpin = 10;
const int LBpin = 11;

//test functions for PWM control:
//speed is a percent of duty cycle for controlling speed, and PWM converts that to an input usable by arduino
const int speed = 100;
const int PWM = speed * 2.55;

void setup() {
  Serial.begin(9600);

  pinMode(RFpin, OUTPUT);
  pinMode(RBpin, OUTPUT);
  pinMode(LFpin, OUTPUT);
  pinMode(LBpin, OUTPUT);
}

void loop() {

  forward();
  delay(1000);

  bankr();
  delay(1000);

  stop();
  delay(1000);

  forward();
  delay(1000);

  bankl();
  delay(1000);

  stop();
  delay(3000);

  
}


void forward() {
  analogWrite(RFpin, PWM);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, PWM);
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

void bankr(){
  analogWrite(RFpin, PWM/2);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, PWM);
  analogWrite(LBpin, 0);
}
void bankl(){
  analogWrite(RFpin, PWM);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, PWM/2);
  analogWrite(LBpin, 0);
}

void turnr(){
  analogWrite(RFpin, PWM);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, 0);
  analogWrite(LBpin, 0);
}
void turnl(){
  analogWrite(RFpin, 0);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, PWM);
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
void turnr_rev(){
  analogWrite(RFpin, 0);
  analogWrite(RBpin, 0);
  analogWrite(LFpin, 0);
  analogWrite(LBpin, PWM);
}
void turnl_rev(){
  analogWrite(RFpin, 0);
  analogWrite(RBpin, PWM);
  analogWrite(LFpin, 0);
  analogWrite(LBpin, 0);
}