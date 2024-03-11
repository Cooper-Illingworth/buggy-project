const int trigPin = 9;
const int echoPin = 10;
const int ledPin = 13;

float duration, distance;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  distPoll();
  distWarn();
}

void distPoll(){
  unsigned long tp;
  unsigned long tc = millis();
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  if (tc - tp > 125) {
    duration = pulseIn(echoPin, HIGH);
    distance = (duration*.0343)/2;
    Serial.print("distance: ");
    Serial.println(distance);
    tp = millis();
  }
}
void distWarn(){
  unsigned long tp = millis();
  unsigned long tc;
  while(distance < 10){
    tc = millis();
    if (tc - tp < 250){
      digitalWrite(ledPin, HIGH);
    } else if (tc - tp < 500) {
    digitalWrite(ledPin, LOW);
    } else if (tc - tp > 500) { 
      tp = millis();
    }
    distPoll();
  }
  digitalWrite(ledPin, LOW);
}
