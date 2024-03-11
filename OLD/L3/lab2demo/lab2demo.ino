//"You didn't update the sensor definitions!"
// CHANGE THESE TO MATCH YOUR WIRING, THEN DELETE THE PREVIOUS "#error" LINE
const int LEYE = 12;
const int REYE = 13;
bool L_PREV = true;
bool R_PREV = true;

void setup() {
  Serial.begin(9600);

  pinMode( LEYE, INPUT );
  pinMode( REYE, INPUT );
}

void loop() {

  if( digitalRead( LEYE ) != L_PREV && digitalRead( REYE ) == R_PREV){
    Serial.println("left has changed. ");
    L_PREV = digitalRead(LEYE);
  } else if(digitalRead( LEYE ) == L_PREV && digitalRead( REYE ) != R_PREV){
    Serial.println("right has changed.");
    R_PREV = digitalRead(REYE);
  } else if(digitalRead( LEYE ) != L_PREV && digitalRead( REYE ) != R_PREV){
    Serial.println("left and right has changed.");
    L_PREV = digitalRead(LEYE);
    R_PREV = digitalRead(REYE);
  }
  
  delay(1000);
}
