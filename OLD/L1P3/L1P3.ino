


String morsetable[] = {
    // Letters A-Z (indices 0-25)
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....",
    "..", ".---", "-.-", ".-..", "--", "-.", "---", ".--.",
    "--.-", ".-.", "...", "-", "..-", "...-", ".--", "-..-",
    "-.--", "--..", 
    // Digits start here (indices 26-35)
    ".----", "..--", "...--", "....-", "....",
    "-...", "--...", "---..", "----.", "-----"
};

int t_unit = 100;
const int LED_PIN = 13;


/* Convert a character to an index into the morse code table.
 *  Rules:
 *    If it's an uppercase letter (A-Z) then return 0-25
 *    If it's a digit (0-9) then return 26-35
 *    Anything else return -1
 *  Assumption:
 *    The character has already been converted to uppercase if it's a letter.
 */
int lookup(char c){

  // If the character is not a valid letter or digit then return -1
  if( !isAlphaNumeric(c) ){
    return -1;
  }


  // Now we do a bit of ASCII trickery to convert letters & digits
  // into positions in our lookup table. See https://www.arduino.cc/reference/en/language/variables/data-types/char/
  // Letters are easier
  if( isAlpha(c) ){
    return c - 'A';
  }

  if( isDigit(c) ){
    // add on 25 because the digits come after the letters in the table, so we skip that far
    return (c - '0') + 25; 
  }

}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode (LED_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // morse for for SOS, this is S sequence:
  digitalWrite(LED_PIN, HIGH);
  delay(t_unit);
  digitalWrite(LED_PIN, LOW);
  delay(t_unit);
  digitalWrite(LED_PIN, HIGH);
  delay(t_unit);
  digitalWrite(LED_PIN, LOW);
  delay(t_unit);
  digitalWrite(LED_PIN, HIGH);
  delay(t_unit);
  digitalWrite(LED_PIN, LOW);
  delay(t_unit);
  //pause
  delay(t_unit*3);
  
  //sequence for O:
  digitalWrite(LED_PIN, HIGH);
  delay(t_unit*3);
  digitalWrite(LED_PIN, LOW);
  delay(t_unit);
  digitalWrite(LED_PIN, HIGH);
  delay(t_unit*3);
  digitalWrite(LED_PIN, LOW);
  delay(t_unit);
  digitalWrite(LED_PIN, HIGH);
  delay(t_unit*3);
  digitalWrite(LED_PIN, LOW);
  delay(t_unit);
  //pause
  delay(t_unit*3);

  //sequence for second S
  digitalWrite(LED_PIN, HIGH);
  delay(t_unit);
  digitalWrite(LED_PIN, LOW);
  delay(t_unit);
  digitalWrite(LED_PIN, HIGH);
  delay(t_unit);
  digitalWrite(LED_PIN, LOW);
  delay(t_unit);
  digitalWrite(LED_PIN, HIGH);
  delay(t_unit);
  digitalWrite(LED_PIN, LOW);
  delay(t_unit);
  //pause
  delay(t_unit*3);

  //stop
  delay(t_unit*3);
  
  
}