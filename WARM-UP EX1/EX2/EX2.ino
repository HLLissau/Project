int switchState = LOW;

#define GREEN 0
#define YELLOW 1
#define RED 2
#define BUTTON 5

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(BUTTON, INPUT);
}

// the loop function runs over and over again forever
void loop() {
  switchState = digitalRead(BUTTON);

  if (switchState == LOW) { 
    digitalWrite(GREEN, HIGH);
    digitalWrite(YELLOW, LOW);
    digitalWrite(RED, LOW);
  } else {
    delay(1000);
    digitalWrite(GREEN, LOW);
    digitalWrite(YELLOW, HIGH);
    digitalWrite(RED, LOW);

    delay(500);

    digitalWrite(YELLOW, LOW);
    digitalWrite(RED, HIGH);

    delay(3000);

    digitalWrite(YELLOW, HIGH);

    delay(500);
  }
}
