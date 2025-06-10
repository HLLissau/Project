int ledPin = 1;
int inputPin = 2;

bool ledState = LOW;
int lastInputState = LOW;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(inputPin, INPUT); // No pullup assumed. Use INPUT_PULLUP if needed.
}

void loop() {
  int currentInputState = digitalRead(inputPin);

  // Detect rising edge
  if (lastInputState == LOW && currentInputState == HIGH) {
    // Toggle LED state
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
  }

  // Save the state for the next loop
  lastInputState = currentInputState;

  delay(10); // small delay to debounce
}
