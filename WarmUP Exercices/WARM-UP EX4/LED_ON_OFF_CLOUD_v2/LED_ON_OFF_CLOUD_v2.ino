const int ledPin = 2;
const int inputPin = 1;
int ledState = LOW;

volatile boolean flag  ;

void isr() 
{flag = true;
}

void setup() {
  Serial.begin(9600);
  // Pin setup
  pinMode(ledPin, OUTPUT);
  //pinMode(inputPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(inputPin), isr, RISING); //changed the pin from input to interrupt
}

void loop() {
  Serial.println(flag);
  
  if (flag) {
    // Toggle ledState variable
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
    flag=false;
  }
  //delay(500);
}
