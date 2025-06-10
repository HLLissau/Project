const int switchPin = 8; // constant so switchPin always will be 8
unsigned long previousTime = 0; // will hold the time a LED was last changed
int switchState = 0; 
int prevSwitchState = 0; 
int led = 2;                    // This will be used to count which LED is the next one to be turned on. Start out with pin 2
long interval = 2000;         // long datatype 10 minutes

void setup() { 
  for(int x = 2;x<8;x++){
    pinMode(x, OUTPUT);  // assign all ports as output
  } 
  pinMode(switchPin, INPUT); // assign switchPin to 8
}

void loop(){
unsigned long currentTime = millis();           // o get the amount of time the Arduino has been running with millis() and store it
if(currentTime - previousTime > interval) {
  previousTime = currentTime;
  digitalWrite(led, HIGH);  // led high
  led++;                    // next led 
  if(led == 7){
  }
}

switchState = digitalRead(switchPin);   // read if LOW or HIGH

if(switchState != prevSwitchState){
  for(int x = 2;x<8;x++){ 
    digitalWrite(x, LOW);
  } 
  led = 2;
  previousTime = currentTime;
}
prevSwitchState = switchState;
}