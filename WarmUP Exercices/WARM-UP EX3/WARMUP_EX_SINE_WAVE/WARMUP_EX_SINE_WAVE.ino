
#define PI 3.1415926535897932384626433832795
const int DACPin = A0;
long t = micros();
const int f = 50;
float deltaT;
float funct;
int analogVal;
int analogVal2;
int analogVal3;


void setup() {
pinMode(DACPin, OUTPUT);
Serial.begin(9600);
}

void loop() {
  analogWriteResolution(10);
  deltaT = (( micros() - t)/1000000.0);
  funct =  sin(2*PI*f*deltaT);
  analogVal = ((funct + 1)*511.5);
  
  Serial.println(analogVal);
  analogVal2 = 1;
  analogVal3 = 1023;
  analogWrite(A0,analogVal2);
  delay(100);
  analogWrite(A0,analogVal3);
  delay(100);
  /*
  Serial.print("deltaT: ");
  Serial.println(deltaT);
  Serial.println(funct);
  Serial.println(analogVal);*/
}
