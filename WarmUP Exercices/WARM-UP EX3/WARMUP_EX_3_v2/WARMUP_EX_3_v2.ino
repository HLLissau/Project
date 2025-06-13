#include "Timer5.h"        //import Timer5 interrupt library
String LEDStatus = "OFF";  //Globle variable
int RedLED = 2;
int YellowLED = 3;
void setup() {  // put your setup code here, to run once:
  pinMode(RedLED, OUTPUT);
  pinMode(YellowLED, OUTPUT);
  // define frequency of interrupt
  MyTimer5.begin(4);                      // 200=for toggle every 5msec
  MyTimer5.attachInterrupt(BlinkYellow);  //Digital Pins=3 with Interrupts
  Serial.begin(9600);
}  //set up serial port and data rate Baut bits/sec.
void loop() {                  // put your main code here, to run repeatedly:
  digitalWrite(RedLED, HIGH);  //Turn the Red LED on
  delay(1000);                 //delay 1 sec.
  digitalWrite(RedLED, LOW);   //Turn the Red LED on
  delay(1000);                 //delay 1 sec.
}
void BlinkYellow() {
  if (LEDStatus == "ON") {
    digitalWrite(YellowLED, LOW);
    LEDStatus = "OFF";
    return;
  }
  if (LEDStatus == "OFF") {
    digitalWrite(YellowLED, HIGH);
    LEDStatus = "ON";
    return;
  }
}
