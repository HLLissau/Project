#include "Timer5.h"

int led = 10; // led referred to in 10
volatile int count=0;

long t = millis();    // timer variable for printout

void setup(){
  pinMode(led,OUTPUT);

	SerialUSB.begin(115200); // debug output at 115200 baud
	while (!SerialUSB) ; // wait for moritor to be ready
		
	SerialUSB.println("starting");    
	MyTimer5.begin(1);  // define frequency of interrupt    // 200=for toggle every 5msec
  MyTimer5.attachInterrupt(Timer5_IRQ);// define the interrupt callback function
  MyTimer5.start(); // start the timer
}

// will be called by the MyTimer5 object
// toggles LED state at pin 10
void Timer5_IRQ(void) {
    static bool on = false;
    
    count++;  // count number of toggles
    if (on == true) {
      on = false;
        digitalWrite(led,LOW);
    } 
    else {
      on = true;
        digitalWrite(led,HIGH);
    }

}

void loop()
{
	// print every 10 secs how often the
    // timer interrupt was called

	while (true) {
		if (millis() - t >= 10000) {
			t = millis();
      SerialUSB.print(millis());
 			SerialUSB.print("  count=");
			SerialUSB.println(count);
		}
	}
}