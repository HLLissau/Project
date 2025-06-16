#include "Timer5.h"  //import Timer5 interrupt library

const int ADCPin = A1;
const int DACPin = A0;
const int frequency = 10000;   // max with disabled: 16370 no more than 16400
const int resolution = 10;
// filtering
#define FILTER_ENABLED true
const float deltaT = (1.0 / frequency);
const float fc = 50.0;                   // Cutoff frequency in Hz
const float RC = 1.0 / (2.0 * PI * fc);  // Time constant RC
const float alpha = deltaT / (RC + deltaT);
float y_prev = 0.0;  // Previous filtered value

void AdcBooster() {
  ADC->CTRLA.bit.ENABLE = 0;  // Disable ADC
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;                                             // Wait for synchronization
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 |    // Divide Clock by 16.
                   ADC_CTRLB_RESSEL_10BIT;        // Result on 10 bits 8/10/12
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample
                     ADC_AVGCTRL_ADJRES(0x00ul);  // Adjusting result by 0
  ADC->SAMPCTRL.reg = 0x00;                       // Sampling Time Length = 0
  ADC->CTRLA.bit.ENABLE = 1;                      // Enable ADC
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;  // Wait for synchronization
}  // AdcBooster

int filter(int x) {
  float result = alpha * x + (1.0 - alpha) * y_prev;
  y_prev = result;
  //Serial.println(result);
  return (int) result;
}

void setup() {

  pinMode(ADCPin, INPUT);
  analogReadResolution(resolution);
  pinMode(DACPin, OUTPUT);
  analogWriteResolution(resolution);
  
  Serial.begin(9600);
  MyTimer5.begin(frequency);                // 200=for toggle every 5msec
  MyTimer5.attachInterrupt(readADCSignal);  //Digital Pins=3 with Interrupts
  AdcBooster();
}

void loop() {

}

void readADCSignal() {
  #if FILTER_ENABLED
    analogWrite(DACPin, filter(analogRead(ADCPin)));
  #else 
    analogWrite(DACPin, analogRead(ADCPin));
  #endif
}

