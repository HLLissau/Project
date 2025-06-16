#include "Timer5.h"  //import Timer5 interrupt library

const int ADCPin = A1;
const int DACPin = A0;
const int testpin = 3;
const int sampleFrequency = 10000;  // max with disabled: 16370 no more than 16400
const int resolution = 10;
// filtering
#define FILTER_ENABLED true
const float deltaT = (1.0 / sampleFrequency);
const float fc = 50.0;                   // Cutoff frequency in Hz
const float RC = 1.0 / (2.0 * PI * fc);  // Time constant RC
const float alpha = deltaT / (RC + deltaT);
float y_prev = 0.0;  // Previous filtered value
// zero-crossing
const int treshold = 248;  // 248 is (0.8mV (offset) * 1023) / 3.3V
#define TRESHOLD_CROSSING_COUNTER true
volatile byte crossingFlag = 0;
int crossingCounter = 0;
int sampleCounter = 0;
const int amountBeforeCalculateFrequency = 100;
const float measuredSampleRate = 10922;
bool output = 0;
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
  return (int)result;
}

int zeroCrossing(int newMeasurement) {
  static int lastMeasurement = 0;  // static so only the first time put it to zero
  if (lastMeasurement >= treshold || newMeasurement < treshold) {
    lastMeasurement = newMeasurement;
    return false;
  }
  lastMeasurement = newMeasurement;
  return true;
}
void setup() {

  pinMode(testpin, OUTPUT);
  pinMode(ADCPin, INPUT);
  analogReadResolution(resolution);
  pinMode(DACPin, OUTPUT);
  analogWriteResolution(resolution);

  Serial.begin(9600);
  MyTimer5.begin(sampleFrequency);          // 200=for toggle every 5msec
  MyTimer5.attachInterrupt(readADCSignal);  //Digital Pins=3 with Interrupts
  AdcBooster();
}

void loop() {

  if (crossingFlag == 1) {
    crossingCounter = crossingCounter + 1;

    if (crossingCounter >= amountBeforeCalculateFrequency) {
      float result = crossingCounter/(sampleCounter*(1 / measuredSampleRate));
      Serial.print("Frequency measured to: ");
      Serial.println(result);
      crossingCounter = 0;
      sampleCounter = 0;
    } 
     crossingFlag = 0;
  }
}
void readADCSignal() {
  static int measuredValue = 0;
#if FILTER_ENABLED
  measuredValue = filter(analogRead(ADCPin));
#else
  measuredValue = analogRead(ADCPin);
#endif

#if TRESHOLD_CROSSING_COUNTER
  if (zeroCrossing(measuredValue)) {
    crossingFlag = 1;
  }
#endif
  analogWrite(DACPin, measuredValue);
  sampleCounter = sampleCounter + 1;
  if (output){
    digitalWrite(testpin, 1);
  } else {
     digitalWrite(testpin, 0);
  }
  output = !output;
}