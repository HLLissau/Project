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
int lastMeasurement = 0;  // static so only the first time put it to zero

volatile byte crossingFlag = 0;
int crossingCounter = 0;
int sampleCounter = 0;
const int amountBeforeCalculateFrequency = 25;
const float measuredSampleRate = 1.0907* sampleFrequency;
bool output = 0;
//interpolation  8
#define INTERPOLATION_ENABLED true && TRESHOLD_CROSSING_COUNTER  //need threshhold crossing to work.

int oldY = 0;


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

int zeroCrossing(int y) {
  if (lastMeasurement >= treshold || y < treshold) {
    lastMeasurement = y;
    return false;
  }
  oldY = lastMeasurement;
  lastMeasurement = y;
  return true;
}

float interpolateZeroCrossingTime() {
  // y1 is current value, y0 is previous value, t1 is current sample index
  // y1 > threshold, y0 < threshold => rising zero-crossing
  float y1f = (float)lastMeasurement;
  float y0f = (float)oldY;
  float x = (y1f - (float)treshold) / (y1f - y0f);  // Linear interpolation factor (0 to 1)
  float interpolatedTime = sampleCounter - x;
  return interpolatedTime;  // this function returns the interpolated time of the zero crossing
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

    //interpolation of zero-crossing
    #if INTERPOLATION_ENABLED 
    static float lastInterpolatedTime = 0;
      float interpolatedTime = interpolateZeroCrossingTime();
      float period = interpolatedTime - lastInterpolatedTime;
      float interpolatedFreq = measuredSampleRate/period;
      Serial.print("Freq (interpolated): ");
      Serial.println(interpolatedFreq, 2);  // 3 decimal places
      lastInterpolatedTime = interpolatedTime;
    #else // use simple zero-crossing (average over "amountBeforeCalculateFrequency" periods)
    crossingCounter = crossingCounter + 1;

    if (crossingCounter >= amountBeforeCalculateFrequency) {
      float result = crossingCounter / (sampleCounter * (1 / measuredSampleRate));
      Serial.print("Frequency:[Hz] ");
      Serial.println(result,2);
      crossingCounter = 0;
      sampleCounter = 0;
    }
    #endif
    crossingFlag = 0;
  }
}

void readADCSignal() {
  static int measuredValue = 0;
// filtering
#if FILTER_ENABLED
  measuredValue = filter(analogRead(ADCPin));
#else
  measuredValue = analogRead(ADCPin);
#endif

// zero crossing
#if TRESHOLD_CROSSING_COUNTER
  if (zeroCrossing(measuredValue)) {
    crossingFlag = 1;
  }
#endif


  analogWrite(DACPin, measuredValue);
  sampleCounter = sampleCounter + 1;

  // to plot the real sampling frequency
  if (output) {
    digitalWrite(testpin, 1);
  } else {
    digitalWrite(testpin, 0);
  }
  output = !output;
}
