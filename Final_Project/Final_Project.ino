#include "Timer5.h"  //import Timer5 interrupt library
#include <math.h>
#include <LiquidCrystal.h>  // Include the LCD library
// Initialize the LCD with the pin numbers: (RS, E, D4, D5, D6, D7)
LiquidCrystal lcd(12, 11, 2, 3, 4, 5);
const int ADCPin = A1;
const int DACPin = A0;
const int testpin = 10;
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
const float measuredSampleRate = 1.0923 * sampleFrequency;
//interpolation  8
#define INTERPOLATION_ENABLED true && TRESHOLD_CROSSING_COUNTER  //need threshhold crossing to work.
int oldY = 0;
//LED 10
#define CONTROL_LED_ENABLED true
const int lowFreqLED = 14;
const int highFreqLED = 13;
const float lowCutoffFreq = 49.975;
const float highCutoffFreq = 50.025;
// RMS step12
#define RMS_ENABLED true
int amountOfRMSDataPoints = 0;
float samples = 0;
float scalingFactor = 1.009;
float conversion = scalingFactor * (3.3 / 1023.0);
// DROOP
#define DROOP_CONTROL_ENABLED true
const float f1 = 49.9;
const float f2 = 50.1;
const float i1 = 6;
const float i2 = 80;
const float droopConstant = (f2 - f1) / (i2 - i1);
const float droopIntercept = f1 - (droopConstant * i1);
const float droopCurrentConstant = 1 / droopConstant;
const float droopCurrentIntercept = -droopIntercept / droopConstant;
float current = 0;
//PWM
// we define all constant to generalize the study case
// First slope
const float PWMFirstCurrent1 = 6;
const float PWMFirstCurrent2 = 52;
const float PWMFirstDutyCycle1 = 10;
const float PWMFirstDutyCycle2 = 85;
const float PWMDroopConstant1 = (PWMFirstCurrent2 - PWMFirstCurrent1) / (PWMFirstDutyCycle2 - PWMFirstDutyCycle1);
const float PWMDroopIntercept1 = PWMFirstCurrent1 - (PWMDroopConstant1 * PWMFirstDutyCycle1);
const float PWMDroopConstantInverse1 = 1 / PWMDroopConstant1;
const float PWMDroopInterceptInverse1 = -PWMDroopIntercept1 / PWMDroopConstant1;
// Second slope
const float PWMSecondCurrent1 = 52.5;
const float PWMSecondCurrent2 = 80;
const float PWMSecondDutyCycle1 = 85;
const float PWMSecondDutyCycle2 = 96;
const float PWMDroopConstant2 = (PWMSecondCurrent2 - PWMSecondCurrent1) / (PWMSecondDutyCycle2 - PWMSecondDutyCycle1);
const float PWMDroopIntercept2 = PWMSecondCurrent1 - (PWMDroopConstant2 * PWMSecondDutyCycle1);
const float PWMDroopConstantInverse2 = 1 / PWMDroopConstant2;
const float PWMDroopInterceptInverse2 = -PWMDroopIntercept2 / PWMDroopConstant2;
//PWM output
// Output 250kHz PWM on timer TCC0 (6-bit resolution)
const int pWMfreq = 96 * 250;  //1kHz
float pWMDutyCycle = 0.80;
const int pWMPin = 7;


// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// AdcBooster
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
}
// filter function
int filter(int x) {
  float result = alpha * x + (1.0 - alpha) * y_prev;
  y_prev = result;
  //Serial.println(result);
  return (int)result;
}
// zero crossing function
int zeroCrossing(int y) {
  if (lastMeasurement >= treshold || y < treshold) {
    lastMeasurement = y;
    return false;
  }
  oldY = lastMeasurement;
  lastMeasurement = y;
  return true;
}
// interpolate zero crossing function
float interpolateZeroCrossingTime() {
  // y1 is current value, y0 is previous value, t1 is current sample index
  // y1 > threshold, y0 < threshold => rising zero-crossing
  float y1f = (float)lastMeasurement;
  float y0f = (float)oldY;
  float x = (y1f - (float)treshold) / (y1f - y0f);  // Linear interpolation factor (0 to 1)
  float interpolatedTime = (sampleCounter - x);
  return interpolatedTime;  // this function returns the interpolated time of the zero crossing
}
// control lights function
void controlLights(int state) {
  switch (state) {
    case 0:
      digitalWrite(lowFreqLED, LOW);
      digitalWrite(highFreqLED, LOW);
      break;
    case 1:
      digitalWrite(lowFreqLED, HIGH);
      digitalWrite(highFreqLED, LOW);
      break;
    case 2:
      digitalWrite(lowFreqLED, LOW);
      digitalWrite(highFreqLED, HIGH);
      break;
  }
}
void printToLCD(String toPrint, int lineNumber) {

  lcd.setCursor(0, lineNumber);  // Go to second line
  lcd.print(toPrint + " ");      // writing an extra blank space to delete duplicate caracters if string is shorter than last string
}
float currentRequest(float frequency) {
  // NEW FCR-N
  if ((frequency >= f1) && (frequency <= f2)) {
    return frequency * droopCurrentConstant + droopCurrentIntercept;
  } else if (frequency > f2) {
    return 80.0;
  } else {
    return 0.0;
  }
}
float PWMRequest(float current) {
  if (current < 6) {
    return 0.0;
  } else if (current < 52) {
    return 0.01 * (current * PWMDroopConstantInverse1 + PWMDroopInterceptInverse1);
  } else if (current > 52.5) {
    return 0.01 *( current * PWMDroopConstantInverse2 + PWMDroopInterceptInverse2);
  } else {
    return 0.85;
  }
}


void PWMsetup() {
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |  // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);    // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |        // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |  // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);         // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  // Enable the port multiplexer for the digital pin D7
  PORT->Group[g_APinDescription[pWMPin].ulPort].PINCFG[g_APinDescription[pWMPin].ulPin].bit.PMUXEN = 1;

  // Connect the TCC0 timer to digital output D7 - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |        // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |    // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;  // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |       // Reverse the output polarity on all TCC0 outputs
                   TCC_WAVE_WAVEGEN_DSBOTH;  // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE)
    ;  // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  REG_TCC0_PER = pWMfreq;  // Set the frequency of the PWM on TCC0 to 1kHz
  while (TCC0->SYNCBUSY.bit.PER)
    ;  // Wait for synchronization

  // Set the PWM signal to output 50% duty cycle
  REG_TCC0_CC3 = pWMDutyCycle * pWMfreq;  // TCC0 CC3 - on D7
  while (TCC0->SYNCBUSY.bit.CC3)
    ;  // Wait for synchronization

  // Divide the 48MHz signal by 1 giving 48MHz (20.83ns) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |  // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;           // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE)
    ;  // Wait for synchronization
}


// %%%%%%%%%%%%%%%%%%%%%%%%%%%%% SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void setup() {
  lcd.begin(16, 2);  // Set up the LCD with 16 columns and 2 rows
  pinMode(testpin, OUTPUT);
  pinMode(ADCPin, INPUT);
  analogReadResolution(resolution);
  pinMode(DACPin, OUTPUT);
  analogWriteResolution(resolution);
  pinMode(lowFreqLED, OUTPUT);
  pinMode(highFreqLED, OUTPUT);

  Serial.begin(9600);
  MyTimer5.begin(sampleFrequency);          // 200=for toggle every 5msec
  MyTimer5.attachInterrupt(readADCSignal);  //Digital Pins=3 with Interrupts
  AdcBooster();
  PWMsetup();
}
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void loop() {
  static float calculatedFreq;
  if (crossingFlag == 1) {
    crossingFlag = 0;
    crossingCounter = crossingCounter + 1;

    if (crossingCounter >= amountBeforeCalculateFrequency) {
      static int beginSampleCounter = 0;
      beginSampleCounter = sampleCounter;
      crossingCounter = 0;

// interpolation of zero-crossing
#if INTERPOLATION_ENABLED
      static float lastInterpolatedTime = 0;
      float interpolatedTime = interpolateZeroCrossingTime();
      float period = (interpolatedTime - lastInterpolatedTime);
      calculatedFreq = (measuredSampleRate / period) * amountBeforeCalculateFrequency;
      //Serial.print("Freq (interpolated): ");
      //Serial.println(calculatedFreq, 2);  // 3 decimal places
      lastInterpolatedTime = interpolatedTime;
#else  // use simple zero-crossing (average over "amountBeforeCalculateFrequency" periods)
      calculatedFreq = crossingCounter / (sampleCounter * (1 / measuredSampleRate));
      // Serial.print("Frequency:[Hz] ");
      // Serial.println(calculatedFreq, 2);

      sampleCounter = 0;
#endif
      printToLCD("FREQ: " + String(calculatedFreq, 3) + " Hz", 0);

// RMS calculation
#if RMS_ENABLED
      float rms = sqrt((samples) / amountOfRMSDataPoints);
      //float voltRMS = (rms/3.3)*240.0;
      samples = 0;
      amountOfRMSDataPoints = 0;
      //printToLCD("VOLT:  " + String(rms) + " V", 1);
#endif
#if DROOP_CONTROL_ENABLED
      current = currentRequest(calculatedFreq);
      pWMDutyCycle = PWMRequest(current);
      REG_TCC0_CC3 = pWMDutyCycle * pWMfreq;  // TCC0 CC3 - on D7

      //   printToLCD("cur:  " + String(current) + " A", 1);
      printToLCD("PWM:  " + String(100 * pWMDutyCycle) + " %", 1);

#endif
    }
  }

#if CONTROL_LED_ENABLED
  if (calculatedFreq < lowCutoffFreq) {
    controlLights(1);
  } else if (calculatedFreq > highCutoffFreq) {
    controlLights(2);
  } else {
    controlLights(0);
  }
#endif
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%% ISR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void readADCSignal() {
  digitalWrite(testpin, 1);  // set output for verification of interupt time
  static int measuredValue = 0;

  // filtering
  int readValue = analogRead(ADCPin);
#if FILTER_ENABLED
  measuredValue = filter(readValue);
#else
  measuredValue = readValue;
#endif

// saving for RMS
#if RMS_ENABLED
  float readValuesInVolts = ((float)readValue - treshold) * conversion;
  samples = samples + (readValuesInVolts * readValuesInVolts);
  amountOfRMSDataPoints = amountOfRMSDataPoints + 1;
#endif

// zero crossing
#if TRESHOLD_CROSSING_COUNTER
  if (zeroCrossing(measuredValue)) {
    crossingFlag = 1;
  }
#endif

  analogWrite(DACPin, measuredValue);
  sampleCounter = sampleCounter + 1;
  digitalWrite(testpin, 0);
}
