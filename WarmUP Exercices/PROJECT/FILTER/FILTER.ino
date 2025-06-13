// Pin configuration
const int inputPin = A1; // Analog input
const int outputPin = A0; // DAC output (real analog)

// Filter configuration
const float fc = 50.0;                  // Cutoff frequency in Hz
const float RC = 1.0 / (2.0 * PI * fc); // Time constant RC

// Variables
unsigned long lastMicros = 0;
float y_prev = 0.0; // Previous filtered value

void setup() {
  analogReadResolution(10);  // 10-bit ADC
  analogWriteResolution(10); // 10-bit DAC
  Serial.begin(9600);
}

void loop() {
  // Measure time step ΔT
  unsigned long now = micros();
  float deltaT = (now - lastMicros) / 1000000.0; // in seconds
  lastMicros = now;

  // Compute alpha for this sample
  float alpha = deltaT / (RC + deltaT);

  // Read input voltage (0-1023)
  int raw = analogRead(inputPin);
  float x = raw; // 10-bit input

  // Apply first-order low-pass filter
  float y = alpha * x + (1.0 - alpha) * y_prev;
  y_prev = y;

  // Output to DAC
  analogWrite(outputPin, (int)y);
}

