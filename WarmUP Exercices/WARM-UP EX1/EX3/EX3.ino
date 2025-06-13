int switchState = LOW;

#define TEMPERATURE 5

const int sensorPin = TEMPERATURE;
const float baselineTemp = 30.0;

// setup the diodes to low
void setup(){
  Serial.begin(9600); // open a serial port
 
 for(int pinNumber = 0; pinNumber<3; pinNumber++){
    pinMode(pinNumber,OUTPUT);
    digitalWrite(pinNumber, LOW);
  } 
}
// loop
void loop() {
  // print the real reading
  Serial.print("Sensor Value: ");
  int sensorVal = analogRead(sensorPin); 
  Serial.print(sensorVal);
  
  // convert the ADC reading to voltage
  Serial.print(", Volts: ");
  float voltage = (sensorVal/1024.0) * 5.0;
  Serial.print(voltage);
  
  // convert the voltage to temperature in degrees
  Serial.print(", degrees C:" ); 
  float temperature = (voltage - .5) * 100;
  Serial.println(temperature);

  if(temperature < baselineTemp){
    digitalWrite(0, LOW);
    digitalWrite(1, LOW);
    digitalWrite(2, LOW);
    }
    else if (temperature >= baselineTemp+2 && temperature < baselineTemp+4){
    digitalWrite(0, HIGH);
    digitalWrite(1, LOW);
    digitalWrite(2, LOW);
    }
    else if(temperature >= baselineTemp+4 && temperature < baselineTemp+6){
    digitalWrite(0, HIGH);
    digitalWrite(1, HIGH);
    digitalWrite(2, LOW);
    }
    else if(temperature >= baselineTemp+6) {
    digitalWrite(0, HIGH);
    digitalWrite(1, HIGH);
    digitalWrite(2, HIGH);
    }
    delay(1000);

}