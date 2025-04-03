#include "DHT.h"
#include <PID_v1.h>


#define DHTPIN 5
#define DHTTYPE DHT22  


DHT dht = DHT(DHTPIN, DHTTYPE);


// Pin Definitions
const int heaterPin = 9;   // Digital pin connected to the heater relay (or MOSFET)
const int coolerPin = 10;  // Digital pin connected to the cooler relay (or MOSFET)


// PID Parameters
double Kp = 1.0, Ki = 5.0, Kd = 0.10;  // Tune these constants based on your system
double setpoint = 26.0;  // Desired temperature (°C)
double input, output;    // Variables for PID controller
double heaterOutput, coolerOutput; // Variables for actuators

// Create PID instance
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);

  pinMode(heaterPin, OUTPUT);
  digitalWrite(heaterPin, LOW);  // Ensure heater is off initially

  dht.begin(); // Initialize DHT22 sensor

  // Initialize PID
  myPID.SetMode(AUTOMATIC);   // Turn on the PID controller
  myPID.SetOutputLimits(0, 255);  // Output range for the PWM signal (0-255)
}


void loop() {
 
  input = measureTemperature();
 
 
  // Compute PID output
  myPID.Compute();


  // Control the heater and cooler based on PID output (use PWM for finer control)
  heaterOutput = controlHeater(output);
  coolerOutput = controlCooler(output);
  analogWrite(heaterPin, heaterOutput);
  analogWrite(coolerPin, coolerOutput);

  // Print temperature and control output to serial monitor for debugging
  Serial.print("Temperature: ");
  Serial.print(input);
  Serial.print(" °C, Heater Output: ");
  Serial.println(output);

  delay(1000);  // Delay to allow the temperature to stabilize before the next reading
}


// FUNCIONES
double measureTemperature(){
  float temperature = dht.readTemperature();
  if (isnan(temperature)) {
    Serial.println(F("Error en leer temperatura de DHT sensor!"));
    Serial.println(temperature,"\n");
    return;
  }
  Serial.println(temperature,"\n");
  double measurement = (temperature);
  return measurement;
}

double controlCooler(output){
  if (output >= 128) {
    return (output - 128) * 2;
  }
  return 0;
}

double controlHeater(output){
  if (output < 128) {
    return output * 2;
  }
  return 0;
}

