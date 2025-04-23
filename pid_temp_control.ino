#include "DHT.h"
#include <PID_v1.h>


#define DHTPIN 5
#define DHTTYPE DHT22  


DHT dht = DHT(DHTPIN, DHTTYPE);


// Pin Definitions
const int heaterPin = 9;   // Digital pin connected to the heater relay (or MOSFET)
const int coolerPin = 10;  // Digital pin connected to the cooler relay (or MOSFET)


// PID Parameters
double Kp = 1.8, Ki = 5.0, Kd = 0.10;  // Tune these constants based on your system
double setpoint = 27.0;  // Desired temperature (°C)
double input, output;    // Variables for PID controller
double heaterOutput, coolerOutput; // Variables for actuators

// Control variables
double maxOutput = 255;
double delta = 1.0;
double lowerLimit = 0, upperLimit = 30;

// Create PID instance
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  Serial.begin(9600);

  pinMode(heaterPin, OUTPUT);
  digitalWrite(heaterPin, LOW);  // Ensure heater is off initially

  dht.begin(); // Initialize DHT22 sensor

  // Initialize PID
  myPID.SetMode(AUTOMATIC);   // Turn on the PID controller
  myPID.SetOutputLimits(0, maxOutput);  // Output range for the PWM signal (0-255)
}


void loop() {
 
  input = measureTemperature();

  // Ensure temperature is within operational limits. Override PID if necessary
  while (input > upperLimit) {
    analogWrite(coolerPin, maxOutput);
    Serial.println("EMERGENCY: Dangerously hot. Systems compromised.");
    delay(1000);
  }

  while (input < lowerLimit) {
    analogWrite(heaterPin, maxOutput);
    Serial.println("EMERGENCY: Dangerously cold. Systems compromised.");
    delay(1000);
  }
 
 
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
  Serial.print(heaterOutput);
  Serial.print(", Cooler Output: ");
  Serial.println(coolerOutput);

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

double controlHeater(double output){
  if ((output >= 128) && (input < (setpoint - delta))) {
    return (output - 128) * 2;
  }
  return 0;
}

double controlCooler(double output){
  if ((output < 128) && (input > (setpoint + delta))) {
    return (128 - output) * 2;
  }
  return 0;
}







