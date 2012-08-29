/*
  Play code for the HSC pressure sensor (without multiplexer)
  JK 6/8/12
*/

#include <Wire.h>
#include "HSC.h"

HSC pSensor1 (0);
double pressure;

// Start main program
void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Reading pressure values:\n");  
  delay(100);  
}

// Test pressure sensor
void loop()
{
  pressure = pSensor1.pressure();
  Serial.print(pressure);
  Serial.println(" cmH2O");
  
  delay(250);
}
