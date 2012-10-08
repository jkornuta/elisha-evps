/*
  Return pressure if chipKIT receives a serial byte
  JK 7/13/12
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
  Serial.println("Waiting to return a pressure...\n");  
  delay(100);  
}

// Test pressure sensor
void loop()
{
  /*
  // If received a byte, return a pressure!
  while ( Serial.available() > 0 )
  {
    // Read in byte to remove from buffer
    char in = Serial.read();
    
    // Return a pressure
    pressure = pSensor1.pressure();
    Serial.print(pressure);
    Serial.print('\n');
  }
  */
  Serial.println( pSensor1.pressure() );

  delay(200);
}
