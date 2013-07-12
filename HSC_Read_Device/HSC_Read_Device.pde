/*
  Return pressure if chipKIT receives a serial byte
  JK 7/13/12
*/

#include <Wire.h>
#include "HSC.h"

HSC pSensor1 (1);
HSC pSensor2 (2);
double pressure;
char buffer[80] = "";

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
  //Serial.print( pSensor1.pressure() );
  //Serial.print( "\t" );
  //Serial.println (pSensor2.pressure() );
  float a = 1.1;
  int8_t b = 2;
  int8_t c = 3;
  //uint16_t bufsize = sprintf( buffer, 
  //                      "%.2f %.2f  %.3f %.3f  %.2f %.2f  %d",
  //                        a, b, c, 4.4, 5.5, 6.6, 100 );
  uint16_t bufsize = sprintf(buffer,"%e\n", a);    
  //Serial.print( bufsize,DEC );
  //Serial.print( "\t" );
  Serial.print(buffer);

  delay(200);
}
