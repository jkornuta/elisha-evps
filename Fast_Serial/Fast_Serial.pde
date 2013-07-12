// Test fast serial - 3/7/13

#include <stdint.h>

void setup()
{
  //Serial.begin(115200);
  Serial.begin(921600);
  float x = 123.45;
  uint32_t time = micros();
  char buffer[80];
    sprintf(buffer, 
      "%.2f %.2f  %.2f %.2f  %.2f %.2f  %.2f %.2f  %.2f %.2f", 
      x, x, x, x, x, x, x, x, x, x);
    Serial.println(buffer);
  time = micros() - time;
  Serial.println(time); 

}

void loop()
{

}
