/*
 * Output voltage tester
 * Test voltage output for DAC A
 *   JK 8/30/2012
 *
 */

#include "Servo.h"
#include "SPI.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/********************
* Define Statements *
********************/

#define DAC_A  0
#define DAC_B  2

#define SYS_DEBUG

#ifdef SYS_DEBUG
#define debugf(msg) Serial.print msg
#else
#define debugf(msg) 
#endif

/************************
* Variable Declarations *
************************/
Servo servo (9, 8, DAC_A); // QD cs = 9, DAC cs = 8, DAC A

/***********************
* Function Definitions *
***********************/

void setup() 
{
    Serial.begin(115200);
    
    servo.init(); // Set up SPI and configure QD and DAC
        
    Serial.println("Begin System Test");

    // Output voltage
    servo.move(5.0);  // Apply 5.0 V
}

void loop()
{

}
