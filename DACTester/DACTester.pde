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
    servo.move(-9.95);  
}

void loop()
{

}
