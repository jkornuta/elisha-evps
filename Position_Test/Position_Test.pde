/*
 * Position (QD) test code 
 *  Test QD while also outputting voltage.
 *   JK 10/1/2012
 *
 */

#if defined(__PIC32MX__)
#include <p32xxxx.h>    /* this gives all the CPU/hardware definitions */
#include <plib.h>       /* this gives the i/o definitions */
#endif


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

#define CLOCK_FREQ 80000000
#define TIMER_PRESCALE 4
#define PRESCALED_TIMER_FREQ 20000000
#define PI 3.14159265

/************************
* Variable Declarations *
************************/
Servo servo1 (9, 8, DAC_A); // QD cs = 9, DAC cs = 8, DAC A
//Servo servo2 (10, 8, DAC_B);
bool apply_value = false;
bool pin_state = false;
double delta_t;
double Ts = 0.25; // Sampling time, sec
double Fs = 1.0 / Ts; // Sampling frequency, Hz

/************************
 * Function Declarations *
 ************************/
void configureTimer45();

/***********************
 * Function Definitions *
 ***********************/

void setup() 
{
  Serial.begin(115200);
    
  servo1.init(); // Set up SPI and configure QD and DAC
  //servo2.init();
        
  Serial.println("Begin System Test");

  // Initial output voltage, 0 V
  //servo1.move(5.0);
  //servo2.move(-5.0);

  // Pause initially, then continue
  delay(2000);  // delay in ms

  // Configure timer for desired sampling frequency
  configureTimer45(Fs);

}

void loop()
{

  // If interrupt was triggered, do something!
  if (apply_value)
  {
    // grab those positions
    double x1 = servo1.position();
    //double x2 = servo2.position();
    
    Serial.println( x1 );
    //Serial.print( "     " );
    //Serial.println( x2 );

    // turn off apply_value trigger
    apply_value = false;
  }

}

void configureTimer45(double freq)
{
  uint32_t t_period;  //For the PIC32 PR4 register

  T4CON = 0x0;
  T5CON = 0x0;

  // Using the desired frequency, clock frequency,
  //  and number of vals per cycle, we can get the
  //  Timer period.
  t_period = (uint32_t) ((double) PRESCALED_TIMER_FREQ / freq );
  delta_t = (double) t_period * (1.0 / (double) PRESCALED_TIMER_FREQ);
  
  //T4CONSET = 0x18; //Prescaler 1:2, internal peripheral source
  T4CONSET = 0x28; //Prescaler 1:4, internal source

  TMR4 = 0;
  PR4 = t_period;
  IPC5SET = 0x5; //Priority level 1, sub-priority level 1
  IFS0CLR = 0x00100000;
  IEC0SET = 0x00100000;

  T4CONSET = 0x8000; // Start timer 45

}

#ifdef __cplusplus
extern "C" {
#endif
  void __ISR(_TIMER_5_VECTOR,IPL3AUTO) write_handler( void )
  {
    apply_value = true;
    IFS0CLR = 0x00100000;  // Clear interrupt flag
  }

#ifdef __cplusplus
}
#endif
