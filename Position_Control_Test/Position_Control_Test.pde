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
float Kp = -8.4e-3;   // Proportional gain
float Ki = -0.003032;  // Integral gain
float Kd = -5.01e-6;  // Derivative gain

/************************
* Variable Declarations *
************************/
Servo servo1 (10, 8, DAC_A); // QD cs = 10, DAC cs = 8, DAC A
//Servo servo2 (9, 8, DAC_B);

bool apply_value = false;
bool pin_state = false;
double delta_t;
double Ts = 0.005;      // Sampling time, sec
double Fs = 1.0 / Ts;   // Sampling frequency, Hz

// Control variables
double e1sum = 0.0; // sum of velocity error
double e1old = 0.0; // old velocity error
double v1old = 0.0; // old velocity
double x1old = 0.0; // old position
double v1d = 10.0; // desired position

/************************
 * Function Declarations *
 ************************/
void configureTimer45();

/***********************
 * Function Definitions *
 ***********************/

void setup() 
{
  // Start serial and SPI communication
  Serial.begin(921600);
  
  // Wait for devices to warm up
  delay(100);

  // Set up SPI and configure QD and DAC  
  servo1.init(); 
  //servo2.init();
        
  //Serial.println(" > Waiting for user (Type 'g' for GO)...");

  // Initial output voltage, 0 V
  servo1.move(0.0);
  //servo2.move(0.0);

  // Pause initially, then continue
  //delay(5000);  // delay in ms

  // Wait until user says go! (types "g")
  bool wait = true;
  while ( wait )
  {
    while ( Serial.available() > 0 )
    {
      // Read in byte to remove from buffer
      char in = Serial.read();
      
      // Did user type "g" for GO? 
      //  If so -- zero out counter and carry on!
      if ( in == 'g' ) 
      { 
        servo1.zero();
        wait = false; 
      }
    }
  }

  // Configure timer for desired sampling frequency
  configureTimer45(Fs);

}

void loop()
{

  // If interrupt was triggered, do something!
  if (apply_value)
  {
    // Grab those positions
    float x1 = servo1.position();
    //double x2 = servo2.position();

    // Calculate velocity, error, etc
    //double v1 = (x1-x1old)/Ts;
    //double e1 = v1d-v1;     // error
    //double e1d = e1-e1old;  // change in error
    //e1sum += e1;            // sum of error
    float e1 = 0.0-x1;
    float u1 = Kp*e1;

    // for next iteration
    //x1old = x1;
    //e1old = e1;

    //double u1 = KP*e1 + KI*e1sum;
    //double u1 = KP*e1;  
    //double u1 = Kp + Ki*Ts/e1sum + Kd*e1d/Ts;  

    // Apply control input(s) 
    servo1.move(u1);

    // Print output position
    //Serial.print( 0.0 );
    //Serial.print( "\t" );
    Serial.print( e1 );
    Serial.print( "\t" );
    //Serial.println( x2 );

    // Print output voltage
    Serial.println( u1 );

    // Turn off apply_value (interrupt) trigger
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
