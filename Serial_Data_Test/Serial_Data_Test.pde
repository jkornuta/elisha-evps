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
Servo servo1 (10, 8, DAC_A); // QD cs = 10, DAC cs = 8, DAC A
//Servo servo2 (9, 8, DAC_B);

uint32_t data_length;    // Length of input data
int8_t *input_data = new int8_t[data_length]; // Input data array
uint32_t data_counter = 0;
bool apply_value = false;
bool pin_state = false;
double delta_t;
double Ts = 0.005;      // Sampling time, sec
double Fs = 1.0 / Ts;   // Sampling frequency, Hz

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
  Serial.begin(115200);
  
  // Wait for devices to warm up
  delay(100);

  // Set up SPI and configure QD and DAC  
  servo1.init();
  //servo2.init();
        
  // Initial output voltage, 0 V
  servo1.move(0.0);
  
  // Wait until $ and input data are received
  bool wait = true;
  while ( wait )
  {
    if ( Serial.available() > 0 )
    {
      // Read in byte
      char in = Serial.read();

      // Was $ received? (i.e. about to send data?)
      if ( in == '$' ) 
      { 
        // Great, read in length of input data
        serialWaitBytes(4);
        data_length = readULongFromBytes(); 

        // Grab rest of input data
        for ( uint32_t i = 0; i < data_length; i++ )
        {
          serialWaitBytes(1);
          input_data[i] = Serial.read();
        }

        // Print to verify data integrity 
        Serial.println( data_length,DEC );
        
        // Exit out of waiting loop
        //wait = false;
         
      }

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

// Loops continuously
void loop()
{
  // If interrupt was triggered, do something!
  if (apply_value)
  {
    if (data_counter == data_length - 1)
    {
      servo1.move(0.0);
      Serial.print('$$$\n');
      while (1) {};
    }

    // Grab those positions
    double x1 = servo1.position();
    //double x2 = servo2.position();

    // Apply control input(s) 
    double u1 = (double) input_data[data_counter];
    servo1.move(u1);

    // Print output position
    Serial.print( x1 );
    Serial.print( "\t" );
    //Serial.println( x2 );

    // Print output voltage
    Serial.println( u1 );
    
    // Turn off apply_value (interrupt) trigger, iterate counter
    apply_value = false;
    data_counter++;
  }

}

// Function to wait until number of bytes are in buffer
void serialWaitBytes(uint16_t num_bytes)
{
  while ( !(Serial.available() >= num_bytes) );
}

// Function to read in uint32_t from bytes
uint32_t readULongFromBytes()
{
  // Use union for C++ magic
  union u_tag
  {
    uint8_t b[4];
    uint32_t ulval;
  } u;

  // Read in bytes
  for ( uint8_t i = 0; i < 4; i++)
  {
    u.b[i] = Serial.read();
    //Serial.println( u.b[i], DEC );
  }

  // Return unsigned long
  return u.ulval;
}

// Configure timer interrupt
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
