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

#include <Wire.h>
#include "HSC.h"
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
Servo servo2 (9, 8, DAC_B);

HSC pSensor1 (1);   // Pressure sensor #1
HSC pSensor2 (2);   // Pressure sensor #2

uint32_t data_length;    // Length of input data
//float *input_data = new float[data_length]; // Input data array
uint32_t data_counter = 0;
char buffer [80]; // String buffer to serially print back to screen
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
  // Start serial and I2C communication
  //Serial.begin(115200);
  // http://www.chipkit.org/forum/viewtopic.php?f=19&t=711
  Serial.begin(921600);
  Wire.begin();
  
  // Wait for devices to warm up
  delay(100);

  // Set up SPI and configure QD and DAC  
  servo1.init();
  servo2.init();
        
  // Initial output voltage, 0 V
  servo1.move(0.0);
  servo2.move(0.0);
  
  // Wait until specified time to start
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
        data_length = readULongFromBytes(); 

        // Print to verify data integrity
        Serial.println( data_length,DEC );         
      }

      // Did user type "g" for GO? 
      //  If so -- zero out counter and carry on!
      if ( in == 'g' ) 
      { 
        servo1.zero();
        servo2.zero();
        wait = false;
        delay(3000); // Pause 3 sec before beginning 
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
    // Time how long this all takes to execute
    uint32_t time = micros();

    // If completed sending data, stop
    if ( data_counter == data_length - 1 )
    {
      servo1.move(0.0);
      servo2.move(0.0);
      Serial.println( "Done!" );
      while(1){};
    }

    // Take in desired input floats from serial buffer
    float r1 = readFloatFromBytes();
    //float r2 = 0;
    float r2 = readFloatFromBytes(); 

    // Grab the motor positions
    float x1 = servo1.position();
    float x2 = servo2.position();

    // Grab pressure values
    float p1 = pSensor1.pressure();
    float p2 = pSensor2.pressure();

    // Apply control input(s) 
    //float u1 = 0.1*r1;
    //float u2 = 0.1*r2;
    servo1.move(r1);
    servo2.move(r2);
    
    // Create string buffer to print: 
    //  [ u1 u2  x1 x2  p1 p2  time ]
    sprintf( buffer, "%.3f %.3f  %.3f %.3f  %.3f %.3f  ",
      r1, r2, x1, x2, p1, p2 );
    Serial.print(buffer);
    /*  
    // Print out stuff: [ u1 u2  x1 x2  p1 p2  time ]    
    Serial.print(r1); Serial.print(" "); Serial.print(r2); Serial.print("   ");
    Serial.print(x1); Serial.print(" "); Serial.print(x2); Serial.print("   ");
    Serial.print(p1); Serial.print(" "); Serial.print(p2); Serial.print("   ");
    */
    // How long did these actions take?
    time = micros() - time;
    Serial.println(time);

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
  serialWaitBytes(4);
  for ( uint8_t i = 0; i < 4; i++)
  {
    u.b[i] = Serial.read();
  }

  // Return unsigned long
  return u.ulval;
}

// Function to read in 32-bit float from bytes
float readFloatFromBytes()
{
  // Use union for C++ magic
  union u_tag
  {
    uint8_t b[4];
    float fval;
  } u;

  // Read in bytes
  serialWaitBytes(4);
  for ( uint8_t i = 0; i < 4; i++)
  {
    u.b[i] = Serial.read();
  }

  // Return float
  return u.fval;
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
