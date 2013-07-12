/*
 *  MPC Control Program 
 *  Test controlling EVPS using MPC algorithm.
 *    JK 3/4/2013
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
#include "Controller.h"

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
// Servo objects
Servo servo1 (10, 8, DAC_A); // QD cs = 10, DAC cs = 8, DAC A
Servo servo2 (9, 8, DAC_B);

// Pressure sensor objects
HSC pSensor1 (1);   // Pressure sensor #1
HSC pSensor2 (2);   // Pressure sensor #2

// Controller object
Controller control;

// Other variables
uint32_t data_length;    // Length of input data
uint32_t data_counter = 0;
float xhat[n][1] = {0}; // Initialize state estimate, xhat (xhat_0)
float Ydk[Hp*p][1] = {0}; // Initialize input (desired output) vector
float y[p][1] = {0};  // Initialize output vector
float x1_old = 0.0;   // Initialize initial servo1 position as zero
float u[m][1] = {0};  // Initialize input vector
bool apply_value = false;
bool pin_state = false;
double delta_t;
double Ts = 0.005;      // Sampling time, sec
double Fs = 1.0 / Ts;   // Sampling frequency, Hz

/************************
 * Function Declarations *
 ************************/
void configureTimer45();
void serialWaitBytes();
uint32_t readULongFromBytes();
float readFloatFromBytes();

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
        // Zero position counters and pause to wait for serial buffer
        //  to be filled with future desired inputs
        servo1.zero();
        servo2.zero();
        delay(3000); // Pause 3 sec
        
        // Fill up initial Ydk vector
        for (uint8_t i = 0; i < Hp*p; i = i+p)
        {
          // Read from serial buffer
          Ydk[i][0] = readFloatFromBytes();
          Ydk[i+1][0] = readFloatFromBytes();
        }
        
        // Exit out of wait loop
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
    // Time how long this all takes to execute
    uint32_t time = micros();

    // If completed sending data, stop (OR is for DEBUGGING)
    if ( data_counter == data_length-1 )
    {
      servo1.move(0.0);
      servo2.move(0.0);
      Serial.println( "Done!" );
      while(1){};
    }

    // Take in next desired input floats from serial buffer
    float r1 = readFloatFromBytes();
    float r2 = readFloatFromBytes();

    /*
    // ######## DEBUG: Print {r1, r2} and Ydk vector
    Serial.print("  r1 = ");
    Serial.println(r1);
    Serial.print("  r2 = ");
    Serial.println(r2);
    Serial.print("  Ydk =");
    for (uint8_t i = 0; i < Hp*p; i++)
    {
      Serial.print(" ");
      Serial.print(Ydk[i][0]);
    }
    Serial.println(" ");

    // ########
    */

    // Make updated desired input vector, Ydk
    for (uint8_t i = 0; i < (Hp-1)*p; i = i+p)
    {
      // Shift everything upwards
      Ydk[i][0] = Ydk[i+p][0];
      Ydk[i+1][0] = Ydk[i+p+1][0];
    }
    Ydk[Hp*p-2][0] = r1;  // Fill in last spots with most recent serial grab
    Ydk[Hp*p-1][0] = r2;

    // Grab the motor positions; stop if out of range (+- limit mm)
    float x1 = servo1.position();
    float x2 = servo2.position();
    float limit = 32.0;
    if (abs(x1) > limit || abs(x2) > limit)
    {
      // Stop motors; end program
      servo1.move(0.0);
      servo2.move(0.0);
      Serial.println( "Motor(s) out of range!" );
      while(1){};
    }

    // Grab pressure values
    float p1 = pSensor1.pressure();
    float p2 = pSensor2.pressure();

    // Form current output vector
    y[0][0] = (x1-x1_old)/Ts;
    y[1][0] = (p1+p2)/2.0;
    //float y[p][1] = { (x1-x1_old)/Ts, (p1+p2)/2.0 };
    x1_old = x1;  // For next iteration
    
    // Calculate control inputs to system via MPC
    control.mpc(Ydk, xhat, u);

    // Apply control inputs
    u[0][0] = 1.0*u[0][0];
    u[1][0] = 1.0*u[1][0];
    servo1.move( u[0][0] );
    servo2.move( u[1][0] );

    // Estimate next state vector using estimator
    control.stateEstim(u, y, xhat);
            
    // How long did these actions take?
    //time = micros() - time;
    /*
    // DEBUG: Print out new xhat each time
    char bufd[60];
    sprintf(bufd, "  xhat_new = %.2f %.2f %.2f", xhat[0][0], xhat[1][0], xhat[2][0]);
    Serial.println(bufd);  
    */
    // Print out stuff: [ Ydk0 Ydk1  y1 y2  u1 u2  x1 x2  p1 p2  time ] 
    char buffer[80];
    sprintf(buffer, 
      "%.2f %.2f  %.2f %.2f  %.2f %.2f  %.2f %.2f  %.2f %.2f", 
      Ydk[0][0], Ydk[1][0], y[0][0], y[1][0], u[0][0], u[1][0],
      x1, x2, p1, p2);
    Serial.println(buffer);
    /*    
    Serial.print(Ydk[0][0]); Serial.print(" "); Serial.print(Ydk[1][0]); Serial.print("   ");
    Serial.print(y[0][0]); Serial.print(" "); Serial.print(y[1][0]); Serial.print("   ");
    Serial.print(u[0][0]); Serial.print(" "); Serial.print(u[1][0]); Serial.print("   ");
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
