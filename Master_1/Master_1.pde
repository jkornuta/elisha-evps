/*
 *  MASTER CONTROL PROGRAM
 *  Debug, System Identification, or Controlling EVPS using PID/MPC.
 *    JK 4/5/2013
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
#include "Controller.h" // for special controller methods

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

// For filtering
float p1_history[max_samples] = {0};// Initialize pressure histories
float p2_history[max_samples] = {0};
uint8_t filter_order = 12;
// For PID (if using)
float Kp;  // P-gain for Pavg
float Ki;  // I-gain for Pavg
float p1_error_sum = 0.0;// Sum of error, p1 
float p2_error_sum = 0.0;// Sum of error, p2
// For MPC (if using)
float xhat[n][1] = {0}; // Initialize state estimate, xhat (xhat_0)
float Ydk[Hp*p][1] = {0}; // Initialize input (desired output) vector
float y[p][1] = {0};  // Initialize output vector
float u[m][1] = {0};  // Initialize input vector
// For solenoid switching
bool xend1 = false; // If motor 1 reaches end
bool xend2 = false; // If motor 2 reaches end
bool xmid1 = true;  // If motor 1 crosses midway
bool xmid2 = true;  // If motor 2 crosses midway
bool switched = false; // Solenoid switch state
uint8_t solenoid = 2; // Solenoid switching pin
float x1_old = 0;
float x2_old = 0;
// Other variables
uint8_t mode; // Program mode: 0 - debug, 1 - identification, 2 - control
uint32_t data_length = 0;    // Length of input data
uint32_t data_counter = 0;
char buffer [80]; // String buffer to serially print back to screen
bool apply_value = false;
bool pin_state = false;
double delta_t;
//double Ts = 0.005;      // Sampling time, sec
//double Fs = 1.0 / Ts;   // Sampling frequency, Hz
double Fs = 300.0;
double Ts = 1.0 / Fs;


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
  // http://www.chipkit.org/forum/viewtopic.php?f=19&t=711
  Serial.begin(921600);
  Wire.begin();

  // Set solenoid pin as output (start out LOW)
  pinMode(solenoid, OUTPUT);
  digitalWrite(solenoid, switched);
  
  // Wait for devices to warm up
  delay(100);

  // Set up SPI and configure QD and DAC  
  servo1.init();
  servo2.init();
        
  // Initial output voltage, 0 V
  servo1.move(0.0);
  servo2.move(0.0);
  
  // Wait until mode specified, then begin
  bool wait = true;
  while ( wait )
  {
    if ( Serial.available() > 0 )
    {
      // Read in byte
      char in = Serial.read();

      // Was 'd' received? (i.e. for debug)
      if ( in == 'd' )
      {
        // Specify mode 0
        mode = 0;

        // Exit out of wait loop
        wait = false;
      }

      // Was 'i' received? (i.e. for identification)
      if ( in == 'i' ) 
      { 
        // Specify mode 1
        mode = 1;

        // Great, read in length of input data
        data_length = readULongFromBytes(); 

        // Print to verify data integrity
        Serial.println( data_length,DEC );  

        // Exit out of wait loop
        wait = false;      
      }

      // Was 'c' received? (i.e. for control)
      if ( in == 'c' ) 
      { 
        // Specify mode 2
        mode = 2;

        // Great, read in length of input data
        data_length = readULongFromBytes(); 

        // Print to verify data integrity
        Serial.println( data_length,DEC );
        
        /* For PID
        // Read in control gains
        Kp = readFloatFromBytes();
        Ki = readFloatFromBytes();
        */

        // For MPC
        // Fill up initial Ydk vector
        for (uint8_t i = 0; i < Hp*p; i = i+p)
        {
          // Read from serial buffer
          Ydk[i][0] = readFloatFromBytes();
          Ydk[i+1][0] = readFloatFromBytes();
        }
 
        // Estimate current xhat (from current output) as initial state!
        float p1 = pSensor1.pressure();
        float p2 = pSensor2.pressure();
        float y1 = p1-p2;
        float y2 = (p1+p2)/2.0;
        y[0][0] = y1;
        y[1][0] = y2;
        control.stateInit(y, xhat);

        // Use current pressure to fill up pressure history...
        // ...will make the filtering start buttery-smooth
        for (uint8_t i = 0; i <= filter_order; i++)
        {
          p1_history[i] = p1;
          p2_history[i] = p2;
        }
        
        // Exit out of wait loop
        wait = false;
      }
    }
  }

  // Zero position counters and pause to wait for serial buffer
  //  to be filled, if need be
  servo1.zero();
  servo2.zero();
  delay(3000); // Pause 3 sec

  // Configure timer for desired sampling frequency
  configureTimer45(Fs);

}

// Loops continuously
void loop()
{
  // If interrupt was triggered, do something!
  if (apply_value)
  {
    // If completed sending data, stop (for ID or control)
    if ( data_counter == data_length-1 && mode != 0 )
    {
      servo1.move(0.0);
      servo2.move(0.0);
      digitalWrite(solenoid, LOW);
      Serial.println( "Done!" );
      while(1){};
    }

    // Take in next desired input floats from serial buffer
    float r1 = readFloatFromBytes();
    float r2 = readFloatFromBytes();
 
    // Grab the motor positions; stop if out of range (+- xlimit mm)
    float x1 = servo1.position();
    float x2 = servo2.position();
    float xlimit = 30.0;
    if (abs(x1) > xlimit || abs(x2) > xlimit)
    {
      // Stop motors; end program
      servo1.move(0.0);
      servo2.move(0.0);
      digitalWrite(solenoid, LOW);
      Serial.println( "Motor(s) out of range!" );
      while(1){};
    }

    // Determine conditions for solenoid switching
    // End location at which solenoid should switch
    float xswitch = 15.0;
    // Logic portion: set proper boolean variables
    // If motor 1 reaches end
    if (abs(x1) > xswitch)
    {
      xend1 = true;
    }
    else
    {
      xend1 = false;
    }
    // If motor 2 reaches end
    if (abs(x2) > xswitch)
    {
      xend2 = true;
    }
    else
    {
      xend2 = false;
    }
    // If motor 1 crosses midway (changes direction)
    if (x1*x1_old < 0)
    {
      xmid1 = true;
    }
    // If motor 2 crosses midway
    if (x2*x2_old < 0)
    {
      xmid2 = true;
    }
    // Store old position values
    x1_old = x1;
    x2_old = x2;

    // One more failsafe...
    if (abs(x1) > 20.0 || abs(x2) > 20.0)
    {
      // Trick into thinking it's crossed midway
      xmid1 = true;
      xmid2 = true;
    }
 
    // Grab pressure values; stop if pressure out of range (+- plimit cmH2O)
    float p1 = pSensor1.pressure();
    float p2 = pSensor2.pressure();
    float plimit = 60.0;
    if (abs(p1) > plimit || abs(p2) > plimit)
    {
      // Stop motors; end program
      servo1.move(0.0);
      servo2.move(0.0);
      digitalWrite(solenoid, LOW);
      Serial.println( "Pressure(s) out of range!" );
      while(1){};
    }

    // If mode 0 or 1, apply desired inputs, print serial buffer, continue
    if ( mode == 0 || mode == 1 )
    {
      // Apply inputs
      servo1.move(r1);
      servo2.move(r2);

      // Print serial buffer: [ r1 r2  x1 x2  p1 p2  time/dummy ]
      sprintf( buffer, "%.3f %.3f  %.3f %.3f  %.3f %.3f  %d",
        r1, r2, x1, x2, p1, p2, 0 );
      Serial.println(buffer);
    }

    // If mode 2, do control, print serial buffer, continue
    if ( mode == 2 )
    {
      // Moving average filter on pressure outputs
      p1 = control.filter(p1, p1_history, filter_order);
      p2 = control.filter(p2, p2_history, filter_order);
      float y1 = p1-p2;
      float y2 = (p1+p2)/2.0;
 
      // If conditions are right -- switch solenoid!
      if ((xend1 && xmid1) || (xend2 && xmid2))
      {
        // Change switched state
        switched = !switched;
        digitalWrite(solenoid, switched);

        // Turn off midway booleans
        xmid1 = false;
        xmid2 = false;
      }
      
      /* For PID    
      // Calculate control inputs to system
      float p1_error = r1-y1;
      p1_error_sum += p1_error;
      float u1 = Kp*p1_error + Ki*Ts*p1_error_sum;
      
      float p2_error = r2-y2;
      p2_error_sum += p2_error;
      float u2 = Kp*p2_error + Ki*Ts*p2_error_sum;
      */

      // For MPC
      // Make updated desired input vector, Ydk
      for (uint8_t i = 0; i < (Hp-1)*p; i = i+p)
      {
        // Shift everything upwards
        Ydk[i][0] = Ydk[i+p][0];
        Ydk[i+1][0] = Ydk[i+p+1][0];
      }
      Ydk[Hp*p-2][0] = r1;  // Fill in last spots with most recent serial grab
      Ydk[Hp*p-1][0] = r2;
      // Calculate control inputs to system via MPC
      control.mpc(Ydk, xhat, u);
      float u1 = u[0][0];
      float u2 = u[1][0];
       
      // Apply control inputs 
      //  (take solenoid direction into account with conditional)
      if (switched == false)
      {
        servo1.move( u1 );
        servo2.move( u2 );
      }
      else
      {
        servo1.move( u2 );
        servo2.move( u1 );
      }

      // For MPC
      // Estimate next state vector using estimator; update desired outputs
      y[0][0] = y1;
      y[1][0] = y2;
      control.stateEstim(u, y, xhat);
      r1 = Ydk[0][0];
      r2 = Ydk[1][0];

      // Print out stuff: [ r1 r2  y1 y2  u1 u2  x1 x2  p1 p2  switched ] 
      sprintf(buffer, 
        "%.3f %.3f  %.3f %.3f  %.3f %.3f  %.3f %.3f  %.3f %.3f  %d", 
        r1, r2, y1, y2, u1, u2, x1, x2, p1, p2, switched);
      Serial.println(buffer);
    }
    
    // Turn off apply_value (interrupt) trigger, increment counter
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
