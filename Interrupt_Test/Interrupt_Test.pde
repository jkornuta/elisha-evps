/*
 * Timer Interrupt Tester
 *
 * Created by Phillip Johnston
 * 8 June 2012
 * 
 * Modified by Jeff Kornuta
 * 26 August 2012
 *
 * This program will configure a timer interrupt 
 * and blink the LED (pin 13).
 *
 * Output will be to serial terminal.
 */

#if defined(__PIC32MX__)
#include <p32xxxx.h>    /* this gives all the CPU/hardware definitions */
#include <plib.h>       /* this gives the i/o definitions */
#endif

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/********************
 * Define Statements *
 ********************/
#define CLOCK_FREQ 80000000
#define TIMER_PRESCALE 4
#define PRESCALED_TIMER_FREQ 20000000
#define PI 3.14159265

// Turn on debugging messages?
//#define SYS_IDENT_DEBUG
#ifdef SYS_IDENT_DEBUG
#define debugf(msg) Serial.print msg
#else
#define debugf(msg) 
#endif

/************************
 * Function Declarations *
 ************************/
void configureTimer45();
void printTMRVals();

/************************
 * Variable Declarations *
 ************************/
//Servo servo (9, 8, DAC_A); //QD cs = 9, Dac cs = 8
bool apply_value = false;
bool pin_state = false;
double delta_t;
uint8_t pinOut = 13;  // Digital pin to ouput pulse
//float Fs = 5.0;  // Sampling frequency, Hz
double Ts = 0.5; // Sampling time, sec
double Fs = 1.0 / Ts; // Sampling frequency, Hz

double preFreq = 20000000.0;

/***********************
 * Function Definitions *
 ***********************/

void setup() 
{
  // Begin serial communication.
  Serial.begin(115200);

  //servo.init(); //Set up SPI and configure QD and DAC
  pinMode(pinOut, OUTPUT);

  // Configure timer for desired sampling frequency
  configureTimer45(Fs);

  Serial.println("====Begining System Test====");
}

void loop()
{
  // If interrupt was triggered, do something!
  if (apply_value)
  {
    // Toggle pin state
    if (pin_state)
    {
      pin_state = false;
      digitalWrite(pinOut, LOW);
    }
    else
    {
      pin_state = true;   
      digitalWrite(pinOut, HIGH);
    }

    // OK, interrupt has been serviced. Carry on!
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

  debugf(("The new timer period is: "));
  debugf((t_period));
  debugf(("\n"));

  //T4CONSET = 0x18; //Prescaler 1:2, internal peripheral source
  T4CONSET = 0x28; //Prescaler 1:4, internal source

  TMR4 = 0;
  PR4 = t_period;
  IPC5SET = 0x5; //Priority level 1, sub-priority level 1
  IFS0CLR = 0x00100000;
  IEC0SET = 0x00100000;

  printTMRVals();

  T4CONSET = 0x8000; // Start timer 45

}

#ifdef __cplusplus
extern "C" {
#endif
  void __ISR(_TIMER_5_VECTOR,IPL3AUTO) write_handler( void )
  {
    apply_value = true;
    debugf(("Interrupt Entered\n"));
    IFS0CLR = 0x00100000;  // Clear interrupt flag
    printTMRVals();
  }

#ifdef __cplusplus
}
#endif


void printTMRVals()
{
  debugf(("TMR5: "));
  debugf((TMR5));
  debugf(("\nTMR4: "));
  debugf((TMR4));
  debugf(("\n")); 
}

