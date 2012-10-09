/*
 * Servo.cpp - Library for interacting with a
 *          Parker MX80L linear stage
 * 
 * Created by Jeff Kornuta, September 18, 2011.
 * Modified by Phillip Johnston, 22 May 2012
 * Released into the public domain.
 *
*/

#include "WProgram.h"
#include "Servo.h"
#include "LS7366R.h"
#include "HardwareSerial.h"
#include "SPI.h"
#include "dac5752.h"
#include "math.h"

/**********************
* Object Declarations *
**********************/
DACClass DAC;
LS7366RClass QD;

/****************************
* Configuration Definitions *
****************************/
// configuration vars for the LS7366R
#define MDR0_CONFIG 3    // 0 0 00 00 11
#define MDR1_CONFIG 0    // 00000000


/***********************
* Function Definitions *
***********************/
Servo::Servo(uint8_t quadDecCS, uint8_t dacCS, uint8_t dac)
{   
  // Set private vars so (private) methods can initialize properly 
  _quadDecCS = quadDecCS;
  _dacCS = dacCS;
  selected_dac = dac; 
}

// read position of motor (via LS7366R)
double Servo::position(void)
{
    QD.setupSPI(); //Ensure SPI is configured for this device
    return QD.readPosition(_quadDecCS);
}

// initialize the motor, encoder and DAC chips
void Servo::init(void)
{
    SPI.begin();
    _ls7366rConfig();
    _dacConfig(BIPOLAR_10V);
}

void Servo::_ls7366rConfig(void)
{
    uint8_t MDR0_Val;
    uint8_t MDR1_Val;

    // Set QD CS pin
    QD.setCSPin(_quadDecCS);

    QD.setupSPI(); //Ensure SPI is configured for this device

    QD.setMDR1Reg(MDR1_CONFIG);
    //delayMicroseconds(200);

    QD.setMDR0Reg(MDR0_CONFIG);
    //delayMicroseconds(200);

    //Now we will get the register vals to make sure everything is kosher
    MDR1_Val = QD.getMDR1Reg();  
    //delayMicroseconds(200);
    MDR0_Val = QD.getMDR0Reg();

    if (MDR0_CONFIG == MDR0_Val && MDR1_CONFIG == MDR1_Val)
    {
        Serial.print(" > LS7366R Quadrature Decoder (CS pin = ");
        Serial.print( QD.getChipSelectPin(), DEC );
        Serial.println(") successfully configured.");
    }
    else
    {
        Serial.println("LS7366R MDR0 vals:");
        Serial.println(MDR0_CONFIG, BIN);
        Serial.println(MDR0_Val, BIN);
        Serial.println("LS7366R MDR1 vals:");
        Serial.println(MDR1_CONFIG, BIN);
        Serial.println(MDR1_Val, BIN);
        Serial.println("\n*** LS7366R CONFIGURATION FAILURE! ***");
        //while (1);
    }

    // Clear counter value initially
    QD.clear(_quadDecCS);

}

void Servo::_dacConfig(uint8_t power_setting)
{  
    uint8_t newly_powered_DAC;

    //Initialize SPI for DAC
    DAC.setCSPin(_dacCS);
  
    DAC.setupSPI(); //Ensure SPI is configured for this device
    DAC.enableSDO();
    
    //See if another DAC is powered on so we don't overwrite settings
    //Note:  For now, it will make sure the DACs use the same range.  
    //If this is not desired, the code needs to be reworked.
    
    DAC.setOutputRange((uint8_t) selected_dac, (uint32_t) power_setting);
    
    uint32_t configuredDACPower = DAC.getPowerControl();
    uint8_t powered_DAC = configuredDACPower & 0xF; //Get the relevant portion
    
    if(selected_dac == DAC_A)
    {
      DAC.setPowerControl(POWER_DAC_A | powered_DAC); //Power up DAC A
      newly_powered_DAC = POWER_DAC_A;
    }
    else if(selected_dac == DAC_B)
    {
      DAC.setPowerControl(POWER_DAC_B | powered_DAC); 
      newly_powered_DAC = POWER_DAC_B;
    }
    else if(selected_dac == DAC_ALL)
    {
      DAC.setPowerControl(POWER_DAC_ALL); 
      newly_powered_DAC = POWER_DAC_ALL;
    }
    
    //Check the value of the power control register.
    uint32_t dacCheckPower = DAC.getPowerControl();

    if( (dacCheckPower & 0xF) == (newly_powered_DAC | powered_DAC))
    {
      Serial.print(" > DAC power control (DAC ");
      Serial.print( selected_dac, DEC );
      Serial.println(") successfully configured.\n");
    }
    else
    {
        Serial.println("DAC5752 Power Control Reg Values:");
        Serial.print("Config Value: ");
        Serial.print((selected_dac | powered_DAC), BIN);
        Serial.print("\nRegister Value: ");
        Serial.print(dacCheckPower & 0xF, BIN);
        Serial.println("\n\n*** DAC5752 CONFIGURATION FAILUE! ***");
        //while(1) {} //Wait indefinitely in error loop
    }
    
    DAC.disableSDO();
}

// apply +-10 V (double) signal to motor
void Servo::move(double volts)
{
    // Code for UNIPOLAR_10V  
    //float scale = volts / 10.0;
    //int16_t value = (int16_t) floor(scale * 0xFFFF); //Scale * value for 10V.

    // Code for BIPOLAR_10V
    int16_t value;

    if ( volts >= 0 )
    {
      // Set saturation
      if ( volts > 10.0 ) { volts = 10.0; }

      // Scale value for 10 V
      value = (int16_t) ( volts / 10.0 * 0x7FFF );
    }
    else
    {
      // Set saturation
      if ( volts < -10.0 ) { volts = -10.0; }

      // Scale value for 10 V, then take 2's complement
      value = (int16_t) ( abs(volts) / 10.0 * 0x8000 );
      value = ( ~value ) + 1;
    }
    
    // Set DAC output
    DAC.setValue(selected_dac, value);
    //DAC.disableSDO}
}

// Zero-out (clear) QD counter value
void Servo::zero(void)
{
  QD.clear(_quadDecCS);
}
