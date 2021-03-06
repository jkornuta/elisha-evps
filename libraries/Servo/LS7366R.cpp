/*
* LS7366R Class
* Code written by Jeff Kornuta
* Packaged and expanded by Phillip Johnston
*
* 25 May 2012
*
* LS7366R.cpp
*/

#include "LS7366R.h"
#include "SPI.h"
#include "SPIDevice.h"

/********************
* Define Statements *
*********************/
#ifdef LS7366R_DEBUG
    #define debugf(msg) Serial.print msg
#else
    #define debugf(msg)
#endif

/***********************
* Function Definitions *
***********************/
LS7366RClass::LS7366RClass()
{
    debugf(("Initializing LS7366R object...\n"));
}

LS7366RClass::~LS7366RClass()
{
    //Not destructing anything currently
}

void LS7366RClass::clear(uint8_t cs_pin)
{
  // Clear counter
  CSpin = cs_pin;
  _send(CLEAR_COUNTER, 0);
}

void LS7366RClass::setupSPI()
{
   SPI.setBitOrder(MSBFIRST);
   SPI.setDataMode(SPI_MODE1); 
   SPI.setClockDivider(SPI_CLOCK_DIV16);
}

void LS7366RClass::setMDR0Reg(uint8_t config_val)
{
    _send(WRITE_MDR0, config_val, 0);
}

void LS7366RClass::setMDR1Reg(uint8_t config_val)
{
    _send(WRITE_MDR1, config_val, 0);
}
      
double LS7366RClass::readPosition(uint8_t cs_pin)
{
    int32_t count;

    // Know which QD to use!
    CSpin = cs_pin;
    
    // send read counter command
    count = _transfer( READ_COUNTER );
    
    // return value in mm (change sign for correct direction)
    return (-1.0*(double)count / 2000.0); //0.5um / count
}

uint8_t LS7366RClass::getMDR0Reg()
{
    uint32_t return_data = _transfer(READ_MDR0, 0, 0);
    return (uint8_t) (return_data & 0xFF);
}

uint8_t LS7366RClass::getMDR1Reg()
{
    uint32_t return_data = _transfer(READ_MDR1, 0, 0);
    return (uint8_t) (return_data & 0xFF); //double check this to make sure we're grabbing the right byte
}
