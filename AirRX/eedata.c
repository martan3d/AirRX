/*
 * eedata.c
 *
 * Created: 6/11/2020 12:46:32 PM
 *  Author: marta
 */ 

#include <avr/eeprom.h>
#include <stdio.h>

/*
 *  EEPROM Storage
 */

uint8_t  EEMEM AirwireChannel;               /* Radio RX channel */
uint16_t EEMEM DCCAddress;

int16_t  EEMEM ServoHigh0;
int16_t  EEMEM ServoHigh1;
int16_t  EEMEM ServoLow0;
int16_t  EEMEM ServoLow1;

uint8_t  EEMEM ServoReverse0;
uint8_t  EEMEM ServoReverse1;

/******************************************************************************/
uint8_t getEEAirwireChannel()
{
    uint8_t eedata;
    eedata = eeprom_read_byte( (const uint8_t*) &AirwireChannel );
    return eedata;
}

void setEEAirwireChannel(uint8_t mode)
{
    eeprom_write_byte( (uint8_t*) &AirwireChannel, mode );
}

/******************************************************************************/
uint16_t getEEDCCAddress()
{
    int16_t eedata;
    eedata = eeprom_read_word( (const uint16_t*) &DCCAddress );
    return eedata;
}

void setEEDCCAddress(int16_t addr)
{
    eeprom_write_word( (uint16_t*) &DCCAddress, addr );
}

/******************************************************************************/
uint16_t getEEServoHi( uint8_t sn)
{
    int16_t eedata = 0;
    switch(sn)
    {
        case 0:
               eedata = eeprom_read_word( (const uint16_t*) &ServoHigh0 );
               break;
               
        case 1:
               eedata = eeprom_read_word( (const uint16_t*) &ServoHigh1 );
               break;
    }
    return eedata;
}

void setEEServoHi(uint8_t sn, uint16_t addr)
{
    switch(sn)
    {
        case 0:
               eeprom_write_word( (uint16_t*) &ServoHigh0, addr );
        break;
        case 1:
               eeprom_write_word( (uint16_t*) &ServoHigh1, addr );
        break;
    }        
}

/******************************************************************************/
uint16_t getEEServoLow( uint8_t sn)
{
    int16_t eedata = 0;
    switch(sn)
    {
        case 0:
        eedata = eeprom_read_word( (const uint16_t*) &ServoLow0 );
        break;
        
        case 1:
        eedata = eeprom_read_word( (const uint16_t*) &ServoLow1 );
        break;
    }
    return eedata;
}

void setEEServoLow(uint8_t sn, uint16_t addr)
{
    switch(sn)
    {
        case 0:
               eeprom_write_word( (uint16_t*) &ServoLow0, addr );
        break;
        case 1:
               eeprom_write_word( (uint16_t*) &ServoLow1, addr );
        break;
    }
}

/******************************************************************************/
uint8_t getEEServoReverse(uint8_t sn, uint8_t mode)
{
    uint8_t eedata = 0;
    switch(sn)
    {
        case 0:
               eedata = eeprom_read_byte( (const uint8_t*) &ServoReverse0);
        break;
        case 2:
               eedata = eeprom_read_byte( (const uint8_t*) &ServoReverse1);
        break;
    }
    return eedata;
}

void setEEServoReverse(uint8_t sn, uint8_t mode)
{
    switch(sn)
    {
        case 0:
               eeprom_write_byte( (uint8_t*) &ServoReverse0, mode );
               break;
        case 1:
               eeprom_write_byte( (uint8_t*) &ServoReverse1, mode );
               break;
        break;
    }        
}
