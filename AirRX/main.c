/*
 * AirRX.c
 *
 * Created: 12/10/2018 8:19:10 PM
 * Author : martan
 
   Airwire DCC plus two Servos
   
   This is for the Attiny24/44/84 series 14pin DIP
  
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#include "servotimer.h"
#include "softUART.h"
#include "spi.h"
#include "dcc.h"
#include "eedata.h"


/*
 *
 *   Main Loop. Decode message packets here and do servos and config variables
 *
 *   CV 200 - Radio Channel 0-15
 *   CV 201 - Radio Power Code
 *   CV 202 - DCC Address lo
 *   CV 203 - DCC Address hi
 *   CV 204 - Servo Mode 0=ESC 1=Steam
 *   CV 205 - Servo0 LowLimit Lo
 *   CV 206 - Servo0 LowLimit Hi
 *   CV 207 - Servo0 HighLimit Lo
 *   CV 208 - Servo0 HighLimit Hi
 *   CV 209 - Servo1 LowLimit Lo
 *   CV 210 - Servo1 LowLimit Hi
 *   CV 211 - Servo1 HighLimit Lo
 *   CV 212 - Servo1 HighLimit Hi
 *   CV 213 - Func Code for Output x
 *   CV 214 - Func Code for Output y
 *   CV 215 - On/Off Code for Output x
 *   CV 216 - On/Off Code for Output y
 */


static volatile uint8_t flagbyte;

void setFlag()
{
    flagbyte = 1;
}


uint8_t rawbuff[7];
DCC_MSG * dccmsgptr;

uint8_t decodeDCCPacket( DCC_MSG * dccptr)
{
    uint8_t l;
    
    l = dccptr->Size;       // length of packet
    
    switch(l)
    {
        case 3:             // three bytes
        rawbuff[0] = dccptr->Data[0];
        rawbuff[1] = dccptr->Data[1];
        rawbuff[2] = dccptr->Data[2];
        break;
        
        case 4:
        rawbuff[0] = dccptr->Data[0];
        rawbuff[1] = dccptr->Data[1];
        rawbuff[2] = dccptr->Data[2];
        rawbuff[3] = dccptr->Data[3];
        break;
        
        case 5:
        rawbuff[0] = dccptr->Data[0];
        rawbuff[1] = dccptr->Data[1];
        rawbuff[2] = dccptr->Data[2];
        rawbuff[3] = dccptr->Data[3];
        rawbuff[4] = dccptr->Data[4];
        break;

        case 6:
        rawbuff[0] = dccptr->Data[0];
        rawbuff[1] = dccptr->Data[1];
        rawbuff[2] = dccptr->Data[2];
        rawbuff[3] = dccptr->Data[3];
        rawbuff[4] = dccptr->Data[4];
        rawbuff[5] = dccptr->Data[5];
        break;
    }

    return l;
}



int main(void)
{
   int i;
       
    DDRB |= 0x03;    // PB0, PB1 = outputs
    
    initServoTimer();
    initializeSPI();
    startModem(0);
    dccInit();
    UART_init();
        
    sei();                                      // enable interrupts
    
    while (1)
    {
        if (flagbyte)
        {
            dccmsgptr = getDCC();
            decodeDCCPacket(dccmsgptr);
            for(i=0;i<6;i++) 
            {
              while(1)
               { 
                   if(UART_tx(rawbuff[i]))
                      break;
               }                   
            }            
            flagbyte = 0;
        }
    }
}

