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

static volatile uint8_t rawbuff[7];
static volatile uint8_t flagbyte;

void setFlag()
{
    flagbyte = 1;
}

uint16_t dccaddress;
uint16_t rxaddress;
uint16_t servolow0;
uint16_t servolow1;
uint16_t servohigh0;
uint16_t servohigh1;
uint16_t s;

uint8_t  servoreverse0;
uint8_t  servoreverse1;
uint8_t  servomode;
uint8_t  radioChannel = 0;
uint8_t  msglen = 0;
uint8_t  dccspeed = 0;
uint8_t  direction = 0;
uint8_t  ouraddress = 0;

uint8_t  s0Func;
uint8_t s1Func, s2Func, svfunc;


#define FORWARD 0
#define REVERSE 1

uint8_t checkCoupler(uint8_t c, uint8_t fc)
{
    return 1;
}    


void initEEPROM()
{
    setEEAirwireChannel(0);
    setEEDCCAddress(3);
    setEEServoHi(0, 1000);
    setEEServoHi(1, 1000);
    setEEServoLow(0, 0);
    setEEServoLow(1, 0);
    setEEServoReverse(0, 0);
    setEEServoReverse(1, 0);
    setEEServoMode(1);
}




int main(void)
{
   int i;
       
    DDRB |= 0x03;    // PB0, PB1 = outputs
    
    initEEPROM();    // TEMPORARY SETUP ONLY
    
    radioChannel = getEEAirwireChannel();
    dccaddress   = getEEDCCAddress();
    servolow0    = getEEServoLow(0);
    servolow1    = getEEServoLow(1);
    servohigh0   = getEEServoHi(0);
    servohigh1   = getEEServoHi(1);
    
    servoreverse0 = getEEServoReverse(0);
    servoreverse1 = getEEServoReverse(1);
    servomode     = getEEServoMode();
        
    switch(servomode )
    {
        case 0:                               // servomode = 0 Steam
            setServoPulse(0,servolow0);
            setServoPulse(1,servohigh1);      // high limit is forward
        break;
        
        case 1:                               // servomode = 1 ESC - center off
            setServoPulse(0, 500);            // ESC Center off is middle of servo pulse 0-1000
        break;
    }    
        
    initServoTimer();
    initializeSPI();
    startModem(radioChannel);
    dccInit();
    UART_init();
        
    sei();                                      // enable interrupts
    
    while (1)
    {
        /* wait for DCC message to be constructed via interrupt */
        
        if (flagbyte)
        {
            getDCC(rawbuff);                       // pass our buffer to dcc to retrieve data
         
            /* MUST be in 128 step mode */
            
            msglen = rawbuff[5];                   // get length of message

            ouraddress = 0;

            switch(msglen)
            {
                case 4:                           // short address
                        if (rawbuff[1] != 0x3f)   // not 128 step mode
                           break;                 // don't process
                
                        rxaddress = rawbuff[0];   // match our address?
                        if (rxaddress != dccaddress)
                           break;                 // nope, bail
                        
                        ouraddress = 1;
                                                
                        dccspeed = rawbuff[2];    // get speed and direction
                        
                        if (dccspeed & 0x80)
                            direction = FORWARD;
                        else
                            direction = REVERSE;
                                                  // remove direction bit
                        dccspeed = dccspeed & 0x7f;

                case 5:                           // long address
                        if (rawbuff[2] != 0x3f)   // not 128 step mode
                           break;

                        rxaddress = rawbuff[0];
                        rxaddress = dccaddress << 8;
                        rxaddress |= rawbuff[1];
                        
                        if (rxaddress != dccaddress)
                            break;
                        
                        ouraddress = 1;
                        
                        dccspeed = rawbuff[3];
                        
                        if (dccspeed & 0x80)
                            direction = FORWARD;
                        else
                            direction = REVERSE;
                            
                        dccspeed = dccspeed & 0x7f;

            }
            
            if(ouraddress)
            {
                switch(servomode)
                {
                    case 0:                                 // Steam mode?
                            if (direction == FORWARD)       // for live steam, servo 1 is direction
                               setServoPulse(1,servohigh1); // high limit is forward
                            else
                               setServoPulse(1,servolow1);  // low limit is reverse
                    
                            s = dccspeed * 10;              // 128 max steps is a bit over servo max of 1000
                            if(s>1000) s = 1000;            // since channel 0 servo is also throttle via servo or ESC, set it same as DCC
                            if (s<0) s = 0;                 // make sure we don't exceed limits
                            setServoPulse(0,s);             // send throttle value to servo
                    break;
                    
                   
                    case 1:                                // Center off ESC mode
                              if (direction == FORWARD)
                              {
                                 s = dccspeed;
                                 s = (s*4) + 500;
                                 if (s>1000) s = 1000;     // since channel 0 servo is also throttle via servo or ESC, set it same as DCC
                                 if (s<0) s = 0;           // make sure we don't exceed limits
                                 setServoPulse(0,s);       // send throttle value to servo
                              }
                              else
                              {
                                s = dccspeed;
                                s = 500 - (s*4);
                                if (s>1000) s = 1000;
                                if (s<0) s = 0;
                                setServoPulse(0,s);
                              }
                    break;
                    
                }
            }
         
         
            
            for(i=0;i<6;i++)                    // send it out on the soft uart
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

