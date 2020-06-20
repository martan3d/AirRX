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

uint8_t functioncode0 = 16;
uint8_t functioncode1 = 16;

uint8_t functionstate0 = 0;
uint8_t functionstate1 = 1;
uint8_t couplerFuncCode0 = 0;
uint8_t couplerFuncCode1 = 0;

uint8_t fcode = 0;

#define FORWARD 0
#define REVERSE 1
#define TRUE 1
#define FALSE 0

/* function code came in, do we care? Check our stored function code, did it come in? */

int8_t checkFunctionCodes(uint8_t ourcode, uint8_t f, uint8_t d)
{
    if ((f & 0xe0) == 0x80)      // function codes f0-f4
    {
        // The format of this instruction is 100DDDDD
        //                                      |||||___ F1
        //                                      ||||____ F2
        //                                      |||_____ F3
        //                                      ||______ F4
        //                                      |_______ F0/FL
        
        switch(ourcode)
        {
            case 0:
                   if (f & 0x10) return 1;
                      else return 0;
            case 1:
                   if (f & 0x01) return 1;
                   else return 0;
            case 2:
                   if (f & 0x02) return 1;
                   else return 0;
            case 3:
                   if (f & 0x04) return 1;
                   else return 0;
            case 4:
                   if (f & 0x08) return 1;
                   else return 0;
            default:
                   return -1;
        }            
    }

    
    if ((f & 0xe0) == 0xa0)     // function codes f5-f8
    {
        // This instruction has the format 101SDDDD  where S is 0
        //                                     ||||_____ F5
        //                                     |||______ F6
        //                                     ||_______ F7
        //                                     |________ F8
        
        switch(ourcode)
        {
            case 5:
                   if (f & 0x01) return 1;
                      else return 0;
            case 6:
                   if (f & 0x02) return 1;
                   else return 0;
            case 7:
                   if (f & 0x04) return 1;
                   else return 0;
            case 8:
                   if (f & 0x08) return 1;
                   else return 0;
            default:
                   return -1;
        }
    }

    if ((f & 0xe0) == 0xb0)     // function codes f9-f12
    {
        // This instruction has the format 101SDDDD  where S is 1
        //                                     ||||_____ F9
        //                                     |||______ F10
        //                                     ||_______ F11
        //                                     |________ F12

        switch(ourcode)
        {
            case 9:
                   if (f & 0x01) return 1;
                      else return 0;
            case 10:
                   if (f & 0x02) return 1;
                   else return 0;
            case 11:
                   if (f & 0x04) return 1;
                   else return 0;
            case 12:
                   if (f & 0x08) return 1;
                   else return 0;
            default:
                   return -1;
        }
    }

    if ((f & 0xe0) == 0xc0)     // function codes f13-f20
    {
        if(( f & 0x1f) == 0x1d)
        {
            // d contains data
            // DDDDDDDD         F13-F20

           switch(ourcode)
           {
            case 13:
                   if (d & 0x01) return 1;
                      else return 0;
            case 14:
                   if (d & 0x02) return 1;
                   else return 0;
            case 15:
                   if (d & 0x04) return 1;
                   else return 0;
            case 16:
                   if (d & 0x08) return 1;
                   else return 0;
            case 17:
                   if (d & 0x10) return 1;
                      else return 0;
            case 18:
                   if (d & 0x20) return 1;
                   else return 0;
            case 19:
                   if (d & 0x40) return 1;
                   else return 0;
            case 20:
                   if (d & 0x80) return 1;
                   else return 0;
            default:
                   return -1;
           }
        }

        if(( f & 0x1f) == 0x1e)
        {
            // d contains data
            // DDDDDDDD         F21-f28

           switch(ourcode)
           {
            case 21:
                   if (d & 0x01) return 1;
                      else return 0;
            case 22:
                   if (d & 0x02) return 1;
                   else return 0;
            case 23:
                   if (d & 0x04) return 1;
                   else return 0;
            case 24:
                   if (d & 0x08) return 1;
                   else return 0;
            case 25:
                   if (d & 0x10) return 1;
                      else return 0;
            case 26:
                   if (d & 0x20) return 1;
                   else return 0;
            case 27:
                   if (d & 0x40) return 1;
                   else return 0;
            case 28:
                   if (d & 0x80) return 1;
                   else return 0;
            default:
                   return -1;
           }
        }
    }

  return -1;
}

void checkOurFunctionCodes()
{
    /* check physical output 0 */
    
    fcode = checkFunctionCodes(functioncode0, rawbuff[1], rawbuff[2]);
    if( fcode != -1)
      {
          if (fcode == 1)
             {
               if (functionstate0 == 1)     // check off state, if one, flip to off
                   PORTA &= 0xfb;
                else
                   PORTA |= 0x40;            // if off state is zero, its on now
               
               if (functionstate0 == 0)     // check off state, if zero, flip to on
                   PORTA |= 0x40;            // if off state is zero, its on now
                else
                   PORTA &= 0xfb;
             }
             else
             {
               if (functionstate0 == 1)     // check off state, if one, flip to off
                   PORTA |= 0x40;            // if off state is zero, its on now
               else
                   PORTA &= 0xfb;
              
               if (functionstate0 == 0)     // check off state, if zero, flip to on
                   PORTA &= 0xfb;
               else
                   PORTA |= 0x40;            // if off state is zero, its on now
            }
      }
                         
    /* check the second physical output, did a function code trigger it ? */
                         
    fcode = checkFunctionCodes(functioncode1, rawbuff[1], rawbuff[2]);
    if( fcode != -1)
       {
         if (fcode == 1)
             {
               if (functionstate0 == 1)     // check off state, if one, flip to off
                   PORTA &= 0xf7;
                 else
                   PORTA |= 0x80;            // if off state is zero, its on now
               
               if (functionstate0 == 0)     // check off state, if zero, flip to on
                   PORTA |= 0x80;            // if off state is zero, its on now
                 else
                   PORTA &= 0xf7;
             }
            else
             {
               if (functionstate0 == 1)     // check off state, if one, flip to off
                   PORTA |= 0x80;            // if off state is zero, its on now
                 else
                   PORTA &= 0xf7;
              
               if (functionstate0 == 0)     // check off state, if zero, flip to on
                   PORTA &= 0xf7;
                 else
                   PORTA |= 0x80;            // if off state is zero, its on now
             }
       }
                         
    /* did the coupler function code come in for coupler 0 */
                         
    fcode = checkFunctionCodes(couplerFuncCode0, rawbuff[1], rawbuff[2]);
    if( fcode != -1)
       {
          if (fcode == 1)
             setServoPulse(0, servohigh0);  // if function code is a 1, highlimit
            else
             setServoPulse(0, servolow0);
       }

   fcode = checkFunctionCodes(couplerFuncCode0, rawbuff[1], rawbuff[2]);
   if( fcode != -1)
      {
          if (fcode == 1)
             setServoPulse(0, servohigh0);  // if function code is a 1, highlimit
          else
             setServoPulse(0, servolow0);
      }
}

/*
 * Decode message packets here and do servos and config variables
 *
 *   CV 200 - Radio Channel 0-15
 *   CV 201 - Radio Power Code
 *   CV 202 - DCC Address lo
 *   CV 203 - DCC Address hi
 *   CV 204 - Servo Mode 0=Steam, 1=couplers, 2=ESC
 *   CV 205 - Servo0 LowLimit Lo
 *   CV 206 - Servo0 LowLimit Hi
 *   CV 207 - Servo0 HighLimit Lo
 *   CV 208 - Servo0 HighLimit Hi
 *   CV 209 - Servo1 LowLimit Lo
 *   CV 210 - Servo1 LowLimit Hi
 *   CV 211 - Servo1 HighLimit Lo
 *   CV 212 - Servo1 HighLimit Hi
 *   CV 213 - Function Code for Output x
 *   CV 214 - Function Code for Output y
 *   CV 215 - On/Off Code for Output x
 *   CV 216 - On/Off Code for Output y
 */

static volatile uint16_t temp;

void checkConfigurationCode(uint8_t * c)
{
    // data is  1110CCVV VVVVVVVV DDDDDDDD
    // this assumes address is ok and c[0] is beginning of data
    
    uint16_t cvd;
    
    if( (c[0] & 0xf0) == 0xe0)
    {
        // configuration write instruction ?
        if((c[0] & 0x0c) == 0x0c)
        {
            cvd = c[2];                    // radio channel, check bounds

            switch(c[1])
            {
                case 200:
                          if(cvd < 0)  cvd = 0;
                          if(cvd > 15) cvd = 15;
                          setEEAirwireChannel(cvd);
                          break;
                          
                case 201: // Power level?  Not used
                          break;
                          
                case 202: // DCC address low byte
                          temp &= 0xff00;
                          temp |= cvd;
                          setEEDCCAddress(temp);
                          break;  
                          
                case 203: // DCC address high byte
                          temp &= 0x00ff;
                          temp |= (cvd<<8);
                          setEEDCCAddress(temp);
                          break;
                          
                case 204: // Servo Mode
                          if(cvd<0) cvd = 0;
                          if(cvd>2) cvd = 2;
                          setEEServoMode(cvd);
                          break;
                          
                case 205: // Servo 0 Low Limit low byte - do this one first
                          temp = cvd;
                          break;
                          
                case 206: // Servo 0 Low Limit high byte - when high byte comes in, write to EEPROM
                          temp &= 0x00ff;
                          temp |= (cvd<<8);
                          setEEServoLow(0, temp);
                          break;
                                       
                case 207: // Servo 1 Low Limit low byte - do this one first
                          temp = cvd;
                          break;
                          
                case 208: // Servo 1 Low Limit high byte - when high byte comes in, write to EEPROM
                          temp &= 0x00ff;
                          temp |= (cvd<<8);
                          setEEServoLow(1, temp);
                          break;

                case 209: // Servo 0 High Limit low byte - do this one first
                          temp = cvd;
                          break;
                          
                case 210: // Servo 0 High Limit high byte - when high byte comes in, write to EEPROM
                          temp &= 0x00ff;
                          temp |= (cvd<<8);
                          setEEServoHi(0, temp);
                          break;
                                       
                case 211: // Servo 1 High Limit low byte - do this one first
                          temp = cvd;
                          break;
                          
                case 212: // Servo 1 High Limit high byte - when high byte comes in, write to EEPROM
                          temp &= 0x00ff;
                          temp |= (cvd<<8);
                          setEEServoHi(1, temp);
                          break;
                          
                case 213:
                          if(cvd < 0)  cvd = 0;
                          if(cvd > 28) cvd = 28;
                          setEECouplerfunctionCode(0, cvd);
                          break;

                case 214:
                          if(cvd < 0)  cvd = 0;
                          if(cvd > 28) cvd = 28;
                          setEECouplerfunctionCode(1, cvd);
                          break;

                case 215:
                          if(cvd < 0) cvd = 0;
                          if(cvd > 1) cvd = 1;
                          setEEFunctionState(0, cvd);
                          break;

                case 216:
                          if(cvd < 0) cvd = 0;
                          if(cvd > 1) cvd = 1;
                          setEEFunctionState(1, cvd);
                          break;

            }
        }
    }
    
}

/*
 * Setup only, initialize the EEPROM with defaults
 *
 */

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
    setEEFunctionOutput(0,12);
    setEEFunctionOutput(1,12);
    setEEFunctionState(0,0);
    setEEFunctionState(1,0);
    setEECouplerfunctionCode(0,16);
    setEECouplerfunctionCode(1,16);
}

/* 
 * Main Loop
 *
 */


int main(void)
{
    //int i;
       
    DDRB |= 0x03;    // PB0, PB1 = outputs
    
    //initEEPROM();    // TEMPORARY SETUP ONLY

    // restore all from EEPROM
        
    radioChannel   = getEEAirwireChannel();
    dccaddress     = getEEDCCAddress();
    functioncode0  = getEEfunctionOutput(0);
    functioncode1  = getEEfunctionOutput(1);
    functionstate0 = getEEfunctionState(0);
    functionstate1 = getEEfunctionState(1);
    servolow0      = getEEServoLow(0);
    servolow1      = getEEServoLow(1);
    servohigh0     = getEEServoHi(0);
    servohigh1     = getEEServoHi(1);
    servoreverse0  = getEEServoReverse(0);
    servoreverse1  = getEEServoReverse(1);
    servomode      = getEEServoMode();
    
    couplerFuncCode0 = getEECouplerfunctionCode(0);
    couplerFuncCode1 = getEECouplerfunctionCode(1);
        
    switch(servomode )
    {
        case 0:                               // servomode = 0 Steam
            setServoPulse(0,servolow0);
            setServoPulse(1,servohigh1);      // high limit is forward
        break;
        
        case 1:
            setServoPulse(0,servolow0);       // Coupler mode
            setServoPulse(1,servolow1);       // low limit is couplers normal
        break;
        
        case 2:                               // ESC - center off
            setServoPulse(0, 500);            // ESC Center off is middle of servo pulse 0-1000
        break;
    }    
        
    initServoTimer();
    initializeSPI();
    startModem(radioChannel);
    dccInit();
    
    UART_init();  // software usart, for debug only, take this out for production
        
    sei();                                   // enable interrupts
    
    while (1)
    {
        /* wait for DCC message to be constructed via interrupt */
        
        if (flagbyte)
        {
            getDCC(rawbuff);                       // pass our buffer to dcc to retrieve data

            /************************************* MUST be in 128 step mode */

            msglen = rawbuff[5];                   // get length of message

            ouraddress = FALSE;                    // plan to fail
            
            switch(msglen)
            {            
                         // three byte message, what is it, Function codes?
                case 3:
                         rxaddress = rawbuff[0];               // match our address?
                         if (rxaddress != dccaddress)
                             break;                            // nope, skip
                         ouraddress = TRUE;
                         checkOurFunctionCodes();
                      break;
                
                
                        // check short address throttle packet or long address function (both are 4 length)
                case 4:   
                        if (rawbuff[1] == 0x3f)   // must be 128 step mode throttle packet
                           {
                             rxaddress = rawbuff[0];   // match our address?
                             if (rxaddress != dccaddress)
                                break;                 // nope, bail
                        
                             ouraddress = TRUE;
                             dccspeed = rawbuff[2];    // get speed and direction
                             if (dccspeed & 0x80)
                                direction = FORWARD;
                             else
                                direction = REVERSE;

                             dccspeed = dccspeed & 0x7f;
                             break;
                           }
                           
                           if (ouraddress)
                              checkOurFunctionCodes();
                           
                           break;
                         
                case 5:                           // long address
                        if(rawbuff[2] == 0x3f)   // must be 128 step mode throttle packet
                          {
                             rxaddress = rawbuff[0];
                             rxaddress = dccaddress << 8;
                             rxaddress |= rawbuff[1];
                        
                             if (rxaddress != dccaddress)
                                break;
                        
                             ouraddress = TRUE;
                             dccspeed = rawbuff[3];
                             if (dccspeed & 0x80)
                                direction = FORWARD;
                             else
                                direction = REVERSE;
                             dccspeed = dccspeed & 0x7f;
                             break;
                          }
                          break;
            }
            
            /* message for our DCC address came in, process it */
            
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
                    
                    case 2:                                // Center off ESC mode
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
            }      /* end of DCC message received, clear flag and wait for another */
            
            flagbyte = 0;
        }
    }
}

