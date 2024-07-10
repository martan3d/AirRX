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
#include "spi.h"
#include "dcc.h"
#include "eedata.h"

static volatile uint8_t rawbuff[7];
static volatile uint8_t flagbyte;

#define DEBUG 1

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


                  // 28 step table. The 5th bit is used in an odd way, this is why the numbers look a bit odd - see the table in the DCC spec
				  
/*
s    bits   index

00 = 00000  00
01 = 00000  00
02 = 10010  18
03 = 00011  03
04 = 10011  19
05 = 00100  04
06 = 10100  20
07 = 00101  05
08 = 10101  21
09 = 00110  06
10 = 10110  22
11 = 00111  07
12 = 10111  23
13 = 01000  08
14 = 11000  24
15 = 01001  09
16 = 11001  25
17 = 01010  10
18 = 11010  26
19 = 01011  11
20 = 11011  27
21 = 01100  12
22 = 11100  28
23 = 01101  13
24 = 11101  29
25 = 01110  14
26 = 11110  30
27 = 01111  15
28 = 11111  31
*/
				  
                  //     00  01  02  03  04  05  06  07  08  09  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31
uint8_t stepTable[]  = {  0,  0,  1,  3,  5,  7,  9, 11, 13, 15, 17, 19, 21, 23, 25, 27,  0,  0,  2,  4,  6,  8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28 };

#define FORWARD 0
#define REVERSE 1
#define TRUE 1
#define FALSE 0
#define STEAM 0
#define COUPLERS 1
#define ESC 2
#define PWM 3



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
                   PORTA |= 0x04;            // if off state is zero, its on now
               
               if (functionstate0 == 0)      // check off state, if zero, flip to on
                   PORTA |= 0x04;            // if off state is zero, its on now
                else
                   PORTA &= 0xfb;
             }
             else
             {
               if (functionstate0 == 1)     // check off state, if one, flip to off
                   PORTA |= 0x04;            // if off state is zero, its on now
               else
                   PORTA &= 0xfb;
              
               if (functionstate0 == 0)     // check off state, if zero, flip to on
                   PORTA &= 0xfb;
               else
                   PORTA |= 0x04;            // if off state is zero, its on now
            }
      }
                         
    /* check the second physical output, did a function code trigger it ? */
                         
    fcode = checkFunctionCodes(functioncode1, rawbuff[1], rawbuff[2]);
    if( fcode != -1)
       {
         if (fcode == 1)
             {
               if (functionstate1 == 1)     // check off state, if one, flip to off
                   PORTA &= 0xf7;
                 else
                   PORTA |= 0x08;            // if off state is zero, its on now
               
               if (functionstate1 == 0)     // check off state, if zero, flip to on
                   PORTA |= 0x08;            // if off state is zero, its on now
                 else
                   PORTA &= 0xf7;
             }
            else
             {
               if (functionstate1 == 1)     // check off state, if one, flip to off
                   PORTA |= 0x08;            // if off state is zero, its on now
                 else
                   PORTA &= 0xf7;
              
               if (functionstate1 == 0)     // check off state, if zero, flip to on
                   PORTA &= 0xf7;
                 else
                   PORTA |= 0x08;            // if off state is zero, its on now
             }
       }
                         
             
    /* don't care about couplers if not in coupler mode */
                         
    if (servomode != 1)
      return;
                         
    /* did the coupler function code come in for coupler 0 */
                         
    fcode = checkFunctionCodes(couplerFuncCode0, rawbuff[1], rawbuff[2]);
    if( fcode != -1)
       {
          if (fcode == 1)
             setServoPulse(0, servohigh0);  // if function code is a 1, highlimit
            else
             setServoPulse(0, servolow0);
       }

   fcode = checkFunctionCodes(couplerFuncCode1, rawbuff[1], rawbuff[2]);
   if( fcode != -1)
      {
          if (fcode == 1)
             setServoPulse(1, servohigh1);  // if function code is a 1, highlimit
          else
             setServoPulse(1, servolow1);
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
    setEEServoMode(0);
    setEEFunctionOutput(0,4);
    setEEFunctionOutput(1,4);
    setEEFunctionState(0,1);
    setEEFunctionState(1,1);
    setEECouplerfunctionCode(0,16);
    setEECouplerfunctionCode(1,16);
}

/*
 * Decode message packets here and do servos and config variables
 *
 *   CV 201 - Radio Channel 0-15
 *   CV 202 - DCC Address lo
 *   CV 203 - DCC Address hi
 *   CV 204 - Servo Mode 0=Steam, 1=couplers, 2=ESC
 *   CV 205 - Servo0 LowLimit Lo
 *   CV 206 - Servo0 LowLimit Hi
 *   CV 207 - Servo0 HighLimit Lo
 *   CV 208 - Servo0 HighLimit Hi
 *   CV 209 - Servo0 Reverse
 *   CV 210 - Servo1 LowLimit Lo
 *   CV 211 - Servo1 LowLimit Hi
 *   CV 212 - Servo1 HighLimit Lo
 *   CV 213 - Servo1 HighLimit Hi
 *   CV 214 - Servo1 Reverse
 *   CV 215 - Function Code for Coupler 0
 *   CV 216 - Function Code for Coupler 1
 *   CV 217 - Function Code for Output x
 *   CV 218 - Function Code for Output y
 *   CV 219 - On/Off Code for Output x
 *   CV 220 - On/Off Code for Output y
 *   CV 230 - Reset to factory defaults
 */

static volatile uint16_t temp;

void checkConfigurationCode(uint8_t addr, uint8_t data)
{
    // command is  1110CCVV VVVVVVVV DDDDDDDD
    
    uint16_t cvd;
    uint8_t mdata;
    
            cvd = data;
            mdata = data;
            
            addr ++;     // DCC sends address - 1

            switch(addr)
            {
                case 201:
                          if(mdata < 0)  break;
                          if(mdata > 16) break;
                          setEEAirwireChannel(mdata);
                          radioChannel = mdata;
                          startModem(radioChannel);
                          break;
                          
                case 202: // DCC address low byte
                          temp = mdata;
                          break;  
                          
                case 203: // DCC address high byte
                          temp &= 0x00ff;
                          mdata = cvd & 0x00ff;
                          temp |= (cvd<<8);
                          
                          if(temp < 10240)       // Max DCC address
                          {
                            setEEDCCAddress(temp);
                            dccaddress = temp;
                          }                          
                          break;
                          
                case 204: // Servo Mode
                          if(cvd<0) break;
                          if(cvd>3) break;       // allow pwm mode now
                          setEEServoMode(cvd);
                          servomode = cvd;
                          break;
                          
                          // **** Servo 0 Limits and Reverse *******
                          
                case 205: // Servo 0 Low Limit low byte - do this one first
                          temp = cvd;
                          break;
                          
                case 206: // Servo 0 Low Limit high byte - when high byte comes in, write to EEPROM
                          temp &= 0x00ff;
                          temp |= (cvd<<8);
                          setEEServoLow(0, temp);
                          setServoPulse(0, temp);
                          servolow0 = temp;
                          break;
                                       
                case 207: // Servo 0 High Limit low byte - do this one first
                          temp = cvd;
                          break;
                          
                case 208: // Servo 0 High Limit high byte - when high byte comes in, write to EEPROM
                          temp &= 0x00ff;
                          temp |= (cvd<<8);
                          setEEServoHi(0, temp);
                          setServoPulse(0, temp);
                          servohigh0 = temp;
                          break;

                case 209: if (cvd>1) break;
                          if (cvd<0) break;
                          servoreverse0 = cvd;
                          setServoReverseValue(0, cvd);
                          setEEServoReverse(0, cvd);
                          break;
                                
                          // ***** Servo 1 Low Limit, High Limit, reverse *******
                
                case 210: // Servo 1 Low Limit low byte - do this one first
                          temp = cvd;
                          break;
                          
                case 211: // Servo 1 Low Limit high byte - when high byte comes in, write to EEPROM
                          temp &= 0x00ff;
                          temp |= (cvd<<8);
                          setEEServoLow(1, temp);
                          setServoPulse(1, temp);
                          servolow1 = temp;
                          break;
                                       
                case 212: // Servo 1 High Limit low byte - do this one first
                          temp = cvd;
                          break;
                          
                case 213: // Servo 1 High Limit high byte - when high byte comes in, write to EEPROM
                          temp &= 0x00ff;
                          temp |= (cvd<<8);
                          setEEServoHi(1, temp);
                          setServoPulse(1, temp);
                          servohigh1 = temp;
                          break;
                          
                case 214: if (cvd>1) break;
                          if (cvd<0) break;
                          servoreverse1 = cvd;
                          setServoReverseValue(1, cvd);
                          setEEServoReverse(1, cvd);
                          break;
                          
                          //*** Servo 0 Coupler Mode FUNCTION CODE *****
                case 215:
                          if(cvd < 0)  break;
                          if(cvd > 28) break;
                          setEECouplerfunctionCode(0, cvd);
                          break;

                          //*** Servo 1 Coupler Mode FUNCTION CODE *******
                case 216:
                          if(cvd < 0)  break;
                          if(cvd > 28) break;
                          setEECouplerfunctionCode(1, cvd);
                          break;

                case 217: // ***** Function code for OUTPUT X ****
                          if(cvd < 0)  break;
                          if(cvd > 28) break;
                          setEEFunctionOutput(0,cvd);
                          functioncode0 = cvd;
                          break;

                case 218: // ***** Function code for OUTPUT Y ***
                          if(cvd < 0)  break;
                          if(cvd > 28) break;
                          setEEFunctionOutput(1,cvd);
                          functioncode1 = cvd;
                          break;

                          // **** Digital Outputs default (OFF) State
                case 219:
                          if(cvd < 0) cvd = 0;
                          if(cvd > 1) cvd = 1;
                          setEEFunctionState(0, cvd);
                          break;

                case 220:
                          if(cvd < 0) cvd = 0;
                          if(cvd > 1) cvd = 1;
                          setEEFunctionState(1, cvd);
                          break;
                          
                          // ******** FACTORY RESET
                case 230:
                          initEEPROM();
                          break;
        }
}


/* 
 * Main Loop
 *
 */


int main(void)
{
      
    DDRB &= 0xfd;    // PB1 = input
    PORTB |= 0x02;   // Pull up
    DDRB |= 0x01;    // PB0 = output
    DDRA |= 0x0f;    // PA0-PA3 outputs
    
    //initEEPROM();
    
    if(getEEProgrammed() != 0)   /** First time power up, set all to defaults **/
    {
        initEEPROM();  
        setEEProgrammed(0);
    }
    
    // check port pin B2 - if low, initialize EEPROM *** USER FACTORY RESET  ***  NEED to test this
 
    if ( (PINB & 0x02) == 0 )
    {
        initEEPROM();
    }        

    // restore all from EEPROM
        
    radioChannel   = getEEAirwireChannel();
    dccaddress     = getEEDCCAddress();
    
    functioncode0  = getEEfunctionOutput(0);
    functioncode1  = getEEfunctionOutput(1);
    
    functionstate0 = getEEfunctionState(0) & 1;
    functionstate1 = getEEfunctionState(1) & 1;
    
    // set outputs to default states
    
    PORTA &= 0xfb;   // output x
    PORTA |= (functionstate0 << 2);
    
    PORTA &= 0xf7;
    PORTA |= (functionstate1 << 3);
    
    // Load up servo values from EEPROM
        
    servolow0      = getEEServoLow(0);
    servolow1      = getEEServoLow(1);
    servohigh0     = getEEServoHi(0);
    servohigh1     = getEEServoHi(1);

    servoreverse0  = getEEServoReverse(0);
    setServoReverseValue(0, servoreverse0);
    
    servoreverse1  = getEEServoReverse(1);
    setServoReverseValue(1, servoreverse1);
    
    servomode      = getEEServoMode();
	
	servomode      = PWM;    // debug *******************
    
    couplerFuncCode0 = getEECouplerfunctionCode(0);
    couplerFuncCode1 = getEECouplerfunctionCode(1);
    
    // Set Servo outputs to startup states depending on mode
    
    switch(servomode)
    {
        case STEAM:                           // servomode = 0 Steam
            initServoTimer(1);
            setServoPulse(0,servolow0);
            setServoPulse(1,servohigh1);      // high limit is forward
        break;
        
        case COUPLERS:
            initServoTimer(1);
            setServoPulse(0,servolow0);       // Coupler mode
            setServoPulse(1,servolow1);       // low limit is couplers normal
        break;
        
        case ESC:                             // ESC - center off
            initServoTimer(1);
            setServoPulse(0, 500);            // ESC Center off is middle of servo pulse 0-1000
            setServoPulse(1,servolow1);
        break;
        
        case PWM:
             //DDRA |= 0x0f;                    // insure that outputx and y are actually outputs
             initPWM();
             initServoTimer(1);
             direction = FORWARD;
        break;        
    }    
        
    initializeSPI();
    startModem(radioChannel);
    dccInit();
    
    sei();                                   // enable interrupts

    /**************************************************************************************************************/
    
    while (1)
    {
        /* wait for DCC message to be constructed via interrupt */
      
        if (flagbyte)
        {
            getDCC(rawbuff);                       // pass our buffer to dcc to retrieve data
            msglen = rawbuff[5];                   // get length of message
            ouraddress = FALSE;                    // plan to ignore
            
            switch(msglen)
            {            
                         // three byte message, what is it?
                case 3:
                         // if idle message, just bag it right now
                         if (rawbuff[0] == 0xff)
                            break;

                         rxaddress = rawbuff[0];               // match our address?
                         if (rxaddress != dccaddress)
                             break;                            // nope, skip
                             
                         ouraddress = TRUE;                    // yes, this message is for us
                         
                         if( (rawbuff[1] & 0x40) == 0x040)     // 28 Speed instruction?
                         {
                             dccspeed = stepTable[rawbuff[1] & 0x1f] * 4;
                             if (rawbuff[1] & 0x20)
                                   direction = FORWARD;        // translate the 28 step to 128
                                else                           // everything internal to this code is 128 now
                                   direction = REVERSE;
                                   
                             break;
                         }
                        
                         checkOurFunctionCodes();
                         
                      break;
                
      
                case 4: // Extended Packet?  Function or 128 throttle?
                
                        if (rawbuff[1] == 0x3f)           // must be 128 step mode throttle packet
                           {
                             rxaddress = rawbuff[0];      // match our address?
                             if (rxaddress != dccaddress)
                                break;                    // nope, bail
                        
                             ouraddress = TRUE;           // sent 128 step instruction, no action required
                             dccspeed = rawbuff[2];       // get speed and direction
                             if (dccspeed & 0x80)
                                direction = FORWARD;
                             else
                                direction = REVERSE;

                             dccspeed = dccspeed & 0x7f;  // remove the direction bit from the speed value
                             break;
                           }

                         // Next choice, perhaps it is a regular 28 speed packet with long address?
 
                         if( (rawbuff[2] & 0x40) == 0x40)     // 28 Speed instruction?
                           {
                             rxaddress = rawbuff[0] & 0x3f;   ///   **** 0x1f
                             rxaddress = rxaddress << 8;
                             rxaddress |= rawbuff[1];
                        
                             if (rxaddress != dccaddress)
                                break;

                             ouraddress = TRUE;               

                             dccspeed = stepTable[rawbuff[2] & 0x1f] * 4;
                             if (rawbuff[2] & 0x20)
                                   direction = FORWARD;
                                else
                                   direction = REVERSE;
                             break;
                           }                           
                           
                           // none of the above, last choice is long address function code
                           rxaddress = rawbuff[0] & 0x3f;  //0x1f
                           rxaddress = rxaddress << 8;
                           rxaddress |= rawbuff[1];
                             
                           if (rxaddress != dccaddress)
                             break;
                                                      
                           checkOurFunctionCodes();
   
                           break;
                         
                case 5:                           // long address
                        if(rawbuff[2] == 0x3f)   // 128 step mode throttle packet?
                          {
                             rxaddress = rawbuff[0] & 0x3f;    ///// 0x1f
                             rxaddress = rxaddress << 8;
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
                          
                          // config?
                          if (rawbuff[1] == 0xec)     // Configuration write?
                          {
                             rxaddress = rawbuff[0];  // short address
                             if (rxaddress != dccaddress)
                               break;
                              
                             checkConfigurationCode(rawbuff[2], rawbuff[3]);
                          }
                          break;

                case 6:
                          // config?
                          if (rawbuff[2] == 0xec)     // Configuration write?
                          {
                             rxaddress = rawbuff[0] & 0x3f;        // long address
                             rxaddress = rxaddress << 8;
                             rxaddress |= rawbuff[1];
                             if (rxaddress != dccaddress)
                               break;

                             checkConfigurationCode(rawbuff[3], rawbuff[4]);
                          }
                          break;
            }
            
            /* message for our DCC address came in, process it */
            
            if(ouraddress)
            {
                switch(servomode)    // coupler mode is handled in function codes
                {
                    case STEAM:                             // Steam mode?
                            if (direction == FORWARD)       // for live steam, servo 1 is direction
                               setServoPulse(1,servohigh1); // high limit is forward
                            else
                               setServoPulse(1,servolow1);  // low limit is reverse
                    
                            s = dccspeed * 10;              // 128 max steps is a bit over servo max of 1000
                            if(s>1000) s = 1000;            // since channel 0 servo is also throttle via servo or ESC, set it same as DCC
                            if (s<0) s = 0;                 // make sure we don't exceed limits
                            setServoPulse(0,s);             // send throttle value to servo
                    break;
                    
                    case ESC:                               // Center off ESC mode
                            if (direction == FORWARD)
                            {
                               s = dccspeed;
                               s = (s*4) + 500;
                               if (s>1000) s = 1000;        // since channel 0 servo is also throttle via servo or ESC, set it same as DCC
                               if (s<0) s = 0;              // make sure we don't exceed limits
                               setServoPulse(0,s);          // send throttle value to servo
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
                    
                    case PWM:
					        if (dccspeed == 0)              // only switch directions if stopped
							{
                               if (direction == FORWARD)
							   {
                                  PORTA |= 0x08;
							   }
                               else
							   {
                                  PORTA &= ~0x08;
							   }
							}
							
                            setPWM(dccspeed);
                    break;
                }
            }      
            
            /* end of DCC message process, clear flag and wait for another */
            flagbyte = 0;
            
        }
    }
}

