/*
 * ServoLibrary.c
 *
 * Created: 11/3/2017 7:22:26 PM
 * Author : martan
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include "servotimer.h" 
#include "eedata.h"


 // servo states
 #define SERVOIDLE    00
 #define STARTSERVO0  11
 #define WAITSERVO0   12
 #define STARTSERVO1  14
 #define WAITSERVO1   15
 #define STARTSERVO2  17
 #define WAITSERVO2   18
 #define ENDSERVO     22

 #define ONEMS  	  1018	        /* ONE MS 8mhz */
 #define NEXTSCAN	  16*ONEMS      /* 16 ms */
 #define ENDSCAN      10*ONEMS      /* time to wait until next servo */
 #define SERVOSOFF    0xfc          /* bottom three pins are the 3 servo outputs */
 
 #define USTIMER      0x01          /* OVF overflow vector - uS timer */
 #define SERVOTIMER   0x02          /* COMPA vector - servos */
 #define COMPAREB     0x04          /* COMPB vector - update accel/decl for servo 0 */

 #define SERVO0       0x01          /* define servo outputs port bits */
 #define SERVO1       0x02

/* Handle Three Servo outputs on PORTA bits 0-2 */

static volatile int16_t ServoPulseMs0 = ONEMS;      /* Actual values being clocked out: 1000 min, 2000 max */
static volatile int16_t ServoPulseMs1 = ONEMS;

static volatile int16_t ServoHighLimit0 = ONEMS;    /* ADDED to Above for 2000 total */
static volatile int16_t ServoHighLimit1 = ONEMS;

static volatile int16_t ServoLowLimit0 = 0;         /* ADDED to Above for 1000 total */
static volatile int16_t ServoLowLimit1 = 0;

static volatile uint8_t servo0Dir = 0;
static volatile uint8_t servo1Dir = 0;

static volatile uint8_t servostate;

 // set up the clock so it runs at 1us per tick

static volatile uint16_t msClock;
static volatile uint16_t msUpper;

uint16_t normalMs = ONEMS;


const uint8_t pulseLo[] PROGMEM = {255, 255, 255, 255, 255, 245, 245, 245, 245, 245, 235, 235, 235, 235, 235, 225, 225, 225, 225, 225, 215, 215, 215, 215, 215, 205, 205, 205, 205, 205, 195, 195, 195, 195, 195, 185, 185, 185, 185, 185, 175, 175, 175, 175, 175, 165, 165, 165, 165, 165, 155, 155, 155, 155, 155, 145, 145, 145, 145, 145, 135, 135, 135, 135, 135, 125, 125, 125, 125, 125, 115, 115, 115, 115, 115, 105, 105, 105, 105, 105, 95, 95, 95, 95, 95, 85, 85, 85, 85, 85, 75, 75, 75, 75, 75, 65, 65, 65, 65, 65, 55, 55, 55, 55, 55, 45, 45, 45, 45, 45, 35, 35, 35, 35, 35, 25, 25, 25, 25, 25, 15, 15, 15, 15, 15, 5, 5, 5};
const uint8_t pulseHi[] PROGMEM = {100, 100, 100, 100, 100, 106, 106, 106, 106, 106, 112, 112, 112, 112, 112, 118, 118, 118, 118, 118, 124, 124, 124, 124, 124, 130, 130, 130, 130, 130, 136, 136, 136, 136, 136, 142, 142, 142, 142, 142, 148, 148, 148, 148, 148, 154, 154, 154, 154, 154, 160, 160, 160, 160, 160, 166, 166, 166, 166, 166, 172, 172, 172, 172, 172, 178, 178, 178, 178, 178, 184, 184, 184, 184, 184, 190, 190, 190, 190, 190, 196, 196, 196, 196, 196, 202, 202, 202, 202, 202, 208, 208, 208, 208, 208, 214, 214, 214, 214, 214, 220, 220, 220, 220, 220, 226, 226, 226, 226, 226, 232, 232, 232, 232, 232, 238, 238, 238, 238, 238, 244, 244, 244, 244, 244, 250, 250, 250};



/* starts the PWM output */

void initPWM(void)
{
    TCCR0A = 0;       // take the defaults
    TCCR0B = 0x02;    // clock divide
    OCR0B  = 200;     // compare register
    TIMSK0 |= 0x04;   // enable timer IRQ
}

volatile uint8_t wavdir = 1;
volatile uint8_t pwmHi =  250;
volatile uint8_t pwmLo  = 1;
volatile uint8_t pwmdir = 1;

/* PWM Output on Servo 0 if configured */

#define PWMMAX 160

ISR(TIM0_COMPB_vect)
{
    if (wavdir)
    {
        if(pwmdir)
           PORTA |= SERVO0;
        else
           PORTA |= SERVO1;
           
        OCR0B = pwmHi;
        wavdir = 0;
    }        
    else
    {
        if(pwmdir)
          PORTA &= ~SERVO0;
          else
          PORTA &= ~SERVO1;
          
        OCR0B = pwmLo;
        wavdir = 1;
    }        
        
    TCNT0 = 0;
}

/* pwm low and high */

/* 250 max hi
     5 lowest lo
     
*/


#define OFS 2
void setPWM(uint8_t pw, uint8_t dir)
{
    pw = pw & 0x7f;
    pwmdir = dir;
    
    pw++;
    
    pwmHi =  pgm_read_byte(&pulseHi[pw]);
    pwmLo =  pgm_read_byte(&pulseLo[pw]);
  
  //  pwmHi = 10;
  //  pwmLo = 200;
      
    if (pwmdir)
        PORTA &= ~SERVO1;
    else
        PORTA &= ~SERVO0;
}

/* starts both the servo cycle and the master 1ms clock source */

void initServoTimer(uint8_t mode)
{
    if (mode) mode = SERVOTIMER;
    
     msClock = 0;
     msUpper = 0;

     DDRA = 0x0f;
    
     servostate = SERVOIDLE;
     
     OCR1A = NEXTSCAN;			// This starts the servo scan.
     OCR1B = normalMs;

     TCCR1A = 0;
     TCCR1B = 0;
     TCNT1  = 0;

     TCCR1B |= 0x02;			               // clock select, divide sysclock by 8
     TIMSK1 |= mode | COMPAREB;                // enable timer1 A & B interrupts, A is servo, B is millisecond clock
}


/* Master Clock Source                            */
/* ------------ 32 bits of milliseconds --------- */
/*   msUpper 16bits   MsClockHigh 16bits          */
/*           0000                 0000            */


ISR(TIM1_COMPB_vect)
{
    OCR1B = normalMs + TCNT1;     // one ms from where we are
    
    msClock++;
    if(msClock == 0)
       msUpper++;
}

uint32_t clockVal;

uint32_t getMSClock()
{
    clockVal = msUpper;
    clockVal = clockVal << 16;
    clockVal |= msClock;
    return clockVal;
}

/* handle each servo one at a time */

ISR(TIM1_COMPA_vect)
{
    switch(servostate)
    {
        case SERVOIDLE:
        PORTA &= SERVOSOFF;
        OCR1A += ONEMS;
        servostate = STARTSERVO0;
        break;
        
        case STARTSERVO0:
        PORTA |= SERVO0;
        OCR1A += ServoPulseMs0;
        servostate = WAITSERVO0;
        break;

        case WAITSERVO0:
        PORTA &= ~(SERVO0);
        OCR1A += ONEMS;
        servostate = STARTSERVO1;
        break;
        
        case STARTSERVO1:
        PORTA |= SERVO1;
        OCR1A += ServoPulseMs1;
        servostate = ENDSERVO;
        break;
        
        case ENDSERVO:
        PORTA &= ~(SERVO1);
        OCR1A += ENDSCAN;
        servostate = SERVOIDLE;
        break;
    }
}

void setServoPulse(uint8_t i, int16_t pulse)
{
    /* Global sanity check pulse value */

    if (pulse < 0 )
    return;

    if (pulse > 1000 )
    return;

    switch(i)
    {
        case 0: if(pulse < ServoLowLimit0)  pulse = ServoLowLimit0;
                if(pulse > ServoHighLimit0) pulse = ServoHighLimit0;
                if(servo0Dir) pulse = 1000 - pulse;
                ServoPulseMs0 = pulse + ONEMS;
        break;

        case 1: if(pulse < ServoLowLimit1)  pulse = ServoLowLimit1;
                if(pulse > ServoHighLimit1) pulse = ServoHighLimit1;
				if(servo1Dir) pulse = 1000 - pulse;
                ServoPulseMs1 = pulse + ONEMS;
        break;
    }
}


void setServoLow(uint8_t sn, int16_t lo)
{
    switch(sn)
    {
        case 0:
                if((lo >= 0) && (lo <= 1000))
                   {
	                 ServoLowLimit0 = lo;
                     setEEServoLow(0, ServoLowLimit0);
                   }
                break;
        case 1:
                if((lo >= 0) && (lo <= 1000))
                   {
	                  ServoLowLimit1 = lo;
                      setEEServoLow(1, ServoLowLimit1);
                   }
                   break;
    }                                         
}

void setServoHigh(uint8_t sn, int16_t hi)
{
     switch(sn)
     {
        case 0:
               if((hi >= 0) && (hi <= 1000))
               {
	              ServoHighLimit0 = hi;
                  setEEServoHi(0, ServoHighLimit0);
               }
               break;                 

        case 1:
               if((hi >= 0) && (hi <= 1000))
               {
                  ServoHighLimit1 = hi;
                  setEEServoHi(1, ServoHighLimit1);
               }
               break;
     }                                 
}


void setServoReverseValue(uint8_t sn, uint8_t direction)
{
  switch(sn)
  {
      case 0:
             servo0Dir = direction;
             break;
      case 1:
             servo1Dir = direction;
             break;
  }             
}


