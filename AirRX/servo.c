/*
 * ServoLibrary.c
 *
 * Created: 11/3/2017 7:22:26 PM
 * Author : martan
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
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

 #define ONEMS  	  1000	        /* ONE MS 16mhz */
 #define NEXTSCAN	  16*ONEMS      /* 16 ms */
 #define ENDSCAN      10*ONEMS      /* time to wait until next servo */
 #define SERVOSOFF    0xf8          /* bottom three pins are the 3 servo outputs */
 
 #define USTIMER      0x01          /* OVF overflow vector - uS timer */
 #define SERVOTIMER   0x02          /* COMPA vector - servos */
 #define COMPAREB     0x04          /* COMPB vector - update accel/decl for servo 0 */

 #define SERVO0       0x01          /* define servo outputs port bits */
 #define SERVO1       0x02
 #define SERVO2       0x04

/* Handle Three Servo outputs on PORTB bits 0-2 */

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

static volatile uint16_t msClockHigh;
static volatile uint32_t msUpper;

uint16_t normalMs = 1000;

/* starts both the servo cycle and the master 1us clock source */

void initServoTimer(void)
{
     msClockHigh = 0;
     msUpper     = 0;

     DDRA = 0x0f;
    
     servostate = SERVOIDLE;
     
     OCR1A = NEXTSCAN;			// This starts the servo scan.
     OCR1B = normalMs;

     TCCR1A = 0;
     TCCR1B = 0;
     TCNT1  = 0;

     TCCR1B |= 0x02;			                     // clock select, divide sysclock by 8
     TIMSK1 |= USTIMER | SERVOTIMER | COMPAREB;      // enable interrupts 

     // Overflow and OCR interrupts, let timer run until overflow, keep track of upper word in s/w 
}


/* Master Clock Source                                  */
/* ------------------------ 64 bits of microseconds --- */
/*   msUpper 32bits   MsClockHigh 16bits  TCLK 16bits   */
/*   0000 0000        0000                0000          */
/*                                                      */
/*   292471 years before it rolls over                  */

ISR(TIM1_OVF_vect)
{
    msClockHigh++;
    if(msClockHigh == 0)
       msUpper++;
}

ISR(TIM1_COMPB_vect)
{
    OCR1B = normalMs + TCNT1;     // one ms from where we are
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
                pulse = pulse * 2;    // 16Mhz, double the time
                ServoPulseMs0 = pulse + ONEMS;
        break;

        case 1: if(pulse < ServoLowLimit1)  pulse = ServoLowLimit1;
                if(pulse > ServoHighLimit1) pulse = ServoHighLimit1;
				if(servo1Dir) pulse = 1000 - pulse;
                pulse = pulse * 2;
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
  setEEServoReverse(sn, direction);
}


