/*
 * ServoLibrary.c
 *
 * Created: 11/3/2017 7:22:26 PM
 * Author : martan
 
 * Moved PWM output to outputX and output Y to preserve the servo functionality 2024
 * added sense of 1 (lowest speed) flatlines the motor driver output.
 
 
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include "spi.h"
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
 
 #define OUTPUTX      0x04
 #define OUTPUTY      0x08

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

static volatile uint8_t pwmmode = CYTRON;
static volatile uint8_t pwmOutput = OUTPUTX;

 // set up the clock so it runs at 1us per tick

static volatile uint16_t msClock;
static volatile uint16_t msUpper;

uint16_t normalMs = ONEMS;

/* starts the PWM output */

void initPWM(uint8_t mode)
{
    TCCR0A = 0;     // take the defaults
    TCCR0B = 0x02;  // clock divide by 8
    OCR0B  = 100;   // compare register
    TIMSK0 |= 0x04;  // enable timer compare B  (A is used by softuart)

    // in Cytron mode, only the output X is used for pulses, OUTPUTY is high or low for direction
	// for the DRV8817, OUTPUTX pulses are used for forward, OUTPUTY pulses are for backwards
	// default it to X in either case

    pwmOutput = OUTPUTX;
	pwmmode = mode;
}

uint8_t wavdir = 0;
uint8_t pwmHigh = 2;
uint8_t pwmLow  = 250;

/* PWM Output if configured */

ISR(TIM0_COMPB_vect)
{
	
	if(pwmHigh <= 2)   /* slow as we can go, just make the output flatline low to prevent creep*/
	{
		PORTA &= ~pwmOutput;
		return;
	}
	
    wavdir ^= 1;
    if (wavdir)
    {
        OCR0B = pwmHigh;
        PORTA |= pwmOutput;
    }        
    else
    {
        OCR0B = pwmLow;
        PORTA &= ~pwmOutput;
    }           
    TCNT0 = 0;
    
}



/* pwm low and high */

void setPWM(uint8_t direction, uint8_t pw)
{
     if (pw == 0)              // only switch directions if stopped
        {
			switch(pwmmode)
			{
			  case CYTRON:
					if (direction == FORWARD)
					{
						PORTA |= OUTPUTY;
					}
					else
					{
						PORTA &= ~OUTPUTY;
					}
					break;
					
			 case DRV8871:
					if (direction == FORWARD)
					{
						pwmOutput = OUTPUTX;
					}
					else
					{
						pwmOutput = OUTPUTY;
					}
					break;
			}
		}
	
	pw = pw*2;
	
    if (pw>250) pw = 250;
	
    pwmHigh = pw;
    pwmLow  = 255 - pw;
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


