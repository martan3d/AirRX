
/*
 * timer1.c
 *
 * Created: 11/18/2018 9:29:08 AM
 *  Author: martan
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer1.h"

/* timer */
 
#define USTIMER      0x01          /* OVF overflow vector - uS timer */
#define OCMPA        0x02          /* Compare A */
#define OCMPB        0x04

static volatile uint64_t msClock = 0;
static volatile uint16_t msClockHigh = 0;
static volatile uint32_t msUpper = 0;

uint16_t compareTimerA = 0;
uint16_t compareTimerB = 0;

// Timer OVF counter
uint16_t bigcounter = 0;
ISR(TIM1_OVF_vect)
{
	msClockHigh++;
	if(msClockHigh == 0)
	msUpper++;
}

#define INTERVALA 20000   // 10 Millisecond clock
ISR(TIM1_COMPA_vect)
{
     OCR1A = OCR1A + INTERVALA;
     compareTimerA ++;
     //PORTB ^= 1;
     
     if (compareTimerA > 50)    // 500ms gone by?
     {
       compareTimerA = 0;
     }     
     
/* EVERY 500 of these IRQs, set the CONTROL DATA FLAG to send data every half second */ 
}

#define INTERVALB 2000   // one Millisecond clock below
ISR(TIM1_COMPB_vect)
{
     OCR1A = OCR1A + INTERVALB;
     compareTimerB ++;
};


void initTimer1(void)
{
	msClock     = 0;
	msClockHigh = 0;
	msUpper     = 0;

	TCCR1A = 0;
	TCCR1C = 0;
	TCNT1  = 0;

	TCCR1B = 0x02;			              // clock select, divide sysclock by 8
	TIMSK1 = USTIMER | OCMPA | OCMPB;     // enable interrupts

	// Overflow and OCR interrupts, let timer run until overflow, keep track of upper word in s/w
}

uint64_t scratch;
uint64_t getMsClock()
{
	scratch = msUpper;
	scratch = (scratch << 32);
	msClock = msClockHigh;
	msClock = (msClock << 16);
	msClock |= TCNT1;
	msClock |= scratch;
	return msClock;
}