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


/*
 *
 *   Main Loop. Decode message packets here and do servos
 *
 *
 */

int main(void)
{
    
    DDRB |= 0x03;    // PB0, PB1 = outputs
        
    dccInit();
    initServoTimer();
    initializeSPI();
    startModem(0);
    
    sei();                                      // enable interrupts
    
   while (1)
    {
        
        /* nothing here right now */
        
        /* dcc message come in? */
        /* Throttle message?
             do servo stuff
           CV message?
             for us?
                save stuff
         */
    }
}

