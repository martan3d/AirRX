/*
 * AirRX.c
 *
 * Created: 12/10/2018 8:19:10 PM
 * Author : marta
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer1.h"
#include "spi.h"

#define BACKGROUNDTIME 960

int64_t now;
int64_t then;

int main(void)
{
    
    initTimer1();
   
    DDRB = 0x03;                                // Port B, 0,1 as debug outputs

    PORTB &= 0xfe;
    PORTB |= 1;    

    initializeSPI();
    
    startModem(0);
    
    sei();                                      // enable interrupts
    
    then = getMsClock();                        // Grab Current Clock, run the loop at 500ms intervals

    while (1)
    {
        now = getMsClock() - then;              // throttle the background loop
        
        if ( now > BACKGROUNDTIME )             // Run on Schedule, this is lower priority
        {                                       // First part of loop runs only on scheduled time
            then = getMsClock();                // Grab Clock Value for next time (if we were going to use it)
            
            //PORTB ^= 1;                         // debug!!
            
        }
    }
}

