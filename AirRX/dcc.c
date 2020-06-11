/*
dcc.cpp

Created: 10/11/2015 8:43:49 AM
Copyright (c) 2018-2019, Martin Sant
All rights reserved.

*/

#include <avr/io.h> 
#include <avr/interrupt.h>
#include <string.h>
#include "servotimer.h"
#include "dcc.h"

#define PREAMBLE 0
#define START_BIT 1
#define DATA 2
#define END_BIT 3

// Any variables that are used between the ISR and other functions are declared volatile
unsigned int i;
int16_t usec;
int16_t dnow;
uint8_t BitCount;
uint8_t  State;
uint8_t dataByte;
uint8_t byteCounter;
uint8_t buffer[sizeof(DCC_MSG)+1];
uint8_t DccBitVal = 0;
uint8_t errorByte = 0;
uint8_t dccbuff[sizeof(DCC_MSG)];

uint8_t * getDCC()
{
     return(dccbuff);
}

void dccInit(void)
{
  State  = PREAMBLE;        // Pin change interrupt
  DDRB   &= ~(0b00001000);  // Pin PB2 is input
  MCUCR = 0;
  MCUCR |= (1<<ISC01);      // interupt on any logical change on pin INT0
  GIMSK = (1<<INT0);        // EXT INT 0 enabled only
}


ISR(EXT_INT0_vect)
{
    /** PORTB 2 is EXT IRQ 0 on Attiny 24/44/84 - INPUT FOR DCC from cc1101 
	    EXT INT0 gives us an IRQ on both a high and a low change of the input stream */
	
    PORTB ^= 1;
    
    if(PINB & 0x08)                         // if it's a one, start of pulse
    {                                       // so, we need to
        usec = TCNT1;                       //   snag the current time in microseconds
        return;                             // and that's all we need, exit
    }    
    else                                    // else we are at the end of the pulse, on the downside
    {                                       // how long was the pulse?
        dnow = TCNT1 - usec;                // Get the now time minus the start time gives pulse width in us
                                            // Longer pulse is a zero, short is one
        if ( dnow > 70 )
            DccBitVal = 0;                  // Longer is a zero
        else 
            DccBitVal = 1;                  // short means a one
    }
    

	/*** after we know if it's a one or a zero, drop through the state machine to see if we are where we think we are in the DCC stream */
    
    BitCount++;                             // Next bit through the state machine

    switch( State )
    {
        case PREAMBLE:                      // preamble is at least 11 bits, but can be more
            if( DccBitVal )
            {
                if( BitCount > 10 )         // once we are sure we have the preamble here, wait on the start bit (low)
                {
                    State = START_BIT;      // Off to the next state
                    buffer[0] = 0;          // brute force clear, hopefully fast
                    buffer[1] = 0;
                    buffer[2] = 0;
                    buffer[3] = 0;
                    buffer[4] = 0;
                    buffer[5] = 0;
                    buffer[6] = 0;
                    buffer[7] = 0;
                }                
            }            
            else
                BitCount = 0 ;              // otherwise sit here and wait on the preamble
        break;
        
        //--------------------------------- Pramble Finished, wait on the start bit

        case START_BIT:                     // preamble at least almost done, wait for the start of the data
             if(!DccBitVal)
             {
                 BitCount = 0;
                 byteCounter = 0;
                 State = DATA;              // soon as we have it, next state, collect the bits for the data bytes
                 dataByte = 0;
             }             
        break;
             
        //---------------------------------- Save the data, clock them into a byte one bit at a time

        case DATA:
            dataByte = (dataByte << 1);
            if(DccBitVal)
                dataByte |= 1;
                
            if(BitCount == 8)
            {
              buffer[byteCounter++] = dataByte;
              dataByte = 0;
              State = END_BIT;
            }            
        break;
        
        //--------------------------------- All done, figure out what to do with the data, we now have a complete message

        case END_BIT:
            if( DccBitVal ) // End of packet?
            {
                State = PREAMBLE;                           // Got everything, next time will be start of new DCC packet coming in
                
                if (byteCounter == 3)                       // see what sort of packet we have, how long is it?
                {                                           // Do error checking here
                    errorByte  = buffer[0];                 // VERY IMPORTANT!
                    errorByte ^= buffer[1];                 // All sorts of stuff flies around on the bus
                    
                    if (errorByte == buffer[2])
                    {
                        buffer[5] = byteCounter;            // save length
                        buffer[6] = 0;

                        for (i=0;i<sizeof(DCC_MSG);i++)     // Move message to buffer for background task
                            dccbuff[i] = buffer[i];

                        //setScheduledTask(TASK1);            // Schedule the background task
                    }
                  break;
                }
                
                if (byteCounter == 4)                       // Four bytes of DCC message
                {                                           // Error checking here
                    errorByte  = buffer[0];                 // XOR across all 
                    errorByte ^= buffer[1];
                    errorByte ^= buffer[2];                 // be VERY picky about what we accept
                    
                    if (errorByte == buffer[3])
                        {
                            buffer[5] = byteCounter;        // save length
                            buffer[6] = 0;
                                                            // move out of operations buffer into background buffer
                            for (i=0;i<sizeof(DCC_MSG);i++)
                              dccbuff[i] = buffer[i];

                            //setScheduledTask(TASK1);        // Schedule the background task
                        }                        
                   break;
                }                
                
                if (byteCounter == 5)						// Five Bytes
                {
                    errorByte  = buffer[0];                 // Compute checksum (xor) across all
                    errorByte ^= buffer[1];
                    errorByte ^= buffer[2];
                    errorByte ^= buffer[3];

                    if (errorByte == buffer[4])             // if it matches, valid message came in
                    {
                        buffer[5] = byteCounter;        	// save length
                        buffer[6] = 0;

                        for (i=0;i<sizeof(DCC_MSG);i++)
                            dccbuff[i] = buffer[i];

                        //setScheduledTask(TASK1);            // Schedule the background task
                    }
                  break;
                }

            }
            else  // Get next Byte
                State = DATA ;

            BitCount = 0 ;

        break;

    }
}

/*

uint8_t rawbuff[7];

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
*/