# AirRX
Attiny84A Airwire RX implementation

Updated July 2024 - made public July 2025

spi.c can be configured for the following radio modules:
- Anaren CC1101 daughterboard
- Darrell's custom CC1101 clone board (I have some of these, just ask)
- ElecHome board

- DCC output via Airwire
- Two servos can be controlled, servo 0 follows the DCC throttle for live steam applications

Also has PWM output if you don't want to use DCC, it works with:
- Cytron
- Adafruit DRV8871

Build this with Microchip Studio, I use the Pololu Atmega Programmer

PCB is the .bac file, use with Bay Area Circuits free PCB designer.

https://bayareacircuits.com/



