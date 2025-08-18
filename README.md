# AirRX
Attiny84A Airwire RX implementation

Updated July 2024 - made public July 2025

spi.c can be configured for the following radio modules:
- Anaren CC1101 daughterboard
- Darrell's custom CC1101 clone board (I have some of these, just ask)
- ElecHome board
Range is generally about 100ft with the Anaren or the Custom boards

This implementation has the following features
- DCC output via Airwire
- Two servos can be controlled, servo 0 follows the DCC throttle for live steam applications
- programming of receiver parameters via CV commands

PWM output is available if you don't want to use DCC or ESC, it works with:
- Cytron
- Adafruit DRV8871

Build this with Microchip Studio, I use the Pololu Atmega Programmer

PCB is the .bac file, use with Bay Area Circuits free PCB designer.

https://bayareacircuits.com/

The following table shows the CV commands used to program the receiver

 * CV Map for AirRx
 *
 *   CV 201 - Radio Channel 0-15
 *   CV 202 - DCC Address lo
 *   CV 203 - DCC Address hi
 *   CV 204 - Servo Mode 0=Steam, 1=couplers, 2=ESC, 3=PWM
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
 *   CV 215 - Function Code for Coupler 0 (if in coupler mode)
 *   CV 216 - Function Code for Coupler 1
 *   CV 217 - Function Code for Output x
 *   CV 218 - Function Code for Output y
 *   CV 219 - On/Off Code for Output x (output port normally on/normally off)
 *   CV 220 - On/Off Code for Output y
 *   CV 230 - Reset to factory defaults (DCC address = 3, ESC servo mode)
 */



