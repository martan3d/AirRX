/*
 * eedata.h
 *
 * Created: 6/11/2020 12:47:59 PM
 *  Author: marta
 */ 


#ifndef EEDATA_H_
#define EEDATA_H_

uint8_t getEEAirwireChannel();
void setEEAirwireChannel(uint8_t mode);
uint16_t getEEDCCAddress();
void setEEDCCAddress(int16_t addr);
uint16_t getEEServoHi( uint8_t sn);
void setEEServoHi(uint8_t sn, uint16_t addr);
uint16_t getEEServoLow( uint8_t sn);
void setEEServoLow(uint8_t sn, uint16_t addr);
uint8_t getEEServoReverse(uint8_t sn, uint8_t mode);
void setEEServoReverse(uint8_t sn, uint8_t mode);

#endif /* EEDATA_H_ */