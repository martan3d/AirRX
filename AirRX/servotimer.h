/*
 * servotimer.h
 *
 * Created: 11/3/2017 7:24:28 PM
 *  Author: martan
 */ 


#ifndef SERVOTIMER_H_
#define SERVOTIMER_H_

#define F_CPU  8000000

#define FORWARD 0
#define REVERSE 1

#define CYTRON    4
#define DRV8871   5

//#define PWMMODE CYTRON
#define PWMMODE DRV8871

void initServoTimer(uint8_t mode);
void setServoPulse(uint8_t i, int16_t pulse);
void setServoLow(uint8_t sn, int16_t lo);
void setServoHigh(uint8_t sn, int16_t hi);
void setServoReverseValue(uint8_t sn, uint8_t direction);
uint32_t getMSClock();
void setPWM(uint8_t direction, uint8_t pw);
void initPWM(uint8_t mode);


#endif /* SERVOTIMER_H_ */