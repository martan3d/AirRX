/*
 * servotimer.h
 *
 * Created: 11/3/2017 7:24:28 PM
 *  Author: martan
 */ 


#ifndef SERVOTIMER_H_
#define SERVOTIMER_H_

#define F_CPU  8000000

void initServoTimer(uint8_t mode);
void setServoPulse(uint8_t i, int16_t pulse);
void setServoLow(uint8_t sn, int16_t lo);
void setServoHigh(uint8_t sn, int16_t hi);
void setServoReverseValue(uint8_t sn, uint8_t direction);
uint32_t getMSClock();
void setPWM(uint8_t pw, uint8_t dir);
void initPWM(void);


#endif /* SERVOTIMER_H_ */