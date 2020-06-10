/*
 * spi.h
 *
 * Created: 12/2/2018 9:33:20 AM
 *  Author: marta
 */ 


#ifndef SPI_H_
#define SPI_H_

#define SPI_DDR_PORT DDRA
#define USCK_DD_PIN DDA4
#define DO_DD_PIN DDA5
#define DI_DD_PIN DDA6
#define SPI_LATCH 0x80

void initializeSPI();
void startModem(uint8_t channel);


#endif /* SPI_H_ */