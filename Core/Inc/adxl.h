#include "stdint.h"

#ifndef ADXL_H
#define ADXL_H

void TimerStart();
uint8_t I2C_init(void);
uint8_t ADXL345_pwr(void);
uint8_t ADXL345_read();

#endif
