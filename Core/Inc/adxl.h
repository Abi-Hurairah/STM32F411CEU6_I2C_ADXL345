#include "stdint.h"

#ifndef ADXL_H
#define ADXL_H

typedef enum {
	STATE_IDLE,
	STATE_START_SENT,
	STATE_ADDR_SENT,
	STATE_POWER_CTL_WRITE,
	STATE_INIT_READ,
	STATE_INIT_READ_SB,
	STATE_INIT_READ_ADDR,
	STATE_INIT_READ_BTF,
	STATE_REPEATED_READ_SB,
	STATE_REPEATED_READ_ADDR,
	STATE_RECEIVING,
	STATE_COMPLETE,
	STATE_DELAY,
	STATE_ERROR
} I2C_State_t;

extern volatile I2C_State_t currentState;

void TimerStart();
uint8_t I2C_init(void);

void ADXL345_StartRead();

#endif
