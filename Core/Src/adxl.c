#include "adxl.h"
#include "stdint.h"
#include "stm32f4xx_it.h"

#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"

volatile I2C_State_t currentState = STATE_IDLE;

volatile int16_t x_raw;
volatile int16_t y_raw;
volatile int16_t z_raw;

volatile uint8_t data_buffer[6];
volatile uint8_t count = 0;

void I2C1_EV_IRQHandler(void){
	// read SR1 register to capture snapshot of the hardware
	uint32_t sr1 = I2C1->SR1;

	switch (currentState) {
		case STATE_START_SENT:
			if (sr1 & (1U << 0)){
				// write the address to SDA bus via data register
				I2C1 -> DR = (0x53 << 1);
				currentState = STATE_ADDR_SENT;
			}
			break;
		case STATE_ADDR_SENT:
			if (sr1 & (1U << 1)){
				 // Perform reads on the status registers to reset ADDR bit
				(void)I2C1 -> SR1;
				(void)I2C1 -> SR2;

				I2C1 -> DR = (0x2D);
				currentState = STATE_POWER_CTL_WRITE;

			}
			break;
		case STATE_POWER_CTL_WRITE:
			if (sr1 & (1U << 7)){

				// IMMEDIATELY send what you want to write for the register
				I2C1 -> DR = (0x08);

				currentState = STATE_INIT_READ;
			}
			break;
		case STATE_INIT_READ:
			if (sr1 & (1U << 7)){
				I2C1 -> CR1 |= (1U << 9);
				// Start reading the the ADXL345 data registers
				I2C1 -> CR1 |= (1U << 8);

				currentState = STATE_INIT_READ_SB;

			}
			break;
		case STATE_INIT_READ_SB:
			if (sr1 & (1U << 0)){

				I2C1 -> DR = (0x53 << 1);

				currentState = STATE_INIT_READ_ADDR;
			}
			break;
		case STATE_INIT_READ_ADDR:
			if (sr1 & (1U << 1)){
				// Perform reads on the status registers to reset ADDR bit
				(void)I2C1 -> SR1;
				(void)I2C1 -> SR2;

				I2C1 -> DR = (0x32);
				currentState = STATE_INIT_READ_BTF;
			}
			break;
		case STATE_INIT_READ_BTF:
			if (sr1 & (1U << 2)){
				// Repeated start to change to receiver mode
				I2C1 -> CR1 |= (1U << 8);

				currentState =  STATE_REPEATED_READ_SB;
			}
			break;
		case STATE_REPEATED_READ_SB:
			if (sr1 & (1U << 0)){
				I2C1 -> DR = (0x53 << 1) | 1U;

				currentState = STATE_REPEATED_READ_ADDR;
			}
			break;
		case STATE_REPEATED_READ_ADDR:
			if (sr1 & (1U << 1)){
				I2C1 -> CR1 |= (1U << 10);
				// ACK is still enabled here because we want 6 bytes in total
				// Perform reads on the status registers to reset ADDR bit
				(void)I2C1 -> SR1;
				(void)I2C1 -> SR2;

				currentState = STATE_RECEIVING;
			}
			break;
		case STATE_RECEIVING:
			if (sr1 & (1U << 6)) {
				data_buffer[count] = I2C1 -> DR;
				count++;

				if (count == 5){
					I2C1 -> CR1 &= ~(1U << 10);
					I2C1 -> CR1 |= (1U << 9);
				}
				else if (count == 6){
					// Set ACK to 1 again to return to original state
					I2C1 -> CR1 |= (1U << 10);

					// Combine the data
					x_raw = (int16_t)((data_buffer[1] << 8) | (data_buffer[0]));
					y_raw = (int16_t)((data_buffer[3] << 8) | (data_buffer[2]));
					z_raw = (int16_t)((data_buffer[5] << 8) | (data_buffer[4]));


					// reset counter for next reading
					count = 0;
					currentState = STATE_COMPLETE;
				}
			}
			break;
	}
}

void I2C1_ER_IRQHandler(void){
	// Release the I2C lines before performing software reset
	I2C1 -> CR1 |= (1U << 9);

	// Software reset
	I2C1 -> CR1 |= (1U << 15);
	I2C1 -> CR1 &= ~(1U << 15);

	// Initialize I2C
	I2C_init();

	// Re-enable ITEVTEN and ITBUFEN
	I2C1->CR2 |= (1U << 9) | (1U << 10);

	count = 0;
	currentState = STATE_IDLE;
}

void TimerStart(){
	// Set to 500 miliseconds
	SysTick->LOAD = 16000 * 500 - 1; // cycles per MS = 16000

	// Reset the clock value
	SysTick-> VAL = 0;

	// Dummy read to clear previous flags
	(void)SysTick->CTRL;

	// Enable and set clock source to processor clock
	SysTick -> CTRL = (1U) | (1U << 2);
}

uint8_t I2C_init(void){
	// clock for B port pins
	RCC-> AHB1ENR |= (1U << 1);
	// Enable I2C1 on APB1
	RCC -> APB1ENR |= (1U << 21);

	// Software Reset to ensure I2C peripheral starts in a known state
	I2C1->CR1 |= (1U << 15);  // Set SWRST bit
	I2C1->CR1 &= ~(1U << 15); // Clear SWRST bit

	// Setup for PB8 as SCL and PB9 as SDA
	GPIOB -> MODER &= ~((3U << 16) | (3U << 18)); // Clear MODER8 and MODER9
	GPIOB -> MODER |=  ((2U << 16) | (2U << 18)); // Set both to Alternate Function (10)

	// Clear and set AFR[1] (pins 8 to 15)
	GPIOB -> AFR[1] &= ~((15U << 0) | (15U << 4)); // Clear AFSR8 and AFSR9
	GPIOB -> AFR[1] |=  ((4U << 0)  | (4U << 4));  // Set both to AF4 (0100)

	// Configure Output Type to Open Drain
	GPIOB -> OTYPER |= (1U << 8) | (1U << 9);

	// Configure Pull-ups
	GPIOB -> PUPDR &= ~((3U << 16) | (3U << 18));
	GPIOB -> PUPDR |=  ((1U << 16) | (1U << 18));

	// I2C Setup
	// Frequency bit for 16 MHz
	I2C1 -> CR2 = 16U;

	// in standard mode: 16 MHz / 2 * 1 KHz = 80
	I2C1 -> CCR = 80U;

	// TRISE = (1000ns / 62.5ns) + 1 = 17
	I2C1 -> TRISE = 17U;

	// Enable peripheral and set start bit to 1
	I2C1 -> CR1 |= 1U;

	return 0;
}


void ADXL345_StartRead(){
	// The hardware enters Controller (Master) Mode
	I2C1 -> CR1 |= (1U << 8);
	currentState = STATE_START_SENT;
}
