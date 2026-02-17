/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void TimerStart(){
	// Set to 1 milisecond
	SysTick->LOAD = 16000 * 500 - 1; // cycles per MS = 16000

	// Reset the clock value
	SysTick-> VAL = 0;

	// Enable and set clock source to processor clock
	SysTick -> CTRL |= (1U) | (1U << 2);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  //LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  //SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  // clock for B port pins
  RCC-> AHB1ENR |= (1U << 1);
  // Enable I2C1 on APB1
  RCC -> APB1ENR |= (1U << 21);

  // Software Reset to ensure I2C peripheral starts in a known state
  I2C1->CR1 |= (1U << 15);  // Set SWRST bit
  I2C1->CR1 &= ~(1U << 15); // Clear SWRST bit

  // Setup for PB6 as SCL and PB7 as SDA
  GPIOB -> MODER &= ~((3U << 12) | (3U << 14));
  GPIOB -> MODER |= (2U << 12) | (2U << 14);

  GPIOB -> AFR[0] &= ~(15U << 24);
  GPIOB -> AFR[0] |= (4U << 24);

  GPIOB -> AFR[0] &= ~(15U << 28);
  GPIOB -> AFR[0] |= (4U << 28);

  // set to open drain
  GPIOB -> OTYPER |= (1U << 6) | (1U << 7);

  // I2C Setup
  // Frequency bit for 16 MHz
  I2C1 -> CR2 = 16U;

  // in standard mode: 16 MHz / 2 * 1 KHz = 80
  I2C1 -> CCR = 80U;

  // TRISE = (1000ns / 62.5ns) + 1 = 17
  I2C1 -> TRISE = 17U;

  // Enable peripheral and set start bit to 1
  // The hardware enters Controller (Master) Mode
  I2C1 -> CR1 |= 1U;
  I2C1 -> CR1 |= (1U << 8);

  TimerStart();
  // Wait for SB flag
  while(!(I2C1->SR1 & (1U << 0))) {
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; //Error: I2C Timeout Occurred
	  }
      // Wait until SB is 1
  }

  // write the address to SDA bus via data register
  I2C1 -> DR = (0x53 << 1);


  // Implements 500 ms wait time before time out
  TimerStart();
  while(!(I2C1 -> SR1 & (1U << 1))){
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // Error: I2C Timeout Occurred
	  }
  }

  // Perform reads on the status registers to reset ADDR bit
  (void)I2C1 -> SR1;
  (void)I2C1 -> SR2;

  I2C1 -> DR = (0x2D);

  // Set delay for sending the bits
  // Makes sure data register is empty before proceeding
  while (!(I2C1-> SR1 & (1U << 7))){

  }

  // waiting for byte transfer finished state
  TimerStart();
  while (!(I2C1-> SR1 & (1U << 2))){
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // Error: I2C Timeout Occurred
	  }
  }

  // Repeated start to change to receiver mode
  I2C1 -> CR1 |= (1U << 8);

  // Wait for SB flag
  while(!(I2C1->SR1 & (1U << 0))) {
      // Wait until SB is 1
  }

  I2C1 -> DR = (0x53 << 1) | 1U;

  // check for ADDR register
  TimerStart();
  while(!(I2C1 -> SR1 & (1U << 1))){
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // Error: I2C Timeout Occurred
	  }
  }

  // Set NACK to stop the sensor from sending information
  I2C1 -> CR1 &= ~(1U << 10);

  // Perform reads on the status registers to reset ADDR bit
  (void)I2C1 -> SR1;
  (void)I2C1 -> SR2;


  I2C1 -> CR1 |= (1U << 9); // prepare stop

  // Check if the data register is not empty
  TimerStart();
  while(!(I2C1 -> SR1 & (1U << 6))){
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // Error: I2C Timeout Occurred
	  }
  }

  uint8_t devpwr = I2C1 -> DR;

  // Set ACK to 1 again to return to original state
  I2C1 -> CR1 |= (1U << 10);

  // Now we get to powerctl and control the ADXL345

  // Start again, since we issued a stop previously
  I2C1 -> CR1 |= (1U << 8);

  TimerStart();
  // Wait for SB flag
  while(!(I2C1->SR1 & (1U << 0))) {
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // Error: I2C Timeout Occurred
	  }
      // Wait until SB is 1
  }

  I2C1 -> DR = (0x53 << 1);

  // Implements 500 ms wait time before time out
  TimerStart();
  while(!(I2C1 -> SR1 & (1U << 1))){
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // // Error: I2C Timeout Occurred
	  }
  }

  // Perform reads on the status registers to clear ADDR bit
  (void)I2C1 -> SR1;
  (void)I2C1 -> SR2;


  I2C1 -> DR = (0x2D);

  // Set delay for sending the bits
  // Makes sure data register is empty before proceeding
  while (!(I2C1-> SR1 & (1U << 7))){

  }

  // IMMEDIATELY send what you want to write for the register
  I2C1 -> DR = (0x08);

  // Set delay for sending the bits
  // Makes sure data register is empty before proceeding
  while (!(I2C1-> SR1 & (1U << 7))){

  }

  // wait  for byte transfer finished
  TimerStart();
  while (!(I2C1 -> SR1 & (1U << 2))){
	  if(SysTick -> CTRL & (1U << 16)){
		  return 1;
	  }
  }

  I2C1 -> CR1 |= (1U << 9); // prepare stop

  // Start reading the the ADXL345 data registers

  I2C1 -> CR1 |= (1U << 8);

  TimerStart();
  // Wait for SB flag
  while(!(I2C1->SR1 & (1U << 0))) {
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // Error: I2C Timeout Occurred
	  }
      // Wait until SB is 1
  }

  // write the address to SDA bus via data register
  I2C1 -> DR = (0x53 << 1);


  // Implements 500 ms wait time before time out
  TimerStart();
  while(!(I2C1 -> SR1 & (1U << 1))){
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // Error: I2C Timeout Occurred
	  }
  }

  // Perform reads on the status registers to reset ADDR bit
  (void)I2C1 -> SR1;
  (void)I2C1 -> SR2;

  uint8_t data_buffer[6];

  I2C1 -> DR = (0x32);

  // Set delay for sending the bits
  // Makes sure data register is empty before proceeding
  while (!(I2C1-> SR1 & (1U << 7))){

  }

  // waiting for byte transfer finished state
  TimerStart();
  while (!(I2C1-> SR1 & (1U << 2))){
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // Error: I2C Timeout Occurred
	  }
  }

  // Repeated start to change to receiver mode
  I2C1 -> CR1 |= (1U << 8);

  // Wait for SB flag
  while(!(I2C1->SR1 & (1U << 0))) {
      // Wait until SB is 1
  }

  I2C1 -> DR = (0x53 << 1) | 1U;

  // check for ADDR register
  TimerStart();
  while(!(I2C1 -> SR1 & (1U << 1))){
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // Error: I2C Timeout Occurred
	  }
  }

  // ACK is still enabled here because we want 6 bytes in total

  // Perform reads on the status registers to reset ADDR bit
  (void)I2C1 -> SR1;
  (void)I2C1 -> SR2;

  // first byte
  TimerStart();
  while(!(I2C1 -> SR1 & (1U << 6))){
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // Error: I2C Timeout Occurred
	  }
  }
  data_buffer[0] = I2C1 -> DR;

  // second byte
  TimerStart();
  while(!(I2C1 -> SR1 & (1U << 6))){
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // Error: I2C Timeout Occurred
	  }
  }
  data_buffer[1] = I2C1 -> DR;

  // third byte
  TimerStart();
  while(!(I2C1 -> SR1 & (1U << 6))){
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // Error: I2C Timeout Occurred
	  }
  }
  data_buffer[2] = I2C1 -> DR;


  // fourth byte
  TimerStart();
  while(!(I2C1 -> SR1 & (1U << 6))){
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // Error: I2C Timeout Occurred
	  }
  }
  data_buffer[3] = I2C1 -> DR;

  // fifth byte
  TimerStart();
  while(!(I2C1 -> SR1 & (1U << 6))){
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // Error: I2C Timeout Occurred
	  }
  }
  I2C1 -> CR1 &= ~(1U << 10); // NACK and STOP must be set before reading the second-to-last byte to terminate the sequence correctly
  I2C1 -> CR1 |= (1U << 9);
  data_buffer[4] = I2C1 -> DR;

  // sixth byte
  TimerStart();
  while(!(I2C1 -> SR1 & (1U << 6))){
	  if((SysTick -> CTRL & (1U << 16))){
		  return 1; // Error: I2C Timeout Occurred
	  }
  }
  data_buffer[5] = I2C1 -> DR;

  // Set ACK to 1 again to return to original state
  I2C1 -> CR1 |= (1U << 10);

  // Combine the data
  int16_t x_raw = (int16_t)((data_buffer[1] << 8) | (data_buffer[0]));
  int16_t y_raw = (int16_t)((data_buffer[3] << 8) | (data_buffer[2]));
  int16_t z_raw = (int16_t)((data_buffer[5] << 8) | (data_buffer[4]));


  // Clean up and upload to github
  // next step:
  // - Continously monitor the ADXL345
  //-  Unit conversion


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(16000000);
  LL_SetSystemCoreClock(16000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
 3 * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
