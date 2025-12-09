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
#include "cmsis_os.h"
#include "usbpd.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "HardwareSerial.h"
#include "BMP280.h"
#include "INA226.h"
#include "HUSB238.h"

#include "Task.h"

//
//
//

#ifdef __cplusplus
extern "C" {
#endif

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_LPUART1_UART_Init(void);
void MX_UCPD1_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
void MX_USART2_UART_Init(void);
void MX_ADC1_Init(void);
void MX_USART3_UART_Init(void);

#ifdef __cplusplus
}
#endif


//
//
//

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;


//
//
//

I2CMaster i2c1(&hi2c1);
I2CMaster i2c2(&hi2c2);


//
//
//

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_UCPD1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();

  //
  Serial2.begin(/*UART_HandleTypeDef * */);
  Serial3.begin(/*UART_HandleTypeDef * */);

  // USBPD initialization
  MX_USBPD_Init();

  //
  startRTOS();

  // We should never get here as control is now taken by the scheduler
  while (1)
  {
  }
}

