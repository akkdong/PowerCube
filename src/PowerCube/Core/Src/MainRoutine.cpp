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
#include <stdarg.h>

#include "HardwareSerial.h"
#include "INA226.h"
#include "HUSB238.h"
#include "BMP280.h"
#include "AHT20.h"
#include "BeaconIndicator.h"
#include "ADCReader.h"

#include "BluetoothHandler.h"
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
void MX_USART2_UART_Init(UART_HandleTypeDef *pHandle);
void MX_ADC1_Init(void);
void MX_USART3_UART_Init(UART_HandleTypeDef *pHandle);


void StartRTOS();

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

#if 0
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
#else
UART_ExHandleTypeDef exhuart2;
UART_ExHandleTypeDef exhuart3;
#endif


//
//
//

I2CMaster i2c1(&hi2c1);
I2CMaster i2c2(&hi2c2);

INA226 ina226(i2c1);
HUSB238 husb238(i2c1);

#if 0
BMP280 baro(i2c2);
#else
Bme280TwoWire baro;
#endif
AHT20 aht20(i2c2);

BeaconIndicator beacon;
ADCReader adc(hadc1, 40.2 / (200 + 40.2));

BluetoothHandler bt(Serial3);

DeviceState devState;



//
//
//

osThreadId mainTaskHandle;
osThreadId adcTaskHandle;
osThreadId varioTaskHandle;

osSemaphoreId powerSemaphoreId;
osSemaphoreId boardSemaphoreId;

osTimerId shutdownTimerId;

osMessageQDef(MainQueue, 8, uint32_t);
osMessageQId mainQueueId;




//static osThreadDef(MainTask, MainTaskProc, osPriorityNormal, 0, 256);
static const osThreadDef_t os_thread_def_MainTask =
{
	(char *)"MainTask",
	MainTaskProc,
	osPriorityLow,
	0,
	256
};

// static osThreadDef(AdcTask, AdcTaskProc, osPriorityNormal, 0, 256);
static const osThreadDef_t os_thread_def_AdcTask =
{
	(char *)"AdcTask",
	AdcTaskProc,
	osPriorityBelowNormal,
	0,
	256
};

// static osThreadDef(VarioTask, VarioTaskProc, osPriorityNormal, 0, 256);
static const osThreadDef_t os_thread_def_VarioTask =
{
	(char *)"VarioTask",
	VarioTaskProc,
	osPriorityNormal,
	0,
	256
};

static osTimerDef(ShutdownTimer, ShutdownTimerCallback);




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
  MX_USART2_UART_Init(&exhuart2.huart);
  MX_ADC1_Init();
  MX_USART3_UART_Init(&exhuart3.huart);

  //
  Serial2.begin(&exhuart2, USART2_IRQn);
  Serial3.begin(&exhuart3, USART3_IRQn);

  // USBPD initialization
  MX_USBPD_Init();

  //
#if 0
{
	bool ina226Ok = i2c1.deviceReady(0x40); 	// OK
	bool husb238Ok = i2c1.deviceReady(0x08); 	// OK
	bool bmp280Ok1 = i2c2.deviceReady(0x76);
	bool bmp280Ok2 = i2c2.deviceReady(0x77); 	// OK
	bool aht20Ok = i2c2.deviceReady(0x38);	// OK
	int i = 0;
	while (1)
	{
		++i;
		break;
	}
}
#endif


  //
  StartRTOS();

  // We should never get here as control is now taken by the scheduler
  while (1)
  {
  }
}





////////////////////////////////////////////////////////////////////////////
//

void StartRTOS()
{
	//
	mainTaskHandle = osThreadCreate(osThread(MainTask), NULL);
	adcTaskHandle = osThreadCreate(osThread(AdcTask), NULL);
	varioTaskHandle = osThreadCreate(osThread(VarioTask), NULL);

	//
	shutdownTimerId = osTimerCreate(osTimer(ShutdownTimer), osTimerOnce, 0);

	//
	mainQueueId = osMessageCreate(osMessageQ(MainQueue), NULL);

	// Start scheduler
	osKernelStart();
}







//
//
//

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == PB_POWER_Pin)
	{
		devState.btnState[PB_BODY] = ioPin[IOPIN_BODY_INPUT].isOn() ? PB_PRESSED : PB_RELEASED;
		devState.btnUpdateMask |= (1 << PB_BODY);

		uint32_t info = MQ_MSRC_EXTINTR | (1 << PB_BODY);
		osMessagePut(mainQueueId, info, osWaitForever);
	}
	else if (GPIO_Pin == PB_BOARD_Pin)
	{
		devState.btnState[PB_BOARD] = ioPin[IOPIN_BOARD_INPUT].isOn() ? PB_PRESSED : PB_RELEASED;
		devState.btnUpdateMask |= (1 << PB_BOARD);

		/*
		GPIO_PinState state = HAL_GPIO_ReadPin(PB_BOARD_GPIO_Port, PB_BOARD_Pin);
		if (boardBtnState != state)
		{
			//osSemaphoreRelease(boardSemaphoreId);
			btnStateChanged |= 0x02;
		}
		*/

		uint32_t info = MQ_MSRC_EXTINTR | (1 << PB_BOARD);
		osMessagePut(mainQueueId, info, osWaitForever);
	}
}


//
//
//

void ShutdownTimerCallback(void const *arg)
{
	uint32_t info = MQ_MSRC_SHUTDOWNTIMER;
	osMessagePut(mainQueueId, info, osWaitForever);
}

