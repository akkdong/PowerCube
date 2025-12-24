/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_lpuart.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stm32g4xx_hal_exuart.h>

#include "version.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */




/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PB_BOARD_Pin GPIO_PIN_13
#define PB_BOARD_GPIO_Port GPIOC
#define PB_BOARD_EXTI_IRQn EXTI15_10_IRQn
#define VBUS_ADC_Pin GPIO_PIN_0
#define VBUS_ADC_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_2
#define GPS_RX_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_3
#define GPS_TX_GPIO_Port GPIOA
#define BARO_I2C_SCL_Pin GPIO_PIN_4
#define BARO_I2C_SCL_GPIO_Port GPIOC
#define PB_POWER_Pin GPIO_PIN_0
#define PB_POWER_GPIO_Port GPIOB
#define PB_POWER_EXTI_IRQn EXTI0_IRQn
#define HOLD_POWER_Pin GPIO_PIN_1
#define HOLD_POWER_GPIO_Port GPIOB
#define VBUS_POWER_Pin GPIO_PIN_2
#define VBUS_POWER_GPIO_Port GPIOB
#define TRACE_RX_Pin GPIO_PIN_10
#define TRACE_RX_GPIO_Port GPIOB
#define TRACE_TX_Pin GPIO_PIN_11
#define TRACE_TX_GPIO_Port GPIOB
#define LED_DEVICERDY_Pin GPIO_PIN_12
#define LED_DEVICERDY_GPIO_Port GPIOB
#define LED_BOARD_Pin GPIO_PIN_6
#define LED_BOARD_GPIO_Port GPIOC
#define BARO_I2C_SDA_Pin GPIO_PIN_8
#define BARO_I2C_SDA_GPIO_Port GPIOA
#define USBPD_I2C_SCL_Pin GPIO_PIN_15
#define USBPD_I2C_SCL_GPIO_Port GPIOA
#define BLUETOOTH_RX_Pin GPIO_PIN_10
#define BLUETOOTH_RX_GPIO_Port GPIOC
#define BLUETOOTH_TX_Pin GPIO_PIN_11
#define BLUETOOTH_TX_GPIO_Port GPIOC
#define BLUETOOTH_RST_Pin GPIO_PIN_3
#define BLUETOOTH_RST_GPIO_Port GPIOB
#define EN_EXTRAPWR_Pin GPIO_PIN_7
#define EN_EXTRAPWR_GPIO_Port GPIOB
#define USBPD_I2C_SDA_Pin GPIO_PIN_9
#define USBPD_I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
