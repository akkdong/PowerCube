/*
 * SerialImpl.cpp
 *
 *  Created on: 2022. 5. 6.
 *      Author: akkdong
 *
 *  Modified on: 2022. 9. 27.
 */


#include "Serial.h"

#ifdef __cplusplus
extern "C" {
#endif


//
// UART IRQ handler
//

#if ENABLE_UART1

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&Serial1.huart);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

#endif // ENABLE_UART1

#if ENABLE_UART2 && 0

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&Serial2.huart);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

#endif // ENABLE_UART2


#if ENABLE_UART3 && 0

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&Serial3.huart);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

#endif // ENABLE_UART1


#if ENABLE_UART4

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&Serial4.huart);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

#endif // ENABLE_UART4



//
// UART Callback
//

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	SER_OnRxComplete((SER_HandleTypeDef *)huart);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	SER_OnTxComplete((SER_HandleTypeDef *)huart);
}


#ifdef __cplusplus
}
#endif

