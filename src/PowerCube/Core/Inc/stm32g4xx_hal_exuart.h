/*
 * stm32g4xx_hal_exuart.h
 *
 *
 */

#ifndef BSP_INC_STM32G4XX_HAL_EXUART_H_
#define BSP_INC_STM32G4XX_HAL_EXUART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_uart.h"


//
//
//

typedef struct __UART_ExHandleTypeDef
{
	UART_HandleTypeDef huart;
	void *pUserData;
} UART_ExHandleTypeDef;


#ifdef __cplusplus
}
#endif


#endif /* BSP_INC_STM32G4XX_HAL_EXUART_H_ */
