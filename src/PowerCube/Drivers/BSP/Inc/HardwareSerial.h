/*
 * HardwareSerial.h
 *
 *  Created on: 2025. 12. 8.
 *      Author: akkdong
 */

#ifndef BSP_INC_HARDWARESERIAL_H_
#define BSP_INC_HARDWARESERIAL_H_

#include <stm32g4xx_hal_exuart.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_uart.h"
#include "RingBuffer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENABLE_UART1		0
#define ENABLE_UART2		1
#define ENABLE_UART3		1
#define ENABLE_UART4		0

enum eHardwareSerial
{
#if ENABLE_UART1
	SERIAL1,
#endif
#if ENABLE_UART2
	SERIAL2,
#endif
#if ENABLE_UART3
	SERIAL3,
#endif
#if ENABLE_UART4
	SERIAL4,
#endif
	SERIALMAX
};


//////////////////////////////////////////////////////////////////////////////////
//

class HardwareSerial
{
	friend void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
	friend void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);

public:
	HardwareSerial();
	~HardwareSerial();

public:
	int begin(UART_ExHandleTypeDef *pHandle, IRQn_Type irq);
	void end();

	int available();
	int read();
	int write(uint8_t ch);
	int write(uint8_t* data, int32_t dataLen);

	int puts(const char* str);

	UART_HandleTypeDef *getHandle() { return m_pHandle; }
	operator UART_HandleTypeDef *() const { return m_pHandle; }

protected:
	void transmit();
	void reset();

	void OnRxComplete();
	void OnTxComplete();

protected:
	//
	UART_HandleTypeDef *m_pHandle;
	IRQn_Type m_irq;

	//
	RingBuffer m_rxBuf;
	RingBuffer m_txBuf;

	uint8_t m_rxData[64];
	uint8_t m_txData[256];

	uint8_t m_rxDataCache;
	volatile int m_txDataCache;
};



//////////////////////////////////////////////////////////////////////////////////////////////
// HardwareSerial instance

#if ENABLE_UART1
extern HardwareSerial Serial1;
#endif // ENABLE_UART1

#if ENABLE_UART2
extern HardwareSerial Serial2;
#endif // ENABLE_UART2

#if ENABLE_UART3
extern HardwareSerial Serial3;
#endif // ENABLE_UART3

#if ENABLE_UART4
extern HardwareSerial Serial4;
#endif //ENABLE_UART4


#ifdef __cplusplus
}
#endif


#endif /* BSP_INC_HARDWARESERIAL_H_ */
