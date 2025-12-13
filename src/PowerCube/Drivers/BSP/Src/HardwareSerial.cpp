/*
 * HardwareSerial.cpp
 *
 *  Created on: 2025. 12. 8.
 *      Author: akkdong
 */


#include <stm32g4xx_hal_exuart.h>
#include "HardwareSerial.h"



//////////////////////////////////////////////////////////////////////////////////
//

#if METHOD_OBSOLETE

#if ENABLE_UART1
extern UART_ExHandleTypeDef exhuart1;
#endif
#if ENABLE_UART2
extern UART_ExHandleTypeDef exhuart2;
#endif
#if ENABLE_UART3
extern UART_ExHandleTypeDef exhuart3;
#endif
#if ENABLE_UART4
extern UART_ExHandleTypeDef exhuart4;
#endif

static HardwareSerial instance[SERIALMAX] =
{
#if ENABLE_UART1
		{ &huart1, USART1_IRQn },
#endif
#if ENABLE_UART2
		{ &huart2, USART2_IRQn },
#endif
#if ENABLE_UART3
		{ &huart3, USART3_IRQn },
#endif
#if ENABLE_UART4
		{ &huart4, USART4_IRQn },
#endif
};


#if ENABLE_UART1
HardwareSerial &Serial1 = instance[SERIAL1];
#endif // ENABLE_UART1

#if ENABLE_UART2
HardwareSerial &Serial2 = instance[SERIAL2];
#endif // ENABLE_UART2

#if ENABLE_UART3
HardwareSerial &Serial3 = instance[SERIAL3];
#endif // ENABLE_UART3

#if ENABLE_UART4
HardwareSerial &Serial4 = instance[SERIAL4];
#endif //ENABLE_UART4

#else // METHOD_OBSOLETE

#if ENABLE_UART1
HardwareSerial Serial;
#endif // ENABLE_UART1

#if ENABLE_UART2
HardwareSerial Serial2;
#endif // ENABLE_UART2

#if ENABLE_UART3
HardwareSerial Serial3;
#endif // ENABLE_UART3

#if ENABLE_UART4
HardwareSerial Serial4;
#endif //ENABLE_UART4

#endif // METHOD_OBSOLETE

static HardwareSerial *getObject(UART_HandleTypeDef *pHandle)
{
#if METHOD_OBSOLETE
#if ENABLE_UART1
	if (Serial1 == pHandle)
		return &Serial1;
#endif // ENABLE_UART1

#if ENABLE_UART2
	if (Serial2 == pHandle)
			return &Serial2;
#endif // ENABLE_UART2

#if ENABLE_UART3
	if (Serial3 == pHandle)
			return &Serial3;
#endif // ENABLE_UART3

#if ENABLE_UART4
	if (Serial4 == pHandle)
			return &Serial4;
#endif //ENABLE_UART4

#else // METHOD_OBSOLETE

	UART_ExHandleTypeDef *pSerial = reinterpret_cast<UART_ExHandleTypeDef *>(pHandle);
	if (pSerial->pUserData)
		return reinterpret_cast<HardwareSerial *>(pSerial->pUserData);

#endif // METHOD_OBSOLETE

	return nullptr;
}





//////////////////////////////////////////////////////////////////////////////////
//

HardwareSerial::HardwareSerial() : m_pHandle(nullptr)
{
}

//HardwareSerial::HardwareSerial(UART_HandleTypeDef *pHandle)
//	: m_pHandle(pHandle)
//{
//}

HardwareSerial::~HardwareSerial()
{
}


int HardwareSerial::begin(UART_ExHandleTypeDef *pHandle, IRQn_Type irq)
{
	//
	pHandle->pUserData = reinterpret_cast<void *>(this);
	m_pHandle = &pHandle->huart;
	m_irq = irq;

	this->reset();

	//
	HAL_UART_Receive_IT(this->getHandle(), &m_rxDataCache, 1);

	return 0;
}

void HardwareSerial::end()
{
}


int HardwareSerial::available()
{
	return RB_GetDataCount(&m_rxBuf);
}

int HardwareSerial::read()
{
	//__disable_irq();
	int ch = RB_Pop(&m_rxBuf);
	//__enable_irq();

	return ch;
}

int HardwareSerial::write(uint8_t ch)
{
	if (RB_Push(&m_txBuf, ch) < 0)
			return -1;

	transmit();

	return 0;
}

int HardwareSerial::write(uint8_t* data, int32_t dataLen)
{
	for (int32_t n = 0; n < dataLen; n++)
		write(data[n]);

	return dataLen;
}


int HardwareSerial::puts(const char* str)
{
	int n = 0;
	while (str[n])
		write(str[n++]);

	return n;
}


void HardwareSerial::transmit()
{
	if (m_txDataCache < 0)
	{
		m_txDataCache = RB_Pop(&m_txBuf);

		if (m_txDataCache >= 0)
		{
			//HAL_NVIC_DisableIRQ(m_irq);
			HAL_UART_Transmit_IT(this->getHandle(), (uint8_t *)&m_txDataCache, 1);
			//HAL_NVIC_EnableIRQ(m_irq);
		}
	}
}

void HardwareSerial::reset()
{
	RB_Init(&m_rxBuf, m_rxData, sizeof(m_rxData));
	RB_Init(&m_txBuf, m_txData, sizeof(m_txData));

	m_txDataCache = -1;
}



void HardwareSerial::OnRxComplete()
{
	// save receive-data
	RB_Push(&m_rxBuf, m_rxDataCache);

	//
	HAL_UART_Receive_IT(this->getHandle(), &m_rxDataCache, 1);
}

void HardwareSerial::OnTxComplete()
{
	m_txDataCache = -1;

	transmit();
}




//
//
//

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *pHandle)
{
	HardwareSerial *pSerial = getObject(pHandle);
	if (pSerial)
		pSerial->OnRxComplete();

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *pHandle)
{
	HardwareSerial *pSerial = getObject(pHandle);
		if (pSerial)
			pSerial->OnTxComplete();
}
