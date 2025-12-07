/*
 * Serial.cpp
 *
 *  Created on: 2022. 5. 4.
 *      Author: akkdong
 */


#include "Serial.h"

#include <stdio.h>
#include <stdlib.h>


//////////////////////////////////////////////////////////////////////////////////////////////
//

int SER_begin(SER_HandleTypeDef *pSerial, void (* init)(UART_HandleTypeDef* handle), IRQn_Type irq)
{
	//
	SER_reset(pSerial);

	//
	init(&pSerial->huart);
	pSerial->m_irq = irq;

	HAL_UART_Receive_IT(&pSerial->huart, &pSerial->m_rxDataCache, 1);

	return 0;
}

void SER_end(SER_HandleTypeDef *pSerial)
{
}

int SER_available(SER_HandleTypeDef *pSerial)
{
	return RB_GetDataCount(&pSerial->m_rxBuf);
}

int SER_read(SER_HandleTypeDef *pSerial)
{
	return RB_Pop(&pSerial->m_rxBuf);
}

int SER_writeByte(SER_HandleTypeDef *pSerial, uint8_t ch)
{
	if (RB_Push(&pSerial->m_txBuf, ch) < 0)
		return -1;

	SER_transmit(pSerial);

	return 0;
}

int SER_writeData(SER_HandleTypeDef *pSerial, uint8_t* data, int32_t dataLen)
{
	for (int32_t n = 0; n < dataLen; n++)
		SER_writeByte(pSerial, data[n]);

	return dataLen;
}

int SER_puts(SER_HandleTypeDef *pSerial, const char* str)
{
	int n = 0;
	while (str[n])
		SER_writeByte(pSerial, str[n++]);

	return n;
}


void SER_OnRxComplete(SER_HandleTypeDef *pSerial)
{
	// save receive-data
	RB_Push(&pSerial->m_rxBuf, pSerial->m_rxDataCache);

	// receive next
	/*
	HAL_StatusTypeDef status;
	do {
		status = HAL_UART_Receive_IT(this, &m_rxDataCache, 1);
	} while (status == HAL_BUSY);
	*/
	//
	HAL_UART_Receive_IT(&pSerial->huart, &pSerial->m_rxDataCache, 1);
	//
}

void SER_OnTxComplete(SER_HandleTypeDef *pSerial)
{
	pSerial->m_txDataCache = -1;

	SER_transmit(pSerial);
}

void SER_transmit(SER_HandleTypeDef *pSerial)
{
	if (pSerial->m_txDataCache < 0)
	{
		pSerial->m_txDataCache = RB_Pop(&pSerial->m_txBuf);

		if (pSerial->m_txDataCache >= 0)
		{
			HAL_NVIC_DisableIRQ(pSerial->m_irq);
			HAL_UART_Transmit_IT(&pSerial->huart, (uint8_t *)&pSerial->m_txDataCache, 1);
			HAL_NVIC_EnableIRQ(pSerial->m_irq);
		}
	}
}

void SER_reset(SER_HandleTypeDef *pSerial)
{
	RB_Init(&pSerial->m_rxBuf, pSerial->m_rxData, sizeof(pSerial->m_rxData));
	RB_Init(&pSerial->m_txBuf, pSerial->m_txData, sizeof(pSerial->m_txData));

	pSerial->m_txDataCache = -1;
}
