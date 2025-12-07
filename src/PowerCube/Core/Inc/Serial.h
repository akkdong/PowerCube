/*
 * Serial.h
 *
 *  Created on: 2022. 5. 4.
 *      Author: akkdong
 *
 *  Modified on: 2022. 9. 27.
 */

#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#include "stm32g4xx_hal.h"
#include "RingBuffer.h"

#define ENABLE_UART1		0
#define ENABLE_UART2		1
#define ENABLE_UART3		1
#define ENABLE_UART4		0


//////////////////////////////////////////////////////////////////////////////////////////////
// class Serial

typedef struct SER_HandleTypeDef
{
	UART_HandleTypeDef huart;
	IRQn_Type			m_irq;

	//
	/*
	void 	(* Init)(SER_HandleTypeDef *pSerial);
	void 	(* DeInit)(SER_HandleTypeDef *pSerial);
	*/
	/*
	int 	(* begin)(SER_HandleTypeDef *pSerial, void (* init)(UART_HandleTypeDef* handle), IRQn_Type irq);
	void 	(* end)(SER_HandleTypeDef *pSerial);

	int		(* available)(SER_HandleTypeDef *pSerial);
	int		(* read)(SER_HandleTypeDef *pSerial);
	int		(* writeByte)(SER_HandleTypeDef *pSerial, uint8_t ch);
	int		(* writeData)(SER_HandleTypeDef *pSerial, uint8_t* data, int32_t dataLen);

	int		(* puts)(SER_HandleTypeDef *pSerial, const char* str);

	void 	(* OnRxComplete)(SER_HandleTypeDef *pSerial);
	void 	(* OnTxComplete)(SER_HandleTypeDef *pSerial);

	void	(* transmit)(SER_HandleTypeDef *pSerial);
	void	(* reset)(SER_HandleTypeDef *pSerial);
	*/


	RingBuffer			m_rxBuf;
	RingBuffer			m_txBuf;

	uint8_t				m_rxData[512];
	uint8_t				m_txData[512];

	uint8_t				m_rxDataCache;
	volatile int		m_txDataCache;

} SER_HandleTypeDef;




//////////////////////////////////////////////////////////////////////////////////////////////
//
/*
void    SER_Init(SER_HandleTypeDef *pSerial);
void    SER_Deinit(SER_HandleTypeDef *pSerial);
*/

int 	SER_begin(SER_HandleTypeDef *pSerial, void (* init)(UART_HandleTypeDef* handle), IRQn_Type irq);
void 	SER_end(SER_HandleTypeDef *pSerial);

int		SER_available(SER_HandleTypeDef *pSerial);
int		SER_read(SER_HandleTypeDef *pSerial);
int		SER_writeByte(SER_HandleTypeDef *pSerial, uint8_t ch);
int		SER_writeData(SER_HandleTypeDef *pSerial, uint8_t* data, int32_t dataLen);

int		SER_puts(SER_HandleTypeDef *pSerial, const char* str);

void 	SER_OnRxComplete(SER_HandleTypeDef *pSerial);
void 	SER_OnTxComplete(SER_HandleTypeDef *pSerial);

void	SER_transmit(SER_HandleTypeDef *pSerial);
void	SER_reset(SER_HandleTypeDef *pSerial);



//////////////////////////////////////////////////////////////////////////////////////////////
// Serial implementations

#if ENABLE_UART1
extern struct SER_HandleTypeDef Serial1;
#endif // ENABLE_UART1

#if ENABLE_UART2
extern struct SER_HandleTypeDef Serial2;
#endif // ENABLE_UART2

#if ENABLE_UART3
extern struct SER_HandleTypeDef Serial3;
#endif // ENABLE_UART3

#if ENABLE_UART4
extern struct SER_HandleTypeDef Serial4;
#endif //ENABLE_UART4


#endif /* INC_SERIAL_H_ */
