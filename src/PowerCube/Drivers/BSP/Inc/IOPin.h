/*
 * IOPin.h
 *
 *
 */

#ifndef BSP_INC_IOPIN_H_
#define BSP_INC_IOPIN_H_

#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_gpio.h"


//
//  IOPin
//

class IOPin
{
public:
	IOPin(GPIO_TypeDef *port, uint16_t pin, bool activeHigh);

public:
	void on();
	void off();
	void toggle();

	GPIO_PinState getState() { return HAL_GPIO_ReadPin(m_port, m_pin); }

	bool isOn();
	bool isOff();

protected:
	GPIO_TypeDef *m_port;
	uint16_t m_pin;
	bool m_activeHigh;
};



//
//
//

enum IOPin_Types
{
	IOPIN_BODY_INPUT,
	IOPIN_BOARD_INPUT,

	IOPIN_POWER_DEVICE,
	IOPIN_POWER_PERIPH,
	IOPIN_POWER_VBUS,

	IOPIN_LED_BODY,
	IOPIN_LED_BOARD,

	IPOIN_COUNT
};


extern IOPin ioPin[IPOIN_COUNT];



#endif /* BSP_INC_IOPIN_H_ */
