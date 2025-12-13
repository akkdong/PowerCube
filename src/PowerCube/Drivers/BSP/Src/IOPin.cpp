/*
 * IOPin.cpp
 *
 *
 */

#include "IOPin.h"
#include "main.h"


//
//  IOPin
//

IOPin::IOPin(GPIO_TypeDef *port, uint16_t pin, bool activeHigh)
{
	m_port = port;
	m_pin = pin;
	m_activeHigh = activeHigh;
}

void IOPin::on()
{
	HAL_GPIO_WritePin(m_port, m_pin, m_activeHigh ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void IOPin::off()
{
	HAL_GPIO_WritePin(m_port, m_pin, m_activeHigh ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void IOPin::toggle()
{
	HAL_GPIO_TogglePin(m_port, m_pin);
}


bool IOPin::isOn()
{
	GPIO_PinState state = getState();
	bool high = state == GPIO_PIN_RESET ? false : true;
	if (!m_activeHigh)
		high = !high;

	return high;
}

bool IOPin::isOff()
{
	return !isOn();
}





//
//
//

IOPin ioPin[IPOIN_COUNT] =
{
	// INPUT
	{ PB_POWER_GPIO_Port, PB_POWER_Pin, false }, 			// IOPIN_BODY_INPUT
	{ PB_BOARD_GPIO_Port, PB_BOARD_Pin, true },  			// IOPIN_BOARD_INPUT

	// OUTPUT
	{ HOLD_POWER_GPIO_Port, HOLD_POWER_Pin, true },			// IOPIN_POWER_DEVICE
	{ EN_EXTRAPWR_GPIO_Port, EN_EXTRAPWR_Pin, false },		// IOPIN_POWER_PERIPH
	{ VBUS_POWER_GPIO_Port, VBUS_POWER_Pin, true },			// IOPIN_POWER_VBUS

	{ LED_DEVICERDY_GPIO_Port, LED_DEVICERDY_Pin, false },	// IOPIN_LED_BODY
	{ LED_BOARD_GPIO_Port, LED_BOARD_Pin, true },			// IOPIN_LED_BOARD
};
