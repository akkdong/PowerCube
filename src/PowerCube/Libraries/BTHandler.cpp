/*
 * BTHandler.cpp
 *
 *
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "main.h"
#include "BTHandler.h"


#ifdef __cplusplus
static const osMutexDef_t os_mutex_def_BT = { 0 };
#else
static osMutexDef(BT);
#endif

static char linebuf[64];



//////////////////////////////////////////////////////////////////////////////
//

BluetoothHandler::BluetoothHandler(HardwareSerial &serial)
	: m_serial(serial)
	, m_state(STANDBY)
{

}

bool BluetoothHandler::begin()
{
	m_mutexId = osMutexCreate(osMutex(BT));

	// 0. reset BT module
	//   - use RESET pin
	//   - State is STANDBY
	//   - delay 100ms
	// 1. check BT module
	//     - send "AT\r"
	//     - check response: "\r\nOK\r\n"
	// 2. check MODE
	//     - send "AT+INFO?\r"
	//     - check response: "\r\n{BT-ADDR},{Name},{Mode},{Status},{Auth}\r\n"
	//     - set mode to 3 if it is not mode3
	//     - check response: "\r\nOK\r\n"
	// 3. Now device is ready: State is PENDING
	//
	// In practice, it only requires step 0, 1

	//
	m_state = STANDBY;

	HAL_GPIO_WritePin(BLUETOOTH_RST_GPIO_Port, BLUETOOTH_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(BLUETOOTH_RST_GPIO_Port, BLUETOOTH_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(100);

	//
	write("AT\r");

	uint32_t lastTick = HAL_GetTick();
	while (1)
	{
		update();
		/*
		if (check receive for "OK")
		   break;
		*/

		if (HAL_GetTick()- lastTick > 400)
		{
			// Timeout: No response
			return false;
		}
	}

	//
	m_state = PENDING;

	return true;
}

void BluetoothHandler::end()
{
	osMutexDelete(m_mutexId);
}


void BluetoothHandler::update()
{
	while (m_serial.available())
	{
		int ch = m_serial.read();
		linebuf[0] = ch;
	}
}


int BluetoothHandler::puts(const char *str)
{
	if (m_state == CONNECTED)
		m_serial.puts(str);

	return 0;
}

int BluetoothHandler::putc(int ch)
{
	if (m_state == CONNECTED)
		m_serial.write(ch);

	return 0;
}

int BluetoothHandler::printf(const char *fmt, ...)
{
	if (m_state == CONNECTED)
	{
		va_list args;
		va_start(args, fmt);
		vsprintf(linebuf, fmt, args);
		va_end(args);

		m_serial.puts(linebuf);
	}

	return 0;
}

void BluetoothHandler::write(const char *str, int len)
{
	const char *ptr = str;
	while (len > 0 || (len < 0 && *ptr))
	{
		m_serial.write(*ptr);
		if (len > 0)
			--len;
		++ptr;
	}
}
