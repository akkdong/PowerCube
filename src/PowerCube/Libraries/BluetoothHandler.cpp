/*
 * BluetoothHandler.cpp
 *
 *
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "main.h"
#include "BluetoothHandler.h"



//
//
//

#ifdef __cplusplus
static const osMutexDef_t os_mutex_def_BT = { 0 };
#else
static osMutexDef(BT);
#endif



//
//
//

char BluetoothHandler::m_line[];
int BluetoothHandler::m_linePos = 0;

char BluetoothHandler::m_temp[];



//
//
//

static uint8_t NumToHexa(uint8_t num)
{
	if (num < 10)
		return '0' + num;
	return 'A' + (num - 10);
}



//////////////////////////////////////////////////////////////////////////////
//

BluetoothHandler::BluetoothHandler(HardwareSerial &serial)
	: m_serial(serial)
	, m_state(STANDBY)
	, m_mutexId(NULL)
{

}

bool BluetoothHandler::begin()
{
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

#if 0
	HAL_GPIO_WritePin(BLUETOOTH_RST_GPIO_Port, BLUETOOTH_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(BLUETOOTH_RST_GPIO_Port, BLUETOOTH_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
#endif

	//
#if 1
	write("AT\r");

	uint32_t lastTick = HAL_GetTick();
	bool bIdentified = false;
	while (!bIdentified)
	{
		if (update() > 0)
		{
			const char *str = this->getReceiveData();
			if (strcmp(str, "OK") == 0)
				bIdentified = true;
			this->clearData();
		}

		if (HAL_GetTick()- lastTick > 1000)
		{
			// Timeout: No response
			return false;
		}
	}
#endif

	//
	m_state = PENDING;

	m_line[0] = 0;
	m_linePos = 0;


	//
	m_mutexId = osMutexCreate(osMutex(BT));

	return true;
}

void BluetoothHandler::end()
{
	if (m_mutexId)
		osMutexDelete(m_mutexId);
}


int BluetoothHandler::update()
{
	while (m_serial.available())
	{
		int ch = m_serial.read();

		if (ch == '\r' || ch == '\n')
		{
			int lineSize = m_linePos;
			m_linePos = 0;

			return lineSize;
		}
		else
		{
			m_line[m_linePos] = ch;
			++m_linePos;
			m_line[m_linePos] = 0;
		}
	}

	return 0;
}

void BluetoothHandler::clearData()
{
	m_line[0] = 0;
	m_linePos = 0;
}


int BluetoothHandler::puts(const char *str, bool appendLineFeed)
{
	if (m_state == CONNECTED)
	{
		if (m_mutexId)
			osMutexWait(m_mutexId, osWaitForever);

		write(str, strlen(str));
		if (appendLineFeed)
			write("\r\n", -1);

		if (m_mutexId)
			osMutexRelease(m_mutexId);
	}

	return 0;
}

int BluetoothHandler::putc(char ch)
{
	if (m_state == CONNECTED)
	{
		if (m_mutexId)
			osMutexWait(m_mutexId, osWaitForever);

		write(&ch, 1);

		if (m_mutexId)
			osMutexRelease(m_mutexId);
	}

	return 0;
}

int BluetoothHandler::printf(const char *fmt, ...)
{
	if (m_state == CONNECTED)
	{
		if (m_mutexId)
			osMutexWait(m_mutexId, osWaitForever);

		va_list args;
		va_start(args, fmt);
		write(fmt, args);
		va_end(args);

		if (m_mutexId)
			osMutexRelease(m_mutexId);
	}

	return 0;
}

int BluetoothHandler::printf(const char *fmt, va_list &args)
{
	if (m_state == CONNECTED)
	{
		if (m_mutexId)
			osMutexWait(m_mutexId, osWaitForever);

		write(fmt, args);

		if (m_mutexId)
			osMutexRelease(m_mutexId);
	}

	return 0;
}

const char *dbgStatement = "$DBG,";

int BluetoothHandler::trace(const char *fmt, va_list &args)
{
	if (m_state == CONNECTED)
	{
		if (m_mutexId)
			osMutexWait(m_mutexId, osWaitForever);

		strcpy(m_temp, dbgStatement);
		char *ptr = &m_temp[0] + strlen(dbgStatement);
		/*int len =*/ vsnprintf(ptr, m_bufSize - (m_temp - ptr) - 5, fmt, args);

		uint8_t crc = 0;
		ptr = &m_temp[1];
		while (*ptr && *ptr != '\r' && *ptr != '\n')
		{
			crc = crc ^ (uint8_t)ptr[0];
			++ptr;
		}
		*ptr++ = '*';
		*ptr++ = NumToHexa((crc >> 4) & 0x0F);
		*ptr++ = NumToHexa(crc & 0x0F);
		*ptr++ = '\r';
		*ptr++ = '\n';
		*ptr   = 0;

		write(m_temp, ptr - &m_temp[0]);

		if (m_mutexId)
			osMutexRelease(m_mutexId);
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

void BluetoothHandler::write(const char *fmt, va_list &args)
{
	int len = vsnprintf(m_temp, m_bufSize, fmt, args);
	write(m_temp, len);
}

