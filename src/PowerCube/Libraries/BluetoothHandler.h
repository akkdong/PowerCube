/*
 * BluetoothHandler.h
 *
 *
 */

#ifndef BLUETOOTHHANDLER_H_
#define BLUETOOTHHANDLER_H_

#include "HardwareSerial.h"
#include "cmsis_os.h"


//////////////////////////////////////////////////////////////////////////////
//

class BluetoothHandler
{
public:
	BluetoothHandler(HardwareSerial &serial);

	enum State {
		STANDBY,
		PENDING,
		PENDING_ESCAPE,
		CONNECTED,
	};

public:
	bool begin();
	void end();

	int update();

	const char *getReceiveData() { return m_line; }
	void clearData();

	State getState() { return m_state; }
	void setState(State state) { m_state = state; }

	int puts(const char *str, bool appendLineFeed = false);
	int putc(char ch);
	int printf(const char *fmt, ...);
	int printf(const char *fmt, va_list &args);

	int trace(const char *fmt, va_list &args);

protected:
	void write(const char *str, int len = -1);
	void write(const char *fmt, va_list &args);

protected:
	HardwareSerial &m_serial;
	State m_state;

	osMutexId m_mutexId;

	//
	static const int m_bufSize = 64;

	static char m_line[m_bufSize];
	static int m_linePos;

	static char m_temp[m_bufSize];
};


#endif /* BLUETOOTHHANDLER_H_ */
