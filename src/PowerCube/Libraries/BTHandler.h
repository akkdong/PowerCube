/*
 * BTHandler.h
 *
 *
 */

#ifndef BTHANDLER_H_
#define BTHANDLER_H_

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
	void update();
	void end();

	State getState() { return m_state; }

	int puts(const char *str);
	int putc(int ch);
	int printf(const char *fmt, ...);

protected:
	void write(const char *str, int len = -1);

protected:
	HardwareSerial &m_serial;
	State m_state;

	osMutexId m_mutexId;
};


#endif /* BTHANDLER_H_ */
