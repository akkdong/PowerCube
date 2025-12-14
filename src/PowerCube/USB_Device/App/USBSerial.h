/*
 * USBSerial.h
 *
 *
 */

#ifndef APP_USBSERIAL_H_
#define APP_USBSERIAL_H_

#include "stdint.h"
#include "stddef.h"

//
//
//




//
//
//

class USBSerial
{
protected:
	USBSerial() {}

	friend USBSerial &CreateInstance();

public:
	void begin();
	void end();

	int available();

	int peek();
	int read();
	size_t readBytes(char *buf, size_t len);

	size_t write(uint8_t ch);
	size_t write(const char *str);
	size_t write(const uint8_t *buf, size_t size);
	size_t puts(const char *str, bool appendLinefeed = false);

	operator bool();
};



//
//
//

extern USBSerial &SerialUSB;




#endif /* APP_USBSERIAL_H_ */
