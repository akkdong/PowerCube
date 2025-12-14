/*
 * USBSerial.cpp
 *
 *
 */


#include "usbd_cdc_if.h"
#include "cdc_queue.h"
#include "utils.h"
#include "USBSerial.h"


//
//
//


USBSerial &CreateInstance()
{
	static USBSerial usbSerial;
	return usbSerial;
}

USBSerial &SerialUSB = CreateInstance();



//
//
//

void USBSerial::begin()
{
}

void USBSerial::end()
{
}


int USBSerial::available()
{
	// Just ReceiveQueue size, available for reading
	return static_cast<int>(CDC_ReceiveQueue_ReadSize(&ReceiveQueue));
}


int USBSerial::peek()
{
	// Peek one symbol, it can't change receive avaiablity
	return CDC_ReceiveQueue_Peek(&ReceiveQueue);
}

int USBSerial::read()
{
	// Dequeue only one char from queue
	// TS: it safe, because only main thread affects ReceiveQueue->read pos
	auto ch = CDC_ReceiveQueue_Dequeue(&ReceiveQueue);
	// Resume receive process, if possible
	CDC_resume_receive();
	return ch;
}

size_t USBSerial::readBytes(char *buf, size_t len)
{
	uint16_t read;
	static const uint32_t _timeout = 1000;
	auto rest = static_cast<uint16_t>(len);
	auto _startMillis = millis();
	do
	{
		read = CDC_ReceiveQueue_Read(&ReceiveQueue, reinterpret_cast<uint8_t *>(buf), rest);
		CDC_resume_receive();
		rest -= read;
		buf += read;
		if (rest == 0)
			return len;
	} while (millis() - _startMillis < _timeout);
	return len - rest;
}

size_t USBSerial::write(uint8_t ch)
{
	// Just write single-byte buffer.
	return write(&ch, 1);
}

size_t USBSerial::write(const char *str)
{
	return write((const uint8_t *)str, strlen(str));
}

size_t USBSerial::write(const uint8_t *buf, size_t size)
{
	size_t rest = size;
	while (rest > 0 && CDC_connected())
	{
		// Determine buffer size available for write
		auto portion = (size_t)CDC_TransmitQueue_WriteSize(&TransmitQueue);
		// Truncate it to content size (if rest is greater)
		if (rest < portion) {
			portion = rest;
		}
		if (portion > 0)
		{
			// Only if some space in the buffer exists.
			// TS: Only main thread calls write and writeSize methods,
			// it's thread-safe since IRQ does not affects
			// TransmitQueue write position
			CDC_TransmitQueue_Enqueue(&TransmitQueue, buf, portion);
			rest -= portion;
			buf += portion;
			// After storing data, start transmitting process
			CDC_continue_transmit();
		}
	}

	return rest;
}

size_t USBSerial::puts(const char *str, bool appendLinefeed)
{
	write(str);
	if (appendLinefeed)
		write("\r\n");

	return 0;
}

USBSerial::operator bool()
{
  return CDC_connected();
}
