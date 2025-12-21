/*
 * logger.c
 *
 *
 */


#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>

#include "logger.h"

#include "usbpd_trace.h"


//
//
//





//
//
//

#if DEBUG_USEBT

#include "BluetoothHandler.h"


void Trace(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	bt.trace(format, args);
	va_end(args);
}


#elif DEBUG_USEUSBPD


#define TRACE_SIZE  50

static char __buf[TRACE_SIZE];
/*
static char __level[] = { 'E', 'W', 'I', 'V', 'D' };
*/

static void TrimRight(char *str)
{
	int len = strlen(str);
	while (len > 1)
	{
		--len;
		int ch = str[len];
		if (!isspace(ch))
			break;
		str[len] = 0;
	}
}

int _usbpd_printf(const char* format, ...)
{
	//
	va_list args;
	va_start(args, format);
	int len = vsnprintf(&__buf[0], TRACE_SIZE, format, args);
	va_end(args);

	TrimRight(__buf);

	//if (len < TRACE_SIZE)
	//	USBPD_TRACE_Add(USBPD_TRACE_DEBUG, (uint8_t)0, 0, (uint8_t*)__buf, strlen(__buf));
	//else
		USBPD_TRACE_Add(USBPD_TRACE_DEBUG, (uint8_t)0, 0, (uint8_t*)__buf, len);

	return len;
}

#endif
