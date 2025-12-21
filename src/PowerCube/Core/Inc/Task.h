/*
 * Task.h
 *
 *  Created on: 2025. 12. 9.
 *      Author: akkdong
 */

#ifndef INC_TASK_H_
#define INC_TASK_H_

#include "cmsis_os.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

//
//
//

#define MQ_MSRC_ADCTASK				0x10000000
#define MQ_MSRC_VARIOTASK			0x20000000
#define MQ_MSRC_SHUTDOWNTIMER		0x40000000
#define MQ_MSRC_EXTINTR				0x80000000




//
//
//

enum PowerObjectType
{
	PO_ACTIVE,
	PO_5V,
	PO_9V,
	PO_12V,
	PO_15V,
	PO_18V,
	PO_24V,
	PO_COUNT
};

typedef struct __PowerObject
{
	uint32_t voltage;
	uint32_t current;
} PowerObject;


enum PB_State
{
	PB_RELEASED = 0,
	PB_PRESSED = 1
};

enum PB_Type
{
	PB_BODY = 0,
	PB_BOARD = 1,

	PB_COUNT
};

typedef struct __DeviceState
{
	// Sensor data: GPS, Pressure, Humidity
	uint32_t now;
	float latitude, longitude, altitude, speed;
	float temperature, pressure, humidity, varioSpeed;

	// Input Power
	uint32_t voltage, current;
	PowerObject po[PO_COUNT];

	// Output Power
	uint32_t shuntVoltage;
	uint32_t vbusVoltage;
	uint32_t vbusCurrent;

	// Push Button
	uint32_t btnState[PB_COUNT];
	uint32_t btnUpdateMask;

} DeviceState;


//
//
//

void MainTaskProc(void const *arg);
void AdcTaskProc(void const *arg);
void VarioTaskProc(void const *arg);


//
//
//

void ShutdownTimerCallback(void const *arg);



//
//
//

#ifdef DEBUG

#if DEBUG_USEBT

extern void Trace(const char *format, ...);

#define TRACE0(msg)			Trace(msg)
#define TRACE(fmt, ...)		Trace(fmt, ##__VA_ARGS__)

#elif DEBUG_USEUSBPD

extern int _usbpd_printf(const char *format, ...);

#define TRACE0(msg)			_usbpd_printf(msg)
#define TRACE(fmt, ...)		_usbpd_printf(fmt, ##__VA_ARGS__)

#else

#include "logger.h"

#define TRACE0(msg)			_log_printf(LOG_LEVEL_DEBUG, msg)
#define TRACE(fmt, ...)		_log_printf(LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)

#endif

#else

#define TRACE0(msg)
#define TRACE(fmt, ...)

#endif



//
//
//


#ifdef __cplusplus
}
#endif


#endif /* INC_TASK_H_ */
