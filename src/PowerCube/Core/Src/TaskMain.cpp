/*
 * Task.cpp
 *
 *  Created on: 2025. 12. 9.
 *      Author: akkdong
 */


#include "Task.h"
#include "usbpd.h"
#include "usb_device.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "HardwareSerial.h"
#include "USBSerial.h"
#include "BMP280.h"
#include "INA226.h"
#include "HUSB238.h"
#include "AHT20.H"
#include "IOPin.h"
#include "BeaconIndicator.h"

#include "BluetoothHandler.h"
#include "LineBuffer.h"
#include "VarioSentence.h"

#include "nmea/NmeaParser.h"


//
//
//

#define ROUTE_TO_BLUETOOTH	1


//
//
//

extern INA226 ina226;
extern HUSB238 husb238;

extern BluetoothHandler bt;
extern BeaconIndicator beacon;

extern DeviceState devState;

extern osTimerId shutdownTimerId;
extern osMessageQId mainQueueId;


//
//
//

const int maxBufSize = 96; // A standard GPS NMEA 0183 sentence has a maximum size of 82 characters

char bufSerial2[maxBufSize];
char bufVario[maxBufSize];

LineBuffer lineBuf(bufSerial2, maxBufSize);
LineBuffer varioBuf(bufVario, maxBufSize);

VarioSentence varioSentense(VSENTENCE_LK8);
NmeaParser nmeaParser;



////////////////////////////////////////////////////////////////////////////
//

void flushVarioSentence()
{
	// make vario sentence
	varioSentense.begin(
			devState.altitude,
			devState.varioSpeed,
			devState.temperature,
			devState.pressure,
			devState.voltage);

	// copy to line-buffer
	varioBuf.reset();
	while (varioSentense.available())
	{
		varioBuf.push(varioSentense.read());

		if (varioBuf.hasCompleteLine() && varioBuf.getLength() > 0)
			break;
	}

	// transfer a sentence to usb-serial and/or bt
	if (SerialUSB)
		SerialUSB.puts(varioBuf, true);

	#if ROUTE_TO_BLUETOOTH
	bt.puts(varioBuf, true);
	#endif
}



////////////////////////////////////////////////////////////////////////////
//

void MainTaskProc(void const * argument)
{
	//
	beacon.begin();
	beacon.off(BeaconIndicator::BeaconType::Body);
	beacon.off(BeaconIndicator::BeaconType::Board);

	ioPin[IOPIN_POWER_DEVICE].on();
	ioPin[IOPIN_POWER_PERIPH].on();
	ioPin[IOPIN_POWER_VBUS].off();

	//
	bt.begin();


	//
	memset(&devState, 0, sizeof(devState));

	devState.btnState[PB_BODY] = ioPin[IOPIN_BODY_INPUT].isOn() ? PB_PRESSED : PB_RELEASED;
	devState.btnState[PB_BOARD] = ioPin[IOPIN_BOARD_INPUT].isOn() ? PB_PRESSED : PB_RELEASED;

	lineBuf.reset();
	nmeaParser.reset();

	//

	beacon.on(BeaconIndicator::BeaconType::Body);
	beacon.blink(BeaconIndicator::BeaconType::Board, 500);


	/* Infinite loop */
	for(;;)
	{
		//
		if (Serial2.available() > 0)
		{
			int ch = Serial2.read();
			lineBuf.push(ch);

			if (nmeaParser.update(ch) > 0) // 1: GPS, 2: VARIO(UNUSED), 3: KEY(UNUSED)
			{
				devState.latitude = nmeaParser.getLatitude();
				devState.longitude = nmeaParser.getLongitude();
				devState.altitude = nmeaParser.getAltitude();
				devState.speed = nmeaParser.getSpeed();
				devState.now = nmeaParser.getDateTime();

				// do what?
			}
		}

		if (bt.update() > 0)
		{
			const char *str = bt.getReceiveData();
			if (strncmp(str, "CONNECT", strlen("CONNECT")) == 0)
				bt.setState(BluetoothHandler::CONNECTED);
			else if (strncmp(str, "DISCONNECT", strlen("DISCONNECT")) == 0)
				bt.setState(BluetoothHandler::PENDING);

			bt.clearData();
		}

		//
		osEvent event = osMessageGet(mainQueueId, 0);
		if (event.status == osEventMessage)
		{
			// message format: x--- abcd
			//   -x: updated target: adc, vario, timer, exti irq
			//   -abcd: updated data: target specific
			//

			uint32_t data = event.value.v & 0x0000FFFF;

			if (event.value.v & MQ_MSRC_ADCTASK) // message from adc-task
			{
				if (data & 1) // updated input voltage
				{
					TRACE("IVBUS,V,%u\r\n", devState.voltage);
				}

				if (data & 2)
				{
					TRACE("OVBUS,V,%u,C,%u,VS,%u\r\n", devState.vbusVoltage, devState.vbusCurrent, (uint32_t)(devState.shuntVoltage / 0.02));
				}
			}
			else if (event.value.v & MQ_MSRC_VARIOTASK) // message from vario-task
			{
#if DEBUG_BMP280 || 1
				TRACE("BMP280,T,%d,P,%d\r\n", (int)devState.temperature, (int)devState.pressure);
#endif
#if ENABLE_AHT20
				TRACE("AHT20,T,%d,H,%d\r\n", (int)devState.temperature, (int)devState.humidity);
#endif

				flushVarioSentence();
			}
			else if (event.value.v & MQ_MSRC_SHUTDOWNTIMER) // message from timer-task
			{
				TRACE("SHUTDOWN\r\n");
				osDelay(200);

				beacon.off(BeaconIndicator::BeaconType::Body);
				beacon.blink(BeaconIndicator::BeaconType::Board, 200);
				ioPin[IOPIN_POWER_DEVICE].off();
				/*
				ioPin[IOPIN_POWER_PERIPH].off();
				*/
				break;
			}
			else if (event.value.v & MQ_MSRC_EXTINTR) // message from exti irq
			{
				if (devState.btnUpdateMask & (1 << PB_BODY))
				{
					TRACE("BTN,BODY,%s\r\n", devState.btnState[PB_BODY] == PB_PRESSED ? "On" : "Off");
					devState.btnUpdateMask &= ~(1 << PB_BODY);

					if (devState.btnState[PB_BODY] == PB_PRESSED)
						osTimerStart(shutdownTimerId, 4000);
					else
						osTimerStop(shutdownTimerId);

				}

				if (devState.btnUpdateMask & (1 << PB_BOARD))
				{
					TRACE("BTN,BOARD,%s\r\n", devState.btnState[PB_BOARD] == PB_PRESSED ? "On" : "Off");
					devState.btnUpdateMask &= ~(1 << PB_BODY);
				}

				devState.btnUpdateMask = 0;
			}
		}

		if (lineBuf.hasCompleteLine() && lineBuf.getLength() > 0)
		{
			// parse NMEA sentence & forward to usb-serial and/or blue-tooth
			//
			// parse NMEA
			// ....
			//

#if USE_NMEAFILTER
			// filtering NMEA sentence
			const char *line = (const char *)lineBuf;
#if BLOCK_SATELLITE_INFO
			// $XXGSx : block GSV, GSA
			if (line[3] != 'G' || line[4] != 'S')
#elif BLOCK_TAKLERID_BD
			if (line[1] != 'B' || line[2] != 'D')
#endif
#endif
			{
#if USE_NMEAFILTER
#if FAKE_GN2GP
				if (line[1] == 'G' && line[2] == 'N')
				{
					char *ptr = (char *)line;
					ptr[2] = 'P';
				}
#endif
#endif

				if (SerialUSB)
					SerialUSB.puts(lineBuf, true);
				#if ROUTE_TO_BLUETOOTH
				bt.puts(lineBuf, true);
				#endif
			}

			lineBuf.reset();
		}
	}

	//
	while(1)
		osDelay(100);
}
