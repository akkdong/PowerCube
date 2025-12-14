/*
 * TaskAdc.cpp
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Task.h"

#include "INA226.h"
#include "HUSB238.h"
#include "ADCReader.h"




//
//
//

extern ADC_HandleTypeDef hadc1;

extern INA226 ina226;
extern HUSB238 husb238;
extern ADCReader adc;

extern DeviceState devState;

extern osMessageQId mainQueueId;


uint32_t adcValue = 0;
uint32_t adcVoltage = 0; // mV

uint32_t vbusVoltage = 0;
uint32_t vbusCurrent = 0;
uint32_t shuntVoltage = 0;



//
// ADC Task
//

void AdcTaskProc(void const * argument)
{
	//
	adc.start();

	//
	ina226.begin();
#if 1
	ina226.setAverage(INA226_64_SAMPLES);
	ina226.setBusVoltageConversionTime(INA226_4200_us);
	//ina226.setMode(7); // shunt-bus-continuous
#else
	ina226.setConfiguration(INA226_64_SAMPLES, INA226_2100_us, INA226_2100_us, INA266_MODE_SHUNTBUSCONT);
#endif
	ina226.calibrate(0.02, 0.025, 0.120);

	//
	husb238.begin();


	//
	bool powerAttached = false;

	while (1)
	{
		//
		float voltage = adc.getVoltage();
		if (devState.voltage != voltage)
		{
			//
			if (!powerAttached && voltage > 1500)
			{
				HUSB238::Capability *pCap = husb238.getCapabilities();
				TRACE("USBPD,CAP,5V,%umA\r\n", pCap->ma_5V);
				devState.po[PO_5V].voltage = 5;
				devState.po[PO_5V].current = pCap->ma_5V;
				TRACE("USBPD,CAP,9V,%umA\r\n", pCap->ma_9V);
				devState.po[PO_5V].voltage = 9;
				devState.po[PO_5V].current = pCap->ma_9V;
				TRACE("USBPD,CAP,12V,%umA\r\n", pCap->ma_12V);
				devState.po[PO_5V].voltage = 12;
				devState.po[PO_5V].current = pCap->ma_12V;
				TRACE("USBPD,CAP,15V,%umA\r\n", pCap->ma_15V);
				devState.po[PO_5V].voltage = 15;
				devState.po[PO_5V].current = pCap->ma_15V;
				TRACE("USBPD,CAP,18V,%umA\r\n", pCap->ma_18V);
				devState.po[PO_5V].voltage = 18;
				devState.po[PO_5V].current = pCap->ma_18V;
				TRACE("USBPD,CAP,20V,%umA\r\n", pCap->ma_20V);
				devState.po[PO_5V].voltage = 20;
				devState.po[PO_5V].current = pCap->ma_20V;

				//
				husb238.updateStatus();
				devState.po[PO_ACTIVE].voltage = husb238.getActiveVoltage(false);
				devState.po[PO_ACTIVE].current = husb238.getActiveCurrent(false);
				TRACE("USBPD,ACTIVE,%uV,%umA\r\n", devState.voltage, devState.current);

				//
#if 0 // TEST: changing voltage
				osDelay(1000);
				husb238.setVoltage(HUSB238::PDO_9V);
				osDelay(1000);
				husb238.updateStatus();
				uint32_t v = husb238.getActiveVoltage(false);
				uint32_t c = husb238.getActiveCurrent(false);
				TRACE("USBPD,ACTIVE,%uV,%umA\r\n", v, c);
#endif

				powerAttached = true;
			}
			else if (powerAttached && voltage < 1500)
			{
				memset(&devState.po[0], 0, sizeof(devState.po));

				powerAttached = false;
			}

			//
			devState.voltage = voltage;

			uint32_t info = MQ_MSRC_ADCTASK | 1; // input-voltage changed
			osMessagePut(mainQueueId, info, osWaitForever);
		}

		//
		uint32_t vbusVoltage = (uint32_t)ina226.getBusVoltage_mV();
		uint32_t vbusCurrent = (uint32_t)ina226.getCurrent_mA();
		/*
		uint32_t shuntVoltage = (uint32_t)ina226.getShuntVoltage_mV();
		*/
		if (vbusVoltage != devState.vbusVoltage || vbusCurrent != devState.vbusCurrent)
		{
			devState.vbusVoltage = vbusVoltage;
			devState.vbusCurrent = vbusCurrent;
			/*
			devState.shuntVoltage = shuntVoltage;
			*/

			//
			uint32_t info = MQ_MSRC_ADCTASK | 2; // output-voltage changed
			osMessagePut(mainQueueId, info, osWaitForever);
		}

		//
		osDelay(1000);
	}
}


