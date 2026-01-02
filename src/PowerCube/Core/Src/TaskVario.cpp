/*
 * TaskVario.cpp
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Task.h"

#include "HardwareSerial.h"
#include "BMP280.h"
#include "INA226.h"
#include "AHT20.H"
#include "BeaconIndicator.h"

#include "BluetoothHandler.h"
#include "LineBuffer.h"

#include "Barometer.h"
#include "VarioFilter.h"
#include "Variometer.h"
#include "VarioSentence.h"


//
//
//

extern I2CMaster i2c2;
#if 0
extern BMP280 baro;
#else
extern Bme280TwoWire baro;
#endif
extern AHT20 aht20;

extern DeviceState devState;

extern osMessageQId mainQueueId;


Variometer vario;



//
//
//

void VarioTaskProc(void const *arg)
{
	//
	osDelay(400);

	//
	baro.begin(Bme280TwoWireAddress::Secondary, &i2c2);

	Bme280Settings powerBaroSet = {
		.mode = Bme280Mode::Normal,
		.temperatureOversampling = Bme280Oversampling::X2,
		.pressureOversampling = Bme280Oversampling::X16,
		.humidityOversampling = Bme280Oversampling::X2,
		.filter = Bme280Filter::X16,
		.standbyTime = Bme280StandbyTime::Ms0_5
	};
	baro.setSettings(powerBaroSet);


#if ENABLE_AHT20
	//
	aht20.begin();
#endif

	vario.begin(CreateBarometer(), CreateVarioFilter());
	uint32_t lastTick = HAL_GetTick();

	while (1)
	{
		//
		if (vario.update() > 0)
		{
			devState.temperature = vario.getTemperature();
			devState.pressure = vario.getPressure();
			devState.varioSpeed = vario.getVelocity();

			//
			if (HAL_GetTick() - lastTick > 500)
			{
				uint32_t info = MQ_MSRC_VARIOTASK | 1; // vario-state changed
				osMessagePut(mainQueueId, info, osWaitForever);

				lastTick = HAL_GetTick();
			}
		}

		//
#if ENABLE_AHT20
		{
			devState.temperature = aht20.getTemperature();
			devState.humidity = aht20.getHumidity();
		}
#endif


		//
		osDelay(20);
	}
}

