/*
 * Task.cpp
 *
 *  Created on: 2025. 12. 9.
 *      Author: akkdong
 */


#include "main.h"
#include "cmsis_os.h"
#include "usbpd.h"
#include "usb_device.h"
#include "Task.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "HardwareSerial.h"
#include "BMP280.h"
#include "INA226.h"
#include "HUSB238.h"
#include "AHT20.H"

#include "BTHandler.h"


////////////////////////////////////////////////////////////////////////////
//

#ifdef __cplusplus
extern "C" {
#endif

void DefaultTaskProc(void const * argument);
void AdcTaskProc(void const * argument);

void ShutdownCallback(void const *arg);

#ifdef __cplusplus
}
#endif





////////////////////////////////////////////////////////////////////////////
//

extern ADC_HandleTypeDef hadc1;

extern I2CMaster i2c1;
extern I2CMaster i2c2;


INA226 ina226(i2c1);
HUSB238 husb238(i2c1);

BluetoothHandler bt(Serial3);


GPIO_PinState powerBtnState;
GPIO_PinState boardBtnState;
uint8_t btnStateChanged = 0; // BIT 0: Power, BIT 1: Board



#define MAX_RXSIZE	64

char rxData[MAX_RXSIZE];
uint8_t rxPos, txPos;

uint8_t btConnected = 0;




osThreadId mainTaskHandle;
osThreadId adcTaskHandle;

osSemaphoreId powerSemaphoreId;
osSemaphoreId boardSemaphoreId;

osTimerId shutdownTimerId;

uint32_t adcValue = 0;
uint32_t adcVoltage = 0; // mV

uint32_t vbusVoltage = 0;
uint32_t vbusCurrent = 0;
uint32_t shuntVoltage = 0;


//static osThreadDef(DefaultTask, DefaultTaskProc, osPriorityNormal, 0, 256);
static const osThreadDef_t os_thread_def_DefaultTask =
{
	(char *)"DefaultTask",
	DefaultTaskProc,
	osPriorityNormal,
	0,
	256
};

// static osThreadDef(AdcTask, AdcTaskProc, osPriorityNormal, 0, 256);
static const osThreadDef_t os_thread_def_AdcTask =
{
	(char *)"AdcTask",
	AdcTaskProc,
	osPriorityNormal,
	0,
	256
};

static osTimerDef(ShutdownTimer, ShutdownCallback);

/*
static osSemaphoreDef(PowerSemaphore);
static osSemaphoreDef(BoardSemaphore);
*/



////////////////////////////////////////////////////////////////////////////
//

#include <stdarg.h>

char traceBuf[64];
void Trace(const char *format, ...)
{
	if (btConnected)
	{
		va_list args;
		va_start(args, format);
		vsprintf(traceBuf, format, args);
		va_end(args);

		Serial3.puts(traceBuf);
	}
}

#ifdef DEBUG
#define TRACE(fmt, ...)	Trace(fmt, __VA_ARGS__)
#else
#define TRACE(fmt, ...)
#endif


////////////////////////////////////////////////////////////////////////////
//

void startRTOS()
{
	//
	mainTaskHandle = osThreadCreate(osThread(DefaultTask), NULL);
	adcTaskHandle = osThreadCreate(osThread(AdcTask), NULL);

	//
	shutdownTimerId = osTimerCreate(osTimer(ShutdownTimer), osTimerOnce, 0);

	//
	/*
	powerSemaphoreId = osSemaphoreCreate(osSemaphore(PowerSemaphore), 1);
	boardSemaphoreId = osSemaphoreCreate(osSemaphore(BoardSemaphore), 1);
	*/



	// Start scheduler
	osKernelStart();
}





////////////////////////////////////////////////////////////////////////////
//

Bme280TwoWire baro;
#if ENABLE_AHT20
	AHT20 aht20(i2c2);
#endif

void DefaultTaskProc(void const * argument)
{
	/* init code for USB_Device */
	//MX_USB_Device_Init();
	/* USER CODE BEGIN 5 */

	// LED on
	HAL_GPIO_WritePin(GPIOB, LED_DEVICERDY_Pin, GPIO_PIN_RESET);
	// DEVICEs on
	HAL_GPIO_WritePin(GPIOB, EN_EXTRAPWR_Pin, GPIO_PIN_RESET);
	// Hold PowerPin
	HAL_GPIO_WritePin(GPIOB, HOLD_POWER_Pin/*|VBUS_POWER_Pin*/, GPIO_PIN_SET);

	#if 0
	{
		bool ina226Ok = i2c1.deviceReady(0x40); 	// OK
		bool husb238Ok = i2c1.deviceReady(0x08); 	// OK
		bool bmp280Ok1 = i2c2.deviceReady(0x76);
		bool bmp280Ok2 = i2c2.deviceReady(0x77); 	// OK
		bool aht20Ok = i2c2.deviceReady(0x38);	// OK
		int i = 0;
		while (1)
		{
			++i;
			break;
		}
	}
	#endif

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

	// save latest button state
	powerBtnState = HAL_GPIO_ReadPin(PB_POWER_GPIO_Port, PB_POWER_Pin);
	boardBtnState = HAL_GPIO_ReadPin(PB_BOARD_GPIO_Port, PB_BOARD_Pin);

	//
	uint32_t tickCount = HAL_GetTick();
	uint8_t ledState = 0;
	uint32_t voltage = adcVoltage;
	uint32_t lastVBusVoltage = vbusVoltage;
	uint32_t lastVBusCurrent = vbusCurrent;
	float temperature = 0, pressure = 0;
#if ENABLE_AHT20
	float temp2 = 0, humidity = 0;
#endif

	rxData[0] = 0;
	rxPos = 0;
	txPos = 0;

	/* Infinite loop */
	for(;;)
	{
		//
		if (HAL_GetTick() - tickCount > 500)
		{
			ledState = 1 - ledState;
			HAL_GPIO_WritePin(GPIOB, LED_DEVICERDY_Pin, ledState > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_BOARD_GPIO_Port, LED_BOARD_Pin, ledState > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
			tickCount = HAL_GetTick();

			temperature = baro.getTemperature();
			pressure = baro.getPressure();

#if ENABLE_AHT20
			temp2 = aht20.getTemperature();
			humidity = aht20.getHumidity();
#endif
		}

		//
		if (Serial2.available() > 0)
		{
			int ch = Serial2.read();
			/*
			Serial3.write(ch);
			*/

			txPos = (/*ch == '\r' ||*/ ch == '\n') ? 0 : txPos + 1;
		}

		if (Serial3.available() > 0)
		{
			int ch = Serial3.read();

			if (ch == '\r' || ch == '\n')
			{
				if (strncmp(rxData, "CONNECT", 7) == 0)
					btConnected = 1;
				else if (strncmp(rxData, "DISCONNECT", 10) == 0)
					btConnected = 0;

				rxData[0] = 0;
				rxPos = 0;
			}
			else
			{
				if (rxPos < MAX_RXSIZE - 1)
				{
					rxData[rxPos] = ch;
					rxPos += 1;
					rxData[rxPos] = 0;
				}
			}
		}

		if (txPos == 0 /*&& btConnected*/)
		{
			// update voltage
			if (voltage != adcVoltage)
			{
				voltage = adcVoltage;

				Trace("VBUS_i: %lu\r\n", voltage);
				Trace("BMP280: T = %d, P = %d\r\n", (int)temperature, (int)pressure);

#if ENABLE_AHT20
				Trace("AHT20: T = %d, H = %d\r\n", (int)temp2, (int)humidity);
#endif
			}

			if (lastVBusVoltage != vbusVoltage)
			{
				lastVBusVoltage = vbusVoltage;
				lastVBusCurrent = vbusCurrent;

				Trace("VBUS_o: %lu, %lu, %lu\r\n", lastVBusVoltage, lastVBusCurrent, (uint32_t)(shuntVoltage / 0.02));
			}

			// check & update key-state
			if (btnStateChanged & 0x01)
			{
				powerBtnState = HAL_GPIO_ReadPin(PB_POWER_GPIO_Port, PB_POWER_Pin);
				btnStateChanged &= ~0x01;

				if (powerBtnState == GPIO_PIN_RESET)
				{
					osTimerStart(shutdownTimerId, 5000);
				}
				else
				{
					osTimerStop(shutdownTimerId);
				}

				Trace("Power Button: %s\r\n", powerBtnState == GPIO_PIN_RESET ? "On" : "Off");
			}

			if (btnStateChanged & 0x02)
			{
				boardBtnState = HAL_GPIO_ReadPin(PB_BOARD_GPIO_Port, PB_BOARD_Pin);
				btnStateChanged &= ~0x02;

				Trace("Board Button: %s\r\n", boardBtnState == GPIO_PIN_RESET ? "Off" : "On");
			}
		}
	}
}



void AdcTaskProc(void const * argument)
{
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adcValue, 1);


	ina226.begin();
#if 1
	ina226.setAverage(INA226_64_SAMPLES);
	ina226.setBusVoltageConversionTime(INA226_4200_us);
	//ina226.setMode(7); // shunt-bus-continuous
#else
	ina226.setConfiguration(INA226_64_SAMPLES, INA226_2100_us, INA226_2100_us, INA266_MODE_SHUNTBUSCONT);
#endif
	ina226.calibrate(0.02, 0.025, 0.120);

	uint32_t lastTick = HAL_GetTick();
	bool husb238Attached = false;

	while (1)
	{
		if (HAL_GetTick() - lastTick > 1000)
		{
			// adcValue : 0x0FFF = adcVoltage : 3300 --> acqVoltage = adcValue * 3300 / 0x0FFF
			float acqVoltage = (float)adcValue * 3300 / 0x0FFF;
			// acqVoltage = adcVoltage * 40.2 / 240.2;
			adcVoltage = acqVoltage * 240.2 / 40.2;

			if (adcVoltage > 1000 && !husb238Attached)
			{
				HUSB238::Capability *pCap = husb238.getCapabilities();
				char line[32];

				sprintf(line, "5V : %u\r\n", pCap->ma_5V);
				Serial3.puts(line);
				sprintf(line, "9V : %u\r\n", pCap->ma_9V);
				Serial3.puts(line);
				sprintf(line, "12V : %u\r\n", pCap->ma_12V);
				Serial3.puts(line);
				sprintf(line, "15V : %u\r\n", pCap->ma_15V);
				Serial3.puts(line);
				sprintf(line, "18V : %u\r\n", pCap->ma_18V);
				Serial3.puts(line);
				sprintf(line, "20V : %u\r\n", pCap->ma_20V);
				Serial3.puts(line);

				husb238Attached = true;
				//HAL_GPIO_WritePin(GPIOB, VBUS_POWER_Pin, GPIO_PIN_SET);

				//
				husb238.updateStatus();
				uint8_t v = husb238.getActiveVoltage(false);
				uint16_t c = husb238.getActiveCurrent(false);
				sprintf(line, ">> %u V, %u mA\r\n", v, c);
				Serial3.puts(line);

				//
#if 0
				osDelay(1000);
				husb238.setVoltage(HUSB238::PDO_9V);
				osDelay(1000);
				husb238.updateStatus();
				v = husb238.getActiveVoltage(false);
				c = husb238.getActiveCurrent(false);
				sprintf(line, "<< %u V, %u mA\r\n", v, c);
				Serial3.puts(line);
#endif
			}
			else if (adcVoltage < 1000 && husb238Attached)
			{
				husb238Attached = false;
				HAL_GPIO_WritePin(GPIOB, VBUS_POWER_Pin, GPIO_PIN_RESET);
			}

			//
			vbusVoltage = (uint32_t)ina226.getBusVoltage_mV();
			vbusCurrent = (uint32_t)ina226.getCurrent_mA();
			shuntVoltage = (uint32_t)ina226.getShuntVoltage_mV();

			//
			lastTick = HAL_GetTick();
		}

		//
	}
}







//
//
//

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == PB_POWER_Pin)
	{
		GPIO_PinState state = HAL_GPIO_ReadPin(PB_POWER_GPIO_Port, PB_POWER_Pin);
		if (powerBtnState != state)
		{
			//osSemaphoreRelease(powerSemaphoreId);
			btnStateChanged |= 0x01;
		}
	}
	else if (GPIO_Pin == PB_BOARD_Pin)
	{
		GPIO_PinState state = HAL_GPIO_ReadPin(PB_BOARD_GPIO_Port, PB_BOARD_Pin);
		if (boardBtnState != state)
		{
			//osSemaphoreRelease(boardSemaphoreId);
			btnStateChanged |= 0x02;
		}
	}
}


void ShutdownCallback(void const *arg)
{
	//osTimerStop(shutdownTimerId);

	Trace("SHUTDOWN!!\r\n");
	osDelay(200);
	HAL_GPIO_WritePin(GPIOB, HOLD_POWER_Pin, GPIO_PIN_RESET);
}
