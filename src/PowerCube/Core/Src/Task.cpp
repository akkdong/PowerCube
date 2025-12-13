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
#include "IOPin.h"
#include "BeaconIndicator.h"

#include "BluetoothHandler.h"
#include "LineBuffer.h"


#define MQ_MSRC_ADCTASK				0x10000000
#define MQ_MSRC_VARIOTASK			0x20000000
#define MQ_MSRC_SHUTDOWNTIMER		0x40000000
#define MQ_MSRC_EXTINTR				0x80000000




////////////////////////////////////////////////////////////////////////////
//

#ifdef __cplusplus
extern "C" {
#endif

void MainTaskProc(void const *arg);
void AdcTaskProc(void const *arg);
void VarioTaskProc(void const *arg);

void ShutdownTimerCallback(void const *arg);

#ifdef __cplusplus
}
#endif





////////////////////////////////////////////////////////////////////////////
//

extern ADC_HandleTypeDef hadc1;

extern I2CMaster i2c1;
extern I2CMaster i2c2;


//

INA226 ina226(i2c1);
HUSB238 husb238(i2c1);

BluetoothHandler bt(Serial3);
BeaconIndicator beacon;

DeviceState devState;


//GPIO_PinState powerBtnState;
//GPIO_PinState boardBtnState;
//uint8_t btnStateChanged = 0; // BIT 0: Power, BIT 1: Board


const int maxBufSize = 64;
char bufSerial2[maxBufSize];

LineBuffer lineBuf(bufSerial2, maxBufSize);



//char rxData[maxBufSize];
//uint8_t rxPos, txPos;

//uint8_t btConnected = 0;

// read GPS
// read sensor
// update Vario state
// update GPS state
// make NMEA sentence
//
//
// LK8EX1 sentence : pressure, altitude, vertical-speed, temperature, battery
//
// GPS ---< >-----> BT and/or USB
//         |
//    {simulation}
//         |
// BT  ----+------> configuration, control, ...
//
//
// DEVICE RUN_MODE
//   STANDBY
//   RUN
//   SHUTDOWN
//
// EVENT: POWER-ON, POWER-KEY-PRESS/RELEASE, BOARD-KEY-PRESS/RELEASE,  BT-CONNECTED/DISCONNECTRED, USB-CONNECTED/DISCONECTED
//
// STATE
//     date/time, latitude, longitude, altitude, horizontal-speed
//     pressure, temperature, vertical-speed, (humidity ?)
//     input-voltage, output-voltage, current
//	   power-button state, board-button state
//	   led1-state, led2-state
//	   bt-state, vbus-state, usb-state
//
//     pdo-list, active-pdo



osThreadId mainTaskHandle;
osThreadId adcTaskHandle;

osSemaphoreId powerSemaphoreId;
osSemaphoreId boardSemaphoreId;

osTimerId shutdownTimerId;

osMessageQDef(MainQueue, 8, uint32_t);
osMessageQId mainQueueId;

/*
uint32_t adcValue = 0;
uint32_t adcVoltage = 0; // mV

uint32_t vbusVoltage = 0;
uint32_t vbusCurrent = 0;
uint32_t shuntVoltage = 0;
*/

//static osThreadDef(MainTask, MainTaskProc, osPriorityNormal, 0, 256);
static const osThreadDef_t os_thread_def_MainTask =
{
	(char *)"MainTask",
	MainTaskProc,
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

// static osThreadDef(VarioTask, VarioTaskProc, osPriorityNormal, 0, 256);
static const osThreadDef_t os_thread_def_VarioTask =
{
	(char *)"VarioTask",
	VarioTaskProc,
	osPriorityNormal,
	0,
	256
};

static osTimerDef(ShutdownTimer, ShutdownTimerCallback);

/*
static osSemaphoreDef(PowerSemaphore);
static osSemaphoreDef(BoardSemaphore);
*/



////////////////////////////////////////////////////////////////////////////
//

#include <stdarg.h>

void Trace(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	bt.printf(format, args);
	va_end(args);
}

#ifdef DEBUG
#define TRACE(fmt, ...)	Trace(fmt, __VA_ARGS__)
#else
#define TRACE(fmt, ...)
#endif


// LED beacon indicator
//   - flash LED : body, board
//   - off: ready to shutdown
//   - on: device ready
//   - short blink: device running
//   - long blink: body only, usb-power activated & usb-host connected
//


////////////////////////////////////////////////////////////////////////////
//

void startRTOS()
{
	//
	mainTaskHandle = osThreadCreate(osThread(MainTask), NULL);
	adcTaskHandle = osThreadCreate(osThread(AdcTask), NULL);

	//
	shutdownTimerId = osTimerCreate(osTimer(ShutdownTimer), osTimerOnce, 0);

	//
	/*
	powerSemaphoreId = osSemaphoreCreate(osSemaphore(PowerSemaphore), 1);
	boardSemaphoreId = osSemaphoreCreate(osSemaphore(BoardSemaphore), 1);
	*/

	mainQueueId = osMessageCreate(osMessageQ(MainQueue), NULL);

	// Start scheduler
	osKernelStart();
}





////////////////////////////////////////////////////////////////////////////
//

Bme280TwoWire baro;
#if ENABLE_AHT20
	AHT20 aht20(i2c2);
#endif

void MainTaskProc(void const * argument)
{
	//
	beacon.begin();
	beacon.off(BeaconIndicator::BeaconType::Body);
	beacon.off(BeaconIndicator::BeaconType::Board);

	ioPin[IOPIN_POWER_DEVICE].on();
	ioPin[IOPIN_POWER_PERIPH].on();
	ioPin[IOPIN_POWER_VBUS].off();

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

	bt.begin();


	//
	memset(&devState, 0, sizeof(devState));

	devState.btnState[PB_BODY] = ioPin[IOPIN_BODY_INPUT].isOn() ? PB_PRESSED : PB_RELEASED;
	devState.btnState[PB_BOARD] = ioPin[IOPIN_BOARD_INPUT].isOn() ? PB_PRESSED : PB_RELEASED;

	lineBuf.reset();

	//
	uint32_t tickCount = HAL_GetTick();

	beacon.on(BeaconIndicator::BeaconType::Body);
	beacon.blink(BeaconIndicator::BeaconType::Board, 500);


	/* Infinite loop */
	for(;;)
	{
		//
		if (HAL_GetTick() - tickCount > 500)
		{
			//ledState = 1 - ledState;
			//HAL_GPIO_WritePin(LED_DEVICERDY_GPIO_Port, LED_DEVICERDY_Pin, ledState > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(LED_BOARD_GPIO_Port, LED_BOARD_Pin, ledState > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

			tickCount = HAL_GetTick();

			devState.temperature = baro.getTemperature();
			devState.pressure = baro.getPressure();

#if ENABLE_AHT20
			devState.temperature = aht20.getTemperature();
			devState.humidity = aht20.getHumidity();
#endif
		}

		//
		if (Serial2.available() > 0)
		{
			int ch = Serial2.read();
			lineBuf.push(ch);
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
					TRACE("VBUS_i: %u\r\n", devState.voltage);
				}

				if (data & 2)
				{
					TRACE("VBUS_o: %u, %u, %u\r\n", devState.vbusVoltage, devState.vbusCurrent, (uint32_t)(devState.shuntVoltage / 0.02));
				}
			}
			else if (event.value.v & MQ_MSRC_VARIOTASK) // message from vario-task
			{
				TRACE("BMP280: T = %d, P = %d\r\n", (int)devState.temperature, (int)devState.pressure);

#if ENABLE_AHT20
				Trace("AHT20: T = %d, H = %d\r\n", (int)devState.temperature, (int)devState.humidity);
#endif
			}
			else if (event.value.v & MQ_MSRC_SHUTDOWNTIMER) // message from timer-task
			{
				TRACE("SHUTDOWN!!\r\n", 0);
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
					TRACE("BODY Button: %s\r\n", devState.btnState[PB_BODY] == PB_PRESSED ? "On" : "Off");
					devState.btnUpdateMask &= ~(1 << PB_BODY);

					if (devState.btnState[PB_BODY] == PB_PRESSED)
						osTimerStart(shutdownTimerId, 4000);
					else
						osTimerStop(shutdownTimerId);

				}

				if (devState.btnUpdateMask & (1 << PB_BOARD))
				{
					TRACE("BOARD Button: %s\r\n", devState.btnState[PB_BOARD] == PB_PRESSED ? "On" : "Off");
					devState.btnUpdateMask &= ~(1 << PB_BODY);
				}
			}
		}

		if (lineBuf.hasCompleteLine() && lineBuf.getLength() > 0)
		{
			// parse NMEA sentence & forward to usb-serial and/or blue-tooth
			//
			// parse NMEA
			// ....
			//

			/*
			cdc.puts(lineBuf, true);
			*/
			bt.puts(lineBuf, true);

			lineBuf.reset();
		}
	}

	//
	while(1)
		osDelay(100);
}




//
// ADC Task
//

uint32_t adcValue = 0;
uint32_t adcVoltage = 0; // mV

uint32_t vbusVoltage = 0;
uint32_t vbusCurrent = 0;
uint32_t shuntVoltage = 0;


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
				TRACE("5V : %u\r\n", pCap->ma_5V);
				TRACE("9V : %u\r\n", pCap->ma_9V);
				TRACE("12V : %u\r\n", pCap->ma_12V);
				TRACE("15V : %u\r\n", pCap->ma_15V);
				TRACE("18V : %u\r\n", pCap->ma_18V);
				TRACE("20V : %u\r\n", pCap->ma_20V);

				husb238Attached = true;
				//HAL_GPIO_WritePin(VBUS_POWER_GPIO_Port, VBUS_POWER_Pin, GPIO_PIN_SET);

				//
				husb238.updateStatus();
				devState.voltage = husb238.getActiveVoltage(false);
				devState.current = husb238.getActiveCurrent(false);
				TRACE(">> %u V, %u mA\r\n", devState.voltage, devState.current);

				//
#if 0 // TEST: changing voltage
				osDelay(1000);
				husb238.setVoltage(HUSB238::PDO_9V);
				osDelay(1000);
				husb238.updateStatus();
				uint32_t v = husb238.getActiveVoltage(false);
				uint32_t c = husb238.getActiveCurrent(false);
				TRACE("<< %u V, %u mA\r\n", v, c);
#endif
			}
			else if (adcVoltage < 1000 && husb238Attached)
			{
				husb238Attached = false;
				//HAL_GPIO_WritePin(VBUS_POWER_GPIO_Port, VBUS_POWER_Pin, GPIO_PIN_RESET);
			}

			//
			vbusVoltage = (uint32_t)ina226.getBusVoltage_mV();
			vbusCurrent = (uint32_t)ina226.getCurrent_mA();
			shuntVoltage = (uint32_t)ina226.getShuntVoltage_mV();

			//
			lastTick = HAL_GetTick();

			uint32_t info = MQ_MSRC_ADCTASK | 3;
			osMessagePut(mainQueueId, info, osWaitForever);
		}

		//
	}
}

void VarioTaskProc(void const *arg)
{
	while (1)
	{
		osDelay(100);
	}
}





//
//
//

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == PB_POWER_Pin)
	{
		devState.btnState[PB_BODY] = ioPin[IOPIN_BODY_INPUT].isOn() ? PB_PRESSED : PB_RELEASED;
		devState.btnUpdateMask |= (1 << PB_BODY);

		uint32_t info = MQ_MSRC_EXTINTR | (1 << PB_BODY);
		osMessagePut(mainQueueId, info, osWaitForever);
	}
	else if (GPIO_Pin == PB_BOARD_Pin)
	{
		devState.btnState[PB_BOARD] = ioPin[IOPIN_BOARD_INPUT].isOn() ? PB_PRESSED : PB_RELEASED;
		devState.btnUpdateMask |= (1 << PB_BOARD);

		/*
		GPIO_PinState state = HAL_GPIO_ReadPin(PB_BOARD_GPIO_Port, PB_BOARD_Pin);
		if (boardBtnState != state)
		{
			//osSemaphoreRelease(boardSemaphoreId);
			btnStateChanged |= 0x02;
		}
		*/

		uint32_t info = MQ_MSRC_EXTINTR | (1 << PB_BOARD);
		osMessagePut(mainQueueId, info, osWaitForever);
	}
}


void ShutdownTimerCallback(void const *arg)
{
	/*
	//osTimerStop(shutdownTimerId);

	Trace("SHUTDOWN!!\r\n");
	osDelay(200);
	//HAL_GPIO_WritePin(GPIOB, HOLD_POWER_Pin, GPIO_PIN_RESET);
	beacon.off(BeaconIndicator::BeaconType::Body);
	ioPin[IOPIN_POWER_DEVICE].off();
	ioPin[IOPIN_POWER_PERIPH].off();
	*/

	uint32_t info = MQ_MSRC_SHUTDOWNTIMER;
	osMessagePut(mainQueueId, info, osWaitForever);
}
