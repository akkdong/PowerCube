/*
 * BeaconIndicator.cpp
 *
 *
 */

#include "BeaconIndicator.h"



//
//
//

static osMutexDef(Beacon);



//
// BeaconIndicator
//
//

BeaconIndicator::BeaconIndicator()
{
	beaconTimer = NULL;
	beaconMutex = NULL;
}

void BeaconIndicator::begin()
{
	// default LED state
	Beacon[Body] = { BeaconMode::OFF, 0, 0, &ioPin[IOPIN_LED_BODY] };
	Beacon[Board] = { BeaconMode::OFF, 0, 0, &ioPin[IOPIN_LED_BOARD] };

	for (int i = 0; i < MaxBeacon; ++i)
	{
		if (Beacon[i].mode == BeaconMode::OFF)
			Beacon[i].pin->off();
		else
			Beacon[i].pin->on();
	}

	//
	beaconTimer = osTimerCreate(osTimer(Beacon), osTimerPeriodic, this);
	if (beaconTimer)
	{
		osStatus status = osTimerStart(beaconTimer, beaconDefaultInterval);
		if (status != osOK)
		{
			osTimerDelete(beaconTimer);
			beaconTimer = NULL;
		}
	}

	beaconMutex = osMutexCreate(osMutex(Beacon));
}

void BeaconIndicator::end()
{
	if (beaconMutex)
	{
		osMutexDelete(beaconMutex);
		beaconMutex = NULL;
	}

	if (beaconTimer)
	{
		osTimerDelete(beaconTimer);
		beaconTimer = NULL;
	}

	for (int i = 0; i < MaxBeacon; ++i)
		Beacon[i].pin->on();
}



void BeaconIndicator::on(BeaconType beacon)
{
	if (beaconMutex)
		osMutexWait(beaconMutex, osWaitForever);

	Beacon[beacon].mode = BeaconMode::ON;
	Beacon[beacon].pin->on();

	if (beaconMutex)
		osMutexRelease(beaconMutex);
}

void BeaconIndicator::off(BeaconType beacon)
{
	if (beaconMutex)
		osMutexWait(beaconMutex, osWaitForever);

	Beacon[beacon].mode = BeaconMode::OFF;
	Beacon[beacon].pin->off();

	if (beaconMutex)
		osMutexRelease(beaconMutex);
}

void BeaconIndicator::blink(BeaconType beacon, uint32_t interval)
{
	if (beaconMutex)
		osMutexWait(beaconMutex, osWaitForever);

	Beacon[beacon].mode = BeaconMode::BLINK;
	Beacon[beacon].interval = interval;
	Beacon[beacon].count = 0;
	Beacon[beacon].pin->on();

	if (beaconMutex)
		osMutexRelease(beaconMutex);
}




#include "timers.h"

void BeaconIndicator::TimerCallback(void const *arg)
{
	TimerHandle_t pHandle = (TimerHandle_t)arg;
	BeaconIndicator *pThis = (BeaconIndicator *)pvTimerGetTimerID(pHandle);

	if (pThis->beaconMutex)
		osMutexWait(pThis->beaconMutex, osWaitForever);

	for (int i = 0; i < MaxBeacon; ++i)
	{
		if (pThis->Beacon[i].mode == BeaconMode::BLINK)
		{
			pThis->Beacon[i].count += pThis->beaconDefaultInterval;
			if (pThis->Beacon[i].count >= pThis->Beacon[i].interval)
			{
				pThis->Beacon[i].pin->toggle();
				pThis->Beacon[i].count = 0;
			}
		}

	}

	if (pThis->beaconMutex)
		osMutexRelease(pThis->beaconMutex);
}
