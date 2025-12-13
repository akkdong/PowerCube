/*
 * BeaconIndicator.h
 *
 *
 */

#ifndef BSP_INC_BEACONINDICATOR_H_
#define BSP_INC_BEACONINDICATOR_H_

#include "cmsis_os.h"
#include "IOPin.h"


//
// BeaconIndicator
//
//

class BeaconIndicator
{
public:
	BeaconIndicator();

	enum BeaconType {
		Body,
		Board,
		MaxBeacon,
	};

	enum BeaconMode {
		OFF,
		ON,
		BLINK
	};

	struct BeaconState {
		BeaconMode mode;
		uint32_t interval;
		uint32_t count;

		IOPin *pin;
	};

	void begin();
	void end();

	void on(BeaconType beacon);
	void off(BeaconType beacon);
	void blink(BeaconType beacon, uint32_t interval);

protected:
	static void TimerCallback(void const *arg);

protected:
	// osTimerDef(Beacon, TimerCallback);
	const osTimerDef_t os_timer_def_Beacon = { TimerCallback };
	const uint32_t beaconDefaultInterval = 200;

	osTimerId beaconTimer;
	osMutexId beaconMutex;

	//
	BeaconState Beacon[MaxBeacon];
};


#endif /* BSP_INC_BEACONINDICATOR_H_ */
