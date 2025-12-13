// Barometer.cpp
//

#include "Barometer.h"
#include "BMP280.h"
#include "logger.h"


///////////////////////////////////////////////////////////////////////////////////////////////
// class Barometer declaration

class Barometer : public IBarometer
{
public:
    Barometer();

public:
    bool getData(float* p, float* t);

protected:
    #if USE_UPDATE_TICK
    uint32_t    lastTick;
    #endif
};



extern Bme280TwoWire baro;



///////////////////////////////////////////////////////////////////////////////////////////////
// class Barometer implementation

Barometer::Barometer()
    #if USE_UPDATE_TICK
    : lastTick(millis())
    #endif
{
}

bool Barometer::getData(float* p, float* t)
{
    //
    #if USE_UPDATE_TICK
    uint32_t tick = millis();
    if (tick - lastTick < 1000 / 25)
        return false;
    lastTick = tick;
    #endif

    // enter-critical-section
    *t = baro.getTemperature();
    *p = baro.getPressure();
    // leave-critical-section

    return true;
}




///////////////////////////////////////////////////////////////////////////////////////////////
//
/*
static Bme280Settings varioSettings()
{
	return {
		.mode = Bme280Mode::Normal,
		.temperatureOversampling = Bme280Oversampling::X2,
		.pressureOversampling = Bme280Oversampling::X16,
		.humidityOversampling = Bme280Oversampling::X2,
		.filter = Bme280Filter::X16,
		.standbyTime = Bme280StandbyTime::Ms0_5
	};
}
*/

IBarometer* CreateBarometer()
{
    static Barometer _baro;
    return &_baro;
}
