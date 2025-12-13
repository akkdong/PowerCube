/*
 * Variometer.cpp
 *
 *
 */

#include <math.h>

#include "Variometer.h"
#include "logger.h"
#include "utils.h"


///////////////////////////////////////////////////////////////////////////////////////////////
// class Variometer

Variometer::Variometer()
    : baroSensor(nullptr)
    , varioFilter(nullptr)
    , seaLevel(101325)
    , altitudeDrift(0)
{
}

int Variometer::begin(IBarometer* baro, IVarioFilter* filter)
{
    baroSensor = baro;
    varioFilter = filter;

    pressure = seaLevel;
    temperature = 0;
    altitudeDrift = 0;
    altitude = 0;
    altitudeFiltered = 0;
    vario = 0;

    updateCount = 0;
    lastUpdateTick = millis();

    return 0;
}

void Variometer::end()
{
}

int Variometer::update()
{
    uint32_t tick = millis();
    if (tick - lastUpdateTick < 1000 / 25)
        return -1;
    lastUpdateTick = tick;

    if (measure() < 0)
        return -1;

    updateInternal();

    return 1; // updated(valid)
}

int Variometer::updatePeriodic()
{
    if (measure() < 0)
        return -1;

    updateInternal();

    return 1; //valid
}

void Variometer::resetUpdate()
{
}

void Variometer::calibrateSeaLevel(float altitudeRef)
{
    seaLevel = calculateSeaLevel(pressure, temperature, altitudeRef);

    updateCount = 0;
}

float Variometer::calculateSeaLevel(float pressure, float temperature, float altitude)
{
    // seaLevel0 = pressure / pow(1.0 - (altitude / 44330.0), 5.257);
    // 44330 = (15 + 273.15) / 0.0065
    //
    // seaLevel1 = pressure * pow((1 - (0.0065 * altitude) / (0.0065 * altitude + temperature + 173.15)), -5.257);
    //
    // 1000 m, 980 hPa, 15 C
    //
    // seaLevel0
    //   980 / ((1 - 1000 / ((15 + 273.15) / 0.0065)) ^ 5.255) = 1104.834376914360709808865911983
    // seaLevel1
    //   980 / ((1 - (0.0065 * 1000) / (15 + (0.0065 * 1000) + 273.15)) ^ 5.257) = 1101.9324015827903952343510590759
    //   980 * ((1 - (0.0065 * 1000) / (15 + (0.0065 * 1000) + 273.15)) ^ -5.257) = 1101.9324015827903952343510590759

    return pressure * pow(1 - (altitude * 0.0065) / ((altitude * 0.0065) + (temperature + 273.15)), -5.257);
}

int Variometer::measure()
{
    float p, t;
    if (!baroSensor->getData(&p, &t))
        return -1;

    pressure = p;
    temperature = t;

    return 0; // updated
}

void Variometer::updateInternal()
{
    // h1 = ((Psea / P) ^ (1 / 5.25579) - 1) * (T + 273.15) / 0.0065
    // h2 = (1 - (P / Psea) ^  (1 / 5.25579)) * (T + 273.15) / 0.0065
    //
    // P = 100000, Psea = 101325, T = 10
    // h1 = ((101325 / 100000) ^ (1 / 5.25579) - 1) * (10 + 273.15) / 0.0065 = 109.23544816447158227324734896528
    // h2 = (1 - (100000 / 101325) ^  (1 / 5.25579)) * (10 + 273.15) / 0.0065 = 108.96221318218422287260605269415
    //
    // 173.14 : Kelvin temperature
    // 0.0065 : standard lapse rate
    // 5.25579 : gM/RL, (g: gravitational acceleration, R : universal gas constance, M: molar mass, L: standard lapse rate)
    // g = 9.80665
    // R = 8.3144598
    // M = 0.0289644
    // L = -0.0065
    // gM/R*L = 9.80665 * 0.0289644 / 8.3144598 / -0.0065 = -5.2557877405521698660261913643691

    altitude = (1.0 - pow(pressure / seaLevel, 1 / 5.25579)) * ((temperature + 273.15) / 0.0065);

    //
    if (updateCount == 0)
    {
        varioFilter->reset(altitude);
        updateCount = 1;
    }

    float altitude_ = 0.0f, vario_ = 0.0f;
    varioFilter->update(altitude, 0, &altitude_, &vario_);

    if (!isnan(altitude_) && !isnan(vario_))
    {
        altitudeFiltered = altitude_;
        vario = vario_;
    }
    else
    {
        // something is going wrong, reset!
        LOGe("Something is going wrong!! Reset vario filter.");
        updateCount = 0;
    }
}
