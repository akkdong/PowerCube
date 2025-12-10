/*
 * INA266.h
 *
 *  Created on: 2025. 12. 7.
 *      Author: akkdong
 */

#ifndef BSP_INC_INA226_H_
#define BSP_INC_INA226_H_

#include "I2C.h"


////////////////////////////////////////////////////////////////////////////////////////
//

//  set by setAlertRegister
#define INA226_SHUNT_OVER_VOLTAGE         0x8000
#define INA226_SHUNT_UNDER_VOLTAGE        0x4000
#define INA226_BUS_OVER_VOLTAGE           0x2000
#define INA226_BUS_UNDER_VOLTAGE          0x1000
#define INA226_POWER_OVER_LIMIT           0x0800
#define INA226_CONVERSION_READY           0x0400


//  returned by getAlertFlag
#define INA226_ALERT_FUNCTION_FLAG        0x0010
#define INA226_CONVERSION_READY_FLAG      0x0008
#define INA226_MATH_OVERFLOW_FLAG         0x0004
#define INA226_ALERT_POLARITY_FLAG        0x0002
#define INA226_ALERT_LATCH_ENABLE_FLAG    0x0001


//  returned by setMaxCurrentShunt
#define INA226_ERR_NONE                   0x0000
#define INA226_ERR_SHUNTVOLTAGE_HIGH      0x8000
#define INA226_ERR_MAXCURRENT_LOW         0x8001
#define INA226_ERR_SHUNT_LOW              0x8002
#define INA226_ERR_NORMALIZE_FAILED       0x8003

//  See issue #26
#ifndef INA226_MINIMAL_SHUNT_OHM
#define INA226_MINIMAL_SHUNT_OHM          0.001
#endif

#define INA226_MAX_WAIT_MS                600   //  millis

#define INA226_MAX_SHUNT_VOLTAGE          (81.92 / 1000)


//  for setAverage() and getAverage()
enum ina226_average_enum {
    INA226_1_SAMPLE     = 0,		// default
    INA226_4_SAMPLES    = 1,
    INA226_16_SAMPLES   = 2,
    INA226_64_SAMPLES   = 3,
    INA226_128_SAMPLES  = 4,
    INA226_256_SAMPLES  = 5,
    INA226_512_SAMPLES  = 6,
    INA226_1024_SAMPLES = 7
};


//  for BVCT and SVCT conversion timing.
enum ina226_timing_enum {
    INA226_140_us  = 0,
    INA226_204_us  = 1,
    INA226_332_us  = 2,
    INA226_588_us  = 3,
    INA226_1100_us = 4,				// default
    INA226_2100_us = 5,
    INA226_4200_us = 6,
    INA226_8300_us = 7
};

// for Mode Settings
enum in226_mode_settings_enum {
	INA266_MODE_POWERDOWN = 0,
	INA266_MODE_SHUNTVOLTGETRIG = 1,
	INA266_MODE_BUSVOLTAGETRIG = 2,
	INA266_MODE_SHUNTANDBUSTRIG = 3,
	INA266_MODE_POWERDOWN2 = 4,
	INA266_MODE_SHUNTVOLTAGECONT = 5,
	INA266_MODE_BUSVOLTAGECONT = 6,
	INA266_MODE_SHUNTBUSCONT = 7		// default
};


//  ALERT Pin Polarity definition
enum ina226_alert_pin_polarity_enum {
    INA226_ACTIVE_LOW  = 0,
    INA226_ACTIVE_HIGH = 1
};


//  ALERT Pin Latch definition
enum ina226_alert_latch_enum {
    INA226_LATCH_TRANSPARENT = 0,
    INA226_LATCH_ENABLED     = 1
};



////////////////////////////////////////////////////////////////////////////////////////
//

class INA226
{
public:
	INA226(I2CMaster &i2c, uint16_t addr = 0x40);

public:
	bool     begin();
	bool     isConnected();
	uint8_t  getAddress();


	//  Core functions
	float    getBusVoltage();       //  Volt
	float    getShuntVoltage();     //  Volt
	float    getCurrent();          //  Ampere
	float    getPower();            //  Watt
	//  See #35
	bool     isConversionReady();   //  conversion ready flag is set.
	bool     waitConversionReady(uint32_t timeout = INA226_MAX_WAIT_MS);


	//  Scale helpers milli range
	float    getBusVoltage_mV()   { return getBusVoltage()   * 1e3; };
	float    getShuntVoltage_mV() { return getShuntVoltage() * 1e3; };
	float    getCurrent_mA()      { return getCurrent()      * 1e3; };
	float    getPower_mW()        { return getPower()        * 1e3; };
	//  Scale helpers micro range
	float    getBusVoltage_uV()   { return getBusVoltage()   * 1e6; };
	float    getShuntVoltage_uV() { return getShuntVoltage() * 1e6; };
	float    getCurrent_uA()      { return getCurrent()      * 1e6; };
	float    getPower_uW()        { return getPower()        * 1e6; };


	//  Configuration
	bool     reset();
	bool     setAverage(uint8_t avg = INA226_1_SAMPLE);
	uint8_t  getAverage();
	bool     setBusVoltageConversionTime(uint8_t bvct = INA226_1100_us);
	uint8_t  getBusVoltageConversionTime();
	bool     setShuntVoltageConversionTime(uint8_t svct = INA226_1100_us);
	uint8_t  getShuntVoltageConversionTime();
	uint16_t setConfiguration(ina226_average_enum avg, ina226_timing_enum vbusct, ina226_timing_enum vshct, in226_mode_settings_enum mode);


	//  Calibration
	//  mandatory to set these values!
	//  datasheet limit == 81.92 mV;
	//    to prevent math overflow 0.02 mV is subtracted.
	//  shunt * maxCurrent <= 81.9 mV otherwise returns INA226_ERR_SHUNTVOLTAGE_HIGH
	//  maxCurrent >= 0.001           otherwise returns INA226_ERR_MAXCURRENT_LOW
	//  shunt      >= 0.001           otherwise returns INA226_ERR_SHUNT_LOW
	int      setMaxCurrentShunt(float maxCurrent = 20.0, float shunt = 0.002, bool normalize = true);
	//  configure provides full user control, not requiring call to setMaxCurrentShunt(args) function
	int      calibrate(float shunt = 0.1, float current_LSB_mA = 0.1, float current_zero_offset_mA = 0, uint16_t bus_V_scaling_e4 = 10000);
	bool     isCalibrated()     { return _current_LSB != 0.0; };

	//  These functions return zero if not calibrated!
	float    getCurrentLSB()    { return _current_LSB;       };
	float    getCurrentLSB_mA() { return _current_LSB * 1e3; };
	float    getCurrentLSB_uA() { return _current_LSB * 1e6; };
	float    getShunt()         { return _shunt;             };
	float    getMaxCurrent()    { return _maxCurrent;        };


	//  Operating mode
	bool     setMode(uint8_t mode = 7);  //  default ModeShuntBusContinuous
	uint8_t  getMode();
	bool     shutDown()                  { return setMode(0); };
	bool     setModeShuntTrigger()       { return setMode(1); };
	bool     setModeBusTrigger()         { return setMode(2); };
	bool     setModeShuntBusTrigger()    { return setMode(3); };
	bool     setModeShuntContinuous()    { return setMode(5); };
	bool     setModeBusContinuous()      { return setMode(6); };
	bool     setModeShuntBusContinuous() { return setMode(7); };  //  default.


	//  ALERT REGISTER
	//  (not tested)
	bool     setAlertRegister(uint16_t mask);
	uint16_t getAlertRegister();
	bool     setAlertLatchEnable(bool latch = false);
	bool     getAlertLatchEnable();
	bool     setAlertPolarity(bool inverted = false);
	bool     getAlertPolarity();

	//  ALERT LIMIT
	//  (not tested)
	bool     setAlertLimit(uint16_t limit);
	uint16_t getAlertLimit();


	//  Meta information
	//
	//                               typical value
	uint16_t getManufacturerID();  //  0x5449
	uint16_t getDieID();           //  0x2260


	//  DEBUG
	uint16_t getRegister(uint8_t reg) { return _readRegister(reg); };

	//
	//  ERROR HANDLING
	//
	int      getLastError();


protected:
	uint16_t _readRegister(uint8_t reg);
	uint16_t _writeRegister(uint8_t reg, uint16_t value);

protected:
	I2CMaster &m_i2c;
	uint16_t m_addr;

	float    _current_LSB;
	float    _shunt;
	float    _maxCurrent;
	float    _current_zero_offset = 0;
	uint16_t _bus_V_scaling_e4 = 10000;

	int       _error;
};



#endif /* BSP_INC_INA226_H_ */
