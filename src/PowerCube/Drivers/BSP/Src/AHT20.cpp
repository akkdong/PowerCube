/****************************************************************
*
 ***************************************************************/
#include "AHT20.h"

#define millis()	HAL_GetTick()
#define delay(x)	HAL_Delay(x)


AHT20::AHT20(I2CMaster &i2c, uint16_t addr) : m_i2c(i2c) , m_addr(addr)
{
}

bool AHT20::begin()
{
	if (!m_i2c.deviceReady(m_addr))
	{
		//If IC failed to respond, give it 20ms more for Power On Startup
		//Datasheet pg 7
		delay(20);

		if (!m_i2c.deviceReady(m_addr))
			return false;
	}

	//Wait 40 ms after power-on before reading temp or humidity. Datasheet pg 8
	while(millis() < 40);

	//Check if the calibrated bit is set. If not, init the sensor.
	if (!getStatus(BIT_CALIBRATED))
	{
		initialize();
		delay(10);
		triggerMeasurement();
		delay(80); //Wait for measurement to complete
		uint8_t counter = 0;
		while (getStatus(BIT_BUSY))
		{
			delay(1);
			if (counter++ > 100)
				return false;
		}
		if (!getStatus(BIT_CALIBRATED))
			return false;
	}

	_sensorQueried.temperature = true;
	_sensorQueried.humidity = true;

	return true;
}

/*------------------------ Measurement Helpers ---------------------------*/

bool AHT20::getStatus(uint8_t bit)
{
	uint8_t data;
	if (m_i2c.read(m_addr, &data, 1))
		return (data & (1 << bit));
	return false;
	/*
    Wire.requestFrom(_deviceAddress, (uint8_t)1);
    if (Wire.available()) return ((uint8_t)Wire.read() & (1 << bit));
    return false;
    */
}

bool AHT20::initialize()
{
	/*
    Wire.beginTransmission(_deviceAddress);
    Wire.write(CMD_INITIALIZE);
    Wire.write((uint8_t)0x08);
    Wire.write((uint8_t)0x00);
    return (Wire.endTransmission() == 0);
    */

	uint8_t data[3] = {	CMD_INITIALIZE, 0x08, 0x00 };
	return m_i2c.write(m_addr, data, 3);
}

bool AHT20::triggerMeasurement()
{
    /*
	Wire.beginTransmission(_deviceAddress);
    Wire.write(CMD_TRIGGER);
    Wire.write((uint8_t)0x33);
    Wire.write((uint8_t)0x00);
    return (Wire.endTransmission() == 0);
    */

	uint8_t data[3] = {	CMD_TRIGGER, 0x33, 0x00 };
	return m_i2c.write(m_addr, data, 3);
}

//Loads the
void AHT20::readData()
{
    _sensorData.temperature = 0;
    _sensorData.humidity = 0;
    /*
    if (Wire.requestFrom(_deviceAddress, (uint8_t)7) > 0)
    {
        Wire.read(); // status byte

        uint32_t incoming = 0;
        incoming = ((uint32_t)Wire.read() << 16);
        incoming |= ((uint32_t)Wire.read() << 8);
        uint8_t midByte = Wire.read();

        incoming |= midByte;
        _sensorData.humidity = incoming >> 4; // 20bits

        _sensorData.temperature = ((uint32_t)midByte << 16);
        _sensorData.temperature |= ((uint32_t)Wire.read() << 8);
        _sensorData.temperature |= (uint32_t)Wire.read();

        _sensorData.crc = Wire.read();
        //Need to get rid of data in bits > 20
        _sensorData.temperature = _sensorData.temperature & ~(0xFFF00000);

        //Mark data as fresh
        _sensorQueried.temperature = false;
        _sensorQueried.humidity = false;
    }
    */
    uint8_t data[7];
    if (m_i2c.read(m_addr, data, 7))
    {
    	// status byte
    	// data[0]

		uint32_t incoming = 0;
		incoming = ((uint32_t)data[1] << 16);
		incoming |= ((uint32_t)data[2] << 8);
		uint8_t midByte = data[3];

		incoming |= midByte;
		_sensorData.humidity = incoming >> 4; // 20bits

		_sensorData.temperature = ((uint32_t)midByte << 16);
		_sensorData.temperature |= ((uint32_t)data[4] << 8);
		_sensorData.temperature |= (uint32_t)data[5];

		_sensorData.crc = data[6];
		//Need to get rid of data in bits > 20
		_sensorData.temperature = _sensorData.temperature & ~(0xFFF00000);

		//Mark data as fresh
		_sensorQueried.temperature = false;
		_sensorQueried.humidity = false;
    }
}

bool AHT20::softReset()
{
	/*
	Wire.beginTransmission(_deviceAddress);
    Wire.write(CMD_SOFTRESET);
    return (Wire.endTransmission() == 0);
    */
	uint8_t data = CMD_SOFTRESET;
	return m_i2c.write(m_addr, &data, 1);
}

float AHT20::getTemperature()
{
	if (_sensorQueried.temperature)
	{
		triggerMeasurement();
		delay(80); //Wait for measurement to complete
		uint8_t counter = 0;
		while (getStatus(BIT_BUSY))
		{
			delay(1);
			if (counter++ > 100)
				return false;
		}

		readData();
	}

	//Mark data as old
	_sensorQueried.temperature = true;

	//From datasheet pg 8
	return ((float)_sensorData.temperature / 1048576) * 200 - 50;
}

float AHT20::getHumidity()
{
	if (_sensorQueried.humidity)
	{
		triggerMeasurement();
		delay(80);
		uint8_t counter = 0;
		while (getStatus(BIT_BUSY))
		{
			delay(1);
			if (counter++ > 100)
				return false;
		}
		readData();
	}

	//Mark data as old
	_sensorQueried.humidity = true;

	//From datasheet pg 8
	return ((float)_sensorData.humidity / 1048576) * 100;
}
