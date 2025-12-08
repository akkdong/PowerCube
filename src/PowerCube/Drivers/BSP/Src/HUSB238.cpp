/*
 * HUSB238.cpp
 *
 *  Created on: 2025. 12. 8.
 *      Author: akkdong
 */


#include "HUSB238.h"


//////////////////////////////////////////////////////////////////////////////////////////////
//

static const uint16_t __srcCurrent[16] =
{
	500,
	700,
	1000,
	1250,
	1500,
	1750,
	2000,
	2250,
	2500,
	2750,
	3000,
	3250,
	3500,
	4000,
	4500,
	5000
};

static const uint8_t __srcVoltage[7] =
{
	0,
	5,
	9,
	12,
	15,
	18,
	20
};

static const uint16_t __appleCurrent[4] =
{
	500,
	1500,
	2400,
	3000
};


//////////////////////////////////////////////////////////////////////////////////////////////
//

HUSB238::HUSB238(I2CMaster &i2c, uint16_t addr) : m_i2c(i2c), m_addr(addr << 1)
{
}


void HUSB238::begin()
{
	/*
	this->getCapabilites();
	*/
	this->updateStatus();
}


HUSB238::Capability *HUSB238::getCapabilities()
{
	uint16_t *pCapability = &m_capability.ma_5V;

	for (uint8_t i = 0; i < CAPABILITY_PDO_COUNT; ++i, ++pCapability)
	{
		uint8_t cap = readRegister(REG_SRC_PDO_5V + i);
		*pCapability = (cap & 0x80) ? __srcCurrent[cap & 0x0F] : 0;
	}

	return &m_capability;
}

void HUSB238::setVoltage(SourcePDO pdo)
{
#if METHOD_0
	m_i2c.writeRegister(m_addr, REG_SRC_PDO, (uint8_t)pdo << 4);
	m_i2c.writeRegister(m_addr, REG_GO_COMMAND, (uint8_t)1);
#else
	uint8_t data[2];
	data[0] = REG_SRC_PDO;
	data[1] = (uint8_t)pdo << 4;
	m_i2c.write(m_addr, data, 2);

	data[0] = REG_GO_COMMAND;
	data[1] = 1;
	m_i2c.write(m_addr, data, 2);
#endif
}

uint16_t HUSB238::updateStatus()
{
	m_status.PD_STATUS0 = readRegister(REG_PD_STATUS0);
	m_status.PD_STATUS1 = readRegister(REG_PD_STATUS1);

	return m_status.combined;
}

uint8_t HUSB238::getActiveVoltage(bool update)
{
	if (update)
		updateStatus();

	return __srcVoltage[m_status.PD_SRC_VOLTAGE];
}

uint16_t HUSB238::getActiveCurrent(bool update)
{
	if (update)
		updateStatus();

	return __srcCurrent[m_status.PD_SRC_CURRENT];
}

void HUSB238::writeRegister(uint8_t regAddr, uint8_t value)
{
	m_i2c.writeRegister(m_addr, regAddr, value);
}

uint8_t HUSB238::readRegister(uint8_t regAddr)
{
	uint8_t value;
	m_i2c.readRegister(m_addr, regAddr, &value);
	return value;

}
