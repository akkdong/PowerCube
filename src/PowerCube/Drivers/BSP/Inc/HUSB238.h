/*
 * HUSB238.h
 *
 *  Created on: 2025. 12. 8.
 *      Author: akkdong
 */

#ifndef BSP_INC_HUSB238_H_
#define BSP_INC_HUSB238_H_

#include "I2C.h"



//////////////////////////////////////////////////////////////////////////////////////////////
//

class HUSB238
{
public:
	HUSB238(I2CMaster &i2c, uint16_t addr = 0x08);

	enum Response {
		PD_NO_RESPONSE = 0,
		PD_SUCCESS = 1,
		PD_INVALID = 3,
		PD_CMD_NOT_SUPPORTED = 4,
		PD_FAIL = 5
	};

	enum Registers
	{
		REG_PD_STATUS0 	= 0x00,
		REG_PD_STATUS1 	= 0x01,
		REG_SRC_PDO_5V 	= 0x02,
		REG_SRC_PDO_9V 	= 0x03,
		REG_SRC_PDO_12V	= 0x04,
		REG_SRC_PDO_15V	= 0x05,
		REG_SRC_PDO_18V	= 0x06,
		REG_SRC_PDO_20V	= 0x07,
		REG_SRC_PDO		= 0x08,
		REG_GO_COMMAND 	= 0x09
	};

	enum SourcePDO
	{
		PDO_5V	= 0b0001,
		PDO_9V	= 0b0010,
		PDO_12V	= 0b0011,
		PDO_15V	= 0b1000,
		PDO_18V	= 0b1001,
		PDO_20V	= 0b1010
	};

	enum CapabilityPDO {
		CAPABILITY_PDO_5V,
		CAPABILITY_PDO_9V,
		CAPABILITY_PDO_12V,
		CAPABILITY_PDO_15V,
		CAPABILITY_PDO_18V,
		CAPABILITY_PDO_20V,
		CAPABILITY_PDO_COUNT
	};

	struct Capability
	{
		uint16_t ma_5V;
		uint16_t ma_9V;
		uint16_t ma_12V;
		uint16_t ma_15V;
		uint16_t ma_18V;
		uint16_t ma_20V;
	};

	union Status
	{
		struct
		{
			unsigned PD_SRC_CURRENT	: 4;
			unsigned PD_SRC_VOLTAGE	: 4;

			unsigned CURRENT_5V		: 2;
			unsigned VOLTAGE_5V		: 1;
			unsigned PD_RESPONSE	: 3;
			unsigned ATTACH			: 1;
			unsigned CC_DIR			: 1;
		};

		struct
		{
			uint8_t PD_STATUS0;
			uint8_t PD_STATUS1;
		};

		uint16_t combined;
	};


public:
	void begin();

	Capability *getCapabilities();
  	uint8_t getActiveVoltage(bool update = true);
  	uint16_t getActiveCurrent(bool update = true);
  	uint16_t updateStatus();

  	void setVoltage(SourcePDO pdo);

protected:
  	void writeRegister(uint8_t regAddr, uint8_t value);
  	uint8_t readRegister(uint8_t regAddr);

protected:
	I2CMaster &m_i2c;
	uint16_t m_addr;

	Capability m_capability;
	Status m_status;
};



#endif /* BSP_INC_HUSB238_H_ */
