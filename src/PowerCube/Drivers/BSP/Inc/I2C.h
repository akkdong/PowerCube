/*
 * I2C.h
 *
 *  Created on: 2025. 12. 7.
 *      Author: akkdong
 */

#ifndef BSP_INC_I2C_H_
#define BSP_INC_I2C_H_

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_i2c.h"


//////////////////////////////////////////////////////////////////////////////
//

class I2CMaster
{
public:
	I2CMaster(I2C_HandleTypeDef *pHandle);

public:
	bool deviceReady(uint16_t devAddr, uint32_t timeout = 1000);

	bool write(uint16_t devAddr, uint8_t *data, uint16_t size, uint32_t timeout = 1000); // devAddr = device_addr << 1
	bool read(uint16_t devAddr, uint8_t *data, uint16_t size, uint32_t timeout = 1000);

	bool writeRegister(uint16_t devAddr, uint8_t regAddr, uint8_t value);
	bool writeRegister(uint16_t devAddr, uint8_t regAddr, uint16_t value);
	bool writeRegister(uint16_t devAddr, uint8_t regAddr, uint8_t *value, uint8_t len);
	bool readRegister(uint16_t devAddr, uint8_t regAddr, uint8_t *value);
	bool readRegister(uint16_t devAddr, uint8_t regAddr, uint16_t *value);
	bool readRegister(uint16_t devAddr, uint8_t regAddr, uint8_t *value, uint8_t len);


protected:
	I2C_HandleTypeDef *m_pHandle;
};


#endif /* BSP_INC_I2C_H_ */
