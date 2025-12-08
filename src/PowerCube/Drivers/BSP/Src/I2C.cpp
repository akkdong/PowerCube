/*
 * I2C.cpp
 *
 *  Created on: 2025. 12. 7.
 *      Author: akkdong
 */


#include "I2C.h"



//////////////////////////////////////////////////////////////////////////////
//

I2CMaster::I2CMaster(I2C_HandleTypeDef *pHandle) : m_pHandle(pHandle)
{

}


bool I2CMaster::deviceReady(uint16_t devAddr, uint32_t timeout)
{
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(m_pHandle, devAddr, 10, timeout);
	return status == HAL_OK;
}


bool I2CMaster::write(uint16_t devAddr, uint8_t *data, uint16_t size, uint32_t timeout)
{
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(m_pHandle, devAddr, data, size, timeout);
	return status == HAL_OK;
}

bool I2CMaster::read(uint16_t devAddr, uint8_t *data, uint16_t size, uint32_t timeout)
{
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(m_pHandle, devAddr, data, size, timeout);
	return status == HAL_OK;
}


bool I2CMaster::writeRegister(uint16_t devAddr, uint8_t regAddr, uint8_t value)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(m_pHandle, devAddr, regAddr, 1, &value, 1, 1000);
	return status == HAL_OK;
}

bool I2CMaster::writeRegister(uint16_t devAddr, uint8_t regAddr, uint16_t value)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(m_pHandle, devAddr, regAddr, 1, (uint8_t *)&value, 2, 1000);
	return status == HAL_OK;
}

bool I2CMaster::writeRegister(uint16_t devAddr, uint8_t regAddr, uint8_t *value, uint8_t len)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(m_pHandle, devAddr, regAddr, 1, value, len, 1000);
	return status == HAL_OK;
}

bool I2CMaster::readRegister(uint16_t devAddr, uint8_t regAddr, uint8_t *value)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(m_pHandle, devAddr, regAddr, 1, value, 1, 1000);
	return status == HAL_OK;
}

bool I2CMaster::readRegister(uint16_t devAddr, uint8_t regAddr, uint16_t *value)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(m_pHandle, devAddr, regAddr, 1, (uint8_t *)value, 2, 1000);
	return status == HAL_OK;
}

bool I2CMaster::readRegister(uint16_t devAddr, uint8_t regAddr, uint8_t *value, uint8_t len)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(m_pHandle, devAddr, regAddr, 1, value, len, 1000);
	return status == HAL_OK;
}

