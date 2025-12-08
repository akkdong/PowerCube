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


bool I2CMaster::transmit(uint16_t devAddr, uint8_t *data, uint16_t size, uint32_t timeout)
{
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(m_pHandle, devAddr << 1, data, size, timeout);
	return status == HAL_OK;
}

bool I2CMaster::receive(uint16_t devAddr, uint8_t *data, uint16_t size, uint32_t timeout)
{
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(m_pHandle, devAddr << 1, data, size, timeout);
	return status == HAL_OK;
}


bool I2CMaster::deviceReady(uint16_t devAddr, uint32_t timeout)
{
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(m_pHandle, devAddr << 1, 10, timeout);
	return status == HAL_OK;
}
