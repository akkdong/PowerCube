/*
 * ADCReader.cpp
 *
 *
 */

#include "ADCReader.h"



///////////////////////////////////////////////////////////////////////////////////////////
//

ADCReader::ADCReader(ADC_HandleTypeDef &hadc, float voltageDivider)
	: m_hadc(hadc)
	, m_value(0)
	, m_voltageDivider(voltageDivider)
{
}

bool ADCReader::start()
{
	HAL_StatusTypeDef status;
	status = HAL_ADCEx_Calibration_Start(&m_hadc, ADC_SINGLE_ENDED);
	if (status != HAL_OK)
		return false;

	status = HAL_ADC_Start_DMA(&m_hadc, (uint32_t *)&m_value, 1);
	return (status != HAL_OK);
}

float ADCReader::getVoltage()
{
	// adcValue : 0x0FFF = adcVoltage : 3300 --> adcVoltage = adcValue * 3300 / 0x0FFF
	float adcVoltage = (float)m_value * 3300 / 0x0FFF;
	// adcVoltage = inputVoltage * voltageDivider; // 40.2 / 240.2
	return adcVoltage / m_voltageDivider;
}
