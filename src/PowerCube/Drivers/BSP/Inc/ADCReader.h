/*
 * ADCReader.h
 *
 *
 */

#ifndef BSP_INC_ADCREADER_H_
#define BSP_INC_ADCREADER_H_

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc.h"


///////////////////////////////////////////////////////////////////////////////////////////
//

class ADCReader
{
public:
	ADCReader(ADC_HandleTypeDef &hadc, float voltageDivider);

public:
	bool start();

	float getVoltage();

protected:
	ADC_HandleTypeDef &m_hadc;
	uint32_t m_value;

	float m_voltageDivider;
};


#endif /* BSP_INC_ADCREADER_H_ */
