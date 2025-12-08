/*
 * INA266.h
 *
 *  Created on: 2025. 12. 7.
 *      Author: akkdong
 */

#ifndef BSP_INC_INA226_H_
#define BSP_INC_INA226_H_

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_i2c.h"

#ifdef __cplusplus
#define extern "C" {
#endif


#ifndef INA226_ADDRESS
#define INA226_ADDRESS				0x80
#endif

#define INA226_CALIB_VAL			1024
#define INA226_CURRENTLSB			0.5F // mA/bit
#define INA226_CURRENTLSB_INV		1/INA226_CURRENTLSB // bit/mA
#define INA226_POWERLSB_INV			1/(INA226_CURRENTLSB*25) // bit/mW
#define INA226_I2CTIMEOUT			10

#define INA226_CONFIG				0x00 // Configuration Register (R/W)
#define INA226_SHUNTV				0x01 // Shunt Voltage (R)
#define INA226_BUSV					0x02 // Bus Voltage (R)
#define INA226_POWER				0x03 // Power (R)
#define INA226_CURRENT				0x04 // Current (R)
#define INA226_CALIB				0x05 // Calibration (R/W)
#define INA226_MASK					0x06 // Mask/Enable (R/W)
#define INA226_ALERTL				0x07 // Alert Limit (R/W)
#define INA226_MANUF_ID				0xFE // Manufacturer ID (R)
#define INA226_DIE_ID				0xFF // Die ID (R)

#define INA226_MODE_POWER_DOWN			(0<<0) // Power-Down
#define INA226_MODE_TRIG_SHUNT_VOLTAGE	(1<<0) // Shunt Voltage, Triggered
#define INA226_MODE_TRIG_BUS_VOLTAGE	(2<<0) // Bus Voltage, Triggered
#define INA226_MODE_TRIG_SHUNT_AND_BUS	(3<<0) // Shunt and Bus, Triggered
#define INA226_MODE_POWER_DOWN2			(4<<0) // Power-Down
#define INA226_MODE_CONT_SHUNT_VOLTAGE	(5<<0) // Shunt Voltage, Continuous
#define INA226_MODE_CONT_BUS_VOLTAGE	(6<<0) // Bus Voltage, Continuous
#define INA226_MODE_CONT_SHUNT_AND_BUS	(7<<0) // Shunt and Bus, Continuous

// Shunt Voltage Conversion Time
#define INA226_VSH_140uS			(0<<3)
#define INA226_VSH_204uS			(1<<3)
#define INA226_VSH_332uS			(2<<3)
#define INA226_VSH_588uS			(3<<3)
#define INA226_VSH_1100uS			(4<<3)
#define INA226_VSH_2116uS			(5<<3)
#define INA226_VSH_4156uS			(6<<3)
#define INA226_VSH_8244uS			(7<<3)

// Bus Voltage Conversion Time (VBUS CT Bit Settings[6-8])
#define INA226_VBUS_140uS			(0<<6)
#define INA226_VBUS_204uS			(1<<6)
#define INA226_VBUS_332uS			(2<<6)
#define INA226_VBUS_588uS			(3<<6)
#define INA226_VBUS_1100uS			(4<<6)
#define INA226_VBUS_2116uS			(5<<6)
#define INA226_VBUS_4156uS			(6<<6)
#define INA226_VBUS_8244uS			(7<<6)

// Averaging Mode (AVG Bit Settings[9-11])
#define INA226_AVG_1				(0<<9)
#define INA226_AVG_4				(1<<9)
#define INA226_AVG_16				(2<<9)
#define INA226_AVG_64				(3<<9)
#define INA226_AVG_128				(4<<9)
#define INA226_AVG_256				(5<<9)
#define INA226_AVG_512				(6<<9)
#define INA226_AVG_1024				(7<<9)

// Reset Bit (RST bit [15])
#define INA226_RESET_ACTIVE			(1<<15)
#define INA226_RESET_INACTIVE		(0<<15)

// Mask/Enable Register
#define INA226_MER_SOL				(1<<15) // Shunt Voltage Over-Voltage
#define INA226_MER_SUL				(1<<14) // Shunt Voltage Under-Voltage
#define INA226_MER_BOL				(1<<13) // Bus Voltagee Over-Voltage
#define INA226_MER_BUL				(1<<12) // Bus Voltage Under-Voltage
#define INA226_MER_POL				(1<<11) // Power Over-Limit
#define INA226_MER_CNVR				(1<<10) // Conversion Ready
#define INA226_MER_AFF				(1<<4)  // Alert Function Flag
#define INA226_MER_CVRF				(1<<3)  // Conversion Ready Flag
#define INA226_MER_OVF				(1<<2)  // Math Overflow Flag
#define INA226_MER_APOL				(1<<1)  // Alert Polarity Bit
#define INA226_MER_LEN				(1<<0)  // Alert Latch Enable

#define INA226_MANUF_ID_DEFAULT		0x5449
#define INA226_DIE_ID_DEFAULT		0x2260



float INA226_getBusV(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
float INA226_getCurrent(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
float INA226_getPower(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);

uint8_t INA226_setConfig(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord);
uint16_t INA226_getConfig(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint16_t INA226_getShuntV(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint16_t INA226_getBusVReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint16_t INA226_getPowerReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint8_t INA226_setCalibrationReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord);
uint16_t INA226_getCalibrationReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint16_t INA226_getCurrentReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint16_t INA226_getManufID(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint16_t INA226_getDieID(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint8_t INA226_setMaskEnable(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord);
uint16_t INA226_getMaskEnable(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint8_t INA226_setAlertLimit(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord);
uint16_t INA226_getAlertLimit(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);



//
//
//

typedef enum {
	OK=0,
	FAIL=-1,
    INA226_TI_ID_MISMATCH = -2,
    INA226_DIE_ID_MISMATCH =-3,
    CONFIG_ERROR = -4,
    I2C_TRANSMISSION_ERROR = -5,
    BAD_PARAMETER = -6,
    NOT_INITIALIZED = -7,
    INVALID_I2C_ADDRESS
} INA226_Status;

typedef struct INA226_HandleTypeDef
{
	I2C_HandleTypeDef *pHandle;

	uint8_t     mInitialized;
	uint8_t  mI2C_Address;
	uint16_t mConfigRegister;        	// local copy from the INA226
	uint16_t mCalibrationValue;        	// local copy from the INA226
	int32_t  mCurrentMicroAmpsPerBit; 	// This is the Current_LSB, as defined in the INA266 spec
	int32_t  mPowerMicroWattPerBit;

} INA226_HandleTypeDef;


enum eOperatingMode {
//	Shutdown					 = 0,
	ShuntVoltageTriggered        = 1,
	BusVoltageTriggered          = 2,
	ShuntAndBusTriggered         = 3,
	Shutdown                     = 4,
	ShuntVoltageContinuous       = 5,
	BusVoltageContinuous         = 6,
	ShuntAndBusVoltageContinuous = 7	 //default
};

enum eAlertTrigger {
	ClearTriggers                = 0x0000, //default
	ShuntVoltageOverLimit        = 0x8000,
	ShuntVoltageUnderLimit       = 0x4000,
	BusVoltageOverLimit          = 0x2000,
	BusVoltageUnderLimit         = 0x1000,
	PowerOverLimit               = 0x0800,
	ConversionReady              = 0x0400
};

enum eAlertTriggerCause {
	Unknown=0,
	AlertFunctionFlag            = 0x10,
	ConversionReadyFlag          = 0x08,
	MathOverflowFlag             = 0x04,
	AlertPolarityBit             = 0x02
};


void INA266_init(I2C_HandleTypeDef *pHandle);

void INA226_Constructor(INA226_HandleTypeDef *);
INA226_Status INA226_CheckI2cAddress(uint8_t aI2C_Address);

//Resets the INA226 and configures it according to the supplied parameters - should be called first.
//status AutoFox_INA226_Init(uint8_t aI2C_Address=0x40, double aShuntResistor_Ohms=0.1, double aMaxCurrent_Amps=3.2767);
INA226_Status INA226_Init(INA226_HandleTypeDef *,uint8_t aI2C_Address, double aShuntResistor_Ohms, double aMaxCurrent_Amps);

int32_t INA226_GetShuntVoltage_uV(INA226_HandleTypeDef *);
int32_t INA226_GetBusVoltage_uV(INA226_HandleTypeDef *);
int32_t INA226_GetCurrent_uA(INA226_HandleTypeDef *);
int32_t INA226_GetPower_uW(INA226_HandleTypeDef *);

INA226_Status INA226_SetOperatingMode(INA226_HandleTypeDef *,enum eOperatingMode aOpMode);
INA226_Status INA226_Hibernate(INA226_HandleTypeDef *); //Enters a very low power mode, no voltage measurements
INA226_Status INA226_Wakeup(INA226_HandleTypeDef *);    //Wake-up and enter the last operating mode

//The trigger value is in microwatts or microvolts, depending on the trigger
INA226_Status INA226_ConfigureAlertPinTrigger(INA226_HandleTypeDef *,enum eAlertTrigger aAlertTrigger, int32_t aValue, uint8_t aLatching);
//status AutoFox_INA226_ResetAlertPin(INA226_HandleTypeDef *);
INA226_Status INA226_ResetAlertPin(INA226_HandleTypeDef *,enum  eAlertTriggerCause* aAlertTriggerCause_p ); //provides feedback as to what caused the alert

//The parameters for the two functions below are indices into the tables defined in the INA226 spec
//These tables are copied below for your information (caNumSamplesAveraged & caVoltageConvTimeMicroSecs)
INA226_Status INA226_ConfigureVoltageConversionTime(INA226_HandleTypeDef *,int aIndexToConversionTimeTable);
INA226_Status INA226_ConfigureNumSampleAveraging(INA226_HandleTypeDef *,int aIndexToSampleAverageTable);
INA226_Status INA226_Debug_GetConfigRegister(INA226_HandleTypeDef *,uint16_t* aConfigReg_p);

//Private functions

INA226_Status INA226_WriteRegister(INA226_HandleTypeDef *,uint8_t aRegister, uint16_t aValue);
INA226_Status NA226_ReadRegister(INA226_HandleTypeDef *,uint8_t aRegister, uint16_t* aValue_p);
INA226_Status INA226_setupCalibration(INA226_HandleTypeDef *,double aShuntResistor_Ohms, double aMaxCurrent_Amps);



#endif /* BSP_INC_INA226_H_ */


#ifdef __cplusplus
}
#endif
