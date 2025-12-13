// utils_time.h
//

#ifndef __UTILS_TIME_H__
#define __UTILS_TIME_H__

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif


inline uint32_t getTick() {
    return HAL_GetTick();
}

inline uint32_t millis() {
	return HAL_GetTick();
}

#ifdef __cplusplus
}
#endif


#endif // __UTILS_TIME_H__
