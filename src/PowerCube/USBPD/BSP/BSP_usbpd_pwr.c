/*
 * BSP_usbpd_pwr.c
 *
 *
 */

#include <stdint.h>

#include "usbpd_pwr_user.h"

#if  defined(_TRACE)
#include "usbpd_core.h"
#include "usbpd_trace.h"
#include "string.h"
#ifndef _STDIO
#include "stdio.h"
#endif /* _STDIO */
#endif /* _TRACE */

#if defined(_TRACE)
#define PWR_DEBUG_TRACE(_PORT_, __MESSAGE__)  USBPD_TRACE_Add(USBPD_TRACE_DEBUG,    (_PORT_), 0u, (uint8_t*)(__MESSAGE__), sizeof(__MESSAGE__) - 1u)
#else
#define PWR_DEBUG_TRACE(_PORT_, __MESSAGE__)
#endif /* _TRACE */



int32_t BSP_USBPD_PWR_VBUSSetVoltage_Fixed(uint32_t PortNum, uint32_t VbusTargetInmv, uint32_t OperatingCurrent, uint32_t MaxOperatingCurrent)
{
	return 0;
}

int32_t BSP_USBPD_PWR_VBUSGetVoltage(uint32_t Instance, uint32_t *pVoltage)
{
  /* USER CODE BEGIN BSP_USBPD_PWR_VBUSGetVoltage */
  /* Check if instance is valid       */
  int32_t ret;
  uint32_t val = 0U;

  if ((Instance >= USBPD_PWR_INSTANCES_NBR) || (NULL == pVoltage))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
    PWR_DEBUG_TRACE(Instance, "ADVICE: Update BSP_USBPD_PWR_VBUSGetVoltage");
  }
  *pVoltage = val;
  return ret;
  /* USER CODE END BSP_USBPD_PWR_VBUSGetVoltage */
}

int32_t BSP_USBPD_PWR_VBUSGetCurrent(uint32_t Instance, int32_t *pCurrent)
{
  /* USER CODE BEGIN BSP_USBPD_PWR_VBUSGetCurrent */
  /* Check if instance is valid       */
  int32_t ret;

  if ((Instance >= USBPD_PWR_INSTANCES_NBR) || (NULL == pCurrent))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    *pCurrent = 0;
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  return ret;
  /* USER CODE END BSP_USBPD_PWR_VBUSGetCurrent */
}

int32_t BSP_USBPD_PWR_VBUSOn(uint32_t Instance)
{
  /* USER CODE BEGIN BSP_USBPD_PWR_VBUSOn */
  /* Check if instance is valid       */
  int32_t ret;

  if (Instance >= USBPD_PWR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
    PWR_DEBUG_TRACE(Instance, "ADVICE: Update BSP_USBPD_PWR_VBUSOn");
  }
  return ret;
  /* USER CODE END BSP_USBPD_PWR_VBUSOn */
}

int32_t BSP_USBPD_PWR_VBUSOff(uint32_t Instance)
{
  /* USER CODE BEGIN BSP_USBPD_PWR_VBUSOff */
  /* Check if instance is valid       */
  int32_t ret;

  if (Instance >= USBPD_PWR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
    PWR_DEBUG_TRACE(Instance, "ADVICE: Update BSP_USBPD_PWR_VBUSOff");
  }
  return ret;
  /* USER CODE END BSP_USBPD_PWR_VBUSOff */
}

int32_t BSP_USBPD_PWR_VBUSIsOn(uint32_t Instance, uint8_t *pState)
{
  /* USER CODE BEGIN BSP_USBPD_PWR_VBUSIsOn */
  /* Check if instance is valid       */
  int32_t ret;
  uint8_t state = 0U;

  if (Instance >= USBPD_PWR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
    PWR_DEBUG_TRACE(Instance, "ADVICE: Update BSP_USBPD_PWR_VBUSIsOn");
  }
  *pState = state;
  return ret;
  /* USER CODE END BSP_USBPD_PWR_VBUSIsOn */
}


