/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fat_app.h"
#include "usb_device.h"
#include "usbd_storage_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*pFunction)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void JumpToApp(void);
void TurnOff(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	FLASH_EraseInitTypeDef flashErase;
	uint32_t ErrorFlash;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  //
  // boot-up procedure
  //   1. LED blinks during the waiting time(about 2s)
  //   2. Check watermark of the user program
  //      - user program must have a valid version string in it's first sector
  //      - user program runs from second sector
  //   2. Check power button (GPIOB_0)
  //      - if power button is pressed, user program is executed --> 3
  //      - the other case, go to update mode --> 4
  //
  //   3. Jump to user application
  //      - jump into FLASH_APP_ADDR + SECTOR_SIZE
  //
  //   4. Update mode
  //      - Initialize virtual FAT
  //      - Prepare MSC
  //      - wait....

	// blink led during waiting time
  	for (int i = 0; i < 10; ++i)
  	{
  		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		HAL_Delay(200);
  	}

  	// check version string
  	memset(versionStr, ' ', MAX_VERSIONSTR);
  	uint32_t watermark = *((uint32_t*)FLASH_APP_ADDR);
  	const char *ptr = (char *)(FLASH_APP_ADDR + sizeof(uint32_t));
  	if(watermark == 0xDEADBEEF && strncmp(ptr, "PowerCUBE", 9) == 0)
		strncpy((char *)versionStr, ptr, MAX_VERSIONSTR);
  	else
  		strcpy((char *)versionStr, "N/A");

  	// check whether power button is pressed
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET && versionStr[0] == 'P')
	{
		//
		// Jump to application
		//
		JumpToApp();
	}

	//
	// ---------- Enter in update mode ----------
	//

	// Initialize the FAT File System
	FAT_Init();

	// Initialize the USB Device MSC
	MX_USB_Device_Init();

	// Turn the User LED ON to indicate that application entered in update mode
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t lastTick = HAL_GetTick();
  uint32_t blinkTick = lastTick, blinkCount = 0;
  uint32_t lastPress = 0;
  while (1)
  {
	    // do noting: explicit exit from update mode
  		if (HAL_GetTick() - lastTick > 5 * 60 * 1000)
  			break; // timeout 5min --> power-off

  		// long key press: implicit exit from update mode
  		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
  		{
  			if (lastPress == 0)
  				lastPress = HAL_GetTick();

  			if (HAL_GetTick() - lastPress > 2500)
  				break; // hold button for 2500ms --> power-off
  		}
  		else
  		{
  			// reset press-tick
  			lastPress = 0;
  		}

  		// blink LED: short ON, long OFF
  		if (HAL_GetTick() - blinkTick > 200)
  		{
  			// 0, 1, 2, 3, 4, 5
  			// ^              ^
  			if (blinkCount == 0 || blinkCount == 5)
  			{
  				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
  				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
  			}

  			blinkCount = (blinkCount + 1) % 6;
  			blinkTick = HAL_GetTick();
  		}

		if(fatINFO.FileTransferCpt == SET)
		{
			/* Wait for the end of transmission */
			HAL_Delay(250);

			/* De-Initialize the USB Device */
			USBD_DeInit(&hUsbDeviceFS);

			/* Erase the Remaining Flash Memory */
			/* Initialize the Erase TypeDef */
			flashErase.Banks = FLASH_BANK_1;
			flashErase.TypeErase = FLASH_TYPEERASE_PAGES;

			/* Calculate the last Memory Position */
			flashErase.Page = FLASH_PAGE_OFFSET + (fatINFO.FileSize + FLASH_PAGE_SIZE-1)/FLASH_PAGE_SIZE;

			/* Unlock the Flash Memory */
			HAL_FLASH_Unlock();

			/* Check which bank corresponds to the calculated page */
			if(flashErase.Page < FLASH_PAGE_NB)
			{
				/* Compute the Number of pages */
				flashErase.NbPages = FLASH_PAGE_NB - flashErase.Page;

				if(HAL_FLASHEx_Erase(&flashErase, &ErrorFlash) != HAL_OK)
				{
					/* Lock the Flash Memory */
					HAL_FLASH_Lock();

					/* Call the Error Handler */
					Error_Handler();
				}

				/* Update the Erase settings */
				flashErase.Banks = FLASH_BANK_2;
				//flashErase.Page = 256;
				flashErase.Page = 0;
				flashErase.NbPages = FLASH_PAGE_NB;

				if(HAL_FLASHEx_Erase(&flashErase, &ErrorFlash) != HAL_OK)
				{
					/* Lock the Flash Memory */
					HAL_FLASH_Lock();

					/* Call the Error Handler */
					Error_Handler();
				}
			}
			else
			{
				/* Update the Erase settings */
				flashErase.Banks = FLASH_BANK_2;
				flashErase.Page &= (FLASH_PAGE_NB - 1);
				flashErase.NbPages = FLASH_PAGE_NB - flashErase.Page;

				if(HAL_FLASHEx_Erase(&flashErase, &ErrorFlash) != HAL_OK)
				{
					/* Lock the Flash Memory */
					HAL_FLASH_Lock();

					/* Call the Error Handler */
					Error_Handler();
				}
			}

			/* Lock the Flash Memory */
			HAL_FLASH_Lock();

			/* Jump To Application */
			JumpToApp();
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  // turn-off power
  TurnOff();

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Perform a jump to the application if it's available
 * @param None
 * @retval None
 */
void JumpToApp(void)
{
	/* Local variables */
	uint32_t JumpAddress;
	pFunction Jump_To_Application;

	/* Check whether App is available */
	if((*(uint32_t*) FLASH_APP_ADDR) != 0xFFFFFFFF && (*(uint32_t*) FLASH_RUN_ADDR) != 0xFFFFFFFF)
	{
		/* De-init the GPIOS */
		//HAL_GPIO_DeInit(GPIOC, GPIO_PIN_13);
		//HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);

		/* Set the Clock to default state */
		HAL_RCC_DeInit();

		/* Jump to user application */
		JumpAddress = *(__IO uint32_t*) (FLASH_RUN_ADDR + 4);
		Jump_To_Application = (pFunction)JumpAddress;

		/* Initialize use application's Stack Pointer */
		__set_MSP(*(__IO uint32_t*)FLASH_RUN_ADDR);
		Jump_To_Application();
	}
	else
	{
		/* Blink the LED at 0.5Hz to indicate that there isn't an application into the memory */
		for (uint32_t i = 0; i < 10; ++i)
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			HAL_Delay(1000);
		}

		TurnOff();
	}
}

/**
 *
 *
 */
void TurnOff(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

	while (1)
		HAL_Delay(100);
}


/**
 * @brief Perform a Flash Page Erase
 * @param page: the page number to be erased
 * @retval status: HAL_StatusTypeDef with the operation result
 */
HAL_StatusTypeDef EraseFlashPage(uint16_t page)
{
	/* Local Variables */
	HAL_StatusTypeDef status;
	FLASH_EraseInitTypeDef flashErase_t;
	uint32_t EraseError;

	/* Erase TypeDef initialization */
	flashErase_t.Page = page;
	flashErase_t.NbPages = 1;
	flashErase_t.TypeErase = FLASH_TYPEERASE_PAGES;

	if(page <= 127)
	{
		flashErase_t.Banks = FLASH_BANK_1;
	}
	else
	{
		flashErase_t.Banks = FLASH_BANK_2;
	}

	/* Unlock the Flash Memory */
	HAL_FLASH_Unlock();

	/* Perform the Flash Erase Page */
	status = HAL_FLASHEx_Erase(&flashErase_t, &EraseError);

	/* Lock the Flash Memory */
	HAL_FLASH_Lock();

	/* Return the operation result */
	return status;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
		/* Toggle the User LED status */
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);

		/* Wait for 500ms */
		HAL_Delay(200);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
