/**
  ******************************************************************************
  * @file           : main.c
  * @author			: <your_name>
  * @version		: 1.0
  * @date			: <project_date>
  * @brief          : Main program body
  *
  * All additional configuration and implementation of features must be
  * implemented by the developer. Comments will need to be updated to include
  * the correct names, descriptions, and dates.
  *
  * Parameters may need to be changed for function prototypes and definitions
  * to correctly pass data for any process.
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 University of Staffordshire
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


/* Definitions ---------------------------------------------------------------*/


/* Includes (Vendor Libraries, Standard Libraries, Personal Libraries) -------*/
#include "main.h"


/* Type Definitions ----------------------------------------------------------*/


/* Extern Function Prototypes ------------------------------------------------*/


/* Static Function Prototypes (Generated, Initialisation, Logic) -------------*/
static void SystemClock_Config(void);

static int program_loop(void);


/* Global Variables (Extern, Static) -----------------------------------------*/


/**
  * @brief  The application entry point.
  * @retval int - status code
  *
  * The developer is required to provide additional
  * configuration for the necessary features that will be enabled
  * for this application. Only the generated configuration is provided.
  */
int main(void)
{
	/* Local Variables */

	/* Configure and Init */
	HAL_Init();
	SystemClock_Config();

	return program_loop();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = 		RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = 				RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = 			RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = 		RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 				4;
  RCC_OscInitStruct.PLL.PLLN = 				128;
  RCC_OscInitStruct.PLL.PLLP = 				RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 				6;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = 			RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = 			RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = 		RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = 		RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = 		RCC_HCLK_DIV8;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}


/**
  * @brief  The program loop
  * @retval int - status code
  */
static int program_loop(void)
{
	/* Local Variables */

	/* Program Loop */
	for(;;)
	{

	}

	return 0;
}
