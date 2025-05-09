/**
  ******************************************************************************
  * @file           : main.c
  * @author			: Max McGill
  * @version		: 1.0
  * @date			: 29.04.25
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
#define MANMS 500
#define AUTOMS 250
#define SAFEMS 125
#define SPEEDLENGTH 50

/* Includes (Vendor Libraries, Standard Libraries, Personal Libraries) -------*/
#include "main.h"
#include "SerialIO/SerialIO.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


/* Type Definitions ----------------------------------------------------------*/
typedef enum { STATE_INITIAL, STATE_MANUAL, STATE_AUTOMATIC, STATE_SAFE } State;
typedef struct StructMachineData
{
	unsigned int lastTick;
	uint16_t lastBit;
	float weight;
	float speed;
	float brakeForce;
	float brakeTime;
	float pulse;
	unsigned char wait;
	unsigned short period;
	float maxSpeed;
	float tSpeed;
	int speedNum;

} MachineData;

typedef State func_state_t(MachineData*);

typedef enum { TMRSTATEREADY, TMRSTATECAPTURED } TimerState;


/* Extern Function Prototypes ------------------------------------------------*/


/* Static Function Prototypes (Generated, Initialisation, Logic) -------------*/
/**
 * @brief SystemClock_Config function
 * @return Description of return value
 */
static void SystemClock_Config(void);
static State machine(State, MachineData*);

/**
 * @brief init_gpio_butt function
 * @return Description of return value
 */
static int init_gpio_butt(void);
static int init_gpio_LED(void);
/**
 * @brief init_gpio_adc function
 * @return Description of return value
 */
static int init_gpio_adc(void);

/**
 * @brief init_uart_irq function
 * @return Description of return value
 */
static int init_uart_irq(void);

/**
 * @brief init_pwm function
 * @return Description of return value
 */
static void init_pwm(void);
static void init_tim3(void);
/**
 * @brief init_tim6 function
 * @return Description of return value
 */
static void init_tim6(void);
static void init_i2c(void);


/**
 * @brief program_loop function
 * @return Description of return value
 */
static int program_loop(void);

/**
 * @brief isFloat function
 * @param char[] Description of char[]
 * @return Description of return value
 */
int isFloat(char[]);
int isDigit(char);
/**
 * @brief toFloat function
 * @param char[] Description of char[]
 * @return Description of return value
 */
float toFloat(char[]);
int setBrake(float);

/* State Functions */
/**
 * @brief state_initial function
 * @param MachineData* Description of MachineData*
 * @return Description of return value
 */
static State state_initial(MachineData*);
static State state_manual(MachineData*);
/**
 * @brief state_automatic function
 * @param MachineData* Description of MachineData*
 * @return Description of return value
 */
static State state_automatic(MachineData*);
static State state_safe(MachineData*);

/* Global Variables (Extern, Static) -----------------------------------------*/
static func_state_t* const stateTable [4] = { state_initial, state_manual, state_automatic, state_safe };

static unsigned char buff[1];

static I2C_HandleTypeDef gI2C1;
static TIM_HandleTypeDef gTIM6;
static TIM_HandleTypeDef gTIM9;
static TIM_HandleTypeDef gTIM3;

static unsigned char gTxByte = 93;
static unsigned char gTskFin = 0;

static ADC_HandleTypeDef gAdcIn;

static unsigned int gCount = 0;
static unsigned int gAccu = 0;

static volatile TimerState gTIM3State = TMRSTATEREADY;
static volatile unsigned int gTIM3Edge1 = 0;
static volatile unsigned int gTIM3Edge2 = 0;
static volatile unsigned int gTIM3ElapsedTicks = 0;
static volatile unsigned short gTIM3OverFlowCnt = 0;
static volatile float gSqWvFreq = 0;

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
	/* Configure and Init */
	HAL_Init();
	SystemClock_Config();
	init_serial_io();

	init_gpio_butt();
	init_gpio_LED();
	init_gpio_adc();

	init_tim3();
	init_tim6();
	init_i2c();
	init_pwm();

	setvbuf(stdin, NULL, _IONBF, 0);

	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
	HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

	HAL_NVIC_EnableIRQ(ADC_IRQn);

	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_TIM_Base_Start_IT(&gTIM3);
	HAL_TIM_IC_Start_IT(&gTIM3, TIM_CHANNEL_1);

	printf("\r\nStarting Program\r\n");


	return program_loop();;
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

  /* Enable PORTD Clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Specialised Clock Enable */
  __HAL_RCC_ADC1_CLK_ENABLE();

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
 * @brief init_gpio_adc function
 * @return Description of return value
 */
static int init_gpio_adc()
{
	/* Local Variables */
	ADC_ChannelConfTypeDef adcChConf;
	GPIO_InitTypeDef gpioConf = {0};

	/*Configure GPIO pin : PA1 */
	gpioConf.Pin = GPIO_PIN_1;
	gpioConf.Mode = GPIO_MODE_ANALOG;
	gpioConf.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpioConf);

	gAdcIn.Instance = ADC1;
	gAdcIn.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	gAdcIn.Init.Resolution = ADC_RESOLUTION_12B;
	gAdcIn.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	gAdcIn.Init.ScanConvMode = DISABLE;
	gAdcIn.Init.ContinuousConvMode = ENABLE;
	gAdcIn.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	gAdcIn.Init.NbrOfConversion = 1;
	gAdcIn.Init.DMAContinuousRequests = DISABLE;
	HAL_ADC_Init(&gAdcIn);

	adcChConf.Channel = ADC_CHANNEL_1;
	adcChConf.Rank = 1;
	adcChConf.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	HAL_ADC_ConfigChannel(&gAdcIn, &adcChConf);

	return 0;
}

/**
 * @brief init_pwm function
 * @return Description of return value
 */
static void init_pwm(void)
{
	/* Local Variables */
	TIM_ClockConfigTypeDef clkSrcCfg = {0};
	TIM_OC_InitTypeDef ocCfg = {0};
	TIM_MasterConfigTypeDef timMstrCfg = {0};

	gTIM9.Instance = TIM9;
	gTIM9.Init.Prescaler = (3200) - 1;
	gTIM9.Init.CounterMode = TIM_COUNTERMODE_UP;
	gTIM9.Init.Period = (200) - 1;
	gTIM9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	gTIM9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&gTIM9);

	clkSrcCfg.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&gTIM9, &clkSrcCfg);

	ocCfg.OCMode = TIM_OCMODE_PWM1;
	ocCfg.Pulse = (15) - 1;
	ocCfg.OCPolarity = TIM_OCPOLARITY_HIGH;
	ocCfg.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&gTIM9, &ocCfg, TIM_CHANNEL_1);

	timMstrCfg.MasterOutputTrigger = TIM_TRGO_RESET;
	timMstrCfg.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&gTIM9, &timMstrCfg);

	HAL_TIM_PWM_Init(&gTIM9);
	HAL_TIM_MspPostInit(&gTIM9);
}

/**
 * @brief init_tim3 function
 * @return Description of return value
 */
static void init_tim3(void)
{
	/* Local Variables */
	TIM_ClockConfigTypeDef clkSrcCfg = {0};
	TIM_IC_InitTypeDef icCfg = {0};
	TIM_MasterConfigTypeDef timMstrCfg = {0};

	gTIM3.Instance = TIM3;
	gTIM3.Init.Prescaler = 0;
	gTIM3.Init.CounterMode = TIM_COUNTERMODE_UP;
	gTIM3.Init.Period = (65536 - 1);
	gTIM3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	gTIM3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&gTIM3);

	clkSrcCfg.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&gTIM3, &clkSrcCfg);

	icCfg.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	icCfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
	icCfg.ICPrescaler = TIM_ICPSC_DIV1;
	icCfg.ICFilter = 0;
	HAL_TIM_IC_ConfigChannel(&gTIM3, &icCfg, TIM_CHANNEL_1);

	timMstrCfg.MasterOutputTrigger = TIM_TRGO_RESET;
	timMstrCfg.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&gTIM3, &timMstrCfg);
}

/**
 * @brief init_tim6 function
 * @return Description of return value
 */
static void init_tim6(void)
{
	/* Local Variables */
	TIM_MasterConfigTypeDef timMstrCfg = {0};

	gTIM6.Instance = 				TIM6;
	gTIM6.Init.Prescaler = 			(64000 - 1);
	gTIM6.Init.CounterMode = 		TIM_COUNTERMODE_UP;
	gTIM6.Init.Period = 			(1000 - 1);
	gTIM6.Init.AutoReloadPreload = 	TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&gTIM6);

	timMstrCfg.MasterOutputTrigger = 	TIM_TRGO_RESET;
	timMstrCfg.MasterSlaveMode = 		TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&gTIM6, &timMstrCfg);
}

/**
 * @brief init_i2c function
 * @return Description of return value
 */
static void init_i2c(void)
{
	gI2C1.Instance = 				I2C1;
	gI2C1.Init.ClockSpeed = 		400000;
	gI2C1.Init.DutyCycle = 			I2C_DUTYCYCLE_2;
	gI2C1.Init.AddressingMode = 	I2C_ADDRESSINGMODE_7BIT;
	gI2C1.Init.DualAddressMode = 	I2C_DUALADDRESS_DISABLE;
	gI2C1.Init.OwnAddress1 = 		0;
	gI2C1.Init.OwnAddress2 = 		0;
	gI2C1.Init.GeneralCallMode = 	I2C_GENERALCALL_DISABLE;
	gI2C1.Init.NoStretchMode = 		I2C_NOSTRETCH_DISABLE;
	HAL_I2C_Init(&gI2C1);
}
/**
 * @brief machine function
 * @param curState Description of curState
 * @param ptrData Description of ptrData
 * @return Description of return value
 */
static State machine(State curState, MachineData* ptrData)
{
	return stateTable[curState](ptrData);
}

/**
 * @brief init_gpio_butt function
 * @return Description of return value
 */
static int init_gpio_butt(void)
{
	/* Local Variables */
	GPIO_InitTypeDef gpioConf = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin : PA0 */
	gpioConf.Pin = GPIO_PIN_0;
	gpioConf.Mode = GPIO_MODE_INPUT;
	gpioConf.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpioConf);
	return 0;
}

/**
 * @brief init_gpio_LED function
 * @return Description of return value
 */
static int init_gpio_LED(void)
{
	/* Local Variables */
	GPIO_InitTypeDef gpioConf = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	/*Configure GPIO pins : PD12-15 */

	gpioConf.Pin = 		GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	gpioConf.Mode = 	GPIO_MODE_OUTPUT_PP;
	gpioConf.Pull = 	GPIO_NOPULL;
	gpioConf.Speed = 	GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &gpioConf);
	return 0;
}

/**
 * @brief isFloat function
 * @param line[] Description of line[]
 * @return Description of return value
 */
int isFloat(char line[])
{
    for (int i = 0; i < strlen(line); i++)
    {
        if (!isDigit(line[i]) && line[i] != '.' && line[i] != '\n')
        {
            return 0;
        }
    }
    return 1;
}

/**
 * @brief isDigit function
 * @param c Description of c
 * @return Description of return value
 */
int isDigit(char c){
    if ('0' <= (int) c && (int) c  <= '9') { /* Is between 0 and 9 (48 and 57 using ascii) */
        return 1; /* True */
    } else {
        return 0; /* False */
    }
}

/**
 * @brief toFloat function
 * @param line[] Description of line[]
 * @return Description of return value
 */
float toFloat(char line[])
{
    int point = strlen(line);
    float num = 0;
    int divisor;

    for (int i = 0; i < strlen(line); i++)
    {
        if (line[i] == '.')
        {
            point = i;
        }

        else if (line[i] == '\n' | line[i] == '\0')
        {
            break;
        }
        else
        {
            num *= 10;
            num += line[i] - '0';
        }
    }
    divisor = strlen(line) - point;
    for (int i = 0; i < divisor - 2; i++)
    {
        num /= 10;
    }
    return num;
}



/**
 * @brief setBrake function
 * @param val Description of val
 * @return Description of return value
 */
int setBrake(float val)
{
	float ms;
	if(val >= 0 && val <=100)
	{
		ms = 0.5f + (2.0f * (val/100.0f));
		gTIM9.Instance->CCR1 = (ms/20) * (200 - 1);
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
* @brief The program loop
* @retval int - status code
*/
static int program_loop(void)
{

	/* State Machine Variables */
	MachineData data = {0, GPIO_PIN_12, 100, 50, 20, 10, 0.5, 0, 0, 0, 0, 0};
	State curState = STATE_INITIAL;

    HAL_TIM_PWM_Start(&gTIM9, TIM_CHANNEL_1);
	HAL_ADC_Start_IT(&gAdcIn);


	/* Program Loop */
	for(;;)
	{
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
		{
			return STATE_SAFE;
		}

		HAL_I2C_EnableListen_IT(&gI2C1);
		curState = machine(curState, &data);

	}
	return 0;
}


/* Initialisation Functions --------------------------------------------------*/
/**
 * @brief state_initial function
 * @param *ptrData Description of *ptrData
 * @return Description of return value
 */
static State state_initial(MachineData *ptrData)
{
	printf("I am in initial state\r\n");
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	char buff[10];
	State curState;

	gTxByte = 100;
	setBrake(100);

	/* Weight */
	for (;;)
	{
		strcpy(buff, "");
		printf("Please enter the Weight in KG: ");
		fgets(buff, "%s", stdin);
		if (isFloat(buff))
		{
			ptrData -> weight = toFloat(buff);
			break;
		}
		else
		{
			printf("\r\nError | Invalid Data\r\n");
		}
	}

	/* Speed */
	for (;;)
	{
		strcpy(buff, "");
		printf("Please enter the Speed in m/s: ");
		fgets(buff, "%s", stdin);
		if (isFloat(buff))
		{
			ptrData -> speed = atoi(buff);
			break;
		}
		else
		{
			printf("\r\nError | Invalid Data\r\n");
		}
	}

	/* BrakeForce */
	for (;;)
	{
		strcpy(buff, "");
		printf("Please enter the Break Force in N: ");
		fgets(buff, "%s", stdin);
		if (isFloat(buff))
		{
			ptrData -> brakeForce = toFloat(buff);
			break;
		}
		else
		{
			printf("\r\nError | Invalid Data\r\n");
		}
	}

	/* BrakeTime */
	for (;;)
	{
		strcpy(buff, "");
		printf("Please enter the Break Time in s: ");
		fgets(buff, "%s", stdin);
		if (isFloat(buff))
		{
			ptrData -> brakeTime = toFloat(buff);
			break;
		}
		else
		{
			printf("\r\nError | Invalid Data\r\n");
		}
	}

	/* Pulses */
	for (;;)
	{
		strcpy(buff, "");
		printf("Please enter the Pulses in m: ");
		fgets(buff, "%s", stdin);
		if (isFloat(buff))
		{
			ptrData -> pulse = toFloat(buff);
			break;
		}
		else
		{
			printf("\r\nError | Invalid Data\r\n");
		}
	}

	for (;;)
	{
		printf("Please enter Manual (m) or Automatic (a): ");
		char c = getchar();
		if (c == 'a' || c == 'A')
		{
			curState = STATE_AUTOMATIC;
			break;
		}

		else if (c == 'm' || c == 'M')
		{
			curState = STATE_MANUAL;
			break;
		}
		else
		{
			printf("\r\nError | Invalid Data\r\n");
		}
	}
	printf("\r\n");

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	{
		return STATE_SAFE;
	}

	return curState;
}

/**
 * @brief state_manual function
 * @param *ptrData Description of *ptrData
 * @return Description of return value
 */
static State state_manual(MachineData *ptrData)
{
    printf("I am in manual state\r\n");
    if(HAL_GetTick() - ptrData -> lastTick >= MANMS)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        ptrData -> lastTick = HAL_GetTick();
    }


    /* Local Variables */
    float speed;
    unsigned int adcVal;
    float volt;
    float voltsPerBit = (3.3f/4095.0f);


    if(gTskFin)
    {
        /* Start Wait */
        if(!ptrData -> wait)
        {
        	ptrData -> period = gTIM6.Instance->CNT;
        	ptrData -> wait = 1;
        }
        else
        {
            if((gTIM6.Instance->CNT - ptrData -> period) >= 10)
            {
            	ptrData -> wait = 0;
                gTskFin = 0;
                if(HAL_I2C_EnableListen_IT(&gI2C1) != HAL_OK)
                {
                    /* This would be an err and move into a safe state */

                }
            }
        }
    }

    if(gCount >= 256)
    {
        adcVal = gAccu >> 8;
        gCount = 0;
        gAccu = 0;
        gTIM9.Instance->CCR1 = (adcVal << 4);

        volt = (adcVal * voltsPerBit);

        volt *= 100.0f;
        volt = (ceil(volt));
        volt /= 100.0f;

        volt = (volt / 3.3f) * 100.0f;

        gTxByte = volt;
        setBrake(volt);

		speed = ptrData -> pulse * gSqWvFreq;
		printf("%.2f%%\t | %fm/s\r\n", volt, speed);
    }

    ptrData -> brakeForce = volt;



    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
    {
    	return STATE_SAFE;
    }

    return STATE_MANUAL;
}

/**
 * @brief state_automatic function
 * @param *ptrData Description of *ptrData
 * @return Description of return value
 */
static State state_automatic(MachineData *ptrData)
{
	printf("I am in automatic state\r\n");
	uint16_t currBit = (ptrData -> lastBit != GPIO_PIN_15) ? ptrData -> lastBit * 2 : GPIO_PIN_12;
	if(HAL_GetTick() - ptrData->lastTick >= 250)
	{
		HAL_GPIO_WritePin(GPIOD, currBit, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, ptrData->lastBit, GPIO_PIN_RESET);

		ptrData -> lastTick = HAL_GetTick();
		ptrData -> lastBit = currBit;
	}

	/* Local Variables */
	float speed;
	float averageSpeed;
	float force;
	float bForce = 0;
	float delta;
	float decel;



	speed = ptrData -> pulse * gSqWvFreq;

	ptrData -> tSpeed += speed;
	ptrData -> speedNum++;
	averageSpeed = ptrData -> tSpeed / ptrData -> speedNum;

	if (speed > ptrData -> maxSpeed)
	{
		ptrData -> maxSpeed = speed;
	}

	/* bForce% calc */
	if (speed > ptrData -> speed)
	{
		delta = speed - ptrData -> speed;
		decel = delta / ptrData -> brakeTime;
		force = ptrData -> weight * decel;
		bForce = force / ptrData -> brakeForce * 100;
		bForce = (bForce > 100) ? 100 : bForce;
	}
	setBrake(bForce);
	gTxByte = bForce;
	printf("%.2f%% bForce\t | %.2fm/s\t | %.2fm/s max\t | %.2fm/s average\r\n", bForce, speed, ptrData -> maxSpeed, averageSpeed);




	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	{
		return STATE_SAFE;
	}


	return STATE_AUTOMATIC;
}

/**
 * @brief state_safe function
 * @param *ptrData Description of *ptrData
 * @return Description of return value
 */
static State state_safe(MachineData *ptrData)
{
	printf("I am in SAFE mode\r\n");
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

	if(HAL_GetTick() - ptrData -> lastTick >= 125)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		ptrData -> lastTick = HAL_GetTick();
	}

	setBrake(100);
	gTxByte = 100;


	return STATE_SAFE;
}


/**
 * @brief HAL_TIM_IC_CaptureCallback function
 * @param htim Description of htim
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
	switch(gTIM3State)
	{
	case TMRSTATEREADY:
		gTIM3Edge1 = htim->Instance->CCR1;
		gTIM3OverFlowCnt = 0;
		gTIM3State = TMRSTATECAPTURED;
		break;
	case TMRSTATECAPTURED:
		gTIM3Edge2 = htim->Instance->CCR1;
		gTIM3ElapsedTicks = (gTIM3Edge2 + (gTIM3OverFlowCnt * (65536))) - gTIM3Edge1;
		gSqWvFreq = (64000000 / (float)gTIM3ElapsedTicks);
		gTIM3State = TMRSTATEREADY;
		break;
	}
}

/**
 * @brief TIM6_DAC_IRQHandler function
 */
void TIM6_DAC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&gTIM6);
}
/**
 * @brief HAL_TIM_PeriodElapsedCallback function
 * @param htim Description of htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM3)
	{
		++gTIM3OverFlowCnt;
	}

	else if (htim->Instance == TIM9)
	{
		if (gTIM9.Instance->CCR1 < 25)
		{
			gTIM9.Instance->CCR1 += 1;
		}
		else
		{
			gTIM9.Instance->CCR1 = 5;
		}
	}
}

/******************************************************************* Interrupts */
/**
* @brief This function handles I2C1 event interrupt.
*/
void I2C1_EV_IRQHandler(void)
{
	HAL_I2C_EV_IRQHandler(&gI2C1);
}
/**
* @brief This function handles I2C1 error interrupt.
*/
void I2C1_ER_IRQHandler(void)
{
	HAL_I2C_ER_IRQHandler(&gI2C1);
}

/******************************************************************* Callbacks */
/**
* @brief Tx Transfer completed callback.
* @param hndI2C: Pointer to a I2C_HandleTypeDef structure
* @retval void
*/
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hndI2C)
{
	gTskFin = 1;
}

/**
* @brief Slave Address Match callback.
* @param hndI2C: Pointer to a I2C_HandleTypeDef structure
* @param transferDirection: Master request Transfer Direction (Write/Read)
* @param addrMatchCode: Address Match Code
* @retval void
*/
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hndI2C, uint8_t transferDirection, uint16_t addrMatchCode)
{
	HAL_I2C_Slave_Seq_Transmit_IT(&gI2C1, &gTxByte, 1, I2C_FIRST_AND_LAST_FRAME);
}

/**
* @brief I2C error callbacks.
* @param hndI2C: Pointer to a I2C_HandleTypeDef structure
* @retval void
*/
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hndI2C)
{
	/* Realistically set an error flag and check this in an error handling function in the main */
	if(HAL_I2C_GetError(&gI2C1) == HAL_I2C_ERROR_BERR)
	{
		fprintf(stderr, "Busy error\r\n");
	}
}


/**
 * @brief ADC_IRQHandler function
 */
void ADC_IRQHandler(void)
{
	HAL_ADC_IRQHandler(&gAdcIn);
}

/**
 * @brief HAL_ADC_ConvCpltCallback function
 * @param hadc Description of hadc
 */
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc)
{
	gAccu += HAL_ADC_GetValue(&gAdcIn);
	gCount++;
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
 * @brief HAL_UART_RxCpltCallback function
 * @param *ptrUart Description of *ptrUart
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *ptrUart)
{
	if(ptrUart->Instance == USART6)
	{
		printf("Recv: %s\r\n", buff);
		HAL_UART_Receive_IT(&serialIO, buff, 1);
	}
}

/**
 * @brief USART6_IRQHandler function
 */
void USART6_IRQHandler(void)
{
	HAL_UART_IRQHandler(&serialIO);
}

/**
 * @brief init_uart_irq function
 * @return Description of return value
 */
static int init_uart_irq(void)
{
	HAL_NVIC_EnableIRQ(USART6_IRQn);
	HAL_UART_Receive_IT(&serialIO, buff, 1);
	return 0;
}

/**
 * @brief TIM3_IRQHandler function
 */
void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&gTIM3);
}

