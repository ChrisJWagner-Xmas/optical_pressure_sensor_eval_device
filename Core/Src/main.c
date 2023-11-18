/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "frameEncoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
void delay_cycles(uint32_t cycles);

enum fsmState{
	ST_IDLE,
	ST_MEASURE,
	ST_TRANSMIT_DATA,
};
static enum fsmState state = ST_IDLE; // Global FSM State
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
	const uint8_t NUM_MEASUREMENTS = 3;
	uint16_t measurementBuffer[NUM_MEASUREMENTS];
	FrameContent frameContent;
	uint8_t firstStartByte = 0xAA;
	uint8_t secondStartByte = 0x55;
	initFrameContentStruct(&frameContent, firstStartByte, secondStartByte);

	uint16_t FMA_I2C_ADDRESS = 0x28 << 1;
	uint8_t i2cBuffer[5];
	uint8_t REG_DATA = 0x00;
	uint16_t forceValue = 0;
	HAL_StatusTypeDef status;
  	uint16_t STATE_MACHINE_CLOCK_FREQ_HZ = 1;
  	uint16_t adcBuffer[2] = {0,0};

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

    // RCC Clocks in RCC_APB2ENR: all enabled via the init functions (see Register content).
    TIM2->PSC = 4000-1;
	uint16_t arrValue = 48000000/((TIM2->PSC)+1)/STATE_MACHINE_CLOCK_FREQ_HZ;
	uint16_t ccr1Value = arrValue; // ARR value = ccr value to toggle pin in sync with state machine.
	TIM2->ARR = arrValue; // Initally set the 10 Hz FSM Clock
	TIM2->CCR1 = ccr1Value; // Value is compared against the CNT registers value
	TIM2->DIER |= TIM_DIER_CC1IE; // Enable Interrupts for the capture compare channel 1 (to GPIO).
	// TIM15->CCMR1 |= (TIM_CCMR1_OC2M & (TIM_OCMODE_TOGGLE << TIM_CCMR1_OC2M_Pos)); // set [14:12] to [011], already done in MXCUBE (sConfigOC.OCMode = TIM_OCMODE_TOGGLE)
	TIM2->CCER |= TIM_CCER_CC1E; // Enable the capture compare output on channel 1.
	TIM2->DIER |= TIM_DIER_UIE; // Enable interrupt generation when CNT = ARR.
	TIM2->CR1 |= TIM_CR1_CEN; // Start the timer.

	// ************* ADC SETUP ***************************
	// Configure ADC channel pins as push/pull.
	GPIOA->CRL |= GPIO_CRL_CNF3_1;
	GPIOA->CRL &= (~GPIO_CRL_CNF3_0);
	GPIOA->CRL |= GPIO_CRL_CNF4_1;
	GPIOA->CRL &= (~GPIO_CRL_CNF4_0);

	// Enable scan mode and DMA transfer for further conversions.
	// ADC1->CR2 |= ADC_CR2_CONT;
	ADC1->CR1 |= ADC_CR1_SCAN;
	ADC1->CR2 |= ADC_CR2_DMA;

	// Set the sequence length and order.
	ADC1->SQR1 |= (ADC_SQR1_L & (0x01<<ADC_SQR1_L_Pos)); // 0x01 = 2 conversions.
	// Set the ADC sample rate by defining the number of cycles for each conversion:
	uint16_t SMP3_239_5_conversions = (ADC_SMPR2_SMP3 & 0x07<<ADC_SMPR2_SMP3_Pos); // For channel 3.
	uint16_t SMP4_239_5_conversions = (ADC_SMPR2_SMP4 & 0x07<<ADC_SMPR2_SMP4_Pos); // For channel 4.
	ADC1->SMPR2 |= (SMP3_239_5_conversions | SMP4_239_5_conversions);

	// Order for channels 3 and 4:
	ADC1->SQR3 = (ADC_SQR3_SQ1 & 0x03<<ADC_SQR3_SQ1_Pos); // Reset register and set first sequence place as channel number 3.
	ADC1->SQR3 |= (ADC_SQR3_SQ2 & 0x04<<ADC_SQR3_SQ2_Pos); // Set second sequence place as channel number 4.

	// Wake up the ADC from any power-down mode.
	ADC1->CR2 |= ADC_CR2_ADON;
	HAL_Delay(1);
	// Turn the ADC on a second time (TODO: NECESSARY?).
	ADC1->CR2 |= ADC_CR2_ADON;

	ADC1->CR2 |= ADC_CR2_CAL; // set calibration start bit
	while(ADC1->CR2 & ADC_CR2_CAL)
	{
		asm("NOP"); // Wait for the calibration to finish. ADC_CR2_CAL flips back to 0.
	}

	// ************* DMA SETUP ***************************
	// RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; // Enable DMA1 bus connection. Already done in MX_DMA_Init() by __HAL_RCC_DMA1_CLK_ENABLE()
	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR)); // Passes the ADC1 data register address to DMA Controller.
	DMA1_Channel1->CMAR = (uint32_t)adcBuffer;
	DMA1_Channel1->CNDTR = 2; // 2 increments in total (2 ADC channels are used, CH3 and CH4). Needs to be written when the DMA channel is disabled (p. 287).
	DMA1_Channel1->CCR |= DMA_CCR_CIRC; // Circular mode for adcBuffer.
	DMA1_Channel1->CCR |= DMA_CCR_MINC; // Memory increment mode on memory (when DIR = 0).
	DMA1_Channel1->CCR |= (DMA_CCR_MSIZE_0); // Memory data size = 16 bit (0x01 in respective location).
	DMA1_Channel1->CCR |= (DMA_CCR_PSIZE_0); // Peripheral data size = 16 bit.
	DMA1_Channel1->CCR |= DMA_CCR_EN; // After the setup, enable the DMA channel 1.

	int numTxBytes = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	switch(state)
	{
	case ST_IDLE:
		asm("NOP"); // Wait for the timer interrupt to trigger ST_TRIGGER_ADC
		break;
	case ST_MEASURE:
		// Start the ADC conversion.
		ADC1->CR2 |= ADC_CR2_ADON;
		// Read out the FMA force sensor.
		i2cBuffer[0] = REG_DATA; // TODO: WHY DID I ADD THIS?
		status = HAL_I2C_Master_Transmit(&hi2c1, FMA_I2C_ADDRESS, i2cBuffer, 1, 1000);
		if(status != HAL_OK)
		{
			forceValue = 1; // Error value for TX Error
		}
		else
		{
			status = HAL_I2C_Master_Receive(&hi2c1, FMA_I2C_ADDRESS, i2cBuffer, 2, 1000);
			if(status != HAL_OK)
			{
				forceValue = 0x2; // Error value for RX Error
			}
			else
			{
				forceValue = ((uint16_t)i2cBuffer[0] << 8 & 0x3F00) | ((uint16_t)i2cBuffer[1] & 0x00FF); // [0] = MSB, [1] = LSB
			}
		}
		// Read out the ADC.
		while(!(DMA1->ISR & DMA_ISR_TCIF1))
		{
			asm("NOP"); // Wait for conversion to finish.
		}
		DMA1->IFCR |= DMA_IFCR_CTCIF1; // clear transfer bit by writing to the IFCR Register.

		// Capture vcsel drive and photo current.
		measurementBuffer[0] = forceValue;
		measurementBuffer[1] = adcBuffer[0];
		measurementBuffer[2] = adcBuffer[1];
		state = ST_TRANSMIT_DATA;
		break;
	case ST_TRANSMIT_DATA:
		// Send via usb.
		numTxBytes = encodeFrame(measurementBuffer, NUM_MEASUREMENTS, &frameContent);
		status = CDC_Transmit_FS(frameContent.txBuffer, numTxBytes);
		state = ST_IDLE;
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Status_LED_Pin */
  GPIO_InitStruct.Pin = Status_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Status_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM2)
	{
		state = ST_MEASURE;
	}
}

void delay_cycles(uint32_t cycles)
{
	volatile int tick = 0;
	for(int n = 0; n < cycles; n++)
	{
		tick++;
	}
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
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
