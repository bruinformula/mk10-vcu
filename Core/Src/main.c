/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

#include "can.h"
#include "mcp2515.h"

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

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*  includes */
#include <stdlib.h>
#define false 0
#define true 1

/*                    VCU Constants                      */



/*                    VCU Constants End                     */
const uint16_t APPS1_RANK = 0;
const uint16_t APPS2_RANK = 1;
const uint16_t BSE_RANK = 2;

const uint16_t APPS_1_ADC_MIN_VAL = 10;
const uint16_t APPS_1_ADC_MAX_VAL = 4095;

const uint16_t APPS_2_ADC_MIN_VAL = 10;
const uint16_t APPS_2_ADC_MAX_VAL = 4095;

//Nm
const float MIN_TORQUE = 0;
const float MAX_TORQUE = 108;

const float REGEN_BASELINE_TORQUE = 0;
const float REGEN_MAX_TORQUE = -30;

const uint16_t BSE_ADC_MIN_VAL = 0;
const uint16_t BSE_ADC_MAX_VAL = 4095;


const uint16_t APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE = 10;
const uint16_t APPS_IMPLAUSIBILITY_TIMEOUT_MILLIS = 100;

const uint16_t BRAKE_ACTIVATED_ADC_VAL = 100;
const uint16_t CROSS_CHECK_IMPLAUSIBILITY_APPS_PERCENT = 25;
const uint16_t CROSS_CHECK_IMPLAUSIBILITY_TIMEOUT_MILLIS = 100;

const uint16_t DMA_READ_TIMEOUT = 10;





/*                    VCU vars                      */

uint16_t apps1Value;
uint16_t apps2Value;
uint16_t bseValue;


float requestedTorque;
float lastRequestedTorque = 0;
float finalTorqueRequest;

//apps plausibility
uint16_t apps_plausible = true;
uint32_t millis_since_apps_implausible;

uint32_t millis_since_dma_read;

//cross check plausibility
uint16_t cross_check_plausible = true;
uint32_t millis_since_cross_check_implausible;

uint32_t ADC_Reads[3];
uint32_t ADC_BUFFER = 3;

//temp
float apps1_as_percent;
float apps2_as_percent;
float bse_as_percent;


/*                    VCU vars end                    */

/*            VCU Method Declarations           */
void readAPPSandBSE(void);
void calculateTorqueRequest(void);
void checkAPPSPlausibility(void);
void checkCrossCheck(void);
void sendTorqueCommand(void);
/*            VCU Method Declarations  end         */


void readAPPSandBSE(void)
{

	// ADC Read Code
	HAL_ADC_Start_DMA(&hadc1, ADC_Reads, ADC_BUFFER);
	if(HAL_GetTick() - millis_since_dma_read >DMA_READ_TIMEOUT){
		apps1Value = ADC_Reads[APPS1_RANK];
		apps2Value = ADC_Reads[APPS2_RANK];
		bseValue = ADC_Reads[BSE_RANK];

		millis_since_dma_read = HAL_GetTick();
	}

//	HAL_ADC_Stop_DMA(&hadc1);


}

void calculateTorqueRequest(void)
{

	float apps1_as_percent = ((float)apps1Value-APPS_1_ADC_MIN_VAL)/(APPS_1_ADC_MAX_VAL-APPS_1_ADC_MIN_VAL);
	float apps2_as_percent = ((float)apps2Value-APPS_2_ADC_MIN_VAL)/(APPS_2_ADC_MAX_VAL-APPS_2_ADC_MIN_VAL);
	float appsValue = ((float)apps1_as_percent + apps2_as_percent)/2;
	if(appsValue >= 0){
		requestedTorque = ((float)(MAX_TORQUE-MIN_TORQUE)) * appsValue + MIN_TORQUE;
	}else{
		float bse_as_percent = ((float)bseValue-BSE_ADC_MIN_VAL)/(BSE_ADC_MAX_VAL-BSE_ADC_MIN_VAL);
		requestedTorque = (REGEN_MAX_TORQUE - REGEN_BASELINE_TORQUE)*bse_as_percent + REGEN_BASELINE_TORQUE;
	}
}

void checkAPPSPlausibility(void)
{

  apps1_as_percent = ((float)apps1Value-APPS_1_ADC_MIN_VAL)/(APPS_1_ADC_MAX_VAL-APPS_1_ADC_MIN_VAL) * 100;
  apps2_as_percent = ((float)apps2Value-APPS_2_ADC_MIN_VAL)/(APPS_2_ADC_MAX_VAL-APPS_2_ADC_MIN_VAL) * 100;

  if(abs(apps1_as_percent-apps2_as_percent)> APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE){
	  if(apps_plausible){
		  millis_since_apps_implausible = HAL_GetTick();
		  apps_plausible = false;
		  requestedTorque = lastRequestedTorque;
	  }else if (HAL_GetTick()-millis_since_apps_implausible>APPS_IMPLAUSIBILITY_TIMEOUT_MILLIS){
		  requestedTorque = 0;
	  }
  }else{
      apps_plausible = true;
  }

}

void checkCrossCheck(void)
{

	bse_as_percent = ((float)bseValue-BSE_ADC_MIN_VAL)/(BSE_ADC_MAX_VAL-BSE_ADC_MIN_VAL) * 100;
	float apps1_as_percent = ((float)apps1Value-APPS_1_ADC_MIN_VAL)/(APPS_1_ADC_MAX_VAL-APPS_1_ADC_MIN_VAL) * 100;
	float apps2_as_percent = ((float)apps2Value-APPS_2_ADC_MIN_VAL)/(APPS_2_ADC_MAX_VAL-APPS_2_ADC_MIN_VAL) * 100;
	float apps_as_percent = ((float)apps1_as_percent+apps2_as_percent)/2;

	if(apps_as_percent > CROSS_CHECK_IMPLAUSIBILITY_APPS_PERCENT && bseValue > BRAKE_ACTIVATED_ADC_VAL){
		if(cross_check_plausible){
			  millis_since_cross_check_implausible = HAL_GetTick();
			  cross_check_plausible = false;
		  }else if (HAL_GetTick()-millis_since_cross_check_implausible>CROSS_CHECK_IMPLAUSIBILITY_TIMEOUT_MILLIS){
			  requestedTorque = 0;
		  }
	}else{
		cross_check_plausible = true;
	}

}

void sendTorqueCommand(void){

	int torqueValue = (int)(requestedTorque * 10);  // Convert to integer, multiply by 10

	// Break the torqueValue into two bytes (little-endian)
	int msg0 = torqueValue & 0xFF;  // Low byte
	int msg1 = (torqueValue >> 8) & 0xFF;  // High byte

	struct can_frame torqueMsg1;
	torqueMsg1.can_id = 0x0C0;
	torqueMsg1.can_dlc = 8;
	for(int i=0;i<8;i++){
		torqueMsg1.data[i] = (msg0 >> i) & 1;
	}

	struct can_frame torqueMsg2;
	torqueMsg2.can_id = 0x0C1;
	torqueMsg2.can_dlc = 8;
	for(int i=0;i<8;i++){
		torqueMsg2.data[i] = (msg1 >> i) & 1;
	}

	struct can_frame directionMsg;
	struct can_frame inverterMsg;

	directionMsg.can_id = 0x0C4;
	directionMsg.can_dlc = 8;
	directionMsg.data[0] = 1;
	for(int i=1;i<8;i++){
		directionMsg.data[i] = 0;
	}

	inverterMsg.can_id = 0x0C5;
	inverterMsg.can_dlc = 8;
	inverterMsg.data[0] = 1;
	for(int i=1;i<8;i++){
		inverterMsg.data[i] = 0;
	}


	MCP_sendMessage(&torqueMsg1);
	MCP_sendMessage(&torqueMsg2);
	MCP_sendMessage(&directionMsg);
	MCP_sendMessage(&inverterMsg);


}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim3);

  MCP_reset();
  MCP_setBitrate(CAN_125KBPS);
  MCP_setNormalMode();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 readAPPSandBSE();
	 calculateTorqueRequest();
	 checkAPPSPlausibility();
	 checkCrossCheck();

	 finalTorqueRequest = requestedTorque;
	 lastRequestedTorque = requestedTorque;

	 sendTorqueCommand();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
