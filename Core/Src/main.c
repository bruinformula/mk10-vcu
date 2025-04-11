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

#include "CANSPI.h"
#include <stdlib.h>     // for abs(), or in our case, use fabsf() from <math.h>
#include <math.h>       // for fabsf()
#include "constants.h"
#include "mcp2515.h"
#include <stdio.h>
#define false 0
#define true 1

//--- BEGIN WAV PLAYBACK DEFINES ---//

// Typical WAV header size
#define WAV_HEADER_SIZE         44
// Number of PCM bytes = total - header
#define WAV_DATA_SIZE           (startup_sound_len - WAV_HEADER_SIZE)
// total halfwords in the PCM data (16-bit)
#define TOTAL_HALFWORDS         (WAV_DATA_SIZE / 2)

// We chunk the wave in up to 30000 halfwords transmissions to avoid the 65535 limit:
#define CHUNK_SIZE_HALFWORDS    30000

// Include your WAV array header
#include "startup_sound.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
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
uCAN_MSG txMessage;
uCAN_MSG rxMessage;
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

uint32_t ADC_Reads[3];
uint32_t ADC_BUFFER = 3;

//temp
float apps1_as_percent;
float apps2_as_percent;
float bse_as_percent;

uint8_t readyToDrive = false;

typedef struct {
    int stateOfCharge;
    int inverterActive;
    int batteryTemperature;
    int faultCode;
    int cellVoltage;
    // Add more fields as needed
} BMSDiagnostics;  // The type alias is BMSDiagnostics

BMSDiagnostics bms_diagnostics;  // Declare a variable of type BMSDiagnostics

typedef struct {
	int motorRpm;
    // Add more fields as needed
} InverterDiagnostics;  // The type alias is BMSDiagnostics

InverterDiagnostics inverter_diagnostics;  // Declare a variable of type BMSDiagnostics

//--- WAV Playback Variables ---//

// global wave position
static uint32_t wavPos = 0;
// pointer to PCM data in Flash (16-bit samples)
static const uint16_t* wavePCM = NULL;
// total halfwords
static uint32_t halfwordCount = 0;
// flag if wave is finished
static uint8_t waveFinished = 0;

//----------------------------------------------------
// VCU method declarations
//----------------------------------------------------
void updateBMSDiagnostics(void);
void readAPPSandBSE(void);
void calculateTorqueRequest(void);
void checkAPPSPlausibility(void);
void checkCrossCheck(void);
void checkReadyToDrive(void);
void updateBMSDiagnostics(void);
void readFromCAN(void);
void updateRpm(void);
void sendTorqueCommand(void);

//----------------------------------------------------
// WAV chunk-based playback
//----------------------------------------------------
static void StartNextChunk(void);
void PlayStartupSoundOnce(void);



void updateRpm(){
 	inverter_diagnostics.motorRpm = (float)(rxMessage.frame.data0 | (rxMessage.frame.data1 << 8));
 }

 void readFromCAN(){
 	if(rxMessage.frame.id == RPM_READ_ID){
 		updateRpm();
 	}
 }


void updateBMSDiagnostics(void){
    // do nothing for now
}

void readAPPSandBSE(void)
{
    // Start ADC DMA read
    HAL_ADC_Start_DMA(&hadc1, ADC_Reads, ADC_BUFFER);
    if(HAL_GetTick() - millis_since_dma_read > DMA_READ_TIMEOUT){
        apps1Value = ADC_Reads[APPS1_RANK];
        apps2Value = ADC_Reads[APPS2_RANK];
        bseValue   = ADC_Reads[BSE_RANK];
        millis_since_dma_read = HAL_GetTick();
    }
}

void calculateTorqueRequest(void)
{

	float apps1_as_percent = ((float)apps1Value-APPS_1_ADC_MIN_VAL)/(APPS_1_ADC_MAX_VAL-APPS_1_ADC_MIN_VAL);
	float apps2_as_percent = ((float)apps2Value-APPS_2_ADC_MIN_VAL)/(APPS_2_ADC_MAX_VAL-APPS_2_ADC_MIN_VAL);
	float appsValue = ((float)apps1_as_percent + apps2_as_percent)/2;
	if(appsValue > 0){
	 		int numPedalSteps = 10;
	 		int numRpmSteps = 10;

	 		float pedalStepSize = 100 / (numPedalSteps - 1);
	 		float rpmStepSize = MAX_RPM / (numRpmSteps - 1);

	 		int pedalLowIndx = (int)(appsValue/pedalStepSize);
	 		int pedalHighIndx = pedalLowIndx + 1;
	 		if (pedalHighIndx >= numPedalSteps) pedalHighIndx = numPedalSteps - 1;

	 		int rpmLowIndx = (int)(inverter_diagnostics.motorRpm / rpmStepSize);
	 		int rpmHighIndx = rpmLowIndx + 1;
	 		if (rpmHighIndx >= numRpmSteps) rpmHighIndx = numRpmSteps - 1;

	 	    float T00 = TORQUE_ARRAY[pedalLowIndx][rpmLowIndx];  // Lower-left
	 	    float T10 = TORQUE_ARRAY[pedalHighIndx][rpmLowIndx]; // Upper-left
	 	    float T01 = TORQUE_ARRAY[pedalLowIndx][rpmHighIndx]; // Lower-right
	 	    float T11 = TORQUE_ARRAY[pedalHighIndx][rpmHighIndx]; // Upper-right

	 	    // Compute interpolation weights
	 	    float pedalLerp = (appsValue - (pedalLowIndx * pedalStepSize)) / pedalStepSize;
	 	    float rpmLerp = (inverter_diagnostics.motorRpm - (rpmLowIndx * rpmStepSize)) / rpmStepSize;

	 	    float torqueLow = T00 + (T01 - T00) * rpmLerp;
	 	    float torqueHigh = T10 + (T11 - T10) * rpmLerp;

	 		requestedTorque =  torqueLow + (torqueHigh - torqueLow) * pedalLerp;

	}else{
		float bse_as_percent = ((float)bseValue-BSE_ADC_MIN_VAL)/(BSE_ADC_MAX_VAL-BSE_ADC_MIN_VAL);
		requestedTorque = (REGEN_MAX_TORQUE - REGEN_BASELINE_TORQUE)*bse_as_percent + REGEN_BASELINE_TORQUE;
	}
}

void checkAPPSPlausibility(void)
{
    apps1_as_percent = ((float)apps1Value - APPS_1_ADC_MIN_VAL)
                      /(APPS_1_ADC_MAX_VAL - APPS_1_ADC_MIN_VAL)*100.0f;
    apps2_as_percent = ((float)apps2Value - APPS_2_ADC_MIN_VAL)
                      /(APPS_2_ADC_MAX_VAL - APPS_2_ADC_MIN_VAL)*100.0f;

    // use fabsf() for float
    if(fabsf(apps1_as_percent - apps2_as_percent) > APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE){
        millis_since_apps_implausible = HAL_GetTick();
        apps_plausible = 0; // false
        requestedTorque = 0;
    }
    else if(!apps_plausible && (HAL_GetTick() - millis_since_apps_implausible < APPS_IMPLAUSIBILITY_TIMEOUT_MILLIS)){
        requestedTorque = 0;
    }
    else {
        apps_plausible = 1; // true
    }
}

void checkCrossCheck(void)
{
    bse_as_percent = ((float)bseValue - BSE_ADC_MIN_VAL)/(BSE_ADC_MAX_VAL - BSE_ADC_MIN_VAL)*100.0f;

    float apps1p = ((float)apps1Value - APPS_1_ADC_MIN_VAL)/(APPS_1_ADC_MAX_VAL - APPS_1_ADC_MIN_VAL)*100.0f;
    float apps2p = ((float)apps2Value - APPS_2_ADC_MIN_VAL)/(APPS_2_ADC_MAX_VAL - APPS_2_ADC_MIN_VAL)*100.0f;
    float apps_as_percent = (apps1p + apps2p)/2.0f;

    if (apps_as_percent > CROSS_CHECK_IMPLAUSIBILITY_APPS_PERCENT && bseValue > BRAKE_ACTIVATED_ADC_VAL){
        cross_check_plausible = 0;
        requestedTorque = 0;
    }
    else if(!cross_check_plausible && apps_as_percent > CROSS_CHECK_RESTORATION_APPS_PERCENT){
        requestedTorque = 0;
    }
    else {
        cross_check_plausible = 1;
    }
}

void sendTorqueCommand(void){

	int torqueValue = (int)(requestedTorque * 10);  // Convert to integer, multiply by 10

	// Break the torqueValue into two bytes (little-endian)
	char msg0 = torqueValue & 0xFF;  // Low byte
	char msg1 = (torqueValue >> 8) & 0xFF;  // High byte

	txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	txMessage.frame.id = 0x0C0;
	txMessage.frame.dlc = 8;
	txMessage.frame.data0 = msg0;
	txMessage.frame.data1 = msg1;
	txMessage.frame.data2 = 0;
	txMessage.frame.data3 = 0;
	txMessage.frame.data4 = 0;
	txMessage.frame.data5 = 0;
	txMessage.frame.data6 = 0;
	txMessage.frame.data7 = 0;
	CANSPI_Transmit(&txMessage);
}

void checkReadyToDrive(void)
{
    uint8_t pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5); // example
    if (pinState == GPIO_PIN_SET && bseValue > BRAKE_ACTIVATED_ADC_VAL) {
        readyToDrive = 1;
    }
}


//-----------------------------------------------
// I2S chunk-based WAV Playback Methods
//-----------------------------------------------
/**
  * @brief Called by HAL when a DMA transmission completes (for one chunk).
  */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2 && !waveFinished)
    {
        // finished one chunk
        if (wavPos < halfwordCount)
        {
            StartNextChunk();
        }
        else
        {
            // entire wave is done
            waveFinished = 1;
        }
    }
}

/**
  * @brief Start the next chunk of PCM in Normal DMA mode
  */
static void StartNextChunk(void)
{
    // how many halfwords remain
    uint32_t remain = halfwordCount - wavPos;
    // pick chunk
    uint16_t thisChunk = (remain > CHUNK_SIZE_HALFWORDS)
                        ? CHUNK_SIZE_HALFWORDS
                        : (uint16_t)remain;

    const uint16_t* chunkPtr = wavePCM + wavPos;
    wavPos += thisChunk;

    // Fire the DMA
    HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)chunkPtr, thisChunk);
}

/**
  * @brief Public function to play the wave from beginning exactly once
  */
void PlayStartupSoundOnce(void)
{
    wavePCM       = (const uint16_t*)&startup_sound[WAV_HEADER_SIZE];
    halfwordCount = TOTAL_HALFWORDS;
    wavPos        = 0;
    waveFinished  = 0;

    // Start the first chunk
    StartNextChunk();
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
  // Start TIM3
  HAL_TIM_Base_Start(&htim3);

  /* initalized to be 500kbps, see canspi.c line 131-133 for details */
  if (CANSPI_Initialize() != true)
  {
	  Error_Handler();
  }


  bms_diagnostics.inverterActive = 0;
  inverter_diagnostics.motorRpm = 0;
  PlayStartupSoundOnce();

  /* USER CODE END 2 */

  /* Infinite loop */

  while (1)
  {
	  /* USER CODE BEGIN WHILE */

	 if(CANSPI_Receive(&rxMessage)){
		 readFromCAN();
	 }


	 readAPPSandBSE();
	 calculateTorqueRequest();
	 checkAPPSPlausibility();
	 checkCrossCheck();
	 checkReadyToDrive();
	 updateBMSDiagnostics();

	 finalTorqueRequest = requestedTorque;
	 lastRequestedTorque = requestedTorque;

	 if(readyToDrive){
		 sendTorqueCommand();
	 }
    /* USER CODE END WHILE */

    // The wave playback is handled by DMA + callback, so no blocking here
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  // never reached
}
  /* USER CODE END 3 */


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
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
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
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* shit for printf support */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the LPUART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
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
