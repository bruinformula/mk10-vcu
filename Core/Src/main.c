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
typedef struct {
	int inverterActive;
	int packVoltage;
	int packCurrent;
// Add more fields as needed
} BMSDiagnostics;  // The type alias is BMSDiagnostics

typedef struct {
	int motorRpm;
	int inverterDCVolts;
// Add more fields as needed
} InverterDiagnostics;  // The type alias is InverterDiagnostics
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

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uCAN_MSG txMessage;
uCAN_MSG rxMessage;

volatile uint32_t apps1Value = 0;
volatile uint32_t apps2Value = 0;
volatile uint32_t bseValue = 0;

uint32_t apps1Buffer[ADC_READ_BUFFER] = { 0 };
uint32_t apps2Buffer[ADC_READ_BUFFER] = { 0 };
uint32_t bseBuffer[ADC_READ_BUFFER] = { 0 };
uint8_t adcBufferIndex = 0;

float requestedTorque;
float lastRequestedTorque = 0;
float finalTorqueRequest;

uint8_t beginTorqueRequests = false;

// APPS plausibility
uint16_t apps_plausible = true;
uint32_t millis_since_apps_implausible;
uint8_t dma_read_complete = 1;
uint32_t millis_since_dma_read = 0;

// Cross-check plausibility
uint16_t cross_check_plausible = true;

// ADC buffer
uint32_t ADC_Reads[ADC_BUFFER];

// Temp variables for display or logic
float apps1_as_percent;
float apps2_as_percent;
float bse_as_percent;

// BSE / brake pressed logic
uint8_t readyToDrive = false;
uint8_t rtdState = false;
uint32_t cpockandballs;
uint32_t millis_RTD;
uint8_t prechargeState = false;
uint8_t prechargeFinished = false;
uint32_t millis_precharge;

// Diagnostics structures
BMSDiagnostics bms_diagnostics;
InverterDiagnostics inverter_diagnostics;

//--- WAV Playback Variables ---//
static uint32_t wavPos = 0;                // global wave position index
static const uint16_t *wavePCM = NULL;     // pointer to PCM data in Flash
static uint32_t halfwordCount = 0;         // total halfwords
static uint8_t waveFinished = 0;           // flag if wave is finished

// customiABILTY shits
const uint8_t BMS_TYPE = ORION_BMS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */

// VCU method declarations
void updateBMSDiagnostics(void);
void readFromCAN(void);
void updateRpm(void);
void calculateTorqueRequest(void);
void checkAPPSPlausibility(void);
void checkCrossCheck(void);
void checkReadyToDrive(void);
void sendTorqueCommand(void);
void sendPrechargeRequest(void);
uint8_t prechargeSequence(void);
void checkShutdown(void);
void PlayStartupSoundOnce(void);
void updateInverterVolts(void);
void lookForRTD(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int compare_uint32_t(const void *a, const void *b) {
	return (*(uint32_t*) a - *(uint32_t*) b);
}

uint32_t median_uint32_t(uint32_t *buffer, uint8_t size) {
	uint32_t temp[ADC_READ_BUFFER];
	memcpy(temp, buffer, size * sizeof(uint32_t));
	qsort(temp, size, sizeof(uint32_t), compare_uint32_t);
	return temp[size / 2];
}

/**
 * @brief ADC Conversion Complete callback. Copies the DMA buffer into apps/bse values.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1) {
	// Store latest samples into buffers
	apps1Buffer[adcBufferIndex] = ADC_Reads[APPS1_RANK - 1];
	apps2Buffer[adcBufferIndex] = ADC_Reads[APPS2_RANK - 1];
	bseBuffer[adcBufferIndex] = ADC_Reads[BSE_RANK - 1];

	// Move buffer index circularly
	adcBufferIndex = (adcBufferIndex + 1) % ADC_READ_BUFFER;

	// Compute and assign median
	apps1Value = median_uint32_t(apps1Buffer, ADC_READ_BUFFER);
	apps2Value = median_uint32_t(apps2Buffer, ADC_READ_BUFFER);
	bseValue = median_uint32_t(bseBuffer, ADC_READ_BUFFER);

	dma_read_complete = 1;
}

//-----------------------------------------------
// I2S chunk-based WAV Playback Methods
//-----------------------------------------------
/**
 * @brief Called by HAL when a DMA transmission completes (for one chunk).
 */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
	if (hi2s->Instance == SPI2 && !waveFinished) {
		// finished one chunk
		if (wavPos < halfwordCount) {
			// Start the next chunk
			uint32_t remain = halfwordCount - wavPos;
			uint16_t thisChunk =
					(remain > CHUNK_SIZE_HALFWORDS) ?
							CHUNK_SIZE_HALFWORDS : (uint16_t) remain;
			const uint16_t *chunkPtr = wavePCM + wavPos;
			wavPos += thisChunk;

			HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*) chunkPtr, thisChunk);
		} else {
			// entire wave is done
			waveFinished = 1;
		}
	}
}s

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_SPI3_Init();
	MX_I2S2_Init();
	/* USER CODE BEGIN 2 */
	// Start TIM3
	HAL_TIM_Base_Start(&htim3);

	// Initialize the CAN at 500kbps (CANSPI_Initialize sets the MCP2515)
	if (CANSPI_Initialize() != true) {
		Error_Handler();
	}

	// Initialize some diagnostics values
//	bms_diagnostics.inverterActive = 1;
	inverter_diagnostics.motorRpm = 1;

	/* TEST RELAYS. COMMENT OUT BEFORE RUNNING */
//	HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin, GPIO_PIN_SET); //steps 1 and 2
//	HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin, GPIO_PIN_RESET); //steps 1 and 2
//
//	HAL_Delay(3000);
//	HAL_GPIO_WritePin(AIR_N_CTRL_GPIO_Port, AIR_N_CTRL_Pin, GPIO_PIN_SET); //steps 1 and 2
//	HAL_GPIO_WritePin(AIR_N_CTRL_GPIO_Port, AIR_N_CTRL_Pin, GPIO_PIN_RESET); //steps 1 and 2
//
//	HAL_Delay(3000);
//	HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin, GPIO_PIN_SET); //steps 1 and 2
//	HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin, GPIO_PIN_RESET); //steps 1 and 2
//	while(1) {};
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	/* initial bootup look for RTD */
	lookForRTD();

	/* DRIVE LOOP */
	while (1) {

//		txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
//		txMessage.frame.id = 0b10000000011;
//		txMessage.frame.dlc = 8;
//		txMessage.frame.data0 = 0x61;
//		txMessage.frame.data1 = 0x73;
//		txMessage.frame.data2 = 0x73;
//		txMessage.frame.data3 = 0x68;
//		txMessage.frame.data4 = 0x6F;
//		txMessage.frame.data5 = 0x6C;
//		txMessage.frame.data6 = 0x65;
//		txMessage.frame.data7 = 0x73;
//		CANSPI_Transmit(&txMessage);
//
//		HAL_Delay(100);
//
//
//		if(CANSPI_Receive(&rxMessage))
//		{
//			uCAN_MSG orangeMessage = rxMessage;
//			uCAN_MSG ppMesage;
//			ppMesage.frame.idType = rxMessage.frame.idType;
//			ppMesage.frame.id = rxMessage.frame.id;
//			ppMesage.frame.dlc = rxMessage.frame.dlc;
//			ppMesage.frame.data0 = rxMessage.frame.data0 | 0xAA;
//			ppMesage.frame.data1 = rxMessage.frame.data1 | 0xAA;
//			ppMesage.frame.data2 = rxMessage.frame.data2 | 0xAA;
//			ppMesage.frame.data3 = rxMessage.frame.data3 | 0xAA;
//			ppMesage.frame.data4 = rxMessage.frame.data4 | 0xAA;
//			ppMesage.frame.data5 = rxMessage.frame.data5 | 0xAA;
//			ppMesage.frame.data6 = rxMessage.frame.data6 | 0xAA;
//			ppMesage.frame.data7 = rxMessage.frame.data7 | 0xAA;
//			uCAN_MSG ppMessage2 = ppMesage;
//
//			CANSPI_Transmit(&ppMesage);
//
//		}
//
		// Check for new CAN data
		if (CANSPI_Receive(&rxMessage)) {
			readFromCAN();
		}

		if (dma_read_complete) {
			HAL_ADC_Start_DMA(&hadc1, ADC_Reads, ADC_BUFFER);
			dma_read_complete = 0;
			millis_since_dma_read = HAL_GetTick();
		}

		updateBMSDiagnostics();

		// Periodically do your torque calculations:
		calculateTorqueRequest();
		apps_plausible = checkAPPSPlausibility();
		checkCrossCheck();

		checkShutdown();  // If pin is low, torque->0, block

		finalTorqueRequest = requestedTorque;
		lastRequestedTorque = requestedTorque;

		if (readyToDrive || rtdoverride == 1)
			sendTorqueCommand();
		// ... do other tasks as needed ...
	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	// Should never get here
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 2;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2S2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S2_Init(void) {

	/* USER CODE BEGIN I2S2_Init 0 */

	/* USER CODE END I2S2_Init 0 */

	/* USER CODE BEGIN I2S2_Init 1 */

	/* USER CODE END I2S2_Init 1 */
	hi2s2.Instance = SPI2;
	hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
	hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
	hi2s2.Init.CPOL = I2S_CPOL_LOW;
	hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2S2_Init 2 */

	/* USER CODE END I2S2_Init 2 */

}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {

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
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 4;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 10001;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			LED1_Pin | PCHG_RLY_CTRL_Pin | AIR_P_CTRL_Pin | AIR_N_CTRL_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, CAN_CS_Pin | CAN1_RESET_Pin | CAN2_RESET_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CAN2_CS_GPIO_Port, CAN2_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : LED1_Pin PCHG_RLY_CTRL_Pin AIR_P_CTRL_Pin AIR_N_CTRL_Pin */
	GPIO_InitStruct.Pin = LED1_Pin | PCHG_RLY_CTRL_Pin | AIR_P_CTRL_Pin
			| AIR_N_CTRL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : CAN_CS_Pin */
	GPIO_InitStruct.Pin = CAN_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(CAN_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : CAN1_RESET_Pin CAN2_RESET_Pin */
	GPIO_InitStruct.Pin = CAN1_RESET_Pin | CAN2_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA2 PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LED2_Pin */
	GPIO_InitStruct.Pin = LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RTD_BTN_Pin PB5 */
	GPIO_InitStruct.Pin = RTD_BTN_Pin | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PRECHARGE_BTN_Pin */
	GPIO_InitStruct.Pin = PRECHARGE_BTN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PRECHARGE_BTN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CAN2_CS_Pin */
	GPIO_InitStruct.Pin = CAN2_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(CAN2_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SHUTDOWN_Pin */
	GPIO_InitStruct.Pin = SHUTDOWN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(SHUTDOWN_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
