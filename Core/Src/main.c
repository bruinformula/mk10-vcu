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
} BMSDiagnostics;  // The type alias is BMSDiagnostics

typedef struct {
	int motorRpm;
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
volatile uint32_t bseValue   = 0;

float requestedTorque      = 0.0f;
float lastRequestedTorque  = 0.0f;
float finalTorqueRequest   = 0.0f;

uint8_t beginTorqueRequests = false;

// APPS plausibility
uint16_t apps_plausible            = true;
uint32_t millis_since_apps_implausible = 0;
uint8_t  dma_read_complete         = 1;
uint32_t millis_since_dma_read     = 0;

// Cross-check plausibility
uint16_t cross_check_plausible = true;

// ADC buffer
uint32_t ADC_Reads[ADC_BUFFER];

// Temp variables for display or logic
float apps1_as_percent   = 0.0f;
float apps2_as_percent   = 0.0f;
float bse_as_percent     = 0.0f;

// BSE / brake pressed logic
uint8_t readyToDrive   = false;
uint8_t rtdState       = false;
uint32_t millis_RTD    = 0;
uint8_t prechargeState = false;
uint32_t millis_precharge = 0;

// Diagnostics structures
BMSDiagnostics      bms_diagnostics;
InverterDiagnostics inverter_diagnostics;

//--- WAV Playback Variables ---//
static uint32_t wavPos        = 0; // global wave position index
static const uint16_t *wavePCM = NULL; // pointer to PCM data in Flash
static uint32_t halfwordCount  = 0; // total halfwords
static uint8_t waveFinished    = 0; // flag if wave is finished

uint8_t rtdoverride = 0;  // for testing

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

void updateBMSDiagnostics(void);
void readFromCAN(void);
void updateRpm(void);
void calculateTorqueRequest(void);
void checkAPPSPlausibility(void);
void checkCrossCheck(void);
void checkReadyToDrive(void);
void sendTorqueCommand(void);
void sendPrechargeRequest(void);
void prechargeSequence(void);
void checkShutdown(void);
void PlayStartupSoundOnce(void);
void lookForRTD(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Repeatedly check the shutdown pin; if high, set torque to 0 and block forever.
 */
void checkShutdown()
{
	uint8_t pinState = HAL_GPIO_ReadPin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin);
	// Adjust logic if your hardware is active-high or active-low
	if (pinState == GPIO_PIN_RESET) {  // If it's "active" at RESET
		requestedTorque = 0;
		sendTorqueCommand();
		while(true) { } // block forever
	}
}

/**
 * @brief Update Inverter RPM reading from the last received CAN message.
 */
void updateRpm() {
	inverter_diagnostics.motorRpm = (float)(rxMessage.frame.data2
			| (rxMessage.frame.data3 << 8));
}

/**
 * @brief Read relevant data from incoming CAN messages
 */
void readFromCAN() {
	if (rxMessage.frame.id == RPM_READ_ID) {
		updateRpm();
	}
	else if(rxMessage.frame.id == BMS_DIAGNOSTICS_ID){
		updateBMSDiagnostics();
	}
}

/**
 * @brief Parse BMS diagnostics from a received CAN message
 */
void updateBMSDiagnostics(void) {
	int16_t pack_current_raw =
		(int16_t)((rxMessage.frame.data1 << 8) | rxMessage.frame.data0);
	float pack_current = pack_current_raw * 0.1f;

	uint16_t pack_voltage_raw =
		(rxMessage.frame.data3 << 8) | rxMessage.frame.data2;
	float pack_voltage = pack_voltage_raw * 0.1f;

	bool is_ready = ((rxMessage.frame.data6 >> 6) & 0x01);

	bms_diagnostics.inverterActive = is_ready ? 1 : 0;
	bms_diagnostics.packCurrent    = (int)pack_current;
	bms_diagnostics.packVoltage    = (int)pack_voltage;
}

/**
 * @brief ADC Conversion Complete callback. Copies the DMA buffer into apps/bse values.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	// Because DMAContinuousRequests is disabled and triggered by TIM3,
	// this will be called each time the sequence finishes.
	//
	// Make sure APPS1_RANK = 1, APPS2_RANK = 2, BSE_RANK = 3 in your "constants.h"
	// so that the indexing below is correct.
	apps1Value = ADC_Reads[APPS1_RANK - 1];
	apps2Value = ADC_Reads[APPS2_RANK - 1];
	bseValue   = ADC_Reads[BSE_RANK  - 1];

	dma_read_complete = 1;
}

/**
 * @brief Calculate the requested torque based on APPS and RPM, or regen based on BSE.
 */
void calculateTorqueRequest(void)
{
	float apps1_as_percent = ((float) apps1Value - APPS_1_ADC_MIN_VAL)
					/ (APPS_1_ADC_MAX_VAL - APPS_1_ADC_MIN_VAL);
	float apps2_as_percent = ((float) apps2Value - APPS_2_ADC_MIN_VAL)
					/ (APPS_2_ADC_MAX_VAL - APPS_2_ADC_MIN_VAL);
	float appsValue = (apps1_as_percent + apps2_as_percent) / 2.0f;

	if (appsValue > 0.0f) {
		// Pedal-based torque map
		int numPedalSteps = 10;
		int numRpmSteps   = 10;

		float pedalStepSize = 100.0f / (numPedalSteps - 1);
		float rpmStepSize   = MAX_RPM / (numRpmSteps - 1);

		int pedalLowIndx = (int)(appsValue * 100.0f / pedalStepSize);
		int pedalHighIndx = pedalLowIndx + 1;
		if (pedalHighIndx >= numPedalSteps) {
			pedalHighIndx = numPedalSteps - 1;
		}

		int rpmLowIndx = (int)(inverter_diagnostics.motorRpm / rpmStepSize);
		int rpmHighIndx = rpmLowIndx + 1;
		if (rpmHighIndx >= numRpmSteps) {
			rpmHighIndx = numRpmSteps - 1;
		}

		float T00 = TORQUE_ARRAY[pedalLowIndx][rpmLowIndx];   // Lower-left
		float T10 = TORQUE_ARRAY[pedalHighIndx][rpmLowIndx];  // Upper-left
		float T01 = TORQUE_ARRAY[pedalLowIndx][rpmHighIndx];  // Lower-right
		float T11 = TORQUE_ARRAY[pedalHighIndx][rpmHighIndx]; // Upper-right

		float pedalBase = (appsValue * 100.0f) - (pedalLowIndx * pedalStepSize);
		float pedalLerp = pedalBase / pedalStepSize;

		float rpmBase = (float)inverter_diagnostics.motorRpm
		                - (rpmLowIndx * rpmStepSize);
		float rpmLerp = rpmBase / rpmStepSize;

		float torqueLow  = T00 + (T01 - T00) * rpmLerp;
		float torqueHigh = T10 + (T11 - T10) * rpmLerp;

		requestedTorque = torqueLow + (torqueHigh - torqueLow) * pedalLerp;
	}
	else {
		// Regen based on brake pedal
		float bse_as_percent = ((float) bseValue - BSE_ADC_MIN_VAL)
			/ (BSE_ADC_MAX_VAL - BSE_ADC_MIN_VAL);

		requestedTorque =
			(REGEN_MAX_TORQUE - REGEN_BASELINE_TORQUE) * bse_as_percent
			+ REGEN_BASELINE_TORQUE;
	}
}

/**
 * @brief Check plausibility of APPS sensors.
 */
void checkAPPSPlausibility(void)
{
	apps1_as_percent = ((float) apps1Value - APPS_1_ADC_MIN_VAL)
				/ (APPS_1_ADC_MAX_VAL - APPS_1_ADC_MIN_VAL) * 100.0f;
	apps2_as_percent = ((float) apps2Value - APPS_2_ADC_MIN_VAL)
				/ (APPS_2_ADC_MAX_VAL - APPS_2_ADC_MIN_VAL) * 100.0f;

	if (fabsf(apps1_as_percent - apps2_as_percent)
			> APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE)
	{
		millis_since_apps_implausible = HAL_GetTick();
		apps_plausible = 0;
		requestedTorque = 0;
	}
	else if (!apps_plausible &&
		(HAL_GetTick() - millis_since_apps_implausible
		 < APPS_IMPLAUSIBILITY_TIMEOUT_MILLIS))
	{
		requestedTorque = 0;
	}
	else {
		apps_plausible = 1;
	}
}

/**
 * @brief Check cross-check between APPS and brake pedal.
 */
void checkCrossCheck(void)
{
	bse_as_percent = ((float) bseValue - BSE_ADC_MIN_VAL)
				/ (BSE_ADC_MAX_VAL - BSE_ADC_MIN_VAL) * 100.0f;

	float apps1p = ((float) apps1Value - APPS_1_ADC_MIN_VAL)
				/ (APPS_1_ADC_MAX_VAL - APPS_1_ADC_MIN_VAL) * 100.0f;
	float apps2p = ((float) apps2Value - APPS_2_ADC_MIN_VAL)
				/ (APPS_2_ADC_MAX_VAL - APPS_2_ADC_MIN_VAL) * 100.0f;
	float apps_as_percent = (apps1p + apps2p) / 2.0f;

	if (apps_as_percent > CROSS_CHECK_IMPLAUSIBILITY_APPS_PERCENT
	    && bseValue > BRAKE_ACTIVATED_ADC_VAL)
	{
		cross_check_plausible = 0;
		requestedTorque = 0;
	}
	else if (!cross_check_plausible
		 && apps_as_percent > CROSS_CHECK_RESTORATION_APPS_PERCENT)
	{
		requestedTorque = 0;
	}
	else {
		cross_check_plausible = 1;
	}
}

/**
 * @brief Send the torque command message over CAN.
 */
void sendTorqueCommand(void)
{
	int torqueValue = (int)(requestedTorque * 10); // multiply by 10

	// Break the torqueValue into two bytes (little-endian)
	char msg0 = (char)(torqueValue & 0xFF);
	char msg1 = (char)((torqueValue >> 8) & 0xFF);

	txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	txMessage.frame.id     = 0x0C0;
	txMessage.frame.dlc    = 8;

	txMessage.frame.data0  = msg0;
	txMessage.frame.data1  = msg1;
	txMessage.frame.data2  = 0;
	txMessage.frame.data3  = 0;
	txMessage.frame.data4  = 1;

	// lockout
	if (beginTorqueRequests) {
		txMessage.frame.data5 = 0;
	}
	else {
		txMessage.frame.data5 = 1;
	}
	txMessage.frame.data6  = 0;
	txMessage.frame.data7  = 0;

	CANSPI_Transmit(&txMessage);
}

/**
 * @brief Check if the driver has pressed the brake pedal and the RTD pin is set.
 */
void checkReadyToDrive(void)
{
	uint8_t pinState = HAL_GPIO_ReadPin(RTD_GPIO_Port, RTD_Pin);
	// Example logic: if pin is SET & bse < threshold & BMS is ready => set RTD
	if (pinState == GPIO_PIN_SET && bseValue < BRAKE_ACTIVATED_ADC_VAL
	    && bms_diagnostics.inverterActive && !rtdState)
	{
		rtdState   = true;
		millis_RTD = HAL_GetTick();
	}
	else if (pinState == GPIO_PIN_RESET
	         || bseValue > BRAKE_ACTIVATED_ADC_VAL
	         || !bms_diagnostics.inverterActive)
	{
		rtdState = false;
	}
	else if ((HAL_GetTick() - millis_precharge) >= RTD_BUTTON_PRESS_MILLIS) {
		readyToDrive = true;
	}
}

/**
 * @brief If a hardware pin requests precharge, triggers precharge sequence
 */
void sendPrechargeRequest(void)
{
	uint8_t pinState = HAL_GPIO_ReadPin(PRECHARGE_GPIO_Port, PRECHARGE_Pin);
	if (pinState == GPIO_PIN_RESET && !prechargeState) {
		prechargeState    = true;
		millis_precharge  = HAL_GetTick();
	}
	else if (pinState == GPIO_PIN_SET) {
		prechargeState = false;
	}
	else if ((HAL_GetTick() - millis_precharge) >= PRECHARGE_BUTTON_PRESS_MILLIS) {
		prechargeSequence();
	}
}

/**
 * @brief Actual procedure of triggering precharge relays
 */
void prechargeSequence(void)
{
	// fill this in as needed
}

//-----------------------------------------------
// I2S chunk-based WAV Playback Methods
//-----------------------------------------------
/**
 * @brief Called by HAL when a DMA transmission completes (for one chunk).
 */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	if (hi2s->Instance == SPI2 && !waveFinished) {
		// finished one chunk
		if (wavPos < halfwordCount) {
			// Start the next chunk
			uint32_t remain    = halfwordCount - wavPos;
			uint16_t thisChunk = (remain > CHUNK_SIZE_HALFWORDS)
			                     ? CHUNK_SIZE_HALFWORDS
			                     : (uint16_t)remain;
			const uint16_t *chunkPtr = wavePCM + wavPos;
			wavPos += thisChunk;

			HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)chunkPtr, thisChunk);
		}
		else {
			// entire wave is done
			waveFinished = 1;
		}
	}
}

/**
 * @brief Play the startup sound from Flash exactly once.
 */
void PlayStartupSoundOnce(void)
{
	wavePCM       = (const uint16_t*)(&startup_sound[WAV_HEADER_SIZE]);
	halfwordCount = TOTAL_HALFWORDS;
	wavPos        = 0;
	waveFinished  = 0;

	// First chunk
	uint32_t remain    = halfwordCount - wavPos;
	uint16_t thisChunk = (remain > CHUNK_SIZE_HALFWORDS)
	                     ? CHUNK_SIZE_HALFWORDS
	                     : (uint16_t)remain;
	const uint16_t *chunkPtr = wavePCM + wavPos;
	wavPos += thisChunk;

	HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)chunkPtr, thisChunk);
}

/**
 * @brief Wait for driver to enable RTD or override
 */
void lookForRTD(void)
{
	if (rtdoverride == 1) {
		beginTorqueRequests = true;
		PlayStartupSoundOnce();
		return;
	}
	while(!readyToDrive) {
		// If you want to read CAN or do other tasks while waiting
		if (CANSPI_Receive(&rxMessage)) {
			readFromCAN();
		}

		// Periodically start ADC if needed
		if (((HAL_GetTick() - millis_since_dma_read) > DMA_READ_TIMEOUT)
		     && dma_read_complete)
		{
			HAL_ADC_Start_DMA(&hadc1, ADC_Reads, ADC_BUFFER);
			dma_read_complete     = 0;
			millis_since_dma_read = HAL_GetTick();
		}

		// handle precharge if needed
		sendPrechargeRequest();

		// check if RTD can be triggered
		checkReadyToDrive();
		if (readyToDrive) {
			beginTorqueRequests = true;
			PlayStartupSoundOnce();
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // (empty)
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

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

  // Start ADC+DMA once here. This ensures the pedal readings begin immediately.
  HAL_ADC_Start_DMA(&hadc1, ADC_Reads, ADC_BUFFER);

  // Initialize the CAN at 500kbps (CANSPI_Initialize sets the MCP2515)
  if (CANSPI_Initialize() != true) {
  	Error_Handler();
  }

  // Initialize some diagnostics values
  bms_diagnostics.inverterActive = 1;
  inverter_diagnostics.motorRpm   = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // On boot, wait for RTD or override
  lookForRTD();

  // DRIVE LOOP
  while (1)
  {
	  // Check for new CAN data
	  if (CANSPI_Receive(&rxMessage)) {
		  readFromCAN();
	  }

	  // Periodically re-start ADC if needed
	  if (((HAL_GetTick() - millis_since_dma_read) > DMA_READ_TIMEOUT)
	       && dma_read_complete)
	  {
		  HAL_ADC_Start_DMA(&hadc1, ADC_Reads, ADC_BUFFER);
		  dma_read_complete     = 0;
		  millis_since_dma_read = HAL_GetTick();
	  }

	  // Example: update BMS info
	  updateBMSDiagnostics();

	  // Periodically do torque calculations
	  calculateTorqueRequest();
	  checkAPPSPlausibility();
	  checkCrossCheck();

	  // If you want to lock out the system when "shutdown" is active
	  // checkShutdown();

	  finalTorqueRequest  = requestedTorque;
	  lastRequestedTorque = requestedTorque;

	  // If the driver is ready to drive (or override), send torque
	  if (readyToDrive || (rtdoverride == 1)) {
		  sendTorqueCommand();
	  }

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
void SystemClock_Config(void)
{
  /* NOTE: Original code, just re-ordered your PLL to 84 MHz. */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* 1) Configure HSI + PLL */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState           = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue= RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState       = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource      = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM           = 8;
  RCC_OscInitStruct.PLL.PLLN           = 84;
  RCC_OscInitStruct.PLL.PLLP           = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ           = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* 2) Configure bus clocks */
  RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK
                                     |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
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
  *
  * Re-ordered so Rank1 => Channel1 (APPS1), Rank2 => Channel14 (APPS2),
  * Rank3 => Channel15 (BSE).
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance                      = ADC1;
  hadc1.Init.ClockPrescaler          = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution              = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode            = ENABLE;
  hadc1.Init.ContinuousConvMode      = DISABLE;
  hadc1.Init.DiscontinuousConvMode   = DISABLE;
  hadc1.Init.ExternalTrigConvEdge    = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv        = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign               = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion         = 3;
  hadc1.Init.DMAContinuousRequests   = DISABLE;
  hadc1.Init.EOCSelection            = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  // Channel 1 => RANK 1 => APPS1
  sConfig.Channel      = ADC_CHANNEL_1;
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  // Channel 14 => RANK 2 => APPS2
  sConfig.Channel      = ADC_CHANNEL_14;
  sConfig.Rank         = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  // Channel 15 => RANK 3 => BSE
  sConfig.Channel      = ADC_CHANNEL_15;
  sConfig.Rank         = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{
  hi2s2.Instance         = SPI2;
  hi2s2.Init.Mode        = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard    = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat  = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput  = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq   = I2S_AUDIOFREQ_44K;
  hi2s2.Init.CPOL        = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{
  hspi3.Instance               = SPI3;
  hspi3.Init.Mode             = SPI_MODE_MASTER;
  hspi3.Init.Direction        = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize         = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity      = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase         = SPI_PHASE_1EDGE;
  hspi3.Init.NSS              = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler= SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit         = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode           = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation   = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial    = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig     = {0};

  htim3.Instance               = TIM3;
  htim3.Init.Prescaler         = 4;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 10001;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  // DMA interrupt init
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // LED1
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  // CAN CS & RESET pins default high
  HAL_GPIO_WritePin(GPIOC, CAN_CS_Pin|CAN1_RESET_Pin|CAN2_RESET_Pin, GPIO_PIN_SET);

  // LED2
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  // CAN2 CS
  HAL_GPIO_WritePin(CAN2_CS_GPIO_Port, CAN2_CS_Pin, GPIO_PIN_SET);

  // LED1 pin
  GPIO_InitStruct.Pin   = LED1_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  // CAN CS pin
  GPIO_InitStruct.Pin   = CAN_CS_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CAN_CS_GPIO_Port, &GPIO_InitStruct);

  // CAN1_RESET, CAN2_RESET
  GPIO_InitStruct.Pin   = CAN1_RESET_Pin|CAN2_RESET_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // USART2 pins PA2, PA3
  GPIO_InitStruct.Pin       = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // LED2 pin
  GPIO_InitStruct.Pin   = LED2_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  // RTD_Pin & PB5
  GPIO_InitStruct.Pin  = RTD_Pin|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // PRECHARGE pin
  GPIO_InitStruct.Pin  = PRECHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PRECHARGE_GPIO_Port, &GPIO_InitStruct);

  // SHUTDOWN pin
  GPIO_InitStruct.Pin  = SHUTDOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SHUTDOWN_GPIO_Port, &GPIO_InitStruct);

  // CAN2_CS pin
  GPIO_InitStruct.Pin   = CAN2_CS_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CAN2_CS_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
// No extra user code here
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    // Hang on error
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number where
  *         the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  // Print debug info or break here if needed
}
#endif /* USE_FULL_ASSERT */
