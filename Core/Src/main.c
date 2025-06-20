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
#include <stdbool.h>     // <-- Added this to ensure 'bool' is recognized
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "CANSPI.h"
#include "constants.h"
#include "mcp2515.h"

/* options:
 * DRIVE : drive the car
 * UNWELD_AIRS : flicker both airs and precharge relay one by one
 * CALIBRATE_PEDALS : calibrate pedals. put apps1_as_percent, apps1Value, apps2_as_percent, apps_plausible, readsPer100ms, etc.
 * CAN_TEST : test can :skull:
 * VCUMODE_DEBUG : idfk you choose
 */
#define VCUMODE DRIVE

/**
 * one button RTD mode; press just the RTD button to both precharge and RTD when ready
 */
#define ONE_BUTTON_RTD_MODE_ENABLE 0

//set this to 1 to override RTD and immediately send torque requests. NEVER DO THIS.
uint8_t rtdoverride = 0;

//--- BEGIN WAV PLAYBACK DEFINES ---//
#include "startup_sound.h"  // Contains 'startup_sound' array and 'startup_sound_len'

// Typical WAV header size
#define WAV_HEADER_SIZE         44
// Number of PCM bytes = total - header
#define WAV_DATA_SIZE           (startup_sound_len - WAV_HEADER_SIZE)
// total halfwords in the PCM data (16-bit)
#define TOTAL_HALFWORDS         (WAV_DATA_SIZE / 2)
// We chunk the wave in up to 30000 halfwords transmissions to avoid the 65535 limit:
#define CHUNK_SIZE_HALFWORDS    30000
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t inverterActive;
	uint16_t packVoltage;
	uint16_t packCurrent;
// Add more fields as needed
} BMSDiagnostics;  // The type alias is BMSDiagnostics

typedef struct {
	uint16_t motorRpm;
	uint16_t inverterDCVolts;
	float carSpeed;
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

/* **** CAN MESSAGES **** */
//message that only stores torque requests. ID 0x0C0
uCAN_MSG torqueRequestMessage;
//message that only stores received data. can be whatever
uCAN_MSG rxMessage;
//DIAGNOSTIC message. ID 0x500; not received by anything
uCAN_MSG diagMessage;
//message to orion bms to turn on fans. NOT EVER USED
uCAN_MSG fanMessage;

#if VCUMODE == CALIBRATE_PEDALS
/* **** APPS READS PER SECOND COUNTERS, FOR DIAGNOSTIC PURPOSES **** */
//counter for APPS reads per 100ms. gets reset every 100ms
uint16_t readsPer100msCounter = 0;
//actual APPS reads per 100ms. not really sure why this also exists in addition to above variable
uint16_t readsPer100ms = 0;
//current tick, update whenever u want a value. only used in calculating APPS reads every 100ms. for debug purposes
uint32_t currtick = 0;

/* **** STRUCTS TO STORE THE MAX AND MIN FOR PEDAL ANALOG READS. FOR DEBUG AND CALIBRATION PURPOSES. ONLY USED IN CALIBRATE_PEDALS **** */
//definition of a pedalbounds struct type
typedef struct {
	uint32_t min;
	uint32_t max;
} PedalBounds;
//struct to store max and min of APPS1
PedalBounds APPS1Bounds;
//struct to store max and min of APPS2
PedalBounds APPS2Bounds;
//struct to store max and min of the BSE
PedalBounds BSEBounds;
#endif

/* **** GLOBAL VARIABLES TO READ AND STORE RAW ADC COUNTS OF PEDAL SENSORS **** */
//buffer array that stores the raw analog reads of the pedals. not to be edited by user, only by the HAL when conducting a DMA read
uint32_t ADC_Reads[ADC_BUFFER];
//raw analog counts of apps1
volatile uint32_t apps1Value = 0;
//raw analog counts of apps2
volatile uint32_t apps2Value = 0;
//raw analog counts of the BSE
volatile uint32_t bseValue = 0;
//callback variable to tell main method that a DMA transfer has finished. no longer used to call various methods in the main methods
uint8_t dma_read_complete = 1;
//time since the last DMA read
uint32_t millis_since_dma_read = 0;

/* **** GLOBAL BUFFERS FOR EACH APPS. USED TO COMPUTE MEDIAN FILTER **** */
//raw analog counts buffer of apps1
uint32_t apps1Buffer[ADC_READ_BUFFER] = { 0 };
//raw analog counts buffer of apps2
uint32_t apps2Buffer[ADC_READ_BUFFER] = { 0 };
//raw analog counts buffer of the BSE
uint32_t bseBuffer[ADC_READ_BUFFER] = { 0 };
//where in the pedal filtering buffer (which moves circularly) the next read should go
uint8_t adcBufferIndex = 0;

/* **** VARIABLES TO STORE TORQUE REQUESTS **** */
//requested torque. value here is still in progress to be sent
float requestedTorque = 0;
//previous final torque request. i.e. what was in finalTorqueRequest one CAN frame ago
float lastRequestedTorque = 0;
//the torque request that actually gets sent to the inverter
float finalTorqueRequest = 0;
//lockout variable. 0 if not yet requesting torque (booting up, booting down), 1 if requesting torque (normal drive mode)
uint8_t beginTorqueRequests = false;

/* **** VARIABLES TO STORE PLAUSIBILITY STATUS OF VARIOUS CHECKS; APPS plausibility (T.4.2), BSE plausibility (T.4.3), Crosscheck (EV.4.7)**** */
//APPS plausibility status. does NOT directly mean that torque requests are 0. is 0 if pedal travel % deviation is over APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE
uint16_t apps_plausible = true;
//the time that the APPS last went implausible. if the difference between this and HAL_GetTick() is less
//than APPS_IMPLAUSIBILITY_TIMEOUT_MILLIS, then torque requests will be 0.
uint32_t millis_since_apps_implausible;
// BSE plausibility status. does NOT directly mean that torque requests are 0. is 0 if BSE out of range
uint16_t bse_plausible = true;
//the time that the BSE last went implausible. if the difference between this and HAL_GetTick() is less
//than BSE_IMPLAUSIBILITY_TIMEOUT_MILLIS, then torque requests will be 0.
uint32_t millis_since_bse_implausible;
// Cross-check plausibility. if
uint16_t cross_check_plausible = true;

/* **** VARIABLES TO STORE PEDAL TRAVELS AS A PERCENTAGE, USED TO CALCULATE PLAUSIBILITY CHECKS **** */
//apps 1 pedal travel represented as a %
float apps1_as_percent = 0;
//apps 2 pedal travel represented as a %
float apps2_as_percent = 0;
//brake pedal travel represented as a %
float bse_as_percent = 0;

/* **** GLOBAL VARIABLES THAT STORE VARIOUS STARTUP LOGIC PARAMETERS **** */
//if car is ready to drive; precharged, driver consented, SDC closed, etc.
uint8_t readyToDrive = false;
//temporary variable that is 1 if just waiting for a lonng enough button press on the RTD button
uint8_t rtdState = false;
//duration that the RTD button has been pressed
uint32_t RTDButtonPressedDuration;
//the time, in ms, that the RTD button last got pressed
uint32_t millis_RTD;
//similar to rtdState, temporary variable that is 1 if just waiting for a lonng enough button press on
//precharge button (ONE_BUTTON_RTD_MODE_ENABLE == 0) or RTD button (ONE_BUTTON_RTD_MODE_ENABLE == 1)
uint8_t prechargeState = false;
//if precharge has been finished
uint8_t prechargeFinished = false;
//the time, in ms, that the precharge (ONE_BUTTON_RTD_MODE_ENABLE == 0) or RTD (ONE_BUTTON_RTD_MODE_ENABLE == 1) button last got pressed
uint32_t millis_precharge;
//current state of the precharge button
volatile GPIO_PinState prechargeButtonState;
//current state of the RTD button
volatile GPIO_PinState RTDButtonState;

/* **** DIAGNOSTIC STRUCTURES **** */
//diagnostics coming from BMS
BMSDiagnostics bms_diagnostics;
//diagnostics coming from inverter
InverterDiagnostics inverter_diagnostics;

// WAV Playback Variables
static uint32_t wavPos = 0; // global wave position index
static const uint16_t *wavePCM = NULL; // pointer to PCM data in Flash
static uint32_t halfwordCount = 0; // total halfwords
static uint8_t waveFinished = 0; // flag if wave is finished

#define BMS_TYPE ORION_BMS
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2S2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE BEGIN PFP */
void updateBMSDiagnostics(void);
void readFromCAN(void);
void updateRpm(void);
void calculateTorqueRequest(void);
void checkAPPSPlausibility(void);
void checkBSEPlausibility(void);
void checkCrossCheck(void);
void checkReadyToDrive(void);
void sendTorqueCommand(void);
void sendPrechargeRequest(void);
uint8_t prechargeSequence(void);
uint8_t checkShutdown(void);
void PlayStartupSoundOnce(void);
void updateInverterVolts(void);
void lookForRTD(void);
void AIRUnweldHelper(void);

void calibratePedalsMain(void);

/**
 * debug variable for pin state, can write to this variable wherever but be careful
 * you dont use it multiple times
 */
uint8_t pinState;
//no fucking idea what this is for, probably some debug variable
uint32_t bms_inverter_delta;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * send diagnostic message
 *
 * documented in CAN protocol here
 *
 * https://bruinformularacing.notion.site/Mk-X-CAN-Protocol-2a6f5a22087347e2a02a4793251fd920?p=1ec9c37d88d98099827feb0cf37ad6e8&pm=s
 */
void sendDiagMsg(void) {

	char msg0 = ((int) (inverter_diagnostics.carSpeed * 100)) & 0xff;
	char msg1 = (((int) (inverter_diagnostics.carSpeed * 100)) >> 8) & 0xff;

	diagMessage.frame.data0 = msg0; //feedback speed
	diagMessage.frame.data1 = msg1;

	int torqueValue = (int) (finalTorqueRequest * 10); // Convert to integer, multiply by 10

	// Break the torqueValue into two bytes (little-endian)
	msg0 = torqueValue & 0xFF;
	msg1 = (torqueValue >> 8) & 0xFF;
	diagMessage.frame.data2 = msg0; // torque request
	diagMessage.frame.data3 = msg1;
	diagMessage.frame.data4 = (int) (apps1_as_percent); //apps1%

	diagMessage.frame.data5 = (int) (apps2_as_percent);
	diagMessage.frame.data6 = bse_as_percent;

	diagMessage.frame.data7 = HAL_GPIO_ReadPin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin)
			<< 1;
	diagMessage.frame.data7 = (diagMessage.frame.data7 | readyToDrive) << 1;
	diagMessage.frame.data7 = (diagMessage.frame.data7 | readyToDrive) << 1;
	diagMessage.frame.data7 = diagMessage.frame.data7 << 3; //will set accy relay states as i go
	diagMessage.frame.data7 = (diagMessage.frame.data7 | cross_check_plausible)
			<< 1;

	CANSPI_Transmit(&diagMessage);
}

/**
 * ngl bro idk why this exists i didnt write it
 */
int compare_uint32_t(const void *a, const void *b) {
	return (*(uint32_t*) a - *(uint32_t*) b);
}

/**
 * @brief median filter for APPS reading
 */
uint32_t median_uint32_t(uint32_t *buffer, uint8_t size) {
	uint32_t temp[ADC_READ_BUFFER];
	memcpy(temp, buffer, size * sizeof(uint32_t));
	qsort(temp, size, sizeof(uint32_t), compare_uint32_t);
	return temp[size / 2];
}

/**
 * @brief  Check the shutdown pin; if high, set torque to 0 and block forever.
 */
uint8_t checkShutdown() {
	uint8_t pinState = HAL_GPIO_ReadPin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin);

	if (pinState == GPIO_PIN_RESET) {
		//disable interrupts so there's no distractions
		HAL_ADC_Stop_DMA(&hadc1);

		// Assert that requested torque is 0 and that motor should not spin
		requestedTorque = 0;
		sendTorqueCommand();

		//toggle airs off. first disconnect hot end for safety, then cascade with the rest
		//5ms delays to avoid any weird timing things

		// Can keep this in as toggling AIRs will not do anything w/ Accumulator disconnected!
		HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin, GPIO_PIN_RESET); //turn off air+
		HAL_Delay(5);
		HAL_GPIO_WritePin(AIR_N_CTRL_GPIO_Port, AIR_N_CTRL_Pin, GPIO_PIN_RESET); //turn off air-
		HAL_Delay(5);
		HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin,
				GPIO_PIN_RESET); //turn off precharge relay
		HAL_Delay(5);

		//disable inverter
		for (int i = 0; i < 3; i++) {
			torqueRequestMessage.frame.data0 = 0;
			torqueRequestMessage.frame.data1 = 0;
			torqueRequestMessage.frame.data2 = 0;
			torqueRequestMessage.frame.data3 = 0;
			torqueRequestMessage.frame.data4 = 0;
			torqueRequestMessage.frame.data5 = 0;
			torqueRequestMessage.frame.data6 = 0;
			torqueRequestMessage.frame.data7 = 0;
			CANSPI_Transmit(&torqueRequestMessage);
		}

		//send diagnostic message. reset SDC, AIRS, Precharge, RTD, looking for RTD bits
		diagMessage.frame.data7 = diagMessage.frame.data7 & 0b00000110;
		sendDiagMsg();

		//reset all relevant variables for re-energization
		readyToDrive = false;
		prechargeState = false;
		rtdState = false;
		prechargeFinished = false;

		RTDButtonPressedDuration = 0;

		diagMessage.frame.data7 = diagMessage.frame.data7 & 0b00000110;
		sendDiagMsg();

		HAL_Delay(5000);

		HAL_ADC_Start_DMA(&hadc1, ADC_Reads, ADC_BUFFER);
		return false;
		//		lookForRTD();
	}
	//just set the SDC bit, don't bother with others
	diagMessage.frame.data7 = diagMessage.frame.data7 | 0b10000000;
	sendDiagMsg();
	return true;
}

/**
 // * @brief Update Inverter RPM reading from the last received CAN message.
 */
void updateRpm() {
	inverter_diagnostics.motorRpm = (float) (rxMessage.frame.data2
			| (rxMessage.frame.data3 << 8));
	inverter_diagnostics.carSpeed = (float) (inverter_diagnostics.motorRpm)
			* RPM_TO_CARSPEED_CONVFACTOR;
}

/**
 * @brief Read relevant data from incoming CAN messages
 */
void readFromCAN() {
	if (rxMessage.frame.id == RPM_READ_ID) {
		updateRpm();
	}

	if (rxMessage.frame.id == INVERTER_VOLTAGE_READ_ID) {
		updateInverterVolts();
	}

	if (rxMessage.frame.id == BMS_DIAGNOSTICS_ID) {
		updateBMSDiagnostics();
	}
}

/**
 * @brief Parse BMS diagnostics from a received CAN message
 *
 * yea thats pretty much it... kinda braindead
 */
void updateBMSDiagnostics(void) {
	// Pack_Current (signed 16-bit at bit 8, factor 0.1)

	uint16_t pack_current_raw = (int16_t) ((rxMessage.frame.data0 << 8)
			| rxMessage.frame.data1);  // Little-endian
	float pack_current = pack_current_raw * 0.1f;

	// Pack_Inst_Voltage (unsigned 16-bit at bit 24, factor 0.1)
	uint16_t pack_voltage_raw = (rxMessage.frame.data2 << 8)
			| rxMessage.frame.data3;
	float pack_voltage = pack_voltage_raw * 0.1f;

	// Is_Ready_State (bit 54)
	bool is_ready = (rxMessage.frame.data6) & 0x01;

	bms_diagnostics.inverterActive = is_ready ? 1 : 0;
	bms_diagnostics.packCurrent = (int) pack_current;
	bms_diagnostics.packVoltage = (int) pack_voltage;
}

/**
 * @brief Parse inverter diagnostics from a received CAN message
 */
void updateInverterVolts(void) {
	// DC volts (signed 16-bit at bit 8, factor 0.1)
	int16_t inverter_dc_volts_raw = (int16_t) ((rxMessage.frame.data1 << 8)
			| rxMessage.frame.data0);  // Little-endian
	float inverter_dc_volts = inverter_dc_volts_raw * 0.1f;

	inverter_diagnostics.inverterDCVolts = (int) inverter_dc_volts;
}

/**
 * @brief ADC Conversion Complete callback. Copies the DMA buffer into apps/bse values.
 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1) {

#if VCUMODE == CALIBRATE_PEDALS
	readsPer100msCounter++;
#endif
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

	//	dma_read_complete = 1;

	millis_since_dma_read = HAL_GetTick();

#if VCUMODE == CALIBRATE_PEDALS
	if (apps1Value > APPS1Bounds.max) APPS1Bounds.max = apps1Value;
	else if (apps1Value < APPS1Bounds.min) APPS1Bounds.min = apps1Value;

	if (apps2Value > APPS2Bounds.max) APPS2Bounds.max = apps2Value;
	else if (apps2Value < APPS2Bounds.min) APPS2Bounds.min = apps2Value;

	if (bseValue > BSEBounds.max) BSEBounds.max = bseValue;
	else if (bseValue < BSEBounds.min) BSEBounds.min = bseValue;
#endif

	if (prechargeFinished && readyToDrive) {
		calculateTorqueRequest();
		checkBSEPlausibility();
		checkAPPSPlausibility();
		checkCrossCheck();

		finalTorqueRequest = requestedTorque;
		lastRequestedTorque = requestedTorque;
	} else {
		finalTorqueRequest = 0;
	}

	dma_read_complete = 1;
}

/**
 * @brief Calculate the requested torque based on APPS and RPM, or regen based on BSE. this one uses actual pedal mapping
 * but was never tested, so keep this commented out or delete it, i (tony) just kept it here because i wanted to keep it in one place
 */

//void calculateTorqueRequest(void)
// {
// 	float apps1_as_percent = ((float)apps1Value-APPS_1_ADC_MIN_VAL)/(APPS_1_ADC_MAX_VAL-APPS_1_ADC_MIN_VAL);
// 	float apps2_as_percent = ((float)apps2Value-APPS_2_ADC_MIN_VAL)/(APPS_2_ADC_MAX_VAL-APPS_2_ADC_MIN_VAL);
// 	float appsValue = ((float)apps1_as_percent + apps2_as_percent)/2;
// 	if(appsValue >= 0){ //apps travel is in range for forward torque
// 		requestedTorque = ((float)(MAX_TORQUE-MIN_TORQUE)) * appsValue + MIN_TORQUE;
//
//		int pedalLowIndx = (int)(appsValue * 100.0f / pedalStepSize);
//		int pedalHighIndx = pedalLowIndx + 1;
//		if (pedalHighIndx >= numPedalSteps) {
//			pedalHighIndx = numPedalSteps - 1;
//		}
//
//		int rpmLowIndx = (int)(inverter_diagnostics.motorRpm / rpmStepSize);
//		int rpmHighIndx = rpmLowIndx + 1;
//		if (rpmHighIndx >= numRpmSteps) {
//			rpmHighIndx = numRpmSteps - 1;
//		}
//
//		float T00 = TORQUE_ARRAY[pedalLowIndx][rpmLowIndx];   // Lower-left
//		float T10 = TORQUE_ARRAY[pedalHighIndx][rpmLowIndx];  // Upper-left
//		float T01 = TORQUE_ARRAY[pedalLowIndx][rpmHighIndx];  // Lower-right
//		float T11 = TORQUE_ARRAY[pedalHighIndx][rpmHighIndx]; // Upper-right
//
//		float pedalBase = (appsValue * 100.0f) - (pedalLowIndx * pedalStepSize);
//		float pedalLerp = pedalBase / pedalStepSize;
//
//		float rpmBase = (float)inverter_diagnostics.motorRpm
//		                - (rpmLowIndx * rpmStepSize);
//		float rpmLerp = rpmBase / rpmStepSize;
//
//		float torqueLow  = T00 + (T01 - T00) * rpmLerp;
//		float torqueHigh = T10 + (T11 - T10) * rpmLerp;
//
//		requestedTorque = torqueLow + (torqueHigh - torqueLow) * pedalLerp;
//	} else {
//		// Regen based on brake pedal
//		float bse_as_percent = ((float)bseValue - BSE_ADC_MIN_VAL)
//				/ (BSE_ADC_MAX_VAL - BSE_ADC_MIN_VAL);
//		requestedTorque = (REGEN_MAX_TORQUE - REGEN_BASELINE_TORQUE)
//				* bse_as_percent + REGEN_BASELINE_TORQUE;
//	}
//}
volatile float appsValue;

/**
 * @brief calculate torque request using one-pedal driving control scheme
 *
 * Three important points; max regenerative torque @ X pedal travel, 0 torque at Y pedal travel, and max torque at Z pedal travel
 * max regenerative torque is at 0 mechanical pedal travel
 * 0 torque is at APPS_INFLECTION_PERCENT (5%, as of 6/16/25 to give drivers a little more control in the acceleration range,
 *     where pedal resolution matters more) mechanical pedal travel
 *  max torque is at 100% mechanical pedal travel
 *
 *  this function linearly interpolates between the three points based on pedal travel percentage.
 */
void calculateTorqueRequest(void) {
	float apps1_as_percent = ((float) apps1Value - APPS_1_ADC_MIN_VAL)
			/ (APPS_1_ADC_MAX_VAL - APPS_1_ADC_MIN_VAL);
	float apps2_as_percent = ((float) apps2Value - APPS_2_ADC_MIN_VAL)
			/ (APPS_2_ADC_MAX_VAL - APPS_2_ADC_MIN_VAL);
	float appsValue = ((float) apps1_as_percent + apps2_as_percent) / 2;
	//	if (!apps_plausible && !cross_check_plausible) {
	//		requestedTorque = 0;
	//	}
	if (appsValue >= APPS_INFLECTION_PERCENT) { //apps travel is in range for forward torque
		requestedTorque = ((float) (MAX_TORQUE - MIN_TORQUE))
				* (appsValue - APPS_INFLECTION_PERCENT);

		if (requestedTorque >= MAX_TORQUE) {
			requestedTorque = MAX_TORQUE;
		}
	} else { //apps travel is in range for reverse torque
		if (inverter_diagnostics.carSpeed < 5.0f && VCUMODE != CALIBRATE_PEDALS) {
			requestedTorque = 0;
		} else {
			//			float bse_as_percent = ((float)bseValue-BSE_ADC_MIN_VAL)/(BSE_ADC_MAX_VAL-BSE_ADC_MIN_VAL);
			requestedTorque = (REGEN_MAX_TORQUE - REGEN_BASELINE_TORQUE)
					* ((APPS_INFLECTION_PERCENT - appsValue)
							/ APPS_INFLECTION_PERCENT);
		}
	}
}

/**
 * @brief Check plausibility of APPS sensors.
 * sorry for the shitty ass variable naming, paininmyass is just the % difference in travel between the pedals
 *
 * some redundancy built in; for some reason it really didnt like to just be stable and woudl randomly jump values
 * from the expected 0 to like -70nm, even with the median filter, so instead of adding extra latency to every input
 * i just reject everythign that doesn't fit the bill and use last normal ass value
 */

float pedalTravelDiffPercent = 0.0;
void checkAPPSPlausibility(void) {
	apps1_as_percent = ((float) apps1Value - APPS_1_ADC_MIN_VAL)
			/ (APPS_1_ADC_MAX_VAL - APPS_1_ADC_MIN_VAL) * 100.0f;
	apps2_as_percent = ((float) apps2Value - APPS_2_ADC_MIN_VAL)
			/ (APPS_2_ADC_MAX_VAL - APPS_2_ADC_MIN_VAL) * 100.0f;

	pedalTravelDiffPercent = fabsf(fabsf(apps1_as_percent) - fabsf(apps2_as_percent));
	if (pedalTravelDiffPercent > APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE) {
		if (apps_plausible)
			millis_since_apps_implausible = HAL_GetTick(); //only grab the tick if it switched
		apps_plausible = 0;
		if (!apps_plausible
				&& (HAL_GetTick() - millis_since_apps_implausible
						> APPS_IMPLAUSIBILITY_TIMEOUT_MILLIS)) {
			requestedTorque = 0;
		}
	}
	if (apps1_as_percent < -10.0 || apps2_as_percent < -10.0) {
		apps_plausible = 0;
		requestedTorque = 0;
	}
	if (pedalTravelDiffPercent > APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE) {
		requestedTorque = 0;
	}

	else if (pedalTravelDiffPercent < APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE) {
		apps_plausible = 1;
	}
}

/**
 * @brief Check plausibility of APPS sensors.
 */
void checkBSEPlausibility(void) {
	bse_as_percent = ((float) bseValue - BSE_ADC_MIN_VAL)
			/ (BSE_ADC_MAX_VAL - BSE_ADC_MIN_VAL) * 100.0f;

	if (bse_plausible && (bse_as_percent > 100.0f || bse_as_percent < 0.0f)) {
		millis_since_bse_implausible = HAL_GetTick();
		bse_plausible = 0;
	}

	else if (!bse_plausible
			&& (HAL_GetTick() - millis_since_bse_implausible
					< BSE_IMPLAUSIBILITY_TIMEOUT_MILLIS)) {
		requestedTorque = 0;
	} else {
		bse_plausible = 1;
	}
}

/**
 * @brief Check rank-check between APPS and brake pedal.
 */
void checkCrossCheck(void) {
	// tryna soften brake ADC chatter
	static uint16_t bseFilt = 0;
	bseFilt = (bseFilt + bseValue) >> 1; // 2-sample moving-average filter

	float apps1p = ((float) apps1Value - APPS_1_ADC_MIN_VAL)
			/ (APPS_1_ADC_MAX_VAL - APPS_1_ADC_MIN_VAL) * 100.0f;
	float apps2p = ((float) apps2Value - APPS_2_ADC_MIN_VAL)
			/ (APPS_2_ADC_MAX_VAL - APPS_2_ADC_MIN_VAL) * 100.0f;
	float apps_as_percent = (apps1p + apps2p) / 2.0f;

	// trip when Brake ON + APPS > 25 % (ev.4.7.1)
	if (apps_as_percent > CROSS_CHECK_IMPLAUSIBILITY_APPS_PERCENT && // >25 %
			bseFilt > BRAKE_ACTIVATED_ADC_VAL) // brake
					{
		cross_check_plausible = 0;  // flag fault
		requestedTorque = 0;  // torque --> 0 immediately
	}

	// stay latched until APPS < 5 % (ev.4.7.2b)
	if (!cross_check_plausible) {
		if (apps_as_percent < CROSS_CHECK_RESTORATION_APPS_PERCENT)  // <5 %
				{
			cross_check_plausible = 1;  // fault cleared
		} else {
			requestedTorque = 0; // keep torque at zero
		}
	}
}

/**
 * @brief Send the torque command message over CAN. also provide final checks to ensure no stupid torque requests
 */
void sendTorqueCommand(void) {
	// Test later for plausibility & crosscheck

	if (finalTorqueRequest >= MAX_TORQUE) {
		finalTorqueRequest = MAX_TORQUE;
	}

	if (finalTorqueRequest <= REGEN_MAX_TORQUE) {
		finalTorqueRequest = REGEN_MAX_TORQUE;
	}

	int torqueValue = (int) (finalTorqueRequest * 10); // Convert to integer, multiply by 10

	// Break the torqueValue into two bytes (little-endian)
	uint8_t msg0 = (uint8_t) (torqueValue & 0xFF);
	uint8_t msg1 = (uint8_t) ((torqueValue >> 8) & 0xFF);

	torqueRequestMessage.frame.data0 = msg0; //torque request
	torqueRequestMessage.frame.data1 = msg1;
	torqueRequestMessage.frame.data2 = 0; // speed request (only maters in speed mode)
	torqueRequestMessage.frame.data3 = 0;
	torqueRequestMessage.frame.data4 = 0; //direction

	//lockout
	if (beginTorqueRequests) {
		torqueRequestMessage.frame.data5 = 1;
	} else {
		torqueRequestMessage.frame.data5 = 0;
	}

	torqueRequestMessage.frame.data6 = 0;
	torqueRequestMessage.frame.data7 = 0;

	CANSPI_Transmit(&torqueRequestMessage);
}

#if VCUMODE == CALIBRATE_PEDALS
/**
 * DEBUG USE ONLY; WILL ONLY BE CALLED WHEN IN VCUMODE CALIBRATE_PEDALS. 0x5XX is VCU debug, 0x5C0 is fake torque messages.
 *
 * should probably just use #if directive shits to change the ID isntead of everything lowkey
 */
void sendDebugTorqueCommand(void) {

	if (requestedTorque >= MAX_TORQUE) {
		requestedTorque = MAX_TORQUE;
	}

	int torqueValue = (int) (requestedTorque * 10); // Convert to integer, multiply by 10

	// Break the torqueValue into two bytes (little-endian)
	uint8_t msg0 = (uint8_t)(torqueValue & 0xFF);
	uint8_t msg1 = (uint8_t)((torqueValue >> 8) & 0xFF);



	torqueRequestMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	torqueRequestMessage.frame.id     = 0x5C0;
	torqueRequestMessage.frame.dlc    = 8;

	torqueRequestMessage.frame.data0 = msg0; //torque request
	torqueRequestMessage.frame.data1 = msg1;
	torqueRequestMessage.frame.data2 = 0; // speed request (only maters in speed mode)
	torqueRequestMessage.frame.data3 = 0;
	torqueRequestMessage.frame.data4 = 0; //direction

	//lockout
	if(beginTorqueRequests){
		torqueRequestMessage.frame.data5 = 1;
	}else{
		torqueRequestMessage.frame.data5 = 0;
	}

	torqueRequestMessage.frame.data6 = 0;
	torqueRequestMessage.frame.data7 = 0;

	CANSPI_Transmit(&torqueRequestMessage);
}
#endif

/**
 * 6/16/25 NOT USED
 *
 * supposed to send a command over CAN to the orion BMS to start the fans, never got it working
 */
void sendFanCommand(void) {
	fanMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	fanMessage.frame.id = 0x501;
	fanMessage.frame.dlc = 2;
	if (readyToDrive) {
		fanMessage.frame.data0 = 0xFF;
		fanMessage.frame.data1 = 0xFF;
	} else {
		fanMessage.frame.data0 = 0x00;
		fanMessage.frame.data1 = 0x00;
	}

	CANSPI_Transmit(&fanMessage);
}
/**
 * @brief Check if the driver has pressed the brake pedal and the RTD pin is set.
 */
void checkReadyToDrive(void) {
#if ONE_BUTTON_RTD_MODE_ENABLE == 0
	RTDButtonState = HAL_GPIO_ReadPin(RTD_BTN_GPIO_Port, RTD_BTN_Pin);
	RTDButtonPressedDuration = HAL_GetTick() - millis_RTD;

//	if (RTDButtonState == GPIO_PIN_SET && bseValue > BRAKE_ACTIVATED_ADC_VAL && bms_diagnostics.inverterActive &&!rtdState) {
//		rtdState = true;
//		millis_RTD = HAL_GetTick();
//	}
//	else if (RTDButtonState == GPIO_PIN_RESET || bseValue < BRAKE_ACTIVATED_ADC_VAL || !bms_diagnostics.inverterActive ){
//		rtdState = false;
//	}

	// We removed checks for seeing if the inverter is active, just kept normal checks for RTD Button & BSE Value
	if (RTDButtonState == GPIO_PIN_SET && bseValue > BRAKE_ACTIVATED_ADC_VAL && !rtdState) {
		rtdState = true;
		millis_RTD = HAL_GetTick();
	} else if (RTDButtonState == GPIO_PIN_RESET || bseValue < BRAKE_ACTIVATED_ADC_VAL) {
		rtdState = false;
	}
	else if(RTDButtonPressedDuration >= RTD_BUTTON_PRESS_MILLIS){
		readyToDrive = true;
	}

// DOES NOT GO HERE FOR OUR MODE!!!! Since ONE_BUTTON_RTD_MODE_ENABLE is 0)
#else

	if (!prechargeFinished
			|| bms_diagnostics.packVoltage
					- inverter_diagnostics.inverterDCVolts >= 5) //precharge not finished yet
		return;

	//if precharge finished, wait a tiny little bit to make sure brake isn't just fake pressed and then RTD
	if (HAL_GPIO_ReadPin(RTD_BTN_GPIO_Port, RTD_BTN_Pin) == GPIO_PIN_SET
			&& bseValue > BRAKE_ACTIVATED_ADC_VAL
			&& bms_diagnostics.inverterActive && prechargeFinished
			&& !rtdState) {
		rtdState = true;
		millis_RTD = HAL_GetTick();
	} else if (HAL_GPIO_ReadPin(RTD_BTN_GPIO_Port, RTD_BTN_Pin)
			== GPIO_PIN_RESET || bseValue < BRAKE_ACTIVATED_ADC_VAL
			|| !bms_diagnostics.inverterActive) {
		rtdState = false;
		RTDButtonPressedDuration = 0;
	}
	if (rtdState) {		// button still acknowledged
		RTDButtonPressedDuration = HAL_GetTick() - millis_RTD;
		if (RTDButtonPressedDuration >= RTD_BUTTON_PRESS_MILLIS) {
			readyToDrive = true;	// ready only if still held
		}
	} else {
		RTDButtonPressedDuration = 0;	// reset after release
	}
#endif
}

/**
 * @brief If a hardware pin requests pre-charge, triggers pre-charge sequence
 */

void sendPrechargeRequest(void) {

	while (!prechargeFinished) {
		//			checkAPPSPlausibility();
		//			checkCrossCheck();
		sendDiagMsg();
		if (CANSPI_Receive(&rxMessage)) {
			readFromCAN();
		}


#if ONE_BUTTON_RTD_MODE_ENABLE == 0
		prechargeButtonState = HAL_GPIO_ReadPin(PRECHARGE_BTN_GPIO_Port, PRECHARGE_BTN_Pin);

#if BMS_TYPE == ORION_BMS
		if(prechargeButtonState == GPIO_PIN_SET && !prechargeState){
			prechargeState = true;
			millis_precharge = HAL_GetTick();
		}
		else if (prechargeButtonState == GPIO_PIN_RESET) {
			prechargeState = false;
		}
		else if (HAL_GetTick()-millis_precharge >= PRECHARGE_BUTTON_PRESS_MILLIS) {
			HAL_ADC_Stop_DMA(&hadc1);
			prechargeFinished = prechargeSequence();
			HAL_ADC_Start_DMA(&hadc1, ADC_Reads, ADC_BUFFER);
		}

#elif BMS_TYPE == CUSTOM_BMS
		while (1) {}
#endif


#else
		prechargeButtonState = HAL_GPIO_ReadPin(RTD_BTN_GPIO_Port, RTD_BTN_Pin);

		/**
		 * wait for button to be pressed AND brakes to be pressed, then disable interrupts (bc i dont need DMA reads while precharging)
		 *
		 */

#if BMS_TYPE == ORION_BMS

		if (!checkShutdown())
			return; //if shutdown circuit opens, stop sending a precharge request and go back to waiting for SDC to close
		if (prechargeButtonState == GPIO_PIN_SET
				&& bseValue > BRAKE_ACTIVATED_ADC_VAL && !prechargeState) {
			prechargeState = true;
			millis_precharge = HAL_GetTick();
		} else if (prechargeButtonState == GPIO_PIN_RESET
				|| bseValue < BRAKE_ACTIVATED_ADC_VAL) {
			prechargeState = false;
		} else if (HAL_GetTick() - millis_precharge
				>= PRECHARGE_BUTTON_PRESS_MILLIS) {
//			__disable_irq();
//			prechargeFinished = prechargeSequence();
//			__enable_irq();
			// SLIME THIS OUT!!!!
			HAL_ADC_Stop_DMA(&hadc1);
			prechargeFinished = prechargeSequence();
			HAL_ADC_Start_DMA(&hadc1, ADC_Reads, ADC_BUFFER);
		}
#elif BMS_TYPE == CUSTOM_BMS
		while (1) {}
#endif
#endif
	} // While Loop end brace
}

/**
 * brief the actual procedure of triggering precharge relays
 * 1) air- closes
 * 2) precharge closes
 * 3) spam bms reads
 * 3.5) wait until voltage is within like 20v of bms packvolts
 * 3.75) timeout at 3s or something like that, prob less
 * 4) close positive air if successful precharge and then  precharge relay after short delay
 * 4.5) if prechg is unsuccessful, open neg air and precharge and error and infinite loop
 *
 * return returns 1 if precharge successful, infinite loop if it doesn't work
 *
 * precharge is done with interrupts OFF (6.16.25)
 */

uint32_t startPrechargeTime = 0;
uint32_t elapsedPrechargeTime = 0;
uint8_t prechargeSequence(void) {
	startPrechargeTime = HAL_GetTick();

	//deep breath before starting lmao
	HAL_Delay(10);

	//send the diagnostic message with updated AIR states
	diagMessage.frame.data7 = diagMessage.frame.data7 | 0b00110000;
	sendDiagMsg();

	// Close negative AIR and precharge relay: STARTS PRECHARGING
	HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin, GPIO_PIN_SET); //steps 1 and 2
	HAL_Delay(3);
	HAL_GPIO_WritePin(AIR_N_CTRL_GPIO_Port, AIR_N_CTRL_Pin, GPIO_PIN_SET); //steps 1 and 2
	HAL_Delay(3);
	HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin, GPIO_PIN_RESET); //steps 1 and 2

	elapsedPrechargeTime = HAL_GetTick() - startPrechargeTime;

	//timeout if precharge has taken longer than PRECHARGE_TIMEOUT_MS or if the SDC opens in between (pulling SHUTDOWN Gpio low)
	while (elapsedPrechargeTime < PRECHARGE_TIMEOUT_MS
			&& HAL_GPIO_ReadPin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin)
					== GPIO_PIN_SET) { //loop for 4

		//get inverter & BMS can data
		if (CANSPI_Receive(&rxMessage)) {
			readFromCAN();
		}

		// Monitor voltages within the if statement
		// Once sufficiently precharged we enter the if statement & stop precharging
		// Return 1; note that prechargedFinished = true

//		if (bms_diagnostics.packVoltage - inverter_diagnostics.inverterDCVolts
//				< PRECHARGE_VOLTAGE_DIFF && bms_diagnostics.packVoltage > 10) {

			HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin,
					GPIO_PIN_SET); //step 4
			HAL_Delay(5);
			HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin,
					GPIO_PIN_RESET);
			prechargeFinished = true;

			bms_inverter_delta = bms_diagnostics.packVoltage - inverter_diagnostics.inverterDCVolts;
			// NOTE: bms_diagnostics.packVoltage - inverter_diagnostics.inverterDCVolts is the "DELTA"

			diagMessage.frame.data7 = diagMessage.frame.data7 | 0b00011000;
			sendDiagMsg();
			return 1;
//		}

		elapsedPrechargeTime = HAL_GetTick() - startPrechargeTime;
		sendDiagMsg();
	}

	checkShutdown(); // In the case we broke out of the loop do to SDC Opening
	// Trigger the "system reset"

	return 0;
}

//-----------------------------------------------
// I2S chunk-based WAV Playback Methods
//-----------------------------------------------
/**
 * @brief Called by HAL when a DMA transmission completes (for one chunk).
 * gang i have no idea how this shit works and i don't feel like figuring it out
 *
 * all i know is that it streams data from flash to i2s word by word
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
}

/**
 * @brief Play the startup sound from Flash exactly once.
 *
 * if you still dont know what this method does dawg idfk what to tell u
 */
void PlayStartupSoundOnce(void) {
	wavePCM = (const uint16_t*) (&startup_sound[WAV_HEADER_SIZE]);
	halfwordCount = TOTAL_HALFWORDS;
	wavPos = 0;
	waveFinished = 0;

	// First chunk
	uint32_t remain = halfwordCount - wavPos;
	uint16_t thisChunk =
			(remain > CHUNK_SIZE_HALFWORDS) ?
					CHUNK_SIZE_HALFWORDS : (uint16_t) remain;
	const uint16_t *chunkPtr = wavePCM + wavPos;
	wavPos += thisChunk;

	HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*) chunkPtr, thisChunk);
}

GPIO_PinState shutdownState;
void lookForRTD(void) {
	if (rtdoverride == 1) {
		beginTorqueRequests = true;
		PlayStartupSoundOnce();
		return;
	}

	diagMessage.frame.data7 = diagMessage.frame.data7 | 1;

	while (!readyToDrive) {

		if (CANSPI_Receive(&rxMessage)) {
			readFromCAN();
		}

		if (dma_read_complete) {
			dma_read_complete = 0;
			millis_since_dma_read = HAL_GetTick();
		}

		// Add check for make sure APPS are 0 travel (?)

		shutdownState = HAL_GPIO_ReadPin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin);
		while (shutdownState == GPIO_PIN_RESET) {
			shutdownState = HAL_GPIO_ReadPin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin);
			HAL_Delay(5);
		}

		sendPrechargeRequest();

		// If the driver is ready to drive, send torque over CAN
		uint8_t prevReadyToDrive = readyToDrive;
		checkReadyToDrive(); // Simple button press check, NON blocking

		if (readyToDrive) {
			// If we just transitioned from not-ready to ready, play sound
			if (!prevReadyToDrive) {
				beginTorqueRequests = true;
				PlayStartupSoundOnce();
				diagMessage.frame.data7 = diagMessage.frame.data7 | 0x10000000;
				diagMessage.frame.data7 = diagMessage.frame.data7 & 0x11111110;

				//send disable message, maybe this'll let lockout go away
				for (int i = 0; i < 3;
						i++) {
					torqueRequestMessage.frame.data0 = 0;
					torqueRequestMessage.frame.data1 = 0;
					torqueRequestMessage.frame.data2 = 0;
					torqueRequestMessage.frame.data3 = 0;
					torqueRequestMessage.frame.data4 = 0;
					torqueRequestMessage.frame.data5 = 0;
					torqueRequestMessage.frame.data6 = 0;
					torqueRequestMessage.frame.data7 = 0;
					CANSPI_Transmit(&torqueRequestMessage);

					HAL_Delay(100);
				}

			} else {
				beginTorqueRequests = false;
			}

		}
		sendDiagMsg();
	}
}

void AIRUnweldHelper(void) {
	uint16_t flicker_delay = 500;
	HAL_ADC_Stop_DMA(&hadc1);

	HAL_ADC_DeInit(&hadc1);

	while (1) {
		HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin,
				GPIO_PIN_SET); //steps 1 and 2
		HAL_Delay(flicker_delay);
		HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin,
				GPIO_PIN_RESET); //steps 1 and 2
		HAL_Delay(2 * flicker_delay);
		HAL_GPIO_WritePin(AIR_N_CTRL_GPIO_Port, AIR_N_CTRL_Pin, GPIO_PIN_SET); //steps 1 and 2
		HAL_Delay(flicker_delay);
		HAL_GPIO_WritePin(AIR_N_CTRL_GPIO_Port, AIR_N_CTRL_Pin, GPIO_PIN_RESET); //steps 1 and 2
		HAL_Delay(2 * flicker_delay);
		HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin, GPIO_PIN_SET); //steps 1 and 2
		HAL_Delay(flicker_delay);
		HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin, GPIO_PIN_RESET); //steps 1 and 2
		HAL_Delay(2 * flicker_delay);
	}
}

#if VCUMODE == CALIBRATE_PEDALS
uint32_t lastCalcReadsPerSecTime = 0;
void calibratePedalsMain(void) {
	//	while (apps1Value == 0 || apps2Value == 0 || bseValue == 0) {
	//		if(dma_read_complete){
	//
	//			dma_read_complete = 0;
	//			millis_since_dma_read = HAL_GetTick();
	//		}
	//	}
	APPS1Bounds.min = 4096;
	APPS1Bounds.max = 0;
	APPS2Bounds.min = 4096;
	APPS2Bounds.max = 0;
	BSEBounds.min = 4096;
	BSEBounds.max = 0;

	lastCalcReadsPerSecTime = HAL_GetTick();
	uint8_t lastdmaread = HAL_GetTick();
	while (1) {
		if(dma_read_complete){
//			__disable_irq(); // SLIME THIS OUT!!!!!
			HAL_ADC_Stop_DMA(&hadc1);
			millis_since_dma_read = HAL_GetTick();
			//
			//			if (apps1Value > APPS1Bounds.max) APPS1Bounds.max = apps1Value;
			//			else if (apps1Value < APPS1Bounds.min) APPS1Bounds.min = apps1Value;
			//
			//			if (apps2Value > APPS2Bounds.max) APPS2Bounds.max = apps2Value;
			//			else if (apps2Value < APPS2Bounds.min) APPS2Bounds.min = apps2Value;
			//
			//			if (bseValue > BSEBounds.max) BSEBounds.max = bseValue;
			//			else if (bseValue < BSEBounds.min) BSEBounds.min = bseValue;
			//
			//			calculateTorqueRequest();
			//			checkBSEPausibility();
			//			checkAPPSPlausibility();
			//			checkCrossCheck();
			//
			//
			//			finalTorqueRequest   = requestedTorque;
			//			lastRequestedTorque  = requestedTorque;
			if (HAL_GetTick() - lastCalcReadsPerSecTime > 100) {
				readsPer100ms = readsPer100msCounter*10; //convert to reads per second
				readsPer100msCounter = 0;
				lastCalcReadsPerSecTime = HAL_GetTick();
			}
			dma_read_complete = 0;
//			__enable_irq();
			HAL_ADC_Start_DMA(&hadc1, ADC_Reads, ADC_BUFFER);
			//			sendDebugTorqueCommand();
			// SLIME THIS OUT!!!
		}


	}
}
#endif

void CANTestHelperMain(void) {
	while (1) {
		torqueRequestMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
		torqueRequestMessage.frame.id = 0b10000000011;
		torqueRequestMessage.frame.dlc = 8;
		torqueRequestMessage.frame.data0 = 0x61;
		torqueRequestMessage.frame.data1 = 0x73;
		torqueRequestMessage.frame.data2 = 0x73;
		torqueRequestMessage.frame.data3 = 0x68;
		torqueRequestMessage.frame.data4 = 0x6F;
		torqueRequestMessage.frame.data5 = 0x6C;
		torqueRequestMessage.frame.data6 = 0x65;
		torqueRequestMessage.frame.data7 = 0x73;
		CANSPI_Transmit(&torqueRequestMessage);

		HAL_Delay(100);

		if (CANSPI_Receive(&rxMessage)) {
			uCAN_MSG orangeMessage = rxMessage;
			uCAN_MSG ppMesage;
			ppMesage.frame.idType = rxMessage.frame.idType;
			ppMesage.frame.id = rxMessage.frame.id;
			ppMesage.frame.dlc = rxMessage.frame.dlc;
			ppMesage.frame.data0 = rxMessage.frame.data0 | 0xAA;
			ppMesage.frame.data1 = rxMessage.frame.data1 | 0xAA;
			ppMesage.frame.data2 = rxMessage.frame.data2 | 0xAA;
			ppMesage.frame.data3 = rxMessage.frame.data3 | 0xAA;
			ppMesage.frame.data4 = rxMessage.frame.data4 | 0xAA;
			ppMesage.frame.data5 = rxMessage.frame.data5 | 0xAA;
			ppMesage.frame.data6 = rxMessage.frame.data6 | 0xAA;
			ppMesage.frame.data7 = rxMessage.frame.data7 | 0xAA;
			uCAN_MSG ppMessage2 = ppMesage;

			CANSPI_Transmit(&ppMesage);

		}
	}
}

void DebugMain(void) {
	while (1) {
		prechargeButtonState = HAL_GPIO_ReadPin(PRECHARGE_BTN_GPIO_Port,
				PRECHARGE_BTN_Pin);
		RTDButtonState = HAL_GPIO_ReadPin(RTD_BTN_GPIO_Port, RTD_BTN_Pin);
		//		GPIO_PinState prechargeButtonState = HAL_GPIO_ReadPin(PRECHARGE_BTN_GPIO_Port, PRECHARGE_BTN_Pin);

		if (dma_read_complete) {
			HAL_ADC_Start_DMA(&hadc1, ADC_Reads, ADC_BUFFER);
			dma_read_complete = 0;
			millis_since_dma_read = HAL_GetTick();
		}

	}

}

//global debug variable
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	// Place for any early variable init

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
  MX_SPI3_Init();
  MX_I2S2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	/** TSSI GPIO handling. do this before everything else, moderately timing critical to prevent light from turning on
	 *
	 * BMSFaultState and IMDFaultState come from the SDCPCB and is normally high (faulted) due to a pull up internal
	 * to the VCU STM32 and an external 100k pullup to 12V on the SDCPCB, which is divided down to a 3V3 signal on the VCU.
	 *
	 * TSSI_LATCH, when pulled high, forces the TSSI green due to a small breakout board. This is rules-mandated behavior
	 * (not in the actual rules, part of a stupid FAQ because FSAE is retarded) Falstad: https://tinyurl.com/ywx84ed5
	 *
	 * This loop should default TSSI_LATCH high if it isn't already (it is), wait for BOTH IMD and BMS faults to clear,
	 * and then finally it will pull TSSI_LATCH low, thereby allowing the SDC to control the TSSI. It will only do this once,
	 * on car startup, because that's the only time that you want to override SDCPCB diagnostic information.
	 *
	 */
	if (VCUMODE == DRIVE) {
		GPIO_PinState BMSFaultState = HAL_GPIO_ReadPin(BMS_FAULT_GPIO_Port,
				BMS_FAULT_Pin);
		GPIO_PinState IMDFaultState = HAL_GPIO_ReadPin(IMD_FAULT_GPIO_Port,
				IMD_FAULT_Pin);

		HAL_GPIO_WritePin(TSSI_LATCH_GPIO_Port, TSSI_LATCH_Pin, GPIO_PIN_SET);
		HAL_Delay(10);
		uint32_t start = HAL_GetTick();
		while ((HAL_GetTick() - start) < 30000) {
			// Need to spoof BMS & IMD Fault State on Shutdown, leave these GPIO Checks in code
			BMSFaultState = HAL_GPIO_ReadPin(BMS_FAULT_GPIO_Port,
					BMS_FAULT_Pin);
			IMDFaultState = HAL_GPIO_ReadPin(IMD_FAULT_GPIO_Port,
					IMD_FAULT_Pin);

			if (BMSFaultState == GPIO_PIN_RESET && IMDFaultState == GPIO_PIN_RESET) {
				break;
			}
			HAL_Delay(10);
		}

		HAL_GPIO_WritePin(TSSI_LATCH_GPIO_Port, TSSI_LATCH_Pin, GPIO_PIN_RESET);
	}


	HAL_TIM_Base_Start(&htim3);

#if VCUMODE == CALIBRATE_PEDALS
	APPS1Bounds.min = 4096;
	APPS1Bounds.max = 0;
	APPS2Bounds.min = 4096;
	APPS2Bounds.max = 0;
	BSEBounds.min = 4096;
	BSEBounds.max = 0;
#endif
	// Initialize the CAN at 500kbps (CANSPI_Initialize sets the MCP2515)

	if (CANSPI_Initialize() != true) {
//		Error_Handler();
	}

	// Initialize some diagnostics values
	//	bms_diagnostics.inverterActive = 1;
	inverter_diagnostics.motorRpm = 1;

	diagMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	diagMessage.frame.id = 0x500;
	diagMessage.frame.dlc = 8;
	diagMessage.frame.data0 = 0x00;
	diagMessage.frame.data1 = 0x00;
	diagMessage.frame.data2 = 0x00;
	diagMessage.frame.data3 = 0x00;
	diagMessage.frame.data4 = 0x00;
	diagMessage.frame.data5 = 0x00;
	diagMessage.frame.data6 = 0x00;
	diagMessage.frame.data7 = 0x00;

	torqueRequestMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	torqueRequestMessage.frame.id = 0x0C0;
	torqueRequestMessage.frame.dlc = 8;

	//start DMA
	HAL_ADC_Start_DMA(&hadc1, ADC_Reads, ADC_BUFFER);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	if (VCUMODE == CALIBRATE_PEDALS) {
		calibratePedalsMain();
	}

	if (VCUMODE == UNWELD_AIRS) {
		AIRUnweldHelper();
	}

	if (VCUMODE == CAN_TEST) {
		CANTestHelperMain();
	}

	if (VCUMODE == DEBUG) {
		DebugMain();
	}

	if (VCUMODE == DRIVE) {
		/* DRIVE LOOP */
		while (1) {

			if (!readyToDrive) {
				lookForRTD();
			}
			//
			// Check for new CAN data
			if (CANSPI_Receive(&rxMessage)) {
				readFromCAN();
			}

			checkShutdown();  // If pin is low, torque->0, block and trigger system reset...

			if (dma_read_complete) {
				if (readyToDrive || rtdoverride == 1) {
					sendTorqueCommand();
					//				sendFanCommand();
				}
				dma_read_complete = 0;
			}

			updateBMSDiagnostics();
			sendDiagMsg();
			// ... do other tasks as needed ...
		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// Should never get here
	/* USER CODE BEGIN 3 */
	// should never reach here
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
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
static void MX_I2S2_Init(void)
{

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
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  htim3.Init.Prescaler = 42;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|PCHG_RLY_CTRL_Pin|AIR_P_CTRL_Pin|AIR_N_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CAN_CS_Pin|CAN1_RESET_Pin|CAN2_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TSSI_LATCH_Pin|GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN2_CS_GPIO_Port, CAN2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_CS_Pin */
  GPIO_InitStruct.Pin = CAN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CAN_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN1_RESET_Pin CAN2_RESET_Pin */
  GPIO_InitStruct.Pin = CAN1_RESET_Pin|CAN2_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
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

  /*Configure GPIO pins : TSSI_LATCH_Pin PB1 */
  GPIO_InitStruct.Pin = TSSI_LATCH_Pin|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RTD_BTN_Pin IMD_FAULT_Pin BMS_FAULT_Pin */
  GPIO_InitStruct.Pin = RTD_BTN_Pin|IMD_FAULT_Pin|BMS_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PCHG_RLY_CTRL_Pin AIR_P_CTRL_Pin AIR_N_CTRL_Pin */
  GPIO_InitStruct.Pin = PCHG_RLY_CTRL_Pin|AIR_P_CTRL_Pin|AIR_N_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PRECHARGE_BTN_Pin */
  GPIO_InitStruct.Pin = PRECHARGE_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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

  /*Configure GPIO pin : BSPD_FAULT_Pin */
  GPIO_InitStruct.Pin = BSPD_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BSPD_FAULT_GPIO_Port, &GPIO_InitStruct);

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
