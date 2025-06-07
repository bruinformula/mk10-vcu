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

#include "CANSPI.h"
#include "constants.h"
#include "mcp2515.h"

/* options:
 * DRIVE : drive the car
 * UNWELD_AIRS : flicker both airs and precharge relay one by one
 * CALIBRATE_PEDALS : calibrate pedals. put apps1_as_percent, apps1Value, apps2_as_percent, apps_plausible, etc.
 * CAN_TEST : test can :skull:
*/
#define VCUMODE CALIBRATE_PEDALS


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
	int inverterActive;
	int packVoltage;
	int packCurrent;
	// Add more fields as needed
} BMSDiagnostics;  // The type alias is BMSDiagnostics

typedef struct {
	int motorRpm;
	int inverterDCVolts;
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
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

// CAN messages
uCAN_MSG txMessage;
uCAN_MSG rxMessage;
uCAN_MSG diagMessage;
uCAN_MSG fanMessage;



// ADC readings
volatile uint32_t apps1Value = 0;
volatile uint32_t apps2Value = 0;
volatile uint32_t bseValue   = 0;

// Torque variables
uint32_t apps1Buffer[ADC_READ_BUFFER] = {0};
uint32_t apps2Buffer[ADC_READ_BUFFER] = {0};
uint32_t bseBuffer[ADC_READ_BUFFER]   = {0};
uint8_t adcBufferIndex = 0;

float requestedTorque;
float lastRequestedTorque = 0;
float finalTorqueRequest;
uint8_t beginTorqueRequests = false;

// APPS plausibility
uint16_t apps_plausible = true;
uint32_t millis_since_apps_implausible;
volatile uint8_t  dma_read_complete = 1;
uint32_t millis_since_dma_read = 0;

// Cross-check plausibility
uint16_t cross_check_plausible = true;

// ADC buffer
uint32_t ADC_Reads[ADC_BUFFER];

// Temp variables for display or logic
float apps1_as_percent = 0;
float apps2_as_percent = 0;
float bse_as_percent = 0;

static float apps1Filt = 0, apps2Filt = 0, bseFilt = 0;


// startup logic global variables
uint8_t readyToDrive = false;
uint8_t rtdState = false;
uint32_t cpockandballs;
uint32_t millis_RTD;
uint8_t prechargeState = false;
uint8_t prechargeFinished = false;
uint32_t millis_precharge;

// Diagnostics structures
BMSDiagnostics      bms_diagnostics;
InverterDiagnostics inverter_diagnostics;

// WAV Playback Variables
static uint32_t wavPos        = 0; // global wave position index
static const uint16_t *wavePCM = NULL; // pointer to PCM data in Flash
static uint32_t halfwordCount  = 0; // total halfwords
static uint8_t waveFinished    = 0; // flag if wave is finished


// customiABILTY shits
const uint8_t BMS_TYPE = ORION_BMS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2S2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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
uint8_t prechargeSequence(void);
void checkShutdown(void);
void PlayStartupSoundOnce(void);
void updateInverterVolts(void);
void lookForRTD(void);
void AIRUnweldHelper(void);

void calibratePedalsMain(void);
uint32_t penispenispenis;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void sendDiagMsg(void) {

	char msg0 = ((int)(inverter_diagnostics.carSpeed * 100) ) & 0xff;
	char msg1 = (((int)(inverter_diagnostics.carSpeed * 100) ) >> 8) & 0xff;

	diagMessage.frame.data0 = msg0; //feedback speed
	diagMessage.frame.data1 = msg1;


	int torqueValue = (int) (requestedTorque * 10); // Convert to integer, multiply by 10

	// Break the torqueValue into two bytes (little-endian)
	msg0 = torqueValue & 0xFF;
	msg1 = (torqueValue >> 8) & 0xFF;
	diagMessage.frame.data2 = msg0; // torque request
	diagMessage.frame.data3 = msg1;
	diagMessage.frame.data4 = (int)(apps1_as_percent); //apps1%


	diagMessage.frame.data5 = (int)(apps2_as_percent);
	diagMessage.frame.data6 = bse_as_percent;


	diagMessage.frame.data7 = HAL_GPIO_ReadPin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin) << 1;
	diagMessage.frame.data7 = (diagMessage.frame.data7 | readyToDrive) << 1;
	diagMessage.frame.data7 = (diagMessage.frame.data7 | readyToDrive) << 1;
	diagMessage.frame.data7 = diagMessage.frame.data7 << 3; //will set accy relay states as i go
	diagMessage.frame.data7 = (diagMessage.frame.data7 | cross_check_plausible) << 1;



	CANSPI_Transmit(&diagMessage);
}

int compare_uint32_t(const void *a, const void *b) {
	return (*(uint32_t*)a - *(uint32_t*)b);
}

uint32_t median_uint32_t(uint32_t *buffer, uint8_t size) {
	uint32_t temp[ADC_READ_BUFFER];
	memcpy(temp, buffer, size * sizeof(uint32_t));
	qsort(temp, size, sizeof(uint32_t), compare_uint32_t);
	return temp[size / 2];
}

/**
 * @brief  Check the shutdown pin; if high, set torque to 0 and block forever.
 */
void checkShutdown(){
	uint8_t pinState = HAL_GPIO_ReadPin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin);
	//	diagMessage.frame.data7 = diagMessage.frame.data7 & 0b00111110;
	sendDiagMsg();
	if (pinState == GPIO_PIN_RESET) {

		requestedTorque = 0;

		sendTorqueCommand();
		txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
		HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin, GPIO_PIN_RESET); //steps 1 and 2
		HAL_Delay(5);
		HAL_GPIO_WritePin(AIR_N_CTRL_GPIO_Port, AIR_N_CTRL_Pin, GPIO_PIN_RESET); //steps 1 and 2
		HAL_Delay(5);
		HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin, GPIO_PIN_RESET); //steps 1 and 2
		HAL_Delay(5);

		readyToDrive = false;

		prechargeState = false;
		rtdState = false;
		prechargeFinished = false;
		cpockandballs = 0;

		for (int i = 0; i < 3; i++) {
			txMessage.frame.id = 0x0C0;
			txMessage.frame.dlc = 8;
			txMessage.frame.data0 = 0;
			txMessage.frame.data1 = 0;
			txMessage.frame.data2 = 0;
			txMessage.frame.data3 = 0;
			txMessage.frame.data4 = 0;
			txMessage.frame.data5 = 0;
			txMessage.frame.data6 = 0;
			txMessage.frame.data7 = 0;
			CANSPI_Transmit(&txMessage);
		}

		diagMessage.frame.data7 = diagMessage.frame.data7 & 0b00000110;
		sendDiagMsg();

		HAL_Delay(5000);

		//		lookForRTD();
	}
}

/**
// * @brief Update Inverter RPM reading from the last received CAN message.
 */
void updateRpm() {
	inverter_diagnostics.motorRpm = (float) (rxMessage.frame.data2 | (rxMessage.frame.data3 << 8));
	inverter_diagnostics.carSpeed = (float)(inverter_diagnostics.motorRpm) * RPM_TO_CARSPEED_CONVFACTOR;
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

	if(rxMessage.frame.id == BMS_DIAGNOSTICS_ID){
		updateBMSDiagnostics();
	}
}

/**
 * @brief Parse BMS diagnostics from a received CAN message
 */
void updateBMSDiagnostics(void) {
	// Pack_Current (signed 16-bit at bit 8, factor 0.1)

	int16_t pack_current_raw = (int16_t)((rxMessage.frame.data0 << 8) | rxMessage.frame.data1);  // Little-endian
	float pack_current = pack_current_raw * 0.1f;

	// Pack_Inst_Voltage (unsigned 16-bit at bit 24, factor 0.1)
	uint16_t pack_voltage_raw = (rxMessage.frame.data2 << 8) | rxMessage.frame.data3;
	float pack_voltage = pack_voltage_raw * 0.1f;

	// Is_Ready_State (bit 54)
	bool is_ready = (rxMessage.frame.data6) & 0x01;

	bms_diagnostics.inverterActive = is_ready ? 1 : 0;
	bms_diagnostics.packCurrent    = (int)pack_current;
	bms_diagnostics.packVoltage    = (int)pack_voltage;
}




/**
 * @brief Parse inverter diagnostics from a received CAN message
 */
void updateInverterVolts(void) {
	// DC volts (signed 16-bit at bit 8, factor 0.1)
	int16_t inverter_dc_volts_raw = (int16_t)((rxMessage.frame.data1 << 8) | rxMessage.frame.data0);  // Little-endian
	float inverter_dc_volts = inverter_dc_volts_raw * 0.1f;

	inverter_diagnostics.inverterDCVolts    = (int)inverter_dc_volts;
}

/**
 * @brief ADC Conversion Complete callback. Copies the DMA buffer into apps/bse values.
 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1) {
//	// Store latest samples into buffers
//	apps1Buffer[adcBufferIndex] = ADC_Reads[APPS1_RANK-1];
//	apps2Buffer[adcBufferIndex] = ADC_Reads[APPS2_RANK-1];
//	bseBuffer[adcBufferIndex]   = ADC_Reads[BSE_RANK-1];
//
//	// Move buffer index circularly
//	adcBufferIndex = (adcBufferIndex + 1) % ADC_READ_BUFFER;
//
//	// Compute and assign median
//	apps1Value = median_uint32_t(apps1Buffer, ADC_READ_BUFFER);
//	apps2Value = median_uint32_t(apps2Buffer, ADC_READ_BUFFER);
//	bseValue   = median_uint32_t(bseBuffer, ADC_READ_BUFFER);
//
//	dma_read_complete = 1;


	/* Grab raw samples */
	apps1Value = ADC_Reads[APPS1_RANK-1];
	apps2Value = ADC_Reads[APPS2_RANK-1];
	bseValue   = ADC_Reads[BSE_RANK-1];

	/* Fast 1-pole IIR (α = 0.25) gives ~2-sample latency */
	apps1Filt += 0.25f * ((float)apps1Value - apps1Filt);
	apps2Filt += 0.25f * ((float)apps2Value - apps2Filt);
	bseFilt   += 0.25f * ((float)bseValue   - bseFilt);
	apps1Value = (uint32_t)apps1Filt;
	apps2Value = (uint32_t)apps2Filt;
	bseValue   = (uint32_t)bseFilt;

	dma_read_complete = 1;
}


/**
 * @brief Calculate the requested torque based on APPS and RPM, or regen based on BSE.
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

/*
 * PEDAL_MAP
 * 0 is braindead
 * 1 is skidpad; looks like x^3 function ish
 * 2 is some weird shit i cooked up for endurance, control at low-med throttle and
 */

#define PEDAL_MAP 0

#define MAP_SIZE 10

typedef struct {
    float throttle;     // x value
    float torque;       // y value
} ThrottleTorquePair;

const static ThrottleTorquePair torqueMap[20] = {
    { 0,  0 },
    { 0.05, 5 },
    {0.10, 10 },
    {0.15, 20 },
    {0.20, 24540 },
    {0.25, 28630 },
    {0.30, 32720 },
    {0.35, 36810 },
    {0.40, 40900 },
    {0.45, 44990 },
    {0.50, 49080 },
    {0.55, 53170 },
    {0.60, 57260 },
    {0.65, 61350 },
	{0.70, 57260 },
	{0.75, 61350 },
	{0.80, 57260 },
	{0.85, 61350 },
	{0.90, 100 },
	{0.95, 120 },
	{1	 , 125}
};

uint16_t throttlePoint0;
uint16_t throttlePoint1;
uint16_t torquePoint0;
uint16_t torquePoint1;
float pedalMapCurrSlope = 0;


void calculateTorqueRequest(void)
{
	float apps1_as_percent = ((float)apps1Value-APPS_1_ADC_MIN_VAL)/(APPS_1_ADC_MAX_VAL-APPS_1_ADC_MIN_VAL);
	float apps2_as_percent = ((float)apps2Value-APPS_2_ADC_MIN_VAL)/(APPS_2_ADC_MAX_VAL-APPS_2_ADC_MIN_VAL);
	float appsValue = ((float)apps1_as_percent + apps2_as_percent)/2;
	if (!apps_plausible && !cross_check_plausible) {
		requestedTorque = 0;
	}
#if	PEDAL_MAP == 0
		if(appsValue >= 0){ //apps travel is in range for forward torque
			requestedTorque = ((float)(MAX_TORQUE-MIN_TORQUE)) * appsValue + MIN_TORQUE;

			if (requestedTorque >= MAX_TORQUE) {
				requestedTorque = MAX_TORQUE;
			}
		} else { //apps travel is in range for reverse torque
			if (inverter_diagnostics.carSpeed < 5.0f) {
				requestedTorque = 0;
			} else {
				float bse_as_percent = ((float)bseValue-BSE_ADC_MIN_VAL)/(BSE_ADC_MAX_VAL-BSE_ADC_MIN_VAL);
				requestedTorque = (REGEN_MAX_TORQUE - REGEN_BASELINE_TORQUE)*bse_as_percent + REGEN_BASELINE_TORQUE;
			}
		}
#elif PEDAL_MAP == 2
		// Search for bounding points
		    for (int i = 0; i < MAP_SIZE - 1; i++) {
		    	throttlePoint0 = torqueMap[i].throttle;
		    	throttlePoint1 = torqueMap[i + 1].throttle;


		        if (inputThrottle >= throttlePoint0 && inputThrottle <= throttlePoint1) {
		        	uint16_t torquePoint0 = torqueMap[i].torque;
					uint16_t torquePoint1 = torqueMap[i + 1].torque;
		            // Interpolate
					pedalMapCurrSlope = (float)(torquePoint1 - torquePoint0) / (throttlePoint1 - throttlePoint0);
		            requestedTorque = torquePoint0 + pedalMapCurrSlope * (inputThrottle - throttlePoint0);
		        }
		    }

		    // Extrapolation below range
		    if (inputThrottle < torqueMap[0].throttle) {
		       requestedTorque = REGEN_MAX_TORQUE;
		    }

		    // Extrapolation above range
		    if (inputThrottle > torqueMap[MAP_SIZE - 1].throttle) {
		        requestedTorque = MAX_TORQUE;
		    }

#endif
}





/**
 * @brief Check plausibility of APPS sensors.
 */
void checkAPPSPlausibility(void) {
	apps1_as_percent = ((float) apps1Value - APPS_1_ADC_MIN_VAL)
							/ (APPS_1_ADC_MAX_VAL - APPS_1_ADC_MIN_VAL) * 100.0f;
	apps2_as_percent = ((float) apps2Value - APPS_2_ADC_MIN_VAL)
							/ (APPS_2_ADC_MAX_VAL - APPS_2_ADC_MIN_VAL) * 100.0f;

	float paininmyass = fabsf(apps1_as_percent - apps2_as_percent);

	if (apps_plausible && fabsf(apps1_as_percent - apps2_as_percent)
			> APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE)
	{
		millis_since_apps_implausible = HAL_GetTick();
		apps_plausible = 0;
		requestedTorque = 0;
	}

	else if (!apps_plausible && (HAL_GetTick() - millis_since_apps_implausible < APPS_IMPLAUSIBILITY_TIMEOUT_MILLIS)) {
		requestedTorque = 0;
	}
	else {
		apps_plausible = 1;
	}
}

/**
 * @brief Check rank-check between APPS and brake pedal.
 */
void checkCrossCheck(void) {

	bse_as_percent = ((float) bseValue - BSE_ADC_MIN_VAL)
							/ (BSE_ADC_MAX_VAL - BSE_ADC_MIN_VAL) * 100.0f;

	float apps1p = ((float) apps1Value - APPS_1_ADC_MIN_VAL)
							/ (APPS_1_ADC_MAX_VAL - APPS_1_ADC_MIN_VAL) * 100.0f;
	float apps2p = ((float) apps2Value - APPS_2_ADC_MIN_VAL)
							/ (APPS_2_ADC_MAX_VAL - APPS_2_ADC_MIN_VAL) * 100.0f;
	float apps_as_percent = (apps1p + apps2p) / 2.0f;

	if (apps_as_percent > CROSS_CHECK_IMPLAUSIBILITY_APPS_PERCENT &&
			bseValue > BRAKE_ACTIVATED_ADC_VAL)
	{
		cross_check_plausible = 0;
		requestedTorque = 0;
	}
	else if (!cross_check_plausible &&
			(apps_as_percent > CROSS_CHECK_RESTORATION_APPS_PERCENT))
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
void sendTorqueCommand(void) {

	if (requestedTorque >= MAX_TORQUE) {
		requestedTorque = MAX_TORQUE;
	}

	if (!apps_plausible && !cross_check_plausible) {
		requestedTorque = 0;
	}

	int torqueValue = (int) (requestedTorque * 10); // Convert to integer, multiply by 10

	// Break the torqueValue into two bytes (little-endian)
	uint8_t msg0 = (uint8_t)(torqueValue & 0xFF);
	uint8_t msg1 = (uint8_t)((torqueValue >> 8) & 0xFF);



	txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	txMessage.frame.id     = 0x0C0;
	txMessage.frame.dlc    = 8;

	txMessage.frame.data0 = msg0; //torque request
	txMessage.frame.data1 = msg1;
	txMessage.frame.data2 = 0; // speed request (only matters in speed mode)
	txMessage.frame.data3 = 0;
	txMessage.frame.data4 = 0; //direction

	//lockout
	if(beginTorqueRequests){
		txMessage.frame.data5 = 1;
	} else {
		txMessage.frame.data5 = 0;
	}

	txMessage.frame.data6 = 0;
	txMessage.frame.data7 = 0;

	CANSPI_Transmit(&txMessage);
}



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

	CANSPI_Transmit(&diagMessage);
}
uint8_t pinState;
/**
 * @brief Check if the driver has pressed the brake pedal and the RTD pin is set.
 */
void checkReadyToDrive(void) {
	pinState = HAL_GPIO_ReadPin(RTD_BTN_GPIO_Port, RTD_BTN_Pin);
	penispenispenis = HAL_GPIO_ReadPin(RTD_BTN_GPIO_Port, RTD_BTN_Pin);
	cpockandballs = HAL_GetTick() -millis_RTD;
	if (pinState == GPIO_PIN_SET && bseValue > BRAKE_ACTIVATED_ADC_VAL && bms_diagnostics.inverterActive &&!rtdState) {
		rtdState = true;
		millis_RTD = HAL_GetTick();
	}
	else if (pinState == GPIO_PIN_RESET || bseValue < BRAKE_ACTIVATED_ADC_VAL || !bms_diagnostics.inverterActive ){
		rtdState = false;
	}
	else if(cpockandballs >= RTD_BUTTON_PRESS_MILLIS){
		readyToDrive = true;
	}
}

/**
 * @brief If a hardware pin requests pre-charge, triggers pre-charge sequence
 */

void sendPrechargeRequest(void){

	while(!prechargeFinished){
		//			checkAPPSPlausibility();
		//			checkCrossCheck();
		sendDiagMsg();
		if (CANSPI_Receive(&rxMessage)) {
			readFromCAN();
		}
		uint8_t pinState = HAL_GPIO_ReadPin(PRECHARGE_BTN_GPIO_Port, PRECHARGE_BTN_Pin);
		if (BMS_TYPE == ORION_BMS) {
			if(pinState == GPIO_PIN_SET && !prechargeState){
				prechargeState = true;
				millis_precharge = HAL_GetTick();
			}
			else if (pinState == GPIO_PIN_RESET){
				prechargeState = false;
			}
			else if(HAL_GetTick()-millis_precharge >= PRECHARGE_BUTTON_PRESS_MILLIS){
				prechargeSequence();
				prechargeFinished = true;
			}
		} else if (BMS_TYPE == CUSTOM_BMS) {
			while (1) {}
		}
	}
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
 */

uint32_t startPrechargeTime = 0;
uint32_t penis = 0;
uint8_t prechargeSequence(void){
	startPrechargeTime = HAL_GetTick();

	HAL_Delay(10);

	diagMessage.frame.data7 = diagMessage.frame.data7 | 0b00110000;
	sendDiagMsg();

	HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin, GPIO_PIN_SET); //steps 1 and 2
	HAL_Delay(3);
	HAL_GPIO_WritePin(AIR_N_CTRL_GPIO_Port, AIR_N_CTRL_Pin, GPIO_PIN_SET); //steps 1 and 2
	HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin, GPIO_PIN_RESET); //steps 1 and 2

	penis = HAL_GetTick() - startPrechargeTime;
	while (penis < PRECHARGE_TIMEOUT_MS && HAL_GPIO_ReadPin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin) == GPIO_PIN_SET) { //loop for 4

		//get inverter & BMS can data
		if (CANSPI_Receive(&rxMessage)) {
			readFromCAN();
		}

		if (bms_diagnostics.packVoltage - inverter_diagnostics.inverterDCVolts < PRECHARGE_VOLTAGE_DIFF) {

			HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin, GPIO_PIN_SET); //step 4
			HAL_Delay(5);
			HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin, GPIO_PIN_RESET);
			prechargeFinished = true;
			penispenispenis = bms_diagnostics.packVoltage - inverter_diagnostics.inverterDCVolts;
			diagMessage.frame.data7 = diagMessage.frame.data7 | 0b00011000;
			sendDiagMsg();
			return 1;
		}

		penis = HAL_GetTick() - startPrechargeTime;
		sendDiagMsg();
	}

	HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(AIR_N_CTRL_GPIO_Port, AIR_N_CTRL_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin, GPIO_PIN_RESET);
	diagMessage.frame.data7 = diagMessage.frame.data7 & 0b11000111;
	sendDiagMsg();
	HAL_Delay(5000);
	prechargeState = false;
	prechargeFinished = false;
	return 0;
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
void PlayStartupSoundOnce(void) {
	wavePCM = (const uint16_t*)(&startup_sound[WAV_HEADER_SIZE]);
	halfwordCount = TOTAL_HALFWORDS;
	wavPos        = 0;
	waveFinished  = 0;

	// First chunk
	uint32_t remain = halfwordCount - wavPos;
	uint16_t thisChunk = (remain > CHUNK_SIZE_HALFWORDS)
	                    		 ? CHUNK_SIZE_HALFWORDS
	                    				 : (uint16_t)remain;
	const uint16_t *chunkPtr = wavePCM + wavPos;
	wavPos += thisChunk;

	HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)chunkPtr, thisChunk);
}

uint8_t rtdoverride = 0;

GPIO_PinState ballsandcock;
void lookForRTD(void) {
	if (rtdoverride == 1) {
		beginTorqueRequests = true;
		PlayStartupSoundOnce();
		return;
	}
	diagMessage.frame.data7 = diagMessage.frame.data7 | 1;
	while(!readyToDrive) {



		if (CANSPI_Receive(&rxMessage)) {
			readFromCAN();
		}

		if(dma_read_complete){
			dma_read_complete = 0;
			millis_since_dma_read = HAL_GetTick();
		}

		//add check for make sure apps are 0 travel
		ballsandcock = HAL_GPIO_ReadPin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin);
		//		while (ballsandcock == GPIO_PIN_RESET) {
		//		  HAL_Delay(1);
		//	  }



		sendPrechargeRequest();

		// If the driver is ready to drive, send torque over CAN
		uint8_t prevReadyToDrive = readyToDrive;
		checkReadyToDrive();


		if (readyToDrive) {
			// If we just transitioned from not-ready to ready, play sound
			if(!prevReadyToDrive){
				beginTorqueRequests = true;
				PlayStartupSoundOnce();
				diagMessage.frame.data7 = diagMessage.frame.data7 | 0x10000000;
				diagMessage.frame.data7 = diagMessage.frame.data7 & 0x11111110;

				//send disable message, maybe this'll let lockout go away
				for (int erectiledysfunction = 0; erectiledysfunction < 3; erectiledysfunction++) {
					txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
					txMessage.frame.id = 0x0C0;
					txMessage.frame.dlc = 8;
					txMessage.frame.data0 = 0;
					txMessage.frame.data1 = 0;
					txMessage.frame.data2 = 0;
					txMessage.frame.data3 = 0;
					txMessage.frame.data4 = 0;
					txMessage.frame.data5 = 0;
					txMessage.frame.data6 = 0;
					txMessage.frame.data7 = 0;
					CANSPI_Transmit(&txMessage);

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
	while(1) {
		HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin, GPIO_PIN_SET); //steps 1 and 2
		HAL_Delay(flicker_delay);
		HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin, GPIO_PIN_RESET); //steps 1 and 2
		HAL_Delay(2*flicker_delay);
		HAL_GPIO_WritePin(AIR_N_CTRL_GPIO_Port, AIR_N_CTRL_Pin, GPIO_PIN_SET); //steps 1 and 2
		HAL_Delay(flicker_delay);
		HAL_GPIO_WritePin(AIR_N_CTRL_GPIO_Port, AIR_N_CTRL_Pin, GPIO_PIN_RESET); //steps 1 and 2
		HAL_Delay(2*flicker_delay);
		HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin, GPIO_PIN_SET); //steps 1 and 2
		HAL_Delay(flicker_delay);
		HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin, GPIO_PIN_RESET); //steps 1 and 2
		HAL_Delay(2*flicker_delay);
	}
}

typedef struct {
	uint32_t min;
	uint32_t max;
} PedalBounds;
PedalBounds APPS1Bounds;
PedalBounds APPS2Bounds;
PedalBounds BSEBounds;

void calibratePedalsMain(void) {
//	while (apps1Value == 0 || apps2Value == 0 || bseValue == 0) {
//		if(dma_read_complete){
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


	while (1) {
		if(dma_read_complete){
			dma_read_complete = 0;
			millis_since_dma_read = HAL_GetTick();
			if (apps1Value > APPS1Bounds.max) APPS1Bounds.max = apps1Value;
			else if (apps1Value < APPS1Bounds.min) APPS1Bounds.min = apps1Value;

			if (apps2Value > APPS2Bounds.max) APPS2Bounds.max = apps2Value;
			else if (apps2Value < APPS2Bounds.min) APPS2Bounds.min = apps2Value;

			if (bseValue > BSEBounds.max) BSEBounds.max = bseValue;
			else if (bseValue < BSEBounds.min) BSEBounds.min = bseValue;
			calculateTorqueRequest();
			checkAPPSPlausibility();
			checkCrossCheck();
		}




	}
}

void CANTestHelperMain(void) {

	txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	txMessage.frame.id = 0b10000000011;
	txMessage.frame.dlc = 8;
	txMessage.frame.data0 = 0x61;
	txMessage.frame.data1 = 0x73;
	txMessage.frame.data2 = 0x73;
	txMessage.frame.data3 = 0x68;
	txMessage.frame.data4 = 0x6F;
	txMessage.frame.data5 = 0x6C;
	txMessage.frame.data6 = 0x65;
	txMessage.frame.data7 = 0x73;
	CANSPI_Transmit(&txMessage);

	HAL_Delay(100);


	if(CANSPI_Receive(&rxMessage))
	{
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
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	// Start TIM4
	//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	//start tim3 for dma reads
	HAL_TIM_Base_Start(&htim3);


	// Initialize the CAN at 500kbps (CANSPI_Initialize sets the MCP2515)
	if (CANSPI_Initialize() != true) {
		Error_Handler();
	}

	// Initialize some diagnostics values
	//	bms_diagnostics.inverterActive = 1;
	inverter_diagnostics.motorRpm   = 1;

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


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#if VCUMODE == CALIBRATE_PEDALS
	calibratePedalsMain();
#elif VCUMODE == UNWELD_AIRS
		AIRUnweldHelper();
#elif VCUMODE == CAN_TEST
		CANTestHelperMain();
#elif VCUMODE == DRIVE
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


			if(dma_read_complete){
				dma_read_complete = 0;
				millis_since_dma_read = HAL_GetTick();

				// Periodically do your torque calculations:
				calculateTorqueRequest();
				checkAPPSPlausibility();
				checkCrossCheck();
				finalTorqueRequest   = requestedTorque;
				lastRequestedTorque  = requestedTorque;



				if (readyToDrive || rtdoverride == 1) {
					sendTorqueCommand();
					//				sendFanCommand();
				}
				sendDiagMsg();
			}


			updateBMSDiagnostics();
			checkShutdown();  // If pin is low, torque->0, block




			// ... do other tasks as needed ...
		}
#endif
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
  hadc1.Init.DMAContinuousRequests = DISABLE;
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


	HAL_ADC_Start_DMA(&hadc1, ADC_Reads, ADC_BUFFER);
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
  htim3.Init.Prescaler = 4;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10001;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3360;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 127;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN2_CS_GPIO_Port, CAN2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RTD_BTN_Pin */
  GPIO_InitStruct.Pin = RTD_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RTD_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PCHG_RLY_CTRL_Pin AIR_P_CTRL_Pin AIR_N_CTRL_Pin */
  GPIO_InitStruct.Pin = PCHG_RLY_CTRL_Pin|AIR_P_CTRL_Pin|AIR_N_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

  /*Configure GPIO pins : PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
