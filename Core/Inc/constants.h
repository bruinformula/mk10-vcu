/*
 * constants.h
 *
 *  Created on: Jan 28, 2025
 *      Author: nakuljoshi
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

const uint16_t APPS1_RANK = 2;
const uint16_t APPS2_RANK = 2;
const uint16_t BSE_RANK = 3;

const uint16_t APPS_1_ADC_MIN_VAL = 650;
const uint16_t APPS_1_ADC_MAX_VAL = 4096;

const uint16_t APPS_2_ADC_MIN_VAL = 650;
const uint16_t APPS_2_ADC_MAX_VAL = 4096;

//Nm
const float MIN_TORQUE = 0;
const float MAX_TORQUE = 108;

const float REGEN_BASELINE_TORQUE = 0;
const float REGEN_MAX_TORQUE = -30;

const float MAX_RPM = 12000;

const uint16_t BSE_ADC_MIN_VAL = 0;
const uint16_t BSE_ADC_MAX_VAL = 4095;


const float APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE = 10;
const uint16_t APPS_IMPLAUSIBILITY_TIMEOUT_MILLIS = 100;

const uint16_t BRAKE_ACTIVATED_ADC_VAL = 800;
const uint16_t CROSS_CHECK_IMPLAUSIBILITY_APPS_PERCENT = 25;
const uint16_t CROSS_CHECK_RESTORATION_APPS_PERCENT = 5;
const uint16_t CROSS_CHECK_IMPLAUSIBILITY_TIMEOUT_MILLIS = 100;

const uint16_t DMA_READ_TIMEOUT = 20;

const float TORQUE_ARRAY[10][10] = {
 	{4.0, 8.0, 12.0, 17.0, 21.0, 25.0, 29.0, 33.0, 45.0, 100.0},
 	{4.0, 8.0, 12.0, 17.0, 21.0, 25.0, 29.0, 33.0, 45.0, 100.0},
 	{4.0, 8.0, 12.0, 17.0, 21.0, 25.0, 29.0, 33.0, 45.0, 100.0},
 	{4.0, 8.0, 12.0, 17.0, 21.0, 25.0, 29.0, 33.0, 45.0, 100.0},
 	{4.0, 8.0, 12.0, 17.0, 21.0, 25.0, 29.0, 33.0, 45.0, 100.0},
 	{4.0, 8.0, 12.0, 17.0, 21.0, 25.0, 29.0, 33.0, 45.0, 100.0},
 	{4.0, 8.0, 12.0, 17.0, 21.0, 25.0, 29.0, 33.0, 45.0, 100.0},
 	{4.0, 8.0, 12.0, 17.0, 21.0, 25.0, 29.0, 33.0, 45.0, 100.0},
 	{4.0, 8.0, 12.0, 17.0, 21.0, 25.0, 29.0, 33.0, 45.0, 100.0},
 	{4.0, 8.0, 12.0, 17.0, 21.0, 25.0, 29.0, 33.0, 45.0, 100.0},
 };

const int RPM_READ_ID = 0x0A5;
const int INVERTER_VOLTAGE_READ_ID = 0x0A7;
const int BMS_DIAGNOSTICS_ID = 0x6B0;
const int PRECHARGE_REQUEST_ID = 0x000;

const int PRECHARGE_TIMEOUT_MS = 300000;
const int PRECHARGE_VOLTAGE_DIFF = 10; //volts off from the pack voltage that is acceptable to finish precharge


const uint16_t RTD_BUTTON_PRESS_MILLIS = 100;
const uint16_t PRECHARGE_BUTTON_PRESS_MILLIS = 1000;

#define CUSTOM_BMS 0
#define ORION_BMS 1

#define ADC_BUFFER 3
#endif /* INC_CONSTANTS_H_ */
