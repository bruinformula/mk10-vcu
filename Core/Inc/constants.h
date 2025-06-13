/*
 * constants.h
 *
 *  Created on: Jan 28, 2025
 *      Author: nakuljoshi
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

const uint16_t APPS1_RANK = 2;
const uint16_t APPS2_RANK = 3;
const uint16_t BSE_RANK = 1;

//const uint16_t APPS_1_ADC_MAX_VAL = 3116;
//const uint16_t APPS_1_ADC_MIN_VAL = 312;
//
//const uint16_t APPS_2_ADC_MAX_VAL = 4095;
//const uint16_t APPS_2_ADC_MIN_VAL = 409;

//const uint16_t APPS_1_ADC_MAX_VAL = 3108;
//const uint16_t APPS_1_ADC_MIN_VAL = 979;
//
//const uint16_t APPS_2_ADC_MAX_VAL = 4095;
//const uint16_t APPS_2_ADC_MIN_VAL = 1657;
const uint16_t APPS_1_ADC_MAX_VAL = 2600;
const uint16_t APPS_1_ADC_MIN_VAL = 680;

const uint16_t APPS_2_ADC_MAX_VAL = 3812;
const uint16_t APPS_2_ADC_MIN_VAL = 1217;


const float APPS_INFLECTION_PERCENT = 0.1;

//Nm
const float MIN_TORQUE = 0;
const float MAX_TORQUE = 125; //adjust maxtorque so its actually at maxtorque at 100% pedal travel
//const float MIN_TORQUE = 0;
//const float MAX_TORQUE = 0;

const float REGEN_BASELINE_TORQUE = 0;
const float REGEN_MAX_TORQUE = -30;
//const float REGEN_BASELINE_TORQUE = 0;
//const float REGEN_MAX_TORQUE = 0;

const float MAX_RPM = 5500;

const uint16_t BSE_ADC_MIN_VAL = 371;
const uint16_t BSE_ADC_MAX_VAL = 4085;


const float APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE = 10;
const uint16_t APPS_IMPLAUSIBILITY_TIMEOUT_MILLIS = 100;

const uint16_t BSE_IMPLAUSIBILITY_TIMEOUT_MILLIS = 100;

const uint16_t BRAKE_ACTIVATED_ADC_VAL = 500;
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

const int RPM_READ_ID = 165;
const int INVERTER_VOLTAGE_READ_ID = 167;
const int BMS_DIAGNOSTICS_ID = 1712;
const int PRECHARGE_REQUEST_ID = 0x000;

const int PRECHARGE_TIMEOUT_MS = 10000;
const int PRECHARGE_VOLTAGE_DIFF = 25; //volts off from the pack voltage that is acceptable to finish precharge



const uint16_t RTD_BUTTON_PRESS_MILLIS = 500;
const uint16_t PRECHARGE_BUTTON_PRESS_MILLIS = 1000;


#define CUSTOM_BMS 0
#define ORION_BMS 1

#define ADC_BUFFER 3

#define ADC_READ_BUFFER 7

#define RPM_TO_CARSPEED_CONVFACTOR (59.0f * 32.0f * 3.14159f * 60.0f) / (12.0f * 39370.1f)

#endif /* INC_CONSTANTS_H_ */
