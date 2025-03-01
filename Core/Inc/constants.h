/*
 * constants.h
 *
 *  Created on: Jan 28, 2025
 *      Author: nakuljoshi
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

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
const uint16_t CROSS_CHECK_RESTORATION_APPS_PERCENT = 5;
const uint16_t CROSS_CHECK_IMPLAUSIBILITY_TIMEOUT_MILLIS = 100;

const uint16_t DMA_READ_TIMEOUT = 10;

#endif /* INC_CONSTANTS_H_ */
