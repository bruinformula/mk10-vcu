#include "pedalProcessing.h"


/**
 * @brief Calculate the requested torque based on APPS and RPM, or regen based on BSE.
 */
//void calculateTorqueRequest(float* requestedTorque) {
//	float apps1_as_percent = ((float) apps1Value - APPS_1_ADC_MIN_VAL)
//					/ (APPS_1_ADC_MAX_VAL - APPS_1_ADC_MIN_VAL);
//	float apps2_as_percent = ((float) apps2Value - APPS_2_ADC_MIN_VAL)
//					/ (APPS_2_ADC_MAX_VAL - APPS_2_ADC_MIN_VAL);
//	float appsValue = (apps1_as_percent + apps2_as_percent) / 2.0f;
//
//	if (appsValue > 0) {
//		// Pedal-based torque map
//		int numPedalSteps = 10;
//		int numRpmSteps   = 10;
//
//		float pedalStepSize = 100.0f / (numPedalSteps - 1);
//		float rpmStepSize   = MAX_RPM / (numRpmSteps - 1);
//
//		int pedalLowIndx = (int)(appsValue / (pedalStepSize / 100.0f));  // handle properly if needed
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
//		float pedalLerp = (appsValue * 100.0f - (pedalLowIndx * pedalStepSize)) / pedalStepSize;
//		float rpmLerp   = (float)(inverter_diagnostics.motorRpm - (rpmLowIndx * rpmStepSize)) / rpmStepSize;
//
//		float torqueLow  = T00 + (T01 - T00) * rpmLerp;
//		float torqueHigh = T10 + (T11 - T10) * rpmLerp;
//
//		requestedTorque = torqueLow + (torqueHigh - torqueLow) * pedalLerp;
//	}
//	else {
//		// Regen based on brake pedal
//		float bse_as_percent = ((float) bseValue - BSE_ADC_MIN_VAL)
//						/ (BSE_ADC_MAX_VAL - BSE_ADC_MIN_VAL);
//		requestedTorque = (REGEN_MAX_TORQUE - REGEN_BASELINE_TORQUE)
//						* bse_as_percent + REGEN_BASELINE_TORQUE;
//	}
//}
void calculateTorqueRequest(float* requestedTorque) {

	float apps1_as_percent = ((float) apps1Value - APPS_1_ADC_MIN_VAL)
			/ (APPS_1_ADC_MAX_VAL - APPS_1_ADC_MIN_VAL);
	float apps2_as_percent = ((float) apps2Value - APPS_2_ADC_MIN_VAL)
			/ (APPS_2_ADC_MAX_VAL - APPS_2_ADC_MIN_VAL);
	float appsValue = ((float) apps1_as_percent + apps2_as_percent) / 2;
	if (appsValue >= 0) {
		*requestedTorque = ((float) (MAX_TORQUE - MIN_TORQUE)) * appsValue
				+ MIN_TORQUE;
	} else {
		float bse_as_percent = ((float) bseValue - BSE_ADC_MIN_VAL)
				/ (BSE_ADC_MAX_VAL - BSE_ADC_MIN_VAL);
		*requestedTorque = (REGEN_MAX_TORQUE - REGEN_BASELINE_TORQUE)
				* bse_as_percent + REGEN_BASELINE_TORQUE;
	}
}

/**
 * @brief Check plausibility of APPS sensors.
 */
void checkAPPSPlausibility(float* requestedTorque, uint16_t* apps_plausible, uint32_t millis_since_apps_implausible) {
	apps1_as_percent = ((float) apps1Value - APPS_1_ADC_MIN_VAL) / (APPS_1_ADC_MAX_VAL - APPS_1_ADC_MIN_VAL) * 100.0f;
	apps2_as_percent = ((float) apps2Value - APPS_2_ADC_MIN_VAL) / (APPS_2_ADC_MAX_VAL - APPS_2_ADC_MIN_VAL) * 100.0f;

	float paininmyass = fabsf(apps1_as_percent - apps2_as_percent);

	if (fabsf(apps1_as_percent - apps2_as_percent) > APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE) {
		millis_since_apps_implausible = HAL_GetTick();
		apps_plausible = 0; // false
		requestedTorque = 0;
	} else if (!apps_plausible && (HAL_GetTick() - millis_since_apps_implausible < APPS_IMPLAUSIBILITY_TIMEOUT_MILLIS)) {
		requestedTorque = 0;
	} else {
		apps_plausible = 1; // true
	}
}

/**
 * @brief Check rank-check between APPS and brake pedal.
 * returns 1 if crosscheck is plausible, 0 otherwise.
 */
uint16_t checkCrossCheck(float* bse_as_percent, uint32_t* bseValue, uint32_t* apps1Value, uint32_t* apps2Value, float* requestedTorque) {
	bse_as_percent = ((float) bseValue - BSE_ADC_MIN_VAL) / (BSE_ADC_MAX_VAL - BSE_ADC_MIN_VAL) * 100.0f;

	float apps1p = ((float) apps1Value - APPS_1_ADC_MIN_VAL) / (APPS_1_ADC_MAX_VAL - APPS_1_ADC_MIN_VAL) * 100.0f;

	float apps2p = ((float) apps2Value - APPS_2_ADC_MIN_VAL) / (APPS_2_ADC_MAX_VAL - APPS_2_ADC_MIN_VAL) * 100.0f;

	float apps_as_percent = (apps1p + apps2p) / 2.0f;


	if (apps_as_percent > CROSS_CHECK_IMPLAUSIBILITY_APPS_PERCENT && bseValue > BRAKE_ACTIVATED_ADC_VAL) {
		requestedTorque = 0;
		return 0;
	} else if (!cross_check_plausible && apps_as_percent > CROSS_CHECK_RESTORATION_APPS_PERCENT) {
		requestedTorque = 0;
		return 0;
	} else {
		return 1;
	}
}
