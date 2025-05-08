/**
 * @brief Repeatedly check the shutdown pin; if high, set torque to 0 and block forever.
 */
void checkShutdown() {
	uint8_t pinState = HAL_GPIO_ReadPin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin);
	if (pinState == GPIO_PIN_RESET) {
		requestedTorque = 0;
		sendTorqueCommand();
		HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin, GPIO_PIN_RESET); //open positive air
		HAL_Delay(5);
		HAL_GPIO_WritePin(AIR_N_CTRL_GPIO_Port, AIR_N_CTRL_Pin, GPIO_PIN_RESET); //open negative air
		HAL_Delay(5);
		HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin,
				GPIO_PIN_RESET); //ooen prechrge relay
		HAL_Delay(5);

		readyToDrive = 0;

		prechargeState = 0;
		rtdState = 0;
		prechargeFinished = 0;
		cpockandballs = 0;

		lookForRTD();
	}
}

/**
 * @brief Check if the driver has pressed the brake pedal and the RTD pin is set.
 */
void checkReadyToDrive(void) {
	uint8_t pinState = HAL_GPIO_ReadPin(RTD_BTN_GPIO_Port, RTD_BTN_Pin);
	cpockandballs = HAL_GetTick() - millis_RTD;
	if (pinState == GPIO_PIN_SET && bseValue > BRAKE_ACTIVATED_ADC_VAL
			&& bms_diagnostics.inverterActive && !rtdState) {
		rtdState = true;
		millis_RTD = HAL_GetTick();
	} else if (pinState == GPIO_PIN_RESET || bseValue < BRAKE_ACTIVATED_ADC_VAL
			|| !bms_diagnostics.inverterActive) {
		rtdState = false;
	} else if (cpockandballs >= RTD_BUTTON_PRESS_MILLIS) {
		readyToDrive = true;
	}
}

/**
 * @brief If a hardware pin requests precharge, triggers precharge sequence
 */
void sendPrechargeRequest(void) {
	uint8_t pinState = HAL_GPIO_ReadPin(PRECHARGE_BTN_GPIO_Port,
			PRECHARGE_BTN_Pin);
	if (BMS_TYPE == ORION_BMS) {
		if (pinState == GPIO_PIN_SET && !prechargeState) {
			prechargeState = true;
			millis_precharge = HAL_GetTick();
		} else if (pinState == GPIO_PIN_RESET) {
			prechargeState = false;
		} else if (HAL_GetTick() - millis_precharge >= PRECHARGE_BUTTON_PRESS_MILLIS) {
			prechargeSequence();
			prechargeFinished = true;
		}
	} else if (BMS_TYPE == CUSTOM_BMS) {
		while (1) {
		}
	}
}

/**
 * @brief the actual procedure of triggering precharge relays
 * 1) air- closes
 * 2) precharge closes
 * 3) spam bms reads
 * 3.5) wait until voltage is within 10v of bms packvolts
 * 3.75) timeout at 3s or something like that, prob less
 * 4) close positive air if successful precharge and then  precharge relay after short delay
 * 4.5) if prechg is unsuccessful, open neg air and precharge and error and infinite loop
 *
 *@return returns 1 if precharge successful, infinite loop if it doesn't work
 */
uint32_t startPrechargeTime = 0;
uint32_t penis = 0;
uint8_t prechargeSequence(void) {
	startPrechargeTime = HAL_GetTick();

	HAL_Delay(10);

	HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin, GPIO_PIN_SET); //steps 1 and 2
	HAL_Delay(3);
	HAL_GPIO_WritePin(AIR_N_CTRL_GPIO_Port, AIR_N_CTRL_Pin, GPIO_PIN_SET); //steps 1 and 2
//	  HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin, GPIO_PIN_RESET); //steps 1 and 2

	penis = HAL_GetTick() - startPrechargeTime;
	while (penis < PRECHARGE_TIMEOUT_MS
			&& HAL_GPIO_ReadPin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin)
					== GPIO_PIN_SET) { //loop for 4

		//get inverter & BMS can data
		if (CANSPI_Receive(&rxMessage)) {
			readFromCAN();
		}

		if (bms_diagnostics.packVoltage - inverter_diagnostics.inverterDCVolts
				< PRECHARGE_VOLTAGE_DIFF) {

			HAL_GPIO_WritePin(AIR_P_CTRL_GPIO_Port, AIR_P_CTRL_Pin,
					GPIO_PIN_SET); //step 4
			HAL_Delay(5);
			HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin,
					GPIO_PIN_RESET);
			return 1;
		}

		penis = HAL_GetTick() - startPrechargeTime;
	}

	HAL_GPIO_WritePin(PCHG_RLY_CTRL_GPIO_Port, PCHG_RLY_CTRL_Pin,
			GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(AIR_N_CTRL_GPIO_Port, AIR_N_CTRL_Pin, GPIO_PIN_RESET);
	while (1) {

	}

	//fill this in pls justin
}

/**
 * @brief Play the startup sound from Flash exactly once.
 */
void PlayStartupSoundOnce(void) {
	wavePCM = (const uint16_t*) &startup_sound[WAV_HEADER_SIZE];
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

uint8_t rtdoverride = 0;

void lookForRTD(void) {
	if (rtdoverride == 1) {
		beginTorqueRequests = true;
		PlayStartupSoundOnce();
		return;
	}
	while (!readyToDrive) {

		if (CANSPI_Receive(&rxMessage)) {
			readFromCAN();
		}

		if (dma_read_complete) {
			HAL_ADC_Start_DMA(&hadc1, ADC_Reads, ADC_BUFFER);
			dma_read_complete = 0;
			millis_since_dma_read = HAL_GetTick();
		}

		//TODO: add check for make sure apps are 0 travel

		while (HAL_GPIO_ReadPin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin)
				== GPIO_PIN_RESET) {
			HAL_Delay(1);
		}

		while (!prechargeFinished) {

			if (CANSPI_Receive(&rxMessage)) {
				readFromCAN();
			}
			sendPrechargeRequest();
		}
		// If the driver is ready to drive, send torque over CAN
		uint8_t prevReadyToDrive = readyToDrive;
		checkReadyToDrive();

		if (readyToDrive) {
			// If we just transitioned from not-ready to ready, play sound
			if (!prevReadyToDrive) {
				beginTorqueRequests = true;
				PlayStartupSoundOnce();

				//send disable message, maybe this'll let lockout go away
				//also erectiledysfunction is just a like loop iter variable
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
	}
}
