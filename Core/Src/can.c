/*
 * can.c
 *
 *  Created on: May 6, 2025
 *      Author: antho
 */

#include "can.h"

/**
 * @brief Send the torque command message over CAN.
 */
void sendTorqueCommand(void) {
	int torqueValue = (int) (requestedTorque * 10); // Convert to integer, multiply by 10

	// Break the torqueValue into two bytes (little-endian)
	char msg0 = torqueValue & 0xFF;
	char msg1 = (torqueValue >> 8) & 0xFF;

	txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	txMessage.frame.id = 0x0C0;
	txMessage.frame.dlc = 8;

	txMessage.frame.data0 = msg0; //torque request
	txMessage.frame.data1 = msg1;
	txMessage.frame.data2 = 0; // speed request (only maters in speed mode)
	txMessage.frame.data3 = 0;
	txMessage.frame.data4 = 0; //direction

	//lockout
	if (beginTorqueRequests) {
		txMessage.frame.data5 = 1; //
	} else {
		txMessage.frame.data5 = 0;
	}
	txMessage.frame.data6 = 0;
	txMessage.frame.data7 = 0;

	CANSPI_Transmit(&txMessage);
}

/**
 * @brief Parse BMS diagnostics from a received CAN message
 */
void updateBMSDiagnostics(void) {
	// Pack_Current (signed 16-bit at bit 8, factor 0.1)
	int16_t pack_current_raw = (int16_t) ((rxMessage.frame.data0 << 8)
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
 * @brief Parse BMS diagnostics from a received CAN message
 */
void updateInverterVolts(void) {
	// DC volts (signed 16-bit at bit 8, factor 0.1)
	int16_t inverter_dc_volts_raw = (int16_t) ((rxMessage.frame.data1 << 8)
			| rxMessage.frame.data0);  // Little-endian
	float inverter_dc_volts = inverter_dc_volts_raw * 0.1f;

	inverter_diagnostics.inverterDCVolts = (int) inverter_dc_volts;
}


/**
 // * @brief Update Inverter RPM reading from the last received CAN message.
 */
void updateRpm(InverterDiagnostics* inverter_diagnostics) {
	inverter_diagnostics->motorRpm = (float) (rxMessage.frame.data2
			| (rxMessage.frame.data3 << 8));
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

