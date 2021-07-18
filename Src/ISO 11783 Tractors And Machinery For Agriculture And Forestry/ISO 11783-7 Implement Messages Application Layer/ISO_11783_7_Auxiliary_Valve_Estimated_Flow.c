/*
 * ISO_11783_7_Auxiliary_Valve_Estimated_Flow.c
 *
 *  Created on: 16 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "ISO_11783_7_Implement_Messages_Application_Layer.h"

/*
 * Send an auxiliary valve estimated flow to all ECU
 * PGN: 0x00FE10 (65040) to 0x00FE1F (65055)
 */
ENUM_J1939_STATUS_CODES ISO_11783_Send_Auxiliary_Valve_Estimated_Flow_To_All_ECU(J1939 *j1939, uint8_t valve_number, uint8_t extend_estimated_flow_standard, uint8_t retract_estimated_flow_standard, uint8_t fail_safe_mode, uint8_t valve_state, uint8_t limit) {
	uint32_t ID = (0x0CFE << 16) | ((0x10 + valve_number) << 8) | j1939->this_ECU_address;
	uint8_t data[8];
	data[0] = extend_estimated_flow_standard;
	data[1] = retract_estimated_flow_standard;
	data[2] = (fail_safe_mode << 6) | (0b11 << 4) | valve_state; 	/* Bit 5 and 6 are reserved */
	data[3] = limit << 5;
	data[4] = data[5] = data[6] = data[7] = 0xFF;					/* All reserved */
	return CAN_Send_Message(ID, data, 0);							/* 0 ms delay */
}

/*
 * Read an auxiliary valve estimated flow from any ECU - Broadcast in other words
 * PGN: 0x00FE10 (65040) to 0x00FE1F (65055)
 */
void ISO_11783_Read_Auxiliary_Estimated_Flow(J1939 *j1939, uint8_t SA, uint8_t valve_number, uint8_t data[]) {
	j1939->auxiliary_valve_estimated_flow[SA][valve_number].extend_estimated_flow_standard = data[0];
	j1939->auxiliary_valve_estimated_flow[SA][valve_number].retract_estimated_flow_standard = data[1];
	j1939->auxiliary_valve_estimated_flow[SA][valve_number].fail_safe_mode = data[2] >> 6;
	j1939->auxiliary_valve_estimated_flow[SA][valve_number].valve_state = data[2] & 0b00001111;
	j1939->auxiliary_valve_estimated_flow[SA][valve_number].limit = data[3] >> 5;
}

