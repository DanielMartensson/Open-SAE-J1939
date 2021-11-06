/*
 * Auxiliary_Valve_Estimated_Flow.c
 *
 *  Created on: 16 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Application_Layer.h"

/* Layers */
#include "../../SAE_J1939/SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"

/*
 * Request auxiliary valve estimated flow to all ECU
 * PGN: 0x00FE10 (65040) to 0x00FE1F (65055)
 */
ENUM_J1939_STATUS_CODES ISO_11783_Send_Request_Auxiliary_Valve_Estimated_Flow(J1939 *j1939, uint8_t DA, uint8_t valve_number) {
	return SAE_J1939_Send_Request(j1939, DA, PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_0 + valve_number); /* valve_number can be 0 to 15 */
}

/*
 * Response the request auxiliary valve estimated flow to all ECU
 * PGN: 0x00FE10 (65040) to 0x00FE1F (65055)
 */
ENUM_J1939_STATUS_CODES ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(J1939 *j1939, uint8_t valve_number) {
	uint32_t ID = (0x0CFE << 16) | ((0x10 + valve_number) << 8) | j1939->information_this_ECU.this_ECU_address;
	uint8_t data[8];
	data[0] = j1939->this_auxiliary_valve_estimated_flow[valve_number].extend_estimated_flow_standard;
	data[1] = j1939->this_auxiliary_valve_estimated_flow[valve_number].retract_estimated_flow_standard;
	data[2] = (j1939->this_auxiliary_valve_estimated_flow[valve_number].fail_safe_mode << 6) | (0b11 << 4) | j1939->this_auxiliary_valve_estimated_flow[valve_number].valve_state; 	/* Bit 5 and 6 are reserved */
	data[3] = j1939->this_auxiliary_valve_estimated_flow[valve_number].limit << 5;
	data[4] = data[5] = data[6] = data[7] = 0xFF;					/* All reserved */
	return CAN_Send_Message(ID, data);
}

/*
 * Read a response request auxiliary valve estimated flow from any ECU - Broadcast in other words
 * PGN: 0x00FE10 (65040) to 0x00FE1F (65055)
 */
void ISO_11783_Read_Response_Request_Auxiliary_Estimated_Flow(J1939 *j1939, uint8_t SA, uint8_t valve_number, uint8_t data[]) {
	j1939->from_other_ecu_auxiliary_valve_estimated_flow[valve_number].extend_estimated_flow_standard = data[0];
	j1939->from_other_ecu_auxiliary_valve_estimated_flow[valve_number].retract_estimated_flow_standard = data[1];
	j1939->from_other_ecu_auxiliary_valve_estimated_flow[valve_number].fail_safe_mode = data[2] >> 6;
	j1939->from_other_ecu_auxiliary_valve_estimated_flow[valve_number].valve_state = data[2] & 0b00001111;
	j1939->from_other_ecu_auxiliary_valve_estimated_flow[valve_number].limit = data[3] >> 5;
	j1939->from_other_ecu_auxiliary_valve_estimated_flow[valve_number].from_ecu_address = SA;
}

