/*
 * ISO_11783_7_General_Purpose_Valve_Estimated_Flow.c
 *
 *  Created on: 16 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "ISO_11783-7_Implement_Messages_Application_Layer.h"

/*
 * Send a general purpose valve estimated flow to an ECU
 * PGN: 0x00C600 (50688)
 */
ENUM_J1939_STATUS_CODES ISO_11783_Send_General_Purpose_Valve_Estimated_Flow(J1939 *j1939, uint8_t DA) {
	uint32_t ID = (0x0CC6 << 16) | (DA << 8) | j1939->this_ECU_address;
	uint8_t data[8];
	data[0] = j1939->this_general_purpose_valve_estimated_flow.extend_estimated_flow_standard;
	data[1] = j1939->this_general_purpose_valve_estimated_flow.retract_estimated_flow_standard;
	data[2] = (j1939->this_general_purpose_valve_estimated_flow.fail_safe_mode << 6) | (0b11 << 4) | j1939->this_general_purpose_valve_estimated_flow.valve_state; 	/* Bit 5 and 6 are reserved */
	data[3] = j1939->this_general_purpose_valve_estimated_flow.limit << 5;
	data[4] = j1939->this_general_purpose_valve_estimated_flow.extend_estimated_flow_extended;
	data[5] = j1939->this_general_purpose_valve_estimated_flow.extend_estimated_flow_extended >> 8;
	data[6] = j1939->this_general_purpose_valve_estimated_flow.retract_estimated_flow_extended;
	data[7] = j1939->this_general_purpose_valve_estimated_flow.retract_estimated_flow_extended >> 8;
	return CAN_Send_Message(ID, data, 0);
}

/*
 * Read a general purpose valve estimated flow from an ECU
 * PGN: 0x00C600 (50688)
 */
void ISO_11783_Read_General_Purpose_Valve_Estimated_Flow(J1939 *j1939, uint8_t data[]) {
	j1939->general_purpose_valve_estimated_flow.extend_estimated_flow_standard = data[0];
	j1939->general_purpose_valve_estimated_flow.retract_estimated_flow_standard = data[1];
	j1939->general_purpose_valve_estimated_flow.fail_safe_mode = data[2] >> 6;
	j1939->general_purpose_valve_estimated_flow.valve_state = data[2] & 0b00001111;
	j1939->general_purpose_valve_estimated_flow.limit = data[3] >> 5;
	j1939->general_purpose_valve_estimated_flow.extend_estimated_flow_extended = (data[5] << 8) | data[4];
	j1939->general_purpose_valve_estimated_flow.retract_estimated_flow_extended = (data[7] << 8) | data[6];
}
