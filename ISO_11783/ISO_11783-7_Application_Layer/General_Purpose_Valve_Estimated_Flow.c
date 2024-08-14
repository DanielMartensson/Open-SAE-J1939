/*
 * General_Purpose_Valve_Estimated_Flow.c
 *
 *  Created on: 16 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Application_Layer.h"

/* Layers */
#include "../../SAE_J1939/SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"

/*
 * Request general purpose valve estimated flow to an ECU
 * PGN: 0x00C600 (50688)
 */
ENUM_J1939_STATUS_CODES ISO_11783_Send_Request_General_Purpose_Valve_Estimated_Flow(J1939 *j1939, uint8_t DA) {
	return SAE_J1939_Send_Request(j1939, DA, PGN_GENERAL_PURPOSE_VALVE_ESTIMATED_FLOW);
}

/*
 * Response the request general purpose valve estimated flow to an ECU
 * PGN: 0x00C600 (50688)
 */
ENUM_J1939_STATUS_CODES ISO_11783_Response_Request_General_Purpose_Valve_Estimated_Flow(J1939 *j1939, uint8_t DA) {
	uint32_t ID = (0x0CC6 << 16) | (DA << 8) | j1939->information_this_ECU.this_ECU_address;
	uint8_t data[8];
	data[0] = j1939->this_general_purpose_valve_estimated_flow.extend_estimated_flow_standard;
	data[1] = j1939->this_general_purpose_valve_estimated_flow.retract_estimated_flow_standard;
	data[2] = (j1939->this_general_purpose_valve_estimated_flow.fail_safe_mode << 6) | (0b11 << 4) | j1939->this_general_purpose_valve_estimated_flow.valve_state; 	/* Bit 5 and 6 are reserved for further use */
	data[3] = j1939->this_general_purpose_valve_estimated_flow.limit << 5;
	data[4] = j1939->this_general_purpose_valve_estimated_flow.extend_estimated_flow_extended;
	data[5] = j1939->this_general_purpose_valve_estimated_flow.extend_estimated_flow_extended >> 8;
	data[6] = j1939->this_general_purpose_valve_estimated_flow.retract_estimated_flow_extended;
	data[7] = j1939->this_general_purpose_valve_estimated_flow.retract_estimated_flow_extended >> 8;
	return CAN_Send_Message(ID, data);
}

/*
 * Read a response request general purpose valve estimated flow from an ECU
 * PGN: 0x00C600 (50688)
 */
void ISO_11783_Read_Response_Request_General_Purpose_Valve_Estimated_Flow(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	j1939->from_other_ecu_general_purpose_valve_estimated_flow.extend_estimated_flow_standard = data[0];
	j1939->from_other_ecu_general_purpose_valve_estimated_flow.retract_estimated_flow_standard = data[1];
	j1939->from_other_ecu_general_purpose_valve_estimated_flow.fail_safe_mode = data[2] >> 6;
	j1939->from_other_ecu_general_purpose_valve_estimated_flow.valve_state = data[2] & 0b00001111;
	j1939->from_other_ecu_general_purpose_valve_estimated_flow.limit = data[3] >> 5;
	j1939->from_other_ecu_general_purpose_valve_estimated_flow.extend_estimated_flow_extended = (data[5] << 8) | data[4];
	j1939->from_other_ecu_general_purpose_valve_estimated_flow.retract_estimated_flow_extended = (data[7] << 8) | data[6];
	j1939->from_other_ecu_general_purpose_valve_estimated_flow.from_ecu_address = SA;
}
