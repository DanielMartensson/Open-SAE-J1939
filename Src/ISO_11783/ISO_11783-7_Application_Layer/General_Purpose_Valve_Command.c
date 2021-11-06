/*
 * General_Purpose_Valve_Command.c
 *
 *  Created on: 16 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Application_Layer.h"

/* Layers */
#include "../../SAE_J1939/SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"

/*
 * Send a general purpose valve command to an ECU
 * PGN: 0x00C400 (50176)
 */
ENUM_J1939_STATUS_CODES ISO_11783_Send_General_Purpose_Valve_Command(J1939 *j1939, uint8_t DA, uint8_t standard_flow, uint8_t fail_safe_mode, uint8_t valve_state, uint16_t extended_flow) {
	uint32_t ID = (0x0CC4 << 16) | (DA << 8) | j1939->information_this_ECU.this_ECU_address;
	uint8_t data[8];
	data[0] = standard_flow;
	data[1] = 0xFF; 												 /* Reserved */
	data[2] = (fail_safe_mode << 6) | (0b11 << 4) | valve_state; 	 /* Bit 5 and 6 are reserved */
	data[3] = extended_flow;
	data[4] = extended_flow >> 8;
	data[5] = data[6] = data[7] = 0xFF;								 /* All reserved */
	return CAN_Send_Message(ID, data);
}

/*
 * Read a general purpose valve command from an ECU
 * PGN: 0x00C400 (50176)
 */
void ISO_11783_Read_General_Purpose_Valve_Command(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	j1939->from_other_ecu_general_purpose_valve_command.standard_flow = data[0];
	j1939->from_other_ecu_general_purpose_valve_command.fail_safe_mode = data[2] >> 6;
	j1939->from_other_ecu_general_purpose_valve_command.valve_state = data[2] & 0b00001111;
	j1939->from_other_ecu_general_purpose_valve_command.extended_flow = (data[4] << 8) | data[3];
	j1939->from_other_ecu_general_purpose_valve_command.from_ecu_address = SA;
}
