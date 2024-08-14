/*
 * Auxiliary_Valve_Command.c
 *
 *  Created on: 16 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Application_Layer.h"

/* Layers */
#include "../../SAE_J1939/SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"

/*
 * Send an auxiliary valve command to all ECU
 * PGN: 0x00FE30 (65072) to 0x00FE3F (65087)
 */
ENUM_J1939_STATUS_CODES ISO_11783_Send_Auxiliary_Valve_Command(J1939 *j1939, uint8_t valve_number, uint8_t standard_flow, uint8_t fail_safe_mode, uint8_t valve_state) {
	uint32_t ID = (0x0CFE << 16) | ((0x30 + valve_number) << 8) | j1939->information_this_ECU.this_ECU_address;
	uint8_t data[8];
	data[0] = standard_flow;
	data[1] = 0xFF; 												/* Reserved */
	data[2] = (fail_safe_mode << 6) | (0b11 << 4) | valve_state; 	/* Bit 5 and 6 are reserved */
	data[3] = data[4] = data[5] = data[6] = data[7] = 0xFF;			/* All reserved */
	return CAN_Send_Message(ID, data);
}

/*
 * Read an auxiliary valve command from any ECU - Broadcast in other words
 * PGN: 0x00FE30 (65072) to 0x00FE3F (65087)
 */
void ISO_11783_Read_Auxiliary_Valve_Command(J1939 *j1939, uint8_t SA, uint8_t valve_number, uint8_t data[]) {
	j1939->from_other_ecu_auxiliary_valve_command[valve_number].standard_flow = data[0];
	j1939->from_other_ecu_auxiliary_valve_command[valve_number].fail_safe_mode = data[2] >> 6;
	j1939->from_other_ecu_auxiliary_valve_command[valve_number].valve_state = data[2] & 0b00001111;
	j1939->from_other_ecu_auxiliary_valve_command[valve_number].from_ecu_address = SA;
}
