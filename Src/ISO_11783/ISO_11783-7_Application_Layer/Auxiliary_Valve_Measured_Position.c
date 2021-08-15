/*
 * ISO_11783_7_Auxiliary_Valve_Measured_Position.c
 *
 *  Created on: 16 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "../../ISO_11783/ISO_11783-7_Application_Layer/Application_Layer.h"

/*
 * Request auxiliary valve measured position to all ECU
 * PGN: 0x00FF20 (65312) to 0x00FF2F (65327)
 */
ENUM_J1939_STATUS_CODES ISO_11783_Send_Request_Auxiliary_Valve_Measured_Position(J1939 *j1939, uint8_t DA, uint8_t valve_number) {
	return SAE_J1939_Send_Request(j1939, DA, PGN_AUXILIARY_VALVE_MEASURED_POSITION_0 + valve_number); /* valve_number can be 0 to 15 */
}

/*
 * Response the request auxiliary valve measured position to all ECU
 * PGN: 0x00FF20 (65312) to 0x00FF2F (65327)
 */
ENUM_J1939_STATUS_CODES ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(J1939 *j1939, uint8_t valve_number) {
	uint32_t ID = (0x0CFF << 16) | ((0x20 + valve_number) << 8) | j1939->this_ECU_address;
	uint8_t data[8];
	data[0] = j1939->this_auxiliary_valve_measured_position[valve_number].measured_position_procent;
	data[1] = j1939->this_auxiliary_valve_measured_position[valve_number].measured_position_procent >> 8;
	data[2] = j1939->this_auxiliary_valve_measured_position[valve_number].valve_state;
	data[3] = j1939->this_auxiliary_valve_measured_position[valve_number].measured_position_micrometer;
	data[4] = j1939->this_auxiliary_valve_measured_position[valve_number].measured_position_micrometer >> 8;
	data[5] = data[6] = data[7] = 0xFF;								/* All reserved */
	return CAN_Send_Message(ID, data, 0);							/* 0 ms delay */
}

/*
 * Read a response request auxiliary valve measured position from any ECU - Broadcast in other words
 * PGN: 0x00FF20 (65312) to 0x00FF2F (65327)
 */
void ISO_11783_Read_Response_Request_Auxiliary_Valve_Measured_Position(J1939 *j1939, uint8_t SA, uint8_t valve_number, uint8_t data[]) {
	j1939->from_other_ecu_auxiliary_valve_measured_position[valve_number].measured_position_procent = (data[1] << 8) | data[0];
	j1939->from_other_ecu_auxiliary_valve_measured_position[valve_number].valve_state = data[2];
	j1939->from_other_ecu_auxiliary_valve_measured_position[valve_number].measured_position_micrometer = (data[4] << 8) | data[3];
	j1939->from_other_ecu_auxiliary_valve_measured_position[valve_number].from_ecu_address = SA;
}
