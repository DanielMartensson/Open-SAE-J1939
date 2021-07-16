/*
 * SAE_J1939_DM14.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "SAE_J1939-73_Diagnostics_Layer.h"

/*
 * Request DM14 from another ECU
 * PGN: 0x00D900 (55552)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_DM14(J1939* j1939, uint8_t DA, uint16_t number_of_requested_bytes, uint8_t pointer_type, uint8_t command, uint32_t pointer, uint8_t pointer_extension, uint16_t key) {
	uint32_t ID = (0x18D9 << 16) | (DA << 8) | j1939->this_ECU_address;
	uint8_t data[8];
	data[0] = number_of_requested_bytes;
	data[1] = (number_of_requested_bytes >> 3) | (pointer_type << 4) | (command << 1) | 0b1;
	data[2] = pointer;
	data[3] = pointer >> 8;
	data[4] = pointer >> 16;
	data[5] = pointer_extension;
	data[6] = key;
	data[7] = key >> 8;
	return CAN_Send_Message(ID, data, 0);									/* 0 ms delay */
}

/*
 * Response the request of DM14 memory request to other ECU about this ECU
 * PGN: 0x00D900 (55552)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_DM14(J1939 *j1939, uint8_t DA, uint8_t data[]) {
	uint16_t number_of_requested_bytes = ((data[1] & 0b11100000) << 3) | data[0];
	uint8_t pointer_type = (data[1] >> 4) & 0b0001;
	uint8_t command = (data[1] >> 1) & 0b0000111;
	uint32_t pointer = (data[4] << 16) | (data[3] << 8) | data[2];
	uint8_t pointer_extension = data[5];
	uint16_t key = (data[7] << 8) | data[6];

	/* Prepare DM15 PGN 00D800 response */
	// TODO: Koppla detta till en EEPROM funktion - Beskriv storleken på variablerna
	uint16_t number_of_allowed_bytes = 0;
	uint8_t status = 0;
	uint32_t EDC_parameter = 0;
	uint8_t EDCP_extention = 0;
	uint16_t seed = 0;
	SAE_J1939_Response_Request_DM15(j1939, DA, number_of_allowed_bytes, status, EDC_parameter, EDCP_extention, seed);

	/* Check if our message was OK - Send DM16 binary data transfer */
	if(status == STATUS_DM15_PROCEED) {
		uint8_t number_of_occurences = 256;
		uint8_t raw_binary_data[256];
		if(SAE_J1939_Send_Binary_Data_Transfer_DM16(j1939, DA, number_of_occurences, raw_binary_data) == STATUS_SEND_OK)
			SAE_J1939_Response_Request_DM15(j1939, DA, number_of_allowed_bytes, STATUS_DM15_OPERATION_COMPLETED, EDC_parameter, EDCP_extention, seed);
		else
			SAE_J1939_Response_Request_DM15(j1939, DA, number_of_allowed_bytes, STATUS_DM15_OPERATION_FAILED, EDC_parameter, EDCP_extention, seed);
	}
}
