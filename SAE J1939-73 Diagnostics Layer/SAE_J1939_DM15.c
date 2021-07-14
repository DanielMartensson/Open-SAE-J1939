/*
 * SAE_J1939_DM15.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "SAE_J1939-73_Diagnostics_Layer.h"

/*
 * Send a memory response. This will be sent after a DM14 memory request
 * *j1939: Pointer to structure J1939
 * DA: Destination ECU address between 0 to 255 (DA 255 = Broadcast to all ECU)
 * number_of_allowed_bytes: How many bytes we have been allowed to read between 0 to 2047
 * status: Result of the response. See the ENUM_DM15_CODES in the header file above.
 * error_indicator_EDC_parameter: See J1939 protocol. Value between Between 0 to 16777215
 * EDCP_extention: This is an 8-bit parameter used to identify how to handle the data in the Error Indicator/ EDC Parameter
 * seed: TODO: Skriv en egen manual
 * PGN: 0x00D800 (55296)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_DM15(J1939 *j1939, uint8_t DA, uint16_t number_of_allowed_bytes, uint8_t status, uint32_t error_indicator_EDC_parameter, uint8_t EDCP_extention, uint16_t seed) {
	uint32_t ID = (0x18D8 << 16) | (DA << 8) | j1939->this_address;
	uint8_t response_data[8];
	response_data[0] = number_of_allowed_bytes;
	response_data[1] = (number_of_allowed_bytes >> 3) | (0b1 << 4) | (status << 1) | 0b1;	/* bit 5 and 1 are reserved */
	response_data[2] = error_indicator_EDC_parameter;
	response_data[3] = error_indicator_EDC_parameter >> 8;
	response_data[4] = error_indicator_EDC_parameter >> 16;
	response_data[5] = EDCP_extention;
	response_data[6] = seed;
	response_data[7] = seed >> 8;
	ENUM_J1939_STATUS_CODES status = CAN_Send_Message(ID, response_data, 0); 				/* 0 ms delay */
}

/*
 * Store the DM15 information about other ECU (This is actually a DM14 response according to the J1939 standard)
 * *j1939: Pointer to structure J1939
 * SA: Source address from which ECU address the message came from
 * data[]: X bytes data array
 * PGN: 0x00D800 (55296)
 */
void SAE_J1939_Read_Response_Request_DM15(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	j1939->dm[SA].dm15.number_of_allowed_bytes = (data[1] >> 3) | data[0];
	j1939->dm[SA].dm15.status = (data[1] >> 1) & 0b0000111;
	j1939->dm[SA].dm15.error_indicator_EDC_parameter = (data[4] << 16) | (data[3] << 8) | data[2];
	j1939->dm[SA].dm15.EDCP_extention = data[5];
	j1939->dm[SA].dm15.seed = (data[7] << 8) | data[6];
}

