/*
 * DM14.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Diagnostics_Layer.h"

/* Layers */
#include "../SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"

/*
 * Request DM14 from another ECU
 * PGN: 0x00D900 (55552)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_DM14(J1939 *j1939, uint8_t DA, uint16_t number_of_requested_bytes, uint8_t pointer_type, uint8_t command, uint32_t pointer, uint8_t pointer_extension, uint16_t key) {
	uint32_t ID = (0x18D9 << 16) | (DA << 8) | j1939->information_this_ECU.this_ECU_address;
	uint8_t data[8];
	data[0] = number_of_requested_bytes;
	data[1] = (number_of_requested_bytes >> 3) | (pointer_type << 4) | (command << 1) | 0b1;
	data[2] = pointer;
	data[3] = pointer >> 8;
	data[4] = pointer >> 16;
	data[5] = pointer_extension;
	data[6] = key;
	data[7] = key >> 8;
	return CAN_Send_Message(ID, data);
}

/*
 * Read the request of DM14 memory request to other ECU about this ECU
 * PGN: 0x00D900 (55552)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Read_Request_DM14(J1939 *j1939, uint8_t DA, uint8_t data[]) {
	/* These are information the ECU want to have */
	uint16_t number_of_requested_bytes = ((data[1] & 0b11100000) << 3) | data[0];
	uint8_t pointer_type = (data[1] >> 4) & 0b0001;
	uint8_t command = (data[1] >> 1) & 0b0000111;
	uint32_t pointer = (data[4] << 16) | (data[3] << 8) | data[2];
	uint8_t pointer_extension = data[5];
	uint16_t key = (data[7] << 8) | data[6];

	/* Load up the amount of bytes we want to send via DM16 */
	uint8_t number_of_occurences = number_of_requested_bytes;
	uint8_t raw_binary_data[number_of_requested_bytes];

	/* Here we ask the flash, eeprom or ram and use pointers */
	FLASH_EEPROM_RAM_Memory(&number_of_requested_bytes, pointer_type, &command, &pointer, &pointer_extension, &key, raw_binary_data);

	/* Prepare a DM15 PGN: 0x00D800 response back to DA */
	uint16_t number_of_allowed_bytes = number_of_requested_bytes;
	uint8_t status = command;
	uint32_t EDC_parameter = pointer;
	uint8_t EDCP_extention = pointer_extension;
	uint16_t seed = key;
	status = SAE_J1939_Send_Response_DM15(j1939, DA, number_of_allowed_bytes, status, EDC_parameter, EDCP_extention, seed);

	/* Check if our message was OK - Send DM16 binary data transfer */
	if(status == STATUS_DM15_PROCEED) {
		if(SAE_J1939_Send_Binary_Data_Transfer_DM16(j1939, DA, number_of_occurences, raw_binary_data) == STATUS_SEND_OK)
			status = SAE_J1939_Send_Response_DM15(j1939, DA, number_of_allowed_bytes, STATUS_DM15_OPERATION_COMPLETED, EDC_parameter, EDCP_extention, seed);
		else
			status = SAE_J1939_Send_Response_DM15(j1939, DA, number_of_allowed_bytes, STATUS_DM15_OPERATION_FAILED, EDC_parameter, EDCP_extention, seed);
	}
	return status;

}
