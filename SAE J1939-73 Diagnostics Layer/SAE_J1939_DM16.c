/*
 * SAE_J1939_DM16.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "SAE_J1939-73_Diagnostics_Layer.h"

/* Send binary data transfer. This will be sent after DM15 memory response (if it was proceeded)
 * PGN 0x00D700 (55040)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Binary_Data_Transfer_DM16(J1939 *j1939, uint8_t DA, uint8_t number_of_occurences, uint8_t raw_binary_data[]) {
	if(number_of_occurences < 8) {
		uint32_t ID = (0x18D7 << 16) | (DA << 8) | j1939->this_ECU_address;
		uint8_t data[number_of_occurences + 1];								/* number_of_occurences must be 7 */
		data[0] = number_of_occurences;										/* How much binary data we want to send */
		for(uint8_t i = 0; i < number_of_occurences; i++)
			data[i+1] = raw_binary_data[i];
		return CAN_Send_Message(ID, data, 0);								/* 0 ms delay */
	}else{
		/* Multiple messages - Use Transport Protocol Connection Management BAM */
		uint8_t data[number_of_occurences + 1];
		uint16_t total_message_size = 0;									/* This can be maximum 256 */
		data[total_message_size++] = number_of_occurences;
		for(uint8_t i = 0; i < number_of_occurences; i++)
			data[total_message_size++] = raw_binary_data[i];				/* When i = 0, then total_message_size = 1 */

		/* Send TP CM BAM and then TP DT data */
		uint8_t number_of_packages = total_message_size % 8 > 1 ? total_message_size/8 + 1 : total_message_size/8; /* Rounding up */
		ENUM_J1939_STATUS_CODES status = SAE_J1939_Send_Transport_Protocol_Connection_Management(j1939, DA, CONTROL_BYTE_TP_CM_BAM, total_message_size, number_of_packages, PGN_DM16);
		if(status != STATUS_SEND_OK)
			return status;
		return SAE_J1939_Send_Transport_Protocol_Data_Transfer(j1939, DA, data, total_message_size, number_of_packages);
	}
}


/*
 * Read binary data transfer
 * PGN 0x00D700 (55040)
 */
void SAE_J1939_Read_Binary_Data_Transfer_DM16(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	j1939->dm[SA].dm16.number_of_occurences = data[0];
	for(uint8_t i = 0; i < 256; i++)
		if(i < data[0])
			j1939->dm[SA].dm16.raw_binary_data[i] = data[i+1];
		else
			j1939->dm[SA].dm16.raw_binary_data[i] = 0xFF;					/* No data */
}

