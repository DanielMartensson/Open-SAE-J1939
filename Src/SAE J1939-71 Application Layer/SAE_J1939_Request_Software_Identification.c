/*
 * SAE_J1939_Request_Software_Identification.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "SAE_J1939-71_Application_Layer.h"

/*
 * Send request software identification to another ECU
 * PGN: 0x00FEDA (65242)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Software_Identification(J1939 *j1939, uint8_t DA) {
	return SAE_J1939_Send_Request(j1939, DA, PGN_SOFTWARE_IDENTIFICATION);
}

/*
 * Response the request of the software identification about this ECU
 * PGN: 0x00FEDA (65242)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Software_Identification(J1939* j1939, uint8_t DA) {
	uint8_t number_of_fields = j1939->this_software_identification.number_of_fields;
	uint8_t length_of_each_identification = j1939->this_software_identification.length_of_each_identification;
	if (number_of_fields*length_of_each_identification < 9) {
		/* If we only using 1 identification field and our identification is below 8, then we can send software identification as a normal message */
		uint32_t ID = (0x18FEDA << 8) | j1939->this_ECU_address;
		uint8_t data[8];
		data[0] = j1939->this_software_identification.number_of_fields;		/* J1939 standard */
		for(uint8_t i = 0; i < 7; i++)
			data[i+1] = j1939->this_software_identification.identifications[i];
		return CAN_Send_Message(ID, data, 0);								/* 0 ms delay */
	} else {
		/* Multiple messages - Use Transport Protocol Connection Management BAM */
		uint16_t total_message_size = 0;
		uint8_t data[1 + length_of_each_identification*number_of_fields];	/* 1 for number_of_fields */
		data[total_message_size++] = number_of_fields;
		for(uint8_t i = 0; i < number_of_fields*length_of_each_identification; i++)
			data[total_message_size++] = j1939->this_software_identification.identifications[i];

		/* Send TP CM BAM and then TP DT data */
		uint8_t number_of_packages = total_message_size % 8 > 1 ? total_message_size/8 + 1 : total_message_size/8; /* Rounding up */
		ENUM_J1939_STATUS_CODES status = SAE_J1939_Send_Transport_Protocol_Connection_Management(j1939, DA, CONTROL_BYTE_TP_CM_BAM, total_message_size, number_of_packages, PGN_SOFTWARE_IDENTIFICATION);
		if(status != STATUS_SEND_OK)
			return status;
		return SAE_J1939_Send_Transport_Protocol_Data_Transfer(j1939, DA, data, total_message_size, number_of_packages);
	}
}

/*
 * Store the software identification about other ECU
 * PGN: 0x00FEDA (65242)
 */
void SAE_J1939_Read_Response_Request_Software_Identification(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	j1939->software_identification[SA].number_of_fields = data[0];			/* How many fields we have */
	uint8_t length_of_each_identification = j1939->software_identification[SA].length_of_each_identification;
	for(uint8_t i = 0; i < data[0]*length_of_each_identification; i++)
		j1939->software_identification[SA].identifications[i] = data[i+1];	/* 1 for the number of fields */
}
