/*
 * SAE_J1939_Request_Component_Identification.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "SAE_J1939-71_Application_Layer.h"

/*
 * Request component identification to another ECU
 * PGN: 0x00FEEB (65259)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Component_Identification(J1939 *j1939, uint8_t DA) {
	return SAE_J1939_Send_Request(j1939, DA, PGN_COMPONENT_IDENTIFICATION);
}

/*
 * Response the request of the component identification about this ECU
 * PGN: 0x00FEEB (65259)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Component_Identification(J1939* j1939, uint8_t DA){
	/* Find the length of the array fields */
	uint8_t length_of_each_field = j1939->this_component_identification.length_of_each_field;
	if (length_of_each_field < 2) {
		/* If each field have the length 1, then we can send component identification as it was a normal message */
		uint32_t ID = (0x18FEEB << 8) | j1939->this_ECU_address;
		uint8_t data[8];
		data[0] = j1939->this_component_identification.component_product_date[0];
		data[1] = j1939->this_component_identification.component_model_name[0];
		data[2] = j1939->this_component_identification.component_serial_number[0];
		data[3] = j1939->this_component_identification.component_unit_name[0];
		data[4] = 0xFF;													/* Reserved */
		data[5] = 0xFF;													/* Reserved */
		data[6] = 0xFF;													/* Reserved */
		data[7] = 0xFF;													/* Reserved */
		return CAN_Send_Message(ID, data, 0);							/* 0 ms delay */
	} else {
		/* Multiple messages - Use Transport Protocol Connection Management BAM */
		uint16_t total_message_size = 0;
		uint8_t data[length_of_each_field*4];							/* Total 4 fields */
		for(uint8_t i = 0; i < length_of_each_field; i++) {
			data[total_message_size++] = j1939->this_component_identification.component_product_date[i];
			data[total_message_size++] = j1939->this_component_identification.component_model_name[i + length_of_each_field];
			data[total_message_size++] = j1939->this_component_identification.component_serial_number[i + length_of_each_field*2];
			data[total_message_size++] = j1939->this_component_identification.component_unit_name[i + length_of_each_field*3];
		}

		/* Send TP CM BAM and then TP DT data */
		uint8_t number_of_packages = total_message_size % 8 > 1 ? total_message_size/8 + 1 : total_message_size/8; /* Rounding up */
		ENUM_J1939_STATUS_CODES status = SAE_J1939_Send_Transport_Protocol_Connection_Management(j1939, DA, CONTROL_BYTE_TP_CM_BAM, total_message_size, number_of_packages, PGN_COMPONENT_IDENTIFICATION);
		if(status != STATUS_SEND_OK)
			return status;
		return SAE_J1939_Send_Transport_Protocol_Data_Transfer(j1939, DA, data, total_message_size, number_of_packages);
	}
}

/*
 * Store the component identification about other ECU
 * PGN: 0x00FEEB (65259)
 */
void SAE_J1939_Read_Response_Request_Component_Identification(J1939 *j1939, uint8_t data[]) {
	/* Component identification have 4 fixed fields in the J1939 struct */
	uint8_t length_of_each_field = j1939->component_identification.length_of_each_field;
	for(uint8_t i = 0; i < length_of_each_field; i++) {
		j1939->component_identification.component_product_date[i] = data[i];
		j1939->component_identification.component_model_name[i] = data[i + length_of_each_field];
		j1939->component_identification.component_serial_number[i] = data[i + length_of_each_field*2];
		j1939->component_identification.component_unit_name[i] = data[i + length_of_each_field*3];
	}
}

