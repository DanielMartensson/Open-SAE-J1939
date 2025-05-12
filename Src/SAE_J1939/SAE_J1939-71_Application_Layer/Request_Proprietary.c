/*
 * Request_Proprietary.c
 *
 *  Created on: 25 December 2023
 *      Author: Daniel M�rtensson
 */

#include "Application_Layer.h"

 /* Layers */
#include "../SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"

/*
 * Request Proprietary A to another ECU
 * PGN: 0x00EF00 (61184)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Proprietary_A(J1939* j1939, uint8_t DA) {
	return SAE_J1939_Send_Request(j1939, DA, PGN_PROPRIETARY_A);
}

/*
 * Response the request Proprietary A about this ECU
 * PGN: 0x00EF00 (61184)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Proprietary_A(J1939* j1939, uint8_t DA) {
	/* Find the length of the array fields */
	uint16_t length_of_each_field = j1939->this_proprietary.proprietary_A.total_bytes;
	if (length_of_each_field < 9) {
		/* If each field have the length 8 or less, then we can send Proprietary A as it was a normal message */
		uint32_t ID = (0x14EF23 << 8) | j1939->information_this_ECU.this_ECU_address;
		uint8_t data[8];
		memcpy(data, j1939->this_proprietary.proprietary_A.data, length_of_each_field);
		return CAN_Send_Message(ID, data);
	}
	else {
		/* Multiple messages - Load data */
		j1939->this_ecu_tp_cm.total_message_size_being_transmitted = length_of_each_field;
		memcpy(j1939->this_ecu_tp_dt.data, j1939->this_proprietary.proprietary_A.data, length_of_each_field);
		
		/* Send TP CM */
		j1939->this_ecu_tp_cm.number_of_packages_being_transmitted = SAE_J1939_Transport_Protocol_GetNumberOfPackages(j1939->this_ecu_tp_cm.total_message_size_being_transmitted);
		j1939->this_ecu_tp_cm.PGN_of_the_packeted_message = PGN_PROPRIETARY_A;
		j1939->this_ecu_tp_cm.control_byte = DA == 0xFF ? CONTROL_BYTE_TP_CM_BAM : CONTROL_BYTE_TP_CM_RTS; /* If broadcast, then use BAM control byte */
		ENUM_J1939_STATUS_CODES status = SAE_J1939_Send_Transport_Protocol_Connection_Management(j1939, DA);
		if (status != STATUS_SEND_OK) {
			return status;
		}

		/* Check if we are going to send it directly (BAM) */
		if (j1939->this_ecu_tp_cm.control_byte == CONTROL_BYTE_TP_CM_BAM) {
			j1939->from_other_ecu_tp_cm.control_byte = j1939->this_ecu_tp_cm.control_byte;
			return SAE_J1939_Send_Transport_Protocol_Data_Transfer(j1939, DA);
		}
		return status;
	}
}

/*
 * Store the Proprietary A about other ECU
 * PGN: 0x00EF00 (61184)
 */
void SAE_J1939_Read_Response_Request_Proprietary_A(J1939* j1939, uint8_t SA, uint8_t data[]) {
	/* Proprietary A have 1 fixed field in the J1939 struct */
	uint16_t total_bytes = j1939->from_other_ecu_proprietary.proprietary_A.total_bytes;
	memcpy(j1939->from_other_ecu_proprietary.proprietary_A.data, data, total_bytes);
	j1939->from_other_ecu_proprietary.proprietary_A.from_ecu_address = SA;
}
