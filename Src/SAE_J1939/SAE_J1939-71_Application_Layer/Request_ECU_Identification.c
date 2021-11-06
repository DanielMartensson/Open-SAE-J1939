/*
 * Request_ECU_Identification.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Application_Layer.h"

/* Layers */
#include "../SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"

/*
 * Request ECU identification to another ECU
 * PGN: 0x00FDC5 (64965)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_ECU_Identification(J1939 *j1939, uint8_t DA) {
	return SAE_J1939_Send_Request(j1939, DA, PGN_ECU_IDENTIFICATION);
}

/*
 * Response the request of the ECU identification about this ECU
 * PGN: 0x00FDC5 (64965)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_ECU_Identification(J1939* j1939, uint8_t DA) {
	/* Find the length of the array fields */
	uint8_t length_of_each_field = j1939->information_this_ECU.this_identifications.ecu_identification.length_of_each_field;
	if (length_of_each_field < 2) {
		/* If each field have the length 1, then we can send ECU identification as it was a normal message */
		uint32_t ID = (0x18FDC5 << 8) | j1939->information_this_ECU.this_ECU_address;
		uint8_t data[8];
		data[0] = j1939->information_this_ECU.this_identifications.ecu_identification.ecu_part_number[0];
		data[1] = j1939->information_this_ECU.this_identifications.ecu_identification.ecu_serial_number[0];
		data[2] = j1939->information_this_ECU.this_identifications.ecu_identification.ecu_location[0];
		data[3] = j1939->information_this_ECU.this_identifications.ecu_identification.ecu_type[0];
		data[4] = 0xFF;													 /* Reserved */
		data[5] = 0xFF;													 /* Reserved */
		data[6] = 0xFF;													 /* Reserved */
		data[7] = 0xFF;													 /* Reserved */
		return CAN_Send_Message(ID, data);
	} else {
		/* Multiple messages - Load data */
		j1939->this_ecu_tp_cm.total_message_size = 0;
		for(uint8_t i = 0; i < length_of_each_field; i++) {
			j1939->this_ecu_tp_dt.data[i] = j1939->information_this_ECU.this_identifications.ecu_identification.ecu_part_number[i];
			j1939->this_ecu_tp_dt.data[length_of_each_field + i] = j1939->information_this_ECU.this_identifications.ecu_identification.ecu_serial_number[i];
			j1939->this_ecu_tp_dt.data[length_of_each_field*2 + i] = j1939->information_this_ECU.this_identifications.ecu_identification.ecu_location[i];
			j1939->this_ecu_tp_dt.data[length_of_each_field*3 + i] = j1939->information_this_ECU.this_identifications.ecu_identification.ecu_type[i];
			j1939->this_ecu_tp_cm.total_message_size += 4;
		}

		/* Send TP CM */
		j1939->this_ecu_tp_cm.number_of_packages = j1939->this_ecu_tp_cm.total_message_size % 8 > 0 ? j1939->this_ecu_tp_cm.total_message_size/8 + 1 : j1939->this_ecu_tp_cm.total_message_size/8; /* Rounding up */
		j1939->this_ecu_tp_cm.PGN_of_the_packeted_message = PGN_ECU_IDENTIFICATION;
		j1939->this_ecu_tp_cm.control_byte = DA == 0xFF ? CONTROL_BYTE_TP_CM_BAM : CONTROL_BYTE_TP_CM_RTS; /* If broadcast, then use BAM control byte */
		ENUM_J1939_STATUS_CODES status = SAE_J1939_Send_Transport_Protocol_Connection_Management(j1939, DA);
		if(status != STATUS_SEND_OK)
			return status;

		/* Check if we are going to send it directly (BAM) - Else, the TP CM will send a RTS control byte to the other ECU and the ECU will answer with control byte CTS */
		if(j1939->this_ecu_tp_cm.control_byte == CONTROL_BYTE_TP_CM_BAM)
			return SAE_J1939_Send_Transport_Protocol_Data_Transfer(j1939, DA);
		return status;
	}
}

/*
 * Store the ECU identification about other ECU
 * PGN: 0x00FDC5 (64965)
 */
void SAE_J1939_Read_Response_Request_ECU_Identification(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	/* ECU identification have 6 fixed fields in the J1939 struct */
	uint8_t length_of_each_field = j1939->from_other_ecu_identifications.ecu_identification.length_of_each_field;
	for(uint8_t i = 0; i < length_of_each_field; i++) {
		j1939->from_other_ecu_identifications.ecu_identification.ecu_part_number[i] = data[i];
		j1939->from_other_ecu_identifications.ecu_identification.ecu_serial_number[i] = data[i + length_of_each_field];
		j1939->from_other_ecu_identifications.ecu_identification.ecu_location[i] = data[i + length_of_each_field*2];
		j1939->from_other_ecu_identifications.ecu_identification.ecu_type[i] = data[i + length_of_each_field*3];
	}
	j1939->from_other_ecu_identifications.ecu_identification.from_ecu_address = SA;
}
