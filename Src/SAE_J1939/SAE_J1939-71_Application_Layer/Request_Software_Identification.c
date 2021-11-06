/*
 * Request_Software_Identification.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Application_Layer.h"

/* Layers */
#include "../SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"

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
	uint8_t number_of_fields = j1939->information_this_ECU.this_identifications.software_identification.number_of_fields;
	if (number_of_fields < 9) {
		uint32_t ID = (0x18FEDA << 8) | j1939->information_this_ECU.this_ECU_address;
		uint8_t data[8];
		data[0] = number_of_fields;
		for(uint8_t i = 0; i < 7; i++)
			data[i+1] = j1939->information_this_ECU.this_identifications.software_identification.identifications[i];
		return CAN_Send_Message(ID, data);
	} else {
		/* Multiple messages - Load data */
		j1939->this_ecu_tp_cm.total_message_size = 0;
		j1939->this_ecu_tp_dt.data[j1939->this_ecu_tp_cm.total_message_size++] = number_of_fields;
		for(uint8_t i = 0; i < number_of_fields; i++)
			j1939->this_ecu_tp_dt.data[j1939->this_ecu_tp_cm.total_message_size++] = j1939->information_this_ECU.this_identifications.software_identification.identifications[i];

		/* Send TP CM */
		j1939->this_ecu_tp_cm.number_of_packages = j1939->this_ecu_tp_cm.total_message_size % 8 > 0 ? j1939->this_ecu_tp_cm.total_message_size/8 + 1 : j1939->this_ecu_tp_cm.total_message_size/8; /* Rounding up */
		j1939->this_ecu_tp_cm.PGN_of_the_packeted_message = PGN_SOFTWARE_IDENTIFICATION;
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
 * Store the software identification about other ECU
 * PGN: 0x00FEDA (65242)
 */
void SAE_J1939_Read_Response_Request_Software_Identification(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	j1939->from_other_ecu_identifications.software_identification.number_of_fields = data[0];			 /* How many fields we have */
	j1939->from_other_ecu_identifications.software_identification.from_ecu_address = SA;
	for(uint8_t i = 0; i < data[0]; i++)
		j1939->from_other_ecu_identifications.software_identification.identifications[i] = data[i+1];	 /* 1 for the number of fields */
}
