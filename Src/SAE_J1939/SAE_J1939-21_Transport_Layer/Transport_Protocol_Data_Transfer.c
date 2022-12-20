/*
 * Transport_Protocol_Data_Transfer.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Transport_Layer.h"

/* Layers */
#include "../SAE_J1939-81_Network_Management_Layer/Network_Management_Layer.h"
#include "../SAE_J1939-73_Diagnostics_Layer/Diagnostics_Layer.h"
#include "../SAE_J1939-71_Application_Layer/Application_Layer.h"

/*
 * Store the sequence data packages from other ECU
 * PGN: 0x00EB00 (60160)
 */
void SAE_J1939_Read_Transport_Protocol_Data_Transfer(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	/* Save the sequence data */
	j1939->from_other_ecu_tp_dt.sequence_number = data[0];
	j1939->from_other_ecu_tp_dt.from_ecu_address = SA;
	uint8_t index = data[0] - 1;
	for (uint8_t i = 1; i < 8; i++)
		j1939->from_other_ecu_tp_dt.data[index*7 + i-1] = data[i]; /* For every package, we send 7 bytes of data where the first byte data[0] is the sequence number */

	/* Check if we have completed our message - Return = Not completed */
	if (j1939->from_other_ecu_tp_cm.number_of_packages != j1939->from_other_ecu_tp_dt.sequence_number || j1939->from_other_ecu_tp_cm.number_of_packages == 0)
		return;

	/* Our message are complete - Build it and call it complete_data[total_message_size] */
	uint32_t PGN = j1939->from_other_ecu_tp_cm.PGN_of_the_packeted_message;
	uint16_t total_message_size = j1939->from_other_ecu_tp_cm.total_message_size;
	uint8_t complete_data[MAX_TP_DT];
	uint16_t inserted_bytes = 0;
	for (uint8_t i = 0; i < j1939->from_other_ecu_tp_dt.sequence_number; i++)
		for (uint8_t j = 0; j < 7; j++)
			if (inserted_bytes < total_message_size)
				complete_data[inserted_bytes++] = j1939->from_other_ecu_tp_dt.data[i*7 + j];

	/* Send an end of message ACK back */
	if(j1939->from_other_ecu_tp_cm.control_byte == CONTROL_BYTE_TP_CM_RTS)
		SAE_J1939_Send_Acknowledgement(j1939, SA, CONTROL_BYTE_TP_CM_EndOfMsgACK, GROUP_FUNCTION_VALUE_NORMAL, PGN);

	/* Check what type of function that message want this ECU to do */
	switch (PGN) {
	case PGN_COMMANDED_ADDRESS:
		SAE_J1939_Read_Commanded_Address(j1939, complete_data);								/* Insert new name and new address to this ECU */
		break;
	case PGN_DM1:
		SAE_J1939_Read_Response_Request_DM1(j1939, SA, complete_data, (total_message_size-2)/4); 	/* Number of DTCs = 4 bytes per DTC excluding 2 bytes for the lamp */
		break;
	case PGN_DM2:
		SAE_J1939_Read_Response_Request_DM2(j1939, SA, complete_data, (total_message_size-2)/4); 	/* Number of DTCs = 4 bytes per DTC excluding 2 bytes for the lamp */
		break;
	case PGN_DM16:
		SAE_J1939_Read_Binary_Data_Transfer_DM16(j1939, SA, complete_data);
		break;
	case PGN_SOFTWARE_IDENTIFICATION:
		SAE_J1939_Read_Response_Request_Software_Identification(j1939, SA, complete_data);
		break;
	case PGN_ECU_IDENTIFICATION:
		SAE_J1939_Read_Response_Request_ECU_Identification(j1939, SA, complete_data);
		break;
	case PGN_COMPONENT_IDENTIFICATION:
		SAE_J1939_Read_Response_Request_Component_Identification(j1939, SA, complete_data);
		break;
		/* Add more here */
	}

	/* Delete TP DT and TP CM */
	memset(&j1939->from_other_ecu_tp_dt, 0, sizeof(j1939->from_other_ecu_tp_dt));
	memset(&j1939->from_other_ecu_tp_cm, 0, sizeof(j1939->from_other_ecu_tp_cm));
}

/*
 * Send sequence data packages to other ECU that we have loaded
 * PGN: 0x00EB00 (60160)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Transport_Protocol_Data_Transfer(J1939 *j1939, uint8_t DA){
	uint32_t ID = (0x1CEB << 16) | (DA << 8) | j1939->information_this_ECU.this_ECU_address;
	uint8_t package[8];
	uint16_t bytes_sent = 0;
	ENUM_J1939_STATUS_CODES status = STATUS_SEND_OK;
	for(uint8_t i = 1; i <= j1939->this_ecu_tp_cm.number_of_packages; i++) {
		package[0] = i; 																	/* Number of package */
		for(uint8_t j = 0; j < 7; j++)
			if(bytes_sent < j1939->this_ecu_tp_cm.total_message_size)
				package[j+1] = j1939->this_ecu_tp_dt.data[bytes_sent++];					/* Data that we have collected */
			 else
				package[j+1] = 0xFF; 														/* Reserved */

		status = CAN_Send_Message(ID, package);
		CAN_Delay(100);																		/* Important CAN delay according to standard */
		if(status != STATUS_SEND_OK)
			return status;
	}
	return status;
}
