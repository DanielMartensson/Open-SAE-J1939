/*
 * Transport_Protocol_Connection_Management.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Transport_Layer.h"

/*
 * Store information about sequence data packages from other ECU who are going to send to this ECU
 * PGN: 0x00EC00 (60416)
 */
void SAE_J1939_Read_Transport_Protocol_Connection_Management(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	j1939->from_other_ecu_tp_cm.control_byte = data[0];
	j1939->from_other_ecu_tp_cm.total_message_size = (data[2] << 8) | data[1];
	j1939->from_other_ecu_tp_cm.number_of_packages = data[3];
	j1939->from_other_ecu_tp_cm.PGN_of_the_packeted_message = (data[7] << 16) | (data[6] << 8) | data[5];
	j1939->from_other_ecu_tp_cm.from_ecu_address = SA;

	/* Check if we got the Request To Send control byte - We need to answer with CTS - Clear To Send */
	if(j1939->from_other_ecu_tp_cm.control_byte == CONTROL_BYTE_TP_CM_RTS){
		j1939->this_ecu_tp_cm = j1939->from_other_ecu_tp_cm; 			/* Copy - We need to have the same data */
		j1939->this_ecu_tp_cm.control_byte = CONTROL_BYTE_TP_CM_CTS;	/* We only need to change the control byte from RTS to CTS */
		SAE_J1939_Send_Transport_Protocol_Connection_Management(j1939, SA);
	}

	/* When we answer with CTS, it means we are going to send the Transport Protocol Data Transfer package */
	if(j1939->from_other_ecu_tp_cm.control_byte == CONTROL_BYTE_TP_CM_CTS)
		SAE_J1939_Send_Transport_Protocol_Data_Transfer(j1939, SA);
}

/*
 * Send information to other ECU about how much sequence data packages this ECU is going to send to other ECU
 * PGN: 0x00EC00 (60416)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Transport_Protocol_Connection_Management(J1939 *j1939, uint8_t DA) {
	uint32_t ID = (0x1CEC << 16) | (DA << 8) | j1939->information_this_ECU.this_ECU_address;
	uint8_t data[8];
	data[0] = j1939->this_ecu_tp_cm.control_byte;
	data[1] = j1939->this_ecu_tp_cm.total_message_size;
	data[2] = j1939->this_ecu_tp_cm.total_message_size >> 8;
	data[3] = j1939->this_ecu_tp_cm.number_of_packages;
	data[4] = 0xFF; 															/* Reserved */
	data[5] = j1939->this_ecu_tp_cm.PGN_of_the_packeted_message;
	data[6] = j1939->this_ecu_tp_cm.PGN_of_the_packeted_message >> 8;
	data[7] = j1939->this_ecu_tp_cm.PGN_of_the_packeted_message >> 16;
	return CAN_Send_Message(ID, data);
}
