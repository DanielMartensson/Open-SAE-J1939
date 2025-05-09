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
	/* Read the control byte */
	j1939->from_other_ecu_tp_cm.control_byte = data[0];

	/* PGN */
	j1939->from_other_ecu_tp_cm.PGN_of_the_packeted_message = (data[7] << 16) | (data[6] << 8) | data[5];

	/* Source address */
	j1939->from_other_ecu_tp_cm.from_ecu_address = SA;

	/* Check the control byte */
	switch (data[0]) {
	case CONTROL_BYTE_TP_CM_RTS:
		/* Set the RTS values */
		j1939->from_other_ecu_tp_cm.total_message_size_being_transmitted = (data[2] << 8) | data[1];
		j1939->from_other_ecu_tp_cm.number_of_packages_being_transmitted = data[3];

		/* Send CTS */
		j1939->this_ecu_tp_cm.control_byte = CONTROL_BYTE_TP_CM_CTS;
		j1939->this_ecu_tp_cm.number_of_packets_to_be_transmitted = 1;
		j1939->this_ecu_tp_cm.next_packet_number_transmitted = 1;
		j1939->this_ecu_tp_cm.PGN_of_the_packeted_message = (data[7] << 16) | (data[6] << 8) | data[5];
		SAE_J1939_Send_Transport_Protocol_Connection_Management(j1939, SA);
		break;
	case CONTROL_BYTE_TP_CM_CTS:
		j1939->from_other_ecu_tp_cm.number_of_packets_to_be_transmitted = data[1];
		j1939->from_other_ecu_tp_cm.next_packet_number_transmitted = data[2];
		SAE_J1939_Send_Transport_Protocol_Data_Transfer(j1939, SA);
		break;
	case CONTROL_BYTE_TP_CM_BAM:
		j1939->from_other_ecu_tp_cm.total_message_size_being_transmitted = (data[2] << 8) | data[1];
		j1939->from_other_ecu_tp_cm.number_of_packages_being_transmitted = data[3];
		SAE_J1939_Send_Transport_Protocol_Data_Transfer(j1939, SA);
		break;
	case CONTROL_BYTE_TP_CM_EndOfMsgACK:
		j1939->from_other_ecu_tp_cm.total_number_of_bytes_received = (data[2] << 8) | data[1];
		j1939->from_other_ecu_tp_cm.total_number_of_packages_received = data[3];
	}
}

/*
 * Send information to other ECU about how much sequence data packages this ECU is going to send to other ECU
 * PGN: 0x00EC00 (60416)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Transport_Protocol_Connection_Management(J1939 *j1939, uint8_t DA) {
	uint32_t ID = (0x1CEC << 16) | (DA << 8) | j1939->information_this_ECU.this_ECU_address;
	uint8_t data[8] = { 0 };

	/* Read the control byte */
	data[0] = j1939->this_ecu_tp_cm.control_byte;

	/* Check the control byte */
	switch (data[0]) {
	case CONTROL_BYTE_TP_CM_RTS:
		data[1] = j1939->this_ecu_tp_cm.total_message_size_being_transmitted;
		data[2] = j1939->this_ecu_tp_cm.total_message_size_being_transmitted >> 8;
		data[3] = j1939->this_ecu_tp_cm.number_of_packages_being_transmitted;
		data[4] = 0x01; 															/* Max number of packages to be transmitted at once */
		break;
	case CONTROL_BYTE_TP_CM_CTS:
		data[1] = j1939->this_ecu_tp_cm.number_of_packets_to_be_transmitted;
		data[2] = j1939->this_ecu_tp_cm.next_packet_number_transmitted;
		data[3] = 0xFF; 															/* Reserved */
		data[4] = 0xFF; 															/* Reserved */
		break;
	case CONTROL_BYTE_TP_CM_BAM:
		data[1] = j1939->this_ecu_tp_cm.total_message_size_being_transmitted;
		data[2] = j1939->this_ecu_tp_cm.total_message_size_being_transmitted >> 8;
		data[3] = j1939->this_ecu_tp_cm.number_of_packages_being_transmitted;
		data[4] = 0xFF; 															/* Reserved */
		break;
	case CONTROL_BYTE_TP_CM_EndOfMsgACK:
		data[1] = j1939->this_ecu_tp_cm.total_number_of_bytes_received;
		data[2] = j1939->this_ecu_tp_cm.total_number_of_bytes_received >> 8;
		data[3] = j1939->this_ecu_tp_cm.total_number_of_packages_received;
		data[4] = 0xFF; 															/* Reserved */
	}
	
	/* PGN */
	data[5] = j1939->this_ecu_tp_cm.PGN_of_the_packeted_message;
	data[6] = j1939->this_ecu_tp_cm.PGN_of_the_packeted_message >> 8;
	data[7] = j1939->this_ecu_tp_cm.PGN_of_the_packeted_message >> 16;

	return CAN_Send_Message(ID, data);
}
