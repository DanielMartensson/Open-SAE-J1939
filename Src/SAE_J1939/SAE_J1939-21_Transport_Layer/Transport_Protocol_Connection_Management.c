/*
 * SAE_J1939_Read_j1939_Transport_Protocol_Connection_Management.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel M�rtensson
 */

#include "../SAE_J1939-21_Transport_Layer/Transport_Layer.h"

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
	if(j1939->from_other_ecu_tp_cm.control_byte == CONTROL_BYTE_TP_CM_RTS)
		SAE_J1939_Send_Transport_Protocol_Connection_Management(j1939, SA, CONTROL_BYTE_TP_CM_CTS, j1939->from_other_ecu_tp_cm.total_message_size, j1939->from_other_ecu_tp_cm.number_of_packages, j1939->from_other_ecu_tp_cm.PGN_of_the_packeted_message);

}

/*
 * Send information to other ECU about how much sequence data packages this ECU is going to send to other ECU
 * PGN: 0x00EC00 (60416)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Transport_Protocol_Connection_Management(J1939 *j1939, uint8_t DA, uint8_t control_byte, uint16_t total_message_size, uint8_t number_of_packages, uint32_t PGN_of_the_packeted_message) {
	uint32_t ID = (0x1CEC << 16) | (DA << 8) | j1939->this_ECU_address;
	uint8_t data[8];
	data[0] = control_byte;
	data[1] = total_message_size;
	data[2] = total_message_size >> 8;
	data[3] = number_of_packages;
	data[4] = 0xFF; 															/* Reserved */
	data[5] = PGN_of_the_packeted_message;
	data[6] = PGN_of_the_packeted_message >> 8;
	data[7] = PGN_of_the_packeted_message >> 16;
	return CAN_Send_Message(ID, data, 100);										/* 100 ms delay */
}
