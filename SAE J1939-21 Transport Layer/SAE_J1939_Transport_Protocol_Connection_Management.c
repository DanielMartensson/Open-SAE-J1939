/*
 * SAE_J1939_Read_j1939_Transport_Protocol_Connection_Management.c
 *
 *  Created on: 14 juli 2021
 *      Author: danie
 */

#include "SAE_J1939-21_Transport_Layer.h"

/*
 * Store information about sequence data packages from other ECU who are going to send to this ECU
 * *j1939: Pointer to structure J1939
 * SA: Source ECU address between 0 to 255 (SA 255 = Broadcast from all ECU)
 * data[]: 8 bytes data array
 * PGN: 0x00EC00 (60416)
 */
void SAE_J1939_Read_Transport_Protocol_Connection_Management(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	j1939->tp_cm[SA].control_byte = data[0];
	j1939->tp_cm[SA].total_message_size = (data[2] << 8) | data[1];
	j1939->tp_cm[SA].number_of_packages = data[3];
	j1939->tp_cm[SA].PGN_of_the_packeted_message = (data[7] << 16) | (data[6] << 8) | data[5];
}

/*
 * Send information to other ECU about how much sequence data packages this ECU is going to send to other ECU
 * *j1939: Pointer to structure J1939
 * DA: Destination ECU address between 0 to 255 (DA 255 = Broadcast to all ECU)
 * control_byte: This indicates the how we are going to send the message. See SAE_J1939_Enum_Control_Byte.h
 * total_message_size: How many bytes we are going to send with Transport Protocol Data Transfer
 * number_of_packages: How many packages we want to send. Rule of thumb is if total_message_size % 8 > 1, then number_of_packages = total_message_size/8 + 1, else number_of_packages = total_message_size/8
 * PGN_of_the_packeted_message: What type of PGN message we are sending. See SAE_J1939_Enum_PGN.h
 * PGN: 0x00EC00 (60416)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Transport_Protocol_Connection_Management(J1939 *j1939, uint8_t DA, uint8_t control_byte, uint16_t total_message_size, uint8_t number_of_packages, uint32_t PGN_of_the_packeted_message) {
	uint32_t ID = (0x1CEC << 16) | (DA << 8) | j1939->this_address;
	uint8_t data[8];
	data[0] = control_byte;
	data[1] = total_message_size;
	data[2] = total_message_size >> 8;
	data[3] = number_of_packages;
	data[4] = 0xFF; 														/* Reserved */
	data[5] = PGN_of_the_packeted_message;
	data[6] = PGN_of_the_packeted_message >> 8;
	data[7] = PGN_of_the_packeted_message >> 16;
	return CAN_Send_Message(ID, data, 100);									/* 100 ms delay */
}
