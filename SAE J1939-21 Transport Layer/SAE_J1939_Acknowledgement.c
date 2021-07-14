/*
 * SAE_J1939_Acknowledgement.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "SAE_J1939-21_Transport_Layer.h"

/*
 * Store acknowledgement information from other ECU about PGN request to other ECU
 * *j1939: Pointer to structure J1939
 * SA: Source ECU address between 0 to 255 (SA 255 = Broadcast from all ECU)
 * data[]: 8 bytes data array
 * PGN: 0x00E800 (59392)
 */
void SAE_J1939_Read_Acknowledgement(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	j1939->acknowledgement[SA].control_byte = data[0];
	j1939->acknowledgement[SA].group_function_value = data[1];
	j1939->acknowledgement[SA].address = data[4]; 							/* The source address from the ECU */
	j1939->acknowledgement[SA].PGN_of_requested_info = (data[7] << 16) | (data[6] << 8) | data[5];
}

/*
 * Send back a acknowledgement to other ECU about the their PGN request
 * *j1939: Pointer to structure J1939
 * DA: Destination ECU address between 0 to 255 (DA 255 = Broadcast to all ECU)
 * control_byte: This indicates the status of the requested information about PGN. See SAE_J1939_Enum_Control_Byte.h
 * group_function_value: The ECU function between 0 to 255
 * address: This is the same as ECU source address between 0 to 254
 * PGN_of_requested_info: This is the PGN request from other ECU
 * PGN: 0x00E800 (59392)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Acknowledgement(J1939 *j1939, uint8_t DA, uint8_t control_byte, uint8_t group_function_value, uint8_t address, uint32_t PGN_of_requested_info) {
	uint32_t ID = (0x18E8 << 16) | (DA << 8) | j1939->this_address;
	uint8_t data[8];
	data[0] = control_byte;
	data[1] = group_function_value;
	data[2] = 0xFF;															/* Reserved */
	data[3] = 0xFF;															/* Reserved */
	data[4] = address;														/* This ECU address */
	data[5] = PGN_of_requested_info;
	data[6] = PGN_of_requested_info >> 8;
	data[7] = PGN_of_requested_info >> 16;
	return CAN_Send_Message(ID, data, 0);									/* 0 ms delay */
}
