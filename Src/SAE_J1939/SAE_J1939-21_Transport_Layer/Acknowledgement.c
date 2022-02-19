/*
 * Acknowledgement.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Transport_Layer.h"

/*
 * Store acknowledgement information from other ECU about PGN request to other ECU
 * PGN: 0x00E800 (59392)
 */
void SAE_J1939_Read_Acknowledgement(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	j1939->from_other_ecu_acknowledgement.control_byte = data[0];
	j1939->from_other_ecu_acknowledgement.group_function_value = data[1];				/* The cause of the control byte */
	j1939->from_other_ecu_acknowledgement.address = data[4]; 							/* The source address from the ECU */
	j1939->from_other_ecu_acknowledgement.PGN_of_requested_info = (data[7] << 16) | (data[6] << 8) | data[5];
	j1939->from_other_ecu_acknowledgement.from_ecu_address = SA;						/* From where came the message */
}

/*
 * Send back a acknowledgement to other ECU about the their PGN request
 * PGN: 0x00E800 (59392)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Acknowledgement(J1939 *j1939, uint8_t DA, uint8_t control_byte, uint8_t group_function_value, uint32_t PGN_of_requested_info) {
	uint32_t ID = (0x18E8 << 16) | (DA << 8) | j1939->information_this_ECU.this_ECU_address;
	uint8_t data[8];
	data[0] = control_byte;
	data[1] = group_function_value;														/* The cause of the control byte */
	data[2] = 0xFF;																		/* Reserved */
	data[3] = 0xFF;																		/* Reserved */
	data[4] = j1939->information_this_ECU.this_ECU_address;													/* This source address */
	data[5] = PGN_of_requested_info;
	data[6] = PGN_of_requested_info >> 8;
	data[7] = PGN_of_requested_info >> 16;
	return CAN_Send_Message(ID, data);
}
