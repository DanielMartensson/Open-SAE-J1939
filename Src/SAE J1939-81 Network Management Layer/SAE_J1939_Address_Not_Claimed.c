/*
 * SAE_J1939_Address_Not_Claimed.c
 *
 *  Created on: 19 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "SAE_J1939-81_Network_Management_Layer.h"

/*
 * Send Address Not Claimed if the address conflicts with each other
 * PGN: 0x00EE00 (60928)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Address_Not_Claimed(J1939 *j1939) {
	uint32_t ID = 0x18EEFFFE;
	uint8_t data[8];
	data[0] = j1939->this_name.identity_number;
	data[1] = j1939->this_name.identity_number >> 8;
	data[2] = (j1939->this_name.identity_number >> 16) |  (j1939->this_name.manufacturer_code << 5);
	data[3] = j1939->this_name.manufacturer_code >> 3;
	data[4] = (j1939->this_name.function_instance << 3) | j1939->this_name.ECU_instance;
	data[5] = j1939->this_name.function;
	data[6] = j1939->this_name.vehicle_system << 1;
	data[7] = (j1939->this_name.arbitrary_address_capable << 7) | (j1939->this_name.industry_group << 4) | j1939->this_name.vehicle_system_instance;
	return CAN_Send_Message(ID, data, 0);							/* 0 ms delay */
}

/*
 * Store the address not claimed information about other ECU. In this case, SA will always be 0xFE = 254
 * PGN: 0x00EE00 (60928)
 */
void SAE_J1939_Read_Address_Not_Claimed(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	j1939->all_name[SA].identity_number = ((data[2] & 0b00011111) << 16) | (data[1] << 8) | data[0];
	j1939->all_name[SA].manufacturer_code = (data[3] << 3) | (data[2] >> 5);
	j1939->all_name[SA].function_instance = data[4] >> 3;
	j1939->all_name[SA].ECU_instance = data[4] & 0b00000111;
	j1939->all_name[SA].function = data[5];
	j1939->all_name[SA].vehicle_system = data[6] >> 1;
	j1939->all_name[SA].arbitrary_address_capable = data[7] >> 7;
	j1939->all_name[SA].industry_group = (data[7] >> 4) & 0b0111;
	j1939->all_name[SA].vehicle_system_instance = data[7] & 0b00001111;
	j1939->all_number_of_cannot_claim_address++;
}

