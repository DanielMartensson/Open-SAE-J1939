/*
 * SAE_J1939_Address_Claimed.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "SAE_J1939-81_Network_Management_Layer.h"

/*
 * Send request address claimed to another ECU
 * PGN 0x00EE00 (60928)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Address_Claimed(J1939* j1939, uint8_t DA) {
	return SAE_J1939_Send_Request(j1939, DA, PGN_ADDRESS_CLAIMED);
}

/*
 * Response the request address claimed about this ECU
 * PGN 0x00EE00 (60928)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Address_Claimed(J1939* j1939, uint8_t DA) {
	uint32_t ID = (0x18EE << 16) | (DA << 8) | j1939->this_ECU_address;
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
 * Store the address claimed information about other ECU
 * PGN 0x00EE00 (60928)
 */
void SAE_J1939_Read_Response_Request_Address_Claimed(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	j1939->name[SA].identity_number = ((data[2] & 0b00011111) << 16) | (data[1] << 8) | data[0];
	j1939->name[SA].manufacturer_code = (data[3] << 3) | (data[2] >> 5);
	j1939->name[SA].function_instance = data[4] >> 3;
	j1939->name[SA].ECU_instance = data[4] & 0b00000111;
	j1939->name[SA].function = data[5];
	j1939->name[SA].vehicle_system = data[6] >> 1;
	j1939->name[SA].arbitrary_address_capable = data[7] >> 7;
	j1939->name[SA].industry_group = (data[7] >> 4) & 0b0111;
	j1939->name[SA].vehicle_system_instance = data[7] & 0b00001111;
	/* Remember the source address of the ECU */
	bool exist = false;
	for (uint8_t i = 0; i < 256; i++)
		if (j1939->ECU_address[i] == SA)
			exist = true;
	if (!exist) {
		j1939->ECU_address[j1939->number_of_ECU] = SA;
		j1939->number_of_ECU++;
	}
}
