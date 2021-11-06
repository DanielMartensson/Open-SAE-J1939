/*
 * Address_Claimed.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Network_Management_Layer.h"

/* Layers */
#include "../SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"

/*
 * Send request address claimed to other ECU. Every time we asking addresses from other ECU, then we clear our storage of other ECU
 * PGN: 0x00EE00 (60928)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Address_Claimed(J1939 *j1939, uint8_t DA) {
	/* Delete all addresses by setting them to broadcast address and set the counters to 0 */
	memset(j1939->other_ECU_address, 0xFF, 0xFF);
	j1939->number_of_cannot_claim_address = 0;
	j1939->number_of_other_ECU = 0;
	return SAE_J1939_Send_Request(j1939, DA, PGN_ADDRESS_CLAIMED);
}

/*
 * Response the request address claimed about this ECU to all ECU - Broadcast. This function must be called at the ECU start up according to J1939 standard
 * PGN: 0x00EE00 (60928)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Address_Claimed(J1939 *j1939) {
	uint32_t ID = (0x18EEFF << 8) | j1939->information_this_ECU.this_ECU_address;
	uint8_t data[8];
	data[0] = j1939->information_this_ECU.this_name.identity_number;
	data[1] = j1939->information_this_ECU.this_name.identity_number >> 8;
	data[2] = (j1939->information_this_ECU.this_name.identity_number >> 16) |  (j1939->information_this_ECU.this_name.manufacturer_code << 5);
	data[3] = j1939->information_this_ECU.this_name.manufacturer_code >> 3;
	data[4] = (j1939->information_this_ECU.this_name.function_instance << 3) | j1939->information_this_ECU.this_name.ECU_instance;
	data[5] = j1939->information_this_ECU.this_name.function;
	data[6] = j1939->information_this_ECU.this_name.vehicle_system << 1;
	data[7] = (j1939->information_this_ECU.this_name.arbitrary_address_capable << 7) | (j1939->information_this_ECU.this_name.industry_group << 4) | j1939->information_this_ECU.this_name.vehicle_system_instance;
	return CAN_Send_Message(ID, data);
}

/*
 * Store the address claimed information about other ECU
 * PGN: 0x00EE00 (60928)
 */
void SAE_J1939_Read_Response_Request_Address_Claimed(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	/* Check if it's the same address */
	if(j1939->information_this_ECU.this_ECU_address == SA)
		SAE_J1939_Send_Address_Not_Claimed(j1939);

	/* If not, then store the temporary information */
	j1939->from_other_ecu_name.identity_number = ((data[2] & 0b00011111) << 16) | (data[1] << 8) | data[0];
	j1939->from_other_ecu_name.manufacturer_code = (data[3] << 3) | (data[2] >> 5);
	j1939->from_other_ecu_name.function_instance = data[4] >> 3;
	j1939->from_other_ecu_name.ECU_instance = data[4] & 0b00000111;
	j1939->from_other_ecu_name.function = data[5];
	j1939->from_other_ecu_name.vehicle_system = data[6] >> 1;
	j1939->from_other_ecu_name.arbitrary_address_capable = data[7] >> 7;
	j1939->from_other_ecu_name.industry_group = (data[7] >> 4) & 0b0111;
	j1939->from_other_ecu_name.vehicle_system_instance = data[7] & 0b00001111;
	j1939->from_other_ecu_name.from_ecu_address = SA;
	/* Remember the source address of the ECU */
	bool exist = false;
	for (uint8_t i = 0; i < j1939->number_of_other_ECU; i++)
		if (j1939->other_ECU_address[i] == SA)
			exist = true;
	if (!exist)
		j1939->other_ECU_address[j1939->number_of_other_ECU++] = SA;	/* For every new ECU address, count how many ECU */
}
