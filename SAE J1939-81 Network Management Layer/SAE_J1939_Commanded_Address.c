/*
 * SAE_J1939_Commanded_Address.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "SAE_J1939-81_Network_Management_Layer.h"

/*
 * Send commanded address to another ECU
 * PGN 0x00FED8 (65240)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Commanded_Address(J1939* j1939, uint8_t DA, uint8_t new_DA, uint32_t identity_number, uint16_t manufacturer_code, uint8_t function_instance, uint8_t ECU_instance, uint8_t function, uint8_t vehicle_system, uint8_t arbitrary_address_capable, uint8_t industry_group, uint8_t vehicle_system_instance) {
	/* Multiple messages - Use Transport Protocol Connection Management BAM */
	uint8_t number_of_packages = 2;
	uint8_t total_message_size = 0;
	uint8_t data[9];
	data[total_message_size++] = identity_number;
	data[total_message_size++] = identity_number >> 8;
	data[total_message_size++] = (identity_number >> 16) |  (manufacturer_code << 5);
	data[total_message_size++] = manufacturer_code >> 3;
	data[total_message_size++] = (function_instance << 3) | ECU_instance;
	data[total_message_size++] = function;
	data[total_message_size++] = vehicle_system << 1;
	data[total_message_size++] = (arbitrary_address_capable << 7) | (industry_group << 4) | vehicle_system_instance;
	data[total_message_size++] = new_DA;									/* New address of the ECU we are sending to*/
	ENUM_J1939_STATUS_CODES status = J1939_Core_Send_TP_CM(DA, j1939->this_address, CONTROL_BYTE_TP_CM_BAM, total_message_size, number_of_packages, PGN_COMMANDED_ADDRESS);
	if(status != J1939_OK)
		return status;
	status = J1939_Core_Send_TP_DT(DA, j1939->this_address, data, total_message_size, number_of_packages);
	if(status == J1939_OK){
		/* Delete old name and old destination address */
		memset(&j1939->name[DA], 0, sizeof(j1939->name[DA]));
		j1939->addresses_ECU[DA] = 0;
		// TODO: Leta fram en PGN som kan nollställa Address Claimed hos andra ECU
		/* According to J1939 standard, request for Address Claimed (broadcast) must be sent after a Commanded Address so we can get back the new name and address */
		J1939_User_Send_Request(j1939, new_DA, PGN_ADDRESS_CLAIMED);
	}
	return status;
}

