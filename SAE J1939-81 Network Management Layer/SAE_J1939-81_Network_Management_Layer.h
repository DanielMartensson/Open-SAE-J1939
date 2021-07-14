/*
 * SAE_J1939-81_Network_Management_Layer.h
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_H_
#define SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_H_

enum NAME_codes {
	HEJ = 0x0 // TODO: Fyll på med t.ex INDUSTRY_GRUOP_0 etc
}ENUM_NAME_CODES;

/* Address claimed */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Address_Claimed(J1939* j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Address_Claimed(J1939* j1939, uint8_t DA);
void SAE_J1939_Read_Response_Request_Address_Claimed(J1939 *j1939, uint8_t SA, uint8_t data[]);

/* Commanded address */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Commanded_Address(J1939* j1939, uint8_t DA, uint8_t new_DA, uint32_t identity_number, uint16_t manufacturer_code, uint8_t function_instance, uint8_t ECU_instance, uint8_t function, uint8_t vehicle_system, uint8_t arbitrary_address_capable, uint8_t industry_group, uint8_t vehicle_system_instance);

#endif /* SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_H_ */
