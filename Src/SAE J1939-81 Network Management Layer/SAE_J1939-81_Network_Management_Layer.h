/*
 * SAE_J1939-81_Network_Management_Layer.h
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_H_
#define SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_H_

/* C standard library */
#include "stdbool.h"

/* Layer */
#include "../SAE J1939-21 Transport Layer/SAE_J1939-21_Transport_Layer.h"

/* Enums */
#include "../SAE J1939 Enums/SAE_J1939_Enum_PGN.h"
#include "../SAE J1939 Enums/SAE_J1939_Enum_Control_Byte.h"
#include "../SAE J1939 Enums/SAE_J1939_Enum_Group_Function_Value.h"
#include "../SAE J1939 Enums/SAE_J1939_Enum_DM1.h"
#include "../SAE J1939 Enums/SAE_J1939_Enum_DM14_DM15.h"
#include "../SAE J1939 Enums/SAE_J1939_Enum_Send_Status.h"
#include "../SAE J1939 Enums/SAE_J1939_Enum_NAME.h"

/* Structs */
#include "../Open SAE J1939/Open_SAE_J1939_Structs.h"


/* Address claimed */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Address_Claimed(J1939* j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Address_Claimed(J1939* j1939);
void SAE_J1939_Read_Response_Request_Address_Claimed(J1939 *j1939, uint8_t SA, uint8_t data[]);

/* Commanded address */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Commanded_Address(J1939* j1939, uint8_t DA, uint8_t new_ECU_address, uint32_t identity_number, uint16_t manufacturer_code, uint8_t function_instance, uint8_t ECU_instance, uint8_t function, uint8_t vehicle_system, uint8_t arbitrary_address_capable, uint8_t industry_group, uint8_t vehicle_system_instance);

#endif /* SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_H_ */
