/*
 * Network_Management_Layer.h
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_H_
#define SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_H_

/* The C standard library */
#include <string.h>

/* Enums and structs */
#include "../../Open_SAE_J1939/Structs.h"
#include "../SAE_J1939_Enums/Enum_Control_Byte.h"
#include "../SAE_J1939_Enums/Enum_DM1_DM2.h"
#include "../SAE_J1939_Enums/Enum_DM14_DM15.h"
#include "../SAE_J1939_Enums/Enum_Group_Function_Value.h"
#include "../SAE_J1939_Enums/Enum_NAME.h"
#include "../SAE_J1939_Enums/Enum_PGN.h"
#include "../SAE_J1939_Enums/Enum_Send_Status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Address claimed */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Address_Claimed(J1939 *j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Address_Claimed(J1939 *j1939);
void SAE_J1939_Read_Response_Request_Address_Claimed(J1939 *j1939, uint8_t SA, uint8_t data[]);

/* Address not claimed */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Address_Not_Claimed(J1939 *j1939);
void SAE_J1939_Read_Address_Not_Claimed(J1939 *j1939, uint8_t SA, uint8_t data[]);

/* Commanded address */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Commanded_Address(J1939 *j1939, uint8_t DA, uint8_t new_ECU_address, uint32_t identity_number, uint16_t manufacturer_code, uint8_t function_instance, uint8_t ECU_instance, uint8_t function, uint8_t vehicle_system, uint8_t arbitrary_address_capable, uint8_t industry_group, uint8_t vehicle_system_instance);
void SAE_J1939_Read_Commanded_Address(J1939 *j1939, uint8_t data[]);

/* Delete address */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Address_Delete(J1939 *j1939, uint8_t DA, uint8_t old_ECU_address);
void SAE_J1939_Read_Address_Delete(J1939 *j1939, uint8_t data[]);

#ifdef __cplusplus
}
#endif

#endif /* SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_SAE_J1939_81_NETWORK_MANAGEMENT_LAYER_H_ */
