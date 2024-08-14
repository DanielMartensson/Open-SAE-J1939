/*
 * Diagnostics_Layer.h
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef SAE_J1939_73_DIAGNOSTICS_LAYER_SAE_J1939_73_DIAGNOSTICS_LAYER_H_
#define SAE_J1939_73_DIAGNOSTICS_LAYER_SAE_J1939_73_DIAGNOSTICS_LAYER_H_

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

/* DM1 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_DM1(J1939 *j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_DM1(J1939* j1939, uint8_t DA);
void SAE_J1939_Read_Response_Request_DM1(J1939 *j1939, uint8_t SA, uint8_t data[], uint8_t errors_dm1_active);

/* DM2 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_DM2(J1939 *j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_DM2(J1939 *j1939, uint8_t DA);
void SAE_J1939_Read_Response_Request_DM2(J1939 *j1939, uint8_t SA, uint8_t data[], uint8_t errors_dm1_active);

/* DM3 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_DM3(J1939 *j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_DM3(J1939 *j1939, uint8_t DA);

/* DM14 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_DM14(J1939 *j1939, uint8_t DA, uint16_t number_of_requested_bytes, uint8_t pointer_type, uint8_t command, uint32_t pointer, uint8_t pointer_extension, uint16_t key);
ENUM_J1939_STATUS_CODES SAE_J1939_Read_Request_DM14(J1939 *j1939, uint8_t DA, uint8_t data[]);

/* DM15 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Response_DM15(J1939 *j1939, uint8_t DA, uint16_t number_of_allowed_bytes, uint8_t status, uint32_t EDC_parameter, uint8_t EDCP_extention, uint16_t seed);
void SAE_J1939_Read_Response_DM15(J1939 *j1939, uint8_t SA, uint8_t data[]);

/* DM16 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Binary_Data_Transfer_DM16(J1939 *j1939, uint8_t DA, const uint8_t number_of_occurences, uint8_t raw_binary_data[]);
void SAE_J1939_Read_Binary_Data_Transfer_DM16(J1939 *j1939, uint8_t SA, uint8_t data[]);

#ifdef __cplusplus
}
#endif

#endif /* SAE_J1939_73_DIAGNOSTICS_LAYER_SAE_J1939_73_DIAGNOSTICS_LAYER_H_ */
