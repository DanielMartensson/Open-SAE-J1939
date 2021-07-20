/*
 * J1939-71_Application_Layer.h
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef SAE_J1939_71_APPLICATION_LAYER_SAE_J1939_71_APPLICATION_LAYER_H_
#define SAE_J1939_71_APPLICATION_LAYER_SAE_J1939_71_APPLICATION_LAYER_H_

/* Layers */
#include "../SAE J1939-21 Transport Layer/Transport_Layer.h"

/* Enums */
#include "../SAE J1939 Enums/Enum_Control_Byte.h"
#include "../SAE J1939 Enums/Enum_Control_Byte.h"
#include "../SAE J1939 Enums/Enum_Group_Function_Value.h"
#include "../SAE J1939 Enums/Enum_DM1_DM2.h"
#include "../SAE J1939 Enums/Enum_DM14_DM15.h"
#include "../SAE J1939 Enums/Enum_Send_Status.h"
#include "../SAE J1939 Enums/Enum_NAME.h"
#include "../SAE J1939 Enums/Enum_PGN.h"

/* Structs */
#include "../../Open SAE J1939/Structs.h"

ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Software_Identification(J1939 *j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Software_Identification(J1939* j1939, uint8_t DA);
void SAE_J1939_Read_Response_Request_Software_Identification(J1939 *j1939, uint8_t SA, uint8_t data[]);

/* ECU identification */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_ECU_Identification(J1939 *j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_ECU_Identification(J1939* j1939, uint8_t DA);
void SAE_J1939_Read_Response_Request_ECU_Identification(J1939 *j1939, uint8_t SA, uint8_t data[]);

/* Component identification */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Component_Identification(J1939 *j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Component_Identification(J1939* j1939, uint8_t DA);
void SAE_J1939_Read_Response_Request_Component_Identification(J1939 *j1939, uint8_t SA, uint8_t data[]);

#endif /* SAE_J1939_71_APPLICATION_LAYER_SAE_J1939_71_APPLICATION_LAYER_H_ */
