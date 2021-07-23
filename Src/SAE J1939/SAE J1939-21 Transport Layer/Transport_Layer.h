/*
 * SAE_J1939-21_Application_Layer.h
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef TRANSPORT_LAYER_SAE_J1939_21_TRANSPORT_LAYER_H_
#define TRANSPORT_LAYER_SAE_J1939_21_TRANSPORT_LAYER_H_

/* Layers */
#include "../../Hardware/CAN Network/CAN_Network.h"
#include "../SAE J1939-71 Application Layer/Application_Layer.h"
#include "../SAE J1939-81 Network Management Layer/Network_Management_Layer.h"
#include "../SAE J1939-73 Diagnostics Layer/Diagnostics_Layer.h"

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

/* Acknowledgement */
void SAE_J1939_Read_Acknowledgement(J1939 *j1939, uint8_t SA, uint8_t data[]);
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Acknowledgement(J1939 *j1939, uint8_t DA, uint8_t control_byte, uint8_t group_function_value, uint32_t PGN_of_requested_info);

/* Request */
void SAE_J1939_Read_Request(J1939 *j1939, uint8_t SA, uint8_t data[]);
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request(J1939* j1939, uint8_t DA, uint32_t PGN_code);

/* Transport Protocol Data Transfer */
void SAE_J1939_Read_Transport_Protocol_Data_Transfer(J1939 *j1939, uint8_t SA, uint8_t data[]);
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Transport_Protocol_Data_Transfer(J1939 *j1939, uint8_t DA, uint8_t data[], uint16_t total_message_size, uint8_t number_of_packages);

/* Transport Protocol Connection Management */
void SAE_J1939_Read_Transport_Protocol_Connection_Management(J1939 *j1939, uint8_t SA, uint8_t data[]);
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Transport_Protocol_Connection_Management(J1939 *j1939, uint8_t DA, uint8_t control_byte, uint16_t total_message_size, uint8_t number_of_packages, uint32_t PGN_of_the_packeted_message);

#endif /* TRANSPORT_LAYER_SAE_J1939_21_TRANSPORT_LAYER_H_ */
