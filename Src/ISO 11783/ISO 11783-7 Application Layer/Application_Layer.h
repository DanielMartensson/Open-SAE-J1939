/*
 * ISO_11783-7_Application_Layer.h
 *
 *  Created on: 16 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef ISO_11783_ISO_11783_7_APPLICATION_LAYER_ISO_11783_7_APPLICATION_LAYER_H_
#define ISO_11783_ISO_11783_7_APPLICATION_LAYER_ISO_11783_7_APPLICATION_LAYER_H_

/* Layers */
#include "../../Hardware/CAN Network/CAN_Network.h"
#include "../../SAE J1939/SAE J1939-21 Transport Layer/Transport_Layer.h"

/* Enums */
#include "../../SAE J1939/SAE J1939 Enums/Enum_PGN.h"
#include "../ISO 11783 Enums/Enum_Auxiliary_And_General_Purpose_Valves.h"

/* Structs */
#include "../../Open SAE J1939/Structs.h"

/* Auxiliary Valve Command */
ENUM_J1939_STATUS_CODES ISO_11783_Send_Auxiliary_Valve_Command(J1939 *j1939, uint8_t valve_number, uint8_t standard_flow, uint8_t fail_safe_mode, uint8_t valve_state);
void ISO_11783_Read_Auxiliary_Valve_Command(J1939 *j1939, uint8_t SA, uint8_t valve_number, uint8_t data[]);

/* Auxiliary Valve Estimated Flow */
ENUM_J1939_STATUS_CODES ISO_11783_Send_Request_Auxiliary_Valve_Estimated_Flow(J1939 *j1939, uint8_t DA, uint8_t valve_number);
ENUM_J1939_STATUS_CODES ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(J1939 *j1939, uint8_t valve_number);
void ISO_11783_Read_Response_Request_Auxiliary_Estimated_Flow(J1939 *j1939, uint8_t SA, uint8_t valve_number, uint8_t data[]);

/* Auxiliary Valve Measured Position */
ENUM_J1939_STATUS_CODES ISO_11783_Send_Request_Auxiliary_Valve_Measured_Position(J1939 *j1939, uint8_t DA, uint8_t valve_number);
ENUM_J1939_STATUS_CODES ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(J1939 *j1939, uint8_t valve_number);
void ISO_11783_Read_Response_Request_Auxiliary_Valve_Measured_Position(J1939 *j1939, uint8_t SA, uint8_t valve_number, uint8_t data[]);

/* General Purpose Valve Command */
ENUM_J1939_STATUS_CODES ISO_11783_Send_General_Purpose_Valve_Command(J1939 *j1939, uint8_t DA, uint8_t standard_flow, uint8_t fail_safe_mode, uint8_t valve_state, uint16_t extended_flow);
void ISO_11783_Read_General_Purpose_Valve_Command(J1939 *j1939, uint8_t SA, uint8_t data[]);

/* General Purpose Valve Estimated Flow */
ENUM_J1939_STATUS_CODES ISO_11783_Send_Request_General_Purpose_Valve_Estimated_Flow(J1939 *j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES ISO_11783_Response_Request_General_Purpose_Valve_Estimated_Flow(J1939 *j1939, uint8_t DA);
void ISO_11783_Read_Response_Request_General_Purpose_Valve_Estimated_Flow(J1939 *j1939, uint8_t SA, uint8_t data[]);


#endif /* ISO_11783_ISO_11783_7_APPLICATION_LAYER_ISO_11783_7_APPLICATION_LAYER_H_ */
