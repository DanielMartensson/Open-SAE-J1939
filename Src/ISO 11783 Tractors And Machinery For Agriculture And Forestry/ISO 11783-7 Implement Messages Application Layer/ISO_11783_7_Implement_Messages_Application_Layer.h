/*
 * ISO_11783-7_Implement_Messages_Application_Layer.h
 *
 *  Created on: 16 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef ISO_11783_TRACTORS_AND_MACHINERY_FOR_AGRICULTURE_AND_FORESTRY_ISO_11783_7_IMPLEMENT_MESSAGES_APPLICATION_LAYER_ISO_11783_7_IMPLEMENT_MESSAGES_APPLICATION_LAYER_H_
#define ISO_11783_TRACTORS_AND_MACHINERY_FOR_AGRICULTURE_AND_FORESTRY_ISO_11783_7_IMPLEMENT_MESSAGES_APPLICATION_LAYER_ISO_11783_7_IMPLEMENT_MESSAGES_APPLICATION_LAYER_H_

/* Layers */
#include "../../CAN Network/CAN_Network.h"

/* Enums */

/* Structs */
#include "../../Open SAE J1939/Open_SAE_J1939_Structs.h"

/* Auxiliary Valve Command */
ENUM_J1939_STATUS_CODES ISO_11783_Send_Auxiliary_Valve_Command_To_All_ECU(J1939 *j1939, uint8_t valve_number, uint8_t standard_flow, uint8_t fail_safe_mode, uint8_t valve_state);
void ISO_11783_Read_Auxiliary_Valve_Command(J1939 *j1939, uint8_t valve_number, uint8_t data[]);

/* Auxiliary Valve Estimated Flow */
ENUM_J1939_STATUS_CODES ISO_11783_Send_Auxiliary_Valve_Estimated_Flow_To_All_ECU(J1939 *j1939, uint8_t valve_number, uint8_t extend_estimated_flow_standard, uint8_t retract_estimated_flow_standard, uint8_t fail_safe_mode, uint8_t valve_state, uint8_t limit);
void ISO_11783_Read_Auxiliary_Estimated_Flow(J1939 *j1939, uint8_t SA, uint8_t valve_number, uint8_t data[]);

/* Auxiliary Valve Measured Position */
ENUM_J1939_STATUS_CODES ISO_11783_Send_Auxiliary_Valve_Measured_Position_To_All_ECU(J1939 *j1939, uint8_t valve_number, uint16_t measured_position_procet, uint8_t valve_state, uint16_t measured_position_micrometer);
void ISO_11783_Read_Auxiliary_Valve_Measured_Position(J1939 *j1939, uint8_t SA, uint8_t valve_number, uint8_t data[]);

/* General Purpose Valve Command */
ENUM_J1939_STATUS_CODES ISO_11783_Send_General_Purpose_Valve_Command(J1939 *j1939, uint8_t DA, uint8_t standard_flow, uint8_t fail_safe_mode, uint8_t valve_state, uint16_t extended_flow);
void ISO_11783_Read_General_Purpose_Valve_Command(J1939 *j1939, uint8_t data[]);

/* General Purpose Valve Estimated Flow */
ENUM_J1939_STATUS_CODES ISO_11783_Send_General_Purpose_Valve_Estimated_Flow(J1939 *j1939, uint8_t DA, uint8_t extended_estimated_flow_standard, uint8_t retract_estimated_flow_standard, uint8_t fail_safe_mode, uint8_t valve_state, uint8_t limit, uint16_t extend_estimated_flow_estended, uint16_t retract_estimated_flow_extended);
void ISO_11783_Read_General_Purpose_Valve_Estimated_Flow(J1939 *j1939, uint8_t SA, uint8_t data[]);


#endif /* ISO_11783_TRACTORS_AND_MACHINERY_FOR_AGRICULTURE_AND_FORESTRY_ISO_11783_7_IMPLEMENT_MESSAGES_APPLICATION_LAYER_ISO_11783_7_IMPLEMENT_MESSAGES_APPLICATION_LAYER_H_ */
