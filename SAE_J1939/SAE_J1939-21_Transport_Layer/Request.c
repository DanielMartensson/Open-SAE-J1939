/*
 * Request.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Transport_Layer.h"

/* Layers */
#include "../../ISO_11783/ISO_11783-7_Application_Layer/Application_Layer.h"
#include "../SAE_J1939-73_Diagnostics_Layer/Diagnostics_Layer.h"
#include "../SAE_J1939-71_Application_Layer/Application_Layer.h"
#include "../SAE_J1939-81_Network_Management_Layer/Network_Management_Layer.h"

/*
 * Read a PGN request from another ECU about PGN information at this ECU. All listed PGN should be here
 * PGN: 0x00EA00 (59904)
 */
void SAE_J1939_Read_Request(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	uint32_t PGN = (data[2] << 16) | (data[1] << 8) | data[0];
	switch (PGN) {
	case PGN_ACKNOWLEDGEMENT:
		SAE_J1939_Send_Acknowledgement(j1939, SA, CONTROL_BYTE_ACKNOWLEDGEMENT_PGN_SUPPORTED, GROUP_FUNCTION_VALUE_NORMAL, PGN);
		break;
	case PGN_ADDRESS_CLAIMED:
		SAE_J1939_Response_Request_Address_Claimed(j1939);
		break;
	case PGN_COMMANDED_ADDRESS:
		SAE_J1939_Send_Acknowledgement(j1939, SA, CONTROL_BYTE_ACKNOWLEDGEMENT_PGN_SUPPORTED, GROUP_FUNCTION_VALUE_NORMAL, PGN);
		break;
	case PGN_ADDRESS_DELETE:
		SAE_J1939_Send_Acknowledgement(j1939, SA, CONTROL_BYTE_ACKNOWLEDGEMENT_PGN_SUPPORTED, GROUP_FUNCTION_VALUE_NORMAL, PGN); /* Not SAE J1939 standard */
		break;
	case PGN_DM1:
		SAE_J1939_Response_Request_DM1(j1939, SA);
		break;
	case PGN_DM2:
		SAE_J1939_Response_Request_DM2(j1939, SA);
		SAE_J1939_Send_Acknowledgement(j1939, SA, CONTROL_BYTE_ACKNOWLEDGEMENT_PGN_SUPPORTED, GROUP_FUNCTION_VALUE_NORMAL, PGN);
		break;
	case PGN_DM3:
		SAE_J1939_Response_Request_DM3(j1939, SA);
		break;
	case PGN_REQUEST:
		SAE_J1939_Send_Acknowledgement(j1939, SA, CONTROL_BYTE_ACKNOWLEDGEMENT_PGN_SUPPORTED, GROUP_FUNCTION_VALUE_NORMAL, PGN);
		break;
	case PGN_TP_CM:
		SAE_J1939_Send_Acknowledgement(j1939, SA, CONTROL_BYTE_ACKNOWLEDGEMENT_PGN_SUPPORTED, GROUP_FUNCTION_VALUE_NORMAL, PGN);
		break;
	case PGN_TP_DT:
		SAE_J1939_Send_Acknowledgement(j1939, SA, CONTROL_BYTE_ACKNOWLEDGEMENT_PGN_SUPPORTED, GROUP_FUNCTION_VALUE_NORMAL, PGN);
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_0:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_1:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_2:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_3:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_4:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_5:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_6:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_7:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_8:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_9:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_10:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_11:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_12:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_13:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_14:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_15:
		ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_GENERAL_PURPOSE_VALVE_ESTIMATED_FLOW:
		ISO_11783_Response_Request_General_Purpose_Valve_Estimated_Flow(j1939, SA);
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_0:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_1:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_2:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_3:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_4:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_5:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_6:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_7:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_8:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_9:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_10:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_11:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_12:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_13:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_14:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_AUXILIARY_VALVE_MEASURED_POSITION_15:
		ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xF); /* PGN & 0xF = valve_number */
		break;
	case PGN_SOFTWARE_IDENTIFICATION:
		SAE_J1939_Response_Request_Software_Identification(j1939, SA);
		break;
	case PGN_ECU_IDENTIFICATION:
		SAE_J1939_Response_Request_ECU_Identification(j1939, SA);
		break;
	case PGN_COMPONENT_IDENTIFICATION:
		SAE_J1939_Response_Request_Component_Identification(j1939, SA);
		break;
	case PGN_PROPRIETARY_A:
		SAE_J1939_Response_Request_Proprietary_A(j1939, SA);
		break;
		/* Add more else if statements here for more read request */
	default:
		SAE_J1939_Send_Acknowledgement(j1939, SA, CONTROL_BYTE_ACKNOWLEDGEMENT_PGN_NOT_SUPPORTED, GROUP_FUNCTION_VALUE_NO_CAUSE, PGN);
		break;
	}
}

/*
 * Request PGN information at other ECU
 * PGN: 0x00EA00 (59904)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request(J1939 *j1939, uint8_t DA, uint32_t PGN_code) {
	uint8_t PGN[3];
	PGN[0] = PGN_code;														/* PGN least significant bit */
	PGN[1] = PGN_code >> 8;													/* PGN mid bit */
	PGN[2] = PGN_code >> 16;												/* PGN most significant bit */
	uint32_t ID = (0x18EA << 16) | (DA << 8) | j1939->information_this_ECU.this_ECU_address;
	return CAN_Send_Request(ID, PGN);
}
