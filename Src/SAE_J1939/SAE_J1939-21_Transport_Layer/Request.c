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
void SAE_J1939_Read_Request(J1939* j1939, uint8_t SA, uint8_t data[])
{
    uint32_t PGN = ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | data[0];

    // Determine response DA for the existing DM request cases:
    //   - Broadcast request (DA=0xFF) → respond broadcast (BAM)
    //   - Unicast request              → respond to the requester (SA)
    uint8_t request_da     = (uint8_t)((j1939->ID >> 8) & 0xFFU);
    uint8_t dm_response_da = (request_da == 0xFFU) ? 0xFFU : SA;

    switch (PGN)
    {
        case PGN_ADDRESS_CLAIMED: SAE_J1939_Response_Request_Address_Claimed(j1939); break;
        case PGN_ACKNOWLEDGEMENT:
        case PGN_COMMANDED_ADDRESS:
        case PGN_ADDRESS_DELETE:
        case PGN_REQUEST:
        case PGN_TP_CM:
        case PGN_TP_DT:
            SAE_J1939_Send_Acknowledgement(
                j1939,
                SA,
                CONTROL_BYTE_ACKNOWLEDGEMENT_PGN_SUPPORTED,
                GROUP_FUNCTION_VALUE_NORMAL,
                PGN);
            break;
        case PGN_DM1: SAE_J1939_Response_Request_DM1(j1939, dm_response_da); break;
        case PGN_DM2:
            SAE_J1939_Response_Request_DM2(j1939, dm_response_da);
            SAE_J1939_Send_Acknowledgement(
                j1939,
                SA,
                CONTROL_BYTE_ACKNOWLEDGEMENT_PGN_SUPPORTED,
                GROUP_FUNCTION_VALUE_NORMAL,
                PGN);
            break;
        case PGN_DM3: SAE_J1939_Response_Request_DM3(j1939, SA); break;
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_0:
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_1:
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_2:
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_3:
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_4:
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_5:
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_6:
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_7:
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_8:
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_9:
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_10:
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_11:
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_12:
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_13:
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_14:
        case PGN_AUXILIARY_VALVE_ESTIMATED_FLOW_15:
            ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(j1939, PGN & 0xFUL);
            break;
        case PGN_GENERAL_PURPOSE_VALVE_ESTIMATED_FLOW:
            ISO_11783_Response_Request_General_Purpose_Valve_Estimated_Flow(j1939, SA);
            break;
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_0:
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_1:
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_2:
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_3:
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_4:
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_5:
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_6:
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_7:
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_8:
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_9:
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_10:
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_11:
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_12:
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_13:
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_14:
        case PGN_AUXILIARY_VALVE_MEASURED_POSITION_15:
            ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(j1939, PGN & 0xFUL);
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
        case PGN_PROPRIETARY_A: SAE_J1939_Response_Request_Proprietary_A(j1939, SA); break;
        default:
            if (((PGN >= PGN_PROPRIETARY_B_START) && (PGN <= PGN_PROPRIETARY_B_END))
                || ((PGN >= PGN_PROPRIETARY_B2_START) && (PGN <= PGN_PROPRIETARY_B2_END)))
            {
                bool is_supported = false;
                SAE_J1939_Response_Request_Proprietary_B(j1939, SA, PGN, &is_supported);
                if (is_supported)
                {
                    break;
                }
            }
            SAE_J1939_Send_Acknowledgement(
                j1939,
                SA,
                CONTROL_BYTE_ACKNOWLEDGEMENT_PGN_NOT_SUPPORTED,
                GROUP_FUNCTION_VALUE_NO_CAUSE,
                PGN);
            break;
    }
}

/*
 * Request PGN information at other ECU
 * PGN: 0x00EA00 (59904)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request(J1939 *j1939, uint8_t DA, uint32_t PGN_code) {
	uint8_t PGN[3];
    PGN[0] = (uint8_t)(PGN_code);                 /* PGN least significant bit */
    PGN[1] = (uint8_t)(PGN_code >> 8);            /* PGN mid bit */
    PGN[2] = (uint8_t)(PGN_code >> 16);           /* PGN most significant bit */

    uint32_t ID = (0x18EAUL << 16) | ((uint32_t)(DA) << 8)
                  | (uint32_t)(j1939->information_this_ECU.this_ECU_address);
    return CAN_Send_Request(ID, PGN);
}
