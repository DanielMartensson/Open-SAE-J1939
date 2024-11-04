/*
 * Open_SAE_J1939.h
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef OPEN_SAE_J1939_OPEN_SAE_J1939_H_
#define OPEN_SAE_J1939_OPEN_SAE_J1939_H_

/* Enum and structs */
#include "Structs.h"

/* Layers */
#include "../SAE_J1939/SAE_J1939-71_Application_Layer/Application_Layer.h"
#include "../SAE_J1939/SAE_J1939-73_Diagnostics_Layer/Diagnostics_Layer.h"
#include "../SAE_J1939/SAE_J1939-81_Network_Management_Layer/Network_Management_Layer.h"
#include "../SAE_J1939/SAE_J1939-21_Transport_Layer/Transport_Layer.h"

#ifdef __cplusplus
extern "C" {
#endif


/* Enum for the supported functionality */
typedef enum {
	RX_MSG_NONE = 0,
    RX_MSG_ACK,
    RX_MSG_REQ,
    RX_MSG_REQ_DM14,
    RX_MSG_DM15,
    RX_MSG_DM16,
    RX_MSG_TP_CONN_MANAGEMENT,
    RX_MSG_TP_CONN_DATA_TRANSFER,
    RX_MSG_RESP_REQ_PROPRIETARY_A,
    RX_MSG_RESP_REQ_PROPRIETARY_B,
    RX_MSG_RESP_REQ_ADDR_CLAIMED,
    RX_MSG_ADDR_NOT_CLAIMED,
    RX_MSG_RESP_REQ_DM1,
    RX_MSG_RESP_REQ_DM2,
    RX_MSG_RESP_REQ_SOFTWARE_IDENTIFICATION,
    RX_MSG_RESP_REQ_ECU_IDENTIFICATION,
    RX_MSG_RESP_REQ_COMPONENT_IDENTIFICATION,
    RX_MSG_RESP_REQ_AUX_ESTIMATED_FLOW,
    RX_MSG_RESP_REQ_GP_VALVE_ESTIMATED_FLOW,
    RX_MSG_RESP_REQ_AUX_VALVE_MEASURED_POSITION,
    RX_MSG_AUX_VALVE_CMD,
    RX_MSG_GP_VALVE_CMD,
    RX_MSG_NOT_SUPPORTED,
    RX_MSG_NOT_SAE_J1939,
    RX_MSG_UNKNOWN
} ENUM_J1939_RX_MSG;

/* This functions must be called all the time, or be placed inside an interrupt listener */
ENUM_J1939_RX_MSG Open_SAE_J1939_Listen_For_Messages(J1939 *j1939);

/* This function should ONLY be called at your ECU startup */
bool Open_SAE_J1939_Startup_ECU(J1939* j1939);

/* This function should ONLY be called at your ECU closedown */
bool Open_SAE_J1939_Closedown_ECU(J1939* j1939);

#ifdef __cplusplus
}
#endif

#endif /* OPEN_SAE_J1939_OPEN_SAE_J1939_H_ */
