/*
 * DM3.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Diagnostics_Layer.h"

/* Layers */
#include "../SAE_J1939-21_Transport_Layer/Transport_Layer.h"

/*
 * Request DM3 from another ECU
 * PGN: 0x00FECC (65228)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_DM3(J1939 *j1939, uint8_t DA) {
	return SAE_J1939_Send_Request(j1939, DA, PGN_DM3);
}

/*
 * Response the request of DM3 (clear DM2, which is previously active errors from DM1 codes) to other ECU about this ECU
 * PGN: 0x00FECC (65228)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_DM3(J1939* j1939, uint8_t DA) {
	memset(&j1939->this_dm.dm2, 0, sizeof(j1939->this_dm.dm2)); 		/* This set all fields of dm2 to 0 */
	j1939->this_dm.errors_dm2_active = 0;
																		/* Removed assigning FMI to 31.  Fixed DMx active message count to not rely on this value*/
	return SAE_J1939_Response_Request_DM2(j1939, DA);					/* Send DM2 codes to the ECU who send the request */
}
