/*
 * SAE_J1939_DM2.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "SAE_J1939-73_Diagnostics_Layer.h"

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
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_DM3(J1939* j1939) {
	for(uint8_t i = 0; i < 256; i++)
		memset(&j1939->this_dm.dm2[i], 0, sizeof(j1939->this_dm.dm2[i])); 	/* This set all fields of dm2 to 0 */

	/* Send DM2 codes to all ECU */
	SAE_J1939_Response_Request_DM2(j1939, 0xFF);							/* Broadcast */
}
