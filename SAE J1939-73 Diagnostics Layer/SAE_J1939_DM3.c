/*
 * SAE_J1939_DM2.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "SAE_J1939-73_Diagnostics_Layer.h"

/*
 * Request DM3 from another ECU
 * *j1939: Pointer to structure J1939
 * DA: Destination ECU address between 0 to 255 (DA 255 = Broadcast to all ECU)
 * PGN: 0x00FECC (65228)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_DM3(J1939 *j1939, uint8_t DA) {
	return SAE_J1939_Send_Request(j1939, DA, PGN_DM2);
}

/*
 * Response the request of DM3 (clear DM2, which is previously active errors from DM1 codes) to other ECU about this ECU
 * *j1939: Pointer to structure J1939
 * DA: Destination ECU address between 0 to 255 (DA 255 = Broadcast to all ECU)
 * PGN: 0x00FECC (65228)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_DM3(J1939* j1939, uint8_t DA) {
	for(uint8_t i = 0; i < 256; i++)
		memset(&j1939->this_dm.dm2[i], 0, sizeof(j1939->this_dm.dm2[i])); /* This set all fields of dm2 to 0 */

	// TODO: Send an ACK
}
