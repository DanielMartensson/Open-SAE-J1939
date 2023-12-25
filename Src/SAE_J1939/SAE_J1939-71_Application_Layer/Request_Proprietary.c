/*
 * Request_Proprietary.c
 *
 *  Created on: 25 December 2023
 *      Author: Daniel Mårtensson
 */

#include "Application_Layer.h"

 /* Layers */
#include "../SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"

/*
 * Request Proprietary A to another ECU
 * PGN: 0x00EF00 (61184)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Proprietary_A(J1939* j1939, uint8_t DA) {
	return SAE_J1939_Send_Request(j1939, DA, PGN_PROPRIETARY_A);
}

/*
 * Response the request Proprietary A about this ECU
 * PGN: 0x00EF00 (61184)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Proprietary_A(J1939* j1939, uint8_t DA) {
	/* This code is going to send the Proprietary_A data to the DA (destination address) 
	 Follow the same structure as the other ECU_Identifications.c or Request_Software_Identitications.c
	*/
}

/*
 * Store the Proprietary A about other ECU
 * PGN: 0x00EF00 (61184)
 */
void SAE_J1939_Read_Response_Request_Proprietary_A(J1939* j1939, uint8_t SA, uint8_t data[]) {
	/* This code is going to save the data in Proprietary_A structure */
}
