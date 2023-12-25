/*
 * Structs_Manufacturer_Specific.h
 *
 *  Created on: 25 December 2023
 *      Author: Daniel Mårtensson
 */

#ifndef OPEN_SAE_J1939_OPEN_SAE_J1939_MANUFACTURER_SPECIFIC_STRUCTS_MANUFACTURER_SPECIFIC_H_
#define OPEN_SAE_J1939_OPEN_SAE_J1939_MANUFACTURER_SPECIFIC_STRUCTS_MANUFACTURER_SPECIFIC_H_

struct Vecmocon_Sync_FUP {
	uint8_t type;
	uint8_t user_byte;
	uint8_t D_SC;
	uint8_t SyncTimeMS;
};

#endif /* OPEN_SAE_J1939_OPEN_SAE_J1939_MANUFACTURER_SPECIFIC_STRUCTS_MANUFACTURER_SPECIFIC_H_ */