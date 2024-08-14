/*
 * Closedown_ECU.c
 *
 *  Created on: 16 Juli. 2023
 *      Author: Daniel Mårtensson
 */

#include "Open_SAE_J1939.h"

 /* Layers */
#include "../Hardware/Hardware.h"

/* Save our ECU parameters into J1939 structure. Very useful if you want your ECU remember its NAME + address + identifications at startup. */
bool Open_SAE_J1939_Closedown_ECU(J1939* j1939) {
	uint32_t ECU_information_length = sizeof(Information_this_ECU);
	uint8_t ECU_information_data[sizeof(Information_this_ECU)];
	memcpy(ECU_information_data, (uint8_t*)&j1939->information_this_ECU, ECU_information_length);
	return Save_Struct(ECU_information_data, ECU_information_length, (char*)INFORMATION_THIS_ECU);
}