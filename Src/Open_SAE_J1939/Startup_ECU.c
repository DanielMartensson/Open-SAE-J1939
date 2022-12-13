/*
 * Startup_ECU.c
 *
 *  Created on: 25 sep. 2021
 *      Author: Daniel Mårtensson
 */

#include "Open_SAE_J1939.h"

/* Layers */
#include "../Hardware/Hardware.h"

/* Load our ECU parameters into J1939 structure. Very useful if you want your ECU remember its NAME + address + identifications at startup. */
bool Open_SAE_J1939_Startup_ECU(J1939* j1939) {
	uint32_t ECU_information_length = sizeof(Information_this_ECU);
	uint8_t ECU_information_data[sizeof(Information_this_ECU)];
	memset(ECU_information_data, 0, ECU_information_length);
	if(!Load_Struct(ECU_information_data, ECU_information_length, (char*)INFORMATION_THIS_ECU))
		return false; /* Problems occurs */
	memcpy(&j1939->information_this_ECU, (Information_this_ECU*)ECU_information_data, ECU_information_length);

	/* If we are going to send and receive the ECU identification and component identification, we need to specify the size of them */
	j1939->information_this_ECU.this_identifications.ecu_identification.length_of_each_field = MAX_IDENTIFICATION;
	j1939->information_this_ECU.this_identifications.component_identification.length_of_each_field = MAX_IDENTIFICATION;
	j1939->from_other_ecu_identifications.ecu_identification.length_of_each_field = MAX_IDENTIFICATION;
	j1939->from_other_ecu_identifications.component_identification.length_of_each_field = MAX_IDENTIFICATION;

	/* Clear other ECU addresses by setting the broadcast address to them */
	memset(j1939->other_ECU_address, 0xFF, 0xFF);
	j1939->number_of_cannot_claim_address = 0;
	j1939->number_of_other_ECU = 0;

	/* This broadcast out this ECU NAME + address to all other ECU:s */
	SAE_J1939_Response_Request_Address_Claimed(j1939);

	/* This asking all ECU about their NAME + address */
	SAE_J1939_Send_Request_Address_Claimed(j1939, 0xFF);

	/* OK */
	return true;
}

