/*
 * Main.c
 *
 *  Created on: 16 juli 2021
 *      Author: Daniel Mårtensson
 */

#include <stdlib.h>
#include <stdio.h>

#include "ISO_11783/ISO_11783-7_Application_Layer/Application_Layer.h"
#include "Open_SAE_J1939/Open_SAE_J1939.h"
#include "SAE_J1939/SAE_J1939-71_Application_Layer/Application_Layer.h"
#include "SAE_J1939/SAE_J1939-73_Diagnostics_Layer/Diagnostics_Layer.h"
#include "SAE_J1939/SAE_J1939-81_Network_Management_Layer/Network_Management_Layer.h"

int main() {

	/* Create our J1939 structure with two ECU */
	J1939 j1939_1 = {0};
	J1939 j1939_2 = {0};

	/* Important to sent all non-address to 0xFF - Else we cannot use ECU address 0x0 */
	for(uint8_t i = 0; i < 255; i++){
		j1939_1.other_ECU_address[i] = 0xFF;
		j1939_2.other_ECU_address[i] = 0xFF;
	}

	/* Set the ECU address */
	j1939_1.this_ECU_address = 0x80;												/* From 0 to 253 because 254 = error address and 255 = broadcast address */
	j1939_2.this_ECU_address = 0xFD;


	return 0;
}
