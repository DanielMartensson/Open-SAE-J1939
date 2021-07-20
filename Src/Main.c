/*
 * Main.c
 *
 *  Created on: 16 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "stdlib.h"
#include "stdio.h"

#include "ISO 11783/ISO 11783-7 Application Layer/Application_Layer.h"
#include "Open SAE J1939/Open_SAE_J1939.h"
#include "SAE J1939/SAE J1939-71 Application Layer/Application_Layer.h"
#include "SAE J1939/SAE J1939-73 Diagnostics Layer/Diagnostics_Layer.h"
#include "SAE J1939/SAE J1939-81 Network Management Layer/Network_Management_Layer.h"


int main() {

	/* Create our J1939 structure with two ECU */
	J1939 j1939_1 = {0};
	J1939 j1939_2 = {0};

	/* Set the ECU address */
	j1939_1.this_ECU_address = 0x80; /* From 0 to 253 because 254 = error address and 255 = broadcast address */
	j1939_2.this_ECU_address = 0xFD;

	/* Do your logic here */

	return 0;
}
