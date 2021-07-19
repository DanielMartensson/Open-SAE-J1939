/*
 * Main.c
 *
 *  Created on: 16 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "stdlib.h"
#include "stdio.h"

/* Our includes */
#include "Open SAE J1939/Open_SAE_J1939.h"
#include "SAE J1939-71 Application Layer/SAE_J1939-71_Application_Layer.h"
#include "SAE J1939-73 Diagnostics Layer/SAE_J1939-73_Diagnostics_Layer.h"
#include "SAE J1939-81 Network Management Layer/SAE_J1939-81_Network_Management_Layer.h"
#include "ISO 11783 Tractors And Machinery For Agriculture And Forestry/ISO 11783-7 Implement Messages Application Layer/ISO_11783_7_Implement_Messages_Application_Layer.h"


int main() {

	/* Create our J1939 structure with two ECU */
	J1939 j1939_1;
	J1939 j1939_2;

	/* Important to set all to 0 */
	memset(&j1939_1, 0, sizeof(j1939_1));
	memset(&j1939_2, 0, sizeof(j1939_2));

	/* Set the ECU address */
	j1939_1.this_ECU_address = 0x80;												/* From 0 to 253 because 254 = error address and 255 = broadcast address */
	j1939_2.this_ECU_address = 0x90;

	/* Set NAME for ECU 1 */
	j1939_1.this_name.identity_number = 100; 										/* From 0 to 2097151 */
	j1939_1.this_name.manufacturer_code = 300; 										/* From 0 to 2047 */
	j1939_1.this_name.function_instance = 10; 										/* From 0 to 31 */
	j1939_1.this_name.ECU_instance = 2; 											/* From 0 to 7 */
	j1939_1.this_name.function = FUNCTION_VDC_MODULE;								/* From 0 to 255 */
	j1939_1.this_name.vehicle_system = 100;											/* From 0 to 127 */
	j1939_1.this_name.arbitrary_address_capable = 0;								/* From 0 to 1 */
	j1939_1.this_name.industry_group = INDUSTRY_GROUP_CONSTRUCTION;					/* From 0 to 7 */
	j1939_1.this_name.vehicle_system_instance = 10;									/* From 0 to 15 */

	/* Set NAME for ECU 2 */
	j1939_2.this_name.identity_number = 1000; 										/* From 0 to 2097151 */
	j1939_2.this_name.manufacturer_code = 400; 										/* From 0 to 2047 */
	j1939_2.this_name.function_instance = 20; 										/* From 0 to 31 */
	j1939_2.this_name.ECU_instance = 1; 											/* From 0 to 7 */
	j1939_2.this_name.function = FUNCTION_AUXILIARY_VALVES_CONTROL;					/* From 0 to 255 */
	j1939_2.this_name.vehicle_system = 50;											/* From 0 to 127 */
	j1939_2.this_name.arbitrary_address_capable = 0;								/* From 0 to 1 */
	j1939_2.this_name.industry_group = INDUSTRY_GROUP_AGRICULTURAL_AND_FORESTRY;	/* From 0 to 7 */
	j1939_2.this_name.vehicle_system_instance = 15;									/* From 0 to 15 */

	/* Broadcast NAME from ECU 1 */
	SAE_J1939_Send_Request_Address_Claimed(&j1939_1, 0x90); 						/* Request ECU NAME from ECU 2 */

	/* Listen for messages at ECU 2 */
	Open_SAE_J1939_Listen_For_Messages(&j1939_2);

	/* Send NAME from ECU 2 to ECU 1 */
	SAE_J1939_Send_Request_Address_Claimed(&j1939_2, 0x80); 						/* Request ECU NAME from ECU 1 */

	/* Listen for messages at ECU 1 */
	Open_SAE_J1939_Listen_For_Messages(&j1939_1);

	/* Print information */
	printf("How many external ECU are connected according to ECU 1? %i\n", j1939_1.number_of_ECU);
	printf("How many external ECU are connected according to ECU 2? %i\n", j1939_2.number_of_ECU);


	return 0;
}

