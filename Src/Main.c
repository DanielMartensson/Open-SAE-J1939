/*
 * Main.c
 *
 *  Created on: 16 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "stdlib.h"
#include "stdio.h"

#include "ISO 11783/ISO 11783-7 Application Layer/ISO_11783-7_Application_Layer.h"
#include "Open SAE J1939/Open_SAE_J1939.h"
#include "SAE J1939-71 Application Layer/SAE_J1939-71_Application_Layer.h"
#include "SAE J1939-73 Diagnostics Layer/SAE_J1939-73_Diagnostics_Layer.h"
#include "SAE J1939-81 Network Management Layer/SAE_J1939-81_Network_Management_Layer.h"

int main() {

	/* Create our J1939 structure with two ECU */
	J1939 j1939_1 = {0};
	J1939 j1939_2 = {0};

	/* Set the ECU address */
	j1939_1.this_ECU_address = 0x80;												/* From 0 to 253 because 254 = error address and 255 = broadcast address */
	j1939_2.this_ECU_address = 0x90;

	/* Set the DM1 error message - You can have multiple error messages, but only send one at the time */
	j1939_1.this_dm.dm1.SAE_lamp_status_malfunction_indicator = 1;
	j1939_1.this_dm.dm1.SAE_lamp_status_red_stop = 0;
	j1939_1.this_dm.dm1.SAE_lamp_status_amber_warning = 1;
	j1939_1.this_dm.dm1.SAE_lamp_status_protect_lamp = 0;
	j1939_1.this_dm.dm1.SAE_flash_lamp_malfunction_indicator = 0;
	j1939_1.this_dm.dm1.SAE_flash_lamp_red_stop = 1;
	j1939_1.this_dm.dm1.SAE_flash_lamp_amber_warning = 0;
	j1939_1.this_dm.dm1.SAE_flash_lamp_protect_lamp = 1;
	j1939_1.this_dm.dm1.FMI = FMI_CURRENT_ABOVE_NORMAL;								/* If FMI_NOT_AVAILABLE, then errors_dm1_active will become 0 */
	j1939_1.this_dm.dm1.SPN = SPN_5_VOLTS_DC_SUPPLY;
	j1939_1.this_dm.dm1.SPN_conversion_method = 1;
	j1939_1.this_dm.dm1.occurrence_count = 50;
	j1939_1.this_dm.errors_dm1_active = 10;											/* If this is above 1, then this is going to be send as multi-packet */

	/* Request DM1 codes from ECU 2 to ECU 1 */
	SAE_J1939_Send_Request(&j1939_2, 0x80, PGN_DM1);

	/* Response request from ECU 1 perspective */
	Open_SAE_J1939_Listen_For_Messages(&j1939_1);

	/* Read response request from ECU 1 to ECU 2 */
	for(uint8_t i = 0; i < 15; i++)
		Open_SAE_J1939_Listen_For_Messages(&j1939_2);

	/* Display what ECU 2 got */
	printf("SAE lamp status malfunction indicator = %i\nSAE lamp status red stop = %i\nSAE lamp status amber warning = %i\nSAE lamp status protect lamp = %i\nSAE flash lamp malfunction indicator = %i\nSAE flash lamp_red stop = %i\nSAE flash lamp amber warning = %i\nSAE flash lamp protect lamp = %i\nFMI = %i\nSPN = %i\nSPN conversion method = %i\nOccurrence_count = %i\nErrors dm1 active = %i\n"
			,j1939_2.from_other_ecu_dm.dm1.SAE_lamp_status_malfunction_indicator
			,j1939_2.from_other_ecu_dm.dm1.SAE_lamp_status_red_stop
			,j1939_2.from_other_ecu_dm.dm1.SAE_lamp_status_amber_warning
			,j1939_2.from_other_ecu_dm.dm1.SAE_lamp_status_protect_lamp
			,j1939_2.from_other_ecu_dm.dm1.SAE_flash_lamp_malfunction_indicator
			,j1939_2.from_other_ecu_dm.dm1.SAE_flash_lamp_red_stop
			,j1939_2.from_other_ecu_dm.dm1.SAE_flash_lamp_amber_warning
			,j1939_2.from_other_ecu_dm.dm1.SAE_flash_lamp_protect_lamp
			,j1939_2.from_other_ecu_dm.dm1.FMI
			,j1939_2.from_other_ecu_dm.dm1.SPN
			,j1939_2.from_other_ecu_dm.dm1.SPN_conversion_method
			,j1939_2.from_other_ecu_dm.dm1.occurrence_count
			,j1939_2.from_other_ecu_dm.errors_dm1_active);

	return 0;
}
