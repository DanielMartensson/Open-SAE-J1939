/*
 * SAE_J1939_DM2.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "SAE_J1939-73_Diagnostics_Layer.h"

/*
 * Request DM2 from another ECU
 * PGN: 0x00FECB (65227)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_DM2(J1939 *j1939, uint8_t DA) {
	return SAE_J1939_Send_Request(j1939, DA, PGN_DM2);
}

/*
 * Response the request of DM2 information to other ECU about this ECU
 * PGN: 0x00FECB (65227)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_DM2(J1939 *j1939, uint8_t DA) {
	if(j1939->this_dm.errors_dm2_active < 2) {
		uint32_t ID = (0x18FECB << 8) | j1939->this_ECU_address;
		uint8_t data[8];
		data[0] = (j1939->this_dm.dm2.SAE_lamp_status_malfunction_indicator << 6) | (j1939->this_dm.dm2.SAE_lamp_status_red_stop << 4) | (j1939->this_dm.dm2.SAE_lamp_status_amber_warning << 2) | (j1939->this_dm.dm2.SAE_lamp_status_protect_lamp);
		data[1] = (j1939->this_dm.dm2.SAE_flash_lamp_malfunction_indicator << 6) | (j1939->this_dm.dm2.SAE_flash_lamp_red_stop << 4) | (j1939->this_dm.dm2.SAE_flash_lamp_amber_warning << 2) | (j1939->this_dm.dm2.SAE_flash_lamp_protect_lamp);
		data[2] = j1939->this_dm.dm2.SPN;
		data[3] = j1939->this_dm.dm2.SPN >> 8;
		data[4] = ((j1939->this_dm.dm2.SPN >> 11) & 0b11100000) | j1939->this_dm.dm2.FMI;
		data[5] = (j1939->this_dm.dm2.SPN_conversion_method << 7) | j1939->this_dm.dm2.occurence_count;
		data[6] = 0xFF;													/* Reserved */
		data[7] = 0xFF;													/* Reserved */
		return CAN_Send_Message(ID, data, 0);							/* 0 ms delay */
	} else {
		/* Multiple messages - Use Transport Protocol Connection Management BAM */
		uint8_t number_of_packages = 2;
		uint16_t total_message_size = 9;
		uint8_t data[9];
		data[0] = (j1939->this_dm.dm2.SAE_lamp_status_malfunction_indicator << 6) | (j1939->this_dm.dm2.SAE_lamp_status_red_stop << 4) | (j1939->this_dm.dm2.SAE_lamp_status_amber_warning << 2) | (j1939->this_dm.dm2.SAE_lamp_status_protect_lamp);
		data[1] = (j1939->this_dm.dm2.SAE_flash_lamp_malfunction_indicator << 6) | (j1939->this_dm.dm2.SAE_flash_lamp_red_stop << 4) | (j1939->this_dm.dm2.SAE_flash_lamp_amber_warning << 2) | (j1939->this_dm.dm2.SAE_flash_lamp_protect_lamp);
		data[2] = j1939->this_dm.dm2.SPN;
		data[3] = j1939->this_dm.dm2.SPN >> 8;
		data[4] = ((j1939->this_dm.dm2.SPN >> 11) & 0b11100000) | j1939->this_dm.dm2.FMI;
		data[5] = (j1939->this_dm.dm2.SPN_conversion_method << 7) | j1939->this_dm.dm2.occurence_count;
		data[6] = 0xFF;							/* Reserved */
		data[7] = 0xFF;							/* Reserved */
		data[8] = j1939->this_dm.errors_dm2_active;
		ENUM_J1939_STATUS_CODES status = SAE_J1939_Send_Transport_Protocol_Connection_Management(j1939, DA, CONTROL_BYTE_TP_CM_BAM, total_message_size, number_of_packages, PGN_DM2);
		if(status != STATUS_SEND_OK)
			return status;
		return SAE_J1939_Send_Transport_Protocol_Data_Transfer(j1939, DA, data, total_message_size, number_of_packages);
	}
}

/*
 * Store the last DM2 information about other ECU. At least we know how many errors are active
 * PGN: 0x00FECB (65227)
 */
void SAE_J1939_Read_Response_Request_DM2(J1939 *j1939, uint8_t data[], uint8_t errors_dm2_active) {
	j1939->from_other_ecu_dm.dm2.SAE_lamp_status_malfunction_indicator = data[0] >> 6;
	j1939->from_other_ecu_dm.dm2.SAE_lamp_status_red_stop = (data[0] >> 4) & 0b00000011;
	j1939->from_other_ecu_dm.dm2.SAE_lamp_status_amber_warning = (data[0] >> 2) & 0b00000011;
	j1939->from_other_ecu_dm.dm2.SAE_lamp_status_protect_lamp = data[0] & 0b00000011;
	j1939->from_other_ecu_dm.dm2.SAE_flash_lamp_malfunction_indicator = data[1] >> 6;
	j1939->from_other_ecu_dm.dm2.SAE_flash_lamp_red_stop = (data[1] >> 4) & 0b00000011;
	j1939->from_other_ecu_dm.dm2.SAE_flash_lamp_amber_warning = (data[1] >> 2) & 0b00000011;
	j1939->from_other_ecu_dm.dm2.SAE_flash_lamp_protect_lamp = data[1] & 0b00000011;
	j1939->from_other_ecu_dm.dm2.SPN = ((data[4] & 0b11100000) << 11) | (data[3] << 8) | data[2];
	j1939->from_other_ecu_dm.dm2.FMI = data[4] & 0b00011111;
	j1939->from_other_ecu_dm.dm2.SPN_conversion_method = data[5] >> 7;
	j1939->from_other_ecu_dm.dm2.occurence_count = data[5] & 0b01111111;

	/* Check if we have no fault cause */
	if(j1939->from_other_ecu_dm.dm2.FMI == DM1_FMI_NOT_AVAILABLE)
		j1939->from_other_ecu_dm.errors_dm2_active = 0;
	else
		j1939->from_other_ecu_dm.errors_dm2_active = errors_dm2_active;
}
