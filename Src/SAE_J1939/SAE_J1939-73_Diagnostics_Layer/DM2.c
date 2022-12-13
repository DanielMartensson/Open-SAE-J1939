/*
 * DM2.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Diagnostics_Layer.h"

/* Layers */
#include "../SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"

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
		uint32_t ID = (0x18FECB << 8) | j1939->information_this_ECU.this_ECU_address;
		uint8_t data[8];
		data[0] = (j1939->this_dm.dm2.SAE_lamp_status_malfunction_indicator << 6) | (j1939->this_dm.dm2.SAE_lamp_status_red_stop << 4) | (j1939->this_dm.dm2.SAE_lamp_status_amber_warning << 2) | (j1939->this_dm.dm2.SAE_lamp_status_protect_lamp);
		data[1] = (j1939->this_dm.dm2.SAE_flash_lamp_malfunction_indicator << 6) | (j1939->this_dm.dm2.SAE_flash_lamp_red_stop << 4) | (j1939->this_dm.dm2.SAE_flash_lamp_amber_warning << 2) | (j1939->this_dm.dm2.SAE_flash_lamp_protect_lamp);
		data[2] = j1939->this_dm.dm2.SPN[1];
		data[3] = j1939->this_dm.dm2.SPN[1] >> 8;
		data[4] = ((j1939->this_dm.dm2.SPN[1] >> 11) & 0b11100000) | j1939->this_dm.dm2.FMI[1];
		data[5] = (j1939->this_dm.dm2.SPN_conversion_method[1] << 7) | j1939->this_dm.dm2.occurrence_count[1];
		data[6] = 0xFF;													/* Reserved */
		data[7] = 0xFF;													/* Reserved */
		return CAN_Send_Message(ID, data);
	} else {
		/* Multiple messages - Load data */
		j1939->this_ecu_tp_cm.total_message_size = (j1939->this_dm.errors_dm2_active *4) +2 ;				/* set total message size where each DTC is 4 btyes, plus 2 bytes for the lamp code */
		j1939->this_ecu_tp_cm.number_of_packages = (j1939->this_ecu_tp_cm.total_message_size)/7;			/* set number of packages, where each package will transmit up to 7 bytes */
		if (j1939->this_ecu_tp_cm.total_message_size % 7 > 0) j1939->this_ecu_tp_cm.number_of_packages++;	/* add extra frame if data rolls over */

		/* Load lamp data to first two bytes */
		j1939->this_ecu_tp_dt.data[0] = (j1939->this_dm.dm2.SAE_lamp_status_malfunction_indicator << 6) | (j1939->this_dm.dm2.SAE_lamp_status_red_stop << 4) | (j1939->this_dm.dm2.SAE_lamp_status_amber_warning << 2) | (j1939->this_dm.dm2.SAE_lamp_status_protect_lamp);
		j1939->this_ecu_tp_dt.data[1] = (j1939->this_dm.dm2.SAE_flash_lamp_malfunction_indicator << 6) | (j1939->this_dm.dm2.SAE_flash_lamp_red_stop << 4) | (j1939->this_dm.dm2.SAE_flash_lamp_amber_warning << 2) | (j1939->this_dm.dm2.SAE_flash_lamp_protect_lamp);
		/* Load DTCs into TP data package */
		for (uint8_t i = 0; i < j1939->this_ecu_tp_cm.total_message_size; i++){
			j1939->this_ecu_tp_dt.data[i*4 + 2] = j1939->this_dm.dm2.SPN[i];
			j1939->this_ecu_tp_dt.data[i*4 + 3] = j1939->this_dm.dm2.SPN[i] >> 8;
			j1939->this_ecu_tp_dt.data[i*4 + 4] = ((j1939->this_dm.dm2.SPN[i] >> 11) & 0b11100000) | j1939->this_dm.dm2.FMI[i];
			j1939->this_ecu_tp_dt.data[i*4 + 5] = (j1939->this_dm.dm2.SPN_conversion_method[i] << 7) | j1939->this_dm.dm2.occurrence_count[i];
		}

		/* Send TP CM */
		j1939->this_ecu_tp_cm.PGN_of_the_packeted_message = PGN_DM2;
		j1939->this_ecu_tp_cm.control_byte = DA == 0xFF ? CONTROL_BYTE_TP_CM_BAM : CONTROL_BYTE_TP_CM_RTS; /* If broadcast, then use BAM control byte */
		ENUM_J1939_STATUS_CODES status = SAE_J1939_Send_Transport_Protocol_Connection_Management(j1939, DA);
		if(status != STATUS_SEND_OK)
			return status;

		/* Check if we are going to send it directly (BAM) - Else, the TP CM will send a RTS control byte to the other ECU and the ECU will answer with control byte CTS */
		if(j1939->this_ecu_tp_cm.control_byte == CONTROL_BYTE_TP_CM_BAM)
			return SAE_J1939_Send_Transport_Protocol_Data_Transfer(j1939, DA);
		return status;
	}
}

/*
 * Store the last DM2 information about other ECU. At least we know how many errors are active
 * PGN: 0x00FECB (65227)
 */
void SAE_J1939_Read_Response_Request_DM2(J1939 *j1939, uint8_t SA, uint8_t data[], uint8_t errors_dm2_active) {
	/* iF there are fewer active DTCs than previous, clear the list so old DTCs do not remain after parsing the new ones */
	if (errors_dm2_active < j1939->from_other_ecu_dm.errors_dm2_active) {
		memset(&j1939->from_other_ecu_dm.dm2.SPN, 0, sizeof(j1939->from_other_ecu_dm.dm2.SPN)); 	/* This set all fields of dm1 to 0 */
	}

	/* Decode lamp status */
	j1939->from_other_ecu_dm.dm2.SAE_lamp_status_malfunction_indicator = data[0] >> 6;
	j1939->from_other_ecu_dm.dm2.SAE_lamp_status_red_stop = (data[0] >> 4) & 0b00000011;
	j1939->from_other_ecu_dm.dm2.SAE_lamp_status_amber_warning = (data[0] >> 2) & 0b00000011;
	j1939->from_other_ecu_dm.dm2.SAE_lamp_status_protect_lamp = data[0] & 0b00000011;
	j1939->from_other_ecu_dm.dm2.SAE_flash_lamp_malfunction_indicator = data[1] >> 6;
	j1939->from_other_ecu_dm.dm2.SAE_flash_lamp_red_stop = (data[1] >> 4) & 0b00000011;
	j1939->from_other_ecu_dm.dm2.SAE_flash_lamp_amber_warning = (data[1] >> 2) & 0b00000011;
	j1939->from_other_ecu_dm.dm2.SAE_flash_lamp_protect_lamp = data[1] & 0b00000011;

	/* Read and Decode DTC info for up to MAX_DM_FIELD previously active DTCs */
	for (uint8_t i = 0; i < errors_dm2_active && i < MAX_DM_FIELD; i++){
		j1939->from_other_ecu_dm.dm2.SPN[i] = ((data[(i*4)+4] & 0b11100000) << 11) | (data[(i*4)+3] << 8) | data[(i*4)+2];
		j1939->from_other_ecu_dm.dm2.FMI[i] = data[(i*4)+4] & 0b00011111;
		j1939->from_other_ecu_dm.dm2.SPN_conversion_method[i] = data[(i*4)+5] >> 7;
		j1939->from_other_ecu_dm.dm2.occurrence_count[i] = data[(i*4)+5] & 0b01111111;
		j1939->from_other_ecu_dm.dm2.from_ecu_address[i] = SA;
	}

	/* Assign number of DTCs in previously active list */
	if (errors_dm2_active == 1 && j1939->from_other_ecu_dm.dm2.SPN[0] == 0) {
		j1939->from_other_ecu_dm.errors_dm2_active = 0;
	} else if (j1939->from_other_ecu_dm.errors_dm2_active < errors_dm2_active) {
		j1939->from_other_ecu_dm.errors_dm2_active = errors_dm2_active;
	}
}
