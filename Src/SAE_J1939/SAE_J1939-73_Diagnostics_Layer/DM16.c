/*
 * DM16.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Diagnostics_Layer.h"

/* Layers */
#include "../SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"

/* 
 * Send binary data transfer. This will be sent after DM15 memory response (if DM15 was proceeded)
 * PGN: 0x00D700 (55040)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Binary_Data_Transfer_DM16(J1939 *j1939, uint8_t DA, uint8_t number_of_occurences, uint8_t raw_binary_data[]) {
	if(number_of_occurences < 8) {
		uint32_t ID = (0x18D7 << 16) | (DA << 8) | j1939->information_this_ECU.this_ECU_address;
		uint8_t data[7 + 1];												/* number_of_occurences must be 7 */
		data[0] = number_of_occurences;										/* How much binary data we want to send */
		for(uint8_t i = 0; i < number_of_occurences; i++)
			data[i+1] = raw_binary_data[i];
		return CAN_Send_Message(ID, data);
	}else{
		/* Multiple messages - Load data */
		j1939->this_ecu_tp_cm.total_message_size = 0;
		j1939->this_ecu_tp_dt.data[j1939->this_ecu_tp_cm.total_message_size++] = number_of_occurences;
		for(uint8_t i = 0; i < number_of_occurences; i++)
			j1939->this_ecu_tp_dt.data[j1939->this_ecu_tp_cm.total_message_size++] = raw_binary_data[i];				/* When i = 0, then total_message_size = 1 */

		/* Send TP CM */
		j1939->this_ecu_tp_cm.number_of_packages = j1939->this_ecu_tp_cm.total_message_size % 8 > 0 ? j1939->this_ecu_tp_cm.total_message_size/8 + 1 : j1939->this_ecu_tp_cm.total_message_size/8; /* Rounding up */
		j1939->this_ecu_tp_cm.PGN_of_the_packeted_message = PGN_DM16;
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
 * Read binary data transfer
 * PGN: 0x00D700 (55040)
 */
void SAE_J1939_Read_Binary_Data_Transfer_DM16(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	j1939->from_other_ecu_dm.dm16.number_of_occurences = data[0];
	j1939->from_other_ecu_dm.dm16.from_ecu_address = SA;
	memset(j1939->from_other_ecu_dm.dm16.raw_binary_data, 0, sizeof(j1939->from_other_ecu_dm.dm16.raw_binary_data));
	for(uint8_t i = 0; i < data[0]; i++){
		j1939->from_other_ecu_dm.dm16.raw_binary_data[i] = data[i+1];
	}
}

