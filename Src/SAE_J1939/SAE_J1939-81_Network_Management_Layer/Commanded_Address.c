/*
 * Commanded_Address.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Network_Management_Layer.h"

/* Layers */
#include "../SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"

/*
 * Send commanded address to another ECU
 * PGN: 0x00FED8 (65240)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Commanded_Address(J1939 *j1939, uint8_t DA, uint8_t new_ECU_address, uint32_t identity_number, uint16_t manufacturer_code, uint8_t function_instance, uint8_t ECU_instance, uint8_t function, uint8_t vehicle_system, uint8_t arbitrary_address_capable, uint8_t industry_group, uint8_t vehicle_system_instance) {
	/* Multiple messages - Load data */
	j1939->this_ecu_tp_cm.number_of_packages = 2;
	j1939->this_ecu_tp_cm.total_message_size = 9;
	j1939->this_ecu_tp_dt.data[0] = identity_number;
	j1939->this_ecu_tp_dt.data[1] = identity_number >> 8;
	j1939->this_ecu_tp_dt.data[2] = (identity_number >> 16) |  (manufacturer_code << 5);
	j1939->this_ecu_tp_dt.data[3] = manufacturer_code >> 3;
	j1939->this_ecu_tp_dt.data[4] = (function_instance << 3) | ECU_instance;
	j1939->this_ecu_tp_dt.data[5] = function;
	j1939->this_ecu_tp_dt.data[6] = vehicle_system << 1;
	j1939->this_ecu_tp_dt.data[7] = (arbitrary_address_capable << 7) | (industry_group << 4) | vehicle_system_instance;
	j1939->this_ecu_tp_dt.data[8] = new_ECU_address;							/* New address of the ECU we are sending to*/

	/* Send TP CM */
	j1939->this_ecu_tp_cm.PGN_of_the_packeted_message = PGN_COMMANDED_ADDRESS;
	j1939->this_ecu_tp_cm.control_byte = DA == 0xFF ? CONTROL_BYTE_TP_CM_BAM : CONTROL_BYTE_TP_CM_RTS; /* If broadcast, then use BAM control byte */
	ENUM_J1939_STATUS_CODES status = SAE_J1939_Send_Transport_Protocol_Connection_Management(j1939, DA);
	if(status != STATUS_SEND_OK)
		return status;

	/* Check if we are going to send it directly (BAM) - Else, the TP CM will send a RTS control byte to the other ECU and the ECU will answer with control byte CTS */
	if(j1939->this_ecu_tp_cm.control_byte == CONTROL_BYTE_TP_CM_BAM)
		return SAE_J1939_Send_Transport_Protocol_Data_Transfer(j1939, DA);
	return status;

}

/*
 * Read the commanded address from another ECU. Will always be called from Transport Protocol Data Transfer due to 9 bytes of data
 * PGN: 0x00FED8 (65240)
 */
void SAE_J1939_Read_Commanded_Address(J1939 *j1939, uint8_t data[]) {
	/* Send to all ECU that the current address is unused */
	SAE_J1939_Send_Address_Delete(j1939, 0xFF, j1939->information_this_ECU.this_ECU_address);

	/* Apply new NAME and address for this ECU */
	j1939->information_this_ECU.this_name.identity_number = ((data[2] & 0b00011111) << 16) | (data[1] << 8) | data[0];
	j1939->information_this_ECU.this_name.manufacturer_code = (data[3] << 3) | (data[2] >> 5);
	j1939->information_this_ECU.this_name.function_instance = data[4] >> 3;
	j1939->information_this_ECU.this_name.ECU_instance = data[4] & 0b00000111;
	j1939->information_this_ECU.this_name.function = data[5];
	j1939->information_this_ECU.this_name.vehicle_system = data[6] >> 1;
	j1939->information_this_ECU.this_name.arbitrary_address_capable = data[7] >> 7;
	j1939->information_this_ECU.this_name.industry_group = (data[7] >> 4) & 0b0111;
	j1939->information_this_ECU.this_name.vehicle_system_instance = data[7] & 0b00001111;
	j1939->information_this_ECU.this_ECU_address = data[8]; 		/* New address of this ECU */
	Save_Struct((uint8_t*)&j1939->information_this_ECU, sizeof(Information_this_ECU), INFORMATION_THIS_ECU);

	/* Broadcast the new NAME and address of this ECU */
	SAE_J1939_Response_Request_Address_Claimed(j1939);
}
