/*
 * Request_Proprietary_B.c
 *
 *  Created on: 3 November 2024
 *      Author: Guillermo Rodríguez
 */

#include "Application_Layer.h"

 /* Layers */
#include "../SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"

/*
 * Request Proprietary B to another ECU
 * PGN: 0x00FF00 <-> 0x00FFFF
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Proprietary_B(J1939* j1939, uint8_t DA, uint32_t PGN) {
	return SAE_J1939_Send_Request(j1939, DA, PGN);
}

 // IMPORTANT: This function can return NULL if given PGN is not expected in this ECU
struct Proprietary_B * Get_Proprietary_B_By_PGN(struct Proprietary * proprietary, uint32_t PGN)
{
	for (int i = 0; i < MAX_PROPRIETARY_B_PGNS; ++i)
	{
		if (proprietary->proprietary_B[i].pgn == PGN)
		{
			return &proprietary->proprietary_B[i];
		}
	}
	return NULL;
}

/*
 * Response the request Proprietary B about this ECU
 * PGN: 0x00FF00 <-> 0x00FFFF
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Proprietary_B(J1939* j1939, uint8_t DA, uint32_t PGN, bool * is_supported) {
	struct Proprietary_B * proprietary_B = Get_Proprietary_B_By_PGN(&j1939->this_proprietary, PGN);
	if (proprietary_B == NULL) /* This PGN is not implemented on this ECU, can't send reply */
	{
		*is_supported = false;
		return STATUS_SEND_ERROR;
	}
	*is_supported = true;

	/* Find the length of the array fields */
	uint16_t length_of_each_field = proprietary_B->total_bytes;
	if (length_of_each_field < 9) {
		/* If each field have the length 8 or less, then we can send Proprietary B as it was a normal message */
		uint32_t ID = (PGN << 8) | j1939->information_this_ECU.this_ECU_address;
		uint8_t data[8];
		memcpy(data, proprietary_B->data, length_of_each_field);
		return CAN_Send_Message(ID, data);
	}
	else {
		/* Multiple messages - Load data */
		j1939->this_ecu_tp_cm.total_message_size_being_transmitted = length_of_each_field;
		memcpy(j1939->this_ecu_tp_dt.data, proprietary_B->data, length_of_each_field);
		
		/* Send TP CM */
		j1939->this_ecu_tp_cm.number_of_packages_being_transmitted = SAE_J1939_Transport_Protocol_GetNumberOfPackages(j1939->this_ecu_tp_cm.total_message_size_being_transmitted);
		j1939->this_ecu_tp_cm.PGN_of_the_packeted_message = PGN;
		j1939->this_ecu_tp_cm.control_byte = DA == 0xFF ? CONTROL_BYTE_TP_CM_BAM : CONTROL_BYTE_TP_CM_RTS; /* If broadcast, then use BAM control byte */
		ENUM_J1939_STATUS_CODES status = SAE_J1939_Send_Transport_Protocol_Connection_Management(j1939, DA);
		if (status != STATUS_SEND_OK) {
			return status;
		}

		/* Check if we are going to send it directly (BAM) */
		if (j1939->this_ecu_tp_cm.control_byte == CONTROL_BYTE_TP_CM_BAM) {
			j1939->from_other_ecu_tp_cm.control_byte = j1939->this_ecu_tp_cm.control_byte;
			return SAE_J1939_Send_Transport_Protocol_Data_Transfer(j1939, DA);
		}
		return status;
	}
}

/*
 * Store the Proprietary B about other ECU
 * PGN: 0x00FF00 <-> 0x00FFFF
 */
void SAE_J1939_Read_Response_Request_Proprietary_B(J1939* j1939, uint8_t SA, uint32_t PGN, uint8_t data[]) {
	struct Proprietary_B * proprietary_B = Get_Proprietary_B_By_PGN(&j1939->from_other_ecu_proprietary, PGN);

	if (proprietary_B == NULL) /* Proprietary B is not expected, don't fill the data anywhere */
	{
		return;
	}

	uint16_t total_bytes = proprietary_B->total_bytes;
	memcpy(proprietary_B->data, data, total_bytes);
	proprietary_B->from_ecu_address = SA;
  if (Callback_Function_Proprietary_B) {
    Callback_Function_Proprietary_B(proprietary_B);
  }
}
