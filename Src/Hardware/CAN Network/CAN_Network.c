/*
 * CAN_Functions.c
 *
 *  Created on: 11 juli 2021
 *      Author: Daniel Mårtensson
 */

/* Layer */
#include "../CAN Network/CAN_Network.h"

/* Platform independent library headers for CAN */
#if PROCESSOR_CHOICE == STM32
#include "main.h"
#include "../../Functions.h"									/* This is important for STM32 PLC - You can delete this if you not running my STM32 PLC board */
#elif PROCESSOR_CHOICE == ARDUINO
#elif PROCESSOR_CHOICE == PIC
#elif PROCESSOR_CHOICE == AVR
#else
/* Internal fields */
static bool internal_new_message[255] = {false};
static uint8_t internal_data[255*8] = {0};
static uint8_t internal_DLC[255] = {0};
static uint32_t internal_ID[255] = {0};
static uint8_t buffer_index = 0;

/* Internal functions */
static ENUM_J1939_STATUS_CODES Internal_Transmit(uint32_t ID, uint8_t data[], uint8_t DLC) {
	internal_ID[buffer_index] = ID;
	internal_DLC[buffer_index] = DLC;
	for(uint8_t i = 0; i < 8; i++)
		if(i < DLC)
			internal_data[buffer_index*8 + i] = data[i];
		else
			internal_data[buffer_index*8 + i] = 0x0;
	internal_new_message[buffer_index] = true;
	buffer_index++;
	return STATUS_SEND_OK;
}

static void Internal_Receive(uint32_t *ID, uint8_t data[], bool *is_new_message) {
	buffer_index--;
	*ID = internal_ID[buffer_index];
	for(uint8_t i = 0; i < 8; i++)
		if(i < internal_DLC[buffer_index])
			data[i] = internal_data[buffer_index*8 + i];
	*is_new_message = internal_new_message[buffer_index];
	internal_new_message[buffer_index] = false;
}
#endif

ENUM_J1939_STATUS_CODES CAN_Send_Message(uint32_t ID, uint8_t data[], uint8_t delay) {
	ENUM_J1939_STATUS_CODES status;
	#if PROCESSOR_CHOICE == STM32
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.DLC = 8;											/* Here we are sending 8 bytes */
	TxHeader.RTR = CAN_RTR_DATA;								/* Data frame */
	TxHeader.IDE = CAN_ID_EXT;									/* We want to send an extended ID */
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.ExtId = ID;
	TxHeader.StdId = 0x00; 										/* Not used */
	status = STM32_PLC_CAN_Transmit(data, &TxHeader);
	HAL_Delay(delay);											/* A delay for messages are required sometimes */
	#elif PROCESSOR_CHOICE == ARDUINO
	/* Implement your CAN send 8 bytes message function for the Arduino platform */
	#elif PROCESSOR_CHOICE == PIC
	/* Implement your CAN send 8 bytes message function for the PIC platform */
	#elif PROCESSOR_CHOICE == AVR
	/* Implement your CAN send 8 bytes message function for the AVR platform */
	#else
	/* If no processor are used, use internal feedback for debugging */
	status = Internal_Transmit(ID, data, 8);
	#endif
	return status;
}

/* Send a PGN request
 * PGN: 0x00EA00 (59904)
 */
ENUM_J1939_STATUS_CODES CAN_Send_Request(uint32_t ID, uint8_t PGN[], uint8_t delay) {
	ENUM_J1939_STATUS_CODES status;
	#if PROCESSOR_CHOICE == STM32
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.DLC = 3;											/* Here we are only sending 3 bytes */
	TxHeader.RTR = CAN_RTR_DATA;								/* Data frame */
	TxHeader.IDE = CAN_ID_EXT;									/* We want to send an extended ID */
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.ExtId = ID;
	TxHeader.StdId = 0x00; 										/* Not used */
	status = STM32_PLC_CAN_Transmit(PGN, &TxHeader);
	HAL_Delay(delay);											/* A delay for messages are required sometimes */
	#elif PROCESSOR_CHOICE == ARDUINO
	/* Implement your CAN send 3 bytes message function for the Arduino platform */
	#elif PROCESSOR_CHOICE == PIC
	/* Implement your CAN send 3 bytes message function for the PIC platform */
	#elif PROCESSOR_CHOICE == AVR
	/* Implement your CAN send 3 bytes message function for the AVR platform */
	#else
	/* If no processor are used, use internal feedback for debugging */
	status = Internal_Transmit(ID, PGN, 3);
	#endif
	return status;
}

/* Read the current CAN-bus message. Returning false if the message has been read before, else true */
bool CAN_Read_Message(uint32_t *ID, uint8_t data[]) {
	bool is_new_message;
	#if PROCESSOR_CHOICE == STM32
	STM32_PLC_CAN_Get_ID_Data(ID, data, &is_new_message);
	#elif PROCESSOR_CHOICE == ARDUINO
	/* Implement your CAN function to get ID, data[] and the flag is_new_message here for the Arduino platform */
	#elif PROCESSOR_CHOICE == PIC
	/* Implement your CAN function to get ID, data[] and the flag is_new_message here for the PIC platform */
	#elif PROCESSOR_CHOICE == AVR
	/* Implement your CAN function to get ID, data[] and the flag is_new_message here for the AVR platform */
	#else
	/* If no processor are used, use internal feedback for debugging */
	Internal_Receive(ID, data, &is_new_message);
	#endif
	return is_new_message;
}
