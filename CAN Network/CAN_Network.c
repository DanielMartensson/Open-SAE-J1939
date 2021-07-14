/*
 * CAN_Functions.c
 *
 *  Created on: 11 juli 2021
 *      Author: Daniel Mårtensson
 */

/* Select your processor here */
#define STM32 1
#define ARDUINO 2
#define PIC 3
#define AVR 4
#define PROCESSOR_CHOICE 1

/* Platform independent library headers for CAN */
#if PROCESSOR_CHOICE == STM32
#include "main.h"
#include "../../Functions.h"									/* This is important for STM32 PLC - You can delete this if you not running my STM32 PLC board */
#elif PROCESSOR_CHOICE == ARDUINO
#elif PROCESSOR_CHOICE == PIC
#elif PROCESSOR_CHOICE == AVR
#endif

/* Mandatory C header */
#include "CAN_Network.h"

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
	#endif
	return status;
}

/* PGN 00EA00 - Send a PGN request */
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
	#endif
	return is_new_message;
}
