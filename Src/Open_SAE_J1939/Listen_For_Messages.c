/*
 * Listen_For_Messages.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Open_SAE_J1939.h"

/* Layers */
#include "../ISO_11783/ISO_11783-7_Application_Layer/Application_Layer.h"
#include "../Hardware/Hardware.h"

/* This function should be called all the time, or be placed inside an interrupt listener */
ENUM_J1939_RX_MSG Open_SAE_J1939_Listen_For_Messages(J1939* j1939) {
	uint32_t ID = 0;
	uint8_t data[8] = {0};
	ENUM_J1939_RX_MSG rx_msg = RX_MSG_NONE;
	bool is_new_message = CAN_Read_Message(&ID, data);
	if(is_new_message) {
		/* Save latest */
		j1939->ID = ID;
		memcpy(j1939->data, data, 8);
		j1939->ID_and_data_is_updated = true;

		uint8_t id0 = ID >> 24;
		uint8_t id1 = ID >> 16;
		uint8_t DA = ID >> 8; 	/* Destination address which is this ECU. if DA = 0xFF = broadcast to all ECU. Sometimes DA can be an ID number too */
		uint8_t SA = ID; 		/* Source address of the ECU that we got the message from */

		uint16_t PGN = (id0 << 8) & id1;

		rx_msg = RX_MSG_NOT_SUPPORTED;

		/* Read request from other ECU */
		if (id0 == 0x18 && id1 == 0xEA && (DA == j1939->information_this_ECU.this_ECU_address || DA == 0xFF)){
			SAE_J1939_Read_Request(j1939, SA, data);
			rx_msg = RX_MSG_REQ;
		}else if (id0 == 0x18 && id1 == 0xD9 && DA == j1939->information_this_ECU.this_ECU_address){
			SAE_J1939_Read_Request_DM14(j1939, SA, data);
			rx_msg = RX_MSG_REQ_DM14;

		/* Read status from other ECU */
		}else if(id0 == 0x18 && id1 == 0xE8 && DA == j1939->information_this_ECU.this_ECU_address){
			SAE_J1939_Read_Acknowledgement(j1939, SA, data);
			rx_msg = RX_MSG_ACK;
		}else if (id0 == 0x18 && id1 == 0xD8 && DA == j1939->information_this_ECU.this_ECU_address){
			SAE_J1939_Read_Response_DM15(j1939, SA, data);
			rx_msg = RX_MSG_DM15;
		}else if(id0 == 0x18 && id1 == 0xD7 && DA == j1939->information_this_ECU.this_ECU_address){
			SAE_J1939_Read_Binary_Data_Transfer_DM16(j1939, SA, data);
			rx_msg = RX_MSG_DM16;

		/* Read Transport Protocol information from other ECU */
		}else if(id0 == 0x1C && id1 == 0xEC && (DA == j1939->information_this_ECU.this_ECU_address || DA == 0xFF)){
			SAE_J1939_Read_Transport_Protocol_Connection_Management(j1939, SA, data);
			rx_msg = RX_MSG_TP_CONN_MANAGEMENT;
		}else if (id0 == 0x1C && id1 == 0xEB && (DA == j1939->information_this_ECU.this_ECU_address || DA == 0xFF)){
			SAE_J1939_Read_Transport_Protocol_Data_Transfer(j1939, SA, data);
			rx_msg = RX_MSG_TP_CONN_DATA_TRANSFER;

		/* Read response request from other ECU - This are response request. They are responses from other ECU about request from this ECU */
		}else if (id0 == 0x14 && id1 == 0xEF && DA == 0x23) {
			SAE_J1939_Read_Response_Request_Proprietary_A(j1939, SA, data);										/* Manufacturer specific data */
			rx_msg = RX_MSG_RESP_REQ_PROPRIETARY_A;
		}else if (((PGN > PGN_PROPRIETARY_B_START) && (PGN < PGN_PROPRIETARY_B_END)) || 
				  ((PGN > PGN_PROPRIETARY_B2_START) && (PGN < PGN_PROPRIETARY_B2_END))
			 && (DA == j1939->information_this_ECU.this_ECU_address || DA == 0xFF)) {
			SAE_J1939_Read_Response_Request_Proprietary_B(j1939, SA, PGN, data);								/* Manufacturer specific data (B) */
			rx_msg = RX_MSG_RESP_REQ_PROPRIETARY_B;
		}else if (id0 == 0x18 && id1 == 0xEE && DA == 0xFF && SA != 0xFE){
			SAE_J1939_Read_Response_Request_Address_Claimed(j1939, SA, data);									/* This is a broadcast response request */
			rx_msg = RX_MSG_RESP_REQ_ADDR_CLAIMED;
		}else if (id0 == 0x18 && id1 == 0xEE && DA == 0xFF && SA == 0xFE) {
			SAE_J1939_Read_Address_Not_Claimed(j1939, SA, data);												/* This is error */
			rx_msg = RX_MSG_ADDR_NOT_CLAIMED;

		}else if (id0 == 0x18 && id1 == 0xFE && DA == 0xCA){
			SAE_J1939_Read_Response_Request_DM1(j1939, SA, data, 1); 											/* Assume that errors_dm1_active = 1 */
			rx_msg = RX_MSG_RESP_REQ_DM1;
		}else if (id0 == 0x18 && id1 == 0xFE && DA == 0xCB){
			SAE_J1939_Read_Response_Request_DM2(j1939, SA, data, 1); 											/* Assume that errors_dm2_active = 1 */
			rx_msg = RX_MSG_RESP_REQ_DM2;
		}else if (id0 == 0x18 && id1 == 0xFE && DA == 0xDA){
			SAE_J1939_Read_Response_Request_Software_Identification(j1939, SA, data);
			rx_msg = RX_MSG_RESP_REQ_SOFTWARE_IDENTIFICATION;
		}else if (id0 == 0x18 && id1 == 0xFD && DA == 0xC5){
			SAE_J1939_Read_Response_Request_ECU_Identification(j1939, SA, data);
			rx_msg = RX_MSG_RESP_REQ_ECU_IDENTIFICATION;
		}else if (id0 == 0x18 && id1 == 0xFE && DA == 0xEB){
			SAE_J1939_Read_Response_Request_Component_Identification(j1939, SA, data);
			rx_msg = RX_MSG_RESP_REQ_COMPONENT_IDENTIFICATION;
		}else if (id0 == 0x0C && id1 == 0xFE && DA >= 0x10 && DA <= 0x1F){
			ISO_11783_Read_Response_Request_Auxiliary_Estimated_Flow(j1939, SA, DA & 0xF, data);				/* DA & 0xF = Valve number. Total 16 valves from 0 to 15 */
			rx_msg = RX_MSG_RESP_REQ_AUX_ESTIMATED_FLOW;
		}else if (id0 == 0x0C && id1 == 0xC6 && DA == j1939->information_this_ECU.this_ECU_address){
			ISO_11783_Read_Response_Request_General_Purpose_Valve_Estimated_Flow(j1939, SA, data);
			rx_msg = RX_MSG_RESP_REQ_GP_VALVE_ESTIMATED_FLOW;
		}else if (id0 == 0x0C && id1 == 0xFF && DA >= 0x20 && DA <= 0x2F) {
			ISO_11783_Read_Response_Request_Auxiliary_Valve_Measured_Position(j1939, SA, DA & 0xF, data); 		/* DA & 0xF = Valve number. Total 16 valves from 0 to 15 */
			rx_msg = RX_MSG_RESP_REQ_AUX_VALVE_MEASURED_POSITION;

		/* Read command from other ECU */
		}else if (id0 == 0x0C && id1 == 0xFE && DA >= 0x30 && DA <= 0x3F){
			ISO_11783_Read_Auxiliary_Valve_Command(j1939, SA, DA & 0xF, data); 									/* DA & 0xF = Valve number. Total 16 valves from 0 to 15 */
			rx_msg = RX_MSG_AUX_VALVE_CMD;
		}else if (id0 == 0x0C && id1 == 0xC4 && DA == j1939->information_this_ECU.this_ECU_address){
			ISO_11783_Read_General_Purpose_Valve_Command(j1939, SA, data);										/* General Purpose Valve Command have only one valve */
			rx_msg = RX_MSG_GP_VALVE_CMD;
		}else if (id0 == 0x0 && id1 == 0x2 && (DA == j1939->information_this_ECU.this_ECU_address || DA == 0xFF)){
			SAE_J1939_Read_Address_Delete(j1939, data);															/* Not a SAE J1939 standard */
			rx_msg = RX_MSG_NOT_SAE_J1939;
		}else{
			rx_msg = RX_MSG_UNKNOWN;																			/* The message was not meant for this ECU */
		}
		/* Add more else if statement here */
	}
	return rx_msg;
}
