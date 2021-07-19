/*
 * SAE_J1939_Listen_For_Messages.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Open_SAE_J1939.h"
#include "../ISO 11783 Tractors And Machinery For Agriculture And Forestry/ISO 11783-7 Implement Messages Application Layer/ISO_11783-7_Implement_Messages_Application_Layer.h"

/* This function should be called all the time */
void Open_SAE_J1939_Listen_For_Messages(J1939* j1939) {
	uint32_t ID = 0;
	uint8_t data[8] = {0};
	if(CAN_Read_Message(&ID, data)){
		uint8_t id0 = ID >> 24;
		uint8_t id1 = ID >> 16;
		uint8_t DA = ID >> 8; 	/* Destination address which is this ECU. if DA = 0xFF = broadcast to all ECU. Sometimes DA can be an ID number too */
		uint8_t SA = ID; 		/* Source address of the ECU that we got the message from */

		/* Read request from other ECU */
		if (id0 == 0x18 && id1 == 0xEA && DA == j1939->this_ECU_address)
			SAE_J1939_Read_Request(j1939, SA, data);
		else if (id0 == 0x18 && id1 == 0xD9 && DA == j1939->this_ECU_address)
			SAE_J1939_Read_Request_DM14(j1939, SA, data);

		/* Read status information from other ECU */
		else if(id0 == 0x18 && id1 == 0xE8 && DA == j1939->this_ECU_address)
			SAE_J1939_Read_Acknowledgement(j1939, data);
		else if (id0 == 0x18 && id1 == 0xD8 && DA == j1939->this_ECU_address)
			SAE_J1939_Read_Response_DM15(j1939, data);
		else if (id0 == 0x0C && id1 == 0xFE && DA >= 0x10 && DA <= 0x1F)
			ISO_11783_Read_Auxiliary_Estimated_Flow(j1939, DA & 0xF, data);				/* DA & 0xF = Valve number. Total 16 valves from 0 to 15 */
		else if (id0 == 0x0C && id1 == 0xC6 && DA == j1939->this_ECU_address)
			ISO_11783_Read_General_Purpose_Valve_Estimated_Flow(j1939, data);
		else if (id0 == 0x0C && id1 == 0xFF && DA >= 0x20 && DA <= 0x2F)
			ISO_11783_Read_Auxiliary_Valve_Measured_Position(j1939, DA & 0xF, data); 	/* DA & 0xF = Valve number. Total 16 valves from 0 to 15 */

		/* Read Transport Protocol information from other ECU */
		else if(id0 == 0x1C && id1 == 0xEC && DA == j1939->this_ECU_address)
			SAE_J1939_Read_Transport_Protocol_Connection_Management(j1939, SA, data);
		else if (id0 == 0x1C && id1 == 0xEB && DA == j1939->this_ECU_address)
			SAE_J1939_Read_Transport_Protocol_Data_Transfer(j1939, SA, data);

		/* Read response request from other ECU - This are response request. They are responses from other ECU about request from this ECU */
		else if (id0 == 0x18 && id1 == 0xEE && DA == 0xFF && SA != 0xFE)
			SAE_J1939_Read_Response_Request_Address_Claimed(j1939, SA, data);				/* This is a broadcast response request */
		else if (id0 == 0x18 && id1 == 0xFE && DA == 0xCA)
			SAE_J1939_Read_Response_Request_DM1(j1939, data, 1); 							/* Assume that errors_dm1_active = 1 */
		else if (id0 == 0x18 && id1 == 0xFE && DA == 0xCB)
			SAE_J1939_Read_Response_Request_DM2(j1939, data, 1); 							/* Assume that errors_dm2_active = 1 */
		else if (id0 == 0x18 && id1 == 0xFE && DA == 0xDA)
			SAE_J1939_Read_Response_Request_Software_Identification(j1939, data);
		else if (id0 == 0x18 && id1 == 0xFD && DA == 0xC5)
			SAE_J1939_Read_Response_Request_ECU_Identification(j1939, data);
		else if (id0 == 0x18 && id1 == 0xFE && DA == 0xEB)
			SAE_J1939_Read_Response_Request_Component_Identification(j1939, data);


		/* Read command from other ECU */
		else if (id0 == 0x0C && id1 == 0xFE && DA >= 0x30 && DA <= 0x3F)
			ISO_11783_Read_Auxiliary_Valve_Command(j1939, DA & 0xF, data); 					/* DA & 0xF = Valve number. Total 16 valves from 0 to 15 */
		else if (id0 == 0x0C && id1 == 0xC4 && DA == j1939->this_ECU_address)
			ISO_11783_Read_General_Purpose_Valve_Command(j1939, data);						/* General Purpose Valve Command have only one valve */

		/* Other */
		else if (id0 == 0x18 && id1 == 0xEE && DA == 0xFF && SA == 0xFE)
			j1939->number_of_cannot_claim_address++;

		/* Add more else if statement here */
	}
}
