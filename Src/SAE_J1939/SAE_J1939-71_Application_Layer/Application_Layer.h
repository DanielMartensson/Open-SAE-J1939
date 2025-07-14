/*
 * Application_Layer.h
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef SAE_J1939_71_APPLICATION_LAYER_SAE_J1939_71_APPLICATION_LAYER_H_
#define SAE_J1939_71_APPLICATION_LAYER_SAE_J1939_71_APPLICATION_LAYER_H_

/* Enums and structs */
#include "../../Open_SAE_J1939/Structs.h"
#include "../SAE_J1939_Enums/Enum_Control_Byte.h"
#include "../SAE_J1939_Enums/Enum_DM1_DM2.h"
#include "../SAE_J1939_Enums/Enum_DM14_DM15.h"
#include "../SAE_J1939_Enums/Enum_Group_Function_Value.h"
#include "../SAE_J1939_Enums/Enum_NAME.h"
#include "../SAE_J1939_Enums/Enum_PGN.h"
#include "../SAE_J1939_Enums/Enum_Send_Status.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void (*Callback_Function_Software_Identification)(struct Software_identification *);
extern void (*Callback_Function_ECU_Identification)(struct ECU_identification *);
extern void (*Callback_Function_Component_Identification)(struct Component_identification *);
extern void (*Callback_Function_Proprietary_A)(struct Proprietary_A *);
extern void (*Callback_Function_Proprietary_B)(struct Proprietary_B *);

/* Callback functions */
void SAE_J1939_Set_Callback_Functions(void (*Callback_Function_Software_Identification_)(struct Software_identification *), void (*Callback_Function_ECU_Identification_)(struct ECU_identification *), void (*Callback_Function_Component_Identification)(struct Component_identification *), void (*Callback_Function_Proprietary_A_)(struct Proprietary_A *), void (*Callback_Function_Proprietary_B_)(struct Proprietary_B *));

/* Software identification */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Software_Identification(J1939 *j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Software_Identification(J1939* j1939, uint8_t DA);
void SAE_J1939_Read_Response_Request_Software_Identification(J1939 *j1939, uint8_t SA, uint8_t data[]);

/* ECU identification */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_ECU_Identification(J1939 *j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_ECU_Identification(J1939* j1939, uint8_t DA);
void SAE_J1939_Read_Response_Request_ECU_Identification(J1939 *j1939, uint8_t SA, uint8_t data[]);

/* Component identification */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Component_Identification(J1939 *j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Component_Identification(J1939* j1939, uint8_t DA);
void SAE_J1939_Read_Response_Request_Component_Identification(J1939 *j1939, uint8_t SA, uint8_t data[]);

/* Proprietary A */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Proprietary_A(J1939* j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Proprietary_A(J1939* j1939, uint8_t DA);
void SAE_J1939_Read_Response_Request_Proprietary_A(J1939* j1939, uint8_t SA, uint8_t data[]);

/* Proprietary B */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Proprietary_B(J1939* j1939, uint8_t DA, uint32_t PGN);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Proprietary_B(J1939* j1939, uint8_t DA, uint32_t PGN, bool * is_supported);
void SAE_J1939_Read_Response_Request_Proprietary_B(J1939* j1939, uint8_t SA, uint32_t PGN, uint8_t data[]);
struct Proprietary_B * Get_Proprietary_B_By_PGN(struct Proprietary * proprietary, uint32_t PGN);

#ifdef __cplusplus
}
#endif

#endif /* SAE_J1939_71_APPLICATION_LAYER_SAE_J1939_71_APPLICATION_LAYER_H_ */
