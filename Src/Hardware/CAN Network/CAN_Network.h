/*
 * CAN_Network.h
 *
 *  Created on: 14 juli 2021
 *      Author: danie
 */

#ifndef HARDWARE_CAN_NETWORK_H_
#define HARDWARE_CAN_NETWORK_H_

/* C standard library */
#include "stdint.h"
#include "stdbool.h"

/* Layers */
#include "../Processor_Choice.h"

/* Enums */
#include "../../SAE J1939/SAE J1939 Enums/Enum_Send_Status.h"

ENUM_J1939_STATUS_CODES CAN_Send_Message(uint32_t ID, uint8_t data[], uint8_t delay);
ENUM_J1939_STATUS_CODES CAN_Send_Request(uint32_t ID, uint8_t PGN[], uint8_t delay);
bool CAN_Read_Message(uint32_t *ID, uint8_t data[]);

#endif /* HARDWARE_CAN_NETWORK_H_ */
