/*
 * Hardware.h
 *
 *  Created on: 6 nov. 2021
 *      Author: Daniel Mårtensson
 */

#ifndef HARDWARE_HARDWARE_H_
#define HARDWARE_HARDWARE_H_

/* Select your processor choice here */
#define NO_PROCESSOR 0
#define STM32 1
#define ARDUINO 2
#define PIC 3
#define AVR 4
#define QT_USB 5
#define INTERNAL_CALLBACK 6
#define PROCESSOR_CHOICE NO_PROCESSOR

/* C Standard library */
#include "../Open_SAE_J1939/C89_Library.h"

/* Enums */
#include "../SAE_J1939/SAE_J1939_Enums/Enum_DM14_DM15.h"
#include "../SAE_J1939/SAE_J1939_Enums/Enum_Send_Status.h"

#ifdef __cplusplus
extern "C" {
#endif

ENUM_J1939_STATUS_CODES CAN_Send_Message(uint32_t ID, uint8_t data[]);
ENUM_J1939_STATUS_CODES CAN_Send_Request(uint32_t ID, uint8_t PGN[]);
bool CAN_Read_Message(uint32_t *ID, uint8_t data[]);
void CAN_Delay(uint8_t milliseconds);
void CAN_Set_Callback_Functions(void (*Callback_Function_Send_)(uint32_t, uint8_t, uint8_t[]), void (*Callback_Function_Read_)(uint32_t*, uint8_t[], bool*), void (*Callback_Function_Traffic_)(uint32_t, uint8_t, uint8_t[], bool), void (*Callback_Function_Delay_ms_)(uint8_t));
void FLASH_EEPROM_RAM_Memory(uint16_t *number_of_requested_bytes, uint8_t pointer_type, uint8_t *command, uint32_t *pointer, uint8_t *pointer_extension, uint16_t *key, uint8_t raw_binary_data[]);
bool Save_Struct(uint8_t data[], uint32_t data_length, char file_name[]);
bool Load_Struct(uint8_t data[], uint32_t data_length, char file_name[]);

#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_HARDWARE_H_ */
