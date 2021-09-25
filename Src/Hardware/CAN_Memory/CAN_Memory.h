/*
 * FLASH_EEPROM_RAM_Memory.h
 *
 *  Created on: 19 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef HARDWARE_CAN_MEMORY_CAN_MEMORY_H_
#define HARDWARE_CAN_MEMORY_CAN_MEMORY_H_

/* C standard library */
#include <stdint.h>
#include <stdbool.h>

#include "../../SAE_J1939/SAE_J1939_Enums/Enum_DM14_DM15.h"
#include "../Processor_Choice.h"
#include "../../Open_SAE_J1939/Structs.h"

/* This text name follows 8.3 filename standard - Important if you want to save to SD card */
#define INFORMATION_THIS_ECU "ECUINFO.TXT"

#ifdef __cplusplus
extern "C" {
#endif

void FLASH_EEPROM_RAM_Memory(uint16_t *number_of_requested_bytes, uint8_t pointer_type, uint8_t *command, uint32_t *pointer, uint8_t *pointer_extension, uint16_t *key, uint8_t raw_binary_data[]);
bool Save_Struct(uint8_t data[], uint32_t data_length, char file_name[]);
bool Load_Struct(uint8_t data[], uint32_t data_length, char file_name[]);

#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_CAN_MEMORY_CAN_MEMORY_H_ */
