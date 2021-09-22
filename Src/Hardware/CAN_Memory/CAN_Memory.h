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

#include "../../SAE_J1939/SAE_J1939_Enums/Enum_DM14_DM15.h"
#include "../Processor_Choice.h"

#ifdef __cplusplus
extern "C" {
#endif

void FLASH_EEPROM_RAM_Memory(uint16_t *number_of_requested_bytes, uint8_t pointer_type, uint8_t *command, uint32_t *pointer, uint8_t *pointer_extension, uint16_t *key, uint8_t raw_binary_data[]);
void Save_Struct_To_SD_Card(J1939* j1939);
void Load_Struct_From_SD_Card(J1939* j1939);

#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_CAN_MEMORY_CAN_MEMORY_H_ */
