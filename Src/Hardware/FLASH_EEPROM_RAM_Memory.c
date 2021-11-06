/*
 * FLASH_EEPROM_RAM_Memory.c
 *
 *  Created on: 19 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Hardware.h"

/*
 * This function ask the hardware for memory access with pointers.
 * Those pointers are translated to DM15 memory responses after the function exist
 * DM14:						DM15:
 * number_of_requested_bytes -> number_of_allowed_bytes
 * command					 -> status
 * pointer					 -> EDC_parameter
 * pointer_extension		 -> EDCP_extention
 * key						 -> seed
 */
void FLASH_EEPROM_RAM_Memory(uint16_t *number_of_requested_bytes, uint8_t pointer_type, uint8_t *command, uint32_t *pointer, uint8_t *pointer_extension, uint16_t *key, uint8_t raw_binary_data[]){
#if PROCESSOR_CHOICE == STM32
	/* Implement your memory handler function for the STM32 platform */
#elif PROCESSOR_CHOICE == ARDUINO
	/* Implement your memory handler function for the Arduino platform */
#elif PROCESSOR_CHOICE == PIC
	/* Implement your memory handler function for the PIC platform */
#elif PROCESSOR_CHOICE == AVR
	/* Implement your memory handler function for the AVR platform */
#else
	/* If no processor are used, use internal feedback for debugging */
	*command = STATUS_DM15_PROCEED;
#endif
}


