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
	raw_binary_data[0] = 0x4C; /* L */
	raw_binary_data[1] = 0x6F; /* o */
	raw_binary_data[2] = 0x6F; /* o */
	raw_binary_data[3] = 0x6B; /* k */
	raw_binary_data[4] = 0x20; /*   */
	raw_binary_data[5] = 0x61; /* a */
	raw_binary_data[6] = 0x74; /* t */
	raw_binary_data[7] = 0x20; /*   */
	raw_binary_data[8] = 0x22; /* " */
	raw_binary_data[9] = 0x46; /* F */
	raw_binary_data[10] = 0x4C; /* L */
	raw_binary_data[11] = 0x41; /* A */
	raw_binary_data[12] = 0x53; /* S */
	raw_binary_data[13] = 0x48; /* H */
	raw_binary_data[14] = 0x5F; /* _ */
	raw_binary_data[15] = 0x45; /* E */
	raw_binary_data[16] = 0x45; /* E */
	raw_binary_data[17] = 0x50; /* P */
	raw_binary_data[18] = 0x52; /* R */
	raw_binary_data[19] = 0x4F; /* O */
	raw_binary_data[20] = 0x4D; /* M */
	raw_binary_data[21] = 0x5F; /*   */
	raw_binary_data[22] = 0x52; /* R */
	raw_binary_data[23] = 0x41; /* A */
	raw_binary_data[24] = 0x4D; /* M */
	raw_binary_data[25] = 0x5F; /* _ */
	raw_binary_data[26] = 0x4D; /* M */
	raw_binary_data[27] = 0x65; /* e */
	raw_binary_data[28] = 0x6D; /* m */
	raw_binary_data[29] = 0x6F; /* o */
	raw_binary_data[30] = 0x72; /* r */
	raw_binary_data[31] = 0x79; /* y */
	raw_binary_data[32] = 0x2E; /* . */
	raw_binary_data[33] = 0x63; /* c */
	raw_binary_data[34] = 0x22; /* " */
	raw_binary_data[35] = 0x0; /* Null termination C-string */
	*number_of_requested_bytes = 40;
	*command = STATUS_DM15_PROCEED;
#endif
}


