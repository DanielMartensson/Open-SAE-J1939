/*
 * Save_Load_J1939_Struct.c
 *
 *  Created on: 22 sep. 2021
 *      Author: Daniel Mårtensson
 */

#include "CAN_Memory.h"

void Save_Struct_To_SD_Card(J1939* j1939){
#if PROCESSOR_CHOICE == STM32
	/* Implement your memory handler function for the STM32 platform */
#elif PROCESSOR_CHOICE == ARDUINO
	/* Implement your memory handler function for the Arduino platform */
#elif PROCESSOR_CHOICE == PIC
	/* Implement your memory handler function for the PIC platform */
#elif PROCESSOR_CHOICE == AVR
	/* Implement your memory handler function for the AVR platform */
#endif
}

void Load_Struct_From_SD_Card(J1939* j1939){
#if PROCESSOR_CHOICE == STM32
	/* Implement your memory handler function for the STM32 platform */
#elif PROCESSOR_CHOICE == ARDUINO
	/* Implement your memory handler function for the Arduino platform */
#elif PROCESSOR_CHOICE == PIC
	/* Implement your memory handler function for the PIC platform */
#elif PROCESSOR_CHOICE == AVR
	/* Implement your memory handler function for the AVR platform */
#endif
}

