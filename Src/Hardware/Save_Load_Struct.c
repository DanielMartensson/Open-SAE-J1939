/*
 * Save_Load_Struct.c
 *
 *  Created on: 22 sep. 2021
 *      Author: Daniel Mårtensson
 */

#include "Hardware.h"

/* Layers */
#include <stdio.h>

bool Save_Struct(uint8_t data[], uint32_t data_length, char file_name[]){
#if OPENSAE_J1939_TARGET_PLATFORM == STM32
	/* Save it to SD card */
	if(STM32_PLC_SD_Mont_Card() != FR_OK){
		return false;
	}
	STM32_PLC_SD_Create_File_With_Write(file_name);
	STM32_PLC_SD_Write_Data(data, data_length);
	STM32_PLC_SD_Close_File();
	STM32_PLC_SD_Unmount_Card();
	return true;
#elif OPENSAE_J1939_TARGET_PLATFORM == ARDUINO
	/* Implement your memory handler function for the Arduino platform */
#elif OPENSAE_J1939_TARGET_PLATFORM == PIC
	/* Implement your memory handler function for the PIC platform */
#elif OPENSAE_J1939_TARGET_PLATFORM == AVR
	/* Implement your memory handler function for the AVR platform */
#else
	/* Write a file */
	FILE *file = NULL;
	file = fopen(file_name, "wb");
	if (file == NULL) {
		return false;
	}
	fwrite(data, 1, data_length, file);
	fclose(file);
	return true;
#endif
}

bool Load_Struct(uint8_t data[], uint32_t data_length, char file_name[]){
#if OPENSAE_J1939_TARGET_PLATFORM == STM32
	/* Load it from SD card */
	if(STM32_PLC_SD_Mont_Card() != FR_OK){
		return false;
	}
	STM32_PLC_SD_Open_File_With_Read(file_name);
	STM32_PLC_SD_Read_Data(data, data_length);
	STM32_PLC_SD_Close_File();
	STM32_PLC_SD_Unmount_Card();
	return true;
#elif OPENSAE_J1939_TARGET_PLATFORM == ARDUINO
	/* Implement your memory handler function for the Arduino platform */
#elif OPENSAE_J1939_TARGET_PLATFORM == PIC
	/* Implement your memory handler function for the PIC platform */
#elif OPENSAE_J1939_TARGET_PLATFORM == AVR
	/* Implement your memory handler function for the AVR platform */
#else
	/* Read a file */
	FILE *file = NULL;
	file = fopen(file_name, "rb");
	if(file == NULL){
		file = fopen(file_name, "wb");
		if (file == NULL) {
			return false;
		}
	}
	fread(data, 1, data_length, file);
	fclose(file);
	return true;
#endif
}
