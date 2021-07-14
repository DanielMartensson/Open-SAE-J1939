/*
 * SAE_J1939-73_Diagnostics_Layer.h
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef SAE_J1939_73_DIAGNOSTICS_LAYER_SAE_J1939_73_DIAGNOSTICS_LAYER_H_
#define SAE_J1939_73_DIAGNOSTICS_LAYER_SAE_J1939_73_DIAGNOSTICS_LAYER_H_

typedef enum {
	COMMAND_READ = 0x1,
	COMMAND_WRITE = 0x2,
	COMMAND_OPERATION_COMPLETED = 0x4,
	COMMAND_OPERATION_FAILED = 0x5,
	COMMAND_BOOT_LOAD = 0x6,
	POINTER_EXTENSION_FLASH_ACCESS = 0x0,
	POINTER_EXTENSION_EEPROM_ACCESS = 0x1,
	POINTER_EXTENSION_VARIABLE_ACCESS = 0x2,
	USER_KEY_LEVEL_USE_LONG_SEED_OR_DM18_SECURITY_MESSAGE = 0x0,
	USER_KEY_LEVEL_NO_KEY_AVAILABLE = 0xFFFF
}ENUM_DM14_CODES;



/* DM1 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_DM1(J1939 *j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_DM1(J1939* j1939, uint8_t DA);
void SAE_J1939_Read_Response_Request_DM1(J1939 *j1939, uint8_t SA, uint8_t data[], uint8_t errors_dm1_active);

/* DM2 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_DM2(J1939 *j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_DM2(J1939* j1939, uint8_t DA);
void SAE_J1939_Read_Response_Request_DM2(J1939 *j1939, uint8_t SA, uint8_t data[], uint8_t errors_dm1_active);

/* DM3 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_DM3(J1939 *j1939, uint8_t DA);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_DM3(J1939* j1939, uint8_t DA);

/* DM14 - Notice that DM14 missing a read response function. That's because DM15 is the read response request */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_DM14(J1939* j1939, uint8_t DA, uint16_t number_of_requested_bytes, uint8_t pointer_type, uint8_t command, uint32_t pointer, uint8_t pointer_extension, uint16_t key_user_level);
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_DM14(J1939 *j1939, uint8_t DA, uint8_t data[]);

/* DM15 - Notice that DM15 missing a send request function. That's because DM14 is the send request */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_DM15(J1939 *j1939, uint8_t DA, uint16_t number_of_allowed_bytes, uint8_t status, uint32_t error_indicator_EDC_parameter, uint8_t EDCP_extention, uint16_t seed);
void SAE_J1939_Read_Response_Request_DM15(J1939 *j1939, uint8_t SA, uint8_t data[]);

/* DM16 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Binary_Data_Transfer_DM16(J1939 *j1939, uint8_t DA, uint8_t number_of_occurences, uint8_t raw_binary_data[]);
void SAE_J1939_Read_Binary_Data_Transfer_DM16(J1939 *j1939, uint8_t SA, uint8_t data[]);


#endif /* SAE_J1939_73_DIAGNOSTICS_LAYER_SAE_J1939_73_DIAGNOSTICS_LAYER_H_ */
