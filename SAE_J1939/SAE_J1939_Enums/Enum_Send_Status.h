/*
 * Enum_Send_Status.h
 *
 *  Created on: 15 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef SAE_J1939_ENUMS_SAE_J1939_ENUM_SEND_STATUS_H_
#define SAE_J1939_ENUMS_SAE_J1939_ENUM_SEND_STATUS_H_

/* Send OK enum */
typedef enum {
	STATUS_SEND_OK = 0x00U,
	STATUS_SEND_ERROR = 0x01U,
	STATUS_SEND_BUSY = 0x02U,
	STATUS_SEND_TIMEOUT = 0x03U
} ENUM_J1939_STATUS_CODES;

#endif /* SAE_J1939_ENUMS_SAE_J1939_ENUM_SEND_STATUS_H_ */
