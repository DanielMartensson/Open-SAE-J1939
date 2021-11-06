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
	STATUS_SEND_OK = 0x00,
	STATUS_SEND_ERROR = 0x01,
	STATUS_SEND_BUSY = 0x02,
	STATUS_SEND_TIMEOUT = 0x03
} ENUM_J1939_STATUS_CODES;

#endif /* SAE_J1939_ENUMS_SAE_J1939_ENUM_SEND_STATUS_H_ */
