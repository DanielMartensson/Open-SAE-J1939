/*
 * SAE_J1939.h
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef SAE_J1939_SAE_J1939_H_
#define SAE_J1939_SAE_J1939_H_

#include "SAE_J1939_Structs.h"

/* These functions must be called all the time */
void SAE_J1939_Listen_For_Messages(J1939* j1939);
void SAE_J1939_Broadcast_Messages(uint16_t time);


#endif /* SAE_J1939_SAE_J1939_H_ */
