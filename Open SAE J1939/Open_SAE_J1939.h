/*
 * SAE_J1939.h
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef OPEN_SAE_J1939_OPEN_SAE_J1939_H_
#define OPEN_SAE_J1939_OPEN_SAE_J1939_H_

/* Layers */
#include "../SAE J1939-71 Application Layer/SAE_J1939-71_Application_Layer.h"

/* Structs */
#include "Open_SAE_J1939_Structs.h"


/* These functions must be called all the time */
void Open_SAE_J1939_Listen_For_Messages(J1939* j1939);
void Open_SAE_J1939_Broadcast_Messages(uint16_t time);


#endif /* OPEN_SAE_J1939_OPEN_SAE_J1939_H_ */
