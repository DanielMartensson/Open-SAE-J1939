/*
 * SAE_J1939.h
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef OPEN_SAE_J1939_OPEN_SAE_J1939_H_
#define OPEN_SAE_J1939_OPEN_SAE_J1939_H_

/* Layers */
#include "../SAE J1939/SAE J1939-71 Application Layer/Application_Layer.h"
#include "../SAE J1939/SAE J1939-81 Network Management Layer/Network_Management_Layer.h"
#include "../SAE J1939/SAE J1939-73 Diagnostics Layer/Diagnostics_Layer.h"

/* Structs */
#include "Structs.h"


/* This functions must be called all the time */
void Open_SAE_J1939_Listen_For_Messages(J1939 *j1939);


#endif /* OPEN_SAE_J1939_OPEN_SAE_J1939_H_ */
