/*
 * SAE_J1939.h
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef OPEN_SAE_J1939_OPEN_SAE_J1939_H_
#define OPEN_SAE_J1939_OPEN_SAE_J1939_H_

/* Layers */
#include "../Open_SAE_J1939/Structs.h"
#include "../SAE_J1939/SAE_J1939-71_Application_Layer/Application_Layer.h"
#include "../SAE_J1939/SAE_J1939-73_Diagnostics_Layer/Diagnostics_Layer.h"
#include "../SAE_J1939/SAE_J1939-81_Network_Management_Layer/Network_Management_Layer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* This functions must be called all the time, or be placed inside an interrupt listener */
bool Open_SAE_J1939_Listen_For_Messages(J1939 *j1939);

#ifdef __cplusplus
}
#endif

#endif /* OPEN_SAE_J1939_OPEN_SAE_J1939_H_ */
