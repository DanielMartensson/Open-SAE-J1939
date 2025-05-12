/*
 * Clibrary.h
 *
 *  Created on: 28 feb. 2023
 *      Author: Daniel Mårtensson
 */

#ifndef OPEN_SAE_J1939_C89_LIBRARY_H_
#define OPEN_SAE_J1939_C89_LIBRARY_H_

/* C standard library */
#include <time.h>
#include <string.h>

#ifndef __cplusplus

/* In ANSI C (C89), the __STDC_VERSION__ is not defined */
#ifndef __STDC_VERSION__
#define __STDC_VERSION__ 199409L		/* STDC version of C89 standard */
#endif

/* C99 has the __STDC_VERSION 199901L */
#if __STDC_VERSION__ < 199901L
/* Standard signed int and unsigned int */
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef signed char int8_t;
typedef signed short int16_t;
typedef signed int int32_t;

/* Standard bool */
typedef uint8_t bool;
#define true 1
#define false 0

#define SAE_J1939_INLINE

#else
/* C99 and above */
#include <stdbool.h>					/* For bool datatype */
#include <stdint.h>						/* For uint8_t, uint16_t and uint32_t */

#define SAE_J1939_INLINE inline

#endif
#endif

#endif /* OPEN_SAE_J1939_C89_LIBRARY_H_ */
