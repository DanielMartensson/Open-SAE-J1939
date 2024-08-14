//###########################################################################
//
// FILE:   cpu.h
//
// TITLE:  Useful C28x CPU defines.
//
//###########################################################################
// $Copyright:
// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

#ifndef CPU_H
#define CPU_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"

//
// External reference to the interrupt flag register (IFR) register
//
#ifndef __TMS320C28XX_CLA__
extern __cregister volatile uint16_t IFR;
#endif

//
// External reference to the interrupt enable register (IER) register
//
#ifndef __TMS320C28XX_CLA__
extern __cregister volatile uint16_t IER;
#endif

//
// Define to enable interrupts
//
#ifndef EINT
#define EINT   __asm(" clrc INTM")
#endif

//
// Define to disable interrupts
//
#ifndef DINT
#define DINT   __asm(" setc INTM")
#endif

//
// Define to enable debug events
//
#ifndef ERTM
#define ERTM   __asm(" clrc DBGM")
#endif

//
// Define to disable debug events
//
#ifndef DRTM
#define DRTM   __asm(" setc DBGM")
#endif

//
// Define to allow writes to protected registers
//
#ifndef EALLOW
#ifndef __TMS320C28XX_CLA__
#define EALLOW __eallow()
#else
#define EALLOW __meallow()
#endif // __TMS320C28XX_CLA__
#endif // EALLOW

//
// Define to disable writes to protected registers
//
#ifndef EDIS
#ifndef __TMS320C28XX_CLA__
#define EDIS   __edis()
#else
#define EDIS   __medis()
#endif // __TMS320C28XX_CLA__
#endif // EDIS

//
// Define for emulation stop
//
#ifndef ESTOP0
#define ESTOP0 __asm(" ESTOP0")
#endif

//
// Define for emulation stop
//
#ifndef ESTOP1
#define ESTOP1 __asm(" ESTOP1")
#endif

//
// Define for no operation
//
#ifndef NOP
#define NOP    __asm(" NOP")
#endif

//
// Define for putting processor into a low-power mode
//
#ifndef _DUAL_HEADERS
#ifndef IDLE
#define IDLE   __asm(" IDLE")
#endif
#else
#define IDLE_ASM __asm(" IDLE");
#endif

//*****************************************************************************
//
// Extern compiler intrinsic prototypes. See compiler User's Guide for details.
//
//*****************************************************************************
extern void __eallow(void);
extern void __edis(void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // CPU_H
