//###########################################################################
//
// FILE:    hw_cputimer.h
//
// TITLE:   Definitions for the CPUTIMER registers.
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

#ifndef HW_CPUTIMER_H
#define HW_CPUTIMER_H

//*************************************************************************************************
//
// The following are defines for the CPUTIMER register offsets
//
//*************************************************************************************************
#define CPUTIMER_O_TIM    0x0U   // CPU-Timer, Counter Register
#define CPUTIMER_O_PRD    0x2U   // CPU-Timer, Period Register
#define CPUTIMER_O_TCR    0x4U   // CPU-Timer, Control Register
#define CPUTIMER_O_TPR    0x6U   // CPU-Timer, Prescale Register
#define CPUTIMER_O_TPRH   0x7U   // CPU-Timer, Prescale Register High


//*************************************************************************************************
//
// The following are defines for the bit fields in the TIM register
//
//*************************************************************************************************
#define CPUTIMER_TIM_LSW_S   0U
#define CPUTIMER_TIM_LSW_M   0xFFFFU       // CPU-Timer Counter Registers
#define CPUTIMER_TIM_MSW_S   16U
#define CPUTIMER_TIM_MSW_M   0xFFFF0000U   // CPU-Timer Counter Registers High

//*************************************************************************************************
//
// The following are defines for the bit fields in the PRD register
//
//*************************************************************************************************
#define CPUTIMER_PRD_LSW_S   0U
#define CPUTIMER_PRD_LSW_M   0xFFFFU       // CPU-Timer Period Registers
#define CPUTIMER_PRD_MSW_S   16U
#define CPUTIMER_PRD_MSW_M   0xFFFF0000U   // CPU-Timer Period Registers High

//*************************************************************************************************
//
// The following are defines for the bit fields in the TCR register
//
//*************************************************************************************************
#define CPUTIMER_TCR_TSS    0x10U     // CPU-Timer stop status bit.
#define CPUTIMER_TCR_TRB    0x20U     // Timer reload
#define CPUTIMER_TCR_SOFT   0x400U    // Emulation modes
#define CPUTIMER_TCR_FREE   0x800U    // Emulation modes
#define CPUTIMER_TCR_TIE    0x4000U   // CPU-Timer Interrupt Enable.
#define CPUTIMER_TCR_TIF    0x8000U   // CPU-Timer Interrupt Flag.

//*************************************************************************************************
//
// The following are defines for the bit fields in the TPR register
//
//*************************************************************************************************
#define CPUTIMER_TPR_TDDR_S   0U
#define CPUTIMER_TPR_TDDR_M   0xFFU     // CPU-Timer Divide-Down.
#define CPUTIMER_TPR_PSC_S    8U
#define CPUTIMER_TPR_PSC_M    0xFF00U   // CPU-Timer Prescale Counter.

//*************************************************************************************************
//
// The following are defines for the bit fields in the TPRH register
//
//*************************************************************************************************
#define CPUTIMER_TPRH_TDDRH_S   0U
#define CPUTIMER_TPRH_TDDRH_M   0xFFU     // CPU-Timer Divide-Down.
#define CPUTIMER_TPRH_PSCH_S    8U
#define CPUTIMER_TPRH_PSCH_M    0xFF00U   // CPU-Timer Prescale Counter.



#endif
