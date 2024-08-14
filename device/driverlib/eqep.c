//###########################################################################
//
// FILE:   eqep.c
//
// TITLE:  C28x eQEP driver.
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

#include "eqep.h"

//*****************************************************************************
//
// EQEP_setCompareConfig
//
//*****************************************************************************
void
EQEP_setCompareConfig(uint32_t base, uint16_t config, uint32_t compareValue,
                      uint16_t cycles)
{
    uint16_t regValue;

    //
    // Check the arguments.
    //
    ASSERT(EQEP_isBaseValid(base));
    ASSERT(cycles <= (EQEP_QPOSCTL_PCSPW_M + 1U));

    //
    // Set the compare match value
    //
    HWREG(base + EQEP_O_QPOSCMP) = compareValue;

    //
    // Set the shadow register settings and pulse width.
    //
    regValue = (config & (uint16_t)(EQEP_QPOSCTL_PCSHDW |
                  EQEP_QPOSCTL_PCLOAD)) | (cycles - 1U);

    HWREGH(base + EQEP_O_QPOSCTL) = (HWREGH(base + EQEP_O_QPOSCTL) &
                                     ~(EQEP_QPOSCTL_PCSPW_M |
                                       EQEP_QPOSCTL_PCLOAD |
                                       EQEP_QPOSCTL_PCSHDW)) | regValue;

    //
    // Set position compare sync-output mode.
    //
    regValue = config & (uint16_t)(EQEP_QDECCTL_SOEN | EQEP_QDECCTL_SPSEL);

    HWREGH(base + EQEP_O_QDECCTL) = (HWREGH(base + EQEP_O_QDECCTL) &
                                     ~(EQEP_QDECCTL_SOEN |
                                       EQEP_QDECCTL_SPSEL)) | regValue;
}

//*****************************************************************************
//
// EQEP_setInputPolarity
//
//*****************************************************************************
void
EQEP_setInputPolarity(uint32_t base, bool invertQEPA, bool invertQEPB,
                      bool invertIndex, bool invertStrobe)
{
    //
    // Check the arguments.
    //
    ASSERT(EQEP_isBaseValid(base));

    //
    // Configure QEPA signal
    //
    if(invertQEPA)
    {
        HWREGH(base + EQEP_O_QDECCTL) |= EQEP_QDECCTL_QAP;
    }
    else
    {
        HWREGH(base + EQEP_O_QDECCTL) &= ~EQEP_QDECCTL_QAP;
    }

    //
    // Configure QEPB signal
    //
    if(invertQEPB)
    {
        HWREGH(base + EQEP_O_QDECCTL) |= EQEP_QDECCTL_QBP;
    }
    else
    {
        HWREGH(base + EQEP_O_QDECCTL) &= ~EQEP_QDECCTL_QBP;
    }

    //
    // Configure index signal
    //
    if(invertIndex)
    {
        HWREGH(base + EQEP_O_QDECCTL) |= EQEP_QDECCTL_QIP;
    }
    else
    {
        HWREGH(base + EQEP_O_QDECCTL) &= ~EQEP_QDECCTL_QIP;
    }

    //
    // Configure strobe signal
    //
    if(invertStrobe)
    {
        HWREGH(base + EQEP_O_QDECCTL) |= EQEP_QDECCTL_QSP;
    }
    else
    {
        HWREGH(base + EQEP_O_QDECCTL) &= ~EQEP_QDECCTL_QSP;
    }
}
