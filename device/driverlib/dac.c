//###########################################################################
//
// FILE:   dac.c
//
// TITLE:  C28x DAC driver.
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

#include "dac.h"

//*****************************************************************************
//
// DAC_tuneOffsetTrim()
//
//*****************************************************************************

void
DAC_tuneOffsetTrim(uint32_t base, float32_t referenceVoltage)
{
    uint16_t oldOffsetTrim;
    float32_t newOffsetTrim;

    //
    // Check the arguments.
    //
    ASSERT(DAC_isBaseValid(base));
    ASSERT(referenceVoltage > 0U);

    //
    // Get the sign-extended offset trim value
    //
    oldOffsetTrim = (HWREGH(base + DAC_O_TRIM) & DAC_TRIM_OFFSET_TRIM_M);
    oldOffsetTrim = ((oldOffsetTrim & (uint16_t)DAC_REG_BYTE_MASK) ^
                    (uint16_t)0x80) - (uint16_t)0x80;

    //
    // Calculate new offset trim value if DAC is operating at a reference
    // voltage other than 2.5v.
    //
    newOffsetTrim = ((float32_t)(2.5 / referenceVoltage) *
                     (int16_t)oldOffsetTrim);

    //
    // Check if the new offset trim value is valid
    //
    ASSERT(((int16_t)newOffsetTrim > -129) && ((int16_t)newOffsetTrim < 128));

    //
    // Set the new offset trim value
    //
    EALLOW;
    HWREGH(base + DAC_O_TRIM) = (HWREGH(base + DAC_O_TRIM) &
                                 ~DAC_TRIM_OFFSET_TRIM_M) |
                                 (int16_t)newOffsetTrim;

    EDIS;

}

