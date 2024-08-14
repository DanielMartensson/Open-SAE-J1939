//###########################################################################
//
// FILE:    hw_dac.h
//
// TITLE:   Definitions for the DAC registers.
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

#ifndef HW_DAC_H
#define HW_DAC_H

//*************************************************************************************************
//
// The following are defines for the DAC register offsets
//
//*************************************************************************************************
#define DAC_O_REV     0x0U   // DAC Revision Register
#define DAC_O_CTL     0x1U   // DAC Control Register
#define DAC_O_VALA    0x2U   // DAC Value Register - Active
#define DAC_O_VALS    0x3U   // DAC Value Register - Shadow
#define DAC_O_OUTEN   0x4U   // DAC Output Enable Register
#define DAC_O_LOCK    0x5U   // DAC Lock Register
#define DAC_O_TRIM    0x6U   // DAC Trim Register


//*************************************************************************************************
//
// The following are defines for the bit fields in the DACREV register
//
//*************************************************************************************************
#define DAC_REV_REV_S   0U
#define DAC_REV_REV_M   0xFFU   // DAC Revision Register

//*************************************************************************************************
//
// The following are defines for the bit fields in the DACCTL register
//
//*************************************************************************************************
#define DAC_CTL_DACREFSEL   0x1U    // DAC Reference Select
#define DAC_CTL_LOADMODE    0x4U    // DACVALA Load Mode
#define DAC_CTL_SYNCSEL_S   4U
#define DAC_CTL_SYNCSEL_M   0xF0U   // DAC EPWMSYNCPER Select

//*************************************************************************************************
//
// The following are defines for the bit fields in the DACVALA register
//
//*************************************************************************************************
#define DAC_VALA_DACVALA_S   0U
#define DAC_VALA_DACVALA_M   0xFFFU   // DAC Active Output Code

//*************************************************************************************************
//
// The following are defines for the bit fields in the DACVALS register
//
//*************************************************************************************************
#define DAC_VALS_DACVALS_S   0U
#define DAC_VALS_DACVALS_M   0xFFFU   // DAC Shadow Output Code

//*************************************************************************************************
//
// The following are defines for the bit fields in the DACOUTEN register
//
//*************************************************************************************************
#define DAC_OUTEN_DACOUTEN   0x1U   // DAC Output Code

//*************************************************************************************************
//
// The following are defines for the bit fields in the DACLOCK register
//
//*************************************************************************************************
#define DAC_LOCK_DACCTL     0x1U      // DAC Control Register Lock
#define DAC_LOCK_DACVAL     0x2U      // DAC Value Register Lock
#define DAC_LOCK_DACOUTEN   0x4U      // DAC Output Enable Register Lock
#define DAC_LOCK_KEY_S      12U
#define DAC_LOCK_KEY_M      0xF000U   // DAC Register Lock Key

//*************************************************************************************************
//
// The following are defines for the bit fields in the DACTRIM register
//
//*************************************************************************************************
#define DAC_TRIM_OFFSET_TRIM_S   0U
#define DAC_TRIM_OFFSET_TRIM_M   0xFFU   // DAC Offset Trim



#endif
