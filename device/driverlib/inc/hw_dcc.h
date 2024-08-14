//###########################################################################
//
// FILE:    hw_dcc.h
//
// TITLE:   Definitions for the DCC registers.
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

#ifndef HW_DCC_H
#define HW_DCC_H

//*************************************************************************************************
//
// The following are defines for the DCC register offsets
//
//*************************************************************************************************
#define DCC_O_GCTRL        0x0U    // Starts / stops the counters. Clears the error signal.
#define DCC_O_CNTSEED0     0x8U    // Seed value for the counter attached to Clock Source 0.
#define DCC_O_VALIDSEED0   0xCU    // Seed value for the timeout counter attached to Clock Source
                                   // 0.
#define DCC_O_CNTSEED1     0x10U   // Seed value for the counter attached to Clock Source 1.
#define DCC_O_STATUS       0x14U   // Specifies the status of the DCC Module.
#define DCC_O_CNT0         0x18U   // Value of the counter attached to Clock Source 0.
#define DCC_O_VALID0       0x1CU   // Value of the valid counter attached to Clock Source 0.
#define DCC_O_CNT1         0x20U   // Value of the counter attached to Clock Source 1.
#define DCC_O_CLKSRC1      0x24U   // Selects the clock source for Counter 1.
#define DCC_O_CLKSRC0      0x28U   // Selects the clock source for Counter 0.


//*************************************************************************************************
//
// The following are defines for the bit fields in the DCCGCTRL register
//
//*************************************************************************************************
#define DCC_GCTRL_DCCENA_S       0U
#define DCC_GCTRL_DCCENA_M       0xFU      // DCC Enable
#define DCC_GCTRL_ERRENA_S       4U
#define DCC_GCTRL_ERRENA_M       0xF0U     // Error Enable
#define DCC_GCTRL_SINGLESHOT_S   8U
#define DCC_GCTRL_SINGLESHOT_M   0xF00U    // Single-Shot Enable
#define DCC_GCTRL_DONEENA_S      12U
#define DCC_GCTRL_DONEENA_M      0xF000U   // DONE Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the DCCCNTSEED0 register
//
//*************************************************************************************************
#define DCC_CNTSEED0_COUNTSEED0_S   0U
#define DCC_CNTSEED0_COUNTSEED0_M   0xFFFFFU   // Seed Value for Counter 0

//*************************************************************************************************
//
// The following are defines for the bit fields in the DCCVALIDSEED0 register
//
//*************************************************************************************************
#define DCC_VALIDSEED0_VALIDSEED_S   0U
#define DCC_VALIDSEED0_VALIDSEED_M   0xFFFFU   // Seed Value for Valid Duration Counter 0

//*************************************************************************************************
//
// The following are defines for the bit fields in the DCCCNTSEED1 register
//
//*************************************************************************************************
#define DCC_CNTSEED1_COUNTSEED1_S   0U
#define DCC_CNTSEED1_COUNTSEED1_M   0xFFFFFU   // Seed Value for Counter 1

//*************************************************************************************************
//
// The following are defines for the bit fields in the DCCSTATUS register
//
//*************************************************************************************************
#define DCC_STATUS_ERR    0x1U   // Error Flag
#define DCC_STATUS_DONE   0x2U   // Single-Shot Done Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the DCCCNT0 register
//
//*************************************************************************************************
#define DCC_CNT0_COUNT0_S   0U
#define DCC_CNT0_COUNT0_M   0xFFFFFU   // Current Value of Counter 0

//*************************************************************************************************
//
// The following are defines for the bit fields in the DCCVALID0 register
//
//*************************************************************************************************
#define DCC_VALID0_VALID0_S   0U
#define DCC_VALID0_VALID0_M   0xFFFFU   // Current Value of Valid 0

//*************************************************************************************************
//
// The following are defines for the bit fields in the DCCCNT1 register
//
//*************************************************************************************************
#define DCC_CNT1_COUNT1_S   0U
#define DCC_CNT1_COUNT1_M   0xFFFFFU   // Current Value of Counter 1

//*************************************************************************************************
//
// The following are defines for the bit fields in the DCCCLKSRC1 register
//
//*************************************************************************************************
#define DCC_CLKSRC1_CLKSRC1_S   0U
#define DCC_CLKSRC1_CLKSRC1_M   0x1FU     // Clock Source Select for Counter 1
#define DCC_CLKSRC1_KEY_S       12U
#define DCC_CLKSRC1_KEY_M       0xF000U   // Enables or Disables Clock Source Selection for COUNT1

//*************************************************************************************************
//
// The following are defines for the bit fields in the DCCCLKSRC0 register
//
//*************************************************************************************************
#define DCC_CLKSRC0_CLKSRC0_S   0U
#define DCC_CLKSRC0_CLKSRC0_M   0xFU      // Clock Source Select for Counter 0
#define DCC_CLKSRC0_KEY_S       12U
#define DCC_CLKSRC0_KEY_M       0xF000U   // Enables or Disables Clock Source Selection for COUNT0



#endif
