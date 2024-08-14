//###########################################################################
//
// FILE:    hw_emif.h
//
// TITLE:   Definitions for the EMIF registers.
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

#ifndef HW_EMIF_H
#define HW_EMIF_H

//*************************************************************************************************
//
// The following are defines for the EMIF register offsets
//
//*************************************************************************************************
#define EMIF_O_RCSR               0x0U    // Revision Code and Status Register
#define EMIF_O_ASYNC_WCCR         0x2U    // Async Wait Cycle Config Register
#define EMIF_O_SDRAM_CR           0x4U    // SDRAM (EMxCS0n) Config Register
#define EMIF_O_SDRAM_RCR          0x6U    // SDRAM Refresh Control Register
#define EMIF_O_ASYNC_CS2_CR       0x8U    // Async 1 (EMxCS2n) Config Register
#define EMIF_O_ASYNC_CS3_CR       0xAU    // Async 2 (EMxCS3n) Config Register
#define EMIF_O_ASYNC_CS4_CR       0xCU    // Async 3 (EMxCS4n) Config Register
#define EMIF_O_SDRAM_TR           0x10U   // SDRAM Timing Register
#define EMIF_O_TOTAL_SDRAM_AR     0x18U   // Total SDRAM Accesses Register
#define EMIF_O_TOTAL_SDRAM_ACTR   0x1AU   // Total SDRAM Activate Register
#define EMIF_O_SDR_EXT_TMNG       0x1EU   // SDRAM SR/PD Exit Timing Register
#define EMIF_O_INT_RAW            0x20U   // Interrupt Raw Register
#define EMIF_O_INT_MSK            0x22U   // Interrupt Masked Register
#define EMIF_O_INT_MSK_SET        0x24U   // Interrupt Mask Set Register
#define EMIF_O_INT_MSK_CLR        0x26U   // Interrupt Mask Clear Register


//*************************************************************************************************
//
// The following are defines for the bit fields in the RCSR register
//
//*************************************************************************************************
#define EMIF_RCSR_MINOR_REVISION_S   0U
#define EMIF_RCSR_MINOR_REVISION_M   0xFFU         // Minor Revision.
#define EMIF_RCSR_MAJOR_REVISION_S   8U
#define EMIF_RCSR_MAJOR_REVISION_M   0xFF00U       // Major Revision.
#define EMIF_RCSR_MODULE_ID_S        16U
#define EMIF_RCSR_MODULE_ID_M        0x3FFF0000U   // EMIF module ID.
#define EMIF_RCSR_FR                 0x40000000U   // EMIF is running in full rate or half rate.
#define EMIF_RCSR_BE                 0x80000000U   // EMIF endian mode.

//*************************************************************************************************
//
// The following are defines for the bit fields in the ASYNC_WCCR register
//
//*************************************************************************************************
#define EMIF_ASYNC_WCCR_MAX_EXT_WAIT_S   0U
#define EMIF_ASYNC_WCCR_MAX_EXT_WAIT_M   0xFFU         // Maximum Extended Wait cycles.
#define EMIF_ASYNC_WCCR_WP0              0x10000000U   //  Polarity for EMxWAIT.

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDRAM_CR register
//
//*************************************************************************************************
#define EMIF_SDRAM_CR_PAGESIGE_S      0U
#define EMIF_SDRAM_CR_PAGESIGE_M      0x7U          // Page Size.
#define EMIF_SDRAM_CR_IBANK_S         4U
#define EMIF_SDRAM_CR_IBANK_M         0x70U         // Internal Bank setup of SDRAM devices.
#define EMIF_SDRAM_CR_BIT_11_9_LOCK   0x100U        // Bits 11 to 9 are writable only if this bit
                                                    // is set.
#define EMIF_SDRAM_CR_CL_S            9U
#define EMIF_SDRAM_CR_CL_M            0xE00U        // CAS Latency.
#define EMIF_SDRAM_CR_NM              0x4000U       // Narrow Mode.
#define EMIF_SDRAM_CR_PDWR            0x20000000U   // Perform refreshes during Power Down.
#define EMIF_SDRAM_CR_PD              0x40000000U   // Power Down.
#define EMIF_SDRAM_CR_SR              0x80000000U   // Self Refresh.

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDRAM_RCR register
//
//*************************************************************************************************
#define EMIF_SDRAM_RCR_REFRESH_RATE_S   0U
#define EMIF_SDRAM_RCR_REFRESH_RATE_M   0x1FFFU   // Refresh Rate.

//*************************************************************************************************
//
// The following are defines for the bit fields in the ASYNC_CS2_CR register
//
//*************************************************************************************************
#define EMIF_ASYNC_CS2_CR_ASIZE_S      0U
#define EMIF_ASYNC_CS2_CR_ASIZE_M      0x3U          // Asynchronous Memory Size.
#define EMIF_ASYNC_CS2_CR_TA_S         2U
#define EMIF_ASYNC_CS2_CR_TA_M         0xCU          // Turn Around cycles.
#define EMIF_ASYNC_CS2_CR_R_HOLD_S     4U
#define EMIF_ASYNC_CS2_CR_R_HOLD_M     0x70U         // Read Strobe Hold cycles.
#define EMIF_ASYNC_CS2_CR_R_STROBE_S   7U
#define EMIF_ASYNC_CS2_CR_R_STROBE_M   0x1F80U       // Read Strobe Duration cycles.
#define EMIF_ASYNC_CS2_CR_R_SETUP_S    13U
#define EMIF_ASYNC_CS2_CR_R_SETUP_M    0x1E000U      // Read Strobe Setup cycles.
#define EMIF_ASYNC_CS2_CR_W_HOLD_S     17U
#define EMIF_ASYNC_CS2_CR_W_HOLD_M     0xE0000U      // Write Strobe Hold cycles.
#define EMIF_ASYNC_CS2_CR_W_STROBE_S   20U
#define EMIF_ASYNC_CS2_CR_W_STROBE_M   0x3F00000U    // Write Strobe Duration cycles.
#define EMIF_ASYNC_CS2_CR_W_SETUP_S    26U
#define EMIF_ASYNC_CS2_CR_W_SETUP_M    0x3C000000U   // Write Strobe Setup cycles.
#define EMIF_ASYNC_CS2_CR_EW           0x40000000U   // Extend Wait mode.
#define EMIF_ASYNC_CS2_CR_SS           0x80000000U   // Select Strobe mode.

//*************************************************************************************************
//
// The following are defines for the bit fields in the ASYNC_CS3_CR register
//
//*************************************************************************************************
#define EMIF_ASYNC_CS3_CR_ASIZE_S      0U
#define EMIF_ASYNC_CS3_CR_ASIZE_M      0x3U          // Asynchronous Memory Size.
#define EMIF_ASYNC_CS3_CR_TA_S         2U
#define EMIF_ASYNC_CS3_CR_TA_M         0xCU          // Turn Around cycles.
#define EMIF_ASYNC_CS3_CR_R_HOLD_S     4U
#define EMIF_ASYNC_CS3_CR_R_HOLD_M     0x70U         // Read Strobe Hold cycles.
#define EMIF_ASYNC_CS3_CR_R_STROBE_S   7U
#define EMIF_ASYNC_CS3_CR_R_STROBE_M   0x1F80U       // Read Strobe Duration cycles.
#define EMIF_ASYNC_CS3_CR_R_SETUP_S    13U
#define EMIF_ASYNC_CS3_CR_R_SETUP_M    0x1E000U      // Read Strobe Setup cycles.
#define EMIF_ASYNC_CS3_CR_W_HOLD_S     17U
#define EMIF_ASYNC_CS3_CR_W_HOLD_M     0xE0000U      // Write Strobe Hold cycles.
#define EMIF_ASYNC_CS3_CR_W_STROBE_S   20U
#define EMIF_ASYNC_CS3_CR_W_STROBE_M   0x3F00000U    // Write Strobe Duration cycles.
#define EMIF_ASYNC_CS3_CR_W_SETUP_S    26U
#define EMIF_ASYNC_CS3_CR_W_SETUP_M    0x3C000000U   // Write Strobe Setup cycles.
#define EMIF_ASYNC_CS3_CR_EW           0x40000000U   // Extend Wait mode.
#define EMIF_ASYNC_CS3_CR_SS           0x80000000U   // Select Strobe mode.

//*************************************************************************************************
//
// The following are defines for the bit fields in the ASYNC_CS4_CR register
//
//*************************************************************************************************
#define EMIF_ASYNC_CS4_CR_ASIZE_S      0U
#define EMIF_ASYNC_CS4_CR_ASIZE_M      0x3U          // Asynchronous Memory Size.
#define EMIF_ASYNC_CS4_CR_TA_S         2U
#define EMIF_ASYNC_CS4_CR_TA_M         0xCU          // Turn Around cycles.
#define EMIF_ASYNC_CS4_CR_R_HOLD_S     4U
#define EMIF_ASYNC_CS4_CR_R_HOLD_M     0x70U         // Read Strobe Hold cycles.
#define EMIF_ASYNC_CS4_CR_R_STROBE_S   7U
#define EMIF_ASYNC_CS4_CR_R_STROBE_M   0x1F80U       // Read Strobe Duration cycles.
#define EMIF_ASYNC_CS4_CR_R_SETUP_S    13U
#define EMIF_ASYNC_CS4_CR_R_SETUP_M    0x1E000U      // Read Strobe Setup cycles.
#define EMIF_ASYNC_CS4_CR_W_HOLD_S     17U
#define EMIF_ASYNC_CS4_CR_W_HOLD_M     0xE0000U      // Write Strobe Hold cycles.
#define EMIF_ASYNC_CS4_CR_W_STROBE_S   20U
#define EMIF_ASYNC_CS4_CR_W_STROBE_M   0x3F00000U    // Write Strobe Duration cycles.
#define EMIF_ASYNC_CS4_CR_W_SETUP_S    26U
#define EMIF_ASYNC_CS4_CR_W_SETUP_M    0x3C000000U   // Write Strobe Setup cycles.
#define EMIF_ASYNC_CS4_CR_EW           0x40000000U   // Extend Wait mode.
#define EMIF_ASYNC_CS4_CR_SS           0x80000000U   // Select Strobe mode.

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDRAM_TR register
//
//*************************************************************************************************
#define EMIF_SDRAM_TR_T_RRD_S   4U
#define EMIF_SDRAM_TR_T_RRD_M   0x70U         // Activate to Activate timing for different bank.
#define EMIF_SDRAM_TR_T_RC_S    8U
#define EMIF_SDRAM_TR_T_RC_M    0xF00U        // Activate to Activate timing .
#define EMIF_SDRAM_TR_T_RAS_S   12U
#define EMIF_SDRAM_TR_T_RAS_M   0xF000U       // Activate to Precharge timing.
#define EMIF_SDRAM_TR_T_WR_S    16U
#define EMIF_SDRAM_TR_T_WR_M    0x70000U      // Last Write to Precharge timing.
#define EMIF_SDRAM_TR_T_RCD_S   20U
#define EMIF_SDRAM_TR_T_RCD_M   0x700000U     // Activate to Read/Write timing.
#define EMIF_SDRAM_TR_T_RP_S    24U
#define EMIF_SDRAM_TR_T_RP_M    0x7000000U    // Precharge to Activate/Refresh timing.
#define EMIF_SDRAM_TR_T_RFC_S   27U
#define EMIF_SDRAM_TR_T_RFC_M   0xF8000000U   // Refresh/Load Mode to Refresh/Activate timing

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDR_EXT_TMNG register
//
//*************************************************************************************************
#define EMIF_SDR_EXT_TMNG_T_XS_S   0U
#define EMIF_SDR_EXT_TMNG_T_XS_M   0x1FU   // Self Refresh exit to new command timing.

//*************************************************************************************************
//
// The following are defines for the bit fields in the INT_RAW register
//
//*************************************************************************************************
#define EMIF_INT_RAW_AT     0x1U    // Asynchronous Timeout.
#define EMIF_INT_RAW_LT     0x2U    // Line Trap.
#define EMIF_INT_RAW_WR_S   2U
#define EMIF_INT_RAW_WR_M   0x3CU   // Wait Rise.

//*************************************************************************************************
//
// The following are defines for the bit fields in the INT_MSK register
//
//*************************************************************************************************
#define EMIF_INT_MSK_AT_MASKED     0x1U    // Asynchronous Timeout.
#define EMIF_INT_MSK_LT_MASKED     0x2U    // Line Trap.
#define EMIF_INT_MSK_WR_MASKED_S   2U
#define EMIF_INT_MSK_WR_MASKED_M   0x3CU   // Wait Rise.

//*************************************************************************************************
//
// The following are defines for the bit fields in the INT_MSK_SET register
//
//*************************************************************************************************
#define EMIF_INT_MSK_SET_AT_MASK_SET     0x1U    // Asynchronous Timeout.
#define EMIF_INT_MSK_SET_LT_MASK_SET     0x2U    // Line Trap.
#define EMIF_INT_MSK_SET_WR_MASK_SET_S   2U
#define EMIF_INT_MSK_SET_WR_MASK_SET_M   0x3CU   // Wait Rise.

//*************************************************************************************************
//
// The following are defines for the bit fields in the INT_MSK_CLR register
//
//*************************************************************************************************
#define EMIF_INT_MSK_CLR_AT_MASK_CLR     0x1U    // Asynchronous Timeout.
#define EMIF_INT_MSK_CLR_LT_MASK_CLR     0x2U    // Line Trap.
#define EMIF_INT_MSK_CLR_WR_MASK_CLR_S   2U
#define EMIF_INT_MSK_CLR_WR_MASK_CLR_M   0x3CU   // Wait Rise.



#endif
