//###########################################################################
//
// FILE:    hw_nmi.h
//
// TITLE:   Definitions for the NMI registers.
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

#ifndef HW_NMI_H
#define HW_NMI_H

//*************************************************************************************************
//
// The following are defines for the NMI register offsets
//
//*************************************************************************************************
#define NMI_O_CFG           0x0U   // NMI Configuration Register
#define NMI_O_FLG           0x1U   // NMI Flag Register (SYSRsn Clear)
#define NMI_O_FLGCLR        0x2U   // NMI Flag Clear Register
#define NMI_O_FLGFRC        0x3U   // NMI Flag Force Register
#define NMI_O_WDCNT         0x4U   // NMI Watchdog Counter Register
#define NMI_O_WDPRD         0x5U   // NMI Watchdog Period Register
#define NMI_O_SHDFLG        0x6U   // NMI Shadow Flag Register
#define NMI_O_ERRORSTS      0x7U   // Error pin status
#define NMI_O_ERRORSTSCLR   0x8U   // ERRORSTS clear register
#define NMI_O_ERRORSTSFRC   0x9U   // ERRORSTS force register
#define NMI_O_ERRORCTL      0xAU   // Error pin control register
#define NMI_O_ERRORLOCK     0xBU   // Lock register to Error pin registers.


//*************************************************************************************************
//
// The following are defines for the bit fields in the NMICFG register
//
//*************************************************************************************************
#define NMI_CFG_NMIE   0x1U   // Global NMI Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the NMIFLG register
//
//*************************************************************************************************
#define NMI_FLG_NMIINT          0x1U      // NMI Interrupt Flag
#define NMI_FLG_CLOCKFAIL       0x2U      // Clock Fail Interrupt Flag
#define NMI_FLG_RAMUNCERR       0x4U      // RAM Uncorrectable Error NMI Flag
#define NMI_FLG_FLUNCERR        0x8U      // Flash Uncorrectable Error NMI Flag
#define NMI_FLG_CPU1HWBISTERR   0x10U     // HW BIST Error NMI Flag
#define NMI_FLG_CPU2HWBISTERR   0x20U     // HW BIST Error NMI Flag
#define NMI_FLG_PIEVECTERR      0x40U     // PIE Vector Fetch Error Flag
#define NMI_FLG_ERADNMI         0x80U     // ERAD Module NMI Flag
#define NMI_FLG_CLBNMI          0x100U    // Configurable Logic Block NMI Flag
#define NMI_FLG_CPU2WDRSN       0x200U    // CPU2 WDRSn Reset Indication Flag
#define NMI_FLG_CPU2NMIWDRSN    0x400U    // CPU2 NMIWDRSn Reset Indication Flag
#define NMI_FLG_CMNMIWDRSN      0x1000U   // CM NMI watch dog has timed out.
#define NMI_FLG_ECATNMIN        0x2000U   // NMI from EtherCAT reset out
#define NMI_FLG_CRC_FAIL        0x4000U   // CRC calculation failed.
#define NMI_FLG_MCAN_ERR        0x8000U   // MCAN module generated an ECC error.

//*************************************************************************************************
//
// The following are defines for the bit fields in the NMIFLGCLR register
//
//*************************************************************************************************
#define NMI_FLGCLR_NMIINT          0x1U      // NMIINT Flag Clear
#define NMI_FLGCLR_CLOCKFAIL       0x2U      // CLOCKFAIL Flag Clear
#define NMI_FLGCLR_RAMUNCERR       0x4U      // RAMUNCERR Flag Clear
#define NMI_FLGCLR_FLUNCERR        0x8U      // FLUNCERR Flag Clear
#define NMI_FLGCLR_CPU1HWBISTERR   0x10U     // CPU1HWBISTERR Flag Clear
#define NMI_FLGCLR_CPU2HWBISTERR   0x20U     // CPU2HWBISTERR Flag Clear
#define NMI_FLGCLR_PIEVECTERR      0x40U     // PIEVECTERR Flag Clear
#define NMI_FLGCLR_ERADNMI         0x80U     // ERADNMI Flag Clear
#define NMI_FLGCLR_CLBNMI          0x100U    // CLBNMI Flag Clear
#define NMI_FLGCLR_CPU2WDRSN       0x200U    // CPU2WDRSn Flag Clear
#define NMI_FLGCLR_CPU2NMIWDRSN    0x400U    // CPU2NMIWDRSn Flag Clear
#define NMI_FLGCLR_CMNMIWDRSN      0x1000U   // DCDCOLF Flag Clear
#define NMI_FLGCLR_ECATNMIN        0x2000U   // ECATNMIn flag clear
#define NMI_FLGCLR_CRC_FAIL        0x4000U   // CRC_FAIL flag clear
#define NMI_FLGCLR_MCAN_ERR        0x8000U   // MCAN_ERR flag clear

//*************************************************************************************************
//
// The following are defines for the bit fields in the NMIFLGFRC register
//
//*************************************************************************************************
#define NMI_FLGFRC_CLOCKFAIL       0x2U      // CLOCKFAIL Flag Force
#define NMI_FLGFRC_RAMUNCERR       0x4U      // RAMUNCERR Flag Force
#define NMI_FLGFRC_FLUNCERR        0x8U      // FLUNCERR Flag Force
#define NMI_FLGFRC_CPU1HWBISTERR   0x10U     // CPU1HWBISTERR Flag Force
#define NMI_FLGFRC_CPU2HWBISTERR   0x20U     // CPU2HWBISTERR Flag Force
#define NMI_FLGFRC_PIEVECTERR      0x40U     // PIEVECTERR Flag Force
#define NMI_FLGFRC_ERADNMI         0x80U     // ERADNMI Flag Force
#define NMI_FLGFRC_CLBNMI          0x100U    // CLBNMI Flag Force
#define NMI_FLGFRC_CPU2WDRSN       0x200U    // CPU2WDRSn Flag Force
#define NMI_FLGFRC_CPU2NMIWDRSN    0x400U    // CPU2NMIWDRSn Flag Force
#define NMI_FLGFRC_CMNMIWDRSN      0x1000U   // DCDCOLF Flag Force
#define NMI_FLGFRC_ECATNMIN        0x2000U   // ECATNMIn flag force
#define NMI_FLGFRC_CRC_FAIL        0x4000U   // CRC_FAIL flag force
#define NMI_FLGFRC_MCAN_ERR        0x8000U   // MCAN_ERR flag force

//*************************************************************************************************
//
// The following are defines for the bit fields in the NMISHDFLG register
//
//*************************************************************************************************
#define NMI_SHDFLG_CLOCKFAIL       0x2U      // Shadow CLOCKFAIL Flag
#define NMI_SHDFLG_RAMUNCERR       0x4U      // Shadow RAMUNCERR Flag
#define NMI_SHDFLG_FLUNCERR        0x8U      // Shadow FLUNCERR Flag
#define NMI_SHDFLG_CPU1HWBISTERR   0x10U     // Shadow CPU1HWBISTERR Flag
#define NMI_SHDFLG_CPU2HWBISTERR   0x20U     // Shadow CPU2HWBISTERR Flag
#define NMI_SHDFLG_PIEVECTERR      0x40U     // Shadow PIEVECTERR Flag
#define NMI_SHDFLG_ERADNMI         0x80U     // Shadow ERADNMI Flag
#define NMI_SHDFLG_CLBNMI          0x100U    // Shadow CLBNMI Flag
#define NMI_SHDFLG_CPU2WDRSN       0x200U    // Shadow CPU2WDRSn Flag
#define NMI_SHDFLG_CPU2NMIWDRSN    0x400U    // Shadow CPU2NMIWDRSn Flag
#define NMI_SHDFLG_CMNMIWDRSN      0x1000U   // Shadow DCDCOLF Flag
#define NMI_SHDFLG_ECATNMIN        0x2000U   // ECATNMIn flag
#define NMI_SHDFLG_CRC_FAIL        0x4000U   // CRC_FAIL flag
#define NMI_SHDFLG_MCAN_ERR        0x8000U   // MCAN_ERR flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the ERRORSTS register
//
//*************************************************************************************************
#define NMI_ERRORSTS_ERROR    0x1U   // Error flag.
#define NMI_ERRORSTS_PINSTS   0x2U   // Error pin status.

//*************************************************************************************************
//
// The following are defines for the bit fields in the ERRORSTSCLR register
//
//*************************************************************************************************
#define NMI_ERRORSTSCLR_ERROR   0x1U   // ERRORFLG.ERROR clear bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the ERRORSTSFRC register
//
//*************************************************************************************************
#define NMI_ERRORSTSFRC_ERROR   0x1U   // ERRORSTS.ERROR pin force.

//*************************************************************************************************
//
// The following are defines for the bit fields in the ERRORCTL register
//
//*************************************************************************************************
#define NMI_ERRORCTL_ERRORPOLSEL   0x1U   // ERROR pin polarity select

//*************************************************************************************************
//
// The following are defines for the bit fields in the ERRORLOCK register
//
//*************************************************************************************************
#define NMI_ERRORLOCK_ERRORCTL   0x1U   // ERRORCTL Lock bit



#endif
