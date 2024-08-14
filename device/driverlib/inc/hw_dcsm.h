//###########################################################################
//
// FILE:    hw_dcsm.h
//
// TITLE:   Definitions for the DCSM registers.
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

#ifndef HW_DCSM_H
#define HW_DCSM_H

//*************************************************************************************************
//
// The following are defines for the DCSM register offsets
//
//*************************************************************************************************
#define DCSM_O_Z1OTP_LINKPOINTER1   0x0U    // Zone 1 Link Pointer1
#define DCSM_O_Z1OTP_LINKPOINTER2   0x2U    // Zone 1 Link Pointer2
#define DCSM_O_Z1OTP_LINKPOINTER3   0x4U    // Zone 1 Link Pointer3
#define DCSM_O_Z1OTP_JLM_ENABLE     0x6U    // Zone 1 JTAGLOCK Enable Register
#define DCSM_O_Z1OTP_GPREG1         0x8U    // Zone 1 General Purpose Register 1
#define DCSM_O_Z1OTP_GPREG2         0xAU    // Zone 1 General Purpose Register 2
#define DCSM_O_Z1OTP_GPREG3         0xCU    // Zone 1 General Purpose Register 3
#define DCSM_O_Z1OTP_GPREG4         0xEU    // Zone 1 General Purpose Register 4
#define DCSM_O_Z1OTP_PSWDLOCK       0x10U   // Secure Password Lock
#define DCSM_O_Z1OTP_CRCLOCK        0x12U   // Secure CRC Lock
#define DCSM_O_Z1OTP_JTAGPSWDH0     0x14U   // JTAG Lock Permanent Password 0
#define DCSM_O_Z1OTP_JTAGPSWDH1     0x16U   // JTAG Lock Permanent Password 1
#define DCSM_O_Z1OTP_CMACKEY0       0x18U   // Secure Boot CMAC Key 0
#define DCSM_O_Z1OTP_CMACKEY1       0x1AU   // Secure Boot CMAC Key 1
#define DCSM_O_Z1OTP_CMACKEY2       0x1CU   // Secure Boot CMAC Key 2
#define DCSM_O_Z1OTP_CMACKEY3       0x1EU   // Secure Boot CMAC Key 3

#define DCSM_O_Z2OTP_LINKPOINTER1   0x0U    // Zone 2 Link Pointer1
#define DCSM_O_Z2OTP_LINKPOINTER2   0x2U    // Zone 2 Link Pointer2
#define DCSM_O_Z2OTP_LINKPOINTER3   0x4U    // Zone 2 Link Pointer3
#define DCSM_O_Z2OTP_GPREG1         0x8U    // Zone 2 General Purpose Register 1
#define DCSM_O_Z2OTP_GPREG2         0xAU    // Zone 2 General Purpose Register 2
#define DCSM_O_Z2OTP_GPREG3         0xCU    // Zone 2 General Purpose Register 3
#define DCSM_O_Z2OTP_GPREG4         0xEU    // Zone 2 General Purpose Register 4
#define DCSM_O_Z2OTP_PSWDLOCK       0x10U   // Secure Password Lock
#define DCSM_O_Z2OTP_CRCLOCK        0x12U   // Secure CRC Lock

#define DCSM_O_Z1_LINKPOINTER      0x0U    // Zone 1 Link Pointer
#define DCSM_O_Z1_OTPSECLOCK       0x2U    // Zone 1 OTP Secure Lock
#define DCSM_O_Z1_JLM_ENABLE       0x4U    // Zone 1 JTAGLOCK Enable Register
#define DCSM_O_Z1_LINKPOINTERERR   0x6U    // Link Pointer Error
#define DCSM_O_Z1_GPREG1           0x8U    // Zone 1 General Purpose Register-1
#define DCSM_O_Z1_GPREG2           0xAU    // Zone 1 General Purpose Register-2
#define DCSM_O_Z1_GPREG3           0xCU    // Zone 1 General Purpose Register-3
#define DCSM_O_Z1_GPREG4           0xEU    // Zone 1 General Purpose Register-4
#define DCSM_O_Z1_CSMKEY0          0x10U   // Zone 1 CSM Key 0
#define DCSM_O_Z1_CSMKEY1          0x12U   // Zone 1 CSM Key 1
#define DCSM_O_Z1_CSMKEY2          0x14U   // Zone 1 CSM Key 2
#define DCSM_O_Z1_CSMKEY3          0x16U   // Zone 1 CSM Key 3
#define DCSM_O_Z1_CR               0x18U   // Zone 1 CSM Control Register
#define DCSM_O_Z1_GRABSECT1R       0x1AU   // Zone 1 Grab Flash Status Register 1
#define DCSM_O_Z1_GRABSECT2R       0x1CU   // Zone 1 Grab Flash Status Register 2
#define DCSM_O_Z1_GRABSECT3R       0x1EU   // Zone 1 Grab Flash Status Register 3
#define DCSM_O_Z1_GRABRAM1R        0x20U   // Zone 1 Grab RAM Status Register 1
#define DCSM_O_Z1_GRABRAM2R        0x22U   // Zone 1 Grab RAM Status Register 2
#define DCSM_O_Z1_GRABRAM3R        0x24U   // Zone 1 Grab RAM Status Register 3
#define DCSM_O_Z1_EXEONLYSECT1R    0x26U   // Zone 1 Execute Only Flash Status Register 1
#define DCSM_O_Z1_EXEONLYSECT2R    0x28U   // Zone 1 Execute Only Flash Status Register 2
#define DCSM_O_Z1_EXEONLYRAM1R     0x2AU   // Zone 1 Execute Only RAM Status Register 1
#define DCSM_O_Z1_JTAGKEY0         0x2EU   // JTAG Unlock Key Register 0
#define DCSM_O_Z1_JTAGKEY1         0x30U   // JTAG Unlock Key Register 1
#define DCSM_O_Z1_JTAGKEY2         0x32U   // JTAG Unlock Key Register 2
#define DCSM_O_Z1_JTAGKEY3         0x34U   // JTAG Unlock Key Register 3
#define DCSM_O_Z1_CMACKEY0         0x36U   // Secure Boot CMAC Key Status Register 0
#define DCSM_O_Z1_CMACKEY1         0x38U   // Secure Boot CMAC Key Status Register 1
#define DCSM_O_Z1_CMACKEY2         0x3AU   // Secure Boot CMAC Key Status Register 2
#define DCSM_O_Z1_CMACKEY3         0x3CU   // Secure Boot CMAC Key Status Register 3

#define DCSM_O_Z2_LINKPOINTER      0x0U    // Zone 2 Link Pointer
#define DCSM_O_Z2_OTPSECLOCK       0x2U    // Zone 2 OTP Secure Lock
#define DCSM_O_Z2_LINKPOINTERERR   0x6U    // Link Pointer Error
#define DCSM_O_Z2_GPREG1           0x8U    // Zone 2 General Purpose Register-1
#define DCSM_O_Z2_GPREG2           0xAU    // Zone 2 General Purpose Register-2
#define DCSM_O_Z2_GPREG3           0xCU    // Zone 2 General Purpose Register-3
#define DCSM_O_Z2_GPREG4           0xEU    // Zone 2 General Purpose Register-4
#define DCSM_O_Z2_CSMKEY0          0x10U   // Zone 2 CSM Key 0
#define DCSM_O_Z2_CSMKEY1          0x12U   // Zone 2 CSM Key 1
#define DCSM_O_Z2_CSMKEY2          0x14U   // Zone 2 CSM Key 2
#define DCSM_O_Z2_CSMKEY3          0x16U   // Zone 2 CSM Key 3
#define DCSM_O_Z2_CR               0x18U   // Zone 2 CSM Control Register
#define DCSM_O_Z2_GRABSECT1R       0x1AU   // Zone 2 Grab Flash Status Register 1
#define DCSM_O_Z2_GRABSECT2R       0x1CU   // Zone 2 Grab Flash Status Register 2
#define DCSM_O_Z2_GRABSECT3R       0x1EU   // Zone 2 Grab Flash Status Register 3
#define DCSM_O_Z2_GRABRAM1R        0x20U   // Zone 2 Grab RAM Status Register 1
#define DCSM_O_Z2_GRABRAM2R        0x22U   // Zone 2 Grab RAM Status Register 2
#define DCSM_O_Z2_GRABRAM3R        0x24U   // Zone 2 Grab RAM Status Register 3
#define DCSM_O_Z2_EXEONLYSECT1R    0x26U   // Zone 2 Execute Only Flash Status Register 1
#define DCSM_O_Z2_EXEONLYSECT2R    0x28U   // Zone 2 Execute Only Flash Status Register 2
#define DCSM_O_Z2_EXEONLYRAM1R     0x2AU   // Zone 2 Execute Only RAM Status Register 1

#define DCSM_O_FLSEM        0x0U    // Flash Wrapper Semaphore Register
#define DCSM_O_SECTSTAT1    0x8U    // Flash Sectors Status Register 1
#define DCSM_O_SECTSTAT2    0xAU    // Flash Sectors Status Register 2
#define DCSM_O_SECTSTAT3    0xCU    // Flash Sectors Status Register 3
#define DCSM_O_RAMSTAT1     0x10U   // RAM Status Register 1
#define DCSM_O_RAMSTAT2     0x12U   // RAM Status Register 2
#define DCSM_O_RAMSTAT3     0x14U   // RAM Status Register 3
#define DCSM_O_SECERRSTAT   0x18U   // Security Error Status Register
#define DCSM_O_SECERRCLR    0x1AU   // Security Error Clear Register
#define DCSM_O_SECERRFRC    0x1CU   // Security Error Force Register




//*************************************************************************************************
//
// The following are defines for the bit fields in the Z1_LINKPOINTER register
//
//*************************************************************************************************
#define DCSM_Z1_LINKPOINTER_LINKPOINTER_S   0U
#define DCSM_Z1_LINKPOINTER_LINKPOINTER_M   0x3FFFU   // Zone1 LINK Pointer

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z1_OTPSECLOCK register
//
//*************************************************************************************************
#define DCSM_Z1_OTPSECLOCK_JTAGLOCK     0x1U     // JTAG Lock Status
#define DCSM_Z1_OTPSECLOCK_PSWDLOCK_S   4U
#define DCSM_Z1_OTPSECLOCK_PSWDLOCK_M   0xF0U    // Zone1 Password Lock.
#define DCSM_Z1_OTPSECLOCK_CRCLOCK_S    8U
#define DCSM_Z1_OTPSECLOCK_CRCLOCK_M    0xF00U   // Zone1 CRC Lock.

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z1_JLM_ENABLE register
//
//*************************************************************************************************
#define DCSM_Z1_JLM_ENABLE_Z1_JLM_ENABLE_S   0U
#define DCSM_Z1_JLM_ENABLE_Z1_JLM_ENABLE_M   0xFU   // Zone1 JLM_ENABLE register.

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z1_LINKPOINTERERR register
//
//*************************************************************************************************
#define DCSM_Z1_LINKPOINTERERR_Z1_LINKPOINTERERR_S   0U
#define DCSM_Z1_LINKPOINTERERR_Z1_LINKPOINTERERR_M   0x3FFFU   // Error to Resolve Z1 Link pointer
                                                               // from OTP loaded values

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z1_CR register
//
//*************************************************************************************************
#define DCSM_Z1_CR_ALLZERO    0x80000U      // CSMPSWD All Zeros
#define DCSM_Z1_CR_ALLONE     0x100000U     // CSMPSWD All Ones
#define DCSM_Z1_CR_UNSECURE   0x200000U     // CSMPSWD Match CSMKEY
#define DCSM_Z1_CR_ARMED      0x400000U     // CSM Passwords Read Status
#define DCSM_Z1_CR_FORCESEC   0x80000000U   // Force Secure

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z1_GRABSECT1R register
//
//*************************************************************************************************
#define DCSM_Z1_GRABSECT1R_GRAB_SECT0_S    0U
#define DCSM_Z1_GRABSECT1R_GRAB_SECT0_M    0x3U         // Grab Flash Sector 0 in CPU1 BANK
#define DCSM_Z1_GRABSECT1R_GRAB_SECT1_S    2U
#define DCSM_Z1_GRABSECT1R_GRAB_SECT1_M    0xCU         // Grab Flash Sector 1 in CPU1 BANK
#define DCSM_Z1_GRABSECT1R_GRAB_SECT2_S    4U
#define DCSM_Z1_GRABSECT1R_GRAB_SECT2_M    0x30U        // Grab Flash Sector 2 in CPU1 BANK
#define DCSM_Z1_GRABSECT1R_GRAB_SECT3_S    6U
#define DCSM_Z1_GRABSECT1R_GRAB_SECT3_M    0xC0U        // Grab Flash Sector 3 in CPU1 BANK
#define DCSM_Z1_GRABSECT1R_GRAB_SECT4_S    8U
#define DCSM_Z1_GRABSECT1R_GRAB_SECT4_M    0x300U       // Grab Flash Sector 4 in CPU1 BANK
#define DCSM_Z1_GRABSECT1R_GRAB_SECT5_S    10U
#define DCSM_Z1_GRABSECT1R_GRAB_SECT5_M    0xC00U       // Grab Flash Sector 5 in CPU1 BANK
#define DCSM_Z1_GRABSECT1R_GRAB_SECT6_S    12U
#define DCSM_Z1_GRABSECT1R_GRAB_SECT6_M    0x3000U      // Grab Flash Sector 6 in CPU1 BANK
#define DCSM_Z1_GRABSECT1R_GRAB_SECT7_S    14U
#define DCSM_Z1_GRABSECT1R_GRAB_SECT7_M    0xC000U      // Grab Flash Sector 7 in CPU1 BANK
#define DCSM_Z1_GRABSECT1R_GRAB_SECT8_S    16U
#define DCSM_Z1_GRABSECT1R_GRAB_SECT8_M    0x30000U     // Grab Flash Sector 8 in CPU1 BANK
#define DCSM_Z1_GRABSECT1R_GRAB_SECT9_S    18U
#define DCSM_Z1_GRABSECT1R_GRAB_SECT9_M    0xC0000U     // Grab Flash Sector 9 in CPU1 BANK
#define DCSM_Z1_GRABSECT1R_GRAB_SECT10_S   20U
#define DCSM_Z1_GRABSECT1R_GRAB_SECT10_M   0x300000U    // Grab Flash Sector 10 in CPU1 BANK
#define DCSM_Z1_GRABSECT1R_GRAB_SECT11_S   22U
#define DCSM_Z1_GRABSECT1R_GRAB_SECT11_M   0xC00000U    // Grab Flash Sector 11 in CPU1 BANK
#define DCSM_Z1_GRABSECT1R_GRAB_SECT12_S   24U
#define DCSM_Z1_GRABSECT1R_GRAB_SECT12_M   0x3000000U   // Grab Flash Sector 12 in CPU1 BANK
#define DCSM_Z1_GRABSECT1R_GRAB_SECT13_S   26U
#define DCSM_Z1_GRABSECT1R_GRAB_SECT13_M   0xC000000U   // Grab Flash Sector 13 in CPU1 BANK

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z1_GRABSECT2R register
//
//*************************************************************************************************
#define DCSM_Z1_GRABSECT2R_GRAB_SECT0_S    0U
#define DCSM_Z1_GRABSECT2R_GRAB_SECT0_M    0x3U         // Grab Flash Sector 0 in CM BANK
#define DCSM_Z1_GRABSECT2R_GRAB_SECT1_S    2U
#define DCSM_Z1_GRABSECT2R_GRAB_SECT1_M    0xCU         // Grab Flash Sector 1 in CM BANK
#define DCSM_Z1_GRABSECT2R_GRAB_SECT2_S    4U
#define DCSM_Z1_GRABSECT2R_GRAB_SECT2_M    0x30U        // Grab Flash Sector 2 in CM BANK
#define DCSM_Z1_GRABSECT2R_GRAB_SECT3_S    6U
#define DCSM_Z1_GRABSECT2R_GRAB_SECT3_M    0xC0U        // Grab Flash Sector 3 in CM BANK
#define DCSM_Z1_GRABSECT2R_GRAB_SECT4_S    8U
#define DCSM_Z1_GRABSECT2R_GRAB_SECT4_M    0x300U       // Grab Flash Sector 4 in CM BANK
#define DCSM_Z1_GRABSECT2R_GRAB_SECT5_S    10U
#define DCSM_Z1_GRABSECT2R_GRAB_SECT5_M    0xC00U       // Grab Flash Sector 5 in CM BANK
#define DCSM_Z1_GRABSECT2R_GRAB_SECT6_S    12U
#define DCSM_Z1_GRABSECT2R_GRAB_SECT6_M    0x3000U      // Grab Flash Sector 6 in CM BANK
#define DCSM_Z1_GRABSECT2R_GRAB_SECT7_S    14U
#define DCSM_Z1_GRABSECT2R_GRAB_SECT7_M    0xC000U      // Grab Flash Sector 7 in CM BANK
#define DCSM_Z1_GRABSECT2R_GRAB_SECT8_S    16U
#define DCSM_Z1_GRABSECT2R_GRAB_SECT8_M    0x30000U     // Grab Flash Sector 8 in CM BANK
#define DCSM_Z1_GRABSECT2R_GRAB_SECT9_S    18U
#define DCSM_Z1_GRABSECT2R_GRAB_SECT9_M    0xC0000U     // Grab Flash Sector 9 in CM BANK
#define DCSM_Z1_GRABSECT2R_GRAB_SECT10_S   20U
#define DCSM_Z1_GRABSECT2R_GRAB_SECT10_M   0x300000U    // Grab Flash Sector 10 in CM BANK
#define DCSM_Z1_GRABSECT2R_GRAB_SECT11_S   22U
#define DCSM_Z1_GRABSECT2R_GRAB_SECT11_M   0xC00000U    // Grab Flash Sector 11 in CM BANK
#define DCSM_Z1_GRABSECT2R_GRAB_SECT12_S   24U
#define DCSM_Z1_GRABSECT2R_GRAB_SECT12_M   0x3000000U   // Grab Flash Sector 12 in CM BANK
#define DCSM_Z1_GRABSECT2R_GRAB_SECT13_S   26U
#define DCSM_Z1_GRABSECT2R_GRAB_SECT13_M   0xC000000U   // Grab Flash Sector 13 in CM BANK

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z1_GRABSECT3R register
//
//*************************************************************************************************
#define DCSM_Z1_GRABSECT3R_GRAB_SECT0_S    0U
#define DCSM_Z1_GRABSECT3R_GRAB_SECT0_M    0x3U         // Grab Flash Sector 0 in CPU2 BANK
#define DCSM_Z1_GRABSECT3R_GRAB_SECT1_S    2U
#define DCSM_Z1_GRABSECT3R_GRAB_SECT1_M    0xCU         // Grab Flash Sector 1 in CPU2 BANK
#define DCSM_Z1_GRABSECT3R_GRAB_SECT2_S    4U
#define DCSM_Z1_GRABSECT3R_GRAB_SECT2_M    0x30U        // Grab Flash Sector 2 in CPU2 BANK
#define DCSM_Z1_GRABSECT3R_GRAB_SECT3_S    6U
#define DCSM_Z1_GRABSECT3R_GRAB_SECT3_M    0xC0U        // Grab Flash Sector 3 in CPU2 BANK
#define DCSM_Z1_GRABSECT3R_GRAB_SECT4_S    8U
#define DCSM_Z1_GRABSECT3R_GRAB_SECT4_M    0x300U       // Grab Flash Sector 4 in CPU2 BANK
#define DCSM_Z1_GRABSECT3R_GRAB_SECT5_S    10U
#define DCSM_Z1_GRABSECT3R_GRAB_SECT5_M    0xC00U       // Grab Flash Sector 5 in CPU2 BANK
#define DCSM_Z1_GRABSECT3R_GRAB_SECT6_S    12U
#define DCSM_Z1_GRABSECT3R_GRAB_SECT6_M    0x3000U      // Grab Flash Sector 6 in CPU2 BANK
#define DCSM_Z1_GRABSECT3R_GRAB_SECT7_S    14U
#define DCSM_Z1_GRABSECT3R_GRAB_SECT7_M    0xC000U      // Grab Flash Sector 7 in CPU2 BANK
#define DCSM_Z1_GRABSECT3R_GRAB_SECT8_S    16U
#define DCSM_Z1_GRABSECT3R_GRAB_SECT8_M    0x30000U     // Grab Flash Sector 8 in CPU2 BANK
#define DCSM_Z1_GRABSECT3R_GRAB_SECT9_S    18U
#define DCSM_Z1_GRABSECT3R_GRAB_SECT9_M    0xC0000U     // Grab Flash Sector 9 in CPU2 BANK
#define DCSM_Z1_GRABSECT3R_GRAB_SECT10_S   20U
#define DCSM_Z1_GRABSECT3R_GRAB_SECT10_M   0x300000U    // Grab Flash Sector 10 in CPU2 BANK
#define DCSM_Z1_GRABSECT3R_GRAB_SECT11_S   22U
#define DCSM_Z1_GRABSECT3R_GRAB_SECT11_M   0xC00000U    // Grab Flash Sector 11 in CPU2 BANK
#define DCSM_Z1_GRABSECT3R_GRAB_SECT12_S   24U
#define DCSM_Z1_GRABSECT3R_GRAB_SECT12_M   0x3000000U   // Grab Flash Sector 12 in CPU2 BANK
#define DCSM_Z1_GRABSECT3R_GRAB_SECT13_S   26U
#define DCSM_Z1_GRABSECT3R_GRAB_SECT13_M   0xC000000U   // Grab Flash Sector 13 in CPU2 BANK

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z1_GRABRAM1R register
//
//*************************************************************************************************
#define DCSM_Z1_GRABRAM1R_GRAB_RAM0_S   0U
#define DCSM_Z1_GRABRAM1R_GRAB_RAM0_M   0x3U       // Grab RAM CPU1.LS0
#define DCSM_Z1_GRABRAM1R_GRAB_RAM1_S   2U
#define DCSM_Z1_GRABRAM1R_GRAB_RAM1_M   0xCU       // Grab RAM CPU1.LS1
#define DCSM_Z1_GRABRAM1R_GRAB_RAM2_S   4U
#define DCSM_Z1_GRABRAM1R_GRAB_RAM2_M   0x30U      // Grab RAM CPU1.LS2
#define DCSM_Z1_GRABRAM1R_GRAB_RAM3_S   6U
#define DCSM_Z1_GRABRAM1R_GRAB_RAM3_M   0xC0U      // Grab RAM CPU1.LS3
#define DCSM_Z1_GRABRAM1R_GRAB_RAM4_S   8U
#define DCSM_Z1_GRABRAM1R_GRAB_RAM4_M   0x300U     // Grab RAM CPU1.LS4
#define DCSM_Z1_GRABRAM1R_GRAB_RAM5_S   10U
#define DCSM_Z1_GRABRAM1R_GRAB_RAM5_M   0xC00U     // Grab RAM CPU1.LS5
#define DCSM_Z1_GRABRAM1R_GRAB_RAM6_S   12U
#define DCSM_Z1_GRABRAM1R_GRAB_RAM6_M   0x3000U    // Grab RAM CPU1.LS6
#define DCSM_Z1_GRABRAM1R_GRAB_RAM7_S   14U
#define DCSM_Z1_GRABRAM1R_GRAB_RAM7_M   0xC000U    // Grab RAM CPU1.LS7
#define DCSM_Z1_GRABRAM1R_GRAB_RAM8_S   16U
#define DCSM_Z1_GRABRAM1R_GRAB_RAM8_M   0x30000U   // Grab RAM CPU1.D0
#define DCSM_Z1_GRABRAM1R_GRAB_RAM9_S   18U
#define DCSM_Z1_GRABRAM1R_GRAB_RAM9_M   0xC0000U   // Grab RAM CPU1.D1

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z1_GRABRAM2R register
//
//*************************************************************************************************
#define DCSM_Z1_GRABRAM2R_GRAB_RAM0_S    0U
#define DCSM_Z1_GRABRAM2R_GRAB_RAM0_M    0x3U          // Grab RAM CM.C0
#define DCSM_Z1_GRABRAM2R_GRAB_RAM1_S    2U
#define DCSM_Z1_GRABRAM2R_GRAB_RAM1_M    0xCU          // Grab RAM CM.C1
#define DCSM_Z1_GRABRAM2R_GRAB_RAM4_S    8U
#define DCSM_Z1_GRABRAM2R_GRAB_RAM4_M    0x300U        // Grab RAM CPU1TOCM MSGRAM0_L
#define DCSM_Z1_GRABRAM2R_GRAB_RAM5_S    10U
#define DCSM_Z1_GRABRAM2R_GRAB_RAM5_M    0xC00U        // Grab RAM CPU1TOCM MSGRAM0_H
#define DCSM_Z1_GRABRAM2R_GRAB_RAM6_S    12U
#define DCSM_Z1_GRABRAM2R_GRAB_RAM6_M    0x3000U       // Grab RAM CMTOCPU1 MSGRAM0_L
#define DCSM_Z1_GRABRAM2R_GRAB_RAM7_S    14U
#define DCSM_Z1_GRABRAM2R_GRAB_RAM7_M    0xC000U       // Grab RAM CMTOCPU1 MSGRAM0_H
#define DCSM_Z1_GRABRAM2R_GRAB_RAM8_S    16U
#define DCSM_Z1_GRABRAM2R_GRAB_RAM8_M    0x30000U      // Grab RAM CPU2TOCM MSGRAM0_L
#define DCSM_Z1_GRABRAM2R_GRAB_RAM9_S    18U
#define DCSM_Z1_GRABRAM2R_GRAB_RAM9_M    0xC0000U      // Grab RAM CPU2TOCM MSGRAM0_H
#define DCSM_Z1_GRABRAM2R_GRAB_RAM10_S   20U
#define DCSM_Z1_GRABRAM2R_GRAB_RAM10_M   0x300000U     // Grab RAM CMTOCPU2 MSGRAM0_L
#define DCSM_Z1_GRABRAM2R_GRAB_RAM11_S   22U
#define DCSM_Z1_GRABRAM2R_GRAB_RAM11_M   0xC00000U     // Grab RAM CMTOCPU2 MSGRAM0_H
#define DCSM_Z1_GRABRAM2R_GRAB_RAM12_S   24U
#define DCSM_Z1_GRABRAM2R_GRAB_RAM12_M   0x3000000U    // Grab RAM CPU1TOCPU2 MSGRAM0_L
#define DCSM_Z1_GRABRAM2R_GRAB_RAM13_S   26U
#define DCSM_Z1_GRABRAM2R_GRAB_RAM13_M   0xC000000U    // Grab RAM CPU1TOCPU2 MSGRAM0_H
#define DCSM_Z1_GRABRAM2R_GRAB_RAM14_S   28U
#define DCSM_Z1_GRABRAM2R_GRAB_RAM14_M   0x30000000U   // Grab RAM CPU2TOCPU1 MSGRAM0_L
#define DCSM_Z1_GRABRAM2R_GRAB_RAM15_S   30U
#define DCSM_Z1_GRABRAM2R_GRAB_RAM15_M   0xC0000000U   // Grab RAM CPU2TOCPU1 MSGRAM0_H

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z1_GRABRAM3R register
//
//*************************************************************************************************
#define DCSM_Z1_GRABRAM3R_GRAB_RAM0_S   0U
#define DCSM_Z1_GRABRAM3R_GRAB_RAM0_M   0x3U       // Grab RAM CPU2.LS0
#define DCSM_Z1_GRABRAM3R_GRAB_RAM1_S   2U
#define DCSM_Z1_GRABRAM3R_GRAB_RAM1_M   0xCU       // Grab RAM CPU2.LS1
#define DCSM_Z1_GRABRAM3R_GRAB_RAM2_S   4U
#define DCSM_Z1_GRABRAM3R_GRAB_RAM2_M   0x30U      // Grab RAM CPU2.LS2
#define DCSM_Z1_GRABRAM3R_GRAB_RAM3_S   6U
#define DCSM_Z1_GRABRAM3R_GRAB_RAM3_M   0xC0U      // Grab RAM CPU2.LS3
#define DCSM_Z1_GRABRAM3R_GRAB_RAM4_S   8U
#define DCSM_Z1_GRABRAM3R_GRAB_RAM4_M   0x300U     // Grab RAM CPU2.LS4
#define DCSM_Z1_GRABRAM3R_GRAB_RAM5_S   10U
#define DCSM_Z1_GRABRAM3R_GRAB_RAM5_M   0xC00U     // Grab RAM CPU2.LS5
#define DCSM_Z1_GRABRAM3R_GRAB_RAM6_S   12U
#define DCSM_Z1_GRABRAM3R_GRAB_RAM6_M   0x3000U    // Grab RAM CPU2.LS6
#define DCSM_Z1_GRABRAM3R_GRAB_RAM7_S   14U
#define DCSM_Z1_GRABRAM3R_GRAB_RAM7_M   0xC000U    // Grab RAM CPU2.LS7
#define DCSM_Z1_GRABRAM3R_GRAB_RAM8_S   16U
#define DCSM_Z1_GRABRAM3R_GRAB_RAM8_M   0x30000U   // Grab RAM CPU2.D0
#define DCSM_Z1_GRABRAM3R_GRAB_RAM9_S   18U
#define DCSM_Z1_GRABRAM3R_GRAB_RAM9_M   0xC0000U   // Grab RAM CPU2.D1

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z1_EXEONLYSECT1R register
//
//*************************************************************************************************
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CPU1_SECT0    0x1U          // Execute-Only Flash Sector 0 in
                                                                  // flash CPU1 BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CPU1_SECT1    0x2U          // Execute-Only Flash Sector 1 in
                                                                  // flash CPU1 BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CPU1_SECT2    0x4U          // Execute-Only Flash Sector 2 in
                                                                  // flash CPU1 BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CPU1_SECT3    0x8U          // Execute-Only Flash Sector 3 in
                                                                  // flash CPU1 BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CPU1_SECT4    0x10U         // Execute-Only Flash Sector 4 in
                                                                  // flash CPU1 BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CPU1_SECT5    0x20U         // Execute-Only Flash Sector 5 in
                                                                  // flash CPU1 BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CPU1_SECT6    0x40U         // Execute-Only Flash Sector 6 in
                                                                  // flash CPU1 BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CPU1_SECT7    0x80U         // Execute-Only Flash Sector 7 in
                                                                  // flash CPU1 BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CPU1_SECT8    0x100U        // Execute-Only Flash Sector 8 in
                                                                  // flash CPU1 BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CPU1_SECT9    0x200U        // Execute-Only Flash Sector 9 in
                                                                  // flash CPU1 BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CPU1_SECT10   0x400U        // Execute-Only Flash Sector 10
                                                                  // in flash CPU1 BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CPU1_SECT11   0x800U        // Execute-Only Flash Sector 11
                                                                  // in flash CPU1 BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CPU1_SECT12   0x1000U       // Execute-Only Flash Sector 12
                                                                  // in flash CPU1 BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CPU1_SECT13   0x2000U       // Execute-Only Flash Sector 13
                                                                  // in flash CPU1 BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CM_SECT0      0x10000U      // Execute-Only Flash Sector 0 in
                                                                  // flash CM BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CM_SECT1      0x20000U      // Execute-Only Flash Sector 1 in
                                                                  // flash CM BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CM_SECT2      0x40000U      // Execute-Only Flash Sector 2 in
                                                                  // flash CM BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CM_SECT3      0x80000U      // Execute-Only Flash Sector 3 in
                                                                  // flash CM BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CM_SECT4      0x100000U     // Execute-Only Flash Sector 4 in
                                                                  // flash CM BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CM_SECT5      0x200000U     // Execute-Only Flash Sector 5 in
                                                                  // flash CM BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CM_SECT6      0x400000U     // Execute-Only Flash Sector 6 in
                                                                  // flash CM BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CM_SECT7      0x800000U     // Execute-Only Flash Sector 7 in
                                                                  // flash CM BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CM_SECT8      0x1000000U    // Execute-Only Flash Sector 8 in
                                                                  // flash CM BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CM_SECT9      0x2000000U    // Execute-Only Flash Sector 9 in
                                                                  // flash CM BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CM_SECT10     0x4000000U    // Execute-Only Flash Sector 10
                                                                  // in flash CM BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CM_SECT11     0x8000000U    // Execute-Only Flash Sector 11
                                                                  // in flash CM BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CM_SECT12     0x10000000U   // Execute-Only Flash Sector 12
                                                                  // in flash CM BANK
#define DCSM_Z1_EXEONLYSECT1R_EXEONLY_CM_SECT13     0x20000000U   // Execute-Only Flash Sector 13
                                                                  // in flash CM BANK

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z1_EXEONLYSECT2R register
//
//*************************************************************************************************
#define DCSM_Z1_EXEONLYSECT2R_EXEONLY_CPU2_SECT0    0x1U      // Execute-Only Flash Sector 0 in
                                                              // flash CPU2 BANK
#define DCSM_Z1_EXEONLYSECT2R_EXEONLY_CPU2_SECT1    0x2U      // Execute-Only Flash Sector 1 in
                                                              // flash CPU2 BANK
#define DCSM_Z1_EXEONLYSECT2R_EXEONLY_CPU2_SECT2    0x4U      // Execute-Only Flash Sector 2 in
                                                              // flash CPU2 BANK
#define DCSM_Z1_EXEONLYSECT2R_EXEONLY_CPU2_SECT3    0x8U      // Execute-Only Flash Sector 3 in
                                                              // flash CPU2 BANK
#define DCSM_Z1_EXEONLYSECT2R_EXEONLY_CPU2_SECT4    0x10U     // Execute-Only Flash Sector 4 in
                                                              // flash CPU2 BANK
#define DCSM_Z1_EXEONLYSECT2R_EXEONLY_CPU2_SECT5    0x20U     // Execute-Only Flash Sector 5 in
                                                              // flash CPU2 BANK
#define DCSM_Z1_EXEONLYSECT2R_EXEONLY_CPU2_SECT6    0x40U     // Execute-Only Flash Sector 6 in
                                                              // flash CPU2 BANK
#define DCSM_Z1_EXEONLYSECT2R_EXEONLY_CPU2_SECT7    0x80U     // Execute-Only Flash Sector 7 in
                                                              // flash CPU2 BANK
#define DCSM_Z1_EXEONLYSECT2R_EXEONLY_CPU2_SECT8    0x100U    // Execute-Only Flash Sector 8 in
                                                              // flash CPU2 BANK
#define DCSM_Z1_EXEONLYSECT2R_EXEONLY_CPU2_SECT9    0x200U    // Execute-Only Flash Sector 9 in
                                                              // flash CPU2 BANK
#define DCSM_Z1_EXEONLYSECT2R_EXEONLY_CPU2_SECT10   0x400U    // Execute-Only Flash Sector 10 in
                                                              // flash CPU2 BANK
#define DCSM_Z1_EXEONLYSECT2R_EXEONLY_CPU2_SECT11   0x800U    // Execute-Only Flash Sector 11 in
                                                              // flash CPU2 BANK
#define DCSM_Z1_EXEONLYSECT2R_EXEONLY_CPU2_SECT12   0x1000U   // Execute-Only Flash Sector 12 in
                                                              // flash CPU2 BANK
#define DCSM_Z1_EXEONLYSECT2R_EXEONLY_CPU2_SECT13   0x2000U   // Execute-Only Flash Sector 13 in
                                                              // flash CPU2 BANK

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z1_EXEONLYRAM1R register
//
//*************************************************************************************************
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM0    0x1U          // Execute-Only RAM CPU1.LS0
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM1    0x2U          // Execute-Only RAM CPU1.LS1
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM2    0x4U          // Execute-Only RAM CPU1.LS2
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM3    0x8U          // Execute-Only RAM CPU1.LS3
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM4    0x10U         // Execute-Only RAM CPU1.LS4
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM5    0x20U         // Execute-Only RAM CPU1.LS5
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM6    0x40U         // Execute-Only RAM CPU1.LS6
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM7    0x80U         // Execute-Only RAM CPU1.LS7
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM8    0x100U        // Execute-Only RAM CPU1.D0
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM9    0x200U        // Execute-Only RAM CPU1.D1
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM16   0x10000U      // Execute-Only RAM on CM.C0
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM17   0x20000U      // Execute-Only RAM on CM.C1
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM22   0x400000U     // Execute-Only RAM CPU2.D1
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM23   0x800000U     // Execute-Only RAM CPU2.D0
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM24   0x1000000U    // Execute-Only RAM CPU2.LS7
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM25   0x2000000U    // Execute-Only RAM CPU2.LS6
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM26   0x4000000U    // Execute-Only RAM CPU2.LS5
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM27   0x8000000U    // Execute-Only RAM CPU2.LS4
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM28   0x10000000U   // Execute-Only RAM CPU2.LS3
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM29   0x20000000U   // Execute-Only RAM CPU2.LS2
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM30   0x40000000U   // Execute-Only RAM CPU2.LS1
#define DCSM_Z1_EXEONLYRAM1R_EXEONLY_RAM31   0x80000000U   // Execute-Only RAM CPU2.LS0


//*************************************************************************************************
//
// The following are defines for the bit fields in the Z2_LINKPOINTER register
//
//*************************************************************************************************
#define DCSM_Z2_LINKPOINTER_LINKPOINTER_S   0U
#define DCSM_Z2_LINKPOINTER_LINKPOINTER_M   0x3FFFU   // Zone2 LINK Pointer

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z2_OTPSECLOCK register
//
//*************************************************************************************************
#define DCSM_Z2_OTPSECLOCK_JTAGLOCK     0x1U     // JTAG Lock Status
#define DCSM_Z2_OTPSECLOCK_PSWDLOCK_S   4U
#define DCSM_Z2_OTPSECLOCK_PSWDLOCK_M   0xF0U    // Zone2 Password Lock.
#define DCSM_Z2_OTPSECLOCK_CRCLOCK_S    8U
#define DCSM_Z2_OTPSECLOCK_CRCLOCK_M    0xF00U   // Zone2 CRC Lock.

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z2_LINKPOINTERERR register
//
//*************************************************************************************************
#define DCSM_Z2_LINKPOINTERERR_Z2_LINKPOINTERERR_S   0U
#define DCSM_Z2_LINKPOINTERERR_Z2_LINKPOINTERERR_M   0x3FFFU   // Error to Resolve Z2 Link pointer
                                                               // from OTP loaded values

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z2_CR register
//
//*************************************************************************************************
#define DCSM_Z2_CR_ALLZERO    0x80000U      // CSMPSWD All Zeros
#define DCSM_Z2_CR_ALLONE     0x100000U     // CSMPSWD All Ones
#define DCSM_Z2_CR_UNSECURE   0x200000U     // CSMPSWD Match CSMKEY
#define DCSM_Z2_CR_ARMED      0x400000U     // CSM Passwords Read Status
#define DCSM_Z2_CR_FORCESEC   0x80000000U   // Force Secure

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z2_GRABSECT1R register
//
//*************************************************************************************************
#define DCSM_Z2_GRABSECT1R_GRAB_SECT0_S    0U
#define DCSM_Z2_GRABSECT1R_GRAB_SECT0_M    0x3U         // Grab Flash Sector 0 in CPU1 BANK
#define DCSM_Z2_GRABSECT1R_GRAB_SECT1_S    2U
#define DCSM_Z2_GRABSECT1R_GRAB_SECT1_M    0xCU         // Grab Flash Sector 1 in CPU1 BANK
#define DCSM_Z2_GRABSECT1R_GRAB_SECT2_S    4U
#define DCSM_Z2_GRABSECT1R_GRAB_SECT2_M    0x30U        // Grab Flash Sector 2 in CPU1 BANK
#define DCSM_Z2_GRABSECT1R_GRAB_SECT3_S    6U
#define DCSM_Z2_GRABSECT1R_GRAB_SECT3_M    0xC0U        // Grab Flash Sector 3 in CPU1 BANK
#define DCSM_Z2_GRABSECT1R_GRAB_SECT4_S    8U
#define DCSM_Z2_GRABSECT1R_GRAB_SECT4_M    0x300U       // Grab Flash Sector 4 in CPU1 BANK
#define DCSM_Z2_GRABSECT1R_GRAB_SECT5_S    10U
#define DCSM_Z2_GRABSECT1R_GRAB_SECT5_M    0xC00U       // Grab Flash Sector 5 in CPU1 BANK
#define DCSM_Z2_GRABSECT1R_GRAB_SECT6_S    12U
#define DCSM_Z2_GRABSECT1R_GRAB_SECT6_M    0x3000U      // Grab Flash Sector 6 in CPU1 BANK
#define DCSM_Z2_GRABSECT1R_GRAB_SECT7_S    14U
#define DCSM_Z2_GRABSECT1R_GRAB_SECT7_M    0xC000U      // Grab Flash Sector 7 in CPU1 BANK
#define DCSM_Z2_GRABSECT1R_GRAB_SECT8_S    16U
#define DCSM_Z2_GRABSECT1R_GRAB_SECT8_M    0x30000U     // Grab Flash Sector 8 in CPU1 BANK
#define DCSM_Z2_GRABSECT1R_GRAB_SECT9_S    18U
#define DCSM_Z2_GRABSECT1R_GRAB_SECT9_M    0xC0000U     // Grab Flash Sector 9 in CPU1 BANK
#define DCSM_Z2_GRABSECT1R_GRAB_SECT10_S   20U
#define DCSM_Z2_GRABSECT1R_GRAB_SECT10_M   0x300000U    // Grab Flash Sector 10 in CPU1 BANK
#define DCSM_Z2_GRABSECT1R_GRAB_SECT11_S   22U
#define DCSM_Z2_GRABSECT1R_GRAB_SECT11_M   0xC00000U    // Grab Flash Sector 11 in CPU1 BANK
#define DCSM_Z2_GRABSECT1R_GRAB_SECT12_S   24U
#define DCSM_Z2_GRABSECT1R_GRAB_SECT12_M   0x3000000U   // Grab Flash Sector 12 in CPU1 BANK
#define DCSM_Z2_GRABSECT1R_GRAB_SECT13_S   26U
#define DCSM_Z2_GRABSECT1R_GRAB_SECT13_M   0xC000000U   // Grab Flash Sector 13 in CPU1 BANK

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z2_GRABSECT2R register
//
//*************************************************************************************************
#define DCSM_Z2_GRABSECT2R_GRAB_SECT0_S    0U
#define DCSM_Z2_GRABSECT2R_GRAB_SECT0_M    0x3U         // Grab Flash Sector 0 in CM BANK
#define DCSM_Z2_GRABSECT2R_GRAB_SECT1_S    2U
#define DCSM_Z2_GRABSECT2R_GRAB_SECT1_M    0xCU         // Grab Flash Sector 1 in CM BANK
#define DCSM_Z2_GRABSECT2R_GRAB_SECT2_S    4U
#define DCSM_Z2_GRABSECT2R_GRAB_SECT2_M    0x30U        // Grab Flash Sector 2 in CM BANK
#define DCSM_Z2_GRABSECT2R_GRAB_SECT3_S    6U
#define DCSM_Z2_GRABSECT2R_GRAB_SECT3_M    0xC0U        // Grab Flash Sector 3 in CM BANK
#define DCSM_Z2_GRABSECT2R_GRAB_SECT4_S    8U
#define DCSM_Z2_GRABSECT2R_GRAB_SECT4_M    0x300U       // Grab Flash Sector 4 in CM BANK
#define DCSM_Z2_GRABSECT2R_GRAB_SECT5_S    10U
#define DCSM_Z2_GRABSECT2R_GRAB_SECT5_M    0xC00U       // Grab Flash Sector 5 in CM BANK
#define DCSM_Z2_GRABSECT2R_GRAB_SECT6_S    12U
#define DCSM_Z2_GRABSECT2R_GRAB_SECT6_M    0x3000U      // Grab Flash Sector 6 in CM BANK
#define DCSM_Z2_GRABSECT2R_GRAB_SECT7_S    14U
#define DCSM_Z2_GRABSECT2R_GRAB_SECT7_M    0xC000U      // Grab Flash Sector 7 in CM BANK
#define DCSM_Z2_GRABSECT2R_GRAB_SECT8_S    16U
#define DCSM_Z2_GRABSECT2R_GRAB_SECT8_M    0x30000U     // Grab Flash Sector 8 in CM BANK
#define DCSM_Z2_GRABSECT2R_GRAB_SECT9_S    18U
#define DCSM_Z2_GRABSECT2R_GRAB_SECT9_M    0xC0000U     // Grab Flash Sector 9 in CM BANK
#define DCSM_Z2_GRABSECT2R_GRAB_SECT10_S   20U
#define DCSM_Z2_GRABSECT2R_GRAB_SECT10_M   0x300000U    // Grab Flash Sector 10 in CM BANK
#define DCSM_Z2_GRABSECT2R_GRAB_SECT11_S   22U
#define DCSM_Z2_GRABSECT2R_GRAB_SECT11_M   0xC00000U    // Grab Flash Sector 11 in CM BANK
#define DCSM_Z2_GRABSECT2R_GRAB_SECT12_S   24U
#define DCSM_Z2_GRABSECT2R_GRAB_SECT12_M   0x3000000U   // Grab Flash Sector 12 in CM BANK
#define DCSM_Z2_GRABSECT2R_GRAB_SECT13_S   26U
#define DCSM_Z2_GRABSECT2R_GRAB_SECT13_M   0xC000000U   // Grab Flash Sector 13 in CM BANK

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z2_GRABSECT3R register
//
//*************************************************************************************************
#define DCSM_Z2_GRABSECT3R_GRAB_SECT0_S    0U
#define DCSM_Z2_GRABSECT3R_GRAB_SECT0_M    0x3U         // Grab Flash Sector 0 in CPU2 BANK
#define DCSM_Z2_GRABSECT3R_GRAB_SECT1_S    2U
#define DCSM_Z2_GRABSECT3R_GRAB_SECT1_M    0xCU         // Grab Flash Sector 1 in CPU2 BANK
#define DCSM_Z2_GRABSECT3R_GRAB_SECT2_S    4U
#define DCSM_Z2_GRABSECT3R_GRAB_SECT2_M    0x30U        // Grab Flash Sector 2 in CPU2 BANK
#define DCSM_Z2_GRABSECT3R_GRAB_SECT3_S    6U
#define DCSM_Z2_GRABSECT3R_GRAB_SECT3_M    0xC0U        // Grab Flash Sector 3 in CPU2 BANK
#define DCSM_Z2_GRABSECT3R_GRAB_SECT4_S    8U
#define DCSM_Z2_GRABSECT3R_GRAB_SECT4_M    0x300U       // Grab Flash Sector 4 in CPU2 BANK
#define DCSM_Z2_GRABSECT3R_GRAB_SECT5_S    10U
#define DCSM_Z2_GRABSECT3R_GRAB_SECT5_M    0xC00U       // Grab Flash Sector 5 in CPU2 BANK
#define DCSM_Z2_GRABSECT3R_GRAB_SECT6_S    12U
#define DCSM_Z2_GRABSECT3R_GRAB_SECT6_M    0x3000U      // Grab Flash Sector 6 in CPU2 BANK
#define DCSM_Z2_GRABSECT3R_GRAB_SECT7_S    14U
#define DCSM_Z2_GRABSECT3R_GRAB_SECT7_M    0xC000U      // Grab Flash Sector 7 in CPU2 BANK
#define DCSM_Z2_GRABSECT3R_GRAB_SECT8_S    16U
#define DCSM_Z2_GRABSECT3R_GRAB_SECT8_M    0x30000U     // Grab Flash Sector 8 in CPU2 BANK
#define DCSM_Z2_GRABSECT3R_GRAB_SECT9_S    18U
#define DCSM_Z2_GRABSECT3R_GRAB_SECT9_M    0xC0000U     // Grab Flash Sector 9 in CPU2 BANK
#define DCSM_Z2_GRABSECT3R_GRAB_SECT10_S   20U
#define DCSM_Z2_GRABSECT3R_GRAB_SECT10_M   0x300000U    // Grab Flash Sector 10 in CPU2 BANK
#define DCSM_Z2_GRABSECT3R_GRAB_SECT11_S   22U
#define DCSM_Z2_GRABSECT3R_GRAB_SECT11_M   0xC00000U    // Grab Flash Sector 11 in CPU2 BANK
#define DCSM_Z2_GRABSECT3R_GRAB_SECT12_S   24U
#define DCSM_Z2_GRABSECT3R_GRAB_SECT12_M   0x3000000U   // Grab Flash Sector 12 in CPU2 BANK
#define DCSM_Z2_GRABSECT3R_GRAB_SECT13_S   26U
#define DCSM_Z2_GRABSECT3R_GRAB_SECT13_M   0xC000000U   // Grab Flash Sector 13 in CPU2 BANK

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z2_GRABRAM1R register
//
//*************************************************************************************************
#define DCSM_Z2_GRABRAM1R_GRAB_RAM0_S   0U
#define DCSM_Z2_GRABRAM1R_GRAB_RAM0_M   0x3U       // Grab RAM CPU1.LS0
#define DCSM_Z2_GRABRAM1R_GRAB_RAM1_S   2U
#define DCSM_Z2_GRABRAM1R_GRAB_RAM1_M   0xCU       // Grab RAM CPU1.LS1
#define DCSM_Z2_GRABRAM1R_GRAB_RAM2_S   4U
#define DCSM_Z2_GRABRAM1R_GRAB_RAM2_M   0x30U      // Grab RAM CPU1.LS2
#define DCSM_Z2_GRABRAM1R_GRAB_RAM3_S   6U
#define DCSM_Z2_GRABRAM1R_GRAB_RAM3_M   0xC0U      // Grab RAM CPU1.LS3
#define DCSM_Z2_GRABRAM1R_GRAB_RAM4_S   8U
#define DCSM_Z2_GRABRAM1R_GRAB_RAM4_M   0x300U     // Grab RAM CPU1.LS4
#define DCSM_Z2_GRABRAM1R_GRAB_RAM5_S   10U
#define DCSM_Z2_GRABRAM1R_GRAB_RAM5_M   0xC00U     // Grab RAM CPU1.LS5
#define DCSM_Z2_GRABRAM1R_GRAB_RAM6_S   12U
#define DCSM_Z2_GRABRAM1R_GRAB_RAM6_M   0x3000U    // Grab RAM CPU1.LS6
#define DCSM_Z2_GRABRAM1R_GRAB_RAM7_S   14U
#define DCSM_Z2_GRABRAM1R_GRAB_RAM7_M   0xC000U    // Grab RAM CPU1.LS7
#define DCSM_Z2_GRABRAM1R_GRAB_RAM8_S   16U
#define DCSM_Z2_GRABRAM1R_GRAB_RAM8_M   0x30000U   // Grab RAM CPU1.D0
#define DCSM_Z2_GRABRAM1R_GRAB_RAM9_S   18U
#define DCSM_Z2_GRABRAM1R_GRAB_RAM9_M   0xC0000U   // Grab RAM CPU1.D1

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z2_GRABRAM2R register
//
//*************************************************************************************************
#define DCSM_Z2_GRABRAM2R_GRAB_RAM0_S    0U
#define DCSM_Z2_GRABRAM2R_GRAB_RAM0_M    0x3U          // Grab RAM CM.C0
#define DCSM_Z2_GRABRAM2R_GRAB_RAM1_S    2U
#define DCSM_Z2_GRABRAM2R_GRAB_RAM1_M    0xCU          // Grab RAM CM.C1
#define DCSM_Z2_GRABRAM2R_GRAB_RAM4_S    8U
#define DCSM_Z2_GRABRAM2R_GRAB_RAM4_M    0x300U        // Grab RAM CPU1TOCM MSGRAM0_L
#define DCSM_Z2_GRABRAM2R_GRAB_RAM5_S    10U
#define DCSM_Z2_GRABRAM2R_GRAB_RAM5_M    0xC00U        // Grab RAM CPU1TOCM MSGRAM0_H
#define DCSM_Z2_GRABRAM2R_GRAB_RAM6_S    12U
#define DCSM_Z2_GRABRAM2R_GRAB_RAM6_M    0x3000U       // Grab RAM CMTOCPU1 MSGRAM0_L
#define DCSM_Z2_GRABRAM2R_GRAB_RAM7_S    14U
#define DCSM_Z2_GRABRAM2R_GRAB_RAM7_M    0xC000U       // Grab RAM CMTOCPU1 MSGRAM0_H
#define DCSM_Z2_GRABRAM2R_GRAB_RAM8_S    16U
#define DCSM_Z2_GRABRAM2R_GRAB_RAM8_M    0x30000U      // Grab RAM CPU2TOCM MSGRAM0_L
#define DCSM_Z2_GRABRAM2R_GRAB_RAM9_S    18U
#define DCSM_Z2_GRABRAM2R_GRAB_RAM9_M    0xC0000U      // Grab RAM CPU2TOCM MSGRAM0_H
#define DCSM_Z2_GRABRAM2R_GRAB_RAM10_S   20U
#define DCSM_Z2_GRABRAM2R_GRAB_RAM10_M   0x300000U     // Grab RAM CMTOCPU2 MSGRAM0_L
#define DCSM_Z2_GRABRAM2R_GRAB_RAM11_S   22U
#define DCSM_Z2_GRABRAM2R_GRAB_RAM11_M   0xC00000U     // Grab RAM CMTOCPU2 MSGRAM0_H
#define DCSM_Z2_GRABRAM2R_GRAB_RAM12_S   24U
#define DCSM_Z2_GRABRAM2R_GRAB_RAM12_M   0x3000000U    // Grab RAM CPU1TOCPU2 MSGRAM0_L
#define DCSM_Z2_GRABRAM2R_GRAB_RAM13_S   26U
#define DCSM_Z2_GRABRAM2R_GRAB_RAM13_M   0xC000000U    // Grab RAM CPU1TOCPU2 MSGRAM0_H
#define DCSM_Z2_GRABRAM2R_GRAB_RAM14_S   28U
#define DCSM_Z2_GRABRAM2R_GRAB_RAM14_M   0x30000000U   // Grab RAM CPU2TOCPU1 MSGRAM0_L
#define DCSM_Z2_GRABRAM2R_GRAB_RAM15_S   30U
#define DCSM_Z2_GRABRAM2R_GRAB_RAM15_M   0xC0000000U   // Grab RAM CPU2TOCPU1 MSGRAM0_H

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z2_GRABRAM3R register
//
//*************************************************************************************************
#define DCSM_Z2_GRABRAM3R_GRAB_RAM0_S   0U
#define DCSM_Z2_GRABRAM3R_GRAB_RAM0_M   0x3U       // Grab RAM CPU2.LS0
#define DCSM_Z2_GRABRAM3R_GRAB_RAM1_S   2U
#define DCSM_Z2_GRABRAM3R_GRAB_RAM1_M   0xCU       // Grab RAM CPU2.LS1
#define DCSM_Z2_GRABRAM3R_GRAB_RAM2_S   4U
#define DCSM_Z2_GRABRAM3R_GRAB_RAM2_M   0x30U      // Grab RAM CPU2.LS2
#define DCSM_Z2_GRABRAM3R_GRAB_RAM3_S   6U
#define DCSM_Z2_GRABRAM3R_GRAB_RAM3_M   0xC0U      // Grab RAM CPU2.LS3
#define DCSM_Z2_GRABRAM3R_GRAB_RAM4_S   8U
#define DCSM_Z2_GRABRAM3R_GRAB_RAM4_M   0x300U     // Grab RAM CPU2.LS4
#define DCSM_Z2_GRABRAM3R_GRAB_RAM5_S   10U
#define DCSM_Z2_GRABRAM3R_GRAB_RAM5_M   0xC00U     // Grab RAM CPU2.LS5
#define DCSM_Z2_GRABRAM3R_GRAB_RAM6_S   12U
#define DCSM_Z2_GRABRAM3R_GRAB_RAM6_M   0x3000U    // Grab RAM CPU2.LS6
#define DCSM_Z2_GRABRAM3R_GRAB_RAM7_S   14U
#define DCSM_Z2_GRABRAM3R_GRAB_RAM7_M   0xC000U    // Grab RAM CPU2.LS7
#define DCSM_Z2_GRABRAM3R_GRAB_RAM8_S   16U
#define DCSM_Z2_GRABRAM3R_GRAB_RAM8_M   0x30000U   // Grab RAM CPU2.D0
#define DCSM_Z2_GRABRAM3R_GRAB_RAM9_S   18U
#define DCSM_Z2_GRABRAM3R_GRAB_RAM9_M   0xC0000U   // Grab RAM CPU2.D1

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z2_EXEONLYSECT1R register
//
//*************************************************************************************************
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CPU1_SECT0    0x1U          // Execute-Only Flash Sector 0 in
                                                                  // flash CPU1 BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CPU1_SECT1    0x2U          // Execute-Only Flash Sector 1 in
                                                                  // flash CPU1 BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CPU1_SECT2    0x4U          // Execute-Only Flash Sector 2 in
                                                                  // flash CPU1 BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CPU1_SECT3    0x8U          // Execute-Only Flash Sector 3 in
                                                                  // flash CPU1 BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CPU1_SECT4    0x10U         // Execute-Only Flash Sector 4 in
                                                                  // flash CPU1 BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CPU1_SECT5    0x20U         // Execute-Only Flash Sector 5 in
                                                                  // flash CPU1 BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CPU1_SECT6    0x40U         // Execute-Only Flash Sector 6 in
                                                                  // flash CPU1 BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CPU1_SECT7    0x80U         // Execute-Only Flash Sector 7 in
                                                                  // flash CPU1 BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CPU1_SECT8    0x100U        // Execute-Only Flash Sector 8 in
                                                                  // flash CPU1 BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CPU1_SECT9    0x200U        // Execute-Only Flash Sector 9 in
                                                                  // flash CPU1 BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CPU1_SECT10   0x400U        // Execute-Only Flash Sector 10
                                                                  // in flash CPU1 BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CPU1_SECT11   0x800U        // Execute-Only Flash Sector 11
                                                                  // in flash CPU1 BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CPU1_SECT12   0x1000U       // Execute-Only Flash Sector 12
                                                                  // in flash CPU1 BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CPU1_SECT13   0x2000U       // Execute-Only Flash Sector 13
                                                                  // in flash CPU1 BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CM_SECT0      0x10000U      // Execute-Only Flash Sector 0 in
                                                                  // flash CM BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CM_SECT1      0x20000U      // Execute-Only Flash Sector 1 in
                                                                  // flash CM BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CM_SECT2      0x40000U      // Execute-Only Flash Sector 2 in
                                                                  // flash CM BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CM_SECT3      0x80000U      // Execute-Only Flash Sector 3 in
                                                                  // flash CM BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CM_SECT4      0x100000U     // Execute-Only Flash Sector 4 in
                                                                  // flash CM BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CM_SECT5      0x200000U     // Execute-Only Flash Sector 5 in
                                                                  // flash CM BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CM_SECT6      0x400000U     // Execute-Only Flash Sector 6 in
                                                                  // flash CM BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CM_SECT7      0x800000U     // Execute-Only Flash Sector 7 in
                                                                  // flash CM BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CM_SECT8      0x1000000U    // Execute-Only Flash Sector 8 in
                                                                  // flash CM BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CM_SECT9      0x2000000U    // Execute-Only Flash Sector 9 in
                                                                  // flash CM BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CM_SECT10     0x4000000U    // Execute-Only Flash Sector 10
                                                                  // in flash CM BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CM_SECT11     0x8000000U    // Execute-Only Flash Sector 11
                                                                  // in flash CM BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CM_SECT12     0x10000000U   // Execute-Only Flash Sector 12
                                                                  // in flash CM BANK
#define DCSM_Z2_EXEONLYSECT1R_EXEONLY_CM_SECT13     0x20000000U   // Execute-Only Flash Sector 13
                                                                  // in flash CM BANK

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z2_EXEONLYSECT2R register
//
//*************************************************************************************************
#define DCSM_Z2_EXEONLYSECT2R_EXEONLY_CPU2_SECT0    0x1U      // Execute-Only Flash Sector 0 in
                                                              // flash CPU2 BANK
#define DCSM_Z2_EXEONLYSECT2R_EXEONLY_CPU2_SECT1    0x2U      // Execute-Only Flash Sector 1 in
                                                              // flash CPU2 BANK
#define DCSM_Z2_EXEONLYSECT2R_EXEONLY_CPU2_SECT2    0x4U      // Execute-Only Flash Sector 2 in
                                                              // flash CPU2 BANK
#define DCSM_Z2_EXEONLYSECT2R_EXEONLY_CPU2_SECT3    0x8U      // Execute-Only Flash Sector 3 in
                                                              // flash CPU2 BANK
#define DCSM_Z2_EXEONLYSECT2R_EXEONLY_CPU2_SECT4    0x10U     // Execute-Only Flash Sector 4 in
                                                              // flash CPU2 BANK
#define DCSM_Z2_EXEONLYSECT2R_EXEONLY_CPU2_SECT5    0x20U     // Execute-Only Flash Sector 5 in
                                                              // flash CPU2 BANK
#define DCSM_Z2_EXEONLYSECT2R_EXEONLY_CPU2_SECT6    0x40U     // Execute-Only Flash Sector 6 in
                                                              // flash CPU2 BANK
#define DCSM_Z2_EXEONLYSECT2R_EXEONLY_CPU2_SECT7    0x80U     // Execute-Only Flash Sector 7 in
                                                              // flash CPU2 BANK
#define DCSM_Z2_EXEONLYSECT2R_EXEONLY_CPU2_SECT8    0x100U    // Execute-Only Flash Sector 8 in
                                                              // flash CPU2 BANK
#define DCSM_Z2_EXEONLYSECT2R_EXEONLY_CPU2_SECT9    0x200U    // Execute-Only Flash Sector 9 in
                                                              // flash CPU2 BANK
#define DCSM_Z2_EXEONLYSECT2R_EXEONLY_CPU2_SECT10   0x400U    // Execute-Only Flash Sector 10 in
                                                              // flash CPU2 BANK
#define DCSM_Z2_EXEONLYSECT2R_EXEONLY_CPU2_SECT11   0x800U    // Execute-Only Flash Sector 11 in
                                                              // flash CPU2 BANK
#define DCSM_Z2_EXEONLYSECT2R_EXEONLY_CPU2_SECT12   0x1000U   // Execute-Only Flash Sector 12 in
                                                              // flash CPU2 BANK
#define DCSM_Z2_EXEONLYSECT2R_EXEONLY_CPU2_SECT13   0x2000U   // Execute-Only Flash Sector 13 in
                                                              // flash CPU2 BANK

//*************************************************************************************************
//
// The following are defines for the bit fields in the Z2_EXEONLYRAM1R register
//
//*************************************************************************************************
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM0    0x1U          // Execute-Only RAM CPU1.LS0
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM1    0x2U          // Execute-Only RAM CPU1.LS1
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM2    0x4U          // Execute-Only RAM CPU1.LS2
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM3    0x8U          // Execute-Only RAM CPU1.LS3
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM4    0x10U         // Execute-Only RAM CPU1.LS4
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM5    0x20U         // Execute-Only RAM CPU1.LS5
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM6    0x40U         // Execute-Only RAM CPU1.LS6
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM7    0x80U         // Execute-Only RAM CPU1.LS7
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM8    0x100U        // Execute-Only RAM CPU1.D0
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM9    0x200U        // Execute-Only RAM CPU1.D1
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM16   0x10000U      // Execute-Only RAM on CM.C0
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM17   0x20000U      // Execute-Only RAM on CM.C1
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM22   0x400000U     // Execute-Only RAM CPU2.D1
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM23   0x800000U     // Execute-Only RAM CPU2.D0
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM24   0x1000000U    // Execute-Only RAM CPU2.LS7
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM25   0x2000000U    // Execute-Only RAM CPU2.LS6
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM26   0x4000000U    // Execute-Only RAM CPU2.LS5
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM27   0x8000000U    // Execute-Only RAM CPU2.LS4
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM28   0x10000000U   // Execute-Only RAM CPU2.LS3
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM29   0x20000000U   // Execute-Only RAM CPU2.LS2
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM30   0x40000000U   // Execute-Only RAM CPU2.LS1
#define DCSM_Z2_EXEONLYRAM1R_EXEONLY_RAM31   0x80000000U   // Execute-Only RAM CPU2.LS0


//*************************************************************************************************
//
// The following are defines for the bit fields in the FLSEM register
//
//*************************************************************************************************
#define DCSM_FLSEM_SEM_S   0U
#define DCSM_FLSEM_SEM_M   0x3U      // Flash Semaphore Bit
#define DCSM_FLSEM_KEY_S   8U
#define DCSM_FLSEM_KEY_M   0xFF00U   // Semaphore Key

//*************************************************************************************************
//
// The following are defines for the bit fields in the SECTSTAT1 register
//
//*************************************************************************************************
#define DCSM_SECTSTAT1_STATUS_SECT0_S    0U
#define DCSM_SECTSTAT1_STATUS_SECT0_M    0x3U         // Zone Status flash CPU1 BANK Sector 0
#define DCSM_SECTSTAT1_STATUS_SECT1_S    2U
#define DCSM_SECTSTAT1_STATUS_SECT1_M    0xCU         // Zone Status flash CPU1 BANK sector 1
#define DCSM_SECTSTAT1_STATUS_SECT2_S    4U
#define DCSM_SECTSTAT1_STATUS_SECT2_M    0x30U        // Zone Status flash CPU1 BANK Sector 2
#define DCSM_SECTSTAT1_STATUS_SECT3_S    6U
#define DCSM_SECTSTAT1_STATUS_SECT3_M    0xC0U        // Zone Status flash CPU1 BANK Sector 3
#define DCSM_SECTSTAT1_STATUS_SECT4_S    8U
#define DCSM_SECTSTAT1_STATUS_SECT4_M    0x300U       // Zone Status flash CPU1 BANK Sector 4
#define DCSM_SECTSTAT1_STATUS_SECT5_S    10U
#define DCSM_SECTSTAT1_STATUS_SECT5_M    0xC00U       // Zone Status flash CPU1 BANK Sector 5
#define DCSM_SECTSTAT1_STATUS_SECT6_S    12U
#define DCSM_SECTSTAT1_STATUS_SECT6_M    0x3000U      // Zone Status flash CPU1 BANK Sector 6
#define DCSM_SECTSTAT1_STATUS_SECT7_S    14U
#define DCSM_SECTSTAT1_STATUS_SECT7_M    0xC000U      // Zone Status flash CPU1 BANK Sector 7
#define DCSM_SECTSTAT1_STATUS_SECT8_S    16U
#define DCSM_SECTSTAT1_STATUS_SECT8_M    0x30000U     // Zone Status flash CPU1 BANK sector 8
#define DCSM_SECTSTAT1_STATUS_SECT9_S    18U
#define DCSM_SECTSTAT1_STATUS_SECT9_M    0xC0000U     // Zone Status flash CPU1 BANK Sector 9
#define DCSM_SECTSTAT1_STATUS_SECT10_S   20U
#define DCSM_SECTSTAT1_STATUS_SECT10_M   0x300000U    // Zone Status flash CPU1 BANK Sector 10
#define DCSM_SECTSTAT1_STATUS_SECT11_S   22U
#define DCSM_SECTSTAT1_STATUS_SECT11_M   0xC00000U    // Zone Status flash CPU1 BANK Sector 11
#define DCSM_SECTSTAT1_STATUS_SECT12_S   24U
#define DCSM_SECTSTAT1_STATUS_SECT12_M   0x3000000U   // Zone Status flash CPU1 BANK Sector 12
#define DCSM_SECTSTAT1_STATUS_SECT13_S   26U
#define DCSM_SECTSTAT1_STATUS_SECT13_M   0xC000000U   // Zone Status flash CPU1 BANK Sector 13

//*************************************************************************************************
//
// The following are defines for the bit fields in the SECTSTAT2 register
//
//*************************************************************************************************
#define DCSM_SECTSTAT2_STATUS_SECT0_S    0U
#define DCSM_SECTSTAT2_STATUS_SECT0_M    0x3U         // Zone Status flash CM BANK Sector 0
#define DCSM_SECTSTAT2_STATUS_SECT1_S    2U
#define DCSM_SECTSTAT2_STATUS_SECT1_M    0xCU         // Zone Status flash CM BANK sector 1
#define DCSM_SECTSTAT2_STATUS_SECT2_S    4U
#define DCSM_SECTSTAT2_STATUS_SECT2_M    0x30U        // Zone Status flash CM BANK Sector 2
#define DCSM_SECTSTAT2_STATUS_SECT3_S    6U
#define DCSM_SECTSTAT2_STATUS_SECT3_M    0xC0U        // Zone Status flash CM BANK Sector 3
#define DCSM_SECTSTAT2_STATUS_SECT4_S    8U
#define DCSM_SECTSTAT2_STATUS_SECT4_M    0x300U       // Zone Status flash CM BANK Sector 4
#define DCSM_SECTSTAT2_STATUS_SECT5_S    10U
#define DCSM_SECTSTAT2_STATUS_SECT5_M    0xC00U       // Zone Status flash CM BANK Sector 5
#define DCSM_SECTSTAT2_STATUS_SECT6_S    12U
#define DCSM_SECTSTAT2_STATUS_SECT6_M    0x3000U      // Zone Status flash CM BANK Sector 6
#define DCSM_SECTSTAT2_STATUS_SECT7_S    14U
#define DCSM_SECTSTAT2_STATUS_SECT7_M    0xC000U      // Zone Status flash CM BANK Sector 7
#define DCSM_SECTSTAT2_STATUS_SECT8_S    16U
#define DCSM_SECTSTAT2_STATUS_SECT8_M    0x30000U     // Zone Status flash CM BANK sector 8
#define DCSM_SECTSTAT2_STATUS_SECT9_S    18U
#define DCSM_SECTSTAT2_STATUS_SECT9_M    0xC0000U     // Zone Status flash CM BANK Sector 9
#define DCSM_SECTSTAT2_STATUS_SECT10_S   20U
#define DCSM_SECTSTAT2_STATUS_SECT10_M   0x300000U    // Zone Status flash CM BANK Sector 10
#define DCSM_SECTSTAT2_STATUS_SECT11_S   22U
#define DCSM_SECTSTAT2_STATUS_SECT11_M   0xC00000U    // Zone Status flash CM BANK Sector 11
#define DCSM_SECTSTAT2_STATUS_SECT12_S   24U
#define DCSM_SECTSTAT2_STATUS_SECT12_M   0x3000000U   // Zone Status flash CM BANK Sector 12
#define DCSM_SECTSTAT2_STATUS_SECT13_S   26U
#define DCSM_SECTSTAT2_STATUS_SECT13_M   0xC000000U   // Zone Status flash CM BANK Sector 13

//*************************************************************************************************
//
// The following are defines for the bit fields in the SECTSTAT3 register
//
//*************************************************************************************************
#define DCSM_SECTSTAT3_STATUS_SECT0_S    0U
#define DCSM_SECTSTAT3_STATUS_SECT0_M    0x3U         // Zone Status flash CPU2 BANK Sector 0
#define DCSM_SECTSTAT3_STATUS_SECT1_S    2U
#define DCSM_SECTSTAT3_STATUS_SECT1_M    0xCU         // Zone Status flash CPU2 BANK sector 1
#define DCSM_SECTSTAT3_STATUS_SECT2_S    4U
#define DCSM_SECTSTAT3_STATUS_SECT2_M    0x30U        // Zone Status flash CPU2 BANK Sector 2
#define DCSM_SECTSTAT3_STATUS_SECT3_S    6U
#define DCSM_SECTSTAT3_STATUS_SECT3_M    0xC0U        // Zone Status flash CPU2 BANK Sector 3
#define DCSM_SECTSTAT3_STATUS_SECT4_S    8U
#define DCSM_SECTSTAT3_STATUS_SECT4_M    0x300U       // Zone Status flash CPU2 BANK Sector 4
#define DCSM_SECTSTAT3_STATUS_SECT5_S    10U
#define DCSM_SECTSTAT3_STATUS_SECT5_M    0xC00U       // Zone Status flash CPU2 BANK Sector 5
#define DCSM_SECTSTAT3_STATUS_SECT6_S    12U
#define DCSM_SECTSTAT3_STATUS_SECT6_M    0x3000U      // Zone Status flash CPU2 BANK Sector 6
#define DCSM_SECTSTAT3_STATUS_SECT7_S    14U
#define DCSM_SECTSTAT3_STATUS_SECT7_M    0xC000U      // Zone Status flash CPU2 BANK Sector 7
#define DCSM_SECTSTAT3_STATUS_SECT8_S    16U
#define DCSM_SECTSTAT3_STATUS_SECT8_M    0x30000U     // Zone Status flash CPU2 BANK sector 8
#define DCSM_SECTSTAT3_STATUS_SECT9_S    18U
#define DCSM_SECTSTAT3_STATUS_SECT9_M    0xC0000U     // Zone Status flash CPU2 BANK Sector 9
#define DCSM_SECTSTAT3_STATUS_SECT10_S   20U
#define DCSM_SECTSTAT3_STATUS_SECT10_M   0x300000U    // Zone Status flash CPU2 BANK Sector 10
#define DCSM_SECTSTAT3_STATUS_SECT11_S   22U
#define DCSM_SECTSTAT3_STATUS_SECT11_M   0xC00000U    // Zone Status flash CPU2 BANK Sector 11
#define DCSM_SECTSTAT3_STATUS_SECT12_S   24U
#define DCSM_SECTSTAT3_STATUS_SECT12_M   0x3000000U   // Zone Status flash CPU2 BANK Sector 12
#define DCSM_SECTSTAT3_STATUS_SECT13_S   26U
#define DCSM_SECTSTAT3_STATUS_SECT13_M   0xC000000U   // Zone Status flash CPU2 BANK Sector 13

//*************************************************************************************************
//
// The following are defines for the bit fields in the RAMSTAT1 register
//
//*************************************************************************************************
#define DCSM_RAMSTAT1_STATUS_RAM0_S   0U
#define DCSM_RAMSTAT1_STATUS_RAM0_M   0x3U       // Zone Status RAM CPU1.LS0
#define DCSM_RAMSTAT1_STATUS_RAM1_S   2U
#define DCSM_RAMSTAT1_STATUS_RAM1_M   0xCU       // Zone Status RAM CPU1.LS1
#define DCSM_RAMSTAT1_STATUS_RAM2_S   4U
#define DCSM_RAMSTAT1_STATUS_RAM2_M   0x30U      // Zone Status RAM CPU1.LS2
#define DCSM_RAMSTAT1_STATUS_RAM3_S   6U
#define DCSM_RAMSTAT1_STATUS_RAM3_M   0xC0U      // Zone Status RAM CPU1.LS3
#define DCSM_RAMSTAT1_STATUS_RAM4_S   8U
#define DCSM_RAMSTAT1_STATUS_RAM4_M   0x300U     // Zone Status RAM CPU1.LS4
#define DCSM_RAMSTAT1_STATUS_RAM5_S   10U
#define DCSM_RAMSTAT1_STATUS_RAM5_M   0xC00U     // Zone Status RAM CPU1.LS5
#define DCSM_RAMSTAT1_STATUS_RAM6_S   12U
#define DCSM_RAMSTAT1_STATUS_RAM6_M   0x3000U    // Zone Status RAM CPU1.LS6
#define DCSM_RAMSTAT1_STATUS_RAM7_S   14U
#define DCSM_RAMSTAT1_STATUS_RAM7_M   0xC000U    // Zone Status RAM CPU1.LS7
#define DCSM_RAMSTAT1_STATUS_RAM8_S   16U
#define DCSM_RAMSTAT1_STATUS_RAM8_M   0x30000U   // Zone Status RAM CPU1.D0
#define DCSM_RAMSTAT1_STATUS_RAM9_S   18U
#define DCSM_RAMSTAT1_STATUS_RAM9_M   0xC0000U   // Zone Status RAM CPU1.D1

//*************************************************************************************************
//
// The following are defines for the bit fields in the RAMSTAT2 register
//
//*************************************************************************************************
#define DCSM_RAMSTAT2_STATUS_RAM0_S    0U
#define DCSM_RAMSTAT2_STATUS_RAM0_M    0x3U          // Zone Status RAM CM.C0
#define DCSM_RAMSTAT2_STATUS_RAM1_S    2U
#define DCSM_RAMSTAT2_STATUS_RAM1_M    0xCU          // Zone Status RAM CM.C1
#define DCSM_RAMSTAT2_STATUS_RAM4_S    8U
#define DCSM_RAMSTAT2_STATUS_RAM4_M    0x300U        // Zone Status RAM CPU1 to CM MSG RAM 1
#define DCSM_RAMSTAT2_STATUS_RAM5_S    10U
#define DCSM_RAMSTAT2_STATUS_RAM5_M    0xC00U        // Zone Status RAM CPU1 to CM MSG RAM 2
#define DCSM_RAMSTAT2_STATUS_RAM6_S    12U
#define DCSM_RAMSTAT2_STATUS_RAM6_M    0x3000U       // Zone Status RAM CM to CPU1 MSG RAM 1
#define DCSM_RAMSTAT2_STATUS_RAM7_S    14U
#define DCSM_RAMSTAT2_STATUS_RAM7_M    0xC000U       // Zone Status RAM CM to CPU1 MSG RAM 2
#define DCSM_RAMSTAT2_STATUS_RAM8_S    16U
#define DCSM_RAMSTAT2_STATUS_RAM8_M    0x30000U      // Zone Status RAM CPU2 to CM MSG RAM 1
#define DCSM_RAMSTAT2_STATUS_RAM9_S    18U
#define DCSM_RAMSTAT2_STATUS_RAM9_M    0xC0000U      // Zone Status RAM CPU2 to CM MSG RAM 2
#define DCSM_RAMSTAT2_STATUS_RAM10_S   20U
#define DCSM_RAMSTAT2_STATUS_RAM10_M   0x300000U     // Zone Status RAM CM to CPU2 MSG RAM 1
#define DCSM_RAMSTAT2_STATUS_RAM11_S   22U
#define DCSM_RAMSTAT2_STATUS_RAM11_M   0xC00000U     // Zone Status RAM CM to CPU2 MSG RAM 2
#define DCSM_RAMSTAT2_STATUS_RAM12_S   24U
#define DCSM_RAMSTAT2_STATUS_RAM12_M   0x3000000U    // Zone Status RAM CPU1 to CPU2 MSG RAM 1
#define DCSM_RAMSTAT2_STATUS_RAM13_S   26U
#define DCSM_RAMSTAT2_STATUS_RAM13_M   0xC000000U    // Zone Status RAM CPU1 to CPU2 MSG RAM 2
#define DCSM_RAMSTAT2_STATUS_RAM14_S   28U
#define DCSM_RAMSTAT2_STATUS_RAM14_M   0x30000000U   // Zone Status RAM CPU2 to CPU1 MSG RAM 1
#define DCSM_RAMSTAT2_STATUS_RAM15_S   30U
#define DCSM_RAMSTAT2_STATUS_RAM15_M   0xC0000000U   // Zone Status RAM CPU2 to CPU1 MSG RAM 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the RAMSTAT3 register
//
//*************************************************************************************************
#define DCSM_RAMSTAT3_STATUS_RAM0_S   0U
#define DCSM_RAMSTAT3_STATUS_RAM0_M   0x3U       // Zone Status RAM CPU2.LS0
#define DCSM_RAMSTAT3_STATUS_RAM1_S   2U
#define DCSM_RAMSTAT3_STATUS_RAM1_M   0xCU       // Zone Status RAM CPU2.LS1
#define DCSM_RAMSTAT3_STATUS_RAM2_S   4U
#define DCSM_RAMSTAT3_STATUS_RAM2_M   0x30U      // Zone Status RAM CPU2.LS2
#define DCSM_RAMSTAT3_STATUS_RAM3_S   6U
#define DCSM_RAMSTAT3_STATUS_RAM3_M   0xC0U      // Zone Status RAM CPU2.LS3
#define DCSM_RAMSTAT3_STATUS_RAM4_S   8U
#define DCSM_RAMSTAT3_STATUS_RAM4_M   0x300U     // Zone Status RAM CPU2.LS4
#define DCSM_RAMSTAT3_STATUS_RAM5_S   10U
#define DCSM_RAMSTAT3_STATUS_RAM5_M   0xC00U     // Zone Status RAM CPU2.LS5
#define DCSM_RAMSTAT3_STATUS_RAM6_S   12U
#define DCSM_RAMSTAT3_STATUS_RAM6_M   0x3000U    // Zone Status RAM CPU2.LS6
#define DCSM_RAMSTAT3_STATUS_RAM7_S   14U
#define DCSM_RAMSTAT3_STATUS_RAM7_M   0xC000U    // Zone Status RAM CPU2.LS7
#define DCSM_RAMSTAT3_STATUS_RAM8_S   16U
#define DCSM_RAMSTAT3_STATUS_RAM8_M   0x30000U   // Zone Status RAM CPU2.D0
#define DCSM_RAMSTAT3_STATUS_RAM9_S   18U
#define DCSM_RAMSTAT3_STATUS_RAM9_M   0xC0000U   // Zone Status RAM CPU2.D1

//*************************************************************************************************
//
// The following are defines for the bit fields in the SECERRSTAT register
//
//*************************************************************************************************
#define DCSM_SECERRSTAT_ERR   0x1U   // Security Configuration load Error Status

//*************************************************************************************************
//
// The following are defines for the bit fields in the SECERRCLR register
//
//*************************************************************************************************
#define DCSM_SECERRCLR_ERR   0x1U   // Clear Security Configuration Load Error Status Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SECERRFRC register
//
//*************************************************************************************************
#define DCSM_SECERRFRC_ERR     0x1U          // Set Security Configuration Load Error Status Bit
#define DCSM_SECERRFRC_KEY_S   16U
#define DCSM_SECERRFRC_KEY_M   0xFFFF0000U   // Valid Register Write Key



#endif
