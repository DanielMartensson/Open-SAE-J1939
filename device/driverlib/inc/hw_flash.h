//###########################################################################
//
// FILE:    hw_flash.h
//
// TITLE:   Definitions for the FLASH registers.
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

#ifndef HW_FLASH_H
#define HW_FLASH_H

//*************************************************************************************************
//
// The following are defines for the FLASH register offsets
//
//*************************************************************************************************
#define FLASH_O_FRDCNTL         0x0U     // Flash Read Control Register
#define FLASH_O_FBAC            0x1EU    // Flash Bank Access Control Register
#define FLASH_O_FBFALLBACK      0x20U    // Flash Bank Fallback Power Register
#define FLASH_O_FBPRDY          0x22U    // Flash Bank Pump Ready Register
#define FLASH_O_FPAC1           0x24U    // Flash Pump Access Control Register 1
#define FLASH_O_FMSTAT          0x2AU    // Flash Module Status Register
#define FLASH_O_FRD_INTF_CTRL   0x180U   // Flash Read Interface Control Register

#define FLASH_O_ECC_ENABLE             0x0U    // ECC Enable
#define FLASH_O_SINGLE_ERR_ADDR_LOW    0x2U    // Single Error Address Low
#define FLASH_O_SINGLE_ERR_ADDR_HIGH   0x4U    // Single Error Address High
#define FLASH_O_UNC_ERR_ADDR_LOW       0x6U    // Uncorrectable Error Address Low
#define FLASH_O_UNC_ERR_ADDR_HIGH      0x8U    // Uncorrectable Error Address High
#define FLASH_O_ERR_STATUS             0xAU    // Error Status
#define FLASH_O_ERR_POS                0xCU    // Error Position
#define FLASH_O_ERR_STATUS_CLR         0xEU    // Error Status Clear
#define FLASH_O_ERR_CNT                0x10U   // Error Control
#define FLASH_O_ERR_THRESHOLD          0x12U   // Error Threshold
#define FLASH_O_ERR_INTFLG             0x14U   // Error Interrupt Flag
#define FLASH_O_ERR_INTCLR             0x16U   // Error Interrupt Flag Clear
#define FLASH_O_FDATAH_TEST            0x18U   // Data High Test
#define FLASH_O_FDATAL_TEST            0x1AU   // Data Low Test
#define FLASH_O_FADDR_TEST             0x1CU   // ECC Test Address
#define FLASH_O_FECC_TEST              0x1EU   // ECC Test Address
#define FLASH_O_FECC_CTRL              0x20U   // ECC Control
#define FLASH_O_FOUTH_TEST             0x22U   // Test Data Out High
#define FLASH_O_FOUTL_TEST             0x24U   // Test Data Out Low
#define FLASH_O_FECC_STATUS            0x26U   // ECC Status


//*************************************************************************************************
//
// The following are defines for the bit fields in the FRDCNTL register
//
//*************************************************************************************************
#define FLASH_FRDCNTL_RWAIT_S   8U
#define FLASH_FRDCNTL_RWAIT_M   0xF00U   // Random Read Waitstate

//*************************************************************************************************
//
// The following are defines for the bit fields in the FBAC register
//
//*************************************************************************************************
#define FLASH_FBAC_BAGP_S   8U
#define FLASH_FBAC_BAGP_M   0xFF00U   // Bank Active Grace Period

//*************************************************************************************************
//
// The following are defines for the bit fields in the FBFALLBACK register
//
//*************************************************************************************************
#define FLASH_FBFALLBACK_BNKPWR0_S   0U
#define FLASH_FBFALLBACK_BNKPWR0_M   0x3U   // Bank Power Mode

//*************************************************************************************************
//
// The following are defines for the bit fields in the FBPRDY register
//
//*************************************************************************************************
#define FLASH_FBPRDY_BANKRDY   0x1U      // Flash Bank Active Power State
#define FLASH_FBPRDY_PUMPRDY   0x8000U   // Flash Pump Active Power Mode

//*************************************************************************************************
//
// The following are defines for the bit fields in the FPAC1 register
//
//*************************************************************************************************
#define FLASH_FPAC1_PMPPWR     0x1U         // Charge Pump Fallback Power Mode
#define FLASH_FPAC1_PSLEEP_S   16U
#define FLASH_FPAC1_PSLEEP_M   0xFFF0000U   // Pump Sleep Down Count

//*************************************************************************************************
//
// The following are defines for the bit fields in the FMSTAT register
//
//*************************************************************************************************
#define FLASH_FMSTAT_PSUSP      0x2U      //  Program Suspend.
#define FLASH_FMSTAT_ESUSP      0x4U      //  Erase Suspend.
#define FLASH_FMSTAT_VOLTSTAT   0x8U      // Core Voltage Status.
#define FLASH_FMSTAT_CSTAT      0x10U     // Command Status.
#define FLASH_FMSTAT_INVDAT     0x20U     // Invalid Data.
#define FLASH_FMSTAT_PGM        0x40U     //  Program Active.
#define FLASH_FMSTAT_ERS        0x80U     //  Erase Active.
#define FLASH_FMSTAT_BUSY       0x100U    // Busy Bit.
#define FLASH_FMSTAT_EV         0x400U    // Erase verify
#define FLASH_FMSTAT_PGV        0x1000U   // Program verify

//*************************************************************************************************
//
// The following are defines for the bit fields in the FRD_INTF_CTRL register
//
//*************************************************************************************************
#define FLASH_FRD_INTF_CTRL_PREFETCH_EN     0x1U   // Prefetch Enable
#define FLASH_FRD_INTF_CTRL_DATA_CACHE_EN   0x2U   // Data Cache Enable


//*************************************************************************************************
//
// The following are defines for the bit fields in the ECC_ENABLE register
//
//*************************************************************************************************
#define FLASH_ECC_ENABLE_ENABLE_S   0U
#define FLASH_ECC_ENABLE_ENABLE_M   0xFU   // Enable ECC

//*************************************************************************************************
//
// The following are defines for the bit fields in the ERR_STATUS register
//
//*************************************************************************************************
#define FLASH_ERR_STATUS_FAIL_0_L    0x1U       // Lower 64bits Single Bit Error Corrected Value 0
#define FLASH_ERR_STATUS_FAIL_1_L    0x2U       // Lower 64bits Single Bit Error Corrected Value 1
#define FLASH_ERR_STATUS_UNC_ERR_L   0x4U       // Lower 64 bits Uncorrectable error occurred
#define FLASH_ERR_STATUS_FAIL_0_H    0x10000U   // Upper 64bits Single Bit Error Corrected Value 0
#define FLASH_ERR_STATUS_FAIL_1_H    0x20000U   // Upper 64bits Single Bit Error Corrected Value 1
#define FLASH_ERR_STATUS_UNC_ERR_H   0x40000U   // Upper 64 bits Uncorrectable error occurred

//*************************************************************************************************
//
// The following are defines for the bit fields in the ERR_POS register
//
//*************************************************************************************************
#define FLASH_ERR_POS_ERR_POS_L_S   0U
#define FLASH_ERR_POS_ERR_POS_L_M   0x3FU        // Bit Position of Single bit Error in lower 64
                                                 // bits
#define FLASH_ERR_POS_ERR_TYPE_L    0x100U       // Error Type in lower 64 bits
#define FLASH_ERR_POS_ERR_POS_H_S   16U
#define FLASH_ERR_POS_ERR_POS_H_M   0x3F0000U    // Bit Position of Single bit Error in upper 64
                                                 // bits
#define FLASH_ERR_POS_ERR_TYPE_H    0x1000000U   // Error Type in upper 64 bits

//*************************************************************************************************
//
// The following are defines for the bit fields in the ERR_STATUS_CLR register
//
//*************************************************************************************************
#define FLASH_ERR_STATUS_CLR_FAIL_0_L_CLR    0x1U       // Lower 64bits Single Bit Error Corrected
                                                        // Value 0 Clear
#define FLASH_ERR_STATUS_CLR_FAIL_1_L_CLR    0x2U       // Lower 64bits Single Bit Error Corrected
                                                        // Value 1 Clear
#define FLASH_ERR_STATUS_CLR_UNC_ERR_L_CLR   0x4U       // Lower 64 bits Uncorrectable error
                                                        // occurred Clear
#define FLASH_ERR_STATUS_CLR_FAIL_0_H_CLR    0x10000U   // Upper 64bits Single Bit Error Corrected
                                                        // Value 0 Clear
#define FLASH_ERR_STATUS_CLR_FAIL_1_H_CLR    0x20000U   // Upper 64bits Single Bit Error Corrected
                                                        // Value 1 Clear
#define FLASH_ERR_STATUS_CLR_UNC_ERR_H_CLR   0x40000U   // Upper 64 bits Uncorrectable error
                                                        // occurred Clear

//*************************************************************************************************
//
// The following are defines for the bit fields in the ERR_CNT register
//
//*************************************************************************************************
#define FLASH_ERR_CNT_ERR_CNT_S   0U
#define FLASH_ERR_CNT_ERR_CNT_M   0xFFFFU   // Error counter

//*************************************************************************************************
//
// The following are defines for the bit fields in the ERR_THRESHOLD register
//
//*************************************************************************************************
#define FLASH_ERR_THRESHOLD_ERR_THRESHOLD_S   0U
#define FLASH_ERR_THRESHOLD_ERR_THRESHOLD_M   0xFFFFU   // Error Threshold

//*************************************************************************************************
//
// The following are defines for the bit fields in the ERR_INTFLG register
//
//*************************************************************************************************
#define FLASH_ERR_INTFLG_SINGLE_ERR_INTFLG   0x1U   // Single Error Interrupt Flag
#define FLASH_ERR_INTFLG_UNC_ERR_INTFLG      0x2U   // Uncorrectable Interrupt Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the ERR_INTCLR register
//
//*************************************************************************************************
#define FLASH_ERR_INTCLR_SINGLE_ERR_INTCLR   0x1U   // Single Error Interrupt Flag Clear
#define FLASH_ERR_INTCLR_UNC_ERR_INTCLR      0x2U   // Uncorrectable Interrupt Flag Clear

//*************************************************************************************************
//
// The following are defines for the bit fields in the FADDR_TEST register
//
//*************************************************************************************************
#define FLASH_FADDR_TEST_ADDRL_S   3U
#define FLASH_FADDR_TEST_ADDRL_M   0xFFF8U     // ECC Address Low
#define FLASH_FADDR_TEST_ADDRH_S   16U
#define FLASH_FADDR_TEST_ADDRH_M   0x3F0000U   // ECC Address High

//*************************************************************************************************
//
// The following are defines for the bit fields in the FECC_TEST register
//
//*************************************************************************************************
#define FLASH_FECC_TEST_ECC_S   0U
#define FLASH_FECC_TEST_ECC_M   0xFFU   // ECC Control Bits

//*************************************************************************************************
//
// The following are defines for the bit fields in the FECC_CTRL register
//
//*************************************************************************************************
#define FLASH_FECC_CTRL_ECC_TEST_EN   0x1U   // Enable ECC Test Logic
#define FLASH_FECC_CTRL_ECC_SELECT    0x2U   // ECC Bit Select
#define FLASH_FECC_CTRL_DO_ECC_CALC   0x4U   // Enable ECC Calculation

//*************************************************************************************************
//
// The following are defines for the bit fields in the FECC_STATUS register
//
//*************************************************************************************************
#define FLASH_FECC_STATUS_SINGLE_ERR       0x1U     // Test Result is Single Bit Error
#define FLASH_FECC_STATUS_UNC_ERR          0x2U     // Test Result is Uncorrectable Error
#define FLASH_FECC_STATUS_DATA_ERR_POS_S   2U
#define FLASH_FECC_STATUS_DATA_ERR_POS_M   0xFCU    // Holds Bit Position of Error
#define FLASH_FECC_STATUS_ERR_TYPE         0x100U   // Holds Bit Position of 8 Check Bits of Error



#endif
