//###########################################################################
//
// FILE:    hw_bgcrc.h
//
// TITLE:   Definitions for the BGCRC registers.
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

#ifndef HW_BGCRC_H
#define HW_BGCRC_H

//*************************************************************************************************
//
// The following are defines for the BGCRC register offsets
//
//*************************************************************************************************
#define BGCRC_O_EN           0x0U    // BGCRC Enable
#define BGCRC_O_CTRL1        0x2U    // BGCRC Control register 1
#define BGCRC_O_CTRL2        0x4U    // BGCRC Control register 2
#define BGCRC_O_START_ADDR   0x6U    // Start address for the BGCRC check
#define BGCRC_O_SEED         0x8U    // Seed for CRC calculation
#define BGCRC_O_GOLDEN       0xEU    // Golden CRC to be compared against
#define BGCRC_O_RESULT       0x10U   // CRC calculated
#define BGCRC_O_CURR_ADDR    0x12U   // Current address regsiter
#define BGCRC_O_WD_CFG       0x1CU   // BGCRC windowed watchdog configuration
#define BGCRC_O_WD_MIN       0x1EU   // BGCRC windowed watchdog min value
#define BGCRC_O_WD_MAX       0x20U   // BGCRC windowed watchdog max value
#define BGCRC_O_WD_CNT       0x22U   // BGCRC windowed watchdog count
#define BGCRC_O_NMIFLG       0x2AU   // BGCRC NMI flag register
#define BGCRC_O_NMICLR       0x2CU   // BGCRC NMI flag clear register
#define BGCRC_O_NMIFRC       0x2EU   // BGCRC NMI flag force register
#define BGCRC_O_INTEN        0x34U   // Interrupt enable
#define BGCRC_O_INTFLG       0x36U   // Interrupt flag
#define BGCRC_O_INTCLR       0x38U   // Interrupt flag clear
#define BGCRC_O_INTFRC       0x3AU   // Interrupt flag force
#define BGCRC_O_LOCK         0x3CU   // BGCRC register map lockconfiguration
#define BGCRC_O_COMMIT       0x3EU   // BGCRC register map commit configuration


//*************************************************************************************************
//
// The following are defines for the bit fields in the BGCRC_EN register
//
//*************************************************************************************************
#define BGCRC_EN_START_S   0U
#define BGCRC_EN_START_M   0xFU          // Start Bit used to  Kick-off CRC calculations
#define BGCRC_EN_RUN_STS   0x80000000U   // CRC module activity monitor

//*************************************************************************************************
//
// The following are defines for the bit fields in the BGCRC_CTRL1 register
//
//*************************************************************************************************
#define BGCRC_CTRL1_FREE_SOFT   0x10U      // emulation control bit
#define BGCRC_CTRL1_NMIDIS_S    16U
#define BGCRC_CTRL1_NMIDIS_M    0xF0000U   // NMI disable configuration

//*************************************************************************************************
//
// The following are defines for the bit fields in the BGCRC_CTRL2 register
//
//*************************************************************************************************
#define BGCRC_CTRL2_BLOCK_SIZE_S   0U
#define BGCRC_CTRL2_BLOCK_SIZE_M   0x3FFU     // block size for memory check
#define BGCRC_CTRL2_TEST_HALT_S    12U
#define BGCRC_CTRL2_TEST_HALT_M    0xF000U    // TEST_HALT configuration
#define BGCRC_CTRL2_SCRUB_MODE_S   16U
#define BGCRC_CTRL2_SCRUB_MODE_M   0xF0000U   // Scrub mode configuration

//*************************************************************************************************
//
// The following are defines for the bit fields in the BGCRC_WD_CFG register
//
//*************************************************************************************************
#define BGCRC_WD_CFG_WDDIS_S   0U
#define BGCRC_WD_CFG_WDDIS_M   0xFU   // CRC Watchdog disable

//*************************************************************************************************
//
// The following are defines for the bit fields in the BGCRC_NMIFLG register
//
//*************************************************************************************************
#define BGCRC_NMIFLG_CRC_FAIL            0x4U    // CRC computation failed
#define BGCRC_NMIFLG_UNCORRECTABLE_ERR   0x8U    // Uncorrectable error obtained during memory data
                                                 // read.
#define BGCRC_NMIFLG_CORRECTABLE_ERR     0x10U   // Correctable ECC error obtained during memory
                                                 // data read.
#define BGCRC_NMIFLG_WD_UNDERFLOW        0x20U   // CRC/scrubbing completed before BGCRC_WD_MIN
#define BGCRC_NMIFLG_WD_OVERFLOW         0x40U   // CRC/scrubbing did not complete within
                                                 // BGCRC_WD_MAX

//*************************************************************************************************
//
// The following are defines for the bit fields in the BGCRC_NMICLR register
//
//*************************************************************************************************
#define BGCRC_NMICLR_CRC_FAIL            0x4U    // CRC_FAIL NMI flag clear
#define BGCRC_NMICLR_UNCORRECTABLE_ERR   0x8U    // UNCORRECTABLE_ERR NMI flag clear
#define BGCRC_NMICLR_CORRECTABLE_ERR     0x10U   // CORRECTABLE_ERR NMI flag clear
#define BGCRC_NMICLR_WD_UNDERFLOW        0x20U   // WD_UNDERFLOW NMI flag clear
#define BGCRC_NMICLR_WD_OVERFLOW         0x40U   // WD_OVERFLOW NMI flag clear

//*************************************************************************************************
//
// The following are defines for the bit fields in the BGCRC_NMIFRC register
//
//*************************************************************************************************
#define BGCRC_NMIFRC_CRC_FAIL            0x4U    // CRC_FAIL NMI force
#define BGCRC_NMIFRC_UNCORRECTABLE_ERR   0x8U    // UNCORRECTABLE_ERR NMI force
#define BGCRC_NMIFRC_CORRECTABLE_ERR     0x10U   // CORRECTABLE_ERR NMI force
#define BGCRC_NMIFRC_WD_UNDERFLOW        0x20U   // WD_UNDERFLOW NMI force
#define BGCRC_NMIFRC_WD_OVERFLOW         0x40U   // WD_OVERFLOW NMI force

//*************************************************************************************************
//
// The following are defines for the bit fields in the BGCRC_INTEN register
//
//*************************************************************************************************
#define BGCRC_INTEN_TEST_DONE           0x2U    // TEST_DONE interrupt enable register
#define BGCRC_INTEN_CRC_FAIL            0x4U    // CRC_FAIL interrupt enable register
#define BGCRC_INTEN_UNCORRECTABLE_ERR   0x8U    // UNCORRECTABLE_ERR interrupt enable register
#define BGCRC_INTEN_CORRECTABLE_ERR     0x10U   // CORRECTABLE_ERR interrupt enable register
#define BGCRC_INTEN_WD_UNDERFLOW        0x20U   // WD_UNDERFLOW interrupt enable register
#define BGCRC_INTEN_WD_OVERFLOW         0x40U   // WD_OVERFLOW interrupt enable register

//*************************************************************************************************
//
// The following are defines for the bit fields in the BGCRC_INTFLG register
//
//*************************************************************************************************
#define BGCRC_INTFLG_INT                 0x1U    // Global Interrupt status flag
#define BGCRC_INTFLG_TEST_DONE           0x2U    // TEST_DONE Interrupt status flag
#define BGCRC_INTFLG_CRC_FAIL            0x4U    // CRC computation failed
#define BGCRC_INTFLG_UNCORRECTABLE_ERR   0x8U    // Uncorrectable error obtained during memory data
                                                 // read.
#define BGCRC_INTFLG_CORRECTABLE_ERR     0x10U   // Correctable ECC error obtained during memory
                                                 // data read.
#define BGCRC_INTFLG_WD_UNDERFLOW        0x20U   // CRC/scrubbing completed before BGCRC_WD_MIN
#define BGCRC_INTFLG_WD_OVERFLOW         0x40U   // CRC/scrubbing did not complete within
                                                 // BGCRC_WD_MAX

//*************************************************************************************************
//
// The following are defines for the bit fields in the BGCRC_INTCLR register
//
//*************************************************************************************************
#define BGCRC_INTCLR_INT                 0x1U    // Global Interrupt clear
#define BGCRC_INTCLR_TEST_DONE           0x2U    // TEST_DONE Interrupt clear
#define BGCRC_INTCLR_CRC_FAIL            0x4U    // CRC_FAIL interrupt clear
#define BGCRC_INTCLR_UNCORRECTABLE_ERR   0x8U    // UNCORRECTABLE_ERR interrupt clear
#define BGCRC_INTCLR_CORRECTABLE_ERR     0x10U   // CORRECTABLE_ERR interrupt clear
#define BGCRC_INTCLR_WD_UNDERFLOW        0x20U   // WD_UNDERFLOW interrupt clear
#define BGCRC_INTCLR_WD_OVERFLOW         0x40U   // WD_OVERFLOW interrupt clear

//*************************************************************************************************
//
// The following are defines for the bit fields in the BGCRC_INTFRC register
//
//*************************************************************************************************
#define BGCRC_INTFRC_TEST_DONE           0x2U    // TEST_DONE Interrupt force
#define BGCRC_INTFRC_CRC_FAIL            0x4U    // CRC_FAIL interrupt force
#define BGCRC_INTFRC_UNCORRECTABLE_ERR   0x8U    // UNCORRECTABLE_ERR interrupt force
#define BGCRC_INTFRC_CORRECTABLE_ERR     0x10U   // CORRECTABLE_ERR interrupt force
#define BGCRC_INTFRC_WD_UNDERFLOW        0x20U   // WD_UNDERFLOW interrupt force
#define BGCRC_INTFRC_WD_OVERFLOW         0x40U   // WD_OVERFLOW interrupt force

//*************************************************************************************************
//
// The following are defines for the bit fields in the BGCRC_LOCK register
//
//*************************************************************************************************
#define BGCRC_LOCK_BGCRC_EN           0x1U          // Register lock configuration
#define BGCRC_LOCK_BGCRC_CTRL1        0x2U          // Register lock configuration
#define BGCRC_LOCK_BGCRC_CTRL2        0x4U          // Register lock configuration
#define BGCRC_LOCK_BGCRC_START_ADDR   0x8U          // Register lock configuration
#define BGCRC_LOCK_BGCRC_SEED         0x10U         // Register lock configuration
#define BGCRC_LOCK_BGCRC_GOLDEN       0x80U         // Register lock configuration
#define BGCRC_LOCK_BGCRC_WD_CFG       0x4000U       // Register lock configuration
#define BGCRC_LOCK_BGCRC_WD_MIN       0x8000U       // Register lock configuration
#define BGCRC_LOCK_BGCRC_WD_MAX       0x10000U      // Register lock configuration
#define BGCRC_LOCK_BGCRC_NMIFRC       0x800000U     // Register lock configuration
#define BGCRC_LOCK_BGCRC_INTEN        0x4000000U    // Register lock configuration
#define BGCRC_LOCK_BGCRC_INTFRC       0x20000000U   // Register lock configuration

//*************************************************************************************************
//
// The following are defines for the bit fields in the BGCRC_COMMIT register
//
//*************************************************************************************************
#define BGCRC_COMMIT_BGCRC_EN           0x1U          // Register lock committed
#define BGCRC_COMMIT_BGCRC_CTRL1        0x2U          // Register lock committed
#define BGCRC_COMMIT_BGCRC_CTRL2        0x4U          // Register lock committed
#define BGCRC_COMMIT_BGCRC_START_ADDR   0x8U          // Register lock committed
#define BGCRC_COMMIT_BGCRC_SEED         0x10U         // Register lock committed
#define BGCRC_COMMIT_BGCRC_GOLDEN       0x80U         // Register lock committed
#define BGCRC_COMMIT_BGCRC_WD_CFG       0x4000U       // Register lock committed
#define BGCRC_COMMIT_BGCRC_WD_MIN       0x8000U       // Register lock committed
#define BGCRC_COMMIT_BGCRC_WD_MAX       0x10000U      // Register lock committed
#define BGCRC_COMMIT_BGCRC_NMIFRC       0x800000U     // Register lock committed
#define BGCRC_COMMIT_BGCRC_INTEN        0x4000000U    // Register lock committed
#define BGCRC_COMMIT_BGCRC_INTFRC       0x20000000U   // Register lock committed



#endif
