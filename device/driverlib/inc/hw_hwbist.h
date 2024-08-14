//###########################################################################
//
// FILE:    hw_hwbist.h
//
// TITLE:   Definitions for the HWBIST registers.
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

#ifndef HW_HWBIST_H
#define HW_HWBIST_H

//*************************************************************************************************
//
// The following are defines for the HWBIST register offsets
//
//*************************************************************************************************
#define HWBIST_O_CSTCGCR1     0x4U    // STC Global Control Register1
#define HWBIST_O_CSTCGCR3     0xCU    // STC Global Control Register3
#define HWBIST_O_CSTCGCR4     0x10U   // STC Global Control Register4
#define HWBIST_O_CSTCGCR5     0x14U   // STC Global Control Register5
#define HWBIST_O_CSTCGCR6     0x18U   // STC Global Control Register6
#define HWBIST_O_CSTCGCR7     0x1CU   // STC Global Control Register7
#define HWBIST_O_CSTCGCR8     0x20U   // STC Global Control Register8
#define HWBIST_O_CSTCPCNT     0x24U   // STC Pattern Count Register
#define HWBIST_O_CSTCCONFIG   0x28U   // STC Registers Configuration Status
#define HWBIST_O_CSTCSADDR    0x2CU   // STC ROM Start Address
#define HWBIST_O_CSTCTEST     0x30U   // C28 HW BIST Test Register
#define HWBIST_O_CSTCRET      0x34U   // C28 Return PC Address
#define HWBIST_O_CSTCCRD      0x38U   // C28 Context Restore Done Register
#define HWBIST_O_CSTCGSTAT    0x40U   // STC Global Status Register
#define HWBIST_O_CSTCCPCR     0x48U   // STC Current Pattern Count Register
#define HWBIST_O_CSTCCADDR    0x4CU   // STC Current ROM Address Register
#define HWBIST_O_CSTCMISR0    0x50U   // MISR Result Register 0
#define HWBIST_O_CSTCMISR1    0x54U   // MISR Result Register 1
#define HWBIST_O_CSTCMISR2    0x58U   // MISR Result Register 2
#define HWBIST_O_CSTCMISR3    0x5CU   // MISR Result Register 3
#define HWBIST_O_CSTCMISR4    0x60U   // MISR Result Register 4
#define HWBIST_O_CSTCMISR5    0x64U   // MISR Result Register 5
#define HWBIST_O_CSTCMISR6    0x68U   // MISR Result Register 6
#define HWBIST_O_CSTCMISR7    0x6CU   // MISR Result Register 7
#define HWBIST_O_CSTCMISR8    0x70U   // MISR Result Register 8
#define HWBIST_O_CSTCMISR9    0x74U   // MISR Result Register 9
#define HWBIST_O_CSTCMISR10   0x78U   // MISR Result Register 10
#define HWBIST_O_CSTCMISR11   0x7CU   // MISR Result Register 11
#define HWBIST_O_CSTCMISR12   0x80U   // MISR Result Register 12
#define HWBIST_O_CSTCMISR13   0x84U   // MISR Result Register 13
#define HWBIST_O_CSTCMISR14   0x88U   // MISR Result Register 14
#define HWBIST_O_CSTCMISR15   0x8CU   // MISR Result Register 15


//*************************************************************************************************
//
// The following are defines for the bit fields in the CSTCGCR3 register
//
//*************************************************************************************************
#define HWBIST_CSTCGCR3_ILS_S   0U
#define HWBIST_CSTCGCR3_ILS_M   0xFU   // Interrupt Logging Start
//*************************************************************************************************
//
// The following are defines for the bit fields in the CSTCGCR4 register
//
//*************************************************************************************************
#define HWBIST_CSTCGCR4_BISTGO_S   0U
#define HWBIST_CSTCGCR4_BISTGO_M   0xFU   // BIST Start
//*************************************************************************************************
//
// The following are defines for the bit fields in the CSTCGCR5 register
//
//*************************************************************************************************
#define HWBIST_CSTCGCR5_RESTART_S    0U
#define HWBIST_CSTCGCR5_RESTART_M    0xFU          // Restart Enable
#define HWBIST_CSTCGCR5_SOFT_RESET   0x80000000U   // Soft reset to BIST controller
//*************************************************************************************************
//
// The following are defines for the bit fields in the CSTCGCR6 register
//
//*************************************************************************************************
#define HWBIST_CSTCGCR6_COV_S   0U
#define HWBIST_CSTCGCR6_COV_M   0x3U   // COVERAGE
//*************************************************************************************************
//
// The following are defines for the bit fields in the CSTCGCR7 register
//
//*************************************************************************************************
#define HWBIST_CSTCGCR7_MCL_S   0U
#define HWBIST_CSTCGCR7_MCL_M   0xFFU      // MAX CHAIN LENGTH
#define HWBIST_CSTCGCR7_DC_S    8U
#define HWBIST_CSTCGCR7_DC_M    0xF00U     // DEAD CYCLES
#define HWBIST_CSTCGCR7_NP_S    12U
#define HWBIST_CSTCGCR7_NP_M    0xF000U    // NUM  OF PIPELINE STAGES
#define HWBIST_CSTCGCR7_PST_S   16U
#define HWBIST_CSTCGCR7_PST_M   0x30000U   // PATTERN SET TYPE
#define HWBIST_CSTCGCR7_SCD_S   18U
#define HWBIST_CSTCGCR7_SCD_M   0xC0000U   // SHIFT_CLOCK_DIVISION
//*************************************************************************************************
//
// The following are defines for the bit fields in the CSTCGCR8 register
//
//*************************************************************************************************
#define HWBIST_CSTCGCR8_CPC_S   0U
#define HWBIST_CSTCGCR8_CPC_M   0xFFFFU   // COMPARE PATTERN CNT
//*************************************************************************************************
//
// The following are defines for the bit fields in the CSTCPCNT register
//
//*************************************************************************************************
#define HWBIST_CSTCPCNT_PCNT_95_S   0U
#define HWBIST_CSTCPCNT_PCNT_95_M   0xFFFFU       // PATTERNS FOR 95% COVERAGE
#define HWBIST_CSTCPCNT_PCNT_99_S   16U
#define HWBIST_CSTCPCNT_PCNT_99_M   0xFFFF0000U   // PATTERNS FOR 99% COVERAGE
//*************************************************************************************************
//
// The following are defines for the bit fields in the CSTCCONFIG register
//
//*************************************************************************************************
#define HWBIST_CSTCCONFIG_CFGDONE_S   0U
#define HWBIST_CSTCCONFIG_CFGDONE_M   0xFU   // Configuration done
//*************************************************************************************************
//
// The following are defines for the bit fields in the CSTCSADDR register
//
//*************************************************************************************************
#define HWBIST_CSTCSADDR_SAPAT_S    0U
#define HWBIST_CSTCSADDR_SAPAT_M    0xFFFFU       // PATTERN ROM Start Address
#define HWBIST_CSTCSADDR_SAMISR_S   16U
#define HWBIST_CSTCSADDR_SAMISR_M   0xFFFF0000U   // MISR ROM Start Address
//*************************************************************************************************
//
// The following are defines for the bit fields in the CSTCTEST register
//
//*************************************************************************************************
#define HWBIST_CSTCTEST_TEST_TO_S         0U
#define HWBIST_CSTCTEST_TEST_TO_M         0xFU          // Test_ Time_Out
#define HWBIST_CSTCTEST_TEST_CMP_FAIL_S   4U
#define HWBIST_CSTCTEST_TEST_CMP_FAIL_M   0xF0U         // Test MISR compare fail
#define HWBIST_CSTCTEST_TEST_NMI_S        8U
#define HWBIST_CSTCTEST_TEST_NMI_M        0xF00U        // Test_NMI
#define HWBIST_CSTCTEST_TEST_S            12U
#define HWBIST_CSTCTEST_TEST_M            0xFFFFF000U   // TEST Bits
//*************************************************************************************************
//
// The following are defines for the bit fields in the CSTCCRD register
//
//*************************************************************************************************
#define HWBIST_CSTCCRD_RESTORE_DONE_S   0U
#define HWBIST_CSTCCRD_RESTORE_DONE_M   0xFU   // Context Restone Done
//*************************************************************************************************
//
// The following are defines for the bit fields in the CSTCGSTAT register
//
//*************************************************************************************************
#define HWBIST_CSTCGSTAT_BISTDONE    0x1U    // HW BIST Complete
#define HWBIST_CSTCGSTAT_MACRODONE   0x2U    // Macro test slot Complete
#define HWBIST_CSTCGSTAT_NMI         0x4U    // Exit due to NMI
#define HWBIST_CSTCGSTAT_BISTFAIL    0x8U    // HW BIST Failure
#define HWBIST_CSTCGSTAT_INTCMPF     0x10U   // Intermediate Comparison Failure
#define HWBIST_CSTCGSTAT_TOFAIL      0x20U   // Time Out Failure
//*************************************************************************************************
//
// The following are defines for the bit fields in the CSTCCPCR register
//
//*************************************************************************************************
#define HWBIST_CSTCCPCR_PATCNT_S   0U
#define HWBIST_CSTCCPCR_PATCNT_M   0xFFFFU   // Current Pattern Count
//*************************************************************************************************
//
// The following are defines for the bit fields in the CSTCCADDR register
//
//*************************************************************************************************
#define HWBIST_CSTCCADDR_PATADDR_S    0U
#define HWBIST_CSTCCADDR_PATADDR_M    0xFFFFU       // Current Pattern ROM Address
#define HWBIST_CSTCCADDR_MISRADDR_S   16U
#define HWBIST_CSTCCADDR_MISRADDR_M   0xFFFF0000U   // Current MISR ROM Address


#endif
