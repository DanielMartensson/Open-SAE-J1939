//###########################################################################
//
// FILE:    hw_cla.h
//
// TITLE:   Definitions for the CLA registers.
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

#ifndef HW_CLA_H
#define HW_CLA_H

//*************************************************************************************************
//
// The following are defines for the CLA register offsets
//
//*************************************************************************************************
#ifndef __TMS320C28XX_CLA__
#define CLA_O_MVECT1             0x0U    // Task Interrupt Vector
#define CLA_O_MVECT2             0x1U    // Task Interrupt Vector
#define CLA_O_MVECT3             0x2U    // Task Interrupt Vector
#define CLA_O_MVECT4             0x3U    // Task Interrupt Vector
#define CLA_O_MVECT5             0x4U    // Task Interrupt Vector
#define CLA_O_MVECT6             0x5U    // Task Interrupt Vector
#define CLA_O_MVECT7             0x6U    // Task Interrupt Vector
#define CLA_O_MVECT8             0x7U    // Task Interrupt Vector
#define CLA_O_MCTL               0x10U   // Control Register
#define CLA_O_MVECTBGRNDACTIVE   0x1BU   // Active register for MVECTBGRND.
#define CLA_O_SOFTINTEN          0x1CU   // CLA Software Interrupt Enable Register
#define CLA_O_MSTSBGRND          0x1DU   // Status register for the back ground task.
#define CLA_O_MCTLBGRND          0x1EU   // Control register for the back ground task.
#define CLA_O_MVECTBGRND         0x1FU   // Vector for the back ground task.
#define CLA_O_MIFR               0x20U   // Interrupt Flag Register
#define CLA_O_MIOVF              0x21U   // Interrupt Overflow Flag Register
#define CLA_O_MIFRC              0x22U   // Interrupt Force Register
#define CLA_O_MICLR              0x23U   // Interrupt Flag Clear Register
#define CLA_O_MICLROVF           0x24U   // Interrupt Overflow Flag Clear Register
#define CLA_O_MIER               0x25U   // Interrupt Enable Register
#define CLA_O_MIRUN              0x26U   // Interrupt Run Status Register
#define CLA_O_MPC                0x28U   // CLA Program Counter
#define CLA_O_MAR0               0x2AU   // CLA Auxiliary Register 0
#define CLA_O_MAR1               0x2BU   // CLA Auxiliary Register 1
#define CLA_O_MSTF               0x2EU   // CLA Floating-Point Status Register
#define CLA_O_MR0                0x30U   // CLA Floating-Point Result Register 0
#define CLA_O_MR1                0x34U   // CLA Floating-Point Result Register 1
#define CLA_O_MR2                0x38U   // CLA Floating-Point Result Register 2
#define CLA_O_MR3                0x3CU   // CLA Floating-Point Result Register 3
#define CLA_O_MPSACTL            0x42U   // CLA PSA Control Register
#define CLA_O_MPSA1              0x44U   // CLA PSA1 Register
#define CLA_O_MPSA2              0x46U   // CLA PSA2 Register
#endif

#ifdef __TMS320C28XX_CLA__
#define CLA_O_MVECTBGRNDACTIVE   0x80U   // Active register for MVECTBGRND.
#define CLA_O_MPSACTL            0xC0U   // CLA PSA Control Register
#define CLA_O_MPSA1              0xC2U   // CLA PSA1 Register
#define CLA_O_MPSA2              0xC4U   // CLA PSA2 Register
#define CLA_O_SOFTINTEN          0xE0U   // CLA Software Interrupt Enable Register
#define CLA_O_SOFTINTFRC         0xE2U   // CLA Software Interrupt Force Register
#endif


#ifndef __TMS320C28XX_CLA__
//*************************************************************************************************
//
// The following are defines for the bit fields in the MCTL register
//
//*************************************************************************************************
#define CLA_MCTL_HARDRESET   0x1U   // Hard Reset
#define CLA_MCTL_SOFTRESET   0x2U   // Soft Reset
#define CLA_MCTL_IACKE       0x4U   // IACK enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTINTEN register
//
//*************************************************************************************************
#define CLA_SOFTINTEN_TASK1   0x1U    // Configure Software Interrupt or End of Task interrupt.
#define CLA_SOFTINTEN_TASK2   0x2U    // Configure Software Interrupt or End of Task interrupt.
#define CLA_SOFTINTEN_TASK3   0x4U    // Configure Software Interrupt or End of Task interrupt.
#define CLA_SOFTINTEN_TASK4   0x8U    // Configure Software Interrupt or End of Task interrupt.
#define CLA_SOFTINTEN_TASK5   0x10U   // Configure Software Interrupt or End of Task interrupt.
#define CLA_SOFTINTEN_TASK6   0x20U   // Configure Software Interrupt or End of Task interrupt.
#define CLA_SOFTINTEN_TASK7   0x40U   // Configure Software Interrupt or End of Task interrupt.
#define CLA_SOFTINTEN_TASK8   0x80U   // Configure Software Interrupt or End of Task interrupt.

//*************************************************************************************************
//
// The following are defines for the bit fields in the _MSTSBGRND register
//
//*************************************************************************************************
#define CLA_MSTSBGRND_RUN      0x1U   // Background task run status bit.
#define CLA_MSTSBGRND_BGINTM   0x2U   // Indicates whether background task can be interrupted.
#define CLA_MSTSBGRND_BGOVF    0x4U   // background task harware trigger overflow.

//*************************************************************************************************
//
// The following are defines for the bit fields in the _MCTLBGRND register
//
//*************************************************************************************************
#define CLA_MCTLBGRND_BGSTART   0x1U      // Background task start bit
#define CLA_MCTLBGRND_TRIGEN    0x2U      // Background task hardware trigger enable
#define CLA_MCTLBGRND_BGEN      0x8000U   // Enable background task

//*************************************************************************************************
//
// The following are defines for the bit fields in the MIFR register
//
//*************************************************************************************************
#define CLA_MIFR_INT1   0x1U    // Task 1 Interrupt Flag
#define CLA_MIFR_INT2   0x2U    // Task 2 Interrupt Flag
#define CLA_MIFR_INT3   0x4U    // Task 3 Interrupt Flag
#define CLA_MIFR_INT4   0x8U    // Task 4 Interrupt Flag
#define CLA_MIFR_INT5   0x10U   // Task 5 Interrupt Flag
#define CLA_MIFR_INT6   0x20U   // Task 6 Interrupt Flag
#define CLA_MIFR_INT7   0x40U   // Task 7 Interrupt Flag
#define CLA_MIFR_INT8   0x80U   // Task 8 Interrupt Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the MIOVF register
//
//*************************************************************************************************
#define CLA_MIOVF_INT1   0x1U    // Task 1 Interrupt Overflow Flag
#define CLA_MIOVF_INT2   0x2U    // Task 2 Interrupt Overflow Flag
#define CLA_MIOVF_INT3   0x4U    // Task 3 Interrupt Overflow Flag
#define CLA_MIOVF_INT4   0x8U    // Task 4 Interrupt Overflow Flag
#define CLA_MIOVF_INT5   0x10U   // Task 5 Interrupt Overflow Flag
#define CLA_MIOVF_INT6   0x20U   // Task 6 Interrupt Overflow Flag
#define CLA_MIOVF_INT7   0x40U   // Task 7 Interrupt Overflow Flag
#define CLA_MIOVF_INT8   0x80U   // Task 8 Interrupt Overflow Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the MIFRC register
//
//*************************************************************************************************
#define CLA_MIFRC_INT1   0x1U    // Task 1 Interrupt Force
#define CLA_MIFRC_INT2   0x2U    // Task 2 Interrupt Force
#define CLA_MIFRC_INT3   0x4U    // Task 3 Interrupt Force
#define CLA_MIFRC_INT4   0x8U    // Task 4 Interrupt Force
#define CLA_MIFRC_INT5   0x10U   // Task 5 Interrupt Force
#define CLA_MIFRC_INT6   0x20U   // Task 6 Interrupt Force
#define CLA_MIFRC_INT7   0x40U   // Task 7 Interrupt Force
#define CLA_MIFRC_INT8   0x80U   // Task 8 Interrupt Force

//*************************************************************************************************
//
// The following are defines for the bit fields in the MICLR register
//
//*************************************************************************************************
#define CLA_MICLR_INT1   0x1U    // Task 1 Interrupt Flag Clear
#define CLA_MICLR_INT2   0x2U    // Task 2 Interrupt Flag Clear
#define CLA_MICLR_INT3   0x4U    // Task 3 Interrupt Flag Clear
#define CLA_MICLR_INT4   0x8U    // Task 4 Interrupt Flag Clear
#define CLA_MICLR_INT5   0x10U   // Task 5 Interrupt Flag Clear
#define CLA_MICLR_INT6   0x20U   // Task 6 Interrupt Flag Clear
#define CLA_MICLR_INT7   0x40U   // Task 7 Interrupt Flag Clear
#define CLA_MICLR_INT8   0x80U   // Task 8 Interrupt Flag Clear

//*************************************************************************************************
//
// The following are defines for the bit fields in the MICLROVF register
//
//*************************************************************************************************
#define CLA_MICLROVF_INT1   0x1U    // Task 1 Interrupt Overflow Flag Clear
#define CLA_MICLROVF_INT2   0x2U    // Task 2 Interrupt Overflow Flag Clear
#define CLA_MICLROVF_INT3   0x4U    // Task 3 Interrupt Overflow Flag Clear
#define CLA_MICLROVF_INT4   0x8U    // Task 4 Interrupt Overflow Flag Clear
#define CLA_MICLROVF_INT5   0x10U   // Task 5 Interrupt Overflow Flag Clear
#define CLA_MICLROVF_INT6   0x20U   // Task 6 Interrupt Overflow Flag Clear
#define CLA_MICLROVF_INT7   0x40U   // Task 7 Interrupt Overflow Flag Clear
#define CLA_MICLROVF_INT8   0x80U   // Task 8 Interrupt Overflow Flag Clear

//*************************************************************************************************
//
// The following are defines for the bit fields in the MIER register
//
//*************************************************************************************************
#define CLA_MIER_INT1   0x1U    // Task 1 Interrupt Enable
#define CLA_MIER_INT2   0x2U    // Task 2 Interrupt Enable
#define CLA_MIER_INT3   0x4U    // Task 3 Interrupt Enable
#define CLA_MIER_INT4   0x8U    // Task 4 Interrupt Enable
#define CLA_MIER_INT5   0x10U   // Task 5 Interrupt Enable
#define CLA_MIER_INT6   0x20U   // Task 6 Interrupt Enable
#define CLA_MIER_INT7   0x40U   // Task 7 Interrupt Enable
#define CLA_MIER_INT8   0x80U   // Task 8 Interrupt Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the MIRUN register
//
//*************************************************************************************************
#define CLA_MIRUN_INT1   0x1U    // Task 1 Run Status
#define CLA_MIRUN_INT2   0x2U    // Task 2 Run Status
#define CLA_MIRUN_INT3   0x4U    // Task 3 Run Status
#define CLA_MIRUN_INT4   0x8U    // Task 4 Run Status
#define CLA_MIRUN_INT5   0x10U   // Task 5 Run Status
#define CLA_MIRUN_INT6   0x20U   // Task 6 Run Status
#define CLA_MIRUN_INT7   0x40U   // Task 7 Run Status
#define CLA_MIRUN_INT8   0x80U   // Task 8 Run Status

//*************************************************************************************************
//
// The following are defines for the bit fields in the _MSTF register
//
//*************************************************************************************************
#define CLA_MSTF_LVF       0x1U         // Latched Overflow Flag
#define CLA_MSTF_LUF       0x2U         // Latched Underflow Flag
#define CLA_MSTF_NF        0x4U         // Negative Float Flag
#define CLA_MSTF_ZF        0x8U         // Zero Float Flag
#define CLA_MSTF_TF        0x40U        // Test Flag
#define CLA_MSTF_RNDF32    0x200U       // Round 32-bit Floating-Point Mode
#define CLA_MSTF_MEALLOW   0x800U       // MEALLOW Status
#define CLA_MSTF_RPC_S     12U
#define CLA_MSTF_RPC_M     0xFFFF000U   // Return PC

//*************************************************************************************************
//
// The following are defines for the bit fields in the _MPSACTL register
//
//*************************************************************************************************
#define CLA_MPSACTL_MPABSTART    0x1U    // Start logging PAB onto PSA1
#define CLA_MPSACTL_MPABCYC      0x2U    // PAB logging into PSA1 is on every cycle or when PAB
                                         // changes.
#define CLA_MPSACTL_MDWDBSTART   0x4U    // Start logging DWDB onto PSA2
#define CLA_MPSACTL_MDWDBCYC     0x8U    // DWDB logging into PSA2 is on every cycle.
#define CLA_MPSACTL_MPSA1CLEAR   0x10U   // PSA1 clear
#define CLA_MPSACTL_MPSA2CLEAR   0x20U   // PSA2 Clear
#define CLA_MPSACTL_MPSA2CFG_S   6U
#define CLA_MPSACTL_MPSA2CFG_M   0xC0U   // PSA2 Polynomial Configuration

#endif

#ifdef __TMS320C28XX_CLA__
//*************************************************************************************************
//
// The following are defines for the bit fields in the _MPSACTL register
//
//*************************************************************************************************
#define CLA_MPSACTL_MPABSTART    0x1U    // Start logging PAB onto PSA1
#define CLA_MPSACTL_MPABCYC      0x2U    // PAB logging into PSA1 is on every cycle or when PAB
                                         // changes.
#define CLA_MPSACTL_MDWDBSTART   0x4U    // Start logging DWDB onto PSA2
#define CLA_MPSACTL_MDWDBCYC     0x8U    // DWDB logging into PSA2 is on every cycle.
#define CLA_MPSACTL_MPSA1CLEAR   0x10U   // PSA1 clear
#define CLA_MPSACTL_MPSA2CLEAR   0x20U   // PSA2 Clear
#define CLA_MPSACTL_MPSA2CFG_S   6U
#define CLA_MPSACTL_MPSA2CFG_M   0xC0U   // PSA2 Polynomial Configuration

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTINTEN register
//
//*************************************************************************************************
#define CLA_SOFTINTEN_TASK1   0x1U    // Configure Software Interrupt or End of Task interrupt.
#define CLA_SOFTINTEN_TASK2   0x2U    // Configure Software Interrupt or End of Task interrupt.
#define CLA_SOFTINTEN_TASK3   0x4U    // Configure Software Interrupt or End of Task interrupt.
#define CLA_SOFTINTEN_TASK4   0x8U    // Configure Software Interrupt or End of Task interrupt.
#define CLA_SOFTINTEN_TASK5   0x10U   // Configure Software Interrupt or End of Task interrupt.
#define CLA_SOFTINTEN_TASK6   0x20U   // Configure Software Interrupt or End of Task interrupt.
#define CLA_SOFTINTEN_TASK7   0x40U   // Configure Software Interrupt or End of Task interrupt.
#define CLA_SOFTINTEN_TASK8   0x80U   // Configure Software Interrupt or End of Task interrupt.

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTINTFRC register
//
//*************************************************************************************************
#define CLA_SOFTINTFRC_TASK1   0x1U    // Force CLA software interrupt for the corresponding task.
#define CLA_SOFTINTFRC_TASK2   0x2U    // Force CLA software interrupt for the corresponding task.
#define CLA_SOFTINTFRC_TASK3   0x4U    // Force CLA software interrupt for the corresponding task.
#define CLA_SOFTINTFRC_TASK4   0x8U    // Force CLA software interrupt for the corresponding task.
#define CLA_SOFTINTFRC_TASK5   0x10U   // Force CLA software interrupt for the corresponding task.
#define CLA_SOFTINTFRC_TASK6   0x20U   // Force CLA software interrupt for the corresponding task.
#define CLA_SOFTINTFRC_TASK7   0x40U   // Force CLA software interrupt for the corresponding task.
#define CLA_SOFTINTFRC_TASK8   0x80U   // Force CLA software interrupt for the corresponding task.

#endif


#endif
