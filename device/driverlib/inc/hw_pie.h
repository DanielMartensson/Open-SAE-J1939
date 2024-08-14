//###########################################################################
//
// FILE:    hw_pie.h
//
// TITLE:   Definitions for the PIE registers.
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

#ifndef HW_PIE_H
#define HW_PIE_H

//*************************************************************************************************
//
// The following are defines for the PIE register offsets
//
//*************************************************************************************************
#define PIE_O_CTRL    0x0U    // ePIE Control Register
#define PIE_O_ACK     0x1U    // Interrupt Acknowledge Register
#define PIE_O_IER1    0x2U    // Interrupt Group 1 Enable Register
#define PIE_O_IFR1    0x3U    // Interrupt Group 1 Flag Register
#define PIE_O_IER2    0x4U    // Interrupt Group 2 Enable Register
#define PIE_O_IFR2    0x5U    // Interrupt Group 2 Flag Register
#define PIE_O_IER3    0x6U    // Interrupt Group 3 Enable Register
#define PIE_O_IFR3    0x7U    // Interrupt Group 3 Flag Register
#define PIE_O_IER4    0x8U    // Interrupt Group 4 Enable Register
#define PIE_O_IFR4    0x9U    // Interrupt Group 4 Flag Register
#define PIE_O_IER5    0xAU    // Interrupt Group 5 Enable Register
#define PIE_O_IFR5    0xBU    // Interrupt Group 5 Flag Register
#define PIE_O_IER6    0xCU    // Interrupt Group 6 Enable Register
#define PIE_O_IFR6    0xDU    // Interrupt Group 6 Flag Register
#define PIE_O_IER7    0xEU    // Interrupt Group 7 Enable Register
#define PIE_O_IFR7    0xFU    // Interrupt Group 7 Flag Register
#define PIE_O_IER8    0x10U   // Interrupt Group 8 Enable Register
#define PIE_O_IFR8    0x11U   // Interrupt Group 8 Flag Register
#define PIE_O_IER9    0x12U   // Interrupt Group 9 Enable Register
#define PIE_O_IFR9    0x13U   // Interrupt Group 9 Flag Register
#define PIE_O_IER10   0x14U   // Interrupt Group 10 Enable Register
#define PIE_O_IFR10   0x15U   // Interrupt Group 10 Flag Register
#define PIE_O_IER11   0x16U   // Interrupt Group 11 Enable Register
#define PIE_O_IFR11   0x17U   // Interrupt Group 11 Flag Register
#define PIE_O_IER12   0x18U   // Interrupt Group 12 Enable Register
#define PIE_O_IFR12   0x19U   // Interrupt Group 12 Flag Register


//*************************************************************************************************
//
// The following are defines for the bit fields in the PIECTRL register
//
//*************************************************************************************************
#define PIE_CTRL_ENPIE       0x1U      // PIE Enable
#define PIE_CTRL_PIEVECT_S   1U
#define PIE_CTRL_PIEVECT_M   0xFFFEU   // PIE Vector Address

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEACK register
//
//*************************************************************************************************
#define PIE_ACK_ACK1    0x1U     // Acknowledge PIE Interrupt Group 1
#define PIE_ACK_ACK2    0x2U     // Acknowledge PIE Interrupt Group 2
#define PIE_ACK_ACK3    0x4U     // Acknowledge PIE Interrupt Group 3
#define PIE_ACK_ACK4    0x8U     // Acknowledge PIE Interrupt Group 4
#define PIE_ACK_ACK5    0x10U    // Acknowledge PIE Interrupt Group 5
#define PIE_ACK_ACK6    0x20U    // Acknowledge PIE Interrupt Group 6
#define PIE_ACK_ACK7    0x40U    // Acknowledge PIE Interrupt Group 7
#define PIE_ACK_ACK8    0x80U    // Acknowledge PIE Interrupt Group 8
#define PIE_ACK_ACK9    0x100U   // Acknowledge PIE Interrupt Group 9
#define PIE_ACK_ACK10   0x200U   // Acknowledge PIE Interrupt Group 10
#define PIE_ACK_ACK11   0x400U   // Acknowledge PIE Interrupt Group 11
#define PIE_ACK_ACK12   0x800U   // Acknowledge PIE Interrupt Group 12

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIER1 register
//
//*************************************************************************************************
#define PIE_IER1_INTX1    0x1U      // Enable for Interrupt 1.1
#define PIE_IER1_INTX2    0x2U      // Enable for Interrupt 1.2
#define PIE_IER1_INTX3    0x4U      // Enable for Interrupt 1.3
#define PIE_IER1_INTX4    0x8U      // Enable for Interrupt 1.4
#define PIE_IER1_INTX5    0x10U     // Enable for Interrupt 1.5
#define PIE_IER1_INTX6    0x20U     // Enable for Interrupt 1.6
#define PIE_IER1_INTX7    0x40U     // Enable for Interrupt 1.7
#define PIE_IER1_INTX8    0x80U     // Enable for Interrupt 1.8
#define PIE_IER1_INTX9    0x100U    // Enable for Interrupt 1.9
#define PIE_IER1_INTX10   0x200U    // Enable for Interrupt 1.10
#define PIE_IER1_INTX11   0x400U    // Enable for Interrupt 1.11
#define PIE_IER1_INTX12   0x800U    // Enable for Interrupt 1.12
#define PIE_IER1_INTX13   0x1000U   // Enable for Interrupt 1.13
#define PIE_IER1_INTX14   0x2000U   // Enable for Interrupt 1.14
#define PIE_IER1_INTX15   0x4000U   // Enable for Interrupt 1.15
#define PIE_IER1_INTX16   0x8000U   // Enable for Interrupt 1.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIFR1 register
//
//*************************************************************************************************
#define PIE_IFR1_INTX1    0x1U      // Flag for Interrupt 1.1
#define PIE_IFR1_INTX2    0x2U      // Flag for Interrupt 1.2
#define PIE_IFR1_INTX3    0x4U      // Flag for Interrupt 1.3
#define PIE_IFR1_INTX4    0x8U      // Flag for Interrupt 1.4
#define PIE_IFR1_INTX5    0x10U     // Flag for Interrupt 1.5
#define PIE_IFR1_INTX6    0x20U     // Flag for Interrupt 1.6
#define PIE_IFR1_INTX7    0x40U     // Flag for Interrupt 1.7
#define PIE_IFR1_INTX8    0x80U     // Flag for Interrupt 1.8
#define PIE_IFR1_INTX9    0x100U    // Flag for Interrupt 1.9
#define PIE_IFR1_INTX10   0x200U    // Flag for Interrupt 1.10
#define PIE_IFR1_INTX11   0x400U    // Flag for Interrupt 1.11
#define PIE_IFR1_INTX12   0x800U    // Flag for Interrupt 1.12
#define PIE_IFR1_INTX13   0x1000U   // Flag for Interrupt 1.13
#define PIE_IFR1_INTX14   0x2000U   // Flag for Interrupt 1.14
#define PIE_IFR1_INTX15   0x4000U   // Flag for Interrupt 1.15
#define PIE_IFR1_INTX16   0x8000U   // Flag for Interrupt 1.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIER2 register
//
//*************************************************************************************************
#define PIE_IER2_INTX1    0x1U      // Enable for Interrupt 2.1
#define PIE_IER2_INTX2    0x2U      // Enable for Interrupt 2.2
#define PIE_IER2_INTX3    0x4U      // Enable for Interrupt 2.3
#define PIE_IER2_INTX4    0x8U      // Enable for Interrupt 2.4
#define PIE_IER2_INTX5    0x10U     // Enable for Interrupt 2.5
#define PIE_IER2_INTX6    0x20U     // Enable for Interrupt 2.6
#define PIE_IER2_INTX7    0x40U     // Enable for Interrupt 2.7
#define PIE_IER2_INTX8    0x80U     // Enable for Interrupt 2.8
#define PIE_IER2_INTX9    0x100U    // Enable for Interrupt 2.9
#define PIE_IER2_INTX10   0x200U    // Enable for Interrupt 2.10
#define PIE_IER2_INTX11   0x400U    // Enable for Interrupt 2.11
#define PIE_IER2_INTX12   0x800U    // Enable for Interrupt 2.12
#define PIE_IER2_INTX13   0x1000U   // Enable for Interrupt 2.13
#define PIE_IER2_INTX14   0x2000U   // Enable for Interrupt 2.14
#define PIE_IER2_INTX15   0x4000U   // Enable for Interrupt 2.15
#define PIE_IER2_INTX16   0x8000U   // Enable for Interrupt 2.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIFR2 register
//
//*************************************************************************************************
#define PIE_IFR2_INTX1    0x1U      // Flag for Interrupt 2.1
#define PIE_IFR2_INTX2    0x2U      // Flag for Interrupt 2.2
#define PIE_IFR2_INTX3    0x4U      // Flag for Interrupt 2.3
#define PIE_IFR2_INTX4    0x8U      // Flag for Interrupt 2.4
#define PIE_IFR2_INTX5    0x10U     // Flag for Interrupt 2.5
#define PIE_IFR2_INTX6    0x20U     // Flag for Interrupt 2.6
#define PIE_IFR2_INTX7    0x40U     // Flag for Interrupt 2.7
#define PIE_IFR2_INTX8    0x80U     // Flag for Interrupt 2.8
#define PIE_IFR2_INTX9    0x100U    // Flag for Interrupt 2.9
#define PIE_IFR2_INTX10   0x200U    // Flag for Interrupt 2.10
#define PIE_IFR2_INTX11   0x400U    // Flag for Interrupt 2.11
#define PIE_IFR2_INTX12   0x800U    // Flag for Interrupt 2.12
#define PIE_IFR2_INTX13   0x1000U   // Flag for Interrupt 2.13
#define PIE_IFR2_INTX14   0x2000U   // Flag for Interrupt 2.14
#define PIE_IFR2_INTX15   0x4000U   // Flag for Interrupt 2.15
#define PIE_IFR2_INTX16   0x8000U   // Flag for Interrupt 2.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIER3 register
//
//*************************************************************************************************
#define PIE_IER3_INTX1    0x1U      // Enable for Interrupt 3.1
#define PIE_IER3_INTX2    0x2U      // Enable for Interrupt 3.2
#define PIE_IER3_INTX3    0x4U      // Enable for Interrupt 3.3
#define PIE_IER3_INTX4    0x8U      // Enable for Interrupt 3.4
#define PIE_IER3_INTX5    0x10U     // Enable for Interrupt 3.5
#define PIE_IER3_INTX6    0x20U     // Enable for Interrupt 3.6
#define PIE_IER3_INTX7    0x40U     // Enable for Interrupt 3.7
#define PIE_IER3_INTX8    0x80U     // Enable for Interrupt 3.8
#define PIE_IER3_INTX9    0x100U    // Enable for Interrupt 3.9
#define PIE_IER3_INTX10   0x200U    // Enable for Interrupt 3.10
#define PIE_IER3_INTX11   0x400U    // Enable for Interrupt 3.11
#define PIE_IER3_INTX12   0x800U    // Enable for Interrupt 3.12
#define PIE_IER3_INTX13   0x1000U   // Enable for Interrupt 3.13
#define PIE_IER3_INTX14   0x2000U   // Enable for Interrupt 3.14
#define PIE_IER3_INTX15   0x4000U   // Enable for Interrupt 3.15
#define PIE_IER3_INTX16   0x8000U   // Enable for Interrupt 3.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIFR3 register
//
//*************************************************************************************************
#define PIE_IFR3_INTX1    0x1U      // Flag for Interrupt 3.1
#define PIE_IFR3_INTX2    0x2U      // Flag for Interrupt 3.2
#define PIE_IFR3_INTX3    0x4U      // Flag for Interrupt 3.3
#define PIE_IFR3_INTX4    0x8U      // Flag for Interrupt 3.4
#define PIE_IFR3_INTX5    0x10U     // Flag for Interrupt 3.5
#define PIE_IFR3_INTX6    0x20U     // Flag for Interrupt 3.6
#define PIE_IFR3_INTX7    0x40U     // Flag for Interrupt 3.7
#define PIE_IFR3_INTX8    0x80U     // Flag for Interrupt 3.8
#define PIE_IFR3_INTX9    0x100U    // Flag for Interrupt 3.9
#define PIE_IFR3_INTX10   0x200U    // Flag for Interrupt 3.10
#define PIE_IFR3_INTX11   0x400U    // Flag for Interrupt 3.11
#define PIE_IFR3_INTX12   0x800U    // Flag for Interrupt 3.12
#define PIE_IFR3_INTX13   0x1000U   // Flag for Interrupt 3.13
#define PIE_IFR3_INTX14   0x2000U   // Flag for Interrupt 3.14
#define PIE_IFR3_INTX15   0x4000U   // Flag for Interrupt 3.15
#define PIE_IFR3_INTX16   0x8000U   // Flag for Interrupt 3.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIER4 register
//
//*************************************************************************************************
#define PIE_IER4_INTX1    0x1U      // Enable for Interrupt 4.1
#define PIE_IER4_INTX2    0x2U      // Enable for Interrupt 4.2
#define PIE_IER4_INTX3    0x4U      // Enable for Interrupt 4.3
#define PIE_IER4_INTX4    0x8U      // Enable for Interrupt 4.4
#define PIE_IER4_INTX5    0x10U     // Enable for Interrupt 4.5
#define PIE_IER4_INTX6    0x20U     // Enable for Interrupt 4.6
#define PIE_IER4_INTX7    0x40U     // Enable for Interrupt 4.7
#define PIE_IER4_INTX8    0x80U     // Enable for Interrupt 4.8
#define PIE_IER4_INTX9    0x100U    // Enable for Interrupt 4.9
#define PIE_IER4_INTX10   0x200U    // Enable for Interrupt 4.10
#define PIE_IER4_INTX11   0x400U    // Enable for Interrupt 4.11
#define PIE_IER4_INTX12   0x800U    // Enable for Interrupt 4.12
#define PIE_IER4_INTX13   0x1000U   // Enable for Interrupt 4.13
#define PIE_IER4_INTX14   0x2000U   // Enable for Interrupt 4.14
#define PIE_IER4_INTX15   0x4000U   // Enable for Interrupt 4.15
#define PIE_IER4_INTX16   0x8000U   // Enable for Interrupt 4.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIFR4 register
//
//*************************************************************************************************
#define PIE_IFR4_INTX1    0x1U      // Flag for Interrupt 4.1
#define PIE_IFR4_INTX2    0x2U      // Flag for Interrupt 4.2
#define PIE_IFR4_INTX3    0x4U      // Flag for Interrupt 4.3
#define PIE_IFR4_INTX4    0x8U      // Flag for Interrupt 4.4
#define PIE_IFR4_INTX5    0x10U     // Flag for Interrupt 4.5
#define PIE_IFR4_INTX6    0x20U     // Flag for Interrupt 4.6
#define PIE_IFR4_INTX7    0x40U     // Flag for Interrupt 4.7
#define PIE_IFR4_INTX8    0x80U     // Flag for Interrupt 4.8
#define PIE_IFR4_INTX9    0x100U    // Flag for Interrupt 4.9
#define PIE_IFR4_INTX10   0x200U    // Flag for Interrupt 4.10
#define PIE_IFR4_INTX11   0x400U    // Flag for Interrupt 4.11
#define PIE_IFR4_INTX12   0x800U    // Flag for Interrupt 4.12
#define PIE_IFR4_INTX13   0x1000U   // Flag for Interrupt 4.13
#define PIE_IFR4_INTX14   0x2000U   // Flag for Interrupt 4.14
#define PIE_IFR4_INTX15   0x4000U   // Flag for Interrupt 4.15
#define PIE_IFR4_INTX16   0x8000U   // Flag for Interrupt 4.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIER5 register
//
//*************************************************************************************************
#define PIE_IER5_INTX1    0x1U      // Enable for Interrupt 5.1
#define PIE_IER5_INTX2    0x2U      // Enable for Interrupt 5.2
#define PIE_IER5_INTX3    0x4U      // Enable for Interrupt 5.3
#define PIE_IER5_INTX4    0x8U      // Enable for Interrupt 5.4
#define PIE_IER5_INTX5    0x10U     // Enable for Interrupt 5.5
#define PIE_IER5_INTX6    0x20U     // Enable for Interrupt 5.6
#define PIE_IER5_INTX7    0x40U     // Enable for Interrupt 5.7
#define PIE_IER5_INTX8    0x80U     // Enable for Interrupt 5.8
#define PIE_IER5_INTX9    0x100U    // Enable for Interrupt 5.9
#define PIE_IER5_INTX10   0x200U    // Enable for Interrupt 5.10
#define PIE_IER5_INTX11   0x400U    // Enable for Interrupt 5.11
#define PIE_IER5_INTX12   0x800U    // Enable for Interrupt 5.12
#define PIE_IER5_INTX13   0x1000U   // Enable for Interrupt 5.13
#define PIE_IER5_INTX14   0x2000U   // Enable for Interrupt 5.14
#define PIE_IER5_INTX15   0x4000U   // Enable for Interrupt 5.15
#define PIE_IER5_INTX16   0x8000U   // Enable for Interrupt 5.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIFR5 register
//
//*************************************************************************************************
#define PIE_IFR5_INTX1    0x1U      // Flag for Interrupt 5.1
#define PIE_IFR5_INTX2    0x2U      // Flag for Interrupt 5.2
#define PIE_IFR5_INTX3    0x4U      // Flag for Interrupt 5.3
#define PIE_IFR5_INTX4    0x8U      // Flag for Interrupt 5.4
#define PIE_IFR5_INTX5    0x10U     // Flag for Interrupt 5.5
#define PIE_IFR5_INTX6    0x20U     // Flag for Interrupt 5.6
#define PIE_IFR5_INTX7    0x40U     // Flag for Interrupt 5.7
#define PIE_IFR5_INTX8    0x80U     // Flag for Interrupt 5.8
#define PIE_IFR5_INTX9    0x100U    // Flag for Interrupt 5.9
#define PIE_IFR5_INTX10   0x200U    // Flag for Interrupt 5.10
#define PIE_IFR5_INTX11   0x400U    // Flag for Interrupt 5.11
#define PIE_IFR5_INTX12   0x800U    // Flag for Interrupt 5.12
#define PIE_IFR5_INTX13   0x1000U   // Flag for Interrupt 5.13
#define PIE_IFR5_INTX14   0x2000U   // Flag for Interrupt 5.14
#define PIE_IFR5_INTX15   0x4000U   // Flag for Interrupt 5.15
#define PIE_IFR5_INTX16   0x8000U   // Flag for Interrupt 5.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIER6 register
//
//*************************************************************************************************
#define PIE_IER6_INTX1    0x1U      // Enable for Interrupt 6.1
#define PIE_IER6_INTX2    0x2U      // Enable for Interrupt 6.2
#define PIE_IER6_INTX3    0x4U      // Enable for Interrupt 6.3
#define PIE_IER6_INTX4    0x8U      // Enable for Interrupt 6.4
#define PIE_IER6_INTX5    0x10U     // Enable for Interrupt 6.5
#define PIE_IER6_INTX6    0x20U     // Enable for Interrupt 6.6
#define PIE_IER6_INTX7    0x40U     // Enable for Interrupt 6.7
#define PIE_IER6_INTX8    0x80U     // Enable for Interrupt 6.8
#define PIE_IER6_INTX9    0x100U    // Enable for Interrupt 6.9
#define PIE_IER6_INTX10   0x200U    // Enable for Interrupt 6.10
#define PIE_IER6_INTX11   0x400U    // Enable for Interrupt 6.11
#define PIE_IER6_INTX12   0x800U    // Enable for Interrupt 6.12
#define PIE_IER6_INTX13   0x1000U   // Enable for Interrupt 6.13
#define PIE_IER6_INTX14   0x2000U   // Enable for Interrupt 6.14
#define PIE_IER6_INTX15   0x4000U   // Enable for Interrupt 6.15
#define PIE_IER6_INTX16   0x8000U   // Enable for Interrupt 6.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIFR6 register
//
//*************************************************************************************************
#define PIE_IFR6_INTX1    0x1U      // Flag for Interrupt 6.1
#define PIE_IFR6_INTX2    0x2U      // Flag for Interrupt 6.2
#define PIE_IFR6_INTX3    0x4U      // Flag for Interrupt 6.3
#define PIE_IFR6_INTX4    0x8U      // Flag for Interrupt 6.4
#define PIE_IFR6_INTX5    0x10U     // Flag for Interrupt 6.5
#define PIE_IFR6_INTX6    0x20U     // Flag for Interrupt 6.6
#define PIE_IFR6_INTX7    0x40U     // Flag for Interrupt 6.7
#define PIE_IFR6_INTX8    0x80U     // Flag for Interrupt 6.8
#define PIE_IFR6_INTX9    0x100U    // Flag for Interrupt 6.9
#define PIE_IFR6_INTX10   0x200U    // Flag for Interrupt 6.10
#define PIE_IFR6_INTX11   0x400U    // Flag for Interrupt 6.11
#define PIE_IFR6_INTX12   0x800U    // Flag for Interrupt 6.12
#define PIE_IFR6_INTX13   0x1000U   // Flag for Interrupt 6.13
#define PIE_IFR6_INTX14   0x2000U   // Flag for Interrupt 6.14
#define PIE_IFR6_INTX15   0x4000U   // Flag for Interrupt 6.15
#define PIE_IFR6_INTX16   0x8000U   // Flag for Interrupt 6.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIER7 register
//
//*************************************************************************************************
#define PIE_IER7_INTX1    0x1U      // Enable for Interrupt 7.1
#define PIE_IER7_INTX2    0x2U      // Enable for Interrupt 7.2
#define PIE_IER7_INTX3    0x4U      // Enable for Interrupt 7.3
#define PIE_IER7_INTX4    0x8U      // Enable for Interrupt 7.4
#define PIE_IER7_INTX5    0x10U     // Enable for Interrupt 7.5
#define PIE_IER7_INTX6    0x20U     // Enable for Interrupt 7.6
#define PIE_IER7_INTX7    0x40U     // Enable for Interrupt 7.7
#define PIE_IER7_INTX8    0x80U     // Enable for Interrupt 7.8
#define PIE_IER7_INTX9    0x100U    // Enable for Interrupt 7.9
#define PIE_IER7_INTX10   0x200U    // Enable for Interrupt 7.10
#define PIE_IER7_INTX11   0x400U    // Enable for Interrupt 7.11
#define PIE_IER7_INTX12   0x800U    // Enable for Interrupt 7.12
#define PIE_IER7_INTX13   0x1000U   // Enable for Interrupt 7.13
#define PIE_IER7_INTX14   0x2000U   // Enable for Interrupt 7.14
#define PIE_IER7_INTX15   0x4000U   // Enable for Interrupt 7.15
#define PIE_IER7_INTX16   0x8000U   // Enable for Interrupt 7.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIFR7 register
//
//*************************************************************************************************
#define PIE_IFR7_INTX1    0x1U      // Flag for Interrupt 7.1
#define PIE_IFR7_INTX2    0x2U      // Flag for Interrupt 7.2
#define PIE_IFR7_INTX3    0x4U      // Flag for Interrupt 7.3
#define PIE_IFR7_INTX4    0x8U      // Flag for Interrupt 7.4
#define PIE_IFR7_INTX5    0x10U     // Flag for Interrupt 7.5
#define PIE_IFR7_INTX6    0x20U     // Flag for Interrupt 7.6
#define PIE_IFR7_INTX7    0x40U     // Flag for Interrupt 7.7
#define PIE_IFR7_INTX8    0x80U     // Flag for Interrupt 7.8
#define PIE_IFR7_INTX9    0x100U    // Flag for Interrupt 7.9
#define PIE_IFR7_INTX10   0x200U    // Flag for Interrupt 7.10
#define PIE_IFR7_INTX11   0x400U    // Flag for Interrupt 7.11
#define PIE_IFR7_INTX12   0x800U    // Flag for Interrupt 7.12
#define PIE_IFR7_INTX13   0x1000U   // Flag for Interrupt 7.13
#define PIE_IFR7_INTX14   0x2000U   // Flag for Interrupt 7.14
#define PIE_IFR7_INTX15   0x4000U   // Flag for Interrupt 7.15
#define PIE_IFR7_INTX16   0x8000U   // Flag for Interrupt 7.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIER8 register
//
//*************************************************************************************************
#define PIE_IER8_INTX1    0x1U      // Enable for Interrupt 8.1
#define PIE_IER8_INTX2    0x2U      // Enable for Interrupt 8.2
#define PIE_IER8_INTX3    0x4U      // Enable for Interrupt 8.3
#define PIE_IER8_INTX4    0x8U      // Enable for Interrupt 8.4
#define PIE_IER8_INTX5    0x10U     // Enable for Interrupt 8.5
#define PIE_IER8_INTX6    0x20U     // Enable for Interrupt 8.6
#define PIE_IER8_INTX7    0x40U     // Enable for Interrupt 8.7
#define PIE_IER8_INTX8    0x80U     // Enable for Interrupt 8.8
#define PIE_IER8_INTX9    0x100U    // Enable for Interrupt 8.9
#define PIE_IER8_INTX10   0x200U    // Enable for Interrupt 8.10
#define PIE_IER8_INTX11   0x400U    // Enable for Interrupt 8.11
#define PIE_IER8_INTX12   0x800U    // Enable for Interrupt 8.12
#define PIE_IER8_INTX13   0x1000U   // Enable for Interrupt 8.13
#define PIE_IER8_INTX14   0x2000U   // Enable for Interrupt 8.14
#define PIE_IER8_INTX15   0x4000U   // Enable for Interrupt 8.15
#define PIE_IER8_INTX16   0x8000U   // Enable for Interrupt 8.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIFR8 register
//
//*************************************************************************************************
#define PIE_IFR8_INTX1    0x1U      // Flag for Interrupt 8.1
#define PIE_IFR8_INTX2    0x2U      // Flag for Interrupt 8.2
#define PIE_IFR8_INTX3    0x4U      // Flag for Interrupt 8.3
#define PIE_IFR8_INTX4    0x8U      // Flag for Interrupt 8.4
#define PIE_IFR8_INTX5    0x10U     // Flag for Interrupt 8.5
#define PIE_IFR8_INTX6    0x20U     // Flag for Interrupt 8.6
#define PIE_IFR8_INTX7    0x40U     // Flag for Interrupt 8.7
#define PIE_IFR8_INTX8    0x80U     // Flag for Interrupt 8.8
#define PIE_IFR8_INTX9    0x100U    // Flag for Interrupt 8.9
#define PIE_IFR8_INTX10   0x200U    // Flag for Interrupt 8.10
#define PIE_IFR8_INTX11   0x400U    // Flag for Interrupt 8.11
#define PIE_IFR8_INTX12   0x800U    // Flag for Interrupt 8.12
#define PIE_IFR8_INTX13   0x1000U   // Flag for Interrupt 8.13
#define PIE_IFR8_INTX14   0x2000U   // Flag for Interrupt 8.14
#define PIE_IFR8_INTX15   0x4000U   // Flag for Interrupt 8.15
#define PIE_IFR8_INTX16   0x8000U   // Flag for Interrupt 8.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIER9 register
//
//*************************************************************************************************
#define PIE_IER9_INTX1    0x1U      // Enable for Interrupt 9.1
#define PIE_IER9_INTX2    0x2U      // Enable for Interrupt 9.2
#define PIE_IER9_INTX3    0x4U      // Enable for Interrupt 9.3
#define PIE_IER9_INTX4    0x8U      // Enable for Interrupt 9.4
#define PIE_IER9_INTX5    0x10U     // Enable for Interrupt 9.5
#define PIE_IER9_INTX6    0x20U     // Enable for Interrupt 9.6
#define PIE_IER9_INTX7    0x40U     // Enable for Interrupt 9.7
#define PIE_IER9_INTX8    0x80U     // Enable for Interrupt 9.8
#define PIE_IER9_INTX9    0x100U    // Enable for Interrupt 9.9
#define PIE_IER9_INTX10   0x200U    // Enable for Interrupt 9.10
#define PIE_IER9_INTX11   0x400U    // Enable for Interrupt 9.11
#define PIE_IER9_INTX12   0x800U    // Enable for Interrupt 9.12
#define PIE_IER9_INTX13   0x1000U   // Enable for Interrupt 9.13
#define PIE_IER9_INTX14   0x2000U   // Enable for Interrupt 9.14
#define PIE_IER9_INTX15   0x4000U   // Enable for Interrupt 9.15
#define PIE_IER9_INTX16   0x8000U   // Enable for Interrupt 9.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIFR9 register
//
//*************************************************************************************************
#define PIE_IFR9_INTX1    0x1U      // Flag for Interrupt 9.1
#define PIE_IFR9_INTX2    0x2U      // Flag for Interrupt 9.2
#define PIE_IFR9_INTX3    0x4U      // Flag for Interrupt 9.3
#define PIE_IFR9_INTX4    0x8U      // Flag for Interrupt 9.4
#define PIE_IFR9_INTX5    0x10U     // Flag for Interrupt 9.5
#define PIE_IFR9_INTX6    0x20U     // Flag for Interrupt 9.6
#define PIE_IFR9_INTX7    0x40U     // Flag for Interrupt 9.7
#define PIE_IFR9_INTX8    0x80U     // Flag for Interrupt 9.8
#define PIE_IFR9_INTX9    0x100U    // Flag for Interrupt 9.9
#define PIE_IFR9_INTX10   0x200U    // Flag for Interrupt 9.10
#define PIE_IFR9_INTX11   0x400U    // Flag for Interrupt 9.11
#define PIE_IFR9_INTX12   0x800U    // Flag for Interrupt 9.12
#define PIE_IFR9_INTX13   0x1000U   // Flag for Interrupt 9.13
#define PIE_IFR9_INTX14   0x2000U   // Flag for Interrupt 9.14
#define PIE_IFR9_INTX15   0x4000U   // Flag for Interrupt 9.15
#define PIE_IFR9_INTX16   0x8000U   // Flag for Interrupt 9.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIER10 register
//
//*************************************************************************************************
#define PIE_IER10_INTX1    0x1U      // Enable for Interrupt 10.1
#define PIE_IER10_INTX2    0x2U      // Enable for Interrupt 10.2
#define PIE_IER10_INTX3    0x4U      // Enable for Interrupt 10.3
#define PIE_IER10_INTX4    0x8U      // Enable for Interrupt 10.4
#define PIE_IER10_INTX5    0x10U     // Enable for Interrupt 10.5
#define PIE_IER10_INTX6    0x20U     // Enable for Interrupt 10.6
#define PIE_IER10_INTX7    0x40U     // Enable for Interrupt 10.7
#define PIE_IER10_INTX8    0x80U     // Enable for Interrupt 10.8
#define PIE_IER10_INTX9    0x100U    // Enable for Interrupt 10.9
#define PIE_IER10_INTX10   0x200U    // Enable for Interrupt 10.10
#define PIE_IER10_INTX11   0x400U    // Enable for Interrupt 10.11
#define PIE_IER10_INTX12   0x800U    // Enable for Interrupt 10.12
#define PIE_IER10_INTX13   0x1000U   // Enable for Interrupt 10.13
#define PIE_IER10_INTX14   0x2000U   // Enable for Interrupt 10.14
#define PIE_IER10_INTX15   0x4000U   // Enable for Interrupt 10.15
#define PIE_IER10_INTX16   0x8000U   // Enable for Interrupt 10.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIFR10 register
//
//*************************************************************************************************
#define PIE_IFR10_INTX1    0x1U      // Flag for Interrupt 10.1
#define PIE_IFR10_INTX2    0x2U      // Flag for Interrupt 10.2
#define PIE_IFR10_INTX3    0x4U      // Flag for Interrupt 10.3
#define PIE_IFR10_INTX4    0x8U      // Flag for Interrupt 10.4
#define PIE_IFR10_INTX5    0x10U     // Flag for Interrupt 10.5
#define PIE_IFR10_INTX6    0x20U     // Flag for Interrupt 10.6
#define PIE_IFR10_INTX7    0x40U     // Flag for Interrupt 10.7
#define PIE_IFR10_INTX8    0x80U     // Flag for Interrupt 10.8
#define PIE_IFR10_INTX9    0x100U    // Flag for Interrupt 10.9
#define PIE_IFR10_INTX10   0x200U    // Flag for Interrupt 10.10
#define PIE_IFR10_INTX11   0x400U    // Flag for Interrupt 10.11
#define PIE_IFR10_INTX12   0x800U    // Flag for Interrupt 10.12
#define PIE_IFR10_INTX13   0x1000U   // Flag for Interrupt 10.13
#define PIE_IFR10_INTX14   0x2000U   // Flag for Interrupt 10.14
#define PIE_IFR10_INTX15   0x4000U   // Flag for Interrupt 10.15
#define PIE_IFR10_INTX16   0x8000U   // Flag for Interrupt 10.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIER11 register
//
//*************************************************************************************************
#define PIE_IER11_INTX1    0x1U      // Enable for Interrupt 11.1
#define PIE_IER11_INTX2    0x2U      // Enable for Interrupt 11.2
#define PIE_IER11_INTX3    0x4U      // Enable for Interrupt 11.3
#define PIE_IER11_INTX4    0x8U      // Enable for Interrupt 11.4
#define PIE_IER11_INTX5    0x10U     // Enable for Interrupt 11.5
#define PIE_IER11_INTX6    0x20U     // Enable for Interrupt 11.6
#define PIE_IER11_INTX7    0x40U     // Enable for Interrupt 11.7
#define PIE_IER11_INTX8    0x80U     // Enable for Interrupt 11.8
#define PIE_IER11_INTX9    0x100U    // Enable for Interrupt 11.9
#define PIE_IER11_INTX10   0x200U    // Enable for Interrupt 11.10
#define PIE_IER11_INTX11   0x400U    // Enable for Interrupt 11.11
#define PIE_IER11_INTX12   0x800U    // Enable for Interrupt 11.12
#define PIE_IER11_INTX13   0x1000U   // Enable for Interrupt 11.13
#define PIE_IER11_INTX14   0x2000U   // Enable for Interrupt 11.14
#define PIE_IER11_INTX15   0x4000U   // Enable for Interrupt 11.15
#define PIE_IER11_INTX16   0x8000U   // Enable for Interrupt 11.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIFR11 register
//
//*************************************************************************************************
#define PIE_IFR11_INTX1    0x1U      // Flag for Interrupt 11.1
#define PIE_IFR11_INTX2    0x2U      // Flag for Interrupt 11.2
#define PIE_IFR11_INTX3    0x4U      // Flag for Interrupt 11.3
#define PIE_IFR11_INTX4    0x8U      // Flag for Interrupt 11.4
#define PIE_IFR11_INTX5    0x10U     // Flag for Interrupt 11.5
#define PIE_IFR11_INTX6    0x20U     // Flag for Interrupt 11.6
#define PIE_IFR11_INTX7    0x40U     // Flag for Interrupt 11.7
#define PIE_IFR11_INTX8    0x80U     // Flag for Interrupt 11.8
#define PIE_IFR11_INTX9    0x100U    // Flag for Interrupt 11.9
#define PIE_IFR11_INTX10   0x200U    // Flag for Interrupt 11.10
#define PIE_IFR11_INTX11   0x400U    // Flag for Interrupt 11.11
#define PIE_IFR11_INTX12   0x800U    // Flag for Interrupt 11.12
#define PIE_IFR11_INTX13   0x1000U   // Flag for Interrupt 11.13
#define PIE_IFR11_INTX14   0x2000U   // Flag for Interrupt 11.14
#define PIE_IFR11_INTX15   0x4000U   // Flag for Interrupt 11.15
#define PIE_IFR11_INTX16   0x8000U   // Flag for Interrupt 11.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIER12 register
//
//*************************************************************************************************
#define PIE_IER12_INTX1    0x1U      // Enable for Interrupt 12.1
#define PIE_IER12_INTX2    0x2U      // Enable for Interrupt 12.2
#define PIE_IER12_INTX3    0x4U      // Enable for Interrupt 12.3
#define PIE_IER12_INTX4    0x8U      // Enable for Interrupt 12.4
#define PIE_IER12_INTX5    0x10U     // Enable for Interrupt 12.5
#define PIE_IER12_INTX6    0x20U     // Enable for Interrupt 12.6
#define PIE_IER12_INTX7    0x40U     // Enable for Interrupt 12.7
#define PIE_IER12_INTX8    0x80U     // Enable for Interrupt 12.8
#define PIE_IER12_INTX9    0x100U    // Enable for Interrupt 12.9
#define PIE_IER12_INTX10   0x200U    // Enable for Interrupt 12.10
#define PIE_IER12_INTX11   0x400U    // Enable for Interrupt 12.11
#define PIE_IER12_INTX12   0x800U    // Enable for Interrupt 12.12
#define PIE_IER12_INTX13   0x1000U   // Enable for Interrupt 12.13
#define PIE_IER12_INTX14   0x2000U   // Enable for Interrupt 12.14
#define PIE_IER12_INTX15   0x4000U   // Enable for Interrupt 12.15
#define PIE_IER12_INTX16   0x8000U   // Enable for Interrupt 12.16

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEIFR12 register
//
//*************************************************************************************************
#define PIE_IFR12_INTX1    0x1U      // Flag for Interrupt 12.1
#define PIE_IFR12_INTX2    0x2U      // Flag for Interrupt 12.2
#define PIE_IFR12_INTX3    0x4U      // Flag for Interrupt 12.3
#define PIE_IFR12_INTX4    0x8U      // Flag for Interrupt 12.4
#define PIE_IFR12_INTX5    0x10U     // Flag for Interrupt 12.5
#define PIE_IFR12_INTX6    0x20U     // Flag for Interrupt 12.6
#define PIE_IFR12_INTX7    0x40U     // Flag for Interrupt 12.7
#define PIE_IFR12_INTX8    0x80U     // Flag for Interrupt 12.8
#define PIE_IFR12_INTX9    0x100U    // Flag for Interrupt 12.9
#define PIE_IFR12_INTX10   0x200U    // Flag for Interrupt 12.10
#define PIE_IFR12_INTX11   0x400U    // Flag for Interrupt 12.11
#define PIE_IFR12_INTX12   0x800U    // Flag for Interrupt 12.12
#define PIE_IFR12_INTX13   0x1000U   // Flag for Interrupt 12.13
#define PIE_IFR12_INTX14   0x2000U   // Flag for Interrupt 12.14
#define PIE_IFR12_INTX15   0x4000U   // Flag for Interrupt 12.15
#define PIE_IFR12_INTX16   0x8000U   // Flag for Interrupt 12.16



#endif
