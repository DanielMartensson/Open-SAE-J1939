//###########################################################################
//
// FILE:    hw_outputxbar.h
//
// TITLE:   Definitions for the XBAR registers.
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

#ifndef HW_OUTPUTXBAR_H
#define HW_OUTPUTXBAR_H

//*************************************************************************************************
//
// The following are defines for the XBAR register offsets
//
//*************************************************************************************************
#define XBAR_O_OUTPUT1MUX0TO15CFG    0x0U    // Output X-BAR Mux Configuration for Output 1
#define XBAR_O_OUTPUT1MUX16TO31CFG   0x2U    // Output X-BAR Mux Configuration for Output 1
#define XBAR_O_OUTPUT2MUX0TO15CFG    0x4U    // Output X-BAR Mux Configuration for Output 2
#define XBAR_O_OUTPUT2MUX16TO31CFG   0x6U    // Output X-BAR Mux Configuration for Output 2
#define XBAR_O_OUTPUT3MUX0TO15CFG    0x8U    // Output X-BAR Mux Configuration for Output 3
#define XBAR_O_OUTPUT3MUX16TO31CFG   0xAU    // Output X-BAR Mux Configuration for Output 3
#define XBAR_O_OUTPUT4MUX0TO15CFG    0xCU    // Output X-BAR Mux Configuration for Output 4
#define XBAR_O_OUTPUT4MUX16TO31CFG   0xEU    // Output X-BAR Mux Configuration for Output 4
#define XBAR_O_OUTPUT5MUX0TO15CFG    0x10U   // Output X-BAR Mux Configuration for Output 5
#define XBAR_O_OUTPUT5MUX16TO31CFG   0x12U   // Output X-BAR Mux Configuration for Output 5
#define XBAR_O_OUTPUT6MUX0TO15CFG    0x14U   // Output X-BAR Mux Configuration for Output 6
#define XBAR_O_OUTPUT6MUX16TO31CFG   0x16U   // Output X-BAR Mux Configuration for Output 6
#define XBAR_O_OUTPUT7MUX0TO15CFG    0x18U   // Output X-BAR Mux Configuration for Output 7
#define XBAR_O_OUTPUT7MUX16TO31CFG   0x1AU   // Output X-BAR Mux Configuration for Output 7
#define XBAR_O_OUTPUT8MUX0TO15CFG    0x1CU   // Output X-BAR Mux Configuration for Output 8
#define XBAR_O_OUTPUT8MUX16TO31CFG   0x1EU   // Output X-BAR Mux Configuration for Output 8
#define XBAR_O_OUTPUT1MUXENABLE      0x20U   // Output X-BAR Mux Enable  for Output 1
#define XBAR_O_OUTPUT2MUXENABLE      0x22U   // Output X-BAR Mux Enable  for Output 2
#define XBAR_O_OUTPUT3MUXENABLE      0x24U   // Output X-BAR Mux Enable  for Output 3
#define XBAR_O_OUTPUT4MUXENABLE      0x26U   // Output X-BAR Mux Enable  for Output 4
#define XBAR_O_OUTPUT5MUXENABLE      0x28U   // Output X-BAR Mux Enable  for Output 5
#define XBAR_O_OUTPUT6MUXENABLE      0x2AU   // Output X-BAR Mux Enable  for Output 6
#define XBAR_O_OUTPUT7MUXENABLE      0x2CU   // Output X-BAR Mux Enable  for Output 7
#define XBAR_O_OUTPUT8MUXENABLE      0x2EU   // Output X-BAR Mux Enable  for Output 8
#define XBAR_O_OUTPUTLATCH           0x30U   // Output X-BAR Output Latch
#define XBAR_O_OUTPUTLATCHCLR        0x32U   // Output X-BAR Output Latch Clear
#define XBAR_O_OUTPUTLATCHFRC        0x34U   // Output X-BAR Output Latch Clear
#define XBAR_O_OUTPUTLATCHENABLE     0x36U   // Output X-BAR Output Latch Enable
#define XBAR_O_OUTPUTINV             0x38U   // Output X-BAR Output Inversion
#define XBAR_O_OUTPUTLOCK            0x3EU   // Output X-BAR Configuration Lock register


//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT1MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT1MUX0TO15CFG_MUX0_S    0U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX0TO15CFG_MUX1_S    2U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX0TO15CFG_MUX2_S    4U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX0TO15CFG_MUX3_S    6U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX0TO15CFG_MUX4_S    8U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX0TO15CFG_MUX5_S    10U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX0TO15CFG_MUX6_S    12U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX0TO15CFG_MUX7_S    14U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX0TO15CFG_MUX8_S    16U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX0TO15CFG_MUX9_S    18U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX0TO15CFG_MUX10_S   20U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX0TO15CFG_MUX11_S   22U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX0TO15CFG_MUX12_S   24U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX0TO15CFG_MUX13_S   26U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX0TO15CFG_MUX14_S   28U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX0TO15CFG_MUX15_S   30U
#define XBAR_OUTPUT1MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for OUTPUT1 of
                                                        // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT1MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT1MUX16TO31CFG_MUX16_S   0U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX16TO31CFG_MUX17_S   2U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX16TO31CFG_MUX18_S   4U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX16TO31CFG_MUX19_S   6U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX16TO31CFG_MUX20_S   8U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX16TO31CFG_MUX21_S   10U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX16TO31CFG_MUX22_S   12U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX16TO31CFG_MUX23_S   14U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX16TO31CFG_MUX24_S   16U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX16TO31CFG_MUX25_S   18U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX16TO31CFG_MUX26_S   20U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX16TO31CFG_MUX27_S   22U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX16TO31CFG_MUX28_S   24U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX16TO31CFG_MUX29_S   26U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX16TO31CFG_MUX30_S   28U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT1MUX16TO31CFG_MUX31_S   30U
#define XBAR_OUTPUT1MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for OUTPUT1 of
                                                         // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT2MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT2MUX0TO15CFG_MUX0_S    0U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX0TO15CFG_MUX1_S    2U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX0TO15CFG_MUX2_S    4U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX0TO15CFG_MUX3_S    6U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX0TO15CFG_MUX4_S    8U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX0TO15CFG_MUX5_S    10U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX0TO15CFG_MUX6_S    12U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX0TO15CFG_MUX7_S    14U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX0TO15CFG_MUX8_S    16U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX0TO15CFG_MUX9_S    18U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX0TO15CFG_MUX10_S   20U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX0TO15CFG_MUX11_S   22U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX0TO15CFG_MUX12_S   24U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX0TO15CFG_MUX13_S   26U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX0TO15CFG_MUX14_S   28U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX0TO15CFG_MUX15_S   30U
#define XBAR_OUTPUT2MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for OUTPUT2 of
                                                        // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT2MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT2MUX16TO31CFG_MUX16_S   0U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX16TO31CFG_MUX17_S   2U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX16TO31CFG_MUX18_S   4U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX16TO31CFG_MUX19_S   6U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX16TO31CFG_MUX20_S   8U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX16TO31CFG_MUX21_S   10U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX16TO31CFG_MUX22_S   12U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX16TO31CFG_MUX23_S   14U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX16TO31CFG_MUX24_S   16U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX16TO31CFG_MUX25_S   18U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX16TO31CFG_MUX26_S   20U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX16TO31CFG_MUX27_S   22U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX16TO31CFG_MUX28_S   24U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX16TO31CFG_MUX29_S   26U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX16TO31CFG_MUX30_S   28U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT2MUX16TO31CFG_MUX31_S   30U
#define XBAR_OUTPUT2MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for OUTPUT2 of
                                                         // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT3MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT3MUX0TO15CFG_MUX0_S    0U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX0TO15CFG_MUX1_S    2U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX0TO15CFG_MUX2_S    4U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX0TO15CFG_MUX3_S    6U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX0TO15CFG_MUX4_S    8U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX0TO15CFG_MUX5_S    10U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX0TO15CFG_MUX6_S    12U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX0TO15CFG_MUX7_S    14U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX0TO15CFG_MUX8_S    16U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX0TO15CFG_MUX9_S    18U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX0TO15CFG_MUX10_S   20U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX0TO15CFG_MUX11_S   22U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX0TO15CFG_MUX12_S   24U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX0TO15CFG_MUX13_S   26U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX0TO15CFG_MUX14_S   28U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX0TO15CFG_MUX15_S   30U
#define XBAR_OUTPUT3MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for OUTPUT3 of
                                                        // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT3MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT3MUX16TO31CFG_MUX16_S   0U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX16TO31CFG_MUX17_S   2U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX16TO31CFG_MUX18_S   4U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX16TO31CFG_MUX19_S   6U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX16TO31CFG_MUX20_S   8U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX16TO31CFG_MUX21_S   10U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX16TO31CFG_MUX22_S   12U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX16TO31CFG_MUX23_S   14U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX16TO31CFG_MUX24_S   16U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX16TO31CFG_MUX25_S   18U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX16TO31CFG_MUX26_S   20U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX16TO31CFG_MUX27_S   22U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX16TO31CFG_MUX28_S   24U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX16TO31CFG_MUX29_S   26U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX16TO31CFG_MUX30_S   28U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT3MUX16TO31CFG_MUX31_S   30U
#define XBAR_OUTPUT3MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for OUTPUT3 of
                                                         // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT4MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT4MUX0TO15CFG_MUX0_S    0U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX0TO15CFG_MUX1_S    2U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX0TO15CFG_MUX2_S    4U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX0TO15CFG_MUX3_S    6U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX0TO15CFG_MUX4_S    8U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX0TO15CFG_MUX5_S    10U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX0TO15CFG_MUX6_S    12U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX0TO15CFG_MUX7_S    14U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX0TO15CFG_MUX8_S    16U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX0TO15CFG_MUX9_S    18U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX0TO15CFG_MUX10_S   20U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX0TO15CFG_MUX11_S   22U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX0TO15CFG_MUX12_S   24U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX0TO15CFG_MUX13_S   26U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX0TO15CFG_MUX14_S   28U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX0TO15CFG_MUX15_S   30U
#define XBAR_OUTPUT4MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for OUTPUT4 of
                                                        // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT4MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT4MUX16TO31CFG_MUX16_S   0U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX16TO31CFG_MUX17_S   2U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX16TO31CFG_MUX18_S   4U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX16TO31CFG_MUX19_S   6U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX16TO31CFG_MUX20_S   8U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX16TO31CFG_MUX21_S   10U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX16TO31CFG_MUX22_S   12U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX16TO31CFG_MUX23_S   14U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX16TO31CFG_MUX24_S   16U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX16TO31CFG_MUX25_S   18U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX16TO31CFG_MUX26_S   20U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX16TO31CFG_MUX27_S   22U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX16TO31CFG_MUX28_S   24U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX16TO31CFG_MUX29_S   26U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX16TO31CFG_MUX30_S   28U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT4MUX16TO31CFG_MUX31_S   30U
#define XBAR_OUTPUT4MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for OUTPUT4 of
                                                         // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT5MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT5MUX0TO15CFG_MUX0_S    0U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX0TO15CFG_MUX1_S    2U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX0TO15CFG_MUX2_S    4U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX0TO15CFG_MUX3_S    6U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX0TO15CFG_MUX4_S    8U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX0TO15CFG_MUX5_S    10U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX0TO15CFG_MUX6_S    12U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX0TO15CFG_MUX7_S    14U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX0TO15CFG_MUX8_S    16U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX0TO15CFG_MUX9_S    18U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX0TO15CFG_MUX10_S   20U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX0TO15CFG_MUX11_S   22U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX0TO15CFG_MUX12_S   24U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX0TO15CFG_MUX13_S   26U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX0TO15CFG_MUX14_S   28U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX0TO15CFG_MUX15_S   30U
#define XBAR_OUTPUT5MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for OUTPUT5 of
                                                        // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT5MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT5MUX16TO31CFG_MUX16_S   0U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX16TO31CFG_MUX17_S   2U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX16TO31CFG_MUX18_S   4U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX16TO31CFG_MUX19_S   6U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX16TO31CFG_MUX20_S   8U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX16TO31CFG_MUX21_S   10U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX16TO31CFG_MUX22_S   12U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX16TO31CFG_MUX23_S   14U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX16TO31CFG_MUX24_S   16U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX16TO31CFG_MUX25_S   18U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX16TO31CFG_MUX26_S   20U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX16TO31CFG_MUX27_S   22U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX16TO31CFG_MUX28_S   24U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX16TO31CFG_MUX29_S   26U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX16TO31CFG_MUX30_S   28U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT5MUX16TO31CFG_MUX31_S   30U
#define XBAR_OUTPUT5MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for OUTPUT5 of
                                                         // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT6MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT6MUX0TO15CFG_MUX0_S    0U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX0TO15CFG_MUX1_S    2U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX0TO15CFG_MUX2_S    4U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX0TO15CFG_MUX3_S    6U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX0TO15CFG_MUX4_S    8U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX0TO15CFG_MUX5_S    10U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX0TO15CFG_MUX6_S    12U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX0TO15CFG_MUX7_S    14U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX0TO15CFG_MUX8_S    16U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX0TO15CFG_MUX9_S    18U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX0TO15CFG_MUX10_S   20U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX0TO15CFG_MUX11_S   22U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX0TO15CFG_MUX12_S   24U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX0TO15CFG_MUX13_S   26U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX0TO15CFG_MUX14_S   28U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX0TO15CFG_MUX15_S   30U
#define XBAR_OUTPUT6MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for OUTPUT6 of
                                                        // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT6MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT6MUX16TO31CFG_MUX16_S   0U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX16TO31CFG_MUX17_S   2U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX16TO31CFG_MUX18_S   4U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX16TO31CFG_MUX19_S   6U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX16TO31CFG_MUX20_S   8U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX16TO31CFG_MUX21_S   10U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX16TO31CFG_MUX22_S   12U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX16TO31CFG_MUX23_S   14U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX16TO31CFG_MUX24_S   16U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX16TO31CFG_MUX25_S   18U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX16TO31CFG_MUX26_S   20U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX16TO31CFG_MUX27_S   22U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX16TO31CFG_MUX28_S   24U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX16TO31CFG_MUX29_S   26U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX16TO31CFG_MUX30_S   28U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT6MUX16TO31CFG_MUX31_S   30U
#define XBAR_OUTPUT6MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for OUTPUT6 of
                                                         // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT7MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT7MUX0TO15CFG_MUX0_S    0U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX0TO15CFG_MUX1_S    2U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX0TO15CFG_MUX2_S    4U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX0TO15CFG_MUX3_S    6U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX0TO15CFG_MUX4_S    8U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX0TO15CFG_MUX5_S    10U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX0TO15CFG_MUX6_S    12U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX0TO15CFG_MUX7_S    14U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX0TO15CFG_MUX8_S    16U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX0TO15CFG_MUX9_S    18U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX0TO15CFG_MUX10_S   20U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX0TO15CFG_MUX11_S   22U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX0TO15CFG_MUX12_S   24U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX0TO15CFG_MUX13_S   26U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX0TO15CFG_MUX14_S   28U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX0TO15CFG_MUX15_S   30U
#define XBAR_OUTPUT7MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for OUTPUT7 of
                                                        // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT7MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT7MUX16TO31CFG_MUX16_S   0U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX16TO31CFG_MUX17_S   2U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX16TO31CFG_MUX18_S   4U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX16TO31CFG_MUX19_S   6U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX16TO31CFG_MUX20_S   8U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX16TO31CFG_MUX21_S   10U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX16TO31CFG_MUX22_S   12U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX16TO31CFG_MUX23_S   14U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX16TO31CFG_MUX24_S   16U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX16TO31CFG_MUX25_S   18U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX16TO31CFG_MUX26_S   20U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX16TO31CFG_MUX27_S   22U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX16TO31CFG_MUX28_S   24U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX16TO31CFG_MUX29_S   26U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX16TO31CFG_MUX30_S   28U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT7MUX16TO31CFG_MUX31_S   30U
#define XBAR_OUTPUT7MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for OUTPUT7 of
                                                         // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT8MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT8MUX0TO15CFG_MUX0_S    0U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX0TO15CFG_MUX1_S    2U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX0TO15CFG_MUX2_S    4U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX0TO15CFG_MUX3_S    6U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX0TO15CFG_MUX4_S    8U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX0TO15CFG_MUX5_S    10U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX0TO15CFG_MUX6_S    12U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX0TO15CFG_MUX7_S    14U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX0TO15CFG_MUX8_S    16U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX0TO15CFG_MUX9_S    18U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX0TO15CFG_MUX10_S   20U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX0TO15CFG_MUX11_S   22U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX0TO15CFG_MUX12_S   24U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX0TO15CFG_MUX13_S   26U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX0TO15CFG_MUX14_S   28U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX0TO15CFG_MUX15_S   30U
#define XBAR_OUTPUT8MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for OUTPUT8 of
                                                        // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT8MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_OUTPUT8MUX16TO31CFG_MUX16_S   0U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX16TO31CFG_MUX17_S   2U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX16TO31CFG_MUX18_S   4U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX16TO31CFG_MUX19_S   6U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX16TO31CFG_MUX20_S   8U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX16TO31CFG_MUX21_S   10U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX16TO31CFG_MUX22_S   12U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX16TO31CFG_MUX23_S   14U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX16TO31CFG_MUX24_S   16U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX16TO31CFG_MUX25_S   18U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX16TO31CFG_MUX26_S   20U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX16TO31CFG_MUX27_S   22U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX16TO31CFG_MUX28_S   24U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX16TO31CFG_MUX29_S   26U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX16TO31CFG_MUX30_S   28U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR
#define XBAR_OUTPUT8MUX16TO31CFG_MUX31_S   30U
#define XBAR_OUTPUT8MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for OUTPUT8 of
                                                         // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT1MUXENABLE register
//
//*************************************************************************************************
#define XBAR_OUTPUT1MUXENABLE_MUX0    0x1U          // Mux0 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX1    0x2U          // Mux1 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX2    0x4U          // Mux2 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX3    0x8U          // Mux3 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX4    0x10U         // Mux4 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX5    0x20U         // Mux5 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX6    0x40U         // Mux6 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX7    0x80U         // Mux7 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX8    0x100U        // Mux8 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX9    0x200U        // Mux9 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX10   0x400U        // Mux10 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX11   0x800U        // Mux11 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX12   0x1000U       // Mux12 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX13   0x2000U       // Mux13 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX14   0x4000U       // Mux14 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX15   0x8000U       // Mux15 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX16   0x10000U      // Mux16 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX17   0x20000U      // Mux17 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX18   0x40000U      // Mux18 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX19   0x80000U      // Mux19 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX20   0x100000U     // Mux20 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX21   0x200000U     // Mux21 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX22   0x400000U     // Mux22 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX23   0x800000U     // Mux23 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX24   0x1000000U    // Mux24 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX25   0x2000000U    // Mux25 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX26   0x4000000U    // Mux26 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX27   0x8000000U    // Mux27 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX28   0x10000000U   // Mux28 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX29   0x20000000U   // Mux29 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX30   0x40000000U   // Mux30 to drive OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUT1MUXENABLE_MUX31   0x80000000U   // Mux31 to drive OUTPUT1 of OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT2MUXENABLE register
//
//*************************************************************************************************
#define XBAR_OUTPUT2MUXENABLE_MUX0    0x1U          // Mux0 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX1    0x2U          // Mux1 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX2    0x4U          // Mux2 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX3    0x8U          // Mux3 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX4    0x10U         // Mux4 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX5    0x20U         // Mux5 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX6    0x40U         // Mux6 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX7    0x80U         // Mux7 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX8    0x100U        // Mux8 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX9    0x200U        // Mux9 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX10   0x400U        // Mux10 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX11   0x800U        // Mux11 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX12   0x1000U       // Mux12 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX13   0x2000U       // Mux13 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX14   0x4000U       // Mux14 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX15   0x8000U       // Mux15 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX16   0x10000U      // Mux16 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX17   0x20000U      // Mux17 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX18   0x40000U      // Mux18 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX19   0x80000U      // Mux19 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX20   0x100000U     // Mux20 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX21   0x200000U     // Mux21 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX22   0x400000U     // Mux22 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX23   0x800000U     // Mux23 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX24   0x1000000U    // Mux24 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX25   0x2000000U    // Mux25 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX26   0x4000000U    // Mux26 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX27   0x8000000U    // Mux27 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX28   0x10000000U   // Mux28 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX29   0x20000000U   // Mux29 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX30   0x40000000U   // Mux30 to drive OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUT2MUXENABLE_MUX31   0x80000000U   // Mux31 to drive OUTPUT2 of OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT3MUXENABLE register
//
//*************************************************************************************************
#define XBAR_OUTPUT3MUXENABLE_MUX0    0x1U          // Mux0 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX1    0x2U          // Mux1 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX2    0x4U          // Mux2 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX3    0x8U          // Mux3 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX4    0x10U         // Mux4 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX5    0x20U         // Mux5 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX6    0x40U         // Mux6 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX7    0x80U         // Mux7 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX8    0x100U        // Mux8 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX9    0x200U        // Mux9 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX10   0x400U        // Mux10 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX11   0x800U        // Mux11 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX12   0x1000U       // Mux12 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX13   0x2000U       // Mux13 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX14   0x4000U       // Mux14 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX15   0x8000U       // Mux15 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX16   0x10000U      // Mux16 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX17   0x20000U      // Mux17 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX18   0x40000U      // Mux18 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX19   0x80000U      // Mux19 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX20   0x100000U     // Mux20 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX21   0x200000U     // Mux21 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX22   0x400000U     // Mux22 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX23   0x800000U     // Mux23 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX24   0x1000000U    // Mux24 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX25   0x2000000U    // Mux25 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX26   0x4000000U    // Mux26 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX27   0x8000000U    // Mux27 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX28   0x10000000U   // Mux28 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX29   0x20000000U   // Mux29 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX30   0x40000000U   // Mux30 to drive OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUT3MUXENABLE_MUX31   0x80000000U   // Mux31 to drive OUTPUT3 of OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT4MUXENABLE register
//
//*************************************************************************************************
#define XBAR_OUTPUT4MUXENABLE_MUX0    0x1U          // Mux0 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX1    0x2U          // Mux1 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX2    0x4U          // Mux2 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX3    0x8U          // Mux3 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX4    0x10U         // Mux4 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX5    0x20U         // Mux5 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX6    0x40U         // Mux6 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX7    0x80U         // Mux7 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX8    0x100U        // Mux8 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX9    0x200U        // Mux9 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX10   0x400U        // Mux10 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX11   0x800U        // Mux11 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX12   0x1000U       // Mux12 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX13   0x2000U       // Mux13 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX14   0x4000U       // Mux14 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX15   0x8000U       // Mux15 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX16   0x10000U      // Mux16 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX17   0x20000U      // Mux17 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX18   0x40000U      // Mux18 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX19   0x80000U      // Mux19 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX20   0x100000U     // Mux20 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX21   0x200000U     // Mux21 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX22   0x400000U     // Mux22 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX23   0x800000U     // Mux23 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX24   0x1000000U    // Mux24 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX25   0x2000000U    // Mux25 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX26   0x4000000U    // Mux26 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX27   0x8000000U    // Mux27 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX28   0x10000000U   // Mux28 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX29   0x20000000U   // Mux29 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX30   0x40000000U   // Mux30 to drive OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUT4MUXENABLE_MUX31   0x80000000U   // Mux31 to drive OUTPUT4 of OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT5MUXENABLE register
//
//*************************************************************************************************
#define XBAR_OUTPUT5MUXENABLE_MUX0    0x1U          // Mux0 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX1    0x2U          // Mux1 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX2    0x4U          // Mux2 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX3    0x8U          // Mux3 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX4    0x10U         // Mux4 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX5    0x20U         // Mux5 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX6    0x40U         // Mux6 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX7    0x80U         // Mux7 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX8    0x100U        // Mux8 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX9    0x200U        // Mux9 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX10   0x400U        // Mux10 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX11   0x800U        // Mux11 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX12   0x1000U       // Mux12 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX13   0x2000U       // Mux13 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX14   0x4000U       // Mux14 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX15   0x8000U       // Mux15 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX16   0x10000U      // Mux16 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX17   0x20000U      // Mux17 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX18   0x40000U      // Mux18 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX19   0x80000U      // Mux19 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX20   0x100000U     // Mux20 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX21   0x200000U     // Mux21 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX22   0x400000U     // Mux22 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX23   0x800000U     // Mux23 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX24   0x1000000U    // Mux24 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX25   0x2000000U    // Mux25 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX26   0x4000000U    // Mux26 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX27   0x8000000U    // Mux27 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX28   0x10000000U   // Mux28 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX29   0x20000000U   // Mux29 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX30   0x40000000U   // Mux30 to drive OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUT5MUXENABLE_MUX31   0x80000000U   // Mux31 to drive OUTPUT5 of OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT6MUXENABLE register
//
//*************************************************************************************************
#define XBAR_OUTPUT6MUXENABLE_MUX0    0x1U          // Mux0 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX1    0x2U          // Mux1 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX2    0x4U          // Mux2 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX3    0x8U          // Mux3 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX4    0x10U         // Mux4 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX5    0x20U         // Mux5 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX6    0x40U         // Mux6 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX7    0x80U         // Mux7 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX8    0x100U        // Mux8 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX9    0x200U        // Mux9 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX10   0x400U        // Mux10 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX11   0x800U        // Mux11 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX12   0x1000U       // Mux12 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX13   0x2000U       // Mux13 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX14   0x4000U       // Mux14 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX15   0x8000U       // Mux15 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX16   0x10000U      // Mux16 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX17   0x20000U      // Mux17 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX18   0x40000U      // Mux18 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX19   0x80000U      // Mux19 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX20   0x100000U     // Mux20 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX21   0x200000U     // Mux21 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX22   0x400000U     // Mux22 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX23   0x800000U     // Mux23 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX24   0x1000000U    // Mux24 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX25   0x2000000U    // Mux25 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX26   0x4000000U    // Mux26 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX27   0x8000000U    // Mux27 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX28   0x10000000U   // Mux28 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX29   0x20000000U   // Mux29 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX30   0x40000000U   // Mux30 to drive OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUT6MUXENABLE_MUX31   0x80000000U   // Mux31 to drive OUTPUT6 of OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT7MUXENABLE register
//
//*************************************************************************************************
#define XBAR_OUTPUT7MUXENABLE_MUX0    0x1U          // Mux0 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX1    0x2U          // Mux1 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX2    0x4U          // Mux2 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX3    0x8U          // Mux3 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX4    0x10U         // Mux4 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX5    0x20U         // Mux5 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX6    0x40U         // Mux6 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX7    0x80U         // Mux7 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX8    0x100U        // Mux8 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX9    0x200U        // Mux9 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX10   0x400U        // Mux10 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX11   0x800U        // Mux11 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX12   0x1000U       // Mux12 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX13   0x2000U       // Mux13 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX14   0x4000U       // Mux14 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX15   0x8000U       // Mux15 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX16   0x10000U      // Mux16 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX17   0x20000U      // Mux17 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX18   0x40000U      // Mux18 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX19   0x80000U      // Mux19 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX20   0x100000U     // Mux20 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX21   0x200000U     // Mux21 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX22   0x400000U     // Mux22 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX23   0x800000U     // Mux23 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX24   0x1000000U    // Mux24 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX25   0x2000000U    // Mux25 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX26   0x4000000U    // Mux26 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX27   0x8000000U    // Mux27 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX28   0x10000000U   // Mux28 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX29   0x20000000U   // Mux29 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX30   0x40000000U   // Mux30 to drive OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUT7MUXENABLE_MUX31   0x80000000U   // Mux31 to drive OUTPUT7 of OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUT8MUXENABLE register
//
//*************************************************************************************************
#define XBAR_OUTPUT8MUXENABLE_MUX0    0x1U          // Mux0 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX1    0x2U          // Mux1 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX2    0x4U          // Mux2 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX3    0x8U          // Mux3 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX4    0x10U         // Mux4 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX5    0x20U         // Mux5 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX6    0x40U         // Mux6 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX7    0x80U         // Mux7 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX8    0x100U        // Mux8 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX9    0x200U        // Mux9 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX10   0x400U        // Mux10 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX11   0x800U        // Mux11 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX12   0x1000U       // Mux12 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX13   0x2000U       // Mux13 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX14   0x4000U       // Mux14 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX15   0x8000U       // Mux15 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX16   0x10000U      // Mux16 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX17   0x20000U      // Mux17 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX18   0x40000U      // Mux18 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX19   0x80000U      // Mux19 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX20   0x100000U     // Mux20 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX21   0x200000U     // Mux21 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX22   0x400000U     // Mux22 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX23   0x800000U     // Mux23 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX24   0x1000000U    // Mux24 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX25   0x2000000U    // Mux25 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX26   0x4000000U    // Mux26 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX27   0x8000000U    // Mux27 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX28   0x10000000U   // Mux28 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX29   0x20000000U   // Mux29 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX30   0x40000000U   // Mux30 to drive OUTPUT8 of OUTPUT-XBAR
#define XBAR_OUTPUT8MUXENABLE_MUX31   0x80000000U   // Mux31 to drive OUTPUT8 of OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUTLATCH register
//
//*************************************************************************************************
#define XBAR_OUTPUTLATCH_OUTPUT1   0x1U    // Records the OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCH_OUTPUT2   0x2U    // Records the OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCH_OUTPUT3   0x4U    // Records the OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCH_OUTPUT4   0x8U    // Records the OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCH_OUTPUT5   0x10U   // Records the OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCH_OUTPUT6   0x20U   // Records the OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCH_OUTPUT7   0x40U   // Records the OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCH_OUTPUT8   0x80U   // Records the OUTPUT8 of OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUTLATCHCLR register
//
//*************************************************************************************************
#define XBAR_OUTPUTLATCHCLR_OUTPUT1   0x1U    // Clears the Output-Latch for OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCHCLR_OUTPUT2   0x2U    // Clears the Output-Latch for OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCHCLR_OUTPUT3   0x4U    // Clears the Output-Latch for OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCHCLR_OUTPUT4   0x8U    // Clears the Output-Latch for OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCHCLR_OUTPUT5   0x10U   // Clears the Output-Latch for OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCHCLR_OUTPUT6   0x20U   // Clears the Output-Latch for OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCHCLR_OUTPUT7   0x40U   // Clears the Output-Latch for OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCHCLR_OUTPUT8   0x80U   // Clears the Output-Latch for OUTPUT8 of OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUTLATCHFRC register
//
//*************************************************************************************************
#define XBAR_OUTPUTLATCHFRC_OUTPUT1   0x1U    // Sets the Output-Latch for OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCHFRC_OUTPUT2   0x2U    // Sets the Output-Latch for OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCHFRC_OUTPUT3   0x4U    // Sets the Output-Latch for OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCHFRC_OUTPUT4   0x8U    // Sets the Output-Latch for OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCHFRC_OUTPUT5   0x10U   // Sets the Output-Latch for OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCHFRC_OUTPUT6   0x20U   // Sets the Output-Latch for OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCHFRC_OUTPUT7   0x40U   // Sets the Output-Latch for OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUTLATCHFRC_OUTPUT8   0x80U   // Sets the Output-Latch for OUTPUT8 of OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUTLATCHENABLE register
//
//*************************************************************************************************
#define XBAR_OUTPUTLATCHENABLE_OUTPUT1   0x1U    // Selects the output latch to drive OUTPUT1 for
                                                 // OUTPUT-XBAR
#define XBAR_OUTPUTLATCHENABLE_OUTPUT2   0x2U    // Selects the output latch to drive OUTPUT2 for
                                                 // OUTPUT-XBAR
#define XBAR_OUTPUTLATCHENABLE_OUTPUT3   0x4U    // Selects the output latch to drive OUTPUT3 for
                                                 // OUTPUT-XBAR
#define XBAR_OUTPUTLATCHENABLE_OUTPUT4   0x8U    // Selects the output latch to drive OUTPUT4 for
                                                 // OUTPUT-XBAR
#define XBAR_OUTPUTLATCHENABLE_OUTPUT5   0x10U   // Selects the output latch to drive OUTPUT5 for
                                                 // OUTPUT-XBAR
#define XBAR_OUTPUTLATCHENABLE_OUTPUT6   0x20U   // Selects the output latch to drive OUTPUT6 for
                                                 // OUTPUT-XBAR
#define XBAR_OUTPUTLATCHENABLE_OUTPUT7   0x40U   // Selects the output latch to drive OUTPUT7 for
                                                 // OUTPUT-XBAR
#define XBAR_OUTPUTLATCHENABLE_OUTPUT8   0x80U   // Selects the output latch to drive OUTPUT8 for
                                                 // OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUTINV register
//
//*************************************************************************************************
#define XBAR_OUTPUTINV_OUTPUT1   0x1U    // Selects polarity for OUTPUT1 of OUTPUT-XBAR
#define XBAR_OUTPUTINV_OUTPUT2   0x2U    // Selects polarity for OUTPUT2 of OUTPUT-XBAR
#define XBAR_OUTPUTINV_OUTPUT3   0x4U    // Selects polarity for OUTPUT3 of OUTPUT-XBAR
#define XBAR_OUTPUTINV_OUTPUT4   0x8U    // Selects polarity for OUTPUT4 of OUTPUT-XBAR
#define XBAR_OUTPUTINV_OUTPUT5   0x10U   // Selects polarity for OUTPUT5 of OUTPUT-XBAR
#define XBAR_OUTPUTINV_OUTPUT6   0x20U   // Selects polarity for OUTPUT6 of OUTPUT-XBAR
#define XBAR_OUTPUTINV_OUTPUT7   0x40U   // Selects polarity for OUTPUT7 of OUTPUT-XBAR
#define XBAR_OUTPUTINV_OUTPUT8   0x80U   // Selects polarity for OUTPUT8 of OUTPUT-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the OUTPUTLOCK register
//
//*************************************************************************************************
#define XBAR_OUTPUTLOCK_LOCK    0x1U          // Locks the configuration for OUTPUT-XBAR
#define XBAR_OUTPUTLOCK_KEY_S   16U
#define XBAR_OUTPUTLOCK_KEY_M   0xFFFF0000U   // Write Protection KEY



#endif
