//###########################################################################
//
// FILE:    hw_epwmxbar.h
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

#ifndef HW_EPWMXBAR_H
#define HW_EPWMXBAR_H

//*************************************************************************************************
//
// The following are defines for the XBAR register offsets
//
//*************************************************************************************************
#define XBAR_O_TRIP4MUX0TO15CFG     0x0U    // ePWM XBAR Mux Configuration for TRIP4
#define XBAR_O_TRIP4MUX16TO31CFG    0x2U    // ePWM XBAR Mux Configuration for TRIP4
#define XBAR_O_TRIP5MUX0TO15CFG     0x4U    // ePWM XBAR Mux Configuration for TRIP5
#define XBAR_O_TRIP5MUX16TO31CFG    0x6U    // ePWM XBAR Mux Configuration for TRIP5
#define XBAR_O_TRIP7MUX0TO15CFG     0x8U    // ePWM XBAR Mux Configuration for TRIP7
#define XBAR_O_TRIP7MUX16TO31CFG    0xAU    // ePWM XBAR Mux Configuration for TRIP7
#define XBAR_O_TRIP8MUX0TO15CFG     0xCU    // ePWM XBAR Mux Configuration for TRIP8
#define XBAR_O_TRIP8MUX16TO31CFG    0xEU    // ePWM XBAR Mux Configuration for TRIP8
#define XBAR_O_TRIP9MUX0TO15CFG     0x10U   // ePWM XBAR Mux Configuration for TRIP9
#define XBAR_O_TRIP9MUX16TO31CFG    0x12U   // ePWM XBAR Mux Configuration for TRIP9
#define XBAR_O_TRIP10MUX0TO15CFG    0x14U   // ePWM XBAR Mux Configuration for TRIP10
#define XBAR_O_TRIP10MUX16TO31CFG   0x16U   // ePWM XBAR Mux Configuration for TRIP10
#define XBAR_O_TRIP11MUX0TO15CFG    0x18U   // ePWM XBAR Mux Configuration for TRIP11
#define XBAR_O_TRIP11MUX16TO31CFG   0x1AU   // ePWM XBAR Mux Configuration for TRIP11
#define XBAR_O_TRIP12MUX0TO15CFG    0x1CU   // ePWM XBAR Mux Configuration for TRIP12
#define XBAR_O_TRIP12MUX16TO31CFG   0x1EU   // ePWM XBAR Mux Configuration for TRIP12
#define XBAR_O_TRIP4MUXENABLE       0x20U   // ePWM XBAR Mux Enable for TRIP4
#define XBAR_O_TRIP5MUXENABLE       0x22U   // ePWM XBAR Mux Enable for TRIP5
#define XBAR_O_TRIP7MUXENABLE       0x24U   // ePWM XBAR Mux Enable for TRIP7
#define XBAR_O_TRIP8MUXENABLE       0x26U   // ePWM XBAR Mux Enable for TRIP8
#define XBAR_O_TRIP9MUXENABLE       0x28U   // ePWM XBAR Mux Enable for TRIP9
#define XBAR_O_TRIP10MUXENABLE      0x2AU   // ePWM XBAR Mux Enable for TRIP10
#define XBAR_O_TRIP11MUXENABLE      0x2CU   // ePWM XBAR Mux Enable for TRIP11
#define XBAR_O_TRIP12MUXENABLE      0x2EU   // ePWM XBAR Mux Enable for TRIP12
#define XBAR_O_TRIPOUTINV           0x38U   // ePWM XBAR Output Inversion Register
#define XBAR_O_TRIPLOCK             0x3EU   // ePWM XBAR Configuration Lock register


//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP4MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_TRIP4MUX0TO15CFG_MUX0_S    0U
#define XBAR_TRIP4MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUX0TO15CFG_MUX1_S    2U
#define XBAR_TRIP4MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUX0TO15CFG_MUX2_S    4U
#define XBAR_TRIP4MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUX0TO15CFG_MUX3_S    6U
#define XBAR_TRIP4MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUX0TO15CFG_MUX4_S    8U
#define XBAR_TRIP4MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUX0TO15CFG_MUX5_S    10U
#define XBAR_TRIP4MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUX0TO15CFG_MUX6_S    12U
#define XBAR_TRIP4MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUX0TO15CFG_MUX7_S    14U
#define XBAR_TRIP4MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUX0TO15CFG_MUX8_S    16U
#define XBAR_TRIP4MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUX0TO15CFG_MUX9_S    18U
#define XBAR_TRIP4MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUX0TO15CFG_MUX10_S   20U
#define XBAR_TRIP4MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUX0TO15CFG_MUX11_S   22U
#define XBAR_TRIP4MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUX0TO15CFG_MUX12_S   24U
#define XBAR_TRIP4MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUX0TO15CFG_MUX13_S   26U
#define XBAR_TRIP4MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUX0TO15CFG_MUX14_S   28U
#define XBAR_TRIP4MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUX0TO15CFG_MUX15_S   30U
#define XBAR_TRIP4MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for TRIP4 of EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP4MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_TRIP4MUX16TO31CFG_MUX16_S   0U
#define XBAR_TRIP4MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for TRIP4 of
                                                       // EPWM-XBAR
#define XBAR_TRIP4MUX16TO31CFG_MUX17_S   2U
#define XBAR_TRIP4MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for TRIP4 of
                                                       // EPWM-XBAR
#define XBAR_TRIP4MUX16TO31CFG_MUX18_S   4U
#define XBAR_TRIP4MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for TRIP4 of
                                                       // EPWM-XBAR
#define XBAR_TRIP4MUX16TO31CFG_MUX19_S   6U
#define XBAR_TRIP4MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for TRIP4 of
                                                       // EPWM-XBAR
#define XBAR_TRIP4MUX16TO31CFG_MUX20_S   8U
#define XBAR_TRIP4MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for TRIP4 of
                                                       // EPWM-XBAR
#define XBAR_TRIP4MUX16TO31CFG_MUX21_S   10U
#define XBAR_TRIP4MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for TRIP4 of
                                                       // EPWM-XBAR
#define XBAR_TRIP4MUX16TO31CFG_MUX22_S   12U
#define XBAR_TRIP4MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for TRIP4 of
                                                       // EPWM-XBAR
#define XBAR_TRIP4MUX16TO31CFG_MUX23_S   14U
#define XBAR_TRIP4MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for TRIP4 of
                                                       // EPWM-XBAR
#define XBAR_TRIP4MUX16TO31CFG_MUX24_S   16U
#define XBAR_TRIP4MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for TRIP4 of
                                                       // EPWM-XBAR
#define XBAR_TRIP4MUX16TO31CFG_MUX25_S   18U
#define XBAR_TRIP4MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for TRIP4 of
                                                       // EPWM-XBAR
#define XBAR_TRIP4MUX16TO31CFG_MUX26_S   20U
#define XBAR_TRIP4MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for TRIP4 of
                                                       // EPWM-XBAR
#define XBAR_TRIP4MUX16TO31CFG_MUX27_S   22U
#define XBAR_TRIP4MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for TRIP4 of
                                                       // EPWM-XBAR
#define XBAR_TRIP4MUX16TO31CFG_MUX28_S   24U
#define XBAR_TRIP4MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for TRIP4 of
                                                       // EPWM-XBAR
#define XBAR_TRIP4MUX16TO31CFG_MUX29_S   26U
#define XBAR_TRIP4MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for TRIP4 of
                                                       // EPWM-XBAR
#define XBAR_TRIP4MUX16TO31CFG_MUX30_S   28U
#define XBAR_TRIP4MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for TRIP4 of
                                                       // EPWM-XBAR
#define XBAR_TRIP4MUX16TO31CFG_MUX31_S   30U
#define XBAR_TRIP4MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for TRIP4 of
                                                       // EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP5MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_TRIP5MUX0TO15CFG_MUX0_S    0U
#define XBAR_TRIP5MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUX0TO15CFG_MUX1_S    2U
#define XBAR_TRIP5MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUX0TO15CFG_MUX2_S    4U
#define XBAR_TRIP5MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUX0TO15CFG_MUX3_S    6U
#define XBAR_TRIP5MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUX0TO15CFG_MUX4_S    8U
#define XBAR_TRIP5MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUX0TO15CFG_MUX5_S    10U
#define XBAR_TRIP5MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUX0TO15CFG_MUX6_S    12U
#define XBAR_TRIP5MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUX0TO15CFG_MUX7_S    14U
#define XBAR_TRIP5MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUX0TO15CFG_MUX8_S    16U
#define XBAR_TRIP5MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUX0TO15CFG_MUX9_S    18U
#define XBAR_TRIP5MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUX0TO15CFG_MUX10_S   20U
#define XBAR_TRIP5MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUX0TO15CFG_MUX11_S   22U
#define XBAR_TRIP5MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUX0TO15CFG_MUX12_S   24U
#define XBAR_TRIP5MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUX0TO15CFG_MUX13_S   26U
#define XBAR_TRIP5MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUX0TO15CFG_MUX14_S   28U
#define XBAR_TRIP5MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUX0TO15CFG_MUX15_S   30U
#define XBAR_TRIP5MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for TRIP5 of EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP5MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_TRIP5MUX16TO31CFG_MUX16_S   0U
#define XBAR_TRIP5MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for TRIP5 of
                                                       // EPWM-XBAR
#define XBAR_TRIP5MUX16TO31CFG_MUX17_S   2U
#define XBAR_TRIP5MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for TRIP5 of
                                                       // EPWM-XBAR
#define XBAR_TRIP5MUX16TO31CFG_MUX18_S   4U
#define XBAR_TRIP5MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for TRIP5 of
                                                       // EPWM-XBAR
#define XBAR_TRIP5MUX16TO31CFG_MUX19_S   6U
#define XBAR_TRIP5MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for TRIP5 of
                                                       // EPWM-XBAR
#define XBAR_TRIP5MUX16TO31CFG_MUX20_S   8U
#define XBAR_TRIP5MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for TRIP5 of
                                                       // EPWM-XBAR
#define XBAR_TRIP5MUX16TO31CFG_MUX21_S   10U
#define XBAR_TRIP5MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for TRIP5 of
                                                       // EPWM-XBAR
#define XBAR_TRIP5MUX16TO31CFG_MUX22_S   12U
#define XBAR_TRIP5MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for TRIP5 of
                                                       // EPWM-XBAR
#define XBAR_TRIP5MUX16TO31CFG_MUX23_S   14U
#define XBAR_TRIP5MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for TRIP5 of
                                                       // EPWM-XBAR
#define XBAR_TRIP5MUX16TO31CFG_MUX24_S   16U
#define XBAR_TRIP5MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for TRIP5 of
                                                       // EPWM-XBAR
#define XBAR_TRIP5MUX16TO31CFG_MUX25_S   18U
#define XBAR_TRIP5MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for TRIP5 of
                                                       // EPWM-XBAR
#define XBAR_TRIP5MUX16TO31CFG_MUX26_S   20U
#define XBAR_TRIP5MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for TRIP5 of
                                                       // EPWM-XBAR
#define XBAR_TRIP5MUX16TO31CFG_MUX27_S   22U
#define XBAR_TRIP5MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for TRIP5 of
                                                       // EPWM-XBAR
#define XBAR_TRIP5MUX16TO31CFG_MUX28_S   24U
#define XBAR_TRIP5MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for TRIP5 of
                                                       // EPWM-XBAR
#define XBAR_TRIP5MUX16TO31CFG_MUX29_S   26U
#define XBAR_TRIP5MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for TRIP5 of
                                                       // EPWM-XBAR
#define XBAR_TRIP5MUX16TO31CFG_MUX30_S   28U
#define XBAR_TRIP5MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for TRIP5 of
                                                       // EPWM-XBAR
#define XBAR_TRIP5MUX16TO31CFG_MUX31_S   30U
#define XBAR_TRIP5MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for TRIP5 of
                                                       // EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP7MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_TRIP7MUX0TO15CFG_MUX0_S    0U
#define XBAR_TRIP7MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUX0TO15CFG_MUX1_S    2U
#define XBAR_TRIP7MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUX0TO15CFG_MUX2_S    4U
#define XBAR_TRIP7MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUX0TO15CFG_MUX3_S    6U
#define XBAR_TRIP7MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUX0TO15CFG_MUX4_S    8U
#define XBAR_TRIP7MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUX0TO15CFG_MUX5_S    10U
#define XBAR_TRIP7MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUX0TO15CFG_MUX6_S    12U
#define XBAR_TRIP7MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUX0TO15CFG_MUX7_S    14U
#define XBAR_TRIP7MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUX0TO15CFG_MUX8_S    16U
#define XBAR_TRIP7MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUX0TO15CFG_MUX9_S    18U
#define XBAR_TRIP7MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUX0TO15CFG_MUX10_S   20U
#define XBAR_TRIP7MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUX0TO15CFG_MUX11_S   22U
#define XBAR_TRIP7MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUX0TO15CFG_MUX12_S   24U
#define XBAR_TRIP7MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUX0TO15CFG_MUX13_S   26U
#define XBAR_TRIP7MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUX0TO15CFG_MUX14_S   28U
#define XBAR_TRIP7MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUX0TO15CFG_MUX15_S   30U
#define XBAR_TRIP7MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for TRIP7 of EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP7MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_TRIP7MUX16TO31CFG_MUX16_S   0U
#define XBAR_TRIP7MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for TRIP7 of
                                                       // EPWM-XBAR
#define XBAR_TRIP7MUX16TO31CFG_MUX17_S   2U
#define XBAR_TRIP7MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for TRIP7 of
                                                       // EPWM-XBAR
#define XBAR_TRIP7MUX16TO31CFG_MUX18_S   4U
#define XBAR_TRIP7MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for TRIP7 of
                                                       // EPWM-XBAR
#define XBAR_TRIP7MUX16TO31CFG_MUX19_S   6U
#define XBAR_TRIP7MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for TRIP7 of
                                                       // EPWM-XBAR
#define XBAR_TRIP7MUX16TO31CFG_MUX20_S   8U
#define XBAR_TRIP7MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for TRIP7 of
                                                       // EPWM-XBAR
#define XBAR_TRIP7MUX16TO31CFG_MUX21_S   10U
#define XBAR_TRIP7MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for TRIP7 of
                                                       // EPWM-XBAR
#define XBAR_TRIP7MUX16TO31CFG_MUX22_S   12U
#define XBAR_TRIP7MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for TRIP7 of
                                                       // EPWM-XBAR
#define XBAR_TRIP7MUX16TO31CFG_MUX23_S   14U
#define XBAR_TRIP7MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for TRIP7 of
                                                       // EPWM-XBAR
#define XBAR_TRIP7MUX16TO31CFG_MUX24_S   16U
#define XBAR_TRIP7MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for TRIP7 of
                                                       // EPWM-XBAR
#define XBAR_TRIP7MUX16TO31CFG_MUX25_S   18U
#define XBAR_TRIP7MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for TRIP7 of
                                                       // EPWM-XBAR
#define XBAR_TRIP7MUX16TO31CFG_MUX26_S   20U
#define XBAR_TRIP7MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for TRIP7 of
                                                       // EPWM-XBAR
#define XBAR_TRIP7MUX16TO31CFG_MUX27_S   22U
#define XBAR_TRIP7MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for TRIP7 of
                                                       // EPWM-XBAR
#define XBAR_TRIP7MUX16TO31CFG_MUX28_S   24U
#define XBAR_TRIP7MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for TRIP7 of
                                                       // EPWM-XBAR
#define XBAR_TRIP7MUX16TO31CFG_MUX29_S   26U
#define XBAR_TRIP7MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for TRIP7 of
                                                       // EPWM-XBAR
#define XBAR_TRIP7MUX16TO31CFG_MUX30_S   28U
#define XBAR_TRIP7MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for TRIP7 of
                                                       // EPWM-XBAR
#define XBAR_TRIP7MUX16TO31CFG_MUX31_S   30U
#define XBAR_TRIP7MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for TRIP7 of
                                                       // EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP8MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_TRIP8MUX0TO15CFG_MUX0_S    0U
#define XBAR_TRIP8MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUX0TO15CFG_MUX1_S    2U
#define XBAR_TRIP8MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUX0TO15CFG_MUX2_S    4U
#define XBAR_TRIP8MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUX0TO15CFG_MUX3_S    6U
#define XBAR_TRIP8MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUX0TO15CFG_MUX4_S    8U
#define XBAR_TRIP8MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUX0TO15CFG_MUX5_S    10U
#define XBAR_TRIP8MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUX0TO15CFG_MUX6_S    12U
#define XBAR_TRIP8MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUX0TO15CFG_MUX7_S    14U
#define XBAR_TRIP8MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUX0TO15CFG_MUX8_S    16U
#define XBAR_TRIP8MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUX0TO15CFG_MUX9_S    18U
#define XBAR_TRIP8MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUX0TO15CFG_MUX10_S   20U
#define XBAR_TRIP8MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUX0TO15CFG_MUX11_S   22U
#define XBAR_TRIP8MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUX0TO15CFG_MUX12_S   24U
#define XBAR_TRIP8MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUX0TO15CFG_MUX13_S   26U
#define XBAR_TRIP8MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUX0TO15CFG_MUX14_S   28U
#define XBAR_TRIP8MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUX0TO15CFG_MUX15_S   30U
#define XBAR_TRIP8MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for TRIP8 of EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP8MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_TRIP8MUX16TO31CFG_MUX16_S   0U
#define XBAR_TRIP8MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for TRIP8 of
                                                       // EPWM-XBAR
#define XBAR_TRIP8MUX16TO31CFG_MUX17_S   2U
#define XBAR_TRIP8MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for TRIP8 of
                                                       // EPWM-XBAR
#define XBAR_TRIP8MUX16TO31CFG_MUX18_S   4U
#define XBAR_TRIP8MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for TRIP8 of
                                                       // EPWM-XBAR
#define XBAR_TRIP8MUX16TO31CFG_MUX19_S   6U
#define XBAR_TRIP8MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for TRIP8 of
                                                       // EPWM-XBAR
#define XBAR_TRIP8MUX16TO31CFG_MUX20_S   8U
#define XBAR_TRIP8MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for TRIP8 of
                                                       // EPWM-XBAR
#define XBAR_TRIP8MUX16TO31CFG_MUX21_S   10U
#define XBAR_TRIP8MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for TRIP8 of
                                                       // EPWM-XBAR
#define XBAR_TRIP8MUX16TO31CFG_MUX22_S   12U
#define XBAR_TRIP8MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for TRIP8 of
                                                       // EPWM-XBAR
#define XBAR_TRIP8MUX16TO31CFG_MUX23_S   14U
#define XBAR_TRIP8MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for TRIP8 of
                                                       // EPWM-XBAR
#define XBAR_TRIP8MUX16TO31CFG_MUX24_S   16U
#define XBAR_TRIP8MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for TRIP8 of
                                                       // EPWM-XBAR
#define XBAR_TRIP8MUX16TO31CFG_MUX25_S   18U
#define XBAR_TRIP8MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for TRIP8 of
                                                       // EPWM-XBAR
#define XBAR_TRIP8MUX16TO31CFG_MUX26_S   20U
#define XBAR_TRIP8MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for TRIP8 of
                                                       // EPWM-XBAR
#define XBAR_TRIP8MUX16TO31CFG_MUX27_S   22U
#define XBAR_TRIP8MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for TRIP8 of
                                                       // EPWM-XBAR
#define XBAR_TRIP8MUX16TO31CFG_MUX28_S   24U
#define XBAR_TRIP8MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for TRIP8 of
                                                       // EPWM-XBAR
#define XBAR_TRIP8MUX16TO31CFG_MUX29_S   26U
#define XBAR_TRIP8MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for TRIP8 of
                                                       // EPWM-XBAR
#define XBAR_TRIP8MUX16TO31CFG_MUX30_S   28U
#define XBAR_TRIP8MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for TRIP8 of
                                                       // EPWM-XBAR
#define XBAR_TRIP8MUX16TO31CFG_MUX31_S   30U
#define XBAR_TRIP8MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for TRIP8 of
                                                       // EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP9MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_TRIP9MUX0TO15CFG_MUX0_S    0U
#define XBAR_TRIP9MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUX0TO15CFG_MUX1_S    2U
#define XBAR_TRIP9MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUX0TO15CFG_MUX2_S    4U
#define XBAR_TRIP9MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUX0TO15CFG_MUX3_S    6U
#define XBAR_TRIP9MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUX0TO15CFG_MUX4_S    8U
#define XBAR_TRIP9MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUX0TO15CFG_MUX5_S    10U
#define XBAR_TRIP9MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUX0TO15CFG_MUX6_S    12U
#define XBAR_TRIP9MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUX0TO15CFG_MUX7_S    14U
#define XBAR_TRIP9MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUX0TO15CFG_MUX8_S    16U
#define XBAR_TRIP9MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUX0TO15CFG_MUX9_S    18U
#define XBAR_TRIP9MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUX0TO15CFG_MUX10_S   20U
#define XBAR_TRIP9MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUX0TO15CFG_MUX11_S   22U
#define XBAR_TRIP9MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUX0TO15CFG_MUX12_S   24U
#define XBAR_TRIP9MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUX0TO15CFG_MUX13_S   26U
#define XBAR_TRIP9MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUX0TO15CFG_MUX14_S   28U
#define XBAR_TRIP9MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUX0TO15CFG_MUX15_S   30U
#define XBAR_TRIP9MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for TRIP9 of EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP9MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_TRIP9MUX16TO31CFG_MUX16_S   0U
#define XBAR_TRIP9MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for TRIP9 of
                                                       // EPWM-XBAR
#define XBAR_TRIP9MUX16TO31CFG_MUX17_S   2U
#define XBAR_TRIP9MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for TRIP9 of
                                                       // EPWM-XBAR
#define XBAR_TRIP9MUX16TO31CFG_MUX18_S   4U
#define XBAR_TRIP9MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for TRIP9 of
                                                       // EPWM-XBAR
#define XBAR_TRIP9MUX16TO31CFG_MUX19_S   6U
#define XBAR_TRIP9MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for TRIP9 of
                                                       // EPWM-XBAR
#define XBAR_TRIP9MUX16TO31CFG_MUX20_S   8U
#define XBAR_TRIP9MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for TRIP9 of
                                                       // EPWM-XBAR
#define XBAR_TRIP9MUX16TO31CFG_MUX21_S   10U
#define XBAR_TRIP9MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for TRIP9 of
                                                       // EPWM-XBAR
#define XBAR_TRIP9MUX16TO31CFG_MUX22_S   12U
#define XBAR_TRIP9MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for TRIP9 of
                                                       // EPWM-XBAR
#define XBAR_TRIP9MUX16TO31CFG_MUX23_S   14U
#define XBAR_TRIP9MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for TRIP9 of
                                                       // EPWM-XBAR
#define XBAR_TRIP9MUX16TO31CFG_MUX24_S   16U
#define XBAR_TRIP9MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for TRIP9 of
                                                       // EPWM-XBAR
#define XBAR_TRIP9MUX16TO31CFG_MUX25_S   18U
#define XBAR_TRIP9MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for TRIP9 of
                                                       // EPWM-XBAR
#define XBAR_TRIP9MUX16TO31CFG_MUX26_S   20U
#define XBAR_TRIP9MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for TRIP9 of
                                                       // EPWM-XBAR
#define XBAR_TRIP9MUX16TO31CFG_MUX27_S   22U
#define XBAR_TRIP9MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for TRIP9 of
                                                       // EPWM-XBAR
#define XBAR_TRIP9MUX16TO31CFG_MUX28_S   24U
#define XBAR_TRIP9MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for TRIP9 of
                                                       // EPWM-XBAR
#define XBAR_TRIP9MUX16TO31CFG_MUX29_S   26U
#define XBAR_TRIP9MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for TRIP9 of
                                                       // EPWM-XBAR
#define XBAR_TRIP9MUX16TO31CFG_MUX30_S   28U
#define XBAR_TRIP9MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for TRIP9 of
                                                       // EPWM-XBAR
#define XBAR_TRIP9MUX16TO31CFG_MUX31_S   30U
#define XBAR_TRIP9MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for TRIP9 of
                                                       // EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP10MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_TRIP10MUX0TO15CFG_MUX0_S    0U
#define XBAR_TRIP10MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for TRIP10 of
                                                       // EPWM-XBAR
#define XBAR_TRIP10MUX0TO15CFG_MUX1_S    2U
#define XBAR_TRIP10MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for TRIP10 of
                                                       // EPWM-XBAR
#define XBAR_TRIP10MUX0TO15CFG_MUX2_S    4U
#define XBAR_TRIP10MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for TRIP10 of
                                                       // EPWM-XBAR
#define XBAR_TRIP10MUX0TO15CFG_MUX3_S    6U
#define XBAR_TRIP10MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for TRIP10 of
                                                       // EPWM-XBAR
#define XBAR_TRIP10MUX0TO15CFG_MUX4_S    8U
#define XBAR_TRIP10MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for TRIP10 of
                                                       // EPWM-XBAR
#define XBAR_TRIP10MUX0TO15CFG_MUX5_S    10U
#define XBAR_TRIP10MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for TRIP10 of
                                                       // EPWM-XBAR
#define XBAR_TRIP10MUX0TO15CFG_MUX6_S    12U
#define XBAR_TRIP10MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for TRIP10 of
                                                       // EPWM-XBAR
#define XBAR_TRIP10MUX0TO15CFG_MUX7_S    14U
#define XBAR_TRIP10MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for TRIP10 of
                                                       // EPWM-XBAR
#define XBAR_TRIP10MUX0TO15CFG_MUX8_S    16U
#define XBAR_TRIP10MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for TRIP10 of
                                                       // EPWM-XBAR
#define XBAR_TRIP10MUX0TO15CFG_MUX9_S    18U
#define XBAR_TRIP10MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for TRIP10 of
                                                       // EPWM-XBAR
#define XBAR_TRIP10MUX0TO15CFG_MUX10_S   20U
#define XBAR_TRIP10MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for TRIP10 of
                                                       // EPWM-XBAR
#define XBAR_TRIP10MUX0TO15CFG_MUX11_S   22U
#define XBAR_TRIP10MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for TRIP10 of
                                                       // EPWM-XBAR
#define XBAR_TRIP10MUX0TO15CFG_MUX12_S   24U
#define XBAR_TRIP10MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for TRIP10 of
                                                       // EPWM-XBAR
#define XBAR_TRIP10MUX0TO15CFG_MUX13_S   26U
#define XBAR_TRIP10MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for TRIP10 of
                                                       // EPWM-XBAR
#define XBAR_TRIP10MUX0TO15CFG_MUX14_S   28U
#define XBAR_TRIP10MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for TRIP10 of
                                                       // EPWM-XBAR
#define XBAR_TRIP10MUX0TO15CFG_MUX15_S   30U
#define XBAR_TRIP10MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for TRIP10 of
                                                       // EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP10MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_TRIP10MUX16TO31CFG_MUX16_S   0U
#define XBAR_TRIP10MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for TRIP10 of
                                                        // EPWM-XBAR
#define XBAR_TRIP10MUX16TO31CFG_MUX17_S   2U
#define XBAR_TRIP10MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for TRIP10 of
                                                        // EPWM-XBAR
#define XBAR_TRIP10MUX16TO31CFG_MUX18_S   4U
#define XBAR_TRIP10MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for TRIP10 of
                                                        // EPWM-XBAR
#define XBAR_TRIP10MUX16TO31CFG_MUX19_S   6U
#define XBAR_TRIP10MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for TRIP10 of
                                                        // EPWM-XBAR
#define XBAR_TRIP10MUX16TO31CFG_MUX20_S   8U
#define XBAR_TRIP10MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for TRIP10 of
                                                        // EPWM-XBAR
#define XBAR_TRIP10MUX16TO31CFG_MUX21_S   10U
#define XBAR_TRIP10MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for TRIP10 of
                                                        // EPWM-XBAR
#define XBAR_TRIP10MUX16TO31CFG_MUX22_S   12U
#define XBAR_TRIP10MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for TRIP10 of
                                                        // EPWM-XBAR
#define XBAR_TRIP10MUX16TO31CFG_MUX23_S   14U
#define XBAR_TRIP10MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for TRIP10 of
                                                        // EPWM-XBAR
#define XBAR_TRIP10MUX16TO31CFG_MUX24_S   16U
#define XBAR_TRIP10MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for TRIP10 of
                                                        // EPWM-XBAR
#define XBAR_TRIP10MUX16TO31CFG_MUX25_S   18U
#define XBAR_TRIP10MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for TRIP10 of
                                                        // EPWM-XBAR
#define XBAR_TRIP10MUX16TO31CFG_MUX26_S   20U
#define XBAR_TRIP10MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for TRIP10 of
                                                        // EPWM-XBAR
#define XBAR_TRIP10MUX16TO31CFG_MUX27_S   22U
#define XBAR_TRIP10MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for TRIP10 of
                                                        // EPWM-XBAR
#define XBAR_TRIP10MUX16TO31CFG_MUX28_S   24U
#define XBAR_TRIP10MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for TRIP10 of
                                                        // EPWM-XBAR
#define XBAR_TRIP10MUX16TO31CFG_MUX29_S   26U
#define XBAR_TRIP10MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for TRIP10 of
                                                        // EPWM-XBAR
#define XBAR_TRIP10MUX16TO31CFG_MUX30_S   28U
#define XBAR_TRIP10MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for TRIP10 of
                                                        // EPWM-XBAR
#define XBAR_TRIP10MUX16TO31CFG_MUX31_S   30U
#define XBAR_TRIP10MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for TRIP10 of
                                                        // EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP11MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_TRIP11MUX0TO15CFG_MUX0_S    0U
#define XBAR_TRIP11MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for TRIP11 of
                                                       // EPWM-XBAR
#define XBAR_TRIP11MUX0TO15CFG_MUX1_S    2U
#define XBAR_TRIP11MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for TRIP11 of
                                                       // EPWM-XBAR
#define XBAR_TRIP11MUX0TO15CFG_MUX2_S    4U
#define XBAR_TRIP11MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for TRIP11 of
                                                       // EPWM-XBAR
#define XBAR_TRIP11MUX0TO15CFG_MUX3_S    6U
#define XBAR_TRIP11MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for TRIP11 of
                                                       // EPWM-XBAR
#define XBAR_TRIP11MUX0TO15CFG_MUX4_S    8U
#define XBAR_TRIP11MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for TRIP11 of
                                                       // EPWM-XBAR
#define XBAR_TRIP11MUX0TO15CFG_MUX5_S    10U
#define XBAR_TRIP11MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for TRIP11 of
                                                       // EPWM-XBAR
#define XBAR_TRIP11MUX0TO15CFG_MUX6_S    12U
#define XBAR_TRIP11MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for TRIP11 of
                                                       // EPWM-XBAR
#define XBAR_TRIP11MUX0TO15CFG_MUX7_S    14U
#define XBAR_TRIP11MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for TRIP11 of
                                                       // EPWM-XBAR
#define XBAR_TRIP11MUX0TO15CFG_MUX8_S    16U
#define XBAR_TRIP11MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for TRIP11 of
                                                       // EPWM-XBAR
#define XBAR_TRIP11MUX0TO15CFG_MUX9_S    18U
#define XBAR_TRIP11MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for TRIP11 of
                                                       // EPWM-XBAR
#define XBAR_TRIP11MUX0TO15CFG_MUX10_S   20U
#define XBAR_TRIP11MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for TRIP11 of
                                                       // EPWM-XBAR
#define XBAR_TRIP11MUX0TO15CFG_MUX11_S   22U
#define XBAR_TRIP11MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for TRIP11 of
                                                       // EPWM-XBAR
#define XBAR_TRIP11MUX0TO15CFG_MUX12_S   24U
#define XBAR_TRIP11MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for TRIP11 of
                                                       // EPWM-XBAR
#define XBAR_TRIP11MUX0TO15CFG_MUX13_S   26U
#define XBAR_TRIP11MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for TRIP11 of
                                                       // EPWM-XBAR
#define XBAR_TRIP11MUX0TO15CFG_MUX14_S   28U
#define XBAR_TRIP11MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for TRIP11 of
                                                       // EPWM-XBAR
#define XBAR_TRIP11MUX0TO15CFG_MUX15_S   30U
#define XBAR_TRIP11MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for TRIP11 of
                                                       // EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP11MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_TRIP11MUX16TO31CFG_MUX16_S   0U
#define XBAR_TRIP11MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for TRIP11 of
                                                        // EPWM-XBAR
#define XBAR_TRIP11MUX16TO31CFG_MUX17_S   2U
#define XBAR_TRIP11MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for TRIP11 of
                                                        // EPWM-XBAR
#define XBAR_TRIP11MUX16TO31CFG_MUX18_S   4U
#define XBAR_TRIP11MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for TRIP11 of
                                                        // EPWM-XBAR
#define XBAR_TRIP11MUX16TO31CFG_MUX19_S   6U
#define XBAR_TRIP11MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for TRIP11 of
                                                        // EPWM-XBAR
#define XBAR_TRIP11MUX16TO31CFG_MUX20_S   8U
#define XBAR_TRIP11MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for TRIP11 of
                                                        // EPWM-XBAR
#define XBAR_TRIP11MUX16TO31CFG_MUX21_S   10U
#define XBAR_TRIP11MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for TRIP11 of
                                                        // EPWM-XBAR
#define XBAR_TRIP11MUX16TO31CFG_MUX22_S   12U
#define XBAR_TRIP11MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for TRIP11 of
                                                        // EPWM-XBAR
#define XBAR_TRIP11MUX16TO31CFG_MUX23_S   14U
#define XBAR_TRIP11MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for TRIP11 of
                                                        // EPWM-XBAR
#define XBAR_TRIP11MUX16TO31CFG_MUX24_S   16U
#define XBAR_TRIP11MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for TRIP11 of
                                                        // EPWM-XBAR
#define XBAR_TRIP11MUX16TO31CFG_MUX25_S   18U
#define XBAR_TRIP11MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for TRIP11 of
                                                        // EPWM-XBAR
#define XBAR_TRIP11MUX16TO31CFG_MUX26_S   20U
#define XBAR_TRIP11MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for TRIP11 of
                                                        // EPWM-XBAR
#define XBAR_TRIP11MUX16TO31CFG_MUX27_S   22U
#define XBAR_TRIP11MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for TRIP11 of
                                                        // EPWM-XBAR
#define XBAR_TRIP11MUX16TO31CFG_MUX28_S   24U
#define XBAR_TRIP11MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for TRIP11 of
                                                        // EPWM-XBAR
#define XBAR_TRIP11MUX16TO31CFG_MUX29_S   26U
#define XBAR_TRIP11MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for TRIP11 of
                                                        // EPWM-XBAR
#define XBAR_TRIP11MUX16TO31CFG_MUX30_S   28U
#define XBAR_TRIP11MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for TRIP11 of
                                                        // EPWM-XBAR
#define XBAR_TRIP11MUX16TO31CFG_MUX31_S   30U
#define XBAR_TRIP11MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for TRIP11 of
                                                        // EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP12MUX0TO15CFG register
//
//*************************************************************************************************
#define XBAR_TRIP12MUX0TO15CFG_MUX0_S    0U
#define XBAR_TRIP12MUX0TO15CFG_MUX0_M    0x3U          // Mux0 Configuration for TRIP12 of
                                                       // EPWM-XBAR
#define XBAR_TRIP12MUX0TO15CFG_MUX1_S    2U
#define XBAR_TRIP12MUX0TO15CFG_MUX1_M    0xCU          // Mux1 Configuration for TRIP12 of
                                                       // EPWM-XBAR
#define XBAR_TRIP12MUX0TO15CFG_MUX2_S    4U
#define XBAR_TRIP12MUX0TO15CFG_MUX2_M    0x30U         // Mux2 Configuration for TRIP12 of
                                                       // EPWM-XBAR
#define XBAR_TRIP12MUX0TO15CFG_MUX3_S    6U
#define XBAR_TRIP12MUX0TO15CFG_MUX3_M    0xC0U         // Mux3 Configuration for TRIP12 of
                                                       // EPWM-XBAR
#define XBAR_TRIP12MUX0TO15CFG_MUX4_S    8U
#define XBAR_TRIP12MUX0TO15CFG_MUX4_M    0x300U        // Mux4 Configuration for TRIP12 of
                                                       // EPWM-XBAR
#define XBAR_TRIP12MUX0TO15CFG_MUX5_S    10U
#define XBAR_TRIP12MUX0TO15CFG_MUX5_M    0xC00U        // Mux5 Configuration for TRIP12 of
                                                       // EPWM-XBAR
#define XBAR_TRIP12MUX0TO15CFG_MUX6_S    12U
#define XBAR_TRIP12MUX0TO15CFG_MUX6_M    0x3000U       // Mux6 Configuration for TRIP12 of
                                                       // EPWM-XBAR
#define XBAR_TRIP12MUX0TO15CFG_MUX7_S    14U
#define XBAR_TRIP12MUX0TO15CFG_MUX7_M    0xC000U       // Mux7 Configuration for TRIP12 of
                                                       // EPWM-XBAR
#define XBAR_TRIP12MUX0TO15CFG_MUX8_S    16U
#define XBAR_TRIP12MUX0TO15CFG_MUX8_M    0x30000U      // Mux8 Configuration for TRIP12 of
                                                       // EPWM-XBAR
#define XBAR_TRIP12MUX0TO15CFG_MUX9_S    18U
#define XBAR_TRIP12MUX0TO15CFG_MUX9_M    0xC0000U      // Mux9 Configuration for TRIP12 of
                                                       // EPWM-XBAR
#define XBAR_TRIP12MUX0TO15CFG_MUX10_S   20U
#define XBAR_TRIP12MUX0TO15CFG_MUX10_M   0x300000U     // Mux10 Configuration for TRIP12 of
                                                       // EPWM-XBAR
#define XBAR_TRIP12MUX0TO15CFG_MUX11_S   22U
#define XBAR_TRIP12MUX0TO15CFG_MUX11_M   0xC00000U     // Mux11 Configuration for TRIP12 of
                                                       // EPWM-XBAR
#define XBAR_TRIP12MUX0TO15CFG_MUX12_S   24U
#define XBAR_TRIP12MUX0TO15CFG_MUX12_M   0x3000000U    // Mux12 Configuration for TRIP12 of
                                                       // EPWM-XBAR
#define XBAR_TRIP12MUX0TO15CFG_MUX13_S   26U
#define XBAR_TRIP12MUX0TO15CFG_MUX13_M   0xC000000U    // Mux13 Configuration for TRIP12 of
                                                       // EPWM-XBAR
#define XBAR_TRIP12MUX0TO15CFG_MUX14_S   28U
#define XBAR_TRIP12MUX0TO15CFG_MUX14_M   0x30000000U   // Mux14 Configuration for TRIP12 of
                                                       // EPWM-XBAR
#define XBAR_TRIP12MUX0TO15CFG_MUX15_S   30U
#define XBAR_TRIP12MUX0TO15CFG_MUX15_M   0xC0000000U   // Mux15 Configuration for TRIP12 of
                                                       // EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP12MUX16TO31CFG register
//
//*************************************************************************************************
#define XBAR_TRIP12MUX16TO31CFG_MUX16_S   0U
#define XBAR_TRIP12MUX16TO31CFG_MUX16_M   0x3U          // Mux16 Configuration for TRIP12 of
                                                        // EPWM-XBAR
#define XBAR_TRIP12MUX16TO31CFG_MUX17_S   2U
#define XBAR_TRIP12MUX16TO31CFG_MUX17_M   0xCU          // Mux17 Configuration for TRIP12 of
                                                        // EPWM-XBAR
#define XBAR_TRIP12MUX16TO31CFG_MUX18_S   4U
#define XBAR_TRIP12MUX16TO31CFG_MUX18_M   0x30U         // Mux18 Configuration for TRIP12 of
                                                        // EPWM-XBAR
#define XBAR_TRIP12MUX16TO31CFG_MUX19_S   6U
#define XBAR_TRIP12MUX16TO31CFG_MUX19_M   0xC0U         // Mux19 Configuration for TRIP12 of
                                                        // EPWM-XBAR
#define XBAR_TRIP12MUX16TO31CFG_MUX20_S   8U
#define XBAR_TRIP12MUX16TO31CFG_MUX20_M   0x300U        // Mux20 Configuration for TRIP12 of
                                                        // EPWM-XBAR
#define XBAR_TRIP12MUX16TO31CFG_MUX21_S   10U
#define XBAR_TRIP12MUX16TO31CFG_MUX21_M   0xC00U        // Mux21 Configuration for TRIP12 of
                                                        // EPWM-XBAR
#define XBAR_TRIP12MUX16TO31CFG_MUX22_S   12U
#define XBAR_TRIP12MUX16TO31CFG_MUX22_M   0x3000U       // Mux22 Configuration for TRIP12 of
                                                        // EPWM-XBAR
#define XBAR_TRIP12MUX16TO31CFG_MUX23_S   14U
#define XBAR_TRIP12MUX16TO31CFG_MUX23_M   0xC000U       // Mux23 Configuration for TRIP12 of
                                                        // EPWM-XBAR
#define XBAR_TRIP12MUX16TO31CFG_MUX24_S   16U
#define XBAR_TRIP12MUX16TO31CFG_MUX24_M   0x30000U      // Mux24 Configuration for TRIP12 of
                                                        // EPWM-XBAR
#define XBAR_TRIP12MUX16TO31CFG_MUX25_S   18U
#define XBAR_TRIP12MUX16TO31CFG_MUX25_M   0xC0000U      // Mux25 Configuration for TRIP12 of
                                                        // EPWM-XBAR
#define XBAR_TRIP12MUX16TO31CFG_MUX26_S   20U
#define XBAR_TRIP12MUX16TO31CFG_MUX26_M   0x300000U     // Mux26 Configuration for TRIP12 of
                                                        // EPWM-XBAR
#define XBAR_TRIP12MUX16TO31CFG_MUX27_S   22U
#define XBAR_TRIP12MUX16TO31CFG_MUX27_M   0xC00000U     // Mux27 Configuration for TRIP12 of
                                                        // EPWM-XBAR
#define XBAR_TRIP12MUX16TO31CFG_MUX28_S   24U
#define XBAR_TRIP12MUX16TO31CFG_MUX28_M   0x3000000U    // Mux28 Configuration for TRIP12 of
                                                        // EPWM-XBAR
#define XBAR_TRIP12MUX16TO31CFG_MUX29_S   26U
#define XBAR_TRIP12MUX16TO31CFG_MUX29_M   0xC000000U    // Mux29 Configuration for TRIP12 of
                                                        // EPWM-XBAR
#define XBAR_TRIP12MUX16TO31CFG_MUX30_S   28U
#define XBAR_TRIP12MUX16TO31CFG_MUX30_M   0x30000000U   // Mux30 Configuration for TRIP12 of
                                                        // EPWM-XBAR
#define XBAR_TRIP12MUX16TO31CFG_MUX31_S   30U
#define XBAR_TRIP12MUX16TO31CFG_MUX31_M   0xC0000000U   // Mux31 Configuration for TRIP12 of
                                                        // EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP4MUXENABLE register
//
//*************************************************************************************************
#define XBAR_TRIP4MUXENABLE_MUX0    0x1U          // mux0 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX1    0x2U          // Mux1 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX2    0x4U          // Mux2 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX3    0x8U          // Mux3 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX4    0x10U         // Mux4 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX5    0x20U         // Mux5 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX6    0x40U         // Mux6 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX7    0x80U         // Mux7 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX8    0x100U        // Mux8 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX9    0x200U        // Mux9 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX10   0x400U        // Mux10 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX11   0x800U        // Mux11 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX12   0x1000U       // Mux12 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX13   0x2000U       // Mux13 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX14   0x4000U       // Mux14 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX15   0x8000U       // Mux15 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX16   0x10000U      // Mux16 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX17   0x20000U      // Mux17 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX18   0x40000U      // Mux18 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX19   0x80000U      // Mux19 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX20   0x100000U     // Mux20 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX21   0x200000U     // Mux21 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX22   0x400000U     // Mux22 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX23   0x800000U     // Mux23 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX24   0x1000000U    // Mux24 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX25   0x2000000U    // Mux25 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX26   0x4000000U    // Mux26 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX27   0x8000000U    // Mux27 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX28   0x10000000U   // Mux28 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX29   0x20000000U   // Mux29 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX30   0x40000000U   // Mux30 to drive TRIP4 of EPWM-XBAR
#define XBAR_TRIP4MUXENABLE_MUX31   0x80000000U   // Mux31 to drive TRIP4 of EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP5MUXENABLE register
//
//*************************************************************************************************
#define XBAR_TRIP5MUXENABLE_MUX0    0x1U          // mux0 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX1    0x2U          // Mux1 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX2    0x4U          // Mux2 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX3    0x8U          // Mux3 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX4    0x10U         // Mux4 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX5    0x20U         // Mux5 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX6    0x40U         // Mux6 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX7    0x80U         // Mux7 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX8    0x100U        // Mux8 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX9    0x200U        // Mux9 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX10   0x400U        // Mux10 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX11   0x800U        // Mux11 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX12   0x1000U       // Mux12 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX13   0x2000U       // Mux13 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX14   0x4000U       // Mux14 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX15   0x8000U       // Mux15 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX16   0x10000U      // Mux16 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX17   0x20000U      // Mux17 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX18   0x40000U      // Mux18 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX19   0x80000U      // Mux19 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX20   0x100000U     // Mux20 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX21   0x200000U     // Mux21 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX22   0x400000U     // Mux22 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX23   0x800000U     // Mux23 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX24   0x1000000U    // Mux24 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX25   0x2000000U    // Mux25 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX26   0x4000000U    // Mux26 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX27   0x8000000U    // Mux27 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX28   0x10000000U   // Mux28 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX29   0x20000000U   // Mux29 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX30   0x40000000U   // Mux30 to drive TRIP5 of EPWM-XBAR
#define XBAR_TRIP5MUXENABLE_MUX31   0x80000000U   // Mux31 to drive TRIP5 of EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP7MUXENABLE register
//
//*************************************************************************************************
#define XBAR_TRIP7MUXENABLE_MUX0    0x1U          // mux0 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX1    0x2U          // Mux1 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX2    0x4U          // Mux2 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX3    0x8U          // Mux3 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX4    0x10U         // Mux4 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX5    0x20U         // Mux5 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX6    0x40U         // Mux6 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX7    0x80U         // Mux7 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX8    0x100U        // Mux8 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX9    0x200U        // Mux9 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX10   0x400U        // Mux10 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX11   0x800U        // Mux11 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX12   0x1000U       // Mux12 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX13   0x2000U       // Mux13 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX14   0x4000U       // Mux14 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX15   0x8000U       // Mux15 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX16   0x10000U      // Mux16 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX17   0x20000U      // Mux17 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX18   0x40000U      // Mux18 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX19   0x80000U      // Mux19 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX20   0x100000U     // Mux20 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX21   0x200000U     // Mux21 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX22   0x400000U     // Mux22 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX23   0x800000U     // Mux23 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX24   0x1000000U    // Mux24 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX25   0x2000000U    // Mux25 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX26   0x4000000U    // Mux26 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX27   0x8000000U    // Mux27 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX28   0x10000000U   // Mux28 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX29   0x20000000U   // Mux29 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX30   0x40000000U   // Mux30 to drive TRIP7 of EPWM-XBAR
#define XBAR_TRIP7MUXENABLE_MUX31   0x80000000U   // Mux31 to drive TRIP7 of EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP8MUXENABLE register
//
//*************************************************************************************************
#define XBAR_TRIP8MUXENABLE_MUX0    0x1U          // mux0 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX1    0x2U          // Mux1 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX2    0x4U          // Mux2 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX3    0x8U          // Mux3 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX4    0x10U         // Mux4 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX5    0x20U         // Mux5 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX6    0x40U         // Mux6 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX7    0x80U         // Mux7 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX8    0x100U        // Mux8 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX9    0x200U        // Mux9 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX10   0x400U        // Mux10 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX11   0x800U        // Mux11 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX12   0x1000U       // Mux12 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX13   0x2000U       // Mux13 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX14   0x4000U       // Mux14 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX15   0x8000U       // Mux15 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX16   0x10000U      // Mux16 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX17   0x20000U      // Mux17 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX18   0x40000U      // Mux18 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX19   0x80000U      // Mux19 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX20   0x100000U     // Mux20 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX21   0x200000U     // Mux21 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX22   0x400000U     // Mux22 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX23   0x800000U     // Mux23 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX24   0x1000000U    // Mux24 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX25   0x2000000U    // Mux25 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX26   0x4000000U    // Mux26 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX27   0x8000000U    // Mux27 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX28   0x10000000U   // Mux28 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX29   0x20000000U   // Mux29 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX30   0x40000000U   // Mux30 to drive TRIP8 of EPWM-XBAR
#define XBAR_TRIP8MUXENABLE_MUX31   0x80000000U   // Mux31 to drive TRIP8 of EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP9MUXENABLE register
//
//*************************************************************************************************
#define XBAR_TRIP9MUXENABLE_MUX0    0x1U          // mux0 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX1    0x2U          // Mux1 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX2    0x4U          // Mux2 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX3    0x8U          // Mux3 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX4    0x10U         // Mux4 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX5    0x20U         // Mux5 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX6    0x40U         // Mux6 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX7    0x80U         // Mux7 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX8    0x100U        // Mux8 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX9    0x200U        // Mux9 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX10   0x400U        // Mux10 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX11   0x800U        // Mux11 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX12   0x1000U       // Mux12 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX13   0x2000U       // Mux13 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX14   0x4000U       // Mux14 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX15   0x8000U       // Mux15 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX16   0x10000U      // Mux16 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX17   0x20000U      // Mux17 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX18   0x40000U      // Mux18 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX19   0x80000U      // Mux19 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX20   0x100000U     // Mux20 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX21   0x200000U     // Mux21 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX22   0x400000U     // Mux22 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX23   0x800000U     // Mux23 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX24   0x1000000U    // Mux24 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX25   0x2000000U    // Mux25 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX26   0x4000000U    // Mux26 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX27   0x8000000U    // Mux27 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX28   0x10000000U   // Mux28 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX29   0x20000000U   // Mux29 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX30   0x40000000U   // Mux30 to drive TRIP9 of EPWM-XBAR
#define XBAR_TRIP9MUXENABLE_MUX31   0x80000000U   // Mux31 to drive TRIP9 of EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP10MUXENABLE register
//
//*************************************************************************************************
#define XBAR_TRIP10MUXENABLE_MUX0    0x1U          // mux0 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX1    0x2U          // Mux1 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX2    0x4U          // Mux2 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX3    0x8U          // Mux3 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX4    0x10U         // Mux4 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX5    0x20U         // Mux5 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX6    0x40U         // Mux6 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX7    0x80U         // Mux7 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX8    0x100U        // Mux8 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX9    0x200U        // Mux9 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX10   0x400U        // Mux10 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX11   0x800U        // Mux11 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX12   0x1000U       // Mux12 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX13   0x2000U       // Mux13 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX14   0x4000U       // Mux14 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX15   0x8000U       // Mux15 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX16   0x10000U      // Mux16 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX17   0x20000U      // Mux17 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX18   0x40000U      // Mux18 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX19   0x80000U      // Mux19 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX20   0x100000U     // Mux20 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX21   0x200000U     // Mux21 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX22   0x400000U     // Mux22 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX23   0x800000U     // Mux23 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX24   0x1000000U    // Mux24 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX25   0x2000000U    // Mux25 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX26   0x4000000U    // Mux26 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX27   0x8000000U    // Mux27 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX28   0x10000000U   // Mux28 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX29   0x20000000U   // Mux29 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX30   0x40000000U   // Mux30 to drive TRIP10 of EPWM-XBAR
#define XBAR_TRIP10MUXENABLE_MUX31   0x80000000U   // Mux31 to drive TRIP10 of EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP11MUXENABLE register
//
//*************************************************************************************************
#define XBAR_TRIP11MUXENABLE_MUX0    0x1U          // mux0 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX1    0x2U          // Mux1 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX2    0x4U          // Mux2 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX3    0x8U          // Mux3 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX4    0x10U         // Mux4 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX5    0x20U         // Mux5 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX6    0x40U         // Mux6 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX7    0x80U         // Mux7 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX8    0x100U        // Mux8 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX9    0x200U        // Mux9 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX10   0x400U        // Mux10 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX11   0x800U        // Mux11 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX12   0x1000U       // Mux12 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX13   0x2000U       // Mux13 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX14   0x4000U       // Mux14 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX15   0x8000U       // Mux15 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX16   0x10000U      // Mux16 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX17   0x20000U      // Mux17 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX18   0x40000U      // Mux18 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX19   0x80000U      // Mux19 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX20   0x100000U     // Mux20 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX21   0x200000U     // Mux21 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX22   0x400000U     // Mux22 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX23   0x800000U     // Mux23 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX24   0x1000000U    // Mux24 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX25   0x2000000U    // Mux25 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX26   0x4000000U    // Mux26 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX27   0x8000000U    // Mux27 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX28   0x10000000U   // Mux28 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX29   0x20000000U   // Mux29 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX30   0x40000000U   // Mux30 to drive TRIP11 of EPWM-XBAR
#define XBAR_TRIP11MUXENABLE_MUX31   0x80000000U   // Mux31 to drive TRIP11 of EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIP12MUXENABLE register
//
//*************************************************************************************************
#define XBAR_TRIP12MUXENABLE_MUX0    0x1U          // mux0 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX1    0x2U          // Mux1 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX2    0x4U          // Mux2 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX3    0x8U          // Mux3 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX4    0x10U         // Mux4 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX5    0x20U         // Mux5 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX6    0x40U         // Mux6 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX7    0x80U         // Mux7 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX8    0x100U        // Mux8 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX9    0x200U        // Mux9 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX10   0x400U        // Mux10 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX11   0x800U        // Mux11 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX12   0x1000U       // Mux12 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX13   0x2000U       // Mux13 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX14   0x4000U       // Mux14 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX15   0x8000U       // Mux15 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX16   0x10000U      // Mux16 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX17   0x20000U      // Mux17 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX18   0x40000U      // Mux18 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX19   0x80000U      // Mux19 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX20   0x100000U     // Mux20 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX21   0x200000U     // Mux21 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX22   0x400000U     // Mux22 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX23   0x800000U     // Mux23 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX24   0x1000000U    // Mux24 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX25   0x2000000U    // Mux25 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX26   0x4000000U    // Mux26 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX27   0x8000000U    // Mux27 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX28   0x10000000U   // Mux28 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX29   0x20000000U   // Mux29 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX30   0x40000000U   // Mux30 to drive TRIP12 of EPWM-XBAR
#define XBAR_TRIP12MUXENABLE_MUX31   0x80000000U   // Mux31 to drive TRIP12 of EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIPOUTINV register
//
//*************************************************************************************************
#define XBAR_TRIPOUTINV_TRIP4    0x1U    // Selects polarity for TRIP4 of EPWM-XBAR
#define XBAR_TRIPOUTINV_TRIP5    0x2U    // Selects polarity for TRIP5 of EPWM-XBAR
#define XBAR_TRIPOUTINV_TRIP7    0x4U    // Selects polarity for TRIP7 of EPWM-XBAR
#define XBAR_TRIPOUTINV_TRIP8    0x8U    // Selects polarity for TRIP8 of EPWM-XBAR
#define XBAR_TRIPOUTINV_TRIP9    0x10U   // Selects polarity for TRIP9 of EPWM-XBAR
#define XBAR_TRIPOUTINV_TRIP10   0x20U   // Selects polarity for TRIP10 of EPWM-XBAR
#define XBAR_TRIPOUTINV_TRIP11   0x40U   // Selects polarity for TRIP11 of EPWM-XBAR
#define XBAR_TRIPOUTINV_TRIP12   0x80U   // Selects polarity for TRIP12 of EPWM-XBAR

//*************************************************************************************************
//
// The following are defines for the bit fields in the TRIPLOCK register
//
//*************************************************************************************************
#define XBAR_TRIPLOCK_LOCK    0x1U          // Locks the configuration for EPWM-XBAR
#define XBAR_TRIPLOCK_KEY_S   16U
#define XBAR_TRIPLOCK_KEY_M   0xFFFF0000U   // Write protection KEY



#endif
