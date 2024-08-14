//###########################################################################
//
// FILE:    hw_sdfm.h
//
// TITLE:   Definitions for the SDFM registers.
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

#ifndef HW_SDFM_H
#define HW_SDFM_H

//*************************************************************************************************
//
// The following are defines for the SDFM register offsets
//
//*************************************************************************************************
#define SDFM_O_SDIFLG                 0x0U    // SD Interrupt Flag Register
#define SDFM_O_SDIFLGCLR              0x2U    // SD Interrupt Flag Clear Register
#define SDFM_O_SDCTL                  0x4U    // SD Control Register
#define SDFM_O_SDMFILEN               0x6U    // SD Master Filter Enable
#define SDFM_O_SDSTATUS               0x7U    // SD Status Register
#define SDFM_O_SDCTLPARM1             0x10U   // Control Parameter Register for Ch1
#define SDFM_O_SDDFPARM1              0x11U   // Data Filter Parameter Register for Ch1
#define SDFM_O_SDDPARM1               0x12U   // Data Parameter Register for Ch1
#define SDFM_O_SDFLT1CMPH1            0x13U   // High-level Threshold Register for Ch1
#define SDFM_O_SDFLT1CMPL1            0x14U   // Low-level Threshold Register for Ch1
#define SDFM_O_SDCPARM1               0x15U   // Comparator Filter Parameter Register for Ch1
#define SDFM_O_SDDATA1                0x16U   // Data Filter Data Register (16 or 32bit) for Ch1
#define SDFM_O_SDDATFIFO1             0x18U   // Filter Data FIFO Output(32b) for Ch1
#define SDFM_O_SDCDATA1               0x1AU   // Comparator Filter Data Register (16b) for Ch1
#define SDFM_O_SDFLT1CMPH2            0x1BU   // Second high level threhold  for CH1
#define SDFM_O_SDFLT1CMPHZ            0x1CU   // High-level (Z) Threshold Register for Ch1
#define SDFM_O_SDFIFOCTL1             0x1DU   // FIFO Control Register for Ch1
#define SDFM_O_SDSYNC1                0x1EU   // SD Filter Sync control for Ch1
#define SDFM_O_SDFLT1CMPL2            0x1FU   // Second low level threhold  for CH1
#define SDFM_O_SDCTLPARM2             0x20U   // Control Parameter Register for Ch2
#define SDFM_O_SDDFPARM2              0x21U   // Data Filter Parameter Register for Ch2
#define SDFM_O_SDDPARM2               0x22U   // Data Parameter Register for Ch2
#define SDFM_O_SDFLT2CMPH1            0x23U   // High-level Threshold Register for Ch2
#define SDFM_O_SDFLT2CMPL1            0x24U   // Low-level Threshold Register for Ch2
#define SDFM_O_SDCPARM2               0x25U   // Comparator Filter Parameter Register for Ch2
#define SDFM_O_SDDATA2                0x26U   // Data Filter Data Register (16 or 32bit) for Ch2
#define SDFM_O_SDDATFIFO2             0x28U   // Filter Data FIFO Output(32b) for Ch2
#define SDFM_O_SDCDATA2               0x2AU   // Comparator Filter Data Register (16b) for Ch2
#define SDFM_O_SDFLT2CMPH2            0x2BU   // Second high level threhold  for CH2
#define SDFM_O_SDFLT2CMPHZ            0x2CU   // High-level (Z) Threshold Register for Ch2
#define SDFM_O_SDFIFOCTL2             0x2DU   // FIFO Control Register for Ch2
#define SDFM_O_SDSYNC2                0x2EU   // SD Filter Sync control for Ch2
#define SDFM_O_SDFLT2CMPL2            0x2FU   // Second low level threhold  for CH2
#define SDFM_O_SDCTLPARM3             0x30U   // Control Parameter Register for Ch3
#define SDFM_O_SDDFPARM3              0x31U   // Data Filter Parameter Register for Ch3
#define SDFM_O_SDDPARM3               0x32U   // Data Parameter Register for Ch3
#define SDFM_O_SDFLT3CMPH1            0x33U   // High-level Threshold Register for Ch3
#define SDFM_O_SDFLT3CMPL1            0x34U   // Low-level Threshold Register for Ch3
#define SDFM_O_SDCPARM3               0x35U   // Comparator Filter Parameter Register for Ch3
#define SDFM_O_SDDATA3                0x36U   // Data Filter Data Register (16 or 32bit) for Ch3
#define SDFM_O_SDDATFIFO3             0x38U   // Filter Data FIFO Output(32b) for Ch3
#define SDFM_O_SDCDATA3               0x3AU   // Comparator Filter Data Register (16b) for Ch3
#define SDFM_O_SDFLT3CMPH2            0x3BU   // Second high level threhold  for CH3
#define SDFM_O_SDFLT3CMPHZ            0x3CU   // High-level (Z) Threshold Register for Ch3
#define SDFM_O_SDFIFOCTL3             0x3DU   // FIFO Control Register for Ch3
#define SDFM_O_SDSYNC3                0x3EU   // SD Filter Sync control for Ch3
#define SDFM_O_SDFLT3CMPL2            0x3FU   // Second low level threhold  for CH3
#define SDFM_O_SDCTLPARM4             0x40U   // Control Parameter Register for Ch4
#define SDFM_O_SDDFPARM4              0x41U   // Data Filter Parameter Register for Ch4
#define SDFM_O_SDDPARM4               0x42U   // Data Parameter Register for Ch4
#define SDFM_O_SDFLT4CMPH1            0x43U   // High-level Threshold Register for Ch4
#define SDFM_O_SDFLT4CMPL1            0x44U   // Low-level Threshold Register for Ch4
#define SDFM_O_SDCPARM4               0x45U   // Comparator Filter Parameter Register for Ch4
#define SDFM_O_SDDATA4                0x46U   // Data Filter Data Register (16 or 32bit) for Ch4
#define SDFM_O_SDDATFIFO4             0x48U   // Filter Data FIFO Output(32b) for Ch4
#define SDFM_O_SDCDATA4               0x4AU   // Comparator Filter Data Register (16b) for Ch4
#define SDFM_O_SDFLT4CMPH2            0x4BU   // Second high level threhold  for CH4
#define SDFM_O_SDFLT4CMPHZ            0x4CU   // High-level (Z) Threshold Register for Ch4
#define SDFM_O_SDFIFOCTL4             0x4DU   // FIFO Control Register for Ch4
#define SDFM_O_SDSYNC4                0x4EU   // SD Filter Sync control for Ch4
#define SDFM_O_SDFLT4CMPL2            0x4FU   // Second low level threhold  for CH4
#define SDFM_O_SDCOMP1CTL             0x60U   // SD Comparator event filter1 Control Register
#define SDFM_O_SDCOMP1EVT2FLTCTL      0x61U   // COMPL/CEVT2 Digital filter1 Control Register
#define SDFM_O_SDCOMP1EVT2FLTCLKCTL   0x62U   // COMPL/CEVT2 Digital filter1 Clock Control Register
#define SDFM_O_SDCOMP1EVT1FLTCTL      0x63U   // COMPH/CEVT1 Digital filter1 Control Register
#define SDFM_O_SDCOMP1EVT1FLTCLKCTL   0x64U   // COMPH/CEVT1 Digital filter1 Clock Control Register
#define SDFM_O_SDCOMP1LOCK            0x67U   // SD  compartor event filter1 Lock Register
#define SDFM_O_SDCOMP2CTL             0x68U   // SD Comparator event filter2 Control Register
#define SDFM_O_SDCOMP2EVT2FLTCTL      0x69U   // COMPL/CEVT2 Digital filter2 Control Register
#define SDFM_O_SDCOMP2EVT2FLTCLKCTL   0x6AU   // COMPL/CEVT2 Digital filter2 Clock Control Register
#define SDFM_O_SDCOMP2EVT1FLTCTL      0x6BU   // COMPH/CEVT1 Digital filter2 Control Register
#define SDFM_O_SDCOMP2EVT1FLTCLKCTL   0x6CU   // COMPH/CEVT1 Digital filter2 Clock Control Register
#define SDFM_O_SDCOMP2LOCK            0x6FU   // SD  compartor event filter2 Lock Register
#define SDFM_O_SDCOMP3CTL             0x70U   // SD Comparator event filter3 Control Register
#define SDFM_O_SDCOMP3EVT2FLTCTL      0x71U   // COMPL/CEVT2 Digital filter3 Control Register
#define SDFM_O_SDCOMP3EVT2FLTCLKCTL   0x72U   // COMPL/CEVT2 Digital filter3 Clock Control Register
#define SDFM_O_SDCOMP3EVT1FLTCTL      0x73U   // COMPH/CEVT1 Digital filter3 Control Register
#define SDFM_O_SDCOMP3EVT1FLTCLKCTL   0x74U   // COMPH/CEVT1 Digital filter3 Clock Control Register
#define SDFM_O_SDCOMP3LOCK            0x77U   // SD  compartor event filter3 Lock Register
#define SDFM_O_SDCOMP4CTL             0x78U   // SD Comparator event filter4 Control Register
#define SDFM_O_SDCOMP4EVT2FLTCTL      0x79U   // COMPL/CEVT2 Digital filter4 Control Register
#define SDFM_O_SDCOMP4EVT2FLTCLKCTL   0x7AU   // COMPL/CEVT2 Digital filter4 Clock Control Register
#define SDFM_O_SDCOMP4EVT1FLTCTL      0x7BU   // COMPH/CEVT1 Digital filter4 Control Register
#define SDFM_O_SDCOMP4EVT1FLTCLKCTL   0x7CU   // COMPH/CEVT1 Digital filter4 Clock Control Register
#define SDFM_O_SDCOMP4LOCK            0x7FU   // SD  compartor event filter4 Lock Register


//*************************************************************************************************
//
// The following are defines for the bit fields in the SDIFLG register
//
//*************************************************************************************************
#define SDFM_SDIFLG_FLT1_FLG_CEVT1   0x1U          // High-level Interrupt flag for Ch1
#define SDFM_SDIFLG_FLT1_FLG_CEVT2   0x2U          // Low-level Interrupt flag for Ch1
#define SDFM_SDIFLG_FLT2_FLG_CEVT1   0x4U          // High-level Interrupt flag for Ch2
#define SDFM_SDIFLG_FLT2_FLG_CEVT2   0x8U          // Low-level Interrupt flag for Ch2
#define SDFM_SDIFLG_FLT3_FLG_CEVT1   0x10U         // High-level Interrupt flag for Ch3
#define SDFM_SDIFLG_FLT3_FLG_CEVT2   0x20U         // Low-level Interrupt flag for Ch3
#define SDFM_SDIFLG_FLT4_FLG_CEVT1   0x40U         // High-level Interrupt flag for Ch4
#define SDFM_SDIFLG_FLT4_FLG_CEVT2   0x80U         // Low-level Interrupt flag for Ch4
#define SDFM_SDIFLG_MF1              0x100U        // Modulator Failure for Filter 1
#define SDFM_SDIFLG_MF2              0x200U        // Modulator Failure for Filter 2
#define SDFM_SDIFLG_MF3              0x400U        // Modulator Failure for Filter 3
#define SDFM_SDIFLG_MF4              0x800U        // Modulator Failure for Filter 4
#define SDFM_SDIFLG_AF1              0x1000U       // Acknowledge flag for Filter 1
#define SDFM_SDIFLG_AF2              0x2000U       // Acknowledge flag for Filter 2
#define SDFM_SDIFLG_AF3              0x4000U       // Acknowledge flag for Filter 3
#define SDFM_SDIFLG_AF4              0x8000U       // Acknowledge flag for Filter 4
#define SDFM_SDIFLG_SDFFOVF1         0x10000U      // FIFO Overflow Flag for Ch1.
#define SDFM_SDIFLG_SDFFOVF2         0x20000U      // FIFO Overflow Flag for Ch2
#define SDFM_SDIFLG_SDFFOVF3         0x40000U      // FIFO Overflow Flag for Ch3
#define SDFM_SDIFLG_SDFFOVF4         0x80000U      // FIFO Overflow Flag for Ch4
#define SDFM_SDIFLG_SDFFINT1         0x100000U     // SDFIFO interrupt for Ch1
#define SDFM_SDIFLG_SDFFINT2         0x200000U     // SDFIFO interrupt for Ch2
#define SDFM_SDIFLG_SDFFINT3         0x400000U     // SDFIFO interrupt for Ch3
#define SDFM_SDIFLG_SDFFINT4         0x800000U     // SDFIFO interrupt for Ch4
#define SDFM_SDIFLG_MIF              0x80000000U   // Master Interrupt Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDIFLGCLR register
//
//*************************************************************************************************
#define SDFM_SDIFLGCLR_FLT1_FLG_CEVT1   0x1U          // High-level Interrupt flag for Ch1
#define SDFM_SDIFLGCLR_FLT1_FLG_CEVT2   0x2U          // Low-level Interrupt flag for Ch1
#define SDFM_SDIFLGCLR_FLT2_FLG_CEVT1   0x4U          // High-level Interrupt flag for Ch2
#define SDFM_SDIFLGCLR_FLT2_FLG_CEVT2   0x8U          // Low-level Interrupt flag for Ch2
#define SDFM_SDIFLGCLR_FLT3_FLG_CEVT1   0x10U         // High-level Interrupt flag for Ch3
#define SDFM_SDIFLGCLR_FLT3_FLG_CEVT2   0x20U         // Low-level Interrupt flag for Ch3
#define SDFM_SDIFLGCLR_FLT4_FLG_CEVT1   0x40U         // High-level Interrupt flag for Ch4
#define SDFM_SDIFLGCLR_FLT4_FLG_CEVT2   0x80U         // Low-level Interrupt flag for Ch4
#define SDFM_SDIFLGCLR_MF1              0x100U        // Modulator Failure for Filter 1
#define SDFM_SDIFLGCLR_MF2              0x200U        // Modulator Failure for Filter 2
#define SDFM_SDIFLGCLR_MF3              0x400U        // Modulator Failure for Filter 3
#define SDFM_SDIFLGCLR_MF4              0x800U        // Modulator Failure for Filter 4
#define SDFM_SDIFLGCLR_AF1              0x1000U       // Acknowledge flag for Filter 1
#define SDFM_SDIFLGCLR_AF2              0x2000U       // Acknowledge flag for Filter 2
#define SDFM_SDIFLGCLR_AF3              0x4000U       // Acknowledge flag for Filter 3
#define SDFM_SDIFLGCLR_AF4              0x8000U       // Acknowledge flag for Filter 4
#define SDFM_SDIFLGCLR_SDFFOVF1         0x10000U      // SDFIFO overflow clear Ch1
#define SDFM_SDIFLGCLR_SDFFOVF2         0x20000U      // SDFIFO overflow clear Ch2
#define SDFM_SDIFLGCLR_SDFFOVF3         0x40000U      // SDFIFO overflow clear Ch3
#define SDFM_SDIFLGCLR_SDFFOVF4         0x80000U      // SDFIFO overflow clear Ch4
#define SDFM_SDIFLGCLR_SDFFINT1         0x100000U     // SDFIFO Interrupt flag-clear bit for Ch1
#define SDFM_SDIFLGCLR_SDFFINT2         0x200000U     // SDFIFO Interrupt flag-clear bit for Ch2
#define SDFM_SDIFLGCLR_SDFFINT3         0x400000U     // SDFIFO Interrupt flag-clear bit for Ch3
#define SDFM_SDIFLGCLR_SDFFINT4         0x800000U     // SDFIFO Interrupt flag-clear bit for Ch4
#define SDFM_SDIFLGCLR_MIF              0x80000000U   // Master Interrupt Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCTL register
//
//*************************************************************************************************
#define SDFM_SDCTL_HZ1   0x1U      // High-level Threshold crossing (Z) flag Ch1
#define SDFM_SDCTL_HZ2   0x2U      // High-level Threshold crossing (Z) flag Ch2
#define SDFM_SDCTL_HZ3   0x4U      // High-level Threshold crossing (Z) flag Ch3
#define SDFM_SDCTL_HZ4   0x8U      // High-level Threshold crossing (Z) flag Ch4
#define SDFM_SDCTL_MIE   0x2000U   // Master SDy_ERR Interrupt enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDMFILEN register
//
//*************************************************************************************************
#define SDFM_SDMFILEN_MFE   0x800U   // Master Filter Enable.

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDSTATUS register
//
//*************************************************************************************************
#define SDFM_SDSTATUS_HZ1   0x1U   // High-level Threshold crossing (Z) flag Ch1
#define SDFM_SDSTATUS_HZ2   0x2U   // High-level Threshold crossing (Z) flag Ch2
#define SDFM_SDSTATUS_HZ3   0x4U   // High-level Threshold crossing (Z) flag Ch3
#define SDFM_SDSTATUS_HZ4   0x8U   // High-level Threshold crossing (Z) flag Ch4

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCTLPARM1 register
//
//*************************************************************************************************
#define SDFM_SDCTLPARM1_MOD_S        0U
#define SDFM_SDCTLPARM1_MOD_M        0x3U    // Modulator clocking modes
#define SDFM_SDCTLPARM1_SDCLKSEL     0x8U    // SD1 Clock source select.
#define SDFM_SDCTLPARM1_SDCLKSYNC    0x10U   // Enable Synchronizer on SD clock
#define SDFM_SDCTLPARM1_SDDATASYNC   0x40U   // Enable Synchronizer on SD data

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDFPARM1 register
//
//*************************************************************************************************
#define SDFM_SDDFPARM1_DOSR_S     0U
#define SDFM_SDDFPARM1_DOSR_M     0xFFU     // Data Filter Oversample Ratio= DOSR+1
#define SDFM_SDDFPARM1_FEN        0x100U    // Filter Enable
#define SDFM_SDDFPARM1_AE         0x200U    // Ack Enable
#define SDFM_SDDFPARM1_SST_S      10U
#define SDFM_SDDFPARM1_SST_M      0xC00U    // Data filter Structure (SincFast/1/2/3)
#define SDFM_SDDFPARM1_SDSYNCEN   0x1000U   // Data Filter Reset Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDPARM1 register
//
//*************************************************************************************************
#define SDFM_SDDPARM1_DR     0x400U    // Data Representation (0/1 = 16/32b 2's complement)
#define SDFM_SDDPARM1_SH_S   11U
#define SDFM_SDDPARM1_SH_M   0xF800U   // Shift Control (# bits to shift in 16b mode)

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT1CMPH1 register
//
//*************************************************************************************************
#define SDFM_SDFLT1CMPH1_HLT_S   0U
#define SDFM_SDFLT1CMPH1_HLT_M   0x7FFFU   // High-level threshold for the comparator filter output

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT1CMPL1 register
//
//*************************************************************************************************
#define SDFM_SDFLT1CMPL1_LLT_S   0U
#define SDFM_SDFLT1CMPL1_LLT_M   0x7FFFU   // Low-level threshold for the comparator filter output

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCPARM1 register
//
//*************************************************************************************************
#define SDFM_SDCPARM1_COSR_S       0U
#define SDFM_SDCPARM1_COSR_M       0x1FU     // Comparator Oversample Ratio. Actual rate COSR+1
#define SDFM_SDCPARM1_EN_CEVT1     0x20U     // CEVT1 Interrupt enable.
#define SDFM_SDCPARM1_EN_CEVT2     0x40U     // CEVT2 Interrupt enable.
#define SDFM_SDCPARM1_CS1_CS0_S    7U
#define SDFM_SDCPARM1_CS1_CS0_M    0x180U    // Comparator Filter Structure (SincFast/1/2/3)
#define SDFM_SDCPARM1_MFIE         0x200U    // Modulator Failure Interrupt enable
#define SDFM_SDCPARM1_HZEN         0x400U    // High level (Z) Threshold crossing output enable
#define SDFM_SDCPARM1_CEVT1SEL_S   11U
#define SDFM_SDCPARM1_CEVT1SEL_M   0x1800U   // Comparator Event1 select
#define SDFM_SDCPARM1_CEN          0x2000U   // Comparator Enable
#define SDFM_SDCPARM1_CEVT2SEL_S   14U
#define SDFM_SDCPARM1_CEVT2SEL_M   0xC000U   // Comparator Event2 select

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDATA1 register
//
//*************************************************************************************************
#define SDFM_SDDATA1_DATA16_S     0U
#define SDFM_SDDATA1_DATA16_M     0xFFFFU       // Lo-order 16b in 32b mode
#define SDFM_SDDATA1_DATA32HI_S   16U
#define SDFM_SDDATA1_DATA32HI_M   0xFFFF0000U   // Hi-order 16b in 32b mode, 16-bit Data in 16b
                                                // mode

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDATFIFO1 register
//
//*************************************************************************************************
#define SDFM_SDDATFIFO1_DATA16_S     0U
#define SDFM_SDDATFIFO1_DATA16_M     0xFFFFU       // Lo-order 16b in 32b mode
#define SDFM_SDDATFIFO1_DATA32HI_S   16U
#define SDFM_SDDATFIFO1_DATA32HI_M   0xFFFF0000U   // Hi-order 16b in 32b mode, 16-bit Data in 16b
                                                   // mode

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT1CMPH2 register
//
//*************************************************************************************************
#define SDFM_SDFLT1CMPH2_HLT2_S   0U
#define SDFM_SDFLT1CMPH2_HLT2_M   0x7FFFU   // Second High level threshold.

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT1CMPHZ register
//
//*************************************************************************************************
#define SDFM_SDFLT1CMPHZ_HLTZ_S   0U
#define SDFM_SDFLT1CMPHZ_HLTZ_M   0x7FFFU   // High-level threshold (Z) for the comparator filter
                                            // output

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFIFOCTL1 register
//
//*************************************************************************************************
#define SDFM_SDFIFOCTL1_SDFFIL_S   0U
#define SDFM_SDFIFOCTL1_SDFFIL_M   0x1FU     // SDFIFO Interrupt Level
#define SDFM_SDFIFOCTL1_SDFFST_S   6U
#define SDFM_SDFIFOCTL1_SDFFST_M   0x7C0U    // SDFIFO Status
#define SDFM_SDFIFOCTL1_FFIEN      0x1000U   // SDFIFO data ready Interrupt Enable
#define SDFM_SDFIFOCTL1_FFEN       0x2000U   // SDFIFO Enable
#define SDFM_SDFIFOCTL1_DRINTSEL   0x4000U   // Data-Ready Interrupt Source Select
#define SDFM_SDFIFOCTL1_OVFIEN     0x8000U   // SDFIFO Overflow interrupt enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDSYNC1 register
//
//*************************************************************************************************
#define SDFM_SDSYNC1_SYNCSEL_S     0U
#define SDFM_SDSYNC1_SYNCSEL_M     0x3FU    // SDSYNC Source Select
#define SDFM_SDSYNC1_WTSYNCEN      0x40U    // Wait-for-Sync Enable
#define SDFM_SDSYNC1_WTSYNFLG      0x80U    // Wait-for-Sync Flag
#define SDFM_SDSYNC1_WTSYNCLR      0x100U   // Wait-for-Sync Flag Clear
#define SDFM_SDSYNC1_FFSYNCCLREN   0x200U   // FIFO Clear-on-SDSYNC Enable
#define SDFM_SDSYNC1_WTSCLREN      0x400U   // WTSYNFLG Clear-on-FIFOINT Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT1CMPL2 register
//
//*************************************************************************************************
#define SDFM_SDFLT1CMPL2_LLT2_S   0U
#define SDFM_SDFLT1CMPL2_LLT2_M   0x7FFFU   // Second low-level threshold for the comparator filter
                                            // output.

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCTLPARM2 register
//
//*************************************************************************************************
#define SDFM_SDCTLPARM2_MOD_S        0U
#define SDFM_SDCTLPARM2_MOD_M        0x3U    // Modulator clocking modes
#define SDFM_SDCTLPARM2_SDCLKSEL     0x8U    // SD2 Clock source select.
#define SDFM_SDCTLPARM2_SDCLKSYNC    0x10U   // Enable Synchronizer on SD clock
#define SDFM_SDCTLPARM2_SDDATASYNC   0x40U   // Enable Synchronizer on SD data

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDFPARM2 register
//
//*************************************************************************************************
#define SDFM_SDDFPARM2_DOSR_S     0U
#define SDFM_SDDFPARM2_DOSR_M     0xFFU     // Data Filter Oversample Ratio= DOSR+1
#define SDFM_SDDFPARM2_FEN        0x100U    // Filter Enable
#define SDFM_SDDFPARM2_AE         0x200U    // Ack Enable
#define SDFM_SDDFPARM2_SST_S      10U
#define SDFM_SDDFPARM2_SST_M      0xC00U    // Data filter Structure (SincFast/1/2/3)
#define SDFM_SDDFPARM2_SDSYNCEN   0x1000U   // Data Filter Reset Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDPARM2 register
//
//*************************************************************************************************
#define SDFM_SDDPARM2_DR     0x400U    // Data Representation (0/1 = 16/32b 2's complement)
#define SDFM_SDDPARM2_SH_S   11U
#define SDFM_SDDPARM2_SH_M   0xF800U   // Shift Control (# bits to shift in 16b mode)

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT2CMPH1 register
//
//*************************************************************************************************
#define SDFM_SDFLT2CMPH1_HLT_S   0U
#define SDFM_SDFLT2CMPH1_HLT_M   0x7FFFU   // High-level threshold for the comparator filter output

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT2CMPL1 register
//
//*************************************************************************************************
#define SDFM_SDFLT2CMPL1_LLT_S   0U
#define SDFM_SDFLT2CMPL1_LLT_M   0x7FFFU   // Low-level threshold for the comparator filter output

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCPARM2 register
//
//*************************************************************************************************
#define SDFM_SDCPARM2_COSR_S       0U
#define SDFM_SDCPARM2_COSR_M       0x1FU     // Comparator Oversample Ratio. Actual rate COSR+1
#define SDFM_SDCPARM2_EN_CEVT1     0x20U     // CEVT1 Interrupt enable.
#define SDFM_SDCPARM2_EN_CEVT2     0x40U     // CEVT2 Interrupt enable.
#define SDFM_SDCPARM2_CS1_CS0_S    7U
#define SDFM_SDCPARM2_CS1_CS0_M    0x180U    // Comparator Filter Structure (SincFast/1/2/3)
#define SDFM_SDCPARM2_MFIE         0x200U    // Modulator Failure Interrupt enable
#define SDFM_SDCPARM2_HZEN         0x400U    // High level (Z) Threshold crossing output enable
#define SDFM_SDCPARM2_CEVT1SEL_S   11U
#define SDFM_SDCPARM2_CEVT1SEL_M   0x1800U   // Comparator Event1 select
#define SDFM_SDCPARM2_CEN          0x2000U   // Comparator Enable
#define SDFM_SDCPARM2_CEVT2SEL_S   14U
#define SDFM_SDCPARM2_CEVT2SEL_M   0xC000U   // Comparator Event2 select

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDATA2 register
//
//*************************************************************************************************
#define SDFM_SDDATA2_DATA16_S     0U
#define SDFM_SDDATA2_DATA16_M     0xFFFFU       // Lo-order 16b in 32b mode
#define SDFM_SDDATA2_DATA32HI_S   16U
#define SDFM_SDDATA2_DATA32HI_M   0xFFFF0000U   // Hi-order 16b in 32b mode, 16-bit Data in 16b
                                                // mode

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDATFIFO2 register
//
//*************************************************************************************************
#define SDFM_SDDATFIFO2_DATA16_S     0U
#define SDFM_SDDATFIFO2_DATA16_M     0xFFFFU       // Lo-order 16b in 32b mode
#define SDFM_SDDATFIFO2_DATA32HI_S   16U
#define SDFM_SDDATFIFO2_DATA32HI_M   0xFFFF0000U   // Hi-order 16b in 32b mode, 16-bit Data in 16b
                                                   // mode

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT2CMPH2 register
//
//*************************************************************************************************
#define SDFM_SDFLT2CMPH2_HLT2_S   0U
#define SDFM_SDFLT2CMPH2_HLT2_M   0x7FFFU   // Second High level threshold.

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT2CMPHZ register
//
//*************************************************************************************************
#define SDFM_SDFLT2CMPHZ_HLTZ_S   0U
#define SDFM_SDFLT2CMPHZ_HLTZ_M   0x7FFFU   // High-level threshold (Z) for the comparator filter
                                            // output

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFIFOCTL2 register
//
//*************************************************************************************************
#define SDFM_SDFIFOCTL2_SDFFIL_S   0U
#define SDFM_SDFIFOCTL2_SDFFIL_M   0x1FU     // SDFIFO Interrupt Level
#define SDFM_SDFIFOCTL2_SDFFST_S   6U
#define SDFM_SDFIFOCTL2_SDFFST_M   0x7C0U    // SDFIFO Status
#define SDFM_SDFIFOCTL2_FFIEN      0x1000U   // SDFIFO data ready Interrupt Enable
#define SDFM_SDFIFOCTL2_FFEN       0x2000U   // SDFIFO Enable
#define SDFM_SDFIFOCTL2_DRINTSEL   0x4000U   // Data-Ready Interrupt Source Select
#define SDFM_SDFIFOCTL2_OVFIEN     0x8000U   // SDFIFO Overflow interrupt enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDSYNC2 register
//
//*************************************************************************************************
#define SDFM_SDSYNC2_SYNCSEL_S     0U
#define SDFM_SDSYNC2_SYNCSEL_M     0x3FU    // SDSYNC Source Select
#define SDFM_SDSYNC2_WTSYNCEN      0x40U    // Wait-for-Sync Enable
#define SDFM_SDSYNC2_WTSYNFLG      0x80U    // Wait-for-Sync Flag
#define SDFM_SDSYNC2_WTSYNCLR      0x100U   // Wait-for-Sync Flag Clear
#define SDFM_SDSYNC2_FFSYNCCLREN   0x200U   // FIFO Clear-on-SDSYNC Enable
#define SDFM_SDSYNC2_WTSCLREN      0x400U   // WTSYNFLG Clear-on-FIFOINT Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT2CMPL2 register
//
//*************************************************************************************************
#define SDFM_SDFLT2CMPL2_LLT2_S   0U
#define SDFM_SDFLT2CMPL2_LLT2_M   0x7FFFU   // Second low-level threshold for the comparator filter
                                            // output.

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCTLPARM3 register
//
//*************************************************************************************************
#define SDFM_SDCTLPARM3_MOD_S        0U
#define SDFM_SDCTLPARM3_MOD_M        0x3U    // Modulator clocking modes
#define SDFM_SDCTLPARM3_SDCLKSEL     0x8U    // SD3 Clock source select.
#define SDFM_SDCTLPARM3_SDCLKSYNC    0x10U   // Enable Synchronizer on SD clock
#define SDFM_SDCTLPARM3_SDDATASYNC   0x40U   // Enable Synchronizer on SD data

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDFPARM3 register
//
//*************************************************************************************************
#define SDFM_SDDFPARM3_DOSR_S     0U
#define SDFM_SDDFPARM3_DOSR_M     0xFFU     // Data Filter Oversample Ratio= DOSR+1
#define SDFM_SDDFPARM3_FEN        0x100U    // Filter Enable
#define SDFM_SDDFPARM3_AE         0x200U    // Ack Enable
#define SDFM_SDDFPARM3_SST_S      10U
#define SDFM_SDDFPARM3_SST_M      0xC00U    // Data filter Structure (SincFast/1/2/3)
#define SDFM_SDDFPARM3_SDSYNCEN   0x1000U   // Data Filter Reset Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDPARM3 register
//
//*************************************************************************************************
#define SDFM_SDDPARM3_DR     0x400U    // Data Representation (0/1 = 16/32b 2's complement)
#define SDFM_SDDPARM3_SH_S   11U
#define SDFM_SDDPARM3_SH_M   0xF800U   // Shift Control (# bits to shift in 16b mode)

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT3CMPH1 register
//
//*************************************************************************************************
#define SDFM_SDFLT3CMPH1_HLT_S   0U
#define SDFM_SDFLT3CMPH1_HLT_M   0x7FFFU   // High-level threshold for the comparator filter output

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT3CMPL1 register
//
//*************************************************************************************************
#define SDFM_SDFLT3CMPL1_LLT_S   0U
#define SDFM_SDFLT3CMPL1_LLT_M   0x7FFFU   // Low-level threshold for the comparator filter output

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCPARM3 register
//
//*************************************************************************************************
#define SDFM_SDCPARM3_COSR_S       0U
#define SDFM_SDCPARM3_COSR_M       0x1FU     // Comparator Oversample Ratio. Actual rate COSR+1
#define SDFM_SDCPARM3_EN_CEVT1     0x20U     // CEVT1 Interrupt enable.
#define SDFM_SDCPARM3_EN_CEVT2     0x40U     // CEVT2 Interrupt enable.
#define SDFM_SDCPARM3_CS1_CS0_S    7U
#define SDFM_SDCPARM3_CS1_CS0_M    0x180U    // Comparator Filter Structure (SincFast/1/2/3)
#define SDFM_SDCPARM3_MFIE         0x200U    // Modulator Failure Interrupt enable
#define SDFM_SDCPARM3_HZEN         0x400U    // High level (Z) Threshold crossing output enable
#define SDFM_SDCPARM3_CEVT1SEL_S   11U
#define SDFM_SDCPARM3_CEVT1SEL_M   0x1800U   // Comparator Event1 select
#define SDFM_SDCPARM3_CEN          0x2000U   // Comparator Enable
#define SDFM_SDCPARM3_CEVT2SEL_S   14U
#define SDFM_SDCPARM3_CEVT2SEL_M   0xC000U   // Comparator Event2 select

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDATA3 register
//
//*************************************************************************************************
#define SDFM_SDDATA3_DATA16_S     0U
#define SDFM_SDDATA3_DATA16_M     0xFFFFU       // Lo-order 16b in 32b mode
#define SDFM_SDDATA3_DATA32HI_S   16U
#define SDFM_SDDATA3_DATA32HI_M   0xFFFF0000U   // Hi-order 16b in 32b mode, 16-bit Data in 16b
                                                // mode

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDATFIFO3 register
//
//*************************************************************************************************
#define SDFM_SDDATFIFO3_DATA16_S     0U
#define SDFM_SDDATFIFO3_DATA16_M     0xFFFFU       // Lo-order 16b in 32b mode
#define SDFM_SDDATFIFO3_DATA32HI_S   16U
#define SDFM_SDDATFIFO3_DATA32HI_M   0xFFFF0000U   // Hi-order 16b in 32b mode, 16-bit Data in 16b
                                                   // mode

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT3CMPH2 register
//
//*************************************************************************************************
#define SDFM_SDFLT3CMPH2_HLT2_S   0U
#define SDFM_SDFLT3CMPH2_HLT2_M   0x7FFFU   // Second High level threshold.

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT3CMPHZ register
//
//*************************************************************************************************
#define SDFM_SDFLT3CMPHZ_HLTZ_S   0U
#define SDFM_SDFLT3CMPHZ_HLTZ_M   0x7FFFU   // High-level threshold (Z) for the comparator filter
                                            // output

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFIFOCTL3 register
//
//*************************************************************************************************
#define SDFM_SDFIFOCTL3_SDFFIL_S   0U
#define SDFM_SDFIFOCTL3_SDFFIL_M   0x1FU     // SDFIFO Interrupt Level
#define SDFM_SDFIFOCTL3_SDFFST_S   6U
#define SDFM_SDFIFOCTL3_SDFFST_M   0x7C0U    // SDFIFO Status
#define SDFM_SDFIFOCTL3_FFIEN      0x1000U   // SDFIFO data ready Interrupt Enable
#define SDFM_SDFIFOCTL3_FFEN       0x2000U   // SDFIFO Enable
#define SDFM_SDFIFOCTL3_DRINTSEL   0x4000U   // Data-Ready Interrupt Source Select
#define SDFM_SDFIFOCTL3_OVFIEN     0x8000U   // SDFIFO Overflow interrupt enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDSYNC3 register
//
//*************************************************************************************************
#define SDFM_SDSYNC3_SYNCSEL_S     0U
#define SDFM_SDSYNC3_SYNCSEL_M     0x3FU    // SDSYNC Source Select
#define SDFM_SDSYNC3_WTSYNCEN      0x40U    // Wait-for-Sync Enable
#define SDFM_SDSYNC3_WTSYNFLG      0x80U    // Wait-for-Sync Flag
#define SDFM_SDSYNC3_WTSYNCLR      0x100U   // Wait-for-Sync Flag Clear
#define SDFM_SDSYNC3_FFSYNCCLREN   0x200U   // FIFO Clear-on-SDSYNC Enable
#define SDFM_SDSYNC3_WTSCLREN      0x400U   // WTSYNFLG Clear-on-FIFOINT Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT3CMPL2 register
//
//*************************************************************************************************
#define SDFM_SDFLT3CMPL2_LLT2_S   0U
#define SDFM_SDFLT3CMPL2_LLT2_M   0x7FFFU   // Second low-level threshold for the comparator filter
                                            // output.

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCTLPARM4 register
//
//*************************************************************************************************
#define SDFM_SDCTLPARM4_MOD_S        0U
#define SDFM_SDCTLPARM4_MOD_M        0x3U    // Modulator clocking modes
#define SDFM_SDCTLPARM4_SDCLKSEL     0x8U    // SD4 Clock source select.
#define SDFM_SDCTLPARM4_SDCLKSYNC    0x10U   // Enable Synchronizer on SD clock
#define SDFM_SDCTLPARM4_SDDATASYNC   0x40U   // Enable Synchronizer on SD data

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDFPARM4 register
//
//*************************************************************************************************
#define SDFM_SDDFPARM4_DOSR_S     0U
#define SDFM_SDDFPARM4_DOSR_M     0xFFU     // Data Filter Oversample Ratio= DOSR+1
#define SDFM_SDDFPARM4_FEN        0x100U    // Filter Enable
#define SDFM_SDDFPARM4_AE         0x200U    // Ack Enable
#define SDFM_SDDFPARM4_SST_S      10U
#define SDFM_SDDFPARM4_SST_M      0xC00U    // Data filter Structure (SincFast/1/2/3)
#define SDFM_SDDFPARM4_SDSYNCEN   0x1000U   // Data Filter Reset Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDPARM4 register
//
//*************************************************************************************************
#define SDFM_SDDPARM4_DR     0x400U    // Data Representation (0/1 = 16/32b 2's complement)
#define SDFM_SDDPARM4_SH_S   11U
#define SDFM_SDDPARM4_SH_M   0xF800U   // Shift Control (# bits to shift in 16b mode)

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT4CMPH1 register
//
//*************************************************************************************************
#define SDFM_SDFLT4CMPH1_HLT_S   0U
#define SDFM_SDFLT4CMPH1_HLT_M   0x7FFFU   // High-level threshold for the comparator filter output

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT4CMPL1 register
//
//*************************************************************************************************
#define SDFM_SDFLT4CMPL1_LLT_S   0U
#define SDFM_SDFLT4CMPL1_LLT_M   0x7FFFU   // Low-level threshold for the comparator filter output

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCPARM4 register
//
//*************************************************************************************************
#define SDFM_SDCPARM4_COSR_S       0U
#define SDFM_SDCPARM4_COSR_M       0x1FU     // Comparator Oversample Ratio. Actual rate COSR+1
#define SDFM_SDCPARM4_EN_CEVT1     0x20U     // CEVT1 Interrupt enable.
#define SDFM_SDCPARM4_EN_CEVT2     0x40U     // CEVT2 Interrupt enable.
#define SDFM_SDCPARM4_CS1_CS0_S    7U
#define SDFM_SDCPARM4_CS1_CS0_M    0x180U    // Comparator Filter Structure (SincFast/1/2/3)
#define SDFM_SDCPARM4_MFIE         0x200U    // Modulator Failure Interrupt enable
#define SDFM_SDCPARM4_HZEN         0x400U    // High level (Z) Threshold crossing output enable
#define SDFM_SDCPARM4_CEVT1SEL_S   11U
#define SDFM_SDCPARM4_CEVT1SEL_M   0x1800U   // Comparator Event1 select
#define SDFM_SDCPARM4_CEN          0x2000U   // Comparator Enable
#define SDFM_SDCPARM4_CEVT2SEL_S   14U
#define SDFM_SDCPARM4_CEVT2SEL_M   0xC000U   // Comparator Event2 select

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDATA4 register
//
//*************************************************************************************************
#define SDFM_SDDATA4_DATA16_S     0U
#define SDFM_SDDATA4_DATA16_M     0xFFFFU       // Lo-order 16b in 32b mode
#define SDFM_SDDATA4_DATA32HI_S   16U
#define SDFM_SDDATA4_DATA32HI_M   0xFFFF0000U   // Hi-order 16b in 32b mode, 16-bit Data in 16b
                                                // mode

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDDATFIFO4 register
//
//*************************************************************************************************
#define SDFM_SDDATFIFO4_DATA16_S     0U
#define SDFM_SDDATFIFO4_DATA16_M     0xFFFFU       // Lo-order 16b in 32b mode
#define SDFM_SDDATFIFO4_DATA32HI_S   16U
#define SDFM_SDDATFIFO4_DATA32HI_M   0xFFFF0000U   // Hi-order 16b in 32b mode, 16-bit Data in 16b
                                                   // mode

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT4CMPH2 register
//
//*************************************************************************************************
#define SDFM_SDFLT4CMPH2_HLT2_S   0U
#define SDFM_SDFLT4CMPH2_HLT2_M   0x7FFFU   // Second High level threshold.

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT4CMPHZ register
//
//*************************************************************************************************
#define SDFM_SDFLT4CMPHZ_HLTZ_S   0U
#define SDFM_SDFLT4CMPHZ_HLTZ_M   0x7FFFU   // High-level threshold (Z) for the comparator filter
                                            // output

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFIFOCTL4 register
//
//*************************************************************************************************
#define SDFM_SDFIFOCTL4_SDFFIL_S   0U
#define SDFM_SDFIFOCTL4_SDFFIL_M   0x1FU     // SDFIFO Interrupt Level
#define SDFM_SDFIFOCTL4_SDFFST_S   6U
#define SDFM_SDFIFOCTL4_SDFFST_M   0x7C0U    // SDFIFO Status
#define SDFM_SDFIFOCTL4_FFIEN      0x1000U   // SDFIFO data ready Interrupt Enable
#define SDFM_SDFIFOCTL4_FFEN       0x2000U   // SDFIFO Enable
#define SDFM_SDFIFOCTL4_DRINTSEL   0x4000U   // Data-Ready Interrupt Source Select
#define SDFM_SDFIFOCTL4_OVFIEN     0x8000U   // SDFIFO Overflow interrupt enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDSYNC4 register
//
//*************************************************************************************************
#define SDFM_SDSYNC4_SYNCSEL_S     0U
#define SDFM_SDSYNC4_SYNCSEL_M     0x3FU    // SDSYNC Source Select
#define SDFM_SDSYNC4_WTSYNCEN      0x40U    // Wait-for-Sync Enable
#define SDFM_SDSYNC4_WTSYNFLG      0x80U    // Wait-for-Sync Flag
#define SDFM_SDSYNC4_WTSYNCLR      0x100U   // Wait-for-Sync Flag Clear
#define SDFM_SDSYNC4_FFSYNCCLREN   0x200U   // FIFO Clear-on-SDSYNC Enable
#define SDFM_SDSYNC4_WTSCLREN      0x400U   // WTSYNFLG Clear-on-FIFOINT Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFLT4CMPL2 register
//
//*************************************************************************************************
#define SDFM_SDFLT4CMPL2_LLT2_S   0U
#define SDFM_SDFLT4CMPL2_LLT2_M   0x7FFFU   // Second low-level threshold for the comparator filter
                                            // output.

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP1CTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP1CTL_CEVT1DIGFILTSEL_S   2U
#define SDFM_SDCOMP1CTL_CEVT1DIGFILTSEL_M   0xCU     // High Comparator Trip Select
#define SDFM_SDCOMP1CTL_CEVT2DIGFILTSEL_S   10U
#define SDFM_SDCOMP1CTL_CEVT2DIGFILTSEL_M   0xC00U   // Low Comparator Trip Select

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP1EVT2FLTCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP1EVT2FLTCTL_SAMPWIN_S   4U
#define SDFM_SDCOMP1EVT2FLTCTL_SAMPWIN_M   0x1F0U    // Sample Window
#define SDFM_SDCOMP1EVT2FLTCTL_THRESH_S    9U
#define SDFM_SDCOMP1EVT2FLTCTL_THRESH_M    0x3E00U   // Majority Voting Threshold
#define SDFM_SDCOMP1EVT2FLTCTL_FILINIT     0x8000U   // Filter Initialization Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP1EVT2FLTCLKCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP1EVT2FLTCLKCTL_CLKPRESCALE_S   0U
#define SDFM_SDCOMP1EVT2FLTCLKCTL_CLKPRESCALE_M   0x3FFU   // Sample Clock Prescale

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP1EVT1FLTCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP1EVT1FLTCTL_SAMPWIN_S   4U
#define SDFM_SDCOMP1EVT1FLTCTL_SAMPWIN_M   0x1F0U    // Sample Window
#define SDFM_SDCOMP1EVT1FLTCTL_THRESH_S    9U
#define SDFM_SDCOMP1EVT1FLTCTL_THRESH_M    0x3E00U   // Majority Voting Threshold
#define SDFM_SDCOMP1EVT1FLTCTL_FILINIT     0x8000U   // Filter Initialization Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP1EVT1FLTCLKCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP1EVT1FLTCLKCTL_CLKPRESCALE_S   0U
#define SDFM_SDCOMP1EVT1FLTCLKCTL_CLKPRESCALE_M   0x3FFU   // Sample Clock Prescale

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP1LOCK register
//
//*************************************************************************************************
#define SDFM_SDCOMP1LOCK_SDCOMP1CTL   0x1U   // COMPCTL Lock
#define SDFM_SDCOMP1LOCK_COMP         0x8U   // COMPevent filter registers Lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP2CTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP2CTL_CEVT1DIGFILTSEL_S   2U
#define SDFM_SDCOMP2CTL_CEVT1DIGFILTSEL_M   0xCU     // High Comparator Trip Select
#define SDFM_SDCOMP2CTL_CEVT2DIGFILTSEL_S   10U
#define SDFM_SDCOMP2CTL_CEVT2DIGFILTSEL_M   0xC00U   // Low Comparator Trip Select

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP2EVT2FLTCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP2EVT2FLTCTL_SAMPWIN_S   4U
#define SDFM_SDCOMP2EVT2FLTCTL_SAMPWIN_M   0x1F0U    // Sample Window
#define SDFM_SDCOMP2EVT2FLTCTL_THRESH_S    9U
#define SDFM_SDCOMP2EVT2FLTCTL_THRESH_M    0x3E00U   // Majority Voting Threshold
#define SDFM_SDCOMP2EVT2FLTCTL_FILINIT     0x8000U   // Filter Initialization Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP2EVT2FLTCLKCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP2EVT2FLTCLKCTL_CLKPRESCALE_S   0U
#define SDFM_SDCOMP2EVT2FLTCLKCTL_CLKPRESCALE_M   0x3FFU   // Sample Clock Prescale

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP2EVT1FLTCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP2EVT1FLTCTL_SAMPWIN_S   4U
#define SDFM_SDCOMP2EVT1FLTCTL_SAMPWIN_M   0x1F0U    // Sample Window
#define SDFM_SDCOMP2EVT1FLTCTL_THRESH_S    9U
#define SDFM_SDCOMP2EVT1FLTCTL_THRESH_M    0x3E00U   // Majority Voting Threshold
#define SDFM_SDCOMP2EVT1FLTCTL_FILINIT     0x8000U   // Filter Initialization Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP2EVT1FLTCLKCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP2EVT1FLTCLKCTL_CLKPRESCALE_S   0U
#define SDFM_SDCOMP2EVT1FLTCLKCTL_CLKPRESCALE_M   0x3FFU   // Sample Clock Prescale

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP2LOCK register
//
//*************************************************************************************************
#define SDFM_SDCOMP2LOCK_SDCOMP2CTL   0x1U   // COMPCTL Lock
#define SDFM_SDCOMP2LOCK_COMP         0x8U   // COMPevent filter registers Lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP3CTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP3CTL_CEVT1DIGFILTSEL_S   2U
#define SDFM_SDCOMP3CTL_CEVT1DIGFILTSEL_M   0xCU     // High Comparator Trip Select
#define SDFM_SDCOMP3CTL_CEVT2DIGFILTSEL_S   10U
#define SDFM_SDCOMP3CTL_CEVT2DIGFILTSEL_M   0xC00U   // Low Comparator Trip Select

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP3EVT2FLTCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP3EVT2FLTCTL_SAMPWIN_S   4U
#define SDFM_SDCOMP3EVT2FLTCTL_SAMPWIN_M   0x1F0U    // Sample Window
#define SDFM_SDCOMP3EVT2FLTCTL_THRESH_S    9U
#define SDFM_SDCOMP3EVT2FLTCTL_THRESH_M    0x3E00U   // Majority Voting Threshold
#define SDFM_SDCOMP3EVT2FLTCTL_FILINIT     0x8000U   // Filter Initialization Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP3EVT2FLTCLKCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP3EVT2FLTCLKCTL_CLKPRESCALE_S   0U
#define SDFM_SDCOMP3EVT2FLTCLKCTL_CLKPRESCALE_M   0x3FFU   // Sample Clock Prescale

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP3EVT1FLTCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP3EVT1FLTCTL_SAMPWIN_S   4U
#define SDFM_SDCOMP3EVT1FLTCTL_SAMPWIN_M   0x1F0U    // Sample Window
#define SDFM_SDCOMP3EVT1FLTCTL_THRESH_S    9U
#define SDFM_SDCOMP3EVT1FLTCTL_THRESH_M    0x3E00U   // Majority Voting Threshold
#define SDFM_SDCOMP3EVT1FLTCTL_FILINIT     0x8000U   // Filter Initialization Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP3EVT1FLTCLKCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP3EVT1FLTCLKCTL_CLKPRESCALE_S   0U
#define SDFM_SDCOMP3EVT1FLTCLKCTL_CLKPRESCALE_M   0x3FFU   // Sample Clock Prescale

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP3LOCK register
//
//*************************************************************************************************
#define SDFM_SDCOMP3LOCK_SDCOMP3CTL   0x1U   // COMPCTL Lock
#define SDFM_SDCOMP3LOCK_COMP         0x8U   // COMPevent filter registers Lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP4CTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP4CTL_CEVT1DIGFILTSEL_S   2U
#define SDFM_SDCOMP4CTL_CEVT1DIGFILTSEL_M   0xCU     // High Comparator Trip Select
#define SDFM_SDCOMP4CTL_CEVT2DIGFILTSEL_S   10U
#define SDFM_SDCOMP4CTL_CEVT2DIGFILTSEL_M   0xC00U   // Low Comparator Trip Select

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP4EVT2FLTCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP4EVT2FLTCTL_SAMPWIN_S   4U
#define SDFM_SDCOMP4EVT2FLTCTL_SAMPWIN_M   0x1F0U    // Sample Window
#define SDFM_SDCOMP4EVT2FLTCTL_THRESH_S    9U
#define SDFM_SDCOMP4EVT2FLTCTL_THRESH_M    0x3E00U   // Majority Voting Threshold
#define SDFM_SDCOMP4EVT2FLTCTL_FILINIT     0x8000U   // Filter Initialization Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP4EVT2FLTCLKCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP4EVT2FLTCLKCTL_CLKPRESCALE_S   0U
#define SDFM_SDCOMP4EVT2FLTCLKCTL_CLKPRESCALE_M   0x3FFU   // Sample Clock Prescale

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP4EVT1FLTCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP4EVT1FLTCTL_SAMPWIN_S   4U
#define SDFM_SDCOMP4EVT1FLTCTL_SAMPWIN_M   0x1F0U    // Sample Window
#define SDFM_SDCOMP4EVT1FLTCTL_THRESH_S    9U
#define SDFM_SDCOMP4EVT1FLTCTL_THRESH_M    0x3E00U   // Majority Voting Threshold
#define SDFM_SDCOMP4EVT1FLTCTL_FILINIT     0x8000U   // Filter Initialization Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP4EVT1FLTCLKCTL register
//
//*************************************************************************************************
#define SDFM_SDCOMP4EVT1FLTCLKCTL_CLKPRESCALE_S   0U
#define SDFM_SDCOMP4EVT1FLTCLKCTL_CLKPRESCALE_M   0x3FFU   // Sample Clock Prescale

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDCOMP4LOCK register
//
//*************************************************************************************************
#define SDFM_SDCOMP4LOCK_SDCOMP4CTL   0x1U   // COMPCTL Lock
#define SDFM_SDCOMP4LOCK_COMP         0x8U   // COMPevent filter registers Lock



#endif
