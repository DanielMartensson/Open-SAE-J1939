//###########################################################################
//
// FILE:    hw_can.h
//
// TITLE:   Definitions for the CAN registers.
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

#ifndef HW_CAN_H
#define HW_CAN_H

//*************************************************************************************************
//
// The following are defines for the CAN register offsets
//
//*************************************************************************************************
#define CAN_O_CTL           0x0U     // CAN Control Register
#define CAN_O_ES            0x4U     // Error and Status Register
#define CAN_O_ERRC          0x8U     // Error Counter Register
#define CAN_O_BTR           0xCU     // Bit Timing Register
#define CAN_O_INT           0x10U    // Interrupt Register
#define CAN_O_TEST          0x14U    // Test Register
#define CAN_O_PERR          0x1CU    // CAN Parity Error Code Register
#define CAN_O_RAM_INIT      0x40U    // CAN RAM Initialization Register
#define CAN_O_GLB_INT_EN    0x50U    // CAN Global Interrupt Enable Register
#define CAN_O_GLB_INT_FLG   0x54U    // CAN Global Interrupt Flag Register
#define CAN_O_GLB_INT_CLR   0x58U    // CAN Global Interrupt Clear Register
#define CAN_O_ABOTR         0x80U    // Auto-Bus-On Time Register
#define CAN_O_TXRQ_X        0x84U    // CAN Transmission Request Register
#define CAN_O_TXRQ_21       0x88U    // CAN Transmission Request 2_1 Register
#define CAN_O_NDAT_X        0x98U    // CAN New Data Register
#define CAN_O_NDAT_21       0x9CU    // CAN New Data 2_1 Register
#define CAN_O_IPEN_X        0xACU    // CAN Interrupt Pending Register
#define CAN_O_IPEN_21       0xB0U    // CAN Interrupt Pending 2_1 Register
#define CAN_O_MVAL_X        0xC0U    // CAN Message Valid Register
#define CAN_O_MVAL_21       0xC4U    // CAN Message Valid 2_1 Register
#define CAN_O_IP_MUX21      0xD8U    // CAN Interrupt Multiplexer 2_1 Register
#define CAN_O_IF1CMD        0x100U   // IF1 Command Register
#define CAN_O_IF1MSK        0x104U   // IF1 Mask Register
#define CAN_O_IF1ARB        0x108U   // IF1 Arbitration Register
#define CAN_O_IF1MCTL       0x10CU   // IF1 Message Control Register
#define CAN_O_IF1DATA       0x110U   // IF1 Data A Register
#define CAN_O_IF1DATB       0x114U   // IF1 Data B Register
#define CAN_O_IF2CMD        0x120U   // IF2 Command Register
#define CAN_O_IF2MSK        0x124U   // IF2 Mask Register
#define CAN_O_IF2ARB        0x128U   // IF2 Arbitration Register
#define CAN_O_IF2MCTL       0x12CU   // IF2 Message Control Register
#define CAN_O_IF2DATA       0x130U   // IF2 Data A Register
#define CAN_O_IF2DATB       0x134U   // IF2 Data B Register
#define CAN_O_IF3OBS        0x140U   // IF3 Observation Register
#define CAN_O_IF3MSK        0x144U   // IF3 Mask Register
#define CAN_O_IF3ARB        0x148U   // IF3 Arbitration Register
#define CAN_O_IF3MCTL       0x14CU   // IF3 Message Control Register
#define CAN_O_IF3DATA       0x150U   // IF3 Data A Register
#define CAN_O_IF3DATB       0x154U   // IF3 Data B Register
#define CAN_O_IF3UPD        0x160U   // IF3 Update Enable Register


//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_CTL register
//
//*************************************************************************************************
#define CAN_CTL_INIT      0x1U        // Initialization
#define CAN_CTL_IE0       0x2U        // Interrupt line 0 Enable
#define CAN_CTL_SIE       0x4U        // Status Change Interrupt Enable
#define CAN_CTL_EIE       0x8U        // Error Interrupt Enable
#define CAN_CTL_DAR       0x20U       // Disable Automatic Retransmission
#define CAN_CTL_CCE       0x40U       // Configuration Change Enable
#define CAN_CTL_TEST      0x80U       // Test Mode Enable
#define CAN_CTL_IDS       0x100U      // Interruption Debug Support Enable
#define CAN_CTL_ABO       0x200U      // Auto-Bus-On Enable
#define CAN_CTL_PMD_S     10U
#define CAN_CTL_PMD_M     0x3C00U     // Parity on/off
#define CAN_CTL_SWR       0x8000U     // SW Reset Enable
#define CAN_CTL_INITDBG   0x10000U    // Debug Mode Status
#define CAN_CTL_IE1       0x20000U    // Interrupt line 1 Enable Disabled
#define CAN_CTL_DE1       0x40000U    // Enable DMA request line
#define CAN_CTL_DE2       0x80000U    // Enable DMA request line
#define CAN_CTL_DE3       0x100000U   // Enable DMA request line

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_ES register
//
//*************************************************************************************************
#define CAN_ES_LEC_S   0U
#define CAN_ES_LEC_M   0x7U     // Last Error Code
#define CAN_ES_TXOK    0x8U     // Transmission status
#define CAN_ES_RXOK    0x10U    // Reception status
#define CAN_ES_EPASS   0x20U    // Error Passive State
#define CAN_ES_EWARN   0x40U    // Warning State
#define CAN_ES_BOFF    0x80U    // Bus-Off State
#define CAN_ES_PER     0x100U   // Parity Error Detected

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_ERRC register
//
//*************************************************************************************************
#define CAN_ERRC_TEC_S   0U
#define CAN_ERRC_TEC_M   0xFFU     // Transmit Error Counter
#define CAN_ERRC_REC_S   8U
#define CAN_ERRC_REC_M   0x7F00U   // Receive Error Counter
#define CAN_ERRC_RP      0x8000U   // Receive Error Passive

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_BTR register
//
//*************************************************************************************************
#define CAN_BTR_BRP_S     0U
#define CAN_BTR_BRP_M     0x3FU      // Baud Rate Prescaler
#define CAN_BTR_SJW_S     6U
#define CAN_BTR_SJW_M     0xC0U      // Synchronization Jump Width
#define CAN_BTR_TSEG1_S   8U
#define CAN_BTR_TSEG1_M   0xF00U     // Time segment
#define CAN_BTR_TSEG2_S   12U
#define CAN_BTR_TSEG2_M   0x7000U    // Time segment
#define CAN_BTR_BRPE_S    16U
#define CAN_BTR_BRPE_M    0xF0000U   // Baud Rate Prescaler Extension

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_INT register
//
//*************************************************************************************************
#define CAN_INT_INT0ID_S   0U
#define CAN_INT_INT0ID_M   0xFFFFU     // Interrupt Identifier
#define CAN_INT_INT1ID_S   16U
#define CAN_INT_INT1ID_M   0xFF0000U   // Interrupt 1 Identifier

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_TEST register
//
//*************************************************************************************************
#define CAN_TEST_SILENT   0x8U     // Silent Mode
#define CAN_TEST_LBACK    0x10U    // Loopback Mode
#define CAN_TEST_TX_S     5U
#define CAN_TEST_TX_M     0x60U    // CANTX Pin Control
#define CAN_TEST_RX       0x80U    // CANRX Pin Status
#define CAN_TEST_EXL      0x100U   // External Loopback Mode
#define CAN_TEST_RDA      0x200U   // RAM Direct Access Enable:

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_PERR register
//
//*************************************************************************************************
#define CAN_PERR_MSG_NUM_S    0U
#define CAN_PERR_MSG_NUM_M    0xFFU    // Message Number
#define CAN_PERR_WORD_NUM_S   8U
#define CAN_PERR_WORD_NUM_M   0x700U   // Word Number

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_RAM_INIT register
//
//*************************************************************************************************
#define CAN_RAM_INIT_KEY0            0x1U    // KEY0
#define CAN_RAM_INIT_KEY1            0x2U    // KEY1
#define CAN_RAM_INIT_KEY2            0x4U    // KEY2
#define CAN_RAM_INIT_KEY3            0x8U    // KEY3
#define CAN_RAM_INIT_CAN_RAM_INIT    0x10U   // Initialize CAN Mailbox RAM
#define CAN_RAM_INIT_RAM_INIT_DONE   0x20U   // CAN RAM initialization complete

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_GLB_INT_EN register
//
//*************************************************************************************************
#define CAN_GLB_INT_EN_GLBINT0_EN   0x1U   // Global Interrupt Enable for  CANINT0
#define CAN_GLB_INT_EN_GLBINT1_EN   0x2U   // Global Interrupt Enable for  CANINT1

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_GLB_INT_FLG register
//
//*************************************************************************************************
#define CAN_GLB_INT_FLG_INT0_FLG   0x1U   // Global Interrupt Flag for CANINT0
#define CAN_GLB_INT_FLG_INT1_FLG   0x2U   // Global Interrupt Flag for CANINT1

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_GLB_INT_CLR register
//
//*************************************************************************************************
#define CAN_GLB_INT_CLR_INT0_FLG_CLR   0x1U   // Global Interrupt flag clear for CANINT0
#define CAN_GLB_INT_CLR_INT1_FLG_CLR   0x2U   // Global Interrupt flag clear for CANINT1

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_TXRQ_X register
//
//*************************************************************************************************
#define CAN_TXRQ_X_TXRQSTREG1_S   0U
#define CAN_TXRQ_X_TXRQSTREG1_M   0x3U   // Transmit Request Register 1
#define CAN_TXRQ_X_TXRQSTREG2_S   2U
#define CAN_TXRQ_X_TXRQSTREG2_M   0xCU   // Transmit Request Register 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_NDAT_X register
//
//*************************************************************************************************
#define CAN_NDAT_X_NEWDATREG1_S   0U
#define CAN_NDAT_X_NEWDATREG1_M   0x3U   // New Data Register 1
#define CAN_NDAT_X_NEWDATREG2_S   2U
#define CAN_NDAT_X_NEWDATREG2_M   0xCU   // New Data Register 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IPEN_X register
//
//*************************************************************************************************
#define CAN_IPEN_X_INTPNDREG1_S   0U
#define CAN_IPEN_X_INTPNDREG1_M   0x3U   // Interrupt Pending Register 1
#define CAN_IPEN_X_INTPNDREG2_S   2U
#define CAN_IPEN_X_INTPNDREG2_M   0xCU   // Interrupt Pending Register 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_MVAL_X register
//
//*************************************************************************************************
#define CAN_MVAL_X_MSGVALREG1_S   0U
#define CAN_MVAL_X_MSGVALREG1_M   0x3U   // Message Valid Register 1
#define CAN_MVAL_X_MSGVALREG2_S   2U
#define CAN_MVAL_X_MSGVALREG2_M   0xCU   // Message Valid Register 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF1CMD register
//
//*************************************************************************************************
#define CAN_IF1CMD_MSG_NUM_S   0U
#define CAN_IF1CMD_MSG_NUM_M   0xFFU       // Message Number
#define CAN_IF1CMD_DMAACTIVE   0x4000U     // DMA Status
#define CAN_IF1CMD_BUSY        0x8000U     // Busy Flag
#define CAN_IF1CMD_DATA_B      0x10000U    // Access Data Bytes 4-7
#define CAN_IF1CMD_DATA_A      0x20000U    // Access Data Bytes 0-3
#define CAN_IF1CMD_TXRQST      0x40000U    // Access Transmission Request Bit
#define CAN_IF1CMD_CLRINTPND   0x80000U    // Clear Interrupt Pending Bit
#define CAN_IF1CMD_CONTROL     0x100000U   // Access Control Bits
#define CAN_IF1CMD_ARB         0x200000U   // Access Arbitration Bits
#define CAN_IF1CMD_MASK        0x400000U   // Access Mask Bits
#define CAN_IF1CMD_DIR         0x800000U   // Write/Read Direction

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF1MSK register
//
//*************************************************************************************************
#define CAN_IF1MSK_MSK_S   0U
#define CAN_IF1MSK_MSK_M   0x1FFFFFFFU   // Identifier Mask
#define CAN_IF1MSK_MDIR    0x40000000U   // Mask Message Direction
#define CAN_IF1MSK_MXTD    0x80000000U   // Mask Extended Identifier

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF1ARB register
//
//*************************************************************************************************
#define CAN_IF1ARB_ID_S     0U
#define CAN_IF1ARB_ID_M     0x1FFFFFFFU   // `
#define CAN_IF1ARB_DIR      0x20000000U   // Message Direction
#define CAN_IF1ARB_XTD      0x40000000U   // Extended Identifier
#define CAN_IF1ARB_MSGVAL   0x80000000U   // Message Valid

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF1MCTL register
//
//*************************************************************************************************
#define CAN_IF1MCTL_DLC_S    0U
#define CAN_IF1MCTL_DLC_M    0xFU      // Data length code
#define CAN_IF1MCTL_EOB      0x80U     // End of Block
#define CAN_IF1MCTL_TXRQST   0x100U    // Transmit Request
#define CAN_IF1MCTL_RMTEN    0x200U    // Remote Enable
#define CAN_IF1MCTL_RXIE     0x400U    // Receive Interrupt Enable
#define CAN_IF1MCTL_TXIE     0x800U    // Transmit Interrupt Enable
#define CAN_IF1MCTL_UMASK    0x1000U   // Use Acceptance Mask
#define CAN_IF1MCTL_INTPND   0x2000U   // Interrupt Pending
#define CAN_IF1MCTL_MSGLST   0x4000U   // Message Lost
#define CAN_IF1MCTL_NEWDAT   0x8000U   // New Data

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF1DATA register
//
//*************************************************************************************************
#define CAN_IF1DATA_DATA_0_S   0U
#define CAN_IF1DATA_DATA_0_M   0xFFU         // Data Byte 0
#define CAN_IF1DATA_DATA_1_S   8U
#define CAN_IF1DATA_DATA_1_M   0xFF00U       // Data Byte 1
#define CAN_IF1DATA_DATA_2_S   16U
#define CAN_IF1DATA_DATA_2_M   0xFF0000U     // Data Byte 2
#define CAN_IF1DATA_DATA_3_S   24U
#define CAN_IF1DATA_DATA_3_M   0xFF000000U   // Data Byte 3

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF1DATB register
//
//*************************************************************************************************
#define CAN_IF1DATB_DATA_4_S   0U
#define CAN_IF1DATB_DATA_4_M   0xFFU         // Data Byte 4
#define CAN_IF1DATB_DATA_5_S   8U
#define CAN_IF1DATB_DATA_5_M   0xFF00U       // Data Byte 5
#define CAN_IF1DATB_DATA_6_S   16U
#define CAN_IF1DATB_DATA_6_M   0xFF0000U     // Data Byte 6
#define CAN_IF1DATB_DATA_7_S   24U
#define CAN_IF1DATB_DATA_7_M   0xFF000000U   // Data Byte 7

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF2CMD register
//
//*************************************************************************************************
#define CAN_IF2CMD_MSG_NUM_S   0U
#define CAN_IF2CMD_MSG_NUM_M   0xFFU       // Message Number
#define CAN_IF2CMD_DMAACTIVE   0x4000U     // DMA Status
#define CAN_IF2CMD_BUSY        0x8000U     // Busy Flag
#define CAN_IF2CMD_DATA_B      0x10000U    // Access Data Bytes 4-7
#define CAN_IF2CMD_DATA_A      0x20000U    // Access Data Bytes 0-3
#define CAN_IF2CMD_TXRQST      0x40000U    // Access Transmission Request Bit
#define CAN_IF2CMD_CLRINTPND   0x80000U    // Clear Interrupt Pending Bit
#define CAN_IF2CMD_CONTROL     0x100000U   // Access Control Bits
#define CAN_IF2CMD_ARB         0x200000U   // Access Arbitration Bits
#define CAN_IF2CMD_MASK        0x400000U   // Access Mask Bits
#define CAN_IF2CMD_DIR         0x800000U   // Write/Read Direction

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF2MSK register
//
//*************************************************************************************************
#define CAN_IF2MSK_MSK_S   0U
#define CAN_IF2MSK_MSK_M   0x1FFFFFFFU   // Identifier Mask
#define CAN_IF2MSK_MDIR    0x40000000U   // Mask Message Direction
#define CAN_IF2MSK_MXTD    0x80000000U   // Mask Extended Identifier

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF2ARB register
//
//*************************************************************************************************
#define CAN_IF2ARB_ID_S     0U
#define CAN_IF2ARB_ID_M     0x1FFFFFFFU   // Message  Identifier
#define CAN_IF2ARB_DIR      0x20000000U   // Message Direction
#define CAN_IF2ARB_XTD      0x40000000U   // Extended Identifier
#define CAN_IF2ARB_MSGVAL   0x80000000U   // Message Valid

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF2MCTL register
//
//*************************************************************************************************
#define CAN_IF2MCTL_DLC_S    0U
#define CAN_IF2MCTL_DLC_M    0xFU      // Data length code
#define CAN_IF2MCTL_EOB      0x80U     // End of Block
#define CAN_IF2MCTL_TXRQST   0x100U    // Transmit Request
#define CAN_IF2MCTL_RMTEN    0x200U    // Remote Enable
#define CAN_IF2MCTL_RXIE     0x400U    // Receive Interrupt Enable
#define CAN_IF2MCTL_TXIE     0x800U    // Transmit Interrupt Enable
#define CAN_IF2MCTL_UMASK    0x1000U   // Use Acceptance Mask
#define CAN_IF2MCTL_INTPND   0x2000U   // Interrupt Pending
#define CAN_IF2MCTL_MSGLST   0x4000U   // Message Lost
#define CAN_IF2MCTL_NEWDAT   0x8000U   // New Data

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF2DATA register
//
//*************************************************************************************************
#define CAN_IF2DATA_DATA_0_S   0U
#define CAN_IF2DATA_DATA_0_M   0xFFU         // Data Byte 0
#define CAN_IF2DATA_DATA_1_S   8U
#define CAN_IF2DATA_DATA_1_M   0xFF00U       // Data Byte 1
#define CAN_IF2DATA_DATA_2_S   16U
#define CAN_IF2DATA_DATA_2_M   0xFF0000U     // Data Byte 2
#define CAN_IF2DATA_DATA_3_S   24U
#define CAN_IF2DATA_DATA_3_M   0xFF000000U   // Data Byte 3

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF2DATB register
//
//*************************************************************************************************
#define CAN_IF2DATB_DATA_4_S   0U
#define CAN_IF2DATB_DATA_4_M   0xFFU         // Data Byte 4
#define CAN_IF2DATB_DATA_5_S   8U
#define CAN_IF2DATB_DATA_5_M   0xFF00U       // Data Byte 5
#define CAN_IF2DATB_DATA_6_S   16U
#define CAN_IF2DATB_DATA_6_M   0xFF0000U     // Data Byte 6
#define CAN_IF2DATB_DATA_7_S   24U
#define CAN_IF2DATB_DATA_7_M   0xFF000000U   // Data Byte 7

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF3OBS register
//
//*************************************************************************************************
#define CAN_IF3OBS_MASK     0x1U      // Mask data read observation
#define CAN_IF3OBS_ARB      0x2U      // Arbitration data read observation
#define CAN_IF3OBS_CTRL     0x4U      // Ctrl read observation
#define CAN_IF3OBS_DATA_A   0x8U      // Data A read observation
#define CAN_IF3OBS_DATA_B   0x10U     // Data B read observation
#define CAN_IF3OBS_IF3SM    0x100U    // IF3 Status of Mask data read access
#define CAN_IF3OBS_IF3SA    0x200U    // IF3 Status of Arbitration data read access
#define CAN_IF3OBS_IF3SC    0x400U    // IF3 Status of Control bits read access
#define CAN_IF3OBS_IF3SDA   0x800U    // IF3 Status of Data A read access
#define CAN_IF3OBS_IF3SDB   0x1000U   // IF3 Status of Data B read access
#define CAN_IF3OBS_IF3UPD   0x8000U   // IF3 Update Data

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF3MSK register
//
//*************************************************************************************************
#define CAN_IF3MSK_MSK_S   0U
#define CAN_IF3MSK_MSK_M   0x1FFFFFFFU   // Mask
#define CAN_IF3MSK_MDIR    0x40000000U   // Mask Message Direction
#define CAN_IF3MSK_MXTD    0x80000000U   // Mask Extended Identifier

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF3ARB register
//
//*************************************************************************************************
#define CAN_IF3ARB_ID_S     0U
#define CAN_IF3ARB_ID_M     0x1FFFFFFFU   // Message  Identifier
#define CAN_IF3ARB_DIR      0x20000000U   // Message Direction
#define CAN_IF3ARB_XTD      0x40000000U   // Extended Identifier
#define CAN_IF3ARB_MSGVAL   0x80000000U   // Message Valid

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF3MCTL register
//
//*************************************************************************************************
#define CAN_IF3MCTL_DLC_S    0U
#define CAN_IF3MCTL_DLC_M    0xFU      // Data length code
#define CAN_IF3MCTL_EOB      0x80U     // End of Block
#define CAN_IF3MCTL_TXRQST   0x100U    // Transmit Request
#define CAN_IF3MCTL_RMTEN    0x200U    // Remote Enable
#define CAN_IF3MCTL_RXIE     0x400U    // Receive Interrupt Enable
#define CAN_IF3MCTL_TXIE     0x800U    // Transmit Interrupt Enable
#define CAN_IF3MCTL_UMASK    0x1000U   // Use Acceptance Mask
#define CAN_IF3MCTL_INTPND   0x2000U   // Interrupt Pending
#define CAN_IF3MCTL_MSGLST   0x4000U   // Message Lost
#define CAN_IF3MCTL_NEWDAT   0x8000U   // New Data

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF3DATA register
//
//*************************************************************************************************
#define CAN_IF3DATA_DATA_0_S   0U
#define CAN_IF3DATA_DATA_0_M   0xFFU         // Data Byte 0
#define CAN_IF3DATA_DATA_1_S   8U
#define CAN_IF3DATA_DATA_1_M   0xFF00U       // Data Byte 1
#define CAN_IF3DATA_DATA_2_S   16U
#define CAN_IF3DATA_DATA_2_M   0xFF0000U     // Data Byte 2
#define CAN_IF3DATA_DATA_3_S   24U
#define CAN_IF3DATA_DATA_3_M   0xFF000000U   // Data Byte 3

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_IF3DATB register
//
//*************************************************************************************************
#define CAN_IF3DATB_DATA_4_S   0U
#define CAN_IF3DATB_DATA_4_M   0xFFU         // Data Byte 4
#define CAN_IF3DATB_DATA_5_S   8U
#define CAN_IF3DATB_DATA_5_M   0xFF00U       // Data Byte 5
#define CAN_IF3DATB_DATA_6_S   16U
#define CAN_IF3DATB_DATA_6_M   0xFF0000U     // Data Byte 6
#define CAN_IF3DATB_DATA_7_S   24U
#define CAN_IF3DATB_DATA_7_M   0xFF000000U   // Data Byte 7



#endif
