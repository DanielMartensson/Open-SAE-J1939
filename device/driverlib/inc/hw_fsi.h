//###########################################################################
//
// FILE:    hw_fsi.h
//
// TITLE:   Definitions for the FSI registers.
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

#ifndef HW_FSI_H
#define HW_FSI_H

//*************************************************************************************************
//
// The following are defines for the FSI register offsets
//
//*************************************************************************************************
#define FSI_O_TX_MASTER_CTRL       0x0U            // Transmit master control register
#define FSI_O_TX_CLK_CTRL          0x2U            // Transmit clock control register
#define FSI_O_TX_OPER_CTRL_LO      0x4U            // Transmit operation control register low
#define FSI_O_TX_OPER_CTRL_HI      0x5U            // Transmit operation control register high
#define FSI_O_TX_FRAME_CTRL        0x6U            // Transmit frame control register
#define FSI_O_TX_FRAME_TAG_UDATA   0x7U            // Transmit frame tag and user data register
#define FSI_O_TX_BUF_PTR_LOAD      0x8U            // Transmit buffer pointer control load register
#define FSI_O_TX_BUF_PTR_STS       0x9U            // Transmit buffer pointer control status
                                                   // register
#define FSI_O_TX_PING_CTRL         0xAU            // Transmit ping control register
#define FSI_O_TX_PING_TAG          0xBU            // Transmit ping tag register
#define FSI_O_TX_PING_TO_REF       0xCU            // Transmit ping timeout counter reference
#define FSI_O_TX_PING_TO_CNT       0xEU            // Transmit ping timeout current count
#define FSI_O_TX_INT_CTRL          0x10U           // Transmit interrupt event control register
#define FSI_O_TX_DMA_CTRL          0x11U           // Transmit DMA event control register
#define FSI_O_TX_LOCK_CTRL         0x12U           // Transmit lock control register
#define FSI_O_TX_EVT_STS           0x14U           // Transmit event and error status flag register
#define FSI_O_TX_EVT_CLR           0x16U           // Transmit event and error clear register
#define FSI_O_TX_EVT_FRC           0x17U           // Transmit event and error flag force register
#define FSI_O_TX_USER_CRC          0x18U           // Transmit user-defined CRC register
#define FSI_O_TX_ECC_DATA          0x20U           // Transmit ECC data register
#define FSI_O_TX_ECC_VAL           0x22U           // Transmit ECC value register
#define FSI_O_TX_BUF_BASE(i)       (0x40U + (i))   // (0 <= i < 16) Base address for transmit
                                                   // buffer

#define FSI_O_RX_MASTER_CTRL       0x0U            // Receive main control register
#define FSI_O_RX_OPER_CTRL         0x4U            // Receive operation control register
#define FSI_O_RX_FRAME_INFO        0x6U            // Receive frame control register
#define FSI_O_RX_FRAME_TAG_UDATA   0x7U            // Receive frame tag and user data register
#define FSI_O_RX_DMA_CTRL          0x8U            // Receive DMA event control register
#define FSI_O_RX_EVT_STS           0xAU            // Receive event and error status flag register
#define FSI_O_RX_CRC_INFO          0xBU            // Receive CRC info of received and computed CRC
#define FSI_O_RX_EVT_CLR           0xCU            // Receive event and error clear register
#define FSI_O_RX_EVT_FRC           0xDU            // Receive event and error flag force register
#define FSI_O_RX_BUF_PTR_LOAD      0xEU            // Receive buffer pointer load register
#define FSI_O_RX_BUF_PTR_STS       0xFU            // Receive buffer pointer status register
#define FSI_O_RX_FRAME_WD_CTRL     0x10U           // Receive frame watchdog control register
#define FSI_O_RX_FRAME_WD_REF      0x12U           // Receive frame watchdog counter reference
#define FSI_O_RX_FRAME_WD_CNT      0x14U           // Receive frame watchdog current count
#define FSI_O_RX_PING_WD_CTRL      0x16U           // Receive ping watchdog control register
#define FSI_O_RX_PING_TAG          0x17U           // Receive ping tag register
#define FSI_O_RX_PING_WD_REF       0x18U           // Receive ping watchdog counter reference
#define FSI_O_RX_PING_WD_CNT       0x1AU           // Receive pingwatchdog current count
#define FSI_O_RX_INT1_CTRL         0x1CU           // Receive interrupt control register for
                                                   // RX_INT1
#define FSI_O_RX_INT2_CTRL         0x1DU           // Receive interrupt control register for
                                                   // RX_INT2
#define FSI_O_RX_LOCK_CTRL         0x1EU           // Receive lock control register
#define FSI_O_RX_ECC_DATA          0x20U           // Receive ECC data register
#define FSI_O_RX_ECC_VAL           0x22U           // Receive ECC value register
#define FSI_O_RX_ECC_SEC_DATA      0x24U           // Receive ECC corrected data register
#define FSI_O_RX_ECC_LOG           0x26U           // Receive ECC log and status register
#define FSI_O_RX_FRAME_TAG_CMP     0x28U           // Receive frame tag compare register
#define FSI_O_RX_PING_TAG_CMP      0x29U           // Receive ping tag compare register
#define FSI_O_RX_DLYLINE_CTRL      0x30U           // Receive delay line control register
#define FSI_O_RX_VIS_1             0x38U           // Receive debug visibility register 1
#define FSI_O_RX_BUF_BASE(i)       (0x40U + (i))   // (0 <= i < 16) Base address for receive data
                                                   // buffer


//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_MASTER_CTRL register
//
//*************************************************************************************************
#define FSI_TX_MASTER_CTRL_CORE_RST   0x1U      // Transmitter Main Core Reset
#define FSI_TX_MASTER_CTRL_FLUSH      0x2U      // Flush Operation Start
#define FSI_TX_MASTER_CTRL_KEY_S      8U
#define FSI_TX_MASTER_CTRL_KEY_M      0xFF00U   // Write Key

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_CLK_CTRL register
//
//*************************************************************************************************
#define FSI_TX_CLK_CTRL_CLK_RST          0x1U     // Soft Reset for the Clock Divider
#define FSI_TX_CLK_CTRL_CLK_EN           0x2U     // Clock Divider Enable
#define FSI_TX_CLK_CTRL_PRESCALE_VAL_S   2U
#define FSI_TX_CLK_CTRL_PRESCALE_VAL_M   0x3FCU   // Prescale value

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_OPER_CTRL_LO register
//
//*************************************************************************************************
#define FSI_TX_OPER_CTRL_LO_DATA_WIDTH_S   0U
#define FSI_TX_OPER_CTRL_LO_DATA_WIDTH_M   0x3U     // Transmit Data width
#define FSI_TX_OPER_CTRL_LO_SPI_MODE       0x4U     // SPI Mode Select
#define FSI_TX_OPER_CTRL_LO_START_MODE_S   3U
#define FSI_TX_OPER_CTRL_LO_START_MODE_M   0x38U    // Transmission Start Mode Select
#define FSI_TX_OPER_CTRL_LO_SW_CRC         0x40U    // CRC Source Select
#define FSI_TX_OPER_CTRL_LO_PING_TO_MODE   0x80U    // Ping Counter Reset Mode Select
#define FSI_TX_OPER_CTRL_LO_SEL_PLLCLK     0x100U   // Input Clock Select
#define FSI_TX_OPER_CTRL_LO_TDM_ENABLE     0x200U   // Transmit TDM Mode Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_OPER_CTRL_HI register
//
//*************************************************************************************************
#define FSI_TX_OPER_CTRL_HI_FORCE_ERR        0x20U     // Error Frame Force
#define FSI_TX_OPER_CTRL_HI_ECC_SEL          0x40U     // ECC Data Width Select
#define FSI_TX_OPER_CTRL_HI_EXT_TRIG_SEL_S   7U
#define FSI_TX_OPER_CTRL_HI_EXT_TRIG_SEL_M   0x1F80U   // External Trigger Select

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_FRAME_CTRL register
//
//*************************************************************************************************
#define FSI_TX_FRAME_CTRL_FRAME_TYPE_S   0U
#define FSI_TX_FRAME_CTRL_FRAME_TYPE_M   0xFU      // Transmit Frame Type
#define FSI_TX_FRAME_CTRL_N_WORDS_S      4U
#define FSI_TX_FRAME_CTRL_N_WORDS_M      0xF0U     // Number of Words to be Transmitted
#define FSI_TX_FRAME_CTRL_START          0x8000U   // Start Transmission

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_FRAME_TAG_UDATA register
//
//*************************************************************************************************
#define FSI_TX_FRAME_TAG_UDATA_FRAME_TAG_S   0U
#define FSI_TX_FRAME_TAG_UDATA_FRAME_TAG_M   0xFU      // Frame Tag
#define FSI_TX_FRAME_TAG_UDATA_USER_DATA_S   8U
#define FSI_TX_FRAME_TAG_UDATA_USER_DATA_M   0xFF00U   // User Data

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_BUF_PTR_LOAD register
//
//*************************************************************************************************
#define FSI_TX_BUF_PTR_LOAD_BUF_PTR_LOAD_S   0U
#define FSI_TX_BUF_PTR_LOAD_BUF_PTR_LOAD_M   0xFU   // Buffer Pointer Force Load

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_BUF_PTR_STS register
//
//*************************************************************************************************
#define FSI_TX_BUF_PTR_STS_CURR_BUF_PTR_S    0U
#define FSI_TX_BUF_PTR_STS_CURR_BUF_PTR_M    0xFU      // Current Buffer Pointer Index
#define FSI_TX_BUF_PTR_STS_CURR_WORD_CNT_S   8U
#define FSI_TX_BUF_PTR_STS_CURR_WORD_CNT_M   0x1F00U   // Remaining Words in Buffer

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_PING_CTRL register
//
//*************************************************************************************************
#define FSI_TX_PING_CTRL_CNT_RST          0x1U     // Ping Counter Reset
#define FSI_TX_PING_CTRL_TIMER_EN         0x2U     // Ping Counter Enable
#define FSI_TX_PING_CTRL_EXT_TRIG_EN      0x4U     // External Trigger Enable
#define FSI_TX_PING_CTRL_EXT_TRIG_SEL_S   3U
#define FSI_TX_PING_CTRL_EXT_TRIG_SEL_M   0x1F8U   // External Trigger Select

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_PING_TAG register
//
//*************************************************************************************************
#define FSI_TX_PING_TAG_TAG_S   0U
#define FSI_TX_PING_TAG_TAG_M   0xFU   // Ping Frame Tag

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_INT_CTRL register
//
//*************************************************************************************************
#define FSI_TX_INT_CTRL_INT1_EN_FRAME_DONE     0x1U     // Enable Frame Done Interrupt to INT1
#define FSI_TX_INT_CTRL_INT1_EN_BUF_UNDERRUN   0x2U     // Enable Buffer Underrun Interrupt to INT1
#define FSI_TX_INT_CTRL_INT1_EN_BUF_OVERRUN    0x4U     // Enable Buffer Overrun Interrupt to INT1
#define FSI_TX_INT_CTRL_INT1_EN_PING_TO        0x8U     // Enable Ping Timer Interrupt to INT1
#define FSI_TX_INT_CTRL_INT2_EN_FRAME_DONE     0x100U   // Enable Frame Done Interrupt to INT2
#define FSI_TX_INT_CTRL_INT2_EN_BUF_UNDERRUN   0x200U   // Enable Buffer Underrun Interrupt to INT2
#define FSI_TX_INT_CTRL_INT2_EN_BUF_OVERRUN    0x400U   // Enable Buffer Overrun Interrupt to INT2
#define FSI_TX_INT_CTRL_INT2_EN_PING_TO        0x800U   // Enable Ping Timer Interrupt to INT2

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_DMA_CTRL register
//
//*************************************************************************************************
#define FSI_TX_DMA_CTRL_DMA_EVT_EN   0x1U   // DMA Event Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_LOCK_CTRL register
//
//*************************************************************************************************
#define FSI_TX_LOCK_CTRL_LOCK    0x1U      // Control Register Lock Enable
#define FSI_TX_LOCK_CTRL_KEY_S   8U
#define FSI_TX_LOCK_CTRL_KEY_M   0xFF00U   // Write Key

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_EVT_STS register
//
//*************************************************************************************************
#define FSI_TX_EVT_STS_FRAME_DONE       0x1U   // Frame Done Flag
#define FSI_TX_EVT_STS_BUF_UNDERRUN     0x2U   // Buffer Underrun Flag
#define FSI_TX_EVT_STS_BUF_OVERRUN      0x4U   // Buffer Overrun Flag
#define FSI_TX_EVT_STS_PING_TRIGGERED   0x8U   // Ping Frame Triggered Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_EVT_CLR register
//
//*************************************************************************************************
#define FSI_TX_EVT_CLR_FRAME_DONE       0x1U   // Frame Done Flag Clear
#define FSI_TX_EVT_CLR_BUF_UNDERRUN     0x2U   // Buffer Underrun Flag Clear
#define FSI_TX_EVT_CLR_BUF_OVERRUN      0x4U   // Buffer Overrun Flag Clear
#define FSI_TX_EVT_CLR_PING_TRIGGERED   0x8U   // Ping Frame Triggered Flag Clear

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_EVT_FRC register
//
//*************************************************************************************************
#define FSI_TX_EVT_FRC_FRAME_DONE       0x1U   // Frame Done Flag Force
#define FSI_TX_EVT_FRC_BUF_UNDERRUN     0x2U   // Buffer Underrun Flag Force
#define FSI_TX_EVT_FRC_BUF_OVERRUN      0x4U   // Buffer Overrun Flag Force
#define FSI_TX_EVT_FRC_PING_TRIGGERED   0x8U   // Ping Frame Triggered Flag Force

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_USER_CRC register
//
//*************************************************************************************************
#define FSI_TX_USER_CRC_USER_CRC_S   0U
#define FSI_TX_USER_CRC_USER_CRC_M   0xFFU   // User-defined CRC

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_ECC_DATA register
//
//*************************************************************************************************
#define FSI_TX_ECC_DATA_DATA_LOW_S    0U
#define FSI_TX_ECC_DATA_DATA_LOW_M    0xFFFFU       // ECC Data Lower 16 Bits
#define FSI_TX_ECC_DATA_DATA_HIGH_S   16U
#define FSI_TX_ECC_DATA_DATA_HIGH_M   0xFFFF0000U   // ECC Data Upper 16 Bits

//*************************************************************************************************
//
// The following are defines for the bit fields in the TX_ECC_VAL register
//
//*************************************************************************************************
#define FSI_TX_ECC_VAL_ECC_VAL_S   0U
#define FSI_TX_ECC_VAL_ECC_VAL_M   0x7FU   // Computed ECC Value


//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_MASTER_CTRL register
//
//*************************************************************************************************
#define FSI_RX_MASTER_CTRL_CORE_RST       0x1U      // Receiver Main Core Reset
#define FSI_RX_MASTER_CTRL_INT_LOOPBACK   0x2U      // Internal Loopback Enable
#define FSI_RX_MASTER_CTRL_SPI_PAIRING    0x4U      // Clock Pairing for SPI-like Behaviour
#define FSI_RX_MASTER_CTRL_KEY_S          8U
#define FSI_RX_MASTER_CTRL_KEY_M          0xFF00U   // Write Key

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_OPER_CTRL register
//
//*************************************************************************************************
#define FSI_RX_OPER_CTRL_DATA_WIDTH_S       0U
#define FSI_RX_OPER_CTRL_DATA_WIDTH_M       0x3U     // Receive Data Width Select
#define FSI_RX_OPER_CTRL_SPI_MODE           0x4U     // SPI Mode Enable
#define FSI_RX_OPER_CTRL_N_WORDS_S          3U
#define FSI_RX_OPER_CTRL_N_WORDS_M          0x78U    // Number of Words to be Received
#define FSI_RX_OPER_CTRL_ECC_SEL            0x80U    // ECC Data Width Select
#define FSI_RX_OPER_CTRL_PING_WD_RST_MODE   0x100U   // Ping Watchdog Timeout Mode Select

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_FRAME_INFO register
//
//*************************************************************************************************
#define FSI_RX_FRAME_INFO_FRAME_TYPE_S   0U
#define FSI_RX_FRAME_INFO_FRAME_TYPE_M   0xFU   // Received Frame Type

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_FRAME_TAG_UDATA register
//
//*************************************************************************************************
#define FSI_RX_FRAME_TAG_UDATA_FRAME_TAG_S   1U
#define FSI_RX_FRAME_TAG_UDATA_FRAME_TAG_M   0x1EU     // Received Frame Tag
#define FSI_RX_FRAME_TAG_UDATA_USER_DATA_S   8U
#define FSI_RX_FRAME_TAG_UDATA_USER_DATA_M   0xFF00U   // Received User Data

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_DMA_CTRL register
//
//*************************************************************************************************
#define FSI_RX_DMA_CTRL_DMA_EVT_EN   0x1U   // DMA Event Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_EVT_STS register
//
//*************************************************************************************************
#define FSI_RX_EVT_STS_PING_WD_TO        0x1U      // Ping Watchdog Timeout Flag
#define FSI_RX_EVT_STS_FRAME_WD_TO       0x2U      // Frame Watchdog Timeout Flag.
#define FSI_RX_EVT_STS_CRC_ERR           0x4U      // CRC Error Flag
#define FSI_RX_EVT_STS_TYPE_ERR          0x8U      // Frame Type Error Flag
#define FSI_RX_EVT_STS_EOF_ERR           0x10U     // End-of-Frame Error Flag
#define FSI_RX_EVT_STS_BUF_OVERRUN       0x20U     // Receive Buffer Overrun Flag
#define FSI_RX_EVT_STS_FRAME_DONE        0x40U     // Frame Done Flag
#define FSI_RX_EVT_STS_BUF_UNDERRUN      0x80U     // Receive Buffer Underrun Flag
#define FSI_RX_EVT_STS_ERR_FRAME         0x100U    // Error Frame Received Flag
#define FSI_RX_EVT_STS_PING_FRAME        0x200U    // Ping Frame Received Flag
#define FSI_RX_EVT_STS_FRAME_OVERRUN     0x400U    // Frame Overrun Flag
#define FSI_RX_EVT_STS_DATA_FRAME        0x800U    // Data Frame Received Flag
#define FSI_RX_EVT_STS_PING_TAG_MATCH    0x1000U   // Ping Tag Match Flag
#define FSI_RX_EVT_STS_DATA_TAG_MATCH    0x2000U   // Data Tag Match Flag
#define FSI_RX_EVT_STS_ERROR_TAG_MATCH   0x4000U   // Error Tag Match Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_CRC_INFO register
//
//*************************************************************************************************
#define FSI_RX_CRC_INFO_RX_CRC_S     0U
#define FSI_RX_CRC_INFO_RX_CRC_M     0xFFU     // Received CRC Value
#define FSI_RX_CRC_INFO_CALC_CRC_S   8U
#define FSI_RX_CRC_INFO_CALC_CRC_M   0xFF00U   // Hardware Calculated CRC

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_EVT_CLR register
//
//*************************************************************************************************
#define FSI_RX_EVT_CLR_PING_WD_TO        0x1U      // Ping Watchdog Timeout Flag Clear
#define FSI_RX_EVT_CLR_FRAME_WD_TO       0x2U      // Frame Watchdog Timeout Flag Clear
#define FSI_RX_EVT_CLR_CRC_ERR           0x4U      // CRC Error Flag Clear
#define FSI_RX_EVT_CLR_TYPE_ERR          0x8U      // Frame Type Error Flag Clear
#define FSI_RX_EVT_CLR_EOF_ERR           0x10U     // End-of-Frame Error Flag Clear
#define FSI_RX_EVT_CLR_BUF_OVERRUN       0x20U     // Receive Buffer Overrun Flag Clear
#define FSI_RX_EVT_CLR_FRAME_DONE        0x40U     // Frame Done Flag Clear
#define FSI_RX_EVT_CLR_BUF_UNDERRUN      0x80U     // Receive Buffer Underrun Flag Clear
#define FSI_RX_EVT_CLR_ERR_FRAME         0x100U    // Error Frame Received Flag Clear
#define FSI_RX_EVT_CLR_PING_FRAME        0x200U    // PING Frame Received Flag Clear
#define FSI_RX_EVT_CLR_FRAME_OVERRUN     0x400U    // Frame Overrun Flag Clear
#define FSI_RX_EVT_CLR_DATA_FRAME        0x800U    // Data Frame Received Flag Clear
#define FSI_RX_EVT_CLR_PING_TAG_MATCH    0x1000U   // Ping Tag Match Flag Clear
#define FSI_RX_EVT_CLR_DATA_TAG_MATCH    0x2000U   // Data Tag Match Flag Clear
#define FSI_RX_EVT_CLR_ERROR_TAG_MATCH   0x4000U   // Error Tag Match Flag Clear

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_EVT_FRC register
//
//*************************************************************************************************
#define FSI_RX_EVT_FRC_PING_WD_TO        0x1U      // Ping Watchdog Timeout Flag Force
#define FSI_RX_EVT_FRC_FRAME_WD_TO       0x2U      // Frame Watchdog Timeout Flag Force
#define FSI_RX_EVT_FRC_CRC_ERR           0x4U      // CRC Error Flag Force
#define FSI_RX_EVT_FRC_TYPE_ERR          0x8U      // Frame Type Error Flag Force
#define FSI_RX_EVT_FRC_EOF_ERR           0x10U     // End-of-Frame Error Flag Force
#define FSI_RX_EVT_FRC_BUF_OVERRUN       0x20U     // Receive Buffer Overrun Flag Force
#define FSI_RX_EVT_FRC_FRAME_DONE        0x40U     // Frame Done Flag Force
#define FSI_RX_EVT_FRC_BUF_UNDERRUN      0x80U     // Receive Buffer Underrun Flag Force
#define FSI_RX_EVT_FRC_ERR_FRAME         0x100U    // Error Frame Received Flag Force
#define FSI_RX_EVT_FRC_PING_FRAME        0x200U    // Ping Frame Received Flag Force
#define FSI_RX_EVT_FRC_FRAME_OVERRUN     0x400U    // Frame Overrun Flag Force
#define FSI_RX_EVT_FRC_DATA_FRAME        0x800U    // Data Frame Received Flag Force
#define FSI_RX_EVT_FRC_PING_TAG_MATCH    0x1000U   // Ping Tag Match Flag Force
#define FSI_RX_EVT_FRC_DATA_TAG_MATCH    0x2000U   // Data Tag Match Flag Force
#define FSI_RX_EVT_FRC_ERROR_TAG_MATCH   0x4000U   // Error Tag Match Flag Force

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_BUF_PTR_LOAD register
//
//*************************************************************************************************
#define FSI_RX_BUF_PTR_LOAD_BUF_PTR_LOAD_S   0U
#define FSI_RX_BUF_PTR_LOAD_BUF_PTR_LOAD_M   0xFU   // Load value for receive buffer pointer

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_BUF_PTR_STS register
//
//*************************************************************************************************
#define FSI_RX_BUF_PTR_STS_CURR_BUF_PTR_S    0U
#define FSI_RX_BUF_PTR_STS_CURR_BUF_PTR_M    0xFU      // Current Buffer Pointer Index
#define FSI_RX_BUF_PTR_STS_CURR_WORD_CNT_S   8U
#define FSI_RX_BUF_PTR_STS_CURR_WORD_CNT_M   0x1F00U   // Available Words in Buffer

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_FRAME_WD_CTRL register
//
//*************************************************************************************************
#define FSI_RX_FRAME_WD_CTRL_FRAME_WD_CNT_RST   0x1U   // Frame Watchdog Counter Reset
#define FSI_RX_FRAME_WD_CTRL_FRAME_WD_EN        0x2U   // Frame Watchdog Counter Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_PING_WD_CTRL register
//
//*************************************************************************************************
#define FSI_RX_PING_WD_CTRL_PING_WD_RST   0x1U   // Ping Watchdog Counter Reset
#define FSI_RX_PING_WD_CTRL_PING_WD_EN    0x2U   // Ping Watchdog Counter Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_PING_TAG register
//
//*************************************************************************************************
#define FSI_RX_PING_TAG_PING_TAG_S   1U
#define FSI_RX_PING_TAG_PING_TAG_M   0x1EU   // Ping Frame Tag

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_INT1_CTRL register
//
//*************************************************************************************************
#define FSI_RX_INT1_CTRL_INT1_EN_PING_WD_TO        0x1U      // Enable Ping Watchdog Timeout
                                                             // Interrupt to INT1
#define FSI_RX_INT1_CTRL_INT1_EN_FRAME_WD_TO       0x2U      // Enable Frame Watchdog Timeout
                                                             // Interrupt to INT1
#define FSI_RX_INT1_CTRL_INT1_EN_CRC_ERR           0x4U      // Enable CRC Error Interrupt to INT1
#define FSI_RX_INT1_CTRL_INT1_EN_TYPE_ERR          0x8U      // Enable Frame Type Error Interrupt
                                                             // to INT1
#define FSI_RX_INT1_CTRL_INT1_EN_EOF_ERR           0x10U     // Enable End-of-Frame Error Interrupt
                                                             // to INT1
#define FSI_RX_INT1_CTRL_INT1_EN_OVERRUN           0x20U     // Enable Receive Buffer Overrun
                                                             // Interrupt to INT1
#define FSI_RX_INT1_CTRL_INT1_EN_FRAME_DONE        0x40U     // Enable Frame Done Interrupt to INT1
#define FSI_RX_INT1_CTRL_INT1_EN_UNDERRUN          0x80U     // Enable Buffer Underrun Interrupt to
                                                             // INT1
#define FSI_RX_INT1_CTRL_INT1_EN_ERR_FRAME         0x100U    // Enable Error Frame Received
                                                             // Interrupt to INT1
#define FSI_RX_INT1_CTRL_INT1_EN_PING_FRAME        0x200U    // Enable Ping Frame Received
                                                             // Interrupt to INT1
#define FSI_RX_INT1_CTRL_INT1_EN_FRAME_OVERRUN     0x400U    // Enable Frame Overrun Interrupt to
                                                             // INT1
#define FSI_RX_INT1_CTRL_INT1_EN_DATA_FRAME        0x800U    // Enable Data Frame Received
                                                             // Interrupt to INT1
#define FSI_RX_INT1_CTRL_INT1_EN_PING_TAG_MATCH    0x1000U   // Enable Ping Frame Tag Matched
                                                             // Interrupt to INT1
#define FSI_RX_INT1_CTRL_INT1_EN_DATA_TAG_MATCH    0x2000U   // Enable Data Frame Tag Matched
                                                             // Interrupt to INT1
#define FSI_RX_INT1_CTRL_INT1_EN_ERROR_TAG_MATCH   0x4000U   // Enable Error Frame Tag Matched
                                                             // Interrupt to INT1

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_INT2_CTRL register
//
//*************************************************************************************************
#define FSI_RX_INT2_CTRL_INT2_EN_PING_WD_TO        0x1U      // Enable Ping Watchdog Timeout
                                                             // Interrupt to INT2
#define FSI_RX_INT2_CTRL_INT2_EN_FRAME_WD_TO       0x2U      // Enable Frame Watchdog Timeout
                                                             // Interrupt to INT2
#define FSI_RX_INT2_CTRL_INT2_EN_CRC_ERR           0x4U      // Enable CRC Errror Interrupt to INT2
#define FSI_RX_INT2_CTRL_INT2_EN_TYPE_ERR          0x8U      // Enable Frame Type Error Interrupt
                                                             // to INT2
#define FSI_RX_INT2_CTRL_INT2_EN_EOF_ERR           0x10U     // Enable End-of-Frame Error Interrupt
                                                             // to INT2
#define FSI_RX_INT2_CTRL_INT2_EN_OVERRUN           0x20U     // Enable Buffer Overrun Interrupt to
                                                             // INT2
#define FSI_RX_INT2_CTRL_INT2_EN_FRAME_DONE        0x40U     // Enable Frame Done Interrupt to INT2
#define FSI_RX_INT2_CTRL_INT2_EN_UNDERRUN          0x80U     // Enable Buffer Underrun Interrupt to
                                                             // INT2
#define FSI_RX_INT2_CTRL_INT2_EN_ERR_FRAME         0x100U    // Enable Error Frame Received
                                                             // Interrupt to INT2
#define FSI_RX_INT2_CTRL_INT2_EN_PING_FRAME        0x200U    // Enable Ping Frame Received
                                                             // Interrupt to INT2
#define FSI_RX_INT2_CTRL_INT2_EN_FRAME_OVERRUN     0x400U    // Enable Frame Overrun Interrupt to
                                                             // INT2
#define FSI_RX_INT2_CTRL_INT2_EN_DATA_FRAME        0x800U    // Enable Data Frame Received
                                                             // Interrupt to INT2
#define FSI_RX_INT2_CTRL_INT2_EN_PING_TAG_MATCH    0x1000U   // Enable Ping Frame Tag Matched
                                                             // Interrupt to INT2
#define FSI_RX_INT2_CTRL_INT2_EN_DATA_TAG_MATCH    0x2000U   // Enable Data Frame Tag Matched
                                                             // Interrupt to INT2
#define FSI_RX_INT2_CTRL_INT2_EN_ERROR_TAG_MATCH   0x4000U   // Enable Error Frame Tag Matched
                                                             // Interrupt to INT2

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_LOCK_CTRL register
//
//*************************************************************************************************
#define FSI_RX_LOCK_CTRL_LOCK    0x1U      // Control Register Lock Enable
#define FSI_RX_LOCK_CTRL_KEY_S   8U
#define FSI_RX_LOCK_CTRL_KEY_M   0xFF00U   // Write Key

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_ECC_DATA register
//
//*************************************************************************************************
#define FSI_RX_ECC_DATA_DATA_LOW_S    0U
#define FSI_RX_ECC_DATA_DATA_LOW_M    0xFFFFU       // ECC Data Lower 16 Bits
#define FSI_RX_ECC_DATA_DATA_HIGH_S   16U
#define FSI_RX_ECC_DATA_DATA_HIGH_M   0xFFFF0000U   // ECC Data Upper 16 Bits

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_ECC_VAL register
//
//*************************************************************************************************
#define FSI_RX_ECC_VAL_ECC_VAL_S   0U
#define FSI_RX_ECC_VAL_ECC_VAL_M   0x7FU   // Computed ECC Value

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_ECC_LOG register
//
//*************************************************************************************************
#define FSI_RX_ECC_LOG_SBE   0x1U   // Single Bit Error Detected
#define FSI_RX_ECC_LOG_MBE   0x2U   // Multiple Bit Errors Detected

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_FRAME_TAG_CMP register
//
//*************************************************************************************************
#define FSI_RX_FRAME_TAG_CMP_TAG_REF_S      0U
#define FSI_RX_FRAME_TAG_CMP_TAG_REF_M      0xFU     // Frame Tag Reference
#define FSI_RX_FRAME_TAG_CMP_TAG_MASK_S     4U
#define FSI_RX_FRAME_TAG_CMP_TAG_MASK_M     0xF0U    // Frame Tag Mask
#define FSI_RX_FRAME_TAG_CMP_CMP_EN         0x100U   // Frame Tag Compare Enable
#define FSI_RX_FRAME_TAG_CMP_BROADCAST_EN   0x200U   // Broadcast Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_PING_TAG_CMP register
//
//*************************************************************************************************
#define FSI_RX_PING_TAG_CMP_TAG_REF_S      0U
#define FSI_RX_PING_TAG_CMP_TAG_REF_M      0xFU     // Ping Tag Reference
#define FSI_RX_PING_TAG_CMP_TAG_MASK_S     4U
#define FSI_RX_PING_TAG_CMP_TAG_MASK_M     0xF0U    // Ping Tag Mask
#define FSI_RX_PING_TAG_CMP_CMP_EN         0x100U   // Ping Tag Compare Enable
#define FSI_RX_PING_TAG_CMP_BROADCAST_EN   0x200U   // Broadcast Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_DLYLINE_CTRL register
//
//*************************************************************************************************
#define FSI_RX_DLYLINE_CTRL_RXCLK_DLY_S   0U
#define FSI_RX_DLYLINE_CTRL_RXCLK_DLY_M   0x1FU     // Delay Line Tap Select for RXCLK
#define FSI_RX_DLYLINE_CTRL_RXD0_DLY_S    5U
#define FSI_RX_DLYLINE_CTRL_RXD0_DLY_M    0x3E0U    // Delay Line Tap Select for RXD0
#define FSI_RX_DLYLINE_CTRL_RXD1_DLY_S    10U
#define FSI_RX_DLYLINE_CTRL_RXD1_DLY_M    0x7C00U   // Delay Line Tap Select for RXD1

//*************************************************************************************************
//
// The following are defines for the bit fields in the RX_VIS_1 register
//
//*************************************************************************************************
#define FSI_RX_VIS_1_RX_CORE_STS   0x8U   // Receiver Core Status



#endif
