//###########################################################################
//
// FILE:    hw_esc_ss.h
//
// TITLE:   Definitions for the ESCSS registers.
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

#ifndef HW_ESC_SS_H
#define HW_ESC_SS_H

//*************************************************************************************************
//
// The following are defines for the ESCSS register offsets
//
//*************************************************************************************************
#define ESCSS_O_IPREVNUM            0x0U    // IP Revision Number
#define ESCSS_O_INTR_RIS            0x2U    // EtherCATSS Interrupt Raw Status
#define ESCSS_O_INTR_MASK           0x4U    // EtherCATSS Interrupt Mask
#define ESCSS_O_INTR_MIS            0x6U    // EtherCATSS Masked Interrupt Status
#define ESCSS_O_INTR_CLR            0x8U    // EtherCATSS Interrupt Clear
#define ESCSS_O_INTR_SET            0xAU    // EtherCATSS Interrupt Set to emulate
#define ESCSS_O_LATCH_SEL           0xCU    // Select for Latch0/1 inputs and LATCHIN input
#define ESCSS_O_ACCESS_CTRL         0xEU    // PDI interface access control config.
#define ESCSS_O_GPIN_DAT            0x10U   // GPIN data capture for debug & override
#define ESCSS_O_GPIN_PIPE           0x12U   // GPIN pipeline select
#define ESCSS_O_GPIN_GRP_CAP_SEL    0x14U   // GPIN pipe group capture trigger
#define ESCSS_O_GPOUT_DAT           0x16U   // GPOUT data capture for debug & override
#define ESCSS_O_GPOUT_PIPE          0x18U   // GPOUT pipeline select
#define ESCSS_O_GPOUT_GRP_CAP_SEL   0x1AU   // GPOUT pipe group capture trigger
#define ESCSS_O_MEM_TEST            0x1CU   // Memory Test Control
#define ESCSS_O_RESET_DEST_CONFIG   0x1EU   // ResetOut impact or destination config
#define ESCSS_O_SYNC0_CONFIG        0x20U   // SYNC0 Configuration for various triggers
#define ESCSS_O_SYNC1_CONFIG        0x22U   // SYNC1 Configuration for various triggers

#define ESCSS_O_CONFIG_LOCK       0x0U    // EtherCATSS Configuration Lock
#define ESCSS_O_MISC_IO_CONFIG    0x2U    // RESET_IN, EEPROM IO connections select
#define ESCSS_O_PHY_IO_CONFIG     0x4U    // Control Register of ESCSS
#define ESCSS_O_SYNC_IO_CONFIG    0x6U    // SYNC Signals IO configurations
#define ESCSS_O_LATCH_IO_CONFIG   0x8U    // LATCH inputs IO pad select
#define ESCSS_O_GPIN_SEL          0xAU    // GPIN Select between IO PAD & tieoff
#define ESCSS_O_GPIN_IOPAD_SEL    0xCU    // GPIN IO pad Select
#define ESCSS_O_GPOUT_SEL         0xEU    // GPOUT IO pad connect select
#define ESCSS_O_GPOUT_IOPAD_SEL   0x10U   // GPOUT IO pad select
#define ESCSS_O_LED_CONFIG        0x12U   // Selection of LED o/p connect to IO pad
#define ESCSS_O_MISC_CONFIG       0x14U   // Miscelleneous Configuration


//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_IPREVNUM register
//
//*************************************************************************************************
#define ESCSS_IPREVNUM_IP_REV_MINOR_S   0U
#define ESCSS_IPREVNUM_IP_REV_MINOR_M   0xFU    // Minor IP Revision Number
#define ESCSS_IPREVNUM_IP_REV_MAJOR_S   4U
#define ESCSS_IPREVNUM_IP_REV_MAJOR_M   0xF0U   // Major IP Revision Number

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_INTR_RIS register
//
//*************************************************************************************************
#define ESCSS_INTR_RIS_SYNC0_RIS          0x1U    // SYNC0 feature RIS
#define ESCSS_INTR_RIS_SYNC1_RIS          0x2U    // SYNC1 feature RIS
#define ESCSS_INTR_RIS_IRQ_RIS            0x4U    // EtherCATSS IRQ RIS
#define ESCSS_INTR_RIS_DMA_DONE_RIS       0x8U    // DMA Done RIS
#define ESCSS_INTR_RIS_TIMEOUT_ERR_RIS    0x10U   // PDI bus Timeout Error RIS
#define ESCSS_INTR_RIS_MASTER_RESET_RIS   0x20U   // ECAT RESET RIS

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_INTR_MASK register
//
//*************************************************************************************************
#define ESCSS_INTR_MASK_SYNC0_MASK          0x1U    // SYNC0 feature Mask
#define ESCSS_INTR_MASK_SYNC1_MASK          0x2U    // SYNC1 feature Mask
#define ESCSS_INTR_MASK_IRQ_MASK            0x4U    // EtherCATSS IRQ Mask
#define ESCSS_INTR_MASK_DMA_DONE_MASK       0x8U    // DMA Done Mask
#define ESCSS_INTR_MASK_TIMEOUT_ERR_MASK    0x10U   // PDI Access Timeout Error Mask
#define ESCSS_INTR_MASK_MASTER_RESET_MASK   0x20U   // EtherCAT Master Reset Mask

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_INTR_MIS register
//
//*************************************************************************************************
#define ESCSS_INTR_MIS_SYNC0_MIS          0x1U    // SYNC0 feature MIS
#define ESCSS_INTR_MIS_SYNC1_MIS          0x2U    // SYNC1 feature MIS
#define ESCSS_INTR_MIS_IRQ_MIS            0x4U    // EtherCATSS IRQ MIS
#define ESCSS_INTR_MIS_DMA_DONE_MIS       0x8U    // DMA Done MIS
#define ESCSS_INTR_MIS_TIMEOUT_ERR_MIS    0x10U   // PDI bus Timeout Error MIS
#define ESCSS_INTR_MIS_MASTER_RESET_MIS   0x20U   // EtherCAT Master Reset MIS

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_INTR_CLR register
//
//*************************************************************************************************
#define ESCSS_INTR_CLR_SYNC0_CLR          0x1U    // SYNC0 feature Clear
#define ESCSS_INTR_CLR_SYNC1_CLR          0x2U    // SYNC1 feature Clear
#define ESCSS_INTR_CLR_IRQ_CLR            0x4U    // EtherCATSS IRQ Clear
#define ESCSS_INTR_CLR_DMA_DONE_CLR       0x8U    // DMA Done Clear
#define ESCSS_INTR_CLR_TIMEOUT_ERR_CLR    0x10U   // PDI Access Timeout Error Clear
#define ESCSS_INTR_CLR_MASTER_RESET_CLR   0x20U   // EtherCAT Master Reset Clear

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_INTR_SET register
//
//*************************************************************************************************
#define ESCSS_INTR_SET_SYNC0_SET          0x1U      // SYNC0 Set Emulate
#define ESCSS_INTR_SET_SYNC1_SET          0x2U      // SYNC1 Set Emulate
#define ESCSS_INTR_SET_IRQ_SET            0x4U      // EtherCATSS IRQ Set Emulate
#define ESCSS_INTR_SET_DMA_DONE_SET       0x8U      // DMA Done Set Emulate
#define ESCSS_INTR_SET_TIMEOUT_ERR_SET    0x10U     // PDI Access Timeout Error Set Emulate
#define ESCSS_INTR_SET_MASTER_RESET_SET   0x20U     // EtherCAT Master Reset Emulate
#define ESCSS_INTR_SET_WRITE_KEY_S        8U
#define ESCSS_INTR_SET_WRITE_KEY_M        0xFF00U   // Key to enable writing lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_LATCH_SEL register
//
//*************************************************************************************************
#define ESCSS_LATCH_SEL_LATCH0_SELECT_S   0U
#define ESCSS_LATCH_SEL_LATCH0_SELECT_M   0x1FU     // LATCH0 Inputs mux select
#define ESCSS_LATCH_SEL_LATCH1_SELECT_S   8U
#define ESCSS_LATCH_SEL_LATCH1_SELECT_M   0x1F00U   // LATCH1 Inputs mux select

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_ACCESS_CTRL register
//
//*************************************************************************************************
#define ESCSS_ACCESS_CTRL_WAIT_STATES_S                 0U
#define ESCSS_ACCESS_CTRL_WAIT_STATES_M                 0x7FU        // Minimum Wait States for
                                                                     // VBUS Bridge
#define ESCSS_ACCESS_CTRL_EN_TIMEOUT                    0x80U        // PDI Timeout enable
#define ESCSS_ACCESS_CTRL_ENABLE_DEBUG_ACCESS           0x200U       // Debug access enable
#define ESCSS_ACCESS_CTRL_ENABLE_PARALLEL_PORT_ACCESS   0x400U       // Parallel port access enable
#define ESCSS_ACCESS_CTRL_TIMEOUT_COUNT_S               16U
#define ESCSS_ACCESS_CTRL_TIMEOUT_COUNT_M               0xFFF0000U   // Max timecount programmed
                                                                     // and count while enabled.

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_GPIN_GRP_CAP_SEL register
//
//*************************************************************************************************
#define ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL0_S   0U
#define ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL0_M   0x7U      // GPI7-0 capture trigger select
#define ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL1_S   4U
#define ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL1_M   0x70U     // GPI15-8 capture trigger select
#define ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL2_S   8U
#define ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL2_M   0x700U    // GPI23-16 capture trigger select
#define ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL3_S   12U
#define ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL3_M   0x7000U   // GPI31-24 capture trigger select

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_GPOUT_GRP_CAP_SEL register
//
//*************************************************************************************************
#define ESCSS_GPOUT_GRP_CAP_SEL_GPO_GRP_CAP_SEL0_S   0U
#define ESCSS_GPOUT_GRP_CAP_SEL_GPO_GRP_CAP_SEL0_M   0x3U      // GPO7-0 capture trigger select
#define ESCSS_GPOUT_GRP_CAP_SEL_GPO_GRP_CAP_SEL1_S   4U
#define ESCSS_GPOUT_GRP_CAP_SEL_GPO_GRP_CAP_SEL1_M   0x30U     // GPO15-8 capture trigger select
#define ESCSS_GPOUT_GRP_CAP_SEL_GPO_GRP_CAP_SEL2_S   8U
#define ESCSS_GPOUT_GRP_CAP_SEL_GPO_GRP_CAP_SEL2_M   0x300U    // GPO23-16 capture trigger select
#define ESCSS_GPOUT_GRP_CAP_SEL_GPO_GRP_CAP_SEL3_S   12U
#define ESCSS_GPOUT_GRP_CAP_SEL_GPO_GRP_CAP_SEL3_M   0x3000U   // GPO31-24 capture trigger select

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_MEM_TEST register
//
//*************************************************************************************************
#define ESCSS_MEM_TEST_INITIATE_MEM_INIT   0x1U   // Initialize memory init
#define ESCSS_MEM_TEST_MEM_INIT_DONE       0x2U   // Memory Init done status

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_RESET_DEST_CONFIG register
//
//*************************************************************************************************
#define ESCSS_RESET_DEST_CONFIG_CPU_RESET_EN      0x1U      // CPU reset enable for ResetOut
#define ESCSS_RESET_DEST_CONFIG_CPU_NMI_EN        0x2U      // CPU NMI enable for ResetOut
#define ESCSS_RESET_DEST_CONFIG_CPU_INT_EN        0x4U      // CPU Interrupt enable for ResetOut
#define ESCSS_RESET_DEST_CONFIG_DEVICE_RESET_EN   0x80U     // Enables RESET_OUT to impact the
                                                            // device reset
#define ESCSS_RESET_DEST_CONFIG_WRITE_KEY_S       8U
#define ESCSS_RESET_DEST_CONFIG_WRITE_KEY_M       0xFF00U   // Key to enable writing lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_SYNC0_CONFIG register
//
//*************************************************************************************************
#define ESCSS_SYNC0_CONFIG_C28X_PIE_EN    0x1U      // Connects the SYNC0 to C28x PIE Interrupt
#define ESCSS_SYNC0_CONFIG_CLA_INT_EN     0x2U      // Connects the SYNC0 to CLA Interrupt
#define ESCSS_SYNC0_CONFIG_C28X_DMA_EN    0x4U      // Connects the SYNC0 to C28x DMA Trigger
#define ESCSS_SYNC0_CONFIG_CM4_NVIC_EN    0x8U      // Connects the SYNC0 to CM4 NVIC Interrupt
#define ESCSS_SYNC0_CONFIG_UDMA_TRIG_EN   0x10U     // Connects the SYNC0 to uDMA Trigger
#define ESCSS_SYNC0_CONFIG_WRITE_KEY_S    8U
#define ESCSS_SYNC0_CONFIG_WRITE_KEY_M    0xFF00U   // Key to enable writing lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_SYNC1_CONFIG register
//
//*************************************************************************************************
#define ESCSS_SYNC1_CONFIG_C28X_PIE_EN    0x1U      // Connects the SYNC1 to C28x PIE Interrupt
#define ESCSS_SYNC1_CONFIG_CLA_INT_EN     0x2U      // Connects the SYNC1 to CLA Interrupt
#define ESCSS_SYNC1_CONFIG_C28X_DMA_EN    0x4U      // Connects the SYNC1 to C28x DMA Trigger
#define ESCSS_SYNC1_CONFIG_CM4_NVIC_EN    0x8U      // Connects the SYNC1 to CM4 NVIC Interrupt
#define ESCSS_SYNC1_CONFIG_UDMA_TRIG_EN   0x10U     // Connects the SYNC1 to uDMA Trigger
#define ESCSS_SYNC1_CONFIG_WRITE_KEY_S    8U
#define ESCSS_SYNC1_CONFIG_WRITE_KEY_M    0xFF00U   // Key to enable writing lock


//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_CONFIG_LOCK register
//
//*************************************************************************************************
#define ESCSS_CONFIG_LOCK_LOCK_ENABLE        0x1U      // Locking writes to ECATSS
#define ESCSS_CONFIG_LOCK_IO_CONFIG_ENABLE   0x10U     // Locking the IO Configuration
#define ESCSS_CONFIG_LOCK_WRITE_KEY_S        8U
#define ESCSS_CONFIG_LOCK_WRITE_KEY_M        0xFF00U   // Key to enable writing lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_MISC_IO_CONFIG register
//
//*************************************************************************************************
#define ESCSS_MISC_IO_CONFIG_RESETIN_GPIO_EN    0x1U      // Enabled ResetIN from GPIO
#define ESCSS_MISC_IO_CONFIG_EEPROM_I2C_IO_EN   0x2U      // Enables the EEPROM I2C IOPAD
                                                          // connection
#define ESCSS_MISC_IO_CONFIG_WRITE_KEY_S        8U
#define ESCSS_MISC_IO_CONFIG_WRITE_KEY_M        0xFF00U   // Key to enable writing lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_PHY_IO_CONFIG register
//
//*************************************************************************************************
#define ESCSS_PHY_IO_CONFIG_PHY_PORT_CNT_S         2U
#define ESCSS_PHY_IO_CONFIG_PHY_PORT_CNT_M         0xCU      // Number of PHY port counts
#define ESCSS_PHY_IO_CONFIG_PHY_INTF_IOPAD_SEL_S   4U
#define ESCSS_PHY_IO_CONFIG_PHY_INTF_IOPAD_SEL_M   0x30U     // IO Combination select for PHY
                                                             // Interface
#define ESCSS_PHY_IO_CONFIG_TX_CLK_AUTO_COMP       0x40U     // Selects TX_CLK IO to do Auto
                                                             // compensation
#define ESCSS_PHY_IO_CONFIG_WRITE_KEY_S            8U
#define ESCSS_PHY_IO_CONFIG_WRITE_KEY_M            0xFF00U   // Key to enable writing lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_SYNC_IO_CONFIG register
//
//*************************************************************************************************
#define ESCSS_SYNC_IO_CONFIG_SYNC0_IOPAD_SEL_S   0U
#define ESCSS_SYNC_IO_CONFIG_SYNC0_IOPAD_SEL_M   0x3U      // SYNC0 IO PAD select option
#define ESCSS_SYNC_IO_CONFIG_SYNC0_GPIO_EN       0x8U      // SYNC0 connection to OUT pad enabled
#define ESCSS_SYNC_IO_CONFIG_SYNC1_IOPAD_SEL_S   4U
#define ESCSS_SYNC_IO_CONFIG_SYNC1_IOPAD_SEL_M   0x30U     // SYNC1 IO PAD select option
#define ESCSS_SYNC_IO_CONFIG_SYNC1_GPIO_EN       0x80U     // SYNC1 connection to OUT pad enabled
#define ESCSS_SYNC_IO_CONFIG_WRITE_KEY_S         8U
#define ESCSS_SYNC_IO_CONFIG_WRITE_KEY_M         0xFF00U   // Key to enable writing lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_LATCH_IO_CONFIG register
//
//*************************************************************************************************
#define ESCSS_LATCH_IO_CONFIG_LATCH0_IOPAD_SEL_S   0U
#define ESCSS_LATCH_IO_CONFIG_LATCH0_IOPAD_SEL_M   0x3U      // LATCH0 IO PAD select option
#define ESCSS_LATCH_IO_CONFIG_LATCH0_GPIO_EN       0x8U      // LATCH0 connection to IN pad enabled
#define ESCSS_LATCH_IO_CONFIG_LATCH1_IOPAD_SEL_S   4U
#define ESCSS_LATCH_IO_CONFIG_LATCH1_IOPAD_SEL_M   0x30U     // LATCH1 IO PAD select option
#define ESCSS_LATCH_IO_CONFIG_LATCH1_GPIO_EN       0x80U     // LATCH1 connection to IN pad enabled
#define ESCSS_LATCH_IO_CONFIG_WRITE_KEY_S          8U
#define ESCSS_LATCH_IO_CONFIG_WRITE_KEY_M          0xFF00U   // Key to enable writing lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_LED_CONFIG register
//
//*************************************************************************************************
#define ESCSS_LED_CONFIG_LINKACT0               0x1U      // GPIO enable for LINKACT0 LED
#define ESCSS_LED_CONFIG_LINKACT1               0x2U      // GPIO enable for LINKACT1 LED
#define ESCSS_LED_CONFIG_STATE                  0x4U      // GPIO enable for STATE LED
#define ESCSS_LED_CONFIG_ERR                    0x8U      // GPIO enable for ERR LED
#define ESCSS_LED_CONFIG_RUN                    0x10U     // GPIO enable for RUN LED
#define ESCSS_LED_CONFIG_LINKACT0_IOPAD_SEL_S   6U
#define ESCSS_LED_CONFIG_LINKACT0_IOPAD_SEL_M   0xC0U     // LINKACT0 LED IO PAD select
#define ESCSS_LED_CONFIG_LINKACT1_IOPAD_SEL_S   8U
#define ESCSS_LED_CONFIG_LINKACT1_IOPAD_SEL_M   0x300U    // LINKACT1 LED IO PAD select
#define ESCSS_LED_CONFIG_STATE_IOPAD_SEL_S      10U
#define ESCSS_LED_CONFIG_STATE_IOPAD_SEL_M      0xC00U    // STATE LED IO PAD select
#define ESCSS_LED_CONFIG_ERR_IOPAD_SEL_S        12U
#define ESCSS_LED_CONFIG_ERR_IOPAD_SEL_M        0x3000U   // ERROR LED IO PAD select
#define ESCSS_LED_CONFIG_RUN_IOPAD_SEL_S        14U
#define ESCSS_LED_CONFIG_RUN_IOPAD_SEL_M        0xC000U   // RUN LED IO PAD select

//*************************************************************************************************
//
// The following are defines for the bit fields in the ESCSS_MISC_CONFIG register
//
//*************************************************************************************************
#define ESCSS_MISC_CONFIG_TX0_SHIFT_CONFIG_S   0U
#define ESCSS_MISC_CONFIG_TX0_SHIFT_CONFIG_M   0x3U     // TX Shift configuration for Port0
#define ESCSS_MISC_CONFIG_TX1_SHIFT_CONFIG_S   2U
#define ESCSS_MISC_CONFIG_TX1_SHIFT_CONFIG_M   0xCU     // TX Shift configuration for Port1
#define ESCSS_MISC_CONFIG_EEPROM_SIZE          0x10U    // EEPROM Size bound select
#define ESCSS_MISC_CONFIG_PDI_EMULATION        0x20U    // PDI Emulation enable
#define ESCSS_MISC_CONFIG_PHY_ADDR_S           6U
#define ESCSS_MISC_CONFIG_PHY_ADDR_M           0x7C0U   // PHY Address Offset



#endif
