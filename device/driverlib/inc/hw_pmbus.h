//###########################################################################
//
// FILE:    hw_pmbus.h
//
// TITLE:   Definitions for the PMBUS registers.
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

#ifndef HW_PMBUS_H
#define HW_PMBUS_H

//*************************************************************************************************
//
// The following are defines for the PMBUS register offsets
//
//*************************************************************************************************
#define PMBUS_O_PMBMC              0x0U    // PMBUS Master Mode Control Register
#define PMBUS_O_PMBTXBUF           0x2U    // PMBUS Transmit Buffer
#define PMBUS_O_PMBRXBUF           0x4U    // PMBUS Receive buffer
#define PMBUS_O_PMBACK             0x6U    // PMBUS Acknowledge Register
#define PMBUS_O_PMBSTS             0x8U    // PMBUS Status Register
#define PMBUS_O_PMBINTM            0xAU    // PMBUS Interrupt Mask Register
#define PMBUS_O_PMBSC              0xCU    // PMBUS Slave Mode Configuration Register
#define PMBUS_O_PMBHSA             0xEU    // PMBUS Hold Slave Address Register
#define PMBUS_O_PMBCTRL            0x10U   // PMBUS Control Register
#define PMBUS_O_PMBTIMCTL          0x12U   // PMBUS Timing Control Register
#define PMBUS_O_PMBTIMCLK          0x14U   // PMBUS Clock Timing Register
#define PMBUS_O_PMBTIMSTSETUP      0x16U   // PMBUS Start Setup Time Register
#define PMBUS_O_PMBTIMBIDLE        0x18U   // PMBUS Bus Idle Time Register
#define PMBUS_O_PMBTIMLOWTIMOUT    0x1AU   // PMBUS Clock Low Timeout Value Register
#define PMBUS_O_PMBTIMHIGHTIMOUT   0x1CU   // PMBUS Clock High Timeout Value Register


//*************************************************************************************************
//
// The following are defines for the bit fields in the PMBMC register
//
//*************************************************************************************************
#define PMBUS_PMBMC_RW             0x1U        // RnW bit of the Message
#define PMBUS_PMBMC_SLAVE_ADDR_S   1U
#define PMBUS_PMBMC_SLAVE_ADDR_M   0xFEU       // Slave Address
#define PMBUS_PMBMC_BYTE_COUNT_S   8U
#define PMBUS_PMBMC_BYTE_COUNT_M   0xFF00U     // Number of Bytes Transmitted
#define PMBUS_PMBMC_CMD_ENA        0x10000U    // Master Command Code Enable
#define PMBUS_PMBMC_EXT_CMD        0x20000U    // Master Extended Command Code Enable
#define PMBUS_PMBMC_PEC_ENA        0x40000U    // Master PEC Processing Enable
#define PMBUS_PMBMC_GRP_CMD        0x80000U    // Master Group Command Message Enable
#define PMBUS_PMBMC_PRC_CALL       0x100000U   // Master Process Call Message Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the PMBACK register
//
//*************************************************************************************************
#define PMBUS_PMBACK_ACK   0x1U   // Allows firmware to ack/nack received data

//*************************************************************************************************
//
// The following are defines for the bit fields in the PMBSTS register
//
//*************************************************************************************************
#define PMBUS_PMBSTS_RD_BYTE_COUNT_S     0U
#define PMBUS_PMBSTS_RD_BYTE_COUNT_M     0x7U        // Number of Data Bytes available in Receive
                                                     // Data Register
#define PMBUS_PMBSTS_DATA_READY          0x8U        // Data Ready Flag
#define PMBUS_PMBSTS_DATA_REQUEST        0x10U       // Data Request Flag
#define PMBUS_PMBSTS_EOM                 0x20U       // End of Message Indicator
#define PMBUS_PMBSTS_NACK                0x40U       // Not Acknowledge Flag Status
#define PMBUS_PMBSTS_PEC_VALID           0x80U       // PEC Valid Indicator
#define PMBUS_PMBSTS_CLK_LOW_TIMEOUT     0x100U      // Clock Low Timeout Status
#define PMBUS_PMBSTS_CLK_HIGH_DETECTED   0x200U      // Clock High Detection Status
#define PMBUS_PMBSTS_SLAVE_ADDR_READY    0x400U      // Slave Address Ready
#define PMBUS_PMBSTS_RPT_START           0x800U      // Repeated Start Flag
#define PMBUS_PMBSTS_UNIT_BUSY           0x1000U     // PMBus Busy Indicator
#define PMBUS_PMBSTS_BUS_FREE            0x2000U     // PMBus Free Indicator
#define PMBUS_PMBSTS_LOST_ARB            0x4000U     // Lost Arbitration Flag
#define PMBUS_PMBSTS_MASTER              0x8000U     // Master Indicator
#define PMBUS_PMBSTS_ALERT_EDGE          0x10000U    // Alert Edge Detection Status
#define PMBUS_PMBSTS_CONTROL_EDGE        0x20000U    // Control Edge Detection Status
#define PMBUS_PMBSTS_ALERT_RAW           0x40000U    // Alert Pin Real Time Status
#define PMBUS_PMBSTS_CONTROL_RAW         0x80000U    // Control Pin Real Time Status
#define PMBUS_PMBSTS_SDA_RAW             0x100000U   // PMBus Data Pin Real Time Status
#define PMBUS_PMBSTS_SCL_RAW             0x200000U   // PMBus Clock Pin Real Time Status

//*************************************************************************************************
//
// The following are defines for the bit fields in the PMBINTM register
//
//*************************************************************************************************
#define PMBUS_PMBINTM_BUS_FREE           0x1U     // Bus Free Interrupt Mask
#define PMBUS_PMBINTM_BUS_LOW_TIMEOUT    0x2U     // Clock Low Timeout Interrupt Mask
#define PMBUS_PMBINTM_DATA_READY         0x4U     // Data Ready Interrupt Mask
#define PMBUS_PMBINTM_DATA_REQUEST       0x8U     // Data Request Interrupt Mask
#define PMBUS_PMBINTM_SLAVE_ADDR_READY   0x10U    // Slave Address Ready Interrupt Mask
#define PMBUS_PMBINTM_EOM                0x20U    // End of Message Interrupt Mask
#define PMBUS_PMBINTM_ALERT              0x40U    // Alert Detection Interrupt Mask
#define PMBUS_PMBINTM_CONTROL            0x80U    // Control Detection Interrupt Mask
#define PMBUS_PMBINTM_LOST_ARB           0x100U   // Lost Arbitration Interrupt Mask
#define PMBUS_PMBINTM_CLK_HIGH_DETECT    0x200U   // Clock High Detection Interrupt Mask

//*************************************************************************************************
//
// The following are defines for the bit fields in the PMBSC register
//
//*************************************************************************************************
#define PMBUS_PMBSC_SLAVE_ADDR_S        0U
#define PMBUS_PMBSC_SLAVE_ADDR_M        0x7FU       // Configures the current device address of the
                                                    // slave.
#define PMBUS_PMBSC_MAN_SLAVE_ACK       0x80U       // Manual Slave Address Acknowledgement Mode
#define PMBUS_PMBSC_SLAVE_MASK_S        8U
#define PMBUS_PMBSC_SLAVE_MASK_M        0x7F00U     // Slave address mask
#define PMBUS_PMBSC_PEC_ENA             0x8000U     // PEC Processing Enable
#define PMBUS_PMBSC_TX_COUNT_S          16U
#define PMBUS_PMBSC_TX_COUNT_M          0x70000U    // Number of valid bytes in Transmit Data
                                                    // Register
#define PMBUS_PMBSC_TX_PEC              0x80000U    // send a PEC byte at end of message
#define PMBUS_PMBSC_MAN_CMD             0x100000U   // Manual Command Acknowledgement Mode
#define PMBUS_PMBSC_RX_BYTE_ACK_CNT_S   21U
#define PMBUS_PMBSC_RX_BYTE_ACK_CNT_M   0x600000U   // Number of data bytes to automatically
                                                    // acknowledge

//*************************************************************************************************
//
// The following are defines for the bit fields in the PMBHSA register
//
//*************************************************************************************************
#define PMBUS_PMBHSA_SLAVE_RW       0x1U    // Stored R/W bit
#define PMBUS_PMBHSA_SLAVE_ADDR_S   1U
#define PMBUS_PMBHSA_SLAVE_ADDR_M   0xFEU   // Stored device address

//*************************************************************************************************
//
// The following are defines for the bit fields in the PMBCTRL register
//
//*************************************************************************************************
#define PMBUS_PMBCTRL_RESET             0x1U          // PMBus Interface Synchronous Reset
#define PMBUS_PMBCTRL_ALERT_EN          0x2U          // Slave Alert Enable
#define PMBUS_PMBCTRL_BUS_LO_INT_EDGE   0x4U          // Clock Low Timeout Interrupt Edge Select
#define PMBUS_PMBCTRL_FAST_MODE         0x8U          // Fast Mode Enable
#define PMBUS_PMBCTRL_CNTL_INT_EDGE     0x20U         // Control Interrupt Edge Select
#define PMBUS_PMBCTRL_ALERT_MODE        0x40U         // Configures mode of Alert pin
#define PMBUS_PMBCTRL_ALERT_VALUE       0x80U         // Configures output value of Alert pin in
                                                      // GPIO Mode
#define PMBUS_PMBCTRL_ALERT_DIR         0x100U        // Configures direction of Alert pin in GPIO
                                                      // mode
#define PMBUS_PMBCTRL_CNTL_MODE         0x200U        // Configures mode of Control pin
#define PMBUS_PMBCTRL_CNTL_VALUE        0x400U        // Configures output value of Control pin in
                                                      // GPIO Mode
#define PMBUS_PMBCTRL_CNTL_DIR          0x800U        // Configures direction of Control pin in
                                                      // GPIO mode
#define PMBUS_PMBCTRL_SDA_MODE          0x1000U       // Configures mode of PMBus Data pin
#define PMBUS_PMBCTRL_SDA_VALUE         0x2000U       // Configures output value of PMBus data pin
                                                      // in GPIO Mode
#define PMBUS_PMBCTRL_SDA_DIR           0x4000U       // Configures direction of PMBus data pin in
                                                      // GPIO mode
#define PMBUS_PMBCTRL_SCL_MODE          0x8000U       // Configures mode of PMBus Clock pin
#define PMBUS_PMBCTRL_SCL_VALUE         0x10000U      // Configures output value of PMBus clock pin
                                                      // in GPIO Mode
#define PMBUS_PMBCTRL_SCL_DIR           0x20000U      // Configures direction of PMBus clock pin in
                                                      // GPIO mode
#define PMBUS_PMBCTRL_IBIAS_A_EN        0x40000U      // PMBus Current Source A Control
#define PMBUS_PMBCTRL_IBIAS_B_EN        0x80000U      // PMBus Current Source B Control
#define PMBUS_PMBCTRL_CLK_LO_DIS        0x100000U     // Clock Low Timeout Disable
#define PMBUS_PMBCTRL_SLAVE_EN          0x200000U     // PMBus Slave Enable
#define PMBUS_PMBCTRL_MASTER_EN         0x400000U     // PMBus Master Enable
#define PMBUS_PMBCTRL_CLKDIV_S          23U
#define PMBUS_PMBCTRL_CLKDIV_M          0xF800000U    // PMBUS Clock Divide Value
#define PMBUS_PMBCTRL_I2CMODE           0x80000000U   // Bit to enable I2C mode

//*************************************************************************************************
//
// The following are defines for the bit fields in the PMBTIMCTL register
//
//*************************************************************************************************
#define PMBUS_PMBTIMCTL_TIM_OVERRIDE   0x1U   // Overide the default settings of the timing
                                              // parameters.

//*************************************************************************************************
//
// The following are defines for the bit fields in the PMBTIMCLK register
//
//*************************************************************************************************
#define PMBUS_PMBTIMCLK_CLK_HIGH_LIMIT_S   0U
#define PMBUS_PMBTIMCLK_CLK_HIGH_LIMIT_M   0xFFU       // Determines the PMBUS master clock high
                                                       // pulse width.
#define PMBUS_PMBTIMCLK_CLK_FREQ_S         16U
#define PMBUS_PMBTIMCLK_CLK_FREQ_M         0xFF0000U   // Determines the PMBUS master clock
                                                       // frequency.

//*************************************************************************************************
//
// The following are defines for the bit fields in the PMBTIMSTSETUP register
//
//*************************************************************************************************
#define PMBUS_PMBTIMSTSETUP_TSU_STA_S   0U
#define PMBUS_PMBTIMSTSETUP_TSU_STA_M   0xFFU   // Setup time, rise edge of PMBUS master clock to
                                                // start edge.

//*************************************************************************************************
//
// The following are defines for the bit fields in the PMBTIMBIDLE register
//
//*************************************************************************************************
#define PMBUS_PMBTIMBIDLE_BUSIDLE_S   0U
#define PMBUS_PMBTIMBIDLE_BUSIDLE_M   0x3FFU   // Determines the Bus Idle Limit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PMBTIMLOWTIMOUT register
//
//*************************************************************************************************
#define PMBUS_PMBTIMLOWTIMOUT_CLKLOWTIMOUT_S   0U
#define PMBUS_PMBTIMLOWTIMOUT_CLKLOWTIMOUT_M   0xFFFFFU   // Determines the clock low timeout value

//*************************************************************************************************
//
// The following are defines for the bit fields in the PMBTIMHIGHTIMOUT register
//
//*************************************************************************************************
#define PMBUS_PMBTIMHIGHTIMOUT_CLKHIGHTIMOUT_S   0U
#define PMBUS_PMBTIMHIGHTIMOUT_CLKHIGHTIMOUT_M   0x3FFU   // Determines the clock high timeout
                                                          // value



#endif
