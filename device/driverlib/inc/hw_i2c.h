//###########################################################################
//
// FILE:    hw_i2c.h
//
// TITLE:   Definitions for the I2C registers.
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

#ifndef HW_I2C_H
#define HW_I2C_H

//*************************************************************************************************
//
// The following are defines for the I2C register offsets
//
//*************************************************************************************************
#define I2C_O_OAR    0x0U    // I2C Own address
#define I2C_O_IER    0x1U    // I2C Interrupt Enable
#define I2C_O_STR    0x2U    // I2C Status
#define I2C_O_CLKL   0x3U    // I2C Clock low-time divider
#define I2C_O_CLKH   0x4U    // I2C Clock high-time divider
#define I2C_O_CNT    0x5U    // I2C Data count
#define I2C_O_DRR    0x6U    // I2C Data receive
#define I2C_O_SAR    0x7U    // I2C Slave address
#define I2C_O_DXR    0x8U    // I2C Data Transmit
#define I2C_O_MDR    0x9U    // I2C Mode
#define I2C_O_ISRC   0xAU    // I2C Interrupt Source
#define I2C_O_EMDR   0xBU    // I2C Extended Mode
#define I2C_O_PSC    0xCU    // I2C Prescaler
#define I2C_O_FFTX   0x20U   // I2C FIFO Transmit
#define I2C_O_FFRX   0x21U   // I2C FIFO Receive


//*************************************************************************************************
//
// The following are defines for the bit fields in the I2COAR register
//
//*************************************************************************************************
#define I2C_OAR_OAR_S   0U
#define I2C_OAR_OAR_M   0x3FFU   // I2C Own address

//*************************************************************************************************
//
// The following are defines for the bit fields in the I2CIER register
//
//*************************************************************************************************
#define I2C_IER_ARBL   0x1U    // Arbitration-lost interrupt enable
#define I2C_IER_NACK   0x2U    // No-acknowledgment interrupt enable
#define I2C_IER_ARDY   0x4U    // Register-access-ready interrupt enable
#define I2C_IER_RRDY   0x8U    // Receive-data-ready interrupt enable
#define I2C_IER_XRDY   0x10U   // Transmit-data-ready interrupt enable
#define I2C_IER_SCD    0x20U   // Stop condition detected interrupt enable
#define I2C_IER_AAS    0x40U   // Addressed as slave interrupt enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the I2CSTR register
//
//*************************************************************************************************
#define I2C_STR_ARBL       0x1U      // Arbitration-lost interrupt flag bit
#define I2C_STR_NACK       0x2U      // No-acknowledgment interrupt flag bit.
#define I2C_STR_ARDY       0x4U      // Register-access-ready interrupt flag bit
#define I2C_STR_RRDY       0x8U      // Receive-data-ready interrupt flag bit.
#define I2C_STR_XRDY       0x10U     // Transmit-data-ready interrupt flag bit.
#define I2C_STR_SCD        0x20U     // Stop condition detected bit.
#define I2C_STR_BYTESENT   0x40U     // Byte transmit over indication
#define I2C_STR_AD0        0x100U    // Address 0 bits
#define I2C_STR_AAS        0x200U    // Addressed-as-slave bit
#define I2C_STR_XSMT       0x400U    // Transmit shift register empty bit.
#define I2C_STR_RSFULL     0x800U    // Receive shift register full bit.
#define I2C_STR_BB         0x1000U   // Bus busy bit.
#define I2C_STR_NACKSNT    0x2000U   // NACK sent bit.
#define I2C_STR_SDIR       0x4000U   // Slave direction bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the I2CDRR register
//
//*************************************************************************************************
#define I2C_DRR_DATA_S   0U
#define I2C_DRR_DATA_M   0xFFU   // Receive data

//*************************************************************************************************
//
// The following are defines for the bit fields in the I2CSAR register
//
//*************************************************************************************************
#define I2C_SAR_SAR_S   0U
#define I2C_SAR_SAR_M   0x3FFU   // Slave Address

//*************************************************************************************************
//
// The following are defines for the bit fields in the I2CDXR register
//
//*************************************************************************************************
#define I2C_DXR_DATA_S   0U
#define I2C_DXR_DATA_M   0xFFU   // Transmit data

//*************************************************************************************************
//
// The following are defines for the bit fields in the I2CMDR register
//
//*************************************************************************************************
#define I2C_MDR_BC_S      0U
#define I2C_MDR_BC_M      0x7U      // Bit count bits.
#define I2C_MDR_FDF       0x8U      // Free Data Format
#define I2C_MDR_STB       0x10U     // START Byte Mode
#define I2C_MDR_IRS       0x20U     // I2C Module Reset
#define I2C_MDR_DLB       0x40U     // Digital Loopback Mode
#define I2C_MDR_RM        0x80U     // Repeat Mode
#define I2C_MDR_XA        0x100U    // Expanded Address Mode
#define I2C_MDR_TRX       0x200U    // Transmitter Mode
#define I2C_MDR_MST       0x400U    // Master Mode
#define I2C_MDR_STP       0x800U    // STOP Condition
#define I2C_MDR_STT       0x2000U   // START condition bit
#define I2C_MDR_FREE      0x4000U   // Debug Action
#define I2C_MDR_NACKMOD   0x8000U   // NACK mode bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the I2CISRC register
//
//*************************************************************************************************
#define I2C_ISRC_INTCODE_S       0U
#define I2C_ISRC_INTCODE_M       0x7U     // Interrupt code bits.
#define I2C_ISRC_WRITE_ZEROS_S   8U
#define I2C_ISRC_WRITE_ZEROS_M   0xF00U   // Always write all 0s to this field

//*************************************************************************************************
//
// The following are defines for the bit fields in the I2CEMDR register
//
//*************************************************************************************************
#define I2C_EMDR_BC    0x1U   // Backwards compatibility mode
#define I2C_EMDR_FCM   0x2U   // Forward Compatibility for Tx behav in Type1

//*************************************************************************************************
//
// The following are defines for the bit fields in the I2CPSC register
//
//*************************************************************************************************
#define I2C_PSC_IPSC_S   0U
#define I2C_PSC_IPSC_M   0xFFU   // I2C Prescaler Divide Down

//*************************************************************************************************
//
// The following are defines for the bit fields in the I2CFFTX register
//
//*************************************************************************************************
#define I2C_FFTX_TXFFIL_S     0U
#define I2C_FFTX_TXFFIL_M     0x1FU     // Transmit FIFO Interrupt Level
#define I2C_FFTX_TXFFIENA     0x20U     // Transmit FIFO Interrupt Enable
#define I2C_FFTX_TXFFINTCLR   0x40U     // Transmit FIFO Interrupt Flag Clear
#define I2C_FFTX_TXFFINT      0x80U     // Transmit FIFO Interrupt Flag
#define I2C_FFTX_TXFFST_S     8U
#define I2C_FFTX_TXFFST_M     0x1F00U   // Transmit FIFO Status
#define I2C_FFTX_TXFFRST      0x2000U   // Transmit FIFO Reset
#define I2C_FFTX_I2CFFEN      0x4000U   // Transmit FIFO Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the I2CFFRX register
//
//*************************************************************************************************
#define I2C_FFRX_RXFFIL_S     0U
#define I2C_FFRX_RXFFIL_M     0x1FU     // Receive FIFO Interrupt Level
#define I2C_FFRX_RXFFIENA     0x20U     // Receive FIFO Interrupt Enable
#define I2C_FFRX_RXFFINTCLR   0x40U     // Receive FIFO Interrupt Flag Clear
#define I2C_FFRX_RXFFINT      0x80U     // Receive FIFO Interrupt Flag
#define I2C_FFRX_RXFFST_S     8U
#define I2C_FFRX_RXFFST_M     0x1F00U   // Receive FIFO Status
#define I2C_FFRX_RXFFRST      0x2000U   // Receive FIFO Reset



#endif
