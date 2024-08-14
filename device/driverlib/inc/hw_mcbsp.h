//###########################################################################
//
// FILE:    hw_mcbsp.h
//
// TITLE:   Definitions for the MCBSP registers.
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

#ifndef HW_MCBSP_H
#define HW_MCBSP_H

//*************************************************************************************************
//
// The following are defines for the MCBSP register offsets
//
//*************************************************************************************************
#define MCBSP_O_DRR2     0x0U    // Data receive register bits 31-16
#define MCBSP_O_DRR1     0x1U    // Data receive register bits 15-0
#define MCBSP_O_DXR2     0x2U    // Data transmit register bits 31-16
#define MCBSP_O_DXR1     0x3U    // Data transmit register bits 15-0
#define MCBSP_O_SPCR2    0x4U    // Serial port control register 2
#define MCBSP_O_SPCR1    0x5U    // Serial port control register 1
#define MCBSP_O_RCR2     0x6U    // Receive Control register 2
#define MCBSP_O_RCR1     0x7U    // Receive Control register 1
#define MCBSP_O_XCR2     0x8U    // Transmit Control register 2
#define MCBSP_O_XCR1     0x9U    // Transmit Control register 1
#define MCBSP_O_SRGR2    0xAU    // Sample rate generator register 2
#define MCBSP_O_SRGR1    0xBU    // Sample rate generator register 1
#define MCBSP_O_MCR2     0xCU    // Multi-channel control register 2
#define MCBSP_O_MCR1     0xDU    // Multi-channel control register 1
#define MCBSP_O_RCERA    0xEU    // Receive channel enable partition A
#define MCBSP_O_RCERB    0xFU    // Receive channel enable partition B
#define MCBSP_O_XCERA    0x10U   // Transmit channel enable partition A
#define MCBSP_O_XCERB    0x11U   // Transmit channel enable partition B
#define MCBSP_O_PCR      0x12U   // Pin Control register
#define MCBSP_O_RCERC    0x13U   // Receive channel enable partition C
#define MCBSP_O_RCERD    0x14U   // Receive channel enable partition D
#define MCBSP_O_XCERC    0x15U   // Transmit channel enable partition C
#define MCBSP_O_XCERD    0x16U   // Transmit channel enable partition D
#define MCBSP_O_RCERE    0x17U   // Receive channel enable partition E
#define MCBSP_O_RCERF    0x18U   // Receive channel enable partition F
#define MCBSP_O_XCERE    0x19U   // Transmit channel enable partition E
#define MCBSP_O_XCERF    0x1AU   // Transmit channel enable partition F
#define MCBSP_O_RCERG    0x1BU   // Receive channel enable partition G
#define MCBSP_O_RCERH    0x1CU   // Receive channel enable partition H
#define MCBSP_O_XCERG    0x1DU   // Transmit channel enable partition G
#define MCBSP_O_XCERH    0x1EU   // Transmit channel enable partition H
#define MCBSP_O_MFFINT   0x23U   // Interrupt enable


//*************************************************************************************************
//
// The following are defines for the bit fields in the DRR2 register
//
//*************************************************************************************************
#define MCBSP_DRR2_HWLB_S   0U
#define MCBSP_DRR2_HWLB_M   0xFFU     // High word low byte
#define MCBSP_DRR2_HWHB_S   8U
#define MCBSP_DRR2_HWHB_M   0xFF00U   // High word high byte

//*************************************************************************************************
//
// The following are defines for the bit fields in the DRR1 register
//
//*************************************************************************************************
#define MCBSP_DRR1_LWLB_S   0U
#define MCBSP_DRR1_LWLB_M   0xFFU     // Low word low byte
#define MCBSP_DRR1_LWHB_S   8U
#define MCBSP_DRR1_LWHB_M   0xFF00U   // Low word high byte

//*************************************************************************************************
//
// The following are defines for the bit fields in the DXR2 register
//
//*************************************************************************************************
#define MCBSP_DXR2_HWLB_S   0U
#define MCBSP_DXR2_HWLB_M   0xFFU     // High word low byte
#define MCBSP_DXR2_HWHB_S   8U
#define MCBSP_DXR2_HWHB_M   0xFF00U   // High word high byte

//*************************************************************************************************
//
// The following are defines for the bit fields in the DXR1 register
//
//*************************************************************************************************
#define MCBSP_DXR1_LWLB_S   0U
#define MCBSP_DXR1_LWLB_M   0xFFU     // Low word low byte
#define MCBSP_DXR1_LWHB_S   8U
#define MCBSP_DXR1_LWHB_M   0xFF00U   // Low word high byte

//*************************************************************************************************
//
// The following are defines for the bit fields in the SPCR2 register
//
//*************************************************************************************************
#define MCBSP_SPCR2_XRST       0x1U     // Transmitter reset
#define MCBSP_SPCR2_XRDY       0x2U     // Transmitter ready
#define MCBSP_SPCR2_XEMPTY     0x4U     // Transmitter empty
#define MCBSP_SPCR2_XSYNCERR   0x8U     // Transmit sync error INT flag
#define MCBSP_SPCR2_XINTM_S    4U
#define MCBSP_SPCR2_XINTM_M    0x30U    // Transmit Interupt mode bits
#define MCBSP_SPCR2_GRST       0x40U    // Sample rate generator reset
#define MCBSP_SPCR2_FRST       0x80U    // Frame sync logic reset
#define MCBSP_SPCR2_SOFT       0x100U   // SOFT bit
#define MCBSP_SPCR2_FREE       0x200U   // FREE bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SPCR1 register
//
//*************************************************************************************************
#define MCBSP_SPCR1_RRST       0x1U      // Receiver reset
#define MCBSP_SPCR1_RRDY       0x2U      // Receiver ready
#define MCBSP_SPCR1_RFULL      0x4U      // Receiver full
#define MCBSP_SPCR1_RSYNCERR   0x8U      // Receive sync error INT flag
#define MCBSP_SPCR1_RINTM_S    4U
#define MCBSP_SPCR1_RINTM_M    0x30U     // Receive Interupt mode bits
#define MCBSP_SPCR1_DXENA      0x80U     // DX delay enable
#define MCBSP_SPCR1_CLKSTP_S   11U
#define MCBSP_SPCR1_CLKSTP_M   0x1800U   // Clock stop mode
#define MCBSP_SPCR1_RJUST_S    13U
#define MCBSP_SPCR1_RJUST_M    0x6000U   // Rx sign extension and justification mode
#define MCBSP_SPCR1_DLB        0x8000U   // Digital loopback

//*************************************************************************************************
//
// The following are defines for the bit fields in the RCR2 register
//
//*************************************************************************************************
#define MCBSP_RCR2_RDATDLY_S    0U
#define MCBSP_RCR2_RDATDLY_M    0x3U      // Receive data delay
#define MCBSP_RCR2_RFIG         0x4U      // Receive frame sync ignore
#define MCBSP_RCR2_RCOMPAND_S   3U
#define MCBSP_RCR2_RCOMPAND_M   0x18U     // Receive Companding Mode selects
#define MCBSP_RCR2_RWDLEN2_S    5U
#define MCBSP_RCR2_RWDLEN2_M    0xE0U     // Receive word length 2
#define MCBSP_RCR2_RFRLEN2_S    8U
#define MCBSP_RCR2_RFRLEN2_M    0x7F00U   // Receive Frame length 2
#define MCBSP_RCR2_RPHASE       0x8000U   // Receive Phase

//*************************************************************************************************
//
// The following are defines for the bit fields in the RCR1 register
//
//*************************************************************************************************
#define MCBSP_RCR1_RWDLEN1_S   5U
#define MCBSP_RCR1_RWDLEN1_M   0xE0U     // Receive word length 1
#define MCBSP_RCR1_RFRLEN1_S   8U
#define MCBSP_RCR1_RFRLEN1_M   0x7F00U   // Receive Frame length 1

//*************************************************************************************************
//
// The following are defines for the bit fields in the XCR2 register
//
//*************************************************************************************************
#define MCBSP_XCR2_XDATDLY_S    0U
#define MCBSP_XCR2_XDATDLY_M    0x3U      // Transmit data delay
#define MCBSP_XCR2_XFIG         0x4U      // Transmit frame sync ignore
#define MCBSP_XCR2_XCOMPAND_S   3U
#define MCBSP_XCR2_XCOMPAND_M   0x18U     // Transmit Companding Mode selects
#define MCBSP_XCR2_XWDLEN2_S    5U
#define MCBSP_XCR2_XWDLEN2_M    0xE0U     // Transmit word length 2
#define MCBSP_XCR2_XFRLEN2_S    8U
#define MCBSP_XCR2_XFRLEN2_M    0x7F00U   // Transmit Frame length 2
#define MCBSP_XCR2_XPHASE       0x8000U   // Transmit Phase

//*************************************************************************************************
//
// The following are defines for the bit fields in the XCR1 register
//
//*************************************************************************************************
#define MCBSP_XCR1_XWDLEN1_S   5U
#define MCBSP_XCR1_XWDLEN1_M   0xE0U     // Transmit word length 1
#define MCBSP_XCR1_XFRLEN1_S   8U
#define MCBSP_XCR1_XFRLEN1_M   0x7F00U   // Transmit Frame length 1

//*************************************************************************************************
//
// The following are defines for the bit fields in the SRGR2 register
//
//*************************************************************************************************
#define MCBSP_SRGR2_FPER_S   0U
#define MCBSP_SRGR2_FPER_M   0xFFFU    // Frame-sync period
#define MCBSP_SRGR2_FSGM     0x1000U   // Frame sync generator mode
#define MCBSP_SRGR2_CLKSM    0x2000U   // Sample rate generator mode
#define MCBSP_SRGR2_GSYNC    0x8000U   // CLKG sync

//*************************************************************************************************
//
// The following are defines for the bit fields in the SRGR1 register
//
//*************************************************************************************************
#define MCBSP_SRGR1_CLKGDV_S   0U
#define MCBSP_SRGR1_CLKGDV_M   0xFFU     // CLKG divider
#define MCBSP_SRGR1_FWID_S     8U
#define MCBSP_SRGR1_FWID_M     0xFF00U   // Frame width

//*************************************************************************************************
//
// The following are defines for the bit fields in the MCR2 register
//
//*************************************************************************************************
#define MCBSP_MCR2_XMCM_S     0U
#define MCBSP_MCR2_XMCM_M     0x3U     // Transmit data delay
#define MCBSP_MCR2_XCBLK_S    2U
#define MCBSP_MCR2_XCBLK_M    0x1CU    // Transmit frame sync ignore
#define MCBSP_MCR2_XPABLK_S   5U
#define MCBSP_MCR2_XPABLK_M   0x60U    // Transmit Companding Mode selects
#define MCBSP_MCR2_XPBBLK_S   7U
#define MCBSP_MCR2_XPBBLK_M   0x180U   // Transmit word length 2
#define MCBSP_MCR2_XMCME      0x200U   // Transmit Frame length 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the MCR1 register
//
//*************************************************************************************************
#define MCBSP_MCR1_RMCM       0x1U     // Receive multichannel mode
#define MCBSP_MCR1_RCBLK_S    2U
#define MCBSP_MCR1_RCBLK_M    0x1CU    // eceive current block
#define MCBSP_MCR1_RPABLK_S   5U
#define MCBSP_MCR1_RPABLK_M   0x60U    // Receive partition A Block
#define MCBSP_MCR1_RPBBLK_S   7U
#define MCBSP_MCR1_RPBBLK_M   0x180U   // Receive partition B Block
#define MCBSP_MCR1_RMCME      0x200U   // Receive multi-channel enhance mode

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCR register
//
//*************************************************************************************************
#define MCBSP_PCR_CLKRP    0x1U     // Receive Clock polarity
#define MCBSP_PCR_CLKXP    0x2U     // Transmit clock polarity
#define MCBSP_PCR_FSRP     0x4U     // Receive Frame synchronization polarity
#define MCBSP_PCR_FSXP     0x8U     // Transmit Frame synchronization polarity
#define MCBSP_PCR_SCLKME   0x80U    // Sample clock mode selection
#define MCBSP_PCR_CLKRM    0x100U   // Receiver Clock Mode
#define MCBSP_PCR_CLKXM    0x200U   // Transmit Clock Mode.
#define MCBSP_PCR_FSRM     0x400U   // Receive Frame Synchronization Mode
#define MCBSP_PCR_FSXM     0x800U   // Transmit Frame Synchronization Mode

//*************************************************************************************************
//
// The following are defines for the bit fields in the MFFINT register
//
//*************************************************************************************************
#define MCBSP_MFFINT_XINT   0x1U   // Enable for Receive Interrupt
#define MCBSP_MFFINT_RINT   0x4U   // Enable for transmit Interrupt



#endif
