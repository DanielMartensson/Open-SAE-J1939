//###########################################################################
//
// FILE:    hw_hic.h
//
// TITLE:   Definitions for the HIC registers.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
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

#ifndef HW_HIC_H
#define HW_HIC_H

//*************************************************************************************************
//
// The following are defines for the HIC register offsets
//
//*************************************************************************************************
#define HIC_O_REV          0x0U    // Module Revision Register
#define HIC_O_GCR          0x2U    // Global Control Register
#define HIC_O_LOCK         0x4U    // Lock Register
#define HIC_O_MODECR       0x6U    // Mode Control Register
#define HIC_O_PINPOLCR     0x8U    // Pin Polarity Control Register
#define HIC_O_BASESEL      0xAU    // Base Select Register
#define HIC_O_HOSTCR       0xCU    // Host Control Register
#define HIC_O_ERRADDR      0xEU    // Host Error Address register
#define HIC_O_H2DTOKEN     0x10U   // Host to Device Token Register
#define HIC_O_D2HTOKEN     0x12U   // Devie to Host Token Register
#define HIC_O_DBADDR0      0x14U   // Device Base Address Register 0
#define HIC_O_DBADDR1      0x16U   // Device Base Address Register 1
#define HIC_O_DBADDR2      0x18U   // Device Base Address Register 2
#define HIC_O_DBADDR3      0x1AU   // Device Base Address Register 3
#define HIC_O_DBADDR4      0x1CU   // Device Base Address Register 4
#define HIC_O_DBADDR5      0x1EU   // Device Base Address Register 5
#define HIC_O_DBADDR6      0x20U   // Device Base Address Register 6
#define HIC_O_DBADDR7      0x22U   // Device Base Address Register 7
#define HIC_O_H2DINTEN     0x28U   // H2D Interrupt Enable
#define HIC_O_H2DINTFLG    0x2AU   // H2D Interrupt status Flag
#define HIC_O_H2DINTCLR    0x2CU   // H2D Interrupt status Clear
#define HIC_O_H2DINTFRC    0x2EU   // H2D Interrupt Set Force
#define HIC_O_D2HINTEN     0x30U   // D2H Interrupt Enable
#define HIC_O_D2HINTFLG    0x32U   // D2H Interrupt status Flag
#define HIC_O_D2HINTCLR    0x34U   // D2H Interrupt status Clear
#define HIC_O_D2HINTFRC    0x36U   // D2H Interrupt Set Force
#define HIC_O_ACCVIOADDR   0x38U   // Access Violation Address
#define HIC_O_H2D_BUF0     0x40U   // Host to Device Buffer 0
#define HIC_O_H2D_BUF1     0x42U   // Host to Device Buffer 1
#define HIC_O_H2D_BUF2     0x44U   // Host to Device Buffer 2
#define HIC_O_H2D_BUF3     0x46U   // Host to Device Buffer 3
#define HIC_O_H2D_BUF4     0x48U   // Host to Device Buffer 4
#define HIC_O_H2D_BUF5     0x4AU   // Host to Device Buffer 5
#define HIC_O_H2D_BUF6     0x4CU   // Host to Device Buffer 6
#define HIC_O_H2D_BUF7     0x4EU   // Host to Device Buffer 7
#define HIC_O_H2D_BUF8     0x50U   // Host to Device Buffer 8
#define HIC_O_H2D_BUF9     0x52U   // Host to Device Buffer 9
#define HIC_O_H2D_BUF10    0x54U   // Host to Device Buffer 10
#define HIC_O_H2D_BUF11    0x56U   // Host to Device Buffer 11
#define HIC_O_H2D_BUF12    0x58U   // Host to Device Buffer 12
#define HIC_O_H2D_BUF13    0x5AU   // Host to Device Buffer 13
#define HIC_O_H2D_BUF14    0x5CU   // Host to Device Buffer 14
#define HIC_O_H2D_BUF15    0x5EU   // Host to Device Buffer 15
#define HIC_O_D2H_BUF0     0x60U   // Device to Host Buffer 0
#define HIC_O_D2H_BUF1     0x62U   // Device to Host Buffer 1
#define HIC_O_D2H_BUF2     0x64U   // Device to Host Buffer 2
#define HIC_O_D2H_BUF3     0x66U   // Device to Host Buffer 3
#define HIC_O_D2H_BUF4     0x68U   // Device to Host Buffer 4
#define HIC_O_D2H_BUF5     0x6AU   // Device to Host Buffer 5
#define HIC_O_D2H_BUF6     0x6CU   // Device to Host Buffer 6
#define HIC_O_D2H_BUF7     0x6EU   // Device to Host Buffer 7
#define HIC_O_D2H_BUF8     0x70U   // Device to Host Buffer 8
#define HIC_O_D2H_BUF9     0x72U   // Device to Host Buffer 9
#define HIC_O_D2H_BUF10    0x74U   // Device to Host Buffer 10
#define HIC_O_D2H_BUF11    0x76U   // Device to Host Buffer 11
#define HIC_O_D2H_BUF12    0x78U   // Device to Host Buffer 12
#define HIC_O_D2H_BUF13    0x7AU   // Device to Host Buffer 13
#define HIC_O_D2H_BUF14    0x7CU   // Device to Host Buffer 14
#define HIC_O_D2H_BUF15    0x7EU   // Device to Host Buffer 15


//*************************************************************************************************
//
// The following are defines for the bit fields in the HICREV register
//
//*************************************************************************************************
#define HIC_REV_MINOR_S    0U
#define HIC_REV_MINOR_M    0x3FU         // Minor Revision Number
#define HIC_REV_CUSTOM_S   6U
#define HIC_REV_CUSTOM_M   0xC0U         // Custom Module Number
#define HIC_REV_MAJOR_S    8U
#define HIC_REV_MAJOR_M    0x700U        // Major Revision Number
#define HIC_REV_RTL_S      11U
#define HIC_REV_RTL_M      0xF800U       // Design Release Number
#define HIC_REV_FUNC_S     16U
#define HIC_REV_FUNC_M     0xFFF0000U    // Functional Release Number
#define HIC_REV_SCHEME_S   30U
#define HIC_REV_SCHEME_M   0xC0000000U   // Defines Scheme for Module
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICGCR register
//
//*************************************************************************************************
#define HIC_GCR_HICEN_S   0U
#define HIC_GCR_HICEN_M   0xFU   // Host Interface Enable
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICLOCK register
//
//*************************************************************************************************
#define HIC_LOCK_LOCK                 0x1U          // LOCK enable
#define HIC_LOCK_WRITE_ENABLE_KEY_S   16U
#define HIC_LOCK_WRITE_ENABLE_KEY_M   0xFFFF0000U   // Key for enabling write
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICMODECR register
//
//*************************************************************************************************
#define HIC_MODECR_DW_MODE_S         0U
#define HIC_MODECR_DW_MODE_M         0x3U     // Data Width Mode
#define HIC_MODECR_RW_MODE           0x10U    // Read-Write Mode
#define HIC_MODECR_BEN_PRESENT       0x20U    // Byte Enable Pins are present
#define HIC_MODECR_RDY_PRESENT       0x40U    // Ready pin present
#define HIC_MODECR_H2DBUF_DEVWREN    0x100U   // Write Enable for Device to H2D Buffer
#define HIC_MODECR_D2HBUF_HOSTWREN   0x200U   // Write Enable for Host to D2H Buffer
#define HIC_MODECR_EN_DEVACC         0x400U   // Enable Host access to Device region
#define HIC_MODECR_EN_HOSTWREALLOW   0x800U   // Enable Host Write to EALLOWCTL register
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICPINPOLCR register
//
//*************************************************************************************************
#define HIC_PINPOLCR_CS_POL    0x1U    // Chip Select Polarity
#define HIC_PINPOLCR_BEN_POL   0x2U    // Byte Enable Polarity
#define HIC_PINPOLCR_OE_POL    0x4U    // Output Enable Polarity
#define HIC_PINPOLCR_WE_POL    0x8U    // Write Enable Polarity
#define HIC_PINPOLCR_RDY_POL   0x10U   // Ready Polarity
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICBASESEL register
//
//*************************************************************************************************
#define HIC_BASESEL_BASE_SELECT_S   0U
#define HIC_BASESEL_BASE_SELECT_M   0x7U   // Base Select
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICHOSTCR register
//
//*************************************************************************************************
#define HIC_HOSTCR_EALLOW_EN   0x1U      // EALLOW Enable
#define HIC_HOSTCR_ACCSIZE     0x2U      // Access Size
#define HIC_HOSTCR_PAGESEL     0x4U      // Page Select
#define HIC_HOSTCR_HKEY_S      8U
#define HIC_HOSTCR_HKEY_M      0xFF00U   // Host Key
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICERRADDR register
//
//*************************************************************************************************
#define HIC_ERRADDR_H2D_ERR_ADDR_S   0U
#define HIC_ERRADDR_H2D_ERR_ADDR_M   0xFFU         // Address of the Host bus captured upon an
                                                   // error for Device
#define HIC_ERRADDR_H2D_BASE_SEL_S   12U
#define HIC_ERRADDR_H2D_BASE_SEL_M   0x7000U       // Base Select corresponding to H2D error event
#define HIC_ERRADDR_D2H_ERR_ADDR_S   16U
#define HIC_ERRADDR_D2H_ERR_ADDR_M   0xFF0000U     // Address of the Host bus captured upon an
                                                   // error for Host
#define HIC_ERRADDR_D2H_BASE_SEL_S   28U
#define HIC_ERRADDR_D2H_BASE_SEL_M   0x70000000U   // Base Select corresponding to D2H error event
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICDBADDR0 register
//
//*************************************************************************************************
#define HIC_DBADDR0_BASE_ADDR_S   7U
#define HIC_DBADDR0_BASE_ADDR_M   0xFFFFFF80U   // Base address of device region
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICDBADDR1 register
//
//*************************************************************************************************
#define HIC_DBADDR1_BASE_ADDR_S   7U
#define HIC_DBADDR1_BASE_ADDR_M   0xFFFFFF80U   // Base address of device region
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICDBADDR2 register
//
//*************************************************************************************************
#define HIC_DBADDR2_BASE_ADDR_S   7U
#define HIC_DBADDR2_BASE_ADDR_M   0xFFFFFF80U   // Base address of device region
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICDBADDR3 register
//
//*************************************************************************************************
#define HIC_DBADDR3_BASE_ADDR_S   7U
#define HIC_DBADDR3_BASE_ADDR_M   0xFFFFFF80U   // Base address of device region
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICDBADDR4 register
//
//*************************************************************************************************
#define HIC_DBADDR4_BASE_ADDR_S   7U
#define HIC_DBADDR4_BASE_ADDR_M   0xFFFFFF80U   // Base address of device region
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICDBADDR5 register
//
//*************************************************************************************************
#define HIC_DBADDR5_BASE_ADDR_S   7U
#define HIC_DBADDR5_BASE_ADDR_M   0xFFFFFF80U   // Base address of device region
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICDBADDR6 register
//
//*************************************************************************************************
#define HIC_DBADDR6_BASE_ADDR_S   7U
#define HIC_DBADDR6_BASE_ADDR_M   0xFFFFFF80U   // Base address of device region
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICDBADDR7 register
//
//*************************************************************************************************
#define HIC_DBADDR7_BASE_ADDR_S   7U
#define HIC_DBADDR7_BASE_ADDR_M   0xFFFFFF80U   // Base address of device region
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICH2DINTEN register
//
//*************************************************************************************************
#define HIC_H2DINTEN_H2D_INTEN      0x1U   // Host To Device Interrupt Enable
#define HIC_H2DINTEN_BUSERR_INTEN   0x2U   // BusError Interrupt Enable
#define HIC_H2DINTEN_ILLWR_INTEN    0x4U   // Illegal Write event interrupt enable
#define HIC_H2DINTEN_ILLRD_INTEN    0x8U   // Illegal Read event interrupt enable
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICH2DINTFLG register
//
//*************************************************************************************************
#define HIC_H2DINTFLG_H2D_FLG      0x1U   // Host To Device Interrupt Flag
#define HIC_H2DINTFLG_BUSERR_FLG   0x2U   // BusError Interrupt Flag
#define HIC_H2DINTFLG_ILLWR_FLG    0x4U   // Illegal write event interrupt flag
#define HIC_H2DINTFLG_ILLRD_FLG    0x8U   // Illegal read event interrupt flag
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICH2DINTCLR register
//
//*************************************************************************************************
#define HIC_H2DINTCLR_H2D_CLR      0x1U   // Host To Device Interrupt Clear
#define HIC_H2DINTCLR_BUSERR_CLR   0x2U   // BusError Interrupt  Clear
#define HIC_H2DINTCLR_ILLWR_CLR    0x4U   // Illegal Write Interrupt Clear
#define HIC_H2DINTCLR_ILLRD_CLR    0x8U   // Illegal Read Interrupt Clear
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICH2DINTFRC register
//
//*************************************************************************************************
#define HIC_H2DINTFRC_H2D_INTFRC      0x1U   // Host To Device Force Set
#define HIC_H2DINTFRC_BUSERR_INTFRC   0x2U   // BusError Interrupt Force Set
#define HIC_H2DINTFRC_ILLWR_INTFRC    0x4U   // Illegal Write Interrupt Force Set
#define HIC_H2DINTFRC_ILLRD_INTFRC    0x8U   // Illegal Read Interrupt Force Set
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICD2HINTEN register
//
//*************************************************************************************************
#define HIC_D2HINTEN_D2H_INTEN        0x1U          // Device to Host Data Ready Interrupt Enable
#define HIC_D2HINTEN_BUSERR_INTEN     0x2U          // BusError Interrupt Enable
#define HIC_D2HINTEN_ILLWR_INTEN      0x4U          // Illegal Write event Interrupt Enable
#define HIC_D2HINTEN_ILLRD_INTEN      0x8U          // Illegal Read event Interrupt Enable
#define HIC_D2HINTEN_ACCVIO_INTEN     0x10U         // Access Violation Interrupt Enable
#define HIC_D2HINTEN_EVTRIG_INTEN_S   16U
#define HIC_D2HINTEN_EVTRIG_INTEN_M   0xFFFF0000U   // Event Trigger Interrupt Enable
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICD2HINTFLG register
//
//*************************************************************************************************
#define HIC_D2HINTFLG_D2H_FLG        0x1U          // Device to Host Data Ready Flag
#define HIC_D2HINTFLG_BUSERR_FLG     0x2U          // BusError Flag
#define HIC_D2HINTFLG_ILLWR_FLG      0x4U          // Illegal Write event Flag
#define HIC_D2HINTFLG_ILLRD_FLG      0x8U          // Illegal Read event Flag
#define HIC_D2HINTFLG_ACCVIO_FLG     0x10U         // Access Violation Flag
#define HIC_D2HINTFLG_EVTRIG_FLG_S   16U
#define HIC_D2HINTFLG_EVTRIG_FLG_M   0xFFFF0000U   // Event Trigger Flag
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICD2HINTCLR register
//
//*************************************************************************************************
#define HIC_D2HINTCLR_D2H_CLR        0x1U          // Device to Host Interrupt Clear
#define HIC_D2HINTCLR_BUSERR_CLR     0x2U          // BusError Interrupt  Clear
#define HIC_D2HINTCLR_ILLWR_CLR      0x4U          // Illegal Write Interrupt Clear
#define HIC_D2HINTCLR_ILLRD_CLR      0x8U          // Illegal Read Interrupt Clear
#define HIC_D2HINTCLR_ACCVIO_CLR     0x10U         // Access Violation Interrupt Clear
#define HIC_D2HINTCLR_EVTRIG_CLR_S   16U
#define HIC_D2HINTCLR_EVTRIG_CLR_M   0xFFFF0000U   // Event Trigger Interrupt Clear
//*************************************************************************************************
//
// The following are defines for the bit fields in the HICD2HINTFRC register
//
//*************************************************************************************************
#define HIC_D2HINTFRC_D2H_INTFRC        0x1U          // Device to Host Force Set
#define HIC_D2HINTFRC_BUSERR_INTFRC     0x2U          // BusError Interrupt Force Set
#define HIC_D2HINTFRC_ILLWR_INTFRC      0x4U          // Illegal Write Interrupt Force Set
#define HIC_D2HINTFRC_ILLRD_INTFRC      0x8U          // Illegal Read Interrupt Force Set
#define HIC_D2HINTFRC_ACCVIO_INTFRC     0x10U         // Access Violation Interrupt Force Set
#define HIC_D2HINTFRC_EVTRIG_INTFRC_S   16U
#define HIC_D2HINTFRC_EVTRIG_INTFRC_M   0xFFFF0000U   // Event Trigger Interrupt Force Set


#endif
