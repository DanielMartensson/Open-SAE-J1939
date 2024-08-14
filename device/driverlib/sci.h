//###########################################################################
//
// FILE:   sci.h
//
// TITLE:  C28x SCI driver.
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

#ifndef SCI_H
#define SCI_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \addtogroup sci_api SCI
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_sci.h"
#include "inc/hw_types.h"
#include "debug.h"

//*****************************************************************************
//
// Values that can be passed to SCI_enableInterrupt, SCI_disableInterrupt, and
// SCI_clearInterruptStatus as the intFlags parameter, and returned from
// SCI_getInterruptStatus.
//
//*****************************************************************************
#define SCI_INT_RXERR          0x01U  //!< RXERR interrupt
#define SCI_INT_RXRDY_BRKDT    0x02U  //!< RXRDY interrupt
#define SCI_INT_TXRDY          0x04U  //!< TXRDY interrupt
#define SCI_INT_TXFF           0x08U  //!< TX FIFO level interrupt
#define SCI_INT_RXFF           0x10U  //!< RX FIFO level interrupt
#define SCI_INT_FE             0x20U  //!< Frame Error
#define SCI_INT_OE             0x40U  //!< Overrun Error
#define SCI_INT_PE             0x80U  //!< Parity Error

//*****************************************************************************
//
// Values that can be passed to SCI_setConfig as the config parameter
// and returned by SCI_getConfig in the config parameter.
// Additionally, the SCI_CONFIG_PAR_* enum subset can be passed to
// SCI_setParityMode as the parity parameter, and are returned by
// SCI_getParityMode.
//
//*****************************************************************************
#define SCI_CONFIG_WLEN_MASK   0x0007U  //!< Mask for extracting word length
#define SCI_CONFIG_WLEN_8      0x0007U  //!< 8 bit data
#define SCI_CONFIG_WLEN_7      0x0006U  //!< 7 bit data
#define SCI_CONFIG_WLEN_6      0x0005U  //!< 6 bit data
#define SCI_CONFIG_WLEN_5      0x0004U  //!< 5 bit data
#define SCI_CONFIG_WLEN_4      0x0003U  //!< 4 bit data
#define SCI_CONFIG_WLEN_3      0x0002U  //!< 3 bit data
#define SCI_CONFIG_WLEN_2      0x0001U  //!< 2 bit data
#define SCI_CONFIG_WLEN_1      0x0000U  //!< 1 bit data
#define SCI_CONFIG_STOP_MASK   0x0080U  //!< Mask for extracting stop bits
#define SCI_CONFIG_STOP_ONE    0x0000U  //!< One stop bit
#define SCI_CONFIG_STOP_TWO    0x0080U  //!< Two stop bits
#define SCI_CONFIG_PAR_MASK    0x0060U  //!< Parity Mask

//*****************************************************************************
//
//! Values that can be used with SCI_setParityMode() and SCI_getParityMode() to
//! describe the parity of the SCI communication.
//
//*****************************************************************************
typedef enum
{
    SCI_CONFIG_PAR_NONE = 0x0000U,  //!< No parity
    SCI_CONFIG_PAR_EVEN = 0x0060U,  //!< Even parity
    SCI_CONFIG_PAR_ODD  = 0x0020U   //!< Odd parity
} SCI_ParityType;

//*****************************************************************************
//
//! Values that can be passed to SCI_setFIFOInterruptLevel() as the txLevel
//! parameter and returned by SCI_getFIFOInteruptLevel() and
//! SCI_getTxFIFOStatus().
//
//*****************************************************************************
typedef enum
{
    SCI_FIFO_TX0  = 0x0000U, //!< Transmit interrupt empty
    SCI_FIFO_TX1  = 0x0001U, //!< Transmit interrupt 1/16 full
    SCI_FIFO_TX2  = 0x0002U, //!< Transmit interrupt 2/16 full
    SCI_FIFO_TX3  = 0x0003U, //!< Transmit interrupt 3/16 full
    SCI_FIFO_TX4  = 0x0004U, //!< Transmit interrupt 4/16 full
    SCI_FIFO_TX5  = 0x0005U, //!< Transmit interrupt 5/16 full
    SCI_FIFO_TX6  = 0x0006U, //!< Transmit interrupt 6/16 full
    SCI_FIFO_TX7  = 0x0007U, //!< Transmit interrupt 7/16 full
    SCI_FIFO_TX8  = 0x0008U, //!< Transmit interrupt 8/16 full
    SCI_FIFO_TX9  = 0x0009U, //!< Transmit interrupt 9/16 full
    SCI_FIFO_TX10 = 0x000AU, //!< Transmit interrupt 10/16 full
    SCI_FIFO_TX11 = 0x000BU, //!< Transmit interrupt 11/16 full
    SCI_FIFO_TX12 = 0x000CU, //!< Transmit interrupt 12/16 full
    SCI_FIFO_TX13 = 0x000DU, //!< Transmit interrupt 13/16 full
    SCI_FIFO_TX14 = 0x000EU, //!< Transmit interrupt 14/16 full
    SCI_FIFO_TX15 = 0x000FU, //!< Transmit interrupt 15/16 full
    SCI_FIFO_TX16 = 0x0010U  //!< Transmit interrupt full
} SCI_TxFIFOLevel;

//*****************************************************************************
//
//! Values that can be passed to SCI_setFIFOInterruptLevel() as the rxLevel
//! parameter and returned by SCI_getFIFOInterruptLevel() and
//! SCI_getRxFIFOStatus().
//
//*****************************************************************************
typedef enum
{
    SCI_FIFO_RX0  = 0x0000U, //!< Receive interrupt empty
    SCI_FIFO_RX1  = 0x0001U, //!< Receive interrupt 1/16 full
    SCI_FIFO_RX2  = 0x0002U, //!< Receive interrupt 2/16 full
    SCI_FIFO_RX3  = 0x0003U, //!< Receive interrupt 3/16 full
    SCI_FIFO_RX4  = 0x0004U, //!< Receive interrupt 4/16 full
    SCI_FIFO_RX5  = 0x0005U, //!< Receive interrupt 5/16 full
    SCI_FIFO_RX6  = 0x0006U, //!< Receive interrupt 6/16 full
    SCI_FIFO_RX7  = 0x0007U, //!< Receive interrupt 7/16 full
    SCI_FIFO_RX8  = 0x0008U, //!< Receive interrupt 8/16 full
    SCI_FIFO_RX9  = 0x0009U, //!< Receive interrupt 9/16 full
    SCI_FIFO_RX10 = 0x000AU, //!< Receive interrupt 10/16 full
    SCI_FIFO_RX11 = 0x000BU, //!< Receive interrupt 11/16 full
    SCI_FIFO_RX12 = 0x000CU, //!< Receive interrupt 12/16 full
    SCI_FIFO_RX13 = 0x000DU, //!< Receive interrupt 13/16 full
    SCI_FIFO_RX14 = 0x000EU, //!< Receive interrupt 14/16 full
    SCI_FIFO_RX15 = 0x000FU, //!< Receive interrupt 15/16 full
    SCI_FIFO_RX16 = 0x0010U  //!< Receive interrupt full
} SCI_RxFIFOLevel;

//*****************************************************************************
//
// Values returned from SCI_getRxStatus().  These correspond to the different
// bits and flags of the SCIRXST register.
//
//*****************************************************************************
#define SCI_RXSTATUS_WAKE       0x0002U  //!< Receiver wake up detect
#define SCI_RXSTATUS_PARITY     0x0004U  //!< Parity error
#define SCI_RXSTATUS_OVERRUN    0x0008U  //!< Overrun error
#define SCI_RXSTATUS_FRAMING    0x0010U  //!< Framing error
#define SCI_RXSTATUS_BREAK      0x0020U  //!< Break detect
#define SCI_RXSTATUS_READY      0x0040U  //!< Receiver ready
#define SCI_RXSTATUS_ERROR      0x0080U  //!< Receiver error

//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
//*****************************************************************************
//
//! \internal
//! Checks a SCI base address.
//!
//! \param base is the base address of the SCI port.
//!
//! This function determines if a SCI port base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
SCI_isBaseValid(uint32_t base)
{
    return(
           (base == SCIA_BASE) ||
           (base == SCIB_BASE) ||
           (base == SCIC_BASE) ||
           (base == SCID_BASE)
          );
}
#endif

//*****************************************************************************
//
//! Sets the type of parity.
//!
//! \param base is the base address of the SCI port.
//! \param parity specifies the type of parity to use.
//!
//! Sets the type of parity to use for transmitting and expect when receiving.
//! The \e parity parameter must be one of the following:
//! \b SCI_CONFIG_PAR_NONE, \b SCI_CONFIG_PAR_EVEN, \b SCI_CONFIG_PAR_ODD.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_setParityMode(uint32_t base, SCI_ParityType parity)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Set the parity mode.
    //
    HWREGH(base + SCI_O_CCR) = ((HWREGH(base + SCI_O_CCR) &
                                 ~(SCI_CONFIG_PAR_MASK)) | (uint16_t)parity);
}

//*****************************************************************************
//
//! Gets the type of parity currently being used.
//!
//! \param base is the base address of the SCI port.
//!
//! This function gets the type of parity used for transmitting data and
//! expected when receiving data.
//!
//! \return Returns the current parity settings, specified as one of the
//! following:
//! \b SCI_CONFIG_PAR_NONE, \b SCI_CONFIG_PAR_EVEN, \b SCI_CONFIG_PAR_ODD.
//
//*****************************************************************************
static inline SCI_ParityType
SCI_getParityMode(uint32_t base)
{
    uint16_t parity;

    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Return the current parity setting.
    //
    parity = (HWREGH(base + SCI_O_CCR) & (SCI_CONFIG_PAR_MASK));

    return((SCI_ParityType)parity);
}

//*****************************************************************************
//
//! Sets the multiprocessor protocol to address-bit mode.
//!
//! \param base is the base address of the SCI port.
//!
//! This function sets the multi-processor protocol to address-bit mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_setAddrMultiProcessorMode(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Enable the address-bit mode protocol
    //
    HWREGH(base + SCI_O_CCR) |= SCI_CCR_ADDRIDLE_MODE;
}

//*****************************************************************************
//
//! Sets the multiprocessor protocol to idle-line mode.
//!
//! \param base is the base address of the SCI port.
//!
//! This function sets the multi-processor protocol to idle-line protocol.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_setIdleMultiProcessorMode(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Disable the address-bit mode protocol
    //
    HWREGH(base + SCI_O_CCR) &= ~SCI_CCR_ADDRIDLE_MODE;
}

//*****************************************************************************
//
//! Locks Autobaud.
//!
//! \param base is the base address of the SCI port.
//!
//! This function performs an autobaud lock for the SCI.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_lockAutobaud(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Prime the baud register
    //
    HWREGH(base + SCI_O_HBAUD) = 0x0U;
    HWREGH(base + SCI_O_LBAUD) = 0x1U;

    //
    // Prepare for autobaud detection.
    // Set the CDC bit to enable autobaud detection and clear the ABD bit.
    //
    HWREGH(base + SCI_O_FFCT) |= SCI_FFCT_CDC;
    HWREGH(base + SCI_O_FFCT) |= SCI_FFCT_ABDCLR;

    //
    // Wait until we correctly read an 'A' or 'a' and lock
    //
    while((HWREGH(base + SCI_O_FFCT) & SCI_FFCT_ABD) != SCI_FFCT_ABD)
    {
    }

    //
    // After autobaud lock, clear the ABD and CDC bits
    //
    HWREGH(base + SCI_O_FFCT) |= SCI_FFCT_ABDCLR;
    HWREGH(base + SCI_O_FFCT) &= ~SCI_FFCT_CDC;
}

//*****************************************************************************
//
//! Sets the FIFO interrupt level at which interrupts are generated.
//!
//! \param base is the base address of the SCI port.
//! \param txLevel is the transmit FIFO interrupt level, specified as one of
//! the following:
//! \b SCI_FIFO_TX0, \b SCI_FIFO_TX1, \b SCI_FIFO_TX2, . . . or
//! \b SCI_FIFO_TX15.
//! \param rxLevel is the receive FIFO interrupt level, specified as one of
//! the following
//! \b SCI_FIFO_RX0, \b SCI_FIFO_RX1, \b SCI_FIFO_RX2, ... or \b SCI_FIFO_RX15.
//!
//! This function sets the FIFO level at which transmit and receive interrupts
//! are generated.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_setFIFOInterruptLevel(uint32_t base, SCI_TxFIFOLevel txLevel,
                          SCI_RxFIFOLevel rxLevel)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Set the FIFO interrupt levels.
    //
    HWREGH(base + SCI_O_FFTX) = (HWREGH(base + SCI_O_FFTX) &
                                 (~SCI_FFTX_TXFFIL_M)) | (uint16_t)txLevel;
    HWREGH(base + SCI_O_FFRX) = (HWREGH(base + SCI_O_FFRX) &
                                 (~SCI_FFRX_RXFFIL_M)) | (uint16_t)rxLevel;
}

//*****************************************************************************
//
//! Gets the FIFO interrupt level at which interrupts are generated.
//!
//! \param base is the base address of the SCI port.
//! \param txLevel is a pointer to storage for the transmit FIFO interrupt
//! level, returned as one of the following:
//! \b SCI_FIFO_TX0, \b SCI_FIFO_TX1, \b SCI_FIFO_TX2, ... or \b SCI_FIFO_TX15.
//! \param rxLevel is a pointer to storage for the receive FIFO interrupt
//! level, returned as one of the following:
//! \b SCI_FIFO_RX0, \b SCI_FIFO_RX1, \b SCI_FIFO_RX2, ... or \b SCI_FIFO_RX15.
//!
//! This function gets the FIFO level at which transmit and receive interrupts
//! are generated.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_getFIFOInterruptLevel(uint32_t base, SCI_TxFIFOLevel *txLevel,
                          SCI_RxFIFOLevel *rxLevel)
{

    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Extract the transmit and receive FIFO levels.
    //
    *txLevel = (SCI_TxFIFOLevel)(HWREGH(base + SCI_O_FFTX) &
                                 SCI_FFTX_TXFFIL_M);
    *rxLevel = (SCI_RxFIFOLevel)(HWREGH(base + SCI_O_FFRX) &
                                 SCI_FFRX_RXFFIL_M);
}

//*****************************************************************************
//
//! Gets the current configuration of a SCI.
//!
//! \param base is the base address of the SCI port.
//! \param lspclkHz is the rate of the clock supplied to the SCI module.  This
//! is the LSPCLK.
//! \param baud is a pointer to storage for the baud rate.
//! \param config is a pointer to storage for the data format.
//!
//! The baud rate and data format for the SCI is determined, given an
//! explicitly provided peripheral clock (hence the ExpClk suffix).  The
//! returned baud rate is the actual baud rate; it may not be the exact baud
//! rate requested or an ``official'' baud rate.  The data format returned in
//! \e config is enumerated the same as the \e config parameter of
//! SCI_setConfig().
//!
//! The peripheral clock is the low speed peripheral clock.  This will be
//! the value returned by SysCtl_getLowSeedClock(), or it can be explicitly
//! hard coded if it is constant and known (to save the code/execution overhead
//! of a call to SysCtl_getLowSpeedClock()).
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_getConfig(uint32_t base, uint32_t lspclkHz, uint32_t *baud,
              uint32_t *config)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Compute the baud rate.
    //
    *baud = lspclkHz /
            ((1U + (uint32_t)((uint32_t)(HWREGH(base + SCI_O_HBAUD) << 8U) |
               HWREGH(base + SCI_O_LBAUD))) * 8U);

    //
    // Get the parity, data length, and number of stop bits.
    //
    *config = HWREGH(base + SCI_O_CCR) & (SCI_CONFIG_PAR_MASK |
                                          SCI_CONFIG_STOP_MASK |
                                          SCI_CONFIG_WLEN_MASK);
}

//*****************************************************************************
//
//! Enables transmitting and receiving.
//!
//! \param base is the base address of the SCI port.
//!
//! Enables SCI by taking SCI out of the software reset. Sets the TXENA, and
//! RXENA bits which enables transmit and receive.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_enableModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Enable RX, TX, and the SCI.
    //
    HWREGH(base + SCI_O_CTL1) |= (SCI_CTL1_TXENA | SCI_CTL1_RXENA |
                                  SCI_CTL1_SWRESET);
}

//*****************************************************************************
//
//! Disables transmitting and receiving.
//!
//! \param base is the base address of the SCI port.
//!
//! Clears the SCIEN, TXE, and RXE bits. The user should ensure that all the
//! data has been sent before disable the module during transmission.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_disableModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Disable the FIFO.
    //
    HWREGH(base + SCI_O_FFTX) &= ~(SCI_FFTX_SCIFFENA);

    //
    // Disable the SCI.
    //
    HWREGH(base + SCI_O_CTL1) &= ~(SCI_CTL1_TXENA | SCI_CTL1_RXENA);
}

//*****************************************************************************
//
//! Enables transmitting.
//!
//! \param base is the base address of the SCI port.
//!
//! Enables SCI by taking SCI out of the software reset. Sets the TXENA bit
//! which enables transmit.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_enableTxModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Enable TX and the SCI.
    //
    HWREGH(base + SCI_O_CTL1) |= (SCI_CTL1_TXENA | SCI_CTL1_SWRESET);
}

//*****************************************************************************
//
//! Disables transmitting.
//!
//! \param base is the base address of the SCI port.
//!
//! Disables SCI by taking SCI out of the software reset. Clears the TXENA bit
//! which disables transmit.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_disableTxModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Disable TX.
    //
    HWREGH(base + SCI_O_CTL1) &= ~SCI_CTL1_TXENA;
}

//*****************************************************************************
//
//! Enables receiving.
//!
//! \param base is the base address of the SCI port.
//!
//! Enables SCI by taking SCI out of the software reset. Sets the RXENA bit
//! which enables receive.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_enableRxModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Enable RX and the SCI.
    //
    HWREGH(base + SCI_O_CTL1) |= (SCI_CTL1_RXENA | SCI_CTL1_SWRESET);
}

//*****************************************************************************
//
//! Disables receiving.
//!
//! \param base is the base address of the SCI port.
//!
//! Disables SCI by taking SCI out of the software reset. Clears the RXENA bit
//! which disables receive.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_disableRxModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Disable RX.
    //
    HWREGH(base + SCI_O_CTL1) &= ~SCI_CTL1_RXENA;
}

//*****************************************************************************
//
//! Enables Sleep Mode
//!
//! \param base is the base address of the SCI port.
//!
//! Enables the sleep mode in SCI by setting the SLEEP bit in SCICTL1 register
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_enableSleepMode(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Set sleep bit
    //
    HWREGH(base + SCI_O_CTL1) |= SCI_CTL1_SLEEP;
}

//*****************************************************************************
//
//! Disables Sleep Mode
//!
//! \param base is the base address of the SCI port.
//!
//! Disables the sleep mode in SCI by clearing the SLEEP bit in SCICTL1 register
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_disableSleepMode(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Clear sleep bit
    //
    HWREGH(base + SCI_O_CTL1) &= ~SCI_CTL1_SLEEP;
}

//*****************************************************************************
//
//! Enables the transmit and receive FIFOs.
//!
//! \param base is the base address of the SCI port.
//!
//! This functions enables the transmit and receive FIFOs in the SCI.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_enableFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Enable the FIFO.
    //
    HWREGH(base + SCI_O_FFTX) |= SCI_FFTX_SCIRST;
    HWREGH(base + SCI_O_FFTX) |= SCI_FFTX_SCIFFENA | SCI_FFTX_TXFIFORESET;
    HWREGH(base + SCI_O_FFRX) |= SCI_FFRX_RXFIFORESET;
}

//*****************************************************************************
//
//! Disables the transmit and receive FIFOs.
//!
//! \param base is the base address of the SCI port.
//!
//! This functions disables the transmit and receive FIFOs in the SCI.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_disableFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Disable the FIFO.
    //
    HWREGH(base + SCI_O_FFTX) &= ~SCI_FFTX_SCIFFENA;
}

//*****************************************************************************
//
//! Determines if the FIFO enhancement is enabled.
//!
//! \param base is the base address of the SCI port.
//!
//! This function returns a flag indicating whether or not the FIFO enhancement
//! is enabled.
//!
//! \return Returns \b true if the FIFO enhancement is enabled or \b false
//! if the FIFO enhancement is disabled.
//
//*****************************************************************************
static inline bool
SCI_isFIFOEnabled(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Return true if the FIFO is enabled and false if it is disabled.
    //
    return(((HWREGH(base + SCI_O_FFTX) & SCI_FFTX_SCIFFENA) ==
            SCI_FFTX_SCIFFENA) ? true : false);
}

//*****************************************************************************
//
//! Resets the receive FIFO.
//!
//! \param base is the base address of the SCI port.
//!
//! This functions resets the receive FIFO of the SCI.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_resetRxFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Reset the specified FIFO.
    //
    HWREGH(base + SCI_O_FFRX) &= ~SCI_FFRX_RXFIFORESET;
    HWREGH(base + SCI_O_FFRX) |= SCI_FFRX_RXFIFORESET;
}

//*****************************************************************************
//
//! Resets the transmit FIFO.
//!
//! \param base is the base address of the SCI port.
//!
//! This functions resets the transmit FIFO of the SCI.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_resetTxFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Reset the specified FIFO.
    //
    HWREGH(base + SCI_O_FFTX) &= ~SCI_FFTX_TXFIFORESET;
    HWREGH(base + SCI_O_FFTX) |= SCI_FFTX_TXFIFORESET;
}

//*****************************************************************************
//
//! Resets the SCI Transmit and Receive Channels
//!
//! \param base is the base address of the SCI port.
//!
//! This functions resets transmit and receive channels in the SCI.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_resetChannels(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Reset the Tx and Rx Channels
    //
    HWREGH(base + SCI_O_FFTX) &= ~SCI_FFTX_SCIRST;
    HWREGH(base + SCI_O_FFTX) |= SCI_FFTX_SCIRST;
}

//*****************************************************************************
//
//! Determines if there are any characters in the receive buffer when the
//! FIFO enhancement is not enabled.
//!
//! \param base is the base address of the SCI port.
//!
//! This function returns a flag indicating whether or not there is data
//! available in the receive buffer.
//!
//! \return Returns \b true if there is data in the receive buffer or \b false
//! if there is no data in the receive buffer.
//
//*****************************************************************************
static inline bool
SCI_isDataAvailableNonFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Return the availability of characters with FIFO disabled.
    //
    return(((HWREGH(base + SCI_O_RXST) & SCI_RXST_RXRDY) ==
            SCI_RXST_RXRDY) ? true : false);
}

//*****************************************************************************
//
//! Determines if there is any space in the transmit buffer when the FIFO
//! enhancement is not enabled.
//!
//! \param base is the base address of the SCI port.
//!
//! This function returns a flag indicating whether or not there is space
//! available in the transmit buffer when not using the FIFO enhancement.
//!
//! \return Returns \b true if there is space available in the transmit buffer
//! or \b false if there is no space available in the transmit buffer.
//
//*****************************************************************************
static inline bool
SCI_isSpaceAvailableNonFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Return the availability of space.
    //
    return(((HWREGH(base + SCI_O_CTL2) & SCI_CTL2_TXRDY) ==
            SCI_CTL2_TXRDY) ? true : false);
}

//*****************************************************************************
//
//! Get the transmit FIFO status
//!
//! \param base is the base address of the SCI port.
//!
//! This functions gets the current number of words in the transmit FIFO.
//!
//! \return Returns the current number of words in the transmit FIFO specified
//! as one of the following:
//! \b SCI_FIFO_TX0, \b SCI_FIFO_TX1, \b SCI_FIFO_TX2, \b SCI_FIFO_TX3
//! \b SCI_FIFO_TX4, ..., or \b SCI_FIFO_TX16
//
//*****************************************************************************
static inline SCI_TxFIFOLevel
SCI_getTxFIFOStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Get the current FIFO status
    //
    return((SCI_TxFIFOLevel)((HWREGH(base + SCI_O_FFTX) & SCI_FFTX_TXFFST_M) >>
                             SCI_FFTX_TXFFST_S));
}

//*****************************************************************************
//
//! Get the receive FIFO status
//!
//! \param base is the base address of the SCI port.
//!
//! This functions gets the current number of words in the receive FIFO.
//!
//! \return Returns the current number of words in the receive FIFO specified
//! as one of the following:
//! \b SCI_FIFO_RX0, \b SCI_FIFO_RX1, \b SCI_FIFO_RX2, \b SCI_FIFO_RX3
//! \b SCI_FIFO_RX4, ..., or \b SCI_FIFO_RX16
//
//*****************************************************************************
static inline SCI_RxFIFOLevel
SCI_getRxFIFOStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Get the current FIFO status
    //
    return((SCI_RxFIFOLevel)((HWREGH(base + SCI_O_FFRX) & SCI_FFRX_RXFFST_M) >>
                             SCI_FFRX_RXFFST_S));
}

//*****************************************************************************
//
//! Determines whether the SCI transmitter is busy or not.
//!
//! \param base is the base address of the SCI port.
//!
//! Allows the caller to determine whether all transmitted bytes have cleared
//! the transmitter hardware when the FIFO is not enabled.  When the FIFO is
//! enabled, this function allows the caller to determine whether there is any
//! data in the FIFO.
//!
//! Without the FIFO enabled, if \b false is returned, the transmit buffer and
//! shift registers are empty and the transmitter is not busy. With the FIFO
//! enabled, if \b false is returned, the FIFO is empty.  This does not
//! necessarily mean that the transmitter is not busy.  The empty FIFO does not
//! reflect the status of the transmitter shift register. The FIFO may be empty
//! while the transmitter is still transmitting data.
//!
//! \return Returns \b true if the SCI is transmitting or \b false if
//! transmissions are complete.
//
//*****************************************************************************
static inline bool
SCI_isTransmitterBusy(uint32_t base)
{
    //
    // Check the argument.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Check if FIFO enhancement is enabled.
    //
    if(SCI_isFIFOEnabled(base))
    {
        //
        // With FIFO enhancement, determine if the SCI is busy.
        //
        return(((HWREGH(base + SCI_O_FFTX) & SCI_FFTX_TXFFST_M) !=
                 0) ? true : false);
    }
    else
    {
        //
        // Without FIFO enhancement, determine if the SCI is busy.
        // Check if the transmit buffer and shift register empty.
        //
        return(((HWREGH(base + SCI_O_CTL2) & SCI_CTL2_TXEMPTY) ==
                SCI_CTL2_TXEMPTY) ? false : true);
    }
}

//*****************************************************************************
//
//! Waits to send a character from the specified port when the FIFO enhancement
//! is enabled.
//!
//! \param base is the base address of the SCI port.
//! \param data is the character to be transmitted.
//!
//! Sends the character \e data to the transmit buffer for the specified port.
//! If there is no space available in the transmit FIFO, this function waits
//! until there is space available before returning. \e data is a uint16_t but
//! only 8 bits are written to the SCI port.  SCI only transmits 8 bit
//! characters.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_writeCharBlockingFIFO(uint32_t base, uint16_t data)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Wait until space is available in the transmit FIFO.
    //
    while(SCI_getTxFIFOStatus(base) == SCI_FIFO_TX16)
    {
    }

    //
    // Send a char.
    //
    HWREGH(base + SCI_O_TXBUF) = data;
}

//*****************************************************************************
//
//! Waits to send a character from the specified port.
//!
//! \param base is the base address of the SCI port.
//! \param data is the character to be transmitted.
//!
//! Sends the character \e data to the transmit buffer for the specified port.
//! If there is no space available in the transmit buffer, or the transmit
//! FIFO if it is enabled, this function waits until there is space available
//! before returning. \e data is a uint16_t but only 8 bits are written to the
//! SCI port.  SCI only transmits 8 bit characters.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_writeCharBlockingNonFIFO(uint32_t base, uint16_t data)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Wait until space is available in the transmit buffer.
    //
    while(!SCI_isSpaceAvailableNonFIFO(base))
    {
    }

    //
    // Send a char.
    //
    HWREGH(base + SCI_O_TXBUF) = data;
}

//*****************************************************************************
//
//! Sends a character to the specified port.
//!
//! \param base is the base address of the SCI port.
//! \param data is the character to be transmitted.
//!
//! Writes the character \e data to the transmit buffer for the specified port.
//! This function does not block and only writes to the transmit buffer.
//! The user should use SCI_isSpaceAvailableNonFIFO() or SCI_getTxFIFOStatus()
//! to determine if the transmit buffer or FIFO have space available.
//! \e data is a uint16_t but only 8 bits are written to the SCI port.  SCI
//! only transmits 8 bit characters.
//!
//! This function replaces the original SCICharNonBlockingPut() API and
//! performs the same actions.  A macro is provided in <tt>sci.h</tt> to map
//! the original API to this API.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_writeCharNonBlocking(uint32_t base, uint16_t data)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Send a char.
    //
    HWREGH(base + SCI_O_TXBUF) = data;
}

//*****************************************************************************
//
//! Gets current receiver status flags.
//!
//! \param base is the base address of the SCI port.
//!
//! This function returns the current receiver status flags.  The returned
//! error flags are equivalent to the error bits returned via the previous
//! reading or receiving of a character with the exception that the overrun
//! error is set immediately the overrun occurs rather than when a character
//! is next read.
//!
//! \return Returns a bitwise OR combination of the receiver status flags,
//! \b SCI_RXSTATUS_WAKE, \b SCI_RXSTATUS_PARITY, \b SCI_RXSTATUS_OVERRUN,
//! \b SCI_RXSTATUS_FRAMING, \b SCI_RXSTATUS_BREAK, \b SCI_RXSTATUS_READY,
//! and \b SCI_RXSTATUS_ERROR.
//
//*****************************************************************************
static inline uint16_t
SCI_getRxStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Return the current value of the receive status register.
    //
    return(HWREGH(base + SCI_O_RXST));
}

//*****************************************************************************
//
//! Waits for a character from the specified port when the FIFO enhancement
//! is enabled.
//!
//! \param base is the base address of the SCI port.
//!
//! Gets a character from the receive FIFO for the specified port.  If there
//! are no characters available, this function waits until a character is
//! received before returning. Returns immediately in case of Error.
//!
//! \return Returns the character read from the specified port as \e uint16_t
//!         or 0x0 in case of Error. The application must use
//!         SCI_getRxStatus() API to check if some error occurred before
//!         consuming the data
//
//*****************************************************************************
static inline uint16_t
SCI_readCharBlockingFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Wait until a character is available in the receive FIFO.
    //
    while(SCI_getRxFIFOStatus(base) == SCI_FIFO_RX0)
    {
        //
        //If there is any error return
        //
        if((SCI_getRxStatus(base) & SCI_RXSTATUS_ERROR) != 0U)
        {
            return(0U);
        }
    }

    //
    // Return the character from the receive buffer.
    //
    return((uint16_t)(HWREGH(base + SCI_O_RXBUF) & SCI_RXBUF_SAR_M));
}

//*****************************************************************************
//
//! Waits for a character from the specified port when the FIFO enhancement
//! is not enabled.
//!
//! \param base is the base address of the SCI port.
//!
//! Gets a character from the receive buffer for the specified port.  If there
//! is no characters available, this function waits until a character is
//! received before returning.
//!
//! \return Returns the character read from the specified port as \e uint16_t.
//
//*****************************************************************************
static inline uint16_t
SCI_readCharBlockingNonFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Wait until a character is available in the receive FIFO.
    //
    while(!SCI_isDataAvailableNonFIFO(base))
    {
    }

    //
    // Return the character from the receive buffer.
    //
    return((uint16_t)(HWREGH(base + SCI_O_RXBUF) & SCI_RXBUF_SAR_M));
}

//*****************************************************************************
//
//! Receives a character from the specified port.
//!
//! \param base is the base address of the SCI port.
//!
//! Gets a character from the receive buffer for the specified port. This
//! function does not block and only reads the receive buffer.  The user should
//! use SCI_isDataAvailableNonFIFO() or SCI_getRxFIFOStatus() to determine if
//! the receive buffer or FIFO have data available.
//!
//! This function replaces the original SCICharNonBlockingGet() API and
//! performs the same actions.  A macro is provided in <tt>sci.h</tt> to map
//! the original API to this API.
//!
//! \return Returns \e uin16_t which is read from the receive buffer.
//
//*****************************************************************************
static inline uint16_t
SCI_readCharNonBlocking(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Return the character from the receive buffer.
    //
    return((uint16_t)(HWREGH(base + SCI_O_RXBUF) & SCI_RXBUF_SAR_M));
}

//*****************************************************************************
//
//! Performs a software reset of the SCI and Clears all reported receiver
//! status flags.
//!
//! \param base is the base address of the SCI port.
//!
//! This function performs a software reset of the SCI port.  It affects the
//! operating flags of the SCI, but it neither affects the configuration bits
//! nor restores the reset values.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_performSoftwareReset(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // To clear all errors a sw reset of the module is required
    //
    HWREGH(base + SCI_O_CTL1) &= ~SCI_CTL1_SWRESET;
    HWREGH(base + SCI_O_CTL1) |= SCI_CTL1_SWRESET;
}

//*****************************************************************************
//
//! Enables Loop Back Test Mode
//!
//! \param base is the base address of the SCI port.
//!
//! Enables the loop back test mode where the Tx pin is internally connected
//! to the Rx pin.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_enableLoopback(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Set the loop back mode.
    //
    HWREGH(base + SCI_O_CCR) |= SCI_CCR_LOOPBKENA;
}

//*****************************************************************************
//
//! Disables Loop Back Test Mode
//!
//! \param base is the base address of the SCI port.
//!
//! Disables the loop back test mode where the Tx pin is no longer internally
//! connected to the Rx pin.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_disableLoopback(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Clear the loop back mode.
    //
    HWREGH(base + SCI_O_CCR) &= ~SCI_CCR_LOOPBKENA;
}

//*****************************************************************************
//
//! Get the receive FIFO Overflow flag status
//!
//! \param base is the base address of the SCI port.
//!
//! This functions gets the receive FIFO overflow flag status.
//!
//! \return Returns \b true if overflow has occurred, else returned \b false if
//! an overflow hasn't occurred.
//
//*****************************************************************************
static inline bool
SCI_getOverflowStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Return the current FIFO overflow status
    //
    return((HWREGH(base + SCI_O_FFRX) & SCI_FFRX_RXFFOVF) == SCI_FFRX_RXFFOVF);
}

//*****************************************************************************
//
//! Clear the receive FIFO Overflow flag status
//!
//! \param base is the base address of the SCI port.
//!
//! This functions clears the receive FIFO overflow flag status.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SCI_clearOverflowStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Clear the current FIFO overflow status
    //
    HWREGH(base + SCI_O_FFRX) |= SCI_FFRX_RXFFOVRCLR;
}

//*****************************************************************************
//
//! Sets the configuration of a SCI.
//!
//! \param base is the base address of the SCI port.
//! \param lspclkHz is the rate of the clock supplied to the SCI module.  This
//! is the LSPCLK.
//! \param baud is the desired baud rate.
//! \param config is the data format for the port (number of data bits,
//! number of stop bits, and parity).
//!
//! This function configures the SCI for operation in the specified data
//! format.  The baud rate is provided in the \e baud parameter and the data
//! format in the \e config parameter.
//!
//! The \e config parameter is the bitwise OR of three values: the number of
//! data bits, the number of stop bits, and the parity.  \b SCI_CONFIG_WLEN_8,
//! \b SCI_CONFIG_WLEN_7, \b SCI_CONFIG_WLEN_6, \b SCI_CONFIG_WLEN_5,
//! \b SCI_CONFIG_WLEN_4, \b SCI_CONFIG_WLEN_3, \b SCI_CONFIG_WLEN_2, and
//! \b SCI_CONFIG_WLEN_1. Select from eight to one data bits per byte
//! (respectively).
//! \b SCI_CONFIG_STOP_ONE and \b SCI_CONFIG_STOP_TWO select one or two stop
//! bits (respectively).  \b SCI_CONFIG_PAR_NONE, \b SCI_CONFIG_PAR_EVEN,
//! \b SCI_CONFIG_PAR_ODD, select the parity mode (no parity bit, even parity
//! bit, odd parity bit respectively).
//!
//! The peripheral clock is the low speed peripheral clock.  This will be
//! the value returned by SysCtl_getLowSpeedClock(), or it can be explicitly
//! hard coded if it is constant and known (to save the code/execution overhead
//! of a call to SysCtl_getLowSpeedClock()).
//!
//! A baud rate divider (BRR) is used in this function to calculate the
//! baud rate. The value of BRR is calculated in float and type casted as int
//! to be fed in the \b SCIHBAUD and  \b SCILBAUD registers. This conversion
//! brings an error in the calculated baud rate and the requested. Error will
//! be significant when operating at higher baud rates. The error is due to
//! lower BRR integer value granularity at higher baud rates.
//!
//! \return None.
//
//*****************************************************************************
extern void
SCI_setConfig(uint32_t base, uint32_t lspclkHz, uint32_t baud,
              uint32_t config);

//*****************************************************************************
//
//! Waits to send an array of characters from the specified port.
//!
//! \param base is the base address of the SCI port.
//! \param array is the address of the array of characters to be transmitted.
//!   It is pointer to the array of characters to be transmitted.
//! \param length is the length of the array, or number of characters in the
//!   array to be transmitted.
//!
//! Sends the number of characters specified by \e length, starting at the
//! address \e array, out of the transmit buffer for the specified port.
//! If there is no space available in the transmit buffer, or the transmit
//! FIFO if it is enabled, this function waits until there is space available
//! and \e length number of characters are transmitted before returning.
//! \e array is a pointer to uint16_ts but only the least significant 8 bits
//! are written to the SCI port.  SCI only transmits 8 bit characters.
//!
//! \return None.
//
//*****************************************************************************
extern void
SCI_writeCharArray(uint32_t base, const uint16_t * const array,
                   uint16_t length);

//*****************************************************************************
//
//! Waits to receive an array of characters from the specified port.
//!
//! \param base is the base address of the SCI port.
//! \param array is the address of the array of characters to be received.
//!   It is a pointer to the array of characters to be received.
//! \param length is the length of the array, or number of characters in the
//!   array to be received.
//!
//! Receives an array of characters from the receive buffer for the specified
//! port, and stores them as an array of characters starting at address
//! \e array.  This function waits until the \e length number of characters are
//! received before returning.
//!
//! \return None.
//
//*****************************************************************************
extern void
SCI_readCharArray(uint32_t base, uint16_t * const array, uint16_t length);

//*****************************************************************************
//
//! Enables individual SCI interrupt sources.
//!
//! \param base is the base address of the SCI port.
//! \param intFlags is the bit mask of the interrupt sources to be enabled.
//!
//! Enables the indicated SCI interrupt sources.  Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor.
//!
//! The \e intFlags parameter is the bitwise OR of any of the following:
//!
//! - \b SCI_INT_RXERR      - RXERR Interrupt
//! - \b SCI_INT_RXRDY_BRKDT - RXRDY/BRKDT Interrupt
//! - \b SCI_INT_TXRDY      - TXRDY Interrupt
//! - \b SCI_INT_TXFF       - TX FIFO Level Interrupt
//! - \b SCI_INT_RXFF       - RX FIFO Level Interrupt
//! - \b SCI_INT_FE         - Frame Error
//! - \b SCI_INT_OE         - Overrun Error
//! - \b SCI_INT_PE         - Parity Error
//!
//! \return None.
//
//*****************************************************************************
extern void
SCI_enableInterrupt(uint32_t base, uint32_t intFlags);

//*****************************************************************************
//
//! Disables individual SCI interrupt sources.
//!
//! \param base is the base address of the SCI port.
//! \param intFlags is the bit mask of the interrupt sources to be disabled.
//!
//! Disables the indicated SCI interrupt sources.  Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor.
//!
//! The \e intFlags parameter has the same definition as the \e intFlags
//! parameter to SCI_enableInterrupt().
//!
//! \return None.
//
//*****************************************************************************
extern void
SCI_disableInterrupt(uint32_t base, uint32_t intFlags);

//*****************************************************************************
//
//! Gets the current interrupt status.
//!
//! \param base is the base address of the SCI port.
//!
//! \return Returns the current interrupt status, enumerated as a bit field of
//! values described in SCI_enableInterrupt().
//
//*****************************************************************************
extern uint32_t
SCI_getInterruptStatus(uint32_t base);

//*****************************************************************************
//
//! Clears SCI interrupt sources.
//!
//! \param base is the base address of the SCI port.
//! \param intFlags is a bit mask of the interrupt sources to be cleared.
//!
//! The specified SCI interrupt sources are cleared, so that they no longer
//! assert.  This function must be called in the interrupt handler to keep the
//! interrupt from being recognized again immediately upon exit.
//!
//! The \e intFlags parameter has the same definition as the \e intFlags
//! parameter to SCI_enableInterrupt().
//!
//! \return None.
//
//*****************************************************************************
extern void
SCI_clearInterruptStatus(uint32_t base, uint32_t intFlags);

//*****************************************************************************
//
//! Sets SCI Baud rate.
//!
//! \param base is the base address of the SCI port.
//! \param lspclkHz is the rate of the clock supplied to the SCI module.  This
//! is the LSPCLK.
//! \param baud is the desired baud rate.
//!
//! This function configures the SCI for operation in the specified baud rate
//! The baud rate is provided in the \e baud parameter.
//!
//! The peripheral clock is the low speed peripheral clock.  This will be
//! the value returned by SysCtl_getLowSpeedClock()
//!
//! \return None.
//
//*****************************************************************************
extern void
SCI_setBaud(uint32_t base, uint32_t lspclkHz, uint32_t baud);

//*****************************************************************************
//
//! Sets the SCI TXWAKE flag
//!
//! \param base is the base address of the SCI port.
//!
//! This function sets the TXWAKE flag bit to indicate that the next frame
//! is an address frame.
//! TXWAKE bit controls selection of data-transmit feature based on
//! which mode is selected from idle-line and address-bit.
//!
//! \return None.
//
//*****************************************************************************
extern void
SCI_setWakeFlag(uint32_t base);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //  SCI_H
