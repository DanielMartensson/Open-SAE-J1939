//###########################################################################
//
// FILE:   spi.h
//
// TITLE:  C28x SPI driver.
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

#ifndef SPI_H
#define SPI_H

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
//! \addtogroup spi_api SPI
//! \brief This module is used for SPI configurations.
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_spi.h"
#include "debug.h"
#include "hw_reg_inclusive_terminology.h"

#ifndef DOXYGEN_PDF_IGNORE
//*****************************************************************************
//
// Values that can be passed to SPI_enableInterrupt(), SPI_disableInterrupt(),
// and SPI_clearInterruptStatus() as the intFlags parameter, and returned by
// SPI_getInterruptStatus().
//
//*****************************************************************************
#define SPI_INT_RX_OVERRUN        0x0001U //!< Receive overrun interrupt
#define SPI_INT_RX_DATA_TX_EMPTY  0x0002U //!< Data received, transmit empty
#define SPI_INT_RXFF              0x0004U //!< RX FIFO level interrupt
#define SPI_INT_TXFF              0x0008U //!< TX FIFO level interrupt
#define SPI_INT_RXFF_OVERFLOW     0x0010U //!< RX FIFO overflow
#endif


//*****************************************************************************
//
//! This macro definition is used to transmit a byte of data
//!
//! \param base specifies the SPI module base address.
//! \param txData is the data to be transmitted over SPI
//!
//! This macro definition is to transmit a byte of data.
//! This macro uses SPI_pollingNonFIFOTransaction function
//! SPI character length is hardcoded to 8 (1 byte = 8 bits)of character length
//!
//! \return None.
//
//*****************************************************************************
#define SPI_transmitByte(base, txData)                                         \
                                SPI_pollingNonFIFOTransaction(base, 8U,  txData)

//*****************************************************************************
//
//! This macro definition is used to transmit a 16-bit word of data
//!
//! \param base specifies the SPI module base address.
//! \param txData is the data to be transmitted over SPI
//!
//! This macro definition is to transmit a 16-bit word of data.
//! This macro uses SPI_pollingNonFIFOTransaction function
//! SPI character length is hardcoded to 16 (16bit word) of character length
//!
//! \return None.
//
//*****************************************************************************
#define SPI_transmit16Bits(base, txData)                                       \
                                SPI_pollingNonFIFOTransaction(base, 16U, txData)

//*****************************************************************************
//
//! This macro definition can be used to transmit 'N' bytes of data
//!
//! \param base specifies the SPI module base address.
//! \param txBuffer is the transmit buffer to be transmitted over SPI
//! \param numOfWords is the number of bytes to be transmitted
//! \param txDelay specifies the number of serial clock cycles delay time after
//!        completion of perious word
//!
//! This macro definition can be used to transmit 'N' bytes of data.
//! This macro definition uses SPI_pollingFIFOTransaction function.
//!
//! SPI character length is hardcoded to 8 (8bits) of character length
//!
//! \return None.
//
//*****************************************************************************
#define SPI_transmitNBytes(base, txBuffer, numOfWords, txDelay)                \
      SPI_pollingFIFOTransaction(base, 8U,  txBuffer, NULL, numOfWords, txDelay)

//*****************************************************************************
//
//! This macro definition can be used to transmit 'N' 16-bit words of data
//!
//! \param base specifies the SPI module base address.
//! \param txBuffer is the transmit buffer to be transmitted over SPI
//! \param numOfWords is the number of 16-bit word to be transmitted
//! \param txDelay specifies the number of serial clock cycles delay time after
//!        completion of perious word
//!
//! This function can be used to transmit 'N' 16-bit words of data.
//! This function uses SPI_pollingFIFOTransaction function.
//! SPI character length is hardcoded to 16 (16-bit word)
//!
//! \return None.
//
//*****************************************************************************
#define SPI_transmitN16BitWord(base, txBuffer, numOfWords, txDelay)            \
      SPI_pollingFIFOTransaction(base, 16U, txBuffer, NULL, numOfWords, txDelay)

//*****************************************************************************
//
//! This macro definition can be used to transmit 'N' with configurable
//! SPI character length
//!
//! \param base specifies the SPI module base address
//! \param charLength specifies the SPI character length
//! \param txBuffer is the transmit buffer to be transmitted over SPI
//! \param numOfWords is the number of 16-bit word to be transmitted
//! \param txDelay specifies the number of serial clock cycles delay time after
//!        completion of perious word
//!
//! This macro definition can be used to transmit 'N' with configurable
//! SPI character length.
//!
//! This macro uses SPIpolling_FIFO_Transaction function.
//! SPI character length is configurable using charLength variable.
//!
//! \return None.
//
//*****************************************************************************
#define SPI_transmitNWordsWithCharLength(base, charLength, txBuffer,           \
                                         numOfWords, txDelay)                  \
              SPI_pollingFIFOTransaction(base, charLength, txBuffer, NULL,     \
                                         numOfWords, txDelay)

//*****************************************************************************
//
//! This macro definition is used to receive a byte of data
//!
//! \param base specifies the SPI module base address.
//! \param dummyData is the data which is transmitted to initiate
//!        SPI transaction to receive SPI data
//!
//! This macro definition is to receive a byte of data.
//! This macro uses SPI_pollingNonFIFOTransaction function
//! SPI character length is hardcoded to 8 (1byte = 8bits) of character length
//!
//! \return the received byte.
//
//*****************************************************************************
#define SPI_receiveByte(base, dummyData)                                       \
                            SPI_pollingNonFIFOTransaction(base, 8U, dummyData)

//*****************************************************************************
//
//! This macro is used to receive 'N' bytes of data
//!
//! \param base specifies the SPI module base address.
//! \param rxBuffer specifies receive buffer which will store the received bytes
//! \param numOfWords specifies the number of bytes to be received
//! \param txDelay specifies the number of serial clock cycles delay time after
//!        completion of perious word
//!
//! This function is used to receive 'N' bytes of data
//! This function uses SPIpolling_FIFO_Transaction function.
//! SPI character length is hardcoded to 8 (1 byte = 8 bits)
//!
//! \return None.
//
//*****************************************************************************
#define SPI_receiveNBytes(base, rxBuffer, numOfWords, txDelay)                 \
      SPI_pollingFIFOTransaction(base, 8U,  NULL, rxBuffer, numOfWords, txDelay)

//*****************************************************************************
//
//! This macro is used to receive 'N' 16-bits words of data
//!
//! \param base specifies the SPI module base address.
//! \param rxBuffer specifies receive buffer which will store the received bytes
//! \param numOfWords specifies the number of 16-bit words to be received
//! \param txDelay specifies the number of serial clock cycles delay time after
//!        completion of perious word
//!
//! This function is used to receive 'N' 16-bit words of data
//! This function uses SPIpolling_FIFO_Transaction function.
//! SPI character length is hardcoded to 16bits
//!
//! \return None.
//
//*****************************************************************************
#define SPI_receiveN16BitWord(base, rxBuffer, numOfWords, txDelay)             \
      SPI_pollingFIFOTransaction(base, 16U, NULL, rxBuffer, numOfWords, txDelay)

//*****************************************************************************
//
//! This macro is used to receive 'N' words with specified character length
//!
//! \param base specifies the SPI module base address.
//! \param charLength specifies the SPI character length of SPI transaction
//! \param rxBuffer specifies receive buffer which will store the received bytes
//! \param numOfWords specifies the number of words with specified character
//!        length
//! \param txDelay specifies the number of serial clock cycles delay time after
//!        completion of perious word
//!
//! This function is used to receive 'N' words with specified character length
//! This function uses SPIpolling_FIFO_Transaction function.
//! SPI character length is configurable using charLength variable
//!
//! \return None.
//
//*****************************************************************************
#define SPI_receiveNWordsWithcharLength(base, charLength, rxBuffer,            \
                                        numOfWords, txDelay)                   \
            SPI_pollingFIFOTransaction(base, charLength, NULL, rxBuffer,       \
                                       numOfWords, txDelay)

//*****************************************************************************
//
//! Values that can be passed to SPI_setConfig() as the \e protocol parameter.
//
//*****************************************************************************
typedef enum
{
    //! Mode 0. Polarity 0, phase 0. Rising edge without delay.
    SPI_PROT_POL0PHA0   = 0x0000U,
    //! Mode 1. Polarity 0, phase 1. Rising edge with delay.
    SPI_PROT_POL0PHA1   = 0x0002U,
    //! Mode 2. Polarity 1, phase 0. Falling edge without delay.
    SPI_PROT_POL1PHA0   = 0x0001U,
    //! Mode 3. Polarity 1, phase 1. Falling edge with delay.
    SPI_PROT_POL1PHA1   = 0x0003U
} SPI_TransferProtocol;

//*****************************************************************************
//
//! Values that can be passed to SPI_setConfig() as the \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    SPI_MODE_PERIPHERAL     = 0x0002U,   //!< SPI peripheral
    SPI_MODE_CONTROLLER     = 0x0006U,   //!< SPI controller
    SPI_MODE_PERIPHERAL_OD  = 0x0000U,   //!< SPI peripheral w/ output disabled
    SPI_MODE_CONTROLLER_OD  = 0x0004U    //!< SPI controller w/ output disabled
} SPI_Mode;

//*****************************************************************************
//
//! Values that can be passed to SPI_setFIFOInterruptLevel() as the \e txLevel
//! parameter, returned by SPI_getFIFOInterruptLevel() in the \e txLevel
//! parameter, and returned by SPI_getTxFIFOStatus().
//
//*****************************************************************************
typedef enum
{
    SPI_FIFO_TXEMPTY    = 0x0000U,      //!< Transmit FIFO empty
    SPI_FIFO_TX0        = 0x0000U,      //!< Transmit FIFO empty
    SPI_FIFO_TX1        = 0x0001U,      //!< Transmit FIFO 1/16 full
    SPI_FIFO_TX2        = 0x0002U,      //!< Transmit FIFO 2/16 full
    SPI_FIFO_TX3        = 0x0003U,      //!< Transmit FIFO 3/16 full
    SPI_FIFO_TX4        = 0x0004U,      //!< Transmit FIFO 4/16 full
    SPI_FIFO_TX5        = 0x0005U,      //!< Transmit FIFO 5/16 full
    SPI_FIFO_TX6        = 0x0006U,      //!< Transmit FIFO 6/16 full
    SPI_FIFO_TX7        = 0x0007U,      //!< Transmit FIFO 7/16 full
    SPI_FIFO_TX8        = 0x0008U,      //!< Transmit FIFO 8/16 full
    SPI_FIFO_TX9        = 0x0009U,      //!< Transmit FIFO 9/16 full
    SPI_FIFO_TX10       = 0x000AU,      //!< Transmit FIFO 10/16 full
    SPI_FIFO_TX11       = 0x000BU,      //!< Transmit FIFO 11/16 full
    SPI_FIFO_TX12       = 0x000CU,      //!< Transmit FIFO 12/16 full
    SPI_FIFO_TX13       = 0x000DU,      //!< Transmit FIFO 13/16 full
    SPI_FIFO_TX14       = 0x000EU,      //!< Transmit FIFO 14/16 full
    SPI_FIFO_TX15       = 0x000FU,      //!< Transmit FIFO 15/16 full
    SPI_FIFO_TX16       = 0x0010U,      //!< Transmit FIFO full
    SPI_FIFO_TXFULL     = 0x0010U       //!< Transmit FIFO full
} SPI_TxFIFOLevel;

//*****************************************************************************
//
//! Values that can be passed to SPI_setFIFOInterruptLevel() as the \e rxLevel
//! parameter, returned by SPI_getFIFOInterruptLevel() in the \e rxLevel
//! parameter, and returned by SPI_getRxFIFOStatus().
//
//*****************************************************************************
typedef enum
{
    SPI_FIFO_RXEMPTY    = 0x0000U,      //!< Receive FIFO empty
    SPI_FIFO_RX0        = 0x0000U,      //!< Receive FIFO empty
    SPI_FIFO_RX1        = 0x0001U,      //!< Receive FIFO 1/16 full
    SPI_FIFO_RX2        = 0x0002U,      //!< Receive FIFO 2/16 full
    SPI_FIFO_RX3        = 0x0003U,      //!< Receive FIFO 3/16 full
    SPI_FIFO_RX4        = 0x0004U,      //!< Receive FIFO 4/16 full
    SPI_FIFO_RX5        = 0x0005U,      //!< Receive FIFO 5/16 full
    SPI_FIFO_RX6        = 0x0006U,      //!< Receive FIFO 6/16 full
    SPI_FIFO_RX7        = 0x0007U,      //!< Receive FIFO 7/16 full
    SPI_FIFO_RX8        = 0x0008U,      //!< Receive FIFO 8/16 full
    SPI_FIFO_RX9        = 0x0009U,      //!< Receive FIFO 9/16 full
    SPI_FIFO_RX10       = 0x000AU,      //!< Receive FIFO 10/16 full
    SPI_FIFO_RX11       = 0x000BU,      //!< Receive FIFO 11/16 full
    SPI_FIFO_RX12       = 0x000CU,      //!< Receive FIFO 12/16 full
    SPI_FIFO_RX13       = 0x000DU,      //!< Receive FIFO 13/16 full
    SPI_FIFO_RX14       = 0x000EU,      //!< Receive FIFO 14/16 full
    SPI_FIFO_RX15       = 0x000FU,      //!< Receive FIFO 15/16 full
    SPI_FIFO_RX16       = 0x0010U,      //!< Receive FIFO full
    SPI_FIFO_RXFULL     = 0x0010U,      //!< Receive FIFO full
    SPI_FIFO_RXDEFAULT  = 0x001FU       //!< To prevent interrupt at reset
} SPI_RxFIFOLevel;

//*****************************************************************************
//
//! Values that can be passed to SPI_setEmulationMode() as the \e mode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! Transmission stops after midway in the bit stream
    SPI_EMULATION_STOP_MIDWAY         = 0x0000U,
    //! Continue SPI operation regardless
    SPI_EMULATION_FREE_RUN            = 0x0010U,
    //! Transmission will stop after a started transmission completes
    SPI_EMULATION_STOP_AFTER_TRANSMIT = 0x0020U
} SPI_EmulationMode;

//*****************************************************************************
//
//! Values that can be passed to SPI_setPTESignalPolarity() as the \e polarity
//! parameter.
//
//*****************************************************************************
typedef enum
{
    SPI_PTE_ACTIVE_LOW  = 0x0000U,        //!< SPIPTE is active low (normal)
    SPI_PTE_ACTIVE_HIGH = SPI_PRI_PTEINV  //!< SPIPTE is active high (inverted)
} SPI_PTEPolarity;

//*****************************************************************************
//
//! Values that can be passed to SPI_receive16Bits(), SPI_receive24Bits(),
//! SPI_receive32Bits()
//
//*****************************************************************************
typedef enum
{
    SPI_DATA_LITTLE_ENDIAN   = 0U, //!< LITTLE ENDIAN
    SPI_DATA_BIG_ENDIAN   = 1U,    //!< BIG ENDIAN
} SPI_endianess;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! \internal
//! Checks an SPI base address.
//!
//! \param base specifies the SPI module base address.
//!
//! This function determines if a SPI module base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
SPI_isBaseValid(uint32_t base)
{
    return(
           (base == SPIA_BASE) ||
           (base == SPIB_BASE) ||
           (base == SPIC_BASE) ||
           (base == SPID_BASE)
          );
}
#endif

//*****************************************************************************
//
//! Enables the serial peripheral interface.
//!
//! \param base specifies the SPI module base address.
//!
//! This function enables operation of the serial peripheral interface.  The
//! serial peripheral interface must be configured before it is enabled.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_enableModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    HWREGH(base + SPI_O_CCR) |= SPI_CCR_SPISWRESET;
}

//*****************************************************************************
//
//! Disables the serial peripheral interface.
//!
//! \param base specifies the SPI module base address.
//!
//! This function disables operation of the serial peripheral interface. Call
//! this function before doing any configuration.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_disableModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    HWREGH(base + SPI_O_CCR) &= ~(SPI_CCR_SPISWRESET);
}

//*****************************************************************************
//
//! Sets the character length of SPI transaction
//!
//! \param base specifies the SPI module base address.
//! \param charLength specifies the character length of SPI transaction
//!
//! This function configures the character length of SPI transaction.
//! SPI character length can be from anywhere between 1-bit word to 16 bit word
//! of character length
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_setcharLength(uint32_t base, uint16_t charLength)
{
    ASSERT((charLength >= 1U) && (charLength <= 16U));
    SPI_disableModule(base);
    HWREGH(base + SPI_O_CCR) = (HWREGH(base + SPI_O_CCR) & ~SPI_CCR_SPICHAR_M) |
                               (charLength - 1U);
    SPI_enableModule(base);
}


//*****************************************************************************
//
//! Enables the transmit and receive FIFOs.
//!
//! \param base is the base address of the SPI port.
//!
//! This functions enables the transmit and receive FIFOs in the SPI.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_enableFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Enable the FIFO.
    //
    HWREGH(base + SPI_O_FFTX) |= SPI_FFTX_SPIFFENA | SPI_FFTX_TXFIFO;
    HWREGH(base + SPI_O_FFRX) |= SPI_FFRX_RXFIFORESET;
}

//*****************************************************************************
//
//! Disables the transmit and receive FIFOs.
//!
//! \param base is the base address of the SPI port.
//!
//! This functions disables the transmit and receive FIFOs in the SPI.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_disableFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Disable the FIFO.
    //
    HWREGH(base + SPI_O_FFTX) &= ~(SPI_FFTX_SPIFFENA | SPI_FFTX_TXFIFO);
    HWREGH(base + SPI_O_FFRX) &= ~SPI_FFRX_RXFIFORESET;
}

//*****************************************************************************
//
//! Resets the transmit FIFO.
//!
//! \param base is the base address of the SPI port.
//!
//! This function resets the transmit FIFO, setting the FIFO pointer back to
//! zero.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_resetTxFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Reset the TX FIFO.
    //
    HWREGH(base + SPI_O_FFTX) &= ~SPI_FFTX_TXFIFO;
    HWREGH(base + SPI_O_FFTX) |= SPI_FFTX_TXFIFO;
}

//*****************************************************************************
//
//! Resets the receive FIFO.
//!
//! \param base is the base address of the SPI port.
//!
//! This function resets the receive FIFO, setting the FIFO pointer back to
//! zero.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_resetRxFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Reset the RX FIFO.
    //
    HWREGH(base + SPI_O_FFRX) &= ~SPI_FFRX_RXFIFORESET;
    HWREGH(base + SPI_O_FFRX) |= SPI_FFRX_RXFIFORESET;
}

//*****************************************************************************
//
//! Sets the FIFO level at which interrupts are generated.
//!
//! \param base is the base address of the SPI port.
//! \param txLevel is the transmit FIFO interrupt level, specified as
//! \b SPI_FIFO_TX0, \b SPI_FIFO_TX1, \b SPI_FIFO_TX2, . . . or
//! \b SPI_FIFO_TX16.
//! \param rxLevel is the receive FIFO interrupt level, specified as
//! \b SPI_FIFO_RX0, \b SPI_FIFO_RX1, \b SPI_FIFO_RX2, . . . or
//! \b SPI_FIFO_RX16.
//!
//! This function sets the FIFO level at which transmit and receive interrupts
//! are generated.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_setFIFOInterruptLevel(uint32_t base, SPI_TxFIFOLevel txLevel,
                          SPI_RxFIFOLevel rxLevel)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Set the FIFO interrupt levels.
    //
    HWREGH(base + SPI_O_FFTX) = (HWREGH(base + SPI_O_FFTX) &
                                 (~SPI_FFTX_TXFFIL_M)) | (uint16_t)txLevel;
    HWREGH(base + SPI_O_FFRX) = (HWREGH(base + SPI_O_FFRX) &
                                 (~SPI_FFRX_RXFFIL_M)) | (uint16_t)rxLevel;
}

//*****************************************************************************
//
//! Gets the FIFO level at which interrupts are generated.
//!
//! \param base is the base address of the SPI port.
//! \param txLevel is a pointer to storage for the transmit FIFO level,
//! returned as one of \b SPI_FIFO_TX0, \b SPI_FIFO_TX1,
//! \b SPI_FIFO_TX2, . . . or \b SPI_FIFO_TX16.
//! \param rxLevel is a pointer to storage for the receive FIFO level,
//! returned as one of \b SPI_FIFO_RX0, \b SPI_FIFO_RX1,
//! \b SPI_FIFO_RX2, . . . or \b SPI_FIFO_RX16.
//!
//! This function gets the FIFO level at which transmit and receive interrupts
//! are generated.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_getFIFOInterruptLevel(uint32_t base, SPI_TxFIFOLevel *txLevel,
                          SPI_RxFIFOLevel *rxLevel)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Extract the transmit and receive FIFO levels.
    //
    *txLevel = (SPI_TxFIFOLevel)(HWREGH(base + SPI_O_FFTX) &
                                 SPI_FFTX_TXFFIL_M);
    *rxLevel = (SPI_RxFIFOLevel)(HWREGH(base + SPI_O_FFRX) &
                                 SPI_FFRX_RXFFIL_M);
}

//*****************************************************************************
//
//! Get the transmit FIFO status
//!
//! \param base is the base address of the SPI port.
//!
//! This function gets the current number of words in the transmit FIFO.
//!
//! \return Returns the current number of words in the transmit FIFO specified
//! as one of the following:
//! \b SPI_FIFO_TX0, \b SPI_FIFO_TX1, \b SPI_FIFO_TX2, \b SPI_FIFO_TX3,
//! ..., or \b SPI_FIFO_TX16
//
//*****************************************************************************
static inline SPI_TxFIFOLevel
SPI_getTxFIFOStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Get the current FIFO status
    //
    return((SPI_TxFIFOLevel)((HWREGH(base + SPI_O_FFTX) & SPI_FFTX_TXFFST_M) >>
                             SPI_FFTX_TXFFST_S));
}

//*****************************************************************************
//
//! Get the receive FIFO status
//!
//! \param base is the base address of the SPI port.
//!
//! This function gets the current number of words in the receive FIFO.
//!
//! \return Returns the current number of words in the receive FIFO specified
//! as one of the following:
//! \b SPI_FIFO_RX0, \b SPI_FIFO_RX1, \b SPI_FIFO_RX2, \b SPI_FIFO_RX3,
//! ..., or \b SPI_FIFO_RX16
//
//*****************************************************************************
static inline SPI_RxFIFOLevel
SPI_getRxFIFOStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Get the current FIFO status
    //
    return((SPI_RxFIFOLevel)((HWREGH(base + SPI_O_FFRX) & SPI_FFRX_RXFFST_M) >>
                             SPI_FFRX_RXFFST_S));
}

//*****************************************************************************
//
//! Determines whether the SPI transmitter is busy or not.
//!
//! \param base is the base address of the SPI port.
//!
//! This function allows the caller to determine whether all transmitted bytes
//! have cleared the transmitter hardware.  If \b false is returned, then the
//! transmit FIFO is empty and all bits of the last transmitted word have left
//! the hardware shift register. This function is only valid when operating in
//! FIFO mode.
//!
//! \return Returns \b true if the SPI is transmitting or \b false if all
//! transmissions are complete.
//
//*****************************************************************************
static inline bool
SPI_isBusy(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Determine if the SPI is busy.
    //
    return((HWREGH(base + SPI_O_FFTX) & SPI_FFTX_TXFFST_M) != 0U);
}

//*****************************************************************************
//
//! Puts a data element into the SPI transmit buffer.
//!
//! \param base specifies the SPI module base address.
//! \param data is the left-justified data to be transmitted over SPI.
//!
//! This function places the supplied data into the transmit buffer of the
//! specified SPI module.
//!
//! \note The data being sent must be left-justified in \e data. The lower
//! 16 - N bits will be discarded where N is the data width selected in
//! SPI_setConfig(). For example, if configured for a 6-bit data width, the
//! lower 10 bits of data will be discarded.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_writeDataNonBlocking(uint32_t base, uint16_t data)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Write data to the transmit buffer.
    //
    HWREGH(base + SPI_O_TXBUF) = data;
}

//*****************************************************************************
//
//! Gets a data element from the SPI receive buffer.
//!
//! \param base specifies the SPI module base address.
//!
//! This function gets received data from the receive buffer of the specified
//! SPI module and returns it.
//!
//! \note Only the lower N bits of the value written to \e data contain valid
//! data, where N is the data width as configured by SPI_setConfig(). For
//! example, if the interface is configured for 8-bit data width, only the
//! lower 8 bits of the value written to \e data contain valid data.
//!
//! \return Returns the word of data read from the SPI receive buffer.
//
//*****************************************************************************
static inline uint16_t
SPI_readDataNonBlocking(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Check for data to read.
    //
    return(HWREGH(base + SPI_O_RXBUF));
}

//*****************************************************************************
//
//! Waits for space in the FIFO and then puts data into the transmit buffer.
//!
//! \param base specifies the SPI module base address.
//! \param data is the left-justified data to be transmitted over SPI.
//!
//! This function places the supplied data into the transmit buffer of the
//! specified SPI module once space is available in the transmit FIFO. This
//! function should only be used when the FIFO is enabled.
//!
//! \note The data being sent must be left-justified in \e data. The lower
//! 16 - N bits will be discarded where N is the data width selected in
//! SPI_setConfig(). For example, if configured for a 6-bit data width, the
//! lower 10 bits of data will be discarded.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_writeDataBlockingFIFO(uint32_t base, uint16_t data)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Wait until space is available in the receive FIFO.
    //
    while(SPI_getTxFIFOStatus(base) == SPI_FIFO_TXFULL)
    {
    }

    //
    // Write data to the transmit buffer.
    //
    HWREGH(base + SPI_O_TXBUF) = data;
}

//*****************************************************************************
//
//! Waits for data in the FIFO and then reads it from the receive buffer.
//!
//! \param base specifies the SPI module base address.
//!
//! This function waits until there is data in the receive FIFO and then reads
//! received data from the receive buffer.  This function should only be used
//! when FIFO mode is enabled.
//!
//! \note Only the lower N bits of the value written to \e data contain valid
//! data, where N is the data width as configured by SPI_setConfig(). For
//! example, if the interface is configured for 8-bit data width, only the
//! lower 8 bits of the value written to \e data contain valid data.
//!
//! \return Returns the word of data read from the SPI receive buffer.
//
//*****************************************************************************
static inline uint16_t
SPI_readDataBlockingFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Wait until data is available in the receive FIFO.
    //
    while(SPI_getRxFIFOStatus(base) == SPI_FIFO_RXEMPTY)
    {
    }

    //
    // Check for data to read.
    //
    return(HWREGH(base + SPI_O_RXBUF));
}

//*****************************************************************************
//
//! Waits for the transmit buffer to empty and then writes data to it.
//!
//! \param base specifies the SPI module base address.
//! \param data is the left-justified data to be transmitted over SPI.
//!
//! This function places the supplied data into the transmit buffer of the
//! specified SPI module once it is empty. This function should not be used
//! when FIFO mode is enabled.
//!
//! \note The data being sent must be left-justified in \e data. The lower
//! 16 - N bits will be discarded where N is the data width selected in
//! SPI_setConfig(). For example, if configured for a 6-bit data width, the
//! lower 10 bits of data will be discarded.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_writeDataBlockingNonFIFO(uint32_t base, uint16_t data)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Wait until the transmit buffer is not full.
    //
    while((HWREGH(base + SPI_O_STS) & SPI_STS_BUFFULL_FLAG) != 0U)
    {
    }

    //
    // Write data to the transmit buffer.
    //
    HWREGH(base + SPI_O_TXBUF) = data;
}

//*****************************************************************************
//
//! Waits for data to be received and then reads it from the buffer.
//!
//! \param base specifies the SPI module base address.
//!
//! This function waits for data to be received and then reads it from the
//! receive buffer of the specified SPI module. This function should not be
//! used when FIFO mode is enabled.
//!
//! \note Only the lower N bits of the value written to \e data contain valid
//! data, where N is the data width as configured by SPI_setConfig(). For
//! example, if the interface is configured for 8-bit data width, only the
//! lower 8 bits of the value written to \e data contain valid data.
//!
//! \return Returns the word of data read from the SPI receive buffer.
//
//*****************************************************************************
static inline uint16_t
SPI_readDataBlockingNonFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Wait until data has been received.
    //
    while((HWREGH(base + SPI_O_STS) & SPI_STS_INT_FLAG) == 0U)
    {
    }

    //
    // Check for data to read.
    //
    return(HWREGH(base + SPI_O_RXBUF));
}

//*****************************************************************************
//
//! Enables SPI 3-wire mode.
//!
//! \param base is the base address of the SPI port.
//!
//! This function enables 3-wire mode. When in controller mode, this allows
//! SPIPICO to become SPICOCI and SPIPOCI to become free for non-SPI use.
//! When in peripheral mode, SPIPOCI because the SPIPIPO pin and SPIPICO is
//! free for non-SPI use.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_enableTriWire(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Set the tri-wire bit to enable 3-wire mode.
    //
    HWREGH(base + SPI_O_PRI) |= SPI_PRI_TRIWIRE;
}

//*****************************************************************************
//
//! Disables SPI 3-wire mode.
//!
//! \param base is the base address of the SPI port.
//!
//! This function disables 3-wire mode. SPI will operate in normal 4-wire mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_disableTriWire(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Clear the tri-wire bit to disable 3-wire mode.
    //
    HWREGH(base + SPI_O_PRI) &= ~SPI_PRI_TRIWIRE;
}

//*****************************************************************************
//
//! Enables SPI loopback mode.
//!
//! \param base is the base address of the SPI port.
//!
//! This function enables loopback mode. This mode is only valid during
//! controller mode and is helpful during device testing as it internally
//! connects PICO and POCI.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_enableLoopback(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Set the bit that enables loopback mode.
    //
    HWREGH(base + SPI_O_CCR) |= SPI_CCR_SPILBK;
}

//*****************************************************************************
//
//! Disables SPI loopback mode.
//!
//! \param base is the base address of the SPI port.
//!
//! This function disables loopback mode. Loopback mode is disabled by default
//! after reset.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_disableLoopback(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Clear the bit that enables loopback mode.
    //
    HWREGH(base + SPI_O_CCR) &= ~SPI_CCR_SPILBK;
}

//*****************************************************************************
//
//! Set the peripheral select (SPIPTE) signal polarity.
//!
//! \param base is the base address of the SPI port.
//! \param polarity is the SPIPTE signal polarity.
//!
//! This function sets the polarity of the peripheral select (SPIPTE) signal.
//! The two modes to choose from for the \e polarity parameter are
//! \b SPI_PTE_ACTIVE_LOW for active-low polarity (typical) and
//! \b SPI_PTE_ACTIVE_HIGH for active-high polarity (considered inverted).
//!
//! \note This has no effect on the PTE signal when in controller mode. It is
//! only applicable to peripheral mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_setPTESignalPolarity(uint32_t base, SPI_PTEPolarity polarity)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Write the polarity of the SPIPTE signal to the register.
    //
    HWREGH(base + SPI_O_PRI) = (HWREGH(base + SPI_O_PRI) & ~SPI_PRI_PTEINV) |
                               (uint16_t)polarity;
}

//*****************************************************************************
//
//! Enables SPI high speed mode.
//!
//! \param base is the base address of the SPI port.
//!
//! This function enables high speed mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_enableHighSpeedMode(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Set the bit that enables high speed mode.
    //
    HWREGH(base + SPI_O_CCR) |= SPI_CCR_HS_MODE;
}

//*****************************************************************************
//
//! Disables SPI high speed mode.
//!
//! \param base is the base address of the SPI port.
//!
//! This function disables high speed mode. High speed mode is disabled by
//! default after reset.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_disableHighSpeedMode(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Clear the bit that enables high speed mode.
    //
    HWREGH(base + SPI_O_CCR) &= ~SPI_CCR_HS_MODE;
}

//*****************************************************************************
//
//! Sets SPI emulation mode.
//!
//! \param base is the base address of the SPI port.
//! \param mode is the emulation mode.
//!
//! This function sets the behavior of the SPI operation when an emulation
//! suspend occurs. The \e mode parameter can be one of the following:
//!
//! - \b SPI_EMULATION_STOP_MIDWAY - Transmission stops midway through the bit
//!   stream. The rest of the bits will be transmitting after the suspend is
//!   deasserted.
//! - \b SPI_EMULATION_STOP_AFTER_TRANSMIT - If the suspend occurs before the
//!   first SPICLK pulse, the transmission will not start. If it occurs later,
//!   the transmission will be completed.
//! - \b SPI_EMULATION_FREE_RUN - SPI operation continues regardless of a
//!   the suspend.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SPI_setEmulationMode(uint32_t base, SPI_EmulationMode mode)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Write the desired emulation mode to the register.
    //
    HWREGH(base + SPI_O_PRI) = (HWREGH(base + SPI_O_PRI) &
                                ~(SPI_PRI_FREE | SPI_PRI_SOFT)) |
                               (uint16_t)mode;
}

//*****************************************************************************
//
//! Configures the FIFO Transmit Delay
//!
//! \param base is the base address of the SPI port.
//! \param delay Tx FIFO delay to be configured in cycles (0..0xFF)
//!
//! This function sets the delay between every transfer from FIFO
//! transmit buffer to transmit shift register. The delay is defined in
//! number SPI serial clock cycles.
//!
//! \return None
//
//*****************************************************************************
static inline void
SPI_setTxFifoTransmitDelay(uint32_t base, uint16_t delay)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));
    ASSERT(delay <= 0xFFU);

    //
    // Configure the FIFO Transmit Delay Bits
    //
    HWREGH(base + SPI_O_FFCT) = delay;
}

//*****************************************************************************
//
//! Returns the Emulation Buffer Received Data
//!
//! \param base is the base address of the SPI port.
//!
//! This function returns the Emulation Buffer Received Data
//!
//! \return Rx emulation buffer data
//
//*****************************************************************************
static inline uint16_t
SPI_readRxEmulationBuffer(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Return Emulation Buffer Received Data
    //
    return(HWREGH(base + SPI_O_RXEMU));
}

//*****************************************************************************
//
//! Enable Trasnmit
//!
//! \param base is the base address of the SPI port.
//!
//! This function sets the TALK bit enabling the data trasnmission.
//! This bit is enabled by SPI_setConfig if the parameter \r mode is selected as
//! SPI_MODE_PERIPHERAL or SPI_MODE_CONTROLLER.
//!
//! \return None
//
//*****************************************************************************
static inline void
SPI_enableTalk(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Set the TALK bit
    //
    HWREGH(base + SPI_O_CTL) |= SPI_CTL_TALK;
}

//*****************************************************************************
//
//! Disable Trasnmit
//!
//! \param base is the base address of the SPI port.
//!
//! This function clears the TALK bit disabling the data trasnmission. The
//! output pin will be put in high-impedance state.
//! This bit is enabled by SPI_setConfig if the parameter \r mode is selected as
//! SPI_MODE_PERIPHERAL or SPI_MODE_CONTROLLER.
//!
//! \return None
//
//*****************************************************************************
static inline void
SPI_disableTalk(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Set the TALK bit
    //
    HWREGH(base + SPI_O_CTL) &= ~SPI_CTL_TALK;
}

//*****************************************************************************
//
//! Reset SPI transmit and receive channels
//!
//! \param base is the base address of the SPI port.
//!
//! This function resets the SPI transmit and receive channels.
//!
//! \return None
//
//*****************************************************************************
static inline void
SPI_reset(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Write to SPRST bit the TX FIFO.
    //
    HWREGH(base + SPI_O_FFTX) &= ~SPI_FFTX_SPIRST;
    HWREGH(base + SPI_O_FFTX) |= SPI_FFTX_SPIRST;
}

//*****************************************************************************
//
//! Configures the serial peripheral interface.
//!
//! \param base specifies the SPI module base address.
//! \param lspclkHz is the rate of the clock supplied to the SPI module
//! (LSPCLK) in Hz.
//! \param protocol specifies the data transfer protocol.
//! \param mode specifies the mode of operation.
//! \param bitRate specifies the clock rate in Hz.
//! \param dataWidth specifies number of bits transferred per frame.
//!
//! This function configures the serial peripheral interface.  It sets the SPI
//! protocol, mode of operation, bit rate, and data width.
//!
//! The \e protocol parameter defines the data frame format.  The \e protocol
//! parameter can be one of the following values: \b SPI_PROT_POL0PHA0,
//! \b SPI_PROT_POL0PHA1, \b SPI_PROT_POL1PHA0, or
//! \b SPI_PROT_POL1PHA1. These frame formats encode the following polarity
//! and phase configurations:
//!
//! <pre>
//! Polarity Phase       Mode
//!   0       0   SPI_PROT_POL0PHA0
//!   0       1   SPI_PROT_POL0PHA1
//!   1       0   SPI_PROT_POL1PHA0
//!   1       1   SPI_PROT_POL1PHA1
//! </pre>
//!
//! The \e mode parameter defines the operating mode of the SPI module.  The
//! SPI module can operate as a controller or peripheral; the SPI can also be be
//! configured to disable output on its serial output line.  The \e mode
//! parameter can be one of the following values: \b SPI_MODE_CONTROLLER,
//! \b SPI_MODE_PERIPHERAL, \b SPI_MODE_CONTROLLER_OD or
//! \b SPI_MODE_PERIPHERAL_OD ("OD" indicates "output disabled").
//!
//! The \e bitRate parameter defines the bit rate for the SPI.  This bit rate
//! must satisfy the following clock ratio criteria:
//!
//! - \e bitRate can be no greater than lspclkHz divided by 4.
//! - \e lspclkHz / \e bitRate cannot be greater than 128.
//!
//! The \e dataWidth parameter defines the width of the data transfers and
//! can be a value between 1 and 16, inclusive.
//!
//! The peripheral clock is the low speed peripheral clock.  This value is
//! returned by SysCtl_getLowSpeedClock(), or it can be explicitly hard coded
//! if it is constant and known (to save the code/execution overhead of a call
//! to SysCtl_getLowSpeedClock()).
//!
//! \note SPI operation should be disabled via SPI_disableModule() before any
//! changes to its configuration.
//!
//! \return None.
//
//*****************************************************************************
extern void
SPI_setConfig(uint32_t base, uint32_t lspclkHz, SPI_TransferProtocol protocol,
              SPI_Mode mode, uint32_t bitRate, uint16_t dataWidth);

//*****************************************************************************
//
//! Configures the baud rate of the serial peripheral interface.
//!
//! \param base specifies the SPI module base address.
//! \param lspclkHz is the rate of the clock supplied to the SPI module
//! (LSPCLK) in Hz.
//! \param bitRate specifies the clock rate in Hz.
//!
//! This function configures the SPI baud rate. The \e bitRate parameter
//! defines the bit rate for the SPI.  This bit rate must satisfy the following
//! clock ratio criteria:
//!
//! - \e bitRate can be no greater than \e lspclkHz divided by 4.
//! - \e lspclkHz / \e bitRate cannot be greater than 128.
//!
//! The peripheral clock is the low speed peripheral clock.  This value is
//! returned by SysCtl_getLowSpeedClock(), or it can be explicitly hard coded
//! if it is constant and known (to save the code/execution overhead of a call
//! to SysCtl_getLowSpeedClock()).
//!
//! \note SPI_setConfig() also sets the baud rate. Use SPI_setBaudRate()
//! if you wish to configure it separately from protocol and mode.
//!
//! \return None.
//
//*****************************************************************************
extern void
SPI_setBaudRate(uint32_t base, uint32_t lspclkHz, uint32_t bitRate);

//*****************************************************************************
//
//! Enables individual SPI interrupt sources.
//!
//! \param base specifies the SPI module base address.
//! \param intFlags is a bit mask of the interrupt sources to be enabled.
//!
//! This function enables the indicated SPI interrupt sources. Only the sources
//! that are enabled can be reflected to the processor interrupt; disabled
//! sources have no effect on the processor.  The \e intFlags parameter can be
//! any of the following values:
//! - \b SPI_INT_RX_OVERRUN - Receive overrun interrupt
//! - \b SPI_INT_RX_DATA_TX_EMPTY - Data received, transmit empty
//! - \b SPI_INT_RXFF (also enables \b SPI_INT_RXFF_OVERFLOW) - RX FIFO level
//!   interrupt (and RX FIFO overflow)
//! - \b SPI_INT_TXFF - TX FIFO level interrupt
//!
//! \note \b SPI_INT_RX_OVERRUN, \b SPI_INT_RX_DATA_TX_EMPTY,
//! \b SPI_INT_RXFF_OVERFLOW, and \b SPI_INT_RXFF are associated with
//! \b SPIRXINT; \b SPI_INT_TXFF is associated with \b SPITXINT.
//!
//! \return None.
//
//*****************************************************************************
extern void
SPI_enableInterrupt(uint32_t base, uint32_t intFlags);

//*****************************************************************************
//
//! Disables individual SPI interrupt sources.
//!
//! \param base specifies the SPI module base address.
//! \param intFlags is a bit mask of the interrupt sources to be disabled.
//!
//! This function disables the indicated SPI interrupt sources.  The
//! \e intFlags parameter can be any of the following values:
//! - \b SPI_INT_RX_OVERRUN
//! - \b SPI_INT_RX_DATA_TX_EMPTY
//! - \b SPI_INT_RXFF (also disables \b SPI_INT_RXFF_OVERFLOW)
//! - \b SPI_INT_TXFF
//!
//! \note \b SPI_INT_RX_OVERRUN, \b SPI_INT_RX_DATA_TX_EMPTY,
//! \b SPI_INT_RXFF_OVERFLOW, and \b SPI_INT_RXFF are associated with
//! \b SPIRXINT; \b SPI_INT_TXFF is associated with \b SPITXINT.
//!
//! \return None.
//
//*****************************************************************************
extern void
SPI_disableInterrupt(uint32_t base, uint32_t intFlags);

//*****************************************************************************
//
//! Gets the current interrupt status.
//!
//! \param base specifies the SPI module base address.
//!
//! This function returns the interrupt status for the SPI module.
//!
//! \return The current interrupt status, enumerated as a bit field of the
//! following values:
//! - \b SPI_INT_RX_OVERRUN - Receive overrun interrupt
//! - \b SPI_INT_RX_DATA_TX_EMPTY - Data received, transmit empty
//! - \b SPI_INT_RXFF - RX FIFO level interrupt
//! - \b SPI_INT_RXFF_OVERFLOW - RX FIFO overflow
//! - \b SPI_INT_TXFF - TX FIFO level interrupt
//
//*****************************************************************************
extern uint32_t
SPI_getInterruptStatus(uint32_t base);

//*****************************************************************************
//
//! Clears SPI interrupt sources.
//!
//! \param base specifies the SPI module base address.
//! \param intFlags is a bit mask of the interrupt sources to be cleared.
//!
//! This function clears the specified SPI interrupt sources so that they no
//! longer assert.  This function must be called in the interrupt handler to
//! keep the interrupts from being triggered again immediately upon exit.  The
//! \e intFlags parameter can consist of a bit field of the following values:
//! - \b SPI_INT_RX_OVERRUN
//! - \b SPI_INT_RX_DATA_TX_EMPTY
//! - \b SPI_INT_RXFF
//! - \b SPI_INT_RXFF_OVERFLOW
//! - \b SPI_INT_TXFF
//!
//! \note \b SPI_INT_RX_DATA_TX_EMPTY is cleared by a read of the receive
//! receive buffer, so it usually doesn't need to be cleared using this
//! function.
//!
//! \note Also note that \b SPI_INT_RX_OVERRUN, \b SPI_INT_RX_DATA_TX_EMPTY,
//! \b SPI_INT_RXFF_OVERFLOW, and \b SPI_INT_RXFF are associated with
//! \b SPIRXINT; \b SPI_INT_TXFF is associated with \b SPITXINT.
//!
//! \return None.
//
//*****************************************************************************
extern void
SPI_clearInterruptStatus(uint32_t base, uint32_t intFlags);


//*****************************************************************************
//
//! This function can be used to transmit a 24-bit word of data
//!
//! \param base specifies the SPI module base address.
//! \param txData is the data to be transmitted over SPI
//! \param txDelay specifies the number of serial clock cycles delay time after
//!        completion of perious word
//!
//! This function can be used to transmit a 24-bit word of data.
//! 24-bit word data is divided into three bytes of data.
//!
//! This function uses SPI_pollingFIFOTransaction function.
//! SPI character length is hardcoded to 8 (8bits) of character length
//!
//! \return None.
//
//*****************************************************************************
extern void
SPI_transmit24Bits(uint32_t base, uint32_t data, uint16_t txDelay);

//*****************************************************************************
//
//! This function can be used to transmit a 32-bit word of data
//!
//! \param base specifies the SPI module base address.
//! \param txData is the data to be transmitted over SPI
//! \param txDelay specifies the number of serial clock cycles delay time after
//!        completion of perious word
//!
//! This function can be used to transmit a 32-bit word of data.
//! 32-bit word data is divided into four bytes of data.
//!
//! This function uses SPI_pollingFIFOTransaction function.
//! SPI character length is hardcoded to 8 (8bits) of character length
//!
//! \return None.
//
//*****************************************************************************
extern void
SPI_transmit32Bits(uint32_t base, uint32_t data, uint16_t txDelay);



//*****************************************************************************
//
//! This function is used to receive a 16-bit word of data
//!
//! \param base specifies the SPI module base address.
//! \param endianness specifies the endianess of received data
//! \param dummyData is the data which is transmitted to initiate
//!        SPI transaction to receive SPI data
//! \param txDelay specifies the number of serial clock cycles delay time after
//!        completion of perious word
//!
//! This function is used to receive a 16-bit word of data.
//! This function uses SPIpolling_FIFO_Transaction function.
//! SPI character length is hardcoded to 8 (1 byte = 8 bits)of character length
//!
//! \return the received 16-bit word.
//
//*****************************************************************************
extern uint16_t
SPI_receive16Bits(uint32_t base, SPI_endianess endianness, uint16_t dummyData,
                  uint16_t txDelay);

//*****************************************************************************
//
//! This function is used to receive a 24-bit word of data
//!
//! \param base specifies the SPI module base address.
//! \param endianness specifies the endianess of received data
//! \param dummyData is the data which is transmitted to initiate
//!        SPI transaction to receive SPI data
//! \param txDelay specifies the number of serial clock cycles delay time after
//!        completion of perious word
//!
//! This function is used to receive a 24-bit word of data.
//! This function uses SPIpolling_FIFO_Transaction function.
//! SPI character length is hardcoded to 8 (1 byte = 8 bits)of character length
//!
//! \return the received 24-bit word.
//
//*****************************************************************************
extern uint32_t
SPI_receive24Bits(uint32_t base, SPI_endianess endianness, uint16_t dummyData,
                  uint16_t txDelay);

//*****************************************************************************
//
//! This function is used to receive a 32-bit word of data
//!
//! \param base specifies the SPI module base address.
//! \param endianness specifies the endianess of received data
//! \param dummyData is the data which is transmitted to initiate
//!        SPI transaction to receive SPI data
//! \param txDelay specifies the number of serial clock cycles delay time after
//!        completion of perious word
//!
//! This function is used to receive a 32-bit word of data.
//! This function uses SPIpolling_FIFO_Transaction function.
//! SPI character length is hardcoded to 8 (1 byte = 8 bits)of character length
//!
//! \return the received 32-bit word.
//
//*****************************************************************************
extern uint32_t
SPI_receive32Bits(uint32_t base, SPI_endianess endianness, uint16_t dummyData,
                  uint16_t txDelay);



//*****************************************************************************
//
//! This function is used to initiate SPI transaction of specified character
//! length
//!
//! \param base specifies the SPI module base address.
//! \param charLength specifies the SPI character length of SPI transaction
//! \param data specified the data to be transmitted
//!
//! This function is used to initiate SPI transaction of specified character.
//! SPI character length is configurable using charLength variable
//!
//! \return .
//
//*****************************************************************************
extern uint16_t
SPI_pollingNonFIFOTransaction(uint32_t base, uint16_t charLength,
                              uint16_t data);

//*****************************************************************************
//
//! This function is used to initiate SPI transaction of specified character
//! length and 'N' words of transaction
//!
//! \param base specifies the SPI module base address.
//! \param charLength specifies the SPI character length of SPI transaction
//! \param pTxBuffer specifies the pointer to transmit buffer
//! \param pRxBuffer specifies the pointer to receive buffer
//! \param numOfWords specified the number of data to be transmitted / received
//!
//! SPI character length is configurable using charLength variable
//!
//! \return none
//
//*****************************************************************************
extern void
SPI_pollingFIFOTransaction(uint32_t base, uint16_t charLength,
                           uint16_t *pTxBuffer, uint16_t *pRxBuffer,
                           uint16_t numOfWords, uint16_t txDelay);

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

#endif // SPI_H
