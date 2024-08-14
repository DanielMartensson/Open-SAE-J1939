//###########################################################################
//
// FILE:   i2c.h
//
// TITLE:  C28x I2C driver.
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

#ifndef I2C_H
#define I2C_H

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
//! \addtogroup i2c_api I2C
//! @{
//
//*****************************************************************************

#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "cpu.h"
#include "debug.h"
#include "hw_reg_inclusive_terminology.h"

//*****************************************************************************
//
// Defines for the API.
//
//*****************************************************************************

#ifndef DOXYGEN_PDF_IGNORE
//*****************************************************************************
//
// I2C Controller commands.
//
//*****************************************************************************
#define I2C_CONTROLLER_SEND_MODE    0x0600U //!< Controller-transmitter mode
#define I2C_CONTROLLER_RECEIVE_MODE 0x0400U //!< Controller-receiver mode
#define I2C_TARGET_SEND_MODE        0x0200U //!< Target-transmitter mode
#define I2C_TARGET_RECEIVE_MODE     0x0000U //!< Target-receiver mode

#define I2C_REPEAT_MODE         0x0080U //!< Only applies to Controller mode
#define I2C_START_BYTE_MODE     0x0010U //!< Enable start byte mode
#define I2C_FREE_DATA_FORMAT    0x0008U //!< Enable free data (no addr) format

//*****************************************************************************
//
// I2C interrupts for use with the intFlags parameter of I2C_enableInterrupt(),
// I2C_disableInterrupt(), and I2C_clearInterruptStatus() and to be returned by
// I2C_getInterruptStatus().
//
//*****************************************************************************
#define I2C_INT_ARB_LOST        0x00001U //!< Arbitration-lost interrupt
#define I2C_INT_NO_ACK          0x00002U //!< NACK interrupt
#define I2C_INT_REG_ACCESS_RDY  0x00004U //!< Register-access-ready interrupt
#define I2C_INT_RX_DATA_RDY     0x00008U //!< Receive-data-ready interrupt
#define I2C_INT_TX_DATA_RDY     0x00010U //!< Transmit-data-ready interrupt
#define I2C_INT_STOP_CONDITION  0x00020U //!< Stop condition detected
#define I2C_INT_ADDR_TARGET      0x00200U //!< Addressed as target interrupt
#define I2C_INT_RXFF            0x10000U //!< RX FIFO level interrupt
#define I2C_INT_TXFF            0x20000U //!< TX FIFO level interrupt

//
// Helpful define to mask out the bits in the I2CSTR register that aren't
// associated with interrupts.
//
#define I2C_STR_INTMASK     ((uint16_t)I2C_INT_ARB_LOST |                      \
                             (uint16_t)I2C_INT_NO_ACK |                        \
                             (uint16_t)I2C_INT_REG_ACCESS_RDY |                \
                             (uint16_t)I2C_INT_RX_DATA_RDY |                   \
                             (uint16_t)I2C_INT_TX_DATA_RDY |                   \
                             (uint16_t)I2C_INT_STOP_CONDITION |                \
                             (uint16_t)I2C_INT_ADDR_TARGET)

//*****************************************************************************
//
// Flags for use as the stsFlags parameter of I2C_clearStatus() and to be
// returned by I2C_getStatus().
//
//*****************************************************************************
#define I2C_STS_ARB_LOST        0x0001U //!< Arbitration-lost
#define I2C_STS_NO_ACK          0x0002U //!< No-acknowledgment (NACK)
#define I2C_STS_REG_ACCESS_RDY  0x0004U //!< Register-access-ready (ARDY)
#define I2C_STS_RX_DATA_RDY     0x0008U //!< Receive-data-ready
#define I2C_STS_TX_DATA_RDY     0x0010U //!< Transmit-data-ready
#define I2C_STS_STOP_CONDITION  0x0020U //!< Stop condition detected
#define I2C_STS_BYTE_SENT       0x0040U //!< Byte transmit complete
#define I2C_STS_ADDR_ZERO       0x0100U //!< Address of all zeros detected
#define I2C_STS_ADDR_TARGET     0x0200U //!< Addressed as target
#define I2C_STS_TX_EMPTY        0x0400U //!< Transmit shift register empty
#define I2C_STS_RX_FULL         0x0800U //!< Receive shift register full
#define I2C_STS_BUS_BUSY        0x1000U //!< Bus busy, wait for STOP or reset
#define I2C_STS_NACK_SENT       0x2000U //!< NACK was sent
#define I2C_STS_TARGET_DIR       0x4000U //!< Addressed as target transmitter
#endif

//*****************************************************************************
//
//! I2C interrupts to be returned by I2C_getInterruptSource().
//
//*****************************************************************************
typedef enum
{
    I2C_INTSRC_NONE,                //!< No interrupt pending
    I2C_INTSRC_ARB_LOST,            //!< Arbitration-lost interrupt
    I2C_INTSRC_NO_ACK,              //!< NACK interrupt
    I2C_INTSRC_REG_ACCESS_RDY,      //!< Register-access-ready interrupt
    I2C_INTSRC_RX_DATA_RDY,         //!< Receive-data-ready interrupt
    I2C_INTSRC_TX_DATA_RDY,         //!< Transmit-data-ready interrupt
    I2C_INTSRC_STOP_CONDITION,      //!< Stop condition detected
    I2C_INTSRC_ADDR_TARGET           //!< Addressed as target interrupt
} I2C_InterruptSource;

//*****************************************************************************
//
//! Values that can be passed to I2C_setFIFOInterruptLevel() as the \e txLevel
//! parameter, returned by I2C_getFIFOInterruptLevel() in the \e txLevel
//! parameter, and returned by I2C_getTxFIFOStatus().
//
//*****************************************************************************
typedef enum
{
    I2C_FIFO_TXEMPTY    = 0x0000U,      //!< Transmit FIFO empty
    I2C_FIFO_TX0        = 0x0000U,      //!< Transmit FIFO empty
    I2C_FIFO_TX1        = 0x0001U,      //!< Transmit FIFO 1/16 full
    I2C_FIFO_TX2        = 0x0002U,      //!< Transmit FIFO 2/16 full
    I2C_FIFO_TX3        = 0x0003U,      //!< Transmit FIFO 3/16 full
    I2C_FIFO_TX4        = 0x0004U,      //!< Transmit FIFO 4/16 full
    I2C_FIFO_TX5        = 0x0005U,      //!< Transmit FIFO 5/16 full
    I2C_FIFO_TX6        = 0x0006U,      //!< Transmit FIFO 6/16 full
    I2C_FIFO_TX7        = 0x0007U,      //!< Transmit FIFO 7/16 full
    I2C_FIFO_TX8        = 0x0008U,      //!< Transmit FIFO 8/16 full
    I2C_FIFO_TX9        = 0x0009U,      //!< Transmit FIFO 9/16 full
    I2C_FIFO_TX10       = 0x000AU,      //!< Transmit FIFO 10/16 full
    I2C_FIFO_TX11       = 0x000BU,      //!< Transmit FIFO 11/16 full
    I2C_FIFO_TX12       = 0x000CU,      //!< Transmit FIFO 12/16 full
    I2C_FIFO_TX13       = 0x000DU,      //!< Transmit FIFO 13/16 full
    I2C_FIFO_TX14       = 0x000EU,      //!< Transmit FIFO 14/16 full
    I2C_FIFO_TX15       = 0x000FU,      //!< Transmit FIFO 15/16 full
    I2C_FIFO_TX16       = 0x0010U,      //!< Transmit FIFO full
    I2C_FIFO_TXFULL     = 0x0010U       //!< Transmit FIFO full
} I2C_TxFIFOLevel;

//*****************************************************************************
//
//! Values that can be passed to I2C_setFIFOInterruptLevel() as the \e rxLevel
//! parameter, returned by I2C_getFIFOInterruptLevel() in the \e rxLevel
//! parameter, and returned by I2C_getRxFIFOStatus().
//
//*****************************************************************************
typedef enum
{
    I2C_FIFO_RXEMPTY    = 0x0000U,      //!< Receive FIFO empty
    I2C_FIFO_RX0        = 0x0000U,      //!< Receive FIFO empty
    I2C_FIFO_RX1        = 0x0001U,      //!< Receive FIFO 1/16 full
    I2C_FIFO_RX2        = 0x0002U,      //!< Receive FIFO 2/16 full
    I2C_FIFO_RX3        = 0x0003U,      //!< Receive FIFO 3/16 full
    I2C_FIFO_RX4        = 0x0004U,      //!< Receive FIFO 4/16 full
    I2C_FIFO_RX5        = 0x0005U,      //!< Receive FIFO 5/16 full
    I2C_FIFO_RX6        = 0x0006U,      //!< Receive FIFO 6/16 full
    I2C_FIFO_RX7        = 0x0007U,      //!< Receive FIFO 7/16 full
    I2C_FIFO_RX8        = 0x0008U,      //!< Receive FIFO 8/16 full
    I2C_FIFO_RX9        = 0x0009U,      //!< Receive FIFO 9/16 full
    I2C_FIFO_RX10       = 0x000AU,      //!< Receive FIFO 10/16 full
    I2C_FIFO_RX11       = 0x000BU,      //!< Receive FIFO 11/16 full
    I2C_FIFO_RX12       = 0x000CU,      //!< Receive FIFO 12/16 full
    I2C_FIFO_RX13       = 0x000DU,      //!< Receive FIFO 13/16 full
    I2C_FIFO_RX14       = 0x000EU,      //!< Receive FIFO 14/16 full
    I2C_FIFO_RX15       = 0x000FU,      //!< Receive FIFO 15/16 full
    I2C_FIFO_RX16       = 0x0010U,      //!< Receive FIFO full
    I2C_FIFO_RXFULL     = 0x0010U       //!< Receive FIFO full
} I2C_RxFIFOLevel;

//*****************************************************************************
//
//! Values that can be passed to I2C_setBitCount() as the \e size parameter.
//
//*****************************************************************************
typedef enum
{
    I2C_BITCOUNT_1  = 1U,   //!< 1 bit per data byte
    I2C_BITCOUNT_2  = 2U,   //!< 2 bits per data byte
    I2C_BITCOUNT_3  = 3U,   //!< 3 bits per data byte
    I2C_BITCOUNT_4  = 4U,   //!< 4 bits per data byte
    I2C_BITCOUNT_5  = 5U,   //!< 5 bits per data byte
    I2C_BITCOUNT_6  = 6U,   //!< 6 bits per data byte
    I2C_BITCOUNT_7  = 7U,   //!< 7 bits per data byte
    I2C_BITCOUNT_8  = 0U    //!< 8 bits per data byte
} I2C_BitCount;

//*****************************************************************************
//
//! Values that can be passed to I2C_setAddressMode() as the \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    I2C_ADDR_MODE_7BITS        = 0x0000U,   //!< 7-bit address
    I2C_ADDR_MODE_10BITS       = 0x0100U    //!< 10-bit address
} I2C_AddressMode;

//*****************************************************************************
//
//! Values that can be passed to I2C_setExtendedMode() as the \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    I2C_EADDR_MODE_BC  = 1U,   //!<  Backward compatibility
    I2C_EADDR_MODE_FWD = 2U    //!<  Forward compatibility
} I2C_ExtendedMode;

//*****************************************************************************
//
//! Values that can be passed to I2C_setEmulationMode() as the \e mode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! If SCL is low, keep it low. If high, stop when it goes low again.
    I2C_EMULATION_STOP_SCL_LOW = 0x0000U,
    //! Continue I2C operation regardless
    I2C_EMULATION_FREE_RUN     = 0x4000U
} I2C_EmulationMode;

//*****************************************************************************
//
//! Values that can be passed to I2C_initController() as the \e dutyCycle
//! parameter.
//
//*****************************************************************************
typedef enum
{
    I2C_DUTYCYCLE_33,       //!< Clock duty cycle is 33%
    I2C_DUTYCYCLE_50        //!< Clock duty cycle is 55%
} I2C_DutyCycle;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! \internal
//! Checks an I2C base address.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function determines if a I2C module base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
I2C_isBaseValid(uint32_t base)
{
    return(
           (base == I2CA_BASE) ||
           (base == I2CB_BASE)
          );
}
#endif

//*****************************************************************************
//
//! Enables the I2C module.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function enables operation of the I2C module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_enableModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    HWREGH(base + I2C_O_MDR) |= I2C_MDR_IRS;
}

//*****************************************************************************
//
//! Disables the I2C module.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function disables operation of the I2C module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_disableModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    HWREGH(base + I2C_O_MDR) &= ~(I2C_MDR_IRS);
}

//*****************************************************************************
//
//! Enables the transmit and receive FIFOs.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This functions enables the transmit and receive FIFOs in the I2C.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_enableFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Enable the FIFO.
    //
    HWREGH(base + I2C_O_FFTX) |= I2C_FFTX_I2CFFEN | I2C_FFTX_TXFFRST;
    HWREGH(base + I2C_O_FFRX) |= I2C_FFRX_RXFFRST;
}

//*****************************************************************************
//
//! Disables the transmit and receive FIFOs.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This functions disables the transmit and receive FIFOs in the I2C.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_disableFIFO(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Disable the FIFO.
    //
    HWREGH(base + I2C_O_FFTX) &= ~(I2C_FFTX_I2CFFEN | I2C_FFTX_TXFFRST);
    HWREGH(base + I2C_O_FFRX) &= ~I2C_FFRX_RXFFRST;
}

//*****************************************************************************
//
//! Sets the FIFO level at which interrupts are generated.
//!
//! \param base is the base address of the I2C instance used.
//! \param txLevel is the transmit FIFO interrupt level, specified as
//! \b I2C_FIFO_TX0, \b I2C_FIFO_TX1, \b I2C_FIFO_TX2, . . . or
//! \b I2C_FIFO_TX16.
//! \param rxLevel is the receive FIFO interrupt level, specified as
//! \b I2C_FIFO_RX0, \b I2C_FIFO_RX1, \b I2C_FIFO_RX2, . . . or
//! \b I2C_FIFO_RX16.
//!
//! This function sets the FIFO level at which transmit and receive interrupts
//! are generated.  The transmit FIFO interrupt flag will be set when the FIFO
//! reaches a value less than or equal to \e txLevel.  The receive FIFO
//! flag will be set when the FIFO reaches a value greater than or equal to
//! \e rxLevel.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_setFIFOInterruptLevel(uint32_t base, I2C_TxFIFOLevel txLevel,
                          I2C_RxFIFOLevel rxLevel)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Set the FIFO interrupt levels.
    //
    HWREGH(base + I2C_O_FFTX) = (HWREGH(base + I2C_O_FFTX) &
                                 (~I2C_FFTX_TXFFIL_M)) | (uint16_t)txLevel;
    HWREGH(base + I2C_O_FFRX) = (HWREGH(base + I2C_O_FFRX) &
                                 (~I2C_FFRX_RXFFIL_M)) | (uint16_t)rxLevel;
}

//*****************************************************************************
//
//! Gets the FIFO level at which interrupts are generated.
//!
//! \param base is the base address of the I2C instance used.
//! \param txLevel is a pointer to storage for the transmit FIFO level,
//! returned as one of \b I2C_FIFO_TX0, \b I2C_FIFO_TX1,
//! \b I2C_FIFO_TX2, . . . or \b I2C_FIFO_TX16.
//! \param rxLevel is a pointer to storage for the receive FIFO level,
//! returned as one of \b I2C_FIFO_RX0, \b I2C_FIFO_RX1,
//! \b I2C_FIFO_RX2, . . . or \b I2C_FIFO_RX16.
//!
//! This function gets the FIFO level at which transmit and receive interrupts
//! are generated.  The transmit FIFO interrupt flag will be set when the FIFO
//! reaches a value less than or equal to \e txLevel.  The receive FIFO
//! flag will be set when the FIFO reaches a value greater than or equal to
//! \e rxLevel.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_getFIFOInterruptLevel(uint32_t base, I2C_TxFIFOLevel *txLevel,
                          I2C_RxFIFOLevel *rxLevel)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Extract the transmit and receive FIFO levels.
    //
    *txLevel = (I2C_TxFIFOLevel)(HWREGH(base + I2C_O_FFTX) &
                                 I2C_FFTX_TXFFIL_M);
    *rxLevel = (I2C_RxFIFOLevel)(HWREGH(base + I2C_O_FFRX) &
                                 I2C_FFRX_RXFFIL_M);
}

//*****************************************************************************
//
//! Get the transmit FIFO status
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function gets the current number of words in the transmit FIFO.
//!
//! \return Returns the current number of words in the transmit FIFO specified
//! as one of the following:
//! \b I2C_FIFO_TX0, \b I2C_FIFO_TX1, \b I2C_FIFO_TX2, \b I2C_FIFO_TX3,
//! ..., or \b I2C_FIFO_TX16
//
//*****************************************************************************
static inline I2C_TxFIFOLevel
I2C_getTxFIFOStatus(uint32_t base)
{
    uint16_t level;

    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Get the current FIFO status
    //
    level = ((HWREGH(base + I2C_O_FFTX) & I2C_FFTX_TXFFST_M) >>
              I2C_FFTX_TXFFST_S);

    return((I2C_TxFIFOLevel)level);
}

//*****************************************************************************
//
//! Get the receive FIFO status
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function gets the current number of words in the receive FIFO.
//!
//! \return Returns the current number of words in the receive FIFO specified
//! as one of the following:
//! \b I2C_FIFO_RX0, \b I2C_FIFO_RX1, \b I2C_FIFO_RX2, \b I2C_FIFO_RX3,
//! ..., or \b I2C_FIFO_RX16
//
//*****************************************************************************
static inline I2C_RxFIFOLevel
I2C_getRxFIFOStatus(uint32_t base)
{
    uint16_t level;

    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Get the current FIFO status
    //
    level = ((HWREGH(base + I2C_O_FFRX) & I2C_FFRX_RXFFST_M) >>
              I2C_FFRX_RXFFST_S);

    return((I2C_RxFIFOLevel)level);
}

//*****************************************************************************
//
//! Reads I2C Module clock prescaler value.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function reads the I2C prescaler value which configures the I2C module
//! clock by dividing down the SYSCLK. I2C_MODULE_CLK = SYSCLK / (I2CPSC + )
//!
//! \return Returns the I2C prescaler(I2CPSC) cast as an uint16_t.
//
//*****************************************************************************
static inline uint16_t
I2C_getPreScaler(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Return the contents of the Prescaler register.
    //
    return(HWREGH(base + I2C_O_PSC));
}

//*****************************************************************************
//
//! Sets the address that the I2C Controller places on the bus.
//!
//! \param base is the base address of the I2C instance used.
//! \param targetAddr 7-bit or 10-bit target address
//!
//! This function configures the address that the I2C Controller places on the bus
//! when initiating a transaction.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_setTargetAddress(uint32_t base, uint16_t targetAddr)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));
    ASSERT(targetAddr <= I2C_TAR_TAR_M);

    HWREGH(base + I2C_O_TAR) = targetAddr;
}

//*****************************************************************************
//
//! Sets the own address for this I2C module.
//!
//! \param base is the base address of the I2C Target module.
//! \param Addr is the 7-bit or 10-bit address
//!
//! This function writes the specified address.
//!
//! The parameter \e Addr is the value that is compared against the
//! target address sent by an I2C controller.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_setOwnAddress(uint32_t base, uint16_t Addr)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));
    ASSERT(Addr <= I2C_OAR_OAR_M);

    HWREGH(base + I2C_O_OAR) = Addr;
}

//*****************************************************************************
//
//! Indicates whether or not the I2C bus is busy.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function returns an indication of whether or not the I2C bus is busy.
//! This function can be used in a multi-controller environment to determine if the
//! bus is free for another data transfer.
//!
//! \return Returns \b true if the I2C bus is busy; otherwise, returns
//! \b false.
//
//*****************************************************************************
static inline bool
I2C_isBusBusy(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    return((HWREGH(base + I2C_O_STR) & I2C_STR_BB) == I2C_STR_BB);
}

//*****************************************************************************
//
//! Gets the current I2C module status.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function returns the status for the I2C module.
//!
//! \return The current module status, enumerated as a bit field of
//! - \b I2C_STS_ARB_LOST - Arbitration-lost
//! - \b I2C_STS_NO_ACK - No-acknowledgment (NACK)
//! - \b I2C_STS_REG_ACCESS_RDY - Register-access-ready (ARDY)
//! - \b I2C_STS_RX_DATA_RDY - Receive-data-ready
//! - \b I2C_STS_TX_DATA_RDY - Transmit-data-ready
//! - \b I2C_STS_STOP_CONDITION - Stop condition detected
//! - \b I2C_STS_BYTE_SENT - Byte transmit complete
//! - \b I2C_STS_ADDR_ZERO - Address of all zeros detected
//! - \b I2C_STS_ADDR_TARGET - Addressed as Target
//! - \b I2C_STS_TX_EMPTY - Transmit shift register empty
//! - \b I2C_STS_RX_FULL - Receive shift register full
//! - \b I2C_STS_BUS_BUSY - Bus busy, wait for STOP or reset
//! - \b I2C_STS_NACK_SENT - NACK was sent
//! - \b I2C_STS_TARGET_DIR- Addressed as Target transmitter
//
//*****************************************************************************
static inline uint16_t
I2C_getStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Return contents of the status register
    //
    return(HWREGH(base + I2C_O_STR));
}

//*****************************************************************************
//
//! Clears I2C status flags.
//!
//! \param base is the base address of the I2C instance used.
//! \param stsFlags is a bit mask of the status flags to be cleared.
//!
//! This function clears the specified I2C status flags. The \e stsFlags
//! parameter is the logical OR of the following values:
//! - \b I2C_STS_ARB_LOST
//! - \b I2C_STS_NO_ACK,
//! - \b I2C_STS_REG_ACCESS_RDY
//! - \b I2C_STS_RX_DATA_RDY
//! - \b I2C_STS_STOP_CONDITION
//! - \b I2C_STS_BYTE_SENT
//! - \b I2C_STS_NACK_SENT
//! - \b I2C_STS_TARGET_DIR
//!
//! \note Note that some of the status flags returned by I2C_getStatus() cannot
//! be cleared by this function. Some may only be cleared by hardware or a
//! reset of the I2C module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_clearStatus(uint32_t base, uint16_t stsFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Write to the status registers to clear them.
    //
    HWREGH(base + I2C_O_STR) = stsFlags;
}

//*****************************************************************************
//
//! Controls the state of the I2C module.
//!
//! \param base is the base address of the I2C instance used.
//! \param config is the command to be issued to the I2C  module.
//!
//! This function is used to control the state of the controller and target send and
//! receive operations. The \e config is a logical OR of the following options.
//!
//! One of the following four options:
//! - \b I2C_CONTROLLER_SEND_MODE - Controller-transmitter mode
//! - \b I2C_CONTROLLER_RECEIVE_MODE - Controller-receiver mode
//! - \b I2C_TARGET_SEND_MODE - Target-transmitter mode
//! - \b I2C_TARGET_RECEIVE_MODE - Target-receiver mode
//!
//! Any of the following:
//! - \b I2C_REPEAT_MODE - Sends data until stop bit is set, ignores data count
//! - \b I2C_START_BYTE_MODE - Use start byte mode
//! - \b I2C_FREE_DATA_FORMAT - Use free data format, transfers have no address
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_setConfig(uint32_t base, uint16_t config)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Write the selected options to the mode register.
    //
    HWREGH(base + I2C_O_MDR) = (HWREGH(base + I2C_O_MDR) &
                                ~(I2C_MDR_CNT | I2C_MDR_TRX | I2C_MDR_RM |
                                  I2C_MDR_STB | I2C_MDR_FDF)) | config;
}

//*****************************************************************************
//
//! Sets the data byte bit count the I2C module.
//!
//! \param base is the base address of the I2C instance used.
//! \param size is the number of bits per data byte.
//!
//! The \e size parameter is a value I2C_BITCOUNT_x where x is the number of
//! bits per data byte.  The default and maximum size is 8 bits.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_setBitCount(uint32_t base, I2C_BitCount size)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Write the selected options to the mode register.
    //
    HWREGH(base + I2C_O_MDR) = (HWREGH(base + I2C_O_MDR) & ~I2C_MDR_BC_M) |
                               (uint16_t)size;
}

//*****************************************************************************
//
//! Issues an I2C START condition.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function causes the I2C module to generate a start condition. This
//! function is only valid when the I2C module specified by the \b base
//! parameter is a controller.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_sendStartCondition(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Set the START condition bit.
    //
    HWREGH(base + I2C_O_MDR) |= I2C_MDR_STT;
}

//*****************************************************************************
//
//! Issues an I2C STOP condition.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function causes the I2C module to generate a stop condition. This
//! function is only valid when the I2C module specified by the \b base
//! parameter is a controller.
//!
//! To check on the status of the STOP condition, I2C_getStopConditionStatus()
//! can be used.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_sendStopCondition(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Set the STOP condition bit.
    //
    HWREGH(base + I2C_O_MDR) |= I2C_MDR_STP;
}

//*****************************************************************************
//
//! Issues a no-acknowledge (NACK) bit.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function causes the I2C module to generate a NACK bit. This is only
//! applicable when the I2C module is acting as a receiver.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_sendNACK(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Set the NACK mode bit.
    //
    HWREGH(base + I2C_O_MDR) |= I2C_MDR_NACKMOD;
}

//*****************************************************************************
//
//! Receives a byte that has been sent to the I2C.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function reads a byte of data from the I2C Data Receive Register.
//!
//! \return Returns the byte received from by the I2C cast as an uint16_t.
//
//*****************************************************************************
static inline uint16_t
I2C_getData(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Return the contents of the receive register.
    //
    return(HWREGH(base + I2C_O_DRR));
}

//*****************************************************************************
//
//! Transmits a byte from the I2C.
//!
//! \param base is the base address of the I2C instance used.
//! \param data is the data to be transmitted from the I2C Controller.
//!
//! This function places the supplied data into I2C Data Transmit Register.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_putData(uint32_t base, uint16_t data)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Place the data into the transmit register.
    //
    HWREGH(base + I2C_O_DXR) = data;
}

//*****************************************************************************
//
//! Get stop condition status.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function reads and returns the stop condition bit status.
//!
//! \return Returns \b true if the STP bit has been set by the device to
//! generate a stop condition when the internal data counter of the I2C module
//! has reached 0. Returns \b false when the STP bit is zero. This bit is
//! automatically cleared after the stop condition has been generated.
//
//*****************************************************************************
static inline bool
I2C_getStopConditionStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Check the stop condition bit and return appropriately.
    //
    return((HWREGH(base + I2C_O_MDR) & I2C_MDR_STP) != 0U);
}

//*****************************************************************************
//
//! Set number of bytes to be to transfer or receive when repeat mode is off.
//!
//! \param base is the base address of the I2C instance used.
//! \param count is the value to be put in the I2C data count register.
//!
//! This function sets the number of bytes to transfer or receive when repeat
//! mode is off.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_setDataCount(uint32_t base, uint16_t count)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Write the count value to the appropriate register.
    //
    HWREGH(base + I2C_O_CNT) = count;
}

//*****************************************************************************
//
//! Sets the addressing mode to either 7-bit or 10-bit.
//!
//! \param base is the base address of the I2C instance used.
//! \param mode is the address mode, 7-bit or 10-bit.
//!
//! This function configures the I2C module for either a 7-bit address
//! (default) or a 10-bit address. The \e mode parameter configures the address
//! length to 10 bits when its value is \b I2C_ADDR_MODE_10BITS and 7 bits when
//! \b I2C_ADDR_MODE_7BITS.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_setAddressMode(uint32_t base, I2C_AddressMode mode)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Write the appropriate value to the address expansion bit.
    //
    HWREGH(base + I2C_O_MDR) = (HWREGH(base + I2C_O_MDR) & ~I2C_MDR_XA) |
                               (uint16_t)mode;
}

//*****************************************************************************
//
//! Sets I2C emulation mode.
//!
//! \param base is the base address of the I2C instance used.
//! \param mode is the emulation mode.
//!
//! This function sets the behavior of the I2C operation when an emulation
//! suspend occurs. The \e mode parameter can be one of the following:
//!
//! - \b I2C_EMULATION_STOP_SCL_LOW - If SCL is low when the breakpoint occurs,
//!   the I2C module stops immediately. If SCL is high, the I2C module waits
//!   until SCL becomes low and then stops.
//! - \b I2C_EMULATION_FREE_RUN - I2C operation continues regardless of a
//!   the suspend.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_setEmulationMode(uint32_t base, I2C_EmulationMode mode)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Write the desired emulation mode to the register.
    //
    HWREGH(base + I2C_O_MDR) = (HWREGH(base + I2C_O_MDR) & ~I2C_MDR_FREE) |
                               (uint16_t)mode;
}

//*****************************************************************************
//
//! Enables I2C loopback mode.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function enables loopback mode. This mode is only valid during controller
//! mode and is helpful during device testing as it causes data transmitted out
//! of the data transmit register to be received in data receive register.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_enableLoopback(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Set the bit that enables loopback mode.
    //
    HWREGH(base + I2C_O_MDR) |= I2C_MDR_DLB;
}

//*****************************************************************************
//
//! Disables I2C loopback mode.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function disables loopback mode. Loopback mode is disabled by default
//! after reset.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_disableLoopback(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Clear the bit that enables loopback mode.
    //
    HWREGH(base + I2C_O_MDR) &= ~I2C_MDR_DLB;
}

//*****************************************************************************
//
//! Returns the current I2C interrupt source.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function returns the event that generated an I2C basic (non-FIFO)
//! interrupt. The possible sources are the following:
//! - \b I2C_INTSRC_NONE
//! - \b I2C_INTSRC_ARB_LOST
//! - \b I2C_INTSRC_NO_ACK
//! - \b I2C_INTSRC_REG_ACCESS_RDY
//! - \b I2C_INTSRC_RX_DATA_RDY
//! - \b I2C_INTSRC_TX_DATA_RDY
//! - \b I2C_INTSRC_STOP_CONDITION
//! - \b I2C_INTSRC_ADDR_TARGET
//!
//! Calling this function will result in hardware automatically clearing the
//! current interrupt code and if ready, loading the next pending enabled
//! interrupt. It will also clear the corresponding interrupt flag if the
//! source is \b I2C_INTSRC_ARB_LOST, \b I2C_INTSRC_NO_ACK, or
//! \b I2C_INTSRC_STOP_CONDITION.
//!
//! \note Note that this function differs from I2C_getInterruptStatus() in that
//! it returns a single interrupt source. I2C_getInterruptSource() will return
//! the status of all interrupt flags possible, including the flags that aren't
//! necessarily enabled to generate interrupts.
//!
//! \return None.
//
//*****************************************************************************
static inline I2C_InterruptSource
I2C_getInterruptSource(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Return the interrupt source value
    //
    return((I2C_InterruptSource)(HWREGH(base + I2C_O_ISRC) &
                                 I2C_ISRC_INTCODE_M));
}

//*****************************************************************************
//
//! Sets the compatibility mode to support I2C Extended Mode.
//!
//! \param base is the base address of the I2C instance used.
//! \param mode is the compatibility modes.
//!
//! This function configures the I2C module for either a Forward or Backward
//! compatibilty. The \e mode parameter configures the compatibility
//! to Forward when its value is \b I2C_EADDR_MODE_FWD and Backward when
//! \b I2C_EADDR_MODE_BC.
//!
//! \return None.
//
//*****************************************************************************
static inline void
I2C_setExtendedMode(uint32_t base, I2C_ExtendedMode mode)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Write the appropriate value to the compatibility mode register.
    //
    HWREGH(base + I2C_O_EMDR) =
   (HWREGH(base + I2C_O_EMDR) & ~(I2C_EMDR_BC | I2C_EMDR_FCM)) |
   (uint16_t)mode;
}

//*****************************************************************************
//
//! Initializes the I2C Controller.
//!
//! \param base is the base address of the I2C instance used.
//! \param sysclkHz is the rate of the clock supplied to the I2C module
//! (SYSCLK) in Hz.
//! \param bitRate is the rate of the controller clock signal, SCL.
//! \param dutyCycle is duty cycle of the SCL signal.
//!
//! This function initializes operation of the I2C Controller by configuring the
//! bus speed for the controller. Note that the I2C module \b must be put into
//! reset before calling this function. You can do this with the function
//! I2C_disableModule().
//!
//! A programmable prescaler in the I2C module divides down the input clock
//! (rate specified by \e sysclkHz) to produce the module clock (calculated to
//! be around 10 MHz in this function). That clock is then divided down further
//! to configure the SCL signal to run at the rate specified by \e bitRate. The
//! \e dutyCycle parameter determines the percentage of time high and time low
//! on the clock signal. The valid values are \b I2C_DUTYCYCLE_33 for 33% and
//! \b I2C_DUTYCYCLE_50 for 50%.
//!
//! The peripheral clock is the system clock.  This value is returned by
//! SysCtl_getClock(), or it can be explicitly hard coded if it is
//! constant and known (to save the code/execution overhead of a call to
//! SysCtl_getClock()).
//!
//! \return None.
//
//*****************************************************************************
extern void
I2C_initController(uint32_t base, uint32_t sysclkHz, uint32_t bitRate,
               I2C_DutyCycle dutyCycle);

//*****************************************************************************
//
//! Enables I2C interrupt sources.
//!
//! \param base is the base address of the I2C instance used.
//! \param intFlags is the bit mask of the interrupt sources to be enabled.
//!
//! This function enables the indicated I2C Controller interrupt sources.  Only the
//! sources that are enabled can be reflected to the processor interrupt.
//! Disabled sources have no effect on the processor.
//!
//! The \e intFlags parameter is the logical OR of any of the following:
//!
//! - \b I2C_INT_ARB_LOST - Arbitration-lost interrupt
//! - \b I2C_INT_NO_ACK - No-acknowledgment (NACK) interrupt
//! - \b I2C_INT_REG_ACCESS_RDY - Register-access-ready interrupt
//! - \b I2C_INT_RX_DATA_RDY - Receive-data-ready interrupt
//! - \b I2C_INT_TX_DATA_RDY - Transmit-data-ready interrupt
//! - \b I2C_INT_STOP_CONDITION - Stop condition detected
//! - \b I2C_INT_ADDR_TARGET - Addressed as target interrupt
//! - \b I2C_INT_RXFF - RX FIFO level interrupt
//! - \b I2C_INT_TXFF - TX FIFO level interrupt
//!
//! \note \b I2C_INT_RXFF and \b I2C_INT_TXFF are associated with the I2C FIFO
//! interrupt vector. All others are associated with the I2C basic interrupt.
//!
//! \return None.
//
//*****************************************************************************
extern void
I2C_enableInterrupt(uint32_t base, uint32_t intFlags);

//*****************************************************************************
//
//! Disables I2C interrupt sources.
//!
//! \param base is the base address of the I2C instance used.
//! \param intFlags is the bit mask of the interrupt sources to be disabled.
//!
//! This function disables the indicated I2C Target interrupt sources.  Only
//! the sources that are enabled can be reflected to the processor interrupt.
//! Disabled sources have no effect on the processor.
//!
//! The \e intFlags parameter has the same definition as the \e intFlags
//! parameter to I2C_enableInterrupt().
//!
//! \return None.
//
//*****************************************************************************
extern void
I2C_disableInterrupt(uint32_t base, uint32_t intFlags);

//*****************************************************************************
//
//! Gets the current I2C interrupt status.
//!
//! \param base is the base address of the I2C instance used.
//!
//! This function returns the interrupt status for the I2C module.
//!
//! \return The current interrupt status, enumerated as a bit field of
//! - \b I2C_INT_ARB_LOST
//! - \b I2C_INT_NO_ACK
//! - \b I2C_INT_REG_ACCESS_RDY
//! - \b I2C_INT_RX_DATA_RDY
//! - \b I2C_INT_TX_DATA_RDY
//! - \b I2C_INT_STOP_CONDITION
//! - \b I2C_INT_ADDR_TARGET
//! - \b I2C_INT_RXFF
//! - \b I2C_INT_TXFF
//!
//! \note This function will only return the status flags associated with
//! interrupts. However, a flag may be set even if its corresponding interrupt
//! is disabled.
//
//*****************************************************************************
extern uint32_t
I2C_getInterruptStatus(uint32_t base);

//*****************************************************************************
//
//! Clears I2C interrupt sources.
//!
//! \param base is the base address of the I2C instance used.
//! \param intFlags is a bit mask of the interrupt sources to be cleared.
//!
//! The specified I2C interrupt sources are cleared, so that they no longer
//! assert.  This function must be called in the interrupt handler to keep the
//! interrupt from being triggered again immediately upon exit.
//!
//! The \e intFlags parameter has the same definition as the \e intFlags
//! parameter to I2C_enableInterrupt().
//!
//! \note \b I2C_INT_RXFF and \b I2C_INT_TXFF are associated with the I2C FIFO
//! interrupt vector. All others are associated with the I2C basic interrupt.
//!
//! \note Also note that some of the status flags returned by
//! I2C_getInterruptStatus() cannot be cleared by this function. Some may only
//! be cleared by hardware or a reset of the I2C module.
//!
//! \return None.
//
//*****************************************************************************
extern void
I2C_clearInterruptStatus(uint32_t base, uint32_t intFlags);

//*****************************************************************************
//
//! Configures I2C Module Clock Frequency
//!
//! \param base is the base address of the I2C instance used.
//! \param sysclkHz is the rate of the clock supplied to the I2C module
//! (SYSCLK) in Hz.
//!
//! This function configures I2C module clock frequency by initializing
//! prescale register based on SYSCLK frequency.
//! Note that the I2C module \b must be put into
//! reset before calling this function. You can do this with the function
//! I2C_disableModule().
//!
//! \return None.
//
//*****************************************************************************
extern void
I2C_configureModuleFrequency(uint32_t base, uint32_t sysclkHz);

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

#endif // I2C_H
