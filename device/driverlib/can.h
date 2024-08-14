//###########################################################################
//
// FILE:   can.h
//
// TITLE:  C28x CAN driver.
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
#ifndef CAN_H
#define CAN_H


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

#ifdef __TMS320C28XX__

//*****************************************************************************
//
//! \addtogroup can_api CAN
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "debug.h"
#include "sysctl.h"

//
// The key value for RAM initialization
//
#define CAN_RAM_INIT_KEY           (0xAU)

//
// RAM Initialization Register Mask
//
#define CAN_RAM_INIT_MASK          (0x003FU)

//
// The Parity disable key value
//
#define CAN_INIT_PARITY_DISABLE    (0x1400U)

#ifndef DOXYGEN_PDF_IGNORE
//*****************************************************************************
//
// Miscellaneous defines for Message ID Types
//
//*****************************************************************************
//*****************************************************************************
//
// These are the flags used by the flags parameter when calling
// the CAN_setupMessageObject() function.
//
//*****************************************************************************

//! This indicates that transmit interrupts should be enabled, or are enabled.
#define CAN_MSG_OBJ_TX_INT_ENABLE          CAN_IF1MCTL_TXIE

//! This indicates that receive interrupts should be enabled, or are
//! enabled.
#define CAN_MSG_OBJ_RX_INT_ENABLE          CAN_IF1MCTL_RXIE

//! This indicates that a message object will use or is using filtering
//! based on the object's message identifier.
#define CAN_MSG_OBJ_USE_ID_FILTER          (0x00000001U)

//! This indicates that a message object will use or is using filtering
//! based on the direction of the transfer.
#define CAN_MSG_OBJ_USE_DIR_FILTER         CAN_IF1MSK_MDIR

//! This indicates that a message object will use or is using message
//! identifier filtering based on the extended identifier.
#define CAN_MSG_OBJ_USE_EXT_FILTER         CAN_IF1MSK_MXTD

//! This indicates that this message object is part of a FIFO structure and
//! not the final message object in a FIFO.
#define CAN_MSG_OBJ_FIFO                   (0x00000002U)

//! This indicates that a message object has no flags set.
#define CAN_MSG_OBJ_NO_FLAGS               (0x00000000U)

//*****************************************************************************
//
// These definitions are used to specify interrupt sources to
// CAN_enableInterrupt() and CAN_disableInterrupt().
//
//*****************************************************************************
//! This flag is used to allow a CAN controller to generate error
//! interrupts.
#define CAN_INT_ERROR              (0x00000008UL)

//! This flag is used to allow a CAN controller to generate status
//! interrupts.
#define CAN_INT_STATUS             (0x00000004UL)

//! This flag is used to allow a CAN controller to generate interrupts
//! on interrupt line 0
#define CAN_INT_IE0                (0x00000002UL)

//! This flag is used to allow a CAN controller to generate interrupts
//! on interrupt line 1
#define CAN_INT_IE1                (0x00020000UL)

//*****************************************************************************
//
// The following definitions contain all error or status indicators that can
// be returned when calling the CAN_getStatus() function.
//
//*****************************************************************************

//! CAN controller has detected a parity error.
#define CAN_STATUS_PERR                  (0x00000100U)

//! CAN controller has entered a Bus Off state.
#define CAN_STATUS_BUS_OFF               (0x00000080U)

//! CAN controller error level has reached warning level.
#define CAN_STATUS_EWARN                 (0x00000040U)

//! CAN controller error level has reached error passive level.
#define CAN_STATUS_EPASS                 (0x00000020U)

//! A message was received successfully since the last read of this status.
#define CAN_STATUS_RXOK                  (0x00000010U)

//! A message was transmitted successfully since the last read of this
//! status.
#define CAN_STATUS_TXOK                  (0x00000008U)

//! This is the mask for the last error code field.
#define CAN_STATUS_LEC_MSK               (0x00000007U)

//! There was no error.
#define CAN_STATUS_LEC_NONE              (0x00000000U)

//! A bit stuffing error has occurred.
#define CAN_STATUS_LEC_STUFF             (0x00000001U)

//! A formatting error has occurred.
#define CAN_STATUS_LEC_FORM              (0x00000002U)

//! An acknowledge error has occurred.
#define CAN_STATUS_LEC_ACK               (0x00000003U)

//! The bus remained a bit level of 1 for longer than is allowed.
#define CAN_STATUS_LEC_BIT1              (0x00000004U)

//! The bus remained a bit level of 0 for longer than is allowed.
#define CAN_STATUS_LEC_BIT0              (0x00000005U)

//! A CRC error has occurred.
#define CAN_STATUS_LEC_CRC               (0x00000006U)

//*****************************************************************************
//
// The following macros are added for the Global Interrupt EN/FLG/CLR
// register
//
//*****************************************************************************
//! CANINT0 global interrupt bit
#define CAN_GLOBAL_INT_CANINT0          (0x00000001U)

//! CANINT1 global interrupt bit
#define CAN_GLOBAL_INT_CANINT1          (0x00000002U)

//*****************************************************************************
//
// The following macros are added for accessing the interrupt register and
// the standard arbitration ID in the interface registers.
//
//*****************************************************************************
//! Status of INT0ID
#define CAN_INT_INT0ID_STATUS           (0x8000U)

//! IF1 Arbitration Standard ID Shift Offset
#define CAN_IF1ARB_STD_ID_S             (18U)

//! IF1 Arbitration Standard ID Mask
#define CAN_IF1ARB_STD_ID_M             (0x1FFC0000U)

//! IF2 Arbitration Standard ID Shift Offset
#define CAN_IF2ARB_STD_ID_S             (18U)

//! IF2 Arbitration Standard ID Mask
#define CAN_IF2ARB_STD_ID_M             (0x1FFC0000U)

#endif // DOXYGEN_PDF_IGNORE

//*****************************************************************************
//
//! This data type is used to decide between STD_ID or EXT_ID for a mailbox.
//! This is used when calling the CAN_setupMessageObject() function.
//
//*****************************************************************************
typedef enum
{
    //! Set the message ID frame to standard.
    CAN_MSG_FRAME_STD,

    //! Set the message ID frame to extended.
    CAN_MSG_FRAME_EXT
} CAN_MsgFrameType;

//*****************************************************************************
//
//! This definition is used to determine the type of message object that will
//! be set up via a call to the CAN_setupMessageObject() API.
//
//*****************************************************************************
typedef enum
{
    //! Transmit message object.
    CAN_MSG_OBJ_TYPE_TX,

    //! Transmit remote request message object
    CAN_MSG_OBJ_TYPE_TX_REMOTE,

    //! Receive message object.
    CAN_MSG_OBJ_TYPE_RX,

    //! Remote frame receive remote, with auto-transmit message object.
    CAN_MSG_OBJ_TYPE_RXTX_REMOTE
} CAN_MsgObjType;

//*****************************************************************************
//
//! This definition is used to determine the clock source that will
//! be set up via a call to the CAN_selectClockSource() API.
//
//*****************************************************************************
typedef enum
{
    //! Peripheral System Clock Source
    CAN_CLOCK_SOURCE_SYS    = 0x0,

    //! External Oscillator Clock Source
    CAN_CLOCK_SOURCE_XTAL   = 0x1,

    //! Auxiliary Clock Input Source
    CAN_CLOCK_SOURCE_AUX    = 0x2
} CAN_ClockSource;


//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! \internal
//!
//! Checks a CAN base address.
//!
//! \param base is the base address of the CAN controller.
//!
//! This function determines if a CAN controller base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
CAN_isBaseValid(uint32_t base)
{
    return(
           (base == CANA_BASE) ||
           (base == CANB_BASE)
          );
}
#endif


//*****************************************************************************
//
//! \internal
//!
//! Copies data from a buffer to the CAN Data registers.
//!
//! \param data is a pointer to the data to be written out to the CAN
//! controller's data registers.
//! \param address is a uint32_t value for the first register of the
//! CAN controller's data registers.  For example, in order to use the IF1
//! register set on CAN controller 0, the value would be: \b CANA_BASE \b +
//! \b CAN_O_IF1DATA.
//! \param size is the number of bytes to copy into the CAN controller.
//!
//! This function takes the steps necessary to copy data from a contiguous
//! buffer in memory into the non-contiguous data registers used by the CAN
//! controller.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_writeDataReg(const uint16_t *const data, uint32_t address,
                 uint32_t size)
{
    uint32_t idx;
    uint32_t dataReg = address;

    //
    // Check the dataReg.
    //
    ASSERT(dataReg != 0U);

    //
    // Loop always copies 1 byte per iteration.
    //
    for(idx = 0U; idx < size; idx++)
    {
        //
        // Write out the data 8 bits at a time.
        //
        HWREGB(dataReg) = data[idx];

        dataReg++;
    }
}

//*****************************************************************************
//
//! \internal
//!
//! Copies data (all 16bits) from a buffer to the CAN Data registers.
//!
//! \param data is a pointer to the data to be written out to the CAN
//! controller's data registers.
//! \param address is a uint32_t value for the first register of the
//! CAN controller's data registers.  For example, in order to use the IF1
//! register set on CAN controller 0, the value would be: \b CANA_BASE \b +
//! \b CAN_O_IF1DATA.
//! \param size is the number of bytes to copy into the CAN controller.
//!
//! This function takes the steps necessary to copy data from a contiguous
//! buffer in memory into the non-contiguous data registers used by the CAN
//! controller.
//!
//! \return None.
//
//*****************************************************************************

static inline void
CAN_writeDataReg_16bit(const uint16_t *const data, uint32_t address,
                 uint32_t size)
{
    uint32_t idx;
    uint32_t dataReg = address;

    //
    // Check the dataReg.
    //
    ASSERT(dataReg != 0U);

    //
    // Loop always copies 1 byte per iteration.
    //
    for(idx = 0U; idx < size; idx++)
    {
        //
        // Write out the data 8 bits at a time.
        //
        HWREGB(dataReg) = (uint32_t)((data[idx / 2]) >> ((idx % 2UL) * 8UL));

        dataReg++;
    }
}


//*****************************************************************************
//
//! \internal
//!
//! Copies data (all 32bits) from a buffer to the CAN Data registers.
//!
//! \param data is a pointer to the data to be written out to the CAN
//! controller's data registers.
//! \param address is a uint32_t value for the first register of the
//! CAN controller's data registers.  For example, in order to use the IF1
//! register set on CAN controller 0, the value would be: \b CANA_BASE \b +
//! \b CAN_O_IF1DATA.
//! \param size is the number of bytes to copy into the CAN controller.
//!
//! This function takes the steps necessary to copy data from a contiguous
//! buffer in memory into the non-contiguous data registers used by the CAN
//! controller.
//!
//! \return None.
//
//*****************************************************************************

static inline void
CAN_writeDataReg_32bit(const uint32_t *const data, uint32_t address,
                 uint32_t size)
{
    uint32_t idx;
    uint32_t dataReg = address;

    //
    // Check the dataReg.
    //
    ASSERT(dataReg != 0U);

    //
    // Loop always copies 1 byte per iteration.
    //
    for(idx = 0U; idx < size; idx++)
    {
        //
        // Write out the data 8 bits at a time.
        //
        HWREGB(dataReg) = ((data[idx / 4]) >> ((idx % 4UL) * 8UL));

        dataReg++;
    }
}

//*****************************************************************************
//
//! \internal
//!
//! Copies data from the CAN Data registers to a buffer.
//!
//! \param data is a pointer to the location to store the data read from the
//! CAN controller's data registers.
//! \param address is a uint32_t value for the first register of the
//! CAN controller's data registers.  For example, in order to use the IF1
//! register set on CAN controller 1, the value would be: \b CANA_BASE \b +
//! \b CAN_O_IF1DATA.
//! \param size is the number of bytes to copy from the CAN controller.
//!
//! This function takes the steps necessary to copy data to a contiguous buffer
//! in memory from the non-contiguous data registers used by the CAN
//! controller.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_readDataReg(uint16_t *data, const uint32_t address, uint32_t size)
{
    uint32_t idx;
    uint32_t dataReg = address;

    //
    // Loop always copies 1 byte per iteration.
    //
    for(idx = 0U; idx < size; idx++)
    {
        //
        // Read out the data
        //
        data[idx] = HWREGB(dataReg);

        dataReg++;
    }
}

//*****************************************************************************
//
//! Initializes the CAN controller's RAM.
//!
//! \param base is the base address of the CAN controller.
//!
//! Performs the initialization of the RAM used for the CAN message objects.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_initRAM(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    HWREGH(base + CAN_O_RAM_INIT) = CAN_RAM_INIT_CAN_RAM_INIT |
                                    CAN_RAM_INIT_KEY;

    while(!((HWREGH(base + CAN_O_RAM_INIT) & CAN_RAM_INIT_MASK) ==
            (CAN_RAM_INIT_RAM_INIT_DONE | CAN_RAM_INIT_KEY2 |
             CAN_RAM_INIT_KEY0)))
    {
        //
        // Wait until RAM Init is complete
        //
    }
}

//*****************************************************************************
//
//! Select CAN Clock Source
//!
//! \param base is the base address of the CAN controller.
//! \param source is the clock source to use for the CAN controller.
//!
//! This function selects the specified clock source for the CAN controller.
//!
//! The \e source parameter can be any one of the following:
//! - \b CAN_CLOCK_SOURCE_SYS  - Peripheral System Clock
//! - \b CAN_CLOCK_SOURCE_XTAL - External Oscillator
//! - \b CAN_CLOCK_SOURCE_AUX  - Auxiliary Clock Input from GPIO
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_selectClockSource(uint32_t base, CAN_ClockSource source)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Determine the CAN controller and set specified clock source
    //
    EALLOW;

    switch(base)
    {
        case CANA_BASE:
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) &=
                ~SYSCTL_CLKSRCCTL2_CANABCLKSEL_M;

            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) |= ((uint16_t)source <<
                SYSCTL_CLKSRCCTL2_CANABCLKSEL_S);
            break;

        case CANB_BASE:
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) &=
                ~SYSCTL_CLKSRCCTL2_CANBBCLKSEL_M;

            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) |= ((uint16_t)source <<
                SYSCTL_CLKSRCCTL2_CANBBCLKSEL_S);
            break;

        default:

            //
            // Do nothing. Not a valid mode value.
            //
            break;
    }

    EDIS;
}

//*****************************************************************************
//
//! Starts the CAN Module's Operations
//!
//! \param base is the base address of the CAN controller.
//!
//! This function starts the CAN module's operations after initialization,
//! which includes the CAN protocol controller state machine of the CAN core
//! and the message handler state machine to begin controlling the CAN's
//! internal data flow.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_startModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Clear Init and CCE bits
    //
    HWREGH(base + CAN_O_CTL) &= ~(CAN_CTL_INIT | CAN_CTL_CCE);
}

//*****************************************************************************
//
//! Enables the CAN controller.
//!
//! \param base is the base address of the CAN controller to enable.
//!
//! Enables the CAN controller for message processing.  Once enabled, the
//! controller will automatically transmit any pending frames, and process any
//! received frames.  The controller can be stopped by calling
//! CAN_disableController().
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_enableController(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Clear the init bit in the control register.
    //
    HWREGH(base + CAN_O_CTL) &= ~CAN_CTL_INIT;
}

//*****************************************************************************
//
//! Disables the CAN controller.
//!
//! \param base is the base address of the CAN controller to disable.
//!
//! Disables the CAN controller for message processing.  When disabled, the
//! controller will no longer automatically process data on the CAN bus.  The
//! controller can be restarted by calling CAN_enableController().  The state
//! of the CAN controller and the message objects in the controller are left as
//! they were before this call was made.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_disableController(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Set the init bit in the control register.
    //
    HWREGH(base + CAN_O_CTL) |= CAN_CTL_INIT;
}

//*****************************************************************************
//
//! Enables the test modes of the CAN controller.
//!
//! \param base is the base address of the CAN controller.
//! \param mode are the the test modes to enable.
//!
//! Enables test modes within the controller.  The following valid options for
//! \e mode can be OR'ed together:
//! - \b CAN_TEST_SILENT - Silent Mode
//! - \b CAN_TEST_LBACK  - Loopback Mode
//! - \b CAN_TEST_EXL    - External Loopback Mode
//!
//! \note Loopback mode and external loopback mode \b can \b not be
//! enabled at the same time.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_enableTestMode(uint32_t base, uint16_t mode)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((mode & (CAN_TEST_LBACK | CAN_TEST_EXL)) !=
           (CAN_TEST_LBACK | CAN_TEST_EXL));

    //
    // Clear the bits in the test register.
    //
    HWREGH(base + CAN_O_TEST) &= ~((uint16_t)CAN_TEST_SILENT |
                                   (uint16_t)CAN_TEST_LBACK |
                                   (uint16_t)CAN_TEST_EXL);

    //
    // Enable test mode and set the bits in the test register.
    //
    HWREGH(base + CAN_O_CTL) |= CAN_CTL_TEST;
    HWREGH(base + CAN_O_TEST) |= mode;
}

//*****************************************************************************
//
//! Disables the test modes of the CAN controller.
//!
//! \param base is the base address of the CAN controller.
//!
//! Disables test modes within the controller and clears the test bits.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_disableTestMode(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Clear the bits in the test register.
    //
    HWREGH(base + CAN_O_TEST) &= ~((uint16_t)CAN_TEST_SILENT |
                                   (uint16_t)CAN_TEST_LBACK |
                                   (uint16_t)CAN_TEST_EXL);

    //
    // Clear the test mode enable bit
    //
    HWREGH(base + CAN_O_CTL) &= ~CAN_CTL_TEST;
}

//*****************************************************************************
//
//! Get the current settings for the CAN controller bit timing.
//!
//! \param base is the base address of the CAN controller.
//!
//! This function reads the current configuration of the CAN controller bit
//! clock timing.
//!
//! \return Returns the value of the bit timing register.
//
//*****************************************************************************
static inline uint32_t
CAN_getBitTiming(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Read and return BTR register
    //
    return(HWREG_BP(base + CAN_O_BTR));
}

//*****************************************************************************
//
//! Enables direct access to the RAM.
//!
//! \param base is the base address of the CAN controller.
//!
//! Enables direct access to the RAM while in Test mode.
//!
//! \note Test Mode must first be enabled to use this function.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_enableMemoryAccessMode(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Set the RAM direct access bit
    //
    HWREGH(base + CAN_O_TEST) |= CAN_TEST_RDA;
}

//*****************************************************************************
//
//! Disables direct access to the RAM.
//!
//! \param base is the base address of the CAN controller.
//!
//! Disables direct access to the RAM while in Test mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_disableMemoryAccessMode(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Clear the RAM direct access bit
    //
    HWREGH(base + CAN_O_TEST) &= ~CAN_TEST_RDA;
}

//*****************************************************************************
//
//! Sets the interruption debug mode of the CAN controller.
//!
//! \param base is the base address of the CAN controller.
//! \param enable is a flag to enable or disable the interruption debug mode.
//!
//! This function sets the interruption debug mode of the CAN controller. When
//! the \e enable parameter is \b true, CAN will be configured to interrupt any
//! transmission or reception and enter debug mode immediately after it is
//! requested. When \b false, CAN will wait for a started transmission or
//! reception to be completed before entering debug mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_setInterruptionDebugMode(uint32_t base, bool enable)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    if(enable)
    {
        //
        // Enable interrupt debug support
        //
        HWREGH(base + CAN_O_CTL) |= CAN_CTL_IDS;
    }
    else
    {
        //
        // Disable interrupt debug support
        //
        HWREGH(base + CAN_O_CTL) &= ~CAN_CTL_IDS;
    }
}

//*****************************************************************************
//
//! Enables DMA Requests from the CAN controller.
//!
//! \param base is the base address of the CAN controller to enable.
//!
//! Enables the CAN controller DMA request lines for each of the 3 interface
//! register sets.  To actually assert the request line, the DMA Active bit
//! must be set in the corresponding interface CMD register.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_enableDMARequests(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Set the DMA enable bits in the control register.
    //
    HWREG_BP(base + CAN_O_CTL) |=
    (CAN_CTL_DE1 | (uint32_t)CAN_CTL_DE2 | (uint32_t)CAN_CTL_DE3);
}

//*****************************************************************************
//
//! Disables DMA Requests from the CAN controller.
//!
//! \param base is the base address of the CAN controller to enable.
//!
//! Disables the CAN controller DMA request lines for each of the 3 interface
//! register sets.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_disableDMARequests(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Clear the DMA enable bits in the control register.
    //
    HWREG_BP(base + CAN_O_CTL) &=
    ~(CAN_CTL_DE1 | (uint32_t)CAN_CTL_DE2 | (uint32_t)CAN_CTL_DE3);
}

//*****************************************************************************
//
//! Disables Auto-Bus-On.
//!
//! \param base is the base address of the CAN controller.
//!
//! Disables the Auto-Bus-On feature of the CAN controller.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_disableAutoBusOn(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Clear the ABO bit in the control register.
    //
    HWREGH(base + CAN_O_CTL) &= ~CAN_CTL_ABO;
}

//*****************************************************************************
//
//! Enables Auto-Bus-On.
//!
//! \param base is the base address of the CAN controller.
//!
//! Enables the Auto-Bus-On feature of the CAN controller.  Be sure to also
//! configure the Auto-Bus-On time using the CAN_setAutoBusOnTime function.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_enableAutoBusOn(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Set the ABO bit in the control register.
    //
    HWREGH(base + CAN_O_CTL) |= CAN_CTL_ABO;
}

//*****************************************************************************
//
//! Sets the time before a Bus-Off recovery sequence is started.
//!
//! \param base is the base address of the CAN controller.
//! \param onTime is number of clock cycles before a Bus-Off recovery sequence
//!        is started.
//!
//! This function sets the number of clock cycles before a Bus-Off recovery
//! sequence is started by clearing the Init bit.
//!
//! \note To enable this functionality, use CAN_enableAutoBusOn().
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_setAutoBusOnTime(uint32_t base, uint32_t onTime)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Set bus-off timer value
    //
    HWREG_BP(base + CAN_O_ABOTR) = onTime;
}

//*****************************************************************************
//
//! Enables individual CAN controller interrupt sources.
//!
//! \param base is the base address of the CAN controller.
//! \param intFlags is the bit mask of the interrupt sources to be enabled.
//!
//! Enables specific interrupt sources of the CAN controller.  Only enabled
//! sources will cause a processor interrupt.
//!
//! The \e intFlags parameter is the logical OR of any of the following:
//! - \b CAN_INT_ERROR - a controller error condition has occurred
//! - \b CAN_INT_STATUS - a message transfer has completed, or a bus error has
//! been detected
//! - \b CAN_INT_IE0 - allow CAN controller to generate interrupts on interrupt
//! line 0
//! - \b CAN_INT_IE1 - allow CAN controller to generate interrupts on interrupt
//! line 1
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_enableInterrupt(uint32_t base, uint32_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((intFlags & ~(CAN_INT_ERROR | CAN_INT_STATUS | CAN_INT_IE0 |
                         CAN_INT_IE1)) == 0U);

    //
    // Enable the specified interrupts.
    //
    HWREG_BP(base + CAN_O_CTL) |= intFlags;
}

//*****************************************************************************
//
//! Disables individual CAN controller interrupt sources.
//!
//! \param base is the base address of the CAN controller.
//! \param intFlags is the bit mask of the interrupt sources to be disabled.
//!
//! Disables the specified CAN controller interrupt sources.  Only enabled
//! interrupt sources can cause a processor interrupt.
//!
//! The \e intFlags parameter has the same definition as in the
//! CAN_enableInterrupt() function.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_disableInterrupt(uint32_t base, uint32_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((intFlags & ~(CAN_INT_ERROR | CAN_INT_STATUS | CAN_INT_IE0 |
                         CAN_INT_IE1)) == 0U);

    //
    // Disable the specified interrupts.
    //
    HWREG_BP(base + CAN_O_CTL) &= ~(intFlags);
}

//*****************************************************************************
//
//! Get the CAN controller Interrupt Line set for each mailbox
//!
//! \param base is the base address of the CAN controller.
//!
//! Gets which interrupt line each message object should assert when an
//! interrupt occurs. Bit 0 corresponds to message object 32 and then bits
//! 1 to 31 correspond to message object 1 through 31 respectively. Bits that
//! are asserted indicate the message object should generate an interrupt on
//! interrupt line 1, while bits that are not asserted indicate the message
//! object should generate an interrupt on line 0.
//!
//! \return Returns the value of the interrupt muxing register.
//
//*****************************************************************************
static inline uint32_t
CAN_getInterruptMux(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Get the interrupt muxing for the CAN peripheral
    //
    return(HWREG_BP(base + CAN_O_IP_MUX21));
}

//*****************************************************************************
//
//! Set the CAN controller Interrupt Line for each mailbox
//!
//! \param base is the base address of the CAN controller.
//! \param mux bit packed representation of which message objects should
//!        generate an interrupt on a given interrupt line.
//!
//! Selects which interrupt line each message object should assert when an
//! interrupt occurs. Bit 0 corresponds to message object 32 and then bits
//! 1 to 31 correspond to message object 1 through 31 respectively. Bits that
//! are asserted indicate the message object should generate an interrupt on
//! interrupt line 1, while bits that are not asserted indicate the message
//! object should generate an interrupt on line 0.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_setInterruptMux(uint32_t base, uint32_t mux)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Set the interrupt muxing for the CAN peripheral
    //
    HWREG_BP(base + CAN_O_IP_MUX21) = mux;
}

//*****************************************************************************
//
//! Enables the CAN controller automatic retransmission behavior.
//!
//! \param base is the base address of the CAN controller.
//!
//! Enables the automatic retransmission of messages with detected errors.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_enableRetry(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Clearing the DAR bit tells the controller to not disable the
    // auto-retry of messages which were not transmitted or received
    // correctly.
    //
    HWREGH(base + CAN_O_CTL) &= ~CAN_CTL_DAR;
}

//*****************************************************************************
//
//! Disables the CAN controller automatic retransmission behavior.
//!
//! \param base is the base address of the CAN controller.
//!
//! Disables the automatic retransmission of messages with detected errors.
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_disableRetry(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Setting the DAR bit tells the controller to disable the auto-retry
    // of messages which were not transmitted or received correctly.
    //
    HWREGH(base + CAN_O_CTL) |= CAN_CTL_DAR;
}

//*****************************************************************************
//
//! Returns the current setting for automatic retransmission.
//!
//! \param base is the base address of the CAN controller.
//!
//! Reads the current setting for the automatic retransmission in the CAN
//! controller and returns it to the caller.
//!
//! \return Returns \b true if automatic retransmission is enabled, \b false
//! otherwise.
//
//*****************************************************************************
static inline bool
CAN_isRetryEnabled(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Read the disable automatic retry setting from the CAN controller.
    //
    return((bool)((HWREGH(base + CAN_O_CTL) & CAN_CTL_DAR) != CAN_CTL_DAR));
}

//*****************************************************************************
//
//! Reads the CAN controller error counter register.
//!
//! \param base is the base address of the CAN controller.
//! \param rxCount is a pointer to storage for the receive error counter.
//! \param txCount is a pointer to storage for the transmit error counter.
//!
//! Reads the error counter register and returns the transmit and receive error
//! counts to the caller along with a flag indicating if the controller receive
//! counter has reached the error passive limit.  The values of the receive and
//! transmit error counters are returned through the pointers provided as
//! parameters.
//!
//! After this call, \e rxCount will hold the current receive error count
//! and \e txCount will hold the current transmit error count.
//!
//! \return Returns \b true if the receive error count has reached the error
//! passive limit, and \b false if the error count is below the error passive
//! limit.
//
//*****************************************************************************
static inline bool
CAN_getErrorCount(uint32_t base, uint32_t *rxCount, uint32_t *txCount)
{
    uint32_t canError = 0U;

    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Read the current count of transmit/receive errors.
    //
    canError = HWREG_BP(base + CAN_O_ERRC);

    //
    // Extract the error numbers from the register value.
    //
    *rxCount = (canError & CAN_ERRC_REC_M) >> CAN_ERRC_REC_S;
    *txCount = (canError & CAN_ERRC_TEC_M) >> CAN_ERRC_TEC_S;

    return((bool)((canError & CAN_ERRC_RP) != 0U));
}

//*****************************************************************************
//
//! Reads the CAN controller error and status register.
//!
//! \param base is the base address of the CAN controller.
//!
//! Reads the error and status register of the CAN controller.
//!
//! \return Returns the value of the register.
//
//*****************************************************************************
static inline uint16_t
CAN_getStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Return error and status register value
    //
    return(HWREGH(base + CAN_O_ES));
}

//*****************************************************************************
//
//! Reads the CAN controller TX request register.
//!
//! \param base is the base address of the CAN controller.
//!
//! Reads the TX request register of the CAN controller.
//!
//! \return Returns the value of the register.
//
//*****************************************************************************
static inline uint32_t
CAN_getTxRequests(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Return Tx requests register value
    //
    return(HWREG_BP(base + CAN_O_TXRQ_21));
}

//*****************************************************************************
//
//! Reads the CAN controller new data status register.
//!
//! \param base is the base address of the CAN controller.
//!
//! Reads the new data status register of the CAN controller for all message
//! objects.
//!
//! \return Returns the value of the register.
//
//*****************************************************************************
static inline uint32_t
CAN_getNewDataFlags(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Return new data register value
    //
    return(HWREG_BP(base + CAN_O_NDAT_21));
}

//*****************************************************************************
//
//! Reads the CAN controller valid message object register.
//!
//! \param base is the base address of the CAN controller.
//!
//! Reads the valid message object register of the CAN controller.
//!
//! \return Returns the value of the register.
//
//*****************************************************************************
static inline uint32_t
CAN_getValidMessageObjects(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Return the valid message register value
    //
    return(HWREG_BP(base + CAN_O_MVAL_21));
}

//*****************************************************************************
//
//! Get the CAN controller interrupt cause.
//!
//! \param base is the base address of the CAN controller.
//!
//! This function returns the value of the interrupt register that indicates
//! the cause of the interrupt.
//!
//! \return Returns the value of the interrupt register.
//
//*****************************************************************************
static inline uint32_t
CAN_getInterruptCause(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Read interrupt identifier status
    //
    return(HWREG_BP(base + CAN_O_INT));
}

//*****************************************************************************
//
//! Get the CAN controller pending interrupt message source.
//!
//! \param base is the base address of the CAN controller.
//!
//! Returns the value of the pending interrupts register that indicates
//! which messages are the source of pending interrupts.
//!
//! \return Returns the value of the pending interrupts register.
//
//*****************************************************************************
static inline uint32_t
CAN_getInterruptMessageSource(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Read message object interrupt status
    //
    return(HWREG_BP(base + CAN_O_IPEN_21));
}

//*****************************************************************************
//
//! CAN Global interrupt Enable function.
//!
//! \param base is the base address of the CAN controller.
//! \param intFlags is the bit mask of the interrupt sources to be enabled.
//!
//! Enables specific CAN interrupt in the global interrupt enable register
//!
//! The \e intFlags parameter is the logical OR of any of the following:
//! - \b CAN_GLOBAL_INT_CANINT0  - Global Interrupt Enable bit for CAN INT0
//! - \b CAN_GLOBAL_INT_CANINT1  - Global Interrupt Enable bit for CAN INT1
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_enableGlobalInterrupt(uint32_t base, uint16_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((intFlags & ~(CAN_GLOBAL_INT_CANINT0 |
                         CAN_GLOBAL_INT_CANINT1)) == 0U);

    //
    // Enable the requested interrupts
    //
    HWREGH(base + CAN_O_GLB_INT_EN) |= intFlags;
}

//*****************************************************************************
//
//! CAN Global interrupt Disable function.
//!
//! \param base is the base address of the CAN controller.
//! \param intFlags is the bit mask of the interrupt sources to be disabled.
//!
//! Disables the specific CAN interrupt in the global interrupt enable register
//!
//! The \e intFlags parameter is the logical OR of any of the following:
//! - \b CAN_GLOBAL_INT_CANINT0 - Global Interrupt bit for CAN INT0
//! - \b CAN_GLOBAL_INT_CANINT1 - Global Interrupt bit for CAN INT1
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_disableGlobalInterrupt(uint32_t base, uint16_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((intFlags & ~(CAN_GLOBAL_INT_CANINT0 |
                         CAN_GLOBAL_INT_CANINT1)) == 0U);

    //
    // Disable the requested interrupts
    //
    HWREGH(base + CAN_O_GLB_INT_EN) &= ~intFlags;
}

//*****************************************************************************
//
//! CAN Global interrupt Clear function.
//!
//! \param base is the base address of the CAN controller.
//! \param intFlags is the bit mask of the interrupt sources to be cleared.
//!
//! Clear the specific CAN interrupt bit in the global interrupt flag register.
//!
//! The \e intFlags parameter is the logical OR of any of the following:
//! - \b CAN_GLOBAL_INT_CANINT0 - Global Interrupt bit for CAN INT0
//! - \b CAN_GLOBAL_INT_CANINT1 - Global Interrupt bit for CAN INT1
//!
//! \return None.
//
//*****************************************************************************
static inline void
CAN_clearGlobalInterruptStatus(uint32_t base, uint16_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((intFlags & ~(CAN_GLOBAL_INT_CANINT0 |
                         CAN_GLOBAL_INT_CANINT1)) == 0U);

    //
    // Clear the requested interrupts
    //
    HWREGH(base + CAN_O_GLB_INT_CLR) |= intFlags;
}

//*****************************************************************************
//
//! Get the CAN Global Interrupt status.
//!
//! \param base is the base address of the CAN controller.
//! \param intFlags is the bit mask of the interrupt sources to be enabled.
//!
//! Check if any interrupt bit is set in the global interrupt flag register.
//!
//! The \e intFlags parameter is the logical OR of any of the following:
//! - \b CAN_GLOBAL_INT_CANINT0 - Global Interrupt bit for CAN INT0
//! - \b CAN_GLOBAL_INT_CANINT1 - Global Interrupt bit for CAN INT1
//!
//! \return True if any of the requested interrupt bits are set. False, if
//! none of the requested bits are set.
//
//*****************************************************************************
static inline bool
CAN_getGlobalInterruptStatus(uint32_t base, uint16_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((intFlags & ~(CAN_GLOBAL_INT_CANINT0 |
                         CAN_GLOBAL_INT_CANINT1)) == 0U);

    //
    // Read and return the global interrupt flag register
    //
    return((bool)((HWREGH(base + CAN_O_GLB_INT_FLG) & intFlags) != 0U));
}

//*****************************************************************************
//
//! Initializes the CAN controller
//!
//! \param base is the base address of the CAN controller.
//!
//! This function initializes the message RAM, which also clears all the
//! message objects, and places the CAN controller in an init state. Write
//! access to the configuration registers is available as a result, allowing
//! the bit timing and message objects to be setup.
//!
//! \note To exit the initialization mode and start the CAN module, use the
//! CAN_startModule() function.
//!
//! \return None.
//
//*****************************************************************************
extern void
CAN_initModule(uint32_t base);

//*****************************************************************************
//
//! Sets the CAN Bit Timing based on requested Bit Rate.
//!
//! \param base is the base address of the CAN controller.
//! \param clockFreq is the CAN module clock frequency before the bit rate
//!        prescaler (Hertz)
//! \param bitRate is the desired bit rate (bits/sec)
//! \param bitTime is the number of time quanta per bit required for desired
//!        bit time (Tq) and must be in the range from 8 to 25
//!
//! This function sets the CAN bit timing values for the bit rate passed in the
//! \e bitRate and \e bitTime parameters based on the \e clockFreq parameter.  The
//! CAN bit clock is calculated to be an average timing value that should work
//! for most systems.  If tighter timing requirements are needed, then the
//! CAN_setBitTiming() function is available for full customization of all of
//! the CAN bit timing values.
//!
//! \note Not all bit-rate and bit-time combinations are valid.
//!       For combinations that would yield the correct bit-rate,
//!       refer to the DCAN_CANBTR_values.xlsx file in the "docs" directory.
//!       The CANBTR register values calculated by the function CAN_setBitRate
//!       may not be suitable for your network parameters. If this is the case
//!       and you have computed the correct values for your network, you could
//!       directly write those parameters in CANBTR register using the
//!       function CAN_setBitTiming.
//!
//! \return None.
//
//*****************************************************************************
extern void
CAN_setBitRate(uint32_t base, uint32_t clockFreq, uint32_t bitRate,
               uint16_t bitTime);

//*****************************************************************************
//
//! Manually set the CAN controller bit timing.
//!
//! \param base is the base address of the CAN controller.
//! \param prescaler is the baud rate prescaler
//! \param prescalerExtension is the baud rate prescaler extension
//! \param tSeg1 is the time segment 1
//! \param tSeg2 is the time segment 2
//! \param sjw is the synchronization jump width
//!
//! This function sets the various timing parameters for the CAN bus bit
//! timing: baud rate prescaler, prescaler extension, time segment 1,
//! time segment 2, and the Synchronization Jump Width.
//!
//! \return None.
//
//*****************************************************************************
extern void
CAN_setBitTiming(uint32_t base, uint16_t prescaler,
                 uint16_t prescalerExtension, uint16_t tSeg1, uint16_t tSeg2,
                 uint16_t sjw);


//*****************************************************************************
//
//! Clears a CAN interrupt source.
//!
//! \param base is the base address of the CAN controller.
//! \param intClr is a value indicating which interrupt source to clear.
//!
//! This function can be used to clear a specific interrupt source.  The
//! \e intClr parameter should be either a number from 1 to 32 to clear a
//! specific message object interrupt or can be the following:
//! - \b CAN_INT_INT0ID_STATUS - Clears a status interrupt.
//!
//! It is not necessary to use this function to clear an interrupt.  This
//! should only be used if the application wants to clear an interrupt source
//! without taking the normal interrupt action.
//!
//! \return None.
//
//*****************************************************************************
extern void
CAN_clearInterruptStatus(uint32_t base, uint32_t intClr);

//*****************************************************************************
//
//! Setup a Message Object
//!
//! \param base is the base address of the CAN controller.
//! \param objID is the message object number to configure (1-32).
//! \param msgID is the CAN message identifier used for the 11 or 29 bit
//!        identifiers
//! \param frame is the CAN ID frame type
//! \param msgType is the message object type
//! \param msgIDMask is the CAN message identifier mask used when identifier
//!        filtering is enabled
//! \param flags is the various flags and settings to be set for the message
//!        object
//! \param msgLen is the number of bytes of data in the message object (0-8)
//!
//! This function sets the various values required for a message object.
//!
//! The \e frame parameter can be one of the following values:
//! - \b CAN_MSG_FRAME_STD - Standard 11 bit identifier
//! - \b CAN_MSG_FRAME_EXT - Extended 29 bit identifier
//!
//! The \e msgType parameter can be one of the following values:
//! - \b CAN_MSG_OBJ_TYPE_TX          - Transmit Message
//! - \b CAN_MSG_OBJ_TYPE_TX_REMOTE   - Transmit Remote Message
//! - \b CAN_MSG_OBJ_TYPE_RX          - Receive Message
//! - \b CAN_MSG_OBJ_TYPE_RXTX_REMOTE - Receive Remote message with
//!                                     auto-transmit
//!
//! The \e flags parameter can be set as \b CAN_MSG_OBJ_NO_FLAGS if no flags
//! are required or the parameter can be a logical OR of any of the following
//! values:
//! - \b CAN_MSG_OBJ_TX_INT_ENABLE    - Enable Transmit Interrupts
//! - \b CAN_MSG_OBJ_RX_INT_ENABLE    - Enable Receive Interrupts
//! - \b CAN_MSG_OBJ_USE_ID_FILTER    - Use filtering based on the Message ID
//! - \b CAN_MSG_OBJ_USE_EXT_FILTER   - Use filtering based on the Extended
//!                                     Message ID
//! - \b CAN_MSG_OBJ_USE_DIR_FILTER   - Use filtering based on the direction of
//!                                     the transfer
//! - \b CAN_MSG_OBJ_FIFO             - Message object is part of a FIFO
//!                                     structure and isn't the final message
//!                                     object in FIFO
//!
//! If filtering is based on message identifier, the value
//! \b CAN_MSG_OBJ_USE_ID_FILTER has to be logically ORed with the \e flag
//! parameter and \b CAN_MSG_OBJ_USE_EXT_FILTER also has to be ORed for
//! message identifier filtering to be based on the extended identifier.
//!
//! \note The \b msgLen Parameter for the Receive Message Object is a "don't
//!       care" but its value should be between 0-8 due to the assert.
//!
//! \return None.
//
//*****************************************************************************
extern void
CAN_setupMessageObject(uint32_t base, uint32_t objID, uint32_t msgID,
                       CAN_MsgFrameType frame, CAN_MsgObjType msgType,
                       uint32_t msgIDMask, uint32_t flags, uint16_t msgLen);

//*****************************************************************************
//
//! Sends a Message Object
//!
//! \param base is the base address of the CAN controller.
//! \param objID is the object number to configure (1-32).
//! \param msgLen is the number of bytes of data in the message object (0-8)
//! \param msgData is a pointer to the message object's data
//!
//! This function is used to transmit a message object and the message data,
//! if applicable.
//!
//! \note The message object requested by the \e objID must first be setup
//! using the CAN_setupMessageObject() function.
//!
//! \return None.
//
//*****************************************************************************
extern void
CAN_sendMessage(uint32_t base, uint32_t objID, uint16_t msgLen,
                const uint16_t *msgData);

//*****************************************************************************
//
//! Sends a Message Object
//!
//! \param base is the base address of the CAN controller.
//! \param objID is the object number to configure (1-32).
//! \param msgLen is the number of bytes of data in the message object (0-8)
//! \param msgData is a pointer to the message object's data (all 16 bits are sent)
//!
//! This function is used to transmit a message object and the message data,
//! if applicable.
//!
//! \note The message object requested by the \e objID must first be setup
//! using the CAN_setupMessageObject() function.
//!
//! \return None.
//
//*****************************************************************************

extern void
CAN_sendMessage_16bit(uint32_t base, uint32_t objID, uint16_t msgLen,
                const uint16_t *msgData);

//*****************************************************************************
//
//! Sends a Message Object
//!
//! \param base is the base address of the CAN controller.
//! \param objID is the object number to configure (1-32).
//! \param msgLen is the number of bytes of data in the message object (0-8)
//! \param msgData is a pointer to the message object's data (all 32 bits are sent)
//!
//! This function is used to transmit a message object and the message data,
//! if applicable.
//!
//! \note The message object requested by the \e objID must first be setup
//! using the CAN_setupMessageObject() function.
//!
//! \return None.
//
//*****************************************************************************

extern void
CAN_sendMessage_32bit(uint32_t base, uint32_t objID, uint16_t msgLen,
                const uint32_t *msgData);

//*****************************************************************************
//
//! Sends a Message Object while dynamically updating data length
//!
//! \param base is the base address of the CAN controller.
//! \param objID is the object number to configure (1-32).
//! \param msgLen is the number of bytes of data in the message object (0-8)
//! \param msgData is a pointer to the message object's data
//!
//! This function is used to transmit a message object and the message data,
//! if applicable and can be used to dynamically update the data length
//! for every subsequent call of this function.
//!
//! \note The message object requested by the \e objID must first be setup
//! using the CAN_setupMessageObject() function.
//!
//! \return None.
//
//*****************************************************************************

extern void
CAN_sendMessage_updateDLC(uint32_t base, uint32_t objID, uint16_t msgLen,
                  const uint16_t *msgData);

//*****************************************************************************
//
//! Sends a Remote Request Message Object
//!
//! \param base is the base address of the CAN controller.
//! \param objID is the object number to configure (1-32).
//!
//! This function is used to transmit a remote request message object.
//!
//! \note The message object requested by the \e objID must first be setup
//! using the CAN_setupMessageObject() function with CAN_MSG_OBJ_TYPE_TX_REMOTE
//! as msgType flag.
//!
//! \return None.
//
//*****************************************************************************
extern void
CAN_sendRemoteRequestMessage(uint32_t base, uint32_t objID);

//*****************************************************************************
//
//! Reads the data in a Message Object
//!
//! \param base is the base address of the CAN controller.
//! \param objID is the object number to read (1-32).
//! \param msgData is a pointer to the array to store the message data
//!
//! This function is used to read the data contents of the specified message
//! object in the CAN controller. The data returned is stored in the
//! \e msgData parameter.
//!
//! \note
//! -# The message object requested by the \e objID must first be setup
//! using the CAN_setupMessageObject() function.
//! -# If the DLC of the received message is larger than the \e msgData
//! buffer provided, then it is possible for a buffer overflow to occur.
//!
//! \return Returns \b true if new data was retrieved, else returns
//! \b false to indicate no new data was retrieved.
//
//*****************************************************************************
extern bool
CAN_readMessage(uint32_t base, uint32_t objID,
                uint16_t *msgData);

//*****************************************************************************
//
//! Reads the data and Message ID in a Message Object
//!
//! \param base is the base address of the CAN controller.
//! \param objID is the object number to read (1-32).
//! \param frameType is a pointer to the CAN_MsgFrameType to store the message
//!        type that has been received in the mailbox
//! The \e frameType parameter shall be filled as of the following values:
//! - \b CAN_MSG_FRAME_STD - Standard 11 bit identifier
//! - \b CAN_MSG_FRAME_EXT - Extended 29 bit identifier
//! This parameter is filled when return value is true for this function.
//! \param msgID is a pointer to storage for the received Message ID
//! Filled when the return value is true for this function.
//! \param msgData is a pointer to the array to store the message data
//! Filled with read Data when the return value is true for this function.
//!
//! This function is used to read the data contents and the Message ID
//! of the specified message object in the CAN controller.The Message ID returned
//! is stored in the \e msgID parameter and its type in \e frameType parameter.
//! The data returned is stored in the \e msgData parameter.
//!
//! \note
//! -# The message object requested by the \e objID must first be setup
//! using the CAN_setupMessageObject() function.
//!
//! \return Returns \b true if new data was retrieved, else returns
//! \b false to indicate no new data was retrieved.
//
//*****************************************************************************
extern bool CAN_readMessageWithID(uint32_t base,
                                  uint32_t objID,
                                  CAN_MsgFrameType *frameType,
                                  uint32_t *msgID,
                                  uint16_t *msgData);


//*****************************************************************************
//
//! Transfers a CAN message between the IF registers and Message RAM.
//!
//! \param base is the base address of the CAN controller.
//! \param interface is the interface to use for the transfer. Valid value are
//!        1 or 2.
//! \param objID is the object number to transfer (1-32).
//! \param direction is the direction of the transfer.
//!         False is Message RAM to IF, True is IF to Message RAM.
//! \param dmaRequest asserts the DMA request line after a transfer if
//!        set to True.
//!
//! This function transfers the contents of the interface registers to message
//! RAM or vice versa depending on the value passed to direction.  This
//! function is designed to be used with DMA transfers.
//!
//! \return None.
//
//*****************************************************************************
extern void
CAN_transferMessage(uint32_t base, uint16_t interface, uint32_t objID,
                    bool direction, bool dmaRequest);


//*****************************************************************************
//
//! Clears a message object so that it is no longer used.
//!
//! \param base is the base address of the CAN controller.
//! \param objID is the message object number to disable (1-32).
//!
//! This function frees(disables) the specified message object from use. Once
//! a message object has been cleared, it will no longer automatically send or
//! receive messages, or generate interrupts.
//!
//! \return None.
//
//*****************************************************************************
extern void
CAN_clearMessage(uint32_t base, uint32_t objID);

//*****************************************************************************
//
//! Disables specific message object
//!
//! \param base is the base address of the CAN controller.
//! \param objID is the message object number to disable (1-32).
//!
//! This function disables the specific message object. Once the message object
//! has been disabled it will be ignored by the message handler until it
//! is configured again.
//!
//! \return None.
//
//*****************************************************************************
extern void
CAN_disableMessageObject(uint32_t base, uint32_t objID);

//*****************************************************************************
//
//! Disables all message objects
//!
//! \param base is the base address of the CAN controller.
//!
//! This function disables all message objects. Once a message object
//! has been disabled it will be ignored by the message handler until it
//! is configured again. All message objects are disabled automatically on
//! reset, however this function can be used to restart CAN operations
//! without an external reset.
//!
//! \return None.
//
//*****************************************************************************
extern void
CAN_disableAllMessageObjects(uint32_t base);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

#endif // #ifdef __TMS320C28XX__

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //  CAN_H
