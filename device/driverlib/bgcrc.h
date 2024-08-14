//###########################################################################
//
// FILE:   bgcrc.h
//
// TITLE:  C28x BGCRC driver.
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

#ifndef BGCRC_H
#define BGCRC_H

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
//! \addtogroup bgcrc_api BGCRC
//! \brief This module is used for configuring BGCRC.
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_bgcrc.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "debug.h"
#include "cpu.h"



//*****************************************************************************
//
//! Values that can be passed to BGCRC_setConfig() as the \e nmiConfig
//! parameter
//
//*****************************************************************************
#define BGCRC_NMI_ENABLE      0x5U  //!< Generate NMI on memory check failure
#define BGCRC_NMI_DISABLE     0xAU  //!< Do not generate NMI on memory check
                                    //!<failure

//*****************************************************************************
//
//! Values that can be passed to BGCRC_setConfig() as the \e emuCtrl parameter
//
//*****************************************************************************
#define BGCRC_EMUCTRL_SOFT   0x00U  //!< CRC module and CRC watchdog stops
                                    //!< immediately on DEBUG SUSPEND
#define BGCRC_EMUCTRL_FREE   0x10U  //!< CRC calculation and CRC watchdog is
                                    //!< not affected by DEBUG HALT

//*****************************************************************************
//
//! Values that can be passed to BGCRC_setRegion() as the \e mode parameter
//
//*****************************************************************************
#define BGCRC_SCRUB_MODE     0xAU   //!< CRC of data is not compared with the
                                    //!< golden CRC. Error check is done using
                                    //!< the ECC/Parity logic.
#define BGCRC_CRC_MODE       0x5U   //!< CRC value is compared with golden CRC
                                    //!< at the end in addition to the data
                                    //!< correctness check by ECC/Parity logic

//*****************************************************************************
//
//! Values that can be passed to all interrupt/NMI functions as \e intflags or
//! \e nmiflags parameter
//
//*****************************************************************************
#define BGCRC_GLOBAL_INT     0x01U  //!< Global Interrupt

#define BGCRC_TEST_DONE      0x02U  //!< Test done interrupt

#define BGCRC_CRC_FAIL       0x04U  //!< CRC Fail Interrupt/NMI
#define BGCRC_UNCORR_ERR     0x08U  //!< Uncorrectable Error Interrupt/NMI
#define BGCRC_CORR_ERR       0x10U  //!< Correctable Error Interrupt/NMI
#define BGCRC_WD_UNDERFLOW   0x20U  //!< Watchdog Underflow Error Interrupt/NMI
#define BGCRC_WD_OVERFLOW    0x40U  //!< Watchdog Overflow Error Interrupt/NMI

#define BGCRC_ALL_ERROR_FLAGS  (BGCRC_CRC_FAIL | BGCRC_UNCORR_ERR |            \
                                BGCRC_CORR_ERR | BGCRC_WD_UNDERFLOW |          \
                                BGCRC_WD_OVERFLOW)

//*****************************************************************************
//
//! Values that can be passed to BGCRC_setRegion as the \e blockSize parameter
//! For example, to configure a region of size 2.5KB, set \e blockSize as
//! BGCRC_SIZE_KBYTES(2) + BGCRC_SIZE_BYTES_512
//
//*****************************************************************************
#define BGCRC_SIZE_BYTES_0      0U
#define BGCRC_SIZE_BYTES_256    1U
#define BGCRC_SIZE_BYTES_512    2U
#define BGCRC_SIZE_BYTES_768    3U
#define BGCRC_SIZE_KBYTES(x)    (4U * (x))


//*****************************************************************************
//
//! Values returned by the function BGCRC_getRunStatus
//
//*****************************************************************************
#define BGCRC_ACTIVE      0x80000000U
#define BGCRC_IDLE        0x00000000U

//*****************************************************************************
//
//! Values that can be passed to BGCRC_lockRegister(), BGCRC_unlockRegister()
//! and BGCRC_commitLockRegister() as the \e regs parameter
//!
//
//*****************************************************************************
#define BGCRC_REG_EN            0x00000001U
#define BGCRC_REG_CTRL1         0x00000002U
#define BGCRC_REG_CTRL2         0x00000004U
#define BGCRC_REG_START_ADDR    0x00000008U
#define BGCRC_REG_SEED          0x00000010U
#define BGCRC_REG_GOLDEN        0x00000080U
#define BGCRC_REG_WD_CFG        0x00004000U
#define BGCRC_REG_WD_MIN        0x00008000U
#define BGCRC_REG_WD_MAX        0x00010000U
#define BGCRC_REG_NMIFRC        0x00800000U
#define BGCRC_REG_INTEN         0x04000000U
#define BGCRC_REG_INTFRC        0x20000000U
#define BGCRC_REG_ALL           0xFFFFFFFFU

//*****************************************************************************
//
//! Values that are used by the BGCRC APIs
//
//*****************************************************************************
#define BGCRC_WD_ENABLE         0x5UL
#define BGCRC_WD_DISABLE        0xAUL
#define BGCRC_START_KEY         0xAUL
#define BGCRC_HALT_KEY          0xAUL

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! \internal
//! Checks an BGCRC base address.
//!
//! \param base is the BGCRC module base address.
//!
//! This function determines if a BGCRC module base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
BGCRC_isBaseValid(uint32_t base)
{
    return(
           (base == BGCRC_CPU_BASE) ||
           (base == BGCRC_CLA1_BASE)
          );
}
#endif

//*****************************************************************************
//
//! Sets the NMI and emulation control configurations
//!
//! \param base is the BGCRC module base address.
//! \param nmiConfig specifies whether NMI needs to be enabled or not for
//!                  memory check failure
//! \param emuCtrl specifies the behaviour of CRC during emulation
//!
//! This function configures the NMI and emulation control configurations for
//! the BGCRC module.
//!
//! \e nmiConfig can take values \b BGCRC_NMI_ENABLE or \b BGCRC_NMI_DISABLE.
//! NMI is enabled by default for the memory check failures.
//!
//! \e emuCtrl can take values \b BGCRC_EMUCTRL_FREE or \b BGCRC_EMUCTRL_SOFT.
//!
//! \return None.
//
//*****************************************************************************
static inline void
BGCRC_setConfig(uint32_t base, uint32_t nmiConfig, uint32_t emuCtrl)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));
    ASSERT((nmiConfig == BGCRC_NMI_ENABLE) || (nmiConfig == BGCRC_NMI_DISABLE));
    ASSERT((emuCtrl == BGCRC_EMUCTRL_FREE) || (emuCtrl == BGCRC_EMUCTRL_SOFT));

    //
    // Set emulation control and NMI configuration
    //
    EALLOW;
    HWREG(base + BGCRC_O_CTRL1) = (nmiConfig << BGCRC_CTRL1_NMIDIS_S) | emuCtrl;
    EDIS;
}

//*****************************************************************************
//
//! Enables the watchdog
//!
//! \param base is the BGCRC module base address.
//!
//! This function enables the BGCRC watchdog timer. The counter is started
//! when the BGCRC test starts
//!
//! \note The watchDog window can be configured using the function
//! BGCRC_setWatchdogWindow()
//!
//! \return None.
//
//*****************************************************************************
static inline void
BGCRC_enableWatchdog(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));

    //
    // Enable watchdog
    //
    EALLOW;
    HWREG(base + BGCRC_O_WD_CFG) = BGCRC_WD_ENABLE;
    EDIS;
}

//*****************************************************************************
//
//! Disables the watchdog
//!
//! \param base is the BGCRC module base address.
//!
//! This function disables the BGCRC watchdog.
//!
//! \return None.
//
//*****************************************************************************
static inline void
BGCRC_disableWatchdog(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));

    //
    // Disable watchdog
    //
    EALLOW;
    HWREG(base + BGCRC_O_WD_CFG) = BGCRC_WD_DISABLE;
    EDIS;
}

//*****************************************************************************
//
//! Configures the BGCRC watchdog window
//!
//! \param base is the BGCRC module base address.
//! \param min is minimum value configuration for the windowed watchdog
//! \param max is maximum value configuration for the windowed watchdog
//!
//! This function configures the BGCRC windowed watchdog
//!
//! \return None.
//
//*****************************************************************************
static inline void
BGCRC_setWatchdogWindow(uint32_t base, uint32_t min, uint32_t max)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));
    ASSERT(max > min);

    //
    // Set the min and max values
    //
    EALLOW;
    HWREG(base + BGCRC_O_WD_MIN) = min;
    HWREG(base + BGCRC_O_WD_MAX) = max;
    EDIS;
}
//*****************************************************************************
//
//! Configures the memory region for check
//!
//! \param base is the BGCRC module base address.
//! \param startAddr is the start address of the block
//! \param blockSize is the size of the block
//! \param mode is the BGCRC test mode
//!
//! This function configures the memory region to be checked
//!
//! \e blockSize can take values \b BGCRC_SIZE_KBYTES(x) and/or
//! \b BGCRC_SIZE_BYTES_x. For example, to configure a region of size 2.5KB,
//! set \e blockSize as BGCRC_SIZE_KBYTES(2) + BGCRC_SIZE_BYTES_512
//!
//! \e mode can take values \b BGCRC_SCRUB_MODE or \b BGCRC_CRC_MODE. In Scrub
//! mode, CRC of data is not compared with the golden CRC. Error check is done
//! using the ECC/Parity logic. In CRC mode, value is compared with golden CRC
//! at the end in addition to the data correctness check by ECC/Parity logic.
//!
//! \return None.
//
//*****************************************************************************
static inline void
BGCRC_setRegion(uint32_t base, uint32_t startAddr,
                uint32_t blockSize, uint32_t mode)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));
    ASSERT((blockSize > BGCRC_SIZE_BYTES_0) &&
           (blockSize <= BGCRC_SIZE_KBYTES(256U)));
    ASSERT((mode == BGCRC_SCRUB_MODE) || (mode == BGCRC_CRC_MODE));

    //
    // Set the start address and size of the block. Set the mode of operation
    //

    EALLOW;

    HWREG(base + BGCRC_O_START_ADDR) = startAddr;
    HWREG(base + BGCRC_O_CTRL2) =
                    ((blockSize - 1U) << BGCRC_CTRL2_BLOCK_SIZE_S) |
                    (mode << BGCRC_CTRL2_SCRUB_MODE_S);
    EDIS;
}

//*****************************************************************************
//
//! Gets the BGCRC watchdog counter value
//!
//! \param base is the BGCRC module base address.
//!
//! This function returns the BGCRC watchdog counter value
//!
//! \return Watchdog counter value.
//
//*****************************************************************************
static inline uint32_t
BGCRC_getWatchdogCounterValue(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));

    return(HWREG(base + BGCRC_O_WD_CNT));
}

//*****************************************************************************
//
//! Enables individual BGCRC interrupt sources
//!
//! \param base is the BGCRC module base address.
//! \param intFlags is the bit mask of the interrupt sources to be disabled.
//!
//! This function enables the indicated BGCRC interrupt sources.  Only the
//! sources that are enabled can be reflected to the processor interrupt,
//! disabled sources have no effect on the processor.
//!
//! The \e intFlags parameter is the logical OR of any of the following:
//!
//!  - \b BGCRC_TEST_DONE    - Test done interrupt
//!  - \b BGCRC_CRC_FAIL     - CRC Fail Interrupt
//!  - \b BGCRC_UNCORR_ERR   - Uncorrectable Error Interrupt
//!  - \b BGCRC_CORR_ERR     - Correctable Error Interrupt
//!  - \b BGCRC_WD_UNDERFLOW - Watchdog Underflow Error Interrupt
//!  - \b BGCRC_WD_OVERFLOW  - Watchdog Overflow Error Interrupt
//!
//! \return None
//
//*****************************************************************************
static inline void
BGCRC_enableInterrupt(uint32_t base, uint32_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));
    ASSERT((intFlags & (BGCRC_TEST_DONE | BGCRC_ALL_ERROR_FLAGS)) == intFlags);

    EALLOW;
    HWREG(base + BGCRC_O_INTEN) |= intFlags;
    EDIS;
}

//*****************************************************************************
//
//! Disables individual BGCRC interrupt sources.
//!
//! \param base is the BGCRC module base address.
//! \param intFlags is the bit mask of the interrupt sources to be disabled.
//!
//! This function disables the indicated BGCRC interrupt sources.  Only the
//! sources that are enabled can be reflected to the processor interrupt
//! disabled sources have no effect on the processor.
//!
//! The \e intFlags parameter has the same definition as the
//! \e intFlags parameter to BGCRC_enableInterrupt().
//!
//! \return None.
//
//*****************************************************************************
static inline void
BGCRC_disableInterrupt(uint32_t base, uint32_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));
    ASSERT((intFlags & (BGCRC_TEST_DONE | BGCRC_ALL_ERROR_FLAGS)) == intFlags);

    EALLOW;
    HWREG(base + BGCRC_O_INTEN) &= ~intFlags;
    EDIS;
}

//*****************************************************************************
//
//! Gets the current BGCRC interrupt status
//!
//! \param base is the BGCRC module base address
//!
//! This function returns the status of the BGCRC interrupts
//!
//! \return The current interrupt status, which is a logical OR of any of the
//! following:
//!
//!  - \b BGCRC_GLOBAL_INT   - Global Interrupt
//!  - \b BGCRC_TEST_DONE    - Test done interrupt
//!  - \b BGCRC_CRC_FAIL     - CRC Fail Interrupt
//!  - \b BGCRC_UNCORR_ERR   - Uncorrectable Error Interrupt
//!  - \b BGCRC_CORR_ERR     - Correctable Error Interrupt
//!  - \b BGCRC_WD_UNDERFLOW - Watchdog Underflow Error Interrupt
//!  - \b BGCRC_WD_OVERFLOW  - Watchdog Overflow Error Interrupt
//
//*****************************************************************************
static inline uint32_t
BGCRC_getInterruptStatus(uint32_t base)
{
    return(HWREG(base + BGCRC_O_INTFLG));
}

//*****************************************************************************
//
//! Clears the BGCRC interrupt sources
//!
//! \param base is the BGCRC module base address
//! \param intFlags is a bit mask of the interrupt sources to be cleared.
//!
//! The specified BGCRC interrupt sources are cleared, so that they no longer
//! assert. This function must be called in the interrupt handler to keep the
//! interrupt from being triggered again immediately upon exit.
//!
//! The \e intFlags parameter has the same definition as the
//! \e intFlags parameter to BGCRC_getInterruptStatus().
//!
//! \return None.
//
//*****************************************************************************
static inline void
BGCRC_clearInterruptStatus(uint32_t base, uint32_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));
    ASSERT((intFlags & (BGCRC_GLOBAL_INT | BGCRC_TEST_DONE |
                        BGCRC_ALL_ERROR_FLAGS)) == intFlags);

    EALLOW;
    HWREG(base + BGCRC_O_INTCLR) = intFlags;
    EDIS;
}

//*****************************************************************************
//
//! Force the BGCRC interrupt flag
//!
//! \param base is the BGCRC module base address
//! \param intFlags is a bit mask of the interrupt flags to be forced.
//!
//! This function forces the specified interrupt flags
//!
//! The \e intFlags parameter has the same definition as the
//! \e intFlags parameter to BGCRC_enableInterrupt().
//!
//! \return None.
//
//*****************************************************************************
static inline void
BGCRC_forceInterrupt(uint32_t base, uint32_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));
    ASSERT((intFlags & (BGCRC_TEST_DONE | BGCRC_ALL_ERROR_FLAGS)) == intFlags);

    EALLOW;
    HWREG(base + BGCRC_O_INTFRC) = intFlags;
    EDIS;
}

//*****************************************************************************
//
//! Gets the current BGCRC NMI status
//!
//! \param base is the BGCRC module base address
//!
//! This function returns the status of the BGCRC NMI flags
//!
//! \return The current NMI status, which is a logical OR of any of the
//! following:
//!
//!  - \b BGCRC_CRC_FAIL     - CRC Fail NMI
//!  - \b BGCRC_UNCORR_ERR   - Uncorrectable Error NMI
//!  - \b BGCRC_CORR_ERR     - Correctable Error NMI
//!  - \b BGCRC_WD_UNDERFLOW - Watchdog Underflow Error NMI
//!  - \b BGCRC_WD_OVERFLOW  - Watchdog Overflow Error NMI
//
//*****************************************************************************
static inline uint32_t
BGCRC_getNMIStatus(uint32_t base)
{
    return(HWREG(base + BGCRC_O_NMIFLG));
}

//*****************************************************************************
//
//! Clears the BGCRC NMI sources
//!
//! \param base is the BGCRC module base address
//! \param nmiFlags is a bit mask of the NMI sources to be cleared.
//!
//! The specified BGCRC NMI sources are cleared, so that they no longer
//! assert. This function must be called in the NMI handler to keep the
//! NMI from being triggered again immediately upon exit.
//!
//! The \e nmiFlags parameter has the same definition as the
//! \e nmiFlags parameter to BGCRC_getNMIStatus().
//!
//! \return None.
//
//*****************************************************************************
static inline void
BGCRC_clearNMIStatus(uint32_t base, uint32_t nmiFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));
    ASSERT((nmiFlags & BGCRC_ALL_ERROR_FLAGS) == nmiFlags);

    EALLOW;
    HWREG(base + BGCRC_O_NMICLR) = nmiFlags;
    EDIS;
}

//*****************************************************************************
//
//! Force the BGCRC NMI flag
//!
//! \param base is the BGCRC module base address
//! \param nmiFlags is a bit mask of the NMI flags to be forced.
//!
//! This function forces the specified interrupt flags
//!
//! The \e nmiFlags parameter has the same definition as the
//! \e nmiFlags parameter to BGCRC_getNMIStatus().
//!
//! \return None.
//
//*****************************************************************************
static inline void
BGCRC_forceNMI(uint32_t base, uint32_t nmiFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));
    ASSERT((nmiFlags & BGCRC_ALL_ERROR_FLAGS) == nmiFlags);

    EALLOW;
    HWREG(base + BGCRC_O_NMIFRC) = nmiFlags;
    EDIS;
}

//*****************************************************************************
//
//! Sets the golden CRC value
//!
//! \param base is the BGCRC module base address
//! \param crcVal is a golden CRC value to be programmed
//!
//! This function sets the golden CRC value of the memory block being tested.
//! If run in CRC mode, the calculated CRC value is compared with golden CRC
//! and status is updated.
//!
//! \return None.
//
//*****************************************************************************
static inline void
BGCRC_setGoldenCRCValue(uint32_t base, uint32_t crcVal)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));

    EALLOW;
    HWREG(base + BGCRC_O_GOLDEN) = crcVal;
    EDIS;
}

//*****************************************************************************
//
//! Starts the module operation
//!
//! \param base is the BGCRC module base address
//!
//! This function starts the module operation. Calling this function during
//! the CRC calculation will reset and re-start the CRC calculation.
//! This also resets the watchdog timer.
//!
//! \return None.
//
//*****************************************************************************
static inline void
BGCRC_start(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));

    EALLOW;
    HWREG(base + BGCRC_O_EN) = BGCRC_START_KEY;
    EDIS;
}

//*****************************************************************************
//
//! Halts the module operation
//!
//! \param base is the BGCRC module base address
//!
//! This function halts the module operation. This function does not stall the
//! watchdog timer.
//!
//! \return None.
//
//*****************************************************************************
static inline void
BGCRC_halt(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));

    EALLOW;
    HWREG(base + BGCRC_O_CTRL2) =
                (HWREG(base + BGCRC_O_CTRL2) & ~BGCRC_CTRL2_TEST_HALT_M) |
                (BGCRC_HALT_KEY << BGCRC_CTRL2_TEST_HALT_S);
    EDIS;
}

//*****************************************************************************
//
//! Resumes the module operation
//!
//! \param base is the BGCRC module base address
//!
//! This function resumes the module operation. The CRC calculation will
//! continue/resume from where it was halted
//!
//! \return None.
//
//*****************************************************************************
static inline void
BGCRC_resume(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));

    EALLOW;
    HWREG(base + BGCRC_O_CTRL2) &= ~BGCRC_CTRL2_TEST_HALT_M;
    EDIS;
}

//*****************************************************************************
//
//! Gets the running status of the module
//!
//! \param base is the BGCRC module base address
//!
//! This function returns whether the module is in ACTIVE or IDLE state
//!
//! \return \b BGCRC_ACTIVE if CRC module is active,
//!         \b BGCRC_IDLE if CRC module is idle
//
//*****************************************************************************
static inline uint32_t
BGCRC_getRunStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));

    return(HWREG(base + BGCRC_O_EN) & BGCRC_EN_RUN_STS);
}
//*****************************************************************************
//
//! Sets the seed value for CRC calculations
//!
//! \param base is the BGCRC module base address
//! \param seed is the seed value to be set
//!
//! This function sets the seed value for the CRC calculations
//!
//! \return None
//
//*****************************************************************************
static inline void
BGCRC_setSeedValue(uint32_t base, uint32_t seed)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));

    EALLOW;
    HWREG(base + BGCRC_O_SEED) = seed;
    EDIS;
}

//*****************************************************************************
//
//! Gets the calculated CRC value
//!
//! \param base is the BGCRC module base address
//!
//! This function returns the calculated CRC value
//!
//! \return 32-bit CRC result
//
//*****************************************************************************
static inline uint32_t
BGCRC_getResult(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));

    return(HWREG(base + BGCRC_O_RESULT));
}

//*****************************************************************************
//
//! Gets the current address
//!
//! \param base is the BGCRC module base address
//!
//! This function returns the current address  from where the data is fetched
//!
//! \return 32-bit address
//
//*****************************************************************************
static inline uint32_t
BGCRC_getCurrentAddress(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));

    return(HWREG(base + BGCRC_O_CURR_ADDR));
}

//*****************************************************************************
//
//! Locks the register configuration
//!
//! \param base is the BGCRC module base address
//! \param regs is the configuration registers to be locked
//!
//! This function locks the register configuration. The register once
//! locked, cannot be updated until the lock is removed
//!
//! The \e regs parameter is the logical OR of any of the following:
//!
//!  - \b BGCRC_REG_EN
//!  - \b BGCRC_REG_CTRL1
//!  - \b BGCRC_REG_CTRL2
//!  - \b BGCRC_REG_START_ADDR
//!  - \b BGCRC_REG_SEED
//!  - \b BGCRC_REG_GOLDEN
//!  - \b BGCRC_REG_WD_CFG
//!  - \b BGCRC_REG_WD_MIN
//!  - \b BGCRC_REG_WD_MAX
//!  - \b BGCRC_REG_NMIFRC
//!  - \b BGCRC_REG_INTEN
//!  - \b BGCRC_REG_INTFRC
//!  - \b BGCRC_REG_ALL
//!
//! \return None
//
//*****************************************************************************
static inline void
BGCRC_lockRegister(uint32_t base, uint32_t regs)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));

    EALLOW;
    HWREG(base + BGCRC_O_LOCK) |= regs;
    EDIS;
}

//*****************************************************************************
//
//! Unlocks the register configuration
//!
//! \param base is the BGCRC module base address
//! \param regs is the configuration registers to be unlocked
//!
//! This function unlocks the register configuration.
//!
//! The \e regs parameter has the same definition as the \e regs parameter
//! to BGCRC_lockRegister().
//!
//! \return None
//
//*****************************************************************************
static inline void
BGCRC_unlockRegister(uint32_t base, uint32_t regs)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));

    EALLOW;
    HWREG(base + BGCRC_O_LOCK) &= ~regs;
    EDIS;
}

//*****************************************************************************
//
//! Commits the register configuration
//!
//! \param base is the BGCRC module base address
//! \param regs is the configuration registers to be unlocked
//!
//! This function commits the register configuration. Once configuration is
//! committed, only reset can change the configuration.
//!
//! The \e regs parameter has the same definition as the \e regs parameter
//! to BGCRC_lockRegister().
//!
//! \return None
//
//*****************************************************************************
static inline void
BGCRC_commitRegisterLock(uint32_t base, uint32_t regs)
{
    //
    // Check the arguments.
    //
    ASSERT(BGCRC_isBaseValid(base));

    EALLOW;
    HWREG(base + BGCRC_O_COMMIT) |= regs;
    EDIS;
}

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

#endif // BGCRC_H
