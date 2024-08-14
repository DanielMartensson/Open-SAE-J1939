//###########################################################################
//
// FILE:   dcc.h
//
// TITLE:  C28x DCC driver.
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

#ifndef DCC_H
#define DCC_H

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
//! \addtogroup dcc_api DCC
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_dcc.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "cpu.h"
#include "debug.h"

//
// Minimum and maximum values of valid seed value
//
#define DCC_VALIDSEED_MIN 4U
#define DCC_VALIDSEED_MAX 0xFFFFU

//
// Minimum and maximum values of counter0/1 seed value
//
#define DCC_COUNTSEED_MIN 1U
#define DCC_COUNTSEED_MAX 0xFFFFFU

//
// The reset value required to start or enable specific DCC operations
//
#define DCC_ENABLE_VALUE     (0xAU)

//
// The reset value required to stop or disable specific DCC operations
//
#define DCC_DISABLE_VALUE    (0x5U)

//
// A 16-bit register mask
//
#define DCC_REG_WORD_MASK    (0xFFFFU)

//
// A 7-bit register mask
//
#define DCC_REG_BYTE_MASK    (0x7FU)

//
// A mask for the DCC counter seed registers
//
#define DCC_SEED_REG_MASK    (0xFFF0U)

//
// A mask for the DCC counter seed value
//
#define DCC_SEED_CNT_MASK    (0xF0000U)

//*****************************************************************************
//
//! The following are defines for the mode parameter of the
//! DCC_enableSingleShotMode() function.
//
//*****************************************************************************
typedef enum
{
    //! Use to stop counting when counter0 and valid0 both reach zero
    DCC_MODE_COUNTER_ZERO = 0xA00U,

    //! Use to stop counting when counter1 reaches zero
    DCC_MODE_COUNTER_ONE  = 0xB00U
} DCC_SingleShotMode;


//*****************************************************************************
//
//! The following are defines for the source parameter of the
//! DCC_setCounter1ClkSource() function.
//
//*****************************************************************************
typedef enum
{
    DCC_COUNT1SRC_PLL           = 0x0U, //!< PLLRAWCLK Clock Out Source
    DCC_COUNT1SRC_AUXPLL        = 0x1U, //!< AUXPLL Clock Out Source
    DCC_COUNT1SRC_INTOSC1       = 0x2U, //!< Internal Oscillator1 Clock Source
    DCC_COUNT1SRC_INTOSC2       = 0x3U, //!< Internal Oscillator2 Clock Source
    DCC_COUNT1SRC_CMCLK         = 0x5U, //!< CM Clock Source
    DCC_COUNT1SRC_CPU1SYSCLK    = 0x6U, //!< CPU1 SYSCLK (System Clock) Source
    DCC_COUNT1SRC_ENETRXCLK     = 0x7U, //!< etherNET RXCLK Clock Source
    DCC_COUNT1SRC_CPU2SYSCLK    = 0x8U, //!< CPU2 SYSCLK (System Clock) Source
    DCC_COUNT1SRC_CROSSBAR      = 0x9U, //!< Input XBAR Clock Source
    DCC_COUNT1SRC_AUXCLKIN      = 0xAU, //!< AUXCLK (Auxiliary Clock) Source
    DCC_COUNT1SRC_EPWM          = 0xBU, //!< EPWM Clock Source
    DCC_COUNT1SRC_LSPCLK        = 0xCU, //!< LSPCLK (Low Speed Clock) Source
    DCC_COUNT1SRC_ECATMII0RXCLK = 0xDU, //!< MII0 RXCLK (etherCAT) Clock Source
    DCC_COUNT1SRC_WDCLK         = 0xEU, //!< Watch Dog Clock Source
    DCC_COUNT1SRC_CANX          = 0xFU, //!< CANxBIT Clock Source
    DCC_COUNT1SRC_ECATMII1RXCLK   = 0x17U, //!< MII1 RXCLK (etherCAT) Clk Src
} DCC_Count1ClockSource;

//*****************************************************************************
//
//! The following are defines for the source parameter of the
//! DCC_setCounter0ClkSource() function.
//
//*****************************************************************************
typedef enum
{
    DCC_COUNT0SRC_XTAL       = 0x0U,    //!< XTAL Clock Source
    DCC_COUNT0SRC_INTOSC1    = 0x1U,    //!< Internal Oscillator1 Clock Source
    DCC_COUNT0SRC_INTOSC2    = 0x2U,    //!< Internal Oscillator2 Clock Source
    DCC_COUNT0SRC_CPU1SYSCLK = 0x5U,    //!< CPU1 SYSCLK (System Clock) Source
    DCC_COUNT0SRC_CPU2SYSCLK = 0x6U,    //!< CPU2 SYSCLK (System Clock) Source
    DCC_COUNT0SRC_XBAR       = 0xCU,    //!< Input XBAR Clock Source
} DCC_Count0ClockSource;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! \internal
//! Checks DCC base address.
//!
//! \param base specifies the DCC module base address.
//!
//! This function determines if an DCC module base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
DCC_isBaseValid(uint32_t base)
{
    return(
           (base == DCC0_BASE) ||
           (base == DCC1_BASE) ||
           (base == DCC2_BASE)
          );
}
#endif

//*****************************************************************************
//
//! Enables the DCC module.
//!
//! \param base is the DCC module base address
//!
//! This function starts the DCC counter operation.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_enableModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Set DCC enable bit field.
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                  ~(DCC_GCTRL_DCCENA_M)) | DCC_ENABLE_VALUE;

    EDIS;
}

//*****************************************************************************
//
//! Disable the DCC module.
//!
//! \param base is the DCC module base address
//!
//! This function stops the DCC counter operation.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_disableModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Reset DCC enable bit field.
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                  ~(DCC_GCTRL_DCCENA_M)) | DCC_DISABLE_VALUE;

    EDIS;
}

//*****************************************************************************
//
//! Enable DCC Error Signal
//!
//! \param base is the DCC module base address
//!
//! This function enables the error signal interrupt.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_enableErrorSignal(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Enable the error signal
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                  ~(DCC_GCTRL_ERRENA_M)) |
                                  (DCC_ENABLE_VALUE << 4U);

    EDIS;
}

//*****************************************************************************
//
//! Enable DCC Done Signal
//!
//! \param base is the DCC module base address
//!
//! This function enables the done signal interrupt.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_enableDoneSignal(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Enable the done interrupt signal
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                  ~(DCC_GCTRL_DONEENA_M)) |
                                  (DCC_ENABLE_VALUE << 12U);

    EDIS;
}

//*****************************************************************************
//
//! Disable DCC Error Signal
//!
//! \param base is the DCC module base address
//!
//! This function disables the error signal interrupt.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_disableErrorSignal(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Disable the error signal
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                  ~(DCC_GCTRL_ERRENA_M)) |
                                 (DCC_DISABLE_VALUE << 4U);

    EDIS;
}

//*****************************************************************************
//
//! Disable DCC Done Signal
//!
//! \param base is the DCC module base address
//!
//! This function disables the done signal interrupt.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_disableDoneSignal(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Disable the done interrupt signal
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                  ~(DCC_GCTRL_DONEENA_M)) |
                                 (DCC_DISABLE_VALUE << DCC_GCTRL_DONEENA_S);

    EDIS;
}

//*****************************************************************************
//
//! Enable DCC Single-Shot Mode
//!
//! \param base is the DCC module base address
//! \param mode is the selected Single-Shot operation mode
//!
//! This function enables the single-shot mode and sets the operation mode.
//!
//! The \e mode parameter can have one of two values:
//! - \b DCC_MODE_COUNTER_ZERO - Stops counting when counter0 and valid0 both
//!   reach zero
//! - \b DCC_MODE_COUNTER_ONE  - Stops counting when counter1 reaches zero
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_enableSingleShotMode(uint32_t base, DCC_SingleShotMode mode)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Set Single-Shot mode value to the appropriate register
    //
    if(mode == DCC_MODE_COUNTER_ZERO)
    {
        EALLOW;

        HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                      ~(DCC_GCTRL_SINGLESHOT_M)) |
                                     (uint16_t)DCC_MODE_COUNTER_ZERO;

        EDIS;
    }
    else
    {
        EALLOW;

        HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                      ~(DCC_GCTRL_SINGLESHOT_M)) |
                                     (uint16_t)DCC_MODE_COUNTER_ONE;

        EDIS;
    }
}

//*****************************************************************************
//
//! Disable DCC Single-Shot Mode
//!
//! \param base is the DCC module base address
//!
//! This function disables the DCC Single-Shot operation mode
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_disableSingleShotMode(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Reset Single-Shot enable bit field.
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                  ~(DCC_GCTRL_SINGLESHOT_M)) |
                                 (DCC_DISABLE_VALUE << DCC_GCTRL_SINGLESHOT_S);

    EDIS;
}

//*****************************************************************************
//
//! Get Error Flag Status
//!
//! \param base is the DCC module base address
//!
//! This function gets the error flag status.
//!
//! \return Returns \b true if an error has occurred, \b false if no errors
//! have occurred.
//
//*****************************************************************************
static inline bool
DCC_getErrorStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Get the error flag
    //
    return((bool)((HWREGH(base + DCC_O_STATUS) & DCC_STATUS_ERR) ==
                  DCC_STATUS_ERR));
}

//*****************************************************************************
//
//! Get Single-Shot Done Flag Status
//!
//! \param base is the DCC module base address
//!
//! This function gets the single-shot done flag status.
//!
//! \return Returns \b true if single-shot mode has completed, \b false if
//! single-shot mode has not completed.
//
//*****************************************************************************
static inline bool
DCC_getSingleShotStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Read the done flag
    //
    return((bool)((HWREGH(base + DCC_O_STATUS) & DCC_STATUS_DONE) ==
                  DCC_STATUS_DONE));
}

//*****************************************************************************
//
//! Clear Error Status Flag
//!
//! \param base is the DCC module base address
//!
//! This function clears the DCC error status flag.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_clearErrorFlag(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Clear error status flag
    //
    EALLOW;

    HWREGH(base + DCC_O_STATUS) |= DCC_STATUS_ERR;

    EDIS;
}

//*****************************************************************************
//
//! Clear Single-Shot Done Status Flag
//!
//! \param base is the DCC module base address
//!
//! This function clears the DCC single-shot done status flag.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_clearDoneFlag(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Clear done status flag
    //
    EALLOW;

    HWREGH(base + DCC_O_STATUS) |= DCC_STATUS_DONE;

    EDIS;
}

//*****************************************************************************
//
//! Get Current Value of Counter 0
//!
//! \param base is the DCC module base address
//!
//! This function gets current value of counter 0.
//!
//! \note Reads of the counter value may not be exact since the read operation
//! is synchronized to the vbus clock.
//!
//! \return Returns the current value of counter 0.
//
//*****************************************************************************
static inline uint32_t
DCC_getCounter0Value(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Get the current counter 0 value
    //
    return(HWREG(base + DCC_O_CNT0));
}

//*****************************************************************************
//
//! Get Current Value of the Valid Duration Counter for Counter 0
//!
//! \param base is the DCC module base address
//!
//! This function gets current value of the valid duration counter for
//! counter 0.
//!
//! \note Reads of the counter value may not be exact since the read operation
//! is synchronized to the vbus clock.
//!
//! \return Returns the current value of the valid duration counter.
//
//*****************************************************************************
static inline uint16_t
DCC_getValidCounter0Value(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Get the current valid duration counter value
    //
    return(HWREGH(base + DCC_O_VALID0));
}

//*****************************************************************************
//
//! Get Current Value of Counter 1
//!
//! \param base is the DCC module base address
//!
//! This function gets current value of counter 1.
//!
//! \note Reads of the counter value may not be exact since the read operation
//! is synchronized to the vbus clock.
//!
//! \return Returns the current value of counter 1.
//
//*****************************************************************************
static inline uint32_t
DCC_getCounter1Value(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Get the current counter 1 value
    //
    return(HWREG(base + DCC_O_CNT1));
}

//*****************************************************************************
//
//! Set Counter 1 Clock Source
//!
//! \param base is the DCC module base address
//! \param source is the selected clock source for counter 1
//!
//! This function sets the counter 1 clock source.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_setCounter1ClkSource(uint32_t base, DCC_Count1ClockSource source)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    //  Set the specified clock source
    //
    EALLOW;

    //
    //  DCC Clk source is of 5bits DCCCLKSRC1[4:0]
    //
    HWREGH(base + DCC_O_CLKSRC1) = (HWREGH(base + DCC_O_CLKSRC1) &
                                   (DCC_REG_BYTE_MASK << 5U)) |
                                   ((DCC_ENABLE_VALUE << 12U) |
                                   (uint16_t)source);

    EDIS;
}

//*****************************************************************************
//
//! Set Counter 0 Clock Source
//!
//! \param base is the DCC module base address
//! \param source is the selected clock source for counter 0
//!
//! This function sets the counter 0 clock source.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_setCounter0ClkSource(uint32_t base, DCC_Count0ClockSource source)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    //  Set the specified clock source
    //
    EALLOW;

    //
    //  DCC Clk source is of 5bits DCCCLKSRC0[4:0]
    //
    HWREGH(base + DCC_O_CLKSRC0) = (HWREGH(base + DCC_O_CLKSRC0) &
                                   (DCC_REG_BYTE_MASK << 5U)) |
                                   ((DCC_ENABLE_VALUE << 12U) |
                                   (uint16_t)source);

    EDIS;
}

//*****************************************************************************
//
//! Get Counter 1 Clock Source
//!
//! \param base is the DCC module base address
//!
//! This function gets the counter 1 clock source.
//!
//! \return Returns Counter 1 clock source from the enum DCC_Count1ClockSource
//
//*****************************************************************************
static inline uint16_t
DCC_getCounter1ClkSource(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    //  Get the specified clock source
    //
    return(HWREGH(base + DCC_O_CLKSRC1) & DCC_CLKSRC1_CLKSRC1_M);
}

//*****************************************************************************
//
//! Get Counter 0 Clock Source
//!
//! \param base is the DCC module base address
//!
//! This function gets the counter 0 clock source.
//!
//! \return Returns Counter 0 clock source from the enum DCC_Count0ClockSource
//
//*****************************************************************************
static inline uint16_t
DCC_getCounter0ClkSource(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    //  Get the specified clock source
    //
    return(HWREGH(base + DCC_O_CLKSRC0) & DCC_CLKSRC0_CLKSRC0_M);
}

//*****************************************************************************
//
//! Set the seed values
//!
//! \param base is the DCC module base address
//! \param counter0 sets the seed value that gets loaded into Counter 0
//! \param validCounter0 sets the seed value that gets loaded into the valid
//!  duration counter for Counter 0
//! \param counter1 sets the seed value that gets loaded into Counter 1
//!
//! This function sets the seed values for Counter 0, Valid Duration Counter 0,
//! and Counter 1.
//!
//! \note
//! -# Operating DCC with '0' set as the seed value for Counter 0, Valid
//! Duration Counter 0, and/or Counter 1 will result in undefined operation.
//! -# The Valid Duration Counter 0 is designed to be at least four cycles
//! wide and shouldn't be programmed with a value less than '4'.
//!
//! \return None.
//
//*****************************************************************************


static inline void
DCC_setCounterSeeds(uint32_t base, uint32_t counter0, uint32_t validCounter0,
                    uint32_t counter1)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));
    ASSERT(validCounter0 >= DCC_VALIDSEED_MIN);
    ASSERT(counter0 >= DCC_COUNTSEED_MIN);
    ASSERT(counter1 >= DCC_COUNTSEED_MIN);
    ASSERT(validCounter0 <= DCC_VALIDSEED_MAX);
    ASSERT(counter0 <= DCC_COUNTSEED_MAX);
    ASSERT(counter1 <= DCC_COUNTSEED_MAX);

    EALLOW;

    //
    // Set Counter 0 Seed
    //
    HWREGH(base + DCC_O_CNTSEED0) = counter0 & DCC_REG_WORD_MASK;
    HWREGH(base + DCC_O_CNTSEED0 + 2U) = (HWREGH(base + DCC_O_CNTSEED0 + 2U) &
                                          DCC_SEED_REG_MASK) |
                                         ((uint32_t)(counter0 &
                                          DCC_SEED_CNT_MASK) >> 16U);

    //
    // Set Valid Duration Counter 0 Seed
    //
    HWREGH(base + DCC_O_VALIDSEED0) = validCounter0;

    //
    // Set Counter 1 Seed
    //
    HWREGH(base + DCC_O_CNTSEED1) = counter1 & DCC_REG_WORD_MASK;
    HWREGH(base + DCC_O_CNTSEED1 + 2U) = (HWREGH(base + DCC_O_CNTSEED1 + 2U) &
                                          DCC_SEED_REG_MASK) |
                                         ((uint32_t)(counter1 &
                                          DCC_SEED_CNT_MASK) >> 16U);

    EDIS;
}


//*****************************************************************************
//
//! Verify the frequency of a clock source using a reference clock
//!
//! \param base is the DCC module base address
//! \param clock1 is the clock source to be verified
//! \param freq1 is the frequency of the clock source to be verified
//! \param clock0 is the reference clock
//! \param freq0 is the frequency of the reference clock
//! \param tolerance is the DCC error tolerance in percentage
//! \param freqerr is the allowable Frequency Tolerance in percentage
//! \param freq_sysclk is the frequency of the system clock
//!
//! This function verifies the frequency of the specified clock using a
//! reference clock.
//!
//! \note Make sure that the frequency of all the clocks are in the same unit
//!
//! \return Returns \b true if the actual frequency of clock1 is matching
//! freq1, \b false otherwise
//
//*****************************************************************************
extern bool
DCC_verifyClockFrequency(uint32_t base,
                         DCC_Count1ClockSource clock1,
                         float32_t freq1,
                         DCC_Count0ClockSource clock0,
                         float32_t freq0,
                         float32_t tolerance,
                         float32_t freqerr,
                         float32_t freq_sysclk);

//*****************************************************************************
//
//! Measure the frequency of a clock source using a reference clock
//!
//! \param base is the DCC module base address
//! \param clock1 is the clock source to be measured
//! \param clock0 is the reference clock
//! \param freq0 is the frequency of the reference clock
//! \param tolerance is the DCC error tolerance in percentage
//! \param freqerr is the allowable Frequency Tolerance in percentage
//! \param freq_sysclk is the frequency of the system clock
//!
//! This function measures the frequency of the specified clock using a
//! reference clock.
//!
//! \note Make sure that the frequency of all the clocks are in the same unit
//!
//! \return Measured clock frequency
//
//*****************************************************************************
extern float32_t
DCC_measureClockFrequency(uint32_t base,
                          DCC_Count1ClockSource clock1,
                          DCC_Count0ClockSource clock0,
                          float32_t freq0,
                          float32_t tolerance,
                          float32_t freqerr,
                          float32_t freq_sysclk);

//*****************************************************************************
//
//! Continuously monitor the frequency of a clock source using a reference clock
//!
//! \param base is the DCC module base address
//! \param clock1 is the clock source to be verified
//! \param freq1 is the frequency of the clock source to be verified
//! \param clock0 is the reference clock
//! \param freq0 is the frequency of the reference clock
//! \param tolerance is the DCC error tolerance in percentage
//! \param freqerr is the allowable Frequency Tolerance in percentage
//! \param freq_sysclk is the frequency of the system clock
//!
//! This function continuously monitors the frequency of the specified clock
//! using a reference clock. An error signal is generated if the frequency
//! ratio is altered.
//!
//! \note Make sure that the frequency of all the clocks are in the same unit
//!
//! \note This function does not enable the DCC interrupt. The user application
//! must enable it to trigger an intetrupt when an error is detected.
//!
//! \return Returns None
//
//*****************************************************************************
extern void
DCC_continuousMonitor(uint32_t base,
                      DCC_Count1ClockSource clock1,
                      float32_t freq1,
                      DCC_Count0ClockSource clock0,
                      float32_t freq0,
                      float32_t tolerance,
                      float32_t freqerr,
                      float32_t freq_sysclk);
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

#endif // DCC_H
