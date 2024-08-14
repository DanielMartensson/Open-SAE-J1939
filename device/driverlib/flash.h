//###########################################################################
//
// FILE:   flash.h
//
// TITLE:  C28x Flash driver.
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

#ifndef FLASH_H
#define FLASH_H

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
//! \addtogroup flash_api Flash
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_flash.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "cpu.h"
#include "debug.h"
#include "ipc.h"

#ifndef __cplusplus
#pragma CODE_SECTION(Flash_setBankPowerMode, ".TI.ramfunc");
#pragma CODE_SECTION(Flash_setPumpPowerMode, ".TI.ramfunc");
#pragma CODE_SECTION(Flash_disableCache, ".TI.ramfunc");
#pragma CODE_SECTION(Flash_disablePrefetch, ".TI.ramfunc");
#pragma CODE_SECTION(Flash_setWaitstates, ".TI.ramfunc");
#pragma CODE_SECTION(Flash_enableCache, ".TI.ramfunc");
#pragma CODE_SECTION(Flash_enablePrefetch, ".TI.ramfunc");
#pragma CODE_SECTION(Flash_enableECC, ".TI.ramfunc");
#endif

//*****************************************************************************
//
//! Values that can be passed to Flash_setBankPowerMode() as the bank parameter
//
//*****************************************************************************
typedef enum
{
    FLASH_BANK = 0x0   //!< Bank
} Flash_BankNumber;

//*****************************************************************************
//
//! Values that can be passed to Flash_claimPumpSemaphore() in order to claim
//! the pump semaphore.
//
//*****************************************************************************
typedef enum
{
    FLASH_CPU1_WRAPPER = 0x2, //!< CPU1 Wrapper
    FLASH_CPU2_WRAPPER = 0x1  //!< CPU2 Wrapper
}Flash_PumpOwnership;

//*****************************************************************************
//
//! Values that can be passed to Flash_setBankPowerMode() as the powerMode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    FLASH_BANK_PWR_SLEEP    = 0x0, //!< Sleep fallback mode
    FLASH_BANK_PWR_STANDBY  = 0x1, //!< Standby fallback mode
    FLASH_BANK_PWR_ACTIVE   = 0x3  //!< Active fallback mode
} Flash_BankPowerMode;

//*****************************************************************************
//
//! Values that can be passed to Flash_setPumpPowerMode() as the powerMode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    FLASH_PUMP_PWR_SLEEP    = 0x0, //!< Sleep fallback mode
    FLASH_PUMP_PWR_ACTIVE   = 0x1  //!< Active fallback mode
} Flash_PumpPowerMode;

//*****************************************************************************
//
//! Type that correspond to values returned from Flash_getLowErrorStatus() and
//! Flash_getHighErrorStatus() determining the error status code.
//
//*****************************************************************************
typedef enum
{
    FLASH_NO_ERR       = 0x0, //!< No error
    FLASH_FAIL_0       = 0x1, //!< Fail on 0
    FLASH_FAIL_1       = 0x2, //!< Fail on 1
    FLASH_UNC_ERR      = 0x4  //!< Uncorrectable error
} Flash_ErrorStatus;

//*****************************************************************************
//
//! Values that can be returned from Flash_getLowErrorType() and
//! Flash_getHighErrorType() determining the error type.
//
//*****************************************************************************
typedef enum
{
    FLASH_DATA_ERR      = 0x0, //!< Data error
    FLASH_ECC_ERR       = 0x1  //!< ECC error
} Flash_ErrorType;

//*****************************************************************************
//
//! Values that can be returned from Flash_getECCTestSingleBitErrorType().
//
//*****************************************************************************
typedef enum
{
    FLASH_DATA_BITS          = 0x0, //!< Data bits
    FLASH_CHECK_BITS         = 0x1  //!< ECC bits
} Flash_SingleBitErrorIndicator;

//*****************************************************************************
//
// Values that can be passed to Flash_clearLowErrorStatus and
// Flash_clearHighErrorStatus.
//
//*****************************************************************************
#define    FLASH_FAIL_0_CLR       0x1 //!< Fail-0 clear
#define    FLASH_FAIL_1_CLR       0x2 //!< Fail-1 clear
#define    FLASH_UNC_ERR_CLR      0x4 //!< Uncorrectable error Clear

//*****************************************************************************
//
// Values that can be returned from Flash_getInterruptFlag and
// Flash_getECCTestStatus.
//
//*****************************************************************************
#define    FLASH_NO_ERROR         0x0 //!< No error
#define    FLASH_SINGLE_ERROR     0x1 //!< Single bit error
#define    FLASH_UNC_ERROR        0x2 //!< Uncorrectable error

//*****************************************************************************
//
// Delay instruction that allows for register configuration to complete.
//
//*****************************************************************************
#define    FLASH_DELAY_CONFIG     __asm(" RPT #7 || NOP")

//*****************************************************************************
//
// Key value for claiming the pump semaphore.
//
//*****************************************************************************
#define FLASH_PUMP_KEY                  0x5A5A0000UL //!< Pump semaphore key

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! \internal
//! Checks a flash wrapper base address for the control registers.
//!
//! \param ctrlBase is the base address of the flash wrapper control registers.
//!
//! This function determines if a flash wrapper control base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
Flash_isCtrlBaseValid(uint32_t ctrlBase)
{
    return((ctrlBase == FLASH0CTRL_BASE));
}
#endif

//*****************************************************************************
//
//! \internal
//! Checks a flash wrapper base address for the ECC registers.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! This function determines if a flash wrapper ECC base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
Flash_isECCBaseValid(uint32_t eccBase)
{
    return((eccBase == FLASH0ECC_BASE));
}
#endif

//*****************************************************************************
//
//! Sets the random read wait state amount.
//!
//! \param ctrlBase is the base address of the flash wrapper control registers.
//! \param waitstates is the wait-state value.
//!
//! This function sets the number of wait states for a flash read access. The
//! \e waitstates parameter is a number between 0 and 15. It is \b important
//! to look at your device's datasheet for information about what the required
//! minimum flash wait-state is for your selected SYSCLK frequency.
//!
//! By default the wait state amount is configured to the maximum 15.
//!
//! \return None.
//
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#endif
static inline void
Flash_setWaitstates(uint32_t ctrlBase, uint16_t waitstates)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isCtrlBaseValid(ctrlBase));

    //
    // waitstates is 4 bits wide.
    //
    ASSERT(waitstates <= 0xFU);

    EALLOW;
    //
    // Write flash read wait-state amount to appropriate register.
    //
    HWREG(ctrlBase + FLASH_O_FRDCNTL) =
        (HWREG(ctrlBase + FLASH_O_FRDCNTL) &
         ~(uint32_t)FLASH_FRDCNTL_RWAIT_M) |
         ((uint32_t)waitstates << FLASH_FRDCNTL_RWAIT_S);
    EDIS;
}

//*****************************************************************************
//
//! Sets the fallback power mode of a flash bank.
//!
//! \param ctrlBase is the base address of the flash wrapper registers.
//! \param bank is the flash bank that is being configured.
//! \param powerMode is the power mode to be entered.
//!
//! This function sets the fallback power mode of the flash bank specified by
//! them \e bank parameter. The power mode is specified by the \e powerMode
//! parameter with one of the following values:
//!
//! - \b FLASH_BANK_PWR_SLEEP - Sense amplifiers and sense reference disabled.
//! - \b FLASH_BANK_PWR_STANDBY - Sense amplifiers disabled but sense reference
//!   enabled.
//! - \b FLASH_BANK_PWR_ACTIVE - Sense amplifiers and sense reference enabled.
//!
//!
//! Note: There is only one Flash_BankNumber value on this device (FLASH_BANK).
//!
//! \return None.
//
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#endif
static inline void
Flash_setBankPowerMode(uint32_t ctrlBase, Flash_BankNumber bank,
                       Flash_BankPowerMode powerMode)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isCtrlBaseValid(ctrlBase));

    EALLOW;

    //
    // Write the power mode to the appropriate register.
    //
    HWREG(ctrlBase + FLASH_O_FBFALLBACK) =
        (HWREG(ctrlBase + FLASH_O_FBFALLBACK) &
         ~((FLASH_FBFALLBACK_BNKPWR0_M) << ((uint32_t)bank * 2U))) |
        ((uint32_t)powerMode << ((uint32_t)bank * 2U));
    EDIS;
}

//*****************************************************************************
//
//! Sets the fallback power mode of the charge pump.
//!
//! \param ctrlBase is the base address of the flash wrapper control registers.
//! \param powerMode is the power mode to be entered.
//!
//! This function sets the fallback power mode flash charge pump.
//!
//! - \b FLASH_PUMP_PWR_SLEEP - All circuits disabled.
//! - \b FLASH_PUMP_PWR_ACTIVE - All pump circuits active.
//!
//! \return None.
//
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#endif
static inline void
Flash_setPumpPowerMode(uint32_t ctrlBase, Flash_PumpPowerMode powerMode)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isCtrlBaseValid(ctrlBase));

    EALLOW;

    //
    // Write the power mode to the appropriate register.
    //
    HWREG(ctrlBase + FLASH_O_FPAC1) =
        (HWREG(ctrlBase + FLASH_O_FPAC1) &
        ~(uint32_t)FLASH_FPAC1_PMPPWR) | (uint32_t)powerMode;
    EDIS;
}

//*****************************************************************************
//
//! Enables prefetch mechanism.
//!
//! \param ctrlBase is the base address of the flash wrapper control registers.
//!
//! \return None.
//
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#endif
static inline void
Flash_enablePrefetch(uint32_t ctrlBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isCtrlBaseValid(ctrlBase));

    EALLOW;

    //
    // Set the prefetch enable bit.
    //
    HWREG(ctrlBase + FLASH_O_FRD_INTF_CTRL) |=
            FLASH_FRD_INTF_CTRL_PREFETCH_EN;
    EDIS;
}

//*****************************************************************************
//
//! Disables prefetch mechanism.
//!
//! \param ctrlBase is the base address of the flash wrapper control registers.
//!
//! \return None.
//
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#endif
static inline void
Flash_disablePrefetch(uint32_t ctrlBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isCtrlBaseValid(ctrlBase));

    EALLOW;

    //
    // Clear the prefetch enable bit.
    //
    HWREG(ctrlBase + FLASH_O_FRD_INTF_CTRL) &=
            ~(uint32_t)FLASH_FRD_INTF_CTRL_PREFETCH_EN;
    EDIS;
}

//*****************************************************************************
//
//! Enables data cache.
//!
//! \param ctrlBase is the base address of the flash wrapper control registers.
//!
//! \return None.
//
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#endif
static inline void
Flash_enableCache(uint32_t ctrlBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isCtrlBaseValid(ctrlBase));

    EALLOW;

    //
    // Set the data cache enable bit.
    //
    HWREG(ctrlBase + FLASH_O_FRD_INTF_CTRL) |=
            FLASH_FRD_INTF_CTRL_DATA_CACHE_EN;
    EDIS;
}

//*****************************************************************************
//
//! Disables data cache.
//!
//! \param ctrlBase is the base address of the flash wrapper control registers.
//!
//! \return None.
//
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#endif
static inline void
Flash_disableCache(uint32_t ctrlBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isCtrlBaseValid(ctrlBase));

    EALLOW;

    //
    // Clear the data cache enable bit.
    //
    HWREG(ctrlBase + FLASH_O_FRD_INTF_CTRL) &=
            ~(uint32_t)FLASH_FRD_INTF_CTRL_DATA_CACHE_EN;
    EDIS;
}

//*****************************************************************************
//
//! Enables flash error correction code (ECC) protection.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! \return None.
//
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#endif
static inline void
Flash_enableECC(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    EALLOW;

    //
    // Write the key value 0xA to ECC_ENABLE register.
    //
    HWREG(eccBase + FLASH_O_ECC_ENABLE) =
        (HWREG(eccBase + FLASH_O_ECC_ENABLE) &
         ~(uint32_t)FLASH_ECC_ENABLE_ENABLE_M) | 0xAU;
    EDIS;
}

//*****************************************************************************
//
//! Disables flash error correction code (ECC) protection.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_disableECC(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    EALLOW;

    //
    // Clear ECC enable field with the one's complement of the key.
    //
    HWREG(eccBase + FLASH_O_ECC_ENABLE) =
        (HWREG(eccBase + FLASH_O_ECC_ENABLE) &
         ~(uint32_t)FLASH_ECC_ENABLE_ENABLE_M) | 0x5U;
    EDIS;
}



//*****************************************************************************
//
//! Sets the bank active grace period.
//!
//! \param ctrlBase is the base address of the flash wrapper control registers.
//! \param period is the starting count value for the BAGP down counter.
//!
//! This function sets the bank active grace period specified by the
//! \e period parameter. The \e period is a value between 0 and 255. This
//! value must be greater than 1 when the fallback mode is not Active.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_setBankActiveGracePeriod(uint32_t ctrlBase, uint32_t period)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isCtrlBaseValid(ctrlBase));
    ASSERT( period <= 255U );

    EALLOW;

    //
    // Write period to the BAGP of the FBAC register.
    //
    HWREG(ctrlBase + FLASH_O_FBAC) =
          (HWREG(ctrlBase + FLASH_O_FBAC) &
             ~(uint32_t)FLASH_FBAC_BAGP_M) | (period << FLASH_FBAC_BAGP_S);
    EDIS;
}

//*****************************************************************************
//
//! Sets the pump wake up time.
//!
//! \param ctrlBase is the base address of the flash wrapper control registers.
//! \param sysclkCycles is the number of SYSCLK cycles it takes for the pump
//! to wakeup.
//!
//! This function sets the wakeup time with \e sysclkCycles parameter.
//! The \e sysclkCycles is a value between 0 and 8190. When the charge pump
//! exits sleep power mode, it will take sysclkCycles to wakeup.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_setPumpWakeupTime(uint32_t ctrlBase, uint16_t sysclkCycles)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isCtrlBaseValid(ctrlBase));

    //
    // PSLEEP = sysclkCycles/2. PSLEEP maximum value is 4095(12 bits wide)
    //
    ASSERT( sysclkCycles <= 8190U );

    EALLOW;

    //
    // Write sysclkCycles/2 to PSLEEP of the FPAC1 register.
    //
    HWREG(ctrlBase + FLASH_O_FPAC1) =
          (HWREG(ctrlBase + FLASH_O_FPAC1) &
          ~(uint32_t)FLASH_FPAC1_PSLEEP_M) |
          (((uint32_t)sysclkCycles / (uint32_t)2) <<
          (uint32_t)FLASH_FPAC1_PSLEEP_S);
    EDIS;
}

//*****************************************************************************
//
//! Reads the bank active power state.
//!
//! \param ctrlBase is the base address of the flash wrapper control registers.
//! \param bank is the flash bank that is being used.
//!
//! \return Returns \b true if the Bank is in Active power state and \b false
//! otherwise.
//
//*****************************************************************************
static inline bool
Flash_isBankReady(uint32_t ctrlBase, Flash_BankNumber bank)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isCtrlBaseValid(ctrlBase));

    bool ready;
    uint32_t bitMask = (uint32_t)FLASH_FBPRDY_BANKRDY << (uint32_t)bank;
    //
    // Return the BANKXRDY bit in FBPRDY.
    //
    if((HWREG(ctrlBase + FLASH_O_FBPRDY) & bitMask) == bitMask)
    {
        ready = true;
    }
    else
    {
        ready = false;
    }
    return(ready);
}

//*****************************************************************************
//
//! Reads the pump active power state.
//!
//! \param ctrlBase is the base address of the flash wrapper control registers.
//!
//! \return Returns \b true if the Pump is in Active power state and \b false
//! otherwise.
//
//*****************************************************************************
static inline bool
Flash_isPumpReady(uint32_t ctrlBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isCtrlBaseValid(ctrlBase));

    bool ready;

    //
    // Return the PUMPRDY bit in FBPRDY.
    //
    if((HWREG(ctrlBase + FLASH_O_FBPRDY) &
        (uint32_t)FLASH_FBPRDY_PUMPRDY) == FLASH_FBPRDY_PUMPRDY)
    {
        ready = true;
    }
    else
    {
        ready = false;
    }
    return(ready);
}


//*****************************************************************************
//
//! Gets the single error address low.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! This function returns the 32-bit address of the single bit error that
//! occurred in the lower 64-bits of a 128-bit memory-aligned data. The
//! returned address is to that 64-bit aligned data.
//!
//! \return Returns the 32 bits of a 64-bit aligned address where a single bit
//! error occurred.
//
//*****************************************************************************
static inline uint32_t
Flash_getSingleBitErrorAddressLow(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    return(HWREG(eccBase + FLASH_O_SINGLE_ERR_ADDR_LOW));
}

//*****************************************************************************
//
//! Gets the single error address high.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! This function returns the 32-bit address of the single bit error that
//! occurred in the upper 64-bits of a 128-bit memory-aligned data. The
//! returned address is to that 64-bit aligned data.
//!
//! \return Returns the 32 bits of a 64-bit aligned address where a single bit
//! error occurred.
//
//*****************************************************************************
static inline uint32_t
Flash_getSingleBitErrorAddressHigh(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    return(HWREG(eccBase + FLASH_O_SINGLE_ERR_ADDR_HIGH));
}

//*****************************************************************************
//
//! Gets the uncorrectable error address low.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! This function returns the 32-bit address of the uncorrectable error that
//! occurred in the lower 64-bits of a 128-bit memory-aligned data. The
//! returned address is to that 64-bit aligned data.
//!
//! \return Returns the 32 bits of a 64-bit aligned address where an
//! uncorrectable error occurred.
//
//*****************************************************************************
static inline uint32_t
Flash_getUncorrectableErrorAddressLow(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    return(HWREG(eccBase + FLASH_O_UNC_ERR_ADDR_LOW));
}

//*****************************************************************************
//
//! Gets the uncorrectable error address high.
//!
//! \param eccBase is the base address of the flash wrapper ECC base.
//!
//! This function returns the 32-bit address of the uncorrectable error that
//! occurred in the upper 64-bits of a 128-bit memory-aligned data. The
//! returned address is to that 64-bit aligned data.
//!
//! \return Returns the 32 bits of a 64-bit aligned address where an
//! uncorrectable error occurred.
//
//*****************************************************************************
static inline uint32_t
Flash_getUncorrectableErrorAddressHigh(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    return(HWREG(eccBase + FLASH_O_UNC_ERR_ADDR_HIGH));
}

//*****************************************************************************
//
//! Gets the error status of the Lower 64-bits.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! This function returns the error status of the lower 64-bits of a 128-bit
//! aligned address.
//!
//! \return Returns value of the low error status bits which can be used with
//! Flash_ErrorStatus type.
//
//*****************************************************************************
static inline Flash_ErrorStatus
Flash_getLowErrorStatus(uint32_t eccBase)
{
    uint32_t errorStatus;
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    //
    // Get the Low Error Status bits
    //
    errorStatus = (HWREG(eccBase + FLASH_O_ERR_STATUS) & 0x7UL);
    return((Flash_ErrorStatus)errorStatus);
}

//*****************************************************************************
//
//! Gets the error status of the Upper 64-bits.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! This function returns the error status of the upper 64-bits of a 128-bit
//! aligned address.
//!
//! \return Returns value of the high error status bits which can be used with
//! Flash_ErrorStatus type.
//
//*****************************************************************************
static inline Flash_ErrorStatus
Flash_getHighErrorStatus(uint32_t eccBase)
{
    uint32_t errorStatus;
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    //
    // Get the High Error Status bits
    //
    errorStatus = ((HWREG(eccBase + FLASH_O_ERR_STATUS) >> 16U) & 0x7UL);
    return((Flash_ErrorStatus)errorStatus);
}

//*****************************************************************************
//
//! Gets the error position of the lower 64-bits for a single bit error.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! This function returns the error position of the lower 64-bits. If the
//! error type is FLASH_ECC_ERR, the position ranges from 0-7 else it ranges
//! from 0-63 for FLASH_DATA_ERR.
//!
//! \return Returns the position of the lower error bit.
//
//*****************************************************************************
static inline uint32_t
Flash_getLowErrorPosition(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    return((HWREG(eccBase + FLASH_O_ERR_POS) &
            (uint32_t)FLASH_ERR_POS_ERR_POS_L_M) >>
            FLASH_ERR_POS_ERR_POS_L_S);
}

//*****************************************************************************
//
//! Gets the error position of the upper 64-bits for a single bit error.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! This function returns the error position of the upper 64-bits. If the
//! error type is FLASH_ECC_ERR, the position ranges from 0-7 else it ranges
//! from 0-63 for FLASH_DATA_ERR.
//!
//! \return Returns the position of the upper error bit.
//
//*****************************************************************************
static inline uint32_t
Flash_getHighErrorPosition(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    return((HWREG(eccBase + FLASH_O_ERR_POS) &
            (uint32_t)FLASH_ERR_POS_ERR_POS_H_M) >>
            FLASH_ERR_POS_ERR_POS_H_S);
}

//*****************************************************************************
//
//! Clears the error position bit of the lower 64-bits for a single bit error.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! This function clears the error position bit of the lower 64-bits.
//!
//! \return None
//
//*****************************************************************************
static inline void
Flash_clearLowErrorPosition(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    EALLOW;
    HWREG(eccBase + FLASH_O_ERR_POS) &= ~(uint32_t)FLASH_ERR_POS_ERR_POS_L_M;
    EDIS;
}

//*****************************************************************************
//
//! Clears the error position of the upper 64-bits for a single bit error.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! This function clears the error position bit of the upper 64-bits.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_clearHighErrorPosition(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    EALLOW;
    HWREG(eccBase + FLASH_O_ERR_POS) &= ~(uint32_t)FLASH_ERR_POS_ERR_POS_H_M;
    EDIS;
}

//*****************************************************************************
//
//! Gets the error type of the lower 64-bits.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! This function returns the error type of the lower 64-bits. The error type
//! can be FLASH_ECC_ERR or FLASH_DATA_ERR.
//!
//! \return Returns the type of the lower 64-bit error.
//
//*****************************************************************************
static inline Flash_ErrorType
Flash_getLowErrorType(uint32_t eccBase)
{
    Flash_ErrorType errorType;
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    //
    // Check which error type.
    // If bit is 1 then ECC error, else it is a Data error.
    //
    if((HWREG(eccBase + FLASH_O_ERR_POS) & FLASH_ERR_POS_ERR_TYPE_L)
                                == FLASH_ERR_POS_ERR_TYPE_L)
    {
        errorType =  FLASH_ECC_ERR;
    }
    else
    {
        errorType =  FLASH_DATA_ERR;
    }

    return(errorType);
}

//*****************************************************************************
//
//! Gets the error type of the upper 64-bits.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! This function returns the error type of the upper 64-bits. The error type
//! can be FLASH_ECC_ERR or FLASH_DATA_ERR.
//!
//! \return Returns the type of the upper 64-bit error.
//
//*****************************************************************************
static inline Flash_ErrorType
Flash_getHighErrorType(uint32_t eccBase)
{
    Flash_ErrorType errorType;
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    //
    // Check which error type.
    // If bit is 1 then ECC error, else it is a Data error.
    //
    if((HWREG(eccBase + FLASH_O_ERR_POS) & FLASH_ERR_POS_ERR_TYPE_H)
                                == FLASH_ERR_POS_ERR_TYPE_H)
    {
        errorType =  FLASH_ECC_ERR;
    }
    else
    {
        errorType =  FLASH_DATA_ERR;
    }

    return(errorType);
}
//*****************************************************************************
//
//! Clears the errors status of the lower 64-bits.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//! \param errorStatus is the error status to clear. errorStatus is a uint16_t.
//! errorStatus is a bitwise OR of the following value:
//!
//! - \b FLASH_FAIL_0_CLR
//! - \b FLASH_FAIL_1_CLR
//! - \b FLASH_UNC_ERR_CLR
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_clearLowErrorStatus(uint32_t eccBase, uint16_t errorStatus)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));
    ASSERT( errorStatus <= 7U );

    EALLOW;
    HWREG(eccBase + FLASH_O_ERR_STATUS_CLR) |= ((uint32_t)errorStatus);
    EDIS;
}

//*****************************************************************************
//
//! Clears the errors status of the upper 64-bits.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//! \param errorStatus is the error status to clear. errorStatus is a uint16_t.
//! errorStatus is a bitwise OR of the following value:
//!
//! - \b FLASH_FAIL_0_CLR
//! - \b FLASH_FAIL_1_CLR
//! - \b FLASH_UNC_ERR_CLR
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_clearHighErrorStatus(uint32_t eccBase, uint16_t errorStatus)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));
    ASSERT( errorStatus <= 7U );

    EALLOW;
    HWREG(eccBase + FLASH_O_ERR_STATUS_CLR) |= ((uint32_t)errorStatus << 16U);
    EDIS;
}

//*****************************************************************************
//
//! Gets the single bit error count.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! \return Returns the single bit error count.
//
//*****************************************************************************
static inline uint32_t
Flash_getErrorCount(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    return(HWREG(eccBase + FLASH_O_ERR_CNT) &
            (uint32_t)FLASH_ERR_CNT_ERR_CNT_M);
}

//*****************************************************************************
//
//! Sets the single bit error threshold.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//! \param threshold is the single bit error threshold. Valid ranges are from
//! 0-65535.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_setErrorThreshold(uint32_t eccBase, uint16_t threshold)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    EALLOW;
    HWREG(eccBase + FLASH_O_ERR_THRESHOLD) = ((uint32_t)threshold &
        (uint32_t)FLASH_ERR_THRESHOLD_ERR_THRESHOLD_M);
    EDIS;
}

//*****************************************************************************
//
//! Gets the error interrupt.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! This function returns the type of error interrupt that occurred.  The
//! values can be used with
//! - \b FLASH_NO_ERROR
//! - \b FLASH_SINGLE_ERROR
//! - \b FLASH_UNC_ERROR
//!
//! \return Returns the interrupt flag.
//
//*****************************************************************************
static inline uint32_t
Flash_getInterruptFlag(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    //
    // Read which type of error occurred.
    //
    return((HWREG(eccBase + FLASH_O_ERR_INTFLG) & (uint32_t)0x3U));
}

//*****************************************************************************
//
//! Clears the single error interrupt flag.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_clearSingleErrorInterruptFlag(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    EALLOW;
    HWREG(eccBase + FLASH_O_ERR_INTCLR) |=
        FLASH_ERR_INTCLR_SINGLE_ERR_INTCLR;
    EDIS;
}

//*****************************************************************************
//
//! Clears the uncorrectable error interrupt flag.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_clearUncorrectableInterruptFlag(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    EALLOW;
    HWREG(eccBase + FLASH_O_ERR_INTCLR) |=
        FLASH_ERR_INTCLR_UNC_ERR_INTCLR;
    EDIS;
}

//*****************************************************************************
//
//! Sets the Data Low Test register for ECC testing.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//! \param data is a 32-bit value that is the low double word of selected
//! 64-bit data
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_setDataLowECCTest(uint32_t eccBase, uint32_t data)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    EALLOW;
    HWREG(eccBase + FLASH_O_FDATAL_TEST) = data;
    EDIS;
}

//*****************************************************************************
//
//! Sets the Data High Test register for ECC testing.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//! \param data is a 32-bit value that is the high double word of selected
//! 64-bit data
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_setDataHighECCTest(uint32_t eccBase, uint32_t data)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    EALLOW;
    HWREG(eccBase + FLASH_O_FDATAH_TEST) = data;
    EDIS;
}

//*****************************************************************************
//
//! Sets the test address register for ECC testing.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//! \param address is a 32-bit value containing an address. Bits 21-3 will be
//! used as the flash word (128-bit) address.
//!
//! This function left shifts the address 1 bit to convert it to a byte address
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_setECCTestAddress(uint32_t eccBase, uint32_t address)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    //
    // Left shift the address 1 bit to make it byte-addressable
    //
    uint32_t byteAddress = address << 1;

    EALLOW;

    //
    // Write bits 21-3 to the register.
    //
    HWREG(eccBase + FLASH_O_FADDR_TEST) = byteAddress;

    EDIS;

}

//*****************************************************************************
//
//! Sets the ECC test bits for ECC testing.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//! \param ecc is a 32-bit value. The least significant 8 bits are used as
//! the ECC Control Bits in the ECC Test.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_setECCTestECCBits(uint32_t eccBase, uint16_t ecc)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    ASSERT( ecc <= 255U );
    EALLOW;

    //
    // Write the 8 ECC Control Bits.
    //
    HWREG(eccBase + FLASH_O_FECC_TEST) =
        ((uint32_t)ecc & (uint32_t)FLASH_FECC_TEST_ECC_M);
    EDIS;
}

//*****************************************************************************
//
//! Enables ECC Test mode.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_enableECCTestMode(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    EALLOW;
    HWREG(eccBase + FLASH_O_FECC_CTRL) |= FLASH_FECC_CTRL_ECC_TEST_EN;
    EDIS;
}

//*****************************************************************************
//
//! Disables ECC Test mode.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_disableECCTestMode(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    EALLOW;
    HWREG(eccBase + FLASH_O_FECC_CTRL) &=
        ~(uint32_t)FLASH_FECC_CTRL_ECC_TEST_EN;
    EDIS;
}

//*****************************************************************************
//
//! Selects the ECC block on bits [63:0] of bank data.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_selectLowECCBlock(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    EALLOW;
    HWREG(eccBase + FLASH_O_FECC_CTRL) &=
        ~(uint32_t)FLASH_FECC_CTRL_ECC_SELECT;
    EDIS;
}

//*****************************************************************************
//
//! Selects the ECC block on bits [127:64] of bank data.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_selectHighECCBlock(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    EALLOW;
    HWREG(eccBase + FLASH_O_FECC_CTRL) |= FLASH_FECC_CTRL_ECC_SELECT;
    EDIS;
}

//*****************************************************************************
//
//! Performs the ECC calculation on the test block.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_performECCCalculation(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    EALLOW;
    HWREG(eccBase + FLASH_O_FECC_CTRL) |= FLASH_FECC_CTRL_DO_ECC_CALC;
    EDIS;
}

//*****************************************************************************
//
//! Gets the ECC Test data out high 63:32 bits.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! \return Returns the ECC TEst data out High.
//
//*****************************************************************************
static inline uint32_t
Flash_getTestDataOutHigh(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    return(HWREG(eccBase + FLASH_O_FOUTH_TEST));
}

//*****************************************************************************
//
//! Gets the ECC Test data out low 31:0 bits.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! \return Returns the ECC Test data out Low.
//
//*****************************************************************************
static inline uint32_t
Flash_getTestDataOutLow(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    return(HWREG(eccBase + FLASH_O_FOUTL_TEST));
}

//*****************************************************************************
//
//! Gets the ECC Test status.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! This function returns the ECC test status. The values can be used with
//! - \b FLASH_NO_ERROR
//! - \b FLASH_SINGLE_ERROR
//! - \b FLASH_UNC_ERROR
//!
//! \return Returns the ECC test status.
//
//*****************************************************************************
static inline uint32_t
Flash_getECCTestStatus(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    //
    // Read which type of error occurred.
    //
    return((HWREG(eccBase + FLASH_O_FECC_STATUS)) & (uint32_t)0x3U);
}

//*****************************************************************************
//
//! Gets the ECC Test single bit error position.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! \return Returns the ECC Test single bit error position. If the error type
//! is check bits than the position can range from 0 to 7. If the error type
//! is data bits than the position can range from 0 to 63.
//
//*****************************************************************************
static inline uint32_t
Flash_getECCTestErrorPosition(uint32_t eccBase)
{
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    //
    // Read the position bits and shift it to the right.
    //
    return((HWREG(eccBase + FLASH_O_FECC_STATUS) &
        (uint32_t)FLASH_FECC_STATUS_DATA_ERR_POS_M) >>
        FLASH_FECC_STATUS_DATA_ERR_POS_S);
}

//*****************************************************************************
//
//! Gets the single bit error type.
//!
//! \param eccBase is the base address of the flash wrapper ECC registers.
//!
//! \return Returns the single bit error type as a
//!  Flash_SingleBitErrorIndicator. FLASH_DATA_BITS and FLASH_CHECK_BITS
//! indicate where the single bit error occurred.
//
//*****************************************************************************
static inline Flash_SingleBitErrorIndicator
Flash_getECCTestSingleBitErrorType(uint32_t eccBase)
{
    uint32_t errorType;
    //
    // Check the arguments.
    //
    ASSERT(Flash_isECCBaseValid(eccBase));

    //
    // Read the ERR_TYPE bit to see where the single bit error was.
    //
    errorType = ((HWREG(eccBase + FLASH_O_FECC_STATUS) &
                (uint32_t)FLASH_FECC_STATUS_ERR_TYPE) >> 8U);
    return((Flash_SingleBitErrorIndicator)errorType);
}

//*****************************************************************************
//
//! Claim the flash pump semaphore.
//!
//! \param wrapper is the Flash_PumpOwnership wrapper claiming the pump
//! semaphore.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_claimPumpSemaphore(Flash_PumpOwnership wrapper)
{
    //
    // Block until the pump semaphore is claimed.
    //
    EALLOW;
    while(IPC_PUMPREQUEST_REG != (uint32_t)wrapper)
    {
        IPC_PUMPREQUEST_REG = FLASH_PUMP_KEY | (uint32_t)wrapper;
    }
    EDIS;
}

//*****************************************************************************
//
//! Release the flash pump semaphore.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Flash_releasePumpSemaphore(void)
{
    //
    // Relinquish the pump semaphore.
    //
    EALLOW;
    IPC_PUMPREQUEST_REG = FLASH_PUMP_KEY;
    EDIS;
}

//*****************************************************************************
//
//! Initializes the flash control registers.
//!
//! \param ctrlBase is the base address of the flash wrapper control registers.
//! \param eccBase is the base address of the flash wrapper ECC registers.
//! \param waitstates is the wait-state value.
//!
//! This function initializes the flash control registers. At reset bank and
//! pump are in sleep.  A flash access will power up the bank and pump
//! automatically. This function will power up Flash bank and pump and set the
//! fallback mode of flash and pump as active.
//!
//! This function also sets the number of wait-states for a flash access
//! (see Flash_setWaitstates() for more details), and enables cache, the
//! prefetch mechanism, and ECC.
//!
//! \return None.
//
//*****************************************************************************
extern void
Flash_initModule(uint32_t ctrlBase, uint32_t eccBase, uint16_t waitstates);

//*****************************************************************************
//
//! Powers down the flash.
//!
//! \param ctrlBase is the base address of the flash wrapper control registers.
//!
//! This function powers down the flash bank(s) and the flash pump.
//!
//! Note: For this device, you must claim the flash pump semaphore before
//! calling this function and powering down the pump. Afterwards, you may want
//! to relinquish the flash pump.
//!
//! \return None.
//
//*****************************************************************************
extern void
Flash_powerDown(uint32_t ctrlBase);

//*****************************************************************************
//
//! Wakes the flash from low power mode.
//!
//! \param ctrlBase is the base address of the flash wrapper control registers.
//!
//! This function will power up Flash bank and pump and set the
//! fallback mode of flash and pump as active.
//!
//! \return None.
//
//*****************************************************************************
extern void
Flash_wakeFromLPM(uint32_t ctrlBase);

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

#endif // FLASH_H
