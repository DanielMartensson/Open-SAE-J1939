//###########################################################################
//
// FILE:   emif.h
//
// TITLE:  C28x EMIF driver.
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

#ifndef EMIF_H
#define EMIF_H

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
//! \addtogroup emif_api EMIF
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_emif.h"
#include "inc/hw_memcfg.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "cpu.h"
#include "debug.h"

//*****************************************************************************
//
// Defines for the API.
//
//*****************************************************************************
//*****************************************************************************
//
// Defines to specify access protection to EMIF_setAccessProtection().
//
//*****************************************************************************
//! This flag is used to specify whether CPU fetches are allowed/blocked
//! for EMIF.
#define EMIF_ACCPROT0_FETCHPROT       MEMCFG_EMIF1ACCPROT0_FETCHPROT_EMIF1

//! This flag is used to specify whether CPU writes are allowed/blocked
//! for EMIF.
#define EMIF_ACCPROT0_CPUWRPROT       MEMCFG_EMIF1ACCPROT0_CPUWRPROT_EMIF1

//! This flag is used to specify whether DMA writes are allowed/blocked
//! for EMIF. It is valid only for EMIF1 instance.
#define EMIF_ACCPROT0_DMAWRPROT       MEMCFG_EMIF1ACCPROT0_DMAWRPROT_EMIF1

//*****************************************************************************
//
// Define to mask out the bits in the EMIF1ACCPROT0 register that aren't
// associated with EMIF1 access protection.
//
//*****************************************************************************
#define EMIF_ACCPROT0_MASK_EMIF1                                              \
                             ((uint16_t)MEMCFG_EMIF1ACCPROT0_FETCHPROT_EMIF1 |\
                              (uint16_t)MEMCFG_EMIF1ACCPROT0_CPUWRPROT_EMIF1 |\
                              (uint16_t)MEMCFG_EMIF1ACCPROT0_DMAWRPROT_EMIF1)

//*****************************************************************************
//
// Define to mask out the bits in the EMIF2ACCPROT0 register that aren't
// associated with EMIF2 access protection.
//
//*****************************************************************************
#define EMIF_ACCPROT0_MASK_EMIF2                                              \
                             ((uint16_t)MEMCFG_EMIF2ACCPROT0_FETCHPROT_EMIF2 |\
                              (uint16_t)MEMCFG_EMIF2ACCPROT0_CPUWRPROT_EMIF2)

//*****************************************************************************
//
// Define to mask out the bits in the ASYNC_CSx_CR register that
// aren't associated with async configuration.
//
//*****************************************************************************
#define EMIF_ASYNC_CS_CR_MASK    ((uint32_t)EMIF_ASYNC_CS2_CR_R_HOLD_M    |   \
                                  (uint32_t)EMIF_ASYNC_CS2_CR_R_STROBE_M  |   \
                                  (uint32_t)EMIF_ASYNC_CS2_CR_R_SETUP_M   |   \
                                  (uint32_t)EMIF_ASYNC_CS2_CR_W_HOLD_M    |   \
                                  (uint32_t)EMIF_ASYNC_CS2_CR_W_STROBE_M  |   \
                                  (uint32_t)EMIF_ASYNC_CS2_CR_W_SETUP_M   |   \
                                  (uint32_t)EMIF_ASYNC_CS2_CR_TA_M)

//*****************************************************************************
//
// Define to mask out the bits in the INT_MSK register that aren't associated
// with interrupts.
//
//*****************************************************************************
#define EMIF_ASYNC_INT_MASK      ((uint16_t)EMIF_INT_MSK_SET_AT_MASK_SET  |   \
                                  (uint16_t)EMIF_INT_MSK_SET_LT_MASK_SET  |   \
                                  (uint16_t)EMIF_INT_MSK_SET_WR_MASK_SET_M)

//*****************************************************************************
//
// Defines to specify interrupt sources to EMIF_enableAsyncInterrupt() and
// EMIF_disableAsyncInterrupt().Three interrupts are available for asynchronous
// memory interface: Masked Asyncronous Timeout(AT) to indicate EMxWAIT signal
// remains active even after maximum wait cycles are reached. Masked Line Trap
// (LT) to indicate illegal memory access or invalid cache line size.
// Masked Wait Rise(WR) to indicate rising edge on EMxWAIT is detected.
//
//*****************************************************************************
//! This flag is used to allow/block EMIF to generate Masked Asynchronous
//! Timeout interrupt.
#define EMIF_ASYNC_INT_AT      EMIF_INT_MSK_SET_AT_MASK_SET

//! This flag is used to allow/block EMIF to generate Masked Line Trap
//! interrupt.
#define EMIF_ASYNC_INT_LT      EMIF_INT_MSK_SET_LT_MASK_SET

//! This flag is used to allow/block EMIF to generate Masked Wait Rise
//! interrupt.
#define EMIF_ASYNC_INT_WR      EMIF_INT_MSK_SET_WR_MASK_SET_M

//*****************************************************************************
//
// Define for key for EMIF1MSEL register that enables the register write.
//
//*****************************************************************************
#define EMIF_MSEL_KEY  0x93A5CE70U

//*****************************************************************************
//
// Define to mask out the bits in the SDRAM_CR register that aren't
// associated with SDRAM configuration parameters.
//
//*****************************************************************************
#define EMIF_SYNC_SDRAM_CR_MASK   ((uint32_t)EMIF_SDRAM_CR_PAGESIGE_M    |    \
                                   (uint32_t)EMIF_SDRAM_CR_IBANK_M       |    \
                                   (uint32_t)EMIF_SDRAM_CR_BIT_11_9_LOCK |    \
                                   (uint32_t)EMIF_SDRAM_CR_CL_M          |    \
                                   (uint32_t)EMIF_SDRAM_CR_NM            |    \
                                   (uint32_t)EMIF_SDRAM_CR_SR)

//*****************************************************************************
//
// Define to mask out the bits in the SDRAM_TR register that aren't
// associated with SDRAM timings parameters.
//
//*****************************************************************************
#define EMIF_SYNC_SDRAM_TR_MASK  ((uint32_t)EMIF_SDRAM_TR_T_RRD_M  |          \
                                  (uint32_t)EMIF_SDRAM_TR_T_RC_M   |          \
                                  (uint32_t)EMIF_SDRAM_TR_T_RAS_M  |          \
                                  (uint32_t)EMIF_SDRAM_TR_T_WR_M   |          \
                                  (uint32_t)EMIF_SDRAM_TR_T_RCD_M  |          \
                                  (uint32_t)EMIF_SDRAM_TR_T_RP_M   |          \
                                  (uint32_t)EMIF_SDRAM_TR_T_RFC_M)

//*****************************************************************************
//
//! Values that can be passed to EMIF_setAsyncMode(),
//! EMIF_setAsyncTimingParams(), EMIF_setAsyncDataBusWidth(),
//! EMIF_enableAsyncExtendedWait() and EMIF_disableAsyncExtendedWait()
//! as the \e offset parameter. Three chip selects are available in
//! asynchronous memory interface so there are three configuration registers
//! available for each EMIF instance. All the three chip select offsets are
//! valid for EMIF1 while only EMIF_ASYNC_CS2_OFFSET is valid for EMIF2.
//
//*****************************************************************************
typedef enum
{
    EMIF_ASYNC_CS2_OFFSET = EMIF_O_ASYNC_CS2_CR, //!<Async chip select 2 offset
    EMIF_ASYNC_CS3_OFFSET = EMIF_O_ASYNC_CS3_CR, //!<Async chip select 3 offset
    EMIF_ASYNC_CS4_OFFSET = EMIF_O_ASYNC_CS4_CR  //!<Async chip select 4 offset
} EMIF_AsyncCSOffset;

//*****************************************************************************
//
//! Values that can be passed to EMIF_setAsyncDataBusWidth() as the
//! \e width parameter.
//
//*****************************************************************************
typedef enum
{
    EMIF_ASYNC_DATA_WIDTH_8  = 0x0000U, //!<ASRAM/FLASH with 8 bit data bus
    EMIF_ASYNC_DATA_WIDTH_16 = 0x0001U, //!<ASRAM/FLASH with 16 bit data bus
    EMIF_ASYNC_DATA_WIDTH_32 = 0x0002U  //!<ASRAM/FLASH with 32 bit data bus
} EMIF_AsyncDataWidth;

//*****************************************************************************
//
//! Values that can be passed to EMIF_setAsyncMode() as the \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    EMIF_ASYNC_STROBE_MODE = 0x80000000U, //!<Enables ASRAM/FLASH strobe mode
    EMIF_ASYNC_NORMAL_MODE = 0x00000000U  //!<Disables ASRAM/FLASH strobe mode
} EMIF_AsyncMode;

//*****************************************************************************
//
//! Values that can be passed to EMIF_setAsyncWaitPolarity() as the
//! \e polarity parameter.
//
//*****************************************************************************
typedef enum
{
    EMIF_ASYNC_WAIT_POLARITY_LOW  = 0x00000000U,//!<EMxWAIT pin polarity is low
    EMIF_ASYNC_WAIT_POLARITY_HIGH = 0x10000000U//!<EMxWAIT pin polarity is high
} EMIF_AsyncWaitPolarity;
//*****************************************************************************
//
//! Values that can be passed to EMIF_selectController() as the
//! \e select parameter.
//
//*****************************************************************************
typedef enum
{
    EMIF_CONTROLLER_CPU1_NG  = 0x00000000U, //!<CPU1 is controller but not grabbed
    EMIF_CONTROLLER_CPU1_G   = 0x00000001U, //!<CPU1 is controller & grabbed
    EMIF_CONTROLLER_CPU2_G   = 0x00000002U, //!<CPU2 is controller & grabbed
    EMIF_CONTROLLER_CPU1_NG2 = 0x00000003U  //!<CPU1 is controller but not grabbed
} EMIF_ControllerSelect;

//*****************************************************************************
//
//! Values that can be passed to EMIF_setSyncMemoryConfig() as the
//! \e config parameter member.
//
//*****************************************************************************
typedef enum
{
    EMIF_SYNC_NARROW_MODE_TRUE = 0x00004000U, //!< MemBusWidth=SystemBusWidth/2
    EMIF_SYNC_NARROW_MODE_FALSE = 0x00000000U //!< MemBusWidth=SystemBusWidth
} EMIF_SyncNarrowMode;

//*****************************************************************************
//
//! Values that can be passed to EMIF_setSyncMemoryConfig() as the
//! \e config parameter member.
//
//*****************************************************************************
typedef enum
{
    EMIF_SYNC_BANK_1 = 0x00000000U, //!< 1 Bank SDRAM device
    EMIF_SYNC_BANK_2 = 0x00000010U, //!< 2 Bank SDRAM device
    EMIF_SYNC_BANK_4 = 0x00000020U  //!< 4 Bank SDRAM device
} EMIF_SyncBank;

//*****************************************************************************
//
//! Values that can be passed to EMIF_setSyncMemoryConfig() as the
//! \e config parameter member.
//
//*****************************************************************************
typedef enum
{
  EMIF_SYNC_CAS_LAT_2    = 0x00000500U,  //!< SDRAM with CAS Latency 2
  EMIF_SYNC_CAS_LAT_3    = 0x00000700U   //!< SDRAM with CAS Latency 3
} EMIF_SyncCASLatency;

//*****************************************************************************
//
//! Values that can be passed to EMIF_setSyncMemoryConfig() as the
//! \e config parameter member.
//
//*****************************************************************************
typedef enum
{
    EMIF_SYNC_COLUMN_WIDTH_8  = 0x00000000U, //!< 256-word pages in SDRAM
    EMIF_SYNC_COLUMN_WIDTH_9  = 0x00000001U, //!< 512-word pages in SDRAM
    EMIF_SYNC_COLUMN_WIDTH_10 = 0x00000002U, //!< 1024-word pages in SDRAM
    EMIF_SYNC_COLUMN_WIDTH_11 = 0x00000003U  //!< 2048-word pages in SDRAM
} EMIF_SyncPageSize;

//*****************************************************************************
//
//! Values that can be passed to EMIF_setAsyncTimingParams() as the
//! \e tParam parameter.
//
//*****************************************************************************
typedef struct
{
    uint32_t rSetup;            //!< Read Setup Cycles
    uint32_t rStrobe;           //!< Read Strobe Cycles
    uint32_t rHold;             //!< Read Hold Cycles
    uint32_t wSetup;            //!< Write Setup Cycles
    uint32_t wStrobe;           //!< Write Strobe Cycles
    uint32_t wHold;             //!< Write Hold Cycles
    uint32_t turnArnd;          //!< TurnAround Cycles
} EMIF_AsyncTimingParams;

//*****************************************************************************
//
//! Values that can be passed to EMIF_setSyncMemoryConfig() as the
//! \e config parameter.
//
//*****************************************************************************
typedef struct
{
    EMIF_SyncNarrowMode narrowMode;     //!< Read Setup Cycles
    EMIF_SyncBank iBank;                //!< Banks available in SDRAM device
    EMIF_SyncCASLatency casLatency;     //!< CAS Latency for SDRAM device
    EMIF_SyncPageSize pageSize;         //!< Pagesize of SDRAM device
} EMIF_SyncConfig;

//*****************************************************************************
//
//! Values that can be passed to EMIF_setSyncTimingParameters() as the
//! \e tParam parameter.
//
//*****************************************************************************
typedef struct
{
    uint32_t tRfc;          //!< Auto refresh time
    uint32_t tRp;           //!< Row precharge time
    uint32_t tRcd;          //!< RAS to CAS delay
    uint32_t tWr;           //!< Write recovery time
    uint32_t tRas;          //!< Row active time
    uint32_t tRc;           //!< Read cycle time
    uint32_t tRrd;          //!< Row active to row active delay
} EMIF_SyncTimingParams;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! \internal
//! Checks an EMIF base address.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! This function determines if EMIF module base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
EMIF_isBaseValid(uint32_t base)
{
    return(
           (base == EMIF1_BASE) ||
           (base == EMIF2_BASE)
           );
}
#endif

//*****************************************************************************
//
//! \internal
//! Checks an EMIF Configuration Register address.
//!
//! \param configBase is the configuration address of the EMIF instance used.
//!
//! This function determines if EMIF1 module configuration address is valid.
//!
//! \return Returns \b true if the configuration address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
EMIF_isEMIF1ConfigBaseValid(uint32_t configBase)
{
    return(configBase == EMIF1CONFIG_BASE);
}
#endif

//*****************************************************************************
//
//! \internal
//! Checks an EMIF Configuration Register address.
//!
//! \param configBase is the configuration address of the EMIF instance used.
//!
//! This function determines if EMIF2 module configuration address is valid.
//!
//! \return Returns \b true if the configuration address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
EMIF_isEMIF2ConfigBaseValid(uint32_t configBase)
{
    return(configBase == EMIF2CONFIG_BASE);
}
#endif
//*****************************************************************************
//
//! Selects the EMIF Controller.
//!
//! \param configBase is the configuration address of the EMIF instance used.
//!
//! \param select is the required controller configuration for EMIF1.
//!
//! This function selects the controller for an EMIF1 instance among CPU1 or
//! CPU2.<em> It is valid only for EMIF1 instance and not for EMIF2 instance.
//! Valid value for configBase parameter is EMIF1CONFIG_BASE. </em> Valid
//! values for select parameter can be \e EMIF_CONTROLLER_CPU1_NG,
//! \e EMIF_CONTROLLER_CPU1_G, \e EMIF_CONTROLLER_CPU2_G or
//! \e EMIF_CONTROLLER_CPU1_NG2.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_selectController(uint32_t configBase, EMIF_ControllerSelect select)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isEMIF1ConfigBaseValid(configBase));

    //
    // Sets the bits that enables EMIF1 controller selection.
    //
    EALLOW;
    HWREG(configBase + MEMCFG_O_EMIF1MSEL) = (EMIF_MSEL_KEY | (uint32_t)select);
    EDIS;
}

//*****************************************************************************
//
//! Sets the access protection.
//!
//! \param configBase is the configuration address of the EMIF instance used.
//!
//! \param access is the required access protection configuration.
//!
//! This function sets the access protection for an EMIF instance from CPU
//! and DMA. The \e access parameter can be any of \b EMIF_ACCPROT0_FETCHPROT,
//! \b EMIF_ACCPROT0_CPUWRPROT \b EMIF_ACCPROT0_DMAWRPROT values or their
//! combination. <em> EMIF_ACCPROT0_DMAWRPROT value is valid as access parameter
//! for EMIF1 instance only </em>.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_setAccessProtection(uint32_t configBase, uint16_t access)
{
    uint16_t temp;
    //
    // Check the arguments.
    //

    ASSERT(EMIF_isEMIF1ConfigBaseValid(configBase) ||
               EMIF_isEMIF2ConfigBaseValid(configBase));
    if(configBase == EMIF1CONFIG_BASE)
    {
        ASSERT(access <= EMIF_ACCPROT0_MASK_EMIF1);
        temp = EMIF_ACCPROT0_MASK_EMIF1;
    }
    else
    {
        ASSERT(access <= EMIF_ACCPROT0_MASK_EMIF2);
        temp = EMIF_ACCPROT0_MASK_EMIF2;
    }

    //
    // Sets the bits that enables access protection config.
    //
    EALLOW;
    HWREGH(configBase + MEMCFG_O_EMIF1ACCPROT0) =
        (HWREGH(configBase + MEMCFG_O_EMIF1ACCPROT0) & ~(temp)) | access;
    EDIS;
}

//*****************************************************************************
//
//! Commits the lock configuration.
//!
//! \param configBase is the configuration address of the EMIF instance used.
//!
//! This function commits the access protection for an EMIF instance from
//! CPU & DMA.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_commitAccessConfig(uint32_t configBase)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isEMIF1ConfigBaseValid(configBase) ||
            EMIF_isEMIF2ConfigBaseValid(configBase));

    //
    // Sets the bits that commits access protection config.
    //
    EALLOW;
    HWREGH(configBase + MEMCFG_O_EMIF1COMMIT) |=
            MEMCFG_EMIF1COMMIT_COMMIT_EMIF1;
    EDIS;
}

//*****************************************************************************
//
//! Locks the write to access configuration fields.
//!
//! \param configBase is the configuration address of the EMIF instance used.
//!
//! This function locks the write to access configuration fields i.e
//! ACCPROT0 & Mselect fields, for an EMIF instance.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_lockAccessConfig(uint32_t configBase)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isEMIF1ConfigBaseValid(configBase) ||
            EMIF_isEMIF2ConfigBaseValid(configBase));

    //
    // Sets the bits that locks access protection config.
    //
    EALLOW;
    HWREGH(configBase + MEMCFG_O_EMIF1LOCK) |= MEMCFG_EMIF1LOCK_LOCK_EMIF1;
    EDIS;
}

//*****************************************************************************
//
//! Unlocks the write to access configuration fields.
//!
//! \param configBase is the configuration address of the EMIF instance used.
//!
//! This function unlocks the write to access configuration fields such as
//! ACCPROT0 & Mselect fields, for an EMIF instance.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_unlockAccessConfig(uint32_t configBase)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isEMIF1ConfigBaseValid(configBase) ||
            EMIF_isEMIF2ConfigBaseValid(configBase));

    //
    // Sets the bits that unlocks access protection config.
    //
    EALLOW;
    HWREGH(configBase + MEMCFG_O_EMIF1LOCK) &=
            ~((uint16_t)MEMCFG_EMIF1LOCK_LOCK_EMIF1);
    EDIS;
}

//*****************************************************************************
//
// Prototypes for Asynchronous Memory Interface
//
//*****************************************************************************
//*****************************************************************************
//
//! Selects the asynchronous mode of operation.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! \param offset is the offset of asynchronous chip select of EMIF instance.
//!
//! \param mode is the desired mode of operation for external memory.
//!
//!
//! This function sets the mode of operation for asynchronous memory
//! between Normal or Strobe mode. Valid values for param \e offset can be
//! \e EMIF_ASYNC_CS2_OFFSET, \e EMIF_ASYNC_CS3_OFFSET &
//! \e EMIF_ASYNC_C43_OFFSET for EMIF1 and \e EMIF_ASYNC_CS2_OFFSET for EMIF2.
//! Valid values for param \e mode can be \e EMIF_ASYNC_STROBE_MODE or
//! \e EMIF_ASYNC_NORMAL_MODE.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_setAsyncMode(uint32_t base, EMIF_AsyncCSOffset offset,
                  EMIF_AsyncMode mode)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));
    if(base == EMIF2_BASE)
    {
        ASSERT(offset == EMIF_ASYNC_CS2_OFFSET);
    }

    //
    // Sets the async mode of operation.
    //
    HWREG(base + (uint32_t)offset) = (HWREG(base + (uint32_t)offset)
                                      & ~((uint32_t)EMIF_ASYNC_CS2_CR_SS))
                                      | (uint32_t)mode;
}

//*****************************************************************************
//
//! Enables the Extended Wait Mode.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! \param offset is the offset of asynchronous chip select of the
//! EMIF instance
//!
//! This function enables the extended wait mode for an asynchronous
//! external memory.Valid values for param \e offset can be
//! \e EMIF_ASYNC_CS2_OFFSET, \e EMIF_ASYNC_CS3_OFFSET &
//! \e EMIF_ASYNC_C43_OFFSET for EMIF1 and \e EMIF_ASYNC_CS2_OFFSET for EMIF2.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_enableAsyncExtendedWait(uint32_t base, EMIF_AsyncCSOffset offset)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));
    if(base == EMIF2_BASE)
    {
        ASSERT(offset == EMIF_ASYNC_CS2_OFFSET);
    }

    //
    // Sets the bit that enables extended wait mode.
    //
    HWREG(base + (uint32_t)offset) = HWREG(base + (uint32_t)offset) |
                                     EMIF_ASYNC_CS2_CR_EW;
}

//*****************************************************************************
//
//! Disables the Extended Wait Mode.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! \param offset is the offset of asynchronous chip select of EMIF instance.
//!
//! This function disables the extended wait mode for an asynchronous external
//! memory.Valid values for param \e offset can be \e EMIF_ASYNC_CS2_OFFSET,
//! \e EMIF_ASYNC_CS3_OFFSET & \e EMIF_ASYNC_C43_OFFSET for EMIF1 and
//! \e EMIF_ASYNC_CS2_OFFSET for EMIF2.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_disableAsyncExtendedWait(uint32_t base, EMIF_AsyncCSOffset offset)
 {
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));
    if(base == EMIF2_BASE)
    {
        ASSERT(offset == EMIF_ASYNC_CS2_OFFSET);
    }

    //
    // Sets the bit that disables extended wait mode.
    //
    HWREG(base + (uint32_t)offset) = HWREG(base + (uint32_t)offset) &
                                     ~((uint32_t)EMIF_ASYNC_CS2_CR_EW);
}

//*****************************************************************************
//
//! Sets the wait polarity.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! \param polarity is desired wait polarity.
//!
//! This function sets the wait polarity for an asynchronous external memory.
//! Valid values for param \e polarity can be \e EMIF_ASYNC_WAIT_POLARITY_LOW
//! or \e EMIF_ASYNC_WAIT_POLARITY_HIGH.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_setAsyncWaitPolarity(uint32_t base, EMIF_AsyncWaitPolarity polarity)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));

    //
    // Sets the polarity for async extended wait mode.
    //
    HWREG(base + EMIF_O_ASYNC_WCCR) = (HWREG(base + EMIF_O_ASYNC_WCCR)
                                      & ~((uint32_t)EMIF_ASYNC_WCCR_WP0))
                                      | (uint32_t)polarity;
}

//*****************************************************************************
//
//! Sets the Maximum Wait Cycles.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! \param value is the desired maximum wait cycles.
//!
//! This function sets the maximum wait cycles for extended asynchronous cycle.
//! Valid values for parameter \e value lies b/w 0x0U-0xFFU or 0-255.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_setAsyncMaximumWaitCycles(uint32_t base, uint16_t value)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));
    ASSERT(value <= (EMIF_ASYNC_WCCR_MAX_EXT_WAIT_M));

    //
    // Sets the bit that enables extended wait mode.
    //
    HWREGH(base + EMIF_O_ASYNC_WCCR) = (HWREGH(base + EMIF_O_ASYNC_WCCR)
                                  & ~((uint16_t)EMIF_ASYNC_WCCR_MAX_EXT_WAIT_M))
                                  | value;
}

//*****************************************************************************
//
//! Sets the Asynchronous Memory Timing Characteristics.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! \param offset is the offset of asynchronous chip select of EMIF instance.
//!
//! \param tParam is the desired timing parameters.
//!
//! This function sets timing characteristics for an external asynchronous
//! memory to be interfaced. Valid values for param \e offset can be
//! \e EMIF_ASYNC_CS2_OFFSET, \e EMIF_ASYNC_CS3_OFFSET and
//! \e EMIF_ASYNC_C43_OFFSET for EMIF1 & EMIF_ASYNC_CS2_OFFSET for EMIF2.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_setAsyncTimingParams(uint32_t base, EMIF_AsyncCSOffset offset,
                          const EMIF_AsyncTimingParams *tParam)
{
    uint32_t temp;
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));
    if(base == EMIF2_BASE)
    {
        ASSERT(offset == EMIF_ASYNC_CS2_OFFSET);
    }

    //
    // Sets the async memory timing parameters.
    //
    temp = (tParam->turnArnd << EMIF_ASYNC_CS2_CR_TA_S)      |
           (tParam->rHold << EMIF_ASYNC_CS2_CR_R_HOLD_S)     |
           (tParam->rStrobe << EMIF_ASYNC_CS2_CR_R_STROBE_S) |
           (tParam->rSetup << EMIF_ASYNC_CS2_CR_R_SETUP_S)   |
           (tParam->wHold << EMIF_ASYNC_CS2_CR_W_HOLD_S)     |
           (tParam->wStrobe << EMIF_ASYNC_CS2_CR_W_STROBE_S) |
           (tParam->wSetup << EMIF_ASYNC_CS2_CR_W_SETUP_S);

    HWREG(base + (uint32_t)offset) = (HWREG(base + (uint32_t)offset) &
                                       ~EMIF_ASYNC_CS_CR_MASK) | temp;
}

//*****************************************************************************
//
//! Sets the Asynchronous Data Bus Width.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! \param offset is the offset of asynchronous chip select of EMIF instance.
//!
//! \param width is the data bus width of the memory.
//!
//! This function sets the data bus size for an external asynchronous memory
//! to be interfaced. Valid values for param \e offset can be
//! \e EMIF_ASYNC_CS2_OFFSET, \e EMIF_ASYNC_CS3_OFFSET &
//! \e EMIF_ASYNC_C43_OFFSET for EMIF1 and \e EMIF_ASYNC_CS2_OFFSET for EMIF2.
//! Valid values of param \e width can be \e EMIF_ASYNC_DATA_WIDTH_8,
//! \e EMIF_ASYNC_DATA_WIDTH_16 or \e EMIF_ASYNC_DATA_WIDTH_32.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_setAsyncDataBusWidth(uint32_t base, EMIF_AsyncCSOffset offset,
                           EMIF_AsyncDataWidth width)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));
    if(base == EMIF2_BASE)
    {
        ASSERT(offset == EMIF_ASYNC_CS2_OFFSET);
    }

    //
    // Sets the async memory data bus width.
    //
    HWREGH(base + (uint32_t)offset) = (HWREGH(base + (uint32_t)offset)
                                      & ~((uint16_t)EMIF_ASYNC_CS2_CR_ASIZE_M))
                                      | (uint32_t)width;
}

//*****************************************************************************
//
// Prototypes for Interrupt Handling
//
//*****************************************************************************
//*****************************************************************************
//
//! Enables the Asynchronous Memory Interrupts.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! \param intFlags is the mask for desired interrupts.
//!
//! This function enables the desired interrupts for an external asynchronous
//! memory interface. Valid values for param \e intFlags can be
//! \b EMIF_ASYNC_INT_AT, \b EMIF_ASYNC_INT_LT, \b EMIF_ASYNC_INT_WR or their
//! combination.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_enableAsyncInterrupt(uint32_t base, uint16_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));
    ASSERT(intFlags <= EMIF_ASYNC_INT_MASK);

    //
    // Sets the bits that enables async memory interrupts.
    //
    HWREGH(base + EMIF_O_INT_MSK_SET) = intFlags;
}

//*****************************************************************************
//
//! Disables the Asynchronous Memory Interrupts.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! \param intFlags is the mask for interrupts to be disabled.
//!
//! This function disables the desired interrupts for an external asynchronous
//! memory interface. Valid values for param \e intFlags can be
//! \b EMIF_ASYNC_INT_AT, \b EMIF_ASYNC_INT_LT, \b EMIF_ASYNC_INT_WR or
//! their combination.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_disableAsyncInterrupt(uint32_t base, uint16_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));
    ASSERT(intFlags <= EMIF_ASYNC_INT_MASK);

    //
    // Sets the bits that disables async memory interrupts.
    //
    HWREGH(base + EMIF_O_INT_MSK_CLR) = intFlags;

}

//*****************************************************************************
//
//! Gets the interrupt status.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! This function gets the interrupt status for an EMIF instance.
//!
//! \return Returns the current interrupt status.
//
//*****************************************************************************
static inline uint16_t
EMIF_getAsyncInterruptStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));

    //
    // Gets the async memory interrupt status.
    //
    return(HWREGH(base + EMIF_O_INT_MSK) & EMIF_ASYNC_INT_MASK);
}

//*****************************************************************************
//
//! Clears the interrupt status for an EMIF instance.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! \param intFlags is the mask for the interrupt status to be cleared.
//!
//! This function clears the interrupt status for an EMIF instance.
//! The \e intFlags parameter can be any of \b EMIF_INT_MSK_SET_AT_MASK_SET,
//! \b EMIF_INT_MSK_SET_LT_MASK_SET, or \b EMIF_INT_MSK_SET_WR_MASK_SET_M
//! values or their combination.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_clearAsyncInterruptStatus(uint32_t base, uint16_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));
    ASSERT(intFlags <= EMIF_ASYNC_INT_MASK);

    //
    // Sets the bit that clears desired async memory interrupts.
    //
    HWREGH(base + EMIF_O_INT_MSK) = intFlags;
}

//*****************************************************************************
//
// Prototypes for Synchronous Memory Interface
//
//*****************************************************************************
//*****************************************************************************
//
//! Sets the Synchronous Memory Timing Parameters.
//!
//! \param base is the base address of an EMIF instance.
//!
//! \param tParam is parameters from memory datasheet in \e ns.
//!
//! This function sets the timing characteristics for an external
//! synchronous memory to be interfaced.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_setSyncTimingParams(uint32_t base, const EMIF_SyncTimingParams *tParam)
{
    uint32_t temp;
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));

    //
    // Sets sync memory timing parameters.
    //
    temp = ((tParam->tRrd << EMIF_SDRAM_TR_T_RRD_S)
                            & EMIF_SDRAM_TR_T_RRD_M)
                    | ((tParam->tRc << EMIF_SDRAM_TR_T_RC_S)
                            & EMIF_SDRAM_TR_T_RC_M)
                    | ((tParam->tRas << EMIF_SDRAM_TR_T_RAS_S)
                            & EMIF_SDRAM_TR_T_RAS_M)
                    | ((tParam->tWr << EMIF_SDRAM_TR_T_WR_S)
                            & EMIF_SDRAM_TR_T_WR_M)
                    | ((tParam->tRcd << EMIF_SDRAM_TR_T_RCD_S)
                            & EMIF_SDRAM_TR_T_RCD_M)
                    | ((tParam->tRp << EMIF_SDRAM_TR_T_RP_S)
                            & EMIF_SDRAM_TR_T_RP_M)
                    | ((tParam->tRfc << EMIF_SDRAM_TR_T_RFC_S)
                            & EMIF_SDRAM_TR_T_RFC_M);

    HWREG(base + EMIF_O_SDRAM_TR) = (HWREG(base + EMIF_O_SDRAM_TR) &
                                     ~EMIF_SYNC_SDRAM_TR_MASK) | temp;
}

//*****************************************************************************
//
//! Sets the SDRAM Self Refresh Exit Timing.
//!
//! \param base is the base address of an EMIF instance.
//!
//! \param tXs is the desired timing value.
//!
//! This function sets the self refresh exit timing for an external
//! synchronous memory to be interfaced. tXs values must lie between
//! 0x0U-0x1FU or 0-31.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_setSyncSelfRefreshExitTmng(uint32_t base, uint16_t tXs)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));
    ASSERT(tXs <= EMIF_SDR_EXT_TMNG_T_XS_M);

    //
    // Sets the self refresh exit timing for sync memory.
    //
    HWREGH(base + EMIF_O_SDR_EXT_TMNG) = (HWREGH(base + EMIF_O_SDR_EXT_TMNG)
                                        & ~((uint16_t)EMIF_SDR_EXT_TMNG_T_XS_M))
                                        | tXs;
}

//*****************************************************************************
//
//! Sets the SDR Refresh Rate.
//!
//! \param base is the base address of an EMIF instance.
//!
//! \param refRate is the refresh rate.
//!
//! This function sets the refresh rate for an external synchronous memory
//! to be interfaced. Valid values for refRate lies b/w 0x0U-0x1FFFU or
//! 0-8191.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_setSyncRefreshRate(uint32_t base, uint16_t refRate)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));
    ASSERT(refRate <= EMIF_SDRAM_RCR_REFRESH_RATE_M);

    //
    // Sets the sync memory refresh rate.
    //
    HWREGH(base + EMIF_O_SDRAM_RCR) = (HWREGH(base + EMIF_O_SDRAM_RCR)
                                   & (~(uint16_t)EMIF_SDRAM_RCR_REFRESH_RATE_M))
                                   | refRate;
}

//*****************************************************************************
//
//! Sets the Synchronous Memory configuration parameters.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! \param config is the desired configuration parameters.
//!
//! This function sets configuration parameters like CL, NM, IBANK
//! and PAGESIZE for an external synchronous memory to be interfaced.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_setSyncMemoryConfig(uint32_t base, const EMIF_SyncConfig *config)
{
    uint32_t temp;
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));

    //
    // Sets the sync memory configuration bits.
    //
    temp = ((uint32_t)config->casLatency | (uint32_t)config->iBank |
            (uint32_t)config->narrowMode | (uint32_t)config->pageSize);

    HWREG(base + EMIF_O_SDRAM_CR) = (HWREG(base + EMIF_O_SDRAM_CR) &
                                     ~EMIF_SYNC_SDRAM_CR_MASK) | temp;
}

//*****************************************************************************
//
// Prototypes for EMIF Low Power Modes
//
//*****************************************************************************
//*****************************************************************************
//
//! Enables Self Refresh.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! This function enables Self Refresh Mode for EMIF.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_enableSyncSelfRefresh(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));

    //
    // Sets the bits that enables sync memory self refresh mode.
    //
    HWREG(base + EMIF_O_SDRAM_CR) |= EMIF_SDRAM_CR_SR;
}

//*****************************************************************************
//
//! Disables Self Refresh.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! This function disables Self Refresh Mode for EMIF.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_disableSyncSelfRefresh(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));

    //
    // Sets the bits that disables sync memory self refresh mode.
    //
    HWREG(base + EMIF_O_SDRAM_CR) &= ~((uint32_t)EMIF_SDRAM_CR_SR);
}

//*****************************************************************************
//
//! Enables Power Down.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! This function Enables Power Down Mode for synchronous memory
//! to be interfaced.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_enableSyncPowerDown(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));

    //
    // Sets the bits that enables sync memory power down mode.
    //
    HWREG(base + EMIF_O_SDRAM_CR) |= EMIF_SDRAM_CR_PD;
}

//*****************************************************************************
//
//! Disables Power Down.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! This function disables Power Down Mode for synchronous memory
//! to be interfaced.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_disableSyncPowerDown(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));

    //
    // Sets the bits that disables sync memory power down mode.
    //
    HWREG(base + EMIF_O_SDRAM_CR) &= ~((uint32_t)EMIF_SDRAM_CR_PD);
}

//*****************************************************************************
//
//! Enables Refresh in Power Down.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! This function enables Refresh in Power Down Mode for synchronous memory
//! to be interfaced.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_enableSyncRefreshInPowerDown(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));

    //
    // Sets the bits that enables refresh in power down mode.
    //
    HWREG(base + EMIF_O_SDRAM_CR) |= EMIF_SDRAM_CR_PDWR;
}

//*****************************************************************************
//
//! Disables Refresh in Power Down.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! This function disables Refresh in Power Down Mode for synchronous memory
//! to be interfaced.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EMIF_disableSyncRefreshInPowerDown(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));

    //
    // Sets the bits that disables refresh in power down mode.
    //
    HWREG(base + EMIF_O_SDRAM_CR) &= ~((uint32_t)EMIF_SDRAM_CR_PDWR);
}

//*****************************************************************************
//
//! Gets total number of SDRAM accesses.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! This function returns total number of SDRAM accesses
//! from a controller(CPUx/CPUx.DMA).
//!
//! \return \e Returns total number of accesses to SDRAM.
//
//*****************************************************************************
static inline uint32_t
EMIF_getSyncTotalAccesses(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));

    //
    // Gets total accesses to sync memory.
    //
    return(HWREG(base + EMIF_O_TOTAL_SDRAM_AR));
}

//*****************************************************************************
//
//! Gets total number of SDRAM accesses which require activate command.
//!
//! \param base is the base address of the EMIF instance used.
//!
//! This function returns total number of accesses to SDRAM which
//! require activate command.
//!
//!\return \e Returns total number of accesses to SDRAM which require activate.
//
//*****************************************************************************
static inline uint32_t
EMIF_getSyncTotalActivateAccesses(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(EMIF_isBaseValid(base));

    //
    // Gets total accesses to sync memory which requires activate command.
    //
    return(HWREG(base + EMIF_O_TOTAL_SDRAM_ACTR));
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

#endif // EMIF_H
