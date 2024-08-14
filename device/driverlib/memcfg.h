//###########################################################################
//
// FILE:   memcfg.h
//
// TITLE:  C28x RAM config driver.
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

#ifndef MEMCFG_H
#define MEMCFG_H

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
//! \addtogroup memcfg_api MemCfg
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memcfg.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "cpu.h"
#include "debug.h"

//*****************************************************************************
//
// Useful defines used within the driver functions. Not intended for use by
// application code.
//
//*****************************************************************************
//
// Masks to decode memory section defines.
//
#define MEMCFG_SECT_TYPE_MASK   0xFF000000U
#define MEMCFG_SECT_TYPE_D      0x00000000U
#define MEMCFG_SECT_TYPE_LS     0x01000000U
#define MEMCFG_SECT_TYPE_GS     0x02000000U
#define MEMCFG_SECT_TYPE_MSG    0x03000000U
#define MEMCFG_SECT_TYPE_ROM      0x04000000U
#define MEMCFG_SECT_TYPE_PERIMEM  0x05000000U
#define MEMCFG_SECT_NUM_MASK    0x00FFFFFFU
#define MEMCFG_XACCPROTX_M      ((uint32_t)MEMCFG_GSXACCPROT0_FETCHPROT_GS0 | \
                                 (uint32_t)MEMCFG_GSXACCPROT0_CPUWRPROT_GS0 | \
                                 (uint32_t)MEMCFG_GSXACCPROT0_DMAWRPROT_GS0)
#define MEMCFG_XTEST_M          MEMCFG_DXTEST_TEST_M0_M

//
// Used for access violation functions.
//
#define MEMCFG_NMVIOL_MASK      0x0000FFFFU
#define MEMCFG_MVIOL_MASK       0x000F0000U
#define MEMCFG_MVIOL_SHIFT      16U

//
// Key for writing to memory test config lock registers
//
#define MEMCFG_TESTLOCK_KEY     0xA5A50000U

#ifndef DOXYGEN_PDF_IGNORE
//*****************************************************************************
//
// Values that can be passed to MemCfg_lockConfig(), MemCfg_unlockConfig(),
// MemCfg_commitConfig(), MemCfg_setProtection(), MemCfg_initSections(),
// MemCfg_setCLAMemType(), MemCfg_setLSRAMControllerSel(), MemCfg_getInitStatus()
// as the memSection(s) or ramSection(s) parameter.
//
//*****************************************************************************
//
// DxRAM - Dedicated RAM config
//
#define MEMCFG_SECT_M0              0x00000001U //!< M0 RAM
#define MEMCFG_SECT_M1              0x00000002U //!< M1 RAM
#define MEMCFG_SECT_D0              0x00000004U //!< D0 RAM
#define MEMCFG_SECT_D1              0x00000008U //!< D1 RAM
#define MEMCFG_SECT_DX_ALL          0x0000000FU //!< All M and D RAM

//
// LSxRAM - Local shared RAM config
//
#define MEMCFG_SECT_LS0             0x01000001U //!< LS0 RAM
#define MEMCFG_SECT_LS1             0x01000002U //!< LS1 RAM
#define MEMCFG_SECT_LS2             0x01000004U //!< LS2 RAM
#define MEMCFG_SECT_LS3             0x01000008U //!< LS3 RAM
#define MEMCFG_SECT_LS4             0x01000010U //!< LS4 RAM
#define MEMCFG_SECT_LS5             0x01000020U //!< LS5 RAM
#define MEMCFG_SECT_LS6             0x01000040U //!< LS6 RAM
#define MEMCFG_SECT_LS7             0x01000080U //!< LS7 RAM
#define MEMCFG_SECT_LSX_ALL         0x010000FFU //!< All LS RAM

//
// GSxRAM - Global shared RAM config
//
#define MEMCFG_SECT_GS0             0x02000001U //!< GS0 RAM
#define MEMCFG_SECT_GS1             0x02000002U //!< GS1 RAM
#define MEMCFG_SECT_GS2             0x02000004U //!< GS2 RAM
#define MEMCFG_SECT_GS3             0x02000008U //!< GS3 RAM
#define MEMCFG_SECT_GS4             0x02000010U //!< GS4 RAM
#define MEMCFG_SECT_GS5             0x02000020U //!< GS5 RAM
#define MEMCFG_SECT_GS6             0x02000040U //!< GS6 RAM
#define MEMCFG_SECT_GS7             0x02000080U //!< GS7 RAM
#define MEMCFG_SECT_GS8             0x02000100U //!< GS8 RAM
#define MEMCFG_SECT_GS9             0x02000200U //!< GS9 RAM
#define MEMCFG_SECT_GS10            0x02000400U //!< GS10 RAM
#define MEMCFG_SECT_GS11            0x02000800U //!< GS11 RAM
#define MEMCFG_SECT_GS12            0x02001000U //!< GS12 RAM
#define MEMCFG_SECT_GS13            0x02002000U //!< GS13 RAM
#define MEMCFG_SECT_GS14            0x02004000U //!< GS14 RAM
#define MEMCFG_SECT_GS15            0x02008000U //!< GS15 RAM
#define MEMCFG_SECT_GSX_ALL         0x0200FFFFU //!< All GS RAM

//
// MSGxRAM - Message RAM config
//
#define MEMCFG_SECT_MSGCPUTOCPU0    0x03000001U //!< CPU-to-CPU message RAM0
#define MEMCFG_SECT_MSGCPUTOCLA1    0x03000002U //!< CPU-to-CLA1 message RAM
#define MEMCFG_SECT_MSGCLA1TOCPU    0x03000004U //!< CLA1-to-CPU message RAM
#define MEMCFG_SECT_MSGCLA1TODMA    0x03000020U //!< CLA1-to-DMA message RAM
#define MEMCFG_SECT_MSGDMATOCLA1    0x03000040U //!< DMA-to-CLA1 message RAM
#define MEMCFG_SECT_MSGCPUTOCPU1    0x03000080U //!< CPU-to-CPU message RAM1
#define MEMCFG_SECT_MSGCPUTOCM0     0x03000100U //!< CPU-to-CM message RAM0
#define MEMCFG_SECT_MSGCPUTOCM1     0x03000200U //!< CPU-to-CM message RAM1
#define MEMCFG_SECT_MSGX_ALL        0x030003E7U //!< All message RAM

//
// ROM memory sections
//
#define MEMCFG_SECT_ROMBOOT           0x04000001U //!< BOOT ROM
#define MEMCFG_SECT_ROMSECURE         0x04000002U //!< Secure ROM
#define MEMCFG_SECT_ROMCLADATA        0x04000004U //!< CLA Data ROM
#define MEMCFG_SECT_ROM_ALL           0x04000007U //!< All ROMs

//
// Peripheral memory sections
//
#define MEMCFG_SECT_PERIMEM_ETHERCAT  0x05000020U //!< Ethercat memory
#define MEMCFG_SECT_PERIMEM_ALL       0x05000020U //!< All peripheral memories

//
// All sections
//
#define MEMCFG_SECT_ALL             0xFFFFFFFFU //!< All configurable RAM

//*****************************************************************************
//
// Values that can be passed to MemCfg_setProtection() as the protectMode
// parameter.
//
//*****************************************************************************
#define MEMCFG_PROT_ALLOWCPUFETCH   0x00000000U //!< CPU fetch allowed
#define MEMCFG_PROT_BLOCKCPUFETCH   0x00000001U //!< CPU fetch blocked

#define MEMCFG_PROT_ALLOWCPUWRITE   0x00000000U //!< CPU write allowed
#define MEMCFG_PROT_BLOCKCPUWRITE   0x00000002U //!< CPU write blocked

#define MEMCFG_PROT_ALLOWDMAWRITE   0x00000000U //!< DMA write allowed (GSxRAM)
#define MEMCFG_PROT_BLOCKDMAWRITE   0x00000004U //!< DMA write blocked (GSxRAM)

//*****************************************************************************
//
// Values that can be passed to MemCfg_enableViolationInterrupt()
// MemCfg_disableViolationInterrupt(), MemCfg_forceViolationInterrupt(),
// MemCfg_clearViolationInterruptStatus(), and MemCfg_getViolationAddress() as
// the intFlags parameter. They also make up the return value of
// MemCfg_getViolationInterruptStatus().
//
//*****************************************************************************
#define MEMCFG_NMVIOL_CPUREAD    0x00000001U //!< Non-controller CPU read access
#define MEMCFG_NMVIOL_CPUWRITE   0x00000002U //!< Non-controller CPU write access
#define MEMCFG_NMVIOL_CPUFETCH   0x00000004U //!< Non-controller CPU fetch access
#define MEMCFG_NMVIOL_DMAWRITE   0x00000008U //!< Non-controller DMA write access
#define MEMCFG_NMVIOL_CLA1READ   0x00000010U //!< Non-controller CLA1 read access
#define MEMCFG_NMVIOL_CLA1WRITE  0x00000020U //!< Non-controller CLA1 write access
#define MEMCFG_NMVIOL_CLA1FETCH  0x00000040U //!< Non-controller CLA1 fetch access
#define MEMCFG_NMVIOL_DMAREAD    0x00000400U //!< Non-controller DMA read access

//*****************************************************************************
//
// Values that can be passed to MemCfg_enableViolationInterrupt()
// MemCfg_disableViolationInterrupt(), MemCfg_forceViolationInterrupt(),
// MemCfg_clearViolationInterruptStatus(), and MemCfg_getViolationAddress() as
// the intFlags parameter. They also make up the return value of
// MemCfg_getViolationInterruptStatus().
//
//*****************************************************************************
#define MEMCFG_MVIOL_CPUFETCH    0x00010000U //!< Controller CPU fetch access
#define MEMCFG_MVIOL_CPUWRITE    0x00020000U //!< Controller CPU write access
#define MEMCFG_MVIOL_DMAWRITE    0x00040000U //!< Controller DMA write access

//*****************************************************************************
//
// Values that can be passed to MemCfg_forceCorrErrorStatus(),
// MemCfg_clearCorrErrorStatus(), and MemCfg_getCorrErrorAddress() as the
// stsFlag(s) parameter and returned by MemCfg_getCorrErrorStatus().
//
//*****************************************************************************
#define MEMCFG_CERR_CPUREAD      0x0001U //!< Correctable CPU read error
#define MEMCFG_CERR_DMAREAD      0x0002U //!< Correctable DMA read error
#define MEMCFG_CERR_CLA1READ     0x0004U //!< Correctable CLA1 read error
//*****************************************************************************
//
// Values that can be passed to MemCfg_forceUncorrErrorStatus(),
// MemCfg_clearUncorrErrorStatus(), and MemCfg_getUncorrErrorAddress() as the
// stsFlag(s) parameter and returned by MemCfg_getUncorrErrorStatus().
//
//*****************************************************************************
#define MEMCFG_UCERR_CPUREAD     0x0001U //!< Uncorrectable CPU read error
#define MEMCFG_UCERR_DMAREAD     0x0002U //!< Uncorrectable DMA read error
#define MEMCFG_UCERR_CLA1READ    0x0004U //!< Uncorrectable CLA1 read error
#define MEMCFG_UCERR_ECATMEMREAD 0x0010U //!< Uncorrectable ECAT RAM read error
//*****************************************************************************
//
// Values that can be passed to MemCfg_clearDiagErrorStatus() as stsFlags
// parameter and returned by MemCfg_getDiagErrorStatus().
//
//*****************************************************************************
#define MEMCFG_DIAGERR_UNCORR    0x00000002U //!< Uncorr error in diag/test mode
#define MEMCFG_DIAGERR_CORR      0x00000001U //!< Corr error in diag/test mode

#endif

//*****************************************************************************
//
//! Values that can be passed to MemCfg_setCLAMemType() as the \e claMemType
//! parameter.
//
//*****************************************************************************
typedef enum
{
    MEMCFG_CLA_MEM_DATA,                //!< Section is CLA data memory
    MEMCFG_CLA_MEM_PROGRAM              //!< Section is CLA program memory
} MemCfg_CLAMemoryType;

//*****************************************************************************
//
//! Values that can be passed to MemCfg_setLSRAMControllerSel() as the
//! \e controllerSel parameter.
//
//*****************************************************************************
typedef enum
{
    MEMCFG_LSRAMCONTROLLER_CPU_ONLY,   //!< CPU is the owner of the section
    MEMCFG_LSRAMCONTROLLER_CPU_CLA1    //!< CPU and CLA1 share this section
} MemCfg_LSRAMControllerSel;

//*****************************************************************************
//
//! Values that can be passed to MemCfg_setGSRAMControllerSel() as the
//! \e controllerSel parameter.
//
//*****************************************************************************
typedef enum
{
    MEMCFG_GSRAMCONTROLLER_CPU1,            //!< CPU1 is controller of the section
    MEMCFG_GSRAMCONTROLLER_CPU2             //!< CPU2 is controller of the section
} MemCfg_GSRAMControllerSel;

//*****************************************************************************
//
//! Values that can be passed to MemCfg_setTestMode() as the \e testMode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! Functional mode. Test mode is disabled.
    MEMCFG_TEST_FUNCTIONAL   = 0,
    //! Writes allowed to data only
    MEMCFG_TEST_WRITE_DATA   = 1,
    //! Writes allowed to ECC only (for DxRAM & LSxRAM)
    MEMCFG_TEST_WRITE_ECC    = 2,
        //! Writes allowed to Parity only (for GSxRAM, MSGxRAM & ROM)
    MEMCFG_TEST_WRITE_PARITY = 2,
    //! Diagnostic mode, similar to functional mode but NMI is not generated.
    MEMCFG_TEST_FUNC_DIAG    = 3
} MemCfg_TestMode;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************

//*****************************************************************************
//
//! Sets the CLA memory type of the specified RAM section.
//!
//! \param ramSections is the logical OR of the sections to be configured.
//! \param claMemType indicates data memory or program memory.
//!
//! This function sets the CLA memory type configuration of the RAM section. If
//! the \e claMemType parameter is \b MEMCFG_CLA_MEM_DATA, the RAM section will
//! be configured as CLA data memory. If \b MEMCFG_CLA_MEM_PROGRAM, the RAM
//! section will be configured as CLA program memory.
//!
//! The \e ramSections parameter is an OR of the following indicators:
//! \b MEMCFG_SECT_LS0 through \b MEMCFG_SECT_LSx.
//!
//! \note This API only applies to LSx RAM and has no effect if the CLA isn't
//! controller of the memory section.
//!
//! \sa MemCfg_setLSRAControllerSel()
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_setCLAMemType(uint32_t ramSections, MemCfg_CLAMemoryType claMemType)
{
    //
    // Check the arguments.
    //
    ASSERT((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS);

    //
    // Write the CLA memory configuration to the appropriate register. Either
    // set or clear the bit that determines the function of the RAM section as
    // it relates to the CLA.
    //
    EALLOW;

    if(claMemType == MEMCFG_CLA_MEM_PROGRAM)
    {
        //
        // Program memory
        //
        HWREG(MEMCFG_BASE + MEMCFG_O_LSXCLAPGM) |= ramSections;
    }
    else
    {
        //
        // Data memory
        //
        HWREG(MEMCFG_BASE + MEMCFG_O_LSXCLAPGM) &= ~ramSections;
    }

    EDIS;
}

//*****************************************************************************
//
//! Enables individual RAM access violation interrupt sources.
//!
//! \param intFlags is a bit mask of the interrupt sources to be enabled.
//! Can be a logical OR any of the following values:
//!  - \b MEMCFG_NMVIOL_CPUREAD   - Non-controller CPU read access
//!  - \b MEMCFG_NMVIOL_CPUWRITE  - Non-controller CPU write access
//!  - \b MEMCFG_NMVIOL_CPUFETCH  - Non-controller CPU fetch access
//!  - \b MEMCFG_NMVIOL_DMAWRITE  - Non-controller DMA write access
//!  - \b MEMCFG_NMVIOL_CLA1READ  - Non-controller CLA1 read access
//!  - \b MEMCFG_NMVIOL_CLA1WRITE - Non-controller CLA1 write access
//!  - \b MEMCFG_NMVIOL_CLA1FETCH - Non-controller CLA1 fetch access
//!  - \b MEMCFG_NMVIOL_DMAREAD   - Non-controller DMA read access
//!  - \b MEMCFG_MVIOL_CPUFETCH   - Controller CPU fetch access
//!  - \b MEMCFG_MVIOL_CPUWRITE   - Controller CPU write access
//!  - \b MEMCFG_MVIOL_DMAWRITE   - Controller DMA write access
//! This function enables the indicated RAM access violation interrupt sources.
//! Only the sources that are enabled can be reflected to the processor
//! interrupt; disabled sources have no effect on the processor.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_enableViolationInterrupt(uint32_t intFlags)
{
    //
    // Enable the specified interrupts.
    //
    EALLOW;

    HWREG(ACCESSPROTECTION_BASE + MEMCFG_O_NMAVINTEN) |=
        intFlags & MEMCFG_NMVIOL_MASK;

    HWREG(ACCESSPROTECTION_BASE + MEMCFG_O_MAVINTEN) |=
        (intFlags & MEMCFG_MVIOL_MASK) >> MEMCFG_MVIOL_SHIFT;

    EDIS;
}

//*****************************************************************************
//
//! Disables individual RAM access violation interrupt sources.
//!
//! \param intFlags is a bit mask of the interrupt sources to be disabled.
//! Can be a logical OR any of the following values:
//!  - \b MEMCFG_NMVIOL_CPUREAD   - Non-controller CPU read access
//!  - \b MEMCFG_NMVIOL_CPUWRITE  - Non-controller CPU write access
//!  - \b MEMCFG_NMVIOL_CPUFETCH  - Non-controller CPU fetch access
//!  - \b MEMCFG_NMVIOL_DMAWRITE  - Non-controller DMA write access
//!  - \b MEMCFG_NMVIOL_CLA1READ  - Non-controller CLA1 read access
//!  - \b MEMCFG_NMVIOL_CLA1WRITE - Non-controller CLA1 write access
//!  - \b MEMCFG_NMVIOL_CLA1FETCH - Non-controller CLA1 fetch access
//!  - \b MEMCFG_NMVIOL_DMAREAD   - Non-controller DMA read access
//!  - \b MEMCFG_MVIOL_CPUFETCH   - Controller CPU fetch access
//!  - \b MEMCFG_MVIOL_CPUWRITE   - Controller CPU write access
//!  - \b MEMCFG_MVIOL_DMAWRITE   - Controller DMA write access
//!
//! This function disables the indicated RAM access violation interrupt
//! sources. Only the sources that are enabled can be reflected to the
//! processor interrupt; disabled sources have no effect on the processor.
//!
//! \note Note that only non-controller violations may generate interrupts.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_disableViolationInterrupt(uint32_t intFlags)
{
    //
    // Disable the specified interrupts.
    //
    EALLOW;

    HWREG(ACCESSPROTECTION_BASE + MEMCFG_O_NMAVINTEN) &=
        ~(intFlags & MEMCFG_NMVIOL_MASK);

    HWREG(ACCESSPROTECTION_BASE + MEMCFG_O_MAVINTEN) &=
        ~((intFlags & MEMCFG_MVIOL_MASK) >> MEMCFG_MVIOL_SHIFT);

    EDIS;
}

//*****************************************************************************
//
//! Gets the current RAM access violation status.
//!
//! This function returns the RAM access violation status. This function will
//! return flags for both controller and non-controller access violations
//! although only the non-controller flags have the ability to cause the
//! generation of an interrupt.
//!
//! \return Returns the current violation status, enumerated as a bit field of
//! the values:
//!  - \b MEMCFG_NMVIOL_CPUREAD   - Non-controller CPU read access
//!  - \b MEMCFG_NMVIOL_CPUWRITE  - Non-controller CPU write access
//!  - \b MEMCFG_NMVIOL_CPUFETCH  - Non-controller CPU fetch access
//!  - \b MEMCFG_NMVIOL_DMAWRITE  - Non-controller DMA write access
//!  - \b MEMCFG_NMVIOL_CLA1READ  - Non-controller CLA1 read access
//!  - \b MEMCFG_NMVIOL_CLA1WRITE - Non-controller CLA1 write access
//!  - \b MEMCFG_NMVIOL_CLA1FETCH - Non-controller CLA1 fetch access
//!  - \b MEMCFG_NMVIOL_DMAREAD   - Non-controller DMA read access
//!  - \b MEMCFG_MVIOL_CPUFETCH   - Controller CPU fetch access
//!  - \b MEMCFG_MVIOL_CPUWRITE   - Controller CPU write access
//!  - \b MEMCFG_MVIOL_DMAWRITE   - Controller DMA write access
//*****************************************************************************
static inline uint32_t
MemCfg_getViolationInterruptStatus(void)
{
    uint32_t status;

    //
    // Read and return RAM access status flags.
    //
    status = (HWREG(ACCESSPROTECTION_BASE + MEMCFG_O_NMAVFLG)) |
             (HWREG(ACCESSPROTECTION_BASE + MEMCFG_O_MAVFLG) <<
              MEMCFG_MVIOL_SHIFT);

    return(status);
}

//*****************************************************************************
//
//! Sets the RAM access violation status.
//!
//! \param intFlags is a bit mask of the access violation flags to be set.
//! Can be a logical OR any of the following values:
//!  - \b MEMCFG_NMVIOL_CPUREAD   - Non-controller CPU read access
//!  - \b MEMCFG_NMVIOL_CPUWRITE  - Non-controller CPU write access
//!  - \b MEMCFG_NMVIOL_CPUFETCH  - Non-controller CPU fetch access
//!  - \b MEMCFG_NMVIOL_DMAWRITE  - Non-controller DMA write access
//!  - \b MEMCFG_NMVIOL_CLA1READ  - Non-controller CLA1 read access
//!  - \b MEMCFG_NMVIOL_CLA1WRITE - Non-controller CLA1 write access
//!  - \b MEMCFG_NMVIOL_CLA1FETCH - Non-controller CLA1 fetch access
//!  - \b MEMCFG_NMVIOL_DMAREAD   - Non-controller DMA read access
//!  - \b MEMCFG_MVIOL_CPUFETCH   - Controller CPU fetch access
//!  - \b MEMCFG_MVIOL_CPUWRITE   - Controller CPU write access
//!  - \b MEMCFG_MVIOL_DMAWRITE   - Controller DMA write access
//!
//! This function sets the RAM access violation status. This function will
//! set flags for both controller and non-controller access violations, and an
//! interrupt will be generated if it is enabled.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_forceViolationInterrupt(uint32_t intFlags)
{
    //
    // Shift and mask the flags appropriately and write them to the
    // corresponding SET register.
    //
    EALLOW;

    HWREG(ACCESSPROTECTION_BASE + MEMCFG_O_NMAVSET) =
        intFlags & MEMCFG_NMVIOL_MASK;

    HWREG(ACCESSPROTECTION_BASE + MEMCFG_O_MAVSET) =
        (intFlags & MEMCFG_MVIOL_MASK) >> MEMCFG_MVIOL_SHIFT;

    EDIS;
}

//*****************************************************************************
//
//! Clears RAM access violation flags.
//!
//! \param intFlags is a bit mask of the access violation flags to be cleared.
//! Can be a logical OR any of the following values:
//!  - \b MEMCFG_NMVIOL_CPUREAD   - Non-controller CPU read access
//!  - \b MEMCFG_NMVIOL_CPUWRITE  - Non-controller CPU write access
//!  - \b MEMCFG_NMVIOL_CPUFETCH  - Non-controller CPU fetch access
//!  - \b MEMCFG_NMVIOL_DMAWRITE  - Non-controller DMA write access
//!  - \b MEMCFG_NMVIOL_CLA1READ  - Non-controller CLA1 read access
//!  - \b MEMCFG_NMVIOL_CLA1WRITE - Non-controller CLA1 write access
//!  - \b MEMCFG_NMVIOL_CLA1FETCH - Non-controller CLA1 fetch access
//!  - \b MEMCFG_NMVIOL_DMAREAD   - Non-controller DMA read access
//!  - \b MEMCFG_MVIOL_CPUFETCH   - Controller CPU fetch access
//!  - \b MEMCFG_MVIOL_CPUWRITE   - Controller CPU write access
//!  - \b MEMCFG_MVIOL_DMAWRITE   - Controller DMA write access
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_clearViolationInterruptStatus(uint32_t intFlags)
{
    //
    // Clear the requested access violation flags.
    //
    EALLOW;

    HWREG(ACCESSPROTECTION_BASE + MEMCFG_O_NMAVCLR) |=
        intFlags & MEMCFG_NMVIOL_MASK;

    HWREG(ACCESSPROTECTION_BASE + MEMCFG_O_MAVCLR) |=
        (intFlags & MEMCFG_MVIOL_MASK) >> MEMCFG_MVIOL_SHIFT;

    EDIS;
}

//*****************************************************************************
//
//! Sets the correctable error threshold value.
//!
//! \param threshold is the correctable error threshold.
//!
//! This value sets the error-count threshold at which a correctable error
//! interrupt is generated. That is when the error count register reaches the
//! value specified by the \e threshold parameter, an interrupt is
//! generated if it is enabled.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_setCorrErrorThreshold(uint32_t threshold)
{
    //
    // Write the threshold value to the appropriate register.
    //
    EALLOW;

    HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRTHRES) = threshold;

    EDIS;
}

//*****************************************************************************
//
//! Gets the correctable error count.
//!
//! \return Returns the number of correctable error have occurred.
//
//*****************************************************************************
static inline uint32_t
MemCfg_getCorrErrorCount(void)
{
    //
    // Read and return the number of errors that have occurred.
    //
    return(HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRCNT));
}

//*****************************************************************************
//
//! Enables individual RAM correctable error interrupt sources.
//!
//! \param intFlags is a bit mask of the interrupt sources to be enabled. Can
//! take the value \b MEMCFG_CERR_CPUREAD only. Other values are reserved.
//!
//! This function enables the indicated RAM correctable error interrupt
//! sources. Only the sources that are enabled can be reflected to the
//! processor interrupt; disabled sources have no effect on the processor.
//!
//! \note Note that only correctable errors may generate interrupts.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_enableCorrErrorInterrupt(uint32_t intFlags)
{
    //
    // Enable the specified interrupts.
    //
    EALLOW;

    HWREG(MEMORYERROR_BASE + MEMCFG_O_CEINTEN) |= intFlags;

    EDIS;
}

//*****************************************************************************
//
//! Disables individual RAM correctable error interrupt sources.
//!
//! \param intFlags is a bit mask of the interrupt sources to be disabled. Can
//! take the value \b MEMCFG_CERR_CPUREAD only. Other values are reserved.
//!
//! This function disables the indicated RAM correctable error interrupt
//! sources. Only the sources that are enabled can be reflected to the
//! processor interrupt; disabled sources have no effect on the processor.
//!
//! \note Note that only correctable errors may generate interrupts.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_disableCorrErrorInterrupt(uint32_t intFlags)
{
    //
    // Disable the specified interrupts.
    //
    EALLOW;

    HWREG(MEMORYERROR_BASE + MEMCFG_O_CEINTEN) &= ~(intFlags);

    EDIS;
}

//*****************************************************************************
//
//! Gets the current RAM correctable error interrupt status.
//!
//! \return Returns the current error interrupt status. Will return a value of
//! \b MEMCFG_CERR_CPUREAD if an interrupt has been generated. If not, the
//! function will return 0.
//
//*****************************************************************************
static inline uint32_t
MemCfg_getCorrErrorInterruptStatus(void)
{
    //
    // Read and return correctable error interrupt flags.
    //
    return(HWREG(MEMORYERROR_BASE + MEMCFG_O_CEINTFLG));
}

//*****************************************************************************
//
//! Sets the RAM correctable error interrupt status.
//!
//! \param intFlags is a bit mask of the interrupt sources to be set. Can take
//! the value \b MEMCFG_CERR_CPUREAD only. Other values are reserved.
//!
//! This function sets the correctable error interrupt flag.
//!
//! \note Note that only correctable errors may generate interrupts.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_forceCorrErrorInterrupt(uint32_t intFlags)
{
    //
    // Write the flags to the appropriate SET register.
    //
    EALLOW;

    HWREG(MEMORYERROR_BASE + MEMCFG_O_CEINTSET) = intFlags;

    EDIS;
}

//*****************************************************************************
//
//! Clears the RAM correctable error interrupt status.
//!
//! \param intFlags is a bit mask of the interrupt sources to be cleared. Can
//! take the value \b MEMCFG_CERR_CPUREAD only. Other values are reserved.
//!
//! This function clears the correctable error interrupt flag.
//!
//! \note Note that only correctable errors may generate interrupts.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_clearCorrErrorInterruptStatus(uint32_t intFlags)
{
    //
    // Clear the requested flags.
    //
    EALLOW;

    HWREG(MEMORYERROR_BASE + MEMCFG_O_CEINTCLR) |= intFlags;

    EDIS;
}

//*****************************************************************************
//
//! Gets the current correctable RAM error status.
//!
//! \return Returns the current error status, enumerated as a bit field of
//! \b MEMCFG_CERR_CPUREAD, \b MEMCFG_CERR_DMAREAD, or \b MEMCFG_CERR_CLA1READ
//
//*****************************************************************************
static inline uint32_t
MemCfg_getCorrErrorStatus(void)
{
    //
    // Read and return RAM error status flags.
    //
    return(HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRFLG));
}

//*****************************************************************************
//
//! Gets the current uncorrectable RAM error status.
//!
//! \return Returns the current error status, enumerated as a bit field of
//! \b MEMCFG_UCERR_CPUREAD, \b MEMCFG_UCERR_DMAREAD, \b MEMCFG_UCERR_CLA1READ,
//! or \b MEMCFG_UCERR_ECATMEMREAD.
//
//*****************************************************************************
static inline uint32_t
MemCfg_getUncorrErrorStatus(void)
{
    //
    // Read and return RAM error status flags.
    //
    return(HWREG(MEMORYERROR_BASE + MEMCFG_O_UCERRFLG));
}

//*****************************************************************************
//
//! Sets the specified correctable RAM error status flag.
//!
//! \param stsFlags is a bit mask of the error sources. This parameter can be
//! any of the following values:
//! \b MEMCFG_CERR_CPUREAD, \b MEMCFG_CERR_DMAREAD, or \b MEMCFG_CERR_CLA1READ
//!
//! This function sets the specified correctable RAM error status flag.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_forceCorrErrorStatus(uint32_t stsFlags)
{
    //
    // Write the flags to the appropriate SET register.
    //
    EALLOW;

    HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRSET) = stsFlags;

    EDIS;
}

//*****************************************************************************
//
//! Sets the specified uncorrectable RAM error status flag.
//!
//! \param stsFlags is a bit mask of the error sources. This parameter can be
//! any of the following values:
//! \b MEMCFG_UCERR_CPUREAD, \b MEMCFG_UCERR_DMAREAD, \b MEMCFG_UCERR_CLA1READ,
//! or \b MEMCFG_UCERR_ECATMEMREAD.
//!
//! This function sets the specified uncorrectable RAM error status flag.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_forceUncorrErrorStatus(uint32_t stsFlags)
{
    //
    // Write the flags to the appropriate SET register.
    //
    EALLOW;

    HWREG(MEMORYERROR_BASE + MEMCFG_O_UCERRSET) = stsFlags;

    EDIS;
}

//*****************************************************************************
//
//! Clears correctable RAM error flags.
//!
//! \param stsFlags is a bit mask of the status flags to be cleared.
//! This parameter can be any of the following :
//! \b MEMCFG_CERR_CPUREAD, \b MEMCFG_CERR_DMAREAD, or \b MEMCFG_CERR_CLA1READ
//!
//! This function clears the specified correctable RAM error flags.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_clearCorrErrorStatus(uint32_t stsFlags)
{
    //
    // Clear the requested flags.
    //
    EALLOW;

    HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRCLR) |= stsFlags;

    EDIS;
}

//*****************************************************************************
//
//! Clears uncorrectable RAM error flags.
//!
//! \param stsFlags is a bit mask of the status flags to be cleared.
//! This parameter can be any of the following :
//! \b MEMCFG_UCERR_CPUREAD, \b MEMCFG_UCERR_DMAREAD, \b MEMCFG_UCERR_CLA1READ,
//! or \b MEMCFG_UCERR_ECATMEMREAD.
//!
//! This function clears the specified uncorrectable RAM error flags.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_clearUncorrErrorStatus(uint32_t stsFlags)
{
    //
    // Clear the requested flags.
    //
    EALLOW;

    HWREG(MEMORYERROR_BASE + MEMCFG_O_UCERRCLR) |= stsFlags;

    EDIS;
}

//*****************************************************************************
//
//! Gets the diagnostics corr/uncorr memory error status in test mode.
//!
//! \return Returns the diagnostics error status in test mode. It can return
//! following values: \b MEMCFG_DIAGERR_UNCORR, and/or \b MEMCFG_DIAGERR_CORR.
//
//*****************************************************************************
static inline uint32_t
MemCfg_getDiagErrorStatus(void)
{
    //
    // Returns diag error status.
    //
    return(HWREG(TESTERROR_BASE + MEMCFG_O_CPU_RAM_TEST_ERROR_STS));
}

//*****************************************************************************
//
//! Clears the diagnostics corr/uncorr memory error status.
//!
//! \param stsFlags is a bit mask of the status flags to be cleared.
//!
//! This function clears the specified diagnostics corr/uncorr RAM/ROM memory
//! error flags in test mode. The param \e stsFlags can be any of the following
//! values: \b MEMCFG_DIAGERR_UNCORR, and/or \b MEMCFG_DIAGERR_CORR.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_clearDiagErrorStatus(uint32_t stsFlags)
{
    //
    // Clear diag error status.
    //
    HWREG(TESTERROR_BASE + MEMCFG_O_CPU_RAM_TEST_ERROR_STS_CLR ) |= stsFlags;
}

//*****************************************************************************
//
//! Gets address location of diagnostics corr/uncorr memory read/write error.
//!
//! \return Return address location of diagnostics corr/uncorr memory error
//! error in test mode.
//
//*****************************************************************************
static inline uint32_t
MemCfg_getDiagErrorAddress(void)
{
    //
    // Return diag error address.
    //
    return(HWREG(TESTERROR_BASE + MEMCFG_O_CPU_RAM_TEST_ERROR_ADDR));
}
//*****************************************************************************
//
//! Enables ROM wait state.
//!
//! This function enables the ROM wait state. This mean CPU accesses to ROM are
//! 1-wait.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_enableROMWaitState(void)
{
    //
    // Clear the disable bit.
    //
    EALLOW;

    HWREG(ROMWAITSTATE_BASE + MEMCFG_O_ROMWAITSTATE) &=
        ~((uint32_t)MEMCFG_ROMWAITSTATE_WSDISABLE);

    EDIS;
}

//*****************************************************************************
//
//! Disables ROM wait state.
//!
//! This function enables the ROM wait state. This mean CPU accesses to ROM are
//! 0-wait.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_disableROMWaitState(void)
{
    //
    // Set the disable bit.
    //
    EALLOW;

    HWREG(ROMWAITSTATE_BASE + MEMCFG_O_ROMWAITSTATE) |=
        MEMCFG_ROMWAITSTATE_WSDISABLE;

    EDIS;
}

//*****************************************************************************
//
//! Enables ROM prefetch.
//!
//! This function enables the ROM prefetch for both secure ROM and boot ROM.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_enableROMPrefetch(void)
{
    //
    // Set the enable bit.
    //
    EALLOW;

    HWREG(ROMPREFETCH_BASE + MEMCFG_O_ROMPREFETCH) |=
        MEMCFG_ROMPREFETCH_PFENABLE;

    EDIS;
}

//*****************************************************************************
//
//! Disables ROM prefetch.
//!
//! This function enables the ROM prefetch for both secure ROM and boot ROM.
//!
//! \return None.
//
//*****************************************************************************
static inline void
MemCfg_disableROMPrefetch(void)
{
    //
    // Clear the enable bit.
    //
    EALLOW;

    HWREG(ROMPREFETCH_BASE + MEMCFG_O_ROMPREFETCH) &=
        ~((uint32_t)MEMCFG_ROMPREFETCH_PFENABLE);

    EDIS;
}

//*****************************************************************************
//
//! Locks the writes to the configuration of specified memory sections.
//!
//! \param memSections is the logical OR of the sections to be configured.
//!
//! This function locks writes to the access protection and controller select
//! configuration of a memory section.That means calling MemCfg_setProtection()
//! or MemCfg_setLSRAMControllerSel() for a locked memory section will have no
//! effect until MemCfg_unlockConfig() is called.
//!
//! The \e memSections parameter is an OR of one of the following sets of
//! indicators:
//! - \b MEMCFG_SECT_M0 and \b MEMCFG_SECT_M1 or \b MEMCFG_SECT_DX_ALL
//! - \b MEMCFG_SECT_D0 and \b MEMCFG_SECT_D1 or \b MEMCFG_SECT_DX_ALL
//! - \b MEMCFG_SECT_LS0 through \b MEMCFG_SECT_LSx or \b MEMCFG_SECT_LSX_ALL
//! - \b MEMCFG_SECT_GS0 through \b MEMCFG_SECT_GSx or \b MEMCFG_SECT_GSX_ALL
//! - \b MEMCFG_SECT_MSGCPUTOCPU0, \b MEMCFG_SECT_MSGCPUTOCLA1,
//!   \b MEMCFG_SECT_MSGCLA1TOCPU, \b MEMCFG_SECT_MSGCLA1TODMA,
//!   \b MEMCFG_SECT_MSGDMATOCLA1, \b MEMCFG_SECT_MSGCPUTOCPU1,
//!   \b MEMCFG_SECT_MSGCPUTOCM0, \b MEMCFG_SECT_MSGCPUTOCM1, or use
//!   \b MEMCFG_SECT_MSGX_ALL to configure all possible message RAM sections.
//! - \b OR use \b MEMCFG_SECT_ALL to configure all possible sections.
//!
//! \return None.
//
//*****************************************************************************
extern void
MemCfg_lockConfig(uint32_t memSections);

//*****************************************************************************
//
//! Unlocks the writes to the configuration of a memory section.
//!
//! \param memSections is the logical OR of the sections to be configured.
//!
//! This function unlocks writes to the access protection and controller select
//! configuration of a memory section that has been locked using
//! MemCfg_lockConfig().
//!
//! The \e memSections parameter is an OR of one of the following sets of
//! indicators:
//! - \b MEMCFG_SECT_M0 and \b MEMCFG_SECT_M1 or \b MEMCFG_SECT_DX_ALL
//! - \b MEMCFG_SECT_D0 and \b MEMCFG_SECT_D1 or \b MEMCFG_SECT_DX_ALL
//! - \b MEMCFG_SECT_LS0 through \b MEMCFG_SECT_LSx or \b MEMCFG_SECT_LSX_ALL
//! - \b MEMCFG_SECT_GS0 through \b MEMCFG_SECT_GSx or \b MEMCFG_SECT_GSX_ALL
//! - \b MEMCFG_SECT_MSGCPUTOCPU0, \b MEMCFG_SECT_MSGCPUTOCLA1,
//!   \b MEMCFG_SECT_MSGCLA1TOCPU, \b MEMCFG_SECT_MSGCLA1TODMA,
//!   \b MEMCFG_SECT_MSGDMATOCLA1, \b MEMCFG_SECT_MSGCPUTOCPU1,
//!   \b MEMCFG_SECT_MSGCPUTOCM0, \b MEMCFG_SECT_MSGCPUTOCM1, or use
//!   \b MEMCFG_SECT_MSGX_ALL to configure all possible message RAM sections.
//! - \b OR use \b MEMCFG_SECT_ALL to configure all possible sections.
//!
//! \return None.
//
//*****************************************************************************
extern void
MemCfg_unlockConfig(uint32_t memSections);

//*****************************************************************************
//
//! Permanently locks writes to the configuration of a memory section.
//!
//! \param memSections is the logical OR of the sections to be configured.
//!
//! This function permanently locks writes to the access protection and controller
//! select configuration of a memory section. That means calling
//! MemCfg_setProtection() or MemCfg_setLSRAMControllerSel() for a locked memory
//! section will have no effect. To lock the configuration in a nonpermanent
//! way, use MemCfg_lockConfig().
//!
//! The \e memSections parameter is an OR of one of the following sets of
//! indicators:
//! - \b MEMCFG_SECT_M0 and \b MEMCFG_SECT_M1 or \b MEMCFG_SECT_DX_ALL
//! - \b MEMCFG_SECT_D0 and \b MEMCFG_SECT_D1 or \b MEMCFG_SECT_DX_ALL
//! - \b MEMCFG_SECT_LS0 through \b MEMCFG_SECT_LSx or \b MEMCFG_SECT_LSX_ALL
//! - \b MEMCFG_SECT_GS0 through \b MEMCFG_SECT_GSx or \b MEMCFG_SECT_GSX_ALL
//! - \b MEMCFG_SECT_MSGCPUTOCPU0, \b MEMCFG_SECT_MSGCPUTOCLA1,
//!   \b MEMCFG_SECT_MSGCLA1TOCPU, \b MEMCFG_SECT_MSGCLA1TODMA,
//!   \b MEMCFG_SECT_MSGDMATOCLA1, \b MEMCFG_SECT_MSGCPUTOCPU1,
//!   \b MEMCFG_SECT_MSGCPUTOCM0, \b MEMCFG_SECT_MSGCPUTOCM1, or use
//!   \b MEMCFG_SECT_MSGX_ALL to configure all possible message RAM sections.
//! - \b OR use \b MEMCFG_SECT_ALL to configure all possible sections.
//!
//! \return None.
//
//*****************************************************************************
extern void
MemCfg_commitConfig(uint32_t memSections);

//*****************************************************************************
//
//! Sets the access protection mode of a single memory section.
//!
//! \param memSection is the memory section to be configured.
//! \param protectMode is the logical OR of the settings to be applied.
//!
//! This function sets the access protection mode of a specified memory section.
//! The mode is passed into the \e protectMode parameter as the logical OR of
//! the following values:
//! - \b MEMCFG_PROT_ALLOWCPUWRITE or \b MEMCFG_PROT_BLOCKCPUWRITE - CPU write
//! - \b MEMCFG_PROT_ALLOWDMAWRITE or \b MEMCFG_PROT_BLOCKDMAWRITE - DMA write
//!
//! The \e memSection parameter is one of the following indicators:
//! - \b MEMCFG_SECT_M0 or \b MEMCFG_SECT_M1
//! - \b MEMCFG_SECT_D0 or \b MEMCFG_SECT_D1
//! - \b MEMCFG_SECT_LS0 through \b MEMCFG_SECT_LSx
//! - \b MEMCFG_SECT_GS0 through \b MEMCFG_SECT_GSx
//! - \b MEMCFG_SECT_MSGCPUTOCPU0 or \b MEMCFG_SECT_MSGCPUTOCPU1 or
//!   \b MEMCFG_SECT_MSGCPUTOCM0 or \b MEMCFG_SECT_MSGCPUTOCM1
//!
//! This function will have no effect if the associated registers have been
//! locked by MemCfg_lockConfig() or MemCfg_commitConfig() or if the memory
//! is configured as CLA program memory.
//!
//! \return None.
//
//*****************************************************************************
extern void
MemCfg_setProtection(uint32_t memSection, uint32_t protectMode);

//*****************************************************************************
//
//! Sets the controller of the specified LSxRAM section.
//!
//! \param ramSection is the LSxRAM section to be configured.
//! \param controllerSel is the sharing selection.
//!
//! This function sets the controller select configuration of the LSxRAM
//! section.
//! If the \e controllerSel parameter is \b MEMCFG_LSRAMCONTROLLER_CPU_ONLY,
//! the LSxRAM section passed into the \e ramSection parameter will be dedicated
//! to the CPU. If \b MEMCFG_LSRAMCONTROLLER_CPU_CLA1, the memory section will
//! be shared between the CPU and the CLA.
//!
//! The \e ramSection parameter should be a value from \b MEMCFG_SECT_LS0
//! through \b MEMCFG_SECT_LSx.
//!
//! This function will have no effect if the associated registers have been
//! locked by MemCfg_lockConfig() or MemCfg_commitConfig().
//!
//! \note This API only applies to LSxRAM.
//!
//! \return None.
//
//*****************************************************************************
extern void
MemCfg_setLSRAMControllerSel(uint32_t ramSection,
                             MemCfg_LSRAMControllerSel controllerSel);

//*****************************************************************************
//
//! Sets the controller of the specified GSxRAM section.
//!
//! \param ramSections is the logical OR of the sections to be configured.
//! \param controllerSel is the sharing selection.
//!
//! This function sets the controller select configuration of the GSxRAM section.If
//! the \e controllerSel parameter is \b MEMCFG_GSRAMCONTROLLER_CPU1, the GSRAM
//! sections passed into the \e ramSections parameter will be dedicated to
//! CPU1. If \b MEMCFG_GSRAMCONTROLLER_CPU2, the memory section will be dedicated
//! to CPU2.
//!
//! The \e ramSections parameter should be a logical OR of values from
//! \b MEMCFG_SECT_GS0 through \b MEMCFG_SECT_GSx.
//!
//! This function will have no effect if the associated registers have been
//! locked by MemCfg_lockConfig() or MemCfg_commitConfig().
//!
//! \note This API only applies to GSxRAM.
//!
//! \return None.
//
//*****************************************************************************
extern void
MemCfg_setGSRAMControllerSel(uint32_t ramSections,
                             MemCfg_GSRAMControllerSel controllerSel);

//*****************************************************************************
//
//! Locks the writes to the test mode configuration of specified memory
//! sections.
//!
//! \param memSections is the logical OR of the sections to be configured.
//!
//! This function locks writes to the test mode configuration of a RAM section.
//! That means calling MemCfg_setTestMode() for a locked RAM section will have
//! no effect until MemCfg_unlockTestConfig() is called.
//!
//! The \e memSections parameter is an OR of one of the following sets of
//! indicators:
//! - \b MEMCFG_SECT_M0, \b MEMCFG_SECT_M1, \b MEMCFG_SECT_D0,
//!   \b MEMCFG_SECT_D1 or use \b MEMCFG_SECT_DX_ALL
//! - \b MEMCFG_SECT_LS0 through \b MEMCFG_SECT_LSx or use
//!   \b MEMCFG_SECT_LSX_ALL to configure all possible LSRAM sections.
//! - \b MEMCFG_SECT_GS0 through \b MEMCFG_SECT_GSx or use
//!   \b MEMCFG_SECT_GSX_ALL to configure all possible GSRAM sections.
//! - \b MEMCFG_SECT_MSGCPUTOCLA1, \b MEMCFG_SECT_MSGCLA1TOCPU,
//!   \b MEMCFG_SECT_MSGCLA1TODMA, \b MEMCFG_SECT_MSGDMATOCLA1,
//!   \b MEMCFG_SECT_MSGCPUTOCPU0, \b MEMCFG_SECT_MSGCPUTOCPU1,
//!   \b MEMCFG_SECT_MSGCPUTOCM0, \b MEMCFG_SECT_MSGCPUTOCM1,
//!   , or use \b MEMCFG_SECT_MSGX_ALL
//! - \b MEMCFG_SECT_ROMBOOT, \b MEMCFG_SECT_ROMSECURE,
//!   \b MEMCFG_SECT_ROMCLADATA or \b MEMCFG_SECT_ROM_ALL to configure all
//!      possible ROM sections.
//! - \b MEMCFG_SECT_PERIMEM_ETHERCAT or MEMCFG_SECT_PERIMEM_ALL to configure
//!      all possible peal memories.ripher
//! - \b OR use \b MEMCFG_SECT_ALL to configure all possible sections.
//!
//! \return None.
//
//*****************************************************************************
extern void
MemCfg_lockTestConfig(uint32_t memSections);

//*****************************************************************************
//
//! Unlocks the writes to the test mode configuration of specified memory
//! sections.
//!
//! \param memSections is the logical OR of the sections to be configured.
//!
//! This function unlocks writes to the test mode configuration of a RAM
//! section that has been locked using MemCfg_lockTestConfig().
//!
//! The \e memSections parameter is an OR of one of the following sets of
//! indicators:
//! - \b MEMCFG_SECT_M0, \b MEMCFG_SECT_M1, \b MEMCFG_SECT_D0,
//!   \b MEMCFG_SECT_D1 or use \b MEMCFG_SECT_DX_ALL
//! - \b MEMCFG_SECT_LS0 through \b MEMCFG_SECT_LSx or use
//!   \b MEMCFG_SECT_LSX_ALL to configure all possible LSRAM sections.
//! - \b MEMCFG_SECT_GS0 through \b MEMCFG_SECT_GSx or use
//!   \b MEMCFG_SECT_GSX_ALL to configure all possible GSRAM sections.
//! - \b MEMCFG_SECT_MSGCPUTOCLA1, \b MEMCFG_SECT_MSGCLA1TOCPU,
//!   \b MEMCFG_SECT_MSGCLA1TODMA, \b MEMCFG_SECT_MSGDMATOCLA1,
//!   \b MEMCFG_SECT_MSGCPUTOCPU0, \b MEMCFG_SECT_MSGCPUTOCPU1,
//!   \b MEMCFG_SECT_MSGCPUTOCM0, \b MEMCFG_SECT_MSGCPUTOCM1,
//!   , or use \b MEMCFG_SECT_MSGX_ALL
//! - \b MEMCFG_SECT_ROMBOOT, \b MEMCFG_SECT_ROMSECURE,
//!   \b MEMCFG_SECT_ROMCLADATA or \b MEMCFG_SECT_ROM_ALL to configure all
//!      possible ROM sections.
//! - \b MEMCFG_SECT_PERIMEM_ETHERCAT or MEMCFG_SECT_PERIMEM_ALL to configure
//!      all possible peripheral memories.
//! - \b OR use \b MEMCFG_SECT_ALL to configure all possible sections.
//!
//! \return None.
//
//*****************************************************************************
extern void
MemCfg_unlockTestConfig(uint32_t memSections);

//*****************************************************************************
//
//! Sets the test mode of the specified memory section.
//!
//! \param memSection is the memory section to be configured.
//! \param testMode is the test mode selected.
//!
//! This function sets the test mode configuration of the RAM section. The
//! \e testMode parameter can take one of the following values:
//! - \b MEMCFG_TEST_FUNCTIONAL
//! - \b MEMCFG_TEST_WRITE_DATA
//! - \b MEMCFG_TEST_WRITE_ECC
//! - \b MEMCFG_TEST_WRITE_PARITY
//! - \b MEMCFG_TEST_FUNC_DIAG
//!
//! The \e memSection parameter is one of the following indicators:
//! - \b MEMCFG_SECT_M0, \b MEMCFG_SECT_M1
//! - \b MEMCFG_SECT_D0, \b MEMCFG_SECT_D1
//! - \b MEMCFG_SECT_LS0 through \b MEMCFG_SECT_LSx
//! - \b MEMCFG_SECT_GS0 through \b MEMCFG_SECT_GSx
//! - \b MEMCFG_SECT_MSGCPUTOCLA1, \b MEMCFG_SECT_MSGCLA1TOCPU,
//!   \b MEMCFG_SECT_MSGCLA1TODMA, \b MEMCFG_SECT_MSGDMATOCLA1,
//!   \b MEMCFG_SECT_MSGCPUTOCPU0, \b MEMCFG_SECT_MSGCPUTOCPU1,
//!   \b MEMCFG_SECT_MSGCPUTOCM0, \b MEMCFG_SECT_MSGCPUTOCM1,
//! - \b MEMCFG_SECT_ROMBOOT, \b MEMCFG_SECT_ROMSECURE,
//!   \b MEMCFG_SECT_ROMCLADATA
//! - \b MEMCFG_SECT_PERIMEM_ETHERCAT
//!
//! \return None.
//
//*****************************************************************************
extern void
MemCfg_setTestMode(uint32_t memSection, MemCfg_TestMode testMode);

//*****************************************************************************
//
//! Starts the initialization the specified RAM sections.
//!
//! \param ramSections is the logical OR of the sections to be initialized.
//!
//! This function starts the initialization of the specified RAM sections. Use
//! MemCfg_getInitStatus() to check if the initialization is done.
//!
//! The \e ramSections parameter is an OR of one of the following sets of
//! indicators:
//! - \b MEMCFG_SECT_M0, \b MEMCFG_SECT_M1, \b MEMCFG_SECT_D0,
//!   \b MEMCFG_SECT_D1, or \b MEMCFG_SECT_DX_ALL
//! - \b MEMCFG_SECT_LS0 through \b MEMCFG_SECT_LSx, or \b MEMCFG_SECT_LSX_ALL
//! - \b MEMCFG_SECT_GS0 through \b MEMCFG_SECT_GSx, or \b MEMCFG_SECT_GSX_ALL
//! - \b MEMCFG_SECT_MSGCPUTOCLA1, \b MEMCFG_SECT_MSGCLA1TOCPU,
//!   \b MEMCFG_SECT_MSGCLA1TODMA, \b MEMCFG_SECT_MSGDMATOCLA1,
//!   \b MEMCFG_SECT_MSGCPUTOCPU0, \b MEMCFG_SECT_MSGCPUTOCPU1,
//!   \b MEMCFG_SECT_MSGCPUTOCM0, \b MEMCFG_SECT_MSGCPUTOCM1,
//!   or \b MEMCFG_SECT_MSGX_ALL
//! - \b OR use \b MEMCFG_SECT_ALL to configure all possible sections.
//!
//! \return None.
//
//*****************************************************************************
extern void
MemCfg_initSections(uint32_t ramSections);

//*****************************************************************************
//
//! Get the status of initialized RAM sections.
//!
//! \param ramSections is the logical OR of the sections to be checked.
//!
//! This function gets the initialization status of the RAM sections specified
//! by the \e ramSections parameter.
//!
//! The \e ramSections parameter is an OR of one of the following sets of
//! indicators:
//! - \b MEMCFG_SECT_M0, \b MEMCFG_SECT_M1, \b MEMCFG_SECT_D0,
//!   \b MEMCFG_SECT_D1, or \b MEMCFG_SECT_DX_ALL
//! - \b MEMCFG_SECT_LS0 through \b MEMCFG_SECT_LSx, or \b MEMCFG_SECT_LSX_ALL
//! - \b MEMCFG_SECT_GS0 through \b MEMCFG_SECT_GSx, or \b MEMCFG_SECT_GSX_ALL
//! - \b MEMCFG_SECT_MSGCPUTOCLA1, \b MEMCFG_SECT_MSGCLA1TOCPU,
//!   \b MEMCFG_SECT_MSGCLA1TODMA, \b MEMCFG_SECT_MSGDMATOCLA1,
//!   \b MEMCFG_SECT_MSGCPUTOCPU0, \b MEMCFG_SECT_MSGCPUTOCPU1,
//!   \b MEMCFG_SECT_MSGCPUTOCM0, \b MEMCFG_SECT_MSGCPUTOCM1,
//!   or \b MEMCFG_SECT_MSGX_ALL
//! - \b OR use \b MEMCFG_SECT_ALL to get status of all possible sections.
//!
//! \note Use MemCfg_initSections() to start the initialization.
//!
//! \return Returns \b true if all the sections specified by \e ramSections
//! have been initialized and \b false if not.
//
//*****************************************************************************
extern bool
MemCfg_getInitStatus(uint32_t ramSections);

//*****************************************************************************
//
//! Get the violation address associated with a intFlag.
//!
//! \param intFlag is the type of access violation as indicated by ONE of
//! these values:
//!  - \b MEMCFG_NMVIOL_CPUREAD   - Non-controller CPU read access
//!  - \b MEMCFG_NMVIOL_CPUWRITE  - Non-controller CPU write access
//!  - \b MEMCFG_NMVIOL_CPUFETCH  - Non-controller CPU fetch access
//!  - \b MEMCFG_NMVIOL_DMAWRITE  - Non-controller DMA write access
//!  - \b MEMCFG_NMVIOL_CLA1READ  - Non-controller CLA1 read access
//!  - \b MEMCFG_NMVIOL_CLA1WRITE - Non-controller CLA1 write access
//!  - \b MEMCFG_NMVIOL_CLA1FETCH - Non-controller CLA1 fetch access
//!  - \b MEMCFG_NMVIOL_DMAREAD   - Non-controller DMA read access
//!  - \b MEMCFG_MVIOL_CPUFETCH   - Controller CPU fetch access
//!  - \b MEMCFG_MVIOL_CPUWRITE   - Controller CPU write access
//!  - \b MEMCFG_MVIOL_DMAWRITE   - Controller DMA write access
//!
//! \return Returns the violation address associated with the \e intFlag.
//
//*****************************************************************************
extern uint32_t
MemCfg_getViolationAddress(uint32_t intFlag);

//*****************************************************************************
//
//! Get the correctable error address associated with a stsFlag.
//!
//! \param stsFlag is the type of error to which the returned address will
//! correspond. It can take following values: \b MEMCFG_CERR_CPUREAD,
//! \b MEMCFG_CERR_CLA1READ, \b MEMCFG_CERR_DMAREAD.
//!
//! \return Returns the error address associated with the stsFlag.
//
//*****************************************************************************
extern uint32_t
MemCfg_getCorrErrorAddress(uint32_t stsFlag);

//*****************************************************************************
//
//! Get the uncorrectable error address associated with a stsFlag.
//!
//! \param stsFlag is the type of error to which the returned address will
//! correspond. It may be passed one of these values:
//! \b MEMCFG_UCERR_CPUREAD, \b MEMCFG_UCERR_DMAREAD,
//! \b MEMCFG_UCERR_CLA1READ, or \b MEMCFG_UCERR_ECATMEMREAD values.
//!
//! \return Returns the error address associated with the stsFlag.
//
//*****************************************************************************
extern uint32_t
MemCfg_getUncorrErrorAddress(uint32_t stsFlag);

//*****************************************************************************
//
//! Forces parity error in the selected memory.
//!
//! \param memSections is the logical OR of the sections to be tested.
//!
//! This function forces the parity error in the memories specified
//! by the \e memSections parameter.
//!
//! The \e memSections parameter is an OR of one of the following sets of
//! indicators:
//! - \b MEMCFG_SECT_ROMBOOT, \b MEMCFG_SECT_ROMSECURE ,
//!   \b MEMCFG_SECT_ROMCLADATA, \b MEMCFG_SECT_ROM_ALL.
//! - \b MEMCFG_SECT_PERIMEM_ETHERCAT, or \b MEMCFG_SECT_PERIMEM_ALL
//! - \b OR use \b MEMCFG_SECT_ALL to force parity error in all possible
//!      memory sections.
//!
//! \return None.
//
//*****************************************************************************
extern void
MemCfg_forceMemError(uint32_t memSections);

//*****************************************************************************
//
//! Enables test mode for selected peripheral memories.
//!
//! \param memSections is the logical OR of the type of peripheral memories
//! to be tested.
//!
//! This function enables test mode for selected peripheral memories specified
//! by the \e memTypes parameter.
//!
//! The \e memTypes parameter is an OR of one of the following sets of
//! indicators:
//! - \b MEMCFG_SECT_PERIMEM_ETHERCAT, or \b MEMCFG_SECT_PERIMEM_ALL
//!
//! \return None.
//
//*****************************************************************************
extern void
MemCfg_enablePeriMemTestMode(uint32_t memSections);

//*****************************************************************************
//
//! Disables the test mode for selected peripheral memories.
//!
//! \param memSections is the logical OR of the type of peripheral memories
//! to be tested.
//!
//! This function disables test mode for selected peripheral memories specified
//! by the \e memSections parameter.
//!
//! The \e memTypes parameter is an OR of one of the following sets of
//! indicators:
//! - \b MEMCFG_SECT_PERIMEM_ETHERCAT, or \b MEMCFG_SECT_PERIMEM_ALL
//!
//! \return None.
//
//*****************************************************************************
extern void
MemCfg_disablePeriMemTestMode(uint32_t memSections);

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

#endif // MEMCFG_H
