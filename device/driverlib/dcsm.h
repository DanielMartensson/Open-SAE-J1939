//#############################################################################
//
// FILE:   dcsm.h
//
// TITLE:  C28x Driver for the DCSM security module.
//
//#############################################################################
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
//#############################################################################

#ifndef DCSM_H
#define DCSM_H

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
//! \addtogroup dcsm_api DCSM
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_dcsm.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "cpu.h"
#include "debug.h"

//*****************************************************************************
//
// Defines for the unlockZone1CSM() and unlockZone2CSM().
// These are not parameters for any function.
// These are not intended for application code.
//
//*****************************************************************************

#define DCSM_O_Z1_CSMPSWD0              0x00U //!< Z1 CSMPSWD0 offset
#define DCSM_O_Z1_CSMPSWD1              0x02U //!< Z1 CSMPSWD1 offset
#define DCSM_O_Z1_CSMPSWD2              0x04U //!< Z1 CSMPSWD2 offset
#define DCSM_O_Z1_CSMPSWD3              0x06U //!< Z1 CSMPSWD3 offset
#define DCSM_O_Z2_CSMPSWD0              0x00U //!< Z2 CSMPSWD0 offset
#define DCSM_O_Z2_CSMPSWD1              0x02U //!< Z2 CSMPSWD1 offset
#define DCSM_O_Z2_CSMPSWD2              0x04U //!< Z2 CSMPSWD2 offset
#define DCSM_O_Z2_CSMPSWD3              0x06U //!< Z2 CSMPSWD3 offset

//*****************************************************************************
//
// Register key defines.
//
//*****************************************************************************
#define FLSEM_KEY                       0xA5U //!< Zone semaphore key
#define DCSM_FORCE_SECERR_KEY                0x5A5AU

//*****************************************************************************
//
//! Data structures to hold password keys.
//
//*****************************************************************************
typedef struct
{
    uint32_t csmKey0;
    uint32_t csmKey1;
    uint32_t csmKey2;
    uint32_t csmKey3;
} DCSM_CSMPasswordKey;

//*****************************************************************************
//
//! Values to distinguish the status of RAM or FLASH sectors. These values
//! describe which zone the memory location belongs too.
//! These values can be returned from DCSM_getRAMZone(),
//! DCSM_getFlashSectorZone().
//
//*****************************************************************************
typedef enum
{
    DCSM_MEMORY_INACCESSIBLE, //!< Inaccessible
    DCSM_MEMORY_ZONE1,        //!< Zone 1
    DCSM_MEMORY_ZONE2,        //!< Zone 2
    DCSM_MEMORY_FULL_ACCESS   //!< Full access
} DCSM_MemoryStatus;

//*****************************************************************************
//
//! Values to pass to DCSM_claimZoneSemaphore(). These values are used
//! to describe the zone that can write to Flash Wrapper registers.
//
//*****************************************************************************
typedef enum
{
    DCSM_FLSEM_ZONE1 = 0x01U, //!< Flash semaphore Zone 1
    DCSM_FLSEM_ZONE2 = 0x02U  //!< Flash semaphore Zone 2
} DCSM_SemaphoreZone;

//*****************************************************************************
//
//! Values to distinguish the security status of the zones.
//! These values can be returned from DCSM_getZone1CSMSecurityStatus(),
//! DCSM_getZone2CSMSecurityStatus().
//
//*****************************************************************************
typedef enum
{
    DCSM_STATUS_SECURE,   //!< Secure
    DCSM_STATUS_UNSECURE, //!< Unsecure
    DCSM_STATUS_LOCKED,   //!< Locked
    DCSM_STATUS_BLOCKED   //!< Blocked
} DCSM_SecurityStatus;

//*****************************************************************************
//
// Values to distinguish the status of the Control Registers. These values
// describe can be used with the return values of
// DCSM_getZone1ControlStatus(), and DCSM_getZone2ControlStatus().
//
//*****************************************************************************
#define DCSM_ALLZERO     0x08U  //!< CSM Passwords all zeros
#define DCSM_ALLONE      0x10U  //!< CSM Passwords all ones
#define DCSM_UNSECURE    0x20U  //!< Zone is secure/unsecure
#define DCSM_ARMED       0x40U  //!< CSM is armed

//*****************************************************************************
//
//! Values to decribe the EXEONLY Status.
//! These values are returned from  to DCSM_getZone1RAMEXEStatus(),
//! DCSM_getZone2RAMEXEStatus(), DCSM_getZone1FlashEXEStatus(),
//! DCSM_getZone2FlashEXEStatus().
//
//*****************************************************************************
typedef enum
{
    DCSM_PROTECTED,      //!< Protected
    DCSM_UNPROTECTED,    //!< Unprotected
    DCSM_INCORRECT_ZONE  //!< Incorrect Zone
}DCSM_EXEOnlyStatus;

//*****************************************************************************
//
//! Values to distinguish RAM Module.
//! These values can be passed to DCSM_getZone1RAMEXEStatus()
//! DCSM_getZone2RAMEXEStatus(), DCSM_getRAMZone().
//
//*****************************************************************************
typedef enum
{
    //
    //C28x RAMs
    //
    DCSM_RAMLS0, //!< RAMLS0
    DCSM_RAMLS1, //!< RAMLS1
    DCSM_RAMLS2, //!< RAMLS2
    DCSM_RAMLS3, //!< RAMLS3
    DCSM_RAMLS4, //!< RAMLS4
    DCSM_RAMLS5, //!< RAMLS5
    DCSM_RAMLS6, //!< RAMLS6
    DCSM_RAMLS7, //!< RAMLS7
    DCSM_RAMD0,  //!< RAMD0
    DCSM_RAMD1,  //!< RAMD1
    DCSM_C28_RAM_END,
    //
    //CM RAMs
    //
    DCSM_RAM_CM_C0     , //!< RAM CM C0
    DCSM_RAM_CM_C1     , //!< RAM CM C1
    DCSM_RAM1_CPU1_CM  , //!< CPU1 to CM Message RAM 1
    DCSM_RAM2_CPU1_CM  , //!< CPU1 to CM Message RAM 2
    DCSM_RAM1_CM_CPU1  , //!< CM to CPU1 Message RAM 1
    DCSM_RAM2_CM_CPU1  , //!< CM to CPU1 Message RAM 2
    DCSM_RAM1_CPU2_CM  , //!< CPU2 to CM Message RAM 1
    DCSM_RAM2_CPU2_CM  , //!< CPU2 to CM Message RAM 2
    DCSM_RAM1_CM_CPU2  , //!< CM to CPU2 Message RAM 1
    DCSM_RAM2_CM_CPU2  , //!< CM to CPU2 Message RAM 2
    DCSM_RAM1_CPU1_CPU2, //!< CPU1 to CPU2 Message RAM 1
    DCSM_RAM2_CPU1_CPU2, //!< CPU1 to CPU2 Message RAM 2
    DCSM_RAM1_CPU2_CPU1, //!< CPU2 to CPU1 Message RAM 1
    DCSM_RAM2_CPU2_CPU1, //!< CPU2 to CPU1 Message RAM 2
    DCSM_CM_RAM_END,
} DCSM_RAMModule;

//*****************************************************************************
//
//! Values to distinguish Flash Sector.
//! These values can be passed to DCSM_getZone1FlashEXEStatus()
//! DCSM_getZone2FlashEXEStatus(), DCSM_getFlashSectorZone().
//
//*****************************************************************************
typedef enum
{
    DCSM_SECTOR_0,  //!< Sector 0
    DCSM_SECTOR_1,  //!< Sector 1
    DCSM_SECTOR_2,  //!< Sector 2
    DCSM_SECTOR_3,  //!< Sector 3
    DCSM_SECTOR_4,  //!< Sector 4
    DCSM_SECTOR_5,  //!< Sector 5
    DCSM_SECTOR_6,  //!< Sector 6
    DCSM_SECTOR_7,  //!< Sector 7
    DCSM_SECTOR_8,  //!< Sector 8
    DCSM_SECTOR_9,  //!< Sector 9
    DCSM_SECTOR_10, //!< Sector 10
    DCSM_SECTOR_11, //!< Sector 11
    DCSM_SECTOR_12, //!< Sector 12
    DCSM_SECTOR_13  //!< Sector 13
} DCSM_Sector;


//*****************************************************************************
//
//! The following are values that can be passed to DCSM_getRAMZone(),
//! DCSM_getZone2FlashEXEStatus() ,DCSM_getZone1FlashEXEStatus() &
//! DCSM_getFlashSectorZone() as \e cpuInst parameter.
//
//*****************************************************************************
typedef enum
{
    //! To access CPU1 Memory
    DCSM_CPUSEL_CPU1 = 0x0U,
    //! To access CPU2 Memory
    DCSM_CPUSEL_CPU2 = 0x2U,
    //! To access CM Memory
    DCSM_CPUSEL_CM = 0x1U

} DCSM_CPUSel;

//*****************************************************************************
//
//! The following are values that can be passed to
//! DCSM_getZone1OTPSecureLockStatus() & DCSM_getZone2OTPSecureLockStatus()
//! as \e lockType parameter.
//
//*****************************************************************************
typedef enum
{
    //! JTAG Lock Status
    DCSM_OTPSECLOCK_JTAG = 0x0U,
    //! Zone Password Lock
    DCSM_OTPSECLOCK_PSWDLOCK = 0x1U,
    //! Zone CRC Lock
    DCSM_OTPSECLOCK_CRCLOCK = 0x2U,

} DCSM_OTPLock;

//*****************************************************************************
//
// Defines for the EXEONLYSECTxR register.
// These values can be used in the DCSM_getZone1FlashEXEStatus() &
// DCSM_getZone2FlashEXEStatus().
//
//*****************************************************************************

#define DCSM_EXEONLYSECTR_M             0x0000FFFFU
#define DCSM_EXEONLYSECTR_S             16U

//*****************************************************************************
//
// Defines for the Z1_CR  & Z2_CR register.
// These values can be used in the DCSM_getZone1ControlStatus() &
// DCSM_getZone2ControlStatus().
//
//*****************************************************************************

#define DCSM_ZX_CR_S                    16U
//*****************************************************************************
//
// Defines for the EXEONLYRAM1R  register for Zone 1 and Zone 2.
// These values can be used in the DCSM_getZone1RAMEXEStatus(),
// DCSM_getZone1RAMEXEStatus().
//
//*****************************************************************************
#define DCSM_Z1_EXEONLYRAM1R_CM         16U
#define DCSM_Z1_EXEONLYRAM1R_CPU2       31U
#define DCSM_Z2_EXEONLYRAM1R_CM         16U
#define DCSM_Z2_EXEONLYRAM1R_CPU2       31U

//*****************************************************************************
//
// Defines for the OTPSECLOCK register.
// These values can be used in the DCSM_getZone1OTPSecureLockStatus()
// & DCSM_getZone2OTPSecureLockStatus()
//
//*****************************************************************************

#define DCSM_Z1_OTPSECLOCK_JTAGLOCK_M             0x1U
#define DCSM_Z1_OTPSECLOCK_JTAGLOCK_S             0U
#define DCSM_OTPSECLOCK_INVALID                   0xFFFFU

//*****************************************************************************
//
// DCSM functions
//
//*****************************************************************************

//*****************************************************************************
//
//! Secures zone 1 by setting the FORCESEC bit of Z1_CR register
//!
//! This function resets the state of the zone. If the zone is unlocked,
//! it will lock(secure) the zone and also reset all the bits in the
//! Control Register.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCSM_secureZone1(void)
{
    //
    // Write to the FORCESEC bit.
    //
    HWREG(DCSM_Z1_BASE + DCSM_O_Z1_CR)|= DCSM_Z1_CR_FORCESEC;
}

//*****************************************************************************
//
//! Secures zone 2 by setting the FORCESEC bit of Z2_CR register
//!
//! This function resets the state of the zone. If the zone is unlocked,
//! it will lock(secure) the zone and also reset all the bits in the
//! Control Register.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCSM_secureZone2(void)
{
    //
    // Write to the FORCESEC bit.
    //
    HWREG(DCSM_Z2_BASE + DCSM_O_Z2_CR)|= DCSM_Z2_CR_FORCESEC;
}

//*****************************************************************************
//
//! Returns the CSM security status of zone 1
//!
//! This function returns the security status of zone 1 CSM
//!
//! \return Returns security status as an enumerated type DCSM_SecurityStatus.
//
//*****************************************************************************
static inline DCSM_SecurityStatus
DCSM_getZone1CSMSecurityStatus(void)
{
    uint32_t status;
    DCSM_SecurityStatus returnStatus;
    status = HWREG(DCSM_Z1_BASE + DCSM_O_Z1_CR);

    //
    // if ARMED bit is set and UNSECURED bit is set then CSM is unsecured.
    // Else it is secure.
    //
    if(((status & DCSM_Z1_CR_ARMED) != 0U) &&
       ((status & DCSM_Z1_CR_UNSECURE) != 0U))
    {
        returnStatus = DCSM_STATUS_UNSECURE;
    }
    else if((status & DCSM_Z1_CR_ALLONE) == DCSM_Z1_CR_ALLONE)
    {
        returnStatus = DCSM_STATUS_BLOCKED;
    }
    else if((status & DCSM_Z1_CR_ALLZERO) == DCSM_Z1_CR_ALLZERO)
    {
        returnStatus = DCSM_STATUS_LOCKED;
    }
    else
    {
        returnStatus = DCSM_STATUS_SECURE;
    }

    return(returnStatus);
}

//*****************************************************************************
//
//! Returns the CSM security status of zone 2
//!
//! This function returns the security status of zone 2 CSM
//!
//! \return Returns security status as an enumerated type DCSM_SecurityStatus.
//
//*****************************************************************************
static inline DCSM_SecurityStatus
DCSM_getZone2CSMSecurityStatus(void)
{
    uint32_t status;
    DCSM_SecurityStatus returnStatus;
    status = HWREG(DCSM_Z2_BASE + DCSM_O_Z2_CR);

    //
    // if ARMED bit is set and UNSECURED bit is set then CSM is unsecured.
    // Else it is secure.
    //
    if(((status & DCSM_Z2_CR_ARMED) != 0U) &&
       ((status & DCSM_Z2_CR_UNSECURE) != 0U))
    {
        returnStatus = DCSM_STATUS_UNSECURE;
    }
    else if((status & DCSM_Z2_CR_ALLONE) == DCSM_Z2_CR_ALLONE)
    {
        returnStatus = DCSM_STATUS_BLOCKED;
    }
    else if((status & DCSM_Z2_CR_ALLZERO) == DCSM_Z2_CR_ALLZERO)
    {
        returnStatus = DCSM_STATUS_LOCKED;
    }
    else
    {
        returnStatus = DCSM_STATUS_SECURE;
    }

    return(returnStatus);
}

//*****************************************************************************
//
//! Returns the Control Status of zone 1
//!
//! This function returns the Control Status of zone 1 CSM
//!
//! \return Returns the contents of the Control Register which can be
//! used with provided defines.
//
//*****************************************************************************
static inline uint16_t
DCSM_getZone1ControlStatus(void)
{
    uint32_t stat;
    //
    // Return the contents of the CR register.
    //

    stat = ((HWREG(DCSM_Z1_BASE + DCSM_O_Z1_CR)) >> DCSM_ZX_CR_S);

    return((uint16_t)stat);

}

//*****************************************************************************
//
//! Returns the Control Status of zone 2
//!
//! This function returns the Control Status of zone 2 CSM
//!
//! \return Returns the contents of the Control Register which can be
//! used with the provided defines.
//
//*****************************************************************************
static inline uint16_t
DCSM_getZone2ControlStatus(void)
{
    uint32_t stat;
    //
    // Return the contents of the CR register.
    //
    stat = ((HWREG(DCSM_Z2_BASE + DCSM_O_Z2_CR)) >> DCSM_ZX_CR_S);

    return((uint16_t)stat);
}

//*****************************************************************************
//
//! Returns the security zone a RAM section belongs to
//!
//! \param module is the RAM module value. Valid values are type DCSM_RAMModule
//! C28x RAMs :
//! - \b DCSM_RAMLS0
//! - \b DCSM_RAMLS1
//! - \b DCSM_RAMLS2
//! - \b DCSM_RAMLS3
//! - \b DCSM_RAMLS4
//! - \b DCSM_RAMLS5
//! - \b DCSM_RAMLS6
//! - \b DCSM_RAMLS7
//! - \b DCSM_RAMD0
//! - \b DCSM_RAMD1
//! CM RAMs :
//! - \b DCSM_RAM_CM_C0
//! - \b DCSM_RAM_CM_C1
//! - \b DCSM_RAM1_CPU1_CM
//! - \b DCSM_RAM2_CPU1_CM
//! - \b DCSM_RAM1_CM_CPU1
//! - \b DCSM_RAM2_CM_CPU1
//! - \b DCSM_RAM1_CPU2_CM
//! - \b DCSM_RAM2_CPU2_CM
//! - \b DCSM_RAM1_CM_CPU2
//! - \b DCSM_RAM2_CM_CPU2
//! - \b DCSM_RAM1_CPU1_CPU2
//! - \b DCSM_RAM2_CPU1_CPU2
//! - \b DCSM_RAM1_CPU2_CPU1
//! - \b DCSM_RAM2_CPU2_CPU1
//!
//! \param cpuInst is the CPU whose RAM Section needs to be accessed .
//!
//! The \e cpuInst parameter can have one of the following values of the
//! DCSM_CPUSel type:
//! - \b DCSM_CPUSEL_CPU1 -  To access CPU1 RAM
//! - \b DCSM_CPUSEL_CPU2 - To access CPU2 RAM
//! - \b DCSM_CPUSEL_CM - To access CM RAM
//!
//! \note  DCSM_CPUSEL_CPU1 & DCSM_CPUSEL_CPU2 can only be used with
//!  C28x RAMs (in DCSM_RAMModule) & DCSM_CPUSEL_CM can only be used with
//!  CM RAMs(in DCSM_RAMModule)
//!
//! This function returns the security zone a RAM section belongs to.
//!
//! \return Returns DCSM_MEMORY_INACCESSIBLE if the section is inaccessible,
//! DCSM_MEMORY_ZONE1 if the section belongs to zone 1, DCSM_MEMORY_ZONE2 if
//! the section belongs to zone 2 and DCSM_MEMORY_FULL_ACCESS if the section
//! doesn't  belong to any zone (or if the section is unsecure).
//
//*****************************************************************************
static inline DCSM_MemoryStatus
DCSM_getRAMZone(DCSM_RAMModule module, DCSM_CPUSel cpuInst)
{
    uint16_t shift = (uint16_t)module * 2U;
    uint32_t RAMStatus;
    uint16_t moduleCM;

    //
    // Check the arguments.
    //
    if((cpuInst == DCSM_CPUSEL_CPU1) || (cpuInst == DCSM_CPUSEL_CPU2))
    {
        ASSERT(module < DCSM_C28_RAM_END);
    }

    if(cpuInst == DCSM_CPUSEL_CM)
    {
        ASSERT((module > DCSM_C28_RAM_END) &&
               (module < DCSM_CM_RAM_END));

        moduleCM  = (uint16_t)module - ((uint16_t)DCSM_C28_RAM_END + 1U);
        shift = moduleCM * 2U;
    }

    //
    //Read the RAMSTAT register for the specific RAM Module.
    //

    RAMStatus = ((HWREG(DCSMCOMMON_BASE + DCSM_O_RAMSTAT1
                                        + (2U * (uint16_t)cpuInst)) >> shift)
                                        & 0x03U);
    return((DCSM_MemoryStatus)RAMStatus);

}

//*****************************************************************************
//
//! Returns the security zone a flash sector belongs to
//!
//! \param sector is the flash sector value.  Use DCSM_Sector type.
//! \param cpuInst is the CPU whose Flash Sector needs to be accessed .
//!
//! The \e cpuInst parameter can have one of the following values of the
//! DCSM_CPUSel type:
//! - \b DCSM_CPUSEL_CPU1 -  To access CPU1 Flash sectors
//! - \b DCSM_CPUSEL_CPU2 - To access CPU2 Flash sectors
//! - \b DCSM_CPUSEL_CM - To access CM Flash sectors
//!
//! This function returns the security zone a flash sector belongs to.
//!
//! \return Returns DCSM_MEMORY_INACCESSIBLE if the section is inaccessible ,
//! DCSM_MEMORY_ZONE1 if the section belongs to zone 1, DCSM_MEMORY_ZONE2 if
//! the section belongs to zone 2 and DCSM_MEMORY_FULL_ACCESS if the section
//! doesn't  belong to any zone (or if the section is unsecure)..
//
//*****************************************************************************
static inline DCSM_MemoryStatus
DCSM_getFlashSectorZone(DCSM_Sector sector, DCSM_CPUSel cpuInst)
{
    uint32_t sectStat;
    uint16_t shift;

    //
    // Get the Sector status register for the specific bank
    //
    sectStat = HWREG(DCSMCOMMON_BASE + DCSM_O_SECTSTAT1 +
                    (2U *(uint16_t)cpuInst));
    shift = (uint16_t)sector * 2U;

    //
    //Read the SECTSTAT register for the specific Flash Sector.
    //
    return((DCSM_MemoryStatus)((uint16_t)((sectStat >> shift) & 0x3U)));
}

//*****************************************************************************
//
//! Read Zone 1 Link Pointer Error
//!
//! A non-zero value indicates an error on the bit position that is set to 1.
//!
//! \return Returns the value of the Zone 1 Link Pointer error.
//
//*****************************************************************************
static inline uint32_t
DCSM_getZone1LinkPointerError(void)
{
    //
    // Return the LinkPointer Error for specific bank
    //
    return(HWREG(DCSM_Z1_BASE + DCSM_O_Z1_LINKPOINTERERR));
}

//*****************************************************************************
//
//! Read Zone 2 Link Pointer Error
//!
//! A non-zero value indicates an error on the bit position that is set to 1.
//!
//! \return Returns the value of the Zone 2 Link Pointer error.
//
//*****************************************************************************
static inline uint32_t
DCSM_getZone2LinkPointerError(void)
{
    //
    // Return the LinkPointer Error for specific bank
    //
    return(HWREG(DCSM_Z2_BASE + DCSM_O_Z2_LINKPOINTERERR));
}

//*****************************************************************************
//
//! Get the status of the security configuration load from USER-OTP or sector
//! error status
//!
//! \return Returns 0 if no error in loading security information from
//! USER-OTP, 1 if an error has occurred in the load from USER-OTP.
//
//*****************************************************************************
static inline bool
DCSM_getFlashErrorStatus(void)
{
    return((bool)(HWREG(DCSMCOMMON_BASE + DCSM_O_SECERRSTAT) &
                  DCSM_SECERRSTAT_ERR));
}

//*****************************************************************************
//
//! Clear the Flash Error Status bit
//!
//! Write a '1' to the clear bit to clear the sector error status bit.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCSM_clearFlashErrorStatus(void)
{
    HWREG(DCSMCOMMON_BASE + DCSM_O_SECERRCLR) |= DCSM_SECERRCLR_ERR;
}

//*****************************************************************************
//
//! Set the force Flash Error Status bit
//!
//! Write a '1' to force bit to set the sector error status bit.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCSM_forceFlashErrorStatus(void)
{
    HWREG(DCSMCOMMON_BASE +
          DCSM_O_SECERRFRC) = DCSM_SECERRFRC_ERR |
                              ((uint32_t)DCSM_FORCE_SECERR_KEY
                               << DCSM_SECERRFRC_KEY_S);
}

//*****************************************************************************
//
//! Returns the OTP secure Lock status of zone 1
//!
//! \param lockType is the  OTP secure Lock feature type .
//!
//! The \e lockType parameter can have one of the following values of the
//! DCSM_CPUSel type:
//! - \b DCSM_OTPSECLOCK_JTAG -  JTAG Lock Status
//! - \b DCSM_OTPSECLOCK_CRCLOCK - Zone CRC Lock
//! - \b DCSM_OTPSECLOCK_PSWDLOCK - Zone Password Lock
//!
//! This function takes in a valid OTP secure Lock feature type and
//! returns the status of zone 1 lock feature
//!
//! \return Returns security lock status can be:
//! For JTAG lock :  0 - JTAG is not locked , 1 - JTAG is locked
//!
//! For Zone Password Lock : 1111 - CSM Pwd locations in the OTP are not
//! protected and can be read from the debugger as well as code running
//! from anywhere.
//! Other Value : CSM Pwd locations in the OTP are protected and can't be read
//! without unlocking CSM of that zone.
//!
//! For Zone CRC Lock : 1111 : VCU has ability to calculate CRC
//! on secure memories.
//! Other Value : VCU doesn't have the ability to calculate CRC on
//! secure memories.
//
//*****************************************************************************
static inline uint32_t
DCSM_getZone1OTPSecureLockStatus(DCSM_OTPLock lockType)
{
    uint32_t status, returnStatus;
    status = HWREG(DCSM_Z1_BASE + DCSM_O_Z1_OTPSECLOCK);

    //
    // Reflects the state of the OTP Sec LOCK feature.
    //
    if(lockType == DCSM_OTPSECLOCK_JTAG)
    {
        returnStatus = (status & DCSM_Z1_OTPSECLOCK_JTAGLOCK_M) >>
                        DCSM_Z1_OTPSECLOCK_JTAGLOCK_S;
    }
    else if(lockType == DCSM_OTPSECLOCK_CRCLOCK)
    {
        returnStatus = (status & DCSM_Z1_OTPSECLOCK_CRCLOCK_M) >>
                        DCSM_Z1_OTPSECLOCK_CRCLOCK_S;
    }
    else if(lockType == DCSM_OTPSECLOCK_PSWDLOCK)
    {
        returnStatus = (status & DCSM_Z1_OTPSECLOCK_PSWDLOCK_M) >>
                        DCSM_Z1_OTPSECLOCK_PSWDLOCK_S;
    }
    else
    {
        returnStatus = (uint32_t)DCSM_OTPSECLOCK_INVALID;
    }

    return(returnStatus);
}

//*****************************************************************************
//
//! Returns the OTP secure Lock status of zone 2
//!
//! \param lockType is the  OTP secure Lock feature type .
//!
//! The \e lockType parameter can have one of the following values of the
//! DCSM_CPUSel type:
//! - \b DCSM_OTPSECLOCK_CRCLOCK - Zone CRC Lock
//! - \b DCSM_OTPSECLOCK_PSWDLOCK - Zone Password Lock
//!
//! This function takes in a valid OTP secure Lock feature type and
//! returns the status of zone 2 lock feature
//!
//! \return Returns security lock status can be:
//!
//! For Zone Password Lock : 1111 - CSM Pwd locations in the OTP are not
//! protected and can be read from the debugger as well as code running
//! from anywhere.
//! Other Value : CSM Pwd locations in the OTP are protected and can't be read
//! without unlocking CSM of that zone.
//!
//! For Zone CRC Lock : 1111 : VCU has ability to calculate CRC
//! on secure memories.
//! Other Value : VCU doesn't have the ability to calculate CRC on
//! secure memories.
//
//*****************************************************************************
static inline uint32_t
DCSM_getZone2OTPSecureLockStatus(DCSM_OTPLock lockType)
{
    uint32_t status, returnStatus;
    status = HWREG(DCSM_Z2_BASE + DCSM_O_Z2_OTPSECLOCK);

    //
    // Reflects the state of the OTP Sec LOCK feature.
    //
    if(lockType == DCSM_OTPSECLOCK_CRCLOCK)
    {
        returnStatus = (status & DCSM_Z2_OTPSECLOCK_CRCLOCK_M) >>
                        DCSM_Z2_OTPSECLOCK_CRCLOCK_S;
    }
    else if(lockType == DCSM_OTPSECLOCK_PSWDLOCK)
    {
        returnStatus = (status & DCSM_Z2_OTPSECLOCK_PSWDLOCK_M) >>
                        DCSM_Z2_OTPSECLOCK_PSWDLOCK_S;
    }
    else
    {
        returnStatus = (uint32_t)DCSM_OTPSECLOCK_INVALID;
    }

    return(returnStatus);
}

//*****************************************************************************
//
//! Unlocks Zone 1 CSM.
//!
//! \param psCMDKey is a pointer to the DCSM_CSMPasswordKey struct that has the
//! CSM  password for zone 1.
//!
//! This function unlocks the CSM password. It first reads the
//! four password locations in the User OTP. If any of the password values is
//! different from 0xFFFFFFFF, it unlocks the device by writing the provided
//! passwords into CSM Key registers
//!
//! \return None.
//!
//! \note This function should not be called in an actual application,
//! should only be used for once to program the OTP memory. Ensure flash data
//! cache is disabled before calling this function(Flash_disableCache).
//
//*****************************************************************************
extern void
DCSM_unlockZone1CSM(const DCSM_CSMPasswordKey * const psCMDKey);

//*****************************************************************************
//
//! Unlocks Zone 2 CSM.
//!
//! \param psCMDKey is a pointer to the CSMPSWDKEY that has the CSM
//!  password for zone 2.
//!
//! This function unlocks the CSM password. It first reads
//! the four password locations in the User OTP. If any of the password values
//! is different from 0xFFFFFFFF, it unlocks the device by writing the
//! provided passwords into CSM Key registers
//!
//! \return None.
//!
//! \note This function should not be called in an actual application,
//! should only be used for once to program the OTP memory. Ensure flash data
//! cache is disabled before calling this function(Flash_disableCache).
//
//*****************************************************************************
extern void
DCSM_unlockZone2CSM(const DCSM_CSMPasswordKey * const psCMDKey);
//*****************************************************************************
//
//! Write Zone 1 CSM.
//!
//! \param psCMDKey is a pointer to the CSMPSWDKEY that has the CSM
//!  password for zone 1.
//!
//! Password match flow is essentially a sequence of dummy reads
//! from password locations (PWL) followed by writes to CSMKEY registers.
//! This function helps writing the provided passwords into the CSM Key
//! registers. The DCSM_readZone1CSMPwd() should be called
//! by CPU1 before calling this API.
//!
//! \return None.
//
//*****************************************************************************
extern void
DCSM_writeZone1CSM(const DCSM_CSMPasswordKey * const psCMDKey);

//*****************************************************************************
//
//! Write Zone 2 CSM.
//!
//! \param psCMDKey is a pointer to the CSMPSWDKEY that has the CSM
//!  password for zone 2.
//!
//! Password match flow is essentially a sequence of dummy reads
//! from password locations (PWL) followed by writes to CSMKEY registers.
//! This function helps writing the provided passwords into the CSM Key
//! registers. The DCSM_readZone2CSMPwd() should be called
//! by CPU1 before calling this API.
//!
//! \return None.
//
//*****************************************************************************
extern void
DCSM_writeZone2CSM(const DCSM_CSMPasswordKey * const psCMDKey);
//*****************************************************************************
//
//! Returns the EXE-ONLY status of zone 1 for a flash sector
//!
//! \param sector is the flash sector value.  Use DCSM_Sector type.
//! \param cpuInst is the CPU whose Flash Sector needs to be accessed .
//!
//! The \e cpuInst parameter can have one of the following values of the
//! DCSM_CPUSel type:
//! - \b DCSM_CPUSEL_CPU1 -  To access CPU1 Flash sectors
//! - \b DCSM_CPUSEL_CPU2 - To access CPU2 Flash sectors
//! - \b DCSM_CPUSEL_CM - To access CM Flash sectors
//!
//! This function takes in a valid sector value and returns the status of EXE
//! ONLY security protection for the sector.
//!
//! \return Returns DCSM_PROTECTED if the sector is EXE-ONLY protected,
//! DCSM_UNPROTECTED if the sector is not EXE-ONLY protected,
//! DCSM_INCORRECT_ZONE if sector does not belong to this zone.
//
//*****************************************************************************
extern DCSM_EXEOnlyStatus
DCSM_getZone1FlashEXEStatus(DCSM_Sector sector, DCSM_CPUSel cpuInst);

//*****************************************************************************
//
//! Returns the EXE-ONLY status of zone 1 for a RAM module
//!
//! \param module is the RAM module value. Valid values are type DCSM_RAMModule
//! C28x RAMs :
//! - \b DCSM_RAMLS0
//! - \b DCSM_RAMLS1
//! - \b DCSM_RAMLS2
//! - \b DCSM_RAMLS3
//! - \b DCSM_RAMLS4
//! - \b DCSM_RAMLS5
//! - \b DCSM_RAMLS6
//! - \b DCSM_RAMLS7
//! - \b DCSM_RAMD0
//! - \b DCSM_RAMD1
//! CM RAMs :
//! - \b DCSM_RAM_CM_C0
//! - \b DCSM_RAM_CM_C1
//! - \b DCSM_RAM1_CPU1_CM
//! - \b DCSM_RAM2_CPU1_CM
//! - \b DCSM_RAM1_CM_CPU1
//! - \b DCSM_RAM2_CM_CPU1
//! - \b DCSM_RAM1_CPU2_CM
//! - \b DCSM_RAM2_CPU2_CM
//! - \b DCSM_RAM1_CM_CPU2
//! - \b DCSM_RAM2_CM_CPU2
//! - \b DCSM_RAM1_CPU1_CPU2
//! - \b DCSM_RAM2_CPU1_CPU2
//! - \b DCSM_RAM1_CPU2_CPU1
//! - \b DCSM_RAM2_CPU2_CPU1
//!
//! This function takes in a valid module value and returns the status of EXE
//! ONLY security protection for that module.
//!
//! \param cpuInst is the CPU whose RAM Section needs to be accessed .
//!
//! The \e cpuInst parameter can have one the following values:
//! - \b DCSM_CPUSEL_CPU1 -  To access CPU1 RAM
//! - \b DCSM_CPUSEL_CPU2 - To access CPU2 RAM
//! - \b DCSM_CPUSEL_CM - To access CM RAM
//!
//! \note  DCSM_CPUSEL_CPU1 & DCSM_CPUSEL_CPU2 can only be used with
//!  C28x RAMs (in DCSM_RAMModule) & DCSM_CPUSEL_CM can only be used with
//!  CM RAMs(in DCSM_RAMModule)
//!
//! \return Returns DCSM_PROTECTED if the module is EXE-ONLY protected,
//! DCSM_UNPROTECTED if the module is not EXE-ONLY protected,
//! DCSM_INCORRECT_ZONE if module does not belong to this zone.
//
//*****************************************************************************
extern DCSM_EXEOnlyStatus
DCSM_getZone1RAMEXEStatus(DCSM_RAMModule module, DCSM_CPUSel cpuInst);

//*****************************************************************************
//
//! Returns the EXE-ONLY status of zone 2 for a flash sector
//!
//! \param sector is the flash sector value. Use DCSM_Sector type.
//! \param cpuInst is the CPU whose Flash Sector needs to be accessed .
//!
//! The \e cpuInst parameter can have one of the following values of the
//! DCSM_CPUSel type:
//! - \b DCSM_CPUSEL_CPU1 -  To access CPU1 Flash sectors
//! - \b DCSM_CPUSEL_CPU2 - To access CPU2 Flash sectors
//! - \b DCSM_CPUSEL_CM - To access CM Flash sectors
//!
//! This function takes in a valid sector value and returns the status of EXE
//! ONLY security protection for the sector.
//!
//! \return Returns DCSM_PROTECTED if the sector is EXE-ONLY protected,
//! DCSM_UNPROTECTED if the sector is not EXE-ONLY protected,
//! DCSM_INCORRECT_ZONE if sector does not belong to this zone.
//
//*****************************************************************************
extern DCSM_EXEOnlyStatus
DCSM_getZone2FlashEXEStatus(DCSM_Sector sector, DCSM_CPUSel cpuInst);

//*****************************************************************************
//
//! Returns the EXE-ONLY status of zone 2 for a RAM module
//!
//! \param module is the RAM module value. Valid values are type DCSM_RAMModule
//! C28x RAMs :
//! - \b DCSM_RAMLS0
//! - \b DCSM_RAMLS1
//! - \b DCSM_RAMLS2
//! - \b DCSM_RAMLS3
//! - \b DCSM_RAMLS4
//! - \b DCSM_RAMLS5
//! - \b DCSM_RAMLS6
//! - \b DCSM_RAMLS7
//! - \b DCSM_RAMD0
//! - \b DCSM_RAMD1
//! CM RAMs :
//! - \b DCSM_RAM_CM_C0
//! - \b DCSM_RAM_CM_C1
//! - \b DCSM_RAM1_CPU1_CM
//! - \b DCSM_RAM2_CPU1_CM
//! - \b DCSM_RAM1_CM_CPU1
//! - \b DCSM_RAM2_CM_CPU1
//! - \b DCSM_RAM1_CPU2_CM
//! - \b DCSM_RAM2_CPU2_CM
//! - \b DCSM_RAM1_CM_CPU2
//! - \b DCSM_RAM2_CM_CPU2
//! - \b DCSM_RAM1_CPU1_CPU2
//! - \b DCSM_RAM2_CPU1_CPU2
//! - \b DCSM_RAM1_CPU2_CPU1
//! - \b DCSM_RAM2_CPU2_CPU1
//!
//! This function takes in a valid module value and returns the status of EXE
//! ONLY security protection for that module.
//!
//! \param cpuInst is the CPU whose RAM Section needs to be accessed .
//!
//! The \e cpuInst parameter can have one the following values:
//! - \b DCSM_CPUSEL_CPU1 -  To access CPU1 RAM
//! - \b DCSM_CPUSEL_CPU2 - To access CPU2 RAM
//! - \b DCSM_CPUSEL_CM - To access CM RAM
//!
//! \note  DCSM_CPUSEL_CPU1 & DCSM_CPUSEL_CPU2 can only be used with
//!  C28x RAMs (in DCSM_RAMModule) & DCSM_CPUSEL_CM can only be used with
//!  CM RAMs(in DCSM_RAMModule).
//!
//! \return Returns DCSM_PROTECTED if the module is EXE-ONLY protected,
//! DCSM_UNPROTECTED if the module is not EXE-ONLY protected,
//! DCSM_INCORRECT_ZONE if module does not belong to this zone.
//
//*****************************************************************************
extern DCSM_EXEOnlyStatus
DCSM_getZone2RAMEXEStatus(DCSM_RAMModule module, DCSM_CPUSel cpuInst);

//*****************************************************************************
//
//! Claims the zone semaphore which allows access to the Flash Wrapper register
//! for that zone.
//!
//! \param zone is the zone which is trying to claim the semaphore which allows
//! access to the Flash Wrapper registers.
//!
//! \return Returns true for a successful semaphore capture, false if it was
//! unable to capture the semaphore.
//
//*****************************************************************************
extern bool
DCSM_claimZoneSemaphore(DCSM_SemaphoreZone zone);

//*****************************************************************************
//
//! Releases the zone semaphore.
//!
//! \return Returns true if it was successful in releasing the zone semaphore
//! and false if it was unsuccessful in releasing the zone semaphore.
//!
//! \note  If the calling function is not in the right zone to be able
//!        to access this register, it will return a false.
//
//*****************************************************************************
extern bool
DCSM_releaseZoneSemaphore(void);

//*****************************************************************************
//
//! Perform dummy reads on the 128-bit Zone 1 CSM password.
//!
//! This function reads the four password locations in the User OTP
//! needed to be done as part of the Password Match Flow before
//! writes to the CSMKEY registers.
//! This would need to be done before a DCSM_writeZone1CSM().
//!
//! \return None.
//!
//! \note This API to be called from CPU1.
//
//*****************************************************************************
extern void
DCSM_readZone1CSMPwd(void);

//*****************************************************************************
//
//! Perform dummy reads on the 128-bit Zone 2 CSM password.
//!
//! This function reads the four password locations in the User OTP
//! needed to be done as part of the Password Match Flow before
//! writes to the CSMKEY registers.
//! This would need to be done before a DCSM_writeZone2CSM().
//!
//! \return None.
//!
//! \note This API to be called from CPU1.
//
//*****************************************************************************
extern void
DCSM_readZone2CSMPwd(void);

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

#endif //  DCSM_H
