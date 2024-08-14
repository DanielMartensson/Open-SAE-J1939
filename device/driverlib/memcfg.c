//###########################################################################
//
// FILE:   memcfg.c
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

#include "memcfg.h"


//*****************************************************************************
//
// MemCfg_lockConfig
//
//*****************************************************************************
void
MemCfg_lockConfig(uint32_t memSections)
{
    //
    // Check the arguments.
    //
    ASSERT(((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_D)   ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS)  ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS)  ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_MSG) ||
           (memSections == MEMCFG_SECT_ALL));

    //
    // Set the bit that blocks writes to the sections' configuration registers.
    //
    EALLOW;

    switch(memSections & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_D:
            HWREG(MEMCFG_BASE + MEMCFG_O_DXLOCK)  |= MEMCFG_SECT_NUM_MASK &
                                                     memSections;
            break;

        case MEMCFG_SECT_TYPE_LS:
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXLOCK) |= MEMCFG_SECT_NUM_MASK &
                                                     memSections;
            break;

        case MEMCFG_SECT_TYPE_GS:
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXLOCK) |= MEMCFG_SECT_NUM_MASK &
                                                     memSections;
            break;

        case MEMCFG_SECT_TYPE_MSG:
            HWREG(MEMCFG_BASE + MEMCFG_O_MSGXLOCK) |= MEMCFG_SECT_NUM_MASK &
                                                      memSections;
            break;

        case MEMCFG_SECT_TYPE_MASK:
            //
            // Lock configuration for all sections.
            //
            HWREG(MEMCFG_BASE + MEMCFG_O_DXLOCK)   |= MEMCFG_SECT_NUM_MASK &
                                                      MEMCFG_SECT_DX_ALL;
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXLOCK)  |= MEMCFG_SECT_NUM_MASK &
                                                      MEMCFG_SECT_LSX_ALL;
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXLOCK)  |= MEMCFG_SECT_NUM_MASK &
                                                      MEMCFG_SECT_GSX_ALL;
            HWREG(MEMCFG_BASE + MEMCFG_O_MSGXLOCK) |= MEMCFG_SECT_NUM_MASK &
                                                      MEMCFG_SECT_MSGX_ALL;
            break;

        default:
            //
            // Do nothing. Invalid memSections. Make sure you aren't OR-ing
            // values for two different types of memory sections.
            //
            break;
    }

    EDIS;
}

//*****************************************************************************
//
// MemCfg_unlockConfig
//
//*****************************************************************************
void
MemCfg_unlockConfig(uint32_t memSections)
{
    //
    // Check the arguments.
    //
    ASSERT(((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_D)   ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS)  ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS)  ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_MSG) ||
           (memSections == MEMCFG_SECT_ALL));

    //
    // Clear the bit that blocks writes to the sections' configuration
    // registers.
    //
    EALLOW;

    switch(memSections & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_D:
            HWREG(MEMCFG_BASE + MEMCFG_O_DXLOCK)  &= ~(MEMCFG_SECT_NUM_MASK &
                                                       memSections);
            break;

        case MEMCFG_SECT_TYPE_LS:
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXLOCK) &= ~(MEMCFG_SECT_NUM_MASK &
                                                       memSections);
            break;

        case MEMCFG_SECT_TYPE_GS:
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXLOCK) &= ~(MEMCFG_SECT_NUM_MASK &
                                                       memSections);
            break;

        case MEMCFG_SECT_TYPE_MSG:
            HWREG(MEMCFG_BASE + MEMCFG_O_MSGXLOCK) &= ~(MEMCFG_SECT_NUM_MASK &
                                                        memSections);
            break;

        case MEMCFG_SECT_TYPE_MASK:
            //
            // Unlock configuration for all sections.
            //
            HWREG(MEMCFG_BASE + MEMCFG_O_DXLOCK) &=
                ~((uint32_t)(MEMCFG_SECT_NUM_MASK & MEMCFG_SECT_DX_ALL));
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXLOCK) &=
                ~((uint32_t)(MEMCFG_SECT_NUM_MASK & MEMCFG_SECT_LSX_ALL));
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXLOCK) &=
                ~((uint32_t)(MEMCFG_SECT_NUM_MASK & MEMCFG_SECT_GSX_ALL));
            HWREG(MEMCFG_BASE + MEMCFG_O_MSGXLOCK) &=
                ~((uint32_t)(MEMCFG_SECT_NUM_MASK & MEMCFG_SECT_MSGX_ALL));
            break;

        default:
            //
            // Do nothing. Invalid memSections. Make sure you aren't OR-ing
            // values for two different types of memory sections.
            //
            break;
    }

    EDIS;
}

//*****************************************************************************
//
// MemCfg_commitConfig
//
//*****************************************************************************
void
MemCfg_commitConfig(uint32_t memSections)
{
    //
    // Check the arguments.
    //
    ASSERT(((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_D)   ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS)  ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS)  ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_MSG) ||
           (memSections == MEMCFG_SECT_ALL));

    //
    // Set the bit that permanently blocks writes to the sections'
    // configuration registers.
    //
    EALLOW;

    switch(memSections & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_D:
            HWREG(MEMCFG_BASE + MEMCFG_O_DXCOMMIT)  |= MEMCFG_SECT_NUM_MASK &
                                                       memSections;
            break;

        case MEMCFG_SECT_TYPE_LS:
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXCOMMIT) |= MEMCFG_SECT_NUM_MASK &
                                                       memSections;
            break;

        case MEMCFG_SECT_TYPE_GS:
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXCOMMIT) |= MEMCFG_SECT_NUM_MASK &
                                                       memSections;
            break;

        case MEMCFG_SECT_TYPE_MSG:
            HWREG(MEMCFG_BASE + MEMCFG_O_MSGXCOMMIT) |= MEMCFG_SECT_NUM_MASK &
                                                        memSections;
            break;

        case MEMCFG_SECT_TYPE_MASK:
            //
            // Commit configuration for all sections.
            //
            HWREG(MEMCFG_BASE + MEMCFG_O_DXCOMMIT)   |= MEMCFG_SECT_NUM_MASK &
                                                        MEMCFG_SECT_DX_ALL;
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXCOMMIT)  |= MEMCFG_SECT_NUM_MASK &
                                                        MEMCFG_SECT_LSX_ALL;
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXCOMMIT)  |= MEMCFG_SECT_NUM_MASK &
                                                        MEMCFG_SECT_GSX_ALL;
            HWREG(MEMCFG_BASE + MEMCFG_O_MSGXCOMMIT) |= MEMCFG_SECT_NUM_MASK &
                                                        MEMCFG_SECT_MSGX_ALL;
            break;

        default:
            //
            // Do nothing. Invalid memSections. Make sure you aren't OR-ing
            // values for two different types of RAM.
            //
            break;
    }

    EDIS;
}

//*****************************************************************************
//
// MemCfg_setProtection
//
//*****************************************************************************
void
MemCfg_setProtection(uint32_t memSection, uint32_t protectMode)
{
    uint32_t shiftVal = 0U;
    uint32_t maskVal;
    uint32_t regVal;
    uint32_t sectionNum;
    uint32_t regOffset;

    //
    // Check the arguments.
    //
    ASSERT(((memSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS)   ||
           ((memSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_D)    ||
           ((memSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_MSG)  ||
           ((memSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS));

    //
    // Calculate how far the protect mode value needs to be shifted. Each
    // section number is represented by a bit in the lower word of memSection
    // and 8 bits in the corresponding ACCPROT register.
    //
    sectionNum = memSection & MEMCFG_SECT_NUM_MASK;

    while(sectionNum != 1U)
    {
        sectionNum = sectionNum >> 1U;
        shiftVal += 8U;
    }

    //
    // Calculate register offset. Also, make sure the shift value is no greater
    // than 31.
    //
    regOffset = (shiftVal & ~(0x1FU)) >> 4U;
    shiftVal &= 0x0001FU;
    maskVal = (uint32_t)MEMCFG_XACCPROTX_M << shiftVal;
    regVal = protectMode << shiftVal;

    //
    // Write the access protection mode into the appropriate field
    //
    EALLOW;

    switch(memSection & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_D:
            HWREG(MEMCFG_BASE + MEMCFG_O_DXACCPROT0 + regOffset) &= ~maskVal;
            HWREG(MEMCFG_BASE + MEMCFG_O_DXACCPROT0 + regOffset) |= regVal;
            break;

        case MEMCFG_SECT_TYPE_LS:
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXACCPROT0 + regOffset) &= ~maskVal;
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXACCPROT0 + regOffset) |= regVal;
            break;

        case MEMCFG_SECT_TYPE_GS:
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXACCPROT0 + regOffset) &= ~maskVal;
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXACCPROT0 + regOffset) |= regVal;
            break;

        case MEMCFG_SECT_TYPE_MSG:
            HWREG(MEMCFG_BASE + MEMCFG_O_MSGXACCPROT0 + regOffset) &= ~maskVal;
            HWREG(MEMCFG_BASE + MEMCFG_O_MSGXACCPROT0 + regOffset) |= regVal;
            break;

        default:
            //
            // Do nothing. Invalid memSection.
            //
            break;
    }

    EDIS;
}

//*****************************************************************************
//
// MemCfg_setLSRAMControllerSel
//
//*****************************************************************************
void
MemCfg_setLSRAMControllerSel(uint32_t ramSection,
                             MemCfg_LSRAMControllerSel controllerSel)
{
    uint32_t shiftVal;
    uint32_t temp;

    //
    // Check the arguments.
    //
    ASSERT((ramSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS);

    //
    // Calculate how far the controller select value needs to be shifted. Each
    // section number is represented by a bit in the lower word of ramSection
    // and 2 bits in the corresponding MSEL register.
    //
    shiftVal = 0U;
    temp = MEMCFG_SECT_NUM_MASK & ramSection;

    while(temp != 1U)
    {
        temp = temp >> 1U;
        shiftVal += 2U;
    }

    //
    // Write the controller select setting into the appropriate field
    //
    EALLOW;

    HWREG(MEMCFG_BASE + MEMCFG_O_LSXMSEL) =
        (HWREG(MEMCFG_BASE + MEMCFG_O_LSXMSEL) &
         ~(MEMCFG_LSXMSEL_MSEL_LS0_M << shiftVal)) |
        ((uint32_t)controllerSel << shiftVal);

    EDIS;
}

//*****************************************************************************
//
// MemCfg_setGSRAMControllerSel
//
//*****************************************************************************
void
MemCfg_setGSRAMControllerSel(uint32_t ramSections,
                             MemCfg_GSRAMControllerSel controllerSel)
{
     uint32_t sectionNum;

    //
    // Check the arguments.
    //
    ASSERT((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS);

    //
    // We only need the section number bits for this function.
    //
    sectionNum = ramSections & MEMCFG_SECT_NUM_MASK;

    //
    // Write the controller select setting into the appropriate field.
    //
    EALLOW;
    if(controllerSel == MEMCFG_GSRAMCONTROLLER_CPU1)
    {
        HWREG(MEMCFG_BASE + MEMCFG_O_GSXMSEL) &= ~sectionNum;
    }
    else
    {
        HWREG(MEMCFG_BASE + MEMCFG_O_GSXMSEL) |= sectionNum;
    }
    EDIS;
}

//*****************************************************************************
//
// MemCfg_lockTestConfig
//
//*****************************************************************************
void
MemCfg_lockTestConfig(uint32_t memSections)
{
    //
    // Check the arguments.
    //
    ASSERT(((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_D)   ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS)  ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS)  ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_MSG) ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_ROM) ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) ==
                                                 MEMCFG_SECT_TYPE_PERIMEM) ||
           (memSections == MEMCFG_SECT_ALL));

    //
    // Set the bit that blocks writes to the sections' configuration registers.
    //
    EALLOW;

    switch(memSections & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_D:
             HWREG(MEMCFG_BASE + MEMCFG_O_DXRAMTEST_LOCK)   |=
                  (MEMCFG_TESTLOCK_KEY | (MEMCFG_SECT_NUM_MASK & memSections));
            break;

        case MEMCFG_SECT_TYPE_LS:
             HWREG(MEMCFG_BASE + MEMCFG_O_LSXRAMTEST_LOCK)   |=
                  (MEMCFG_TESTLOCK_KEY | (MEMCFG_SECT_NUM_MASK & memSections));
            break;

        case MEMCFG_SECT_TYPE_GS:
             HWREG(MEMCFG_BASE + MEMCFG_O_GSXRAMTEST_LOCK)   |=
                  (MEMCFG_TESTLOCK_KEY | (MEMCFG_SECT_NUM_MASK & memSections));
            break;

        case MEMCFG_SECT_TYPE_MSG:
             HWREG(MEMCFG_BASE + MEMCFG_O_MSGXRAMTEST_LOCK)  |=
                  (MEMCFG_TESTLOCK_KEY | (MEMCFG_SECT_NUM_MASK & memSections));
            break;

        case MEMCFG_SECT_TYPE_ROM:
            HWREG(MEMCFG_BASE + MEMCFG_O_ROM_LOCK)           |=
                  (MEMCFG_TESTLOCK_KEY | (MEMCFG_SECT_NUM_MASK & memSections));
            break;

        case MEMCFG_SECT_TYPE_PERIMEM:
            HWREG(MEMCFG_BASE + MEMCFG_O_PERI_MEM_TEST_LOCK) |=
                  (MEMCFG_TESTLOCK_KEY | (MEMCFG_SECT_NUM_MASK & memSections));
            break;

        case MEMCFG_SECT_TYPE_MASK:
            //
            // Lock configuration for all sections.
            //
            HWREG(MEMCFG_BASE + MEMCFG_O_DXRAMTEST_LOCK)     |=
                  (MEMCFG_TESTLOCK_KEY | (MEMCFG_SECT_NUM_MASK & memSections));
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXRAMTEST_LOCK)    |=
                  (MEMCFG_TESTLOCK_KEY | (MEMCFG_SECT_NUM_MASK & memSections));
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXRAMTEST_LOCK)    |=
                  (MEMCFG_TESTLOCK_KEY | (MEMCFG_SECT_NUM_MASK & memSections));
            HWREG(MEMCFG_BASE + MEMCFG_O_MSGXRAMTEST_LOCK)   |=
                  (MEMCFG_TESTLOCK_KEY | (MEMCFG_SECT_NUM_MASK & memSections));
            HWREG(MEMCFG_BASE + MEMCFG_O_ROM_LOCK)           |=
                  (MEMCFG_TESTLOCK_KEY | (MEMCFG_SECT_NUM_MASK & memSections));
            HWREG(MEMCFG_BASE + MEMCFG_O_PERI_MEM_TEST_LOCK) |=
                  (MEMCFG_TESTLOCK_KEY | (MEMCFG_SECT_NUM_MASK & memSections));
            break;

        default:
            //
            // Do nothing. Invalid ramSections. Make sure you aren't OR-ing
            // values for two different types of memory sections.
            //
            break;
    }

    EDIS;
}
//*****************************************************************************
//
// MemCfg_unlockTestConfig
//
//*****************************************************************************
void
MemCfg_unlockTestConfig(uint32_t memSections)
{
    //
    // Check the arguments.
    //
    ASSERT(((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_D)   ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS)  ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS)  ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_MSG) ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_ROM) ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) ==
                                                 MEMCFG_SECT_TYPE_PERIMEM) ||
           (memSections == MEMCFG_SECT_ALL));


    //
    // Clear the bit that blocks writes to the sections' configuration
    // registers.
    //
    EALLOW;

    switch(memSections & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_D:
            HWREG(MEMCFG_BASE + MEMCFG_O_DXRAMTEST_LOCK) =
                               MEMCFG_TESTLOCK_KEY |
                               (HWREG(MEMCFG_BASE + MEMCFG_O_DXRAMTEST_LOCK) &
                                ~(MEMCFG_SECT_NUM_MASK & memSections));
            break;

        case MEMCFG_SECT_TYPE_LS:
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXRAMTEST_LOCK) =
                               MEMCFG_TESTLOCK_KEY |
                               (HWREG(MEMCFG_BASE + MEMCFG_O_LSXRAMTEST_LOCK) &
                                ~(MEMCFG_SECT_NUM_MASK & memSections));
            break;

        case MEMCFG_SECT_TYPE_GS:
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXRAMTEST_LOCK) =
                               MEMCFG_TESTLOCK_KEY |
                               (HWREG(MEMCFG_BASE + MEMCFG_O_GSXRAMTEST_LOCK) &
                                ~(MEMCFG_SECT_NUM_MASK & memSections));
            break;

        case MEMCFG_SECT_TYPE_MSG:
            HWREG(MEMCFG_BASE + MEMCFG_O_MSGXRAMTEST_LOCK) =
                              MEMCFG_TESTLOCK_KEY |
                              (HWREG(MEMCFG_BASE + MEMCFG_O_MSGXRAMTEST_LOCK) &
                               ~(MEMCFG_SECT_NUM_MASK & memSections));
            break;

        case MEMCFG_SECT_TYPE_ROM:
            HWREG(MEMCFG_BASE + MEMCFG_O_ROM_LOCK) = (MEMCFG_TESTLOCK_KEY |
                                  (HWREG(MEMCFG_BASE + MEMCFG_O_ROM_LOCK) &
                                   ~(MEMCFG_SECT_NUM_MASK & memSections)));
            break;

        case MEMCFG_SECT_TYPE_PERIMEM:
            HWREG(MEMCFG_BASE + MEMCFG_O_PERI_MEM_TEST_LOCK) =
                        (MEMCFG_TESTLOCK_KEY |
                         (HWREG(MEMCFG_BASE + MEMCFG_O_PERI_MEM_TEST_LOCK) &
                         ~(MEMCFG_SECT_NUM_MASK & memSections)));
            break;

        case MEMCFG_SECT_TYPE_MASK:
            //
            // Unlock configuration for all sections.
            //
            HWREG(MEMCFG_BASE + MEMCFG_O_DXRAMTEST_LOCK)   =
                 HWREG(MEMCFG_BASE + MEMCFG_O_DXRAMTEST_LOCK)   |
                 (MEMCFG_TESTLOCK_KEY | ~(MEMCFG_SECT_NUM_MASK & memSections));
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXRAMTEST_LOCK)  =
                 HWREG(MEMCFG_BASE + MEMCFG_O_LSXRAMTEST_LOCK)  |
                 (MEMCFG_TESTLOCK_KEY | ~(MEMCFG_SECT_NUM_MASK & memSections));
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXRAMTEST_LOCK)  =
                 HWREG(MEMCFG_BASE + MEMCFG_O_GSXRAMTEST_LOCK)  |
                 (MEMCFG_TESTLOCK_KEY | ~(MEMCFG_SECT_NUM_MASK & memSections));
            HWREG(MEMCFG_BASE + MEMCFG_O_MSGXRAMTEST_LOCK) =
                 HWREG(MEMCFG_BASE + MEMCFG_O_MSGXRAMTEST_LOCK) |
                 (MEMCFG_TESTLOCK_KEY | ~(MEMCFG_SECT_NUM_MASK & memSections));
            HWREG(MEMCFG_BASE + MEMCFG_O_ROM_LOCK)   =
                 HWREG(MEMCFG_BASE + MEMCFG_O_ROM_LOCK)   |
                 (MEMCFG_TESTLOCK_KEY | ~(MEMCFG_SECT_NUM_MASK & memSections));
            HWREG(MEMCFG_BASE + MEMCFG_O_PERI_MEM_TEST_LOCK) =
                 HWREG(MEMCFG_BASE + MEMCFG_O_PERI_MEM_TEST_LOCK) |
                 (MEMCFG_TESTLOCK_KEY | ~(MEMCFG_SECT_NUM_MASK & memSections));
            break;

        default:
            //
            // Do nothing. Invalid memSections. Make sure you aren't OR-ing
            // values for two different types of RAM.
            //
            break;
    }

    EDIS;

}
//*****************************************************************************
//
// MemCfg_setTestMode
//
//*****************************************************************************
void
MemCfg_setTestMode(uint32_t memSection, MemCfg_TestMode testMode)
{
    uint32_t shiftVal = 0U;
    uint32_t maskVal;
    uint32_t regVal;
    uint32_t sectionNum;

    //
    // Check the arguments.
    //
    ASSERT(((memSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_D)    ||
           ((memSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS)   ||
           ((memSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS)   ||
           ((memSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_ROM)  ||
           ((memSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_MSG));

    //
    // Calculate how far the protect mode value needs to be shifted. Each
    // section number is represented by a bit in the lower word of memSection
    // and 2 bits in the corresponding TEST register.
    //
    sectionNum = memSection & MEMCFG_SECT_NUM_MASK;

    while(sectionNum != 1U)
    {
        sectionNum = sectionNum >> 1U;
        shiftVal += 2U;
    }

    maskVal = (uint32_t)MEMCFG_XTEST_M << shiftVal;
    regVal = (uint32_t)testMode << shiftVal;

    //
    // Write the test mode into the appropriate field
    //
    EALLOW;

    switch(memSection & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_D:
            HWREG(MEMCFG_BASE + MEMCFG_O_DXTEST) &= ~maskVal;
            HWREG(MEMCFG_BASE + MEMCFG_O_DXTEST) |= regVal;
            break;

        case MEMCFG_SECT_TYPE_LS:
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXTEST) &= ~maskVal;
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXTEST) |= regVal;
            break;

        case MEMCFG_SECT_TYPE_GS:
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXTEST) &= ~maskVal;
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXTEST) |= regVal;
            break;

        case MEMCFG_SECT_TYPE_MSG:
            HWREG(MEMCFG_BASE + MEMCFG_O_MSGXTEST) &= ~maskVal;
            HWREG(MEMCFG_BASE + MEMCFG_O_MSGXTEST) |= regVal;
            break;

        case MEMCFG_SECT_TYPE_ROM:
            HWREG(MEMCFG_BASE + MEMCFG_O_ROM_TEST) &= ~maskVal;
            HWREG(MEMCFG_BASE + MEMCFG_O_ROM_TEST) |= regVal;
            break;

        default:
            //
            // Do nothing. Invalid memSection.
            //
            break;
    }

    EDIS;
}

//*****************************************************************************
//
// MemCfg_initSections
//
//*****************************************************************************
void
MemCfg_initSections(uint32_t ramSections)
{
    //
    // Check the arguments.
    //
    ASSERT(((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_D)   ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS)  ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS)  ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_MSG) ||
           (ramSections == MEMCFG_SECT_ALL));

    //
    // Set the bit in the various initialization registers that starts
    // initialization.
    //
    EALLOW;

    switch(ramSections & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_D:
            HWREG(MEMCFG_BASE + MEMCFG_O_DXINIT)   |= MEMCFG_SECT_NUM_MASK &
                                                      ramSections;
            break;

        case MEMCFG_SECT_TYPE_LS:
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXINIT)  |= MEMCFG_SECT_NUM_MASK &
                                                      ramSections;
            break;

        case MEMCFG_SECT_TYPE_GS:
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXINIT)  |= MEMCFG_SECT_NUM_MASK &
                                                      ramSections;
            break;

        case MEMCFG_SECT_TYPE_MSG:
            HWREG(MEMCFG_BASE + MEMCFG_O_MSGXINIT) |= MEMCFG_SECT_NUM_MASK &
                                                      ramSections;
            break;

        case MEMCFG_SECT_TYPE_MASK:
            //
            // Initialize all sections.
            //
            HWREG(MEMCFG_BASE + MEMCFG_O_DXINIT)   |= MEMCFG_SECT_NUM_MASK &
                                                      MEMCFG_SECT_DX_ALL;
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXINIT)  |= MEMCFG_SECT_NUM_MASK &
                                                      MEMCFG_SECT_LSX_ALL;
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXINIT)  |= MEMCFG_SECT_NUM_MASK &
                                                      MEMCFG_SECT_GSX_ALL;
            HWREG(MEMCFG_BASE + MEMCFG_O_MSGXINIT) |= MEMCFG_SECT_NUM_MASK &
                                                      MEMCFG_SECT_MSGX_ALL;
            break;

        default:
            //
            // Do nothing. Invalid ramSections. Make sure you aren't OR-ing
            // values for two different types of RAM.
            //
            break;
    }

    EDIS;
}

//*****************************************************************************
//
// MemCfg_getInitStatus
//
//*****************************************************************************
bool
MemCfg_getInitStatus(uint32_t ramSections)
{
    uint32_t status;

    //
    // Check the arguments.
    //
    ASSERT(((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_D)   ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS)  ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS)  ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_MSG) ||
           (ramSections == MEMCFG_SECT_ALL));

    //
    // Read registers containing the initialization complete status.
    //
    switch(ramSections & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_D:
            status = HWREG(MEMCFG_BASE + MEMCFG_O_DXINITDONE);
            break;

        case MEMCFG_SECT_TYPE_LS:
            status = HWREG(MEMCFG_BASE + MEMCFG_O_LSXINITDONE);
            break;

        case MEMCFG_SECT_TYPE_GS:
            status = HWREG(MEMCFG_BASE + MEMCFG_O_GSXINITDONE);
            break;

        case MEMCFG_SECT_TYPE_MSG:
            status = HWREG(MEMCFG_BASE + MEMCFG_O_MSGXINITDONE);
            break;

        case MEMCFG_SECT_TYPE_MASK:
            //
            // Return the overall status.
            //
            if((HWREG(MEMCFG_BASE + MEMCFG_O_DXINITDONE) ==
                MEMCFG_SECT_DX_ALL) &&
               (HWREG(MEMCFG_BASE + MEMCFG_O_LSXINITDONE) ==
                MEMCFG_SECT_LSX_ALL) &&
               (HWREG(MEMCFG_BASE + MEMCFG_O_GSXINITDONE) ==
                MEMCFG_SECT_GSX_ALL) &&
               (HWREG(MEMCFG_BASE + MEMCFG_O_MSGXINITDONE) ==
                MEMCFG_SECT_MSGX_ALL))
            {
                status = MEMCFG_SECT_NUM_MASK;
            }
            else
            {
                status = 0U;
            }
            break;

        default:
            //
            // Invalid ramSections. Make sure you aren't OR-ing values for two
            // different types of RAM.
            //
            status = 0U;
            break;
    }

    return((ramSections & status) == (ramSections & MEMCFG_SECT_NUM_MASK));
}

//*****************************************************************************
//
// MemCfg_getViolationAddress
//
//*****************************************************************************
uint32_t
MemCfg_getViolationAddress(uint32_t intFlag)
{
    uint32_t address;
    uint32_t stsNumber;

    //
    // Calculate the the address of the desired violation address register.
    //
    if((intFlag & MEMCFG_MVIOL_MASK) != 0U)
    {
        stsNumber = intFlag >> MEMCFG_MVIOL_SHIFT;
        address = ACCESSPROTECTION_BASE + MEMCFG_O_MCPUFAVADDR;
    }
    else
    {
        stsNumber = intFlag;
        address = ACCESSPROTECTION_BASE + MEMCFG_O_NMCPURDAVADDR;
    }

    while(stsNumber > 1U)
    {
        stsNumber = stsNumber >> 1U;
        address += (uint32_t)(MEMCFG_O_NMCPUWRAVADDR - MEMCFG_O_NMCPURDAVADDR);
    }

    //
    // Read and return the access violation address at the calculated location.
    //
    return(HWREG(address));
}

//*****************************************************************************
//
// MemCfg_getCorrErrorAddress
//
//*****************************************************************************
uint32_t
MemCfg_getCorrErrorAddress(uint32_t stsFlag)
{
    uint32_t address, temp;

    //
    // Calculate the the address of the desired error address register.
    //
    address = MEMORYERROR_BASE + MEMCFG_O_CCPUREADDR;

    temp = stsFlag;

    while(temp > 1U)
    {
        temp = temp >> 1U;
        address += (uint32_t)(MEMCFG_O_CERRCLR - MEMCFG_O_CERRSET);
    }

    //
    // Read and return the error address at the calculated location.
    //
    return(HWREG(address));

}

//*****************************************************************************
//
// MemCfg_getUncorrErrorAddress
//
//*****************************************************************************
uint32_t
MemCfg_getUncorrErrorAddress(uint32_t stsFlag)
{
    uint32_t address;
    uint32_t temp;

    //
    // Calculate the the address of the desired error address register.
    //
    address = MEMORYERROR_BASE + MEMCFG_O_UCCPUREADDR;

    temp = stsFlag;

    while(temp > 1U)
    {
        temp = temp >> 1U;
        address += (uint32_t)(MEMCFG_O_UCDMAREADDR - MEMCFG_O_UCCPUREADDR);
    }

    //
    // Read and return the error address at the calculated location.
    //
    return(HWREG(address));
}

//*****************************************************************************
//
// MemCfg_forceMemError
//
//*****************************************************************************
void
MemCfg_forceMemError(uint32_t memSections)
{
    //
    // Check the arguments.
    //
    ASSERT(((memSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_ROM) ||
           ((memSections & MEMCFG_SECT_TYPE_MASK) ==
                                                 MEMCFG_SECT_TYPE_PERIMEM) ||
           (memSections == MEMCFG_SECT_ALL));

    //
    // Forces error in the selected memory.
    //
    switch(memSections & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_ROM:
            HWREG(MEMCFG_BASE + MEMCFG_O_ROM_FORCE_ERROR)       |=
                                          (MEMCFG_SECT_NUM_MASK & memSections);
            break;
        case MEMCFG_SECT_TYPE_PERIMEM:
            HWREG(MEMCFG_BASE + MEMCFG_O_PERI_MEM_TEST_CONTROL) |=
                                          (MEMCFG_SECT_NUM_MASK & memSections);
            break;

        case MEMCFG_SECT_TYPE_MASK:
            HWREG(MEMCFG_BASE + MEMCFG_O_ROM_FORCE_ERROR)       |=
                                          (MEMCFG_SECT_NUM_MASK & memSections);
            HWREG(MEMCFG_BASE + MEMCFG_O_PERI_MEM_TEST_CONTROL) |=
                                          (MEMCFG_SECT_NUM_MASK & memSections);
            break;

        default:
            //
            // Do nothing. Invalid memory.
            //
            break;
    }
}

//*****************************************************************************
//
// MemCfg_enablePeriMemTestMode
//
//*****************************************************************************
void
MemCfg_enablePeriMemTestMode(uint32_t memSections)
{
    //
    // Check the arguments.
    //
    ASSERT(((memSections & MEMCFG_SECT_TYPE_MASK) ==
             MEMCFG_SECT_TYPE_PERIMEM) || (memSections == MEMCFG_SECT_ALL));

    //
    // Enables test mode for selected memory type.
    //
    switch(memSections)
    {
        case MEMCFG_SECT_PERIMEM_ETHERCAT:
            HWREG(MEMCFG_BASE + MEMCFG_O_PERI_MEM_TEST_CONTROL) |=
                             MEMCFG_PERI_MEM_TEST_CONTROL_ETHERCAT_TEST_ENABLE;
            break;

        default:
            //
            // Do nothing. Invalid memory type.
            //
            break;
    }
}

//*****************************************************************************
//
// MemCfg_disablePeriMemTestMode
//
//*****************************************************************************
void
MemCfg_disablePeriMemTestMode(uint32_t memSections)
{
    //
    // Check the arguments.
    //
    ASSERT(((memSections & MEMCFG_SECT_TYPE_MASK) ==
             MEMCFG_SECT_TYPE_PERIMEM) || (memSections == MEMCFG_SECT_ALL));

    //
    // Enables test mode for selected memory type.
    //
    switch(memSections)
    {
        case MEMCFG_SECT_PERIMEM_ETHERCAT:
            HWREG(MEMCFG_BASE + MEMCFG_O_PERI_MEM_TEST_CONTROL) &=
                           ~MEMCFG_PERI_MEM_TEST_CONTROL_ETHERCAT_TEST_ENABLE;
            break;

        default:
            //
            // Do nothing. Invalid memory type.
            //
            break;
    }
}
