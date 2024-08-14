//###########################################################################
//
// FILE:   interrupt.c
//
// TITLE:  C28x Interrupt (PIE) driver.
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

#include "interrupt.h"

//*****************************************************************************
//
//! \internal
//! Clears the IFR flag in the CPU.
//!
//! \param group specifies the interrupt group to be cleared.
//!
//! This function clears the IFR flag.  This switch is needed because the
//! clearing of the IFR can only be done with a constant.
//
//*****************************************************************************
static void Interrupt_clearIFR(uint16_t group)
{
    switch(group)
    {
        case 0x0001U:
            IFR &= ~(uint16_t)0x0001U;
            break;
        case 0x0002U:
            IFR &= ~(uint16_t)0x0002U;
            break;
        case 0x0004U:
            IFR &= ~(uint16_t)0x0004U;
            break;
        case 0x0008U:
            IFR &= ~(uint16_t)0x0008U;
            break;
        case 0x0010U:
            IFR &= ~(uint16_t)0x0010U;
            break;
        case 0x0020U:
            IFR &= ~(uint16_t)0x0020U;
            break;
        case 0x0040U:
            IFR &= ~(uint16_t)0x0040U;
            break;
        case 0x0080U:
            IFR &= ~(uint16_t)0x0080U;
            break;
        case 0x0100U:
            IFR &= ~(uint16_t)0x0100U;
            break;
        case 0x0200U:
            IFR &= ~(uint16_t)0x0200U;
            break;
        case 0x0400U:
            IFR &= ~(uint16_t)0x0400U;
            break;
        case 0x0800U:
            IFR &= ~(uint16_t)0x0800U;
            break;
        case 0x1000U:
            IFR &= ~(uint16_t)0x1000U;
            break;
        case 0x2000U:
            IFR &= ~(uint16_t)0x2000U;
            break;
        case 0x4000U:
            IFR &= ~(uint16_t)0x4000U;
            break;
        case 0x8000U:
            IFR &= ~(uint16_t)0x8000U;
            break;
        default:
            //
            // Invalid group mask.
            //
            ASSERT((bool)false);
            break;
    }
}

//*****************************************************************************
//
// Interrupt_initModule
//
//*****************************************************************************
void
Interrupt_initModule(void)
{
    //
    // Disable and clear all interrupts at the CPU
    //
    (void)Interrupt_disableGlobal();
    IER = 0x0000U;
    IFR = 0x0000U;

    //
    // Clear all PIEIER registers
    //
    HWREGH(PIECTRL_BASE + PIE_O_IER1) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IER2) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IER3) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IER4) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IER5) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IER6) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IER7) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IER8) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IER9) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IER10) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IER11) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IER12) = 0U;

    //
    // Clear all PIEIFR registers
    //
    HWREGH(PIECTRL_BASE + PIE_O_IFR1) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IFR2) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IFR3) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IFR4) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IFR5) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IFR6) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IFR7) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IFR8) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IFR9) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IFR10) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IFR11) = 0U;
    HWREGH(PIECTRL_BASE + PIE_O_IFR12) = 0U;

    //
    // Enable vector fetching from PIE block
    //
    HWREGH(PIECTRL_BASE + PIE_O_CTRL) |= PIE_CTRL_ENPIE;

}

//*****************************************************************************
//
// Interrupt_initVectorTable
//
//*****************************************************************************
void
Interrupt_initVectorTable(void)
{
    uint16_t i;

    EALLOW;

    //
    // We skip the first three locations because they are initialized by Boot
    // ROM with boot variables.
    //
    for(i = 3U; i < 224U; i++)
    {
        HWREG(PIEVECTTABLE_BASE + (2U * i)) =
            (uint32_t)Interrupt_defaultHandler;
    }

    //
    // NMI and ITRAP get their own handlers.
    //
    HWREG(PIEVECTTABLE_BASE + ((INT_NMI >> 16U) * 2U)) =
        (uint32_t)Interrupt_nmiHandler;
    HWREG(PIEVECTTABLE_BASE + ((INT_ILLEGAL >> 16U) * 2U)) =
        (uint32_t)Interrupt_illegalOperationHandler;

    EDIS;
}

//*****************************************************************************
//
//Interrupt_enable
//
//*****************************************************************************
void
Interrupt_enable(uint32_t interruptNumber)
{
    bool intsDisabled;
    uint16_t intGroup;
    uint16_t groupMask;
    uint16_t vectID;

    vectID = (uint16_t)(interruptNumber >> 16U);

    //
    // Globally disable interrupts but save status
    //
    intsDisabled = Interrupt_disableGlobal();

    //
    // PIE Interrupts
    //
    if(vectID >= 0x20U)
    {
        intGroup = (uint16_t)(((interruptNumber & 0xFF00UL) >> 8U) - 1U);
        groupMask = (uint16_t)1U << intGroup;

        HWREGH((PIECTRL_BASE + PIE_O_IER1 + (intGroup * 2U))) |=
            (uint16_t)1U << ((interruptNumber & 0xFFU) - 1U);

        //
        // Enable PIE Group Interrupt
        //
        IER |= groupMask;
    }

    //
    // INT13, INT14, DLOGINT, & RTOSINT
    //
    else if((vectID >= 0x0DU) && (vectID <= 0x10U))
    {
        IER |= (uint16_t)1U << (vectID - 1U);
    }
    else
    {
        //
        // Other interrupts
        //
    }

    //
    // Re-enable interrupts if they were enabled
    //
    if(!intsDisabled)
    {
        (void)Interrupt_enableGlobal();
    }
}

//*****************************************************************************
//
// Interrupt_disable
//
//*****************************************************************************
void
Interrupt_disable(uint32_t interruptNumber)
{
    bool intsDisabled;
    uint16_t intGroup;
    uint16_t groupMask;
    uint16_t vectID;

    vectID = (uint16_t)(interruptNumber >> 16U);

    intsDisabled = Interrupt_disableGlobal();

    //
    // PIE Interrupts
    //
    if(vectID >= 0x20U)
    {
        intGroup = (uint16_t)(((interruptNumber & 0xFF00UL) >> 8U) - 1U);
        groupMask = (uint16_t)1U << intGroup;

        //
        // Disable individual PIE interrupt
        //
        HWREGH((PIECTRL_BASE + PIE_O_IER1 + (intGroup * 2U))) &=
            ~(1U << ((interruptNumber & 0xFFUL) - 1U));

        //
        // Wait for any pending interrupts to get to the CPU
        //
        NOP;
        NOP;
        NOP;
        NOP;
        NOP;

        Interrupt_clearIFR(groupMask);

        //
        // Acknowledge any interrupts
        //
        HWREGH(PIECTRL_BASE + PIE_O_ACK) = groupMask;
    }

    //
    // INT13, INT14, DLOGINT, & RTOSINT
    //
    else if((vectID >= 0x0DU) && (vectID <= 0x10U))
    {
        IER &= ~((uint16_t)1U << (vectID - 1U));

        //
        // Wait for any pending interrupts to get to the CPU
        //
        NOP;
        NOP;
        NOP;
        NOP;
        NOP;

        Interrupt_clearIFR((uint16_t)1U << (vectID - 1U));
    }
    else
    {
        //
        // Other interrupts
        //
    }

    //
    // Re-enable interrupts if they were enabled
    //
    if(!intsDisabled)
    {
        (void)Interrupt_enableGlobal();
    }
}
