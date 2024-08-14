//###########################################################################
//
// FILE:   interrupt.h
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

#ifndef INTERRUPT_H
#define INTERRUPT_H

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
//! \addtogroup interrupt_api Interrupt
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_pie.h"
#include "inc/hw_types.h"
#include "cpu.h"
#include "debug.h"


#ifndef DOXYGEN_PDF_IGNORE
//*****************************************************************************
//
// The following are values that can be passed to the Interrupt_enableInCPU()
// and Interrupt_disableInCPU() functions as the cpuInterrupt parameter.
//
//*****************************************************************************
#define INTERRUPT_CPU_INT1    0x1U      //!< CPU Interrupt Number 1
#define INTERRUPT_CPU_INT2    0x2U      //!< CPU Interrupt Number 2
#define INTERRUPT_CPU_INT3    0x4U      //!< CPU Interrupt Number 3
#define INTERRUPT_CPU_INT4    0x8U      //!< CPU Interrupt Number 4
#define INTERRUPT_CPU_INT5    0x10U     //!< CPU Interrupt Number 5
#define INTERRUPT_CPU_INT6    0x20U     //!< CPU Interrupt Number 6
#define INTERRUPT_CPU_INT7    0x40U     //!< CPU Interrupt Number 7
#define INTERRUPT_CPU_INT8    0x80U     //!< CPU Interrupt Number 8
#define INTERRUPT_CPU_INT9    0x100U    //!< CPU Interrupt Number 9
#define INTERRUPT_CPU_INT10   0x200U    //!< CPU Interrupt Number 10
#define INTERRUPT_CPU_INT11   0x400U    //!< CPU Interrupt Number 11
#define INTERRUPT_CPU_INT12   0x800U    //!< CPU Interrupt Number 12
#define INTERRUPT_CPU_INT13   0x1000U   //!< CPU Interrupt Number 13
#define INTERRUPT_CPU_INT14   0x2000U   //!< CPU Interrupt Number 14
#define INTERRUPT_CPU_DLOGINT 0x4000U   //!< CPU Data Log Interrupt
#define INTERRUPT_CPU_RTOSINT 0x8000U   //!< CPU RTOS Interrupt

//*****************************************************************************
//
// The following are values that can be passed to the Interrupt_clearACKGroup()
// function as the group parameter.
//
//*****************************************************************************
#define INTERRUPT_ACK_GROUP1    0x1U    //!< Acknowledge PIE Interrupt Group 1
#define INTERRUPT_ACK_GROUP2    0x2U    //!< Acknowledge PIE Interrupt Group 2
#define INTERRUPT_ACK_GROUP3    0x4U    //!< Acknowledge PIE Interrupt Group 3
#define INTERRUPT_ACK_GROUP4    0x8U    //!< Acknowledge PIE Interrupt Group 4
#define INTERRUPT_ACK_GROUP5    0x10U   //!< Acknowledge PIE Interrupt Group 5
#define INTERRUPT_ACK_GROUP6    0x20U   //!< Acknowledge PIE Interrupt Group 6
#define INTERRUPT_ACK_GROUP7    0x40U   //!< Acknowledge PIE Interrupt Group 7
#define INTERRUPT_ACK_GROUP8    0x80U   //!< Acknowledge PIE Interrupt Group 8
#define INTERRUPT_ACK_GROUP9    0x100U  //!< Acknowledge PIE Interrupt Group 9
#define INTERRUPT_ACK_GROUP10   0x200U  //!< Acknowledge PIE Interrupt Group 10
#define INTERRUPT_ACK_GROUP11   0x400U  //!< Acknowledge PIE Interrupt Group 11
#define INTERRUPT_ACK_GROUP12   0x800U  //!< Acknowledge PIE Interrupt Group 12
#endif

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! \internal
//! The default interrupt handler.
//!
//! This is the default interrupt handler.  The Interrupt_initVectorTable()
//! function sets all vectors to this function.  Also, when an interrupt is
//! unregistered using the Interrupt_unregister() function, this handler takes
//! its place.  This should never be called during normal operation.
//!
//! The ESTOP0 statement is for debug purposes only. Remove and replace with an
//! appropriate error handling routine for your program.
//!
//! \return None.
//
//*****************************************************************************
static void Interrupt_defaultHandler(void)
{
    uint16_t pieVect;
    uint16_t vectID;

    //
    // Calculate the vector ID. If the vector is in the lower PIE, it's the
    // offset of the vector that was fetched (bits 7:1 of PIECTRL.PIEVECT)
    // divided by two.
    //
    pieVect = HWREGH(PIECTRL_BASE + PIE_O_CTRL);
    vectID = (pieVect & 0xFEU) >> 1U;

    //
    // If the vector is in the upper PIE, the vector ID is 128 or higher.
    //
    if(pieVect >= 0x0E00U)
    {
        vectID += 128U;
    }

    //
    // Something has gone wrong. An interrupt without a proper registered
    // handler function has occurred. To help you debug the issue, local
    // variable vectID contains the vector ID of the interrupt that occurred.
    //
    ESTOP0;
    for(;;)
    {
        ;
    }
}

//*****************************************************************************
//
//! \internal
//! The default illegal instruction trap interrupt handler.
//!
//! This is the default interrupt handler for an illegal instruction trap
//! (ITRAP).  The Interrupt_initVectorTable() function sets the appropriate
//! vector to this function.  This should never be called during normal
//! operation.
//!
//! The ESTOP0 statement is for debug purposes only.  Remove and replace with
//! an appropriate error handling routine for your program.
//!
//! \return None.
//
//*****************************************************************************
static void Interrupt_illegalOperationHandler(void)
{
    //
    // Something has gone wrong.  The CPU has tried to execute an illegal
    // instruction, generating an illegal instruction trap (ITRAP).
    //
    ESTOP0;
    for(;;)
    {
        ;
    }
}

//*****************************************************************************
//
//! \internal
//! The default non-maskable interrupt handler.
//!
//! This is the default interrupt handler for a non-maskable interrupt (NMI).
//! The Interrupt_initVectorTable() function sets the appropriate vector to
//! this function.  This should never be called during normal operation.
//!
//! The ESTOP0 statement is for debug purposes only. Remove and replace with an
//! appropriate error handling routine for your program.
//!
//! \return None.
//
//*****************************************************************************
static void Interrupt_nmiHandler(void)
{
    //
    // A non-maskable interrupt has occurred, indicating that a hardware error
    // has occurred in the system.  You can use SysCtl_getNMIFlagStatus() to
    // to read the NMIFLG register and determine what caused the NMI.
    //
    ESTOP0;
    for(;;)
    {
        ;
    }
}

//*****************************************************************************
//
//! Allows the CPU to process interrupts.
//!
//! This function clears the global interrupt mask bit (INTM) in the CPU,
//! allowing the processor to respond to interrupts.
//!
//! \return Returns \b true if interrupts were disabled when the function was
//! called or \b false if they were initially enabled.
//
//*****************************************************************************
static inline bool
Interrupt_enableGlobal(void)
{
    //
    // Enable processor interrupts.
    //
    return(((__enable_interrupts() & 0x1U) != 0U) ? true : false);
}

//*****************************************************************************
//
//! Stops the CPU from processing interrupts.
//!
//! This function sets the global interrupt mask bit (INTM) in the CPU,
//! preventing the processor from receiving maskable interrupts.
//!
//! \return Returns \b true if interrupts were already disabled when the
//! function was called or \b false if they were initially enabled.
//
//*****************************************************************************
static inline bool
Interrupt_disableGlobal(void)
{
    //
    // Disable processor interrupts.
    //
    return(((__disable_interrupts() & 0x1U) != 0U) ? true : false);
}

//*****************************************************************************
//
//! Registers a function to be called when an interrupt occurs.
//!
//! \param interruptNumber specifies the interrupt in question.
//! \param handler is a pointer to the function to be called.
//!
//! This function is used to specify the handler function to be called when the
//! given interrupt is asserted to the processor.  When the interrupt occurs,
//! if it is enabled (via Interrupt_enable()), the handler function will be
//! called in interrupt context.  Since the handler function can preempt other
//! code, care must be taken to protect memory or peripherals that are accessed
//! by the handler and other non-handler code.
//!
//! The available \e interruptNumber values are supplied in
//! <tt>inc/hw_ints.h</tt>.
//!
//! \note This function assumes that the PIE has been enabled. See
//! Interrupt_initModule().
//!
//! \return None.
//
//*****************************************************************************
static inline void
Interrupt_register(uint32_t interruptNumber, void (*handler)(void))
{
    uint32_t address;

    //
    // Calculate appropriate address for the interrupt number
    //
    address = (uint32_t)PIEVECTTABLE_BASE +
              (((interruptNumber & 0xFFFF0000U) >> 16U) * 2U);

    //
    // Copy ISR address into PIE table
    //
    EALLOW;
    HWREG(address) = (uint32_t)handler;
    EDIS;
}

//*****************************************************************************
//
//! Unregisters the function to be called when an interrupt occurs.
//!
//! \param interruptNumber specifies the interrupt in question.
//!
//! This function is used to indicate that a default handler
//! Interrupt_defaultHandler() should be called when the given interrupt is
//! asserted to the processor.  Call Interrupt_disable() to disable
//! the interrupt before calling this function.
//!
//! The available \e interruptNumber values are supplied in
//! <tt>inc/hw_ints.h</tt>.
//!
//! \sa Interrupt_register() for important information about registering
//! interrupt handlers.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Interrupt_unregister(uint32_t interruptNumber)
{
    uint32_t address;

    //
    // Calculate appropriate address for the interrupt number
    //
    address = (uint32_t)PIEVECTTABLE_BASE +
              (((interruptNumber & 0xFFFF0000U) >> 16U) * 2U);

    //
    // Copy default ISR address into PIE table
    //
    EALLOW;
    HWREG(address) = (uint32_t)Interrupt_defaultHandler;
    EDIS;
}

//*****************************************************************************
//
//! Enables CPU interrupt channels
//!
//! \param cpuInterrupt specifies the CPU interrupts to be enabled.
//!
//! This function enables the specified interrupts in the CPU. The
//! \e cpuInterrupt parameter is a logical OR of the values
//! \b INTERRUPT_CPU_INTx where x is the interrupt number between 1 and 14,
//! \b INTERRUPT_CPU_DLOGINT, and \b INTERRUPT_CPU_RTOSINT.
//!
//! \note Note that interrupts 1-12 correspond to the PIE groups with those
//! same numbers.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Interrupt_enableInCPU(uint16_t cpuInterrupt)
{
    //
    // Set the interrupt bits in the CPU.
    //
    IER |= cpuInterrupt;
}

//*****************************************************************************
//
//! Disables CPU interrupt channels
//!
//! \param cpuInterrupt specifies the CPU interrupts to be disabled.
//!
//! This function disables the specified interrupts in the CPU. The
//! \e cpuInterrupt parameter is a logical OR of the values
//! \b INTERRUPT_CPU_INTx where x is the interrupt number between 1 and 14,
//! \b INTERRUPT_CPU_DLOGINT, and \b INTERRUPT_CPU_RTOSINT.
//!
//! \note Note that interrupts 1-12 correspond to the PIE groups with those
//! same numbers.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Interrupt_disableInCPU(uint16_t cpuInterrupt)
{
    //
    // Clear the interrupt bits in the CPU.
    //
    IER &= ~cpuInterrupt;
}

//*****************************************************************************
//
//! Acknowledges PIE Interrupt Group
//!
//! \param group specifies the interrupt group to be acknowledged.
//!
//! The specified interrupt group is acknowledged and clears any interrupt
//! flag within that respective group.
//!
//! The \e group parameter must be a logical OR of the following:
//! \b INTERRUPT_ACK_GROUP1, \b INTERRUPT_ACK_GROUP2, \b INTERRUPT_ACK_GROUP3
//! \b INTERRUPT_ACK_GROUP4, \b INTERRUPT_ACK_GROUP5, \b INTERRUPT_ACK_GROUP6
//! \b INTERRUPT_ACK_GROUP7, \b INTERRUPT_ACK_GROUP8, \b INTERRUPT_ACK_GROUP9
//! \b INTERRUPT_ACK_GROUP10, \b INTERRUPT_ACK_GROUP11,
//! \b INTERRUPT_ACK_GROUP12.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Interrupt_clearACKGroup(uint16_t group)
{
    //
    // Set interrupt group acknowledge bits
    //
    HWREGH(PIECTRL_BASE + PIE_O_ACK) = group;
}

//*****************************************************************************
//
//! Enables the PIE block.
//!
//! This function enables the vector fetching for the peripheral interrupts by
//! enabling the PIE block.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Interrupt_enablePIE(void)
{
    HWREGH(PIECTRL_BASE + PIE_O_CTRL) |= PIE_CTRL_ENPIE;
}

//*****************************************************************************
//
//! Disables the PIE block.
//!
//! This function disables the vector fetching for the peripheral interrupts by
//! disabling the PIE block. PIEACK, PIEIFR, and PIEIER registers can be
//! accessed even when the PIE block is disabled.
//!
//! \return None.
//
//*****************************************************************************
static inline void
Interrupt_disablePIE(void)
{
    HWREGH(PIECTRL_BASE + PIE_O_CTRL) &= ~PIE_CTRL_ENPIE;
}

//*****************************************************************************
//
//! Initializes the PIE control registers by setting them to a known state.
//!
//! This function initializes the PIE control registers. After globally
//! disabling interrupts and enabling the PIE, it clears all of the PIE
//! interrupt enable bits and interrupt flags.
//!
//! \return None.
//
//*****************************************************************************
extern void
Interrupt_initModule(void);

//*****************************************************************************
//
//! Initializes the PIE vector table by setting all vectors to a default
//! handler function.
//!
//! \return None.
//
//*****************************************************************************
extern void
Interrupt_initVectorTable(void);

//*****************************************************************************
//
//! Enables an interrupt.
//!
//! \param interruptNumber specifies the interrupt to be enabled.
//!
//! The specified interrupt is enabled in the interrupt controller.  Other
//! enables for the interrupt (such as at the peripheral level) are unaffected
//! by this function.
//!
//! The available \e interruptNumber values are supplied in
//! <tt>inc/hw_ints.h</tt>.
//!
//! \return None.
//
//*****************************************************************************
extern void
Interrupt_enable(uint32_t interruptNumber);

//*****************************************************************************
//
//! Disables an interrupt.
//!
//! \param interruptNumber specifies the interrupt to be disabled.
//!
//! The specified interrupt is disabled in the interrupt controller.  Other
//! enables for the interrupt (such as at the peripheral level) are unaffected
//! by this function.
//!
//! The available \e interruptNumber values are supplied in
//! <tt>inc/hw_ints.h</tt>.
//!
//! \return None.
//
//*****************************************************************************
extern void
Interrupt_disable(uint32_t interruptNumber);

//*****************************************************************************
//
// Extern compiler intrinsic prototypes. See compiler User's Guide for details.
//
//*****************************************************************************
extern uint16_t __disable_interrupts(void);
extern uint16_t __enable_interrupts(void);

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

#endif // INTERRUPT_H
