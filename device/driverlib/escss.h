//###########################################################################
//
// FILE:   escss.h
//
// TITLE:  C28x EtherCAT SS Driver.
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

#ifndef ESCSS_H
#define ESCSS_H

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
//! \addtogroup escss_api ESCSS
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_esc_ss.h"
#include "debug.h"
#include "sysctl.h"

//
// Define for Mask Value to get the IP Revision Number.
//
#define ESCSS_IPREV_MASK (ESCSS_IPREVNUM_IP_REV_MINOR_S |\
                          ESCSS_IPREVNUM_IP_REV_MINOR_M)

//
// Define for Mask Value to get the Raw Interrupt Mask.
//
#define ESCSS_RAW_INTERRUPT_MASK (ESCSS_INTR_RIS_SYNC0_RIS |\
                                  ESCSS_INTR_RIS_SYNC1_RIS |\
                                  ESCSS_INTR_RIS_IRQ_RIS |\
                                  ESCSS_INTR_RIS_DMA_DONE_RIS |\
                                  ESCSS_INTR_RIS_TIMEOUT_ERR_RIS |\
                                  ESCSS_INTR_RIS_MASTER_RESET_RIS)

//
// Defines for the Group Capture Select Mask.
//
#define ESCSS_GRP_CAP_SELECT0_MASK ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL0_M
#define ESCSS_GRP_CAP_SELECT1_MASK ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL1_M
#define ESCSS_GRP_CAP_SELECT2_MASK ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL2_M
#define ESCSS_GRP_CAP_SELECT3_MASK ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL3_M

//
// Defines for Valid and Bad Key Return from Various API's.
//
#define ESCSS_API_SUCCESS  0x01U
#define ESCSS_API_FAIL   0x00U

//
// Define for the Valid Key Value.
//
#define ESCSS_VALID_KEY_VALUE 0xA5U

//
// Defines for GPIN0 to GPIN31.
//
#define ESCSS_GPIN_0  0x1U
#define ESCSS_GPIN_1  0x2U
#define ESCSS_GPIN_2  0x4U
#define ESCSS_GPIN_3  0x8U
#define ESCSS_GPIN_4  0x10U
#define ESCSS_GPIN_5  0x20U
#define ESCSS_GPIN_6  0x40U
#define ESCSS_GPIN_7  0x80U
#define ESCSS_GPIN_8  0x100U
#define ESCSS_GPIN_9  0x200U
#define ESCSS_GPIN_10 0x400U
#define ESCSS_GPIN_11 0x800U
#define ESCSS_GPIN_12 0x1000U
#define ESCSS_GPIN_13 0x2000U
#define ESCSS_GPIN_14 0x4000U
#define ESCSS_GPIN_15 0x8000U
#define ESCSS_GPIN_16 0x10000U
#define ESCSS_GPIN_17 0x20000U
#define ESCSS_GPIN_18 0x40000U
#define ESCSS_GPIN_19 0x80000U
#define ESCSS_GPIN_20 0x100000U
#define ESCSS_GPIN_21 0x200000U
#define ESCSS_GPIN_22 0x400000U
#define ESCSS_GPIN_23 0x800000U
#define ESCSS_GPIN_24 0x1000000U
#define ESCSS_GPIN_25 0x2000000U
#define ESCSS_GPIN_26 0x4000000U
#define ESCSS_GPIN_27 0x8000000U
#define ESCSS_GPIN_28 0x10000000U
#define ESCSS_GPIN_29 0x20000000U
#define ESCSS_GPIN_30 0x40000000U
#define ESCSS_GPIN_31 0x80000000U

//
// Defines for GPOUT0 to GPOUT31.
//
#define ESCSS_GPOUT_0  0x1U
#define ESCSS_GPOUT_1  0x2U
#define ESCSS_GPOUT_2  0x4U
#define ESCSS_GPOUT_3  0x8U
#define ESCSS_GPOUT_4  0x10U
#define ESCSS_GPOUT_5  0x20U
#define ESCSS_GPOUT_6  0x40U
#define ESCSS_GPOUT_7  0x80U
#define ESCSS_GPOUT_8  0x100U
#define ESCSS_GPOUT_9  0x200U
#define ESCSS_GPOUT_10 0x400U
#define ESCSS_GPOUT_11 0x800U
#define ESCSS_GPOUT_12 0x1000U
#define ESCSS_GPOUT_13 0x2000U
#define ESCSS_GPOUT_14 0x4000U
#define ESCSS_GPOUT_15 0x8000U
#define ESCSS_GPOUT_16 0x10000U
#define ESCSS_GPOUT_17 0x20000U
#define ESCSS_GPOUT_18 0x40000U
#define ESCSS_GPOUT_19 0x80000U
#define ESCSS_GPOUT_20 0x100000U
#define ESCSS_GPOUT_21 0x200000U
#define ESCSS_GPOUT_22 0x400000U
#define ESCSS_GPOUT_23 0x800000U
#define ESCSS_GPOUT_24 0x1000000U
#define ESCSS_GPOUT_25 0x2000000U
#define ESCSS_GPOUT_26 0x4000000U
#define ESCSS_GPOUT_27 0x8000000U
#define ESCSS_GPOUT_28 0x10000000U
#define ESCSS_GPOUT_29 0x20000000U
#define ESCSS_GPOUT_30 0x40000000U
#define ESCSS_GPOUT_31 0x80000000U

//*****************************************************************************
//
//! This data type is used to define the signal hookup to 32 possible LATCH0/1
//! trigger sources.
//
//*****************************************************************************
typedef enum
{
    ESCSS_TRIGGER_LATCH0 = 0U,       //!< Latch0 Trigger.
    ESCSS_TRIGGER_LATCH1 = 1U,       //!< Latch1 Trigger
    ESCSS_TRIGGER_CPUNMI = 2U,       //!< CPUNMI Trigger.
    ESCSS_TRIGGER_CMNMI = 3U,        //!< CMNMI Trigger.
    ESCSS_TRIGGER_ERRORSTS = 4U,     //!< ERRORSTS Trigger.
    ESCSS_TRIGGER_GPTRIPOUT0 = 5U,   //!< GPTRIPOUT0 Trigger.
    ESCSS_TRIGGER_GPTRIPOUT1 = 6U,   //!< GPTRIPOUT1 Trigger.
    ESCSS_TRIGGER_GPTRIPOUT2 = 7U,   //!< GPTRIPOUT2 Trigger.
    ESCSS_TRIGGER_GPTRIPOUT3 = 8U,   //!< GPTRIPOUT3 Trigger.
    ESCSS_TRIGGER_GPTRIPOUT4 = 9U,   //!< GPTRIPOUT4 Trigger.
    ESCSS_TRIGGER_GPTRIPOUT5 = 10U,  //!< GPTRIPOUT5 Trigger.
    ESCSS_TRIGGER_GPTRIPOUT6 = 11U,  //!< GPTRIPOUT6 Trigger.
    ESCSS_TRIGGER_GPTRIPOUT7 = 12U,  //!< GPTRIPOUT7 Trigger.
    ESCSS_TRIGGER_GPTRIPOUT8 = 13U,  //!< GPTRIPOUT8 Trigger.
    ESCSS_TRIGGER_GPTRIPOUT9 = 14U,  //!< GPTRIPOUT9 Trigger.
    ESCSS_TRIGGER_GPTRIPOUT10 = 15U, //!< GPTRIPOUT10 Trigger.
    ESCSS_TRIGGER_GPTRIPOUT11 = 16U, //!< GPTRIPOUT11 Trigger.
    ESCSS_TRIGGER_GPTRIPOUT12 = 17U, //!< GPTRIPOUT12 Trigger.
    ESCSS_TRIGGER_GPTRIPOUT13 = 18U, //!< GPTRIPOUT13 Trigger.
    ESCSS_TRIGGER_GPTRIPOUT14 = 19U, //!< GPTRIPOUT14 Trigger.
    ESCSS_TRIGGER_GPTRIPOUT15 = 20U, //!< GPTRIPOUT15 Trigger.
    ESCSS_TRIGGER_PWMXBAROUT0 = 21U, //!< PWMXBAROUT0 Trigger.
    ESCSS_TRIGGER_PWMXBAROUT1 = 22U, //!< PWMXBAROUT1 Trigger.
    ESCSS_TRIGGER_PWMXBAROUT2 = 23U, //!< PWMXBAROUT2 Trigger.
    ESCSS_TRIGGER_PWMXBAROUT3 = 24U, //!< PWMXBAROUT3 Trigger.
    ESCSS_TRIGGER_PWMXBAROUT4 = 25U, //!< PWMXBAROUT4 Trigger.
    ESCSS_TRIGGER_PWMXBAROUT5 = 26U, //!< PWMXBAROUT5 Trigger.
    ESCSS_TRIGGER_PWMXBAROUT6 = 27U, //!< PWMXBAROUT6 Trigger.
    ESCSS_TRIGGER_PWMXBAROUT7 = 28U  //!< PWMXBAROUT7 Trigger.
} ESCSS_LatchTrigger;

//*****************************************************************************
//
//! The definitions are used to select the capture trigger.
//
//*****************************************************************************
typedef enum
{
    ESCSS_SOF_CAPTURE_TRIGGER = 0U,    //!< Start of Frame as Capture Trigger
                                       //!< Select.
    ESCSS_SYNC0_CAPTURE_TRIGGER = 4U,  //!< SYNC0 as Capture Trigger Select.
    ESCSS_SYNC1_CAPTURE_TRIGGER = 5U,  //!< SYNC1 as Capture Trigger Select.
    ESCSS_LATCH0_CAPTURE_TRIGGER = 6U, //!< LATCH0 as Capture Trigger Select.
    ESCSS_LATCH1_CAPTURE_TRIGGER = 7U  //!< LATCH1 as Capture Trigger Select.
} ESCSS_CaptureTrigger;

//*****************************************************************************
//
//! The definition are there to select number of Ports.
//
//*****************************************************************************
typedef enum
{
    ESCSS_ONE_PORT_SELECTION = 0U, //!< One port operation(Port0).
    ESCSS_TWO_PORT_SELECTION = 1U  //!< Two port operation(Port0,Port1).
} ESCSS_PortSelection;

//*****************************************************************************
//
//! The definition values are there to select the size to pass as a parameter
//! to ESCSS_configureEEPROMSize().
//
//*****************************************************************************
typedef enum
{
    ESCSS_LESS_THAN_16K,        //!< EEPROMs of size 16K bits or less.
    ESCSS_GREATER_THAN_16K      //!< EEPROMs of size greater than 16K bits.
} ESCSS_SizeSelect;

//*****************************************************************************
//
//! The definition values is there to select the Group Capture Trigger for the
//! ESCSS_setGPINGroupCaptureTriggerSelect and
//! ESCSS_setGPOUTGroupCaptureTriggerSelect APIs.
//
//*****************************************************************************
typedef enum
{
    ESCSS_GROUP_CAPTURE_SELECT0 = ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL0_S,
    ESCSS_GROUP_CAPTURE_SELECT1 = ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL1_S,
    ESCSS_GROUP_CAPTURE_SELECT2 = ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL2_S,
    ESCSS_GROUP_CAPTURE_SELECT3 = ESCSS_GPIN_GRP_CAP_SEL_GPI_GRP_CAP_SEL3_S
} ESCSS_GroupCaptureSelect;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! \internal
//!
//! Checks the EtherCAT Sub-System base address.
//!
//! \param base is the base address of EtherCAT Sub-System.
//!
//! This function determines if EtherCAT Sub-System base address passed is
//! valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
ESCSS_isBaseValid(uint32_t base)
{
    return((ESC_SS_BASE == base));
}
#endif

//*****************************************************************************
//
//! \internal
//!
//! Checks the EtherCAT Sub-System Configuration base address.
//!
//! \param base is the base address of EtherCAT Sub-System Configuration
//!        Registers Base.
//!
//! This function determines if EtherCAT Sub-System Configuration Registers
//! base address passed is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
ESCSS_isConfigBaseValid(uint32_t base)
{
    return((ESC_SS_CONFIG_BASE == base));
}
#endif

//*****************************************************************************
//
//! Reads the Minor IP Revision Number for EtherCAT.
//!
//! \param base is the base address of EtherCAT Sub-System.
//!
//! This function returns the Minor IP Revision Number. Reset value for this
//! are Hard Coded and increments with minor changes to the IP.
//!
//! \return Returns the EtherCAT Minor IP Revision Number.
//
//*****************************************************************************
static inline uint16_t
ESCSS_readIPMinorRevNumber(uint32_t base)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Returns the Minor IP Revision Number.
    //
    return(((HWREG(base + ESCSS_O_IPREVNUM)) &
            ESCSS_IPREVNUM_IP_REV_MINOR_M) >>
           ESCSS_IPREVNUM_IP_REV_MINOR_S);
}

//*****************************************************************************
//
//! Reads the Major IP Revision Number for EtherCAT.
//!
//! \param base is the base address of EtherCAT Sub-System.
//!
//! This function returns the Major IP Revision Number. Reset value for this
//! are Hard Coded and increments with minor changes to the IP.
//!
//! \return Returns the EtherCAT Major IP Revision Number.
//
//*****************************************************************************
static inline uint16_t
ESCSS_readIPMajorRevNumber(uint32_t base)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Returns the Major IP Revision Number.
    //
    return(((HWREG(base + ESCSS_O_IPREVNUM)) &
            ESCSS_IPREVNUM_IP_REV_MAJOR_M) >>
           ESCSS_IPREVNUM_IP_REV_MAJOR_S);
}

//*****************************************************************************
//
//! Reads the Major and Minor IP Revision Number for EtherCAT.
//!
//! \param base is the base address of EtherCAT Sub-System.
//!
//! This function returns the IP Revision Number. Reset value for this
//! are Hard Coded and increments with minor changes to the IP.
//!
//! \return Returns the EtherCAT IP Revision Number.
//
//*****************************************************************************
static inline uint32_t
ESCSS_readIPRevNumber(uint32_t base)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Return the IP Revision Number.
    //
    return((HWREG(base + ESCSS_O_IPREVNUM)) & ESCSS_IPREV_MASK);
}

//*****************************************************************************
//
//! Gets the Raw Interrupt Status for selected interrupts.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param interruptMask is the mask value to select the different type of
//!        interrupt.
//!
//! This function returns the Raw Interrupt Status of all the different
//! interrupt triggers.
//!
//! \return Returns the Raw Interrupt Status Value.
//
//*****************************************************************************
static inline uint16_t
ESCSS_getRawInterruptStatus(uint32_t base, uint16_t interruptMask)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));
    ASSERT((interruptMask & ~(ESCSS_INTR_RIS_SYNC0_RIS |
                              ESCSS_INTR_RIS_SYNC1_RIS |
                              ESCSS_INTR_RIS_IRQ_RIS |
                              ESCSS_INTR_RIS_DMA_DONE_RIS |
                              ESCSS_INTR_RIS_TIMEOUT_ERR_RIS |
                              ESCSS_INTR_RIS_MASTER_RESET_RIS)) == 0U);

    //
    // Return the specific Raw Interrupt Status Value.
    //
    return((HWREG(base + ESCSS_O_INTR_RIS)) & interruptMask);
}

//*****************************************************************************
//
//! Reads the Raw Interrupt Status for different interrupt triggers.
//!
//! \param base is the base address of EtherCAT Sub-System.
//!
//! This function returns the Raw Interrupt Status of individual interrupt
//! triggers.
//!
//! The Raw interrupt Status can have the following valid options for
//! \e interruptMask can be OR'ed together:
//!  \b ESCSS_INTR_RIS_SYNC0_RIS, \b ESCSS_INTR_RIS_SYNC1_RIS,
//!  \b ESCSS_INTR_RIS_IRQ_RIS, \b ESCSS_INTR_RIS_DMA_DONE_RIS,
//!  \b ESCSS_INTR_RIS_TIMEOUT_ERR_RIS, \b ESCSS_INTR_RIS_MASTER_RESET_RIS.
//!
//! \return Returns the Raw Interrupt Status Value.
//
//*****************************************************************************
static inline uint32_t
ESCSS_readRawInterruptStatus(uint32_t base)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Return the Raw Interrupt Status Value.
    //
    return(HWREG(base + ESCSS_O_INTR_RIS) & ESCSS_RAW_INTERRUPT_MASK);
}

//*****************************************************************************
//
//! Allows to mask individual interrupt cause impacting the interrupt.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param interruptMask is the mask value to select the different type of
//!        interrupt.
//!
//! This function sets the Masked Interrupt Status of the different
//! interrupt triggers as per the input mask value.
//!
//! The Masked interrupt Status can have the following valid options for
//! \e interruptMask can be OR'ed together:
//! - \b ESCSS_INTR_MASK_SYNC0_MASK - Masks SYNC0 to have effect on interrupts
//!      or other CPU/DMA Trigger,
//! - \b ESCSS_INTR_MASK_SYNC1_MASK - Masks SYNC1 to have effect on interrupts
//!      or other CPU/DMA Trigger,
//! - \b ESCSS_INTR_MASK_IRQ_MASK - Masks EtherCATSS IRQ to have effect on
//!      interrupts or other CPU/DMA Trigger,
//! - \b ESCSS_INTR_MASK_DMA_DONE_MASK - Masks DMA Done to have effect on
//!      interrupts or other CPU/DMA Trigger,
//! - \b ESCSS_INTR_MASK_TIMEOUT_ERR_MASK - Masks PDI access timeout Error to
//!      have effect on interrupts or other CPU/DMA Trigger,
//! - \b ESCSS_INTR_MASK_MASTER_RESET_MASK - Masks EtherCAT Master reset event
//!      against any effect on interrupts or other CPU Interrupts.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_setMaskedInterruptStatus(uint32_t base, uint16_t interruptMask)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));
    ASSERT((interruptMask & ~(ESCSS_INTR_MASK_SYNC0_MASK |
                              ESCSS_INTR_MASK_SYNC1_MASK |
                              ESCSS_INTR_MASK_IRQ_MASK |
                              ESCSS_INTR_MASK_DMA_DONE_MASK |
                              ESCSS_INTR_MASK_TIMEOUT_ERR_MASK |
                              ESCSS_INTR_MASK_MASTER_RESET_MASK)) == 0U);
    //
    // Set the Masked Interrupt Status Value.
    //
    HWREGH(base + ESCSS_O_INTR_MASK) |= interruptMask;
}

//*****************************************************************************
//
//! Allows to reset the mask individual interrupt cause impacting the
//! interrupt.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param interruptMask is the mask value to select the different type of
//!        interrupt.
//!
//! This function resets the Masked Interrupt Status of the different
//! interrupt triggers as per the input mask value passed as parameter.
//!
//! The Masked interrupt Status can have the following valid options for
//! \e interruptMask can be OR'ed together:
//! - \b ESCSS_INTR_MASK_SYNC0_MASK - Masks SYNC0 to have effect on interrupts
//!      or other CPU/DMA Trigger,
//! - \b ESCSS_INTR_MASK_SYNC1_MASK - Masks SYNC1 to have effect on interrupts
//!      or other CPU/DMA Trigger,
//! - \b ESCSS_INTR_MASK_IRQ_MASK - Masks EtherCATSS IRQ to have effect on
//!      interrupts or other CPU/DMA Trigger,
//! - \b ESCSS_INTR_MASK_DMA_DONE_MASK - Masks DMA Done to have effect on
//!      interrupts or other CPU/DMA Trigger,
//! - \b ESCSS_INTR_MASK_TIMEOUT_ERR_MASK - Masks PDI access timeout Error to
//!      have effect on interrupts or other CPU/DMA Trigger,
//! - \b ESCSS_INTR_MASK_MASTER_RESET_MASK - Masks EtherCAT Master reset event
//!      against any effect on interrupts or other CPU Interrupts.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_resetMaskedInterruptStatus(uint32_t base, uint16_t interruptMask)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));
    ASSERT((interruptMask & ~(ESCSS_INTR_MASK_SYNC0_MASK |
                              ESCSS_INTR_MASK_SYNC1_MASK |
                              ESCSS_INTR_MASK_IRQ_MASK |
                              ESCSS_INTR_MASK_DMA_DONE_MASK |
                              ESCSS_INTR_MASK_TIMEOUT_ERR_MASK |
                              ESCSS_INTR_MASK_MASTER_RESET_MASK)) == 0U);
    //
    // Reset the Masked Interrupt Status Value.
    //
    HWREGH(base + ESCSS_O_INTR_MASK) &= ~interruptMask;
}

//*****************************************************************************
//
//! Gets the Masked Interrupt Status all interrupt triggers.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param interruptMask is the mask value to select the different type of
//!        interrupt.
//!
//! This function returns the Masked Interrupt Status of the different
//! interrupt triggers as per the input mask value.
//!
//! The Masked interrupt Status can have the following valid options for
//! \e interruptMask can be OR'ed together:
//!  \b ESCSS_INTR_MIS_SYNC0_MIS, \b ESCSS_INTR_MIS_SYNC1_MIS,
//!  \b ESCSS_INTR_MIS_IRQ_MIS, \b ESCSS_INTR_MIS_DMA_DONE_MIS,
//!  \b ESCSS_INTR_MIS_TIMEOUT_ERR_MIS, \b ESCSS_INTR_MIS_MASTER_RESET_MIS.
//!
//! \return Returns the Masked Interrupt Status Value.
//
//*****************************************************************************
static inline uint16_t
ESCSS_getMaskedInterruptStatus(uint32_t base, uint16_t interruptMask)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));
    ASSERT((interruptMask & ~(ESCSS_INTR_MIS_SYNC0_MIS |
                              ESCSS_INTR_MIS_SYNC1_MIS |
                              ESCSS_INTR_MIS_IRQ_MIS |
                              ESCSS_INTR_MIS_DMA_DONE_MIS |
                              ESCSS_INTR_MIS_TIMEOUT_ERR_MIS |
                              ESCSS_INTR_MIS_MASTER_RESET_MIS)) == 0U);

    //
    // Return the Masked Interrupt Status Value.
    //
    return((HWREGH(base + ESCSS_O_INTR_MIS)) & interruptMask);
}

//*****************************************************************************
//
//! Clears the individual interrupt cause.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param interruptMask is the mask value to select the different type of
//!        interrupt.
//!
//! This function clears the individual interrupt cause based on the mask value
//! passed. Writing a 1 clears the Raw Status.
//!
//! The interrupt Status Mask parameter can have the following valid options
//! for \e interruptMask can be OR'ed together:
//!  \b ESCSS_INTR_CLR_SYNC0_CLR, \b ESCSS_INTR_CLR_SYNC1_CLR,
//!  \b ESCSS_INTR_CLR_IRQ_CLR, \b ESCSS_INTR_CLR_DMA_DONE_CLR,
//!  \b ESCSS_INTR_CLR_TIMEOUT_ERR_CLR, \b ESCSS_INTR_CLR_MASTER_RESET_CLR.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_clearRawInterruptStatus(uint32_t base, uint16_t interruptMask)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));
    ASSERT((interruptMask & ~(ESCSS_INTR_CLR_SYNC0_CLR |
                              ESCSS_INTR_CLR_SYNC1_CLR |
                              ESCSS_INTR_CLR_IRQ_CLR |
                              ESCSS_INTR_CLR_DMA_DONE_CLR |
                              ESCSS_INTR_CLR_TIMEOUT_ERR_CLR |
                              ESCSS_INTR_CLR_MASTER_RESET_CLR)) == 0U);
    //
    // Clear the Interrupt Status using the Mask Value passed.
    //
    HWREGH(base + ESCSS_O_INTR_CLR) = interruptMask;
}

//*****************************************************************************
//
//! Selects the LATCH0 Inputs Mux Select.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param muxSelectValue is the mux select value for Latch0.
//!
//! This function sets the Mux Select value for LATCH0 input to ESC SS.
//!
//! The \e muxSelectValue parameter can be one of the possible values from
//! the ESCSS_LatchTrigger enum.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_selectLatch0Mux(uint32_t base, ESCSS_LatchTrigger muxSelectValue)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Set the Mux Select Value for Latch0.
    //
    HWREGH(base + ESCSS_O_LATCH_SEL) |=
    (((uint32_t)muxSelectValue << ESCSS_LATCH_SEL_LATCH0_SELECT_S) &
     ESCSS_LATCH_SEL_LATCH0_SELECT_M);
}

//*****************************************************************************
//
//! Select the LATCH1 Inputs Mux Select.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param muxSelectValue is the mux select value for Latch1.
//!
//! This function sets the Mux Select value for LATCH1 input to ESC SS.
//!
//! The \e muxSelectValue parameter can be one of the possible values from
//! the ESCSS_LatchTrigger enum.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_selectLatch1Mux(uint32_t base, ESCSS_LatchTrigger muxSelectValue)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Set the Mux Select Value for Latch1.
    //
    HWREGH(base + ESCSS_O_LATCH_SEL) |=
    (((uint32_t)muxSelectValue << ESCSS_LATCH_SEL_LATCH1_SELECT_S) &
     ESCSS_LATCH_SEL_LATCH1_SELECT_M);
}

//*****************************************************************************
//
//! Configures the Wait State on the 16 bit Asynchronous Interface.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param waitState is the minimum wait state value.
//!
//! This function configures the predefined minimum number of wait-states which
//! the VBUS bridge will put out accesses on the 16-bit Asynchronous interface.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_configure16BitAsyncAccessWaitState(uint32_t base, uint16_t waitState)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Configure the Wait State Value.
    //
    HWREGH(base + ESCSS_O_ACCESS_CTRL) |=
    ((waitState << ESCSS_ACCESS_CTRL_WAIT_STATES_S) &
     ESCSS_ACCESS_CTRL_WAIT_STATES_M);
}

//*****************************************************************************
//
//! Enables the PDI Timeout Feature.
//!
//! \param base is the base address of EtherCAT Sub-System.
//!
//! Enables the Timeout features which counts programmed number of Sys clocks
//! before the Local host aborts the transaction.
//! The timeout counter starts counting upon BUSY is asserted by EtherCAT IP.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_enablePDIAccessTimeOut(uint32_t base)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Enable timeout.
    //
    HWREGH(base + ESCSS_O_ACCESS_CTRL) |= ESCSS_ACCESS_CTRL_EN_TIMEOUT;
}

//*****************************************************************************
//
//! Disables the PDI Timeout Feature.
//!
//! \param base is the base address of EtherCAT Sub-System.
//!
//! Disables the Timeout features which counts programmed number of Sys clocks
//! before the Local host aborts the transaction on PDI interface.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_disablePDIAccessTimeOut(uint32_t base)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Disable Timeout.
    //
    HWREGH(base + ESCSS_O_ACCESS_CTRL) &= (~ESCSS_ACCESS_CTRL_EN_TIMEOUT);
}

//*****************************************************************************
//
//! Enables the Debug Access through the PDI Controller.
//!
//! \param base is the base address of EtherCAT Sub-System.
//!
//! This function configures the debug accesses to be allowed to go
//! through.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_enableDebugAccess(uint32_t base)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Set the Bit for Debug Access.
    //
    HWREGH(base + ESCSS_O_ACCESS_CTRL) |=
    ESCSS_ACCESS_CTRL_ENABLE_DEBUG_ACCESS;
}

//*****************************************************************************
//
//! Disables the Debug Access through the PDI Controller.
//!
//! \param base is the base address of EtherCAT Sub-System.
//!
//! This function configures the debug accesses to be not allowed to go
//! through.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_disableDebugAccess(uint32_t base)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Reset the Bit for Debug Access.
    //
    HWREGH(base + ESCSS_O_ACCESS_CTRL) &=
    ~ESCSS_ACCESS_CTRL_ENABLE_DEBUG_ACCESS;
}


//*****************************************************************************
//
//! Reads the GPIN Data.
//!
//! \param base is the base address of EtherCAT Sub-System.
//!
//! This function returns the GPIN Data. Local GPIN data register connects to
//! GPIN pipelined register for debug & override purposes.
//!
//! \return Returns the GPIN Data Value.
//
//*****************************************************************************
static inline uint32_t
ESCSS_readGPINData(uint32_t base)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Return the GPIN Data Read from the register.
    //
    return(HWREG(base + ESCSS_O_GPIN_DAT));
}

//*****************************************************************************
//
//! Sets the GPIN Data Register Value.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param pinValue is the value to be written in the GPIN Register.
//!
//! This function writes the GPIN Data. Local GPIN data register connects to
//! GPIN pipelined register for debug & override purposes.
//!
//! The \e pinValue can have the respective GPIN Defines OR'ed to select
//! which PIN to write to. The defines are ESCSS_GPIN_x where x is from 0 to
//! 31.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_setGPINData(uint32_t base, uint32_t pinValue)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Set the GPIN data register value.
    //
    HWREG(base + ESCSS_O_GPIN_DAT) |= pinValue;
}

//*****************************************************************************
//
//! Resets the GPIN Data Register Value.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param pinValue is the value to be written in the GPIN Register.
//!
//! This function writes the GPIN Data. Local GPIN data register connects to
//! GPIN pipelined register for debug & override purposes.
//!
//! The \e pinValue can have the respective GPIN Defines OR'ed to select
//! which PIN to write to. The defines are ESCSS_GPIN_x where x is from 0 to
//! 31.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_resetGPINData(uint32_t base, uint32_t pinValue)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Reset the GPIN data register value.
    //
    HWREG(base + ESCSS_O_GPIN_DAT) &= ~pinValue;
}

//*****************************************************************************
//
//! Enables the connection of GPIN to EtherCATSS through pipelined register as
//! against the direct from IO.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param pipelineMask is the mask to enable connection is through pipelined
//!        register.
//!
//! This function enables that the connection is through the pipelined
//! register which is captured on programmed event.
//!
//! The \e pipelineMask can have the respective GPIN Defines OR'ed to select
//! which PIN to write to. The defines are ESCSS_GPIN_x where x is from 0 to
//! 31.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_enableGPIPipelinedRegCaptureOnEvent(uint32_t base, uint32_t pipelineMask)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Set the value for the GPIN Pipeline Select.
    //
    HWREG(base + ESCSS_O_GPIN_PIPE) |= pipelineMask;
}

//*****************************************************************************
//
//! Disables the connection of GPIN to EtherCATSS through pipelined register
//! and is direct from the IO Pad.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param pipelineMask is the mask to enable connection is through pipelined
//!        register.
//!
//! This function disables the connection through the pipelined register
//! and connection is directly from the IO pad.
//!
//! The \e pipelineMask can have the respective GPIN Defines OR'ed to select
//! which PIN to write to. The defines are ESCSS_GPIN_x where x is from 0 to
//! 31.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_disableGPIPipelinedRegCaptureOnEvent(uint32_t base,
                                           uint32_t pipelineMask)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Reset the value for the GPIN Pipeline Select.
    //
    HWREG(base + ESCSS_O_GPIN_PIPE) &= ~pipelineMask;
}

//*****************************************************************************
//
//! Reads the GPOUT Data.
//!
//! \param base is the base address of EtherCAT Sub-System.
//!
//! This function returns the GPOUT Data. Local GPOUT data register which is
//! synchronized version on SysClk, each bit is represents GPOUT IORead is
//! allowed for CPU to process (IO extender or so if required).
//!
//! \return Returns the GPOUT Data Value.
//
//*****************************************************************************
static inline uint32_t
ESCSS_readGPOUTData(uint32_t base)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Return the GPOUT Data Read from the register.
    //
    return(HWREG(base + ESCSS_O_GPOUT_DAT));
}

//*****************************************************************************
//
//! Enables the connection of EtherCATSS GPOUT output to the IO pad through
//! pipelined register as against the direct connection.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param pipelineMask is the mask to enable connection is through pipelined
//!        register.
//!
//! This function enables the connection to be through the pipelined register
//! which captures EtherCATSS o/p on programmed event.
//!
//! The \e pipelineMask can have the respective GPOUT Defines OR'ed to select
//! which PIN to write to. The defines are ESCSS_GPOUT_x where x is from 0 to
//! 31.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_enableGPOUTPipelinedRegCaptureOnEvent(uint32_t base,
                                            uint32_t pipelineMask)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Set the GPOUT pipelined register value.
    //
    HWREG(base + ESCSS_O_GPOUT_PIPE) |= pipelineMask;
}

//*****************************************************************************
//
//! Disables the connection of GPOUT to EtherCATSS through pipelined register
//! and is direct from the IO Pad.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param pipelineMask is the mask to enable connection is through pipelined
//!        register.
//!
//! This function disables the connection through the pipelined register
//! and connection is directly from the IO pad.
//!
//! The \e pipelineMask can have the respective GPOUT Defines OR'ed to select
//! which PIN to write to. The defines are ESCSS_GPOUT_x where x is from 0 to
//! 31.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_disableGPOUTPipelinedRegCaptureOnEvent(uint32_t base,
                                             uint32_t pipelineMask)
{
    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Reset the GPOUT pipelined register value.
    //
    HWREG(base + ESCSS_O_GPOUT_PIPE) &= ~pipelineMask;
}

//*****************************************************************************
//
//! Initializes the Memory Init.
//!
//! \param base is the base address of EtherCAT Sub-System.
//!
//! This function starts the initialization when set.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ESCSS_initMemory(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(ESCSS_isBaseValid(base));

    //
    // Trigger the memory initialization.
    //
    HWREGH(base + ESCSS_O_MEM_TEST) |= ESCSS_MEM_TEST_INITIATE_MEM_INIT;
}

//*****************************************************************************
//
//! Returns the memory init status.
//!
//! \param base is the base address of EtherCAT Sub-System.
//!
//! This function returns the status of the memory initialization.
//!
//! \return Returns \b true indicating memory initialization completion, and
//! \b false if not ye completed.
//!
//
//*****************************************************************************
static inline bool
ESCSS_getMemoryInitDoneStatusNonBlocking(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(ESCSS_isBaseValid(base));

    bool memoryInitStatus;

    //
    // Get the memory init Status.
    //
    if((HWREG(base + ESCSS_O_MEM_TEST) & ESCSS_MEM_TEST_MEM_INIT_DONE) ==
       ESCSS_MEM_TEST_MEM_INIT_DONE)
        memoryInitStatus = (bool)true;
    else
        memoryInitStatus = (bool)false;

    return(memoryInitStatus);
}

//*****************************************************************************
//
//! Waits for the memory init status to be set till the loop count.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param loopCount is the count till which the loop will run till the Memory
//!        init is done.
//!
//! This function returns the status of the memory initialization after waiting
//! till a loop count.
//!
//! \return Returns \b true indicating memory initialization completion, and
//! \b false if not ye completed.
//!
//
//*****************************************************************************
static inline uint32_t
ESCSS_getMemoryInitDoneStatusBlocking(uint32_t base, uint32_t loopCount)
{
    uint32_t count = 0U;

    //
    // Check the arguments.
    //
    ASSERT(ESCSS_isBaseValid(base));

    uint32_t apiStatus = ESCSS_API_FAIL;

    while(count < loopCount)
    {
        //
        // Get the memory init Status.
        //
        if((HWREG(base + ESCSS_O_MEM_TEST) & ESCSS_MEM_TEST_MEM_INIT_DONE) ==
         ESCSS_MEM_TEST_MEM_INIT_DONE)
        {
            //
            // Memory Init Successful.
            //
            apiStatus = ESCSS_API_SUCCESS;
            break;
        }
    else
        {
            apiStatus = ESCSS_API_FAIL;
        }

        count++;
    }

    return(apiStatus);
}

//*****************************************************************************
//
//! Checks if lock configuration is done.
//!
//! \param base is the base address of EtherCAT Sub-System.
//!
//! This function checks if the lock configuration is done and returns the
//! status.
//!
//! \return None.
//
//*****************************************************************************
static inline bool
ESCSS_isConfigurationLockEnabled(uint32_t base)
{
    bool configStatus;

    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isConfigBaseValid(base));

    //
    // Check if lock configuration is enabled.
    //
    if((HWREG(base + ESCSS_O_CONFIG_LOCK) & ESCSS_CONFIG_LOCK_LOCK_ENABLE) ==
       ESCSS_CONFIG_LOCK_LOCK_ENABLE)
    {
        configStatus = (bool)true;
    }
    else
    {
        configStatus = (bool)false;
    }

    return(configStatus);
}

//*****************************************************************************
//
//! Enables the selection of LED o/p connect to IO Pad.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param ledSelectConfig is the Mux Selection Value.
//!
//! This function enables the selection of LED o/p connect to IO Pad. This
//! selection assumes both buffer input and buffer enable connection as
//! required.
//!
//! The LED Configuration Mask parameter can have the following valid options
//! for \e ledSelectConfig can be OR'ed together:
//! - \b ESCSS_LED_CONFIG_LINKACT0 - GPIO enable for LINKACT0 LED,
//! - \b ESCSS_LED_CONFIG_LINKACT1 - GPIO enable for LINKACT1 LED,
//! - \b ESCSS_LED_CONFIG_STATE - GPIO enable for STATE LED,
//! - \b ESCSS_LED_CONFIG_ERR - GPIO enable for ERR LED,
//! - \b ESCSS_LED_CONFIG_RUN - GPIO enable for RUN LED.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
static inline uint16_t
ESCSS_enableLEDOptions(uint32_t base, uint16_t ledSelectConfig)
{
    uint16_t apiStatus;

    //
    // Check the arguments.
    //
    ASSERT(ESCSS_isConfigBaseValid(base));
    ASSERT((ledSelectConfig & ~(ESCSS_LED_CONFIG_LINKACT0 |
                                ESCSS_LED_CONFIG_LINKACT1 |
                                ESCSS_LED_CONFIG_STATE |
                                ESCSS_LED_CONFIG_ERR |
                                ESCSS_LED_CONFIG_RUN)) == 0U);

    if((bool)true != ESCSS_isConfigurationLockEnabled(ESC_SS_CONFIG_BASE))
    {
        //
        // Set the LED Options.
        //
        HWREGH(base + ESCSS_O_LED_CONFIG) |= ledSelectConfig;
        apiStatus = ESCSS_API_SUCCESS;
    }
    else
    {
        //
        // Return API Fail if Configuration Lock is Enabled.
        //
        apiStatus = ESCSS_API_FAIL;
    }

    return(apiStatus);
}

//*****************************************************************************
//
//! Disables the selection of LED o/p connect to IO Pad.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param ledSelectConfig is the Mux Selection Value.
//!
//! This function disables the selection of LED o/p connect to IO Pad. The
//! non-EtherCAT function is selected on the IO.
//!
//! The LED Configuration Mask parameter can have the following valid options
//! for \e ledSelectConfig can be OR'ed together:
//! - \b ESCSS_LED_CONFIG_LINKACT0,
//! - \b ESCSS_LED_CONFIG_LINKACT1,
//! - \b ESCSS_LED_CONFIG_STATE,
//! - \b ESCSS_LED_CONFIG_ERR,
//! - \b ESCSS_LED_CONFIG_RUN.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
static inline uint16_t
ESCSS_disableLEDOptions(uint32_t base, uint16_t ledSelectConfig)
{
    uint16_t apiStatus;

    //
    // Check the arguments.
    //
    ASSERT(ESCSS_isConfigBaseValid(base));
    ASSERT((ledSelectConfig & ~(ESCSS_LED_CONFIG_LINKACT0 |
                                ESCSS_LED_CONFIG_LINKACT1 |
                                ESCSS_LED_CONFIG_STATE |
                                ESCSS_LED_CONFIG_ERR |
                                ESCSS_LED_CONFIG_RUN)) == 0U);

    if((bool)true != ESCSS_isConfigurationLockEnabled(ESC_SS_CONFIG_BASE))
    {
        //
        // Disable the LED Options.
        //
        HWREGH(base + ESCSS_O_LED_CONFIG) &= ~ledSelectConfig;
        apiStatus = ESCSS_API_SUCCESS;
    }
    else
    {
        //
        // Return API Fail if Configuration Lock is Enabled.
        //
        apiStatus = ESCSS_API_FAIL;
    }

    return(apiStatus);
}

//*****************************************************************************
//
//! Enable for GPIN Connection to IO pad.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param selectValue is the mask for the mux select.
//!
//! This function allows bit-wise selection of the GPIN  be connected from
//! GPIO PAD. Once those are not driven by GPIO, will be driven from register
//! writable from local Host.
//!
//! The \e selectValue can have the respective GPIN Defines OR'ed to select
//! which PIN to enable. The defines are ESCSS_GPIN_x where x is from 0 to
//! 31.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
static inline uint16_t
ESCSS_enableGPIN(uint32_t base, uint32_t selectValue)
{
    uint16_t apiStatus;

    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isConfigBaseValid(base));

    if((bool)true != ESCSS_isConfigurationLockEnabled(ESC_SS_CONFIG_BASE))
    {
        //
        // Mux Select the GPIN from the dedicated IO PAD.
        //
        HWREG(base + ESCSS_O_GPIN_SEL) |= selectValue;
        apiStatus = ESCSS_API_SUCCESS;
    }
    else
    {
        //
        // Return API Fail if Configuration Lock is Enabled.
        //
        apiStatus = ESCSS_API_FAIL;
    }

    return(apiStatus);
}

//*****************************************************************************
//
//! Disable for GPIN Connection to IO pad.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param selectValue is the mask for the mux select.
//!
//! This function disables connection to GPIO PAD, but connects to
//! ESCSS_GPIN_DAT.
//!
//! The \e selectValue can have the respective GPIN Defines OR'ed to select
//! which GPIN to disable. The defines are ESCSS_GPIN_x where x is from 0 to
//! 31.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
static inline uint16_t
ESCSS_disableGPIN(uint32_t base, uint32_t selectValue)
{
    uint16_t apiStatus;

    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isConfigBaseValid(base));

    if((bool)true != ESCSS_isConfigurationLockEnabled(ESC_SS_CONFIG_BASE))
    {
        //
        // Disables connection to GPIO PADMux.
        //
        HWREG(base + ESCSS_O_GPIN_SEL) &= ~selectValue;
        apiStatus = ESCSS_API_SUCCESS;
    }
    else
    {
        //
        // Return API Fail if Configuration Lock is Enabled.
        //
        apiStatus = ESCSS_API_FAIL;
    }

    return(apiStatus);
}

//*****************************************************************************
//
//! Enable GPOUT selection between register or GPIO pad.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param selectValue is the mask for the mux select.
//!
//! This function is to allow the EtherCAT GPOUT output be driven to either of
//! the two IO-pads allocated.
//!
//! The \e selectValue can have the respective GPOUT Defines OR'ed to select
//! which GPOUT to enable. The defines are ESCSS_GPOUT_x where x is from 0 to
//! 31.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
static inline uint16_t
ESCSS_enableGPOUT(uint32_t base, uint32_t selectValue)
{
    uint16_t apiStatus;

    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isConfigBaseValid(base));

    if((bool)true != ESCSS_isConfigurationLockEnabled(ESC_SS_CONFIG_BASE))
    {
        //
        // Mux Select the GPOUT.
        //
        HWREG(base + ESCSS_O_GPOUT_SEL) |= selectValue;
        apiStatus = ESCSS_API_SUCCESS;
    }
    else
    {
        //
        // Return API Fail if Configuration Lock is Enabled.
        //
        apiStatus = ESCSS_API_FAIL;
    }

    return(apiStatus);
}

//*****************************************************************************
//
//! Disable GPOUT selection between register or GPIO pad.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param selectValue is the mask for the mux select.
//!
//! This function is to disable the EtherCAT GPOUT output be driven to either
//! of the two IO-pads allocated.
//!
//! The \e selectValue can have the respective GPOUT Defines OR'ed to select
//! which GPOUT to disable. The defines are ESCSS_GPOUT_x where x is from 0 to
//! 31.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
static inline uint16_t
ESCSS_disableGPOUT(uint32_t base, uint32_t selectValue)
{
    uint16_t apiStatus;

    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isConfigBaseValid(base));

    if((bool)true != ESCSS_isConfigurationLockEnabled(ESC_SS_CONFIG_BASE))
    {
        //
        // Mux Select the GPOUT to disable.
        //
        HWREG(base + ESCSS_O_GPOUT_SEL) &= ~selectValue;
        apiStatus = ESCSS_API_SUCCESS;
    }
    else
    {
        //
        // Return API Fail if Configuration Lock is Enabled.
        //
        apiStatus = ESCSS_API_FAIL;
    }

    return(apiStatus);
}

//*****************************************************************************
//
//! Configure the TX for shift for Port 0.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param shiftValue is the shift in 10ns terms.
//!
//! This function is to configure the TX for shift for Port 0. Two bit TX_SHIFT
//! configuration in terms of 10ns counts for port0. This is the shift added
//! to TX_ENA & TX_DATA to match delay of PHY TX_CLK w.r.t. device internal
//! clock.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
static inline uint16_t
ESCSS_configureTX0ShiftForTxEnaAndTxData(uint32_t base, uint32_t shiftValue)
{
    uint16_t apiStatus;

    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isConfigBaseValid(base));

    if((bool)true != ESCSS_isConfigurationLockEnabled(ESC_SS_CONFIG_BASE))
    {
        //
        // Set the shift Value.
        //
        HWREGH(base + ESCSS_O_MISC_CONFIG) |=
        ((uint16_t)((uint32_t)shiftValue <<
         ESCSS_MISC_CONFIG_TX0_SHIFT_CONFIG_S)
         & ESCSS_MISC_CONFIG_TX0_SHIFT_CONFIG_M);

        apiStatus = ESCSS_API_SUCCESS;
    }
    else
    {
        //
        // Return API Fail if Configuration Lock is Enabled.
        //
        apiStatus = ESCSS_API_FAIL;
    }

    return(apiStatus);
}

//*****************************************************************************
//
//! Configure the TX for shift for Port 1.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param shiftValue is the shift in 10ns terms.
//!
//! This function is to configure the TX for shift for Port 1. Two bit TX_SHIFT
//! configuration in terms of 10ns counts for port1. This is the shift added
//! to TX_ENA & TX_DATA to match delay of PHY TX_CLK w.r.t. device internal
//! clock.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
static inline uint16_t
ESCSS_configureTX1ShiftForTxEnaAndTxData(uint32_t base, uint32_t shiftValue)
{
    uint16_t apiStatus;

    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isConfigBaseValid(base));

    if((bool)true != ESCSS_isConfigurationLockEnabled(ESC_SS_CONFIG_BASE))
    {
        //
        // Set the shift Value.
        //
        HWREGH(base + ESCSS_O_MISC_CONFIG) |=
        ((shiftValue << ESCSS_MISC_CONFIG_TX1_SHIFT_CONFIG_S) &
         ESCSS_MISC_CONFIG_TX1_SHIFT_CONFIG_M);

        apiStatus = ESCSS_API_SUCCESS;
    }
    else
    {
        //
        // Return API Fail if Configuration Lock is Enabled.
        //
        apiStatus = ESCSS_API_FAIL;
    }

    return(apiStatus);
}

//*****************************************************************************
//
//! Enable the PDI Emulation.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//!
//! This function will be hooked up to the PDI_EMULATION input of the
//! EtherCAT IP.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
static inline uint16_t
ESCSS_enablePDIEmulation(uint32_t base)
{
    uint16_t apiStatus;

    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isConfigBaseValid(base));

    if((bool)true != ESCSS_isConfigurationLockEnabled(ESC_SS_CONFIG_BASE))
    {
        //
        // Set the PDI Emulation.
        //
        HWREGH(base + ESCSS_O_MISC_CONFIG) |= ESCSS_MISC_CONFIG_PDI_EMULATION;
        apiStatus = ESCSS_API_SUCCESS;
    }
    else
    {
        //
        // Return API Fail if Configuration Lock is Enabled.
        //
        apiStatus = ESCSS_API_FAIL;
    }

    return(apiStatus);
}

//*****************************************************************************
//
//! Disable the PDI Emulation.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//!
//! This function will not allow hooking up to the PDI_EMULATION input of the
//! EtherCAT IP.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
static inline uint16_t
ESCSS_disablePDIEmulation(uint32_t base)
{
    uint16_t apiStatus;

    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isConfigBaseValid(base));

    if((bool)true != ESCSS_isConfigurationLockEnabled(ESC_SS_CONFIG_BASE))
    {
        //
        // Set the PDI Emulation.
        //
        HWREGH(base + ESCSS_O_MISC_CONFIG) &= ~ESCSS_MISC_CONFIG_PDI_EMULATION;
        apiStatus = ESCSS_API_SUCCESS;
    }
    else
    {
        //
        // Return API Fail if Configuration Lock is Enabled.
        //
        apiStatus = ESCSS_API_FAIL;
    }

    return(apiStatus);
}

//*****************************************************************************
//
//! Configure the Physical Address Offset.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param registerOffset is the register offset value.
//!
//! This function will set the PHY_OFFSET[4:0] input of the EtherCAT IP.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
static inline uint16_t
ESCSS_configurePhyAddressOffset(uint32_t base, uint16_t registerOffset)
{
    uint16_t apiStatus;

    //
    // Check if the passed parameter base is valid.
    //
    ASSERT(ESCSS_isConfigBaseValid(base));

    if((bool)true != ESCSS_isConfigurationLockEnabled(ESC_SS_CONFIG_BASE))
    {
        //
        // Set the Physical Address Offset.
        //
        HWREGH(base + ESCSS_O_MISC_CONFIG) |=
        ((uint16_t)((uint32_t)registerOffset << ESCSS_MISC_CONFIG_PHY_ADDR_S)
         & ESCSS_MISC_CONFIG_PHY_ADDR_M);
        apiStatus = ESCSS_API_SUCCESS;
    }
    else
    {
        //
        // Return API Fail if Configuration Lock is Enabled.
        //
        apiStatus = ESCSS_API_FAIL;
    }

    return(apiStatus);
}

//*****************************************************************************
//
//! Sets the Raw Interrupt Cause Status.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param interruptMask is the mask value to select the different type of
//!        interrupt.
//! \param writeKey is the key Value which decides if the writes to this
//!        register are to take affect.
//!
//! This function sets the individual interrupt cause based on the mask value
//! passed to emulate the interrupt cause.
//!
//! The interrupt Status Mask parameter can have the following valid options
//! for \e interruptMask can be OR'ed together:
//!  \b ESCSS_INTR_SET_SYNC0_SET, \b ESCSS_INTR_SET_SYNC1_SET,
//!  \b ESCSS_INTR_SET_IRQ_SET, \b ESCSS_INTR_SET_DMA_DONE_SET,
//!  \b ESCSS_INTR_SET_TIMEOUT_ERR_SET, \b ESCSS_INTR_SET_MASTER_RESET_SET.
//!
//! \return Returns \b ESCSS_API_SUCCESS  if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_setRawInterruptStatus(uint32_t base, uint16_t interruptMask,
                            uint16_t writeKey);

//*****************************************************************************
//
//! Sets the GPIN pipe group capture trigger.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param selectGPIN is to select the trigger to capture the IO input in
//!        pipeline register for selective GPIN.
//! \param triggerCapSelect is the GPIN Data to be written.
//!
//! This function selects the trigger to capture the IO input in pipeline
//! register for GPIN.
//! The \e selectGPIN parameter takes the following parameters:
//! \b ESCSS_GROUP_CAPTURE_SELECT0, \b ESCSS_GROUP_CAPTURE_SELECT1,
//! \b ESCSS_GROUP_CAPTURE_SELECT2 and \b ESCSS_GROUP_CAPTURE_SELECT3.
//! The \e ESCSS_CaptureTrigger parameter takes the values:
//! \b ESCSS_SOF_CAPTURE_TRIGGER, \b ESCSS_SYNC0_CAPTURE_TRIGGER, \b
//! ESCSS_SYNC1_CAPTURE_TRIGGER, \b ESCSS_LATCH0_CAPTURE_TRIGGER, \b
//! ESCSS_LATCH1_CAPTURE_TRIGGER.
//!
//! \return None.
//
//*****************************************************************************
extern void
ESCSS_setGPINGroupCaptureTriggerSelect(uint32_t base,
                                       ESCSS_GroupCaptureSelect selectGPIN,
                                       ESCSS_CaptureTrigger triggerCapSelect);

//*****************************************************************************
//
//! GPOUT pipe group capture trigger.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param selectGPOUT is to select the trigger to capture the IO input in
//!        pipeline register for selective GPIN.
//! \param triggerCapSelect is the GPIN Data to be written.
//!
//! This function selects the trigger to capture the IO input in pipeline
//! register for GPOUT.
//! The \e selectGPOUT parameter takes the following parameters:
//! \b ESCSS_GROUP_CAPTURE_SELECT0, \b ESCSS_GROUP_CAPTURE_SELECT1,
//! \b ESCSS_GROUP_CAPTURE_SELECT2 and \b ESCSS_GROUP_CAPTURE_SELECT3.
//! The \e ESCSS_CaptureTrigger parameter takes the values:
//! \b ESCSS_SOF_CAPTURE_TRIGGER, \b ESCSS_SYNC0_CAPTURE_TRIGGER, \b
//! ESCSS_SYNC1_CAPTURE_TRIGGER, \b ESCSS_LATCH0_CAPTURE_TRIGGER, \b
//! ESCSS_LATCH1_CAPTURE_TRIGGER.
//!
//! \return None.
//
//*****************************************************************************
extern void
ESCSS_setGPOUTGroupCaptureTriggerSelect(uint32_t base,
                                        ESCSS_GroupCaptureSelect selectGPOUT,
                                        ESCSS_CaptureTrigger triggerCapSelect);

//*****************************************************************************
//
//! Enables EtherCAT Reset drives to EtherCAT IP and PHY Reset.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param writeKey is the key Value which decides if the writes to this
//!        register are to take affect.
//!
//! This function enables the EtherCAT Reset to drive the IP & PHY reset.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_enableCPUReset(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! EtherCAT Reset does not drive to EtherCAT IP and PHY Reset.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param writeKey is the key Value which decides if the writes to this
//!        register are to take affect.
//!
//! This function configures the EtherCAT Reset not to drive to EtherCAT IP
//! and PHY Reset. The application shall configure NMI/Interrupt to eventually
//! complete the reset through system control soft reset.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_disableCPUReset(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! Enables for reset out to drive the CPU NMI.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param writeKey is the key Value which decides if the writes to this
//!        register are to take affect.
//!
//! This function configures the EtherCAT Reset to drive the CPU NMI. IP Reset
//! out drives CPU NMI to which it belongs. NMI handler is expected to complete
//! the required tasks or context save if any and then reset the EtherCAT
//! through the system control soft reset.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_enableResetToNMI(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! Disables for reset out to drive the CPU NMI
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param writeKey is the key Value which decides if the writes to this
//!        register are to take affect.
//!
//! This function configures the EtherCAT Reset not to drive the CPU NMI.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_disableResetToNMI(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! Enables for reset out to drive the interrupt to CPU which it belongs to.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param writeKey is the key Value which decides if the writes to this
//!        register are to take affect.
//!
//! This function enables the EtherCAT Reset to drive the interrupt to CPU
//! which it belongs to.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_enableResetToInterrupt(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! Disables for reset out to drive the interrupt to CPU which it belongs to.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param writeKey is the key Value which decides if the writes to this
//!        register are to take affect.
//!
//! This function configures the EtherCAT Reset not to drive the interrupt to
//! CPU which it belongs to.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_disableResetToInterrupt(uint32_t base, uint16_t writeKey);


//*****************************************************************************
//
//! Configures the EtherCAT connection from SYNC0 to a particular interrupt.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param connectionInterrupt is the interrupt to which SYNC0 output is
//!        connected to.
//! \param writeKey is the key Value which decides if the writes to this
//!        register are to take affect.
//!
//! This function configures the connection from SYNC0 to the interrupt to the
//! respective selection made. The following valid options for
//! \e connectionInterrupt are:
//! - \b ESCSS_SYNC0_CONFIG_C28X_PIE_EN - Connects the SYNC0 to C28x
//!                                              PIE Interrupt
//! - \b ESCSS_SYNC0_CONFIG_CLA_INT_EN  - Connects the SYNC0 to CLA
//!                                              Interrupt
//! - \b ESCSS_SYNC0_CONFIG_C28X_DMA_EN - Connects the SYNC0 to C28x
//!                                              DMA Trigger
//! - \b ESCSS_SYNC0_CONFIG_CM4_NVIC_EN - Connects the SYNC0 to CM4
//!                                              NVIC Interrupt
//! - \b ESCSS_SYNC0_CONFIG_UDMA_TRIG_EN - Connects the SYNC0 to uDMA
//!                                               Trigger
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_configureSync0Connections(uint32_t base, uint16_t connectionInterrupt,
                                uint16_t writeKey);

//*****************************************************************************
//
//! Configures the EtherCAT connection from SYNC1 to a particular interrupt.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param connectionInterrupt is the interrupt to which SYNC1 output is
//!        connected to.
//! \param writeKey is the key Value which decides if the writes to this
//!        register are to take affect.
//!
//! This function configures the connection from SYNC1 to the interrupt to the
//! respective selection made. The following valid options for
//! \e connectionInterrupt are:
//! - \b ESCSS_SYNC1_CONFIG_C28X_PIE_EN - Connects the SYNC1 to C28x
//!                                              PIE Interrupt
//! - \b ESCSS_SYNC1_CONFIG_CLA_INT_EN  - Connects the SYNC1 to CLA
//!                                              Interrupt
//! - \b ESCSS_SYNC1_CONFIG_C28X_DMA_EN - Connects the SYNC1 to C28x
//!                                              DMA Trigger
//! - \b ESCSS_SYNC1_CONFIG_CM4_NVIC_EN - Connects the SYNC1 to CM4
//!                                              NVIC Interrupt
//! - \b ESCSS_SYNC1_CONFIG_UDMA_TRIG_EN - Connects the SYNC1 to uDMA
//!                                               Trigger
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_configureSync1Connections(uint32_t base, uint16_t connectionInterrupt,
                                uint16_t writeKey);

//*****************************************************************************
//
//! Enables the configuration lock.
//!
//! \param base is the base address of EtherCAT Sub-System.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This bit enables locking the contents of all the EtherCAT configuration
//! registers. This bit can be set only once after ecatXRSN and gets reset
//! after the next ecatXRSN.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_enableConfigurationLock(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! Locking the IO Configuration.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This function leads to locking of the IO Configurations. Changing IO
//! selections or IO configurations after this bit is set can have
//! unpredictable IO behavior on the device IOs.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_enableIOConnectionLock(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! Disable Lock on the IO Configuration.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This function leads to disabling of the locking of the IO Configurations.
//! EtherCAT ports are not connected to the IO pad.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_disableIOConnectionLock(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! Enabled ResetIN from GPIO.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This function enables the RESET_IN GPIO pad input to be  connected in reset
//! input cone.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_enableResetInputFromGpioPad(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! Disable ResetIN from GPIO.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This function leads to RESET_IN GPIO pad to not be enabled, only SW & PMM
//! resets affect EtherCAT reset.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_disableResetInputFromGpioPad(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! Enables the EEPROM I2C IOPAD connection.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This function leads to EEPROM I2C connections driving the IOPAD
//! connections.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_enableESCEEPROMI2CIoPadConnection(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! Disable the EEPROM I2C IOPAD connection.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This function leads to EEPROM I2C Connections not connected to IOPAD.
//!
//! \return Returns \b ESCSS_API_SUCCESS if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_disableESCEEPROMI2CIoPadConnection(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! Number of PHY port counts.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param portConfig indicating the number of PHY Ports to be selected.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! Indicates the number of PHY ports selected for operation in addition to
//! Port0 which is default. The various values for the selection of
//! \e portConfig are:
//! \b ESCSS_ONE_PORT_SELECTION, \b ESCSS_TWO_PORT_SELECTION.
//!
//! \return Returns \b ESCSS_API_SUCCESS  if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_configurePortCount(uint32_t base, ESCSS_PortSelection portConfig,
                         uint16_t writeKey);

//*****************************************************************************
//
//! Auto Compensation based on sampling of TX_CLK.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This setting is used to allocate the IO pad for TX_CLK for doing the Auto
//! compensation for the sampling of TXEN & TXDATA.
//!
//! \return Returns \b ESCSS_API_SUCCESS  if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_enableAutoCompensationTxClkIOPad(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! Manual Compensation is there using CLK_IN.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This function sets the manual compensation using CLK_IN no TX_CLK Pad,
//! IP input is tied to '0'.
//!
//! \return Returns \b ESCSS_API_SUCCESS  if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_disableAutoCompensationTxClkIOPad(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! SYNC0 connection to OUT pad enabled.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This function enables the direct mux between Sync0 output of EtherCAT and
//! other GPIO functions.
//!
//! \return Returns \b ESCSS_API_SUCCESS  if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_enableSync0GpioMuxConnection(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! SYNC0 connection to OUT pad disabled.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This function does not enables the direct mux between Sync0 output of
//! EtherCAT and other GPIO functions.
//!
//! \return Returns \b ESCSS_API_SUCCESS  if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_disableSync0GpioMuxConnection(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! SYNC1 connection to OUT pad enabled.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This function enables the direct mux between Sync1 output of EtherCAT and
//! other GPIO functions.
//!
//! \return Returns \b ESCSS_API_SUCCESS  if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_enableSync1GpioMuxConnection(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! SYNC1 connection to OUT pad disabled.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This function does not enables the direct mux between Sync1 output of
//! EtherCAT and other GPIO functions.
//!
//! \return Returns \b ESCSS_API_SUCCESS  if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_disableSync1GpioMuxConnection(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! LATCH0 connection to IN pad enabled.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This function enables the direct mux between LATCH0 input from IOPAD and
//! other GPIO functions to the EtherCATSS input.
//!
//! \return Returns \b ESCSS_API_SUCCESS  if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_enableLatch0GpioMuxConnection(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! LATCH0 connection to IN pad disabled.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This function does not enable the direct mux between LATCH0 input from
//! IOPAD and other GPIO functions to the EtherCATSS input.
//!
//! \return Returns \b ESCSS_API_SUCCESS  if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_disableLatch0GpioMuxConnection(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! LATCH1 connection to IN pad enabled.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This function enables the direct mux between LATCH1 input from IOPAD and
//! other GPIO functions to the EtherCATSS input.
//!
//! \return Returns \b ESCSS_API_SUCCESS  if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_enableLatch1GpioMuxConnection(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! LATCH1 connection to IN pad disabled.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param writeKey is the key Value to enable the writing lock.
//!
//! This function does not enable the direct mux between LATCH1 input from
//! IOPAD and other GPIO functions to the EtherCATSS input.
//!
//! \return Returns \b ESCSS_API_SUCCESS  if the Register Access is successful,
//! \b ESCSS_API_FAIL if access to the Register Fails.
//
//*****************************************************************************
extern uint16_t
ESCSS_disableLatch1GpioMuxConnection(uint32_t base, uint16_t writeKey);

//*****************************************************************************
//
//! Configure EEPROM Size bound as per the select.
//!
//! \param base is the base address of EtherCAT configuration Registers.
//! \param eepromSize is the shift in 10ns terms.
//!
//! This function set the bit which will be hooked up to the EEPROM_SIZE input
//! of the EtherCAT IP . This is set to 0 for EEPROMs of size 16K bits or lower
//! and set to 1 for EEPROMs of size above 16K bits.
//!
//! \return None.
//
//*****************************************************************************
extern void
ESCSS_configureEEPROMSize(uint32_t base, ESCSS_SizeSelect eepromSize);

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

#endif //  ESCSS_H
