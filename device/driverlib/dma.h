//###########################################################################
//
// FILE:   dma.h
//
// TITLE:  C28x DMA driver.
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

#ifndef DMA_H
#define DMA_H

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
//! \addtogroup dma_api DMA
//! \brief This module is used for DMA configurations.
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_dma.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "cpu.h"
#include "debug.h"

//*****************************************************************************
//
// Values that can be passed to DMA_configMode() as the config parameter.
//
//*****************************************************************************
//! Only one burst transfer performed per trigger.
#define DMA_CFG_ONESHOT_DISABLE     0U
//! Burst transfers occur without additional event triggers after the first.
#define DMA_CFG_ONESHOT_ENABLE      DMA_MODE_ONESHOT

//! DMA channel will be disabled at the end of a transfer.
#define DMA_CFG_CONTINUOUS_DISABLE  0U
//! DMA reinitializes when the transfer count is zero and waits for a trigger.
#define DMA_CFG_CONTINUOUS_ENABLE   DMA_MODE_CONTINUOUS

//! DMA transfers 16 bits at a time.
#define DMA_CFG_SIZE_16BIT          0U
//! DMA transfers 32 bits at a time.
#define DMA_CFG_SIZE_32BIT          DMA_MODE_DATASIZE

//*****************************************************************************
//
//! Values that can be passed to DMA_configMode() as the \e trigger parameter.
//
//*****************************************************************************
typedef enum
{
    DMA_TRIGGER_SOFTWARE     = 0,

    DMA_TRIGGER_ADCA1        = 1,
    DMA_TRIGGER_ADCA2        = 2,
    DMA_TRIGGER_ADCA3        = 3,
    DMA_TRIGGER_ADCA4        = 4,
    DMA_TRIGGER_ADCAEVT      = 5,
    DMA_TRIGGER_ADCB1        = 6,
    DMA_TRIGGER_ADCB2        = 7,
    DMA_TRIGGER_ADCB3        = 8,
    DMA_TRIGGER_ADCB4        = 9,
    DMA_TRIGGER_ADCBEVT      = 10,
    DMA_TRIGGER_ADCC1        = 11,
    DMA_TRIGGER_ADCC2        = 12,
    DMA_TRIGGER_ADCC3        = 13,
    DMA_TRIGGER_ADCC4        = 14,
    DMA_TRIGGER_ADCCEVT      = 15,
    DMA_TRIGGER_ADCD1        = 16,
    DMA_TRIGGER_ADCD2        = 17,
    DMA_TRIGGER_ADCD3        = 18,
    DMA_TRIGGER_ADCD4        = 19,
    DMA_TRIGGER_ADCDEVT      = 20,

    DMA_TRIGGER_XINT1        = 29,
    DMA_TRIGGER_XINT2        = 30,
    DMA_TRIGGER_XINT3        = 31,
    DMA_TRIGGER_XINT4        = 32,
    DMA_TRIGGER_XINT5        = 33,

    DMA_TRIGGER_EPWM1SOCA    = 36,
    DMA_TRIGGER_EPWM1SOCB    = 37,
    DMA_TRIGGER_EPWM2SOCA    = 38,
    DMA_TRIGGER_EPWM2SOCB    = 39,
    DMA_TRIGGER_EPWM3SOCA    = 40,
    DMA_TRIGGER_EPWM3SOCB    = 41,
    DMA_TRIGGER_EPWM4SOCA    = 42,
    DMA_TRIGGER_EPWM4SOCB    = 43,
    DMA_TRIGGER_EPWM5SOCA    = 44,
    DMA_TRIGGER_EPWM5SOCB    = 45,
    DMA_TRIGGER_EPWM6SOCA    = 46,
    DMA_TRIGGER_EPWM6SOCB    = 47,
    DMA_TRIGGER_EPWM7SOCA    = 48,
    DMA_TRIGGER_EPWM7SOCB    = 49,
    DMA_TRIGGER_EPWM8SOCA    = 50,
    DMA_TRIGGER_EPWM8SOCB    = 51,
    DMA_TRIGGER_EPWM9SOCA    = 52,
    DMA_TRIGGER_EPWM9SOCB    = 53,
    DMA_TRIGGER_EPWM10SOCA   = 54,
    DMA_TRIGGER_EPWM10SOCB   = 55,
    DMA_TRIGGER_EPWM11SOCA   = 56,
    DMA_TRIGGER_EPWM11SOCB   = 57,
    DMA_TRIGGER_EPWM12SOCA   = 58,
    DMA_TRIGGER_EPWM12SOCB   = 59,
    DMA_TRIGGER_EPWM13SOCA   = 60,
    DMA_TRIGGER_EPWM13SOCB   = 61,
    DMA_TRIGGER_EPWM14SOCA   = 62,
    DMA_TRIGGER_EPWM14SOCB   = 63,
    DMA_TRIGGER_EPWM15SOCA   = 64,
    DMA_TRIGGER_EPWM15SOCB   = 65,
    DMA_TRIGGER_EPWM16SOCA   = 66,
    DMA_TRIGGER_EPWM16SOCB   = 67,

    DMA_TRIGGER_TINT0        = 68,
    DMA_TRIGGER_TINT1        = 69,
    DMA_TRIGGER_TINT2        = 70,

    DMA_TRIGGER_MCBSPAMXEVT  = 71,
    DMA_TRIGGER_MCBSPAMREVT  = 72,
    DMA_TRIGGER_MCBSPBMXEVT  = 73,
    DMA_TRIGGER_MCBSPBMREVT  = 74,

    DMA_TRIGGER_ECAP1        = 75,
    DMA_TRIGGER_ECAP2        = 76,
    DMA_TRIGGER_ECAP3        = 77,
    DMA_TRIGGER_ECAP4        = 78,
    DMA_TRIGGER_ECAP5        = 79,
    DMA_TRIGGER_ECAP6        = 80,
    DMA_TRIGGER_ECAP7        = 81,

    DMA_TRIGGER_SDFM1FLT1    = 95,
    DMA_TRIGGER_SDFM1FLT2    = 96,
    DMA_TRIGGER_SDFM1FLT3    = 97,
    DMA_TRIGGER_SDFM1FLT4    = 98,

    DMA_TRIGGER_SDFM2FLT1    = 99,
    DMA_TRIGGER_SDFM2FLT2    = 100,
    DMA_TRIGGER_SDFM2FLT3    = 101,
    DMA_TRIGGER_SDFM2FLT4    = 102,

    DMA_TRIGGER_SYNC         = 103,

    DMA_TRIGGER_SPIATX       = 109,
    DMA_TRIGGER_SPIARX       = 110,
    DMA_TRIGGER_SPIBTX       = 111,
    DMA_TRIGGER_SPIBRX       = 112,
    DMA_TRIGGER_SPICTX       = 113,
    DMA_TRIGGER_SPICRX       = 114,
    DMA_TRIGGER_SPIDTX       = 115,
    DMA_TRIGGER_SPIDRX       = 116,

    DMA_TRIGGER_CLB5INT      = 117,
    DMA_TRIGGER_CLB6INT      = 118,
    DMA_TRIGGER_CLB7INT      = 119,
    DMA_TRIGGER_CLB8INT      = 120,

    DMA_TRIGGER_FSITXA       = 123,
    DMA_TRIGGER_FSIRXA       = 125,

    DMA_TRIGGER_CLB1INT      = 127,
    DMA_TRIGGER_CLB2INT      = 128,
    DMA_TRIGGER_CLB3INT      = 129,
    DMA_TRIGGER_CLB4INT      = 130,

    DMA_TRIGGER_USBA_RX1     = 131,
    DMA_TRIGGER_USBA_TX1     = 132,
    DMA_TRIGGER_USBA_RX2     = 133,
    DMA_TRIGGER_USBA_TX2     = 134,
    DMA_TRIGGER_USBA_RX3     = 135,
    DMA_TRIGGER_USBA_TX3     = 136,

    DMA_TRIGGER_FSIRXC       = 143,
    DMA_TRIGGER_FSIRXD       = 144,
    DMA_TRIGGER_FSIRXE       = 145,
    DMA_TRIGGER_FSIRXF       = 146,
    DMA_TRIGGER_FSIRXG       = 147,
    DMA_TRIGGER_FSIRXH       = 148,

    DMA_TRIGGER_FSITXB       = 155,
    DMA_TRIGGER_FSIRXB       = 157,

    DMA_TRIGGER_CANAIF1      = 167,
    DMA_TRIGGER_CANAIF2      = 168,
    DMA_TRIGGER_CANAIF3      = 169,

    DMA_TRIGGER_CANBIF1      = 170,
    DMA_TRIGGER_CANBIF2      = 171,
    DMA_TRIGGER_CANBIF3      = 172
} DMA_Trigger;

//*****************************************************************************
//
//! Values that can be passed to DMA_setInterruptMode() as the \e mode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! DMA interrupt is generated at the beginning of a transfer
    DMA_INT_AT_BEGINNING,
    //! DMA interrupt is generated at the end of a transfer
    DMA_INT_AT_END
} DMA_InterruptMode;

//*****************************************************************************
//
//! Values that can be passed to DMA_setEmulationMode() as the \e mode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! Transmission stops after current read-write access is completed
    DMA_EMULATION_STOP,
    //! Continue DMA operation regardless of emulation suspend
    DMA_EMULATION_FREE_RUN
} DMA_EmulationMode;

//*****************************************************************************
//
//! Values that can be passed to DMA_configChannel() as the
//! configure parameter.
//
//*****************************************************************************
typedef struct
{
    DMA_Trigger          transferTrigger;  //DMA transfer triggers
    DMA_InterruptMode    interruptMode;    //Channel interrupt mode
    bool                 enableInterrupt;  //Enable/Disable interrupt mode
    uint32_t             configSize;    //Data bus width (16 or 32 bits)
    uint32_t             transferMode;  //Burst transfer mode
    uint32_t             reinitMode;    //DMA reinitialization mode
    uint32_t             burstSize;     //Number of words transferred per burst
    uint32_t             transferSize;  //Number of bursts per transfer
    //! Number of bursts to be transferred before a wrap of the source address
    //! occurs.
    uint32_t             srcWrapSize;
    //! Number of bursts to be transferred before a wrap of the destination
    //! address occurs.
    uint32_t             destWrapSize;
    uint32_t             destAddr;    //destination address
    uint32_t             srcAddr;     //source address
    //! Amount to inc or dec the source address after each word of a burst
    int16_t              srcBurstStep;
    //! Amount to inc or dec the destination address after each word of a burst
    int16_t              destBurstStep;
    //! Amount to inc or dec the source address after each burst of a transfer
    int16_t              srcTransferStep;
    //! Amount to inc or dec the destination address after each burst of a
    //! transfer
    int16_t              destTransferStep;
    //! Amount to inc or dec the source address when the wrap occurs
    int16_t              srcWrapStep;
    //! Amount to inc or dec the destination address when the wrap occurs
    int16_t              destWrapStep;

} DMA_ConfigParams;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! \internal
//! Checks an DMA channel base address.
//!
//! \param base specifies the DMA channel base address.
//!
//! This function determines if a DMA channel base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
DMA_isBaseValid(uint32_t base)
{
    return((base == DMA_CH1_BASE) || (base == DMA_CH2_BASE) ||
           (base == DMA_CH3_BASE) || (base == DMA_CH4_BASE) ||
           (base == DMA_CH5_BASE) || (base == DMA_CH6_BASE));
}
#endif

//*****************************************************************************
//
//! Initializes the DMA controller to a known state.
//!
//! This function configures does a hard reset of the DMA controller in order
//! to put it into a known state. The function also sets the DMA to run free
//! during an emulation suspend (see the field DEBUGCTRL.FREE for more info).
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_initController(void)
{
    EALLOW;

    //
    // Set the hard reset bit. One NOP is required after HARDRESET.
    //
    HWREGH(DMA_BASE + DMA_O_CTRL) |= DMA_CTRL_HARDRESET;
    NOP;

    EDIS;
}

//*****************************************************************************
//
//! Channel Soft Reset
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function does a soft reset to place the channel into its default state
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_triggerSoftReset(uint32_t base)
{
    EALLOW;

    //
    // Set the soft reset bit. One NOP is required after SOFTRESET.
    //
    HWREGH(base + DMA_O_CONTROL) |= DMA_CONTROL_SOFTRESET;
    NOP;

    EDIS;
}

//*****************************************************************************
//
//! Sets DMA emulation mode.
//!
//! \param mode is the emulation mode to be selected.
//!
//! This function sets the behavior of the DMA operation when an emulation
//! suspend occurs. The \e mode parameter can be one of the following:
//!
//! - \b DMA_EMULATION_STOP - DMA runs until the current read-write access is
//!   completed.
//! - \b DMA_EMULATION_FREE_RUN - DMA operation continues regardless of a
//!   the suspend.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_setEmulationMode(DMA_EmulationMode mode)
{
    EALLOW;

    //
    // Set emulation mode
    //
    if(mode == DMA_EMULATION_STOP)
    {
        HWREGH(DMA_BASE + DMA_O_DEBUGCTRL) &= ~DMA_DEBUGCTRL_FREE;
    }
    else
    {
        HWREGH(DMA_BASE + DMA_O_DEBUGCTRL) |= DMA_DEBUGCTRL_FREE;
    }

    EDIS;
}

//*****************************************************************************
//
//! Enables peripherals to trigger a DMA transfer.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function enables the selected peripheral trigger to start a DMA
//! transfer on the specified channel.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_enableTrigger(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Set the peripheral interrupt trigger enable bit.
    //
    EALLOW;
    HWREGH(base + DMA_O_MODE) |= DMA_MODE_PERINTE;
    EDIS;
}

//*****************************************************************************
//
//! Disables peripherals from triggering a DMA transfer.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function disables the selected peripheral trigger from starting a DMA
//! transfer on the specified channel. This also disables the use of the
//! software force using the DMA_forceTrigger() API.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_disableTrigger(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Clear the peripheral interrupt trigger enable bit.
    //
    EALLOW;
    HWREGH(base + DMA_O_MODE) &= ~DMA_MODE_PERINTE;
    EDIS;
}

//*****************************************************************************
//
//! Force a peripheral trigger to a DMA channel.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function sets the peripheral trigger flag and if triggering a DMA
//! burst is enabled (see DMA_enableTrigger()), a DMA burst transfer will be
//! forced.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_forceTrigger(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Set the peripheral interrupt trigger force bit.
    //
    EALLOW;
    HWREGH(base + DMA_O_CONTROL) |= DMA_CONTROL_PERINTFRC;
    EDIS;
}

//*****************************************************************************
//
//! Clears a DMA channel's peripheral trigger flag.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function clears the peripheral trigger flag. Normally, you would use
//! this function when initializing the DMA for the first time. The flag is
//! cleared automatically when the DMA starts the first burst of a transfer.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_clearTriggerFlag(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Write a one to the clear bit to clear the peripheral trigger flag.
    //
    EALLOW;
    HWREGH(base + DMA_O_CONTROL) |= DMA_CONTROL_PERINTCLR;
    EDIS;
}

//*****************************************************************************
//
//! Gets the status of a DMA channel's Transfer Status Flag.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function returns \b true if the Transfer Status Flag is set, which
//! means a DMA transfer has begun.
//! This flag is cleared when TRANSFER_COUNT reaches zero, or when the
//! HARDRESET or SOFTRESET bit is set.
//!
//! \return Returns \b true if the Transfer Status Flag is set. Returns \b false
//! otherwise.
//
//*****************************************************************************
static inline bool
DMA_getTransferStatusFlag(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Read the Transfer Status Flag and return appropriately.
    //
    return((HWREGH(base + DMA_O_CONTROL) & DMA_CONTROL_TRANSFERSTS) != 0U);
}

//*****************************************************************************
//
//! Gets the status of a DMA channel's Burst Status Flag.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function returns \b true if the Burst Status Flag is set, which
//! means a DMA burst has begun.
//! This flag is cleared when BURST_COUNT reaches zero, or when the
//! HARDRESET or SOFTRESET bit is set.
//!
//! \return Returns \b true if the Burst Status Flag is set. Returns \b false
//! otherwise.
//
//*****************************************************************************
static inline bool
DMA_getBurstStatusFlag(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Read the Burst Status Flag and return appropriately.
    //
    return((HWREGH(base + DMA_O_CONTROL) & DMA_CONTROL_BURSTSTS) != 0U);
}

//*****************************************************************************
//
//! Gets the status of a DMA channel's Run Status Flag.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function returns \b true if the Run Status Flag is set, which
//! means the DMA channel is enabled.
//! This flag is cleared when a transfer completes (TRANSFER_COUNT = 0) and
//! continuous mode is disabled, or when the HARDRESET, SOFTRESET, or HALT bit
//! is set.
//!
//! \return Returns \b true if the channel is enabled. Returns \b false
//! otherwise.
//
//*****************************************************************************
static inline bool
DMA_getRunStatusFlag(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Read the Run Status Flag and return appropriately.
    //
    return((HWREGH(base + DMA_O_CONTROL) & DMA_CONTROL_RUNSTS) != 0U);
}

//*****************************************************************************
//
//! Gets the status of a DMA channel's Overflow Flag.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function returns \b true if the Overflow Flag is set, which
//! means peripheral event trigger was received while Peripheral Event Trigger
//! Flag was already set.
//! This flag can be cleared by writing to ERRCLR bit, using the function
//! DMA_clearErrorFlag().
//!
//! \return Returns \b true if the channel is enabled. Returns \b false
//! otherwise.
//
//*****************************************************************************
static inline bool
DMA_getOverflowFlag(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Read the Overflow Flag and return appropriately.
    //
    return((HWREGH(base + DMA_O_CONTROL) & DMA_CONTROL_OVRFLG) != 0U);
}

//*****************************************************************************
//
//! Gets the status of a DMA channel's peripheral trigger flag.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function returns \b true if a peripheral trigger event has occurred
//! The flag is automatically cleared when the first burst transfer begins, but
//! if needed, it can be cleared using DMA_clearTriggerFlag().
//!
//! \return Returns \b true if a peripheral trigger event has occurred and its
//! flag is set. Returns \b false otherwise.
//
//*****************************************************************************
static inline bool
DMA_getTriggerFlagStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Read the peripheral trigger flag and return appropriately.
    //
    return((HWREGH(base + DMA_O_CONTROL) & DMA_CONTROL_PERINTFLG) != 0U);
}

//*****************************************************************************
//
//! Starts a DMA channel.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function starts the DMA running, typically after you have configured
//! it. It will wait for the first trigger event to start operation. To halt
//! the channel use DMA_stopChannel().
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_startChannel(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Set the run bit.
    //
    EALLOW;
    HWREGH(base + DMA_O_CONTROL) |= DMA_CONTROL_RUN;
    EDIS;
}

//*****************************************************************************
//
//! Halts a DMA channel.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function halts the DMA at its current state and any current read-write
//! access is completed. To start the channel again use DMA_startChannel().
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_stopChannel(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Set the halt bit.
    //
    EALLOW;
    HWREGH(base + DMA_O_CONTROL) |= DMA_CONTROL_HALT;
    EDIS;
}

//*****************************************************************************
//
//! Enables a DMA channel interrupt source.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function enables the indicated DMA channel interrupt source.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_enableInterrupt(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Enable the specified DMA channel interrupt.
    //
    EALLOW;
    HWREGH(base + DMA_O_MODE) |= DMA_MODE_CHINTE;
    EDIS;
}

//*****************************************************************************
//
//! Disables a DMA channel interrupt source.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function disables the indicated DMA channel interrupt source.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_disableInterrupt(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Disable the specified DMA channel interrupt.
    //
    EALLOW;
    HWREGH(base + DMA_O_MODE) &= ~DMA_MODE_CHINTE;
    EDIS;
}

//*****************************************************************************
//
//! Enables the DMA channel overrun interrupt.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function enables the indicated DMA channel's ability to generate an
//! interrupt upon the detection of an overrun. An overrun is when a peripheral
//! event trigger is received by the DMA before a previous trigger on that
//! channel had been serviced and its flag had been cleared.
//!
//! Note that this is the same interrupt signal as the interrupt that gets
//! generated at the beginning/end of a transfer. That interrupt must first be
//! enabled using DMA_enableInterrupt() in order for the overrun interrupt to
//! be generated.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_enableOverrunInterrupt(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Enable the specified DMA channel interrupt.
    //
    EALLOW;
    HWREGH(base + DMA_O_MODE) |= DMA_MODE_OVRINTE;
    EDIS;
}

//*****************************************************************************
//
//! Disables the DMA channel overrun interrupt.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function disables the indicated DMA channel's ability to generate an
//! interrupt upon the detection of an overrun.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_disableOverrunInterrupt(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Disable the specified DMA channel interrupt.
    //
    EALLOW;
    HWREGH(base + DMA_O_MODE) &= ~DMA_MODE_OVRINTE;
    EDIS;
}

//*****************************************************************************
//
//! Clears the DMA channel error flags.
//!
//! \param base is the base address of the DMA channel control registers.
//!
//! This function clears both the DMA channel's sync error flag and its
//! overrun error flag.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_clearErrorFlag(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Write to the error clear bit.
    //
    EALLOW;
    HWREGH(base + DMA_O_CONTROL) |= DMA_CONTROL_ERRCLR;
    EDIS;
}

//*****************************************************************************
//
//! Sets the interrupt generation mode of a DMA channel interrupt.
//!
//! \param base is the base address of the DMA channel control registers.
//! \param mode is a flag to indicate the channel interrupt mode.
//!
//! This function sets the channel interrupt mode. When the \e mode parameter
//! is \b DMA_INT_AT_END, the DMA channel interrupt will be generated at the
//! end of the transfer. If \b DMA_INT_AT_BEGINNING, the interrupt will be
//! generated at the beginning of a new transfer. Generating at the beginning
//! of a new transfer is the default behavior.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_setInterruptMode(uint32_t base, DMA_InterruptMode mode)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    EALLOW;

    //
    // Write the selected interrupt generation mode to the register.
    //
    if(mode == DMA_INT_AT_END)
    {
        HWREGH(base + DMA_O_MODE) |= DMA_MODE_CHINTMODE;
    }
    else
    {
        HWREGH(base + DMA_O_MODE) &= ~DMA_MODE_CHINTMODE;
    }

    EDIS;
}

//*****************************************************************************
//
//! Sets the DMA channel priority mode.
//!
//! \param ch1IsHighPri is a flag to indicate the channel interrupt mode.
//!
//! This function sets the channel interrupt mode. When the \e ch1IsHighPri
//! parameter is \b false, the DMA channels are serviced in round-robin mode.
//! This is the default behavior.
//!
//! If \b true, channel 1 will be given higher priority than the other
//! channels. This means that if a channel 1 trigger occurs, the current word
//! transfer on any other channel is completed and channel 1 is serviced for
//! the complete burst count. The lower-priority channel's interrupted transfer
//! will then resume.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_setPriorityMode(bool ch1IsHighPri)
{
    EALLOW;

    //
    // Write the selected priority mode to the register.
    //
    if(ch1IsHighPri)
    {
        HWREGH(DMA_BASE + DMA_O_PRIORITYCTRL1) |=
            DMA_PRIORITYCTRL1_CH1PRIORITY;
    }
    else
    {
        HWREGH(DMA_BASE + DMA_O_PRIORITYCTRL1) &=
            ~DMA_PRIORITYCTRL1_CH1PRIORITY;
    }

    EDIS;
}

//*****************************************************************************
//
//! Configures the source address for the DMA channel
//!
//! \param base is the base address of the DMA channel control registers.
//! \param *srcAddr is a source address.
//!
//! This function configures the source address of a DMA
//! channel.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_configSourceAddress(uint32_t base, const void *srcAddr)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    EALLOW;

    //
    // Set up SOURCE address.
    //
    HWREG(base + DMA_O_SRC_BEG_ADDR_SHADOW) = (uint32_t)srcAddr;
    HWREG(base + DMA_O_SRC_ADDR_SHADOW)     = (uint32_t)srcAddr;

    EDIS;
}

//*****************************************************************************
//
//! Configures the destination address for the DMA channel
//!
//! \param base is the base address of the DMA channel control registers.
//! \param *destAddr is the destination address.
//!
//! This function configures the destinaton address of a DMA
//! channel.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DMA_configDestAddress(uint32_t base, const void *destAddr)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    EALLOW;

    //
    // Set up DESTINATION address.
    //
    HWREG(base + DMA_O_DST_BEG_ADDR_SHADOW) = (uint32_t)destAddr;
    HWREG(base + DMA_O_DST_ADDR_SHADOW)     = (uint32_t)destAddr;

    EDIS;
}

//*****************************************************************************
//
//! Setup DMA to transfer data on the specified channel.
//!
//! \param base is Base address of the DMA channel control register
//! \param *transfParams configuration parameter
//!                      Refer struct #DMA_ConfigParams
//!
//! This function configures the DMA transfer on the specified channel.
//!
//! \return None.
//
//*****************************************************************************
extern void
DMA_configChannel(uint32_t base, const DMA_ConfigParams *transfParams);

//*****************************************************************************
//
//! Configures the DMA channel
//!
//! \param base is the base address of the DMA channel control registers.
//! \param *destAddr is the destination address.
//! \param *srcAddr is a source address.
//!
//! This function configures the source and destination addresses of a DMA
//! channel. The parameters are pointers to the data to be transferred.
//!
//! \return None.
//
//*****************************************************************************
extern void
DMA_configAddresses(uint32_t base, const void *destAddr, const void *srcAddr);

//*****************************************************************************
//
//! Configures the DMA channel's burst settings.
//!
//! \param base is the base address of the DMA channel control registers.
//! \param size is the number of words transferred per burst.
//! \param srcStep is the amount to increment or decrement the source address
//! after each word of a burst.
//! \param destStep is the amount to increment or decrement the destination
//! address after each word of a burst.
//!
//! This function configures the size of each burst and the address step size.
//!
//! The \e size parameter is the number of words that will be transferred
//! during a single burst. Possible amounts range from 1 word to 32 words.
//!
//! The \e srcStep and \e destStep parameters specify the address step that
//! should be added to the source and destination addresses after each
//! transferred word of a burst. Only signed values from -4096 to 4095 are
//! valid.
//!
//! \note Note that regardless of what data size (configured by
//! DMA_configMode()) is used, parameters are in terms of 16-bits words.
//!
//! \return None.
//
//*****************************************************************************
extern void DMA_configBurst(uint32_t base, uint16_t size, int16_t srcStep,
                            int16_t destStep);

//*****************************************************************************
//
//! Configures the DMA channel's transfer settings.
//!
//! \param base is the base address of the DMA channel control registers.
//! \param transferSize is the number of bursts per transfer.
//! \param srcStep is the amount to increment or decrement the source address
//! after each burst of a transfer unless a wrap occurs.
//! \param destStep is the amount to increment or decrement the destination
//! address after each burst of a transfer unless a wrap occurs.
//!
//! This function configures the transfer size and the address step that is
//! made after each burst.
//!
//! The \e transferSize parameter is the number of bursts per transfer. If DMA
//! channel interrupts are enabled, they will occur after this number of bursts
//! have completed. The maximum number of bursts is 65536.
//!
//! The \e srcStep and \e destStep parameters specify the address step that
//! should be added to the source and destination addresses after each
//! transferred burst of a transfer. Only signed values from -4096 to 4095 are
//! valid. If a wrap occurs, these step values will be ignored. Wrapping is
//! configured with DMA_configWrap().
//!
//! \note Note that regardless of what data size (configured by
//! DMA_configMode()) is used, parameters are in terms of 16-bits words.
//!
//! \return None.
//
//*****************************************************************************
extern void
DMA_configTransfer(uint32_t base, uint32_t transferSize, int16_t srcStep,
                   int16_t destStep);

//*****************************************************************************
//
//! Configures the DMA channel's wrap settings.
//!
//! \param base is the base address of the DMA channel control registers.
//! \param srcWrapSize is the number of bursts to be transferred before a wrap
//! of the source address occurs.
//! \param srcStep is the amount to increment or decrement the source address
//! after each burst of a transfer unless a wrap occurs.
//! \param destWrapSize is the number of bursts to be transferred before a wrap
//! of the destination address occurs.
//! \param destStep is the amount to increment or decrement the destination
//! address after each burst of a transfer unless a wrap occurs.
//!
//! This function configures the DMA channel's wrap settings.
//!
//! The \e srcWrapSize and \e destWrapSize parameters are the number of bursts
//! that are to be transferred before their respective addresses are wrapped.
//! The maximum wrap size is 65536 bursts.
//!
//! The \e srcStep and \e destStep parameters specify the address step that
//! should be added to the source and destination addresses when the wrap
//! occurs.  Only signed values from -4096 to 4095 are valid.
//!
//! \note Note that regardless of what data size (configured by
//! DMA_configMode()) is used, parameters are in terms of 16-bits words.
//!
//! \return None.
//
//*****************************************************************************
extern void
DMA_configWrap(uint32_t base, uint32_t srcWrapSize, int16_t srcStep,
               uint32_t destWrapSize, int16_t destStep);

//*****************************************************************************
//
//! Configures the DMA channel trigger and mode.
//!
//! \param base is the base address of the DMA channel control registers.
//! \param trigger is the interrupt source that triggers a DMA transfer.
//! \param config is a bit field of several configuration selections.
//!
//! This function configures the DMA channel's trigger and mode.
//!
//! The \e trigger parameter is the interrupt source that will trigger the
//! start of a DMA transfer.
//!
//! The \e config parameter is the logical OR of the following values:
//! - \b DMA_CFG_ONESHOT_DISABLE or \b DMA_CFG_ONESHOT_ENABLE. If enabled,
//!   the subsequent burst transfers occur without additional event triggers
//!   after the first event trigger. If disabled, only one burst transfer is
//!   performed per event trigger.
//! - \b DMA_CFG_CONTINUOUS_DISABLE or \b DMA_CFG_CONTINUOUS_ENABLE. If enabled
//!   the DMA reinitializes when the transfer count is zero and waits for the
//!   next interrupt event trigger. If disabled, the DMA stops and clears the
//!   run status bit.
//! - \b DMA_CFG_SIZE_16BIT or \b DMA_CFG_SIZE_32BIT. This setting selects
//!   whether the databus width is 16 or 32 bits.
//!
//! \return None.
//
//*****************************************************************************
extern void
DMA_configMode(uint32_t base, DMA_Trigger trigger, uint32_t config);

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

#endif // DMA_H
