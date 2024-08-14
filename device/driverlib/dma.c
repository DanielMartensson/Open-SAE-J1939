//###########################################################################
//
// FILE:   dma.c
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

#include "dma.h"

//*****************************************************************************
//
// DMA_configAddresses
//
//*****************************************************************************
void DMA_configAddresses(uint32_t base, const void *destAddr,
                         const void *srcAddr)
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

    //
    // Set up DESTINATION address.
    //
    HWREG(base + DMA_O_DST_BEG_ADDR_SHADOW) = (uint32_t)destAddr;
    HWREG(base + DMA_O_DST_ADDR_SHADOW)     = (uint32_t)destAddr;

    EDIS;
}

//*****************************************************************************
//
// DMA_configBurst
//
//*****************************************************************************
void DMA_configBurst(uint32_t base, uint16_t size, int16_t srcStep,
                     int16_t destStep)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));
    ASSERT((size >= 1U) && (size <= 32U));

    EALLOW;

    //
    // Set up BURST registers.
    //
    HWREGH(base + DMA_O_BURST_SIZE)     = size - 1U;
    HWREGH(base + DMA_O_SRC_BURST_STEP) = srcStep;
    HWREGH(base + DMA_O_DST_BURST_STEP) = destStep;

    EDIS;
}

//*****************************************************************************
//
// DMA_configTransfer
//
//*****************************************************************************
void DMA_configTransfer(uint32_t base, uint32_t transferSize, int16_t srcStep,
                        int16_t destStep)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));
    ASSERT(transferSize <= 0x10000U);

    EALLOW;

    //
    // Set up TRANSFER registers.
    //
    HWREGH(base + DMA_O_TRANSFER_SIZE)     = (uint16_t)(transferSize - 1U);
    HWREGH(base + DMA_O_SRC_TRANSFER_STEP) = srcStep;
    HWREGH(base + DMA_O_DST_TRANSFER_STEP) = destStep;

    EDIS;
}

//*****************************************************************************
//
// DMA_configWrap
//
//*****************************************************************************
void DMA_configWrap(uint32_t base, uint32_t srcWrapSize, int16_t srcStep,
                    uint32_t destWrapSize, int16_t destStep)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));
    ASSERT((srcWrapSize <= 0x10000U) || (destWrapSize <= 0x10000U));

    EALLOW;

    //
    // Set up WRAP registers.
    //
    HWREGH(base + DMA_O_SRC_WRAP_SIZE) = (uint16_t)(srcWrapSize - 1U);
    HWREGH(base + DMA_O_SRC_WRAP_STEP) = srcStep;

    HWREGH(base + DMA_O_DST_WRAP_SIZE) = (uint16_t)(destWrapSize - 1U);
    HWREGH(base + DMA_O_DST_WRAP_STEP) = destStep;

    EDIS;
}

//*****************************************************************************
//
// DMA_configMode
//
//*****************************************************************************
void DMA_configMode(uint32_t base, DMA_Trigger trigger, uint32_t config)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    EALLOW;

    //
    // Set up trigger selection in the CMA/CLA trigger source selection
    // registers. These are considered part of system control.
    //
    switch(base)
    {
        case DMA_CH1_BASE:
            //
            // Channel 1
            //
            HWREG(DMACLASRCSEL_BASE + SYSCTL_O_DMACHSRCSEL1) =
                (HWREG(DMACLASRCSEL_BASE + SYSCTL_O_DMACHSRCSEL1) &
                 ~((uint32_t)SYSCTL_DMACHSRCSEL1_CH1_M)) |
                ((uint32_t)trigger << SYSCTL_DMACHSRCSEL1_CH1_S);

            //
            // Set peripheral interrupt select bits to the channel number.
            //
            HWREGH(DMA_CH1_BASE + DMA_O_MODE) =
               (HWREGH(DMA_CH1_BASE + DMA_O_MODE) & ~DMA_MODE_PERINTSEL_M) | 1U;
            break;

        case DMA_CH2_BASE:
            //
            // Channel 2
            //
            HWREG(DMACLASRCSEL_BASE + SYSCTL_O_DMACHSRCSEL1) =
                (HWREG(DMACLASRCSEL_BASE + SYSCTL_O_DMACHSRCSEL1) &
                 ~((uint32_t)SYSCTL_DMACHSRCSEL1_CH2_M)) |
                ((uint32_t)trigger << SYSCTL_DMACHSRCSEL1_CH2_S);

            //
            // Set peripheral interrupt select bits to the channel number.
            //
            HWREGH(DMA_CH2_BASE + DMA_O_MODE) =
               (HWREGH(DMA_CH2_BASE + DMA_O_MODE) & ~DMA_MODE_PERINTSEL_M) | 2U;
            break;

        case DMA_CH3_BASE:
            //
            // Channel 3
            //
            HWREG(DMACLASRCSEL_BASE + SYSCTL_O_DMACHSRCSEL1) =
                (HWREG(DMACLASRCSEL_BASE + SYSCTL_O_DMACHSRCSEL1) &
                 ~((uint32_t)SYSCTL_DMACHSRCSEL1_CH3_M)) |
                ((uint32_t)trigger << SYSCTL_DMACHSRCSEL1_CH3_S);

            //
            // Set peripheral interrupt select bits to the channel number.
            //
            HWREGH(DMA_CH3_BASE + DMA_O_MODE) =
               (HWREGH(DMA_CH3_BASE + DMA_O_MODE) & ~DMA_MODE_PERINTSEL_M) | 3U;
            break;

        case DMA_CH4_BASE:
            //
            // Channel 4
            //
            HWREG(DMACLASRCSEL_BASE + SYSCTL_O_DMACHSRCSEL1) =
                (HWREG(DMACLASRCSEL_BASE + SYSCTL_O_DMACHSRCSEL1) &
                 ~((uint32_t)SYSCTL_DMACHSRCSEL1_CH4_M)) |
                ((uint32_t)trigger << SYSCTL_DMACHSRCSEL1_CH4_S);

            //
            // Set peripheral interrupt select bits to the channel number.
            //
            HWREGH(DMA_CH4_BASE + DMA_O_MODE) =
               (HWREGH(DMA_CH4_BASE + DMA_O_MODE) & ~DMA_MODE_PERINTSEL_M) | 4U;
            break;

        case DMA_CH5_BASE:
            //
            // Channel 5
            //
            HWREG(DMACLASRCSEL_BASE + SYSCTL_O_DMACHSRCSEL2) =
                (HWREG(DMACLASRCSEL_BASE + SYSCTL_O_DMACHSRCSEL2) &
                 ~((uint32_t)SYSCTL_DMACHSRCSEL2_CH5_M)) |
                ((uint32_t)trigger << SYSCTL_DMACHSRCSEL2_CH5_S);

            //
            // Set peripheral interrupt select bits to the channel number.
            //
            HWREGH(DMA_CH5_BASE + DMA_O_MODE) =
               (HWREGH(DMA_CH5_BASE + DMA_O_MODE) & ~DMA_MODE_PERINTSEL_M) | 5U;
            break;

        case DMA_CH6_BASE:
            //
            // Channel 6
            //
            HWREG(DMACLASRCSEL_BASE + SYSCTL_O_DMACHSRCSEL2) =
                (HWREG(DMACLASRCSEL_BASE + SYSCTL_O_DMACHSRCSEL2) &
                 ~((uint32_t)SYSCTL_DMACHSRCSEL2_CH6_M)) |
                ((uint32_t)trigger << SYSCTL_DMACHSRCSEL2_CH6_S);

            //
            // Set peripheral interrupt select bits to the channel number.
            //
            HWREGH(DMA_CH6_BASE + DMA_O_MODE) =
               (HWREGH(DMA_CH6_BASE + DMA_O_MODE) & ~DMA_MODE_PERINTSEL_M) | 6U;
            break;

        default:
            //
            // Invalid base.
            //
            break;
    }

    //
    // Write the configuration to the mode register.
    //
    HWREGH(base + DMA_O_MODE) &= ~(DMA_MODE_DATASIZE | DMA_MODE_CONTINUOUS |
                                   DMA_MODE_ONESHOT);
    HWREGH(base + DMA_O_MODE) |= config;

    EDIS;
}

//*****************************************************************************
//
// DMA_configChannel
//
//*****************************************************************************
void DMA_configChannel(uint32_t base, const DMA_ConfigParams *transfParams)
{
    //
    // Check the arguments.
    //
    ASSERT(DMA_isBaseValid(base));

    //
    // Configure DMA Channel
    //
    DMA_configAddresses(base, (const void *)transfParams->destAddr,
                        (const void *)transfParams->srcAddr);

    //
    // Configure the size of each burst and the address step size
    //
    DMA_configBurst(base, transfParams->burstSize, transfParams->srcBurstStep,
                    transfParams->destBurstStep);

    //
    // Configure the transfer size and the address step that is
    // made after each burst.
    //
    DMA_configTransfer(base, transfParams->transferSize,
                       transfParams->srcTransferStep,
                       transfParams->destTransferStep);

    //
    // Configure the DMA channel's wrap settings
    //
    DMA_configWrap(base, transfParams->srcWrapSize, transfParams->srcWrapStep,
                   transfParams->destWrapSize, transfParams->destWrapStep);

    //
    // Configure the DMA channel's trigger and mode
    //
    DMA_configMode(base, transfParams->transferTrigger,
                   transfParams->transferMode | transfParams->reinitMode |
                   transfParams->configSize);

    //
    // Enable the selected peripheral trigger to start a DMA transfer
    //
    DMA_enableTrigger(base);

    if(transfParams->enableInterrupt)
    {
        //
        // Set the channel interrupt mode
        //
        DMA_setInterruptMode(base, transfParams->interruptMode);

        //
        // Enable the indicated DMA channel interrupt source
        //
        DMA_enableInterrupt(base);
    }
    else
    {
        //
        // Disable the indicated DMA channel interrupt source
        //
        DMA_disableInterrupt(base);
    }
}

