//###########################################################################
//
// FILE:   mcbsp.c
//
// TITLE:  C28x McBSP driver.
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

#include <stdbool.h>
#include <stdint.h>
#include "mcbsp.h"

//*****************************************************************************
//
// McBSP_transmit16BitdataNonBlocking
//
//*****************************************************************************
void
McBSP_transmit16BitDataNonBlocking(uint32_t base, uint16_t data)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Write data.
    //
    McBSP_write16bitData(base, data);
}

//*****************************************************************************
//
// McBSP_transmit16BitdataBlocking
//
//*****************************************************************************
void
McBSP_transmit16BitDataBlocking(uint32_t base, uint16_t data)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Check if Transmitter buffer is ready.
    //
    while(!McBSP_isTxReady(base))
    {
    }

    //
    // Write data.
    //
    McBSP_write16bitData(base, data);
}

//*****************************************************************************
//
// McBSP_transmit32BitDataNonBlocking
//
//*****************************************************************************
void
McBSP_transmit32BitDataNonBlocking(uint32_t base, uint32_t data)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Write data.
    //
    McBSP_write32bitData(base, data);
}

//*****************************************************************************
//
// McBSP_transmit32BitdataBlocking
//
//*****************************************************************************
void
McBSP_transmit32BitDataBlocking(uint32_t base, uint32_t data)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Check if Transmitter buffer is ready.
    //
    while(!McBSP_isTxReady(base))
    {
    }

    //
    // Write data.
    //
    McBSP_write32bitData(base, data);
}

//*****************************************************************************
//
// McBSP_receive16BitDataNonBlocking
//
//*****************************************************************************
void
McBSP_receive16BitDataNonBlocking(uint32_t base, uint16_t *receiveData)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Read the data.
    //
    *receiveData = McBSP_read16bitData(base);
}

//*****************************************************************************
//
// McBSP_receive16BitDataBlocking
//
//*****************************************************************************
void
McBSP_receive16BitDataBlocking(uint32_t base, uint16_t *receiveData)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Wait until new data arrives.
    //
    while(!McBSP_isRxReady(base))
    {
    }

    //
    // Read the data.
    //
    *receiveData = McBSP_read16bitData(base);
}

//*****************************************************************************
//
// McBSP_receive32BitDataNonBlocking
//
//*****************************************************************************
void
McBSP_receive32BitDataNonBlocking(uint32_t base, uint32_t *receiveData)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Read the data.
    //
    *receiveData = McBSP_read32bitData(base);
}

//*****************************************************************************
//
// McBSP_receive32BitDataBlocking
//
//*****************************************************************************
void
McBSP_receive32BitDataBlocking(uint32_t base, uint32_t *receiveData)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Wait until new data arrives.
    //
    while(!McBSP_isRxReady(base))
    {
    }

    //
    // Read the data.
    //
    *receiveData = McBSP_read32bitData(base);
}

//*****************************************************************************
//
// McBSP_setRxDataSize
//
//*****************************************************************************
void
McBSP_setRxDataSize(uint32_t base, const McBSP_DataPhaseFrame dataFrame,
                    const McBSP_DataBitsPerWord  bitsPerWord,
                    uint16_t wordsPerFrame)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));
    ASSERT(wordsPerFrame < 128U);

    if(dataFrame == MCBSP_PHASE_ONE_FRAME)
    {
        //
        // Set bits per word , write to RWDLEN1 and words per frame , write to
        // RFRLEN1.
        //
        HWREGH(base + MCBSP_O_RCR1) =
        ((HWREGH(base + MCBSP_O_RCR1) & ~MCBSP_RCR1_M) |
         ((uint16_t)bitsPerWord | (wordsPerFrame << MCBSP_RCR1_RFRLEN1_S)));
    }
    else
    {
        //
        // Set bits per word , write to RWDLEN2 and words per frame, write to
        // RFRLEN2.
        //
        HWREGH(base + MCBSP_O_RCR2) =
        ((HWREGH(base + MCBSP_O_RCR2) & ~MCBSP_RCR2_M) |
         ((uint16_t)bitsPerWord | (wordsPerFrame << MCBSP_RCR2_RFRLEN2_S)));
    }
}

//*****************************************************************************
//
// McBSP_setTxDataSize
//
//*****************************************************************************
void
McBSP_setTxDataSize(uint32_t base, const McBSP_DataPhaseFrame dataFrame,
                    const McBSP_DataBitsPerWord  bitsPerWord,
                    uint16_t wordsPerFrame)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));
    ASSERT(wordsPerFrame < 128U);

    if(dataFrame == MCBSP_PHASE_ONE_FRAME)
    {
        //
        // Set bits per word XWDLEN1 and words per frame XFRLEN1.
        //
        HWREGH(base + MCBSP_O_XCR1) =
        ((HWREGH(base + MCBSP_O_XCR1) & ~MCBSP_XCR1_M) |
        ((uint16_t)bitsPerWord | (wordsPerFrame << MCBSP_XCR1_XFRLEN1_S)));
    }
    else
    {
        //
        // Set bits per word XWDLEN2 and words per frame XFRLEN2.
        //
        HWREGH(base + MCBSP_O_XCR2) =
        ((HWREGH(base + MCBSP_O_XCR2) & ~MCBSP_XCR2_M) |
         ((uint16_t)bitsPerWord | (wordsPerFrame << MCBSP_XCR2_XFRLEN2_S)));
    }
}

//*****************************************************************************
//
// McBSP_disableRxChannel
//
//*****************************************************************************
void
McBSP_disableRxChannel(uint32_t base,
                       const McBSP_MultichannelPartition partition,
                       uint16_t channel)
{
    uint16_t block;
    uint16_t bitOffset;
    uint16_t registerOffset;
    uint16_t oddBlock;

    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));
    ASSERT(channel < 128U);

    //
    // Determine channel block.
    //
    block = channel >> 4U;

    //
    // Determine bit location.
    //
    bitOffset = channel - (block * 16U);

    //
    // Determine register offset for Eight partition.
    //
    if(partition == MCBSP_MULTICHANNEL_EIGHT_PARTITION)
    {
        //
        // For channel number  0 - 31.
        //
        if(channel < 32U)
        {
            //
            // Determines whether it is RCERA or RCERB.
            //
            registerOffset = channel >> 4U;
        }

        //
        // For channel number 32 - 127.
        //
        else
        {
            //
            // Determines whether it is RCERC or RCERD or RCERE or RCERF or
            // RCERG or RCERH.
            //
            oddBlock = (block & 1U);
            registerOffset = oddBlock + (2U * (block - (2U + oddBlock))) +
                             0x5U;
        }
    }

    //
    // Determine register offset for Two partition.
    //
    else
    {
        //
        // Determine whether it is RCERA or RCERB.
        //
        registerOffset = block & 0x1U;
    }

    HWREGH(base + MCBSP_O_RCERA + registerOffset) &= ~(1U << bitOffset);
}

//*****************************************************************************
//
// McBSP_enableRxChannel
//
//*****************************************************************************
void
McBSP_enableRxChannel(uint32_t base,
                      const McBSP_MultichannelPartition partition,
                      uint16_t channel)
{
    uint16_t block;
    uint16_t bitOffset;
    uint16_t registerOffset;
    uint16_t oddBlock;

    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));
    ASSERT(channel < 128U);

    //
    // Determine channel block.
    //
    block = channel >> 4U;

    //
    // Determine bit location.
    //
    bitOffset = channel - (block * 16U);

    //
    // Determine register offset for Eight partition.
    //
    if(partition == MCBSP_MULTICHANNEL_EIGHT_PARTITION)
    {
        //
        // For channel number  0 - 31.
        //
        if(channel < 32U)
        {
            //
            // Determines whether it is RCERA or RCERB.
            //
            registerOffset = channel >> 4U;
        }

        //
        // For channel number  32 - 127.
        //
        else
        {
            //
            // Determines whether it is RCERC or RCERD or RCERE or RCERF or
            // RCERG or RCERH.
            //
            oddBlock = (block & 1U);
            registerOffset = oddBlock + (2U * (block - (2U + oddBlock))) +
                             0x5U;
        }
    }

    //
    // Determine register offset for Two partition.
    //
    else
    {
        //
        // Determine whether it is RCERA or RCERB.
        //
        registerOffset = block & 0x1U;
    }

    HWREGH(base + MCBSP_O_RCERA + registerOffset) |= (1U << bitOffset);
}

//*****************************************************************************
//
// McBSP_disableTxChannel
//
//*****************************************************************************
void
McBSP_disableTxChannel(uint32_t base,
                       const McBSP_MultichannelPartition partition,
                       uint16_t channel)
{
    uint16_t block;
    uint16_t bitOffset;
    uint16_t registerOffset;
    uint16_t oddBlock;

    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));
    ASSERT(channel < 128U);

    //
    // Determine channel block.
    //
    block = channel >> 4U;

    //
    // Determine bit location.
    //
    bitOffset = channel - (block * 16U);

    //
    // Determine register offset for Eight partition.
    //
    if(partition == MCBSP_MULTICHANNEL_EIGHT_PARTITION)
    {
        //
        // For channel number  0 - 31.
        //
        if(channel < 32U)
        {
            //
            // Determines whether it is XCERA or XCERB.
            //
            registerOffset = channel >> 4U;
        }

        //
        // For channel number  32 - 127.
        //
        else
        {
            //
            // Determines whether it is XCERC or XCERD or XCERE or XCERF or
            // XCERG or XCERH.
            //
            oddBlock = (block & 1U);
            registerOffset = oddBlock + (2U * (block - (2U + oddBlock))) +
                             0x5U;
        }
    }

    //
    // Determine register offset for Two partition.
    //
    else
    {
        //
        // Determine whether it is XCERA or XCERB.
        //
        registerOffset = block & 0x1U;
    }

    HWREGH(base + MCBSP_O_XCERA + registerOffset) &= ~(1U << bitOffset);
}

//*****************************************************************************
//
// McBSP_enableTxChannel
//
//*****************************************************************************
void McBSP_enableTxChannel(uint32_t base,
                           const McBSP_MultichannelPartition partition,
                           uint16_t channel)
{
    uint16_t block;
    uint16_t bitOffset;
    uint16_t registerOffset;
    uint16_t oddBlock;

    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));
    ASSERT(channel < 128U);

    //
    // Determine channel block.
    //
    block = channel >> 4U;

    //
    // Determine bit location.
    //
    bitOffset = channel - (block * 16U);

    //
    // Determine register offset for Eight partition.
    //
    if(partition == MCBSP_MULTICHANNEL_EIGHT_PARTITION)
    {
        //
        // For channel number  0 - 31.
        //
        if(channel < 32U)
        {
            //
            // Determines whether it is XCERA or XCERB.
            //
            registerOffset = channel >> 4U;
        }

        //
        // For channel number  32 - 127.
        //
        else
        {
            //
            // Determines whether it is XCERC or XCERD or XCERE or XCERF or
            // XCERG or XCERH.
            //
            oddBlock = (block & 1U);
            registerOffset = oddBlock + (2U * (block - (2U + oddBlock))) +
                             0x5U;
        }
    }

    //
    // Determine register offset for Two partition.
    //
    else
    {
        //
        // Determine wheter it is XCERA or XCERB.
        //
        registerOffset = block & 0x1U;
    }

    HWREGH(base + MCBSP_O_XCERA + registerOffset) |= (1U << bitOffset);
}

//*****************************************************************************
//
// McBSP_configureTxClock
//
//*****************************************************************************
void
McBSP_configureTxClock(uint32_t base, const McBSP_ClockParams *ptrClockParams)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Select TX clock source as SRG or External.
    //
    McBSP_setTxClockSource(base,
                           (McBSP_TxClockSource)ptrClockParams->clockSourceTx);

    //
    // Check if using SRG to drive Transmitter clock.
    //
    if((McBSP_TxClockSource)ptrClockParams->clockSourceTx ==
        MCBSP_INTERNAL_TX_CLOCK_SOURCE)
    {
        //
        // Set the SRG clock source.
        //
        McBSP_setTxSRGClockSource(base,
                     (McBSP_SRGTxClockSource)ptrClockParams->clockTxSRGSource);

        //
        // Check if SRG is clocked from MCLKR pin. GSYNC feature can be enabled
        // in this case as SRG input clock source is MCLKR pin.
        //
        if((McBSP_SRGTxClockSource)ptrClockParams->clockTxSRGSource ==
           MCBSP_SRG_TX_CLOCK_SOURCE_MCLKR_PIN)
        {
            //
            // Set the input clock polarity.
            //
            McBSP_setRxClockPolarity(base,
                    (McBSP_RxClockPolarity)ptrClockParams->clockMCLKRPolarity);

            //
            // Check if SRG is to be synced with FSR that is GSYNC is to be
            // enabled or not.
            //
            if(ptrClockParams->clockSRGSyncFlag)
            {
                McBSP_enableSRGSyncFSR(base);
            }
            else
            {
                McBSP_disableSRGSyncFSR(base);
            }
        }

        //
        // Set SRG clock divider.
        //
        McBSP_setSRGDataClockDivider(base,
                                    (uint16_t)ptrClockParams->clockSRGDivider);
    }

    //
    // Input polarity if using external clock on MCLKX.
    // Output polarity if using SRG as clock source.
    //
    McBSP_setTxClockPolarity(base,
                    (McBSP_TxClockPolarity)ptrClockParams->clockMCLKXPolarity);
}

//*****************************************************************************
//
// McBSP_configureRxClock
//
//*****************************************************************************
void
McBSP_configureRxClock(uint32_t base, const McBSP_ClockParams *ptrClockParams)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Select RX clock source as SRG or External.
    //
    McBSP_setRxClockSource(base,
                           (McBSP_RxClockSource)ptrClockParams->clockSourceRx);

    //
    // Check if using SRG to drive Receiver clock.
    //
    if((McBSP_RxClockSource)ptrClockParams->clockSourceRx ==
        MCBSP_INTERNAL_RX_CLOCK_SOURCE)
    {
        //
        // Set the SRG clock source.
        //
        McBSP_setRxSRGClockSource(base,
                     (McBSP_SRGRxClockSource)ptrClockParams->clockRxSRGSource);

        //
        // Check if SRG is clocked from MCLKX pin. GSYNC cannot be enabled in
        // this case as GSYNC feature can be used only when SRG clock source is
        // MCLKR pin.
        //
        if((McBSP_SRGRxClockSource)ptrClockParams->clockRxSRGSource ==
           MCBSP_SRG_RX_CLOCK_SOURCE_MCLKX_PIN)
        {
            //
            // Set the input clock polarity.
            //
            McBSP_setTxClockPolarity(base,
                    (McBSP_TxClockPolarity)ptrClockParams->clockMCLKXPolarity);
        }

        //
        // Set SRG clock divider.
        //
        McBSP_setSRGDataClockDivider(base,
                                    (uint16_t)ptrClockParams->clockSRGDivider);
    }

    //
    // Input polarity if using external clock on MCLKR.
    // Output polarity if using SRG as clock source.
    //
    McBSP_setRxClockPolarity(base,
                    (McBSP_RxClockPolarity)ptrClockParams->clockMCLKRPolarity);
}

//*****************************************************************************
//
// McBSP_configureTxFrameSync
//
//*****************************************************************************
void
McBSP_configureTxFrameSync(uint32_t base,
                           const McBSP_TxFsyncParams *ptrFsyncParams)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Select frame-sync signal source.
    //
    McBSP_setTxFrameSyncSource(base,
                        (McBSP_TxFrameSyncSource)ptrFsyncParams->syncSourceTx);

    //
    // Check if using internal frame-sync source.
    //
    if((McBSP_TxFrameSyncSource)ptrFsyncParams->syncSourceTx ==
        MCBSP_TX_INTERNAL_FRAME_SYNC_SOURCE)
    {
        //
        // Select the internal frame-sync trigger source.
        //
        McBSP_setTxInternalFrameSyncSource(base,
               (McBSP_TxInternalFrameSyncSource)ptrFsyncParams->syncIntSource);

        //
        // Check if using SRG FSG to trigger frame-sync pulse and GSYNC feature
        // is disabled that is FSG is not derived from external MCLKR pin.
        //
        if((ptrFsyncParams->syncIntSource ==
            MCBSP_TX_INTERNAL_FRAME_SYNC_SRG) &&
           (ptrFsyncParams->syncSRGSyncFSRFlag == false))
        {
            //
            // Set the frame-sync pulse period and width dividers.
            //
            McBSP_setFrameSyncPulsePeriod(base,
                                           ptrFsyncParams->syncClockDivider);
            McBSP_setFrameSyncPulseWidthDivider(base,
                                             ptrFsyncParams->syncPulseDivider);
        }
    }

    //
    // Set the frame-sync polarity.
    //
    McBSP_setTxFrameSyncPolarity(base,
                   (McBSP_TxFrameSyncPolarity)ptrFsyncParams->syncFSXPolarity);

    //
    // Configure frame-sync error detect flag.
    //
    if(ptrFsyncParams->syncErrorDetect)
    {
        McBSP_enableTxFrameSyncErrorDetection(base);
    }
    else
    {
        McBSP_disableTxFrameSyncErrorDetection(base);
    }
}

//*****************************************************************************
//
// McBSP_configureRxFrameSync
//
//*****************************************************************************
void
McBSP_configureRxFrameSync(uint32_t base,
                           const McBSP_RxFsyncParams *ptrFsyncParams)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Select frame-sync signal source.
    //
    McBSP_setRxFrameSyncSource(base,
                        (McBSP_RxFrameSyncSource)ptrFsyncParams->syncSourceRx);

    //
    // If using internal frame-sync source.
    //
    if(ptrFsyncParams->syncSourceRx == MCBSP_RX_INTERNAL_FRAME_SYNC_SOURCE)
    {
        //
        // Check if GSYNC feature is disabled that is FSG is not derived
        // from external MCLKR pin.
        //
        if(ptrFsyncParams->syncSRGSyncFSRFlag == false)
        {
            //
            // Set the frame-sync pulse period and width dividers.
            //
            McBSP_setFrameSyncPulsePeriod(base,
                                           ptrFsyncParams->syncClockDivider);
            McBSP_setFrameSyncPulseWidthDivider(base,
                                           ptrFsyncParams->syncPulseDivider);
        }
    }

    //
    // Set the frame-sync polarity.
    //
    McBSP_setRxFrameSyncPolarity(base,
                   (McBSP_RxFrameSyncPolarity)ptrFsyncParams->syncFSRPolarity);

    //
    // Configure frame-sync error detect flag.
    //
    if(ptrFsyncParams->syncErrorDetect)
    {
        McBSP_enableRxFrameSyncErrorDetection(base);
    }
    else
    {
        McBSP_disableTxFrameSyncErrorDetection(base);
    }
}

//*****************************************************************************
//
// McBSP_configureTxDataFormat
//
//*****************************************************************************
void
McBSP_configureTxDataFormat(uint32_t base,
                            const McBSP_TxDataParams *ptrDataParams)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set loop back mode.
    //
    if(ptrDataParams->loopbackModeFlag)
    {
        McBSP_enableLoopback(base);
    }
    else
    {
        McBSP_disableLoopback(base);
    }

    //
    // Configure the module to work in McBSP.
    //
    McBSP_setClockStopMode(base, MCBSP_CLOCK_MCBSP_MODE);

    //
    // Start with single phase - a TX must at least has a single phase.
    //
    McBSP_setTxDataSize(base, MCBSP_PHASE_ONE_FRAME,
                        (McBSP_DataBitsPerWord)ptrDataParams->phase1WordLength,
                        ptrDataParams->phase1FrameLength);

    //
    // Disable second phase by default.
    //
    McBSP_disableTwoPhaseTx(base);

    //
    // Check if second phase is being used.
    //
    if(ptrDataParams->twoPhaseModeFlag)
    {
        //
        // Enable second phase.
        //
        McBSP_enableTwoPhaseTx(base);

        //
        // Set the parameters for the second phase.
        //
        McBSP_setTxDataSize(base, MCBSP_PHASE_TWO_FRAME,
                        (McBSP_DataBitsPerWord)ptrDataParams->phase2WordLength,
                         ptrDataParams->phase2FrameLength);
    }

    //
    // Set the Tx companding mode.
    //
    McBSP_setTxCompandingMode(base,
                          (McBSP_CompandingMode)ptrDataParams->compandingMode);

    //
    // Set Tx data delay in bits.
    //
    McBSP_setTxDataDelayBits(base,
                            (McBSP_DataDelayBits)ptrDataParams->dataDelayBits);

    //
    // Set DX pin delay.
    //
    if(ptrDataParams->pinDelayEnableFlag)
    {
        McBSP_enableDxPinDelay(base);
    }
    else
    {
        McBSP_disableDxPinDelay(base);
    }

    //
    // Set the transmitter interrupt source.
    //
    McBSP_setTxInterruptSource(base,
                        (McBSP_TxInterruptSource)ptrDataParams->interruptMode);
}

//*****************************************************************************
//
// McBSP_configureRxDataFormat
//
//*****************************************************************************
void
McBSP_configureRxDataFormat(uint32_t base,
                            const McBSP_RxDataParams *ptrDataParams)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set loop back mode.
    //
    if(ptrDataParams->loopbackModeFlag)
    {
        McBSP_enableLoopback(base);
    }
    else
    {
        McBSP_disableLoopback(base);
    }

    //
    // Configure the module to work in McBSP mode.
    //
    McBSP_setClockStopMode(base, MCBSP_CLOCK_MCBSP_MODE);

    //
    // Start with single phase - an RX must at least have a single phase.
    //
    McBSP_setRxDataSize(base, MCBSP_PHASE_ONE_FRAME,
                        (McBSP_DataBitsPerWord)ptrDataParams->phase1WordLength,
                         ptrDataParams->phase1FrameLength);

    //
    // Disable second phase by default.
    //
    McBSP_disableTwoPhaseRx(base);

    //
    // Check if second phase is to be enabled.
    //
    if(ptrDataParams->twoPhaseModeFlag)
    {
        //
        // Enable second phase.
        //
        McBSP_enableTwoPhaseRx(base);

        //
        // Set the parameters for the second phase.
        //
        McBSP_setRxDataSize(base, MCBSP_PHASE_TWO_FRAME,
                        (McBSP_DataBitsPerWord)ptrDataParams->phase2WordLength,
                         ptrDataParams->phase2FrameLength);
    }

    //
    // Set the receiver companding mode.
    //
    McBSP_setRxCompandingMode(base,
                          (McBSP_CompandingMode)ptrDataParams->compandingMode);

    //
    // Set receiver data delay in bits.
    //
    McBSP_setRxDataDelayBits(base,
                         (McBSP_DataDelayBits)ptrDataParams->dataDelayBits);

    //
    // Set receiver sign-extension and justification mode.
    //
    McBSP_setRxSignExtension(base,
                        (McBSP_RxSignExtensionMode)ptrDataParams->signExtMode);

    //
    // Set the receiver interrupt source.
    //
    McBSP_setRxInterruptSource(base,
                        (McBSP_RxInterruptSource)ptrDataParams->interruptMode);
}

//*****************************************************************************
//
// McBSP_configureTxMultichannel
//
//*****************************************************************************
uint16_t
McBSP_configureTxMultichannel(uint32_t base,
                              const McBSP_TxMultichannelParams *ptrMchnParams)
{
    uint16_t index;
    uint16_t block;
    uint16_t partitionAblock;
    uint16_t partitionBblock;
    uint16_t partitionAflag;
    uint16_t partitionBflag;
    uint16_t errorTx;

    errorTx = 0U;
    partitionAblock = 0U;
    partitionBblock = 0U;

    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Configure Tx Channel Selection mode.
    //
    McBSP_setTxChannelMode(base,
                       (McBSP_TxChannelMode)ptrMchnParams->multichannelModeTx);

    //
    // Configuration for multichannel selections that is for
    // MCBSP_TX_CHANNEL_SELECTION_ENABLED,
    // MCBSP_ENABLE_MASKED_TX_CHANNEL_SELECTION or
    // MCBSP_SYMMERTIC_RX_TX_SELECTION.
    //
    if(((McBSP_TxChannelMode)ptrMchnParams->multichannelModeTx) !=
         MCBSP_ALL_TX_CHANNELS_ENABLED)
    {
        //
        // Select 2 partition or 8 partition.
        //
        McBSP_setTxMultichannelPartition(base,
                      (McBSP_MultichannelPartition)ptrMchnParams->partitionTx);

        //
        // Disable dual phase transmission mode.
        //
        McBSP_disableTwoPhaseTx(base);

        //
        // Multichannel configuration for 2 partition mode.
        //
        if((McBSP_MultichannelPartition)ptrMchnParams->partitionTx ==
            MCBSP_MULTICHANNEL_TWO_PARTITION)
        {
            if(((uint16_t)ptrMchnParams->channelCountTx) > 32U)
            {
                errorTx = MCBSP_ERROR_EXCEEDED_CHANNELS;
            }
            partitionAflag = 0U;
            partitionBflag = 0U;

            //
            // Assign blocks to partition for the provided channels and
            // enable the channels. Only the channels which belong to the
            // block currently assigned to partition A or B can be enabled.
            //
            for(index = 0U; index < (uint16_t)ptrMchnParams->channelCountTx;
                index++)
            {
                //
                // Get the block to which channel belongs.
                //
                block = (uint16_t)(*((ptrMchnParams->ptrChannelsListTx) +
                                     index)) >> 4U;

                //
                // Check if channel block can be assigned to partition A. Only
                // even numbered blocks can be assigned to partition A.
                //
                if((block & 0x1U) == 0U)
                {
                    //
                    // Check if block is yet to be assigned to partition A.
                    //
                    if(partitionAflag == 0U)
                    {
                        //
                        // Assign block to partition A.
                        //
                        McBSP_setTxTwoPartitionBlock(base,
                                                  (McBSP_PartitionBlock)block);

                        //
                        // Set flag to indicate that a block is now assigned
                        // to partition A. Only one block can be assigned to
                        // a partition at a time in 2 partition mode.
                        //
                        partitionAflag = 1U;
                        partitionAblock = block;
                    }

                    //
                    // Check if the channel to be enabled belong to the block
                    // assigned to partition A.
                    //
                    if(partitionAblock == block)
                    {
                        //
                        // Enable the channel belonging to the block assigned
                        // to partition A.
                        //
                        McBSP_enableTxChannel(base,
                                              MCBSP_MULTICHANNEL_TWO_PARTITION,
                               (uint16_t)(*(ptrMchnParams->ptrChannelsListTx)
                                              + index));
                    }
                    else
                    {
                        errorTx = MCBSP_ERROR_2_PARTITION_A;
                    }
                }

                //
                // Check if channel block can be assigned to partition B. Only
                // odd numbered blocks can be assigned to partition B.
                //
                else
                {
                    //
                    // Check if block is yet to be assigned to partition B.
                    //
                    if(partitionBflag == 0U)
                    {
                        //
                        // Assign block to partition B.
                        //
                        McBSP_setTxTwoPartitionBlock(base,
                                                  (McBSP_PartitionBlock)block);

                        //
                        // Set flag to indicate that a block is now assigned
                        // to partition B. Only one block can be assigned to
                        // a partition at a time in 2 partition mode.
                        //
                        partitionBflag = 1U;
                        partitionBblock = block;
                    }

                    //
                    // Check if the channel to be enabled belong to the block
                    // assigned to partition B.
                    //
                    if(partitionBblock == block)
                    {
                        //
                        // Enable the channel belonging to the block assigned
                        // to partition B.
                        //
                        McBSP_enableTxChannel(base,
                                              MCBSP_MULTICHANNEL_TWO_PARTITION,
                               (uint16_t)(*((ptrMchnParams->ptrChannelsListTx)
                                              + index)));
                    }
                    else
                    {
                        errorTx |= MCBSP_ERROR_2_PARTITION_B;
                    }
                }
            }
        }

        //
        // Multichannel configuration for 8 partition mode.
        //
        else
        {
            if((uint16_t)ptrMchnParams->channelCountTx > 128U)
            {
                errorTx = MCBSP_ERROR_EXCEEDED_CHANNELS;
            }
            for(index = 0U; index < (uint16_t)ptrMchnParams->channelCountTx;
                index++)
            {
                //
                // Enable the Tx channels.
                //
                McBSP_enableTxChannel(base, MCBSP_MULTICHANNEL_EIGHT_PARTITION,
                               (uint16_t)(*((ptrMchnParams->ptrChannelsListTx)
                                      + index)));
            }
        }
    }
    return(errorTx);
}

//*****************************************************************************
//
// McBSP_configureRxMultichannel
//
//*****************************************************************************
uint16_t
McBSP_configureRxMultichannel(uint32_t base,
                              const McBSP_RxMultichannelParams *ptrMchnParams)
{
    uint16_t index;
    uint16_t block;
    uint16_t partitionAblock;
    uint16_t partitionBblock;
    uint16_t partitionAflag;
    uint16_t partitionBflag;
    uint16_t errorRx;

    errorRx = 0U;
    partitionAblock = 0U;
    partitionBblock = 0U;

    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Configure Tx Channel Selection mode.
    //
    McBSP_setRxChannelMode(base,
                       (McBSP_RxChannelMode)ptrMchnParams->multichannelModeRx);

    //
    // Select 2 partition or 8 partition.
    //
    McBSP_setRxMultichannelPartition(base,
                      (McBSP_MultichannelPartition)ptrMchnParams->partitionRx);

    //
    // Configuration for multichannel selections that is for
    // MCBSP_RX_CHANNEL_SELECTION_ENABLED.
    //
    if((ptrMchnParams->multichannelModeRx) ==
       MCBSP_RX_CHANNEL_SELECTION_ENABLED)
    {
        //
        // Disable dual phase reception mode.
        //
        McBSP_disableTwoPhaseRx(base);

        //
        // Multichannel configuration for 2 partition mode.
        //
        if((McBSP_MultichannelPartition)ptrMchnParams->partitionRx ==
            MCBSP_MULTICHANNEL_TWO_PARTITION)
        {
            if((uint16_t)ptrMchnParams->channelCountRx > 32U)
            {
                errorRx = MCBSP_ERROR_EXCEEDED_CHANNELS;
            }
            partitionAflag = 0U;
            partitionBflag = 0U;

            //
            // Assign blocks to partition for the provided channels and
            // enable the channels. Only the channels which belong to the
            // block currently assigned to partition A or B can be enabled.
            //
            for(index = 0U; index < (uint16_t)ptrMchnParams->channelCountRx;
                index++)
            {
                //
                // Get the block to which channel belongs.
                //
                block = (*((ptrMchnParams->ptrChannelsListRx) + index)) >> 4U;

                //
                // Check if channel block can be assigned to partition A. Only
                // even numbered blocks can be assigned to partition A.
                //
                if((block & 0x1U) == 0U)
                {
                    //
                    // Check if block is yet to be assigned to partition A.
                    //
                    if(partitionAflag == 0U)
                    {
                        //
                        // Assign block to partition A.
                        //
                        McBSP_setRxTwoPartitionBlock(base,
                                                  (McBSP_PartitionBlock)block);

                        //
                        // Set flag to indicate that a block is now assigned
                        // to partition A. Only one block can be assigned to
                        // a partition at a time in 2 partition mode.
                        //
                        partitionAflag = 1U;
                        partitionAblock = block;
                    }

                    //
                    // Check if the channel to be enabled belong to the block
                    // assigned to partition A.
                    //
                    if(partitionAblock == block)
                    {
                        //
                        // Enable the channel belonging to the block assigned
                        // to partition A.
                        //
                        McBSP_enableRxChannel(base,
                                              MCBSP_MULTICHANNEL_TWO_PARTITION,
                              (uint16_t)(*((ptrMchnParams->ptrChannelsListRx)
                                              + index)));
                    }
                    else
                    {
                        errorRx = MCBSP_ERROR_2_PARTITION_A;
                    }
                }

                //
                // Check if channel block can be assigned to partition B. Only
                // odd numbered blocks can be assigned to partition B.
                //
                else
                {
                    //
                    // Check if block is yet to be assigned to partition B.
                    //
                    if(partitionBflag == 0U)
                    {
                        //
                        // Assign block to partition B.
                        //
                        McBSP_setRxTwoPartitionBlock(base,
                                                  (McBSP_PartitionBlock)block);
                        //
                        // Set flag to indicate that a block is now assigned
                        // to partition B. Only one block can be assigned to
                        // a partition at a time in 2 partition mode.
                        //
                        partitionBflag = 1U;
                        partitionBblock = block;
                    }

                    //
                    // Check if the channel to be enabled belong to the block
                    // assigned to partition B.
                    //
                    if(partitionBblock == block)
                    {
                        //
                        // Enable the Rx channel belonging to the block
                        // assigned to partition B.
                        //
                        McBSP_enableRxChannel(base,
                                              MCBSP_MULTICHANNEL_TWO_PARTITION,
                              (uint16_t)(*((ptrMchnParams->ptrChannelsListRx)
                                              + index)));
                    }
                    else
                    {
                        errorRx |= MCBSP_ERROR_2_PARTITION_B;
                    }
                }
            }
        }

        //
        // Multichannel configuration for 8 partition mode.
        //
        else
        {
            if(ptrMchnParams->channelCountRx > 128U)
            {
                errorRx = MCBSP_ERROR_EXCEEDED_CHANNELS;
            }
            for(index = 0U; index < (uint16_t)ptrMchnParams->channelCountRx;
                index++)
            {
                //
                // Enable the Rx channels.
                //
                McBSP_enableRxChannel(base,
                                      MCBSP_MULTICHANNEL_EIGHT_PARTITION,
                   (uint16_t)(*((ptrMchnParams->ptrChannelsListRx) + index)));
            }
        }
    }
    return(errorRx);
}

//*****************************************************************************
//
// McBSP_configureSPIMasterMode
//
//*****************************************************************************
void
McBSP_configureSPIMasterMode(uint32_t base,
                         const McBSP_SPIMasterModeParams *ptrSPIMasterMode)
{
    //
    // Configure clock stop mode.
    //
    if(((ptrSPIMasterMode->clockStopMode) == MCBSP_CLOCK_SPI_MODE_NO_DELAY) ||
       ((ptrSPIMasterMode->clockStopMode) == MCBSP_CLOCK_SPI_MODE_DELAY))
    {
        //
        // Set SPI mode.
        //
        McBSP_setClockStopMode(base,
                         (McBSP_ClockStopMode)ptrSPIMasterMode->clockStopMode);

        //
        // Set loop back mode.
        //
        if(ptrSPIMasterMode->loopbackModeFlag)
        {
            McBSP_enableLoopback(base);
        }
        else
        {
            McBSP_disableLoopback(base);
        }

        //
        // Configure module as master. Use SRG as clock source for driving
        // master clock. MCLKX pin will be the master clock out pin.
        //
        McBSP_setTxClockSource(base, MCBSP_INTERNAL_TX_CLOCK_SOURCE);

        //
        // Set internal clock (LSPCLK) as SRG clock source.
        //
        McBSP_setTxSRGClockSource(base, MCBSP_SRG_TX_CLOCK_SOURCE_LSPCLK);

        //
        // Set SRG clock divider for generating CLKG.
        //
        McBSP_setSRGDataClockDivider(base,
                                  (uint16_t)ptrSPIMasterMode->clockSRGDivider);

        //
        // Set the output master clock polarity.
        //
        McBSP_setTxClockPolarity(base,
                             (McBSP_TxClockPolarity)ptrSPIMasterMode->spiMode);

        //
        // Set FSX as an output driven by SRG.
        //
        McBSP_setTxFrameSyncSource(base, MCBSP_TX_INTERNAL_FRAME_SYNC_SOURCE);

        //
        // FSX is triggered when data is written to DXR registers.
        //
        McBSP_setTxInternalFrameSyncSource(base,
                                           MCBSP_TX_INTERNAL_FRAME_SYNC_DATA);

        //
        // Set the polarity for FSX pin as active low.
        //
        McBSP_setTxFrameSyncPolarity(base,
                                     MCBSP_TX_FRAME_SYNC_POLARITY_LOW);

        //
        // Disable dual phase mode.
        //
        McBSP_disableTwoPhaseTx(base);

        //
        // Set the data format for transmission & reception.
        //
        McBSP_setTxDataSize(base, MCBSP_PHASE_ONE_FRAME,
                      (McBSP_DataBitsPerWord)ptrSPIMasterMode->wordLength, 0U);
        McBSP_setRxDataSize(base, MCBSP_PHASE_ONE_FRAME,
                      (McBSP_DataBitsPerWord)ptrSPIMasterMode->wordLength, 0U);

        //
        // Set one bit data delay for transmission & reception to set correct
        // setup time on FSX signal.
        //
        McBSP_setTxDataDelayBits(base, MCBSP_DATA_DELAY_BIT_1);
        McBSP_setRxDataDelayBits(base, MCBSP_DATA_DELAY_BIT_1);
    }
}

//*****************************************************************************
//
// McBSP_configureSPISlaveMode
//
//*****************************************************************************
void
McBSP_configureSPISlaveMode(uint32_t base,
                            const McBSP_SPISlaveModeParams *ptrSPISlaveMode)
{

    //
    // Configure clock stop mode.
    //
    if(((ptrSPISlaveMode->clockStopMode) == MCBSP_CLOCK_SPI_MODE_NO_DELAY) ||
       ((ptrSPISlaveMode->clockStopMode) == MCBSP_CLOCK_SPI_MODE_DELAY))
    {
        //
        // Set SPI mode.
        //
        McBSP_setClockStopMode(base,
                          (McBSP_ClockStopMode)ptrSPISlaveMode->clockStopMode);
        //
        // Set loop back mode.
        //
        if(ptrSPISlaveMode->loopbackModeFlag)
        {
            McBSP_enableLoopback(base);
        }
        else
        {
            McBSP_disableLoopback(base);
        }

        //
        // Configure module as Slave. MCLKX pin acts as input slave
        // clock and is driven externally by SPI master.
        //
        McBSP_setTxClockSource(base, MCBSP_EXTERNAL_TX_CLOCK_SOURCE);

        //
        // Set the input slave clock polarity.
        //
        McBSP_setTxClockPolarity(base,
                              (McBSP_TxClockPolarity)ptrSPISlaveMode->spiMode);

        //
        // Set internal clock (LSPCLK) as SRG clock source. SRG is used to
        // synchronize McBSP logic with externally generated master clock.
        //
        McBSP_setRxSRGClockSource(base, MCBSP_SRG_RX_CLOCK_SOURCE_LSPCLK);

        //
        // Assign a clock divider value of 1 for generating CLKG.
        //
        McBSP_setSRGDataClockDivider(base, 1U);

        //
        // Set FSX as an input which is driven by slave-enable signal
        // from SPI master.
        //
        McBSP_setTxFrameSyncSource(base, MCBSP_TX_EXTERNAL_FRAME_SYNC_SOURCE);

        //
        // Set the polarity for FSX pin as active low.
        //
        McBSP_setTxFrameSyncPolarity(base,
                                     MCBSP_TX_FRAME_SYNC_POLARITY_LOW);

        //
        // Disable dual phase mode.
        //
        McBSP_disableTwoPhaseTx(base);

        //
        // Set the data format for transmission & reception..
        //
        McBSP_setTxDataSize(base, MCBSP_PHASE_ONE_FRAME,
                       (McBSP_DataBitsPerWord)ptrSPISlaveMode->wordLength, 0U);
        McBSP_setRxDataSize(base, MCBSP_PHASE_ONE_FRAME,
                       (McBSP_DataBitsPerWord)ptrSPISlaveMode->wordLength, 0U);

        //
        // Set zero bit data delay for transmission & reception.
        //
        McBSP_setTxDataDelayBits(base, MCBSP_DATA_DELAY_BIT_0);
        McBSP_setRxDataDelayBits(base, MCBSP_DATA_DELAY_BIT_0);
    }
}
