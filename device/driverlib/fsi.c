//###########################################################################
//
// FILE:   fsi.c
//
// TITLE:  C28x FSI driver.
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

#include "fsi.h"

//*****************************************************************************
//
//! \internal
//!
//! Provides delay based on requested count. Needed while performing reset and
//! flush sequence, sufficient delay ensures reliability in operation.
//
//*****************************************************************************
static void FSI_delayWait(uint32_t n)
{
    //
    // Taking local temp counter to avoid MISRAC:2012 rule 17.8 which states not
    // to modify parameter passed to a function call.
    //
    uint32_t temp;
    for(temp = n; temp > 0U; temp--)
    {
    }
}

//*****************************************************************************
//
// FSI_resetTxModule
//
//*****************************************************************************
void
FSI_resetTxModule(uint32_t base, FSI_TxSubmoduleInReset submodule)
{
    //
    // Check the arguments.
    //
    ASSERT(FSI_isTxBaseValid(base));

    EALLOW;
    switch(submodule)
    {
        case FSI_TX_MAIN_CORE_RESET:
            HWREGH(base + FSI_O_TX_MAIN_CTRL) =
                            ((uint16_t)FSI_TX_MAIN_CTRL_CORE_RST |
                             (FSI_CTRL_REG_KEY << FSI_TX_MAIN_CTRL_KEY_S));
            break;

        case FSI_TX_CLOCK_RESET:
            HWREGH(base + FSI_O_TX_CLK_CTRL) |= FSI_TX_CLK_CTRL_CLK_RST;
            break;

        case FSI_TX_PING_TIMEOUT_CNT_RESET:
            HWREGH(base + FSI_O_TX_PING_CTRL) |= FSI_TX_PING_CTRL_CNT_RST;
            break;

        default:
            //
            // Invalid submodule input
            //
            ASSERT(false);
            break;
    }
    EDIS;
}

//*****************************************************************************
//
// FSI_clearTxModuleReset
//
//*****************************************************************************
void
FSI_clearTxModuleReset(uint32_t base, FSI_TxSubmoduleInReset submodule)
{
    //
    // Check the arguments.
    //
    ASSERT(FSI_isTxBaseValid(base));

    EALLOW;
    switch(submodule)
    {
        //
        // Key value must be written into main control register and ensure
        // that FLUSH pattern is not sent while doing Tx core reset
        //
        case FSI_TX_MAIN_CORE_RESET:
            HWREGH(base + FSI_O_TX_MAIN_CTRL) =
                              (HWREGH(base + FSI_O_TX_MAIN_CTRL) &
                               (~FSI_TX_MAIN_CTRL_CORE_RST)) |
                              (FSI_CTRL_REG_KEY << FSI_TX_MAIN_CTRL_KEY_S);
            break;

        case FSI_TX_CLOCK_RESET:
            HWREGH(base + FSI_O_TX_CLK_CTRL) &= ~FSI_TX_CLK_CTRL_CLK_RST;
            break;

        case FSI_TX_PING_TIMEOUT_CNT_RESET:
            HWREGH(base + FSI_O_TX_PING_CTRL) &= ~FSI_TX_PING_CTRL_CNT_RST;
            break;

        default:
            //
            // Invalid submodule input
            //
            ASSERT(false);
            break;
    }
    EDIS;
}

//*****************************************************************************
//
// FSI_writeTxBuffer
//
//*****************************************************************************
void
FSI_writeTxBuffer(uint32_t base, const uint16_t array[], uint16_t length,
                  uint16_t bufOffset)
{
    //
    // Check the arguments.
    //
    ASSERT(FSI_isTxBaseValid(base));
    ASSERT((length - 1U) <= FSI_MAX_VALUE_BUF_PTR_OFF);
    ASSERT(bufOffset <= FSI_MAX_VALUE_BUF_PTR_OFF);

    uint16_t i = 0U, j, tempBufOffset = bufOffset;

    for(j = 0U; j < length; j++)
    {
        //
        // Check for wrap around in case more words to be written in buffer
        // when last write happened at maximum offset in Tx buffer
        //
        if((tempBufOffset + i) > FSI_MAX_VALUE_BUF_PTR_OFF)
        {
            tempBufOffset = 0U;
            i = 0U;
        }

        //
        // Write one 16 bit word, increment buffer pointer
        //
        HWREGH(base + FSI_O_TX_BUF_BASE(tempBufOffset) + i) = array[j];
        i++;
    }
}

//*****************************************************************************
//
// FSI_resetRxModule
//
//*****************************************************************************
void
FSI_resetRxModule(uint32_t base, FSI_RxSubmoduleInReset submodule)
{
    //
    // Check the arguments.
    //
    ASSERT(FSI_isRxBaseValid(base));

    EALLOW;
    switch(submodule)
    {
        case FSI_RX_MAIN_CORE_RESET:
            HWREGH(base + FSI_O_RX_MAIN_CTRL) |=
                         ((uint16_t)FSI_RX_MAIN_CTRL_CORE_RST |
                          (FSI_CTRL_REG_KEY << FSI_RX_MAIN_CTRL_KEY_S));
            break;

        case FSI_RX_FRAME_WD_CNT_RESET:
            HWREGH(base + FSI_O_RX_FRAME_WD_CTRL) |=
                                    FSI_RX_FRAME_WD_CTRL_FRAME_WD_CNT_RST;
            break;

        case FSI_RX_PING_WD_CNT_RESET:
            HWREGH(base + FSI_O_RX_PING_WD_CTRL) |=
                                    FSI_RX_PING_WD_CTRL_PING_WD_RST;
            break;

        default:
            //
            // Invalid submodule input
            //
            ASSERT(false);
            break;
    }
    EDIS;
}

//*****************************************************************************
//
// FSI_clearRxModuleReset
//
//*****************************************************************************
void
FSI_clearRxModuleReset(uint32_t base, FSI_RxSubmoduleInReset submodule)
{
    //
    // Check the arguments.
    //
    ASSERT(FSI_isRxBaseValid(base));

    EALLOW;
    switch(submodule)
    {
        case FSI_RX_MAIN_CORE_RESET:
            HWREGH(base + FSI_O_RX_MAIN_CTRL) =
                            (HWREGH(base + FSI_O_RX_MAIN_CTRL) &
                             (~FSI_RX_MAIN_CTRL_CORE_RST)) |
                            (FSI_CTRL_REG_KEY << FSI_RX_MAIN_CTRL_KEY_S);
            break;

        case FSI_RX_FRAME_WD_CNT_RESET:
            HWREGH(base + FSI_O_RX_FRAME_WD_CTRL) &=
                                        ~FSI_RX_FRAME_WD_CTRL_FRAME_WD_CNT_RST;
            break;

        case FSI_RX_PING_WD_CNT_RESET:
            HWREGH(base + FSI_O_RX_PING_WD_CTRL) &=
                                        ~FSI_RX_PING_WD_CTRL_PING_WD_RST;
            break;

        default:
            //
            // Invalid submodule input
            //
            ASSERT(false);
            break;
    }
    EDIS;
}

//*****************************************************************************
//
// FSI_readRxBuffer
//
//*****************************************************************************
void
FSI_readRxBuffer(uint32_t base, uint16_t array[], uint16_t length,
                 uint16_t bufOffset)
{
    //
    // Check the arguments.
    //
    ASSERT(FSI_isRxBaseValid(base));
    ASSERT((length - 1U) <= FSI_MAX_VALUE_BUF_PTR_OFF);
    ASSERT(bufOffset <= FSI_MAX_VALUE_BUF_PTR_OFF);

    uint16_t i = 0U, j, tempBufOffset = bufOffset;

    for(j = 0U; j < length; j++)
    {
        //
        // Check for wrap around in case more words to be read in buffer when
        // last read happened at maximum offset in Rx buffer
        //
        if((tempBufOffset + i) > FSI_MAX_VALUE_BUF_PTR_OFF)
        {
            tempBufOffset = 0U;
            i = 0U;
        }

        //
        // Read one 16 bit word, increment buffer pointer
        //
        array[j] = HWREGH(base + FSI_O_RX_BUF_BASE(tempBufOffset) + i);
        i++;
     }
}

//*****************************************************************************
//
// FSI_configRxDelayLine
//
//*****************************************************************************
void
FSI_configRxDelayLine(uint32_t base, FSI_RxDelayTapType delayTapType,
                      uint16_t tapValue)
{
    //
    // Check the arguments.
    //
    ASSERT(FSI_isRxBaseValid(base));
    ASSERT(tapValue <= FSI_RX_MAX_DELAY_LINE_VAL);

    EALLOW;
    switch(delayTapType)
    {
        case FSI_RX_DELAY_CLK:
            HWREGH(base + FSI_O_RX_DLYLINE_CTRL) =
                            (HWREGH(base + FSI_O_RX_DLYLINE_CTRL) &
                             (~FSI_RX_DLYLINE_CTRL_RXCLK_DLY_M)) |
                            (tapValue << FSI_RX_DLYLINE_CTRL_RXCLK_DLY_S);
            break;

        case FSI_RX_DELAY_D0:
            HWREGH(base + FSI_O_RX_DLYLINE_CTRL) =
                            (HWREGH(base + FSI_O_RX_DLYLINE_CTRL) &
                             (~FSI_RX_DLYLINE_CTRL_RXD0_DLY_M)) |
                            (tapValue << FSI_RX_DLYLINE_CTRL_RXD0_DLY_S);
            break;

        case FSI_RX_DELAY_D1:
            HWREGH(base + FSI_O_RX_DLYLINE_CTRL) =
                            (HWREGH(base + FSI_O_RX_DLYLINE_CTRL) &
                             (~FSI_RX_DLYLINE_CTRL_RXD1_DLY_M)) |
                            (tapValue << FSI_RX_DLYLINE_CTRL_RXD1_DLY_S);
            break;

        default:
            //
            // Invalid tap selection input
            //
            ASSERT(false);
            break;
    }
    EDIS;
}

//*****************************************************************************
//
// FSI_performTxInitialization
//
//*****************************************************************************
void FSI_performTxInitialization(uint32_t base, uint16_t prescalar)
{
    //
    // Check the arguments.
    //
    ASSERT(FSI_isTxBaseValid(base));
    ASSERT(prescalar <= 0xFFU );

    //
    // Reset the Tx core
    //
    FSI_resetTxModule(base, FSI_TX_MAIN_CORE_RESET);

    //
    // Select PLL clock as source for clock dividers
    //
    FSI_selectTxPLLClock(base);

    //
    // Sequence is :: Reset clock divider -> Set prescalar value
    // in reset -> Wait -> Enable clock -> Release reset
    //
    FSI_resetTxModule(base, FSI_TX_CLOCK_RESET);
    FSI_configPrescalar(base, prescalar);
    FSI_delayWait(1);
    FSI_enableTxClock(base);
    FSI_clearTxModuleReset(base, FSI_TX_CLOCK_RESET);

    //
    // Wait sufficient time before releasing reset of Tx core
    //
    FSI_delayWait(prescalar);
    FSI_clearTxEvents(base, FSI_TX_EVTMASK);
    FSI_clearTxModuleReset(base, FSI_TX_MAIN_CORE_RESET);
    FSI_delayWait(2U * prescalar);
}

//*****************************************************************************
//
// FSI_performRxInitialization
//
//*****************************************************************************
void FSI_performRxInitialization(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(FSI_isRxBaseValid(base));

    FSI_resetRxModule(base, FSI_RX_MAIN_CORE_RESET);

    FSI_clearRxEvents(base, FSI_RX_EVTMASK);

    //
    // It is important to ensure that there is no active clocks on RXCLK before
    // releasing the reset. This can be accomplished by handshaking or by using
    // the chip level connectivity options to ensure that no receive clocks are
    // seen till the reset is released.
    // FSI Rx module will come out of reset only when the receive clock is sent
    // in or by sending a Flush pattern from FSI Tx.
    //
    FSI_delayWait(1);
    FSI_clearRxModuleReset(base, FSI_RX_MAIN_CORE_RESET);
}

//*****************************************************************************
//
// FSI_executeTxFlushSequence
//
//*****************************************************************************
void FSI_executeTxFlushSequence(uint32_t base, uint16_t prescalar)
{
    //
    // Check the arguments.
    //
    ASSERT(FSI_isTxBaseValid(base));
    ASSERT(prescalar <= 0xFFU );

    FSI_sendTxFlush(base);

    //
    // Inserting delay as recommended in TRM, It ensures that the transmit
    // module core sees sufficient TXCLK cycles before stopping FLUSH pattern.
    //
    FSI_delayWait(2U * prescalar);

    FSI_stopTxFlush(base);
}
