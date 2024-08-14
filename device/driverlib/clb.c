//###########################################################################
//
// FILE:   clb.c
//
// TITLE:  C28x CLB driver.
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

#include "clb.h"

//*****************************************************************************
//
// CLB_configCounterLoadMatch
//
//*****************************************************************************
void CLB_configCounterLoadMatch(uint32_t base, CLB_Counters counterID,
                               uint32_t load, uint32_t match1, uint32_t match2)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    switch(counterID)
    {
        case CLB_CTR0:
            CLB_writeInterface(base, CLB_ADDR_COUNTER_0_LOAD, load);
            CLB_writeInterface(base, CLB_ADDR_COUNTER_0_MATCH1, match1);
            CLB_writeInterface(base, CLB_ADDR_COUNTER_0_MATCH2, match2);
            break;

        case CLB_CTR1:
            CLB_writeInterface(base, CLB_ADDR_COUNTER_1_LOAD, load);
            CLB_writeInterface(base, CLB_ADDR_COUNTER_1_MATCH1, match1);
            CLB_writeInterface(base, CLB_ADDR_COUNTER_1_MATCH2, match2);
            break;

        case CLB_CTR2:
            CLB_writeInterface(base, CLB_ADDR_COUNTER_2_LOAD, load);
            CLB_writeInterface(base, CLB_ADDR_COUNTER_2_MATCH1, match1);
            CLB_writeInterface(base, CLB_ADDR_COUNTER_2_MATCH2, match2);
            break;

        default:
            //
            // Invalid counterID value
            //
            break;
    }
    EDIS;
}

//*****************************************************************************
//
// CLB_clearFIFOs
//
//*****************************************************************************
void CLB_clearFIFOs(uint32_t base)
{
    uint16_t i;

    ASSERT(CLB_isBaseValid(base));

    for(i = 0U; i < CLB_FIFO_SIZE; i++)
    {
        HWREG(base + CLB_DATAEXCH + CLB_O_PULL(i)) = 0U;
    }

    HWREG(base + CLB_LOGICCTL + CLB_O_BUF_PTR) = 0U;
}

//*****************************************************************************
//
// CLB_writeFIFOs
//
//*****************************************************************************
void CLB_writeFIFOs(uint32_t base , const uint32_t pullData[])
{
    ASSERT(CLB_isBaseValid(base));

    //
    // Clear the FIFO and pointer
    //
    CLB_clearFIFOs(base);

    //
    // Write data into the FIFO.
    //
    HWREG(base + CLB_DATAEXCH + CLB_O_PULL(0)) = pullData[0U];
    HWREG(base + CLB_DATAEXCH + CLB_O_PULL(1)) = pullData[1U];
    HWREG(base + CLB_DATAEXCH + CLB_O_PULL(2)) = pullData[2U];
    HWREG(base + CLB_DATAEXCH + CLB_O_PULL(3)) = pullData[3U];
}

//*****************************************************************************
//
// CLB_readFIFOs
//
//*****************************************************************************
void CLB_readFIFOs(uint32_t base , uint32_t pushData[])
{
    ASSERT(CLB_isBaseValid(base));

    //
    // Read data from the FIFO.
    //
    pushData[0U] = HWREG(base + CLB_DATAEXCH + CLB_O_PUSH(0)) ;
    pushData[1U] = HWREG(base + CLB_DATAEXCH + CLB_O_PUSH(1)) ;
    pushData[2U] = HWREG(base + CLB_DATAEXCH + CLB_O_PUSH(2)) ;
    pushData[3U] = HWREG(base + CLB_DATAEXCH + CLB_O_PUSH(3)) ;
}


