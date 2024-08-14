//###########################################################################
//
// FILE:   i2c.c
//
// TITLE:  C28x I2C driver.
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
#include "i2c.h"

//*****************************************************************************
//
// I2C_initController
//
//*****************************************************************************
void
I2C_initController(uint32_t base, uint32_t sysclkHz, uint32_t bitRate,
                   I2C_DutyCycle dutyCycle)
{
    uint32_t modPrescale;
    uint32_t divider;
    uint32_t dValue;

    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));
    ASSERT((10000000U / bitRate) >  10U);

    //
    // Set the prescaler for the module clock.
    //
    modPrescale = (sysclkHz / 10000000U) - 1U;
    HWREGH(base + I2C_O_PSC) = I2C_PSC_IPSC_M & modPrescale;

    switch(modPrescale)
    {
        case 0U:
            dValue = 7U;
            break;

        case 1U:
            dValue = 6U;
            break;

        default:
            dValue = 5U;
            break;
    }

    //
    // Set the divider for the time low
    //
    divider = (10000000U / bitRate) - (2U * dValue);

    if(dutyCycle == I2C_DUTYCYCLE_50)
    {
        HWREGH(base + I2C_O_CLKH) = divider / 2U;
    }
    else
    {
        HWREGH(base + I2C_O_CLKH) = divider / 3U;
    }

    HWREGH(base + I2C_O_CLKL) = divider - HWREGH(base + I2C_O_CLKH);
}

//*****************************************************************************
//
// I2C_enableInterrupt
//
//*****************************************************************************
void
I2C_enableInterrupt(uint32_t base, uint32_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Enable the desired basic interrupts
    //
    HWREGH(base + I2C_O_IER) |= (intFlags & 0x00FFU);

    //
    // Enabling addressed-as-target interrupt separately because its bit is
    // different between the IER and STR registers.
    //
    if((intFlags & I2C_INT_ADDR_TARGET) != 0U)
    {
        HWREGH(base + I2C_O_IER) |= I2C_IER_AAT;
    }

    //
    // Enable desired FIFO interrupts.
    //
    if((intFlags & I2C_INT_TXFF) != 0U)
    {
        HWREGH(base + I2C_O_FFTX) |= I2C_FFTX_TXFFIENA;
    }

    if((intFlags & I2C_INT_RXFF) != 0U)
    {
        HWREGH(base + I2C_O_FFRX) |= I2C_FFRX_RXFFIENA;
    }
}

//*****************************************************************************
//
// I2C_disableInterrupt
//
//*****************************************************************************
void
I2C_disableInterrupt(uint32_t base, uint32_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Disable the desired basic interrupts.
    //
    HWREGH(base + I2C_O_IER) &= ~(intFlags & 0x00FFU);

    //
    // Disabling addressed-as-target interrupt separately because its bit is
    // different between the IER and STR registers.
    //
    if((intFlags & I2C_INT_ADDR_TARGET) != 0U)
    {
        HWREGH(base + I2C_O_IER) &= ~I2C_IER_AAT;
    }

    //
    // Disable the desired FIFO interrupts.
    //
    if((intFlags & I2C_INT_TXFF) != 0U)
    {
        HWREGH(base + I2C_O_FFTX) &= ~(I2C_FFTX_TXFFIENA);
    }

    if((intFlags & I2C_INT_RXFF) != 0U)
    {
        HWREGH(base + I2C_O_FFRX) &= ~(I2C_FFRX_RXFFIENA);
    }
}

//*****************************************************************************
//
// I2C_getInterruptStatus
//
//*****************************************************************************
uint32_t
I2C_getInterruptStatus(uint32_t base)
{
    uint32_t temp;

    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Return only the status bits associated with interrupts.
    //
    temp = (uint32_t)(HWREGH(base + I2C_O_STR) & I2C_STR_INTMASK);

    //
    // Read FIFO interrupt flags.
    //
    if((HWREGH(base + I2C_O_FFTX) & I2C_FFTX_TXFFINT) != 0U)
    {
        temp |= I2C_INT_TXFF;
    }

    if((HWREGH(base + I2C_O_FFRX) & I2C_FFRX_RXFFINT) != 0U)
    {
        temp |= I2C_INT_RXFF;
    }

    return(temp);
}

//*****************************************************************************
//
// I2C_clearInterruptStatus
//
//*****************************************************************************
void
I2C_clearInterruptStatus(uint32_t base, uint32_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Clear the interrupt flags that are located in STR.
    //
    HWREGH(base + I2C_O_STR) = ((uint16_t)intFlags & I2C_STR_INTMASK);

    //
    // Clear the FIFO interrupt flags if needed.
    //
    if((intFlags & I2C_INT_TXFF) != 0U)
    {
        HWREGH(base + I2C_O_FFTX) |= I2C_FFTX_TXFFINTCLR;
    }

    if((intFlags & I2C_INT_RXFF) != 0U)
    {
        HWREGH(base + I2C_O_FFRX) |= I2C_FFRX_RXFFINTCLR;
    }
}
//*****************************************************************************
//
// I2C_configureModuleFrequency
//
//*****************************************************************************
void
I2C_configureModuleFrequency(uint32_t base, uint32_t sysclkHz)
{
    uint32_t modPrescale;

    //
    // Check the arguments.
    //
    ASSERT(I2C_isBaseValid(base));

    //
    // Set the prescaler for the module clock.
    //
    modPrescale = (sysclkHz / 10000000U) - 1U;
    HWREGH(base + I2C_O_PSC) = I2C_PSC_IPSC_M & modPrescale;
}
