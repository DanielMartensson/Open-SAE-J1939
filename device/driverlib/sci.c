//###########################################################################
//
// FILE:   sci.c
//
// TITLE:  C28x SCI driver.
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

#include "sci.h"

//*****************************************************************************
//
// SCI_setConfig
//
//*****************************************************************************
void
SCI_setConfig(uint32_t base, uint32_t lspclkHz, uint32_t baud, uint32_t config)
{
    uint32_t divider;

    //
    // Check the arguments.
    // Is the required baud rate greater than the maximum rate supported?
    //
    ASSERT(SCI_isBaseValid(base));
    ASSERT(baud != 0U);
    ASSERT((baud * 16U) <= lspclkHz);

    //
    // Stop the SCI.
    //
    SCI_disableModule(base);

    //
    // Compute the baud rate divider.
    //
    divider = ((lspclkHz  / (baud * 8U)) - 1U);

    //
    // Set the baud rate.
    //
    HWREGH(base + SCI_O_HBAUD) = (divider & 0xFF00U) >> 8U;
    HWREGH(base + SCI_O_LBAUD) = divider & 0x00FFU;

    //
    // Set parity, data length, and number of stop bits.
    //
    HWREGH(base + SCI_O_CCR) = ((HWREGH(base + SCI_O_CCR) &
                                 ~(SCI_CONFIG_PAR_MASK |
                                   SCI_CONFIG_STOP_MASK |
                                   SCI_CONFIG_WLEN_MASK)) | config);

    //
    // Start the SCI.
    //
    SCI_enableModule(base);
}

//*****************************************************************************
//
// SCI_writeCharArray
//
//*****************************************************************************
void
SCI_writeCharArray(uint32_t base, const uint16_t * const array,
                   uint16_t length)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    uint16_t i;
    //
    // Check if FIFO enhancement is enabled.
    //
    if(SCI_isFIFOEnabled(base))
    {
        //
        // FIFO is enabled.
        // For loop to write (Blocking) 'length' number of characters
        //
        for(i = 0U; i < length; i++)
        {
            //
            // Wait until space is available in the transmit FIFO.
            //
            while(SCI_getTxFIFOStatus(base) == SCI_FIFO_TX15)
            {
            }

            //
            // Send a char.
            //
            HWREGH(base + SCI_O_TXBUF) = array[i];
        }
    }
    else
    {
        //
        // FIFO is not enabled.
        // For loop to write (Blocking) 'length' number of characters
        //
        for(i = 0U; i < length; i++)
        {
            //
            // Wait until space is available in the transmit buffer.
            //
            while(!SCI_isSpaceAvailableNonFIFO(base))
            {
            }

            //
            // Send a char.
            //
            HWREGH(base + SCI_O_TXBUF) = array[i];
        }
    }
}

//*****************************************************************************
//
// SCI_readCharArray
//
//*****************************************************************************
void
SCI_readCharArray(uint32_t base, uint16_t * const array, uint16_t length)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    uint16_t i;
    //
    // Check if FIFO enhancement is enabled.
    //
    if(SCI_isFIFOEnabled(base))
    {
        //
        // FIFO is enabled.
        // For loop to read (Blocking) 'length' number of characters
        //
        for(i = 0U; i < length; i++)
        {
            //
            // Wait until a character is available in the receive FIFO.
            //
            while(SCI_getRxFIFOStatus(base) == SCI_FIFO_RX0)
            {
            }

            //
            // Return the character from the receive buffer.
            //
            array[i] = (uint16_t)
                       (HWREGH(base + SCI_O_RXBUF) & SCI_RXBUF_SAR_M);
        }
    }
    else
    {
        //
        // FIFO is not enabled.
        // For loop to read (Blocking) 'length' number of characters
        //
        for(i = 0U; i < length; i++)
        {
            //
            // Wait until a character is available in the receive buffer.
            //
            while(!SCI_isDataAvailableNonFIFO(base))
            {
            }

            //
            // Return the character from the receive buffer.
            //
            array[i] = (uint16_t)
                       (HWREGH(base + SCI_O_RXBUF) & SCI_RXBUF_SAR_M);
        }
    }
}

//*****************************************************************************
//
// SCI_enableInterrupt
//
//*****************************************************************************
void
SCI_enableInterrupt(uint32_t base, uint32_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Enable the specified interrupts.
    //
    if((intFlags & SCI_INT_RXERR) == SCI_INT_RXERR)
    {
        HWREGH(base + SCI_O_CTL1) |= SCI_CTL1_RXERRINTENA;
    }
    if((intFlags & SCI_INT_RXRDY_BRKDT) == SCI_INT_RXRDY_BRKDT)
    {
        HWREGH(base + SCI_O_CTL2) |= SCI_CTL2_RXBKINTENA;
    }
    if((intFlags & SCI_INT_TXRDY) == SCI_INT_TXRDY)
    {
        HWREGH(base + SCI_O_CTL2) |= SCI_CTL2_TXINTENA;
    }
    if((intFlags & SCI_INT_TXFF) == SCI_INT_TXFF)
    {
        HWREGH(base + SCI_O_FFTX) |= SCI_FFTX_TXFFIENA;
    }
    if((intFlags & SCI_INT_RXFF) == SCI_INT_RXFF)
    {
        HWREGH(base + SCI_O_FFRX) |= SCI_FFRX_RXFFIENA;
    }
}

//*****************************************************************************
//
// SCI_disableInterrupt
//
//*****************************************************************************
void
SCI_disableInterrupt(uint32_t base, uint32_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Disable the specified interrupts.
    //
    if((intFlags & SCI_INT_RXERR) == SCI_INT_RXERR)
    {
        HWREGH(base + SCI_O_CTL1) &= ~SCI_CTL1_RXERRINTENA;
    }
    if((intFlags & SCI_INT_RXRDY_BRKDT) == SCI_INT_RXRDY_BRKDT)
    {
        HWREGH(base + SCI_O_CTL2) &= ~SCI_CTL2_RXBKINTENA;
    }
    if((intFlags & SCI_INT_TXRDY) == SCI_INT_TXRDY)
    {
        HWREGH(base + SCI_O_CTL2) &= ~SCI_CTL2_TXINTENA;
    }
    if((intFlags & SCI_INT_TXFF) == SCI_INT_TXFF)
    {
        HWREGH(base + SCI_O_FFTX) &= ~SCI_FFTX_TXFFIENA;
    }
    if((intFlags & SCI_INT_RXFF) == SCI_INT_RXFF)
    {
        HWREGH(base + SCI_O_FFRX) &= ~SCI_FFRX_RXFFIENA;
    }
}

//*****************************************************************************
//
// SCI_getInterruptStatus
//
//*****************************************************************************
uint32_t
SCI_getInterruptStatus(uint32_t base)
{
    uint32_t interruptStatus = 0;

    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Return the interrupt status.
    //
    if((HWREGH(base + SCI_O_CTL2) & SCI_CTL2_TXRDY) == SCI_CTL2_TXRDY)
    {
        interruptStatus |= SCI_INT_TXRDY;
    }
    if((HWREGH(base + SCI_O_RXST) & SCI_RXST_RXERROR) == SCI_RXST_RXERROR)
    {
        interruptStatus |= SCI_INT_RXERR;
    }
    if(((HWREGH(base + SCI_O_RXST) & SCI_RXST_RXRDY) == SCI_RXST_RXRDY)  ||
       ((HWREGH(base + SCI_O_RXST) & SCI_RXST_BRKDT) ==  SCI_RXST_BRKDT))
    {
        interruptStatus |= SCI_INT_RXRDY_BRKDT;
    }
    if((HWREGH(base + SCI_O_FFTX) & SCI_FFTX_TXFFINT) == SCI_FFTX_TXFFINT)
    {
        interruptStatus |= SCI_INT_TXFF;
    }
    if((HWREGH(base + SCI_O_FFRX) & SCI_FFRX_RXFFINT) == SCI_FFRX_RXFFINT)
    {
        interruptStatus |= SCI_INT_RXFF;
    }
    if((HWREGH(base + SCI_O_RXST) & SCI_RXST_FE) == SCI_RXST_FE)
    {
        interruptStatus |= SCI_INT_FE;
    }
    if((HWREGH(base + SCI_O_RXST) & SCI_RXST_OE) == SCI_RXST_OE)
    {
        interruptStatus |= SCI_INT_OE;
    }
    if((HWREGH(base + SCI_O_RXST) & SCI_RXST_PE) == SCI_RXST_PE)
    {
        interruptStatus |= SCI_INT_PE;
    }

    return(interruptStatus);
}

//*****************************************************************************
//
// SCI_clearInterruptStatus
//
//*****************************************************************************
void
SCI_clearInterruptStatus(uint32_t base, uint32_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Clear the requested interrupt sources.
    //
    if(((intFlags & SCI_INT_RXERR) == SCI_INT_RXERR) ||
       ((intFlags & SCI_INT_RXRDY_BRKDT) == SCI_INT_RXRDY_BRKDT) ||
       ((intFlags & SCI_INT_FE) == SCI_INT_FE) ||
       ((intFlags & SCI_INT_OE) == SCI_INT_OE) ||
       ((intFlags & SCI_INT_PE) == SCI_INT_PE))
    {
        SCI_performSoftwareReset(base);
    }
    if((intFlags & SCI_INT_TXFF) == SCI_INT_TXFF)
    {
        HWREGH(base + SCI_O_FFTX) |= SCI_FFTX_TXFFINTCLR;
    }
    if((intFlags & SCI_INT_RXFF) == SCI_INT_RXFF)
    {
         HWREGH(base + SCI_O_FFRX) |= SCI_FFRX_RXFFINTCLR;
    }
}

//*****************************************************************************
//
// SCI_setBaud
//
//*****************************************************************************
void SCI_setBaud(uint32_t base, uint32_t lspclkHz, uint32_t baud)
{
    uint32_t divider;

    //
    // Compute the baud rate divider {ROUND TO NEAREST INTEGER}
    //
    divider = ((float)((float)lspclkHz / ((float)baud * 8.0F)) - 1.0F) + 0.5F;

    //
    // Set the baud rate.
    //
    HWREGH(base + SCI_O_HBAUD) = (divider & 0xFF00U) >> 8U;
    HWREGH(base + SCI_O_LBAUD) = divider & 0x00FFU;
}

//*****************************************************************************
//
// SCI_setWakeFlag
//
//*****************************************************************************
void SCI_setWakeFlag(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Set the TX wake flag bit to indicate
    // that the next frame is an address frame.
    //
    HWREGH(base + SCI_O_CTL1) |= SCI_CTL1_TXWAKE;
}
