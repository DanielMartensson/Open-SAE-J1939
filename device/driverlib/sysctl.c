//###########################################################################
//
// FILE:   sysctl.c
//
// TITLE:  C28x system control driver.
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

#include "cputimer.h"
#include "sysctl.h"
#include "dcc.h"

//
// Define to isolate inline assembly
//
#define SYSCTL_DELAY        __asm(" .if __TI_EABI__\n"                         \
                                  " .asg    SysCtl_delay    , _SysCtl_delay\n" \
                                  " .endif\n"                                  \
                                  " .def _SysCtl_delay\n"                      \
                                  " .sect \".TI.ramfunc\"\n"                   \
                                  " .global  _SysCtl_delay\n"                  \
                                  "_SysCtl_delay:\n"                           \
                                  " SUB    ACC,#1\n"                           \
                                  " BF     _SysCtl_delay, GEQ\n"               \
                                  " LRETR\n")


//
// Macro used for adding delay between 2 consecutive writes to CLKSRCCTL1
// register.
// Delay = 300 NOPs
//
#define SYSCTL_CLKSRCCTL1_DELAY  asm(" RPT #250 || NOP \n RPT #50 || NOP")

//*****************************************************************************
//
// SysCtl_delay()
//
//*****************************************************************************
SYSCTL_DELAY;


//*****************************************************************************
//
// SysCtl_pollX1Counter()
//
//*****************************************************************************
static void
SysCtl_pollX1Counter(void)
{
    uint16_t loopCount = 0U;

    //
    // Delay for 1 ms while the XTAL powers up
    //
    // 2000 loops, 5 cycles per loop + 9 cycles overhead = 10009 cycles
    //
    SysCtl_delay(2000);

    //
    // Clear and saturate X1CNT 4 times to guarantee operation
    //
    do
    {
        //
        // Keep clearing the counter until it is no longer saturated
        //
        while(HWREG(CLKCFG_BASE + SYSCTL_O_X1CNT) > 0x1FFU)
        {
            HWREG(CLKCFG_BASE + SYSCTL_O_X1CNT) |= SYSCTL_X1CNT_CLR;
            HWREG(CLKCFG_BASE + SYSCTL_O_X1CNT) &= ~SYSCTL_X1CNT_CLR;
        }

        //
        // Wait for the X1 clock to saturate
        //
        while(HWREGH(CLKCFG_BASE + SYSCTL_O_X1CNT) != SYSCTL_X1CNT_X1CNT_M)
        {
            //
            // If your application is stuck in this loop, please check if the
            // input clock source is valid.
            //
        }

        //
        // Increment the counter
        //
        loopCount++;
    }while(loopCount < 4U);
}

//*****************************************************************************
//
// SysCtl_getClock()
//
//*****************************************************************************
uint32_t
SysCtl_getClock(uint32_t clockInHz)
{
    uint32_t temp;
    uint32_t oscSource;
    uint32_t clockOut;

    //
    // Don't proceed if an MCD failure is detected.
    //
    if(SysCtl_isMCDClockFailureDetected())
    {
        //
        // OSCCLKSRC2 failure detected. Returning the INTOSC1 rate. You need
        // to handle the MCD and clear the failure.
        //
        clockOut = SYSCTL_DEFAULT_OSC_FREQ;
    }
    else
    {
        //
        // If one of the internal oscillators is being used, start from the
        // known default frequency.  Otherwise, use clockInHz parameter.
        //
        oscSource = HWREG(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &
                    (uint32_t)SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M;

        if((oscSource == (SYSCTL_OSCSRC_OSC2 >> SYSCTL_OSCSRC_S)) ||
           (oscSource == (SYSCTL_OSCSRC_OSC1 >> SYSCTL_OSCSRC_S)))
        {
            clockOut = SYSCTL_DEFAULT_OSC_FREQ;
        }
        else
        {
            clockOut = clockInHz;
        }

        //
        // If the PLL is enabled calculate its effect on the clock
        //
        if((HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &
            (SYSCTL_SYSPLLCTL1_PLLEN | SYSCTL_SYSPLLCTL1_PLLCLKEN)) == 3U)
        {
            //
            // Calculate integer multiplier
            //
            clockOut = clockOut * ((HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) &
                                    SYSCTL_SYSPLLMULT_IMULT_M) >>
                                   SYSCTL_SYSPLLMULT_IMULT_S);

            //
            // Calculate PLL divider
            //
            temp = ((((HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) &
                       SYSCTL_SYSPLLMULT_REFDIV_M) >>
                      SYSCTL_SYSPLLMULT_REFDIV_S) + 1U) *
                    (((HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) &
                       SYSCTL_SYSPLLMULT_ODIV_M) >>
                      SYSCTL_SYSPLLMULT_ODIV_S) + 1U));

            //
            //  Divide dividers
            //
            if(temp != 0U)
            {
                clockOut /= temp;
            }
        }

        if((HWREG(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
            SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M) != 0U)
        {
            clockOut /= (2U * (HWREG(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                               SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M));
        }
    }

    return(clockOut);
}

//*****************************************************************************
//
// SysCtl_getAuxClock()
//
//*****************************************************************************
uint32_t SysCtl_getAuxClock(uint32_t clockInHz)
{
    uint32_t temp;
    uint32_t oscSource;
    uint32_t clockOut;
    uint32_t divider;

    oscSource = HWREG(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) &
                (uint32_t)SYSCTL_CLKSRCCTL2_AUXOSCCLKSRCSEL_M;

    //
    // If one of the internal oscillators is being used, start from the
    // known default frequency.  Otherwise, use clockInHz parameter.
    //
    if(oscSource == (SYSCTL_AUXPLL_OSCSRC_OSC2 >> SYSCTL_OSCSRC_S))
    {
        //
        // 10MHz Internal Clock
        //
        clockOut = SYSCTL_DEFAULT_OSC_FREQ;
    }
    else
    {
        clockOut = clockInHz;
    }

    //
    // If the PLL is enabled calculate its effect on the clock
    //
    if((HWREG(CLKCFG_BASE + SYSCTL_O_AUXPLLCTL1) &
        (SYSCTL_AUXPLLCTL1_PLLEN | SYSCTL_AUXPLLCTL1_PLLCLKEN)) == 3U)
    {
        //
        // Calculate integer multiplier
        //
        clockOut = clockOut * ((HWREG(CLKCFG_BASE + SYSCTL_O_AUXPLLMULT) &
                                SYSCTL_AUXPLLMULT_IMULT_M) >>
                               SYSCTL_AUXPLLMULT_IMULT_S);

        //
        // Calculate AUXPLL divider
        //
        temp = ((((HWREG(CLKCFG_BASE + SYSCTL_O_AUXPLLMULT) &
                   SYSCTL_AUXPLLMULT_REFDIV_M) >>
                  SYSCTL_AUXPLLMULT_REFDIV_S) + 1U) *
                (((HWREG(CLKCFG_BASE + SYSCTL_O_AUXPLLMULT) &
                   SYSCTL_AUXPLLMULT_ODIV_M) >>
                  SYSCTL_AUXPLLMULT_ODIV_S) + 1U));

        //
        //  Divide by calculated dividers
        //
        if(temp != 0U)
        {
            clockOut /= temp;
        }
    }

    //
    // Calculate divider and divide the clock
    //
    divider = (HWREG(CLKCFG_BASE + SYSCTL_O_AUXCLKDIVSEL) &
               SYSCTL_AUXCLKDIVSEL_AUXPLLDIV_M);

    if(divider < 0x4U)
    {
        clockOut /= (1UL << divider);
    }
    else if(divider > 0x4U)
    {
        clockOut /= divider;
    }
    else
    {
        clockOut /= 0x3U;
    }

    return(clockOut);
}

//*****************************************************************************
//
// SysCtl_setClock()
//
//*****************************************************************************
bool
SysCtl_setClock(uint32_t config)
{
    uint16_t divSel, pllen, oscclksrcsel, pllLockStatus, xtalval;
    uint32_t oscSource, pllMult, mult;
    uint32_t timeout, refdiv;
    bool status = false;

    //
    // Check the arguments.
    //
    ASSERT((config & SYSCTL_OSCSRC_M) <= SYSCTL_OSCSRC_M);
    ASSERT(((config & SYSCTL_PLL_CONFIG_M) == SYSCTL_PLL_ENABLE) ||
           ((config & SYSCTL_PLL_CONFIG_M) == SYSCTL_PLL_BYPASS) ||
           ((config & SYSCTL_PLL_CONFIG_M) == SYSCTL_PLL_DISABLE));

    //
    // Don't proceed to the PLL initialization if an MCD failure is detected.
    //
    if(SysCtl_isMCDClockFailureDetected())
    {
        //
        // OSCCLKSRC2 failure detected. Returning false. You'll need to clear
        // the MCD error.
        //
        status = false;
    }
    else
    {
        //
        // Bypass PLL
        //
        EALLOW;
        HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &=
            ~SYSCTL_SYSPLLCTL1_PLLCLKEN;
        EDIS;

        //
        // Delay of at least 120 OSCCLK cycles required post PLL bypass
        //
        SysCtl_delay(23U);

        //
        // Derive the current and previous oscillator clock source values
        //
        oscclksrcsel = HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &
                      (uint16_t)SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M;

        xtalval = (HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) &
                  (uint16_t)SYSCTL_XTALCR_SE);

        oscSource = (config & SYSCTL_OSCSRC_M) >> SYSCTL_OSCSRC_S;

        //
        // Check if the oscillator clock source has changed
        //
        if((oscclksrcsel | xtalval) != oscSource)
        {
            //
            // Turn off PLL
            //
            EALLOW;
            HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &=
                ~SYSCTL_SYSPLLCTL1_PLLEN;
            EDIS;

            //
            // Delay of at least 66 OSCCLK cycles required between
            // powerdown to powerup of PLL
            //
            SysCtl_delay(12U);

            //
            // Configure oscillator source
            //
            SysCtl_selectOscSource(config & SYSCTL_OSCSRC_M);

            //
            // Delay of at least 60 OSCCLK cycles
            //
            SysCtl_delay(11U);
        }

        //
        // Set dividers to /1 to ensure the fastest PLL configuration
        //
        SysCtl_setPLLSysClk(0U);

        //
        // Configure PLL if PLL usage is enabled or bypassed in config
        //
        if(((config & SYSCTL_PLL_CONFIG_M) == SYSCTL_PLL_ENABLE) ||
           ((config & SYSCTL_PLL_CONFIG_M) == SYSCTL_PLL_BYPASS))
        {
            //
            // Get the PLL multiplier settings from config
            //
            pllMult  = ((config & SYSCTL_IMULT_M) <<
                         SYSCTL_SYSPLLMULT_IMULT_S);

            pllMult |= (((config & SYSCTL_REFDIV_M) >>
                        SYSCTL_REFDIV_S) <<
                        SYSCTL_SYSPLLMULT_REFDIV_S);

            pllMult |= (((config & SYSCTL_ODIV_M) >>
                          SYSCTL_ODIV_S) <<
                          SYSCTL_SYSPLLMULT_ODIV_S);

            //
            // Get the PLL multipliers currently programmed
            //
            mult  = ((HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) &
                      SYSCTL_SYSPLLMULT_IMULT_M) >>
                      SYSCTL_SYSPLLMULT_IMULT_S);

            mult |= (HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) &
                     SYSCTL_SYSPLLMULT_REFDIV_M);

            mult |= (HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) &
                     SYSCTL_SYSPLLMULT_ODIV_M);

            pllen = (HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &
                     SYSCTL_SYSPLLCTL1_PLLEN);

            //
            // Lock PLL only if the multipliers need an update or PLL needs
            // to be powered on / enabled
            //
            if((mult !=  pllMult) || (pllen != 1U))
            {
                //
                // Turn off PLL
                //
                EALLOW;
                HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &=
                    ~SYSCTL_SYSPLLCTL1_PLLEN;
                EDIS;

                //
                // Delay of at least 66 OSCCLK cycles required between
                // powerdown to powerup of PLL
                //
                SysCtl_delay(12U);

                //
                // Write multiplier, which automatically turns on the PLL
                //
                EALLOW;
                HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) = pllMult;

                //
                // Enable/ turn on PLL
                //
                HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) |=
                       SYSCTL_SYSPLLCTL1_PLLEN;

                //
                // Wait for the SYSPLL lock counter or a timeout
                // This timeout needs to be computed based on OSCCLK
                // with a factor of REFDIV.
                // Lock time is 1024 OSCCLK * (REFDIV+1)
                //
                refdiv  = ((config & SYSCTL_REFDIV_M) >> SYSCTL_REFDIV_S);

                timeout = (1024U * (refdiv + 1U));
                pllLockStatus = HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLSTS) &
                                SYSCTL_SYSPLLSTS_LOCKS;

                while((pllLockStatus != 1U) && (timeout != 0U))
                {
                    pllLockStatus = HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLSTS) &
                                    SYSCTL_SYSPLLSTS_LOCKS;
                    timeout--;
                }
                EDIS;

                //
                // Check PLL Frequency using DCC
                //
                status = SysCtl_isPLLValid(
                        (config & SYSCTL_DCC_BASE_M),
                        (config & SYSCTL_OSCSRC_M),
                        SYSCTL_SOURCE_SYSPLL,
                        (config & (SYSCTL_IMULT_M | SYSCTL_ODIV_M |
                                   SYSCTL_REFDIV_M)));

            }
            else
            {
                //
                // Re-Lock of PLL not needed since the multipliers
                // are not updated
                //
                status = true;
            }
        }
        else if((config & SYSCTL_PLL_CONFIG_M) == SYSCTL_PLL_DISABLE)
        {
            //
            // Turn off PLL when the PLL is disabled in config
            //
            EALLOW;
            HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &=
                   ~SYSCTL_SYSPLLCTL1_PLLEN;
            EDIS;

            //
            // PLL is bypassed and not in use
            // Status is updated to true to allow configuring the dividers later
            //
            status = true;
        }
        else
        {
            //
            // Empty
            //
        }

        //
        // If PLL locked successfully, configure the dividers
        // Or if PLL is bypassed, only configure the dividers
        //
        if(status)
        {
            //
            // Set divider to produce slower output frequency to limit current
            // increase.
            //
            divSel = (uint16_t)(config & SYSCTL_SYSDIV_M) >> SYSCTL_SYSDIV_S;

            EALLOW;
            if(divSel != (126U / 2U))
            {
                HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) =
                    (HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                     ~(uint16_t)SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M) |
                    (divSel + 1U);
            }
            else
            {
                HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) =
                    (HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                     ~(uint16_t)SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M) | divSel;
            }

            EDIS;

            //
            // Feed system clock from SYSPLL only if PLL usage is enabled
            //
            if((config & SYSCTL_PLL_CONFIG_M) == SYSCTL_PLL_ENABLE)
            {

                //
                // Enable PLLSYSCLK is fed from system PLL clock
                //
                EALLOW;
                HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) |=
                       SYSCTL_SYSPLLCTL1_PLLCLKEN;
                EDIS;

            }

            //
            // ~200 PLLSYSCLK delay to allow voltage regulator to stabilize
            // prior to increasing entire system clock frequency.
            //
            SysCtl_delay(40U);

            //
            // Set the divider to user value
            //
            EALLOW;
            HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) =
                (HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                 ~SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M) | divSel;
            EDIS;
        }
        else
        {
            ESTOP0; // If the frequency is out of range, stop here.
        }
    }

    return(status);
}
//*****************************************************************************
//
// SysCtl_setAuxClock()
//
//*****************************************************************************
void SysCtl_setAuxClock(uint32_t config)
{
    uint32_t pllMult = 0U, mult, oscSource;
    uint16_t started = 0U, pllen, oscclksrcsel, xtalval;
    bool status = false;

    //
    // Check the arguments
    //
    ASSERT((config & SYSCTL_OSCSRC_M) <= SYSCTL_OSCSRC_M);
    ASSERT(((config & SYSCTL_PLL_CONFIG_M) == SYSCTL_AUXPLL_ENABLE) ||
           ((config & SYSCTL_PLL_CONFIG_M) == SYSCTL_AUXPLL_BYPASS) ||
           ((config & SYSCTL_PLL_CONFIG_M) == SYSCTL_AUXPLL_DISABLE));

    //
    // Bypass PLL
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_AUXPLLCTL1) &= ~SYSCTL_AUXPLLCTL1_PLLCLKEN;

    //
    // Delay of at least 120 OSCCLK cycles required post PLL bypass
    //
    SysCtl_delay(23U);
    EDIS;

    //
    // Derive the current and previous oscillator clock source values
    //
    oscclksrcsel = HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) &
                  SYSCTL_CLKSRCCTL2_AUXOSCCLKSRCSEL_M;

    oscSource = (config & SYSCTL_OSCSRC_M) >> SYSCTL_OSCSRC_S;

    xtalval = (HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) &  SYSCTL_XTALCR_SE);

    //
    // Check if the oscillator clock source has changed
    //
    if((oscclksrcsel | xtalval) != oscSource)
    {
        //
        // Turn off PLL
        //
        EALLOW;
        HWREGH(CLKCFG_BASE + SYSCTL_O_AUXPLLCTL1) &= ~SYSCTL_AUXPLLCTL1_PLLEN;
        EDIS;

        //
        // Delay of at least 66 OSCCLK cycles
        //
        SysCtl_delay(12U);

        //
        // Configure oscillator source
        //
        SysCtl_selectOscSourceAuxPLL(config & SYSCTL_OSCSRC_M);

        //
        // Delay of at least 60 OSCCLK cycles
        //
        SysCtl_delay(11U);
    }

    //
    // Configure PLL if PLL usage is enabled or bypassed in config
    //
    if(((config & SYSCTL_PLL_CONFIG_M) == SYSCTL_AUXPLL_ENABLE) ||
       ((config & SYSCTL_PLL_CONFIG_M) == SYSCTL_AUXPLL_BYPASS))
    {
        //
        // Get the PLL multiplier settings from config
        //
        pllMult |= ((config & SYSCTL_IMULT_M) << SYSCTL_AUXPLLMULT_IMULT_S);

        pllMult |= (((config & SYSCTL_REFDIV_M) >> SYSCTL_REFDIV_S) <<
                    SYSCTL_AUXPLLMULT_REFDIV_S);

        pllMult |= (((config & SYSCTL_ODIV_M) >> SYSCTL_ODIV_S) <<
                    SYSCTL_AUXPLLMULT_ODIV_S);

        //
        // Get the PLL multipliers currently programmed
        //
        mult  = ((HWREG(CLKCFG_BASE + SYSCTL_O_AUXPLLMULT) &
                  SYSCTL_AUXPLLMULT_IMULT_M) >>
                  SYSCTL_AUXPLLMULT_IMULT_S);

        mult |= (HWREG(CLKCFG_BASE + SYSCTL_O_AUXPLLMULT) &
                 SYSCTL_AUXPLLMULT_REFDIV_M);

        mult |= (HWREG(CLKCFG_BASE + SYSCTL_O_AUXPLLMULT) &
                 SYSCTL_AUXPLLMULT_ODIV_M);

        pllen = (HWREGH(CLKCFG_BASE + SYSCTL_O_AUXPLLCTL1) &
                 SYSCTL_AUXPLLCTL1_PLLEN);

        //
        // Lock PLL only if the multipliers need an update or PLL needs
        // to be powered on / enabled
        //
        if((mult !=  pllMult) || (pllen != 1U))
        {

            EALLOW;

            //
            // Turn off AUXPLL and delay for it to power down.
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_AUXPLLCTL1) &=
                ~SYSCTL_AUXPLLCTL1_PLLEN;

            //
            // Delay of at least 66 OSCCLK cycles
            //
            SysCtl_delay(12U);

            //
            // Set integer and fractional multiplier, which automatically
            // turns on the PLL
            //
            HWREG(CLKCFG_BASE + SYSCTL_O_AUXPLLMULT) = pllMult;

            //
            // Enable AUXPLL
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_AUXPLLCTL1) |=
                SYSCTL_AUXPLLCTL1_PLLEN;
            EDIS;

            //
            // Wait for the AUXPLL lock counter
            //

            while((HWREGH(CLKCFG_BASE + SYSCTL_O_AUXPLLSTS) &
                   SYSCTL_AUXPLLSTS_LOCKS) != 1U)
            {
                //
                // Consider to servicing the watchdog using
                // SysCtl_serviceWatchdog()
                //
            }

            //
            // Check PLL Frequency using DCC
            //
            status = SysCtl_isPLLValid(
                    (config & SYSCTL_DCC_BASE_M), (config & SYSCTL_OSCSRC_M),
                     SYSCTL_SOURCE_AUXPLL ,
                    (config & (SYSCTL_IMULT_M | SYSCTL_ODIV_M |
                               SYSCTL_REFDIV_M )));

            //
            // Check DCC Status, If the frequency is out of range, stop here.
            //
            if(status)
            {
                //
                // Set flag to indicate PLL started
                //
                started = 1U;
            }
            else
            {
                ESTOP0;
            }

            if((config & SYSCTL_PLL_CONFIG_M) == SYSCTL_AUXPLL_ENABLE)
            {
                //
                // Enable AUXPLLCLK to be fed from AUXPLL
                //
                EALLOW;
                HWREGH(CLKCFG_BASE + SYSCTL_O_AUXPLLCTL1) |=
                       SYSCTL_AUXPLLCTL1_PLLCLKEN;
                SysCtl_delay(3U);

                EDIS;
            }

            if(started == 0U)
            {
                //
                // AUX PLL may not have started. Reset multiplier to 0 (bypass
                // PLL).
                //
                EALLOW;
                HWREGH(CLKCFG_BASE + SYSCTL_O_AUXPLLMULT) = 0U;
                EDIS;

                //
                // The user should put some handler code here based on how
                // this condition should be handled in their application.
                //
                ESTOP0;
            }

        }
        else
        {
            if((config & SYSCTL_PLL_CONFIG_M) == SYSCTL_AUXPLL_ENABLE)
            {
                //
                // Enable AUXPLLCLK to be fed from AUXPLL
                //
                EALLOW;
                HWREGH(CLKCFG_BASE + SYSCTL_O_AUXPLLCTL1) |=
                       SYSCTL_AUXPLLCTL1_PLLCLKEN;
                SysCtl_delay(3U);

                EDIS;
            }
        }

    }
    else if((config & SYSCTL_PLL_CONFIG_M) == SYSCTL_AUXPLL_DISABLE)
    {
        //
        // Turn off PLL when the PLL is disabled in config
        //
        EALLOW;
        HWREGH(CLKCFG_BASE + SYSCTL_O_AUXPLLCTL1) &= ~SYSCTL_AUXPLLCTL1_PLLEN;
        EDIS;
    }
    else
    {
        //
        // Empty
        //
    }

    //
    // Set divider to desired value
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_AUXCLKDIVSEL) =
        (uint16_t)(config & SYSCTL_SYSDIV_M) >> SYSCTL_SYSDIV_S;
    EDIS;

}


//*****************************************************************************
//
// SysCtl_selectXTAL()
//
//*****************************************************************************
void
SysCtl_selectXTAL(void)
{
    EALLOW;

    //
    // Turn on XTAL and select crystal mode
    //
    HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) &= ~SYSCTL_XTALCR_OSCOFF;
    HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) &= ~SYSCTL_XTALCR_SE;
    EDIS;

    //
    // Wait for the X1 clock to saturate
    //
    SysCtl_pollX1Counter();

    //
    // Select XTAL as the oscillator source
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) =
    ((HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &
      (~SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M)) |
     (SYSCTL_OSCSRC_XTAL >> SYSCTL_OSCSRC_S));
    EDIS;

    //
    // If a missing clock failure was detected, try waiting for the X1 counter
    // to saturate again. Consider modifying this code to add a 10ms timeout.
    //
    while(SysCtl_isMCDClockFailureDetected())
    {
        //
        // Clear the MCD failure
        //
        SysCtl_resetMCD();

        //
        // Wait for the X1 clock to saturate
        //
        SysCtl_pollX1Counter();

        //
        // Select XTAL as the oscillator source
        //
        EALLOW;
        HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) =
        ((HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &
          (~SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M)) |
         (SYSCTL_OSCSRC_XTAL >> SYSCTL_OSCSRC_S));
        EDIS;
    }
}

//*****************************************************************************
//
// SysCtl_selectXTALSingleEnded()
//
//*****************************************************************************
void
SysCtl_selectXTALSingleEnded(void)
{
    //
    // Turn on XTAL and select single-ended mode.
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) &= ~SYSCTL_XTALCR_OSCOFF;
    HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) |= SYSCTL_XTALCR_SE;
    EDIS;

    //
    // Wait for the X1 clock to saturate
    //
    SysCtl_pollX1Counter();

    //
    // Select XTAL as the oscillator source
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) =
    ((HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &
      (~SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M)) |
     (SYSCTL_OSCSRC_XTAL >> SYSCTL_OSCSRC_S));
    EDIS;

    //
    // Something is wrong with the oscillator module. Replace the ESTOP0 with
    // an appropriate error-handling routine.
    //
    while(SysCtl_isMCDClockFailureDetected())
    {
        ESTOP0;
    }
}

//*****************************************************************************
//
// SysCtl_selectOscSource()
//
//*****************************************************************************
void
SysCtl_selectOscSource(uint32_t oscSource)
{
    ASSERT((oscSource == SYSCTL_OSCSRC_OSC1) ||
           (oscSource == SYSCTL_OSCSRC_OSC2) ||
           (oscSource == SYSCTL_OSCSRC_XTAL) ||
           (oscSource == SYSCTL_OSCSRC_XTAL_SE));

    //
    // Select the specified source.
    //
    EALLOW;
    switch(oscSource)
    {
        case SYSCTL_OSCSRC_OSC2:
            //
            // Clk Src = INTOSC2
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &=
                ~SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M;

            break;

        case SYSCTL_OSCSRC_XTAL:
            //
            // Select XTAL in crystal mode and wait for it to power up
            //
            SysCtl_selectXTAL();
            break;

        case SYSCTL_OSCSRC_XTAL_SE:
            //
            // Select XTAL in single-ended mode and wait for it to power up
            //
            SysCtl_selectXTALSingleEnded();
            break;

        case SYSCTL_OSCSRC_OSC1:
            //
            // Clk Src = INTOSC1
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) =
                   (HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &
                    ~SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M) |
                   (SYSCTL_OSCSRC_OSC1 >> SYSCTL_OSCSRC_S);

            break;

        default:
            //
            // Do nothing. Not a valid oscSource value.
            //
            break;
    }
    EDIS;
}

//*****************************************************************************
//
// SysCtl_selectOscSourceAuxPLL()
//
//*****************************************************************************
void
SysCtl_selectOscSourceAuxPLL(uint32_t oscSource)
{
    EALLOW;

    switch(oscSource)
    {
        case SYSCTL_AUXPLL_OSCSRC_OSC2:
            //
            // Clk Src = INTOSC2
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) &=
                    ~(SYSCTL_CLKSRCCTL2_AUXOSCCLKSRCSEL_M);
            break;

        case SYSCTL_AUXPLL_OSCSRC_XTAL:
            //
            // Turn on XTALOSC
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &=
                   ~(SYSCTL_CLKSRCCTL1_XTALOFF);

            //
            // Select crystal mode
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) &= ~SYSCTL_XTALCR_SE;

            //
            // Wait for the X1 clock to saturate
            //
            SysCtl_pollX1Counter();

            //
            // Clk Src = XTAL
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) =
                    (HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) &
                     ~(SYSCTL_CLKSRCCTL2_AUXOSCCLKSRCSEL_M)) |
                    (1U << SYSCTL_CLKSRCCTL2_AUXOSCCLKSRCSEL_S);
            break;

        case SYSCTL_AUXPLL_OSCSRC_XTAL_SE:
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &=
                   ~(SYSCTL_CLKSRCCTL1_XTALOFF);
            //
            // Select crystal mode
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) |= SYSCTL_XTALCR_SE;

            //
            // Wait for the X1 clock to saturate
            //
            SysCtl_pollX1Counter();

            //
            // Clk Src = XTAL
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) =
                    (HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) &
                     ~(SYSCTL_CLKSRCCTL2_AUXOSCCLKSRCSEL_M)) |
                    (1U << SYSCTL_CLKSRCCTL2_AUXOSCCLKSRCSEL_S);
            break;

        case SYSCTL_AUXPLL_OSCSRC_AUXCLKIN:
            //
            // Clk Src = AUXCLKIN
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) =
                    (HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) &
                     ~(SYSCTL_CLKSRCCTL2_AUXOSCCLKSRCSEL_M)) |
                    (2U << SYSCTL_CLKSRCCTL2_AUXOSCCLKSRCSEL_S);
            break;

        default:
            //
            // Do nothing. Not a valid clock source value.
            //
            break;
    }
    EDIS;
}

//*****************************************************************************
//
// SysCtl_getLowSpeedClock()
//
//*****************************************************************************
uint32_t
SysCtl_getLowSpeedClock(uint32_t clockInHz)
{
    uint32_t clockOut;

    //
    // Get the main system clock
    //
    clockOut = SysCtl_getClock(clockInHz);

    //
    // Apply the divider to the main clock
    //
    if((HWREG(CLKCFG_BASE + SYSCTL_O_LOSPCP) &
        SYSCTL_LOSPCP_LSPCLKDIV_M) != 0U)
    {
        clockOut /= (2U * (HWREG(CLKCFG_BASE + SYSCTL_O_LOSPCP) &
                            SYSCTL_LOSPCP_LSPCLKDIV_M));
    }

    return(clockOut);
}

//*****************************************************************************
//
// SysCtl_getDeviceParametric()
//
//*****************************************************************************
uint16_t
SysCtl_getDeviceParametric(SysCtl_DeviceParametric parametric)
{
    uint32_t value;

    //
    // Get requested parametric value
    //
    switch(parametric)
    {
        case SYSCTL_DEVICE_QUAL:
            //
            // Qualification Status
            //
            value = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDL) &
                      SYSCTL_PARTIDL_QUAL_M) >> SYSCTL_PARTIDL_QUAL_S);
            break;

        case SYSCTL_DEVICE_PINCOUNT:
            //
            // Pin Count
            //
            value = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDL) &
                      SYSCTL_PARTIDL_PIN_COUNT_M) >>
                     SYSCTL_PARTIDL_PIN_COUNT_S);
            break;

        case SYSCTL_DEVICE_INSTASPIN:
            //
            // InstaSPIN Feature Set
            //
            value = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDL) &
                      SYSCTL_PARTIDL_INSTASPIN_M) >>
                     SYSCTL_PARTIDL_INSTASPIN_S);
            break;

        case SYSCTL_DEVICE_FLASH:
            //
            // Flash Size (KB)
            //
            value = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDL) &
                      SYSCTL_PARTIDL_FLASH_SIZE_M) >>
                     SYSCTL_PARTIDL_FLASH_SIZE_S);
            break;

        case SYSCTL_DEVICE_PARTID:
            //
            // PARTID Format Revision
            //
            value = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDL) &
                      SYSCTL_PARTIDL_PARTID_FORMAT_REVISION_M) >>
                     SYSCTL_PARTIDL_PARTID_FORMAT_REVISION_S);
            break;

        case SYSCTL_DEVICE_FAMILY:
            //
            // Device Family
            //
            value = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDH) &
                      SYSCTL_PARTIDH_FAMILY_M) >> SYSCTL_PARTIDH_FAMILY_S);
            break;

        case SYSCTL_DEVICE_PARTNO:
            //
            // Part Number
            //
            value = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDH) &
                      SYSCTL_PARTIDH_PARTNO_M) >> SYSCTL_PARTIDH_PARTNO_S);
            break;

        case SYSCTL_DEVICE_CLASSID:
            //
            // Class ID
            //
            value = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDH) &
                      SYSCTL_PARTIDH_DEVICE_CLASS_ID_M) >>
                     SYSCTL_PARTIDH_DEVICE_CLASS_ID_S);
            break;

        default:
            //
            // Not a valid value for PARTID register
            //
            value = 0U;
            break;
    }

    return((uint16_t)value);
}
//*****************************************************************************
//
// SysCtl_isPLLValid()
//
//*****************************************************************************
bool
SysCtl_isPLLValid(uint32_t base, uint32_t oscSource,
                  SysCtl_PLLClockSource pllclk, uint32_t pllMultDiv)
{
    uint32_t imult, odiv, refdiv;
    float  fclk1_0ratio, fclk0_1ratio;
    float32_t total_error, window;

    DCC_Count0ClockSource dccClkSrc0;
    DCC_Count1ClockSource dccClkSrc1;
    uint32_t dccCounterSeed0, dccCounterSeed1, dccValidSeed0;

    switch(oscSource)
    {
        case SYSCTL_OSCSRC_OSC2:
            //
            // Select DCC Clk Src0 as INTOSC2
            //
            dccClkSrc0 = DCC_COUNT0SRC_INTOSC2;
            break;
        case SYSCTL_OSCSRC_XTAL:
        case SYSCTL_OSCSRC_XTAL_SE:
            //
            // Select DCC Clk Src0 as XTAL
            //
            dccClkSrc0 = DCC_COUNT0SRC_XTAL;
            break;
        case SYSCTL_OSCSRC_OSC1:
            //
            // Select DCC Clk Src0 as INTOSC1
            //
            dccClkSrc0 = DCC_COUNT0SRC_INTOSC1;
            break;
        default:
            //
            // Select DCC Clk Src0 as INTOSC1
            //
            dccClkSrc0 = DCC_COUNT0SRC_INTOSC1;
            break;
    }

    switch(pllclk)
    {
        case SYSCTL_SOURCE_SYSPLL:
            //
            // Select DCC Clk Src1 as SYSPLL
            //
            dccClkSrc1 = DCC_COUNT1SRC_PLL;
            break;
        case SYSCTL_SOURCE_AUXPLL:
            //
            // Select DCC Clk Src1 as AUXPLL
            //
            dccClkSrc1 = DCC_COUNT1SRC_AUXPLL;
            break;
        default:
            //
            // Select DCC Clk Src1 as SYSPLL
            //
            dccClkSrc1 = DCC_COUNT1SRC_PLL;
            break;
    }

    //
    // Assigning DCC for PLL validation
    //
    if(base == SYSCTL_DCC_BASE_0)
    {
        base = DCC0_BASE;
    }
    else if(base == SYSCTL_DCC_BASE_1)
    {
        base = DCC1_BASE;
    }
    else
    {
        base = DCC2_BASE;
    }

    //
    // Retrieving PLL parameters
    //
    imult = pllMultDiv & SYSCTL_IMULT_M;
    odiv = (pllMultDiv & SYSCTL_ODIV_M) >> SYSCTL_ODIV_S;
    refdiv = (pllMultDiv & SYSCTL_REFDIV_M) >> SYSCTL_REFDIV_S;

    fclk1_0ratio = (float)imult / (((float)odiv + 1.0F) *
                                   ((float)refdiv + 1.0F));
    fclk0_1ratio = (((float)refdiv + 1.0F) * ((float)odiv + 1.0F)) /
                                             (float)imult;

    if(fclk1_0ratio >= 1.0F)
    {
        //
        // Setting Counter0 & Valid Seed Value with +/-1% tolerance
        // Total error is 12
        //
        window = (12U * 100U) / ((uint32_t)SYSCTL_DCC_COUNTER0_TOLERANCE);
        dccCounterSeed0 = (uint32_t)window - 12U;
        dccValidSeed0 = 24U;
    }
    else
    {
        total_error = ((2.0F * fclk0_1ratio) + 10.0F);
        window = (total_error * 100.0F) /
                 ((float32_t)SYSCTL_DCC_COUNTER0_TOLERANCE);

        //
        // Setting Counter0 & Valid Seed Value with +/-1% tolerance
        //
        dccCounterSeed0 = window - total_error;
        dccValidSeed0 = 2U * (uint32_t)total_error;
    }

    //
    // Multiplying Counter-0 window with PLL Integer Multiplier
    //
    dccCounterSeed1 = (window * fclk1_0ratio);


    //
    // Enable peripheral clock to DCC
    //
    if(base == DCC0_BASE)
    {
        SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DCC0);
    }
    else if(base == DCC1_BASE)
    {
        SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DCC1);
    }
    else
    {
        SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DCC2);
    }

    //
    // Clear Error & Done Flag
    //
    DCC_clearErrorFlag(base);
    DCC_clearDoneFlag(base);

    //
    // Disable DCC
    //
    DCC_disableModule(base);

    //
    // Disable Error Signal
    //
    DCC_disableErrorSignal(base);

    //
    // Disable Done Signal
    //
    DCC_disableDoneSignal(base);

    //
    // Configure Clock Source0 to whatever set as a clock source for PLL
    //
    DCC_setCounter0ClkSource(base, dccClkSrc0);

    //
    // Configure Clock Source1 to PLL
    //
    DCC_setCounter1ClkSource(base, dccClkSrc1);

    //
    // Configure COUNTER-0, COUNTER-1 & Valid Window
    //
    DCC_setCounterSeeds(base, dccCounterSeed0, dccValidSeed0,
                        dccCounterSeed1);

    //
    // Enable Single Shot mode
    //
    DCC_enableSingleShotMode(base, DCC_MODE_COUNTER_ZERO);


    //
    // Enable DCC to start counting
    //
    DCC_enableModule(base);

    //
    // Timeout for the loop
    //
    uint32_t timeout = dccCounterSeed1;

    //
    // Wait until Error or Done Flag is generated
    //
    while(((HWREGH(base + DCC_O_STATUS) &
            (DCC_STATUS_ERR | DCC_STATUS_DONE)) == 0U) && (timeout != 0U))

    {
        timeout--;
    }


    //
    // Returns true if DCC completes without error
    //

    return((HWREGH(base + DCC_O_STATUS) &
            (DCC_STATUS_ERR | DCC_STATUS_DONE)) == DCC_STATUS_DONE);
}
//*****************************************************************************
//
// SysCtl_controlCPU2Reset()
//
//*****************************************************************************
void
SysCtl_controlCPU2Reset(SysCtl_CoreReset control)
{
    uint32_t clearvalue;

    //
    //Based on whether the Core is to be reset or not,
    //the core would be put into reset or brought out.
    //
    if(control != SYSCTL_CORE_DEACTIVE)
    {
        //
        //On matching key, write to the reset bits would be successful
        //

        EALLOW;
        HWREG(DEVCFG_BASE +
            SYSCTL_O_CPU2RESCTL)  = ((uint32_t)SYSCTL_CPU2RESCTL_RESET |
                                    (SYSCTL_REG_KEY & SYSCTL_CPU2RESCTL_KEY_M));
        EDIS;
    }
    else
    {
        EALLOW;

        //
        //On matching key, write to the reset bits would be successful
        //
        clearvalue = HWREG(DEVCFG_BASE + SYSCTL_O_CPU2RESCTL);
        clearvalue &= ~SYSCTL_CPU2RESCTL_RESET;
        HWREG(DEVCFG_BASE + SYSCTL_O_CPU2RESCTL) = (SYSCTL_REG_KEY &
                                                    SYSCTL_CPU2RESCTL_KEY_M) |
                                                    clearvalue;
        EDIS;
    }
}

//*****************************************************************************
//
// SysCtl_configureType()
//
//*****************************************************************************
void
SysCtl_configureType(SysCtl_SelType type , uint16_t config, uint16_t lock)
{
    ASSERT(lock <= 1U);

    EALLOW;

    //
    // Check which type needs to be configured , the type would be enabled /
    // disabled along with making the configurations unalterable as per input.
    //
    switch(type)
    {
        case SYSCTL_ECAPTYPE:
            HWREGH(DEVCFG_BASE + SYSCTL_O_ECAPTYPE) =
                                config | (lock << SYSCTL_TYPE_LOCK_S);
            break;

        case SYSCTL_SDFMTYPE:
            HWREGH(DEVCFG_BASE + SYSCTL_O_SDFMTYPE) =
                                config | (lock << SYSCTL_TYPE_LOCK_S);
            break;

        case SYSCTL_USBTYPE:
            HWREGH(DEVCFG_BASE + SYSCTL_O_USBTYPE) =
                                config | (lock << SYSCTL_TYPE_LOCK_S);
            break;

        case SYSCTL_MEMMAPTYPE:
            HWREGH(DEVCFG_BASE + SYSCTL_O_MEMMAPTYPE) =
                                config | (lock << SYSCTL_TYPE_LOCK_S);
            break;

        default:
            break;
    }
    EDIS;

}

//*****************************************************************************
//
// SysCtl_isConfigTypeLocked()
//
//*****************************************************************************
bool
SysCtl_isConfigTypeLocked(SysCtl_SelType type)
{
    bool lock = false;

    //
    // Check if the provided type registers can be modified or not.
    //
    switch(type)
    {
        case SYSCTL_ECAPTYPE:
            lock = ((HWREGH(DEVCFG_BASE + SYSCTL_O_ECAPTYPE) &
                     SYSCTL_ECAPTYPE_LOCK) != 0U);
            break;

        case SYSCTL_USBTYPE:
            lock = ((HWREGH(DEVCFG_BASE + SYSCTL_O_USBTYPE) &
                     SYSCTL_USBTYPE_LOCK) != 0U);
            break;

        case SYSCTL_SDFMTYPE:
            lock = ((HWREGH(DEVCFG_BASE + SYSCTL_O_SDFMTYPE) &
                     SYSCTL_SDFMTYPE_LOCK) != 0U);
            break;

        case SYSCTL_MEMMAPTYPE:
            lock = ((HWREGH(DEVCFG_BASE + SYSCTL_O_MEMMAPTYPE) &
                     SYSCTL_MEMMAPTYPE_LOCK) != 0U);
            break;

        default:
            break;
    }

  return(lock);
}

//*****************************************************************************
//
// SysCtl_setSemOwner()
//
//*****************************************************************************
void
SysCtl_setSemOwner (SysCtl_CPUSel cpuInst)
{
    uint32_t semVal = (cpuInst == SYSCTL_CPUSEL_CPU1) ? 0x10UL : 0x01UL;

    EALLOW;

    //
    // Free the semaphore
    //
    HWREG(CLKCFG_BASE + SYSCTL_O_CLKSEM) = SYSCTL_REG_KEY;

    //
    // CPU1 Acquire the semaphore
    //
    HWREG(CLKCFG_BASE + SYSCTL_O_CLKSEM) = semVal | SYSCTL_REG_KEY;

    EDIS;
}

//*****************************************************************************
//
// SysCtl_lockClkConfig()
//
//*****************************************************************************
void
SysCtl_lockClkConfig(SysCtl_ClkRegSel registerName)
{
    uint16_t bitIndex;

    //
    // Decode the register variable.
    //
    bitIndex = ((uint16_t)registerName & SYSCTL_PERIPH_BIT_M) >>
                SYSCTL_PERIPH_BIT_S;

    //
    // Locks the particular clock configuration register
    //
    EALLOW;
    HWREG(CLKCFG_BASE + SYSCTL_O_CLKCFGLOCK1) |= (1UL << bitIndex);
    EDIS;
}

//*****************************************************************************
//
// SysCtl_lockSysConfig()
//
//*****************************************************************************
void
SysCtl_lockSysConfig (SysCtl_CpuRegSel registerName)
{
    uint16_t regIndex;
    uint16_t bitIndex;

    //
    // Decode the register variable.
    //
    regIndex = 2U * ((uint16_t)registerName & SYSCTL_PERIPH_REG_M);
    bitIndex = ((uint16_t)registerName & SYSCTL_PERIPH_BIT_M) >>
                 SYSCTL_PERIPH_BIT_S;

    //
    // Locks the particular System configuration register
    //
    EALLOW;
    HWREG(CPUSYS_BASE + SYSCTL_O_CPUSYSLOCK1 + regIndex) |=
                                            (1UL << bitIndex);
    EDIS;
}

//*****************************************************************************
//
// SysCtl_controlCMReset()
//
//*****************************************************************************
void
SysCtl_controlCMReset(SysCtl_CoreReset control)
{
    if(control == SYSCTL_CORE_ACTIVE)
    {
        //
        // Assert reset to CM
        //
        EALLOW;
        HWREG(CMCONF_BASE + SYSCTL_O_CMRESCTL) = SYSCTL_CMRESCTL_RESET |
                                                 SYSCTL_REG_KEY;

        //
        // Wait until CM is out to reset.
        //
        while(!SysCtl_isCMReset())
        {
        }

        //
        // De-assert reset to CM
        //
        HWREG(CMCONF_BASE + SYSCTL_O_CMRESCTL) = SYSCTL_REG_KEY;
        EDIS;
    }
    else
    {
        //
        // Bring CM out of reset only when CM is in reset state
        //
        if(SysCtl_isCMReset())
        {
            EALLOW;

            //
            // De-assert reset to CM
            //
            HWREG(CMCONF_BASE + SYSCTL_O_CMRESCTL) = SYSCTL_REG_KEY;
            EDIS;
        }
    }
}


