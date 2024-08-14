//###########################################################################
//
// FILE:   sysctl.h
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

#ifndef SYSCTL_H
#define SYSCTL_H

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
//! \addtogroup sysctl_api SysCtl
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_nmi.h"
#include "inc/hw_wwd.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "cpu.h"
#include "debug.h"
#include "interrupt.h"


//*****************************************************************************
//
// Defines for system control functions. Not intended for use by application
// code.
//
//*****************************************************************************

//
// Shifted pattern for WDCR register's WDCHK field.
//
#define SYSCTL_WD_CHKBITS       0x0028U

//
// Keys for WDKEY field. The first enables resets and the second resets.
//
#define SYSCTL_WD_ENRSTKEY      0x0055U
#define SYSCTL_WD_RSTKEY        0x00AAU

//
// Values to help decode peripheral parameter
//
#define SYSCTL_PERIPH_REG_M     0x001FU
#define SYSCTL_PERIPH_REG_S     0x0000U
#define SYSCTL_PERIPH_BIT_M     0x1F00U
#define SYSCTL_PERIPH_BIT_S     0x0008U

//
//Keys for the System control registers write protection
//
#define SYSCTL_REG_KEY       0xA5A50000U
#define SYSCTL_PLL_KEY       0XCAFE0000U

//
//Values to help access shifting of bits
//
#define SYSCTL_CMCLKCTL_CMDIVSRCSEL_S   0x0U
#define SYSCTL_CMCLKCTL_ETHDIVSRCSEL_S  0X4U
#define SYSCTL_ETHERCATCLKCTL_DIVSRCSEL_S 0x0U
#define SYSCTL_ETHERCATCLKCTL_PHYCLKEN_S  0x8U
#define SYSCTL_CLBCLKCTL_TILECLKDIV_S   0x4U
#define SYSCTL_TYPE_LOCK_S              0xFU


//
// LPM defines for LPMCR.LPM
//
#define SYSCTL_LPM_IDLE         0x0000U
#define SYSCTL_LPM_STANDBY      0x0001U

//
// Bit shift for DAC to configure the CPUSEL register
//
#define SYSCTL_CPUSEL_DAC_S     0x10U

//
// Default internal oscillator frequency, 10 MHz
//
#define SYSCTL_DEFAULT_OSC_FREQ     10000000U


//
// Boot ROM Booting and Reset Status
//
#define SYSCTL_BOOT_ROM_STATUS    0x0002U
#define SYSCTL_BOOT_ROM_POR       0x8000U
#define SYSCTL_BOOT_ROM_XRS       0x4000U

#define SYSCTL_DEVICECAL_CONTEXT_SAVE asm(" PUSH ACC  \n\
                                            PUSH DP   \n\
                                            PUSH XAR0 \n\
                                            PUSH XAR2 \n\
                                            PUSH XAR3 \n\
                                            PUSH XAR4 \n\
                                            PUSH XAR5 \n\
                                           ")

#define SYSCTL_DEVICECAL_CONTEXT_RESTORE  asm(" POP XAR5 \n\
                                                POP XAR4 \n\
                                                POP XAR3 \n\
                                                POP XAR2 \n\
                                                POP XAR0 \n\
                                                POP DP   \n\
                                                POP ACC  \n\
                                              ")

//
// Device_cal function which is available in OTP memory
// This function is called in SysCtl_resetPeripheral after resetting
// analog peripherals
//
#define Device_cal ((void (*)(void))((uintptr_t)0x70260))

//*****************************************************************************
//
// The following are values that can be passed to the SysCtl_setClock() API as
// the config parameter.
//
//*****************************************************************************
//
// System clock divider (SYSDIV)
//
#define SYSCTL_SYSDIV_M     0x00003F00UL // Mask for SYSDIV value in config
#define SYSCTL_SYSDIV_S     8U           // Shift for SYSDIV value in config

//
// Mask and shift for Reference Clock Divider value in config
//
#define SYSCTL_REFDIV_M      0x007C0000UL // Mask for Reference Clock Divider
#define SYSCTL_REFDIV_S      18U          // Shift for Reference Clock Divider

//
// Mask and shift for Output Clock Divider value in config
//
#define SYSCTL_ODIV_M        0x0F800000UL
#define SYSCTL_ODIV_S        23U

//!
//! Macro to format Clock divider value. x is a number from 1 to 32.
//!
#define SYSCTL_REFDIV(x)     ((((uint32_t)(x) - 1U) << SYSCTL_REFDIV_S) &      \
                                  SYSCTL_REFDIV_M)

//!
//! Macro to format Clock divider value. x is a number from 1 to 32.
//!
#define SYSCTL_ODIV(x)       ((((uint32_t)(x) - 1U) << SYSCTL_ODIV_S) &        \
                                  SYSCTL_ODIV_M)



//! Macro to format system clock divider value. x must be 1 or even values up
//! to 126.
#define SYSCTL_SYSDIV(x)    ((((x) / 2U) << SYSCTL_SYSDIV_S) & SYSCTL_SYSDIV_M)

//
// Integer multiplier (IMULT)
//
//
// Mask for IMULT value in config
//
#define SYSCTL_IMULT_M      0x000000FFUL
#define SYSCTL_IMULT_S      0U           // Shift for IMULT value in config
//! Macro to format integer multiplier value. x is a number from 1 to 127.
//!
#define SYSCTL_IMULT(x)     (((x) << SYSCTL_IMULT_S) & SYSCTL_IMULT_M)

#ifndef DOXYGEN_PDF_IGNORE
//
// Fractional multiplier (FMULT)
//
#define SYSCTL_FMULT_M      0x0000C000U // Mask for FMULT value in config
#define SYSCTL_FMULT_S      14U         // Shift for FMULT value in config
#define SYSCTL_FMULT_NONE   0x00000000UL //!< No fractional multiplier
#define SYSCTL_FMULT_0      0x00000000UL //!< No fractional multiplier
#define SYSCTL_FMULT_1_4    0x00004000UL //!< Fractional multiplier of 0.25
#define SYSCTL_FMULT_1_2    0x00008000UL //!< Fractional multiplier of 0.50
#define SYSCTL_FMULT_3_4    0x0000C000UL //!< Fractional multiplier of 0.75

//
// DCC module selection for checking PLL clock validity
//
// Mask and shift for DCC module base address in config
//
#define SYSCTL_DCC_BASE_M    0x30000000UL

#define SYSCTL_DCC_BASE_S    28U

#define SYSCTL_DCC_BASE_0       0x00000000UL //!< DCC0 module
#define SYSCTL_DCC_BASE_1       0x10000000UL //!< DCC1 module
#define SYSCTL_DCC_BASE_2       0x20000000UL //!< DCC2 module


//
// Oscillator source
//
// Also used with the SysCtl_selectOscSource(), SysCtl_turnOnOsc(),
// and SysCtl_turnOffOsc() functions as the oscSource parameter.
//
#define SYSCTL_OSCSRC_M         0x00030000UL // Mask for OSCSRC value in config
#define SYSCTL_OSCSRC_S         16U          // Shift for OSCSRC value in config
//! Internal oscillator INTOSC2
#define SYSCTL_OSCSRC_OSC2      0x00000000UL
//! External oscillator (XTAL) in crystal mode
#define SYSCTL_OSCSRC_XTAL      0x00010000U
//! External oscillator (XTAL) in single-ended mode
#define SYSCTL_OSCSRC_XTAL_SE   0x00030000U
//! Internal oscillator INTOSC1
#define SYSCTL_OSCSRC_OSC1      0x00020000UL

//
// Enable/disable PLL
//
#define SYSCTL_PLL_ENABLE       0x80000000U //!< Enable PLL
#define SYSCTL_PLL_DISABLE      0x00000000U //!< Disable PLL
#define SYSCTL_PLL_BYPASS       0x40000000U //!< Bypass PLL

//
//Mask to check the PLL configuration selected
//

#define SYSCTL_PLL_CONFIG_M     0xC0000000U

#define SYSCTL_DCC_COUNTER0_TOLERANCE   1U
#define SYSCTL_DCC_COUNTER0_WINDOW      1000U   // DCC Counter0 Window
//*****************************************************************************
//
// The following are values that can be passed to the SysCtl_setAuxClock() API
// as the config parameter.
//
//*****************************************************************************
//
// Auxiliary clock divider (AUXCLKDIV)
//
#define SYSCTL_AUXPLL_DIV_1        0x00000000U //!< Auxiliary PLL divide by 1
#define SYSCTL_AUXPLL_DIV_2        0x00000100U //!< Auxiliary PLL divide by 2
#define SYSCTL_AUXPLL_DIV_4        0x00000200U //!< Auxiliary PLL divide by 4
#define SYSCTL_AUXPLL_DIV_8        0x00000300U //!< Auxiliary PLL divide by 8
#define SYSCTL_AUXPLL_DIV_3        0x00000400U //!< Auxiliary PLL divide by 3
#define SYSCTL_AUXPLL_DIV_5        0x00000500U //!< Auxiliary PLL divide by 5
#define SYSCTL_AUXPLL_DIV_6        0x00000600U //!< Auxiliary PLL divide by 6
#define SYSCTL_AUXPLL_DIV_7        0x00000700U //!< Auxiliary PLL divide by 7

//! Macro to format Clock divider value. x is a number from 1 to 32.
//!
#define SYSCTL_AUXPLL_REFDIV(x)      SYSCTL_REFDIV((x))

//! Macro to format Clock divider value. x is a number from 1 to 32.
//!
#define SYSCTL_AUXPLL_ODIV(x)        SYSCTL_ODIV((x))


//
// Integer multiplier (IMULT)
//
//! Macro to format integer multiplier value. x is a number from 1 to 127.
//!
#define SYSCTL_AUXPLL_IMULT(x)      SYSCTL_IMULT((x))

//
// Fractional multiplier (FMULT)
//
#define SYSCTL_AUXPLL_FMULT_NONE  0x00000000U //!< No fractional multiplier
#define SYSCTL_AUXPLL_FMULT_0     0x00000000U //!< No fractional multiplier
#define SYSCTL_AUXPLL_FMULT_1_4   0x00004000U //!< Fractional multiplier - 0.25
#define SYSCTL_AUXPLL_FMULT_1_2   0x00008000U //!< Fractional multiplier - 0.50
#define SYSCTL_AUXPLL_FMULT_3_4   0x0000C000U //!< Fractional multiplier - 0.75

//
// Oscillator source
//
//! Internal oscillator INTOSC2 as auxiliary clock input
#define SYSCTL_AUXPLL_OSCSRC_OSC2       0x00000000UL
//! External oscillator (XTAL) as auxiliary clock input
#define SYSCTL_AUXPLL_OSCSRC_XTAL       0x00010000UL
//! AUXCLKIN (from GPIO) as auxiliary clock input
#define SYSCTL_AUXPLL_OSCSRC_AUXCLKIN   0x00020000U
//! External oscillator (XTAL) in single-ended mode
#define SYSCTL_AUXPLL_OSCSRC_XTAL_SE    0x00030000U

//
// Enable/disable PLL
//
#define SYSCTL_AUXPLL_ENABLE    0x80000000U //!< Enable AUXPLL
#define SYSCTL_AUXPLL_DISABLE   0x00000000U //!< Disable AUXPLL
#define SYSCTL_AUXPLL_BYPASS    0x40000000U //!< Bypass AUXPLL
//*****************************************************************************
//
// Values that can be passed to SysCtl_clearNMIStatus(),
// SysCtl_forceNMIFlags(), SysCtl_isNMIFlagSet(), and
// SysCtl_isNMIShadowFlagSet() as the nmiFlags parameter and returned by
// SysCtl_getNMIFlagStatus() and SysCtl_getNMIShadowFlagStatus().
//
//*****************************************************************************
#define SYSCTL_NMI_NMIINT           0x1U //!<  NMI Interrupt Flag
#define SYSCTL_NMI_CLOCKFAIL        0x2U //!<  Clock Fail Interrupt Flag
#define SYSCTL_NMI_RAMUNCERR        0x4U //!<  RAM Uncorrectable Error NMI Flag
#define SYSCTL_NMI_FLUNCERR         0x8U //!<  Flash Uncorrectable Error NMI Flag
#define SYSCTL_NMI_CPU1HWBISTERR    0x10U //!<  HW BIST Error NMI Flag
#define SYSCTL_NMI_CPU2HWBISTERR    0x20U //!<  HW BIST Error NMI Flag
#define SYSCTL_NMI_PIEVECTERR       0x40U //!<  PIE Vector Fetch Error Flag
#define SYSCTL_NMI_ERADNMI          0x80U //!<  ERAD Module NMI Flag
#define SYSCTL_NMI_CLBNMI           0x100U //!<  Configurable Logic Block NMI Flag
#define SYSCTL_NMI_CPU2WDRSN        0x200U //!<  CPU2 WDRSn Reset Indication Flag
#define SYSCTL_NMI_CPU2NMIWDRSN     0x400U //!<  CPU2 NMIWDRSn Reset Indication Flag
#define SYSCTL_NMI_CMNMIWDRSN       0x1000U //!<  CM NMI watch dog has timed out.
#define SYSCTL_NMI_ECATNMIN         0x2000U //!<  NMI from EtherCAT reset out
#define SYSCTL_NMI_CRC_FAIL         0x4000U //!<  CRC calculation failed.
#define SYSCTL_NMI_MCAN_ERR         0x8000U //!<  MCAN module generated an ECC error.

//*****************************************************************************
//
// Values that can be passed to SysCtl_enableCMtoCPUNMI() &
// SysCtl_enableCMtoCPUInterrupt() as the Flags parameter
//
//*****************************************************************************

#define SYSCTL_FLAG_CMNMIWDRST       0x0004U //!< CM NMIWD Reset Indication

//*****************************************************************************
//
// Values that can be passed to/returned from SysCtl_getCMInterruptStatus()
// SysCtl_clearCMInterruptStatus() or SysCtl_setCMInterruptStatus()
// as the intFlags parameter
//
//*****************************************************************************
#define SYSCTL_STATUS_CMGINT          0x0001U //!< CM Global interrupt
#define SYSCTL_STATUS_CMNMIWDRST      0x0002U //!< CM NMIWD Interrupt
#define SYSCTL_STATUS_CMSYSRESETREQ   0x0004U //!< CM System Reset Interrupt
#define SYSCTL_STATUS_CMVECTRESET     0x0008U //!< CM Vector Reset Interrupt

//*****************************************************************************
//
// Values that can be passed to/returned from SysCtl_getInterruptStatus()
// SysCtl_clearInterruptStatus() or SysCtl_setInterruptStatus()
// as the intFlags parameter
//
//*****************************************************************************
#define SYSCTL_STATUS_GINT                         0x1U //!<  Global Interrupt flag
#define SYSCTL_STATUS_EMIF_ERR                     0x2U //!<  EMIF error event flag
#define SYSCTL_STATUS_RAM_CORRECTABLE_ERR          0x4U //!<  RAM correctable error flag
#define SYSCTL_STATUS_FLASH_CORRECTABLE_ERR        0x8U //!<  FLASH correctable error flag
#define SYSCTL_STATUS_RAM_ACC_VIOL                 0x10U //!<  RAM access vioation flag.
#define SYSCTL_STATUS_DCC0                         0x80U //!<  DCC0 Interrupt flag.
#define SYSCTL_STATUS_DCC1                         0x100U //!<  DCC1 Interrupt flag.
#define SYSCTL_STATUS_DCC2                         0x200U //!<  DCC2 Interrupt flag.


//*****************************************************************************
//
// The following are values that can be passed to the SysCtl_clearResetCause()
// or SysCtl_simulateReset()
// API as rstCauses or returned by the SysCtl_getResetCause() API.
//
//*****************************************************************************
#define SYSCTL_CAUSE_POR                 0x00000001U //!< Power-on reset
#define SYSCTL_CAUSE_XRS                 0x00000002U //!< External reset pin
#define SYSCTL_CAUSE_WDRS                0x00000004U //!< Watchdog reset
#define SYSCTL_CAUSE_NMIWDRS             0x00000008U //!< NMI watchdog reset
#define SYSCTL_CAUSE_SCCRESET            0x00000100U //!< SCCRESETn by DCSM
#define SYSCTL_CAUSE_HWBISTN             0x00000020U //!< HWBISTn Reset
#define SYSCTL_CAUSE_ECAT_RESET_OUT      0x00000200U //!< ECAT_RESET_OUT Reset
#define SYSCTL_CAUSE_SIMRESET_CPU1RSN    0x00000400U //!< SIMRESET_CPU1 Reset
#define SYSCTL_CAUSE_SIMRESET_XRSN       0x00000800U //!< SIMRESET_XRSn Reset
#define SYSCTL_CAUSE_CPU1RSN             0x00000001U //!< Simulated CPU1Reset

//*****************************************************************************
//
// The following are one of the values that can be passed
// to the SysCtl_clearCPU2ResetStatus() API as rstCauses or returned
// by the SysCtl_getCPU2ResetStatus() API.
//
//*****************************************************************************
#define SYSCTL_RSTSTAT_CPU2HWBISTRST     0xCU

//*****************************************************************************
//
// The following values define the adcsocSrc parameter for
// SysCtl_enableExtADCSOCSource() and SysCtl_disableExtADCSOCSource().
//
//*****************************************************************************
#define SYSCTL_ADCSOC_SRC_PWM1SOCA   0x1U //!<ePWM1 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM2SOCA   0x2U //!<ePWM2 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM3SOCA   0x4U //!<ePWM3 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM4SOCA   0x8U //!<ePWM4 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM5SOCA   0x10U //!<ePWM5 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM6SOCA   0x20U //!<ePWM6 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM7SOCA   0x40U //!<ePWM7 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM8SOCA   0x80U //!<ePWM8 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM9SOCA   0x100U //!<ePWM9 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM10SOCA   0x200U //!<ePWM10 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM11SOCA   0x400U //!<ePWM11 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM12SOCA   0x800U //!<ePWM12 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM13SOCA   0x1000U //!<ePWM13 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM14SOCA   0x2000U //!<ePWM14 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM15SOCA   0x4000U //!<ePWM15 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM16SOCA   0x8000U //!<ePWM16 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM1SOCB   0x10000U //!<ePWM1 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM2SOCB   0x20000U //!<ePWM2 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM3SOCB   0x40000U //!<ePWM3 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM4SOCB   0x80000U //!<ePWM4 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM5SOCB   0x100000U //!<ePWM5 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM6SOCB   0x200000U //!<ePWM6 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM7SOCB   0x400000U //!<ePWM7 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM8SOCB   0x800000U //!<ePWM8 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM9SOCB   0x1000000U //!<ePWM9 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM10SOCB   0x2000000U //!<ePWM10 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM11SOCB   0x4000000U //!<ePWM11 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM12SOCB   0x8000000U //!<ePWM12 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM13SOCB   0x10000000U //!<ePWM13 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM14SOCB   0x20000000U //!<ePWM14 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM15SOCB   0x40000000U //!<ePWM15 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM16SOCB   0x80000000U //!<ePWM16 SOCB for ADCSOCBO
#endif

//*****************************************************************************
//
//! The following are values that can be passed to SysCtl_enablePeripheral()
//! and SysCtl_disablePeripheral() as the \e peripheral parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_PERIPH_CLK_CLA1 = 0x0000, //!< CLA1 clock
    SYSCTL_PERIPH_CLK_DMA = 0x0200, //!< DMA clock
    SYSCTL_PERIPH_CLK_TIMER0 = 0x0300, //!< CPUTIMER0 clock
    SYSCTL_PERIPH_CLK_TIMER1 = 0x0400, //!< CPUTIMER1 clock
    SYSCTL_PERIPH_CLK_TIMER2 = 0x0500, //!< CPUTIMER2 clock
    SYSCTL_PERIPH_CLK_CPUBGCRC = 0x0D00, //!< CPUBGCRC clock
    SYSCTL_PERIPH_CLK_CLA1BGCRC = 0x0E00, //!< CLA1BGCRC clock
    SYSCTL_PERIPH_CLK_HRCAL = 0x1000, //!< HRCAL clock
    SYSCTL_PERIPH_CLK_TBCLKSYNC = 0x1200, //!< TBCLKSYNC clock
    SYSCTL_PERIPH_CLK_GTBCLKSYNC = 0x1300, //!< GTBCLKSYNC clock
    SYSCTL_PERIPH_CLK_ERAD = 0x1800, //!< ERAD clock
    SYSCTL_PERIPH_CLK_EMIF1 = 0x0001, //!< EMIF1 clock
    SYSCTL_PERIPH_CLK_EMIF2 = 0x0101, //!< EMIF2 clock
    SYSCTL_PERIPH_CLK_EPWM1 = 0x0002, //!< EPWM1 clock
    SYSCTL_PERIPH_CLK_EPWM2 = 0x0102, //!< EPWM2 clock
    SYSCTL_PERIPH_CLK_EPWM3 = 0x0202, //!< EPWM3 clock
    SYSCTL_PERIPH_CLK_EPWM4 = 0x0302, //!< EPWM4 clock
    SYSCTL_PERIPH_CLK_EPWM5 = 0x0402, //!< EPWM5 clock
    SYSCTL_PERIPH_CLK_EPWM6 = 0x0502, //!< EPWM6 clock
    SYSCTL_PERIPH_CLK_EPWM7 = 0x0602, //!< EPWM7 clock
    SYSCTL_PERIPH_CLK_EPWM8 = 0x0702, //!< EPWM8 clock
    SYSCTL_PERIPH_CLK_EPWM9 = 0x0802, //!< EPWM9 clock
    SYSCTL_PERIPH_CLK_EPWM10 = 0x0902, //!< EPWM10 clock
    SYSCTL_PERIPH_CLK_EPWM11 = 0x0A02, //!< EPWM11 clock
    SYSCTL_PERIPH_CLK_EPWM12 = 0x0B02, //!< EPWM12 clock
    SYSCTL_PERIPH_CLK_EPWM13 = 0x0C02, //!< EPWM13 clock
    SYSCTL_PERIPH_CLK_EPWM14 = 0x0D02, //!< EPWM14 clock
    SYSCTL_PERIPH_CLK_EPWM15 = 0x0E02, //!< EPWM15 clock
    SYSCTL_PERIPH_CLK_EPWM16 = 0x0F02, //!< EPWM16 clock
    SYSCTL_PERIPH_CLK_ECAP1 = 0x0003, //!< ECAP1 clock
    SYSCTL_PERIPH_CLK_ECAP2 = 0x0103, //!< ECAP2 clock
    SYSCTL_PERIPH_CLK_ECAP3 = 0x0203, //!< ECAP3 clock
    SYSCTL_PERIPH_CLK_ECAP4 = 0x0303, //!< ECAP4 clock
    SYSCTL_PERIPH_CLK_ECAP5 = 0x0403, //!< ECAP5 clock
    SYSCTL_PERIPH_CLK_ECAP6 = 0x0503, //!< ECAP6 clock
    SYSCTL_PERIPH_CLK_ECAP7 = 0x0603, //!< ECAP7 clock
    SYSCTL_PERIPH_CLK_EQEP1 = 0x0004, //!< EQEP1 clock
    SYSCTL_PERIPH_CLK_EQEP2 = 0x0104, //!< EQEP2 clock
    SYSCTL_PERIPH_CLK_EQEP3 = 0x0204, //!< EQEP3 clock
    SYSCTL_PERIPH_CLK_SD1 = 0x0006, //!< SD1 clock
    SYSCTL_PERIPH_CLK_SD2 = 0x0106, //!< SD2 clock
    SYSCTL_PERIPH_CLK_SCIA = 0x0007, //!< SCI_A clock
    SYSCTL_PERIPH_CLK_SCIB = 0x0107, //!< SCI_B clock
    SYSCTL_PERIPH_CLK_SCIC = 0x0207, //!< SCI_C clock
    SYSCTL_PERIPH_CLK_SCID = 0x0307, //!< SCI_D clock
    SYSCTL_PERIPH_CLK_SPIA = 0x0008, //!< SPI_A clock
    SYSCTL_PERIPH_CLK_SPIB = 0x0108, //!< SPI_B clock
    SYSCTL_PERIPH_CLK_SPIC = 0x0208, //!< SPI_C clock
    SYSCTL_PERIPH_CLK_SPID = 0x0308, //!< SPI_D clock
    SYSCTL_PERIPH_CLK_I2CA = 0x0009, //!< I2C_A clock
    SYSCTL_PERIPH_CLK_I2CB = 0x0109, //!< I2C_B clock
    SYSCTL_PERIPH_CLK_CANA = 0x000A, //!< CAN_A clock
    SYSCTL_PERIPH_CLK_CANB = 0x010A, //!< CAN_B clock
    SYSCTL_PERIPH_CLK_MCANA = 0x040A, //!< MCAN_A clock
    SYSCTL_PERIPH_CLK_MCBSPA = 0x000B, //!< MCBSP_A clock
    SYSCTL_PERIPH_CLK_MCBSPB = 0x010B, //!< MCBSP_B clock
    SYSCTL_PERIPH_CLK_USBA = 0x100B, //!< USB_A clock
    SYSCTL_PERIPH_CLK_ADCA = 0x000D, //!< ADC_A clock
    SYSCTL_PERIPH_CLK_ADCB = 0x010D, //!< ADC_B clock
    SYSCTL_PERIPH_CLK_ADCC = 0x020D, //!< ADC_C clock
    SYSCTL_PERIPH_CLK_ADCD = 0x030D, //!< ADC_D clock
    SYSCTL_PERIPH_CLK_CMPSS1 = 0x000E, //!< CMPSS1 clock
    SYSCTL_PERIPH_CLK_CMPSS2 = 0x010E, //!< CMPSS2 clock
    SYSCTL_PERIPH_CLK_CMPSS3 = 0x020E, //!< CMPSS3 clock
    SYSCTL_PERIPH_CLK_CMPSS4 = 0x030E, //!< CMPSS4 clock
    SYSCTL_PERIPH_CLK_CMPSS5 = 0x040E, //!< CMPSS5 clock
    SYSCTL_PERIPH_CLK_CMPSS6 = 0x050E, //!< CMPSS6 clock
    SYSCTL_PERIPH_CLK_CMPSS7 = 0x060E, //!< CMPSS7 clock
    SYSCTL_PERIPH_CLK_CMPSS8 = 0x070E, //!< CMPSS8 clock
    SYSCTL_PERIPH_CLK_DACA = 0x1010, //!< DAC_A clock
    SYSCTL_PERIPH_CLK_DACB = 0x1110, //!< DAC_B clock
    SYSCTL_PERIPH_CLK_DACC = 0x1210, //!< DAC_C clock
    SYSCTL_PERIPH_CLK_CLB1 = 0x0011, //!< CLB1 clock
    SYSCTL_PERIPH_CLK_CLB2 = 0x0111, //!< CLB2 clock
    SYSCTL_PERIPH_CLK_CLB3 = 0x0211, //!< CLB3 clock
    SYSCTL_PERIPH_CLK_CLB4 = 0x0311, //!< CLB4 clock
    SYSCTL_PERIPH_CLK_CLB5 = 0x0411, //!< CLB5 clock
    SYSCTL_PERIPH_CLK_CLB6 = 0x0511, //!< CLB6 clock
    SYSCTL_PERIPH_CLK_CLB7 = 0x0611, //!< CLB7 clock
    SYSCTL_PERIPH_CLK_CLB8 = 0x0711, //!< CLB8 clock
    SYSCTL_PERIPH_CLK_FSITXA = 0x0012, //!< FSITX_A clock
    SYSCTL_PERIPH_CLK_FSITXB = 0x0112, //!< FSITX_B clock
    SYSCTL_PERIPH_CLK_FSIRXA = 0x1012, //!< FSIRX_A clock
    SYSCTL_PERIPH_CLK_FSIRXB = 0x1112, //!< FSIRX_B clock
    SYSCTL_PERIPH_CLK_FSIRXC = 0x1212, //!< FSIRX_C clock
    SYSCTL_PERIPH_CLK_FSIRXD = 0x1312, //!< FSIRX_D clock
    SYSCTL_PERIPH_CLK_FSIRXE = 0x1412, //!< FSIRX_E clock
    SYSCTL_PERIPH_CLK_FSIRXF = 0x1512, //!< FSIRX_F clock
    SYSCTL_PERIPH_CLK_FSIRXG = 0x1612, //!< FSIRX_G clock
    SYSCTL_PERIPH_CLK_FSIRXH = 0x1712, //!< FSIRX_H clock
    SYSCTL_PERIPH_CLK_PMBUSA = 0x0014, //!< PMBUS_A clock
    SYSCTL_PERIPH_CLK_DCC0 = 0x0015, //!< DCC0 clock
    SYSCTL_PERIPH_CLK_DCC1 = 0x0115, //!< DCC1 clock
    SYSCTL_PERIPH_CLK_DCC2 = 0x0215, //!< DCC2 clock
    SYSCTL_PERIPH_CLK_MPOSTCLK = 0x0016, //!< MPOSTCLK clock
    SYSCTL_PERIPH_CLK_ECAT = 0x0017  //!< ETHERCAT clock
} SysCtl_PeripheralPCLOCKCR;

//*****************************************************************************
//
//! The following are values that can be passed to SysCtl_resetPeripheral() as
//! the \e peripheral parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_PERIPH_RES_CPU1CLA1 = 0x0000, //!< Reset CPU1_CLA1 clock
    SYSCTL_PERIPH_RES_CPU2CLA1 = 0x0200, //!< Reset CPU2_CLA1 clock
    SYSCTL_PERIPH_RES_CPU1CPUBGCRC = 0x0D00, //!< Reset CPU1_CPUBGCRC clock
    SYSCTL_PERIPH_RES_CPU1CLA1BGCRC = 0x0E00, //!< Reset CPU1_CLA1BGCRC clock
    SYSCTL_PERIPH_RES_CPU2CPUBGCRC = 0x1000, //!< Reset CPU2_CPUBGCRC clock
    SYSCTL_PERIPH_RES_CPU2CLA1BGCRC = 0x1100, //!< Reset CPU2_CLA1BGCRC clock
    SYSCTL_PERIPH_RES_CPU1ERAD = 0x1800, //!< Reset CPU1_ERAD clock
    SYSCTL_PERIPH_RES_CPU2ERAD = 0x1900, //!< Reset CPU2_ERAD clock
    SYSCTL_PERIPH_RES_EMIF1 = 0x0001, //!< Reset EMIF1 clock
    SYSCTL_PERIPH_RES_EMIF2 = 0x0101, //!< Reset EMIF2 clock
    SYSCTL_PERIPH_RES_EPWM1 = 0x0002, //!< Reset EPWM1 clock
    SYSCTL_PERIPH_RES_EPWM2 = 0x0102, //!< Reset EPWM2 clock
    SYSCTL_PERIPH_RES_EPWM3 = 0x0202, //!< Reset EPWM3 clock
    SYSCTL_PERIPH_RES_EPWM4 = 0x0302, //!< Reset EPWM4 clock
    SYSCTL_PERIPH_RES_EPWM5 = 0x0402, //!< Reset EPWM5 clock
    SYSCTL_PERIPH_RES_EPWM6 = 0x0502, //!< Reset EPWM6 clock
    SYSCTL_PERIPH_RES_EPWM7 = 0x0602, //!< Reset EPWM7 clock
    SYSCTL_PERIPH_RES_EPWM8 = 0x0702, //!< Reset EPWM8 clock
    SYSCTL_PERIPH_RES_EPWM9 = 0x0802, //!< Reset EPWM9 clock
    SYSCTL_PERIPH_RES_EPWM10 = 0x0902, //!< Reset EPWM10 clock
    SYSCTL_PERIPH_RES_EPWM11 = 0x0A02, //!< Reset EPWM11 clock
    SYSCTL_PERIPH_RES_EPWM12 = 0x0B02, //!< Reset EPWM12 clock
    SYSCTL_PERIPH_RES_EPWM13 = 0x0C02, //!< Reset EPWM13 clock
    SYSCTL_PERIPH_RES_EPWM14 = 0x0D02, //!< Reset EPWM14 clock
    SYSCTL_PERIPH_RES_EPWM15 = 0x0E02, //!< Reset EPWM15 clock
    SYSCTL_PERIPH_RES_EPWM16 = 0x0F02, //!< Reset EPWM16 clock
    SYSCTL_PERIPH_RES_ECAP1 = 0x0003, //!< Reset ECAP1 clock
    SYSCTL_PERIPH_RES_ECAP2 = 0x0103, //!< Reset ECAP2 clock
    SYSCTL_PERIPH_RES_ECAP3 = 0x0203, //!< Reset ECAP3 clock
    SYSCTL_PERIPH_RES_ECAP4 = 0x0303, //!< Reset ECAP4 clock
    SYSCTL_PERIPH_RES_ECAP5 = 0x0403, //!< Reset ECAP5 clock
    SYSCTL_PERIPH_RES_ECAP6 = 0x0503, //!< Reset ECAP6 clock
    SYSCTL_PERIPH_RES_ECAP7 = 0x0603, //!< Reset ECAP7 clock
    SYSCTL_PERIPH_RES_EQEP1 = 0x0004, //!< Reset EQEP1 clock
    SYSCTL_PERIPH_RES_EQEP2 = 0x0104, //!< Reset EQEP2 clock
    SYSCTL_PERIPH_RES_EQEP3 = 0x0204, //!< Reset EQEP3 clock
    SYSCTL_PERIPH_RES_SD1 = 0x0006, //!< Reset SD1 clock
    SYSCTL_PERIPH_RES_SD2 = 0x0106, //!< Reset SD2 clock
    SYSCTL_PERIPH_RES_SCIA = 0x0007, //!< Reset SCI_A clock
    SYSCTL_PERIPH_RES_SCIB = 0x0107, //!< Reset SCI_B clock
    SYSCTL_PERIPH_RES_SCIC = 0x0207, //!< Reset SCI_C clock
    SYSCTL_PERIPH_RES_SCID = 0x0307, //!< Reset SCI_D clock
    SYSCTL_PERIPH_RES_SPIA = 0x0008, //!< Reset SPI_A clock
    SYSCTL_PERIPH_RES_SPIB = 0x0108, //!< Reset SPI_B clock
    SYSCTL_PERIPH_RES_SPIC = 0x0208, //!< Reset SPI_C clock
    SYSCTL_PERIPH_RES_SPID = 0x0308, //!< Reset SPI_D clock
    SYSCTL_PERIPH_RES_I2CA = 0x0009, //!< Reset I2C_A clock
    SYSCTL_PERIPH_RES_I2CB = 0x0109, //!< Reset I2C_B clock
    SYSCTL_PERIPH_RES_CANA = 0x000A, //!< Reset CAN_A clock
    SYSCTL_PERIPH_RES_CANB = 0x010A, //!< Reset CAN_B clock
    SYSCTL_PERIPH_RES_MCANA = 0x040A, //!< Reset MCAN_A clock
    SYSCTL_PERIPH_RES_MCBSPA = 0x000B, //!< Reset MCBSP_A clock
    SYSCTL_PERIPH_RES_MCBSPB = 0x010B, //!< Reset MCBSP_B clock
    SYSCTL_PERIPH_RES_USBA = 0x100B, //!< Reset USB_A clock
    SYSCTL_PERIPH_RES_ADCA = 0x000D, //!< Reset ADC_A clock
    SYSCTL_PERIPH_RES_ADCB = 0x010D, //!< Reset ADC_B clock
    SYSCTL_PERIPH_RES_ADCC = 0x020D, //!< Reset ADC_C clock
    SYSCTL_PERIPH_RES_ADCD = 0x030D, //!< Reset ADC_D clock
    SYSCTL_PERIPH_RES_CMPSS1 = 0x000E, //!< Reset CMPSS1 clock
    SYSCTL_PERIPH_RES_CMPSS2 = 0x010E, //!< Reset CMPSS2 clock
    SYSCTL_PERIPH_RES_CMPSS3 = 0x020E, //!< Reset CMPSS3 clock
    SYSCTL_PERIPH_RES_CMPSS4 = 0x030E, //!< Reset CMPSS4 clock
    SYSCTL_PERIPH_RES_CMPSS5 = 0x040E, //!< Reset CMPSS5 clock
    SYSCTL_PERIPH_RES_CMPSS6 = 0x050E, //!< Reset CMPSS6 clock
    SYSCTL_PERIPH_RES_CMPSS7 = 0x060E, //!< Reset CMPSS7 clock
    SYSCTL_PERIPH_RES_CMPSS8 = 0x070E, //!< Reset CMPSS8 clock
    SYSCTL_PERIPH_RES_DACA = 0x1010, //!< Reset DAC_A clock
    SYSCTL_PERIPH_RES_DACB = 0x1110, //!< Reset DAC_B clock
    SYSCTL_PERIPH_RES_DACC = 0x1210, //!< Reset DAC_C clock
    SYSCTL_PERIPH_RES_CLB1 = 0x0011, //!< Reset CLB1 clock
    SYSCTL_PERIPH_RES_CLB2 = 0x0111, //!< Reset CLB2 clock
    SYSCTL_PERIPH_RES_CLB3 = 0x0211, //!< Reset CLB3 clock
    SYSCTL_PERIPH_RES_CLB4 = 0x0311, //!< Reset CLB4 clock
    SYSCTL_PERIPH_RES_CLB5 = 0x0411, //!< Reset CLB5 clock
    SYSCTL_PERIPH_RES_CLB6 = 0x0511, //!< Reset CLB6 clock
    SYSCTL_PERIPH_RES_CLB7 = 0x0611, //!< Reset CLB7 clock
    SYSCTL_PERIPH_RES_CLB8 = 0x0711, //!< Reset CLB8 clock
    SYSCTL_PERIPH_RES_FSITXA = 0x0012, //!< Reset FSITX_A clock
    SYSCTL_PERIPH_RES_FSITXB = 0x0112, //!< Reset FSITX_B clock
    SYSCTL_PERIPH_RES_FSIRXA = 0x1012, //!< Reset FSIRX_A clock
    SYSCTL_PERIPH_RES_FSIRXB = 0x1112, //!< Reset FSIRX_B clock
    SYSCTL_PERIPH_RES_FSIRXC = 0x1212, //!< Reset FSIRX_C clock
    SYSCTL_PERIPH_RES_FSIRXD = 0x1312, //!< Reset FSIRX_D clock
    SYSCTL_PERIPH_RES_FSIRXE = 0x1412, //!< Reset FSIRX_E clock
    SYSCTL_PERIPH_RES_FSIRXF = 0x1512, //!< Reset FSIRX_F clock
    SYSCTL_PERIPH_RES_FSIRXG = 0x1612, //!< Reset FSIRX_G clock
    SYSCTL_PERIPH_RES_FSIRXH = 0x1712, //!< Reset FSIRX_H clock
    SYSCTL_PERIPH_RES_PMBUSA = 0x0014, //!< Reset PMBUS_A clock
    SYSCTL_PERIPH_RES_DCC0 = 0x0015, //!< Reset DCC0 clock
    SYSCTL_PERIPH_RES_DCC1 = 0x0115, //!< Reset DCC1 clock
    SYSCTL_PERIPH_RES_DCC2 = 0x0215, //!< Reset DCC2 clock
    SYSCTL_PERIPH_RES_ECAT = 0x0017  //!< Reset ETHERCAT clock
} SysCtl_PeripheralSOFTPRES;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_lockCPUSelectRegs() as the \e peripheral parameter.
//
//*****************************************************************************
typedef enum
{
    //! Configure CPU Select for EPWM
    SYSCTL_CPUSEL0_EPWM      = 0x0U,
    //! Configure CPU Select for ECAP
    SYSCTL_CPUSEL1_ECAP      = 0x1U,
    //! Configure CPU Select for EQEP
    SYSCTL_CPUSEL2_EQEP      = 0x2U,
    //! Configure CPU Select for SD
    SYSCTL_CPUSEL4_SD        = 0x4U,
    //! Configure CPU Select for SCI
    SYSCTL_CPUSEL5_SCI       = 0x5U,
    //! Configure CPU Select for SPI
    SYSCTL_CPUSEL6_SPI       = 0x6U,
    //! Configure CPU Select for I2C
    SYSCTL_CPUSEL7_I2C       = 0x7U,
    //! Configure CPU Select for CAN
    SYSCTL_CPUSEL8_CAN       = 0x8U,
    //! Configure CPU Select for MCBSP
    SYSCTL_CPUSEL9_MCBSP     = 0x9U,
    //! Configure CPU Select for ADC
    SYSCTL_CPUSEL11_ADC      = 0xBU,
    //! Configure CPU Select for CMPSS
    SYSCTL_CPUSEL12_CMPSS    = 0xCU,
    //! Configure CPU Select for DAC
    SYSCTL_CPUSEL14_DAC      = 0xEU,
    //! Configure CPU Select for CLB
    SYSCTL_CPUSEL15_CLB      = 0xFU,
    //! Configure CPU Select for FSI
    SYSCTL_CPUSEL16_FSI      = 0x10U,
    //! Configure CPU Select for PMBUS
    SYSCTL_CPUSEL18_PMBUS    = 0x12U,
    //! Configure CPU Select for HRCAL
    SYSCTL_CPUSEL25_HRCAL    = 0x19U
} SysCtl_CPUSelPeripheral;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_selectCPUForPeripheralInstance() as the \e peripheral parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_CPUSEL_EPWM1 = 0x0000,
    SYSCTL_CPUSEL_EPWM2 = 0x0100,
    SYSCTL_CPUSEL_EPWM3 = 0x0200,
    SYSCTL_CPUSEL_EPWM4 = 0x0300,
    SYSCTL_CPUSEL_EPWM5 = 0x0400,
    SYSCTL_CPUSEL_EPWM6 = 0x0500,
    SYSCTL_CPUSEL_EPWM7 = 0x0600,
    SYSCTL_CPUSEL_EPWM8 = 0x0700,
    SYSCTL_CPUSEL_EPWM9 = 0x0800,
    SYSCTL_CPUSEL_EPWM10 = 0x0900,
    SYSCTL_CPUSEL_EPWM11 = 0x0A00,
    SYSCTL_CPUSEL_EPWM12 = 0x0B00,
    SYSCTL_CPUSEL_EPWM13 = 0x0C00,
    SYSCTL_CPUSEL_EPWM14 = 0x0D00,
    SYSCTL_CPUSEL_EPWM15 = 0x0E00,
    SYSCTL_CPUSEL_EPWM16 = 0x0F00,
    SYSCTL_CPUSEL_ECAP1 = 0x0001,
    SYSCTL_CPUSEL_ECAP2 = 0x0101,
    SYSCTL_CPUSEL_ECAP3 = 0x0201,
    SYSCTL_CPUSEL_ECAP4 = 0x0301,
    SYSCTL_CPUSEL_ECAP5 = 0x0401,
    SYSCTL_CPUSEL_ECAP6 = 0x0501,
    SYSCTL_CPUSEL_ECAP7 = 0x0601,
    SYSCTL_CPUSEL_EQEP1 = 0x0002,
    SYSCTL_CPUSEL_EQEP2 = 0x0102,
    SYSCTL_CPUSEL_EQEP3 = 0x0202,
    SYSCTL_CPUSEL_SD1 = 0x0004,
    SYSCTL_CPUSEL_SD2 = 0x0104,
    SYSCTL_CPUSEL_SCIA = 0x0005,
    SYSCTL_CPUSEL_SCIB = 0x0105,
    SYSCTL_CPUSEL_SCIC = 0x0205,
    SYSCTL_CPUSEL_SCID = 0x0305,
    SYSCTL_CPUSEL_SPIA = 0x0006,
    SYSCTL_CPUSEL_SPIB = 0x0106,
    SYSCTL_CPUSEL_SPIC = 0x0206,
    SYSCTL_CPUSEL_SPID = 0x0306,
    SYSCTL_CPUSEL_I2CA = 0x0007,
    SYSCTL_CPUSEL_I2CB = 0x0107,
    SYSCTL_CPUSEL_CANA = 0x0008,
    SYSCTL_CPUSEL_CANB = 0x0108,
    SYSCTL_CPUSEL_MCANA = 0x0408,
    SYSCTL_CPUSEL_MCBSPA = 0x0009,
    SYSCTL_CPUSEL_MCBSPB = 0x0109,
    SYSCTL_CPUSEL_ADCA = 0x000B,
    SYSCTL_CPUSEL_ADCB = 0x010B,
    SYSCTL_CPUSEL_ADCC = 0x020B,
    SYSCTL_CPUSEL_ADCD = 0x030B,
    SYSCTL_CPUSEL_CMPSS1 = 0x000C,
    SYSCTL_CPUSEL_CMPSS2 = 0x010C,
    SYSCTL_CPUSEL_CMPSS3 = 0x020C,
    SYSCTL_CPUSEL_CMPSS4 = 0x030C,
    SYSCTL_CPUSEL_CMPSS5 = 0x040C,
    SYSCTL_CPUSEL_CMPSS6 = 0x050C,
    SYSCTL_CPUSEL_CMPSS7 = 0x060C,
    SYSCTL_CPUSEL_CMPSS8 = 0x070C,
    SYSCTL_CPUSEL_DACA = 0x100E,
    SYSCTL_CPUSEL_DACB = 0x110E,
    SYSCTL_CPUSEL_DACC = 0x120E,
    SYSCTL_CPUSEL_CLB1 = 0x000F,
    SYSCTL_CPUSEL_CLB2 = 0x010F,
    SYSCTL_CPUSEL_CLB3 = 0x020F,
    SYSCTL_CPUSEL_CLB4 = 0x030F,
    SYSCTL_CPUSEL_CLB5 = 0x040F,
    SYSCTL_CPUSEL_CLB6 = 0x050F,
    SYSCTL_CPUSEL_CLB7 = 0x060F,
    SYSCTL_CPUSEL_CLB8 = 0x070F,
    SYSCTL_CPUSEL_FSITXA = 0x0010,
    SYSCTL_CPUSEL_FSITXB = 0x0110,
    SYSCTL_CPUSEL_FSIRXA = 0x1010,
    SYSCTL_CPUSEL_FSIRXB = 0x1110,
    SYSCTL_CPUSEL_FSIRXC = 0x1210,
    SYSCTL_CPUSEL_FSIRXD = 0x1310,
    SYSCTL_CPUSEL_FSIRXE = 0x1410,
    SYSCTL_CPUSEL_FSIRXF = 0x1510,
    SYSCTL_CPUSEL_FSIRXG = 0x1610,
    SYSCTL_CPUSEL_FSIRXH = 0x1710,
    SYSCTL_CPUSEL_PMBUSA = 0x0012,
    SYSCTL_CPUSEL_HRCALA = 0x0019
} SysCtl_CPUSelPeriphInstance;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_selectCPUForPeripheral() as \e cpuInst parameter.
//
//*****************************************************************************
typedef enum
{
    //! Connect the peripheral (indicated by SysCtl_CPUSelPeripheral) to CPU1
    SYSCTL_CPUSEL_CPU1 = 0x0U,
    //! Connect the peripheral (indicated by SysCtl_CPUSelPeripheral) to CPU2
    SYSCTL_CPUSEL_CPU2 = 0x1U
} SysCtl_CPUSel;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setWatchdogPredivider() as the \e predivider parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_WD_PREDIV_2    = 0x800,  //!< PREDIVCLK = INTOSC1 / 2
    SYSCTL_WD_PREDIV_4    = 0x900,  //!< PREDIVCLK = INTOSC1 / 4
    SYSCTL_WD_PREDIV_8    = 0xA00,  //!< PREDIVCLK = INTOSC1 / 8
    SYSCTL_WD_PREDIV_16   = 0xB00,  //!< PREDIVCLK = INTOSC1 / 16
    SYSCTL_WD_PREDIV_32   = 0xC00,  //!< PREDIVCLK = INTOSC1 / 32
    SYSCTL_WD_PREDIV_64   = 0xD00,  //!< PREDIVCLK = INTOSC1 / 64
    SYSCTL_WD_PREDIV_128  = 0xE00,  //!< PREDIVCLK = INTOSC1 / 128
    SYSCTL_WD_PREDIV_256  = 0xF00,  //!< PREDIVCLK = INTOSC1 / 256
    SYSCTL_WD_PREDIV_512  = 0x000,  //!< PREDIVCLK = INTOSC1 / 512
    SYSCTL_WD_PREDIV_1024 = 0x100,  //!< PREDIVCLK = INTOSC1 / 1024
    SYSCTL_WD_PREDIV_2048 = 0x200,  //!< PREDIVCLK = INTOSC1 / 2048
    SYSCTL_WD_PREDIV_4096 = 0x300   //!< PREDIVCLK = INTOSC1 / 4096
} SysCtl_WDPredivider;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setWatchdogPrescaler() as the \e prescaler parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_WD_PRESCALE_1  = 1,      //!< WDCLK = PREDIVCLK / 1
    SYSCTL_WD_PRESCALE_2  = 2,      //!< WDCLK = PREDIVCLK / 2
    SYSCTL_WD_PRESCALE_4  = 3,      //!< WDCLK = PREDIVCLK / 4
    SYSCTL_WD_PRESCALE_8  = 4,      //!< WDCLK = PREDIVCLK / 8
    SYSCTL_WD_PRESCALE_16 = 5,      //!< WDCLK = PREDIVCLK / 16
    SYSCTL_WD_PRESCALE_32 = 6,      //!< WDCLK = PREDIVCLK / 32
    SYSCTL_WD_PRESCALE_64 = 7       //!< WDCLK = PREDIVCLK / 64
} SysCtl_WDPrescaler;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setWatchdogMode() as the \e prescaler parameter.
//
//*****************************************************************************
typedef enum
{
    //! Watchdog can generate a reset signal
    SYSCTL_WD_MODE_RESET,
    //! Watchdog can generate an interrupt signal; reset signal is disabled
    SYSCTL_WD_MODE_INTERRUPT
} SysCtl_WDMode;

//*****************************************************************************
//
//! The following are values that can be passed to SysCtl_setLowSpeedClock() as
//! the \e prescaler parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_LSPCLK_PRESCALE_1    = 0,    //!< LSPCLK = SYSCLK / 1
    SYSCTL_LSPCLK_PRESCALE_2    = 1,    //!< LSPCLK = SYSCLK / 2
    SYSCTL_LSPCLK_PRESCALE_4    = 2,    //!< LSPCLK = SYSCLK / 4 (default)
    SYSCTL_LSPCLK_PRESCALE_6    = 3,    //!< LSPCLK = SYSCLK / 6
    SYSCTL_LSPCLK_PRESCALE_8    = 4,    //!< LSPCLK = SYSCLK / 8
    SYSCTL_LSPCLK_PRESCALE_10   = 5,    //!< LSPCLK = SYSCLK / 10
    SYSCTL_LSPCLK_PRESCALE_12   = 6,    //!< LSPCLK = SYSCLK / 12
    SYSCTL_LSPCLK_PRESCALE_14   = 7     //!< LSPCLK = SYSCLK / 14
} SysCtl_LSPCLKPrescaler;

//*****************************************************************************
//
//! The following are values that can be passed to SysCtl_setEPWMClockDivider()
//! as the \e divider parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_EPWMCLK_DIV_1,           //!< EPWMCLK = PLLSYSCLK / 1
    SYSCTL_EPWMCLK_DIV_2            //!< EPWMCLK = PLLSYSCLK / 2
} SysCtl_EPWMCLKDivider;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setEMIF1ClockDivider() as the \e divider parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_EMIF1CLK_DIV_1,           //!< EMIF1CLK = PLLSYSCLK / 1
    SYSCTL_EMIF1CLK_DIV_2            //!< EMIF1CLK = PLLSYSCLK / 2
} SysCtl_EMIF1CLKDivider;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setEMIF2ClockDivider() as the \e divider parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_EMIF2CLK_DIV_1,           //!< EMIF2CLK = PLLSYSCLK / 1
    SYSCTL_EMIF2CLK_DIV_2            //!< EMIF2CLK = PLLSYSCLK / 2
} SysCtl_EMIF2CLKDivider;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setPeripheralAccessControl() and SysCtl_getPeripheralAccessControl()
//! as the \e peripheral parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_ACCESS_ADCA          = 0x0, //!< ADCA access
    SYSCTL_ACCESS_ADCB          = 0x2, //!< ADCB access
    SYSCTL_ACCESS_ADCC          = 0x4, //!< ADCC access
    SYSCTL_ACCESS_ADCD          = 0x6, //!< ADCD access
    SYSCTL_ACCESS_CMPSS1        = 0x10, //!< CMPSS1 access
    SYSCTL_ACCESS_CMPSS2        = 0x12, //!< CMPSS2 access
    SYSCTL_ACCESS_CMPSS3        = 0x14, //!< CMPSS3 access
    SYSCTL_ACCESS_CMPSS4        = 0x16, //!< CMPSS4 access
    SYSCTL_ACCESS_CMPSS5        = 0x18, //!< CMPSS5 access
    SYSCTL_ACCESS_CMPSS6        = 0x1A, //!< CMPSS6 access
    SYSCTL_ACCESS_CMPSS7        = 0x1C, //!< CMPSS7 access
    SYSCTL_ACCESS_CMPSS8        = 0x1E, //!< CMPSS8 access
    SYSCTL_ACCESS_DACA          = 0x28, //!< DACA access
    SYSCTL_ACCESS_DACB          = 0x2A, //!< DACB access
    SYSCTL_ACCESS_DACC          = 0x2C, //!< DACC access
    SYSCTL_ACCESS_EPWM1         = 0x48, //!< EPWM1 access
    SYSCTL_ACCESS_EPWM2         = 0x4A, //!< EPWM2 access
    SYSCTL_ACCESS_EPWM3         = 0x4C, //!< EPWM3 access
    SYSCTL_ACCESS_EPWM4         = 0x4E, //!< EPWM4 access
    SYSCTL_ACCESS_EPWM5         = 0x50, //!< EPWM5 access
    SYSCTL_ACCESS_EPWM6         = 0x52, //!< EPWM6 access
    SYSCTL_ACCESS_EPWM7         = 0x54, //!< EPWM7 access
    SYSCTL_ACCESS_EPWM8         = 0x56, //!< EPWM8 access
    SYSCTL_ACCESS_EPWM9         = 0x58, //!< EPWM9 access
    SYSCTL_ACCESS_EPWM10        = 0x5A, //!< EPWM10 access
    SYSCTL_ACCESS_EPWM11        = 0x5C, //!< EPWM11 access
    SYSCTL_ACCESS_EPWM12        = 0x5E, //!< EPWM12 access
    SYSCTL_ACCESS_EPWM13        = 0x60, //!< EPWM13 access
    SYSCTL_ACCESS_EPWM14        = 0x62, //!< EPWM14 access
    SYSCTL_ACCESS_EPWM15        = 0x64, //!< EPWM15 access
    SYSCTL_ACCESS_EPWM16        = 0x66, //!< EPWM16 access
    SYSCTL_ACCESS_EQEP1         = 0x70, //!< EQEP1 access
    SYSCTL_ACCESS_EQEP2         = 0x72, //!< EQEP2 access
    SYSCTL_ACCESS_EQEP3         = 0x74, //!< EQEP3 access
    SYSCTL_ACCESS_ECAP1         = 0x80, //!< ECAP1 access
    SYSCTL_ACCESS_ECAP2         = 0x82, //!< ECAP2 access
    SYSCTL_ACCESS_ECAP3         = 0x84, //!< ECAP3 access
    SYSCTL_ACCESS_ECAP4         = 0x86, //!< ECAP4 access
    SYSCTL_ACCESS_ECAP5         = 0x88, //!< ECAP5 access
    SYSCTL_ACCESS_ECAP6         = 0x8A, //!< ECAP6 access
    SYSCTL_ACCESS_ECAP7         = 0x8C, //!< ECAP7 access
    SYSCTL_ACCESS_SDFM1         = 0xA8, //!< SDFM1 access
    SYSCTL_ACCESS_SDFM2         = 0xAA, //!< SDFM2 access
    SYSCTL_ACCESS_CLB1          = 0xB0, //!< CLB1 access
    SYSCTL_ACCESS_CLB2          = 0xB2, //!< CLB2 access
    SYSCTL_ACCESS_CLB3          = 0xB4, //!< CLB3 access
    SYSCTL_ACCESS_CLB4          = 0xB6, //!< CLB4 access
    SYSCTL_ACCESS_CLB5          = 0xB8, //!< CLB5 access
    SYSCTL_ACCESS_CLB6          = 0xBA, //!< CLB6 access
    SYSCTL_ACCESS_CLB7          = 0xBC, //!< CLB7 access
    SYSCTL_ACCESS_CLB8          = 0xBE, //!< CLB8 access
    SYSCTL_ACCESS_SPIA          = 0x110, //!< SPIA access
    SYSCTL_ACCESS_SPIB          = 0x112, //!< SPIB access
    SYSCTL_ACCESS_SPIC          = 0x114, //!< SPIC access
    SYSCTL_ACCESS_SPID          = 0x116, //!< SPID access
    SYSCTL_ACCESS_PMBUSA       = 0x130, //!< PMBUS_A access
    SYSCTL_ACCESS_CANA         = 0x140, //!< CAN_A access
    SYSCTL_ACCESS_CANB         = 0x142, //!< CAN_B access
    SYSCTL_ACCESS_MCBSPA        = 0x150, //!< MCBSPA access
    SYSCTL_ACCESS_MCBSPB        = 0x152, //!< MCBSPB access
    SYSCTL_ACCESS_USBA          = 0x180, //!< USBA access
    SYSCTL_ACCESS_HRPWM         = 0x1A8, //!< HRPWM access
    SYSCTL_ACCESS_ECAT      = 0x1AA, //!< ETHERCAT access
    SYSCTL_ACCESS_FSIATX        = 0x1B0, //!< FSIATX access
    SYSCTL_ACCESS_FSIARX        = 0x1B2, //!< FSIARX access
    SYSCTL_ACCESS_FSIBTX        = 0x1B4, //!< FSIBTX access
    SYSCTL_ACCESS_FSIBRX        = 0x1B6, //!< FSIBRX access
    SYSCTL_ACCESS_FSICRX        = 0x1BA, //!< FSICRX access
    SYSCTL_ACCESS_FSIDRX        = 0x1BE, //!< FSIDRX access
    SYSCTL_ACCESS_FSIERX        = 0x1C2, //!< FSIERX access
    SYSCTL_ACCESS_FSIFRX        = 0x1C6, //!< FSIFRX access
    SYSCTL_ACCESS_FSIGRX        = 0x1CA, //!< FSIGRX access
    SYSCTL_ACCESS_FSIHRX        = 0x1CE, //!< FSIHRX access
    SYSCTL_ACCESS_MCANA         = 0x1D0  //!< MCANA access
} SysCtl_AccessPeripheral;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setPeripheralAccessControl() and SysCtl_getPeripheralAccessControl()
//! as the \e controller parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_ACCESS_CPUX  = 0U,   //!< CPU access to the peripheral
    SYSCTL_ACCESS_CLA1  = 2U,   //!< CLA1 access to the peripheral
    SYSCTL_ACCESS_DMA1  = 4U    //!< DMA access to the peripheral
} SysCtl_AccessController;
//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setPeripheralAccessControl() as the \e permission parameter.
//
//*****************************************************************************
typedef enum
{
    //! Full Access for both read and write.
    SYSCTL_ACCESS_FULL      = 3U,
    //! Protected RD access such that FIFOs. Clear on read, registers are not
    //! changed and no write access.
    SYSCTL_ACCESS_PROTECTED = 2U,
    //! No read or write access.
    SYSCTL_ACCESS_NONE      = 0U
} SysCtl_AccessPermission;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_selectClockOutSource() as the \e source parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_CLOCKOUT_PLLSYS     = 0U,   //!< PLL System Clock post SYSCLKDIV
    SYSCTL_CLOCKOUT_PLLRAW     = 1U,   //!< PLL Raw Clock
    SYSCTL_CLOCKOUT_SYSCLK     = 2U,   //!< CPU System Clock
    SYSCTL_CLOCKOUT_SYSCLK2    = 3U,   //!< CPU 2 System Clock
    SYSCTL_CLOCKOUT_AUXPLLCLK  = 4U,   //!< Aux PLL Clock
    SYSCTL_CLOCKOUT_INTOSC1    = 5U,   //!< Internal Oscillator 1
    SYSCTL_CLOCKOUT_INTOSC2    = 6U,   //!< Internal Oscillator 2
    SYSCTL_CLOCKOUT_XTALOSC    = 7U,   //!< External Oscillator
    SYSCTL_CLOCKOUT_CMCLK      = 8U,   //!< CMCLK
    SYSCTL_CLOCKOUT_PUMPOSC    = 9U,   //!< PUMPOSC
    SYSCTL_SYSPLLCLK_AUX       = 10U,  //!< Test Clk of the System APLL
    SYSCTL_AUXPLLCLK_AUX       = 11U,  //!< Test Clk of the Auxillary APLL
    SYSCTL_SYSPLLCLKOUT        = 12U,  //!< PLL System Clock pre SYSCLKDIV
    SYSCTL_AUXPLLCLKOUT        = 13U   //!< PLL System Clock pre AUXCLKDIV
} SysCtl_ClockOut;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setExternalOscMode() as the \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_XTALMODE_CRYSTAL = 1U,   //!< XTAL Oscillator Crystal Mode
    SYSCTL_XTALMODE_SINGLE  = 2U    //!< XTAL Oscillator Single-Ended Mode
} SysCtl_ExternalOscMode;


//*****************************************************************************
//
//! The following values define the \e syncSrc parameter for
//! SysCtl_setSyncOutputConfig().
//
//*****************************************************************************
typedef enum
{
    SYSCTL_SYNC_OUT_SRC_EPWM1SYNCOUT  = 0X0U,  //!< EPWM1SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM2SYNCOUT  = 0X1U,  //!< EPWM2SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM3SYNCOUT  = 0X2U,  //!< EPWM3SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM4SYNCOUT  = 0X3U,  //!< EPWM4SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM5SYNCOUT  = 0X4U,  //!< EPWM5SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM6SYNCOUT  = 0X5U,  //!< EPWM6SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM7SYNCOUT  = 0X6U,  //!< EPWM7SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM8SYNCOUT  = 0X7U,  //!< EPWM8SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM9SYNCOUT  = 0X8U,  //!< EPWM9SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM10SYNCOUT = 0X9U,  //!< EPWM10SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM11SYNCOUT = 0XAU,  //!< EPWM11SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM12SYNCOUT = 0XBU,  //!< EPWM12SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM13SYNCOUT = 0XCU,  //!< EPWM13SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM14SYNCOUT = 0XDU,  //!< EPWM14SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM15SYNCOUT = 0XEU,  //!< EPWM15SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM16SYNCOUT = 0XFU,  //!< EPWM16SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_ECAP1SYNCOUT  = 0x18,  //!< ECAP1SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_ECAP2SYNCOUT  = 0x19,  //!< ECAP2SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_ECAP3SYNCOUT  = 0x20,  //!< ECAP3SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_ECAP4SYNCOUT  = 0x21,  //!< ECAP4SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_ECAP5SYNCOUT  = 0x22,  //!< ECAP5SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_ECAP6SYNCOUT  = 0x23,  //!< ECAP6SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_ECAP7SYNCOUT  = 0x24   //!< ECAP7SYNCOUT --> EXTSYNCOUT

} SysCtl_SyncOutputSource;

//*****************************************************************************
//
//! The following values define the \e parametric parameter for
//! SysCtl_getDeviceParametric().
//
//*****************************************************************************
typedef enum
{
    SYSCTL_DEVICE_QUAL,       //!< Device Qualification Status
    SYSCTL_DEVICE_PINCOUNT,   //!< Device Pin Count
    SYSCTL_DEVICE_INSTASPIN,  //!< Device InstaSPIN Feature Set
    SYSCTL_DEVICE_FLASH,      //!< Device Flash size (KB)
    SYSCTL_DEVICE_PARTID,     //!< Device Part ID Format Revision
    SYSCTL_DEVICE_FAMILY,     //!< Device Family
    SYSCTL_DEVICE_PARTNO,     //!< Device Part Number
    SYSCTL_DEVICE_CLASSID     //!< Device Class ID
} SysCtl_DeviceParametric;

//*****************************************************************************
//
//! The following are values that can be passed to SysCtl_controlCMReset() &
//! SysCtl_controlCPU2Reset() as the \e control parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_CORE_DEACTIVE,   //!< Core reset is deactivated
    SYSCTL_CORE_ACTIVE      //!< Core is held in reset

}SysCtl_CoreReset;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_readADCWrapper as the \e peripheral parameter.
//
//*****************************************************************************
typedef enum
{
    //
    // ADC
    //
    SYSCTL_SELECT_ADCA,    //!< ADCA access
    SYSCTL_SELECT_ADCB,    //!< ADCB access
    SYSCTL_SELECT_ADCC,    //!< ADCC access
    SYSCTL_SELECT_ADCD     //!< ADCD access

}SysCtl_SelADC;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_allocateSharedPeripheral() as the \e peripheral parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_PALLOCATE_USBA,     //!< Allocate USB_A to CM
    SYSCTL_PALLOCATE_ETHERCAT, //!< Allocate ETHERCAT to CM
    SYSCTL_PALLOCATE_CAN_A,    //!< Allocate CAN_A to CM
    SYSCTL_PALLOCATE_CAN_B,    //!< Allocate CAN_B to CM
    SYSCTL_PALLOCATE_MCAN_A,   //!< Allocate MCAN_A to CM

}SysCtl_SharedPeripheral;

//*****************************************************************************
//
//! The following are values that can be passed to SysCtl_configureType()
//! as the \e peripheral parameter.
//
//*****************************************************************************
typedef enum
{
    //! Configure USB Type :
    //!  - Type 0 : Disable Global interrupt feature
    //!  - TYpe 1 : Enable Global interrupt feature
    SYSCTL_USBTYPE = 0x0,

    //! Configure ECAP Type :
    //!  - Type 0 : No EALLOW protection for ECAP registers
    //!  - Type 1 : ECAP registers are EALLOW protected
    SYSCTL_ECAPTYPE = 0x1,

    //! Configure SDFM Type :
    //!  - Type 0 : Data Ready conditions combined with the fault conditions
    //!             on the interrupt line.
    //!             Data ready interrupts from individual filters are not
    //!             generated.
    //!  - Type 1 : Data Ready conditions do not generate the SDFMINT.
    //!             Each filter generates a separate data ready interrupts.
    SYSCTL_SDFMTYPE = 0x2,

    //! Configure MEMMAP Type :
    //!  - Type 0 : Disables re-mapping SDRAM in lower 64kb of address space
    //!  - Type 1 : Enables re-mapping SDRAM in lower 64kb of address space.
    SYSCTL_MEMMAPTYPE = 0x4

}SysCtl_SelType;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setXClk() as \e divider parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_XCLKOUT_DIV_1  = 0,      //!<  XCLKOUT =  XCLKOUT / 1
    SYSCTL_XCLKOUT_DIV_2  = 1,      //!<  XCLKOUT =  XCLKOUT / 2
    SYSCTL_XCLKOUT_DIV_4  = 2,      //!<  XCLKOUT =  XCLKOUT / 4
    SYSCTL_XCLKOUT_DIV_8  = 3       //!<  XCLKOUT =  XCLKOUT / 8

}SysCtl_XClkDivider;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setCMClk() as \e divider parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_CMCLKOUT_DIV_1,      //!<  CM clock =  CM clock / 1
    SYSCTL_CMCLKOUT_DIV_2,      //!<  CM clock =  CM clock / 2
    SYSCTL_CMCLKOUT_DIV_3,      //!<  CM clock =  CM clock / 3
    SYSCTL_CMCLKOUT_DIV_4,      //!<  CM clock =  CM clock / 4
    SYSCTL_CMCLKOUT_DIV_5,      //!<  CM clock =  CM clock / 5
    SYSCTL_CMCLKOUT_DIV_6,      //!<  CM clock =  CM clock / 6
    SYSCTL_CMCLKOUT_DIV_7,      //!<  CM clock =  CM clock / 7
    SYSCTL_CMCLKOUT_DIV_8       //!<  CM clock =  CM clock / 8

}SysCtl_CMClkDivider;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setEnetClk() as \e divider parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_ENETCLKOUT_DIV_1,      //!<  Enet clock =  Enet clock / 1
    SYSCTL_ENETCLKOUT_DIV_2,      //!<  Enet clock =  Enet clock / 2
    SYSCTL_ENETCLKOUT_DIV_3,      //!<  Enet clock =  Enet clock / 3
    SYSCTL_ENETCLKOUT_DIV_4,      //!<  Enet clock =  Enet clock / 4
    SYSCTL_ENETCLKOUT_DIV_5,      //!<  Enet clock =  Enet clock / 5
    SYSCTL_ENETCLKOUT_DIV_6,      //!<  Enet clock =  Enet clock / 6
    SYSCTL_ENETCLKOUT_DIV_7,      //!<  Enet clock =  Enet clock / 7
    SYSCTL_ENETCLKOUT_DIV_8       //!<  Enet clock =  Enet clock / 8

}SysCtl_EnetClkDivider;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setECatClk() as \e divider parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_ECATCLKOUT_DIV_1,      //!<  ECat clock =  ECat clock / 1
    SYSCTL_ECATCLKOUT_DIV_2,      //!<  ECat clock =  ECat clock / 2
    SYSCTL_ECATCLKOUT_DIV_3,      //!<  ECat clock =  ECat clock / 3
    SYSCTL_ECATCLKOUT_DIV_4,      //!<  ECat clock =  ECat clock / 4
    SYSCTL_ECATCLKOUT_DIV_5,      //!<  ECat clock =  ECat clock / 5
    SYSCTL_ECATCLKOUT_DIV_6,      //!<  ECat clock =  ECat clock / 6
    SYSCTL_ECATCLKOUT_DIV_7,      //!<  ECat clock =  ECat clock / 7
    SYSCTL_ECATCLKOUT_DIV_8       //!<  ECat clock =  ECat clock / 8

}SysCtl_ECatClkDivider;

//*****************************************************************************
//
//! The following are values that can be passed to SysCtl_setCMClk(),
//! SysCtl_setECatClk() & SysCtl_isPLLValid() as \e source parameter.
//
//*****************************************************************************
typedef enum
{
    //! Auxillary PLL
    SYSCTL_SOURCE_AUXPLL,
    //! System PLL
    SYSCTL_SOURCE_SYSPLL

}SysCtl_PLLClockSource;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setMCANClk() as \e divider parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_MCANCLK_DIV_1  = 0x0,   //!<  MCAN clock =  MCAN clock / 1
    SYSCTL_MCANCLK_DIV_2  = 0x1,   //!<  MCAN clock =  MCAN clock / 2
    SYSCTL_MCANCLK_DIV_3  = 0x2,   //!<  MCAN clock =  MCAN clock / 3
    SYSCTL_MCANCLK_DIV_4  = 0x3,   //!<  MCAN clock =  MCAN clock / 4
    SYSCTL_MCANCLK_DIV_5  = 0x4,   //!<  MCAN clock =  MCAN clock / 5
    SYSCTL_MCANCLK_DIV_6  = 0x5,   //!<  MCAN clock =  MCAN clock / 6
    SYSCTL_MCANCLK_DIV_7  = 0x6,   //!<  MCAN clock =  MCAN clock / 7
    SYSCTL_MCANCLK_DIV_8  = 0x7,   //!<  MCAN clock =  MCAN clock / 8
    SYSCTL_MCANCLK_DIV_9  = 0x8,   //!<  MCAN clock =  MCAN clock / 9
    SYSCTL_MCANCLK_DIV_10 = 0x9,   //!<  MCAN clock =  MCAN clock / 10
    SYSCTL_MCANCLK_DIV_11 = 0xA,   //!<  MCAN clock =  MCAN clock / 11
    SYSCTL_MCANCLK_DIV_12 = 0xB,   //!<  MCAN clock =  MCAN clock / 12
    SYSCTL_MCANCLK_DIV_13 = 0xC,   //!<  MCAN clock =  MCAN clock / 13
    SYSCTL_MCANCLK_DIV_14 = 0xD,   //!<  MCAN clock =  MCAN clock / 14
    SYSCTL_MCANCLK_DIV_15 = 0xE,   //!<  MCAN clock =  MCAN clock / 15
    SYSCTL_MCANCLK_DIV_16 = 0xF,   //!<  MCAN clock =  MCAN clock / 16
    SYSCTL_MCANCLK_DIV_17 = 0x10,  //!<  MCAN clock =  MCAN clock / 17
    SYSCTL_MCANCLK_DIV_18 = 0x11,  //!<  MCAN clock =  MCAN clock / 18
    SYSCTL_MCANCLK_DIV_19 = 0x12,  //!<  MCAN clock =  MCAN clock / 19
    SYSCTL_MCANCLK_DIV_20 = 0x13   //!<  MCAN clock =  MCAN clock / 20

}SysCtl_MCANClkDivider;


//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setAuxPLLClk() as \e divider parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_AUXPLLCLK_DIV_1,  //!<  AUXPLL clock =  AUXPLL clock / 1
    SYSCTL_AUXPLLCLK_DIV_2,  //!<  AUXPLL clock =  AUXPLL clock / 2
    SYSCTL_AUXPLLCLK_DIV_4,  //!<  AUXPLL clock =  AUXPLL clock / 4
    SYSCTL_AUXPLLCLK_DIV_8,  //!<  AUXPLL clock =  AUXPLL clock / 8
    SYSCTL_AUXPLLCLK_DIV_3,  //!<  AUXPLL clock =  AUXPLL clock / 3
    SYSCTL_AUXPLLCLK_DIV_5,  //!<  AUXPLL clock =  AUXPLL clock / 5
    SYSCTL_AUXPLLCLK_DIV_6,  //!<  AUXPLL clock =  AUXPLL clock / 6
    SYSCTL_AUXPLLCLK_DIV_7   //!<  AUXPLL clock =  AUXPLL clock / 7

}SysCtl_AuxPLLClkDivider;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setCputimer2Clk() as \e divider parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_TMR2CLKPRESCALE_1,   //!<  Cputimer2 clock =  Cputimer2 clock / 1
    SYSCTL_TMR2CLKPRESCALE_2,   //!<  Cputimer2 clock =  Cputimer2 clock / 2
    SYSCTL_TMR2CLKPRESCALE_4,   //!<  Cputimer2 clock =  Cputimer2 clock / 4
    SYSCTL_TMR2CLKPRESCALE_8,   //!<  Cputimer2 clock =  Cputimer2 clock / 8
    SYSCTL_TMR2CLKPRESCALE_16   //!<  Cputimer2 clock =  Cputimer2 clock / 16

}SysCtl_Cputimer2ClkDivider;

//*****************************************************************************
//
//! The following are values that can be passed to SysCtl_setCputimer2Clk()
//! as \e source parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_TMR2CLKSRCSEL_SYSCLK     = 0U,   //!< System Clock
    SYSCTL_TMR2CLKSRCSEL_INTOSC1    = 1U,   //!< Internal Oscillator 1
    SYSCTL_TMR2CLKSRCSEL_INTOSC2    = 2U,   //!< Internal Oscillator 2
    SYSCTL_TMR2CLKSRCSEL_XTAL       = 3U,   //!< Crystal oscillator
    SYSCTL_TMR2CLKSRCSEL_AUXPLLCLK  = 6U    //!< Aux PLL CLock

}SysCtl_Cputimer2ClkSource;

//*****************************************************************************
//
//! The following are values that can be passed to SysCtl_lockClkConfig()
//! as the \e peripheral parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_REG_SEL_CLKSRCCTL1  = 0x0000,  //!< CLKSRCCTL1 lock
    SYSCTL_REG_SEL_CLKSRCCTL2  = 0x0100,  //!< CLKSRCCTL2 lock
    SYSCTL_REG_SEL_CLKSRCCTL3  = 0x0200,  //!< CLKSRCCTL3 lock
    SYSCTL_REG_SEL_SYSPLLCTL1  = 0x0300,  //!< SYSPLLCTL1 lock
    SYSCTL_REG_SEL_SYSPLLCTL2  = 0x0400,  //!< SYSPLLCTL2 lock
    SYSCTL_REG_SEL_SYSPLLCTL3  = 0x0500,  //!< SYSPLLCTL3 lock
    SYSCTL_REG_SEL_SYSPLLMULT  = 0x0600,  //!< SYSPLLMULT lock
    SYSCTL_REG_SEL_AUXPLLCTL1  = 0x0700,  //!< AUXPLLCTL1 lock
    SYSCTL_REG_SEL_AUXPLLMULT  = 0x0A00,  //!< AUXPLLMULT lock
    SYSCTL_REG_SEL_SYSCLKDIVSEL  = 0x0B00,  //!< SYSCLKDIVSEL lock
    SYSCTL_REG_SEL_AUXCLKDIVSEL  = 0x0C00,  //!< AUXCLKDIVSEL lock
    SYSCTL_REG_SEL_PERCLKDIVSEL  = 0x0D00,  //!< PERCLKDIVSEL lock
    SYSCTL_REG_SEL_CLBCLKCTL  = 0x0E00,  //!< CLBCLKCTL lock
    SYSCTL_REG_SEL_LOSPCP  = 0x0F00,  //!< LOSPCP lock
    SYSCTL_REG_SEL_XTALCR  = 0x1000,  //!< XTALCR lock
    SYSCTL_REG_SEL_ETHERCATCLKCTL  = 0x1100,  //!< ETHERCATCLKCTL lock
    SYSCTL_REG_SEL_CMCLKCTL  = 0x1200   //!< CMCLKCTL lock
} SysCtl_ClkRegSel;

//*****************************************************************************
//
//! The following are values that can be passed to SysCtl_lockSysConfig()
//! as the \e peripheral parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_REG_SEL_PIEVERRADDR  = 0x0200,  //!< PIEVERRADDR lock
    SYSCTL_REG_SEL_PCLKCR0  = 0x0300,  //!< PCLKCR0 lock
    SYSCTL_REG_SEL_PCLKCR1  = 0x0400,  //!< PCLKCR1 lock
    SYSCTL_REG_SEL_PCLKCR2  = 0x0500,  //!< PCLKCR2 lock
    SYSCTL_REG_SEL_PCLKCR3  = 0x0600,  //!< PCLKCR3 lock
    SYSCTL_REG_SEL_PCLKCR4  = 0x0700,  //!< PCLKCR4 lock
    SYSCTL_REG_SEL_PCLKCR6  = 0x0900,  //!< PCLKCR6 lock
    SYSCTL_REG_SEL_PCLKCR7  = 0x0A00,  //!< PCLKCR7 lock
    SYSCTL_REG_SEL_PCLKCR8  = 0x0B00,  //!< PCLKCR8 lock
    SYSCTL_REG_SEL_PCLKCR9  = 0x0C00,  //!< PCLKCR9 lock
    SYSCTL_REG_SEL_PCLKCR10  = 0x0D00,  //!< PCLKCR10 lock
    SYSCTL_REG_SEL_PCLKCR11  = 0x0E00,  //!< PCLKCR11 lock
    SYSCTL_REG_SEL_PCLKCR13  = 0x1000,  //!< PCLKCR13 lock
    SYSCTL_REG_SEL_PCLKCR14  = 0x1100,  //!< PCLKCR14 lock
    SYSCTL_REG_SEL_PCLKCR16  = 0x1300,  //!< PCLKCR16 lock
    SYSCTL_REG_SEL_LPMCR  = 0x1500,  //!< LPMCR lock
    SYSCTL_REG_SEL_GPIOLPMSEL0  = 0x1600,  //!< GPIOLPMSEL0 lock
    SYSCTL_REG_SEL_GPIOLPMSEL1  = 0x1700,  //!< GPIOLPMSEL1 lock
    SYSCTL_REG_SEL_PCLKCR17  = 0x1800,  //!< PCLKCR17 lock
    SYSCTL_REG_SEL_PCLKCR18  = 0x1900,  //!< PCLKCR18 lock
    SYSCTL_REG_SEL_PCLKCR20  = 0x1B00,  //!< PCLKCR20 lock
    SYSCTL_REG_SEL_PCLKCR21  = 0x1C00,  //!< PCLKCR21 lock
    SYSCTL_REG_SEL_PCLKCR22  = 0x1D00,  //!< PCLKCR22 lock
    SYSCTL_REG_SEL_PCLKCR23  = 0x1E00,  //!< PCLKCR23 lock
    SYSCTL_REG_SEL_ETHERCATCTL  = 0x0001   //!< ETHERCATCTL lock
} SysCtl_CpuRegSel;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setCLBClk() as \e cpuInst parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_CLBCLKOUT_DIV_1,      //!<  CLB clock =  CLB clock / 1
    SYSCTL_CLBCLKOUT_DIV_2,      //!<  CLB clock =  CLB clock / 2
    SYSCTL_CLBCLKOUT_DIV_3,      //!<  CLB clock =  CLB clock / 3
    SYSCTL_CLBCLKOUT_DIV_4,      //!<  CLB clock =  CLB clock / 4
    SYSCTL_CLBCLKOUT_DIV_5,      //!<  CLB clock =  CLB clock / 5
    SYSCTL_CLBCLKOUT_DIV_6,      //!<  CLB clock =  CLB clock / 6
    SYSCTL_CLBCLKOUT_DIV_7,      //!<  CLB clock =  CLB clock / 7
    SYSCTL_CLBCLKOUT_DIV_8       //!<  CLB clock =  CLB clock / 8

}SysCtl_CLBClkDivider;

typedef enum
{
    SYSCTL_CLBTCLKOUT_DIV_1,      //!<  CLBTCLKOUT =  CLB clock / 1
    SYSCTL_CLBTCLKOUT_DIV_2       //!<  CLBTCLKOUT =  CLB clock / 2

}SysCtl_CLBTClkDivider;

typedef enum
{
    SYSCTL_CLB1 = 0x10,    //!< CLB 1 instance
    SYSCTL_CLB2 = 0x11,    //!< CLB 2 instance
    SYSCTL_CLB3 = 0x12,    //!< CLB 3 instance
    SYSCTL_CLB4 = 0x13,    //!< CLB 4 instance
    SYSCTL_CLB5 = 0x14,    //!< CLB 5 instance
    SYSCTL_CLB6 = 0x15,    //!< CLB 6 instance
    SYSCTL_CLB7 = 0x16,    //!< CLB 7 instance
    SYSCTL_CLB8 = 0x17     //!< CLB 8 instance
}SysCtl_CLBInst;

typedef enum
{
    SYSCTL_CLBCLK_SYNC,       //!<  CLB is synchronous to SYSCLK
    SYSCTL_CLBCLK_ASYNC       //!<  CLB runs of asynchronous clock

}SysCtl_CLBClkm;


//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! Wrapper function for Device_cal function
//!
//! \param None
//!
//! This is a wrapper function for the Device_cal function available in the OTP
//! memory.
//! The function saves and restores the core registers which are being
//! used by the Device_cal function
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_deviceCal(void)
{
    //
    // Save the core registers used by Device_cal function in the stack
    //
    SYSCTL_DEVICECAL_CONTEXT_SAVE;

    //
    // Call the Device_cal function
    //
    Device_cal();

    //
    // Restore the core registers
    //
    SYSCTL_DEVICECAL_CONTEXT_RESTORE;
}

//*****************************************************************************
//
//! Resets a peripheral
//!
//! \param peripheral is the peripheral to reset.
//!
//! This function uses the SOFTPRESx registers to reset a specified peripheral.
//! Module registers will be returned to their reset states.
//!
//! \note This includes registers containing trim values.The peripheral
//! software reset needed by CPU2 can be communicated to CPU1 via
//! IPC for all shared peripherals.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_resetPeripheral(SysCtl_PeripheralSOFTPRES peripheral)
{
    uint16_t regIndex;
    uint16_t bitIndex;

    //
    // Decode the peripheral variable.
    //
    regIndex = (uint16_t)2U * ((uint16_t)peripheral &
                               (uint16_t)SYSCTL_PERIPH_REG_M);
    bitIndex = ((uint16_t)peripheral & SYSCTL_PERIPH_BIT_M) >>
               SYSCTL_PERIPH_BIT_S;

    EALLOW;

    //
    // Sets the appropriate reset bit and then clears it.
    //
    HWREG(DEVCFG_BASE + SYSCTL_O_SOFTPRES0 + regIndex) |=  (1UL << bitIndex);
    HWREG(DEVCFG_BASE + SYSCTL_O_SOFTPRES0 + regIndex) &= ~(1UL << bitIndex);

    //
    // Call Device_cal function
    //
    if((((uint16_t)peripheral & SYSCTL_PERIPH_REG_M) == 0xDU) ||      // ADCx
       (((uint16_t)peripheral & SYSCTL_PERIPH_REG_M) == 0x10U)        // DACx
       )
    {
        SysCtl_deviceCal();
    }

    EDIS;
}

//*****************************************************************************
//
//! Enables a peripheral.
//!
//! \param peripheral is the peripheral to enable.
//!
//! Peripherals are enabled with this function.  At power-up, all peripherals
//! are disabled; they must be enabled in order to operate or respond to
//! register reads/writes.
//!
//! \note Note that there should be atleast 5 cycles delay between enabling the
//! peripheral clock and accessing the peripheral registers. The delay should be
//! added by the user if the peripheral is accessed immediately after this
//! function call.
//! Use asm(" RPT #5 || NOP"); to add 5 cycle delay post this function call.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enablePeripheral(SysCtl_PeripheralPCLOCKCR peripheral)
{
    uint16_t regIndex;
    uint16_t bitIndex;

    //
    // Decode the peripheral variable.
    //
    regIndex = (uint16_t)2U * ((uint16_t)peripheral &
                               (uint16_t)SYSCTL_PERIPH_REG_M);
    bitIndex = ((uint16_t)peripheral & SYSCTL_PERIPH_BIT_M) >>
               SYSCTL_PERIPH_BIT_S;

    EALLOW;

    //
    // Turn on the module clock.
    //
    HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR0 + regIndex) |= (1UL << bitIndex);
    EDIS;
}

//*****************************************************************************
//
//! Disables a peripheral.
//!
//! \param peripheral is the peripheral to disable.
//!
//! Peripherals are disabled with this function.  Once disabled, they will not
//! operate or respond to register reads/writes.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disablePeripheral(SysCtl_PeripheralPCLOCKCR peripheral)
{
    uint16_t regIndex;
    uint16_t bitIndex;

    //
    // Decode the peripheral variable.
    //
    regIndex = (uint16_t)2U * ((uint16_t)peripheral &
                               (uint16_t)SYSCTL_PERIPH_REG_M);
    bitIndex = ((uint16_t)peripheral & SYSCTL_PERIPH_BIT_M) >>
               SYSCTL_PERIPH_BIT_S;

    EALLOW;

    //
    // Turn off the module clock.
    //
    HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR0 + regIndex) &= ~(1UL << bitIndex);
    EDIS;
}

//*****************************************************************************
//
//! Resets the device.
//!
//! This function performs a watchdog reset of the device.
//!
//! \return This function does not return.
//
//*****************************************************************************
static inline void
SysCtl_resetDevice(void)
{
    //
    // Write an incorrect check value to the watchdog control register
    // This will cause a device reset
    //
    EALLOW;

    //
    // Enable the watchdog
    //
    HWREGH(WD_BASE + SYSCTL_O_WDCR) = SYSCTL_WD_CHKBITS;

    //
    // Write a bad check value
    //
    HWREGH(WD_BASE + SYSCTL_O_WDCR) = 0U;

    EDIS;

    //
    // The device should have reset, so this should never be reached.  Just in
    // case, loop forever.
    //
    while((bool)1)
    {
    }
}

//*****************************************************************************
//
//! Gets the reason for a reset.
//!
//! This function will return the reason(s) for a reset.  Since the reset
//! reasons are sticky until either cleared by software or an external reset,
//! multiple reset reasons may be returned if multiple resets have occurred.
//! The reset reason will be a logical OR of
//! - \b SYSCTL_CAUSE_POR - Power-on reset
//! - \b SYSCTL_CAUSE_XRS - External reset pin
//! - \b SYSCTL_CAUSE_WDRS - Watchdog reset
//! - \b SYSCTL_CAUSE_NMIWDRS - NMI watchdog reset
//! - \b SYSCTL_CAUSE_SCCRESET - SCCRESETn reset from DCSM
//! - \b SYSCTL_CAUSE_HWBISTN - HWBISTn reset
//! - \b SYSCTL_CAUSE_ECAT_RESET_OUT - Ethercat Reset
//! - \b SYSCTL_CAUSE_SIMRESET_CPU1RSN - SIMRESET from CPU1
//! - \b SYSCTL_CAUSE_SIMRESET_XRSN - SIMRESET from XRSn
//! - \b SYSCTL_RESC_XRSN_PIN_STATUS - XRSN Pin Status
//! - \b SYSCTL_RESC_TRSTN_PIN_STATUS -TRSTN Pin Status
//!
//! \note If you re-purpose the reserved boot ROM RAM, the POR and XRS reset
//! statuses won't be accurate.
//!
//! \return Returns the reason(s) for a reset.
//
//*****************************************************************************
static inline uint32_t
SysCtl_getResetCause(void)
{
    uint32_t resetCauses;

    //
    // Read CPU reset register
    //
    resetCauses = HWREG(CPUSYS_BASE + SYSCTL_O_RESC) &
                  ((uint32_t)SYSCTL_RESC_POR | (uint32_t)SYSCTL_RESC_XRSN |
                   (uint32_t)SYSCTL_RESC_WDRSN |
                   (uint32_t)SYSCTL_RESC_NMIWDRSN |
                   (uint32_t)SYSCTL_RESC_SCCRESETN
                    | (uint32_t)SYSCTL_RESC_HWBISTN |
                   (uint32_t)SYSCTL_RESC_ECAT_RESET_OUT |
                   (uint32_t)SYSCTL_RESC_SIMRESET_CPU1RSN |
                   (uint32_t)SYSCTL_RESC_SIMRESET_XRSN |
                   (uint32_t)SYSCTL_RESC_XRSN_PIN_STATUS |
                   (uint32_t)SYSCTL_RESC_TRSTN_PIN_STATUS
                   );

    //
    // Set POR and XRS Causes from boot ROM Status
    //
    if((HWREG(SYSCTL_BOOT_ROM_STATUS) & (uint32_t)SYSCTL_BOOT_ROM_POR) ==
       (uint32_t)SYSCTL_BOOT_ROM_POR)
    {
        resetCauses |= SYSCTL_RESC_POR;
    }
    if((HWREG(SYSCTL_BOOT_ROM_STATUS) & (uint32_t)SYSCTL_BOOT_ROM_XRS) ==
       (uint32_t)SYSCTL_BOOT_ROM_XRS)
    {
        resetCauses |= SYSCTL_RESC_XRSN;
    }

    //
    // Return the reset reasons.
    //
    return(resetCauses);
}

//*****************************************************************************
//
//! Clears reset reasons.
//!
//! \param rstCauses are the reset causes to be cleared; must be a logical
//! OR of
//! - \b SYSCTL_CAUSE_POR - Power-on reset
//! - \b SYSCTL_CAUSE_XRS - External reset pin
//! - \b SYSCTL_CAUSE_WDRS - Watchdog reset
//! - \b SYSCTL_CAUSE_NMIWDRS - NMI watchdog reset
//! - \b SYSCTL_CAUSE_SCCRESET - SCCRESETn reset from DCSM
//! - \b SYSCTL_CAUSE_HWBISTN - HWBISTn reset
//! - \b SYSCTL_CAUSE_ECAT_RESET_OUT - Ethercat Reset
//! - \b SYSCTL_CAUSE_SIMRESET_CPU1RSN - SIMRESET from CPU1
//! - \b SYSCTL_CAUSE_SIMRESET_XRSN - SIMRESET from XRSn
//!
//! This function clears the specified sticky reset reasons.  Once cleared,
//! another reset for the same reason can be detected, and a reset for a
//! different reason can be distinguished (instead of having two reset causes
//! set).  If the reset reason is used by an application, all reset causes
//! should be cleared after they are retrieved with SysCtl_getResetCause().
//!
//! \note Some reset causes are cleared by the boot ROM.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearResetCause(uint32_t rstCauses)
{
    //
    // Clear the given reset reasons.
    //
    HWREG(CPUSYS_BASE + SYSCTL_O_RESC) = rstCauses;
}

//*****************************************************************************
//
//! Sets the low speed peripheral clock rate prescaler.
//!
//! \param prescaler is the LSPCLK rate relative to SYSCLK
//!
//! This function configures the clock rate of the low speed peripherals. The
//! \e prescaler parameter is the value by which the SYSCLK rate is divided to
//! get the LSPCLK rate. For example, a \e prescaler of
//! \b SYSCTL_LSPCLK_PRESCALE_4 will result in a LSPCLK rate that is a quarter
//! of the SYSCLK rate.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setLowSpeedClock(SysCtl_LSPCLKPrescaler prescaler)
{
    //
    // Write the divider selection to the appropriate register.
    //
    EALLOW;
    HWREG(CLKCFG_BASE + SYSCTL_O_LOSPCP) =
        (HWREG(CLKCFG_BASE + SYSCTL_O_LOSPCP) &
         ~(uint32_t)SYSCTL_LOSPCP_LSPCLKDIV_M) | (uint32_t)prescaler;
    EDIS;
}

//*****************************************************************************
//
//! Sets the ePWM clock divider.
//!
//! \param divider is the value by which PLLSYSCLK is divided
//!
//! This function configures the clock rate of the EPWMCLK. The
//! \e divider parameter is the value by which the SYSCLK rate is divided to
//! get the EPWMCLK rate. For example, \b SYSCTL_EPWMCLK_DIV_2 will select an
//! EPWMCLK rate that is half the PLLSYSCLK rate.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setEPWMClockDivider(SysCtl_EPWMCLKDivider divider)
{
    //
    // Write the divider selection to the appropriate register.
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_PERCLKDIVSEL) =
        (HWREGH(CLKCFG_BASE + SYSCTL_O_PERCLKDIVSEL) &
         ~SYSCTL_PERCLKDIVSEL_EPWMCLKDIV_M) | (uint16_t)divider;
    EDIS;
}

//*****************************************************************************
//
//! Sets the EMIF1 clock divider.
//!
//! \param divider is the value by which PLLSYSCLK (or CPU1.SYSCLK on a dual
//! core device) is divided
//!
//! This function configures the clock rate of the EMIF1CLK. The
//! \e divider parameter is the value by which the SYSCLK rate is divided to
//! get the EMIF1CLK rate. For example, \b SYSCTL_EMIF1CLK_DIV_2 will select an
//! EMIF1CLK rate that is half the PLLSYSCLK (or CPU1.SYSCLK on a dual
//! core device) rate.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setEMIF1ClockDivider(SysCtl_EMIF1CLKDivider divider)
{
    //
    // Write the divider selection to the appropriate register.
    //
    EALLOW;
    if(divider == SYSCTL_EMIF1CLK_DIV_2)
    {
        HWREGH(CLKCFG_BASE + SYSCTL_O_PERCLKDIVSEL) |=
            SYSCTL_PERCLKDIVSEL_EMIF1CLKDIV;
    }
    else
    {
        HWREGH(CLKCFG_BASE + SYSCTL_O_PERCLKDIVSEL) &=
            ~SYSCTL_PERCLKDIVSEL_EMIF1CLKDIV;
    }
    EDIS;
}

//*****************************************************************************
//
//! Sets the EMIF2 clock divider.
//!
//! \param divider is the value by which PLLSYSCLK (or CPU1.SYSCLK on a dual
//! core device) is divided
//!
//! This function configures the clock rate of the EMIF2CLK. The
//! \e divider parameter is the value by which the SYSCLK rate is divided to
//! get the EMIF2CLK rate. For example, \b SYSCTL_EMIF2CLK_DIV_2 will select an
//! EMIF2CLK rate that is half the PLLSYSCLK (or CPU1.SYSCLK on a dual
//! core device) rate.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setEMIF2ClockDivider(SysCtl_EMIF2CLKDivider divider)
{
    //
    // Write the divider selection to the appropriate register.
    //
    EALLOW;
    if(divider == SYSCTL_EMIF2CLK_DIV_2)
    {
        HWREGH(CLKCFG_BASE + SYSCTL_O_PERCLKDIVSEL) |=
            SYSCTL_PERCLKDIVSEL_EMIF2CLKDIV;
    }
    else
    {
        HWREGH(CLKCFG_BASE + SYSCTL_O_PERCLKDIVSEL) &=
            ~SYSCTL_PERCLKDIVSEL_EMIF2CLKDIV;
    }
    EDIS;
}

//*****************************************************************************
//
//! Selects a clock source to mux to an external GPIO pin (XCLKOUT).
//!
//! \param source is the internal clock source to be configured.
//!
//! This function configures the specified clock source to be muxed to an
//! external clock out (XCLKOUT) GPIO pin. The \e source parameter may take a
//! value of one of the following values:
//! - \b SYSCTL_CLOCKOUT_PLLSYS
//! - \b SYSCTL_CLOCKOUT_PLLRAW
//! - \b SYSCTL_CLOCKOUT_SYSCLK
//! - \b SYSCTL_CLOCKOUT_SYSCLK2
//! - \b SYSCTL_CLOCKOUT_AUXPLLCLK
//! - \b SYSCTL_CLOCKOUT_INTOSC1
//! - \b SYSCTL_CLOCKOUT_INTOSC2
//! - \b SYSCTL_CLOCKOUT_XTALOSC
//! - \b SYSCTL_CLOCKOUT_CMCLK
//! - \b SYSCTL_CLOCKOUT_PUMPOSC
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_selectClockOutSource(SysCtl_ClockOut source)
{
    EALLOW;

    //
    // Clear clock out source
    //
    HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL3) &=
        ~SYSCTL_CLKSRCCTL3_XCLKOUTSEL_M;

    //
    // Set clock out source
    //
    HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL3) |= (uint16_t)source;

    EDIS;
}

//*****************************************************************************
//
//! Set the external oscillator mode.
//!
//! \param mode is the external oscillator mode to be configured.
//!
//! This function sets the external oscillator mode specified by the \e mode
//! parameter which may take one of two values:
//! - \b SYSCTL_XTALMODE_CRYSTAL - Crystal Mode
//! - \b SYSCTL_XTALMODE_SINGLE  - Single-Ended Mode
//!
//! \note The external oscillator must be powered off before this configuration
//! can be performed.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setExternalOscMode(SysCtl_ExternalOscMode mode)
{
    EALLOW;

    switch(mode)
    {
        case SYSCTL_XTALMODE_CRYSTAL:
            //
            // Set mode to Crystal
            //
            HWREG(CLKCFG_BASE + SYSCTL_O_XTALCR) &= ~(uint32_t)SYSCTL_XTALCR_SE;
            break;

        case SYSCTL_XTALMODE_SINGLE:
            //
            // Set mode to Single-Ended
            //
            HWREG(CLKCFG_BASE + SYSCTL_O_XTALCR) |= SYSCTL_XTALCR_SE;
            break;

        default:
            //
            // Do nothing. Not a valid mode value.
            //
            break;
    }

    EDIS;
}

//*****************************************************************************
//
//! Gets the external oscillator counter value.
//!
//! This function returns the X1 clock counter value. When the return value
//! reaches 0x3FF, it freezes. Before switching from INTOSC2 to an external
//! oscillator (XTAL), an application should call this function to make sure
//! the counter is saturated.
//!
//! \return Returns the value of the 10-bit X1 clock counter.
//
//*****************************************************************************
static inline uint16_t
SysCtl_getExternalOscCounterValue(void)
{
    return(HWREGH(CLKCFG_BASE + SYSCTL_O_X1CNT) & SYSCTL_X1CNT_X1CNT_M);
}

//*****************************************************************************
//
//! Clears the external oscillator counter value.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearExternalOscCounterValue(void)
{
    HWREG(CLKCFG_BASE + SYSCTL_O_X1CNT) |= SYSCTL_X1CNT_CLR;
    HWREG(CLKCFG_BASE + SYSCTL_O_X1CNT) &= ~SYSCTL_X1CNT_CLR;
}

//*****************************************************************************
//
//! Turns on the specified oscillator sources.
//!
//! \param oscSource is the oscillator source to be configured.
//!
//! This function turns on the oscillator specified by the \e oscSource
//! parameter which may take a value of  \b SYSCTL_OSCSRC_XTAL
//!
//! \note \b SYSCTL_OSCSRC_OSC1 is not a valid value for \e oscSource.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_turnOnOsc(uint32_t oscSource)
{
    ASSERT(
           (oscSource == SYSCTL_OSCSRC_XTAL)
          );

    EALLOW;

    switch(oscSource)
    {
        case SYSCTL_OSCSRC_XTAL:
            //
            // Turn on XTALOSC
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &=
                ~SYSCTL_CLKSRCCTL1_XTALOFF;
            HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) &= ~SYSCTL_XTALCR_OSCOFF;

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
//! Turns off the specified oscillator sources.
//!
//! \param oscSource is the oscillator source to be configured.
//!
//! This function turns off the oscillator specified by the \e oscSource
//! parameter which may take a value of  \b SYSCTL_OSCSRC_XTAL
//!
//! \note \b SYSCTL_OSCSRC_OSC1 is not a valid value for \e oscSource.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_turnOffOsc(uint32_t oscSource)
{
    ASSERT(
           (oscSource == SYSCTL_OSCSRC_XTAL)
          );

    EALLOW;

    switch(oscSource)
    {
        case SYSCTL_OSCSRC_XTAL:
            //
            // Turn off XTALOSC
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) |=
                SYSCTL_CLKSRCCTL1_XTALOFF;
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
//! Enters IDLE mode.
//!
//! This function puts the device into IDLE mode. The CPU clock is gated while
//! all peripheral clocks are left running. Any enabled interrupt will wake the
//! CPU up from IDLE mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enterIdleMode(void)
{
    EALLOW;

    //
    // Configure the device to go into IDLE mode when IDLE is executed.
    //
    HWREG(CPUSYS_BASE + SYSCTL_O_LPMCR) =
                (HWREG(CPUSYS_BASE + SYSCTL_O_LPMCR) &
                 ~(uint32_t)SYSCTL_LPMCR_LPM_M) | SYSCTL_LPM_IDLE;

    EDIS;

#ifndef _DUAL_HEADERS
    IDLE;
#else
    IDLE_ASM;
#endif
}

//*****************************************************************************
//
//! Enters STANDBY mode.
//!
//! This function puts the device into STANDBY mode. This will gate both the
//! CPU clock and any peripheral clocks derived from SYSCLK. The watchdog is
//! left active, and an NMI or an optional watchdog interrupt will wake the
//! CPU subsystem from STANDBY mode.
//!
//! GPIOs may be configured to wake the CPU subsystem. See
//! SysCtl_enableLPMWakeupPin().
//!
//! The CPU will receive an interrupt (WAKEINT) on wakeup.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enterStandbyMode(void)
{
    EALLOW;

    //
    // Configure the device to go into STANDBY mode when IDLE is executed.
    //
    HWREG(CPUSYS_BASE + SYSCTL_O_LPMCR) =
                (HWREG(CPUSYS_BASE + SYSCTL_O_LPMCR) &
                 ~(uint32_t)SYSCTL_LPMCR_LPM_M) | SYSCTL_LPM_STANDBY;

    EDIS;

#ifndef _DUAL_HEADERS
    IDLE;
#else
    IDLE_ASM;
#endif
}

//*****************************************************************************
//! Enables a pin to wake up the device from the following mode(s):
//!  - STANDBY
//!
//! \param pin is the identifying number of the pin.
//!
//! This function connects a pin to the LPM circuit, allowing an event on the
//! pin to wake up the device when when it is in following mode(s):
//!  - STANDBY
//!
//! The pin is specified by its numerical value. For example, GPIO34 is
//! specified by passing 34 as \e pin. Only GPIOs 0 through 63 are capable of
//! being connected to the LPM circuit.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableLPMWakeupPin(uint32_t pin)
{
    uint32_t pinMask;

    //
    // Check the arguments.
    //
    ASSERT(pin <= 63U);

    pinMask = 1UL << (pin % 32U);

    EALLOW;

    if(pin < 32U)
    {
        HWREG(CPUSYS_BASE + SYSCTL_O_GPIOLPMSEL0) |= pinMask;
    }
    else
    {
        HWREG(CPUSYS_BASE + SYSCTL_O_GPIOLPMSEL1) |= pinMask;
    }

    EDIS;
}

//*****************************************************************************
//! Disables a pin to wake up the device from the following mode(s):
//!  - STANDBY
//!
//! \param pin is the identifying number of the pin.
//!
//! This function disconnects a pin to the LPM circuit, disallowing an event on
//! the pin to wake up the device when when it is in following mode(s):
//!  - STANDBY
//!
//! The pin is specified by its numerical value. For example, GPIO34 is
//! specified by passing 34 as \e pin. Only GPIOs 0 through 63 are valid.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disableLPMWakeupPin(uint32_t pin)
{
    uint32_t pinMask;

    //
    // Check the arguments.
    //
    ASSERT(pin <= 63U);

    pinMask = 1UL << (pin % 32U);

    EALLOW;

    if(pin < 32U)
    {
        HWREG(CPUSYS_BASE + SYSCTL_O_GPIOLPMSEL0) &= ~pinMask;
    }
    else
    {
        HWREG(CPUSYS_BASE + SYSCTL_O_GPIOLPMSEL1) &= ~pinMask;
    }

    EDIS;
}

//*****************************************************************************
//
//! Sets the number of cycles to qualify an input on waking from STANDBY mode.
//!
//! \param cycles is the number of OSCCLK cycles.
//!
//! This function sets the number of OSCCLK clock cycles used to qualify the
//! selected inputs when waking from STANDBY mode. The \e cycles parameter
//! should be passed a cycle count between 2 and 65 cycles inclusive.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setStandbyQualificationPeriod(uint16_t cycles)
{
    //
    // Check the arguments.
    //
    ASSERT((cycles >= 2U) && (cycles <= 65U));

    EALLOW;

    HWREGH(CPUSYS_BASE + SYSCTL_O_LPMCR) =
                (HWREGH(CPUSYS_BASE + SYSCTL_O_LPMCR) &
                 ~(uint16_t)SYSCTL_LPMCR_QUALSTDBY_M) |
                ((cycles - 2U) << SYSCTL_LPMCR_QUALSTDBY_S);

    EDIS;
}

//*****************************************************************************
//
//! Enable the device to wake from STANDBY mode upon a watchdog interrupt.
//!
//! \note In order to use this option, you must configure the watchdog to
//! generate an interrupt using SysCtl_setWatchdogMode().
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableWatchdogStandbyWakeup(void)
{
    EALLOW;

    //
    // Set the bit enables the watchdog to wake up the device from STANDBY.
    //
    HWREGH(CPUSYS_BASE + SYSCTL_O_LPMCR) |= SYSCTL_LPMCR_WDINTE;

    EDIS;
}

//*****************************************************************************
//
//! Disable the device from waking from STANDBY mode upon a watchdog interrupt.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disableWatchdogStandbyWakeup(void)
{
    EALLOW;

    //
    // Clear the bit enables the watchdog to wake up the device from STANDBY.
    //
    HWREGH(CPUSYS_BASE + SYSCTL_O_LPMCR) &= ~SYSCTL_LPMCR_WDINTE;

    EDIS;
}

//*****************************************************************************
//
//! Configures whether the watchdog generates a reset or an interrupt signal.
//!
//! \param mode is a flag to select the watchdog mode.
//!
//! This function configures the action taken when the watchdog counter reaches
//! its maximum value. When the \e mode parameter is
//! \b SYSCTL_WD_MODE_INTERRUPT, the watchdog is enabled to generate a watchdog
//! interrupt signal and disables the generation of a reset signal. This will
//! allow the watchdog module to wake up the device from IDLE
//! or STANDBY if desired (see SysCtl_enableWatchdogStandbyWakeup()).
//!
//! When the \e mode parameter is \b SYSCTL_WD_MODE_RESET, the watchdog will
//! be put into reset mode and generation of a watchdog interrupt signal will
//! be disabled. This is how the watchdog is configured by default.
//!
//! \note Check the status of the watchdog interrupt using
//! SysCtl_isWatchdogInterruptActive() before calling this function. If the
//! interrupt is still active, switching from interrupt mode to reset mode will
//! immediately reset the device.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setWatchdogMode(SysCtl_WDMode mode)
{
    EALLOW;

    //
    // Either set or clear the WDENINT bit to that will determine whether the
    // watchdog will generate a reset signal or an interrupt signal. Take care
    // not to write a 1 to WDOVERRIDE.
    //
    if(mode == SYSCTL_WD_MODE_INTERRUPT)
    {
        HWREGH(WD_BASE + SYSCTL_O_SCSR) =
                (HWREGH(WD_BASE + SYSCTL_O_SCSR) & ~SYSCTL_SCSR_WDOVERRIDE) |
                SYSCTL_SCSR_WDENINT;
    }
    else
    {
        HWREGH(WD_BASE + SYSCTL_O_SCSR) &= ~(SYSCTL_SCSR_WDENINT |
                                             SYSCTL_SCSR_WDOVERRIDE);
    }

    EDIS;
}

//*****************************************************************************
//
//! Gets the status of the watchdog interrupt signal.
//!
//! This function returns the status of the watchdog interrupt signal. If the
//! interrupt is active, this function will return \b true. If \b false, the
//! interrupt is NOT active.
//!
//! \note Make sure to call this function to ensure that the interrupt is not
//! active before making any changes to the configuration of the watchdog to
//! prevent any unexpected behavior. For instance, switching from interrupt
//! mode to reset mode while the interrupt is active will immediately reset the
//! device.
//!
//! \return \b true if the interrupt is active and \b false if it is not.
//
//*****************************************************************************
static inline bool
SysCtl_isWatchdogInterruptActive(void)
{
    //
    // If the status bit is cleared, the WDINTn signal is active.
    //
    return((HWREGH(WD_BASE + SYSCTL_O_SCSR) & SYSCTL_SCSR_WDINTS) == 0U);
}

//*****************************************************************************
//
//! Disables the watchdog.
//!
//! This function disables the watchdog timer. Note that the watchdog timer is
//! enabled on reset.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disableWatchdog(void)
{
    EALLOW;

    //
    // Set the disable bit.
    //
    HWREGH(WD_BASE + SYSCTL_O_WDCR) |= SYSCTL_WD_CHKBITS | SYSCTL_WDCR_WDDIS;

    EDIS;
}

//*****************************************************************************
//
//! Enables the watchdog.
//!
//! This function enables the watchdog timer. Note that the watchdog timer is
//! enabled on reset.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableWatchdog(void)
{
    EALLOW;

    //
    // Clear the disable bit.
    //
    HWREGH(WD_BASE + SYSCTL_O_WDCR) = (HWREGH(WD_BASE + SYSCTL_O_WDCR) &
                                       ~SYSCTL_WDCR_WDDIS) | SYSCTL_WD_CHKBITS;

    EDIS;
}

//*****************************************************************************
//
//! Services the watchdog.
//!
//! This function resets the watchdog.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_serviceWatchdog(void)
{
    EALLOW;

    //
    // Enable the counter to be reset and then reset it.
    //
    HWREGH(WD_BASE + SYSCTL_O_WDKEY) = SYSCTL_WD_ENRSTKEY;
    HWREGH(WD_BASE + SYSCTL_O_WDKEY) = SYSCTL_WD_RSTKEY;

    EDIS;
}

//*****************************************************************************
//
//! Writes the first key to enter the watchdog reset.
//!
//! This function writes the first key to enter the watchdog reset.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableWatchdogReset(void)
{
    EALLOW;

    //
    // Enable the counter to be reset
    //
    HWREGH(WD_BASE + SYSCTL_O_WDKEY) = SYSCTL_WD_ENRSTKEY;

    EDIS;
}

//*****************************************************************************
//
//! Writes the second key to reset the watchdog.
//!
//! This function writes the second key to reset the watchdog.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_resetWatchdog(void)
{
    EALLOW;

    //
    // Reset the watchdog counter
    //
    HWREGH(WD_BASE + SYSCTL_O_WDKEY) = SYSCTL_WD_RSTKEY;
    EDIS;
}

//*****************************************************************************
//
//! Sets up watchdog clock (WDCLK) pre-divider.
//!
//! \param predivider is the value that configures the pre-divider.
//!
//! This function sets up the watchdog clock (WDCLK) pre-divider. There are two
//! dividers that scale INTOSC1 to WDCLK. The \e predivider parameter divides
//! INTOSC1 down to PREDIVCLK and the prescaler (set by the
//! SysCtl_setWatchdogPrescaler() function) divides PREDIVCLK down to WDCLK.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setWatchdogPredivider(SysCtl_WDPredivider predivider)
{
    uint16_t regVal;

    regVal = (uint16_t)predivider | SYSCTL_WD_CHKBITS;

    EALLOW;

    //
    // Write the predivider to the appropriate register.
    //
    HWREGH(WD_BASE + SYSCTL_O_WDCR) = (HWREGH(WD_BASE + SYSCTL_O_WDCR) &
                                       ~(SYSCTL_WDCR_WDPRECLKDIV_M)) | regVal;

    EDIS;
}

//*****************************************************************************
//
//! Sets up watchdog clock (WDCLK) prescaler.
//!
//! \param prescaler is the value that configures the watchdog clock relative
//! to the value from the pre-divider.
//!
//! This function sets up the watchdog clock (WDCLK) prescaler. There are two
//! dividers that scale INTOSC1 to WDCLK. The predivider (set with the
//! SysCtl_setWatchdogPredivider() function) divides INTOSC1 down to PREDIVCLK
//! and the \e prescaler parameter divides PREDIVCLK down to WDCLK.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setWatchdogPrescaler(SysCtl_WDPrescaler prescaler)
{
    uint16_t regVal;

    regVal = (uint16_t)prescaler | (uint16_t)SYSCTL_WD_CHKBITS;

    EALLOW;

    //
    // Write the prescaler to the appropriate register.
    //
    HWREGH(WD_BASE + SYSCTL_O_WDCR) = (HWREGH(WD_BASE + SYSCTL_O_WDCR) &
                                       ~(SYSCTL_WDCR_WDPS_M)) | regVal;

    EDIS;
}

//*****************************************************************************
//
//! Gets the watchdog counter value.
//!
//! \return Returns the current value of the 8-bit watchdog counter. If this
//! count value overflows, a watchdog output pulse is generated.
//
//*****************************************************************************
static inline uint16_t
SysCtl_getWatchdogCounterValue(void)
{
    //
    // Read and return the value of the watchdog counter.
    //
    return(HWREGH(WD_BASE + SYSCTL_O_WDCNTR));
}

//*****************************************************************************
//
//! Gets the watchdog reset status.
//!
//! This function returns the watchdog reset status. If this function returns
//! \b true, that indicates that a watchdog reset generated the last reset
//! condition. Otherwise, it was an external device or power-up reset
//! condition.
//!
//! \return Returns \b true if the watchdog generated the last reset condition.
//
//*****************************************************************************
static inline bool
SysCtl_getWatchdogResetStatus(void)
{
    //
    // Read and return the status of the watchdog reset status flag.
    //
    return((HWREGH(CPUSYS_BASE + SYSCTL_O_RESC) & SYSCTL_RESC_WDRSN) != 0U);
}

//*****************************************************************************
//
//! Clears the watchdog reset status.
//!
//! This function clears the watchdog reset status. To check if it was set
//! first, see SysCtl_getWatchdogResetStatus().
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearWatchdogResetStatus(void)
{
    EALLOW;

    //
    // Read and return the status of the watchdog reset status flag.
    //
    HWREGH(CPUSYS_BASE + SYSCTL_O_RESCCLR) = SYSCTL_RESCCLR_WDRSN;

    EDIS;
}

//*****************************************************************************
//
//! Set the minimum threshold value for windowed watchdog
//!
//! \param value is the value to set the window threshold
//!
//! This function sets the minimum threshold value used to define the lower
//! limit of the windowed watchdog functionality.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setWatchdogWindowValue(uint16_t value)
{
    EALLOW;

    //
    // Clear the windowed value
    //
    HWREGH(WD_BASE + SYSCTL_O_WDWCR) &= ~SYSCTL_WDWCR_MIN_M;

    //
    // Set the windowed value
    //
    HWREGH(WD_BASE + SYSCTL_O_WDWCR) |= (value & SYSCTL_WDWCR_MIN_M);

    EDIS;
}

//*****************************************************************************
//
//! Clears the watchdog override.
//!
//! This function clears the watchdog override and locks the watchdog timer
//! module to remain in its prior state which could be either enable /disable.
//! The watchdog timer will remain in this state until the next system reset.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearWatchdogOverride(void)
{
    EALLOW;

    HWREGH(WD_BASE + SYSCTL_O_SCSR) |= SYSCTL_SCSR_WDOVERRIDE;

    EDIS;
}

//*****************************************************************************
//
//! Enable the NMI Global interrupt bit
//!
//! \b Note: This bit should be set after the device security related
//! initialization is complete.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableNMIGlobalInterrupt(void)
{
    EALLOW;

    HWREGH(NMI_BASE + NMI_O_CFG) |= NMI_CFG_NMIE;

    EDIS;
}

//*****************************************************************************
//
//! Read NMI interrupts.
//!
//! Read the current state of NMI interrupt.
//!
//! \return \b true if NMI interrupt is triggered, \b false if not.
//
//*****************************************************************************
static inline bool
SysCtl_getNMIStatus(void)
{
    //
    // Read and return the current value of the NMI flag register, masking out
    // all but the NMI bit.
    //
    return((HWREGH(NMI_BASE + NMI_O_FLG) & NMI_FLG_NMIINT) != 0U);
}

//*****************************************************************************
//
//! Read NMI Flags.
//!
//! Read the current state of individual NMI interrupts
//!
//! \return Value of NMIFLG register. These defines are provided to decode
//! the value:
//! - \b SYSCTL_NMI_NMIINT           -  NMI Interrupt Flag
//! - \b SYSCTL_NMI_CLOCKFAIL        -  Clock Fail Interrupt Flag
//! - \b SYSCTL_NMI_RAMUNCERR        -  RAM Uncorrectable Error NMI Flag
//! - \b SYSCTL_NMI_FLUNCERR         -  Flash Uncorrectable Error NMI Flag
//! - \b SYSCTL_NMI_CPU1HWBISTERR    -  HW BIST Error NMI Flag
//! - \b SYSCTL_NMI_CPU2HWBISTERR    -  HW BIST Error NMI Flag
//! - \b SYSCTL_NMI_PIEVECTERR       -  PIE Vector Fetch Error Flag
//! - \b SYSCTL_NMI_ERADNMI          -  ERAD Module NMI Flag
//! - \b SYSCTL_NMI_CLBNMI           -  Configurable Logic Block NMI Flag
//! - \b SYSCTL_NMI_CPU2WDRSN        -  CPU2 WDRSn Reset Indication Flag
//! - \b SYSCTL_NMI_CPU2NMIWDRSN     -  CPU2 NMIWDRSn Reset Indication Flag
//! - \b SYSCTL_NMI_CMNMIWDRSN       -  CM NMI watch dog has timed out.
//! - \b SYSCTL_NMI_ECATNMIN         -  NMI from EtherCAT reset out
//! - \b SYSCTL_NMI_CRC_FAIL         -  CRC calculation failed.
//! - \b SYSCTL_NMI_MCAN_ERR         -  MCAN module generated an ECC error.
//
//*****************************************************************************
static inline uint16_t
SysCtl_getNMIFlagStatus(void)
{
    //
    // Read and return the current value of the NMI flag register.
    //
    return(HWREGH(NMI_BASE + NMI_O_FLG));
}

//*****************************************************************************
//
//! Check if the individual NMI interrupts are set.
//!
//! \param nmiFlags Bit mask of the NMI interrupts that user wants to clear.
//! The bit format of this parameter is same as of the NMIFLG register. These
//! defines are provided:
//! - \b SYSCTL_NMI_NMIINT           -  NMI Interrupt Flag
//! - \b SYSCTL_NMI_CLOCKFAIL        -  Clock Fail Interrupt Flag
//! - \b SYSCTL_NMI_RAMUNCERR        -  RAM Uncorrectable Error NMI Flag
//! - \b SYSCTL_NMI_FLUNCERR         -  Flash Uncorrectable Error NMI Flag
//! - \b SYSCTL_NMI_CPU1HWBISTERR    -  HW BIST Error NMI Flag
//! - \b SYSCTL_NMI_CPU2HWBISTERR    -  HW BIST Error NMI Flag
//! - \b SYSCTL_NMI_PIEVECTERR       -  PIE Vector Fetch Error Flag
//! - \b SYSCTL_NMI_ERADNMI          -  ERAD Module NMI Flag
//! - \b SYSCTL_NMI_CLBNMI           -  Configurable Logic Block NMI Flag
//! - \b SYSCTL_NMI_CPU2WDRSN        -  CPU2 WDRSn Reset Indication Flag
//! - \b SYSCTL_NMI_CPU2NMIWDRSN     -  CPU2 NMIWDRSn Reset Indication Flag
//! - \b SYSCTL_NMI_CMNMIWDRSN       -  CM NMI watch dog has timed out.
//! - \b SYSCTL_NMI_ECATNMIN         -  NMI from EtherCAT reset out
//! - \b SYSCTL_NMI_CRC_FAIL         -  CRC calculation failed.
//! - \b SYSCTL_NMI_MCAN_ERR         -  MCAN module generated an ECC error.
//!
//! Check if interrupt flags corresponding to the passed in bit mask are
//! asserted.
//!
//! \return \b true if any of the NMI asked for in the parameter bit mask
//! is set. \b false if none of the NMI requested in the parameter bit mask are
//! set.
//
//*****************************************************************************
static inline bool
SysCtl_isNMIFlagSet(uint16_t nmiFlags)
{
    //
    // Check the arguments.
    // Make sure if reserved bits are not set in nmiFlags.
    //
    ASSERT((nmiFlags & ~(
                         SYSCTL_NMI_NMIINT           |
                         SYSCTL_NMI_CLOCKFAIL        |
                         SYSCTL_NMI_RAMUNCERR        |
                         SYSCTL_NMI_FLUNCERR         |
                         SYSCTL_NMI_CPU1HWBISTERR    |
                         SYSCTL_NMI_CPU2HWBISTERR    |
                         SYSCTL_NMI_PIEVECTERR       |
                         SYSCTL_NMI_ERADNMI          |
                         SYSCTL_NMI_CLBNMI           |
                         SYSCTL_NMI_CPU2WDRSN        |
                         SYSCTL_NMI_CPU2NMIWDRSN     |
                         SYSCTL_NMI_CMNMIWDRSN       |
                         SYSCTL_NMI_ECATNMIN         |
                         SYSCTL_NMI_CRC_FAIL         |
                         SYSCTL_NMI_MCAN_ERR
                         )) == 0U);

    //
    // Read the flag register and return true if any of them are set.
    //
    return((HWREGH(NMI_BASE + NMI_O_FLG) & nmiFlags) != 0U);
}

//*****************************************************************************
//
//! Function to clear individual NMI interrupts.
//!
//! \param nmiFlags Bit mask of the NMI interrupts that user wants to clear.
//! The bit format of this parameter is same as of the NMIFLG register. These
//! defines are provided:
//! - \b SYSCTL_NMI_NMIINT           -  NMI Interrupt Flag
//! - \b SYSCTL_NMI_CLOCKFAIL        -  Clock Fail Interrupt Flag
//! - \b SYSCTL_NMI_RAMUNCERR        -  RAM Uncorrectable Error NMI Flag
//! - \b SYSCTL_NMI_FLUNCERR         -  Flash Uncorrectable Error NMI Flag
//! - \b SYSCTL_NMI_CPU1HWBISTERR    -  HW BIST Error NMI Flag
//! - \b SYSCTL_NMI_CPU2HWBISTERR    -  HW BIST Error NMI Flag
//! - \b SYSCTL_NMI_PIEVECTERR       -  PIE Vector Fetch Error Flag
//! - \b SYSCTL_NMI_ERADNMI          -  ERAD Module NMI Flag
//! - \b SYSCTL_NMI_CLBNMI           -  Configurable Logic Block NMI Flag
//! - \b SYSCTL_NMI_CPU2WDRSN        -  CPU2 WDRSn Reset Indication Flag
//! - \b SYSCTL_NMI_CPU2NMIWDRSN     -  CPU2 NMIWDRSn Reset Indication Flag
//! - \b SYSCTL_NMI_CMNMIWDRSN       -  CM NMI watch dog has timed out.
//! - \b SYSCTL_NMI_ECATNMIN         -  NMI from EtherCAT reset out
//! - \b SYSCTL_NMI_CRC_FAIL         -  CRC calculation failed.
//! - \b SYSCTL_NMI_MCAN_ERR         -  MCAN module generated an ECC error.
//!
//! Clear NMI interrupt flags that correspond with the passed in bit mask.
//!
//! \b Note: The NMI Interrupt flag is always cleared by default and
//! therefore doesn't have to be included in the bit mask.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearNMIStatus(uint16_t nmiFlags)
{
    //
    // Check the arguments.
    // Make sure if reserved bits are not set in nmiFlags.
    //
    ASSERT((nmiFlags & ~(
                         SYSCTL_NMI_NMIINT           |
                         SYSCTL_NMI_CLOCKFAIL        |
                         SYSCTL_NMI_RAMUNCERR        |
                         SYSCTL_NMI_FLUNCERR         |
                         SYSCTL_NMI_CPU1HWBISTERR    |
                         SYSCTL_NMI_CPU2HWBISTERR    |
                         SYSCTL_NMI_PIEVECTERR       |
                         SYSCTL_NMI_ERADNMI          |
                         SYSCTL_NMI_CLBNMI           |
                         SYSCTL_NMI_CPU2WDRSN        |
                         SYSCTL_NMI_CPU2NMIWDRSN     |
                         SYSCTL_NMI_CMNMIWDRSN       |
                         SYSCTL_NMI_ECATNMIN         |
                         SYSCTL_NMI_CRC_FAIL         |
                         SYSCTL_NMI_MCAN_ERR
                         )) == 0U);

    EALLOW;

    //
    // Clear the individual flags as well as NMI Interrupt flag
    //
    HWREGH(NMI_BASE + NMI_O_FLGCLR) = nmiFlags;
    HWREGH(NMI_BASE + NMI_O_FLGCLR) = NMI_FLG_NMIINT;

    EDIS;
}

//*****************************************************************************
//
//! Clear all the NMI Flags that are currently set.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearAllNMIFlags(void)
{
    uint16_t nmiFlags;

    //
    // Read the flag status register and then write to the clear register,
    // clearing all the flags that were returned plus the NMI flag.
    //
    EALLOW;

    nmiFlags = SysCtl_getNMIFlagStatus();
    HWREGH(NMI_BASE + NMI_O_FLGCLR) = nmiFlags;
    HWREGH(NMI_BASE + NMI_O_FLGCLR) = NMI_FLG_NMIINT;

    EDIS;
}

//*****************************************************************************
//
//! Function to force individual NMI interrupt fail flags
//!
//! \param nmiFlags Bit mask of the NMI interrupts that user wants to clear.
//! The bit format of this parameter is same as of the NMIFLG register. These
//! defines are provided:
//! - \b SYSCTL_NMI_NMIINT           -  NMI Interrupt Flag
//! - \b SYSCTL_NMI_CLOCKFAIL        -  Clock Fail Interrupt Flag
//! - \b SYSCTL_NMI_RAMUNCERR        -  RAM Uncorrectable Error NMI Flag
//! - \b SYSCTL_NMI_FLUNCERR         -  Flash Uncorrectable Error NMI Flag
//! - \b SYSCTL_NMI_CPU1HWBISTERR    -  HW BIST Error NMI Flag
//! - \b SYSCTL_NMI_CPU2HWBISTERR    -  HW BIST Error NMI Flag
//! - \b SYSCTL_NMI_PIEVECTERR       -  PIE Vector Fetch Error Flag
//! - \b SYSCTL_NMI_ERADNMI          -  ERAD Module NMI Flag
//! - \b SYSCTL_NMI_CLBNMI           -  Configurable Logic Block NMI Flag
//! - \b SYSCTL_NMI_CPU2WDRSN        -  CPU2 WDRSn Reset Indication Flag
//! - \b SYSCTL_NMI_CPU2NMIWDRSN     -  CPU2 NMIWDRSn Reset Indication Flag
//! - \b SYSCTL_NMI_CMNMIWDRSN       -  CM NMI watch dog has timed out.
//! - \b SYSCTL_NMI_ECATNMIN         -  NMI from EtherCAT reset out
//! - \b SYSCTL_NMI_CRC_FAIL         -  CRC calculation failed.
//! - \b SYSCTL_NMI_MCAN_ERR         -  MCAN module generated an ECC error.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_forceNMIFlags(uint16_t nmiFlags)
{
    //
    // Check the arguments.
    // Make sure if reserved bits are not set in nmiFlags.
    //
    ASSERT((nmiFlags & ~(
                         SYSCTL_NMI_NMIINT           |
                         SYSCTL_NMI_CLOCKFAIL        |
                         SYSCTL_NMI_RAMUNCERR        |
                         SYSCTL_NMI_FLUNCERR         |
                         SYSCTL_NMI_CPU1HWBISTERR    |
                         SYSCTL_NMI_CPU2HWBISTERR    |
                         SYSCTL_NMI_PIEVECTERR       |
                         SYSCTL_NMI_ERADNMI          |
                         SYSCTL_NMI_CLBNMI           |
                         SYSCTL_NMI_CPU2WDRSN        |
                         SYSCTL_NMI_CPU2NMIWDRSN     |
                         SYSCTL_NMI_CMNMIWDRSN       |
                         SYSCTL_NMI_ECATNMIN         |
                         SYSCTL_NMI_CRC_FAIL         |
                         SYSCTL_NMI_MCAN_ERR
                        )) == 0U);

    EALLOW;

    //
    // Set the Flags for the individual interrupts in the NMI flag
    // force register
    //
    HWREGH(NMI_BASE + NMI_O_FLGFRC) |= nmiFlags;

    EDIS;
}

//*****************************************************************************
//
//! Gets the NMI watchdog counter value.
//!
//! \b Note: The counter is clocked at the SYSCLKOUT rate.
//!
//! \return Returns the NMI watchdog counter register's current value.
//
//*****************************************************************************
static inline uint16_t
SysCtl_getNMIWatchdogCounter(void)
{
    //
    // Read and return the NMI watchdog counter register's value.
    //
    return(HWREGH(NMI_BASE + NMI_O_WDCNT));
}

//*****************************************************************************
//
//! Sets the NMI watchdog period value.
//!
//! \param wdPeriod is the 16-bit value at which a reset is generated.
//!
//! This function writes to the NMI watchdog period register that holds the
//! value to which the NMI watchdog counter is compared. When the two registers
//! match, a reset is generated. By default, the period is 0xFFFF.
//!
//! \note If a value smaller than the current counter value is passed into the
//! \e wdPeriod parameter, a NMIRSn will be forced.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setNMIWatchdogPeriod(uint16_t wdPeriod)
{
    EALLOW;

    //
    // Write to the period register.
    //
    HWREGH(NMI_BASE + NMI_O_WDPRD) = wdPeriod;

    EDIS;
}

//*****************************************************************************
//
//! Gets the NMI watchdog period value.
//!
//! \return Returns the NMI watchdog period register's current value.
//
//*****************************************************************************
static inline uint16_t
SysCtl_getNMIWatchdogPeriod(void)
{
    //
    // Read and return the NMI watchdog period register's value.
    //
    return(HWREGH(NMI_BASE + NMI_O_WDPRD));
}

//*****************************************************************************
//
//! Read NMI Shadow Flags.
//!
//! Read the current state of individual NMI interrupts
//!
//! \return Value of NMISHDFLG register. These defines are provided to decode
//! the value:
//! - \b SYSCTL_NMI_NMIINT           -  NMI Interrupt Flag
//! - \b SYSCTL_NMI_CLOCKFAIL        -  Clock Fail Interrupt Flag
//! - \b SYSCTL_NMI_RAMUNCERR        -  RAM Uncorrectable Error NMI Flag
//! - \b SYSCTL_NMI_FLUNCERR         -  Flash Uncorrectable Error NMI Flag
//! - \b SYSCTL_NMI_CPU1HWBISTERR    -  HW BIST Error NMI Flag
//! - \b SYSCTL_NMI_CPU2HWBISTERR    -  HW BIST Error NMI Flag
//! - \b SYSCTL_NMI_PIEVECTERR       -  PIE Vector Fetch Error Flag
//! - \b SYSCTL_NMI_ERADNMI          -  ERAD Module NMI Flag
//! - \b SYSCTL_NMI_CLBNMI           -  Configurable Logic Block NMI Flag
//! - \b SYSCTL_NMI_CPU2WDRSN        -  CPU2 WDRSn Reset Indication Flag
//! - \b SYSCTL_NMI_CPU2NMIWDRSN     -  CPU2 NMIWDRSn Reset Indication Flag
//! - \b SYSCTL_NMI_CMNMIWDRSN       -  CM NMI watch dog has timed out.
//! - \b SYSCTL_NMI_ECATNMIN         -  NMI from EtherCAT reset out
//! - \b SYSCTL_NMI_CRC_FAIL         -  CRC calculation failed.
//! - \b SYSCTL_NMI_MCAN_ERR         -  MCAN module generated an ECC error.
//
//*****************************************************************************
static inline uint32_t
SysCtl_getNMIShadowFlagStatus(void)
{
    //
    // Read and return the current value of the NMI shadow flag register.
    //
    return(HWREGH(NMI_BASE + NMI_O_SHDFLG));
}

//*****************************************************************************
//
//! Check if the individual NMI shadow flags are set.
//!
//! \param nmiFlags Bit mask of the NMI interrupts that user wants  to clear.
//! The bit format of this parameter is same as of the NMIFLG register. These
//! defines are provided:
//! - \b SYSCTL_NMI_NMIINT           -  NMI Interrupt Flag
//! - \b SYSCTL_NMI_CLOCKFAIL        -  Clock Fail Interrupt Flag
//! - \b SYSCTL_NMI_RAMUNCERR        -  RAM Uncorrectable Error NMI Flag
//! - \b SYSCTL_NMI_FLUNCERR         -  Flash Uncorrectable Error NMI Flag
//! - \b SYSCTL_NMI_CPU1HWBISTERR    -  HW BIST Error NMI Flag
//! - \b SYSCTL_NMI_CPU2HWBISTERR    -  HW BIST Error NMI Flag
//! - \b SYSCTL_NMI_PIEVECTERR       -  PIE Vector Fetch Error Flag
//! - \b SYSCTL_NMI_ERADNMI          -  ERAD Module NMI Flag
//! - \b SYSCTL_NMI_CLBNMI           -  Configurable Logic Block NMI Flag
//! - \b SYSCTL_NMI_CPU2WDRSN        -  CPU2 WDRSn Reset Indication Flag
//! - \b SYSCTL_NMI_CPU2NMIWDRSN     -  CPU2 NMIWDRSn Reset Indication Flag
//! - \b SYSCTL_NMI_CMNMIWDRSN       -  CM NMI watch dog has timed out.
//! - \b SYSCTL_NMI_ECATNMIN         -  NMI from EtherCAT reset out
//! - \b SYSCTL_NMI_CRC_FAIL         -  CRC calculation failed.
//! - \b SYSCTL_NMI_MCAN_ERR         -  MCAN module generated an ECC error.
//!
//! Check if interrupt flags corresponding to the passed in bit mask are
//! asserted.
//!
//! \return \b true if any of the NMI asked for in the parameter bit mask
//! is set. \b false if none of the NMI requested in the parameter bit mask are
//! set.
//
//*****************************************************************************
static inline bool
SysCtl_isNMIShadowFlagSet(uint16_t nmiFlags)
{
    //
    // Check the arguments.
    // Make sure if reserved bits are not set in nmiFlags.
    //
    ASSERT((nmiFlags & ~(
                         SYSCTL_NMI_NMIINT           |
                         SYSCTL_NMI_CLOCKFAIL        |
                         SYSCTL_NMI_RAMUNCERR        |
                         SYSCTL_NMI_FLUNCERR         |
                         SYSCTL_NMI_CPU1HWBISTERR    |
                         SYSCTL_NMI_CPU2HWBISTERR    |
                         SYSCTL_NMI_PIEVECTERR       |
                         SYSCTL_NMI_ERADNMI          |
                         SYSCTL_NMI_CLBNMI           |
                         SYSCTL_NMI_CPU2WDRSN        |
                         SYSCTL_NMI_CPU2NMIWDRSN     |
                         SYSCTL_NMI_CMNMIWDRSN       |
                         SYSCTL_NMI_ECATNMIN         |
                         SYSCTL_NMI_CRC_FAIL         |
                         SYSCTL_NMI_MCAN_ERR
                        )) == 0U);

    //
    // Read the flag register and return true if any of them are set.
    //
    return((HWREGH(NMI_BASE + NMI_O_SHDFLG) & nmiFlags) != 0U);
}

//*****************************************************************************
//
//! Enable the missing clock detection (MCD) Logic
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableMCD(void)
{
    EALLOW;

    HWREGH(CLKCFG_BASE + SYSCTL_O_MCDCR) &= ~(SYSCTL_MCDCR_MCLKOFF);

    EDIS;
}

//*****************************************************************************
//
//! Disable the missing clock detection (MCD) Logic
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disableMCD(void)
{
    EALLOW;

    HWREGH(CLKCFG_BASE + SYSCTL_O_MCDCR) |= SYSCTL_MCDCR_MCLKOFF;

    EDIS;
}

//*****************************************************************************
//
//! Get the missing clock detection Failure Status
//!
//! \note A failure means the oscillator clock is missing
//!
//! \return Returns \b true if a failure is detected or \b false if a
//! failure isn't detected
//
//*****************************************************************************
static inline bool
SysCtl_isMCDClockFailureDetected(void)
{
    //
    //  Check the status bit to determine failure
    //
    return((HWREGH(CLKCFG_BASE + SYSCTL_O_MCDCR) & SYSCTL_MCDCR_MCLKSTS) != 0U);
}

//*****************************************************************************
//
//! Reset the missing clock detection logic after clock failure
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_resetMCD(void)
{
    EALLOW;

    HWREGH(CLKCFG_BASE + SYSCTL_O_MCDCR) |= SYSCTL_MCDCR_MCLKCLR;

    EDIS;
}

//*****************************************************************************
//
//! Re-connect missing clock detection clock source to stop simulating clock
//! failure
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_connectMCDClockSource(void)
{
    EALLOW;

    HWREGH(CLKCFG_BASE + SYSCTL_O_MCDCR) &= ~(SYSCTL_MCDCR_OSCOFF);

    EDIS;
}

//*****************************************************************************
//
//! Disconnect missing clock detection clock source to simulate clock failure.
//! This is for testing the MCD functionality.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disconnectMCDClockSource(void)
{
    EALLOW;

    HWREGH(CLKCFG_BASE + SYSCTL_O_MCDCR) |= SYSCTL_MCDCR_OSCOFF;

    EDIS;
}

//*****************************************************************************
//
//! Lock the Access Control Registers
//!
//! This function locks the access control registers and puts them in a
//! read-only state.
//!
//! \note Only a reset can unlock the access control registers.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_lockAccessControlRegs(void)
{
    EALLOW;

    HWREGH(PERIPHAC_BASE + SYSCTL_O_PERIPH_AC_LOCK) |=
        SYSCTL_PERIPH_AC_LOCK_LOCK_AC_WR;

    EDIS;
}

//*****************************************************************************
//
//! Set the peripheral access control permissions
//!
//! \param peripheral is the selected peripheral
//! \param controller is the selected controller
//! \param permission is the selected access permissions
//!
//! This function sets the specified peripheral access control permissions for
//! the the specified controller
//!
//! The \e peripheral parameter can have one enumerated value in the format of
//! \b SYSCTL_ACCESS_X where X is the name of the peripheral instance to be
//! configured such as \b SYSCTL_ACCESS_ADCA.
//!
//! The \e controller parameter can have one the following enumerated values:
//! - \b SYSCTL_ACCESS_CPUX
//! - \b SYSCTL_ACCESS_CLA1
//! - \b SYSCTL_ACCESS_DMA1
//!
//! The \e permission parameter can have one the following enumerated values:
//! - \b SYSCTL_ACCESS_FULL      - Full Access for both read and write
//! - \b SYSCTL_ACCESS_PROTECTED - Protected read access such that FIFOs, clear
//!                                on read registers are not changed, and no
//!                                write access
//! - \b SYSCTL_ACCESS_NONE      - No read or write access
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setPeripheralAccessControl(SysCtl_AccessPeripheral peripheral,
                                  SysCtl_AccessController controller,
                                  SysCtl_AccessPermission permission)
{
    //
    // Set controller permissions for specified peripheral. Each controller has
    // two bits dedicated to its permission setting.
    //
    EALLOW;

    HWREGH(PERIPHAC_BASE + (uint16_t)peripheral) =
        (HWREGH(PERIPHAC_BASE + (uint16_t)peripheral) &
         ~(0x3U << (uint16_t)controller)) |
        ((uint16_t)permission << (uint16_t)controller);

    EDIS;
}

//*****************************************************************************
//
//! Get the peripheral access control permissions
//!
//! \param peripheral is the selected peripheral
//! \param controller is the selected controller
//!
//! This function gets the specified peripheral access control permissions for
//! the the specified controller
//!
//! The \e peripheral parameter can have one enumerated value in the format of
//! \b SYSCTL_ACCESS_X where X is the name of the peripheral instance to be
//! configured such as \b SYSCTL_ACCESS_ADCA.
//!
//! The \e controller parameter can have one the following enumerated values:
//! - \b SYSCTL_ACCESS_CPUX
//! - \b SYSCTL_ACCESS_CLA1
//! - \b SYSCTL_ACCESS_DMA1
//!
//! \return Returns one of the following enumerated permission values:
//! - \b SYSCTL_ACCESS_FULL      - Full Access for both read and write
//! - \b SYSCTL_ACCESS_PROTECTED - Protected read access such that FIFOs, clear
//!                                on read registers are not changed, and no
//!                                write access
//! - \b SYSCTL_ACCESS_NONE      - No read or write access
//
//*****************************************************************************
static inline uint16_t
SysCtl_getPeripheralAccessControl(SysCtl_AccessPeripheral peripheral,
                                  SysCtl_AccessController controller)
{
    //
    // Read controller permissions for specified peripheral. Each controller has
    // two bits dedicated to its permission setting.
    //
    return((HWREGH(PERIPHAC_BASE + (uint16_t)peripheral) >>
           (uint16_t)controller) & 0x3U);
}

//*****************************************************************************
//
//! Configures the sync output source.
//!
//! \param syncSrc is sync output source selection.
//!
//! This function configures the sync output source from the ePWM modules. The
//! \e syncSrc parameter is a value \b SYSCTL_SYNC_OUT_SRC_XXXX, where XXXX is
//! a sync signal coming from an ePWM such as SYSCTL_SYNC_OUT_SRC_EPWM1SYNCOUT
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setSyncOutputConfig(SysCtl_SyncOutputSource syncSrc)
{
    //
    // Write the sync output source selection to the appropriate register.
    //
    EALLOW;
    HWREG(SYNCSOC_BASE + SYSCTL_O_SYNCSELECT) =
        (HWREG(SYNCSOC_BASE + SYSCTL_O_SYNCSELECT) &
         ~((uint32_t)SYSCTL_SYNCSELECT_SYNCOUT_M)) |
        ((uint32_t)syncSrc << SYSCTL_SYNCSELECT_SYNCOUT_S);
    EDIS;

}
//*****************************************************************************
//
//! Enables ePWM SOC signals to drive an external (off-chip) ADCSOC signal.
//!
//! \param adcsocSrc is a bit field of the selected signals to be enabled
//!
//! This function configures which ePWM SOC signals are enabled as a source for
//! either ADCSOCAO or ADCSOCBO. The \e adcsocSrc parameter takes a logical OR
//! of \b SYSCTL_ADCSOC_SRC_PWMxSOCA/B values that correspond to different
//! signals.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableExtADCSOCSource(uint32_t adcsocSrc)
{
    //
    // Set the bits that correspond to signal to be enabled.
    //
    EALLOW;
    HWREG(SYNCSOC_BASE + SYSCTL_O_ADCSOCOUTSELECT) |= adcsocSrc;
    EDIS;
}

//*****************************************************************************
//
//! Disables ePWM SOC signals from driving an external ADCSOC signal.
//!
//! \param adcsocSrc is a bit field of the selected signals to be disabled
//!
//! This function configures which ePWM SOC signals are disabled as a source
//! for either ADCSOCAO or ADCSOCBO. The \e adcsocSrc parameter takes a logical
//! OR of \b SYSCTL_ADCSOC_SRC_PWMxSOCA/B values that correspond to different
//! signals.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disableExtADCSOCSource(uint32_t adcsocSrc)
{
    //
    // Clear the bits that correspond to signal to be disabled.
    //
    EALLOW;
    HWREG(SYNCSOC_BASE + SYSCTL_O_ADCSOCOUTSELECT) &= ~adcsocSrc;
    EDIS;
}

//*****************************************************************************
//
//! Locks the SOC Select of the Trig X-BAR.
//!
//! This function locks the external ADC SOC select of the Trig X-BAR.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_lockExtADCSOCSelect(void)
{
    //
    // Lock the ADCSOCOUTSELECT bit of the SYNCSOCLOCK register.
    //
    EALLOW;
    HWREG(SYNCSOC_BASE + SYSCTL_O_SYNCSOCLOCK) =
        SYSCTL_SYNCSOCLOCK_ADCSOCOUTSELECT;
    EDIS;
}

//*****************************************************************************
//
//! Locks the Sync Select of the Trig X-BAR.
//!
//! This function locks Sync Input and Output Select of the Trig X-BAR.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_lockSyncSelect(void)
{
    //
    // Lock the SYNCSELECT register.
    //
    EALLOW;
    HWREG(SYNCSOC_BASE + SYSCTL_O_SYNCSOCLOCK) = SYSCTL_SYNCSOCLOCK_SYNCSELECT;
    EDIS;
}
//*****************************************************************************
//
//! Configures whether a peripheral is connected to CPU1 or CPU2.
//!
//! \param peripheral is the peripheral for which CPU needs to be configured.
//! \param cpuInst is the CPU to which the peripheral instance need to be
//!                connected.
//!
//! The \e peripheral parameter can have one enumerated value from
//! SysCtl_CPUSelPeriphInstance
//!
//! The \e cpuInst parameter can have one the following values:
//! - \b SYSCTL_CPUSEL_CPU1 - to connect to CPU1
//! - \b SYSCTL_CPUSEL_CPU2 - to connect to CPU2
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_selectCPUForPeripheralInstance(SysCtl_CPUSelPeriphInstance peripheral,
                                      SysCtl_CPUSel cpuInst)
{
    uint16_t regIndex;
    uint16_t bitIndex;

    //
    // Identify the register index and bit position
    //
    regIndex = 2U * ((uint16_t)peripheral & SYSCTL_PERIPH_REG_M);
    bitIndex = ((uint16_t)peripheral & SYSCTL_PERIPH_BIT_M) >>
               SYSCTL_PERIPH_BIT_S;

    EALLOW;

    //
    // Configure the CPU owner for the specified peripheral instance
    //
    HWREG(DEVCFG_BASE + SYSCTL_O_CPUSEL0 + regIndex) =
       (HWREG(DEVCFG_BASE + SYSCTL_O_CPUSEL0 + regIndex) & ~(1UL << bitIndex)) |
       ((uint32_t)cpuInst << bitIndex);

    EDIS;

}
//*****************************************************************************
//
//! Configures whether a peripheral is connected to CPU1 or CPU2.
//!
//! \param peripheral is the peripheral for which CPU needs to be configured.
//! \param peripheralInst is the instance for which CPU needs to be configured.
//! \param cpuInst is the CPU to which the peripheral instance need to be
//!                connected.
//!
//! The \e peripheral parameter can have one enumerated value from
//! SysCtl_CPUSelPeripheral
//!
//! The \e peripheralInst parameter is the instance number for example
//! 1 for EPWM1, 2 for EPWM2 so on.For instances which are named with alphabets
//! (instead of numbers) the following convention needs to be followed.
//! 1 for A (SPI_A), 2 for B (SPI_B), 3 for C (SPI_C) so on...
//! For peripherals with different RX and TX instances which are named
//! with alphabets, the following convention needs to be followed.
//! 1 for TX_A (FSITX_A), 2 for TX_B (FSITX_B) and so on...
//! 17 for RX_A (FSIRX_A), 18 for RX_B (FSIRX_B) and so on...
//!
//! The \e cpuInst parameter can have one the following values:
//! - \b SYSCTL_CPUSEL_CPU1 - to connect to CPU1
//! - \b SYSCTL_CPUSEL_CPU2 - to connect to CPU2
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \note This function is retained for compatibility puposes. Recommended to
//! to use the function \e SysCtl_selectCPUForPeripheralInstance()
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_selectCPUForPeripheral(SysCtl_CPUSelPeripheral peripheral,
                              uint16_t peripheralInst, SysCtl_CPUSel cpuInst)
{
    uint32_t tempValue;
    uint16_t shift;

    if(SYSCTL_CPUSEL14_DAC == peripheral)
    {
        shift = peripheralInst + SYSCTL_CPUSEL_DAC_S - 1U;
    }
    else
    {
        shift = peripheralInst - 1U;
    }

    tempValue =
        HWREG(DEVCFG_BASE + SYSCTL_O_CPUSEL0 + ((uint32_t)peripheral * 2U)) &
        (~(1UL << shift));

    EALLOW;
    HWREG(DEVCFG_BASE + SYSCTL_O_CPUSEL0 + ((uint32_t)peripheral * 2U)) =
        tempValue | ((uint32_t)cpuInst << shift);
    EDIS;
}
//*****************************************************************************
//
//! Get the Device Silicon Revision ID
//!
//! This function returns the silicon revision ID for the device.
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return Returns the silicon revision ID value.
//
//*****************************************************************************
static inline uint32_t
SysCtl_getDeviceRevision(void)
{
    //
    // Returns the device silicon revision ID
    //
    return(HWREG(DEVCFG_BASE + SYSCTL_O_REVID));
}

//*****************************************************************************
//
//! Locks the CPU select registers for the peripherals
//!
//! \param peripheral is the peripheral for which CPU needs to be selected.
//!
//! The \e peripheral parameter can have one enumerated value from
//! SysCtl_CPUSelPeripheral
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_lockCPUSelectRegs(SysCtl_CPUSelPeripheral peripheral)
{
    EALLOW;
    HWREG(DEVCFG_BASE + SYSCTL_O_DEVCFGLOCK1) |= (1UL << (uint32_t)peripheral);
    EDIS;
}


//*****************************************************************************
//
//! Gets the error status of the Efuse
//!
//! The function provides both the Efuse Autoload & the Efuse Self Test
//! Error Status.
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return Fuse Error status.
//
//*****************************************************************************
static inline uint16_t
SysCtl_getEfuseError(void)
{
    return(HWREGH(DEVCFG_BASE + SYSCTL_O_FUSEERR));
}

//*****************************************************************************
//
//! Reads the ADC wrapper mode for the provided ADC
//!
//! \param peripheral is the ADC for which the configuration is to be read.
//!
//! The \e peripheral parameter can have one enumerated value from
//! SysCtl_SelADC
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return Returns the supported resolution of the particular ADC.
//!  This can be either of the 2 resolutions below:
//!  1. Operate in 16-bit or 12-bit configurable (0x0U)
//!  2. Operate only in 12-bit configurable (0x1U)
//
//*****************************************************************************
static inline uint32_t
SysCtl_readADCWrapper(SysCtl_SelADC peripheral)
{

    //
    // Reads the ADC wrapper mode
    //
    return((HWREG(DEVCFG_BASE + SYSCTL_O_PERCNF1) &
            (SYSCTL_PERCNF1_ADC_A_MODE << (uint32_t)peripheral)) >>
           (uint32_t)peripheral);
}

//*****************************************************************************
//
//! Check if the CPU2 is held in reset or not
//!
//! Provides the reset status of CPU2 to CPU1
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return \b true if CPU2 core is in reset \b false if CPU2 core
//! is out of reset
//
//*****************************************************************************
static inline bool
SysCtl_isCPU2Reset(void)
{
    return((HWREGH(DEVCFG_BASE + SYSCTL_O_RSTSTAT) &
            SYSCTL_RSTSTAT_CPU2RES) == 0U);
}

//*****************************************************************************
//
//! Gets the status/causes of the CPU2 reset.
//!
//! The function provides causes of the CPU2 being in reset / not.
//! This could be either of the following values or a combination of both:
//!
//! - SYSCTL_RSTSTAT_CPU2RES
//! - SYSCTL_RSTSTAT_CPU2NMIWDRST
//! - SYSCTL_RSTSTAT_CPU2HWBISTRST
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return Reset cause of CPU2.
//
//*****************************************************************************
static inline uint16_t
SysCtl_getCPU2ResetStatus(void)
{
    return(HWREGH(DEVCFG_BASE + SYSCTL_O_RSTSTAT));
}

//*****************************************************************************
//
//! Clears the CPU2 reset status.
//!
//! \param rstCauses are the reset causes to be cleared
//!
//! This function clears the CPU2 reset status. This could be either of the
//! following values or a combination of both:
//!
//! - SYSCTL_RSTSTAT_CPU2NMIWDRST
//! - SYSCTL_RSTSTAT_CPU2HWBISTRST.
//!
//! To check if it was set first, see SysCtl_getCPU2ResetStatus().
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearCPU2ResetStatus(uint16_t rstCauses )
{
    HWREGH(DEVCFG_BASE + SYSCTL_O_RSTSTAT) |= rstCauses;
}
//*****************************************************************************
//
//! Gets the state of the CPU2.
//!
//! The function indicates the power mode of the CPU2. It could be either in
//! ACTIVE , IDLE or STANDBY mode.
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return Power mode of CPU2.
//
//*****************************************************************************
static inline uint16_t
SysCtl_getCPU2LPMStatus(void)
{
    return(HWREGH(DEVCFG_BASE + SYSCTL_O_LPMSTAT));
}

//*****************************************************************************
//
//! Gets the Ownership details for clock configuration
//!
//! A CPU can perform read/writes to any of the CLKCFG registers only
//! if  it owns the semaphore. Otherwise, writes are ignored and reads
//! will return 0x0.
//!
//! \note Reads and writes of this ownership register are always
//! allowed from both CPU1 and CPU2.
//!
//! \return Clock control Ownership details
//
//*****************************************************************************
static inline uint16_t
SysCtl_getSemOwner(void)
{

    return(HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSEM));
}

//*****************************************************************************
//
//! Sets up XCLK divider.
//!
//! \param divider is the value that configures the divider.
//!
//! This function sets up the XCLK divider. There is only one
//! divider that scales INTOSC1 to XCLK.
//!
//! The \e divider parameter can have one enumerated value from
//! SysCtl_XClkDivider
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setXClk(SysCtl_XClkDivider divider)
{
    //
    // Clears the divider then configures it.
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_XCLKOUTDIVSEL) =
                        (HWREGH(CLKCFG_BASE + SYSCTL_O_XCLKOUTDIVSEL) &
                         ~(SYSCTL_XCLKOUTDIVSEL_XCLKOUTDIV_M)) |
                        (uint16_t)divider;
    EDIS;
}

//*****************************************************************************
//
//! Sets up CM CLK source & divider.
//!
//! \param divider is the value that configures the divider.
//! \param source is the source for the clock divider
//!
//! This function sets up the CM CLK divider based on the source that
//! is selected. There is only one divider that scales the "source" to CM CLK.
//!
//! The \e divider parameter can have one enumerated value from
//! SysCtl_CMClkDivider
//! The \e source parameter can have one enumerated value from
//! SysCtl_PLLClockSource
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setCMClk(SysCtl_CMClkDivider divider, SysCtl_PLLClockSource source)
{
    //
    // Clears the divider & the source, then configures it.
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_CMCLKCTL) =
                (HWREGH(CLKCFG_BASE + SYSCTL_O_CMCLKCTL) &
                 ~(SYSCTL_CMCLKCTL_CMCLKDIV_M | SYSCTL_CMCLKCTL_CMDIVSRCSEL));

    HWREGH(CLKCFG_BASE + SYSCTL_O_CMCLKCTL) |=
                    ((uint16_t)divider << SYSCTL_CMCLKCTL_CMCLKDIV_S) |
                    ((uint16_t)source << SYSCTL_CMCLKCTL_CMDIVSRCSEL_S);
    EDIS;
}

//*****************************************************************************
//
//! Sets up Ethernet CLK source & divider.
//!
//! \param divider is the value that configures the divider.
//! \param source is the source for the clock divider
//!
//! This function sets up the Ethernet CLK divider based on the source that
//! is selected. There is only one divider that scales the "source" to
//! Ethernet CLK.
//!
//! The \e divider parameter can have one enumerated value from
//! SysCtl_EnetClkDivider
//! The \e source parameter can have one enumerated value from
//! SysCtl_PLLClockSource
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setEnetClk(SysCtl_EnetClkDivider divider, SysCtl_PLLClockSource source)
{
    //
    // Clears the divider & the source, then configures it.
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_CMCLKCTL) =
                    (HWREGH(CLKCFG_BASE + SYSCTL_O_CMCLKCTL) &
                    ~(SYSCTL_CMCLKCTL_ETHDIV_M | SYSCTL_CMCLKCTL_ETHDIVSRCSEL));

    HWREGH(CLKCFG_BASE + SYSCTL_O_CMCLKCTL) |=
                    ((uint16_t)divider << SYSCTL_CMCLKCTL_ETHDIV_S) |
                    ((uint16_t)source << SYSCTL_CMCLKCTL_ETHDIVSRCSEL_S);
    EDIS;
}

//*****************************************************************************
//
//! Sets up Ethercat CLK source, divider & PHY clock.
//!
//! \param divider is the value that configures the divider.
//! \param source is the source for the clock divider
//! \param enable enables/disables the etherCAT PHY clock
//!
//! This function sets up the Ethercat CLK divider based on the source that
//! is selected. There is only one divider that scales the "source" to
//! Ethercat CLK.
//! This also enables/disables the etherCAT PHY clock.
//!
//! The \e divider parameter can have one enumerated value from
//! SysCtl_ECatClkDivider
//! The \e source parameter can have one enumerated value from
//! SysCtl_PLLClockSource
//! The \e enable parameter can be either of these values:
//! 0x0U: etherCAT PHY clock is disabled
//! 0x1U: etherCAT PHY clock is enabled
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setECatClk(SysCtl_ECatClkDivider divider, SysCtl_PLLClockSource source,
                        uint16_t enable)
{
    //
    // Clears the divider & the source, then configures it.
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_ETHERCATCLKCTL) =
                    (HWREGH(CLKCFG_BASE + SYSCTL_O_ETHERCATCLKCTL) &
                     ~(SYSCTL_ETHERCATCLKCTL_PHYCLKEN  |
                       SYSCTL_ETHERCATCLKCTL_ECATDIV_M |
                       SYSCTL_ETHERCATCLKCTL_DIVSRCSEL));

    HWREGH(CLKCFG_BASE + SYSCTL_O_ETHERCATCLKCTL) |=
                ((uint16_t)divider << SYSCTL_ETHERCATCLKCTL_ECATDIV_S) |
                ((uint16_t)source << SYSCTL_ETHERCATCLKCTL_DIVSRCSEL_S) |
                (enable << SYSCTL_ETHERCATCLKCTL_PHYCLKEN_S);
    EDIS;
}

//*****************************************************************************
//
//! Sets up PLLSYSCLK divider.
//!
//! \param divider is the value that configures the divider.
//!
//! This function sets up the PLLSYSCLK divider. There is only one
//! divider that scales PLLSYSCLK to generate the system clock.
//!
//! The \e divider parameter can have one value from the set below:
//!     0x0 = /1
//!     0x1 = /2
//!     0x2 = /4 (default on reset)
//!     0x3 = /6
//!     0x4 = /8
//!    ......
//!     0x3F =/126
//!
//! \return None.
//!
//! \note Please make sure to check if the PLL is locked and valid using the
//! SysCtl_isPLLValid() before setting the divider.
//
//*****************************************************************************
static inline void
SysCtl_setPLLSysClk(uint16_t divider)
{
    //
    // Clears the divider then configures it.
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) =
                    (HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                     ~(SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M)) | divider;
    EDIS;
}

//*****************************************************************************
//
//! Sets up AUXPLLCLK divider.
//!
//! \param divider is the value that configures the divider.
//!
//! This function sets up the AUXPLLCLK divider. There is only one
//! divider that scales AUXPLLCLK to generate the system clock.
//!
//! The \e divider parameter can have one enumerated value from
//! SysCtl_AuxPLLClkDivider
//!
//! \return None.
//!
//! \note Please make sure to check if the PLL is locked and valid using the
//! SysCtl_isPLLValid() before setting the divider.
//
//*****************************************************************************
static inline void
SysCtl_setAuxPLLClk(SysCtl_AuxPLLClkDivider divider)
{
    //
    // Clears the divider then configures it.
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_AUXCLKDIVSEL) =
                    (HWREGH(CLKCFG_BASE + SYSCTL_O_AUXCLKDIVSEL) &
                     ~(SYSCTL_AUXCLKDIVSEL_AUXPLLDIV_M)) | (uint16_t)divider;
    EDIS;
}

//*****************************************************************************
//
//! Sets up MCAN Clk divider.
//!
//! \param divider is the value that configures the divider.
//!
//! This function sets up the MCANCLK divider. There is only one
//! divider that scales MCAN clock.
//!
//! The \e divider parameter can have one enumerated value from
//! SysCtl_MCANClkDivider
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setMCANClk(SysCtl_MCANClkDivider divider)
{
    //
    // Clears the divider then configures it.
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_AUXCLKDIVSEL) =
                    (HWREGH(CLKCFG_BASE + SYSCTL_O_AUXCLKDIVSEL) &
                     ~(SYSCTL_AUXCLKDIVSEL_MCANCLKDIV_M)) |
                    ((uint16_t)divider << SYSCTL_AUXCLKDIVSEL_MCANCLKDIV_S);

    EDIS;
}

//*****************************************************************************
//
//! Sets up CPU Timer 2  CLK source & divider.
//!
//! \param divider is the value that configures the divider.
//! \param source is the source for the clock divider
//!
//! This function sets up the CPU Timer 2 CLK divider based on the source that
//! is selected. There is only one divider that scales the "source" to
//! CPU Timer 2 CLK.
//!
//! The \e divider parameter can have one enumerated value from
//! SysCtl_Cputimer2ClkDivider
//! The \e source parameter can have one enumerated value from
//! SysCtl_Cputimer2ClkSource
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setCputimer2Clk(SysCtl_Cputimer2ClkDivider divider,
                               SysCtl_Cputimer2ClkSource source)
{
  //
    // Clears the divider & the source, then configures it.
    //
    EALLOW;
    HWREGH(CPUSYS_BASE + SYSCTL_O_TMR2CLKCTL) =
                    (HWREGH(CPUSYS_BASE + SYSCTL_O_TMR2CLKCTL) &
                     ~(SYSCTL_TMR2CLKCTL_TMR2CLKSRCSEL_M |
                       SYSCTL_TMR2CLKCTL_TMR2CLKPRESCALE_M));

    HWREGH(CPUSYS_BASE + SYSCTL_O_TMR2CLKCTL) |=
                    ((uint16_t)divider << SYSCTL_TMR2CLKCTL_TMR2CLKPRESCALE_S) |
                    ((uint16_t)source << SYSCTL_TMR2CLKCTL_TMR2CLKSRCSEL_S);
    EDIS;
}

//*****************************************************************************
//
//! Gets the PIE Vector Fetch Error Handler Routine Address.
//!
//! The function indicates the address of the PIE Vector Fetch Error
//! handler routine.
//!
//! \return Error Handler Address.
//!
//! \note Its the responsibility of user to initialize this register. If this
//! register is not initialized, a default error handler at address
//! 0x3fffbe will get executed.
//
//*****************************************************************************
static inline uint32_t
SysCtl_getPIEVErrAddr(void)
{
    return(HWREG(CPUSYS_BASE + SYSCTL_O_PIEVERRADDR));
}

//*****************************************************************************
//
//! Simulates a reset to the CPU1
//!
//! \param rstCauses is the cause for the reset.
//!
//! The \e rstCauses parameter can be one/ more of these values:
//! SYSCTL_CAUSE_CPU1RSN or SYSCTL_CAUSE_XRS
//!
//! \return None.
//!
//! \note This API exists only on CPU1
//
//*****************************************************************************
static inline void
SysCtl_simulateReset(uint32_t rstCauses)
{
    //
    //Write will succeed only if a matching key value is written
    //to the KEY field
    //Sets the appropriate reset bit.
    //
    HWREG(CPUSYS_BASE + SYSCTL_O_SIMRESET)  = (rstCauses |
                                              (SYSCTL_REG_KEY &
                                               SYSCTL_SIMRESET_KEY_M));
}

//*****************************************************************************
//
//! Check if the CM is held in reset or not
//!
//! Provides the reset status of CM to the CPU
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return \b true if CM is in reset \b false if CM is out of reset
//
//*****************************************************************************
static inline bool
SysCtl_isCMReset(void)
{
    return((HWREGH(CMCONF_BASE + SYSCTL_O_CMRESCTL) &
            SYSCTL_CMRESCTL_RESETSTS) == 0U);
}

//*****************************************************************************
//
//! Enables the NMI generation to the C28x core
//!
//! \param Flags is the Bit mask of the NMI interrupts that user
//! wants, to generate an NMI to the C28x.
//!
//! The \e Flags parameter can be the value :
//! SYSCTL_FLAG_CMNMIWDRST.
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableCMtoCPUNMI (uint16_t Flags)
{
    EALLOW;
    HWREGH(CMCONF_BASE + SYSCTL_O_CMTOCPU1NMICTL)|= Flags ;
    EDIS;
}

//*****************************************************************************
//
//! Disables the NMI generation to the C28x core
//!
//! \param Flags is the Bit mask of the NMI interrupts that user
//! wants, to generate an NMI to the C28x.
//!
//! The \e Flags parameter can be the value :
//! SYSCTL_FLAG_CMNMIWDRST.
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disableCMtoCPUNMI (uint16_t Flags)
{
    EALLOW;
    HWREGH(CMCONF_BASE + SYSCTL_O_CMTOCPU1NMICTL) &= ~Flags ;
    EDIS;
}

//*****************************************************************************
//
//! Gets the NMI generated on the C28x core
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return the NMI generated to the C28x.  The value can be:
//! SYSCTL_FLAG_CMNMIWDRST.
//
//*****************************************************************************
static inline uint16_t
SysCtl_getCMtoCPUNMI (void)
{
    return(HWREGH(CMCONF_BASE + SYSCTL_O_CMTOCPU1NMICTL));
}

//*****************************************************************************
//
//! Enables the Interrupt generation to the C28x core
//!
//! \param Flags is the Bit mask of the Interrupts that user
//! wants, to generate an Interrupt to the C28x.
//!
//! The \e Flags parameter can be the value :
//! SYSCTL_FLAG_CMNMIWDRST.
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableCMtoCPUInterrupt (uint16_t Flags)
{
    EALLOW;
    HWREGH(CMCONF_BASE + SYSCTL_O_CMTOCPU1INTCTL) |= Flags ;
    EDIS;
}

//*****************************************************************************
//
//! Disables the Interrupt generation to the C28x core
//!
//! \param Flags is the Bit mask of the Interrupts that user
//! wants, to generate an Interrupt to the C28x.
//!
//! The \e Flags parameter can be the value :
//! SYSCTL_FLAG_CMNMIWDRST.
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disableCMtoCPUInterrupt (uint16_t Flags)
{
    EALLOW;
    HWREGH(CMCONF_BASE + SYSCTL_O_CMTOCPU1INTCTL) &= ~Flags ;
    EDIS;
}

//*****************************************************************************
//
//! Gets the Interrupt generated on the C28x core
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return the Interrupt generated to the C28x.  The value can be:
//! SYSCTL_FLAG_CMNMIWDRST.
//
//*****************************************************************************
static inline uint16_t
SysCtl_getCMtoCPUInterrupt (void)
{
    return(HWREGH(CMCONF_BASE + SYSCTL_O_CMTOCPU1INTCTL));
}

//*****************************************************************************
//
//! Configures the CM Peripheral Allocation for shared peripherals.
//!
//! \param peripheral is the peripheral being allocated to CM.
//! \param allocate decides if the peripheral is allocated to the C28x CPU1/2
//! or the CM.
//!
//! The \e peripheral parameter can have one enumerated value from
//! SysCtl_SharedPeripheral
//!
//! The \e allocate parameter can have one of the values from:
//! 0x0U: Peripheral is allocated to C28x CPU1/2, CM accesses to it will
//! be ignored and interrupts from it will not be generated to CM.
//! 0x1U: Peripheral is allocated to CM, C28x CPU1/2 accesses to it will
//! be ignored and interrupts from it will not be generated to C28x CPU1/2.
//!
//! \return None.
//!
//! \note This API must be configured prior to enabling the
//! peripheral clocks. CPU2 does not have access to this API for MCAN
//! EtherCAT & USB peripherals.
//
//*****************************************************************************
static inline void
SysCtl_allocateSharedPeripheral(SysCtl_SharedPeripheral peripheral,
                                uint16_t allocate)
{
    EALLOW;

    //
    // Allocates the peripheral based on if the C28x or CM need access
    //
    if(allocate != 0x0U)
    {
        HWREG(CMCONF_BASE + SYSCTL_O_PALLOCATE0) |=
                    (1UL << (uint16_t)peripheral);
    }
    else
    {
        HWREG(CMCONF_BASE + SYSCTL_O_PALLOCATE0) &=
                    ~(1UL << (uint16_t)peripheral);
    }
    EDIS;
}

//*****************************************************************************
//
//! Locks the CM configuration registers
//!
//! This function locks the CM configuration registers below.
//! 1. PALLOCATE0
//! 2. CMTOCPU1NMICTL
//! 3. CMTOCPU1INTCTL
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_lockCMConfig(void)
{
    EALLOW;
    HWREG(CMCONF_BASE + SYSCTL_O_CM_CONF_REGS_LOCK) =
                                    SYSCTL_CM_CONF_REGS_LOCK_LOCK;
    EDIS;
}

//*****************************************************************************
//
//! Gets the status of interrupts due to multiple sources of Cortex-M4 reset.
//!
//! \return the Interrupt generated on reset of the CM.  The values can be
//! one/ more from:
//! SYSCTL_STATUS_CMGINT , SYSCTL_STATUS_CMNMIWDRST ,
//! SYSCTL_STATUS_CMSYSRESETREQ or SYSCTL_STATUS_CMVECTRESET.
//!
//! \note This API  is present only on CPU1.
//
//*****************************************************************************
static inline uint32_t
SysCtl_getCMInterruptStatus(void)
{
    return(HWREG(SYSSTAT_BASE + SYSCTL_O_CM_STATUS_INT_FLG));
}

//*****************************************************************************
//
//! Clears the interrupts due to multiple sources of Cortex-M4 reset.
//!
//! \param intFlags is the interrupt that needs to be cleared.
//!
//! The \e intFlags parameter are the Interrupts generated on reset of the CM
//! that need to be cleared. The values can be one/ more from:
//! SYSCTL_STATUS_CMGINT , SYSCTL_STATUS_CMNMIWDRST ,
//! SYSCTL_STATUS_CMSYSRESETREQ or SYSCTL_STATUS_CMVECTRESET.
//!
//! \return None.
//!
//! \note This API  is present only on CPU1.
//
//*****************************************************************************
static inline void
SysCtl_clearCMInterruptStatus(uint32_t intFlags)
{
    HWREGH(SYSSTAT_BASE + SYSCTL_O_CM_STATUS_INT_CLR) |= (uint16_t)intFlags;
}

//*****************************************************************************
//
//! Sets the interrupt for the multiple sources of Cortex-M4 reset.
//!
//! \param intFlags is the interrupt that needs to be set.
//!
//! The \e intFlags parameter are the Interrupts generated on reset of the CM
//! that need to be set. The values can be one/ more from:
//! SYSCTL_STATUS_CMGINT , SYSCTL_STATUS_CMNMIWDRST ,
//! SYSCTL_STATUS_CMSYSRESETREQ or SYSCTL_STATUS_CMVECTRESET.
//!
//! \return None.
//!
//! \note This API  is present only on CPU1.
//
//*****************************************************************************
static inline void
SysCtl_setCMInterruptStatus(uint32_t intFlags)
{
    EALLOW;
    HWREG(SYSSTAT_BASE + SYSCTL_O_CM_STATUS_INT_SET)  =
                (intFlags | (SYSCTL_REG_KEY & SYSCTL_CM_STATUS_INT_SET_KEY_M));
    EDIS;
}

//*****************************************************************************
//
//! Gets the masked interrupts due to multiple sources of Cortex-M4 reset.
//!
//! \return the Interrupt generated on reset of the CM.  The values can be
//! one/ more from:
//! SYSCTL_STATUS_CMGINT , SYSCTL_STATUS_CMNMIWDRST ,
//! SYSCTL_STATUS_CMSYSRESETREQ or SYSCTL_STATUS_CMVECTRESET.
//!
//! \note This API  is present only on CPU1.
//
//*****************************************************************************
static inline uint32_t
SysCtl_getCMInterruptStatusMask(void)
{
    return(HWREG(SYSSTAT_BASE + SYSCTL_O_CM_STATUS_MASK));
}

//*****************************************************************************
//
//! Sets the interrupt mask for the multiple sources of Cortex-M4 reset.
//!
//! \param intFlags is the interrupt that needs to be set.
//!
//! The \e intFlags parameter are the Interrupts generated on reset of the CM
//! that need to be masked. The values can be one/ more from:
//! SYSCTL_STATUS_CMGINT , SYSCTL_STATUS_CMNMIWDRST ,
//! SYSCTL_STATUS_CMSYSRESETREQ or SYSCTL_STATUS_CMVECTRESET.
//!
//! \return None.
//!
//! \note This API  is present only on CPU1.
//
//*****************************************************************************
static inline void
SysCtl_setCMInterruptStatusMask(uint32_t intFlags)
{
    EALLOW;
    HWREG(SYSSTAT_BASE + SYSCTL_O_CM_STATUS_MASK)  =
            (intFlags | (SYSCTL_REG_KEY & SYSCTL_CM_STATUS_MASK_KEY_M));
    EDIS;
}

//*****************************************************************************
//
//! Gets the status of interrupts due to multiple different
//! errors in the system.
//!
//! \return the Interrupt generated on the system.
//! The values can be one/ more from:
//! - SYSCTL_STATUS_EMIF_ERR
//! - SYSCTL_STATUS_RAM_CORRECTABLE_ERR
//! - SYSCTL_STATUS_FLASH_CORRECTABLE_ERR
//! - SYSCTL_STATUS_RAM_ACC_VIOL
//! - SYSCTL_STATUS_DCC0
//! - SYSCTL_STATUS_DCC1
//! - SYSCTL_STATUS_DCC2
//
//*****************************************************************************
static inline uint32_t
SysCtl_getInterruptStatus(void)
{
    return(HWREG(SYSSTAT_BASE + SYSCTL_O_SYS_ERR_INT_FLG));
}
//*****************************************************************************
//
//! Clears the interrupts due to multiple different errors in the system.
//!
//! \param intFlags is the interrupt that needs to be cleared.
//!
//! The \e intFlags parameter are the Interrupts generated on errors in
//! the system that need to be cleared. The values can be one or more from:
//! - SYSCTL_STATUS_GINT
//! - SYSCTL_STATUS_EMIF_ERR
//! - SYSCTL_STATUS_RAM_CORRECTABLE_ERR
//! - SYSCTL_STATUS_FLASH_CORRECTABLE_ERR
//! - SYSCTL_STATUS_RAM_ACC_VIOL
//! - SYSCTL_STATUS_DCC0
//! - SYSCTL_STATUS_DCC1
//! - SYSCTL_STATUS_DCC2
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearInterruptStatus(uint32_t intFlags)
{
    HWREGH(SYSSTAT_BASE + SYSCTL_O_SYS_ERR_INT_CLR) |= (uint16_t)intFlags;
}
//*****************************************************************************
//
//! Sets the interrupts for the multiple different errors in the system.
//!
//! \param intFlags is the interrupt that needs to be set.
//!
//! The \e intFlags parameter are the Interrupts that can be set
//! for the errors in the system. The values can be one/ more from:
//! - SYSCTL_STATUS_EMIF_ERR
//! - SYSCTL_STATUS_RAM_CORRECTABLE_ERR
//! - SYSCTL_STATUS_FLASH_CORRECTABLE_ERR
//! - SYSCTL_STATUS_RAM_ACC_VIOL
//! - SYSCTL_STATUS_DCC0
//! - SYSCTL_STATUS_DCC1
//! - SYSCTL_STATUS_DCC2
//!
//! \return None.
//!
//! \note This API  is present only on CPU1.
//
//*****************************************************************************
static inline void
SysCtl_setInterruptStatus(uint32_t intFlags)
{
    EALLOW;
    HWREG(SYSSTAT_BASE + SYSCTL_O_SYS_ERR_INT_SET) =
                (intFlags | (SYSCTL_REG_KEY & SYSCTL_SYS_ERR_INT_SET_KEY_M));
    EDIS;
}

//*****************************************************************************
//
//! Gets the masked interrupts due to multiple different
//! errors in the system.
//!
//! \return the Interrupt generated on the system.
//! The values can be one/ more from:
//! - SYSCTL_STATUS_EMIF_ERR
//! - SYSCTL_STATUS_RAM_CORRECTABLE_ERR
//! - SYSCTL_STATUS_FLASH_CORRECTABLE_ERR
//! - SYSCTL_STATUS_RAM_ACC_VIOL
//! - SYSCTL_STATUS_SYS_PLL_SLIP
//! - SYSCTL_STATUS_AUX_PLL_SLIP
//! - SYSCTL_STATUS_DCC0
//! - SYSCTL_STATUS_DCC1
//! - SYSCTL_STATUS_DCC2
//
//*****************************************************************************
static inline uint32_t
SysCtl_getInterruptStatusMask(void)
{
    return(HWREG(SYSSTAT_BASE + SYSCTL_O_SYS_ERR_MASK));
}

//*****************************************************************************
//
//! Masks the interrupts for the multiple different errors in the system.
//!
//! \param intFlags is the interrupt that needs to be set.
//!
//! The \e intFlags parameter are the Interrupts that can be masked
//! for the errors in the system. The values can be one/ more from:
//! - SYSCTL_STATUS_EMIF_ERR
//! - SYSCTL_STATUS_RAM_CORRECTABLE_ERR
//! - SYSCTL_STATUS_FLASH_CORRECTABLE_ERR
//! - SYSCTL_STATUS_RAM_ACC_VIOL
//! - SYSCTL_STATUS_SYS_PLL_SLIP
//! - SYSCTL_STATUS_AUX_PLL_SLIP
//! - SYSCTL_STATUS_DCC0
//! - SYSCTL_STATUS_DCC1
//! - SYSCTL_STATUS_DCC2
//!
//! \return None.
//!
//! \note This API  is present only on CPU1.
//
//*****************************************************************************
static inline void
SysCtl_setInterruptStatusMask(uint32_t intFlags)
{
    EALLOW;
    HWREG(SYSSTAT_BASE + SYSCTL_O_SYS_ERR_MASK) =
                (intFlags | (SYSCTL_REG_KEY & SYSCTL_SYS_ERR_MASK_KEY_M));
    EDIS;
}

//*****************************************************************************
//
//! Check if the Internal PHY is present or not for the USB module
//!
//! Provides the USB module Internal PHY presence
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return \b true if Internal USB PHY Module is present \b false if
//! Internal USB PHY Module is not present
//
//*****************************************************************************
static inline bool
SysCtl_isPresentUSBPHY(void)
{
    return((HWREG(DEVCFG_BASE + SYSCTL_O_PERCNF1) &
            SYSCTL_PERCNF1_USB_A_PHY) != 0U);
}

//*****************************************************************************
//
//! Sets up CLB CLK dividers & configurations for a particuler CLB.
//!
//! \param divider is the value that configures the clock divider.
//! \param tdivider is the value that configures the tile clock divider.
//! \param inst is the CLB instance that needs clock settings.
//! \param config is the mode for the clock
//!
//! This function sets up the CLB CLK configurations based on the instance
//! that is selected. There are 2 dividers that scales the "source" to CLB
//! CLK. The first one is the divider & the other the tile divider.
//!
//! The \e divider parameter can have one enumerated value from
//! SysCtl_CLBClkDivider
//! The \e tdivider parameter can have one enumerated value from
//! SysCtl_CLBTClkDivider
//! The \e inst parameter can have one enumerated value from
//! SysCtl_CLBInst
//! The \e config parameter can have one enumerated value from
//! SysCtl_CLBClkm
//!
//! \note See also \e SysCtl_setCLBClkDivider() and \e SysCtl_CLBClkConfig()
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setCLBClk (SysCtl_CLBClkDivider divider, SysCtl_CLBTClkDivider tdivider,
                  SysCtl_CLBInst inst, SysCtl_CLBClkm config)
{
    EALLOW;
    //
    //clear the CLB clk configurations
    //
    HWREG(CLKCFG_BASE + SYSCTL_O_CLBCLKCTL) =
                        (HWREG(CLKCFG_BASE + SYSCTL_O_CLBCLKCTL) &
                         ~(SYSCTL_CLBCLKCTL_CLBCLKDIV_M |
                           SYSCTL_CLBCLKCTL_TILECLKDIV |
                           (0x1UL << (uint16_t)inst)));

    //
    //set the clock configurations for the particular CLB
    //
    HWREG(CLKCFG_BASE + SYSCTL_O_CLBCLKCTL) |=
                ((uint32_t)divider << SYSCTL_CLBCLKCTL_CLBCLKDIV_S) |
                ((uint32_t)tdivider << SYSCTL_CLBCLKCTL_TILECLKDIV_S) |
                ((uint32_t)config  << (uint16_t)inst);
    EDIS;
}

//*****************************************************************************
//
//! Sets up CLB CLK dividers
//!
//! \param divider is the value that configures the clock divider.
//! \param tdivider is the value that configures the tile clock divider.
//!
//! This function sets up the CLB CLK dividers.
//! There are 2 dividers that scales the "source" to CLB CLK. The first one is
//! the divider & the other the tile divider.
//!
//! The \e divider parameter can have one enumerated value from
//! SysCtl_CLBClkDivider
//! The \e tdivider parameter can have one enumerated value from
//! SysCtl_CLBTClkDivider
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setCLBClkDivider(SysCtl_CLBClkDivider divider,
                        SysCtl_CLBTClkDivider tdivider)
{
    EALLOW;

    //
    // Clear the CLB clk configurations
    //
    HWREG(CLKCFG_BASE + SYSCTL_O_CLBCLKCTL) =
                        (HWREG(CLKCFG_BASE + SYSCTL_O_CLBCLKCTL) &
                         ~(SYSCTL_CLBCLKCTL_CLBCLKDIV_M |
                           SYSCTL_CLBCLKCTL_TILECLKDIV));

    //
    // Set the clock dividers
    //
    HWREG(CLKCFG_BASE + SYSCTL_O_CLBCLKCTL) |=
                ((uint32_t)divider << SYSCTL_CLBCLKCTL_CLBCLKDIV_S) |
                ((uint32_t)tdivider << SYSCTL_CLBCLKCTL_TILECLKDIV_S);
    EDIS;
}

//*****************************************************************************
//
//! Sets up CLB CLK configurations for a particuler CLB.
//!
//! \param inst is the CLB instance that needs clock settings.
//! \param config is the mode for the clock
//!
//! This function sets up the CLB CLK configurations based on the instance
//! that is selected.
//!
//! The \e inst parameter can have one enumerated value from
//! SysCtl_CLBInst
//! The \e config parameter can have one enumerated value from
//! SysCtl_CLBClkm
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_CLBClkConfig(SysCtl_CLBInst inst, SysCtl_CLBClkm config)
{
    EALLOW;

    //
    // Clear the CLB clk configurations
    //
    HWREG(CLKCFG_BASE + SYSCTL_O_CLBCLKCTL) =
                        (HWREG(CLKCFG_BASE + SYSCTL_O_CLBCLKCTL) &
                         ~(0x1UL << (uint16_t)inst));

    //
    // Set the clock configurations for the particular CLB
    //
    HWREG(CLKCFG_BASE + SYSCTL_O_CLBCLKCTL) |=
                        ((uint32_t)config  << (uint16_t)inst);
    EDIS;
}
//*****************************************************************************
//
//!  Check if One or more of the error sources triggered
//!
//!  Following are the events/triggers that can indicate an error:
//!1. nmi interrupt on C28x
//!2. Watchdog reset
//!3. Error on a Pie vector fetch
//!4. Efuse error
//!5. nmi interrupt on CM
//!
//! \return \b true if the error is triggered
//! \b false if the error is not triggered
//
//*****************************************************************************
static inline bool
SysCtl_isErrorTriggered(void)
{
    return((HWREGH(NMI_BASE + NMI_O_ERRORSTS) & NMI_ERRORSTS_ERROR) != 0U);
}

//*****************************************************************************
//
//!  Check if Error status pin is high or not
//!
//! \return \b true if the error status pin is high
//! \b false if the error status pin is low
//
//*****************************************************************************
static inline bool
SysCtl_getErrorPinStatus(void)
{
    //
    // Read and return the status of the ErrorPin
    //
    return((HWREGH(NMI_BASE + NMI_O_ERRORSTS) & NMI_ERRORSTS_PINSTS) != 0U);
}

//*****************************************************************************
//
//! Forces an error flag to set to indicate an error being generated.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_forceError(void)
{
    EALLOW;
    HWREGH(NMI_BASE + NMI_O_ERRORSTSFRC) |= NMI_ERRORSTSFRC_ERROR;
    EDIS;
}

//*****************************************************************************
//
//! Clears any error flag set due to error generated.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearError(void)
{
    EALLOW;
    HWREGH(NMI_BASE + NMI_O_ERRORSTSCLR) |= NMI_ERRORSTSCLR_ERROR;
    EDIS;
}

//*****************************************************************************
//
//! Selects the polarity of the error pin
//!
//! \param pol is the ERROR pin polarity
//!
//! The \e pol parameter can take any of the below values:
//! 0x0U: If an error is already triggered, Error pin will be driven
//! with a value of 0, else 1.
//! 0x1U: If an error is already triggered, Error pin will be driven
//! with a value of 1, else 0.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_selectErrPinPolarity(uint16_t pol)
{
    EALLOW;
    //
    //Configures the error pin polarity based on the provided polarity
    //
    if(pol == 1U)
    {
        HWREGH(NMI_BASE + NMI_O_ERRORCTL) |= NMI_ERRORCTL_ERRORPOLSEL;
    }
    else
    {
        HWREGH(NMI_BASE + NMI_O_ERRORCTL) &= ~NMI_ERRORCTL_ERRORPOLSEL;
    }
    EDIS;
}

//*****************************************************************************
//
//! Locks the Error control registers
//!
//! This function locks the Error configuration registers.
//!
//! \return None.
//!
//! \note The lock register is cleared only on a system reset.
//
//*****************************************************************************
static inline void
SysCtl_lockErrControl(void)
{
    EALLOW;
    HWREG(NMI_BASE + NMI_O_ERRORLOCK) = NMI_ERRORLOCK_ERRORCTL;
    EDIS;
}

//*****************************************************************************
//
//!  Check if the MCAN wakeup event has occured.
//!
//! \return \b true if the MCAN wakeup event has occured.
//!  \b false if the MCAN wakeup event has not occured.
//
//*****************************************************************************
static inline bool
SysCtl_isMCANWakeStatusSet(void)
{
    //
    // Read the MCAN wakeup event status and return true if set.
    //
    return((HWREGH(CPUSYS_BASE + SYSCTL_O_MCANWAKESTATUS) &
            SYSCTL_MCANWAKESTATUS_WAKE) != 0U);
}

//*****************************************************************************
//
//! Clears the MCAN wakeup event status.
//!
//! This function clears the MCAN wakeup event status. To check if it was set
//! first, see SysCtl_isMCANWakeStatusSet().
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearMCANWakeStatus(void)
{
    //
    // Clear the MCAN wakeup event status
    //
    HWREGH(CPUSYS_BASE + SYSCTL_O_MCANWAKESTATUSCLR) |=
                                        SYSCTL_MCANWAKESTATUSCLR_WAKE;
}

//*****************************************************************************
//
//! Delays for a fixed number of cycles.
//!
//! \param count is the number of delay loop iterations to perform.
//!
//! This function generates a constant length delay using assembly code. The
//! loop takes 5 cycles per iteration plus 9 cycles of overhead.
//!
//! \note If count is equal to zero, the loop will underflow and run for a
//! very long time.
//!
//! \note Refer to the macro DEVICE_DELAY_US(x) in device.h which can be used to
//! insert a delay in microseconds.
//!
//! \return None.
//
//*****************************************************************************
extern void
SysCtl_delay(uint32_t count);

//*****************************************************************************
//
//! Calculates the system clock frequency (SYSCLK).
//!
//! \param clockInHz is the frequency of the oscillator clock source (OSCCLK).
//!
//! This function determines the frequency of the system clock based on the
//! frequency of the oscillator clock source (from \e clockInHz) and the PLL
//! and clock divider configuration registers.
//!
//! \return Returns the system clock frequency. If a missing clock is detected,
//! the function will return the INTOSC1 frequency. This needs to be
//! corrected and cleared (see SysCtl_resetMCD()) before trying to call this
//! function again.
//
//*****************************************************************************
extern uint32_t
SysCtl_getClock(uint32_t clockInHz);

//*****************************************************************************
//
//! Calculates the system auxiliary clock frequency (AUXPLLCLK).
//!
//! \param clockInHz is the frequency of the oscillator clock source
//!  (AUXOSCCLK).
//!
//! This function determines the frequency of the auxiliary clock based on the
//! frequency of the oscillator clock source (from \e clockInHz) and the AUXPLL
//! and clock divider configuration registers.
//!
//! \return Returns the auxiliary clock frequency.
//
//*****************************************************************************
extern uint32_t
SysCtl_getAuxClock(uint32_t clockInHz);

//*****************************************************************************
//
//! Configures the clocking of the device.
//!
//! \param config is the required configuration of the device clocking.
//!
//! This function configures the clocking of the device.  The input crystal
//! frequency, oscillator to be used, use of the PLL, and the system clock
//! divider are all configured with this function.
//!
//! The \e config parameter is the OR of several different values, many of
//! which are grouped into sets where only one can be chosen.
//!
//! - The system clock divider is chosen with the macro \b SYSCTL_SYSDIV(x)
//!   where x is either 1 or an even value up to 126.
//!
//! - The use of the PLL is chosen with ONLY one of the below modes:
//!   \b SYSCTL_PLL_ENABLE - This is to Enable the PLL Clock to the System
//!    or
//!   \b SYSCTL_PLL_BYPASS -This is to Bypass the PLLCLK from the System,
//!    this will also power up the PLL if the user desires to power up the PLL
//!    but not use it for System.
//!    or
//!   \b SYSCTL_PLL_DISABLE-This is to Power Down the PLL and Bypass the
//!    PLLCLK to the System.
//!
//! - The integer multiplier is chosen \b SYSCTL_IMULT(x) where x is a value
//!   from 1 to 127.
//!
//! - The reference clock divider is chosen \b SYSCTL_REFDIV(x) where
//!    x is a value from 1 to 32.
//!
//! - The output clock divider is chosen \b SYSCTL_ODIV(x) where x
//!     is a value from 1 to 32.
//!
//! - The DCC module selected for checking PLL clock validity chosen with
//!    either \b SYSCTL_DCC_BASE_0, \b SYSCTL_DCC_BASE_1,
//!    or \b SYSCTL_DCC_BASE_2.
//!
//! - The oscillator source chosen with \b SYSCTL_OSCSRC_OSC2,
//!   \b SYSCTL_OSCSRC_XTAL, \b SYSCTL_OSCSRC_XTAL_SE or \b SYSCTL_OSCSRC_OSC1.
//!
//! This function uses the DCC to check that the PLLRAWCLK is running at the
//! expected rate. If you are using the DCC, you must back up its configuration
//! before calling this function and restore it afterward.
//! Locking PLL sequence is only done if the multipliers are updated.
//!
//! \note See your device errata for more details about locking the PLL.
//! Please note the PLL can take inputs from 2Mhz to 48Mhz.
//! PLL can be locked from 220Mhz to 800Mhz.
//! The output of PLL cannot be more than 500Mhz (after ODIV).
//!
//! \return Returns \b false if a missing clock error is detected. This needs
//! to be cleared (see SysCtl_resetMCD()) before trying to call this function
//! again. Also, returns \b false if the PLLRAWCLK is not running and its
//! expected rate. Otherwise, returns \b true.
//
//*****************************************************************************
extern bool
SysCtl_setClock(uint32_t config);

//*****************************************************************************
//
//! Validates PLL Raw Clock Frequency (PLLRAWCLK)
//!
//! \param base is the DCC module base address
//! \param oscSource is the Clock Source for the PLL that is also used for DCC
//! \param pllclk is the PLL Clock which has to be validated.
//! \param pllMultDiv has the PLL Multiplier Register configuration which
//!  include integer multiplier and divider values used to configure the
//!  DCC Counter1 clock
//!
//!  This function uses DCC module to validate the PLL clock frequency.
//!  It uses oscSource as a reference clock for DCC, and PLL is used as clock
//!  under test. As long as the Counter0 (running of oscSource) & Counter1
//!  (running of PLL) expire at the same time, DCC will not generate an Error.
//!  This function gives 100 attempts for PLL to lock and make sure frequency
//!  is as expected.
//!
//!  \note This function does not validate if PLL output frequency (PLLRAWCLK)
//!  is within the operating range as per the datasheet.
//!
//! - The \e oscSource parameter is the oscillator source chosen with
//!   \b SYSCTL_OSCSRC_OSC2, \b SYSCTL_OSCSRC_XTAL, \b SYSCTL_OSCSRC_XTAL_SE
//!    or \b SYSCTL_OSCSRC_OSC1.
//!
//! - The \e pllclk parameter can have one enumerated value from
//!   SysCtl_PLLClockSource
//! - The \e pllMultDiv parameter is a bitwise OR of \b SYSCTL_IMULT(x)
//!   where x is a value from 1 to 127 and both of the following divider
//!   values which is chosen with the macro \b SYSCTL_REFDIV(x) and
//!   SYSCTL_ODIV(x) where x is a value from 1 to 32 and can be different
//!   for both macros.
//!
//!  \return Returns \b true if the DCCSTATUS error flag is not set.
//!    Otherwise, returns \b false.
//
//*****************************************************************************
extern bool
SysCtl_isPLLValid(uint32_t base, uint32_t oscSource,
                  SysCtl_PLLClockSource pllclk, uint32_t pllMultDiv);
//*****************************************************************************
//
//! Configures the external oscillator for the clocking of the device.
//!
//! This function configures the external oscillator (XTAL) to be used for the
//! clocking of the device in crystal mode. It follows the procedure to turn on
//! the oscillator, wait for it to power up, and select it as the source of the
//! system clock.
//!
//! Please note that this function blocks while it waits for the XTAL to power
//! up. If the XTAL does not manage to power up properly, the function will
//! loop for a long time. It is recommended that you modify this function to
//! add an appropriate timeout and error-handling procedure.
//!
//! \return None.
//
//*****************************************************************************
extern void
SysCtl_selectXTAL(void);

//*****************************************************************************
//
//! Configures the external oscillator for the clocking of the device in
//! single-ended mode.
//!
//! This function configures the external oscillator (XTAL) to be used for the
//! clocking of the device in single-ended mode. It follows the procedure to
//! turn on the oscillator, wait for it to power up, and select it as the
//! source of the system clock.
//!
//! Please note that this function blocks while it waits for the XTAL to power
//! up. If the XTAL does not manage to power up properly, the function will
//! loop for a long time. It is recommended that you modify this function to
//! add an appropriate timeout and error-handling procedure.
//!
//! \return None.
//
//*****************************************************************************
extern void
SysCtl_selectXTALSingleEnded(void);

//*****************************************************************************
//
//! Selects the oscillator to be used for the clocking of the device.
//!
//! \param oscSource is the oscillator source to be configured.
//!
//! This function configures the oscillator to be used in the clocking of the
//! device. The \e oscSource parameter may take a value of
//! \b SYSCTL_OSCSRC_OSC2, \b SYSCTL_OSCSRC_XTAL, \b SYSCTL_OSCSRC_XTAL_SE, or
//! \b SYSCTL_OSCSRC_OSC1.
//!
//! \sa SysCtl_turnOnOsc()
//!
//! \return None.
//
//*****************************************************************************
extern void
SysCtl_selectOscSource(uint32_t oscSource);

//*****************************************************************************
//
//! Selects the oscillator to be used for the AUXPLL.
//!
//! \param oscSource is the oscillator source to be configured.
//!
//! This function configures the oscillator to be used in the clocking of the
//! AUXPLL. The \e oscSource parameter may take a value of
//! \b SYSCTL_OSCSRC_OSC2, \b SYSCTL_OSCSRC_XTAL, \b SYSCTL_OSCSRC_XTAL_SE, or
//! \b SYSCTL_OSCSRC_OSC1.
//!
//! \sa SysCtl_turnOnOsc()
//!
//! \return None.
//
//*****************************************************************************
extern void
SysCtl_selectOscSourceAuxPLL(uint32_t oscSource);

//*****************************************************************************
//
//! Calculates the low-speed peripheral clock frequency (LSPCLK).
//!
//! \param clockInHz is the frequency of the oscillator clock source (OSCCLK).
//!
//! This function determines the frequency of the low-speed peripheral clock
//! based on the frequency of the oscillator clock source (from \e clockInHz)
//! and the PLL and clock divider configuration registers.
//!
//! \return Returns the low-speed peripheral clock frequency.
//
//*****************************************************************************
extern uint32_t
SysCtl_getLowSpeedClock(uint32_t clockInHz);

//*****************************************************************************
//
//! Get the device part parametric value
//!
//! \param parametric is the requested device parametric value
//!
//! This function gets the device part parametric value.
//!
//! The \e parametric parameter can have one the following enumerated values:
//! - \b SYSCTL_DEVICE_QUAL      - Device Qualification Status
//! - \b SYSCTL_DEVICE_PINCOUNT  - Device Pin Count
//! - \b SYSCTL_DEVICE_INSTASPIN - Device InstaSPIN Feature Set
//! - \b SYSCTL_DEVICE_FLASH     - Device Flash size (KB)
//! - \b SYSCTL_DEVICE_PARTID    - Device Part ID Format Revision
//! - \b SYSCTL_DEVICE_FAMILY    - Device Family
//! - \b SYSCTL_DEVICE_PARTNO    - Device Part Number
//! - \b SYSCTL_DEVICE_CLASSID   - Device Class ID
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return Returns the specified parametric value.
//
//*****************************************************************************
extern uint16_t
SysCtl_getDeviceParametric(SysCtl_DeviceParametric parametric);

//*****************************************************************************
//
//! Configures the auxiliary PLL.
//!
//! \param config is the required configuration of the device clocking.
//!
//! This function configures the clock source for auxiliary PLL, the integer
//! multiplier, fractional multiplier and divider.
//!
//! The \e config parameter is the OR of several different values, many of
//! which are grouped into sets where only one can be chosen.
//!

//! - The system clock divider is chosen with one of the following macros:
//!    \b SYSCTL_AUXPLL_DIV_1,
//!    \b SYSCTL_AUXPLL_DIV_2,
//!    \b SYSCTL_AUXPLL_DIV_4,
//!    \b SYSCTL_AUXPLL_DIV_8
//!    \b SYSCTL_AUXPLL_DIV_3,
//!    \b SYSCTL_AUXPLL_DIV_5,
//!    \b SYSCTL_AUXPLL_DIV_6,
//!    \b SYSCTL_AUXPLL_DIV_7
//!
//! - The use of the PLL is chosen with ONLY one of the below modes:
//!   \b SYSCTL_AUXPLL_ENABLE - This is to Enable the PLL Clock to the System
//!    or
//!   \b SYSCTL_AUXPLL_BYPASS -This is to Bypass the PLLCLK from the System,
//!    this will also power up the PLL if the user desires to power up the PLL
//!    but not use it for System.
//!    or
//!   \b SYSCTL_AUXPLL_DISABLE-This is to Power Down the PLL and Bypass the
//!    PLLCLK to the System.
//!
//! - The integer multiplier is chosen with \b SYSCTL_AUXPLL_IMULT(x) where x
//!   is a value from 1 to 127.
//!
//! - The reference clock divider is chosen \b SYSCTL_AUXPLL_REFDIV((x) where
//!    x is a value from 1 to 32.
//!
//! - The output clock divider is chosen \b SYSCTL_AUXPLL_ODIV(x) where x
//!     is a value from 1 to 32.
//!
//! - The DCC module selected for checking PLL clock validity chosen with
//!    either \b SYSCTL_DCC_BASE_0, \b SYSCTL_DCC_BASE_1,
//!    or \b SYSCTL_DCC_BASE_2.
//!
//! - The oscillator source chosen with one of
//!     \b SYSCTL_AUXPLL_OSCSRC_OSC2,
//!     \b SYSCTL_AUXPLL_OSCSRC_XTAL,
//!     \b SYSCTL_AUXPLL_OSCSRC_AUXCLKIN,
//!     \b SYSCTL_AUXPLL_OSCSRC_XTAL_SE
//!
//! \note This function uses the DCC to check that the AUXPLLRAWCLK is
//! expected rate. If you are using the DCC, you must back up its configuration
//! before calling this function and restore it afterward.
//! Locking PLL sequence is only done if the multipliers are updated.
//!
//! Please note the PLL can take inputs from 2Mhz to 48Mhz.
//! PLL can be locked from 220Mhz to 800Mhz.
//! The output of PLL cannot be more than 500Mhz (after ODIV).
//!
//! \note See your device errata for more details about locking the PLL.
//!
//! \return None.
//
//*****************************************************************************
extern void
SysCtl_setAuxClock(uint32_t config);

//*****************************************************************************
//
//! Controls the reset of CPU2 by CPU1
//!
//! \param control is to deactivate / activate the reset to the CPU2.
//!
//! The \e control parameter can be a value from the enumeration
//! SysCtl_CoreReset
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return None.
//
//*****************************************************************************
extern void
SysCtl_controlCPU2Reset(SysCtl_CoreReset control);

//*****************************************************************************
//
//! Configures & locks/unlocks the peripheral type
//!
//! \param type is the peripheral type that needs to be configured.
//! \param config is the configuration done to the peripheral which is
//!  dependent on the peripheral type.
//! \param lock is to decide if writes for any further configuration is to
//!  be allowed or not.
//!
//! The \e type parameter can be a value from the enumeration
//! SysCtl_SelType
//! The \e config parameter can be a value from the ones below:
//! 0x0U : disables the feature for the type.
//! 0x1U : enables the feature for the type.
//!
//! For ECAP: ECAP registers are EALLOW protected or not.
//! For SDFM: Data Ready conditions do not generate the SDFMINT.
//! & Each filter generates a separate data ready interrupts.
//! For USB : Global interrupt feature is enabled or not
//! For MEMMAP: Enables remapping SDRAM in lower 64kb of address space or not.
//!
//! The \e lock parameter can be a value from the ones below:
//! 0x1U : Write for any further configuration is not allowed.
//! 0x0U : Write for any further configuration is allowed.
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return None.
//
//*****************************************************************************
extern void
SysCtl_configureType(SysCtl_SelType type , uint16_t config, uint16_t lock);

//*****************************************************************************
//
//!  Check if writes for any further configuration of peripheral types is to
//!  be allowed or not.
//!
//! \param type is the peripheral type for which permissions are being checked
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return \b true if Write for any further configuration is not allowed.
//! \b false if Write for any further configuration is allowed.
//
//*****************************************************************************
extern bool
SysCtl_isConfigTypeLocked(SysCtl_SelType type);

//*****************************************************************************
//
//! Sets the owner for clock configuration
//!
//! \param cpuInst is owner for the clock configuration.
//!
//! The \e cpuInst parameter can be a value from the enumeration
//! SysCtl_CPUSel
//!
//! \return None.
//
//*****************************************************************************

extern void
SysCtl_setSemOwner (SysCtl_CPUSel cpuInst);

//*****************************************************************************
//
//! Locks the Clock configuration registers
//!
//! \param registerName is clock configuration register which needs to
//! be locked.
//!
//! The \e registerName parameter can be a value from the enumeration
//! SysCtl_ClkRegSel
//!
//! \return None.
//!
//! \note The register is unlocked only on a system reset.
//
//*****************************************************************************

extern void
SysCtl_lockClkConfig(SysCtl_ClkRegSel registerName);

//*****************************************************************************
//
//! Locks the CPU system configuration registers
//!
//! \param registerName is CPU system configuration register which needs to
//! be locked.
//!
//! The \e registerName parameter can be a value from the enumeration
//! SysCtl_CpuRegSel
//!
//! \return None.
//!
//! \note The register is unlocked only on a system reset.
//
//*****************************************************************************

extern void
SysCtl_lockSysConfig (SysCtl_CpuRegSel registerName);

//*****************************************************************************
//
//! Controls the reset of CM
//!
//! \param control is to deactivate /activate the reset to the CM.
//!
//! The \e control parameter can be a value from the enumeration
//! SysCtl_CoreReset
//!
//! \return None.
//!
//! \note This API should activate reset to CM until CM is not in reset.
//! This API is applicable only for the CPU1 subsystem.
//!
//
//*****************************************************************************
extern void
SysCtl_controlCMReset(SysCtl_CoreReset control);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // SYSCTL_H
