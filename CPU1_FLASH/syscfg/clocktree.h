//#############################################################################
//
// FILE:   clockTree.h
//
// TITLE:  Setups device clocking for examples.
//
//#############################################################################
// $Copyright:
// Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com
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

#ifndef CLOCKTREE_H
#define CLOCKTREE_H

//*****************************************************************************
//
// Summary of SYSPLL related clock configuration
//
//*****************************************************************************
//
// Input Clock to SYSPLL (OSCCLK)	= 25 MHz    (X1 provides OSCCLK)
//
//##### SYSPLL ENABLED #####
//
// PLLRAWCLK				= 200 MHz   (Output of SYSPLL if enabled) 
// PLLSYSCLK				= 200 MHz
// CPU1CLK					= 200 MHz
// CPU2CLK					= 200 MHz 
// CPU1_SYSCLK				= 200 MHz
// CPU2_SYSCLK 				= 200 MHz
// LSPCLK					= 50 MHz 
	
//*****************************************************************************
//
// Macro definitions used in device.c (SYSPLL / LSPCLK)
//
//*****************************************************************************
//
//	Input Clock to SYSPLL (OSCCLK) = X1 = 25 MHz
//
#define DEVICE_OSCSRC_FREQ          25000000U
//
// Define to pass to SysCtl_setClock(). Will configure the clock as follows:
// SYSPLL ENABLED
// SYSCLK = 200 MHz = 25 MHz (OSCCLK) * 32 (IMULT) / (2 (REFDIV) * 2 (ODIV) * 1 (SYSCLKDIVSEL))
//
#define DEVICE_SETCLOCK_CFG         (SYSCTL_OSCSRC_XTAL  | SYSCTL_IMULT(32) | \
									 SYSCTL_REFDIV(2) | SYSCTL_ODIV(2) | \
									 SYSCTL_SYSDIV(1) | SYSCTL_PLL_ENABLE | \ 
									 SYSCTL_DCC_BASE_0)
									 
									 
#define DEVICE_SYSCLK_FREQ          (DEVICE_OSCSRC_FREQ * 32) / (2 * 2 * 1)

//
// Define to pass to SysCtl_setLowSpeedClock().
// Low Speed Clock (LSPCLK) = 200 MHz / 4 = 50 MHz
//
#define DEVICE_LSPCLK_CFG  			SYSCTL_LSPCLK_PRESCALE_4

#define DEVICE_LSPCLK_FREQ          (DEVICE_SYSCLK_FREQ / 4)


//*****************************************************************************
//
// Summary of AUXPLL related clock configuration
//
//*****************************************************************************
//
// Input Clock to AUXOSCCLK	= 25 MHz (X1 provides AUXOSCCLK)  
//
//##### AUXPLL ENABLED #####
//
// AUXPLLRAWCLK				= 125 MHz (Output of AUXPLL if enabled)
// AUXPLLCLK				= 125 MHz 
//
//*****************************************************************************
//
// Macro definitions used in device.c (AUXPLL)
//
//*****************************************************************************
//
//	Input Clock to AUXPLL (AUXOSCCLK) = X1 = 25 MHz
//
#define DEVICE_AUXOSCSRC_FREQ 		25000000U
//
// Define to pass to SysCtl_setAuxClock(). Will configure the clock as follows:
// AUXPLL ENABLED
// AUXPLLCLK = 125 MHz = 25 MHz (AUXOSCCLK) * 40 (IMULT) / (2 (REFDIV) * 4 (ODIV) * 1 (AUXCLKDIVSEL))
#define DEVICE_AUXCLK_FREQ          (DEVICE_OSCSRC_FREQ * 40) / (2 * 4 * 1)
//
#define DEVICE_AUXSETCLOCK_CFG      (SYSCTL_AUXPLL_OSCSRC_XTAL  | SYSCTL_AUXPLL_IMULT(40) | \
									 SYSCTL_REFDIV(2) | SYSCTL_ODIV(4)| \
									 SYSCTL_AUXPLL_DIV_1 | SYSCTL_AUXPLL_ENABLE | \
									 SYSCTL_DCC_BASE_0)
									 
									 

	
//*****************************************************************************
//
// CPU1CLK / CPU2CLK Domain (200 MHz)
//
//*****************************************************************************
// VCU
// TMU
// FPU
// Flash
// DCSM
// HWBIST
//	

//*****************************************************************************
//
// CPU1 SYSCLK Domain (200 MHz)
//
//*****************************************************************************
// CPUTIMERx
// DMA
// CLA1
// XINT
// PIE
// LSxRAMs
// MSGRAMs
// Mx/DxRAM
// BootROM
// BGCRC
// ERAD
// EMIF2
// WDCLK
//	

/////////////////////	
// Gated CPU1 SYSCLK
/////////////////////
// USB
//

//*****************************************************************************
//
// CPU2 SYSCLK Domain (200 MHz)
//
//*****************************************************************************
// CPUTIMERx
// DMA
// CLA1
// XINT
// PIE
// LSxRAMs
// MSGRAMs
// Mx/DxRAM
// BootROM
// BGCRC
// ERAD
//
	
/////////////////////	
// Gated CPU2 SYSCLK
/////////////////////
// CPU2_CLA1
// CPU2_DMA
// CPU2_Timer
//
//*****************************************************************************
//
// Gated Peripheral EPWM Domain ( MHz) 
//
//*****************************************************************************
// EPWM
// HRPWM
//
//*****************************************************************************
//
// Gated Peripheral SYSCLK Domain (200 MHz) 
//
//*****************************************************************************
// ADC
// CMPSS
// DAC
// ePWM
// HRPWM
// eCAP
// eQEP
// I2C
// McBSP
// SDFM
// FSI
// PMBUS
// HRCAL
// SPI
// SCI
// DCC
// CAN
//	
//*****************************************************************************
//
// Gated LSPCLK Domain (50 MHz) 
//
//*****************************************************************************
// SCI
// SPI
// McBSP

#endif // CLOCKTREE_H

