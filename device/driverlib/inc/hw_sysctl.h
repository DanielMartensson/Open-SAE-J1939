//###########################################################################
//
// FILE:    hw_sysctl.h
//
// TITLE:   Definitions for the SYSCTL registers.
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

#ifndef HW_SYSCTL_H
#define HW_SYSCTL_H

//*************************************************************************************************
//
// The following are defines for the SYSCTL register offsets
//
//*************************************************************************************************
#define SYSCTL_O_CLKSEM           0x0U    // Clock Control Semaphore Register
#define SYSCTL_O_CLKCFGLOCK1      0x2U    // Lock bit for CLKCFG registers
#define SYSCTL_O_CLKSRCCTL1       0x8U    // Clock Source Control register-1
#define SYSCTL_O_CLKSRCCTL2       0xAU    // Clock Source Control register-2
#define SYSCTL_O_CLKSRCCTL3       0xCU    // Clock Source Control register-3
#define SYSCTL_O_SYSPLLCTL1       0xEU    // SYSPLL Control register-1
#define SYSCTL_O_SYSPLLMULT       0x14U   // SYSPLL Multiplier register
#define SYSCTL_O_SYSPLLSTS        0x16U   // SYSPLL Status register
#define SYSCTL_O_AUXPLLCTL1       0x18U   // AUXPLL Control register-1
#define SYSCTL_O_AUXPLLMULT       0x1EU   // AUXPLL Multiplier register
#define SYSCTL_O_AUXPLLSTS        0x20U   // AUXPLL Status register
#define SYSCTL_O_SYSCLKDIVSEL     0x22U   // System Clock Divider Select register
#define SYSCTL_O_AUXCLKDIVSEL     0x24U   // Auxillary Clock Divider Select register
#define SYSCTL_O_PERCLKDIVSEL     0x26U   // Peripheral Clock Divider Selet register
#define SYSCTL_O_XCLKOUTDIVSEL    0x28U   // XCLKOUT Divider Select register
#define SYSCTL_O_CLBCLKCTL        0x2AU   // CLB Clocking Control Register
#define SYSCTL_O_LOSPCP           0x2CU   // Low Speed Clock Source Prescalar
#define SYSCTL_O_MCDCR            0x2EU   // Missing Clock Detect Control Register
#define SYSCTL_O_X1CNT            0x30U   // 10-bit Counter on X1 Clock
#define SYSCTL_O_XTALCR           0x32U   // XTAL Control Register
#define SYSCTL_O_ETHERCATCLKCTL   0x36U   // ETHERCATCLKCTL
#define SYSCTL_O_CMCLKCTL         0x38U   // CMCLKCTL

#define SYSCTL_O_CPUSYSLOCK1         0x0U    // Lock bit for CPUSYS registers
#define SYSCTL_O_CPUSYSLOCK2         0x2U    // Lock bit for CPUSYS registers
#define SYSCTL_O_PIEVERRADDR         0xAU    // PIE Vector Fetch Error Address register
#define SYSCTL_O_PCLKCR0             0x22U   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR1             0x24U   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR2             0x26U   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR3             0x28U   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR4             0x2AU   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR6             0x2EU   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR7             0x30U   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR8             0x32U   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR9             0x34U   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR10            0x36U   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR11            0x38U   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR13            0x3CU   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR14            0x3EU   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR16            0x42U   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR17            0x44U   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR18            0x46U   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR20            0x4AU   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR21            0x4CU   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR22            0x4EU   // Peripheral Clock Gating Registers
#define SYSCTL_O_PCLKCR23            0x50U   // Peripheral Clock Gating Registers
#define SYSCTL_O_SIMRESET            0x70U   // Simulated Reset Register
#define SYSCTL_O_LPMCR               0x76U   // LPM Control Register
#define SYSCTL_O_GPIOLPMSEL0         0x78U   // GPIO LPM Wakeup select registers
#define SYSCTL_O_GPIOLPMSEL1         0x7AU   // GPIO LPM Wakeup select registers
#define SYSCTL_O_TMR2CLKCTL          0x7CU   // Timer2 Clock Measurement functionality control
                                             // register
#define SYSCTL_O_RESCCLR             0x7EU   // Reset Cause Clear Register
#define SYSCTL_O_RESC                0x80U   // Reset Cause register
#define SYSCTL_O_MCANWAKESTATUS      0x98U   // MCAN Wake Status Register
#define SYSCTL_O_MCANWAKESTATUSCLR   0x9AU   // MCAN Wake Status Clear Register

#define SYSCTL_O_CLA1TASKSRCSELLOCK   0x0U    // CLA1 Task Trigger Source Select Lock Register
#define SYSCTL_O_DMACHSRCSELLOCK      0x4U    // DMA Channel Triger Source Select Lock Register
#define SYSCTL_O_CLA1TASKSRCSEL1      0x6U    // CLA1 Task Trigger Source Select Register-1
#define SYSCTL_O_CLA1TASKSRCSEL2      0x8U    // CLA1 Task Trigger Source Select Register-2
#define SYSCTL_O_DMACHSRCSEL1         0x16U   // DMA Channel Trigger Source Select Register-1
#define SYSCTL_O_DMACHSRCSEL2         0x18U   // DMA Channel Trigger Source Select Register-2

#define SYSCTL_O_DEVCFGLOCK1   0x0U     // Lock bit for DEVCFG registers
#define SYSCTL_O_PARTIDL       0x8U     // Lower 32-bit of Device PART Identification Number
#define SYSCTL_O_PARTIDH       0xAU     // Upper 32-bit of Device PART Identification Number
#define SYSCTL_O_REVID         0xCU     // Device Revision Number
#define SYSCTL_O_PERCNF1       0x60U    // Peripheral Configuration register
#define SYSCTL_O_FUSEERR       0x74U    // e-Fuse error Status register
#define SYSCTL_O_SOFTPRES0     0x82U    // Processing Block Software Reset register
#define SYSCTL_O_SOFTPRES1     0x84U    // EMIF Software Reset register
#define SYSCTL_O_SOFTPRES2     0x86U    // Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES3     0x88U    // Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES4     0x8AU    // Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES6     0x8EU    // Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES7     0x90U    // Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES8     0x92U    // Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES9     0x94U    // Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES10    0x96U    // Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES11    0x98U    // Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES13    0x9CU    // Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES14    0x9EU    // Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES16    0xA2U    // Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES17    0xA4U    // Reserved Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES18    0xA6U    // Reserved Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES20    0xAAU    // Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES21    0xACU    // Peripheral Software Reset register
#define SYSCTL_O_SOFTPRES23    0xB0U    // Peripheral Software Reset register
#define SYSCTL_O_CPUSEL0       0xD6U    // CPU Select register for common peripherals
#define SYSCTL_O_CPUSEL1       0xD8U    // CPU Select register for common peripherals
#define SYSCTL_O_CPUSEL2       0xDAU    // CPU Select register for common peripherals
#define SYSCTL_O_CPUSEL4       0xDEU    // CPU Select register for common peripherals
#define SYSCTL_O_CPUSEL5       0xE0U    // CPU Select register for common peripherals
#define SYSCTL_O_CPUSEL6       0xE2U    // CPU Select register for common peripherals
#define SYSCTL_O_CPUSEL7       0xE4U    // CPU Select register for common peripherals
#define SYSCTL_O_CPUSEL8       0xE6U    // CPU Select register for common peripherals
#define SYSCTL_O_CPUSEL9       0xE8U    // CPU Select register for common peripherals
#define SYSCTL_O_CPUSEL11      0xECU    // CPU Select register for common peripherals
#define SYSCTL_O_CPUSEL12      0xEEU    // CPU Select register for common peripherals
#define SYSCTL_O_CPUSEL14      0xF2U    // CPU Select register for common peripherals
#define SYSCTL_O_CPUSEL15      0xF4U    // CPU Select register for common peripherals
#define SYSCTL_O_CPUSEL16      0xF6U    // CPU Select register for common peripherals
#define SYSCTL_O_CPUSEL18      0xFAU    // CPU Select register for common peripherals
#define SYSCTL_O_CPUSEL25      0x108U   // CPU Select register for common peripherals
#define SYSCTL_O_CPU2RESCTL    0x122U   // CPU2 Reset Control Register
#define SYSCTL_O_RSTSTAT       0x124U   // Reset Status register for secondary C28x CPUs
#define SYSCTL_O_LPMSTAT       0x125U   // LPM Status Register for secondary C28x CPUs
#define SYSCTL_O_USBTYPE       0x19AU   // Configures USB Type for the device
#define SYSCTL_O_ECAPTYPE      0x19BU   // Configures ECAP Type for the device
#define SYSCTL_O_SDFMTYPE      0x19CU   // Configures SDFM Type for the device
#define SYSCTL_O_MEMMAPTYPE    0x19EU   // Configures Memory Map Type for the device

#define SYSCTL_O_ADCA_AC          0x0U     // ADCA Master Access Control Register
#define SYSCTL_O_ADCB_AC          0x2U     // ADCB Master Access Control Register
#define SYSCTL_O_ADCC_AC          0x4U     // ADCC Master Access Control Register
#define SYSCTL_O_ADCD_AC          0x6U     // ADCD Master Access Control Register
#define SYSCTL_O_CMPSS1_AC        0x10U    // CMPSS1 Master Access Control Register
#define SYSCTL_O_CMPSS2_AC        0x12U    // CMPSS2 Master Access Control Register
#define SYSCTL_O_CMPSS3_AC        0x14U    // CMPSS3 Master Access Control Register
#define SYSCTL_O_CMPSS4_AC        0x16U    // CMPSS4 Master Access Control Register
#define SYSCTL_O_CMPSS5_AC        0x18U    // CMPSS5 Master Access Control Register
#define SYSCTL_O_CMPSS6_AC        0x1AU    // CMPSS6 Master Access Control Register
#define SYSCTL_O_CMPSS7_AC        0x1CU    // CMPSS7 Master Access Control Register
#define SYSCTL_O_CMPSS8_AC        0x1EU    // CMPSS8 Master Access Control Register
#define SYSCTL_O_DACA_AC          0x28U    // DACA Master Access Control Register
#define SYSCTL_O_DACB_AC          0x2AU    // DACB Master Access Control Register
#define SYSCTL_O_DACC_AC          0x2CU    // DACC Master Access Control Register
#define SYSCTL_O_EPWM1_AC         0x48U    // EPWM1 Master Access Control Register
#define SYSCTL_O_EPWM2_AC         0x4AU    // EPWM2 Master Access Control Register
#define SYSCTL_O_EPWM3_AC         0x4CU    // EPWM3 Master Access Control Register
#define SYSCTL_O_EPWM4_AC         0x4EU    // EPWM4 Master Access Control Register
#define SYSCTL_O_EPWM5_AC         0x50U    // EPWM5 Master Access Control Register
#define SYSCTL_O_EPWM6_AC         0x52U    // EPWM6 Master Access Control Register
#define SYSCTL_O_EPWM7_AC         0x54U    // EPWM7 Master Access Control Register
#define SYSCTL_O_EPWM8_AC         0x56U    // EPWM8 Master Access Control Register
#define SYSCTL_O_EPWM9_AC         0x58U    // EPWM9 Master Access Control Register
#define SYSCTL_O_EPWM10_AC        0x5AU    // EPWM10 Master Access Control Register
#define SYSCTL_O_EPWM11_AC        0x5CU    // EPWM11 Master Access Control Register
#define SYSCTL_O_EPWM12_AC        0x5EU    // EPWM12 Master Access Control Register
#define SYSCTL_O_EPWM13_AC        0x60U    // EPWM13 Master Access Control Register
#define SYSCTL_O_EPWM14_AC        0x62U    // EPWM14 Master Access Control Register
#define SYSCTL_O_EPWM15_AC        0x64U    // EPWM15 Master Access Control Register
#define SYSCTL_O_EPWM16_AC        0x66U    // EPWM16 Master Access Control Register
#define SYSCTL_O_EQEP1_AC         0x70U    // EQEP1 Master Access Control Register
#define SYSCTL_O_EQEP2_AC         0x72U    // EQEP2 Master Access Control Register
#define SYSCTL_O_EQEP3_AC         0x74U    // EQEP3 Master Access Control Register
#define SYSCTL_O_ECAP1_AC         0x80U    // ECAP1 Master Access Control Register
#define SYSCTL_O_ECAP2_AC         0x82U    // ECAP2 Master Access Control Register
#define SYSCTL_O_ECAP3_AC         0x84U    // ECAP3 Master Access Control Register
#define SYSCTL_O_ECAP4_AC         0x86U    // ECAP4 Master Access Control Register
#define SYSCTL_O_ECAP5_AC         0x88U    // ECAP5 Master Access Control Register
#define SYSCTL_O_ECAP6_AC         0x8AU    // ECAP6 Master Access Control Register
#define SYSCTL_O_ECAP7_AC         0x8CU    // ECAP7 Master Access Control Register
#define SYSCTL_O_SDFM1_AC         0xA8U    // SDFM1 Master Access Control Register
#define SYSCTL_O_SDFM2_AC         0xAAU    // SDFM2 Master Access Control Register
#define SYSCTL_O_CLB1_AC          0xB0U    // CLB1 Master Access Control Register
#define SYSCTL_O_CLB2_AC          0xB2U    // CLB2 Master Access Control Register
#define SYSCTL_O_CLB3_AC          0xB4U    // CLB3 Master Access Control Register
#define SYSCTL_O_CLB4_AC          0xB6U    // CLB4 Master Access Control Register
#define SYSCTL_O_CLB5_AC          0xB8U    // CLB5 Master Access Control Register
#define SYSCTL_O_CLB6_AC          0xBAU    // CLB6 Master Access Control Register
#define SYSCTL_O_CLB7_AC          0xBCU    // CLB7 Master Access Control Register
#define SYSCTL_O_CLB8_AC          0xBEU    // CLB8 Master Access Control Register
#define SYSCTL_O_SPIA_AC          0x110U   // SPIA Master Access Control Register
#define SYSCTL_O_SPIB_AC          0x112U   // SPIB Master Access Control Register
#define SYSCTL_O_SPIC_AC          0x114U   // SPIC Master Access Control Register
#define SYSCTL_O_SPID_AC          0x116U   // SPID Master Access Control Register
#define SYSCTL_O_PMBUS_A_AC       0x130U   // PMBUSA Master Access Control Register
#define SYSCTL_O_CAN_A_AC         0x140U   // CAN_A Master Access Control Register
#define SYSCTL_O_CAN_B_AC         0x142U   // CAN_B Master Access Control Register
#define SYSCTL_O_MCBSPA_AC        0x150U   // MCBSPA Master Access Control Register
#define SYSCTL_O_MCBSPB_AC        0x152U   // MCBSPB Master Access Control Register
#define SYSCTL_O_USBA_AC          0x180U   // USBA Master Access Control Register
#define SYSCTL_O_HRPWM_AC         0x1A8U   // HRPWM Master Access Control Register
#define SYSCTL_O_ETHERCAT_AC      0x1AAU   // ETHERCAT Master Access Control Register
#define SYSCTL_O_FSIATX_AC        0x1B0U   // FSIATX Master Access Control Register
#define SYSCTL_O_FSIARX_AC        0x1B2U   // FSIARX Master Access Control Register
#define SYSCTL_O_FSIBTX_AC        0x1B4U   // FSIBTX Master Access Control Register
#define SYSCTL_O_FSIBRX_AC        0x1B6U   // FSIBRX Master Access Control Register
#define SYSCTL_O_FSICRX_AC        0x1BAU   // FSICRX Master Access Control Register
#define SYSCTL_O_FSIDRX_AC        0x1BEU   // FSIDRX Master Access Control Register
#define SYSCTL_O_FSIERX_AC        0x1C2U   // FSIERX Master Access Control Register
#define SYSCTL_O_FSIFRX_AC        0x1C6U   // FSIFRX Master Access Control Register
#define SYSCTL_O_FSIGRX_AC        0x1CAU   // FSIGRX Master Access Control Register
#define SYSCTL_O_FSIHRX_AC        0x1CEU   // FSIHRX Master Access Control Register
#define SYSCTL_O_MCANA_AC         0x1D0U   // MCANA Master Access Control Register
#define SYSCTL_O_PERIPH_AC_LOCK   0x1FEU   // Lock Register to stop Write access to peripheral
                                           // Access register.

#define SYSCTL_O_CMRESCTL            0x0U     // CM Reset Control Register
#define SYSCTL_O_CMTOCPU1NMICTL      0x2U     // CM To CPU1 NMI Control register
#define SYSCTL_O_CMTOCPU1INTCTL      0x4U     // CM To CPU1 interrupt Control register
#define SYSCTL_O_PALLOCATE0          0x20U    // CM Peripheral Allocation Register.
#define SYSCTL_O_CM_CONF_REGS_LOCK   0x3FEU   // CM Configuration Registers Lock

#define SYSCTL_O_CM_STATUS_INT_FLG   0x0U    // Status of interrupts due to multiple sources of
                                             // Cortex-M4 reset.
#define SYSCTL_O_CM_STATUS_INT_CLR   0x2U    // CM_STATUS_INT_FLG clear register
#define SYSCTL_O_CM_STATUS_INT_SET   0x4U    // CM_STATUS_INT_FLG set register
#define SYSCTL_O_CM_STATUS_MASK      0x6U    // CM_STATUS_MASK register
#define SYSCTL_O_SYS_ERR_INT_FLG     0x10U   // Status of interrupts due to multiple different
                                             // errors in the system.
#define SYSCTL_O_SYS_ERR_INT_CLR     0x12U   // SYS_ERR_INT_FLG clear register
#define SYSCTL_O_SYS_ERR_INT_SET     0x14U   // SYS_ERR_INT_FLG set register
#define SYSCTL_O_SYS_ERR_MASK        0x16U   // SYS_ERR_MASK register

#define SYSCTL_O_SYNCSELECT        0x0U   // Sync Input and Output Select Register
#define SYSCTL_O_ADCSOCOUTSELECT   0x2U   // External ADC (Off Chip) SOC Select Register
#define SYSCTL_O_SYNCSOCLOCK       0x4U   // SYNCSEL and EXTADCSOC Select Lock register


//*************************************************************************************************
//
// The following are defines for the bit fields in the CLKSEM register
//
//*************************************************************************************************
#define SYSCTL_CLKSEM_SEM_S   0U
#define SYSCTL_CLKSEM_SEM_M   0x3U          // Semaphore for CLKCFG Ownership by CPU1 or CPU2
#define SYSCTL_CLKSEM_KEY_S   16U
#define SYSCTL_CLKSEM_KEY_M   0xFFFF0000U   // Key Qualifier for writes to this register

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLKCFGLOCK1 register
//
//*************************************************************************************************
#define SYSCTL_CLKCFGLOCK1_CLKSRCCTL1       0x1U       // Lock bit for CLKSRCCTL1 register
#define SYSCTL_CLKCFGLOCK1_CLKSRCCTL2       0x2U       // Lock bit for CLKSRCCTL2 register
#define SYSCTL_CLKCFGLOCK1_CLKSRCCTL3       0x4U       // Lock bit for CLKSRCCTL3 register
#define SYSCTL_CLKCFGLOCK1_SYSPLLCTL1       0x8U       // Lock bit for SYSPLLCTL1 register
#define SYSCTL_CLKCFGLOCK1_SYSPLLCTL2       0x10U      // Lock bit for SYSPLLCTL2 register
#define SYSCTL_CLKCFGLOCK1_SYSPLLCTL3       0x20U      // Lock bit for SYSPLLCTL3 register
#define SYSCTL_CLKCFGLOCK1_SYSPLLMULT       0x40U      // Lock bit for SYSPLLMULT register
#define SYSCTL_CLKCFGLOCK1_AUXPLLCTL1       0x80U      // Lock bit for AUXPLLCTL1 register
#define SYSCTL_CLKCFGLOCK1_AUXPLLMULT       0x400U     // Lock bit for AUXPLLMULT register
#define SYSCTL_CLKCFGLOCK1_SYSCLKDIVSEL     0x800U     // Lock bit for SYSCLKDIVSEL register
#define SYSCTL_CLKCFGLOCK1_AUXCLKDIVSEL     0x1000U    // Lock bit for AUXCLKDIVSEL register
#define SYSCTL_CLKCFGLOCK1_PERCLKDIVSEL     0x2000U    // Lock bit for PERCLKDIVSEL register
#define SYSCTL_CLKCFGLOCK1_CLBCLKCTL        0x4000U    // Lock bit for CLBCLKCTL register
#define SYSCTL_CLKCFGLOCK1_LOSPCP           0x8000U    // Lock bit for LOSPCP register
#define SYSCTL_CLKCFGLOCK1_XTALCR           0x10000U   // Lock bit for XTALCR register
#define SYSCTL_CLKCFGLOCK1_ETHERCATCLKCTL   0x20000U   // Lock bit for ETHERCATCLKCTL register
#define SYSCTL_CLKCFGLOCK1_CMCLKCTL         0x40000U   // Lock bit for CMCLKCTL register

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLKSRCCTL1 register
//
//*************************************************************************************************
#define SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_S            0U
#define SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M            0x3U    // OSCCLK Source Select Bit
#define SYSCTL_CLKSRCCTL1_INTOSC2OFF_NOTSUPPORTED   0x8U    // Internal Oscillator 2 Off Bit
#define SYSCTL_CLKSRCCTL1_XTALOFF                   0x10U   // Crystal (External) Oscillator Off
                                                            // Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLKSRCCTL2 register
//
//*************************************************************************************************
#define SYSCTL_CLKSRCCTL2_AUXOSCCLKSRCSEL_S   0U
#define SYSCTL_CLKSRCCTL2_AUXOSCCLKSRCSEL_M   0x3U     // AUXOSCCLK Source Select Bit
#define SYSCTL_CLKSRCCTL2_CANABCLKSEL_S       2U
#define SYSCTL_CLKSRCCTL2_CANABCLKSEL_M       0xCU     // CANA Clock Source Select Bit
#define SYSCTL_CLKSRCCTL2_CANBBCLKSEL_S       4U
#define SYSCTL_CLKSRCCTL2_CANBBCLKSEL_M       0x30U    // CANB Clock Source Select Bit
#define SYSCTL_CLKSRCCTL2_MCANABITCLKSEL_S    10U
#define SYSCTL_CLKSRCCTL2_MCANABITCLKSEL_M    0xC00U   // MCAN (global) Bit-Clock Source Select Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLKSRCCTL3 register
//
//*************************************************************************************************
#define SYSCTL_CLKSRCCTL3_XCLKOUTSEL_S   0U
#define SYSCTL_CLKSRCCTL3_XCLKOUTSEL_M   0xFU   // XCLKOUT Source Select Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SYSPLLCTL1 register
//
//*************************************************************************************************
#define SYSCTL_SYSPLLCTL1_PLLEN      0x1U   // SYSPLL enable/disable bit
#define SYSCTL_SYSPLLCTL1_PLLCLKEN   0x2U   // SYSPLL bypassed or included in the PLLSYSCLK path

//*************************************************************************************************
//
// The following are defines for the bit fields in the SYSPLLMULT register
//
//*************************************************************************************************
#define SYSCTL_SYSPLLMULT_IMULT_S    0U
#define SYSCTL_SYSPLLMULT_IMULT_M    0xFFU         // SYSPLL Integer Multiplier
#define SYSCTL_SYSPLLMULT_ODIV_S     16U
#define SYSCTL_SYSPLLMULT_ODIV_M     0x1F0000U     // Output Clock Divider
#define SYSCTL_SYSPLLMULT_REFDIV_S   24U
#define SYSCTL_SYSPLLMULT_REFDIV_M   0x1F000000U   // Reference Clock Divider

//*************************************************************************************************
//
// The following are defines for the bit fields in the SYSPLLSTS register
//
//*************************************************************************************************
#define SYSCTL_SYSPLLSTS_LOCKS                0x1U   // SYSPLL Lock Status Bit
#define SYSCTL_SYSPLLSTS_SLIPS_NOTSUPPORTED   0x2U   // SYSPLL Slip Status Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the AUXPLLCTL1 register
//
//*************************************************************************************************
#define SYSCTL_AUXPLLCTL1_PLLEN      0x1U   // AUXPLL enable/disable bit
#define SYSCTL_AUXPLLCTL1_PLLCLKEN   0x2U   // AUXPLL bypassed or included in the AUXPLLCLK path

//*************************************************************************************************
//
// The following are defines for the bit fields in the AUXPLLMULT register
//
//*************************************************************************************************
#define SYSCTL_AUXPLLMULT_IMULT_S    0U
#define SYSCTL_AUXPLLMULT_IMULT_M    0xFFU         // AUXPLL Integer Multiplier
#define SYSCTL_AUXPLLMULT_ODIV_S     16U
#define SYSCTL_AUXPLLMULT_ODIV_M     0x1F0000U     // Output Clock Divider
#define SYSCTL_AUXPLLMULT_REFDIV_S   24U
#define SYSCTL_AUXPLLMULT_REFDIV_M   0x1F000000U   // Reference Clock Divider

//*************************************************************************************************
//
// The following are defines for the bit fields in the AUXPLLSTS register
//
//*************************************************************************************************
#define SYSCTL_AUXPLLSTS_LOCKS                0x1U   // AUXPLL Lock Status Bit
#define SYSCTL_AUXPLLSTS_SLIPS_NOTSUPPORTED   0x2U   // AUXPLL Slip Status Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SYSCLKDIVSEL register
//
//*************************************************************************************************
#define SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_S   0U
#define SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M   0x3FU   // PLLSYSCLK Divide Select

//*************************************************************************************************
//
// The following are defines for the bit fields in the AUXCLKDIVSEL register
//
//*************************************************************************************************
#define SYSCTL_AUXCLKDIVSEL_AUXPLLDIV_S    0U
#define SYSCTL_AUXCLKDIVSEL_AUXPLLDIV_M    0x7U      // AUXPLLCLK Divide Select
#define SYSCTL_AUXCLKDIVSEL_MCANCLKDIV_S   8U
#define SYSCTL_AUXCLKDIVSEL_MCANCLKDIV_M   0x1F00U   // Divider between CANFD Source Clock and
                                                     // CANFD Bit CLK

//*************************************************************************************************
//
// The following are defines for the bit fields in the PERCLKDIVSEL register
//
//*************************************************************************************************
#define SYSCTL_PERCLKDIVSEL_EPWMCLKDIV_S   0U
#define SYSCTL_PERCLKDIVSEL_EPWMCLKDIV_M   0x3U    // EPWM Clock Divide Select
#define SYSCTL_PERCLKDIVSEL_EMIF1CLKDIV    0x10U   // EMIF1  Clock Divide Select
#define SYSCTL_PERCLKDIVSEL_EMIF2CLKDIV    0x40U   // EMIF2 Clock Divide Select

//*************************************************************************************************
//
// The following are defines for the bit fields in the XCLKOUTDIVSEL register
//
//*************************************************************************************************
#define SYSCTL_XCLKOUTDIVSEL_XCLKOUTDIV_S   0U
#define SYSCTL_XCLKOUTDIVSEL_XCLKOUTDIV_M   0x3U   // XCLKOUT Divide Select

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLBCLKCTL register
//
//*************************************************************************************************
#define SYSCTL_CLBCLKCTL_CLBCLKDIV_S   0U
#define SYSCTL_CLBCLKCTL_CLBCLKDIV_M   0x7U        // CLB clock divider configuration.
#define SYSCTL_CLBCLKCTL_TILECLKDIV    0x10U       // CLB Tile clock divider configuration.
#define SYSCTL_CLBCLKCTL_CLKMODECLB1   0x10000U    // Clock mode of CLB1
#define SYSCTL_CLBCLKCTL_CLKMODECLB2   0x20000U    // Clock mode of CLB2
#define SYSCTL_CLBCLKCTL_CLKMODECLB3   0x40000U    // Clock mode of CLB3
#define SYSCTL_CLBCLKCTL_CLKMODECLB4   0x80000U    // Clock mode of CLB4
#define SYSCTL_CLBCLKCTL_CLKMODECLB5   0x100000U   // Clock mode of CLB5
#define SYSCTL_CLBCLKCTL_CLKMODECLB6   0x200000U   // Clock mode of CLB6
#define SYSCTL_CLBCLKCTL_CLKMODECLB7   0x400000U   // Clock mode of CLB7
#define SYSCTL_CLBCLKCTL_CLKMODECLB8   0x800000U   // Clock mode of CLB8

//*************************************************************************************************
//
// The following are defines for the bit fields in the LOSPCP register
//
//*************************************************************************************************
#define SYSCTL_LOSPCP_LSPCLKDIV_S   0U
#define SYSCTL_LOSPCP_LSPCLKDIV_M   0x7U   // LSPCLK Divide Select

//*************************************************************************************************
//
// The following are defines for the bit fields in the MCDCR register
//
//*************************************************************************************************
#define SYSCTL_MCDCR_MCLKSTS   0x1U   // Missing Clock Status Bit
#define SYSCTL_MCDCR_MCLKCLR   0x2U   // Missing Clock Clear Bit
#define SYSCTL_MCDCR_MCLKOFF   0x4U   // Missing Clock Detect Off Bit
#define SYSCTL_MCDCR_OSCOFF    0x8U   // Oscillator Clock Off Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the X1CNT register
//
//*************************************************************************************************
#define SYSCTL_X1CNT_X1CNT_S   0U
#define SYSCTL_X1CNT_X1CNT_M   0x3FFU     // X1 Counter
#define SYSCTL_X1CNT_CLR       0x10000U   // X1 Counter Clear

//*************************************************************************************************
//
// The following are defines for the bit fields in the XTALCR register
//
//*************************************************************************************************
#define SYSCTL_XTALCR_OSCOFF   0x1U   // XTAL Oscillator powered-down
#define SYSCTL_XTALCR_SE       0x2U   // XTAL Oscilator in Single-Ended mode

//*************************************************************************************************
//
// The following are defines for the bit fields in the ETHERCATCLKCTL register
//
//*************************************************************************************************
#define SYSCTL_ETHERCATCLKCTL_DIVSRCSEL   0x1U     // Clock source select for the etherCAT clock
                                                   // divider.
#define SYSCTL_ETHERCATCLKCTL_ECATDIV_S   1U
#define SYSCTL_ETHERCATCLKCTL_ECATDIV_M   0xEU     // etherCAT clock divider configuration.
#define SYSCTL_ETHERCATCLKCTL_PHYCLKEN    0x100U   // etherCAT PHY clock enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMCLKCTL register
//
//*************************************************************************************************
#define SYSCTL_CMCLKCTL_CMDIVSRCSEL    0x1U    // Clock source select for the CM clock divider.
#define SYSCTL_CMCLKCTL_CMCLKDIV_S     1U
#define SYSCTL_CMCLKCTL_CMCLKDIV_M     0xEU    // CM clock divider configuration.
#define SYSCTL_CMCLKCTL_ETHDIVSRCSEL   0x10U   // Clock source select for the etherNET clock
                                               // divider.
#define SYSCTL_CMCLKCTL_ETHDIV_S       5U
#define SYSCTL_CMCLKCTL_ETHDIV_M       0xE0U   // Ethernet clock divider configuration


//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSYSLOCK1 register
//
//*************************************************************************************************
#define SYSCTL_CPUSYSLOCK1_PIEVERRADDR   0x4U          // Lock bit for PIEVERRADDR Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR0       0x8U          // Lock bit for PCLKCR0 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR1       0x10U         // Lock bit for PCLKCR1 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR2       0x20U         // Lock bit for PCLKCR2 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR3       0x40U         // Lock bit for PCLKCR3 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR4       0x80U         // Lock bit for PCLKCR4 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR6       0x200U        // Lock bit for PCLKCR6 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR7       0x400U        // Lock bit for PCLKCR7 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR8       0x800U        // Lock bit for PCLKCR8 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR9       0x1000U       // Lock bit for PCLKCR9 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR10      0x2000U       // Lock bit for PCLKCR10 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR11      0x4000U       // Lock bit for PCLKCR11 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR13      0x10000U      // Lock bit for PCLKCR13 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR14      0x20000U      // Lock bit for PCLKCR14 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR16      0x80000U      // Lock bit for PCLKCR16 Register
#define SYSCTL_CPUSYSLOCK1_LPMCR         0x200000U     // Lock bit for LPMCR Register
#define SYSCTL_CPUSYSLOCK1_GPIOLPMSEL0   0x400000U     // Lock bit for GPIOLPMSEL0 Register
#define SYSCTL_CPUSYSLOCK1_GPIOLPMSEL1   0x800000U     // Lock bit for GPIOLPMSEL1 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR17      0x1000000U    // Lock bit for PCLKCR17 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR18      0x2000000U    // Lock bit for PCLKCR18 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR20      0x8000000U    // Lock bit for PCLKCR20 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR21      0x10000000U   // Lock bit for PCLKCR21 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR22      0x20000000U   // Lock bit for PCLKCR22 Register
#define SYSCTL_CPUSYSLOCK1_PCLKCR23      0x40000000U   // Lock bit for PCLKCR23 Register

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSYSLOCK2 register
//
//*************************************************************************************************
#define SYSCTL_CPUSYSLOCK2_ETHERCATCTL   0x1U   // Lock bit for ETHERCATCTL register

//*************************************************************************************************
//
// The following are defines for the bit fields in the PIEVERRADDR register
//
//*************************************************************************************************
#define SYSCTL_PIEVERRADDR_ADDR_S   0U
#define SYSCTL_PIEVERRADDR_ADDR_M   0x3FFFFFU   // PIE Vector Fetch Error Handler Routine Address

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR0 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR0_CLA1         0x1U         // CLA1 Clock Enable Bit
#define SYSCTL_PCLKCR0_DMA          0x4U         // DMA Clock Enable bit
#define SYSCTL_PCLKCR0_CPUTIMER0    0x8U         // CPUTIMER0 Clock Enable bit
#define SYSCTL_PCLKCR0_CPUTIMER1    0x10U        // CPUTIMER1 Clock Enable bit
#define SYSCTL_PCLKCR0_CPUTIMER2    0x20U        // CPUTIMER2 Clock Enable bit
#define SYSCTL_PCLKCR0_CPUBGCRC     0x2000U      // CPUBGCRC Clock Enable Bit
#define SYSCTL_PCLKCR0_CLA1BGCRC    0x4000U      // CLA1BGCRC Clock Enable Bit
#define SYSCTL_PCLKCR0_HRCAL        0x10000U     // HRCAL Clock Enable Bit
#define SYSCTL_PCLKCR0_TBCLKSYNC    0x40000U     // EPWM Time Base Clock sync
#define SYSCTL_PCLKCR0_GTBCLKSYNC   0x80000U     // EPWM Time Base Clock Global sync
#define SYSCTL_PCLKCR0_ERAD         0x1000000U   // ERAD module clock enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR1 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR1_EMIF1   0x1U   // EMIF1 Clock Enable bit
#define SYSCTL_PCLKCR1_EMIF2   0x2U   // EMIF2 Clock Enable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR2 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR2_EPWM1    0x1U      // EPWM1 Clock Enable bit
#define SYSCTL_PCLKCR2_EPWM2    0x2U      // EPWM2 Clock Enable bit
#define SYSCTL_PCLKCR2_EPWM3    0x4U      // EPWM3 Clock Enable bit
#define SYSCTL_PCLKCR2_EPWM4    0x8U      // EPWM4 Clock Enable bit
#define SYSCTL_PCLKCR2_EPWM5    0x10U     // EPWM5 Clock Enable bit
#define SYSCTL_PCLKCR2_EPWM6    0x20U     // EPWM6 Clock Enable bit
#define SYSCTL_PCLKCR2_EPWM7    0x40U     // EPWM7 Clock Enable bit
#define SYSCTL_PCLKCR2_EPWM8    0x80U     // EPWM8 Clock Enable bit
#define SYSCTL_PCLKCR2_EPWM9    0x100U    // EPWM9 Clock Enable bit
#define SYSCTL_PCLKCR2_EPWM10   0x200U    // EPWM10 Clock Enable bit
#define SYSCTL_PCLKCR2_EPWM11   0x400U    // EPWM11 Clock Enable bit
#define SYSCTL_PCLKCR2_EPWM12   0x800U    // EPWM12 Clock Enable bit
#define SYSCTL_PCLKCR2_EPWM13   0x1000U   // EPWM13 Clock Enable bit
#define SYSCTL_PCLKCR2_EPWM14   0x2000U   // EPWM14 Clock Enable bit
#define SYSCTL_PCLKCR2_EPWM15   0x4000U   // EPWM15 Clock Enable bit
#define SYSCTL_PCLKCR2_EPWM16   0x8000U   // EPWM16 Clock Enable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR3 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR3_ECAP1   0x1U    // ECAP1 Clock Enable bit
#define SYSCTL_PCLKCR3_ECAP2   0x2U    // ECAP2 Clock Enable bit
#define SYSCTL_PCLKCR3_ECAP3   0x4U    // ECAP3 Clock Enable bit
#define SYSCTL_PCLKCR3_ECAP4   0x8U    // ECAP4 Clock Enable bit
#define SYSCTL_PCLKCR3_ECAP5   0x10U   // ECAP5 Clock Enable bit
#define SYSCTL_PCLKCR3_ECAP6   0x20U   // ECAP6 Clock Enable bit
#define SYSCTL_PCLKCR3_ECAP7   0x40U   // ECAP7 Clock Enable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR4 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR4_EQEP1   0x1U   // EQEP1 Clock Enable bit
#define SYSCTL_PCLKCR4_EQEP2   0x2U   // EQEP2 Clock Enable bit
#define SYSCTL_PCLKCR4_EQEP3   0x4U   // EQEP3 Clock Enable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR6 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR6_SD1   0x1U   // SD1 Clock Enable bit
#define SYSCTL_PCLKCR6_SD2   0x2U   // SD2 Clock Enable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR7 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR7_SCI_A   0x1U   // SCI_A Clock Enable bit
#define SYSCTL_PCLKCR7_SCI_B   0x2U   // SCI_B Clock Enable bit
#define SYSCTL_PCLKCR7_SCI_C   0x4U   // SCI_C Clock Enable bit
#define SYSCTL_PCLKCR7_SCI_D   0x8U   // SCI_D Clock Enable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR8 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR8_SPI_A   0x1U   // SPI_A Clock Enable bit
#define SYSCTL_PCLKCR8_SPI_B   0x2U   // SPI_B Clock Enable bit
#define SYSCTL_PCLKCR8_SPI_C   0x4U   // SPI_C Clock Enable bit
#define SYSCTL_PCLKCR8_SPI_D   0x8U   // SPI_D Clock Enable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR9 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR9_I2C_A   0x1U   // I2C_A Clock Enable bit
#define SYSCTL_PCLKCR9_I2C_B   0x2U   // I2C_B Clock Enable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR10 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR10_CAN_A    0x1U    // CAN_A Clock Enable bit
#define SYSCTL_PCLKCR10_CAN_B    0x2U    // CAN_B Clock Enable bit
#define SYSCTL_PCLKCR10_MCAN_A   0x10U   // MCAN_A Clock Enable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR11 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR11_MCBSP_A   0x1U       // McBSP_A Clock Enable bit
#define SYSCTL_PCLKCR11_MCBSP_B   0x2U       // McBSP_B Clock Enable bit
#define SYSCTL_PCLKCR11_USB_A     0x10000U   // USB_A Clock Enable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR13 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR13_ADC_A   0x1U   // ADC_A Clock Enable bit
#define SYSCTL_PCLKCR13_ADC_B   0x2U   // ADC_B Clock Enable bit
#define SYSCTL_PCLKCR13_ADC_C   0x4U   // ADC_C Clock Enable bit
#define SYSCTL_PCLKCR13_ADC_D   0x8U   // ADC_D Clock Enable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR14 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR14_CMPSS1   0x1U    // CMPSS1 Clock Enable bit
#define SYSCTL_PCLKCR14_CMPSS2   0x2U    // CMPSS2 Clock Enable bit
#define SYSCTL_PCLKCR14_CMPSS3   0x4U    // CMPSS3 Clock Enable bit
#define SYSCTL_PCLKCR14_CMPSS4   0x8U    // CMPSS4 Clock Enable bit
#define SYSCTL_PCLKCR14_CMPSS5   0x10U   // CMPSS5 Clock Enable bit
#define SYSCTL_PCLKCR14_CMPSS6   0x20U   // CMPSS6 Clock Enable bit
#define SYSCTL_PCLKCR14_CMPSS7   0x40U   // CMPSS7 Clock Enable bit
#define SYSCTL_PCLKCR14_CMPSS8   0x80U   // CMPSS8 Clock Enable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR16 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR16_DAC_A   0x10000U   // Buffered_DAC12_1 Clock Enable Bit
#define SYSCTL_PCLKCR16_DAC_B   0x20000U   // Buffered_DAC12_2 Clock Enable Bit
#define SYSCTL_PCLKCR16_DAC_C   0x40000U   // Buffered_DAC12_3 Clock Enable Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR17 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR17_CLB1   0x1U    // CLB1 Clock Enable bit
#define SYSCTL_PCLKCR17_CLB2   0x2U    // CLB2 Clock Enable bit
#define SYSCTL_PCLKCR17_CLB3   0x4U    // CLB3 Clock Enable bit
#define SYSCTL_PCLKCR17_CLB4   0x8U    // CLB4 Clock Enable bit
#define SYSCTL_PCLKCR17_CLB5   0x10U   // CLB5 Clock Enable bit
#define SYSCTL_PCLKCR17_CLB6   0x20U   // CLB6 Clock Enable bit
#define SYSCTL_PCLKCR17_CLB7   0x40U   // CLB7 Clock Enable bit
#define SYSCTL_PCLKCR17_CLB8   0x80U   // CLB8 Clock Enable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR18 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR18_FSITX_A   0x1U        // FSITX_A Clock Enable bit
#define SYSCTL_PCLKCR18_FSITX_B   0x2U        // FSITX_B Clock Enable bit
#define SYSCTL_PCLKCR18_FSIRX_A   0x10000U    // FSIRX_A Clock Enable bit
#define SYSCTL_PCLKCR18_FSIRX_B   0x20000U    // FSIRX_B Clock Enable bit
#define SYSCTL_PCLKCR18_FSIRX_C   0x40000U    // FSIRX_C Clock Enable bit
#define SYSCTL_PCLKCR18_FSIRX_D   0x80000U    // FSIRX_D Clock Enable bit
#define SYSCTL_PCLKCR18_FSIRX_E   0x100000U   // FSIRX_E Clock Enable bit
#define SYSCTL_PCLKCR18_FSIRX_F   0x200000U   // FSIRX_F Clock Enable bit
#define SYSCTL_PCLKCR18_FSIRX_G   0x400000U   // FSIRX_G Clock Enable bit
#define SYSCTL_PCLKCR18_FSIRX_H   0x800000U   // FSIRX_H Clock Enable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR20 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR20_PMBUS_A   0x1U   // PMBUS_A Clock Enable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR21 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR21_DCC0   0x1U   // DCC0 Clock Enable Bit
#define SYSCTL_PCLKCR21_DCC1   0x2U   // DCC1 Clock Enable Bit
#define SYSCTL_PCLKCR21_DCC2   0x4U   // DCC2 Clock Enable Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR22 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR22_MPOSTCLK   0x1U   // MPOSTCLK Clock Enable Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the PCLKCR23 register
//
//*************************************************************************************************
#define SYSCTL_PCLKCR23_ETHERCAT   0x1U   // ETHERCAT Clock Enable Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SIMRESET register
//
//*************************************************************************************************
#define SYSCTL_SIMRESET_CPU1RSN   0x1U          // Generates a reset to CPU
#define SYSCTL_SIMRESET_XRSN      0x2U          // Generates a simulated XRSn
#define SYSCTL_SIMRESET_KEY_S     16U
#define SYSCTL_SIMRESET_KEY_M     0xFFFF0000U   // Key value

//*************************************************************************************************
//
// The following are defines for the bit fields in the LPMCR register
//
//*************************************************************************************************
#define SYSCTL_LPMCR_LPM_S         0U
#define SYSCTL_LPMCR_LPM_M         0x3U      // Low Power Mode setting
#define SYSCTL_LPMCR_QUALSTDBY_S   2U
#define SYSCTL_LPMCR_QUALSTDBY_M   0xFCU     // STANDBY Wakeup Pin Qualification Setting
#define SYSCTL_LPMCR_WDINTE        0x8000U   // Enable for WDINT wakeup from STANDBY

//*************************************************************************************************
//
// The following are defines for the bit fields in the GPIOLPMSEL0 register
//
//*************************************************************************************************
#define SYSCTL_GPIOLPMSEL0_GPIO0    0x1U          // GPIO0 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO1    0x2U          // GPIO1 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO2    0x4U          // GPIO2 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO3    0x8U          // GPIO3 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO4    0x10U         // GPIO4 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO5    0x20U         // GPIO5 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO6    0x40U         // GPIO6 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO7    0x80U         // GPIO7 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO8    0x100U        // GPIO8 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO9    0x200U        // GPIO9 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO10   0x400U        // GPIO10 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO11   0x800U        // GPIO11 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO12   0x1000U       // GPIO12 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO13   0x2000U       // GPIO13 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO14   0x4000U       // GPIO14 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO15   0x8000U       // GPIO15 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO16   0x10000U      // GPIO16 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO17   0x20000U      // GPIO17 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO18   0x40000U      // GPIO18 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO19   0x80000U      // GPIO19 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO20   0x100000U     // GPIO20 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO21   0x200000U     // GPIO21 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO22   0x400000U     // GPIO22 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO23   0x800000U     // GPIO23 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO24   0x1000000U    // GPIO24 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO25   0x2000000U    // GPIO25 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO26   0x4000000U    // GPIO26 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO27   0x8000000U    // GPIO27 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO28   0x10000000U   // GPIO28 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO29   0x20000000U   // GPIO29 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO30   0x40000000U   // GPIO30 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL0_GPIO31   0x80000000U   // GPIO31 Enable for LPM Wakeup

//*************************************************************************************************
//
// The following are defines for the bit fields in the GPIOLPMSEL1 register
//
//*************************************************************************************************
#define SYSCTL_GPIOLPMSEL1_GPIO32   0x1U          // GPIO32 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO33   0x2U          // GPIO33 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO34   0x4U          // GPIO34 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO35   0x8U          // GPIO35 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO36   0x10U         // GPIO36 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO37   0x20U         // GPIO37 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO38   0x40U         // GPIO38 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO39   0x80U         // GPIO39 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO40   0x100U        // GPIO40 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO41   0x200U        // GPIO41 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO42   0x400U        // GPIO42 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO43   0x800U        // GPIO43 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO44   0x1000U       // GPIO44 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO45   0x2000U       // GPIO45 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO46   0x4000U       // GPIO46 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO47   0x8000U       // GPIO47 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO48   0x10000U      // GPIO48 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO49   0x20000U      // GPIO49 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO50   0x40000U      // GPIO50 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO51   0x80000U      // GPIO51 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO52   0x100000U     // GPIO52 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO53   0x200000U     // GPIO53 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO54   0x400000U     // GPIO54 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO55   0x800000U     // GPIO55 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO56   0x1000000U    // GPIO56 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO57   0x2000000U    // GPIO57 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO58   0x4000000U    // GPIO58 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO59   0x8000000U    // GPIO59 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO60   0x10000000U   // GPIO60 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO61   0x20000000U   // GPIO61 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO62   0x40000000U   // GPIO62 Enable for LPM Wakeup
#define SYSCTL_GPIOLPMSEL1_GPIO63   0x80000000U   // GPIO63 Enable for LPM Wakeup

//*************************************************************************************************
//
// The following are defines for the bit fields in the TMR2CLKCTL register
//
//*************************************************************************************************
#define SYSCTL_TMR2CLKCTL_TMR2CLKSRCSEL_S     0U
#define SYSCTL_TMR2CLKCTL_TMR2CLKSRCSEL_M     0x7U    // CPU Timer 2 Clock Source Select Bit
#define SYSCTL_TMR2CLKCTL_TMR2CLKPRESCALE_S   3U
#define SYSCTL_TMR2CLKCTL_TMR2CLKPRESCALE_M   0x38U   // CPU Timer 2 Clock Pre-Scale Value

//*************************************************************************************************
//
// The following are defines for the bit fields in the RESCCLR register
//
//*************************************************************************************************
#define SYSCTL_RESCCLR_POR                0x1U     // POR Reset Cause Indication Bit
#define SYSCTL_RESCCLR_XRSN               0x2U     // XRSn Reset Cause Indication Bit
#define SYSCTL_RESCCLR_WDRSN              0x4U     // WDRSn Reset Cause Indication Bit
#define SYSCTL_RESCCLR_NMIWDRSN           0x8U     // NMIWDRSn Reset Cause Indication Bit
#define SYSCTL_RESCCLR_HWBISTN            0x20U    // HWBISTn Reset Cause Indication Bit
#define SYSCTL_RESCCLR_SCCRESETN          0x100U   // SCCRESETn Reset Cause Indication Bit
#define SYSCTL_RESCCLR_ECAT_RESET_OUT     0x200U   // ECAT_RESET_OUT Reset Cause Indication Bit
#define SYSCTL_RESCCLR_SIMRESET_CPU1RSN   0x400U   // SIMRESET_CPU1RSn Reset Cause Indication Bit
#define SYSCTL_RESCCLR_SIMRESET_XRSN      0x800U   // SIMRESET_XRSn Reset Cause Indication Bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the RESC register
//
//*************************************************************************************************
#define SYSCTL_RESC_POR                0x1U          // POR Reset Cause Indication Bit
#define SYSCTL_RESC_XRSN               0x2U          // XRSn Reset Cause Indication Bit
#define SYSCTL_RESC_WDRSN              0x4U          // WDRSn Reset Cause Indication Bit
#define SYSCTL_RESC_NMIWDRSN           0x8U          // NMIWDRSn Reset Cause Indication Bit
#define SYSCTL_RESC_HWBISTN            0x20U         // HWBISTn Reset Cause Indication Bit
#define SYSCTL_RESC_SCCRESETN          0x100U        // SCCRESETn Reset Cause Indication Bit
#define SYSCTL_RESC_ECAT_RESET_OUT     0x200U        // ECAT_RESET_OUT Reset Cause Indication Bit
#define SYSCTL_RESC_SIMRESET_CPU1RSN   0x400U        // SIMRESET_CPU1RSn Reset Cause Indication Bit
#define SYSCTL_RESC_SIMRESET_XRSN      0x800U        // SIMRESET_XRSn Reset Cause Indication Bit
#define SYSCTL_RESC_XRSN_PIN_STATUS    0x40000000U   // XRSN Pin Status
#define SYSCTL_RESC_TRSTN_PIN_STATUS   0x80000000U   // TRSTn Status

//*************************************************************************************************
//
// The following are defines for the bit fields in the MCANWAKESTATUS register
//
//*************************************************************************************************
#define SYSCTL_MCANWAKESTATUS_WAKE   0x1U   // MCAN Wake Status

//*************************************************************************************************
//
// The following are defines for the bit fields in the MCANWAKESTATUSCLR register
//
//*************************************************************************************************
#define SYSCTL_MCANWAKESTATUSCLR_WAKE   0x1U   // Cear bit for MCANWAKESTATUS.WAKE bit


//*************************************************************************************************
//
// The following are defines for the bit fields in the CLA1TASKSRCSELLOCK register
//
//*************************************************************************************************
#define SYSCTL_CLA1TASKSRCSELLOCK_CLA1TASKSRCSEL1   0x1U   // CLA1TASKSRCSEL1 Register Lock bit
#define SYSCTL_CLA1TASKSRCSELLOCK_CLA1TASKSRCSEL2   0x2U   // CLA1TASKSRCSEL2 Register Lock bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the DMACHSRCSELLOCK register
//
//*************************************************************************************************
#define SYSCTL_DMACHSRCSELLOCK_DMACHSRCSEL1   0x1U   // DMACHSRCSEL1 Register Lock bit
#define SYSCTL_DMACHSRCSELLOCK_DMACHSRCSEL2   0x2U   // DMACHSRCSEL2 Register Lock bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLA1TASKSRCSEL1 register
//
//*************************************************************************************************
#define SYSCTL_CLA1TASKSRCSEL1_TASK1_S   0U
#define SYSCTL_CLA1TASKSRCSEL1_TASK1_M   0xFFU         // Selects the Trigger Source for TASK1 of
                                                       // CLA1
#define SYSCTL_CLA1TASKSRCSEL1_TASK2_S   8U
#define SYSCTL_CLA1TASKSRCSEL1_TASK2_M   0xFF00U       // Selects the Trigger Source for TASK2 of
                                                       // CLA1
#define SYSCTL_CLA1TASKSRCSEL1_TASK3_S   16U
#define SYSCTL_CLA1TASKSRCSEL1_TASK3_M   0xFF0000U     // Selects the Trigger Source for TASK3 of
                                                       // CLA1
#define SYSCTL_CLA1TASKSRCSEL1_TASK4_S   24U
#define SYSCTL_CLA1TASKSRCSEL1_TASK4_M   0xFF000000U   // Selects the Trigger Source for TASK4 of
                                                       // CLA1

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLA1TASKSRCSEL2 register
//
//*************************************************************************************************
#define SYSCTL_CLA1TASKSRCSEL2_TASK5_S   0U
#define SYSCTL_CLA1TASKSRCSEL2_TASK5_M   0xFFU         // Selects the Trigger Source for TASK5 of
                                                       // CLA1
#define SYSCTL_CLA1TASKSRCSEL2_TASK6_S   8U
#define SYSCTL_CLA1TASKSRCSEL2_TASK6_M   0xFF00U       // Selects the Trigger Source for TASK6 of
                                                       // CLA1
#define SYSCTL_CLA1TASKSRCSEL2_TASK7_S   16U
#define SYSCTL_CLA1TASKSRCSEL2_TASK7_M   0xFF0000U     // Selects the Trigger Source for TASK7 of
                                                       // CLA1
#define SYSCTL_CLA1TASKSRCSEL2_TASK8_S   24U
#define SYSCTL_CLA1TASKSRCSEL2_TASK8_M   0xFF000000U   // Selects the Trigger Source for TASK8 of
                                                       // CLA1

//*************************************************************************************************
//
// The following are defines for the bit fields in the DMACHSRCSEL1 register
//
//*************************************************************************************************
#define SYSCTL_DMACHSRCSEL1_CH1_S   0U
#define SYSCTL_DMACHSRCSEL1_CH1_M   0xFFU         // Selects the Trigger and Sync Source CH1 of DMA
#define SYSCTL_DMACHSRCSEL1_CH2_S   8U
#define SYSCTL_DMACHSRCSEL1_CH2_M   0xFF00U       // Selects the Trigger and Sync Source CH2 of DMA
#define SYSCTL_DMACHSRCSEL1_CH3_S   16U
#define SYSCTL_DMACHSRCSEL1_CH3_M   0xFF0000U     // Selects the Trigger and Sync Source CH3 of DMA
#define SYSCTL_DMACHSRCSEL1_CH4_S   24U
#define SYSCTL_DMACHSRCSEL1_CH4_M   0xFF000000U   // Selects the Trigger and Sync Source CH4 of DMA

//*************************************************************************************************
//
// The following are defines for the bit fields in the DMACHSRCSEL2 register
//
//*************************************************************************************************
#define SYSCTL_DMACHSRCSEL2_CH5_S   0U
#define SYSCTL_DMACHSRCSEL2_CH5_M   0xFFU     // Selects the Trigger and Sync Source CH5 of DMA
#define SYSCTL_DMACHSRCSEL2_CH6_S   8U
#define SYSCTL_DMACHSRCSEL2_CH6_M   0xFF00U   // Selects the Trigger and Sync Source CH6 of DMA


//*************************************************************************************************
//
// The following are defines for the bit fields in the DEVCFGLOCK1 register
//
//*************************************************************************************************
#define SYSCTL_DEVCFGLOCK1_CPUSEL0    0x1U         // Lock bit for CPUSEL0 register
#define SYSCTL_DEVCFGLOCK1_CPUSEL1    0x2U         // Lock bit for CPUSEL1 register
#define SYSCTL_DEVCFGLOCK1_CPUSEL2    0x4U         // Lock bit for CPUSEL2 register
#define SYSCTL_DEVCFGLOCK1_CPUSEL4    0x10U        // Lock bit for CPUSEL4 register
#define SYSCTL_DEVCFGLOCK1_CPUSEL5    0x20U        // Lock bit for CPUSEL5 register
#define SYSCTL_DEVCFGLOCK1_CPUSEL6    0x40U        // Lock bit for CPUSEL6 register
#define SYSCTL_DEVCFGLOCK1_CPUSEL7    0x80U        // Lock bit for CPUSEL7 register
#define SYSCTL_DEVCFGLOCK1_CPUSEL8    0x100U       // Lock bit for CPUSEL8 register
#define SYSCTL_DEVCFGLOCK1_CPUSEL9    0x200U       // Lock bit for CPUSEL9 register
#define SYSCTL_DEVCFGLOCK1_CPUSEL11   0x800U       // Lock bit for CPUSEL11 register
#define SYSCTL_DEVCFGLOCK1_CPUSEL12   0x1000U      // Lock bit for CPUSEL12 register
#define SYSCTL_DEVCFGLOCK1_CPUSEL14   0x4000U      // Lock bit for CPUSEL14 register
#define SYSCTL_DEVCFGLOCK1_CPUSEL15   0x8000U      // Lock bit for CPUSEL15 register
#define SYSCTL_DEVCFGLOCK1_CPUSEL16   0x10000U     // Lock bit for CPUSEL16 register
#define SYSCTL_DEVCFGLOCK1_CPUSEL18   0x40000U     // Lock bit for CPUSEL18 register
#define SYSCTL_DEVCFGLOCK1_CPUSEL25   0x2000000U   // Lock bit for CPUSEL25 register

//*************************************************************************************************
//
// The following are defines for the bit fields in the PARTIDL register
//
//*************************************************************************************************
#define SYSCTL_PARTIDL_QUAL_S                     6U
#define SYSCTL_PARTIDL_QUAL_M                     0xC0U         // Qualification Status
#define SYSCTL_PARTIDL_PIN_COUNT_S                8U
#define SYSCTL_PARTIDL_PIN_COUNT_M                0x700U        // Device Pin Count
#define SYSCTL_PARTIDL_INSTASPIN_S                13U
#define SYSCTL_PARTIDL_INSTASPIN_M                0x6000U       // Motorware feature set
#define SYSCTL_PARTIDL_FLASH_SIZE_S               16U
#define SYSCTL_PARTIDL_FLASH_SIZE_M               0xFF0000U     // Flash size in KB per CPU
#define SYSCTL_PARTIDL_PARTID_FORMAT_REVISION_S   28U
#define SYSCTL_PARTIDL_PARTID_FORMAT_REVISION_M   0xF0000000U   // Revision of the PARTID format

//*************************************************************************************************
//
// The following are defines for the bit fields in the PARTIDH register
//
//*************************************************************************************************
#define SYSCTL_PARTIDH_FAMILY_S            8U
#define SYSCTL_PARTIDH_FAMILY_M            0xFF00U       // Device family
#define SYSCTL_PARTIDH_PARTNO_S            16U
#define SYSCTL_PARTIDH_PARTNO_M            0xFF0000U     // Device part number
#define SYSCTL_PARTIDH_DEVICE_CLASS_ID_S   24U
#define SYSCTL_PARTIDH_DEVICE_CLASS_ID_M   0xFF000000U   // Device class ID

//*************************************************************************************************
//
// The following are defines for the bit fields in the REVID register
//
//*************************************************************************************************
#define SYSCTL_REVID_REVID_S   0U
#define SYSCTL_REVID_REVID_M   0xFFFFU   // Device Revision ID. This is specific to the Device

//*************************************************************************************************
//
// The following are defines for the bit fields in the PERCNF1 register
//
//*************************************************************************************************
#define SYSCTL_PERCNF1_ADC_A_MODE   0x1U       // ADC Wrapper-1 mode setting bit
#define SYSCTL_PERCNF1_ADC_B_MODE   0x2U       // ADC Wrapper-2 mode setting bit
#define SYSCTL_PERCNF1_ADC_C_MODE   0x4U       // ADC Wrapper-3 mode setting bit
#define SYSCTL_PERCNF1_ADC_D_MODE   0x8U       // ADC Wrapper-4 mode setting bit
#define SYSCTL_PERCNF1_USB_A_PHY    0x10000U   // USB_A_PHY enable/disable bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the FUSEERR register
//
//*************************************************************************************************
#define SYSCTL_FUSEERR_ALERR_S   0U
#define SYSCTL_FUSEERR_ALERR_M   0x1FU   // Efuse Autoload Error Status
#define SYSCTL_FUSEERR_ERR       0x20U   // Efuse Self Test Error Status

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES0 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES0_CPU1_CLA1        0x1U         // CPU1_CLA1 software reset bit
#define SYSCTL_SOFTPRES0_CPU2_CLA1        0x4U         // CPU2_CLA1 software reset bit
#define SYSCTL_SOFTPRES0_CPU1_CPUBGCRC    0x2000U      // CPUBGCRC Module reset bit
#define SYSCTL_SOFTPRES0_CPU1_CLA1BGCRC   0x4000U      // CLA1BGCRC Module reset bit
#define SYSCTL_SOFTPRES0_CPU2_CPUBGCRC    0x10000U     // CPUBGCRC Module reset bit
#define SYSCTL_SOFTPRES0_CPU2_CLA1BGCRC   0x20000U     // CLA1BGCRC Module reset bit
#define SYSCTL_SOFTPRES0_CPU1_ERAD        0x1000000U   // ERAD Module reset bit
#define SYSCTL_SOFTPRES0_CPU2_ERAD        0x2000000U   // ERAD Module reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES1 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES1_EMIF1   0x1U   // EMIF1 software reset bit
#define SYSCTL_SOFTPRES1_EMIF2   0x2U   // EMIF2 software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES2 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES2_EPWM1    0x1U      // EPWM1 software reset bit
#define SYSCTL_SOFTPRES2_EPWM2    0x2U      // EPWM2 software reset bit
#define SYSCTL_SOFTPRES2_EPWM3    0x4U      // EPWM3 software reset bit
#define SYSCTL_SOFTPRES2_EPWM4    0x8U      // EPWM4 software reset bit
#define SYSCTL_SOFTPRES2_EPWM5    0x10U     // EPWM5 software reset bit
#define SYSCTL_SOFTPRES2_EPWM6    0x20U     // EPWM6 software reset bit
#define SYSCTL_SOFTPRES2_EPWM7    0x40U     // EPWM7 software reset bit
#define SYSCTL_SOFTPRES2_EPWM8    0x80U     // EPWM8 software reset bit
#define SYSCTL_SOFTPRES2_EPWM9    0x100U    // EPWM9 software reset bit
#define SYSCTL_SOFTPRES2_EPWM10   0x200U    // EPWM10 software reset bit
#define SYSCTL_SOFTPRES2_EPWM11   0x400U    // EPWM11 software reset bit
#define SYSCTL_SOFTPRES2_EPWM12   0x800U    // EPWM12 software reset bit
#define SYSCTL_SOFTPRES2_EPWM13   0x1000U   // EPWM13 software reset bit
#define SYSCTL_SOFTPRES2_EPWM14   0x2000U   // EPWM14 software reset bit
#define SYSCTL_SOFTPRES2_EPWM15   0x4000U   // EPWM15 software reset bit
#define SYSCTL_SOFTPRES2_EPWM16   0x8000U   // EPWM16 software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES3 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES3_ECAP1   0x1U    // ECAP1 software reset bit
#define SYSCTL_SOFTPRES3_ECAP2   0x2U    // ECAP2 software reset bit
#define SYSCTL_SOFTPRES3_ECAP3   0x4U    // ECAP3 software reset bit
#define SYSCTL_SOFTPRES3_ECAP4   0x8U    // ECAP4 software reset bit
#define SYSCTL_SOFTPRES3_ECAP5   0x10U   // ECAP5 software reset bit
#define SYSCTL_SOFTPRES3_ECAP6   0x20U   // ECAP6 software reset bit
#define SYSCTL_SOFTPRES3_ECAP7   0x40U   // ECAP7 software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES4 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES4_EQEP1   0x1U   // EQEP1 software reset bit
#define SYSCTL_SOFTPRES4_EQEP2   0x2U   // EQEP2 software reset bit
#define SYSCTL_SOFTPRES4_EQEP3   0x4U   // EQEP3 software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES6 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES6_SD1   0x1U   // SD1 software reset bit
#define SYSCTL_SOFTPRES6_SD2   0x2U   // SD2 software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES7 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES7_SCI_A   0x1U   // SCI_A software reset bit
#define SYSCTL_SOFTPRES7_SCI_B   0x2U   // SCI_B software reset bit
#define SYSCTL_SOFTPRES7_SCI_C   0x4U   // SCI_C software reset bit
#define SYSCTL_SOFTPRES7_SCI_D   0x8U   // SCI_D software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES8 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES8_SPI_A   0x1U   // SPI_A software reset bit
#define SYSCTL_SOFTPRES8_SPI_B   0x2U   // SPI_B software reset bit
#define SYSCTL_SOFTPRES8_SPI_C   0x4U   // SPI_C software reset bit
#define SYSCTL_SOFTPRES8_SPI_D   0x8U   // SPI_D software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES9 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES9_I2C_A   0x1U   // I2C_A software reset bit
#define SYSCTL_SOFTPRES9_I2C_B   0x2U   // I2C_B software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES10 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES10_CAN_A    0x1U    // CAN_A software reset bit
#define SYSCTL_SOFTPRES10_CAN_B    0x2U    // CAN_B software reset bit
#define SYSCTL_SOFTPRES10_MCAN_A   0x10U   // MCAN_A software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES11 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES11_MCBSP_A   0x1U       // McBSP_A software reset bit
#define SYSCTL_SOFTPRES11_MCBSP_B   0x2U       // McBSP_B software reset bit
#define SYSCTL_SOFTPRES11_USB_A     0x10000U   // USB_A software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES13 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES13_ADC_A   0x1U   // ADC_A software reset bit
#define SYSCTL_SOFTPRES13_ADC_B   0x2U   // ADC_B software reset bit
#define SYSCTL_SOFTPRES13_ADC_C   0x4U   // ADC_C software reset bit
#define SYSCTL_SOFTPRES13_ADC_D   0x8U   // ADC_D software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES14 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES14_CMPSS1   0x1U    // CMPSS1 software reset bit
#define SYSCTL_SOFTPRES14_CMPSS2   0x2U    // CMPSS2 software reset bit
#define SYSCTL_SOFTPRES14_CMPSS3   0x4U    // CMPSS3 software reset bit
#define SYSCTL_SOFTPRES14_CMPSS4   0x8U    // CMPSS4 software reset bit
#define SYSCTL_SOFTPRES14_CMPSS5   0x10U   // CMPSS5 software reset bit
#define SYSCTL_SOFTPRES14_CMPSS6   0x20U   // CMPSS6 software reset bit
#define SYSCTL_SOFTPRES14_CMPSS7   0x40U   // CMPSS7 software reset bit
#define SYSCTL_SOFTPRES14_CMPSS8   0x80U   // CMPSS8 software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES16 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES16_DAC_A   0x10000U   // Buffered_DAC12_1 software reset bit
#define SYSCTL_SOFTPRES16_DAC_B   0x20000U   // Buffered_DAC12_2 software reset bit
#define SYSCTL_SOFTPRES16_DAC_C   0x40000U   // Buffered_DAC12_3 software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES17 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES17_CLB1   0x1U    // CLB1 software reset bit
#define SYSCTL_SOFTPRES17_CLB2   0x2U    // CLB2 software reset bit
#define SYSCTL_SOFTPRES17_CLB3   0x4U    // CLB3 software reset bit
#define SYSCTL_SOFTPRES17_CLB4   0x8U    // CLB4 software reset bit
#define SYSCTL_SOFTPRES17_CLB5   0x10U   // CLB5 software reset bit
#define SYSCTL_SOFTPRES17_CLB6   0x20U   // CLB6 software reset bit
#define SYSCTL_SOFTPRES17_CLB7   0x40U   // CLB7 software reset bit
#define SYSCTL_SOFTPRES17_CLB8   0x80U   // CLB8 software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES18 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES18_FSITX_A   0x1U        // FSITX_A software reset bit
#define SYSCTL_SOFTPRES18_FSITX_B   0x2U        // FSITX_B software reset bit
#define SYSCTL_SOFTPRES18_FSIRX_A   0x10000U    // FSIRX_A software reset bit
#define SYSCTL_SOFTPRES18_FSIRX_B   0x20000U    // FSIRX_B software reset bit
#define SYSCTL_SOFTPRES18_FSIRX_C   0x40000U    // FSIRX_C software reset bit
#define SYSCTL_SOFTPRES18_FSIRX_D   0x80000U    // FSIRX_D software reset bit
#define SYSCTL_SOFTPRES18_FSIRX_E   0x100000U   // FSIRX_E software reset bit
#define SYSCTL_SOFTPRES18_FSIRX_F   0x200000U   // FSIRX_F software reset bit
#define SYSCTL_SOFTPRES18_FSIRX_G   0x400000U   // FSIRX_G software reset bit
#define SYSCTL_SOFTPRES18_FSIRX_H   0x800000U   // FSIRX_H software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES20 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES20_PMBUS_A   0x1U   // PMBUS_A software reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES21 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES21_DCC0   0x1U   // DCC0 Module reset bit
#define SYSCTL_SOFTPRES21_DCC1   0x2U   // DCC1 Module reset bit
#define SYSCTL_SOFTPRES21_DCC2   0x4U   // DCC2 Module reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SOFTPRES23 register
//
//*************************************************************************************************
#define SYSCTL_SOFTPRES23_ETHERCAT   0x1U   // ETHERCAT Module reset bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL0 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL0_EPWM1    0x1U      // EPWM1 CPU select bit
#define SYSCTL_CPUSEL0_EPWM2    0x2U      // EPWM2 CPU select bit
#define SYSCTL_CPUSEL0_EPWM3    0x4U      // EPWM3 CPU select bit
#define SYSCTL_CPUSEL0_EPWM4    0x8U      // EPWM4 CPU select bit
#define SYSCTL_CPUSEL0_EPWM5    0x10U     // EPWM5 CPU select bit
#define SYSCTL_CPUSEL0_EPWM6    0x20U     // EPWM6 CPU select bit
#define SYSCTL_CPUSEL0_EPWM7    0x40U     // EPWM7 CPU select bit
#define SYSCTL_CPUSEL0_EPWM8    0x80U     // EPWM8 CPU select bit
#define SYSCTL_CPUSEL0_EPWM9    0x100U    // EPWM9 CPU select bit
#define SYSCTL_CPUSEL0_EPWM10   0x200U    // EPWM10 CPU select bit
#define SYSCTL_CPUSEL0_EPWM11   0x400U    // EPWM11 CPU select bit
#define SYSCTL_CPUSEL0_EPWM12   0x800U    // EPWM12 CPU select bit
#define SYSCTL_CPUSEL0_EPWM13   0x1000U   // EPWM13 CPU select bit
#define SYSCTL_CPUSEL0_EPWM14   0x2000U   // EPWM14 CPU select bit
#define SYSCTL_CPUSEL0_EPWM15   0x4000U   // EPWM15 CPU select bit
#define SYSCTL_CPUSEL0_EPWM16   0x8000U   // EPWM16 CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL1 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL1_ECAP1   0x1U    // ECAP1 CPU select bit
#define SYSCTL_CPUSEL1_ECAP2   0x2U    // ECAP2 CPU select bit
#define SYSCTL_CPUSEL1_ECAP3   0x4U    // ECAP3 CPU select bit
#define SYSCTL_CPUSEL1_ECAP4   0x8U    // ECAP4 CPU select bit
#define SYSCTL_CPUSEL1_ECAP5   0x10U   // ECAP5 CPU select bit
#define SYSCTL_CPUSEL1_ECAP6   0x20U   // ECAP6 CPU select bit
#define SYSCTL_CPUSEL1_ECAP7   0x40U   // ECAP7 CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL2 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL2_EQEP1   0x1U   // EQEP1 CPU select bit
#define SYSCTL_CPUSEL2_EQEP2   0x2U   // EQEP2 CPU select bit
#define SYSCTL_CPUSEL2_EQEP3   0x4U   // EQEP3 CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL4 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL4_SD1   0x1U   // SD1 CPU select bit
#define SYSCTL_CPUSEL4_SD2   0x2U   // SD2 CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL5 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL5_SCI_A   0x1U   // SCI_A CPU select bit
#define SYSCTL_CPUSEL5_SCI_B   0x2U   // SCI_B CPU select bit
#define SYSCTL_CPUSEL5_SCI_C   0x4U   // SCI_C CPU select bit
#define SYSCTL_CPUSEL5_SCI_D   0x8U   // SCI_D CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL6 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL6_SPI_A   0x1U   // SPI_A CPU select bit
#define SYSCTL_CPUSEL6_SPI_B   0x2U   // SPI_B CPU select bit
#define SYSCTL_CPUSEL6_SPI_C   0x4U   // SPI_C CPU select bit
#define SYSCTL_CPUSEL6_SPI_D   0x8U   // SPI_D CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL7 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL7_I2C_A   0x1U   // I2C_A CPU select bit
#define SYSCTL_CPUSEL7_I2C_B   0x2U   // I2C_B CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL8 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL8_CAN_A    0x1U    // CAN_A CPU select bit
#define SYSCTL_CPUSEL8_CAN_B    0x2U    // CAN_B CPU select bit
#define SYSCTL_CPUSEL8_MCAN_A   0x10U   // MCAN_A CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL9 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL9_MCBSP_A   0x1U   // McBSP_A CPU select bit
#define SYSCTL_CPUSEL9_MCBSP_B   0x2U   // McBSP_B CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL11 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL11_ADC_A   0x1U   // ADC_A CPU select bit
#define SYSCTL_CPUSEL11_ADC_B   0x2U   // ADC_B CPU select bit
#define SYSCTL_CPUSEL11_ADC_C   0x4U   // ADC_C CPU select bit
#define SYSCTL_CPUSEL11_ADC_D   0x8U   // ADC_D CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL12 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL12_CMPSS1   0x1U    // CMPSS1 CPU select bit
#define SYSCTL_CPUSEL12_CMPSS2   0x2U    // CMPSS2 CPU select bit
#define SYSCTL_CPUSEL12_CMPSS3   0x4U    // CMPSS3 CPU select bit
#define SYSCTL_CPUSEL12_CMPSS4   0x8U    // CMPSS4 CPU select bit
#define SYSCTL_CPUSEL12_CMPSS5   0x10U   // CMPSS5 CPU select bit
#define SYSCTL_CPUSEL12_CMPSS6   0x20U   // CMPSS6 CPU select bit
#define SYSCTL_CPUSEL12_CMPSS7   0x40U   // CMPSS7 CPU select bit
#define SYSCTL_CPUSEL12_CMPSS8   0x80U   // CMPSS8 CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL14 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL14_DAC_A   0x10000U   // Buffered_DAC12_1 CPU select bit
#define SYSCTL_CPUSEL14_DAC_B   0x20000U   // Buffered_DAC12_2 CPU select bit
#define SYSCTL_CPUSEL14_DAC_C   0x40000U   // Buffered_DAC12_3 CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL15 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL15_CLB1   0x1U    // CLB1 CPU select bit
#define SYSCTL_CPUSEL15_CLB2   0x2U    // CLB2 CPU select bit
#define SYSCTL_CPUSEL15_CLB3   0x4U    // CLB3 CPU select bit
#define SYSCTL_CPUSEL15_CLB4   0x8U    // CLB4 CPU select bit
#define SYSCTL_CPUSEL15_CLB5   0x10U   // CLB5 CPU select bit
#define SYSCTL_CPUSEL15_CLB6   0x20U   // CLB6 CPU select bit
#define SYSCTL_CPUSEL15_CLB7   0x40U   // CLB7 CPU select bit
#define SYSCTL_CPUSEL15_CLB8   0x80U   // CLB8 CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL16 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL16_FSITX_A   0x1U        // FSITX_A CPU select bit
#define SYSCTL_CPUSEL16_FSITX_B   0x2U        // FSITX_B CPU select bit
#define SYSCTL_CPUSEL16_FSIRX_A   0x10000U    // FSIRX_A CPU select bit
#define SYSCTL_CPUSEL16_FSIRX_B   0x20000U    // FSIRX_B CPU select bit
#define SYSCTL_CPUSEL16_FSIRX_C   0x40000U    // FSIRX_C CPU select bit
#define SYSCTL_CPUSEL16_FSIRX_D   0x80000U    // FSIRX_D CPU select bit
#define SYSCTL_CPUSEL16_FSIRX_E   0x100000U   // FSIRX_E CPU select bit
#define SYSCTL_CPUSEL16_FSIRX_F   0x200000U   // FSIRX_F CPU select bit
#define SYSCTL_CPUSEL16_FSIRX_G   0x400000U   // FSIRX_G CPU select bit
#define SYSCTL_CPUSEL16_FSIRX_H   0x800000U   // FSIRX_H CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL18 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL18_PMBUS_A   0x1U   // PMBUS_A CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPUSEL25 register
//
//*************************************************************************************************
#define SYSCTL_CPUSEL25_HRCAL_A   0x1U   // HRCAL CPU select bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU2RESCTL register
//
//*************************************************************************************************
#define SYSCTL_CPU2RESCTL_RESET   0x1U          // CPU2 Reset Control bit
#define SYSCTL_CPU2RESCTL_KEY_S   16U
#define SYSCTL_CPU2RESCTL_KEY_M   0xFFFF0000U   // Key Qualifier for writes to this register

//*************************************************************************************************
//
// The following are defines for the bit fields in the RSTSTAT register
//
//*************************************************************************************************
#define SYSCTL_RSTSTAT_CPU2RES           0x1U   // CPU2 Reset Status bit
#define SYSCTL_RSTSTAT_CPU2NMIWDRST      0x2U   // Tells whether a CPU2.NMIWD reset was issued to
                                                // CPU2 or not
#define SYSCTL_RSTSTAT_CPU2HWBISTRST_S   2U
#define SYSCTL_RSTSTAT_CPU2HWBISTRST_M   0xCU   // Tells whether a HWBIST reset was issued to CPU2
                                                // or not

//*************************************************************************************************
//
// The following are defines for the bit fields in the LPMSTAT register
//
//*************************************************************************************************
#define SYSCTL_LPMSTAT_CPU2LPMSTAT_S   0U
#define SYSCTL_LPMSTAT_CPU2LPMSTAT_M   0x3U   // CPU2 LPM Status

//*************************************************************************************************
//
// The following are defines for the bit fields in the USBTYPE register
//
//*************************************************************************************************
#define SYSCTL_USBTYPE_TYPE_S   0U
#define SYSCTL_USBTYPE_TYPE_M   0x3U      // Configure USB type
#define SYSCTL_USBTYPE_LOCK     0x8000U   // Lock bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the ECAPTYPE register
//
//*************************************************************************************************
#define SYSCTL_ECAPTYPE_TYPE_S   0U
#define SYSCTL_ECAPTYPE_TYPE_M   0x3U      // Configure ECAP type
#define SYSCTL_ECAPTYPE_LOCK     0x8000U   // Lock bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFMTYPE register
//
//*************************************************************************************************
#define SYSCTL_SDFMTYPE_TYPE_S   0U
#define SYSCTL_SDFMTYPE_TYPE_M   0x3U      // Configure SDFM type
#define SYSCTL_SDFMTYPE_LOCK     0x8000U   // Lock bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the MEMMAPTYPE register
//
//*************************************************************************************************
#define SYSCTL_MEMMAPTYPE_TYPE_S   0U
#define SYSCTL_MEMMAPTYPE_TYPE_M   0x3U      // Configures system specific features related to
                                             // memory map.
#define SYSCTL_MEMMAPTYPE_LOCK     0x8000U   // Lock bit


//*************************************************************************************************
//
// The following are defines for the bit fields in the ADCA_AC register
//
//*************************************************************************************************
#define SYSCTL_ADCA_AC_CPUX_ACC_S   0U
#define SYSCTL_ADCA_AC_CPUX_ACC_M   0x3U    // CPU1 Access conditions to peripheral
#define SYSCTL_ADCA_AC_CLA1_ACC_S   2U
#define SYSCTL_ADCA_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_ADCA_AC_DMA1_ACC_S   4U
#define SYSCTL_ADCA_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the ADCB_AC register
//
//*************************************************************************************************
#define SYSCTL_ADCB_AC_CPUX_ACC_S   0U
#define SYSCTL_ADCB_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_ADCB_AC_CLA1_ACC_S   2U
#define SYSCTL_ADCB_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_ADCB_AC_DMA1_ACC_S   4U
#define SYSCTL_ADCB_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the ADCC_AC register
//
//*************************************************************************************************
#define SYSCTL_ADCC_AC_CPUX_ACC_S   0U
#define SYSCTL_ADCC_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_ADCC_AC_CLA1_ACC_S   2U
#define SYSCTL_ADCC_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_ADCC_AC_DMA1_ACC_S   4U
#define SYSCTL_ADCC_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the ADCD_AC register
//
//*************************************************************************************************
#define SYSCTL_ADCD_AC_CPUX_ACC_S   0U
#define SYSCTL_ADCD_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_ADCD_AC_CLA1_ACC_S   2U
#define SYSCTL_ADCD_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_ADCD_AC_DMA1_ACC_S   4U
#define SYSCTL_ADCD_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMPSS1_AC register
//
//*************************************************************************************************
#define SYSCTL_CMPSS1_AC_CPUX_ACC_S   0U
#define SYSCTL_CMPSS1_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_CMPSS1_AC_CLA1_ACC_S   2U
#define SYSCTL_CMPSS1_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_CMPSS1_AC_DMA1_ACC_S   4U
#define SYSCTL_CMPSS1_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMPSS2_AC register
//
//*************************************************************************************************
#define SYSCTL_CMPSS2_AC_CPUX_ACC_S   0U
#define SYSCTL_CMPSS2_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_CMPSS2_AC_CLA1_ACC_S   2U
#define SYSCTL_CMPSS2_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_CMPSS2_AC_DMA1_ACC_S   4U
#define SYSCTL_CMPSS2_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMPSS3_AC register
//
//*************************************************************************************************
#define SYSCTL_CMPSS3_AC_CPUX_ACC_S   0U
#define SYSCTL_CMPSS3_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_CMPSS3_AC_CLA1_ACC_S   2U
#define SYSCTL_CMPSS3_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_CMPSS3_AC_DMA1_ACC_S   4U
#define SYSCTL_CMPSS3_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMPSS4_AC register
//
//*************************************************************************************************
#define SYSCTL_CMPSS4_AC_CPUX_ACC_S   0U
#define SYSCTL_CMPSS4_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_CMPSS4_AC_CLA1_ACC_S   2U
#define SYSCTL_CMPSS4_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_CMPSS4_AC_DMA1_ACC_S   4U
#define SYSCTL_CMPSS4_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMPSS5_AC register
//
//*************************************************************************************************
#define SYSCTL_CMPSS5_AC_CPUX_ACC_S   0U
#define SYSCTL_CMPSS5_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_CMPSS5_AC_CLA1_ACC_S   2U
#define SYSCTL_CMPSS5_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_CMPSS5_AC_DMA1_ACC_S   4U
#define SYSCTL_CMPSS5_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMPSS6_AC register
//
//*************************************************************************************************
#define SYSCTL_CMPSS6_AC_CPUX_ACC_S   0U
#define SYSCTL_CMPSS6_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_CMPSS6_AC_CLA1_ACC_S   2U
#define SYSCTL_CMPSS6_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_CMPSS6_AC_DMA1_ACC_S   4U
#define SYSCTL_CMPSS6_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMPSS7_AC register
//
//*************************************************************************************************
#define SYSCTL_CMPSS7_AC_CPUX_ACC_S   0U
#define SYSCTL_CMPSS7_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_CMPSS7_AC_CLA1_ACC_S   2U
#define SYSCTL_CMPSS7_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_CMPSS7_AC_DMA1_ACC_S   4U
#define SYSCTL_CMPSS7_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMPSS8_AC register
//
//*************************************************************************************************
#define SYSCTL_CMPSS8_AC_CPUX_ACC_S   0U
#define SYSCTL_CMPSS8_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_CMPSS8_AC_CLA1_ACC_S   2U
#define SYSCTL_CMPSS8_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_CMPSS8_AC_DMA1_ACC_S   4U
#define SYSCTL_CMPSS8_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the DACA_AC register
//
//*************************************************************************************************
#define SYSCTL_DACA_AC_CPUX_ACC_S   0U
#define SYSCTL_DACA_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_DACA_AC_CLA1_ACC_S   2U
#define SYSCTL_DACA_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_DACA_AC_DMA1_ACC_S   4U
#define SYSCTL_DACA_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the DACB_AC register
//
//*************************************************************************************************
#define SYSCTL_DACB_AC_CPUX_ACC_S   0U
#define SYSCTL_DACB_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_DACB_AC_CLA1_ACC_S   2U
#define SYSCTL_DACB_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_DACB_AC_DMA1_ACC_S   4U
#define SYSCTL_DACB_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the DACC_AC register
//
//*************************************************************************************************
#define SYSCTL_DACC_AC_CPUX_ACC_S   0U
#define SYSCTL_DACC_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_DACC_AC_CLA1_ACC_S   2U
#define SYSCTL_DACC_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_DACC_AC_DMA1_ACC_S   4U
#define SYSCTL_DACC_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM1_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM1_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM1_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM1_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM1_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM1_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM1_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM2_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM2_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM2_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM2_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM2_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM2_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM2_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM3_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM3_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM3_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM3_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM3_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM3_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM3_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM4_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM4_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM4_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM4_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM4_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM4_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM4_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM5_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM5_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM5_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM5_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM5_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM5_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM5_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM6_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM6_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM6_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM6_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM6_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM6_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM6_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM7_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM7_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM7_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM7_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM7_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM7_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM7_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM8_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM8_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM8_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM8_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM8_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM8_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM8_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM9_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM9_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM9_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM9_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM9_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM9_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM9_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM10_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM10_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM10_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM10_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM10_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM10_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM10_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM11_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM11_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM11_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM11_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM11_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM11_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM11_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM12_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM12_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM12_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM12_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM12_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM12_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM12_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM13_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM13_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM13_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM13_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM13_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM13_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM13_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM14_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM14_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM14_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM14_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM14_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM14_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM14_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM15_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM15_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM15_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM15_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM15_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM15_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM15_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EPWM16_AC register
//
//*************************************************************************************************
#define SYSCTL_EPWM16_AC_CPUX_ACC_S   0U
#define SYSCTL_EPWM16_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EPWM16_AC_CLA1_ACC_S   2U
#define SYSCTL_EPWM16_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EPWM16_AC_DMA1_ACC_S   4U
#define SYSCTL_EPWM16_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EQEP1_AC register
//
//*************************************************************************************************
#define SYSCTL_EQEP1_AC_CPUX_ACC_S   0U
#define SYSCTL_EQEP1_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EQEP1_AC_CLA1_ACC_S   2U
#define SYSCTL_EQEP1_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EQEP1_AC_DMA1_ACC_S   4U
#define SYSCTL_EQEP1_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EQEP2_AC register
//
//*************************************************************************************************
#define SYSCTL_EQEP2_AC_CPUX_ACC_S   0U
#define SYSCTL_EQEP2_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EQEP2_AC_CLA1_ACC_S   2U
#define SYSCTL_EQEP2_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EQEP2_AC_DMA1_ACC_S   4U
#define SYSCTL_EQEP2_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the EQEP3_AC register
//
//*************************************************************************************************
#define SYSCTL_EQEP3_AC_CPUX_ACC_S   0U
#define SYSCTL_EQEP3_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_EQEP3_AC_CLA1_ACC_S   2U
#define SYSCTL_EQEP3_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_EQEP3_AC_DMA1_ACC_S   4U
#define SYSCTL_EQEP3_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the ECAP1_AC register
//
//*************************************************************************************************
#define SYSCTL_ECAP1_AC_CPUX_ACC_S   0U
#define SYSCTL_ECAP1_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_ECAP1_AC_CLA1_ACC_S   2U
#define SYSCTL_ECAP1_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_ECAP1_AC_DMA1_ACC_S   4U
#define SYSCTL_ECAP1_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the ECAP2_AC register
//
//*************************************************************************************************
#define SYSCTL_ECAP2_AC_CPUX_ACC_S   0U
#define SYSCTL_ECAP2_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_ECAP2_AC_CLA1_ACC_S   2U
#define SYSCTL_ECAP2_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_ECAP2_AC_DMA1_ACC_S   4U
#define SYSCTL_ECAP2_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the ECAP3_AC register
//
//*************************************************************************************************
#define SYSCTL_ECAP3_AC_CPUX_ACC_S   0U
#define SYSCTL_ECAP3_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_ECAP3_AC_CLA1_ACC_S   2U
#define SYSCTL_ECAP3_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_ECAP3_AC_DMA1_ACC_S   4U
#define SYSCTL_ECAP3_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the ECAP4_AC register
//
//*************************************************************************************************
#define SYSCTL_ECAP4_AC_CPUX_ACC_S   0U
#define SYSCTL_ECAP4_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_ECAP4_AC_CLA1_ACC_S   2U
#define SYSCTL_ECAP4_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_ECAP4_AC_DMA1_ACC_S   4U
#define SYSCTL_ECAP4_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the ECAP5_AC register
//
//*************************************************************************************************
#define SYSCTL_ECAP5_AC_CPUX_ACC_S   0U
#define SYSCTL_ECAP5_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_ECAP5_AC_CLA1_ACC_S   2U
#define SYSCTL_ECAP5_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_ECAP5_AC_DMA1_ACC_S   4U
#define SYSCTL_ECAP5_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the ECAP6_AC register
//
//*************************************************************************************************
#define SYSCTL_ECAP6_AC_CPUX_ACC_S   0U
#define SYSCTL_ECAP6_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_ECAP6_AC_CLA1_ACC_S   2U
#define SYSCTL_ECAP6_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_ECAP6_AC_DMA1_ACC_S   4U
#define SYSCTL_ECAP6_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the ECAP7_AC register
//
//*************************************************************************************************
#define SYSCTL_ECAP7_AC_CPUX_ACC_S   0U
#define SYSCTL_ECAP7_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_ECAP7_AC_CLA1_ACC_S   2U
#define SYSCTL_ECAP7_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_ECAP7_AC_DMA1_ACC_S   4U
#define SYSCTL_ECAP7_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFM1_AC register
//
//*************************************************************************************************
#define SYSCTL_SDFM1_AC_CPUX_ACC_S   0U
#define SYSCTL_SDFM1_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_SDFM1_AC_CLA1_ACC_S   2U
#define SYSCTL_SDFM1_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_SDFM1_AC_DMA1_ACC_S   4U
#define SYSCTL_SDFM1_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the SDFM2_AC register
//
//*************************************************************************************************
#define SYSCTL_SDFM2_AC_CPUX_ACC_S   0U
#define SYSCTL_SDFM2_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_SDFM2_AC_CLA1_ACC_S   2U
#define SYSCTL_SDFM2_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_SDFM2_AC_DMA1_ACC_S   4U
#define SYSCTL_SDFM2_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB1_AC register
//
//*************************************************************************************************
#define SYSCTL_CLB1_AC_CPUX_ACC_S   0U
#define SYSCTL_CLB1_AC_CPUX_ACC_M   0x3U   // CPUx Access conditions to peripheral
#define SYSCTL_CLB1_AC_CLA1_ACC_S   2U
#define SYSCTL_CLB1_AC_CLA1_ACC_M   0xCU   // CLA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB2_AC register
//
//*************************************************************************************************
#define SYSCTL_CLB2_AC_CPUX_ACC_S   0U
#define SYSCTL_CLB2_AC_CPUX_ACC_M   0x3U   // CPUx Access conditions to peripheral
#define SYSCTL_CLB2_AC_CLA1_ACC_S   2U
#define SYSCTL_CLB2_AC_CLA1_ACC_M   0xCU   // CLA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB3_AC register
//
//*************************************************************************************************
#define SYSCTL_CLB3_AC_CPUX_ACC_S   0U
#define SYSCTL_CLB3_AC_CPUX_ACC_M   0x3U   // CPUx Access conditions to peripheral
#define SYSCTL_CLB3_AC_CLA1_ACC_S   2U
#define SYSCTL_CLB3_AC_CLA1_ACC_M   0xCU   // CLA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB4_AC register
//
//*************************************************************************************************
#define SYSCTL_CLB4_AC_CPUX_ACC_S   0U
#define SYSCTL_CLB4_AC_CPUX_ACC_M   0x3U   // CPUx Access conditions to peripheral
#define SYSCTL_CLB4_AC_CLA1_ACC_S   2U
#define SYSCTL_CLB4_AC_CLA1_ACC_M   0xCU   // CLA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB5_AC register
//
//*************************************************************************************************
#define SYSCTL_CLB5_AC_CPUX_ACC_S   0U
#define SYSCTL_CLB5_AC_CPUX_ACC_M   0x3U   // CPUx Access conditions to peripheral
#define SYSCTL_CLB5_AC_CLA1_ACC_S   2U
#define SYSCTL_CLB5_AC_CLA1_ACC_M   0xCU   // CLA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB6_AC register
//
//*************************************************************************************************
#define SYSCTL_CLB6_AC_CPUX_ACC_S   0U
#define SYSCTL_CLB6_AC_CPUX_ACC_M   0x3U   // CPUx Access conditions to peripheral
#define SYSCTL_CLB6_AC_CLA1_ACC_S   2U
#define SYSCTL_CLB6_AC_CLA1_ACC_M   0xCU   // CLA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB7_AC register
//
//*************************************************************************************************
#define SYSCTL_CLB7_AC_CPUX_ACC_S   0U
#define SYSCTL_CLB7_AC_CPUX_ACC_M   0x3U   // CPUx Access conditions to peripheral
#define SYSCTL_CLB7_AC_CLA1_ACC_S   2U
#define SYSCTL_CLB7_AC_CLA1_ACC_M   0xCU   // CLA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB8_AC register
//
//*************************************************************************************************
#define SYSCTL_CLB8_AC_CPUX_ACC_S   0U
#define SYSCTL_CLB8_AC_CPUX_ACC_M   0x3U   // CPUx Access conditions to peripheral
#define SYSCTL_CLB8_AC_CLA1_ACC_S   2U
#define SYSCTL_CLB8_AC_CLA1_ACC_M   0xCU   // CLA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the SPIA_AC register
//
//*************************************************************************************************
#define SYSCTL_SPIA_AC_CPUX_ACC_S   0U
#define SYSCTL_SPIA_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_SPIA_AC_CLA1_ACC_S   2U
#define SYSCTL_SPIA_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_SPIA_AC_DMA1_ACC_S   4U
#define SYSCTL_SPIA_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the SPIB_AC register
//
//*************************************************************************************************
#define SYSCTL_SPIB_AC_CPUX_ACC_S   0U
#define SYSCTL_SPIB_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_SPIB_AC_CLA1_ACC_S   2U
#define SYSCTL_SPIB_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_SPIB_AC_DMA1_ACC_S   4U
#define SYSCTL_SPIB_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the SPIC_AC register
//
//*************************************************************************************************
#define SYSCTL_SPIC_AC_CPUX_ACC_S   0U
#define SYSCTL_SPIC_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_SPIC_AC_CLA1_ACC_S   2U
#define SYSCTL_SPIC_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_SPIC_AC_DMA1_ACC_S   4U
#define SYSCTL_SPIC_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the SPID_AC register
//
//*************************************************************************************************
#define SYSCTL_SPID_AC_CPUX_ACC_S   0U
#define SYSCTL_SPID_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_SPID_AC_CLA1_ACC_S   2U
#define SYSCTL_SPID_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_SPID_AC_DMA1_ACC_S   4U
#define SYSCTL_SPID_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the PMBUS_A_AC register
//
//*************************************************************************************************
#define SYSCTL_PMBUS_A_AC_CPUX_ACC_S   0U
#define SYSCTL_PMBUS_A_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_PMBUS_A_AC_CLA1_ACC_S   2U
#define SYSCTL_PMBUS_A_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_PMBUS_A_AC_DMA1_ACC_S   4U
#define SYSCTL_PMBUS_A_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_A_AC register
//
//*************************************************************************************************
#define SYSCTL_CAN_A_AC_CPUX_ACC_S   0U
#define SYSCTL_CAN_A_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_CAN_A_AC_DMA1_ACC_S   4U
#define SYSCTL_CAN_A_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the CAN_B_AC register
//
//*************************************************************************************************
#define SYSCTL_CAN_B_AC_CPUX_ACC_S   0U
#define SYSCTL_CAN_B_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_CAN_B_AC_DMA1_ACC_S   4U
#define SYSCTL_CAN_B_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the MCBSPA_AC register
//
//*************************************************************************************************
#define SYSCTL_MCBSPA_AC_CPUX_ACC_S   0U
#define SYSCTL_MCBSPA_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_MCBSPA_AC_CLA1_ACC_S   2U
#define SYSCTL_MCBSPA_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_MCBSPA_AC_DMA1_ACC_S   4U
#define SYSCTL_MCBSPA_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the MCBSPB_AC register
//
//*************************************************************************************************
#define SYSCTL_MCBSPB_AC_CPUX_ACC_S   0U
#define SYSCTL_MCBSPB_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_MCBSPB_AC_CLA1_ACC_S   2U
#define SYSCTL_MCBSPB_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_MCBSPB_AC_DMA1_ACC_S   4U
#define SYSCTL_MCBSPB_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the USBA_AC register
//
//*************************************************************************************************
#define SYSCTL_USBA_AC_CPUX_ACC_S   0U
#define SYSCTL_USBA_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_USBA_AC_DMA1_ACC_S   4U
#define SYSCTL_USBA_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the HRPWM_AC register
//
//*************************************************************************************************
#define SYSCTL_HRPWM_AC_CPUX_ACC_S   0U
#define SYSCTL_HRPWM_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_HRPWM_AC_CLA1_ACC_S   2U
#define SYSCTL_HRPWM_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_HRPWM_AC_DMA1_ACC_S   4U
#define SYSCTL_HRPWM_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the ETHERCAT_AC register
//
//*************************************************************************************************
#define SYSCTL_ETHERCAT_AC_CPU1_ACC_S   0U
#define SYSCTL_ETHERCAT_AC_CPU1_ACC_M   0x3U    // CPU1 Access conditions to peripheral
#define SYSCTL_ETHERCAT_AC_DMA1_ACC_S   4U
#define SYSCTL_ETHERCAT_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the FSIATX_AC register
//
//*************************************************************************************************
#define SYSCTL_FSIATX_AC_CPUX_ACC_S   0U
#define SYSCTL_FSIATX_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_FSIATX_AC_CLA1_ACC_S   2U
#define SYSCTL_FSIATX_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_FSIATX_AC_DMA1_ACC_S   4U
#define SYSCTL_FSIATX_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the FSIARX_AC register
//
//*************************************************************************************************
#define SYSCTL_FSIARX_AC_CPUX_ACC_S   0U
#define SYSCTL_FSIARX_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_FSIARX_AC_CLA1_ACC_S   2U
#define SYSCTL_FSIARX_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_FSIARX_AC_DMA1_ACC_S   4U
#define SYSCTL_FSIARX_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the FSIBTX_AC register
//
//*************************************************************************************************
#define SYSCTL_FSIBTX_AC_CPUX_ACC_S   0U
#define SYSCTL_FSIBTX_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_FSIBTX_AC_CLA1_ACC_S   2U
#define SYSCTL_FSIBTX_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_FSIBTX_AC_DMA1_ACC_S   4U
#define SYSCTL_FSIBTX_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the FSIBRX_AC register
//
//*************************************************************************************************
#define SYSCTL_FSIBRX_AC_CPUX_ACC_S   0U
#define SYSCTL_FSIBRX_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_FSIBRX_AC_CLA1_ACC_S   2U
#define SYSCTL_FSIBRX_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_FSIBRX_AC_DMA1_ACC_S   4U
#define SYSCTL_FSIBRX_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the FSICRX_AC register
//
//*************************************************************************************************
#define SYSCTL_FSICRX_AC_CPUX_ACC_S   0U
#define SYSCTL_FSICRX_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_FSICRX_AC_CLA1_ACC_S   2U
#define SYSCTL_FSICRX_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_FSICRX_AC_DMA1_ACC_S   4U
#define SYSCTL_FSICRX_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the FSIDRX_AC register
//
//*************************************************************************************************
#define SYSCTL_FSIDRX_AC_CPUX_ACC_S   0U
#define SYSCTL_FSIDRX_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_FSIDRX_AC_CLA1_ACC_S   2U
#define SYSCTL_FSIDRX_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_FSIDRX_AC_DMA1_ACC_S   4U
#define SYSCTL_FSIDRX_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the FSIERX_AC register
//
//*************************************************************************************************
#define SYSCTL_FSIERX_AC_CPUX_ACC_S   0U
#define SYSCTL_FSIERX_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_FSIERX_AC_CLA1_ACC_S   2U
#define SYSCTL_FSIERX_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_FSIERX_AC_DMA1_ACC_S   4U
#define SYSCTL_FSIERX_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the FSIFRX_AC register
//
//*************************************************************************************************
#define SYSCTL_FSIFRX_AC_CPUX_ACC_S   0U
#define SYSCTL_FSIFRX_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_FSIFRX_AC_CLA1_ACC_S   2U
#define SYSCTL_FSIFRX_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_FSIFRX_AC_DMA1_ACC_S   4U
#define SYSCTL_FSIFRX_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the FSIGRX_AC register
//
//*************************************************************************************************
#define SYSCTL_FSIGRX_AC_CPUX_ACC_S   0U
#define SYSCTL_FSIGRX_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_FSIGRX_AC_CLA1_ACC_S   2U
#define SYSCTL_FSIGRX_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_FSIGRX_AC_DMA1_ACC_S   4U
#define SYSCTL_FSIGRX_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the FSIHRX_AC register
//
//*************************************************************************************************
#define SYSCTL_FSIHRX_AC_CPUX_ACC_S   0U
#define SYSCTL_FSIHRX_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_FSIHRX_AC_CLA1_ACC_S   2U
#define SYSCTL_FSIHRX_AC_CLA1_ACC_M   0xCU    // CLA1 Access Conditions to Peripheral
#define SYSCTL_FSIHRX_AC_DMA1_ACC_S   4U
#define SYSCTL_FSIHRX_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the MCANA_AC register
//
//*************************************************************************************************
#define SYSCTL_MCANA_AC_CPUX_ACC_S   0U
#define SYSCTL_MCANA_AC_CPUX_ACC_M   0x3U    // CPUx Access conditions to peripheral
#define SYSCTL_MCANA_AC_DMA1_ACC_S   4U
#define SYSCTL_MCANA_AC_DMA1_ACC_M   0x30U   // DMA1 Access Conditions to Peripheral

//*************************************************************************************************
//
// The following are defines for the bit fields in the PERIPH_AC_LOCK register
//
//*************************************************************************************************
#define SYSCTL_PERIPH_AC_LOCK_LOCK_AC_WR   0x1U   // Lock control for Access control registers
                                                  // write.


//*************************************************************************************************
//
// The following are defines for the bit fields in the CMRESCTL register
//
//*************************************************************************************************
#define SYSCTL_CMRESCTL_RESET      0x1U          // Software reset to CM
#define SYSCTL_CMRESCTL_RESETSTS   0x2U          // CM Reset status
#define SYSCTL_CMRESCTL_KEY_S      16U
#define SYSCTL_CMRESCTL_KEY_M      0xFFFF0000U   // Key value

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMTOCPU1NMICTL register
//
//*************************************************************************************************
#define SYSCTL_CMTOCPU1NMICTL_CMNMIWDRST   0x4U   // CMNMIWDRST NMI enable bit, enables nmi
                                                  // generation to C28x

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMTOCPU1INTCTL register
//
//*************************************************************************************************
#define SYSCTL_CMTOCPU1INTCTL_VECTRESET     0x1U   // VECTRESET Interrupt enable bit, enables
                                                   // interrupt to C28x
#define SYSCTL_CMTOCPU1INTCTL_SYSRESETREQ   0x2U   // SYSRESETREQ Interrupt enable bit, enables
                                                   // interrupt to C28x
#define SYSCTL_CMTOCPU1INTCTL_CMNMIWDRST    0x4U   // CMNMIWDRST Interrupt enable bit, enables
                                                   // interrupt to C28x

//*************************************************************************************************
//
// The following are defines for the bit fields in the PALLOCATE0 register
//
//*************************************************************************************************
#define SYSCTL_PALLOCATE0_USB_A      0x1U    // Allocate USB_A to CM
#define SYSCTL_PALLOCATE0_ETHERCAT   0x2U    // Allocate ETHERCAT to CM
#define SYSCTL_PALLOCATE0_CAN_A      0x4U    // Allocate CANA to CM
#define SYSCTL_PALLOCATE0_CAN_B      0x8U    // Allocate CANB to CM
#define SYSCTL_PALLOCATE0_MCAN_A     0x10U   // Allocate MCANA to CM

//*************************************************************************************************
//
// The following are defines for the bit fields in the CM_CONF_REGS_LOCK register
//
//*************************************************************************************************
#define SYSCTL_CM_CONF_REGS_LOCK_LOCK   0x1U   // Lock one time CM configuration registers.


//*************************************************************************************************
//
// The following are defines for the bit fields in the CM_STATUS_INT_FLG register
//
//*************************************************************************************************
#define SYSCTL_CM_STATUS_INT_FLG_GINT            0x1U   // Global Interrupt flag
#define SYSCTL_CM_STATUS_INT_FLG_CMNMIWDRST      0x2U   // CMNMIWDRST caused a reset of CM
#define SYSCTL_CM_STATUS_INT_FLG_CMSYSRESETREQ   0x4U   // CMSYSRESETREQ caused a reset of CM
#define SYSCTL_CM_STATUS_INT_FLG_CMVECTRESET     0x8U   // CMVECTRESET caused a reset of CM

//*************************************************************************************************
//
// The following are defines for the bit fields in the CM_STATUS_INT_CLR register
//
//*************************************************************************************************
#define SYSCTL_CM_STATUS_INT_CLR_GINT            0x1U   // Global Interrupt flag Clear bit
#define SYSCTL_CM_STATUS_INT_CLR_CMNMIWDRST      0x2U   // CMNMIWDRST interrupt flag clear bit
#define SYSCTL_CM_STATUS_INT_CLR_CMSYSRESETREQ   0x4U   // CMSYSRESETREQ interrupt flag clear bit
#define SYSCTL_CM_STATUS_INT_CLR_CMVECTRESET     0x8U   // CMVECTRESET interrupt flag clear bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CM_STATUS_INT_SET register
//
//*************************************************************************************************
#define SYSCTL_CM_STATUS_INT_SET_CMNMIWDRST      0x2U          // CMNMIWDRST interrupt flag set bit
#define SYSCTL_CM_STATUS_INT_SET_CMSYSRESETREQ   0x4U          // CMSYSRESETREQ interrupt flag set
                                                               // bit
#define SYSCTL_CM_STATUS_INT_SET_CMVECTRESET     0x8U          // CMVECTRESET interrupt flag set
                                                               // bit
#define SYSCTL_CM_STATUS_INT_SET_KEY_S           16U
#define SYSCTL_CM_STATUS_INT_SET_KEY_M           0xFFFF0000U   // KEY field

//*************************************************************************************************
//
// The following are defines for the bit fields in the CM_STATUS_MASK register
//
//*************************************************************************************************
#define SYSCTL_CM_STATUS_MASK_CMNMIWDRST      0x2U          // CMNMIWDRST flag mask bit
#define SYSCTL_CM_STATUS_MASK_CMSYSRESETREQ   0x4U          // CMSYSRESETREQ interrupt flag set bit
#define SYSCTL_CM_STATUS_MASK_CMVECTRESET     0x8U          // CMVECTRESET interrupt flag set bit
#define SYSCTL_CM_STATUS_MASK_KEY_S           16U
#define SYSCTL_CM_STATUS_MASK_KEY_M           0xFFFF0000U   // KEY field

//*************************************************************************************************
//
// The following are defines for the bit fields in the SYS_ERR_INT_FLG register
//
//*************************************************************************************************
#define SYSCTL_SYS_ERR_INT_FLG_GINT                        0x1U     // Global Interrupt flag
#define SYSCTL_SYS_ERR_INT_FLG_EMIF_ERR                    0x2U     // EMIF error event flag
#define SYSCTL_SYS_ERR_INT_FLG_RAM_CORRECTABLE_ERR         0x4U     // RAM correctable error flag
#define SYSCTL_SYS_ERR_INT_FLG_FLASH_CORRECTABLE_ERR       0x8U     // FLASH correctable error flag
#define SYSCTL_SYS_ERR_INT_FLG_RAM_ACC_VIOL                0x10U    // RAM access vioation flag.
#define SYSCTL_SYS_ERR_INT_FLG_SYS_PLL_SLIP_NOTSUPPORTED   0x20U    // System PLL Slip event flag.
#define SYSCTL_SYS_ERR_INT_FLG_AUX_PLL_SLIP_NOTSUPPORTED   0x40U    // Auxillary PLL Slip event
                                                                    // flag.
#define SYSCTL_SYS_ERR_INT_FLG_DCC0                        0x80U    // DCC0 Interrupt flag.
#define SYSCTL_SYS_ERR_INT_FLG_DCC1                        0x100U   // DCC1 Interrupt flag.
#define SYSCTL_SYS_ERR_INT_FLG_DCC2                        0x200U   // DCC2 Interrupt flag.

//*************************************************************************************************
//
// The following are defines for the bit fields in the SYS_ERR_INT_CLR register
//
//*************************************************************************************************
#define SYSCTL_SYS_ERR_INT_CLR_GINT                        0x1U     // Global Interrupt flag Clear
                                                                    // bit
#define SYSCTL_SYS_ERR_INT_CLR_EMIF_ERR                    0x2U     // EMIF_ERR interrupt flag
                                                                    // clear bit
#define SYSCTL_SYS_ERR_INT_CLR_RAM_CORRECTABLE_ERR         0x4U     // RAM_CORRECTABLE_ERR
                                                                    // interrupt flag clear bit
#define SYSCTL_SYS_ERR_INT_CLR_FLASH_CORRECTABLE_ERR       0x8U     // FLASH_CORRECTABLE_ERR
                                                                    // interrupt flag clear bit
#define SYSCTL_SYS_ERR_INT_CLR_RAM_ACC_VIOL                0x10U    // RAM_ACC_VIOL interrupt flag
                                                                    // clear bit
#define SYSCTL_SYS_ERR_INT_CLR_SYS_PLL_SLIP_NOTSUPPORTED   0x20U    // SYS_PLL_SLIP interrupt flag
                                                                    // clear bit
#define SYSCTL_SYS_ERR_INT_CLR_AUX_PLL_SLIP_NOTSUPPORTED   0x40U    // AUX_PLL_SLIP interrupt flag
                                                                    // clear bit
#define SYSCTL_SYS_ERR_INT_CLR_DCC0                        0x80U    // DCC0 interrupt flag clear
                                                                    // bit
#define SYSCTL_SYS_ERR_INT_CLR_DCC1                        0x100U   // DCC1 interrupt flag clear
                                                                    // bit
#define SYSCTL_SYS_ERR_INT_CLR_DCC2                        0x200U   // DCC2 interrupt flag clear
                                                                    // bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the SYS_ERR_INT_SET register
//
//*************************************************************************************************
#define SYSCTL_SYS_ERR_INT_SET_EMIF_ERR                    0x2U          // Reserved
#define SYSCTL_SYS_ERR_INT_SET_RAM_CORRECTABLE_ERR         0x4U          // RAM_CORRECTABLE_ERR
                                                                         // interrupt flag set bit
#define SYSCTL_SYS_ERR_INT_SET_FLASH_CORRECTABLE_ERR       0x8U          // FLASH_CORRECTABLE_ERR
                                                                         // interrupt flag set bit
#define SYSCTL_SYS_ERR_INT_SET_RAM_ACC_VIOL                0x10U         // RAM_ACC_VIOL interrupt
                                                                         // flag set bit
#define SYSCTL_SYS_ERR_INT_SET_SYS_PLL_SLIP_NOTSUPPORTED   0x20U         // SYS_PLL_SLIP interrupt
                                                                         // flag set bit
#define SYSCTL_SYS_ERR_INT_SET_AUX_PLL_SLIP_NOTSUPPORTED   0x40U         // AUX_PLL_SLIP interrupt
                                                                         // flag set bit
#define SYSCTL_SYS_ERR_INT_SET_DCC0                        0x80U         // DCC0 interrupt flag set
                                                                         // bit
#define SYSCTL_SYS_ERR_INT_SET_DCC1                        0x100U        // DCC1 interrupt flag set
                                                                         // bit
#define SYSCTL_SYS_ERR_INT_SET_DCC2                        0x200U        // DCC2 interrupt flag set
                                                                         // bit
#define SYSCTL_SYS_ERR_INT_SET_KEY_S                       16U
#define SYSCTL_SYS_ERR_INT_SET_KEY_M                       0xFFFF0000U   // KEY field

//*************************************************************************************************
//
// The following are defines for the bit fields in the SYS_ERR_MASK register
//
//*************************************************************************************************
#define SYSCTL_SYS_ERR_MASK_EMIF_ERR                0x2U          // Reserved
#define SYSCTL_SYS_ERR_MASK_RAM_CORRECTABLE_ERR     0x4U          // RAM_CORRECTABLE_ERR flag mask
                                                                  // bit
#define SYSCTL_SYS_ERR_MASK_FLASH_CORRECTABLE_ERR   0x8U          // FLASH_CORRECTABLE_ERR flag
                                                                  // mask bit
#define SYSCTL_SYS_ERR_MASK_RAM_ACC_VIOL            0x10U         // RAM_ACC_VIOL flag mask bit
#define SYSCTL_SYS_ERR_MASK_SYS_PLL_SLIP            0x20U         // SYS_PLL_SLIP flag mask bit
#define SYSCTL_SYS_ERR_MASK_AUX_PLL_SLIP            0x40U         // AUX_PLL_SLIP flag mask bit
#define SYSCTL_SYS_ERR_MASK_DCC0                    0x80U         // DCC0 flag mask bit
#define SYSCTL_SYS_ERR_MASK_DCC1                    0x100U        // DCC1 flag mask bit
#define SYSCTL_SYS_ERR_MASK_DCC2                    0x200U        // DCC2 flag mask bit
#define SYSCTL_SYS_ERR_MASK_KEY_S                   16U
#define SYSCTL_SYS_ERR_MASK_KEY_M                   0xFFFF0000U   // KEY field


//*************************************************************************************************
//
// The following are defines for the bit fields in the SYNCSELECT register
//
//*************************************************************************************************
#define SYSCTL_SYNCSELECT_SYNCOUT_S   24U
#define SYSCTL_SYNCSELECT_SYNCOUT_M   0x1F000000U   // Select Syncout Source

//*************************************************************************************************
//
// The following are defines for the bit fields in the ADCSOCOUTSELECT register
//
//*************************************************************************************************
#define SYSCTL_ADCSOCOUTSELECT_PWM1SOCAEN    0x1U          // PWM1SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM2SOCAEN    0x2U          // PWM2SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM3SOCAEN    0x4U          // PWM3SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM4SOCAEN    0x8U          // PWM4SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM5SOCAEN    0x10U         // PWM5SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM6SOCAEN    0x20U         // PWM6SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM7SOCAEN    0x40U         // PWM7SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM8SOCAEN    0x80U         // PWM8SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM9SOCAEN    0x100U        // PWM9SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM10SOCAEN   0x200U        // PWM10SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM11SOCAEN   0x400U        // PWM11SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM12SOCAEN   0x800U        // PWM12SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM13SOCAEN   0x1000U       // PWM13SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM14SOCAEN   0x2000U       // PWM14SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM15SOCAEN   0x4000U       // PWM15SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM16SOCAEN   0x8000U       // PWM16SOCAEN Enable for ADCSOCAOn
#define SYSCTL_ADCSOCOUTSELECT_PWM1SOCBEN    0x10000U      // PWM1SOCBEN Enable for ADCSOCBOn
#define SYSCTL_ADCSOCOUTSELECT_PWM2SOCBEN    0x20000U      // PWM2SOCBEN Enable for ADCSOCBOn
#define SYSCTL_ADCSOCOUTSELECT_PWM3SOCBEN    0x40000U      // PWM3SOCBEN Enable for ADCSOCBOn
#define SYSCTL_ADCSOCOUTSELECT_PWM4SOCBEN    0x80000U      // PWM4SOCBEN Enable for ADCSOCBOn
#define SYSCTL_ADCSOCOUTSELECT_PWM5SOCBEN    0x100000U     // PWM5SOCBEN Enable for ADCSOCBOn
#define SYSCTL_ADCSOCOUTSELECT_PWM6SOCBEN    0x200000U     // PWM6SOCBEN Enable for ADCSOCBOn
#define SYSCTL_ADCSOCOUTSELECT_PWM7SOCBEN    0x400000U     // PWM7SOCBEN Enable for ADCSOCBOn
#define SYSCTL_ADCSOCOUTSELECT_PWM8SOCBEN    0x800000U     // PWM8SOCBEN Enable for ADCSOCBOn
#define SYSCTL_ADCSOCOUTSELECT_PWM9SOCBEN    0x1000000U    // PWM9SOCBEN Enable for ADCSOCBOn
#define SYSCTL_ADCSOCOUTSELECT_PWM10SOCBEN   0x2000000U    // PWM10SOCBEN Enable for ADCSOCBOn
#define SYSCTL_ADCSOCOUTSELECT_PWM11SOCBEN   0x4000000U    // PWM11SOCBEN Enable for ADCSOCBOn
#define SYSCTL_ADCSOCOUTSELECT_PWM12SOCBEN   0x8000000U    // PWM12SOCBEN Enable for ADCSOCBOn
#define SYSCTL_ADCSOCOUTSELECT_PWM13SOCBEN   0x10000000U   // PWM13SOCBEN Enable for ADCSOCBOn
#define SYSCTL_ADCSOCOUTSELECT_PWM14SOCBEN   0x20000000U   // PWM14SOCBEN Enable for ADCSOCBOn
#define SYSCTL_ADCSOCOUTSELECT_PWM15SOCBEN   0x40000000U   // PWM15SOCBEN Enable for ADCSOCBOn
#define SYSCTL_ADCSOCOUTSELECT_PWM16SOCBEN   0x80000000U   // PWM16SOCBEN Enable for ADCSOCBOn

//*************************************************************************************************
//
// The following are defines for the bit fields in the SYNCSOCLOCK register
//
//*************************************************************************************************
#define SYSCTL_SYNCSOCLOCK_SYNCSELECT        0x1U   // SYNCSEL Register Lock bit
#define SYSCTL_SYNCSOCLOCK_ADCSOCOUTSELECT   0x2U   // ADCSOCOUTSELECT Register Lock bit



#endif
