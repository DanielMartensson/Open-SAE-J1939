//#############################################################################
//
// FILE:   device.h
//
// TITLE:  Device setup for examples.
//
//#############################################################################
//
//
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
//#############################################################################

#ifndef __DEVICE_H__
#define __DEVICE_H__

#ifdef __cplusplus
extern "C"
{
#endif

//
// Included Files
//

#include "driverlib.h"

#if (!defined(CPU1) && !defined(CPU2))
#error "You must define CPU1 or CPU2 in your project properties.  Otherwise, \
the offsets in your header files will be inaccurate."
#endif

#if (defined(CPU1) && defined(CPU2))
#error "You have defined both CPU1 and CPU2 in your project properties.  Only \
a single CPU should be defined."
#endif

//
// Check for invalid compile flags
//
#if defined(__TMS320C28XX_TMU1__)
#error "Invalid TMU Configuration"
#endif

//*****************************************************************************
//
// Defines for pin numbers and other GPIO configuration
//
//*****************************************************************************
//
// LEDs
//
#define DEVICE_GPIO_PIN_LED1        31U   // GPIO number for LD1
#define DEVICE_GPIO_PIN_LED2        34U   // GPIO number for LD2
#define DEVICE_GPIO_PIN_LED3        145U  // GPIO number for LD7 (EtherCAT Error LED)
#define DEVICE_GPIO_PIN_LED4        146U  // GPIO number for LD8 (EtherCAT Run LED)
#define DEVICE_GPIO_CFG_LED1        GPIO_31_GPIO31    // "pinConfig" for LD1
#define DEVICE_GPIO_CFG_LED2        GPIO_34_GPIO34    // "pinConfig" for LD2
#define DEVICE_GPIO_CFG_LED3        GPIO_145_GPIO145  // "pinConfig" for LD7
#define DEVICE_GPIO_CFG_LED4        GPIO_146_GPIO146  // "pinConfig" for LD8


//
// SCI for USB-to-UART adapter on FTDI chip
//
#define DEVICE_GPIO_PIN_SCIRXDA     28U             // GPIO number for SCI RX
#define DEVICE_GPIO_PIN_SCITXDA     29U             // GPIO number for SCI TX
#define DEVICE_GPIO_CFG_SCIRXDA     GPIO_28_SCIA_RX // "pinConfig" for SCI RX
#define DEVICE_GPIO_CFG_SCITXDA     GPIO_29_SCIA_TX // "pinConfig" for SCI TX

//
// CANA
//
#define DEVICE_GPIO_PIN_CANTXA      37U  // GPIO number for CANTXA
#define DEVICE_GPIO_PIN_CANRXA      36U  // GPIO number for CANRXA

//
// CAN External Loopback
//
#define DEVICE_GPIO_CFG_CANRXA      GPIO_36_CANA_RX  // "pinConfig" for CANA RX
#define DEVICE_GPIO_CFG_CANTXA      GPIO_37_CANA_TX  // "pinConfig" for CANA TX
#define DEVICE_GPIO_CFG_CANRXB      GPIO_10_CANB_RX  // "pinConfig" for CANB RX
#define DEVICE_GPIO_CFG_CANTXB      GPIO_12_CANB_TX  // "pinConfig" for CANB TX

//
// MCAN
//
#define DEVICE_GPIO_CFG_MCANRXA    GPIO_30_MCAN_RX  // "pinConfig" for MCAN RX
#define DEVICE_GPIO_CFG_MCANTXA    GPIO_31_MCAN_TX  // "pinConfig" for MCAN TX

//
// FSI
//
#define DEVICE_GPIO_PIN_FSI_RXCLKA      9U  // GPIO number for FSI RXCLKA
#define DEVICE_GPIO_PIN_FSI_RX0A        8U  // GPIO number for FSI RX0A
#define DEVICE_GPIO_PIN_FSI_RX1A        10U  // GPIO number for FSI RX1A

#define DEVICE_GPIO_PIN_FSI_RXCLKB      60U  // GPIO number for FSI RXCLKB
#define DEVICE_GPIO_PIN_FSI_RX0B        58U  // GPIO number for FSI RX0B
#define DEVICE_GPIO_PIN_FSI_RX1B        59U  // GPIO number for FSI RX1B

#define DEVICE_GPIO_PIN_FSI_RXCLKC      14U  // GPIO number for FSI RXCLKC
#define DEVICE_GPIO_PIN_FSI_RX0C        12U  // GPIO number for FSI RX0C
#define DEVICE_GPIO_PIN_FSI_RX1C        13U  // GPIO number for FSI RX1C


#define DEVICE_GPIO_CFG_FSI_TXCLK       GPIO_27_FSITXA_CLK  // "pinConfig" for FSI TXCLK
#define DEVICE_GPIO_CFG_FSI_TX0         GPIO_26_FSITXA_D0   // "pinConfig" for FSI TX0
#define DEVICE_GPIO_CFG_FSI_TX1         GPIO_25_FSITXA_D1   // "pinConfig" for FSI TX1

#define DEVICE_GPIO_CFG_FSI_RXCLKA      GPIO_9_FSIRXA_CLK   // "pinConfig" for FSI RXCLKA
#define DEVICE_GPIO_CFG_FSI_RX0A        GPIO_8_FSIRXA_D0    // "pinConfig" for FSI RX0A
#define DEVICE_GPIO_CFG_FSI_RX1A        GPIO_10_FSIRXA_D1   // "pinConfig" for FSI RX1A

#define DEVICE_GPIO_CFG_FSI_RXCLKB      GPIO_60_FSIRXB_CLK  // "pinConfig" for FSI RXCLKB
#define DEVICE_GPIO_CFG_FSI_RX0B        GPIO_58_FSIRXB_D0   // "pinConfig" for FSI RX0B
#define DEVICE_GPIO_CFG_FSI_RX1B        GPIO_59_FSIRXB_D1   // "pinConfig" for FSI RX1B

#define DEVICE_GPIO_CFG_FSI_RXCLKC      GPIO_14_FSIRXC_CLK  // "pinConfig" for FSI RXCLKC
#define DEVICE_GPIO_CFG_FSI_RX0C        GPIO_12_FSIRXC_D0   // "pinConfig" for FSI RX0C
#define DEVICE_GPIO_CFG_FSI_RX1C        GPIO_13_FSIRXC_D1   // "pinConfig" for FSI RX1C

//!  --------------------------------
//!    Signal   |  I2CA   |  I2CB
//!  --------------------------------
//!     SCL     | GPIO105 |  GPIO41
//!     SDA     | GPIO104 |  GPIO40
//!  --------------------------------
//I2CA GPIO pins
#define DEVICE_GPIO_PIN_SDAA    104
#define DEVICE_GPIO_PIN_SCLA    105

//I2CB GPIO pins
#define DEVICE_GPIO_PIN_SDAB    40
#define DEVICE_GPIO_PIN_SCLB    41

#define DEVICE_GPIO_CFG_SDAA GPIO_104_I2CA_SDA
#define DEVICE_GPIO_CFG_SCLA GPIO_105_I2CA_SCL

#define DEVICE_GPIO_CFG_SDAB GPIO_40_I2CB_SDA
#define DEVICE_GPIO_CFG_SCLB GPIO_41_I2CB_SCL


//*****************************************************************************
//
// Defines related to clock configuration
//
//*****************************************************************************
#ifdef USE_20MHZ_XTAL

//
// 20MHz XTAL on controlCARD. For use with SysCtl_getClock() and
// SysCtl_getAuxClock().
//
#define DEVICE_OSCSRC_FREQ          20000000U

//
// Define to pass to SysCtl_setClock(). Will configure the clock as follows:
// PLLSYSCLK = 20MHz (XTAL_OSC) * 40 (IMULT) / (2 (REFDIV) * 2 (ODIV) * 1(SYSDIV))
//
#define DEVICE_SETCLOCK_CFG          (SYSCTL_OSCSRC_XTAL | SYSCTL_IMULT(40) | \
                                      SYSCTL_REFDIV(2) | SYSCTL_ODIV(2) | \
                                      SYSCTL_SYSDIV(1) | SYSCTL_PLL_ENABLE | \
                                      SYSCTL_DCC_BASE_1)

//
// 200MHz SYSCLK frequency based on the above DEVICE_SETCLOCK_CFG. Update the
// code below if a different clock configuration is used!
//
#define DEVICE_SYSCLK_FREQ          ((DEVICE_OSCSRC_FREQ * 40) / (2 * 2 * 1))

//
// 50MHz LSPCLK frequency based on the above DEVICE_SYSCLK_FREQ and a default
// low speed peripheral clock divider of 4. Update the code below if a
// different LSPCLK divider is used!
//
#define DEVICE_LSPCLK_FREQ          (DEVICE_SYSCLK_FREQ / 4)

//
// Define to pass to SysCtl_setAuxClock(). Will configure the clock as follows:
// AUXPLLCLK = 20MHz (XTAL_OSC) * 50 (IMULT) / (2 (REFDIV) * 4 (ODIV) * 1(AUXPLLDIV) )
//
#define DEVICE_AUXSETCLOCK_CFG       (SYSCTL_AUXPLL_OSCSRC_XTAL | SYSCTL_AUXPLL_IMULT(50) |  \
                                      SYSCTL_REFDIV(2U) | SYSCTL_ODIV(4U) | \
                                      SYSCTL_AUXPLL_DIV_1 | SYSCTL_AUXPLL_ENABLE | \
                                      SYSCTL_DCC_BASE_0)

//
// 125MHz AUXCLK frequency based on the above DEVICE_AUXSETCLOCK_CFG. Update
// the code below if a different clock configuration is used!
//
#define DEVICE_AUXCLK_FREQ          ((DEVICE_OSCSRC_FREQ * 50) / (2 * 4 * 1))


#else // USE_25MHZ_XTAL

//
// 25MHz XTAL on controlCARD. For use with SysCtl_getClock() and
// SysCtl_getAuxClock().
//
#define DEVICE_OSCSRC_FREQ          25000000U

//
// Define to pass to SysCtl_setClock(). Will configure the clock as follows:
// PLLSYSCLK = 25MHz (XTAL_OSC) * 32 (IMULT) / (2 (REFDIV) * 2 (ODIV) * 1(SYSDIV))
//
#define DEVICE_SETCLOCK_CFG          (SYSCTL_OSCSRC_XTAL | SYSCTL_IMULT(32) | \
                                      SYSCTL_REFDIV(2) | SYSCTL_ODIV(2) | \
                                      SYSCTL_SYSDIV(1) | SYSCTL_PLL_ENABLE | \
                                      SYSCTL_DCC_BASE_1)

//
// 200MHz SYSCLK frequency based on the above DEVICE_SETCLOCK_CFG. Update the
// code below if a different clock configuration is used!
//
#define DEVICE_SYSCLK_FREQ          ((DEVICE_OSCSRC_FREQ * 32) / (2 * 2 * 1))

//
// 50MHz LSPCLK frequency based on the above DEVICE_SYSCLK_FREQ and a default
// low speed peripheral clock divider of 4. Update the code below if a
// different LSPCLK divider is used!
//
#define DEVICE_LSPCLK_FREQ          (DEVICE_SYSCLK_FREQ / 4)

//
// Define to pass to SysCtl_setAuxClock(). Will configure the clock as follows:
// AUXPLLCLK = 25MHz (XTAL_OSC) * 40 (IMULT) / (2 (REFDIV) * 4 (ODIV) * 1(AUXPLLDIV) )
//
#define DEVICE_AUXSETCLOCK_CFG       (SYSCTL_AUXPLL_OSCSRC_XTAL | SYSCTL_AUXPLL_IMULT(40) |  \
                                      SYSCTL_REFDIV(2U) | SYSCTL_ODIV(4U) | \
                                      SYSCTL_AUXPLL_DIV_1 | SYSCTL_AUXPLL_ENABLE | \
                                      SYSCTL_DCC_BASE_0)

//
// 125MHz AUXCLK frequency based on the above DEVICE_AUXSETCLOCK_CFG. Update
// the code below if a different clock configuration is used!
//
#define DEVICE_AUXCLK_FREQ          ((DEVICE_OSCSRC_FREQ * 40) / (2 * 4 * 1))

#endif

//*****************************************************************************
//
// Macro to call SysCtl_delay() to achieve a delay in microseconds. The macro
// will convert the desired delay in microseconds to the count value expected
// by the function. \b x is the number of microseconds to delay.
//
//*****************************************************************************
#define DEVICE_DELAY_US(x) SysCtl_delay(((((long double)(x)) / (1000000.0L /  \
                              (long double)DEVICE_SYSCLK_FREQ)) - 9.0L) / 5.0L)

//
//  Defines for setting FSI clock speeds
//
#define FSI_PRESCALE_50MHZ          2U
#define FSI_PRESCALE_25MHZ          4U
#define FSI_PRESCALE_10MHZ          10U
#define FSI_PRESCALE_5MHZ           20U
							  
//*****************************************************************************
//
// Macros related to booting CPU2 and CM. These can be used while invoking the
// functions Device_bootCPU2() and Device_bootCM(). 
//
// Note that the macros CM_BOOT_FREQ_125MHZ and CPU2_BOOT_FREQ_200MHZ are used
// in the functions Device_bootCM() and Device_bootCPU2() respectively. Please
// update the function and the macros if you are using a different clock
// frequency.
//
//*****************************************************************************
#ifdef CPU1
#define BOOT_KEY                                0x5A000000UL
#define CM_BOOT_FREQ_125MHZ                     0x7D00U
#define CPU2_BOOT_FREQ_200MHZ                   0xC800U

#define BOOTMODE_BOOT_TO_FLASH_SECTOR0          0x03U
#define BOOTMODE_BOOT_TO_FLASH_SECTOR4          0x23U
#define BOOTMODE_BOOT_TO_FLASH_SECTOR8          0x43U
#define BOOTMODE_BOOT_TO_FLASH_SECTOR13         0x63U
#define BOOTMODE_BOOT_TO_SECURE_FLASH_SECTOR0   0x0AU
#define BOOTMODE_BOOT_TO_SECURE_FLASH_SECTOR4   0x2AU
#define BOOTMODE_BOOT_TO_SECURE_FLASH_SECTOR8   0x4AU
#define BOOTMODE_BOOT_TO_SECURE_FLASH_SECTOR13  0x6AU
#define BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_M1RAM  0x0CU
#define BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_S0RAM  0x0CU
#define BOOTMODE_BOOT_TO_M0RAM                  0x05U
#define BOOTMODE_BOOT_TO_S0RAM                  0x05U
#define BOOTMODE_BOOT_TO_USEROTP                0x0BU

#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_100W    0x10000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_200W    0x20000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_300W    0x30000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_400W    0x40000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_500W    0x50000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_600W    0x60000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_700W    0x70000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_800W    0x80000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_900W    0x90000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_1000W   0xA0000U
#endif

//*****************************************************************************
//
// Defines, Globals, and Header Includes related to Flash Support
//
//*****************************************************************************
#ifdef _FLASH
#include <stddef.h>

#ifndef CMDTOOL
extern uint16_t RamfuncsLoadStart;
extern uint16_t RamfuncsLoadEnd;
extern uint16_t RamfuncsLoadSize;
extern uint16_t RamfuncsRunStart;
extern uint16_t RamfuncsRunEnd;
extern uint16_t RamfuncsRunSize;
#endif

#define DEVICE_FLASH_WAITSTATES 3

#endif

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************
//*****************************************************************************
//
//! \addtogroup device_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Function to initialize the device. Primarily initializes system control to a
//! known state by disabling the watchdog, setting up the SYSCLKOUT frequency,
//! and enabling the clocks to the peripherals.
//!
//! \param None.
//! \return None.
//
//*****************************************************************************
extern void Device_init(void);

#ifdef CPU1

//*****************************************************************************
//
//! @brief Function to boot CPU2.
//!
//! \param bootmode is the mode in which CPU2 should boot.
//!
//! Available bootmodes :
//!      - BOOTMODE_BOOT_TO_FLASH_SECTOR0
//!      - BOOTMODE_BOOT_TO_FLASH_SECTOR4
//!      - BOOTMODE_BOOT_TO_FLASH_SECTOR8
//!      - BOOTMODE_BOOT_TO_FLASH_SECTOR13
//!      - BOOTMODE_BOOT_TO_SECURE_FLASH_SECTOR0
//!      - BOOTMODE_BOOT_TO_SECURE_FLASH_SECTOR4
//!      - BOOTMODE_BOOT_TO_SECURE_FLASH_SECTOR8
//!      - BOOTMODE_BOOT_TO_SECURE_FLASH_SECTOR13
//!      - BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_M1RAM
//!      - BOOTMODE_BOOT_TO_M0RAM
//!      - BOOTMODE_BOOT_TO_USEROTP
//!
//! Note that while using BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_M1RAM,
//! BOOTMODE_IPC_MSGRAM_COPY_LENGTH_xxxW must be ORed with the bootmode parameter
//!
//! This function must be called after Device_init function
//! \return None.
//
//*****************************************************************************
extern void Device_bootCPU2(uint32_t bootmode);

//*****************************************************************************
//
//! @brief Function to boot CM
//!
//! \param bootmode is the mode in which CM should boot.
//!
//! Available bootmodes :
//!      - BOOTMODE_BOOT_TO_FLASH_SECTOR0
//!      - BOOTMODE_BOOT_TO_FLASH_SECTOR4
//!      - BOOTMODE_BOOT_TO_FLASH_SECTOR8
//!      - BOOTMODE_BOOT_TO_FLASH_SECTOR13
//!      - BOOTMODE_BOOT_TO_SECURE_FLASH_SECTOR0
//!      - BOOTMODE_BOOT_TO_SECURE_FLASH_SECTOR4
//!      - BOOTMODE_BOOT_TO_SECURE_FLASH_SECTOR8
//!      - BOOTMODE_BOOT_TO_SECURE_FLASH_SECTOR13
//!      - BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_S0RAM
//!      - BOOTMODE_BOOT_TO_S0RAM
//!      - BOOTMODE_BOOT_TO_USEROTP
//!
//! Note that while using BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_M1RAM,
//! BOOTMODE_IPC_MSGRAM_COPY_LENGTH_xxxW must be ORed with the bootmode parameter
//!
//! This function must be called after Device_init function
//! \return None.
//
//*****************************************************************************
extern void Device_bootCM(uint32_t bootmode);

//*****************************************************************************
//
//!
//! @brief Function to verify the XTAL frequency
//! \param freq is the XTAL frequency in MHz
//! \return The function return true if the the actual XTAL frequency matches with the
//! input value
//
//*****************************************************************************
extern bool Device_verifyXTAL(float freq);
#endif

//*****************************************************************************
//!
//!
//! @brief Function to turn on all peripherals, enabling reads and writes to the
//! peripherals' registers.
//!
//! Note that to reduce power, unused peripherals should be disabled.
//!
//! @param None
//! @return None
//
//*****************************************************************************
extern void Device_enableAllPeripherals(void);
//*****************************************************************************
//!
//!
//! @brief Function to disable pin locks on GPIOs.
//!
//! @param None
//! @return None
//
//*****************************************************************************
extern void Device_initGPIO(void);
//*****************************************************************************
//!
//! @brief Function to enable pullups for the unbonded GPIOs on the 176PTP package:
//! GPIOs     Grp Bits
//! 95-132    C   31
//!           D   31:0
//!           E   4:0
//! 134-168   E   31:6
//!           F   8:0
//!
//! @param None
//! @return None
//
//*****************************************************************************
extern void Device_enableUnbondedGPIOPullupsFor176Pin(void);
//*****************************************************************************
//!
//! @brief Function to enable pullups for the unbonded GPIOs on the
//! 176PTP package.
//!
//! @param None
//! @return None
//
//*****************************************************************************

extern void Device_enableUnbondedGPIOPullups(void);
//*****************************************************************************
//!
//! @brief Error handling function to be called when an ASSERT is violated
//!
//! @param *filename File name in which the error has occurred
//! @param line Line number within the file
//! @return None
//
//*****************************************************************************

extern void __error__(const char *filename, uint32_t line);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

#ifdef __cplusplus
}
#endif

#endif // __DEVICE_H__
