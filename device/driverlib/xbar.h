//###########################################################################
//
// FILE:   xbar.h
//
// TITLE:  C28x X-BAR driver.
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

#ifndef XBAR_H
#define XBAR_H

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
//! \addtogroup xbar_api XBAR
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_clbxbar.h"
#include "inc/hw_epwmxbar.h"
#include "inc/hw_inputxbar.h"
#include "inc/hw_outputxbar.h"
#include "inc/hw_xbar.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "cpu.h"
#include "debug.h"

//*****************************************************************************
//
// Useful defines used within the driver functions.
// Not intended for use by application code.
//
//*****************************************************************************
#define XBAR_EPWM_CFG_REG_BASE    (EPWMXBAR_BASE + XBAR_O_TRIP4MUX0TO15CFG)
#define XBAR_EPWM_EN_REG_BASE     (EPWMXBAR_BASE + XBAR_O_TRIP4MUXENABLE)
#define XBAR_CLB_CFG_REG_BASE     (CLBXBAR_BASE + XBAR_O_AUXSIG0MUX0TO15CFG)
#define XBAR_CLB_EN_REG_BASE      (CLBXBAR_BASE + XBAR_O_AUXSIG0MUXENABLE)

#define XBAR_INPUT_FLG_INPUT_M    0x00FFU
#define XBAR_INPUT_FLG_REG_M      0xFF00U
#define XBAR_INPUT_FLG_REG_1      0x0000U
#define XBAR_INPUT_FLG_REG_2      0x0100U
#define XBAR_INPUT_FLG_REG_3      0x0200U
#define XBAR_INPUT_FLG_REG_4      0x0300U

#define XBAR_GPIO_MAX_CNT                168U
#define XBAR_NON_GPIO_MIN_CNT            0xFFFDU
#define XBAR_NON_GPIO_MAX_CNT            0xFFFFU

#ifndef DOXYGEN_PDF_IGNORE
//*****************************************************************************
//
// The following values define the muxes parameter for XBAR_enableEPWMMux(),
// XBAR_enableOutputMux(), XBAR_disableEPWMMux(), and
// XBAR_disableOutputMux().
//
//*****************************************************************************
#define XBAR_MUX00                  0x00000001U //!< Mask for X-BAR mux 0
#define XBAR_MUX01                  0x00000002U //!< Mask for X-BAR mux 1
#define XBAR_MUX02                  0x00000004U //!< Mask for X-BAR mux 2
#define XBAR_MUX03                  0x00000008U //!< Mask for X-BAR mux 3
#define XBAR_MUX04                  0x00000010U //!< Mask for X-BAR mux 4
#define XBAR_MUX05                  0x00000020U //!< Mask for X-BAR mux 5
#define XBAR_MUX06                  0x00000040U //!< Mask for X-BAR mux 6
#define XBAR_MUX07                  0x00000080U //!< Mask for X-BAR mux 7
#define XBAR_MUX08                  0x00000100U //!< Mask for X-BAR mux 8
#define XBAR_MUX09                  0x00000200U //!< Mask for X-BAR mux 9
#define XBAR_MUX10                  0x00000400U //!< Mask for X-BAR mux 10
#define XBAR_MUX11                  0x00000800U //!< Mask for X-BAR mux 11
#define XBAR_MUX12                  0x00001000U //!< Mask for X-BAR mux 12
#define XBAR_MUX13                  0x00002000U //!< Mask for X-BAR mux 13
#define XBAR_MUX14                  0x00004000U //!< Mask for X-BAR mux 14
#define XBAR_MUX15                  0x00008000U //!< Mask for X-BAR mux 15
#define XBAR_MUX16                  0x00010000U //!< Mask for X-BAR mux 16
#define XBAR_MUX17                  0x00020000U //!< Mask for X-BAR mux 17
#define XBAR_MUX18                  0x00040000U //!< Mask for X-BAR mux 18
#define XBAR_MUX19                  0x00080000U //!< Mask for X-BAR mux 19
#define XBAR_MUX20                  0x00100000U //!< Mask for X-BAR mux 20
#define XBAR_MUX21                  0x00200000U //!< Mask for X-BAR mux 21
#define XBAR_MUX22                  0x00400000U //!< Mask for X-BAR mux 22
#define XBAR_MUX23                  0x00800000U //!< Mask for X-BAR mux 23
#define XBAR_MUX24                  0x01000000U //!< Mask for X-BAR mux 24
#define XBAR_MUX25                  0x02000000U //!< Mask for X-BAR mux 25
#define XBAR_MUX26                  0x04000000U //!< Mask for X-BAR mux 26
#define XBAR_MUX27                  0x08000000U //!< Mask for X-BAR mux 27
#define XBAR_MUX28                  0x10000000U //!< Mask for X-BAR mux 28
#define XBAR_MUX29                  0x20000000U //!< Mask for X-BAR mux 29
#define XBAR_MUX30                  0x40000000U //!< Mask for X-BAR mux 30
#define XBAR_MUX31                  0x80000000U //!< Mask for X-BAR mux 31
#endif

//*****************************************************************************
//
//! The following values define the \e output parameter for
//! XBAR_setOutputMuxConfig(), XBAR_enableOutputMux(), and
//! XBAR_disableOutputMux().
//
//*****************************************************************************
typedef enum
{
    XBAR_OUTPUT1 = 0,       //!< OUTPUT1 of the Output X-BAR
    XBAR_OUTPUT2 = 2,       //!< OUTPUT2 of the Output X-BAR
    XBAR_OUTPUT3 = 4,       //!< OUTPUT3 of the Output X-BAR
    XBAR_OUTPUT4 = 6,       //!< OUTPUT4 of the Output X-BAR
    XBAR_OUTPUT5 = 8,       //!< OUTPUT5 of the Output X-BAR
    XBAR_OUTPUT6 = 10,      //!< OUTPUT6 of the Output X-BAR
    XBAR_OUTPUT7 = 12,      //!< OUTPUT7 of the Output X-BAR
    XBAR_OUTPUT8 = 14,       //!< OUTPUT8 of the Output X-BAR
} XBAR_OutputNum;

//*****************************************************************************
//
//! The following values define the \e trip parameter for
//! XBAR_setEPWMMuxConfig(), XBAR_enableEPWMMux(), and XBAR_disableEPWMMux().
//
//*****************************************************************************
typedef enum
{
    XBAR_TRIP4  = 0,        //!< TRIP4 of the ePWM X-BAR
    XBAR_TRIP5  = 2,        //!< TRIP5 of the ePWM X-BAR
    XBAR_TRIP7  = 4,        //!< TRIP7 of the ePWM X-BAR
    XBAR_TRIP8  = 6,        //!< TRIP8 of the ePWM X-BAR
    XBAR_TRIP9  = 8,        //!< TRIP9 of the ePWM X-BAR
    XBAR_TRIP10 = 10,       //!< TRIP10 of the ePWM X-BAR
    XBAR_TRIP11 = 12,       //!< TRIP11 of the ePWM X-BAR
    XBAR_TRIP12 = 14        //!< TRIP12 of the ePWM X-BAR
} XBAR_TripNum;

//*****************************************************************************
//
// The following values define the trip parameter for XBAR_setCLBMuxConfig(),
// XBAR_enableCLBMux(), and XBAR_disableCLBMux().
//
//*****************************************************************************
typedef enum
{
    XBAR_AUXSIG0 = 0,
    XBAR_AUXSIG1 = 2,
    XBAR_AUXSIG2 = 4,
    XBAR_AUXSIG3 = 6,
    XBAR_AUXSIG4 = 8,
    XBAR_AUXSIG5 = 10,
    XBAR_AUXSIG6 = 12,
    XBAR_AUXSIG7 = 14
} XBAR_AuxSigNum;

//*****************************************************************************
//
//! The following values define the \e input parameter for XBAR_setInputPin().
//
//*****************************************************************************
typedef enum
{
    XBAR_INPUT1,            //!< ePWM[TZ1], ePWM[TRIP1], X-BARs, eCAPs
    XBAR_INPUT2,            //!< ePWM[TZ2], ePWM[TRIP2], X-BARs, eCAPs
    XBAR_INPUT3,            //!< ePWM[TZ3], ePWM[TRIP3], X-BARs, eCAPs
    XBAR_INPUT4,            //!< ADC wrappers, X-BARs, XINT1, eCAPs
    XBAR_INPUT5,            //!< EXTSYNCIN1, X-BARs, XINT2, eCAPs
    XBAR_INPUT6,            //!< EXTSYNCIN2, ePWM[TRIP6], X-BARs, XINT3, eCAPs
    XBAR_INPUT7,            //!< X-BARs, eCAPs
    XBAR_INPUT8,            //!< X-BARs, eCAPs
    XBAR_INPUT9,            //!< X-BARs, eCAPs
    XBAR_INPUT10,           //!< X-BARs, eCAPs
    XBAR_INPUT11,           //!< X-BARs, eCAPs
    XBAR_INPUT12,           //!< X-BARs, eCAPs
    XBAR_INPUT13,           //!< XINT4, X-BARs, eCAPs
    XBAR_INPUT14,           //!< XINT5, X-BARs, eCAPs
    XBAR_INPUT15,           //!< eCAPs
    XBAR_INPUT16            //!< eCAPs
} XBAR_InputNum;

#ifndef DOXYGEN_PDF_IGNORE
//*****************************************************************************
//
//! The following values define the \e muxConfig parameter for
//! XBAR_setOutputMuxConfig().
//
//*****************************************************************************
typedef enum
{
    //
    //OUTPUTXBAR
    //
    XBAR_OUT_MUX00_CMPSS1_CTRIPOUTH        = 0x0000,
    XBAR_OUT_MUX00_CMPSS1_CTRIPOUTH_OR_L   = 0x0001,
    XBAR_OUT_MUX00_ADCAEVT1                = 0x0002,
    XBAR_OUT_MUX00_ECAP1_OUT               = 0x0003,
    XBAR_OUT_MUX01_CMPSS1_CTRIPOUTL        = 0x0200,
    XBAR_OUT_MUX01_INPUTXBAR1              = 0x0201,
    XBAR_OUT_MUX01_CLB1_OUT4               = 0x0202,
    XBAR_OUT_MUX01_ADCCEVT1                = 0x0203,
    XBAR_OUT_MUX02_CMPSS2_CTRIPOUTH        = 0x0400,
    XBAR_OUT_MUX02_CMPSS2_CTRIPOUTH_OR_L   = 0x0401,
    XBAR_OUT_MUX02_ADCAEVT2                = 0x0402,
    XBAR_OUT_MUX02_ECAP2_OUT               = 0x0403,
    XBAR_OUT_MUX03_CMPSS2_CTRIPOUTL        = 0x0600,
    XBAR_OUT_MUX03_INPUTXBAR2              = 0x0601,
    XBAR_OUT_MUX03_CLB1_OUT5               = 0x0602,
    XBAR_OUT_MUX03_ADCCEVT2                = 0x0603,
    XBAR_OUT_MUX04_CMPSS3_CTRIPOUTH        = 0x0800,
    XBAR_OUT_MUX04_CMPSS3_CTRIPOUTH_OR_L   = 0x0801,
    XBAR_OUT_MUX04_ADCAEVT3                = 0x0802,
    XBAR_OUT_MUX04_ECAP3_OUT               = 0x0803,
    XBAR_OUT_MUX05_CMPSS3_CTRIPOUTL        = 0x0A00,
    XBAR_OUT_MUX05_INPUTXBAR3              = 0x0A01,
    XBAR_OUT_MUX05_CLB2_OUT4               = 0x0A02,
    XBAR_OUT_MUX05_ADCCEVT3                = 0x0A03,
    XBAR_OUT_MUX06_CMPSS4_CTRIPOUTH        = 0x0C00,
    XBAR_OUT_MUX06_CMPSS4_CTRIPOUTH_OR_L   = 0x0C01,
    XBAR_OUT_MUX06_ADCAEVT4                = 0x0C02,
    XBAR_OUT_MUX06_ECAP4_OUT               = 0x0C03,
    XBAR_OUT_MUX07_CMPSS4_CTRIPOUTL        = 0x0E00,
    XBAR_OUT_MUX07_INPUTXBAR4              = 0x0E01,
    XBAR_OUT_MUX07_CLB2_OUT5               = 0x0E02,
    XBAR_OUT_MUX07_ADCCEVT4                = 0x0E03,
    XBAR_OUT_MUX08_CMPSS5_CTRIPOUTH        = 0x1000,
    XBAR_OUT_MUX08_CMPSS5_CTRIPOUTH_OR_L   = 0x1001,
    XBAR_OUT_MUX08_ADCBEVT1                = 0x1002,
    XBAR_OUT_MUX08_ECAP5_OUT               = 0x1003,
    XBAR_OUT_MUX09_CMPSS5_CTRIPOUTL        = 0x1200,
    XBAR_OUT_MUX09_INPUTXBAR5              = 0x1201,
    XBAR_OUT_MUX09_CLB3_OUT4               = 0x1202,
    XBAR_OUT_MUX10_CMPSS6_CTRIPOUTH        = 0x1400,
    XBAR_OUT_MUX10_CMPSS6_CTRIPOUTH_OR_L   = 0x1401,
    XBAR_OUT_MUX10_ADCBEVT2                = 0x1402,
    XBAR_OUT_MUX10_ECAP6_OUT               = 0x1403,
    XBAR_OUT_MUX11_CMPSS6_CTRIPOUTL        = 0x1600,
    XBAR_OUT_MUX11_INPUTXBAR6              = 0x1601,
    XBAR_OUT_MUX11_CLB3_OUT5               = 0x1602,
    XBAR_OUT_MUX12_CMPSS7_CTRIPOUTH        = 0x1800,
    XBAR_OUT_MUX12_CMPSS7_CTRIPOUTH_OR_L   = 0x1801,
    XBAR_OUT_MUX12_ADCBEVT3                = 0x1802,
    XBAR_OUT_MUX12_ECAP7_OUT               = 0x1803,
    XBAR_OUT_MUX13_CMPSS7_CTRIPOUTL        = 0x1A00,
    XBAR_OUT_MUX13_ADCSOCA                 = 0x1A01,
    XBAR_OUT_MUX13_CLB4_OUT4               = 0x1A02,
    XBAR_OUT_MUX14_ADCBEVT4                = 0x1C02,
    XBAR_OUT_MUX14_EXTSYNCOUT              = 0x1C03,
    XBAR_OUT_MUX15_ADCSOCB                 = 0x1E01,
    XBAR_OUT_MUX15_CLB4_OUT5               = 0x1E02,
    XBAR_OUT_MUX16_SD1FLT1_COMPH           = 0x2000,
    XBAR_OUT_MUX16_SD1FLT1_COMPH_OR_COMPL  = 0x2001,
    XBAR_OUT_MUX17_SD1FLT1_COMPL           = 0x2200,
    XBAR_OUT_MUX17_CLB5_OUT4               = 0x2202,
    XBAR_OUT_MUX17_CLAHALT                 = 0x2203,
    XBAR_OUT_MUX18_SD1FLT2_COMPH           = 0x2400,
    XBAR_OUT_MUX18_SD1FLT2_COMPH_OR_COMPL  = 0x2401,
    XBAR_OUT_MUX19_SD1FLT2_COMPL           = 0x2600,
    XBAR_OUT_MUX19_CLB5_OUT5               = 0x2602,
    XBAR_OUT_MUX20_SD1FLT3_COMPH           = 0x2800,
    XBAR_OUT_MUX20_SD1FLT3_COMPH_OR_COMPL  = 0x2801,
    XBAR_OUT_MUX21_SD1FLT3_COMPL           = 0x2A00,
    XBAR_OUT_MUX21_CLB6_OUT4               = 0x2A02,
    XBAR_OUT_MUX22_SD1FLT4_COMPH           = 0x2C00,
    XBAR_OUT_MUX22_SD1FLT4_COMPH_OR_COMPL  = 0x2C01,
    XBAR_OUT_MUX23_SD1FLT4_COMPL           = 0x2E00,
    XBAR_OUT_MUX23_CLB6_OUT5               = 0x2E02,
    XBAR_OUT_MUX24_SD2FLT1_COMPH           = 0x3000,
    XBAR_OUT_MUX24_SD2FLT1_COMPH_OR_COMPL  = 0x3001,
    XBAR_OUT_MUX25_SD2FLT1_COMPL           = 0x3200,
    XBAR_OUT_MUX25_CLB7_OUT4               = 0x3203,
    XBAR_OUT_MUX26_SD2FLT2_COMPH           = 0x3400,
    XBAR_OUT_MUX26_SD2FLT2_COMPH_OR_COMPL  = 0x3401,
    XBAR_OUT_MUX27_SD2FLT2_COMPL           = 0x3600,
    XBAR_OUT_MUX27_ERRORSTS                = 0x3602,
    XBAR_OUT_MUX27_CLB7_OUT5               = 0x3603,
    XBAR_OUT_MUX28_SD2FLT3_COMPH           = 0x3800,
    XBAR_OUT_MUX28_SD2FLT3_COMPH_OR_COMPL  = 0x3801,
    XBAR_OUT_MUX28_XCLKOUT                 = 0x3802,
    XBAR_OUT_MUX29_SD2FLT3_COMPL           = 0x3A00,
    XBAR_OUT_MUX29_CLB8_OUT4               = 0x3A03,
    XBAR_OUT_MUX30_SD2FLT4_COMPH           = 0x3C00,
    XBAR_OUT_MUX30_SD2FLT4_COMPH_OR_COMPL  = 0x3C01,
    XBAR_OUT_MUX31_SD2FLT4_COMPL           = 0x3E00,
    XBAR_OUT_MUX31_CLB8_OUT5               = 0x3E03,

    //
    //CLBOUTPUTXBAR
    //
    XBAR_OUT_MUX00_CLB1_OUT0               = 0x0000,
    XBAR_OUT_MUX00_CLB5_OUT0               = 0x0001,
    XBAR_OUT_MUX01_CLB1_OUT1               = 0x0200,
    XBAR_OUT_MUX01_CLB5_OUT1               = 0x0201,
    XBAR_OUT_MUX02_CLB1_OUT2               = 0x0400,
    XBAR_OUT_MUX02_CLB5_OUT2               = 0x0401,
    XBAR_OUT_MUX03_CLB1_OUT3               = 0x0600,
    XBAR_OUT_MUX03_CLB5_OUT3               = 0x0601,
    XBAR_OUT_MUX04_CLB1_OUT4               = 0x0800,
    XBAR_OUT_MUX04_CLB5_OUT4               = 0x0801,
    XBAR_OUT_MUX05_CLB1_OUT5               = 0x0A00,
    XBAR_OUT_MUX05_CLB5_OUT5               = 0x0A01,
    XBAR_OUT_MUX06_CLB1_OUT6               = 0x0C00,
    XBAR_OUT_MUX06_CLB5_OUT6               = 0x0C01,
    XBAR_OUT_MUX07_CLB1_OUT7               = 0x0E00,
    XBAR_OUT_MUX07_CLB5_OUT7               = 0x0E01,
    XBAR_OUT_MUX08_CLB2_OUT0               = 0x1000,
    XBAR_OUT_MUX08_CLB6_OUT0               = 0x1001,
    XBAR_OUT_MUX09_CLB2_OUT1               = 0x1200,
    XBAR_OUT_MUX09_CLB6_OUT1               = 0x1201,
    XBAR_OUT_MUX10_CLB2_OUT2              = 0x1400,
    XBAR_OUT_MUX10_CLB6_OUT2              = 0x1401,
    XBAR_OUT_MUX11_CLB2_OUT3              = 0x1600,
    XBAR_OUT_MUX11_CLB6_OUT3              = 0x1601,
    XBAR_OUT_MUX12_CLB2_OUT4              = 0x1800,
    XBAR_OUT_MUX12_CLB6_OUT4              = 0x1801,
    XBAR_OUT_MUX13_CLB2_OUT5              = 0x1A00,
    XBAR_OUT_MUX13_CLB6_OUT5              = 0x1A01,
    XBAR_OUT_MUX14_CLB2_OUT6              = 0x1C00,
    XBAR_OUT_MUX14_CLB6_OUT6              = 0x1C01,
    XBAR_OUT_MUX15_CLB2_OUT7              = 0x1E00,
    XBAR_OUT_MUX15_CLB6_OUT7              = 0x1E01,
    XBAR_OUT_MUX16_CLB3_OUT0              = 0x2000,
    XBAR_OUT_MUX16_CLB7_OUT0              = 0x2001,
    XBAR_OUT_MUX17_CLB3_OUT1              = 0x2200,
    XBAR_OUT_MUX17_CLB7_OUT1              = 0x2201,
    XBAR_OUT_MUX18_CLB3_OUT2              = 0x2400,
    XBAR_OUT_MUX18_CLB7_OUT2              = 0x2401,
    XBAR_OUT_MUX19_CLB3_OUT3              = 0x2600,
    XBAR_OUT_MUX19_CLB7_OUT3              = 0x2601,
    XBAR_OUT_MUX20_CLB3_OUT4              = 0x2800,
    XBAR_OUT_MUX20_CLB7_OUT4              = 0x2801,
    XBAR_OUT_MUX21_CLB3_OUT5              = 0x2A00,
    XBAR_OUT_MUX21_CLB7_OUT5              = 0x2A01,
    XBAR_OUT_MUX22_CLB3_OUT6              = 0x2C00,
    XBAR_OUT_MUX22_CLB7_OUT6              = 0x2C01,
    XBAR_OUT_MUX23_CLB3_OUT7              = 0x2E00,
    XBAR_OUT_MUX23_CLB7_OUT7              = 0x2E01,
    XBAR_OUT_MUX24_CLB4_OUT0              = 0x3000,
    XBAR_OUT_MUX24_CLB8_OUT0              = 0x3001,
    XBAR_OUT_MUX25_CLB4_OUT1              = 0x3200,
    XBAR_OUT_MUX25_CLB8_OUT1              = 0x3201,
    XBAR_OUT_MUX26_CLB4_OUT2              = 0x3400,
    XBAR_OUT_MUX26_CLB8_OUT2              = 0x3401,
    XBAR_OUT_MUX27_CLB4_OUT3              = 0x3600,
    XBAR_OUT_MUX27_CLB8_OUT3              = 0x3601,
    XBAR_OUT_MUX28_CLB4_OUT4              = 0x3800,
    XBAR_OUT_MUX28_CLB8_OUT4              = 0x3801,
    XBAR_OUT_MUX29_CLB4_OUT5              = 0x3A00,
    XBAR_OUT_MUX29_CLB8_OUT5              = 0x3A01,
    XBAR_OUT_MUX30_CLB4_OUT6              = 0x3C00,
    XBAR_OUT_MUX30_CLB8_OUT6              = 0x3C01,
    XBAR_OUT_MUX31_CLB4_OUT7              = 0x3E00,
    XBAR_OUT_MUX31_CLB8_OUT7              = 0x3E01,
} XBAR_OutputMuxConfig;

//*****************************************************************************
//
//! The following values define the \e muxConfig parameter for
//! XBAR_setEPWMMuxConfig().
//
//*****************************************************************************
typedef enum
{
    XBAR_EPWM_MUX00_CMPSS1_CTRIPH          = 0x0000,
    XBAR_EPWM_MUX00_CMPSS1_CTRIPH_OR_L     = 0x0001,
    XBAR_EPWM_MUX00_ADCAEVT1               = 0x0002,
    XBAR_EPWM_MUX00_ECAP1_OUT              = 0x0003,
    XBAR_EPWM_MUX01_CMPSS1_CTRIPL          = 0x0200,
    XBAR_EPWM_MUX01_INPUTXBAR1             = 0x0201,
    XBAR_EPWM_MUX01_CLB1_OUT4              = 0x0202,
    XBAR_EPWM_MUX01_ADCCEVT1               = 0x0203,
    XBAR_EPWM_MUX02_CMPSS2_CTRIPH          = 0x0400,
    XBAR_EPWM_MUX02_CMPSS2_CTRIPH_OR_L     = 0x0401,
    XBAR_EPWM_MUX02_ADCAEVT2               = 0x0402,
    XBAR_EPWM_MUX02_ECAP2_OUT              = 0x0403,
    XBAR_EPWM_MUX03_CMPSS2_CTRIPL          = 0x0600,
    XBAR_EPWM_MUX03_INPUTXBAR2             = 0x0601,
    XBAR_EPWM_MUX03_CLB1_OUT5              = 0x0602,
    XBAR_EPWM_MUX03_ADCCEVT2               = 0x0603,
    XBAR_EPWM_MUX04_CMPSS3_CTRIPH          = 0x0800,
    XBAR_EPWM_MUX04_CMPSS3_CTRIPH_OR_L     = 0x0801,
    XBAR_EPWM_MUX04_ADCAEVT3               = 0x0802,
    XBAR_EPWM_MUX04_ECAP3_OUT              = 0x0803,
    XBAR_EPWM_MUX05_CMPSS3_CTRIPL          = 0x0A00,
    XBAR_EPWM_MUX05_INPUTXBAR3             = 0x0A01,
    XBAR_EPWM_MUX05_CLB2_OUT4              = 0x0A02,
    XBAR_EPWM_MUX05_ADCCEVT3               = 0x0A03,
    XBAR_EPWM_MUX06_CMPSS4_CTRIPH          = 0x0C00,
    XBAR_EPWM_MUX06_CMPSS4_CTRIPH_OR_L     = 0x0C01,
    XBAR_EPWM_MUX06_ADCAEVT4               = 0x0C02,
    XBAR_EPWM_MUX06_ECAP4_OUT              = 0x0C03,
    XBAR_EPWM_MUX07_CMPSS4_CTRIPL          = 0x0E00,
    XBAR_EPWM_MUX07_INPUTXBAR4             = 0x0E01,
    XBAR_EPWM_MUX07_CLB2_OUT5              = 0x0E02,
    XBAR_EPWM_MUX07_ADCCEVT4               = 0x0E03,
    XBAR_EPWM_MUX08_CMPSS5_CTRIPH          = 0x1000,
    XBAR_EPWM_MUX08_CMPSS5_CTRIPH_OR_L     = 0x1001,
    XBAR_EPWM_MUX08_ADCBEVT1               = 0x1002,
    XBAR_EPWM_MUX08_ECAP5_OUT              = 0x1003,
    XBAR_EPWM_MUX09_CMPSS5_CTRIPL          = 0x1200,
    XBAR_EPWM_MUX09_INPUTXBAR5             = 0x1201,
    XBAR_EPWM_MUX09_CLB3_OUT4              = 0x1202,
    XBAR_EPWM_MUX10_CMPSS6_CTRIPH          = 0x1400,
    XBAR_EPWM_MUX10_CMPSS6_CTRIPH_OR_L     = 0x1401,
    XBAR_EPWM_MUX10_ADCBEVT2               = 0x1402,
    XBAR_EPWM_MUX10_ECAP6_OUT              = 0x1403,
    XBAR_EPWM_MUX11_CMPSS6_CTRIPL          = 0x1600,
    XBAR_EPWM_MUX11_INPUTXBAR6             = 0x1601,
    XBAR_EPWM_MUX11_CLB3_OUT5              = 0x1602,
    XBAR_EPWM_MUX11_ADCDEVT2               = 0x1603,
    XBAR_EPWM_MUX12_CMPSS7_CTRIPH          = 0x1800,
    XBAR_EPWM_MUX12_CMPSS7_CTRIPH_OR_L     = 0x1801,
    XBAR_EPWM_MUX12_ADCBEVT3               = 0x1802,
    XBAR_EPWM_MUX12_ECAP7_OUT              = 0x1803,
    XBAR_EPWM_MUX13_CMPSS7_CTRIPL          = 0x1A00,
    XBAR_EPWM_MUX13_ADCSOCA                = 0x1A01,
    XBAR_EPWM_MUX13_CLB4_OUT4              = 0x1A02,
    XBAR_EPWM_MUX13_ADCDEVT3               = 0x1A03,
    XBAR_EPWM_MUX14_CMPSS8_CTRIPH          = 0x1C00,
    XBAR_EPWM_MUX14_CMPSS8_CTRIPH_OR_L     = 0x1C01,
    XBAR_EPWM_MUX14_ADCBEVT4               = 0x1C02,
    XBAR_EPWM_MUX14_EXTSYNCOUT             = 0x1C03,
    XBAR_EPWM_MUX15_CMPSS8_CTRIPL          = 0x1E00,
    XBAR_EPWM_MUX15_ADCSOCB                = 0x1E01,
    XBAR_EPWM_MUX15_CLB4_OUT5              = 0x1E02,
    XBAR_EPWM_MUX15_ADCDEVT4               = 0x1E03,
    XBAR_EPWM_MUX16_SD1FLT1_COMPH          = 0x2000,
    XBAR_EPWM_MUX16_SD1FLT1_COMPH_OR_COMPL = 0x2001,
    XBAR_EPWM_MUX16_ERRORSTS               = 0x2003,
    XBAR_EPWM_MUX17_SD1FLT1_COMPL          = 0x2200,
    XBAR_EPWM_MUX17_INPUTXBAR7             = 0x2201,
    XBAR_EPWM_MUX17_CLB5_OUT4              = 0x2202,
    XBAR_EPWM_MUX17_CLAHALT                = 0x2203,
    XBAR_EPWM_MUX18_SD1FLT2_COMPH          = 0x2400,
    XBAR_EPWM_MUX18_SD1FLT2_COMPH_OR_COMPL = 0x2401,
    XBAR_EPWM_MUX18_ECATSYNC0              = 0x2403,
    XBAR_EPWM_MUX19_SD1FLT2_COMPL          = 0x2600,
    XBAR_EPWM_MUX19_INPUTXBAR8             = 0x2601,
    XBAR_EPWM_MUX19_CLB5_OUT5              = 0x2602,
    XBAR_EPWM_MUX19_ECATSYNC1              = 0x2603,
    XBAR_EPWM_MUX20_SD1FLT3_COMPH          = 0x2800,
    XBAR_EPWM_MUX20_SD1FLT3_COMPH_OR_COMPL = 0x2801,
    XBAR_EPWM_MUX21_SD1FLT3_COMPL          = 0x2A00,
    XBAR_EPWM_MUX21_INPUTXBAR9             = 0x2A01,
    XBAR_EPWM_MUX21_CLB6_OUT4              = 0x2A02,
    XBAR_EPWM_MUX22_SD1FLT4_COMPH          = 0x2C00,
    XBAR_EPWM_MUX22_SD1FLT4_COMPH_OR_COMPL = 0x2C01,
    XBAR_EPWM_MUX23_SD1FLT4_COMPL          = 0x2E00,
    XBAR_EPWM_MUX23_INPUTXBAR10            = 0x2E01,
    XBAR_EPWM_MUX23_CLB6_OUT5              = 0x2E02,
    XBAR_EPWM_MUX24_SD2FLT1_COMPH          = 0x3000,
    XBAR_EPWM_MUX24_SD2FLT1_COMPH_OR_COMPL = 0x3001,
    XBAR_EPWM_MUX25_SD2FLT1_COMPL          = 0x3200,
    XBAR_EPWM_MUX25_INPUTXBAR11            = 0x3201,
    XBAR_EPWM_MUX25_MCANA_FEVT0            = 0x3202,
    XBAR_EPWM_MUX25_CLB7_OUT4              = 0x3203,
    XBAR_EPWM_MUX26_SD2FLT2_COMPH          = 0x3400,
    XBAR_EPWM_MUX26_SD2FLT2_COMPH_OR_COMPL = 0x3401,
    XBAR_EPWM_MUX27_SD2FLT2_COMPL          = 0x3600,
    XBAR_EPWM_MUX27_INPUTXBAR12            = 0x3601,
    XBAR_EPWM_MUX27_MCANA_FEVT1            = 0x3602,
    XBAR_EPWM_MUX27_CLB7_OUT5              = 0x3603,
    XBAR_EPWM_MUX28_SD2FLT3_COMPH          = 0x3800,
    XBAR_EPWM_MUX28_SD2FLT3_COMPH_OR_COMPL = 0x3801,
    XBAR_EPWM_MUX29_SD2FLT3_COMPL          = 0x3A00,
    XBAR_EPWM_MUX29_INPUTXBAR13            = 0x3A01,
    XBAR_EPWM_MUX29_MCANA_FEVT2            = 0x3A02,
    XBAR_EPWM_MUX29_CLB8_OUT4              = 0x3A03,
    XBAR_EPWM_MUX30_SD2FLT4_COMPH          = 0x3C00,
    XBAR_EPWM_MUX30_SD2FLT4_COMPH_OR_COMPL = 0x3C01,
    XBAR_EPWM_MUX31_INPUTXBAR14            = 0x3E01,
    XBAR_EPWM_MUX31_CLB8_OUT5              = 0x3E03,
    XBAR_EPWM_MUX31_SD2FLT4_COMPL          = 0x3E00
} XBAR_EPWMMuxConfig;

//*****************************************************************************
//
// The following values define the muxConfig parameter for
// XBAR_setCLBMuxConfig().
//
//*****************************************************************************
typedef enum
{
    XBAR_CLB_MUX00_CMPSS1_CTRIPH           = 0x0000,
    XBAR_CLB_MUX00_CMPSS1_CTRIPH_OR_L      = 0x0001,
    XBAR_CLB_MUX00_ADCAEVT1                = 0x0002,
    XBAR_CLB_MUX00_ECAP1_OUT               = 0x0003,
    XBAR_CLB_MUX01_CMPSS1_CTRIPL           = 0x0200,
    XBAR_CLB_MUX01_INPUTXBAR1              = 0x0201,
    XBAR_CLB_MUX01_CLB1_OUT4               = 0x0202,
    XBAR_CLB_MUX01_ADCCEVT1                = 0x0203,
    XBAR_CLB_MUX02_CMPSS2_CTRIPH           = 0x0400,
    XBAR_CLB_MUX02_CMPSS2_CTRIPH_OR_L      = 0x0401,
    XBAR_CLB_MUX02_ADCAEVT2                = 0x0402,
    XBAR_CLB_MUX02_ECAP2_OUT               = 0x0403,
    XBAR_CLB_MUX03_CMPSS2_CTRIPL           = 0x0600,
    XBAR_CLB_MUX03_INPUTXBAR2              = 0x0601,
    XBAR_CLB_MUX03_CLB1_OUT5               = 0x0602,
    XBAR_CLB_MUX03_ADCCEVT2                = 0x0603,
    XBAR_CLB_MUX04_CMPSS3_CTRIPH           = 0x0800,
    XBAR_CLB_MUX04_CMPSS3_CTRIPH_OR_L      = 0x0801,
    XBAR_CLB_MUX04_ADCAEVT3                = 0x0802,
    XBAR_CLB_MUX04_ECAP3_OUT               = 0x0803,
    XBAR_CLB_MUX05_CMPSS3_CTRIPL           = 0x0A00,
    XBAR_CLB_MUX05_INPUTXBAR3              = 0x0A01,
    XBAR_CLB_MUX05_CLB2_OUT4               = 0x0A02,
    XBAR_CLB_MUX05_ADCCEVT3                = 0x0A03,
    XBAR_CLB_MUX06_CMPSS4_CTRIPH           = 0x0C00,
    XBAR_CLB_MUX06_CMPSS4_CTRIPH_OR_L      = 0x0C01,
    XBAR_CLB_MUX06_ADCAEVT4                = 0x0C02,
    XBAR_CLB_MUX06_ECAP4_OUT               = 0x0C03,
    XBAR_CLB_MUX07_CMPSS4_CTRIPL           = 0x0E00,
    XBAR_CLB_MUX07_INPUTXBAR4              = 0x0E01,
    XBAR_CLB_MUX07_CLB2_OUT5               = 0x0E02,
    XBAR_CLB_MUX07_ADCCEVT4                = 0x0E03,
    XBAR_CLB_MUX08_CMPSS5_CTRIPH           = 0x1000,
    XBAR_CLB_MUX08_CMPSS5_CTRIPH_OR_L      = 0x1001,
    XBAR_CLB_MUX08_ADCBEVT1                = 0x1002,
    XBAR_CLB_MUX08_ECAP5_OUT               = 0x1003,
    XBAR_CLB_MUX09_CMPSS5_CTRIPL           = 0x1200,
    XBAR_CLB_MUX09_INPUTXBAR5              = 0x1201,
    XBAR_CLB_MUX09_CLB3_OUT4               = 0x1202,
    XBAR_CLB_MUX10_CMPSS6_CTRIPH           = 0x1400,
    XBAR_CLB_MUX10_CMPSS6_CTRIPH_OR_L      = 0x1401,
    XBAR_CLB_MUX10_ADCBEVT2                = 0x1402,
    XBAR_CLB_MUX10_ECAP6_OUT               = 0x1403,
    XBAR_CLB_MUX11_CMPSS6_CTRIPL           = 0x1600,
    XBAR_CLB_MUX11_INPUTXBAR6              = 0x1601,
    XBAR_CLB_MUX11_CLB3_OUT5               = 0x1602,
    XBAR_CLB_MUX12_CMPSS7_CTRIPH           = 0x1800,
    XBAR_CLB_MUX12_CMPSS7_CTRIPH_OR_L      = 0x1801,
    XBAR_CLB_MUX12_ADCBEVT3                = 0x1802,
    XBAR_CLB_MUX12_ECAP7_OUT               = 0x1803,
    XBAR_CLB_MUX13_CMPSS7_CTRIPL           = 0x1A00,
    XBAR_CLB_MUX13_ADCSOCA                 = 0x1A01,
    XBAR_CLB_MUX13_CLB4_OUT4               = 0x1A02,
    XBAR_CLB_MUX14_ADCBEVT4                = 0x1C02,
    XBAR_CLB_MUX14_EXTSYNCOUT              = 0x1C03,
    XBAR_CLB_MUX15_ADCSOCB                 = 0x1E01,
    XBAR_CLB_MUX15_CLB4_OUT5               = 0x1E02,
    XBAR_CLB_MUX16_SD1FLT1_COMPH           = 0x2000,
    XBAR_CLB_MUX16_SD1FLT1_COMPH_OR_COMPL  = 0x2001,
    XBAR_CLB_MUX16_SD1FLT1_COMPZ           = 0x2002,
    XBAR_CLB_MUX16_SD1FLT1_DRINT           = 0x2003,
    XBAR_CLB_MUX17_SD1FLT1_COMPL           = 0x2200,
    XBAR_CLB_MUX17_INPUTXBAR7              = 0x2201,
    XBAR_CLB_MUX17_CLB5_OUT4               = 0x2202,
    XBAR_CLB_MUX17_CLAHALT                 = 0x2203,
    XBAR_CLB_MUX18_SD1FLT2_COMPH           = 0x2400,
    XBAR_CLB_MUX18_SD1FLT2_COMPH_OR_COMPL  = 0x2401,
    XBAR_CLB_MUX18_SD1FLT2_COMPZ           = 0x2402,
    XBAR_CLB_MUX18_SD1FLT2_DRINT           = 0x2403,
    XBAR_CLB_MUX19_SD1FLT2_COMPL           = 0x2600,
    XBAR_CLB_MUX19_INPUTXBAR8              = 0x2601,
    XBAR_CLB_MUX19_CLB5_OUT5               = 0x2602,
    XBAR_CLB_MUX20_SD1FLT3_COMPH           = 0x2800,
    XBAR_CLB_MUX20_SD1FLT3_COMPH_OR_COMPL  = 0x2801,
    XBAR_CLB_MUX20_SD1FLT3_COMPZ           = 0x2802,
    XBAR_CLB_MUX20_SD1FLT3_DRINT           = 0x2803,
    XBAR_CLB_MUX21_SD1FLT3_COMPL           = 0x2A00,
    XBAR_CLB_MUX21_INPUTXBAR9              = 0x2A01,
    XBAR_CLB_MUX21_CLB6_OUT4               = 0x2A02,
    XBAR_CLB_MUX22_SD1FLT4_COMPH           = 0x2C00,
    XBAR_CLB_MUX22_SD1FLT4_COMPH_OR_COMPL  = 0x2C01,
    XBAR_CLB_MUX22_SD1FLT4_COMPZ           = 0x2C02,
    XBAR_CLB_MUX22_SD1FLT4_DRINT           = 0x2C03,
    XBAR_CLB_MUX23_SD1FLT4_COMPL           = 0x2E00,
    XBAR_CLB_MUX23_INPUTXBAR10             = 0x2E01,
    XBAR_CLB_MUX23_CLB6_OUT5               = 0x2E02,
    XBAR_CLB_MUX23_CLB_EMAC_PPS1           = 0x2E03,
    XBAR_CLB_MUX24_SD2FLT1_COMPH           = 0x3000,
    XBAR_CLB_MUX24_SD2FLT1_COMPH_OR_COMPL  = 0x3001,
    XBAR_CLB_MUX24_SD2FLT1_COMPZ           = 0x3002,
    XBAR_CLB_MUX24_SD2FLT1_DRINT           = 0x3003,
    XBAR_CLB_MUX25_SD2FLT1_COMPL           = 0x3200,
    XBAR_CLB_MUX25_INPUTXBAR11             = 0x3201,
    XBAR_CLB_MUX25_MCANA_FEVT0             = 0x3202,
    XBAR_CLB_MUX25_CLB7_OUT4               = 0x3203,
    XBAR_CLB_MUX26_SD2FLT2_COMPH           = 0x3400,
    XBAR_CLB_MUX26_SD2FLT2_COMPH_OR_COMPL  = 0x3401,
    XBAR_CLB_MUX26_SD2FLT2_COMPZ           = 0x3402,
    XBAR_CLB_MUX26_SD2FLT2_DRINT           = 0x3403,
    XBAR_CLB_MUX27_SD2FLT2_COMPL           = 0x3600,
    XBAR_CLB_MUX27_INPUTXBAR12             = 0x3601,
    XBAR_CLB_MUX27_MCANA_FEVT1             = 0x3602,
    XBAR_CLB_MUX27_CLB7_OUT5               = 0x3603,
    XBAR_CLB_MUX28_SD2FLT3_COMPH           = 0x3800,
    XBAR_CLB_MUX28_SD2FLT3_COMPH_OR_COMPL  = 0x3801,
    XBAR_CLB_MUX28_SD2FLT3_COMPZ           = 0x3802,
    XBAR_CLB_MUX28_SD2FLT3_DRINT           = 0x3803,
    XBAR_CLB_MUX29_SD2FLT3_COMPL           = 0x3A00,
    XBAR_CLB_MUX29_INPUTXBAR13             = 0x3A01,
    XBAR_CLB_MUX29_MCANA_FEVT2             = 0x3A02,
    XBAR_CLB_MUX29_CLB8_OUT4               = 0x3A03,
    XBAR_CLB_MUX30_SD2FLT4_COMPH           = 0x3C00,
    XBAR_CLB_MUX30_SD2FLT4_COMPH_OR_COMPL  = 0x3C01,
    XBAR_CLB_MUX30_SD2FLT4_COMPZ           = 0x3C02,
    XBAR_CLB_MUX30_SD2FLT4_DRINT           = 0x3C03,
    XBAR_CLB_MUX31_SD2FLT4_COMPL           = 0x3E00,
    XBAR_CLB_MUX31_CLB_EMAC_PPS0           = 0x3E02,
    XBAR_CLB_MUX31_CLB8_OUT5               = 0x3E03,
    XBAR_CLB_MUX31_INPUTXBAR14             = 0x3E01,
} XBAR_CLBMuxConfig;


//*****************************************************************************
//
//! The following values define the \e inputFlag parameter for
//! XBAR_getInputFlagStatus() and XBAR_clearInputFlag().
//
//*****************************************************************************
typedef enum
{
    //
    // XBARFLG1
    //
    XBAR_INPUT_FLG_CMPSS1_CTRIPL    = 0x0000,
    XBAR_INPUT_FLG_CMPSS1_CTRIPH    = 0x0001,
    XBAR_INPUT_FLG_CMPSS2_CTRIPL    = 0x0002,
    XBAR_INPUT_FLG_CMPSS2_CTRIPH    = 0x0003,
    XBAR_INPUT_FLG_CMPSS3_CTRIPL    = 0x0004,
    XBAR_INPUT_FLG_CMPSS3_CTRIPH    = 0x0005,
    XBAR_INPUT_FLG_CMPSS4_CTRIPL    = 0x0006,
    XBAR_INPUT_FLG_CMPSS4_CTRIPH    = 0x0007,
    XBAR_INPUT_FLG_CMPSS5_CTRIPL    = 0x0008,
    XBAR_INPUT_FLG_CMPSS5_CTRIPH    = 0x0009,
    XBAR_INPUT_FLG_CMPSS6_CTRIPL    = 0x000A,
    XBAR_INPUT_FLG_CMPSS6_CTRIPH    = 0x000B,
    XBAR_INPUT_FLG_CMPSS7_CTRIPL    = 0x000C,
    XBAR_INPUT_FLG_CMPSS7_CTRIPH    = 0x000D,
    XBAR_INPUT_FLG_CMPSS8_CTRIPL    = 0x000E,
    XBAR_INPUT_FLG_CMPSS8_CTRIPH    = 0x000F,
    XBAR_INPUT_FLG_CMPSS1_CTRIPOUTL = 0x0010,
    XBAR_INPUT_FLG_CMPSS1_CTRIPOUTH = 0x0011,
    XBAR_INPUT_FLG_CMPSS2_CTRIPOUTL = 0x0012,
    XBAR_INPUT_FLG_CMPSS2_CTRIPOUTH = 0x0013,
    XBAR_INPUT_FLG_CMPSS3_CTRIPOUTL = 0x0014,
    XBAR_INPUT_FLG_CMPSS3_CTRIPOUTH = 0x0015,
    XBAR_INPUT_FLG_CMPSS4_CTRIPOUTL = 0x0016,
    XBAR_INPUT_FLG_CMPSS4_CTRIPOUTH = 0x0017,
    XBAR_INPUT_FLG_CMPSS5_CTRIPOUTL = 0x0018,
    XBAR_INPUT_FLG_CMPSS5_CTRIPOUTH = 0x0019,
    XBAR_INPUT_FLG_CMPSS6_CTRIPOUTL = 0x001A,
    XBAR_INPUT_FLG_CMPSS6_CTRIPOUTH = 0x001B,
    XBAR_INPUT_FLG_CMPSS7_CTRIPOUTL = 0x001C,
    XBAR_INPUT_FLG_CMPSS7_CTRIPOUTH = 0x001D,
    XBAR_INPUT_FLG_CMPSS8_CTRIPOUTL = 0x001E,
    XBAR_INPUT_FLG_CMPSS8_CTRIPOUTH = 0x001F,
    //
    // XBARFLG2
    //
    XBAR_INPUT_FLG_INPUT1           = 0x0100,
    XBAR_INPUT_FLG_INPUT2           = 0x0101,
    XBAR_INPUT_FLG_INPUT3           = 0x0102,
    XBAR_INPUT_FLG_INPUT4           = 0x0103,
    XBAR_INPUT_FLG_INPUT5           = 0x0104,
    XBAR_INPUT_FLG_INPUT6           = 0x0105,
    XBAR_INPUT_FLG_ADCSOCA          = 0x0106,
    XBAR_INPUT_FLG_ADCSOCB          = 0x0107,
    XBAR_INPUT_FLG_INPUT7           = 0x0108,
    XBAR_INPUT_FLG_INPUT8           = 0x0109,
    XBAR_INPUT_FLG_INPUT9           = 0x010A,
    XBAR_INPUT_FLG_INPUT10          = 0x010B,
    XBAR_INPUT_FLG_INPUT11          = 0x010C,
    XBAR_INPUT_FLG_INPUT12          = 0x010D,
    XBAR_INPUT_FLG_INPUT13          = 0x010E,
    XBAR_INPUT_FLG_INPUT14          = 0x010F,
    XBAR_INPUT_FLG_ECAP1_OUT        = 0x0110,
    XBAR_INPUT_FLG_ECAP2_OUT        = 0x0111,
    XBAR_INPUT_FLG_ECAP3_OUT        = 0x0112,
    XBAR_INPUT_FLG_ECAP4_OUT        = 0x0113,
    XBAR_INPUT_FLG_ECAP5_OUT        = 0x0114,
    XBAR_INPUT_FLG_ECAP6_OUT        = 0x0115,
    XBAR_INPUT_FLG_EXTSYNCOUT       = 0x0116,
    XBAR_INPUT_FLG_ADCAEVT1         = 0x0117,
    XBAR_INPUT_FLG_ADCAEVT2         = 0x0118,
    XBAR_INPUT_FLG_ADCAEVT3         = 0x0119,
    XBAR_INPUT_FLG_ADCAEVT4         = 0x011A,
    XBAR_INPUT_FLG_ADCBEVT1         = 0x011B,
    XBAR_INPUT_FLG_ADCBEVT2         = 0x011C,
    XBAR_INPUT_FLG_ADCBEVT3         = 0x011D,
    XBAR_INPUT_FLG_ADCBEVT4         = 0x011E,
    XBAR_INPUT_FLG_ADCCEVT1         = 0x011F,
    //
    // XBARFLG3
    //
    XBAR_INPUT_FLG_ADCCEVT2         = 0x0200,
    XBAR_INPUT_FLG_ADCCEVT3         = 0x0201,
    XBAR_INPUT_FLG_ADCCEVT4         = 0x0202,
    XBAR_INPUT_FLG_ADCDEVT1         = 0x0203,
    XBAR_INPUT_FLG_ADCDEVT2         = 0x0204,
    XBAR_INPUT_FLG_ADCDEVT3         = 0x0205,
    XBAR_INPUT_FLG_ADCDEVT4         = 0x0206,
    XBAR_INPUT_FLG_SD1FLT1_COMPL    = 0x0207,
    XBAR_INPUT_FLG_SD1FLT1_COMPH    = 0x0208,
    XBAR_INPUT_FLG_SD1FLT2_COMPL    = 0x0209,
    XBAR_INPUT_FLG_SD1FLT2_COMPH    = 0x020A,
    XBAR_INPUT_FLG_SD1FLT3_COMPL    = 0x020B,
    XBAR_INPUT_FLG_SD1FLT3_COMPH    = 0x020C,
    XBAR_INPUT_FLG_SD1FLT4_COMPL    = 0x020D,
    XBAR_INPUT_FLG_SD1FLT4_COMPH    = 0x020E,
    XBAR_INPUT_FLG_SD2FLT1_COMPL    = 0x020F,
    XBAR_INPUT_FLG_SD2FLT1_COMPH    = 0x0210,
    XBAR_INPUT_FLG_SD2FLT2_COMPL    = 0x0211,
    XBAR_INPUT_FLG_SD2FLT2_COMPH    = 0x0212,
    XBAR_INPUT_FLG_SD2FLT3_COMPL    = 0x0213,
    XBAR_INPUT_FLG_SD2FLT3_COMPH    = 0x0214,
    XBAR_INPUT_FLG_SD2FLT4_COMPL    = 0x0215,
    XBAR_INPUT_FLG_SD2FLT4_COMPH    = 0x0216,
    XBAR_INPUT_FLG_ECAP7_OUT        = 0x0217,
    XBAR_INPUT_FLG_SD1FLT1_COMPZ    = 0x0218,
    XBAR_INPUT_FLG_SD1FLT1_DRINT    = 0x0219,
    XBAR_INPUT_FLG_SD1FLT2_COMPZ    = 0x021A,
    XBAR_INPUT_FLG_SD1FLT2_DRINT    = 0x021B,
    XBAR_INPUT_FLG_SD1FLT3_COMPZ    = 0x021C,
    XBAR_INPUT_FLG_SD1FLT3_DRINT    = 0x021D,
    XBAR_INPUT_FLG_SD1FLT4_COMPZ    = 0x021E,
    XBAR_INPUT_FLG_SD1FLT4_DRINT    = 0x021F,
    //
    // XBARFLG4
    //
    XBAR_INPUT_FLG_SD2FLT1_COMPZ    = 0x0300,
    XBAR_INPUT_FLG_SD2FLT1_DRINT    = 0x0301,
    XBAR_INPUT_FLG_SD2FLT2_COMPZ    = 0x0302,
    XBAR_INPUT_FLG_SD2FLT2_DRINT    = 0x0303,
    XBAR_INPUT_FLG_SD2FLT3_COMPZ    = 0x0304,
    XBAR_INPUT_FLG_SD2FLT3_DRINT    = 0x0305,
    XBAR_INPUT_FLG_SD2FLT4_COMPZ    = 0x0306,
    XBAR_INPUT_FLG_SD2FLT4_DRINT    = 0x0307,
    XBAR_INPUT_FLG_EMAC_PPS0        = 0x0308,
    XBAR_INPUT_FLG_MCANA_FEVT0      = 0x0309,
    XBAR_INPUT_FLG_MCANA_FEVT1      = 0x030A,
    XBAR_INPUT_FLG_MCANA_FEVT2      = 0x030B,
    XBAR_INPUT_FLG_CLB7_OUT4        = 0x030C,
    XBAR_INPUT_FLG_CLB7_OUT5        = 0x030D,
    XBAR_INPUT_FLG_CLB8_OUT4        = 0x030E,
    XBAR_INPUT_FLG_CLB8_OUT5        = 0x030F,
    XBAR_INPUT_FLG_CLB1_OUT4        = 0x0310,
    XBAR_INPUT_FLG_CLB1_OUT5        = 0x0311,
    XBAR_INPUT_FLG_CLB2_OUT4        = 0x0312,
    XBAR_INPUT_FLG_CLB2_OUT5        = 0x0313,
    XBAR_INPUT_FLG_CLB3_OUT4        = 0x0314,
    XBAR_INPUT_FLG_CLB3_OUT5        = 0x0315,
    XBAR_INPUT_FLG_CLB4_OUT4        = 0x0316,
    XBAR_INPUT_FLG_CLB4_OUT5        = 0x0317,
    XBAR_INPUT_FLG_CLB5_OUT4        = 0x0318,
    XBAR_INPUT_FLG_CLB5_OUT5        = 0x0319,
    XBAR_INPUT_FLG_CLB6_OUT4        = 0x031A,
    XBAR_INPUT_FLG_CLB6_OUT5        = 0x031B,
    XBAR_INPUT_FLG_ERRORSTS_ERROR   = 0x031C,
    XBAR_INPUT_FLG_ECATSYNC0        = 0x031D,
    XBAR_INPUT_FLG_ECATSYNC1        = 0x031E,
    XBAR_INPUT_FLG_CLAHALT          = 0x031F,
} XBAR_InputFlag;
#endif

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! \internal
//! Checks a X-BAR base address.
//!
//! \param base is the base address of the X-BAR.
//!
//! This function determines if a X-BAR base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
XBAR_isBaseValid(uint32_t base)
{
    return((base == OUTPUTXBAR_BASE) ||
           (base == CLBOUTPUTXBAR_BASE) || (base == CLBINPUTXBAR_BASE ) ||
           (base == INPUTXBAR_BASE));

}
#endif
//*****************************************************************************
//
//! Enables the Output X-BAR mux values to be passed to the output signal.
//!
//! \param base specifies the X-BAR Enable register base address.
//! \param output is the X-BAR output being configured.
//! \param muxes is a bit field of the muxes to be enabled.
//!
//! This function enables the mux values to be passed to the X-BAR output
//! signal. The \e output parameter is a value \b XBAR_OUTPUTy where y is
//! the output number between 1 and 8 inclusive.
//!
//! The \e base parameter can take base addresses
//! OUTPUTXBAR_BASE
//! or CLBOUTPUTXBAR_BASE.
//!
//! The \e muxes parameter is a bit field of the muxes being enabled where bit
//! 0 represents mux 0, bit 1 represents mux 1 and so on. Defines are provided
//! in the form of \b XBAR_MUXnn that can be OR'd together to enable several
//! muxes on an output at the same time. For example, passing this function
//! ( \b XBAR_MUX04 | \b XBAR_MUX10 ) would enable muxes 4 and 10.
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_enableOutputMux(uint32_t base, XBAR_OutputNum output, uint32_t muxes)
{
    uint16_t outputNum = (uint16_t)output;
    //
    // Check the arguments.
    //
    ASSERT(XBAR_isBaseValid(base));

    //
    // Set the enable bit.
    //
    EALLOW;

    HWREG(base + XBAR_O_OUTPUT1MUXENABLE + outputNum) |= muxes;

    EDIS;
}

//*****************************************************************************
//
//! Disables the Output X-BAR mux values from being passed to the output.
//!
//! \param base specifies the X-BAR Enable Register base address.
//! \param output is the X-BAR output being configured.
//! \param muxes is a bit field of the muxes to be disabled.
//!
//! This function disables the mux values from being passed to the X-BAR output
//! signal. The \e output parameter is a value \b XBAR_OUTPUTy where y is
//! the output number between 1 and 8 inclusive.
//!
//! The \e base parameter can take base addresses
//! OUTPUTXBAR_BASE
//! or CLBOUTPUTXBAR_BASE.
//!
//! The \e muxes parameter is a bit field of the muxes being disabled where bit
//! 0 represents mux 0, bit 1 represents mux 1 and so on. Defines are provided
//! in the form of \b XBAR_MUXnn that can be OR'd together to disable several
//! muxes on an output at the same time. For example, passing this function
//! ( \b XBAR_MUX04 | \b XBAR_MUX10 ) would disable muxes 4 and 10.
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_disableOutputMux(uint32_t base, XBAR_OutputNum output, uint32_t muxes)
{
    uint16_t outputNum = (uint16_t)output;

    //
    // Check the arguments.
    //
    ASSERT(XBAR_isBaseValid(base));

    //
    // Clear the enable bit.
    //
    EALLOW;

    HWREG(base + XBAR_O_OUTPUT1MUXENABLE + outputNum) &= ~(muxes);

    EDIS;
}

//*****************************************************************************
//
//! Enables or disables the output latch to drive the selected output.
//!
//! \param base specifies the X-BAR base address.
//! \param output is the X-BAR output being configured.
//! The valid inputs are XBAR_OUTPUTy where y is from 1 to 8.
//! \param enable is a flag that determines whether or not the latch is
//! selected to drive the X-BAR output.
//!
//! The \e base parameter can take base addresses
//! OUTPUTXBAR_BASE
//! or CLBOUTPUTXBAR_BASE.
//!
//! This function sets the Output X-BAR output signal latch mode. If the
//! \e enable parameter is \b true, the output specified by \e output will be
//! driven by the output latch.
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_setOutputLatchMode(uint32_t base, XBAR_OutputNum output, bool enable)
{
    //
    // Check the arguments.
    //
    ASSERT(XBAR_isBaseValid(base));

    EALLOW;

    //
    // Set or clear the latch setting bit based on the enable parameter.
    //
    if(enable)
    {
        HWREGH(base + XBAR_O_OUTPUTLATCHENABLE) |=
               0x1U << ((uint16_t)output / 2U);
    }
    else
    {
        HWREGH(base + XBAR_O_OUTPUTLATCHENABLE) &=
               ~(0x1U << ((uint16_t)output / 2U));
    }

    EDIS;
}

//*****************************************************************************
//
//! Returns the status of the output latch
//!
//! \param base specifies the X-BAR base address.
//! \param output is the X-BAR output being checked.
//! The valid inputs are XBAR_OUTPUTy where y is from 1 to 8.
//!
//! The \e base parameter can take base addresses
//! OUTPUTXBAR_BASE
//! or CLBOUTPUTXBAR_BASE.
//!
//! \return Returns \b true if the output corresponding to \e output was
//! triggered. If not, it will return \b false.
//
//*****************************************************************************
static inline bool
XBAR_getOutputLatchStatus(uint32_t base, XBAR_OutputNum output)
{
    //
    // Check the arguments.
    //
    ASSERT(XBAR_isBaseValid(base));

    //
    // Get the status of the Output X-BAR output latch.
    //
    return((HWREGH(base + XBAR_O_OUTPUTLATCH) &
            (0x1U << ((uint16_t)output / 2U))) != 0U);
}

//*****************************************************************************
//
//! Clears the output latch for the specified output.
//!
//! \param base specifies the X-BAR base address.
//! \param output is the X-BAR output being configured.
//! The valid inputs are XBAR_OUTPUTy where y is from 1 to 8.
//!
//! The \e base parameter can take base addresses
//! OUTPUTXBAR_BASE
//! or CLBOUTPUTXBAR_BASE.
//!
//! This function clears the Output X-BAR output latch. The output to be
//! configured is specified by the \e output parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_clearOutputLatch(uint32_t base, XBAR_OutputNum output)
{
    //
    // Check the arguments.
    //
    ASSERT(XBAR_isBaseValid(base));

    //
    // Set the bit that clears the corresponding OUTPUTLATCH bit.
    //
        HWREGH(base + XBAR_O_OUTPUTLATCHCLR) |=
        0x1U << ((uint16_t)output / 2U);
}

//*****************************************************************************
//
//! Forces the output latch for the specified output.
//!
//! \param base specifies the X-BAR base address.
//! \param output is the X-BAR output being configured.
//! The valid inputs are XBAR_OUTPUTy where y is from 1 to 8.
//!
//! The \e base parameter can take base addresses
//! OUTPUTXBAR_BASE
//! or CLBOUTPUTXBAR_BASE.
//!
//! This function forces the Output X-BAR output latch. The output to be
//! configured is specified by the \e output parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_forceOutputLatch(uint32_t base, XBAR_OutputNum output)
{
    //
    // Check the arguments.
    //
    ASSERT(XBAR_isBaseValid(base));

    //
    // Set the bit that forces the corresponding OUTPUTLATCH bit.
    //
    HWREGH(base + XBAR_O_OUTPUTLATCHFRC) =
        (uint16_t)0x1U << ((uint16_t)output / 2U);
}

//*****************************************************************************
//
//! Configures the polarity of an Output X-BAR output.
//!
//! \param base specifies the X-BAR base address.
//! \param output is the X-BAR output being configured.
//! The valid inputs are XBAR_OUTPUTy where y is from 1 to 8.
//! \param invert is a flag that determines whether the output is active-high
//! or active-low.
//!
//! The \e base parameter can take base addresses
//! OUTPUTXBAR_BASE
//! or CLBOUTPUTXBAR_BASE.
//!
//! This function inverts the Output X-BAR signal if the \e invert parameter is
//! \b true. If \e invert is \b false, the signal will be passed as is. The
//! \e output parameter is a value \b XBAR_OUTPUTy where y is the output
//! number between 1 and 8 inclusive.
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_invertOutputSignal(uint32_t base, XBAR_OutputNum output, bool invert)
{
    //
    // Check the arguments.
    //
    ASSERT(XBAR_isBaseValid(base));

    //
    // Set or clear the polarity setting bit based on the invert parameter.
    //
    EALLOW;

    if(invert)
    {
        HWREGH(base + XBAR_O_OUTPUTINV) |=
            0x1U << ((uint16_t)output / 2U);
    }
    else
    {
        HWREGH(base + XBAR_O_OUTPUTINV) &=
            ~(0x1U << ((uint16_t)output / 2U));
    }

    EDIS;
}

//*****************************************************************************
//
//! Enables the ePWM X-BAR mux values to be passed to an ePWM module.
//!
//! \param trip is the X-BAR output being configured.
//! \param muxes is a bit field of the muxes to be enabled.
//!
//! This function enables the mux values to be passed to the X-BAR trip
//! signal. The \e trip parameter is a value \b XBAR_TRIPy where y is
//! the number of the trip signal on the ePWM.
//!
//! The \e muxes parameter is a bit field of the muxes being enabled where bit
//! 0 represents mux 0, bit 1 represents mux 1 and so on. Defines are provided
//! in the form of \b XBAR_MUXnn that can be logically OR'd together to
//! enable several muxes on an output at the same time.
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_enableEPWMMux(XBAR_TripNum trip, uint32_t muxes)
{
    //
    // Set the enable bit.
    //
    EALLOW;

    HWREG(XBAR_EPWM_EN_REG_BASE + (uint32_t)trip) |= muxes;

    EDIS;
}

//*****************************************************************************
//
//! Disables the ePWM X-BAR mux values to be passed to an ePWM module.
//!
//! \param trip is the X-BAR output being configured.
//! \param muxes is a bit field of the muxes to be disabled.
//!
//! This function disables the mux values to be passed to the X-BAR trip
//! signal. The \e trip parameter is a value \b XBAR_TRIPy where y is
//! the number of the trip signal on the ePWM.
//!
//! The \e muxes parameter is a bit field of the muxes being disabled where bit
//! 0 represents mux 0, bit 1 represents mux 1 and so on. Defines are provided
//! in the form of \b XBAR_MUXnn that can be logically OR'd together to
//! disable several muxes on an output at the same time.
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_disableEPWMMux(XBAR_TripNum trip, uint32_t muxes)
{
    //
    // Clear the enable bit.
    //
    EALLOW;

    HWREG(XBAR_EPWM_EN_REG_BASE + (uint32_t)trip) &= ~(muxes);

    EDIS;
}

//*****************************************************************************
//
//! Configures the polarity of an ePWM X-BAR output.
//!
//! \param trip is the X-BAR output being configured.
//! \param invert is a flag that determines whether the output is active-high
//! or active-low.
//!
//! This function inverts the ePWM X-BAR trip signal if the \e invert
//! parameter is \b true. If \e invert is \b false, the signal will be passed
//! as is. The \e trip parameter is a value \b XBAR_TRIPy where y is
//! the number of the trip signal on the ePWM X-BAR that is being configured.
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_invertEPWMSignal(XBAR_TripNum trip, bool invert)
{
    //
    // Set or clear the polarity setting bit based on the invert parameter.
    //
    EALLOW;

    if(invert)
    {
        HWREGH(EPWMXBAR_BASE + XBAR_O_TRIPOUTINV) |=
            0x1U << ((uint16_t)trip / 2U);
    }
    else
    {
        HWREGH(EPWMXBAR_BASE + XBAR_O_TRIPOUTINV) &=
            ~(0x1U << ((uint16_t)trip / 2U));
    }

    EDIS;
}

//*****************************************************************************
//
//! Sets the GPIO / non-GPIO pin for an Input X-BAR input.
//!
//! \param base specifies the X-BAR base address.
//! \param input is the X-BAR input being configured.
//! \param pin is the identifying number of the pin.
//!
//! The \e base parameter can take base addresses
//! INPUTXBAR_BASE
//! or CLBINPUTXBAR_BASE.
//!
//! This function configures which GPIO is assigned to an Input X-BAR input.
//! The \e input parameter is a value in the form of a define \b XBAR_INPUTy
//! where y is a the input number for the Input X-BAR.
//!
//! The pin is specified by its numerical value. For example, GPIO34 is
//! specified by passing 34 as \e pin.
//!
//! For the other non - GPIO values:
//! 0xFFFD: '1' will be driven to the destination
//! 0xFFFE: '1' will be driven to the destination
//! 0xFFFF: '0' will be driven to the destination
//! NOTE: Pin value greater than the available number of GPIO pins on a
//! device (except 0xFFFF) will cause the destination to be driven '1'.
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_setInputPin(uint32_t base, XBAR_InputNum input, uint16_t pin)
{
    //
    // Check the argument.
    //
    ASSERT((pin <= XBAR_GPIO_MAX_CNT) ||
          ((pin >= XBAR_NON_GPIO_MIN_CNT) && (pin <= XBAR_NON_GPIO_MAX_CNT)));
    ASSERT(XBAR_isBaseValid(base));

    //
    // Write the requested pin to the appropriate input select register.
    //
    EALLOW;

    HWREGH(base + XBAR_O_INPUT1SELECT + (uint16_t)input) = pin;

    EDIS;
}

//*****************************************************************************
//
//! Locks an input to the Input X-BAR.
//!
//! \param base specifies the X-BAR base address.
//! \param input is an input to the Input X-BAR.
//!
//! This function locks the specific input on the Input X-BAR.
//!
//! The \e base parameter can take base addresses
//! INPUTXBAR_BASE
//! or CLBINPUTXBAR_BASE .
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_lockInput(uint32_t base, XBAR_InputNum input)
{
    //
    // Check the arguments.
    //
    ASSERT(XBAR_isBaseValid(base));

    //
    // lock the input in the INPUTSELECTLOCK register.
    //
    EALLOW;
    HWREG(base + XBAR_O_INPUTSELECTLOCK) =
            1UL << (uint16_t)input;
    EDIS;
}

//*****************************************************************************
//
//! Locks the Output X-BAR.
//!
//! \param base specifies the X-BAR base address.
//! This function locks the Output X-BAR.
//!
//! The \e base parameter can take base addresses
//! OUTPUTXBAR_BASE
//! or CLBOUTPUTXBAR_BASE.
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_lockOutput(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(XBAR_isBaseValid(base));

    //
    // Lock the Output X-BAR with the OUTPUTLOCK register.
    // Write key 0x5A5A to the KEY bits and 1 to LOCK bit.
    //
    EALLOW;

    HWREG(base + XBAR_O_OUTPUTLOCK) =
         ((uint32_t)0x5A5A << XBAR_OUTPUTLOCK_KEY_S) |
         (uint32_t)XBAR_OUTPUTLOCK_LOCK;

    EDIS;
}

//*****************************************************************************
//
//! Locks the ePWM X-BAR.
//!
//! This function locks the ePWM X-BAR.
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_lockEPWM(void)
{
    //
    // Lock the ePWM X-BAR with the TRIPLOCK register.
    // Write key 0x5A5A to the KEY bits and 1 to LOCK bit.
    //
    EALLOW;

    HWREG(EPWMXBAR_BASE + XBAR_O_TRIPLOCK) =
        ((uint32_t)0x5A5A << XBAR_TRIPLOCK_KEY_S) |
        (uint32_t)XBAR_TRIPLOCK_LOCK;
    EDIS;
}

//*****************************************************************************
//
//! Enables the CLB X-BAR mux values to be passed to an CLB module.
//!
//! \param auxSignal is the X-BAR output being configured.
//! \param muxes is a bit field of the muxes to be enabled.
//!
//! This function enables the mux values to be passed to the X-BAR auxSignal
//! signal. The \e auxSignal parameter is a value \b XBAR_AUXSIGy where y is
//! the number of the signal on the CLB.
//!
//! The \e muxes parameter is a bit field of the muxes being enabled where bit
//! 0 represents mux 0, bit 1 represents mux 1 and so on. Defines are provided
//! in the form of \b XBAR_MUXnn that can be logically OR'd together to
//! enable several muxes on an output at the same time.
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_enableCLBMux(XBAR_AuxSigNum auxSignal, uint32_t muxes)
{
    //
    // Set the enable bit.
    //
    EALLOW;

    HWREG(XBAR_CLB_EN_REG_BASE + (uint32_t)auxSignal) |= muxes;

    EDIS;
}

//*****************************************************************************
//
//! Disables the CLB X-BAR mux values to be passed to an CLB module.
//!
//! \param auxSignal is the X-BAR output being configured.
//! \param muxes is a bit field of the muxes to be disabled.
//!
//! This function disables the mux values to be passed to the X-BAR auxSignal
//! signal. The \e auxSignal parameter is a value \b XBAR_AUXSIGy where y is
//! the number of the signal on the CLB.
//!
//! The \e muxes parameter is a bit field of the muxes being disabled where bit
//! 0 represents mux 0, bit 1 represents mux 1 and so on. Defines are provided
//! in the form of \b XBAR_MUXnn that can be logically OR'd together to
//! disable several muxes on an output at the same time.
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_disableCLBMux(XBAR_AuxSigNum auxSignal, uint32_t muxes)
{
    //
    // Clear the enable bit.
    //
    EALLOW;

    HWREG(XBAR_CLB_EN_REG_BASE + (uint32_t)auxSignal) &= ~(muxes);

    EDIS;
}

//*****************************************************************************
//
//! Configures the polarity of an CLB X-BAR output.
//!
//! \param auxSignal is the X-BAR output being configured.
//! \param invert is a flag that determines whether the output is active-high
//! or active-low.
//!
//! This function inverts the CLB X-BAR auxSignal signal if the \e invert
//! parameter is \b true. If \e invert is \b false, the signal will be passed
//! as is. The \e auxSignal parameter is a value \b XBAR_AUXSIGy where y is
//! the number of the signal on the CLB X-BAR that is being configured.
//!
//! \return None.
//
//*****************************************************************************
static inline void
XBAR_invertCLBSignal(XBAR_AuxSigNum auxSignal, bool invert)
{
    //
    // Set or clear the polarity setting bit based on the invert parameter.
    //
    EALLOW;

    if(invert)
    {
        HWREGH(CLBXBAR_BASE + XBAR_O_AUXSIGOUTINV) |=
            0x1U << ((uint16_t)auxSignal / 2U);
    }
    else
    {
        HWREGH(CLBXBAR_BASE + XBAR_O_AUXSIGOUTINV) &=
            ~(0x1U << ((uint16_t)auxSignal / 2U));
    }

    EDIS;
}

//*****************************************************************************
//
//! Configures the Output X-BAR mux that determines the signals passed to an
//! output.
//!
//! \param base specifies the X-BAR Config Register base address.
//! \param output is the X-BAR output being configured.
//! \param muxConfig is mux configuration that specifies the signal.
//!
//! This function configures an Output X-BAR mux. This determines which
//! signal(s) should be passed through the X-BAR to a GPIO. The \e output
//! parameter is a value \b XBAR_OUTPUTy where y is a the output number
//! between 1 and 8 inclusive.
//!
//! The \e base parameter can take base addresses
//! OUTPUTXBAR_BASE
//! or CLBOUTPUTXBAR_BASE.
//!
//! The \e muxConfig parameter for OUTPUT XBAR is the mux configuration
//! value that specifies which signal will be passed from the mux. The
//! values have the format of \b XBAR_OUT_MUXnn_xx where the 'xx' is
//! the signal and nn is the mux number.
//!
//! The \e muxConfig parameter for the CLB OUTPUT XBAR have the similar
//! format as \b XBAR_OUT_MUXnn_xx where the 'xx' is the signal and nn is
//! the mux number.
//!
//! This function may be called for each mux of an output and their values will
//! be logically OR'd before being passed to the output signal. This means that
//! this function may be called, for example, with the argument
//! \b XBAR_OUT_MUX00_ECAP1_OUT and then with the argument
//! \b XBAR_OUT_MUX01_INPUTXBAR1, resulting in the values of MUX00 and MUX01
//! being logically OR'd if both are enabled. Calling the function twice for
//! the same mux on the output will result in the configuration in the second
//! call overwriting the first.
//!
//! \return None.
//
//*****************************************************************************
extern void
XBAR_setOutputMuxConfig(uint32_t base, XBAR_OutputNum output,
                        XBAR_OutputMuxConfig muxConfig);

//*****************************************************************************
//
//! Configures the ePWM X-BAR mux that determines the signals passed to an
//! ePWM module.
//!
//! \param trip is the X-BAR output being configured.
//! \param muxConfig is mux configuration that specifies the signal.
//!
//! This function configures an ePWM X-BAR mux. This determines which signal(s)
//! should be passed through the X-BAR to an ePWM module. The \e trip
//! parameter is a value \b XBAR_TRIPy where y is a the number of the trip
//! signal on the ePWM.
//!
//! The \e muxConfig parameter is the mux configuration value that specifies
//! which signal will be passed from the mux. The values have the format of
//! \b XBAR_EPWM_MUXnn_xx where the 'xx' is the signal and nn is the mux
//! number (0 through 31). The possible values are found in <tt>xbar.h</tt>
//!
//! This function may be called for each mux of an output and their values will
//! be logically OR'd before being passed to the trip signal. This means that
//! this function may be called, for example, with the argument
//! \b XBAR_EPWM_MUX00_ECAP1_OUT and then with the argument
//! \b XBAR_EPWM_MUX01_INPUTXBAR1, resulting in the values of MUX00 and MUX01
//! being logically OR'd if both are enabled. Calling the function twice for
//! the same mux on the output will result in the configuration in the second
//! call overwriting the first.
//!
//! \return None.
//
//*****************************************************************************
extern void
XBAR_setEPWMMuxConfig(XBAR_TripNum trip, XBAR_EPWMMuxConfig muxConfig);

//*****************************************************************************
//
//! Returns the status of the input latch.
//!
//! \param inputFlag is the X-BAR input latch being checked. Values are in the
//! format of /b XBAR_INPUT_FLG_XXXX where "XXXX" is name of the signal.
//!
//! \return Returns \b true if the X-BAR input corresponding to the
//! \e inputFlag has been triggered. If not, it will return \b false.
//
//*****************************************************************************
extern bool
XBAR_getInputFlagStatus(XBAR_InputFlag inputFlag);

//*****************************************************************************
//
//! Clears the input latch for the specified input latch.
//!
//! \param inputFlag is the X-BAR input latch being cleared.
//!
//! This function clears the Input X-BAR input latch. The input latch to be
//! cleared is specified by the \e inputFlag parameter.
//!
//! \return None.
//
//*****************************************************************************
extern void
XBAR_clearInputFlag(XBAR_InputFlag inputFlag);

//*****************************************************************************
//
//! Configures the CLB X-BAR mux that determines the signals passed to a
//! CLB module.
//!
//! \param auxSignal is the X-BAR output being configured.
//! \param muxConfig is mux configuration that specifies the signal.
//!
//! This function configures an CLB X-BAR mux. This determines which signal(s)
//! should be passed through the X-BAR to an CLB module. The \e auxSignal
//! parameter is a value \b XBAR_AUXSIGy where y is a the number of the
//! signal on the CLB.
//!
//! The \e muxConfig parameter is the mux configuration value that specifies
//! which signal will be passed from the mux. The values have the format of
//! \b XBAR_CLB_MUXnn_xx where the 'xx' is the signal and nn is the mux
//! number (0 through 31). The possible values are found in <tt>xbar.h</tt>
//!
//! This function may be called for each mux of an output and their values will
//! be logically OR'd before being passed to the signal. This means that
//! this function may be called, for example, with the argument
//! \b XBAR_CLB_MUX00_ECAP1_OUT and then with the argument
//! \b XBAR_CLB_MUX03_INPUTXBAR2, resulting in the values of MUX00 and MUX03
//! being logically OR'd if both are enabled. Calling the function twice for
//! the same mux on the output will result in the configuration in the second
//! call overwriting the first.
//!
//! \return None.
//
//*****************************************************************************
extern void XBAR_setCLBMuxConfig(XBAR_AuxSigNum auxSignal,
                                 XBAR_CLBMuxConfig muxConfig);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // XBAR_H
