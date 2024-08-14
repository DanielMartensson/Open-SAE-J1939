//###########################################################################
//
// FILE:   clb.h
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

#ifndef CLB_H
#define CLB_H

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
//! \addtogroup clb_api CLB
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_clb.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "cpu.h"
#include "debug.h"

//*****************************************************************************
//
// Address offsets from LOGICCFG to LOGICCTL and DATAEXCH register memory maps
//
//*****************************************************************************
#define CLB_LOGICCTL                0x0100U
#define CLB_DATAEXCH                0x0180U

//*****************************************************************************
//
// Address offsets for CLB-internal memory space
//
//*****************************************************************************
#define CLB_ADDR_COUNTER_0_LOAD     0x0U
#define CLB_ADDR_COUNTER_1_LOAD     0x1U
#define CLB_ADDR_COUNTER_2_LOAD     0x2U

#define CLB_ADDR_COUNTER_0_MATCH1   0x4U
#define CLB_ADDR_COUNTER_1_MATCH1   0x5U
#define CLB_ADDR_COUNTER_2_MATCH1   0x6U

#define CLB_ADDR_COUNTER_0_MATCH2   0x8U
#define CLB_ADDR_COUNTER_1_MATCH2   0x9U
#define CLB_ADDR_COUNTER_2_MATCH2   0xAU

#define CLB_ADDR_HLC_R0             0xCU
#define CLB_ADDR_HLC_R1             0xDU
#define CLB_ADDR_HLC_R2             0xEU
#define CLB_ADDR_HLC_R3             0xFU

#define CLB_ADDR_HLC_BASE           0x20U
#define CLB_NUM_HLC_INSTR           31U

//*****************************************************************************
//
// PUSH/PULL FIFO size (32-bit registers)
//
//*****************************************************************************
#define CLB_FIFO_SIZE               4U

//*****************************************************************************
//
// Key to enable writes to the CLB registers
//
//*****************************************************************************
#define CLB_LOCK_KEY                0x5A5AU

//*****************************************************************************
//
// Shift and masks needed by the API for Input selection
//
//*****************************************************************************
#define CLB_LCL_MUX_SEL_MISC_INPUT_SEL_M          0x20U
#define CLB_LCL_MUX_SEL_MISC_INPUT_SEL_S          28U
#define CLB_LCL_MUX_SEL_MISC_INPUT_SEL_BITM       (uint32_t)1U

//*****************************************************************************
//
//! Values that can be passed to control the CLB output enable signal. It can
//! be passed to CLB_setOutputMask() as the \e outputMask parameter.
//
//*****************************************************************************
#define CLB_OUTPUT_00 0x00000001U //!< Mask for CLB OUTPUT ENABLE/DISABLE 0
#define CLB_OUTPUT_01 0x00000002U //!< Mask for CLB OUTPUT ENABLE/DISABLE 1
#define CLB_OUTPUT_02 0x00000004U //!< Mask for CLB OUTPUT ENABLE/DISABLE 2
#define CLB_OUTPUT_03 0x00000008U //!< Mask for CLB OUTPUT ENABLE/DISABLE 3
#define CLB_OUTPUT_04 0x00000010U //!< Mask for CLB OUTPUT ENABLE/DISABLE 4
#define CLB_OUTPUT_05 0x00000020U //!< Mask for CLB OUTPUT ENABLE/DISABLE 5
#define CLB_OUTPUT_06 0x00000040U //!< Mask for CLB OUTPUT ENABLE/DISABLE 6
#define CLB_OUTPUT_07 0x00000080U //!< Mask for CLB OUTPUT ENABLE/DISABLE 7
#define CLB_OUTPUT_08 0x00000100U //!< Mask for CLB OUTPUT ENABLE/DISABLE 8
#define CLB_OUTPUT_09 0x00000200U //!< Mask for CLB OUTPUT ENABLE/DISABLE 9
#define CLB_OUTPUT_10 0x00000400U //!< Mask for CLB OUTPUT ENABLE/DISABLE 10
#define CLB_OUTPUT_11 0x00000800U //!< Mask for CLB OUTPUT ENABLE/DISABLE 11
#define CLB_OUTPUT_12 0x00001000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 12
#define CLB_OUTPUT_13 0x00002000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 13
#define CLB_OUTPUT_14 0x00004000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 14
#define CLB_OUTPUT_15 0x00008000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 15
#define CLB_OUTPUT_16 0x00010000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 16
#define CLB_OUTPUT_17 0x00020000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 17
#define CLB_OUTPUT_18 0x00040000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 18
#define CLB_OUTPUT_19 0x00080000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 19
#define CLB_OUTPUT_20 0x00100000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 20
#define CLB_OUTPUT_21 0x00200000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 21
#define CLB_OUTPUT_22 0x00400000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 22
#define CLB_OUTPUT_23 0x00800000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 23
#define CLB_OUTPUT_24 0x01000000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 24
#define CLB_OUTPUT_25 0x02000000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 25
#define CLB_OUTPUT_26 0x04000000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 26
#define CLB_OUTPUT_27 0x08000000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 27
#define CLB_OUTPUT_28 0x10000000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 28
#define CLB_OUTPUT_29 0x20000000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 29
#define CLB_OUTPUT_30 0x40000000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 30
#define CLB_OUTPUT_31 0x80000000U //!< Mask for CLB OUTPUT ENABLE/DISABLE 31

//*****************************************************************************
//
//! Values that can be passed to select CLB input signal
//
//*****************************************************************************
typedef enum
{
    CLB_IN0 = 0,                    //!< Input 0
    CLB_IN1 = 1,                    //!< Input 1
    CLB_IN2 = 2,                    //!< Input 2
    CLB_IN3 = 3,                    //!< Input 3
    CLB_IN4 = 4,                    //!< Input 4
    CLB_IN5 = 5,                    //!< Input 5
    CLB_IN6 = 6,                    //!< Input 6
    CLB_IN7 = 7                     //!< Input 7
} CLB_Inputs;

//*****************************************************************************
//
//! Values that can be passed to select CLB output signal. It can be passed to
//! CLB_configOutputLUT() as the \e outID parameter.
//
//*****************************************************************************
typedef enum
{
    CLB_OUT0 = 0,                   //!< Output 0
    CLB_OUT1 = 1,                   //!< Output 1
    CLB_OUT2 = 2,                   //!< Output 2
    CLB_OUT3 = 3,                   //!< Output 3
    CLB_OUT4 = 4,                   //!< Output 4
    CLB_OUT5 = 5,                   //!< Output 5
    CLB_OUT6 = 6,                   //!< Output 6
    CLB_OUT7 = 7                    //!< Output 7
} CLB_Outputs;

//*****************************************************************************
//
//! Values that can be passed to select CLB AOC signal. It can be passed to
//! CLB_configAOC() as the \e aocID parameter.
//! AOC is the Asynchronous Output Conditioning block.
//
//*****************************************************************************
typedef enum
{
    CLB_AOC0 = 0,                   //!< AOC 0
    CLB_AOC1 = 1,                   //!< AOC 1
    CLB_AOC2 = 2,                   //!< AOC 2
    CLB_AOC3 = 3,                   //!< AOC 3
    CLB_AOC4 = 4,                   //!< AOC 4
    CLB_AOC5 = 5,                   //!< AOC 5
    CLB_AOC6 = 6,                   //!< AOC 6
    CLB_AOC7 = 7                    //!< AOC 7
} CLB_AOCs;

//*****************************************************************************
//
//! Values that can be passed to set/clear CLB SW release signals. It can
//! be passed to CLB_writeSWReleaseControl() as the \e inID parameter.
//
//*****************************************************************************
typedef enum
{
    CLB_SW_RLS_CTRL0 = 0,           //!< SW RLS CTRL 0
    CLB_SW_RLS_CTRL1 = 1,           //!< SW RLS CTRL 1
    CLB_SW_RLS_CTRL2 = 2,           //!< SW RLS CTRL 2
    CLB_SW_RLS_CTRL3 = 3,           //!< SW RLS CTRL 3
    CLB_SW_RLS_CTRL4 = 4,           //!< SW RLS CTRL 4
    CLB_SW_RLS_CTRL5 = 5,           //!< SW RLS CTRL 5
    CLB_SW_RLS_CTRL6 = 6,           //!< SW RLS CTRL 6
    CLB_SW_RLS_CTRL7 = 7            //!< SW RLS CTRL 7
} CLB_SWReleaseCtrl;

//*****************************************************************************
//
//! Values that can be passed to set/clear CLB SW release signals. It can
//! be passed to CLB_writeSWGateControl() as the \e inID parameter.
//
//*****************************************************************************
typedef enum
{
    CLB_SW_GATE_CTRL0 = 0,           //!< SW GATE CTRL 0
    CLB_SW_GATE_CTRL1 = 1,           //!< SW GATE CTRL 1
    CLB_SW_GATE_CTRL2 = 2,           //!< SW GATE CTRL 2
    CLB_SW_GATE_CTRL3 = 3,           //!< SW GATE CTRL 3
    CLB_SW_GATE_CTRL4 = 4,           //!< SW GATE CTRL 4
    CLB_SW_GATE_CTRL5 = 5,           //!< SW GATE CTRL 5
    CLB_SW_GATE_CTRL6 = 6,           //!< SW GATE CTRL 6
    CLB_SW_GATE_CTRL7 = 7            //!< SW GATE CTRL 7
} CLB_SWGateCtrl;

//*****************************************************************************
//
//! Values that can be passed to select CLB counter. It can be passed to
//! CLB_configCounterLoadMatch() as the \e counterID parameter.
//
//*****************************************************************************
typedef enum
{
    CLB_CTR0 = 0,                   //!< Counter 0
    CLB_CTR1 = 1,                   //!< Counter 1
    CLB_CTR2 = 2                    //!< Counter 2
} CLB_Counters;

//*****************************************************************************
//
//! Values that can be passed to CLB_getRegister() as the \e registerID
//! parameter.
//
//*****************************************************************************
typedef enum
{
    CLB_REG_HLC_R0 = CLB_O_DBG_R0,  //!< HLC R0 register
    CLB_REG_HLC_R1 = CLB_O_DBG_R1,  //!< HLC R1 register
    CLB_REG_HLC_R2 = CLB_O_DBG_R2,  //!< HLC R2 register
    CLB_REG_HLC_R3 = CLB_O_DBG_R3,  //!< HLC R3 register
    CLB_REG_CTR_C0 = CLB_O_DBG_C0,  //!< Counter 0 register
    CLB_REG_CTR_C1 = CLB_O_DBG_C1,  //!< Counter 1 register
    CLB_REG_CTR_C2 = CLB_O_DBG_C2   //!< Counter 2 register
} CLB_Register;

//*****************************************************************************
//
//! Values that can be passed to CLB_selectInputFilter() as the \e filterType
//! parameter.
//
//*****************************************************************************
typedef enum
{
    CLB_FILTER_NONE         = 0,    //!< No filtering
    CLB_FILTER_RISING_EDGE  = 1,    //!< Rising edge detect
    CLB_FILTER_FALLING_EDGE = 2,    //!< Falling edge detect
    CLB_FILTER_ANY_EDGE     = 3     //!< Any edge detect
} CLB_FilterType;

//*****************************************************************************
//
//! Values that can be passed to CLB_configGPInputMux() as the \e gpMuxCfg
//! parameter.
//
//*****************************************************************************
typedef enum
{
    CLB_GP_IN_MUX_EXTERNAL  = 0,    //!< Use external input path
    CLB_GP_IN_MUX_GP_REG    = 1     //!< Use CLB_GP_REG bit value as input
} CLB_GPInputMux;

//*****************************************************************************
//
//! Values that can be passed to CLB_configLocalInputMux() as the
//! \e localMuxCfg parameter.
//
//*****************************************************************************
typedef enum
{
    CLB_LOCAL_IN_MUX_GLOBAL_IN          = 0,   //!< Global input mux selection
    CLB_LOCAL_IN_MUX_EPWM_DCAEVT1       = 1,   //!< EPWMx DCAEVT1
    CLB_LOCAL_IN_MUX_EPWM_DCAEVT2       = 2,   //!< EPWMx DCAEVT2
    CLB_LOCAL_IN_MUX_EPWM_DCBEVT1       = 3,   //!< EPWMx DCBEVT1
    CLB_LOCAL_IN_MUX_EPWM_DCBEVT2       = 4,   //!< EPWMx DCBEVT2
    CLB_LOCAL_IN_MUX_EPWM_DCAH          = 5,   //!< EPWMx DCAH
    CLB_LOCAL_IN_MUX_EPWM_DCAL          = 6,   //!< EPWMx DCAL
    CLB_LOCAL_IN_MUX_EPWM_DCBH          = 7,   //!< EPWMx DCBH
    CLB_LOCAL_IN_MUX_EPWM_DCBL          = 8,   //!< EPWMx DCBL
    CLB_LOCAL_IN_MUX_EPWM_OST           = 9,   //!< EPWMx OST
    CLB_LOCAL_IN_MUX_EPWM_CBC           = 10,  //!< EPWMx CBC
    CLB_LOCAL_IN_MUX_ECAP_ECAPIN        = 11,  //!< ECAPx ECAPIN
    CLB_LOCAL_IN_MUX_ECAP_ECAP_OUT      = 12,  //!< ECAPx ECAP_OUT
    CLB_LOCAL_IN_MUX_ECAP_ECAP_OUT_EN   = 13,  //!< ECAPx ECAP_OUT_EN
    CLB_LOCAL_IN_MUX_ECAP_CEVT1         = 14,  //!< ECAPx CEVT1
    CLB_LOCAL_IN_MUX_ECAP_CEVT2         = 15,  //!< ECAPx CEVT2
    CLB_LOCAL_IN_MUX_ECAP_CEVT3         = 16,  //!< ECAPx CEVT3
    CLB_LOCAL_IN_MUX_ECAP_CEVT4         = 17,  //!< ECAPx CEVT4
    CLB_LOCAL_IN_MUX_EQEP_EQEPA         = 18,  //!< EQEPx EQEPA (CLB 1-4)
    CLB_LOCAL_IN_MUX_FSI_DATA_PKT_RCVD  = 18,  //!< FSI_DATA_PKT_RCVD(CLB 5-8)
    CLB_LOCAL_IN_MUX_EQEP_EQEPB         = 19,  //!< EQEPx EQEPB (CLB 1-4)
    CLB_LOCAL_IN_MUX_FSI_ERROR_PKT_RCVD = 19,  //!< FSI_ERROR_PKT_RCVD(CLB 5-8)
    CLB_LOCAL_IN_MUX_EQEP_EQEPI         = 20,  //!< EQEPx EQEPI (CLB 1-4)
    CLB_LOCAL_IN_MUX_FSI_PING_PKT_RCVD  = 20,  //!< FSI_PING_PKT_RCVD(CLB 5-8)
    CLB_LOCAL_IN_MUX_EQEP_EQEPS         = 21,  //!< EQEPx EQEPS (CLB 1-4)
    CLB_LOCAL_IN_MUX_CPU2_HALT          = 21,  //!< CPU2.HALT (CLB 5-8)
    CLB_LOCAL_IN_MUX_CPU1_TBCLKSYNC     = 22,  //!< CPU1.TBCLKSYNC
    CLB_LOCAL_IN_MUX_CPU2_TBCLKSYNC     = 23,  //!< CPU2.TBCLKSYNC
    CLB_LOCAL_IN_MUX_CPU1_HALT          = 24,  //!< CPU1.HALT
    CLB_LOCAL_IN_MUX_SPIPICO_CONTROLLER = 25,  //!< SPIPICO Controller Output
    CLB_LOCAL_IN_MUX_SPICLK             = 26,  //!< SPI Clock
    CLB_LOCAL_IN_MUX_SPIPICO_PERIPHERAL = 27,  //!< SPIPICO Peripheral Input
    CLB_LOCAL_IN_MUX_SPIPTE             = 28,  //!< SPI PTE
    CLB_LOCAL_IN_MUX_SCI_TX             = 29,  //!< SCI TX
    CLB_LOCAL_IN_MUX_SPIPOCI_OUT        = 30,  //!< SPIPOCI(OUT)
    CLB_LOCAL_IN_MUX_CLB_PSCLK          = 31,  //!< CLB prescaled clock
    CLB_LOCAL_IN_MUX_EPWM9A             = 32,  //!< EPWM9A (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM13A            = 32,  //!< EPWM13A (CLB 5-8)
    CLB_LOCAL_IN_MUX_EPWM9A_OE          = 33,  //!< EPWM9A trip output (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM13A_OE         = 33,  //!< EPWM13A trip output (CLB 5-8)
    CLB_LOCAL_IN_MUX_EPWM9B             = 34,  //!< EPWM9B (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM13B            = 34,  //!< EPWM13B (CLB 5-8)
    CLB_LOCAL_IN_MUX_EPWM9B_OE          = 35,  //!< EPWM9B trip output (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM13B_OE         = 35,  //!< EPWM13B trip output (CLB 5-8)
    CLB_LOCAL_IN_MUX_EPWM10A            = 36,  //!< EPWM10A (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM14A            = 36,  //!< EPWM14A (CLB 5-8)
    CLB_LOCAL_IN_MUX_EPWM10A_OE         = 37,  //!< EPWM10A trip output (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM14A_OE         = 37,  //!< EPWM14A trip output (CLB 5-8)
    CLB_LOCAL_IN_MUX_EPWM10B            = 38,  //!< EPWM10B (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM14B            = 38,  //!< EPWM14B (CLB 5-8)
    CLB_LOCAL_IN_MUX_EPWM10B_OE         = 39,  //!< EPWM10B trip output (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM14B_OE         = 39,  //!< EPWM14B trip output (CLB 5-8)
    CLB_LOCAL_IN_MUX_EPWM11A            = 40,  //!< EPWM11A (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM15A            = 40,  //!< EPWM15A (CLB 5-8)
    CLB_LOCAL_IN_MUX_EPWM11A_OE         = 41,  //!< EPWM11A trip output (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM15A_OE         = 41,  //!< EPWM15A trip output (CLB 5-8)
    CLB_LOCAL_IN_MUX_EPWM11B            = 42,  //!< EPWM11B (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM15B            = 42,  //!< EPWM15B (CLB 5-8)
    CLB_LOCAL_IN_MUX_EPWM11B_OE         = 43,  //!< EPWM11B trip output (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM15B_OE         = 43,  //!< EPWM15B trip output (CLB 5-8)
    CLB_LOCAL_IN_MUX_EPWM12A            = 44,  //!< EPWM12A (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM16A            = 44,  //!< EPWM16A (CLB 5-8)
    CLB_LOCAL_IN_MUX_EPWM12A_OE         = 45,  //!< EPWM12A trip output (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM16A_OE         = 45,  //!< EPWM16A trip output (CLB 5-8)
    CLB_LOCAL_IN_MUX_EPWM12B            = 46,  //!< EPWM12B (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM16B            = 46,  //!< EPWM16B (CLB 5-8)
    CLB_LOCAL_IN_MUX_EPWM12B_OE         = 47,  //!< EPWM12B trip output (CLB 1-4)
    CLB_LOCAL_IN_MUX_EPWM16B_OE         = 47,  //!< EPWM16B trip output (CLB 5-8)
    CLB_LOCAL_IN_MUX_INPUT1             = 48,  //!< CLBINPUTXBAR INPUT1
    CLB_LOCAL_IN_MUX_INPUT2             = 49,  //!< CLBINPUTXBAR INPUT2
    CLB_LOCAL_IN_MUX_INPUT3             = 50,  //!< CLBINPUTXBAR INPUT3
    CLB_LOCAL_IN_MUX_INPUT4             = 51,  //!< CLBINPUTXBAR INPUT4
    CLB_LOCAL_IN_MUX_INPUT5             = 52,  //!< CLBINPUTXBAR INPUT5
    CLB_LOCAL_IN_MUX_INPUT6             = 53,  //!< CLBINPUTXBAR INPUT6
    CLB_LOCAL_IN_MUX_INPUT7             = 54,  //!< CLBINPUTXBAR INPUT7
    CLB_LOCAL_IN_MUX_INPUT8             = 55,  //!< CLBINPUTXBAR INPUT8
    CLB_LOCAL_IN_MUX_INPUT9             = 56,  //!< CLBINPUTXBAR INPUT9
    CLB_LOCAL_IN_MUX_INPUT10            = 57,  //!< CLBINPUTXBAR INPUT10
    CLB_LOCAL_IN_MUX_INPUT11            = 58,  //!< CLBINPUTXBAR INPUT11
    CLB_LOCAL_IN_MUX_INPUT12            = 59,  //!< CLBINPUTXBAR INPUT12
    CLB_LOCAL_IN_MUX_INPUT13            = 60,  //!< CLBINPUTXBAR INPUT13
    CLB_LOCAL_IN_MUX_INPUT14            = 61,  //!< CLBINPUTXBAR INPUT14
    CLB_LOCAL_IN_MUX_INPUT15            = 62,  //!< CLBINPUTXBAR INPUT15
    CLB_LOCAL_IN_MUX_INPUT16            = 63   //!< CLBINPUTXBAR INPUT16
} CLB_LocalInputMux;

//*****************************************************************************
//
//! Values that can be passed to CLB_configGlobalInputMux() as the
//! \e globalMuxCfg parameter.
//
//*****************************************************************************
typedef enum
{
    CLB_GLOBAL_IN_MUX_EPWM1A            = 0,   //!< EPWM1A (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5A            = 0,   //!< EPWM5A (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM1A_OE         = 1,   //!< EPWM1A trip output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5A_OE         = 1,   //!< EPWM5A trip output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM1B            = 2,   //!< EPWM1B (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5B            = 2,   //!< EPWM5B (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM1B_OE         = 3,   //!< EPWM1B trip output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5B_OE         = 3,   //!< EPWM5B trip output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM1_CTR_ZERO    = 4,   //!< EPWM1 TBCTR = Zero (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5_CTR_ZERO    = 4,   //!< EPWM5 TBCTR = Zero (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM1_CTR_PRD     = 5,   //!< EPWM1 TBCTR = TBPRD (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5_CTR_PRD     = 5,   //!< EPWM5 TBCTR = TBPRD (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM1_CTRDIR      = 6,   //!< EPWM1 CTRDIR (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5_CTRDIR      = 6,   //!< EPWM5 CTRDIR (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM1_TBCLK       = 7,   //!< EPWM1 TBCLK (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5_TBCLK       = 7,   //!< EPWM5 TBCLK (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM1_CTR_CMPA    = 8,   //!< EPWM1 TBCTR = CMPA (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5_CTR_CMPA    = 8,   //!< EPWM5 TBCTR = CMPA (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM1_CTR_CMPB    = 9,   //!< EPWM1 TBCTR = CMPB (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5_CTR_CMPB    = 9,   //!< EPWM5 TBCTR = CMPB (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM1_CTR_CMPC    = 10,  //!< EPWM1 TBCTR = CMPC (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5_CTR_CMPC    = 10,  //!< EPWM5 TBCTR = CMPC (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM1_CTR_CMPD    = 11,  //!< EPWM1 TBCTR = CMPD (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5_CTR_CMPD    = 11,  //!< EPWM5 TBCTR = CMPD (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM1A_AQ         = 12,  //!< EPWM1A AQ submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5A_AQ         = 12,  //!< EPWM5A AQ submodule output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM1B_AQ         = 13,  //!< EPWM1B AQ submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5B_AQ         = 13,  //!< EPWM5B AQ submodule output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM1A_DB         = 14,  //!< EPWM1A DB submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5A_DB         = 14,  //!< EPWM5A DB submodule output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM1B_DB         = 15,  //!< EPWM1B DB submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM5B_DB         = 15,  //!< EPWM5B DB submodule output (CLB 5-8)

    CLB_GLOBAL_IN_MUX_EPWM2A            = 16,   //!< EPWM2A (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6A            = 16,   //!< EPWM6A (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM2A_OE         = 17,   //!< EPWM2A trip output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6A_OE         = 17,   //!< EPWM6A trip output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM2B            = 18,   //!< EPWM2B (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6B            = 18,   //!< EPWM6B (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM2B_OE         = 19,   //!< EPWM2B trip output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6B_OE         = 19,   //!< EPWM6B trip output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM2_CTR_ZERO    = 20,   //!< EPWM2 TBCTR = Zero (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6_CTR_ZERO    = 20,   //!< EPWM6 TBCTR = Zero (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM2_CTR_PRD     = 21,   //!< EPWM2 TBCTR = TBPRD (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6_CTR_PRD     = 21,   //!< EPWM6 TBCTR = TBPRD (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM2_CTRDIR      = 22,   //!< EPWM2 CTRDIR (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6_CTRDIR      = 22,   //!< EPWM6 CTRDIR (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM2_TBCLK       = 23,   //!< EPWM2 TBCLK (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6_TBCLK       = 23,   //!< EPWM6 TBCLK (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM2_CTR_CMPA    = 24,   //!< EPWM2 TBCTR = CMPA (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6_CTR_CMPA    = 24,   //!< EPWM6 TBCTR = CMPA (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM2_CTR_CMPB    = 25,   //!< EPWM2 TBCTR = CMPB (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6_CTR_CMPB    = 25,   //!< EPWM6 TBCTR = CMPB (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM2_CTR_CMPC    = 26,  //!< EPWM2 TBCTR = CMPC (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6_CTR_CMPC    = 26,  //!< EPWM6 TBCTR = CMPC (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM2_CTR_CMPD    = 27,  //!< EPWM2 TBCTR = CMPD (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6_CTR_CMPD    = 27,  //!< EPWM6 TBCTR = CMPD (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM2A_AQ         = 28,  //!< EPWM2A AQ submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6A_AQ         = 28,  //!< EPWM6A AQ submodule output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM2B_AQ         = 29,  //!< EPWM2B AQ submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6B_AQ         = 29,  //!< EPWM6B AQ submodule output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM2A_DB         = 30,  //!< EPWM2A DB submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6A_DB         = 30,  //!< EPWM6A DB submodule output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM2B_DB         = 31,  //!< EPWM2B DB submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM6B_DB         = 31,  //!< EPWM6B DB submodule output (CLB 5-8)

    CLB_GLOBAL_IN_MUX_EPWM3A            = 32,   //!< EPWM3A (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7A            = 32,   //!< EPWM7A (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM3A_OE         = 33,   //!< EPWM3A trip output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7A_OE         = 33,   //!< EPWM7A trip output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM3B            = 34,   //!< EPWM3B (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7B            = 34,   //!< EPWM7B (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM3B_OE         = 35,   //!< EPWM3B trip output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7B_OE         = 35,   //!< EPWM7B trip output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM3_CTR_ZERO    = 36,   //!< EPWM3 TBCTR = Zero (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7_CTR_ZERO    = 36,   //!< EPWM7 TBCTR = Zero (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM3_CTR_PRD     = 37,   //!< EPWM3 TBCTR = TBPRD (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7_CTR_PRD     = 37,   //!< EPWM7 TBCTR = TBPRD (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM3_CTRDIR      = 38,   //!< EPWM3 CTRDIR (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7_CTRDIR      = 38,   //!< EPWM7 CTRDIR (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM3_TBCLK       = 39,   //!< EPWM3 TBCLK (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7_TBCLK       = 39,   //!< EPWM7 TBCLK (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM3_CTR_CMPA    = 40,   //!< EPWM3 TBCTR = CMPA (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7_CTR_CMPA    = 40,   //!< EPWM7 TBCTR = CMPA (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM3_CTR_CMPB    = 41,   //!< EPWM3 TBCTR = CMPB (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7_CTR_CMPB    = 41,   //!< EPWM7 TBCTR = CMPB (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM3_CTR_CMPC    = 42,  //!< EPWM3 TBCTR = CMPC (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7_CTR_CMPC    = 42,  //!< EPWM7 TBCTR = CMPC (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM3_CTR_CMPD    = 43,  //!< EPWM3 TBCTR = CMPD (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7_CTR_CMPD    = 43,  //!< EPWM7 TBCTR = CMPD (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM3A_AQ         = 44,  //!< EPWM3A AQ submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7A_AQ         = 44,  //!< EPWM7A AQ submodule output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM3B_AQ         = 45,  //!< EPWM3B AQ submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7B_AQ         = 45,  //!< EPWM7B AQ submodule output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM3A_DB         = 46,  //!< EPWM3A DB submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7A_DB         = 46,  //!< EPWM7A DB submodule output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM3B_DB         = 47,  //!< EPWM3B DB submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM7B_DB         = 47,  //!< EPWM7B DB submodule output (CLB 5-8)

    CLB_GLOBAL_IN_MUX_EPWM4A            = 48,   //!< EPWM4A (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8A            = 48,   //!< EPWM8A (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM4A_OE         = 49,   //!< EPWM4A trip output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8A_OE         = 49,   //!< EPWM8A trip output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM4B            = 50,   //!< EPWM4B (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8B            = 50,   //!< EPWM8B (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM4B_OE         = 51,   //!< EPWM4B trip output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8B_OE         = 51,   //!< EPWM8B trip output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM4_CTR_ZERO    = 52,   //!< EPWM4 TBCTR = Zero (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8_CTR_ZERO    = 52,   //!< EPWM8 TBCTR = Zero (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM4_CTR_PRD     = 53,   //!< EPWM4 TBCTR = TBPRD (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8_CTR_PRD     = 53,   //!< EPWM8 TBCTR = TBPRD (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM4_CTRDIR      = 54,   //!< EPWM4 CTRDIR (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8_CTRDIR      = 54,   //!< EPWM8 CTRDIR (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM4_TBCLK       = 55,   //!< EPWM4 TBCLK (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8_TBCLK       = 55,   //!< EPWM8 TBCLK (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM4_CTR_CMPA    = 56,   //!< EPWM4 TBCTR = CMPA (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8_CTR_CMPA    = 56,   //!< EPWM8 TBCTR = CMPA (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM4_CTR_CMPB    = 57,   //!< EPWM4 TBCTR = CMPB (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8_CTR_CMPB    = 57,   //!< EPWM8 TBCTR = CMPB (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM4_CTR_CMPC    = 58,  //!< EPWM4 TBCTR = CMPC (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8_CTR_CMPC    = 58,  //!< EPWM8 TBCTR = CMPC (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM4_CTR_CMPD    = 59,  //!< EPWM4 TBCTR = CMPD (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8_CTR_CMPD    = 59,  //!< EPWM8 TBCTR = CMPD (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM4A_AQ         = 60,  //!< EPWM4A AQ submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8A_AQ         = 60,  //!< EPWM8A AQ submodule output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM4B_AQ         = 61,  //!< EPWM4B AQ submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8B_AQ         = 61,  //!< EPWM8B AQ submodule output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM4A_DB         = 62,  //!< EPWM4A DB submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8A_DB         = 62,  //!< EPWM8A DB submodule output (CLB 5-8)
    CLB_GLOBAL_IN_MUX_EPWM4B_DB         = 63,  //!< EPWM4B DB submodule output (CLB 1-4)
    CLB_GLOBAL_IN_MUX_EPWM8B_DB         = 63,  //!< EPWM8B DB submodule output (CLB 5-8)

    CLB_GLOBAL_IN_MUX_CLB_AUXSIG0       = 64,  //!< CLB X-BAR AUXSIG0
    CLB_GLOBAL_IN_MUX_CLB_AUXSIG1       = 65,  //!< CLB X-BAR AUXSIG1
    CLB_GLOBAL_IN_MUX_CLB_AUXSIG2       = 66,  //!< CLB X-BAR AUXSIG2
    CLB_GLOBAL_IN_MUX_CLB_AUXSIG3       = 67,  //!< CLB X-BAR AUXSIG3
    CLB_GLOBAL_IN_MUX_CLB_AUXSIG4       = 68,  //!< CLB X-BAR AUXSIG4
    CLB_GLOBAL_IN_MUX_CLB_AUXSIG5       = 69,  //!< CLB X-BAR AUXSIG5
    CLB_GLOBAL_IN_MUX_CLB_AUXSIG6       = 70,  //!< CLB X-BAR AUXSIG6
    CLB_GLOBAL_IN_MUX_CLB_AUXSIG7       = 71,  //!< CLB X-BAR AUXSIG7

    CLB_GLOBAL_IN_MUX_CLB1_OUT16         = 72,  //!< CLB1 OUT16 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB5_OUT16         = 72,  //!< CLB5 OUT16 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CLB1_OUT17         = 73,  //!< CLB1 OUT17 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB5_OUT17         = 73,  //!< CLB5 OUT17 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CLB1_OUT18         = 74,  //!< CLB1 OUT18 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB5_OUT18         = 74,  //!< CLB5 OUT18 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CLB1_OUT19         = 75,  //!< CLB1 OUT19 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB5_OUT19         = 75,  //!< CLB5 OUT19 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CLB1_OUT20         = 76,  //!< CLB1 OUT20 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB5_OUT20         = 76,  //!< CLB5 OUT20 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CLB1_OUT21         = 77,  //!< CLB1 OUT21 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB5_OUT21         = 77,  //!< CLB5 OUT21 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CLB1_OUT22         = 78,  //!< CLB1 OUT22 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB5_OUT22         = 78,  //!< CLB5 OUT22 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CLB1_OUT23         = 79,  //!< CLB1 OUT23 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB5_OUT23         = 79,  //!< CLB5 OUT23 (CLB 5-8)

    CLB_GLOBAL_IN_MUX_CLB2_OUT16         = 80,  //!< CLB2 OUT16 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB6_OUT16         = 80,  //!< CLB6 OUT16 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CLB2_OUT17         = 81,  //!< CLB2 OUT17 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB6_OUT17         = 81,  //!< CLB6 OUT17 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CLB2_OUT18         = 82,  //!< CLB2 OUT18 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB6_OUT18         = 82,  //!< CLB6 OUT18 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CLB2_OUT19         = 83,  //!< CLB2 OUT19 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB6_OUT19         = 83,  //!< CLB6 OUT19 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CLB2_OUT20         = 84,  //!< CLB2 OUT20 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB6_OUT20         = 84,  //!< CLB6 OUT20 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CLB2_OUT21         = 85,  //!< CLB2 OUT21 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB6_OUT21         = 85,  //!< CLB6 OUT21 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CLB2_OUT22         = 86,  //!< CLB2 OUT22 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB6_OUT22         = 86,  //!< CLB6 OUT22 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CLB2_OUT23         = 87,  //!< CLB2 OUT23 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CLB6_OUT23         = 87,  //!< CLB6 OUT23 (CLB 5-8)

    CLB_GLOBAL_IN_MUX_CLB3_OUT16         = 88,  //!< CLB3 OUT16
    CLB_GLOBAL_IN_MUX_CLB3_OUT17         = 89,  //!< CLB3 OUT17
    CLB_GLOBAL_IN_MUX_CLB3_OUT18         = 90,  //!< CLB3 OUT18
    CLB_GLOBAL_IN_MUX_CLB3_OUT19         = 91,  //!< CLB3 OUT19
    CLB_GLOBAL_IN_MUX_CLB3_OUT20         = 92,  //!< CLB3 OUT20
    CLB_GLOBAL_IN_MUX_CLB3_OUT21         = 93,  //!< CLB3 OUT21
    CLB_GLOBAL_IN_MUX_CLB3_OUT22         = 94,  //!< CLB3 OUT22
    CLB_GLOBAL_IN_MUX_CLB3_OUT23         = 95,  //!< CLB3 OUT23

    CLB_GLOBAL_IN_MUX_CLB4_OUT16         = 96,   //!< CLB4 OUT16
    CLB_GLOBAL_IN_MUX_CLB4_OUT17         = 97,   //!< CLB4 OUT17
    CLB_GLOBAL_IN_MUX_CLB4_OUT18         = 98,   //!< CLB4 OUT18
    CLB_GLOBAL_IN_MUX_CLB4_OUT19         = 99,   //!< CLB4 OUT19
    CLB_GLOBAL_IN_MUX_CLB4_OUT20         = 100,  //!< CLB4 OUT20
    CLB_GLOBAL_IN_MUX_CLB4_OUT21         = 101,  //!< CLB4 OUT21
    CLB_GLOBAL_IN_MUX_CLB4_OUT22         = 102,  //!< CLB4 OUT22
    CLB_GLOBAL_IN_MUX_CLB4_OUT23         = 103,  //!< CLB4 OUT23

    CLB_GLOBAL_IN_MUX_CPU1_ERAD_EVENT0       = 104,  //!< CPU1 ERAD Event 0 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CPU2_ERAD_EVENT0       = 104,  //!< CPU2 ERAD Event 0 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CPU1_ERAD_EVENT1       = 105,  //!< CPU1 ERAD Event 1 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CPU2_ERAD_EVENT1       = 105,  //!< CPU2 ERAD Event 1 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CPU1_ERAD_EVENT2       = 106,  //!< CPU1 ERAD Event 2 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CPU2_ERAD_EVENT2       = 106,  //!< CPU2 ERAD Event 2 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CPU1_ERAD_EVENT3       = 107,  //!< CPU1 ERAD Event 3 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CPU2_ERAD_EVENT3       = 107,  //!< CPU2 ERAD Event 3 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CPU1_ERAD_EVENT4       = 108,  //!< CPU1 ERAD Event 4 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CPU2_ERAD_EVENT4       = 108,  //!< CPU2 ERAD Event 4 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CPU1_ERAD_EVENT5       = 109,  //!< CPU1 ERAD Event 5 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CPU2_ERAD_EVENT5       = 109,  //!< CPU2 ERAD Event 5 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CPU1_ERAD_EVENT6       = 110,  //!< CPU1 ERAD Event 6 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CPU2_ERAD_EVENT6       = 110,  //!< CPU2 ERAD Event 6 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_CPU1_ERAD_EVENT7       = 111,  //!< CPU1 ERAD Event 7 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_CPU2_ERAD_EVENT7       = 111,  //!< CPU2 ERAD Event 7 (CLB 5-8)

    CLB_GLOBAL_IN_MUX_FSIRXA_DATA_PACKET_RCVD  = 112,  //!< FSIRXA Data Packet Received (CLB 1-4)
    CLB_GLOBAL_IN_MUX_FSIRXA_PING_TAG_MATCH    = 112,  //!< FSIRXA PING TAG Match (CLB 5-8)
    CLB_GLOBAL_IN_MUX_FSIRXA_ERROR_PACKET_RCVD = 113,  //!< FSIRXA Error Packet Received (CLB 1-4)
    CLB_GLOBAL_IN_MUX_FSIRXA_DATA_TAG_MATCH    = 113,  //!< FSIRXA DATA TAG Match (CLB 5-8)
    CLB_GLOBAL_IN_MUX_FSIRXA_PING_PACKET_RCVD  = 114,  //!< FSIRXA PING Packet Received (CLB 1-4)
    CLB_GLOBAL_IN_MUX_FSIRXA_ERROR_TAG_MATCH   = 114,  //!< FSIRXA ERROR TAG Match (CLB 5-8)
    CLB_GLOBAL_IN_MUX_FSIRXA_OUTPUT_FRAME_DONE = 115,  //!< FSIRXA Output Frame Done (CLB 1-4)
    CLB_GLOBAL_IN_MUX_FSIRXB_PING_TAG_MATCH    = 115,  //!< FSIRXB PING TAG Match (CLB 5-8)
    CLB_GLOBAL_IN_MUX_FSIRXA_PACKET_TAG0       = 116,  //!< FSIRXA Packet TAG0 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_FSIRXB_DATA_TAG_MATCH    = 116,  //!< FSIRXB DATA TAG Match (CLB 5-8)
    CLB_GLOBAL_IN_MUX_FSIRXA_PACKET_TAG1       = 117,  //!< FSIRXA Packet TAG1 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_FSIRXB_ERROR_TAG_MATCH   = 117,  //!< FSIRXB ERROR TAG Match (CLB 5-8)
    CLB_GLOBAL_IN_MUX_FSIRXA_PACKET_TAG2       = 118,  //!< FSIRXA Packet TAG2 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_ECAT_SOF                 = 118,  //!< ECAT Start of Frame (CLB 5-8)
    CLB_GLOBAL_IN_MUX_FSIRXA_PACKET_TAG3       = 119,  //!< FSIRXA Packet TAG3 (CLB 1-4)
    CLB_GLOBAL_IN_MUX_ECAT_EOF                 = 119,  //!< ECAT End of Frame (CLB 5-8)

    CLB_GLOBAL_IN_MUX_SPI1_SPICLK              = 120,  //!< SPI1 SPICLK OUT (CLB 1-4)
    CLB_GLOBAL_IN_MUX_SPI3_SPICLK              = 120,  //!< SPI3 SPICLK OUT (CLB 5-8)
    CLB_GLOBAL_IN_MUX_SPI1_SPIPOCI_CONTROLLER  = 121,  //!< SPI1 SPIPOCI Controller IN (CLB 1-4)
    CLB_GLOBAL_IN_MUX_SPI3_SPIPOCI_CONTROLLER  = 121,  //!< SPI3 SPIPOCI Controller IN (CLB 5-8)
    CLB_GLOBAL_IN_MUX_SPI1_SPIPTE              = 122,  //!< SPI1 SPIPTE OUT (CLB 1-4)
    CLB_GLOBAL_IN_MUX_SPI3_SPIPTE              = 122,  //!< SPI3 SPIPTE OUT (CLB 5-8)
    CLB_GLOBAL_IN_MUX_SPI2_SPICLK              = 123,  //!< SPI2 SPICLK OUT (CLB 1-4)
    CLB_GLOBAL_IN_MUX_SPI4_SPICLK              = 123,  //!< SPI4 SPICLK OUT (CLB 5-8)
    CLB_GLOBAL_IN_MUX_SPI2_SPIPOCI_CONTROLLER  = 124,  //!< SPI2 SPIPOCI Controller IN (CLB 1-4)
    CLB_GLOBAL_IN_MUX_SPI4_SPIPOCI_CONTROLLER  = 124,  //!< SPI4 SPIPOCI Controller IN (CLB 5-8)
    CLB_GLOBAL_IN_MUX_SPI2_SPIPTE              = 125,  //!< SPI2 SPIPTE OUT (CLB 1-4)
    CLB_GLOBAL_IN_MUX_SPI4_SPIPTE              = 125,  //!< SPI4 SPIPTE OUT (CLB 5-8)

    CLB_GLOBAL_IN_MUX_CPU2_HALT             = 126,  //!< CPU2 HALT (CLB 1-4)
    CLB_GLOBAL_IN_MUX_ECAT_SYNC0            = 126,  //!< ECAT Sync 0 (CLB 5-8)
    CLB_GLOBAL_IN_MUX_ECAT_SYNC1            = 127,  //!< ECAT Sync 1 (CLB 5-8)
} CLB_GlobalInputMux;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//
//!
//! Checks the CLB base address.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function determines if a CLB base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool CLB_isBaseValid(uint32_t base)
{
    return(
           (base == CLB1_BASE) ||
           (base == CLB2_BASE) ||
           (base == CLB3_BASE) ||
           (base == CLB4_BASE) ||
           (base == CLB5_BASE) ||
           (base == CLB6_BASE) ||
           (base == CLB7_BASE) ||
           (base == CLB8_BASE)
          );
}

//*****************************************************************************
//
//!
//! Checks the CLB internal memory address.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function determines if a CLB base address is valid.
//!
//! \return Returns \b true if the address is valid and \b false otherwise.
//
//*****************************************************************************
static inline bool CLB_isAddressValid(uint32_t address)
{
    return(address <= (CLB_ADDR_HLC_BASE + CLB_NUM_HLC_INSTR));
}
#endif

//*****************************************************************************
//
//! Set global enable.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function enables the CLB via global enable register.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_enableCLB(uint32_t base)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREGH(base + CLB_LOGICCTL + CLB_O_LOAD_EN) |= CLB_LOAD_EN_GLOBAL_EN;
    EDIS;
}

//*****************************************************************************
//
//! Clear global enable.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function disables the CLB via global enable register.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_disableCLB(uint32_t base)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREGH(base + CLB_LOGICCTL + CLB_O_LOAD_EN) &= ~CLB_LOAD_EN_GLOBAL_EN;
    EDIS;
}

//*****************************************************************************
//
//! Enable HLC NMI.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function enables the CLB HLC NMI.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_enableNMI(uint32_t base)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREGH(base + CLB_LOGICCTL + CLB_O_LOAD_EN) |= CLB_LOAD_EN_NMI_EN;
    EDIS;
}

//*****************************************************************************
//
//! Disable HLC NMI.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function disables the CLB HLC NMI.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_disableNMI(uint32_t base)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREGH(base + CLB_LOGICCTL + CLB_O_LOAD_EN) &= ~CLB_LOAD_EN_NMI_EN;
    EDIS;
}

//*****************************************************************************
//
//! Configure Clock Prescalar.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function enables and configures the CLB Clock Precalar.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_configureClockPrescalar(uint32_t base, uint16_t prescl)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREG(base + CLB_LOGICCTL + CLB_O_PRESCALE_CTRL) =
        (HWREG(base + CLB_LOGICCTL + CLB_O_PRESCALE_CTRL) &
         ~(CLB_PRESCALE_CTRL_PRESCALE_M)) |
        ((uint32_t)prescl << CLB_PRESCALE_CTRL_PRESCALE_S);
    HWREG(base + CLB_LOGICCTL + CLB_O_PRESCALE_CTRL) |= CLB_PRESCALE_CTRL_CLKEN;
    EDIS;
}

//*****************************************************************************
//
//! Configures Clock Precalar Strobe Mode.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function enables and configures the CLB Clock Precalar Strobe Mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_configureStrobeMode(uint32_t base, uint16_t strb)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREG(base + CLB_LOGICCTL + CLB_O_PRESCALE_CTRL) =
        (HWREG(base + CLB_LOGICCTL + CLB_O_PRESCALE_CTRL) &
         ~((uint32_t)CLB_PRESCALE_CTRL_TAP_M)) |
        ((uint32_t)strb << CLB_PRESCALE_CTRL_TAP_S);
    HWREG(base + CLB_LOGICCTL + CLB_O_PRESCALE_CTRL) |= CLB_PRESCALE_CTRL_STRB;
    EDIS;
}

//*****************************************************************************
//
//! Configures the general purpose SW release control value.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param inID is the specified CLB SW Release input signal.
//! \param val is the value of the SW RLS control.
//!
//! This function configures the general purpose SW release control value.
//! The \e inID parameter can have one enumeration value from CLB_SWReleaseCtrl.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_writeSWReleaseControl(uint32_t base,
                                            CLB_SWReleaseCtrl inID, bool val)
{
    ASSERT(CLB_isBaseValid(base));


    HWREG(base + CLB_LOGICCTL + CLB_O_GP_REG) =
            (HWREG(base + CLB_LOGICCTL + CLB_O_GP_REG) &
                ~(0x1000000U << inID)) |
                    (((uint32_t)val) << (24U + inID));
}


//*****************************************************************************
//
//! Configures the general purpose SW gate control value.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param inID is the specified CLB SW Release input signal.
//! \param val is the value of the SW GATE control.
//!
//! This function configures the general purpose SW release control value.
//! The \e inID parameter can have one enumeration value from CLB_SWGateCtrl.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_writeSWGateControl(uint32_t base,
                                            CLB_SWGateCtrl inID, bool val)
{
    ASSERT(CLB_isBaseValid(base));

    HWREG(base + CLB_LOGICCTL + CLB_O_GP_REG) =
            (HWREG(base + CLB_LOGICCTL + CLB_O_GP_REG) &
                ~(0x10000U << inID)) |
                    (((uint32_t)val) << (16U + inID));

}


//*****************************************************************************
//
//! Configures Counter TAP Selects.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param miscCtrl is the value to represent counter tap selects.
//!        Generated by tool as \b TILEx_CFG_TAP_SELL.
//!
//! This function configures the counter tap selects.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_configCounterTapSelects(uint32_t base, uint32_t tapSel)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREG(base + CLB_O_COUNT_MATCH_TAP_SEL) = tapSel;
    EDIS;
}

//*****************************************************************************
//
//! Configures AOC (Asynchronous Output Conditioning) functions.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param aocID is the specified CLB tile AOC signal.
//! \param aocCfg is the value for the AOC signal function and input
//!        signal selections. Generated by tool as \b TILEx_OUTPUT_COND_CTR_n
//!        where n is the output number.
//!
//! This function configures the input signals and equations of the aoc LUT
//! corresponding to the /e aocID parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_configAOC(uint32_t base, CLB_AOCs aocID,
                                     uint32_t aocCfg)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREG(base + CLB_O_OUTPUT_COND_CTRL_0 + (sizeof(uint32_t) * aocID)) = aocCfg;
    EDIS;
}

//*****************************************************************************
//
//! Enable CLB lock.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function enables the lock bit of the lock register. The lock can only
//! be set once and can only be cleared by a device reset.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_enableLock(uint32_t base)
{
    ASSERT(CLB_isBaseValid(base));

    //
    // Setting the lock bit requires key 0x5A5A to be written at the same time
    //
    EALLOW;
    HWREG(base + CLB_LOGICCTL + CLB_O_LOCK) =
        (uint32_t)CLB_LOCK_LOCK | ((uint32_t)CLB_LOCK_KEY << CLB_LOCK_KEY_S);
    EDIS;
}

//*****************************************************************************
//
//! Write value to address.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param address is the address of CLB internal memory.
//! \param value is the value to write to specified address.
//!
//! This function writes the specified value to CLB internal memory.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_writeInterface(uint32_t base, uint32_t address,
                                      uint32_t value)
{
    ASSERT(CLB_isBaseValid(base));
    ASSERT(CLB_isAddressValid(address));

    EALLOW;
    HWREG(base + CLB_LOGICCTL + CLB_O_LOAD_ADDR) = address;
    HWREG(base + CLB_LOGICCTL + CLB_O_LOAD_DATA) =  value;
    HWREG(base + CLB_LOGICCTL + CLB_O_LOAD_EN) |= CLB_LOAD_EN_LOAD_EN;
    EDIS;
}

//*****************************************************************************
//
//! Select input filter type.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param inID is the specified CLB tile input signal.
//! \param filterType is the selected type of filter applied to the input.
//!
//! This function configures the filter selection for the specified input.
//! The \e inID parameter can have one enumeration value from CLB_Inputs.
//! The \e filterType parameter can have one enumeration value from
//! CLB_FilterType.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_selectInputFilter(uint32_t base, CLB_Inputs inID,
                                         CLB_FilterType filterType)
{
    uint16_t shiftVal;

    ASSERT(CLB_isBaseValid(base));

    //
    // Each input has a 2-bit field in this register so need to calculate
    // shift amount accordingly.
    //
    shiftVal = (uint16_t)inID << 1;

    HWREGH(base + CLB_LOGICCTL + CLB_O_INPUT_FILTER) =
        (HWREGH(base + CLB_LOGICCTL + CLB_O_INPUT_FILTER) &
         ~(CLB_INPUT_FILTER_FIN0_M << shiftVal)) |
        ((uint16_t)filterType << shiftVal);
}

//*****************************************************************************
//
//! Enables synchronization of an input signal.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param inID is the specified CLB tile input signal.
//!
//! This function enables synchronization on the specified input signal.
//! The \e inID parameter can have one enumeration value from CLB_Inputs.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_enableSynchronization(uint32_t base, CLB_Inputs inID)
{
    ASSERT(CLB_isBaseValid(base));

    HWREGH(base + CLB_LOGICCTL + CLB_O_INPUT_FILTER + 1U) |=
                                                    (1U << (uint16_t)inID);
}

//*****************************************************************************
//
//! Disables synchronization of an input signal.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param inID is the specified CLB tile input signal.
//!
//! This function disables synchronization on the specified input signal.
//! The \e inID parameter can have one enumeration value from CLB_Inputs.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_disableSynchronization(uint32_t base, CLB_Inputs inID)
{
    ASSERT(CLB_isBaseValid(base));

    HWREGH(base + CLB_LOGICCTL + CLB_O_INPUT_FILTER + 1U) &=
                                                    ~(1U << (uint16_t)inID);
}

//*****************************************************************************
//
//! Configures the general purpose input mux.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param inID is the specified CLB tile input signal.
//! \param gpMuxCfg is the mux selection for the general purpose input mux.
//!
//! This function configures the general purpose input mux. The \e gpMuxCfg
//! parameter can select either the use of an external input signal
//! (\b CLB_GP_IN_MUX_EXTERNAL) or the use of the corresponding CLB_GP_REG bit
//! as an input (\b CLB_GP_IN_MUX_GP_REG).
//! The \e inID parameter can have one enumeration value from CLB_Inputs.
//!
//! \sa CLB_setGPREG() to write to the CLB_GP_REG.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_configGPInputMux(uint32_t base, CLB_Inputs inID,
                                        CLB_GPInputMux gpMuxCfg)
{
    ASSERT(CLB_isBaseValid(base));

    HWREGH(base + CLB_LOGICCTL + CLB_O_IN_MUX_SEL_0) =
        (HWREGH(base + CLB_LOGICCTL + CLB_O_IN_MUX_SEL_0) &
         ~(CLB_IN_MUX_SEL_0_SEL_GP_IN_0 << inID)) | (gpMuxCfg << inID);
}

//*****************************************************************************
//
//! Sets the CLB_GP_REG register value.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param gpRegVal is the value to be written to CLB_GP_REG.
//!
//! This function writes to the CLB_GP_REG register. When the general purpose
//! input mux is configured to use CLB_GP_REG, each bit in \e gpRegVal
//! corresponds to an input signal (bit 0 to Input 0, bit 1 to Input 1, and
//! so on).
//!
//! \sa CLB_configGPInputMux() to select the CLB_GP_REG as the source for
//! an input signal.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_setGPREG(uint32_t base, uint32_t gpRegVal)
{
    ASSERT(CLB_isBaseValid(base));

    HWREG(base + CLB_LOGICCTL + CLB_O_GP_REG) = gpRegVal;
}

//*****************************************************************************
//
//! Gets the CLB_GP_REG register value.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function writes to the CLB_GP_REG register. When the general purpose
//! input mux is configured to use CLB_GP_REG, each bit in \e gpRegVal
//! corresponds to an input signal (bit 0 to Input 0, bit 1 to Input 1, and
//! so on).
//!
//! \sa CLB_configGPInputMux() to select the CLB_GP_REG as the source for
//! an input signal.
//!
//! \return CLB_GP_REG value.
//
//*****************************************************************************
static inline uint32_t CLB_getGPREG(uint32_t base)
{
    ASSERT(CLB_isBaseValid(base));

    return(HWREG(base + CLB_LOGICCTL + CLB_O_GP_REG));
}

//*****************************************************************************
//
//! Configures the local input mux.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param inID is the specified CLB tile input signal.
//! \param localMuxCfg is the mux selection for the local input mux.
//!
//! This function configures the local input mux for the specified input
//! signal.
//!
//! The \e inID parameter can have one enumeration value from CLB_Inputs.
//! The \e localMuxCfg parameter can have one enumeration value from
//! CLB_LocalInputMux.
//!
//! \note The local input mux options' peripheral sources depend on which
//! instance of the CLB (\e base) you are using. For example, for CLB1 the
//! EPWM signal selections come from EPWM1 but for CLB2 they come from EPWM2.
//! See your technical reference manual for details.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_configLocalInputMux(uint32_t base, CLB_Inputs inID,
                                            CLB_LocalInputMux localMuxCfg)
{
    uint16_t shiftVal;
    uint32_t miscShiftVal, inputMuxSel;

    ASSERT(CLB_isBaseValid(base));

    //
    // Each local input has a 5-bit field in this register so need to calculate
    // shift amount accordingly.
    //
    shiftVal = (uint16_t)inID * CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_1_S;

    //
    // If the local inputs are selected then only 5-bits are used in this
    // register but if miscellaneous inputs are selected then another 1-bit is
    // used along with the above 5. So we need to calculate this accordingly.
    //

    //
    // Each miscellaneous input has a 1-bit field in this register so
    // need to calculate shift amount accordingly.
    //
    miscShiftVal = (uint32_t)inID + CLB_LCL_MUX_SEL_MISC_INPUT_SEL_S;

    //
    // Check if input < input4
    //
    if(inID < CLB_IN4)
    {
        //
        // Each miscellaneous input has an extra 1-bit to be set
        // apart from the local input bits
        //
        inputMuxSel =
       ((((uint32_t)localMuxCfg & CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_0_M) <<
        shiftVal) |
       ((((uint32_t)localMuxCfg & CLB_LCL_MUX_SEL_MISC_INPUT_SEL_M) >>
        CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_1_S) << miscShiftVal));

        //
        //Update the local / misc input mux selections
        //
        HWREG(base + CLB_LOGICCTL + CLB_O_LCL_MUX_SEL_1) =
       (HWREG(base + CLB_LOGICCTL + CLB_O_LCL_MUX_SEL_1) &
        ~(((uint32_t)CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_0_M << shiftVal) |
         (CLB_LCL_MUX_SEL_MISC_INPUT_SEL_BITM << miscShiftVal))) |
        inputMuxSel;
    }
    else
    {
        //
        //Adjust the previously calculated shifts for the inputs >3
        //
        shiftVal -= 4U * CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_5_S;
        miscShiftVal -= 4U;

        //
        // Each miscellaneous input has an extra 1-bit to be set
        // apart from the local input bits
        //
        inputMuxSel =
        (((uint32_t)(localMuxCfg & CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_4_M) <<
        shiftVal) |
        ((((uint32_t)localMuxCfg & CLB_LCL_MUX_SEL_MISC_INPUT_SEL_M) >>
        CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_5_S) << miscShiftVal));

        HWREG(base + CLB_LOGICCTL + CLB_O_LCL_MUX_SEL_2) =
       (HWREG(base + CLB_LOGICCTL + CLB_O_LCL_MUX_SEL_2) &
        ~(((uint32_t)CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_4_M << shiftVal) |
         (CLB_LCL_MUX_SEL_MISC_INPUT_SEL_BITM << miscShiftVal))) |
        inputMuxSel;

    }
}

//*****************************************************************************
//
//! Configures the global input mux.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param inID is the specified CLB tile input signal.
//! \param globalMuxCfg is the mux selection for the global input mux.
//!
//! This function configures the global input mux for the specified input
//! signal.
//! The \e inID parameter can have one enumeration value from CLB_Inputs.
//! The \e globalMuxCfg parameter can have one enumeration value from
//! CLB_GlobalInputMux.
//!
//! \note The global input mux options' peripheral sources depend on which
//! instance of the CLB (\e base) you are using. For example, for CLB1 the
//! EPWM signal selections come from EPWM1 but for CLB2 they come from EPWM2.
//! See your technical reference manual for details.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_configGlobalInputMux(uint32_t base, CLB_Inputs inID,
                                            CLB_GlobalInputMux globalMuxCfg)
{
    uint16_t shiftVal;

    ASSERT(CLB_isBaseValid(base));

    //
    // Each input has a 5-bit field in this register so need to calculate
    // shift amount accordingly.
    //
    shiftVal = (uint16_t)inID * CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_1_S;

    if(inID < CLB_IN4)
    {
        HWREG(base + CLB_LOGICCTL + CLB_O_GLBL_MUX_SEL_1) =
            (HWREG(base + CLB_LOGICCTL + CLB_O_GLBL_MUX_SEL_1) &
             ~((uint32_t)CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_0_M << shiftVal)) |
            ((uint32_t)globalMuxCfg << shiftVal);
    }
    else
    {
        shiftVal -= 4U * CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_1_S;
        HWREG(base + CLB_LOGICCTL + CLB_O_GLBL_MUX_SEL_2) =
            (HWREG(base + CLB_LOGICCTL + CLB_O_GLBL_MUX_SEL_2) &
             ~((uint32_t)CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_0_M << shiftVal)) |
            ((uint32_t)globalMuxCfg << shiftVal);
    }
}

//*****************************************************************************
//
//! Controls the output enable.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param outputMask is a mask of the outputs to be enabled.
//! \param enable is a switch to decide if the CLB outputs need to be enabled
//!  or not.
//!
//! This function is used to enable and disable CLB outputs by writing a mask
//! to CLB_OUT_EN. Each bit corresponds to a CLB output. When a bit is 1, the
//! corresponding output is enabled; when a bit is 0, the output is disabled.
//!
//! The \e outputMask parameter takes a logical OR of any of the CLB_OUTPUT_0x
//! values that correspond to the CLB OUTPUT ENABLE for the respective outputs.
//! The \e enable parameter can have one of the values from:
//! false: Disable the respective CLB outputs
//! true: Enable the respective CLB outputs
//!
//! \note Note that the 8 CLB outputs are replicated to create more output
//!  paths. See your technical reference manual for more details.
//!  If no further modifications are expected, then it is advised to set the
//!  block writes bit of the MISC_ACCESS_CTRL Register. This will prevent
//!  accidental writes.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_setOutputMask(uint32_t base, uint32_t outputMask ,
                                             bool enable)
{
    ASSERT(CLB_isBaseValid(base));

    if(enable == true)
     {
        HWREG(base + CLB_LOGICCTL + CLB_O_OUT_EN) |= outputMask;
     }
    else
     {
        HWREG(base + CLB_LOGICCTL + CLB_O_OUT_EN) &= ~outputMask;
     }
}

//*****************************************************************************
//
//! Reads the interrupt tag register.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! \return Returns the value in the interrupt tag register which is a 6-bit
//!         constant set by the HLC.
//
//*****************************************************************************
static inline uint16_t CLB_getInterruptTag(uint32_t base)
{
    ASSERT(CLB_isBaseValid(base));

    return(HWREGH(base + CLB_LOGICCTL + CLB_O_INTR_TAG_REG));
}

//*****************************************************************************
//
//! Clears the interrupt tag register.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function clears the interrupt tag register, setting it to 0.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_clearInterruptTag(uint32_t base)
{
    ASSERT(CLB_isBaseValid(base));

    HWREGH(base + CLB_LOGICCTL + CLB_O_INTR_TAG_REG) = 0U;
}

//*****************************************************************************
//
//! Selects LUT4 inputs.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param lut4In0 is the value for LUT4 input signal 0. Generated by tool as
//!        \b TILEx_CFG_LUT4_IN0.
//! \param lut4In1 is the value for LUT4 input signal 1. Generated by tool as
//!        \b TILEx_CFG_LUT4_IN1.
//! \param lut4In2 is the value for LUT4 input signal 2. Generated by tool as
//!        \b TILEx_CFG_LUT4_IN2.
//! \param lut4In3 is the value for LUT4 input signal 3. Generated by tool as
//!        \b TILEx_CFG_LUT4_IN3.
//!
//! This function configures the LUT4 block's input signals.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_selectLUT4Inputs(uint32_t base, uint32_t lut4In0,
                                        uint32_t lut4In1, uint32_t lut4In2,
                                        uint32_t lut4In3)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREG(base + CLB_O_LUT4_IN0) = lut4In0;
    HWREG(base + CLB_O_LUT4_IN1) = lut4In1;
    HWREG(base + CLB_O_LUT4_IN2) = lut4In2;
    HWREG(base + CLB_O_LUT4_IN3) = lut4In3;
    EDIS;
}

//*****************************************************************************
//
//! Configures LUT4 functions.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param lut4Fn10 is the equation value for LUT4 blocks 0 and 1. Generated by
//!        tool as \b TILEx_CFG_LUT4_FN10.
//! \param lut4Fn2 is the equation value for LUT4 block2. Generated by tool as
//!        \b TILEx_CFG_LUT4_FN2.
//!
//! This function configures the LUT4 block's equations.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_configLUT4Function(uint32_t base, uint32_t lut4Fn10,
                                          uint32_t lut4Fn2)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREG(base + CLB_O_LUT4_FN1_0) = lut4Fn10;
    HWREG(base + CLB_O_LUT4_FN2) = lut4Fn2;
    EDIS;
}

//*****************************************************************************
//
//! Selects FSM inputs.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param external0 is the value for FSM external 0 input. Generated by tool
//!        as \b TILEx_CFG_FSM_EXT_IN0.
//! \param external1 is the value for FSM external 1 input. Generated by tool
//!        as \b TILEx_CFG_FSM_EXT_IN1.
//! \param extra0 is the value for FSM extra 0 input. Generated by tool
//!        as \b TILEx_CFG_FSM_EXTRA_IN0.
//! \param extra1 is the value for FSM extra 1 input. Generated by tool
//!        as \b TILEx_CFG_FSM_EXTRA_IN1.
//!
//! This function configures the FSM block's external inputs and extra external
//! inputs.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_selectFSMInputs(uint32_t base, uint32_t external0,
                                       uint32_t external1, uint32_t extra0,
                                       uint32_t extra1)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREG(base + CLB_O_FSM_EXTERNAL_IN0) = external0;
    HWREG(base + CLB_O_FSM_EXTERNAL_IN1) = external1;
    HWREG(base + CLB_O_FSM_EXTRA_IN0) = extra0;
    HWREG(base + CLB_O_FSM_EXTRA_IN1) = extra1;
    EDIS;
}

//*****************************************************************************
//
//! Configures FSM LUT function.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param fsmLutFn10 is the value for FSM 0 & FSM 1 LUT function. Generated
//!        by tool as \b TILEx_CFG_FSM_LUT_FN10.
//! \param fsmLutFn2 is the value for FSM 2 LUT function. Generated by tool as
//!        \b TILEx_CFG_FSM_LUT_FN2.
//!
//! This function configures the FSM block's LUT equations.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_configFSMLUTFunction(uint32_t base, uint32_t fsmLutFn10,
                                            uint32_t fsmLutFn2)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREG(base + CLB_O_FSM_LUT_FN1_0) = fsmLutFn10;
    HWREG(base + CLB_O_FSM_LUT_FN2) = fsmLutFn2;
    EDIS;
}

//*****************************************************************************
//
//! Configures FSM next state.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param nextState0 is the value for FSM 0's next state. Generated by tool as
//!        \b TILEx_CFG_FSM_NEXT_STATE_0.
//! \param nextState1 is the value for FSM 1's next state. Generated by tool as
//!        \b TILEx_CFG_FSM_NEXT_STATE_1.
//! \param nextState2 is the value for FSM 2's next state. Generated by tool as
//!        \b TILEx_CFG_FSM_NEXT_STATE_2.
//!
//! This function configures the FSM's next state equation.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_configFSMNextState(uint32_t base, uint32_t nextState0,
                                          uint32_t nextState1,
                                          uint32_t nextState2)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREG(base + CLB_O_FSM_NEXT_STATE_0) = nextState0;
    HWREG(base + CLB_O_FSM_NEXT_STATE_1) = nextState1;
    HWREG(base + CLB_O_FSM_NEXT_STATE_2) = nextState2;
    EDIS;
}

//*****************************************************************************
//
//! Selects Counter inputs.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param reset is the value for counter's reset inputs. Generated by tool as
//!        \b TILEx_CFG_COUNTER_RESET.
//! \param event is the value for counter's event inputs. Generated by tool as
//!        \b TILEx_CFG_COUNTER_EVENT.
//! \param mode0 is the value for counter's mode 0 inputs. Generated by tool as
//!        \b TILEx_CFG_COUNTER_MODE_0.
//! \param mode1 is the value for counter's mode 1 inputs. Generated by tool as
//!        \b TILEx_CFG_COUNTER_MODE_1.
//!
//! This function selects the input signals to the counter block.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_selectCounterInputs(uint32_t base, uint32_t reset,
                                           uint32_t event, uint32_t mode0,
                                           uint32_t mode1)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREG(base + CLB_O_COUNT_RESET) = reset;
    HWREG(base + CLB_O_COUNT_EVENT) = event;
    HWREG(base + CLB_O_COUNT_MODE_0) = mode0;
    HWREG(base + CLB_O_COUNT_MODE_1) = mode1;
    EDIS;
}

//*****************************************************************************
//
//! Configures Counter and FSM modes.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param miscCtrl is the value to represent counter and FSM modes.
//!        Generated by tool as \b TILEx_CFG_MISC_CONTROL.
//!
//! This function configures the counter mode, particularly add/shift, load
//! modes. The function also configures whether the FSM should use state inputs
//! or an extra external input.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_configMiscCtrlModes(uint32_t base, uint32_t miscCtrl)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREG(base + CLB_O_MISC_CONTROL) = miscCtrl;
    EDIS;
}

//*****************************************************************************
//
//! Configures Output LUT functions.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param outID is the specified CLB tile output signal.
//! \param outputCfg is the value for the output LUT signal function and input
//!        signal selections. Generated by tool as \b TILEx_CFG_OUTLUT_n where
//!        n is the output number.
//!
//! This function configures the input signals and equations of the output LUT
//! corresponding to the /e outID parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_configOutputLUT(uint32_t base, CLB_Outputs outID,
                                       uint32_t outputCfg)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREG(base + CLB_O_OUTPUT_LUT_0 + (sizeof(uint32_t) * outID)) = outputCfg;
    EDIS;
}

//*****************************************************************************
//
//! Configures HLC event selection.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param eventSel is the value for HLC event selection. Generated by tool as
//!        \b TILEx_HLC_EVENT_SEL.
//!
//! This function configures the event selection for the High Level Controller.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_configHLCEventSelect(uint32_t base, uint32_t eventSel)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREG(base + CLB_O_HLC_EVENT_SEL) = eventSel;
    EDIS;
}

//*****************************************************************************
//
//! Program HLC instruction.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param instructionNum is the index into the HLC instruction memory. For
//!        example, a value of 0 corresponds to instruction 0 of event 0,
//!        a value of 1 corresponds to instruction 1 of event 0, and so on up
//!        to a value of 31 which corresponds to instruction 7 of event 3.
//! \param instruction is the instruction to be programmed. Generated by tool
//!        as \b TILEx_HLCINSTR_n where n is the instruction number.
//!
//! This function configures the CLB internal memory corresponding to the
//! specified HLC instruction number with the given instruction.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_programHLCInstruction(uint32_t base,
                                             uint32_t instructionNum,
                                             uint32_t instruction)
{
    ASSERT(CLB_isBaseValid(base));
    ASSERT(instructionNum < 32);

    CLB_writeInterface(base, CLB_ADDR_HLC_BASE + instructionNum, instruction);
}

//*****************************************************************************
//
//! Set HLC registers.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param r0Init is the value to write to HLC register R0. Generated by tool
//!        as \b TILEx_HLC_R0_INIT.
//! \param r1Init is the value to write to HLC register R1. Generated by tool
//!        as \b TILEx_HLC_R1_INIT.
//! \param r2Init is the value to write to HLC register R2. Generated by tool
//!        as \b TILEx_HLC_R2_INIT.
//! \param r3Init is the value to write to HLC register R3. Generated by tool
//!        as \b TILEx_HLC_R3_INIT.
//!
//! This function configures the CLB internal memory corresponding to the HLC
//! registers R0-R3 with the specified values.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_setHLCRegisters(uint32_t base, uint32_t r0Init,
                                       uint32_t r1Init, uint32_t r2Init,
                                       uint32_t r3Init)
{
    ASSERT(CLB_isBaseValid(base));

    CLB_writeInterface(base, CLB_ADDR_HLC_R0, r0Init);
    CLB_writeInterface(base, CLB_ADDR_HLC_R1, r1Init);
    CLB_writeInterface(base, CLB_ADDR_HLC_R2, r2Init);
    CLB_writeInterface(base, CLB_ADDR_HLC_R3, r3Init);
}

//*****************************************************************************
//
//! Get HLC or counter register values.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param registerID is the internal register from which to read. Can be
//!        either an HLC register (\b CLB_REG_HLC_Rn) or a counter value
//!        (\b CLB_REG_CTR_Cn).
//!
//! \return Returns the value in the specified HLC register or counter.
//
//*****************************************************************************
static inline uint32_t CLB_getRegister(uint32_t base, CLB_Register registerID)
{
    ASSERT(CLB_isBaseValid(base));

    return(HWREG(base + CLB_LOGICCTL + (uint32_t)registerID));
}

//*****************************************************************************
//
//! Get output status.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! \return Returns the output status of various components within the CLB tile
//!         such as a counter match or LUT output. Use the \b CLB_DBG_OUT_*
//!         masks from <tt>hw_clb.h</tt> to decode the bits.
//
//*****************************************************************************
static inline uint32_t CLB_getOutputStatus(uint32_t base)
{
    ASSERT(CLB_isBaseValid(base));

    return(HWREG(base + CLB_LOGICCTL + CLB_O_DBG_OUT));
}

//*****************************************************************************
//
//! Enable CLB Pipeline Mode.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function enables the CLB Pipeline Mode
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_enablePipelineMode(uint32_t base)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREGH(base + CLB_LOGICCTL + CLB_O_LOAD_EN) |= CLB_LOAD_EN_PIPELINE_EN;
    EDIS;
}

//*****************************************************************************
//
//! Disable CLB Pipeline Mode.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function disables the CLB Pipeline Mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_disablePipelineMode(uint32_t base)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREGH(base + CLB_LOGICCTL + CLB_O_LOAD_EN) &= ~CLB_LOAD_EN_PIPELINE_EN;
    EDIS;
}

//*****************************************************************************
//
//! Disable CLB Output Mask Updates.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function disables the CLB Output Mask updates
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_disableOutputMaskUpdates(uint32_t base)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREGH(base + CLB_O_MISC_ACCESS_CTRL) |= CLB_MISC_ACCESS_CTRL_BLKEN;
    EDIS;
}

//*****************************************************************************
//
//! Enable CLB Output Mask Updates.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function enables the CLB Output Mask updates
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_enableOutputMaskUpdates(uint32_t base)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREGH(base + CLB_O_MISC_ACCESS_CTRL) &= ~CLB_MISC_ACCESS_CTRL_BLKEN;
    EDIS;
}

//*****************************************************************************
//
//! Enable Input Pipeline Mode.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function enables the CLB Input Pipeline mode
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_enableInputPipelineMode(uint32_t base, CLB_Inputs inID)
{
    ASSERT(CLB_isBaseValid(base));

    HWREG(base + CLB_LOGICCTL + CLB_O_INPUT_FILTER) |=
            (CLB_INPUT_FILTER_PIPE0 << (uint32_t)inID);
}

//*****************************************************************************
//
//! Disable Input Pipeline Mode.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function disables the CLB Input Pipeline mode
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_disableInputPipelineMode(uint32_t base, CLB_Inputs inID)
{
    ASSERT(CLB_isBaseValid(base));

    HWREG(base + CLB_LOGICCTL + CLB_O_INPUT_FILTER) &=
            ~(CLB_INPUT_FILTER_PIPE0 << (uint32_t)inID);
}

//*****************************************************************************
//
//! Disable SPI RX Buffer Access.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function disables the CLB SPI RX Buffer access
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_disableSPIBufferAccess(uint32_t base)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREGH(base + CLB_O_MISC_ACCESS_CTRL) &= ~CLB_MISC_ACCESS_CTRL_SPIEN;
    EDIS;
}

//*****************************************************************************
//
//! Enable SPI RX Buffer Access.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function enables the CLB SPI RX Buffer access
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_enableSPIBufferAccess(uint32_t base)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREGH(base + CLB_O_MISC_ACCESS_CTRL) |= CLB_MISC_ACCESS_CTRL_SPIEN;
    EDIS;
}

//*****************************************************************************
//
//! Configures SPI RX Buffer Load Signal event selection.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param eventSel is the value for HLC event selection. Generated by tool as
//!        \b TILEx_SPI_BUF_EVENT_SEL.
//!
//! This function configures the event selection for the SPI RX Buffer.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_configSPIBufferLoadSignal(uint32_t base, uint16_t eventSel)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREGH(base + CLB_O_SPI_DATA_CTRL_HI) =
        (HWREGH(base + CLB_O_SPI_DATA_CTRL_HI) &
         ~((uint16_t)CLB_SPI_DATA_CTRL_HI_STRB_M << CLB_SPI_DATA_CTRL_HI_STRB_S)) |
        ((uint16_t)eventSel << CLB_SPI_DATA_CTRL_HI_STRB_S);
    EDIS;
}

//*****************************************************************************
//
//! Configures SPI Export HLC R0 Shift value.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param shiftVal is the value for SPI export HLC R0 bit range selection.
//!
//! This function configures the SPI Export HLC R0 Shift value.
//!
//! \return None.
//
//*****************************************************************************
static inline void CLB_configSPIBufferShift(uint32_t base, uint16_t shiftVal)
{
    ASSERT(CLB_isBaseValid(base));

    EALLOW;
    HWREGH(base + CLB_O_SPI_DATA_CTRL_HI) =
        (HWREGH(base + CLB_O_SPI_DATA_CTRL_HI) &
         ~((uint16_t)CLB_SPI_DATA_CTRL_HI_SHIFT_M << CLB_SPI_DATA_CTRL_HI_SHIFT_S)) |
        ((uint16_t)shiftVal << CLB_SPI_DATA_CTRL_HI_SHIFT_S);
    EDIS;
}

//*****************************************************************************
//
//! Configures Counter load and match.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param counterID is the specified counter unit.
//! \param load is the value for counter's load mode. Generated by tool as
//!        \b TILEx_COUNTER_n_LOAD_VAL where n is the counter number.
//! \param match1 is the value for counter's match 1. Generated by tool as
//!        \b TILEx_COUNTER_n_MATCH1_VAL where n is the counter number.
//! \param match2 is the value for counter's match 2. Generated by tool as
//!        \b TILEx_COUNTER_n_MATCH2_VAL where n is the counter number.
//!
//! This function configures the CLB internal memory corresponding to the
//! counter block's load and match values.
//!
//! \return None.
//
//*****************************************************************************
extern void CLB_configCounterLoadMatch(uint32_t base, CLB_Counters counterID,
                                       uint32_t load, uint32_t match1,
                                       uint32_t match2);

//*****************************************************************************
//
//! Clear FIFO registers.
//!
//! \param base is the base address of a CLB tile's logic config register.
//!
//! This function clears the PUSH/PULL FIFOs as well as its pointers.
//!
//! \return None.
//
//*****************************************************************************
extern void CLB_clearFIFOs(uint32_t base);

//*****************************************************************************
//
//! Configure the FIFO registers.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param pullData[] is a pointer to an array of bytes which needs to be
//! written into the FIFO. The 0th FIFO data is in the 0th index.
//!
//! This function writes to the PULL FIFO. This also clears the FIFOs and
//! its pointer using the CLB_clearFIFOs() API prior to writing to
//! the FIFO.
//!
//! \return None.
//
//*****************************************************************************
extern void CLB_writeFIFOs(uint32_t base, const uint32_t pullData[]);

//*****************************************************************************
//
//! Read FIFO registers.
//!
//! \param base is the base address of a CLB tile's logic config register.
//! \param pushData[] is a pointer to an array of bytes which needs to be
//! read from the FIFO.
//!
//! This function reads from the PUSH FIFO. The 0th FIFO data would be in
//! the 0th index.
//!
//! \return None.
//
//*****************************************************************************
extern void CLB_readFIFOs(uint32_t base , uint32_t pushData[]);

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

#endif // CLB_H
