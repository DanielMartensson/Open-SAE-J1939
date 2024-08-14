//###########################################################################
//
// FILE:   adc.h
//
// TITLE:  C28x ADC driver.
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

#ifndef ADC_H
#define ADC_H

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
//! \addtogroup adc_api ADC
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_adc.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "cpu.h"
#include "debug.h"
//*****************************************************************************
//
// Useful defines used within the driver functions. Not intended for use by
// application code.
//
//*****************************************************************************
#define ADC_NUM_INTERRUPTS          4U

#define ADC_SOCxCTL_OFFSET_BASE     ADC_O_SOC0CTL
#define ADC_RESULTx_OFFSET_BASE     ADC_O_RESULT0
#define ADC_INTSELxNy_OFFSET_BASE   ADC_O_INTSEL1N2
#define ADC_PPBxRESULT_OFFSET_BASE  ADC_O_PPB1RESULT

#define ADC_PPBxCONFIG_STEP         (ADC_O_PPB2CONFIG - ADC_O_PPB1CONFIG)
#define ADC_PPBxTRIPHI_STEP         (ADC_O_PPB2TRIPHI - ADC_O_PPB1TRIPHI)
#define ADC_PPBxTRIPLO_STEP         (ADC_O_PPB2TRIPLO - ADC_O_PPB1TRIPLO)
#define ADC_PPBxSTAMP_STEP          (ADC_O_PPB2STAMP - ADC_O_PPB1STAMP)
#define ADC_PPBxOFFCAL_STEP         (ADC_O_PPB2OFFCAL - ADC_O_PPB1OFFCAL)
#define ADC_PPBxOFFREF_STEP         (ADC_O_PPB2OFFREF - ADC_O_PPB1OFFREF)

#define ADC_PPBTRIP_MASK            ((uint32_t)ADC_PPB1TRIPHI_LIMITHI_M      |\
                                     (uint32_t)ADC_PPB1TRIPHI_HSIGN)
//
// Slope of the temperature sensor
//
#define ADC_EXT_REF_TSSLOPE         (*(float32_t *)((uintptr_t)0x701C8))

//
// Offset of the temp sensor output
//
#define ADC_EXT_REF_TSOFFSET        (*(float32_t *)((uintptr_t)0x701CA))


#ifndef DOXYGEN_PDF_IGNORE
//*****************************************************************************
//
// Values that can be passed to ADC_enablePPBEvent(), ADC_disablePPBEvent(),
// ADC_enablePPBEventInterrupt(), ADC_disablePPBEventInterrupt(), and
// ADC_clearPPBEventStatus() as the intFlags and evtFlags parameters. They also
// make up the enumerated bit field returned by ADC_getPPBEventStatus().
//
//*****************************************************************************
#define ADC_EVT_TRIPHI              0x0001U //!< Trip High Event
#define ADC_EVT_TRIPLO              0x0002U //!< Trip Low Event
#define ADC_EVT_ZERO                0x0004U //!< Zero Crossing Event
#endif

//*****************************************************************************
//
// Values that can be passed to ADC_forceMultipleSOC() as socMask parameter.
// These values can be OR'd together to trigger multiple SOCs at a time.
//
//*****************************************************************************
#define ADC_FORCE_SOC0              0x0001U //!< SW trigger ADC SOC 0
#define ADC_FORCE_SOC1              0x0002U //!< SW trigger ADC SOC 1
#define ADC_FORCE_SOC2              0x0004U //!< SW trigger ADC SOC 2
#define ADC_FORCE_SOC3              0x0008U //!< SW trigger ADC SOC 3
#define ADC_FORCE_SOC4              0x0010U //!< SW trigger ADC SOC 4
#define ADC_FORCE_SOC5              0x0020U //!< SW trigger ADC SOC 5
#define ADC_FORCE_SOC6              0x0040U //!< SW trigger ADC SOC 6
#define ADC_FORCE_SOC7              0x0080U //!< SW trigger ADC SOC 7
#define ADC_FORCE_SOC8              0x0100U //!< SW trigger ADC SOC 8
#define ADC_FORCE_SOC9              0x0200U //!< SW trigger ADC SOC 9
#define ADC_FORCE_SOC10             0x0400U //!< SW trigger ADC SOC 10
#define ADC_FORCE_SOC11             0x0800U //!< SW trigger ADC SOC 11
#define ADC_FORCE_SOC12             0x1000U //!< SW trigger ADC SOC 12
#define ADC_FORCE_SOC13             0x2000U //!< SW trigger ADC SOC 13
#define ADC_FORCE_SOC14             0x4000U //!< SW trigger ADC SOC 14
#define ADC_FORCE_SOC15             0x8000U //!< SW trigger ADC SOC 15

//*****************************************************************************
//
//! Values that can be passed to ADC_setPrescaler() as the \e clkPrescale
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_CLK_DIV_1_0 = 0U,            //!< ADCCLK = (input clock) / 1.0
    ADC_CLK_DIV_2_0 = 2U,            //!< ADCCLK = (input clock) / 2.0
    ADC_CLK_DIV_2_5 = 3U,            //!< ADCCLK = (input clock) / 2.5
    ADC_CLK_DIV_3_0 = 4U,            //!< ADCCLK = (input clock) / 3.0
    ADC_CLK_DIV_3_5 = 5U,            //!< ADCCLK = (input clock) / 3.5
    ADC_CLK_DIV_4_0 = 6U,            //!< ADCCLK = (input clock) / 4.0
    ADC_CLK_DIV_4_5 = 7U,            //!< ADCCLK = (input clock) / 4.5
    ADC_CLK_DIV_5_0 = 8U,            //!< ADCCLK = (input clock) / 5.0
    ADC_CLK_DIV_5_5 = 9U,            //!< ADCCLK = (input clock) / 5.5
    ADC_CLK_DIV_6_0 = 10U,           //!< ADCCLK = (input clock) / 6.0
    ADC_CLK_DIV_6_5 = 11U,           //!< ADCCLK = (input clock) / 6.5
    ADC_CLK_DIV_7_0 = 12U,           //!< ADCCLK = (input clock) / 7.0
    ADC_CLK_DIV_7_5 = 13U,           //!< ADCCLK = (input clock) / 7.5
    ADC_CLK_DIV_8_0 = 14U,           //!< ADCCLK = (input clock) / 8.0
    ADC_CLK_DIV_8_5 = 15U            //!< ADCCLK = (input clock) / 8.5
} ADC_ClkPrescale;

//*****************************************************************************
//
//! Values that can be passed to ADC_setMode() as the \e resolution
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_RESOLUTION_12BIT = 0x00U,    //!< 12-bit conversion resolution
    ADC_RESOLUTION_16BIT = 0x40U     //!< 16-bit conversion resolution
} ADC_Resolution;

//*****************************************************************************
//
//! Values that can be passed to ADC_setMode() as the \e signalMode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_MODE_SINGLE_ENDED = 0x00U,   //!< Sample on single pin with VREFLO
    ADC_MODE_DIFFERENTIAL = 0x80U    //!< Sample on pair of pins
} ADC_SignalMode;

//*****************************************************************************
//
//! Values that can be passed to ADC_setupSOC() as the \e trigger
//! parameter to specify the event that will trigger a conversion to start.
//! It is also used with ADC_setBurstModeConfig() and
//! ADC_triggerRepeaterSelect().
//
//*****************************************************************************
typedef enum
{
    ADC_TRIGGER_SW_ONLY     = 0U,     //!< Software only
    ADC_TRIGGER_CPU1_TINT0  = 1U,     //!< CPU1 Timer 0, TINT0
    ADC_TRIGGER_CPU1_TINT1  = 2U,     //!< CPU1 Timer 1, TINT1
    ADC_TRIGGER_CPU1_TINT2  = 3U,     //!< CPU1 Timer 2, TINT2
    ADC_TRIGGER_GPIO        = 4U,     //!< GPIO, ADCEXTSOC
    ADC_TRIGGER_EPWM1_SOCA  = 5U,     //!< ePWM1, ADCSOCA
    ADC_TRIGGER_EPWM1_SOCB  = 6U,     //!< ePWM1, ADCSOCB
    ADC_TRIGGER_EPWM2_SOCA  = 7U,     //!< ePWM2, ADCSOCA
    ADC_TRIGGER_EPWM2_SOCB  = 8U,     //!< ePWM2, ADCSOCB
    ADC_TRIGGER_EPWM3_SOCA  = 9U,     //!< ePWM3, ADCSOCA
    ADC_TRIGGER_EPWM3_SOCB  = 10U,    //!< ePWM3, ADCSOCB
    ADC_TRIGGER_EPWM4_SOCA  = 11U,    //!< ePWM4, ADCSOCA
    ADC_TRIGGER_EPWM4_SOCB  = 12U,    //!< ePWM4, ADCSOCB
    ADC_TRIGGER_EPWM5_SOCA  = 13U,    //!< ePWM5, ADCSOCA
    ADC_TRIGGER_EPWM5_SOCB  = 14U,    //!< ePWM5, ADCSOCB
    ADC_TRIGGER_EPWM6_SOCA  = 15U,    //!< ePWM6, ADCSOCA
    ADC_TRIGGER_EPWM6_SOCB  = 16U,    //!< ePWM6, ADCSOCB
    ADC_TRIGGER_EPWM7_SOCA  = 17U,    //!< ePWM7, ADCSOCA
    ADC_TRIGGER_EPWM7_SOCB  = 18U,    //!< ePWM7, ADCSOCB
    ADC_TRIGGER_EPWM8_SOCA  = 19U,    //!< ePWM8, ADCSOCA
    ADC_TRIGGER_EPWM8_SOCB  = 20U,    //!< ePWM8, ADCSOCB
    ADC_TRIGGER_EPWM9_SOCA  = 21U,    //!< ePWM9, ADCSOCA
    ADC_TRIGGER_EPWM9_SOCB  = 22U,    //!< ePWM9, ADCSOCB
    ADC_TRIGGER_EPWM10_SOCA = 23U,    //!< ePWM10, ADCSOCA
    ADC_TRIGGER_EPWM10_SOCB = 24U,    //!< ePWM10, ADCSOCB
    ADC_TRIGGER_EPWM11_SOCA = 25U,    //!< ePWM11, ADCSOCA
    ADC_TRIGGER_EPWM11_SOCB = 26U,    //!< ePWM11, ADCSOCB
    ADC_TRIGGER_EPWM12_SOCA = 27U,    //!< ePWM12, ADCSOCA
    ADC_TRIGGER_EPWM12_SOCB = 28U,    //!< ePWM12, ADCSOCB
    ADC_TRIGGER_CPU2_TINT0  = 29U,    //!< CPU2 Timer 0, TINT0
    ADC_TRIGGER_CPU2_TINT1  = 30U,    //!< CPU2 Timer 1, TINT1
    ADC_TRIGGER_CPU2_TINT2  = 31U,    //!< CPU2 Timer 2, TINT2
    ADC_TRIGGER_EPWM13_SOCA = 32U,    //!< ePWM13, ADCSOCA
    ADC_TRIGGER_EPWM13_SOCB = 33U,    //!< ePWM13, ADCSOCB
    ADC_TRIGGER_EPWM14_SOCA = 34U,    //!< ePWM14, ADCSOCA
    ADC_TRIGGER_EPWM14_SOCB = 35U,    //!< ePWM14, ADCSOCB
    ADC_TRIGGER_EPWM15_SOCA = 36U,    //!< ePWM15, ADCSOCA
    ADC_TRIGGER_EPWM15_SOCB = 37U,    //!< ePWM15, ADCSOCB
    ADC_TRIGGER_EPWM16_SOCA = 38U,    //!< ePWM16, ADCSOCA
    ADC_TRIGGER_EPWM16_SOCB = 39U     //!< ePWM16, ADCSOCB
} ADC_Trigger;

//*****************************************************************************
//
//! Values that can be passed to ADC_setupSOC() as the \e channel
//! parameter. This is the input pin on which the signal to be converted is
//! located.
//
//*****************************************************************************
typedef enum
{
    ADC_CH_ADCIN0  = 0U,          //!< single-ended, ADCIN0
    ADC_CH_ADCIN1  = 1U,          //!< single-ended, ADCIN1
    ADC_CH_ADCIN2  = 2U,          //!< single-ended, ADCIN2
    ADC_CH_ADCIN3  = 3U,          //!< single-ended, ADCIN3
    ADC_CH_ADCIN4  = 4U,          //!< single-ended, ADCIN4
    ADC_CH_ADCIN5  = 5U,          //!< single-ended, ADCIN5
    ADC_CH_ADCIN6  = 6U,          //!< single-ended, ADCIN6
    ADC_CH_ADCIN7  = 7U,          //!< single-ended, ADCIN7
    ADC_CH_ADCIN8  = 8U,          //!< single-ended, ADCIN8
    ADC_CH_ADCIN9  = 9U,          //!< single-ended, ADCIN9
    ADC_CH_ADCIN10 = 10U,         //!< single-ended, ADCIN10
    ADC_CH_ADCIN11 = 11U,         //!< single-ended, ADCIN11
    ADC_CH_ADCIN12 = 12U,         //!< single-ended, ADCIN12
    ADC_CH_ADCIN13 = 13U,         //!< single-ended, ADCIN13
    ADC_CH_ADCIN14 = 14U,         //!< single-ended, ADCIN14
    ADC_CH_ADCIN15 = 15U,         //!< single-ended, ADCIN15
    ADC_CH_ADCIN0_ADCIN1   = 0U,  //!< differential, ADCIN0 and ADCIN1
    ADC_CH_ADCIN2_ADCIN3   = 2U,  //!< differential, ADCIN2 and ADCIN3
    ADC_CH_ADCIN4_ADCIN5   = 4U,  //!< differential, ADCIN4 and ADCIN5
    ADC_CH_ADCIN6_ADCIN7   = 6U,  //!< differential, ADCIN6 and ADCIN7
    ADC_CH_ADCIN8_ADCIN9   = 8U,  //!< differential, ADCIN8 and ADCIN9
    ADC_CH_ADCIN10_ADCIN11 = 10U, //!< differential, ADCIN10 and ADCIN11
    ADC_CH_ADCIN12_ADCIN13 = 12U, //!< differential, ADCIN12 and ADCIN13
    ADC_CH_ADCIN14_ADCIN15 = 14U  //!< differential, ADCIN14 and ADCIN15
} ADC_Channel;

//*****************************************************************************
//
//! Values that can be passed to ADC_setInterruptPulseMode() as the
//! \e pulseMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Occurs at the end of the acquisition window
    ADC_PULSE_END_OF_ACQ_WIN = 0x00U,
    //! Occurs at the end of the conversion
    ADC_PULSE_END_OF_CONV    = 0x04U
} ADC_PulseMode;

//*****************************************************************************
//
//! Values that can be passed to ADC_enableInterrupt(), ADC_disableInterrupt(),
//! and ADC_getInterruptStatus() as the \e adcIntNum parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_INT_NUMBER1 = 0U,        //!< ADCINT1 Interrupt
    ADC_INT_NUMBER2 = 1U,        //!< ADCINT2 Interrupt
    ADC_INT_NUMBER3 = 2U,        //!< ADCINT3 Interrupt
    ADC_INT_NUMBER4 = 3U         //!< ADCINT4 Interrupt
} ADC_IntNumber;

//*****************************************************************************
//
//! Values that can be passed in as the \e ppbNumber parameter for several
//! functions.
//
//*****************************************************************************
typedef enum
{
    ADC_PPB_NUMBER1 = 0U,        //!< Post-processing block 1
    ADC_PPB_NUMBER2 = 1U,        //!< Post-processing block 2
    ADC_PPB_NUMBER3 = 2U,        //!< Post-processing block 3
    ADC_PPB_NUMBER4 = 3U         //!< Post-processing block 4
} ADC_PPBNumber;

//*****************************************************************************
//
//! Values that can be passed in as the \e socNumber parameter for several
//! functions. This value identifies the start-of-conversion (SOC) that a
//! function is configuring or accessing. Note that in some cases (for example,
//! ADC_setInterruptSource()) \e socNumber is used to refer to the
//! corresponding end-of-conversion (EOC).
//
//*****************************************************************************
typedef enum
{
    ADC_SOC_NUMBER0  = 0U,        //!< SOC/EOC number 0
    ADC_SOC_NUMBER1  = 1U,        //!< SOC/EOC number 1
    ADC_SOC_NUMBER2  = 2U,        //!< SOC/EOC number 2
    ADC_SOC_NUMBER3  = 3U,        //!< SOC/EOC number 3
    ADC_SOC_NUMBER4  = 4U,        //!< SOC/EOC number 4
    ADC_SOC_NUMBER5  = 5U,        //!< SOC/EOC number 5
    ADC_SOC_NUMBER6  = 6U,        //!< SOC/EOC number 6
    ADC_SOC_NUMBER7  = 7U,        //!< SOC/EOC number 7
    ADC_SOC_NUMBER8  = 8U,        //!< SOC/EOC number 8
    ADC_SOC_NUMBER9  = 9U,        //!< SOC/EOC number 9
    ADC_SOC_NUMBER10 = 10U,       //!< SOC/EOC number 10
    ADC_SOC_NUMBER11 = 11U,       //!< SOC/EOC number 11
    ADC_SOC_NUMBER12 = 12U,       //!< SOC/EOC number 12
    ADC_SOC_NUMBER13 = 13U,       //!< SOC/EOC number 13
    ADC_SOC_NUMBER14 = 14U,       //!< SOC/EOC number 14
    ADC_SOC_NUMBER15 = 15U        //!< SOC/EOC number 15
} ADC_SOCNumber;

//*****************************************************************************
//
//! Values that can be passed in as the \e trigger parameter for the
//! ADC_setInterruptSOCTrigger() function.
//
//*****************************************************************************
typedef enum
{
    ADC_INT_SOC_TRIGGER_NONE    = 0U,   //!< No ADCINT will trigger the SOC
    ADC_INT_SOC_TRIGGER_ADCINT1 = 1U,   //!< ADCINT1 will trigger the SOC
    ADC_INT_SOC_TRIGGER_ADCINT2 = 2U    //!< ADCINT2 will trigger the SOC
} ADC_IntSOCTrigger;

//*****************************************************************************
//
//! Values that can be passed to ADC_setSOCPriority() as the \e priMode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_PRI_ALL_ROUND_ROBIN = 0U,    //!< Round robin mode is used for all
    ADC_PRI_SOC0_HIPRI      = 1U,    //!< SOC 0 hi pri, others in round robin
    ADC_PRI_THRU_SOC1_HIPRI = 2U,    //!< SOC 0-1 hi pri, others in round robin
    ADC_PRI_THRU_SOC2_HIPRI = 3U,    //!< SOC 0-2 hi pri, others in round robin
    ADC_PRI_THRU_SOC3_HIPRI = 4U,    //!< SOC 0-3 hi pri, others in round robin
    ADC_PRI_THRU_SOC4_HIPRI = 5U,    //!< SOC 0-4 hi pri, others in round robin
    ADC_PRI_THRU_SOC5_HIPRI = 6U,    //!< SOC 0-5 hi pri, others in round robin
    ADC_PRI_THRU_SOC6_HIPRI = 7U,    //!< SOC 0-6 hi pri, others in round robin
    ADC_PRI_THRU_SOC7_HIPRI = 8U,    //!< SOC 0-7 hi pri, others in round robin
    ADC_PRI_THRU_SOC8_HIPRI = 9U,    //!< SOC 0-8 hi pri, others in round robin
    ADC_PRI_THRU_SOC9_HIPRI = 10U,   //!< SOC 0-9 hi pri, others in round robin
    ADC_PRI_THRU_SOC10_HIPRI = 11U,  //!< SOC 0-10 hi pri, others in round robin
    ADC_PRI_THRU_SOC11_HIPRI = 12U,  //!< SOC 0-11 hi pri, others in round robin
    ADC_PRI_THRU_SOC12_HIPRI = 13U,  //!< SOC 0-12 hi pri, others in round robin
    ADC_PRI_THRU_SOC13_HIPRI = 14U,  //!< SOC 0-13 hi pri, others in round robin
    ADC_PRI_THRU_SOC14_HIPRI = 15U,  //!< SOC 0-14 hi pri, SOC15 in round robin
    ADC_PRI_ALL_HIPRI = 16U          //!< All priorities based on SOC number
} ADC_PriorityMode;

//*****************************************************************************
//
//! Values that can be passed to ADC_configOSDetectMode() as the \e modeVal
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ADC_OSDETECT_MODE_DISABLED            = 0x0U,//!< Open/Shorts detection cir-
                                                 //!< cuit(O/S DC) is disabled
    ADC_OSDETECT_MODE_VSSA                = 0x1U,//!< O/S DC is enabled at zero
                                                 //!< scale
    ADC_OSDETECT_MODE_VDDA                = 0x2U,//!< O/S DC is enabled at full
                                                 //!< scale
    ADC_OSDETECT_MODE_5BY12_VDDA          = 0x3U,//!< O/S DC is enabled at 5/12
                                                 //!< scale
    ADC_OSDETECT_MODE_7BY12_VDDA          = 0x4U,//!< O/S DC is enabled at 7/12
                                                 //!< scale
    ADC_OSDETECT_MODE_5K_PULLDOWN_TO_VSSA = 0x5U,//!< O/S DC is enabled at 5K
                                                 //!< pulldown to VSSA
    ADC_OSDETECT_MODE_5K_PULLUP_TO_VDDA   = 0x6U,//!< O/S DC is enabled at 5K
                                                 //!< pullup to VDDA
    ADC_OSDETECT_MODE_7K_PULLDOWN_TO_VSSA = 0x7U //!< O/S DC is enabled at 7K
                                                 //!< pulldown to VSSA
} ADC_OSDetectMode;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! \internal
//! Checks an ADC base address.
//!
//! \param base specifies the ADC module base address.
//!
//! This function determines if a ADC module base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
ADC_isBaseValid(uint32_t base)
{
    return(
           (base == ADCA_BASE) ||
           (base == ADCB_BASE) ||
           (base == ADCC_BASE) ||
           (base == ADCD_BASE)
          );
}
#endif

//*****************************************************************************
//
//! Configures the analog-to-digital converter module prescaler.
//!
//! \param base is the base address of the ADC module.
//! \param clkPrescale is the ADC clock prescaler.
//!
//! This function configures the ADC module's ADCCLK.
//!
//! The \e clkPrescale parameter specifies the value by which the input clock
//! is divided to make the ADCCLK.  The clkPrescale value can be specified with
//! any of the following enum values:
//! \b ADC_CLK_DIV_1_0, \b ADC_CLK_DIV_2_0, \b ADC_CLK_DIV_2_5, ...,
//! \b ADC_CLK_DIV_7_5, \b ADC_CLK_DIV_8_0, or \b ADC_CLK_DIV_8_5.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setPrescaler(uint32_t base, ADC_ClkPrescale clkPrescale)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Set the configuration of the ADC module prescaler.
    //
    EALLOW;
    HWREGH(base + ADC_O_CTL2) = (HWREGH(base + ADC_O_CTL2) &
                                 ~ADC_CTL2_PRESCALE_M) | (uint16_t)clkPrescale;
    EDIS;
}

//*****************************************************************************
//
//! Configures a start-of-conversion (SOC) in the ADC.
//!
//! \param base is the base address of the ADC module.
//! \param socNumber is the number of the start-of-conversion.
//! \param trigger the source that will cause the SOC.
//! \param channel is the number associated with the input signal.
//! \param sampleWindow is the acquisition window duration.
//!
//! This function configures the a start-of-conversion (SOC) in the ADC module.
//!
//! The \e socNumber number is a value \b ADC_SOC_NUMBERX where X is a number
//! from 0 to 15 specifying which SOC is to be configured on the ADC module
//! specified by \e base.
//!
//! The \e trigger specifies the event that causes the SOC such as software, a
//! timer interrupt, an ePWM event, or an ADC interrupt. It should be a value
//! in the format of \b ADC_TRIGGER_XXXX where XXXX is the event such as
//! \b ADC_TRIGGER_SW_ONLY, \b ADC_TRIGGER_CPU1_TINT0, \b ADC_TRIGGER_GPIO,
//! \b ADC_TRIGGER_EPWM1_SOCA, and so on.
//!
//! The \e channel parameter specifies the channel to be converted. In
//! single-ended mode this is a single pin given by \b ADC_CH_ADCINx where x is
//! the number identifying the pin between 0 and 15 inclusive. In differential
//! mode, two pins are used as inputs and are passed in the \e channel
//! parameter as \b ADC_CH_ADCIN0_ADCIN1, \b ADC_CH_ADCIN2_ADCIN3, ..., or
//! \b ADC_CH_ADCIN14_ADCIN15.
//!
//! The \e sampleWindow parameter is the acquisition window duration in SYSCLK
//! cycles. It should be a value between 1 and 512 cycles inclusive. The
//! selected duration must be at least as long as one ADCCLK cycle. Also, the
//! datasheet will specify a minimum window duration requirement in
//! nanoseconds.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setupSOC(uint32_t base, ADC_SOCNumber socNumber, ADC_Trigger trigger,
             ADC_Channel channel, uint32_t sampleWindow)
{
    uint32_t ctlRegAddr;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));
    ASSERT((sampleWindow >= 1U) && (sampleWindow <= 512U));

    //
    // Calculate address for the SOC control register.
    //
    ctlRegAddr = base + ADC_SOCxCTL_OFFSET_BASE + ((uint32_t)socNumber * 2U);

    //
    // Set the configuration of the specified SOC.
    //
    EALLOW;
    HWREG(ctlRegAddr) = ((uint32_t)channel << ADC_SOC0CTL_CHSEL_S) |
                        ((uint32_t)trigger << ADC_SOC0CTL_TRIGSEL_S) |
                        (sampleWindow - 1U);
    EDIS;
}

//*****************************************************************************
//
//! Configures the interrupt SOC trigger of an SOC.
//!
//! \param base is the base address of the ADC module.
//! \param socNumber is the number of the start-of-conversion.
//! \param trigger the interrupt source that will cause the SOC.
//!
//! This function configures the interrupt start-of-conversion trigger in
//! the ADC module.
//!
//! The \e socNumber number is a value \b ADC_SOC_NUMBERX where X is a number
//! from 0 to 15 specifying which SOC is to be configured on the ADC module
//! specified by \e base.
//!
//! The \e trigger specifies the interrupt that causes a start of conversion or
//! none. It should be one of the following values.
//!
//! - \b ADC_INT_SOC_TRIGGER_NONE
//! - \b ADC_INT_SOC_TRIGGER_ADCINT1
//! - \b ADC_INT_SOC_TRIGGER_ADCINT2
//!
//! This functionality is useful for creating continuous conversions.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setInterruptSOCTrigger(uint32_t base, ADC_SOCNumber socNumber,
                           ADC_IntSOCTrigger trigger)
{
    uint16_t shiftVal;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Each SOC has a 2-bit field in this register.
    //
    shiftVal = (uint16_t)socNumber << 1U;

    //
    // Set the configuration of the specified SOC. Not that we're treating
    // ADCINTSOCSEL1 and ADCINTSOCSEL2 as one 32-bit register here.
    //
    EALLOW;
    HWREG(base + ADC_O_INTSOCSEL1) = (HWREG(base + ADC_O_INTSOCSEL1) &
                                      ~((uint32_t)ADC_INTSOCSEL1_SOC0_M <<
                                        shiftVal)) |
                                     ((uint32_t)trigger << shiftVal);
    EDIS;
}

//*****************************************************************************
//
//! Sets the timing of the end-of-conversion pulse
//!
//! \param base is the base address of the ADC module.
//! \param pulseMode is the generation mode of the EOC pulse.
//!
//! This function configures the end-of-conversion (EOC) pulse generated by ADC.
//! This pulse will be generated either at the end of the acquisition window
//! plus a number of SYSCLK cycles configured by ADC_setInterruptCycleOffset()
//! (pass \b ADC_PULSE_END_OF_ACQ_WIN into \e pulseMode) or at the end of the
//! voltage conversion, one cycle prior to the ADC result latching into it's
//! result register (pass \b ADC_PULSE_END_OF_CONV into \e pulseMode).
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setInterruptPulseMode(uint32_t base, ADC_PulseMode pulseMode)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Set the position of the pulse.
    //
    EALLOW;
    HWREGH(base + ADC_O_CTL1) = (HWREGH(base + ADC_O_CTL1) &
                                 ~ADC_CTL1_INTPULSEPOS) | (uint16_t)pulseMode;
    EDIS;
}

//*****************************************************************************
//
//! Sets the timing of early interrupt generation.
//!
//! \param base is the base address of the ADC module.
//! \param cycleOffset is the cycles from an SOC falling edge to an early
//! interrupt pulse.
//!
//! This function configures cycle offset between the negative edge of a sample
//! pulse and an early interrupt pulse being generated. This number of cycles
//! is specified with the \e cycleOffset parameter.
//!
//! This function only applies when early interrupt generation is enabled. That
//! means the ADC_setInterruptPulseMode() function \e pulseMode parameter is
//! configured as \b ADC_PULSE_END_OF_ACQ_WIN.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setInterruptCycleOffset(uint32_t base, uint16_t cycleOffset)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Set the position of the pulse.
    //
    EALLOW;
    HWREGH(base + ADC_O_INTCYCLE) = cycleOffset;
    EDIS;
}


//*****************************************************************************
//
//! Powers up the analog-to-digital converter core.
//!
//! \param base is the base address of the ADC module.
//!
//! This function powers up the analog circuitry inside the analog core.
//!
//! \note Allow at least a 500us delay before sampling after calling this API.
//! If you enable multiple ADCs, you can delay after they all have begun
//! powering up.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enableConverter(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Set the bit that powers up the analog circuitry.
    //
    EALLOW;
    HWREGH(base + ADC_O_CTL1) |= ADC_CTL1_ADCPWDNZ;
    EDIS;
}

//*****************************************************************************
//
//! Powers down the analog-to-digital converter module.
//!
//! \param base is the base address of the ADC module.
//!
//! This function powers down the analog circuitry inside the analog core.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disableConverter(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Clear the bit that powers down the analog circuitry.
    //
    EALLOW;
    HWREGH(base + ADC_O_CTL1) &= ~ADC_CTL1_ADCPWDNZ;
    EDIS;
}

//*****************************************************************************
//
//! Forces a SOC flag to a 1 in the analog-to-digital converter.
//!
//! \param base is the base address of the ADC module.
//! \param socNumber is the number of the start-of-conversion.
//!
//! This function forces the SOC flag associated with the SOC specified by
//! \e socNumber. This initiates a conversion once that SOC is given
//! priority. This software trigger can be used whether or not the SOC has been
//! configured to accept some other specific trigger.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_forceSOC(uint32_t base, ADC_SOCNumber socNumber)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Write to the register that will force a 1 to the corresponding SOC flag
    //
    HWREGH(base + ADC_O_SOCFRC1) = ((uint16_t)1U << (uint16_t)socNumber);
}

//*****************************************************************************
//
//! Forces multiple SOC flags to 1 in the analog-to-digital converter.
//!
//! \param base is the base address of the ADC module.
//! \param socMask is the SOCs to be forced through software
//!
//! This function forces the SOCFRC1 flags associated with the SOCs specified
//! by \e socMask. This initiates a conversion once the desired SOCs are given
//! priority. This software trigger can be used whether or not the SOC has been
//! configured to accept some other specific trigger.
//! Valid values for \e socMask parameter can be any of the individual
//! ADC_FORCE_SOCx values or any of their OR'd combination to trigger multiple
//! SOCs.
//!
//! \note To trigger SOC0, SOC1 and SOC2, value (ADC_FORCE_SOC0 |
//! ADC_FORCE_SOC1 | ADC_FORCE_SOC2) should be passed as socMask.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_forceMultipleSOC(uint32_t base, uint16_t socMask)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Write to the register that will force a 1 to desired SOCs
    //
    HWREGH(base + ADC_O_SOCFRC1) = socMask;
}

//*****************************************************************************
//
//! Gets the current ADC interrupt status.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function returns the interrupt status for the analog-to-digital
//! converter.
//!
//! \e adcIntNum takes a one of the values \b ADC_INT_NUMBER1,
//! \b ADC_INT_NUMBER2, \b ADC_INT_NUMBER3 or \b ADC_INT_NUMBER4 to get
//! the interrupt status for the given interrupt number of the ADC module.
//!
//! \return \b true if the interrupt flag for the specified interrupt number is
//! set and \b false if it is not.
//
//*****************************************************************************
static inline bool
ADC_getInterruptStatus(uint32_t base, ADC_IntNumber adcIntNum)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));
    //
    // Get the specified ADC interrupt status.
    //
    return((HWREGH(base + ADC_O_INTFLG) & (1U << (uint16_t)adcIntNum)) != 0U);
}

//*****************************************************************************
//
//! Clears ADC interrupt sources.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function clears the specified ADC interrupt sources so that they no
//! longer assert. If not in continuous mode, this function must be called
//! before any further interrupt pulses may occur.
//!
//! \e adcIntNum takes a one of the values \b ADC_INT_NUMBER1,
//! \b ADC_INT_NUMBER2, \b ADC_INT_NUMBER3 or \b ADC_INT_NUMBER4 to express
//! which of the four interrupts of the ADC module should be cleared.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_clearInterruptStatus(uint32_t base, ADC_IntNumber adcIntNum)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Clear the specified interrupt.
    //
    HWREGH(base + ADC_O_INTFLGCLR) = (uint16_t)1U << (uint16_t)adcIntNum;
}

//*****************************************************************************
//
//! Gets the current ADC interrupt overflow status.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function returns the interrupt overflow status for the
//! analog-to-digital converter. An overflow condition is generated
//! irrespective of the continuous mode.
//!
//! \e adcIntNum takes a one of the values \b ADC_INT_NUMBER1,
//! \b ADC_INT_NUMBER2, \b ADC_INT_NUMBER3 or \b ADC_INT_NUMBER4 to get
//! the interrupt overflow status for the given interrupt number.
//!
//! \return \b true if the interrupt overflow flag for the specified interrupt
//! number is set and \b false if it is not.
//
//*****************************************************************************
static inline bool
ADC_getInterruptOverflowStatus(uint32_t base, ADC_IntNumber adcIntNum)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Get the specified ADC interrupt status.
    //
    return((HWREGH(base + ADC_O_INTOVF) & (1U << (uint16_t)adcIntNum)) != 0U);
}

//*****************************************************************************
//
//! Clears ADC interrupt overflow sources.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function clears the specified ADC interrupt overflow sources so that
//! they no longer assert. If software tries to clear the overflow in the same
//! cycle that hardware tries to set the overflow, then hardware has priority.
//!
//! \e adcIntNum takes a one of the values \b ADC_INT_NUMBER1,
//! \b ADC_INT_NUMBER2, \b ADC_INT_NUMBER3 or \b ADC_INT_NUMBER4 to express
//! which of the four interrupt overflow status of the ADC module
//! should be cleared.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_clearInterruptOverflowStatus(uint32_t base, ADC_IntNumber adcIntNum)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Clear the specified interrupt overflow bit.
    //
    HWREGH(base + ADC_O_INTOVFCLR) = (uint16_t)1U << (uint16_t)adcIntNum;
}

//*****************************************************************************
//
//! Reads the conversion result.
//!
//! \param resultBase is the base address of the ADC results.
//! \param socNumber is the number of the start-of-conversion.
//!
//! This function returns the conversion result that corresponds to the base
//! address passed into \e resultBase and the SOC passed into \e socNumber.
//!
//! The \e socNumber number is a value \b ADC_SOC_NUMBERX where X is a number
//! from 0 to 15 specifying which SOC's result is to be read.
//!
//! \note Take care that you are using a base address for the result registers
//! (ADCxRESULT_BASE) and not a base address for the control registers.
//!
//! \return Returns the conversion result.
//
//*****************************************************************************
static inline uint16_t
ADC_readResult(uint32_t resultBase, ADC_SOCNumber socNumber)
{
    //
    // Check the arguments.
    //
    ASSERT(
           (resultBase == ADCARESULT_BASE) ||
           (resultBase == ADCBRESULT_BASE) ||
           (resultBase == ADCCRESULT_BASE) ||
           (resultBase == ADCDRESULT_BASE)
          );
    //
    // Return the ADC result for the selected SOC.
    //
    return(HWREGH(resultBase + (uint32_t)ADC_RESULTx_OFFSET_BASE +
                  (uint32_t)socNumber));
}

//*****************************************************************************
//
//! Determines whether the ADC is busy or not.
//!
//! \param base is the base address of the ADC.
//!
//! This function allows the caller to determine whether or not the ADC is
//! busy and can sample another channel.
//!
//! \return Returns \b true if the ADC is sampling or \b false if all
//! samples are complete.
//
//*****************************************************************************
static inline bool
ADC_isBusy(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Determine if the ADC is busy.
    //
    return((HWREGH(base + ADC_O_CTL1) & ADC_CTL1_ADCBSY) != 0U);
}

//*****************************************************************************
//
//! Set SOC burst mode.
//!
//! \param base is the base address of the ADC.
//! \param trigger the source that will cause the burst conversion sequence.
//! \param burstSize is the number of SOCs converted during a burst sequence.
//!
//! This function configures the burst trigger and burstSize of an ADC module.
//! Burst mode allows a single trigger to walk through the round-robin SOCs one
//! or more at a time. When burst mode is enabled, the trigger selected by the
//! ADC_setupSOC() API will no longer have an effect on the SOCs in round-robin
//! mode. Instead, the source specified through the \e trigger parameter will
//! cause a burst of \e burstSize conversions to occur.
//!
//! The \e trigger parameter takes the same values as the ADC_setupSOC() API
//! The \e burstSize parameter should be a value between 1 and 16 inclusive.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setBurstModeConfig(uint32_t base, ADC_Trigger trigger, uint16_t burstSize)
{
    uint16_t regValue;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));
    ASSERT(((uint16_t)trigger & ~((uint16_t)0x1FU)) == 0U);
    ASSERT((burstSize >= 1U) && (burstSize <= 16U));

    //
    // Write the burst mode configuration to the register.
    //
    EALLOW;

    regValue = (uint16_t)trigger | ((burstSize - 1U) <<
                                    ADC_BURSTCTL_BURSTSIZE_S);

    HWREGH(base + ADC_O_BURSTCTL) = (HWREGH(base + ADC_O_BURSTCTL) &
                                     ~((uint16_t)ADC_BURSTCTL_BURSTTRIGSEL_M |
                                       ADC_BURSTCTL_BURSTSIZE_M)) | regValue;

    EDIS;
}

//*****************************************************************************
//
//! Enables SOC burst mode.
//!
//! \param base is the base address of the ADC.
//!
//! This function enables SOC burst mode operation of the ADC. Burst mode
//! allows a single trigger to walk through the round-robin SOCs one or more at
//! a time. When burst mode is enabled, the trigger selected by the
//! ADC_setupSOC() API will no longer have an effect on the SOCs in round-robin
//! mode. Use ADC_setBurstMode() to configure the burst trigger and size.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enableBurstMode(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Enable burst mode.
    //
    EALLOW;
    HWREGH(base + ADC_O_BURSTCTL) |= ADC_BURSTCTL_BURSTEN;
    EDIS;
}

//*****************************************************************************
//
//! Disables SOC burst mode.
//!
//! \param base is the base address of the ADC.
//!
//! This function disables SOC burst mode operation of the ADC. SOCs in
//! round-robin mode will be triggered by the trigger configured using the
//! ADC_setupSOC() API.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disableBurstMode(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Disable burst mode.
    //
    EALLOW;
    HWREGH(base + ADC_O_BURSTCTL) &= ~ADC_BURSTCTL_BURSTEN;
    EDIS;
}

//*****************************************************************************
//
//! Sets the priority mode of the SOCs.
//!
//! \param base is the base address of the ADC.
//! \param priMode is the priority mode of the SOCs.
//!
//! This function sets the priority mode of the SOCs. There are three main
//! modes that can be passed in the \e priMode parameter
//!
//! - All SOCs are in round-robin mode. This means no SOC has an inherent
//! higher priority over another. This is selected by passing in the value
//! \b ADC_PRI_ALL_ROUND_ROBIN.
//! - All priorities are in high priority mode. This means that the priority of
//! the SOC is determined by its SOC number. This option is selected by passing
//! in the value \b ADC_PRI_ALL_HIPRI.
//! - A range of SOCs are assigned high priority, with all others in round
//! robin mode. High priority mode means that an SOC with high priority will
//! interrupt the round robin wheel and insert itself as the next conversion.
//! Passing in the value \b ADC_PRI_SOC0_HIPRI will make SOC0 highest priority,
//! \b ADC_PRI_THRU_SOC1_HIPRI will put SOC0 and SOC 1 in high priority, and so
//! on up to \b ADC_PRI_THRU_SOC14_HIPRI where SOCs 0 through 14 are in high
//! priority.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setSOCPriority(uint32_t base, ADC_PriorityMode priMode)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    EALLOW;

    HWREGH(base + ADC_O_SOCPRICTL) = (HWREGH(base + ADC_O_SOCPRICTL) &
                                      ~ADC_SOCPRICTL_SOCPRIORITY_M) |
                                     (uint16_t)priMode;

    EDIS;
}

//*****************************************************************************
//
//! Configures Open/Shorts Detection Circuit Mode.
//!
//! \param base is the base address of the ADC.
//! \param modeVal is the desired open/shorts detection circuit mode.
//!
//! This function configures the open/shorts detection circuit mode of the ADC.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_configOSDetectMode(uint32_t base, ADC_OSDetectMode modeVal)
{
    //
    // Configure open/shorts detection circuit mode.
    //
    EALLOW;
    HWREGH(base + ADC_O_OSDETECT) = ((HWREGH(base + ADC_O_OSDETECT) &
                                     (~ADC_OSDETECT_DETECTCFG_M)) |
                                     (uint16_t)modeVal);
    EDIS;
}

//*****************************************************************************
//
//! Configures a post-processing block (PPB) in the ADC.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param socNumber is the number of the start-of-conversion.
//!
//! This function associates a post-processing block with a SOC.
//!
//! The \e ppbNumber is a value \b ADC_PPB_NUMBERX where X is a value from 1 to
//! 4 inclusive that identifies a PPB to be configured.  The \e socNumber
//! number is a value \b ADC_SOC_NUMBERX where X is a number from 0 to 15
//! specifying which SOC is to be configured on the ADC module specified by
//! \e base.
//!
//! \note You can have more that one PPB associated with the same SOC, but a
//! PPB can only be configured to correspond to one SOC at a time. Also note
//! that when you have multiple PPBs for the same SOC, the calibration offset
//! that actually gets applied will be that of the PPB with the highest number.
//! Since SOC0 is the default for all PPBs, look out for unintentional
//! overwriting of a lower numbered PPB's offset.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setupPPB(uint32_t base, ADC_PPBNumber ppbNumber, ADC_SOCNumber socNumber)
{
    uint32_t ppbOffset;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_PPBxCONFIG_STEP * (uint32_t)ppbNumber) + ADC_O_PPB1CONFIG;

    //
    // Write the configuration to the register.
    //
    EALLOW;
    HWREGH(base + ppbOffset) = (HWREGH(base + ppbOffset) &
                                ~ADC_PPB1CONFIG_CONFIG_M) |
                               ((uint16_t)socNumber & ADC_PPB1CONFIG_CONFIG_M);
    EDIS;
}

//*****************************************************************************
//
//! Enables individual ADC PPB event sources.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param evtFlags is a bit mask of the event sources to be enabled.
//!
//! This function enables the indicated ADC PPB event sources.  This will allow
//! the specified events to propagate through the X-BAR to a pin or to an ePWM
//! module.  The \e evtFlags parameter can be any of the \b ADC_EVT_TRIPHI,
//! \b ADC_EVT_TRIPLO, or \b ADC_EVT_ZERO values.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enablePPBEvent(uint32_t base, ADC_PPBNumber ppbNumber, uint16_t evtFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));
    ASSERT((evtFlags & ~0x7U) == 0U);

    //
    // Enable the specified event.
    //
    EALLOW;
    HWREGH(base + ADC_O_EVTSEL) |= evtFlags << ((uint16_t)ppbNumber * 4U);
    EDIS;
}

//*****************************************************************************
//
//! Disables individual ADC PPB event sources.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param evtFlags is a bit mask of the event sources to be enabled.
//!
//! This function disables the indicated ADC PPB event sources.  This will stop
//! the specified events from propagating through the X-BAR to other modules.
//! The \e evtFlags parameter can be any of the \b ADC_EVT_TRIPHI,
//! \b ADC_EVT_TRIPLO, or \b ADC_EVT_ZERO values.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disablePPBEvent(uint32_t base, ADC_PPBNumber ppbNumber, uint16_t evtFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));
    ASSERT((evtFlags & ~0x7U) == 0U);

    //
    // Disable the specified event.
    //
    EALLOW;
    HWREGH(base + ADC_O_EVTSEL) &= ~(evtFlags << ((uint16_t)ppbNumber * 4U));
    EDIS;
}

//*****************************************************************************
//
//! Enables individual ADC PPB event interrupt sources.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param intFlags is a bit mask of the interrupt sources to be enabled.
//!
//! This function enables the indicated ADC PPB interrupt sources.  Only the
//! sources that are enabled can be reflected to the processor interrupt.
//! Disabled sources have no effect on the processor.  The \e intFlags
//! parameter can be any of the \b ADC_EVT_TRIPHI, \b ADC_EVT_TRIPLO, or
//! \b ADC_EVT_ZERO values.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enablePPBEventInterrupt(uint32_t base, ADC_PPBNumber ppbNumber,
                            uint16_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));
    ASSERT((intFlags & ~0x7U) == 0U);

    //
    // Enable the specified event interrupts.
    //
    EALLOW;
    HWREGH(base + ADC_O_EVTINTSEL) |= intFlags << ((uint16_t)ppbNumber * 4U);
    EDIS;
}

//*****************************************************************************
//
//! Disables individual ADC PPB event interrupt sources.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param intFlags is a bit mask of the interrupt source to be disabled.
//!
//! This function disables the indicated ADC PPB interrupt sources.  Only the
//! sources that are enabled can be reflected to the processor interrupt.
//! Disabled sources have no effect on the processor.  The \e intFlags
//! parameter can be any of the \b ADC_EVT_TRIPHI, \b ADC_EVT_TRIPLO, or
//! \b ADC_EVT_ZERO values.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disablePPBEventInterrupt(uint32_t base, ADC_PPBNumber ppbNumber,
                             uint16_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));
    ASSERT((intFlags & ~0x7U) == 0U);

    //
    // Disable the specified event interrupts.
    //
    EALLOW;
    HWREGH(base + ADC_O_EVTINTSEL) &= ~(intFlags <<
                                        ((uint16_t)ppbNumber * 4U));
    EDIS;
}

//*****************************************************************************
//
//! Gets the current ADC event status.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the event status for the analog-to-digital converter.
//!
//! \return Returns the current event status, enumerated as a bit field of
//! \b ADC_EVT_TRIPHI, \b ADC_EVT_TRIPLO, and \b ADC_EVT_ZERO.
//
//*****************************************************************************
static inline uint16_t
ADC_getPPBEventStatus(uint32_t base, ADC_PPBNumber ppbNumber)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Get the event status for the specified post-processing block.
    //
    return((HWREGH(base + ADC_O_EVTSTAT) >> ((uint16_t)ppbNumber * 4U)) &
           0x7U);
}

//*****************************************************************************
//
//! Clears ADC event flags.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param evtFlags is a bit mask of the event source to be cleared.
//!
//! This function clears the indicated ADC PPB event flags. After an event
//! occurs this function must be called to allow additional events to be
//! produced. The \e evtFlags parameter can be any of the \b ADC_EVT_TRIPHI,
//! \b ADC_EVT_TRIPLO, or \b ADC_EVT_ZERO values.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_clearPPBEventStatus(uint32_t base, ADC_PPBNumber ppbNumber,
                        uint16_t evtFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));
    ASSERT((evtFlags & ~0x7U) == 0U);

    //
    // Clear the specified event interrupts.
    //
    HWREGH(base + ADC_O_EVTCLR) |= evtFlags << ((uint16_t)ppbNumber * 4U);
}

//*****************************************************************************
//
//! Enables cycle-by-cycle clear of ADC PPB event flags.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function enables the automatic cycle-by-cycle clear of ADC PPB event
//! flags. When enabled, the desired PPB event flags are automatically cleared
//! on the next PPBxRESULT load, unless a set condition is also occurring at
//! the same time, in which case the set takes precedence.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enablePPBEventCBCClear(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint32_t ppbOffset;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_PPBxCONFIG_STEP * (uint32_t)ppbNumber) + ADC_O_PPB1CONFIG;

    //
    // Set automatic cycle-by-cycle flag clear bit
    //
    EALLOW;
    HWREGH(base + ppbOffset) |= ADC_PPB1CONFIG_CBCEN;
    EDIS;
}

//*****************************************************************************
//
//! Disables cycle-by-cycle clear of ADC PPB event flags.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function disables the cycle-by-cycle clear of ADC PPB event flags. When
//! disabled, the desired PPB event flags are to be cleared explicitly in
//! software inorder to generate next set of interrupts/events.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disablePPBEventCBCClear(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint32_t ppbOffset;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_PPBxCONFIG_STEP * (uint32_t)ppbNumber) + ADC_O_PPB1CONFIG;

    //
    // Clear automatic cycle-by-cycle flag clear bit
    //
    EALLOW;
    HWREGH(base + ppbOffset) &= ~ADC_PPB1CONFIG_CBCEN;
    EDIS;
}


//*****************************************************************************
//
//! Reads the processed conversion result from the PPB.
//!
//! \param resultBase is the base address of the ADC results.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the processed conversion result that corresponds to
//! the base address passed into \e resultBase and the PPB passed into
//! \e ppbNumber.
//!
//! \note Take care that you are using a base address for the result registers
//! (ADCxRESULT_BASE) and not a base address for the control registers.
//!
//! \return Returns the signed 32-bit conversion result.
//
//*****************************************************************************
static inline int32_t
ADC_readPPBResult(uint32_t resultBase, ADC_PPBNumber ppbNumber)
{
    //
    // Check the arguments.
    //
    ASSERT(
           (resultBase == ADCARESULT_BASE) ||
           (resultBase == ADCBRESULT_BASE) ||
           (resultBase == ADCCRESULT_BASE) ||
           (resultBase == ADCDRESULT_BASE)
          );
    //
    // Return the result of selected PPB.
    //
    return((int32_t)HWREG(resultBase + (uint32_t)ADC_PPBxRESULT_OFFSET_BASE +
           ((uint32_t)ppbNumber * 2UL)));
}

//*****************************************************************************
//
//! Reads sample delay time stamp from a PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function returns the sample delay time stamp. This delay is the number
//! of system clock cycles between the SOC being triggered and when it began
//! converting.
//!
//! \return Returns the delay time stamp.
//
//*****************************************************************************
static inline uint16_t
ADC_getPPBDelayTimeStamp(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint32_t ppbOffset;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Get the offset to the appropriate delay.
    //
    ppbOffset = (ADC_PPBxSTAMP_STEP * (uint32_t)ppbNumber) + ADC_O_PPB1STAMP;

    //
    // Return the delay time stamp.
    //
    return(HWREGH(base + ppbOffset) & ADC_PPB2STAMP_DLYSTAMP_M);
}

//*****************************************************************************
//
//! Sets the post processing block offset correction.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param offset is the 10-bit signed value subtracted from ADC the output.
//!
//! This function sets the PPB offset correction value.  This value can be used
//! to digitally remove any system-level offset inherent in the ADCIN circuit
//! before it is stored in the appropriate result register. The \e offset
//! parameter is \b subtracted from the ADC output and is a signed value from
//! -512 to 511 inclusive. For example, when \e offset = 1, ADCRESULT = ADC
//! output - 1. When \e offset = -512, ADCRESULT = ADC output - (-512) or ADC
//! output + 512.
//!
//! Passing a zero in to the \e offset parameter will effectively disable the
//! calculation, allowing the raw ADC result to be passed unchanged into the
//! result register.
//!
//! \note If multiple PPBs are applied to the same SOC, the offset that will be
//! applied will be that of the PPB with the highest number.
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_setPPBCalibrationOffset(uint32_t base, ADC_PPBNumber ppbNumber,
                            int16_t offset)
{
    uint32_t ppbOffset;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Get the offset to the appropriate offset register.
    //
    ppbOffset = (ADC_PPBxOFFCAL_STEP * (uint32_t)ppbNumber) + ADC_O_PPB1OFFCAL;

    //
    // Write the offset amount.
    //
    EALLOW;
    HWREGH(base + ppbOffset) = (HWREGH(base + ppbOffset) &
                                ~ADC_PPB1OFFCAL_OFFCAL_M) |
                               ((uint16_t)offset & ADC_PPB1OFFCAL_OFFCAL_M);
    EDIS;
}

//*****************************************************************************
//
//! Sets the post processing block reference offset.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param offset is the 16-bit unsigned value subtracted from ADC the output.
//!
//! This function sets the PPB reference offset value. This can be used to
//! either calculate the feedback error or convert a unipolar signal to bipolar
//! by subtracting a reference value. The result will be stored in the
//! appropriate PPB result register which can be read using ADC_readPPBResult().
//!
//! Passing a zero in to the \e offset parameter will effectively disable the
//! calculation and will pass the ADC result to the PPB result register
//! unchanged.
//!
//! \note If in 12-bit mode, you may only pass a 12-bit value into the \e offset
//! parameter.
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_setPPBReferenceOffset(uint32_t base, ADC_PPBNumber ppbNumber,
                          uint16_t offset)
{
    uint32_t ppbOffset;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Get the offset to the appropriate offset register.
    //
    ppbOffset = (ADC_PPBxOFFREF_STEP * (uint32_t)ppbNumber) + ADC_O_PPB1OFFREF;

    //
    // Write the offset amount.
    //
    HWREGH(base + ppbOffset) = offset;
}

//*****************************************************************************
//
//! Enables two's complement capability in the PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function enables two's complement in the post-processing block
//! specified by the \e ppbNumber parameter. When enabled, a two's complement
//! will be performed on the output of the offset subtraction before it is
//! stored in the appropriate PPB result register. In other words, the PPB
//! result will be the reference offset value minus the the ADC result value
//! (ADCPPBxRESULT = ADCSOCxOFFREF - ADCRESULTx).
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_enablePPBTwosComplement(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint32_t ppbOffset;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_PPBxCONFIG_STEP * (uint32_t)ppbNumber) + ADC_O_PPB1CONFIG;

    //
    // Enable PPB two's complement.
    //
    EALLOW;
    HWREGH(base + ppbOffset) |= ADC_PPB1CONFIG_TWOSCOMPEN;
    EDIS;
}

//*****************************************************************************
//
//! Disables two's complement capability in the PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//!
//! This function disables two's complement in the post-processing block
//! specified by the \e ppbNumber parameter. When disabled, a two's complement
//! will \b NOT be performed on the output of the offset subtraction before it
//! is stored in the appropriate PPB result register. In other words, the PPB
//! result will be the ADC result value minus the reference offset value
//! (ADCPPBxRESULT = ADCRESULTx - ADCSOCxOFFREF).
//!
//! \return None
//
//*****************************************************************************
static inline void
ADC_disablePPBTwosComplement(uint32_t base, ADC_PPBNumber ppbNumber)
{
    uint32_t ppbOffset;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Get the offset to the appropriate PPB configuration register.
    //
    ppbOffset = (ADC_PPBxCONFIG_STEP * (uint32_t)ppbNumber) + ADC_O_PPB1CONFIG;

    //
    // Disable PPB two's complement.
    //
    EALLOW;
    HWREGH(base + ppbOffset) &= ~ADC_PPB1CONFIG_TWOSCOMPEN;
    EDIS;
}

//*****************************************************************************
//
//! Enables an ADC interrupt source.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function enables the indicated ADC interrupt source.  Only the
//! sources that are enabled can be reflected to the processor interrupt.
//! Disabled sources have no effect on the processor.
//!
//! \e adcIntNum can take the value \b ADC_INT_NUMBER1,
//! \b ADC_INT_NUMBER2, \b ADC_INT_NUMBER3 or \b ADC_INT_NUMBER4 to express
//! which of the four interrupts of the ADC module should be enabled.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enableInterrupt(uint32_t base, ADC_IntNumber adcIntNum)
{
    uint32_t intRegAddr;
    uint16_t shiftVal;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Each INTSEL register manages two interrupts. If the interrupt number is
    // even, we'll be accessing the upper byte and will need to shift.
    //
    intRegAddr = base + ADC_INTSELxNy_OFFSET_BASE + ((uint32_t)adcIntNum >> 1);
    shiftVal = ((uint16_t)adcIntNum & 0x1U) << 3U;

    //
    // Enable the specified ADC interrupt.
    //
    EALLOW;

    HWREGH(intRegAddr) |= ADC_INTSEL1N2_INT1E << shiftVal;

    EDIS;
}

//*****************************************************************************
//
//! Disables an ADC interrupt source.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function disables the indicated ADC interrupt source.
//! Only the sources that are enabled can be reflected to the processor
//! interrupt. Disabled sources have no effect on the processor.
//!
//! \e adcIntNum can take the value \b ADC_INT_NUMBER1,
//! \b ADC_INT_NUMBER2, \b ADC_INT_NUMBER3 or \b ADC_INT_NUMBER4 to express
//! which of the four interrupts of the ADC module should be disabled.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disableInterrupt(uint32_t base, ADC_IntNumber adcIntNum)
{
    uint32_t intRegAddr;
    uint16_t shiftVal;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Each INTSEL register manages two interrupts. If the interrupt number is
    // even, we'll be accessing the upper byte and will need to shift.
    //
    intRegAddr = base + ADC_INTSELxNy_OFFSET_BASE + ((uint32_t)adcIntNum >> 1);
    shiftVal = ((uint16_t)adcIntNum & 0x1U) << 3U;

    //
    // Disable the specified ADC interrupt.
    //
    EALLOW;

    HWREGH(intRegAddr) &= ~(ADC_INTSEL1N2_INT1E << shiftVal);

    EDIS;
}

//*****************************************************************************
//
//! Sets the source EOC for an analog-to-digital converter interrupt.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//! \param socNumber is the number of the start-of-conversion.
//!
//! This function sets which conversion is the source of an ADC interrupt.
//!
//! The \e intTrigger number is a value \b ADC_SOC_NUMBERX where X is a number
//! from 0 to 15 specifying which EOC is to be configured on the ADC module
//! specified by \e base. Refer \b ADC_SOCNumber enum for valid values for
//! this input.
//!
//! \e adcIntNum can take the value \b ADC_INT_NUMBER1,
//! \b ADC_INT_NUMBER2, \b ADC_INT_NUMBER3 or \b ADC_INT_NUMBER4 to express
//! which of the four interrupts of the ADC module is being configured.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_setInterruptSource(uint32_t base, ADC_IntNumber adcIntNum,
                       uint16_t intTrigger)
{
    uint32_t intRegAddr;
    uint16_t shiftVal;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));
    ASSERT(intTrigger < 16U);

    //
    // Each INTSEL register manages two interrupts. If the interrupt number is
    // even, we'll be accessing the upper byte and will need to shift.
    //
    intRegAddr = base + ADC_INTSELxNy_OFFSET_BASE + ((uint32_t)adcIntNum >> 1);
    shiftVal = ((uint16_t)adcIntNum & 0x1U) << 3U;

    //
    // Set the specified ADC interrupt source.
    //
    EALLOW;

    HWREGH(intRegAddr) =
        (HWREGH(intRegAddr) & ~(ADC_INTSEL1N2_INT1SEL_M << shiftVal)) |
        ((uint16_t)intTrigger << shiftVal);

    EDIS;
}

//*****************************************************************************
//
//! Enables continuous mode for an ADC interrupt.
//!
//! \param base is the base address of the ADC.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function enables continuous mode for the ADC interrupt passed into
//! \e adcIntNum. This means that pulses will be generated for the specified
//! ADC interrupt whenever an EOC pulse is generated irrespective of whether or
//! not the flag bit is set.
//!
//! \e adcIntNum can take the value \b ADC_INT_NUMBER1,
//! \b ADC_INT_NUMBER2, \b ADC_INT_NUMBER3 or \b ADC_INT_NUMBER4 to express
//! which of the four interrupts of the ADC module is being configured.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_enableContinuousMode(uint32_t base, ADC_IntNumber adcIntNum)
{
    uint32_t intRegAddr;
    uint16_t shiftVal;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Each INTSEL register manages two interrupts. If the interrupt number is
    // even, we'll be accessing the upper byte and will need to shift.
    //
    intRegAddr = base + ADC_INTSELxNy_OFFSET_BASE + ((uint32_t)adcIntNum >> 1);
    shiftVal = ((uint16_t)adcIntNum & 0x1U) << 3U;

    //
    // Enable continuous mode for the specified ADC interrupt.
    //
    EALLOW;

    HWREGH(intRegAddr) |= ADC_INTSEL1N2_INT1CONT << shiftVal;

    EDIS;
}

//*****************************************************************************
//
//! Disables continuous mode for an ADC interrupt.
//!
//! \param base is the base address of the ADC.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function disables continuous mode for the ADC interrupt passed into
//! \e adcIntNum. This means that pulses will not be generated for the
//! specified ADC interrupt until the corresponding interrupt flag for the
//! previous interrupt occurrence has been cleared using
//! ADC_clearInterruptStatus().
//!
//! \e adcIntNum can take the value \b ADC_INT_NUMBER1,
//! \b ADC_INT_NUMBER2, \b ADC_INT_NUMBER3 or \b ADC_INT_NUMBER4 to express
//! which of the four interrupts of the ADC module is being configured.
//!
//! \return None.
//
//*****************************************************************************
static inline void
ADC_disableContinuousMode(uint32_t base, ADC_IntNumber adcIntNum)
{
    uint32_t intRegAddr;
    uint16_t shiftVal;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Each INTSEL register manages two interrupts. If the interrupt number is
    // even, we'll be accessing the upper byte and will need to shift.
    //
    intRegAddr = base + ADC_INTSELxNy_OFFSET_BASE + ((uint32_t)adcIntNum >> 1);
    shiftVal = ((uint16_t)adcIntNum & 0x1U) << 3U;

    //
    // Disable continuous mode for the specified ADC interrupt.
    //
    EALLOW;

    HWREGH(intRegAddr) &= ~(ADC_INTSEL1N2_INT1CONT << shiftVal);

    EDIS;
}

//*****************************************************************************
//
//! Converts temperature from sensor reading to degrees C
//!
//! \param tempResult is the raw ADC A conversion result from the temp sensor.
//! \param vref is the reference voltage being used (for example 3.3 for 3.3V).
//!
//! This function converts temperature from temp sensor reading to degrees C.
//! Temp sensor values in production test are derived with 2.5V reference.
//! The \b vref argument in the function is used to scale the temp sensor
//! reading accordingly if temp sensor value is read at a different VREF
//! setting.
//!
//! \return Returns the temperature sensor reading converted to degrees C.
//
//*****************************************************************************
static inline int16_t
ADC_getTemperatureC(uint16_t tempResult, float32_t vref)
{
    //
    // Read temp sensor slope and offset locations from OTP and convert
    //
    return((int16_t)((((float32_t)tempResult * vref / 4096.0F) -
                      ADC_EXT_REF_TSOFFSET) / ADC_EXT_REF_TSSLOPE));
}

//*****************************************************************************
//
//! Converts temperature from sensor reading to degrees K
//!
//! \param tempResult is the raw ADC A conversion result from the temp sensor.
//! \param vref is the reference voltage being used (for example 3.3 for 3.3V).
//!
//! This function converts temperature from temp sensor reading to degrees K.
//! Temp sensor values in production test are derived with 2.5V reference.
//! The \b vref argument in the function is used to scale the temp sensor
//! reading accordingly if temp sensor value is read at a different VREF
//! setting.
//!
//! \return Returns the temperature sensor reading converted to degrees K.
//
//*****************************************************************************
static inline int16_t
ADC_getTemperatureK(uint16_t tempResult, float32_t vref)
{
    //
    // Read temp sensor slope and offset locations from OTP and convert
    //
    return((int16_t)(((((float32_t)tempResult * vref / 4096.0F) -
                       ADC_EXT_REF_TSOFFSET) / ADC_EXT_REF_TSSLOPE) + 273.0F));
}

//*****************************************************************************
//
//! Configures the analog-to-digital converter resolution and signal mode.
//!
//! \param base is the base address of the ADC module.
//! \param resolution is the resolution of the converter (12 or 16 bits).
//! \param signalMode is the input signal mode of the converter.
//!
//! This function configures the ADC module's conversion resolution and input
//! signal mode and ensures that the corresponding trims are loaded.
//!
//! The \e resolution parameter specifies the resolution of the conversion.
//! It can be 12-bit or 16-bit specified by \b ADC_RESOLUTION_12BIT
//! or \b ADC_RESOLUTION_16BIT.
//!
//! The \e signalMode parameter specifies the signal mode. In single-ended
//! mode, which is indicated by \b ADC_MODE_SINGLE_ENDED, the input voltage is
//! sampled on a single pin referenced to VREFLO. In differential mode, which
//! is indicated by \b ADC_MODE_DIFFERENTIAL, the input voltage to the
//! converter is sampled on a pair of input pins, a positive and a negative.
//!
//! \b Note: In this device, differential signal conversions are supported
//!          only in 16-bit resolution mode and single-ended signal conversions
//!          are supported in both 12-bit & 16-bit resolution mode.
//!
//! \return None.
//
//*****************************************************************************
extern void
ADC_setMode(uint32_t base, ADC_Resolution resolution,
            ADC_SignalMode signalMode);

//*****************************************************************************
//
//! Configures the offset trim for the desired ADC instance
//!
//! \param base is the base address of the ADC module.
//!
//! This function loads the offset trims for the desired ADC instance.
//!
//! \return None.
//
//*****************************************************************************
extern void
ADC_setOffsetTrim(uint32_t base);

//*****************************************************************************
//
//! Configures the INL trim for the desired ADC instance
//!
//! \param base is the base address of the ADC module.
//!
//! This function loads the INL trims for the desired ADC instance.
//!
//! \return None.
//
//*****************************************************************************
extern void
ADC_setINLTrim(uint32_t base);

//*****************************************************************************
//
//! Sets the windowed trip limits for a PPB.
//!
//! \param base is the base address of the ADC module.
//! \param ppbNumber is the number of the post-processing block.
//! \param tripHiLimit is the value is the digital comparator trip high limit.
//! \param tripLoLimit is the value is the digital comparator trip low limit.
//!
//! This function sets the windowed trip limits for a PPB. These values set
//! the digital comparator so that when one of the values is exceeded, either a
//! high or low trip event will occur.
//!
//! The \e ppbNumber is a value \b ADC_PPB_NUMBERX where X is a value from 1 to
//! 4 inclusive that identifies a PPB to be configured.
//!
//! If using 16-bit mode, you may pass a 17-bit number into the \e tripHiLimit
//! and \e tripLoLimit parameters where the 17th bit is the sign bit (that is
//! a value from -65536 and 65535). In 12-bit mode, only bits 12:0 will be
//! compared against bits 12:0 of the PPB result.
//!
//!
//! \return None.
//
//*****************************************************************************
extern void
ADC_setPPBTripLimits(uint32_t base, ADC_PPBNumber ppbNumber,
                     int32_t tripHiLimit, int32_t tripLoLimit);

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

#endif // ADC_H
