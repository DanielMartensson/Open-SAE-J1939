//#############################################################################
//
// FILE:   epwm.h
//
// TITLE:   C28x EPWM Driver
//
//#############################################################################
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

#ifndef EPWM_H
#define EPWM_H

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
//! \addtogroup epwm_api ePWM
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_epwm.h"
#include "cpu.h"
#include "debug.h"
#include "sysctl.h"


//*****************************************************************************
//
// Defines for the API.
//
//*****************************************************************************
//*****************************************************************************
//
// Define to specify mask for source parameter for
// EPWM_enableSyncOutPulseSource() & EPWM_disableSyncOutPulseSource()
//
//*****************************************************************************
#define EPWM_SYNC_OUT_SOURCE_M    ((uint16_t)EPWM_SYNCOUTEN_SWEN             |\
                                   (uint16_t)EPWM_SYNCOUTEN_ZEROEN           |\
                                   (uint16_t)EPWM_SYNCOUTEN_CMPBEN           |\
                                   (uint16_t)EPWM_SYNCOUTEN_CMPCEN           |\
                                   (uint16_t)EPWM_SYNCOUTEN_CMPDEN           |\
                                   (uint16_t)EPWM_SYNCOUTEN_DCAEVT1EN        |\
                                   (uint16_t)EPWM_SYNCOUTEN_DCBEVT1EN)

//*****************************************************************************
//
// Values that can be passed to EPWM_enableSyncOutPulseSource() &
// EPWM_disableSyncOutPulseSource() as the \e mode parameter.
//
//*****************************************************************************
//! Software force generated EPWM sync-out pulse
#define EPWM_SYNC_OUT_PULSE_ON_SOFTWARE         EPWM_SYNCOUTEN_SWEN
//! Counter zero event generates EPWM sync-out pulse
#define EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO        EPWM_SYNCOUTEN_ZEROEN
//! Counter equal to CMPB event generates EPWM sync-out pulse
#define EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_B   EPWM_SYNCOUTEN_CMPBEN
//! Counter equal to CMPC event generates EPWM sync-out pulse
#define EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_C   EPWM_SYNCOUTEN_CMPCEN
//! Counter equal to CMPD event generates EPWM sync-out pulse
#define EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_D   EPWM_SYNCOUTEN_CMPDEN
//! DCA Event 1 Sync signal generates EPWM sync-out pulse
#define EPWM_SYNC_OUT_PULSE_ON_DCA_EVT1_SYNC    EPWM_SYNCOUTEN_DCAEVT1EN
//! DCB Event 1 Sync signal generates EPWM sync-out pulse
#define EPWM_SYNC_OUT_PULSE_ON_DCB_EVT1_SYNC    EPWM_SYNCOUTEN_DCBEVT1EN
//! Enable all the above sources
#define EPWM_SYNC_OUT_PULSE_ON_ALL              EPWM_SYNC_OUT_SOURCE_M

//
// Time Base Module
//
//*****************************************************************************
//
//! Values that can be passed to EPWM_setEmulationMode() as the
//! \e emulationMode parameter.
//
//*****************************************************************************
typedef enum
{
   //! Stop after next Time Base counter increment or decrement
   EPWM_EMULATION_STOP_AFTER_NEXT_TB = 0,
   //! Stop when counter completes whole cycle
   EPWM_EMULATION_STOP_AFTER_FULL_CYCLE = 1,
   //! Free run
   EPWM_EMULATION_FREE_RUN = 2
} EPWM_EmulationMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setCountModeAfterSync() as the
//! \e mode parameter.
//
//*****************************************************************************
typedef enum
{
   EPWM_COUNT_MODE_DOWN_AFTER_SYNC = 0, //!< Count down after sync event
   EPWM_COUNT_MODE_UP_AFTER_SYNC = 1    //!< Count up after sync event
} EPWM_SyncCountMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setClockPrescaler() as the
//! \e prescaler parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_CLOCK_DIVIDER_1 = 0,     //!< Divide clock by 1
    EPWM_CLOCK_DIVIDER_2 = 1,     //!< Divide clock by 2
    EPWM_CLOCK_DIVIDER_4 = 2,     //!< Divide clock by 4
    EPWM_CLOCK_DIVIDER_8 = 3,     //!< Divide clock by 8
    EPWM_CLOCK_DIVIDER_16 = 4,    //!< Divide clock by 16
    EPWM_CLOCK_DIVIDER_32 = 5,    //!< Divide clock by 32
    EPWM_CLOCK_DIVIDER_64 = 6,    //!< Divide clock by 64
    EPWM_CLOCK_DIVIDER_128 = 7    //!< Divide clock by 128
} EPWM_ClockDivider;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setClockPrescaler() as the
//! \e highSpeedPrescaler parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_HSCLOCK_DIVIDER_1 = 0,     //!< Divide clock by 1
    EPWM_HSCLOCK_DIVIDER_2 = 1,     //!< Divide clock by 2
    EPWM_HSCLOCK_DIVIDER_4 = 2,     //!< Divide clock by 4
    EPWM_HSCLOCK_DIVIDER_6 = 3,     //!< Divide clock by 6
    EPWM_HSCLOCK_DIVIDER_8 = 4,     //!< Divide clock by 8
    EPWM_HSCLOCK_DIVIDER_10 = 5,    //!< Divide clock by 10
    EPWM_HSCLOCK_DIVIDER_12 = 6,    //!< Divide clock by 12
    EPWM_HSCLOCK_DIVIDER_14 = 7     //!< Divide clock by 14
} EPWM_HSClockDivider;


//*****************************************************************************
//
//! Values that can be passed to EPWM_setSyncInPulseSource() as the \e mode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! Disable Sync-in
    EPWM_SYNC_IN_PULSE_SRC_DISABLE         = 0x0,
    //! Sync-in source is EPWM1 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1   = 0x1,
    //! Sync-in source is EPWM2 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM2   = 0x2,
    //! Sync-in source is EPWM3 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM3   = 0x3,
    //! Sync-in source is EPWM4 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM4   = 0x4,
    //! Sync-in source is EPWM5 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM5   = 0x5,
    //! Sync-in source is EPWM6 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM6   = 0x6,
    //! Sync-in source is EPWM7 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM7   = 0x7,
    //! Sync-in source is EPWM8 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM8   = 0x8,
    //! Sync-in source is EPWM9 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM9   = 0x9,
    //! Sync-in source is EPWM10 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM10  = 0xA,
    //! Sync-in source is EPWM11 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM11  = 0xB,
    //! Sync-in source is EPWM12 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM12  = 0xC,
    //! Sync-in source is EPWM13 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM13  = 0xD,
    //! Sync-in source is EPWM14 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM14  = 0xE,
    //! Sync-in source is EPWM15 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM15  = 0xF,
    //! Sync-in source is EPWM16 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM16  = 0x10,
    //! Sync-in source is ECAP1 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP1   = 0x11,
    //! Sync-in source is ECAP2 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP2   = 0x12,
    //! Sync-in source is ECAP3 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP3   = 0x13,
    //! Sync-in source is ECAP4 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP4   = 0x14,
    //! Sync-in source is ECAP5 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP5   = 0x15,
    //! Sync-in source is ECAP6 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP6   = 0x16,
    //! Sync-in source is ECAP7 sync-out signal
    EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP7  = 0x17,
    //! Sync-in source is Input XBAR out5 signal
    EPWM_SYNC_IN_PULSE_SRC_INPUTXBAR_OUT5 = 0x18,
    //! Sync-in source is Input XBAR out6 signal
    EPWM_SYNC_IN_PULSE_SRC_INPUTXBAR_OUT6 = 0x19,
    //! Sync-in source is Ethercat sync0 signal
    EPWM_SYNC_IN_PULSE_SRC_ETHERCAT_SYNC0 = 0x1A,
    //! Sync-in source is Ethercat sync1 signal
    EPWM_SYNC_IN_PULSE_SRC_ETHERCAT_SYNC1 = 0x1B,
} EPWM_SyncInPulseSource;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setOneShotSyncOutTrigger() as the
//! \e trigger parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_OSHT_SYNC_OUT_TRIG_SYNC   = 0x0, //!< Trigger is OSHT sync
    EPWM_OSHT_SYNC_OUT_TRIG_RELOAD = 0x1  //!< Trigger is OSHT reload
} EPWM_OneShotSyncOutTrigger;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setPeriodLoadMode()  as the
//! \e loadMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! PWM Period register access is through shadow register
    EPWM_PERIOD_SHADOW_LOAD = 0,
    //! PWM Period register access is directly
    EPWM_PERIOD_DIRECT_LOAD = 1
} EPWM_PeriodLoadMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTimeBaseCounterMode() as the
//! \e counterMode parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_COUNTER_MODE_UP = 0,         //!< Up - count mode
    EPWM_COUNTER_MODE_DOWN = 1,       //!< Down - count mode
    EPWM_COUNTER_MODE_UP_DOWN = 2,    //!< Up - down - count mode
    EPWM_COUNTER_MODE_STOP_FREEZE = 3 //!< Stop - Freeze counter
} EPWM_TimeBaseCountMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_selectPeriodLoadEvent() as the
//! \e shadowLoadMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Shadow to active load occurs when time base counter reaches 0
    EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO = 0,
    //! Shadow to active load occurs when time base counter reaches 0 and a
    //! SYNC occurs
    EPWM_SHADOW_LOAD_MODE_COUNTER_SYNC = 1,
    //! Shadow to active load occurs only when a SYNC occurs
    EPWM_SHADOW_LOAD_MODE_SYNC         = 2
} EPWM_PeriodShadowLoadMode;

//*****************************************************************************
//
// Values that can be returned by the EPWM_getTimeBaseCounterDirection()
//
//*****************************************************************************
//! Time base counter is counting up
//!
#define EPWM_TIME_BASE_STATUS_COUNT_UP                1U
//! Time base counter is counting down
//!
#define EPWM_TIME_BASE_STATUS_COUNT_DOWN              0U

//*****************************************************************************
//
//! Values that can be passed to EPWM_setupEPWMLinks() as the \e epwmLink
//! parameter.
//
//*****************************************************************************
typedef enum
{
     EPWM_LINK_WITH_EPWM_1 = 0,   //!< link current ePWM with ePWM1
     EPWM_LINK_WITH_EPWM_2 = 1,   //!< link current ePWM with ePWM2
     EPWM_LINK_WITH_EPWM_3 = 2,   //!< link current ePWM with ePWM3
     EPWM_LINK_WITH_EPWM_4 = 3,   //!< link current ePWM with ePWM4
     EPWM_LINK_WITH_EPWM_5 = 4,   //!< link current ePWM with ePWM5
     EPWM_LINK_WITH_EPWM_6 = 5,   //!< link current ePWM with ePWM6
     EPWM_LINK_WITH_EPWM_7 = 6,   //!< link current ePWM with ePWM7
     EPWM_LINK_WITH_EPWM_8 = 7,   //!< link current ePWM with ePWM8
     EPWM_LINK_WITH_EPWM_9 = 8,   //!< link current ePWM with ePWM9
     EPWM_LINK_WITH_EPWM_10 = 9,   //!< link current ePWM with ePWM10
     EPWM_LINK_WITH_EPWM_11 = 10,   //!< link current ePWM with ePWM11
     EPWM_LINK_WITH_EPWM_12 = 11,   //!< link current ePWM with ePWM12
     EPWM_LINK_WITH_EPWM_13 = 12,   //!< link current ePWM with ePWM13
     EPWM_LINK_WITH_EPWM_14 = 13,   //!< link current ePWM with ePWM14
     EPWM_LINK_WITH_EPWM_15 = 14,   //!< link current ePWM with ePWM15
     EPWM_LINK_WITH_EPWM_16 = 15    //!< link current ePWM with ePWM16
} EPWM_CurrentLink;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setupEPWMLinks() as the \e linkComp
//! parameter.
//
//*****************************************************************************
typedef enum
{
     EPWM_LINK_TBPRD = 0U,   //!< link TBPRD registers
     EPWM_LINK_COMP_A = 4U,   //!< link COMPA registers
     EPWM_LINK_COMP_B = 8U,   //!< link COMPB registers
     EPWM_LINK_COMP_C = 12U,   //!< link COMPC registers
     EPWM_LINK_COMP_D = 16U,   //!< link COMPD registers
     EPWM_LINK_GLDCTL2 = 28U    //!< link GLDCTL2 registers
} EPWM_LinkComponent;

//
// Counter Compare Module
//
//*****************************************************************************
//
//! Values that can be passed to the EPWM_getCounterCompareShadowStatus(),
//! EPWM_setCounterCompareValue(), EPWM_setCounterCompareShadowLoadMode(),
//! EPWM_disableCounterCompareShadowLoadMode()
//! as the \e compModule parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_COUNTER_COMPARE_A = 0, //!< Counter compare A
    EPWM_COUNTER_COMPARE_B = 2, //!< Counter compare B
    EPWM_COUNTER_COMPARE_C = 5, //!< Counter compare C
    EPWM_COUNTER_COMPARE_D = 7  //!< Counter compare D
} EPWM_CounterCompareModule;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setCounterCompareShadowLoadMode() as the
//! \e loadMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Load when counter equals zero
    EPWM_COMP_LOAD_ON_CNTR_ZERO = 0,
    //! Load when counter equals period
    EPWM_COMP_LOAD_ON_CNTR_PERIOD = 1,
    //! Load when counter equals zero or period
    EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD = 2,
    //! Freeze shadow to active load
    EPWM_COMP_LOAD_FREEZE = 3,
    //! Load on sync or when counter equals zero
    EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO = 4,
    //! Load on sync or when counter equals period
    EPWM_COMP_LOAD_ON_SYNC_CNTR_PERIOD = 5,
    //! Load on sync or when counter equals zero or period
    EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO_PERIOD = 6,
    //! Load on sync only
    EPWM_COMP_LOAD_ON_SYNC_ONLY = 8
} EPWM_CounterCompareLoadMode;

//
// Action Qualifier Module
//
//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierShadowLoadMode() and
//! EPWM_disableActionQualifierShadowLoadMode() as the \e aqModule parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_ACTION_QUALIFIER_A = 0, //!< Action Qualifier A
    EPWM_ACTION_QUALIFIER_B = 2  //!< Action Qualifier B
} EPWM_ActionQualifierModule;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierShadowLoadMode() as the
//! \e loadMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Load when counter equals zero
    EPWM_AQ_LOAD_ON_CNTR_ZERO = 0,
    //! Load when counter equals period
    EPWM_AQ_LOAD_ON_CNTR_PERIOD = 1,
    //! Load when counter equals zero or period
    EPWM_AQ_LOAD_ON_CNTR_ZERO_PERIOD = 2,
    //! Freeze shadow to active load
    EPWM_AQ_LOAD_FREEZE = 3,
    //! Load on sync or when counter equals zero
    EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO = 4,
    //! Load on sync or when counter equals period
    EPWM_AQ_LOAD_ON_SYNC_CNTR_PERIOD = 5,
    //! Load on sync or when counter equals zero or period
    EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO_PERIOD = 6,
    //! Load on sync only
    EPWM_AQ_LOAD_ON_SYNC_ONLY = 8
} EPWM_ActionQualifierLoadMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierT1TriggerSource() and
//! EPWM_setActionQualifierT2TriggerSource() as the \e trigger parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1 = 0,      //!< Digital compare event A 1
    EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_2 = 1,      //!< Digital compare event A 2
    EPWM_AQ_TRIGGER_EVENT_TRIG_DCB_1 = 2,      //!< Digital compare event B 1
    EPWM_AQ_TRIGGER_EVENT_TRIG_DCB_2 = 3,      //!< Digital compare event B 2
    EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_1  = 4,      //!< Trip zone 1
    EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_2  = 5,      //!< Trip zone 2
    EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_3  = 6,      //!< Trip zone 3
    EPWM_AQ_TRIGGER_EVENT_TRIG_EPWM_SYNCIN = 7,//!< ePWM sync
    EPWM_AQ_TRIGGER_EVENT_TRIG_DC_EVTFILT = 8  //!< Digital compare filter event
} EPWM_ActionQualifierTriggerSource;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierAction() as the \e
//! event parameter.
//
//*****************************************************************************
typedef enum
{
    //! Time base counter equals zero
    EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO       = 0,
    //! Time base counter equals period
    EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD     = 2,
    //! Time base counter up equals COMPA
    EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA    = 4,
    //! Time base counter down equals COMPA
    EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA  = 6,
    //! Time base counter up equals COMPB
    EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB    = 8,
    //! Time base counter down equals COMPB
    EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB  = 10,
    //! T1 event on count up
    EPWM_AQ_OUTPUT_ON_T1_COUNT_UP         = 1,
    //! T1 event on count down
    EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN       = 3,
    //! T2 event on count up
    EPWM_AQ_OUTPUT_ON_T2_COUNT_UP         = 5,
    //! T2 event on count down
    EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN       = 7
} EPWM_ActionQualifierOutputEvent;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierSWAction(),
//! EPWM_setActionQualifierAction() as the \e outPut parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_AQ_OUTPUT_NO_CHANGE = 0,  //!< No change in the output pins
    EPWM_AQ_OUTPUT_LOW       = 1,  //!< Set output pins to low
    EPWM_AQ_OUTPUT_HIGH      = 2,  //!< Set output pins to High
    EPWM_AQ_OUTPUT_TOGGLE    = 3   //!< Toggle the output pins
} EPWM_ActionQualifierOutput;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierContSWForceAction()
//! as the \e outPut parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_AQ_SW_DISABLED         = 0,  //!< Software forcing disabled
    EPWM_AQ_SW_OUTPUT_LOW       = 1,  //!< Set output pins to low
    EPWM_AQ_SW_OUTPUT_HIGH      = 2   //!< Set output pins to High
} EPWM_ActionQualifierSWOutput;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierActionComplete()
//! as the \e action parameter.
//
//*****************************************************************************
typedef enum
{
    //! Time base counter equals zero and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_ZERO = 0x0,
    //! Time base counter equals zero and set output pins to low
    EPWM_AQ_OUTPUT_LOW_ZERO = 0x1,
    //! Time base counter equals zero and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_ZERO = 0x2,
    //! Time base counter equals zero and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_ZERO = 0x3,
    //! Time base counter equals period and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_PERIOD = 0x0,
    //! Time base counter equals period and set output pins to low
    EPWM_AQ_OUTPUT_LOW_PERIOD = 0x4,
    //! Time base counter equals period and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_PERIOD = 0x8,
    //! Time base counter equals period and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_PERIOD = 0xC,
    //! Time base counter up equals COMPA and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_UP_CMPA = 0x00,
    //! Time base counter up equals COMPA and set output pins to low
    EPWM_AQ_OUTPUT_LOW_UP_CMPA = 0x10,
    //! Time base counter up equals COMPA and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_UP_CMPA = 0x20,
    //! Time base counter up equals COMPA and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_UP_CMPA = 0x30,
    //! Time base counter down equals COMPA and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_CMPA = 0x00,
    //! Time base counter down equals COMPA and set output pins to low
    EPWM_AQ_OUTPUT_LOW_DOWN_CMPA = 0x40,
    //! Time base counter down equals COMPA and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_DOWN_CMPA = 0x80,
    //! Time base counter down equals COMPA and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_DOWN_CMPA = 0xC0,
    //! Time base counter up equals COMPB and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_UP_CMPB = 0x000,
    //! Time base counter up equals COMPB and set output pins to low
    EPWM_AQ_OUTPUT_LOW_UP_CMPB = 0x100,
    //! Time base counter up equals COMPB and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_UP_CMPB = 0x200,
    //! Time base counter up equals COMPB and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_UP_CMPB = 0x300,
    //! Time base counter down equals COMPB and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_CMPB = 0x000,
    //! Time base counter down equals COMPB and set output pins to low
    EPWM_AQ_OUTPUT_LOW_DOWN_CMPB = 0x400,
    //! Time base counter down equals COMPB and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_DOWN_CMPB = 0x800,
    //! Time base counter down equals COMPB and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_DOWN_CMPB = 0xC00
} EPWM_ActionQualifierEventAction;

//*****************************************************************************
//
//! Values that can be passed to
//! EPWM_setAdditionalActionQualifierActionComplete()  as the \e action
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! T1 event on count up and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_UP_T1 = 0x0,
    //! T1 event on count up and set output pins to low
    EPWM_AQ_OUTPUT_LOW_UP_T1 = 0x1,
    //! T1 event on count up and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_UP_T1 = 0x2,
    //! T1 event on count up and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_UP_T1 = 0x3,
    //! T1 event on count down and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_T1 = 0x0,
    //! T1 event on count down and set output pins to low
    EPWM_AQ_OUTPUT_LOW_DOWN_T1 = 0x4,
    //! T1 event on count down and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_DOWN_T1 = 0x8,
    //! T1 event on count down and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_DOWN_T1 = 0xC,
    //! T2 event on count up and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_UP_T2 = 0x00,
    //! T2 event on count up and set output pins to low
    EPWM_AQ_OUTPUT_LOW_UP_T2 = 0x10,
    //! T2 event on count up and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_UP_T2 = 0x20,
    //! T2 event on count up and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_UP_T2 = 0x30,
    //! T2 event on count down and no change in the output pins
    EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_T2 = 0x00,
    //! T2 event on count down and set output pins to low
    EPWM_AQ_OUTPUT_LOW_DOWN_T2 = 0x40,
    //! T2 event on count down and set output pins to high
    EPWM_AQ_OUTPUT_HIGH_DOWN_T2 = 0x80,
    //! T2 event on count down and toggle the output pins
    EPWM_AQ_OUTPUT_TOGGLE_DOWN_T2 = 0xC0
} EPWM_AdditionalActionQualifierEventAction;

//*****************************************************************************
//
//! Values that can be passed to EPWM_forceActionQualifierSWAction(),
//! EPWM_setActionQualifierSWAction(), EPWM_setActionQualifierAction()
//! EPWM_setActionQualifierContSWForceAction() as the \e epwmOutput parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_AQ_OUTPUT_A = 0, //!< ePWMxA output
    EPWM_AQ_OUTPUT_B = 2  //!< ePWMxB output
} EPWM_ActionQualifierOutputModule;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setActionQualifierContSWForceShadowMode()
//! as the \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Shadow mode load when counter equals zero
    EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO        = 0,
    //! Shadow mode load when counter equals period
    EPWM_AQ_SW_SH_LOAD_ON_CNTR_PERIOD      = 1,
    //! Shadow mode load when counter equals zero or period
    EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO_PERIOD = 2,
    //! No shadow load mode. Immediate mode only.
    EPWM_AQ_SW_IMMEDIATE_LOAD   = 3
} EPWM_ActionQualifierContForce;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDeadBandOutputSwapMode()
//! as the \e output parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DB_OUTPUT_A = 1, //!< DB output is ePWMA
    EPWM_DB_OUTPUT_B = 0  //!< DB output is ePWMB
} EPWM_DeadBandOutput;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDeadBandDelayPolarity(),
//! EPWM_setDeadBandDelayMode() as the \e delayMode parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DB_RED = 1, //!< DB RED (Rising Edge Delay) mode
    EPWM_DB_FED = 0  //!< DB FED (Falling Edge Delay) mode
} EPWM_DeadBandDelayMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDeadBandDelayPolarity as the
//! \e polarity parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DB_POLARITY_ACTIVE_HIGH = 0, //!< DB polarity is not inverted
    EPWM_DB_POLARITY_ACTIVE_LOW  = 1  //!< DB polarity is inverted
} EPWM_DeadBandPolarity;

//*****************************************************************************
//
// Values that can be passed to EPWM_setRisingEdgeDeadBandDelayInput(),
// EPWM_setFallingEdgeDeadBandDelayInput() as the input parameter.
//
//*****************************************************************************
//! Input signal is ePWMA
//!
#define EPWM_DB_INPUT_EPWMA       0U
//! Input signal is ePWMB
//!
#define EPWM_DB_INPUT_EPWMB       1U
//! Input signal is the output of Rising Edge delay
//!
#define EPWM_DB_INPUT_DB_RED      2U

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDeadBandControlShadowLoadMode() as
//! the \e loadMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Load when counter equals zero
    EPWM_DB_LOAD_ON_CNTR_ZERO        = 0,
    //! Load when counter equals period
    EPWM_DB_LOAD_ON_CNTR_PERIOD      = 1,
    //! Load when counter equals zero or period
    EPWM_DB_LOAD_ON_CNTR_ZERO_PERIOD = 2,
    //! Freeze shadow to active load
    EPWM_DB_LOAD_FREEZE = 3
} EPWM_DeadBandControlLoadMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setRisingEdgeDelayCountShadowLoadMode()
//! as the \e loadMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Load when counter equals zero
    EPWM_RED_LOAD_ON_CNTR_ZERO        = 0,
    //! Load when counter equals period
    EPWM_RED_LOAD_ON_CNTR_PERIOD      = 1,
    //! Load when counter equals zero or period
    EPWM_RED_LOAD_ON_CNTR_ZERO_PERIOD = 2,
    //! Freeze shadow to active load
    EPWM_RED_LOAD_FREEZE = 3
} EPWM_RisingEdgeDelayLoadMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setFallingEdgeDelayCountShadowLoadMode()
//! as the \e loadMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Load when counter equals zero
    EPWM_FED_LOAD_ON_CNTR_ZERO        = 0,
    //! Load when counter equals period
    EPWM_FED_LOAD_ON_CNTR_PERIOD      = 1,
    //! Load when counter equals zero or period
    EPWM_FED_LOAD_ON_CNTR_ZERO_PERIOD = 2,
    //! Freeze shadow to active load
    EPWM_FED_LOAD_FREEZE = 3
} EPWM_FallingEdgeDelayLoadMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDeadBandCounterClock() as the
//! \e clockMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Dead band counter runs at TBCLK rate
    EPWM_DB_COUNTER_CLOCK_FULL_CYCLE = 0,
    //! Dead band counter runs at 2*TBCLK rate
    EPWM_DB_COUNTER_CLOCK_HALF_CYCLE = 1
} EPWM_DeadBandClockMode;

//
// Trip Zone
//
//*****************************************************************************
//
// Values that can be passed to EPWM_enableTripZoneSignals() and
// EPWM_disableTripZoneSignals() as the tzSignal parameter.
//
//*****************************************************************************
//! TZ1 Cycle By Cycle
//!
#define EPWM_TZ_SIGNAL_CBC1          0x1U
//! TZ2 Cycle By Cycle
//!
#define EPWM_TZ_SIGNAL_CBC2          0x2U
//! TZ3 Cycle By Cycle
//!
#define EPWM_TZ_SIGNAL_CBC3          0x4U
//! TZ4 Cycle By Cycle
//!
#define EPWM_TZ_SIGNAL_CBC4          0x8U
//! TZ5 Cycle By Cycle
//!
#define EPWM_TZ_SIGNAL_CBC5          0x10U
//! TZ6 Cycle By Cycle
//!
#define EPWM_TZ_SIGNAL_CBC6          0x20U
//! DCAEVT2 Cycle By Cycle
//!
#define EPWM_TZ_SIGNAL_DCAEVT2       0x40U
//! DCBEVT2 Cycle By Cycle
//!
#define EPWM_TZ_SIGNAL_DCBEVT2       0x80U
//! One-shot TZ1
//!
#define EPWM_TZ_SIGNAL_OSHT1         0x100U
//! One-shot TZ2
//!
#define EPWM_TZ_SIGNAL_OSHT2         0x200U
//! One-shot TZ3
//!
#define EPWM_TZ_SIGNAL_OSHT3         0x400U
//! One-shot TZ4
//!
#define EPWM_TZ_SIGNAL_OSHT4         0x800U
//! One-shot TZ5
//!
#define EPWM_TZ_SIGNAL_OSHT5         0x1000U
//! One-shot TZ6
//!
#define EPWM_TZ_SIGNAL_OSHT6         0x2000U
//! One-shot DCAEVT1
//!
#define EPWM_TZ_SIGNAL_DCAEVT1       0x4000U
//! One-shot DCBEVT1
//!
#define EPWM_TZ_SIGNAL_DCBEVT1       0x8000U

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTripZoneDigitalCompareEventCondition()
//! as the \e dcType parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_TZ_DC_OUTPUT_A1 = 0, //!< Digital Compare output 1 A
    EPWM_TZ_DC_OUTPUT_A2 = 3, //!< Digital Compare output 2 A
    EPWM_TZ_DC_OUTPUT_B1 = 6, //!< Digital Compare output 1 B
    EPWM_TZ_DC_OUTPUT_B2 = 9  //!< Digital Compare output 2 B
} EPWM_TripZoneDigitalCompareOutput;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTripZoneDigitalCompareEventCondition()
//! as the \e dcEvent parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_TZ_EVENT_DC_DISABLED = 0,       //!< Event is disabled
    EPWM_TZ_EVENT_DCXH_LOW    = 1,       //!< Event when DCxH low
    EPWM_TZ_EVENT_DCXH_HIGH   = 2,       //!< Event when DCxH high
    EPWM_TZ_EVENT_DCXL_LOW    = 3,       //!< Event when DCxL low
    EPWM_TZ_EVENT_DCXL_HIGH   = 4,       //!< Event when DCxL high
    EPWM_TZ_EVENT_DCXL_HIGH_DCXH_LOW = 5 //!< Event when DCxL high DCxH low
} EPWM_TripZoneDigitalCompareOutputEvent;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTripZoneAction() as the \e tzEvent
//! parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_TZ_ACTION_EVENT_TZA = 0,     //!< TZ1 - TZ6, DCAEVT2, DCAEVT1
    EPWM_TZ_ACTION_EVENT_TZB = 2,     //!< TZ1 - TZ6, DCBEVT2, DCBEVT1
    EPWM_TZ_ACTION_EVENT_DCAEVT1 = 4, //!< DCAEVT1 (Digital Compare A event 1)
    EPWM_TZ_ACTION_EVENT_DCAEVT2 = 6, //!< DCAEVT2 (Digital Compare A event 2)
    EPWM_TZ_ACTION_EVENT_DCBEVT1 = 8, //!< DCBEVT1 (Digital Compare B event 1)
    EPWM_TZ_ACTION_EVENT_DCBEVT2 = 10 //!< DCBEVT2 (Digital Compare B event 2)
} EPWM_TripZoneEvent;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTripZoneAction() as the
//! \e tzAction parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_TZ_ACTION_HIGH_Z  = 0, //!< High impedance output
    EPWM_TZ_ACTION_HIGH    = 1, //!< High voltage state
    EPWM_TZ_ACTION_LOW     = 2, //!< Low voltage state
    EPWM_TZ_ACTION_DISABLE = 3  //!< Disable action
} EPWM_TripZoneAction;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTripZoneAdvAction() as the
//! \e tzAdvEvent parameter.
//
//*****************************************************************************
typedef enum
{
    //! TZ1 - TZ6, DCBEVT2, DCBEVT1 while counting down
    EPWM_TZ_ADV_ACTION_EVENT_TZB_D = 9,
    //! TZ1 - TZ6, DCBEVT2, DCBEVT1 while counting up
    EPWM_TZ_ADV_ACTION_EVENT_TZB_U = 6,
    //! TZ1 - TZ6, DCAEVT2, DCAEVT1 while counting down
    EPWM_TZ_ADV_ACTION_EVENT_TZA_D = 3,
    //! TZ1 - TZ6, DCAEVT2, DCAEVT1 while counting up
    EPWM_TZ_ADV_ACTION_EVENT_TZA_U = 0
} EPWM_TripZoneAdvancedEvent;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTripZoneAdvDigitalCompareActionA(),
//! EPWM_setTripZoneAdvDigitalCompareActionB(),EPWM_setTripZoneAdvAction()
//! as the \e tzAdvDCAction parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_TZ_ADV_ACTION_HIGH_Z  = 0, //!< High impedance output
    EPWM_TZ_ADV_ACTION_HIGH    = 1, //!< High voltage state
    EPWM_TZ_ADV_ACTION_LOW     = 2, //!< Low voltage state
    EPWM_TZ_ADV_ACTION_TOGGLE  = 3, //!< Toggle the output
    EPWM_TZ_ADV_ACTION_DISABLE = 7  //!< Disable action
} EPWM_TripZoneAdvancedAction;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setTripZoneAdvDigitalCompareActionA() and
//! EPWM_setTripZoneAdvDigitalCompareActionB() as the \e tzAdvDCEvent
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! Digital Compare event A/B 1 while counting up
    EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U = 0,
    //! Digital Compare event A/B 1 while counting down
    EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D = 3,
    //! Digital Compare event A/B 2 while counting up
    EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U = 6,
    //! Digital Compare event A/B 2 while counting down
    EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D =  9
} EPWM_TripZoneAdvDigitalCompareEvent;


//*****************************************************************************
//
// Values that can be passed to EPWM_enableTripZoneInterrupt()and
// EPWM_disableTripZoneInterrupt() as the tzInterrupt parameter .
//
//*****************************************************************************
//! Trip Zones Cycle By Cycle interrupt
//!
#define EPWM_TZ_INTERRUPT_CBC      0x2U
//! Trip Zones One Shot interrupt
//!
#define EPWM_TZ_INTERRUPT_OST      0x4U
//! Digital Compare A Event 1 interrupt
//!
#define EPWM_TZ_INTERRUPT_DCAEVT1  0x8U
//! Digital Compare A Event 2 interrupt
//!
#define EPWM_TZ_INTERRUPT_DCAEVT2  0x10U
//! Digital Compare B Event 1 interrupt
//!
#define EPWM_TZ_INTERRUPT_DCBEVT1  0x20U
//! Digital Compare B Event 2 interrupt
//!
#define EPWM_TZ_INTERRUPT_DCBEVT2  0x40U

//*****************************************************************************
//
// Values that can be returned by EPWM_getTripZoneFlagStatus() .
//
//*****************************************************************************
//! Trip Zones Cycle By Cycle flag
//!
#define EPWM_TZ_FLAG_CBC      0x2U
//! Trip Zones One Shot flag
//!
#define EPWM_TZ_FLAG_OST      0x4U
//! Digital Compare A Event 1 flag
//!
#define EPWM_TZ_FLAG_DCAEVT1  0x8U
//! Digital Compare A Event 2 flag
//!
#define EPWM_TZ_FLAG_DCAEVT2  0x10U
//! Digital Compare B Event 1 flag
//!
#define EPWM_TZ_FLAG_DCBEVT1  0x20U
//! Digital Compare B Event 2 flag
//!
#define EPWM_TZ_FLAG_DCBEVT2  0x40U

//*****************************************************************************
//
// Value can be passed to EPWM_clearTripZoneFlag() as the
// tzInterrupt parameter and returned by EPWM_getTripZoneFlagStatus().
//
//*****************************************************************************
//! Trip Zone interrupt
//!
#define EPWM_TZ_INTERRUPT     0x1U

//*****************************************************************************
//
// Values that can be passed to EPWM_clearCycleByCycleTripZoneFlag()
// as the tzCbcFlag parameter and returned by
// EPWM_getCycleByCycleTripZoneFlagStatus().
//
//*****************************************************************************
//! CBC flag 1
//!
#define EPWM_TZ_CBC_FLAG_1  0x1U
//! CBC flag 2
//!
#define EPWM_TZ_CBC_FLAG_2  0x2U
//! CBC flag 3
//!
#define EPWM_TZ_CBC_FLAG_3  0x4U
//! CBC flag 4
//!
#define EPWM_TZ_CBC_FLAG_4  0x8U
//! CBC flag 5
//!
#define EPWM_TZ_CBC_FLAG_5  0x10U
//! CBC flag 6
//!
#define EPWM_TZ_CBC_FLAG_6   0x20U
//! CBC flag Digital compare event A2
//!
#define EPWM_TZ_CBC_FLAG_DCAEVT2  0x40U
//! CBC flag Digital compare event B2
//!
#define EPWM_TZ_CBC_FLAG_DCBEVT2  0x80U

//*****************************************************************************
//
// Values that can be passed to EPWM_clearOneShotTripZoneFlag() as
// the tzCbcFlag parameter and returned by the
// EPWM_getOneShotTripZoneFlagStatus() .
//
//*****************************************************************************
//! OST flag OST1
//!
#define EPWM_TZ_OST_FLAG_OST1  0x1U
//! OST flag OST2
//!
#define EPWM_TZ_OST_FLAG_OST2  0x2U
//! OST flag OST3
//!
#define EPWM_TZ_OST_FLAG_OST3  0x4U
//! OST flag OST4
//!
#define EPWM_TZ_OST_FLAG_OST4  0x8U
//! OST flag OST5
//!
#define EPWM_TZ_OST_FLAG_OST5  0x10U
//! OST flag OST6
//!
#define EPWM_TZ_OST_FLAG_OST6  0x20U
//! OST flag Digital compare event A1
//!
#define EPWM_TZ_OST_FLAG_DCAEVT1   0x40U
//! OST flag Digital compare event B1
//!
#define EPWM_TZ_OST_FLAG_DCBEVT1   0x80U

//*****************************************************************************
//
//! Values that can be passed to EPWM_selectCycleByCycleTripZoneClearEvent() as
//! the \e clearMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Clear CBC pulse when counter equals zero
    EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO = 0,
    //! Clear CBC pulse when counter equals period
    EPWM_TZ_CBC_PULSE_CLR_CNTR_PERIOD = 1,
    //! Clear CBC pulse when counter equals zero or period
    EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO_PERIOD = 2
} EPWM_CycleByCycleTripZoneClearMode;

//*****************************************************************************
//
// Values that can be passed to EPWM_forceTripZoneEvent() as the
// tzForceEvent parameter.
//
//*****************************************************************************
//! Force Cycle By Cycle trip event
//!
#define EPWM_TZ_FORCE_EVENT_CBC  0x2U
//! Force a One-Shot Trip Event
//!
#define EPWM_TZ_FORCE_EVENT_OST  0x4U
//! Force Digital Compare Output A Event 1
//!
#define EPWM_TZ_FORCE_EVENT_DCAEVT1  0x8U
//! Force Digital Compare Output A Event 2
//!
#define EPWM_TZ_FORCE_EVENT_DCAEVT2  0x10U
//! Force Digital Compare Output B Event 1
//!
#define EPWM_TZ_FORCE_EVENT_DCBEVT1  0x20U
//! Force Digital Compare Output B Event 2
//!
#define EPWM_TZ_FORCE_EVENT_DCBEVT2  0x40U

//*****************************************************************************
//
// Values that can be passed to EPWM_setInterruptSource() as the
// interruptSource parameter.
//
//*****************************************************************************
//! Time-base counter is disabled
//!
#define EPWM_INT_TBCTR_DISABLED 0U
//! Time-base counter equal to zero
//!
#define EPWM_INT_TBCTR_ZERO  1U
//! Time-base counter equal to period
//!
#define EPWM_INT_TBCTR_PERIOD  2U
//! Time-base counter equal to zero or period
//!
#define EPWM_INT_TBCTR_ZERO_OR_PERIOD  3U
//! time-base counter equal to CMPA when the timer is incrementing
//!
#define EPWM_INT_TBCTR_U_CMPA  4U
//! time-base counter equal to CMPC when the timer is incrementing
//!
#define EPWM_INT_TBCTR_U_CMPC  8U
//! time-base counter equal to CMPA when the timer is decrementing
//!
#define EPWM_INT_TBCTR_D_CMPA  5U
//! time-base counter equal to CMPC when the timer is decrementing
//!
#define EPWM_INT_TBCTR_D_CMPC  10U
//! time-base counter equal to CMPB when the timer is incrementing
//!
#define EPWM_INT_TBCTR_U_CMPB  6U
//! time-base counter equal to CMPD when the timer is incrementing
//!
#define EPWM_INT_TBCTR_U_CMPD  12U
//! time-base counter equal to CMPB when the timer is decrementing
//!
#define EPWM_INT_TBCTR_D_CMPB  7U
//! time-base counter equal to CMPD when the timer is decrementing
//!
#define EPWM_INT_TBCTR_D_CMPD  14U

//*****************************************************************************
//
//! Values that can be passed to EPWM_enableADCTrigger(),
//! EPWM_disableADCTrigger(),EPWM_setADCTriggerSource(),
//! EPWM_setADCTriggerEventPrescale(),EPWM_getADCTriggerFlagStatus(),
//! EPWM_clearADCTriggerFlag(),EPWM_enableADCTriggerEventCountInit(),
//! EPWM_disableADCTriggerEventCountInit(),EPWM_forceADCTriggerEventCountInit(),
//! EPWM_setADCTriggerEventCountInitValue(),EPWM_getADCTriggerEventCount(),
//! EPWM_forceADCTrigger() as the \e adcSOCType parameter
//
//*****************************************************************************
typedef enum
{
    EPWM_SOC_A = 0,  //!< SOC A
    EPWM_SOC_B = 1   //!< SOC B
} EPWM_ADCStartOfConversionType;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setADCTriggerSource() as the
//! \e socSource parameter.
//
//*****************************************************************************
typedef enum
{
    //! Event is based on DCxEVT1
    EPWM_SOC_DCxEVT1 = 0,
    //! Time-base counter equal to zero
    EPWM_SOC_TBCTR_ZERO = 1,
    //! Time-base counter equal to period
    EPWM_SOC_TBCTR_PERIOD = 2,
    //! Time-base counter equal to zero or period
    EPWM_SOC_TBCTR_ZERO_OR_PERIOD = 3,
    //! Time-base counter equal to CMPA when the timer is incrementing
    EPWM_SOC_TBCTR_U_CMPA = 4,
    //! Time-base counter equal to CMPC when the timer is incrementing
    EPWM_SOC_TBCTR_U_CMPC = 8,
    //! Time-base counter equal to CMPA when the timer is decrementing
    EPWM_SOC_TBCTR_D_CMPA = 5,
    //! Time-base counter equal to CMPC when the timer is decrementing
    EPWM_SOC_TBCTR_D_CMPC = 10,
    //! Time-base counter equal to CMPB when the timer is incrementing
    EPWM_SOC_TBCTR_U_CMPB = 6,
    //! Time-base counter equal to CMPD when the timer is incrementing
    EPWM_SOC_TBCTR_U_CMPD = 12,
    //! Time-base counter equal to CMPB when the timer is decrementing
    EPWM_SOC_TBCTR_D_CMPB = 7,
    //! Time-base counter equal to CMPD when the timer is decrementing
    EPWM_SOC_TBCTR_D_CMPD = 14
} EPWM_ADCStartOfConversionSource;

//
// Digital Compare Module
//
//*****************************************************************************
//
//! Values that can be passed to EPWM_selectDigitalCompareTripInput(),
//! EPWM_enableDigitalCompareTripCombinationInput(),
//! EPWM_disableDigitalCompareTripCombinationInput() as the \e dcType
//! parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DC_TYPE_DCAH = 0,  //!< Digital Compare A High
    EPWM_DC_TYPE_DCAL = 1,  //!< Digital Compare A Low
    EPWM_DC_TYPE_DCBH = 2,  //!< Digital Compare B High
    EPWM_DC_TYPE_DCBL = 3   //!< Digital Compare B Low
} EPWM_DigitalCompareType;

//*****************************************************************************
//
//! Values that can be passed to EPWM_selectDigitalCompareTripInput()
//! as the \e tripSource parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DC_TRIP_TRIPIN1 = 0,   //!< Trip 1
    EPWM_DC_TRIP_TRIPIN2 = 1,   //!< Trip 2
    EPWM_DC_TRIP_TRIPIN3 = 2,   //!< Trip 3
    EPWM_DC_TRIP_TRIPIN4 = 3,   //!< Trip 4
    EPWM_DC_TRIP_TRIPIN5 = 4,   //!< Trip 5
    EPWM_DC_TRIP_TRIPIN6 = 5,   //!< Trip 6
    EPWM_DC_TRIP_TRIPIN7 = 6,   //!< Trip 7
    EPWM_DC_TRIP_TRIPIN8 = 7,   //!< Trip 8
    EPWM_DC_TRIP_TRIPIN9 = 8,   //!< Trip 9
    EPWM_DC_TRIP_TRIPIN10 = 9,  //!< Trip 10
    EPWM_DC_TRIP_TRIPIN11 = 10, //!< Trip 11
    EPWM_DC_TRIP_TRIPIN12 = 11, //!< Trip 12
    EPWM_DC_TRIP_TRIPIN14 = 13, //!< Trip 14
    EPWM_DC_TRIP_TRIPIN15 = 14, //!< Trip 15
    EPWM_DC_TRIP_COMBINATION = 15 //!< All Trips (Trip1 - Trip 15) are selected
} EPWM_DigitalCompareTripInput;

//*****************************************************************************
//
// Values that can be passed to EPWM_enableDigitalCompareTripCombinationInput(),
// EPWM_disableDigitalCompareTripCombinationInput() as the tripInput
// parameter.
//
//*****************************************************************************
//! Combinational Trip 1 input
//!
#define EPWM_DC_COMBINATIONAL_TRIPIN1   0x1U
//! Combinational Trip 2 input
//!
#define EPWM_DC_COMBINATIONAL_TRIPIN2   0x2U
//! Combinational Trip 3 input
//!
#define EPWM_DC_COMBINATIONAL_TRIPIN3   0x4U
//! Combinational Trip 4 input
//!
#define EPWM_DC_COMBINATIONAL_TRIPIN4   0x8U
//! Combinational Trip 5 input
//!
#define EPWM_DC_COMBINATIONAL_TRIPIN5   0x10U
//! Combinational Trip 6 input
//!
#define EPWM_DC_COMBINATIONAL_TRIPIN6   0x20U
//! Combinational Trip 7 input
//!
#define EPWM_DC_COMBINATIONAL_TRIPIN7   0x40U
//! Combinational Trip 8 input
//!
#define EPWM_DC_COMBINATIONAL_TRIPIN8   0x80U
//! Combinational Trip 9 input
//!
#define EPWM_DC_COMBINATIONAL_TRIPIN9   0x100U
//! Combinational Trip 10 input
//!
#define EPWM_DC_COMBINATIONAL_TRIPIN10  0x200U
//! Combinational Trip 11 input
//!
#define EPWM_DC_COMBINATIONAL_TRIPIN11  0x400U
//! Combinational Trip 12 input
//!
#define EPWM_DC_COMBINATIONAL_TRIPIN12  0x800U
//! Combinational Trip 14 input
//!
#define EPWM_DC_COMBINATIONAL_TRIPIN14  0x2000U
//! Combinational Trip 15 input
//!
#define EPWM_DC_COMBINATIONAL_TRIPIN15  0x4000U

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareBlankingEvent() as the
//! the \e blankingPulse parameter.
//
//*****************************************************************************
typedef enum
{
    //! Time base counter equals period
    EPWM_DC_WINDOW_START_TBCTR_PERIOD = 0,
    //! Time base counter equals zero
    EPWM_DC_WINDOW_START_TBCTR_ZERO   = 1,
    //! Time base counter equals zero or period
    EPWM_DC_WINDOW_START_TBCTR_ZERO_PERIOD  = 2
} EPWM_DigitalCompareBlankingPulse;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareFilterInput()
//! as the \e filterInput parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DC_WINDOW_SOURCE_DCAEVT1 = 0, //!< DC filter signal source is DCAEVT1
    EPWM_DC_WINDOW_SOURCE_DCAEVT2 = 1, //!< DC filter signal source is DCAEVT2
    EPWM_DC_WINDOW_SOURCE_DCBEVT1 = 2, //!< DC filter signal source is DCBEVT1
    EPWM_DC_WINDOW_SOURCE_DCBEVT2 = 3  //!< DC filter signal source is DCBEVT2
} EPWM_DigitalCompareFilterInput;

//*****************************************************************************
//
//! Values that can be assigned to EPWM_setDigitalCompareEventSource(),
//! EPWM_setDigitalCompareEventSyncMode(),EPWM_enableDigitalCompareSyncEvent()
//! EPWM_enableDigitalCompareADCTrigger(),EPWM_disableDigitalCompareSyncEvent()
//! EPWM_disableDigitalCompareADCTrigger() as the \e dcModule parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DC_MODULE_A = 0, //!< Digital Compare Module A
    EPWM_DC_MODULE_B = 1  //!< Digital Compare Module B
} EPWM_DigitalCompareModule;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareEventSource(),
//! EPWM_setDigitalCompareEventSyncMode as the \e dcEvent parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DC_EVENT_1 = 0, //!< Digital Compare Event number 1
    EPWM_DC_EVENT_2 = 1  //!< Digital Compare Event number 2
} EPWM_DigitalCompareEvent;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareEventSource() as the
//! \e dcEventSource parameter.
//
//*****************************************************************************
typedef enum
{
    //! Signal source is unfiltered (DCAEVT1/2)
    EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL = 0,
    //! Signal source is filtered (DCEVTFILT)
    EPWM_DC_EVENT_SOURCE_FILT_SIGNAL = 1
} EPWM_DigitalCompareEventSource;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareEventSyncMode() as the
//! \e syncMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! DC input signal is synced with TBCLK
    EPWM_DC_EVENT_INPUT_SYNCED = 0,
    //! DC input signal is not synced with TBCLK
    EPWM_DC_EVENT_INPUT_NOT_SYNCED = 1
} EPWM_DigitalCompareSyncMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareCBCLatchMode() as the
//! \e latchMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! DC cycle-by-cycle(CBC) latch is disabled
    EPWM_DC_CBC_LATCH_DISABLED = 0x0,
    //! DC cycle-by-cycle(CBC) latch is enabled
    EPWM_DC_CBC_LATCH_ENABLED  = 0x1
} EPWM_DigitalCompareCBCLatchMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_selectDigitalCompareCBCLatchClearEvent()
//! as the \e latchMode parameter.
//
//*****************************************************************************
typedef enum
{
    //! Clear CBC latch when counter equals zero
    EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO           = 0x0,
    //! Clear CBC latch when counter equals period
    EPWM_DC_CBC_LATCH_CLR_ON_CNTR_PERIOD      = 0x1,
    //! Clear CBC latch when counter equals zero or period
    EPWM_DC_CBC_LATCH_CLR_ON_CNTR_ZERO_PERIOD = 0x2
} EPWM_DigitalCompareCBCLatchClearEvent;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setGlobalLoadTrigger() as the
//! \e loadTrigger parameter.
//
//*****************************************************************************
typedef enum
{
    //! Load when counter is equal to zero
    EPWM_GL_LOAD_PULSE_CNTR_ZERO = 0x0,
    //! Load when counter is equal to period
    EPWM_GL_LOAD_PULSE_CNTR_PERIOD = 0x1,
    //! Load when counter is equal to zero or period
    EPWM_GL_LOAD_PULSE_CNTR_ZERO_PERIOD = 0x2,
    //! Load on sync event
    EPWM_GL_LOAD_PULSE_SYNC = 0x3,
    //! Load on sync event or when counter  is equal to zero
    EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_ZERO = 0x4,
    //! Load on sync event or when counter  is equal to period
    EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_PERIOD = 0x5,
    //! Load on sync event or when counter is equal to period or zero
    EPWM_GL_LOAD_PULSE_SYNC_CNTR_ZERO_PERIOD = 0x6,
    //! Load on global force
    EPWM_GL_LOAD_PULSE_GLOBAL_FORCE = 0xF
} EPWM_GlobalLoadTrigger;

//*****************************************************************************
//
// Values that can be passed to EPWM_enableGlobalLoadRegisters(),
// EPWM_disableGlobalLoadRegisters() as theloadRegister parameter.
//
//*****************************************************************************
//! Global load TBPRD:TBPRDHR
//!
#define EPWM_GL_REGISTER_TBPRD_TBPRDHR   0x1U
//! Global load CMPA:CMPAHR
//!
#define EPWM_GL_REGISTER_CMPA_CMPAHR     0x2U
//! Global load CMPB:CMPBHR
//!
#define EPWM_GL_REGISTER_CMPB_CMPBHR     0x4U
//! Global load CMPC
//!
#define EPWM_GL_REGISTER_CMPC            0x8U
//! Global load CMPD
//!
#define EPWM_GL_REGISTER_CMPD            0x10U
//! Global load DBRED:DBREDHR
//!
#define EPWM_GL_REGISTER_DBRED_DBREDHR   0x20U
//! Global load DBFED:DBFEDHR
//!
#define EPWM_GL_REGISTER_DBFED_DBFEDHR   0x40U
//! Global load DBCTL
//!
#define EPWM_GL_REGISTER_DBCTL           0x80U
//! Global load AQCTLA/A2
//!
#define EPWM_GL_REGISTER_AQCTLA_AQCTLA2  0x100U
//! Global load AQCTLB/B2
//!
#define EPWM_GL_REGISTER_AQCTLB_AQCTLB2  0x200U
//! Global load AQCSFRC
//!
#define EPWM_GL_REGISTER_AQCSFRC         0x400U

//*****************************************************************************
//
//! Values that can be passed to EPWM_setValleyTriggerSource() as the \e
//! trigger parameter.
//
//*****************************************************************************
typedef enum
{
    //! Valley capture trigged by software
    EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE = 0U,
    //! Valley capture trigged by when counter is equal to zero
    EPWM_VALLEY_TRIGGER_EVENT_CNTR_ZERO = 1U,
    //! Valley capture trigged by when counter is equal period
    EPWM_VALLEY_TRIGGER_EVENT_CNTR_PERIOD = 2U,
    //! Valley capture trigged when counter is equal to zero or period
    EPWM_VALLEY_TRIGGER_EVENT_CNTR_ZERO_PERIOD = 3U,
    //! Valley capture trigged by DCAEVT1 (Digital Compare A event 1)
    EPWM_VALLEY_TRIGGER_EVENT_DCAEVT1 = 4U,
    //! Valley capture trigged by DCAEVT2 (Digital Compare A event 2)
    EPWM_VALLEY_TRIGGER_EVENT_DCAEVT2 = 5U,
    //! Valley capture trigged by DCBEVT1 (Digital Compare B event 1)
    EPWM_VALLEY_TRIGGER_EVENT_DCBEVT1 = 6U,
    //! Valley capture trigged by DCBEVT2 (Digital Compare B event 2)
    EPWM_VALLEY_TRIGGER_EVENT_DCBEVT2 = 7U
} EPWM_ValleyTriggerSource;

//*****************************************************************************
//
//! Values that can be passed to EPWM_getValleyCountEdgeStatus() as the \e edge
//! parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_VALLEY_COUNT_START_EDGE = 0, //!< Valley count start edge
    EPWM_VALLEY_COUNT_STOP_EDGE  = 1  //!< Valley count stop edge
} EPWM_ValleyCounterEdge;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setValleyDelayValue() as the \e delayMode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! Delay value equals the offset value defines by software
    EPWM_VALLEY_DELAY_MODE_SW_DELAY   = 0U,
    //! Delay value equals the sum of the Hardware counter value and the offset
    //! value defines by software
    EPWM_VALLEY_DELAY_MODE_VCNT_DELAY_SW_DELAY = 1U,
    //! Delay value equals the the Hardware counter shifted by
    //! (1 + the offset value defines by software)
    EPWM_VALLEY_DELAY_MODE_VCNT_DELAY_SHIFT_1_SW_DELAY = 2U,
    //! Delay value equals the the Hardware counter shifted by
    //! (2 + the offset value defines by software)
    EPWM_VALLEY_DELAY_MODE_VCNT_DELAY_SHIFT_2_SW_DELAY = 3U,
    //! Delay value equals the the Hardware counter shifted by
    //! (4 + the offset value defines by software)
    EPWM_VALLEY_DELAY_MODE_VCNT_DELAY_SHIFT_4_SW_DELAY = 4U
} EPWM_ValleyDelayMode;

//
// DC Edge Filter
//
//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareEdgeFilterMode()
//! as the \e edgeMode parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DC_EDGEFILT_MODE_RISING  = 0, //!< Digital Compare Edge filter low
                                       //!< to high edge mode
    EPWM_DC_EDGEFILT_MODE_FALLING = 1, //!< Digital Compare Edge filter high
                                       //!< to low edge mode
    EPWM_DC_EDGEFILT_MODE_BOTH    = 2  //!< Digital Compare Edge filter both
                                       //!< edges mode
} EPWM_DigitalCompareEdgeFilterMode;

//*****************************************************************************
//
//! Values that can be passed to EPWM_setDigitalCompareEdgeFilterEdgeCount()
//! as the \e edgeCount parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_DC_EDGEFILT_EDGECNT_0  = 0, //!< Digital Compare Edge filter edge
                                     //!< count = 0
    EPWM_DC_EDGEFILT_EDGECNT_1  = 1, //!< Digital Compare Edge filter edge
                                     //!< count = 1
    EPWM_DC_EDGEFILT_EDGECNT_2  = 2, //!< Digital Compare Edge filter edge
                                     //!< count = 2
    EPWM_DC_EDGEFILT_EDGECNT_3  = 3, //!< Digital Compare Edge filter edge
                                     //!< count = 3
    EPWM_DC_EDGEFILT_EDGECNT_4  = 4, //!< Digital Compare Edge filter edge
                                     //!< count = 4
    EPWM_DC_EDGEFILT_EDGECNT_5  = 5, //!< Digital Compare Edge filter edge
                                     //!< count = 5
    EPWM_DC_EDGEFILT_EDGECNT_6  = 6, //!< Digital Compare Edge filter edge
                                     //!< count = 6
    EPWM_DC_EDGEFILT_EDGECNT_7  = 7  //!< Digital Compare Edge filter edge
                                     //!< count = 7
} EPWM_DigitalCompareEdgeFilterEdgeCount;

//*****************************************************************************
//
//! Values that can be passed to EPWM_lockRegisters() as the \e registerGroup
//! parameter.
//
//*****************************************************************************
typedef enum
{
    EPWM_REGISTER_GROUP_GLOBAL_LOAD = 0x2,     //!< Global load register group
    EPWM_REGISTER_GROUP_TRIP_ZONE = 0x4,       //!< Trip zone register group
    EPWM_REGISTER_GROUP_TRIP_ZONE_CLEAR = 0x8, //!< Trip zone clear group
    EPWM_REGISTER_GROUP_DIGITAL_COMPARE = 0x10 //!< Digital compare group
} EPWM_LockRegisterGroup;

//*****************************************************************************
//
//! Values that can be passed to EPWM_configureSignal() as the
//! \e signalParams parameter.
//
//*****************************************************************************
typedef struct
{
    float32_t              freqInHz;    //!< Desired Signal Frequency(in Hz)
    float32_t              dutyValA;    //!< Desired ePWMxA Signal Duty
    float32_t              dutyValB;    //!< Desired ePWMxB Signal Duty
    bool                   invertSignalB; //!< Invert ePWMxB Signal if true
    float32_t              sysClkInHz;  //!< SYSCLK Frequency(in Hz)
    SysCtl_EPWMCLKDivider  epwmClkDiv;  //!< EPWM Clock Divider
    EPWM_TimeBaseCountMode tbCtrMode;   //!< Time Base Counter Mode
    EPWM_ClockDivider      tbClkDiv;    //!< Time Base Counter Clock Divider
    EPWM_HSClockDivider    tbHSClkDiv;  //!< Time Base Counter HS Clock Divider
} EPWM_SignalParams;

//*****************************************************************************
//
// Functions APIs shared with HRPWM module
//
//*****************************************************************************

//
// Period Control related API
//
#define EPWM_setSyncPulseSource                 HRPWM_setSyncPulseSource

//*****************************************************************************
//
// Prototypes for the API.
//
//*****************************************************************************

//*****************************************************************************
//
//! \internal
//! Checks ePWM base address.
//!
//! \param base specifies the ePWM module base address.
//!
//! This function determines if an ePWM module base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool EPWM_isBaseValid(uint32_t base)
{
    return(
           (base == EPWM1_BASE) ||
           (base == EPWM2_BASE) ||
           (base == EPWM3_BASE) ||
           (base == EPWM4_BASE) ||
           (base == EPWM5_BASE) ||
           (base == EPWM6_BASE) ||
           (base == EPWM7_BASE) ||
           (base == EPWM8_BASE) ||
           (base == EPWM9_BASE) ||
           (base == EPWM10_BASE) ||
           (base == EPWM11_BASE) ||
           (base == EPWM12_BASE) ||
           (base == EPWM13_BASE) ||
           (base == EPWM14_BASE) ||
           (base == EPWM15_BASE) ||
           (base == EPWM16_BASE)
          );
}
#endif

//
// Time Base Sub Module related APIs
//
//*****************************************************************************
//
//! Set the time base count
//!
//! \param base is the base address of the EPWM module.
//! \param count is the time base count value.
//!
//! This function sets the 16 bit counter value of the time base counter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setTimeBaseCounter(uint32_t base, uint16_t count)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Write to TBCTR register
    //
    HWREGH(base + EPWM_O_TBCTR) = count;
}

//*****************************************************************************
//
//! Set count mode after phase shift sync
//!
//! \param base is the base address of the EPWM module.
//! \param mode is the count mode.
//!
//! This function sets the time base count to count up or down after a new
//! phase value set by the EPWM_setPhaseShift(). The count direction is
//! determined by the variable mode. Valid inputs for mode are:
//!  - EPWM_COUNT_MODE_UP_AFTER_SYNC      - Count up after sync
//!  - EPWM_COUNT_MODE_DOWN_AFTER_SYNC    - Count down after sync
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setCountModeAfterSync(uint32_t base, EPWM_SyncCountMode mode)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    if(mode == EPWM_COUNT_MODE_UP_AFTER_SYNC)
    {
        //
        // Set PHSDIR bit
        //
        HWREGH(base + EPWM_O_TBCTL) |= EPWM_TBCTL_PHSDIR;
    }
    else
    {
        //
        // Clear PHSDIR bit
        //
        HWREGH(base + EPWM_O_TBCTL) &= ~EPWM_TBCTL_PHSDIR;
    }
}

//*****************************************************************************
//
//! Set the time base clock and the high speed time base clock count pre-scaler
//!
//! \param base is the base address of the EPWM module.
//! \param prescaler is the time base count pre scale value.
//! \param highSpeedPrescaler is the high speed time base count pre scale
//!        value.
//!
//! This function sets the pre scaler(divider)value for the time base clock
//! counter and the high speed time base clock counter.
//! Valid values for pre-scaler and highSpeedPrescaler are EPWM_CLOCK_DIVIDER_X,
//! where X is 1,2,4,8,16, 32,64 or 128.
//! The actual numerical values for these macros represent values 0,1...7.
//! The equation for the output clock is:
//!   TBCLK = EPWMCLK/(highSpeedPrescaler * pre-scaler)
//!
//! \b Note: EPWMCLK is a scaled version of SYSCLK. At reset EPWMCLK is half
//!          SYSCLK.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setClockPrescaler(uint32_t base, EPWM_ClockDivider prescaler,
                       EPWM_HSClockDivider highSpeedPrescaler)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Write to CLKDIV and HSPCLKDIV bit
    //
    HWREGH(base + EPWM_O_TBCTL) =
                ((HWREGH(base + EPWM_O_TBCTL)  &
                ~(EPWM_TBCTL_CLKDIV_M | EPWM_TBCTL_HSPCLKDIV_M)) |
                (((uint16_t)prescaler << EPWM_TBCTL_CLKDIV_S) |
                ((uint16_t)highSpeedPrescaler << EPWM_TBCTL_HSPCLKDIV_S)));
}

//*****************************************************************************
//
//! Force a software sync pulse
//!
//! \param base is the base address of the EPWM module.
//!
//! This function causes a single software initiated sync pulse. Make sure the
//! appropriate mode is selected using EPWM_setupSyncOutputMode() before using
//! this function.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_forceSyncPulse(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set SWFSYNC bit
    //
    HWREGH(base + EPWM_O_TBCTL) |= EPWM_TBCTL_SWFSYNC;
}

//*****************************************************************************
//
//! Set up the source for sync-in pulse.
//!
//! \param base is the base address of the EPWM module.
//! \param source is the sync-in pulse source.
//!
//! This function set the sync out pulse mode.
//! Valid values for mode are:
//!  - EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1-16 - sync-in pulse source can be
//!                                              any of the EPWM1-16 sync-out
//!                                              signal
//!  - EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP1-7 - sync-in pulse source can be
//!                                             selected as any of the ECAP1-16
//!                                             sync-out signal
//!  - EPWM_SYNC_IN_PULSE_SRC_INPUTXBAR_OUT5-6 - sync-in pulse source can be
//!                                              selected as any of the Input
//!                                              xbar out5-6 signal
//!  - EPWM_SYNC_IN_PULSE_SRC_ETHERCAT_SYNC0-1 - sync-in pulse source can be
//!                                              selected as any of the Ethercat
//!                                              sync0-1 signal
//!  - EPWM_SYNC_IN_PULSE_SRC_DISABLE - sync-in pulse is disabled for the
//!                                     EPWM module
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setSyncInPulseSource(uint32_t base, EPWM_SyncInPulseSource source)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set EPWM Sync-In Source Mode.
    //
    HWREGH(base + EPWM_O_SYNCINSEL) =
            (HWREGH(base + EPWM_O_SYNCINSEL) & (~EPWM_SYNCINSEL_SEL_M)) |
            ((uint16_t)source & EPWM_SYNCINSEL_SEL_M);
}

//*****************************************************************************
//
//! Enables sync-out pulse source.
//!
//! \param base is the base address of the EPWM module.
//! \param source is the sync-out pulse source.
//!
//! This function enables the sync-out pulse source.
//! Below valid values for param \b source can be OR'd together to enable
//! multiple sync-out sources:
//! - EPWM_SYNC_OUT_PULSE_ON_SOFTWARE - sync-out pulse is generated by
//!                                     software when EPWM_forceSyncPulse()
//!                                     function is called or by EPWMxSYNCI
//!                                     signal.
//! - EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO - sync-out pulse is generated when
//!                                      time base counter equals zero.
//! - EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_B - sync-out pulse is generated when
//!                                           time base counter equals compare
//!                                           B value.
//! - EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_C - sync-out pulse is generated when
//!                                           time base counter equals compare
//!                                           C value.
//! - EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_D - sync-out pulse is generated when
//!                                           time base counter equals compare
//!                                           D value.
//! - EPWM_SYNC_OUT_PULSE_ON_DCA_EVT1_SYNC - sync-out pulse is generated by DCA
//!                                          event1 sync signal
//! - EPWM_SYNC_OUT_PULSE_ON_DCB_EVT1_SYNC - sync-out pulse is generated by DCB
//!                                          event1 sync signal
//! - EPWM_SYNC_OUT_PULSE_ON_ALL  - sync-out pulse is generated by all
//!                                 the above sources
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableSyncOutPulseSource(uint32_t base, uint16_t source)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(source <= EPWM_SYNC_OUT_SOURCE_M);

    //
    // Enable selected EPWM Sync-Out Sources.
    //
    HWREGH(base + EPWM_O_SYNCOUTEN) = HWREGH(base + EPWM_O_SYNCOUTEN) |
                                      (uint16_t)source;
}

//*****************************************************************************
//
//! Disables sync-out pulse source.
//!
//! \param base is the base address of the EPWM module.
//! \param source is the sync-out pulse source.
//!
//! This function disables the sync-out pulse source.
//! Below valid values for param \b source can be OR'd together to disable
//! multiple sync-out sources:
//!  - EPWM_SYNC_OUT_PULSE_ON_SOFTWARE - disables software as sync-out source
//!
//!  - EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO - disables counter equal to zero event
//!                                       as sync-out source
//!  - EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_B - disables counter equal to cmpb
//!                                            event as sync-out source
//!  - EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_C - disables counter equal to cmpc
//!                                            event as sync-out source
//!  - EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_D - disables counter equal to cmpd
//!                                            event as sync-out source
//!  - EPWM_SYNC_OUT_PULSE_ON_DCA_EVT1_SYNC - disables DCA event1 sync signal as
//!                                           sync-out source
//!  - EPWM_SYNC_OUT_PULSE_ON_DCB_EVT1_SYNC - disables DCB event1 sync signal as
//!                                           sync-out source
//!  - EPWM_SYNC_OUT_PULSE_ON_ALL - disables all the above sync-out sources
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableSyncOutPulseSource(uint32_t base, uint16_t source)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(source <= EPWM_SYNC_OUT_SOURCE_M);

    //
    // Disable EPWM Sync-Out Sources.
    //
    HWREGH(base + EPWM_O_SYNCOUTEN) = HWREGH(base + EPWM_O_SYNCOUTEN) &
                                      ~((uint16_t)source);
}

//*****************************************************************************
//
//! Set up the one-shot sync-out trigger source.
//!
//! \param base is the base address of the EPWM module.
//! \param trigger is the one-shot sync-out signal trigger source.
//!
//! This function sets the one-shot sync-out trigger source.
//! Valid values for param \b trigger are:
//!  - EPWM_OSHT_SYNC_OUT_TRIG_SYNC - Trigger for one-shot sync-out signal is
//!                                    one-shot sync event.
//!  - EPWM_OSHT_SYNC_OUT_TRIG_RELOAD - Trigger for one-shot sync-out signal is
//!                                     one-shot reload event.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setOneShotSyncOutTrigger(uint32_t base,
                              EPWM_OneShotSyncOutTrigger trigger)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set source for One-Shot Sync-Out Pulse.
    //
    HWREGH(base + EPWM_O_TBCTL3) =
            (HWREGH(base + EPWM_O_TBCTL3) & ~(EPWM_TBCTL3_OSSFRCEN)) |
            (uint16_t)trigger;
}

//*****************************************************************************
//
//! Set PWM period load mode.
//!
//! \param base is the base address of the EPWM module.
//! \param loadMode is the PWM period load mode.
//!
//! This function sets the load mode for the PWM period. If loadMode is set to
//! EPWM_PERIOD_SHADOW_LOAD, a write or read to the TBPRD (PWM Period count
//! register) accesses the shadow register. If loadMode is set to
//! EPWM_PERIOD_DIRECT_LOAD, a write or read to the TBPRD register accesses the
//! register directly.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setPeriodLoadMode(uint32_t base, EPWM_PeriodLoadMode loadMode)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    if(loadMode == EPWM_PERIOD_SHADOW_LOAD)
    {
        //
        // Clear PRDLD
        //
        HWREGH(base + EPWM_O_TBCTL) &= ~EPWM_TBCTL_PRDLD;
    }
    else
    {
        //
        // Set PRDLD
        //
        HWREGH(base + EPWM_O_TBCTL) |= EPWM_TBCTL_PRDLD;
    }
}

//*****************************************************************************
//
//! Enable phase shift load
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables loading of phase shift when the appropriate sync
//! event occurs.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enablePhaseShiftLoad(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set PHSEN bit
    //
    HWREGH(base + EPWM_O_TBCTL) |= EPWM_TBCTL_PHSEN;
}

//*****************************************************************************
//
//! Disable phase shift load
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables loading of phase shift.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disablePhaseShiftLoad(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Clear PHSEN bit
    //
    HWREGH(base + EPWM_O_TBCTL) &= ~EPWM_TBCTL_PHSEN;
}

//*****************************************************************************
//
//! Set time base counter mode
//!
//! \param base is the base address of the EPWM module.
//! \param counterMode is the time base counter mode.
//!
//! This function sets up the time base counter mode.
//! Valid values for counterMode are:
//!  - EPWM_COUNTER_MODE_UP          - Up - count mode.
//!  - EPWM_COUNTER_MODE_DOWN        - Down - count mode.
//!  - EPWM_COUNTER_MODE_UP_DOWN     - Up - down - count mode.
//!  - EPWM_COUNTER_MODE_STOP_FREEZE - Stop - Freeze counter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setTimeBaseCounterMode(uint32_t base, EPWM_TimeBaseCountMode counterMode)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Write to CTRMODE bit
    //
    HWREGH(base + EPWM_O_TBCTL) =
            ((HWREGH(base + EPWM_O_TBCTL) & ~(EPWM_TBCTL_CTRMODE_M)) |
             ((uint16_t)counterMode));
}

//*****************************************************************************
//
//! Set shadow to active period load on sync mode
//!
//! \param base is the base address of the EPWM module.
//! \param shadowLoadMode is the shadow to active load mode.
//!
//! This function sets up the shadow to active Period register load mode with
//! respect to a sync event. Valid values for shadowLoadMode are:
//!  - EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO - shadow to active load occurs when
//!                                         time base counter reaches 0.
//!  - EPWM_SHADOW_LOAD_MODE_COUNTER_SYNC - shadow to active load occurs when
//!                                         time base counter reaches 0 and a
//!                                         SYNC occurs.
//!  - EPWM_SHADOW_LOAD_MODE_SYNC         - shadow to active load occurs only
//!                                         when a SYNC occurs.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_selectPeriodLoadEvent(uint32_t base,
                           EPWM_PeriodShadowLoadMode shadowLoadMode)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Write to PRDLDSYNC bit
    //
    HWREGH(base + EPWM_O_TBCTL2) =
                ((HWREGH(base + EPWM_O_TBCTL2) & ~(EPWM_TBCTL2_PRDLDSYNC_M)) |
                 ((uint16_t)shadowLoadMode << EPWM_TBCTL2_PRDLDSYNC_S));
}
//*****************************************************************************
//
//! Enable one shot sync mode
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables one shot sync mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableOneShotSync(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set OSHTSYNCMODE bit
    //
    HWREGH(base + EPWM_O_TBCTL2) |= EPWM_TBCTL2_OSHTSYNCMODE;
}

//*****************************************************************************
//
//! Disable one shot sync mode
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables one shot sync mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableOneShotSync(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Clear OSHTSYNCMODE bit
    //
    HWREGH(base + EPWM_O_TBCTL2) &= ~EPWM_TBCTL2_OSHTSYNCMODE;
}

//*****************************************************************************
//
//! Start one shot sync mode
//!
//! \param base is the base address of the EPWM module.
//!
//! This function propagates a one shot sync pulse.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_startOneShotSync(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set OSHTSYNC bit
    //
    HWREGH(base + EPWM_O_TBCTL2) |= EPWM_TBCTL2_OSHTSYNC;
}

//*****************************************************************************
//
//! Returns time base counter value.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the current value of the time base counter.
//!
//! \return returns time base counter value
//
//*****************************************************************************
static inline uint16_t
EPWM_getTimeBaseCounterValue(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Returns TBCTR value
    //
    return(HWREGH(base + EPWM_O_TBCTR));
}

//*****************************************************************************
//
//! Return time base counter maximum status.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the status of the time base max counter.
//!
//! \return Returns true if the counter has reached 0xFFFF.
//!         Returns false if the counter hasn't reached 0xFFFF.
//
//*****************************************************************************
static inline bool
EPWM_getTimeBaseCounterOverflowStatus(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return true if CTRMAX bit is set, false otherwise
    //
    return(((HWREGH(base + EPWM_O_TBSTS) & EPWM_TBSTS_CTRMAX) ==
            EPWM_TBSTS_CTRMAX) ? true : false);
}

//*****************************************************************************
//
//! Clear max time base counter event.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function clears the max time base counter latch event. The latch event
//! occurs when the time base counter reaches its maximum value of 0xFFFF.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_clearTimeBaseCounterOverflowEvent(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set CTRMAX bit
    //
    HWREGH(base + EPWM_O_TBSTS) = EPWM_TBSTS_CTRMAX;
}

//*****************************************************************************
//
//! Return external sync signal status.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the external sync signal status.
//!
//! \return Returns true if if an external sync signal event
//!         Returns false if there is no event.
//
//*****************************************************************************
static inline bool
EPWM_getSyncStatus(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return true if SYNCI bit is set, false otherwise
    //
    return(((HWREGH(base + EPWM_O_TBSTS) & EPWM_TBSTS_SYNCI) ==
            EPWM_TBSTS_SYNCI) ? true : false);
}

//*****************************************************************************
//
//! Clear external sync signal event.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function clears the external sync signal latch event.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_clearSyncEvent(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set SYNCI bit
    //
    HWREGH(base + EPWM_O_TBSTS) = EPWM_TBSTS_SYNCI;
}

//*****************************************************************************
//
//! Return time base counter direction.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the direction of the time base counter.
//!
//! \return returns EPWM_TIME_BASE_STATUS_COUNT_UP if the counter is counting
//!                 up or EPWM_TIME_BASE_STATUS_COUNT_DOWN if the counter is
//!                 counting down.
//
//*****************************************************************************
static inline uint16_t
EPWM_getTimeBaseCounterDirection(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return CTRDIR bit
    //
    return(HWREGH(base + EPWM_O_TBSTS) & EPWM_TBSTS_CTRDIR);
}

//*****************************************************************************
//
//! Sets the phase shift offset counter value.
//!
//! \param base is the base address of the EPWM module.
//! \param phaseCount is the phase shift count value.
//!
//! This function sets the 16 bit time-base counter phase of the ePWM relative
//! to the time-base that is supplying the synchronization input signal. Call
//! the EPWM_enablePhaseShiftLoad() function to enable loading of the
//! phaseCount phase shift value when a sync event occurs.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setPhaseShift(uint32_t base, uint16_t phaseCount)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Write to TBPHS bit
    //
    HWREG(base + EPWM_O_TBPHS) =
                   ((HWREG(base + EPWM_O_TBPHS) &
                    ~((uint32_t)EPWM_TBPHS_TBPHS_M)) |
                    ((uint32_t)phaseCount << EPWM_TBPHS_TBPHS_S));
}

//*****************************************************************************
//
//! Sets the PWM period count.
//!
//! \param base is the base address of the EPWM module.
//! \param periodCount is period count value.
//!
//! This function sets the period of the PWM count. The value of periodCount is
//! the value written to the register. User should map the desired period or
//! frequency of the waveform into the correct periodCount.
//! Invoke the function EPWM_selectPeriodLoadEvent() with the appropriate
//! parameter to set the load mode of the Period count. periodCount has a
//! maximum valid value of 0xFFFF
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setTimeBasePeriod(uint32_t base, uint16_t periodCount)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Write to TBPRD bit
    //
    HWREGH(base + EPWM_O_TBPRD) = periodCount;
}

//*****************************************************************************
//
//! Gets the PWM period count.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function gets the period of the PWM count.
//!
//! \return The period count value.
//
//*****************************************************************************
static inline uint16_t
EPWM_getTimeBasePeriod(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Read from TBPRD bit
    //
    return(HWREGH(base + EPWM_O_TBPRD));
}

//*****************************************************************************
//
//! Sets the EPWM links.
//!
//! \param base is the base address of the EPWM module.
//! \param epwmLink is the ePWM instance to link with.
//! \param linkComp is the ePWM component to link.
//!
//! This function links the component defined in linkComp in the current ePWM
//! instance with the linkComp component of the ePWM instance defined by
//! epwmLink. A change (a write) in the value of linkComp component of epwmLink
//! instance, causes a change in the current ePWM linkComp component.
//! For example if the current ePWM is ePWM3 and the values of epwmLink and
//! linkComp are EPWM_LINK_WITH_EPWM_1 and EPWM_LINK_COMP_C respectively,
//! then a write to COMPC register in ePWM1, will result in a simultaneous
//! write to COMPC register in ePWM3.
//! Valid values for epwmLink are:
//!   - EPWM_LINK_WITH_EPWM_1  - link current ePWM with ePWM1
//!   - EPWM_LINK_WITH_EPWM_2  - link current ePWM with ePWM2
//!   - EPWM_LINK_WITH_EPWM_3  - link current ePWM with ePWM3
//!   - EPWM_LINK_WITH_EPWM_4  - link current ePWM with ePWM4
//!   - EPWM_LINK_WITH_EPWM_5  - link current ePWM with ePWM5
//!   - EPWM_LINK_WITH_EPWM_6  - link current ePWM with ePWM6
//!   - EPWM_LINK_WITH_EPWM_7  - link current ePWM with ePWM7
//!   - EPWM_LINK_WITH_EPWM_8  - link current ePWM with ePWM8
//!   - EPWM_LINK_WITH_EPWM_9  - link current ePWM with ePWM9
//!   - EPWM_LINK_WITH_EPWM_10  - link current ePWM with ePWM10
//!   - EPWM_LINK_WITH_EPWM_11  - link current ePWM with ePWM11
//!   - EPWM_LINK_WITH_EPWM_12  - link current ePWM with ePWM12
//!   - EPWM_LINK_WITH_EPWM_13  - link current ePWM with ePWM13
//!   - EPWM_LINK_WITH_EPWM_14  - link current ePWM with ePWM14
//!   - EPWM_LINK_WITH_EPWM_15  - link current ePWM with ePWM15
//!   - EPWM_LINK_WITH_EPWM_16  - link current ePWM with ePWM16
//!
//! Valid values for linkComp are:
//!   - EPWM_LINK_TBPRD   - link TBPRD:TBPRDHR registers
//!   - EPWM_LINK_COMP_A   - link COMPA registers
//!   - EPWM_LINK_COMP_B   - link COMPB registers
//!   - EPWM_LINK_COMP_C   - link COMPC registers
//!   - EPWM_LINK_COMP_D   - link COMPD registers
//!   - EPWM_LINK_GLDCTL2  - link GLDCTL2 registers
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setupEPWMLinks(uint32_t base, EPWM_CurrentLink epwmLink,
                    EPWM_LinkComponent linkComp)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    uint32_t registerOffset;
    registerOffset = base + EPWM_O_XLINK;

    //
    // Configure EPWM links
    //
    HWREG(registerOffset) =
        ((HWREG(registerOffset) & ~(EPWM_XLINK_TBPRDLINK_M << (uint32_t)linkComp)) |
        ((uint32_t)epwmLink << (uint32_t)linkComp));
}


//*****************************************************************************
//
//! Sets up the Counter Compare shadow load mode
//!
//! \param base is the base address of the EPWM module.
//! \param compModule is the counter compare module.
//! \param loadMode is the shadow to active load mode.
//!
//! This function enables and sets up the counter compare shadow load mode.
//! Valid values for the variables are:
//!  - compModule
//!      - EPWM_COUNTER_COMPARE_A - counter compare A.
//!      - EPWM_COUNTER_COMPARE_B - counter compare B.
//!      - EPWM_COUNTER_COMPARE_C - counter compare C.
//!      - EPWM_COUNTER_COMPARE_D - counter compare D.
//!  - loadMode
//!      - EPWM_COMP_LOAD_ON_CNTR_ZERO - load when counter equals zero
//!      - EPWM_COMP_LOAD_ON_CNTR_PERIOD - load when counter equals period
//!      - EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD - load when counter equals
//!                                             zero or period
//!      - EPWM_COMP_LOAD_FREEZE  - Freeze shadow to active load
//!      - EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO - load when counter equals zero
//!      - EPWM_COMP_LOAD_ON_SYNC_CNTR_PERIOD -load when counter equals period
//!      - EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO_PERIOD - load when counter equals
//!                                                  zero or period
//!      - EPWM_COMP_LOAD_ON_SYNC_ONLY - load on sync only
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setCounterCompareShadowLoadMode(uint32_t base,
                                     EPWM_CounterCompareModule compModule,
                                     EPWM_CounterCompareLoadMode loadMode)
{
    uint16_t syncModeOffset;
    uint16_t loadModeOffset;
    uint16_t shadowModeOffset;
    uint32_t registerOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    if((compModule == EPWM_COUNTER_COMPARE_A) ||
       (compModule == EPWM_COUNTER_COMPARE_C))
    {
        syncModeOffset = 10U;
        loadModeOffset = 0U;
        shadowModeOffset = 4U;
    }
    else
    {
        syncModeOffset = 12U;
        loadModeOffset = 2U;
        shadowModeOffset = 6U;
    }

    //
    // Get the register offset.  EPWM_O_CMPCTL for A&B or
    // EPWM_O_CMPCTL2 for C&D
    //
    registerOffset = base + EPWM_O_CMPCTL + ((uint32_t)compModule & 0x1U);

    //
    // Set the appropriate sync and load mode bits and also enable shadow
    // load mode. Shadow to active load can also be frozen.
    //
    HWREGH(registerOffset) = ((HWREGH(registerOffset) &
                         ~((0x3U << syncModeOffset) | // Clear sync mode
                           (0x3U << loadModeOffset) | // Clear load mode
                           (0x1U << shadowModeOffset))) | // shadow mode
                         ((((uint16_t)loadMode >> 2U) << syncModeOffset) |
                         (((uint16_t)loadMode & 0x3U) << loadModeOffset)));
}

//*****************************************************************************
//
//! Disable Counter Compare shadow load mode
//!
//! \param base is the base address of the EPWM module.
//! \param compModule is the counter compare module.
//!
//! This function disables counter compare shadow load mode.
//! Valid values for the variables are:
//!  - compModule
//!      - EPWM_COUNTER_COMPARE_A - counter compare A.
//!      - EPWM_COUNTER_COMPARE_B - counter compare B.
//!      - EPWM_COUNTER_COMPARE_C - counter compare C.
//!      - EPWM_COUNTER_COMPARE_D - counter compare D.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableCounterCompareShadowLoadMode(uint32_t base,
                                         EPWM_CounterCompareModule compModule)
{
    uint16_t shadowModeOffset;
    uint32_t registerOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    if((compModule == EPWM_COUNTER_COMPARE_A) ||
       (compModule == EPWM_COUNTER_COMPARE_C))
    {
        shadowModeOffset = 4U;
    }
    else
    {
        shadowModeOffset = 6U;
    }

    //
    // Get the register offset.  EPWM_O_CMPCTL for A&B or
    // EPWM_O_CMPCTL2 for C&D
    //
    registerOffset = base + EPWM_O_CMPCTL + ((uint32_t)compModule & 0x1U);

    //
    // Disable shadow load mode.
    //
    HWREGH(registerOffset) = (HWREGH(registerOffset) |
                             (0x1U << shadowModeOffset));
}

//*****************************************************************************
//
//! Set counter compare values.
//!
//! \param base is the base address of the EPWM module.
//! \param compModule is the Counter Compare value module.
//! \param compCount is the counter compare count value.
//!
//! This function sets the counter compare value for counter compare registers.
//! The maximum value for compCount is 0xFFFF.
//! Valid values for compModule are:
//!   - EPWM_COUNTER_COMPARE_A - counter compare A.
//!   - EPWM_COUNTER_COMPARE_B - counter compare B.
//!   - EPWM_COUNTER_COMPARE_C - counter compare C.
//!   - EPWM_COUNTER_COMPARE_D - counter compare D.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setCounterCompareValue(uint32_t base, EPWM_CounterCompareModule compModule,
                            uint16_t compCount)
{
    uint32_t registerOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Get the register offset for the Counter compare
    //
    registerOffset = EPWM_O_CMPA + (uint32_t)compModule;

    //
    // Write to the counter compare registers.
    //
    if((compModule == EPWM_COUNTER_COMPARE_A) ||
        (compModule == EPWM_COUNTER_COMPARE_B))
    {
        //
        // Write to COMPA or COMPB bits
        //
        HWREGH(base + registerOffset + 0x1U) = compCount;
    }
    else
    {
        //
        // Write to COMPC or COMPD bits
        //
        HWREGH(base + registerOffset) = compCount;
    }
}

//*****************************************************************************
//
//! Get counter compare values.
//!
//! \param base is the base address of the EPWM module.
//! \param compModule is the Counter Compare value module.
//!
//! This function gets the counter compare value for counter compare registers.
//! Valid values for compModule are:
//!   - EPWM_COUNTER_COMPARE_A - counter compare A.
//!   - EPWM_COUNTER_COMPARE_B - counter compare B.
//!   - EPWM_COUNTER_COMPARE_C - counter compare C.
//!   - EPWM_COUNTER_COMPARE_D - counter compare D.
//!
//! \return The counter compare count value.
//
//*****************************************************************************
static inline uint16_t
EPWM_getCounterCompareValue(uint32_t base, EPWM_CounterCompareModule compModule)
{
    uint32_t registerOffset;
    uint16_t compCount;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Get the register offset for the Counter compare
    //
    registerOffset = EPWM_O_CMPA + (uint32_t)compModule;

    //
    // Read from the counter compare registers.
    //
    if((compModule == EPWM_COUNTER_COMPARE_A) ||
        (compModule == EPWM_COUNTER_COMPARE_B))
    {
        //
        // Read COMPA or COMPB bits
        //
        compCount = (uint16_t)((HWREG(base + registerOffset) &
                     0xFFFF0000UL) >> 16U);
    }
    else
    {
        //
        // Read COMPC or COMPD bits
        //
        compCount = HWREGH(base + registerOffset);
    }
    return(compCount);
}

//*****************************************************************************
//
//! Return the counter compare shadow register full status.
//!
//! \param base is the base address of the EPWM module.
//! \param compModule is the Counter Compare value module.
//!
//! This function returns the counter Compare shadow register full status flag.
//! Valid values for compModule are:
//!   - EPWM_COUNTER_COMPARE_A - counter compare A.
//!   - EPWM_COUNTER_COMPARE_B - counter compare B.
//!
//! \return Returns true if the shadow register is full.
//!         Returns false if the shadow register is not full.
//
//*****************************************************************************
static inline bool
EPWM_getCounterCompareShadowStatus(uint32_t base,
                                   EPWM_CounterCompareModule compModule)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Check the validity of input.
    // COMPA and COMPB are valid input arguments.
    //
    ASSERT((compModule == EPWM_COUNTER_COMPARE_A) ||
            (compModule == EPWM_COUNTER_COMPARE_B));

    //
    // Read the value of SHDWAFULL or SHDWBFULL bit
    //
    return((((HWREG(base + EPWM_O_CMPCTL) >>
              ((((uint16_t)compModule >> 1U) & 0x1U) + 8U)) &
              0x1U) == 0x1U) ? true:false);
}

//
// Action Qualifier module related APIs
//
//*****************************************************************************
//
//! Sets the Action Qualifier shadow load mode
//!
//! \param base is the base address of the EPWM module.
//! \param aqModule is the Action Qualifier module value.
//! \param loadMode is the shadow to active load mode.
//!
//! This function enables and sets the Action Qualifier shadow load mode.
//! Valid values for the variables are:
//!  - aqModule
//!      - EPWM_ACTION_QUALIFIER_A - Action Qualifier A.
//!      - EPWM_ACTION_QUALIFIER_B - Action Qualifier B.
//!  - loadMode
//!      - EPWM_AQ_LOAD_ON_CNTR_ZERO - load when counter equals zero
//!      - EPWM_AQ_LOAD_ON_CNTR_PERIOD - load when counter equals period
//!      - EPWM_AQ_LOAD_ON_CNTR_ZERO_PERIOD - load when counter equals
//!                                               zero or period
//!      - EPWM_AQ_LOAD_FREEZE  - Freeze shadow to active load
//!      - EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO - load on sync or when counter
//!                                          equals zero
//!      - EPWM_AQ_LOAD_ON_SYNC_CNTR_PERIOD - load on sync or when counter
//!                                           equals period
//!      - EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO_PERIOD - load on sync or when
//!                                               counter equals zero or period
//!      - EPWM_AQ_LOAD_ON_SYNC_ONLY - load on sync only
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierShadowLoadMode(uint32_t base,
                                      EPWM_ActionQualifierModule aqModule,
                                      EPWM_ActionQualifierLoadMode loadMode)
{
    uint16_t syncModeOffset;
    uint16_t shadowModeOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    syncModeOffset = 8U + (uint16_t)aqModule;
    shadowModeOffset = 4U + (uint16_t)aqModule;

    //
    // Set the appropriate sync and load mode bits and also enable shadow
    // load mode. Shadow to active load can also be frozen.
    //
    HWREGH(base + EPWM_O_AQCTL) = ((HWREGH(base + EPWM_O_AQCTL) &
                                   (~((0x3U << (uint16_t)aqModule) |
                                   (0x3U << (uint16_t)syncModeOffset))) |
                                   (0x1U << shadowModeOffset)) |
                                   ((((uint16_t)loadMode >> 2U) <<
                                     syncModeOffset) | (((uint16_t)loadMode &
                                    0x3U) << (uint16_t)aqModule)));
}

//*****************************************************************************
//
//! Disable Action Qualifier shadow load mode
//!
//! \param base is the base address of the EPWM module.
//! \param aqModule is the Action Qualifier module value.
//!
//! This function disables the Action Qualifier  shadow load mode.
//! Valid values for the variables are:
//!  - aqModule
//!      - EPWM_ACTION_QUALIFIER_A - Action Qualifier A.
//!      - EPWM_ACTION_QUALIFIER_B - Action Qualifier B.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableActionQualifierShadowLoadMode(uint32_t base,
                                          EPWM_ActionQualifierModule aqModule)
{
    uint16_t shadowModeOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    shadowModeOffset = 4U + (uint16_t)aqModule;

    //
    // Disable shadow load mode. Action qualifier is loaded on
    // immediate mode only.
    //
    HWREGH(base + EPWM_O_AQCTL) &= ~(1U << shadowModeOffset);
}

//*****************************************************************************
//
//! Set up Action qualifier trigger source for event T1
//!
//! \param base is the base address of the EPWM module.
//! \param trigger sources for Action Qualifier triggers.
//!
//! This function sets up the sources for Action Qualifier event T1.
//! Valid values for trigger are:
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1       - Digital compare event A 1
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_2       - Digital compare event A 2
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCB_1       - Digital compare event B 1
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCB_2       - Digital compare event B 2
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_1        - Trip zone 1
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_2        - Trip zone 2
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_3        - Trip zone 3
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_EPWM_SYNCIN - ePWM sync
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DC_EVTFILT  - Digital compare filter event
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierT1TriggerSource(uint32_t base,
                                     EPWM_ActionQualifierTriggerSource trigger)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set T1 trigger source
    //
    HWREGH(base + EPWM_O_AQTSRCSEL) =
         ((HWREGH(base + EPWM_O_AQTSRCSEL) & (~EPWM_AQTSRCSEL_T1SEL_M)) |
          ((uint16_t)trigger));
}

//*****************************************************************************
//
//! Set up Action qualifier trigger source for event T2
//!
//! \param base is the base address of the EPWM module.
//! \param trigger sources for Action Qualifier triggers.
//!
//! This function sets up the sources for Action Qualifier event T2.
//! Valid values for trigger are:
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1       - Digital compare event A 1
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_2       - Digital compare event A 2
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCB_1       - Digital compare event B 1
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DCB_2       - Digital compare event B 2
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_1        - Trip zone 1
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_2        - Trip zone 2
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_3        - Trip zone 3
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_EPWM_SYNCIN - ePWM sync
//!   - EPWM_AQ_TRIGGER_EVENT_TRIG_DC_EVTFILT  - Digital compare filter event
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierT2TriggerSource(uint32_t base,
                                      EPWM_ActionQualifierTriggerSource trigger)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set T2 trigger source
    //
    HWREGH(base + EPWM_O_AQTSRCSEL) =
          ((HWREGH(base + EPWM_O_AQTSRCSEL) & (~EPWM_AQTSRCSEL_T2SEL_M)) |
           ((uint16_t)trigger << EPWM_AQTSRCSEL_T2SEL_S));
}

//*****************************************************************************
//
//! Set up Action qualifier outputs
//!
//! \param base is the base address of the EPWM module.
//! \param epwmOutput is the ePWM pin type.
//! \param output is the Action Qualifier output.
//! \param event is the event that causes a change in output.
//!
//! This function sets up the Action Qualifier output on ePWM A or ePWMB,
//! depending on the value of epwmOutput, to a value specified by outPut based
//! on the input events - specified by event.
//! The following are valid values for the parameters.
//!   - epwmOutput
//!       - EPWM_AQ_OUTPUT_A          - ePWMxA output
//!       - EPWM_AQ_OUTPUT_B          - ePWMxB output
//!   - output
//!       - EPWM_AQ_OUTPUT_NO_CHANGE  - No change in the output pins
//!       - EPWM_AQ_OUTPUT_LOW        - Set output pins to low
//!       - EPWM_AQ_OUTPUT_HIGH       - Set output pins to High
//!       - EPWM_AQ_OUTPUT_TOGGLE     - Toggle the output pins
//!   - event
//!       - EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO       - Time base counter equals
//!                                                 zero
//!       - EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD     - Time base counter equals
//!                                                 period
//!       - EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA    - Time base counter up equals
//!                                                 COMPA
//!       - EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA  - Time base counter down
//!                                                 equals COMPA
//!       - EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB    - Time base counter up equals
//!                                                 COMPB
//!       - EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB  - Time base counter down
//!                                                 equals COMPB
//!       - EPWM_AQ_OUTPUT_ON_T1_COUNT_UP         - T1 event on count up
//!       - EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN       - T1 event on count down
//!       - EPWM_AQ_OUTPUT_ON_T2_COUNT_UP         - T2 event on count up
//!       - EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN       - T2 event on count down
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierAction(uint32_t base,
                              EPWM_ActionQualifierOutputModule epwmOutput,
                              EPWM_ActionQualifierOutput output,
                              EPWM_ActionQualifierOutputEvent event)
{
    uint32_t registerOffset;
    uint32_t registerTOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Get the register offset
    //
    registerOffset = EPWM_O_AQCTLA + (uint32_t)epwmOutput;
    registerTOffset = EPWM_O_AQCTLA2 + (uint32_t)epwmOutput;

    //
    // If the event occurs on T1 or T2 events
    //
    if(((uint16_t)event & 0x1U) == 1U)
    {
        //
        // Write to T1U,T1D,T2U or T2D of AQCTLA2 register
        //
        HWREGH(base + registerTOffset) =
         ((HWREGH(base + registerTOffset) & ~(3U << ((uint16_t)event - 1U))) |
          ((uint16_t)output << ((uint16_t)event - 1U)));
    }
    else
    {
        //
        // Write to ZRO,PRD,CAU,CAD,CBU or CBD bits of AQCTLA register
        //
        HWREGH(base + registerOffset) =
                ((HWREGH(base + registerOffset) & ~(3U << (uint16_t)event)) |
                 ((uint16_t)output << (uint16_t)event));
    }
}

//*****************************************************************************
//
//! Set up Action qualifier event outputs
//!
//! \param base is the base address of the EPWM module.
//! \param epwmOutput is the ePWM pin type.
//! \param action is the desired action when the specified event occurs
//!
//! This function sets up the Action Qualifier output on ePWMA or ePWMB,
//! depending on the value of epwmOutput, to a value specified by action.
//! Valid action param values from different time base counter scenarios
//! should be OR'd together to configure complete action for a pwm output.
//! The following are valid values for the parameters.
//!   - epwmOutput
//!       - EPWM_AQ_OUTPUT_A          - ePWMxA output
//!       - EPWM_AQ_OUTPUT_B          - ePWMxB output
//!
//!   - action
//!       - When time base counter equals zero
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_ZERO   - Time base counter equals zero
//!                                             and no change in output pins
//!         - EPWM_AQ_OUTPUT_LOW_ZERO         - Time base counter equals zero
//!                                             and set output pins to low
//!         - EPWM_AQ_OUTPUT_HIGH_ZERO        - Time base counter equals zero
//!                                             and set output pins to high
//!         - EPWM_AQ_OUTPUT_TOGGLE_ZERO      - Time base counter equals zero
//!                                             and toggle the output pins
//!       - When time base counter equals period
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_PERIOD - Time base counter equals period
//!                                             and no change in output pins
//!         - EPWM_AQ_OUTPUT_LOW_PERIOD       - Time base counter equals period
//!                                             and set output pins to low
//!         - EPWM_AQ_OUTPUT_HIGH_PERIOD      - Time base counter equals period
//!                                             and set output pins to high
//!         - EPWM_AQ_OUTPUT_TOGGLE_PERIOD    - Time base counter equals period
//!                                             and toggle the output pins
//!       - When time base counter equals CMPA during up-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_UP_CMPA  - Time base counter up equals
//!                                               COMPA and no change in the
//!                                               output pins
//!         - EPWM_AQ_OUTPUT_LOW_UP_CMPA        - Time base counter up equals
//!                                               COMPA and set output pins low
//!         - EPWM_AQ_OUTPUT_HIGH_UP_CMPA       - Time base counter up equals
//!                                               COMPA and set output pins high
//!         - EPWM_AQ_OUTPUT_TOGGLE_UP_CMPA     - Time base counter up equals
//!                                               COMPA and toggle output pins
//!       - When time base counter equals CMPA during down-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_CMPA - Time base counter down equals
//!                                                COMPA and no change in the
//!                                                output pins
//!         - EPWM_AQ_OUTPUT_LOW_DOWN_CMPA      - Time base counter down equals
//!                                               COMPA and set output pins low
//!         - EPWM_AQ_OUTPUT_HIGH_DOWN_CMPA     - Time base counter down equals
//!                                               COMPA and set output pins high
//!         - EPWM_AQ_OUTPUT_TOGGLE_DOWN_CMPA   - Time base counter down equals
//!                                               COMPA and toggle output pins
//!       - When time base counter equals CMPB during up-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_UP_CMPB  - Time base counter up equals
//!                                               COMPB and no change in the
//!                                               output pins
//!         - EPWM_AQ_OUTPUT_LOW_UP_CMPB        - Time base counter up equals
//!                                               COMPB and set output pins low
//!         - EPWM_AQ_OUTPUT_HIGH_UP_CMPB       - Time base counter up equals
//!                                               COMPB and set output pins high
//!         - EPWM_AQ_OUTPUT_TOGGLE_UP_CMPB     - Time base counter up equals
//!                                               COMPB and toggle output pins
//!       - When time base counter equals CMPB during down-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_CMPB- Time base counter down equals
//!                                               COMPB and no change in the
//!                                               output pins
//!         - EPWM_AQ_OUTPUT_LOW_DOWN_CMPB      - Time base counter down equals
//!                                               COMPB and set output pins low
//!         - EPWM_AQ_OUTPUT_HIGH_DOWN_CMPB     - Time base counter down equals
//!                                               COMPB and set output pins high
//!         - EPWM_AQ_OUTPUT_TOGGLE_DOWN_CMPB   - Time base counter down equals
//!                                               COMPB and toggle output pins
//!
//! \b note:  A logical OR of the valid values should be passed as the action
//!           parameter. Single action should be configured for each time base
//!           counter scenario.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierActionComplete(uint32_t base,
                                   EPWM_ActionQualifierOutputModule epwmOutput,
                                   uint16_t action)
{
    uint32_t registerOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Get the register offset
    //
    registerOffset = EPWM_O_AQCTLA + (uint32_t)epwmOutput;

    //
    // Write to ZRO, PRD, CAU, CAD, CBU or CBD bits of AQCTLA register
    //
    HWREGH(base + registerOffset) = (uint16_t)action;
}

//*****************************************************************************
//
//! Set up Additional action qualifier event outputs
//!
//! \param base is the base address of the EPWM module.
//! \param epwmOutput is the ePWM pin type.
//! \param action is the desired action when the specified event occurs
//!
//! This function sets up the Additional Action Qualifier output on ePWMA or
//! ePWMB depending on the value of epwmOutput, to a value specified by action.
//! Valid action param values from different event scenarios should be OR'd
//! together to configure complete action for a pwm output.
//! The following are valid values for the parameters.
//!   - epwmOutput
//!       - EPWM_AQ_OUTPUT_A          - ePWMxA output
//!       - EPWM_AQ_OUTPUT_B          - ePWMxB output
//!   - action
//!       - When T1 event occurs during up-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_UP_T1  - T1 event on count up
//!                                             and no change in output pins
//!         - EPWM_AQ_OUTPUT_LOW_UP_T1        - T1 event on count up
//!                                           and set output pins to low
//!         - EPWM_AQ_OUTPUT_HIGH_UP_T1       - T1 event on count up
//!                                           and set output pins to high
//!         - EPWM_AQ_OUTPUT_TOGGLE_UP_T1     - T1 event on count up
//!                                           and toggle the output pins
//!       - When T1 event occurs during down-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_T1- T1 event on count down
//!                                           and no change in output pins
//!         - EPWM_AQ_OUTPUT_LOW_DOWN_T1      - T1 event on count down
//!                                           and set output pins to low
//!         - EPWM_AQ_OUTPUT_HIGH_DOWN_T1     - T1 event on count down
//!                                           and set output pins to high
//!         - EPWM_AQ_OUTPUT_TOGGLE_DOWN_T1   - T1 event on count down
//!                                           and toggle the output pins
//!       - When T2 event occurs during up-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_UP_T2  - T2 event on count up
//!                                             and no change in output pins
//!         - EPWM_AQ_OUTPUT_LOW_UP_T2        - T2 event on count up
//!                                             and set output pins to low
//!         - EPWM_AQ_OUTPUT_HIGH_UP_T2       - T2 event on count up
//!                                             and set output pins to high
//!         - EPWM_AQ_OUTPUT_TOGGLE_UP_T2     - T2 event on count up
//!                                             and toggle the output pins
//!       - When T2 event occurs during down-count
//!         - EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_T2 - T2 event on count down
//!                                              and no change in output pins
//!         - EPWM_AQ_OUTPUT_LOW_DOWN_T2       - T2 event on count down
//!                                              and set output pins to low
//!         - EPWM_AQ_OUTPUT_HIGH_DOWN_T2      - T2 event on count down
//!                                              and set output pins to high
//!         - EPWM_AQ_OUTPUT_TOGGLE_DOWN_T2    - T2 event on count down
//!                                              and toggle the output pins
//!
//! \b note:  A logical OR of the valid values should be passed as the action
//!           parameter. Single action should be configured for each event
//!           scenario.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setAdditionalActionQualifierActionComplete(uint32_t base,
                               EPWM_ActionQualifierOutputModule epwmOutput,
                               uint16_t action)
{
    uint32_t registerTOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Get the register offset
    //
    registerTOffset = EPWM_O_AQCTLA2 + (uint32_t)epwmOutput;

    //
    // Write to T1U, T1D, T2U or T2D of AQCTLA2 register
    //
    HWREGH(base + registerTOffset) = (uint16_t)action;
}

//*****************************************************************************
//
//! Sets up Action qualifier continuous software load mode.
//!
//! \param base is the base address of the EPWM module.
//! \param mode is the mode for shadow to active load mode.
//!
//! This function sets up the AQCFRSC register load mode for continuous
//! software force reload mode. The software force actions are determined by
//! the EPWM_setActionQualifierContSWForceAction() function.
//! Valid values for mode are:
//!   - EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO        - shadow mode load when counter
//!                                               equals zero
//!   - EPWM_AQ_SW_SH_LOAD_ON_CNTR_PERIOD      - shadow mode load when counter
//!                                               equals period
//!   - EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO_PERIOD - shadow mode load when counter
//!                                               equals zero or period
//!   - EPWM_AQ_SW_IMMEDIATE_LOAD               - immediate mode load only
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierContSWForceShadowMode(uint32_t base,
                                             EPWM_ActionQualifierContForce mode)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set the Action qualifier software action reload mode.
    // Write to RLDCSF bit
    //
    HWREGH(base + EPWM_O_AQSFRC) =
            ((HWREGH(base + EPWM_O_AQSFRC) & ~EPWM_AQSFRC_RLDCSF_M) |
             ((uint16_t)mode << EPWM_AQSFRC_RLDCSF_S));
}

//*****************************************************************************
//
//! Triggers a continuous software forced event.
//!
//! \param base is the base address of the EPWM module.
//! \param epwmOutput is the ePWM pin type.
//! \param output is the Action Qualifier output.
//!
//! This function triggers a continuous software forced Action Qualifier output
//! on ePWM A or B based on the value of epwmOutput.
//! Valid values for the parameters are:
//!   - epwmOutput
//!       - EPWM_AQ_OUTPUT_A          - ePWMxA output
//!       - EPWM_AQ_OUTPUT_B          - ePWMxB output
//!   - output
//!       - EPWM_AQ_SW_DISABLED       - Software forcing disabled.
//!       - EPWM_AQ_SW_OUTPUT_LOW     - Set output pins to low
//!       - EPWM_AQ_SW_OUTPUT_HIGH    - Set output pins to High
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierContSWForceAction(uint32_t base,
                                    EPWM_ActionQualifierOutputModule epwmOutput,
                                    EPWM_ActionQualifierSWOutput output)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Initiate a continuous software forced output
    //
    if(epwmOutput == EPWM_AQ_OUTPUT_A)
    {
        HWREGH(base + EPWM_O_AQCSFRC) =
                ((HWREGH(base + EPWM_O_AQCSFRC) & ~EPWM_AQCSFRC_CSFA_M) |
                 ((uint16_t)output));
    }
    else
    {
        HWREGH(base + EPWM_O_AQCSFRC) =
                ((HWREGH(base + EPWM_O_AQCSFRC) & ~EPWM_AQCSFRC_CSFB_M) |
                 ((uint16_t)output << EPWM_AQCSFRC_CSFB_S)) ;
    }
}

//*****************************************************************************
//
//! Set up one time software forced Action qualifier outputs
//!
//! \param base is the base address of the EPWM module.
//! \param epwmOutput is the ePWM pin type.
//! \param output is the Action Qualifier output.
//!
//! This function sets up the one time software forced Action Qualifier output
//! on ePWM A or ePWMB, depending on the value of epwmOutput to a value
//! specified by outPut.
//! The following are valid values for the parameters.
//!   - epwmOutput
//!       - EPWM_AQ_OUTPUT_A          - ePWMxA output
//!       - EPWM_AQ_OUTPUT_B          - ePWMxB output
//!   - output
//!       - EPWM_AQ_OUTPUT_NO_CHANGE  - No change in the output pins
//!       - EPWM_AQ_OUTPUT_LOW        - Set output pins to low
//!       - EPWM_AQ_OUTPUT_HIGH       - Set output pins to High
//!       - EPWM_AQ_OUTPUT_TOGGLE     - Toggle the output pins
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setActionQualifierSWAction(uint32_t base,
                                EPWM_ActionQualifierOutputModule epwmOutput,
                                EPWM_ActionQualifierOutput output)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set the one time software forced action
    //
    if(epwmOutput == EPWM_AQ_OUTPUT_A)
    {
        HWREGH(base + EPWM_O_AQSFRC) =
                    ((HWREGH(base + EPWM_O_AQSFRC) & ~EPWM_AQSFRC_ACTSFA_M) |
                     ((uint16_t)output));
    }
    else
    {
        HWREGH(base + EPWM_O_AQSFRC) =
                    ((HWREGH(base + EPWM_O_AQSFRC) & ~EPWM_AQSFRC_ACTSFB_M) |
                     ((uint16_t)output << EPWM_AQSFRC_ACTSFB_S));
    }
}

//*****************************************************************************
//
//! Triggers a one time software forced event on Action qualifier
//!
//! \param base is the base address of the EPWM module.
//! \param epwmOutput is the ePWM pin type.
//!
//! This function triggers a one time software forced Action Qualifier event
//! on ePWM A or B based on the value of epwmOutput.
//! Valid values for epwmOutput are:
//!   - EPWM_AQ_OUTPUT_A          - ePWMxA output
//!   - EPWM_AQ_OUTPUT_B          - ePWMxB output
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_forceActionQualifierSWAction(uint32_t base,
                                  EPWM_ActionQualifierOutputModule epwmOutput)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Initiate a software forced event
    //
    if(epwmOutput == EPWM_AQ_OUTPUT_A)
    {
        HWREGH(base + EPWM_O_AQSFRC) |= EPWM_AQSFRC_OTSFA;
    }
    else
    {
        HWREGH(base + EPWM_O_AQSFRC) |= EPWM_AQSFRC_OTSFB;
    }
}

//
// Dead Band Module related APIs
//
//*****************************************************************************
//
//! Sets Dead Band signal output swap mode.
//!
//! \param base is the base address of the EPWM module.
//! \param output is the ePWM Dead Band output.
//! \param enableSwapMode is the output swap mode.
//!
//! This function sets up the output signal swap mode. For example if the
//! output variable is set to EPWM_DB_OUTPUT_A and enableSwapMode is true, then
//! the ePWM A output gets its signal from the ePWM B signal path. Valid values
//! for the input variables are:
//!  - output
//!      - EPWM_DB_OUTPUT_A   - ePWM output A
//!      - EPWM_DB_OUTPUT_B   - ePWM output B
//!  - enableSwapMode
//!      - true     - the output is swapped
//!      - false    - the output and the signal path are the same.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setDeadBandOutputSwapMode(uint32_t base, EPWM_DeadBandOutput output,
                               bool enableSwapMode)
{
    uint16_t mask;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    mask = (uint16_t)1U << ((uint16_t)output + EPWM_DBCTL_OUTSWAP_S);

    if(enableSwapMode)
    {
        //
        // Set the appropriate outswap bit to swap output
        //
        HWREGH(base + EPWM_O_DBCTL) = (HWREGH(base + EPWM_O_DBCTL) | mask);
    }
    else
    {
        //
        // Clear the appropriate outswap bit to disable output swap
        //
        HWREGH(base + EPWM_O_DBCTL) = (HWREGH(base + EPWM_O_DBCTL) & ~mask);
    }
}

//*****************************************************************************
//
//! Sets Dead Band signal output mode.
//!
//! \param base is the base address of the EPWM module.
//! \param delayMode is the Dead Band delay type.
//! \param enableDelayMode is the dead band delay mode.
//!
//! This function sets up the dead band delay mode. The delayMode variable
//! determines if the applied delay is Rising Edge or Falling Edge. The
//! enableDelayMode determines if a dead band delay should be applied.
//! Valid values for the variables are:
//!  - delayMode
//!      - EPWM_DB_RED   - Rising Edge delay
//!      - EPWM_DB_FED   - Falling Edge delay
//!  - enableDelayMode
//!      - true     - Falling edge or Rising edge delay is applied.
//!      - false    - Dead Band delay is bypassed.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setDeadBandDelayMode(uint32_t base, EPWM_DeadBandDelayMode delayMode,
                          bool enableDelayMode)
{
    uint16_t mask;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    mask = (uint16_t)1U << ((uint16_t)delayMode + EPWM_DBCTL_OUT_MODE_S);

    if(enableDelayMode)
    {
         //
         // Set the appropriate outmode bit to enable Dead Band delay
         //
         HWREGH(base + EPWM_O_DBCTL) = (HWREGH(base + EPWM_O_DBCTL) | mask);
    }
    else
    {
        //
        // Clear the appropriate outswap bit to disable output swap
        //
        HWREGH(base + EPWM_O_DBCTL) = (HWREGH(base + EPWM_O_DBCTL) & ~ mask);
    }
}

//*****************************************************************************
//
//! Sets Dead Band delay polarity.
//!
//! \param base is the base address of the EPWM module.
//! \param delayMode is the Dead Band delay type.
//! \param polarity is the polarity of the delayed signal.
//!
//! This function sets up the polarity as determined by the variable polarity
//! of the Falling Edge or Rising Edge delay depending on the value of
//! delayMode. Valid values for the variables are:
//!   - delayMode
//!       - EPWM_DB_RED   - Rising Edge delay
//!       - EPWM_DB_FED   - Falling Edge delay
//!   - polarity
//!       - EPWM_DB_POLARITY_ACTIVE_HIGH   - polarity is not inverted.
//!       - EPWM_DB_POLARITY_ACTIVE_LOW    - polarity is inverted.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setDeadBandDelayPolarity(uint32_t base,
                              EPWM_DeadBandDelayMode delayMode,
                              EPWM_DeadBandPolarity polarity)
{
    uint16_t shift;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    shift = (((uint16_t)delayMode ^ 0x1U) + EPWM_DBCTL_POLSEL_S);

    //
    // Set the appropriate polsel bits for dead band polarity
    //
    HWREGH(base + EPWM_O_DBCTL) =
                       ((HWREGH(base + EPWM_O_DBCTL) & ~ (1U << shift)) |
                        ((uint16_t)polarity << shift));
}

//*****************************************************************************
//
//! Sets Rising Edge Dead Band delay input.
//!
//! \param base is the base address of the EPWM module.
//! \param input is the input signal to the dead band.
//!
//! This function sets up the rising Edge delay input signal.
//! Valid values for input are:
//!     - EPWM_DB_INPUT_EPWMA   - Input signal is ePWMA( Valid for both Falling
//!                                  Edge and Rising Edge)
//!     - EPWM_DB_INPUT_EPWMB   - Input signal is ePWMB( Valid for both Falling
//!                                  Edge and Rising Edge)
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setRisingEdgeDeadBandDelayInput(uint32_t base, uint16_t input)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT((input == EPWM_DB_INPUT_EPWMA) ||
           (input == EPWM_DB_INPUT_EPWMB));

    //
    // Set the Rising Edge Delay input
    //
    HWREGH(base + EPWM_O_DBCTL) =
            ((HWREGH(base + EPWM_O_DBCTL) & ~(1U << (EPWM_DBCTL_IN_MODE_S))) |
             (input << EPWM_DBCTL_IN_MODE_S));
}

//*****************************************************************************
//
//! Sets Dead Band delay input.
//!
//! \param base is the base address of the EPWM module.
//! \param input is the input signal to the dead band.
//!
//! This function sets up the rising Edge delay input signal.
//! Valid values for input are:
//!   - EPWM_DB_INPUT_EPWMA   - Input signal is ePWMA(Valid for both Falling
//!                                Edge and Rising Edge)
//!   - EPWM_DB_INPUT_EPWMB   - Input signal is ePWMB(Valid for both Falling
//!                                Edge and Rising Edge)
//!   - EPWM_DB_INPUT_DB_RED  - Input signal is the output of Rising
//!                                Edge delay.
//!                               (Valid only for Falling Edge delay)
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setFallingEdgeDeadBandDelayInput(uint32_t base, uint16_t input)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT((input == EPWM_DB_INPUT_EPWMA) ||
           (input == EPWM_DB_INPUT_EPWMB) ||
           (input == EPWM_DB_INPUT_DB_RED));

    if(input == EPWM_DB_INPUT_DB_RED)
    {
        //
        // Set the Falling Edge Delay input
        //
        HWREGH(base + EPWM_O_DBCTL) |= EPWM_DBCTL_DEDB_MODE;
    }
    else
    {
        //
        // Set the Falling Edge Delay input
        //
        HWREGH(base + EPWM_O_DBCTL) &= ~EPWM_DBCTL_DEDB_MODE;

        //
        // Set the Rising Edge Delay input
        //
        HWREGH(base + EPWM_O_DBCTL) =
        ((HWREGH(base + EPWM_O_DBCTL) & ~(1U << (EPWM_DBCTL_IN_MODE_S + 1U))) |
         (input << (EPWM_DBCTL_IN_MODE_S + 1U)));
    }
}

//*****************************************************************************
//
//! Set the Dead Band control shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//! \param loadMode is the shadow to active load mode.
//!
//! This function enables and sets the Dead Band control register shadow
//! load mode.
//! Valid values for the \e loadMode parameter are:
//!     - EPWM_DB_LOAD_ON_CNTR_ZERO         - load when counter equals zero.
//!     - EPWM_DB_LOAD_ON_CNTR_PERIOD       - load when counter equals period.
//!     - EPWM_DB_LOAD_ON_CNTR_ZERO_PERIOD  - load when counter equals zero or
//!                                            period.
//!     - EPWM_DB_LOAD_FREEZE                - Freeze shadow to active load.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setDeadBandControlShadowLoadMode(uint32_t base,
                                      EPWM_DeadBandControlLoadMode loadMode)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Enable the shadow mode and setup the load event
    //
    HWREGH(base + EPWM_O_DBCTL2) =
          ((HWREGH(base + EPWM_O_DBCTL2) & ~EPWM_DBCTL2_LOADDBCTLMODE_M) |
           (EPWM_DBCTL2_SHDWDBCTLMODE | (uint16_t)loadMode));
}

//*****************************************************************************
//
//! Disable Dead Band control shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the Dead Band control register shadow
//! load mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableDeadBandControlShadowLoadMode(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Disable the shadow load mode. Only immediate load mode only.
    //
    HWREGH(base + EPWM_O_DBCTL2) =
                 (HWREGH(base + EPWM_O_DBCTL2) & ~EPWM_DBCTL2_SHDWDBCTLMODE);
}

//*****************************************************************************
//
//! Set the RED (Rising Edge Delay) shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//! \param loadMode is the shadow to active load event.
//!
//! This function sets the Rising Edge Delay register shadow load mode.
//! Valid values for the \e loadMode parameter are:
//!     - EPWM_RED_LOAD_ON_CNTR_ZERO        - load when counter equals zero.
//!     - EPWM_RED_LOAD_ON_CNTR_PERIOD      - load when counter equals period.
//!     - EPWM_RED_LOAD_ON_CNTR_ZERO_PERIOD - load when counter equals zero or
//!                                           period.
//!     - EPWM_RED_LOAD_FREEZE               - Freeze shadow to active load.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setRisingEdgeDelayCountShadowLoadMode(uint32_t base,
                                         EPWM_RisingEdgeDelayLoadMode loadMode)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Enable the shadow mode. Set-up the load mode
    //
    HWREGH(base + EPWM_O_DBCTL) =
               ((HWREGH(base + EPWM_O_DBCTL) & ~EPWM_DBCTL_LOADREDMODE_M) |
                ((uint16_t)EPWM_DBCTL_SHDWDBREDMODE |
                ((uint16_t)loadMode << EPWM_DBCTL_LOADREDMODE_S)));

}

//*****************************************************************************
//
//! Disable the RED (Rising Edge Delay) shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the Rising Edge Delay register shadow load mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableRisingEdgeDelayCountShadowLoadMode(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Disable the shadow mode.
    //
    HWREGH(base + EPWM_O_DBCTL) =
                   (HWREGH(base + EPWM_O_DBCTL) & ~EPWM_DBCTL_SHDWDBREDMODE);

}

//*****************************************************************************
//
//! Set the FED (Falling Edge Delay) shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//! \param loadMode is the shadow to active load event.
//!
//! This function enables and sets the Falling Edge Delay register shadow load
//! mode. Valid values for the \e loadMode parameters are:
//!     - EPWM_FED_LOAD_ON_CNTR_ZERO        - load when counter equals zero.
//!     - EPWM_FED_LOAD_ON_CNTR_PERIOD      - load when counter equals period.
//!     - EPWM_FED_LOAD_ON_CNTR_ZERO_PERIOD - load when counter equals zero or
//!                                            period.
//!     - EPWM_FED_LOAD_FREEZE               - Freeze shadow to active load.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setFallingEdgeDelayCountShadowLoadMode(uint32_t base,
                                        EPWM_FallingEdgeDelayLoadMode loadMode)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Enable the shadow mode. Setup the load mode.
    //
    HWREGH(base + EPWM_O_DBCTL) =
            ((HWREGH(base + EPWM_O_DBCTL) & ~EPWM_DBCTL_LOADFEDMODE_M) |
                (EPWM_DBCTL_SHDWDBFEDMODE |
                ((uint16_t)loadMode << EPWM_DBCTL_LOADFEDMODE_S)));

}

//*****************************************************************************
//
//! Disables the FED (Falling Edge Delay) shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the Falling Edge Delay register shadow load mode.
//! Valid values for the parameters are:
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableFallingEdgeDelayCountShadowLoadMode(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Disable the shadow mode.
    //
    HWREGH(base + EPWM_O_DBCTL) =
              (HWREGH(base + EPWM_O_DBCTL) & ~EPWM_DBCTL_SHDWDBFEDMODE);
}

//*****************************************************************************
//
//! Sets Dead Band Counter clock rate.
//!
//! \param base is the base address of the EPWM module.
//! \param clockMode is the Dead Band counter clock mode.
//!
//! This function sets up the Dead Band counter clock rate with respect to
//! TBCLK (ePWM time base counter).
//! Valid values for clockMode are:
//!   - EPWM_DB_COUNTER_CLOCK_FULL_CYCLE  -Dead band counter runs at TBCLK
//!                                           (ePWM Time Base Counter) rate.
//!   - EPWM_DB_COUNTER_CLOCK_HALF_CYCLE  -Dead band counter runs at 2*TBCLK
//!                                         (twice ePWM Time Base Counter)rate.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setDeadBandCounterClock(uint32_t base,
                             EPWM_DeadBandClockMode clockMode)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set the DB clock mode
    //
    HWREGH(base + EPWM_O_DBCTL) =
                ((HWREGH(base + EPWM_O_DBCTL) & ~EPWM_DBCTL_HALFCYCLE) |
                 ((uint16_t)clockMode << 15U));
}

//*****************************************************************************
//
//! Set ePWM RED count
//!
//! \param base is the base address of the EPWM module.
//! \param redCount is the RED(Rising Edge Delay) count.
//!
//! This function sets the RED (Rising Edge Delay) count value.
//! The value of redCount should be less than 0x4000U.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setRisingEdgeDelayCount(uint32_t base, uint16_t redCount)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(redCount < 0x4000U);

    //
    // Set the RED (Rising Edge Delay) count
    //
    HWREGH(base + EPWM_O_DBRED) = redCount;
}

//*****************************************************************************
//
//! Set ePWM FED count
//!
//! \param base is the base address of the EPWM module.
//! \param fedCount is the FED(Falling Edge Delay) count.
//!
//! This function sets the FED (Falling Edge Delay) count value.
//! The value of fedCount should be less than 0x4000U.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setFallingEdgeDelayCount(uint32_t base, uint16_t fedCount)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(fedCount < 0x4000U);

    //
    // Set the RED (Rising Edge Delay) count
    //
    HWREGH(base + EPWM_O_DBFED) = fedCount;
}

//
// Chopper module related APIs
//
//*****************************************************************************
//
//! Enable chopper mode
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables ePWM chopper module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableChopper(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set CHPEN bit. Enable Chopper
    //
    HWREGH(base + EPWM_O_PCCTL) |= EPWM_PCCTL_CHPEN;
}

//*****************************************************************************
//
//! Disable chopper mode
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables ePWM chopper module.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableChopper(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Clear CHPEN bit. Disable Chopper
    //
    HWREGH(base + EPWM_O_PCCTL) &= ~EPWM_PCCTL_CHPEN;
}

//*****************************************************************************
//
//! Set chopper duty cycle.
//!
//! \param base is the base address of the EPWM module.
//! \param dutyCycleCount is the chopping clock duty cycle count.
//!
//! This function sets the chopping clock duty cycle. The value of
//! dutyCycleCount should be less than 7. The dutyCycleCount value is converted
//! to the actual chopper duty cycle value base on the following equation:
//!   chopper duty cycle = (dutyCycleCount + 1) / 8
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setChopperDutyCycle(uint32_t base, uint16_t dutyCycleCount)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(dutyCycleCount < 7U);

    //
    // Set the chopper duty cycle
    //
    HWREGH(base + EPWM_O_PCCTL) =
               ((HWREGH(base + EPWM_O_PCCTL) & ~EPWM_PCCTL_CHPDUTY_M) |
                (dutyCycleCount << EPWM_PCCTL_CHPDUTY_S));
}

//*****************************************************************************
//
//! Set chopper clock frequency scaler.
//!
//! \param base is the base address of the EPWM module.
//! \param freqDiv is the chopping clock frequency divider.
//!
//! This function sets the scaler for the chopping clock frequency. The value
//! of freqDiv should be less than 8. The chopping clock frequency is altered
//! based on the following equation.
//!   chopper clock frequency = SYSCLKOUT / ( 1 + freqDiv)
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setChopperFreq(uint32_t base, uint16_t freqDiv)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(freqDiv < 8U);

    //
    // Set the chopper clock
    //
    HWREGH(base + EPWM_O_PCCTL) =
                    ((HWREGH(base + EPWM_O_PCCTL) &
                      ~(uint16_t)EPWM_PCCTL_CHPFREQ_M) |
                       (freqDiv << EPWM_PCCTL_CHPFREQ_S));
}

//*****************************************************************************
//
//! Set chopper clock frequency scaler.
//!
//! \param base is the base address of the EPWM module.
//! \param firstPulseWidth is the width of the first pulse.
//!
//! This function sets the first pulse width of chopper output waveform. The
//! value of firstPulseWidth should be less than 0x10. The value of the first
//! pulse width in seconds is given using the following equation:
//!     first pulse width = 1 / (((firstPulseWidth + 1) * SYSCLKOUT)/8)
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setChopperFirstPulseWidth(uint32_t base, uint16_t firstPulseWidth)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(firstPulseWidth < 16U);

    //
    // Set the chopper clock
    //
    HWREGH(base + EPWM_O_PCCTL) =
              ((HWREGH(base + EPWM_O_PCCTL) &
               ~(uint16_t)EPWM_PCCTL_OSHTWTH_M) |
               (firstPulseWidth << EPWM_PCCTL_OSHTWTH_S));
}

//
// Trip Zone module related APIs
//
//*****************************************************************************
//
//! Enables Trip Zone signal.
//!
//! \param base is the base address of the EPWM module.
//! \param tzSignal is the Trip Zone signal.
//!
//! This function enables the Trip Zone signals specified by tzSignal as a
//! source for the Trip Zone module.
//! Valid values for tzSignal are:
//!   - EPWM_TZ_SIGNAL_CBC1       - TZ1 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC2       - TZ2 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC3       - TZ3 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC4       - TZ4 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC5       - TZ5 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC6       - TZ6 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_DCAEVT2    - DCAEVT2 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_DCBEVT2    - DCBEVT2 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_OSHT1      - One-shot TZ1
//!   - EPWM_TZ_SIGNAL_OSHT2      - One-shot TZ2
//!   - EPWM_TZ_SIGNAL_OSHT3      - One-shot TZ3
//!   - EPWM_TZ_SIGNAL_OSHT4      - One-shot TZ4
//!   - EPWM_TZ_SIGNAL_OSHT5      - One-shot TZ5
//!   - EPWM_TZ_SIGNAL_OSHT6      - One-shot TZ6
//!   - EPWM_TZ_SIGNAL_DCAEVT1    - One-shot DCAEVT1
//!   - EPWM_TZ_SIGNAL_DCBEVT1    - One-shot DCBEVT1
//!
//! \b note:  A logical OR of the valid values can be passed as the tzSignal
//!           parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableTripZoneSignals(uint32_t base, uint16_t tzSignal)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set the trip zone bits
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZSEL) |= tzSignal;
    EDIS;
}

//*****************************************************************************
//
//! Disables Trip Zone signal.
//!
//! \param base is the base address of the EPWM module.
//! \param tzSignal is the Trip Zone signal.
//!
//! This function disables the Trip Zone signal specified by tzSignal as a
//! source for the Trip Zone module.
//! Valid values for tzSignal are:
//!   - EPWM_TZ_SIGNAL_CBC1       - TZ1 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC2       - TZ2 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC3       - TZ3 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC4       - TZ4 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC5       - TZ5 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_CBC6       - TZ6 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_DCAEVT2    - DCAEVT2 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_DCBEVT2    - DCBEVT2 Cycle By Cycle
//!   - EPWM_TZ_SIGNAL_OSHT1      - One-shot TZ1
//!   - EPWM_TZ_SIGNAL_OSHT2      - One-shot TZ2
//!   - EPWM_TZ_SIGNAL_OSHT3      - One-shot TZ3
//!   - EPWM_TZ_SIGNAL_OSHT4      - One-shot TZ4
//!   - EPWM_TZ_SIGNAL_OSHT5      - One-shot TZ5
//!   - EPWM_TZ_SIGNAL_OSHT6      - One-shot TZ6
//!   - EPWM_TZ_SIGNAL_DCAEVT1    - One-shot DCAEVT1
//!   - EPWM_TZ_SIGNAL_DCBEVT1    - One-shot DCBEVT1
//!
//! \b note:  A logical OR of the valid values can be passed as the tzSignal
//!           parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableTripZoneSignals(uint32_t base, uint16_t tzSignal)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Clear the trip zone bits
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZSEL) &= ~tzSignal;
    EDIS;
}

//*****************************************************************************
//
//! Set Digital compare conditions that cause Trip Zone event.
//!
//! \param base is the base address of the EPWM module.
//! \param dcType is the Digital compare output type.
//! \param dcEvent is the Digital Compare output event.
//!
//! This function sets up the Digital Compare output Trip Zone event sources.
//! The dcType variable specifies the event source to be whether Digital
//! Compare output A or Digital Compare output B. The dcEvent parameter
//! specifies the event that causes Trip Zone.
//! Valid values for the parameters are:
//!  - dcType
//!      - EPWM_TZ_DC_OUTPUT_A1     - Digital Compare output 1 A
//!      - EPWM_TZ_DC_OUTPUT_A2     - Digital Compare output 2 A
//!      - EPWM_TZ_DC_OUTPUT_B1     - Digital Compare output 1 B
//!      - EPWM_TZ_DC_OUTPUT_B2     - Digital Compare output 2 B
//!  - dcEvent
//!      - EPWM_TZ_EVENT_DC_DISABLED         - Event Trigger is disabled
//!      - EPWM_TZ_EVENT_DCXH_LOW            - Trigger event when DCxH low
//!      - EPWM_TZ_EVENT_DCXH_HIGH           - Trigger event when DCxH high
//!      - EPWM_TZ_EVENT_DCXL_LOW            - Trigger event when DCxL low
//!      - EPWM_TZ_EVENT_DCXL_HIGH           - Trigger event when DCxL high
//!      - EPWM_TZ_EVENT_DCXL_HIGH_DCXH_LOW  - Trigger event when DCxL high
//!                                            DCxH low
//!
//!  \note x in DCxH/DCxL represents DCAH/DCAL or DCBH/DCBL
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setTripZoneDigitalCompareEventCondition(uint32_t base,
                                 EPWM_TripZoneDigitalCompareOutput dcType,
                                 EPWM_TripZoneDigitalCompareOutputEvent dcEvent)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set Digital Compare Events conditions that cause a Digital Compare trip
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZDCSEL) =
           ((HWREGH(base + EPWM_O_TZDCSEL) & ~(0x7U << (uint16_t)dcType)) |
            ((uint16_t)dcEvent << (uint16_t)dcType));
    EDIS;
}

//*****************************************************************************
//
//! Enable advanced Trip Zone event Action.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the advanced actions of the Trip Zone events. The
//! advanced features combine the trip zone events with the direction of the
//! counter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableTripZoneAdvAction(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Enable Advanced feature. Set ETZE bit
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZCTL2) |= EPWM_TZCTL2_ETZE;
    EDIS;
}

//*****************************************************************************
//
//! Disable advanced Trip Zone event Action.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the advanced actions of the Trip Zone events.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableTripZoneAdvAction(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Disable Advanced feature. clear ETZE bit
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZCTL2) &= ~EPWM_TZCTL2_ETZE;
    EDIS;
}

//*****************************************************************************
//
//! Set Trip Zone Action.
//!
//! \param base is the base address of the EPWM module.
//! \param tzEvent is the Trip Zone event type.
//! \param tzAction is the Trip zone Action.
//!
//! This function sets the Trip Zone Action to be taken when a Trip Zone event
//! occurs.
//! Valid values for the parameters are:
//!  - tzEvent
//!      - EPWM_TZ_ACTION_EVENT_DCBEVT2  - DCBEVT2 (Digital Compare B event 2)
//!      - EPWM_TZ_ACTION_EVENT_DCBEVT1  - DCBEVT1 (Digital Compare B event 1)
//!      - EPWM_TZ_ACTION_EVENT_DCAEVT2  - DCAEVT2 (Digital Compare A event 2)
//!      - EPWM_TZ_ACTION_EVENT_DCAEVT1  - DCAEVT1 (Digital Compare A event 1)
//!      - EPWM_TZ_ACTION_EVENT_TZB      - TZ1 - TZ6, DCBEVT2, DCBEVT1
//!      - EPWM_TZ_ACTION_EVENT_TZA      - TZ1 - TZ6, DCAEVT2, DCAEVT1
//!  - tzAction
//!      - EPWM_TZ_ACTION_HIGH_Z         - high impedance output
//!      - EPWM_TZ_ACTION_HIGH           - high output
//!      - EPWM_TZ_ACTION_LOW            - low low
//!      - EPWM_TZ_ACTION_DISABLE        - disable action
//!
//! \note Disable the advanced Trip Zone event using
//!       EPWM_disableTripZoneAdvAction()  before calling this function.
//! \note This function operates on both ePWMA and ePWMB depending on the
//!       tzEvent parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setTripZoneAction(uint32_t base, EPWM_TripZoneEvent tzEvent,
                       EPWM_TripZoneAction tzAction)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set the Action for Trip Zone events
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZCTL) =
         ((HWREGH(base + EPWM_O_TZCTL) & ~(0x3U << (uint16_t)tzEvent)) |
          ((uint16_t)tzAction << (uint16_t)tzEvent)) ;
    EDIS;
}

//*****************************************************************************
//
//! Set Advanced Trip Zone Action.
//!
//! \param base is the base address of the EPWM module.
//! \param tzAdvEvent is the Trip Zone event type.
//! \param tzAdvAction is the Trip zone Action.
//!
//! This function sets the Advanced Trip Zone Action to be taken when an
//! advanced Trip Zone event occurs.
//!
//! Valid values for the parameters are:
//!  - tzAdvEvent
//!      - EPWM_TZ_ADV_ACTION_EVENT_TZB_D  - TZ1 - TZ6, DCBEVT2, DCBEVT1 while
//!                                             counting down
//!      - EPWM_TZ_ADV_ACTION_EVENT_TZB_U  - TZ1 - TZ6, DCBEVT2, DCBEVT1 while
//!                                             counting up
//!      - EPWM_TZ_ADV_ACTION_EVENT_TZA_D  - TZ1 - TZ6, DCAEVT2, DCAEVT1 while
//!                                             counting down
//!      - EPWM_TZ_ADV_ACTION_EVENT_TZA_U  - TZ1 - TZ6, DCAEVT2, DCAEVT1 while
//!                                             counting up
//!  - tzAdvAction
//!      - EPWM_TZ_ADV_ACTION_HIGH_Z    - high impedance output
//!      - EPWM_TZ_ADV_ACTION_HIGH      - high voltage state
//!      - EPWM_TZ_ADV_ACTION_LOW       - low voltage state
//!      - EPWM_TZ_ADV_ACTION_TOGGLE    - Toggle output
//!      - EPWM_TZ_ADV_ACTION_DISABLE   - disable action
//!
//! \note This function enables the advanced Trip Zone event.
//!
//! \note This function operates on both ePWMA and ePWMB depending on the
//!       tzAdvEvent  parameter.
//! \note Advanced Trip Zone events take into consideration the direction of
//!       the counter in addition to Trip Zone events.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setTripZoneAdvAction(uint32_t base, EPWM_TripZoneAdvancedEvent tzAdvEvent,
                          EPWM_TripZoneAdvancedAction tzAdvAction)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set the Advanced Action for Trip Zone events
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZCTL2) =
       ((HWREGH(base + EPWM_O_TZCTL2) & ~(0x7U << (uint16_t)tzAdvEvent)) |
        ((uint16_t)tzAdvAction << (uint16_t)tzAdvEvent));

    HWREGH(base + EPWM_O_TZCTL2) |= EPWM_TZCTL2_ETZE;
    EDIS;
}

//*****************************************************************************
//
//! Set Advanced Digital Compare Trip Zone Action on ePWMA.
//!
//! \param base is the base address of the EPWM module.
//! \param tzAdvDCEvent is the Digital Compare Trip Zone event type.
//! \param tzAdvDCAction is the Digital Compare Trip zone Action.
//!
//! This function sets the Digital Compare (DC) Advanced Trip Zone Action to be
//! taken on ePWMA when an advanced Digital Compare Trip Zone A event occurs.
//! Valid values for the parameters are:
//!  - tzAdvDCEvent
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D  - Digital Compare event A2 while
//!                                                 counting down
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U  - Digital Compare event A2 while
//!                                                 counting up
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D  - Digital Compare event A1 while
//!                                                 counting down
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U  - Digital Compare event A1 while
//!                                                 counting up
//!  - tzAdvDCAction
//!      - EPWM_TZ_ADV_ACTION_HIGH_Z    - high impedance output
//!      - EPWM_TZ_ADV_ACTION_HIGH      - high voltage state
//!      - EPWM_TZ_ADV_ACTION_LOW       - low voltage state
//!      - EPWM_TZ_ADV_ACTION_TOGGLE    - Toggle output
//!      - EPWM_TZ_ADV_ACTION_DISABLE   - disable action
//!
//! \note This function enables the advanced Trip Zone event.
//!
//! \note Advanced Trip Zone events take into consideration the direction of
//!       the counter in addition to Digital Compare Trip Zone events.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setTripZoneAdvDigitalCompareActionA(uint32_t base,
                              EPWM_TripZoneAdvDigitalCompareEvent tzAdvDCEvent,
                              EPWM_TripZoneAdvancedAction tzAdvDCAction)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set the Advanced Action for Trip Zone events
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZCTLDCA) =
      ((HWREGH(base + EPWM_O_TZCTLDCA) & ~(0x7U << (uint16_t)tzAdvDCEvent)) |
       ((uint16_t)tzAdvDCAction << (uint16_t)tzAdvDCEvent));

    HWREGH(base + EPWM_O_TZCTL2) |= EPWM_TZCTL2_ETZE;
    EDIS;
}

//*****************************************************************************
//
//! Set Advanced Digital Compare Trip Zone Action on ePWMB.
//!
//! \param base is the base address of the EPWM module.
//! \param tzAdvDCEvent is the Digital Compare Trip Zone event type.
//! \param tzAdvDCAction is the Digital Compare Trip zone Action.
//!
//! This function sets the Digital Compare (DC) Advanced Trip Zone Action to be
//! taken on ePWMB when an advanced Digital Compare Trip Zone B event occurs.
//! Valid values for the parameters are:
//!  - tzAdvDCEvent
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D  - Digital Compare event B2 while
//!                                                 counting down
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U  - Digital Compare event B2 while
//!                                                 counting up
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D  - Digital Compare event B1 while
//!                                                 counting down
//!      - EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U  - Digital Compare event B1 while
//!                                                 counting up
//!  - tzAdvDCAction
//!      - EPWM_TZ_ADV_ACTION_HIGH_Z    - high impedance output
//!      - EPWM_TZ_ADV_ACTION_HIGH      - high voltage state
//!      - EPWM_TZ_ADV_ACTION_LOW       - low voltage state
//!      - EPWM_TZ_ADV_ACTION_TOGGLE    - Toggle output
//!      - EPWM_TZ_ADV_ACTION_DISABLE   - disable action
//!
//! \note This function enables the advanced Trip Zone event.
//!
//! \note Advanced Trip Zone events take into consideration the direction of
//!       the counter in addition to Digital Compare Trip Zone events.
//!
//! \return None.
//
//*****************************************************************************
static inline void EPWM_setTripZoneAdvDigitalCompareActionB(uint32_t base,
                              EPWM_TripZoneAdvDigitalCompareEvent tzAdvDCEvent,
                              EPWM_TripZoneAdvancedAction tzAdvDCAction)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set the Advanced Action for Trip Zone events
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZCTLDCB) =
     ((HWREGH(base + EPWM_O_TZCTLDCB) & ~(0x7U << (uint16_t)tzAdvDCEvent)) |
      ((uint16_t)tzAdvDCAction << (uint16_t)tzAdvDCEvent));

    HWREGH(base + EPWM_O_TZCTL2) |= EPWM_TZCTL2_ETZE;
    EDIS;
}

//*****************************************************************************
//
//! Enable Trip Zone interrupts.
//!
//! \param base is the base address of the EPWM module.
//! \param tzInterrupt is the Trip Zone interrupt.
//!
//! This function enables the Trip Zone interrupts.
//! Valid values for tzInterrupt are:
//!   - EPWM_TZ_INTERRUPT_CBC     - Trip Zones Cycle By Cycle interrupt
//!   - EPWM_TZ_INTERRUPT_OST     - Trip Zones One Shot interrupt
//!   - EPWM_TZ_INTERRUPT_DCAEVT1 - Digital Compare A Event 1 interrupt
//!   - EPWM_TZ_INTERRUPT_DCAEVT2 - Digital Compare A Event 2 interrupt
//!   - EPWM_TZ_INTERRUPT_DCBEVT1 - Digital Compare B Event 1 interrupt
//!   - EPWM_TZ_INTERRUPT_DCBEVT2 - Digital Compare B Event 2 interrupt
//!
//! \b note:  A logical OR of the valid values can be passed as the tzInterrupt
//!           parameter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableTripZoneInterrupt(uint32_t base, uint16_t tzInterrupt)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT((tzInterrupt > 0U) && (tzInterrupt < 0x80U));

    //
    // Enable Trip zone interrupts
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZEINT) |= tzInterrupt;
    EDIS;
}

//*****************************************************************************
//
//! Disable Trip Zone interrupts.
//!
//! \param base is the base address of the EPWM module.
//! \param tzInterrupt is the Trip Zone interrupt.
//!
//! This function disables the Trip Zone interrupts.
//! Valid values for tzInterrupt are:
//!   - EPWM_TZ_INTERRUPT_CBC     - Trip Zones Cycle By Cycle interrupt
//!   - EPWM_TZ_INTERRUPT_OST    - Trip Zones One Shot interrupt
//!   - EPWM_TZ_INTERRUPT_DCAEVT1 - Digital Compare A Event 1 interrupt
//!   - EPWM_TZ_INTERRUPT_DCAEVT2 - Digital Compare A Event 2 interrupt
//!   - EPWM_TZ_INTERRUPT_DCBEVT1 - Digital Compare B Event 1 interrupt
//!   - EPWM_TZ_INTERRUPT_DCBEVT2 - Digital Compare B Event 2 interrupt
//!
//! \b note:  A logical OR of the valid values can be passed as the tzInterrupt
//!           parameter.
//!
//! \return None.
//
//***************************************************************************
static inline void
EPWM_disableTripZoneInterrupt(uint32_t base, uint16_t tzInterrupt)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT((tzInterrupt > 0U) && (tzInterrupt < 0x80U));

    //
    // Disable Trip zone interrupts
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZEINT) &= ~tzInterrupt;
    EDIS;
}

//*****************************************************************************
//
//! Gets the Trip Zone status flag
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the Trip Zone status flag.
//!
//! \return The function returns the following or the bitwise OR value
//!         of the following values.
//!         - EPWM_TZ_INTERRUPT    - Trip Zone interrupt was generated
//!                                  due to the following TZ events.
//!         - EPWM_TZ_FLAG_CBC     - Trip Zones Cycle By Cycle event status flag
//!         - EPWM_TZ_FLAG_OST     - Trip Zones One Shot event status flag
//!         - EPWM_TZ_FLAG_DCAEVT1 - Digital Compare A Event 1 status flag
//!         - EPWM_TZ_FLAG_DCAEVT2 - Digital Compare A Event 2 status flag
//!         - EPWM_TZ_FLAG_DCBEVT1 - Digital Compare B Event 1 status flag
//!         - EPWM_TZ_FLAG_DCBEVT2 - Digital Compare B Event 2 status flag
//
//***************************************************************************
static inline uint16_t
EPWM_getTripZoneFlagStatus(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return the Trip zone flag status
    //
    return(HWREGH(base + EPWM_O_TZFLG) & 0x7FU);
}

//*****************************************************************************
//
//! Gets the Trip Zone Cycle by Cycle flag status
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the specific Cycle by Cycle Trip Zone flag
//! status.
//!
//! \return The function returns the following values.
//!           - EPWM_TZ_CBC_FLAG_1     - CBC 1 status flag
//!           - EPWM_TZ_CBC_FLAG_2     - CBC 2 status flag
//!           - EPWM_TZ_CBC_FLAG_3     - CBC 3 status flag
//!           - EPWM_TZ_CBC_FLAG_4     - CBC 4 status flag
//!           - EPWM_TZ_CBC_FLAG_5     - CBC 5 status flag
//!           - EPWM_TZ_CBC_FLAG_6     - CBC 6 status flag
//!           - EPWM_TZ_CBC_FLAG_DCAEVT2  - CBC status flag for Digital compare
//!                                                event A2
//!           - EPWM_TZ_CBC_FLAG_DCBEVT2  - CBC status flag for Digital compare
//!                                                event B2
//
//***************************************************************************
static inline uint16_t
EPWM_getCycleByCycleTripZoneFlagStatus(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return the Cycle By Cycle Trip zone flag status
    //
    return(HWREGH(base + EPWM_O_TZCBCFLG) & 0xFFU);
}

//*****************************************************************************
//
//! Gets the Trip Zone One Shot flag status
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the specific One Shot Trip Zone flag status.
//!
//! \return The function returns the bitwise OR of the following flags.
//!           - EPWM_TZ_OST_FLAG_OST1     - OST status flag for OST1
//!           - EPWM_TZ_OST_FLAG_OST2     - OST status flag for OST2
//!           - EPWM_TZ_OST_FLAG_OST3     - OST status flag for OST3
//!           - EPWM_TZ_OST_FLAG_OST4     - OST status flag for OST4
//!           - EPWM_TZ_OST_FLAG_OST5     - OST status flag for OST5
//!           - EPWM_TZ_OST_FLAG_OST6     - OST status flag for OST6
//!           - EPWM_TZ_OST_FLAG_DCAEVT1  - OST status flag for Digital
//!                                                 compare event A1
//!           - EPWM_TZ_OST_FLAG_DCBEVT1  - OST status flag for Digital
//!                                                 compare event B1
//
//***************************************************************************
static inline uint16_t
EPWM_getOneShotTripZoneFlagStatus(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return the One Shot Trip zone flag status
    //
    return(HWREGH(base + EPWM_O_TZOSTFLG) & 0xFFU);
}

//*****************************************************************************
//
//! Set the Trip Zone CBC pulse clear event.
//!
//! \param base is the base address of the EPWM module.
//! \param clearEvent is the CBC trip zone clear event.
//!
//! This function set the event which automatically clears the
//! CBC (Cycle by Cycle) latch.
//! Valid values for clearEvent are:
//!   - EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO         - Clear CBC pulse when counter
//!                                                  equals zero
//!   - EPWM_TZ_CBC_PULSE_CLR_CNTR_PERIOD       - Clear CBC pulse when counter
//!                                                  equals period
//!   - EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO_PERIOD  - Clear CBC pulse when counter
//!                                                  equals zero or period
//!
//!  \return None.
//
//**************************************************************************
static inline void
EPWM_selectCycleByCycleTripZoneClearEvent(uint32_t base,
                                 EPWM_CycleByCycleTripZoneClearMode clearEvent)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set the Cycle by Cycle Trip Latch mode
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZCLR) =
                 ((HWREGH(base + EPWM_O_TZCLR) & ~EPWM_TZCLR_CBCPULSE_M) |
                  ((uint16_t)clearEvent << EPWM_TZCLR_CBCPULSE_S));
    EDIS;
}

//*****************************************************************************
//
//! Clear Trip Zone flag
//!
//! \param base is the base address of the EPWM module.
//! \param tzFlags is the Trip Zone flags.
//!
//! This function clears the Trip Zone flags
//! Valid values for tzFlags are:
//!   - EPWM_TZ_INTERRUPT    - Global Trip Zone interrupt flag
//!   - EPWM_TZ_FLAG_CBC     - Trip Zones Cycle By Cycle flag
//!   - EPWM_TZ_FLAG_OST     - Trip Zones One Shot flag
//!   - EPWM_TZ_FLAG_DCAEVT1 - Digital Compare A Event 1 flag
//!   - EPWM_TZ_FLAG_DCAEVT2 - Digital Compare A Event 2 flag
//!   - EPWM_TZ_FLAG_DCBEVT1 - Digital Compare B Event 1 flag
//!   - EPWM_TZ_FLAG_DCBEVT2 - Digital Compare B Event 2 flag
//!
//! \b note: A bitwise OR of the valid values can be passed as the tzFlags
//! parameter.
//!
//! \return None.
//
//***************************************************************************
static inline void
EPWM_clearTripZoneFlag(uint32_t base, uint16_t tzFlags)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(tzFlags < 0x80U);

    //
    // Clear Trip zone event flag
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZCLR) |= tzFlags;
    EDIS;
}

//*****************************************************************************
//
//! Clear the Trip Zone Cycle by Cycle flag.
//!
//! \param base is the base address of the EPWM module.
//! \param tzCBCFlags is the CBC flag to be cleared.
//!
//! This function clears the specific Cycle by Cycle Trip Zone flag.
//! The following are valid values for tzCBCFlags.
//!   - EPWM_TZ_CBC_FLAG_1     - CBC 1 flag
//!   - EPWM_TZ_CBC_FLAG_2     - CBC 2 flag
//!   - EPWM_TZ_CBC_FLAG_3     - CBC 3 flag
//!   - EPWM_TZ_CBC_FLAG_4     - CBC 4 flag
//!   - EPWM_TZ_CBC_FLAG_5     - CBC 5 flag
//!   - EPWM_TZ_CBC_FLAG_6     - CBC 6 flag
//!   - EPWM_TZ_CBC_FLAG_DCAEVT2  - CBC flag Digital compare
//!                                         event A2
//!   - EPWM_TZ_CBC_FLAG_DCBEVT2  - CBC flag Digital compare
//!                                         event B2
//!
//! \return None.
//
//***************************************************************************
static inline void
EPWM_clearCycleByCycleTripZoneFlag(uint32_t base, uint16_t tzCBCFlags)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(tzCBCFlags < 0x800U);

    //
    // Clear the Cycle By Cycle Trip zone flag
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZCBCCLR) |= tzCBCFlags;
    EDIS;
}

//*****************************************************************************
//
//! Clear the Trip Zone One Shot flag.
//!
//! \param base is the base address of the EPWM module.
//! \param tzOSTFlags is the OST flags to be cleared.
//!
//! This function clears the specific One Shot (OST) Trip Zone flag.
//! The following are valid values for tzOSTFlags.
//!  - EPWM_TZ_OST_FLAG_OST1      - OST flag for OST1
//!  - EPWM_TZ_OST_FLAG_OST2      - OST flag for OST2
//!  - EPWM_TZ_OST_FLAG_OST3      - OST flag for OST3
//!  - EPWM_TZ_OST_FLAG_OST4      - OST flag for OST4
//!  - EPWM_TZ_OST_FLAG_OST5      - OST flag for OST5
//!  - EPWM_TZ_OST_FLAG_OST6      - OST flag for OST6
//!  - EPWM_TZ_OST_FLAG_DCAEVT1   - OST flag for Digital compare event A1
//!  - EPWM_TZ_OST_FLAG_DCBEVT1   - OST flag for Digital compare event B1
//!
//! \return None.
//
//***************************************************************************
static inline void
EPWM_clearOneShotTripZoneFlag(uint32_t base, uint16_t tzOSTFlags)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(tzOSTFlags < 0x800U);

    //
    // Clear the Cycle By Cycle Trip zone flag
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZOSTCLR) |= tzOSTFlags;
    EDIS;
}

//*****************************************************************************
//
//! Force Trip Zone events.
//!
//! \param base is the base address of the EPWM module.
//! \param tzForceEvent is the forced Trip Zone event.
//!
//! This function forces a Trip Zone event.
//! Valid values for tzForceEvent are:
//!   - EPWM_TZ_FORCE_EVENT_CBC     - Force Trip Zones Cycle By Cycle event
//!   - EPWM_TZ_FORCE_EVENT_OST     - Force Trip Zones One Shot Event
//!   - EPWM_TZ_FORCE_EVENT_DCAEVT1 - Force Digital Compare A Event 1
//!   - EPWM_TZ_FORCE_EVENT_DCAEVT2 - Force Digital Compare A Event 2
//!   - EPWM_TZ_FORCE_EVENT_DCBEVT1 - Force Digital Compare B Event 1
//!   - EPWM_TZ_FORCE_EVENT_DCBEVT2 - Force Digital Compare B Event 2
//!
//! \return None.
//
//***************************************************************************
static inline void
EPWM_forceTripZoneEvent(uint32_t base, uint16_t tzForceEvent)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT((tzForceEvent & 0xFF81U)== 0U);

    //
    // Force a Trip Zone event
    //
    EALLOW;
    HWREGH(base + EPWM_O_TZFRC) |= tzForceEvent;
    EDIS;
}

//
// Event Trigger related APIs
//
//*****************************************************************************
//
//! Enable ePWM interrupt.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the ePWM interrupt.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableInterrupt(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Enable ePWM interrupt
    //
    HWREGH(base + EPWM_O_ETSEL) |= EPWM_ETSEL_INTEN;
}

//*****************************************************************************
//
//! disable ePWM interrupt.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the ePWM interrupt.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableInterrupt(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Disable ePWM interrupt
    //
    HWREGH(base + EPWM_O_ETSEL) &= ~EPWM_ETSEL_INTEN;
}

//*****************************************************************************
//
//! Sets the ePWM interrupt source.
//!
//! \param base is the base address of the EPWM module.
//! \param interruptSource is the ePWM interrupt source.
//!
//! This function sets the ePWM interrupt source.
//! Valid values for interruptSource are:
//!   - EPWM_INT_TBCTR_DISABLED       - Time-base counter is disabled
//!   - EPWM_INT_TBCTR_ZERO           - Time-base counter equal to zero
//!   - EPWM_INT_TBCTR_PERIOD         - Time-base counter equal to period
//!   - EPWM_INT_TBCTR_ZERO_OR_PERIOD - Time-base counter equal to zero or
//!                                     period
//!   - EPWM_INT_TBCTR_ZERO_OR_PERIOD - Time-base counter equal to zero or
//!                                     period
//!   - EPWM_INT_TBCTR_U_CMPx         - Where x is A,B,C or D
//!                                     Time-base counter equal to CMPA, CMPB,
//!                                     CMPC or CMPD (depending the value of x)
//!                                     when the timer is incrementing
//!   - EPWM_INT_TBCTR_D_CMPx         - Where x is A,B,C or D
//!                                     Time-base counter equal to CMPA, CMPB,
//!                                     CMPC or CMPD (depending the value of x)
//!                                     when the timer is decrementing
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setInterruptSource(uint32_t base, uint16_t interruptSource)
{
    uint16_t intSource;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(((interruptSource > 0U) && (interruptSource < 9U)) ||
           (interruptSource == 10U) || (interruptSource == 12U) ||
           (interruptSource == 14U));

    if((interruptSource == EPWM_INT_TBCTR_U_CMPC) ||
       (interruptSource == EPWM_INT_TBCTR_U_CMPD) ||
       (interruptSource == EPWM_INT_TBCTR_D_CMPC) ||
       (interruptSource == EPWM_INT_TBCTR_D_CMPD))
    {
          //
          // Shift the interrupt source by 1
          //
          intSource = interruptSource >> 1U;

          //
          // Enable events based on comp C or comp D
          //
          HWREGH(base + EPWM_O_ETSEL) |= EPWM_ETSEL_INTSELCMP;
    }
    else if((interruptSource == EPWM_INT_TBCTR_U_CMPA) ||
            (interruptSource == EPWM_INT_TBCTR_U_CMPB) ||
            (interruptSource == EPWM_INT_TBCTR_D_CMPA) ||
            (interruptSource == EPWM_INT_TBCTR_D_CMPB))
    {
        intSource = interruptSource;

        //
        // Enable events based on comp A or comp B
        //
        HWREGH(base + EPWM_O_ETSEL) &= ~EPWM_ETSEL_INTSELCMP;
    }
    else
    {
        intSource = interruptSource;
    }

    //
    // Set the interrupt source
    //
    HWREGH(base + EPWM_O_ETSEL) =
            ((HWREGH(base + EPWM_O_ETSEL) & ~EPWM_ETSEL_INTSEL_M) | intSource);
}

//*****************************************************************************
//
//! Sets the ePWM interrupt event counts.
//!
//! \param base is the base address of the EPWM module.
//! \param eventCount is the event count for interrupt scale
//!
//! This function sets the interrupt event count that determines the number of
//! events that have to occur before an interrupt is issued.
//! Maximum value for eventCount is 15.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setInterruptEventCount(uint32_t base, uint16_t eventCount)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(eventCount < 16U);

    //
    // Enable advanced feature of interrupt every up to 15 events
    //
    HWREGH(base + EPWM_O_ETPS) |= EPWM_ETPS_INTPSSEL;
    HWREGH(base + EPWM_O_ETINTPS) =
         ((HWREGH(base + EPWM_O_ETINTPS) & ~EPWM_ETINTPS_INTPRD2_M) |
           eventCount);
}

//*****************************************************************************
//
//! Return the interrupt status.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the ePWM interrupt status.
//! \b Note This function doesn't return the Trip Zone status.
//!
//! \return Returns true if ePWM interrupt was generated.
//!         Returns false if no interrupt was generated
//
//*****************************************************************************
static inline bool
EPWM_getEventTriggerInterruptStatus(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return INT bit of ETFLG register
    //
    return(((HWREGH(base + EPWM_O_ETFLG) & 0x1U) == 0x1U) ? true : false);
}

//*****************************************************************************
//
//! Clear interrupt flag.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function clears the ePWM interrupt flag.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_clearEventTriggerInterruptFlag(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Clear INT bit of ETCLR register
    //
    HWREGH(base + EPWM_O_ETCLR) |= EPWM_ETCLR_INT;
}

//*****************************************************************************
//
//! Enable Pre-interrupt count load.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the ePWM interrupt counter to be pre-interrupt loaded
//! with a count value.
//!
//! \note This is valid only for advanced/expanded interrupt mode
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableInterruptEventCountInit(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Enable interrupt event count initializing/loading
    //
    HWREGH(base + EPWM_O_ETCNTINITCTL) |= EPWM_ETCNTINITCTL_INTINITEN;
}

//*****************************************************************************
//
//! Disable interrupt count load.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the ePWM interrupt counter from being loaded with
//! pre-interrupt count value.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableInterruptEventCountInit(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Disable interrupt event count initializing/loading
    //
    HWREGH(base + EPWM_O_ETCNTINITCTL) &= ~EPWM_ETCNTINITCTL_INTINITEN;
}

//*****************************************************************************
//
//! Force a software pre interrupt event counter load.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function forces the ePWM interrupt counter to be loaded with the
//! contents set by EPWM_setPreInterruptEventCount().
//!
//! \note make sure the EPWM_enablePreInterruptEventCountLoad() function is
//!       is called before invoking this function.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_forceInterruptEventCountInit(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Load the Interrupt Event counter value
    //
    HWREGH(base + EPWM_O_ETCNTINITCTL) |= EPWM_ETCNTINITCTL_INTINITFRC;
}

//*****************************************************************************
//
//! Set interrupt count.
//!
//! \param base is the base address of the EPWM module.
//! \param eventCount is the ePWM interrupt count value.
//!
//! This function sets the ePWM interrupt count. eventCount is the value of the
//! pre-interrupt value that is to be loaded. The maximum value of eventCount
//! is 15.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setInterruptEventCountInitValue(uint32_t base, uint16_t eventCount)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(eventCount < 16U);

    //
    // Set the Pre-interrupt event count
    //
    HWREGH(base + EPWM_O_ETCNTINIT) =
         ((HWREGH(base + EPWM_O_ETCNTINIT) & ~EPWM_ETCNTINIT_INTINIT_M) |
          (uint16_t)(eventCount & 0xFU));
}

//*****************************************************************************
//
//! Get the interrupt count.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the ePWM interrupt event count.
//!
//! \return The interrupt event counts that have occurred.
//
//*****************************************************************************
static inline uint16_t
EPWM_getInterruptEventCount(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return the interrupt event count
    //
    return(((HWREGH(base + EPWM_O_ETINTPS) & EPWM_ETINTPS_INTCNT2_M) >>
             EPWM_ETINTPS_INTCNT2_S));
}

//*****************************************************************************
//
//! Force ePWM interrupt.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function forces an ePWM interrupt.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_forceEventTriggerInterrupt(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set INT bit of ETFRC register
    //
    HWREGH(base + EPWM_O_ETFRC) |= EPWM_ETFRC_INT;
}

//
// ADC SOC configuration related APIs
//
//*****************************************************************************
//
//! Enable ADC SOC event.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function enables the ePWM module to trigger an ADC SOC event.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableADCTrigger(uint32_t base, EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Enable an SOC
    //
    if(adcSOCType == EPWM_SOC_A)
    {
         HWREGH(base + EPWM_O_ETSEL) |= EPWM_ETSEL_SOCAEN;
    }
    else
    {
         HWREGH(base + EPWM_O_ETSEL) |= EPWM_ETSEL_SOCBEN;
    }
}

//*****************************************************************************
//
//! Disable ADC SOC event.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function disables the ePWM module from triggering an ADC SOC event.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableADCTrigger(uint32_t base, EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Disable an SOC
    //
    if(adcSOCType == EPWM_SOC_A)
    {
         HWREGH(base + EPWM_O_ETSEL) &= ~EPWM_ETSEL_SOCAEN;
    }
    else
    {
         HWREGH(base + EPWM_O_ETSEL) &= ~EPWM_ETSEL_SOCBEN;
    }
}

//*****************************************************************************
//
//! Sets the ePWM SOC source.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//! \param socSource is the SOC source.
//!
//! This function sets the ePWM ADC SOC source.
//! Valid values for socSource are:
//!  - adcSOCType
//!     - EPWM_SOC_A  - SOC A
//!     - EPWM_SOC_B  - SOC B
//!  - socSource
//!     - EPWM_SOC_DCxEVT1              - Event is based on DCxEVT1
//!     - EPWM_SOC_TBCTR_ZERO           - Time-base counter equal to zero
//!     - EPWM_SOC_TBCTR_PERIOD         - Time-base counter equal to period
//!     - EPWM_SOC_TBCTR_ZERO_OR_PERIOD - Time-base counter equal to zero or
//!                                       period
//!     - EPWM_SOC_TBCTR_U_CMPx         - Where x is A,B,C or D
//!                                       Time-base counter equal to CMPA, CMPB,
//!                                       CMPC or CMPD(depending the value of x)
//!                                       when the timer is incrementing
//!     - EPWM_SOC_TBCTR_D_CMPx         - Where x is A,B,C or D
//!                                       Time-base counter equal to CMPA, CMPB,
//!                                       CMPC or CMPD(depending the value of x)
//!                                       when the timer is decrementing
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setADCTriggerSource(uint32_t base,
                         EPWM_ADCStartOfConversionType adcSOCType,
                         EPWM_ADCStartOfConversionSource socSource)
{
    uint16_t source;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    if((socSource == EPWM_SOC_TBCTR_U_CMPC) ||
       (socSource == EPWM_SOC_TBCTR_U_CMPD) ||
       (socSource == EPWM_SOC_TBCTR_D_CMPC) ||
       (socSource == EPWM_SOC_TBCTR_D_CMPD))
    {
        source = (uint16_t)socSource >> 1U;
    }
    else
    {
        source = (uint16_t)socSource;
    }

    if(adcSOCType == EPWM_SOC_A)
    {
        //
        // Set the SOC source
        //
        HWREGH(base + EPWM_O_ETSEL) =
                ((HWREGH(base + EPWM_O_ETSEL) & ~EPWM_ETSEL_SOCASEL_M) |
                 (source << EPWM_ETSEL_SOCASEL_S));

        //
        // Enable the comparator selection
        //
        if((socSource == EPWM_SOC_TBCTR_U_CMPA) ||
           (socSource == EPWM_SOC_TBCTR_U_CMPB) ||
           (socSource == EPWM_SOC_TBCTR_D_CMPA) ||
           (socSource == EPWM_SOC_TBCTR_D_CMPB))
        {
            //
            // Enable events based on comp A or comp B
            //
            HWREGH(base + EPWM_O_ETSEL) &= ~EPWM_ETSEL_SOCASELCMP;
        }
        else if((socSource == EPWM_SOC_TBCTR_U_CMPC) ||
                (socSource == EPWM_SOC_TBCTR_U_CMPD) ||
                (socSource == EPWM_SOC_TBCTR_D_CMPC) ||
                (socSource == EPWM_SOC_TBCTR_D_CMPD))
        {
            //
            // Enable events based on comp C or comp D
            //
            HWREGH(base + EPWM_O_ETSEL) |= EPWM_ETSEL_SOCASELCMP;
        }
        else
        {
            //
            // No action required for the other socSource options
            //
        }
    }
    else
    {
        //
        // Enable the comparator selection
        //
        HWREGH(base + EPWM_O_ETSEL) =
                ((HWREGH(base + EPWM_O_ETSEL) & ~EPWM_ETSEL_SOCBSEL_M) |
                 (source << EPWM_ETSEL_SOCBSEL_S));

        //
        // Enable the comparator selection
        //
        if((socSource == EPWM_SOC_TBCTR_U_CMPA) ||
           (socSource == EPWM_SOC_TBCTR_U_CMPB) ||
           (socSource == EPWM_SOC_TBCTR_D_CMPA) ||
           (socSource == EPWM_SOC_TBCTR_D_CMPB))
        {
            //
            // Enable events based on comp A or comp B
            //
            HWREGH(base + EPWM_O_ETSEL) &= ~EPWM_ETSEL_SOCBSELCMP;
        }
        else if((socSource == EPWM_SOC_TBCTR_U_CMPC) ||
                (socSource == EPWM_SOC_TBCTR_U_CMPD) ||
                (socSource == EPWM_SOC_TBCTR_D_CMPC) ||
                (socSource == EPWM_SOC_TBCTR_D_CMPD))
        {
            //
            // Enable events based on comp C or comp D
            //
            HWREGH(base + EPWM_O_ETSEL) |= EPWM_ETSEL_SOCBSELCMP;
        }
        else
        {
            //
            // No action required for the other socSource options
            //
        }
    }
}

//*****************************************************************************
//
//! Sets the ePWM SOC event counts.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//! \param preScaleCount is the event count number.
//!
//! This function sets the SOC event count that determines the number of
//! events that have to occur before an SOC is issued.
//!  Valid values for the parameters are:
//!   - adcSOCType
//!       - EPWM_SOC_A  - SOC A
//!       - EPWM_SOC_B  - SOC B
//!   - preScaleCount
//!        - [1 - 15]  -  Generate SOC pulse every preScaleCount
//!                       up to 15 events.
//!
//! \note A preScaleCount value of 0 disables the prescale.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setADCTriggerEventPrescale(uint32_t base,
                                EPWM_ADCStartOfConversionType adcSOCType,
                                uint16_t preScaleCount)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(preScaleCount < 16U);

    //
    // Enable advanced feature of SOC every up to 15 events
    //
    HWREGH(base + EPWM_O_ETPS) |= EPWM_ETPS_SOCPSSEL;
    if(adcSOCType == EPWM_SOC_A)
    {
        //
        // Set the count for SOC A
        //
        HWREGH(base + EPWM_O_ETSOCPS) =
           ((HWREGH(base + EPWM_O_ETSOCPS) & ~EPWM_ETSOCPS_SOCAPRD2_M) |
             preScaleCount);
    }
    else
    {
        //
        // Set the count for SOC B
        //
        HWREGH(base + EPWM_O_ETSOCPS) =
             ((HWREGH(base + EPWM_O_ETSOCPS) & ~EPWM_ETSOCPS_SOCBPRD2_M) |
              (preScaleCount << EPWM_ETSOCPS_SOCBPRD2_S));
    }
}

//*****************************************************************************
//
//! Return the SOC event status.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function returns the ePWM SOC status.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \return Returns true if the selected adcSOCType SOC was generated.
//!         Returns false if the selected adcSOCType SOC was not generated.
//
//*****************************************************************************
static inline bool
EPWM_getADCTriggerFlagStatus(uint32_t base,
                             EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return the SOC A/ B status
    //
    return((((HWREGH(base + EPWM_O_ETFLG) >>
              ((uint16_t)adcSOCType + 2U)) & 0x1U) == 0x1U) ? true : false);
}

//*****************************************************************************
//
//! Clear SOC flag.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function clears the ePWM SOC flag.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_clearADCTriggerFlag(uint32_t base,
                         EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Clear SOC A/B bit of ETCLR register
    //
    HWREGH(base + EPWM_O_ETCLR) |= 1U << ((uint16_t)adcSOCType + 2U);
}

//*****************************************************************************
//
//! Enable Pre-SOC event count load.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function enables the ePWM SOC event counter which is set by the
//! EPWM_setADCTriggerEventCountInitValue() function to be loaded before
//! an SOC event.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \note This is valid only for advanced/expanded SOC mode
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableADCTriggerEventCountInit(uint32_t base,
                                    EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Enable SOC event count initializing/loading
    //
    HWREGH(base + EPWM_O_ETCNTINITCTL) |= 1U << ((uint16_t)adcSOCType + 14U);
}

//*****************************************************************************
//
//! Disable Pre-SOC event count load.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function disables the ePWM SOC event counter from being loaded before
//! an SOC event (only an SOC event causes an increment of the counter value).
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \note This is valid only for advanced/expanded SOC mode
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableADCTriggerEventCountInit(uint32_t base,
                                     EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Disable SOC event count initializing/loading
    //
    HWREGH(base + EPWM_O_ETCNTINITCTL) &=
                                  ~(1U << ((uint16_t)adcSOCType + 14U));
}

//*****************************************************************************
//
//! Force a software pre SOC event counter load.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type
//!
//! This function forces the ePWM SOC counter to be loaded with the
//! contents set by EPWM_setPreADCStartOfConversionEventCount().
//!
//! \note make sure the EPWM_enableADCTriggerEventCountInit()
//!       function is called before invoking this function.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_forceADCTriggerEventCountInit(uint32_t base,
                                   EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Load the Interrupt Event counter value
    //
    HWREGH(base + EPWM_O_ETCNTINITCTL) |= 1U << ((uint16_t)adcSOCType + 11U);
}

//*****************************************************************************
//
//! Set ADC Trigger count values.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//! \param eventCount is the ePWM interrupt count value.
//!
//! This function sets the ePWM ADC Trigger count values.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//! The eventCount has a maximum value of 15.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setADCTriggerEventCountInitValue(uint32_t base,
                                      EPWM_ADCStartOfConversionType adcSOCType,
                                      uint16_t eventCount)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(eventCount < 16U);

    //
    // Set the ADC Trigger event count
    //
    if(adcSOCType == EPWM_SOC_A)
    {
        HWREGH(base + EPWM_O_ETCNTINIT) =
            ((HWREGH(base + EPWM_O_ETCNTINIT) & ~EPWM_ETCNTINIT_SOCAINIT_M) |
             (uint16_t)(eventCount << EPWM_ETCNTINIT_SOCAINIT_S));
    }
    else
    {
        HWREGH(base + EPWM_O_ETCNTINIT) =
             ((HWREGH(base + EPWM_O_ETCNTINIT) & ~EPWM_ETCNTINIT_SOCBINIT_M) |
              (eventCount << EPWM_ETCNTINIT_SOCBINIT_S));
    }
}

//*****************************************************************************
//
//! Get the SOC event count.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function returns the ePWM SOC event count.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \return The SOC event counts that have occurred.
//
//*****************************************************************************
static inline uint16_t
EPWM_getADCTriggerEventCount(uint32_t base,
                             EPWM_ADCStartOfConversionType adcSOCType)
{
    uint16_t eventCount;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return the SOC event count
    //
    if(adcSOCType == EPWM_SOC_A)
    {
        eventCount = (HWREGH(base + EPWM_O_ETSOCPS) >>
                                         EPWM_ETSOCPS_SOCACNT2_S) & 0xFU;
    }
    else
    {
        eventCount = (HWREGH(base + EPWM_O_ETSOCPS) >>
                                        EPWM_ETSOCPS_SOCBCNT2_S) & 0xFU;
    }
    return(eventCount);
}

//*****************************************************************************
//
//! Force SOC event.
//!
//! \param base is the base address of the EPWM module.
//! \param adcSOCType is the ADC SOC type.
//!
//! This function forces an ePWM SOC event.
//! Valid values for adcSOCType are:
//!   - EPWM_SOC_A  - SOC A
//!   - EPWM_SOC_B  - SOC B
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_forceADCTrigger(uint32_t base, EPWM_ADCStartOfConversionType adcSOCType)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set SOC A/B bit of ETFRC register
    //
    HWREGH(base + EPWM_O_ETFRC) |= 1U << ((uint16_t)adcSOCType + 2U);
}

//
// Digital Compare module related APIs
//
//*****************************************************************************
//
//! Set the DC trip input.
//!
//! \param base is the base address of the EPWM module.
//! \param tripSource is the tripSource.
//! \param dcType is the Digital Compare type.
//!
//! This function sets the trip input to the Digital Compare (DC). For a given
//! dcType the function sets the tripSource to be the input to the DC.
//! Valid values for the parameter are:
//!  - tripSource
//!     - EPWM_DC_TRIP_TRIPINx - Trip x,where x ranges from 1 to 15 excluding 13
//!     - EPWM_DC_TRIP_COMBINATION - selects all the Trip signals whose input
//!                                  is enabled by the following function
//!                              EPWM_enableDigitalCompareTripCombinationInput()
//!  - dcType
//!     - EPWM_DC_TYPE_DCAH  - Digital Compare A High
//!     - EPWM_DC_TYPE_DCAL  - Digital Compare A Low
//!     - EPWM_DC_TYPE_DCBH  - Digital Compare B High
//!     - EPWM_DC_TYPE_DCBL  - Digital Compare B Low
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_selectDigitalCompareTripInput(uint32_t base,
                                   EPWM_DigitalCompareTripInput tripSource,
                                   EPWM_DigitalCompareType dcType)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set the DC trip input
    //
    EALLOW;
    HWREGH(base + EPWM_O_DCTRIPSEL) =
     ((HWREGH(base + EPWM_O_DCTRIPSEL) & ~(0xFU << ((uint16_t)dcType << 2U))) |
      ((uint16_t)tripSource << ((uint16_t)dcType << 2U)));
    EDIS;
}

//
// DCFILT
//
//*****************************************************************************
//
//! Enable DC filter blanking window.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the DC filter blanking window.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_enableDigitalCompareBlankingWindow(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Enable DC filter blanking window
    //
    EALLOW;
    HWREGH(base + EPWM_O_DCFCTL) |= EPWM_DCFCTL_BLANKE;
    EDIS;
}

//*****************************************************************************
//
//! Disable DC filter blanking window.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the DC filter blanking window.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_disableDigitalCompareBlankingWindow(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Disable DC filter blanking window
    //
    EALLOW;
    HWREGH(base + EPWM_O_DCFCTL) &= ~EPWM_DCFCTL_BLANKE;
    EDIS;
}

//*****************************************************************************
//
//! Enable Digital Compare Window inverse mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the Digital Compare Window inverse mode. This will
//! invert the blanking window.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_enableDigitalCompareWindowInverseMode(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Enable DC window inverse mode.
    //
    EALLOW;
    HWREGH(base + EPWM_O_DCFCTL) |= EPWM_DCFCTL_BLANKINV;
    EDIS;
}

//*****************************************************************************
//
//! Disable Digital Compare Window inverse mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the Digital Compare Window inverse mode.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_disableDigitalCompareWindowInverseMode(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Disable DC window inverse mode.
    //
    EALLOW;
    HWREGH(base + EPWM_O_DCFCTL) &= ~EPWM_DCFCTL_BLANKINV;
    EDIS;
}

//*****************************************************************************
//
//! Set the Digital Compare filter blanking pulse.
//!
//! \param base is the base address of the EPWM module.
//! \param blankingPulse is Pulse that starts blanking window.
//!
//! This function sets the input pulse that starts the Digital Compare blanking
//! window.
//! Valid values for blankingPulse are:
//!   - EPWM_DC_WINDOW_START_TBCTR_PERIOD - Time base counter equals period
//!   - EPWM_DC_WINDOW_START_TBCTR_ZERO   - Time base counter equals zero
//!   - EPWM_DC_WINDOW_START_TBCTR_ZERO_PERIOD - Time base counter equals zero
//!                                              or period.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareBlankingEvent(uint32_t base,
                                EPWM_DigitalCompareBlankingPulse blankingPulse)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set DC blanking event
    //
    EALLOW;
    HWREGH(base + EPWM_O_DCFCTL) =
            ((HWREGH(base + EPWM_O_DCFCTL) & ~EPWM_DCFCTL_PULSESEL_M) |
             ((uint16_t)((uint32_t)blankingPulse << EPWM_DCFCTL_PULSESEL_S)));
    EDIS;
}

//*****************************************************************************
//
//! Set up the Digital Compare filter input.
//!
//! \param base is the base address of the EPWM module.
//! \param filterInput is Digital Compare signal source.
//!
//! This function sets the signal input source that will be filtered by the
//! Digital Compare module.
//! Valid values for filterInput are:
//!   - EPWM_DC_WINDOW_SOURCE_DCAEVT1  - DC filter signal source is DCAEVT1
//!   - EPWM_DC_WINDOW_SOURCE_DCAEVT2  - DC filter signal source is DCAEVT2
//!   - EPWM_DC_WINDOW_SOURCE_DCBEVT1  - DC filter signal source is DCBEVT1
//!   - EPWM_DC_WINDOW_SOURCE_DCBEVT2  - DC filter signal source is DCBEVT2
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareFilterInput(uint32_t base,
                                  EPWM_DigitalCompareFilterInput filterInput)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set the signal source that will be filtered
    //
    EALLOW;
    HWREGH(base + EPWM_O_DCFCTL) =
                  ((HWREGH(base + EPWM_O_DCFCTL) & ~EPWM_DCFCTL_SRCSEL_M) |
                   ((uint16_t)filterInput));
    EDIS;
}

//
// DC Edge Filter
//
//*****************************************************************************
//
//! Enable Digital Compare Edge Filter.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the Digital Compare Edge filter to generate event
//! after configured number of edges.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_enableDigitalCompareEdgeFilter(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Enable DC Edge Filter
    //
    EALLOW;
    HWREGH(base + EPWM_O_DCFCTL) |= EPWM_DCFCTL_EDGEFILTSEL;
    EDIS;
}

//*****************************************************************************
//
//! Disable Digital Compare Edge Filter.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables the Digital Compare Edge filter.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_disableDigitalCompareEdgeFilter(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Disable DC Edge Filter
    //
    EALLOW;
    HWREGH(base + EPWM_O_DCFCTL) &= ~EPWM_DCFCTL_EDGEFILTSEL;
    EDIS;
}

//*****************************************************************************
//
//! Set the Digital Compare Edge Filter Mode.
//!
//! \param base is the base address of the EPWM module.
//! \param edgeMode is Digital Compare Edge filter mode.
//!
//! This function sets the Digital Compare Event filter mode. Valid values
//! for edgeMode are:
//!   - EPWM_DC_EDGEFILT_MODE_RISING  - DC edge filter mode is rising edge
//!   - EPWM_DC_EDGEFILT_MODE_FALLING - DC edge filter mode is falling edge
//!   - EPWM_DC_EDGEFILT_MODE_BOTH    - DC edge filter mode is both edges
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareEdgeFilterMode(uint32_t base,
                                     EPWM_DigitalCompareEdgeFilterMode edgeMode)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set DC Edge filter mode
    //
    EALLOW;
    HWREGH(base + EPWM_O_DCFCTL) =
        (HWREGH(base + EPWM_O_DCFCTL) & ~EPWM_DCFCTL_EDGEMODE_M) |
         ((uint16_t)edgeMode << EPWM_DCFCTL_EDGEMODE_S);
    EDIS;
}

//*****************************************************************************
//
//! Set the Digital Compare Edge Filter Edge Count.
//!
//! \param base is the base address of the EPWM module.
//! \param edgeCount is Digital Compare event filter count
//!
//! This function sets the Digital Compare Event filter Edge Count to generate
//! events. Valid values for edgeCount can be:
//!  - EPWM_DC_EDGEFILT_EDGECNT_0 - No edge is required to generate event
//!  - EPWM_DC_EDGEFILT_EDGECNT_1 - 1 edge is required for event generation
//!  - EPWM_DC_EDGEFILT_EDGECNT_2 - 2 edges are required for event generation
//!  - EPWM_DC_EDGEFILT_EDGECNT_3 - 3 edges are required for event generation
//!  - EPWM_DC_EDGEFILT_EDGECNT_4 - 4 edges are required for event generation
//!  - EPWM_DC_EDGEFILT_EDGECNT_5 - 5 edges are required for event generation
//!  - EPWM_DC_EDGEFILT_EDGECNT_6 - 6 edges are required for event generation
//!  - EPWM_DC_EDGEFILT_EDGECNT_7 - 7 edges are required for event generation
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareEdgeFilterEdgeCount(uint32_t base, uint16_t edgeCount)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set DC Edge filter edge count
    //
    EALLOW;
    HWREGH(base + EPWM_O_DCFCTL) = (HWREGH(base + EPWM_O_DCFCTL) &
                                    ~EPWM_DCFCTL_EDGECOUNT_M) |
                                   (edgeCount << EPWM_DCFCTL_EDGECOUNT_S);
    EDIS;
}

//*****************************************************************************
//
//! Returns the Digital Compare Edge Filter Edge Count.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the configured Digital Compare Edge filter edge
//! count required to generate events. It can return values from 0-7.
//!
//! \return Returns the configured DigitalCompare Edge filter edge count.
//
//*****************************************************************************
static inline uint16_t
EPWM_getDigitalCompareEdgeFilterEdgeCount(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return configured DC edge filter edge count
    //
    return((HWREGH(base + EPWM_O_DCFCTL) & EPWM_DCFCTL_EDGECOUNT_M) >>
           EPWM_DCFCTL_EDGECOUNT_S);
}

//*****************************************************************************
//
//! Returns the Digital Compare Edge filter captured edge count status.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the count of edges captured by Digital Compare Edge
//! filter. It can return values from 0-7.
//!
//! \return Returns the count of captured edges
//
//*****************************************************************************
static inline uint16_t
EPWM_getDigitalCompareEdgeFilterEdgeStatus(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return captured edge count by DC Edge filter
    //
    return((HWREGH(base + EPWM_O_DCFCTL) & EPWM_DCFCTL_EDGESTATUS_M) >>
           EPWM_DCFCTL_EDGESTATUS_S);
}

//*****************************************************************************
//
//! Set up the Digital Compare filter window offset
//!
//! \param base is the base address of the EPWM module.
//! \param windowOffsetCount is blanking window offset length.
//!
//! This function sets the offset between window start pulse and blanking
//! window in TBCLK count.
//! The function take a 16bit count value for the offset value.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareWindowOffset(uint32_t base, uint16_t windowOffsetCount)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set the blanking window offset in TBCLK counts
    //
    HWREGH(base + EPWM_O_DCFOFFSET) = windowOffsetCount;
}

//*****************************************************************************
//
//! Set up the Digital Compare filter window length
//!
//! \param base is the base address of the EPWM module.
//! \param windowLengthCount is blanking window length.
//!
//! This function sets up the Digital Compare filter blanking window length in
//! TBCLK count.The function takes a 16bit count value for the window length.
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareWindowLength(uint32_t base, uint16_t windowLengthCount)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set the blanking window length in TBCLK counts
    //
    HWREGH(base + EPWM_O_DCFWINDOW) = windowLengthCount;
}

//*****************************************************************************
//
//! Return DC filter blanking window offset count.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns DC filter blanking window offset count.
//!
//! \return None
//
//*****************************************************************************
static inline uint16_t
EPWM_getDigitalCompareBlankingWindowOffsetCount(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return the Blanking Window Offset count
    //
    return(HWREGH(base + EPWM_O_DCFOFFSETCNT));
}

//*****************************************************************************
//
//! Return DC filter blanking window length count.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns DC filter blanking window length count.
//!
//! \return None
//
//*****************************************************************************
static inline uint16_t
EPWM_getDigitalCompareBlankingWindowLengthCount(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return the Blanking Window Length count
    //
    return(HWREGH(base + EPWM_O_DCFWINDOWCNT));
}

//*****************************************************************************
//
//! Set up the Digital Compare Event source.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//! \param dcEvent is the Digital Compare Event number.
//! \param dcEventSource is the - Digital Compare Event source.
//!
//! This function sets up the Digital Compare module Event sources.
//! The following are valid values for the parameters.
//!  - dcModule
//!      - EPWM_DC_MODULE_A  - Digital Compare Module A
//!      - EPWM_DC_MODULE_B  - Digital Compare Module B
//!  - dcEvent
//!      - EPWM_DC_EVENT_1   - Digital Compare Event number 1
//!      - EPWM_DC_EVENT_2   - Digital Compare Event number 2
//!  - dcEventSource
//!      - EPWM_DC_EVENT_SOURCE_FILT_SIGNAL  - signal source is filtered
//!            \note The signal source for this option is DCxEVTy, where the
//!                  value of x is dependent on dcModule and the value of y is
//!                  dependent on dcEvent. Possible signal sources are DCAEVT1,
//!                  DCBEVT1, DCAEVT2 or DCBEVT2 depending on the value of both
//!                  dcModule and dcEvent.
//!      - EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL  - signal source is unfiltered
//!                   The signal source for this option is DCxEVTy.
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareEventSource(uint32_t base,
                                  EPWM_DigitalCompareModule dcModule,
                                  EPWM_DigitalCompareEvent dcEvent,
                                  EPWM_DigitalCompareEventSource dcEventSource)
{
    uint32_t registerOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    registerOffset = EPWM_O_DCACTL + (uint32_t)dcModule;

    //
    // Set the DC event 1 source source
    //
    EALLOW;
    if(dcEvent == EPWM_DC_EVENT_1)
    {
        HWREGH(base + registerOffset) =
                ((HWREGH(base + registerOffset) & ~EPWM_DCACTL_EVT1SRCSEL) |
                 (uint16_t)dcEventSource);
    }
    else
    {
        HWREGH(base + registerOffset) =
                 ((HWREGH(base + registerOffset) & ~EPWM_DCACTL_EVT2SRCSEL) |
                  ((uint16_t)dcEventSource << 8U));
    }
    EDIS;
}

//*****************************************************************************
//
//! Set up the Digital Compare input sync mode.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//! \param dcEvent is the Digital Compare Event number.
//! \param syncMode is the Digital Compare Event sync mode.
//!
//! This function sets up the Digital Compare module Event sources.
//! The following are valid values for the parameters.
//!  - dcModule
//!      - EPWM_DC_MODULE_A  - Digital Compare Module A
//!      - EPWM_DC_MODULE_B  - Digital Compare Module B
//!  - dcEvent
//!      - EPWM_DC_EVENT_1   - Digital Compare Event number 1
//!      - EPWM_DC_EVENT_2   - Digital Compare Event number 2
//!  - syncMode
//!      - EPWM_DC_EVENT_INPUT_SYNCED      - DC input signal is synced with
//!                                          TBCLK
//!      - EPWM_DC_EVENT_INPUT_NOT SYNCED  - DC input signal is not synced with
//!                                          TBCLK
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareEventSyncMode(uint32_t base,
                                    EPWM_DigitalCompareModule dcModule,
                                    EPWM_DigitalCompareEvent dcEvent,
                                    EPWM_DigitalCompareSyncMode syncMode)
{
    uint32_t registerOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    registerOffset = EPWM_O_DCACTL + (uint32_t)dcModule;

    //
    // Set the DC event sync mode
    //
    EALLOW;
    if(dcEvent == EPWM_DC_EVENT_1)
    {
        HWREGH(base + registerOffset) =
           ((HWREGH(base + registerOffset) & ~EPWM_DCACTL_EVT1FRCSYNCSEL) |
            ((uint16_t)syncMode << 1U));
    }
    else
    {
        HWREGH(base + registerOffset) =
            ((HWREGH(base + registerOffset) & ~EPWM_DCACTL_EVT2FRCSYNCSEL) |
             ((uint16_t)syncMode << 9U));
    }
    EDIS;
}

//*****************************************************************************
//
//! Enable Digital Compare to generate Start of Conversion.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//!
//! This function enables the Digital Compare Event 1 to generate Start of
//! Conversion.
//! The following are valid values for the \e dcModule parameter.
//!     - EPWM_DC_MODULE_A  - Digital Compare Module A
//!     - EPWM_DC_MODULE_B  - Digital Compare Module B
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_enableDigitalCompareADCTrigger(uint32_t base,
                                    EPWM_DigitalCompareModule dcModule)
{
    uint32_t registerOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    registerOffset = EPWM_O_DCACTL + (uint32_t)dcModule;

    //
    // Enable Digital Compare start of conversion generation
    //
    EALLOW;
    HWREGH(base + registerOffset) =
                       (HWREGH(base + registerOffset) | EPWM_DCACTL_EVT1SOCE);
    EDIS;
}

//*****************************************************************************
//
//! Disable Digital Compare from generating Start of Conversion.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//!
//! This function disables the Digital Compare Event 1 from generating Start of
//! Conversion.
//! The following are valid values for the \e dcModule parameter.
//!     - EPWM_DC_MODULE_A  - Digital Compare Module A
//!     - EPWM_DC_MODULE_B  - Digital Compare Module B
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_disableDigitalCompareADCTrigger(uint32_t base,
                                     EPWM_DigitalCompareModule dcModule)
{
    uint32_t registerOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    registerOffset = EPWM_O_DCACTL + (uint32_t)dcModule;

    //
    // Disable Digital Compare start of conversion generation
    //
    EALLOW;
    HWREGH(base + registerOffset) =
                      (HWREGH(base + registerOffset) & ~EPWM_DCACTL_EVT1SOCE);
    EDIS;
}

//*****************************************************************************
//
//! Enable Digital Compare to generate sync out pulse.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//!
//! This function enables the Digital Compare Event 1 to generate sync out
//! pulse
//! The following are valid values for the \e dcModule parameter.
//!     - EPWM_DC_MODULE_A  - Digital Compare Module A
//!     - EPWM_DC_MODULE_B  - Digital Compare Module B
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_enableDigitalCompareSyncEvent(uint32_t base,
                                   EPWM_DigitalCompareModule dcModule)
{
    uint32_t registerOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    registerOffset = EPWM_O_DCACTL + (uint32_t)dcModule;

    //
    // Enable Digital Compare sync out pulse generation
    //
    EALLOW;
    HWREGH(base + registerOffset) =
                     (HWREGH(base + registerOffset) | EPWM_DCACTL_EVT1SYNCE);
    EDIS;
}

//*****************************************************************************
//
//! Disable Digital Compare from generating Start of Conversion.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//!
//! This function disables the Digital Compare Event 1 from generating synch
//! out pulse.
//! The following are valid values for the \e dcModule parameters.
//!     - EPWM_DC_MODULE_A  - Digital Compare Module A
//!     - EPWM_DC_MODULE_B  - Digital Compare Module B
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_disableDigitalCompareSyncEvent(uint32_t base,
                                    EPWM_DigitalCompareModule dcModule)
{
    uint32_t registerOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    registerOffset = EPWM_O_DCACTL + (uint32_t)dcModule;

    //
    // Disable Digital Compare sync out pulse generation
    //
    EALLOW;
    HWREGH(base + registerOffset) =
                      (HWREGH(base + registerOffset) & ~EPWM_DCACTL_EVT1SYNCE);
    EDIS;
}

//*****************************************************************************
//
//! Set up the Digital Compare CBC latch mode.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//! \param dcEvent is the Digital Compare Event number.
//! \param latchMode is the Digital Compare CBC latch mode.
//!
//! This function sets up the Digital Compare CBC latch mode.
//! The following are valid values for the parameters.
//!  - dcModule
//!      - EPWM_DC_MODULE_A  - Digital Compare Module A
//!      - EPWM_DC_MODULE_B  - Digital Compare Module B
//!  - dcEvent
//!      - EPWM_DC_EVENT_1   - Digital Compare Event number 1
//!      - EPWM_DC_EVENT_2   - Digital Compare Event number 2
//!  - latchMode
//!      - EPWM_DC_CBC_LATCH_DISABLED - DC cycle-by-cycle(CBC) latch is disabled
//!      - EPWM_DC_CBC_LATCH_ENABLED  - DC cycle-by-cycle(CBC) latch is enabled
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareCBCLatchMode(uint32_t base,
                                   EPWM_DigitalCompareModule dcModule,
                                   EPWM_DigitalCompareEvent dcEvent,
                                   EPWM_DigitalCompareCBCLatchMode latchMode)
{
    uint32_t registerOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    registerOffset = EPWM_O_DCACTL + (uint32_t)dcModule;

    //
    // Set the DC CBC Latch Mode
    //
    EALLOW;
    if(dcEvent == EPWM_DC_EVENT_1)
    {
        HWREGH(base + registerOffset) =
           ((HWREGH(base + registerOffset) & ~EPWM_DCACTL_EVT1LATSEL) |
            ((uint16_t)latchMode << 4U));
    }
    else
    {
        HWREGH(base + registerOffset) =
            ((HWREGH(base + registerOffset) & ~EPWM_DCACTL_EVT2LATSEL) |
             ((uint16_t)latchMode << 12U));
    }
    EDIS;
}

//*****************************************************************************
//
//! Sets the Digital Compare CBC latch clear event.
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//! \param dcEvent is the Digital Compare Event number.
//! \param clearEvent is the Digital Compare CBC latch clear event.
//!
//! This function sets the Digital Compare CBC latch clear event.
//! The following are valid values for the parameters.
//!  - dcModule
//!      - EPWM_DC_MODULE_A  - Digital Compare Module A
//!      - EPWM_DC_MODULE_B  - Digital Compare Module B
//!  - dcEvent
//!      - EPWM_DC_EVENT_1   - Digital Compare Event number 1
//!      - EPWM_DC_EVENT_2   - Digital Compare Event number 2
//!  - clearEvent
//!      - EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO - DC CBC latch is cleared when
//!                                          counter is zero
//!      - EPWM_DC_CBC_LATCH_CLR_ON_CNTR_PERIOD - DC CBC latch is cleared when
//!                                               counter is equal to period
//!      - EPWM_DC_CBC_LATCH_CLR_ON_CNTR_ZERO_PERIOD - DC CBC latch is cleared
//!                                                    when either counter is
//!                                                    zero or equal to period
//!
//! \return None
//
//*****************************************************************************
static inline void
EPWM_selectDigitalCompareCBCLatchClearEvent(uint32_t base,
                              EPWM_DigitalCompareModule dcModule,
                              EPWM_DigitalCompareEvent dcEvent,
                              EPWM_DigitalCompareCBCLatchClearEvent clearEvent)
{
    uint32_t registerOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    registerOffset = EPWM_O_DCACTL + (uint32_t)dcModule;

    //
    // Set the DC CBC Latch Clear Event
    //
    EALLOW;
    if(dcEvent == EPWM_DC_EVENT_1)
    {
        HWREGH(base + registerOffset) =
           ((HWREGH(base + registerOffset) & ~EPWM_DCACTL_EVT1LATCLRSEL_M) |
            ((uint16_t)clearEvent << EPWM_DCACTL_EVT1LATCLRSEL_S));
    }
    else
    {
        HWREGH(base + registerOffset) =
            ((HWREGH(base + registerOffset) & ~EPWM_DCACTL_EVT2LATCLRSEL_M) |
             ((uint16_t)clearEvent << EPWM_DCACTL_EVT2LATCLRSEL_S));
    }
    EDIS;
}

//*****************************************************************************
//
//! Gets the Digital Compare CBC latch status
//!
//! \param base is the base address of the EPWM module.
//! \param dcModule is the Digital Compare module.
//! \param dcEvent is the Digital Compare Event number.
//!
//! This function returns the Digital Compare module cycle-by-cycle(CBC) latch
//! status.
//! The following are valid values for the parameters.
//!  - dcModule
//!      - EPWM_DC_MODULE_A  - Digital Compare Module A
//!      - EPWM_DC_MODULE_B  - Digital Compare Module B
//!  - dcEvent
//!      - EPWM_DC_EVENT_1   - Digital Compare Event number 1
//!      - EPWM_DC_EVENT_2   - Digital Compare Event number 2
//!
//! \return Returns Digital Compare CBC latch status.
//! -\b true  - CBC latch is enabled
//! -\b false - CBC latch is disabled
//
//*****************************************************************************
static inline bool
EPWM_getDigitalCompareCBCLatchStatus(uint32_t base,
                                     EPWM_DigitalCompareModule dcModule,
                                     EPWM_DigitalCompareEvent dcEvent)
{
    uint32_t registerOffset;
    uint16_t status;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    registerOffset = EPWM_O_DCACTL + (uint32_t)dcModule;

    //
    // Get DC CBC Latch Clear Event
    //
    if(dcEvent == EPWM_DC_EVENT_1)
    {
        status = HWREGH(base + registerOffset) & EPWM_DCACTL_EVT1LAT;
    }
    else
    {
        status = HWREGH(base + registerOffset) & EPWM_DCACTL_EVT2LAT;
    }
    return(status != 0U);
}

//
// DC capture mode
//
//*****************************************************************************
//
//! Enables the Time Base Counter Capture controller.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables the time Base Counter Capture.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableDigitalCompareCounterCapture(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Enable Time base counter capture
    //
    EALLOW;
    HWREGH(base + EPWM_O_DCCAPCTL) |= EPWM_DCCAPCTL_CAPE;
    EDIS;
}

//*****************************************************************************
//
//! Disables the Time Base Counter Capture controller.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disable the time Base Counter Capture.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableDigitalCompareCounterCapture(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Disable Time base counter capture
    //
    EALLOW;
    HWREGH(base + EPWM_O_DCCAPCTL) &= ~EPWM_DCCAPCTL_CAPE;
    EDIS;
}

//*****************************************************************************
//
//! Set the Time Base Counter Capture mode.
//!
//! \param base is the base address of the EPWM module.
//! \param enableShadowMode is the shadow read mode flag.
//!
//! This function sets the mode the Time Base Counter value is read from. If
//! enableShadowMode is true, CPU reads of the DCCAP register will return the
//! shadow register contents.If enableShadowMode is false, CPU reads of the
//! DCCAP register will return the active register contents.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setDigitalCompareCounterShadowMode(uint32_t base, bool enableShadowMode)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    EALLOW;
    if(enableShadowMode)
    {
        //
        // Enable DC counter shadow mode
        //
        HWREGH(base + EPWM_O_DCCAPCTL) &= ~EPWM_DCCAPCTL_SHDWMODE;
    }
    else
    {
       //
       // Disable DC counter shadow mode
       //
       HWREGH(base + EPWM_O_DCCAPCTL) |= EPWM_DCCAPCTL_SHDWMODE;
    }
    EDIS;
}

//*****************************************************************************
//
//! Return the DC Capture event status.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the DC capture event status.
//!
//! \return Returns true if a DC capture event has occurs.
//!         Returns false if no DC Capture event has occurred.
//!
//! \return None.
//
//*****************************************************************************
static inline bool
EPWM_getDigitalCompareCaptureStatus(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return the DC compare status
    //
    return((HWREGH(base + EPWM_O_DCCAPCTL) & EPWM_DCCAPCTL_CAPSTS) ==
                                                        EPWM_DCCAPCTL_CAPSTS);
}

//*****************************************************************************
//
//! Return the DC Time Base Counter capture value.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the DC Time Base Counter capture value. The value
//! read is determined by the mode as set in the
//! EPWM_setTimeBaseCounterReadMode() function.
//!
//! \return Returns the DC Time Base Counter Capture count value.
//
//*****************************************************************************
static inline uint16_t
EPWM_getDigitalCompareCaptureCount(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return the DC Time Base Counter Capture count value
    //
    return(HWREGH(base + EPWM_O_DCCAP));
}

//*****************************************************************************
//
//! Enable DC TRIP combinational input.
//!
//! \param base is the base address of the EPWM module.
//! \param tripInput is the Trip number.
//! \param dcType is the Digital Compare module.
//!
//! This function enables the specified Trip input.
//! Valid values for the parameters are:
//!  - tripInput
//!      - EPWM_DC_COMBINATIONAL_TRIPINx, where x is 1,2,...12,14,15
//!  - dcType
//!      - EPWM_DC_TYPE_DCAH  - Digital Compare A High
//!      - EPWM_DC_TYPE_DCAL  - Digital Compare A Low
//!      - EPWM_DC_TYPE_DCBH  - Digital Compare B High
//!      - EPWM_DC_TYPE_DCBL  - Digital Compare B Low
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableDigitalCompareTripCombinationInput(uint32_t base,
                                              uint16_t tripInput,
                                              EPWM_DigitalCompareType dcType)
{
    uint32_t registerOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Get the DCAHTRIPSEL, DCALTRIPSEL, DCBHTRIPSEL, DCBLTRIPSEL register
    // offset with respect to DCAHTRIPSEL
    //
    registerOffset = EPWM_O_DCAHTRIPSEL + (uint32_t)dcType;

    //
    // Set the DC trip input
    //
    EALLOW;
    HWREGH(base + registerOffset) =
                           (HWREGH(base + registerOffset) | tripInput);

    //
    // Enable the combination input
    //
    HWREGH(base + EPWM_O_DCTRIPSEL) =
      (HWREGH(base + EPWM_O_DCTRIPSEL) | (0xFU << ((uint16_t)dcType << 2U)));
    EDIS;
}

//*****************************************************************************
//
//! Disable DC TRIP combinational input.
//!
//! \param base is the base address of the EPWM module.
//! \param tripInput is the Trip number.
//! \param dcType is the Digital Compare module.
//!
//! This function disables the specified Trip input.
//! Valid values for the parameters are:
//!  - tripInput
//!      - EPWM_DC_COMBINATIONAL_TRIPINx, where x is 1,2,...12,14,15
//!  - dcType
//!      - EPWM_DC_TYPE_DCAH  - Digital Compare A High
//!      - EPWM_DC_TYPE_DCAL  - Digital Compare A Low
//!      - EPWM_DC_TYPE_DCBH  - Digital Compare B High
//!      - EPWM_DC_TYPE_DCBL  - Digital Compare B Low
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableDigitalCompareTripCombinationInput(uint32_t base,
                                               uint16_t tripInput,
                                               EPWM_DigitalCompareType dcType)
{
    uint32_t registerOffset;

    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Get the DCAHTRIPSEL, DCALTRIPSEL, DCBHTRIPSEL, DCBLTRIPSEL register
    // offset with respect to DCAHTRIPSEL
    //
    registerOffset = EPWM_O_DCAHTRIPSEL + (uint32_t)dcType;

    //
    // Set the DC trip input
    //
    EALLOW;
    HWREGH(base + registerOffset) =
                           (HWREGH(base + registerOffset) & ~tripInput);
    EDIS;
}

//
// Valley switching
//
//*****************************************************************************
//
//! Enable valley capture mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables Valley Capture mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableValleyCapture(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set VCAPE bit
    //
    EALLOW;
    HWREGH(base + EPWM_O_VCAPCTL) |= EPWM_VCAPCTL_VCAPE;
    EDIS;
}

//*****************************************************************************
//
//! Disable valley capture mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables Valley Capture mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableValleyCapture(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Clear VCAPE bit
    //
    EALLOW;
    HWREGH(base + EPWM_O_VCAPCTL) &= ~EPWM_VCAPCTL_VCAPE;
    EDIS;
}

//*****************************************************************************
//
//! Start valley capture mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function starts Valley Capture sequence.
//!
//! \b Make sure you invoke EPWM_setValleyTriggerSource with the trigger
//!    variable set to EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE before calling this
//!    function.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_startValleyCapture(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set VCAPSTART bit
    //
    EALLOW;
    HWREGH(base + EPWM_O_VCAPCTL) |= EPWM_VCAPCTL_VCAPSTART;
    EDIS;
}

//*****************************************************************************
//
//! Set valley capture trigger.
//!
//! \param base is the base address of the EPWM module.
//! \param trigger is the Valley counter trigger.
//!
//! This function sets the trigger value that initiates Valley Capture sequence
//!
//! \b Set the number of Trigger source events for starting and stopping the
//!    valley capture using EPWM_setValleyTriggerEdgeCounts().
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setValleyTriggerSource(uint32_t base, EPWM_ValleyTriggerSource trigger)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Write to TRIGSEL bits
    //
    EALLOW;
    HWREGH(base + EPWM_O_VCAPCTL) =
             ((HWREGH(base + EPWM_O_VCAPCTL) & ~EPWM_VCAPCTL_TRIGSEL_M) |
              ((uint16_t)trigger << 2U));
    EDIS;
}

//*****************************************************************************
//
//! Set valley capture trigger source count.
//!
//! \param base is the base address of the EPWM module.
//! \param startCount
//! \param stopCount
//!
//! This function sets the number of trigger events required to start and stop
//! the valley capture count.
//! Maximum values for both startCount and stopCount is 15 corresponding to the
//! 15th edge of the trigger event.
//!
//! \b Note:
//!    A startCount value of 0 prevents starting the valley counter.
//!    A stopCount value of 0 prevents the valley counter from stopping.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setValleyTriggerEdgeCounts(uint32_t base, uint16_t startCount,
                                uint16_t stopCount)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT((startCount < 16U) && (stopCount < 16U));

    //
    // Write to STARTEDGE and STOPEDGE bits
    //
    EALLOW;
    HWREGH(base + EPWM_O_VCNTCFG) =
                    ((HWREGH(base + EPWM_O_VCNTCFG) &
                      ~(EPWM_VCNTCFG_STARTEDGE_M | EPWM_VCNTCFG_STOPEDGE_M)) |
                      (startCount | (stopCount << 8U)));
    EDIS;
}

//*****************************************************************************
//
//! Enable valley switching delay.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables Valley switching delay.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableValleyHWDelay(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set EDGEFILTDLYSEL bit
    //
    EALLOW;
    HWREGH(base + EPWM_O_VCAPCTL) |= EPWM_VCAPCTL_EDGEFILTDLYSEL;
    EDIS;
}

//*****************************************************************************
//
//! Disable valley switching delay.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables Valley switching delay.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableValleyHWDelay(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Clear EDGEFILTDLYSEL bit
    //
    EALLOW;
    HWREGH(base + EPWM_O_VCAPCTL) &= ~EPWM_VCAPCTL_EDGEFILTDLYSEL;
    EDIS;
}

//*****************************************************************************
//
//! Set Valley delay values.
//!
//! \param base is the base address of the EPWM module.
//! \param delayOffsetValue is the software defined delay offset value.
//!
//! This function sets the Valley delay value.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setValleySWDelayValue(uint32_t base, uint16_t delayOffsetValue)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Write to SWVDELVAL bits
    //
    HWREGH(base + EPWM_O_SWVDELVAL) = delayOffsetValue;
}

//*****************************************************************************
//
//! Set Valley delay mode.
//!
//! \param base is the base address of the EPWM module.
//! \param delayMode is the Valley delay mode.
//!
//! This function sets the Valley delay mode values.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setValleyDelayDivider(uint32_t base, EPWM_ValleyDelayMode delayMode)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Write to VDELAYDIV bits
    //
    EALLOW;
    HWREGH(base + EPWM_O_VCAPCTL) =
         ((HWREGH(base + EPWM_O_VCAPCTL) & ~EPWM_VCAPCTL_VDELAYDIV_M) |
          ((uint16_t)delayMode << 7U));
    EDIS;
}

//*****************************************************************************
//
//! Get the valley edge status bit.
//!
//! \param base is the base address of the EPWM module.
//! \param edge is the start or stop edge.
//!
//! This function returns the status of the start or stop valley status
//! depending on the value of edge.
//! If a start or stop edge has occurred, the function returns true, if not it
//! returns false.
//!
//! \return Returns true if the specified edge has occurred,
//!         Returns false if the specified edge has not occurred.
//
//*****************************************************************************
static inline bool
EPWM_getValleyEdgeStatus(uint32_t base, EPWM_ValleyCounterEdge edge)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    if(edge == EPWM_VALLEY_COUNT_START_EDGE)
    {
        //
        // Returns STARTEDGESTS status
        //
        return(((HWREGH(base + EPWM_O_VCNTCFG) & EPWM_VCNTCFG_STARTEDGESTS) ==
                   EPWM_VCNTCFG_STARTEDGESTS ) ? true : false);
    }
    else
    {
        //
        // Returns STOPEDGESTS status
        //
        return(((HWREGH(base + EPWM_O_VCNTCFG) & EPWM_VCNTCFG_STOPEDGESTS) ==
                EPWM_VCNTCFG_STOPEDGESTS) ? true : false);
    }
}

//*****************************************************************************
//
//! Get the Valley Counter value.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the valley time base count value which is captured
//! upon occurrence of the stop edge condition selected by
//! EPWM_setValleyTriggerSource() and by the stopCount variable of the
//! EPWM_setValleyTriggerEdgeCounts() function.
//!
//! \return Returns the valley base time count.
//
//*****************************************************************************
static inline uint16_t
EPWM_getValleyCount(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Read VCNTVAL register
    //
    return(HWREGH(base + EPWM_O_VCNTVAL));
}

//*****************************************************************************
//
//! Get the Valley delay value.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the hardware valley delay count.
//!
//! \return Returns the valley delay count.
//
//*****************************************************************************
static inline uint16_t
EPWM_getValleyHWDelay(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Read HWVDELVAL  register
    //
    return(HWREGH(base + EPWM_O_HWVDELVAL));
}

//*****************************************************************************
//
//! Enable Global shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables Global shadow to active load mode of registers.
//! The trigger source for loading shadow to active is determined by
//! EPWM_setGlobalLoadTrigger() function.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableGlobalLoad(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Shadow to active load is controlled globally
    //
    EALLOW;
    HWREGH(base + EPWM_O_GLDCTL) |= EPWM_GLDCTL_GLD;
    EDIS;
}

//*****************************************************************************
//
//! Disable Global shadow load mode.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function disables Global shadow to active load mode of registers.
//! Loading shadow to active is determined individually.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableGlobalLoad(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Shadow to active load is controlled individually
    //
    EALLOW;
    HWREGH(base + EPWM_O_GLDCTL) &= ~EPWM_GLDCTL_GLD;
    EDIS;
}

//*****************************************************************************
//
//! Set the Global shadow load pulse.
//!
//! \param base is the base address of the EPWM module.
//! \param loadTrigger is the pulse that causes global shadow load.
//!
//! This function sets the pulse that causes Global shadow to active load.
//! Valid values for the loadTrigger parameter are:
//!
//!   - EPWM_GL_LOAD_PULSE_CNTR_ZERO              - load when counter is equal
//!                                                 to zero
//!   - EPWM_GL_LOAD_PULSE_CNTR_PERIOD            - load when counter is equal
//!                                                 to period
//!   - EPWM_GL_LOAD_PULSE_CNTR_ZERO_PERIOD       - load when counter is equal
//!                                                 to zero or period
//!   - EPWM_GL_LOAD_PULSE_SYNC                    - load on sync event
//!   - EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_ZERO      - load on sync event or when
//!                                                 counter is equal to zero
//!   - EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_PERIOD    - load on sync event or when
//!                                                 counter is equal to period
//!   - EPWM_GL_LOAD_PULSE_SYNC_CNTR_ZERO_PERIOD  - load on sync event or when
//!                                                 counter is equal to period
//!                                                 or zero
//!   - EPWM_GL_LOAD_PULSE_GLOBAL_FORCE            - load on global force
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setGlobalLoadTrigger(uint32_t base, EPWM_GlobalLoadTrigger loadTrigger)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set the Global shadow to active load pulse
    //
    EALLOW;
    HWREGH(base + EPWM_O_GLDCTL) =
               ((HWREGH(base + EPWM_O_GLDCTL) & ~EPWM_GLDCTL_GLDMODE_M) |
                ((uint16_t)loadTrigger << EPWM_GLDCTL_GLDMODE_S));
    EDIS;
}

//*****************************************************************************
//
//! Set the number of Global load pulse event counts
//!
//! \param base is the base address of the EPWM module.
//! \param prescalePulseCount is the pulse event counts.
//!
//! This function sets the number of Global Load pulse events that have to
//! occurred before a global load pulse is issued. Valid values for
//! prescaleCount range from 0 to 7. 0 being no event (disables counter), and 7
//! representing 7 events.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setGlobalLoadEventPrescale(uint32_t base, uint16_t prescalePulseCount)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT(prescalePulseCount < 8U);

    //
    // Set the number of counts that have to occur before
    // a load strobe is issued
    //
    EALLOW;
    HWREGH(base + EPWM_O_GLDCTL) =
                ((HWREGH(base + EPWM_O_GLDCTL) & ~EPWM_GLDCTL_GLDPRD_M) |
                 (prescalePulseCount << EPWM_GLDCTL_GLDPRD_S));
    EDIS;
}

//*****************************************************************************
//
//! Return the number of Global load pulse event counts
//!
//! \param base is the base address of the EPWM module.
//!
//! This function returns the number of Global Load pulse events that have
//! occurred. These pulse events are set by the EPWM_setGlobalLoadTrigger()
//! function.
//!
//! \return None.
//
//*****************************************************************************
static inline uint16_t
EPWM_getGlobalLoadEventCount(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Return the number of events that have occurred
    //
    return((HWREGH(base + EPWM_O_GLDCTL) >> EPWM_GLDCTL_GLDCNT_S) & 0x7U);
}

//*****************************************************************************
//
//! Enable continuous global shadow to active load.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables global continuous shadow to active load. Register
//! load happens every time the event set by the
//! EPWM_setGlobalLoadTrigger() occurs.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableGlobalLoadOneShotMode(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Enable global continuous shadow to active load
    //
    EALLOW;
    HWREGH(base + EPWM_O_GLDCTL) &= ~EPWM_GLDCTL_OSHTMODE;
    EDIS;
}

//*****************************************************************************
//
//! Enable One shot global shadow to active load.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function enables a one time global shadow to active load. Register
//! load happens every time the event set by the
//! EPWM_setGlobalLoadTrigger() occurs.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableGlobalLoadOneShotMode(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Enable global continuous shadow to active load
    //
    EALLOW;
    HWREGH(base + EPWM_O_GLDCTL) |= EPWM_GLDCTL_OSHTMODE;
    EDIS;
}

//*****************************************************************************
//
//! Set One shot global shadow to active load pulse.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function sets a one time global shadow to active load pulse. The pulse
//! propagates to generate a load signal if any of the events set by
//! EPWM_setGlobalLoadTrigger() occur.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_setGlobalLoadOneShotLatch(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Set a one shot Global shadow load pulse.
    //
    EALLOW;
    HWREGH(base + EPWM_O_GLDCTL2) |= EPWM_GLDCTL2_OSHTLD;
    EDIS;
}

//*****************************************************************************
//
//! Force a software One shot global shadow to active load pulse.
//!
//! \param base is the base address of the EPWM module.
//!
//! This function forces a software a one time global shadow to active load
//! pulse.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_forceGlobalLoadOneShotEvent(uint32_t base)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Force a Software Global shadow load pulse
    //
    EALLOW;
    HWREGH(base + EPWM_O_GLDCTL2) |= EPWM_GLDCTL2_GFRCLD;
    EDIS;
}

//*****************************************************************************
//
//! Enable a register to be loaded Globally.
//!
//! \param base is the base address of the EPWM module.
//! \param loadRegister is the register.
//!
//! This function enables the register specified by loadRegister to be globally
//! loaded.
//! Valid values for loadRegister are:
//!  - EPWM_GL_REGISTER_TBPRD_TBPRDHR  - Register TBPRD:TBPRDHR
//!  - EPWM_GL_REGISTER_CMPA_CMPAHR    - Register CMPA:CMPAHR
//!  - EPWM_GL_REGISTER_CMPB_CMPBHR    - Register CMPB:CMPBHR
//!  - EPWM_GL_REGISTER_CMPC           - Register CMPC
//!  - EPWM_GL_REGISTER_CMPD           - Register CMPD
//!  - EPWM_GL_REGISTER_DBRED_DBREDHR  - Register DBRED:DBREDHR
//!  - EPWM_GL_REGISTER_DBFED_DBFEDHR  - Register DBFED:DBFEDHR
//!  - EPWM_GL_REGISTER_DBCTL          - Register DBCTL
//!  - EPWM_GL_REGISTER_AQCTLA_AQCTLA2 - Register AQCTLA/A2
//!  - EPWM_GL_REGISTER_AQCTLB_AQCTLB2 - Register AQCTLB/B2
//!  - EPWM_GL_REGISTER_AQCSFRC        - Register AQCSFRC
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_enableGlobalLoadRegisters(uint32_t base, uint16_t loadRegister)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT((loadRegister > 0x0000U) && (loadRegister < 0x0800U));

    //
    // The register specified by loadRegister is loaded globally
    //
    EALLOW;
    HWREGH(base + EPWM_O_GLDCFG) |= loadRegister;
    EDIS;
}

//*****************************************************************************
//
//! Disable a register to be loaded Globally.
//!
//! \param base is the base address of the EPWM module.
//! \param loadRegister is the register.
//!
//! This function disables the register specified by loadRegister from being
//! loaded globally. The shadow to active load happens as specified by the
//! register control
//! Valid values for loadRegister are:
//!  - EPWM_GL_REGISTER_TBPRD_TBPRDHR  - Register TBPRD:TBPRDHR
//!  - EPWM_GL_REGISTER_CMPA_CMPAHR    - Register CMPA:CMPAHR
//!  - EPWM_GL_REGISTER_CMPB_CMPBHR    - Register CMPB:CMPBHR
//!  - EPWM_GL_REGISTER_CMPC           - Register CMPC
//!  - EPWM_GL_REGISTER_CMPD           - Register CMPD
//!  - EPWM_GL_REGISTER_DBRED_DBREDHR  - Register DBRED:DBREDHR
//!  - EPWM_GL_REGISTER_DBFED_DBFEDHR  - Register DBFED:DBFEDHR
//!  - EPWM_GL_REGISTER_DBCTL          - Register DBCTL
//!  - EPWM_GL_REGISTER_AQCTLA_AQCTLA2 - Register AQCTLA/A2
//!  - EPWM_GL_REGISTER_AQCTLB_AQCTLB2 - Register AQCTLB/B2
//!  - EPWM_GL_REGISTER_AQCSFRC        - Register AQCSFRC
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_disableGlobalLoadRegisters(uint32_t base, uint16_t loadRegister)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));
    ASSERT((loadRegister > 0x0000U) && (loadRegister < 0x0800U));

    //
    // The register specified by loadRegister is loaded by individual
    // register configuration setting
    //
    EALLOW;
    HWREGH(base + EPWM_O_GLDCFG) &= ~loadRegister;
    EDIS;
}

//*****************************************************************************
//
//! Lock EALLOW protected register groups
//!
//! \param base is the base address of the EPWM module.
//! \param registerGroup is the EALLOW register groups.
//!
//! This functions locks the EALLOW protected register groups specified by
//! the registerGroup variable.
//!
//! \return None.
//
//*****************************************************************************
static inline void
EPWM_lockRegisters(uint32_t base, EPWM_LockRegisterGroup registerGroup)
{
    //
    // Check the arguments
    //
    ASSERT(EPWM_isBaseValid(base));

    //
    // Write the Key to EPWMLOCK register
    //
    HWREG(base + EPWM_O_LOCK) =
            (0xA5A50000UL | ((uint32_t)registerGroup));
}

//*****************************************************************************
//
//! Set emulation mode
//!
//! \param base is the base address of the EPWM module.
//! \param emulationMode is the emulation mode.
//!
//! This function sets the emulation behaviours of the time base counter. Valid
//! values for emulationMode are:
//!  - EPWM_EMULATION_STOP_AFTER_NEXT_TB    - Stop after next Time Base counter
//!                                           increment or decrement.
//!  - EPWM_EMULATION_STOP_AFTER_FULL_CYCLE - Stop when counter completes whole
//!                                           cycle.
//!  - EPWM_EMULATION_FREE_RUN               - Free run.
//!
//! \return None.
//
//*****************************************************************************
extern void
EPWM_setEmulationMode(uint32_t base, EPWM_EmulationMode emulationMode);

//*****************************************************************************
//
//! Configures ePWM signal with desired frequency & duty
//!
//! \param base is the base address of the EPWM module.
//! \param signalParams is the desired signal parameters.
//!
//! This function configures the ePWM module to generate a signal with
//! desired frequency & duty.
//!
//! \return None.
//
//*****************************************************************************
extern void
EPWM_configureSignal(uint32_t base, const EPWM_SignalParams *signalParams);

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

#endif // EPWM_H
