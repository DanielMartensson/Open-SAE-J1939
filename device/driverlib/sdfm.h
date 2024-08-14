//###########################################################################
//
// FILE:   sdfm.h
//
// TITLE:   C28x SDFM Driver
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
#ifndef SDFM_H
#define SDFM_H

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
//! \addtogroup sdfm_api SDFM
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_types.h"
#include "inc/hw_sdfm.h"
#include "inc/hw_memmap.h"
#include "cpu.h"
#include "debug.h"

//*****************************************************************************
//
// Defines for the API.
//
//*****************************************************************************
//! Macro to get the low threshold
//!
#define SDFM_GET_LOW_THRESHOLD(C)    ((uint16_t)(C))

//! Macro to get the high threshold
//!
#define SDFM_GET_HIGH_THRESHOLD(C)   ((uint16_t)((uint32_t)(C) >> 16U))

//! Macro to get the high threshold 1 & 2 to be passed as lowThreshold
//! parameter to SDFM_setCompFilterLowThreshold().
//!
#define SDFM_GET_LOW_THRESHOLD_BOTH(C1, C2)                                    \
                        ((((uint32_t)(SDFM_GET_LOW_THRESHOLD(C2))) << 16U) |   \
                         ((uint32_t)(SDFM_GET_LOW_THRESHOLD(C1))))

//! Macro to get the high threshold 1 & 2 to be passed as highThreshold
//! parameter to SDFM_setCompFilterHighThreshold().
//!
#define SDFM_GET_HIGH_THRESHOLD_BOTH(C1, C2)                                   \
                        ((((uint32_t)(SDFM_GET_HIGH_THRESHOLD(C2))) << 16U) |  \
                         ((uint32_t)(SDFM_GET_HIGH_THRESHOLD(C1))))

//! Macro to convert comparator over sampling ratio to acceptable bit location
//!
#define SDFM_SET_OSR(X)         (((X) - 1) << 8U)
//! Macro to convert the data shift bit values to acceptable bit location
//!
#define SDFM_SHIFT_VALUE(X)     ((X) << 2U)

//! Macro to combine high threshold and low threshold values
//!
#define SDFM_THRESHOLD(H, L)     ((((uint32_t)(H)) << 16U) | (L))

//! Macro to set the FIFO level to acceptable bit location
//!
#define SDFM_SET_FIFO_LEVEL(X)  ((X) << 7U)

//! Macro to set and enable the zero cross threshold value.
//!
#define SDFM_SET_ZERO_CROSS_THRESH_VALUE(X) (0x8000 | (X))

//! Macros to enable or disable filter.
//!
#define SDFM_FILTER_DISABLE     0x0U
#define SDFM_FILTER_ENABLE      0x2U

//*****************************************************************************
//
// Defines for SDFM register offsets. Added for internal use. Not to be used by
// application.
//
//*****************************************************************************
//!< SD filter offset
#define SDFM_SDFIL_OFFSET       (SDFM_O_SDCTLPARM2 - SDFM_O_SDCTLPARM1)

//!< Event Digital filter offset
#define SDFM_DIGFIL_OFFSET      (SDFM_O_SDCOMP2CTL - SDFM_O_SDCOMP1CTL)

//!< Offset between high threshold 1 & 2 registers
#define SDFM_SDFLT1CMPHx_OFFSET    (SDFM_O_SDFLT1CMPH2 - SDFM_O_SDFLT1CMPH1)

//!< Offset between low threshold 1 & 2 registers
#define SDFM_SDFLT1CMPLx_OFFSET    (SDFM_O_SDFLT1CMPL2 - SDFM_O_SDFLT1CMPL1)

//*****************************************************************************
//
// Define to mask out the bits in the SDCOMPHFILCTL register that aren't
// associated with comparator event filter configurations. Added for internal
// use, not to be used in application code.
//
//*****************************************************************************
#define SDFM_COMPEVT_FILTER_CONFIG_M  (SDFM_SDCOMP1EVT1FLTCTL_SAMPWIN_M  |     \
                                       SDFM_SDCOMP1EVT1FLTCTL_THRESH_M)

//*****************************************************************************
//
// Define to mask out the bits in the SDCOMPLOCK register that aren't
// associated with lock configuration.  Added for internal use, not to be used
// in application code.
//
//*****************************************************************************
#define SDFM_COMPEVT_FILTER_LOCK_M  (SDFM_SDCOMPLOCK_SDCOMPCTL |               \
                                     SDFM_SDCOMPLOCK_COMP)

//*****************************************************************************
//
// Values that can be passed to SDFM_enableSynchronizer() or
// SDFM_disableSynchronizer() as syncConfig parameter.
//
//*****************************************************************************
//! Define for Clock synchronizer Configuration
#define SDFM_CLOCK_SYNCHRONIZER    SDFM_SDCTLPARM1_SDCLKSYNC
//! Define for Data Synchronizer Configuration
#define SDFM_DATA_SYNCHRONIZER     SDFM_SDCTLPARM1_SDDATASYNC

//*****************************************************************************
//
//! Values that can be returned from SDFM_getThresholdStatus()
//
//*****************************************************************************
typedef enum
{
    SDFM_OUTPUT_WITHIN_THRESHOLD = 0,   //!< SDFM output is within threshold
    SDFM_OUTPUT_ABOVE_THRESHOLD  = 1,   //!< SDFM output is above threshold
    SDFM_OUTPUT_BELOW_THRESHOLD  = 2    //!< SDFM output is below threshold
} SDFM_OutputThresholdStatus;

//*****************************************************************************
//
//! Values that can be passed to all functions as the \e filterNumber
//! parameter.
//
//*****************************************************************************
typedef enum
{
    SDFM_FILTER_1 = 0, //!< Digital filter 1
    SDFM_FILTER_2 = 1, //!< Digital filter 2
    SDFM_FILTER_3 = 2, //!< Digital filter 3
    SDFM_FILTER_4 = 3  //!< Digital filter 4
} SDFM_FilterNumber;

//*****************************************************************************
//
//! Values that can be passed to SDFM_setFilterType(),
//! SDFM_setComparatorFilterType() as the \e filterType parameter.
//
//*****************************************************************************
typedef enum
{
    //! Digital filter with SincFast structure.
    SDFM_FILTER_SINC_FAST = 0x00,
    //! Digital filter with Sinc1 structure
    SDFM_FILTER_SINC_1    = 0x10,
    //! Digital filter with Sinc3 structure.
    SDFM_FILTER_SINC_2    = 0x20,
    //! Digital filter with Sinc4 structure.
    SDFM_FILTER_SINC_3    = 0x30
} SDFM_FilterType;

//*****************************************************************************
//
//! Values that can be passed to SDFM_setupModulatorClock(),as the
//! \e clockMode parameter.
//
//*****************************************************************************
typedef enum
{
   //! Modulator clock is identical to the data rate
   SDFM_MODULATOR_CLK_EQUAL_DATA_RATE  = 0,
} SDFM_ModulatorClockMode;

//*****************************************************************************
//
//! Values that can be passed to SDFM_setOutputDataFormat(),as the
//! \e dataFormat parameter.
//
//*****************************************************************************
typedef enum
{
   //! Filter output is in 16 bits 2's complement format.
   SDFM_DATA_FORMAT_16_BIT = 0,
   //! Filter output is in 32 bits 2's complement format.
   SDFM_DATA_FORMAT_32_BIT = 1
} SDFM_OutputDataFormat;

//*****************************************************************************
//
//! Values that can be passed to SDFM_setDataReadyInterruptSource(),as the
//! \e dataReadySource parameter.
//
//*****************************************************************************
typedef enum
{
   //! Data ready interrupt source is direct (non -FIFO).
   SDFM_DATA_READY_SOURCE_DIRECT = 0,
   //! Data ready interrupt source is FIFO.
   SDFM_DATA_READY_SOURCE_FIFO = 1
} SDFM_DataReadyInterruptSource;

//*****************************************************************************
//
//! Values that can be passed to SDFM_setPWMSyncSource(),as the
//! \e syncSource parameter.
//
//*****************************************************************************
typedef enum
{
   SDFM_SYNC_PWM1_SOCA = 0,    //!< SDFM sync source is PWM1 SOCA
   SDFM_SYNC_PWM1_SOCB = 1,    //!< SDFM sync source is PWM1 SOCB
   SDFM_SYNC_PWM2_SOCA = 4,    //!< SDFM sync source is PWM2 SOCA
   SDFM_SYNC_PWM2_SOCB = 5,    //!< SDFM sync source is PWM2 SOCB
   SDFM_SYNC_PWM3_SOCA = 8,    //!< SDFM sync source is PWM3 SOCA
   SDFM_SYNC_PWM3_SOCB = 9,    //!< SDFM sync source is PWM3 SOCB
   SDFM_SYNC_PWM4_SOCA = 12,   //!< SDFM sync source is PWM4 SOCA
   SDFM_SYNC_PWM4_SOCB = 13,   //!< SDFM sync source is PWM4 SOCB
   SDFM_SYNC_PWM5_SOCA = 16,   //!< SDFM sync source is PWM5 SOCA
   SDFM_SYNC_PWM5_SOCB = 17,   //!< SDFM sync source is PWM5 SOCB
   SDFM_SYNC_PWM6_SOCA = 20,   //!< SDFM sync source is PWM6 SOCA
   SDFM_SYNC_PWM6_SOCB = 21,   //!< SDFM sync source is PWM6 SOCB
   SDFM_SYNC_PWM7_SOCA = 24,   //!< SDFM sync source is PWM7 SOCA
   SDFM_SYNC_PWM7_SOCB = 25,   //!< SDFM sync source is PWM7 SOCB
   SDFM_SYNC_PWM8_SOCA = 28,   //!< SDFM sync source is PWM8 SOCA
   SDFM_SYNC_PWM8_SOCB = 29,   //!< SDFM sync source is PWM8 SOCB
   SDFM_SYNC_PWM9_SOCA = 32,   //!< SDFM sync source is PWM9 SOCA
   SDFM_SYNC_PWM9_SOCB = 33,   //!< SDFM sync source is PWM9 SOCB
   SDFM_SYNC_PWM10_SOCA = 36,  //!< SDFM sync source is PWM10 SOCA
   SDFM_SYNC_PWM10_SOCB = 37,  //!< SDFM sync source is PWM10 SOCB
   SDFM_SYNC_PWM11_SOCA = 40,  //!< SDFM sync source is PWM11 SOCA
   SDFM_SYNC_PWM11_SOCB = 41,  //!< SDFM sync source is PWM11 SOCB
   SDFM_SYNC_PWM12_SOCA = 44,  //!< SDFM sync source is PWM12 SOCA
   SDFM_SYNC_PWM12_SOCB = 45,  //!< SDFM sync source is PWM12 SOCB
   SDFM_SYNC_PWM13_SOCA = 48,  //!< SDFM sync source is PWM13 SOCA
   SDFM_SYNC_PWM13_SOCB = 49,  //!< SDFM sync source is PWM13 SOCB
   SDFM_SYNC_PWM14_SOCA = 52,  //!< SDFM sync source is PWM14 SOCA
   SDFM_SYNC_PWM14_SOCB = 53,  //!< SDFM sync source is PWM14 SOCB
   SDFM_SYNC_PWM15_SOCA = 56,  //!< SDFM sync source is PWM15 SOCA
   SDFM_SYNC_PWM15_SOCB = 57,  //!< SDFM sync source is PWM15 SOCB
   SDFM_SYNC_PWM16_SOCA = 60,  //!< SDFM sync source is PWM16 SOCA
   SDFM_SYNC_PWM16_SOCB = 61   //!< SDFM sync source is PWM16 SOCB
} SDFM_PWMSyncSource;

//*****************************************************************************
//
//! Values that can be passed to SDFM_setFIFOClearOnSyncMode(),as the
//! \e fifoClearSyncMode parameter.
//
//*****************************************************************************
typedef enum
{
   //! SDFM FIFO buffer is not cleared on Sync signal
   SDFM_FIFO_NOT_CLEARED_ON_SYNC = 0,
   //! SDFM FIFO buffer is cleared on Sync signal
   SDFM_FIFO_CLEARED_ON_SYNC     = 1
} SDFM_FIFOClearSyncMode;

//*****************************************************************************
//
//! Values that can be passed to SDFM_setWaitForSyncClearMode(),as the
//! \e syncClearMode parameter.
//
//*****************************************************************************
typedef enum
{
   //! Wait for sync cleared using software.
   SDFM_MANUAL_CLEAR_WAIT_FOR_SYNC = 0,
   //! Wait for sync cleared automatically
   SDFM_AUTO_CLEAR_WAIT_FOR_SYNC   = 1
} SDFM_WaitForSyncClearMode;

//*****************************************************************************
//
//! Values that can be passed to SDFM_selectCompEventSource() as the
//! \e compEventNum parameter.
//
//*****************************************************************************
typedef enum
{
    SDFM_COMP_EVENT_1 = SDFM_SDCPARM1_CEVT1SEL_S, //!< Selects CEVT1
    SDFM_COMP_EVENT_2 = SDFM_SDCPARM1_CEVT2SEL_S  //!< Selects CEVT2
} SDFM_CompEventNumber;

//*****************************************************************************
//
//! Values that can be passed to SDFM_selectCompEventSource() as the
//! \e compEventSource parameter.
//
//*****************************************************************************
typedef enum
{
    SDFM_COMP_EVENT_SRC_COMPH1    = 0, //!< COMPH1 event is the source
    SDFM_COMP_EVENT_SRC_COMPH1_L1 = 1, //!< Either of COMPH1 or COMPL1 event
                                       //!< can be the source
    SDFM_COMP_EVENT_SRC_COMPH2    = 2, //!< COMPH2 event is the source
    SDFM_COMP_EVENT_SRC_COMPH2_L2 = 3, //!< Either of COMPH2 or COMPL2 event
                                       //!< can be the source
    SDFM_COMP_EVENT_SRC_COMPL1    = 0, //!< COMPL1 event is the source
    SDFM_COMP_EVENT_SRC_COMPL2    = 2  //!< COMPL2 event is the source
} SDFM_CompEventSource;

//*****************************************************************************
//
//! Values that can be passed to SDFM_selectClockSource() as the \e clkSource
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! Source is respective channel clock
    SDFM_CLK_SOURCE_CHANNEL_CLK = 0x0,
    //! Source is SD1 channel clock is the source
    SDFM_CLK_SOURCE_SD1_CLK = SDFM_SDCTLPARM1_SDCLKSEL
} SDFM_ClockSource;

//*****************************************************************************
//
//! Values that can be passed to SDFM_selectCompEventHighSource() as the
//! \e source parameter.
//
//*****************************************************************************
typedef enum
{
    //! Comparator event high source is unfiltered event
    SDFM_COMPHOUT_SOURCE_COMPHIN = 0x0,
    //! Comparator event high source is filtered event
    SDFM_COMPHOUT_SOURCE_FILTER  = 0x8
} SDFM_CompEventHighSource;

//*****************************************************************************
//
//! Values that can be passed to SDFM_selectCompEventLowSource() as the
//! \e source parameter.
//
//*****************************************************************************
typedef enum
{
    //! Comparator event low source is unfiltered event
    SDFM_COMPLOUT_SOURCE_COMPLIN = 0x000,
    //! Comparator event low source is filtered event
    SDFM_COMPLOUT_SOURCE_FILTER  = 0x800
} SDFM_CompEventLowSource;

//*****************************************************************************
//
//! Values that can be passed to SDFM_configCompEventLowFilter() &
//! SDFM_configCompEventHighFilter() as the \e filterNumber.
//
//*****************************************************************************
typedef struct
{
    uint16_t sampleWindow; //!< Sample window size
    uint16_t threshold;    //!< Majority voting threshold
    uint16_t clkPrescale;  //!< Sample clock pre-scale
} SDFM_CompEventFilterConfig;

//*****************************************************************************
//
// Values that can be passed to SDFM_enableInterrupt and SDFM_disableInterrupt
// as intFlags parameter
//
//*****************************************************************************
//! Interrupt is generated if Modulator fails.
//!
#define SDFM_MODULATOR_FAILURE_INTERRUPT    0x200U
//! Interrupt on Comparator low-level threshold.
//!
#define SDFM_LOW_LEVEL_THRESHOLD_INTERRUPT  0x40U
//! Interrupt on Comparator high-level threshold.
//!
#define SDFM_HIGH_LEVEL_THRESHOLD_INTERRUPT 0x20U
//! Interrupt on Acknowledge flag
//!
#define SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT 0x1U
//! Interrupt on FIFO level
//!
#define SDFM_FIFO_INTERRUPT                 0x1000U
//! Interrupt on FIFO overflow
//!
#define SDFM_FIFO_OVERFLOW_INTERRUPT        0x8000U

//*****************************************************************************
//
// Values that can be passed to SDFM_clearInterruptFlag flags parameter
//
//*****************************************************************************
//! Main interrupt flag
//!
#define SDFM_MAIN_INTERRUPT_FLAG           0x80000000U
//! Filter 1 high -level threshold flag
//!
#define SDFM_FILTER_1_HIGH_THRESHOLD_FLAG  0x1U
//! Filter 1 low -level threshold flag
//!
#define SDFM_FILTER_1_LOW_THRESHOLD_FLAG   0x2U
//! Filter 2 high -level threshold flag
//!
#define SDFM_FILTER_2_HIGH_THRESHOLD_FLAG  0x4U
//! Filter 2 low -level threshold flag
//!
#define SDFM_FILTER_2_LOW_THRESHOLD_FLAG   0x8U
//! Filter 3 high -level threshold flag
//!
#define SDFM_FILTER_3_HIGH_THRESHOLD_FLAG  0x10U
//! Filter 3 low -level threshold flag
//!
#define SDFM_FILTER_3_LOW_THRESHOLD_FLAG   0x20U
//! Filter 4 high -level threshold flag
//!
#define SDFM_FILTER_4_HIGH_THRESHOLD_FLAG  0x40U
//! Filter 4 low -level threshold flag
//!
#define SDFM_FILTER_4_LOW_THRESHOLD_FLAG   0x80U
//! Filter 1 modulator failed flag
//!
#define SDFM_FILTER_1_MOD_FAILED_FLAG      0x100U
//! Filter 2 modulator failed flag
//!
#define SDFM_FILTER_2_MOD_FAILED_FLAG      0x200U
//! Filter 3 modulator failed flag
//!
#define SDFM_FILTER_3_MOD_FAILED_FLAG      0x400U
//! Filter 4 modulator failed flag
//!
#define SDFM_FILTER_4_MOD_FAILED_FLAG      0x800U
//! Filter 1 new data flag
//!
#define SDFM_FILTER_1_NEW_DATA_FLAG        0x1000U
//! Filter 2 new data flag
//!
#define SDFM_FILTER_2_NEW_DATA_FLAG        0x2000U
//! Filter 3 new data flag
//!
#define SDFM_FILTER_3_NEW_DATA_FLAG        0x4000U
//! Filter 4 new data flag
//!
#define SDFM_FILTER_4_NEW_DATA_FLAG        0x8000U
//! Filter 1 FIFO overflow flag
//!
#define SDFM_FILTER_1_FIFO_OVERFLOW_FLAG   0x10000U
//! Filter 2 FIFO overflow flag
//!
#define SDFM_FILTER_2_FIFO_OVERFLOW_FLAG   0x20000U
//! Filter 3 FIFO overflow flag
//!
#define SDFM_FILTER_3_FIFO_OVERFLOW_FLAG   0x40000U
//! Filter 4 FIFO overflow flag
//!
#define SDFM_FILTER_4_FIFO_OVERFLOW_FLAG   0x80000U
//! Filter 1 FIFO overflow flag
//!
#define SDFM_FILTER_1_FIFO_INTERRUPT_FLAG  0x100000U
//! Filter 2 FIFO overflow flag
//!
#define SDFM_FILTER_2_FIFO_INTERRUPT_FLAG  0x200000U
//! Filter 3 FIFO overflow flag
//!
#define SDFM_FILTER_3_FIFO_INTERRUPT_FLAG  0x400000U
//! Filter 4 FIFO overflow flag
//!
#define SDFM_FILTER_4_FIFO_INTERRUPT_FLAG  0x800000U

//*****************************************************************************
//
//! \internal
//! Checks SDFM base address.
//!
//! \param base specifies the SDFM module base address.
//!
//! This function determines if SDFM module base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
SDFM_isBaseValid(uint32_t base)
{
    return(
           (base == SDFM1_BASE) ||
           (base == SDFM2_BASE)
          );
}
#endif

//*****************************************************************************
//
//! Enable external reset
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function enables data filter to be reset by an external source (PWM
//! compare output).
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_enableExternalReset(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Set the SDSYNCEN bit
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDDFPARM1 + ((uint32_t)filterNumber * 16U)) |=
        SDFM_SDDFPARM1_SDSYNCEN;
    EDIS;
}

//*****************************************************************************
//
//! Disable external reset
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function disables data filter from being reset by an external source
//! (PWM compare output).
//!
//! \return None.
//*****************************************************************************
static inline void
SDFM_disableExternalReset(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Clear the SDSYNCEN bit
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDDFPARM1 + ((uint32_t)filterNumber * 16U)) &=
        ~SDFM_SDDFPARM1_SDSYNCEN;
    EDIS;
}

//*****************************************************************************
//
//! Enable filter
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function enables the filter specified by the \e filterNumber variable.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_enableFilter(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Set the FEN bit
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDDFPARM1 + ((uint32_t)filterNumber * 16U)) |=
        SDFM_SDDFPARM1_FEN;
    EDIS;
}

//*****************************************************************************
//
//! Disable filter
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function disables the filter specified by the \e filterNumber
//! variable.
//!
//! \return None.
//*****************************************************************************
static inline void
SDFM_disableFilter(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Clear the FEN bit
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDDFPARM1 + ((uint32_t)filterNumber * 16U)) &=
        ~SDFM_SDDFPARM1_FEN;
    EDIS;
}

//*****************************************************************************
//
//! Enable FIFO buffer
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function enables the filter FIFO buffer specified by the
//! \e filterNumber variable.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_enableFIFOBuffer(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Set the FFEN bit
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDFIFOCTL1 + ((uint32_t)filterNumber * 16U)) |=
        SDFM_SDFIFOCTL1_FFEN;
    EDIS;
}

//*****************************************************************************
//
//! Disable FIFO buffer
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function disables the filter FIFO buffer specified by the
//! \e filterNumber variable.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_disableFIFOBuffer(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Clear the FFEN bit
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDFIFOCTL1 + ((uint32_t)filterNumber * 16U)) &=
        ~SDFM_SDFIFOCTL1_FFEN;
    EDIS;
}

//*****************************************************************************
//
//! Return the Zero Cross Trip status
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function returns the Zero Cross Trip status for the filter
//! specified by filterNumber variable.
//!
//! \return \b true if Comparator filter output >= High-level threshold (Z)
//!         \b false if Comparator filter output < High-level threshold (Z)
//
//*****************************************************************************
static inline bool
SDFM_getZeroCrossTripStatus(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    return(((HWREGH(base + SDFM_O_SDSTATUS) >> (uint16_t)filterNumber) &
            0x1U) == 1U);
}

//*****************************************************************************
//
//! Clear the Zero Cross Trip status
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//!  This function clears the Zero Cross Trip status for the filter
//!  specified by filterNumber variable.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_clearZeroCrossTripStatus(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Set SDCTL MIE bit
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDCTL) |= ((uint16_t)1U << (uint16_t)filterNumber);
    EDIS;
}

//*****************************************************************************
//
//! Enable Comparator.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//!  This function enables the Comparator for the selected filter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_enableComparator(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Set CEN bit
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDCPARM1 + ((uint32_t)filterNumber * 16U)) |=
        SDFM_SDCPARM1_CEN;
    EDIS;
}

//*****************************************************************************
//
//! Disable Comparator.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//!  This function disables the Comparator for the selected filter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_disableComparator(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Clear CEN bit
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDCPARM1 + ((uint32_t)filterNumber * 16U)) &=
        ~SDFM_SDCPARM1_CEN;
    EDIS;
}

//*****************************************************************************
//
//! Selects Comparator Event Source.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number
//! \param compEventNum is the event number
//! \param compEventSource is the event source
//!
//!  This function selects the comparator event source. Valid values for
//! \e compEventNum are:
//! - SDFM_COMP_EVENT_1 - Selects comparator event 1
//! - SDFM_COMP_EVENT_2 - Selects comparator event 2
//! Valid values for \e SDFM_COMP_EVENT_1 are:
//! - SDFM_COMP_EVENT_SRC_COMPH1 - COMPH1 event is the source for selected event
//! - SDFM_COMP_EVENT_SRC_COMPH1_L1 - Either of COMPH1 or COMPL1 event can be
//!                                   the source for selected event
//! - SDFM_COMP_EVENT_SRC_COMPH2 - COMPH2 event is the source for selected event
//! - SDFM_COMP_EVENT_SRC_COMPH2_L2 - Either of COMPH2 or COMPL2 event can be
//!                                   the source for selected event
//!
//! Valid values for \e SDFM_COMP_EVENT_2 are:
//! - SDFM_COMP_EVENT_SRC_COMPL1 - COMPL1 event is the source for selected event
//! - SDFM_COMP_EVENT_SRC_COMPH1_L1 - Either of COMPH1 or COMPL1 event can be
//!                                   the source for selected event
//! - SDFM_COMP_EVENT_SRC_COMPL2 - COMPL2 event is the source for selected event
//! - SDFM_COMP_EVENT_SRC_COMPH2_L2 - Either of COMPH2 or COMPL2 event can be
//!                                   the source for selected event
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_selectCompEventSource(uint32_t base, SDFM_FilterNumber filterNumber,
                           SDFM_CompEventNumber compEventNum,
                           SDFM_CompEventSource compEventSource)
{
    uint32_t address;
    ASSERT(SDFM_isBaseValid(base));
    address = base + SDFM_O_SDCPARM1 + ((uint32_t)filterNumber *
                                         SDFM_SDFIL_OFFSET);

    //
    // Select source for selected comparator event
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & ~((uint16_t)0x2U <<
                       (uint16_t)compEventNum)) |
                      ((uint16_t)compEventSource << (uint16_t)compEventNum) ;
    EDIS;
}

//*****************************************************************************
//
//! Set filter type.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param filterType is the filter type or structure.
//!
//! This function sets the filter type or structure to be used as specified by
//! filterType for the selected filter number as specified by filterNumber.
//!
//! \return None.
//*****************************************************************************
static inline void
SDFM_setFilterType(uint32_t base, SDFM_FilterNumber filterNumber,
                   SDFM_FilterType filterType)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));

    address = base + SDFM_O_SDDFPARM1 + ((uint32_t)filterNumber * 16U);

    //
    // Write to SST bits
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & (~SDFM_SDDFPARM1_SST_M)) |
                      ((uint16_t)filterType << 6U);
    EDIS;
}

//*****************************************************************************
//
//! Set data filter over sampling ratio.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param overSamplingRatio is the data filter over sampling ratio.
//!
//! This function sets the filter oversampling ratio for the filter specified
//! by the filterNumber variable.Valid values for the variable
//! overSamplingRatio are 0 to 255 inclusive. The actual oversampling ratio
//! will be this value plus one.
//!
//! \return None.
//*****************************************************************************
static inline void
SDFM_setFilterOverSamplingRatio(uint32_t base, SDFM_FilterNumber filterNumber,
                                uint16_t overSamplingRatio)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));
    ASSERT(overSamplingRatio < 256U);

    address = base + SDFM_O_SDDFPARM1 + ((uint32_t)filterNumber * 16U);

    //
    // Write to DOSR bits
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & (~SDFM_SDDFPARM1_DOSR_M)) |
                      overSamplingRatio;
    EDIS;
}

//*****************************************************************************
//
//! Set modulator clock mode.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param clockMode is the modulator clock mode.
//!
//! This function sets the modulator clock mode specified by clockMode
//! for the filter specified by filterNumber.
//!
//! \note This function also enables the data and clock synchronizers for
//! the specified filter.
//!
//! \return None.
//*****************************************************************************
static inline void
SDFM_setupModulatorClock(uint32_t base, SDFM_FilterNumber filterNumber,
                         SDFM_ModulatorClockMode clockMode)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));

    address = base + SDFM_O_SDCTLPARM1 + ((uint32_t)filterNumber * 16U);

    //
    // Write to MOD bits
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & (~SDFM_SDCTLPARM1_MOD_M)) |
                      (uint16_t)clockMode;

    //
    // Enable data and clock synchronizer
    //
    HWREGH(base + SDFM_O_SDCTLPARM1 +
           ((uint32_t)filterNumber * SDFM_SDFIL_OFFSET)) |=
           (SDFM_CLOCK_SYNCHRONIZER | SDFM_DATA_SYNCHRONIZER);
    EDIS;
}

//*****************************************************************************
//
//! Set the output data format
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param dataFormat is the output data format.
//!
//! This function sets the output data format for the filter specified by
//! filterNumber.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_setOutputDataFormat(uint32_t base, SDFM_FilterNumber filterNumber,
                         SDFM_OutputDataFormat dataFormat)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));

    address = base + SDFM_O_SDDPARM1 + ((uint32_t)filterNumber * 16U);

    //
    // Write to DR bit
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & (~SDFM_SDDPARM1_DR)) |
                      ((uint16_t)dataFormat << 10U);
    EDIS;
}

//*****************************************************************************
//
//! Set data shift value.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param shiftValue is the data shift value.
//!
//! This function sets the shift value for the 16 bit 2's complement data
//! format. The valid maximum value for shiftValue is 31.
//!
//! \b Note: Use this function with 16 bit 2's complement data format only.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_setDataShiftValue(uint32_t base, SDFM_FilterNumber filterNumber,
                       uint16_t shiftValue)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));
    ASSERT(shiftValue < 32U);

    address = base + SDFM_O_SDDPARM1 + ((uint32_t)filterNumber * 16U);

    //
    // Write to SH bit
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & (~SDFM_SDDPARM1_SH_M)) |
                      (shiftValue << SDFM_SDDPARM1_SH_S);
    EDIS;
}


//*****************************************************************************
//
//! Set Filter output high-level threshold.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param highThreshold is the high-level threshold 1 & 2.
//!
//! This function sets the unsigned high-level threshold value for the
//! Comparator filter output. If the output value of the filter exceeds
//! highThreshold and interrupt generation is enabled, an interrupt will be
//! issued. The param \b highThreshold takes both high threshold 1 & 2 values.
//! The upper 16-bits represent the high threshold 2 value while lower 16-bits
//! represent the threshold 1 values.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_setCompFilterHighThreshold(uint32_t base, SDFM_FilterNumber filterNumber,
                                uint32_t highThreshold)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));
    ASSERT((uint16_t)highThreshold <= SDFM_SDFLT1CMPH1_HLT_M);
    ASSERT((uint16_t)(highThreshold >> 16U) <= SDFM_SDFLT1CMPH2_HLT2_M);

    address = base + SDFM_O_SDFLT1CMPH1 +
              ((uint32_t)filterNumber * SDFM_SDFIL_OFFSET);

    //
    // Write to HLT bit
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & ~SDFM_SDFLT1CMPH1_HLT_M) |
                      (uint16_t)highThreshold;
    HWREGH(address + SDFM_SDFLT1CMPHx_OFFSET) =
                                  (HWREGH(address + SDFM_SDFLT1CMPHx_OFFSET) &
                                   ~SDFM_SDFLT1CMPH2_HLT2_M) |
                                  (uint16_t)(highThreshold >> 16U);
    EDIS;
}

//*****************************************************************************
//
//! Set Filter output low-level threshold.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number
//! \param lowThreshold is the low-level threshold
//!
//! This function sets the unsigned low-level threshold value 1 or 2 for the
//! Comparator filter output. If the output value of the filter gets below
//! lowThreshold and interrupt generation is enabled, an interrupt will be
//! issued. The param \b lowThreshold takes both low threshold 1 & 2 values.
//! The upper 16-bits represent the low threshold 2 value while lower 16-bits
//! represent the threshold 1 values.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_setCompFilterLowThreshold(uint32_t base, SDFM_FilterNumber filterNumber,
                               uint32_t lowThreshold)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));
    ASSERT((uint16_t)lowThreshold <= SDFM_SDFLT1CMPL1_LLT_M);
    ASSERT((uint16_t)(lowThreshold >> 16U) <= SDFM_SDFLT1CMPL2_LLT2_M);

    address = base + SDFM_O_SDFLT1CMPL1 +
              ((uint32_t)filterNumber * SDFM_SDFIL_OFFSET);

    //
    // Write to LLT bit.
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & ~SDFM_SDFLT1CMPL1_LLT_M) |
                      (uint16_t)lowThreshold;
    HWREGH(address + SDFM_SDFLT1CMPLx_OFFSET) =
                                   (HWREGH(address + SDFM_SDFLT1CMPLx_OFFSET) &
                                    ~SDFM_SDFLT1CMPL2_LLT2_M) |
                                   (uint16_t)(lowThreshold >> 16U);
    EDIS;
}
//*****************************************************************************
//
//! Set Filter output zero-cross threshold.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param zeroCrossThreshold is the zero-cross threshold.
//!
//! This function sets the unsigned zero-cross threshold value for the
//! Comparator filter output.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_setCompFilterZeroCrossThreshold(uint32_t base,
                                     SDFM_FilterNumber filterNumber,
                                     uint16_t zeroCrossThreshold)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));
    ASSERT(zeroCrossThreshold < 0x7FFFU);

    address = base + SDFM_O_SDFLT1CMPHZ + ((uint32_t)filterNumber * 16U);

    //
    // Write to ZCT bit
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & ~SDFM_SDFLT1CMPHZ_HLTZ_M) |
                      zeroCrossThreshold;

    EDIS;
}

//*****************************************************************************
//
//! Enable zero-cross Edge detect mode.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function enables Zero Cross Edge detection.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_enableZeroCrossEdgeDetect(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Set ZCEN bit
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDCPARM1 + ((uint32_t)filterNumber * 16U)) |=
        SDFM_SDCPARM1_HZEN;
    EDIS;
}

//*****************************************************************************
//
//! Disable zero-cross Edge detect mode.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function disables Zero Cross Edge detection.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_disableZeroCrossEdgeDetect(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Clear ZCEN bit
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDCPARM1 + ((uint32_t)filterNumber * 16U)) &=
        ~SDFM_SDCPARM1_HZEN;
    EDIS;
}

//*****************************************************************************
//
//! Enable SDFM interrupts.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param intFlags is the interrupt source.
//!
//! This function enables the low threshold , high threshold or modulator
//! failure interrupt as determined by intFlags for the filter specified
//! by filterNumber.
//! Valid values for intFlags are:
//!  SDFM_MODULATOR_FAILURE_INTERRUPT , SDFM_LOW_LEVEL_THRESHOLD_INTERRUPT,
//!  SDFM_HIGH_LEVEL_THRESHOLD_INTERRUPT, SDFM_FIFO_INTERRUPT,
//!  SDFM_FIFO_OVERFLOW_INTERRUPT,SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_enableInterrupt(uint32_t base, SDFM_FilterNumber filterNumber,
                     uint16_t intFlags)
{
    uint16_t offset;

    ASSERT(SDFM_isBaseValid(base));

    offset = (uint16_t)filterNumber * 16U;

    EALLOW;

    //
    // Low, high threshold, Modulator failure
    //
    if((intFlags & (SDFM_MODULATOR_FAILURE_INTERRUPT |
                    SDFM_LOW_LEVEL_THRESHOLD_INTERRUPT |
                    SDFM_HIGH_LEVEL_THRESHOLD_INTERRUPT)) != 0U)
    {
        //
        // Set IEL or IEH or MFIE bit of SDFM_O_SDCPARMx
        //
        HWREGH(base + SDFM_O_SDCPARM1 + offset) |=
                (intFlags & (SDFM_MODULATOR_FAILURE_INTERRUPT |
                             SDFM_LOW_LEVEL_THRESHOLD_INTERRUPT |
                             SDFM_HIGH_LEVEL_THRESHOLD_INTERRUPT));
    }

    //
    // Data filter acknowledge interrupt
    //
    if((intFlags & SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT) != 0U)
    {
        HWREGH(base + SDFM_O_SDDFPARM1 + offset) |= SDFM_SDDFPARM1_AE;
    }

    //
    // FIFO , FIFO overflow interrupt
    //
    if((intFlags & (SDFM_FIFO_INTERRUPT | SDFM_FIFO_OVERFLOW_INTERRUPT)) != 0U)
    {
        //
        // Set OVFIEN or FFIEN bits of SDFM_O_SDFIFOCTLx
        //
        HWREGH(base + SDFM_O_SDFIFOCTL1 + offset) |=
                (intFlags & (SDFM_FIFO_INTERRUPT |
                             SDFM_FIFO_OVERFLOW_INTERRUPT));
    }
    EDIS;
}

//*****************************************************************************
//
//! Disable SDFM interrupts.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param intFlags is the interrupt source.
//!
//! This function disables the low threshold , high threshold or modulator
//! failure interrupt as determined by intFlags for the filter
//! specified by filterNumber.
//! Valid values for intFlags are:
//!  SDFM_MODULATOR_FAILURE_INTERRUPT , SDFM_LOW_LEVEL_THRESHOLD_INTERRUPT,
//!  SDFM_HIGH_LEVEL_THRESHOLD_INTERRUPT, SDFM_FIFO_INTERRUPT,
//!  SDFM_FIFO_OVERFLOW_INTERRUPT,SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_disableInterrupt(uint32_t base, SDFM_FilterNumber filterNumber,
                      uint16_t intFlags)
{
    uint16_t offset;

    ASSERT(SDFM_isBaseValid(base));

    offset = (uint16_t)filterNumber * 16U;

    EALLOW;

    //
    // Low, high threshold, modulator failure interrupts
    //
    if((intFlags & (SDFM_MODULATOR_FAILURE_INTERRUPT |
                    SDFM_LOW_LEVEL_THRESHOLD_INTERRUPT |
                    SDFM_HIGH_LEVEL_THRESHOLD_INTERRUPT)) != 0U)
    {
        //
        // Set IEL or IEH or MFIE bit of SDFM_O_SDCPARMx
        //
        HWREGH(base + SDFM_O_SDCPARM1 + offset) &=
            ~(intFlags & (SDFM_MODULATOR_FAILURE_INTERRUPT |
                          SDFM_LOW_LEVEL_THRESHOLD_INTERRUPT |
                          SDFM_HIGH_LEVEL_THRESHOLD_INTERRUPT));
    }

    //
    // Data filter acknowledge interrupt
    //
    if((intFlags & SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT) != 0U)
    {
        HWREGH(base + SDFM_O_SDDFPARM1 + offset) &= ~SDFM_SDDFPARM1_AE;
    }

    //
    // FIFO , FIFO overflow interrupt
    //
    if((intFlags & (SDFM_FIFO_INTERRUPT | SDFM_FIFO_OVERFLOW_INTERRUPT)) != 0U)
    {
         //
         // Set OVFIEN or FFIEN bits of SDFM_O_SDFIFOCTLx
         //
         HWREGH(base + SDFM_O_SDFIFOCTL1 + offset) &=
            ~(intFlags & (SDFM_FIFO_INTERRUPT | SDFM_FIFO_OVERFLOW_INTERRUPT));
    }
    EDIS;
}

//*****************************************************************************
//
//! Set the comparator filter type.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param filterType is the comparator filter type or structure.
//!
//! This function sets the Comparator filter type or structure to be used as
//! specified by filterType for the selected filter number as specified by
//! filterNumber.
//!
//! \return None.
//*****************************************************************************
static inline void
SDFM_setComparatorFilterType(uint32_t base, SDFM_FilterNumber filterNumber,
                             SDFM_FilterType filterType)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));

    address = base + SDFM_O_SDCPARM1 + ((uint32_t)filterNumber * 16U);

    //
    // Write to CS1_CS0 bits
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & (~SDFM_SDCPARM1_CS1_CS0_M)) |
                      ((uint16_t)filterType << 3U);
    EDIS;
}

//*****************************************************************************
//
//! Set Comparator filter over sampling ratio.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param overSamplingRatio is the comparator filter over sampling ration.
//!
//! This function sets the comparator filter oversampling ratio for the filter
//! specified by the filterNumber.Valid values for the variable
//! overSamplingRatio are 0 to 31 inclusive.
//! The actual oversampling ratio will be this value plus one.
//!
//! \return None.
//*****************************************************************************
static inline void
SDFM_setCompFilterOverSamplingRatio(uint32_t base,
                                    SDFM_FilterNumber filterNumber,
                                    uint16_t overSamplingRatio)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));
    ASSERT(overSamplingRatio < 32U);

    address = base + SDFM_O_SDCPARM1 + ((uint32_t)filterNumber * 16U);

    //
    // Write to COSR bits
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & (~SDFM_SDCPARM1_COSR_M)) |
                      overSamplingRatio;
    EDIS;
}

//*****************************************************************************
//
//! Get the filter data output.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function returns the latest data filter output. Depending on the
//! filter data output format selected, the valid value will be the lower 16
//! bits or the whole 32 bits of the returned value.
//!
//! \return Returns the latest data filter output.
//*****************************************************************************
static inline uint32_t
SDFM_getFilterData(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Read SDDATA bits
    //
    return(HWREG(base + SDFM_O_SDDATA1 + ((uint32_t)filterNumber * 16U)));
}

//*****************************************************************************
//
//! Get the Comparator threshold status.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function returns the Comparator output threshold status for the given
//! filterNumber.
//!
//! \return Returns the following status flags.
//! - \b SDFM_OUTPUT_WITHIN_THRESHOLD if the output is within the
//!                                   specified threshold.
//! - \b SDFM_OUTPUT_ABOVE_THRESHOLD  if the output is above the high
//!                                   threshold
//! - \b SDFM_OUTPUT_BELOW_THRESHOLD  if the output is below the low
//!                                   threshold.
//!
//*****************************************************************************
static inline SDFM_OutputThresholdStatus
SDFM_getThresholdStatus(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Read SDIFLG high/low threshold bits
    //
    return((SDFM_OutputThresholdStatus)((HWREG(base + SDFM_O_SDIFLG) >>
                                        (2U * (uint16_t)filterNumber)) & 0x3U));
}

//*****************************************************************************
//
//! Get the Modulator status.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function returns the Modulator status.
//!
//! \return Returns true if the Modulator is operating normally
//!         Returns false if the Modulator has failed
//!
//*****************************************************************************
static inline bool
SDFM_getModulatorStatus(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Read SDIFLG MF1, MF2, MF3 OR MF4 bits
    //
    return(((HWREG(base + SDFM_O_SDIFLG) >> ((uint16_t)filterNumber + 8U)) &
            0x1U) != 0x1U);
}

//*****************************************************************************
//
//! Check if new Filter data is available.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function returns new filter data status.
//!
//! \return Returns \b true if new filter data is available
//!         Returns \b false if no new filter data is available
//!
//*****************************************************************************
static inline bool
SDFM_getNewFilterDataStatus(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Read SDIFLG AF1, AF2, AF3 OR AF4 bits
    //
    return(((HWREG(base + SDFM_O_SDIFLG) >> ((uint16_t)filterNumber + 12U)) &
            0x1U) == 0x1U);
}

//*****************************************************************************
//
//! Check if FIFO buffer is overflowed.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function returns the status of the FIFO buffer overflow for the given
//! filter value.
//!
//! \return Returns \b true if FIFO buffer is overflowed
//!         Returns \b false if FIFO buffer is not overflowed
//!
//*****************************************************************************
static inline bool
SDFM_getFIFOOverflowStatus(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Read SDIFLG SDFFOVF1, SDFFOVF2, SDFFOVF3 OR SDFFOVF4 bits
    //
    return(((HWREG(base + SDFM_O_SDIFLG) >> ((uint16_t)filterNumber + 16U)) &
            0x1U) == 0x1U);
}

//*****************************************************************************
//
//! Check FIFO buffer interrupt status.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function returns the status of the FIFO buffer interrupt for the given
//! filter.
//!
//! \return Returns \b true if FIFO buffer interrupt has occurred.
//!         Returns \b false if FIFO buffer interrupt has not occurred.
//!
//*****************************************************************************
static inline bool
SDFM_getFIFOISRStatus(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Read SDIFLG SDFFINT1, SDFFINT2, SDFFINT3 OR SDFFINT4 bits
    //
    return(((HWREG(base + SDFM_O_SDIFLG) >> ((uint16_t)filterNumber + 20U)) &
            0x1U) == 0x1U);
}

//*****************************************************************************
//
//! Get pending interrupt.
//!
//! \param base is the base address of the SDFM module
//!
//! This function returns any pending interrupt status.
//!
//! \return Returns \b true if there is a pending interrupt.
//!         Returns \b false if no interrupt is pending.
//!
//*****************************************************************************
static inline bool
SDFM_getIsrStatus(uint32_t base)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Read SDIFLG MIF
    //
    return((HWREG(base + SDFM_O_SDIFLG) >> 31U) == 0x1U);
}

//*****************************************************************************
//
//! Clear pending flags.
//!
//! \param base is the base address of the SDFM module
//! \param flag is the SDFM status
//!
//! This function clears the specified pending interrupt flag.
//! Valid values are
//! SDFM_MAIN_INTERRUPT_FLAG,SDFM_FILTER_1_NEW_DATA_FLAG,
//! SDFM_FILTER_2_NEW_DATA_FLAG,SDFM_FILTER_3_NEW_DATA_FLAG,
//! SDFM_FILTER_4_NEW_DATA_FLAG,SDFM_FILTER_1_MOD_FAILED_FLAG,
//! SDFM_FILTER_2_MOD_FAILED_FLAG,SDFM_FILTER_3_MOD_FAILED_FLAG,
//! SDFM_FILTER_4_MOD_FAILED_FLAG,SDFM_FILTER_1_HIGH_THRESHOLD_FLAG,
//! SDFM_FILTER_1_LOW_THRESHOLD_FLAG,SDFM_FILTER_2_HIGH_THRESHOLD_FLAG,
//! SDFM_FILTER_2_LOW_THRESHOLD_FLAG,SDFM_FILTER_3_HIGH_THRESHOLD_FLAG,
//! SDFM_FILTER_3_LOW_THRESHOLD_FLAG,SDFM_FILTER_4_HIGH_THRESHOLD_FLAG,
//! SDFM_FILTER_4_LOW_THRESHOLD_FLAG,SDFM_FILTER_1_FIFO_OVERFLOW_FLAG,
//! SDFM_FILTER_2_FIFO_OVERFLOW_FLAG,SDFM_FILTER_3_FIFO_OVERFLOW_FLAG
//! SDFM_FILTER_4_FIFO_OVERFLOW_FLAG,SDFM_FILTER_1_FIFO_INTERRUPT_FLAG,
//! SDFM_FILTER_2_FIFO_INTERRUPT_FLAG,SDFM_FILTER_3_FIFO_INTERRUPT_FLAG
//! SDFM_FILTER_4_FIFO_INTERRUPT_FLAG or any combination of the above flags.
//!
//! \return None
//!
//*****************************************************************************
static inline void
SDFM_clearInterruptFlag(uint32_t base, uint32_t flag)
{
    ASSERT(SDFM_isBaseValid(base));
    ASSERT((flag & 0x80FFFFFFU) == flag);

    //
    // Write to  SDIFLGCLR register
    //
    HWREG(base + SDFM_O_SDIFLGCLR) |= flag;
}

//*****************************************************************************
//
//! Enable main interrupt.
//!
//! \param base is the base address of the SDFM module
//!
//! This function enables the main SDFM interrupt.
//!
//! \return None
//!
//*****************************************************************************
static inline void
SDFM_enableMainInterrupt(uint32_t base)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Set SDCTL MIE bit
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDCTL) |= SDFM_SDCTL_MIE;
    EDIS;
}

//*****************************************************************************
//
//! Disable main interrupt.
//!
//! \param base is the base address of the SDFM module
//!
//! This function disables the main SDFM interrupt.
//!
//! \return None
//!
//*****************************************************************************
static inline void
SDFM_disableMainInterrupt(uint32_t base)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Clear SDCTL MIE bit
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDCTL) &= ~SDFM_SDCTL_MIE;
    EDIS;
}

//*****************************************************************************
//
//! Enable main filter.
//!
//! \param base is the base address of the SDFM module
//!
//! This function enables main filter.
//!
//! \return None
//!
//*****************************************************************************
static inline void
SDFM_enableMainFilter(uint32_t base)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Set SDMFILEN MFE bit
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDMFILEN) |= SDFM_SDMFILEN_MFE;
    EDIS;
}

//*****************************************************************************
//
//! Disable main filter.
//!
//! \param base is the base address of the SDFM module
//!
//! This function disables main filter.
//!
//! \return None
//!
//*****************************************************************************
static inline void
SDFM_disableMainFilter(uint32_t base)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Clear SDMFILEN MFE bit
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDMFILEN) &= ~SDFM_SDMFILEN_MFE;
    EDIS;
}

//*****************************************************************************
//
//! Return the FIFO data count
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function returns the FIFO data count.
//!
//! \return Returns the number of data words available in FIFO buffer.
//
//*****************************************************************************
static inline uint16_t
SDFM_getFIFODataCount(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Read SDFFST
    //
    return((HWREGH(base + SDFM_O_SDFIFOCTL1 +
        ((uint32_t)filterNumber * 16U)) & SDFM_SDFIFOCTL1_SDFFST_M) >>
         SDFM_SDFIFOCTL1_SDFFST_S);
}

//*****************************************************************************
//
//! Return the Comparator sinc filter data
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function returns the Comparator sinc filter data output.
//!
//! \return Returns the Comparator sinc filter data output.
//!
//
//*****************************************************************************
static inline uint16_t
SDFM_getComparatorSincData(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Read SDCDATA
    //
    return(HWREGH(base + SDFM_O_SDCDATA1 + ((uint32_t)filterNumber * 16U)));
}

//*****************************************************************************
//
//! Return the FIFO data
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function returns the latest FIFO data.
//!
//! \return Returns the latest FIFO data.
//!
//! \note Discard the upper 16 bits if the output data format is 16bits.
//
//*****************************************************************************
static inline uint32_t
SDFM_getFIFOData(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Read SDDATFIFO
    //
    return(HWREG(base + SDFM_O_SDDATFIFO1 + ((uint32_t)filterNumber * 16U)));
}

//*****************************************************************************
//
//! Set the FIFO interrupt level.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param fifoLevel is the FIFO interrupt level.
//!
//! This function sets the FIFO interrupt level. Interrupt is generated when
//! the FIFO buffer word count gets to or exceeds the value of \e fifoLevel.
//! Maximum value for \e fifoLevel is 16.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_setFIFOInterruptLevel(uint32_t base, SDFM_FilterNumber filterNumber,
                           uint16_t fifoLevel)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));
    ASSERT(fifoLevel <= 16U);

    address = base + SDFM_O_SDFIFOCTL1 + ((uint32_t)filterNumber * 16U);

    //
    // Write to SDFFIL bit
    //
    EALLOW;
    HWREGH(address) =
        ((HWREGH(address) & (~SDFM_SDFIFOCTL1_SDFFIL_M)) | fifoLevel);
    EDIS;
}

//*****************************************************************************
//
//! Set data ready interrupt source.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param dataReadySource is the data ready interrupt source.
//!
//! This function sets the data ready interrupt source.
//! Valid values for \e dataReadySource:
//!   - SDFM_DATA_READY_SOURCE_DIRECT - Direct data ready
//!   - SDFM_DATA_READY_SOURCE_FIFO  - FIFO data ready.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_setDataReadyInterruptSource(uint32_t base, SDFM_FilterNumber filterNumber,
                                 SDFM_DataReadyInterruptSource dataReadySource)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));

    address = base + SDFM_O_SDFIFOCTL1 + ((uint32_t)filterNumber * 16U);

    //
    // Write to DRINTSEL
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & ~SDFM_SDFIFOCTL1_DRINTSEL) |
                      ((uint16_t)dataReadySource << 14U);
    EDIS;
}

//*****************************************************************************
//
//! Get the wait-for-sync event status.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function returns the Wait-for-Sync event status.
//!
//! \return Returns true if sync event has occurred.
//!         Returns false if sync event has not occurred.
//
//*****************************************************************************
static inline bool
SDFM_getWaitForSyncStatus(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Read WTSYNFLG bit
    //
    return(((HWREGH(base + SDFM_O_SDSYNC1 + ((uint32_t)filterNumber * 16U)) &
            SDFM_SDSYNC1_WTSYNFLG) >> 7U) == 0x1U);
}

//*****************************************************************************
//
//! Clear the Wait-for-sync event status.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function clears the Wait-for-sync event status.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_clearWaitForSyncFlag(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    EALLOW;

    //
    // Clear WTSYNCLR bit
    //
    HWREGH(base + SDFM_O_SDSYNC1 + ((uint32_t)filterNumber * 16U)) |=
           SDFM_SDSYNC1_WTSYNCLR;
    EDIS;
}

//*****************************************************************************
//
//! Enable wait for sync mode.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function enables the wait for sync mode. Data to FIFO will be written
//! only after PWM sync event.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_enableWaitForSync(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    EALLOW;

    //
    // Set WTSYNCEN bit
    //
    HWREGH(base + SDFM_O_SDSYNC1 + ((uint32_t)filterNumber * 16U)) |=
        SDFM_SDSYNC1_WTSYNCEN;
    EDIS;
}

//*****************************************************************************
//
//! Disable wait for sync mode.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function disables the wait for sync mode. Data to FIFO will be written
//! every Data ready event.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_disableWaitForSync(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    EALLOW;

    //
    // Clear WTSYNCEN bit
    //
    HWREGH(base + SDFM_O_SDSYNC1 + ((uint32_t)filterNumber * 16U)) &=
        ~SDFM_SDSYNC1_WTSYNCEN;
    EDIS;
}

//*****************************************************************************
//
//! Set the PWM sync mode.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param syncSource is the PWM sync source.
//!
//! This function sets the PWM sync source for the specific SDFM filter. Valid
//! values for syncSource are SDFM_SYNC_PWMx_CMPy. Where x ranges from 1 to 8
//! Representing PWM1 to PWM8 respectively and y ranges from A to D
//! representing PWM comparators A to D.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_setPWMSyncSource(uint32_t base, SDFM_FilterNumber filterNumber,
                      SDFM_PWMSyncSource syncSource)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));

    address = base + SDFM_O_SDSYNC1 + ((uint32_t)filterNumber * 16U);

    EALLOW;

    //
    // Write to SYNCSEL bits
    //
    HWREGH(address) =
        (HWREGH(address) & ~SDFM_SDSYNC1_SYNCSEL_M) | (uint16_t)syncSource;
    EDIS;

    //
    //Enable SDSYNC reset to data filter
    //
    SDFM_enableExternalReset(base, filterNumber);
}

//*****************************************************************************
//
//! Set FIFO clear on sync mode.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param fifoClearSyncMode is the FIFO clear on sync mode.
//!
//! This function sets the FIFO clear mode for the specified filter when a sync
//! happens depending on the value of fifoClearSyncMode.
//! Valid values for fifoClearSyncMode are:
//!  - SDFM_FIFO_NOT_CLEARED_ON_SYNC - FIFO is not cleared on sync.
//!  - SDFM_FIFO_CLEARED_ON_SYNC - FIFO is cleared on sync.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_setFIFOClearOnSyncMode(uint32_t base, SDFM_FilterNumber filterNumber,
                            SDFM_FIFOClearSyncMode fifoClearSyncMode)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));

    address = base + SDFM_O_SDSYNC1 + ((uint32_t)filterNumber * 16U);

    EALLOW;

    //
    // Write to FFSYNCCLREN bit
    //
    HWREGH(address) = (HWREGH(address) & ~SDFM_SDSYNC1_FFSYNCCLREN) |
                      ((uint16_t)fifoClearSyncMode << 9U);
    EDIS;
}

//*****************************************************************************
//
//! Set Wait-for-sync clear mode.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param syncClearMode is the wait-for-sync clear mode.
//!
//! This function sets the Wait-For-sync clear mode depending on the value of
//! syncClearMode.
//! Valid values for syncClearMode are:
//!   - SDFM_MANUAL_CLEAR_WAIT_FOR_SYNC - Wait-for-sync flag is cleared by
//!                                       invoking SDFM_clearWaitForSyncFlag().
//!   - SDFM_AUTO_CLEAR_WAIT_FOR_SYNC   - Wait-for-sync flag is cleared
//!                                       automatically on FIFO interrupt.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_setWaitForSyncClearMode(uint32_t base, SDFM_FilterNumber filterNumber,
                             SDFM_WaitForSyncClearMode syncClearMode)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));

    address = base + SDFM_O_SDSYNC1 + ((uint32_t)filterNumber * 16U);

    EALLOW;

    //
    // Write to WTSCLREN  bit
    //
    HWREGH(address) = (HWREGH(address) & ~SDFM_SDSYNC1_WTSCLREN) |
                      ((uint16_t)syncClearMode << 10U);
    EDIS;
}

//*****************************************************************************
//
//! Selects clock source for SDFM channels.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param clkSource is the clock source
//!
//! This function selects the clock for SDFM module filter channels.
//! Valid values for clkSource are:
//!   - SDFM_CLK_SOURCE_CHANNEL_CLK - Respective channel's clk is the source
//!   - SDFM_CLK_SOURCE_SD1_CLK     - Filter 1 clock is the source
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_selectClockSource(uint32_t base, SDFM_FilterNumber filterNumber,
                       SDFM_ClockSource clkSource)
{
    uint32_t address;

    ASSERT(SDFM_isBaseValid(base));
    address = base + SDFM_O_SDCTLPARM1 +
              ((uint32_t)filterNumber * SDFM_SDFIL_OFFSET);

    //
    // Select SDFM clock source.
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & ~(SDFM_SDCTLPARM1_SDCLKSEL)) |
                      (uint16_t)clkSource;
    EDIS;
}

//*****************************************************************************
//
//! Enables Input Synchronizer.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param syncConfig defines which synchronizer to be enabled
//!
//! This function enables either data or clock or both synchronizer.
//! Valid values for syncConfig can be the logical OR of any of the values:
//!   - SDFM_CLOCK_SYNCHRONIZER - Enable SDFM input clock synchronizer
//!   - SDFM_DATA_SYNCHRONIZER  - Enable SDFM input data synchronizer
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_enableSynchronizer(uint32_t base, SDFM_FilterNumber filterNumber,
                        uint16_t syncConfig)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Select SDFM clock source.
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDCTLPARM1 +
           ((uint32_t)filterNumber * SDFM_SDFIL_OFFSET)) |= syncConfig;
    EDIS;
}

//*****************************************************************************
//
//! Disables Input Synchronizer.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param syncConfig defines which synchronizer to be disabled
//!
//! This function disables either data or clock or both synchronizer.
//! Valid values for syncConfig can be the logical OR of any of the values:
//!   - SDFM_CLOCK_SYNCHRONIZER - Disable SDFM input clock synchronizer
//!   - SDFM_DATA_SYNCHRONIZER  - Disable SDFM input data synchronizer
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_disableSynchronizer(uint32_t base, SDFM_FilterNumber filterNumber,
                         uint16_t syncConfig)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Select SDFM clock source.
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDCTLPARM1 +
           ((uint32_t)filterNumber * SDFM_SDFIL_OFFSET)) &= ~syncConfig;
    EDIS;
}

//*****************************************************************************
//
//! Selects comparator event high source.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param source is the comparator event high source
//!
//! This function selects the source for comparator event high.
//! Valid values for source are:
//!   - SDFM_COMPHOUT_SOURCE_COMPHIN - Original COMPHIN/CEVT1 signal is source
//!   - SDFM_COMPHOUT_SOURCE_FILTER  - Filtered COMPHIN/CEVT1 signal is source
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_selectCompEventHighSource(uint32_t base, SDFM_FilterNumber filterNumber,
                               SDFM_CompEventHighSource source)
{
    uint32_t address;
    ASSERT(SDFM_isBaseValid(base));
    address = base + SDFM_O_SDCOMP1CTL +
              ((uint32_t)filterNumber * SDFM_DIGFIL_OFFSET);

    //
    // Set COMPHOUT source.
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & ~SDFM_SDCOMP1CTL_CEVT1DIGFILTSEL_M) |
                      (uint16_t)source;
    EDIS;
}

//*****************************************************************************
//
//! Selects comparator event low source.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param source is the comparator event low source
//!
//! This function selects the source for comparator event low.
//! Valid values for source are:
//!   - SDFM_COMPLOUT_SOURCE_COMPLIN - Original COMPLIN/CEVT2 signal is source
//!   - SDFM_COMPHOUT_SOURCE_FILTER  - Filtered COMPLIN/CEVT2 signal is source
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_selectCompEventLowSource(uint32_t base, SDFM_FilterNumber filterNumber,
                              SDFM_CompEventLowSource source)
{
    uint32_t address;
    ASSERT(SDFM_isBaseValid(base));
    address = base + SDFM_O_SDCOMP1CTL +
              ((uint32_t)filterNumber * SDFM_DIGFIL_OFFSET);

    //
    // Set COMPLOUT source.
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & ~SDFM_SDCOMP1CTL_CEVT2DIGFILTSEL_M) |
                      (uint16_t)source;
    EDIS;
}

//*****************************************************************************
//
//! Initializes Comparator Event Low Filter.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function initializes Comparator Event Low Filter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_initCompEventLowFilter(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Initialize comparator event low filter.
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDCOMP1EVT2FLTCTL +
           ((uint32_t)filterNumber * SDFM_DIGFIL_OFFSET)) |=
                                      (uint16_t)SDFM_SDCOMP1EVT2FLTCTL_FILINIT;
    EDIS;
}

//*****************************************************************************
//
//! Initializes Comparator Event High Filter.
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//!
//! This function initializes Comparator Event High Filter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_initCompEventHighFilter(uint32_t base, SDFM_FilterNumber filterNumber)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Initialize comparator event high filter.
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDCOMP1EVT1FLTCTL +
           ((uint32_t)filterNumber * SDFM_DIGFIL_OFFSET)) |=
                                      (uint16_t)SDFM_SDCOMP1EVT1FLTCTL_FILINIT;
    EDIS;
}

//*****************************************************************************
//
//! Lock Comparator Event Filter Configurations
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param lockConfig defines the configurations to be locked
//!
//! This function locks the comparator event filter configurations. Valid
//! values of the lockConfig can be logical OR of any of the following values:
//! - SDFM_SDCOMPLOCK_SDCOMPCTL - Locks write access to SDCOMPCTL register
//! - SDFM_SDCOMPLOCK_COMP - Locks write access to SDCOMPxFILCTL &
//!                          SDCOMPxFILCLKCTL register
//!
//! \return None.
//
//*****************************************************************************
static inline void
SDFM_lockCompEventFilterConfig(uint32_t base, SDFM_FilterNumber filterNumber,
                               uint16_t lockConfig)
{
    ASSERT(SDFM_isBaseValid(base));

    //
    // Lock comparator event filter related configurations.
    //
    EALLOW;
    HWREGH(base + SDFM_O_SDCOMP1LOCK +
           ((uint32_t)filterNumber * SDFM_DIGFIL_OFFSET)) |= lockConfig;
    EDIS;
}

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! Configures SDFM comparator for filter config & threshold values
//!
//! \param base is the base address of the SDFM module
//! \param config1 is the filter number, filter type and over sampling ratio.
//! \param config2 is high-level and low-level threshold 1 values.
//! \param config3 is the zero-cross threshold value.
//!
//! This function configures the comparator filter for filter config and
//! threshold values based on provided inputs.
//!
//! The config1 parameter is the logical OR of the filter number, filter type
//! and oversampling ratio.
//! The bit definitions for config1 are as follow:
//!   - config1.[3:0]  filter number
//!   - config1.[7:4]  filter type
//!   - config1.[15:8] Over sampling Ratio
//! Valid values for filter number and filter type are defined in
//! SDFM_FilterNumber and SDFM_FilterType enumerations respectively.
//! SDFM_SET_OSR(X) macro can be used to set the value of the oversampling
//! ratio ,which ranges [1,32] inclusive, in the appropriate bit location.
//! For example the value
//! (SDFM_FILTER_1 | SDFM_FILTER_SINC_2 | SDFM_SET_OSR(16))
//! will select Filter 1, SINC 2 type with an oversampling ratio of 16.
//!
//! The config2 parameter is the logical OR of the filter high and low
//! threshold 1 values.
//! The bit definitions for config2 are as follow:
//!   - config2.[15:0]  low threshold 1
//!   - config2.[31:16] high threshold 1
//! The upper 16 bits define the high threshold 1 and the lower 16 bits define
//! the low threshold 1. SDFM_THRESHOLD(H,L) can be used to combine the high
//! and low thresholds.
//! The config3 parameter is the logical OR of the zero cross threshold
//! enable flag and the zero-cross threshold value.
//! The bit definitions for config3 are as follow:
//!   - config3.[15] - Enable or disable zero cross threshold. Valid values
//!                    are 1 or 0 to enable or disable the zero cross threshold
//!                    respectively.
//!   -config3.[14:0] - Zero Cross Threshold value.
//! The SDFM_SET_ZERO_CROSS_THRESH_VALUE(X) macro can be used to specify the
//! zero-cross threshold value and OR the 1 to enable it.
//!
//! \return None.
//!
//*****************************************************************************
extern void
SDFM_configComparator(uint32_t base, uint16_t config1,
                      uint32_t config2, uint16_t config3);

//*****************************************************************************
//
//! Configure SDFM enhanced comparator for filter config & threshold values
//!
//! \param base is the base address of the SDFM module
//! \param filterConfig is the filter number, filter type & over sampling ratio.
//! \param highLowThreshold1 is high-level and low-level threshold 1 values.
//! \param highLowThreshold2 is high-level and low-level threshold 2 values.
//! \param zeroCrossThreshold is the zero-cross threshold value.
//!
//! This function configures the comparator filter for filter config and
//! threshold values based on input parameters.
//!
//! The filterConfig parameter is the logical OR of the filter number, filter
//! type and oversampling ratio.
//! The bit definitions for filterConfig are as follow:
//!   - filterConfig.[3:0]  filter number
//!   - filterConfig.[7:4]  filter type
//!   - filterConfig.[15:8] Over sampling Ratio
//! Valid values for filter number and filter type are defined in
//! SDFM_FilterNumber and SDFM_FilterType enumerations respectively.
//! SDFM_SET_OSR(X) macro can be used to set the value of the oversampling
//! ratio ,which ranges [1,32] inclusive, in the appropriate bit location.
//! For example the value
//! (SDFM_FILTER_1 | SDFM_FILTER_SINC_2 | SDFM_SET_OSR(16))
//! will select Filter 1, SINC 2 type with an oversampling ratio of 16.
//!
//! The highLowThreshold1 parameter is the logical OR of the filter high & low
//! threshold 1 values.
//! The bit definitions for highLowThreshold1 are as follow:
//!   - highLowThreshold1.[15:0]  low threshold 1
//!   - highLowThreshold1.[31:16] high threshold 1
//! The upper 16 bits define the high threshold and the lower 16 bits define
//! the low threshold. SDFM_THRESHOLD(H,L) can be used to combine the high and
//! low thresholds.
//!
//! The highLowThreshold2 parameter is the logical OR of the filter high & low
//! threshold 2 values.
//! The bit definitions for highLowThreshold2 are as follow:
//!   - highLowThreshold2.[15:0]  low threshold 2
//!   - highLowThreshold2.[31:16] high threshold 2
//! The upper 16 bits define the high threshold and the lower 16 bits define
//! the low threshold. SDFM_THRESHOLD(H,L) can be used to combine the high &
//! low thresholds.
//!
//! The zeroCrossThreshold parameter is the logical OR of the zero cross
//! threshold enable flag and the zero-cross threshold value. The bit
//! definitions for zeroCrossThreshold are as follows:
//!   - zeroCrossThreshold.[15] - Enable or disable zero cross threshold. Valid
//!     values are 1 or 0 to enable or disable the zero cross threshold
//!     respectively.
//!   - zeroCrossThreshold.[14:0] - Zero Cross Threshold value.
//! The SDFM_SET_ZERO_CROSS_THRESH_VALUE(X) macro can be used as parameter
//! zeroCrossThreshold to enable & specify the zero-cross threshold value.
//!
//! \return None.
//!
//*****************************************************************************
extern void
SDFM_configEnhancedComparator(uint32_t base, uint16_t filterConfig,
                              uint32_t highLowThreshold1,
                              uint32_t highLowThreshold2,
                              uint16_t zeroCrossThreshold);

//*****************************************************************************
//
//! Configure SDFM data filter
//!
//! \param base is the base address of the SDFM module
//! \param config1 is the filter number, filter type and over sampling ratio
//!                configuration.
//! \param config2 is filter switch, data representation and data shift values
//!                configuration.
//!
//! This function configures the data filter based on configurations
//! config1 and config2.
//!
//! The config1 parameter is the logical OR of the filter number, filter type
//! and oversampling ratio.
//! The bit definitions for config1 are as follow:
//!   - config1.[3:0]  Filter number
//!   - config1.[7:4]  Filter type
//!   - config1.[15:8] Over sampling Ratio
//! Valid values for filter number and filter type are defined in
//! SDFM_FilterNumber and SDFM_FilterType enumerations respectively.
//! SDFM_SET_OSR(X) macro can be used to set the value of the oversampling
//! ratio , which ranges [1,256] inclusive , in the appropriate bit location
//! for config1. For example the value
//! (SDFM_FILTER_2 | SDFM_FILTER_SINC_3 | SDFM_SET_OSR(64))
//! will select Filter 2 , SINC 3 type with an oversampling ratio of 64.
//!
//! The config2 parameter is the logical OR of data representation, filter
//! switch, and data shift values
//! The bit definitions for config2 are as follow:
//!   - config2.[0]  Data representation
//!   - config2.[1]  Filter switch
//!   - config2.[15:2]  Shift values
//! Valid values for data representation are given in SDFM_OutputDataFormat
//! enumeration. SDFM_FILTER_DISABLE or SDFM_FILTER_ENABLE will define the
//! filter switch values.SDFM_SHIFT_VALUE(X) macro can be used to set the value
//! of the data shift value,which ranges [0,31] inclusive, in the appropriate
//! bit location for config2.
//! The shift value is valid only in SDFM_DATA_FORMAT_16_BIT data
//! representation format.
//!
//! \return None.
//!
//*****************************************************************************
extern void
SDFM_configDataFilter(uint32_t base, uint16_t config1, uint16_t config2);

//*****************************************************************************
//
//! Configure SDFM comparator Zero Cross threshold
//!
//! \param base is the base address of the SDFM module
//! \param config1 is the filter number, filter type and over sampling ratio.
//! \param config2 is the zero cross threshold value.
//!
//! This function configures the comparator filter zero cross threshold values
//! based on configurations config1 and config2.
//!
//! The config1 parameter is the logical OR of the filter number, filter type
//! and oversampling ratio.
//! The bit definitions for config1 are as follow:
//!   - config1.[3:0]  filter number
//!   - config1.[7:4]  filter type
//!   - config1.[15:8] Over sampling Ratio
//! Valid values for filter number and filter type are defined in
//! SDFM_FilterNumber and SDFM_FilterType enumerations respectively.
//! SDFM_SET_OSR(X) macro can be used to set the value of the oversampling
//! ratio ,which ranges [1,32] inclusive, in the appropriate bit location.
//! For example the value
//! (SDFM_FILTER_1 | SDFM_FILTER_SINC_2 | SDFM_SET_OSR(16))
//! will select Filter 1 , SINC 2 type with an oversampling ratio of 16.
//!
//! The config2 parameter is the value of the zero cross threshold. The maximum
//! acceptable value is 32767.
//!
//! \return None.
//!
//*****************************************************************************
extern void
SDFM_configZeroCrossComparator(uint32_t base, uint16_t config1,
                               uint16_t config2);

//*****************************************************************************
//
//! Configure SDFM data filter FIFO
//!
//! \param base is the base address of the SDFM module
//! \param config1 is the filter number, filter type and over sampling ratio
//!                configuration.
//! \param config2 is filter switch, data representation and data shift values
//!                and FIFO level configuration.
//!
//! This function enables and configures the data filter FIFO based on
//! configurations config1 and config2.
//!
//! The config1 parameter is the logical OR of the filter number, filter type
//! and oversampling ratio.
//! The bit definitions for config1 are as follow:
//!   - config1.[3:0]  filter number
//!   - config1.[7:4]  filter type
//!   - config1.[15:8] Over sampling Ratio
//! Valid values for filter number and filter type are defined in
//! SDFM_FilterNumber and SDFM_FilterType enumerations respectively.
//! SDFM_SET_OSR(X) macro can be used to set the value of the oversampling
//! ratio , which ranges [1,256] inclusive , in the appropriate bit location
//! for config1. For example the value
//! (SDFM_FILTER_2 | SDFM_FILTER_SINC_3 | SDFM_SET_OSR(64))
//! will select Filter 2 , SINC 3 type with an oversampling ratio of 64.
//!
//! The config2 parameter is the logical OR of data representation, filter
//! switch, data shift value, and FIFO level
//! The bit definitions for config2 are as follow:
//!   - config2.[0]  Data representation
//!   - config2.[1]  filter switch.
//!   - config2.[6:2]  shift values.
//!   - config2.[15:7] FIFO level
//! Valid values for data representation are given in SDFM_OutputDataFormat
//! enumeration. SDFM_FILTER_DISABLE or SDFM_FILTER_ENABLE will define the
//! filter switch values.SDFM_SHIFT_VALUE(X) macro can be used to set the value
//! of the data shift value,which ranges [0,31] inclusive, in the appropriate
//! bit location for config2.
//! The value of FIFO level ranges [1,16] inclusive. The macro
//! SDFM_SET_FIFO_LEVEL(X) can be used to set the value of the FIFO level.
//!
//! \return None.
//!
//*****************************************************************************
extern void
SDFM_configDataFilterFIFO(uint32_t base, uint16_t config1, uint16_t config2);

//*****************************************************************************
//
//! Configure Comparator Event Low Filter
//!
//! \param base is the base address of the SDFM module
//! \param filterNumber is the filter number.
//! \param config is the comparator event low source
//!
//! This function configures the sample window, threshold and clock prescale
//! configurations for the comparator event low filter.
//!
//! \return None.
//
//*****************************************************************************
extern void
SDFM_configCompEventLowFilter(uint32_t base, SDFM_FilterNumber filterNumber,
                              const SDFM_CompEventFilterConfig *config);

//*****************************************************************************
//
//! Configure Comparator Event High Filter
//!
//! \param base is the base address of the SDFM module.
//! \param filterNumber is the filter number.
//! \param config is the comparator event high source
//!
//! This function configures the sample window, threshold and clock prescale
//! configurations for the comparator event high filter.
//!
//! \return None.
//
//*****************************************************************************
extern void
SDFM_configCompEventHighFilter(uint32_t base, SDFM_FilterNumber filterNumber,
                               const SDFM_CompEventFilterConfig *config);

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
#endif // SDFM_H
