//###########################################################################
//
// FILE:   sdfm.c
//
// TITLE:  C28x SDFM Driver
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

#include "sdfm.h"

//*****************************************************************************
//
// Defines for filter configurations. Not intended for use by application code.
//
//*****************************************************************************
//
// Get filter oversampling ratio
//
#define SDFM_GET_OSR(C)             ((C) >> 8U)

//
// Maximum acceptable comparator filter oversampling ratio
//
#define SDFM_MAX_COMP_FILTER_OSR    31U

//
// Maximum acceptable data filter oversampling ratio
//
#define SDFM_MAX_DATA_FILTER_OSR    255U

//
// Get the filter type
//
#define SDFM_GET_FILTER_TYPE(C)     ((C) & 0x30U)

//
// Get the filter number
//
#define SDFM_GET_FILTER_NUMBER(C)   ((C) & 0x3U)

//
// Get the zero-crossing high threshold
//
#define SDFM_GET_ZHIGH_THRESHOLD(C)  ((C) & 0x7FFFU))
#define SDFM_ZHIGH_ENABLE_MASK      0x8000U

//
// Get data shift value
//
#define SDFM_GET_SHIFT_VALUE(C)     (((C) >> 2U) & 0x1FU)

//*****************************************************************************
//
// SDFM_configComparator
//
//*****************************************************************************
void SDFM_configComparator(uint32_t base, uint16_t config1, uint32_t config2,
                           uint16_t config3)
{
    SDFM_FilterNumber filter;
    uint16_t ratio;
    SDFM_FilterType filterType;

    filter = (SDFM_FilterNumber)(SDFM_GET_FILTER_NUMBER(config1));
    ratio = SDFM_GET_OSR(config1);
    filterType = (SDFM_FilterType)SDFM_GET_FILTER_TYPE(config1);

    //
    // Limit the oversampling ratio
    //
    if(ratio > SDFM_MAX_COMP_FILTER_OSR)
    {
        ratio = SDFM_MAX_COMP_FILTER_OSR;
    }

    //
    // Set the comparator filter type
    //
    SDFM_setComparatorFilterType(base, filter, filterType);

    //
    // Set the comparator filter over sampling ratio
    //
    SDFM_setCompFilterOverSamplingRatio(base, filter, ratio);

    //
    // Set the comparator high threshold value
    //
    SDFM_setCompFilterHighThreshold(base, filter,
                                    SDFM_GET_HIGH_THRESHOLD(config2));

    //
    // Set the comparator low threshold value
    //
    SDFM_setCompFilterLowThreshold(base, filter,
                                   SDFM_GET_LOW_THRESHOLD(config2));


    //
    // If requested, enable and set the zero-cross high threshold value
    //
    if((config3 & SDFM_ZHIGH_ENABLE_MASK) != 0U)
    {
        SDFM_setCompFilterZeroCrossThreshold(base, filter,
                                             SDFM_GET_ZHIGH_THRESHOLD(config3);
        SDFM_enableZeroCrossEdgeDetect(base, filter);
    }
}

//*****************************************************************************
//
// SDFM_configEnhancedComparator
//
//*****************************************************************************
void SDFM_configEnhancedComparator(uint32_t base, uint16_t filterConfig,
                                   uint32_t highLowThreshold1,
                                   uint32_t highLowThreshold2,
                                   uint16_t zeroCrossThreshold)
{
    SDFM_FilterNumber filter;
    uint16_t ratio;
    SDFM_FilterType filterType;

    filter = (SDFM_FilterNumber)(SDFM_GET_FILTER_NUMBER(filterConfig));
    ratio = SDFM_GET_OSR(filterConfig);
    filterType = (SDFM_FilterType)SDFM_GET_FILTER_TYPE(filterConfig);

    //
    // Limit the oversampling ratio
    //
    if(ratio > SDFM_MAX_COMP_FILTER_OSR)
    {
        ratio = SDFM_MAX_COMP_FILTER_OSR;
    }

    //
    // Set the comparator filter type
    //
    SDFM_setComparatorFilterType(base, filter, filterType);

    //
    // Set the comparator filter over sampling ratio
    //
    SDFM_setCompFilterOverSamplingRatio(base, filter, ratio);

    //
    // Set the comparator high threshold value
    //
    SDFM_setCompFilterHighThreshold(base, filter,
                            SDFM_GET_HIGH_THRESHOLD_BOTH(highLowThreshold1,
                                                         highLowThreshold2));

    //
    // Set the comparator low threshold value
    //
    SDFM_setCompFilterLowThreshold(base, filter,
                            SDFM_GET_LOW_THRESHOLD_BOTH(highLowThreshold1,
                                                         highLowThreshold2));

    //
    // If requested, enable and set the zero-cross high threshold value
    //
    if((zeroCrossThreshold & SDFM_ZHIGH_ENABLE_MASK) != 0U)
    {
        SDFM_setCompFilterZeroCrossThreshold(base, filter,
                                  SDFM_GET_ZHIGH_THRESHOLD(zeroCrossThreshold);
        SDFM_enableZeroCrossEdgeDetect(base, filter);
    }
}
//*****************************************************************************
//
// SDFM_configDataFilter
//
//*****************************************************************************
void SDFM_configDataFilter(uint32_t base, uint16_t config1, uint16_t config2)
{
    SDFM_FilterNumber filter;
    uint16_t ratio;
    SDFM_FilterType filterType;

    filter = (SDFM_FilterNumber)(SDFM_GET_FILTER_NUMBER(config1));
    ratio = SDFM_GET_OSR(config1);
    filterType = (SDFM_FilterType)SDFM_GET_FILTER_TYPE(config1);

    //
    // Limit the oversampling ratio
    //
    if(ratio > SDFM_MAX_DATA_FILTER_OSR)
    {
        ratio = SDFM_MAX_DATA_FILTER_OSR;
    }

    //
    // Set the comparator filter type
    //
    SDFM_setFilterType(base, filter, filterType);

    //
    // Set the comparator filter over sampling ratio
    //
    SDFM_setFilterOverSamplingRatio(base, filter, ratio);

    //
    // If filter switch on
    //
    if((config2 & SDFM_FILTER_ENABLE) == SDFM_FILTER_ENABLE)
    {
        SDFM_enableFilter(base, filter);
    }
    else
    {
        SDFM_disableFilter(base, filter);
    }

    //
    // Set output data format
    //
    SDFM_setOutputDataFormat(base, filter,
                             (SDFM_OutputDataFormat)(config2 & 0x1U));

    //
    // Set the shift value if data is in 16-bit 2's complement format
    //
    if((config2 & 0x1U) == (uint16_t)(SDFM_DATA_FORMAT_16_BIT))
    {
        SDFM_setDataShiftValue(base, filter, SDFM_GET_SHIFT_VALUE(config2));
    }
}

//*****************************************************************************
//
// SDFM_configZeroCrossComparator
//
//*****************************************************************************
void SDFM_configZeroCrossComparator(uint32_t base, uint16_t config1,
                                    uint16_t config2)
{
    SDFM_FilterNumber filter;
    uint16_t ratio;
    SDFM_FilterType filterType;

    filter = (SDFM_FilterNumber)(SDFM_GET_FILTER_NUMBER(config1));
    ratio = SDFM_GET_OSR(config1);
    filterType = (SDFM_FilterType)SDFM_GET_FILTER_TYPE(config1);

    //
    // Limit the oversampling ratio
    //
    if(ratio > SDFM_MAX_COMP_FILTER_OSR)
    {
        ratio = SDFM_MAX_COMP_FILTER_OSR;
    }

    //
    // Set the comparator filter type
    //
    SDFM_setComparatorFilterType(base, filter, filterType);

    //
    // Set the comparator filter over sampling ratio
    //
    SDFM_setCompFilterOverSamplingRatio(base, filter, ratio);

    //
    // Set the zero cross threshold value
    //
    SDFM_setCompFilterZeroCrossThreshold(base, filter, config2);
}

//*****************************************************************************
//
// SDFM_configDataFilterFIFO
//
//*****************************************************************************
void SDFM_configDataFilterFIFO(uint32_t base, uint16_t config1,
                               uint16_t config2)
{
    SDFM_FilterNumber filter;
    uint16_t ratio;
    uint16_t fifoLevel;
    SDFM_FilterType filterType;

    filter = (SDFM_FilterNumber)(SDFM_GET_FILTER_NUMBER(config1));
    fifoLevel = ((config2 >> 7U) & 0x1FU);
    ratio = SDFM_GET_OSR(config1);
    filterType = (SDFM_FilterType)SDFM_GET_FILTER_TYPE(config1);

    //
    // Limit the oversampling ratio
    //
    if(ratio > SDFM_MAX_DATA_FILTER_OSR)
    {
        ratio = SDFM_MAX_DATA_FILTER_OSR;
    }

    //
    // Set the comparator filter type
    //
    SDFM_setFilterType(base, filter, filterType);

    //
    // Set the comparator filter over sampling ratio
    //
    SDFM_setFilterOverSamplingRatio(base, filter, ratio);

    //
    // If filter switch on
    //
    if((config2 & SDFM_FILTER_ENABLE) == SDFM_FILTER_ENABLE)
    {
        SDFM_enableFilter(base, filter);
    }
    else
    {
        SDFM_disableFilter(base, filter);
    }

    //
    // Set output data format
    //
    SDFM_setOutputDataFormat(base, filter,
                             (SDFM_OutputDataFormat)(config2 & 0x1U));

    //
    // Set the shift value if data is in 16bit 2's complement format
    //
    if((config2 & 0x1U) == (uint16_t)(SDFM_DATA_FORMAT_16_BIT))
    {
        SDFM_setDataShiftValue(base, filter, SDFM_GET_SHIFT_VALUE(config2));
    }

    //
    // Set the FIFO level
    //
    SDFM_setFIFOInterruptLevel(base, filter, fifoLevel);
    SDFM_enableFIFOBuffer(base, filter);
}

//*****************************************************************************
//
// SDFM_configCompEventLowFilter
//
//*****************************************************************************
void
SDFM_configCompEventLowFilter(uint32_t base, SDFM_FilterNumber filterNumber,
                              const SDFM_CompEventFilterConfig *config)
{
    uint32_t address;
    ASSERT(SDFM_isBaseValid(base));
    ASSERT(config->clkPrescale <= SDFM_SDCOMP1EVT2FLTCLKCTL_CLKPRESCALE_M);
    ASSERT((config->sampleWindow - 1U) <= (SDFM_SDCOMP1EVT2FLTCTL_SAMPWIN_M >>
                                           SDFM_SDCOMP1EVT2FLTCTL_SAMPWIN_S));
    ASSERT(config->threshold <= (SDFM_SDCOMP1EVT2FLTCTL_THRESH_M >>
                                 SDFM_SDCOMP1EVT2FLTCTL_THRESH_S));
    ASSERT(config->threshold > ((config->sampleWindow - 1U) / 2U));

    address = base + SDFM_O_SDCOMP1EVT2FLTCTL +
              ((uint32_t)filterNumber * SDFM_DIGFIL_OFFSET);

    //
    // Configure sample window & threshold.
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & ~(SDFM_COMPEVT_FILTER_CONFIG_M)) |
           (((config->sampleWindow - 1U) << SDFM_SDCOMP1EVT2FLTCTL_SAMPWIN_S) |
            (config->threshold << SDFM_SDCOMP1EVT2FLTCTL_THRESH_S));

    //
    // Configure clock pre-scale.
    //
    address = base + SDFM_O_SDCOMP1EVT2FLTCLKCTL +
              ((uint32_t)filterNumber * SDFM_DIGFIL_OFFSET);
    HWREGH(address) = (HWREGH(address) &
                       ~(SDFM_SDCOMP1EVT2FLTCLKCTL_CLKPRESCALE_M)) |
                      (config->clkPrescale);
    EDIS;
}

//*****************************************************************************
//
// SDFM_configCompEventHighFilter
//
//*****************************************************************************
void
SDFM_configCompEventHighFilter(uint32_t base, SDFM_FilterNumber filterNumber,
                               const SDFM_CompEventFilterConfig *config)
{
    uint32_t address;
    ASSERT(SDFM_isBaseValid(base));
    ASSERT(config->clkPrescale <= SDFM_SDCOMP1EVT1FLTCLKCTL_CLKPRESCALE_M);
    ASSERT((config->sampleWindow - 1U) <= (SDFM_SDCOMP1EVT1FLTCTL_SAMPWIN_M >>
                                           SDFM_SDCOMP1EVT1FLTCTL_SAMPWIN_S));
    ASSERT(config->threshold <= (SDFM_SDCOMP1EVT1FLTCTL_THRESH_M >>
                                 SDFM_SDCOMP1EVT1FLTCTL_THRESH_S));
    ASSERT(config->threshold > ((config->sampleWindow - 1U) / 2U));

    address = base + SDFM_O_SDCOMP1EVT1FLTCTL +
              ((uint32_t)filterNumber * SDFM_DIGFIL_OFFSET);

    //
    // Configure sample window & threshold.
    //
    EALLOW;
    HWREGH(address) = (HWREGH(address) & ~(SDFM_COMPEVT_FILTER_CONFIG_M)) |
           (((config->sampleWindow - 1U) << SDFM_SDCOMP1EVT1FLTCTL_SAMPWIN_S) |
            (config->threshold << SDFM_SDCOMP1EVT1FLTCTL_THRESH_S));

    //
    // Configure clock pre-scale.
    //
    address = base + SDFM_O_SDCOMP1EVT1FLTCLKCTL +
              ((uint32_t)filterNumber * SDFM_DIGFIL_OFFSET);
    HWREGH(address) =
              (HWREGH(address) & ~(SDFM_SDCOMP1EVT1FLTCLKCTL_CLKPRESCALE_M)) |
                   (config->clkPrescale);
    EDIS;
}


