//#############################################################################
//
// FILE:   pmbus_common.h
//
// TITLE:  C28x PMBUS Driver
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

#ifndef PMBUS_COMMON_H
#define PMBUS_COMMON_H

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
//! \addtogroup pmbus_api PMBus
//! @{
//
//
// Defines for the API.
//
//*****************************************************************************

#ifndef DOXYGEN_PDF_IGNORE


//*****************************************************************************
//
//PMBus Version 1.2 command number constants:
//
//*****************************************************************************
//! PMBus Command PAGE
#define PMBUS_CMD_PAGE                          (0x00U)
//! PMBus Command OPERATION
#define PMBUS_CMD_OPERATION                     (0x01U)
//! PMBus Command ON_OFF_CONFIG
#define PMBUS_CMD_ON_OFF_CONFIG                 (0x02U)
//! PMBus Command CLEAR_FAULTS
#define PMBUS_CMD_CLEAR_FAULTS                  (0x03U)
//! PMBus Command PHASE
#define PMBUS_CMD_PHASE                         (0x04U)
//! PMBus Command PAGE_PLUS_WRITE
#define PMBUS_CMD_PAGE_PLUS_WRITE               (0x05U)
//! PMBus Command PAGE_PLUS_READ
#define PMBUS_CMD_PAGE_PLUS_READ                (0x06U)
                                                // 0x07-0x0F Reserved
//! PMBus Command WRITE_PROTECT
#define PMBUS_CMD_WRITE_PROTECT                 (0x10U)
//! PMBus Command STORE_DEFAULT_ALL
#define PMBUS_CMD_STORE_DEFAULT_ALL             (0x11U)
//! PMBus Command RESTORE_DEFAULT_ALL
#define PMBUS_CMD_RESTORE_DEFAULT_ALL           (0x12U)
//! PMBus Command STORE_DEFAULT_CODE
#define PMBUS_CMD_STORE_DEFAULT_CODE            (0x13U)
//! PMBus Command RESTORE_DEFAULT_CODE
#define PMBUS_CMD_RESTORE_DEFAULT_CODE          (0x14U)
//! PMBus Command STORE_USER_ALL
#define PMBUS_CMD_STORE_USER_ALL                (0x15U)
//! PMBus Command RESTORE_USER_ALL
#define PMBUS_CMD_RESTORE_USER_ALL              (0x16U)
//! PMBus Command STORE_USER_CODE
#define PMBUS_CMD_STORE_USER_CODE               (0x17U)
//! PMBus Command RESTORE_USER_CODE
#define PMBUS_CMD_RESTORE_USER_CODE             (0x18U)
//! PMBus Command CAPABILITY
#define PMBUS_CMD_CAPABILITY                    (0x19U)
//! PMBus Command QUERY
#define PMBUS_CMD_QUERY                         (0x1AU)
//! PMBus Command SMBALERT_MASK
#define PMBUS_CMD_SMBALERT_MASK                 (0x1BU)
                                                // 0x1C - 0x1F Reserved
//! PMBus Command VOUT_MODE
#define PMBUS_CMD_VOUT_MODE                     (0x20U)
//! PMBus Command VOUT_COMMAND
#define PMBUS_CMD_VOUT_COMMAND                  (0x21U)
//! PMBus Command VOUT_TRIM
#define PMBUS_CMD_VOUT_TRIM                     (0x22U)
//! PMBus Command VOUT_CAL_OFFSET
#define PMBUS_CMD_VOUT_CAL_OFFSET               (0x23U)
//! PMBus Command VOUT_MAX
#define PMBUS_CMD_VOUT_MAX                      (0x24U)
//! PMBus Command VOUT_MARGIN_HIGH
#define PMBUS_CMD_VOUT_MARGIN_HIGH              (0x25U)
//! PMBus Command VOUT_MARGIN_LOW
#define PMBUS_CMD_VOUT_MARGIN_LOW               (0x26U)
//! PMBus Command VOUT_TRANSITION_RATE
#define PMBUS_CMD_VOUT_TRANSITION_RATE          (0x27U)
//! PMBus Command VOUT_DROOP
#define PMBUS_CMD_VOUT_DROOP                    (0x28U)
//! PMBus Command VOUT_SCALE_LOOP
#define PMBUS_CMD_VOUT_SCALE_LOOP               (0x29U)
//! PMBus Command VOUT_SCALE_MONITOR
#define PMBUS_CMD_VOUT_SCALE_MONITOR            (0x2AU)
                                                // 0x2B - 0x2F Reserved
//! PMBus Command COEFFICIENTS
#define PMBUS_CMD_COEFFICIENTS                  (0x30U)
//! PMBus Command POUT_MAX
#define PMBUS_CMD_POUT_MAX                      (0x31U)
//! PMBus Command MAX_DUTY
#define PMBUS_CMD_MAX_DUTY                      (0x32U)
//! PMBus Command FREQUENCY_SWITCH
#define PMBUS_CMD_FREQUENCY_SWITCH              (0x33U)
                                                // 0x34 Reserved
//! PMBus Command VIN_ON
#define PMBUS_CMD_VIN_ON                        (0x35U)
//! PMBus Command VIN_OFF
#define PMBUS_CMD_VIN_OFF                       (0x36U)
//! PMBus Command INTERLEAVE
#define PMBUS_CMD_INTERLEAVE                    (0x37U)
//! PMBus Command IOUT_CAL_GAIN
#define PMBUS_CMD_IOUT_CAL_GAIN                 (0x38U)
//! PMBus Command IOUT_CAL_OFFSET
#define PMBUS_CMD_IOUT_CAL_OFFSET               (0x39U)
//! PMBus Command FAN_CONFIG_1_2
#define PMBUS_CMD_FAN_CONFIG_1_2                (0x3AU)
//! PMBus Command FAN_COMMAND_1
#define PMBUS_CMD_FAN_COMMAND_1                 (0x3BU)
//! PMBus Command FAN_COMMAND_2
#define PMBUS_CMD_FAN_COMMAND_2                 (0x3CU)
//! PMBus Command FAN_CONFIG_3_4
#define PMBUS_CMD_FAN_CONFIG_3_4                (0x3DU)
//! PMBus Command FAN_COMMAND_3
#define PMBUS_CMD_FAN_COMMAND_3                 (0x3EU)
//! PMBus Command FAN_COMMAND_4
#define PMBUS_CMD_FAN_COMMAND_4                 (0x3FU)
//! PMBus Command VOUT_OV_FAULT_LIMIT
#define PMBUS_CMD_VOUT_OV_FAULT_LIMIT           (0x40U)
//! PMBus Command VOUT_OV_FAULT_RESPONSE
#define PMBUS_CMD_VOUT_OV_FAULT_RESPONSE        (0x41U)
//! PMBus Command VOUT_OV_WARN_LIMIT
#define PMBUS_CMD_VOUT_OV_WARN_LIMIT            (0x42U)
//! PMBus Command VOUT_UV_WARN_LIMIT
#define PMBUS_CMD_VOUT_UV_WARN_LIMIT            (0x43U)
//! PMBus Command VOUT_UV_FAULT_LIMIT
#define PMBUS_CMD_VOUT_UV_FAULT_LIMIT           (0x44U)
//! PMBus Command VOUT_UV_FAULT_RESPONSE
#define PMBUS_CMD_VOUT_UV_FAULT_RESPONSE        (0x45U)
//! PMBus Command IOUT_OC_FAULT_LIMIT
#define PMBUS_CMD_IOUT_OC_FAULT_LIMIT           (0x46U)
//! PMBus Command IOUT_OC_FAULT_RESPONSE
#define PMBUS_CMD_IOUT_OC_FAULT_RESPONSE        (0x47U)
//! PMBus Command IOUT_OC_LV_FAULT_LIMIT
#define PMBUS_CMD_IOUT_OC_LV_FAULT_LIMIT        (0x48U)
//! PMBus Command IOUT_OC_LV_FAULT_RESPONSE
#define PMBUS_CMD_IOUT_OC_LV_FAULT_RESPONSE     (0x49U)
//! PMBus Command IOUT_OC_WARN_LIMIT
#define PMBUS_CMD_IOUT_OC_WARN_LIMIT            (0x4AU)
//! PMBus Command IOUT_UC_FAULT_LIMIT
#define PMBUS_CMD_IOUT_UC_FAULT_LIMIT           (0x4BU)
//! PMBus Command IOUT_UC_FAULT_RESPONSE
#define PMBUS_CMD_IOUT_UC_FAULT_RESPONSE        (0x4CU)
                                                // 0x4D – 0x4E Reserved
//! PMBus Command OT_FAULT_LIMIT
#define PMBUS_CMD_OT_FAULT_LIMIT                (0x4FU)
//! PMBus Command OT_FAULT_RESPONSE
#define PMBUS_CMD_OT_FAULT_RESPONSE             (0x50U)
//! PMBus Command OT_WARN_LIMIT
#define PMBUS_CMD_OT_WARN_LIMIT                 (0x51U)
//! PMBus Command UT_WARN_LIMIT
#define PMBUS_CMD_UT_WARN_LIMIT                 (0x52U)
//! PMBus Command UT_FAULT_LIMIT
#define PMBUS_CMD_UT_FAULT_LIMIT                (0x53U)
//! PMBus Command UT_FAULT_RESPONSE
#define PMBUS_CMD_UT_FAULT_RESPONSE             (0x54U)
//! PMBus Command VIN_OV_FAULT_LIMIT
#define PMBUS_CMD_VIN_OV_FAULT_LIMIT            (0x55U)
//! PMBus Command VIN_OV_FAULT_RESPONSE
#define PMBUS_CMD_VIN_OV_FAULT_RESPONSE         (0x56U)
//! PMBus Command VIN_OV_WARN_LIMIT
#define PMBUS_CMD_VIN_OV_WARN_LIMIT             (0x57U)
//! PMBus Command VIN_UV_WARN_LIMIT
#define PMBUS_CMD_VIN_UV_WARN_LIMIT             (0x58U)
//! PMBus Command VIN_UV_FAULT_LIMIT
#define PMBUS_CMD_VIN_UV_FAULT_LIMIT            (0x59U)
//! PMBus Command VIN_UV_FAULT_RESPONSE
#define PMBUS_CMD_VIN_UV_FAULT_RESPONSE         (0x5AU)
//! PMBus Command IIN_OC_FAULT_LIMIT (For CBC current limit)
#define PMBUS_CMD_IIN_OC_FAULT_LIMIT            (0x5BU)
//! PMBus Command IIN_OC_FAULT_RESPONSE
#define PMBUS_CMD_IIN_OC_FAULT_RESPONSE         (0x5CU)
//! PMBus Command IIN_OC_WARN_LIMIT
#define PMBUS_CMD_IIN_OC_WARN_LIMIT             (0x5DU)
//! PMBus Command POWER_GOOD_ON
#define PMBUS_CMD_POWER_GOOD_ON                 (0x5EU)
//! PMBus Command POWER_GOOD_OFF
#define PMBUS_CMD_POWER_GOOD_OFF                (0x5FU)
//! PMBus Command TON_DELAY
#define PMBUS_CMD_TON_DELAY                     (0x60U)
//! PMBus Command TON_RISE
#define PMBUS_CMD_TON_RISE                      (0x61U)
//! PMBus Command TON_MAX_FAULT_LIMIT
#define PMBUS_CMD_TON_MAX_FAULT_LIMIT           (0x62U)
//! PMBus Command TON_MAX_FAULT_RESPONSE
#define PMBUS_CMD_TON_MAX_FAULT_RESPONSE        (0x63U)
//! PMBus Command TOFF_DELAY
#define PMBUS_CMD_TOFF_DELAY                    (0x64U)
//! PMBus Command TOFF_FALL
#define PMBUS_CMD_TOFF_FALL                     (0x65U)
//! PMBus Command TOFF_MAX_WARN_LIMIT
#define PMBUS_CMD_TOFF_MAX_WARN_LIMIT           (0x66U)
                                                // 0x67 Rsvd Deleted PMBus v1.1
//! PMBus Command POUT_OP_FAULT_LIMIT
#define PMBUS_CMD_POUT_OP_FAULT_LIMIT           (0x68U)
//! PMBus Command POUT_OP_FAULT_RESPONSE
#define PMBUS_CMD_POUT_OP_FAULT_RESPONSE        (0x69U)
//! PMBus Command POUT_OP_WARN_LIMIT
#define PMBUS_CMD_POUT_OP_WARN_LIMIT            (0x6AU)
//! PMBus Command PIN_OP_WARN_LIMIT
#define PMBUS_CMD_PIN_OP_WARN_LIMIT             (0x6BU)
                                                // 0x6C - 0x77 Reserved
//! PMBus Command STATUS_BYTE
#define PMBUS_CMD_STATUS_BYTE                   (0x78U)
//! PMBus Command STATUS_WORD
#define PMBUS_CMD_STATUS_WORD                   (0x79U)
//! PMBus Command STATUS_VOUT
#define PMBUS_CMD_STATUS_VOUT                   (0x7AU)
//! PMBus Command STATUS_IOUT
#define PMBUS_CMD_STATUS_IOUT                   (0x7BU)
//! PMBus Command STATUS_INPUT
#define PMBUS_CMD_STATUS_INPUT                  (0x7CU)
//! PMBus Command STATUS_TEMPERATURE
#define PMBUS_CMD_STATUS_TEMPERATURE            (0x7DU)
//! PMBus Command STATUS_CML
#define PMBUS_CMD_STATUS_CML                    (0x7EU)
//! PMBus Command STATUS_OTHER
#define PMBUS_CMD_STATUS_OTHER                  (0x7FU)
//! PMBus Command STATUS_MFR_SPECIFIC
#define PMBUS_CMD_STATUS_MFR_SPECIFIC           (0x80U)
//! PMBus Command STATUS_FANS_1_2
#define PMBUS_CMD_STATUS_FANS_1_2               (0x81U)
//! PMBus Command STATUS_FANS_3_4
#define PMBUS_CMD_STATUS_FANS_3_4               (0x82U)
                                                // 0x83 - 0x85 Reserved
//! PMBus Command READ_EIN
#define PMBUS_CMD_READ_EIN                      (0x86U)
//! PMBus Command READ_EOUT
#define PMBUS_CMD_READ_EOUT                     (0x87U)
//! PMBus Command READ_VIN
#define PMBUS_CMD_READ_VIN                      (0x88U)
//! PMBus Command READ_IIN
#define PMBUS_CMD_READ_IIN                      (0x89U)
//! PMBus Command READ_VCAP
#define PMBUS_CMD_READ_VCAP                     (0x8AU)
//! PMBus Command READ_VOUT
#define PMBUS_CMD_READ_VOUT                     (0x8BU)
//! PMBus Command READ_IOUT
#define PMBUS_CMD_READ_IOUT                     (0x8CU)
//! PMBus Command READ_TEMPERATURE_1
#define PMBUS_CMD_READ_TEMPERATURE_1            (0x8DU)
//! PMBus Command READ_TEMPERATURE_2
#define PMBUS_CMD_READ_TEMPERATURE_2            (0x8EU)
//! PMBus Command READ_TEMPERATURE_3
#define PMBUS_CMD_READ_TEMPERATURE_3            (0x8FU)
//! PMBus Command READ_FAN_SPEED_1
#define PMBUS_CMD_READ_FAN_SPEED_1              (0x90U)
//! PMBus Command READ_FAN_SPEED_2
#define PMBUS_CMD_READ_FAN_SPEED_2              (0x91U)
//! PMBus Command READ_FAN_SPEED_3
#define PMBUS_CMD_READ_FAN_SPEED_3              (0x92U)
//! PMBus Command READ_FAN_SPEED_4
#define PMBUS_CMD_READ_FAN_SPEED_4              (0x93U)
//! PMBus Command READ_DUTY_CYCLE
#define PMBUS_CMD_READ_DUTY_CYCLE               (0x94U)
//! PMBus Command READ_FREQUENCY
#define PMBUS_CMD_READ_FREQUENCY                (0x95U)
//! PMBus Command READ_POUT
#define PMBUS_CMD_READ_POUT                     (0x96U)
//! PMBus Command READ_PIN
#define PMBUS_CMD_READ_PIN                      (0x97U)
//! PMBus Command PMBUS_REVISION
#define PMBUS_CMD_PMBUS_REVISION                (0x98U)
//! PMBus Command MFR_ID
#define PMBUS_CMD_MFR_ID                        (0x99U)
//! PMBus Command MFR_MODEL
#define PMBUS_CMD_MFR_MODEL                     (0x9AU)
//! PMBus Command MFR_REVISION
#define PMBUS_CMD_MFR_REVISION                  (0x9BU)
//! PMBus Command MFR_LOCATION
#define PMBUS_CMD_MFR_LOCATION                  (0x9CU)
//! PMBus Command MFR_DATE
#define PMBUS_CMD_MFR_DATE                      (0x9DU)
//! PMBus Command MFR_SERIAL
#define PMBUS_CMD_MFR_SERIAL                    (0x9EU)
//! PMBus Command APP_PROFILE_SUPPORT
#define PMBUS_CMD APP_PROFILE_SUPPORT           (0x9FU)
//! PMBus Command MFR_VIN_MIN
#define PMBUS_CMD_MFR_VIN_MIN                   (0xA0U)
//! PMBus Command MFR_VIN_MAX
#define PMBUS_CMD_MFR_VIN_MAX                   (0xA1U)
//! PMBus Command MFR_IIN_MAX
#define PMBUS_CMD_MFR_IIN_MAX                   (0xA2U)
//! PMBus Command MFR_PIN_MAX
#define PMBUS_CMD_MFR_PIN_MAX                   (0xA3U)
//! PMBus Command MFR_VOUT_MIN
#define PMBUS_CMD_MFR_VOUT_MIN                  (0xA4U)
//! PMBus Command MFR_VOUT_MAX
#define PMBUS_CMD_MFR_VOUT_MAX                  (0xA5U)
//! PMBus Command MFR_IOUT_MAX
#define PMBUS_CMD_MFR_IOUT_MAX                  (0xA6U)
//! PMBus Command MFR_POUT_MAX
#define PMBUS_CMD_MFR_POUT_MAX                  (0xA7U)
//! PMBus Command MFR_TAMBIENT_MAX
#define PMBUS_CMD_MFR_TAMBIENT_MAX              (0xA8U)
//! PMBus Command MFR_TAMBIENT_MIN
#define PMBUS_CMD_MFR_TAMBIENT_MIN              (0xA9U)
//! PMBus Command MFR_EFFICIENCY_LL
#define PMBUS_CMD_MFR_EFFICIENCY_LL             (0xAAU)
//! PMBus Command MFR_EFFICIENCY_HL
#define PMBUS_CMD_MFR_EFFICIENCY_HL             (0xABU)
//! PMBus Command MFR_PIN_ACURRACY
#define PMBUS_CMD_MFR_PIN_ACURRACY              (0xACU)
//! PMBus Command MFR_IC_DEVICE
#define PMBUS_CMD_MFR_IC_DEVICE                 (0xADU)
//! PMBus Command MFR_IC_DEVICE_REV
#define PMBUS_CMD_MFR_IC_DEVICE_REV             (0xAEU)
                                                // 0xAF Reserved
//! PMBus Command USER_DATA_00
#define PMBUS_CMD_USER_DATA_00                  (0xB0U)
//! PMBus Command USER_DATA_01
#define PMBUS_CMD_USER_DATA_01                  (0xB1U)
//! PMBus Command USER_DATA_02
#define PMBUS_CMD_USER_DATA_02                  (0xB2U)
//! PMBus Command USER_DATA_03
#define PMBUS_CMD_USER_DATA_03                  (0xB3U)
//! PMBus Command USER_DATA_04
#define PMBUS_CMD_USER_DATA_04                  (0xB4U)
//! PMBus Command USER_DATA_05
#define PMBUS_CMD_USER_DATA_05                  (0xB5U)
//! PMBus Command USER_DATA_06
#define PMBUS_CMD_USER_DATA_06                  (0xB6U)
//! PMBus Command USER_DATA_07
#define PMBUS_CMD_USER_DATA_07                  (0xB7U)
//! PMBus Command USER_DATA_08
#define PMBUS_CMD_USER_DATA_08                  (0xB8U)
//! PMBus Command USER_DATA_09
#define PMBUS_CMD_USER_DATA_09                  (0xB9U)
//! PMBus Command USER_DATA_10
#define PMBUS_CMD_USER_DATA_10                  (0xBAU)
//! PMBus Command USER_DATA_11
#define PMBUS_CMD_USER_DATA_11                  (0xBBU)
//! PMBus Command USER_DATA_12
#define PMBUS_CMD_USER_DATA_12                  (0xBCU)
//! PMBus Command USER_DATA_13
#define PMBUS_CMD_USER_DATA_13                  (0xBDU)
//! PMBus Command USER_DATA_14
#define PMBUS_CMD_USER_DATA_14                  (0xBEU)
//! PMBus Command USER_DATA_15
#define PMBUS_CMD_USER_DATA_15                  (0xBFU)
//! PMBus Command MFR_MAX_TEMP_1
#define PMBUS_CMD_MFR_MAX_TEMP_1                (0xC0U)
//! PMBus Command MFR_MAX_TEMP_2
#define PMBUS_CMD_MFR_MAX_TEMP_2                (0xC1U)
//! PMBus Command MFR_MAX_TEMP_3
#define PMBUS_CMD_MFR_MAX_TEMP_3                (0xC2U)
                                                // 0xC3-0xCF Reserved
//! PMBus Command MFR_LIGHT_LOAD_ENB
#define PMBUS_CMD_MFR_LIGHT_LOAD_ENB            (0xD0U)
//! PMBus Command MFR_SPECIFIC_01
#define PMBUS_CMD_MFR_SPECIFIC_01               (0xD1U)
//! PMBus Command MFR_SPECIFIC_02
#define PMBUS_CMD_MFR_SPECIFIC_02               (0xD2U)
//! PMBus Command MFR_SPECIFIC_03
#define PMBUS_CMD_MFR_SPECIFIC_03               (0xD3U)
//! PMBus Command MFR_SPECIFIC_04
#define PMBUS_CMD_MFR_SPECIFIC_04               (0xD4U)
//! PMBus Command MFR_SPECIFIC_05
#define PMBUS_CMD_MFR_SPECIFIC_05               (0xD5U)
//! PMBus Command MFR_SPECIFIC_06
#define PMBUS_CMD_MFR_SPECIFIC_06               (0xD6U)
//! PMBus Command MFR_SPECIFIC_07
#define PMBUS_CMD_MFR_SPECIFIC_07               (0xD7U)
//! PMBus Command MFR_SPECIFIC_08
#define PMBUS_CMD_MFR_SPECIFIC_08               (0xD8U)
//! PMBus Command ROM_MODE
#define PMBUS_CMD_ROM_MODE                      (0xD9U)
//! PMBus Command USER_RAM_00
#define PMBUS_CMD_USER_RAM_00                   (0xDAU)
//! PMBus Command MFR_PHASE_CONTROL
#define PMBUS_CMD_MFR_PHASE_CONTROL             (0xDBU)
//! PMBus Command MFR_IOUT_OC_FAULT_LIMIT_LOW
#define PMBUS_CMD_MFR_IOUT_OC_FAULT_LIMIT_LOW   (0xDCU)
//! PMBus Command MFR_VIN_SCALE
#define PMBUS_CMD_MFR_VIN_SCALE                 (0xDDU)
//! PMBus Command MFR_VIN_OFFSET
#define PMBUS_CMD_MFR_VIN_OFFSET                (0xDEU)
//! PMBus Command MFR_READ_TEMPERATURE_4
#define PMBUS_CMD_MFR_READ_TEMPERATURE_4        (0xDFU)
//! PMBus Command MFR_OT_LIMIT_1
#define PMBUS_CMD_MFR_OT_LIMIT_1                (0xE0U)
//! PMBus Command MFR_OT_LIMIT_2
#define PMBUS_CMD_MFR_OT_LIMIT_2                (0xE1U)
//! PMBus Command MFR_PARM_INFO
#define PMBUS_CMD_MFR_PARM_INFO                 (0xE2U)
//! PMBus Command MFR_PARM_VALUE
#define PMBUS_CMD_MFR_PARM_VALUE                (0xE3U)
//! PMBus Command MFR_CMDS_DCDC_PAGED
#define PMBUS_CMD_MFR_CMDS_DCDC_PAGED           (0xE4U)
//! PMBus Command MFR_CMDS_DCDC_NONPAGED
#define PMBUS_CMD_MFR_CMDS_DCDC_NONPAGED        (0xE5U)
//! PMBus Command MFR_CMDS_PFC
#define PMBUS_CMD_MFR_CMDS_PFC                  (0xE6U)
//! PMBus Command MFR_SETUP_ID
#define PMBUS_CMD_MFR_SETUP_ID                  (0xE7U)
//! PMBus Command MFR_OT_LIMIT_3
#define PMBUS_CMD_MFR_OT_LIMIT_3                (0xE8U)
//! PMBus Command MFR_OT_LIMIT_4
#define PMBUS_CMD_MFR_OT_LIMIT_4                (0xE9U)
//! PMBus Command MFR_DEADBAND_CONFIG
#define PMBUS_CMD_MFR_DEADBAND_CONFIG           (0xEAU)
//! PMBus Command MFR_PIN_CAL_A
#define PMBUS_CMD_MFR_PIN_CAL_A                 (0xEBU)
//! PMBus Command MFR_PIN_CAL_B
#define PMBUS_CMD_MFR_PIN_CAL_B                 (0xECU)
//! PMBus Command MFR_PIN_CAL_C
#define PMBUS_CMD_MFR_PIN_CAL_C                 (0xEDU)
//! PMBus Command MFR_PIN_CAL_D
#define PMBUS_CMD_MFR_PIN_CAL_D                 (0xEEU)
//! PMBus Command MFR_TEMP_CAL_OFFSET
#define PMBUS_CMD_MFR_TEMP_CAL_OFFSET           (0xEFU)
//! PMBus Command MFR_DEBUG_BUFFER
#define PMBUS_CMD_MFR_DEBUG_BUFFER              (0xF0U)
//! PMBus Command MFR_TEMP_CAL_GAIN
#define PMBUS_CMD_MFR_TEMP_CAL_GAIN             (0xF1U)
//! PMBus Command MFR_STATUS_BIT_MASK
#define PMBUS_CMD_MFR_STATUS_BIT_MASK           (0xF2U)
//! PMBus Command MFR_SPECIFIC_35
#define PMBUS_CMD_MFR_SPECIFIC_35               (0xF3U)
//! PMBus Command MFR_SPECIFIC_36
#define PMBUS_CMD_MFR_SPECIFIC_36               (0xF4U)
//! PMBus Command MFR_SPECIFIC_37
#define PMBUS_CMD_MFR_SPECIFIC_37               (0xF5U)
//! PMBus Command MFR_SPECIFIC_38
#define PMBUS_CMD_MFR_SPECIFIC_38               (0xF6U)
//! PMBus Command MFR_SPECIFIC_39
#define PMBUS_CMD_MFR_SPECIFIC_39               (0xF7U)
//! PMBus Command MFR_VOUT_CAL_MONITOR
#define PMBUS_CMD_MFR_VOUT_CAL_MONITOR          (0xF8U)
//! PMBus Command ROM_MODE_WITH_PASSWORD
#define PMBUS_CMD_ROM_MODE_WITH_PASSWORD        (0xF9U)
//! PMBus Command MFR_SPECIFIC_42
#define PMBUS_CMD_MFR_SPECIFIC_42               (0xFAU)
//! PMBus Command MFR_SPECIFIC_43
#define PMBUS_CMD_MFR_SPECIFIC_43               (0xFBU)
//! PMBus Command MFR_SPECIFIC_44
#define PMBUS_CMD_MFR_SPECIFIC_44               (0xFCU)
//! PMBus Command MFR_DEVICE_ID
#define PMBUS_CMD_MFR_DEVICE_ID                 (0xFDU)
//! PMBus Command MFR_SPECIFIC_COMMAND
#define PMBUS_CMD_MFR_SPECIFIC_COMMAND          (0xFEU)
//! PMBus Command PMBUS_COMMAND_EXT
#define PMBUS_CMD_PMBUS_COMMAND_EXT             (0xFFU)
#endif //DOXYGEN_PDF_IGNORE

//*****************************************************************************
//
//! Transaction Descriptor
//!
//! Defines the transaction type, used in the command object
//! and passed to PMBus_configTransfer()
//
//*****************************************************************************
typedef enum{
  PMBUS_TRANSACTION_NONE          = 0U,  //!< No Transaction
  PMBUS_TRANSACTION_QUICKCOMMAND  = 1U,  //!< Quick Command
  PMBUS_TRANSACTION_WRITEBYTE     = 2U,  //!< Write single byte
  PMBUS_TRANSACTION_READBYTE      = 3U,  //!< Read single byte
  PMBUS_TRANSACTION_SENDBYTE      = 4U,  //!< Send Byte
  PMBUS_TRANSACTION_RECEIVEBYTE   = 5U,  //!< Receive Byte
  PMBUS_TRANSACTION_BLOCKWRITE    = 6U,  //!< Block Write (up to 255 bytes)
  PMBUS_TRANSACTION_BLOCKREAD     = 7U,  //!< Block Read (up to 255 bytes)
  PMBUS_TRANSACTION_WRITEWORD     = 8U,  //!< Write word
  PMBUS_TRANSACTION_READWORD      = 9U,  //!< Read word
  PMBUS_TRANSACTION_BLOCKWRPC     = 10U  //!< Block write, then process call
}PMBus_Transaction;

#ifdef __cplusplus
}
#endif
#endif // PMBUS_COMMON_H
