//###########################################################################
//
// FILE:    hw_clb.h
//
// TITLE:   Definitions for the CLB registers.
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

#ifndef HW_CLB_H
#define HW_CLB_H

//*************************************************************************************************
//
// The following are defines for the CLB register offsets
//
//*************************************************************************************************
#define CLB_O_COUNT_RESET           0x2U    // Counter Block RESET
#define CLB_O_COUNT_MODE_1          0x4U    // Counter Block MODE_1
#define CLB_O_COUNT_MODE_0          0x6U    // Counter Block MODE_0
#define CLB_O_COUNT_EVENT           0x8U    // Counter Block EVENT
#define CLB_O_FSM_EXTRA_IN0         0xAU    // FSM Extra EXT_IN0
#define CLB_O_FSM_EXTERNAL_IN0      0xCU    // FSM EXT_IN0
#define CLB_O_FSM_EXTERNAL_IN1      0xEU    // FSM_EXT_IN1
#define CLB_O_FSM_EXTRA_IN1         0x10U   // FSM Extra_EXT_IN1
#define CLB_O_LUT4_IN0              0x12U   // LUT4_0/1/2 IN0 input source
#define CLB_O_LUT4_IN1              0x14U   // LUT4_0/1/2 IN1 input source
#define CLB_O_LUT4_IN2              0x16U   // LUT4_0/1/2 IN2 input source
#define CLB_O_LUT4_IN3              0x18U   // LUT4_0/1/2 IN3 input source
#define CLB_O_FSM_LUT_FN1_0         0x1CU   // LUT function for FSM Unit 1 and Unit 0
#define CLB_O_FSM_LUT_FN2           0x1EU   // LUT function for FSM Unit 2
#define CLB_O_LUT4_FN1_0            0x20U   // LUT function for LUT4 block of Unit 1 and 0
#define CLB_O_LUT4_FN2              0x22U   // LUT function for LUT4 block of Unit 2
#define CLB_O_FSM_NEXT_STATE_0      0x24U   // FSM Next state equations for Unit 0
#define CLB_O_FSM_NEXT_STATE_1      0x26U   // FSM Next state equations for Unit 1
#define CLB_O_FSM_NEXT_STATE_2      0x28U   // FSM Next state equations for Unit 2
#define CLB_O_MISC_CONTROL          0x2AU   // Static controls for Ctr,FSM
#define CLB_O_OUTPUT_LUT_0          0x2CU   // Inp Sel, LUT fns for Out0
#define CLB_O_OUTPUT_LUT_1          0x2EU   // Inp Sel, LUT fns for Out1
#define CLB_O_OUTPUT_LUT_2          0x30U   // Inp Sel, LUT fns for Out2
#define CLB_O_OUTPUT_LUT_3          0x32U   // Inp Sel, LUT fns for Out3
#define CLB_O_OUTPUT_LUT_4          0x34U   // Inp Sel, LUT fns for Out4
#define CLB_O_OUTPUT_LUT_5          0x36U   // Inp Sel, LUT fns for Out5
#define CLB_O_OUTPUT_LUT_6          0x38U   // Inp Sel, LUT fns for Out6
#define CLB_O_OUTPUT_LUT_7          0x3AU   // Inp Sel, LUT fns for Out7
#define CLB_O_HLC_EVENT_SEL         0x3CU   // Event Selector register for the High Level
                                            // controller
#define CLB_O_COUNT_MATCH_TAP_SEL   0x3EU   // Counter tap values for match1 and match2 outputs
#define CLB_O_OUTPUT_COND_CTRL_0    0x40U   // Output conditioning control for output 0
#define CLB_O_OUTPUT_COND_CTRL_1    0x42U   // Output conditioning control for output 1
#define CLB_O_OUTPUT_COND_CTRL_2    0x44U   // Output conditioning control for output 2
#define CLB_O_OUTPUT_COND_CTRL_3    0x46U   // Output conditioning control for output 3
#define CLB_O_OUTPUT_COND_CTRL_4    0x48U   // Output conditioning control for output 4
#define CLB_O_OUTPUT_COND_CTRL_5    0x4AU   // Output conditioning control for output 5
#define CLB_O_OUTPUT_COND_CTRL_6    0x4CU   // Output conditioning control for output 6
#define CLB_O_OUTPUT_COND_CTRL_7    0x4EU   // Output conditioning control for output 7
#define CLB_O_MISC_ACCESS_CTRL      0x50U   // Miscellaneous Access and enable control
#define CLB_O_SPI_DATA_CTRL_HI      0x51U   // CLB to SPI buffer control High

#define CLB_O_LOAD_EN              0x0U    // Global enable & indirect load enable control
#define CLB_O_LOAD_ADDR            0x2U    // Indirect address
#define CLB_O_LOAD_DATA            0x4U    // Data for indirect loads
#define CLB_O_INPUT_FILTER         0x6U    // Input filter selection for both edge detection and
                                           // synchronizers
#define CLB_O_IN_MUX_SEL_0         0x8U    // Input selection to decide between Signals and GP
                                           // register
#define CLB_O_LCL_MUX_SEL_1        0xAU    // Input Mux selection for local mux
#define CLB_O_LCL_MUX_SEL_2        0xCU    // Input Mux selection for local mux
#define CLB_O_BUF_PTR              0xEU    // PUSH and PULL pointers
#define CLB_O_GP_REG               0x10U   // General purpose register for CELL inputs
#define CLB_O_OUT_EN               0x12U   // CELL output enable register
#define CLB_O_GLBL_MUX_SEL_1       0x14U   // Global Mux select for CELL inputs
#define CLB_O_GLBL_MUX_SEL_2       0x16U   // Global Mux select for CELL inputs
#define CLB_O_PRESCALE_CTRL        0x18U   // Prescaler register control
#define CLB_O_INTR_TAG_REG         0x20U   // Interrupt Tag register
#define CLB_O_LOCK                 0x22U   // Lock control register
#define CLB_O_HLC_INSTR_READ_PTR   0x24U   // HLC instruction read pointer
#define CLB_O_HLC_INSTR_VALUE      0x26U   // HLC instruction read value
#define CLB_O_DBG_OUT_2            0x2EU   // Visibility for CLB inputs and final  asynchronous
                                           // outputs
#define CLB_O_DBG_R0               0x30U   // R0 of High level Controller
#define CLB_O_DBG_R1               0x32U   // R1 of High level Controller
#define CLB_O_DBG_R2               0x34U   // R2 of High level Controller
#define CLB_O_DBG_R3               0x36U   // R3 of High level Controller
#define CLB_O_DBG_C0               0x38U   // Count of Unit 0
#define CLB_O_DBG_C1               0x3AU   // Count of Unit 1
#define CLB_O_DBG_C2               0x3CU   // Count of Unit 2
#define CLB_O_DBG_OUT              0x3EU   // Outputs of various units in the Cell

#define CLB_O_PUSH(i)   (0x0U + ((i) * 0x2U))    // (0 <= i < 4) CLB_PUSH FIFO Registers (from HLC)
#define CLB_O_PULL(i)   (0x40U + ((i) * 0x2U))   // (0 <= i < 4) CLB_PULL FIFO Registers (TO HLC)


//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_COUNT_RESET register
//
//*************************************************************************************************
#define CLB_COUNT_RESET_SEL_0_S   0U
#define CLB_COUNT_RESET_SEL_0_M   0x1FU     // Count Reset Select 0
#define CLB_COUNT_RESET_SEL_1_S   5U
#define CLB_COUNT_RESET_SEL_1_M   0x3E0U    // Count Reset Select 1
#define CLB_COUNT_RESET_SEL_2_S   10U
#define CLB_COUNT_RESET_SEL_2_M   0x7C00U   // Count Reset Select 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_COUNT_MODE_1 register
//
//*************************************************************************************************
#define CLB_COUNT_MODE_1_SEL_0_S   0U
#define CLB_COUNT_MODE_1_SEL_0_M   0x1FU     // Counter mode 1 select 0
#define CLB_COUNT_MODE_1_SEL_1_S   5U
#define CLB_COUNT_MODE_1_SEL_1_M   0x3E0U    // Counter mode 1 select 1
#define CLB_COUNT_MODE_1_SEL_2_S   10U
#define CLB_COUNT_MODE_1_SEL_2_M   0x7C00U   // Counter mode 1 select 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_COUNT_MODE_0 register
//
//*************************************************************************************************
#define CLB_COUNT_MODE_0_SEL_0_S   0U
#define CLB_COUNT_MODE_0_SEL_0_M   0x1FU     // Counter mode 0 select 0
#define CLB_COUNT_MODE_0_SEL_1_S   5U
#define CLB_COUNT_MODE_0_SEL_1_M   0x3E0U    // Counter mode 0 select 1
#define CLB_COUNT_MODE_0_SEL_2_S   10U
#define CLB_COUNT_MODE_0_SEL_2_M   0x7C00U   // Counter mode 0 select 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_COUNT_EVENT register
//
//*************************************************************************************************
#define CLB_COUNT_EVENT_SEL_0_S   0U
#define CLB_COUNT_EVENT_SEL_0_M   0x1FU     // Counter event select 0
#define CLB_COUNT_EVENT_SEL_1_S   5U
#define CLB_COUNT_EVENT_SEL_1_M   0x3E0U    // Counter event select 1
#define CLB_COUNT_EVENT_SEL_2_S   10U
#define CLB_COUNT_EVENT_SEL_2_M   0x7C00U   // Counter event select 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_FSM_EXTRA_IN0 register
//
//*************************************************************************************************
#define CLB_FSM_EXTRA_IN0_SEL_0_S   0U
#define CLB_FSM_EXTRA_IN0_SEL_0_M   0x1FU     // FSM extra ext input select 0
#define CLB_FSM_EXTRA_IN0_SEL_1_S   5U
#define CLB_FSM_EXTRA_IN0_SEL_1_M   0x3E0U    // FSM extra ext input select 1
#define CLB_FSM_EXTRA_IN0_SEL_2_S   10U
#define CLB_FSM_EXTRA_IN0_SEL_2_M   0x7C00U   // FSM extra ext input select 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_FSM_EXTERNAL_IN0 register
//
//*************************************************************************************************
#define CLB_FSM_EXTERNAL_IN0_SEL_0_S   0U
#define CLB_FSM_EXTERNAL_IN0_SEL_0_M   0x1FU     // FSM EXT_IN0 select input for unit 0
#define CLB_FSM_EXTERNAL_IN0_SEL_1_S   5U
#define CLB_FSM_EXTERNAL_IN0_SEL_1_M   0x3E0U    // FSM EXT_IN0 select input for unit 1
#define CLB_FSM_EXTERNAL_IN0_SEL_2_S   10U
#define CLB_FSM_EXTERNAL_IN0_SEL_2_M   0x7C00U   // FSM EXT_IN0 select input for unit 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_FSM_EXTERNAL_IN1 register
//
//*************************************************************************************************
#define CLB_FSM_EXTERNAL_IN1_SEL_0_S   0U
#define CLB_FSM_EXTERNAL_IN1_SEL_0_M   0x1FU     // FSM EXT_IN1 select input for unit 0
#define CLB_FSM_EXTERNAL_IN1_SEL_1_S   5U
#define CLB_FSM_EXTERNAL_IN1_SEL_1_M   0x3E0U    // FSM EXT_IN1 select input for unit 1
#define CLB_FSM_EXTERNAL_IN1_SEL_2_S   10U
#define CLB_FSM_EXTERNAL_IN1_SEL_2_M   0x7C00U   // FSM EXT_IN1 select input for unit 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_FSM_EXTRA_IN1 register
//
//*************************************************************************************************
#define CLB_FSM_EXTRA_IN1_SEL_0_S   0U
#define CLB_FSM_EXTRA_IN1_SEL_0_M   0x1FU     // FSM extra ext input select 0
#define CLB_FSM_EXTRA_IN1_SEL_1_S   5U
#define CLB_FSM_EXTRA_IN1_SEL_1_M   0x3E0U    // FSM extra ext input select 1
#define CLB_FSM_EXTRA_IN1_SEL_2_S   10U
#define CLB_FSM_EXTRA_IN1_SEL_2_M   0x7C00U   // FSM extra ext input select 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_LUT4_IN0 register
//
//*************************************************************************************************
#define CLB_LUT4_IN0_SEL_0_S   0U
#define CLB_LUT4_IN0_SEL_0_M   0x1FU     // Select inputs for unit 0
#define CLB_LUT4_IN0_SEL_1_S   5U
#define CLB_LUT4_IN0_SEL_1_M   0x3E0U    // Select inputs for unit 1
#define CLB_LUT4_IN0_SEL_2_S   10U
#define CLB_LUT4_IN0_SEL_2_M   0x7C00U   // Select inputs for unit 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_LUT4_IN1 register
//
//*************************************************************************************************
#define CLB_LUT4_IN1_SEL_0_S   0U
#define CLB_LUT4_IN1_SEL_0_M   0x1FU     // Select inputs for unit 0
#define CLB_LUT4_IN1_SEL_1_S   5U
#define CLB_LUT4_IN1_SEL_1_M   0x3E0U    // Select inputs for unit 1
#define CLB_LUT4_IN1_SEL_2_S   10U
#define CLB_LUT4_IN1_SEL_2_M   0x7C00U   // Select inputs for unit 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_LUT4_IN2 register
//
//*************************************************************************************************
#define CLB_LUT4_IN2_SEL_0_S   0U
#define CLB_LUT4_IN2_SEL_0_M   0x1FU     // Select inputs for unit 0
#define CLB_LUT4_IN2_SEL_1_S   5U
#define CLB_LUT4_IN2_SEL_1_M   0x3E0U    // Select inputs for unit 1
#define CLB_LUT4_IN2_SEL_2_S   10U
#define CLB_LUT4_IN2_SEL_2_M   0x7C00U   // Select inputs for unit 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_LUT4_IN3 register
//
//*************************************************************************************************
#define CLB_LUT4_IN3_SEL_0_S   0U
#define CLB_LUT4_IN3_SEL_0_M   0x1FU     // Select inputs for unit 0
#define CLB_LUT4_IN3_SEL_1_S   5U
#define CLB_LUT4_IN3_SEL_1_M   0x3E0U    // Select inputs for unit 1
#define CLB_LUT4_IN3_SEL_2_S   10U
#define CLB_LUT4_IN3_SEL_2_M   0x7C00U   // Select inputs for unit 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_FSM_LUT_FN1_0 register
//
//*************************************************************************************************
#define CLB_FSM_LUT_FN1_0_FN0_S   0U
#define CLB_FSM_LUT_FN1_0_FN0_M   0xFFFFU       // FSM LUT output function for unit 0
#define CLB_FSM_LUT_FN1_0_FN1_S   16U
#define CLB_FSM_LUT_FN1_0_FN1_M   0xFFFF0000U   // FSM LUT output function for unit 1

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_FSM_LUT_FN2 register
//
//*************************************************************************************************
#define CLB_FSM_LUT_FN2_FN1_S   0U
#define CLB_FSM_LUT_FN2_FN1_M   0xFFFFU   // FSM LUT output function for unit 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_LUT4_FN1_0 register
//
//*************************************************************************************************
#define CLB_LUT4_FN1_0_FN0_S   0U
#define CLB_LUT4_FN1_0_FN0_M   0xFFFFU       // LUT4 output function for unit 0
#define CLB_LUT4_FN1_0_FN1_S   16U
#define CLB_LUT4_FN1_0_FN1_M   0xFFFF0000U   // LUT4 output function for unit 1

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_LUT4_FN2 register
//
//*************************************************************************************************
#define CLB_LUT4_FN2_FN1_S   0U
#define CLB_LUT4_FN2_FN1_M   0xFFFFU   // LUT4 output function for unit 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_FSM_NEXT_STATE_0 register
//
//*************************************************************************************************
#define CLB_FSM_NEXT_STATE_0_S0_S   0U
#define CLB_FSM_NEXT_STATE_0_S0_M   0xFFFFU       // FSM next state function for S0
#define CLB_FSM_NEXT_STATE_0_S1_S   16U
#define CLB_FSM_NEXT_STATE_0_S1_M   0xFFFF0000U   // FSM next state function for S1

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_FSM_NEXT_STATE_1 register
//
//*************************************************************************************************
#define CLB_FSM_NEXT_STATE_1_S0_S   0U
#define CLB_FSM_NEXT_STATE_1_S0_M   0xFFFFU       // FSM next state function for S0
#define CLB_FSM_NEXT_STATE_1_S1_S   16U
#define CLB_FSM_NEXT_STATE_1_S1_M   0xFFFF0000U   // FSM next state function for S1

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_FSM_NEXT_STATE_2 register
//
//*************************************************************************************************
#define CLB_FSM_NEXT_STATE_2_S0_S   0U
#define CLB_FSM_NEXT_STATE_2_S0_M   0xFFFFU       // FSM next state function for S0
#define CLB_FSM_NEXT_STATE_2_S1_S   16U
#define CLB_FSM_NEXT_STATE_2_S1_M   0xFFFF0000U   // FSM next state function for S1

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_MISC_CONTROL register
//
//*************************************************************************************************
#define CLB_MISC_CONTROL_COUNT_ADD_SHIFT_0      0x1U         // Add/Shift for counter 0
#define CLB_MISC_CONTROL_COUNT_DIR_0            0x2U         // Direction for counter 0
#define CLB_MISC_CONTROL_COUNT_EVENT_CTRL_0     0x4U         // Event control for counter 0
#define CLB_MISC_CONTROL_COUNT_ADD_SHIFT_1      0x8U         // Add/Shift for counter 1
#define CLB_MISC_CONTROL_COUNT_DIR_1            0x10U        // Direction for counter 1
#define CLB_MISC_CONTROL_COUNT_EVENT_CTRL_1     0x20U        // Event control for counter 1
#define CLB_MISC_CONTROL_COUNT_ADD_SHIFT_2      0x40U        // Add/Shift for counter 2
#define CLB_MISC_CONTROL_COUNT_DIR_2            0x80U        // Direction for counter 2
#define CLB_MISC_CONTROL_COUNT_EVENT_CTRL_2     0x100U       // Event control for counter 2
#define CLB_MISC_CONTROL_COUNT_SERIALIZER_0     0x200U       // Serializer enable 0
#define CLB_MISC_CONTROL_COUNT_SERIALIZER_1     0x400U       // Serializer enable 1
#define CLB_MISC_CONTROL_COUNT_SERIALIZER_2     0x800U       // Serializer enable 2
#define CLB_MISC_CONTROL_FSM_EXTRA_SEL0_0       0x1000U      // FSM extra_sel0 for 0
#define CLB_MISC_CONTROL_FSM_EXTRA_SEL1_0       0x2000U      // FSM extra_sel1 for 0
#define CLB_MISC_CONTROL_FSM_EXTRA_SEL0_1       0x4000U      // FSM extra_sel0 for 1
#define CLB_MISC_CONTROL_FSM_EXTRA_SEL1_1       0x8000U      // FSM extra_sel1 for 1
#define CLB_MISC_CONTROL_FSM_EXTRA_SEL0_2       0x10000U     // FSM extra_sel0 for 2
#define CLB_MISC_CONTROL_FSM_EXTRA_SEL1_2       0x20000U     // FSM extra_sel1 for 2
#define CLB_MISC_CONTROL_COUNT0_MATCH1_TAP_EN   0x40000U     // Match1 Tap Enable for Counter 0
#define CLB_MISC_CONTROL_COUNT1_MATCH1_TAP_EN   0x80000U     // Match1 Tap Enable for Counter 1
#define CLB_MISC_CONTROL_COUNT2_MATCH1_TAP_EN   0x100000U    // Match1 Tap Enable for Counter 2
#define CLB_MISC_CONTROL_COUNT0_MATCH2_TAP_EN   0x200000U    // Match2 Tap Enable for Counter 0
#define CLB_MISC_CONTROL_COUNT1_MATCH2_TAP_EN   0x400000U    // Match2 Tap Enable for Counter 1
#define CLB_MISC_CONTROL_COUNT2_MATCH2_TAP_EN   0x800000U    // Match2 Tap Enable for Counter 2
#define CLB_MISC_CONTROL_COUNT0_LFSR_EN         0x1000000U   // Enable LFSR mode for Counter 0
#define CLB_MISC_CONTROL_COUNT1_LFSR_EN         0x2000000U   // Enable LFSR mode for Counter 1
#define CLB_MISC_CONTROL_COUNT2_LFSR_EN         0x4000000U   // Enable LFSR mode for Counter 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_LUT_0 register
//
//*************************************************************************************************
#define CLB_OUTPUT_LUT_0_IN0_S   0U
#define CLB_OUTPUT_LUT_0_IN0_M   0x1FU       // Select value for IN0 of output LUT
#define CLB_OUTPUT_LUT_0_IN1_S   5U
#define CLB_OUTPUT_LUT_0_IN1_M   0x3E0U      // Select value for IN1 of output LUT
#define CLB_OUTPUT_LUT_0_IN2_S   10U
#define CLB_OUTPUT_LUT_0_IN2_M   0x7C00U     // Select value for IN2 of output LUT
#define CLB_OUTPUT_LUT_0_FN_S    15U
#define CLB_OUTPUT_LUT_0_FN_M    0x7F8000U   // Output function for output LUT

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_LUT_1 register
//
//*************************************************************************************************
#define CLB_OUTPUT_LUT_1_IN0_S   0U
#define CLB_OUTPUT_LUT_1_IN0_M   0x1FU       // Select value for IN0 of output LUT
#define CLB_OUTPUT_LUT_1_IN1_S   5U
#define CLB_OUTPUT_LUT_1_IN1_M   0x3E0U      // Select value for IN1 of output LUT
#define CLB_OUTPUT_LUT_1_IN2_S   10U
#define CLB_OUTPUT_LUT_1_IN2_M   0x7C00U     // Select value for IN2 of output LUT
#define CLB_OUTPUT_LUT_1_FN_S    15U
#define CLB_OUTPUT_LUT_1_FN_M    0x7F8000U   // Output function for output LUT

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_LUT_2 register
//
//*************************************************************************************************
#define CLB_OUTPUT_LUT_2_IN0_S   0U
#define CLB_OUTPUT_LUT_2_IN0_M   0x1FU       // Select value for IN0 of output LUT
#define CLB_OUTPUT_LUT_2_IN1_S   5U
#define CLB_OUTPUT_LUT_2_IN1_M   0x3E0U      // Select value for IN1 of output LUT
#define CLB_OUTPUT_LUT_2_IN2_S   10U
#define CLB_OUTPUT_LUT_2_IN2_M   0x7C00U     // Select value for IN2 of output LUT
#define CLB_OUTPUT_LUT_2_FN_S    15U
#define CLB_OUTPUT_LUT_2_FN_M    0x7F8000U   // Output function for output LUT

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_LUT_3 register
//
//*************************************************************************************************
#define CLB_OUTPUT_LUT_3_IN0_S   0U
#define CLB_OUTPUT_LUT_3_IN0_M   0x1FU       // Select value for IN0 of output LUT
#define CLB_OUTPUT_LUT_3_IN1_S   5U
#define CLB_OUTPUT_LUT_3_IN1_M   0x3E0U      // Select value for IN1 of output LUT
#define CLB_OUTPUT_LUT_3_IN2_S   10U
#define CLB_OUTPUT_LUT_3_IN2_M   0x7C00U     // Select value for IN2 of output LUT
#define CLB_OUTPUT_LUT_3_FN_S    15U
#define CLB_OUTPUT_LUT_3_FN_M    0x7F8000U   // Output function for output LUT

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_LUT_4 register
//
//*************************************************************************************************
#define CLB_OUTPUT_LUT_4_IN0_S   0U
#define CLB_OUTPUT_LUT_4_IN0_M   0x1FU       // Select value for IN0 of output LUT
#define CLB_OUTPUT_LUT_4_IN1_S   5U
#define CLB_OUTPUT_LUT_4_IN1_M   0x3E0U      // Select value for IN1 of output LUT
#define CLB_OUTPUT_LUT_4_IN2_S   10U
#define CLB_OUTPUT_LUT_4_IN2_M   0x7C00U     // Select value for IN2 of output LUT
#define CLB_OUTPUT_LUT_4_FN_S    15U
#define CLB_OUTPUT_LUT_4_FN_M    0x7F8000U   // Output function for output LUT

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_LUT_5 register
//
//*************************************************************************************************
#define CLB_OUTPUT_LUT_5_IN0_S   0U
#define CLB_OUTPUT_LUT_5_IN0_M   0x1FU       // Select value for IN0 of output LUT
#define CLB_OUTPUT_LUT_5_IN1_S   5U
#define CLB_OUTPUT_LUT_5_IN1_M   0x3E0U      // Select value for IN1 of output LUT
#define CLB_OUTPUT_LUT_5_IN2_S   10U
#define CLB_OUTPUT_LUT_5_IN2_M   0x7C00U     // Select value for IN2 of output LUT
#define CLB_OUTPUT_LUT_5_FN_S    15U
#define CLB_OUTPUT_LUT_5_FN_M    0x7F8000U   // Output function for output LUT

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_LUT_6 register
//
//*************************************************************************************************
#define CLB_OUTPUT_LUT_6_IN0_S   0U
#define CLB_OUTPUT_LUT_6_IN0_M   0x1FU       // Select value for IN0 of output LUT
#define CLB_OUTPUT_LUT_6_IN1_S   5U
#define CLB_OUTPUT_LUT_6_IN1_M   0x3E0U      // Select value for IN1 of output LUT
#define CLB_OUTPUT_LUT_6_IN2_S   10U
#define CLB_OUTPUT_LUT_6_IN2_M   0x7C00U     // Select value for IN2 of output LUT
#define CLB_OUTPUT_LUT_6_FN_S    15U
#define CLB_OUTPUT_LUT_6_FN_M    0x7F8000U   // Output function for output LUT

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_LUT_7 register
//
//*************************************************************************************************
#define CLB_OUTPUT_LUT_7_IN0_S   0U
#define CLB_OUTPUT_LUT_7_IN0_M   0x1FU       // Select value for IN0 of output LUT
#define CLB_OUTPUT_LUT_7_IN1_S   5U
#define CLB_OUTPUT_LUT_7_IN1_M   0x3E0U      // Select value for IN1 of output LUT
#define CLB_OUTPUT_LUT_7_IN2_S   10U
#define CLB_OUTPUT_LUT_7_IN2_M   0x7C00U     // Select value for IN2 of output LUT
#define CLB_OUTPUT_LUT_7_FN_S    15U
#define CLB_OUTPUT_LUT_7_FN_M    0x7F8000U   // Output function for output LUT

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_HLC_EVENT_SEL register
//
//*************************************************************************************************
#define CLB_HLC_EVENT_SEL_EVENT0_SEL_S     0U
#define CLB_HLC_EVENT_SEL_EVENT0_SEL_M     0x1FU       // Event Select 0
#define CLB_HLC_EVENT_SEL_EVENT1_SEL_S     5U
#define CLB_HLC_EVENT_SEL_EVENT1_SEL_M     0x3E0U      // Event Select 1
#define CLB_HLC_EVENT_SEL_EVENT2_SEL_S     10U
#define CLB_HLC_EVENT_SEL_EVENT2_SEL_M     0x7C00U     // Event Select 2
#define CLB_HLC_EVENT_SEL_EVENT3_SEL_S     15U
#define CLB_HLC_EVENT_SEL_EVENT3_SEL_M     0xF8000U    // Event Select 3
#define CLB_HLC_EVENT_SEL_ALT_EVENT0_SEL   0x100000U   // Event Select 3
#define CLB_HLC_EVENT_SEL_ALT_EVENT1_SEL   0x200000U   // Event Select 3
#define CLB_HLC_EVENT_SEL_ALT_EVENT2_SEL   0x400000U   // Event Select 3
#define CLB_HLC_EVENT_SEL_ALT_EVENT3_SEL   0x800000U   // Event Select 3

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_COUNT_MATCH_TAP_SEL register
//
//*************************************************************************************************
#define CLB_COUNT_MATCH_TAP_SEL_COUNT0_MATCH1_S   0U
#define CLB_COUNT_MATCH_TAP_SEL_COUNT0_MATCH1_M   0x1FU         // Match1 tap select for Counter 0
#define CLB_COUNT_MATCH_TAP_SEL_COUNT1_MATCH1_S   5U
#define CLB_COUNT_MATCH_TAP_SEL_COUNT1_MATCH1_M   0x3E0U        // Match1 tap select for Counter 1
#define CLB_COUNT_MATCH_TAP_SEL_COUNT2_MATCH1_S   10U
#define CLB_COUNT_MATCH_TAP_SEL_COUNT2_MATCH1_M   0x7C00U       // Match1 tap select for Counter 2
#define CLB_COUNT_MATCH_TAP_SEL_COUNT0_MATCH2_S   16U
#define CLB_COUNT_MATCH_TAP_SEL_COUNT0_MATCH2_M   0x1F0000U     // Match2 tap select for Counter 0
#define CLB_COUNT_MATCH_TAP_SEL_COUNT1_MATCH2_S   21U
#define CLB_COUNT_MATCH_TAP_SEL_COUNT1_MATCH2_M   0x3E00000U    // Match2 tap select for Counter 1
#define CLB_COUNT_MATCH_TAP_SEL_COUNT2_MATCH2_S   26U
#define CLB_COUNT_MATCH_TAP_SEL_COUNT2_MATCH2_M   0x7C000000U   // Match2 tap select for Counter 2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_0 register
//
//*************************************************************************************************
#define CLB_OUTPUT_COND_CTRL_0_LEVEL_1_SEL          0x1U      // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_0_LEVEL_2_SEL_S        1U
#define CLB_OUTPUT_COND_CTRL_0_LEVEL_2_SEL_M        0x6U      // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_0_LEVEL_3_SEL_S        3U
#define CLB_OUTPUT_COND_CTRL_0_LEVEL_3_SEL_M        0x18U     // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_0_SEL_GATING_CTRL_S    5U
#define CLB_OUTPUT_COND_CTRL_0_SEL_GATING_CTRL_M    0xE0U     // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_0_SEL_RELEASE_CTRL_S   8U
#define CLB_OUTPUT_COND_CTRL_0_SEL_RELEASE_CTRL_M   0x700U    // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_0_HW_GATING_CTRL_SEL   0x800U    // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_0_HW_RLS_CTRL_SEL      0x1000U   // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_0_SEL_RAW_IN           0x2000U   // Select input mode for the CLB AOC
#define CLB_OUTPUT_COND_CTRL_0_ASYNC_COND_EN        0x4000U   // Enable for conditioning

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_1 register
//
//*************************************************************************************************
#define CLB_OUTPUT_COND_CTRL_1_LEVEL_1_SEL          0x1U      // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_1_LEVEL_2_SEL_S        1U
#define CLB_OUTPUT_COND_CTRL_1_LEVEL_2_SEL_M        0x6U      // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_1_LEVEL_3_SEL_S        3U
#define CLB_OUTPUT_COND_CTRL_1_LEVEL_3_SEL_M        0x18U     // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_1_SEL_GATING_CTRL_S    5U
#define CLB_OUTPUT_COND_CTRL_1_SEL_GATING_CTRL_M    0xE0U     // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_1_SEL_RELEASE_CTRL_S   8U
#define CLB_OUTPUT_COND_CTRL_1_SEL_RELEASE_CTRL_M   0x700U    // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_1_HW_GATING_CTRL_SEL   0x800U    // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_1_HW_RLS_CTRL_SEL      0x1000U   // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_1_SEL_RAW_IN           0x2000U   // Select input mode for the CLB AOC
#define CLB_OUTPUT_COND_CTRL_1_ASYNC_COND_EN        0x4000U   // Enable for conditioning

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_2 register
//
//*************************************************************************************************
#define CLB_OUTPUT_COND_CTRL_2_LEVEL_1_SEL          0x1U      // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_2_LEVEL_2_SEL_S        1U
#define CLB_OUTPUT_COND_CTRL_2_LEVEL_2_SEL_M        0x6U      // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_2_LEVEL_3_SEL_S        3U
#define CLB_OUTPUT_COND_CTRL_2_LEVEL_3_SEL_M        0x18U     // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_2_SEL_GATING_CTRL_S    5U
#define CLB_OUTPUT_COND_CTRL_2_SEL_GATING_CTRL_M    0xE0U     // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_2_SEL_RELEASE_CTRL_S   8U
#define CLB_OUTPUT_COND_CTRL_2_SEL_RELEASE_CTRL_M   0x700U    // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_2_HW_GATING_CTRL_SEL   0x800U    // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_2_HW_RLS_CTRL_SEL      0x1000U   // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_2_SEL_RAW_IN           0x2000U   // Select input mode for the CLB AOC
#define CLB_OUTPUT_COND_CTRL_2_ASYNC_COND_EN        0x4000U   // Enable for conditioning

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_3 register
//
//*************************************************************************************************
#define CLB_OUTPUT_COND_CTRL_3_LEVEL_1_SEL          0x1U      // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_3_LEVEL_2_SEL_S        1U
#define CLB_OUTPUT_COND_CTRL_3_LEVEL_2_SEL_M        0x6U      // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_3_LEVEL_3_SEL_S        3U
#define CLB_OUTPUT_COND_CTRL_3_LEVEL_3_SEL_M        0x18U     // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_3_SEL_GATING_CTRL_S    5U
#define CLB_OUTPUT_COND_CTRL_3_SEL_GATING_CTRL_M    0xE0U     // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_3_SEL_RELEASE_CTRL_S   8U
#define CLB_OUTPUT_COND_CTRL_3_SEL_RELEASE_CTRL_M   0x700U    // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_3_HW_GATING_CTRL_SEL   0x800U    // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_3_HW_RLS_CTRL_SEL      0x1000U   // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_3_SEL_RAW_IN           0x2000U   // Select input mode for the CLB AOC
#define CLB_OUTPUT_COND_CTRL_3_ASYNC_COND_EN        0x4000U   // Enable for conditioning

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_4 register
//
//*************************************************************************************************
#define CLB_OUTPUT_COND_CTRL_4_LEVEL_1_SEL          0x1U      // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_4_LEVEL_2_SEL_S        1U
#define CLB_OUTPUT_COND_CTRL_4_LEVEL_2_SEL_M        0x6U      // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_4_LEVEL_3_SEL_S        3U
#define CLB_OUTPUT_COND_CTRL_4_LEVEL_3_SEL_M        0x18U     // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_4_SEL_GATING_CTRL_S    5U
#define CLB_OUTPUT_COND_CTRL_4_SEL_GATING_CTRL_M    0xE0U     // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_4_SEL_RELEASE_CTRL_S   8U
#define CLB_OUTPUT_COND_CTRL_4_SEL_RELEASE_CTRL_M   0x700U    // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_4_HW_GATING_CTRL_SEL   0x800U    // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_4_HW_RLS_CTRL_SEL      0x1000U   // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_4_SEL_RAW_IN           0x2000U   // Select input mode for the CLB AOC
#define CLB_OUTPUT_COND_CTRL_4_ASYNC_COND_EN        0x4000U   // Enable for conditioning

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_5 register
//
//*************************************************************************************************
#define CLB_OUTPUT_COND_CTRL_5_LEVEL_1_SEL          0x1U      // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_5_LEVEL_2_SEL_S        1U
#define CLB_OUTPUT_COND_CTRL_5_LEVEL_2_SEL_M        0x6U      // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_5_LEVEL_3_SEL_S        3U
#define CLB_OUTPUT_COND_CTRL_5_LEVEL_3_SEL_M        0x18U     // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_5_SEL_GATING_CTRL_S    5U
#define CLB_OUTPUT_COND_CTRL_5_SEL_GATING_CTRL_M    0xE0U     // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_5_SEL_RELEASE_CTRL_S   8U
#define CLB_OUTPUT_COND_CTRL_5_SEL_RELEASE_CTRL_M   0x700U    // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_5_HW_GATING_CTRL_SEL   0x800U    // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_5_HW_RLS_CTRL_SEL      0x1000U   // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_5_SEL_RAW_IN           0x2000U   // Select input mode for the CLB AOC
#define CLB_OUTPUT_COND_CTRL_5_ASYNC_COND_EN        0x4000U   // Enable for conditioning

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_6 register
//
//*************************************************************************************************
#define CLB_OUTPUT_COND_CTRL_6_LEVEL_1_SEL          0x1U      // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_6_LEVEL_2_SEL_S        1U
#define CLB_OUTPUT_COND_CTRL_6_LEVEL_2_SEL_M        0x6U      // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_6_LEVEL_3_SEL_S        3U
#define CLB_OUTPUT_COND_CTRL_6_LEVEL_3_SEL_M        0x18U     // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_6_SEL_GATING_CTRL_S    5U
#define CLB_OUTPUT_COND_CTRL_6_SEL_GATING_CTRL_M    0xE0U     // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_6_SEL_RELEASE_CTRL_S   8U
#define CLB_OUTPUT_COND_CTRL_6_SEL_RELEASE_CTRL_M   0x700U    // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_6_HW_GATING_CTRL_SEL   0x800U    // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_6_HW_RLS_CTRL_SEL      0x1000U   // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_6_SEL_RAW_IN           0x2000U   // Select input mode for the CLB AOC
#define CLB_OUTPUT_COND_CTRL_6_ASYNC_COND_EN        0x4000U   // Enable for conditioning

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_7 register
//
//*************************************************************************************************
#define CLB_OUTPUT_COND_CTRL_7_LEVEL_1_SEL          0x1U      // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_7_LEVEL_2_SEL_S        1U
#define CLB_OUTPUT_COND_CTRL_7_LEVEL_2_SEL_M        0x6U      // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_7_LEVEL_3_SEL_S        3U
#define CLB_OUTPUT_COND_CTRL_7_LEVEL_3_SEL_M        0x18U     // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_7_SEL_GATING_CTRL_S    5U
#define CLB_OUTPUT_COND_CTRL_7_SEL_GATING_CTRL_M    0xE0U     // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_7_SEL_RELEASE_CTRL_S   8U
#define CLB_OUTPUT_COND_CTRL_7_SEL_RELEASE_CTRL_M   0x700U    // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_7_HW_GATING_CTRL_SEL   0x800U    // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_7_HW_RLS_CTRL_SEL      0x1000U   // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_7_SEL_RAW_IN           0x2000U   // Select input mode for the CLB AOC
#define CLB_OUTPUT_COND_CTRL_7_ASYNC_COND_EN        0x4000U   // Enable for conditioning

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_MISC_ACCESS_CTRL register
//
//*************************************************************************************************
#define CLB_MISC_ACCESS_CTRL_SPIEN   0x1U   // Enable CLB SPI Buffer feature
#define CLB_MISC_ACCESS_CTRL_BLKEN   0x2U   // Block Register write

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_SPI_DATA_CTRL_HI register
//
//*************************************************************************************************
#define CLB_SPI_DATA_CTRL_HI_STRB_S    0U
#define CLB_SPI_DATA_CTRL_HI_STRB_M    0x1FU     // Select value for strobe
#define CLB_SPI_DATA_CTRL_HI_SHIFT_S   8U
#define CLB_SPI_DATA_CTRL_HI_SHIFT_M   0x1F00U   // Shift value select


//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_LOAD_EN register
//
//*************************************************************************************************
#define CLB_LOAD_EN_LOAD_EN       0x1U    // Load Enable
#define CLB_LOAD_EN_GLOBAL_EN     0x2U    // Global Enable
#define CLB_LOAD_EN_STOP          0x4U    // Debug stop control
#define CLB_LOAD_EN_NMI_EN        0x8U    // NMI output enable
#define CLB_LOAD_EN_PIPELINE_EN   0x10U   // Enable input pipelining

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_LOAD_ADDR register
//
//*************************************************************************************************
#define CLB_LOAD_ADDR_ADDR_S   0U
#define CLB_LOAD_ADDR_ADDR_M   0x3FU   // Indirect Address

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_INPUT_FILTER register
//
//*************************************************************************************************
#define CLB_INPUT_FILTER_FIN0_S   0U
#define CLB_INPUT_FILTER_FIN0_M   0x3U          // Input filter control 0
#define CLB_INPUT_FILTER_FIN1_S   2U
#define CLB_INPUT_FILTER_FIN1_M   0xCU          // Input filter control 1
#define CLB_INPUT_FILTER_FIN2_S   4U
#define CLB_INPUT_FILTER_FIN2_M   0x30U         // Input filter control 2
#define CLB_INPUT_FILTER_FIN3_S   6U
#define CLB_INPUT_FILTER_FIN3_M   0xC0U         // Input filter control 3
#define CLB_INPUT_FILTER_FIN4_S   8U
#define CLB_INPUT_FILTER_FIN4_M   0x300U        // Input filter control 4
#define CLB_INPUT_FILTER_FIN5_S   10U
#define CLB_INPUT_FILTER_FIN5_M   0xC00U        // Input filter control 5
#define CLB_INPUT_FILTER_FIN6_S   12U
#define CLB_INPUT_FILTER_FIN6_M   0x3000U       // Input filter control 6
#define CLB_INPUT_FILTER_FIN7_S   14U
#define CLB_INPUT_FILTER_FIN7_M   0xC000U       // Input filter control 7
#define CLB_INPUT_FILTER_SYNC0    0x10000U      // Synchronizer control 0
#define CLB_INPUT_FILTER_SYNC1    0x20000U      // Synchronizer control 1
#define CLB_INPUT_FILTER_SYNC2    0x40000U      // Synchronizer control 2
#define CLB_INPUT_FILTER_SYNC3    0x80000U      // Synchronizer control 3
#define CLB_INPUT_FILTER_SYNC4    0x100000U     // Synchronizer control 4
#define CLB_INPUT_FILTER_SYNC5    0x200000U     // Synchronizer control 5
#define CLB_INPUT_FILTER_SYNC6    0x400000U     // Synchronizer control 6
#define CLB_INPUT_FILTER_SYNC7    0x800000U     // Synchronizer control 7
#define CLB_INPUT_FILTER_PIPE0    0x1000000U    // Enable pipeline 0
#define CLB_INPUT_FILTER_PIPE1    0x2000000U    // Enable pipeline 1
#define CLB_INPUT_FILTER_PIPE2    0x4000000U    // Enable pipeline 2
#define CLB_INPUT_FILTER_PIPE3    0x8000000U    // Enable pipeline 3
#define CLB_INPUT_FILTER_PIPE4    0x10000000U   // Enable pipeline 4
#define CLB_INPUT_FILTER_PIPE5    0x20000000U   // Enable pipeline 5
#define CLB_INPUT_FILTER_PIPE6    0x40000000U   // Enable pipeline 6
#define CLB_INPUT_FILTER_PIPE7    0x80000000U   // Enable pipeline 7

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_IN_MUX_SEL_0 register
//
//*************************************************************************************************
#define CLB_IN_MUX_SEL_0_SEL_GP_IN_0   0x1U    // Select GP register 0
#define CLB_IN_MUX_SEL_0_SEL_GP_IN_1   0x2U    // Select GP register 1
#define CLB_IN_MUX_SEL_0_SEL_GP_IN_2   0x4U    // Select GP register 2
#define CLB_IN_MUX_SEL_0_SEL_GP_IN_3   0x8U    // Select GP register 3
#define CLB_IN_MUX_SEL_0_SEL_GP_IN_4   0x10U   // Select GP register 4
#define CLB_IN_MUX_SEL_0_SEL_GP_IN_5   0x20U   // Select GP register 5
#define CLB_IN_MUX_SEL_0_SEL_GP_IN_6   0x40U   // Select GP register 6
#define CLB_IN_MUX_SEL_0_SEL_GP_IN_7   0x80U   // Select GP register 7

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_LCL_MUX_SEL_1 register
//
//*************************************************************************************************
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_0_S   0U
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_0_M   0x1FU         // Local Mux select 0
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_1_S   5U
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_1_M   0x3E0U        // Local Mux select 1
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_2_S   10U
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_2_M   0x7C00U       // Local Mux select 2
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_3_S   15U
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_3_M   0xF8000U      // Local Mux select 3
#define CLB_LCL_MUX_SEL_1_MISC_INPUT_SEL_0     0x10000000U   // Select MISC_INPUT
#define CLB_LCL_MUX_SEL_1_MISC_INPUT_SEL_1     0x20000000U   // Select MISC_INPUT
#define CLB_LCL_MUX_SEL_1_MISC_INPUT_SEL_2     0x40000000U   // Select MISC_INPUT
#define CLB_LCL_MUX_SEL_1_MISC_INPUT_SEL_3     0x80000000U   // Select MISC_INPUT

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_LCL_MUX_SEL_2 register
//
//*************************************************************************************************
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_4_S   0U
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_4_M   0x1FU         // Local Mux select 4
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_5_S   5U
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_5_M   0x3E0U        // Local Mux select 5
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_6_S   10U
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_6_M   0x7C00U       // Local Mux select 6
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_7_S   15U
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_7_M   0xF8000U      // Local Mux select 7
#define CLB_LCL_MUX_SEL_2_MISC_INPUT_SEL_4     0x10000000U   // Select MISC_INPUT
#define CLB_LCL_MUX_SEL_2_MISC_INPUT_SEL_5     0x20000000U   // Select MISC_INPUT
#define CLB_LCL_MUX_SEL_2_MISC_INPUT_SEL_6     0x40000000U   // Select MISC_INPUT
#define CLB_LCL_MUX_SEL_2_MISC_INPUT_SEL_7     0x80000000U   // Select MISC_INPUT

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_BUF_PTR register
//
//*************************************************************************************************
#define CLB_BUF_PTR_PULL_S   0U
#define CLB_BUF_PTR_PULL_M   0xFFU       // Data pointer for pull
#define CLB_BUF_PTR_PUSH_S   16U
#define CLB_BUF_PTR_PUSH_M   0xFF0000U   // Data pointer for pull

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_GP_REG register
//
//*************************************************************************************************
#define CLB_GP_REG_REG_S              0U
#define CLB_GP_REG_REG_M              0xFFU         // General Purpose bit register
#define CLB_GP_REG_SW_GATING_CTRL_0   0x10000U      // Software gating control 0
#define CLB_GP_REG_SW_GATING_CTRL_1   0x20000U      // Software gating control 1
#define CLB_GP_REG_SW_GATING_CTRL_2   0x40000U      // Software gating control 2
#define CLB_GP_REG_SW_GATING_CTRL_3   0x80000U      // Software gating control 3
#define CLB_GP_REG_SW_GATING_CTRL_4   0x100000U     // Software gating control 4
#define CLB_GP_REG_SW_GATING_CTRL_5   0x200000U     // Software gating control 5
#define CLB_GP_REG_SW_GATING_CTRL_6   0x400000U     // Software gating control 6
#define CLB_GP_REG_SW_GATING_CTRL_7   0x800000U     // Software gating control 7
#define CLB_GP_REG_SW_RLS_CTRL_0      0x1000000U    // Software release control 0
#define CLB_GP_REG_SW_RLS_CTRL_1      0x2000000U    // Software release control 1
#define CLB_GP_REG_SW_RLS_CTRL_2      0x4000000U    // Software release control 2
#define CLB_GP_REG_SW_RLS_CTRL_3      0x8000000U    // Software release control 3
#define CLB_GP_REG_SW_RLS_CTRL_4      0x10000000U   // Software release control 4
#define CLB_GP_REG_SW_RLS_CTRL_5      0x20000000U   // Software release control 5
#define CLB_GP_REG_SW_RLS_CTRL_6      0x40000000U   // Software release control 6
#define CLB_GP_REG_SW_RLS_CTRL_7      0x80000000U   // Software release control 7

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_GLBL_MUX_SEL_1 register
//
//*************************************************************************************************
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_0_S   0U
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_0_M   0x7FU        // Global Mux select 0
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_1_S   7U
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_1_M   0x3F80U      // Global Mux select 1
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_2_S   14U
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_2_M   0x1FC000U    // Global Mux select 2
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_3_S   21U
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_3_M   0xFE00000U   // Global Mux select 3

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_GLBL_MUX_SEL_2 register
//
//*************************************************************************************************
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_4_S   0U
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_4_M   0x7FU        // Global Mux select 4
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_5_S   7U
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_5_M   0x3F80U      // Global Mux select 5
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_6_S   14U
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_6_M   0x1FC000U    // Global Mux select 6
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_7_S   21U
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_7_M   0xFE00000U   // Global Mux select 7

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_PRESCALE_CTRL register
//
//*************************************************************************************************
#define CLB_PRESCALE_CTRL_CLKEN        0x1U          // Enable the prescale clock generator
#define CLB_PRESCALE_CTRL_STRB         0x2U          // Enable the Strobe mode of operation
#define CLB_PRESCALE_CTRL_TAP_S        2U
#define CLB_PRESCALE_CTRL_TAP_M        0x3CU         // TAP Select value
#define CLB_PRESCALE_CTRL_PRESCALE_S   16U
#define CLB_PRESCALE_CTRL_PRESCALE_M   0xFFFF0000U   // Value of prescale register

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_INTR_TAG_REG register
//
//*************************************************************************************************
#define CLB_INTR_TAG_REG_TAG_S   0U
#define CLB_INTR_TAG_REG_TAG_M   0x3FU   // Interrupt tag

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_LOCK register
//
//*************************************************************************************************
#define CLB_LOCK_LOCK    0x1U          // LOCK enable
#define CLB_LOCK_KEY_S   16U
#define CLB_LOCK_KEY_M   0xFFFF0000U   // Key for enabling write

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_HLC_INSTR_READ_PTR register
//
//*************************************************************************************************
#define CLB_HLC_INSTR_READ_PTR_READ_PTR_S   0U
#define CLB_HLC_INSTR_READ_PTR_READ_PTR_M   0x1FU   // HLC instruction read pointer

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_HLC_INSTR_VALUE register
//
//*************************************************************************************************
#define CLB_HLC_INSTR_VALUE_INSTR_S   0U
#define CLB_HLC_INSTR_VALUE_INSTR_M   0xFFFU   // HLC instruction value

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_DBG_OUT_2 register
//
//*************************************************************************************************
#define CLB_DBG_OUT_2_OUT_S   0U
#define CLB_DBG_OUT_2_OUT_M   0xFFU     // Outputs of CLB Async block
#define CLB_DBG_OUT_2_IN_S    8U
#define CLB_DBG_OUT_2_IN_M    0xFF00U   // CLB CELL Inputs

//*************************************************************************************************
//
// The following are defines for the bit fields in the CLB_DBG_OUT register
//
//*************************************************************************************************
#define CLB_DBG_OUT_COUNT0_MATCH2   0x2U          // COUNT_MATCH2 UNIT 0
#define CLB_DBG_OUT_COUNT0_ZERO     0x4U          // COUNT_ZERO UNIT 0
#define CLB_DBG_OUT_COUNT0_MATCH1   0x8U          // COUNT_MATCH1 UNIT 0
#define CLB_DBG_OUT_FSM0_S0         0x10U         // FSM_S0 UNIT 0
#define CLB_DBG_OUT_FSM0_S1         0x20U         // FSM_S1 UNIT 0
#define CLB_DBG_OUT_FSM0_LUTOUT     0x40U         // FSM_LUT_OUT UNIT 0
#define CLB_DBG_OUT_LUT40_OUT       0x80U         // LUT4_OUT UNIT 0
#define CLB_DBG_OUT_COUNT1_MATCH2   0x200U        // COUNT_MATCH2 UNIT 1
#define CLB_DBG_OUT_COUNT1_ZERO     0x400U        // COUNT_ZERO UNIT 1
#define CLB_DBG_OUT_COUNT1_MATCH1   0x800U        // COUNT_MATCH1 UNIT 1
#define CLB_DBG_OUT_FSM1_S0         0x1000U       // FSM_S0 UNIT 1
#define CLB_DBG_OUT_FSM1_S1         0x2000U       // FSM_S1 UNIT 1
#define CLB_DBG_OUT_FSM1_LUTOUT     0x4000U       // FSM_LUT_OUT UNIT 1
#define CLB_DBG_OUT_LUT41_OUT       0x8000U       // LUT4_OUT UNIT 1
#define CLB_DBG_OUT_COUNT2_MATCH2   0x20000U      // COUNT_MATCH2 UNIT 2
#define CLB_DBG_OUT_COUNT2_ZERO     0x40000U      // COUNT_ZERO UNIT 2
#define CLB_DBG_OUT_COUNT2_MATCH1   0x80000U      // COUNT_MATCH1 UNIT 2
#define CLB_DBG_OUT_FSM2_S0         0x100000U     // FSM_S0 UNIT 2
#define CLB_DBG_OUT_FSM2_S1         0x200000U     // FSM_S1 UNIT 2
#define CLB_DBG_OUT_FSM2_LUTOUT     0x400000U     // FSM_LUT_OUT UNIT 2
#define CLB_DBG_OUT_LUT42_OUT       0x800000U     // LUT4_OUT UNIT 2
#define CLB_DBG_OUT_OUT0            0x1000000U    // CELL Output 0
#define CLB_DBG_OUT_OUT1            0x2000000U    // CELL Output 1
#define CLB_DBG_OUT_OUT2            0x4000000U    // CELL Output 2
#define CLB_DBG_OUT_OUT3            0x8000000U    // CELL Output 3
#define CLB_DBG_OUT_OUT4            0x10000000U   // CELL Output 4
#define CLB_DBG_OUT_OUT5            0x20000000U   // CELL Output 5
#define CLB_DBG_OUT_OUT6            0x40000000U   // CELL Output 6
#define CLB_DBG_OUT_OUT7            0x80000000U   // CELL Output 7




#endif
