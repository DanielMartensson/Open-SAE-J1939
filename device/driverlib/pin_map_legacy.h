//###########################################################################
//
// FILE:   pin_map.h
//
// TITLE:  Legacy definitions of pin mux info for gpio.c.
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

#ifndef __PIN_MAP_LEGACY_H__
#define __PIN_MAP_LEGACY_H__


#include "pin_map.h"

//*****************************************************************************
// Legacy pinmuxing MACROS - Retained for portability across devices ONLY
// Not recommended for new users
//*****************************************************************************

#define GPIO_0_SDAA                     GPIO_0_I2CA_SDA
#define GPIO_0_FSITXD0A                 GPIO_0_FSITXA_D0

#define GPIO_1_SCLA                     GPIO_1_I2CA_SCL
#define GPIO_1_FSITXD1A                 GPIO_1_FSITXA_D1

#define GPIO_2_SDAB                     GPIO_2_I2CB_SDA
#define GPIO_2_FSITXCLKA                GPIO_2_FSITXA_CLK

#define GPIO_3_SCLB                     GPIO_3_I2CB_SCL
#define GPIO_3_FSIRXD0A                 GPIO_3_FSIRXA_D0

#define GPIO_4_CANTXA                   GPIO_4_CANA_TX
#define GPIO_4_FSIRXD1A                 GPIO_4_FSIRXA_D1

#define GPIO_5_CANRXA                   GPIO_5_CANA_RX
#define GPIO_5_FSIRXCLKA                GPIO_5_FSIRXA_CLK

#define GPIO_6_EPWMSYNCO                GPIO_6_EXTSYNCOUT
#define GPIO_6_EQEP3A                   GPIO_6_EQEP3_A
#define GPIO_6_CANTXB                   GPIO_6_CANB_TX
#define GPIO_6_FSITXD0B                 GPIO_6_FSITXB_D0

#define GPIO_7_EQEP3B                   GPIO_7_EQEP3_B
#define GPIO_7_CANRXB                   GPIO_7_CANB_RX
#define GPIO_7_FSITXD1B                 GPIO_7_FSITXB_D1

#define GPIO_8_CANTXB                   GPIO_8_CANB_TX
#define GPIO_8_EQEP3S                   GPIO_8_EQEP3_STROBE
#define GPIO_8_SCITXDA                  GPIO_8_SCIA_TX
#define GPIO_8_FSITXCLKB                GPIO_8_FSITXB_CLK
#define GPIO_8_FSITXD1A                 GPIO_8_FSITXA_D1
#define GPIO_8_FSIRXD0A                 GPIO_8_FSIRXA_D0

#define GPIO_9_SCITXDB                  GPIO_9_SCIB_TX
#define GPIO_9_EQEP3I                   GPIO_9_EQEP3_INDEX
#define GPIO_9_SCIRXDA                  GPIO_9_SCIA_RX
#define GPIO_9_FSIRXD0B                 GPIO_9_FSIRXB_D0
#define GPIO_9_FSITXD0A                 GPIO_9_FSITXA_D0
#define GPIO_9_FSIRXCLKA                GPIO_9_FSIRXA_CLK

#define GPIO_10_CANRXB                   GPIO_10_CANB_RX
#define GPIO_10_EQEP1A                   GPIO_10_EQEP1_A
#define GPIO_10_SCITXDB                  GPIO_10_SCIB_TX
#define GPIO_10_FSIRXD1B                 GPIO_10_FSIRXB_D1
#define GPIO_10_FSITXCLKA                GPIO_10_FSITXA_CLK
#define GPIO_10_FSIRXD1A                 GPIO_10_FSIRXA_D1

#define GPIO_11_SCIRXDB                  GPIO_11_SCIB_RX
#define GPIO_11_EQEP1B                   GPIO_11_EQEP1_B
#define GPIO_11_FSIRXCLKB                GPIO_11_FSIRXB_CLK
#define GPIO_11_FSIRXD1A                 GPIO_11_FSIRXA_D1

#define GPIO_12_CANTXB                   GPIO_12_CANB_TX
#define GPIO_12_EQEP1S                   GPIO_12_EQEP1_STROBE
#define GPIO_12_SCITXDC                  GPIO_12_SCIC_TX
#define GPIO_12_FSIRXD0C                 GPIO_12_FSIRXC_D0
#define GPIO_12_FSIRXD0A                 GPIO_12_FSIRXA_D0

#define GPIO_13_CANRXB                   GPIO_13_CANB_RX
#define GPIO_13_EQEP1I                   GPIO_13_EQEP1_INDEX
#define GPIO_13_SCIRXDC                  GPIO_13_SCIC_RX
#define GPIO_13_FSIRXD1C                 GPIO_13_FSIRXC_D1
#define GPIO_13_FSIRXCLKA                GPIO_13_FSIRXA_CLK

#define GPIO_14_SCITXDB                  GPIO_14_SCIB_TX
#define GPIO_14_FSIRXCLKC                GPIO_14_FSIRXC_CLK

#define GPIO_15_SCIRXDB                  GPIO_15_SCIB_RX
#define GPIO_15_FSIRXD0D                 GPIO_15_FSIRXD_D0

#define GPIO_16_SPISIMOA                 GPIO_16_SPIA_SIMO
#define GPIO_16_CANTXB                   GPIO_16_CANB_TX
#define GPIO_16_SD_D1                    GPIO_16_SD1_D1
#define GPIO_16_FSIRXD1D                 GPIO_16_FSIRXD_D1

#define GPIO_17_SPISOMIA                 GPIO_17_SPIA_SOMI
#define GPIO_17_CANRXB                   GPIO_17_CANB_RX
#define GPIO_17_SD_C1                    GPIO_17_SD1_C1
#define GPIO_17_FSIRXCLKD                GPIO_17_FSIRXD_CLK

#define GPIO_18_SPICLKA                  GPIO_18_SPIA_CLK
#define GPIO_18_SCITXDB                  GPIO_18_SCIB_TX
#define GPIO_18_CANRXA                   GPIO_18_CANA_RX
#define GPIO_18_SD_D2                    GPIO_18_SD1_D2
#define GPIO_18_EM1CS2N                  GPIO_18_EMIF1_CS2N
#define GPIO_18_FSIRXD0E                 GPIO_18_FSIRXE_D0

#define GPIO_19_SPISTEA                  GPIO_19_SPIA_STEN
#define GPIO_19_SCIRXDB                  GPIO_19_SCIB_RX
#define GPIO_19_CANTXA                   GPIO_19_CANA_TX
#define GPIO_19_SD_C2                    GPIO_19_SD1_C2
#define GPIO_19_EM1CS3N                  GPIO_19_EMIF1_CS3N
#define GPIO_19_FSIRXD1E                 GPIO_19_FSIRXE_D1

#define GPIO_20_EQEP1A                   GPIO_20_EQEP1_A
#define GPIO_20_CANTXB                   GPIO_20_CANB_TX
#define GPIO_20_SD_D3                    GPIO_20_SD1_D3
#define GPIO_20_EM1BA0                   GPIO_20_EMIF1_BA0
#define GPIO_20_TRACEDATA0               GPIO_20_TRACE_DATA0
#define GPIO_20_FSIRXCLKE                GPIO_20_FSIRXE_CLK
#define GPIO_20_SPISIMOC                 GPIO_20_SPIC_SIMO

#define GPIO_21_EQEP1B                   GPIO_21_EQEP1_B
#define GPIO_21_CANRXB                   GPIO_21_CANB_RX
#define GPIO_21_SD_C3                    GPIO_21_SD1_C3
#define GPIO_21_EM1BA1                   GPIO_21_EMIF1_BA1
#define GPIO_21_TRACEDATA1               GPIO_21_TRACE_DATA1
#define GPIO_21_FSIRXD0F                 GPIO_21_FSIRXF_D0
#define GPIO_21_SPISOMIC                 GPIO_21_SPIC_SOMI

#define GPIO_22_EQEP1S                   GPIO_22_EQEP1_STROBE
#define GPIO_22_SCITXDB                  GPIO_22_SCIB_TX
#define GPIO_22_SPICLKB                  GPIO_22_SPIB_CLK
#define GPIO_22_SD_D4                    GPIO_22_SD1_D4
#define GPIO_22_EM1RAS                   GPIO_22_EMIF1_RAS
#define GPIO_22_TRACEDATA2               GPIO_22_TRACE_DATA2
#define GPIO_22_FSIRXD1F                 GPIO_22_FSIRXF_D1
#define GPIO_22_SPICLKC                  GPIO_22_SPIC_CLK

#define GPIO_23_EQEP1I                   GPIO_23_EQEP1_INDEX
#define GPIO_23_SCIRXDB                  GPIO_23_SCIB_RX
#define GPIO_23_SPISTEB                  GPIO_23_SPIB_STEN
#define GPIO_23_SD_C4                    GPIO_23_SD1_C4
#define GPIO_23_EM1CAS                   GPIO_23_EMIF1_CAS
#define GPIO_23_TRACEDATA3               GPIO_23_TRACE_DATA3
#define GPIO_23_FSIRXCLKF                GPIO_23_FSIRXF_CLK
#define GPIO_23_SPISTEC                  GPIO_23_SPIC_STEN

#define GPIO_24_EQEP2A                   GPIO_24_EQEP2_A
#define GPIO_24_SPISIMOB                 GPIO_24_SPIB_SIMO
#define GPIO_24_SD_D5                    GPIO_24_SD2_D1
#define GPIO_24_PMBCLK                   GPIO_24_PMBUSA_SCL
#define GPIO_24_EM1DQM0                  GPIO_24_EMIF1_DQM0
#define GPIO_24_TRACECLK                 GPIO_24_TRACE_CLK
#define GPIO_24_FSIRXD0G                 GPIO_24_FSIRXG_D0

#define GPIO_25_EQEP2B                   GPIO_25_EQEP2_B
#define GPIO_25_SPISOMIB                 GPIO_25_SPIB_SOMI
#define GPIO_25_SD_C5                    GPIO_25_SD2_C1
#define GPIO_25_PMBDAT                   GPIO_25_PMBUSA_SDA
#define GPIO_25_EM1DQM1                  GPIO_25_EMIF1_DQM1
#define GPIO_25_TRACESWO                 GPIO_25_TRACE_SWO
#define GPIO_25_FSITXD1A                 GPIO_25_FSITXA_D1
#define GPIO_25_FSIRXD1G                 GPIO_25_FSIRXG_D1

#define GPIO_26_EQEP2I                   GPIO_26_EQEP2_INDEX
#define GPIO_26_SPICLKB                  GPIO_26_SPIB_CLK
#define GPIO_26_SD_D6                    GPIO_26_SD2_D2
#define GPIO_26_PMBALERT                 GPIO_26_PMBUSA_ALERT
#define GPIO_26_EM1DQM2                  GPIO_26_EMIF1_DQM2
#define GPIO_26_FSITXD0A                 GPIO_26_FSITXA_D0
#define GPIO_26_FSIRXCLKG                GPIO_26_FSIRXG_CLK

#define GPIO_27_EQEP2S                   GPIO_27_EQEP2_STROBE
#define GPIO_27_SPISTEB                  GPIO_27_SPIB_STEN
#define GPIO_27_SD_C6                    GPIO_27_SD2_C2
#define GPIO_27_PMBCTRL                  GPIO_27_PMBUSA_CTL
#define GPIO_27_EM1DQM3                  GPIO_27_EMIF1_DQM3
#define GPIO_27_FSITXCLKA                GPIO_27_FSITXA_CLK
#define GPIO_27_FSIRXD0H                 GPIO_27_FSIRXH_D0

#define GPIO_28_SCIRXDA                  GPIO_28_SCIA_RX
#define GPIO_28_EM1CS4N                  GPIO_28_EMIF1_CS4N
#define GPIO_28_EQEP3A                   GPIO_28_EQEP3_A
#define GPIO_28_SD_D7                    GPIO_28_SD2_D3
#define GPIO_28_EM1CS2N                  GPIO_28_EMIF1_CS2N
#define GPIO_28_FSIRXD1H                 GPIO_28_FSIRXH_D1

#define GPIO_29_SCITXDA                  GPIO_29_SCIA_TX
#define GPIO_29_EM1SDCKE                 GPIO_29_EMIF1_SDCKE
#define GPIO_29_EQEP3B                   GPIO_29_EQEP3_B
#define GPIO_29_SD_C7                    GPIO_29_SD2_C3
#define GPIO_29_EM1CS3N                  GPIO_29_EMIF1_CS3N
#define GPIO_29_FSIRXCLKH                GPIO_29_FSIRXH_CLK

#define GPIO_30_CANRXA                   GPIO_30_CANA_RX
#define GPIO_30_EM1CLK                   GPIO_30_EMIF1_CLK
#define GPIO_30_MCANRXA                  GPIO_30_MCAN_RX
#define GPIO_30_EQEP3S                   GPIO_30_EQEP3_STROBE
#define GPIO_30_SD_D8                    GPIO_30_SD2_D4
#define GPIO_30_EM1CS4N                  GPIO_30_EMIF1_CS4N
#define GPIO_30_SPISIMOD                 GPIO_30_SPID_SIMO

#define GPIO_31_CANTXA                   GPIO_31_CANA_TX
#define GPIO_31_EM1WEN                   GPIO_31_EMIF1_WEN
#define GPIO_31_MCANTXA                  GPIO_31_MCAN_TX
#define GPIO_31_EQEP3I                   GPIO_31_EQEP3_INDEX
#define GPIO_31_SD_C8                    GPIO_31_SD2_C4
#define GPIO_31_EM1RNW                   GPIO_31_EMIF1_RNW
#define GPIO_31_SDAA                     GPIO_31_I2CA_SDA
#define GPIO_31_SPISOMID                 GPIO_31_SPID_SOMI

#define GPIO_32_SDAA                     GPIO_32_I2CA_SDA
#define GPIO_32_EM1CS0N                  GPIO_32_EMIF1_CS0N
#define GPIO_32_SPISIMOA                 GPIO_32_SPIA_SIMO
#define GPIO_32_EM1OEN                   GPIO_32_EMIF1_OEN
#define GPIO_32_SCLA                     GPIO_32_I2CA_SCL
#define GPIO_32_SPICLKD                  GPIO_32_SPID_CLK

#define GPIO_33_SCLA                     GPIO_33_I2CA_SCL
#define GPIO_33_EM1RNW                   GPIO_33_EMIF1_RNW
#define GPIO_33_SPISOMIA                 GPIO_33_SPIA_SOMI
#define GPIO_33_EM1BA0                   GPIO_33_EMIF1_BA0
#define GPIO_33_SPISTED                  GPIO_33_SPID_STEN

#define GPIO_34_EM1CS2N                  GPIO_34_EMIF1_CS2N
#define GPIO_34_SPICLKA                  GPIO_34_SPIA_CLK
#define GPIO_34_SDAB                     GPIO_34_I2CB_SDA
#define GPIO_34_EM1BA1                   GPIO_34_EMIF1_BA1
#define GPIO_34_SCITXDA                  GPIO_34_SCIA_TX

#define GPIO_35_SCIRXDA                  GPIO_35_SCIA_RX
#define GPIO_35_EM1CS3N                  GPIO_35_EMIF1_CS3N
#define GPIO_35_SPISTEA                  GPIO_35_SPIA_STEN
#define GPIO_35_SCLB                     GPIO_35_I2CB_SCL
#define GPIO_35_EM1A0                    GPIO_35_EMIF1_A0

#define GPIO_36_SCITXDA                  GPIO_36_SCIA_TX
#define GPIO_36_EM1WAIT0                 GPIO_36_EMIF1_WAIT
#define GPIO_36_CANRXA                   GPIO_36_CANA_RX
#define GPIO_36_EM1A1                    GPIO_36_EMIF1_A1
#define GPIO_36_SD_D1                    GPIO_36_SD1_D1

#define GPIO_37_EM1OEN                   GPIO_37_EMIF1_OEN
#define GPIO_37_CANTXA                   GPIO_37_CANA_TX
#define GPIO_37_EM1A2                    GPIO_37_EMIF1_A2
#define GPIO_37_SD_D2                    GPIO_37_SD1_D2

#define GPIO_38_EM1A0                    GPIO_38_EMIF1_A0
#define GPIO_38_SCITXDC                  GPIO_38_SCIC_TX
#define GPIO_38_CANTXB                   GPIO_38_CANB_TX
#define GPIO_38_EM1A3                    GPIO_38_EMIF1_A3
#define GPIO_38_SD_D3                    GPIO_38_SD1_D3

#define GPIO_39_EM1A1                    GPIO_39_EMIF1_A1
#define GPIO_39_SCIRXDC                  GPIO_39_SCIC_RX
#define GPIO_39_CANRXB                   GPIO_39_CANB_RX
#define GPIO_39_EM1A4                    GPIO_39_EMIF1_A4
#define GPIO_39_SD_D4                    GPIO_39_SD1_D4

#define GPIO_40_EM1A2                    GPIO_40_EMIF1_A2
#define GPIO_40_SDAB                     GPIO_40_I2CB_SDA

#define GPIO_41_EM1A3                    GPIO_41_EMIF1_A3
#define GPIO_41_SCLB                     GPIO_41_I2CB_SCL

#define GPIO_42_SDAA                     GPIO_42_I2CA_SDA
#define GPIO_42_SCITXDA                  GPIO_42_SCIA_TX

#define GPIO_43_SCLA                     GPIO_43_I2CA_SCL
#define GPIO_43_SCIRXDA                  GPIO_43_SCIA_RX

#define GPIO_44_EM1A4                    GPIO_44_EMIF1_A4

#define GPIO_45_EM1A5                    GPIO_45_EMIF1_A5

#define GPIO_46_EM1A6                    GPIO_46_EMIF1_A6
#define GPIO_46_SCIRXDD                  GPIO_46_SCID_RX

#define GPIO_47_EM1A7                    GPIO_47_EMIF1_A7
#define GPIO_47_SCITXDD                  GPIO_47_SCID_TX

#define GPIO_48_EM1A8                    GPIO_48_EMIF1_A8
#define GPIO_48_SCITXDA                  GPIO_48_SCIA_TX
#define GPIO_48_SD_D1                    GPIO_48_SD1_D1

#define GPIO_49_EM1A9                    GPIO_49_EMIF1_A9
#define GPIO_49_SCIRXDA                  GPIO_49_SCIA_RX
#define GPIO_49_SD_C1                    GPIO_49_SD1_C1
#define GPIO_49_EM1A5                    GPIO_49_EMIF1_A5
#define GPIO_49_SD_D5                    GPIO_49_SD2_D1
#define GPIO_49_FSITXD0A                 GPIO_49_FSITXA_D0

#define GPIO_50_EQEP1A                   GPIO_50_EQEP1_A
#define GPIO_50_EM1A10                   GPIO_50_EMIF1_A10
#define GPIO_50_SPISIMOC                 GPIO_50_SPIC_SIMO
#define GPIO_50_SD_D2                    GPIO_50_SD1_D2
#define GPIO_50_EM1A6                    GPIO_50_EMIF1_A6
#define GPIO_50_SD_D6                    GPIO_50_SD2_D2
#define GPIO_50_FSITXD1A                 GPIO_50_FSITXA_D1

#define GPIO_51_EQEP1B                   GPIO_51_EQEP1_B
#define GPIO_51_EM1A11                   GPIO_51_EMIF1_A11
#define GPIO_51_SPISOMIC                 GPIO_51_SPIC_SOMI
#define GPIO_51_SD_C2                    GPIO_51_SD1_C2
#define GPIO_51_EM1A7                    GPIO_51_EMIF1_A7
#define GPIO_51_SD_D7                    GPIO_51_SD2_D3
#define GPIO_51_FSITXCLKA                GPIO_51_FSITXA_CLK

#define GPIO_52_EQEP1S                   GPIO_52_EQEP1_STROBE
#define GPIO_52_EM1A12                   GPIO_52_EMIF1_A12
#define GPIO_52_SPICLKC                  GPIO_52_SPIC_CLK
#define GPIO_52_SD_D3                    GPIO_52_SD1_D3
#define GPIO_52_EM1A8                    GPIO_52_EMIF1_A8
#define GPIO_52_SD_D8                    GPIO_52_SD2_D4
#define GPIO_52_FSIRXD0A                 GPIO_52_FSIRXA_D0

#define GPIO_53_EQEP1I                   GPIO_53_EQEP1_INDEX
#define GPIO_53_EM1D31                   GPIO_53_EMIF1_D31
#define GPIO_53_EM2D15                   GPIO_53_EMIF2_D15
#define GPIO_53_SPISTEC                  GPIO_53_SPIC_STEN
#define GPIO_53_SD_C3                    GPIO_53_SD1_C3
#define GPIO_53_EM1A9                    GPIO_53_EMIF1_A9
#define GPIO_53_SD_C1                    GPIO_53_SD1_C1
#define GPIO_53_FSIRXD1A                 GPIO_53_FSIRXA_D1

#define GPIO_54_SPISIMOA                 GPIO_54_SPIA_SIMO
#define GPIO_54_EM1D30                   GPIO_54_EMIF1_D30
#define GPIO_54_EM2D14                   GPIO_54_EMIF2_D14
#define GPIO_54_EQEP2A                   GPIO_54_EQEP2_A
#define GPIO_54_SCITXDB                  GPIO_54_SCIB_TX
#define GPIO_54_SD_D4                    GPIO_54_SD1_D4
#define GPIO_54_EM1A10                   GPIO_54_EMIF1_A10
#define GPIO_54_SD_C2                    GPIO_54_SD1_C2
#define GPIO_54_FSIRXCLKA                GPIO_54_FSIRXA_CLK

#define GPIO_55_SPISOMIA                 GPIO_55_SPIA_SOMI
#define GPIO_55_EM1D29                   GPIO_55_EMIF1_D29
#define GPIO_55_EM2D13                   GPIO_55_EMIF2_D13
#define GPIO_55_EQEP2B                   GPIO_55_EQEP2_B
#define GPIO_55_SCIRXDB                  GPIO_55_SCIB_RX
#define GPIO_55_SD_C4                    GPIO_55_SD1_C4
#define GPIO_55_EM1D0                    GPIO_55_EMIF1_D0
#define GPIO_55_SD_C3                    GPIO_55_SD1_C3
#define GPIO_55_FSITXD0B                 GPIO_55_FSITXB_D0

#define GPIO_56_SPICLKA                  GPIO_56_SPIA_CLK
#define GPIO_56_EM1D28                   GPIO_56_EMIF1_D28
#define GPIO_56_EM2D12                   GPIO_56_EMIF2_D12
#define GPIO_56_EQEP2S                   GPIO_56_EQEP2_STROBE
#define GPIO_56_SCITXDC                  GPIO_56_SCIC_TX
#define GPIO_56_SD_D5                    GPIO_56_SD2_D1
#define GPIO_56_EM1D1                    GPIO_56_EMIF1_D1
#define GPIO_56_SDAA                     GPIO_56_I2CA_SDA
#define GPIO_56_SD_C4                    GPIO_56_SD1_C4
#define GPIO_56_FSITXCLKB                GPIO_56_FSITXB_CLK

#define GPIO_57_SPISTEA                  GPIO_57_SPIA_STEN
#define GPIO_57_EM1D27                   GPIO_57_EMIF1_D27
#define GPIO_57_EM2D11                   GPIO_57_EMIF2_D11
#define GPIO_57_EQEP2I                   GPIO_57_EQEP2_INDEX
#define GPIO_57_SCIRXDC                  GPIO_57_SCIC_RX
#define GPIO_57_SD_C5                    GPIO_57_SD2_C1
#define GPIO_57_EM1D2                    GPIO_57_EMIF1_D2
#define GPIO_57_SCLA                     GPIO_57_I2CA_SCL
#define GPIO_57_FSITXD1B                 GPIO_57_FSITXB_D1

#define GPIO_58_EM1D26                   GPIO_58_EMIF1_D26
#define GPIO_58_EM2D10                   GPIO_58_EMIF2_D10
#define GPIO_58_SPICLKB                  GPIO_58_SPIB_CLK
#define GPIO_58_SD_D6                    GPIO_58_SD2_D2
#define GPIO_58_EM1D3                    GPIO_58_EMIF1_D3
#define GPIO_58_SD_C6                    GPIO_58_SD2_C2
#define GPIO_58_FSIRXD0B                 GPIO_58_FSIRXB_D0
#define GPIO_58_SPISIMOA                 GPIO_58_SPIA_SIMO

#define GPIO_59_EM1D25                   GPIO_59_EMIF1_D25
#define GPIO_59_EM2D9                    GPIO_59_EMIF2_D9
#define GPIO_59_SPISTEB                  GPIO_59_SPIB_STEN
#define GPIO_59_SD_C6                    GPIO_59_SD2_C2
#define GPIO_59_EM1D4                    GPIO_59_EMIF1_D4
#define GPIO_59_SD_C7                    GPIO_59_SD2_C3
#define GPIO_59_FSIRXD1B                 GPIO_59_FSIRXB_D1
#define GPIO_59_SPISOMIA                 GPIO_59_SPIA_SOMI

#define GPIO_60_EM1D24                   GPIO_60_EMIF1_D24
#define GPIO_60_EM2D8                    GPIO_60_EMIF2_D8
#define GPIO_60_SPISIMOB                 GPIO_60_SPIB_SIMO
#define GPIO_60_SD_D7                    GPIO_60_SD2_D3
#define GPIO_60_EM1D5                    GPIO_60_EMIF1_D5
#define GPIO_60_SD_C8                    GPIO_60_SD2_C4
#define GPIO_60_FSIRXCLKB                GPIO_60_FSIRXB_CLK
#define GPIO_60_SPICLKA                  GPIO_60_SPIA_CLK

#define GPIO_61_EM1D23                   GPIO_61_EMIF1_D23
#define GPIO_61_EM2D7                    GPIO_61_EMIF2_D7
#define GPIO_61_SPISOMIB                 GPIO_61_SPIB_SOMI
#define GPIO_61_SD_C7                    GPIO_61_SD2_C3
#define GPIO_61_EM1D6                    GPIO_61_EMIF1_D6
#define GPIO_61_CANRXA                   GPIO_61_CANA_RX
#define GPIO_61_SPISTEA                  GPIO_61_SPIA_STEN

#define GPIO_62_SCIRXDC                  GPIO_62_SCIC_RX
#define GPIO_62_EM1D22                   GPIO_62_EMIF1_D22
#define GPIO_62_EM2D6                    GPIO_62_EMIF2_D6
#define GPIO_62_EQEP3A                   GPIO_62_EQEP3_A
#define GPIO_62_CANRXA                   GPIO_62_CANA_RX
#define GPIO_62_SD_D8                    GPIO_62_SD2_D4
#define GPIO_62_EM1D7                    GPIO_62_EMIF1_D7
#define GPIO_62_CANTXA                   GPIO_62_CANA_TX

#define GPIO_63_SCITXDC                  GPIO_63_SCIC_TX
#define GPIO_63_EM1D21                   GPIO_63_EMIF1_D21
#define GPIO_63_EM2D5                    GPIO_63_EMIF2_D5
#define GPIO_63_EQEP3B                   GPIO_63_EQEP3_B
#define GPIO_63_CANTXA                   GPIO_63_CANA_TX
#define GPIO_63_SD_C8                    GPIO_63_SD2_C4
#define GPIO_63_SD_D1                    GPIO_63_SD1_D1
#define GPIO_63_SPISIMOB                 GPIO_63_SPIB_SIMO

#define GPIO_64_EM1D20                   GPIO_64_EMIF1_D20
#define GPIO_64_EM2D4                    GPIO_64_EMIF2_D4
#define GPIO_64_EQEP3S                   GPIO_64_EQEP3_STROBE
#define GPIO_64_SCIRXDA                  GPIO_64_SCIA_RX
#define GPIO_64_SD_C1                    GPIO_64_SD1_C1
#define GPIO_64_SPISOMIB                 GPIO_64_SPIB_SOMI

#define GPIO_65_EM1D19                   GPIO_65_EMIF1_D19
#define GPIO_65_EM2D3                    GPIO_65_EMIF2_D3
#define GPIO_65_EQEP3I                   GPIO_65_EQEP3_INDEX
#define GPIO_65_SCITXDA                  GPIO_65_SCIA_TX
#define GPIO_65_SD_D2                    GPIO_65_SD1_D2
#define GPIO_65_SPICLKB                  GPIO_65_SPIB_CLK

#define GPIO_66_EM1D18                   GPIO_66_EMIF1_D18
#define GPIO_66_EM2D2                    GPIO_66_EMIF2_D2
#define GPIO_66_SDAB                     GPIO_66_I2CB_SDA
#define GPIO_66_SD_C2                    GPIO_66_SD1_C2
#define GPIO_66_SPISTEB                  GPIO_66_SPIB_STEN

#define GPIO_67_EM1D17                   GPIO_67_EMIF1_D17
#define GPIO_67_EM2D1                    GPIO_67_EMIF2_D1
#define GPIO_67_SD_D3                    GPIO_67_SD1_D3

#define GPIO_68_EM1D16                   GPIO_68_EMIF1_D16
#define GPIO_68_EM2D0                    GPIO_68_EMIF2_D0
#define GPIO_68_SD_C3                    GPIO_68_SD1_C3

#define GPIO_69_EM1D15                   GPIO_69_EMIF1_D15
#define GPIO_69_SCLB                     GPIO_69_I2CB_SCL
#define GPIO_69_SD_D4                    GPIO_69_SD1_D4
#define GPIO_69_SPISIMOC                 GPIO_69_SPIC_SIMO

#define GPIO_70_EM1D14                   GPIO_70_EMIF1_D14
#define GPIO_70_CANRXA                   GPIO_70_CANA_RX
#define GPIO_70_SCITXDB                  GPIO_70_SCIB_TX
#define GPIO_70_SD_C4                    GPIO_70_SD1_C4
#define GPIO_70_SPISOMIC                 GPIO_70_SPIC_SOMI

#define GPIO_71_EM1D13                   GPIO_71_EMIF1_D13
#define GPIO_71_CANTXA                   GPIO_71_CANA_TX
#define GPIO_71_SCIRXDB                  GPIO_71_SCIB_RX
#define GPIO_71_SPICLKC                  GPIO_71_SPIC_CLK

#define GPIO_72_EM1D12                   GPIO_72_EMIF1_D12
#define GPIO_72_CANTXB                   GPIO_72_CANB_TX
#define GPIO_72_SCITXDC                  GPIO_72_SCIC_TX
#define GPIO_72_SPISTEC                  GPIO_72_SPIC_STEN

#define GPIO_73_EM1D11                   GPIO_73_EMIF1_D11
#define GPIO_73_CANRXB                   GPIO_73_CANB_RX
#define GPIO_73_SCIRXDC                  GPIO_73_SCIC_RX
#define GPIO_73_SD_D6                    GPIO_73_SD2_D2

#define GPIO_74_EM1D10                   GPIO_74_EMIF1_D10
#define GPIO_74_SD_C6                    GPIO_74_SD2_C2

#define GPIO_75_EM1D9                    GPIO_75_EMIF1_D9
#define GPIO_75_SD_D7                    GPIO_75_SD2_D3

#define GPIO_76_EM1D8                    GPIO_76_EMIF1_D8
#define GPIO_76_SCITXDD                  GPIO_76_SCID_TX
#define GPIO_76_SD_C7                    GPIO_76_SD2_C3

#define GPIO_77_EM1D7                    GPIO_77_EMIF1_D7
#define GPIO_77_SCIRXDD                  GPIO_77_SCID_RX
#define GPIO_77_SD_D8                    GPIO_77_SD2_D4

#define GPIO_78_EM1D6                    GPIO_78_EMIF1_D6
#define GPIO_78_EQEP2A                   GPIO_78_EQEP2_A
#define GPIO_78_SD_C8                    GPIO_78_SD2_C4

#define GPIO_79_EM1D5                    GPIO_79_EMIF1_D5
#define GPIO_79_EQEP2B                   GPIO_79_EQEP2_B
#define GPIO_79_SD_D5                    GPIO_79_SD2_D1

#define GPIO_80_EM1D4                    GPIO_80_EMIF1_D4
#define GPIO_80_EQEP2S                   GPIO_80_EQEP2_STROBE
#define GPIO_80_SD_C5                    GPIO_80_SD2_C1

#define GPIO_81_EM1D3                    GPIO_81_EMIF1_D3
#define GPIO_81_EQEP2I                   GPIO_81_EQEP2_INDEX

#define GPIO_82_EM1D2                    GPIO_82_EMIF1_D2

#define GPIO_83_EM1D1                    GPIO_83_EMIF1_D1

#define GPIO_84_SCITXDA                  GPIO_84_SCIA_TX
#define GPIO_84_CMUARTTXA                GPIO_84_UARTA_TX

#define GPIO_85_EM1D0                    GPIO_85_EMIF1_D0
#define GPIO_85_SCIRXDA                  GPIO_85_SCIA_RX
#define GPIO_85_CMUARTRXA                GPIO_85_UARTA_RX

#define GPIO_86_EM1A13                   GPIO_86_EMIF1_A13
#define GPIO_86_EM1CAS                   GPIO_86_EMIF1_CAS
#define GPIO_86_SCITXDB                  GPIO_86_SCIB_TX

#define GPIO_87_EM1A14                   GPIO_87_EMIF1_A14
#define GPIO_87_EM1RAS                   GPIO_87_EMIF1_RAS
#define GPIO_87_SCIRXDB                  GPIO_87_SCIB_RX
#define GPIO_87_EM1DQM3                  GPIO_87_EMIF1_DQM3

#define GPIO_88_EM1A15                   GPIO_88_EMIF1_A15
#define GPIO_88_EM1DQM0                  GPIO_88_EMIF1_DQM0
#define GPIO_88_EM1DQM1                  GPIO_88_EMIF1_DQM1

#define GPIO_89_EM1A16                   GPIO_89_EMIF1_A16
#define GPIO_89_EM1DQM1                  GPIO_89_EMIF1_DQM1
#define GPIO_89_SCITXDC                  GPIO_89_SCIC_TX
#define GPIO_89_EM1CAS                   GPIO_89_EMIF1_CAS

#define GPIO_90_EM1A17                   GPIO_90_EMIF1_A17
#define GPIO_90_EM1DQM2                  GPIO_90_EMIF1_DQM2
#define GPIO_90_SCIRXDC                  GPIO_90_SCIC_RX
#define GPIO_90_EM1RAS                   GPIO_90_EMIF1_RAS

#define GPIO_91_EM1A18                   GPIO_91_EMIF1_A18
#define GPIO_91_EM1DQM3                  GPIO_91_EMIF1_DQM3
#define GPIO_91_SDAA                     GPIO_91_I2CA_SDA
#define GPIO_91_EM1DQM2                  GPIO_91_EMIF1_DQM2
#define GPIO_91_PMBCLK                   GPIO_91_PMBUSA_SCL
#define GPIO_91_FSIRXD0F                 GPIO_91_FSIRXF_D0
#define GPIO_91_SPISIMOD                 GPIO_91_SPID_SIMO

#define GPIO_92_EM1A19                   GPIO_92_EMIF1_A19
#define GPIO_92_EM1BA1                   GPIO_92_EMIF1_BA1
#define GPIO_92_SCLA                     GPIO_92_I2CA_SCL
#define GPIO_92_EM1DQM0                  GPIO_92_EMIF1_DQM0
#define GPIO_92_PMBDAT                   GPIO_92_PMBUSA_SDA
#define GPIO_92_FSIRXD1F                 GPIO_92_FSIRXF_D1
#define GPIO_92_SPISOMID                 GPIO_92_SPID_SOMI

#define GPIO_93_EM1BA0                   GPIO_93_EMIF1_BA0
#define GPIO_93_SCITXDD                  GPIO_93_SCID_TX
#define GPIO_93_PMBALERT                 GPIO_93_PMBUSA_ALERT
#define GPIO_93_FSIRXCLKF                GPIO_93_FSIRXF_CLK
#define GPIO_93_SPICLKD                  GPIO_93_SPID_CLK

#define GPIO_94_SCIRXDD                  GPIO_94_SCID_RX
#define GPIO_94_EM1BA1                   GPIO_94_EMIF1_BA1
#define GPIO_94_PMBCTRL                  GPIO_94_PMBUSA_CTL
#define GPIO_94_FSIRXD0G                 GPIO_94_FSIRXG_D0
#define GPIO_94_SPISTED                  GPIO_94_SPID_STEN

#define GPIO_95_EM2A12                   GPIO_95_EMIF2_A12
#define GPIO_95_FSIRXD1G                 GPIO_95_FSIRXG_D1

#define GPIO_96_EM2DQM1                  GPIO_96_EMIF2_DQM1
#define GPIO_96_EQEP1A                   GPIO_96_EQEP1_A
#define GPIO_96_FSIRXCLKG                GPIO_96_FSIRXG_CLK

#define GPIO_97_EM2DQM0                  GPIO_97_EMIF2_DQM0
#define GPIO_97_EQEP1B                   GPIO_97_EQEP1_B
#define GPIO_97_FSIRXD0H                 GPIO_97_FSIRXH_D0

#define GPIO_98_EM2A0                    GPIO_98_EMIF2_A0
#define GPIO_98_EQEP1S                   GPIO_98_EQEP1_STROBE
#define GPIO_98_FSIRXD1H                 GPIO_98_FSIRXH_D1

#define GPIO_99_EM2A1                    GPIO_99_EMIF2_A1
#define GPIO_99_EQEP1I                   GPIO_99_EQEP1_INDEX
#define GPIO_99_FSIRXCLKH                GPIO_99_FSIRXH_CLK

#define GPIO_100_EM2A2                    GPIO_100_EMIF2_A2
#define GPIO_100_EQEP2A                   GPIO_100_EQEP2_A
#define GPIO_100_SPISIMOC                 GPIO_100_SPIC_SIMO
#define GPIO_100_FSITXD0A                 GPIO_100_FSITXA_D0

#define GPIO_101_EM2A3                    GPIO_101_EMIF2_A3
#define GPIO_101_EQEP2B                   GPIO_101_EQEP2_B
#define GPIO_101_SPISOMIC                 GPIO_101_SPIC_SOMI
#define GPIO_101_FSITXD1A                 GPIO_101_FSITXA_D1

#define GPIO_102_EM2A4                    GPIO_102_EMIF2_A4
#define GPIO_102_EQEP2S                   GPIO_102_EQEP2_STROBE
#define GPIO_102_SPICLKC                  GPIO_102_SPIC_CLK
#define GPIO_102_FSITXCLKA                GPIO_102_FSITXA_CLK

#define GPIO_103_EM2A5                    GPIO_103_EMIF2_A5
#define GPIO_103_EQEP2I                   GPIO_103_EQEP2_INDEX
#define GPIO_103_SPISTEC                  GPIO_103_SPIC_STEN
#define GPIO_103_FSIRXD0A                 GPIO_103_FSIRXA_D0

#define GPIO_104_SDAA                     GPIO_104_I2CA_SDA
#define GPIO_104_EM2A6                    GPIO_104_EMIF2_A6
#define GPIO_104_EQEP3A                   GPIO_104_EQEP3_A
#define GPIO_104_SCITXDD                  GPIO_104_SCID_TX
#define GPIO_104_FSIRXD1A                 GPIO_104_FSIRXA_D1

#define GPIO_105_SCLA                     GPIO_105_I2CA_SCL
#define GPIO_105_EM2A7                    GPIO_105_EMIF2_A7
#define GPIO_105_EQEP3B                   GPIO_105_EQEP3_B
#define GPIO_105_SCIRXDD                  GPIO_105_SCID_RX
#define GPIO_105_FSIRXCLKA                GPIO_105_FSIRXA_CLK

#define GPIO_106_EM2A8                    GPIO_106_EMIF2_A8
#define GPIO_106_EQEP3S                   GPIO_106_EQEP3_STROBE
#define GPIO_106_SCITXDC                  GPIO_106_SCIC_TX
#define GPIO_106_FSITXD0B                 GPIO_106_FSITXB_D0

#define GPIO_107_EM2A9                    GPIO_107_EMIF2_A9
#define GPIO_107_EQEP3I                   GPIO_107_EQEP3_INDEX
#define GPIO_107_SCIRXDC                  GPIO_107_SCIC_RX
#define GPIO_107_FSITXD1B                 GPIO_107_FSITXB_D1

#define GPIO_108_EM2A10                   GPIO_108_EMIF2_A10
#define GPIO_108_FSITXCLKB                GPIO_108_FSITXB_CLK

#define GPIO_109_EM2A11                   GPIO_109_EMIF2_A11

#define GPIO_110_EM2WAIT0                 GPIO_110_EMIF2_WAIT
#define GPIO_110_FSIRXD0B                 GPIO_110_FSIRXB_D0

#define GPIO_111_EM2BA0                   GPIO_111_EMIF2_BA0
#define GPIO_111_FSIRXD1B                 GPIO_111_FSIRXB_D1

#define GPIO_112_EM2BA1                   GPIO_112_EMIF2_BA1
#define GPIO_112_FSIRXCLKB                GPIO_112_FSIRXB_CLK

#define GPIO_113_EM2CAS                   GPIO_113_EMIF2_CAS

#define GPIO_114_EM2RAS                   GPIO_114_EMIF2_RAS

#define GPIO_115_EM2CS0N                  GPIO_115_EMIF2_CS0N
#define GPIO_115_FSIRXD0C                 GPIO_115_FSIRXC_D0

#define GPIO_116_EM2CS2N                  GPIO_116_EMIF2_CS2N
#define GPIO_116_FSIRXD1C                 GPIO_116_FSIRXC_D1

#define GPIO_117_EM2SDCKE                 GPIO_117_EMIF2_SDCKE
#define GPIO_117_FSIRXCLKC                GPIO_117_FSIRXC_CLK

#define GPIO_118_EM2CLK                   GPIO_118_EMIF2_CLK
#define GPIO_118_FSIRXD0D                 GPIO_118_FSIRXD_D0

#define GPIO_119_EM2RNW                   GPIO_119_EMIF2_RNW
#define GPIO_119_FSIRXD1D                 GPIO_119_FSIRXD_D1

#define GPIO_120_EM2WEN                   GPIO_120_EMIF2_WEN
#define GPIO_120_FSIRXCLKD                GPIO_120_FSIRXD_CLK

#define GPIO_121_EM2OEN                   GPIO_121_EMIF2_OEN
#define GPIO_121_FSIRXD0E                 GPIO_121_FSIRXE_D0

#define GPIO_122_EM2D15                   GPIO_122_EMIF2_D15
#define GPIO_122_SPISIMOC                 GPIO_122_SPIC_SIMO
#define GPIO_122_SD_D1                    GPIO_122_SD1_D1

#define GPIO_123_EM2D14                   GPIO_123_EMIF2_D14
#define GPIO_123_SPISOMIC                 GPIO_123_SPIC_SOMI
#define GPIO_123_SD_C1                    GPIO_123_SD1_C1

#define GPIO_124_EM2D13                   GPIO_124_EMIF2_D13
#define GPIO_124_SPICLKC                  GPIO_124_SPIC_CLK
#define GPIO_124_SD_D2                    GPIO_124_SD1_D2

#define GPIO_125_EM2D12                   GPIO_125_EMIF2_D12
#define GPIO_125_SPISTEC                  GPIO_125_SPIC_STEN
#define GPIO_125_SD_C2                    GPIO_125_SD1_C2
#define GPIO_125_FSIRXD1E                 GPIO_125_FSIRXE_D1
#define GPIO_125_ECAT_LATCH0              GPIO_125_ESC_LATCH0

#define GPIO_126_EM2D11                   GPIO_126_EMIF2_D11
#define GPIO_126_SD_D3                    GPIO_126_SD1_D3
#define GPIO_126_FSIRXCLKE                GPIO_126_FSIRXE_CLK
#define GPIO_126_ECAT_LATCH1              GPIO_126_ESC_LATCH1

#define GPIO_127_EM2D10                   GPIO_127_EMIF2_D10
#define GPIO_127_SD_C3                    GPIO_127_SD1_C3
#define GPIO_127_ECAT_SYNC0               GPIO_127_ESC_SYNC0

#define GPIO_128_EM2D9                    GPIO_128_EMIF2_D9
#define GPIO_128_SD_D4                    GPIO_128_SD1_D4
#define GPIO_128_ECAT_SYNC1               GPIO_128_ESC_SYNC1

#define GPIO_129_EM2D8                    GPIO_129_EMIF2_D8
#define GPIO_129_SD_C4                    GPIO_129_SD1_C4
#define GPIO_129_ECAT_TX1_ENA             GPIO_129_ESC_TX1_ENA

#define GPIO_130_EM2D7                    GPIO_130_EMIF2_D7
#define GPIO_130_SD_D5                    GPIO_130_SD2_D1
#define GPIO_130_ECAT_TX1_CLK             GPIO_130_ESC_TX1_CLK

#define GPIO_131_EM2D6                    GPIO_131_EMIF2_D6
#define GPIO_131_SD_C5                    GPIO_131_SD2_C1
#define GPIO_131_ECAT_TX1_DATA0           GPIO_131_ESC_TX1_DATA0

#define GPIO_132_EM2D5                    GPIO_132_EMIF2_D5
#define GPIO_132_SD_D6                    GPIO_132_SD2_D2
#define GPIO_132_ECAT_TX1_DATA1           GPIO_132_ESC_TX1_DATA1

#define GPIO_133_SD_C6                    GPIO_133_SD2_C2

#define GPIO_134_EM2D4                    GPIO_134_EMIF2_D4
#define GPIO_134_SD_D7                    GPIO_134_SD2_D3
#define GPIO_134_ECAT_TX1_DATA2           GPIO_134_ESC_TX1_DATA2

#define GPIO_135_EM2D3                    GPIO_135_EMIF2_D3
#define GPIO_135_SCITXDA                  GPIO_135_SCIA_TX
#define GPIO_135_SD_C7                    GPIO_135_SD2_C3
#define GPIO_135_ECAT_TX1_DATA3           GPIO_135_ESC_TX1_DATA3

#define GPIO_136_EM2D2                    GPIO_136_EMIF2_D2
#define GPIO_136_SCIRXDA                  GPIO_136_SCIA_RX
#define GPIO_136_SD_D8                    GPIO_136_SD2_D4
#define GPIO_136_ECAT_RX1_DV              GPIO_136_ESC_RX1_DV

#define GPIO_137_EM2D1                    GPIO_137_EMIF2_D1
#define GPIO_137_SCITXDB                  GPIO_137_SCIB_TX
#define GPIO_137_SD_C8                    GPIO_137_SD2_C4
#define GPIO_137_ECAT_RX1_CLK             GPIO_137_ESC_RX1_CLK

#define GPIO_138_EM2D0                    GPIO_138_EMIF2_D0
#define GPIO_138_SCIRXDB                  GPIO_138_SCIB_RX
#define GPIO_138_ECAT_RX1_ERR             GPIO_138_ESC_RX1_ERR

#define GPIO_139_SCIRXDC                  GPIO_139_SCIC_RX
#define GPIO_139_ECAT_RX1_DATA0           GPIO_139_ESC_RX1_DATA0

#define GPIO_140_SCITXDC                  GPIO_140_SCIC_TX
#define GPIO_140_ECAT_RX1_DATA1           GPIO_140_ESC_RX1_DATA1

#define GPIO_141_SCIRXDD                  GPIO_141_SCID_RX
#define GPIO_141_ECAT_RX1_DATA2           GPIO_141_ESC_RX1_DATA2

#define GPIO_142_SCITXDD                  GPIO_142_SCID_TX
#define GPIO_142_ECAT_RX1_DATA3           GPIO_142_ESC_RX1_DATA3

#define GPIO_143_ECAT_LED_LINK0_ACTIVE    GPIO_143_ESC_LED_LINK0_ACTIVE

#define GPIO_144_ECAT_LED_LINK1_ACTIVE    GPIO_144_ESC_LED_LINK1_ACTIVE

#define GPIO_148_ECAT_PHY0_LINKSTATUS     GPIO_148_ESC_PHY0_LINKSTATUS

#define GPIO_149_ECAT_PHY1_LINKSTATUS     GPIO_149_ESC_PHY1_LINKSTATUS

#define GPIO_150_ECAT_I2C_SDA             GPIO_150_ESC_I2C_SDA

#define GPIO_151_ECAT_I2C_SCL             GPIO_151_ESC_I2C_SCL

#define GPIO_152_ECAT_MDIO_CLK            GPIO_152_ESC_MDIO_CLK

#define GPIO_153_ECAT_MDIO_DATA           GPIO_153_ESC_MDIO_DATA

#define GPIO_154_ECAT_PHY_CLK             GPIO_154_ESC_PHY_CLK

#define GPIO_155_ECAT_PHY_RESETN          GPIO_155_ESC_PHY_RESETN

#define GPIO_156_ECAT_TX0_ENA             GPIO_156_ESC_TX0_ENA

#define GPIO_157_ECAT_TX0_CLK             GPIO_157_ESC_TX0_CLK

#define GPIO_158_ECAT_TX0_DATA0           GPIO_158_ESC_TX0_DATA0

#define GPIO_159_ECAT_TX0_DATA1           GPIO_159_ESC_TX0_DATA1

#define GPIO_160_ECAT_TX0_DATA2           GPIO_160_ESC_TX0_DATA2

#define GPIO_161_ECAT_TX0_DATA3           GPIO_161_ESC_TX0_DATA3

#define GPIO_162_ECAT_RX0_DV              GPIO_162_ESC_RX0_DV

#define GPIO_163_ECAT_RX0_CLK             GPIO_163_ESC_RX0_CLK

#define GPIO_164_ECAT_RX0_ERR             GPIO_164_ESC_RX0_ERR

#define GPIO_165_ECAT_RX0_DATA0           GPIO_165_ESC_RX0_DATA0

#define GPIO_166_ECAT_RX0_DATA1           GPIO_166_ESC_RX0_DATA1

#define GPIO_167_ECAT_RX0_DATA2           GPIO_167_ESC_RX0_DATA2

#define GPIO_168_ECAT_RX0_DATA3           GPIO_168_ESC_RX0_DATA3

#endif // __PIN_MAP_LEGACY_H__
