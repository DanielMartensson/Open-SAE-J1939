//###########################################################################
//
// FILE:   mcbsp.h
//
// TITLE:  C28x McBSP driver.
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

#ifndef MCBSP_H
#define MCBSP_H

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
//! \addtogroup mcbsp_api McBSP
//! @{
//
//*****************************************************************************

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_mcbsp.h"
#include "inc/hw_memmap.h"
#include "debug.h"
#include "interrupt.h"

//*****************************************************************************
//
// Defines for the API.
//
//*****************************************************************************
//*****************************************************************************
//
// Define to specify mask for setting the word and frame length in
// McBSP_setTxDataSize() anf McBSP_setRxDataSize().
//
//*****************************************************************************
#define MCBSP_XCR1_M            (MCBSP_XCR1_XWDLEN1_M |                       \
                                 MCBSP_XCR1_XFRLEN1_M)
#define MCBSP_RCR1_M            (MCBSP_RCR1_RWDLEN1_M |                       \
                                 MCBSP_RCR1_RFRLEN1_M)
#define MCBSP_XCR2_M            (MCBSP_XCR2_XWDLEN2_M |                       \
                                 MCBSP_XCR2_XFRLEN2_M)
#define MCBSP_RCR2_M            (MCBSP_RCR2_RWDLEN2_M |                       \
                                 MCBSP_RCR2_RFRLEN2_M)

//*****************************************************************************
//
// Defines the values that can be returned by McBSP_getRxErrorStatus() when
// there is an error in Rx.
//
//*****************************************************************************
#define MCBSP_RX_NO_ERROR                0x0U //!< No error.
#define MCBSP_RX_BUFFER_ERROR            0x4U //!< Buffer Full.
#define MCBSP_RX_FRAME_SYNC_ERROR        0x8U //!< Frame sync error.
#define MCBSP_RX_BUFFER_FRAME_SYNC_ERROR 0xCU //!< Buffer and frame sync error.

//*****************************************************************************
//
// Defines the values that can be returned by McBSP_getTxErrorStatus() when
// there is an error in Tx.
//
//*****************************************************************************
#define MCBSP_TX_NO_ERROR                0x0U //!< No error.
#define MCBSP_TX_BUFFER_ERROR            0x4U //!< Buffer overrun.
#define MCBSP_TX_FRAME_SYNC_ERROR        0x8U //!< Frame sync error.
#define MCBSP_TX_BUFFER_FRAME_SYNC_ERROR 0xCU //!< Buffer and frame sync error.

//*****************************************************************************
//
// Values that can be returned by  McBSP_configureTxMultichannel() and
// McBSP_configureRxMultichannel().
//
//*****************************************************************************
#define MCBSP_ERROR_EXCEEDED_CHANNELS  0x1U //!< Exceeded number of channels.
#define MCBSP_ERROR_2_PARTITION_A      0x2U //!< Error in 2 partition A setup.
#define MCBSP_ERROR_2_PARTITION_B      0x4U //!< Error in 2 partition B setup.
#define MCBSP_ERROR_INVALID_MODE       0x8U //!< Invalid mode.

//*****************************************************************************
//
//! Values that can be passed to McBSP_setRxSignExtension() as the \e
//! mode parameters.
//
//*****************************************************************************
typedef enum
{
    MCBSP_RIGHT_JUSTIFY_FILL_ZERO = 0x0000U, //!< Right justify and
                                             //!< zero fill MSB.
    MCBSP_RIGHT_JUSTIFY_FILL_SIGN = 0x2000U, //!< Right justified sign
                                             //!< extended into MSBs.
    MCBSP_LEFT_JUSTIFY_FILL_ZER0  = 0x4000U  //!< Left justifies LBS
                                             //!< filled with zero.
}McBSP_RxSignExtensionMode;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setClockStopMode() as the \e mode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_CLOCK_MCBSP_MODE        = 0x0000U, //!< Disables clock stop mode.
    MCBSP_CLOCK_SPI_MODE_NO_DELAY = 0x1000U, //!< Enables clock stop mode.
    MCBSP_CLOCK_SPI_MODE_DELAY    = 0x1800U  //!< Enables clock stop mode
                                             //!< with half cycle delay.
}McBSP_ClockStopMode;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setRxInterruptSource() as the
//! \e interruptSource parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_RX_ISR_SOURCE_SERIAL_WORD  = 0x0000U, //!<Interrupt when Rx is ready.
    MCBSP_RX_ISR_SOURCE_END_OF_BLOCK = 0x0010U, //!<Interrupt at block end.
    MCBSP_RX_ISR_SOURCE_FRAME_SYNC   = 0x0020U, //!<Interrupt when
                                                //!<frame sync occurs.
    MCBSP_RX_ISR_SOURCE_SYNC_ERROR   = 0x0030U  //!<Interrupt on
                                                //!<frame sync error.
}McBSP_RxInterruptSource;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setEmulationMode() as the \e
//! emulationMode parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_EMULATION_IMMEDIATE_STOP = 0x0000U, //!< McBSP TX and RX stop when
                                              //!< a breakpoint is reached.
    MCBSP_EMULATION_SOFT_STOP      = 0x0100U, //!< McBSP TX stops after
                                              //!<current word transmitted.
    MCBSP_EMULATION_FREE_RUN       = 0x0200U  //!< McBSP TX and RX run
                                              //!< ignoring the breakpoint.
}McBSP_EmulationMode;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setTxInterruptSource() as the \e
//! interruptSource parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_TX_ISR_SOURCE_TX_READY     = 0x0000U, //!<Interrupt when Tx Ready.
    MCBSP_TX_ISR_SOURCE_END_OF_BLOCK = 0x0010U, //!<Interrupt at block end.
    MCBSP_TX_ISR_SOURCE_FRAME_SYNC   = 0x0020U, //!<Interrupt when frame
                                                //!<sync occurs.
    MCBSP_TX_ISR_SOURCE_SYNC_ERROR   = 0x0030U  //!<Interrupt on frame sync
                                                //!<error.
}McBSP_TxInterruptSource;

//*****************************************************************************
//
//! Values that can be passed to to McBSP_setTxDataSize() and
//! McBSP_setRxDataSize() as the \e dataFrame parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_PHASE_ONE_FRAME = 0x0000U, //!< Single Phase.
    MCBSP_PHASE_TWO_FRAME = 0x0001U  //!< Dual Phase.
}McBSP_DataPhaseFrame;

//*****************************************************************************
//
//! Values that can be passed as  of McBSP_setTxDataSize()
//! and McBSP_setRxDataSize() as the \e bitsPerWord parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_BITS_PER_WORD_8  = 0x0000U,  //!< 8 bit word.
    MCBSP_BITS_PER_WORD_12 = 0x0020U,  //!< 12 bit word.
    MCBSP_BITS_PER_WORD_16 = 0x0040U,  //!< 16 bit word.
    MCBSP_BITS_PER_WORD_20 = 0x0060U,  //!< 20 bit word.
    MCBSP_BITS_PER_WORD_24 = 0x0080U,  //!< 24 bit word.
    MCBSP_BITS_PER_WORD_32 = 0x00A0U   //!< 32 bit word.
}McBSP_DataBitsPerWord;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setTxCompandingMode() and
//! McBSP_setRxCompandingMode() as the \e compandingMode parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_COMPANDING_NONE = 0x0000U,           //!< Disables companding.
    MCBSP_COMPANDING_NONE_LSB_FIRST = 0x0008U, //!< Disables companding and
                                               //!< Enables 8 bit LSB first
                                               //!< data reception.
    MCBSP_COMPANDING_U_LAW_SET = 0x0010U,      //!< U-law companding.
    MCBSP_COMPANDING_A_LAW_SET = 0x0018U       //!< A-law companding.
} McBSP_CompandingMode;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setTxDataDelayBits()
//! and McBSP_setRxDataDelayBits() as the \e delayBits parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_DATA_DELAY_BIT_0 = 0x0000U, //!< O bit delay.
    MCBSP_DATA_DELAY_BIT_1 = 0x0001U, //!< 1 bit delay.
    MCBSP_DATA_DELAY_BIT_2 = 0x0002U  //!< 2 bit delay.
}McBSP_DataDelayBits;

//*****************************************************************************
//
//! Values that can be passed  for SRG for McBSP_setRxSRGClockSource() as
//! the \e clockSource parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_SRG_RX_CLOCK_SOURCE_LSPCLK = 0x0001U, //!< LSPCLK is SRG clock
                                                //!< source.
    MCBSP_SRG_RX_CLOCK_SOURCE_MCLKX_PIN = 0x0003U //!< MCLKx is SRG clock
                                                  //!< source.
}McBSP_SRGRxClockSource;

//*****************************************************************************
//
//! Values that can be passed for SRG to McBSP_setTxSRGClockSource()
//! as the \e clockSource parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_SRG_TX_CLOCK_SOURCE_LSPCLK = 0x0001U,//!< LSPCLK is SRG clock source.
    MCBSP_SRG_TX_CLOCK_SOURCE_MCLKR_PIN = 0x0002U//!< MCLKris SRG clock source.
}McBSP_SRGTxClockSource;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setTxInternalFrameSyncSource() as the
//! \e syncMode parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_TX_INTERNAL_FRAME_SYNC_DATA = 0x0000U, //!< Data is frame
                                                        //!< sync source.
    MCBSP_TX_INTERNAL_FRAME_SYNC_SRG  = 0x1000U  //!< SRG is frame
                                                        //!< sync source.
}McBSP_TxInternalFrameSyncSource;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setRxMultichannelPartition() and
//! McBSP_setTxMultichannelPartition() as the \e MultichannelPartition
//! parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_MULTICHANNEL_TWO_PARTITION   = 0x0000U,   //!< Two partition.
    MCBSP_MULTICHANNEL_EIGHT_PARTITION = 0x0200U    //!< Eight partition.
}McBSP_MultichannelPartition;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setRxTwoPartitionBlock() and
//! McBSP_setTxTwoPartitionBlock() as the \e block parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_PARTITION_BLOCK_0 = 0x0000U,   //!< Partition block 0.
    MCBSP_PARTITION_BLOCK_1 = 0x0001U,   //!< Partition block 1.
    MCBSP_PARTITION_BLOCK_2 = 0x0002U,   //!< Partition block 2.
    MCBSP_PARTITION_BLOCK_3 = 0x0003U,   //!< Partition block 3.
    MCBSP_PARTITION_BLOCK_4 = 0x0004U,   //!< Partition block 4.
    MCBSP_PARTITION_BLOCK_5 = 0x0005U,   //!< Partition block 5.
    MCBSP_PARTITION_BLOCK_6 = 0x0006U,   //!< Partition block 6.
    MCBSP_PARTITION_BLOCK_7 = 0x0007U    //!< Partition block 7.
}McBSP_PartitionBlock;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setRxChannelMode() as the \e channelMode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_ALL_RX_CHANNELS_ENABLED      = 0x0000U,//!< All Channels are enabled.
    MCBSP_RX_CHANNEL_SELECTION_ENABLED = 0x0001U//!< Selected channels enabled.
}McBSP_RxChannelMode;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setTxChannelMode() as the \e channelMode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_ALL_TX_CHANNELS_ENABLED            = 0x0000U, //!< All Channels
                                                        //!< Enabled.
    MCBSP_TX_CHANNEL_SELECTION_ENABLED       = 0x0001U, //!< Selection Enabled.
    MCBSP_ENABLE_MASKED_TX_CHANNEL_SELECTION = 0x0002U, //!< Masked Tx Channel.
    MCBSP_SYMMERTIC_RX_TX_SELECTION         = 0x0003U //!< Symmetric Selection.
}McBSP_TxChannelMode;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setTxFrameSyncSource() as the \e
//! syncSource parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_TX_EXTERNAL_FRAME_SYNC_SOURCE = 0x0000U, //!< FSR pin supplies
                                                   //!< frame sync signal.
    MCBSP_TX_INTERNAL_FRAME_SYNC_SOURCE = 0x0800U  //!< SRG supplies
                                                   //!< frame sync signal.
}McBSP_TxFrameSyncSource;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setRxFrameSyncSource() as the \e
//! syncSource parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_RX_EXTERNAL_FRAME_SYNC_SOURCE = 0x0000U, //!< FSR pin supplies
                                                   //!< frame sync signal.
    MCBSP_RX_INTERNAL_FRAME_SYNC_SOURCE = 0x0400U  //!< SRG supplies
                                                   //!< frame sync signal.
}McBSP_RxFrameSyncSource;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setTxClockSource() as the Transmitter \e
//! clockSource parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_EXTERNAL_TX_CLOCK_SOURCE = 0x0000U, //!< Clock source is external.
    MCBSP_INTERNAL_TX_CLOCK_SOURCE = 0x0200U  //!< Clock source is internal.
}McBSP_TxClockSource;

//*****************************************************************************
//
//! Values that can be passed  toMcBSP_setRxClockSource() as the Receiver \e
//! clockSource parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_EXTERNAL_RX_CLOCK_SOURCE = 0x0000U, //!< Clock source is external.
    MCBSP_INTERNAL_RX_CLOCK_SOURCE = 0x0100U  //!< Clock source is internal.
}McBSP_RxClockSource;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setTxFrameSyncPolarity() as the
//! Transmitter \e syncPolarity parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_TX_FRAME_SYNC_POLARITY_HIGH = 0x0000U,//!<Pulse active high.
    MCBSP_TX_FRAME_SYNC_POLARITY_LOW = 0x0008U  //!<Pulse active low.
}McBSP_TxFrameSyncPolarity;

//*****************************************************************************
//
//! Values that can be passed to McBSP_setRxFrameSyncPolarity() as the Receiver
//! \e syncPolarity parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_RX_FRAME_SYNC_POLARITY_HIGH = 0x0000U,//!<Pulse active high.
    MCBSP_RX_FRAME_SYNC_POLARITY_LOW = 0x0004U  //!<Pulse active low.
}McBSP_RxFrameSyncPolarity;

//*****************************************************************************
//
//! Values that can be passed for Transmitter of McBSP_setTxClockPolarity()
//! as the Transmiiter \e clockPolarity parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_TX_POLARITY_RISING_EDGE  = 0x0000U, //!< TX data  on rising edge.
    MCBSP_TX_POLARITY_FALLING_EDGE = 0x0002U  //!< TX data  on falling edge.
}McBSP_TxClockPolarity;

//*****************************************************************************
//
//! Values that can be passed for Receiver of McBSP_setRxClockPolarity()
//! as the Receiver \e clockPolarity parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_RX_POLARITY_FALLING_EDGE  = 0x0000U, //!< RX data sampled falling
                                               //!< edge.
    MCBSP_RX_POLARITY_RISING_EDGE =  0x0001U  //!< RX data sampled rising edge.
}McBSP_RxClockPolarity;

//*****************************************************************************
//
//! Values that can be passed to McBSP_getLeftJustifyData() as the
//! \e compandingType parameter.
//
//*****************************************************************************
typedef enum
{
    MCBSP_COMPANDING_U_LAW = 0x0002U, //!< U-law companding.
    MCBSP_COMPANDING_A_LAW = 0x0003U  //!< A-law companding.
} McBSP_CompandingType;

//*****************************************************************************
//
//! Values that can be passed to McBSP_configureRxClock() and
//! McBSP_configureTxClock() as the \e clock parameters.
//
//*****************************************************************************
typedef struct
{
    bool clockSRGSyncFlag;                     //!< SRG - FSR sync flag.
    uint16_t clockSRGDivider;                  //!< Data clock divider.
    McBSP_TxClockSource clockSourceTx;         //!< McBSP TX clock source.
    McBSP_RxClockSource clockSourceRx;         //!< McBSP RX clock source.
    McBSP_SRGRxClockSource clockRxSRGSource;   //!< SRG clock source.
    McBSP_SRGTxClockSource clockTxSRGSource;   //!< SRG clock source.
    McBSP_TxClockPolarity clockMCLKXPolarity;  //!< Clock polarity of MCLKX
                                               //!< pin.
    McBSP_RxClockPolarity clockMCLKRPolarity;  //!< Clock polarity of MCLKR
                                               //!< pin.
}McBSP_ClockParams;

//*****************************************************************************
//
//! Values that can be passed to McBSP_configureTxFrameSync() as the
//! Transmitter \e frameSync parameters.
//
//*****************************************************************************
typedef struct
{
    bool syncSRGSyncFSRFlag;                      //!< Frame sync pulse.
    bool syncErrorDetect;                         //!< Frame sync detect flag.
    uint16_t syncClockDivider;         //!< Clock divider for sync period.
    uint16_t syncPulseDivider;         //!< Clock divider for sync pulse width.
    McBSP_TxFrameSyncSource syncSourceTx;           //!< Frame sync source.
    McBSP_TxInternalFrameSyncSource syncIntSource;//!< Internal Sync source.
    McBSP_TxFrameSyncPolarity syncFSXPolarity;    //!< Frame sync polarity.
}McBSP_TxFsyncParams;

//*****************************************************************************
//
//! Values that can be passed to McBSP_configureTxDataFormat() as the
//! Receiver \e frameSync parameters.
//
//*****************************************************************************
typedef struct
{
    bool syncSRGSyncFSRFlag;                    //!< Frame sync pulse.
    bool syncErrorDetect;                       //!< Frame sync error detect.
    uint16_t syncClockDivider;                  //!< Clock divider sync period.
    uint16_t syncPulseDivider;         //!< Clock divider for sync pulse width.
    McBSP_RxFrameSyncSource syncSourceRx;       //!< Frame sync source.
    McBSP_RxFrameSyncPolarity syncFSRPolarity;  //!< Frame sync polarity.
}McBSP_RxFsyncParams;

//*****************************************************************************
//
//! Values that can be passed to McBSP_configureRxDataFormat() as the
//! Transmitter \e data parameters.
//
//*****************************************************************************
typedef struct
{
    bool loopbackModeFlag;                   //!< Digital Loopback Mode.
    bool twoPhaseModeFlag;                   //!< Two phase transmitter mode.
    bool pinDelayEnableFlag;                 //!< DX Pin delay enable flag.
    uint16_t phase1FrameLength;              //!< Phase 1 frame length.
    uint16_t phase2FrameLength;              //!< Phase 2 frame length.
    McBSP_ClockStopMode clockStopMode;       //!< Clock stop mode.
    McBSP_DataBitsPerWord phase1WordLength;  //!< Phase 1 word length.
    McBSP_DataBitsPerWord phase2WordLength;  //!< Phase 2 word length.
    McBSP_CompandingMode compandingMode;     //!< Data companding mode.
    McBSP_DataDelayBits dataDelayBits;       //!< Data delay in bits.
    McBSP_TxInterruptSource interruptMode;   //!< Transmitter interrupt mode.
}McBSP_TxDataParams;

//*****************************************************************************
//
//! Values that can be passed to McBSP_configureRxDataFormat() as the
//! Receiver \e data parameters.
//
//*****************************************************************************
typedef struct
{
    bool loopbackModeFlag;                   //!< Digital Loopback Mode.
    bool twoPhaseModeFlag;                   //!< Two phase receiver mode.
    uint16_t phase1FrameLength;              //!< Phase 1 frame length.
    uint16_t phase2FrameLength;              //!< Phase 2 frame length.
    McBSP_ClockStopMode clockStopMode;       //!< Clock stop mode.
    McBSP_DataBitsPerWord phase1WordLength;  //!< Phase 1 word length.
    McBSP_DataBitsPerWord phase2WordLength;  //!< Phase 2 word length.
    McBSP_CompandingMode compandingMode;     //!< Data companding mode.
    McBSP_DataDelayBits dataDelayBits;       //!< Data delay in bits.
    McBSP_RxSignExtensionMode signExtMode;   //!< Sign extension mode.
    McBSP_RxInterruptSource interruptMode;   //!< Transmitter interrupt mode.
}McBSP_RxDataParams;

//*****************************************************************************
//
//! Values that can be passed to McBSP_configureRxMultichannel() as the
//! Receiver \e multichannel parameters.
//
//*****************************************************************************
typedef struct
{
    uint16_t channelCountRx;             //!< Number of channels to be enabled.
    uint16_t *ptrChannelsListRx;         //!< Pointer to array that has list of
                                         //!< channels.
    McBSP_RxChannelMode multichannelModeRx;   //!< Multichannel modes.
    McBSP_MultichannelPartition partitionRx;  //!< Multichannel partition.
}McBSP_RxMultichannelParams;

//*****************************************************************************
//
//! Values that can be passed to McBSP_configureTxMultichannel() as the
//! Transmitter \e multichannel parameters.
//
//*****************************************************************************
typedef struct
{
    uint16_t channelCountTx;     //!< Number of channels to be enabled.
    uint16_t *ptrChannelsListTx; //!< Pointer to array that has list of
                                 //!< channels.
    McBSP_TxChannelMode multichannelModeTx;   //!< Multichannel modes.
    McBSP_MultichannelPartition partitionTx;  //!< Multichannel partition.
}McBSP_TxMultichannelParams;

//*****************************************************************************
//
//! Values that can be passed to McBSP_configureSPIMasterMode() as the \e
//! SPIMasterMode parameters.
//
//*****************************************************************************
typedef struct
{
    bool loopbackModeFlag;             //!< Digital Loopback Mode .
    uint32_t clockSRGDivider;          //!< Clock divider.
    McBSP_ClockStopMode clockStopMode; //!< Clock stop mode.
    McBSP_DataBitsPerWord wordLength;  //!< Word length.
    McBSP_TxClockPolarity spiMode;     //!< Master out clock polarity.
}
McBSP_SPIMasterModeParams;

//*****************************************************************************
//
//! Values that can be passed to McBSP_configureSPISlaveMode() as the \e
//! SPISlaveMode parameters.
//
//*****************************************************************************
typedef struct
{
    bool loopbackModeFlag;             //!< Digital Loopback Mode.
    McBSP_ClockStopMode clockStopMode; //!< Clock stop mode.
    McBSP_DataBitsPerWord wordLength;  //!< Word length.
    McBSP_TxClockPolarity spiMode;     //!< Clock polarity.
}
McBSP_SPISlaveModeParams;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! \internal
//! Checks McBSP base address.
//!
//! \param base specifies the McBSP module base address.
//!
//! This function determines if a McBSP module base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool
McBSP_isBaseValid(uint32_t base)
{
    return(
           (base == MCBSPA_BASE) ||
           (base == MCBSPB_BASE)
          );
}
#endif

//*****************************************************************************
//
//! Disables digital loop back mode.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function disables digital loop back mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_disableLoopback(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear DLB bit.
    //
    HWREGH(base + MCBSP_O_SPCR1) &= ~MCBSP_SPCR1_DLB;
}

//*****************************************************************************
//
//! Enables digital loop back mode.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function enables digital loop back mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_enableLoopback(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set DLB bit.
    //
    HWREGH(base + MCBSP_O_SPCR1) |= MCBSP_SPCR1_DLB;
}

//*****************************************************************************
//
//! Configures receiver sign extension mode.
//!
//! \param base is the base address of the McBSP module.
//! \param mode is the sign extension mode.
//!
//! This function sets the sign extension mode. Valid values for mode are:
//!  - \b MCBSP_RIGHT_JUSTIFY_FILL_ZERO - right justified MSB filled with zero.
//!  - \b MCBSP_RIGHT_JUSTIFY_FILL_SIGN - right justified sign extended in
//!                                       MSBs.
//!  - \b MCBSP_LEFT_JUSTIFY_FILL_ZER0 - left justifies LBS filled with zero.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setRxSignExtension(uint32_t base, const McBSP_RxSignExtensionMode mode)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Write to RJUST bits.
    //
    HWREGH(base + MCBSP_O_SPCR1) =
    ((HWREGH(base + MCBSP_O_SPCR1) & ~MCBSP_SPCR1_RJUST_M ) | (uint16_t)mode);
}

//*****************************************************************************
//
//! Configures clock stop mode.
//!
//! \param base is the base address of the McBSP module.
//! \param mode is the clock stop mode.
//!
//! This function sets the cock stop mode. Valid values for mode are
//!  - \b MCBSP_CLOCK_MCBSP_MODE disables clock stop mode.
//!  - \b MCBSP_CLOCK_SPI_MODE_NO_DELAY enables clock stop mode
//!  - \b MCBSP_CLOCK_SPI_MODE_DELAY enables clock stop mode with delay.
//!
//!  If an invalid value is provided, the function will exit with out altering
//!  the register bits involved.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setClockStopMode(uint32_t base, const McBSP_ClockStopMode mode)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Write to CLKSTP bits.
    //
    HWREGH(base + MCBSP_O_SPCR1) =
    ((HWREGH(base + MCBSP_O_SPCR1) & ~MCBSP_SPCR1_CLKSTP_M) | (uint16_t)mode);
}

//*****************************************************************************
//
//! Disables delay at DX pin.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function disables delay on pin DX when turning the module on.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_disableDxPinDelay(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set DXENA bit
    //
    HWREGH(base + MCBSP_O_SPCR1) &= ~MCBSP_SPCR1_DXENA;
}

//*****************************************************************************
//
//! Enables delay at DX pin.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function enables a delay on pin DX when turning the module on. Look at
//! McBSP timing diagrams for details.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_enableDxPinDelay(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set DXENA bit.
    //
    HWREGH(base + MCBSP_O_SPCR1) |= MCBSP_SPCR1_DXENA;
}

//*****************************************************************************
//
//! Configures receiver interrupt sources.
//!
//! \param base is the base address of the McBSP module.
//! \param interruptSource is the ISR source.
//!
//! This function sets the receiver interrupt sources.
//! Valid values for interruptSource are:
//!  - \b MCBSP_RX_ISR_SOURCE_SERIAL_WORD   - interrupt at each serial word.
//!  - \b MCBSP_RX_ISR_SOURCE_END_OF_BLOCK  - interrupt at the end of block.
//!  - \b MCBSP_RX_ISR_SOURCE_FRAME_SYNC    - interrupt when frame sync occurs.
//!  - \b MCBSP_RX_ISR_SOURCE_SYNC_ERROR    - interrupt on frame sync error.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setRxInterruptSource(uint32_t base,
                           const McBSP_RxInterruptSource interruptSource)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Write to RINTM bits.
    //
    HWREGH(base + MCBSP_O_SPCR1) =
    ((HWREGH(base + MCBSP_O_SPCR1) & ~MCBSP_SPCR1_RINTM_M) |
     (uint16_t)interruptSource);
}

//*****************************************************************************
//
//! Clear the receiver frame sync error.
//!
//! \param base is the base address of the McBSP module.
//!
//!  This function clears the receive frame sync error.
//!
//! \return  None.
//
//*****************************************************************************
static inline void
McBSP_clearRxFrameSyncError(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear RSYNCERR bit.
    //
    HWREGH(base + MCBSP_O_SPCR1) &= ~MCBSP_SPCR1_RSYNCERR;
}

//*****************************************************************************
//
//! Return receiver error.
//!
//! \param base is the base address of the McBSP module.
//!
//!  This function returns McBSP receiver errors.
//!
//! \return  Returns the following error codes.
//!  - \b MCBSP_RX_NO_ERROR - if there is no error.
//!  - \b MCBSP_RX_BUFFER_ERROR - if buffergets full.
//!  - \b MCBSP_RX_FRAME_SYNC_ERROR -if unexpected frame sync occurs.
//!  - \b MCBSP_RX_BUFFER_FRAME_SYNC_ERROR - if buffer overrun and frame sync
//!       error occurs.
//
//*****************************************************************************
static inline uint16_t
McBSP_getRxErrorStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Return the value of RFULL & RSYNCERR bit.
    //
    return(HWREGH(base + MCBSP_O_SPCR1) & (MCBSP_SPCR1_RFULL |
                                           MCBSP_SPCR1_RSYNCERR));
}

//*****************************************************************************
//
//! Check if data is received by the receiver.
//!
//! \param base is the base address of the McBSP port.
//!
//! This function returns the status of the receiver buffer , indicating if
//! new data is available.
//!
//! \return \b true if new data is available or if the current data was never
//!            read.
//!         \b false if there is no new data in the receive buffer.
//
//*****************************************************************************
static inline bool
McBSP_isRxReady(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Check RRDY bit.
    //
    return((HWREGH(base + MCBSP_O_SPCR1) & MCBSP_SPCR1_RRDY)
           == MCBSP_SPCR1_RRDY);
}

//*****************************************************************************
//
//! Reset McBSP receiver.
//!
//! \param base is the base address of the McBSP module.
//!
//!  This function resets McBSP receiver.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_resetReceiver(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear RRST bit.
    //
    HWREGH(base + MCBSP_O_SPCR1) &= ~ MCBSP_SPCR1_RRST;
}

//*****************************************************************************
//
//! Enable McBSP receiver.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function enables McBSP receiver.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_enableReceiver(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set RRST bit.
    //
    HWREGH(base + MCBSP_O_SPCR1) |= MCBSP_SPCR1_RRST;
}

//*****************************************************************************
//
//! Configures emulation mode.
//!
//! \param base is the base address of the McBSP module.
//! \param emulationMode is the McBSP emulation character.
//!
//! This function sets the McBSP characters when a breakpoint is encountered
//! in emulation mode. Valid values for emulationMode are:
//! - \b MCBSP_EMULATION_IMMEDIATE_STOP - transmitter and receiver both stop
//!                                        when a breakpoint is reached.
//! - \b MCBSP_EMULATION_SOFT_STOP      - transmitter stops after current
//!                                       word is transmitted. Receiver is not
//!                                       affected.
//! - \b MCBSP_EMULATION_FREE_RUN       - McBSP runs ignoring the breakpoint.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setEmulationMode(uint32_t base, const McBSP_EmulationMode emulationMode)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // write to FREE and SOFT bits.
    //
    HWREGH(base + MCBSP_O_SPCR2) =
    ((HWREGH(base + MCBSP_O_SPCR2) & ~(MCBSP_SPCR2_FREE | MCBSP_SPCR2_SOFT))
     | (uint16_t)emulationMode);
}

//*****************************************************************************
//
//! Reset frame sync logic.
//!
//! \param base is the base address of the McBSP module.
//!
//!  Resets frame sync logic.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_resetFrameSyncLogic(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear FRST bit.
    //
    HWREGH(base + MCBSP_O_SPCR2) &= ~MCBSP_SPCR2_FRST;
}

//*****************************************************************************
//
//! Enable frame sync logic.
//!
//! \param base is the base address of the McBSP module.
//!
//!  Enables frame sync logic.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_enableFrameSyncLogic(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set FRST bit.
    //
    HWREGH(base + MCBSP_O_SPCR2) |= MCBSP_SPCR2_FRST;
}

//*****************************************************************************
//
//! Reset sample rate generator.
//!
//! \param base is the base address of the McBSP module.
//!
//!  Resets sample rate generator by clearing GRST bit.
//!
//! \return
//
//*****************************************************************************
static inline void
McBSP_resetSampleRateGenerator(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear GRST bit.
    //
    HWREGH(base + MCBSP_O_SPCR2) &= ~MCBSP_SPCR2_GRST;
}

//*****************************************************************************
//
//! Enable sample rate generator.
//!
//! \param base is the base address of the McBSP module.
//!
//!  Enables sample rate generator by setting GRST bit.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_enableSampleRateGenerator(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set GRST bit.
    //
    HWREGH(base + MCBSP_O_SPCR2) |= MCBSP_SPCR2_GRST;
}

//*****************************************************************************
//
//! Configures transmitter interrupt sources.
//!
//! \param base is the base address of the McBSP module.
//! \param interruptSource is the ISR source.
//!
//! This function sets the transmitter interrupt sources.
//! Valid values for interruptSource are:
//!  - \b MCBSP_TX_ISR_SOURCE_TX_READY   - interrupt when transmitter is ready
//!                                        to accept data.
//!  - \b MCBSP_TX_ISR_SOURCE_END_OF_BLOCK  - interrupt at the end of block.
//!  - \b MCBSP_TX_ISR_SOURCE_FRAME_SYNC    - interrupt when frame sync occurs.
//!  - \b MCBSP_TX_ISR_SOURCE_SYNC_ERROR    - interrupt on frame sync error.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setTxInterruptSource(uint32_t base,
                           const McBSP_TxInterruptSource interruptSource)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Write to XINTM bits.
    //
    HWREGH(base + MCBSP_O_SPCR2) =
    ((HWREGH(base + MCBSP_O_SPCR2) & ~MCBSP_SPCR2_XINTM_M) |
     (uint16_t)interruptSource);
}

//*****************************************************************************
//
//! Return Transmitter error.
//!
//! \param base is the base address of the McBSP module.
//!
//!  This function returns McBSP transmitter errors.
//!
//! \return  Returns the following error codes.
//!  - \b MCBSP_TX_NO_ERROR - if buffer overrun occurs.
//!  - \b MCBSP_TX_BUFFER_ERROR -if unexpected frame sync occurs.
//!  - \b MCBSP_TX_FRAME_SYNC_ERROR - if there is no error.
//!  - \b MCBSP_TX_BUFFER_FRAME_SYNC_ERROR - if buffer overrun and frame sync
//!       error occurs.
//
//*****************************************************************************
static inline uint16_t
McBSP_getTxErrorStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Return the value of XEMPTY & XSYNCERR bit.
    //
    return(HWREGH(base + MCBSP_O_SPCR2) & (MCBSP_SPCR2_XEMPTY |
                                           MCBSP_SPCR2_XSYNCERR));
}

//*****************************************************************************
//
//! Clear the Transmitter frame sync error.
//!
//! \param base is the base address of the McBSP module.
//!
//!  This function clears the transmitter frame sync error.
//!
//! \return  None.
//
//*****************************************************************************
static inline void
McBSP_clearTxFrameSyncError(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear XSYNCERR bit.
    //
    HWREGH(base + MCBSP_O_SPCR2) &= ~MCBSP_SPCR2_XSYNCERR;
}

//*****************************************************************************
//
//! Check if Transmitter is ready.
//!
//! \param base is the base address of the McBSP port.
//!
//! This function returns the status of the transmitter ready buffer,
//! indicating if data can be written to the transmitter.
//!
//! \return \b true if transmitter is ready to accept new data.
//!         \b false if transmitter is not ready to accept new data.
//
//*****************************************************************************
static inline bool
McBSP_isTxReady(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Check XRDY bit.
    //
    return((HWREGH(base + MCBSP_O_SPCR2) & MCBSP_SPCR2_XRDY)
           == MCBSP_SPCR2_XRDY);
}

//*****************************************************************************
//
//! Reset McBSP transmitter.
//!
//! \param base is the base address of the McBSP module.
//!
//! This functions resets McBSP transmitter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_resetTransmitter(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear XRST bit.
    //
    HWREGH(base + MCBSP_O_SPCR2) &= ~MCBSP_SPCR2_XRST;
}

//*****************************************************************************
//
//! Enable McBSP transmitter.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function enables McBSP transmitter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_enableTransmitter(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set XRST bit.
    //
    HWREGH(base + MCBSP_O_SPCR2) |= MCBSP_SPCR2_XRST;
}

//*****************************************************************************
//
//! Disable 2 Phase operation for data reception.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function disables 2 phase reception.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_disableTwoPhaseRx(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear RPHASE bit.
    //
    HWREGH(base + MCBSP_O_RCR2) &= ~MCBSP_RCR2_RPHASE;
}

//*****************************************************************************
//
//! Enable 2 Phase operation for data Reception.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function enables 2 phase reception.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_enableTwoPhaseRx(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set RPHASE bit.
    //
    HWREGH(base + MCBSP_O_RCR2) |= MCBSP_RCR2_RPHASE;
}

//*****************************************************************************
//
//! Configure receive data companding.
//!
//! \param base is the base address of the McBSP module.
//! \param compandingMode is the companding mode to be used.
//!
//! This function configures the receive companding logic. The following are
//! valid compandingMode values:
//!
//! - \b MCBSP_COMPANDING_NONE disables companding.
//! - \b MCBSP_COMPANDING_NONE_LSB_FIRST disables companding and enables 8 bit
//!      LSB first data reception.
//! - \b MCBSP_COMPANDING_U_LAW_SET  enables U-law companding.
//! - \b MCBSP_COMPANDING_A_LAW_SET  enables A-law companding.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setRxCompandingMode(uint32_t base,
                          const McBSP_CompandingMode compandingMode)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Write to RCOMPAND bits.
    //
    HWREGH(base + MCBSP_O_RCR2) =
    ((HWREGH(base + MCBSP_O_RCR2) & ~MCBSP_RCR2_RCOMPAND_M) |
     (uint16_t)compandingMode);
}

//*****************************************************************************
//
//! Disables receiver unexpected frame sync error detection.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function disables unexpected frame sync error detection in the
//! receiver.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_disableRxFrameSyncErrorDetection(uint32_t base)
{
    //
    // Check the arguments.
    //

    ASSERT(McBSP_isBaseValid(base));

    //
    // Set RFIG bit.
    //
    HWREGH(base + MCBSP_O_RCR2) |= MCBSP_RCR2_RFIG;
}

//*****************************************************************************
//
//! Enable receiver unexpected frame sync error detection.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function enables unexpected frame sync error detection in the
//! receiver.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_enableRxFrameSyncErrorDetection(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear RFIG bit.
    //
    HWREGH(base + MCBSP_O_RCR2) &= ~MCBSP_RCR2_RFIG;
}

//*****************************************************************************
//
//! Sets the receive bit data delay.
//!
//! \param base is the base address of the McBSP module.
//! \param delayBits is the number of bits to delay.
//!
//! This functions sets the bit delay after the frame sync pulse as specified
//! by delayBits. Valid delay bits are \b MCBSP_DATA_DELAY_BIT_0,
//! \b MCBSP_DATA_DELAY_BIT_1 or \b MCBSP_DATA_DELAY_BIT_2 corresponding to
//! 0, 1 or 2 bit delay respectively.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setRxDataDelayBits(uint32_t base, const McBSP_DataDelayBits delayBits)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Write to RDATDLY bits.
    //
    HWREGH(base + MCBSP_O_RCR2) =
    ((HWREGH(base + MCBSP_O_RCR2) & ~MCBSP_RCR2_RDATDLY_M) |
     (uint16_t)delayBits);
}

//*****************************************************************************
//
//! Disable 2 Phase operation for data Transmission.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function disables 2 phase transmission.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_disableTwoPhaseTx(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear XPHASE bit.
    //
    HWREGH(base + MCBSP_O_XCR2) &= ~MCBSP_XCR2_XPHASE;
}

//*****************************************************************************
//
//! Enable 2 Phase operation for data Transmission.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function enables 2 phase transmission.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_enableTwoPhaseTx(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set XPHASE bit
    //
    HWREGH(base + MCBSP_O_XCR2) |= MCBSP_XCR2_XPHASE;
}

//*****************************************************************************
//
//! Configure transmit data companding.
//!
//! \param base is the base address of the McBSP module.
//! \param compandingMode is the companding mode to be used.
//!
//! This function configures the transmit companding logic. The following are
//! valid compandingMode values:
//!
//! - \b MCBSP_COMPANDING_NONE disables companding.
//! - \b MCBSP_COMPANDING_NONE_LSB_FIRST disables companding and enables 8 bit
//!      LSB first data reception.
//! - \b MCBSP_COMPANDING_U_LAW_SET  enables U-law companding.
//! - \b MCBSP_COMPANDING_A_LAW_SET  enables A-law companding.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setTxCompandingMode(uint32_t base,
                          const McBSP_CompandingMode compandingMode)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Write to XCOMPAND bits.
    //
    HWREGH(base + MCBSP_O_XCR2) =
    ((HWREGH(base + MCBSP_O_XCR2) & ~MCBSP_XCR2_XCOMPAND_M) |
     (uint16_t)compandingMode);
}

//*****************************************************************************
//
//! Disables transmitter unexpected frame sync error detection.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function disables unexpected frame sync error detection in the
//! transmitter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_disableTxFrameSyncErrorDetection(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set XFIG bit.
    //
    HWREGH(base + MCBSP_O_XCR2) |= MCBSP_XCR2_XFIG;
}

//*****************************************************************************
//
//! Enable transmitter unexpected frame sync error detection.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function enables unexpected frame sync error detection in the
//! transmitter.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_enableTxFrameSyncErrorDetection(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear XFIG bit.
    //
    HWREGH(base + MCBSP_O_XCR2) &= ~MCBSP_XCR2_XFIG;
}

//*****************************************************************************
//
//! Sets the transmit bit delay.
//!
//! \param base is the base address of the McBSP module.
//! \param delayBits is the number of bits to delay.
//!
//! This function sets the bit delay after the frame sync pulse as specified
//! by delayBits. Valid delay bits are \b MCBSP_DATA_DELAY_BIT_0,
//! \b MCBSP_DATA_DELAY_BIT_1 or \b MCBSP_DATA_DELAY_BIT_2 corresponding to
//! 0, 1 or 2 bit delay respectively.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setTxDataDelayBits(uint32_t base, const McBSP_DataDelayBits delayBits)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Write to XDATDLY bits.
    //
    HWREGH(base + MCBSP_O_XCR2) =
    ((HWREGH(base + MCBSP_O_XCR2) & ~MCBSP_XCR2_XDATDLY_M) |
     (uint16_t)delayBits);
}

//*****************************************************************************
//
//! Sets the period for frame synchronisation pulse.
//!
//! \param base is the base address of the McBSP module.
//! \param frameClockDivider is the divider count for the sync clock.
//!
//!  This function sets the sample rate generator clock divider for the McBSP
//!  frame sync clock(FSG).
//!  FSG = CLKG / (frameClockDivider + 1).
//!  frameClockDivider determines the period count.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setFrameSyncPulsePeriod(uint32_t base, uint16_t frameClockDivider)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));
    ASSERT(frameClockDivider < 4096U);

    //
    // Assign value to FPER to set the pulse width.
    //
    HWREGH(base + MCBSP_O_SRGR2) =
    ((HWREGH(base + MCBSP_O_SRGR2) & ~MCBSP_SRGR2_FPER_M) | frameClockDivider);
}

//*****************************************************************************
//
//! Sets the frame sync pulse width divider value.
//!
//! \param base is the base address of the McBSP module.
//! \param pulseWidthDivider is the divider count for sync clock pulse.
//!
//! This function sets the pulse width divider bits for the McBSP frame sync
//! clock(FSG).
//! (pulseWidthDivider + 1) is the pulse width in CLKG cycles.
//! pulseWidthDivider determines the pulse width (the on count).
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setFrameSyncPulseWidthDivider(uint32_t base, uint16_t pulseWidthDivider)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));
    ASSERT(pulseWidthDivider < 256U);

    //
    // Assign value to FWID to set sync clock rate.
    //
    HWREGH(base + MCBSP_O_SRGR1) =
    ((HWREGH(base + MCBSP_O_SRGR1) & ~MCBSP_SRGR1_FWID_M) |
     (uint16_t)(pulseWidthDivider << MCBSP_SRGR1_FWID_S));
}

//*****************************************************************************
//
//! Sets the data clock divider values.
//!
//! \param base is the base address of the McBSP module.
//! \param dataClockDivider is the divider count for the data rate.
//!
//!  This function sets the sample rate generator clock divider for the McBSP
//!  data clock(CLKG).
//!  CLKG = CLKSRG / (clockDivider + 1).
//!  Valid ranges for clockDivider are 0 to 0xFF.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setSRGDataClockDivider(uint32_t base, uint16_t dataClockDivider)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));
    ASSERT(dataClockDivider < 256U);

    //
    // Assign value to CLKGDV to set data clock rate.
    //
    HWREGH(base + MCBSP_O_SRGR1) =
    ((HWREGH(base + MCBSP_O_SRGR1) & ~MCBSP_SRGR1_CLKGDV_M) |
     dataClockDivider);
}

//*****************************************************************************
//
//! Disables external clock sync with sample generator.
//!
//! \param base is the base address of the McBSP module.
//!
//!  This function disables CLKG and FSG sync with the external pulse
//!  on pin FSR.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_disableSRGSyncFSR(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear GSYNC  bit.
    //
    HWREGH(base + MCBSP_O_SRGR2) &= ~MCBSP_SRGR2_GSYNC;
}

//*****************************************************************************
//
//! Enables external clock to synch with sample generator.
//!
//! \param base is the base address of the McBSP module.
//!
//!  This function enables CLKG and FSG to sync with the external pulse
//!  on pin FSR.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_enableSRGSyncFSR(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set GSYNC bit.
    //
    HWREGH(base + MCBSP_O_SRGR2) |= MCBSP_SRGR2_GSYNC;
}

//*****************************************************************************
//
//! Configures receiver input clock source for sample generator.
//!
//! \param base is the base address of the McBSP module.
//! \param srgClockSource is clock source for the sample generator.
//!
//! This functions sets the clock source for the sample rate generator.
//! Valid values for \e clockSource are
//!  - \b MCBSP_SRG_RX_CLOCK_SOURCE_LSPCLK for LSPCLK.
//!  - \b MCBSP_SRG_RX_CLOCK_SOURCE_MCLKX_PIN for external clock at MCLKX pin.
//! MCLKR pin will be an output driven by sample rate generator.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setRxSRGClockSource(uint32_t base,
                          const McBSP_SRGRxClockSource srgClockSource)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set or clear CLKSM bit.
    //
    HWREGH(base + MCBSP_O_SRGR2) =
    ((HWREGH(base + MCBSP_O_SRGR2) & ~MCBSP_SRGR2_CLKSM) |
     ((uint16_t)((uint16_t)srgClockSource & 0x1U) << 13U));

    //
    // Set or clear SCLKME bit.
    //
    HWREGH(base + MCBSP_O_PCR) =
    ((HWREGH(base + MCBSP_O_PCR) & ~MCBSP_PCR_SCLKME) |
     (uint16_t)(((uint16_t)srgClockSource >> 1U) << 7U));
}

//*****************************************************************************
//
//! Configures transmitter input clock source for sample generator.
//!
//! \param base is the base address of the McBSP module.
//! \param srgClockSource is clock source for the sample generator.
//!
//! This functions sets the clock source for the sample rate generator.
//! Valid values for \e clockSource are
//!  - \b MCBSP_SRG_TX_CLOCK_SOURCE_LSPCLK for LSPCLK.
//!  - \b MCBSP_SRG_TX_CLOCK_SOURCE_MCLKR_PIN for external clock at MCLKR pin.
//! MCLKX pin will be an output driven by sample rate generator.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setTxSRGClockSource(uint32_t base,
                          const McBSP_SRGTxClockSource srgClockSource)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set or clear CLKSM bit.
    //
    HWREGH(base + MCBSP_O_SRGR2) =
    ((HWREGH(base + MCBSP_O_SRGR2) & ~MCBSP_SRGR2_CLKSM) |
     ((uint16_t)((uint16_t)srgClockSource & 0x1U) << 13U));
    //
    // Set or clear SCLKME bit.
    //
    HWREGH(base + MCBSP_O_PCR) =
    ((HWREGH(base + MCBSP_O_PCR) & ~MCBSP_PCR_SCLKME) |
     (uint16_t)(((uint16_t)srgClockSource >> 1U) << 7U));
}

//*****************************************************************************
//
//! Sets the mode for transmitter internal frame sync signal.
//!
//! \param base is the base address of the McBSP module.
//! \param syncMode is the frame sync mode.
//!
//! This function sets the frame sync signal generation mode. The signal can be
//! generated based on clock divider as set in McBSP_setFrameSyncPulsePeriod()
//! function or when data is transferred from DXR registers to XSR registers.
//! Valid input for syncMode are:
//!
//! - \b MCBSP_TX_INTERNAL_FRAME_SYNC_DATA - frame sync signal is
//!                                   generated when data is transferred from
//!                                   DXR registers to XSR registers.
//! - \b MCBSP_TX_INTERNAL_FRAME_SYNC_SRG - frame sync signal is
//!                                   generated based on the clock counter
//!                                   value as defined in
//!                                   McBSP_setFrameSyncPulsePeriod()
//!                                   function.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setTxInternalFrameSyncSource(uint32_t base,
                                const McBSP_TxInternalFrameSyncSource syncMode)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set or clear FSGM bit.
    //
    HWREGH(base + MCBSP_O_SRGR2) =
    ((HWREGH(base + MCBSP_O_SRGR2) & ~MCBSP_SRGR2_FSGM) | (uint16_t)syncMode);
}

//*****************************************************************************
//
//! Set Multichannel receiver partitions.
//!
//! \param base is the base address of the McBSP module.
//! \param partition is the number of partitions.
//!
//!  This function sets the partitions for Multichannel receiver. Valid values
//!  for partition are \b MCBSP_MULTICHANNEL_TWO_PARTITION or \b
//!  MCBSP_MULTICHANNEL_EIGHT_PARTITION for 2 and 8 partitions respectively.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setRxMultichannelPartition(uint32_t base,
                                 const McBSP_MultichannelPartition partition)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set or Clear RMCME bit.
    //
    HWREGH(base + MCBSP_O_MCR1) =
    ((HWREGH(base + MCBSP_O_MCR1) & ~MCBSP_MCR1_RMCME) | (uint16_t)partition);
}

//*****************************************************************************
//
//! Sets block to receiver in two partition configuration.
//!
//! \param base is the base address of the McBSP module.
//! \param block is the block to assign to the partition.
//!
//!  This function assigns the block the user provides to the appropriate
//!  receiver partition.
//!  If user sets the value of block to 0,2,4 or 6 the API will assign the
//!  blocks to partition A. If values 1,3,5,or 7 are set to block, then
//!  the API assigns the block to partition B.
//!
//! \note This function should be used with the two partition configuration
//!       only and not with eight partition configuration.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setRxTwoPartitionBlock(uint32_t base, const McBSP_PartitionBlock block)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    //Check the block value if it is 0,2,4,6 or 1,3,5,7.
    //
    if(((uint16_t)block ==  0U) ||
       ((uint16_t)block ==  2U) ||
       ((uint16_t)block ==  4U) ||
       ((uint16_t)block ==  6U))
    {
        //
        // write to RPABLK bits.
        //
        HWREGH(base + MCBSP_O_MCR1)  =
        ((HWREGH(base + MCBSP_O_MCR1) & ~MCBSP_MCR1_RPABLK_M) |
         (uint16_t)(((uint16_t)block >> 1U)<< 5U));
    }
    else
    {
        //
        // write to RPBBLK bits.
        //
        HWREGH(base + MCBSP_O_MCR1)  =
        ((HWREGH(base + MCBSP_O_MCR1) & ~MCBSP_MCR1_RPBBLK_M) |
         (uint16_t)(((uint16_t)block >> 1U)<< 7U));
    }
}

//*****************************************************************************
//
//! Returns the current active receiver block number.
//!
//! \param base is the base address of the McBSP module.
//!
//!  This function returns the current active receiver block involved in McBSP
//!  reception.
//!
//! \return Active block in McBSP reception. Returned values range from 0 to 7
//!  representing the respective active block number .
//
//*****************************************************************************
static inline uint16_t
McBSP_getRxActiveBlock(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // return RCBLK bits.
    //
    return((HWREGH(base + MCBSP_O_MCR1) & MCBSP_MCR1_RCBLK_M) >>
           MCBSP_MCR1_RCBLK_S);
}

//*****************************************************************************
//
//! Configure channel selection mode for receiver.
//!
//! \param base is the base address of the McBSP module.
//! \param channelMode is the channel selection mode.
//!
//!  This function configures the channel selection mode. The following are
//!  valid values for channelMode:
//!
//!  - \b MCBSP_ALL_RX_CHANNELS_ENABLED - enables all channels.
//!  - \b MCBSP_RX_CHANNEL_SELECTION_ENABLED - lets the user enable desired
//!       channels by using McBSP_enableRxChannel().
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setRxChannelMode(uint32_t base, const McBSP_RxChannelMode channelMode)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set or clear RMCM bit.
    //
    HWREGH(base + MCBSP_O_MCR1) =
    ((HWREGH(base + MCBSP_O_MCR1) & ~MCBSP_MCR1_RMCM) | (uint16_t)channelMode);
}

//*****************************************************************************
//
//! Set Multichannel transmitter partitions.
//!
//! \param base is the base address of the McBSP module.
//! \param partition is the number of partitions.
//!
//!  This function sets the partitions for Multichannel transmitter. Valid
//!  values for partition are \b MCBSP_MULTICHANNEL_TWO_PARTITION or \b
//!  MCBSP_MULTICHANNEL_EIGHT_PARTITION for 2 and 8 partitions respectively.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setTxMultichannelPartition(uint32_t base,
                                 const McBSP_MultichannelPartition partition)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set or clear XMCME bit.
    //
    HWREGH(base + MCBSP_O_MCR2) =
    ((HWREGH(base + MCBSP_O_MCR2) & ~MCBSP_MCR2_XMCME) | (uint16_t)partition);
}

//*****************************************************************************
//
//! Sets block to transmitter in two partition configuration.
//!
//! \param base is the base address of the McBSP module.
//! \param block is the block to assign to the partition.
//!
//!  This function assigns the block the user provides to the appropriate
//!  transmitter partition.
//!  If user sets the value of block to 0,2,4 or 6 the API will assign the
//!  blocks to partition A. If values 1,3,5,or 7 are set to block, then
//!  the API assigns the block to partition B.
//!
//! \note This function should be used with the two partition configuration
//!       only and not with eight partition configuration.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setTxTwoPartitionBlock(uint32_t base, const McBSP_PartitionBlock block)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    if(((uint16_t)block ==  0U) ||
       ((uint16_t)block ==  2U) ||
       ((uint16_t)block ==  4U) ||
       ((uint16_t)block ==  6U))
    {
        //
        // write to XPABLK bits.
        //
        HWREGH(base + MCBSP_O_MCR2)  =
        ((HWREGH(base + MCBSP_O_MCR2) & ~MCBSP_MCR2_XPABLK_M) |
         ((uint16_t)((uint16_t)block >> 1U)<< 5U));
    }
    else
    {
        //
        // write to XPBBLK bits.
        //
        HWREGH(base + MCBSP_O_MCR2)  =
        ((HWREGH(base + MCBSP_O_MCR2) & ~MCBSP_MCR2_XPBBLK_M) |
         ((uint16_t)((uint16_t)block >> 1U)<< 7U));
    }
}

//*****************************************************************************
//
//! Returns the current active transmitter block number.
//!
//! \param base is the base address of the McBSP module.
//!
//!  This function returns the current active transmitter block involved in
//!  McBSP transmission.
//!
//! \return Active block in McBSP transmission. Returned values range from
//!  0 to 7 representing the respective active block number.
//
//*****************************************************************************
static inline uint16_t
McBSP_getTxActiveBlock(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // return XCBLK bits.
    //
    return((HWREGH(base + MCBSP_O_MCR2) & MCBSP_MCR2_XCBLK_M) >>
           MCBSP_MCR2_XCBLK_S);

}

//*****************************************************************************
//
//! Configure channel selection mode for transmitter.
//!
//! \param base is the base address of the McBSP module.
//! \param channelMode is the channel selection mode.
//!
//!  This function configures the channel selection mode. The following are
//!  valid values for channelMode:
//!
//!  - \b MCBSP_ALL_TX_CHANNELS_ENABLED - enables and unmasks all channels
//!  - \b MCBSP_TX_CHANNEL_SELECTION_ENABLED - lets the user enable and unmask
//!                           desired channels by using McBSP_enableTxChannel()
//!  - \b MCBSP_ENABLE_MASKED_TX_CHANNEL_SELECTION - All channels enables but
//!                             until enabled by McBSP_enableTxChannel()
//!  - \b MCBSP_SYMMERTIC_RX_TX_SELECTION - Symmetric transmission and
//!                                        reception.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setTxChannelMode(uint32_t base, const McBSP_TxChannelMode channelMode)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set values to the XMCM bits.
    //
    HWREGH(base + MCBSP_O_MCR2) =
    ((HWREGH(base + MCBSP_O_MCR2) & ~MCBSP_MCR2_XMCM_M) |
     (uint16_t)channelMode);
}

//*****************************************************************************
//
//! Select the transmitter frame sync signal source.
//!
//! \param base is the base address of the McBSP module.
//! \param syncSource is the transmitter frame sync source.
//!
//! This function sets external or internal sync signal source based on the
//! syncSource selection. Valid input for syncSource are:
//!
//! - \b MCBSP_TX_EXTERNAL_FRAME_SYNC_SOURCE - frame sync signal is supplied
//!                                            externally by pin FSX.
//! - \b MCBSP_TX_INTERNAL_FRAME_SYNC_SOURCE - frame sync signal is supplied
//!                                             internally.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setTxFrameSyncSource(uint32_t base,
                           const McBSP_TxFrameSyncSource syncSource)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    //Set or Clear the FSXM bit.
    //
    HWREGH(base + MCBSP_O_PCR) =
    ((HWREGH(base + MCBSP_O_PCR) & ~MCBSP_PCR_FSXM) | (uint16_t)syncSource);
}

//*****************************************************************************
//
//! Select receiver frame sync signal source.
//!
//! \param base is the base address of the McBSP module.
//! \param syncSource is the receiver frame sync source.
//!
//! This function sets external or internal sync signal source based on the
//! syncSource selection. Valid input for syncSource are:
//!
//! - \b MCBSP_RX_EXTERNAL_FRAME_SYNC_SOURCE - frame sync signal is supplied
//!                                            externally by pin FSR.
//! - \b MCBSP_RX_INTERNAL_FRAME_SYNC_SOURCE - frame sync signal is supplied
//!                                             by SRG.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setRxFrameSyncSource(uint32_t base,
                           const McBSP_RxFrameSyncSource syncSource)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set or clear FSRM bit.
    //
    HWREGH(base + MCBSP_O_PCR) =
    ((HWREGH(base + MCBSP_O_PCR) & ~MCBSP_PCR_FSRM) | (uint16_t)syncSource);
}

//*****************************************************************************
//
//! Configures the Transmit clock source.
//!
//! \param base is the base address of the McBSP module.
//! \param clockSource is clock source for the transmission pin.
//!
//!  This function configures the clock source for the transmitter. Valid input
//!  for rxClockSource are:
//!  - \b MCBSP_INTERNAL_TX_CLOCK_SOURCE - internal clock source. SRG is the
//!                                     source.
//!  - \b MCBSP_EXTERNAL_TX_CLOCK_SOURCE - external clock source.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setTxClockSource(uint32_t base, const McBSP_TxClockSource clockSource)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set or clear CLKXM bit.
    //
    HWREGH(base + MCBSP_O_PCR) =
    ((HWREGH(base + MCBSP_O_PCR) & ~MCBSP_PCR_CLKXM ) | (uint16_t)clockSource);
}

//*****************************************************************************
//
//! Configures the Receive clock source.
//!
//! \param base is the base address of the McBSP module.
//! \param clockSource is clock source for the reception pin.
//!
//!  This function configures the clock source for the receiver. Valid input
//!  for base are:
//!  - \b MCBSP_INTERNAL_RX_CLOCK_SOURCE - internal clock source. Sample Rate
//!                                         Generator will be used.
//!  - \b MCBSP_EXTERNAL_RX_CLOCK_SOURCE - external clock will drive the data.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setRxClockSource(uint32_t base, const McBSP_RxClockSource clockSource)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set or clear CLKRM bit.
    //
    HWREGH(base + MCBSP_O_PCR) =
    ((HWREGH(base + MCBSP_O_PCR) & ~MCBSP_PCR_CLKRM) | (uint16_t)clockSource);
}

//*****************************************************************************
//
//! Sets transmitter frame sync polarity.
//!
//! \param base is the base address of the McBSP module.
//! \param syncPolarity is the polarity of frame sync pulse.
//!
//! This function sets the polarity (rising or falling edge)of the frame sync
//! on FSX pin. Use \b MCBSP_TX_FRAME_SYNC_POLARITY_LOW for active low
//! frame sync pulse and \b MCBSP_TX_FRAME_SYNC_POLARITY_HIGH for active
//! high sync pulse.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setTxFrameSyncPolarity(uint32_t base,
                             const McBSP_TxFrameSyncPolarity syncPolarity)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set or clear FSXP bit.
    //
    HWREGH(base + MCBSP_O_PCR) =
    ((HWREGH(base + MCBSP_O_PCR) & ~MCBSP_PCR_FSXP) | (uint16_t)syncPolarity);
}

//*****************************************************************************
//! Sets receiver frame sync polarity.
//!
//! \param base is the base address of the McBSP module.
//! \param syncPolarity is the polarity of frame sync pulse.
//!
//! This function sets the polarity (rising or falling edge)of the frame sync
//! on FSR pin. Use \b MCBSP_RX_FRAME_SYNC_POLARITY_LOW for active low
//! frame sync pulse and \b MCBSP_RX_FRAME_SYNC_POLARITY_HIGH for active
//! high sync pulse.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setRxFrameSyncPolarity(uint32_t base,
                             const McBSP_RxFrameSyncPolarity syncPolarity)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set or clear FSRP bit.
    //
    HWREGH(base + MCBSP_O_PCR) =
    ((HWREGH(base + MCBSP_O_PCR) & ~MCBSP_PCR_FSRP) | (uint16_t)syncPolarity);
}

//*****************************************************************************
//! Sets transmitter clock polarity when using external clock source.
//!
//! \param base is the base address of the McBSP module.
//! \param clockPolarity is the polarity of external clock.
//!
//! This function sets the polarity (rising or falling edge) of the transmitter
//! clock on MCLKX pin.
//! Valid values for clockPolarity are:
//!   - \b MCBSP_TX_POLARITY_RISING_EDGE  for rising edge.
//!   - \b MCBSP_TX_POLARITY_FALLING_EDGE for falling edge.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setTxClockPolarity(uint32_t base,
                         const McBSP_TxClockPolarity clockPolarity)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear CLKXP bit first , then set or clear CLKXP bit.
    //
    HWREGH(base + MCBSP_O_PCR) =
    ((HWREGH(base + MCBSP_O_PCR) & ~MCBSP_PCR_CLKXP) |
     (uint16_t)clockPolarity);
}

//*****************************************************************************
//! Sets receiver clock polarity when using external clock source.
//!
//! \param base is the base address of the McBSP module.
//! \param clockPolarity is the polarity of external clock.
//!
//! This function sets the polarity (rising or falling edge) of the receiver
//! clock on MCLKR pin. If external clock is used, the polarity will affect
//! CLKG signal.
//! Valid values for clockPolarity are:
//!   - \b MCBSP_RX_POLARITY_RISING_EDGE for rising edge.
//!   - \b MCBSP_RX_POLARITY_FALLING_EDGE for falling edge.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_setRxClockPolarity(uint32_t base,
                         const McBSP_RxClockPolarity clockPolarity)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear CLKRP bit first , then set or clear CLKRP bit.
    //
    HWREGH(base + MCBSP_O_PCR) =
    ((HWREGH(base + MCBSP_O_PCR) & ~MCBSP_PCR_CLKRP) |
     (uint16_t)clockPolarity);
}

//*****************************************************************************
//
//! Read 8,12 or 16 bit data word from McBSP data receive registers.
//!
//! \param base is the base address of the McBSP port.
//!
//! This function returns the data value in data receive register.
//!
//! \return received data.
//
//*****************************************************************************
static inline
uint16_t McBSP_read16bitData(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Read DRR1 register.
    //
    return(HWREGH(base + MCBSP_O_DRR1));
}

//*****************************************************************************
//
//! Read 20, 24 or 32 bit data word from McBSP data receive registers.
//!
//! \param base is the base address of the McBSP port.
//!
//! This function returns the data values in data receive registers.
//!
//! \return received data.
//
//*****************************************************************************
static inline uint32_t
McBSP_read32bitData(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Read DDR1 register and return DDR2:DDR1.
    //
    return((((uint32_t)HWREGH(base + MCBSP_O_DRR2) << 16U) |
            HWREGH(base + MCBSP_O_DRR1)));
}

//*****************************************************************************
//
//! Write 8,12 or 16 bit data word to McBSP data transmit registers.
//!
//! \param base is the base address of the McBSP port.
//! \param data is the data to be written.
//!
//! This function writes 8,12 or 16 bit data to data transmit register.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_write16bitData(uint32_t base, uint16_t data)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Write to DXR1 register.
    //
    HWREGH(base + MCBSP_O_DXR1) = data;
}

//*****************************************************************************
//
//! Write 20, 24 or 32 bit data word to McBSP data transmit registers.
//!
//! \param base is the base address of the McBSP port.
//! \param data is the data to be written.
//!
//! This function writes 20, 24 or 32 bit data to data transmit registers.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_write32bitData(uint32_t base, uint32_t data)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Write to DXR2 register first.
    //
    HWREGH(base + MCBSP_O_DXR2) = data >> 16U;

    //
    // Write to DXR1 register.
    //
    HWREGH(base + MCBSP_O_DXR1) = data & 0xFFFFU;
}

//*****************************************************************************
//
//! Return left justified for data for U Law or A Law companding.
//!
//! \param data is the 14 bit word.
//! \param compandingType specifies the type comapnding desired.
//!
//!  This functions returns U law or A law adjusted word.
//!
//! \return U law or A law left justified word.
//
//*****************************************************************************
static inline uint16_t
McBSP_getLeftJustifyData(uint16_t data,
                         const McBSP_CompandingType compandingType)
{
    return(data << (uint16_t)compandingType);
}


//*****************************************************************************
//
//! Enable Recieve Interrupt.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function enables Recieve Interrupt on RRDY.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_enableRxInterrupt(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set RINT ENA bit.
    //
    HWREGH(base + MCBSP_O_MFFINT) |= MCBSP_MFFINT_RINT;
}

//*****************************************************************************
//
//! Disable Recieve Interrupt.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function disables Recieve Interrupt on RRDY.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_disableRxInterrupt(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear RINT ENA bit.
    //
    HWREGH(base + MCBSP_O_MFFINT) |= ~(MCBSP_MFFINT_RINT);
}

//*****************************************************************************
//
//! Enable Transmit Interrupt.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function enables Transmit Interrupt on XRDY.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_enableTxInterrupt(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Set XINT ENA bit.
    //
    HWREGH(base + MCBSP_O_MFFINT) |= MCBSP_MFFINT_XINT;
}

//*****************************************************************************
//
//! Disable Transmit Interrupt.
//!
//! \param base is the base address of the McBSP module.
//!
//! This function disables Transmit Interrupt on XRDY.
//!
//! \return None.
//
//*****************************************************************************
static inline void
McBSP_disableTxInterrupt(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(McBSP_isBaseValid(base));

    //
    // Clear XINT ENA bit.
    //
    HWREGH(base + MCBSP_O_MFFINT) |= ~(MCBSP_MFFINT_XINT);
}

//*****************************************************************************
//
//! Write 8,12 or 16 bit data word to McBSP data transmit registers
//!
//! \param base is the base address of the McBSP port.
//! \param data is the data to be written.
//!
//! This function sends 16 bit or less data to the transmitter buffer.
//!
//! \return None.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_transmit16BitDataNonBlocking(uint32_t base, uint16_t data);

//*****************************************************************************
//
//! Write 8,12 or 16 bit data word to McBSP data transmit registers
//!
//! \param base is the base address of the McBSP port.
//! \param data is the data to be written.
//!
//! This function sends 16 bit or less data to the transmitter buffer. If
//! transmit buffer is not ready the function will wait until transmit buffer
//! is empty. If the transmitter buffer is empty the data will be written to
//! the data registers.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_transmit16BitDataBlocking(uint32_t base, uint16_t data);

//*****************************************************************************
//
//! Write 20 , 24 or 32 bit data word to McBSP data transmit registers
//!
//! \param base is the base address of the McBSP port.
//! \param data is the data to be written.
//!
//! This function sends 20 , 24 or 32 bit data to the transmitter buffer. If
//! the transmitter buffer is empty the data will be written to the data
//! registers.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_transmit32BitDataNonBlocking(uint32_t base, uint32_t data);

//*****************************************************************************
//
//! Write 20 , 24 or 32 bit data word to McBSP data transmit registers
//!
//! \param base is the base address of the McBSP port.
//! \param data is the data to be written.
//!
//! This function sends 20 , 24 or 32 bit data to the transmitter buffer. If
//! transmit buffer is not ready the function will wait until transmit buffer
//! is empty. If the transmitter buffer is empty the data will be written
//! to the data registers.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_transmit32BitDataBlocking(uint32_t base, uint32_t data);

//*****************************************************************************
//
//! Read 8,12 or 16 bit data word from McBSP data receive registers
//!
//! \param base is the base address of the McBSP port.
//! \param receiveData is the pointer to the receive data.
//!
//! This function reads 8,12 or 16 bit data from the receiver buffer.
//! If the receiver buffer has new data, the data will be read.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_receive16BitDataNonBlocking(uint32_t base, uint16_t *receiveData);

//*****************************************************************************
//
//! Read 8,12 or 16 bit data word from McBSP data receive registers
//!
//! \param base is the base address of the McBSP port.
//! \param receiveData is the pointer to the receive data.
//!
//! This function reads 8,12 or 16 bit data from the receiver buffer. If
//! receiver buffer is not ready the function will wait until receiver buffer
//! has new data.
//! If the receiver buffer has new data, the data will be read.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_receive16BitDataBlocking(uint32_t base, uint16_t *receiveData);

//*****************************************************************************
//
//! Read 20, 24 or 32 bit data word from McBSP data receive registers
//!
//! \param base is the base address of the McBSP port.
//! \param receiveData is the pointer to the receive data.
//!
//! This function reads 20, 24 or 32 bit data from the receiver buffer.
//! If the receiver buffer has new data, the data will be read.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_receive32BitDataNonBlocking(uint32_t base, uint32_t *receiveData);

//*****************************************************************************
//
//! Read 20, 24 or 32 bit data word from McBSP data receive registers
//!
//! \param base is the base address of the McBSP port.
//! \param receiveData is the pointer to the receive data.
//!
//! This function reads 20, 24 or 32 bit data from the receiver buffer. If
//! receiver buffer is not ready the function will wait until receiver buffer
//! has new data.
//! If the receiver buffer has new data, the data will be read.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_receive32BitDataBlocking(uint32_t base, uint32_t *receiveData);


//*****************************************************************************
//
//! Sets number of words per frame and bits per word for data Reception.
//!
//! \param base is the base address of the McBSP module.
//! \param dataFrame is the data frame phase.
//! \param bitsPerWord is the number of bits per word.
//! \param wordsPerFrame is the number of words per frame per phase.
//!
//! This function sets the number of bits per word and the number of words per
//! frame for the given phase.
//! Valid inputs for phase are \b MCBSP_PHASE_ONE_FRAME or \b
//! MCBSP_PHASE_TWO_FRAME representing the first or second frame phase
//! respectively. Valid value for bitsPerWord are:
//! - \b MCBSP_BITS_PER_WORD_8   8 bit word.
//! - \b MCBSP_BITS_PER_WORD_12  12 bit word.
//! - \b MCBSP_BITS_PER_WORD_16  16 bit word.
//! - \b MCBSP_BITS_PER_WORD_20  20 bit word.
//! - \b MCBSP_BITS_PER_WORD_24  24 bit word.
//! - \b MCBSP_BITS_PER_WORD_32  32 bit word.
//! The maximum value for wordsPerFrame is 127 (128 - 1)representing 128 words.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_setRxDataSize(uint32_t base, const McBSP_DataPhaseFrame dataFrame,
                    const McBSP_DataBitsPerWord  bitsPerWord,
                    uint16_t wordsPerFrame);

//*****************************************************************************
//
//! Sets number of words per frame and bits per word for data Transmission.
//!
//! \param base is the base address of the McBSP module.
//! \param dataFrame is the data frame phase.
//! \param bitsPerWord is the number of bits per word.
//! \param wordsPerFrame is the number of words per frame per phase.
//!
//! This function sets the number of bits per word and the number of words per
//! frame for the given phase.
//! Valid inputs for phase are \b MCBSP_PHASE_ONE_FRAME or \b
//! MCBSP_PHASE_TWO_FRAME representing single or dual phase respectively.
//! Valid values for bitsPerWord are:
//! - \b MCBSP_BITS_PER_WORD_8   8 bit word.
//! - \b MCBSP_BITS_PER_WORD_12  12 bit word.
//! - \b MCBSP_BITS_PER_WORD_16  16 bit word.
//! - \b MCBSP_BITS_PER_WORD_20  20 bit word.
//! - \b MCBSP_BITS_PER_WORD_24  24 bit word.
//! - \b MCBSP_BITS_PER_WORD_32  32 bit word.
//! The maximum value for wordsPerFrame is 127 (128 - 1)representing 128 words.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_setTxDataSize(uint32_t base,
                    const McBSP_DataPhaseFrame dataFrame,
                    const McBSP_DataBitsPerWord  bitsPerWord,
                    uint16_t wordsPerFrame);

//*****************************************************************************
//
//! Disables a channel in an eight partition receiver
//!
//! \param base is the base address of the McBSP module.
//! \param partition is the partition of the channel.
//! \param channel is the receiver channel number to be enabled.
//!
//!  This function disables the given receiver channel number for the partition
//!  provided.
//!  Valid values for partition are \b MCBSP_MULTICHANNEL_TWO_PARTITION or
//!  \b MCBSP_MULTICHANNEL_EIGHT_PARTITION for 2 or 8 partitions respectively.
//!  Valid values for channel range from 0 to 127.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_disableRxChannel(uint32_t base,
                       const McBSP_MultichannelPartition partition,
                       uint16_t channel);

//*****************************************************************************
//
//! Enables a channel for eight partition receiver
//!
//! \param base is the base address of the McBSP module.
//! \param partition is the partition of the channel.
//! \param channel is the receiver channel number to be enabled.
//!
//!  This function enables the given receiver channel number for the partition
//!  provided.
//!  Valid values for partition are \b MCBSP_MULTICHANNEL_TWO_PARTITION or
//!  \b MCBSP_MULTICHANNEL_EIGHT_PARTITION for 2 or 8 partitions respectively.
//!  Valid values for channel range from 0 to 127.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_enableRxChannel(uint32_t base,
                      const McBSP_MultichannelPartition partition,
                      uint16_t channel);

//*****************************************************************************
//
//! Disables a channel in an eight partition transmitter
//!
//! \param base is the base address of the McBSP module.
//! \param partition is the partition of the channel.
//! \param channel is the transmitter channel number to be enabled.
//!
//!  This function disables the given transmitter channel number for the
//!  partition provided.
//!  Valid values for partition are \b MCBSP_MULTICHANNEL_TWO_PARTITION or
//!  \b MCBSP_MULTICHANNEL_EIGHT_PARTITION for 2 or 8 partitions respectively.
//!  Valid values for channel range from 0 to 127.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_disableTxChannel(uint32_t base,
                       const McBSP_MultichannelPartition partition,
                       uint16_t channel);

//*****************************************************************************
//
//! Enables a channel for eight partition transmitter
//!
//! \param base is the base address of the McBSP module.
//! \param partition is the partition of the channel.
//! \param channel is the transmitter channel number to be enabled.
//!
//!  This function enables the given transmitter channel number for the
//!  partition provided.
//!  Valid values for partition are \b MCBSP_MULTICHANNEL_TWO_PARTITION or
//!  \b MCBSP_MULTICHANNEL_EIGHT_PARTITION for 2 or 8 partitions respectively.
//!  Valid values for channel range from 0 to 127.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_enableTxChannel(uint32_t base,
                      const McBSP_MultichannelPartition partition,
                      uint16_t channel);

//*****************************************************************************
//
//! Configures transmitter clock
//!
//! \param base is the base address of the McBSP module.
//! \param ptrClockParams is a pointer to a structure containing \e clock
//!        parameters McBSP_ClockParams.
//! This function sets up the transmitter clock. The following are valid
//! values and ranges for the parameters of the McBSP_TxFsyncParams.
//!  - \b clockSRGSyncFSR    - true to sync with signal on FSR pin,
//!                            false to ignore signal on FSR pin.
//!                            the pulse on FSR pin.
//!  - \b clockSRGDivider    - Maximum valid value is 255.
//!  - \b clockSource        - MCBSP_EXTERNAL_TX_CLOCK_SOURCE or
//!                            MCBSP_INTERNAL_TX_CLOCK_SOURCE
//!  - \b clockTxSRGSource   - MCBSP_SRG_TX_CLOCK_SOURCE_LSPCLK or
//!                            MCBSP_SRG_TX_CLOCK_SOURCE_MCLKR_PIN
//!  - \b clockMCLKXPolarity - Output polarity on MCLKX pin.
//!                          - MCBSP_TX_POLARITY_RISING_EDGE
//!                          - MCBSP_TX_POLARITY_FALLING_EDGE
//!  - \b clockMCLKRPolarity - Input polarity on MCLKR pin (if SRG is
//!                            sourced from MCLKR pin).
//!                          - MCBSP_RX_POLARITY_FALLING_EDGE
//!                          - MCBSP_RX_POLARITY_RISING_EDGE
//!
//! \note Make sure the clock divider is such that, the McBSP clock is not
//!       running faster than 1/2 the speed of the source clock.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_configureTxClock(uint32_t base,
                       const McBSP_ClockParams * ptrClockParams);

//*****************************************************************************
//
//! Configures receiver clock
//!
//! \param base is the base address of the McBSP module.
//! \param ptrClockParams is a pointer to a structure containing \e clock
//!        parameters McBSP_ClockParams.
//! This function sets up the receiver clock. The following are valid
//! values and ranges for the parameters of the McBSP_TxFsyncParams.
//!  - \b clockSRGSyncFlag  - true to sync with signal on FSR pin, false to
//!                           ignore the pulse on FSR pin.
//!  - \b clockSRGDivider   - Maximum valid value is 255.
//!  - \b clockSource       - MCBSP_EXTERNAL_RX_CLOCK_SOURCE or
//!                           MCBSP_INTERNAL_RX_CLOCK_SOURCE
//!  - \b clockRxSRGSource  - MCBSP_SRG_RX_CLOCK_SOURCE_LSPCLK or
//!                           MCBSP_SRG_RX_CLOCK_SOURCE_MCLKX_PIN
//!  - \b clockMCLKRPolarity- output polarity on MCLKR pin.
//!                         - MCBSP_RX_POLARITY_FALLING_EDGE or
//!                         - MCBSP_RX_POLARITY_RISING_EDGE
//!  - \b clockMCLKXPolarity- Input polarity on MCLKX pin (if SRG is sourced
//!                           from MCLKX pin).
//!                         - MCBSP_TX_POLARITY_RISING_EDGE or
//!                         - MCBSP_TX_POLARITY_FALLING_EDGE
//!
//! \note Make sure the clock divider is such that, the McBSP clock is not
//!       running faster than 1/2 the speed of the source clock.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_configureRxClock(uint32_t base,
                       const McBSP_ClockParams * ptrClockParams);

//*****************************************************************************
//
//! Configures transmitter frame sync.
//!
//! \param base is the base address of the McBSP module.
//! \param ptrFsyncParams is a pointer to a structure containing \e frame sync
//!        parameters McBSPTxFsyncParams.
//! This function sets up the transmitter frame sync. The following are valid
//! values and ranges for the parameters of the McBSPTxFsyncParams.
//!  - \b syncSRGSyncFSRFlag - true to sync with signal on FSR pin, false to
//!                            ignore the pulse on FSR pin.This value has to
//!                            be similar to the value of
//!                            McBSP_ClockParams.clockSRGSyncFlag.
//!  - \b syncErrorDetect    - true to enable frame sync error detect. false
//!                            to disable.
//!  - \b syncClockDivider     - Maximum valid value is 4095.
//!  - \b syncPulseDivider   - Maximum valid value is 255.
//!  - \b syncSourceTx       - MCBSP_TX_INTERNAL_FRAME_SYNC_SOURCE or
//!                            MCBSP_TX_EXTERNAL_FRAME_SYNC_SOURCE
//!  - \b syncIntSource      - MCBSP_TX_INTERNAL_FRAME_SYNC_DATA or
//!                            MCBSP_TX_INTERNAL_FRAME_SYNC_SRG
//!  - \b syncFSXPolarity    - MCBSP_TX_FRAME_SYNC_POLARITY_LOW or
//!                            MCBSP_TX_FRAME_SYNC_POLARITY_HIGH.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_configureTxFrameSync(uint32_t base,
                           const McBSP_TxFsyncParams * ptrFsyncParams);

//*****************************************************************************
//
//! Configures receiver frame sync.
//!
//! \param base is the base address of the McBSP module.
//! \param ptrFsyncParams is a pointer to a structure containing \e frame sync
//!        parameters McBSP_RxFsyncParams.
//! This function sets up the receiver frame sync. The following are valid
//! values and ranges for the parameters of the McBSPTxFsyncParams.
//!  - \b syncSRGSyncFSRFlag   - true to sync with signal on FSR pin,
//!                              false to ignore the pulse on FSR pin.
//!                              This value has to be similar to the value of
//!                              McBSP_ClockParams.clockSRGSyncFlag.
//!  - \b syncErrorDetect      - true to enable frame sync error detect.
//!                              false to disable.
//!  - \b syncClockDivider     - Maximum valid value is 4095.
//!  - \b syncPulseDivider     - Maximum valid value is 255.
//!  - \b syncSourceRx         - MCBSP_RX_INTERNAL_FRAME_SYNC_SOURCE or
//!                              MCBSP_RX_EXTERNAL_FRAME_SYNC_SOURCE
//!  - \b syncFSRPolarity      - MCBSP_RX_FRAME_SYNC_POLARITY_LOW or
//!                              MCBSP_RX_FRAME_SYNC_POLARITY_HIGH
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_configureRxFrameSync(uint32_t base,
                           const McBSP_RxFsyncParams * ptrFsyncParams);

//*****************************************************************************
//
//! Configures transmitter data format.
//!
//! \param base is the base address of the McBSP module.
//! \param ptrDataParams is a pointer to a structure containing \e data format
//!        parameters McBSPTxDataParams.
//! This function sets up the transmitter data format and properties. The
//! following are valid values and ranges for the parameters of the
//! McBSPTxDataParams.
//!  - \b loopbackModeFlag   - true for digital loop-back mode.
//!                            false for no loop-back mode.
//!  - \b twoPhaseModeFlag   - true for two phase mode.
//!                            false for single phase mode.
//!  - \b pinDelayEnableFlag - true to enable DX pin delay.
//!                            false to disable DX pin delay.
//!  - \b phase1FrameLength  - maximum value of 127.
//!  - \b phase2FrameLength  - maximum value of 127.
//!  - \b clockStopMode      - MCBSP_CLOCK_SPI_MODE_NO_DELAY or
//!                            MCBSP_CLOCK_SPI_MODE_DELAY
//!  - \b phase1WordLength   - MCBSP_BITS_PER_WORD_x , x = 8,12,16,20,24,32
//!  - \b phase2WordLength   - MCBSP_BITS_PER_WORD_x , x = 8,12,16,20,24,32
//!  - \b compandingMode     - MCBSP_COMPANDING_NONE,
//!                            MCBSP_COMPANDING_NONE_LSB_FIRST
//!                            MCBSP_COMPANDING_U_LAW_SET or
//!                            MCBSP_COMPANDING_A_LAW_SET.
//!  - \b dataDelayBits      - MCBSP_DATA_DELAY_BIT_0,
//!                            MCBSP_DATA_DELAY_BIT_1 or
//!                            MCBSP_DATA_DELAY_BIT_2
//!  - \b interruptMode      - MCBSP_TX_ISR_SOURCE_TX_READY,
//!                            MCBSP_TX_ISR_SOURCE_END_OF_BLOCK,
//!                            MCBSP_TX_ISR_SOURCE_FRAME_SYNC or
//!                            MCBSP_TX_ISR_SOURCE_SYNC_ERROR
//!
//!     \b Note - When using companding,phase1WordLength and phase2WordLength
//!               must be 8 bits wide.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_configureTxDataFormat(uint32_t base,
                            const McBSP_TxDataParams * ptrDataParams);

//*****************************************************************************
//
//! Configures receiver data format.
//!
//! \param base is the base address of the McBSP module.
//! \param ptrDataParams is a pointer to a structure containing data format
//!        parameters McBSP_RxDataParams.
//! This function sets up the transmitter data format and properties. The
//! following are valid values and ranges for the parameters of the
//! McBSP_RxDataParams.
//!  - \b loopbackModeFlag   - true for digital loop-back mode.
//!                            false for non loop-back mode.
//!  - \b twoPhaseModeFlag   - true for two phase mode.
//!                            false for single phase mode.
//!  - \b phase1FrameLength  - maximum value of 127.
//!  - \b phase2FrameLength  - maximum value of 127.
//!  - \b phase1WordLength   - MCBSP_BITS_PER_WORD_x , x = 8,12,16,20,24,32
//!  - \b phase2WordLength   - MCBSP_BITS_PER_WORD_x , x = 8,12,16,20,24,32
//!  - \b compandingMode     - MCBSP_COMPANDING_NONE,
//!                            MCBSP_COMPANDING_NONE_LSB_FIRST
//!                            MCBSP_COMPANDING_U_LAW_SET or
//!                            MCBSP_COMPANDING_A_LAW_SET.
//!  - \b dataDelayBits      - MCBSP_DATA_DELAY_BIT_0,
//!                            MCBSP_DATA_DELAY_BIT_1 or
//!                            MCBSP_DATA_DELAY_BIT_2
//!  - \b signExtMode        - MCBSP_RIGHT_JUSTIFY_FILL_ZERO,
//!                            MCBSP_RIGHT_JUSTIFY_FILL_SIGN or
//!                            MCBSP_LEFT_JUSTIFY_FILL_ZER0
//!  - \b interruptMode      - MCBSP_RX_ISR_SOURCE_SERIAL_WORD,
//!                            MCBSP_RX_ISR_SOURCE_END_OF_BLOCK,
//!                            MCBSP_RX_ISR_SOURCE_FRAME_SYNC or
//!                            MCBSP_RX_ISR_SOURCE_SYNC_ERROR
//!
//!     \b Note - When using companding,phase1WordLength and phase2WordLength
//!               must be 8 bits wide.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_configureRxDataFormat(uint32_t base,
                            const McBSP_RxDataParams * ptrDataParams);

//*****************************************************************************
//
//! Configures transmitter multichannel.
//!
//! \param base is the base address of the McBSP module.
//! \param ptrMchnParams is a pointer to a structure containing multichannel
//!        parameters McBSP_TxMultichannelParams.
//!
//! This function sets up the transmitter multichannel mode. The following are
//! valid values and ranges for the parameters of the
//! McBSP_TxMultichannelParams.
//!  - \b channelCount         - Maximum value of 128 for partition 8
//!                              Maximum value of 32 for partition 2
//!  - \b ptrChannelsList      - Pointer to an array of size channelCount that
//!                              has unique channels.
//!  - \b multichannelMode    -  MCBSP_ALL_TX_CHANNELS_ENABLED,
//!                              MCBSP_TX_CHANNEL_SELECTION_ENABLED,
//!                              MCBSP_ENABLE_MASKED_TX_CHANNEL_SELECTION or
//!                              MCBSP_SYMMERTIC_RX_TX_SELECTION
//!  - \b partition           -  MCBSP_MULTICHANNEL_TWO_PARTITION or
//!                              MCBSP_MULTICHANNEL_EIGHT_PARTITION
//! \note -  In 2 partition mode only channels that belong to a single even or
//!          odd block number should be listed. It is valid to have an even and
//!          odd channels. For example you can have channels [48 -63] and
//!          channels [96 - 111] enables as one belongs to an even block and
//!          the other to an odd block or two partitions. But not channels
//!          [48 - 63] and channels [112 - 127] since they both are even blocks
//!          or similar partitions.
//!
//! \return returns the following error codes.
//!    - \b MCBSP_ERROR_EXCEEDED_CHANNELS - number of channels exceeds 128
//!    - \b MCBSP_ERROR_2_PARTITION_A - invalid channel combination for
//!                                     partition A
//!    - \b MCBSP_ERROR_2_PARTITION_B - invalid channel combination for
//!                                     partition B
//!    - \b MCBSP_ERROR_INVALID_MODE - invalid transmitter channel mode.
//!
//! \return  Returns the following error codes.
//!  - \b MCBSP_ERROR_EXCEEDED_CHANNELS - Exceeded number of channels.
//!  - \b MCBSP_ERROR_2_PARTITION_A - Error in 2 partition A setup.
//!  - \b MCBSP_ERROR_2_PARTITION_B - Error in 2 partition B setup.
//!  - \b MCBSP_ERROR_INVALID_MODE - Invalid mode.
//
//*****************************************************************************
extern uint16_t
McBSP_configureTxMultichannel(uint32_t base,
                             const McBSP_TxMultichannelParams * ptrMchnParams);

//*****************************************************************************
//
//! Configures receiver multichannel.
//!
//! \param base is the base address of the McBSP module.
//! \param ptrMchnParams is a pointer to a structure containing multichannel
//!        parameters McBSP_RxMultiChannelParams.
//!
//! This function sets up the receiver multichannel mode. The following are
//! valid values and ranges for the parameters of the McBSPMultichannelParams.
//!  - \b channelCount       - Maximum value of 128 for partition 8
//!                            Maximum value of 32 for partition 2
//!  - \b ptrChannelsList    - Pointer to an array of size channelCount that
//!                            has unique channels.
//!  - \b multichannelMode  -  MCBSP_ALL_RX_CHANNELS_ENABLED,
//!                            MCBSP_RX_CHANNEL_SELECTION_ENABLED,
//!  - \b partition          - MCBSP_MULTICHANNEL_TWO_PARTITION or
//!                            MCBSP_MULTICHANNEL_EIGHT_PARTITION
//! \note -  In 2 partition mode only channels that belong to a single even or
//!          odd block number should be listed. It is valid to have an even
//!          and odd channels. For example you can have channels [48 - 63] and
//!          channels [96 - 111] enables as one belongs to an even block and
//!          the other to an odd block or two partitions. But not channels
//!          [48 - 63]and channels [112 - 127] since they both are even blocks
//!          or similar partitions.
//!
//! \return returns the following error codes.
//!    - \b MCBSP_ERROR_EXCEEDED_CHANNELS - number of channels exceeds 128
//!    - \b MCBSP_ERROR_2_PARTITION_A - invalid channel combination for
//!                                     partition A
//!    - \b MCBSP_ERROR_2_PARTITION_B - invalid channel combination for
//!                                     partition B
//!    - \b MCBSP_ERROR_INVALID_MODE - invalid transmitter channel mode.
//!
//! \return  Returns the following error codes.
//!  - \b MCBSP_ERROR_EXCEEDED_CHANNELS - Exceeded number of channels.
//!  - \b MCBSP_ERROR_2_PARTITION_A - Error in 2 partition A setup.
//!  - \b MCBSP_ERROR_2_PARTITION_B - Error in 2 partition B setup.
//!  - \b MCBSP_ERROR_INVALID_MODE - Invalid mode.
//
//*****************************************************************************
extern uint16_t
McBSP_configureRxMultichannel(uint32_t base,
                             const McBSP_RxMultichannelParams * ptrMchnParams);

//*****************************************************************************
//
//! Configures McBSP in SPI master mode
//!
//! \param base is the base address of the McBSP module.
//! \param ptrSPIMasterMode is a pointer to a structure containing SPI
//!        parameters McBSP_SPIMasterModeParams.
//! This function sets up the McBSP module in SPI master mode.The following are
//! valid values and ranges for the parameters of the
//! McBSP_SPIMasterModeParams.
//!  - \b loopbackModeFlag - true for digital loop-back
//!                          false for no loop-back
//!  - \b clockStopMode    - MCBSP_CLOCK_SPI_MODE_NO_DELAY or
//!                          MCBSP_CLOCK_SPI_MODE_DELAY
//!  - \b wordLength       - MCBSP_BITS_PER_WORD_x , x = 8,12,16,20,24,32
//!  - \b spiMode            It represents the clock polarity can take values:
//!                        - MCBSP_TX_POLARITY_RISING_EDGE or
//!                          MCBSP_TX_POLARITY_FALLING_EDGE
//!  - \b clockSRGDivider  - Maximum valid value is 255.
//!
//! \note Make sure the clock divider is such that, the McBSP clock is not
//!       running faster than 1/2 the speed of the source clock.
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_configureSPIMasterMode(uint32_t base,
                           const McBSP_SPIMasterModeParams * ptrSPIMasterMode);

//*****************************************************************************
//
//! Configures McBSP in SPI slave mode
//!
//! \param base is the base address of the McBSP module.
//! \param ptrSPISlaveMode is a pointer to a structure containing SPI
//!        parameters McBSP_SPISlaveModeParams.
//! This function sets up the McBSP module in SPI slave mode.The following are
//! valid values and ranges for the parameters of the McBSP_SPISlaveModeParams.
//!  - \b loopbackModeFlag - true for digital loop-back
//!                          false for no loop-back
//!  - \b clockStopMode    - MCBSP_CLOCK_SPI_MODE_NO_DELAY or
//!                          MCBSP_CLOCK_SPI_MODE_DELAY
//!  - \b wordLength       - MCBSP_BITS_PER_WORD_x , x = 8,12,16,20,24,32
//!  - \b spiMode            It represents the clock polarity and can take
//!                          values:
//!                        - MCBSP_RX_POLARITY_FALLING_EDGE or
//!                          MCBSP_RX_POLARITY_RISING_EDGE
//!
//! \return None.
//
//*****************************************************************************
extern void
McBSP_configureSPISlaveMode(uint32_t base,
                            const McBSP_SPISlaveModeParams * ptrSPISlaveMode);

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

#endif // MCBSP_H
