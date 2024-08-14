/*
 *   Copyright (c) Texas Instruments Incorporated 2016
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 *  \file     mcan.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of MCAN.
 *            This also contains some related macros.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include "inc/stw_types.h"
#include "inc/stw_dataTypes.h"
#include "inc/hw_types_mcan.h"
#include "mcan.h"
#include "debug.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \brief  Mask and shift for Tx Buffers elements.
 */
#define MCANSS_TX_BUFFER_ELEM_ID_SHIFT                           (0U)
#define MCANSS_TX_BUFFER_ELEM_ID_MASK                            (0x1FFFFFFFU)
#define MCANSS_TX_BUFFER_ELEM_RTR_SHIFT                          (29U)
#define MCANSS_TX_BUFFER_ELEM_RTR_MASK                           (0x20000000U)
#define MCANSS_TX_BUFFER_ELEM_XTD_SHIFT                          (30U)
#define MCANSS_TX_BUFFER_ELEM_XTD_MASK                           (0x40000000U)
#define MCANSS_TX_BUFFER_ELEM_ESI_SHIFT                          (31U)
#define MCANSS_TX_BUFFER_ELEM_ESI_MASK                           (0x80000000U)
#define MCANSS_TX_BUFFER_ELEM_DLC_SHIFT                          (16U)
#define MCANSS_TX_BUFFER_ELEM_DLC_MASK                           (0x000F0000U)
#define MCANSS_TX_BUFFER_ELEM_BRS_SHIFT                          (20U)
#define MCANSS_TX_BUFFER_ELEM_BRS_MASK                           (0x00100000U)
#define MCANSS_TX_BUFFER_ELEM_FDF_SHIFT                          (21U)
#define MCANSS_TX_BUFFER_ELEM_FDF_MASK                           (0x00200000U)
#define MCANSS_TX_BUFFER_ELEM_EFC_SHIFT                          (23U)
#define MCANSS_TX_BUFFER_ELEM_EFC_MASK                           (0x00800000U)
#define MCANSS_TX_BUFFER_ELEM_MM_SHIFT                           (24U)
#define MCANSS_TX_BUFFER_ELEM_MM_MASK                            (0xFF000000U)

/**
 * \brief  Mask and shift for Rx Buffers elements.
 */
#define MCANSS_RX_BUFFER_ELEM_ID_SHIFT                           (0U)
#define MCANSS_RX_BUFFER_ELEM_ID_MASK                            (0x1FFFFFFFU)
#define MCANSS_RX_BUFFER_ELEM_RTR_SHIFT                          (29U)
#define MCANSS_RX_BUFFER_ELEM_RTR_MASK                           (0x20000000U)
#define MCANSS_RX_BUFFER_ELEM_XTD_SHIFT                          (30U)
#define MCANSS_RX_BUFFER_ELEM_XTD_MASK                           (0x40000000U)
#define MCANSS_RX_BUFFER_ELEM_ESI_SHIFT                          (31U)
#define MCANSS_RX_BUFFER_ELEM_ESI_MASK                           (0x80000000U)
#define MCANSS_RX_BUFFER_ELEM_RXTS_SHIFT                         (0U)
#define MCANSS_RX_BUFFER_ELEM_RXTS_MASK                          (0x0000FFFFU)
#define MCANSS_RX_BUFFER_ELEM_DLC_SHIFT                          (16U)
#define MCANSS_RX_BUFFER_ELEM_DLC_MASK                           (0x000F0000U)
#define MCANSS_RX_BUFFER_ELEM_BRS_SHIFT                          (20U)
#define MCANSS_RX_BUFFER_ELEM_BRS_MASK                           (0x00100000U)
#define MCANSS_RX_BUFFER_ELEM_FDF_SHIFT                          (21U)
#define MCANSS_RX_BUFFER_ELEM_FDF_MASK                           (0x00200000U)
#define MCANSS_RX_BUFFER_ELEM_FIDX_SHIFT                         (24U)
#define MCANSS_RX_BUFFER_ELEM_FIDX_MASK                          (0x7F000000U)
#define MCANSS_RX_BUFFER_ELEM_ANMF_SHIFT                         (31U)
#define MCANSS_RX_BUFFER_ELEM_ANMF_MASK                          (0x80000000U)

/**
 * \brief  Mask and shift for Standard Message ID Filter Elements.
 */
#define MCANSS_STD_ID_FILTER_SFID2_SHIFT                         (0U)
#define MCANSS_STD_ID_FILTER_SFID2_MASK                          (0x000003FFU)
#define MCANSS_STD_ID_FILTER_SFID1_SHIFT                         (16U)
#define MCANSS_STD_ID_FILTER_SFID1_MASK                          (0x03FF0000U)
#define MCANSS_STD_ID_FILTER_SFEC_SHIFT                          (27U)
#define MCANSS_STD_ID_FILTER_SFEC_MASK                           (0x38000000U)
#define MCANSS_STD_ID_FILTER_SFT_SHIFT                           (30U)
#define MCANSS_STD_ID_FILTER_SFT_MASK                            (0xC0000000U)

/**
 * \brief  Extended Message ID Filter Element.
 */
#define MCANSS_EXT_ID_FILTER_EFID2_SHIFT                        (0U)
#define MCANSS_EXT_ID_FILTER_EFID2_MASK                         (0x1FFFFFFFU)
#define MCANSS_EXT_ID_FILTER_EFID1_SHIFT                        (0U)
#define MCANSS_EXT_ID_FILTER_EFID1_MASK                         (0x1FFFFFFFU)
#define MCANSS_EXT_ID_FILTER_EFEC_SHIFT                         (29U)
#define MCANSS_EXT_ID_FILTER_EFEC_MASK                          (0xE0000000U)
#define MCANSS_EXT_ID_FILTER_EFT_SHIFT                          (30U)
#define MCANSS_EXT_ID_FILTER_EFT_MASK                           (0xC0000000U)

/**
 * \brief  Mask and shift for Tx Event FIFO elements.
 */
#define MCANSS_TX_EVENT_FIFO_ELEM_ID_SHIFT                      (0U)
#define MCANSS_TX_EVENT_FIFO_ELEM_ID_MASK                       (0x1FFFFFFFU)
#define MCANSS_TX_EVENT_FIFO_ELEM_RTR_SHIFT                     (29U)
#define MCANSS_TX_EVENT_FIFO_ELEM_RTR_MASK                      (0x20000000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_XTD_SHIFT                     (30U)
#define MCANSS_TX_EVENT_FIFO_ELEM_XTD_MASK                      (0x40000000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_ESI_SHIFT                     (31U)
#define MCANSS_TX_EVENT_FIFO_ELEM_ESI_MASK                      (0x80000000U)

#define MCANSS_TX_EVENT_FIFO_ELEM_TXTS_SHIFT                    (0U)
#define MCANSS_TX_EVENT_FIFO_ELEM_TXTS_MASK                     (0x0000FFFFU)
#define MCANSS_TX_EVENT_FIFO_ELEM_DLC_SHIFT                     (16U)
#define MCANSS_TX_EVENT_FIFO_ELEM_DLC_MASK                      (0x000F0000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_BRS_SHIFT                     (20U)
#define MCANSS_TX_EVENT_FIFO_ELEM_BRS_MASK                      (0x00100000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_FDF_SHIFT                     (21U)
#define MCANSS_TX_EVENT_FIFO_ELEM_FDF_MASK                      (0x00200000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_ET_SHIFT                      (22U)
#define MCANSS_TX_EVENT_FIFO_ELEM_ET_MASK                       (0x00C00000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_MM_SHIFT                      (24U)
#define MCANSS_TX_EVENT_FIFO_ELEM_MM_MASK                       (0xFF000000U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/**
 * \brief   This API will unblock write access to write protected registers.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 *
 * \return  None.
 */
static void MCAN_writeProtectedRegAccessUnlock(uint32_t baseAddr);

/**
 * \brief   This API will block write access to write protected registers.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 *
 * \return  None.
 */
static void MCAN_writeProtectedRegAccessLock(uint32_t baseAddr);

/**
 * \brief   This API will load the register from ECC memory bank.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 * \param   regOffset       Offset of the register to read.
 *
 * \return  None.
 */
static void MCAN_eccLoadRegister(uint32_t baseAddr, uint32_t regOffset);

/**
 * \brief   This API will read the message object from Message RAM.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 * \param   elemAddr        Address of the message object.
 * \param   elem            Message Object.
 *                          Refer struct #MCAN_RxBufElement.
 *
 * \return  None.
 */
static void MCAN_readMsg(uint32_t           baseAddr,
                         uint32_t           elemAddr,
                         MCAN_RxBufElement *elem);

/**
 * \brief   This API will write the message object to Message RAM.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 * \param   elemAddr        Address of the message object.
 * \param   elem            Message Object.
 *                          Refer struct #MCAN_TxBufElement.
 *
 * \return  None.
 */
static void MCAN_writeMsg(uint32_t                 baseAddr,
                          uint32_t                 elemAddr,
                          const MCAN_TxBufElement *elem);

/**
 * \brief   This API will return payload depending on 'dlc'  field.
 *
 * \param   dlc             Data Length Code.
 *
 * \return  data size       Size of the payload.
 */
static uint32_t MCAN_getDataSize(uint32_t dlc);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void MCAN_selectClockSource(uint32_t baseAddr, MCAN_ClockSource source)
{
    //
    // Determine the CAN controller and set specified clock source
    //
    EALLOW;

    switch(baseAddr)
    {
        case MCANA_DRIVER_BASE:
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) = 
                    (HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) & 
                    ~SYSCTL_CLKSRCCTL2_MCANABITCLKSEL_M) |
                    ((uint16_t)source << SYSCTL_CLKSRCCTL2_MCANABITCLKSEL_S); 
            break;


        default:

            //
            // Do nothing. Not a valid mode value.
            //
            break;
    }

    EDIS;

}

uint32_t MCAN_isInReset(uint32_t baseAddr)
{
    uint32_t reset;
    uint32_t state;

    reset = HW_RD_FIELD32(baseAddr + MCAN_MCANSS_STAT,
                          MCAN_MCANSS_STAT_RESET);
    if(1U == reset)
    {
        state = (uint32_t) TRUE;
    }
    else
    {
        state = (uint32_t) FALSE;
    }
    return state;
}

uint32_t MCAN_isFDOpEnable(uint32_t baseAddr)
{
    uint32_t fdoe;
    uint32_t state;

    fdoe = HW_RD_FIELD32(baseAddr + MCAN_MCANSS_STAT,
                         MCAN_MCANSS_STAT_ENABLE_FDOE);
    if(1U == fdoe)
    {
        state = (uint32_t) TRUE;
    }
    else
    {
        state = (uint32_t) FALSE;
    }
    return state;
}

uint32_t MCAN_isMemInitDone(uint32_t baseAddr)
{
    uint32_t memInit;
    uint32_t state;

    memInit = HW_RD_FIELD32(baseAddr + MCAN_MCANSS_STAT,
                            MCAN_MCANSS_STAT_MEM_INIT_DONE);
    if(1U == memInit)
    {
        state = (uint32_t) TRUE;
    }
    else
    {
        state = (uint32_t) FALSE;
    }
    return state;
}

void MCAN_setOpMode(uint32_t baseAddr, uint32_t mode)
{
    HW_WR_FIELD32(baseAddr + MCAN_CCCR, MCAN_CCCR_INIT, mode);
}

uint32_t MCAN_getOpMode(uint32_t baseAddr)
{
    return(HW_RD_FIELD32(baseAddr + MCAN_CCCR, MCAN_CCCR_INIT));
}

int32_t MCAN_init(uint32_t baseAddr, const MCAN_InitParams *initParams)
{
    int32_t  status;
    uint32_t regVal;

    /* Configure MCAN wakeup and clock stop controls */
    regVal = HW_RD_REG32(baseAddr + MCAN_MCANSS_CTRL);
    HW_SET_FIELD32(regVal,
                   MCAN_MCANSS_CTRL_WAKEUPREQEN,
                   initParams->wkupReqEnable);
    HW_SET_FIELD32(regVal,
                   MCAN_MCANSS_CTRL_AUTOWAKEUP,
                   initParams->autoWkupEnable);
    HW_SET_FIELD32(regVal,
                   MCAN_MCANSS_CTRL_EMUEN,
                   initParams->emulationEnable);

    HW_WR_REG32(baseAddr + MCAN_MCANSS_CTRL, regVal);

    MCAN_writeProtectedRegAccessUnlock(baseAddr);

    /* Configure MCAN mode(FD vs Classic CAN operation) and controls */
    regVal = HW_RD_REG32(baseAddr + MCAN_CCCR);
    HW_SET_FIELD32(regVal,
                   MCAN_CCCR_FDOE,
                   initParams->fdMode);
    HW_SET_FIELD32(regVal,
                   MCAN_CCCR_BRSE,
                   initParams->brsEnable);
    HW_SET_FIELD32(regVal,
                   MCAN_CCCR_TXP,
                   initParams->txpEnable);
    HW_SET_FIELD32(regVal,
                   MCAN_CCCR_EFBI,
                   initParams->efbi);
    HW_SET_FIELD32(regVal,
                   MCAN_CCCR_PXHD,
                   initParams->pxhddisable);
    HW_SET_FIELD32(regVal,
                   MCAN_CCCR_DAR,
                   initParams->darEnable);
    HW_WR_REG32(baseAddr + MCAN_CCCR, regVal);

    if((MCAN_TDCR_TDCF_MAX >= initParams->tdcConfig.tdcf) &&
        (MCAN_TDCR_TDCO_MAX >= initParams->tdcConfig.tdco) &&
        (MCAN_RWD_WDC_MAX >= initParams->wdcPreload))
    {
        /* Configure Transceiver Delay Compensation */
        HW_WR_FIELD32(baseAddr + MCAN_TDCR,
                      MCAN_TDCR_TDCF,
                      initParams->tdcConfig.tdcf);
        HW_WR_FIELD32(baseAddr + MCAN_TDCR,
                      MCAN_TDCR_TDCO,
                      initParams->tdcConfig.tdco);
        /* Configure MSG RAM watchdog counter preload value */
        HW_WR_FIELD32(baseAddr + MCAN_RWD,
                      MCAN_RWD_WDC,
                      initParams->wdcPreload);
        /* Enable/Disable Transceiver Delay Compensation */
        HW_WR_FIELD32(baseAddr + MCAN_DBTP,
                      MCAN_DBTP_TDC,
                      initParams->tdcEnable);
        status = STW_SOK;
    }
    else
    {
        status = STW_EFAIL;
    }

    MCAN_writeProtectedRegAccessLock(baseAddr);

    return status;
}

int32_t MCAN_config(uint32_t baseAddr, const MCAN_ConfigParams *configParams)
{
    int32_t status;

    MCAN_writeProtectedRegAccessUnlock(baseAddr);

    /* Configure MCAN control registers */
    HW_WR_FIELD32(baseAddr + MCAN_CCCR,
                  MCAN_CCCR_MON,
                  configParams->monEnable);
    HW_WR_FIELD32(baseAddr + MCAN_CCCR,
                  MCAN_CCCR_ASM,
                  configParams->asmEnable);
    /* Configure Global Filter */
    HW_WR_FIELD32(baseAddr + MCAN_GFC,
                  MCAN_GFC_RRFE,
                  configParams->filterConfig.rrfe);
    HW_WR_FIELD32(baseAddr + MCAN_GFC,
                  MCAN_GFC_RRFS,
                  configParams->filterConfig.rrfs);
    HW_WR_FIELD32(baseAddr + MCAN_GFC,
                  MCAN_GFC_ANFE,
                  configParams->filterConfig.anfe);
    HW_WR_FIELD32(baseAddr + MCAN_GFC,
                  MCAN_GFC_ANFS,
                  configParams->filterConfig.anfs);

    if((MCAN_TSCC_TCP_MAX >= configParams->tsPrescalar) &&
        (MCAN_TOCC_TOP_MAX >= configParams->timeoutPreload))
    {
        /* Configure Time-stamp counter */
        HW_WR_FIELD32(baseAddr + MCAN_TSCC,
                      MCAN_TSCC_TSS,
                      configParams->tsSelect);
        HW_WR_FIELD32(baseAddr + MCAN_TSCC,
                      MCAN_TSCC_TCP,
                      (configParams->tsPrescalar - 1U));
        /* Configure Time-out counter */
        HW_WR_FIELD32(baseAddr + MCAN_TOCC,
                      MCAN_TOCC_TOS,
                      configParams->timeoutSelect);
        HW_WR_FIELD32(baseAddr + MCAN_TOCC,
                      MCAN_TOCC_TOP,
                      configParams->timeoutPreload);
        /* Enable Time-out counter */
        HW_WR_FIELD32(baseAddr + MCAN_TOCC,
                      MCAN_TOCC_ETOC,
                      configParams->timeoutCntEnable);
        status = STW_SOK;
    }
    else
    {
        status = STW_EFAIL;
    }

    MCAN_writeProtectedRegAccessLock(baseAddr);

    return status;
}

void MCAN_eccConfig(uint32_t                    baseAddr,
                    const MCAN_ECCConfigParams *configParams)
{
    MCAN_eccLoadRegister(baseAddr, MCAN_ECC_AGGR_CONTROL);
    HW_WR_FIELD32(baseAddr + MCAN_ECC_AGGR_CONTROL,
                  MCAN_ECC_AGGR_CONTROL_ECC_CHECK,
                  configParams->enableChk);
    MCAN_eccLoadRegister(baseAddr, MCAN_ECC_AGGR_CONTROL);
    HW_WR_FIELD32(baseAddr + MCAN_ECC_AGGR_CONTROL,
                  MCAN_ECC_AGGR_CONTROL_ECC_ENABLE,
                  configParams->enable);
    MCAN_eccLoadRegister(baseAddr, MCAN_ECC_AGGR_CONTROL);
    HW_WR_FIELD32(baseAddr + MCAN_ECC_AGGR_CONTROL,
                  MCAN_ECC_AGGR_CONTROL_ENABLE_RMW,
                  configParams->enableRdModWr);
    MCAN_eccLoadRegister(baseAddr, MCAN_ECC_AGGR_CONTROL);

}

int32_t MCAN_setBitTime(uint32_t                    baseAddr,
                        const MCAN_BitTimingParams *configParams)
{
    int32_t status;

    MCAN_writeProtectedRegAccessUnlock(baseAddr);

    if((MCAN_NBTP_NSJW_MAX >= configParams->nomSynchJumpWidth) &&
        (MCAN_NBTP_NTSEG2_MAX >= configParams->nomTimeSeg2) &&
        (MCAN_NBTP_NTSEG1_MAX >= configParams->nomTimeSeg1) &&
        (MCAN_NBTP_NBRP_MAX >= configParams->nomRatePrescalar))
    {
        HW_WR_FIELD32(baseAddr + MCAN_NBTP,
                      MCAN_NBTP_NSJW,
                      configParams->nomSynchJumpWidth);
        HW_WR_FIELD32(baseAddr + MCAN_NBTP,
                      MCAN_NBTP_NTSEG2,
                      configParams->nomTimeSeg2);
        HW_WR_FIELD32(baseAddr + MCAN_NBTP,
                      MCAN_NBTP_NTSEG1,
                      configParams->nomTimeSeg1);
        HW_WR_FIELD32(baseAddr + MCAN_NBTP,
                      MCAN_NBTP_NBRP,
                      configParams->nomRatePrescalar);
        status = STW_SOK;
    }
    else
    {
        status = STW_EFAIL;
    }
    if(STW_SOK == status)
    {
        if((MCAN_DBTP_DSJW_MAX >= configParams->dataSynchJumpWidth) &&
            (MCAN_DBTP_DTSEG2_MAX >= configParams->dataTimeSeg2) &&
            (MCAN_DBTP_DTSEG1_MAX >= configParams->dataTimeSeg1) &&
            (MCAN_DBTP_DBRP_MAX >= configParams->dataRatePrescalar))
        {
            HW_WR_FIELD32(baseAddr + MCAN_DBTP,
                          MCAN_DBTP_DSJW,
                          configParams->dataSynchJumpWidth);
            HW_WR_FIELD32(baseAddr + MCAN_DBTP,
                          MCAN_DBTP_DTSEG2,
                          configParams->dataTimeSeg2);
            HW_WR_FIELD32(baseAddr + MCAN_DBTP,
                          MCAN_DBTP_DTSEG1,
                          configParams->dataTimeSeg1);
            HW_WR_FIELD32(baseAddr + MCAN_DBTP,
                          MCAN_DBTP_DBRP,
                          configParams->dataRatePrescalar);
            status = STW_SOK;
        }
        else
        {
            status = STW_EFAIL;
        }
    }

    MCAN_writeProtectedRegAccessLock(baseAddr);
    return status;
}

int32_t MCAN_msgRAMConfig(uint32_t                       baseAddr,
                          const MCAN_MsgRAMConfigParams *msgRAMConfigParams)
{
    int32_t  status;
    uint32_t elemNum = 0U;

    MCAN_writeProtectedRegAccessUnlock(baseAddr);

    /* Configure Message Filters section */
    if(0U != msgRAMConfigParams->lss)
    {
        HW_WR_FIELD32(baseAddr + MCAN_SIDFC,
                      MCAN_SIDFC_FLSSA,
                      (msgRAMConfigParams->flssa >> 2U));
        HW_WR_FIELD32(baseAddr + MCAN_SIDFC,
                      MCAN_SIDFC_LSS,
                      msgRAMConfigParams->lss);
    }
    if(0U != msgRAMConfigParams->lse)
    {
        HW_WR_FIELD32(baseAddr + MCAN_XIDFC,
                      MCAN_XIDFC_FLESA,
                      (msgRAMConfigParams->flesa >> 2U));
        HW_WR_FIELD32(baseAddr + MCAN_XIDFC,
                      MCAN_XIDFC_LSE,
                      msgRAMConfigParams->lse);
    }
    /* Configure Rx FIFO 0 section */
    if(0U != msgRAMConfigParams->rxFIFO0size)
    {
        HW_WR_FIELD32(baseAddr + MCAN_RXF0C,
                      MCAN_RXF0C_F0SA,
                      (msgRAMConfigParams->rxFIFO0startAddr >> 2U));
        HW_WR_FIELD32(baseAddr + MCAN_RXF0C,
                      MCAN_RXF0C_F0S,
                      msgRAMConfigParams->rxFIFO0size);
        HW_WR_FIELD32(baseAddr + MCAN_RXF0C,
                      MCAN_RXF0C_F0WM,
                      msgRAMConfigParams->rxFIFO0waterMark);
        HW_WR_FIELD32(baseAddr + MCAN_RXF0C,
                      MCAN_RXF0C_F0OM,
                      msgRAMConfigParams->rxFIFO0OpMode);
        /* Configure Rx FIFO0 elements size */
        HW_WR_FIELD32(baseAddr + MCAN_RXESC,
                      MCAN_RXESC_F0DS,
                      msgRAMConfigParams->rxFIFO0ElemSize);
    }
    /* Configure Rx FIFO 1 section */
    if(0U != msgRAMConfigParams->rxFIFO1size)
    {
        HW_WR_FIELD32(baseAddr + MCAN_RXF1C,
                      MCAN_RXF1C_F1SA,
                      (msgRAMConfigParams->rxFIFO1startAddr >> 2U));
        HW_WR_FIELD32(baseAddr + MCAN_RXF1C,
                      MCAN_RXF1C_F1S,
                      msgRAMConfigParams->rxFIFO1size);
        HW_WR_FIELD32(baseAddr + MCAN_RXF1C,
                      MCAN_RXF1C_F1WM,
                      msgRAMConfigParams->rxFIFO1waterMark);
        HW_WR_FIELD32(baseAddr + MCAN_RXF1C,
                      MCAN_RXF1C_F1OM,
                      msgRAMConfigParams->rxFIFO1OpMode);
        /* Configure Rx FIFO1 elements size */
        HW_WR_FIELD32(baseAddr + MCAN_RXESC,
                      MCAN_RXESC_F1DS,
                      msgRAMConfigParams->rxFIFO1ElemSize);
    }
    /* Configure Rx Buffer Start Address */
    HW_WR_FIELD32(baseAddr + MCAN_RXBC,
                  MCAN_RXBC_RBSA,
                  (msgRAMConfigParams->rxBufStartAddr >> 2U));
    /* Configure Rx Buffer elements size */
    HW_WR_FIELD32(baseAddr + MCAN_RXESC,
                  MCAN_RXESC_RBDS,
                  msgRAMConfigParams->rxBufElemSize);
    /* Configure Tx Event FIFO section */
    if(0U != msgRAMConfigParams->txEventFIFOSize)
    {
        HW_WR_FIELD32(baseAddr + MCAN_TXEFC,
                      MCAN_TXEFC_EFSA,
                      (msgRAMConfigParams->txEventFIFOStartAddr >> 2U));
        HW_WR_FIELD32(baseAddr + MCAN_TXEFC,
                      MCAN_TXEFC_EFS,
                      msgRAMConfigParams->txEventFIFOSize);
        HW_WR_FIELD32(baseAddr + MCAN_TXEFC,
                      MCAN_TXEFC_EFWM,
                      msgRAMConfigParams->txEventFIFOWaterMark);
    }
    /* Configure Tx Buffer and FIFO/Q section */
    elemNum = msgRAMConfigParams->txBufNum + msgRAMConfigParams->txFIFOSize;
    if((MCANSS_TX_BUFFER_MAX >= elemNum) &&
        ((0U != msgRAMConfigParams->txBufNum) ||
         (0U != msgRAMConfigParams->txFIFOSize)))
    {
        HW_WR_FIELD32(baseAddr + MCAN_TXBC,
                      MCAN_TXBC_TBSA,
                      (msgRAMConfigParams->txStartAddr >> 2U));
        HW_WR_FIELD32(baseAddr + MCAN_TXBC,
                      MCAN_TXBC_NDTB,
                      msgRAMConfigParams->txBufNum);
        HW_WR_FIELD32(baseAddr + MCAN_TXBC,
                      MCAN_TXBC_TFQS,
                      msgRAMConfigParams->txFIFOSize);
        HW_WR_FIELD32(baseAddr + MCAN_TXBC,
                      MCAN_TXBC_TFQM,
                      msgRAMConfigParams->txBufMode);
        /* Configure Tx Buffer/FIFO0/FIFO1 elements size */
        HW_WR_FIELD32(baseAddr + MCAN_TXESC,
                      MCAN_TXESC_TBDS,
                      msgRAMConfigParams->txBufElemSize);
        status = STW_SOK;
    }
    else
    {
        status = STW_EFAIL;
    }

    MCAN_writeProtectedRegAccessLock(baseAddr);

    return status;
}

int32_t MCAN_setExtIDAndMask(uint32_t baseAddr, uint32_t idMask)
{
    int32_t status;

    if(MCAN_XIDAM_EIDM_MAX >= idMask)
    {
        MCAN_writeProtectedRegAccessUnlock(baseAddr);

        HW_WR_FIELD32(baseAddr + MCAN_XIDAM,
                      MCAN_XIDAM_EIDM,
                      idMask);

        MCAN_writeProtectedRegAccessLock(baseAddr);
        status = STW_SOK;
    }
    else
    {
        status = STW_EFAIL;
    }
    return status;
}

void MCAN_writeMsgRam(uint32_t                 baseAddr,
                      uint32_t                 memType,
                      uint32_t                 bufNum,
                      const MCAN_TxBufElement *elem)
{
    uint32_t startAddr = 0U, elemSize = 0U, elemAddr = 0U;
    uint32_t idx       = 0U, enableMod = 0U;

    if((uint32_t)MCAN_MEM_TYPE_BUF == memType)
    {
        idx       = bufNum;
        enableMod = 1U;
    }
    if((uint32_t)MCAN_MEM_TYPE_FIFO == memType)
    {
        idx       = HW_RD_FIELD32(baseAddr + MCAN_TXFQS, MCAN_TXFQS_TFQPI);
        enableMod = 1U;
    }
    if(1U == enableMod)
    {
        startAddr = HW_RD_FIELD32(baseAddr + MCAN_TXBC,
                                  MCAN_TXBC_TBSA);
        elemSize = HW_RD_FIELD32(baseAddr + MCAN_TXESC,
                                 MCAN_TXESC_TBDS);
        startAddr = (uint32_t) (startAddr << 2U);
        elemSize  = MCAN_getMsgObjSize(elemSize);
        elemSize *= 4U;
        elemAddr  = startAddr + (elemSize * idx);
        elemAddr += MCAN_MCAN_MSG_MEM;
        MCAN_writeMsg(baseAddr, elemAddr, elem);
    }
}

int32_t MCAN_txBufAddReq(uint32_t baseAddr, uint32_t bufNum)
{
    int32_t  status;
    uint32_t regVal;

    if(MCANSS_TX_BUFFER_MAX > bufNum)
    {
        regVal  = HW_RD_REG32(baseAddr + MCAN_TXBAR);
        regVal |= ((uint32_t) 1U << bufNum);

        // For writing to TXBAR CCE bit should be '0'. This need not be
        // reverted because for other qualified writes this is locked state
        // and can't be written.
        MCAN_writeProtectedRegAccessLock(baseAddr);
        HW_WR_REG32(baseAddr + MCAN_TXBAR, regVal);

        status = STW_SOK;
    }
    else
    {
        status = STW_EFAIL;
    }
    return status;
}

void  MCAN_getNewDataStatus(uint32_t              baseAddr,
                            MCAN_RxNewDataStatus *newDataStatus)
{
    newDataStatus->statusLow  = HW_RD_REG32(baseAddr + MCAN_NDAT1);
    newDataStatus->statusHigh = HW_RD_REG32(baseAddr + MCAN_NDAT2);
}

void  MCAN_clearNewDataStatus(uint32_t                    baseAddr,
                              const MCAN_RxNewDataStatus *newDataStatus)
{
    HW_WR_REG32(baseAddr + MCAN_NDAT1, newDataStatus->statusLow);
    HW_WR_REG32(baseAddr + MCAN_NDAT2, newDataStatus->statusHigh);
}

void MCAN_readMsgRam(uint32_t           baseAddr,
                     uint32_t           memType,
                     uint32_t           bufNum,
                     uint32_t           fifoNum,
                     MCAN_RxBufElement *elem)
{
    uint32_t startAddr = 0U, elemSize = 0U, elemAddr = 0U;
    uint32_t enableMod = 0U, idx = 0U;

    if((uint32_t)MCAN_MEM_TYPE_BUF == memType)
    {
        startAddr = HW_RD_FIELD32(baseAddr + MCAN_RXBC,
                                  MCAN_RXBC_RBSA);
        elemSize = HW_RD_FIELD32(baseAddr + MCAN_RXESC,
                                 MCAN_RXESC_RBDS);
        idx       = bufNum;
        enableMod = 1U;
    }
    if((uint32_t)MCAN_MEM_TYPE_FIFO == memType)
    {
        switch (fifoNum)
        {
            case MCAN_RX_FIFO_NUM_0:
                startAddr = HW_RD_FIELD32(baseAddr + MCAN_RXF0C,
                                          MCAN_RXF0C_F0SA);
                elemSize = HW_RD_FIELD32(baseAddr + MCAN_RXESC,
                                         MCAN_RXESC_F0DS);
                idx = HW_RD_FIELD32(baseAddr + MCAN_RXF0S,
                                    MCAN_RXF0S_F0GI);
                enableMod = 1U;
                break;
            case MCAN_RX_FIFO_NUM_1:
                startAddr = HW_RD_FIELD32(baseAddr + MCAN_RXF1C,
                                          MCAN_RXF1C_F1SA);
                elemSize = HW_RD_FIELD32(baseAddr + MCAN_RXESC,
                                         MCAN_RXESC_F1DS);
                idx = HW_RD_FIELD32(baseAddr + MCAN_RXF1S,
                                    MCAN_RXF1S_F1GI);
                enableMod = 1U;
                break;
            default:
                /* Invalid option */
                break;
        }
    }
    if(1U == enableMod)
    {
        startAddr = (uint32_t) (startAddr << 2U);
        elemSize  = MCAN_getMsgObjSize(elemSize);
        elemSize *= 4U;
        elemAddr  = startAddr + (elemSize * idx);
        elemAddr += MCAN_MCAN_MSG_MEM;
        MCAN_readMsg(baseAddr, elemAddr, elem);
    }
}

void MCAN_readTxEventFIFO(uint32_t           baseAddr,
                          MCAN_TxEventFIFOElement *txEventElem)
{
    uint32_t startAddr = 0U, elemSize = 0U, elemAddr = 0U;
    uint32_t idx = 0U, regVal;


    startAddr = HW_RD_FIELD32(baseAddr + MCAN_TXEFC,
                              MCAN_TXEFC_EFSA);
    elemSize = MCANSS_TX_EVENT_FIFO_SIZE_WORDS;
    idx = HW_RD_FIELD32(baseAddr + MCAN_TXEFS,
                        MCAN_TXEFS_EFGI);

    startAddr = (uint32_t) (startAddr << 2U);
    elemSize *= 4U;
    elemAddr  = startAddr + (elemSize * idx);
    elemAddr += MCAN_MCAN_MSG_MEM;

    regVal   = HW_RD_REG32(baseAddr + elemAddr);
    txEventElem->id = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_ID_MASK)
                           >> MCANSS_TX_EVENT_FIFO_ELEM_ID_SHIFT);
    txEventElem->rtr = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_RTR_MASK)
                            >> MCANSS_TX_EVENT_FIFO_ELEM_RTR_SHIFT);
    txEventElem->xtd = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_XTD_MASK)
                            >> MCANSS_TX_EVENT_FIFO_ELEM_XTD_SHIFT);
    txEventElem->esi = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_ESI_MASK)
                            >> MCANSS_TX_EVENT_FIFO_ELEM_ESI_SHIFT);
    elemAddr  += 4U;
    regVal     = HW_RD_REG32(baseAddr + elemAddr);
    txEventElem->txts = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_TXTS_MASK)
                             >> MCANSS_TX_EVENT_FIFO_ELEM_TXTS_SHIFT);
    txEventElem->dlc = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_DLC_MASK)
                            >> MCANSS_TX_EVENT_FIFO_ELEM_DLC_SHIFT);
    txEventElem->brs = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_BRS_MASK)
                            >> MCANSS_TX_EVENT_FIFO_ELEM_BRS_SHIFT);
    txEventElem->fdf = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_FDF_MASK)
                            >> MCANSS_TX_EVENT_FIFO_ELEM_FDF_SHIFT);
    txEventElem->et = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_ET_MASK)
                             >> MCANSS_TX_EVENT_FIFO_ELEM_ET_SHIFT);
    txEventElem->mm = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_MM_MASK)
                             >> MCANSS_TX_EVENT_FIFO_ELEM_MM_SHIFT);
}

void MCAN_addStdMsgIDFilter(uint32_t                          baseAddr,
                            uint32_t                          filtNum,
                            const MCAN_StdMsgIDFilterElement *elem)
{
    uint32_t startAddr, elemAddr, regVal;

    startAddr = HW_RD_FIELD32(baseAddr + MCAN_SIDFC,
                              MCAN_SIDFC_FLSSA);
    startAddr = (uint32_t) (startAddr << 2U);
    elemAddr  = startAddr + (filtNum * MCANSS_STD_ID_FILTER_SIZE_WORDS * 4U);
    elemAddr += MCAN_MCAN_MSG_MEM;

    regVal  = 0U;
    regVal |= (uint32_t) (elem->sfid2 << MCANSS_STD_ID_FILTER_SFID2_SHIFT);
    regVal |= (uint32_t) (elem->sfid1 << MCANSS_STD_ID_FILTER_SFID1_SHIFT);
    regVal |= (uint32_t) (elem->sfec << MCANSS_STD_ID_FILTER_SFEC_SHIFT);
    regVal |= (uint32_t) (elem->sft << MCANSS_STD_ID_FILTER_SFT_SHIFT);
    HW_WR_REG32(baseAddr + elemAddr, regVal);
}

void MCAN_addExtMsgIDFilter(uint32_t                          baseAddr,
                            uint32_t                          filtNum,
                            const MCAN_ExtMsgIDFilterElement *elem)
{
    uint32_t startAddr, elemAddr, regVal;

    startAddr = HW_RD_FIELD32(baseAddr + MCAN_XIDFC,
                              MCAN_XIDFC_FLESA);
    startAddr = (uint32_t) (startAddr << 2U);
    elemAddr  = startAddr + (filtNum * MCANSS_EXT_ID_FILTER_SIZE_WORDS * 4U);
    elemAddr += MCAN_MCAN_MSG_MEM;

    regVal  = 0U;
    regVal |= (uint32_t) (elem->efid1 << MCANSS_EXT_ID_FILTER_EFID1_SHIFT);
    regVal |= (uint32_t) (elem->efec << MCANSS_EXT_ID_FILTER_EFEC_SHIFT);
    HW_WR_REG32(baseAddr + elemAddr, regVal);

    elemAddr += 4U;
    regVal    = 0U;
    regVal   |= (uint32_t) (elem->efid2 << MCANSS_EXT_ID_FILTER_EFID2_SHIFT);
    regVal   |= (uint32_t) (elem->eft << MCANSS_EXT_ID_FILTER_EFT_SHIFT);
    HW_WR_REG32(baseAddr + elemAddr, regVal);
}

void MCAN_lpbkModeEnable(uint32_t baseAddr,
                         uint32_t lpbkMode,
                         uint32_t enable)
{
    MCAN_writeProtectedRegAccessUnlock(baseAddr);

    if(TRUE == enable)
    {
        HW_WR_FIELD32(baseAddr + MCAN_CCCR, MCAN_CCCR_TEST, 0x1U);
        HW_WR_FIELD32(baseAddr + MCAN_TEST,
                      MCAN_TEST_LBCK,
                      enable);
        if((uint32_t)MCAN_LPBK_MODE_INTERNAL == lpbkMode)
        {
            HW_WR_FIELD32(baseAddr + MCAN_CCCR,
                          MCAN_CCCR_MON,
                          0x1U);
        }
    }
    else
    {
        HW_WR_FIELD32(baseAddr + MCAN_TEST,
                      MCAN_TEST_LBCK,
                      enable);
        HW_WR_FIELD32(baseAddr + MCAN_CCCR, MCAN_CCCR_TEST, 0x0U);
        if((uint32_t)MCAN_LPBK_MODE_INTERNAL == lpbkMode)
        {
            HW_WR_FIELD32(baseAddr + MCAN_CCCR,
                          MCAN_CCCR_MON,
                          0x0U);
        }
    }
    MCAN_writeProtectedRegAccessLock(baseAddr);
}

void  MCAN_getErrCounters(uint32_t           baseAddr,
                          MCAN_ErrCntStatus *errCounter)
{
    errCounter->transErrLogCnt = HW_RD_FIELD32(baseAddr + MCAN_ECR,
                                               MCAN_ECR_TEC);
    errCounter->recErrCnt = HW_RD_FIELD32(baseAddr + MCAN_ECR,
                                          MCAN_ECR_REC);
    errCounter->rpStatus = HW_RD_FIELD32(baseAddr + MCAN_ECR,
                                         MCAN_ECR_RP);
    errCounter->canErrLogCnt = HW_RD_FIELD32(baseAddr + MCAN_ECR,
                                             MCAN_ECR_CEL);
}

void  MCAN_getProtocolStatus(uint32_t             baseAddr,
                             MCAN_ProtocolStatus *protStatus)
{
    uint32_t regVal;

    regVal = HW_RD_REG32(baseAddr + MCAN_PSR);
    protStatus->lastErrCode   = HW_GET_FIELD(regVal, MCAN_PSR_LEC);
    protStatus->act           = HW_GET_FIELD(regVal, MCAN_PSR_ACT);
    protStatus->errPassive    = HW_GET_FIELD(regVal, MCAN_PSR_EP);
    protStatus->warningStatus = HW_GET_FIELD(regVal, MCAN_PSR_EW);
    protStatus->busOffStatus  = HW_GET_FIELD(regVal, MCAN_PSR_BO);
    protStatus->dlec          = HW_GET_FIELD(regVal, MCAN_PSR_DLEC);
    protStatus->resi          = HW_GET_FIELD(regVal, MCAN_PSR_RESI);
    protStatus->rbrs          = HW_GET_FIELD(regVal, MCAN_PSR_RBRS);
    protStatus->rfdf          = HW_GET_FIELD(regVal, MCAN_PSR_RFDF);
    protStatus->pxe           = HW_GET_FIELD(regVal, MCAN_PSR_PXE);
    protStatus->tdcv          = HW_GET_FIELD(regVal, MCAN_PSR_TDCV);
}

void MCAN_enableIntr(uint32_t baseAddr, uint32_t intrMask, uint32_t enable)
{
    uint32_t regVal;

    if(TRUE == enable)
    {
        regVal  = HW_RD_REG32(baseAddr + MCAN_IE);
        regVal |= intrMask;
        HW_WR_REG32(baseAddr + MCAN_IE, regVal);
    }
    else
    {
        regVal  = HW_RD_REG32(baseAddr + MCAN_IE);
        regVal &= ~intrMask;
        HW_WR_REG32(baseAddr + MCAN_IE, regVal);
    }
}

void MCAN_selectIntrLine(uint32_t baseAddr,
                         uint32_t intrMask,
                         uint32_t lineNum)
{
    uint32_t regVal;

    if((uint32_t)MCAN_INTR_LINE_NUM_0 == lineNum)
    {
        regVal  = HW_RD_REG32(baseAddr + MCAN_ILS);
        regVal &= ~intrMask;
        HW_WR_REG32(baseAddr + MCAN_ILS, regVal);
    }
    else
    {
        regVal  = HW_RD_REG32(baseAddr + MCAN_ILS);
        regVal |= intrMask;
        HW_WR_REG32(baseAddr + MCAN_ILS, regVal);
    }
}

uint32_t MCAN_getIntrLineSelectStatus(uint32_t baseAddr)
{
    return(HW_RD_REG32(baseAddr + MCAN_ILS));
}

void MCAN_enableIntrLine(uint32_t baseAddr,
                         uint32_t lineNum,
                         uint32_t enable)
{
    uint32_t regVal;

    lineNum &= MCANSS_INTR_LINE_EN_MASK;
    regVal   = HW_RD_REG32(baseAddr + MCAN_ILE);
    regVal  &= ~((uint32_t) 0x1U << lineNum);
    regVal  |= (uint32_t) (enable << lineNum);
    HW_WR_REG32(baseAddr + MCAN_ILE, regVal);
}

uint32_t MCAN_getIntrStatus(uint32_t baseAddr)
{
    return(HW_RD_REG32(baseAddr + MCAN_IR));
}

void MCAN_clearIntrStatus(uint32_t baseAddr, uint32_t intrMask)
{
    HW_WR_REG32(baseAddr + MCAN_IR, intrMask);
}

void MCAN_clearInterrupt(uint32_t baseAddr, uint16_t intrNum)
{
    HW_WR_FIELD32(baseAddr + MCAN_MCANSS_EOI, MCAN_MCANSS_EOI, intrNum);
}

void  MCAN_getHighPriorityMsgStatus(uint32_t                  baseAddr,
                                    MCAN_HighPriorityMsgInfo *hpm)
{
    hpm->bufIdx = HW_RD_FIELD32(baseAddr + MCAN_HPMS,
                                MCAN_HPMS_BIDX);
    hpm->msi = HW_RD_FIELD32(baseAddr + MCAN_HPMS,
                             MCAN_HPMS_MSI);
    hpm->filterIdx = HW_RD_FIELD32(baseAddr + MCAN_HPMS,
                                   MCAN_HPMS_FIDX);
    hpm->filterList = HW_RD_FIELD32(baseAddr + MCAN_HPMS,
                                    MCAN_HPMS_FLST);
}

void MCAN_getRxFIFOStatus(uint32_t           baseAddr,
                          MCAN_RxFIFOStatus *fifoStatus)
{
    uint32_t regVal;

    switch (fifoStatus->num)
    {
        case MCAN_RX_FIFO_NUM_0:
            regVal = HW_RD_REG32(baseAddr + MCAN_RXF0S);
            fifoStatus->fillLvl  = HW_GET_FIELD(regVal, MCAN_RXF0S_F0FL);
            fifoStatus->getIdx   = HW_GET_FIELD(regVal, MCAN_RXF0S_F0GI);
            fifoStatus->putIdx   = HW_GET_FIELD(regVal, MCAN_RXF0S_F0PI);
            fifoStatus->fifoFull = HW_GET_FIELD(regVal, MCAN_RXF0S_F0F);
            fifoStatus->msgLost  = HW_GET_FIELD(regVal, MCAN_RXF0S_RF0L);
            break;
        case MCAN_RX_FIFO_NUM_1:
            regVal = HW_RD_REG32(baseAddr + MCAN_RXF1S);
            fifoStatus->fillLvl  = HW_GET_FIELD(regVal, MCAN_RXF1S_F1FL);
            fifoStatus->getIdx   = HW_GET_FIELD(regVal, MCAN_RXF1S_F1GI);
            fifoStatus->putIdx   = HW_GET_FIELD(regVal, MCAN_RXF1S_F1PI);
            fifoStatus->fifoFull = HW_GET_FIELD(regVal, MCAN_RXF1S_F1F);
            fifoStatus->msgLost  = HW_GET_FIELD(regVal, MCAN_RXF1S_RF1L);
            break;
        default:
            /* Invalid option */
            break;
    }
}

int32_t MCAN_writeRxFIFOAck(uint32_t baseAddr,
                            uint32_t fifoNum,
                            uint32_t idx)
{
    int32_t  status;
    uint32_t size;

    switch (fifoNum)
    {
        case MCAN_RX_FIFO_NUM_0:
            size = HW_RD_FIELD32(baseAddr + MCAN_RXF0C,
                                 MCAN_RXF0C_F0S);
            if(size >= idx)
            {
                HW_WR_FIELD32(baseAddr + MCAN_RXF0A,
                              MCAN_RXF0A_F0AI,
                              idx);
                status = STW_SOK;
            }
            else
            {
                status = STW_EFAIL;
            }
            break;
        case MCAN_RX_FIFO_NUM_1:
            size = HW_RD_FIELD32(baseAddr + MCAN_RXF1C,
                                 MCAN_RXF1C_F1S);
            if(size >= idx)
            {
                HW_WR_FIELD32(baseAddr + MCAN_RXF1A,
                              MCAN_RXF1A_F1AI,
                              idx);
                status = STW_SOK;
            }
            else
            {
                status = STW_EFAIL;
            }
            break;
        default:
            status = STW_EFAIL;
            break;
    }

    return status;
}

void MCAN_getTxFIFOQueStatus(uint32_t           baseAddr,
                             MCAN_TxFIFOStatus *fifoStatus)
{
    uint32_t regVal;

    regVal = HW_RD_REG32(baseAddr + MCAN_TXFQS);
    fifoStatus->freeLvl  = HW_GET_FIELD(regVal, MCAN_TXFQS_TFFL);
    fifoStatus->getIdx   = HW_GET_FIELD(regVal, MCAN_TXFQS_TFGI);
    fifoStatus->putIdx   = HW_GET_FIELD(regVal, MCAN_TXFQS_TFQPI);
    fifoStatus->fifoFull = HW_GET_FIELD(regVal, MCAN_TXFQS_TFQF);
}

uint32_t MCAN_getTxBufReqPend(uint32_t baseAddr)
{
    return(HW_RD_REG32(baseAddr + MCAN_TXBRP));
}

int32_t MCAN_txBufCancellationReq(uint32_t baseAddr, uint32_t buffNum)
{
    int32_t  status;
    uint32_t regVal;

    if(MCANSS_TX_BUFFER_MAX > buffNum)
    {
        regVal  = HW_RD_REG32(baseAddr + MCAN_TXBCR);
        regVal |= ((uint32_t) 1U << buffNum);

        // For writing to TXBCR CCE bit should be '0'. This need not be
        // reverted because for other qualified writes this is locked state
        // and can't be written.
        MCAN_writeProtectedRegAccessLock(baseAddr);
        HW_WR_REG32(baseAddr + MCAN_TXBCR, regVal);

        status = STW_SOK;
    }
    else
    {
        status = STW_EFAIL;
    }
    return status;
}

uint32_t MCAN_getTxBufTransmissionStatus(uint32_t baseAddr)
{
    return(HW_RD_REG32(baseAddr + MCAN_TXBTO));
}

uint32_t MCAN_txBufCancellationStatus(uint32_t baseAddr)
{
    return(HW_RD_REG32(baseAddr + MCAN_TXBCF));
}

int32_t MCAN_txBufTransIntrEnable(uint32_t baseAddr,
                                  uint32_t bufNum,
                                  uint32_t enable)
{
    int32_t  status;
    uint32_t regVal;

    if(MCANSS_TX_BUFFER_MAX > bufNum)
    {
        if(TRUE == enable)
        {
            regVal  = HW_RD_REG32(baseAddr + MCAN_TXBTIE);
            regVal |= ((uint32_t) 1U << bufNum);
            HW_WR_REG32(baseAddr + MCAN_TXBTIE, regVal);
        }
        else
        {
            regVal  = HW_RD_REG32(baseAddr + MCAN_TXBTIE);
            regVal &= ~((uint32_t) 0x1U << bufNum);
            HW_WR_REG32(baseAddr + MCAN_TXBTIE, regVal);
        }
        status = STW_SOK;
    }
    else
    {
        status = STW_EFAIL;
    }
    return status;
}

int32_t MCAN_getTxBufCancellationIntrEnable(uint32_t baseAddr,
                                            uint32_t bufNum,
                                            uint32_t enable)
{
    int32_t  status;
    uint32_t regVal;

    if(MCANSS_TX_BUFFER_MAX > bufNum)
    {
        if(TRUE == enable)
        {
            regVal  = HW_RD_REG32(baseAddr + MCAN_TXBCIE);
            regVal |= ((uint32_t) 0x1U << bufNum);
            HW_WR_REG32(baseAddr + MCAN_TXBCIE, regVal);
        }
        else
        {
            regVal  = HW_RD_REG32(baseAddr + MCAN_TXBCIE);
            regVal &= ~((uint32_t) 0x1U << bufNum);
            HW_WR_REG32(baseAddr + MCAN_TXBCIE, regVal);
        }
        status = STW_SOK;
    }
    else
    {
        status = STW_EFAIL;
    }
    return status;
}

void MCAN_getTxEventFIFOStatus(uint32_t                baseAddr,
                               MCAN_TxEventFIFOStatus *fifoStatus)
{
    uint32_t regVal;

    regVal = HW_RD_REG32(baseAddr + MCAN_TXEFS);
    fifoStatus->fillLvl  = HW_GET_FIELD(regVal, MCAN_TXEFS_EFFL);
    fifoStatus->getIdx   = HW_GET_FIELD(regVal, MCAN_TXEFS_EFGI);
    fifoStatus->putIdx   = HW_GET_FIELD(regVal, MCAN_TXEFS_EFPI);
    fifoStatus->fifoFull = HW_GET_FIELD(regVal, MCAN_TXEFS_EFF);
    fifoStatus->eleLost  = HW_GET_FIELD(regVal, MCAN_TXEFS_TEFL);
}

void MCAN_addClockStopRequest(uint32_t baseAddr, uint32_t enable)
{
    if(TRUE == enable)
    {
        HW_WR_FIELD32(baseAddr + MCAN_CCCR, MCAN_CCCR_CSR, 0x1U);
    }
    else
    {
        HW_WR_FIELD32(baseAddr + MCAN_CCCR, MCAN_CCCR_CSR, 0x0U);
    }
}

int32_t MCAN_writeTxEventFIFOAck(uint32_t baseAddr, uint32_t idx)
{
    int32_t  status;
    uint32_t size;

    size = HW_RD_FIELD32(baseAddr + MCAN_TXEFC,
                         MCAN_TXEFC_EFS);
    if(size >= idx)
    {
        HW_WR_FIELD32(baseAddr + MCAN_TXEFA,
                      MCAN_TXEFA_EFAI,
                      idx);
        status = STW_SOK;
    }
    else
    {
        status = STW_EFAIL;
    }

    return status;
}

void MCAN_eccForceError(uint32_t                      baseAddr,
                        const MCAN_ECCErrForceParams *eccErr)
{
    uint32_t regVal;

    if((eccErr->errType == (uint32_t)MCAN_ECC_ERR_TYPE_SEC) ||
       (eccErr->errType == (uint32_t)MCAN_ECC_ERR_TYPE_DED))
    {
        MCAN_eccLoadRegister(baseAddr, MCAN_ECC_AGGR_ERROR_CTRL1);
        regVal = HW_RD_REG32(baseAddr + MCAN_ECC_AGGR_ERROR_CTRL1);
        HW_SET_FIELD32(regVal,
                       MCAN_ECC_AGGR_ERROR_CTRL1_ECC_ROW,
                       eccErr->rowNum);
        HW_WR_REG32(baseAddr + MCAN_ECC_AGGR_ERROR_CTRL1, regVal);
        MCAN_eccLoadRegister(baseAddr, MCAN_ECC_AGGR_ERROR_CTRL2);
        regVal = HW_RD_REG32(baseAddr + MCAN_ECC_AGGR_ERROR_CTRL2);
        HW_SET_FIELD32(regVal,
                       MCAN_ECC_AGGR_ERROR_CTRL2_ECC_BIT1,
                       eccErr->bit1);
        HW_SET_FIELD32(regVal,
                       MCAN_ECC_AGGR_ERROR_CTRL2_ECC_BIT2,
                       eccErr->bit2);
        HW_WR_REG32(baseAddr + MCAN_ECC_AGGR_ERROR_CTRL2, regVal);
        MCAN_eccLoadRegister(baseAddr, MCAN_ECC_AGGR_CONTROL);
        regVal = HW_RD_REG32(baseAddr + MCAN_ECC_AGGR_CONTROL);
        HW_SET_FIELD32(regVal,
                       MCAN_ECC_AGGR_CONTROL_FORCE_N_ROW,
                       eccErr->errForce);
        HW_SET_FIELD32(regVal,
                       MCAN_ECC_AGGR_CONTROL_ERROR_ONCE,
                       eccErr->errOnce);
        if(eccErr->errType == (uint32_t)MCAN_ECC_ERR_TYPE_SEC)
        {
            HW_SET_FIELD32(regVal,
                           MCAN_ECC_AGGR_CONTROL_FORCE_SEC,
                           0x1U);
        }
        else if(eccErr->errType == (uint32_t)MCAN_ECC_ERR_TYPE_DED)
        {
            HW_SET_FIELD32(regVal,
                           MCAN_ECC_AGGR_CONTROL_FORCE_DED,
                           0x1U);
        }
        else
        {
            /* MISRA C Compliance */
        }
        HW_WR_REG32(baseAddr + MCAN_ECC_AGGR_CONTROL, regVal);
        MCAN_eccLoadRegister(baseAddr, MCAN_ECC_AGGR_CONTROL);
    }
}


void MCAN_eccGetErrorStatus(uint32_t           baseAddr,
                            MCAN_ECCErrStatus *eccErr)
{
    uint32_t regVal;

    MCAN_eccLoadRegister(baseAddr, MCAN_ECC_AGGR_ERROR_STATUS1);
    regVal = HW_RD_REG32(baseAddr + MCAN_ECC_AGGR_ERROR_STATUS1);
    eccErr->secErr = HW_GET_FIELD(regVal,
                                  MCAN_ECC_AGGR_ERROR_STATUS1_ECC_SEC);
    eccErr->dedErr = HW_GET_FIELD(regVal,
                                  MCAN_ECC_AGGR_ERROR_STATUS1_ECC_DED);
    eccErr->bit1 = HW_GET_FIELD(regVal,
                                MCAN_ECC_AGGR_ERROR_STATUS1_ECC_BIT1);
    MCAN_eccLoadRegister(baseAddr, MCAN_ECC_AGGR_ERROR_STATUS2);
    regVal = HW_RD_REG32(baseAddr + MCAN_ECC_AGGR_ERROR_STATUS2);
    eccErr->row = HW_GET_FIELD(regVal,
                               MCAN_ECC_AGGR_ERROR_STATUS2_ECC_ROW);
}

void MCAN_eccClearErrorStatus(uint32_t baseAddr, uint32_t errType)
{
    MCAN_eccLoadRegister(baseAddr, MCAN_ECC_AGGR_ERROR_STATUS1);
    switch (errType)
    {
        case MCAN_ECC_ERR_TYPE_SEC:
            HW_WR_FIELD32(baseAddr + MCAN_ECC_AGGR_ERROR_STATUS1,
                          MCAN_ECC_AGGR_ERROR_STATUS1_CLR_ECC_SEC,
                          0x1U);
            break;
        case MCAN_ECC_ERR_TYPE_DED:
            HW_WR_FIELD32(baseAddr + MCAN_ECC_AGGR_ERROR_STATUS1,
                          MCAN_ECC_AGGR_ERROR_STATUS1_CLR_ECC_DED,
                          0x1U);
            break;
        default:
            /* Invalid option */
            break;
    }
    MCAN_eccLoadRegister(baseAddr, MCAN_ECC_AGGR_ERROR_STATUS1);
}

void MCAN_eccWriteEOI(uint32_t baseAddr, uint32_t errType)
{
    switch (errType)
    {
        case MCAN_ECC_ERR_TYPE_SEC:
            HW_WR_FIELD32(baseAddr + MCAN_ECC_AGGR_SEC_EOI_REG,
                          MCAN_ECC_AGGR_SEC_EOI_REG_WR,
                          0x1U);
            break;
        case MCAN_ECC_ERR_TYPE_DED:
            HW_WR_FIELD32(baseAddr + MCAN_ECC_AGGR_DED_EOI_REG,
                          MCAN_ECC_AGGR_DED_EOI_REG_WR,
                          0x1U);
            break;
        default:
            /* Invalid option */
            break;
    }
}

void MCAN_eccEnableIntr(uint32_t baseAddr, uint32_t errType, uint32_t enable)
{
    if(TRUE == enable)
    {
        switch (errType)
        {
            case MCAN_ECC_ERR_TYPE_SEC:
                HW_WR_FIELD32(baseAddr + MCAN_ECC_AGGR_SEC_ENABLE_SET_REG0,
                              MCAN_ECC_AGGR_SEC_ENABLE_SET_REG0_MSGMEM,
                              0x1U);
                break;
            case MCAN_ECC_ERR_TYPE_DED:
                HW_WR_FIELD32(baseAddr + MCAN_ECC_AGGR_DED_ENABLE_SET_REG0,
                              MCAN_ECC_AGGR_DED_ENABLE_SET_REG0_MSGMEM,
                              0x1U);
                break;
            default:
                /* Invalid option */
                break;
        }
    }
    else
    {
        switch (errType)
        {
            case MCAN_ECC_ERR_TYPE_SEC:
                HW_WR_FIELD32(baseAddr + MCAN_ECC_AGGR_SEC_ENABLE_CLR_REG0,
                              MCAN_ECC_AGGR_SEC_ENABLE_CLR_REG0_MSGMEM,
                              0x1U);
                break;
            case MCAN_ECC_ERR_TYPE_DED:
                HW_WR_FIELD32(baseAddr + MCAN_ECC_AGGR_DED_ENABLE_CLR_REG0,
                              MCAN_ECC_AGGR_DED_ENABLE_CLR_REG0_MSGMEM,
                              0x1U);
                break;
            default:
                /* Invalid option */
                break;
        }
    }
}

uint32_t MCAN_eccGetIntrStatus(uint32_t baseAddr, uint32_t errType)
{
    uint32_t retVal = 0U;

    switch (errType)
    {
        case MCAN_ECC_ERR_TYPE_SEC:
            retVal = HW_RD_FIELD32(baseAddr + MCAN_ECC_AGGR_SEC_STATUS_REG0,
                                   MCAN_ECC_AGGR_SEC_STATUS_REG0_MSGMEM_PEND);
            break;
        case MCAN_ECC_ERR_TYPE_DED:
            retVal = HW_RD_FIELD32(baseAddr + MCAN_ECC_AGGR_DED_STATUS_REG0,
                                   MCAN_ECC_AGGR_DED_STATUS_REG0_MSGMEM_PEND);
            break;
        default:
            retVal = 0U;
            break;
    }
    return retVal;
}

void MCAN_eccClearIntrStatus(uint32_t baseAddr, uint32_t errType)
{
    switch (errType)
    {
        case MCAN_ECC_ERR_TYPE_SEC:
            HW_WR_FIELD32(baseAddr + MCAN_ECC_AGGR_SEC_STATUS_REG0,
                          MCAN_ECC_AGGR_SEC_STATUS_REG0_MSGMEM_PEND,
                          0x1U);
            break;
        case MCAN_ECC_ERR_TYPE_DED:
            HW_WR_FIELD32(baseAddr + MCAN_ECC_AGGR_DED_STATUS_REG0,
                          MCAN_ECC_AGGR_DED_STATUS_REG0_MSGMEM_PEND,
                          0x1U);
            break;
        default:
            break;
    }
}

void MCAN_extTSCounterConfig(uint32_t baseAddr,
                             uint32_t prescalar)
{
    HW_WR_FIELD32(baseAddr + MCAN_MCANSS_EXT_TS_PRESCALER,
                  MCAN_MCANSS_EXT_TS_PRESCALER, prescalar);
}

void MCAN_extTSCounterEnable(uint32_t baseAddr, uint32_t enable)
{
    HW_WR_FIELD32(baseAddr + MCAN_MCANSS_CTRL,
                  MCAN_MCANSS_CTRL_EXT_TS_CNTR_EN,
                  enable);
}

void MCAN_extTSEnableIntr(uint32_t baseAddr, uint32_t enable)
{
    if(TRUE == enable)
    {
        HW_WR_FIELD32(baseAddr + MCAN_MCANSS_IE,
                      MCAN_MCANSS_IE_EXT_TS_CNTR_OVFL,
                      1U);
    }
    else
    {
        HW_WR_FIELD32(baseAddr + MCAN_MCANSS_IECS,
                      MCAN_MCANSS_IECS_EXT_TS_CNTR_OVFL,
                      1U);
    }
}

void MCAN_extTSWriteEOI(uint32_t baseAddr)
{
    HW_WR_FIELD32(baseAddr + MCAN_MCANSS_EOI,
                  MCAN_MCANSS_EOI,
                  0x1U);
}

uint32_t MCAN_extTSGetUnservicedIntrCount(uint32_t baseAddr)
{
    return(HW_RD_FIELD32(baseAddr +
                          MCAN_MCANSS_EXT_TS_UNSERVICED_INTR_CNTR,
                          MCAN_MCANSS_EXT_TS_UNSERVICED_INTR_CNTR));
}

/* ========================================================================== */
/*                          Advance Functions                                 */
/* ========================================================================== */

void MCAN_getRevisionId(uint32_t baseAddr, MCAN_RevisionId *revId)
{
    uint32_t regVal;

    regVal        = HW_RD_REG32(baseAddr + MCAN_MCANSS_PID);
    revId->minor  = HW_GET_FIELD(regVal, MCAN_MCANSS_PID_MINOR);
    revId->custom = HW_GET_FIELD(regVal, MCAN_MCANSS_PID_CUSTOM);
    revId->major  = HW_GET_FIELD(regVal, MCAN_MCANSS_PID_MAJOR);
    revId->rtlRev = HW_GET_FIELD(regVal, MCAN_MCANSS_PID_RTL);
    revId->modId  = HW_GET_FIELD(regVal, MCAN_MCANSS_PID_MODULE_ID);
    revId->bu     = HW_GET_FIELD(regVal, MCAN_MCANSS_PID_BU);
    revId->scheme = HW_GET_FIELD(regVal, MCAN_MCANSS_PID_SCHEME);

    regVal         = HW_RD_REG32(baseAddr + MCAN_CREL);
    revId->day     = HW_GET_FIELD(regVal, MCAN_CREL_DAY);
    revId->mon     = HW_GET_FIELD(regVal, MCAN_CREL_MON);
    revId->year    = HW_GET_FIELD(regVal, MCAN_CREL_YEAR);
    revId->subStep = HW_GET_FIELD(regVal, MCAN_CREL_SUBSTEP);
    revId->step    = HW_GET_FIELD(regVal, MCAN_CREL_STEP);
    revId->rel     = HW_GET_FIELD(regVal, MCAN_CREL_REL);
}

uint32_t MCAN_getClockStopAck(uint32_t baseAddr)
{
    return(HW_RD_FIELD32(baseAddr + MCAN_CCCR, MCAN_CCCR_CSR));
}

void MCAN_extTSSetRawStatus(uint32_t baseAddr)
{
    HW_WR_FIELD32(baseAddr + MCAN_MCANSS_IRS,
                  MCAN_MCANSS_IRS_EXT_TS_CNTR_OVFL,
                  1U);
}

void MCAN_extTSClearRawStatus(uint32_t baseAddr)
{
    HW_WR_FIELD32(baseAddr + MCAN_MCANSS_ICS,
                  MCAN_MCANSS_ICS_EXT_TS_CNTR_OVFL,
                  1U);
}

uint32_t MCAN_getRxPinState(uint32_t baseAddr)
{
    return(HW_RD_FIELD32(baseAddr + MCAN_TEST, MCAN_TEST_RX));
}

void MCAN_setTxPinState(uint32_t baseAddr, uint32_t state)
{
    MCAN_writeProtectedRegAccessUnlock(baseAddr);

    HW_WR_FIELD32(baseAddr + MCAN_CCCR, MCAN_CCCR_TEST, 0x1U);
    HW_WR_FIELD32(baseAddr + MCAN_TEST,
                  MCAN_TEST_TX,
                  state);

    MCAN_writeProtectedRegAccessLock(baseAddr);
}

uint32_t MCAN_getTxPinState(uint32_t baseAddr)
{
    return(HW_RD_FIELD32(baseAddr + MCAN_TEST, MCAN_TEST_TX));
}

uint32_t MCAN_getTSCounterVal(uint32_t baseAddr)
{
    return(HW_RD_FIELD32(baseAddr + MCAN_TSCV, MCAN_TSCV_TSC));
}

uint32_t MCAN_getClkStopAck(uint32_t baseAddr)
{
    return(HW_RD_FIELD32(baseAddr + MCAN_CCCR, MCAN_CCCR_CSA));
}

void MCAN_getBitTime(uint32_t              baseAddr,
                     MCAN_BitTimingParams *configParams)
{
    configParams->nomSynchJumpWidth = HW_RD_FIELD32(baseAddr + MCAN_NBTP,
                                                    MCAN_NBTP_NSJW);
    configParams->nomTimeSeg2 = HW_RD_FIELD32(baseAddr + MCAN_NBTP,
                                              MCAN_NBTP_NTSEG2);
    configParams->nomTimeSeg1 = HW_RD_FIELD32(baseAddr + MCAN_NBTP,
                                              MCAN_NBTP_NTSEG1);
    configParams->nomRatePrescalar = HW_RD_FIELD32(baseAddr + MCAN_NBTP,
                                                   MCAN_NBTP_NBRP);

    configParams->dataSynchJumpWidth = HW_RD_FIELD32(baseAddr + MCAN_DBTP,
                                                     MCAN_DBTP_DSJW);
    configParams->dataTimeSeg2 = HW_RD_FIELD32(baseAddr + MCAN_DBTP,
                                               MCAN_DBTP_DTSEG2);
    configParams->dataTimeSeg1 = HW_RD_FIELD32(baseAddr + MCAN_DBTP,
                                               MCAN_DBTP_DTSEG1);
    configParams->dataRatePrescalar = HW_RD_FIELD32(baseAddr + MCAN_DBTP,
                                                    MCAN_DBTP_DBRP);
}

void MCAN_resetTSCounter(uint32_t baseAddr)
{
    HW_WR_FIELD32(baseAddr + MCAN_TSCV, MCAN_TSCV_TSC, 0x0U);
}

uint32_t MCAN_getTOCounterVal(uint32_t baseAddr)
{
    return(HW_RD_FIELD32(baseAddr + MCAN_TOCV, MCAN_TOCV_TOC));
}

void MCAN_eccAggrGetRevisionId(uint32_t baseAddr, MCAN_ECCAggrRevisionId *revId)
{
    uint32_t regVal;

    regVal        = HW_RD_REG32(baseAddr + MCAN_ECC_AGGR_REVISION);
    revId->minor  = HW_GET_FIELD(regVal, MCAN_ECC_AGGR_REVISION_REVMIN);
    revId->custom = HW_GET_FIELD(regVal, MCAN_ECC_AGGR_REVISION_CUSTOM);
    revId->major  = HW_GET_FIELD(regVal, MCAN_ECC_AGGR_REVISION_REVMAJ);
    revId->rtlRev = HW_GET_FIELD(regVal, MCAN_ECC_AGGR_REVISION_REVRTL);
    revId->modId  = HW_GET_FIELD(regVal, MCAN_ECC_AGGR_REVISION_MODULE_ID);
    revId->bu     = HW_GET_FIELD(regVal, MCAN_ECC_AGGR_REVISION_BU);
    revId->scheme = HW_GET_FIELD(regVal, MCAN_ECC_AGGR_REVISION_SCHEME);
}

void MCAN_eccWrapGetRevisionId(uint32_t baseAddr, MCAN_ECCWrapRevisionId *revId)
{
    uint32_t regVal;

    MCAN_eccLoadRegister(baseAddr, MCAN_ECC_AGGR_WRAP_REVISION);
    regVal        = HW_RD_REG32(baseAddr + MCAN_ECC_AGGR_WRAP_REVISION);
    revId->minor  = HW_GET_FIELD(regVal, MCAN_ECC_AGGR_WRAP_REVISION_REVMIN);
    revId->custom = HW_GET_FIELD(regVal, MCAN_ECC_AGGR_WRAP_REVISION_CUSTOM);
    revId->major  = HW_GET_FIELD(regVal, MCAN_ECC_AGGR_WRAP_REVISION_REVMAJ);
    revId->rtlRev = HW_GET_FIELD(regVal, MCAN_ECC_AGGR_WRAP_REVISION_REVRTL);
    revId->modId  = HW_GET_FIELD(regVal, MCAN_ECC_AGGR_WRAP_REVISION_MODULE_ID);
    revId->bu     = HW_GET_FIELD(regVal, MCAN_ECC_AGGR_WRAP_REVISION_BU);
    revId->scheme = HW_GET_FIELD(regVal, MCAN_ECC_AGGR_WRAP_REVISION_SCHEME);
}

uint32_t MCAN_extTSIsIntrEnable(uint32_t baseAddr)
{
    uint32_t status;

    if(1U == HW_RD_FIELD32(baseAddr + MCAN_MCANSS_IES,
                            MCAN_MCANSS_IES_EXT_TS_CNTR_OVFL))
    {
        status = (uint32_t) TRUE;
    }
    else
    {
        status = (uint32_t) FALSE;
    }

    return status;
}

uint32_t MCAN_getEndianVal(uint32_t baseAddr)
{
    return(HW_RD_FIELD32(baseAddr + MCAN_ENDN, MCAN_ENDN_ETV));
}

uint32_t MCAN_getExtIDANDMask(uint32_t baseAddr)
{
    return(HW_RD_FIELD32(baseAddr + MCAN_XIDAM, MCAN_XIDAM_EIDM));
}

/* ========================================================================== */
/*                          Internal Functions                                */
/* ========================================================================== */

static void MCAN_writeProtectedRegAccessUnlock(uint32_t baseAddr)
{
    HW_WR_FIELD32(baseAddr + MCAN_CCCR, MCAN_CCCR_CCE, 0x1U);
}

static void MCAN_writeProtectedRegAccessLock(uint32_t baseAddr)
{
    HW_WR_FIELD32(baseAddr + MCAN_CCCR, MCAN_CCCR_CCE, 0x0U);
}

static void MCAN_eccLoadRegister(uint32_t baseAddr, uint32_t regOffset)
{
    uint32_t regVal = 0U, offset;

    offset  = regOffset & 0xFFU;
    regVal |= ((uint32_t)MCANSS_MSG_RAM_NUM << MCAN_ECC_AGGR_VECTOR_SHIFT);
    regVal |= (offset << MCAN_ECC_AGGR_VECTOR_RD_SVBUS_ADDRESS_SHIFT);
    regVal |= ((uint32_t)1U << MCAN_ECC_AGGR_VECTOR_RD_SVBUS_SHIFT);
    HW_WR_REG32(baseAddr + MCAN_ECC_AGGR_VECTOR, regVal);
    while(MCAN_ECC_AGGR_VECTOR_RD_SVBUS_DONE_MASK !=
           (HW_RD_REG32(baseAddr + MCAN_ECC_AGGR_VECTOR) &
            MCAN_ECC_AGGR_VECTOR_RD_SVBUS_DONE_MASK))
    {}
}

static void MCAN_readMsg(uint32_t           baseAddr,
                         uint32_t           elemAddr,
                         MCAN_RxBufElement *elem)
{
    uint32_t regVal = 0U, loopCnt = 0U;

    regVal   = HW_RD_REG32(baseAddr + elemAddr);
    elem->id = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_ID_MASK)
                           >> MCANSS_RX_BUFFER_ELEM_ID_SHIFT);
    elem->rtr = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_RTR_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_RTR_SHIFT);
    elem->xtd = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_XTD_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_XTD_SHIFT);
    elem->esi = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_ESI_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_ESI_SHIFT);

    elemAddr  += 4U;
    regVal     = HW_RD_REG32(baseAddr + elemAddr);
    elem->rxts = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_RXTS_MASK)
                             >> MCANSS_RX_BUFFER_ELEM_RXTS_SHIFT);
    elem->dlc = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_DLC_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_DLC_SHIFT);
    elem->brs = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_BRS_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_BRS_SHIFT);
    elem->fdf = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_FDF_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_FDF_SHIFT);
    elem->fidx = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_FIDX_MASK)
                             >> MCANSS_RX_BUFFER_ELEM_FIDX_SHIFT);
    elem->anmf = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_ANMF_MASK)
                             >> MCANSS_RX_BUFFER_ELEM_ANMF_SHIFT);
    elemAddr += 4U;

    loopCnt = 0U;
    /* Reading words from message RAM and forming payload bytes out of it */
    while((4U <= (MCAN_getDataSize(elem->dlc) - loopCnt)) &&
           (0U != (MCAN_getDataSize(elem->dlc) - loopCnt)))
    {
        ASSERT((loopCnt + 3U) < MCAN_MAX_PAYLOAD_BYTES);
        regVal = HW_RD_REG32(baseAddr + elemAddr);
        elem->data[loopCnt]       = (uint16_t)(regVal & 0x000000FFU);
        elem->data[(loopCnt + 1U)] = (uint16_t)((regVal & 0x0000FF00U) >> 8U);
        elem->data[(loopCnt + 2U)] = (uint16_t)((regVal & 0x00FF0000U) >> 16U);
        elem->data[(loopCnt + 3U)] = (uint16_t)((regVal & 0xFF000000U) >> 24U);
        elemAddr += 4U;
        loopCnt  += 4U;
    }
    /* Reading remaining bytes from message RAM */
    if(0U < (MCAN_getDataSize(elem->dlc) - loopCnt))
    {
        ASSERT((loopCnt + 2U) < MCAN_MAX_PAYLOAD_BYTES);
        regVal = HW_RD_REG32(baseAddr + elemAddr);
        elem->data[loopCnt]       = (uint16_t)(regVal & 0x000000FFU);
        elem->data[(loopCnt + 1U)] = (uint16_t)((regVal & 0x0000FF00U) >> 8U);
        elem->data[(loopCnt + 2U)] = (uint16_t)((regVal & 0x00FF0000U) >> 16U);
    }
}

static void MCAN_writeMsg(uint32_t                 baseAddr,
                          uint32_t                 elemAddr,
                          const MCAN_TxBufElement *elem)
{
    uint32_t regVal = 0, loopCnt = 0U;

    regVal  = 0U;
    regVal |= (((uint32_t) (elem->id << MCANSS_TX_BUFFER_ELEM_ID_SHIFT)) |
               ((uint32_t) (elem->rtr << MCANSS_TX_BUFFER_ELEM_RTR_SHIFT)) |
               ((uint32_t) (elem->xtd << MCANSS_TX_BUFFER_ELEM_XTD_SHIFT)) |
               ((uint32_t) (elem->esi << MCANSS_TX_BUFFER_ELEM_ESI_SHIFT)));
    HW_WR_REG32(baseAddr + elemAddr, regVal);
    elemAddr += 4U;

    regVal  = 0U;
    regVal |= ((uint32_t) (elem->dlc << MCANSS_TX_BUFFER_ELEM_DLC_SHIFT)) |
              ((uint32_t) (elem->brs << MCANSS_TX_BUFFER_ELEM_BRS_SHIFT)) |
              ((uint32_t) (elem->fdf << MCANSS_TX_BUFFER_ELEM_FDF_SHIFT)) |
              ((uint32_t) (elem->efc << MCANSS_TX_BUFFER_ELEM_EFC_SHIFT)) |
              ((uint32_t) (elem->mm << MCANSS_TX_BUFFER_ELEM_MM_SHIFT));
    HW_WR_REG32(baseAddr + elemAddr, regVal);
    elemAddr += 4U;

    loopCnt = 0U;
    /* Framing words out of the payload bytes and writing it to message RAM */
    while((4U <= (MCAN_getDataSize(elem->dlc) - loopCnt)) &&
           (0U != (MCAN_getDataSize(elem->dlc) - loopCnt)))
    {
        ASSERT((loopCnt + 3U) < MCAN_MAX_PAYLOAD_BYTES);
        regVal  = 0U;
        regVal |= ((uint32_t)elem->data[loopCnt] |
                   ((uint32_t)elem->data[(loopCnt + 1U)] << 8U) |
                   ((uint32_t)elem->data[(loopCnt + 2U)] << 16U) |
                   ((uint32_t)elem->data[(loopCnt + 3U)] << 24U));
        HW_WR_REG32(baseAddr + elemAddr, regVal);
        elemAddr += 4U;
        loopCnt  += 4U;
    }
    /* Framing a word out of remaining payload bytes and writing it to
     * message RAM */
    if(0U < (MCAN_getDataSize(elem->dlc) - loopCnt))
    {
        ASSERT((loopCnt + 3U) < MCAN_MAX_PAYLOAD_BYTES);
        regVal  = 0U;
        regVal |= ((uint32_t)elem->data[loopCnt] |
                   ((uint32_t)elem->data[(loopCnt + 1U)] << 8U) |
                   ((uint32_t)elem->data[(loopCnt + 2U)] << 16U) |
                   ((uint32_t)elem->data[(loopCnt + 3U)] << 24U));
        HW_WR_REG32(baseAddr + elemAddr, regVal);
    }
}

static uint32_t MCAN_getDataSize(uint32_t dlc)
{
    uint32_t dataSize[16] = {0,  1,  2,  3,  4,  5,  6, 7, 8,
                             12, 16, 20, 24, 32, 48, 64};
    ASSERT(dlc < 16U);
    return(dataSize[dlc]);
}

uint32_t MCAN_getMsgObjSize(uint32_t elemSize)
{
    uint32_t objSize[8] = {4, 5, 6, 7, 8, 10, 14, 18};
    ASSERT(elemSize < 8U);
    return(objSize[elemSize]);
}
