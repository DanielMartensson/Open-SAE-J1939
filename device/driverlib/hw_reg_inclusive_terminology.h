#ifndef HW_REG_INCLUSIVE_TERMINOLOGY_H
#define HW_REG_INCLUSIVE_TERMINOLOGY_H

//*****************************************************************************
// PMBUS
//*****************************************************************************
#define PMBUS_O_PMBCCR                   PMBUS_O_PMBMC
#define PMBUS_O_PMBTCR                   PMBUS_O_PMBSC
#define PMBUS_O_PMBHTA                   PMBUS_O_PMBHSA

#define PMBUS_PMBCCR_RW                  PMBUS_PMBMC_RW
#define PMBUS_PMBCCR_TARGET_ADDR_S       PMBUS_PMBMC_SLAVE_ADDR_S
#define PMBUS_PMBCCR_TARGET_ADDR_M       PMBUS_PMBMC_SLAVE_ADDR_M
#define PMBUS_PMBCCR_BYTE_COUNT_S        PMBUS_PMBMC_BYTE_COUNT_S
#define PMBUS_PMBCCR_BYTE_COUNT_M        PMBUS_PMBMC_BYTE_COUNT_M
#define PMBUS_PMBCCR_CMD_ENA             PMBUS_PMBMC_CMD_ENA
#define PMBUS_PMBCCR_EXT_CMD             PMBUS_PMBMC_EXT_CMD
#define PMBUS_PMBCCR_PEC_ENA             PMBUS_PMBMC_PEC_ENA
#define PMBUS_PMBCCR_GRP_CMD             PMBUS_PMBMC_GRP_CMD
#define PMBUS_PMBCCR_PRC_CALL            PMBUS_PMBMC_PRC_CALL

#define PMBUS_PMBTCR_TARGET_ADDR_S       PMBUS_PMBSC_SLAVE_ADDR_S
#define PMBUS_PMBTCR_TARGET_ADDR_M       PMBUS_PMBSC_SLAVE_ADDR_M
#define PMBUS_PMBTCR_MAN_TARGET_ACK      PMBUS_PMBSC_MAN_SLAVE_ACK
#define PMBUS_PMBTCR_TARGET_MASK_S       PMBUS_PMBSC_SLAVE_MASK_S
#define PMBUS_PMBTCR_TARGET_MASK_M       PMBUS_PMBSC_SLAVE_MASK_M
#define PMBUS_PMBTCR_PEC_ENA             PMBUS_PMBSC_PEC_ENA
#define PMBUS_PMBTCR_TX_COUNT_S          PMBUS_PMBSC_TX_COUNT_S
#define PMBUS_PMBTCR_TX_COUNT_M          PMBUS_PMBSC_TX_COUNT_M
#define PMBUS_PMBTCR_TX_PEC              PMBUS_PMBSC_TX_PEC
#define PMBUS_PMBTCR_MAN_CMD             PMBUS_PMBSC_MAN_CMD
#define PMBUS_PMBTCR_RX_BYTE_ACK_CNT_S   PMBUS_PMBSC_RX_BYTE_ACK_CNT_S
#define PMBUS_PMBTCR_RX_BYTE_ACK_CNT_M   PMBUS_PMBSC_RX_BYTE_ACK_CNT_M

#define PMBUS_PMBHTA_TARGET_RW           PMBUS_PMBHSA_SLAVE_RW
#define PMBUS_PMBHTA_TARGET_ADDR_S       PMBUS_PMBHSA_SLAVE_ADDR_S
#define PMBUS_PMBHTA_TARGET_ADDR_M       PMBUS_PMBHSA_SLAVE_ADDR_M

#define PMBUS_PMBCTRL_CONTROLLER_EN      PMBUS_PMBCTRL_MASTER_EN
#define PMBUS_PMBCTRL_TARGET_EN          PMBUS_PMBCTRL_SLAVE_EN

#define PMBUS_PMBSTS_CONTROLLER          PMBUS_PMBSTS_MASTER
#define PMBUS_PMBSTS_TARGET_ADDR_READY   PMBUS_PMBSTS_SLAVE_ADDR_READY

#define PMBUS_PMBINTM_TARGET_ADDR_READY  PMBUS_PMBINTM_SLAVE_ADDR_READY

//*****************************************************************************
// FSI
//*****************************************************************************
#define FSI_O_TX_MAIN_CTRL               FSI_O_TX_MASTER_CTRL
#define FSI_O_RX_MAIN_CTRL               FSI_O_RX_MASTER_CTRL

#define FSI_TX_MAIN_CTRL_CORE_RST        FSI_TX_MASTER_CTRL_CORE_RST
#define FSI_TX_MAIN_CTRL_FLUSH           FSI_TX_MASTER_CTRL_FLUSH
#define FSI_TX_MAIN_CTRL_KEY_S           FSI_TX_MASTER_CTRL_KEY_S
#define FSI_TX_MAIN_CTRL_KEY_M           FSI_TX_MASTER_CTRL_KEY_M

#define FSI_RX_MAIN_CTRL_CORE_RST        FSI_RX_MASTER_CTRL_CORE_RST
#define FSI_RX_MAIN_CTRL_INT_LOOPBACK    FSI_RX_MASTER_CTRL_INT_LOOPBACK
#define FSI_RX_MAIN_CTRL_SPI_PAIRING     FSI_RX_MASTER_CTRL_SPI_PAIRING
#define FSI_RX_MAIN_CTRL_INPUT_ISOLATE   FSI_RX_MASTER_CTRL_INPUT_ISOLATE
#define FSI_RX_MAIN_CTRL_DATA_FILTER_EN  FSI_RX_MASTER_CTRL_DATA_FILTER_EN
#define FSI_RX_MAIN_CTRL_KEY_S           FSI_RX_MASTER_CTRL_KEY_S
#define FSI_RX_MAIN_CTRL_KEY_M           FSI_RX_MASTER_CTRL_KEY_M

//*****************************************************************************
// SPI
//*****************************************************************************
#define SPI_CTL_CONTROLLER_PERIPHERAL    SPI_CTL_MASTER_SLAVE
#define SPI_PRI_PTEINV                   SPI_PRI_STEINV

//*****************************************************************************
// I2C
//*****************************************************************************
#define I2C_O_TAR         I2C_O_SAR

#define I2C_TAR_TAR_S     I2C_SAR_SAR_S
#define I2C_TAR_TAR_M     I2C_SAR_SAR_M

#define I2C_IER_AAT       I2C_IER_AAS

#define I2C_STR_AAT       I2C_STR_AAS
#define I2C_STR_TDIR      I2C_STR_SDIR

#define I2C_MDR_CNT       I2C_MDR_MST



#endif // HW_REG_INCLUSIVE_TERMINOLOGY_H
