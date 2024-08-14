#ifndef DRIVER_INCLUSIVE_TERMINOLOGY_MAPPING_H_
#define DRIVER_INCLUSIVE_TERMINOLOGY_MAPPING_H_


//*****************************************************************************
// CLB
//*****************************************************************************
#define CLB_LOCAL_IN_MUX_SPISIMO_SLAVE          CLB_LOCAL_IN_MUX_SPIPICO_PERIPHERAL
#define CLB_LOCAL_IN_MUX_SPISIMO_MASTER         CLB_LOCAL_IN_MUX_SPIPICO_CONTROLLER
#define CLB_GLOBAL_IN_MUX_SPI1_SPISOMI_MASTER   CLB_GLOBAL_IN_MUX_SPI1_SPIPOCI_CONTROLLER
#define CLB_GLOBAL_IN_MUX_SPI1_SPISTE           CLB_GLOBAL_IN_MUX_SPI1_SPIPTE
#define CLB_GLOBAL_IN_MUX_SPI2_SPISOMI_MASTER   CLB_GLOBAL_IN_MUX_SPI2_SPIPOCI_CONTROLLER
#define CLB_GLOBAL_IN_MUX_SPI2_SPISTE           CLB_GLOBAL_IN_MUX_SPI2_SPIPTE
#define CLB_GLOBAL_IN_MUX_SPI3_SPISOMI_MASTER   CLB_GLOBAL_IN_MUX_SPI3_SPIPOCI_CONTROLLER
#define CLB_GLOBAL_IN_MUX_SPI3_SPISTE           CLB_GLOBAL_IN_MUX_SPI3_SPIPTE
#define CLB_GLOBAL_IN_MUX_SPI4_SPISOMI_MASTER   CLB_GLOBAL_IN_MUX_SPI4_SPIPOCI_CONTROLLER
#define CLB_GLOBAL_IN_MUX_SPI4_SPISTE           CLB_GLOBAL_IN_MUX_SPI4_SPIPTE

//*****************************************************************************
// FSI
//*****************************************************************************
#define FSI_TX_MASTER_CORE_RESET    FSI_TX_MAIN_CORE_RESET
#define FSI_RX_MASTER_CORE_RESET    FSI_RX_MAIN_CORE_RESET


//*****************************************************************************
// SPI
//*****************************************************************************
#define SPI_MODE_SLAVE              SPI_MODE_PERIPHERAL
#define SPI_MODE_MASTER             SPI_MODE_CONTROLLER
#define SPI_MODE_SLAVE_OD           SPI_MODE_PERIPHERAL_OD
#define SPI_MODE_MASTER_OD          SPI_MODE_CONTROLLER_OD

#define SPI_STE_ACTIVE_LOW          SPI_PTE_ACTIVE_LOW
#define SPI_STE_ACTIVE_HIGH         SPI_PTE_ACTIVE_HIGH

#define SPI_setSTESignalPolarity    SPI_setPTESignalPolarity


//*****************************************************************************
// Interrupt
//*****************************************************************************
#define Interrupt_enableMaster      Interrupt_enableGlobal
#define Interrupt_disableMaster     Interrupt_disableGlobal


//*****************************************************************************
// SysCtrl
//*****************************************************************************
#define SysCtl_AccessMaster     SysCtl_AccessController


//*****************************************************************************
// GPIO
//*****************************************************************************
#define GPIO_setMasterCore GPIO_setControllerCore



//*****************************************************************************
// Memcfg
//*****************************************************************************
#define MemCfg_LSRAMMMasterSel          MemCfg_LSRAMMControllerSel
#define MEMCFG_LSRAMMASTER_CPU_ONLY     MEMCFG_LSRAMCONTROLLER_CPU_ONLY
#define MEMCFG_LSRAMMASTER_CPU_CLA1     MEMCFG_LSRAMCONTROLLER_CPU_CLA1
#define MemCfg_setLSRAMMasterSel        MemCfg_setLSRAMControllerSel

#define MemCfg_GSRAMMasterSel           MemCfg_GSRAMControllerSel
#define MEMCFG_GSRAMMASTER_CPU1         MEMCFG_GSRAMCONTROLLER_CPU1
#define MEMCFG_GSRAMMASTER_CPU2         MEMCFG_GSRAMCONTROLLER_CPU2
#define MemCfg_setGSRAMMasterSel        MemCfg_setGSRAMControllerSel

//*****************************************************************************
// EMIF
//*****************************************************************************
#define EMIF_MasterSelect               EMIF_ControllerSelect
#define EMIF_selectMaster               EMIF_selectController
#define EMIF_MASTER_CPU1_NG             EMIF_CONTROLLER_CPU1_NG
#define EMIF_MASTER_CPU1_G              EMIF_CONTROLLER_CPU1_G
#define EMIF_MASTER_CPU2_G              EMIF_CONTROLLER_CPU2_G
#define EMIF_MASTER_CPU1_NG2            EMIF_CONTROLLER_CPU1_NG2


//*****************************************************************************
// I2C
//*****************************************************************************
#define I2C_MASTER_SEND_MODE              I2C_CONTROLLER_SEND_MODE
#define I2C_MASTER_RECEIVE_MODE           I2C_CONTROLLER_RECEIVE_MODE
#define I2C_SLAVE_SEND_MODE               I2C_TARGET_SEND_MODE
#define I2C_SLAVE_RECEIVE_MODE            I2C_TARGET_RECEIVE_MODE
#define I2C_INT_ADDR_SLAVE                I2C_INT_ADDR_TARGET
#define I2C_STS_ADDR_SLAVE                I2C_STS_ADDR_TARGET
#define I2C_STS_SLAVE_DIR                 I2C_STS_TARGET_DIR
#define I2C_INTSRC_ADDR_SLAVE             I2C_INTSRC_ADDR_TARGET

#define I2C_initMaster                    I2C_initController
#define I2C_setSlaveAddress               I2C_setTargetAddress
#define I2C_setOwnSlaveAddress            I2C_setOwnAddress

//*****************************************************************************
// PMBUS
//*****************************************************************************
#define PMBUS_MASTER_ENABLE_PRC_CALL      PMBUS_CONTROLLER_ENABLE_PRC_CALL
#define PMBUS_MASTER_ENABLE_GRP_CMD       PMBUS_CONTROLLER_ENABLE_GRP_CMD
#define PMBUS_MASTER_ENABLE_PEC           PMBUS_CONTROLLER_ENABLE_PEC
#define PMBUS_MASTER_ENABLE_EXT_CMD       PMBUS_CONTROLLER_ENABLE_EXT_CMD
#define PMBUS_MASTER_ENABLE_CMD           PMBUS_CONTROLLER_ENABLE_CMD
#define PMBUS_MASTER_ENABLE_READ          PMBUS_CONTROLLER_ENABLE_READ
#define PMBUS_INT_SLAVE_ADDR_READY        PMBUS_INT_TARGET_ADDR_READY
#define PMBUS_SLAVE_ENABLE_MANUAL_ACK     PMBUS_TARGET_ENABLE_MANUAL_ACK
#define PMBUS_SLAVE_ENABLE_PEC_PROCESSING PMBUS_TARGET_ENABLE_PEC_PROCESSING
#define PMBUS_SLAVE_TRANSMIT_PEC          PMBUS_TARGET_TRANSMIT_PEC
#define PMBUS_SLAVE_ENABLE_MANUAL_CMD_ACK PMBUS_TARGET_ENABLE_MANUAL_CMD_ACK
#define PMBUS_SLAVE_DISABLE_ADDRESS_MASK  PMBUS_TARGET_DISABLE_ADDRESS_MASK
#define PMBUS_SLAVE_AUTO_ACK_1_BYTES      PMBUS_TARGET_AUTO_ACK_1_BYTES
#define PMBUS_SLAVE_AUTO_ACK_2_BYTES      PMBUS_TARGET_AUTO_ACK_2_BYTES
#define PMBUS_SLAVE_AUTO_ACK_3_BYTES      PMBUS_TARGET_AUTO_ACK_3_BYTES
#define PMBUS_SLAVE_AUTO_ACK_4_BYTES      PMBUS_TARGET_AUTO_ACK_4_BYTES
#define PMBUS_ENABLE_SLAVE_ALERT          PMBUS_ENABLE_TARGET_ALERT
#define PMBUS_ENABLE_SLAVE_MODE           PMBUS_ENABLE_TARGET_MODE
#define PMBUS_ENABLE_MASTER_MODE          PMBUS_ENABLE_CONTROLLER_MODE
#define PMBUS_INTSRC_SLAVE_ADDR_READY     PMBUS_INTSRC_TARGET_ADDR_READY
#define PMBus_getMasterData               PMBus_getControllerData
#define PMBus_getSlaveData                PMBus_getTargetData

#define PMBus_configMaster                PMBus_configControlleer
#define PMBus_putMasterData               PMBus_putControllerData
#define PMBus_putSlaveData                PMBus_putTargetData
#define PMBus_initSlaveMode               PMBus_initTargetMode
#define PMBus_initMasterMode              PMBus_initControllerMode

//*****************************************************************************
// SDFM
//*****************************************************************************
#define SDFM_enableMasterInterrupt      SDFM_enableMainInterrupt
#define SDFM_disableMasterInterrupt     SDFM_disableMainInterrupt
#define SDFM_enableMasterFilter         SDFM_enableMainFilter
#define SDFM_disableMasterFilter        SDFM_disableMainFilter

#define SDFM_MASTER_INTERRUPT_FLAG      SDFM_MAIN_INTERRUPT_FLAG

#endif /* DRIVER_INCLUSIVE_TERMINOLOGY_MAPPING_H_ */
