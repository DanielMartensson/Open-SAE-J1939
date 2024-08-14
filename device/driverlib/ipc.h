//###########################################################################
//
// FILE:   ipc.h
//
// TITLE:  C28x IPC driver.
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

#ifndef IPC_H
#define IPC_H

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
//! \addtogroup ipc_api IPC
//! \brief This module is used for inter-processor communications.
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "debug.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ipc.h"
#include "inc/hw_ints.h"
#include "interrupt.h"

//*****************************************************************************
//
// Defines for the APIs
//
//*****************************************************************************
//*****************************************************************************
//
// Driver configuration macros
//
//*****************************************************************************
#define IPC_MSGQ_SUPPORT       1U

//
// Number of IPC messages in circular buffer (must be interval of 2)
//
#define IPC_BUFFER_SIZE        4U

//
// Number of IPC interrupts using circular buffer (must be same number on both
// CPUs)
//
#define IPC_NUM_OF_INTERRUPTS  4U

//*****************************************************************************
//
// Values that can be passed as parameter flags in all the IPC API functions.
//
//*****************************************************************************
#ifndef IPC_FLAGS_DEFINED
#define IPC_FLAGS_DEFINED
#define IPC_NO_FLAG     0x00000000U  //!< NO FLAG
#define IPC_FLAG0       0x00000001U  //!< IPC FLAG 0
#define IPC_FLAG1       0x00000002U  //!< IPC FLAG 1
#define IPC_FLAG2       0x00000004U  //!< IPC FLAG 2
#define IPC_FLAG3       0x00000008U  //!< IPC FLAG 3
#define IPC_FLAG4       0x00000010U  //!< IPC FLAG 4
#define IPC_FLAG5       0x00000020U  //!< IPC FLAG 5
#define IPC_FLAG6       0x00000040U  //!< IPC FLAG 6
#define IPC_FLAG7       0x00000080U  //!< IPC FLAG 7
#define IPC_FLAG8       0x00000100U  //!< IPC FLAG 8
#define IPC_FLAG9       0x00000200U  //!< IPC FLAG 9
#define IPC_FLAG10      0x00000400U  //!< IPC FLAG 10
#define IPC_FLAG11      0x00000800U  //!< IPC FLAG 11
#define IPC_FLAG12      0x00001000U  //!< IPC FLAG 12
#define IPC_FLAG13      0x00002000U  //!< IPC FLAG 13
#define IPC_FLAG14      0x00004000U  //!< IPC FLAG 14
#define IPC_FLAG15      0x00008000U  //!< IPC FLAG 15
#define IPC_FLAG16      0x00010000U  //!< IPC FLAG 16
#define IPC_FLAG17      0x00020000U  //!< IPC FLAG 17
#define IPC_FLAG18      0x00040000U  //!< IPC FLAG 18
#define IPC_FLAG19      0x00080000U  //!< IPC FLAG 19
#define IPC_FLAG20      0x00100000U  //!< IPC FLAG 20
#define IPC_FLAG21      0x00200000U  //!< IPC FLAG 21
#define IPC_FLAG22      0x00400000U  //!< IPC FLAG 22
#define IPC_FLAG23      0x00800000U  //!< IPC FLAG 23
#define IPC_FLAG24      0x01000000U  //!< IPC FLAG 24
#define IPC_FLAG25      0x02000000U  //!< IPC FLAG 25
#define IPC_FLAG26      0x04000000U  //!< IPC FLAG 26
#define IPC_FLAG27      0x08000000U  //!< IPC FLAG 27
#define IPC_FLAG28      0x10000000U  //!< IPC FLAG 28
#define IPC_FLAG29      0x20000000U  //!< IPC FLAG 29
#define IPC_FLAG30      0x40000000U  //!< IPC FLAG 30
#define IPC_FLAG31      0x80000000U  //!< IPC FLAG 31
#define IPC_FLAG_ALL    0xFFFFFFFFU  //!< All IPC flags
#endif

//*****************************************************************************
//
// Values that can be passed as parameter ipcInt in
// IPC_registerInterrupt and IPC_unregisterInterrupt functions.
// Please refer to the datasheet for the actual number of interrupts available
// for each IPC instance
//
//*****************************************************************************
#define IPC_INT0        0x0U  //!< IPC Interrupt 0
#define IPC_INT1        0x1U  //!< IPC Interrupt 1
#define IPC_INT2        0x2U  //!< IPC Interrupt 2
#define IPC_INT3        0x3U  //!< IPC Interrupt 3
#define IPC_INT4        0x4U  //!< IPC Interrupt 4
#define IPC_INT5        0x5U  //!< IPC Interrupt 5
#define IPC_INT6        0x6U  //!< IPC Interrupt 6
#define IPC_INT7        0x7U  //!< IPC Interrupt 7

//*****************************************************************************
//
// Values that can be passed as parameter addrCorrEnable in
// IPC_sendCommand, IPC_readCommand, IPC_sendMessageToQueue and
// IPC_readMessageFromQueue functions.
//
//*****************************************************************************
#define IPC_ADDR_CORRECTION_ENABLE   true
#define IPC_ADDR_CORRECTION_DISABLE  false

//*****************************************************************************
//
// Values that can be passed as parameter block in
// IPC_sendMessageToQueue and IPC_readMessageFromQueue functions.
//
//*****************************************************************************
#define IPC_BLOCKING_CALL     true
#define IPC_NONBLOCKING_CALL  false

//*****************************************************************************
//
// Macros used to get and release the flash pump semaphore
//
//*****************************************************************************

#ifdef CPU2
#define IPC_PUMPREQUEST_REG                                                    \
        IPC_Instance[IPC_CPU2_L_CPU1_R].IPC_Boot_Pump_Reg->IPC_PUMPREQUEST
#else
#define IPC_PUMPREQUEST_REG                                                    \
        IPC_Instance[IPC_CPU1_L_CPU2_R].IPC_Boot_Pump_Reg->IPC_PUMPREQUEST
#endif


//*****************************************************************************
//
// Internal macros used for message queue implementation
//
//*****************************************************************************
#define IPC_MAX_BUFFER_INDEX    (IPC_BUFFER_SIZE - 1U)

//*****************************************************************************
//
// Enums for the APIs
//
//*****************************************************************************

//*****************************************************************************
//
//! Values that can be passed as parameter \e ipcType in all the driver
//! functions
//
//*****************************************************************************
typedef enum
{
    IPC_CPU1_L_CPU2_R,       //!< CPU1 - Local core, CPU2 - Remote core
    IPC_CPU1_L_CM_R,         //!< CPU1 - Local core, CM   - Remote core
    IPC_CPU2_L_CPU1_R,       //!< CPU2 - Local core, CPU1 - Remote core
    IPC_CPU2_L_CM_R,         //!< CPU2 - Local core, CM   - Remote core
    IPC_TOTAL_NUM
}IPC_Type_t;

//*****************************************************************************
//
// Internal structs for register and messaage queue accesses
//
//*****************************************************************************
typedef struct
{
    uint32_t IPC_ACK;
    uint32_t IPC_STS;
    uint32_t IPC_SET;
    uint32_t IPC_CLR;
    uint32_t IPC_FLG;
    uint32_t IPC_RSVDREG;
    uint32_t IPC_COUNTERL;
    uint32_t IPC_COUNTERH;
}IPC_Flag_Ctr_Reg_t;

typedef struct
{
    uint32_t IPC_SENDCOM;
    uint32_t IPC_SENDADDR;
    uint32_t IPC_SENDDATA;
    uint32_t IPC_REMOTEREPLY;
}IPC_SendCmd_Reg_t;

typedef struct
{
    uint32_t IPC_RECVCOM;
    uint32_t IPC_RECVADDR;
    uint32_t IPC_RECVDATA;
    uint32_t IPC_LOCALREPLY;
}IPC_RecvCmd_Reg_t;

typedef struct
{
    uint32_t IPC_BOOTSTS;
    uint32_t IPC_BOOTMODE;
    uint32_t IPC_PUMPREQUEST;
}IPC_Boot_Pump_Reg_t;

#if IPC_MSGQ_SUPPORT == 1U
typedef struct
{
    uint32_t command;
    uint32_t address;
    uint32_t dataw1;
    uint32_t dataw2;
}IPC_Message_t;

typedef struct
{
    IPC_Message_t Buffer[IPC_NUM_OF_INTERRUPTS][IPC_BUFFER_SIZE];
    uint16_t      PutWriteIndex[IPC_NUM_OF_INTERRUPTS];
    uint16_t      GetReadIndex[IPC_NUM_OF_INTERRUPTS];
}IPC_PutBuffer_t;

typedef struct
{
    IPC_Message_t Buffer[IPC_NUM_OF_INTERRUPTS][IPC_BUFFER_SIZE];
    uint16_t      GetWriteIndex[IPC_NUM_OF_INTERRUPTS];
    uint16_t      PutReadIndex[IPC_NUM_OF_INTERRUPTS];
}IPC_GetBuffer_t;
#endif

//*****************************************************************************
//
// Internal struct used to store the required information regarding an IPC
// instance
//
//*****************************************************************************
typedef struct
{
    volatile IPC_Flag_Ctr_Reg_t  *IPC_Flag_Ctr_Reg;
    volatile IPC_SendCmd_Reg_t   *IPC_SendCmd_Reg;
    volatile IPC_RecvCmd_Reg_t   *IPC_RecvCmd_Reg;
    volatile IPC_Boot_Pump_Reg_t *IPC_Boot_Pump_Reg;
    uint32_t                      IPC_IntNum[8U];
    uint32_t                      IPC_MsgRam_LtoR;
    uint32_t                      IPC_MsgRam_RtoL;
    uint32_t                      IPC_Offset_Corr;
#if IPC_MSGQ_SUPPORT == 1U
    IPC_PutBuffer_t              *IPC_PutBuffer;
    IPC_GetBuffer_t              *IPC_GetBuffer;
#endif
}IPC_Instance_t;

extern const IPC_Instance_t IPC_Instance[IPC_TOTAL_NUM];

#if IPC_MSGQ_SUPPORT == 1U
//*****************************************************************************
//
// A structure that defines an IPC message queue.  These
// fields are used by the IPC drivers, and normally it is not necessary for
// user software to directly read or write fields in the table.
//
//*****************************************************************************

typedef struct
{
    IPC_Message_t *  PutBuffer;
    uint32_t         PutFlag;
    uint16_t *       PutWriteIndex;
    uint16_t *       PutReadIndex;
    IPC_Message_t *  GetBuffer;
    uint16_t *       GetWriteIndex;
    uint16_t *       GetReadIndex;
} IPC_MessageQueue_t;
#endif

//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************

//*****************************************************************************
//
//! Local core sets Local to Remote IPC Flag
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param flags is the IPC flag mask for the flags being set
//!
//! This function will allow the Local core system to set the designated IPC
//! flags to send to the Remote core system. The \e flags parameter can be any
//! of the IPC flag values: \b IPC_FLAG0 - \b IPC_FLAG31.
//!
//! \return None.
//
//*****************************************************************************
static inline void
IPC_setFlagLtoR(IPC_Type_t ipcType, uint32_t flags)
{
    IPC_Instance[ipcType].IPC_Flag_Ctr_Reg->IPC_SET = flags;
}

//*****************************************************************************
//
//! Local core clears Local to Remote IPC Flag
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param flags is the IPC flag mask for the flags being cleared
//!
//! This function will allow the Local core system to clear the designated IPC
//! flags sent to the Remote core system. The \e flags parameter can be any
//! of the IPC flag values: \b IPC_FLAG0 - \b IPC_FLAG31.
//!
//! \return None.
//
//*****************************************************************************
static inline void
IPC_clearFlagLtoR(IPC_Type_t ipcType, uint32_t flags)
{
    IPC_Instance[ipcType].IPC_Flag_Ctr_Reg->IPC_CLR = flags;
}

//*****************************************************************************
//
//! Local core acknowledges Remote to Local IPC Flag.
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param flags is the IPC flag mask for the flags being acknowledged.
//!
//! This function will allow the Local core system to acknowledge/clear the IPC
//! flag set by the Remote core system. The \e flags parameter can be any of
//! the IPC flag values: \b IPC_FLAG0 - \b IPC_FLAG31.
//!
//! \return None.
//
//*****************************************************************************
static inline void
IPC_ackFlagRtoL(IPC_Type_t ipcType, uint32_t flags)
{
    IPC_Instance[ipcType].IPC_Flag_Ctr_Reg->IPC_ACK = flags;
}

//*****************************************************************************
//
//! Determines whether the given IPC flags are busy or not.
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param flags is the Local to Remote IPC flag masks to check the status of
//!
//! Allows the caller to determine whether the designated Local to Remote
//! IPC flags are pending. The \e flags parameter can be any of the IPC flag
//! values: \b IPC_FLAG0 - \b IPC_FLAG31.
//!
//! \return Returns \b true if the any of the designated IPC flags are busy
//! or \b false if all the designated IPC flags are free.
//
//*****************************************************************************
static inline bool
IPC_isFlagBusyLtoR(IPC_Type_t ipcType, uint32_t flags)
{
    return((IPC_Instance[ipcType].IPC_Flag_Ctr_Reg->IPC_FLG & flags) != 0U);
}

//*****************************************************************************
//
//! Determines whether the given Remote to Local IPC flags are busy or not.
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param flags is the Remote to Local IPC Flag masks to check the status of
//!
//! Allows the caller to determine whether the designated Remote to Local
//! IPC flags are pending. The \e flags parameter can be any of the IPC flag
//! values: \b IPC_FLAG0 - \b IPC_FLAG31.
//!
//! \return Returns \b true if the any of the designated IPC flags are busy
//! or \b false if all the designated IPC flags are free.
//
//*****************************************************************************
static inline bool
IPC_isFlagBusyRtoL(IPC_Type_t ipcType, uint32_t flags)
{
    return((IPC_Instance[ipcType].IPC_Flag_Ctr_Reg->IPC_STS & flags) != 0U);
}

//*****************************************************************************
//
//! Wait for the remote core to send a flag
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param flag is the Remote to Local IPC flag mask to wait for
//!
//! Allows the caller to wait for the Remote to Local flag to be send by
//! the remote core. The \e flags parameter can be any of the IPC flag
//! values: \b IPC_FLAG0 - \b IPC_FLAG31.
//!
//! \return None
//
//*****************************************************************************
static inline void
IPC_waitForFlag(IPC_Type_t ipcType, uint32_t flag)
{
    while((IPC_Instance[ipcType].IPC_Flag_Ctr_Reg->IPC_STS & flag) == 0U)
    {
    }
}

//*****************************************************************************
//
//! Wait for the IPC flag to be acknowledged
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param flag is the IPC flag mask for which ack is pending
//!
//! Allows the caller to wait for the IPC flag to be acknowledged by the
//! remote core. The \e flagsparameter can be any of the IPC flag values:
//! \b IPC_FLAG0 - \b IPC_FLAG31.
//!
//! \return None
//
//*****************************************************************************
static inline void
IPC_waitForAck(IPC_Type_t ipcType, uint32_t flag)
{
    while((IPC_Instance[ipcType].IPC_Flag_Ctr_Reg->IPC_FLG & flag) != 0U)
    {
    }
}

//*****************************************************************************
//
//! Synchronises the two cores
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param flag is the IPC flag mask with which synchronisation is done
//!
//! Allows the local and remote cores to synchronise. Neither core will return
//! from this function call before the other core enters it.
//!
//! \note Must be called with same flag mask on both the cores
//!
//! \return None
//
//*****************************************************************************
static inline void
IPC_sync(IPC_Type_t ipcType, uint32_t flag)
{
    IPC_setFlagLtoR(ipcType, flag);
    IPC_waitForFlag(ipcType, flag);
    IPC_ackFlagRtoL(ipcType, flag);
    IPC_waitForAck(ipcType, flag);
}

//*****************************************************************************
//
//! Initialize IPC
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//
//! This function initializes IPC by clearing all the flags
//!
//! \return None
//
//*****************************************************************************
static inline void
IPC_init(IPC_Type_t ipcType)
{
    IPC_clearFlagLtoR(ipcType, IPC_FLAG_ALL);
}

//*****************************************************************************
//
//! Sends a command to the Remote core
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param flags is the IPC flag mask for the flags to be set
//! \param addrCorrEnable is the flag used to determine whether or not to
//!    convert the addr parameter to remote core's address space
//! \param command is the 32-bit command value
//! \param addr is the 32-bit address to be sent as part of command
//! \param data is the 32-bit data to be sent as part of command
//!
//! Allows the caller to send a command to the remote core. A command consists
//! of a unique command value, a 32-bit address and a 32-bit data. The function
//! also sends the designated flags to the remote core.
//! There may be differences in the address spaces of Local and Remote core.
//! For example in case of F2838X device, the address spaces of C28x core and
//! CM core are different. In case the \e addr refers to an address in the IPC
//! MSG RAM, \e addrCorrEnable param may be used to correct the address mismatch
//!
//! The \e flags parameter can be any of the IPC flag values: \b IPC_FLAG0 -
//! \b IPC_FLAG31.
//! The \e addrCorrEnable parameter can take values IPC_ADDR_CORRECTION_ENABLE
//! (converts the address to remote core's address space) or
//! IPC_ADDR_CORRECTION_DISABLE(does not modify the addr parmeter)
//!
//! The application shall use the function IPC_getResponse to read the response
//! sent by the remote core.
//!
//! \note The application is expected to wait until the the response is
//! received before sending another command.
//!
//! \note \e addrCorrEnable parameter must be kept same on the sending and
//! receiving cores
//!
//! \return Returns \b true if the command is sent properly and \b false if
//! the designated flags were busy and hence command was not sent.
//
//*****************************************************************************
extern bool
IPC_sendCommand(IPC_Type_t ipcType, uint32_t flags, bool addrCorrEnable,
                uint32_t command, uint32_t addr, uint32_t data);

//*****************************************************************************
//
//! Reads a command sent by the Remote core
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param flags is the IPC flag mask for the flags sent by the remote core
//! \param addrCorrEnable is the flag used to determine whether or not to
//!    convert the addr parameter to remote core's address space
//! \param command is the 32-bit pointer at which the command value is read to
//! \param addr is the 32-bit pointer at which address value is read to
//! \param data is the 32-bit pointer at which the data is read to
//!
//! Allows the caller to read a command sent by the remote core. A command
//! consists of a unique command value, a 32-bit address and a 32-bit data.
//! There may be differences in the address spaces of Local and Remote core.
//! For example in case of F2838X device, the address spaces of C28x core and
//! CM core are different. In case the \e addr refers to an address in the IPC
//! MSG RAM, \e addrCorrEnable param may be used to correct the address mismatch
//!
//! The \e flags parameter can be any of the IPC flag values: \b IPC_FLAG0 -
//! \b IPC_FLAG31.
//! The \e addrCorrEnable parameter can take values IPC_ADDR_CORRECTION_ENABLE
//! (converts the address to remote core's address space) or
//! IPC_ADDR_CORRECTION_DISABLE(does not modify the addr parmeter)
//!
//! \note The application is expected to acknowledge the flag and send a
//! response (if needed) after reading the command
//!
//! \note \e addrCorrEnable parameter must be kept same on the sending and
//! receiving cores
//!
//! \return Returns \b true if the command is read properly and \b false if
//!  the designated flags were empty and hence command was not read.
//
//*****************************************************************************
extern bool
IPC_readCommand(IPC_Type_t ipcType, uint32_t flags, bool addrCorrEnable,
                uint32_t *command, uint32_t *addr, uint32_t *data);

//*****************************************************************************
//
//! Sends the response to the command sent by remote core.
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param data is the 32-bit value of the response to be sent
//!
//! Allows the caller to send a response to the command previously sent by the
//! remote core
//!
//! \return None.
//
//*****************************************************************************
static inline void
IPC_sendResponse(IPC_Type_t ipcType, uint32_t data)
{
    IPC_Instance[ipcType].IPC_RecvCmd_Reg->IPC_LOCALREPLY = data;
}

//*****************************************************************************
//
//! Reads the response from the remote core.
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//!
//! Allows the caller to read the response sent by the remote core to the
//! command previously sent by the local core
//!
//! \return the 32-bit value of the response.
//
//*****************************************************************************
static inline uint32_t
IPC_getResponse(IPC_Type_t ipcType)
{
    return(IPC_Instance[ipcType].IPC_SendCmd_Reg->IPC_REMOTEREPLY);
}

//*****************************************************************************
//
//! Sets the BOOTMODE register.
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param mode is the 32-bit value to be set
//!
//! Allows the caller to set the BOOTMODE register.
//!
//! \note This function shall be called by CPU1 only.
//! \note Boot registers are not available in CPU2<->CM IPC instance
//!
//! \return None
//
//*****************************************************************************
static inline void
IPC_setBootMode(IPC_Type_t ipcType, uint32_t mode)
{
    ASSERT((ipcType == IPC_CPU1_L_CPU2_R) || (ipcType == IPC_CPU1_L_CM_R));

    IPC_Instance[ipcType].IPC_Boot_Pump_Reg->IPC_BOOTMODE = mode;
}

//*****************************************************************************
//
//! Reads the BOOTMODE register.
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//!
//! Allows the caller to read the BOOTMODE register.
//!
//! \note Boot registers are not available in CPU2<->CM IPC instance
//!
//! \return 32-bit value of the BOOOTMODE register
//
//*****************************************************************************
static inline uint32_t
IPC_getBootMode(IPC_Type_t ipcType)
{
    return(IPC_Instance[ipcType].IPC_Boot_Pump_Reg->IPC_BOOTMODE);
}

//*****************************************************************************
//
//! Sets the BOOTSTS register.
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param status is the 32-bit value to be set
//!
//! Allows the caller to set the BOOTSTS register.
//!
//! \note This function shall be called by CPU2 and CM only
//!
//! \note This function shall be called by CPU2 and CM only. Boot registers
//! are not  available in CPU2<->CM IPC instance
//!
//! \return None.
//
//*****************************************************************************
static inline void
IPC_setBootStatus(IPC_Type_t ipcType, uint32_t status)
{
    ASSERT(ipcType == IPC_CPU2_L_CPU1_R);

    IPC_Instance[ipcType].IPC_Boot_Pump_Reg->IPC_BOOTSTS = status;
}

//*****************************************************************************
//
//! Reads the BOOTSTS register.
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//!
//! Allows the caller to set the BOOTMODE register.
//!
//! \note Boot registers are not available in CPU2<->CM IPC instance
//!
//! \return 32-bit value of the BOOOTSTS register
//
//*****************************************************************************
static inline uint32_t
IPC_getBootStatus(IPC_Type_t ipcType)
{
    return(IPC_Instance[ipcType].IPC_Boot_Pump_Reg->IPC_BOOTSTS);
}

//*****************************************************************************
//
//! Reads the timestamp counter value.
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//!
//! Allows the caller to read the IPC timestamp counter value.
//!
//! \return 64-bit counter value.
//
//*****************************************************************************
static inline uint64_t
IPC_getCounter(IPC_Type_t ipcType)
{
    //
    // Get the Counter High and Low values. Read to the Counter low register
    // saves the value of Counter High register.
    //
    uint32_t ctrL = IPC_Instance[ipcType].IPC_Flag_Ctr_Reg->IPC_COUNTERL;
    uint32_t ctrH = IPC_Instance[ipcType].IPC_Flag_Ctr_Reg->IPC_COUNTERH;

    //
    // Return the 64-bit value of the counter
    //
    return(((uint64_t)ctrH << 32) | ((uint64_t)ctrL));
}

//*****************************************************************************
//
//! Registers an interrupt handler for IPC
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param ipcInt is the Flag number for which interrupt is being registered
//! \param pfnHandler is the pointer to ISR function
//!
//! This function registers the handler to be called when an IPC interrupt
//! occurs.  This function enables the global interrupt in the interrupt
//! controller.
//! The \e ipcInt parameter can be any of the IPC flag values:\b IPC_INT0 -
//! \b IPC_INT7. IPC_INT0 corresponds to IPC Flag 0 interrupt and so on.
//
//*****************************************************************************
extern void
IPC_registerInterrupt(IPC_Type_t ipcType, uint32_t ipcInt,
                      void (*pfnHandler)(void));

//*****************************************************************************
//
//! Unregisters an interrupt handler for IPC
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param ipcInt is the Flag number for which interrupt is being unregistered
//!
//! This function clears the handler to be called when an IPC interrupt
//! occurs.  This function also masks off the interrupt in the interrupt
//! controller so that the interrupt handler no longer is called.
//! The \e ipcInt parameter can be any of the IPC flag values:\b IPC_INT0 -
//! \b IPC_INT7. IPC_INT0 corresponds to IPC Flag 0 interrupt and so on.
//
//*****************************************************************************
extern void
IPC_unregisterInterrupt(IPC_Type_t ipcType, uint32_t ipcInt);

#if IPC_MSGQ_SUPPORT == 1U
//*****************************************************************************
//
//! Initializes the IPC message queue
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param msgQueue specifies the address of a \e IPC_MessageQueue_t instance
//! \param ipcInt_L specifies the interrupt number on the local core used by
//!                 the message queue .
//! \param ipcInt_R specifies the interrupt number on the remote core used by
//!                 the message queue.
//!
//! This function initializes the IPC message queue with circular buffer
//! and index addresses for an IPC interrupt pair. The
//! \e ipcInt_L and \e ipcInt_R parameters can be one of the following values:
//! \b IPC_INT0, \b IPC_INT1, \b IPC_INT2, \b IPC_INT3.
//!
//! \note If an interrupt is currently in use by an \e IPC_MessageQueue_t
//! instance, that particular interrupt should not be tied to a second
//! \e IPC_MessageQueue_t instance.
//!
//! \note For a particular ipcInt_L - ipcInt_R pair, there must be an instance
//! of IPC_MessageQueue_t defined and initialized on both the locakl and remote
//! systems.
//!
//! \return None.
//
//*****************************************************************************
extern void
IPC_initMessageQueue(IPC_Type_t ipcType, volatile IPC_MessageQueue_t *msgQueue,
                     uint32_t ipcInt_L, uint32_t ipcInt_R);

//*****************************************************************************
//
//! Sends a message into the messageQueue.
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param msgQueue specifies the address of a \e IPC_MessageQueue_t instance
//! \param addrCorrEnable is the flag used to determine whether or not to
//!                 convert the addr parameter to remote core's address space
//! \param msg specifies the address of the \e IPC_Message_t instance to be
//!                 sent to message queue.
//! \param block specifies whether to allow function to block until the buffer
//!                 has a free slot
//!
//! This function checks if there is a free slot in the message queue. If so, it
//! puts the message pointed to by \e msg into the free and sets the
//! appropriate IPC interrupt flag
//!
//! The \e addrCorrEnable parameter can take values IPC_ADDR_CORRECTION_ENABLE
//! (converts the address to remote core's address space) or
//! IPC_ADDR_CORRECTION_DISABLE(does not modify the addr parmeter)
//! The \e block parameter can be one of the following values:
//! \b IPC_BLOCKING_CALL or \b IPC_NONBLOCKING_CALL.
//!
//! \return \b false if the queue is full. \b true if the message is
//! successfully sent.
//
//*****************************************************************************
extern bool
IPC_sendMessageToQueue(IPC_Type_t ipcType,
                       volatile IPC_MessageQueue_t *msgQueue,
                       bool addrCorrEnable, IPC_Message_t *msg, bool block);

//*****************************************************************************
//
//! Reads a message from the messageQueue.
//!
//! \param ipcType is the enum corresponding to the IPC instance used
//! \param msgQueue specifies the address of a \e IPC_MessageQueue_t instance
//! \param addrCorrEnable is the flag used to determine whether or not to
//!                 convert the addr parameter to remote core's address space
//! \param msg specifies the address of the \e IPC_Message_t instance to which
//!                 the message needs to be read
//! \param block specifies whether to allow function to block until a message
//!                 is available in the message queue
//!
//! This function checks if there is a message in the message queue. If so, it
//! reads the message and writes to the address pointed to by \e msg into.
//!
//! The \e addrCorrEnable parameter can take values IPC_ADDR_CORRECTION_ENABLE
//! (converts the address to remote core's address space) or
//! IPC_ADDR_CORRECTION_DISABLE(does not modify the addr parmeter)
//! The \e block parameter can be one of the following values:
//! \b IPC_BLOCKING_CALL or \b IPC_NONBLOCKING_CALL.
//!
//! \return \b false if the queue is empty. \b true if the message successfully
//! read.
//
//*****************************************************************************
extern bool
IPC_readMessageFromQueue(IPC_Type_t ipcType,
                         volatile IPC_MessageQueue_t *msgQueue,
                         bool addrCorrEnable, IPC_Message_t *msg, bool block);
#endif
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

#endif // IPC_H
