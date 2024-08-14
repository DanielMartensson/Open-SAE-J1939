//###########################################################################
//
// FILE:    hw_ipc.h
//
// TITLE:   Definitions for the IPC registers.
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

#ifndef HW_IPC_H
#define HW_IPC_H

//*************************************************************************************************
//
// The following are defines for the IPC register offsets
//
//*************************************************************************************************
#define IPC_O_CPU1TOCMIPCACK        0x0U    // CPU1TOCMIPCACK Register
#define IPC_O_CMTOCPU1IPCSTS        0x2U    // CMTOCPU1IPCSTS Register
#define IPC_O_CPU1TOCMIPCSET        0x4U    // CPU1TOCMIPCSET Register
#define IPC_O_CPU1TOCMIPCCLR        0x6U    // CPU1TOCMIPCCLR Register
#define IPC_O_CPU1TOCMIPCFLG        0x8U    // CPU1TOCMIPCFLG Register
#define IPC_O_COUNTERL              0xCU    // IPCCOUNTERL Register
#define IPC_O_COUNTERH              0xEU    // IPCCOUNTERH Register
#define IPC_O_CPU1TOCMIPCSENDCOM    0x10U   // CPU1TOCMIPCSENDCOM Register
#define IPC_O_CPU1TOCMIPCSENDADDR   0x12U   // CPU1TOCMIPCSENDADDR Register
#define IPC_O_CPU1TOCMIPCSENDDATA   0x14U   // CPU1TOCMIPCSENDDATA Register
#define IPC_O_CMTOCPU1IPCREPLY      0x16U   // CMTOCPU1IPCREPLY Register
#define IPC_O_CMTOCPU1IPCRECVCOM    0x18U   // CMTOCPU1IPCRECVCOM Register
#define IPC_O_CMTOCPU1IPCRECVADDR   0x1AU   // CMTOCPU1IPCRECVADDR Register
#define IPC_O_CMTOCPU1IPCRECVDATA   0x1CU   // CMTOCPU1IPCRECVDATA Register
#define IPC_O_CPU1TOCMIPCREPLY      0x1EU   // CPU1TOCMIPCREPLY Register
#define IPC_O_CMTOCPU1IPCBOOTSTS    0x20U   // CMTOCPU1IPCBOOTSTS Register
#define IPC_O_CPU1TOCMIPCBOOTMODE   0x22U   // CPU1TOCMIPCBOOTMODE Register

#define IPC_O_CMTOCPU1IPCACK        0x0U    // CMTOCPU1IPCACK Register
#define IPC_O_CPU1TOCMIPCSTS        0x2U    // CPU1TOCMIPCSTS Register
#define IPC_O_CMTOCPU1IPCSET        0x4U    // CMTOCPU1IPCSET Register
#define IPC_O_CMTOCPU1IPCCLR        0x6U    // CMTOCPU1IPCCLR Register
#define IPC_O_CMTOCPU1IPCFLG        0x8U    // CMTOCPU1IPCFLG Register
#define IPC_O_COUNTERL              0xCU    // IPCCOUNTERL Register
#define IPC_O_COUNTERH              0xEU    // IPCCOUNTERH Register
#define IPC_O_CPU1TOCMIPCRECVCOM    0x10U   // CPU1TOCMIPCRECVCOM Register
#define IPC_O_CPU1TOCMIPCRECVADDR   0x12U   // CPU1TOCMIPCRECVADDR Register
#define IPC_O_CPU1TOCMIPCRECVDATA   0x14U   // CPU1TOCMIPCRECVDATA Register
#define IPC_O_CMTOCPU1IPCREPLY      0x16U   // CMTOCPU1IPCREPLY Register
#define IPC_O_CMTOCPU1IPCSENDCOM    0x18U   // CMTOCPU1IPCSENDCOM Register
#define IPC_O_CMTOCPU1IPCSENDADDR   0x1AU   // CMTOCPU1IPCSENDADDR Register
#define IPC_O_CMTOCPU1IPCSENDDATA   0x1CU   // CMTOCPU1IPCSENDDATA Register
#define IPC_O_CPU1TOCMIPCREPLY      0x1EU   // CPU1TOCMIPCREPLY Register
#define IPC_O_CMTOCPU1IPCBOOTSTS    0x20U   // CMTOCPU1IPCBOOTSTS Register
#define IPC_O_CPU1TOCMIPCBOOTMODE   0x22U   // CPU1TOCMIPCBOOTMODE Register
#define IPC_O_PUMPREQUEST           0x24U   // PUMPREQUEST Register

#define IPC_O_CPU2TOCMIPCACK        0x0U    // CPU2TOCMIPCACK Register
#define IPC_O_CMTOCPU2IPCSTS        0x2U    // CMTOCPU2IPCSTS Register
#define IPC_O_CPU2TOCMIPCSET        0x4U    // CPU2TOCMIPCSET Register
#define IPC_O_CPU2TOCMIPCCLR        0x6U    // CPU2TOCMIPCCLR Register
#define IPC_O_CPU2TOCMIPCFLG        0x8U    // CPU2TOCMIPCFLG Register
#define IPC_O_COUNTERL              0xCU    // IPCCOUNTERL Register
#define IPC_O_COUNTERH              0xEU    // IPCCOUNTERH Register
#define IPC_O_CPU2TOCMIPCSENDCOM    0x10U   // CPU2TOCMIPCSENDCOM Register
#define IPC_O_CPU2TOCMIPCSENDADDR   0x12U   // CPU2TOCMIPCSENDADDR Register
#define IPC_O_CPU2TOCMIPCSENDDATA   0x14U   // CPU2TOCMIPCSENDDATA Register
#define IPC_O_CMTOCPU2IPCREPLY      0x16U   // CMTOCPU2IPCREPLY Register
#define IPC_O_CMTOCPU2IPCRECVCOM    0x18U   // CMTOCPU2IPCRECVCOM Register
#define IPC_O_CMTOCPU2IPCRECVADDR   0x1AU   // CMTOCPU2IPCRECVADDR Register
#define IPC_O_CMTOCPU2IPCRECVDATA   0x1CU   // CMTOCPU2IPCRECVDATA Register
#define IPC_O_CPU2TOCMIPCREPLY      0x1EU   // CPU2TOCMIPCREPLY Register

#define IPC_O_CMTOCPU2IPCACK        0x0U    // CMTOCPU2IPCACK Register
#define IPC_O_CPU2TOCMIPCSTS        0x2U    // CPU2TOCMIPCSTS Register
#define IPC_O_CMTOCPU2IPCSET        0x4U    // CMTOCPU2IPCSET Register
#define IPC_O_CMTOCPU2IPCCLR        0x6U    // CMTOCPU2IPCCLR Register
#define IPC_O_CMTOCPU2IPCFLG        0x8U    // CMTOCPU2IPCFLG Register
#define IPC_O_COUNTERL              0xCU    // IPCCOUNTERL Register
#define IPC_O_COUNTERH              0xEU    // IPCCOUNTERH Register
#define IPC_O_CPU2TOCMIPCRECVCOM    0x10U   // CPU2TOCMIPCRECVCOM Register
#define IPC_O_CPU2TOCMIPCRECVADDR   0x12U   // CPU2TOCMIPCRECVADDR Register
#define IPC_O_CPU2TOCMIPCRECVDATA   0x14U   // CPU2TOCMIPCRECVDATA Register
#define IPC_O_CMTOCPU2IPCREPLY      0x16U   // CMTOCPU2IPCREPLY Register
#define IPC_O_CMTOCPU2IPCSENDCOM    0x18U   // CMTOCPU2IPCSENDCOM Register
#define IPC_O_CMTOCPU2IPCSENDADDR   0x1AU   // CMTOCPU2IPCSENDADDR Register
#define IPC_O_CMTOCPU2IPCSENDDATA   0x1CU   // CMTOCPU2IPCSENDDATA Register
#define IPC_O_CPU2TOCMIPCREPLY      0x1EU   // CPU2TOCMIPCREPLY Register

#define IPC_O_CPU1TOCPU2IPCACK        0x0U    // CPU1TOCPU2IPCACK Register
#define IPC_O_CPU2TOCPU1IPCSTS        0x2U    // CPU2TOCPU1IPCSTS Register
#define IPC_O_CPU1TOCPU2IPCSET        0x4U    // CPU1TOCPU2IPCSET Register
#define IPC_O_CPU1TOCPU2IPCCLR        0x6U    // CPU1TOCPU2IPCCLR Register
#define IPC_O_CPU1TOCPU2IPCFLG        0x8U    // CPU1TOCPU2IPCFLG Register
#define IPC_O_COUNTERL                0xCU    // IPCCOUNTERL Register
#define IPC_O_COUNTERH                0xEU    // IPCCOUNTERH Register
#define IPC_O_CPU1TOCPU2IPCSENDCOM    0x10U   // CPU1TOCPU2IPCSENDCOM Register
#define IPC_O_CPU1TOCPU2IPCSENDADDR   0x12U   // CPU1TOCPU2IPCSENDADDR Register
#define IPC_O_CPU1TOCPU2IPCSENDDATA   0x14U   // CPU1TOCPU2IPCSENDDATA Register
#define IPC_O_CPU2TOCPU1IPCREPLY      0x16U   // CPU2TOCPU1IPCREPLY Register
#define IPC_O_CPU2TOCPU1IPCRECVCOM    0x18U   // CPU2TOCPU1IPCRECVCOM Register
#define IPC_O_CPU2TOCPU1IPCRECVADDR   0x1AU   // CPU2TOCPU1IPCRECVADDR Register
#define IPC_O_CPU2TOCPU1IPCRECVDATA   0x1CU   // CPU2TOCPU1IPCRECVDATA Register
#define IPC_O_CPU1TOCPU2IPCREPLY      0x1EU   // CPU1TOCPU2IPCREPLY Register
#define IPC_O_CPU2TOCPU1IPCBOOTSTS    0x20U   // CPU2TOCPU1IPCBOOTSTS Register
#define IPC_O_CPU1TOCPU2IPCBOOTMODE   0x22U   // CPU1TOCPU2IPCBOOTMODE Register
#define IPC_O_PUMPREQUEST             0x24U   // PUMPREQUEST Register

#define IPC_O_CPU2TOCPU1IPCACK        0x0U    // CPU2TOCPU1IPCACK Register
#define IPC_O_CPU1TOCPU2IPCSTS        0x2U    // CPU1TOCPU2IPCSTS Register
#define IPC_O_CPU2TOCPU1IPCSET        0x4U    // CPU2TOCPU1IPCSET Register
#define IPC_O_CPU2TOCPU1IPCCLR        0x6U    // CPU2TOCPU1IPCCLR Register
#define IPC_O_CPU2TOCPU1IPCFLG        0x8U    // CPU2TOCPU1IPCFLG Register
#define IPC_O_COUNTERL                0xCU    // IPCCOUNTERL Register
#define IPC_O_COUNTERH                0xEU    // IPCCOUNTERH Register
#define IPC_O_CPU1TOCPU2IPCRECVCOM    0x10U   // CPU1TOCPU2IPCRECVCOM Register
#define IPC_O_CPU1TOCPU2IPCRECVADDR   0x12U   // CPU1TOCPU2IPCRECVADDR Register
#define IPC_O_CPU1TOCPU2IPCRECVDATA   0x14U   // CPU1TOCPU2IPCRECVDATA Register
#define IPC_O_CPU2TOCPU1IPCREPLY      0x16U   // CPU2TOCPU1IPCREPLY Register
#define IPC_O_CPU2TOCPU1IPCSENDCOM    0x18U   // CPU2TOCPU1IPCSENDCOM Register
#define IPC_O_CPU2TOCPU1IPCSENDADDR   0x1AU   // CPU2TOCPU1IPCSENDADDR Register
#define IPC_O_CPU2TOCPU1IPCSENDDATA   0x1CU   // CPU2TOCPU1IPCSENDDATA Register
#define IPC_O_CPU1TOCPU2IPCREPLY      0x1EU   // CPU1TOCPU2IPCREPLY Register
#define IPC_O_CPU2TOCPU1IPCBOOTSTS    0x20U   // CPU2TOCPU1IPCBOOTSTS Register
#define IPC_O_CPU1TOCPU2IPCBOOTMODE   0x22U   // CPU1TOCPU2IPCBOOTMODE Register
#define IPC_O_PUMPREQUEST             0x24U   // PUMPREQUEST Register


//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU1TOCMIPCACK register
//
//*************************************************************************************************
#define IPC_CPU1TOCMIPCACK_IPC0    0x1U          // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC0 bit
#define IPC_CPU1TOCMIPCACK_IPC1    0x2U          // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC1 bit
#define IPC_CPU1TOCMIPCACK_IPC2    0x4U          // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC2 bit
#define IPC_CPU1TOCMIPCACK_IPC3    0x8U          // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC3 bit
#define IPC_CPU1TOCMIPCACK_IPC4    0x10U         // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC4 bit
#define IPC_CPU1TOCMIPCACK_IPC5    0x20U         // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC5 bit
#define IPC_CPU1TOCMIPCACK_IPC6    0x40U         // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC6 bit
#define IPC_CPU1TOCMIPCACK_IPC7    0x80U         // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC7 bit
#define IPC_CPU1TOCMIPCACK_IPC8    0x100U        // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC8 bit
#define IPC_CPU1TOCMIPCACK_IPC9    0x200U        // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC9 bit
#define IPC_CPU1TOCMIPCACK_IPC10   0x400U        // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC10 bit
#define IPC_CPU1TOCMIPCACK_IPC11   0x800U        // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC11 bit
#define IPC_CPU1TOCMIPCACK_IPC12   0x1000U       // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC12 bit
#define IPC_CPU1TOCMIPCACK_IPC13   0x2000U       // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC13 bit
#define IPC_CPU1TOCMIPCACK_IPC14   0x4000U       // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC14 bit
#define IPC_CPU1TOCMIPCACK_IPC15   0x8000U       // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC15 bit
#define IPC_CPU1TOCMIPCACK_IPC16   0x10000U      // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC16 bit
#define IPC_CPU1TOCMIPCACK_IPC17   0x20000U      // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC17 bit
#define IPC_CPU1TOCMIPCACK_IPC18   0x40000U      // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC18 bit
#define IPC_CPU1TOCMIPCACK_IPC19   0x80000U      // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC19 bit
#define IPC_CPU1TOCMIPCACK_IPC20   0x100000U     // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC20 bit
#define IPC_CPU1TOCMIPCACK_IPC21   0x200000U     // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC21 bit
#define IPC_CPU1TOCMIPCACK_IPC22   0x400000U     // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC22 bit
#define IPC_CPU1TOCMIPCACK_IPC23   0x800000U     // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC23 bit
#define IPC_CPU1TOCMIPCACK_IPC24   0x1000000U    // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC24 bit
#define IPC_CPU1TOCMIPCACK_IPC25   0x2000000U    // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC25 bit
#define IPC_CPU1TOCMIPCACK_IPC26   0x4000000U    // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC26 bit
#define IPC_CPU1TOCMIPCACK_IPC27   0x8000000U    // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC27 bit
#define IPC_CPU1TOCMIPCACK_IPC28   0x10000000U   // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC28 bit
#define IPC_CPU1TOCMIPCACK_IPC29   0x20000000U   // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC29 bit
#define IPC_CPU1TOCMIPCACK_IPC30   0x40000000U   // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC30 bit
#define IPC_CPU1TOCMIPCACK_IPC31   0x80000000U   // Acknowledgement from CPU1 to
                                                 // CMTOCPU1IPCFLG.IPC31 bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMTOCPU1IPCSTS register
//
//*************************************************************************************************
#define IPC_CMTOCPU1IPCSTS_IPC0    0x1U          // IPC0 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC1    0x2U          // IPC1 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC2    0x4U          // IPC2 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC3    0x8U          // IPC3 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC4    0x10U         // IPC4 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC5    0x20U         // IPC5 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC6    0x40U         // IPC6 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC7    0x80U         // IPC7 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC8    0x100U        // IPC8 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC9    0x200U        // IPC9 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC10   0x400U        // IPC10 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC11   0x800U        // IPC11 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC12   0x1000U       // IPC12 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC13   0x2000U       // IPC13 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC14   0x4000U       // IPC14 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC15   0x8000U       // IPC15 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC16   0x10000U      // IPC16 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC17   0x20000U      // IPC17 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC18   0x40000U      // IPC18 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC19   0x80000U      // IPC19 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC20   0x100000U     // IPC20 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC21   0x200000U     // IPC21 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC22   0x400000U     // IPC22 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC23   0x800000U     // IPC23 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC24   0x1000000U    // IPC24 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC25   0x2000000U    // IPC25 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC26   0x4000000U    // IPC26 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC27   0x8000000U    // IPC27 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC28   0x10000000U   // IPC28 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC29   0x20000000U   // IPC29 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC30   0x40000000U   // IPC30 Request from  CM to CPU1
#define IPC_CMTOCPU1IPCSTS_IPC31   0x80000000U   // IPC31 Request from  CM to CPU1

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU1TOCMIPCSET register
//
//*************************************************************************************************
#define IPC_CPU1TOCMIPCSET_IPC0    0x1U          // Set CPU1TOCMIPCFLG.IPC0 Flag
#define IPC_CPU1TOCMIPCSET_IPC1    0x2U          // Set CPU1TOCMIPCFLG.IPC1 Flag
#define IPC_CPU1TOCMIPCSET_IPC2    0x4U          // Set CPU1TOCMIPCFLG.IPC2 Flag
#define IPC_CPU1TOCMIPCSET_IPC3    0x8U          // Set CPU1TOCMIPCFLG.IPC3 Flag
#define IPC_CPU1TOCMIPCSET_IPC4    0x10U         // Set CPU1TOCMIPCFLG.IPC4 Flag
#define IPC_CPU1TOCMIPCSET_IPC5    0x20U         // Set CPU1TOCMIPCFLG.IPC5 Flag
#define IPC_CPU1TOCMIPCSET_IPC6    0x40U         // Set CPU1TOCMIPCFLG.IPC6 Flag
#define IPC_CPU1TOCMIPCSET_IPC7    0x80U         // Set CPU1TOCMIPCFLG.IPC7 Flag
#define IPC_CPU1TOCMIPCSET_IPC8    0x100U        // Set CPU1TOCMIPCFLG.IPC8 Flag
#define IPC_CPU1TOCMIPCSET_IPC9    0x200U        // Set CPU1TOCMIPCFLG.IPC9 Flag
#define IPC_CPU1TOCMIPCSET_IPC10   0x400U        // Set CPU1TOCMIPCFLG.IPC10 Flag
#define IPC_CPU1TOCMIPCSET_IPC11   0x800U        // Set CPU1TOCMIPCFLG.IPC11 Flag
#define IPC_CPU1TOCMIPCSET_IPC12   0x1000U       // Set CPU1TOCMIPCFLG.IPC12 Flag
#define IPC_CPU1TOCMIPCSET_IPC13   0x2000U       // Set CPU1TOCMIPCFLG.IPC13 Flag
#define IPC_CPU1TOCMIPCSET_IPC14   0x4000U       // Set CPU1TOCMIPCFLG.IPC14 Flag
#define IPC_CPU1TOCMIPCSET_IPC15   0x8000U       // Set CPU1TOCMIPCFLG.IPC15 Flag
#define IPC_CPU1TOCMIPCSET_IPC16   0x10000U      // Set CPU1TOCMIPCFLG.IPC16 Flag
#define IPC_CPU1TOCMIPCSET_IPC17   0x20000U      // Set CPU1TOCMIPCFLG.IPC17 Flag
#define IPC_CPU1TOCMIPCSET_IPC18   0x40000U      // Set CPU1TOCMIPCFLG.IPC18 Flag
#define IPC_CPU1TOCMIPCSET_IPC19   0x80000U      // Set CPU1TOCMIPCFLG.IPC19 Flag
#define IPC_CPU1TOCMIPCSET_IPC20   0x100000U     // Set CPU1TOCMIPCFLG.IPC20 Flag
#define IPC_CPU1TOCMIPCSET_IPC21   0x200000U     // Set CPU1TOCMIPCFLG.IPC21 Flag
#define IPC_CPU1TOCMIPCSET_IPC22   0x400000U     // Set CPU1TOCMIPCFLG.IPC22 Flag
#define IPC_CPU1TOCMIPCSET_IPC23   0x800000U     // Set CPU1TOCMIPCFLG.IPC23 Flag
#define IPC_CPU1TOCMIPCSET_IPC24   0x1000000U    // Set CPU1TOCMIPCFLG.IPC24 Flag
#define IPC_CPU1TOCMIPCSET_IPC25   0x2000000U    // Set CPU1TOCMIPCFLG.IPC25 Flag
#define IPC_CPU1TOCMIPCSET_IPC26   0x4000000U    // Set CPU1TOCMIPCFLG.IPC26 Flag
#define IPC_CPU1TOCMIPCSET_IPC27   0x8000000U    // Set CPU1TOCMIPCFLG.IPC27 Flag
#define IPC_CPU1TOCMIPCSET_IPC28   0x10000000U   // Set CPU1TOCMIPCFLG.IPC28 Flag
#define IPC_CPU1TOCMIPCSET_IPC29   0x20000000U   // Set CPU1TOCMIPCFLG.IPC29 Flag
#define IPC_CPU1TOCMIPCSET_IPC30   0x40000000U   // Set CPU1TOCMIPCFLG.IPC30 Flag
#define IPC_CPU1TOCMIPCSET_IPC31   0x80000000U   // Set CPU1TOCMIPCFLG.IPC31 Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU1TOCMIPCCLR register
//
//*************************************************************************************************
#define IPC_CPU1TOCMIPCCLR_IPC0    0x1U          // Clear CPU1TOCMIPCFLG.IPC0 Flag
#define IPC_CPU1TOCMIPCCLR_IPC1    0x2U          // Clear CPU1TOCMIPCFLG.IPC1 Flag
#define IPC_CPU1TOCMIPCCLR_IPC2    0x4U          // Clear CPU1TOCMIPCFLG.IPC2 Flag
#define IPC_CPU1TOCMIPCCLR_IPC3    0x8U          // Clear CPU1TOCMIPCFLG.IPC3 Flag
#define IPC_CPU1TOCMIPCCLR_IPC4    0x10U         // Clear CPU1TOCMIPCFLG.IPC4 Flag
#define IPC_CPU1TOCMIPCCLR_IPC5    0x20U         // Clear CPU1TOCMIPCFLG.IPC5 Flag
#define IPC_CPU1TOCMIPCCLR_IPC6    0x40U         // Clear CPU1TOCMIPCFLG.IPC6 Flag
#define IPC_CPU1TOCMIPCCLR_IPC7    0x80U         // Clear CPU1TOCMIPCFLG.IPC7 Flag
#define IPC_CPU1TOCMIPCCLR_IPC8    0x100U        // Clear CPU1TOCMIPCFLG.IPC8 Flag
#define IPC_CPU1TOCMIPCCLR_IPC9    0x200U        // Clear CPU1TOCMIPCFLG.IPC9 Flag
#define IPC_CPU1TOCMIPCCLR_IPC10   0x400U        // Clear CPU1TOCMIPCFLG.IPC10 Flag
#define IPC_CPU1TOCMIPCCLR_IPC11   0x800U        // Clear CPU1TOCMIPCFLG.IPC11 Flag
#define IPC_CPU1TOCMIPCCLR_IPC12   0x1000U       // Clear CPU1TOCMIPCFLG.IPC12 Flag
#define IPC_CPU1TOCMIPCCLR_IPC13   0x2000U       // Clear CPU1TOCMIPCFLG.IPC13 Flag
#define IPC_CPU1TOCMIPCCLR_IPC14   0x4000U       // Clear CPU1TOCMIPCFLG.IPC14 Flag
#define IPC_CPU1TOCMIPCCLR_IPC15   0x8000U       // Clear CPU1TOCMIPCFLG.IPC15 Flag
#define IPC_CPU1TOCMIPCCLR_IPC16   0x10000U      // Clear CPU1TOCMIPCFLG.IPC16 Flag
#define IPC_CPU1TOCMIPCCLR_IPC17   0x20000U      // Clear CPU1TOCMIPCFLG.IPC17 Flag
#define IPC_CPU1TOCMIPCCLR_IPC18   0x40000U      // Clear CPU1TOCMIPCFLG.IPC18 Flag
#define IPC_CPU1TOCMIPCCLR_IPC19   0x80000U      // Clear CPU1TOCMIPCFLG.IPC19 Flag
#define IPC_CPU1TOCMIPCCLR_IPC20   0x100000U     // Clear CPU1TOCMIPCFLG.IPC20 Flag
#define IPC_CPU1TOCMIPCCLR_IPC21   0x200000U     // Clear CPU1TOCMIPCFLG.IPC21 Flag
#define IPC_CPU1TOCMIPCCLR_IPC22   0x400000U     // Clear CPU1TOCMIPCFLG.IPC22 Flag
#define IPC_CPU1TOCMIPCCLR_IPC23   0x800000U     // Clear CPU1TOCMIPCFLG.IPC23 Flag
#define IPC_CPU1TOCMIPCCLR_IPC24   0x1000000U    // Clear CPU1TOCMIPCFLG.IPC24 Flag
#define IPC_CPU1TOCMIPCCLR_IPC25   0x2000000U    // Clear CPU1TOCMIPCFLG.IPC25 Flag
#define IPC_CPU1TOCMIPCCLR_IPC26   0x4000000U    // Clear CPU1TOCMIPCFLG.IPC26 Flag
#define IPC_CPU1TOCMIPCCLR_IPC27   0x8000000U    // Clear CPU1TOCMIPCFLG.IPC27 Flag
#define IPC_CPU1TOCMIPCCLR_IPC28   0x10000000U   // Clear CPU1TOCMIPCFLG.IPC28 Flag
#define IPC_CPU1TOCMIPCCLR_IPC29   0x20000000U   // Clear CPU1TOCMIPCFLG.IPC29 Flag
#define IPC_CPU1TOCMIPCCLR_IPC30   0x40000000U   // Clear CPU1TOCMIPCFLG.IPC30 Flag
#define IPC_CPU1TOCMIPCCLR_IPC31   0x80000000U   // Clear CPU1TOCMIPCFLG.IPC31 Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU1TOCMIPCFLG register
//
//*************************************************************************************************
#define IPC_CPU1TOCMIPCFLG_IPC0    0x1U          // CPU1 to CM IPC0 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC1    0x2U          // CPU1 to CM IPC1 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC2    0x4U          // CPU1 to CM IPC2 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC3    0x8U          // CPU1 to CM IPC3 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC4    0x10U         // CPU1 to CM IPC4 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC5    0x20U         // CPU1 to CM IPC5 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC6    0x40U         // CPU1 to CM IPC6 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC7    0x80U         // CPU1 to CM IPC7 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC8    0x100U        // CPU1 to CM IPC8 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC9    0x200U        // CPU1 to CM IPC9 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC10   0x400U        // CPU1 to CM IPC10 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC11   0x800U        // CPU1 to CM IPC11 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC12   0x1000U       // CPU1 to CM IPC12 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC13   0x2000U       // CPU1 to CM IPC13 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC14   0x4000U       // CPU1 to CM IPC14 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC15   0x8000U       // CPU1 to CM IPC15 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC16   0x10000U      // CPU1 to CM IPC16 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC17   0x20000U      // CPU1 to CM IPC17 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC18   0x40000U      // CPU1 to CM IPC18 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC19   0x80000U      // CPU1 to CM IPC19 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC20   0x100000U     // CPU1 to CM IPC20 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC21   0x200000U     // CPU1 to CM IPC21 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC22   0x400000U     // CPU1 to CM IPC22 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC23   0x800000U     // CPU1 to CM IPC23 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC24   0x1000000U    // CPU1 to CM IPC24 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC25   0x2000000U    // CPU1 to CM IPC25 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC26   0x4000000U    // CPU1 to CM IPC26 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC27   0x8000000U    // CPU1 to CM IPC27 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC28   0x10000000U   // CPU1 to CM IPC28 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC29   0x20000000U   // CPU1 to CM IPC29 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC30   0x40000000U   // CPU1 to CM IPC30 Flag Status
#define IPC_CPU1TOCMIPCFLG_IPC31   0x80000000U   // CPU1 to CM IPC31 Flag Status


//*************************************************************************************************
//
// The following are defines for the bit fields in the CMTOCPU1IPCACK register
//
//*************************************************************************************************
#define IPC_CMTOCPU1IPCACK_IPC0    0x1U          // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC0
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC1    0x2U          // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC1
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC2    0x4U          // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC2
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC3    0x8U          // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC3
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC4    0x10U         // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC4
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC5    0x20U         // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC5
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC6    0x40U         // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC6
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC7    0x80U         // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC7
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC8    0x100U        // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC8
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC9    0x200U        // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC9
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC10   0x400U        // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC10
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC11   0x800U        // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC11
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC12   0x1000U       // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC12
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC13   0x2000U       // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC13
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC14   0x4000U       // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC14
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC15   0x8000U       // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC15
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC16   0x10000U      // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC16
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC17   0x20000U      // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC17
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC18   0x40000U      // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC18
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC19   0x80000U      // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC19
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC20   0x100000U     // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC20
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC21   0x200000U     // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC21
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC22   0x400000U     // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC22
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC23   0x800000U     // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC23
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC24   0x1000000U    // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC24
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC25   0x2000000U    // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC25
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC26   0x4000000U    // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC26
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC27   0x8000000U    // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC27
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC28   0x10000000U   // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC28
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC29   0x20000000U   // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC29
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC30   0x40000000U   // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC30
                                                 // bit
#define IPC_CMTOCPU1IPCACK_IPC31   0x80000000U   // Acknowledgement from CM to CPU1TOCMIPCFLG.IPC31
                                                 // bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU1TOCMIPCSTS register
//
//*************************************************************************************************
#define IPC_CPU1TOCMIPCSTS_IPC0    0x1U          // IPC0 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC1    0x2U          // IPC1 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC2    0x4U          // IPC2 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC3    0x8U          // IPC3 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC4    0x10U         // IPC4 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC5    0x20U         // IPC5 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC6    0x40U         // IPC6 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC7    0x80U         // IPC7 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC8    0x100U        // IPC8 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC9    0x200U        // IPC9 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC10   0x400U        // IPC10 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC11   0x800U        // IPC11 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC12   0x1000U       // IPC12 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC13   0x2000U       // IPC13 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC14   0x4000U       // IPC14 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC15   0x8000U       // IPC15 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC16   0x10000U      // IPC16 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC17   0x20000U      // IPC17 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC18   0x40000U      // IPC18 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC19   0x80000U      // IPC19 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC20   0x100000U     // IPC20 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC21   0x200000U     // IPC21 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC22   0x400000U     // IPC22 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC23   0x800000U     // IPC23 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC24   0x1000000U    // IPC24 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC25   0x2000000U    // IPC25 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC26   0x4000000U    // IPC26 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC27   0x8000000U    // IPC27 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC28   0x10000000U   // IPC28 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC29   0x20000000U   // IPC29 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC30   0x40000000U   // IPC30 Request from  CPU1 to CM
#define IPC_CPU1TOCMIPCSTS_IPC31   0x80000000U   // IPC31 Request from  CPU1 to CM

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMTOCPU1IPCSET register
//
//*************************************************************************************************
#define IPC_CMTOCPU1IPCSET_IPC0    0x1U          // Set CMTOCPU1IPCFLG.IPC0 Flag
#define IPC_CMTOCPU1IPCSET_IPC1    0x2U          // Set CMTOCPU1IPCFLG.IPC1 Flag
#define IPC_CMTOCPU1IPCSET_IPC2    0x4U          // Set CMTOCPU1IPCFLG.IPC2 Flag
#define IPC_CMTOCPU1IPCSET_IPC3    0x8U          // Set CMTOCPU1IPCFLG.IPC3 Flag
#define IPC_CMTOCPU1IPCSET_IPC4    0x10U         // Set CMTOCPU1IPCFLG.IPC4 Flag
#define IPC_CMTOCPU1IPCSET_IPC5    0x20U         // Set CMTOCPU1IPCFLG.IPC5 Flag
#define IPC_CMTOCPU1IPCSET_IPC6    0x40U         // Set CMTOCPU1IPCFLG.IPC6 Flag
#define IPC_CMTOCPU1IPCSET_IPC7    0x80U         // Set CMTOCPU1IPCFLG.IPC7 Flag
#define IPC_CMTOCPU1IPCSET_IPC8    0x100U        // Set CMTOCPU1IPCFLG.IPC8 Flag
#define IPC_CMTOCPU1IPCSET_IPC9    0x200U        // Set CMTOCPU1IPCFLG.IPC9 Flag
#define IPC_CMTOCPU1IPCSET_IPC10   0x400U        // Set CMTOCPU1IPCFLG.IPC10 Flag
#define IPC_CMTOCPU1IPCSET_IPC11   0x800U        // Set CMTOCPU1IPCFLG.IPC11 Flag
#define IPC_CMTOCPU1IPCSET_IPC12   0x1000U       // Set CMTOCPU1IPCFLG.IPC12 Flag
#define IPC_CMTOCPU1IPCSET_IPC13   0x2000U       // Set CMTOCPU1IPCFLG.IPC13 Flag
#define IPC_CMTOCPU1IPCSET_IPC14   0x4000U       // Set CMTOCPU1IPCFLG.IPC14 Flag
#define IPC_CMTOCPU1IPCSET_IPC15   0x8000U       // Set CMTOCPU1IPCFLG.IPC15 Flag
#define IPC_CMTOCPU1IPCSET_IPC16   0x10000U      // Set CMTOCPU1IPCFLG.IPC16 Flag
#define IPC_CMTOCPU1IPCSET_IPC17   0x20000U      // Set CMTOCPU1IPCFLG.IPC17 Flag
#define IPC_CMTOCPU1IPCSET_IPC18   0x40000U      // Set CMTOCPU1IPCFLG.IPC18 Flag
#define IPC_CMTOCPU1IPCSET_IPC19   0x80000U      // Set CMTOCPU1IPCFLG.IPC19 Flag
#define IPC_CMTOCPU1IPCSET_IPC20   0x100000U     // Set CMTOCPU1IPCFLG.IPC20 Flag
#define IPC_CMTOCPU1IPCSET_IPC21   0x200000U     // Set CMTOCPU1IPCFLG.IPC21 Flag
#define IPC_CMTOCPU1IPCSET_IPC22   0x400000U     // Set CMTOCPU1IPCFLG.IPC22 Flag
#define IPC_CMTOCPU1IPCSET_IPC23   0x800000U     // Set CMTOCPU1IPCFLG.IPC23 Flag
#define IPC_CMTOCPU1IPCSET_IPC24   0x1000000U    // Set CMTOCPU1IPCFLG.IPC24 Flag
#define IPC_CMTOCPU1IPCSET_IPC25   0x2000000U    // Set CMTOCPU1IPCFLG.IPC25 Flag
#define IPC_CMTOCPU1IPCSET_IPC26   0x4000000U    // Set CMTOCPU1IPCFLG.IPC26 Flag
#define IPC_CMTOCPU1IPCSET_IPC27   0x8000000U    // Set CMTOCPU1IPCFLG.IPC27 Flag
#define IPC_CMTOCPU1IPCSET_IPC28   0x10000000U   // Set CMTOCPU1IPCFLG.IPC28 Flag
#define IPC_CMTOCPU1IPCSET_IPC29   0x20000000U   // Set CMTOCPU1IPCFLG.IPC29 Flag
#define IPC_CMTOCPU1IPCSET_IPC30   0x40000000U   // Set CMTOCPU1IPCFLG.IPC30 Flag
#define IPC_CMTOCPU1IPCSET_IPC31   0x80000000U   // Set CMTOCPU1IPCFLG.IPC31 Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMTOCPU1IPCCLR register
//
//*************************************************************************************************
#define IPC_CMTOCPU1IPCCLR_IPC0    0x1U          // Clear CMTOCPU1IPCFLG.IPC0 Flag
#define IPC_CMTOCPU1IPCCLR_IPC1    0x2U          // Clear CMTOCPU1IPCFLG.IPC1 Flag
#define IPC_CMTOCPU1IPCCLR_IPC2    0x4U          // Clear CMTOCPU1IPCFLG.IPC2 Flag
#define IPC_CMTOCPU1IPCCLR_IPC3    0x8U          // Clear CMTOCPU1IPCFLG.IPC3 Flag
#define IPC_CMTOCPU1IPCCLR_IPC4    0x10U         // Clear CMTOCPU1IPCFLG.IPC4 Flag
#define IPC_CMTOCPU1IPCCLR_IPC5    0x20U         // Clear CMTOCPU1IPCFLG.IPC5 Flag
#define IPC_CMTOCPU1IPCCLR_IPC6    0x40U         // Clear CMTOCPU1IPCFLG.IPC6 Flag
#define IPC_CMTOCPU1IPCCLR_IPC7    0x80U         // Clear CMTOCPU1IPCFLG.IPC7 Flag
#define IPC_CMTOCPU1IPCCLR_IPC8    0x100U        // Clear CMTOCPU1IPCFLG.IPC8 Flag
#define IPC_CMTOCPU1IPCCLR_IPC9    0x200U        // Clear CMTOCPU1IPCFLG.IPC9 Flag
#define IPC_CMTOCPU1IPCCLR_IPC10   0x400U        // Clear CMTOCPU1IPCFLG.IPC10 Flag
#define IPC_CMTOCPU1IPCCLR_IPC11   0x800U        // Clear CMTOCPU1IPCFLG.IPC11 Flag
#define IPC_CMTOCPU1IPCCLR_IPC12   0x1000U       // Clear CMTOCPU1IPCFLG.IPC12 Flag
#define IPC_CMTOCPU1IPCCLR_IPC13   0x2000U       // Clear CMTOCPU1IPCFLG.IPC13 Flag
#define IPC_CMTOCPU1IPCCLR_IPC14   0x4000U       // Clear CMTOCPU1IPCFLG.IPC14 Flag
#define IPC_CMTOCPU1IPCCLR_IPC15   0x8000U       // Clear CMTOCPU1IPCFLG.IPC15 Flag
#define IPC_CMTOCPU1IPCCLR_IPC16   0x10000U      // Clear CMTOCPU1IPCFLG.IPC16 Flag
#define IPC_CMTOCPU1IPCCLR_IPC17   0x20000U      // Clear CMTOCPU1IPCFLG.IPC17 Flag
#define IPC_CMTOCPU1IPCCLR_IPC18   0x40000U      // Clear CMTOCPU1IPCFLG.IPC18 Flag
#define IPC_CMTOCPU1IPCCLR_IPC19   0x80000U      // Clear CMTOCPU1IPCFLG.IPC19 Flag
#define IPC_CMTOCPU1IPCCLR_IPC20   0x100000U     // Clear CMTOCPU1IPCFLG.IPC20 Flag
#define IPC_CMTOCPU1IPCCLR_IPC21   0x200000U     // Clear CMTOCPU1IPCFLG.IPC21 Flag
#define IPC_CMTOCPU1IPCCLR_IPC22   0x400000U     // Clear CMTOCPU1IPCFLG.IPC22 Flag
#define IPC_CMTOCPU1IPCCLR_IPC23   0x800000U     // Clear CMTOCPU1IPCFLG.IPC23 Flag
#define IPC_CMTOCPU1IPCCLR_IPC24   0x1000000U    // Clear CMTOCPU1IPCFLG.IPC24 Flag
#define IPC_CMTOCPU1IPCCLR_IPC25   0x2000000U    // Clear CMTOCPU1IPCFLG.IPC25 Flag
#define IPC_CMTOCPU1IPCCLR_IPC26   0x4000000U    // Clear CMTOCPU1IPCFLG.IPC26 Flag
#define IPC_CMTOCPU1IPCCLR_IPC27   0x8000000U    // Clear CMTOCPU1IPCFLG.IPC27 Flag
#define IPC_CMTOCPU1IPCCLR_IPC28   0x10000000U   // Clear CMTOCPU1IPCFLG.IPC28 Flag
#define IPC_CMTOCPU1IPCCLR_IPC29   0x20000000U   // Clear CMTOCPU1IPCFLG.IPC29 Flag
#define IPC_CMTOCPU1IPCCLR_IPC30   0x40000000U   // Clear CMTOCPU1IPCFLG.IPC30 Flag
#define IPC_CMTOCPU1IPCCLR_IPC31   0x80000000U   // Clear CMTOCPU1IPCFLG.IPC31 Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMTOCPU1IPCFLG register
//
//*************************************************************************************************
#define IPC_CMTOCPU1IPCFLG_IPC0    0x1U          // CM to CPU1 IPC0 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC1    0x2U          // CM to CPU1 IPC1 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC2    0x4U          // CM to CPU1 IPC2 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC3    0x8U          // CM to CPU1 IPC3 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC4    0x10U         // CM to CPU1 IPC4 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC5    0x20U         // CM to CPU1 IPC5 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC6    0x40U         // CM to CPU1 IPC6 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC7    0x80U         // CM to CPU1 IPC7 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC8    0x100U        // CM to CPU1 IPC8 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC9    0x200U        // CM to CPU1 IPC9 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC10   0x400U        // CM to CPU1 IPC10 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC11   0x800U        // CM to CPU1 IPC11 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC12   0x1000U       // CM to CPU1 IPC12 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC13   0x2000U       // CM to CPU1 IPC13 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC14   0x4000U       // CM to CPU1 IPC14 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC15   0x8000U       // CM to CPU1 IPC15 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC16   0x10000U      // CM to CPU1 IPC16 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC17   0x20000U      // CM to CPU1 IPC17 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC18   0x40000U      // CM to CPU1 IPC18 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC19   0x80000U      // CM to CPU1 IPC19 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC20   0x100000U     // CM to CPU1 IPC20 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC21   0x200000U     // CM to CPU1 IPC21 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC22   0x400000U     // CM to CPU1 IPC22 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC23   0x800000U     // CM to CPU1 IPC23 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC24   0x1000000U    // CM to CPU1 IPC24 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC25   0x2000000U    // CM to CPU1 IPC25 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC26   0x4000000U    // CM to CPU1 IPC26 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC27   0x8000000U    // CM to CPU1 IPC27 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC28   0x10000000U   // CM to CPU1 IPC28 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC29   0x20000000U   // CM to CPU1 IPC29 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC30   0x40000000U   // CM to CPU1 IPC30 Flag Status
#define IPC_CMTOCPU1IPCFLG_IPC31   0x80000000U   // CM to CPU1 IPC31 Flag Status

//*************************************************************************************************
//
// The following are defines for the bit fields in the PUMPREQUEST register
//
//*************************************************************************************************
#define IPC_PUMPREQUEST_SEM_S   0U
#define IPC_PUMPREQUEST_SEM_M   0x3U          // Flash Pump Request Semaphore between CPU1, CPU2
                                              // and CM
#define IPC_PUMPREQUEST_KEY_S   16U
#define IPC_PUMPREQUEST_KEY_M   0xFFFF0000U   // Key Qualifier for writes to this register


//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU2TOCMIPCACK register
//
//*************************************************************************************************
#define IPC_CPU2TOCMIPCACK_IPC0    0x1U          // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC0 bit
#define IPC_CPU2TOCMIPCACK_IPC1    0x2U          // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC1 bit
#define IPC_CPU2TOCMIPCACK_IPC2    0x4U          // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC2 bit
#define IPC_CPU2TOCMIPCACK_IPC3    0x8U          // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC3 bit
#define IPC_CPU2TOCMIPCACK_IPC4    0x10U         // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC4 bit
#define IPC_CPU2TOCMIPCACK_IPC5    0x20U         // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC5 bit
#define IPC_CPU2TOCMIPCACK_IPC6    0x40U         // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC6 bit
#define IPC_CPU2TOCMIPCACK_IPC7    0x80U         // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC7 bit
#define IPC_CPU2TOCMIPCACK_IPC8    0x100U        // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC8 bit
#define IPC_CPU2TOCMIPCACK_IPC9    0x200U        // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC9 bit
#define IPC_CPU2TOCMIPCACK_IPC10   0x400U        // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC10 bit
#define IPC_CPU2TOCMIPCACK_IPC11   0x800U        // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC11 bit
#define IPC_CPU2TOCMIPCACK_IPC12   0x1000U       // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC12 bit
#define IPC_CPU2TOCMIPCACK_IPC13   0x2000U       // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC13 bit
#define IPC_CPU2TOCMIPCACK_IPC14   0x4000U       // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC14 bit
#define IPC_CPU2TOCMIPCACK_IPC15   0x8000U       // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC15 bit
#define IPC_CPU2TOCMIPCACK_IPC16   0x10000U      // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC16 bit
#define IPC_CPU2TOCMIPCACK_IPC17   0x20000U      // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC17 bit
#define IPC_CPU2TOCMIPCACK_IPC18   0x40000U      // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC18 bit
#define IPC_CPU2TOCMIPCACK_IPC19   0x80000U      // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC19 bit
#define IPC_CPU2TOCMIPCACK_IPC20   0x100000U     // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC20 bit
#define IPC_CPU2TOCMIPCACK_IPC21   0x200000U     // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC21 bit
#define IPC_CPU2TOCMIPCACK_IPC22   0x400000U     // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC22 bit
#define IPC_CPU2TOCMIPCACK_IPC23   0x800000U     // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC23 bit
#define IPC_CPU2TOCMIPCACK_IPC24   0x1000000U    // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC24 bit
#define IPC_CPU2TOCMIPCACK_IPC25   0x2000000U    // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC25 bit
#define IPC_CPU2TOCMIPCACK_IPC26   0x4000000U    // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC26 bit
#define IPC_CPU2TOCMIPCACK_IPC27   0x8000000U    // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC27 bit
#define IPC_CPU2TOCMIPCACK_IPC28   0x10000000U   // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC28 bit
#define IPC_CPU2TOCMIPCACK_IPC29   0x20000000U   // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC29 bit
#define IPC_CPU2TOCMIPCACK_IPC30   0x40000000U   // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC30 bit
#define IPC_CPU2TOCMIPCACK_IPC31   0x80000000U   // Acknowledgement from CPU2 to
                                                 // CMTOCPU2IPCFLG.IPC31 bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMTOCPU2IPCSTS register
//
//*************************************************************************************************
#define IPC_CMTOCPU2IPCSTS_IPC0    0x1U          // IPC0 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC1    0x2U          // IPC1 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC2    0x4U          // IPC2 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC3    0x8U          // IPC3 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC4    0x10U         // IPC4 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC5    0x20U         // IPC5 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC6    0x40U         // IPC6 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC7    0x80U         // IPC7 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC8    0x100U        // IPC8 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC9    0x200U        // IPC9 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC10   0x400U        // IPC10 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC11   0x800U        // IPC11 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC12   0x1000U       // IPC12 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC13   0x2000U       // IPC13 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC14   0x4000U       // IPC14 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC15   0x8000U       // IPC15 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC16   0x10000U      // IPC16 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC17   0x20000U      // IPC17 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC18   0x40000U      // IPC18 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC19   0x80000U      // IPC19 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC20   0x100000U     // IPC20 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC21   0x200000U     // IPC21 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC22   0x400000U     // IPC22 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC23   0x800000U     // IPC23 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC24   0x1000000U    // IPC24 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC25   0x2000000U    // IPC25 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC26   0x4000000U    // IPC26 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC27   0x8000000U    // IPC27 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC28   0x10000000U   // IPC28 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC29   0x20000000U   // IPC29 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC30   0x40000000U   // IPC30 Request from  CM to CPU2
#define IPC_CMTOCPU2IPCSTS_IPC31   0x80000000U   // IPC31 Request from  CM to CPU2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU2TOCMIPCSET register
//
//*************************************************************************************************
#define IPC_CPU2TOCMIPCSET_IPC0    0x1U          // Set CPU2TOCMIPCFLG.IPC0 Flag
#define IPC_CPU2TOCMIPCSET_IPC1    0x2U          // Set CPU2TOCMIPCFLG.IPC1 Flag
#define IPC_CPU2TOCMIPCSET_IPC2    0x4U          // Set CPU2TOCMIPCFLG.IPC2 Flag
#define IPC_CPU2TOCMIPCSET_IPC3    0x8U          // Set CPU2TOCMIPCFLG.IPC3 Flag
#define IPC_CPU2TOCMIPCSET_IPC4    0x10U         // Set CPU2TOCMIPCFLG.IPC4 Flag
#define IPC_CPU2TOCMIPCSET_IPC5    0x20U         // Set CPU2TOCMIPCFLG.IPC5 Flag
#define IPC_CPU2TOCMIPCSET_IPC6    0x40U         // Set CPU2TOCMIPCFLG.IPC6 Flag
#define IPC_CPU2TOCMIPCSET_IPC7    0x80U         // Set CPU2TOCMIPCFLG.IPC7 Flag
#define IPC_CPU2TOCMIPCSET_IPC8    0x100U        // Set CPU2TOCMIPCFLG.IPC8 Flag
#define IPC_CPU2TOCMIPCSET_IPC9    0x200U        // Set CPU2TOCMIPCFLG.IPC9 Flag
#define IPC_CPU2TOCMIPCSET_IPC10   0x400U        // Set CPU2TOCMIPCFLG.IPC10 Flag
#define IPC_CPU2TOCMIPCSET_IPC11   0x800U        // Set CPU2TOCMIPCFLG.IPC11 Flag
#define IPC_CPU2TOCMIPCSET_IPC12   0x1000U       // Set CPU2TOCMIPCFLG.IPC12 Flag
#define IPC_CPU2TOCMIPCSET_IPC13   0x2000U       // Set CPU2TOCMIPCFLG.IPC13 Flag
#define IPC_CPU2TOCMIPCSET_IPC14   0x4000U       // Set CPU2TOCMIPCFLG.IPC14 Flag
#define IPC_CPU2TOCMIPCSET_IPC15   0x8000U       // Set CPU2TOCMIPCFLG.IPC15 Flag
#define IPC_CPU2TOCMIPCSET_IPC16   0x10000U      // Set CPU2TOCMIPCFLG.IPC16 Flag
#define IPC_CPU2TOCMIPCSET_IPC17   0x20000U      // Set CPU2TOCMIPCFLG.IPC17 Flag
#define IPC_CPU2TOCMIPCSET_IPC18   0x40000U      // Set CPU2TOCMIPCFLG.IPC18 Flag
#define IPC_CPU2TOCMIPCSET_IPC19   0x80000U      // Set CPU2TOCMIPCFLG.IPC19 Flag
#define IPC_CPU2TOCMIPCSET_IPC20   0x100000U     // Set CPU2TOCMIPCFLG.IPC20 Flag
#define IPC_CPU2TOCMIPCSET_IPC21   0x200000U     // Set CPU2TOCMIPCFLG.IPC21 Flag
#define IPC_CPU2TOCMIPCSET_IPC22   0x400000U     // Set CPU2TOCMIPCFLG.IPC22 Flag
#define IPC_CPU2TOCMIPCSET_IPC23   0x800000U     // Set CPU2TOCMIPCFLG.IPC23 Flag
#define IPC_CPU2TOCMIPCSET_IPC24   0x1000000U    // Set CPU2TOCMIPCFLG.IPC24 Flag
#define IPC_CPU2TOCMIPCSET_IPC25   0x2000000U    // Set CPU2TOCMIPCFLG.IPC25 Flag
#define IPC_CPU2TOCMIPCSET_IPC26   0x4000000U    // Set CPU2TOCMIPCFLG.IPC26 Flag
#define IPC_CPU2TOCMIPCSET_IPC27   0x8000000U    // Set CPU2TOCMIPCFLG.IPC27 Flag
#define IPC_CPU2TOCMIPCSET_IPC28   0x10000000U   // Set CPU2TOCMIPCFLG.IPC28 Flag
#define IPC_CPU2TOCMIPCSET_IPC29   0x20000000U   // Set CPU2TOCMIPCFLG.IPC29 Flag
#define IPC_CPU2TOCMIPCSET_IPC30   0x40000000U   // Set CPU2TOCMIPCFLG.IPC30 Flag
#define IPC_CPU2TOCMIPCSET_IPC31   0x80000000U   // Set CPU2TOCMIPCFLG.IPC31 Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU2TOCMIPCCLR register
//
//*************************************************************************************************
#define IPC_CPU2TOCMIPCCLR_IPC0    0x1U          // Clear CPU2TOCMIPCFLG.IPC0 Flag
#define IPC_CPU2TOCMIPCCLR_IPC1    0x2U          // Clear CPU2TOCMIPCFLG.IPC1 Flag
#define IPC_CPU2TOCMIPCCLR_IPC2    0x4U          // Clear CPU2TOCMIPCFLG.IPC2 Flag
#define IPC_CPU2TOCMIPCCLR_IPC3    0x8U          // Clear CPU2TOCMIPCFLG.IPC3 Flag
#define IPC_CPU2TOCMIPCCLR_IPC4    0x10U         // Clear CPU2TOCMIPCFLG.IPC4 Flag
#define IPC_CPU2TOCMIPCCLR_IPC5    0x20U         // Clear CPU2TOCMIPCFLG.IPC5 Flag
#define IPC_CPU2TOCMIPCCLR_IPC6    0x40U         // Clear CPU2TOCMIPCFLG.IPC6 Flag
#define IPC_CPU2TOCMIPCCLR_IPC7    0x80U         // Clear CPU2TOCMIPCFLG.IPC7 Flag
#define IPC_CPU2TOCMIPCCLR_IPC8    0x100U        // Clear CPU2TOCMIPCFLG.IPC8 Flag
#define IPC_CPU2TOCMIPCCLR_IPC9    0x200U        // Clear CPU2TOCMIPCFLG.IPC9 Flag
#define IPC_CPU2TOCMIPCCLR_IPC10   0x400U        // Clear CPU2TOCMIPCFLG.IPC10 Flag
#define IPC_CPU2TOCMIPCCLR_IPC11   0x800U        // Clear CPU2TOCMIPCFLG.IPC11 Flag
#define IPC_CPU2TOCMIPCCLR_IPC12   0x1000U       // Clear CPU2TOCMIPCFLG.IPC12 Flag
#define IPC_CPU2TOCMIPCCLR_IPC13   0x2000U       // Clear CPU2TOCMIPCFLG.IPC13 Flag
#define IPC_CPU2TOCMIPCCLR_IPC14   0x4000U       // Clear CPU2TOCMIPCFLG.IPC14 Flag
#define IPC_CPU2TOCMIPCCLR_IPC15   0x8000U       // Clear CPU2TOCMIPCFLG.IPC15 Flag
#define IPC_CPU2TOCMIPCCLR_IPC16   0x10000U      // Clear CPU2TOCMIPCFLG.IPC16 Flag
#define IPC_CPU2TOCMIPCCLR_IPC17   0x20000U      // Clear CPU2TOCMIPCFLG.IPC17 Flag
#define IPC_CPU2TOCMIPCCLR_IPC18   0x40000U      // Clear CPU2TOCMIPCFLG.IPC18 Flag
#define IPC_CPU2TOCMIPCCLR_IPC19   0x80000U      // Clear CPU2TOCMIPCFLG.IPC19 Flag
#define IPC_CPU2TOCMIPCCLR_IPC20   0x100000U     // Clear CPU2TOCMIPCFLG.IPC20 Flag
#define IPC_CPU2TOCMIPCCLR_IPC21   0x200000U     // Clear CPU2TOCMIPCFLG.IPC21 Flag
#define IPC_CPU2TOCMIPCCLR_IPC22   0x400000U     // Clear CPU2TOCMIPCFLG.IPC22 Flag
#define IPC_CPU2TOCMIPCCLR_IPC23   0x800000U     // Clear CPU2TOCMIPCFLG.IPC23 Flag
#define IPC_CPU2TOCMIPCCLR_IPC24   0x1000000U    // Clear CPU2TOCMIPCFLG.IPC24 Flag
#define IPC_CPU2TOCMIPCCLR_IPC25   0x2000000U    // Clear CPU2TOCMIPCFLG.IPC25 Flag
#define IPC_CPU2TOCMIPCCLR_IPC26   0x4000000U    // Clear CPU2TOCMIPCFLG.IPC26 Flag
#define IPC_CPU2TOCMIPCCLR_IPC27   0x8000000U    // Clear CPU2TOCMIPCFLG.IPC27 Flag
#define IPC_CPU2TOCMIPCCLR_IPC28   0x10000000U   // Clear CPU2TOCMIPCFLG.IPC28 Flag
#define IPC_CPU2TOCMIPCCLR_IPC29   0x20000000U   // Clear CPU2TOCMIPCFLG.IPC29 Flag
#define IPC_CPU2TOCMIPCCLR_IPC30   0x40000000U   // Clear CPU2TOCMIPCFLG.IPC30 Flag
#define IPC_CPU2TOCMIPCCLR_IPC31   0x80000000U   // Clear CPU2TOCMIPCFLG.IPC31 Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU2TOCMIPCFLG register
//
//*************************************************************************************************
#define IPC_CPU2TOCMIPCFLG_IPC0    0x1U          // CPU2 to CM IPC0 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC1    0x2U          // CPU2 to CM IPC1 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC2    0x4U          // CPU2 to CM IPC2 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC3    0x8U          // CPU2 to CM IPC3 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC4    0x10U         // CPU2 to CM IPC4 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC5    0x20U         // CPU2 to CM IPC5 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC6    0x40U         // CPU2 to CM IPC6 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC7    0x80U         // CPU2 to CM IPC7 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC8    0x100U        // CPU2 to CM IPC8 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC9    0x200U        // CPU2 to CM IPC9 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC10   0x400U        // CPU2 to CM IPC10 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC11   0x800U        // CPU2 to CM IPC11 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC12   0x1000U       // CPU2 to CM IPC12 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC13   0x2000U       // CPU2 to CM IPC13 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC14   0x4000U       // CPU2 to CM IPC14 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC15   0x8000U       // CPU2 to CM IPC15 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC16   0x10000U      // CPU2 to CM IPC16 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC17   0x20000U      // CPU2 to CM IPC17 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC18   0x40000U      // CPU2 to CM IPC18 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC19   0x80000U      // CPU2 to CM IPC19 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC20   0x100000U     // CPU2 to CM IPC20 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC21   0x200000U     // CPU2 to CM IPC21 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC22   0x400000U     // CPU2 to CM IPC22 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC23   0x800000U     // CPU2 to CM IPC23 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC24   0x1000000U    // CPU2 to CM IPC24 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC25   0x2000000U    // CPU2 to CM IPC25 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC26   0x4000000U    // CPU2 to CM IPC26 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC27   0x8000000U    // CPU2 to CM IPC27 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC28   0x10000000U   // CPU2 to CM IPC28 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC29   0x20000000U   // CPU2 to CM IPC29 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC30   0x40000000U   // CPU2 to CM IPC30 Flag Status
#define IPC_CPU2TOCMIPCFLG_IPC31   0x80000000U   // CPU2 to CM IPC31 Flag Status


//*************************************************************************************************
//
// The following are defines for the bit fields in the CMTOCPU2IPCACK register
//
//*************************************************************************************************
#define IPC_CMTOCPU2IPCACK_IPC0    0x1U          // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC0
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC1    0x2U          // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC1
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC2    0x4U          // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC2
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC3    0x8U          // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC3
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC4    0x10U         // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC4
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC5    0x20U         // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC5
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC6    0x40U         // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC6
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC7    0x80U         // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC7
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC8    0x100U        // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC8
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC9    0x200U        // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC9
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC10   0x400U        // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC10
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC11   0x800U        // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC11
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC12   0x1000U       // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC12
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC13   0x2000U       // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC13
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC14   0x4000U       // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC14
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC15   0x8000U       // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC15
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC16   0x10000U      // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC16
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC17   0x20000U      // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC17
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC18   0x40000U      // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC18
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC19   0x80000U      // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC19
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC20   0x100000U     // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC20
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC21   0x200000U     // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC21
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC22   0x400000U     // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC22
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC23   0x800000U     // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC23
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC24   0x1000000U    // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC24
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC25   0x2000000U    // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC25
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC26   0x4000000U    // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC26
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC27   0x8000000U    // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC27
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC28   0x10000000U   // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC28
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC29   0x20000000U   // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC29
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC30   0x40000000U   // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC30
                                                 // bit
#define IPC_CMTOCPU2IPCACK_IPC31   0x80000000U   // Acknowledgement from CM to CPU2TOCMIPCFLG.IPC31
                                                 // bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU2TOCMIPCSTS register
//
//*************************************************************************************************
#define IPC_CPU2TOCMIPCSTS_IPC0    0x1U          // IPC0 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC1    0x2U          // IPC1 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC2    0x4U          // IPC2 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC3    0x8U          // IPC3 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC4    0x10U         // IPC4 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC5    0x20U         // IPC5 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC6    0x40U         // IPC6 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC7    0x80U         // IPC7 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC8    0x100U        // IPC8 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC9    0x200U        // IPC9 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC10   0x400U        // IPC10 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC11   0x800U        // IPC11 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC12   0x1000U       // IPC12 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC13   0x2000U       // IPC13 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC14   0x4000U       // IPC14 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC15   0x8000U       // IPC15 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC16   0x10000U      // IPC16 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC17   0x20000U      // IPC17 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC18   0x40000U      // IPC18 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC19   0x80000U      // IPC19 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC20   0x100000U     // IPC20 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC21   0x200000U     // IPC21 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC22   0x400000U     // IPC22 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC23   0x800000U     // IPC23 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC24   0x1000000U    // IPC24 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC25   0x2000000U    // IPC25 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC26   0x4000000U    // IPC26 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC27   0x8000000U    // IPC27 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC28   0x10000000U   // IPC28 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC29   0x20000000U   // IPC29 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC30   0x40000000U   // IPC30 Request from  CPU2 to CM
#define IPC_CPU2TOCMIPCSTS_IPC31   0x80000000U   // IPC31 Request from  CPU2 to CM

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMTOCPU2IPCSET register
//
//*************************************************************************************************
#define IPC_CMTOCPU2IPCSET_IPC0    0x1U          // Set CMTOCPU2IPCFLG.IPC0 Flag
#define IPC_CMTOCPU2IPCSET_IPC1    0x2U          // Set CMTOCPU2IPCFLG.IPC1 Flag
#define IPC_CMTOCPU2IPCSET_IPC2    0x4U          // Set CMTOCPU2IPCFLG.IPC2 Flag
#define IPC_CMTOCPU2IPCSET_IPC3    0x8U          // Set CMTOCPU2IPCFLG.IPC3 Flag
#define IPC_CMTOCPU2IPCSET_IPC4    0x10U         // Set CMTOCPU2IPCFLG.IPC4 Flag
#define IPC_CMTOCPU2IPCSET_IPC5    0x20U         // Set CMTOCPU2IPCFLG.IPC5 Flag
#define IPC_CMTOCPU2IPCSET_IPC6    0x40U         // Set CMTOCPU2IPCFLG.IPC6 Flag
#define IPC_CMTOCPU2IPCSET_IPC7    0x80U         // Set CMTOCPU2IPCFLG.IPC7 Flag
#define IPC_CMTOCPU2IPCSET_IPC8    0x100U        // Set CMTOCPU2IPCFLG.IPC8 Flag
#define IPC_CMTOCPU2IPCSET_IPC9    0x200U        // Set CMTOCPU2IPCFLG.IPC9 Flag
#define IPC_CMTOCPU2IPCSET_IPC10   0x400U        // Set CMTOCPU2IPCFLG.IPC10 Flag
#define IPC_CMTOCPU2IPCSET_IPC11   0x800U        // Set CMTOCPU2IPCFLG.IPC11 Flag
#define IPC_CMTOCPU2IPCSET_IPC12   0x1000U       // Set CMTOCPU2IPCFLG.IPC12 Flag
#define IPC_CMTOCPU2IPCSET_IPC13   0x2000U       // Set CMTOCPU2IPCFLG.IPC13 Flag
#define IPC_CMTOCPU2IPCSET_IPC14   0x4000U       // Set CMTOCPU2IPCFLG.IPC14 Flag
#define IPC_CMTOCPU2IPCSET_IPC15   0x8000U       // Set CMTOCPU2IPCFLG.IPC15 Flag
#define IPC_CMTOCPU2IPCSET_IPC16   0x10000U      // Set CMTOCPU2IPCFLG.IPC16 Flag
#define IPC_CMTOCPU2IPCSET_IPC17   0x20000U      // Set CMTOCPU2IPCFLG.IPC17 Flag
#define IPC_CMTOCPU2IPCSET_IPC18   0x40000U      // Set CMTOCPU2IPCFLG.IPC18 Flag
#define IPC_CMTOCPU2IPCSET_IPC19   0x80000U      // Set CMTOCPU2IPCFLG.IPC19 Flag
#define IPC_CMTOCPU2IPCSET_IPC20   0x100000U     // Set CMTOCPU2IPCFLG.IPC20 Flag
#define IPC_CMTOCPU2IPCSET_IPC21   0x200000U     // Set CMTOCPU2IPCFLG.IPC21 Flag
#define IPC_CMTOCPU2IPCSET_IPC22   0x400000U     // Set CMTOCPU2IPCFLG.IPC22 Flag
#define IPC_CMTOCPU2IPCSET_IPC23   0x800000U     // Set CMTOCPU2IPCFLG.IPC23 Flag
#define IPC_CMTOCPU2IPCSET_IPC24   0x1000000U    // Set CMTOCPU2IPCFLG.IPC24 Flag
#define IPC_CMTOCPU2IPCSET_IPC25   0x2000000U    // Set CMTOCPU2IPCFLG.IPC25 Flag
#define IPC_CMTOCPU2IPCSET_IPC26   0x4000000U    // Set CMTOCPU2IPCFLG.IPC26 Flag
#define IPC_CMTOCPU2IPCSET_IPC27   0x8000000U    // Set CMTOCPU2IPCFLG.IPC27 Flag
#define IPC_CMTOCPU2IPCSET_IPC28   0x10000000U   // Set CMTOCPU2IPCFLG.IPC28 Flag
#define IPC_CMTOCPU2IPCSET_IPC29   0x20000000U   // Set CMTOCPU2IPCFLG.IPC29 Flag
#define IPC_CMTOCPU2IPCSET_IPC30   0x40000000U   // Set CMTOCPU2IPCFLG.IPC30 Flag
#define IPC_CMTOCPU2IPCSET_IPC31   0x80000000U   // Set CMTOCPU2IPCFLG.IPC31 Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMTOCPU2IPCCLR register
//
//*************************************************************************************************
#define IPC_CMTOCPU2IPCCLR_IPC0    0x1U          // Clear CMTOCPU2IPCFLG.IPC0 Flag
#define IPC_CMTOCPU2IPCCLR_IPC1    0x2U          // Clear CMTOCPU2IPCFLG.IPC1 Flag
#define IPC_CMTOCPU2IPCCLR_IPC2    0x4U          // Clear CMTOCPU2IPCFLG.IPC2 Flag
#define IPC_CMTOCPU2IPCCLR_IPC3    0x8U          // Clear CMTOCPU2IPCFLG.IPC3 Flag
#define IPC_CMTOCPU2IPCCLR_IPC4    0x10U         // Clear CMTOCPU2IPCFLG.IPC4 Flag
#define IPC_CMTOCPU2IPCCLR_IPC5    0x20U         // Clear CMTOCPU2IPCFLG.IPC5 Flag
#define IPC_CMTOCPU2IPCCLR_IPC6    0x40U         // Clear CMTOCPU2IPCFLG.IPC6 Flag
#define IPC_CMTOCPU2IPCCLR_IPC7    0x80U         // Clear CMTOCPU2IPCFLG.IPC7 Flag
#define IPC_CMTOCPU2IPCCLR_IPC8    0x100U        // Clear CMTOCPU2IPCFLG.IPC8 Flag
#define IPC_CMTOCPU2IPCCLR_IPC9    0x200U        // Clear CMTOCPU2IPCFLG.IPC9 Flag
#define IPC_CMTOCPU2IPCCLR_IPC10   0x400U        // Clear CMTOCPU2IPCFLG.IPC10 Flag
#define IPC_CMTOCPU2IPCCLR_IPC11   0x800U        // Clear CMTOCPU2IPCFLG.IPC11 Flag
#define IPC_CMTOCPU2IPCCLR_IPC12   0x1000U       // Clear CMTOCPU2IPCFLG.IPC12 Flag
#define IPC_CMTOCPU2IPCCLR_IPC13   0x2000U       // Clear CMTOCPU2IPCFLG.IPC13 Flag
#define IPC_CMTOCPU2IPCCLR_IPC14   0x4000U       // Clear CMTOCPU2IPCFLG.IPC14 Flag
#define IPC_CMTOCPU2IPCCLR_IPC15   0x8000U       // Clear CMTOCPU2IPCFLG.IPC15 Flag
#define IPC_CMTOCPU2IPCCLR_IPC16   0x10000U      // Clear CMTOCPU2IPCFLG.IPC16 Flag
#define IPC_CMTOCPU2IPCCLR_IPC17   0x20000U      // Clear CMTOCPU2IPCFLG.IPC17 Flag
#define IPC_CMTOCPU2IPCCLR_IPC18   0x40000U      // Clear CMTOCPU2IPCFLG.IPC18 Flag
#define IPC_CMTOCPU2IPCCLR_IPC19   0x80000U      // Clear CMTOCPU2IPCFLG.IPC19 Flag
#define IPC_CMTOCPU2IPCCLR_IPC20   0x100000U     // Clear CMTOCPU2IPCFLG.IPC20 Flag
#define IPC_CMTOCPU2IPCCLR_IPC21   0x200000U     // Clear CMTOCPU2IPCFLG.IPC21 Flag
#define IPC_CMTOCPU2IPCCLR_IPC22   0x400000U     // Clear CMTOCPU2IPCFLG.IPC22 Flag
#define IPC_CMTOCPU2IPCCLR_IPC23   0x800000U     // Clear CMTOCPU2IPCFLG.IPC23 Flag
#define IPC_CMTOCPU2IPCCLR_IPC24   0x1000000U    // Clear CMTOCPU2IPCFLG.IPC24 Flag
#define IPC_CMTOCPU2IPCCLR_IPC25   0x2000000U    // Clear CMTOCPU2IPCFLG.IPC25 Flag
#define IPC_CMTOCPU2IPCCLR_IPC26   0x4000000U    // Clear CMTOCPU2IPCFLG.IPC26 Flag
#define IPC_CMTOCPU2IPCCLR_IPC27   0x8000000U    // Clear CMTOCPU2IPCFLG.IPC27 Flag
#define IPC_CMTOCPU2IPCCLR_IPC28   0x10000000U   // Clear CMTOCPU2IPCFLG.IPC28 Flag
#define IPC_CMTOCPU2IPCCLR_IPC29   0x20000000U   // Clear CMTOCPU2IPCFLG.IPC29 Flag
#define IPC_CMTOCPU2IPCCLR_IPC30   0x40000000U   // Clear CMTOCPU2IPCFLG.IPC30 Flag
#define IPC_CMTOCPU2IPCCLR_IPC31   0x80000000U   // Clear CMTOCPU2IPCFLG.IPC31 Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the CMTOCPU2IPCFLG register
//
//*************************************************************************************************
#define IPC_CMTOCPU2IPCFLG_IPC0    0x1U          // CM to CPU2 IPC0 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC1    0x2U          // CM to CPU2 IPC1 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC2    0x4U          // CM to CPU2 IPC2 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC3    0x8U          // CM to CPU2 IPC3 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC4    0x10U         // CM to CPU2 IPC4 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC5    0x20U         // CM to CPU2 IPC5 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC6    0x40U         // CM to CPU2 IPC6 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC7    0x80U         // CM to CPU2 IPC7 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC8    0x100U        // CM to CPU2 IPC8 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC9    0x200U        // CM to CPU2 IPC9 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC10   0x400U        // CM to CPU2 IPC10 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC11   0x800U        // CM to CPU2 IPC11 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC12   0x1000U       // CM to CPU2 IPC12 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC13   0x2000U       // CM to CPU2 IPC13 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC14   0x4000U       // CM to CPU2 IPC14 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC15   0x8000U       // CM to CPU2 IPC15 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC16   0x10000U      // CM to CPU2 IPC16 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC17   0x20000U      // CM to CPU2 IPC17 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC18   0x40000U      // CM to CPU2 IPC18 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC19   0x80000U      // CM to CPU2 IPC19 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC20   0x100000U     // CM to CPU2 IPC20 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC21   0x200000U     // CM to CPU2 IPC21 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC22   0x400000U     // CM to CPU2 IPC22 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC23   0x800000U     // CM to CPU2 IPC23 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC24   0x1000000U    // CM to CPU2 IPC24 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC25   0x2000000U    // CM to CPU2 IPC25 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC26   0x4000000U    // CM to CPU2 IPC26 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC27   0x8000000U    // CM to CPU2 IPC27 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC28   0x10000000U   // CM to CPU2 IPC28 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC29   0x20000000U   // CM to CPU2 IPC29 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC30   0x40000000U   // CM to CPU2 IPC30 Flag Status
#define IPC_CMTOCPU2IPCFLG_IPC31   0x80000000U   // CM to CPU2 IPC31 Flag Status


//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU1TOCPU2IPCACK register
//
//*************************************************************************************************
#define IPC_CPU1TOCPU2IPCACK_IPC0    0x1U          // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC0 bit
#define IPC_CPU1TOCPU2IPCACK_IPC1    0x2U          // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC1 bit
#define IPC_CPU1TOCPU2IPCACK_IPC2    0x4U          // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC2 bit
#define IPC_CPU1TOCPU2IPCACK_IPC3    0x8U          // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC3 bit
#define IPC_CPU1TOCPU2IPCACK_IPC4    0x10U         // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC4 bit
#define IPC_CPU1TOCPU2IPCACK_IPC5    0x20U         // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC5 bit
#define IPC_CPU1TOCPU2IPCACK_IPC6    0x40U         // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC6 bit
#define IPC_CPU1TOCPU2IPCACK_IPC7    0x80U         // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC7 bit
#define IPC_CPU1TOCPU2IPCACK_IPC8    0x100U        // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC8 bit
#define IPC_CPU1TOCPU2IPCACK_IPC9    0x200U        // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC9 bit
#define IPC_CPU1TOCPU2IPCACK_IPC10   0x400U        // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC10 bit
#define IPC_CPU1TOCPU2IPCACK_IPC11   0x800U        // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC11 bit
#define IPC_CPU1TOCPU2IPCACK_IPC12   0x1000U       // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC12 bit
#define IPC_CPU1TOCPU2IPCACK_IPC13   0x2000U       // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC13 bit
#define IPC_CPU1TOCPU2IPCACK_IPC14   0x4000U       // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC14 bit
#define IPC_CPU1TOCPU2IPCACK_IPC15   0x8000U       // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC15 bit
#define IPC_CPU1TOCPU2IPCACK_IPC16   0x10000U      // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC16 bit
#define IPC_CPU1TOCPU2IPCACK_IPC17   0x20000U      // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC17 bit
#define IPC_CPU1TOCPU2IPCACK_IPC18   0x40000U      // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC18 bit
#define IPC_CPU1TOCPU2IPCACK_IPC19   0x80000U      // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC19 bit
#define IPC_CPU1TOCPU2IPCACK_IPC20   0x100000U     // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC20 bit
#define IPC_CPU1TOCPU2IPCACK_IPC21   0x200000U     // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC21 bit
#define IPC_CPU1TOCPU2IPCACK_IPC22   0x400000U     // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC22 bit
#define IPC_CPU1TOCPU2IPCACK_IPC23   0x800000U     // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC23 bit
#define IPC_CPU1TOCPU2IPCACK_IPC24   0x1000000U    // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC24 bit
#define IPC_CPU1TOCPU2IPCACK_IPC25   0x2000000U    // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC25 bit
#define IPC_CPU1TOCPU2IPCACK_IPC26   0x4000000U    // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC26 bit
#define IPC_CPU1TOCPU2IPCACK_IPC27   0x8000000U    // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC27 bit
#define IPC_CPU1TOCPU2IPCACK_IPC28   0x10000000U   // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC28 bit
#define IPC_CPU1TOCPU2IPCACK_IPC29   0x20000000U   // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC29 bit
#define IPC_CPU1TOCPU2IPCACK_IPC30   0x40000000U   // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC30 bit
#define IPC_CPU1TOCPU2IPCACK_IPC31   0x80000000U   // Acknowledgement from CPU1 to
                                                   // CPU2TOCPU1IPCFLG.IPC31 bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU2TOCPU1IPCSTS register
//
//*************************************************************************************************
#define IPC_CPU2TOCPU1IPCSTS_IPC0    0x1U          // IPC0 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC1    0x2U          // IPC1 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC2    0x4U          // IPC2 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC3    0x8U          // IPC3 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC4    0x10U         // IPC4 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC5    0x20U         // IPC5 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC6    0x40U         // IPC6 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC7    0x80U         // IPC7 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC8    0x100U        // IPC8 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC9    0x200U        // IPC9 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC10   0x400U        // IPC10 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC11   0x800U        // IPC11 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC12   0x1000U       // IPC12 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC13   0x2000U       // IPC13 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC14   0x4000U       // IPC14 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC15   0x8000U       // IPC15 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC16   0x10000U      // IPC16 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC17   0x20000U      // IPC17 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC18   0x40000U      // IPC18 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC19   0x80000U      // IPC19 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC20   0x100000U     // IPC20 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC21   0x200000U     // IPC21 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC22   0x400000U     // IPC22 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC23   0x800000U     // IPC23 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC24   0x1000000U    // IPC24 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC25   0x2000000U    // IPC25 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC26   0x4000000U    // IPC26 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC27   0x8000000U    // IPC27 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC28   0x10000000U   // IPC28 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC29   0x20000000U   // IPC29 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC30   0x40000000U   // IPC30 Request from  CPU2 to CPU1
#define IPC_CPU2TOCPU1IPCSTS_IPC31   0x80000000U   // IPC31 Request from  CPU2 to CPU1

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU1TOCPU2IPCSET register
//
//*************************************************************************************************
#define IPC_CPU1TOCPU2IPCSET_IPC0    0x1U          // Set CPU1TOCPU2IPCFLG.IPC0 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC1    0x2U          // Set CPU1TOCPU2IPCFLG.IPC1 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC2    0x4U          // Set CPU1TOCPU2IPCFLG.IPC2 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC3    0x8U          // Set CPU1TOCPU2IPCFLG.IPC3 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC4    0x10U         // Set CPU1TOCPU2IPCFLG.IPC4 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC5    0x20U         // Set CPU1TOCPU2IPCFLG.IPC5 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC6    0x40U         // Set CPU1TOCPU2IPCFLG.IPC6 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC7    0x80U         // Set CPU1TOCPU2IPCFLG.IPC7 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC8    0x100U        // Set CPU1TOCPU2IPCFLG.IPC8 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC9    0x200U        // Set CPU1TOCPU2IPCFLG.IPC9 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC10   0x400U        // Set CPU1TOCPU2IPCFLG.IPC10 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC11   0x800U        // Set CPU1TOCPU2IPCFLG.IPC11 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC12   0x1000U       // Set CPU1TOCPU2IPCFLG.IPC12 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC13   0x2000U       // Set CPU1TOCPU2IPCFLG.IPC13 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC14   0x4000U       // Set CPU1TOCPU2IPCFLG.IPC14 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC15   0x8000U       // Set CPU1TOCPU2IPCFLG.IPC15 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC16   0x10000U      // Set CPU1TOCPU2IPCFLG.IPC16 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC17   0x20000U      // Set CPU1TOCPU2IPCFLG.IPC17 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC18   0x40000U      // Set CPU1TOCPU2IPCFLG.IPC18 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC19   0x80000U      // Set CPU1TOCPU2IPCFLG.IPC19 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC20   0x100000U     // Set CPU1TOCPU2IPCFLG.IPC20 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC21   0x200000U     // Set CPU1TOCPU2IPCFLG.IPC21 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC22   0x400000U     // Set CPU1TOCPU2IPCFLG.IPC22 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC23   0x800000U     // Set CPU1TOCPU2IPCFLG.IPC23 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC24   0x1000000U    // Set CPU1TOCPU2IPCFLG.IPC24 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC25   0x2000000U    // Set CPU1TOCPU2IPCFLG.IPC25 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC26   0x4000000U    // Set CPU1TOCPU2IPCFLG.IPC26 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC27   0x8000000U    // Set CPU1TOCPU2IPCFLG.IPC27 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC28   0x10000000U   // Set CPU1TOCPU2IPCFLG.IPC28 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC29   0x20000000U   // Set CPU1TOCPU2IPCFLG.IPC29 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC30   0x40000000U   // Set CPU1TOCPU2IPCFLG.IPC30 Flag
#define IPC_CPU1TOCPU2IPCSET_IPC31   0x80000000U   // Set CPU1TOCPU2IPCFLG.IPC31 Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU1TOCPU2IPCCLR register
//
//*************************************************************************************************
#define IPC_CPU1TOCPU2IPCCLR_IPC0    0x1U          // Clear CPU1TOCPU2IPCFLG.IPC0 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC1    0x2U          // Clear CPU1TOCPU2IPCFLG.IPC1 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC2    0x4U          // Clear CPU1TOCPU2IPCFLG.IPC2 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC3    0x8U          // Clear CPU1TOCPU2IPCFLG.IPC3 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC4    0x10U         // Clear CPU1TOCPU2IPCFLG.IPC4 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC5    0x20U         // Clear CPU1TOCPU2IPCFLG.IPC5 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC6    0x40U         // Clear CPU1TOCPU2IPCFLG.IPC6 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC7    0x80U         // Clear CPU1TOCPU2IPCFLG.IPC7 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC8    0x100U        // Clear CPU1TOCPU2IPCFLG.IPC8 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC9    0x200U        // Clear CPU1TOCPU2IPCFLG.IPC9 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC10   0x400U        // Clear CPU1TOCPU2IPCFLG.IPC10 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC11   0x800U        // Clear CPU1TOCPU2IPCFLG.IPC11 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC12   0x1000U       // Clear CPU1TOCPU2IPCFLG.IPC12 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC13   0x2000U       // Clear CPU1TOCPU2IPCFLG.IPC13 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC14   0x4000U       // Clear CPU1TOCPU2IPCFLG.IPC14 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC15   0x8000U       // Clear CPU1TOCPU2IPCFLG.IPC15 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC16   0x10000U      // Clear CPU1TOCPU2IPCFLG.IPC16 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC17   0x20000U      // Clear CPU1TOCPU2IPCFLG.IPC17 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC18   0x40000U      // Clear CPU1TOCPU2IPCFLG.IPC18 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC19   0x80000U      // Clear CPU1TOCPU2IPCFLG.IPC19 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC20   0x100000U     // Clear CPU1TOCPU2IPCFLG.IPC20 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC21   0x200000U     // Clear CPU1TOCPU2IPCFLG.IPC21 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC22   0x400000U     // Clear CPU1TOCPU2IPCFLG.IPC22 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC23   0x800000U     // Clear CPU1TOCPU2IPCFLG.IPC23 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC24   0x1000000U    // Clear CPU1TOCPU2IPCFLG.IPC24 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC25   0x2000000U    // Clear CPU1TOCPU2IPCFLG.IPC25 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC26   0x4000000U    // Clear CPU1TOCPU2IPCFLG.IPC26 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC27   0x8000000U    // Clear CPU1TOCPU2IPCFLG.IPC27 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC28   0x10000000U   // Clear CPU1TOCPU2IPCFLG.IPC28 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC29   0x20000000U   // Clear CPU1TOCPU2IPCFLG.IPC29 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC30   0x40000000U   // Clear CPU1TOCPU2IPCFLG.IPC30 Flag
#define IPC_CPU1TOCPU2IPCCLR_IPC31   0x80000000U   // Clear CPU1TOCPU2IPCFLG.IPC31 Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU1TOCPU2IPCFLG register
//
//*************************************************************************************************
#define IPC_CPU1TOCPU2IPCFLG_IPC0    0x1U          // CPU1 to CPU2 IPC0 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC1    0x2U          // CPU1 to CPU2 IPC1 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC2    0x4U          // CPU1 to CPU2 IPC2 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC3    0x8U          // CPU1 to CPU2 IPC3 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC4    0x10U         // CPU1 to CPU2 IPC4 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC5    0x20U         // CPU1 to CPU2 IPC5 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC6    0x40U         // CPU1 to CPU2 IPC6 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC7    0x80U         // CPU1 to CPU2 IPC7 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC8    0x100U        // CPU1 to CPU2 IPC8 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC9    0x200U        // CPU1 to CPU2 IPC9 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC10   0x400U        // CPU1 to CPU2 IPC10 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC11   0x800U        // CPU1 to CPU2 IPC11 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC12   0x1000U       // CPU1 to CPU2 IPC12 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC13   0x2000U       // CPU1 to CPU2 IPC13 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC14   0x4000U       // CPU1 to CPU2 IPC14 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC15   0x8000U       // CPU1 to CPU2 IPC15 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC16   0x10000U      // CPU1 to CPU2 IPC16 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC17   0x20000U      // CPU1 to CPU2 IPC17 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC18   0x40000U      // CPU1 to CPU2 IPC18 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC19   0x80000U      // CPU1 to CPU2 IPC19 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC20   0x100000U     // CPU1 to CPU2 IPC20 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC21   0x200000U     // CPU1 to CPU2 IPC21 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC22   0x400000U     // CPU1 to CPU2 IPC22 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC23   0x800000U     // CPU1 to CPU2 IPC23 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC24   0x1000000U    // CPU1 to CPU2 IPC24 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC25   0x2000000U    // CPU1 to CPU2 IPC25 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC26   0x4000000U    // CPU1 to CPU2 IPC26 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC27   0x8000000U    // CPU1 to CPU2 IPC27 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC28   0x10000000U   // CPU1 to CPU2 IPC28 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC29   0x20000000U   // CPU1 to CPU2 IPC29 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC30   0x40000000U   // CPU1 to CPU2 IPC30 Flag Status
#define IPC_CPU1TOCPU2IPCFLG_IPC31   0x80000000U   // CPU1 to CPU2 IPC31 Flag Status

//*************************************************************************************************
//
// The following are defines for the bit fields in the PUMPREQUEST register
//
//*************************************************************************************************
#define IPC_PUMPREQUEST_SEM_S   0U
#define IPC_PUMPREQUEST_SEM_M   0x3U          // Flash Pump Request Semaphore between CPU1, CPU2
                                              // and CM
#define IPC_PUMPREQUEST_KEY_S   16U
#define IPC_PUMPREQUEST_KEY_M   0xFFFF0000U   // Key Qualifier for writes to this register


//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU2TOCPU1IPCACK register
//
//*************************************************************************************************
#define IPC_CPU2TOCPU1IPCACK_IPC0    0x1U          // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC0 bit
#define IPC_CPU2TOCPU1IPCACK_IPC1    0x2U          // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC1 bit
#define IPC_CPU2TOCPU1IPCACK_IPC2    0x4U          // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC2 bit
#define IPC_CPU2TOCPU1IPCACK_IPC3    0x8U          // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC3 bit
#define IPC_CPU2TOCPU1IPCACK_IPC4    0x10U         // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC4 bit
#define IPC_CPU2TOCPU1IPCACK_IPC5    0x20U         // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC5 bit
#define IPC_CPU2TOCPU1IPCACK_IPC6    0x40U         // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC6 bit
#define IPC_CPU2TOCPU1IPCACK_IPC7    0x80U         // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC7 bit
#define IPC_CPU2TOCPU1IPCACK_IPC8    0x100U        // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC8 bit
#define IPC_CPU2TOCPU1IPCACK_IPC9    0x200U        // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC9 bit
#define IPC_CPU2TOCPU1IPCACK_IPC10   0x400U        // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC10 bit
#define IPC_CPU2TOCPU1IPCACK_IPC11   0x800U        // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC11 bit
#define IPC_CPU2TOCPU1IPCACK_IPC12   0x1000U       // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC12 bit
#define IPC_CPU2TOCPU1IPCACK_IPC13   0x2000U       // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC13 bit
#define IPC_CPU2TOCPU1IPCACK_IPC14   0x4000U       // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC14 bit
#define IPC_CPU2TOCPU1IPCACK_IPC15   0x8000U       // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC15 bit
#define IPC_CPU2TOCPU1IPCACK_IPC16   0x10000U      // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC16 bit
#define IPC_CPU2TOCPU1IPCACK_IPC17   0x20000U      // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC17 bit
#define IPC_CPU2TOCPU1IPCACK_IPC18   0x40000U      // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC18 bit
#define IPC_CPU2TOCPU1IPCACK_IPC19   0x80000U      // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC19 bit
#define IPC_CPU2TOCPU1IPCACK_IPC20   0x100000U     // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC20 bit
#define IPC_CPU2TOCPU1IPCACK_IPC21   0x200000U     // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC21 bit
#define IPC_CPU2TOCPU1IPCACK_IPC22   0x400000U     // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC22 bit
#define IPC_CPU2TOCPU1IPCACK_IPC23   0x800000U     // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC23 bit
#define IPC_CPU2TOCPU1IPCACK_IPC24   0x1000000U    // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC24 bit
#define IPC_CPU2TOCPU1IPCACK_IPC25   0x2000000U    // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC25 bit
#define IPC_CPU2TOCPU1IPCACK_IPC26   0x4000000U    // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC26 bit
#define IPC_CPU2TOCPU1IPCACK_IPC27   0x8000000U    // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC27 bit
#define IPC_CPU2TOCPU1IPCACK_IPC28   0x10000000U   // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC28 bit
#define IPC_CPU2TOCPU1IPCACK_IPC29   0x20000000U   // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC29 bit
#define IPC_CPU2TOCPU1IPCACK_IPC30   0x40000000U   // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC30 bit
#define IPC_CPU2TOCPU1IPCACK_IPC31   0x80000000U   // Acknowledgement from CPU2 to
                                                   // CPU1TOCPU2IPCFLG.IPC31 bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU1TOCPU2IPCSTS register
//
//*************************************************************************************************
#define IPC_CPU1TOCPU2IPCSTS_IPC0    0x1U          // IPC0 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC1    0x2U          // IPC1 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC2    0x4U          // IPC2 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC3    0x8U          // IPC3 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC4    0x10U         // IPC4 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC5    0x20U         // IPC5 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC6    0x40U         // IPC6 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC7    0x80U         // IPC7 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC8    0x100U        // IPC8 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC9    0x200U        // IPC9 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC10   0x400U        // IPC10 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC11   0x800U        // IPC11 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC12   0x1000U       // IPC12 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC13   0x2000U       // IPC13 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC14   0x4000U       // IPC14 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC15   0x8000U       // IPC15 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC16   0x10000U      // IPC16 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC17   0x20000U      // IPC17 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC18   0x40000U      // IPC18 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC19   0x80000U      // IPC19 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC20   0x100000U     // IPC20 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC21   0x200000U     // IPC21 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC22   0x400000U     // IPC22 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC23   0x800000U     // IPC23 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC24   0x1000000U    // IPC24 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC25   0x2000000U    // IPC25 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC26   0x4000000U    // IPC26 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC27   0x8000000U    // IPC27 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC28   0x10000000U   // IPC28 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC29   0x20000000U   // IPC29 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC30   0x40000000U   // IPC30 Request from  CPU1 to CPU2
#define IPC_CPU1TOCPU2IPCSTS_IPC31   0x80000000U   // IPC31 Request from  CPU1 to CPU2

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU2TOCPU1IPCSET register
//
//*************************************************************************************************
#define IPC_CPU2TOCPU1IPCSET_IPC0    0x1U          // Set CPU2TOCPU1IPCFLG.IPC0 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC1    0x2U          // Set CPU2TOCPU1IPCFLG.IPC1 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC2    0x4U          // Set CPU2TOCPU1IPCFLG.IPC2 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC3    0x8U          // Set CPU2TOCPU1IPCFLG.IPC3 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC4    0x10U         // Set CPU2TOCPU1IPCFLG.IPC4 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC5    0x20U         // Set CPU2TOCPU1IPCFLG.IPC5 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC6    0x40U         // Set CPU2TOCPU1IPCFLG.IPC6 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC7    0x80U         // Set CPU2TOCPU1IPCFLG.IPC7 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC8    0x100U        // Set CPU2TOCPU1IPCFLG.IPC8 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC9    0x200U        // Set CPU2TOCPU1IPCFLG.IPC9 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC10   0x400U        // Set CPU2TOCPU1IPCFLG.IPC10 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC11   0x800U        // Set CPU2TOCPU1IPCFLG.IPC11 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC12   0x1000U       // Set CPU2TOCPU1IPCFLG.IPC12 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC13   0x2000U       // Set CPU2TOCPU1IPCFLG.IPC13 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC14   0x4000U       // Set CPU2TOCPU1IPCFLG.IPC14 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC15   0x8000U       // Set CPU2TOCPU1IPCFLG.IPC15 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC16   0x10000U      // Set CPU2TOCPU1IPCFLG.IPC16 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC17   0x20000U      // Set CPU2TOCPU1IPCFLG.IPC17 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC18   0x40000U      // Set CPU2TOCPU1IPCFLG.IPC18 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC19   0x80000U      // Set CPU2TOCPU1IPCFLG.IPC19 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC20   0x100000U     // Set CPU2TOCPU1IPCFLG.IPC20 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC21   0x200000U     // Set CPU2TOCPU1IPCFLG.IPC21 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC22   0x400000U     // Set CPU2TOCPU1IPCFLG.IPC22 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC23   0x800000U     // Set CPU2TOCPU1IPCFLG.IPC23 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC24   0x1000000U    // Set CPU2TOCPU1IPCFLG.IPC24 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC25   0x2000000U    // Set CPU2TOCPU1IPCFLG.IPC25 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC26   0x4000000U    // Set CPU2TOCPU1IPCFLG.IPC26 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC27   0x8000000U    // Set CPU2TOCPU1IPCFLG.IPC27 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC28   0x10000000U   // Set CPU2TOCPU1IPCFLG.IPC28 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC29   0x20000000U   // Set CPU2TOCPU1IPCFLG.IPC29 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC30   0x40000000U   // Set CPU2TOCPU1IPCFLG.IPC30 Flag
#define IPC_CPU2TOCPU1IPCSET_IPC31   0x80000000U   // Set CPU2TOCPU1IPCFLG.IPC31 Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU2TOCPU1IPCCLR register
//
//*************************************************************************************************
#define IPC_CPU2TOCPU1IPCCLR_IPC0    0x1U          // Clear CPU2TOCPU1IPCFLG.IPC0 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC1    0x2U          // Clear CPU2TOCPU1IPCFLG.IPC1 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC2    0x4U          // Clear CPU2TOCPU1IPCFLG.IPC2 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC3    0x8U          // Clear CPU2TOCPU1IPCFLG.IPC3 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC4    0x10U         // Clear CPU2TOCPU1IPCFLG.IPC4 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC5    0x20U         // Clear CPU2TOCPU1IPCFLG.IPC5 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC6    0x40U         // Clear CPU2TOCPU1IPCFLG.IPC6 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC7    0x80U         // Clear CPU2TOCPU1IPCFLG.IPC7 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC8    0x100U        // Clear CPU2TOCPU1IPCFLG.IPC8 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC9    0x200U        // Clear CPU2TOCPU1IPCFLG.IPC9 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC10   0x400U        // Clear CPU2TOCPU1IPCFLG.IPC10 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC11   0x800U        // Clear CPU2TOCPU1IPCFLG.IPC11 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC12   0x1000U       // Clear CPU2TOCPU1IPCFLG.IPC12 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC13   0x2000U       // Clear CPU2TOCPU1IPCFLG.IPC13 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC14   0x4000U       // Clear CPU2TOCPU1IPCFLG.IPC14 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC15   0x8000U       // Clear CPU2TOCPU1IPCFLG.IPC15 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC16   0x10000U      // Clear CPU2TOCPU1IPCFLG.IPC16 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC17   0x20000U      // Clear CPU2TOCPU1IPCFLG.IPC17 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC18   0x40000U      // Clear CPU2TOCPU1IPCFLG.IPC18 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC19   0x80000U      // Clear CPU2TOCPU1IPCFLG.IPC19 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC20   0x100000U     // Clear CPU2TOCPU1IPCFLG.IPC20 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC21   0x200000U     // Clear CPU2TOCPU1IPCFLG.IPC21 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC22   0x400000U     // Clear CPU2TOCPU1IPCFLG.IPC22 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC23   0x800000U     // Clear CPU2TOCPU1IPCFLG.IPC23 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC24   0x1000000U    // Clear CPU2TOCPU1IPCFLG.IPC24 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC25   0x2000000U    // Clear CPU2TOCPU1IPCFLG.IPC25 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC26   0x4000000U    // Clear CPU2TOCPU1IPCFLG.IPC26 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC27   0x8000000U    // Clear CPU2TOCPU1IPCFLG.IPC27 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC28   0x10000000U   // Clear CPU2TOCPU1IPCFLG.IPC28 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC29   0x20000000U   // Clear CPU2TOCPU1IPCFLG.IPC29 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC30   0x40000000U   // Clear CPU2TOCPU1IPCFLG.IPC30 Flag
#define IPC_CPU2TOCPU1IPCCLR_IPC31   0x80000000U   // Clear CPU2TOCPU1IPCFLG.IPC31 Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the CPU2TOCPU1IPCFLG register
//
//*************************************************************************************************
#define IPC_CPU2TOCPU1IPCFLG_IPC0    0x1U          // CPU2 to CPU1 IPC0 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC1    0x2U          // CPU2 to CPU1 IPC1 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC2    0x4U          // CPU2 to CPU1 IPC2 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC3    0x8U          // CPU2 to CPU1 IPC3 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC4    0x10U         // CPU2 to CPU1 IPC4 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC5    0x20U         // CPU2 to CPU1 IPC5 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC6    0x40U         // CPU2 to CPU1 IPC6 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC7    0x80U         // CPU2 to CPU1 IPC7 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC8    0x100U        // CPU2 to CPU1 IPC8 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC9    0x200U        // CPU2 to CPU1 IPC9 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC10   0x400U        // CPU2 to CPU1 IPC10 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC11   0x800U        // CPU2 to CPU1 IPC11 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC12   0x1000U       // CPU2 to CPU1 IPC12 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC13   0x2000U       // CPU2 to CPU1 IPC13 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC14   0x4000U       // CPU2 to CPU1 IPC14 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC15   0x8000U       // CPU2 to CPU1 IPC15 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC16   0x10000U      // CPU2 to CPU1 IPC16 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC17   0x20000U      // CPU2 to CPU1 IPC17 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC18   0x40000U      // CPU2 to CPU1 IPC18 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC19   0x80000U      // CPU2 to CPU1 IPC19 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC20   0x100000U     // CPU2 to CPU1 IPC20 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC21   0x200000U     // CPU2 to CPU1 IPC21 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC22   0x400000U     // CPU2 to CPU1 IPC22 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC23   0x800000U     // CPU2 to CPU1 IPC23 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC24   0x1000000U    // CPU2 to CPU1 IPC24 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC25   0x2000000U    // CPU2 to CPU1 IPC25 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC26   0x4000000U    // CPU2 to CPU1 IPC26 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC27   0x8000000U    // CPU2 to CPU1 IPC27 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC28   0x10000000U   // CPU2 to CPU1 IPC28 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC29   0x20000000U   // CPU2 to CPU1 IPC29 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC30   0x40000000U   // CPU2 to CPU1 IPC30 Flag Status
#define IPC_CPU2TOCPU1IPCFLG_IPC31   0x80000000U   // CPU2 to CPU1 IPC31 Flag Status

//*************************************************************************************************
//
// The following are defines for the bit fields in the PUMPREQUEST register
//
//*************************************************************************************************
#define IPC_PUMPREQUEST_SEM_S   0U
#define IPC_PUMPREQUEST_SEM_M   0x3U          // Flash Pump Request Semaphore between CPU1, CPU2
                                              // and CM
#define IPC_PUMPREQUEST_KEY_S   16U
#define IPC_PUMPREQUEST_KEY_M   0xFFFF0000U   // Key Qualifier for writes to this register



#endif
