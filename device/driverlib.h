//#############################################################################
//
// FILE:   driverlib.h
//
// TITLE:  C28x Driverlib Header File
//
//#############################################################################
//
//
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
//#############################################################################
#ifndef DRIVERLIB_H
#define DRIVERLIB_H

#include "inc/hw_memmap.h"

#include "adc.h"
#include "asysctl.h"
#include "bgcrc.h"
#include "can.h"
#include "cla.h"
#include "clb.h"
#include "cmpss.h"
#include "cpu.h"
#include "cputimer.h"
#include "dac.h"
#include "dcc.h"
#include "dcsm.h"
#include "debug.h"
#include "dma.h"
#include "ecap.h"
#include "emif.h"
#include "epwm.h"
#include "eqep.h"
#include "erad.h"
#include "escss.h"
#include "flash.h"
#include "fsi.h"
#include "gpio.h"
#include "hrcap.h"
#include "hrpwm.h"
#include "i2c.h"
#include "interrupt.h"
#include "ipc.h"
#include "mcbsp.h"
#include "memcfg.h"
#include "pin_map.h"
#include "pmbus.h"
#include "sci.h"
#include "sdfm.h"
#include "spi.h"
#include "sysctl.h"
#include "usb.h"
#include "xbar.h"

//
// Include MCAN driverlib header only if bitfield header is not already included
//
#ifndef F2838x_MCAN_H
#include "mcan.h"
#endif

#include "driver_inclusive_terminology_mapping.h"

#endif  // end of DRIVERLIB_H definition
