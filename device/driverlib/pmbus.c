//#############################################################################
//
// FILE:   pmbus.c
//
// TITLE:  C28x PMBUS driver
//
//#############################################################################
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

#include "pmbus.h"

//*****************************************************************************
//
// Globals
//
//*****************************************************************************
#if PMBUS_INCLUDE_CRC8_TABLE == 0x1U

//
// CRC table for the polynomial x^8+x^2+x^1+1 (0x7). File scope only
//
const uint16_t PMBus_crc8Table[256U] = {
    0x00U, 0x07U, 0x0EU, 0x09U, 0x1CU, 0x1BU, 0x12U, 0x15U,
    0x38U, 0x3FU, 0x36U, 0x31U, 0x24U, 0x23U, 0x2AU, 0x2DU,
    0x70U, 0x77U, 0x7EU, 0x79U, 0x6CU, 0x6BU, 0x62U, 0x65U,
    0x48U, 0x4FU, 0x46U, 0x41U, 0x54U, 0x53U, 0x5AU, 0x5DU,
    0xE0U, 0xE7U, 0xEEU, 0xE9U, 0xFCU, 0xFBU, 0xF2U, 0xF5U,
    0xD8U, 0xDFU, 0xD6U, 0xD1U, 0xC4U, 0xC3U, 0xCAU, 0xCDU,
    0x90U, 0x97U, 0x9EU, 0x99U, 0x8CU, 0x8BU, 0x82U, 0x85U,
    0xA8U, 0xAFU, 0xA6U, 0xA1U, 0xB4U, 0xB3U, 0xBAU, 0xBDU,
    0xC7U, 0xC0U, 0xC9U, 0xCEU, 0xDBU, 0xDCU, 0xD5U, 0xD2U,
    0xFFU, 0xF8U, 0xF1U, 0xF6U, 0xE3U, 0xE4U, 0xEDU, 0xEAU,
    0xB7U, 0xB0U, 0xB9U, 0xBEU, 0xABU, 0xACU, 0xA5U, 0xA2U,
    0x8FU, 0x88U, 0x81U, 0x86U, 0x93U, 0x94U, 0x9DU, 0x9AU,
    0x27U, 0x20U, 0x29U, 0x2EU, 0x3BU, 0x3CU, 0x35U, 0x32U,
    0x1FU, 0x18U, 0x11U, 0x16U, 0x03U, 0x04U, 0x0DU, 0x0AU,
    0x57U, 0x50U, 0x59U, 0x5EU, 0x4BU, 0x4CU, 0x45U, 0x42U,
    0x6FU, 0x68U, 0x61U, 0x66U, 0x73U, 0x74U, 0x7DU, 0x7AU,
    0x89U, 0x8EU, 0x87U, 0x80U, 0x95U, 0x92U, 0x9BU, 0x9CU,
    0xB1U, 0xB6U, 0xBFU, 0xB8U, 0xADU, 0xAAU, 0xA3U, 0xA4U,
    0xF9U, 0xFEU, 0xF7U, 0xF0U, 0xE5U, 0xE2U, 0xEBU, 0xECU,
    0xC1U, 0xC6U, 0xCFU, 0xC8U, 0xDDU, 0xDAU, 0xD3U, 0xD4U,
    0x69U, 0x6EU, 0x67U, 0x60U, 0x75U, 0x72U, 0x7BU, 0x7CU,
    0x51U, 0x56U, 0x5FU, 0x58U, 0x4DU, 0x4AU, 0x43U, 0x44U,
    0x19U, 0x1EU, 0x17U, 0x10U, 0x05U, 0x02U, 0x0BU, 0x0CU,
    0x21U, 0x26U, 0x2FU, 0x28U, 0x3DU, 0x3AU, 0x33U, 0x34U,
    0x4EU, 0x49U, 0x40U, 0x47U, 0x52U, 0x55U, 0x5CU, 0x5BU,
    0x76U, 0x71U, 0x78U, 0x7FU, 0x6AU, 0x6DU, 0x64U, 0x63U,
    0x3EU, 0x39U, 0x30U, 0x37U, 0x22U, 0x25U, 0x2CU, 0x2BU,
    0x06U, 0x01U, 0x08U, 0x0FU, 0x1AU, 0x1DU, 0x14U, 0x13U,
    0xAEU, 0xA9U, 0xA0U, 0xA7U, 0xB2U, 0xB5U, 0xBCU, 0xBBU,
    0x96U, 0x91U, 0x98U, 0x9FU, 0x8AU, 0x8DU, 0x84U, 0x83U,
    0xDEU, 0xD9U, 0xD0U, 0xD7U, 0xC2U, 0xC5U, 0xCCU, 0xCBU,
    0xE6U, 0xE1U, 0xE8U, 0xEFU, 0xFAU, 0xFDU, 0xF4U, 0xF3U,
};
#endif //PMBUS_INCLUDE_CRC8_TABLE == 0x1U


//*****************************************************************************
//
// PMBus_getInterruptStatus
//
//*****************************************************************************
uint32_t
PMBus_getInterruptStatus(uint32_t base)
{
    uint32_t intStatus;

    //
    // Create a bit mask of all the interrupt sources
    //
    const uint32_t bitmask = (
          PMBUS_PMBSTS_DATA_READY          |
          PMBUS_PMBSTS_DATA_REQUEST        |
          PMBUS_PMBSTS_EOM                 |
          PMBUS_PMBSTS_CLK_LOW_TIMEOUT     |
          PMBUS_PMBSTS_CLK_HIGH_DETECTED   |
          PMBUS_PMBSTS_TARGET_ADDR_READY   |
          PMBUS_PMBSTS_BUS_FREE            |
          PMBUS_PMBSTS_LOST_ARB            |
          PMBUS_PMBSTS_ALERT_EDGE          |
          PMBUS_PMBSTS_CONTROL_EDGE
          );


    //
    // Check the arguments.
    //
    ASSERT(PMBus_isBaseValid(base));

    intStatus = HWREG(base + PMBUS_O_PMBSTS) & 0x0003FFFFU;

    //
    // Mask off bits that are not an interrupt source and return
    //
    return(intStatus & bitmask);
}

//*****************************************************************************
//
// PMBus_initTargetMode
//
//*****************************************************************************
void PMBus_initTargetMode(uint32_t base, uint16_t address, uint16_t mask)
{
    //
    // Locals
    //
    uint32_t interruptState;

    //
    // Check the arguments.
    //
    ASSERT(PMBus_isBaseValid(base));
    ASSERT(address <= 0x7FU);
    ASSERT(mask <= 0x7FU);

    EALLOW;

    //
    // Save off the interrupt state and disable them
    //
    interruptState = HWREG(base + PMBUS_O_PMBINTM);
    HWREG(base + PMBUS_O_PMBINTM) = PMBUS_INT_ALL;

    //
    // PMBUS comes out of rest with target mode enabled. Disable controller mode,
    // if set, and enable controller mode
    //
    HWREG(base + PMBUS_O_PMBCTRL) &= ~(uint32_t)PMBUS_ENABLE_CONTROLLER_MODE;
    HWREG(base + PMBUS_O_PMBCTRL) |= PMBUS_ENABLE_TARGET_MODE;

    //
    // Zero out the target address and mask first
    //
    HWREG(base + PMBUS_O_PMBTCR) &= (~(uint32_t)PMBUS_PMBTCR_TARGET_ADDR_M &
                                    ~(uint32_t)PMBUS_PMBTCR_TARGET_MASK_M);


    //
    // Write the address and mask to PMBTCR
    //
    HWREG(base + PMBUS_O_PMBTCR) |= ((((uint32_t)address <<
                                                PMBUS_PMBTCR_TARGET_ADDR_S) &
                                                PMBUS_PMBTCR_TARGET_ADDR_M) |
                                             (((uint32_t)mask <<
                                                PMBUS_PMBTCR_TARGET_MASK_S) &
                                                PMBUS_PMBTCR_TARGET_MASK_M));

    //
    // Restore the interrupt status
    //
    HWREG(base + PMBUS_O_PMBINTM) = interruptState;

    EDIS;
}


//*****************************************************************************
//
// PMBus_configTarget
//
//*****************************************************************************
void PMBus_configTarget(uint32_t base, uint32_t configWord)
{
    //
    // Locals
    //
    uint32_t interruptState;

    //
    // Form a bit mask of the bit fields configWord changes
    //
    const uint32_t bitmask = (PMBUS_PMBTCR_MAN_TARGET_ACK | PMBUS_PMBTCR_PEC_ENA |
               PMBUS_PMBTCR_MAN_CMD | PMBUS_PMBTCR_RX_BYTE_ACK_CNT_M);

    //
    // Check the arguments.
    //
    ASSERT(PMBus_isBaseValid(base));

    EALLOW;

    //
    // Save off the interrupt state and disable them
    //
    interruptState = HWREG(base + PMBUS_O_PMBINTM);
    HWREG(base + PMBUS_O_PMBINTM) = PMBUS_INT_ALL;

    //
    // Zero out the bit fields that are changed by configWord, preserve the
    // rest that is, address and mask
    //
    HWREG(base + PMBUS_O_PMBTCR) &= ~bitmask;

    //
    // Write the user configured bit fields (passed in configWord)
    // do not alter address and mask
    //
    HWREG(base + PMBUS_O_PMBTCR) |= (configWord & bitmask);

    //
    // Restore the interrupt status
    //
    HWREG(base + PMBUS_O_PMBINTM) = interruptState;

    EDIS;
}

//*****************************************************************************
//
// PMBus_putTargetData
//
//*****************************************************************************
void
PMBus_putTargetData(uint32_t base, uint16_t *buffer, uint16_t nBytes,
                   bool txPEC)
{
    //
    // Locals
    //
    uint16_t i = 0U;
    uint32_t tx_data = 0U;
    uint32_t command;
    uint16_t *pBuffer = buffer;

    //
    // Check the arguments.
    //
    ASSERT(PMBus_isBaseValid(base));
    ASSERT((nBytes > 0U) && (nBytes <= 4U));
    ASSERT((txPEC == true) || (txPEC == false));
    EALLOW;

    //
    // Read the existing Target configuration, mask out the TX count
    //
    command = HWREG(base + PMBUS_O_PMBTCR);
    command &= ~PMBUS_PMBTCR_TX_COUNT_M;

    //
    // Update the transmit count and place the data into the transmit register.
    //
    command |= (((uint32_t)nBytes << PMBUS_PMBTCR_TX_COUNT_S) &
                                     PMBUS_PMBTCR_TX_COUNT_M);
    if(txPEC == 1U)
    {
        command |= PMBUS_PMBTCR_TX_PEC;
    }
    else
    {
        command &= ~(uint32_t)PMBUS_PMBTCR_TX_PEC;
    }

    //
    // Write to the Target Config register
    //
    HWREG(base + PMBUS_O_PMBTCR) = command;
    EDIS;

    //
    // Construct the transmit long word
    //
    while(i < nBytes)
    {
        tx_data |= (((uint32_t)*pBuffer & 0x000000FFUL) <<
                     ((uint32_t)i << 0x3U));
        i++;
        pBuffer++;
    }

    HWREG(base + PMBUS_O_PMBTXBUF) =  tx_data;

}

//*****************************************************************************
//
// PMBus_ackAddress
//
//*****************************************************************************
void
PMBus_ackAddress(uint32_t base, uint32_t address, uint32_t status,
                 uint16_t *buffer)
{
    //
    // Check the arguments.
    //
    ASSERT(PMBus_isBaseValid(base));
    ASSERT(address <= 0x7FUL);

    //
    // Check if manual target acknowledge mode is enabled
    //
    ASSERT((HWREG(base + PMBUS_O_PMBTCR) & PMBUS_PMBTCR_MAN_TARGET_ACK) != 0U);

    if((status & (uint32_t)PMBUS_PMBSTS_TARGET_ADDR_READY) != 0U)
    {
        //
        // Read the receive buffer
        //
        (void)PMBus_getTargetData(base, &buffer[0], status);

        //
        // Compare the low byte, the address, to the arguments
        //
        if((buffer[0] & 0x7FU) == address)
        {
            //
            // Acknowledge
            //
            HWREG(base + PMBUS_O_PMBACK) |= PMBUS_PMBACK_ACK;
        }
        else
        {
            //
            // NACK
            //
            HWREG(base + PMBUS_O_PMBACK) &= ~(uint32_t)PMBUS_PMBACK_ACK;
        }
    }
}

//*****************************************************************************
//
// PMBus_ackCommand
//
//*****************************************************************************
void
PMBus_ackCommand(uint32_t base, uint32_t command, uint32_t status,
                 uint16_t *buffer)
{
    //
    // Check the arguments.
    //
    ASSERT(PMBus_isBaseValid(base));
    ASSERT(command <= 0xFFUL);

    //
    // Check if manual target acknowledge mode is enabled
    //
    ASSERT((HWREG(base + PMBUS_O_PMBTCR) & PMBUS_PMBTCR_MAN_CMD) == 0U);

    if((status & (uint32_t)PMBUS_PMBSTS_DATA_REQUEST) != 0U)
    {
        //
        // Read the receive buffer
        //
        (void)PMBus_getTargetData(base, &buffer[0], status);

        //
        // Compare the low byte, the address, to the arguments
        //
        if((buffer[0] & 0xFFU) == command)
        {
            //
            // Acknowledge
            //
            HWREG(base + PMBUS_O_PMBACK) |= PMBUS_PMBACK_ACK;
        }
        else
        {
            //
            // NACK
            //
            HWREG(base + PMBUS_O_PMBACK) &= ~(uint32_t)PMBUS_PMBACK_ACK;
        }
    }
}

//*****************************************************************************
//
// PMBus_initContollerMode
//
//*****************************************************************************
void PMBus_initControllerMode(uint32_t base)
{
    //
    // Locals
    //
    uint32_t interruptState;

    //
    // Check the arguments.
    //
    ASSERT(PMBus_isBaseValid(base));

    EALLOW;

    //
    // Save off the interrupt state and disable them
    //
    interruptState = HWREG(base + PMBUS_O_PMBINTM);
    HWREG(base + PMBUS_O_PMBINTM) = PMBUS_INT_ALL;

    //
    // PMBUS comes out of rest with target mode enabled. Disable target mode
    // enable controller mode
    //
    HWREG(base + PMBUS_O_PMBCTRL) &= ~(uint32_t)PMBUS_ENABLE_TARGET_MODE;
    HWREG(base + PMBUS_O_PMBCTRL) |= PMBUS_ENABLE_CONTROLLER_MODE;

    //
    // Restore the interrupt status
    //
    HWREG(base + PMBUS_O_PMBINTM) = interruptState;
    EDIS;
}

//*****************************************************************************
//
// PMBus_generateCRCTable
//
//*****************************************************************************
void PMBus_generateCRCTable(uint16_t *crcTable)
{
    register uint16_t i;
    register uint16_t j;
    register uint16_t crc8_accum;

    for(i = 0U; i < 256U; i++)
    {
        crc8_accum = i;
        for(j = 0U; j < 8U; j++)
        {
            if((crc8_accum & 0x80U) != 0U)
            {
                crc8_accum = ((crc8_accum << 1U) & 0xffU) ^ (uint16_t)0x07U;
            }
            else
            {
                crc8_accum = (crc8_accum << 1U) & 0xffU;
            }
        }
        crcTable[i] = crc8_accum;
    }
}

//*****************************************************************************
//
// PMBus_verifyPEC
//
//*****************************************************************************
bool
PMBus_verifyPEC(uint32_t base, uint16_t *buffer, const uint16_t *crcTable,
                uint16_t byteCount, uint16_t pec)
{
    register uint16_t i;
    register uint16_t parity = 0U;
    register uint16_t tableIndex;
    register uint16_t accumulator;

    //
    // Check the arguments.
    //
    ASSERT(PMBus_isBaseValid(base));
    ASSERT(byteCount > 0U);
    ASSERT(pec < 0xFFU);

    //
    // PEC is always calculated first with the address + R/W bit
    //
    tableIndex  = HWREG(base + PMBUS_O_PMBHTA);

    accumulator = crcTable[tableIndex];

    //
    // The assumption is the message bytes are packed into 16-bit words
    // and the calculation starts from the low byte
    // The memory arrangement is as follows (ignore the addresses
    // Address|__LB__|__HB__|
    // 0x0000 |__D0__|__D1__|
    // 0x0001 |__D2__|__D3__|
    // 0x0002 |__D4__|__D5__|
    // 0x0003 |__D6__|__D7__|
    // 0x0004 |__D8__|__D9__|
    // ...
    //
    for(i = 0U; i < byteCount; i++)
    {
        //
        // __byte selects either the low(0) or high(1) byte in a word
        // the initial selection provided by the enumeration parity
        // We store each PMBUS byte in the low byte of successive words
        // hence parity is always even (0) and incremented by 2
        //

        tableIndex = accumulator ^ (uint16_t)__byte((int16_t *)buffer, parity);
        accumulator = crcTable[tableIndex];
        parity += 2U;
    }

    //
    // Compare the result with the PEC
    //
    return(accumulator == pec);
}


//*****************************************************************************
//
// PMBus_getData
//
//*****************************************************************************
uint16_t
PMBus_getData(uint32_t base, uint16_t *buffer, uint32_t status)
{

    //
    // Locals
    //
    uint32_t temp, i;
    uint16_t num_rx_bytes;

    //
    // Check the arguments.
    //
    ASSERT(PMBus_isBaseValid(base));

    //
    // Get the contents of the Receive buffer.
    //
    num_rx_bytes = (uint16_t)((status & PMBUS_PMBSTS_RD_BYTE_COUNT_M) >>
                                        PMBUS_PMBSTS_RD_BYTE_COUNT_S);

    if(num_rx_bytes != 0U)
    {
        temp = HWREG(base + PMBUS_O_PMBRXBUF);
        for(i = 0U; i < num_rx_bytes; i++)
        {
            buffer[i] = (uint16_t)(temp & 0x000000FFUL);
            temp      = temp >> 8U;
        }
    }

    return(num_rx_bytes);
}

//*****************************************************************************
//
// PMBus_putControllerData
//
//*****************************************************************************
void
PMBus_putControllerData(uint32_t base, uint16_t *buffer, uint16_t nBytes)
{
    //
    // Locals
    //
    uint16_t i = 0U;
    uint32_t tx_data = 0U;
    uint16_t *pBuffer = buffer;

    //
    // Check the arguments.
    //
    ASSERT(PMBus_isBaseValid(base));
    ASSERT((nBytes > 0U) && (nBytes <= 4U));

    while(i < nBytes)
    {
        tx_data |= (((uint32_t)*pBuffer & 0x000000FFUL) <<
                       ((uint32_t)i << 3U));
        pBuffer++;
        i++;
    }

    //
    // Write to the transmit register
    //
    HWREG(base + PMBUS_O_PMBTXBUF) = tx_data;

}

//*****************************************************************************
//
// PMBus_configModuleClock
//
//*****************************************************************************
uint32_t PMBus_configModuleClock(uint32_t base, uint32_t moduleFrequency,
               uint32_t sysFrequency)
{
    //
    // Locals
    //
    uint32_t interruptState, clockDivider, calcModuleFreq;

    //
    // Check the arguments.
    //
    ASSERT(PMBus_isBaseValid(base));
    ASSERT((moduleFrequency >= PMBUS_MODULE_FREQ_MIN) &&
           (moduleFrequency <= PMBUS_MODULE_FREQ_MAX));
    ASSERT((sysFrequency >= PMBUS_SYS_FREQ_MIN) &&
           (sysFrequency <= PMBUS_SYS_FREQ_MAX));

    EALLOW;

    //
    // Save off the interrupt state and disable them
    //
    interruptState = HWREG(base + PMBUS_O_PMBINTM);
    HWREG(base + PMBUS_O_PMBINTM) = PMBUS_INT_ALL;

    //
    // Calculate the clock divider. If the ratio of sysFrequency to
    // moduleFrequency is larger than 32, the divider is set to its maximum
    // possible value
    //
    clockDivider = (sysFrequency / moduleFrequency) - 1U;
    if(clockDivider > 31UL)
    {
        clockDivider = 31UL;
    }

    //
    // Write to the PMBCTRL register
    //
    HWREG(base + PMBUS_O_PMBCTRL) |=
                                  ((clockDivider << PMBUS_PMBCTRL_CLKDIV_S)
                                                   & PMBUS_PMBCTRL_CLKDIV_M);

    //
    // Calculate the actual bus frequency
    //
    calcModuleFreq = sysFrequency / (clockDivider + 1U);

    //
    // Restore the interrupt status
    //
    HWREG(base + PMBUS_O_PMBINTM) = interruptState;
    EDIS;
    return(calcModuleFreq);
}

//*****************************************************************************
//
// PMBus_configBusClock
//
//*****************************************************************************
bool PMBus_configBusClock(uint32_t base, PMBus_ClockMode mode,
                  uint32_t moduleFrequency)
{
    //
    // Locals
    //
    uint32_t interruptState, clockHighLimit, clockFreq;
    uint32_t clockLowTimeout, clockHighTimeout, timePeriod;
    uint32_t busIdle, setupTime;
    const uint32_t bitmask = PMBUS_PMBCTRL_FAST_MODE;
    bool status = false;

    //
    // Check the arguments.
    //
    ASSERT(PMBus_isBaseValid(base));
    ASSERT((moduleFrequency >= PMBUS_MODULE_FREQ_MIN) &&
           (moduleFrequency <= PMBUS_MODULE_FREQ_MAX));

    EALLOW;

    //
    // Save off the interrupt state and disable them
    //
    interruptState = HWREG(base + PMBUS_O_PMBINTM);
    HWREG(base + PMBUS_O_PMBINTM) = PMBUS_INT_ALL;

    //
    // Calculate the period for the module clock in ns
    //
    timePeriod = (uint32_t)1000000000UL / moduleFrequency;

    //
    // Disable FAST mode
    //
    HWREG(base + PMBUS_O_PMBCTRL) &= ~bitmask;

    //
    // Set status to true
    //
    status = true;

    switch(mode)
    {
        //
        // Standard Mode
        // Parameter                Timing Value in ns
        // CLK_LOW_LIMIT                   5000
        // CLK_HIGH_LIMIT                  5000
        // CLK_FREQ                        10000
        // START_LIMIT                     4700
        // busIdle                         50000
        // CLK_HIGH_TIMEOUT_LIMIT          50000
        // CLK_LOW_TIMEOUT_LIMIT           35000000
        // DATA_ACK_SETUP                  250
        //
        case PMBUS_CLOCKMODE_STANDARD:
            clockHighLimit    = (5000U / timePeriod) - 3U;
            clockFreq         = (10000U / timePeriod) - 4U;
            clockLowTimeout   = (35000000U / timePeriod) - 1U;
            clockHighTimeout  = (50000U / timePeriod) - 1U;
            busIdle           = (50000U / timePeriod) - 1U;
            setupTime         = (4700U / timePeriod);
        break;

        //
        // Fast Mode
        // Parameter                Timing Value in ns
        // CLK_LOW_LIMIT                   1250
        // CLK_HIGH_LIMIT                  1250
        // CLK_FREQ                        2500
        // START_LIMIT                     600
        // busIdle                         12500
        // CLK_HIGH_TIMEOUT_LIMIT          50000
        // CLK_LOW_TIMEOUT_LIMIT           35000000
        // DATA_ACK_SETUP                  100
        //
        case PMBUS_CLOCKMODE_FAST:
            clockHighLimit    = (1250U / timePeriod) - 3U;
            clockFreq         = (2500U / timePeriod) - 4U;
            clockLowTimeout   = (35000000U / timePeriod) - 1U;
            clockHighTimeout  = (50000U / timePeriod) - 1U;
            busIdle           = (12500U / timePeriod) - 1U;
            setupTime         = (600U / timePeriod);
            HWREG(base + PMBUS_O_PMBCTRL) |= PMBUS_PMBCTRL_FAST_MODE;
        break;

        default:
            clockHighLimit = 0U;
            clockFreq = 0U;
            clockLowTimeout = 0U;
            clockHighTimeout = 0U;
            busIdle = 0U;
            setupTime = 0U;
            status = false;
        break;
    }

    //
    // Write the above values to their respective registers
    // Override the timing control
    //
    HWREG(base + PMBUS_O_PMBTIMCTL) |= PMBUS_PMBTIMCTL_TIM_OVERRIDE;
    if(status)
    {
        HWREG(base + PMBUS_O_PMBTIMCLK) =
                    (((clockHighLimit << PMBUS_PMBTIMCLK_CLK_HIGH_LIMIT_S) &
                                         PMBUS_PMBTIMCLK_CLK_HIGH_LIMIT_M) |
                     ((clockFreq      << PMBUS_PMBTIMCLK_CLK_FREQ_S) &
                                         PMBUS_PMBTIMCLK_CLK_FREQ_M));
        HWREG(base + PMBUS_O_PMBTIMLOWTIMOUT) =
                 ((clockLowTimeout << PMBUS_PMBTIMLOWTIMOUT_CLKLOWTIMOUT_S) &
                                      PMBUS_PMBTIMLOWTIMOUT_CLKLOWTIMOUT_M);
        HWREG(base + PMBUS_O_PMBTIMHIGHTIMOUT) =
              ((clockHighTimeout << PMBUS_PMBTIMHIGHTIMOUT_CLKHIGHTIMOUT_S) &
                                    PMBUS_PMBTIMHIGHTIMOUT_CLKHIGHTIMOUT_M);
        HWREG(base + PMBUS_O_PMBTIMBIDLE) =
                                  ((busIdle << PMBUS_PMBTIMBIDLE_BUSIDLE_S) &
                                               PMBUS_PMBTIMBIDLE_BUSIDLE_M);
        HWREG(base + PMBUS_O_PMBTIMSTSETUP) =
                              ((setupTime << PMBUS_PMBTIMSTSETUP_TSU_STA_S) &
                                             PMBUS_PMBTIMSTSETUP_TSU_STA_M);
    }

    //
    // Restore the interrupt status
    //
    HWREG(base + PMBUS_O_PMBINTM) = interruptState;
    EDIS;
    return(status);
}

//
// End of file
//
