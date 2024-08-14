//###########################################################################
//
// FILE:   erad.c
//
// TITLE:  C28x ERAD driver.
//
//###########################################################################

#include "erad.h"

//*****************************************************************************
//
// ERAD_configBusComp
//
//*****************************************************************************
void
ERAD_configBusComp(uint32_t base, ERAD_BusComp_Config config_params)
{
    //
    // Check if owner is APPLICATION or NO_OWNER
    //
    ASSERT((ERAD_getOwnership() == ERAD_OWNER_APPLICATION) ||
           (ERAD_getOwnership() == ERAD_OWNER_NOOWNER));

    //
    // Check if the base is valid
    //
    ASSERT(ERAD_isValidBusCompBase(base));

    //
    // Check if the bus comparator is in ERAD_STATE_IDLE
    //
    ASSERT(ERAD_getBusCompStatus(base) == ERAD_STATE_IDLE);

    //
    // Write into necessary registers to configure the bus comparator
    //
    EALLOW;
    HWREG(base + ERAD_O_HWBP_MASK) = config_params.mask;
    HWREG(base + ERAD_O_HWBP_REF)  = config_params.reference;

    //
    // Set the comparison mode and the CPU bus to be monitored
    // Enable interrupt and CPU halt if specified in the parameters
    //
    HWREGH(base + ERAD_O_HWBP_CNTL) =
        ((uint16_t)config_params.comp_mode   << ERAD_HWBP_CNTL_COMP_MODE_S) |
        ((uint16_t)config_params.bus_sel     << ERAD_HWBP_CNTL_BUS_SEL_S)   |
        ((uint16_t)config_params.enable_stop << ERAD_HWBP_CNTL_STOP_S)      |
        ((uint16_t)config_params.enable_int  << ERAD_HWBP_CNTL_RTOSINT_S);
    EDIS;
}

//*****************************************************************************
//
// ERAD_configCounterInCountingMode
//
//*****************************************************************************
void
ERAD_configCounterInCountingMode(uint32_t base,
                                 ERAD_Counter_Config config_params)
{
    //
    // Check if owner is APPLICATION or NO_OWNER
    //
    ASSERT((ERAD_getOwnership() == ERAD_OWNER_APPLICATION) ||
           (ERAD_getOwnership() == ERAD_OWNER_NOOWNER));

    //
    // Check if the base is valid
    //
    ASSERT(ERAD_isValidCounterBase(base));

    //
    // Check if the counter is in ERAD_STATE_IDLE
    //
    ASSERT(ERAD_getCounterStatus(base) == ERAD_STATE_IDLE);

    //
    // Write into registers to configure the counter
    //
    EALLOW;

    HWREG(base + ERAD_O_CTM_REF) = config_params.reference;

    if(config_params.event == ERAD_EVENT_NO_EVENT)
    {
        //
        // If ERAD_EVENT_NO_EVENT is selected, clear the counter input select
        // enable bit. The counter will count CPU cycles.
        //
        HWREGH(base + ERAD_O_CTM_CNTL) &= ERAD_CTM_CNTL_CNT_INP_SEL_EN;
    }
    else
    {
        //
        // For any other value, set the event to be counted and counter input
        // select enable bit.
        //
        HWREGH(base + ERAD_O_CTM_INPUT_SEL) =
          (((uint16_t)config_params.event << ERAD_CTM_INPUT_SEL_CNT_INP_SEL_S) &
          ERAD_CTM_INPUT_SEL_CNT_INP_SEL_M);

        HWREGH(base + ERAD_O_CTM_CNTL) |= ERAD_CTM_CNTL_CNT_INP_SEL_EN;
    }

    //
    // Set the counter in Normal count mode
    // Set the event mode as Rising edge or Active
    // Enable interrupt, CPU halt and reset on match if specified in the
    // parameters
    //
    HWREGH(base + ERAD_O_CTM_CNTL) =
      (HWREGH(base + ERAD_O_CTM_CNTL) & (~(ERAD_CTM_CNTL_START_STOP_CUMULATIVE |
                                           ERAD_CTM_CNTL_RTOSINT      |
                                           ERAD_CTM_CNTL_STOP         |
                                           ERAD_CTM_CNTL_RST_ON_MATCH |
                                           ERAD_CTM_CNTL_EVENT_MODE   |
                                           ERAD_CTM_CNTL_START_STOP_MODE))) |
      ((uint16_t)config_params.enable_int   << ERAD_CTM_CNTL_RTOSINT_S)     |
      ((uint16_t)config_params.enable_stop  << ERAD_CTM_CNTL_STOP_S)        |
      ((uint16_t)config_params.event_mode   << ERAD_CTM_CNTL_EVENT_MODE_S)  |
      ((uint16_t)config_params.rst_on_match << ERAD_CTM_CNTL_RST_ON_MATCH_S);

    EDIS;
}

//*****************************************************************************
//
// ERAD_configCounterInStartStopMode
//
//*****************************************************************************
void
ERAD_configCounterInStartStopMode(uint32_t base,
                                  ERAD_Counter_Config config_params,
                                  ERAD_Counter_Input_Event start_event,
                                  ERAD_Counter_Input_Event stop_event)
{
    //
    // Check if owner is APPLICATION or NO_OWNER
    //
    ASSERT((ERAD_getOwnership() == ERAD_OWNER_APPLICATION) ||
                                  (ERAD_getOwnership() == ERAD_OWNER_NOOWNER));

    //
    // Check if the base is valid
    //
    ASSERT(ERAD_isValidCounterBase(base));

    //
    // Check if the counter is in IDLE state
    //
    ASSERT(ERAD_getCounterStatus(base) == ERAD_STATE_IDLE);

    //
    // Write into registers to configure the counter
    //
    EALLOW;

    HWREG(base + ERAD_O_CTM_REF) = config_params.reference;

    if(config_params.event == ERAD_EVENT_NO_EVENT)
    {
        //
        // If ERAD_EVENT_NO_EVENT is selected, clear the counter input select
        // enable bit. The counter will count CPU cycles.
        // Set start and stop events
        //
        HWREGH(base + ERAD_O_CTM_CNTL) &= ~ERAD_CTM_CNTL_CNT_INP_SEL_EN;

        HWREGH(base + ERAD_O_CTM_INPUT_SEL) =
                ((uint16_t)start_event << ERAD_CTM_INPUT_SEL_STA_INP_SEL_S);

        HWREGH(base + ERAD_O_CTM_INPUT_SEL_2) |=
                ((uint16_t)stop_event << ERAD_CTM_INPUT_SEL_2_STO_INP_SEL_S);

    }
    else
    {
        //
        // For any other value, set the counter input select enable bit and
        // configure the counter input, start event and stop event,
        //
        //
        HWREGH(base + ERAD_O_CTM_CNTL) |= ERAD_CTM_CNTL_CNT_INP_SEL_EN;

        HWREGH(base + ERAD_O_CTM_INPUT_SEL) =
           ((uint16_t)config_params.event << ERAD_CTM_INPUT_SEL_CNT_INP_SEL_S) |
           ((uint16_t)start_event         << ERAD_CTM_INPUT_SEL_STA_INP_SEL_S);

        HWREGH(base + ERAD_O_CTM_INPUT_SEL_2) |=
           ((uint16_t)stop_event << ERAD_CTM_INPUT_SEL_2_STO_INP_SEL_S);

    }

    //
    // Set the counter in Normal count mode
    // Set the event mode as Rising edge or Active
    // Enable interrupt, CPU halt and reset on match if specified in the
    // parameters
    //
    HWREGH(base + ERAD_O_CTM_CNTL) =
      (HWREGH(base + ERAD_O_CTM_CNTL) & (~(ERAD_CTM_CNTL_START_STOP_CUMULATIVE |
                                           ERAD_CTM_CNTL_RTOSINT      |
                                           ERAD_CTM_CNTL_STOP         |
                                           ERAD_CTM_CNTL_RST_ON_MATCH |
                                           ERAD_CTM_CNTL_EVENT_MODE)))     |
      ERAD_CTM_CNTL_START_STOP_MODE                                        |
      ((uint16_t)config_params.enable_int   << ERAD_CTM_CNTL_RTOSINT_S)    |
      ((uint16_t)config_params.enable_stop  << ERAD_CTM_CNTL_STOP_S)       |
      ((uint16_t)config_params.event_mode   << ERAD_CTM_CNTL_EVENT_MODE_S) |
      ((uint16_t)config_params.rst_on_match << ERAD_CTM_CNTL_RST_ON_MATCH_S);

    EDIS;
}

//*****************************************************************************
//
// ERAD_configCounterInCumulativeMode
//
//*****************************************************************************
void
ERAD_configCounterInCumulativeMode(uint32_t base,
                                   ERAD_Counter_Config config_params,
                                   ERAD_Counter_Input_Event start_event,
                                   ERAD_Counter_Input_Event stop_event)
{
    //
    // Check if owner is APPLICATION or NO_OWNER
    //
    ASSERT((ERAD_getOwnership() == ERAD_OWNER_APPLICATION) ||
                                  (ERAD_getOwnership() == ERAD_OWNER_NOOWNER));

    //
    // Check if the base is valid
    //
    ASSERT(ERAD_isValidCounterBase(base));

    //
    // Check if the counter is in IDLE state
    //
    ASSERT(ERAD_getCounterStatus(base) == ERAD_STATE_IDLE);

    //
    // Write into registers to configure the counter
    //
    EALLOW;

    HWREG(base + ERAD_O_CTM_REF) = config_params.reference;

    if(config_params.event == ERAD_EVENT_NO_EVENT)
    {
        //
        // If ERAD_EVENT_NO_EVENT is selected, clear the counter input select
        // enable bit. The counter will count CPU cycles.
        // Set start and stop events
        //
        HWREGH(base + ERAD_O_CTM_CNTL) &= ~ERAD_CTM_CNTL_CNT_INP_SEL_EN;

        HWREGH(base + ERAD_O_CTM_INPUT_SEL) =
                ((uint16_t)start_event << ERAD_CTM_INPUT_SEL_STA_INP_SEL_S);

        HWREGH(base + ERAD_O_CTM_INPUT_SEL_2) |=
                ((uint16_t)stop_event << ERAD_CTM_INPUT_SEL_2_STO_INP_SEL_S);

    }
    else
    {
        //
        // For any other value, set the counter input select enable bit and
        // configure the counter input, start event and stop event,
        //
        //
        HWREGH(base + ERAD_O_CTM_CNTL) |= ERAD_CTM_CNTL_CNT_INP_SEL_EN;

        HWREGH(base + ERAD_O_CTM_INPUT_SEL) =
           ((uint16_t)config_params.event << ERAD_CTM_INPUT_SEL_CNT_INP_SEL_S) |
           ((uint16_t)start_event         << ERAD_CTM_INPUT_SEL_STA_INP_SEL_S);

        HWREGH(base + ERAD_O_CTM_INPUT_SEL_2) |=
            ((uint16_t)stop_event << ERAD_CTM_INPUT_SEL_2_STO_INP_SEL_S);

    }

    //
    // Set the counter in Normal count mode
    // Set the event mode as Rising edge or Active
    // Enable interrupt, CPU halt and reset on match if specified in the
    // parameters
    //
    HWREGH(base + ERAD_O_CTM_CNTL) =
        (HWREGH(base + ERAD_O_CTM_CNTL) & (~(ERAD_CTM_CNTL_RTOSINT      |
                                             ERAD_CTM_CNTL_STOP         |
                                             ERAD_CTM_CNTL_RST_ON_MATCH |
                                             ERAD_CTM_CNTL_EVENT_MODE)))     |
        ERAD_CTM_CNTL_START_STOP_MODE                                        |
        ERAD_CTM_CNTL_START_STOP_CUMULATIVE                                  |
        ((uint16_t)config_params.enable_int   << ERAD_CTM_CNTL_RTOSINT_S)    |
        ((uint16_t)config_params.enable_stop  << ERAD_CTM_CNTL_STOP_S)       |
        ((uint16_t)config_params.event_mode   << ERAD_CTM_CNTL_EVENT_MODE_S) |
        ((uint16_t)config_params.rst_on_match << ERAD_CTM_CNTL_RST_ON_MATCH_S);

    EDIS;

}

//*****************************************************************************
//
// ERAD_configMask
//
//*****************************************************************************
void
ERAD_configMask(ERAD_Mask mask, uint32_t instances, bool enable_int)
{
    uint16_t bitpos;
    uint16_t mask_tmp = (uint16_t)mask;

    EALLOW;

    if(mask_tmp < 4U)
    {
        bitpos = mask_tmp << 3U;

        HWREG(ERAD_GLOBAL_BASE + ERAD_O_GLBL_EVENT_AND_MASK) =
                    (HWREG(ERAD_GLOBAL_BASE + ERAD_O_GLBL_EVENT_AND_MASK) |
                     (0xFFUL << bitpos)) & (~(instances << bitpos));

        bitpos = mask_tmp;

        if(enable_int)
        {
            HWREGH(ERAD_GLOBAL_BASE + ERAD_O_GLBL_AND_EVENT_INT_MASK) &=
                    ~(1U << bitpos);
        }
        else
        {
            HWREGH(ERAD_GLOBAL_BASE + ERAD_O_GLBL_AND_EVENT_INT_MASK) |=
                    (1U << bitpos);
        }
    }
    else
    {
        bitpos = (mask_tmp - 4U) << 3U;

        HWREG(ERAD_GLOBAL_BASE + ERAD_O_GLBL_EVENT_OR_MASK) =
                    (HWREG(ERAD_GLOBAL_BASE + ERAD_O_GLBL_EVENT_OR_MASK) |
                     (0xFFUL << bitpos)) & (~(instances << bitpos));

        bitpos = mask_tmp - 4U;

        if(enable_int)
        {
            HWREGH(ERAD_GLOBAL_BASE + ERAD_O_GLBL_OR_EVENT_INT_MASK) &=
                    ~(1U << bitpos);
        }
        else
        {
            HWREGH(ERAD_GLOBAL_BASE + ERAD_O_GLBL_OR_EVENT_INT_MASK) |=
                    (1U << bitpos);
        }
    }
    EDIS;
}
//*****************************************************************************
//
// ERAD_profile
//
//*****************************************************************************
void
ERAD_profile(ERAD_Profile_Params config_params)
{
    ERAD_BusComp_Config buscomp1_params, buscomp2_params;
    ERAD_Counter_Config counter_params;

    //
    // Clear any previous events
    //
    ERAD_clearBusCompEvent(config_params.busComp_base1);
    ERAD_clearBusCompEvent(config_params.busComp_base2);

    //
    // Clear any previous events and overflow
    //
    ERAD_clearCounterEvent(config_params.counter_base);
    ERAD_clearCounterOverflow(config_params.counter_base);

    //
    // Reset the counter
    //
    ERAD_resetCounter(ERAD_getCounterInstance(config_params.counter_base));

    //
    // Configure the first bus comparator
    // No interrupt or CPU halt. Compare for equality. No mask.
    // Reference is the address of the first instruction
    //
    buscomp1_params.bus_sel = config_params.bus_sel;
    buscomp1_params.comp_mode = ERAD_BUSCOMP_COMPMODE_EQ;
    buscomp1_params.enable_stop = 0;
    buscomp1_params.enable_int = 0;
    buscomp1_params.mask = 0x0;
    buscomp1_params.reference = config_params.start_address;
    ERAD_configBusComp(config_params.busComp_base1, buscomp1_params);

    //
    // Configure the second bus comparator
    // No interrupt or CPU halt. Compare for equality. No mask.
    // Reference is the address of the last instruction
    //
    buscomp2_params.bus_sel = config_params.bus_sel;
    buscomp2_params.comp_mode = ERAD_BUSCOMP_COMPMODE_EQ;
    buscomp2_params.enable_stop = 0;
    buscomp2_params.enable_int = 0;
    buscomp2_params.mask = 0x0;
    buscomp2_params.reference = config_params.end_address;
    ERAD_configBusComp(config_params.busComp_base2, buscomp2_params);

    //
    // Configure the counter
    // No interrupt, no CPU halt, no reset on match, no reference.
    // Counting the number of CPU cycles
    // Start event is the first bus comparator and stop is the second
    //
    counter_params.enable_int = 0;
    counter_params.enable_stop = 0;
    counter_params.rst_on_match = 0;
    counter_params.reference = 0x0;
    counter_params.event_mode = ERAD_COUNTER_MODE_ACTIVE;
    counter_params.event = ERAD_EVENT_NO_EVENT;
    ERAD_Counter_Input_Event start = (ERAD_Counter_Input_Event)
                    (ERAD_BUSCOMP_BASE_TO_EVENT(config_params.busComp_base1));
    ERAD_Counter_Input_Event stop = (ERAD_Counter_Input_Event)
                    (ERAD_BUSCOMP_BASE_TO_EVENT(config_params.busComp_base2));
    ERAD_configCounterInStartStopMode(config_params.counter_base,
                                        counter_params, start, stop);

    //
    // Enable the modules
    //
    ERAD_enableModules(ERAD_getBusCompInstance(config_params.busComp_base1) |
                       ERAD_getBusCompInstance(config_params.busComp_base2) |
                       ERAD_getCounterInstance(config_params.counter_base));
}

//*****************************************************************************
//
// ERAD_enableInterruptAtAddress
//
//*****************************************************************************
void
ERAD_enableInterruptOnAddressHit(ERAD_AddressHit_Params config_params,
                                 uint32_t busComp_base)
{
    ERAD_BusComp_Config buscomp_params;

    //
    // Clear any previous events
    //
    ERAD_clearBusCompEvent(busComp_base);

    //
    // Set the address and mask. Disable CPU halt
    // and enable Interrupt
    //
    buscomp_params.reference = config_params.address;
    buscomp_params.mask = config_params.mask;
    buscomp_params.enable_stop = 0;
    buscomp_params.enable_int = 1;

    //
    // Comparison mode is set to equality and the bus to be
    // monitored is the Virtual Program Counter
    //
    buscomp_params.comp_mode = ERAD_BUSCOMP_COMPMODE_EQ;
    buscomp_params.bus_sel = config_params.bus_sel;

    //
    // Configure the bus comparator
    //
    ERAD_configBusComp(busComp_base, buscomp_params);

    //
    // Enable the bus comparator
    //
    ERAD_enableModules(ERAD_getBusCompInstance(busComp_base));
}

//*****************************************************************************
//
// ERAD_countEventHits
//
//*****************************************************************************
void
ERAD_countAddressHits(ERAD_AddressHit_Params config_params,
                      uint32_t busComp_base, uint32_t counter_base)
{
    ERAD_BusComp_Config buscomp_params;
    ERAD_Counter_Config counter_params;

    //
    // Clear any previous events and overflows
    //
    ERAD_clearBusCompEvent(busComp_base);
    ERAD_clearCounterEvent(counter_base);
    ERAD_clearCounterOverflow(counter_base);

    //
    // Reset the counter
    //
    ERAD_resetCounter(ERAD_getCounterInstance(counter_base));

    //
    // Configure the bus comparator
    // No interrupt or CPU halt. Compare for equality. No mask.
    // Reference is the address of the instruction to be counted
    // Bus to be monitored is specified in the bus argument
    //
    buscomp_params.reference = config_params.address;
    buscomp_params.mask = config_params.mask;
    buscomp_params.comp_mode = ERAD_BUSCOMP_COMPMODE_EQ;
    buscomp_params.enable_stop = 0;
    buscomp_params.bus_sel = config_params.bus_sel;
    buscomp_params.enable_int = 0;
    ERAD_configBusComp(busComp_base, buscomp_params);

    //
    // Configure the counter
    // No interrupt, no CPU halt, no reset on match, no reference.
    // Counting the number of rising edges of the bus comparator event
    //
    counter_params.enable_int = 0;
    counter_params.enable_stop = 0;
    counter_params.event_mode = ERAD_COUNTER_MODE_RISING_EDGE;
    counter_params.rst_on_match = 0;
    counter_params.reference = 0x0;
    counter_params.event = (ERAD_Counter_Input_Event)
                      (ERAD_BUSCOMP_BASE_TO_EVENT(busComp_base));
    ERAD_configCounterInCountingMode(counter_base, counter_params);

    //
    // Enable the modules
    //
    ERAD_enableModules(ERAD_getBusCompInstance(busComp_base) |
                       ERAD_getCounterInstance(counter_base));
}

//
// End of file
//
