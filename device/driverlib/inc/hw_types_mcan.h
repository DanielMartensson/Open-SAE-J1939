/**
 * \file  hw_types.h
 *
 * \brief This file contains the in-line functions required to read/write
 *        values from/to the hardware registers. This file also contains field
 *        manipulation macros to get and set field values.
 *
 *  \copyright Copyright (C) 2013 Texas Instruments Incorporated - www.ti.com
 */

/**
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


#ifndef HW_TYPES_H_
#define HW_TYPES_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \brief   This macro reads a 32-bit value from a hardware register
 *           and returns the value.
 *
 *  \param   addr    Address of the memory mapped hardware register.
 *
 *  \return  Unsigned 32-bit value read from a register.
 */
#define HW_RD_REG32(addr) (uint32_t)(HW_RD_REG32_RAW((uint32_t) (addr)))

/**
 *  \brief   This macro writes a 32-bit value to a hardware register.
 *
 *  \param   addr    Address of the memory mapped hardware register.
 *  \param   value   unsigned 32-bit value which has to be written to the
 *                   register.
 */
#define HW_WR_REG32(addr, value)                                               \
    (HW_WR_REG32_RAW((uint32_t) (addr), (uint32_t) (value)))

/**
 *  \brief   This macro reads a 16-bit value from a hardware register
 *           and returns the value.
 *
 *  \param   addr    Address of the memory mapped hardware register.
 *
 *  \return  Unsigned 16-bit value read from a register.
 */
#define HW_RD_REG16(addr) (HW_RD_REG16_RAW((uint32_t) (addr)))

/**
 *  \brief   This macro writes a 16-bit value to a hardware register.
 *
 *  \param   addr    Address of the memory mapped hardware register.
 *  \param   value   unsigned 16-bit value which has to be written to the
 *                   register.
 */
#define HW_WR_REG16(addr, value)                                               \
    (HW_WR_REG16_RAW((uint32_t) (addr), (uint16_t) (value)))


/**
 *  \brief Macro to extract a field value. This macro extracts the field value
 *         from a 32-bit variable (which contains the register value).
 *         This macro does not read from actual register address, and only
 *         extracts the field from a variable.
 *
 *  \param regVal         32-bit variable containing the register value.
 *  \param REG_FIELD      Peripheral register bit field name, whose value has to
 *                        be extracted.
 */
/*TI_INSPECTED 76 S : MISRAC_2012_R.20.11
 * "The places in which order of evaluation has no side effects could be a
    waiver. This used in SHIFT and MASK to extract specified field value " */
/*TI_INSPECTED 125 S : MISRAC_2012_R.20.12
 * "The places in which order of evaluation has no side effects could be a
    waiver. ## operator here used in SHIFT and MASK to extract specified field
    value " */
#define HW_GET_FIELD(regVal, REG_FIELD)                                        \
    (((regVal) & (uint32_t) REG_FIELD##_MASK) >> (uint32_t) REG_FIELD##_SHIFT)

/**
 *  \brief Macro to write a specific field value. This macro first clears the
 *         specified field value and then performs "OR" of the field value which
 *         is shifted and masked. This will set the field value at its
 *         desired position.
 *
 *  \param regVal         32-bit variable containing the register value.
 *  \param REG_FIELD      Peripheral register bit field name, to which specified
 *                        value has to be set.
 *  \param fieldVal       Value of the field which has to be set.
 */
/*TI_INSPECTED 76 S : MISRAC_2012_R.20.11
 * "The places in which order of evaluation has no side effects could be a
    waiver. This used in SHIFT and MASK to extract specified field value " */
/*TI_INSPECTED 125 S : MISRAC_2012_R.20.12
 * "The places in which order of evaluation has no side effects could be a
    waiver. ## operator here used in SHIFT and MASK to extract specified field
    value " */
#define HW_SET_FIELD32(regVal, REG_FIELD, fieldVal)                            \
    ((regVal) = ((regVal) & (uint32_t) (~(uint32_t) REG_FIELD##_MASK)) |       \
                    ((((uint32_t) (fieldVal)) << (uint32_t) REG_FIELD##_SHIFT) \
                        & (uint32_t) REG_FIELD##_MASK))

/**
 *  \brief Macro to write a specific field value. This macro first clears the
 *         specified field value and then performs "OR" of the field value which
 *         is shifted and masked. This will set the field value at its
 *         desired position.
 *
 *  \param regVal         16-bit variable containing the register value.
 *  \param REG_FIELD      Peripheral register bit field name, to which specified
 *                        value has to be set.
 *  \param fieldVal       Value of the field which has to be set.
 */
/*TI_INSPECTED 76 S : MISRAC_2012_R.20.11
 * "The places in which order of evaluation has no side effects could be a
    waiver. This used in SHIFT and MASK to extract specified field value " */
/*TI_INSPECTED 125 S : MISRAC_2012_R.20.12
 * "The places in which order of evaluation has no side effects could be a
    waiver. ## operator here used in SHIFT and MASK to extract specified field
    value " */
#define HW_SET_FIELD16(regVal, REG_FIELD, fieldVal)                            \
    ((regVal) = ((regVal) & (uint16_t) (~(uint16_t) REG_FIELD##_MASK)) |       \
                    ((((uint16_t) (fieldVal)) << (uint16_t) REG_FIELD##_SHIFT) \
                        & (uint16_t) REG_FIELD##_MASK))

/**
 *  \brief This macro calls read-modify-write API for 32 bit register. It also
 *         frames the mask and shift from register field macro.
 *
 *  \param regAddr        Register Address.
 *  \param REG_FIELD      Peripheral register bit field name, to which specified
 *                        value has to be set.
 *  \param fieldVal       Value of the field which has to be set.
 */
/*TI_INSPECTED 76 S : MISRAC_2012_R.20.11
 * "The places in which order of evaluation has no side effects could be a
    waiver. This used in SHIFT and MASK to extract specified field value " */
/*TI_INSPECTED 125 S : MISRAC_2012_R.20.12
 * "The places in which order of evaluation has no side effects could be a
    waiver. ## operator here used in SHIFT and MASK to extract specified field
    value " */
#define HW_WR_FIELD32(regAddr, REG_FIELD, fieldVal)                            \
    (HW_WR_FIELD32_RAW((uint32_t) (regAddr), (uint32_t) REG_FIELD##_MASK,      \
                          (uint32_t) REG_FIELD##_SHIFT, (uint32_t) (fieldVal)))

/**
 *  \brief This macro calls read-modify-write API for 16 bit register. It also
 *         frames the mask and shift from register field macro.
 *
 *  \param regAddr        Register Address.
 *  \param REG_FIELD      Peripheral register bit field name, to which specified
 *                        value has to be set.
 *  \param fieldVal       Value of the field which has to be set.
 */
/*TI_INSPECTED 76 S : MISRAC_2012_R.20.11
 * "The places in which order of evaluation has no side effects could be a
    waiver. This used in SHIFT and MASK to extract specified field value " */
/*TI_INSPECTED 125 S : MISRAC_2012_R.20.12
 * "The places in which order of evaluation has no side effects could be a
    waiver. ## operator here used in SHIFT and MASK to extract specified field
    value " */
#define HW_WR_FIELD16(regAddr, REG_FIELD, fieldVal)                            \
    (HW_WR_FIELD16_RAW((uint32_t) (regAddr), (uint16_t) REG_FIELD##_MASK,      \
                          (uint32_t) REG_FIELD##_SHIFT, (uint16_t) (fieldVal)))

/**
 *  \brief This macro calls read field API for 32 bit register. It also
 *         frames the mask and shift from register field macro.
 *
 *  \param regAddr        Register Address.
 *  \param REG_FIELD      Peripheral register bit field name, from which
 *                        specified bit-field value has to be read.
 *  \return Value of the bit-field
 */
/*TI_INSPECTED 76 S : MISRAC_2012_R.20.11
 * "The places in which order of evaluation has no side effects could be a
    waiver. This used in SHIFT and MASK to extract specified field value " */
/*TI_INSPECTED 125 S : MISRAC_2012_R.20.12
 * "The places in which order of evaluation has no side effects could be a
    waiver. ## operator here used in SHIFT and MASK to extract specified field
    value " */
#define HW_RD_FIELD32(regAddr, REG_FIELD)                                      \
    (HW_RD_FIELD32_RAW((uint32_t) (regAddr), (uint32_t) REG_FIELD##_MASK,      \
                          (uint32_t) REG_FIELD##_SHIFT))

/**
 *  \brief This macro calls read field API for 16 bit register. It also
 *         frames the mask and shift from register field macro.
 *
 *  \param regAddr        Register Address.
 *  \param REG_FIELD      Peripheral register bit field name, from which
 *                        specified bit-field value has to be read.
 *  \return Value of the bit-field
 */
/*TI_INSPECTED 76 S : MISRAC_2012_R.20.11
 * "The places in which order of evaluation has no side effects could be a
    waiver. This used in SHIFT and MASK to extract specified field value " */
/*TI_INSPECTED 125 S : MISRAC_2012_R.20.12
 * "The places in which order of evaluation has no side effects could be a
    waiver. ## operator here used in SHIFT and MASK to extract specified field
    value " */
#define HW_RD_FIELD16(regAddr, REG_FIELD)                                      \
    (HW_RD_FIELD16_RAW((uint32_t) (regAddr), (uint16_t) REG_FIELD##_MASK,      \
                          (uint32_t) REG_FIELD##_SHIFT))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Global Variables Declarations                        */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief   This function reads a 32-bit value from a hardware register
 *           and returns the value.
 *
 *  \param   addr    Address of the memory mapped hardware register.
 *
 *  \return  Unsigned 32-bit value read from a register.
 */
static inline uint32_t HW_RD_REG32_RAW(uint32_t addr);

/**
 *  \brief   This function writes a 32-bit value to a hardware register.
 *
 *  \param   addr    Address of the memory mapped hardware register.
 *  \param   value   unsigned 32-bit value which has to be written to the
 *                   register.
 */
static inline void HW_WR_REG32_RAW(uint32_t addr, uint32_t value);

/**
 *  \brief   This function reads a 16-bit value from a hardware register
 *           and returns the value.
 *
 *  \param   addr    Address of the memory mapped hardware register.
 *
 *  \return  Unsigned 16-bit value read from a register.
 */
static inline uint16_t HW_RD_REG16_RAW(uint32_t addr);

/**
 *  \brief   This function writes a 16-bit value to a hardware register.
 *
 *  \param   addr    Address of the memory mapped hardware register.
 *  \param   value   unsigned 16-bit value which has to be written to the
 *                   register.
 */
static inline void HW_WR_REG16_RAW(uint32_t addr, uint16_t value);

/**
 *  \brief   This function reads a 32 bit register, modifies specific set of
 *           bits and writes back to the register.
 *
 *  \param   addr    Address of the memory mapped hardware register.
 *  \param   mask    Mask for the bit field.
 *  \param   shift   Bit field shift from LSB.
 *  \param   value   Value to be written to bit-field.
 */
static inline void HW_WR_FIELD32_RAW(uint32_t addr,
                                     uint32_t mask,
                                     uint32_t shift,
                                     uint32_t value);

/**
 *  \brief   This function reads a 16 bit register, modifies specific set of
 *           bits and writes back to the register.
 *
 *  \param   addr    Address of the memory mapped hardware register.
 *  \param   mask    Mask for the bit field.
 *  \param   shift   Bit field shift from LSB.
 *  \param   value   Value to be written to bit-field.
 */
static inline void HW_WR_FIELD16_RAW(uint32_t addr,
                                     uint16_t mask,
                                     uint32_t shift,
                                     uint16_t value);

/**
 *  \brief   This function reads a 32 bit register, masks specific set of bits
 *           and the left shifted value.
 *
 *  \param   addr    Address of the memory mapped hardware register.
 *  \param   mask    Mask for the bit field.
 *  \param   shift   Bit field shift from LSB.
 *
 *  \return  Bit-field value (absolute value - shifted to LSB position)
 */
static inline uint32_t HW_RD_FIELD32_RAW(uint32_t addr,
                                         uint32_t mask,
                                         uint32_t shift);

/**
 *  \brief   This function reads a 16 bit register, masks specific set of bits
 *           and the left shifted value.
 *
 *  \param   addr    Address of the memory mapped hardware register.
 *  \param   mask    Mask for the bit field.
 *  \param   shift   Bit field shift from LSB.
 *
 *  \return  Bit-field value (absolute value - shifted to LSB position)
 */
static inline uint16_t HW_RD_FIELD16_RAW(uint32_t addr,
                                         uint16_t mask,
                                         uint32_t shift);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/*TI_INSPECTED 35 S : MISRAC_2012_R.2.1
 * "Reason - This function is called using HW_RD_REG32 mcaro " */
/*TI_INSPECTED 101 S : MISRAC_2012_R.10.3
 * "Reason - Tool Issue: Not able to understand Inline Function return type " */
/*TI_INSPECTED 102 S : MISRAC_2012_R.8.4
 * "Reason - Tool Issue: Not able to understand Inline Function return type " */
static inline uint32_t HW_RD_REG32_RAW(uint32_t addr)
{
/*TI_INSPECTED 440 S : MISRAC_2012_R.11.4
 * "Reason - Pointer typecast required here since addr variable holds
 * required register value" */
    uint32_t regVal = *(volatile uint32_t *) addr;
    return (regVal);
}

/*TI_INSPECTED 35 S : MISRAC_2012_R.2.1
 * "Reason - This function is called using HW_WR_REG32 mcaro " */
/*TI_INSPECTED 131 S : MISRAC_2012_R.5.3
 * "Tool Issue - argument 'value' used here is in same scope " */
/*TI_INSPECTED 14 D 1: MISRAC_2012_R.5.8
 * "Tool Issue - Modifying the contents stored at address not the addr itself */
static inline void HW_WR_REG32_RAW(uint32_t addr, uint32_t value)
{
/*TI_INSPECTED 18 D : MISRAC_2012_R.5.3
 * "Tool Issue - argument 'value' used here is in same scope " */
    *(volatile uint32_t *) addr = value;
    return;
}

/*TI_INSPECTED 35 S : MISRAC_2012_R.2.1
 * "Reason - This function is called using HW_RD_REG16 mcaro " */
static inline uint16_t HW_RD_REG16_RAW(uint32_t addr)
{
/*TI_INSPECTED 440 S : MISRAC_2012_R.11.4
 * "Reason - Pointer typecast required here since addr variable holds
 * required register value" */
    uint16_t regVal = *(volatile uint16_t *) addr;
    return (regVal);
}

/*TI_INSPECTED 35 S : MISRAC_2012_R.2.1
 * "Reason - This function is called using HW_WR_REG16 mcaro " */
/*TI_INSPECTED 131 S : MISRAC_2012_R.5.3
 * "Tool Issue - argument 'value' used here is in same scope " */
/*TI_INSPECTED 14 D 1: MISRAC_2012_R.5.8
 * "Tool Issue - Modifying the contents stored at address not the addr itself */
static inline void HW_WR_REG16_RAW(uint32_t addr, uint16_t value)
{
/*TI_INSPECTED 18 D : MISRAC_2012_R.5.3
 * "Tool Issue - argument 'value' used here is in same scope " */
    *(volatile uint16_t *) addr = value;
    return;
}

/*TI_INSPECTED 35 S : MISRAC_2012_R.2.1
 * "Reason - This function is called using HW_WR_FIELD32 mcaro " */
/*TI_INSPECTED 14 D 1: MISRAC_2012_R.5.8
 * "Tool Issue - Modifying the contents stored at address not the addr itself */
static inline void HW_WR_FIELD32_RAW(uint32_t addr,
                                     uint32_t mask,
                                     uint32_t shift,
                                     uint32_t value)
{
/*TI_INSPECTED 440 S : MISRAC_2012_R.11.4
 * "Reason - Pointer typecast required here since addr variable holds
 * required register value" */
    uint32_t regVal = *(volatile uint32_t *) addr;
    regVal &= (~mask);
    regVal |= (value << shift) & mask;
/*TI_INSPECTED 440 S : MISRAC_2012_R.11.4
 * "Reason - Pointer typecast required here since addr variable holds
 * required register value" */
    *(volatile uint32_t *) addr = regVal;
    return;
}

/*TI_INSPECTED 35 S : MISRAC_2012_R.2.1
 * "Reason - This function is called using HW_WR_FIELD16 mcaro " */
/*TI_INSPECTED 14 D 1: MISRAC_2012_R.5.8
 * "Tool Issue - Modifying the contents stored at address not the addr itself */
static inline void HW_WR_FIELD16_RAW(uint32_t addr,
                                     uint16_t mask,
                                     uint32_t shift,
                                     uint16_t value)
{
    uint32_t tempVal;
/*TI_INSPECTED 440 S : MISRAC_2012_R.11.4
 * "Reason - Pointer typecast required here since addr variable holds
 * required register value" */
    uint16_t regVal = *(volatile uint16_t *) addr;
    tempVal = ((uint32_t) regVal);
    tempVal &=  (~((uint32_t) mask));
    tempVal |= (((uint32_t) value) << shift) & ((uint32_t) mask);
    regVal = (uint16_t) tempVal;
/*TI_INSPECTED 440 S : MISRAC_2012_R.11.4
 * "Reason - Pointer typecast required here since addr variable holds
 * required register value" */
    *(volatile uint16_t *) addr = regVal;
    return;
}

/*TI_INSPECTED 35 S : MISRAC_2012_R.2.1
 * "Reason - This function is called using HW_RD_FIELD32 mcaro " */
static inline uint32_t HW_RD_FIELD32_RAW(uint32_t addr,
                                         uint32_t mask,
                                         uint32_t shift)
{
/*TI_INSPECTED 440 S : MISRAC_2012_R.11.4
 * "Reason - Pointer typecast required here since addr variable holds
 * required register value" */
    uint32_t regVal = *(volatile uint32_t *) addr;
    regVal = (regVal & mask) >> shift;
    return (regVal);
}

/*TI_INSPECTED 35 S : MISRAC_2012_R.2.1
 * "Reason - This function is called using HW_RD_FIELD16 mcaro " */
static inline uint16_t HW_RD_FIELD16_RAW(uint32_t addr,
                                         uint16_t mask,
                                         uint32_t shift)
{
    uint32_t tempVal;
/*TI_INSPECTED 440 S : MISRAC_2012_R.11.4
 * "Reason - Pointer typecast required here since addr variable holds
 * required register value" */
    uint16_t regVal = *(volatile uint16_t *) addr;
    tempVal = (((uint32_t) regVal & (uint32_t) mask) >> shift);
    regVal = (uint16_t) tempVal;
    return (regVal);
}

#ifdef __cplusplus
}
#endif

#endif /* #ifndef HW_TYPES_H_ */
