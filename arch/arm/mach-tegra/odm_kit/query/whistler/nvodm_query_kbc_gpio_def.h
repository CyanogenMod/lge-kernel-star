/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 * <b>NVIDIA APX ODM Kit::
 *         The KBC GPIO pin definitions</b>
 *
 * @b Description: Define the KBC GPIO pins in row and column numbers.
 */

#ifndef NVODM_QUERY_KBC_GPIO_DEF_H
#define NVODM_QUERY_KBC_GPIO_DEF_H

typedef enum
{
    NvOdmKbcGpioPin_KBRow0 = 0,
    NvOdmKbcGpioPin_KBRow1,
    NvOdmKbcGpioPin_KBRow2,
    NvOdmKbcGpioPin_KBRow3,
    NvOdmKbcGpioPin_KBRow4,
    NvOdmKbcGpioPin_KBRow5,
    NvOdmKbcGpioPin_KBRow6,
    NvOdmKbcGpioPin_KBRow7,
    NvOdmKbcGpioPin_KBRow8,
    NvOdmKbcGpioPin_KBRow9,
    NvOdmKbcGpioPin_KBRow10,
    NvOdmKbcGpioPin_KBRow11,
    NvOdmKbcGpioPin_KBRow12,
    NvOdmKbcGpioPin_KBRow13,
    NvOdmKbcGpioPin_KBRow14,
    NvOdmKbcGpioPin_KBRow15,
    NvOdmKbcGpioPin_KBCol0,
    NvOdmKbcGpioPin_KBCol1,
    NvOdmKbcGpioPin_KBCol2,
    NvOdmKbcGpioPin_KBCol3,
    NvOdmKbcGpioPin_KBCol4,
    NvOdmKbcGpioPin_KBCol5,
    NvOdmKbcGpioPin_KBCol6,
    NvOdmKbcGpioPin_KBCol7,
    NvOdmKbcGpioPin_Num,
    NvOdmKbcGpioPin_Force32 = 0x7FFFFFFF
}NvOdmKbcGpioPin;

#endif // NVODM_QUERY_KBC_GPIO_DEF_H
