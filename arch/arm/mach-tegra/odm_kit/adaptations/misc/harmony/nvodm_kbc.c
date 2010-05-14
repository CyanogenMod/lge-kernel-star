/*
 * Copyright (c) 2008-2009 NVIDIA Corporation.
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
 * @file          Nvodm_Kbc.c
 * @brief         <b>KBC odm implementation</b>
 *
 * @Description : Implementation of the odm KBC API
 */
#include "nvodm_kbc.h"
#include "../../../query/harmony/nvodm_query_kbc_qwerty_def.h"

NvU32
NvOdmKbcFilterKeys(
    NvU32 *pRows,
    NvU32 *pCols,
    NvU32 NumOfKeysPressed)
{
    NvBool IsFunctionKeyFound = NV_FALSE;
    NvU32 KeyIndex;
    
    for (KeyIndex = 0; KeyIndex < NumOfKeysPressed; ++KeyIndex)
    {
        if ((pRows[KeyIndex] == KBC_QWERTY_FUNCTION_KEY_ROW_NUMBER) &&
                (pCols[KeyIndex] == KBC_QWERTY_FUNCTION_KEY_COLUMN_NUMBER))
        {
            IsFunctionKeyFound = NV_TRUE;
            break;
        }
    }
    if (!IsFunctionKeyFound)
        return NumOfKeysPressed;

    // Add function row base to treat as special case
    for (KeyIndex = 0; KeyIndex < NumOfKeysPressed; ++KeyIndex)
        pRows[KeyIndex] += KBC_QWERTY_FUNCTION_KEY_ROW_BASE;
        
    return NumOfKeysPressed;
}


