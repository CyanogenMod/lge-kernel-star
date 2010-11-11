/*
 * Copyright (c) 2008-2009 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
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


