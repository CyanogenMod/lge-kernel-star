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


#define KEYPAD_HAS_DIODES 1

NvU32
NvOdmKbcFilterKeys(
    NvU32 *pRows,
    NvU32 *pCols,
    NvU32 NumOfKeysPressed)
{

#if KEYPAD_HAS_DIODES
    return NumOfKeysPressed;
#else
    NvU32 i=0;
    NvU32 j=0;
    NvU32 k=0;
    NvU32 FilterKeys[2] = {0};
    NvBool IsFiltered = NV_FALSE;
    NvU32 NewKeyPressCount = NumOfKeysPressed;
    
    if (NumOfKeysPressed <= 3)
    {
        for (i=0; i<NumOfKeysPressed; i++)
        {
            for (j=(i+1); j<NumOfKeysPressed; j++)
            {
                if ((pRows[i]+1==pRows[j])||(pRows[j]+1==pRows[i]))
                {
                    for (k=j; i<(NumOfKeysPressed - 1); i++)
                    {
                        pRows[k] = pRows[k+1];
                        pCols[k] = pCols[k+1];
                    }
                    NumOfKeysPressed--;
                }
                if ((pCols[i]+1==pCols[j])||(pCols[j]+1==pCols[i]))
                {
                    for (k=j; i<(NumOfKeysPressed - 1); i++)
                    {
                        pRows[k] = pRows[k+1];
                        pCols[k] = pCols[k+1];
                    }
                    NumOfKeysPressed--;
                }
            }
        }
        return NumOfKeysPressed;
    }   

    for (i=0; i<NumOfKeysPressed; i++)
    {
        for (j=(i+1); j<NumOfKeysPressed; j++)
        {
            if (pRows[i] == pRows[j])
            {
                for (k=0; k<NumOfKeysPressed; k++)
                {
                    if (k == i)
                        continue;

                    if(pCols[i] == pCols[k])
                    {
                        FilterKeys[0] = k;
                        IsFiltered = NV_TRUE;
                    }
                }
                for (k=0; k<NumOfKeysPressed; k++)
                {
                    if (k == j)
                        continue;
                    
                    if (pCols[j] == pCols[k])
                    {
                        FilterKeys[1] = k;
                        IsFiltered = NV_TRUE;
                    }
                }
                goto end;
            }
        }
    }

    end:
        if (IsFiltered)
        {
            for (i=FilterKeys[0]; i<(NumOfKeysPressed - 1); i++)
            {
                pRows[i] = pRows[i+1];
                pCols[i] = pCols[i+1];
            }
            NewKeyPressCount--;
            for (i=FilterKeys[1]; i<(NumOfKeysPressed - 1); i++)
            {
                pRows[i] = pRows[i+1];
                pCols[i] = pCols[i+1];
            }
            NewKeyPressCount--;
        }
        NumOfKeysPressed = NewKeyPressCount;
    return NewKeyPressCount;
#endif
    
}


