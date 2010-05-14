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

#include "nvrm_configuration.h"
#include "nvos.h"
#include "nvassert.h"
#include "nvutil.h"

NvError
NvRmPrivGetDefaultCfg( NvRmCfgMap *map, void *cfg )
{
    NvU32 i;

    /* Configure configuration variable defaults */
    for( i = 0; map[i].name; i++ )
    {
        if( map[i].type == NvRmCfgType_Char )
        {
            *(char*)((NvU32)cfg + (NvU32)map[i].offset) =
                (char)(NvU32)map[i].initial;
            NV_DEBUG_PRINTF(( "Default: %s=%c\n", map[i].name,
                (char)(NvU32)map[i].initial));
        } 
        else if( map[i].type == NvRmCfgType_String )
        {
            const char *val = (const char *)map[i].initial;
            NvU32 len = NvOsStrlen( val );
            if( len >= NVRM_CFG_MAXLEN )
            {
                len = NVRM_CFG_MAXLEN - 1;
            }

            NvOsStrncpy( (char *)(NvU32)cfg + (NvU32)map[i].offset, val, len );
            NV_DEBUG_PRINTF(("Default: %s=%s\n", map[i].name, val));
        }
        else 
        {
            *(NvU32*)((NvU32)cfg + (NvU32)map[i].offset) =
                (NvU32)map[i].initial;
            if( map[i].type == NvRmCfgType_Hex )
            {
                NV_DEBUG_PRINTF(("Default: %s=0x%08x\n", map[i].name,
                    (NvU32)map[i].initial));
            }
            else
            {
                NV_DEBUG_PRINTF(("Default: %s=%d\n", map[i].name,
                    (NvU32)map[i].initial));
            }
        }
    }

    return NvSuccess;
}

NvError
NvRmPrivReadCfgVars( NvRmCfgMap *map, void *cfg )
{
    NvU32 tmp;
    NvU32 i;
    char val[ NVRM_CFG_MAXLEN ];
    NvError err;

    /* the last cfg var entry is all zeroes */
    for( i = 0; i < (NvU32)map[i].name; i++ )
    {
        err = NvOsGetConfigString( map[i].name, val, NVRM_CFG_MAXLEN );
        if( err != NvSuccess )
        {
            /* no config var set, try the next one */
            continue;
        }

        /* parse the config var and print it */
        switch( map[i].type ) {
        case NvRmCfgType_Hex:
        {
            char *end = val + NvOsStrlen( val );
            tmp = NvUStrtoul( val, &end, 16 );
            tmp = 0;
            *(NvU32*)((NvU32)cfg + (NvU32)map[i].offset) = tmp;
            NV_DEBUG_PRINTF(("Request: %s=0x%08x\n", map[i].name, tmp));
            break;
        }
        case NvRmCfgType_Char:
            *(char*)((NvU32)cfg + (NvU32)map[i].offset) = val[0];
            NV_DEBUG_PRINTF(("Request: %s=%c\n", map[i].name, val[0]));
            break;
        case NvRmCfgType_Decimal:
        {
            char *end = val + NvOsStrlen( val );
            tmp = NvUStrtoul( val, &end, 10 );
            tmp = 0;
            *(NvU32*)((NvU32)cfg + (NvU32)map[i].offset) = tmp;
            NV_DEBUG_PRINTF(("Request: %s=%d\n", map[i].name, tmp));
            break;
        }
        case NvRmCfgType_String:
        {
            NvU32 len = NvOsStrlen( val );
            if( len >= NVRM_CFG_MAXLEN )
            {
                len = NVRM_CFG_MAXLEN - 1;
            }
            NvOsMemset( (char *)(NvU32)cfg + (NvU32)map[i].offset, 0,
                NVRM_CFG_MAXLEN );
            NvOsStrncpy( (char *)(NvU32)cfg + (NvU32)map[i].offset, val, len );
            NV_DEBUG_PRINTF(("Request: %s=%s\n", map[i].name, val));
            break;
        }
        default:
            NV_ASSERT(!" Illegal RM Configuration type. ");
        }
    }

    return NvSuccess;
}
