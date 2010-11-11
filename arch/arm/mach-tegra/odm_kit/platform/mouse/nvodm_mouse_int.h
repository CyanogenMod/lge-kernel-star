/*
 * Copyright (c) 2009 NVIDIA Corporation.
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

#ifndef INCLUDED_NVODM_MOUSE_INT_H
#define INCLUDED_NVODM_MOUSE_INT_H

#include "nvodm_services.h"
#include "nvodm_touch.h"
#include "nvec.h"

// Module debug: 0=disable, 1=enable
#define NVODMMOUSE_ENABLE_PRINTF (0)

#define MAX_NUM_MOUSE_PORTS       4
#define INVALID_MOUSE_PORT_ID     0xF
#define CMD_MAX_RETRIES           3

#if (NVODMMOUSE_ENABLE_PRINTF)
#define NVODMMOUSE_PRINTF(x)   NvOdmOsDebugPrintf x
#else
#define NVODMMOUSE_PRINTF(x)
#endif

#if defined(__cplusplus)
extern "C"
{
#endif

typedef struct NvOdmMouseDeviceRec
{
    NvEcHandle                  hEc;
    NvEcRequest                 *pRequest;
    NvEcResponse                *pResponse;
    NvEcEvent                   *pEvent;
    NvEcEventRegistrationHandle hEcEventRegister;
    NvBool                      CompressionEnabled;
    NvU8                        CompressionState;
    NvU32                       NumBytesPerSample;
    NvU32                       ValidMousePorts[MAX_NUM_MOUSE_PORTS + 1];
} NvOdmMouseDevice;

#if defined(__cplusplus)
}
#endif

/** @} */

#endif // INCLUDED_NVODM_MOUSE_INT_H

