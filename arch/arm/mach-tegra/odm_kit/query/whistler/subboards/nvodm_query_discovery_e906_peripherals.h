/*
 * Copyright (c) 2009 NVIDIA Corporation.
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
 *         Implementation of the ODM Peripheral Discovery API</b>
 * 
 * @b Description: Specifies the peripheral connectivity
 *                 database Peripheral entries for the E906 LCD
 *                 Module.
 */

// LCD module
{
    NV_ODM_GUID('S','H','P','_','A','P','2','0'),   // Sharp WVGA panel with AP20 backlight control
    s_ffaMainDisplayAddresses,
    NV_ARRAY_SIZE(s_ffaMainDisplayAddresses),
    NvOdmPeripheralClass_Display,
},

// DSI module
{
    NV_ODM_GUID('s','h','a','r','p','d','s','i'),
    s_DsiAddresses,
    NV_ARRAY_SIZE(s_DsiAddresses),
    NvOdmPeripheralClass_Display,
},

//  Touch Panel
{
    NV_ODM_GUID('t','p','k','t','o','u','c','h'),
    s_ffaTouchPanelAddresses,
    NV_ARRAY_SIZE(s_ffaTouchPanelAddresses),
    NvOdmPeripheralClass_HCI
},

// NOTE: This list *must* end with a trailing comma.
