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
 * @brief <b>nVIDIA Driver Development Kit:
 *           DMA Resource manager private API for Hw access </b>
 *
 * @b Description: Implements the private interface of the hw access NvRM DMA.
 * This files implements the API for accessing the register of the AP15 Dma
 * controller.
 *
 */

#include "nvcommon.h"
#include "nvrm_interrupt.h"
#include "nvassert.h"
#include "nvrm_hwintf.h"
#include "nvrm_processor.h"
#include "nvrm_drf.h"
#include "ap15/arapbdma.h"
#include "rm_dma_hw_private.h"

#define NV_APB_DMA_REGR(rm,reg)                NV_REGR(rm, NvRmPrivModuleID_ApbDma, 0, APBDMA_##reg##_0)
#define NV_APB_DMA_REGW(rm,reg,data)           NV_REGW(rm, NvRmPrivModuleID_ApbDma, 0, APBDMA_##reg##_0, data)

NvU32 NvRmPrivDmaInterruptDecode(NvRmDeviceHandle hRmDevice )
{
    NvU32   Channel;
    NvU32   Reg;

    // Read the APB DMA channel interrupt status register.
    Reg = NV_APB_DMA_REGR(hRmDevice, IRQ_STA_CPU);

    // Get the interrupting channel number.
    Channel = 31 - CountLeadingZeros(Reg);

    // Get the interrupt disable mask.
    Reg = 1 << Channel;

    // Disable the source.
    NV_APB_DMA_REGW(hRmDevice, IRQ_MASK_CLR, Reg);

    return Channel;
}

void NvRmPrivDmaInterruptEnable(NvRmDeviceHandle hRmDevice, NvU32 Channel, NvBool Enable )
{
    NvU32   Reg;

    // Generate the channel mask.
    Reg = 1 << Channel;

    if (Enable)
    {
        // Enable the channel interrupt.
        NV_APB_DMA_REGW(hRmDevice, IRQ_MASK_SET, Reg);
    }
    else
    {
        // Disable the channel interrupt.
        NV_APB_DMA_REGW(hRmDevice, IRQ_MASK_CLR, Reg);
    }
}

