/*
 * arch/arm/mach-tegra/nvrm/io/common/nvrm_dma.c
 *
 * RM DMA APIs implemented on system DMA driver
 *
 * Copyright (c) 2008-2009, NVIDIA Corporation.
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

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/slab.h>

#include <mach/dma.h>
#include <mach/io.h>
#include <nvcommon.h>
#include <nverror.h>
#include <nvrm_dma.h>

struct dma_action {
	struct tegra_dma_req req;
	struct work_struct work;
	struct list_head node;
	NvRmDmaHandle dma;
	union {
		NvOsSemaphoreHandle dma_sem;
		struct completion *dma_done;
	};
};

typedef struct NvRmDmaRec
{
	struct tegra_dma_channel *ch;
	struct list_head req_list;
	unsigned int last_xfer;
	spinlock_t lock;
	int mod_sel;
	int mod_width;
} NvRmDma;

static void dma_complete_work(struct work_struct *work)
{
	struct dma_action *action = container_of(work, struct dma_action, work);

	spin_lock(&action->dma->lock);
	list_del(&action->node);

	if (action->dma_sem) {
		if (action->req.status==TEGRA_DMA_REQ_SUCCESS)
			NvOsSemaphoreSignal(action->dma_sem);
		NvOsSemaphoreDestroy(action->dma_sem);
	}
	spin_unlock(&action->dma->lock);
	kfree(action);
}

static void dma_complete_async(struct tegra_dma_req *req)
{
	struct dma_action *action = container_of(req, struct dma_action, req);

	action->dma->last_xfer = req->bytes_transferred;
	schedule_work(&action->work);
}

static void dma_complete_sync(struct tegra_dma_req *req)
{
	struct dma_action *action = container_of(req, struct dma_action, req);

	action->dma->last_xfer = req->bytes_transferred;
	complete(action->dma_done);
}

#define match(_mod, _id) (req==NvRmDmaModuleID_##_mod && id==(_id))

static int dma_req_sel(NvRmDmaModuleID req, NvU32 id, int *width)
{
	if (match(I2s,0)) {
		*width = 32;
		return TEGRA_DMA_REQ_SEL_I2S_1;
	} else if (match(I2s,1)) {
		*width = 32;
		return TEGRA_DMA_REQ_SEL_I2S2_1;
	} else if (match(I2c, 0)) {
		*width = 32;
		return TEGRA_DMA_REQ_SEL_I2C;
	} else if (match(I2c, 1)) {
		*width = 32;
		return TEGRA_DMA_REQ_SEL_I2C2;
	} else if (match(I2c, 2)) {
		*width = 32;
		return TEGRA_DMA_REQ_SEL_I2C3;
	} else if (match(Spdif, 0)) {
		*width = 32;
		return TEGRA_DMA_REQ_SEL_SPD_I;
	} else if (match(Slink, 0)) {
		*width = 32;
		return TEGRA_DMA_REQ_SEL_SL2B1;
	} else if (match(Slink, 1)) {
		*width = 32;
		return TEGRA_DMA_REQ_SEL_SL2B2;
	} else if (match(Slink, 2)) {
		*width = 32;
		return TEGRA_DMA_REQ_SEL_SL2B3;
	} else if (match(Slink, 3)) {
		*width = 32;
		return TEGRA_DMA_REQ_SEL_SL2B4;
	} else if (match(Spi, 0)) {
		*width = 32;
		return TEGRA_DMA_REQ_SEL_SPI;
	}

	*width = 0;
	return TEGRA_DMA_REQ_SEL_INVALID;
}

void NvRmDmaFree(NvRmDmaHandle dma)
{
	if (!dma)
		return;

	NvRmDmaAbort(dma);

	if (dma->ch)
		tegra_dma_free_channel(dma->ch);

	kfree(dma);
}

NvError NvRmDmaAllocate(NvRmDeviceHandle rm, NvRmDmaHandle *rm_dma,
	NvBool swap, NvRmDmaPriority priority,
	NvRmDmaModuleID requester, NvU32 instance)
{
	NvRmDma *dma = kzalloc(sizeof(*dma), GFP_KERNEL);
	NvError e;
	int mode;

	if (!dma) {
		e = NvError_InsufficientMemory;
		goto fail;
	}

	if (!rm_dma) {
		e = NvError_BadParameter;
		goto fail;
	}

	dma->mod_sel = dma_req_sel(requester, instance, &dma->mod_width);
	if (dma->mod_sel == TEGRA_DMA_REQ_SEL_INVALID) {
		e = NvError_BadValue;
		goto fail;
	}

	if (priority == NvRmDmaPriority_Low)
		mode = TEGRA_DMA_SHARED | TEGRA_DMA_MODE_ONESHOT;
	else if (requester==NvRmDmaModuleID_I2s ||
		 requester==NvRmDmaModuleID_Spdif)
		mode = TEGRA_DMA_MODE_CONTINUOUS;
	else
		mode = TEGRA_DMA_MODE_ONESHOT;

	dma->ch = tegra_dma_allocate_channel(mode);
	if (IS_ERR_OR_NULL(dma->ch)) {
		e = NvError_DmaChannelNotAvailable;
		goto fail;
	}
	INIT_LIST_HEAD(&dma->req_list);
	spin_lock_init(&dma->lock);

	*rm_dma = (NvRmDmaHandle)dma;

	return NvSuccess;
fail:
	if (dma) {
		if (!IS_ERR_OR_NULL(dma->ch))
			tegra_dma_free_channel(dma->ch);
		kfree(dma);
	}
	if (rm_dma)
		*rm_dma = NULL;
	return e;
}

NvError NvRmDmaStartDmaTransfer(NvRmDmaHandle dma, NvRmDmaClientBuffer *b,
	NvRmDmaDirection dir, NvU32 timeout, NvOsSemaphoreHandle wakeup)
{
	bool periph_src, periph_dst;
	unsigned long src, dst;
	unsigned long src_width, dst_width, periph_width;
	unsigned long src_wrap, dst_wrap;
	struct dma_action *action;
	NvError e = NvSuccess;
	DECLARE_COMPLETION_ONSTACK(dma_done);

	if ((b->SourceBufferPhyAddress | b->DestinationBufferPhyAddress) & 3) {
		pr_debug("%s: invalid address\n", __func__);
		return NvError_InvalidAddress;
	}

	if (!b->TransferSize || (b->TransferSize & 3)) {
		pr_debug("%s: invalid size\n", __func__);
		return NvError_InvalidSize;
	}

	src = b->SourceBufferPhyAddress;
	dst = b->DestinationBufferPhyAddress;

	src_wrap = b->SourceAddressWrapSize & 0xFFFF;
	dst_wrap = b->DestinationAddressWrapSize & 0xFFFF;

	src_width = (b->SourceAddressWrapSize >> 16) & 0xFFFF;
	dst_width = (b->DestinationAddressWrapSize >> 16) & 0xFFFF;

	WARN_ON_ONCE((src < PAGE_SIZE) || (dst < PAGE_SIZE));

	periph_src = (src - IO_APB_PHYS < IO_APB_SIZE);
	periph_dst = (dst - IO_APB_PHYS < IO_APB_SIZE);
	if (!(periph_src ^ periph_dst)) {
		pr_debug("%s: not supported\n", __func__);
		return NvError_NotSupported;
	}

	action = kzalloc(sizeof(*action), GFP_KERNEL);
	if (!action) {
		pr_debug("%s: insufficient memory\n", __func__);
		return NvError_InsufficientMemory;
	}

	action->req.size = b->TransferSize;
	action->req.req_sel = dma->mod_sel;
	action->req.threshold = NULL;
	action->dma = dma;

	if (periph_src && src_width)
		periph_width = src_width*8;
	else if (periph_dst && dst_width)
		periph_width = dst_width*8;
	else
		periph_width = dma->mod_width;

	if ((periph_src && dir==NvRmDmaDirection_Forward) ||
	    (periph_dst && dir==NvRmDmaDirection_Reverse)) {
		action->req.to_memory = 1;
		action->req.dest_bus_width = 32;
		action->req.source_bus_width = periph_width;
	} else {
		action->req.to_memory = 0;
		action->req.dest_bus_width = periph_width;
		action->req.source_bus_width = 32;
	}

	if (dir==NvRmDmaDirection_Forward) {
		action->req.dest_addr = dst;
		action->req.source_addr = src;
		action->req.dest_wrap = dst_wrap;
		action->req.source_wrap = src_wrap;
	} else {
		action->req.dest_addr = src;
		action->req.source_addr = dst;
		action->req.dest_wrap = src_wrap;
		action->req.source_wrap = dst_wrap;
	}

	action->dma_sem = NULL;
	if (timeout) {
		action->dma_done = &dma_done;
		action->req.complete = dma_complete_sync;
	} else if (wakeup) {
		NvError e;
		e = NvOsSemaphoreClone(wakeup,
				       (NvOsSemaphoreHandle *)&action->dma_sem);
		if (e != NvSuccess) {
			kfree(action);
			pr_debug("%s: SemaphoreClone returned 0x%x\n",
			       __func__, e);
			return e;
		}

		INIT_WORK(&action->work, dma_complete_work);
		action->req.complete = dma_complete_async;
	} else {
		action->dma_sem = NULL;
		INIT_WORK(&action->work, dma_complete_work);
		action->req.complete = dma_complete_async;
	}

	spin_lock(&dma->lock);
	list_add_tail(&action->node, &dma->req_list);

	if (tegra_dma_enqueue_req(dma->ch, &action->req)) {
		list_del(&action->node);
		spin_unlock(&dma->lock);
		pr_debug("%s: failed to enqueue DMA request\n", __func__);
		if (action->dma_sem)
			NvOsSemaphoreDestroy(action->dma_sem);
		return NvError_BadParameter;
	}

	if (timeout) {
		if (!wait_for_completion_timeout(&dma_done,
						 msecs_to_jiffies(timeout))) {
			pr_debug("%s: DMA didn't complete in %u ms\n",
			       __func__, timeout);
			tegra_dma_dequeue_req(dma->ch, &action->req);
			e = NvError_Timeout;
		}
		list_del(&action->node);
		kfree(action);
	}
	spin_unlock(&dma->lock);

	return e;
}


void NvRmDmaAbort(NvRmDmaHandle dma)
{
	if (!dma)
		return;

	tegra_dma_flush(dma->ch);

	spin_lock(&dma->lock);
	while (!list_empty(&dma->req_list)) {
		struct dma_action *action;

		action = list_first_entry(&dma->req_list,
					  struct dma_action, node);

		if (action->dma_sem)
			NvOsSemaphoreDestroy(action->dma_sem);

		list_del(&action->node);
		kfree(action);
	}
	spin_unlock(&dma->lock);
}



NvError NvRmDmaGetCapabilities(NvRmDeviceHandle rm, NvRmDmaCapabilities *caps)
{
	if (!caps)
		return NvError_BadParameter;

	caps->DmaAddressAlignmentSize = 4;
	caps->DmaGranularitySize = 4;
	return NvSuccess;
}

NvError NvRmDmaGetTransferredCount(NvRmDmaHandle dma, NvU32 *count, NvBool stop)
{
	struct dma_action *action;
	struct tegra_dma_req *req;

	if (!dma || !count)
		return NvError_BadParameter;

	spin_lock(&dma->lock);
	if (list_empty(&dma->req_list)) {
		*count = dma->last_xfer;
		spin_unlock(&dma->lock);
		return NvSuccess;
	}
	action = list_first_entry(&dma->req_list, struct dma_action, node);
	req = &action->req;

	*count = tegra_dma_transferred_req(dma->ch, req);
	spin_unlock(&dma->lock);

	if (stop)
		tegra_dma_dequeue_req(dma->ch, req);

	return NvSuccess;
}

NvBool NvRmDmaIsDmaTransferCompletes(NvRmDmaHandle dma, NvBool first)
{
	BUG();
	return NV_FALSE;
}

NvError NvRmPrivDmaSuspend(void)
{
	return NvSuccess;
}

NvError NvRmPrivDmaResume(void)
{
	return NvSuccess;
}

NvError NvRmPrivDmaInit(NvRmDeviceHandle rm)
{
	return NvSuccess;
}

void NvRmPrivDmaDeInit(void)
{
}
