/*
 * arch/arm/mach-tegra/dma.c
 *
 * System DMA driver for NVIDIA Tegra SoCs
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

/*#define DEBUG 1*/

#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <mach/dma.h>
#include <mach/irqs.h>
#include <mach/iomap.h>

#define APB_DMA_GEN				0x000
#define GEN_ENABLE				(1<<31)

#define APB_DMA_CNTRL				0x010

#define APB_DMA_IRQ_MASK			0x01c

#define APB_DMA_IRQ_MASK_SET			0x020

#define APB_DMA_CHAN_CSR			0x000
#define CSR_ENB					(1<<31)
#define CSR_IE_EOC				(1<<30)
#define CSR_HOLD				(1<<29)
#define CSR_DIR					(1<<28)
#define CSR_ONCE				(1<<27)
#define CSR_FLOW				(1<<21)
#define CSR_REQ_SEL_SHIFT			16
#define CSR_REQ_SEL_MASK			(0x1F<<CSR_REQ_SEL_SHIFT)
#define CSR_WCOUNT_SHIFT			2
#define CSR_WCOUNT_MASK				0xFFFC

#define APB_DMA_CHAN_STA				0x004
#define STA_BUSY				(1<<31)
#define STA_ISE_EOC				(1<<30)
#define STA_HALT				(1<<29)
#define STA_PING_PONG				(1<<28)
#define STA_COUNT_SHIFT				2
#define STA_COUNT_MASK				0xFFFC

#define APB_DMA_CHAN_AHB_PTR				0x010

#define APB_DMA_CHAN_AHB_SEQ				0x014
#define AHB_SEQ_INTR_ENB			(1<<31)
#define AHB_SEQ_BUS_WIDTH_SHIFT			28
#define AHB_SEQ_BUS_WIDTH_MASK			(0x7<<AHB_SEQ_BUS_WIDTH_SHIFT)
#define AHB_SEQ_BUS_WIDTH_8			(0<<AHB_SEQ_BUS_WIDTH_SHIFT)
#define AHB_SEQ_BUS_WIDTH_16			(1<<AHB_SEQ_BUS_WIDTH_SHIFT)
#define AHB_SEQ_BUS_WIDTH_32			(2<<AHB_SEQ_BUS_WIDTH_SHIFT)
#define AHB_SEQ_BUS_WIDTH_64			(3<<AHB_SEQ_BUS_WIDTH_SHIFT)
#define AHB_SEQ_BUS_WIDTH_128			(4<<AHB_SEQ_BUS_WIDTH_SHIFT)
#define AHB_SEQ_DATA_SWAP			(1<<27)
#define AHB_SEQ_BURST_MASK			(0x7<<24)
#define AHB_SEQ_BURST_1				(4<<24)
#define AHB_SEQ_BURST_4				(5<<24)
#define AHB_SEQ_BURST_8				(6<<24)
#define AHB_SEQ_DBL_BUF				(1<<19)
#define AHB_SEQ_WRAP_SHIFT			16
#define AHB_SEQ_WRAP_MASK			(0x7<<AHB_SEQ_WRAP_SHIFT)

#define APB_DMA_CHAN_APB_PTR				0x018

#define APB_DMA_CHAN_APB_SEQ				0x01c
#define APB_SEQ_BUS_WIDTH_SHIFT			28
#define APB_SEQ_BUS_WIDTH_MASK			(0x7<<APB_SEQ_BUS_WIDTH_SHIFT)
#define APB_SEQ_BUS_WIDTH_8			(0<<APB_SEQ_BUS_WIDTH_SHIFT)
#define APB_SEQ_BUS_WIDTH_16			(1<<APB_SEQ_BUS_WIDTH_SHIFT)
#define APB_SEQ_BUS_WIDTH_32			(2<<APB_SEQ_BUS_WIDTH_SHIFT)
#define APB_SEQ_BUS_WIDTH_64			(3<<APB_SEQ_BUS_WIDTH_SHIFT)
#define APB_SEQ_BUS_WIDTH_128			(4<<APB_SEQ_BUS_WIDTH_SHIFT)
#define APB_SEQ_DATA_SWAP			(1<<27)
#define APB_SEQ_WRAP_SHIFT			16
#define APB_SEQ_WRAP_MASK			(0x7<<APB_SEQ_WRAP_SHIFT)

#define TEGRA_SYSTEM_DMA_CH_NR			16
#define TEGRA_SYSTEM_DMA_AVP_CH_NUM		4
#define TEGRA_SYSTEM_DMA_CH_MIN			0
#define TEGRA_SYSTEM_DMA_CH_MAX	\
	(TEGRA_SYSTEM_DMA_CH_NR - TEGRA_SYSTEM_DMA_AVP_CH_NUM - 1)

#define NV_DMA_MAX_TRASFER_SIZE 0x10000

const unsigned int ahb_addr_wrap_table[8] = {
	0, 32, 64, 128, 256, 512, 1024, 2048
};

const unsigned int apb_addr_wrap_table[8] = {0, 1, 2, 4, 8, 16, 32, 64};

const unsigned int bus_width_table[5] = {8, 16, 32, 64, 128};

#define TEGRA_DMA_NAME_SIZE 16
struct tegra_dma_channel {
	struct list_head	list;
	int			id;
	spinlock_t		lock;
	char			name[TEGRA_DMA_NAME_SIZE];
	void  __iomem		*addr;
	int			mode;
	int			irq;
	bool			is_int_service_due;

	/* Register shadow */
	u32			csr;
	u32			ahb_seq;
	u32			ahb_ptr;
	u32			apb_seq;
	u32			apb_ptr;
};

#define  NV_DMA_MAX_CHANNELS  32

static DEFINE_SPINLOCK(global_dma_lock);

static DECLARE_BITMAP(channel_usage, NV_DMA_MAX_CHANNELS);
static struct tegra_dma_channel dma_channels[NV_DMA_MAX_CHANNELS];

static void tegra_dma_update_hw(struct tegra_dma_channel *ch,
	struct tegra_dma_req *req);
static void tegra_dma_update_hw_partial(struct tegra_dma_channel *ch,
	struct tegra_dma_req *req);
static void tegra_dma_init_hw(struct tegra_dma_channel *ch);


void tegra_dma_dequeue(struct tegra_dma_channel *ch)
{
	struct tegra_dma_req *req;

	if (IS_ERR_OR_NULL(ch))
		BUG();

	if (tegra_dma_is_empty(ch))
		return;

	req = list_entry(ch->list.next, typeof(*req), node);

	tegra_dma_dequeue_req(ch, req);
	return;
}

void tegra_dma_stop(struct tegra_dma_channel *ch)
{
	unsigned int csr;
	unsigned int status;

	if (IS_ERR_OR_NULL(ch))
		BUG();

	csr = ch->csr;
	csr &= ~CSR_IE_EOC;
	writel(csr, ch->addr + APB_DMA_CHAN_CSR);

	csr &= ~CSR_ENB;
	writel(csr, ch->addr + APB_DMA_CHAN_CSR);

	status = readl(ch->addr + APB_DMA_CHAN_STA);
	if (status & STA_ISE_EOC)
		writel(status, ch->addr + APB_DMA_CHAN_STA);
}


int tegra_dma_cancel(struct tegra_dma_channel *ch)
{
	unsigned long irq_flags;

	if (IS_ERR_OR_NULL(ch))
		BUG();

	spin_lock_irqsave(&ch->lock, irq_flags);
	while (!list_empty(&ch->list))
		list_del(ch->list.next);

	tegra_dma_stop(ch);

	spin_unlock_irqrestore(&ch->lock, irq_flags);
	return 0;
}

void tegra_dma_flush(struct tegra_dma_channel *ch)
{
	tegra_dma_cancel(ch);
}
EXPORT_SYMBOL(tegra_dma_flush);

/* should be called with the channel lock held */
static unsigned int dma_active_count(struct tegra_dma_channel *ch,
	struct tegra_dma_req *req, unsigned int status)
{
	unsigned int to_transfer;
	unsigned int req_transfer_count;

	unsigned int bytes_transferred;

	to_transfer = (status & STA_COUNT_MASK) >> STA_COUNT_SHIFT;
	req_transfer_count = (ch->csr & CSR_WCOUNT_MASK) >> CSR_WCOUNT_SHIFT;
	req_transfer_count += 1;
	to_transfer += 1;

	bytes_transferred = req_transfer_count;

	if (status & STA_BUSY)
		bytes_transferred -= to_transfer;

	/* In continuous transfer mode, DMA only tracks the count of the
	 * half DMA buffer. So, if the DMA already finished half the DMA
	 * then add the half buffer to the completed count.
	 */
	if (!(ch->mode & TEGRA_DMA_MODE_CONTINUOUS))
		goto skip_handle_cont;

	if (!((status & STA_ISE_EOC) || (ch->is_int_service_due)))
		goto skip_irq_handle;

	pr_debug("Dma transfer adjusted out of isr is_int_service_due %d\n",
						ch->is_int_service_due);
	if (req->is_repeat_req) {
		if (req->buffer_status == TEGRA_DMA_REQ_BUF_STATUS_EMPTY) {
			pr_debug("Int hit when buffer status empty\n");
			req->bytes_transferred = req_transfer_count << 2;
			req->buffer_status = TEGRA_DMA_REQ_BUF_STATUS_HALF_FULL;
		} else if (req->buffer_status ==
					TEGRA_DMA_REQ_BUF_STATUS_HALF_FULL) {
			pr_debug("Int hit when buffer status half full\n");
			pr_debug("The curr byte transferred 0x%x status 0x%x\n",
					 bytes_transferred, status);
			req->bytes_transferred = 0;
			req->buffer_status = TEGRA_DMA_REQ_BUF_STATUS_EMPTY;
		} else {
			BUG();
		}
		ch->is_int_service_due = false;
	}
skip_irq_handle:
	bytes_transferred += req->bytes_transferred >> 2;
skip_handle_cont:
	bytes_transferred *= 4;
	return bytes_transferred;
}

static unsigned int get_channel_status(struct tegra_dma_channel *ch,
			struct tegra_dma_req *req, bool is_stop_dma)
{
	static DEFINE_SPINLOCK(global_dma_bit_access_lock);
	void __iomem *addr = IO_ADDRESS(TEGRA_APB_DMA_BASE);
	unsigned long g_irq_flags;
	unsigned int status;

	if (is_stop_dma) {
		/* STOP the DMA and get the transfer count.
		 * Getting the transfer count is tricky.
		 *  - Globally disable DMA on all channels
		 *  - Read the channel's status register to know the number of pending
		 *    bytes to be transfered.
		 *  - Stop the dma channel
		 *  - Globally re-enable DMA to resume other transfers
		 */
		spin_lock_irqsave(&global_dma_bit_access_lock, g_irq_flags);
		writel(0, addr + APB_DMA_GEN);
		status = readl(ch->addr + APB_DMA_CHAN_STA);
		tegra_dma_stop(ch);
		writel(GEN_ENABLE, addr + APB_DMA_GEN);
		spin_unlock_irqrestore(&global_dma_bit_access_lock, g_irq_flags);

		if (status & STA_ISE_EOC) {
			pr_err("Got Dma Int here clearing");
			writel(status, ch->addr + APB_DMA_CHAN_STA);
		}
		req->status = TEGRA_DMA_REQ_STOPPED;
	} else {
		status = readl(ch->addr + APB_DMA_CHAN_STA);
		if (status & STA_ISE_EOC) {
			pr_err("Got Dma Int here clearing");
			writel(status, ch->addr + APB_DMA_CHAN_STA);
		}
	}
	return status;
}

unsigned int tegra_dma_transferred_req(struct tegra_dma_channel *ch,
		struct tegra_dma_req *req)
{
	unsigned long irq_flags;
	unsigned int bytes_transferred;
	unsigned int status;

	if (IS_ERR_OR_NULL(ch))
		BUG();

	spin_lock_irqsave(&ch->lock, irq_flags);

	if (list_entry(ch->list.next, struct tegra_dma_req, node)!=req) {
		spin_unlock_irqrestore(&ch->lock, irq_flags);
		return req->bytes_transferred;
	}

	status = readl(ch->addr + APB_DMA_CHAN_STA);
	bytes_transferred = dma_active_count(ch, req, status);

	spin_unlock_irqrestore(&ch->lock, irq_flags);

	return bytes_transferred;
}
EXPORT_SYMBOL(tegra_dma_transferred_req);

int tegra_dma_get_transfer_count(struct tegra_dma_channel *ch,
			struct tegra_dma_req *req, bool is_stop_dma)
{
	unsigned int status;
	unsigned long irq_flags;
	int bytes_transferred = 0;

	if (IS_ERR_OR_NULL(ch))
		BUG();

	spin_lock_irqsave(&ch->lock, irq_flags);

	if (list_entry(ch->list.next, struct tegra_dma_req, node) != req) {
		spin_unlock_irqrestore(&ch->lock, irq_flags);
		pr_err("The dma request is not running\n");
		return -1;
	}

	if (req->status != TEGRA_DMA_REQ_INFLIGHT) {
		spin_unlock_irqrestore(&ch->lock, irq_flags);
		pr_err("The dma request is not running\n");
		return -1;
	}
	status = get_channel_status(ch, req, is_stop_dma);
	bytes_transferred = dma_active_count(ch, req, status);
	spin_unlock_irqrestore(&ch->lock, irq_flags);
	return bytes_transferred;
}
EXPORT_SYMBOL(tegra_dma_get_transfer_count);

int tegra_dma_start_dma(struct tegra_dma_channel *ch, struct tegra_dma_req *req)
{
	unsigned int csr;
	unsigned int status;
	unsigned long irq_flags;

	if (IS_ERR_OR_NULL(ch))
		BUG();

	if (IS_ERR_OR_NULL(req))
		BUG();

	spin_lock_irqsave(&ch->lock, irq_flags);

	if (list_entry(ch->list.next, struct tegra_dma_req, node)!=req) {
		spin_unlock_irqrestore(&ch->lock, irq_flags);
		pr_err("The dma request is not on top request\n");
		return -1;
	}

	if (req->status != TEGRA_DMA_REQ_STOPPED) {
		spin_unlock_irqrestore(&ch->lock, irq_flags);
		pr_err("Dma state is not in STOPPED\n");
		return -1;
	}

	/* clear interrupt if already there */
	status = readl(ch->addr + APB_DMA_CHAN_STA);
	if (status & STA_ISE_EOC)
		writel(status, ch->addr + APB_DMA_CHAN_STA);

	csr = ch->csr;
	csr |= CSR_IE_EOC;
	writel(csr, ch->addr + APB_DMA_CHAN_CSR);
	ch->csr = csr;

	req->bytes_transferred = 0;
	req->status = TEGRA_DMA_REQ_INFLIGHT;
	req->buffer_status = TEGRA_DMA_REQ_BUF_STATUS_EMPTY;

	csr |= CSR_ENB;
	writel(csr, ch->addr + APB_DMA_CHAN_CSR);
	spin_unlock_irqrestore(&ch->lock, irq_flags);
	return 0;
}
EXPORT_SYMBOL(tegra_dma_start_dma);

int tegra_dma_dequeue_req(struct tegra_dma_channel *ch,
	struct tegra_dma_req *_req)
{
	struct tegra_dma_req *req = NULL;
	int found = 0;
	unsigned int status;
	unsigned long irq_flags;
	int stop = 0;

	if (IS_ERR_OR_NULL(ch))
		BUG();

	spin_lock_irqsave(&ch->lock, irq_flags);

	if (list_entry(ch->list.next, struct tegra_dma_req, node)==_req)
		stop = 1;

	list_for_each_entry(req, &ch->list, node) {
		if (req == _req) {
			list_del(&req->node);
			found = 1;
			break;
		}
	}
	if (!found) {
		spin_unlock_irqrestore(&ch->lock, irq_flags);
		return 0;
	}

	if (!stop)
		goto skip_status;

	status = get_channel_status(ch, req, true);
	req->bytes_transferred = dma_active_count(ch, req, status);

	if (!list_empty(&ch->list)) {
		/* if the list is not empty, queue the next request */
		struct tegra_dma_req *next_req;
		next_req = list_entry(ch->list.next,
			typeof(*next_req), node);
		tegra_dma_update_hw(ch, next_req);
	}
skip_status:
	req->status = -TEGRA_DMA_REQ_ERROR_ABORTED;

	spin_unlock_irqrestore(&ch->lock, irq_flags);

	/* Callback should be called without any lock */
	req->complete(req);
	return 0;
}
EXPORT_SYMBOL(tegra_dma_dequeue_req);

bool tegra_dma_is_empty(struct tegra_dma_channel *ch)
{
	unsigned long irq_flags;
	bool is_empty;

	if (IS_ERR_OR_NULL(ch))
		BUG();

	spin_lock_irqsave(&ch->lock, irq_flags);
	if (list_empty(&ch->list))
		is_empty = true;
	else
		is_empty = false;
	spin_unlock_irqrestore(&ch->lock, irq_flags);
	return is_empty;
}
EXPORT_SYMBOL(tegra_dma_is_empty);

bool tegra_dma_is_req_inflight(struct tegra_dma_channel *ch,
	struct tegra_dma_req *_req)
{
	unsigned long irq_flags;
	struct tegra_dma_req *req;

	if (IS_ERR_OR_NULL(ch))
		BUG();

	spin_lock_irqsave(&ch->lock, irq_flags);
	list_for_each_entry(req, &ch->list, node) {
		if (req == _req) {
			spin_unlock_irqrestore(&ch->lock, irq_flags);
			return true;
		}
	}
	spin_unlock_irqrestore(&ch->lock, irq_flags);
	return false;
}
EXPORT_SYMBOL(tegra_dma_is_req_inflight);

int tegra_dma_enqueue_req(struct tegra_dma_channel *ch,
	struct tegra_dma_req *req)
{
	unsigned long irq_flags;
	int start_dma = 0;

	if (IS_ERR_OR_NULL(ch))
		BUG();

	if (IS_ERR_OR_NULL(req))
		BUG();

	if (req->size > NV_DMA_MAX_TRASFER_SIZE ||
		req->source_addr & 0x3 || req->dest_addr & 0x3) {
		pr_err("Invalid DMA request for channel %d\n", ch->id);
		return -EINVAL;
	}

	spin_lock_irqsave(&ch->lock, irq_flags);

	req->bytes_transferred = 0;
	req->status = 0;
	req->buffer_status = 0;
	if (list_empty(&ch->list))
		start_dma = 1;

	list_add_tail(&req->node, &ch->list);

	if (start_dma)
		tegra_dma_update_hw(ch, req);

	spin_unlock_irqrestore(&ch->lock, irq_flags);

	return 0;
}
EXPORT_SYMBOL(tegra_dma_enqueue_req);

struct tegra_dma_channel *tegra_dma_allocate_channel(int mode)
{
	int channel;
	struct tegra_dma_channel *ch;
	unsigned long irq_flags;

	spin_lock_irqsave(&global_dma_lock, irq_flags);

	/* first channel is the shared channel */
	if (mode & TEGRA_DMA_SHARED) {
		channel = TEGRA_SYSTEM_DMA_CH_MIN;
	} else {
		channel = find_first_zero_bit(channel_usage,
			ARRAY_SIZE(dma_channels));
		if (channel >= ARRAY_SIZE(dma_channels)) {
			spin_unlock_irqrestore(&global_dma_lock, irq_flags);
			return ERR_PTR(-ENODEV);
		}
	}
	__set_bit(channel, channel_usage);
	spin_unlock_irqrestore(&global_dma_lock, irq_flags);

	ch = &dma_channels[channel];
	ch->mode = mode;
	ch->is_int_service_due = false;
	return ch;
}
EXPORT_SYMBOL(tegra_dma_allocate_channel);

void tegra_dma_free_channel(struct tegra_dma_channel *ch)
{
	unsigned long irq_flags;

	if (IS_ERR_OR_NULL(ch))
		BUG();

	if (ch->mode & TEGRA_DMA_SHARED)
		return;
	tegra_dma_cancel(ch);
	spin_lock_irqsave(&global_dma_lock, irq_flags);
	__clear_bit(ch->id, channel_usage);
	spin_unlock_irqrestore(&global_dma_lock, irq_flags);

}
EXPORT_SYMBOL(tegra_dma_free_channel);

static void tegra_dma_update_hw_partial(struct tegra_dma_channel *ch,
	struct tegra_dma_req *req)
{
	if (req->to_memory) {
		ch->apb_ptr = req->source_addr;
		ch->ahb_ptr = req->dest_addr;
	} else {
		ch->apb_ptr = req->dest_addr;
		ch->ahb_ptr = req->source_addr;
	}
	writel(ch->apb_ptr, ch->addr + APB_DMA_CHAN_APB_PTR);
	writel(ch->ahb_ptr, ch->addr + APB_DMA_CHAN_AHB_PTR);

	req->status = TEGRA_DMA_REQ_INFLIGHT;
	return;
}

static void set_burst_size(struct tegra_dma_channel *ch,
	struct tegra_dma_req *req)
{
	ch->ahb_seq &= ~AHB_SEQ_BURST_MASK;
	switch(req->req_sel) {
	case TEGRA_DMA_REQ_SEL_SL2B1:
	case TEGRA_DMA_REQ_SEL_SL2B2:
	case TEGRA_DMA_REQ_SEL_SL2B3:
	case TEGRA_DMA_REQ_SEL_SL2B4:
	case TEGRA_DMA_REQ_SEL_SPI:
		/* For spi/slink the burst size based on transfer size
		 * i.e. if multiple of 16 bytes then busrt is
		 * 4 word else burst size is 1 word */
		if (req->size & 0xF)
			ch->ahb_seq |= AHB_SEQ_BURST_1;
		else
			ch->ahb_seq |= AHB_SEQ_BURST_4;
		break;
	case TEGRA_DMA_REQ_SEL_I2S_2:
	case TEGRA_DMA_REQ_SEL_I2S_1:
	case TEGRA_DMA_REQ_SEL_I2S2_2:
	case TEGRA_DMA_REQ_SEL_I2S2_1:
		ch->ahb_seq |= AHB_SEQ_BURST_4;
		break;
	default:
		ch->ahb_seq |= AHB_SEQ_BURST_1;
		break;
	}
}
static void tegra_dma_update_hw(struct tegra_dma_channel *ch,
	struct tegra_dma_req *req)
{
	int ahb_addr_wrap;
	int apb_addr_wrap;
	int ahb_bus_width;
	int apb_bus_width;
	int index;
	unsigned long csr;

	ch->csr |= CSR_FLOW;
	ch->csr &= ~CSR_REQ_SEL_MASK;
	ch->csr |= req->req_sel << CSR_REQ_SEL_SHIFT;

	set_burst_size(ch, req);

	/* One shot mode is always single buffered,
	 * continuous mode is always double buffered
	 * */
	if (ch->mode & TEGRA_DMA_MODE_ONESHOT) {
		ch->csr |= CSR_ONCE;
		ch->ahb_seq &= ~AHB_SEQ_DBL_BUF;
		ch->csr &= ~CSR_WCOUNT_MASK;
		ch->csr |= ((req->size>>2) - 1) << CSR_WCOUNT_SHIFT;
	} else {
		ch->csr &= ~CSR_ONCE;
		ch->ahb_seq |= AHB_SEQ_DBL_BUF;

		/* In double buffered mode, we set the size to half the
		 * requested size and interrupt when half the buffer
		 * is full */
		ch->csr &= ~CSR_WCOUNT_MASK;
		ch->csr |= ((req->size>>3) - 1) << CSR_WCOUNT_SHIFT;
	}

	if (req->to_memory) {
		ch->csr &= ~CSR_DIR;
		ch->apb_ptr = req->source_addr;
		ch->ahb_ptr = req->dest_addr;

		apb_addr_wrap = req->source_wrap;
		ahb_addr_wrap = req->dest_wrap;
		apb_bus_width = req->source_bus_width;
		ahb_bus_width = req->dest_bus_width;

	} else {
		ch->csr |= CSR_DIR;
		ch->apb_ptr = req->dest_addr;
		ch->ahb_ptr = req->source_addr;

		apb_addr_wrap = req->dest_wrap;
		ahb_addr_wrap = req->source_wrap;
		apb_bus_width = req->dest_bus_width;
		ahb_bus_width = req->source_bus_width;
	}

	apb_addr_wrap >>= 2;
	ahb_addr_wrap >>= 2;

	/* set address wrap for APB size */
	index = 0;
	do  {
		if (apb_addr_wrap_table[index] == apb_addr_wrap)
			break;
		index++;
	} while (index < ARRAY_SIZE(apb_addr_wrap_table));
	BUG_ON(index == ARRAY_SIZE(apb_addr_wrap_table));
	ch->apb_seq &= ~APB_SEQ_WRAP_MASK;
	ch->apb_seq |= index << APB_SEQ_WRAP_SHIFT;

	/* set address wrap for AHB size */
	index = 0;
	do  {
		if (ahb_addr_wrap_table[index] == ahb_addr_wrap)
			break;
		index++;
	} while (index < ARRAY_SIZE(ahb_addr_wrap_table));
	BUG_ON(index == ARRAY_SIZE(ahb_addr_wrap_table));
	ch->ahb_seq &= ~AHB_SEQ_WRAP_MASK;
	ch->ahb_seq |= index << AHB_SEQ_WRAP_SHIFT;

	for (index = 0; index < ARRAY_SIZE(bus_width_table); index++) {
		if (bus_width_table[index] == ahb_bus_width)
			break;
	}
	BUG_ON(index == ARRAY_SIZE(bus_width_table));
	ch->ahb_seq &= ~AHB_SEQ_BUS_WIDTH_MASK;
	ch->ahb_seq |= index << AHB_SEQ_BUS_WIDTH_SHIFT;

	for (index = 0; index < ARRAY_SIZE(bus_width_table); index++) {
		if (bus_width_table[index] == apb_bus_width)
			break;
	}
	BUG_ON(index == ARRAY_SIZE(bus_width_table));
	ch->apb_seq &= ~APB_SEQ_BUS_WIDTH_MASK;
	ch->apb_seq |= index << APB_SEQ_BUS_WIDTH_SHIFT;

	ch->csr |= CSR_IE_EOC;

	/* update hw registers with the shadow */
	writel(ch->csr, ch->addr + APB_DMA_CHAN_CSR);
	writel(ch->apb_seq, ch->addr + APB_DMA_CHAN_APB_SEQ);
	writel(ch->apb_ptr, ch->addr + APB_DMA_CHAN_APB_PTR);
	writel(ch->ahb_seq, ch->addr + APB_DMA_CHAN_AHB_SEQ);
	writel(ch->ahb_ptr, ch->addr + APB_DMA_CHAN_AHB_PTR);

	csr = ch->csr | CSR_ENB;
	writel(csr, ch->addr + APB_DMA_CHAN_CSR);

	req->status = TEGRA_DMA_REQ_INFLIGHT;
}

static void tegra_dma_init_hw(struct tegra_dma_channel *ch)
{
	/* One shot with an interrupt to CPU after transfer */
	ch->csr = CSR_ONCE | CSR_IE_EOC;
	ch->ahb_seq = AHB_SEQ_BUS_WIDTH_32 | AHB_SEQ_INTR_ENB;
	ch->apb_seq = APB_SEQ_BUS_WIDTH_32 | 1 << APB_SEQ_WRAP_SHIFT;
}

static void handle_oneshot_dma(struct tegra_dma_channel *ch)
{
	struct tegra_dma_req *req;
	unsigned long irq_flags;

	spin_lock_irqsave(&ch->lock, irq_flags);
	if (list_empty(&ch->list)) {
		spin_unlock_irqrestore(&ch->lock, irq_flags);
		return;
	}

	req = list_entry(ch->list.next, typeof(*req), node);
	if (req) {
		int bytes_transferred;

		bytes_transferred =
			(ch->csr & CSR_WCOUNT_MASK) >> CSR_WCOUNT_SHIFT;
		bytes_transferred += 1;
		bytes_transferred <<= 2;

		list_del(&req->node);
		req->bytes_transferred = bytes_transferred;
		req->status = TEGRA_DMA_REQ_SUCCESS;

		spin_unlock_irqrestore(&ch->lock, irq_flags);
		/* Callback should be called without any lock */
		pr_debug("%s: transferred %d bytes\n", __func__,
			req->bytes_transferred);
		req->complete(req);
		spin_lock_irqsave(&ch->lock, irq_flags);
	}

	if (!list_empty(&ch->list)) {
		req = list_entry(ch->list.next, typeof(*req), node);
		/* the complete function we just called may have enqueued another
		   req, in which case dma has already started */
		if (req->status != TEGRA_DMA_REQ_INFLIGHT)
			tegra_dma_update_hw(ch, req);
	}
	spin_unlock_irqrestore(&ch->lock, irq_flags);
}

static void handle_continuous_dma(struct tegra_dma_channel *ch)
{
	struct tegra_dma_req *req;
	struct tegra_dma_req *next_req;
	unsigned long irq_flags;
	int bytes_transferred;

	spin_lock_irqsave(&ch->lock, irq_flags);
	if (list_empty(&ch->list)) {
		spin_unlock_irqrestore(&ch->lock, irq_flags);
		return;
	}
	if (!ch->is_int_service_due) {
		pr_debug("This int already served, ignoring...\n");
		spin_unlock_irqrestore(&ch->lock, irq_flags);
		return;
	}
	ch->is_int_service_due = false;

	req = list_entry(ch->list.next, typeof(*req), node);
	if (req) {
		if (req->buffer_status == TEGRA_DMA_REQ_BUF_STATUS_EMPTY) {
			bool is_dma_ping_complete;
			is_dma_ping_complete = (readl(ch->addr + APB_DMA_CHAN_STA)
						& STA_PING_PONG) ? true : false;
			if( req->to_memory )
				is_dma_ping_complete = !is_dma_ping_complete;
			/* Out of sync - Release current buffer */
			if( !is_dma_ping_complete ) {
				bytes_transferred =
					(ch->csr & CSR_WCOUNT_MASK) >>
							 CSR_WCOUNT_SHIFT;
				bytes_transferred += 1;
				bytes_transferred <<= 3;
				req->buffer_status = TEGRA_DMA_REQ_BUF_STATUS_FULL;
				req->bytes_transferred = bytes_transferred;
				req->status = TEGRA_DMA_REQ_SUCCESS;
				tegra_dma_stop(ch);

				if (!list_is_last(&req->node, &ch->list)) {
					struct tegra_dma_req *next_req;

					next_req = list_entry(req->node.next,
						typeof(*next_req), node);
					tegra_dma_update_hw(ch, next_req);
				}

				list_del(&req->node);

				/* DMA lock is NOT held when callbak is called */
				spin_unlock_irqrestore(&ch->lock, irq_flags);
				req->complete(req);
				return;
			}
			/* Load the next request into the hardware, if available
			 * */
			bytes_transferred =
				(ch->csr & CSR_WCOUNT_MASK) >> CSR_WCOUNT_SHIFT;
			bytes_transferred += 1;
			bytes_transferred <<= 2;
			req->bytes_transferred = bytes_transferred;
			if (!req->is_repeat_req) {
				if (!list_is_last(&req->node, &ch->list)) {
					next_req = list_entry(req->node.next,
						typeof(*next_req), node);
					tegra_dma_update_hw_partial(ch, next_req);
				}
				req->buffer_status = TEGRA_DMA_REQ_BUF_STATUS_HALF_FULL;
				req->status = TEGRA_DMA_REQ_SUCCESS;
			} else {
				req->buffer_status = TEGRA_DMA_REQ_BUF_STATUS_HALF_FULL;
				req->status = TEGRA_DMA_REQ_INFLIGHT;
			}
			/* DMA lock is NOT held when callback is called */
			spin_unlock_irqrestore(&ch->lock, irq_flags);
			if (likely(req->threshold))
				req->threshold(req);
			return;

		} else if (req->buffer_status ==
			TEGRA_DMA_REQ_BUF_STATUS_HALF_FULL) {
			/* Callback when the buffer is completely full (i.e on
			 * the second  interrupt */

			bytes_transferred =
				(ch->csr & CSR_WCOUNT_MASK) >> CSR_WCOUNT_SHIFT;
			bytes_transferred += 1;
			bytes_transferred <<= 2;

			req->buffer_status = TEGRA_DMA_REQ_BUF_STATUS_FULL;
			req->bytes_transferred += bytes_transferred;
			req->status = TEGRA_DMA_REQ_SUCCESS;
			if (!req->is_repeat_req) {
				if (list_is_last(&req->node, &ch->list)) {
					tegra_dma_stop(ch);
				} else {
					/* It may be possible that req came after
					 * half dma complete so it need to start
					 * immediately */
					next_req = list_entry(req->node.next,
							typeof(*next_req), node);
					if (next_req->status !=
							TEGRA_DMA_REQ_INFLIGHT) {
						tegra_dma_stop(ch);
						tegra_dma_update_hw(ch, next_req);
					}
				}
				list_del(&req->node);

				/* DMA lock is NOT held when callbak is called */
				spin_unlock_irqrestore(&ch->lock, irq_flags);
				req->complete(req);
			} else {
				req->bytes_transferred = 0;
				req->buffer_status = TEGRA_DMA_REQ_BUF_STATUS_EMPTY;
				req->status = TEGRA_DMA_REQ_INFLIGHT;
				spin_unlock_irqrestore(&ch->lock, irq_flags);
				if (likely(req->threshold))
					req->threshold(req);
			}
			return;

		} else {
			tegra_dma_stop(ch);
			/* Dma should be stop much earlier */
			BUG();
		}
	}
	spin_unlock_irqrestore(&ch->lock, irq_flags);
}

static irqreturn_t dma_isr(int irq, void *data)
{
	struct tegra_dma_channel *ch = data;
	unsigned long status;
	unsigned long irq_flags;
	unsigned long ret;

	spin_lock_irqsave(&ch->lock, irq_flags);
	status = readl(ch->addr + APB_DMA_CHAN_STA);
	if (status & STA_ISE_EOC) {
		writel(status, ch->addr + APB_DMA_CHAN_STA);
		ch->is_int_service_due = true;
		ret = IRQ_WAKE_THREAD;
	} else {
		pr_warning("Got a spurious ISR for DMA channel %d\n", ch->id);
		ret = IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&ch->lock, irq_flags);
	return ret;
}

static irqreturn_t dma_thread_fn(int irq, void *data)
{
	struct tegra_dma_channel *ch = data;

	if (ch->mode & TEGRA_DMA_MODE_ONESHOT)
		handle_oneshot_dma(ch);
	else
		handle_continuous_dma(ch);


	return IRQ_HANDLED;
}

int __init tegra_dma_init(void)
{
	int ret = 0;
	int i;
	unsigned int irq;
	void __iomem *addr;

	addr = IO_ADDRESS(TEGRA_APB_DMA_BASE);
	writel(GEN_ENABLE, addr + APB_DMA_GEN);
	writel(0, addr + APB_DMA_CNTRL);
	writel(0xFFFFFFFFul >> (31 - TEGRA_SYSTEM_DMA_CH_MAX),
		addr + APB_DMA_IRQ_MASK_SET);

	memset(channel_usage, 0, sizeof(channel_usage));
	memset(dma_channels, 0, sizeof(dma_channels));

	/* Reserve all the channels we are not supposed to touch */
	for (i = 0; i < TEGRA_SYSTEM_DMA_CH_MIN; i++)
		__set_bit(i, channel_usage);

	for (i = TEGRA_SYSTEM_DMA_CH_MIN; i <= TEGRA_SYSTEM_DMA_CH_MAX; i++) {
		struct tegra_dma_channel *ch = &dma_channels[i];

		__clear_bit(i, channel_usage);

		ch->id = i;
		snprintf(ch->name, TEGRA_DMA_NAME_SIZE, "dma_channel_%d", i);

		ch->addr = IO_ADDRESS(TEGRA_APB_DMA_CH0_BASE +
			TEGRA_APB_DMA_CH0_SIZE * i);

		spin_lock_init(&ch->lock);
		INIT_LIST_HEAD(&ch->list);
		tegra_dma_init_hw(ch);

		irq = INT_APB_DMA_CH0 + i;
		ret = request_threaded_irq(irq, dma_isr, dma_thread_fn, 0,
			dma_channels[i].name, ch);
		if (ret) {
			pr_err("Failed to register IRQ %d for DMA %d\n",
				irq, i);
			goto fail;
		}
		ch->irq = irq;
	}
	/* mark the shared channel allocated */
	__set_bit(TEGRA_SYSTEM_DMA_CH_MIN, channel_usage);

	for (i = TEGRA_SYSTEM_DMA_CH_MAX+1; i < NV_DMA_MAX_CHANNELS; i++)
		__set_bit(i, channel_usage);

	return ret;
fail:
	writel(0, addr + APB_DMA_GEN);
	for (i = TEGRA_SYSTEM_DMA_CH_MIN; i <= TEGRA_SYSTEM_DMA_CH_MAX; i++) {
		struct tegra_dma_channel *ch = &dma_channels[i];
		if (ch->irq)
			free_irq(ch->irq, ch);
	}
	return ret;
}

#ifdef CONFIG_PM
static u32 apb_dma[5*TEGRA_SYSTEM_DMA_CH_NR + 3];

void tegra_dma_suspend(void)
{
	void __iomem *addr = IO_ADDRESS(TEGRA_APB_DMA_BASE);
	u32 *ctx = apb_dma;
	int i;

	*ctx++ = readl(addr + APB_DMA_GEN);
	*ctx++ = readl(addr + APB_DMA_CNTRL);
	*ctx++ = readl(addr + APB_DMA_IRQ_MASK);

	for (i=0; i<TEGRA_SYSTEM_DMA_CH_NR; i++) {
		addr = IO_ADDRESS(TEGRA_APB_DMA_CH0_BASE +
				  TEGRA_APB_DMA_CH0_SIZE * i);

		*ctx++ = readl(addr + APB_DMA_CHAN_CSR);
		*ctx++ = readl(addr + APB_DMA_CHAN_AHB_PTR);
		*ctx++ = readl(addr + APB_DMA_CHAN_AHB_SEQ);
		*ctx++ = readl(addr + APB_DMA_CHAN_APB_PTR);
		*ctx++ = readl(addr + APB_DMA_CHAN_APB_SEQ);
	}
}

void tegra_dma_resume(void)
{
	void __iomem *addr = IO_ADDRESS(TEGRA_APB_DMA_BASE);
	u32 *ctx = apb_dma;
	int i;

	writel(*ctx++, addr + APB_DMA_GEN);
	writel(*ctx++, addr + APB_DMA_CNTRL);
	writel(*ctx++, addr + APB_DMA_IRQ_MASK_SET);

	for (i=0; i<TEGRA_SYSTEM_DMA_CH_NR; i++) {
		addr = IO_ADDRESS(TEGRA_APB_DMA_CH0_BASE +
				  TEGRA_APB_DMA_CH0_SIZE * i);

		writel(*ctx++, addr + APB_DMA_CHAN_CSR);
		writel(*ctx++, addr + APB_DMA_CHAN_AHB_PTR);
		writel(*ctx++, addr + APB_DMA_CHAN_AHB_SEQ);
		writel(*ctx++, addr + APB_DMA_CHAN_APB_PTR);
		writel(*ctx++, addr + APB_DMA_CHAN_APB_SEQ);
	}
}

#endif
