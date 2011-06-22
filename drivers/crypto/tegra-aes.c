/*
 * drivers/crypto/tegra-aes.c
 *
 * aes driver for NVIDIA tegra aes hardware
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/workqueue.h>

#include <mach/arb_sema.h>
#include <mach/clk.h>
#include "../video/tegra/nvmap/nvmap.h"

#include <crypto/scatterwalk.h>
#include <crypto/aes.h>
#include <crypto/internal/rng.h>

#include "tegra-aes.h"

#define FLAGS_MODE_MASK		0x00ff
#define FLAGS_ENCRYPT		BIT(0)
#define FLAGS_CBC		BIT(1)
#define FLAGS_GIV		BIT(2)
#define FLAGS_RNG		BIT(3)
#define FLAGS_OFB		BIT(4)
#define FLAGS_INIT		BIT(5)
#define FLAGS_BUSY		1

/*
 * Defines AES engine Max process bytes size in one go, which takes 1 msec.
 * AES engine spends about 176 cycles/16-bytes or 11 cycles/byte
 * The duration CPU can use the BSE to 1 msec, then the number of available
 * cycles of AVP/BSE is 216K. In this duration, AES can process 216/11 ~= 19KB
 * Based on this AES_HW_DMA_BUFFER_SIZE_BYTES is configured to 16KB.
 */
#define AES_HW_DMA_BUFFER_SIZE_BYTES	0x4000

/*
 * The key table length is 64 bytes
 * (This includes first upto 32 bytes key + 16 bytes original initial vector
 * and 16 bytes updated initial vector)
 */
#define AES_HW_KEY_TABLE_LENGTH_BYTES	64

#define AES_HW_IV_SIZE	16
#define AES_HW_KEYSCHEDULE_LEN	256
#define ARB_SEMA_TIMEOUT	500

/*
 * The memory being used is divides as follows:
 * 1. Key - 32 bytes
 * 2. Original IV - 16 bytes
 * 3. Updated IV - 16 bytes
 * 4. Key schedule - 256 bytes
 *
 * 1+2+3 constitute the hw key table.
 */
#define AES_IVKEY_SIZE (AES_HW_KEY_TABLE_LENGTH_BYTES + AES_HW_KEYSCHEDULE_LEN)

#define DEFAULT_RNG_BLK_SZ	16

/* As of now only 5 commands are USED for AES encryption/Decryption */
#define AES_HW_MAX_ICQ_LENGTH	4

#define ICQBITSHIFT_BLKCNT	0

/* memdma_vd command */
#define MEMDMA_DIR_DTOVRAM	0
#define MEMDMA_DIR_VTODRAM	1
#define MEMDMABITSHIFT_DIR	25
#define MEMDMABITSHIFT_NUM_WORDS	12

/* Define AES Interactive command Queue commands Bit positions */
enum {
	ICQBITSHIFT_KEYTABLEADDR = 0,
	ICQBITSHIFT_KEYTABLEID = 17,
	ICQBITSHIFT_VRAMSEL = 23,
	ICQBITSHIFT_TABLESEL = 24,
	ICQBITSHIFT_OPCODE = 26,
};

/* Define Ucq opcodes required for AES operation */
enum {
	UCQOPCODE_BLKSTARTENGINE = 0x0E,
	UCQOPCODE_DMASETUP = 0x10,
	UCQOPCODE_DMACOMPLETE = 0x11,
	UCQOPCODE_SETTABLE = 0x15,
	UCQOPCODE_MEMDMAVD = 0x22,
};

/* Define Aes command values */
enum {
	UCQCMD_VRAM_SEL = 0x1,
	UCQCMD_CRYPTO_TABLESEL = 0x3,
	UCQCMD_KEYSCHEDTABLESEL = 0x4,
	UCQCMD_KEYTABLESEL = 0x8,
};

#define UCQCMD_KEYTABLEADDRMASK 0x1FFFF

#define AES_NR_KEYSLOTS	8
#define SSK_SLOT_NUM	4

struct tegra_aes_slot {
	struct list_head node;
	int slot_num;
	bool available;
};

struct tegra_aes_reqctx {
	unsigned long mode;
};

#define TEGRA_AES_QUEUE_LENGTH	500

struct tegra_aes_engine {
	struct tegra_aes_dev *dd;
	struct clk *iclk;
	struct clk *pclk;
	struct ablkcipher_request *req;
	struct scatterlist *in_sg;
	struct completion op_complete;
	struct scatterlist *out_sg;
	void __iomem *io_base;
	void __iomem *ivkey_base;
	unsigned long phys_base;
	unsigned long iram_phys;
	void *iram_virt;
	dma_addr_t ivkey_phys_base;
	dma_addr_t dma_buf_in;
	dma_addr_t dma_buf_out;
	size_t total;
	size_t in_offset;
	size_t out_offset;
	u32 engine_offset;
	u32 *buf_in;
	u32 *buf_out;
	int res_id;
	int slot_num;
	int keylen;
	unsigned long busy;
	u8 irq;
	bool new_key;
	bool use_ssk;
};

struct tegra_aes_dev {
	struct device *dev;
	struct tegra_aes_slot *slots;
	struct tegra_aes_engine bsev;
	struct tegra_aes_engine bsea;
	struct nvmap_client *client;
	struct nvmap_handle_ref *h_ref;
	struct tegra_aes_ctx *ctx;
	struct crypto_queue queue;
	spinlock_t lock;
	u64 ctr;
	unsigned long flags;
	u8 dt[DEFAULT_RNG_BLK_SZ];
};

static struct tegra_aes_dev *aes_dev;

struct tegra_aes_ctx {
	struct tegra_aes_dev *dd;
	struct tegra_aes_slot *slot;
};

static struct tegra_aes_ctx rng_ctx;

/* keep registered devices data here */
static LIST_HEAD(slot_list);
static DEFINE_SPINLOCK(list_lock);
static DEFINE_MUTEX(aes_lock);

/* Engine specific work queues */
static void bsev_workqueue_handler(struct work_struct *work);
static void bsea_workqueue_handler(struct work_struct *work);

static DECLARE_WORK(bsev_work, bsev_workqueue_handler);
static DECLARE_WORK(bsea_work, bsea_workqueue_handler);

static struct workqueue_struct *bsev_wq;
static struct workqueue_struct *bsea_wq;

extern unsigned long long tegra_chip_uid(void);

static inline u32 aes_readl(struct tegra_aes_engine *engine, u32 offset)
{
	return readl(engine->io_base + offset);
}

static inline void aes_writel(struct tegra_aes_engine *engine,
	u32 val, u32 offset)
{
	writel(val, engine->io_base + offset);
}

static int alloc_iram(struct tegra_aes_dev *dd)
{
	size_t size, align ;
	int err;

	dd->h_ref = NULL;

	/* [key+iv+u-iv=64B] * 8 = 512Bytes */
	size = align = (AES_HW_KEY_TABLE_LENGTH_BYTES * AES_NR_KEYSLOTS);
	dd->client = nvmap_create_client(nvmap_dev, "aes_bsea");
	if (IS_ERR(dd->client)) {
		dev_err(dd->dev, "nvmap_create_client failed\n");
		goto out;
	}

	dd->h_ref = nvmap_create_handle(dd->client, size);
	if (IS_ERR(dd->h_ref)) {
		dev_err(dd->dev, "nvmap_create_handle failed\n");
		goto out;
	}

	/* Allocate memory in the iram */
	err = nvmap_alloc_handle_id(dd->client, nvmap_ref_to_id(dd->h_ref),
		NVMAP_HEAP_CARVEOUT_IRAM, align, 0);
	if (err) {
		dev_err(dd->dev, "nvmap_alloc_handle_id failed\n");
		nvmap_free_handle_id(dd->client, nvmap_ref_to_id(dd->h_ref));
		goto out;
	}
	dd->bsea.iram_phys = nvmap_handle_address(dd->client,
					nvmap_ref_to_id(dd->h_ref));

	dd->bsea.iram_virt = nvmap_mmap(dd->h_ref);	/* get virtual address */
	if (!dd->bsea.iram_virt) {
		dev_err(dd->dev, "%s: no mem, BSEA IRAM alloc failure\n",
		__func__);
		goto out;
	}
	memset(dd->bsea.iram_virt, 0, dd->h_ref->handle->size);

	return 0;

out:
	if (dd->bsea.iram_virt)
		nvmap_munmap(dd->h_ref, dd->bsea.iram_virt);

	if (dd->client) {
		nvmap_free_handle_id(dd->client, nvmap_ref_to_id(dd->h_ref));
		nvmap_client_put(dd->client);
	}

	return -ENOMEM;
}

static void free_iram(struct tegra_aes_dev *dd)
{
	if (dd->bsea.iram_virt)
		nvmap_munmap(dd->h_ref, dd->bsea.iram_virt);

	if (dd->client) {
		nvmap_free_handle_id(dd->client, nvmap_ref_to_id(dd->h_ref));
		nvmap_client_put(dd->client);
	}
}

static int aes_hw_init(struct tegra_aes_engine *engine)
{
	struct tegra_aes_dev *dd = aes_dev;
	int ret = 0;

	if (engine->pclk) {
		ret = clk_enable(engine->pclk);
		if (ret < 0) {
			dev_err(dd->dev, "%s: pclock enable fail(%d)\n",
			__func__, ret);
			return ret;
		}
	}

	if (engine->iclk) {
		ret = clk_enable(engine->iclk);
		if (ret < 0) {
			dev_err(dd->dev, "%s: iclock enable fail(%d)\n",
			__func__, ret);
			if (engine->pclk)
				clk_disable(engine->pclk);
			return ret;
		}
	}

	return ret;
}

static void aes_hw_deinit(struct tegra_aes_engine *engine)
{
	if (engine->pclk)
		clk_disable(engine->pclk);

	if (engine->iclk)
		clk_disable(engine->iclk);
}

static int aes_start_crypt(struct tegra_aes_engine *eng, u32 in_addr,
	u32 out_addr, int nblocks, int mode, bool upd_iv)
{
	u32 cmdq[AES_HW_MAX_ICQ_LENGTH];
	int qlen = 0, i, eng_busy, icq_empty, ret;
	u32 value;

	aes_writel(eng, 0xFFFFFFFF, INTR_STATUS);

	/* error, dma xfer complete */
	aes_writel(eng, 0x33, INT_ENB);
	enable_irq(eng->irq);

	cmdq[qlen++] = UCQOPCODE_DMASETUP << ICQBITSHIFT_OPCODE;
	cmdq[qlen++] = in_addr;
	cmdq[qlen++] = UCQOPCODE_BLKSTARTENGINE << ICQBITSHIFT_OPCODE |
		(nblocks-1) << ICQBITSHIFT_BLKCNT;
	cmdq[qlen++] = UCQOPCODE_DMACOMPLETE << ICQBITSHIFT_OPCODE;

	value = aes_readl(eng, CMDQUE_CONTROL);
	/* access SDRAM through AHB */
	value &= (~CMDQ_CTRL_SRC_STM_SEL_FIELD & ~CMDQ_CTRL_DST_STM_SEL_FIELD);
	value |= (CMDQ_CTRL_SRC_STM_SEL_FIELD | CMDQ_CTRL_DST_STM_SEL_FIELD |
		CMDQ_CTRL_ICMDQEN_FIELD | CMDQ_CTRL_ERROR_FLUSH_ENB);
	aes_writel(eng, value, CMDQUE_CONTROL);

	value = 0;
	if (mode & FLAGS_CBC) {
		value = ((0x1 << SECURE_INPUT_ALG_SEL_SHIFT) |
			((eng->keylen * 8) << SECURE_INPUT_KEY_LEN_SHIFT) |
			((u32)upd_iv << SECURE_IV_SELECT_SHIFT) |
			(((mode & FLAGS_ENCRYPT) ? 2 : 3)
				<< SECURE_XOR_POS_SHIFT) |
			(0 << SECURE_INPUT_SEL_SHIFT) |
			(((mode & FLAGS_ENCRYPT) ? 2 : 3)
				<< SECURE_VCTRAM_SEL_SHIFT) |
			((mode & FLAGS_ENCRYPT) ? 1 : 0)
				<< SECURE_CORE_SEL_SHIFT |
			(0 << SECURE_RNG_ENB_SHIFT) |
			(0 << SECURE_HASH_ENB_SHIFT));
	} else if (mode & FLAGS_OFB) {
		value = ((0x1 << SECURE_INPUT_ALG_SEL_SHIFT) |
			((eng->keylen * 8) << SECURE_INPUT_KEY_LEN_SHIFT) |
			((u32)upd_iv << SECURE_IV_SELECT_SHIFT) |
			((u32)0 << SECURE_IV_SELECT_SHIFT) |
			(SECURE_XOR_POS_FIELD) |
			(2 << SECURE_INPUT_SEL_SHIFT) |
			(0 << SECURE_VCTRAM_SEL_SHIFT) |
			(SECURE_CORE_SEL_FIELD) |
			(0 << SECURE_RNG_ENB_SHIFT) |
			(0 << SECURE_HASH_ENB_SHIFT));
	} else if (mode & FLAGS_RNG){
		value = ((0x1 << SECURE_INPUT_ALG_SEL_SHIFT) |
			((eng->keylen * 8) << SECURE_INPUT_KEY_LEN_SHIFT) |
			((u32)upd_iv << SECURE_IV_SELECT_SHIFT) |
			(0 << SECURE_XOR_POS_SHIFT) |
			(0 << SECURE_INPUT_SEL_SHIFT) |
			((mode & FLAGS_ENCRYPT) ? 1 : 0)
				<< SECURE_CORE_SEL_SHIFT |
			(1 << SECURE_RNG_ENB_SHIFT) |
			(0 << SECURE_HASH_ENB_SHIFT));
	} else {
		value = ((0x1 << SECURE_INPUT_ALG_SEL_SHIFT) |
			((eng->keylen * 8) << SECURE_INPUT_KEY_LEN_SHIFT) |
			((u32)upd_iv << SECURE_IV_SELECT_SHIFT) |
			(0 << SECURE_XOR_POS_SHIFT) |
			(0 << SECURE_INPUT_SEL_SHIFT) |
			(((mode & FLAGS_ENCRYPT) ? 1 : 0)
				<< SECURE_CORE_SEL_SHIFT) |
			(0 << SECURE_RNG_ENB_SHIFT) |
				(0 << SECURE_HASH_ENB_SHIFT));
	}
	aes_writel(eng, value, SECURE_INPUT_SELECT);

	aes_writel(eng, out_addr, SECURE_DEST_ADDR);
	INIT_COMPLETION(eng->op_complete);

	for (i = 0; i < qlen - 1; i++) {
		do {
			value = aes_readl(eng, INTR_STATUS);
			eng_busy = value & BIT(0);
			icq_empty = value & BIT(3);
		} while (eng_busy || (!icq_empty));
		aes_writel(eng, cmdq[i], ICMDQUE_WR);
	}

	ret = wait_for_completion_timeout(&eng->op_complete,
		msecs_to_jiffies(150));
	if (ret == 0) {
		dev_err(aes_dev->dev, "engine%d timed out (0x%x)\n",
			eng->res_id, aes_readl(eng, INTR_STATUS));
		disable_irq(eng->irq);
		return -ETIMEDOUT;
	}

	disable_irq(eng->irq);
	aes_writel(eng, cmdq[qlen - 1], ICMDQUE_WR);
	return 0;
}

static void aes_release_key_slot(struct tegra_aes_ctx *ctx)
{
	spin_lock(&list_lock);
	ctx->slot->available = true;
	ctx->slot = NULL;
	spin_unlock(&list_lock);
}

static struct tegra_aes_slot *aes_find_key_slot(struct tegra_aes_dev *dd)
{
	struct tegra_aes_slot *slot = NULL;
	bool found = 0;

	spin_lock(&list_lock);
	list_for_each_entry(slot, &slot_list, node) {
		dev_dbg(dd->dev, "empty:%d, num:%d\n", slot->available,
			slot->slot_num);
		if (slot->available) {
			slot->available = false;
			found = 1;
			break;
		}
	}

	spin_unlock(&list_lock);
	return found ? slot : NULL;
}

static int aes_set_key(struct tegra_aes_engine *eng)
{
	struct tegra_aes_dev *dd = aes_dev;
	u32 value, cmdq[2];
	int i, eng_busy, icq_empty, dma_busy;

	if (!eng) {
		dev_err(dd->dev, "%s: context invalid\n", __func__);
		return -EINVAL;
	}

	/* enable key schedule generation in hardware */
	value = aes_readl(eng, SECURE_CONFIG_EXT);
	value &= ~SECURE_KEY_SCH_DIS_FIELD;
	aes_writel(eng, value, SECURE_CONFIG_EXT);

	/* select the key slot */
	value = aes_readl(eng, SECURE_CONFIG);
	value &= ~SECURE_KEY_INDEX_FIELD;
	value |= (eng->slot_num << SECURE_KEY_INDEX_SHIFT);
	aes_writel(eng, value, SECURE_CONFIG);

	if (eng->use_ssk)
		goto out;

	if (eng->res_id == TEGRA_ARB_BSEV) {
		/* copy the key table from sdram to vram */
		cmdq[0] = 0;
		cmdq[0] = UCQOPCODE_MEMDMAVD << ICQBITSHIFT_OPCODE |
				(MEMDMA_DIR_DTOVRAM << MEMDMABITSHIFT_DIR) |
			(AES_HW_KEY_TABLE_LENGTH_BYTES/sizeof(u32))
			<< MEMDMABITSHIFT_NUM_WORDS;
		cmdq[1] = (u32)eng->ivkey_phys_base;
		for (i = 0; i < ARRAY_SIZE(cmdq); i++)
			aes_writel(eng, cmdq[i], ICMDQUE_WR);
		do {
			value = aes_readl(eng, INTR_STATUS);
			eng_busy = value & BIT(0);
			icq_empty = value & BIT(3);
			dma_busy = value & BIT(23);
		} while (eng_busy & (!icq_empty) & dma_busy);

		/* settable command to get key into internal registers */
		value = 0;
		value = UCQOPCODE_SETTABLE << ICQBITSHIFT_OPCODE |
			UCQCMD_CRYPTO_TABLESEL << ICQBITSHIFT_TABLESEL |
			UCQCMD_VRAM_SEL << ICQBITSHIFT_VRAMSEL |
			(UCQCMD_KEYTABLESEL | eng->slot_num)
			<< ICQBITSHIFT_KEYTABLEID;
		aes_writel(eng, value, ICMDQUE_WR);
		do {
			value = aes_readl(eng, INTR_STATUS);
			eng_busy = value & BIT(0);
			icq_empty = value & BIT(3);
		} while (eng_busy & (!icq_empty));
	} else {
		/* settable command to get key into internal registers */
		value = 0;
		value = UCQOPCODE_SETTABLE << ICQBITSHIFT_OPCODE |
			UCQCMD_CRYPTO_TABLESEL << ICQBITSHIFT_TABLESEL |
			(UCQCMD_KEYTABLESEL | eng->slot_num)
			<< ICQBITSHIFT_KEYTABLEID |
			dd->bsea.iram_phys >> 2;
			aes_writel(eng, value, ICMDQUE_WR);
		do {
			value = aes_readl(eng, INTR_STATUS);
			eng_busy = value & BIT(0);
			icq_empty = value & BIT(3);
		} while (eng_busy & (!icq_empty));
	}
out:
	return 0;
}

static int tegra_aes_handle_req(struct tegra_aes_engine *eng)
{
	struct tegra_aes_dev *dd = aes_dev;
	struct tegra_aes_ctx *ctx;
	struct crypto_async_request *async_req, *backlog;
	struct tegra_aes_reqctx *rctx;
	struct ablkcipher_request *req;
	unsigned long irq_flags;
	int dma_max = AES_HW_DMA_BUFFER_SIZE_BYTES;
	int nblocks, total, ret = 0, count = 0;
	dma_addr_t addr_in, addr_out;
	struct scatterlist *in_sg, *out_sg;

	spin_lock_irqsave(&dd->lock, irq_flags);
	backlog = crypto_get_backlog(&dd->queue);
	async_req = crypto_dequeue_request(&dd->queue);
	if (!async_req)
		clear_bit(FLAGS_BUSY, &eng->busy);
	spin_unlock_irqrestore(&dd->lock, irq_flags);

	if (!async_req)
		return -ENODATA;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	req = ablkcipher_request_cast(async_req);
	dev_dbg(dd->dev, "%s: get new req (engine #%d)\n", __func__,
		eng->res_id);
	if (!req->src || !req->dst)
		return -EINVAL;

	/* take the hardware semaphore */
	if (tegra_arb_mutex_lock_timeout(eng->res_id, ARB_SEMA_TIMEOUT) < 0) {
		dev_err(dd->dev, "aes hardware (%d) not available\n",
		eng->res_id);
		return -EBUSY;
	}

	/* assign new request to device */
	eng->req = req;
	eng->total = req->nbytes;
	eng->in_offset = 0;
	eng->in_sg = req->src;
	eng->out_offset = 0;
	eng->out_sg = req->dst;

	in_sg = eng->in_sg;
	out_sg = eng->out_sg;
	total = eng->total;

	rctx = ablkcipher_request_ctx(req);
	ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	rctx->mode &= FLAGS_MODE_MASK;
	dd->flags = (dd->flags & ~FLAGS_MODE_MASK) | rctx->mode;

	if (eng->new_key) {
		aes_set_key(eng);
		eng->new_key = false;
	}

	if (((dd->flags & FLAGS_CBC) || (dd->flags & FLAGS_OFB)) && req->info) {
		/* set iv to the aes hw slot
		 * Hw generates updated iv only after iv is set in slot.
		 * So key and iv is passed asynchronously.
		 */
		memcpy(eng->buf_in, (u8 *)req->info, AES_BLOCK_SIZE);

		ret = aes_start_crypt(eng, (u32)eng->dma_buf_in,
			(u32)eng->dma_buf_out, 1, FLAGS_CBC, false);
		if (ret < 0) {
			dev_err(dd->dev, "aes_start_crypt fail(%d)\n", ret);
			goto out;
		}
	}

	while (total) {
		dev_dbg(dd->dev, "remain: %d\n", total);
		ret = dma_map_sg(dd->dev, in_sg, 1, DMA_TO_DEVICE);
		if (!ret) {
			dev_err(dd->dev, "dma_map_sg() error\n");
			goto out;
		}

		ret = dma_map_sg(dd->dev, out_sg, 1, DMA_FROM_DEVICE);
		if (!ret) {
			dev_err(dd->dev, "dma_map_sg() error\n");
			dma_unmap_sg(dd->dev, eng->in_sg,
				1, DMA_TO_DEVICE);
			goto out;
		}

		addr_in = sg_dma_address(in_sg);
		addr_out = sg_dma_address(out_sg);
		count = min((int)sg_dma_len(in_sg), (int)dma_max);
		WARN_ON(sg_dma_len(in_sg) != sg_dma_len(out_sg));
		nblocks = DIV_ROUND_UP(count, AES_BLOCK_SIZE);

		ret = aes_start_crypt(eng, addr_in, addr_out, nblocks,
			dd->flags, true);

		dma_unmap_sg(dd->dev, out_sg, 1, DMA_FROM_DEVICE);
		dma_unmap_sg(dd->dev, in_sg, 1, DMA_TO_DEVICE);
		if (ret < 0) {
			dev_err(dd->dev, "aes_start_crypt fail(%d)\n", ret);
			goto out;
		}

		dev_dbg(dd->dev, "out: copied %d\n", count);
		total -= count;
		in_sg = sg_next(in_sg);
		out_sg = sg_next(out_sg);
		WARN_ON(((total != 0) && (!in_sg || !out_sg)));
	}

out:
	/* release the hardware semaphore */
	tegra_arb_mutex_unlock(eng->res_id);
	eng->total = total;

	if (eng->req->base.complete)
		eng->req->base.complete(&eng->req->base, ret);

	dev_dbg(dd->dev, "%s: exit\n", __func__);
	return ret;
}

static int tegra_aes_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
	unsigned int keylen)
{
	struct tegra_aes_ctx *ctx = crypto_ablkcipher_ctx(tfm);
	struct tegra_aes_dev *dd = aes_dev;
	struct tegra_aes_slot *key_slot;

	if (!ctx || !dd) {
		pr_err("ctx=0x%x, dd=0x%x\n",
			(unsigned int)ctx, (unsigned int)dd);
		return -EINVAL;
	}

	if ((keylen != AES_KEYSIZE_128) && (keylen != AES_KEYSIZE_192) &&
		(keylen != AES_KEYSIZE_256)) {
		dev_err(dd->dev, "unsupported key size\n");
		return -EINVAL;
	}

	dev_dbg(dd->dev, "keylen: %d\n", keylen);

	ctx->dd = dd;

	if (key) {
		if (!ctx->slot) {
			key_slot = aes_find_key_slot(dd);
			if (!key_slot) {
				dev_err(dd->dev, "no empty slot\n");
				return -ENOMEM;
			}
			ctx->slot = key_slot;
		}

		/* copy the key */
		memset(dd->bsev.ivkey_base, 0, AES_HW_KEY_TABLE_LENGTH_BYTES);
		memset(dd->bsea.iram_virt, 0, AES_HW_KEY_TABLE_LENGTH_BYTES);
		memcpy(dd->bsev.ivkey_base, key, keylen);
		memcpy(dd->bsea.iram_virt, key, keylen);
	} else {
		dd->bsev.slot_num = SSK_SLOT_NUM;
		dd->bsea.slot_num = SSK_SLOT_NUM;
		dd->bsev.use_ssk = true;
		dd->bsea.use_ssk = true;
		keylen = AES_KEYSIZE_128;
	}

	dd->bsev.keylen = keylen;
	dd->bsea.keylen = keylen;
	dd->bsev.new_key = true;
	dd->bsea.new_key = true;
	dev_dbg(dd->dev, "done\n");
	return 0;
}

static void bsev_workqueue_handler(struct work_struct *work)
{
	struct tegra_aes_dev *dd = aes_dev;
	struct tegra_aes_engine *engine = &dd->bsev;
	int ret;

	aes_hw_init(engine);

	/* empty the crypto queue and then return */
	do {
		ret = tegra_aes_handle_req(engine);
	} while (!ret);

	aes_hw_deinit(engine);
}

static void bsea_workqueue_handler(struct work_struct *work)
{
	struct tegra_aes_dev *dd = aes_dev;
	struct tegra_aes_engine *engine = &dd->bsea;
	int ret;

	aes_hw_init(engine);

	/* empty the crypto queue and then return */
	do {
		ret = tegra_aes_handle_req(engine);
	} while (!ret);

	aes_hw_deinit(engine);
}

#define INT_ERROR_MASK	0xFFF000
static irqreturn_t aes_bsev_irq(int irq, void *dev_id)
{
	struct tegra_aes_dev *dd = (struct tegra_aes_dev *)dev_id;
	u32 value = aes_readl(&dd->bsev, INTR_STATUS);

	dev_dbg(dd->dev, "bsev irq_stat: 0x%x", value);
	if (value & INT_ERROR_MASK)
		aes_writel(&dd->bsev, INT_ERROR_MASK, INTR_STATUS);

	value = aes_readl(&dd->bsev, INTR_STATUS);
	if (!(value & ENGINE_BUSY_FIELD))
		complete(&dd->bsev.op_complete);

	return IRQ_HANDLED;
}

static irqreturn_t aes_bsea_irq(int irq, void *dev_id)
{
	struct tegra_aes_dev *dd = (struct tegra_aes_dev *)dev_id;
	u32 value = aes_readl(&dd->bsea, INTR_STATUS);

	dev_dbg(dd->dev, "bsea irq_stat: 0x%x", value);
	if (value & INT_ERROR_MASK)
		aes_writel(&dd->bsea, INT_ERROR_MASK, INTR_STATUS);

	value = aes_readl(&dd->bsea, INTR_STATUS);
	if (!(value & ENGINE_BUSY_FIELD))
		complete(&dd->bsea.op_complete);

	return IRQ_HANDLED;
}

static int tegra_aes_crypt(struct ablkcipher_request *req, unsigned long mode)
{
	struct tegra_aes_reqctx *rctx = ablkcipher_request_ctx(req);
	struct tegra_aes_dev *dd = aes_dev;
	unsigned long flags;
	int err = 0;
	int bsev_busy;
	int bsea_busy;

	dev_dbg(dd->dev, "nbytes: %d, enc: %d, cbc: %d\n", req->nbytes,
		!!(mode & FLAGS_ENCRYPT),
		!!(mode & FLAGS_CBC));

	rctx->mode = mode;

	spin_lock_irqsave(&dd->lock, flags);
	err = ablkcipher_enqueue_request(&dd->queue, req);
	bsev_busy = test_and_set_bit(FLAGS_BUSY, &dd->bsev.busy);
	bsea_busy = test_and_set_bit(FLAGS_BUSY, &dd->bsea.busy);
	spin_unlock_irqrestore(&dd->lock, flags);

	if (!bsev_busy)
		queue_work(bsev_wq, &bsev_work);
	if (!bsea_busy)
		queue_work(bsea_wq, &bsea_work);

	return err;
}

static int tegra_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	return tegra_aes_crypt(req, FLAGS_ENCRYPT);
}

static int tegra_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	return tegra_aes_crypt(req, 0);
}

static int tegra_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	return tegra_aes_crypt(req, FLAGS_ENCRYPT | FLAGS_CBC);
}

static int tegra_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	return tegra_aes_crypt(req, FLAGS_CBC);
}
static int tegra_aes_ofb_encrypt(struct ablkcipher_request *req)
{
	return tegra_aes_crypt(req, FLAGS_ENCRYPT | FLAGS_OFB);
}

static int tegra_aes_ofb_decrypt(struct ablkcipher_request *req)
{
	return tegra_aes_crypt(req, FLAGS_OFB);
}
static int tegra_aes_get_random(struct crypto_rng *tfm, u8 *rdata,
	unsigned int dlen)
{
	struct tegra_aes_dev *dd = aes_dev;
	struct tegra_aes_engine *eng = &dd->bsea;
	int ret, i;
	u8 *dest = rdata, *dt = dd->dt;

	/* take mutex to access the aes hw */
	mutex_lock(&aes_lock);

	/* take the hardware semaphore */
	if (tegra_arb_mutex_lock_timeout(eng->res_id, ARB_SEMA_TIMEOUT) < 0) {
		dev_err(dd->dev, "aes hardware (%d) not available\n",
		eng->res_id);
		mutex_unlock(&aes_lock);
		return -EBUSY;
	}

	ret = aes_hw_init(eng);
	if (ret < 0) {
		dev_err(dd->dev, "%s: hw init fail(%d)\n", __func__, ret);
		dlen = ret;
		goto fail;
	}

	memset(eng->buf_in, 0, AES_BLOCK_SIZE);
	memcpy(eng->buf_in, dt, DEFAULT_RNG_BLK_SZ);

	ret = aes_start_crypt(eng, (u32)eng->dma_buf_in, (u32)eng->dma_buf_out,
		1, FLAGS_ENCRYPT | FLAGS_RNG, true);
	if (ret < 0) {
		dev_err(dd->dev, "aes_start_crypt fail(%d)\n", ret);
		dlen = ret;
		goto out;
	}
	memcpy(dest, eng->buf_out, dlen);

	/* update the DT */
	for (i = DEFAULT_RNG_BLK_SZ - 1; i >= 0; i--) {
		dt[i] += 1;
		if (dt[i] != 0)
			break;
	}

out:
	aes_hw_deinit(eng);

fail:
	/* release the hardware semaphore */
	tegra_arb_mutex_unlock(eng->res_id);
	mutex_unlock(&aes_lock);
	dev_dbg(dd->dev, "%s: done\n", __func__);
	return dlen;
}

static int tegra_aes_rng_reset(struct crypto_rng *tfm, u8 *seed,
	unsigned int slen)
{
	struct tegra_aes_dev *dd = aes_dev;
	struct tegra_aes_ctx *ctx = &rng_ctx;
	struct tegra_aes_engine *eng = &dd->bsea;
	struct tegra_aes_slot *key_slot;
	struct timespec ts;
	int ret = 0;
	u64 nsec, tmp[2];
	u8 *dt;

	if (!eng || !dd) {
		dev_err(dd->dev, "eng=0x%x, dd=0x%x\n",
			(unsigned int)eng, (unsigned int)dd);
		return -EINVAL;
	}

	if (slen < (DEFAULT_RNG_BLK_SZ + AES_KEYSIZE_128)) {
		return -ENOMEM;
	}

	dd->flags = FLAGS_ENCRYPT | FLAGS_RNG;

	/* take mutex to access the aes hw */
	mutex_lock(&aes_lock);

	if (!ctx->slot) {
		key_slot = aes_find_key_slot(dd);
		if (!key_slot) {
			dev_err(dd->dev, "no empty slot\n");
			mutex_unlock(&aes_lock);
			return -ENOMEM;
		}
		ctx->slot = key_slot;
	}

	/* copy the key to the key slot */
	memset(dd->bsea.iram_virt, 0, AES_HW_KEY_TABLE_LENGTH_BYTES);
	memcpy(dd->bsea.iram_virt, seed + DEFAULT_RNG_BLK_SZ, AES_KEYSIZE_128);

	/* take the hardware semaphore */
	if (tegra_arb_mutex_lock_timeout(eng->res_id, ARB_SEMA_TIMEOUT) < 0) {
		dev_err(dd->dev, "aes hardware (%d) not available\n",
		eng->res_id);
		mutex_unlock(&aes_lock);
		return -EBUSY;
	}

	ret = aes_hw_init(eng);
	if (ret < 0) {
		dev_err(dd->dev, "%s: hw init fail(%d)\n", __func__, ret);
		goto fail;
	}

	dd->ctx = ctx;
	eng->slot_num = ctx->slot->slot_num;
	eng->keylen = AES_KEYSIZE_128;
	aes_set_key(eng);

	/* set seed to the aes hw slot */
	memset(eng->buf_in, 0, AES_BLOCK_SIZE);
	memcpy(eng->buf_in, seed, DEFAULT_RNG_BLK_SZ);
	ret = aes_start_crypt(eng, (u32)eng->dma_buf_in,
	  (u32)eng->dma_buf_out, 1, FLAGS_CBC, false);
	if (ret < 0) {
		dev_err(dd->dev, "aes_start_crypt fail(%d)\n", ret);
		goto out;
	}

	if (slen >= (2 * DEFAULT_RNG_BLK_SZ + AES_KEYSIZE_128)) {
		dt = seed + DEFAULT_RNG_BLK_SZ + AES_KEYSIZE_128;
	} else {
		getnstimeofday(&ts);
		nsec = timespec_to_ns(&ts);
		do_div(nsec, 1000);
		nsec ^= dd->ctr << 56;
		dd->ctr++;
		tmp[0] = nsec;
		tmp[1] = tegra_chip_uid();
		dt = (u8 *)tmp;
	}
	memcpy(dd->dt, dt, DEFAULT_RNG_BLK_SZ);

out:
	aes_hw_deinit(eng);

fail:
	/* release the hardware semaphore */
	tegra_arb_mutex_unlock(eng->res_id);
	mutex_unlock(&aes_lock);

	dev_dbg(dd->dev, "%s: done\n", __func__);
	return ret;
}

static int tegra_aes_cra_init(struct crypto_tfm *tfm)
{
	tfm->crt_ablkcipher.reqsize = sizeof(struct tegra_aes_reqctx);
	return 0;
}

void tegra_aes_cra_exit(struct crypto_tfm *tfm)
{
	struct tegra_aes_ctx *ctx = crypto_ablkcipher_ctx((struct crypto_ablkcipher *)tfm);

	if (ctx && ctx->slot)
		aes_release_key_slot(ctx);
}

static struct crypto_alg algs[] = {
	{
		.cra_name = "disabled_ecb(aes)",
		.cra_driver_name = "ecb-aes-tegra",
		.cra_priority = 100,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct tegra_aes_ctx),
		.cra_alignmask = 3,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_aes_cra_init,
		.cra_exit = tegra_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.setkey = tegra_aes_setkey,
			.encrypt = tegra_aes_ecb_encrypt,
			.decrypt = tegra_aes_ecb_decrypt,
		},
	}, {
		.cra_name = "disabled_cbc(aes)",
		.cra_driver_name = "cbc-aes-tegra",
		.cra_priority = 100,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = AES_BLOCK_SIZE,
		.cra_ctxsize  = sizeof(struct tegra_aes_ctx),
		.cra_alignmask = 3,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_aes_cra_init,
		.cra_exit = tegra_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.ivsize = AES_MIN_KEY_SIZE,
			.setkey = tegra_aes_setkey,
			.encrypt = tegra_aes_cbc_encrypt,
			.decrypt = tegra_aes_cbc_decrypt,
		}
	}, {
		.cra_name = "disabled_ofb(aes)",
		.cra_driver_name = "ofb-aes-tegra",
		.cra_priority = 100,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = AES_BLOCK_SIZE,
		.cra_ctxsize  = sizeof(struct tegra_aes_ctx),
		.cra_alignmask = 3,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_aes_cra_init,
		.cra_exit = tegra_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.ivsize = AES_MIN_KEY_SIZE,
			.setkey = tegra_aes_setkey,
			.encrypt = tegra_aes_ofb_encrypt,
			.decrypt = tegra_aes_ofb_decrypt,
		}
	}, {
		.cra_name = "disabled_ansi_cprng",
		.cra_driver_name = "rng-aes-tegra",
		.cra_priority = 100,
		.cra_flags = CRYPTO_ALG_TYPE_RNG,
		.cra_ctxsize = sizeof(struct tegra_aes_ctx),
		.cra_type = &crypto_rng_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_aes_cra_init,
		.cra_exit = tegra_aes_cra_exit,
		.cra_u.rng = {
			.rng_make_random = tegra_aes_get_random,
			.rng_reset = tegra_aes_rng_reset,
			.seedsize = AES_KEYSIZE_128 + (2 * DEFAULT_RNG_BLK_SZ),
		}
	}
};

static int tegra_aes_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tegra_aes_dev *dd;
	struct resource *res[2];
	int err = -ENOMEM, i = 0, j;

	if (aes_dev)
		return -EEXIST;

	dd = kzalloc(sizeof(struct tegra_aes_dev), GFP_KERNEL);
	if (dd == NULL) {
		dev_err(dev, "unable to alloc data struct.\n");
		return -ENOMEM;;
	}
	dd->dev = dev;
	platform_set_drvdata(pdev, dd);

	dd->slots = kzalloc(sizeof(struct tegra_aes_slot) * AES_NR_KEYSLOTS,
		GFP_KERNEL);
	if (dd->slots == NULL) {
		dev_err(dev, "unable to alloc slot struct.\n");
		goto out;
	}

	spin_lock_init(&dd->lock);
	crypto_init_queue(&dd->queue, TEGRA_AES_QUEUE_LENGTH);

	/* Get the module base address */
	res[0] = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res[1] = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res[0] || !res[1]) {
		dev_err(dev, "invalid resource type: base\n");
		err = -ENODEV;
		goto out;
	}
	dd->bsev.phys_base = res[0]->start;
	dd->bsev.io_base = ioremap(dd->bsev.phys_base, resource_size(res[0]));
	dd->bsea.phys_base = res[1]->start;
	dd->bsea.io_base = ioremap(dd->bsea.phys_base, resource_size(res[1]));

	if (!dd->bsev.io_base || !dd->bsea.io_base) {
		dev_err(dev, "can't ioremap phys_base\n");
		err = -ENOMEM;
		goto out;
	}

	err = alloc_iram(dd);
	if (err < 0) {
		dev_err(dev, "Failed to allocate IRAM for BSEA\n");
		goto out;
	}

	dd->bsev.res_id = TEGRA_ARB_BSEV;
	dd->bsea.res_id = TEGRA_ARB_BSEA;

	dd->bsev.pclk = clk_get(dev, "bsev");
	if (IS_ERR(dd->bsev.pclk)) {
		dev_err(dev, "v: pclock intialization failed.\n");
		err = -ENODEV;
		goto out;
	}

	dd->bsev.iclk = clk_get(dev, "vde");
	if (IS_ERR(dd->bsev.iclk)) {
		dev_err(dev, "v: iclock intialization failed.\n");
		err = -ENODEV;
		goto out;
	}

	dd->bsea.pclk = clk_get(dev, "bsea");
	if (IS_ERR(dd->bsea.pclk)) {
		dev_err(dev, "a: pclock intialization failed.\n");
		err = -ENODEV;
		goto out;
	}

	dd->bsea.iclk = clk_get(dev, "sclk");
	if (IS_ERR(dd->bsea.iclk)) {
		dev_err(dev, "a: iclock intialization failed.\n");
		err = -ENODEV;
		goto out;
	}

	err = clk_set_rate(dd->bsev.iclk, ULONG_MAX);
	if (err) {
		dev_err(dd->dev, "bsev iclk set_rate fail(%d)\n", err);
		goto out;
	}

	err = clk_set_rate(dd->bsea.iclk, ULONG_MAX);
	if (err) {
		dev_err(dd->dev, "bsea iclk set_rate fail(%d)\n", err);
		goto out;
	}

	/*
	 * the foll contiguous memory is allocated as follows -
	 * - hardware key table
	 * - key schedule
	 */
	dd->bsev.ivkey_base = dma_alloc_coherent(dev, SZ_512,
		&dd->bsev.ivkey_phys_base, GFP_KERNEL);
	dd->bsea.ivkey_base = NULL;
	if (!dd->bsev.ivkey_base) {
		dev_err(dev, "can not allocate iv/key buffer for BSEV\n");
		err = -ENOMEM;
		goto out;
	}

	dd->bsev.buf_in = dma_alloc_coherent(dev, AES_HW_DMA_BUFFER_SIZE_BYTES,
		&dd->bsev.dma_buf_in, GFP_KERNEL);
	dd->bsea.buf_in = dma_alloc_coherent(dev, AES_HW_DMA_BUFFER_SIZE_BYTES,
		&dd->bsea.dma_buf_in, GFP_KERNEL);
	if (!dd->bsev.buf_in || !dd->bsea.buf_in) {
		dev_err(dev, "can not allocate dma-in buffer\n");
		err = -ENOMEM;
		goto out;
	}

	dd->bsev.buf_out = dma_alloc_coherent(dev, AES_HW_DMA_BUFFER_SIZE_BYTES,
		&dd->bsev.dma_buf_out, GFP_KERNEL);
	dd->bsea.buf_out = dma_alloc_coherent(dev, AES_HW_DMA_BUFFER_SIZE_BYTES,
		&dd->bsea.dma_buf_out, GFP_KERNEL);
	if (!dd->bsev.buf_out || !dd->bsea.buf_out) {
		dev_err(dev, "can not allocate dma-out buffer\n");
		err = -ENOMEM;
		goto out;
	}

	init_completion(&dd->bsev.op_complete);
	init_completion(&dd->bsea.op_complete);

	bsev_wq = alloc_workqueue("bsev_wq", WQ_HIGHPRI | WQ_UNBOUND, 1);
	bsea_wq = alloc_workqueue("bsea_wq", WQ_HIGHPRI | WQ_UNBOUND, 1);
	if (!bsev_wq || !bsea_wq) {
		dev_err(dev, "alloc_workqueue failed\n");
		goto out;
	}

	/* get the irq */
	dd->bsev.irq = INT_VDE_BSE_V;
	err = request_irq(dd->bsev.irq, aes_bsev_irq, IRQF_TRIGGER_HIGH,
		"tegra-aes", dd);
	if (err) {
		dev_err(dev, "request_irq failed fir BSEV Engine\n");
		goto out;
	}
	disable_irq(dd->bsev.irq);

	dd->bsea.irq = INT_VDE_BSE_A;
	err = request_irq(dd->bsea.irq, aes_bsea_irq, IRQF_TRIGGER_HIGH,
		"tegra-aes", dd);
	if (err) {
		dev_err(dev, "request_irq failed for BSEA Engine\n");
		goto out;
	}
	disable_irq(dd->bsea.irq);

	spin_lock_init(&list_lock);
	spin_lock(&list_lock);
	for (i = 0; i < AES_NR_KEYSLOTS; i++) {
		if (i == SSK_SLOT_NUM)
			continue;
		dd->slots[i].available = true;
		dd->slots[i].slot_num = i;
		INIT_LIST_HEAD(&dd->slots[i].node);
		list_add_tail(&dd->slots[i].node, &slot_list);
	}
	spin_unlock(&list_lock);

	aes_dev = dd;

	for (i = 0; i < ARRAY_SIZE(algs); i++) {
		INIT_LIST_HEAD(&algs[i].cra_list);
		err = crypto_register_alg(&algs[i]);
		if (err)
			goto out;
	}

	dev_info(dev, "registered");
	return 0;

out:
	for (j = 0; j < i; j++)
		crypto_unregister_alg(&algs[j]);

	free_iram(dd);

	if (dd->bsev.ivkey_base) {
		dma_free_coherent(dev, SZ_512, dd->bsev.ivkey_base,
			dd->bsev.ivkey_phys_base);
	}

	if (dd->bsev.buf_in && dd->bsea.buf_in) {
		dma_free_coherent(dev, AES_HW_DMA_BUFFER_SIZE_BYTES,
			dd->bsev.buf_in, dd->bsev.dma_buf_in);
		dma_free_coherent(dev, AES_HW_DMA_BUFFER_SIZE_BYTES,
			dd->bsea.buf_in, dd->bsea.dma_buf_in);
	}

	if (dd->bsev.buf_out && dd->bsea.buf_out) {
		dma_free_coherent(dev, AES_HW_DMA_BUFFER_SIZE_BYTES,
			dd->bsev.buf_out, dd->bsev.dma_buf_out);
		dma_free_coherent(dev, AES_HW_DMA_BUFFER_SIZE_BYTES,
			dd->bsea.buf_out, dd->bsea.dma_buf_out);
	}

	if (dd->bsev.io_base && dd->bsea.io_base) {
		iounmap(dd->bsev.io_base);
		iounmap(dd->bsea.io_base);
	}

	if (dd->bsev.pclk)
		clk_put(dd->bsev.pclk);

	if (dd->bsev.iclk)
		clk_put(dd->bsev.iclk);

	if (dd->bsea.pclk)
		clk_put(dd->bsea.pclk);

	if (bsev_wq)
		destroy_workqueue(bsev_wq);

	if (bsea_wq)
		destroy_workqueue(bsea_wq);

	if (dd->bsev.irq)
		free_irq(dd->bsev.irq, dd);

	if (dd->bsea.irq)
		free_irq(dd->bsea.irq, dd);

	spin_lock(&list_lock);
	list_del(&slot_list);
	spin_unlock(&list_lock);

	kfree(dd->slots);
	kfree(dd);
	aes_dev = NULL;

	dev_err(dev, "%s: initialization failed.\n", __func__);
	return err;
}

static int __devexit tegra_aes_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tegra_aes_dev *dd = platform_get_drvdata(pdev);
	int i;

	if (!dd)
		return -ENODEV;

	cancel_work_sync(&bsev_work);
	cancel_work_sync(&bsea_work);
	destroy_workqueue(bsev_wq);
	destroy_workqueue(bsea_wq);
	free_irq(dd->bsev.irq, dd);
	free_irq(dd->bsea.irq, dd);
	spin_lock(&list_lock);
	list_del(&slot_list);
	spin_unlock(&list_lock);

	for (i = 0; i < ARRAY_SIZE(algs); i++)
		crypto_unregister_alg(&algs[i]);

	free_iram(dd);
	dma_free_coherent(dev, SZ_512, dd->bsev.ivkey_base,
		dd->bsev.ivkey_phys_base);
	dma_free_coherent(dev, AES_HW_DMA_BUFFER_SIZE_BYTES, dd->bsev.buf_in,
		dd->bsev.dma_buf_in);
	dma_free_coherent(dev, AES_HW_DMA_BUFFER_SIZE_BYTES, dd->bsea.buf_in,
		dd->bsea.dma_buf_in);
	dma_free_coherent(dev, AES_HW_DMA_BUFFER_SIZE_BYTES, dd->bsev.buf_out,
		dd->bsev.dma_buf_out);
	dma_free_coherent(dev, AES_HW_DMA_BUFFER_SIZE_BYTES, dd->bsea.buf_out,
		dd->bsea.dma_buf_out);

	iounmap(dd->bsev.io_base);
	iounmap(dd->bsea.io_base);
	clk_put(dd->bsev.iclk);
	clk_put(dd->bsev.pclk);
	clk_put(dd->bsea.pclk);
	kfree(dd->slots);
	kfree(dd);
	aes_dev = NULL;

	return 0;
}

static struct platform_driver tegra_aes_driver = {
	.probe  = tegra_aes_probe,
	.remove = __devexit_p(tegra_aes_remove),
	.driver = {
		.name   = "tegra-aes",
		.owner  = THIS_MODULE,
	},
};

static int __init tegra_aes_mod_init(void)
{
	mutex_init(&aes_lock);
	INIT_LIST_HEAD(&slot_list);
	return  platform_driver_register(&tegra_aes_driver);
}

static void __exit tegra_aes_mod_exit(void)
{
	platform_driver_unregister(&tegra_aes_driver);
}

module_init(tegra_aes_mod_init);
module_exit(tegra_aes_mod_exit);

MODULE_DESCRIPTION("Tegra AES hw acceleration support.");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPLv2");
