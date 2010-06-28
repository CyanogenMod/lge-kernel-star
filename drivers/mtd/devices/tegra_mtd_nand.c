/*
 * drivers/mtd/devices/tegra_mtd_nand.c
 *
 * MTD-class device driver for the internal NAND controller in Tegra SoCs
 *
 * Copyright (c) 2009, NVIDIA Corporation.
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

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/wakelock.h>

#include <asm/dma-mapping.h>

#define NV_DEBUG 0

#include <mach/nvrm_linux.h>
#include <mach/nand.h>
#include "nvddk_nand.h"
#include "nvos.h"
#include "nvassert.h"

#define DRIVER_NAME	"tegra_nand"
#define DRIVER_DESC	"Nvidia Tegra NAND Flash Controller driver"

#define NDFLASH_CS_MAX 8

#define NAND_SPARE_SIZE	 64

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL,  };
#endif

struct tegra_nand_chip {
	spinlock_t	lock;
	uint32_t	chipsize;
	int		num_chips;
	int		curr_chip;
	unsigned int	chip_shift;
	unsigned int	page_shift;
	unsigned int	page_mask;
	unsigned int	column_mask; /* column within page */
	unsigned int	block_shift;
	unsigned int	block_mask;
	unsigned int	page_in_block;
	void		*priv;
};

#define page_size(_mtd) ((_mtd)->chip.column_mask + 1)

struct tegra_nand_info {
	struct tegra_nand_chip	chip;
	struct mtd_info		mtd;
	struct device		*dev;
	struct mtd_partition	*parts;
	struct mutex		lock;
	NvDdkNandHandle		ddk;
	unsigned long		*bb_bitmap; /* block map 1=good 0=bad/unknown */
};
#define MTD_TO_INFO(mtd) container_of((mtd), struct tegra_nand_info, mtd)

#define RT_BAD_MARKER 1

/* must be called with lock held */
static int check_block_isbad(struct mtd_info *mtd, loff_t offs,
	NvU8 tag[NAND_SPARE_SIZE])
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	unsigned long block = offs >> info->chip.block_shift;
	unsigned int chip_nr;
	unsigned int block_nr;
	NandBlockInfo block_info;
	NvU32 page_list[NDFLASH_CS_MAX];
	NvError e;
	int ret = 0;

	NvDdkNandResumeClocks(info->ddk);
	if (info->bb_bitmap[BIT_WORD(block)] & BIT_MASK(block))
		return 0;

	memset(&block_info, 0, sizeof(block_info));
	block_info.pTagBuffer = tag;

	chip_nr = offs >> info->chip.chip_shift;
	block_nr = block & info->chip.block_mask;

	NvDdkNandGetBlockInfo(info->ddk, chip_nr, block_nr,
		&block_info, NV_FALSE);

	if (!block_info.IsFactoryGoodBlock) {
		ret = 1;
		goto out;
	}

	memset(page_list, 0xff, sizeof(page_list));

	/* Second byte of the spare area in the first page of the block
	 * has the run-time bad block marker */
	page_list[chip_nr] = block_nr <<
		(info->chip.block_shift - info->chip.page_shift);

	e = NvDdkNandReadSpare(info->ddk, chip_nr, page_list,
			       tag, 0, NAND_SPARE_SIZE);
	if (e != NvSuccess) {
		dev_err(info->dev, "failed to read spare area [chip=%u, "
			"page=0x%u]\n", chip_nr, page_list[chip_nr]);
		ret = -EIO;
		goto out;
	}

	if (tag[RT_BAD_MARKER] != 0xFF)
		ret = 1;

out:
	NvDdkNandSuspendClocks(info->ddk);
	/* update the bitmap if the block is good */
	if (ret == 0)
		set_bit(block, info->bb_bitmap);
	else if (ret == 1) {
		dev_info(info->dev, "Block 0x%lx is bad [chip=%u,"
			 "offset=0x%llx]\n", block, chip_nr, offs);
	}
	return ret;
}

static int tegra_nand_block_isbad(struct mtd_info *mtd, loff_t offs)
{
	NvU8 tag[NAND_SPARE_SIZE];
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	int ret;

	if (offs >= mtd->size)
		return -EINVAL;

	mutex_lock(&info->lock);
	ret = check_block_isbad(mtd, offs, tag);
	mutex_unlock(&info->lock);

	return ret;
}

static int tegra_nand_block_markbad(struct mtd_info *mtd, loff_t offs)
{
	NvError e = NvSuccess;
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	uint32_t block = offs >> info->chip.block_shift;
	NvU32 page_list[NDFLASH_CS_MAX];
	NvU32 i;
	NvU32 index;
	NvU32 deviceNum;
	NvU8 TagBuff[64];
	const NvU32  BadBlockMarkerOffset = 0x1;

	if (offs >= mtd->size)
		return -EINVAL;

	dev_dbg(info->dev, "marking block 0x%x bad\n", block);

	mutex_lock(&info->lock);
	offs &= ~(mtd->erasesize - 1);

	/* mark the block bad in our bitmap */
	clear_bit(block, info->bb_bitmap);
	mtd->ecc_stats.badblocks++;

	for (i=0;i<NDFLASH_CS_MAX;i++)
		page_list[i] = -1;

	NvOsMemset(TagBuff, sizeof(TagBuff), 0x0);

	index = block & info->chip.block_mask;
	deviceNum = (block * mtd->erasesize) / info->chip.chipsize;
	NV_ASSERT(deviceNum < NDFLASH_CS_MAX);
	page_list[deviceNum] = index * info->chip.page_in_block;

	NvDdkNandResumeClocks(info->ddk);
	NV_CHECK_ERROR_CLEANUP(
		NvDdkNandWriteSpare(info->ddk, deviceNum, page_list,
			TagBuff, BadBlockMarkerOffset, 1)
	);

	NvDdkNandSuspendClocks(info->ddk);
	mutex_unlock(&info->lock);
	return 0;
fail:
	NvDdkNandSuspendClocks(info->ddk);
	mutex_unlock(&info->lock);
	return -ENXIO;
}

static int tegra_nand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	uint32_t num_blocks;
	NvError e;
	NvU32 page_list[NDFLASH_CS_MAX];
	NvU32 i, blocksToErase = 1;
	NvU8  TagArea[NAND_SPARE_SIZE];
	unsigned int page_nr = 0;
	unsigned int chip_nr = 0;
	int ret = 0;
	NvU64 curAddr = instr->addr;

	for (i=0;i<NDFLASH_CS_MAX;i++)
		page_list[i] = -1;

	dev_dbg(info->dev, "%s: addr=0x%llx len=0x%llx\n", __func__,
		instr->addr, instr->len);

	if ((instr->addr + instr->len) > mtd->size) {
		dev_err(info->dev, "attempting to erase past end of device\n");
		instr->state = MTD_ERASE_FAILED;
		return -EINVAL;
	}

	if ((instr->addr | instr->len) & (mtd->erasesize - 1)) {
		dev_err(info->dev, "attempting non-aligned erase\n");
		instr->state = MTD_ERASE_FAILED;
		return -EINVAL;
	}

	mutex_lock(&info->lock);

	instr->state = MTD_ERASING;
	num_blocks = instr->len >> info->chip.block_shift;

	NvDdkNandResumeClocks(info->ddk);

	for ( ; num_blocks ; num_blocks--, curAddr += mtd->erasesize) {

		if (check_block_isbad(mtd, curAddr, TagArea))
			continue;

		/* get the nand flash chip number */
		chip_nr = curAddr >> info->chip.chip_shift;

		/* get the page number on the nand flash chip */
		page_nr = curAddr >> info->chip.page_shift;
		page_nr &= info->chip.page_mask;

		page_list[chip_nr] = page_nr;

		e = NvDdkNandErase(info->ddk, chip_nr, page_list, &blocksToErase);
		if (e!=NvSuccess) {
			ret = -EIO;
			instr->fail_addr = curAddr;
			dev_err(info->dev, "Erase failed for block 0x%llx\n",
				instr->addr >> info->chip.block_shift);
			break;
		}
		BUG_ON(blocksToErase != 1);
		page_list[chip_nr] = -1;
	}

	instr->state = (ret==0) ? MTD_ERASE_DONE : MTD_ERASE_FAILED;
	NvDdkNandSuspendClocks(info->ddk);
	mutex_unlock(&info->lock);
	mtd_erase_callback(instr);
	return ret;
}

static int tegra_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen,	uint8_t *buf)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	NvU32 page_list[NDFLASH_CS_MAX];
	NvU8  tag[NAND_SPARE_SIZE];

	memset(page_list, 0xff, sizeof(page_list));

	mutex_lock(&info->lock);
	if (retlen)
		*retlen = 0;

	NvDdkNandResumeClocks(info->ddk);
	while (len) {
		NvError e;
		NvU32 count = 1;
		unsigned int chip_nr;
		unsigned int page_nr;

		/* get the nand flash chip number */
		chip_nr = from >> info->chip.chip_shift;

		/* get the page number on the nand flash chip */
		page_nr = from >> info->chip.page_shift;
		page_nr &= info->chip.page_mask;

		page_list[chip_nr] = page_nr;

		if (check_block_isbad(mtd, from, tag)) {
			dev_err(info->dev, "reading from bad block\n");
			break;
		}

		e = NvDdkNandRead(info->ddk, chip_nr, page_list,
				  buf, NULL, &count, NV_FALSE);
		if (e != NvSuccess) {
			dev_err(info->dev, "failed to read from block\n");
			break;
		}

		page_list[chip_nr] = -1;

		BUG_ON(count != 1);
		count = min_t(unsigned int, len, mtd->writesize);
		from += count;
		len -= count;
		buf += count;
		if (retlen)
			*retlen += count;
	}

	NvDdkNandSuspendClocks(info->ddk);
	mutex_unlock(&info->lock);
	return (len) ? -EIO : 0;
}

static int do_read_oob(struct mtd_info *mtd, loff_t from,
	struct mtd_oob_ops *ops)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	uint8_t *oobbuf = ops->oobbuf;
	uint8_t *datbuf = ops->datbuf;
	uint32_t ooblen = oobbuf ? ops->ooblen : 0;
	NvError e;
	NvU32 page_list[NDFLASH_CS_MAX];
	NvU32 i, pageCount = 1;
	NvU8 tempSpareBuffer[NAND_SPARE_SIZE];
	NvU32 page_nr = 0;
	unsigned int chip_nr = 0;
	NvU64 curAddr = from;

	mutex_lock(&info->lock);

	if (check_block_isbad(mtd, curAddr, tempSpareBuffer)) {
		dev_err(info->dev, "Reading OOB data of bad block\n");
		mutex_unlock(&info->lock);
		return -EINVAL;
	}

	if (ooblen > mtd->oobavail)
		return -EINVAL;

	for (i=0;i<NDFLASH_CS_MAX;i++)
		page_list[i] = -1;

	/* get the nand flash chip number */
	chip_nr = (NvU32)(curAddr >> info->chip.chip_shift);

	/* get the page number on the nand flash chip */
	page_nr = (NvU32)((curAddr >> info->chip.page_shift) &
		info->chip.page_mask);

	NV_ASSERT(chip_nr < NDFLASH_CS_MAX);
	page_list[chip_nr] = page_nr;

	NvDdkNandResumeClocks(info->ddk);
	NV_CHECK_ERROR_CLEANUP(
		NvDdkNandRead(info->ddk, chip_nr, page_list,
			datbuf, tempSpareBuffer, &pageCount, NV_FALSE)
	);
	NvDdkNandSuspendClocks(info->ddk);

	NV_ASSERT(pageCount == 1);

	NvOsMemcpy(oobbuf, tempSpareBuffer, ooblen);

	ops->retlen = 0;
	ops->oobretlen = 0;

	mutex_unlock(&info->lock);
	return 0;

fail:
	dev_err(info->dev, "Failed to read OOB 0x%llx\n", curAddr);
	ops->retlen = 0;
	ops->oobretlen = 0;
	NvDdkNandSuspendClocks(info->ddk);

	mutex_unlock(&info->lock);
	return -EINVAL;
}

/* just does some parameter checking and calls do_read_oob */
static int tegra_nand_read_oob(struct mtd_info *mtd, loff_t from,
	struct mtd_oob_ops *ops)
{
	if (ops->datbuf && unlikely((from + ops->len) > mtd->size)) {
		pr_err("%s: Can't read past end of device.\n", __func__);
		return -EINVAL;
	}

	if (unlikely(ops->oobbuf && !ops->ooblen)) {
		pr_err("%s: Reading 0 bytes from OOB is meaningless\n",
			__func__);
		return -EINVAL;
	}

	if (unlikely(ops->mode != MTD_OOB_AUTO)) {
		if (ops->oobbuf && ops->datbuf) {
			pr_err("%s: can't read OOB + Data in non-AUTO mode.\n",
				__func__);
			return -EINVAL;
		}
		if ((ops->mode == MTD_OOB_RAW) && !ops->datbuf) {
			pr_err("%s: Raw mode only supports reading data.\n",
				__func__);
			return -EINVAL;
		}
	}

	NV_ASSERT(ops->oobbuf);

	return do_read_oob(mtd, from, ops);
}

static int tegra_nand_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const uint8_t *buf)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	NvU32 page_list[NDFLASH_CS_MAX];
	NvU8  tag[NAND_SPARE_SIZE];

	dev_dbg(info->dev, "%s to=0x%llx len=%x\n", __func__, to, len);

	mutex_lock(&info->lock);

	memset(page_list, 0xff, sizeof(page_list));
	*retlen = 0;

	NvDdkNandResumeClocks(info->ddk);
	while (len) {
		NvError e;
		NvU32 count = 1;
		unsigned int page_nr;
		unsigned int chip_nr;

		/* get the nand flash chip number */
		chip_nr = to >> info->chip.chip_shift;

		/* get the page number on the nand flash chip */
		page_nr = to >> info->chip.page_shift;
		page_nr &= info->chip.page_mask;

		page_list[chip_nr] = page_nr;

		if (check_block_isbad(mtd, to, tag)) {
			dev_err(info->dev, "writing to bad block\n");
			break;
		}

		e = NvDdkNandWrite(info->ddk, chip_nr, page_list,
				   buf, NULL, &count);

		if (e != NvSuccess) {
			dev_err(info->dev, "failed to write block (0x%x)\n",e);
			break;
		}

		page_list[chip_nr] = -1;

		BUG_ON(count != 1);
		count = min_t(unsigned int, len, mtd->writesize);
		to += count;
		*retlen += count;
		len -= count;
		buf += count;
	}

	NvDdkNandSuspendClocks(info->ddk);
	mutex_unlock(&info->lock);
	return (len) ? -EIO : 0;
}

static int do_write_oob(struct mtd_info *mtd, loff_t to,
	struct mtd_oob_ops *ops)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	uint8_t *oobbuf = ops->oobbuf;
	uint8_t *datbuf = ops->datbuf;
	uint32_t ooblen = oobbuf ? ops->ooblen : 0;
	NvU32 page_list[NDFLASH_CS_MAX];
	NvU32 count = 1;
	NvU8 spare_buff[NAND_SPARE_SIZE];
	unsigned int page_nr = 0;
	unsigned int chip_nr = 0;
	NvError e;

	mutex_lock(&info->lock);

	if (ooblen > mtd->oobavail)
		return -EINVAL;

	memset(spare_buff, 0xFF, mtd->oobavail);
	memcpy(spare_buff, oobbuf, ooblen);
	memset(page_list, 0xff, sizeof(page_list));

	/* get the nand flash chip number */
	chip_nr = to >> info->chip.chip_shift;

	/* get the page number on the nand flash chip */
	page_nr = to >> info->chip.page_shift;
	page_nr &= info->chip.page_mask;

	page_list[chip_nr] = page_nr;

	NvDdkNandResumeClocks(info->ddk);
	e = NvDdkNandWrite(info->ddk, chip_nr, page_list,
			   datbuf, spare_buff, &count);
	if (e != NvSuccess)
		dev_err(info->dev, "failed to write OOB area (0x%08x)\n", e);

	BUG_ON(count != 1);

	ops->retlen = 0;
	ops->oobretlen = 0;

	NvDdkNandSuspendClocks(info->ddk);
	mutex_unlock(&info->lock);
	return (e==NvSuccess) ? 0 : -EINVAL;
}

static int tegra_nand_write_oob(struct mtd_info *mtd, loff_t to,
	struct mtd_oob_ops *ops)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);

	if (unlikely(to & info->chip.column_mask)) {
		dev_err(info->dev, "Unaligned writes to OOB not supported\n");
		return -EINVAL;
	}

	if (unlikely(ops->oobbuf && !ops->ooblen)) {
		dev_err(info->dev, "Writing 0 bytes to OOB is meaningless\n");
		return -EINVAL;
	}

	return do_write_oob(mtd, to, ops);
}

static int tegra_nand_suspend(struct mtd_info *mtd)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);

	if (unlikely(!info->ddk)) {
		dev_err(info->dev, "invalid NAND DDK handle in suspend\n");
		return -ENODEV;
	} else if (NvDdkNandSuspend(info->ddk)!=NvSuccess) {
		dev_err(info->dev, "error suspending NAND controller\n");
		return -EIO;
	}
	return 0;
}

static void tegra_nand_resume(struct mtd_info *mtd)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);

	if (unlikely(!info->ddk)) {
		dev_err(info->dev, "invalid NAND DDK handle in resume\n");
	} else if (NvDdkNandResume(info->ddk) != NvSuccess) {
		dev_err(info->dev, "failed to resume\n");
	}
}

static int scan_bad_blocks(struct tegra_nand_info *info)
{
	struct mtd_info *mtd = &info->mtd;
	unsigned long num_blocks = mtd->size >> info->chip.block_shift;
	unsigned long block;
	int is_bad = 0;

	dev_vdbg(info->dev, "%s (blocks=%lu)\n", __func__, num_blocks);
	for (block = 0; block < num_blocks; ++block) {
		/* make sure the bit is cleared, meaning it's bad/unknown before
		 * we check. */
		clear_bit(block, info->bb_bitmap);

		is_bad = mtd->block_isbad(mtd, block << info->chip.block_shift);

		if (is_bad == 0) {
			set_bit(block, info->bb_bitmap);
		} else if (is_bad < 0) {
			dev_err(info->dev, "%s: fatal error %d\n", __func__, is_bad);
			return is_bad;
		}
	}
	return 0;
}

/* Scans for nand flash devices, identifies them, and fills in the
 * device info. */
static int tegra_nand_scan(struct mtd_info *mtd)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	uint32_t dev_id;
	uint32_t vendor_id;
	NvDdkNandDeviceInfo ddk_info;
	NvError e;

	BUG_ON(!s_hRmGlobal);
	e = NvDdkNandOpen(s_hRmGlobal, &info->ddk);
	if (e != NvSuccess) {
		dev_err(info->dev, "Failed to open NAND DDK\n");
		return -ENODEV;
	}

	e = NvDdkNandGetDeviceInfo(info->ddk, 0, &ddk_info);
	if (e != NvSuccess) {
		dev_err(info->dev, "Failed to get NAND device info\n");
		return -ENODEV;
	}

	/*Get some info from the nand driver */
	vendor_id = ddk_info.VendorId;
	dev_id = ddk_info.DeviceId;

	dev_info(info->dev, "%s: found NAND chip (Vendor = 0x%02x, "
		 "DevId = 0x%02x)\n", __func__, vendor_id, dev_id);

	info->chip.num_chips = ddk_info.NumberOfDevices;
	info->chip.chipsize = ddk_info.DeviceCapacityInKBytes * 1024;
	info->chip.page_in_block = ddk_info.PagesPerBlock;
	BUG_ON(ddk_info.NoOfBlocks & (ddk_info.NoOfBlocks-1));
	info->chip.block_mask = ddk_info.NoOfBlocks-1;

	mtd->size = info->chip.num_chips * info->chip.chipsize;

	/* page_size is same as read and write size */
	mtd->writesize = ddk_info.PageSize;
	info->chip.column_mask = mtd->writesize - 1;

	/* Note: See oob layout description of why we only support 2k pages. */
	if (mtd->writesize > 2048) {
		dev_err(info->dev, "large page (>2KB) devices not supported\n");
		return -EINVAL;
	} else if (mtd->writesize < 2048) {
		dev_err(info->dev, "small page (<2KB) devices not supported\n");
		return -EINVAL;
	}

	mtd->oobsize = NAND_SPARE_SIZE;
	mtd->oobavail = ddk_info.TagSize;

	/* data block size (erase size) (w/o spare) */
	mtd->erasesize = ddk_info.PagesPerBlock * ddk_info.PageSize;
	info->chip.block_shift = ffs(mtd->erasesize) - 1;

	/* used to select the appropriate chip/page in case multiple devices
	 * are connected */
	info->chip.chip_shift = ffs(info->chip.chipsize) - 1;
	info->chip.page_shift = ffs(mtd->writesize) - 1;
	info->chip.page_mask =
		(info->chip.chipsize >> info->chip.page_shift) - 1;

	/* now fill in the rest of the mtd fields */
	mtd->ecclayout = NULL;
	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;

	mtd->erase = tegra_nand_erase;
	mtd->lock = NULL;
	mtd->point = NULL;
	mtd->unpoint = NULL;
	mtd->read = tegra_nand_read;
	mtd->write = tegra_nand_write;
	mtd->read_oob = tegra_nand_read_oob;
	mtd->write_oob = tegra_nand_write_oob;

	mtd->resume = tegra_nand_resume;
	mtd->suspend = tegra_nand_suspend;
	mtd->block_isbad = tegra_nand_block_isbad;
	mtd->block_markbad = tegra_nand_block_markbad;
	NvDdkNandSuspendClocks(info->ddk);

	return 0;
}

static int __devinit tegra_nand_probe(struct platform_device *pdev)
{
	struct tegra_nand_info *info = NULL;
	struct tegra_nand_chip *chip = NULL;
	struct tegra_nand_platform *plat = pdev->dev.platform_data;
	struct mtd_info *mtd = NULL;
	int err = 0;

	dev_info(&pdev->dev, "%s: probing (%p)\n", __func__, pdev);

	info = kzalloc(sizeof(struct tegra_nand_info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "no memory for flash info\n");
		return -ENOMEM;
	}

	info->dev = &pdev->dev;

	mutex_init(&info->lock);

	chip = &info->chip;
	chip->priv = &info->mtd;
	chip->curr_chip = -1;

	mtd = &info->mtd;
	mtd->name = dev_name(&pdev->dev);
	mtd->priv = &info->chip;
	mtd->owner = THIS_MODULE;

	err = tegra_nand_scan(mtd);
	if (err)
		goto fail;

	/* alloc the bad block bitmap */
	info->bb_bitmap = kzalloc(BITS_TO_LONGS(mtd->size >>
		info->chip.block_shift) * sizeof(unsigned long), GFP_KERNEL);
	if (!info->bb_bitmap) {
		err = -ENOMEM;
		goto fail;
	}

	err = scan_bad_blocks(info);
	if (err)
		goto fail;

#ifdef CONFIG_MTD_PARTITIONS
	err = parse_mtd_partitions(mtd, part_probes, &info->parts, 0);

	if (err > 0) {
		err = add_mtd_partitions(mtd, info->parts, err);
	} else if (err <= 0 && plat && plat->nr_parts) {
		err = add_mtd_partitions(mtd, plat->parts, plat->nr_parts);
	} else
#endif
		err = add_mtd_device(mtd);

	if (err)
		goto fail;

	dev_set_drvdata(&pdev->dev, info);
	return 0;

fail:
	if (info) {
		if (info->bb_bitmap)
			kfree(info->bb_bitmap);
		if (info->ddk) {
			NvDdkNandSuspendClocks(info->ddk);
			NvDdkNandClose(info->ddk);
		}
		kfree(info);
	}

	return err;
}

static int __devexit tegra_nand_remove(struct platform_device *pdev)
{
	struct tegra_nand_info *info = dev_get_drvdata(&pdev->dev);

	dev_set_drvdata(&pdev->dev, NULL);

	if (info) {
		kfree(info->bb_bitmap);
		NvDdkNandSuspendClocks(info->ddk);
		NvDdkNandClose(info->ddk);
		info->ddk = NULL;
		kfree(info);
	}

	return 0;
}

static struct platform_driver tegra_nand_driver = {
	.probe		= tegra_nand_probe,
	.remove		= __devexit_p(tegra_nand_remove),
	.driver		= {
		.name	= "tegra_nand",
		.owner	= THIS_MODULE,
	},
};

static int __init tegra_nand_init(void)
{
	return platform_driver_register(&tegra_nand_driver);
}

static void __exit tegra_nand_exit(void)
{
	platform_driver_unregister(&tegra_nand_driver);
}

module_init(tegra_nand_init);
module_exit(tegra_nand_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
