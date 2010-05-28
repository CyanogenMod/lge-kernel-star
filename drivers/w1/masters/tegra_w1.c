/*
 * drivers/w1/masters/tegra_w1.c
 *
 * ONE WIRE (OWR) bus driver for internal OWR controllers in NVIDIA Tegra SoCs
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
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

#define NV_DEBUG 0

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/uaccess.h>

#include "../w1.h"
#include "../w1_int.h"
#include "../w1_log.h"

#include <mach/w1.h>
#include <mach/nvrm_linux.h>
#include <nvrm_owr.h>
#include <nvos.h>

struct tegra_w1_dev {
	NvRmOwrHandle rm_owr;
	unsigned int pin_map;
	struct platform_device *pdev;
	struct w1_bus_master bus_master;
};

static void tegra_w1_write_op(void *data, u8 a_byte,
			      NvRmOwrTransactionFlags flag)
{
	struct tegra_w1_dev *dev = data;
	NvRmOwrTransactionInfo info;
	NvError err;

	info.Flags = flag,
	info.NumBytes = 1;
	info.Address = 0;
	info.Offset = 0;

	err = NvRmOwrTransaction(dev->rm_owr, dev->pin_map,
				 &a_byte, 1, &info, 1);
	if (err != NvSuccess) {
		dev_err(&dev->pdev->dev, "%s (op:%s) failed 0x%x\r\n",
			__func__, (flag==NvRmOwr_WriteByte)?"byte":"bit", err);
	}
}

static u8 tegra_w1_read_op(void *data, NvRmOwrTransactionFlags flag)
{
	struct tegra_w1_dev *dev = data;
	NvRmOwrTransactionInfo info;
	NvError err;
	u8 byte;

	info.Flags = flag;
	info.NumBytes = 1;
	info.Address = 0;
	info.Offset = 0;

	err = NvRmOwrTransaction(dev->rm_owr, dev->pin_map,
				 &byte, info.NumBytes, &info, 1);
	if (err != NvSuccess) {
		dev_err(&dev->pdev->dev, "%s (op:%s) failed 0x%x\n",
			__func__, (flag==NvRmOwr_ReadByte)?"byte":"bit", err);
		return 0;
	}

	return byte;
}

static u8 tegra_w1_read_byte(void *data)
{
	return tegra_w1_read_op(data, NvRmOwr_ReadByte);
}

static void tegra_w1_write_byte(void *data, u8 a_byte)
{
	tegra_w1_write_op(data, a_byte, NvRmOwr_WriteByte);
}

static u8 tegra_w1_read_bit(void *data)
{
	return tegra_w1_read_op(data, NvRmOwr_ReadBit) & 1;
}

static void tegra_w1_write_bit(void *data, u8 bit)
{
	tegra_w1_write_op(data, bit&0x1, NvRmOwr_WriteBit);
}

/* Performs a write-0 or write-1 cycle and samples the level */
static u8 tegra_w1_touch_bit(void *data, u8 bit)
{
	if (bit) {
		return tegra_w1_read_bit(data);
	} else {
		tegra_w1_write_bit(data, 0);
		return 0;
	}
}

static u8 tegra_w1_reset_bus(void *data)
{
	struct tegra_w1_dev *dev = data;
	NvRmOwrTransactionInfo info;
	NvError err;
	u8 buffer[1];

	info.Flags = NvRmOwr_CheckPresence;
	info.NumBytes = 1;
	info.Address = 0;
	info.Offset = 0;

	err = NvRmOwrTransaction(dev->rm_owr, dev->pin_map,
				 buffer, info.NumBytes, &info, 1);
	if (err != NvSuccess) {
		dev_err(&dev->pdev->dev, "%s failed 0x%x\r\n", __func__, err);
		return 1;
	}

	return 0;
}

static int tegra_w1_probe(struct platform_device *pdev)
{
	struct tegra_w1_platform_data *plat = pdev->dev.platform_data;
	struct tegra_w1_dev *dev = NULL;
	int rc;

	dev_dbg(&pdev->dev, "%s: %p\n", __func__, pdev);
	if (!plat) {
		dev_err(&pdev->dev, "no platform data?\n");
		return -EINVAL;
	}

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "out of memory\n");
		return -ENOMEM;
	}

	if (NvRmOwrOpen(s_hRmGlobal, pdev->id, &dev->rm_owr) != NvSuccess) {
		dev_err(&pdev->dev, "unable to open RM One-Wire API\n");
		rc = -ENODEV;
		goto fail;
	}

	dev->pin_map = plat->pinmux;
	dev->pdev = pdev;

	dev->bus_master.data = dev;
	dev->bus_master.read_byte = tegra_w1_read_byte;
	dev->bus_master.write_byte = tegra_w1_write_byte;
	dev->bus_master.read_bit = tegra_w1_read_bit;
	dev->bus_master.write_bit = tegra_w1_write_bit;
	dev->bus_master.touch_bit = tegra_w1_touch_bit;
	dev->bus_master.reset_bus = tegra_w1_reset_bus;

	if (tegra_w1_reset_bus(dev)) {
		dev_err(&pdev->dev, "No device present\n");
		rc = -ENODEV;
		goto fail;
	}

	rc = w1_add_master_device(&dev->bus_master);
	if (rc) {
		dev_err(&pdev->dev, "failed to add device: %d\n", rc);
		goto fail;
	}

	platform_set_drvdata(pdev, dev);

	return 0;

fail:
	if (dev) {
		if (dev->rm_owr)
			NvRmOwrClose(dev->rm_owr);
		kfree(dev);
	}
	return rc;
}

static int tegra_w1_remove(struct platform_device *pdev)
{
	struct tegra_w1_dev *dev = platform_get_drvdata(pdev);

	NvRmOwrClose(dev->rm_owr);
	w1_remove_master_device(&dev->bus_master);
	kfree(dev);
	return 0;
}

static int tegra_w1_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int tegra_w1_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver tegra_w1_driver = {
	.probe   = tegra_w1_probe,
	.remove  = tegra_w1_remove,
	.suspend = tegra_w1_suspend,
	.resume  = tegra_w1_resume,
	.driver  = {
		.name  = "tegra_w1",
		.owner = THIS_MODULE,
	},
};

static int __init tegra_w1_init(void)
{
	return platform_driver_register(&tegra_w1_driver);
}
module_init(tegra_w1_init);

static void __exit tegra_w1_exit(void)
{
	platform_driver_unregister(&tegra_w1_driver);
}
module_exit(tegra_w1_exit);
