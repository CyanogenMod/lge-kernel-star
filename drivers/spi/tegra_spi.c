/*
 * drivers/spi/tegra_spi.c
 *
 * SPI bus driver for NVIDIA Tegra SoCs, based on NvRm APIs
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
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

#define NV_DEBUG 0

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/workqueue.h>


#include <mach/spi.h>
#include <mach/nvrm_linux.h>
#include <nvrm_spi.h>
#include <nvodm_query.h>

#include <rm_spi_slink.h>

struct tegra_spi {
	NvRmSpiHandle		rm_spi;
	NvU32			pinmux;
	NvU32			Mode;
	struct list_head	msg_queue;
	spinlock_t		lock;
	struct work_struct	work;
	struct workqueue_struct	*queue;
}; 

/* Only these signaling mode are supported */
#define NV_SUPPORTED_MODE_BITS (SPI_CPOL | SPI_CPHA)

static int tegra_spi_setup(struct spi_device *device)
{
	struct tegra_spi *spi;

	spi = spi_master_get_devdata(device->master);

	if (device->mode & ~NV_SUPPORTED_MODE_BITS) {
		dev_dbg(&device->dev, "setup: unsupported mode bits 0x%x\n",
			device->mode & ~NV_SUPPORTED_MODE_BITS);
	}

	spi->Mode = device->mode & NV_SUPPORTED_MODE_BITS;
	switch (spi->Mode) {
	case SPI_MODE_0:
		spi->Mode = NvOdmQuerySpiSignalMode_0;
		break;
	case SPI_MODE_1:
		spi->Mode = NvOdmQuerySpiSignalMode_1;
		break;
	case SPI_MODE_2:
		spi->Mode = NvOdmQuerySpiSignalMode_2;
		break;
	case SPI_MODE_3:
		spi->Mode = NvOdmQuerySpiSignalMode_3;
		break;
	}

	if (device->bits_per_word == 0)
		device->bits_per_word = 8;

	NvRmSpiSetSignalMode(spi->rm_spi, device->chip_select, spi->Mode);
	return 0;
}

static int tegra_spi_transfer(struct spi_device *device,
	struct spi_message *msg)
{
	struct tegra_spi *spi;

	if (unlikely(list_empty(&msg->transfers) || !device->max_speed_hz))
		return -EINVAL;

	/* FIXME validate the msg */

	spi = spi_master_get_devdata(device->master);

	/* Add the message to the queue and signal the worker thread */
	spin_lock(&spi->lock);
	list_add_tail(&msg->queue, &spi->msg_queue);
	queue_work(spi->queue, &spi->work);
	spin_unlock(&spi->lock);

	return 0;
}

static void tegra_spi_cleanup(struct spi_device *device)
{
	return;
}

static int tegra_spi_do_message(struct tegra_spi *spi, struct spi_message *m)
{
	NvRmSpiTransactionInfo trans[64];
	struct spi_transfer *t;
	unsigned int len = 0;
	int i = 0;

	list_for_each_entry(t, &m->transfers, transfer_list) {
		if (i==ARRAY_SIZE(trans))
			return -EIO;

		if (t->len && !t->tx_buf && !t->rx_buf)
			return -EINVAL;

		if (t->cs_change) {
			WARN_ON_ONCE(1);
			return -EIO;
		}

		if (t->len) {
			trans[i].rxBuffer = t->rx_buf;
			trans[i].txBuffer = (NvU8*)t->tx_buf;
			trans[i].len = t->len;
			len += t->len;
		}

		i++;
	}

	if (!i)
		return 0;

	m->actual_length += len;
	NvRmSpiMultipleTransactions(spi->rm_spi, spi->pinmux,
		m->spi->chip_select, m->spi->max_speed_hz / 1000,
		m->spi->bits_per_word, trans, i);

	return 0;
}

static void tegra_spi_workerthread(struct work_struct *w)
{
	struct tegra_spi *spi;

	spi = container_of(w, struct tegra_spi, work);

	spin_lock(&spi->lock);

	while (!list_empty(&spi->msg_queue)) {
		struct spi_message *m;

		m = container_of(spi->msg_queue.next, struct spi_message, queue);
		list_del_init(&m->queue);
		spin_unlock(&spi->lock);

		if (!m->spi) {
			WARN_ON(1);
			return;
		}
		m->status = tegra_spi_do_message(spi, m);
		m->complete(m->context);

		spin_lock(&spi->lock);
	}

	spin_unlock(&spi->lock);
}

static int __init tegra_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct tegra_spi *spi;
	struct tegra_spi_platform_data *plat = pdev->dev.platform_data;
	int status= 0;
	NvError e;

	master = spi_alloc_master(&pdev->dev, sizeof(*spi));
	if (IS_ERR_OR_NULL(master)) {
		dev_err(&pdev->dev, "master allocation failed\n");
		return -ENOMEM;
	}

	master->setup = tegra_spi_setup;
	master->transfer = tegra_spi_transfer;
	master->cleanup = tegra_spi_cleanup;
	master->num_chipselect = 4;
	master->bus_num = pdev->id;
	master->mode_bits = NV_SUPPORTED_MODE_BITS;

	dev_set_drvdata(&pdev->dev, master);
	spi = spi_master_get_devdata(master);

	spi->pinmux = plat->pinmux;

	if (plat->is_slink) { 
		e = NvRmSpiOpen(s_hRmGlobal, NvOdmIoModule_Spi,
				pdev->id, NV_TRUE, &spi->rm_spi);
	} else {
		e = NvRmSpiOpen(s_hRmGlobal, NvOdmIoModule_Sflash,
				0, NV_TRUE, &spi->rm_spi);
	}
	if (e != NvSuccess) {
		dev_err(&pdev->dev, "NvRmSpiOpen returned 0x%x\n", e);
		status = -ENODEV;
		goto spi_open_failed;
	}

	spi->queue = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!spi->queue) {
		dev_err(&pdev->dev, "Failed to create work queue\n");
		goto workQueueCreate_failed;
	}

	INIT_WORK(&spi->work, tegra_spi_workerthread);

	spin_lock_init(&spi->lock);
	INIT_LIST_HEAD(&spi->msg_queue);

	status = spi_register_master(master);
	if (status < 0) {
		dev_err(&pdev->dev, "spi_register_master failed %d\n", status);
		goto spi_register_failed;
	}

	return status;
	
spi_register_failed:
	destroy_workqueue(spi->queue);
workQueueCreate_failed:
	NvRmSpiClose(spi->rm_spi);
spi_open_failed:
	spi_master_put(master);
	return status;
}
	
static int tegra_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master;
	struct tegra_spi *spi;

	master = dev_get_drvdata(&pdev->dev);
	spi = spi_master_get_devdata(master);

	spi_unregister_master(master);
	NvRmSpiClose(spi->rm_spi);
	destroy_workqueue(spi->queue);

	return 0;
}

static struct platform_driver tegra_spi_driver = {
	.probe = tegra_spi_probe,
	.remove = tegra_spi_remove,
	.driver	= {
		.name	= "tegra_spi",
		.owner	= THIS_MODULE,
	},
};

static int __init tegra_spi_init(void)
{
	return platform_driver_register(&tegra_spi_driver);
}
module_init(tegra_spi_init);

static void __exit tegra_spi_exit(void)
{
	platform_driver_unregister(&tegra_spi_driver);
}
module_exit(tegra_spi_exit);
