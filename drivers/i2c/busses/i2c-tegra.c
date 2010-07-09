/*
 * drivers/i2c/busses/i2c-tegra.c
 *
 * I2C bus driver for internal I2C controllers in NVIDIA Tegra SoCs
 *
 * Copyright (C) 2009-2010 NVIDIA Corporation
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

#include <linux/err.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/uaccess.h>

#include <mach/nvrm_linux.h>
#include <mach/i2c.h>
#include <nvodm_modules.h>
#include <nvrm_i2c.h>

struct tegra_i2c_dev;

struct tegra_i2c_bus {
	struct tegra_i2c_dev *i2c_dev;
	unsigned int pinmux;
	unsigned long rate;
	struct mutex *bus_lock;
	struct i2c_adapter adapter;
};

struct tegra_i2c_dev {
	struct device *dev;
	NvRmI2cHandle rm_i2c;
	int bus_count;
	struct tegra_i2c_bus busses[1];
};

static int tegra_i2c_lock(struct tegra_i2c_bus *i2c_bus)
{
	if (likely(i2c_bus->bus_lock == &i2c_bus->adapter.bus_lock))
		return 0;

	if (in_atomic() || irqs_disabled()) {
		if (!mutex_trylock(i2c_bus->bus_lock))
			return -EAGAIN;
	} else {
		mutex_lock_nested(i2c_bus->bus_lock, i2c_bus->adapter.level);
	}
	return 0;
}

static void tegra_i2c_unlock(struct tegra_i2c_bus *i2c_bus)
{
	if (likely(i2c_bus->bus_lock == &i2c_bus->adapter.bus_lock))
		return;

	mutex_unlock(i2c_bus->bus_lock);
}

#define kzalloc_stack(num, buff)					\
	({								\
		typeof(buff[0]) *bptr = buff;				\
		if (num > ARRAY_SIZE(buff))				\
			bptr = kmalloc(sizeof(*bptr)*num, GFP_ATOMIC);	\
		memset(bptr, 0, num*sizeof(*bptr));			\
		bptr;							\
	})

#define kfree_stack(ptr, buff)				\
	do {						\
		if (ptr && ptr != buff)			\
			kfree(ptr);			\
	} while (0);

static int tegra_i2c_xfer(struct i2c_adapter *adap,
	struct i2c_msg msgs[], int num)
{
	struct tegra_i2c_bus *i2c_bus = i2c_get_adapdata(adap);
	struct tegra_i2c_dev *i2c_dev = i2c_bus->i2c_dev;
	NvRmI2cTransactionInfo onstack_tx[3];
	NvRmI2cTransactionInfo *rm_tx = NULL;
	NvU8 onstack_data[32];	
	NvU8 *data = NULL, *cpy;
	unsigned int len = 0;
	int rc = 0;
	int i;

	for(i=0; i<num; i++)
		len += msgs[i].len;

	if (tegra_i2c_lock(i2c_bus))
		return -EAGAIN;

	i = 0;

	rm_tx = kzalloc_stack(num, onstack_tx);
	if (!rm_tx) {
		dev_err(i2c_dev->dev, "allocate transaction array failed\n");
		rc = -ENOMEM;
		goto clean;
	}

	if (len) {
		data = kzalloc_stack(len, onstack_data);
		if (!data) {
			dev_err(i2c_dev->dev, "allocate data array failed\n");
			rc = -ENOMEM;
			goto clean;
		}
	}

	cpy = data;

	for (i=0; i<num; i++) {
		if (msgs[i].flags & I2C_M_NOSTART) {
			rm_tx[i].Flags |= NVRM_I2C_NOSTOP;
		}
		if (msgs[i].flags & I2C_M_IGNORE_NAK) {
			rm_tx[i].Flags |= NVRM_I2C_NOACK;
		}
		if (msgs[i].flags & I2C_M_RD) {
			rm_tx[i].Flags |= NVRM_I2C_READ;
		} else {
			rm_tx[i].Flags |= NVRM_I2C_WRITE;
			memcpy(cpy, msgs[i].buf, msgs[i].len);
		}
		cpy += msgs[i].len;
		rm_tx[i].NumBytes = msgs[i].len;
		rm_tx[i].Address = msgs[i].addr << 1;
		rm_tx[i].Is10BitAddress = !!(msgs[i].flags & I2C_M_TEN);
	}

	switch (NvRmI2cTransaction(i2c_dev->rm_i2c, i2c_bus->pinmux, 1000,
				   i2c_bus->rate/1000, data, len, rm_tx, num)) {
	case NvSuccess:
		break;
	case NvError_I2cDeviceNotFound:
		dev_err(i2c_dev->dev, "no slave found on adapter %d at "
			"address 0x%x\n", i2c_bus->adapter.nr, msgs[0].addr);
		rc = -ENXIO;
		break;
	case NvError_I2cReadFailed:
	case NvError_I2cWriteFailed:
		dev_err(i2c_dev->dev, "read/write failed on adapter %d at "
			"address 0x%x\n", i2c_bus->adapter.nr, msgs[0].addr);
		rc = -EIO;
		break;
	case NvError_Timeout:
		dev_err(i2c_dev->dev, "i2c timeout on adapter %d at "
			"address 0x%x\n", i2c_bus->adapter.nr, msgs[0].addr);
		rc = -EIO;
		break;
	default:
		dev_err(i2c_dev->dev, "unknown error on adapter %d at "
			"address 0x%x\n", i2c_bus->adapter.nr, msgs[0].addr);
		rc = -ENXIO;
		break;
	}

	num = i;

	cpy = data;
	for (i=0; i<num; i++) {
		if (rm_tx[i].Flags & NVRM_I2C_READ)
			memcpy(msgs[i].buf, cpy, msgs[i].len);
		cpy += msgs[i].len;
	}

clean:
	kfree_stack(data, onstack_data);
	kfree_stack(rm_tx, onstack_tx);
	tegra_i2c_unlock(i2c_bus);
	return i;
}

static u32 tegra_i2c_func(struct i2c_adapter *adap)
{
	/* FIXME: For now keep it simple and don't support protocol mangling
	   features */
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm tegra_i2c_algo = {
	.master_xfer	= tegra_i2c_xfer,
	.functionality	= tegra_i2c_func,
};

static int tegra_i2c_probe(struct platform_device *pdev)
{
	struct tegra_i2c_dev *i2c_dev;
	struct tegra_i2c_plat_parms *plat = pdev->dev.platform_data;
	int ret = 0;
	int nbus;
	NvError e;
	int i = 0;

	dev_dbg(&pdev->dev, "%s: %p\n", __func__, plat);

	if (!plat) {
		dev_err(&pdev->dev, "no platform data?\n");
		return -ENODEV;
	}

	WARN_ON(plat->bus_count > TEGRA_I2C_MAX_BUS);
	nbus = min(TEGRA_I2C_MAX_BUS, plat->bus_count);
	BUG_ON(!nbus);

	i2c_dev = kzalloc(sizeof(*i2c_dev) +
			  (nbus-1)*sizeof(struct tegra_i2c_bus), GFP_KERNEL);

	if (!i2c_dev) {
		dev_err(&pdev->dev, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, i2c_dev);
	if (plat->is_dvc) {
		e = NvRmI2cOpen(s_hRmGlobal, NvOdmIoModule_I2c_Pmu,
				0, &i2c_dev->rm_i2c);
	} else {
		e = NvRmI2cOpen(s_hRmGlobal, NvOdmIoModule_I2c,
				pdev->id, &i2c_dev->rm_i2c);
	}
	if (e != NvSuccess) {
		dev_err(&pdev->dev, "failed to open RM I2C\n");
		kfree(i2c_dev);
		return -ENODEV;
	}

	i2c_dev->dev = &pdev->dev;

	for (i=0; i<nbus; i++) {
		struct tegra_i2c_bus *i2c_bus = &i2c_dev->busses[i];

		i2c_bus->i2c_dev = i2c_dev;
		i2c_bus->pinmux = plat->bus_mux[i];
		i2c_bus->rate = plat->bus_clk[i];
		i2c_bus->bus_lock = &i2c_dev->busses[0].adapter.bus_lock;

		i2c_bus->adapter.algo = &tegra_i2c_algo;
		i2c_set_adapdata(&i2c_bus->adapter, i2c_bus);
		i2c_bus->adapter.owner = THIS_MODULE;
		i2c_bus->adapter.class = I2C_CLASS_HWMON;
		strlcpy(i2c_bus->adapter.name, "Tegra I2C adapter",
			sizeof(i2c_bus->adapter.name));
		i2c_bus->adapter.dev.parent = &pdev->dev;
		i2c_bus->adapter.nr = plat->adapter_nr + i;
		ret = i2c_add_numbered_adapter(&i2c_bus->adapter);
		if (ret) {
			dev_err(&pdev->dev, "failed to add adapter %d\n", i);
			goto err;
		}
		i2c_dev->bus_count++;
	}

	return 0;

err:
	while (i2c_dev->bus_count--)
		i2c_del_adapter(&i2c_dev->busses[i2c_dev->bus_count].adapter);

	kfree(i2c_dev);
	return ret;
}

static int tegra_i2c_remove(struct platform_device *pdev)
{
	struct tegra_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	while (i2c_dev->bus_count--)
		i2c_del_adapter(&i2c_dev->busses[i2c_dev->bus_count].adapter);

	NvRmI2cClose(i2c_dev->rm_i2c);
	kfree(i2c_dev);
	return 0;
}

static int tegra_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* nothing to do; handled by RM */
	return 0;
}

static int tegra_i2c_resume(struct platform_device *pdev)
{
	/* nothing to do; handled by RM */
	return 0;
}

static struct platform_driver tegra_i2c_driver = {
	.probe   = tegra_i2c_probe,
	.remove  = tegra_i2c_remove,
	.suspend = tegra_i2c_suspend,
	.resume  = tegra_i2c_resume,
	.driver  = {
		.name  = "tegra_i2c",
		.owner = THIS_MODULE,
	},
};

/* I2C may be needed to bring up other drivers */
static int __init tegra_i2c_init(void)
{
	return platform_driver_register(&tegra_i2c_driver);
}
module_init(tegra_i2c_init);

static void __exit tegra_i2c_exit(void)
{
	platform_driver_unregister(&tegra_i2c_driver);
}
module_exit(tegra_i2c_exit);
