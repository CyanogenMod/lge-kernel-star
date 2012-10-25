/*
 * driver/mfd/tps65090.c
 *
 * Core driver for TI TPS65090 PMIC family
 *
 * Copyright (C) 2012 NVIDIA Corporation
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
 *
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/i2c.h>

#include <linux/mfd/core.h>
#include <linux/mfd/tps65090.h>
#include <linux/regmap.h>

/* interrupt status registers */
#define TPS65090_INT_STS	0x0
#define TPS65090_INT_STS2	0x1

/* interrupt mask registers */
#define TPS65090_INT_MSK	0x2
#define TPS65090_INT_MSK2	0x3


enum irq_type {
	EVENT,
};

struct tps65090_irq_data {
	u8		mask_reg;
	u8		mask_pos;
	enum irq_type	type;
};

#define TPS65090_IRQ(_reg, _mask_pos, _type)	\
	{					\
		.mask_reg	= (_reg),	\
		.mask_pos	= (_mask_pos),	\
		.type		= (_type),	\
	}

static const struct tps65090_irq_data tps65090_irqs[] = {
	[0]		= TPS65090_IRQ(0, 0, EVENT),
	[1]		= TPS65090_IRQ(0, 1, EVENT),
	[2]		= TPS65090_IRQ(0, 2, EVENT),
	[3]		= TPS65090_IRQ(0, 3, EVENT),
	[4]		= TPS65090_IRQ(0, 4, EVENT),
	[5]		= TPS65090_IRQ(0, 5, EVENT),
	[6]		= TPS65090_IRQ(0, 6, EVENT),
	[7]		= TPS65090_IRQ(0, 7, EVENT),
	[8]		= TPS65090_IRQ(1, 0, EVENT),
	[9]		= TPS65090_IRQ(1, 1, EVENT),
	[10]		= TPS65090_IRQ(1, 2, EVENT),
	[11]		= TPS65090_IRQ(1, 3, EVENT),
	[12]		= TPS65090_IRQ(1, 4, EVENT),
	[13]		= TPS65090_IRQ(1, 5, EVENT),
	[14]		= TPS65090_IRQ(1, 6, EVENT),
	[15]		= TPS65090_IRQ(1, 7, EVENT),
};

struct tps65090 {
	struct mutex		lock;
	struct device		*dev;
	struct i2c_client	*client;
	struct regmap		*rmap;
	struct irq_chip		irq_chip;
	struct mutex		irq_lock;
	int			irq_base;
	u32			irq_en;
	u8			mask_cache[2];
	u8			mask_reg[2];
};
int tps65090_write(struct device *dev, int reg, uint8_t val)
{
	struct tps65090 *tps = dev_get_drvdata(dev);
	return regmap_write(tps->rmap, reg, val);
}
int tps65090_read(struct device *dev, int reg, uint8_t *val)
{
	int rval, ret = 0;
	struct tps65090 *tps = dev_get_drvdata(dev);
	ret = regmap_read(tps->rmap, reg, &rval);
	*val = rval;
	return 0;
}
EXPORT_SYMBOL_GPL(tps65090_read);

int tps65090_set_bits(struct device *dev, int reg, uint8_t bit_num)
{
	struct tps65090 *tps = dev_get_drvdata(dev);
	return regmap_update_bits_lazy(tps->rmap, reg, BIT(bit_num), ~0u);

}
EXPORT_SYMBOL_GPL(tps65090_set_bits);

int tps65090_clr_bits(struct device *dev, int reg, uint8_t bit_num)
{
	struct tps65090 *tps = dev_get_drvdata(dev);
	return regmap_update_bits_lazy(tps->rmap, reg, BIT(bit_num), 0u);
}
EXPORT_SYMBOL_GPL(tps65090_clr_bits);

static int __remove_subdev(struct device *dev, void *unused)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static int tps65090_remove_subdevs(struct tps65090 *tps65090)
{
	return device_for_each_child(tps65090->dev, NULL, __remove_subdev);
}

static void tps65090_irq_lock(struct irq_data *data)
{
	struct tps65090 *tps65090 = irq_data_get_irq_chip_data(data);

	mutex_lock(&tps65090->irq_lock);
}

static void tps65090_irq_mask(struct irq_data *irq_data)
{
	struct tps65090 *tps65090 = irq_data_get_irq_chip_data(irq_data);
	unsigned int __irq = irq_data->irq - tps65090->irq_base;
	const struct tps65090_irq_data *data = &tps65090_irqs[__irq];

	if (data->type == EVENT)
		tps65090->mask_reg[data->mask_reg] |= (1 << data->mask_pos);
	else
		tps65090->mask_reg[data->mask_reg] |= (3 << data->mask_pos);

	tps65090->irq_en &= ~(1 << __irq);
}

static void tps65090_irq_unmask(struct irq_data *irq_data)
{
	struct tps65090 *tps65090 = irq_data_get_irq_chip_data(irq_data);

	unsigned int __irq = irq_data->irq - tps65090->irq_base;
	const struct tps65090_irq_data *data = &tps65090_irqs[__irq];

	if (data->type == EVENT) {
		tps65090->mask_reg[data->mask_reg] &= ~(1 << data->mask_pos);
		tps65090->irq_en |= (1 << __irq);
	}
}

static void tps65090_irq_sync_unlock(struct irq_data *data)
{
	struct tps65090 *tps65090 = irq_data_get_irq_chip_data(data);
	int i;

	for (i = 0; i < ARRAY_SIZE(tps65090->mask_reg); i++) {
		if (tps65090->mask_reg[i] != tps65090->mask_cache[i]) {
			if (!WARN_ON(tps65090_write(tps65090->dev,
						TPS65090_INT_MSK + 2*i,
						tps65090->mask_reg[i])))
				tps65090->mask_cache[i] = tps65090->mask_reg[i];
		}
	}

	mutex_unlock(&tps65090->irq_lock);
}

static int tps65090_irq_set_type(struct irq_data *irq_data, unsigned int type)
{
	struct tps65090 *tps65090 = irq_data_get_irq_chip_data(irq_data);

	unsigned int __irq = irq_data->irq - tps65090->irq_base;
	const struct tps65090_irq_data *data = &tps65090_irqs[__irq];

	if (data->type != EVENT) /* add support for GPIO, if needed */
		return -EINVAL;

	return 0;
}

static irqreturn_t tps65090_irq(int irq, void *data)
{
	struct tps65090 *tps65090 = data;
	int ret = 0;
	u8 tmp[3];
	u8 int_ack;
	u32 acks, mask = 0;
	int i;

	for (i = 0; i < 3; i++) {
		ret = tps65090_read(tps65090->dev, TPS65090_INT_STS + 2*i,
				&tmp[i]);
		if (ret < 0) {
			dev_err(tps65090->dev,
				"failed to read interrupt status\n");
			return IRQ_NONE;
		}
		if (tmp[i]) {
			/* Ack only those interrupts which are enabled */
			int_ack = tmp[i] & (~(tps65090->mask_cache[i]));
			ret = tps65090_write(tps65090->dev,
					TPS65090_INT_STS + 2*i,	int_ack);
			if (ret < 0) {
				dev_err(tps65090->dev,
					"failed to write interrupt status\n");
				return IRQ_NONE;
			}
		}
	}

	acks = (tmp[2] << 16) | (tmp[1] << 8) | tmp[0];

	for (i = 0; i < ARRAY_SIZE(tps65090_irqs); i++) {
		if (tps65090_irqs[i].type == EVENT)
			mask = (1 << (tps65090_irqs[i].mask_pos
					+ tps65090_irqs[i].mask_reg*8));
		else
			return -EINVAL;
		if ((acks & mask) && (tps65090->irq_en & (1 << i)))
			handle_nested_irq(tps65090->irq_base + i);
	}
	return IRQ_HANDLED;
}

static int __devinit tps65090_irq_init(struct tps65090 *tps65090, int irq,
				int irq_base)
{
	int i, ret;

	if (!irq_base) {
		dev_warn(tps65090->dev, "No interrupt support on IRQ base\n");
		return -EINVAL;
	}

	mutex_init(&tps65090->irq_lock);

	tps65090->mask_reg[0] = 0xFF;
	tps65090->mask_reg[1] = 0xFF;
	for (i = 0; i < 2; i++) {
		tps65090->mask_cache[i] = tps65090->mask_reg[i];
		tps65090_write(tps65090->dev, TPS65090_INT_MSK + i,
				 tps65090->mask_cache[i]);
	}

	for (i = 0; i < 2; i++)
		tps65090_write(tps65090->dev, TPS65090_INT_STS + i, 0xff);

	tps65090->irq_base = irq_base;

	tps65090->irq_chip.name = "tps65090";
	tps65090->irq_chip.irq_mask = tps65090_irq_mask;
	tps65090->irq_chip.irq_unmask = tps65090_irq_unmask;
	tps65090->irq_chip.irq_bus_lock = tps65090_irq_lock;
	tps65090->irq_chip.irq_bus_sync_unlock = tps65090_irq_sync_unlock;
	tps65090->irq_chip.irq_set_type = tps65090_irq_set_type;

	for (i = 0; i < ARRAY_SIZE(tps65090_irqs); i++) {
		int __irq = i + tps65090->irq_base;
		irq_set_chip_data(__irq, tps65090);
		irq_set_chip_and_handler(__irq, &tps65090->irq_chip,
					 handle_simple_irq);
		irq_set_nested_thread(__irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(__irq, IRQF_VALID);
#endif
	}

	ret = request_threaded_irq(irq, NULL, tps65090_irq, IRQF_ONESHOT,
				"tps65090", tps65090);
	if (!ret) {
		device_init_wakeup(tps65090->dev, 1);
		enable_irq_wake(irq);
	}

	return ret;
}

static int __devinit tps65090_add_subdevs(struct tps65090 *tps65090,
					  struct tps65090_platform_data *pdata)
{
	struct tps65090_subdev_info *subdev;
	struct platform_device *pdev;
	int i, ret = 0;

	for (i = 0; i < pdata->num_subdevs; i++) {
		subdev = &pdata->subdevs[i];

		pdev = platform_device_alloc(subdev->name, subdev->id);

		pdev->dev.parent = tps65090->dev;
		pdev->dev.platform_data = subdev->platform_data;

		ret = platform_device_add(pdev);
		if (ret)
			goto failed;
	}
	return 0;

failed:
	tps65090_remove_subdevs(tps65090);
	return ret;
}
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
static void print_regs(const char *header, struct seq_file *s,
		struct i2c_client *client, int start_offset,
		int end_offset)
{
	int reg_val;
	int i;
	int ret;
	struct tps65090 *tps = s->private;
	seq_printf(s, "%s\n", header);
	for (i = start_offset; i <= end_offset; ++i) {
		ret = regmap_read(tps->rmap, i, &reg_val);
		if (ret >= 0)
			seq_printf(s, "Reg 0x%02x Value 0x%02x\n", i, reg_val);
	}
	seq_printf(s, "------------------\n");
}

static int dbg_tps_show(struct seq_file *s, void *unused)
{
	struct tps65090 *tps = s->private;
	struct i2c_client *client = tps->client;

	seq_printf(s, "TPS65090 Registers\n");
	seq_printf(s, "------------------\n");

	print_regs("All Regs",    s, client, 0x0, 24);
	return 0;
}

static int dbg_tps_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_tps_show, inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_tps_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void __init tps65090_debuginit(struct tps65090 *tps)
{
	(void)debugfs_create_file("tps65090", S_IRUGO, NULL,
			tps, &debug_fops);
}
#else
static void __init tps65090_debuginit(struct tps65090 *tpsi)
{
	return;
}
#endif

static const struct regmap_config tps65090_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int __devinit tps65090_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct tps65090_platform_data *pdata = client->dev.platform_data;
	struct tps65090 *tps65090;
	int ret;

	if (!pdata) {
		dev_err(&client->dev, "tps65090 requires platform data\n");
		return -ENOTSUPP;
	}

	tps65090 = kzalloc(sizeof(struct tps65090), GFP_KERNEL);
	if (tps65090 == NULL)
		return -ENOMEM;

	tps65090->client = client;
	tps65090->dev = &client->dev;
	i2c_set_clientdata(client, tps65090);

	mutex_init(&tps65090->lock);

	if (client->irq) {
		ret = tps65090_irq_init(tps65090, client->irq,
					pdata->irq_base);
		if (ret) {
			dev_err(&client->dev, "IRQ init failed: %d\n", ret);
			goto err_irq_init;
		}
	}

	ret = tps65090_add_subdevs(tps65090, pdata);
	if (ret) {
		dev_err(&client->dev, "add devices failed: %d\n", ret);
		goto err_add_devs;
	}

	tps65090->rmap = regmap_init_i2c(tps65090->client,
		&tps65090_regmap_config);
	if (IS_ERR(tps65090->rmap)) {
		dev_err(&client->dev, "regmap_init failed: %ld\n",
			PTR_ERR(tps65090->rmap));
		goto err_add_devs;
	};
	tps65090_debuginit(tps65090);

	return 0;

err_add_devs:
	if (client->irq)
		free_irq(client->irq, tps65090);
err_irq_init:
	kfree(tps65090);
	return ret;
}

static int __devexit tps65090_i2c_remove(struct i2c_client *client)
{
	struct tps65090 *tps65090 = i2c_get_clientdata(client);

	if (client->irq)
		free_irq(client->irq, tps65090);

	regmap_exit(tps65090->rmap);
	kfree(tps65090);
	return 0;
}
#ifdef CONFIG_PM
static int tps65090_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	if (client->irq)
		disable_irq(client->irq);
	return 0;
}

static int tps65090_i2c_resume(struct i2c_client *client)
{
	if (client->irq)
		enable_irq(client->irq);
	return 0;
}
#endif


static const struct i2c_device_id tps65090_id_table[] = {
	{ "tps65090", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, tps65090_id_table);

static struct i2c_driver tps65090_driver = {
	.driver	= {
		.name	= "tps65090",
		.owner	= THIS_MODULE,
	},
	.probe		= tps65090_i2c_probe,
	.remove		= __devexit_p(tps65090_i2c_remove),
#ifdef CONFIG_PM
	.suspend	= tps65090_i2c_suspend,
	.resume		= tps65090_i2c_resume,
#endif
	.id_table	= tps65090_id_table,
};

static int __init tps65090_init(void)
{
	return i2c_add_driver(&tps65090_driver);
}
subsys_initcall(tps65090_init);

static void __exit tps65090_exit(void)
{
	i2c_del_driver(&tps65090_driver);
}
module_exit(tps65090_exit);

MODULE_DESCRIPTION("TPS65090 core driver");
MODULE_LICENSE("GPL");
