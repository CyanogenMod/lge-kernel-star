/*
 * drivers/input/keyboard/tegra-kbc.c
 *
 * Keyboard class input driver for the NVIDIA Tegra SoC internal matrix
 * keyboard controller
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

//#define RM_SUPPORT

#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <mach/kbc.h>

#ifdef RM_SUPPORT
#include <linux/tegra_devices.h>
#include <mach/nvrm_linux.h>
#include "nvrm_power.h"
#endif

#define KBC_CONTROL_0	0
#define KBC_INT_0	4
#define KBC_ROW_CFG0_0	8
#define KBC_COL_CFG0_0	0x18
#define KBC_RPT_DLY_0	0x2c
#define KBC_KP_ENT0_0	0x30
#define KBC_KP_ENT1_0	0x34
#define KBC_ROW0_MASK_0	0x38

#define res_size(res)	((res)->end - (res)->start + 1)

struct tegra_kbc {
	void __iomem *mmio;
	struct input_dev *idev;
	int irq;
	spinlock_t lock;
	unsigned int repoll_time;
	struct tegra_kbc_plat *pdata;
	struct work_struct key_repeat;
#ifdef RM_SUPPORT
	NvU32 client_id;
#else
	struct clk *clk;
	struct regulator *reg;
#endif
};

#ifdef RM_SUPPORT
static int alloc_resource(struct tegra_kbc *kbc, struct platform_device *pdev)
{
	NvError e = NvRmPowerRegister(s_hRmGlobal, NULL, &kbc->client_id);
	if (e!=NvSuccess) return -ENXIO;
	return 0;
}

static void free_resource(struct tegra_kbc *kbc)
{
	NvRmPowerUnRegister(s_hRmGlobal, kbc->client_id);
}

static void enable_power(struct tegra_kbc *kbc)
{
	NvError e;
	e = NvRmPowerVoltageControl(s_hRmGlobal,
		NVRM_MODULE_ID(NvRmModuleID_Kbc, 0),
		kbc->client_id, NvRmVoltsUnspecified,
		NvRmVoltsUnspecified, NULL, 0, NULL);
	BUG_ON(e!=NvSuccess);
}

static void disable_power(struct tegra_kbc *kbc)
{
	NvError e;
	e = NvRmPowerVoltageControl(s_hRmGlobal,
		NVRM_MODULE_ID(NvRmModuleID_Kbc, 0),
		kbc->client_id, NvRmVoltsOff,
		NvRmVoltsOff, NULL, 0, NULL);
	BUG_ON(e!=NvSuccess);
}

static void enable_clock(struct tegra_kbc *kbc)
{
	NvError e;
	e = NvRmPowerModuleClockControl(s_hRmGlobal,
		NVRM_MODULE_ID(NvRmModuleID_Kbc,0), 0, NV_TRUE);
	BUG_ON(e!=NvSuccess);
	NvRmModuleReset(s_hRmGlobal, NVRM_MODULE_ID(NvRmModuleID_Kbc, 0));
}

static void disable_clock(struct tegra_kbc *kbc)
{
	NvRmPowerModuleClockControl(s_hRmGlobal,
		NVRM_MODULE_ID(NvRmModuleID_Kbc,0), 0, NV_FALSE);
}

#else
static int alloc_resource(struct tegra_kbc *kbc, struct platform_device *pdev)
{
	kbc->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(kbc->clk)) {
		int err;
		dev_err(&pdev->dev, "failed to get keypad clock\n");
		err = PTR_ERR(kbc->clk);
		kbc->clk = NULL;
		return err;
	}
	kbc->reg = regulator_get(&pdev->dev, "Vcc");
	if (IS_ERR(kbc->reg)) {
		dev_err(&pdev->dev, "no regulator support\n");
		kbc->reg = NULL;
	}
	return 0;
}

static void free_resource(struct tegra_kbc *kbc)
{
	if (kbc->clk) clk_put(kbc->clk);
	if (kbc->reg) regulator_put(kbc->reg);
}

static void enable_power(struct tegra_kbc *kbc)
{
	if (kbc->reg) regulator_enable(kbc->reg);
}

static void disable_power(struct tegra_kbc *kbc)
{
	if (kbc->reg) regulator_disable(kbc->reg);
}

static void enable_clock(struct tegra_kbc *kbc)
{
	clk_enable(kbc->clk);
}

static void disable_clock(struct tegra_kbc *kbc)
{
	clk_disable(kbc->clk);
}
#endif

static int tegra_kbc_keycode(const struct tegra_kbc *kbc, int r, int c) {
	unsigned int i = kbc_indexof(r,c);

	if (kbc->pdata->keymap)
		return kbc->pdata->keymap[i];

	return i;
}

#ifdef CONFIG_PM
static int tegra_kbc_open(struct input_dev *dev);
static void tegra_kbc_close(struct input_dev *dev);
static void tegra_kbc_setup_wakekeys(struct tegra_kbc *kbc, bool filter);

static int tegra_kbc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_kbc *kbc = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev)) {
		tegra_kbc_setup_wakekeys(kbc, true);
		enable_irq_wake(kbc->irq);
		disable_power(kbc);
	} else {
		tegra_kbc_close(kbc->idev);
	}

	return 0;
}

static int tegra_kbc_resume(struct platform_device *pdev)
{
	struct tegra_kbc *kbc = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev)) {
		enable_power(kbc);
		disable_irq_wake(kbc->irq);
		tegra_kbc_setup_wakekeys(kbc, false);
	} else if (kbc->idev->users)
		return tegra_kbc_open(kbc->idev);

	return 0;
}
#endif

static void tegra_kbc_report_keys(struct tegra_kbc *kbc, int *fifo)
{
	int curr_fifo[KBC_MAX_KPENT];
	u32 kp_ent_val[(KBC_MAX_KPENT*8 + 3) / 4];
	u32 *kp_ents = kp_ent_val;
	u32 kp_ent;
	unsigned long flags;
	int i, j, valid=0;

	local_irq_save(flags);
	for (i=0; i<ARRAY_SIZE(kp_ent_val); i++)
		kp_ent_val[i] = readl(kbc->mmio + KBC_KP_ENT0_0 + (i*4));
	local_irq_restore(flags);

	for (i=0; i<KBC_MAX_KPENT; i++) {
		if (!(i&3)) kp_ent=*kp_ents++;

		if (kp_ent & 0x80) {
			int c = kp_ent & 0x7;
			int r = (kp_ent >> 3) & 0xf;
			int k = tegra_kbc_keycode(kbc, r, c);
			if (likely(k!=-1)) curr_fifo[valid++] = k;
		}
		kp_ent >>= 8;
	}

	for (i=0; i<KBC_MAX_KPENT; i++) {
		if (fifo[i]==-1) continue;
		for (j=0; j<valid; j++) {
			if (curr_fifo[j] == fifo[i]) {
				curr_fifo[j] = -1;
				break;
                        }
		}
		if (j==valid) {
			input_report_key(kbc->idev, fifo[i], 0);
			fifo[i] = -1;
		}
	}
	for (j=0; j<valid; j++) {
		if (curr_fifo[j]==-1) continue;
		for (i=0; i<KBC_MAX_KPENT; i++) {
			if (fifo[i]==-1) break;
		}
		if (i!=KBC_MAX_KPENT) {
			fifo[i] = curr_fifo[j];
			input_report_key(kbc->idev, fifo[i], 1);
		} else
			WARN_ON(1);
	}
}

static void tegra_kbc_key_repeat(struct work_struct *work)
{
	struct tegra_kbc *kbc;
	unsigned long flags;
	u32 val;
	int fifo[KBC_MAX_KPENT];
	int i;

	kbc = container_of(work, struct tegra_kbc, key_repeat);
	for (i=0; i<ARRAY_SIZE(fifo); i++) fifo[i] = -1;

	while (1) {
		val = (readl(kbc->mmio + KBC_INT_0) >> 4) & 0xf;
		if (!val) {
			/* release any pressed keys and exit the loop */
			for (i=0; i<ARRAY_SIZE(fifo); i++) {
				if (fifo[i]==-1) continue;
				input_report_key(kbc->idev, fifo[i], 0);
			}
			break;
		}
		tegra_kbc_report_keys(kbc, fifo);
		/* FIXME: why is this here? */
		msleep((val==1) ? kbc->repoll_time : 1);
	}

	spin_lock_irqsave(&kbc->lock, flags);
	val = readl(kbc->mmio + KBC_CONTROL_0);
	val |= (1<<3);
	writel(val, kbc->mmio + KBC_CONTROL_0);
	spin_unlock_irqrestore(&kbc->lock, flags);
}

static void tegra_kbc_close(struct input_dev *dev)
{
	struct tegra_kbc *kbc = input_get_drvdata(dev);
	unsigned long flags;
	u32 val;

	val = readl(kbc->mmio + KBC_CONTROL_0);
	val &= ~1;
	writel(val, kbc->mmio + KBC_CONTROL_0);
	spin_unlock_irqrestore(&kbc->lock, flags);

	disable_clock(kbc);
	disable_power(kbc);
}

#ifdef CONFIG_ARCH_TEGRA_1x_SOC
#define tegra_kbc_setup_wakekeys(kbc, filter) do { } while (0)
#else
static void tegra_kbc_setup_wakekeys(struct tegra_kbc *kbc, bool filter)
{
	int i;
	unsigned int rst_val;

	BUG_ON(kbc->pdata->wake_cnt > KBC_MAX_KEY);
	rst_val = (filter && kbc->pdata->wake_cnt) ? ~0 : 0;

	for (i=0; i<KBC_MAX_ROW; i++)
		writel(rst_val, kbc->mmio+KBC_ROW0_MASK_0+i*4);

	if (filter) {
		for (i=0; i<kbc->pdata->wake_cnt; i++) {
			u32 val, addr;
			addr = kbc->pdata->wake_cfg[i].row*4 + KBC_ROW0_MASK_0;
			val = readl(kbc->mmio + addr);
			val &= ~(1<<kbc->pdata->wake_cfg[i].col);
			writel(val, kbc->mmio + addr);
		}
	}
}
#endif

static void tegra_kbc_config_pins(struct tegra_kbc *kbc)
{
	const struct tegra_kbc_plat *pdata = kbc->pdata;
	int i;

	for (i=0; i<KBC_MAX_GPIO; i++) {
		u32 row_cfg, col_cfg;
		u32 r_shift = 5 * (i%6);
		u32 c_shift = 4 * (i%8);
		u32 r_mask = 0x1f << r_shift;
		u32 c_mask = 0xf << c_shift;
		u32 r_offs = (i / 6) * 4 + KBC_ROW_CFG0_0;
		u32 c_offs = (i / 8) * 4 + KBC_COL_CFG0_0;

		row_cfg = readl(kbc->mmio + r_offs);
		col_cfg = readl(kbc->mmio + c_offs);

		row_cfg &= ~r_mask;
		col_cfg &= ~c_mask;

		if (pdata->pin_cfg[i].is_row)
			row_cfg |= ((pdata->pin_cfg[i].num<<1) | 1) << r_shift;
		else if (pdata->pin_cfg[i].is_col)
			col_cfg |= ((pdata->pin_cfg[i].num<<1) | 1) << c_shift;

		writel(row_cfg, kbc->mmio + r_offs);
		writel(col_cfg, kbc->mmio + c_offs);
	}
}

static int tegra_kbc_open(struct input_dev *dev)
{
	struct tegra_kbc *kbc = input_get_drvdata(dev);
	unsigned long flags;
	u32 val = 0;

	enable_power(kbc);
	enable_clock(kbc);

	tegra_kbc_config_pins(kbc);
	tegra_kbc_setup_wakekeys(kbc, false);

	/* atomically clear out any remaining entries in the key FIFO
	 * and enable keyboard interrupts */
	spin_lock_irqsave(&kbc->lock, flags);
	val = readl(kbc->mmio + KBC_INT_0);
	val >>= 4;
	if (val) {
		val = readl(kbc->mmio + KBC_KP_ENT0_0);
		val = readl(kbc->mmio + KBC_KP_ENT1_0);
	}
	writel(0x7, kbc->mmio + KBC_INT_0);
	spin_unlock_irqrestore(&kbc->lock, flags);

	writel(kbc->pdata->repeat_cnt, kbc->mmio + KBC_RPT_DLY_0);

	val = kbc->pdata->debounce_cnt << 4;
	val |= 1<<14; /* fifo interrupt threshold = 1 entry */
	val |= 1<<3;  /* interrupt on FIFO threshold reached */
	val |= 1;     /* enable */
	writel(val, kbc->mmio + KBC_CONTROL_0);

	return 0;
}


static int __devexit tegra_kbc_remove(struct platform_device *pdev)
{
	struct tegra_kbc *kbc = platform_get_drvdata(pdev);
	struct resource *res;

	free_irq(kbc->irq, pdev);
	disable_clock(kbc);
	disable_power(kbc);
	free_resource(kbc);

	input_unregister_device(kbc->idev);
	input_free_device(kbc->idev);
	iounmap(kbc->mmio);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res_size(res));

	kfree(kbc);
	return 0;
}

static irqreturn_t tegra_kbc_isr(int irq, void *args)
{
	struct tegra_kbc *kbc = args;
	u32 val, ctl;

	/* until all keys are released, defer further processing to
	 * the polling loop in tegra_kbc_key_repeat */
	ctl = readl(kbc->mmio + KBC_CONTROL_0);
	ctl &= ~(1<<3);
	writel(ctl, kbc->mmio + KBC_CONTROL_0);

	/* quickly bail out & reenable interrupts if the interrupt source
	 * wasn't fifo count threshold */
	val = readl(kbc->mmio + KBC_INT_0);
	writel(val, kbc->mmio + KBC_INT_0);

	if (!(val & (1<<2))) {
		ctl |= 1<<3;
		writel(ctl, kbc->mmio + KBC_CONTROL_0);
		return IRQ_HANDLED;
	}

	schedule_work(&kbc->key_repeat);
	return IRQ_HANDLED;
}

static int __init tegra_kbc_probe(struct platform_device *pdev)
{
	struct tegra_kbc *kbc;
	struct tegra_kbc_plat *pdata = pdev->dev.platform_data;
	struct resource *res;
	int irq;
	int err;
	int rows[KBC_MAX_ROW];
	int cols[KBC_MAX_COL];
	int i, j;
	int nr = 0;

	if (!pdata) return -EINVAL;

	kbc = kzalloc(sizeof(*kbc), GFP_KERNEL);
	if (!kbc) return -ENOMEM;

	kbc->pdata = pdata;
	kbc->irq = -EINVAL;

	memset(rows, 0, sizeof(rows));
	memset(cols, 0, sizeof(cols));

	kbc->idev = input_allocate_device();
	if (!kbc->idev) {
		err = -ENOMEM;
		goto fail;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get I/O memory\n");
		err = -ENXIO;
		goto fail;
	}
	res = request_mem_region(res->start, res_size(res), pdev->name);
	if (!res) {
		dev_err(&pdev->dev, "failed to request I/O memory\n");
		err = -EBUSY;
		goto fail;
	}
	kbc->mmio = ioremap(res->start, res_size(res));
	if (!kbc->mmio) {
		dev_err(&pdev->dev, "failed to remap I/O memory\n");
		err = -ENXIO;
		goto fail;
	}
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get keypad IRQ\n");
		err = -ENXIO;
		goto fail;
	}
	err = alloc_resource(kbc, pdev);
	if (err) goto fail;

	platform_set_drvdata(pdev, kbc);

	kbc->idev->name = pdev->name;
	input_set_drvdata(kbc->idev, kbc);
	kbc->idev->id.bustype = BUS_HOST;
	kbc->idev->open = tegra_kbc_open;
	kbc->idev->close = tegra_kbc_close;
	kbc->idev->dev.parent = &pdev->dev;
	spin_lock_init(&kbc->lock);

	for (i=0; i<KBC_MAX_GPIO; i++) {
		if (pdata->pin_cfg[i].is_row && pdata->pin_cfg[i].is_col) {
			dev_err(&pdev->dev, "invalid pin configuration data\n");
			err = -EINVAL;
			goto fail;
		}

		if (pdata->pin_cfg[i].is_row) {
			if (pdata->pin_cfg[i].num >= KBC_MAX_ROW) {
				dev_err(&pdev->dev, "invalid row number\n");
				err = -EINVAL;
				goto fail;
			}
			rows[pdata->pin_cfg[i].num] = 1;
			nr++;
		} else if (pdata->pin_cfg[i].is_col) {
			if (pdata->pin_cfg[i].num >= KBC_MAX_COL) {
				dev_err(&pdev->dev, "invalid column number\n");
				err = -EINVAL;
				goto fail;
			}
			cols[pdata->pin_cfg[i].num] = 1;
		}
	}

	kbc->repoll_time = 5 + (16+pdata->debounce_cnt)*nr + pdata->repeat_cnt;
	kbc->repoll_time = (kbc->repoll_time*1000 + 16384) / 32768;

	kbc->idev->evbit[0] = BIT_MASK(EV_KEY);

	for (i=0; i<KBC_MAX_COL; i++) {
		if (!cols[i]) continue;
		for (j=0; j<KBC_MAX_ROW; j++) {
			int keycode;
			if (!rows[j]) continue;
			keycode = tegra_kbc_keycode(kbc, j, i);
			if (keycode==-1) continue;
			set_bit(keycode, kbc->idev->keybit);
		}
	}

	/* keycode FIFO needs to be read atomically; leave local
	 * interrupts disabled when handling KBC interrupt */
	INIT_WORK(&kbc->key_repeat, tegra_kbc_key_repeat);
	err = request_irq(irq, tegra_kbc_isr, IRQF_DISABLED, pdev->name, kbc);
	if (err) {
		dev_err(&pdev->dev, "failed to request keypad IRQ\n");
		goto fail;
	}
	kbc->irq = irq;

	err = input_register_device(kbc->idev);
	if (err) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto fail;
	}

	device_init_wakeup(&pdev->dev, 1);
	return 0;

fail:
	if (kbc->irq >= 0) free_irq(kbc->irq, pdev);
	if (kbc->idev) input_free_device(kbc->idev);
	free_resource(kbc);
	if (kbc->mmio) iounmap(kbc->mmio);
	kfree(kbc);
	return err;
}

static struct platform_driver tegra_kbc_driver = {
	.probe		= tegra_kbc_probe,
	.remove		= tegra_kbc_remove,
#ifdef CONFIG_PM
	.suspend	= tegra_kbc_suspend,
	.resume		= tegra_kbc_resume,
#endif
	.driver	= {
		.name	= "tegra_kbc"
	}
};

static void __exit tegra_kbc_exit(void)
{
	platform_driver_unregister(&tegra_kbc_driver);
}

static int __devinit tegra_kbc_init(void)
{
	return platform_driver_register(&tegra_kbc_driver);
}

module_exit(tegra_kbc_exit);
module_init(tegra_kbc_init);

MODULE_DESCRIPTION("Tegra matrix keyboard controller driver");
