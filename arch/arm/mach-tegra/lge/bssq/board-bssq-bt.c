/*
 * arch/arm/mach-tegra/board-star.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach-tegra/board.h>
#include <lge/board-bssq.h>
#include <mach-tegra/devices.h>
#include <mach-tegra/gpio-names.h>

#ifdef CONFIG_BCM4329_RFKILL
#include <linux/lbee9qmb-rfkill.h>

#define GPIO_BT_RESET		TEGRA_GPIO_PM2
#define GPIO_BT_WAKE		TEGRA_GPIO_PX4
#define GPIO_BT_HOSTWAKE	TEGRA_GPIO_PC7

static bool uart_req;


static int plat_bt_uart_enable(void)
{
	int err = 0;
    printk(KERN_DEBUG "%s", __func__);
#if 0 //LMH_TEMP	
	if (!uart_req) {
		//add enable code if needed
		uart_req = true;
	}
#endif	
	return err;
}

static int plat_bt_uart_disable(void)
{
	int err = 0;
    printk(KERN_DEBUG "%s", __func__);
#if 0 //LMH_TEMP		
	if (uart_req) {
		//add disable code if needed
		uart_req = false;
	}
#endif	
	return err;
}

static struct lbee9qmb_platform_data lbee9qmb_platform = {
	.gpio_reset = GPIO_BT_RESET,
#ifdef CONFIG_BRCM_BT_WAKE
	.gpio_btwake = GPIO_BT_WAKE,
#endif
#ifdef CONFIG_BRCM_HOST_WAKE
	.gpio_hostwake = GPIO_BT_HOSTWAKE,
#endif
	.active_low = 0, /* 0: active high, 1: active low */
	.delay = 100,
	.chip_enable = plat_bt_uart_enable,
	.chip_disable = plat_bt_uart_disable,
};
static struct platform_device lbee9qmb_device = {
    .name = "lbee9qmb-rfkill",
    .dev = {
        .platform_data = &lbee9qmb_platform,
    },
};

void bssq_bt_rfkill(void)
{
    tegra_gpio_enable(GPIO_BT_RESET);
    printk(KERN_DEBUG "%s : tegra_gpio_enable(reset) [%d]", __func__, GPIO_BT_RESET);
    tegra_gpio_enable(GPIO_BT_WAKE);
    printk(KERN_DEBUG "%s : tegra_gpio_enable(btwake) [%d]", __func__, GPIO_BT_WAKE);
    tegra_gpio_enable(GPIO_BT_HOSTWAKE);
    printk(KERN_DEBUG "%s : tegra_gpio_enable(hostwake) [%d]", __func__, GPIO_BT_HOSTWAKE);

    if (platform_device_register(&lbee9qmb_device))
        printk(KERN_DEBUG "%s: lbee9qmb_device registration failed \n", __func__);
	else
        printk(KERN_DEBUG "%s: lbee9qmb_device registration OK \n", __func__);
		
    return;
}
#endif /* CONFIG_BCM4329_RFKILL */

void __init star_setup_bluesleep(void)
{
	struct platform_device *pdev = NULL;
	struct resource *res;

	pdev = platform_device_alloc("bluesleep", 0);
	if (!pdev) {
		pr_err("unable to allocate platform device for bluesleep");
		return;
	}

	res = kzalloc(sizeof(struct resource) * 3, GFP_KERNEL);
	if (!res) {
		pr_err("unable to allocate resource for bluesleep\n");
		goto err_free_dev;
	}

	res[0].name   = "gpio_host_wake";
	res[0].start  = TEGRA_GPIO_PC7;
	res[0].end	  = TEGRA_GPIO_PC7;
	res[0].flags  = IORESOURCE_IO;

	res[1].name   = "gpio_ext_wake";
	res[1].start  = TEGRA_GPIO_PX4;
	res[1].end	  = TEGRA_GPIO_PX4;
	res[1].flags  = IORESOURCE_IO;

	res[2].name   = "host_wake";
	res[2].start  = gpio_to_irq(TEGRA_GPIO_PC7);
	res[2].end	  = gpio_to_irq(TEGRA_GPIO_PC7);
	res[2].flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE ;

	if (platform_device_add_resources(pdev, res, 3)) {
		pr_err("unable to add resources to bluesleep device\n");
		goto err_free_res;
	}

	if (platform_device_add(pdev)) {
		pr_err("unable to add bluesleep device\n");
		goto err_free_res;
	}
	tegra_gpio_enable(TEGRA_GPIO_PC7);
	tegra_gpio_enable(TEGRA_GPIO_PX4);

	return;

err_free_res:
	kfree(res);
err_free_dev:
	platform_device_put(pdev);
	return;
}
