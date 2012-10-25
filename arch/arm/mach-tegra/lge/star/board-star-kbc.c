/*
 * Copyright (C) 2010-2011 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */


#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/gpio_keys.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/kbc.h>
#include <mach-tegra/wakeups-t2.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach-tegra/gpio-names.h>

static int star_wakeup_key(void);

#define PMC_WAKE_STATUS 0x14
#define PMC_WAKE2_STATUS	0x168

#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

static struct gpio_keys_button star_keys[] = {    
#if defined(CONFIG_MACH_STAR_REV_D)    
	[0] = GPIO_KEY(KEY_HOME, PK6, 0),
#else	
	[0] = GPIO_KEY(KEY_HOME, PV6, 1),   
#endif	
	[1] = GPIO_KEY(KEY_VOLUMEDOWN, PG0, 0),
	[2] = GPIO_KEY(KEY_VOLUMEUP, PG1, 0),
	[3] = GPIO_KEY(KEY_POWER, PV2, 1),	
};

static struct gpio_keys_platform_data star_keys_platform_data = {
   .buttons  = star_keys,
   .nbuttons = ARRAY_SIZE(star_keys),   
   .wakeup_key  = star_wakeup_key,
 };

struct platform_device star_keys_device = {
   .name = "gpio-keys",
   .id   = -1,
   .dev  = {
       .platform_data = &star_keys_platform_data,
   },
};

static int star_wakeup_key(void)
{
      int status;
	
	status = readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);

	printk("star_wakeup_key : status %lu\n", status);

	return (status & TEGRA_WAKE_GPIO_PV2) ? KEY_POWER : 
                    (status & TEGRA_WAKE_GPIO_PV6) ? KEY_HOME : KEY_RESERVED;
                
//	return status & TEGRA_WAKE_GPIO_PV2 ? KEY_POWER : KEY_RESERVED;
}

int __init star_kbc_init(void)
{
	int i;	
	int ret = 0;
 
	pr_info("Registering tegra-kbc-gpio\n");

	for (i = 0; i < ARRAY_SIZE(star_keys); i++)
		tegra_gpio_enable(star_keys[i].gpio);	

	platform_device_register(&star_keys_device);

	return 0;
}
