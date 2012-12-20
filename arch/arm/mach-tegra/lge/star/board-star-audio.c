/*
 * arch/arm/mach-tegra/board-star.c
 *
 * Copyright (c) 2010 - 2011, NVIDIA Corporation.
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
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/switch.h>

#include <mach/tegra_wm8994_pdata.h>
#include <linux/mfd/wm8994/pdata.h>

#include <mach-tegra/gpio-names.h>
#include <mach-tegra/devices.h>
#include <lge/board-star.h>


static struct wm8994_drc_cfg wm8994_drc_data[] = {
	{
		.name = "AIF1DRC1 Mode",
		.regs = {0x198, 0x84A, 0x818, 0x3EE, 0x1A9},
	},
	{
		.name = "AIF2DRC Mode",
		.regs = {0x198, 0x84E, 0x818, 0x265, 0x187},
	},
};

struct wm8994_pdata wm8994_data = {
	/* configure gpio1 function: 0x0001(Logic level input/output) */
	.gpio_defaults[0] = 0xa101,
	.gpio_defaults[1] = 0xa101,
	/* configure gpio3/4/5/7 function for AIF2 voice */
	.gpio_defaults[2] = 0x8100,
	.gpio_defaults[3] = 0x8100,
	.gpio_defaults[4] = 0x8100,
	.gpio_defaults[5] = 0xa101,
	.gpio_defaults[6] = 0x0100,
	/* configure gpio8/9/10/11 function for AIF3 BT */
	.gpio_defaults[7] = 0x8100,
	.gpio_defaults[8] = 0x0100,
	.gpio_defaults[9] = 0x0100,
	.gpio_defaults[10] = 0x0100,
	.num_drc_cfgs = 2,
	.drc_cfgs  = &wm8994_drc_data,
};


static struct i2c_board_info __initdata wm8994_board_info[] = {
    {
    	I2C_BOARD_INFO("wm8994", 0x1a),
    	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_HP_DET),
    	.platform_data = &wm8994_data,
    },
};

static struct tegra_wm8994_platform_data star_audio_pdata = {
	.name				= "h2w",
	.gpio_hook  		= TEGRA_GPIO_HP_HOOK,
	.gpio_ear_mic		= TEGRA_GPIO_EAR_MIC,

	.gpio_spkr_en = -1, //TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det = TEGRA_GPIO_HP_DET,
	.gpio_hp_mute = -1,
	.gpio_int_mic_en = -1,
	.gpio_ext_mic_en = -1,
	.debounce_time_hp = 200,
};

struct platform_device star_audio_device = {
    .name	= "tegra-snd-wm8994",
	.id	= 0,
	.dev	= {
		.platform_data  = &star_audio_pdata,
	},
};

struct gpio_switch_platform_data star_headset_data = {
    .name = "h2w",
    .gpio = TEGRA_GPIO_PG3,
};

struct platform_device star_headset_detect_device =
{
	.name = "star_headset",
	.id	= -1,
	.dev.platform_data = &star_headset_data,
};

void star_audio_init(void)
{
      i2c_register_board_info(2, wm8994_board_info, ARRAY_SIZE(wm8994_board_info));
}
