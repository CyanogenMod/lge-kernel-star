/*
 * arch/arm/mach-tegra/board-bssq.c
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
#include <lge/board-bssq.h>


static struct wm8994_drc_cfg wm8994_drc_data[] = {
	{
		.name = "AIF1DRC1 Mode",
		.regs = {0x19B, 0x843, 0x818, 0x320, 0x30C},
	},
	{
		.name = "AIF2DRC Mode",
	 	.regs = {0x198, 0x84E, 0x818, 0x265, 0x187},
	},
};

struct wm8994_pdata wm8994_data = {
	/* configure gpio1 function: 0x0001(Logic level input/output) */
	.gpio_defaults[0] = 0x0001,
	/* configure gpio3/4/5/7 function for AIF2 voice */
	.gpio_defaults[2] = 0x8100,
	.gpio_defaults[3] = 0x8100,
	.gpio_defaults[4] = 0x8100,
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
		I2C_BOARD_INFO("wm8994", 0x1A),
    	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_HP_DET),
    	.platform_data = &wm8994_data,
    },
};

static struct tegra_wm8994_platform_data bssq_audio_pdata = {
	.name				= "h2w",
	.gpio_hook  		= TEGRA_GPIO_HP_HOOK,
	.gpio_ear_mic		= TEGRA_GPIO_EAR_MIC,

//LGE_UPDATE_S, bae.cheolhwan@lge.com 2012.05.09. Power consumption for audio. (nVidia Patch)
#if defined(CONFIG_MACH_BSSQ)
	.gpio_spkr_en = -1, //TEGRA_GPIO_SPKR_EN,
#else
	.gpio_spkr_en = TEGRA_GPIO_SPKR_EN,
#endif
//LGE_UPDATE_E, bae.cheolhwan@lge.com 2012.05.09.
	.gpio_hp_det = TEGRA_GPIO_HP_DET,
	.gpio_hp_mute = -1,
	.gpio_int_mic_en = -1,
	.gpio_ext_mic_en = -1,
	.debounce_time_hp = 200,
};

struct platform_device bssq_audio_device = {
    .name	= "tegra-snd-wm8994",
	.id	= 0,
	.dev	= {
		.platform_data  = &bssq_audio_pdata,
	},
};

//LGE_UPDATE_S, bae.cheolhwan@lge.com 2012.05.09. Power consumption for audio. (nVidia Patch)
#if defined(CONFIG_MACH_BSSQ)
static struct platform_device *bssq_audio_devices[] __initdata = {
	&bssq_audio_device,
};
#endif
//LGE_UPDATE_E, bae.cheolhwan@lge.com 2012.05.09.

//LGE_UPDATE_S, bae.cheolhwan@lge.com 2012.05.09. Power consumption for audio. (nVidia Patch)
#if defined(CONFIG_MACH_BSSQ)
void __init bssq_audio_init(void)
{
      i2c_register_board_info(2, wm8994_board_info, ARRAY_SIZE(wm8994_board_info));
      platform_add_devices(bssq_audio_devices, ARRAY_SIZE(bssq_audio_devices));
}
#else
void bssq_audio_init(void)
{
      i2c_register_board_info(2, wm8994_board_info, ARRAY_SIZE(wm8994_board_info));
}
#endif
//LGE_UPDATE_E, bae.cheolhwan@lge.com 2012.05.09.
