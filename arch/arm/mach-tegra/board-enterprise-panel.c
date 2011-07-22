/*
 * arch/arm/mach-tegra/board-enterprise-panel.c
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/tegra_pwm_bl.h>
#include <asm/atomic.h>
#include <mach/nvhost.h>
#include <mach/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>

#include "board.h"
#include "board-enterprise.h"
#include "devices.h"
#include "gpio-names.h"

/* Select panel to be used. */
#define AVDD_LCD PMU_TCA6416_GPIO_PORT17
#define DSI_PANEL_RESET 0

#define enterprise_lvds_shutdown	TEGRA_GPIO_PL2
#define enterprise_hdmi_hpd		TEGRA_GPIO_PN7

#define enterprise_dsi_panel_reset	TEGRA_GPIO_PW0

#define enterprise_lcd_2d_3d		TEGRA_GPIO_PH1
#define ENTERPRISE_STEREO_3D		0
#define ENTERPRISE_STEREO_2D		1

#define enterprise_lcd_swp_pl		TEGRA_GPIO_PH2
#define ENTERPRISE_STEREO_LANDSCAPE	0
#define ENTERPRISE_STEREO_PORTRAIT	1

static struct regulator *enterprise_dsi_reg = NULL;

static struct regulator *enterprise_hdmi_reg;
static struct regulator *enterprise_hdmi_pll;
static struct regulator *enterprise_hdmi_vddio;

static atomic_t sd_brightness = ATOMIC_INIT(255);

static struct platform_tegra_pwm_backlight_data enterprise_disp1_backlight_data = {
	.which_dc	= 0,
	.which_pwm	= TEGRA_PWM_PM1,
	.max_brightness	= 255,
	.dft_brightness	= 224,
	.period		= 0x3F,
	.clk_div	= 1,
	.clk_select	= 2,
};

static struct platform_device enterprise_disp1_backlight_device = {
	.name	= "tegra-pwm-bl",
	.id	= -1,
	.dev	= {
		.platform_data = &enterprise_disp1_backlight_data,
	},
};

static int enterprise_hdmi_vddio_enable(void)
{
	int ret;
	if (!enterprise_hdmi_vddio) {
		enterprise_hdmi_vddio = regulator_get(NULL, "hdmi_5v0");
		if (IS_ERR_OR_NULL(enterprise_hdmi_vddio)) {
			ret = PTR_ERR(enterprise_hdmi_vddio);
			pr_err("hdmi: couldn't get regulator hdmi_5v0\n");
			enterprise_hdmi_vddio = NULL;
			return ret;
		}
	}
	ret = regulator_enable(enterprise_hdmi_vddio);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator hdmi_5v0\n");
		regulator_put(enterprise_hdmi_vddio);
		enterprise_hdmi_vddio = NULL;
		return ret;
	}
	return ret;
}

static int enterprise_hdmi_vddio_disable(void)
{
	if (enterprise_hdmi_vddio) {
		regulator_disable(enterprise_hdmi_vddio);
		regulator_put(enterprise_hdmi_vddio);
		enterprise_hdmi_vddio = NULL;
	}
	return 0;
}

static int enterprise_hdmi_enable(void)
{
	int ret;
	if (!enterprise_hdmi_reg) {
		enterprise_hdmi_reg = regulator_get(NULL, "avdd_hdmi");
		if (IS_ERR_OR_NULL(enterprise_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			enterprise_hdmi_reg = NULL;
			return PTR_ERR(enterprise_hdmi_reg);
		}
	}
	ret = regulator_enable(enterprise_hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}
	if (!enterprise_hdmi_pll) {
		enterprise_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(enterprise_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			enterprise_hdmi_pll = NULL;
			regulator_put(enterprise_hdmi_reg);
			enterprise_hdmi_reg = NULL;
			return PTR_ERR(enterprise_hdmi_pll);
		}
	}
	ret = regulator_enable(enterprise_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}
	return 0;
}

static int enterprise_hdmi_disable(void)
{

	regulator_disable(enterprise_hdmi_reg);
	regulator_put(enterprise_hdmi_reg);
	enterprise_hdmi_reg = NULL;

	regulator_disable(enterprise_hdmi_pll);
	regulator_put(enterprise_hdmi_pll);
	enterprise_hdmi_pll = NULL;

	return 0;
}
static struct resource enterprise_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0,	/* Filled in by enterprise_panel_init() */
		.end	= 0,	/* Filled in by enterprise_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "dsi_regs",
		.start	= TEGRA_DSI_BASE,
		.end	= TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource enterprise_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
		.start	= 0,
		.end	= 0,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_sd_settings enterprise_sd_settings = {
	.enable = 1, /* Normal mode operation */
	.use_auto_pwm = false,
	.hw_update_delay = 0,
	.bin_width = -1,
	.aggressiveness = 5,
	.use_vid_luma = true,
	/* Default video coefficients */
	.coeff = {5, 9, 2},
	.fc = {0, 0},
	/* Immediate backlight changes */
	.blp = {1024, 255},
	/* Gammas: R: 2.2 G: 2.2 B: 2.2 */
	/* Default BL TF */
	.bltf = {
			{
				{57, 65, 74, 83},
				{93, 103, 114, 126},
				{138, 151, 165, 179},
				{194, 209, 225, 242},
			},
			{
				{58, 66, 75, 84},
				{94, 105, 116, 127},
				{140, 153, 166, 181},
				{196, 211, 227, 244},
			},
			{
				{60, 68, 77, 87},
				{97, 107, 119, 130},
				{143, 156, 170, 184},
				{199, 215, 231, 248},
			},
			{
				{64, 73, 82, 91},
				{102, 113, 124, 137},
				{149, 163, 177, 192},
				{207, 223, 240, 255},
			},
		},
	/* Default LUT */
	.lut = {
			{
				{250, 250, 250},
				{194, 194, 194},
				{149, 149, 149},
				{113, 113, 113},
				{82, 82, 82},
				{56, 56, 56},
				{34, 34, 34},
				{15, 15, 15},
				{0, 0, 0},
			},
			{
				{246, 246, 246},
				{191, 191, 191},
				{147, 147, 147},
				{111, 111, 111},
				{80, 80, 80},
				{55, 55, 55},
				{33, 33, 33},
				{14, 14, 14},
				{0, 0, 0},
			},
			{
				{239, 239, 239},
				{185, 185, 185},
				{142, 142, 142},
				{107, 107, 107},
				{77, 77, 77},
				{52, 52, 52},
				{30, 30, 30},
				{12, 12, 12},
				{0, 0, 0},
			},
			{
				{224, 224, 224},
				{173, 173, 173},
				{133, 133, 133},
				{99, 99, 99},
				{70, 70, 70},
				{46, 46, 46},
				{25, 25, 25},
				{7, 7, 7},
				{0, 0, 0},
			},
		},
	.sd_brightness = &sd_brightness,
	.bl_device = &enterprise_disp1_backlight_device,
};

static struct tegra_fb_data enterprise_hdmi_fb_data = {
	.win		= 0,
	.xres		= 1366,
	.yres		= 768,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_out enterprise_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus	= 3,
	.hotplug_gpio	= enterprise_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= enterprise_hdmi_enable,
	.disable	= enterprise_hdmi_disable,
	.postsuspend	= enterprise_hdmi_vddio_disable,
	.hotplug_init	= enterprise_hdmi_vddio_enable,
};

static struct tegra_dc_platform_data enterprise_disp2_pdata = {
	.flags		= 0,
	.default_out	= &enterprise_disp2_out,
	.fb		= &enterprise_hdmi_fb_data,
	.emc_clk_rate	= 300000000,
};

static int enterprise_dsi_panel_enable(void)
{
	int ret;

	if (enterprise_dsi_reg == NULL) {
		enterprise_dsi_reg = regulator_get(NULL, "avdd_dsi_csi");
		if (IS_ERR_OR_NULL(enterprise_dsi_reg)) {
			pr_err("dsi: Could not get regulator avdd_dsi_csi\n");
				enterprise_dsi_reg = NULL;
				return PTR_ERR(enterprise_dsi_reg);
		}
	}
	ret = regulator_enable(enterprise_dsi_reg);
	if (ret < 0) {
		printk(KERN_ERR
			"DSI regulator avdd_dsi_csi could not be enabled\n");
		return ret;
	}

#if DSI_PANEL_RESET
	ret = gpio_request(enterprise_dsi_panel_reset, "panel reset");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(enterprise_dsi_panel_reset, 0);
	if (ret < 0) {
		gpio_free(enterprise_dsi_panel_reset);
		return ret;
	}
	tegra_gpio_enable(enterprise_dsi_panel_reset);

	gpio_set_value(enterprise_dsi_panel_reset, 0);
	udelay(2000);
	gpio_set_value(enterprise_dsi_panel_reset, 1);
	mdelay(20);
#endif

	return ret;
}

static int enterprise_dsi_panel_disable(void)
{
#if DSI_PANEL_RESET
	tegra_gpio_disable(enterprise_dsi_panel_reset);
	gpio_free(enterprise_dsi_panel_reset);
#endif
	return 0;
}

static void enterprise_stereo_set_mode(int mode)
{
	switch (mode) {
	case TEGRA_DC_STEREO_MODE_2D:
		gpio_set_value(TEGRA_GPIO_PH1, ENTERPRISE_STEREO_2D);
		break;
	case TEGRA_DC_STEREO_MODE_3D:
		gpio_set_value(TEGRA_GPIO_PH1, ENTERPRISE_STEREO_3D);
		break;
	}
}

static void enterprise_stereo_set_orientation(int mode)
{
	switch (mode) {
	case TEGRA_DC_STEREO_LANDSCAPE:
		gpio_set_value(TEGRA_GPIO_PH2, ENTERPRISE_STEREO_LANDSCAPE);
		break;
	case TEGRA_DC_STEREO_PORTRAIT:
		gpio_set_value(TEGRA_GPIO_PH2, ENTERPRISE_STEREO_PORTRAIT);
		break;
	}
}

static int enterprise_dsi_panel_postsuspend(void)
{
	int err = 0;

	if (enterprise_dsi_reg) {
		err = regulator_disable(enterprise_dsi_reg);
		if (err < 0)
			printk(KERN_ERR
			"DSI regulator avdd_dsi_csi disable failed\n");
		regulator_put(enterprise_dsi_reg);
		enterprise_dsi_reg = NULL;
	}

	return err;
}

static struct tegra_dsi_cmd dsi_init_cmd[]= {
	DSI_CMD_SHORT(0x05, 0x11, 0x00),
	DSI_DLY_MS(150),
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(20),
};

static struct tegra_dsi_cmd dsi_suspend_cmd[] = {
	DSI_CMD_SHORT(0x05, 0x28, 0x00),
	DSI_DLY_MS(20),
	DSI_CMD_SHORT(0x05, 0x10, 0x00),
	DSI_DLY_MS(5),
};

struct tegra_dsi_out enterprise_dsi = {
	.n_data_lanes = 2,
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.refresh_rate = 60,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.panel_has_frame_buffer = true,
	.dsi_instance = 0,

	.panel_reset = DSI_PANEL_RESET,

	.n_init_cmd = ARRAY_SIZE(dsi_init_cmd),
	.dsi_init_cmd = dsi_init_cmd,

	.n_suspend_cmd = ARRAY_SIZE(dsi_suspend_cmd),
	.dsi_suspend_cmd = dsi_suspend_cmd,
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,
	.lp_cmd_mode_freq_khz = 430000,
};

static struct tegra_stereo_out enterprise_stereo = {
	.set_mode		= &enterprise_stereo_set_mode,
	.set_orientation	= &enterprise_stereo_set_orientation,
};

static struct tegra_dc_mode enterprise_dsi_modes[] = {
	{
		.pclk = 10000000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 16,
		.v_sync_width = 1,
		.h_back_porch = 32,
		.v_back_porch = 1,
		.h_active = 540,
		.v_active = 960,
		.h_front_porch = 32,
		.v_front_porch = 2,
	},
};


static struct tegra_fb_data enterprise_dsi_fb_data = {
	.win		= 0,
	.xres		= 540,
	.yres		= 960,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};


static struct tegra_dc_out enterprise_disp1_out = {
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.sd_settings	= &enterprise_sd_settings,

	.type		= TEGRA_DC_OUT_DSI,

	.modes		= enterprise_dsi_modes,
	.n_modes	= ARRAY_SIZE(enterprise_dsi_modes),

	.dsi		= &enterprise_dsi,
	.stereo		= &enterprise_stereo,

	.enable		= enterprise_dsi_panel_enable,
	.disable	= enterprise_dsi_panel_disable,
	.postsuspend	= enterprise_dsi_panel_postsuspend,
};
static struct tegra_dc_platform_data enterprise_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &enterprise_disp1_out,
	.emc_clk_rate	= 204000000,
	.fb		= &enterprise_dsi_fb_data,
};
static struct nvhost_device enterprise_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= enterprise_disp1_resources,
	.num_resources	= ARRAY_SIZE(enterprise_disp1_resources),
	.dev = {
		.platform_data = &enterprise_disp1_pdata,
	},
};

static struct nvhost_device enterprise_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= enterprise_disp2_resources,
	.num_resources	= ARRAY_SIZE(enterprise_disp2_resources),
	.dev = {
		.platform_data = &enterprise_disp2_pdata,
	},
};

static struct nvmap_platform_carveout enterprise_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by enterprise_panel_init() */
		.size		= 0,	/* Filled in by enterprise_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data enterprise_nvmap_data = {
	.carveouts	= enterprise_carveouts,
	.nr_carveouts	= ARRAY_SIZE(enterprise_carveouts),
};

static struct platform_device enterprise_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &enterprise_nvmap_data,
	},
};

static struct platform_device *enterprise_gfx_devices[] __initdata = {
	&enterprise_nvmap_device,
	&tegra_grhost_device,
	&tegra_pwfm0_device,
};

static struct platform_device *enterprise_bl_devices[]  = {
	&enterprise_disp1_backlight_device,
};

int __init enterprise_panel_init(void)
{
	int err;
	struct resource *res;

	enterprise_carveouts[1].base = tegra_carveout_start;
	enterprise_carveouts[1].size = tegra_carveout_size;

	tegra_gpio_enable(enterprise_hdmi_hpd);
	gpio_request(enterprise_hdmi_hpd, "hdmi_hpd");
	gpio_direction_input(enterprise_hdmi_hpd);

	tegra_gpio_enable(enterprise_lcd_2d_3d);
	gpio_request(enterprise_lcd_2d_3d, "lcd_2d_3d");
	gpio_direction_output(enterprise_lcd_2d_3d, 0);
	enterprise_stereo_set_mode(enterprise_stereo.mode_2d_3d);

	tegra_gpio_enable(enterprise_lcd_swp_pl);
	gpio_request(enterprise_lcd_swp_pl, "lcd_swp_pl");
	gpio_direction_output(enterprise_lcd_swp_pl, 0);
	enterprise_stereo_set_orientation(enterprise_stereo.orientation);

	tegra_gpio_disable(TEGRA_GPIO_PW1);

	err = platform_add_devices(enterprise_gfx_devices,
				ARRAY_SIZE(enterprise_gfx_devices));

	res = nvhost_get_resource_byname(&enterprise_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
		min(tegra_fb_size, tegra_bootloader_fb_size));

	if (!err)
		err = nvhost_device_register(&enterprise_disp1_device);

	res = nvhost_get_resource_byname(&enterprise_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;
	if (!err)
		err = nvhost_device_register(&enterprise_disp2_device);

	err = platform_add_devices(enterprise_bl_devices,
				ARRAY_SIZE(enterprise_bl_devices));
	return err;
}
