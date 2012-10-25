/*
 * arch/arm/mach-tegra/board-kai-panel.c
 *
 * Copyright (c) 2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
#include <linux/earlysuspend.h>
#include <linux/pwm_backlight.h>
#include <asm/atomic.h>
#include <linux/nvhost.h>
#include <mach/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>

#include "board.h"
#include "board-kai.h"
#include "devices.h"
#include "gpio-names.h"

/* kai default display board pins */
#define kai_lvds_avdd_en		TEGRA_GPIO_PH6
#define kai_lvds_stdby			TEGRA_GPIO_PG5
#define kai_lvds_rst			TEGRA_GPIO_PG7
#define kai_lvds_shutdown		TEGRA_GPIO_PN6
#define kai_lvds_rs			TEGRA_GPIO_PV6
#define kai_lvds_lr			TEGRA_GPIO_PG1

/* kai A00 display board pins */
#define kai_lvds_rs_a00		TEGRA_GPIO_PH1

/* common pins( backlight ) for all display boards */
#define kai_bl_enb			TEGRA_GPIO_PH3
#define kai_bl_pwm			TEGRA_GPIO_PH0
#define kai_hdmi_hpd			TEGRA_GPIO_PN7

#ifdef CONFIG_TEGRA_DC
static struct regulator *kai_hdmi_reg;
static struct regulator *kai_hdmi_pll;
static struct regulator *kai_hdmi_vddio;
#endif

static atomic_t sd_brightness = ATOMIC_INIT(255);

static struct regulator *kai_lvds_reg;
static struct regulator *kai_lvds_vdd_panel;

static tegra_dc_bl_output kai_bl_output_measured = {
	0, 1, 2, 3, 4, 5, 6, 7,
	8, 9, 10, 11, 12, 13, 14, 15,
	16, 17, 18, 19, 20, 21, 22, 23,
	24, 25, 26, 27, 28, 29, 30, 31,
	32, 33, 34, 35, 36, 37, 38, 39,
	40, 41, 42, 43, 44, 45, 46, 47,
	48, 49, 49, 50, 51, 52, 53, 54,
	55, 56, 57, 58, 59, 60, 61, 62,
	63, 64, 65, 66, 67, 68, 69, 70,
	70, 72, 73, 74, 75, 76, 77, 78,
	79, 80, 81, 82, 83, 84, 85, 86,
	87, 88, 89, 90, 91, 92, 93, 94,
	95, 96, 97, 98, 99, 100, 101, 102,
	103, 104, 105, 106, 107, 108, 110, 111,
	112, 113, 114, 115, 116, 117, 118, 119,
	120, 121, 122, 123, 124, 124, 125, 126,
	127, 128, 129, 130, 131, 132, 133, 133,
	134, 135, 136, 137, 138, 139, 140, 141,
	142, 143, 144, 145, 146, 147, 148, 148,
	149, 150, 151, 152, 153, 154, 155, 156,
	157, 158, 159, 160, 161, 162, 163, 164,
	165, 166, 167, 168, 169, 170, 171, 172,
	173, 174, 175, 176, 177, 179, 180, 181,
	182, 184, 185, 186, 187, 188, 189, 190,
	191, 192, 193, 194, 195, 196, 197, 198,
	199, 200, 201, 202, 203, 204, 205, 206,
	207, 208, 209, 211, 212, 213, 214, 215,
	216, 217, 218, 219, 220, 221, 222, 223,
	224, 225, 226, 227, 228, 229, 230, 231,
	232, 233, 234, 235, 236, 237, 238, 239,
	240, 241, 242, 243, 244, 245, 246, 247,
	248, 249, 250, 251, 252, 253, 254, 255
};

static p_tegra_dc_bl_output bl_output;

static int kai_backlight_init(struct device *dev)
{
	int ret;

	bl_output = kai_bl_output_measured;

	if (WARN_ON(ARRAY_SIZE(kai_bl_output_measured) != 256))
		pr_err("bl_output array does not have 256 elements\n");

	tegra_gpio_disable(kai_bl_pwm);

	ret = gpio_request(kai_bl_enb, "backlight_enb");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(kai_bl_enb, 1);
	if (ret < 0)
		gpio_free(kai_bl_enb);
	else
		tegra_gpio_enable(kai_bl_enb);

	return ret;
};

static void kai_backlight_exit(struct device *dev)
{
	/* int ret; */
	/*ret = gpio_request(kai_bl_enb, "backlight_enb");*/
	gpio_set_value(kai_bl_enb, 0);
	gpio_free(kai_bl_enb);
	tegra_gpio_disable(kai_bl_enb);
	return;
}

static int kai_backlight_notify(struct device *unused, int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);

	/* Set the backlight GPIO pin mode to 'backlight_enable' */
	gpio_set_value(kai_bl_enb, !!brightness);

	/* SD brightness is a percentage, 8-bit value. */
	brightness = (brightness * cur_sd_brightness) / 255;

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		brightness = bl_output[brightness];

	return brightness;
}

static int kai_disp1_check_fb(struct device *dev, struct fb_info *info);

static struct platform_pwm_backlight_data kai_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 224,
	.pwm_period_ns	= 100000,
	.init		= kai_backlight_init,
	.exit		= kai_backlight_exit,
	.notify		= kai_backlight_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb	= kai_disp1_check_fb,
};

static struct platform_device kai_backlight_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &kai_backlight_data,
	},
};

static int kai_panel_enable(void)
{
	if (kai_lvds_reg == NULL) {
		kai_lvds_reg = regulator_get(NULL, "vdd_lvds");
		if (WARN_ON(IS_ERR(kai_lvds_reg)))
			pr_err("%s: couldn't get regulator vdd_lvds: %ld\n",
			       __func__, PTR_ERR(kai_lvds_reg));
		else
			regulator_enable(kai_lvds_reg);
	}

	if (kai_lvds_vdd_panel == NULL) {
		kai_lvds_vdd_panel = regulator_get(NULL, "vdd_lcd_panel");
		if (WARN_ON(IS_ERR(kai_lvds_vdd_panel)))
			pr_err("%s: couldn't get regulator vdd_lcd_panel: %ld\n",
			       __func__, PTR_ERR(kai_lvds_vdd_panel));
		else
			regulator_enable(kai_lvds_vdd_panel);
	}

	mdelay(5);

	gpio_set_value(kai_lvds_avdd_en, 1);
	mdelay(5);

	gpio_set_value(kai_lvds_stdby, 1);
	gpio_set_value(kai_lvds_rst, 1);
	gpio_set_value(kai_lvds_shutdown, 1);
	gpio_set_value(kai_lvds_lr, 1);

	mdelay(10);

	return 0;
}

static int kai_panel_disable(void)
{
	gpio_set_value(kai_lvds_lr, 0);
	gpio_set_value(kai_lvds_shutdown, 0);
	gpio_set_value(kai_lvds_rst, 0);
	gpio_set_value(kai_lvds_stdby, 0);
	mdelay(5);

	gpio_set_value(kai_lvds_avdd_en, 0);
	mdelay(5);

	regulator_disable(kai_lvds_reg);
	regulator_put(kai_lvds_reg);
	kai_lvds_reg = NULL;

	regulator_disable(kai_lvds_vdd_panel);
	regulator_put(kai_lvds_vdd_panel);
	kai_lvds_vdd_panel = NULL;

	return 0;
}

#ifdef CONFIG_TEGRA_DC
static int kai_hdmi_vddio_enable(void)
{
	int ret;
	if (!kai_hdmi_vddio) {
		kai_hdmi_vddio = regulator_get(NULL, "vdd_hdmi_con");
		if (IS_ERR_OR_NULL(kai_hdmi_vddio)) {
			ret = PTR_ERR(kai_hdmi_vddio);
			pr_err("hdmi: couldn't get regulator vdd_hdmi_con\n");
			kai_hdmi_vddio = NULL;
			return ret;
		}
	}
	ret = regulator_enable(kai_hdmi_vddio);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator vdd_hdmi_con\n");
		regulator_put(kai_hdmi_vddio);
		kai_hdmi_vddio = NULL;
		return ret;
	}
	return ret;
}

static int kai_hdmi_vddio_disable(void)
{
	if (kai_hdmi_vddio) {
		regulator_disable(kai_hdmi_vddio);
		regulator_put(kai_hdmi_vddio);
		kai_hdmi_vddio = NULL;
	}
	return 0;
}

static int kai_hdmi_enable(void)
{
	int ret;
	if (!kai_hdmi_reg) {
		kai_hdmi_reg = regulator_get(NULL, "avdd_hdmi");
		if (IS_ERR_OR_NULL(kai_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			kai_hdmi_reg = NULL;
			return PTR_ERR(kai_hdmi_reg);
		}
	}
	ret = regulator_enable(kai_hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}
	if (!kai_hdmi_pll) {
		kai_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(kai_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			kai_hdmi_pll = NULL;
			regulator_put(kai_hdmi_reg);
			kai_hdmi_reg = NULL;
			return PTR_ERR(kai_hdmi_pll);
		}
	}
	ret = regulator_enable(kai_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}
	return 0;
}

static int kai_hdmi_disable(void)
{
	regulator_disable(kai_hdmi_reg);
	regulator_put(kai_hdmi_reg);
	kai_hdmi_reg = NULL;

	regulator_disable(kai_hdmi_pll);
	regulator_put(kai_hdmi_pll);
	kai_hdmi_pll = NULL;
	return 0;
}

static struct resource kai_disp1_resources[] = {
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
		.start	= 0,	/* Filled in by kai_panel_init() */
		.end	= 0,	/* Filled in by kai_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource kai_disp2_resources[] = {
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
#endif

static struct tegra_dc_mode kai_panel_modes[] = {
	{
		/* 1024x600@60Hz */
		.pclk = 51206000,
		.h_ref_to_sync = 11,
		.v_ref_to_sync = 1,
		.h_sync_width = 10,
		.v_sync_width = 5,
		.h_back_porch = 10,
		.v_back_porch = 15,
		.h_active = 1024,
		.v_active = 600,
		.h_front_porch = 300,
		.v_front_porch = 15,
	},
};

static struct tegra_dc_sd_settings kai_sd_settings = {
	.enable = 1, /* enabled by default. */
	.use_auto_pwm = false,
	.hw_update_delay = 0,
	.bin_width = -1,
	.aggressiveness = 1,
	.phase_in_adjustments = true,
	.use_vid_luma = false,
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
	.bl_device = &kai_backlight_device,
};

#ifdef CONFIG_TEGRA_DC
static struct tegra_fb_data kai_fb_data = {
	.win		= 0,
	.xres		= 1024,
	.yres		= 600,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_fb_data kai_hdmi_fb_data = {
	.win		= 0,
	.xres		= 1024,
	.yres		= 600,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_out kai_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus	= 3,
	.hotplug_gpio	= kai_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= kai_hdmi_enable,
	.disable	= kai_hdmi_disable,

	.postsuspend	= kai_hdmi_vddio_disable,
	.hotplug_init	= kai_hdmi_vddio_enable,
};

static struct tegra_dc_platform_data kai_disp2_pdata = {
	.flags		= 0,
	.default_out	= &kai_disp2_out,
	.fb		= &kai_hdmi_fb_data,
	.emc_clk_rate	= 300000000,
};
#endif

static struct tegra_dc_out kai_disp1_out = {
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.sd_settings	= &kai_sd_settings,
	.parent_clk	= "pll_p",

	.type		= TEGRA_DC_OUT_RGB,
	.depth		= 18,
	.dither		= TEGRA_DC_ORDERED_DITHER,

	.modes		= kai_panel_modes,
	.n_modes	= ARRAY_SIZE(kai_panel_modes),

	.enable		= kai_panel_enable,
	.disable	= kai_panel_disable,
};

#ifdef CONFIG_TEGRA_DC
static struct tegra_dc_platform_data kai_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &kai_disp1_out,
	.emc_clk_rate	= 300000000,
	.fb		= &kai_fb_data,
};

static struct nvhost_device kai_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= kai_disp1_resources,
	.num_resources	= ARRAY_SIZE(kai_disp1_resources),
	.dev = {
		.platform_data = &kai_disp1_pdata,
	},
};

static int kai_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &kai_disp1_device.dev;
}

static struct nvhost_device kai_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= kai_disp2_resources,
	.num_resources	= ARRAY_SIZE(kai_disp2_resources),
	.dev = {
		.platform_data = &kai_disp2_pdata,
	},
};
#else
static int kai_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return 0;
}
#endif

static struct nvmap_platform_carveout kai_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by kai_panel_init() */
		.size		= 0,	/* Filled in by kai_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data kai_nvmap_data = {
	.carveouts	= kai_carveouts,
	.nr_carveouts	= ARRAY_SIZE(kai_carveouts),
};

static struct platform_device kai_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &kai_nvmap_data,
	},
};


static struct platform_device *kai_gfx_devices[] __initdata = {
	&kai_nvmap_device,
#ifdef CONFIG_TEGRA_GRHOST
	&tegra_grhost_device,
#endif
	&tegra_pwfm0_device,
	&kai_backlight_device,
};


#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend kai_panel_early_suspender;

static void kai_panel_early_suspend(struct early_suspend *h)
{
	/* power down LCD, add use a black screen for HDMI */
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
	if (num_registered_fb > 1)
		fb_blank(registered_fb[1], FB_BLANK_NORMAL);
}

static void kai_panel_late_resume(struct early_suspend *h)
{
	unsigned i;
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
}
#endif

int __init kai_panel_init(void)
{
	int err;
	struct resource __maybe_unused *res;
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	kai_carveouts[1].base = tegra_carveout_start;
	kai_carveouts[1].size = tegra_carveout_size;

	gpio_request(kai_lvds_avdd_en, "lvds_avdd_en");
	gpio_direction_output(kai_lvds_avdd_en, 1);
	tegra_gpio_enable(kai_lvds_avdd_en);

	gpio_request(kai_lvds_stdby, "lvds_stdby");
	gpio_direction_output(kai_lvds_stdby, 1);
	tegra_gpio_enable(kai_lvds_stdby);

	gpio_request(kai_lvds_rst, "lvds_rst");
	gpio_direction_output(kai_lvds_rst, 1);
	tegra_gpio_enable(kai_lvds_rst);

	if (board_info.fab & BOARD_FAB_A00) {
		gpio_request(kai_lvds_rs_a00, "lvds_rs");
		gpio_direction_output(kai_lvds_rs_a00, 0);
		tegra_gpio_enable(kai_lvds_rs_a00);
	} else {
		gpio_request(kai_lvds_rs, "lvds_rs");
		gpio_direction_output(kai_lvds_rs, 0);
		tegra_gpio_enable(kai_lvds_rs);
	}

	gpio_request(kai_lvds_lr, "lvds_lr");
	gpio_direction_output(kai_lvds_lr, 1);
	tegra_gpio_enable(kai_lvds_lr);

	gpio_request(kai_lvds_shutdown, "lvds_shutdown");
	gpio_direction_output(kai_lvds_shutdown, 1);
	tegra_gpio_enable(kai_lvds_shutdown);

	tegra_gpio_enable(kai_hdmi_hpd);
	gpio_request(kai_hdmi_hpd, "hdmi_hpd");
	gpio_direction_input(kai_hdmi_hpd);

#ifdef CONFIG_HAS_EARLYSUSPEND
	kai_panel_early_suspender.suspend = kai_panel_early_suspend;
	kai_panel_early_suspender.resume = kai_panel_late_resume;
	kai_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&kai_panel_early_suspender);
#endif

	err = platform_add_devices(kai_gfx_devices,
				ARRAY_SIZE(kai_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&kai_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;
#endif

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
				min(tegra_fb_size, tegra_bootloader_fb_size));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	if (!err)
		err = nvhost_device_register(&kai_disp1_device);

	res = nvhost_get_resource_byname(&kai_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;
	if (!err)
		err = nvhost_device_register(&kai_disp2_device);
#endif

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_NVAVP)
	if (!err)
		err = nvhost_device_register(&nvavp_device);
#endif
	return err;
}
