/*
 * drivers/video/tegra-fb.c
 *
 * Dumb framebuffer driver for NVIDIA Tegra SoCs
 *
 * Copyright (C) 2009 NVIDIA Corporation
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

#include <linux/fb.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <asm/cacheflush.h>
#include <mach/nvrm_linux.h>
#include "nvcommon.h"
#include "nvos.h"
#include "nvcolor.h"
#include "nvbootargs.h"
#include "nvrm_module.h"
#include "nvrm_memmgr.h"
#include "nvrm_ioctls.h"

static struct fb_info tegra_fb_info = {
	.fix = {
		.id		= "nvtegrafb",
		.type		= FB_TYPE_PACKED_PIXELS,
		.visual		= FB_VISUAL_TRUECOLOR,
		.xpanstep	= 0,
		.ypanstep	= 0,
		.accel		= FB_ACCEL_NONE,
		.line_length	= 800 * 2,
	},

	.var = {
		.xres		= 800,
		.yres		= 480,
		.xres_virtual	= 800,
		.yres_virtual	= 480,
		.bits_per_pixel	= 16,
		.red		= {11, 5, 0},
		.green		= {5, 6, 0},
		.blue		= {0, 5, 0},
		.transp		= {0, 0, 0},
		.activate	= FB_ACTIVATE_NOW,
		.height		= -1,
		.width		= -1,
		.pixclock	= 24500,
		.left_margin	= 0,
		.right_margin	= 0,
		.upper_margin	= 0,
		.lower_margin	= 0,
		.hsync_len	= 0,
		.vsync_len	= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};

unsigned long s_fb_addr;
unsigned long s_fb_size;
unsigned long s_fb_width;
unsigned long s_fb_height;
int s_fb_Bpp;
NvRmMemHandle s_fb_hMem;

/* palette attary used by the fbcon */
u32 pseudo_palette[16];

/* fb_ops kernel interface */

int tegra_fb_open(struct fb_info *info, int user);
int tegra_fb_release(struct fb_info *info, int user);
int tegra_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info);
int tegra_fb_set_par(struct fb_info *info);
int tegra_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
	unsigned blue, unsigned transp, struct fb_info *info);
int tegra_fb_blank(int blank, struct fb_info *info);
void tegra_fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect);
void tegra_fb_copyarea(struct fb_info *info, const struct fb_copyarea *region);
void tegra_fb_imageblit(struct fb_info *info, const struct fb_image *image);
int tegra_fb_cursor(struct fb_info *info, struct fb_cursor *cursor);
int tegra_fb_sync(struct fb_info *info);

static struct fb_ops tegra_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= tegra_fb_open,
	.fb_release	= tegra_fb_release,
	.fb_check_var	= tegra_fb_check_var,
	.fb_set_par	= tegra_fb_set_par,
	.fb_setcolreg	= tegra_fb_setcolreg,
	.fb_blank	= tegra_fb_blank,
	.fb_fillrect	= tegra_fb_fillrect,
	.fb_copyarea	= tegra_fb_copyarea,
	.fb_imageblit	= tegra_fb_imageblit,
	.fb_cursor	= tegra_fb_cursor,
	.fb_sync	= tegra_fb_sync,
};

int tegra_fb_open(struct fb_info *info, int user)
{
	return 0;
}

int tegra_fb_release(struct fb_info *info, int user)
{
	return 0;
}

int tegra_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	return 0;
}

int tegra_fb_set_par(struct fb_info *info)
{
	return 0;
}

int tegra_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
	unsigned blue, unsigned transp, struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;

	if( info->fix.visual == FB_VISUAL_TRUECOLOR ||
		info->fix.visual == FB_VISUAL_DIRECTCOLOR )
	{
		u32 v;

		if( regno >= 16 )
		{
			return -EINVAL;
		}

		v = (red << var->red.offset) |
			(green << var->green.offset) |
			(blue << var->blue.offset);

		((u32 *)info->pseudo_palette)[regno] = v;
	}

	return 0;
}

int tegra_fb_blank(int blank, struct fb_info *info)
{
	return 0;
}

void tegra_fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	cfb_fillrect(info, rect);
}

void tegra_fb_copyarea(struct fb_info *info, const struct fb_copyarea *region)
{
	cfb_copyarea(info, region);
}

void tegra_fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	cfb_imageblit(info, image);
}

int tegra_fb_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	return 0;
}

int tegra_fb_sync(struct fb_info *info)
{
	return 0;
}

static int tegra_plat_probe( struct platform_device *d )
{
	NvError e;
	NvBootArgsFramebuffer boot_fb;

	e = NvOsBootArgGet(NvBootArgKey_Framebuffer, &boot_fb, sizeof(boot_fb));
	if (e != NvSuccess || !boot_fb.MemHandleKey) {
		printk("tegrafb: bootargs not found\n");
		return -1;
	}

	e = NvRmMemHandleClaimPreservedHandle(s_hRmGlobal, boot_fb.MemHandleKey,
		&s_fb_hMem );
	if (e != NvSuccess) {
		printk("tegrafb: Unable to query bootup framebuffer memory.\n");
		return -1;
	}

	s_fb_width = boot_fb.Width;
	s_fb_height = boot_fb.Height * boot_fb.NumSurfaces;
	s_fb_size = boot_fb.Size;
	s_fb_addr = NvRmMemPin(s_fb_hMem);
	s_fb_Bpp = NV_COLOR_GET_BPP(boot_fb.ColorFormat) >> 3;

	tegra_fb_info.fix.smem_start = s_fb_addr;
	tegra_fb_info.fix.smem_len = s_fb_size;
	tegra_fb_info.fix.line_length = boot_fb.Pitch;

	tegra_fb_info.fbops = &tegra_fb_ops;

	tegra_fb_info.screen_size = s_fb_size;
	tegra_fb_info.pseudo_palette = pseudo_palette;
	tegra_fb_info.screen_base = ioremap_nocache(s_fb_addr, s_fb_size);

	tegra_fb_info.var.xres = s_fb_width;
	tegra_fb_info.var.yres = boot_fb.Height;
	tegra_fb_info.var.xres_virtual = s_fb_width;
	tegra_fb_info.var.yres_virtual = s_fb_height;

	if( tegra_fb_info.screen_base == 0 ) {
		printk("framebuffer map failure\n");
		NvRmMemHandleFree(s_fb_hMem);
		s_fb_hMem = NULL;
		return -1;
	}

	printk("nvtegrafb: base address: %x\n",
		(unsigned int)tegra_fb_info.screen_base );

	register_framebuffer(&tegra_fb_info);
	return 0;
}

struct platform_driver tegra_platform_driver =
{
	.probe	= tegra_plat_probe,
	.driver	= {
		.name = "nvtegrafb",
		.owner = THIS_MODULE,
	},
};

static struct platform_device tegra_fb_device =
{
	.name		= "nvtegrafb",
	.id		= -1,
	.num_resources	= 0,
	.resource	= 0,
	.dev		= {
		.platform_data = NULL,
	},
};

static int __init tegra_fb_init(void)
{
	int e;
	e = platform_driver_register(&tegra_platform_driver);
	if (e) {
		printk("nvtegrafb: platform_driver_register failed\n");
		return e;
	}

	e = platform_device_register(&tegra_fb_device);
	if (e) {
		printk("nvtegrafb: platform_device_register failed\n");
	}

	return e;
}

static void __exit tegra_exit( void )
{
	unregister_framebuffer(&tegra_fb_info);
}
module_exit(tegra_exit);

module_init(tegra_fb_init);
