/*
 * arch/arm/mach-tegra/board-whistler-panel.c
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#include <mach/dc.h>
#include <mach/fb.h>

static struct tegra_dc_mode whistler_panel_modes[] = {
	{
		.pclk = 216000000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 2,
		.h_sync_width = 10,
		.v_sync_width = 3,
		.h_back_porch = 20,
		.v_back_porch = 3,
		.h_active = 800,
		.v_active = 480,
		.h_front_porch = 70,
		.v_front_porch = 3,
	},
};

static struct tegra_fb_data whistler_fb_data = {
	.win		= 0,
	.xres		= 800,
	.yres		= 480,
	.bits_per_pixel	= 16,
};

int __init whistler_panel_init(void)
{
	return 0;
}

