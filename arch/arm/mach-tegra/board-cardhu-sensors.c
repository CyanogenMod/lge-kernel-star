/*
 * arch/arm/mach-tegra/board-cardhu-sensors.c
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/i2c.h>
#include <linux/i2c/pca954x.h>

#include "board-cardhu.h"

#ifdef CONFIG_I2C_MUX_PCA954x
static struct pca954x_platform_mode cardhu_pca954x_modes[] = {
	{ .adap_id = PCA954x_I2C_BUS0, },
	{ .adap_id = PCA954x_I2C_BUS1, },
	{ .adap_id = PCA954x_I2C_BUS2, },
	{ .adap_id = PCA954x_I2C_BUS3, },
};

static struct pca954x_platform_data cardhu_pca954x_data = {
	.modes    = cardhu_pca954x_modes,
	.num_modes      = ARRAY_SIZE(cardhu_pca954x_modes),
};

#endif

static const struct i2c_board_info cardhu_i2c3_board_info[] = {
#ifdef CONFIG_I2C_MUX_PCA954x
	{
		I2C_BOARD_INFO("pca9546", 0x70),
		.platform_data = &cardhu_pca954x_data,
	},
#endif
};

int __init cardhu_sensors_init(void)
{
	if (ARRAY_SIZE(cardhu_i2c3_board_info))
		i2c_register_board_info(3, cardhu_i2c3_board_info,
			ARRAY_SIZE(cardhu_i2c3_board_info));

	return 0;
}
