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
#include <linux/i2c/pca953x.h>

#include <linux/nct1008.h>
#include "board.h"
#include "board-cardhu.h"
#include "gpio-names.h"

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

static struct nct1008_platform_data cardhu_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = false,
	.conv_rate = 0x08,
	.offset = 0,
	.hysteresis = 5,
	.shutdown_ext_limit = 75,
	.shutdown_local_limit = 75,
	.throttling_ext_limit = 60,
	.alarm_fn = NULL,
};

static struct i2c_board_info cardhu_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &cardhu_nct1008_pdata,
		.irq = -1,
	}
};

static int cardhu_nct1008_init(void)
{
	int nct1008_port = -1;
	struct board_info BoardInfo;
	int ret;

	tegra_get_board_info(&BoardInfo);
	if ((BoardInfo.board_id == BOARD_E1198) ||
		(BoardInfo.board_id == BOARD_E1291)) {
		nct1008_port = TEGRA_GPIO_PCC2;
	} else if ((BoardInfo.board_id == BOARD_E1186) ||
		(BoardInfo.board_id == BOARD_E1187)) {
		/* FIXME: seems to be conflicting with usb3 vbus on E1186 */
		/* nct1008_port = TEGRA_GPIO_PH7; */
	}

	if (nct1008_port >= 0) {
		/* FIXME: enable irq when throttling is supported */
		/* cardhu_i2c4_board_info[0].irq = */
		/* TEGRA_GPIO_TO_IRQ(nct1008_port); */

		ret = gpio_request(nct1008_port, "temp_alert");
		if (ret < 0)
			return ret;

		ret = gpio_direction_input(nct1008_port);
		if (ret < 0)
			gpio_free(nct1008_port);
		else
			tegra_gpio_enable(nct1008_port);

	}
	return ret;
}

#if defined(CONFIG_GPIO_PCA953X)
static struct pca953x_platform_data cardhu_pmu_tca6416_data = {
	.gpio_base      = PMU_TCA6416_GPIO_BASE,
};

static const struct i2c_board_info cardhu_i2c4_board_info_tca6416[] = {
	{
		I2C_BOARD_INFO("tca6416", 0x20),
		.platform_data = &cardhu_pmu_tca6416_data,
	},
};

static int __init pmu_tca6416_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	if ((board_info.board_id == BOARD_E1198) ||
			(board_info.board_id == BOARD_E1291))
		return 0;

	pr_info("Registering pmu pca6416\n");
	i2c_register_board_info(4, cardhu_i2c4_board_info_tca6416,
		ARRAY_SIZE(cardhu_i2c4_board_info_tca6416));
	return 0;
}
#else
static int __init pmu_tca6416_init(void)
{
	return 0;
}
#endif

int __init cardhu_sensors_init(void)
{
	int err;

	if (ARRAY_SIZE(cardhu_i2c3_board_info))
		i2c_register_board_info(3, cardhu_i2c3_board_info,
			ARRAY_SIZE(cardhu_i2c3_board_info));

	pmu_tca6416_init();

	i2c_register_board_info(4, cardhu_i2c4_board_info,
		ARRAY_SIZE(cardhu_i2c4_board_info));

	err = cardhu_nct1008_init();
	if (err)
		return err;

	return 0;
}
