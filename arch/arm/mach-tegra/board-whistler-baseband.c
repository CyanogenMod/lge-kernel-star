/*
 * arch/arm/mach-tegra/board-whistler-baseband.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/tegra_caif.h>
#include "board-whistler-baseband.h"

unsigned long baseband_type = BOARD_WHISTLER_BASEBAND_U3XX;

module_param(baseband_type, ulong, 0644);
MODULE_PARM_DESC(baseband_type, "baseband type");

static struct tegra_clk_init_table u3xx_clk[] = {
	/* spi slave controller clock @ 4 x 13 Mhz interface clock */
	{ "sbc1",	"pll_m",	52000000,	true},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_clk_init_table n731_clk[] = {
	/* spi master controller clock @ 4 x 12 Mhz interface clock */
	{ "sbc1",	"pll_m",	48000000,	true},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_clk_init_table spi_loopback_clk[] = {
	/* spi slave / master controller clocks @ 4 x max interface clock */
	{ "sbc1",	"pll_m",	60000000,	true},
	{ "sbc2",	"pll_m",	60000000,	true},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_clk_init_table hsic_clk[] = {
	{ NULL,		NULL,		0,		0},
};

static struct platform_device *u3xx_device[] = {
	/* spi slave */
	&tegra_spi_slave_device1,
};

static struct platform_device *n731_device[] = {
	/* spi master */
	&tegra_spi_device1,
};

static struct platform_device *spi_loopback_device[] = {
	/* spi slave / master */
	&tegra_spi_slave_device1,
	&tegra_spi_device2,
};

static struct platform_device *hsic_device[] = {
};

struct tegra_caif_platform_data  tegra_whistler_u3xx_plat_data = {
	.reset = TEGRA_CAIF_SSPI_GPIO_RESET,
	.power = TEGRA_CAIF_SSPI_GPIO_POWER,
	.awr = TEGRA_CAIF_SSPI_GPIO_AWR,
	.cwr = TEGRA_CAIF_SSPI_GPIO_CWR,
	.spi_int = TEGRA_CAIF_SSPI_GPIO_SPI_INT,
	.spi_ss = TEGRA_CAIF_SSPI_GPIO_SS,
};

static struct spi_board_info u3xx_spi_board_info[] = {
	/* spi slave */
	{
		.modalias = "baseband_spi_slave0.0",
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.max_speed_hz = 52000000,
		.platform_data = &tegra_whistler_u3xx_plat_data,
		.irq = 0,
	},
};

static struct spi_board_info n731_spi_board_info[] = {
	/* spi master */
	{
		.modalias = "spi_ipc",
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_1,
		.max_speed_hz = 12000000,
		.platform_data = NULL,
		.irq = 0,
	},
};

static struct spi_board_info spi_loopback_spi_board_info[] = {
	/* spi slave <---> spi master */
	{
		.modalias = "baseband_loopback",
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.max_speed_hz = 13000000,
		.platform_data = &tegra_whistler_u3xx_plat_data,
		.irq = 0,
	},
	{
		.modalias = "baseband_spi_master1.1",
		.bus_num = 1,
		.chip_select = 1,
		.mode = SPI_MODE_0,
		.max_speed_hz = 13000000,
		.platform_data = NULL,
		.irq = 0,
	},
};

static struct whistler_baseband whistler_baseband[] = {
	[BOARD_WHISTLER_BASEBAND_U3XX] = {
		.clk_init = u3xx_clk,
		.platform_device = u3xx_device,
		.platform_device_size = ARRAY_SIZE(u3xx_device),
		.spi_board_info = u3xx_spi_board_info,
		.spi_board_info_size = ARRAY_SIZE(u3xx_spi_board_info),
	},
	[BOARD_WHISTLER_BASEBAND_N731] = {
		.clk_init = n731_clk,
		.platform_device = n731_device,
		.platform_device_size = ARRAY_SIZE(n731_device),
		.spi_board_info = n731_spi_board_info,
		.spi_board_info_size = ARRAY_SIZE(n731_spi_board_info),
	},
	[BOARD_WHISTLER_BASEBAND_SPI_LOOPBACK] = {
		.clk_init = spi_loopback_clk,
		.platform_device = spi_loopback_device,
		.platform_device_size = ARRAY_SIZE(spi_loopback_device),
		.spi_board_info = spi_loopback_spi_board_info,
		.spi_board_info_size = ARRAY_SIZE(spi_loopback_spi_board_info),
	},
	[BOARD_WHISTLER_BASEBAND_HSIC] = {
		.clk_init = hsic_clk,
		.platform_device = hsic_device,
		.platform_device_size = ARRAY_SIZE(hsic_device),
	},
};

int whistler_baseband_init(void)
{
	int idx;
	int err;
	idx = baseband_type;

	if (whistler_baseband[idx].clk_init)
		tegra_clk_init_from_table(whistler_baseband[idx].clk_init);

	if (whistler_baseband[idx].platform_device_size)
		platform_add_devices(whistler_baseband[idx].platform_device,
			whistler_baseband[idx].platform_device_size);

	if (whistler_baseband[idx].spi_board_info_size) {
		err = spi_register_board_info(
			whistler_baseband[idx].spi_board_info,
			whistler_baseband[idx].spi_board_info_size);
		if (err < 0)
			pr_err("%s: spi_register_board returned error %d\n",
				__func__, err);
	}

	tegra_gpio_enable(TEGRA_CAIF_SSPI_GPIO_RESET);
	tegra_gpio_enable(TEGRA_CAIF_SSPI_GPIO_POWER);
	tegra_gpio_enable(TEGRA_CAIF_SSPI_GPIO_AWR);
	tegra_gpio_enable(TEGRA_CAIF_SSPI_GPIO_CWR);
	tegra_gpio_enable(TEGRA_CAIF_SSPI_GPIO_SPI_INT);
	tegra_gpio_enable(TEGRA_CAIF_SSPI_GPIO_SS);
	return 0;
}

static int rainbow_570_reset(void);
static int rainbow_570_handshake(void);
static int ph450_reset(void);
static int ph450_handshake(void);

static __initdata struct tegra_pingroup_config whistler_null_ulpi_pinmux[] = {
	{TEGRA_PINGROUP_UAA, TEGRA_MUX_ULPI, TEGRA_PUPD_NORMAL,
	 TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_UAB, TEGRA_MUX_ULPI, TEGRA_PUPD_NORMAL,
	 TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_UDA, TEGRA_MUX_ULPI, TEGRA_PUPD_NORMAL,
	 TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_UAC, TEGRA_MUX_RSVD4, TEGRA_PUPD_NORMAL,
	 TEGRA_TRI_NORMAL},
};

static struct tegra_ulpi_trimmer e951_trimmer = { 10, 1, 1, 1 };

static struct tegra_ulpi_config ehci2_null_ulpi_phy_config = {
	.inf_type = TEGRA_USB_NULL_ULPI,
	.trimmer = &e951_trimmer,
	.preinit = rainbow_570_reset,
	.postinit = rainbow_570_handshake,
};

static struct tegra_ehci_platform_data ehci2_null_ulpi_platform_data = {
	.operating_mode = TEGRA_USB_HOST,
	.power_down_on_bus_suspend = 0,
	.phy_config = &ehci2_null_ulpi_phy_config,
};

static int __init tegra_null_ulpi_init(void)
{
	tegra_ehci2_device.dev.platform_data = &ehci2_null_ulpi_platform_data;
	platform_device_register(&tegra_ehci2_device);
	return 0;
}

static int __init rainbow_570_init(void)
{
	int ret;

	ret = gpio_request(MODEM_PWR_ON, "mdm_power");
	if (ret)
		return ret;

	ret = gpio_request(MODEM_RESET, "mdm_reset");
	if (ret) {
		gpio_free(MODEM_PWR_ON);
		return ret;
	}
	ret = gpio_request(AWR, "mdm_awr");
	if (ret) {
		gpio_free(MODEM_PWR_ON);
		gpio_free(MODEM_RESET);
		return ret;
	}
	ret = gpio_request(CWR, "mdm_cwr");
	if (ret) {
		gpio_free(MODEM_PWR_ON);
		gpio_free(MODEM_RESET);
		gpio_free(AWR);
		return ret;
	}

	tegra_gpio_enable(MODEM_PWR_ON);
	tegra_gpio_enable(MODEM_RESET);
	tegra_gpio_enable(AWR);
	tegra_gpio_enable(CWR);

	gpio_direction_output(MODEM_PWR_ON, 0);
	gpio_direction_output(MODEM_RESET, 0);
	gpio_direction_output(AWR, 0);
	gpio_direction_input(CWR);

	return 0;
}

static int rainbow_570_reset(void)
{
	gpio_set_value(AWR, 0);
	gpio_set_value(MODEM_PWR_ON, 0);
	gpio_set_value(MODEM_RESET, 0);
	mdelay(300);
	gpio_set_value(MODEM_RESET, 1);
	mdelay(300);

	/* pulse modem power on for 1200 ms */
	gpio_set_value(MODEM_PWR_ON, 1);
	mdelay(1200);
	gpio_set_value(MODEM_PWR_ON, 0);
	mdelay(100);

	return 0;
}

static int rainbow_570_handshake(void)
{
	/* set AWR high */
	gpio_set_value(AWR, 1);

	/* wait for CWR high if modem firmware requires */

	return 0;
}

static int __init ph450_init(void)
{
	int ret;

	ret = gpio_request(MODEM_PWR_ON, "mdm_power");
	if (ret)
		return ret;

	ret = gpio_request(MODEM_RESET, "mdm_reset");
	if (ret) {
		gpio_free(MODEM_PWR_ON);
		return ret;
	}
	ret = gpio_request(AP2MDM_ACK2, "ap2mdm_ack2");
	if (ret) {
		gpio_free(MODEM_PWR_ON);
		gpio_free(MODEM_RESET);
		return ret;
	}
	ret = gpio_request(MDM2AP_ACK2, "mdm2ap_ack2");
	if (ret) {
		gpio_free(MODEM_PWR_ON);
		gpio_free(MODEM_RESET);
		gpio_free(AP2MDM_ACK2);
		return ret;
	}

	/* enable pull-up for MDM2AP_ACK2 */
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_UAC, TEGRA_PUPD_PULL_UP);

	tegra_gpio_enable(MODEM_PWR_ON);
	tegra_gpio_enable(MODEM_RESET);
	tegra_gpio_enable(AP2MDM_ACK2);
	tegra_gpio_enable(MDM2AP_ACK2);

	gpio_direction_output(MODEM_PWR_ON, 0);
	gpio_direction_output(MODEM_RESET, 0);
	gpio_direction_output(AP2MDM_ACK2, 1);
	gpio_direction_input(MDM2AP_ACK2);

	return 0;
}

static int ph450_reset(void)
{
	int retry = 100; /* retry for 10 sec */

	gpio_set_value(AP2MDM_ACK2, 1);
	gpio_set_value(MODEM_PWR_ON, 0);
	gpio_set_value(MODEM_RESET, 0);
	mdelay(200);
	gpio_set_value(MODEM_RESET, 1);
	mdelay(30);
	gpio_set_value(MODEM_PWR_ON, 1);

	while (retry) {
		/* wait for MDM2AP_ACK2 low */
		int val = gpio_get_value(MDM2AP_ACK2);
		if (!val)
			break;
		else
			retry--;
			mdelay(100);
	}

	return 1;
}

static int ph450_handshake(void)
{
	/* set AP2MDM_ACK2 low */
	gpio_set_value(AP2MDM_ACK2, 0);

	return 0;
}

int __init whistler_baseband_ph450_init(void)
{
	int ret;

	tegra_pinmux_config_table(whistler_null_ulpi_pinmux,
				  ARRAY_SIZE(whistler_null_ulpi_pinmux));

	ret = rainbow_570_init();
	if (ret) {
		pr_err("modem init failed\n");
		return ret;
	}

	tegra_null_ulpi_init();
	return 0;
}
