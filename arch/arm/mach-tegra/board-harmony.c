/*
 * arch/arm/mach-tegra/board-harmony.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2011 NVIDIA, Inc.
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
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>
#include <linux/pda_power.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/memblock.h>
#include <linux/delay.h>
#include <linux/mfd/tps6586x.h>

#include <sound/wm8903.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/tegra_wm8903_pdata.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/sdhci.h>
#include <mach/nand.h>
#include <mach/clk.h>
#include <mach/usb_phy.h>

#include "clock.h"
#include "board.h"
#include "board-harmony.h"
#include "clock.h"
#include "devices.h"
#include "gpio-names.h"

/* NVidia bootloader tags */
#define ATAG_NVIDIA		0x41000801

#define ATAG_NVIDIA_RM			0x1
#define ATAG_NVIDIA_DISPLAY		0x2
#define ATAG_NVIDIA_FRAMEBUFFER		0x3
#define ATAG_NVIDIA_CHIPSHMOO		0x4
#define ATAG_NVIDIA_CHIPSHMOOPHYS	0x5
#define ATAG_NVIDIA_PRESERVED_MEM_0	0x10000
#define ATAG_NVIDIA_PRESERVED_MEM_N	2
#define ATAG_NVIDIA_FORCE_32		0x7fffffff

struct tag_tegra {
	__u32 bootarg_key;
	__u32 bootarg_len;
	char bootarg[1];
};

static int __init parse_tag_nvidia(const struct tag *tag)
{

	return 0;
}
__tagtable(ATAG_NVIDIA, parse_tag_nvidia);

static struct tegra_utmip_config utmi_phy_config = {
	.hssync_start_delay = 0,
	.idle_wait_delay = 17,
	.elastic_limit = 16,
	.term_range_adj = 6,
	.xcvr_setup = 9,
	.xcvr_lsfslew = 2,
	.xcvr_lsrslew = 2,
};

static struct tegra_ehci_platform_data tegra_ehci_pdata = {
	.phy_config = &utmi_phy_config,
	.operating_mode = TEGRA_USB_HOST,
	.power_down_on_bus_suspend = 1,
};

static struct tegra_nand_chip_parms nand_chip_parms[] = {
	/* Samsung K5E2G1GACM */
	[0] = {
	       .vendor_id = 0xEC,
	       .device_id = 0xAA,
	       .read_id_fourth_byte = 0x15,
	       .capacity  = 256,
	       .timing = {
			  .trp = 21,
			  .trh = 15,
			  .twp = 21,
			  .twh = 15,
			  .tcs = 31,
			  .twhr = 60,
			  .tcr_tar_trr = 20,
			  .twb = 100,
			  .trp_resp = 30,
			  .tadl = 100,
			  },
	       },
	/* Hynix H5PS1GB3EFR */
	[1] = {
	       .vendor_id = 0xAD,
	       .device_id = 0xDC,
	       .read_id_fourth_byte = 0x95,
	       .capacity  = 512,
	       .timing = {
			  .trp = 12,
			  .trh = 10,
			  .twp = 12,
			  .twh = 10,
			  .tcs = 20,
			  .twhr = 80,
			  .tcr_tar_trr = 20,
			  .twb = 100,
			  .trp_resp = 20,
			  .tadl = 70,
			  },
	       },
};

struct tegra_nand_platform harmony_nand_data = {
	.max_chips = 8,
	.chip_parms = nand_chip_parms,
	.nr_chip_parms = ARRAY_SIZE(nand_chip_parms),
	.wp_gpio = TEGRA_GPIO_PC7,
};

static struct resource resources_nand[] = {
	[0] = {
	       .start = INT_NANDFLASH,
	       .end = INT_NANDFLASH,
	       .flags = IORESOURCE_IRQ,
	       },
};

struct platform_device tegra_nand_device = {
	.name = "tegra_nand",
	.id = -1,
	.num_resources = ARRAY_SIZE(resources_nand),
	.resource = resources_nand,
	.dev = {
		.platform_data = &harmony_nand_data,
		},
};

static struct plat_serial8250_port debug_uart_platform_data[] = {
	{
		.membase	= IO_ADDRESS(TEGRA_UARTD_BASE),
		.mapbase	= TEGRA_UARTD_BASE,
		.irq		= INT_UARTD,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 216000000,
	}, {
		.flags		= 0
	}
};

static struct gpio_keys_button harmony_gpio_keys_buttons[] = {
	{
		.code		= KEY_POWER,
		.gpio		= TEGRA_GPIO_POWERKEY,
		.active_low	= 1,
		.desc		= "Power",
		.type		= EV_KEY,
		.wakeup		= 1,
	},
};

static struct gpio_keys_platform_data harmony_gpio_keys = {
	.buttons	= harmony_gpio_keys_buttons,
	.nbuttons	= ARRAY_SIZE(harmony_gpio_keys_buttons),
};

static struct platform_device harmony_gpio_keys_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data = &harmony_gpio_keys,
	}
};

static struct platform_device debug_uart = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = debug_uart_platform_data,
	},
};

static void harmony_keys_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(harmony_gpio_keys_buttons); i++)
		tegra_gpio_enable(harmony_gpio_keys_buttons[i].gpio);
}

static struct tegra_wm8903_platform_data harmony_audio_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= TEGRA_GPIO_INT_MIC_EN,
	.gpio_ext_mic_en	= TEGRA_GPIO_EXT_MIC_EN,
};

static struct platform_device harmony_audio_device = {
	.name	= "tegra-snd-wm8903",
	.id	= 0,
	.dev	= {
		.platform_data  = &harmony_audio_pdata,
	},
};

static struct tegra_i2c_platform_data harmony_i2c1_platform_data = {
	.adapter_nr     = 0,
	.bus_count      = 1,
	.bus_clk_rate   = { 400000, 0 },
};

static const struct tegra_pingroup_config i2c2_ddc = {
	.pingroup       = TEGRA_PINGROUP_DDC,
	.func           = TEGRA_MUX_I2C2,
};

static const struct tegra_pingroup_config i2c2_gen2 = {
	.pingroup       = TEGRA_PINGROUP_PTA,
	.func           = TEGRA_MUX_I2C2,
};

static struct tegra_i2c_platform_data harmony_i2c2_platform_data = {
	.adapter_nr     = 1,
	.bus_count      = 2,
	.bus_clk_rate   = { 100000, 100000 },
	.bus_mux        = { &i2c2_ddc, &i2c2_gen2 },
	.bus_mux_len    = { 1, 1 },
};

static struct tegra_i2c_platform_data harmony_i2c3_platform_data = {
	.adapter_nr     = 3,
	.bus_count      = 1,
	.bus_clk_rate   = { 400000, 0 },
};

static struct tegra_i2c_platform_data harmony_dvc_platform_data = {
	.adapter_nr     = 4,
	.bus_count      = 1,
	.bus_clk_rate   = { 400000, 0 },
	.is_dvc         = true,
};

static struct wm8903_platform_data harmony_wm8903_pdata = {
	.irq_active_low = 0,
	.micdet_cfg = 0,
	.micdet_delay = 100,
	.gpio_base = HARMONY_GPIO_WM8903(0),
	.gpio_cfg = {
		WM8903_GPIO_NO_CONFIG,
		WM8903_GPIO_NO_CONFIG,
		0,
		WM8903_GPIO_NO_CONFIG,
		WM8903_GPIO_NO_CONFIG,
	},
};

static struct i2c_board_info __initdata wm8903_board_info = {
	I2C_BOARD_INFO("wm8903", 0x1a),
	.platform_data = &harmony_wm8903_pdata,
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CDC_IRQ),
};

static void __init harmony_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &harmony_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &harmony_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &harmony_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &harmony_dvc_platform_data;

	platform_device_register(&tegra_i2c_device1);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device4);

	i2c_register_board_info(0, &wm8903_board_info, 1);
}

/* OTG gadget device */
/*static u64 tegra_otg_dmamask = DMA_BIT_MASK(32);


static struct resource tegra_otg_resources[] = {
	[0] = {
		.start  = TEGRA_USB_BASE,
		.end    = TEGRA_USB_BASE + TEGRA_USB_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = INT_USB,
		.end    = INT_USB,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct fsl_usb2_platform_data tegra_otg_pdata = {
	.operating_mode	= FSL_USB2_DR_DEVICE,
	.phy_mode	= FSL_USB2_PHY_UTMI,
};

static struct platform_device tegra_otg = {
	.name = "fsl-tegra-udc",
	.id   = -1,
	.dev  = {
		.dma_mask		= &tegra_otg_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data = &tegra_otg_pdata,
	},
	.resource = tegra_otg_resources,
	.num_resources = ARRAY_SIZE(tegra_otg_resources),
};*/

/* PDA power */
static struct pda_power_pdata pda_power_pdata = {
};

static struct platform_device pda_power_device = {
	.name   = "pda_power",
	.id     = -1,
	.dev    = {
		.platform_data  = &pda_power_pdata,
	},
};

static struct platform_device *harmony_devices[] __initdata = {
	&debug_uart,
	&tegra_sdhci_device1,
	&tegra_sdhci_device2,
	&tegra_sdhci_device4,
	&tegra_i2s_device1,
	&tegra_spdif_device,
	&tegra_das_device,
	&spdif_dit_device,
	&tegra_pcm_device,
	&harmony_audio_device,
	&tegra_pmu_device,
	&tegra_nand_device,
	&tegra_udc_device,
	&harmony_gpio_keys_device,
	&pda_power_device,
	&tegra_ehci3_device,
	&tegra_spi_device1,
	&tegra_spi_device2,
	&tegra_spi_device3,
	&tegra_spi_device4,
	&tegra_gart_device,
};

static void __init tegra_harmony_fixup(struct machine_desc *desc,
	struct tag *tags, char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 2;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].size = 448 * SZ_1M;
	mi->bank[1].start = SZ_512M;
	mi->bank[1].size = SZ_512M;
}

static __initdata struct tegra_clk_init_table harmony_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "uartd",	"pll_p",	216000000,	true },
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "sdmmc1",	"clk_m",	48000000,	true },
	{ "sdmmc2",	"clk_m",	48000000,	true },
	{ "sdmmc4",	"clk_m",	48000000,	true },
	{ "ndflash",	"pll_p",	108000000,	true},
	{ "pwm",	"clk_32k",	32768,		false},
	{ NULL,		NULL,		0,		0},
};


static struct tegra_sdhci_platform_data sdhci_pdata1 = {
	.cd_gpio	= -1,
	.wp_gpio	= -1,
	.power_gpio	= -1,
};

static struct tegra_sdhci_platform_data sdhci_pdata2 = {
	.cd_gpio	= TEGRA_GPIO_SD2_CD,
	.wp_gpio	= TEGRA_GPIO_SD2_WP,
	.power_gpio	= TEGRA_GPIO_SD2_POWER,
};

static struct tegra_sdhci_platform_data sdhci_pdata4 = {
	.cd_gpio	= TEGRA_GPIO_SD4_CD,
	.wp_gpio	= TEGRA_GPIO_SD4_WP,
	.power_gpio	= TEGRA_GPIO_SD4_POWER,
	.is_8bit	= 1,
};

static int __init harmony_wifi_init(void)
{
        int gpio_pwr, gpio_rst;

	if (!machine_is_harmony())
		return 0;

        /* WLAN - Power up (low) and Reset (low) */
        gpio_pwr = gpio_request(TEGRA_GPIO_WLAN_PWR_LOW, "wlan_pwr");
        gpio_rst = gpio_request(TEGRA_GPIO_WLAN_RST_LOW, "wlan_rst");
        if (gpio_pwr < 0 || gpio_rst < 0)
                pr_warning("Unable to get gpio for WLAN Power and Reset\n");
        else {

		tegra_gpio_enable(TEGRA_GPIO_WLAN_PWR_LOW);
		tegra_gpio_enable(TEGRA_GPIO_WLAN_RST_LOW);
                /* toggle in this order as per spec */
                gpio_direction_output(TEGRA_GPIO_WLAN_PWR_LOW, 0);
                gpio_direction_output(TEGRA_GPIO_WLAN_RST_LOW, 0);
		udelay(5);
                gpio_direction_output(TEGRA_GPIO_WLAN_PWR_LOW, 1);
                gpio_direction_output(TEGRA_GPIO_WLAN_RST_LOW, 1);
        }

	return 0;
}

/*
 * subsys_initcall_sync is good synch point to call harmony_wifi_init
 * This makes sure that the required regulators (LDO3
 * supply of external PMU and 1.2V regulator) are properly enabled,
 * and mmc driver has not yet probed for a device on SDIO bus.
 */
subsys_initcall_sync(harmony_wifi_init);

static void harmony_power_off(void)
{
	int ret;

	ret = tps6586x_power_off();
	if (ret)
		pr_err("harmony: failed to power off\n");

	while (1);
}

static void __init harmony_power_off_init(void)
{
	pm_power_off = harmony_power_off;
}

static void __init tegra_harmony_init(void)
{
	tegra_clk_init_from_table(harmony_clk_init_table);

	harmony_pinmux_init();

	harmony_keys_init();

	tegra_sdhci_device1.dev.platform_data = &sdhci_pdata1;
	tegra_sdhci_device2.dev.platform_data = &sdhci_pdata2;
	tegra_sdhci_device4.dev.platform_data = &sdhci_pdata4;

	tegra_ehci3_device.dev.platform_data = &tegra_ehci_pdata;

	platform_add_devices(harmony_devices, ARRAY_SIZE(harmony_devices));
	harmony_i2c_init();
	harmony_regulator_init();
	harmony_suspend_init();
	harmony_panel_init();
#ifdef CONFIG_KEYBOARD_TEGRA
	harmony_kbc_init();
#endif
	harmony_pcie_init();
	harmony_power_off_init();
}

void __init tegra_harmony_reserve(void)
{
	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	tegra_reserve(SZ_128M, SZ_8M, 0);
}

MACHINE_START(HARMONY, "harmony")
	.boot_params  = 0x00000100,
	.fixup		= tegra_harmony_fixup,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_harmony_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_harmony_init,
MACHINE_END
