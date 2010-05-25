/*
 * arch/arm/mach-tegra/board-nvodm.c
 *
 * Converts data from ODM query library into platform data
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
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
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/dma-mapping.h>
#include <linux/regulator/machine.h>
#include <linux/lbee9qmb-rfkill.h>

#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/pinmux.h>
#include <mach/usb-hcd.h>
#include <mach/usb-otg.h>
#include <mach/serial.h>
#include <mach/sdhci.h>
#include <mach/nand.h>
#include <mach/regulator.h>

#include <mach/nvrm_linux.h>

#include "nvrm_gpio.h"
#include "nvodm_query.h"
#include "nvodm_query_pinmux.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query_gpio.h"
#include "nvrm_pinmux.h"
#include "nvrm_module.h"

NvRmGpioHandle s_hGpioGlobal;

static u64 tegra_dma_mask = DMA_BIT_MASK(32);

extern const struct tegra_pingroup_config *tegra_pinmux_get(const char *dev_id,
	int config, int *len);


static struct plat_serial8250_port debug_uart_platform[] = {
	{
		.flags = UPF_BOOT_AUTOCONF,
		.iotype = UPIO_MEM,
		.regshift = 2,
	}, {
		.flags = 0,
	}
};
static struct platform_device debug_uart = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = debug_uart_platform,
	},
};

static void __init tegra_setup_debug_uart(void)
{
	NvOdmDebugConsole uart = NvOdmQueryDebugConsole();
	const struct tegra_pingroup_config *pinmux = NULL;
	const NvU32 *odm_table;
	struct clk *c;
	NvU32 odm_nr;
	int nr_pins;

	if (uart < NvOdmDebugConsole_UartA ||
	    uart > NvOdmDebugConsole_UartE)
		return;

	NvOdmQueryPinMux(NvOdmIoModule_Uart, &odm_table, &odm_nr);
	if (odm_nr <= (uart - NvOdmDebugConsole_UartA)) {
		pr_err("%s: ODM query configured improperly\n", __func__);
		WARN_ON(1);
		return;
	}

	odm_nr = odm_table[uart - NvOdmDebugConsole_UartA];

	if (uart == NvOdmDebugConsole_UartA) {
		pinmux = tegra_pinmux_get("tegra_uart.0", odm_nr, &nr_pins);
		c = clk_get_sys("uart.0", NULL);
		debug_uart_platform[0].membase = IO_ADDRESS(TEGRA_UARTA_BASE);
		debug_uart_platform[0].mapbase = TEGRA_UARTA_BASE;
		debug_uart_platform[0].irq = INT_UARTA;
	} else if (uart == NvOdmDebugConsole_UartB) {
		pinmux = tegra_pinmux_get("tegra_uart.1", odm_nr, &nr_pins);
		c = clk_get_sys("uart.1", NULL);
		debug_uart_platform[0].membase = IO_ADDRESS(TEGRA_UARTB_BASE);
		debug_uart_platform[0].mapbase = TEGRA_UARTB_BASE;
		debug_uart_platform[0].irq = INT_UARTB;
	} else if (uart == NvOdmDebugConsole_UartC) {
		pinmux = tegra_pinmux_get("tegra_uart.2", odm_nr, &nr_pins);
		c = clk_get_sys("uart.2", NULL);
		debug_uart_platform[0].membase = IO_ADDRESS(TEGRA_UARTC_BASE);
		debug_uart_platform[0].mapbase = TEGRA_UARTC_BASE;
		debug_uart_platform[0].irq = INT_UARTC;
	} else if (uart == NvOdmDebugConsole_UartD) {
		pinmux = tegra_pinmux_get("tegra_uart.3", odm_nr, &nr_pins);
		c = clk_get_sys("uart.3", NULL);
		debug_uart_platform[0].membase = IO_ADDRESS(TEGRA_UARTD_BASE);
		debug_uart_platform[0].mapbase = TEGRA_UARTD_BASE;
		debug_uart_platform[0].irq = INT_UARTD;
	} else if (uart == NvOdmDebugConsole_UartE) {
		pinmux = tegra_pinmux_get("tegra_uart.4", odm_nr, &nr_pins);
		c = clk_get_sys("uart.4", NULL);
		debug_uart_platform[0].membase = IO_ADDRESS(TEGRA_UARTE_BASE);
		debug_uart_platform[0].mapbase = TEGRA_UARTE_BASE;
		debug_uart_platform[0].irq = INT_UARTE;
	}

	if (!c || !pinmux || !nr_pins) {
		if (c)
			clk_put(c);
		return;
	}

	tegra_pinmux_config_tristate_table(pinmux, nr_pins, TEGRA_TRI_NORMAL);
	clk_set_rate(c, 115200*16);
	clk_enable(c);
	debug_uart_platform[0].uartclk = clk_get_rate(c);
	clk_put(c);

	platform_device_register(&debug_uart);
}

#ifdef CONFIG_MMC_SDHCI_TEGRA
extern struct tegra_nand_platform tegra_nand_plat;
static struct tegra_sdhci_platform_data tegra_sdhci_platform[] = {
	[0] = {
		.bus_width = 4,
		.debounce = 5,
	},
	[1] = {
		.bus_width = 4,
		.debounce = 5,
	},
	[2] = {
		.bus_width = 4,
		.debounce = 5,
	},
	[3] = {
		.bus_width = 4,
		.debounce = 5,
	},
};
static struct resource tegra_sdhci_resources[][2] = {
	[0] = {
		[0] = {
			.start = TEGRA_SDMMC1_BASE,
			.end = TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = INT_SDMMC1,
			.end = INT_SDMMC1,
			.flags = IORESOURCE_IRQ,
		},
	},
	[1] = {
		[0] = {
			.start = TEGRA_SDMMC2_BASE,
			.end = TEGRA_SDMMC2_BASE + TEGRA_SDMMC2_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = INT_SDMMC2,
			.end = INT_SDMMC2,
			.flags = IORESOURCE_IRQ,
		},
	},
	[2] = {
		[0] = {
			.start = TEGRA_SDMMC3_BASE,
			.end = TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = INT_SDMMC3,
			.end = INT_SDMMC3,
			.flags = IORESOURCE_IRQ,
		},
	},
	[3] = {
		[0] = {
			.start = TEGRA_SDMMC4_BASE,
			.end = TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = INT_SDMMC4,
			.end = INT_SDMMC4,
			.flags = IORESOURCE_IRQ,
		},
	},
};
static struct platform_device tegra_sdhci_devices[] = {
	[0] = {
		.id = 0,
		.name = "tegra-sdhci",
		.resource = tegra_sdhci_resources[0],
		.num_resources = ARRAY_SIZE(tegra_sdhci_resources[0]),
		.dev = {
			.platform_data = &tegra_sdhci_platform[0],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	[1] = {
		.id = 1,
		.name = "tegra-sdhci",
		.resource = tegra_sdhci_resources[1],
		.num_resources = ARRAY_SIZE(tegra_sdhci_resources[1]),
		.dev = {
			.platform_data = &tegra_sdhci_platform[1],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	[2] = {
		.id = 2,
		.name = "tegra-sdhci",
		.resource = tegra_sdhci_resources[2],
		.num_resources = ARRAY_SIZE(tegra_sdhci_resources[2]),
		.dev = {
			.platform_data = &tegra_sdhci_platform[2],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	[3] = {
		.id = 3,
		.name = "tegra-sdhci",
		.resource = tegra_sdhci_resources[3],
		.num_resources = ARRAY_SIZE(tegra_sdhci_resources[3]),
		.dev = {
			.platform_data = &tegra_sdhci_platform[3],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
};

#define active_high(_pin) ((_pin)->activeState == NvOdmGpioPinActiveState_High ? 1 : 0)

static void __init tegra_setup_sdhci(void) {
	const NvOdmGpioPinInfo *gpio;
	struct tegra_sdhci_platform_data *plat;
	const NvU32 *clock_limits;
	const NvU32 *pinmux;
	NvU32 nr_pinmux;
	NvU32 clock_count;
	NvU32 gpio_count;
	NvRmModuleSdmmcInterfaceCaps caps;
	int i;

	NvOdmQueryClockLimits(NvOdmIoModule_Sdio, &clock_limits, &clock_count);
	NvOdmQueryPinMux(NvOdmIoModule_Sdio, &pinmux, &nr_pinmux);

#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	/* check if an "MBR" partition was parsed from the tegra partition
	 * command line, and store it in sdhci.3's offset field */
	for (i=0; i<tegra_nand_plat.nr_parts; i++) {
		plat = &tegra_sdhci_platform[3];
		if (strcmp("mbr", tegra_nand_plat.parts[i].name))
			continue;
		plat->offset = tegra_nand_plat.parts[i].offset;
	}
#endif

	for (i=0; i<ARRAY_SIZE(tegra_sdhci_platform); i++) {
		const NvOdmQuerySdioInterfaceProperty *prop;
		prop = NvOdmQueryGetSdioInterfaceProperty(i);
		if (!i || prop->usage==NvOdmQuerySdioSlotUsage_unused)
			continue;

		plat = &tegra_sdhci_platform[i];
		gpio = NvOdmQueryGpioPinMap(NvOdmGpioPinGroup_Sdio,
			i, &gpio_count);
		if (!gpio)
			gpio_count = 0;
		switch (gpio_count) {
		case 2:
			plat->gpio_nr_wp = 8*gpio[1].Port + gpio[1].Pin;
			plat->gpio_nr_cd = 8*gpio[0].Port + gpio[0].Pin;
			plat->gpio_polarity_wp = active_high(&gpio[1]);
			plat->gpio_polarity_cd = active_high(&gpio[0]);
			break;
		case 1:
			plat->gpio_nr_wp = -1;
			plat->gpio_nr_cd = 8*gpio[0].Port + gpio[0].Pin;
			plat->gpio_polarity_cd = active_high(&gpio[0]);
			break;
		case 0:
			plat->gpio_nr_wp = -1;
			plat->gpio_nr_cd = -1;
			break;
		}

		if (NvRmGetModuleInterfaceCapabilities(s_hRmGlobal,
			NVRM_MODULE_ID(NvRmModuleID_Sdio, i),
			sizeof(caps), &caps)==NvSuccess)
			plat->bus_width = caps.MmcInterfaceWidth;

		if (clock_limits && i<clock_count)
			plat->max_clk = clock_limits[i] * 1000;

		if (pinmux && i<nr_pinmux) {
			char name[20];
			snprintf(name, sizeof(name), "tegra-sdhci.%d", i);
			plat->pinmux = tegra_pinmux_get(name,
				pinmux[i], &plat->nr_pins);
		}

		platform_device_register(&tegra_sdhci_devices[i]);
	}
}
#else
static void __init tegra_setup_sdhci(void) { }
#endif

#ifdef CONFIG_SERIAL_TEGRA
static struct tegra_serial_platform_data tegra_uart_platform[] = {
	{
		.p = {
			.membase = IO_ADDRESS(TEGRA_UARTA_BASE),
			.mapbase = TEGRA_UARTA_BASE,
			.irq = INT_UARTA,
		},
	},
	{
		.p = {
			.membase = IO_ADDRESS(TEGRA_UARTB_BASE),
			.mapbase = TEGRA_UARTB_BASE,
			.irq = INT_UARTB,
		},
	},
	{
		.p = {
			.membase = IO_ADDRESS(TEGRA_UARTC_BASE),
			.mapbase = TEGRA_UARTC_BASE,
			.irq = INT_UARTC,
		},
	},
	{
		.p = {
			.membase = IO_ADDRESS(TEGRA_UARTD_BASE),
			.mapbase = TEGRA_UARTD_BASE,
			.irq = INT_UARTD,
		},
	},
	{
		.p = {
			.membase = IO_ADDRESS(TEGRA_UARTE_BASE),
			.mapbase = TEGRA_UARTE_BASE,
			.irq = INT_UARTE,
		},
	},
};
static struct platform_device tegra_uart[] = {
	{
		.name = "tegra_uart",
		.id = 0,
		.dev = {
			.platform_data = &tegra_uart_platform[0],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	{
		.name = "tegra_uart",
		.id = 1,
		.dev = {
			.platform_data = &tegra_uart_platform[1],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	{
		.name = "tegra_uart",
		.id = 2,
		.dev = {
			.platform_data = &tegra_uart_platform[2],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	{
		.name = "tegra_uart",
		.id = 3,
		.dev = {
			.platform_data = &tegra_uart_platform[3],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	{
		.name = "tegra_uart",
		.id = 4,
		.dev = {
			.platform_data = &tegra_uart_platform[4],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},

};
static void __init tegra_setup_hsuart(void)
{
	NvOdmDebugConsole uart = NvOdmQueryDebugConsole();
	int dbg_id = (int)uart - (int)NvOdmDebugConsole_UartA;
	const NvU32 *odm_table;
	NvU32 odm_nr;
	int i;

	NvOdmQueryPinMux(NvOdmIoModule_Uart, &odm_table, &odm_nr);

	for (i=0; i<ARRAY_SIZE(tegra_uart); i++) {
		struct tegra_serial_platform_data *plat;
		char name[16];

		if (i==dbg_id)
			continue;

		plat = &tegra_uart_platform[i];

		snprintf(name, sizeof(name), "%s.%d",
			 tegra_uart[i].name, tegra_uart[i].id);

		if (i < odm_nr) {
			plat->pinmux = tegra_pinmux_get(name,
				odm_table[i], &plat->nr_pins);
		} else {
			plat->pinmux = NULL;
			plat->nr_pins = 0;
		}

		if (platform_device_register(&tegra_uart[i])) {
			pr_err("%s: failed to register %s.%d\n",
			       __func__, tegra_uart[i].name, tegra_uart[i].id);
		}
	}
}
#else
static void __init tegra_setup_hsuart(void) { }
#endif

#ifdef CONFIG_USB_TEGRA_HCD
static struct tegra_hcd_platform_data tegra_hcd_platform[] = {
	[0] = {
		.instance = 0,
	},
	[1] = {
		.instance = 1,
	},
	[2] = {
		.instance = 2,
	},
};
static struct resource tegra_hcd_resources[][2] = {
	[0] = {
		[0] = {
			.flags = IORESOURCE_MEM,
			.start = TEGRA_USB_BASE,
			.end = TEGRA_USB_BASE + TEGRA_USB_SIZE - 1,
		},
		[1] = {
			.flags = IORESOURCE_IRQ,
			.start = INT_USB,
			.end = INT_USB,
		},
	},
	[1] = {
		[0] = {
			.flags = IORESOURCE_MEM,
			.start = TEGRA_USB1_BASE,
			.end = TEGRA_USB1_BASE + TEGRA_USB1_SIZE - 1,
		},
		[1] = {
			.flags = IORESOURCE_IRQ,
			.start = INT_USB2,
			.end = INT_USB2,
		},
	},
	[2] = {
		[0] = {
			.flags = IORESOURCE_MEM,
			.start = TEGRA_USB2_BASE,
			.end = TEGRA_USB2_BASE + TEGRA_USB2_SIZE - 1,
		},
		[1] = {
			.flags = IORESOURCE_IRQ,
			.start = INT_USB3,
			.end = INT_USB3,
		},
	},
};
static struct platform_device tegra_hcd[] = {
	[0] = {
		.name = "tegra-ehci",
		.id = 0,
		.dev = {
			.platform_data = &tegra_hcd_platform[0],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
		.resource = tegra_hcd_resources[0],
		.num_resources = ARRAY_SIZE(tegra_hcd_resources[0]),
	},
	[1] = {
		.name = "tegra-ehci",
		.id = 1,
		.dev = {
			.platform_data = &tegra_hcd_platform[1],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
		.resource = tegra_hcd_resources[1],
		.num_resources = ARRAY_SIZE(tegra_hcd_resources[1]),
	},
	[2] = {
		.name = "tegra-ehci",
		.id = 2,
		.dev = {
			.platform_data = &tegra_hcd_platform[2],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
		.resource = tegra_hcd_resources[2],
		.num_resources = ARRAY_SIZE(tegra_hcd_resources[2]),
	},
};

#ifdef CONFIG_USB_TEGRA_OTG
#define otg_is_okay(_instance) ((_instance)==0)
static struct tegra_otg_platform_data tegra_otg_platform = {
	.instance = 0,
};
static struct resource tegra_otg_resources[] = {
	[0] = {
		.start = TEGRA_USB_BASE,
		.end = TEGRA_USB_BASE + TEGRA_USB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_USB,
		.end = INT_USB,
		.flags = IORESOURCE_IRQ,
	},
};
static struct platform_device tegra_otg = {
	.name = "tegra-otg",
	.id = 0,
	.resource = tegra_otg_resources,
	.num_resources = ARRAY_SIZE(tegra_otg_resources),
	.dev = {
		.platform_data = &tegra_otg_platform,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.dma_mask = &tegra_dma_mask,
	},
};
#else
#define otg_is_okay(_instance) (0)
#endif

static void __init tegra_setup_hcd(void)
{
	int i;

	for (i=0; i<ARRAY_SIZE(tegra_hcd_platform); i++) {
		const NvOdmUsbProperty *p;
		struct tegra_hcd_platform_data *plat = &tegra_hcd_platform[i];

		p = NvOdmQueryGetUsbProperty(NvOdmIoModule_Usb, i);

		if (p->UsbMode == NvOdmUsbModeType_Device)
			continue;

		plat->otg_mode = (p->UsbMode == NvOdmUsbModeType_OTG);
		if (plat->otg_mode && !otg_is_okay(i)) {
			pr_err("%s: OTG not enabled in kernel for USB "
			       "controller %d, but ODM kit specifes OTG\n",
			       __func__, i);
			continue;
		}
#ifdef CONFIG_USB_TEGRA_OTG
		if (plat->otg_mode && otg_is_okay(i)) {
			tegra_otg_platform.usb_property = p;
			platform_device_register(&tegra_otg);
		}
#endif
		if (p->IdPinDetectionType == NvOdmUsbIdPinType_Gpio) {
			const NvOdmGpioPinInfo *gpio;
			NvU32 count;

			gpio = NvOdmQueryGpioPinMap(NvOdmGpioPinGroup_Usb,
						    i, &count);
			if (!gpio || (count<=NvOdmGpioPin_UsbCableId)) {
				pr_err("%s: invalid ODM query for controller "
				       "%d\n", __func__, i);
				WARN_ON(1);
				continue;
			}
			plat->id_detect = ID_PIN_GPIO;
			gpio += NvOdmGpioPin_UsbCableId;
			plat->gpio_nr = gpio->Port*8 + gpio->Pin;
		} else if (p->IdPinDetectionType == NvOdmUsbIdPinType_CableId) {
			plat->id_detect = ID_PIN_CABLE_ID;
		}
		platform_device_register(&tegra_hcd[i]);
	}
}
#else
static inline void tegra_setup_hcd(void) { }
#endif

#ifdef CONFIG_RTC_DRV_TEGRA_ODM
static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
};
#endif
#ifdef CONFIG_TEGRA_NVEC
static struct platform_device tegra_nvec_device = {
	.name = "nvec",
	.id = -1,
};
#endif
#ifdef CONFIG_TEGRA_BATTERY_NVEC
static struct platform_device tegra_battery_device = {
	.name = "tegra_battery",
	.id = -1,
};
#endif
#ifdef CONFIG_REGULATOR_TEGRA
static struct regulator_consumer_supply pex_clk_consumers[] = {
	[0] = {
		.supply = "pex_clk",
	},
};
static struct regulator_consumer_supply lbee9qmb_consumers[] = {
	[0] = {
		.supply = "Vdd",
		.dev_name = "lbee9qmb-rfkill.0",
	},
};
static struct tegra_regulator_entry tegra_regulators[] = {
	[0] = {
		.guid = NV_VDD_PEX_CLK_ODM_ID,
		.name = "pex_clk",
		.id = 0,
		.consumers = pex_clk_consumers,
		.nr_consumers = ARRAY_SIZE(pex_clk_consumers),
	},
	[1] = {
		.guid = NV_ODM_GUID('b','l','u','t','o','o','t','h'),
		.name = "lbee9qmb_vdd",
		.id = 1,
		.consumers = lbee9qmb_consumers,
		.nr_consumers = ARRAY_SIZE(lbee9qmb_consumers),
	},
};
static struct tegra_regulator_platform_data tegra_regulator_platform = {
	.regs = tegra_regulators,
	.nr_regs = ARRAY_SIZE(tegra_regulators),
};
static struct platform_device tegra_regulator_device = {
	.name = "tegra_regulator",
	.id = -1,
	.dev = {
		.platform_data = &tegra_regulator_platform,
	},
};
#endif
#ifdef CONFIG_LBEE9QMB_RFKILL
static struct lbee9qmb_platform_data lbee9qmb_platform;
static struct platform_device lbee9qmb_device = {
	.name = "lbee9qmb-rfkill",
	.dev = {
		.platform_data = &lbee9qmb_platform,
	},
};
static void tegra_setup_rfkill(void)
{
	const NvOdmPeripheralConnectivity *con;
	unsigned int i;

	con = NvOdmPeripheralGetGuid(NV_ODM_GUID('b','l','u','t','o','o','t','h'));
	if (!con)
		return;

	for (i=0; i<con->NumAddress; i++) {
		if (con->AddressList[i].Interface == NvOdmIoModule_Gpio) {
			int nr_gpio = con->AddressList[i].Instance * 8 +
				con->AddressList[i].Address;
			lbee9qmb_platform.gpio_reset = nr_gpio;
			if (platform_device_register(&lbee9qmb_device))
				pr_err("%s: registration failed\n", __func__);
			return;
		}
	}
}
#else
static void tegra_setup_rfkill(void) { }
#endif

static struct platform_device *nvodm_devices[] __initdata = {
#ifdef CONFIG_RTC_DRV_TEGRA_ODM
	&tegra_rtc_device,
#endif
#ifdef CONFIG_TEGRA_NVEC
	&tegra_nvec_device,
#endif
#ifdef CONFIG_TEGRA_BATTERY_NVEC
	&tegra_battery_device,
#endif
#ifdef CONFIG_REGULATOR_TEGRA
	&tegra_regulator_device,
#endif
};

#ifdef CONFIG_TEGRA_PCI
extern void __init tegra_pcie_init(void);
static int tegra_setup_pcie(void)
{
	const struct tegra_pingroup_config *pinmux = NULL;
	const NvU32 *odmpinmux;
	NvU32 nr_configs;
	int nr_pins;

	NvOdmQueryPinMux(NvOdmIoModule_PciExpress, &odmpinmux, &nr_configs);
	if (!odmpinmux || !nr_configs) {
		pr_info("%s: PCIE not supported on platform\n", __func__);
		return 0;
	}
	pinmux = tegra_pinmux_get("tegra_pcie",	odmpinmux[0], &nr_pins);
	tegra_pinmux_config_tristate_table(pinmux, nr_pins, TEGRA_TRI_NORMAL);
	tegra_pcie_init();
	return 0;
}
late_initcall(tegra_setup_pcie);
#endif

void __init tegra_setup_nvodm(void)
{
	NvRmGpioOpen(s_hRmGlobal, &s_hGpioGlobal);
	tegra_setup_debug_uart();
	tegra_setup_hcd();
	tegra_setup_hsuart();
	tegra_setup_sdhci();
	tegra_setup_rfkill();
	platform_add_devices(nvodm_devices, ARRAY_SIZE(nvodm_devices));
}
