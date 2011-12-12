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
#include <linux/gpio.h>
#include <linux/console.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#ifdef CONFIG_TOUCHSCREEN_PANJIT_I2C
#include <linux/i2c/panjit_ts.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_ATMEL_MT_T9
#include <linux/i2c/atmel_maxtouch.h>
#endif

//20100419  for headset detetion [LGE_START]
#if defined(CONFIG_MACH_STAR)
#include <linux/switch.h>	//20100419  for Headset Detection [LGE]
#endif /* CONFIG_MACH_STAR */
//20100419  for headset detection [LGE_END]

#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/pinmux.h>
#include <mach/usb-hcd.h>
#include <mach/usb-otg.h>
#include <mach/serial.h>
#include <mach/sdhci.h>
#include <mach/nand.h>
#include <mach/regulator.h>
#include <mach/kbc.h>
#include <mach/i2c.h>
#include <mach/spi.h>
#include <mach/w1.h>

#include <mach/nvrm_linux.h>

#include "nvrm_gpio.h"
#include "nvodm_query.h"
#include "nvodm_query_pinmux.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query_gpio.h"
#include "nvrm_pinmux.h"
#include "nvrm_module.h"
#include "nvodm_kbc.h"
#include "nvodm_query_kbc.h"
#include "nvodm_kbc_keymapping.h"
#include "gpio-names.h"
#include "power.h"
#include "board.h"
#include "nvrm_pmu.h"

# define BT_RESET 0
# define BT_SHUTDOWN 1

#if defined(CONFIG_KEYBOARD_GPIO)
#include "nvodm_query_gpio.h"
#include <linux/gpio_keys.h>
#include <linux/input.h>
#endif

//20100830, gunwoo1.kim, soft reset [START]
#if defined(CONFIG_INPUT_KEYRESET)
#include <linux/keyreset.h>
#endif
//20100830, gunwoo1.kim, soft reset [END]

//20100724  for poweroff leakage [LGE_START]
#include "odm_kit/star/adaptations/pmu/max8907/max8907_supply_info_table.h"
#include <linux/delay.h>
//20100724  for poweroff leakage [LGE_END]
#include <linux/rtc.h>

#if 1
extern void NvRmPrivDvsStop(void);
extern void star_emergency_restart(const char *domain, int timeout);
#endif

extern NvBool IsBoardTango(void);

NvRmGpioHandle s_hGpioGlobal;

struct debug_port_data {
	NvOdmDebugConsole port;
	const struct tegra_pingroup_config *pinmux;
	struct clk *clk_data;
	int nr_pins;
};

static u64 tegra_dma_mask = DMA_BIT_MASK(32);

static struct debug_port_data uart_debug_port = {
			.port = NvOdmDebugConsole_None,
};

extern const struct tegra_pingroup_config *tegra_pinmux_get(const char *dev_id,
	int config, int *len);


static struct plat_serial8250_port debug_uart_platform[] = {
	{
		/* Force the debug console UART port type to PORT_TEGRA.*/
		.flags = UPF_BOOT_AUTOCONF | UPF_FIXED_TYPE,
		.type = PORT_TEGRA,
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

#define TEGRA_EMC_MRW          0x00e8
static void __iomem *emc = IO_ADDRESS(TEGRA_EMC_BASE);
static inline void emc_writel(u32 val, unsigned long addr)
{
	writel(val, emc + addr);
}

/* read LPDDR2 memory modes */
void tegra_emc_write_mrw(unsigned long addr)
{
	emc_writel(0x90000, TEGRA_EMC_MRW);    
	printk("Sending 0xB0 to TEGRA_EMC_MRW\n");
	emc_writel(0x900B0, TEGRA_EMC_MRW);
	printk("Sending 0xE0 to TEGRA_EMC_MRW\n");
	emc_writel(0x900E0, TEGRA_EMC_MRW);
	printk("Sending 0x90 to TEGRA_EMC_MRW\n");
	emc_writel(0x90090, TEGRA_EMC_MRW);
	/* Sending the test op code per Hynix FAE requested. 
	   0xBD - reducing self-refresh rate from 30 us to 20 us.
	   */
	printk("Sending 0xBD to TEGRA_EMC_MRW\n");
	emc_writel(0x900BD, TEGRA_EMC_MRW);
}

#if defined (CONFIG_MACH_STAR)
extern void write_cmd_reserved_buffer(unsigned char *buf, size_t len);
#endif

typedef enum
{       
        RESET_COLD_BOOT,
        RESET_NORMAL_RESET,      
#if defined (CONFIG_STAR_HIDDEN_RESET)
        RESET_HIDDEN_RESET,      
#endif        
        RESET_PANIC_RESET,  
} RESET_MODE_TYPE;

int reset_status = RESET_COLD_BOOT;

static void reset_status_setup(char *str)
{
	switch (str[0])
	{
		case 'w':
			reset_status = RESET_NORMAL_RESET;
			printk("reset_status_setup = RESET_NORMAL_RESET\n");
			break;
#if defined (CONFIG_STAR_HIDDEN_RESET)			
		case 'h':	
			reset_status = RESET_HIDDEN_RESET;
			printk("reset_status_setup = RESET_HIDDEN_RESET\n");
			break;
#endif			
		case 'p':
			reset_status = RESET_PANIC_RESET;
			printk("reset_status_setup = RESET_PANIC_RESET\n");
			break;
		default :
			reset_status = RESET_COLD_BOOT;
			printk("reset_status_setup = RESET_COLD_BOOT\n");
			break;
	}


}
__setup("rs=", reset_status_setup);

int get_reset_status()
{
	return reset_status;
}


EXPORT_SYMBOL(get_reset_status);

static void __init tegra_setup_debug_uart(void)
{
	NvOdmDebugConsole uart = NvOdmQueryDebugConsole();
	const struct tegra_pingroup_config *pinmux = NULL;
	const NvU32 *odm_table;
	struct clk *c = NULL;
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

	platform_device_register(&debug_uart);

	uart_debug_port.port = uart;
	uart_debug_port.pinmux = pinmux;
	uart_debug_port.nr_pins = nr_pins;
	uart_debug_port.clk_data = c;
}

static void tegra_debug_port_suspend(void)
{
	if (uart_debug_port.port == NvOdmDebugConsole_None)
		return;
	clk_disable(uart_debug_port.clk_data);
	tegra_pinmux_config_tristate_table(uart_debug_port.pinmux,
				uart_debug_port.nr_pins, TEGRA_TRI_TRISTATE);
}

static void tegra_debug_port_resume(void)
{
	if (uart_debug_port.port == NvOdmDebugConsole_None)
		return;
	clk_enable(uart_debug_port.clk_data);
	tegra_pinmux_config_tristate_table(uart_debug_port.pinmux,
				uart_debug_port.nr_pins, TEGRA_TRI_NORMAL);
}

//20100830, gunwoo1.kim, soft reset [START]
#if defined(CONFIG_INPUT_KEYRESET)
static int star_reset_keys_up[] = { 0 };

static struct keyreset_platform_data star_reset_keys_pdata = {
    .keys_up = star_reset_keys_up,
    .keys_down = {
        KEY_POWER,
        KEY_VOLUMEUP,
        0
    },
};

struct platform_device star_reset_keys_device = {
    .name = KEYRESET_NAME,
    .dev.platform_data = &star_reset_keys_pdata,
};
#endif
//20100830, gunwoo1.kim, soft reset [END]

// 20101111 BT:  - For the BD Address Read /write [Start]
struct platform_device star_bd_address_device = {
	.name = "star_bd_address",
	.id = -1,
};
// 20101111 BT:  - For the BD Address Read /write [End]


#ifdef CONFIG_MMC_SDHCI_TEGRA
extern struct tegra_nand_platform tegra_nand_plat;
static struct tegra_sdhci_platform_data tegra_sdhci_platform[] = {
	[0] = {
		.bus_width = 4,
		.debounce = 5,
// 20100827  [WLAN] NVIDIA bug fix - lock up after suspend [START]
		.is_always_on = 1,
// 20100827  [WLAN] NVIDIA bug fix - lock up after suspend [END]
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
		if (!prop || prop->usage==NvOdmQuerySdioSlotUsage_unused)
			continue;

		plat = &tegra_sdhci_platform[i];
		gpio = NvOdmQueryGpioPinMap(NvOdmGpioPinGroup_Sdio,
			i, &gpio_count);

		plat->is_removable = prop->IsCardRemovable;
		plat->is_always_on = prop->AlwaysON;

#ifdef CONFIG_MACH_VENTANA
		if (prop->usage == NvOdmQuerySdioSlotUsage_wlan)
			plat->register_status_notify =
				ventana_wifi_status_register;
#endif

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

//, skip registering with useless uart port [START]
       if ( 0 == odm_table[i] )
			continue;
//, skip registering with useless uart port [END]
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
/* EHCI transfers must be 32B aligned */
static u64 tegra_ehci_dma_mask = DMA_BIT_MASK(32) & ~0x1f;
static struct platform_device tegra_hcd[] = {
	[0] = {
		.name = "tegra-ehci",
		.id = 0,
		.dev = {
			.platform_data = &tegra_hcd_platform[0],
			.coherent_dma_mask = DMA_BIT_MASK(32) & ~0x1f,
			.dma_mask = &tegra_ehci_dma_mask,
		},
		.resource = tegra_hcd_resources[0],
		.num_resources = ARRAY_SIZE(tegra_hcd_resources[0]),
	},
	[1] = {
		.name = "tegra-ehci",
		.id = 1,
		.dev = {
			.platform_data = &tegra_hcd_platform[1],
			.coherent_dma_mask = DMA_BIT_MASK(32) & ~0x1f,
			.dma_mask = &tegra_ehci_dma_mask,
		},
		.resource = tegra_hcd_resources[1],
		.num_resources = ARRAY_SIZE(tegra_hcd_resources[1]),
	},
	[2] = {
		.name = "tegra-ehci",
		.id = 2,
		.dev = {
			.platform_data = &tegra_hcd_platform[2],
			.coherent_dma_mask = DMA_BIT_MASK(32) & ~0x1f,
			.dma_mask = &tegra_ehci_dma_mask,
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

		if ((p->UsbMode == NvOdmUsbModeType_Device) ||
		    (p->UsbMode == NvOdmUsbModeType_None))
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

#ifdef CONFIG_KEYBOARD_TEGRA
struct tegra_kbc_plat tegra_kbc_platform;

static noinline void __init tegra_setup_kbc(void)
{
	struct tegra_kbc_plat *pdata = &tegra_kbc_platform;
	const NvOdmPeripheralConnectivity *conn;
	NvOdmPeripheralSearch srch_attr = NvOdmPeripheralSearch_IoModule;
	const struct NvOdmKeyVirtTableDetail **vkeys;
	NvU32 srch_val = NvOdmIoModule_Kbd;
	NvU32 temp;
	NvU64 guid;
	NvU32 i, j, k;
	NvU32 cols=0;
	NvU32 rows=0;
	NvU32 *wake_row;
	NvU32 *wake_col;
	NvU32 wake_num;
	NvU32 vnum;

	pdata->keymap = kzalloc(sizeof(*pdata->keymap)*KBC_MAX_KEY, GFP_KERNEL);
	if (!pdata->keymap) {
		pr_err("%s: out of memory for key mapping\n", __func__);
		return;
	}
	pdata->wake_cnt = 0;
	if (NvOdmKbcIsSelectKeysWkUpEnabled(&wake_row, &wake_col, &wake_num)) {
		BUG_ON(!wake_num || wake_num>=KBC_MAX_KEY);
		pdata->wake_cfg = kzalloc(sizeof(*pdata->wake_cfg)*wake_num,
			GFP_KERNEL);
		if (pdata->wake_cfg) {
			pdata->wake_cnt = (int)wake_num;
			for (i=0; i<wake_num; i++) {
				pdata->wake_cfg[i].row=wake_row[i];
				pdata->wake_cfg[i].col=wake_col[i];
			}
		} else
			pr_err("disabling wakeup key filtering due to "
				"out-of-memory error\n");
	}

	NvOdmKbcGetParameter(NvOdmKbcParameter_DebounceTime, 1, &temp);

	/* debounce time is reported from ODM in terms of clock ticks. */
	pdata->debounce_cnt = temp;

	/* Get the scanning timeout in terms of MilliSeconds.*/
	temp = 0;
	NvOdmKbcGetParameter(NvOdmKbcParameter_KeyScanTimeout, 1, &temp);
	/* If value is 0 then set it to 5 second as default */
	if (!temp)
		temp = 5000;
	/* Convert Milliseconds to clock count of 32Kz */
	pdata->scan_timeout_cnt = temp*32;

	/* repeat cycle is reported from ODM in milliseconds,
	 * but needs to be specified in 32KHz ticks */
	temp = 0;
	NvOdmKbcGetParameter(NvOdmKbcParameter_RepeatCycleTime, 1, &temp);
	pdata->repeat_cnt = temp * 32;

	temp = NvOdmPeripheralEnumerate(&srch_attr, &srch_val, 1, &guid, 1);
	if (!temp) {
		kfree(pdata->keymap);
		pr_err("%s: failed to find keyboard module\n", __func__);
		return;
	}
	conn = NvOdmPeripheralGetGuid(guid);
	if (!conn) {
		kfree(pdata->keymap);
		pr_err("%s: failed to find keyboard\n", __func__);
		return;
	}

	for (i=0; i<conn->NumAddress; i++) {
		NvU32 addr = conn->AddressList[i].Address;

		if (conn->AddressList[i].Interface!=NvOdmIoModule_Kbd) continue;

		if (conn->AddressList[i].Instance) {
			pdata->pin_cfg[addr].num = cols++;
			pdata->pin_cfg[addr].is_col = true;
		} else {
			pdata->pin_cfg[addr].num = rows++;
			pdata->pin_cfg[addr].is_row = true;
		}
	}

	for (i=0; i<KBC_MAX_KEY; i++)
		pdata->keymap[i] = -1;

	vnum = NvOdmKbcKeyMappingGetVirtualKeyMappingList(&vkeys);

	for (i=0; i<rows; i++) {
		for (j=0; j<cols; j++) {
			NvU32 sc = NvOdmKbcGetKeyCode(i, j, rows, cols);
			for (k=0; k<vnum; k++) {
				if (sc >= vkeys[k]->StartScanCode &&
				    sc <= vkeys[k]->EndScanCode) {
					sc -= vkeys[k]->StartScanCode;
					sc = vkeys[k]->pVirtualKeyTable[sc];
					if (!sc) continue;
					pdata->keymap[kbc_indexof(i,j)]=sc;
				}

                        }
		}
	}
}
#else
static void tegra_setup_kbc(void) { }
#endif

#if defined(CONFIG_KEYBOARD_GPIO)
struct gpio_keys_platform_data tegra_button_data;
static char *gpio_key_names = "gpio_keys";
static noinline void __init tegra_setup_gpio_key(void)
{
	struct gpio_keys_button *tegra_buttons = NULL;
	int ngpiokeys = 0;
	const NvOdmGpioPinInfo *gpio_key_info;
	int i;
	NvOdmGpioPinKeyInfo *gpio_pin_info = NULL;

	gpio_key_info = NvOdmQueryGpioPinMap(NvOdmGpioPinGroup_keypadMisc, 0,
						 &ngpiokeys);

	if (!ngpiokeys) {
		pr_info("No gpio is configured as buttons\n");
		return;
	}

	tegra_buttons = kzalloc(ngpiokeys * sizeof(struct gpio_keys_button),
				 GFP_KERNEL);
	if (!tegra_buttons) {
		pr_err("Memory allocation failed for tegra_buttons\n");
		return;
	}

	for (i = 0; i < ngpiokeys; ++i) {
		tegra_buttons[i].gpio =
			(int)(gpio_key_info[i].Port*8 + gpio_key_info[i].Pin);
		gpio_pin_info = gpio_key_info[i].GpioPinSpecificData;
		tegra_buttons[i].code = (int)gpio_pin_info->Code;
		tegra_buttons[i].desc = gpio_key_names;

		if (gpio_key_info[i].activeState == NvOdmGpioPinActiveState_Low)
			tegra_buttons[i].active_low = 1;
		else
			tegra_buttons[i].active_low = 0;
		tegra_buttons[i].type = EV_KEY;
		tegra_buttons[i].wakeup = (gpio_pin_info->Wakeup)? 1: 0;
		tegra_buttons[i].debounce_interval =
				 gpio_pin_info->DebounceTimeMs;
	}

	tegra_button_data.buttons = tegra_buttons;
	tegra_button_data.nbuttons = ngpiokeys;
	return;
}
#else
static void tegra_setup_gpio_key(void) { }
#endif

#ifdef CONFIG_RTC_DRV_TEGRA
static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};
#endif

#ifdef CONFIG_RTC_DRV_TEGRA_ODM
static struct platform_device tegra_rtc_odm_device = {
	.name = "tegra_rtc_odm",
	.id   = -1,
};
#endif

#ifdef CONFIG_TEGRA_NVEC
static struct platform_device tegra_nvec_device = {
	.name = "nvec",
	.id = -1,
};
#endif

//20100527, , For Star Battery Driver [START]
#if (defined(CONFIG_MACH_STAR) && defined(CONFIG_STAR_BATTERY_CHARGER))
#ifndef CONFIG_TEGRA_BATTERY_ODM
#error "You have to set defconfig(Device Drivers -> Power supply class support -> NVIDIA Tegra ODM kit battery driver)"
#else
static struct platform_device star_battery_charger_device =
{
    .name = "star_battery_charger",
    .id   = -1,
};
#endif // error
#elif defined(CONFIG_TEGRA_BATTERY_NVEC) || defined(CONFIG_TEGRA_BATTERY_ODM)
static struct platform_device tegra_battery_device = {
	.name = "tegra_battery",
	.id = -1,
};
#endif // CONFIG_MACH_STAR
//20100527, , For Star Battery Driver [END]

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
static struct regulator_consumer_supply tegra_soc_consumers[] = {
	[0] = {
		.supply = "soc_main",
	},
};
static struct regulator_consumer_supply tegra_vdd_bb_consumers[] = {
	[0] = {
		.supply   = "vddio bb",
	},
};
static struct regulator_consumer_supply tegra_vdd_lcd_consumers[] = {
	[0] = {
		.supply   = "vddio lcd",
	},
};
static struct regulator_consumer_supply tegra_vdd_vi_consumers[] = {
	[0] = {
		.supply   = "vddio vi",
	},
};
static struct regulator_consumer_supply tegra_vdd_uart_consumers[] = {
	[0] = {
		.supply   = "vddio uart",
	},
};
static struct regulator_consumer_supply tegra_vdd_ddr_consumers[] = {
	[0] = {
		.supply   = "vddio ddr",
	},
};
static struct regulator_consumer_supply tegra_vdd_nand_consumers[] = {
	[0] = {
		.supply   = "vddio nand",
	},
};
static struct regulator_consumer_supply tegra_vdd_sys_consumers[] = {
	[0] = {
		.supply   = "vddio sys",
	},
};
static struct regulator_consumer_supply tegra_vdd_audio_consumers[] = {
	[0] = {
		.supply   = "vddio audio",
	},
};
static struct regulator_consumer_supply tegra_vdd_sd_consumers[] = {
	[0] = {
		.supply   = "vddio sd",
	},
};

#ifdef CONFIG_TEGRA_USB_CHARGE
static struct regulator_consumer_supply tegra_vbus_consumers[] = {
	[0] = {
		.supply = "vbus_draw",
		.dev_name = "tegra-udc.0",
	},
};
#endif
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
	[2] = {
		.guid = NV_VDD_SoC_ODM_ID,
		.name = "soc_main",
		.id = 2,
		.consumers = tegra_soc_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_soc_consumers),
	},
	[3] = {
		.guid = NV_VDD_BB_ODM_ID,
		.name = "vddio bb",
		.id = 3,
		.consumers = tegra_vdd_bb_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_bb_consumers),
	},
	[4] = {
		.guid = NV_VDD_LCD_ODM_ID,
		.name = "vddio lcd",
		.id = 4,
		.consumers = tegra_vdd_lcd_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_lcd_consumers),
	},
	[5] = {
		.guid = NV_VDD_VI_ODM_ID,
		.name = "vddio vi",
		.id = 5,
		.consumers = tegra_vdd_vi_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_vi_consumers),
	},
	[6] = {
		.guid = NV_VDD_UART_ODM_ID,
		.name = "vddio uart",
		.id = 6,
		.consumers = tegra_vdd_uart_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_uart_consumers),
	},
	[7] = {
		.guid = NV_VDD_DDR_ODM_ID,
		.name = "vddio ddr",
		.id = 7,
		.consumers = tegra_vdd_ddr_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_ddr_consumers),
	},
	[8] = {
		.guid = NV_VDD_NAND_ODM_ID,
		.name = "vddio nand",
		.id = 8,
		.consumers = tegra_vdd_nand_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_nand_consumers),
	},
	[9] = {
		.guid = NV_VDD_SYS_ODM_ID,
		.name = "vddio sys",
		.id = 9,
		.consumers = tegra_vdd_sys_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_sys_consumers),
	},
	[10] = {
		.guid = NV_VDD_AUD_ODM_ID,
		.name = "vddio audio",
		.id = 10,
		.consumers = tegra_vdd_audio_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_audio_consumers),
	},
	[11] = {
		.guid = NV_VDD_SDIO_ODM_ID,
		.name = "vddio sd",
		.id = 11,
		.consumers = tegra_vdd_sd_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_sd_consumers),
	},
	[12] = {
		.guid = NV_ODM_GUID('l','b','e','e','9','q','m','b'),
		.name = "lbee9qmb_vdd",
		.id = 12,
		.consumers = lbee9qmb_consumers,
		.nr_consumers = ARRAY_SIZE(lbee9qmb_consumers),
	},
#ifdef CONFIG_TEGRA_USB_CHARGE
	[13] = {
		.charging_path = NvRmPmuChargingPath_UsbBus,
		.name = "vbus_draw",
		.id = 13,
		.consumers = tegra_vbus_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vbus_consumers),
		.is_charger = true,
	},
#endif
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

#ifdef BRCM_BT_WAKE
static struct platform_device lbee9qmb_btwake_device = {
	.name = "lbee9qmb-rfkill_btwake",
	.dev = {
		.platform_data = &lbee9qmb_platform,
	},
};
#endif

static noinline void __init tegra_setup_rfkill(void)
{
	const NvOdmPeripheralConnectivity *con;
	unsigned int i;
/* 20100815  for bluetooth on [LGE_START] */

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

#ifdef BRCM_BT_WAKE
			int btwake_gpio = con->AddressList[3].Instance * 8 +
				con->AddressList[3].Address;
			pr_err("BRCM_LPM: GOT BT wake gpio=%x",btwake_gpio);
			lbee9qmb_platform.gpio_btwake = btwake_gpio;
#endif
#ifdef BRCM_HOST_WAKE
			int hostwake_gpio = con->AddressList[2].Instance * 8 +
				con->AddressList[2].Address;
			pr_err("BRCM_LPM: GOT HOST wake gpio=%x",hostwake_gpio);
			lbee9qmb_platform.gpio_hostwake = hostwake_gpio;
#endif		
#ifdef BRCM_BT_WAKE
			if (platform_device_register(&lbee9qmb_btwake_device))
			pr_err("%s: registration failed\n", __func__);
#endif		
                return;
        }
	}
/* ORIGINAL
	lbee9qmb_platform.delay=5;
	lbee9qmb_platform.gpio_pwr=-1;
	if ((con = NvOdmPeripheralGetGuid(NV_ODM_GUID('l','b','e','e','9','q','m','b'))))
	{
		for (i=0; i<con->NumAddress; i++) {
			if (con->AddressList[i].Interface == NvOdmIoModule_Gpio
					&& con->AddressList[i].Purpose == BT_RESET ){
				int nr_gpio = con->AddressList[i].Instance * 8 +
					con->AddressList[i].Address;
				lbee9qmb_platform.gpio_reset = nr_gpio;
				if (platform_device_register(&lbee9qmb_device))
					pr_err("%s: registration failed\n", __func__);
				return;
			}
		}
	}
	else if ((con = NvOdmPeripheralGetGuid(NV_ODM_GUID('b','c','m','_','4','3','2','9'))))
	{
		int nr_gpio;
		for (i=0; i<con->NumAddress; i++) {
                        if (con->AddressList[i].Interface == NvOdmIoModule_Gpio
						&& con->AddressList[i].Purpose == BT_RESET){
					nr_gpio = con->AddressList[i].Instance * 8 +
						con->AddressList[i].Address;
					lbee9qmb_platform.gpio_reset = nr_gpio;
				}
			else if (con->AddressList[i].Interface == NvOdmIoModule_Gpio
						&& con->AddressList[i].Purpose == BT_SHUTDOWN ){
					nr_gpio = con->AddressList[i].Instance * 8 +
						 con->AddressList[i].Address;
					lbee9qmb_platform.gpio_pwr = nr_gpio;
				}
		}
		lbee9qmb_platform.delay=200;
                if (platform_device_register(&lbee9qmb_device))
			pr_err("%s: registration failed\n", __func__);
                return;
        }
        return;
ORIGINAL */
/* 20100815  for bluetooth on [LGE_END] */
}
#else
static void tegra_setup_rfkill(void) { }
#endif

#ifdef CONFIG_TOUCHSCREEN_TEGRA_ODM
static struct platform_device tegra_touch_device = {
	.name = "tegra_touch",
	.id = -1,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_ANDROID_VIRTUALKEYS
static ssize_t star_virtual_keys_show(struct kobject *kobj,
               struct kobj_attribute *attr, char *buf)
{
       /* Dimensions, 80x80, y starts at 830
       center: x: menu: 75, home: 185, back: 295, search 405, y: 910 */
       return sprintf(buf,
                       __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":75:910:110:110"
                       ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":185:910:110:110"
                       ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":295:910:110:110"
                       ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":405:910:110:110"
                       "\n");
}

static struct kobj_attribute star_virtual_keys_attr = {
       .attr = {
               .name = "virtualkeys.nvodm_touch",
               .mode = S_IRUGO,
       },
       .show = &star_virtual_keys_show,
};

static struct attribute *star_properties_attrs[] = {
       &star_virtual_keys_attr.attr,
       NULL
};

static struct attribute_group star_properties_attr_group = {
       .attrs = star_properties_attrs,
};

static int __init star_init_android_virtualkeys(void)
{
	struct kobject *properties_kobj;

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj) {
		if (sysfs_create_group(properties_kobj,
					&star_properties_attr_group))
			pr_err("failed to create board_properties\n");
	} else {
		pr_err("failed to create board_properties\n");
	}
}
#endif

#ifdef CONFIG_STAR_GYRO_ACCEL
static struct platform_device tegra_accelerometer_device =
{
	.name = "tegra_accelerometer",
	.id   = -1,
};

static struct platform_device tegra_gyro_accel_device =
{
	.name = "tegra_gyro_accel",
	.id   = -1,
};
#endif

//20100526 , For Compass Driver [start]
#ifdef CONFIG_STAR_COMPASS
static struct platform_device star_compass_device =
{
	.name = "tegra_compass",
	.id	  = -1,
};
#endif

// 20100917 jay.sim@lge.com, Temp for Sensor Modulazation --

#ifdef CONFIG_TOUCHSCREEN_PANJIT_I2C
static struct panjit_i2c_ts_platform_data panjit_data = {
	.gpio_reset = TEGRA_GPIO_PQ7,
};

static const struct i2c_board_info ventana_i2c_bus1_touch_info[] = {
	{
	 I2C_BOARD_INFO("panjit_touch", 0x3),
	 .irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV6),
	 .platform_data = &panjit_data,
	 },
};

static int __init ventana_touch_init_panjit(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PV6);	/* FIXME:  Ventana-specific GPIO assignment	*/
	tegra_gpio_enable(TEGRA_GPIO_PQ7);	/* FIXME:  Ventana-specific GPIO assignment	*/
	i2c_register_board_info(0, ventana_i2c_bus1_touch_info, 1);

	return 0;
}
#endif

#ifdef CONFIG_TOUCHSCREEN_ATMEL_MT_T9
/* Atmel MaxTouch touchscreen              Driver data */
/*-----------------------------------------------------*/
/*
 * Reads the CHANGELINE state; interrupt is valid if the changeline
 * is low.
 */
static u8 read_chg()
{
	return gpio_get_value(TEGRA_GPIO_PV6);
}

static u8 valid_interrupt()
{
	return !read_chg();
}

static struct mxt_platform_data Atmel_mxt_info = {
	/* Maximum number of simultaneous touches to report. */
	.numtouch = 10,
	// TODO: no need for any hw-specific things at init/exit?
	.init_platform_hw = NULL,
	.exit_platform_hw = NULL,
	.max_x = 1366,
	.max_y = 768,
	.valid_interrupt = &valid_interrupt,
	.read_chg = &read_chg,
};

static struct i2c_board_info __initdata i2c_info[] = {
	{
	 I2C_BOARD_INFO("maXTouch", MXT_I2C_ADDRESS),
	 .irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV6),
	 .platform_data = &Atmel_mxt_info,
	 },
};

static int __init ventana_touch_init_atmel(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PV6);	/* FIXME:  Ventana-specific GPIO assignment	*/
	tegra_gpio_enable(TEGRA_GPIO_PQ7);	/* FIXME:  Ventana-specific GPIO assignment	*/

	gpio_set_value(TEGRA_GPIO_PQ7, 0);
	msleep(1);
	gpio_set_value(TEGRA_GPIO_PQ7, 1);
	msleep(100);

	i2c_register_board_info(0, i2c_info, 1);

	return 0;
}
#endif

#ifdef CONFIG_INPUT_TEGRA_ODM_ACCEL
static struct platform_device tegra_accelerometer_device = {
	.name = "tegra_accelerometer",
	.id   = -1,
};

static struct platform_device tegra_gyro_accel_device =
{
	.name = "tegra_gyro_accel",
	.id   = -1,
};

static struct platform_device star_compass_device =
{
	.name = "tegra_compass",
	.id	  = -1,
};
#endif
// 20100917 , Temp for Sensor Modulazation --


//20100526 , For Vibrator Driver [start]
#ifdef CONFIG_STAR_VIBRATOR
static struct platform_device star_vib_device =
{
    .name = "star_vib_name",
    .id   = -1,
};
#endif
//20100526 , For Vibrator Driver [end]

#ifdef CONFIG_STAR_HALL //20100903 
static struct platform_device star_hall_device =
{
    .name = "star_hall",
    .id   = -1,
};
#endif

#ifdef CONFIG_INPUT_TEGRA_ODM_SCROLL
static struct platform_device tegra_scrollwheel_device = {
	.name = "tegra_scrollwheel",
	.id   = -1,
};
#endif

#ifdef CONFIG_TEGRA_ODM_VIBRATE
static struct platform_device tegra_vibrator_device = {
	.name = "tegra_vibrator",
	.id = -1,
};
#endif
//20100401  MUIC driver [START]
#if defined(CONFIG_MACH_STAR)
static struct platform_device star_muic_device =
{
    .name = "star_muic",
    .id   = -1,
};
#endif
// 20100401  MUIC driver [END]
// 20100526  Proximity driver [START]
#ifdef CONFIG_STAR_PROXIMITY
static struct platform_device star_proximity_device =
{
    .name = "star_proximity",
    .id   = -1,
};
#endif
// 20100526  Proximity driver [END]

//20100413, , star powerkey [START]
#ifdef CONFIG_MACH_STAR
#ifdef CONFIG_STAR_POWERKEY
static struct platform_device star_powerkey =
{
    .name = "star_powerkey",
    .id   = -1,
};
#endif


//20100611, , touch LED [START]
#ifdef CONFIG_STAR_TOUCH_LED
static struct platform_device star_touch_led =
{
    .name = "star_touch_led",
    .id   = -1,
};

#endif
//20100611, , touch LED [END]

//20100702, , HDMI regulator [START]
#ifdef CONFIG_STAR_HDMI_REG
static struct platform_device star_hdmi_reg =
{
    .name = "star_hdmi_reg",
    .id   = -1,
};
#endif
//20100702, , HDMI regulator [END]

#endif
//20100413, , star powerkey [END]


//20100419  headset detection [LGE_START]
#if defined(CONFIG_MACH_STAR)
static struct gpio_switch_platform_data star_headset_data = {
	.name = "h2w",
    .gpio = 170,	//20100419  GPIO Index, not used for nVidia gpio
};
static struct platform_device star_headset_device = {
	.name		= "star_headset",
	.id		= -1,
	.dev.platform_data = &star_headset_data,
};
#endif
//20100419  headset detection [LGE_END]

//LGE_UPDATE_S  2010-05-024 GPS UART & GPIO Setting
static struct platform_device tegra_gps_gpio =
{
    .name = "tegra_gps_gpio",
    .id   = -1,
};
//LGE_UPDATE_E  2010-05-024 GPS UART & GPIO Setting

//20100704  jongik's headset porting [LGE_START]
#if defined(CONFIG_MACH_STAR)
static struct platform_device star_wm8994_pdevice =
{
	.name = "star_wm8994",
	.id	  = -1,
};
#endif
//20100704  jongik's headset porting [LGE_END]
//20100701  crashdump [LGE_START]
#if defined(CONFIG_ANDROID_RAM_CONSOLE)

#if defined (CONFIG_STAR_HIDDEN_RESET)
#define RAM_CONSOLE_RESERVED_SIZE 2
#else
#define RAM_CONSOLE_RESERVED_SIZE 1
#endif
#define CARVEOUT_SIZE 128
#define STAR_RAM_CONSOLE_BASE 	((512-CARVEOUT_SIZE-RAM_CONSOLE_RESERVED_SIZE)*SZ_1M)
#ifdef CONFIG_MACH_STAR_TMUS
#define STAR_RAM_CONSOLE_SIZE	(128*SZ_1K) 	
#else
#define STAR_RAM_CONSOLE_SIZE	(512*SZ_1K) 	
#endif
static struct resource ram_console_resource[] = {
    {
        .name = "ram_console",
        .start = STAR_RAM_CONSOLE_BASE,
        .end = STAR_RAM_CONSOLE_BASE + STAR_RAM_CONSOLE_SIZE - 1,
        .flags  = IORESOURCE_MEM,
    }
};

static struct platform_device ram_console_device = {
        .name = "ram_console",
        .id = -1,
        .num_resources  = ARRAY_SIZE(ram_console_resource),
        .resource       = ram_console_resource,
};
#endif
//20100701  crashdump  [LGE_END]

static struct platform_device *nvodm_devices[] __initdata = {
#ifdef CONFIG_RTC_DRV_TEGRA
	&tegra_rtc_device,
#endif
#ifdef CONFIG_RTC_DRV_TEGRA_ODM
	&tegra_rtc_odm_device,
#endif
#ifdef CONFIG_TEGRA_NVEC
	&tegra_nvec_device,
#endif
//20100527, , For Star Battery Driver [START]
#if (defined(CONFIG_MACH_STAR) && defined(CONFIG_STAR_BATTERY_CHARGER))
	&star_battery_charger_device,
#elif defined(CONFIG_TEGRA_BATTERY_NVEC) || defined(CONFIG_TEGRA_BATTERY_ODM)
	&tegra_battery_device,
#endif // CONFIG_MACH_STAR
//20100527, , For Star Battery Driver [END]
#ifdef CONFIG_REGULATOR_TEGRA
	&tegra_regulator_device,
#endif
#ifdef CONFIG_TOUCHSCREEN_TEGRA_ODM
	&tegra_touch_device,
#endif
#ifdef CONFIG_INPUT_TEGRA_ODM_SCROLL
	//&tegra_scrollwheel_device,
#endif

#ifdef CONFIG_STAR_GYRO_ACCEL
	&tegra_accelerometer_device,
	&tegra_gyro_accel_device,
#endif

#ifdef CONFIG_STAR_COMPASS
	&star_compass_device,
#endif

/* Temp for sensor driver module */
#ifdef CONFIG_STAR_SENSORS
	&tegra_gyro_accel_device,
	&tegra_accelerometer_device,
	&star_compass_device,
#endif
/**/

#ifdef CONFIG_TEGRA_ODM_VIBRATE
//	&tegra_vibrator_device,
#endif

#ifdef CONFIG_STAR_VIBRATOR
	&star_vib_device,
#endif

#ifdef CONFIG_STAR_HALL
	&star_hall_device,
#endif

#if defined(CONFIG_STAR_MUIC) || defined(CONFIG_STAR_MUIC_TI)
    	&star_muic_device,
#endif
#ifdef CONFIG_STAR_POWERKEY
    &star_powerkey,
#endif

#ifdef CONFIG_STAR_PROXIMITY
    	&star_proximity_device,
#endif

#ifdef CONFIG_STAR_TOUCH_LED
    &star_touch_led,
#endif

#ifdef CONFIG_STAR_HDMI_REG
    &star_hdmi_reg,
#endif

//20100419  for headset detection [LGE_START]
#if defined(CONFIG_MACH_STAR)
    &star_wm8994_pdevice,                       // 20110621
#endif /* CONFIG_MACH_STAR */
//20100419  for headset detection [LGE_END]

//LGE_UPDATE_S  2010-05-024 GPS UART & GPIO Setting
    &tegra_gps_gpio,
//LGE_UPDATE_E  2010-05-024 GPS UART & GPIO Setting

//20100704  jongik's wm8994 driver porting [LGE_START]
#if defined(CONFIG_MACH_STAR)
    &star_headset_device,                       // 20110621
#endif /* CONFIG_MACH_STAR */
//20100704  jongik's wm8994 driver porting [LGE_END]
#if defined(CONFIG_ANDROID_RAM_CONSOLE)
    &ram_console_device,
#endif /* CONFIG_MACH_STAR */
//20100830, , soft reset [START]
#if defined(CONFIG_INPUT_KEYRESET)
    &star_reset_keys_device,
#endif
//20100830, , soft reset [END]

// 20101115 BT:  - For the BD Address Read /write [Start]
	&star_bd_address_device,
// 20101115 BT:  - For the BD Address Read /write [End]

};

//20100711-1, , add spi_ifxn721 [START]
#if defined(CONFIG_SPI_TEGRA) && !defined(CONFIG_SPI_MDM6600)
#include <linux/spi/spi.h>

static struct spi_board_info tegra_spi_board_info[] __initdata = {
    {
        .modalias = "spi_ifxn721",
        .bus_num = 1,
        .chip_select = 0,
        .mode = SPI_MODE_1,
        .max_speed_hz = 24000000,
//        .platform_data = NULL,//검토
        .irq = 0,
    },
};
static void __init tegra_register_ifxn721(void)
{
    NvError err;
    NvRmGpioPinHandle hPin;
    NvU32 irq;
    NvU32 instance = 0xFFFF;
    NvU32 cs = 0xFFF;
	NvU32 pin = 0xFFFF, port = 0xFFFF;	//SPI_SRDY
	NvU32 pin2 = 0xFFFF, port2 = 0xFFFF; //SPI_MRDY	//20100607, , Add spi_mrdy
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    int i;
    const NvOdmQuerySpiDeviceInfo *pSpiDeviceInfo;

    pConnectivity =
        NvOdmPeripheralGetGuid(NV_ODM_GUID('s','t','a','r','-','s','p','i'));	//20100607, , modify ifxn-721 -> star-spi
	
    if (!pConnectivity){
	 printk("[tegra_spi]pConnectivity = %d \n", (int)pConnectivity);
        return;
    }

    for (i = 0; i < pConnectivity->NumAddress; i++)
    {
        switch (pConnectivity->AddressList[i].Interface)
        {
            case NvOdmIoModule_Spi:
                instance = pConnectivity->AddressList[i].Instance;
                cs = pConnectivity->AddressList[i].Address;
                break;
            case NvOdmIoModule_Gpio:
		if(pin==0xFFFF && port==0xFFFF)
		{
			   port = pConnectivity->AddressList[i].Instance;
			   pin = pConnectivity->AddressList[i].Address;
		}
		else //20100607, , add spi_mrdy
		{
			   port2 = pConnectivity->AddressList[i].Instance;
			   pin2 = pConnectivity->AddressList[i].Address;
		}
                break;
            default:
                break;
        }
    }

    /* SPI ethernet driver needs one SPI info and a gpio for interrupt */
    if (instance == 0xffff || cs == 0xffff || port == 0xFFFF || pin == 0xFFFF
		|| port2 == 0xFFFF || pin2 == 0xFFFF){	//20100607, , add spi_mrdy
	 printk("[tegra_spi]instance = %d, cs = %d, srdy[%d-%d], mrdy[%d-%d]\n", instance, cs, port, pin, port2, pin2);
        return;
    }

    /* Check if the SPI is configured as a master for this instance
     * If it it not, don't register the device.
     * */
    pSpiDeviceInfo = NvOdmQuerySpiGetDeviceInfo(NvOdmIoModule_Spi, instance, cs);
    if (pSpiDeviceInfo && pSpiDeviceInfo->IsSlave)
        return;

#ifndef CONFIG_MACH_STAR_TMUS
 //Set SRDY pin as interrupt 	
    err = NvRmGpioAcquirePinHandle(s_hGpioGlobal, port, pin, &hPin);
    if (err)
    {
        return;
    }
    NvRmGpioConfigPins(s_hGpioGlobal, &hPin, 1,
        NvRmGpioPinMode_InputInterruptFallingEdge);
    NvRmGpioGetIrqs(s_hRmGlobal, &hPin, &irq, 1);

    printk("[tegra_spi]Register ifxn721 SPI driver\n");

    tegra_spi_board_info[0].irq = irq; //SRDY
#endif
    /* FIXME, instance need not be same as bus number. */
    tegra_spi_board_info[0].bus_num = instance;
    tegra_spi_board_info[0].chip_select = cs;
    spi_register_board_info(tegra_spi_board_info, ARRAY_SIZE(tegra_spi_board_info));
}
#endif /*CONFIG_SPI_TEGRA*/
//20100711, , add spi_ifxn721 [END]

#if defined(CONFIG_SPI_TEGRA) && !defined(CONFIG_SPI_SLAVE_TEGRA)
static struct tegra_spi_platform_data tegra_spi_platform[] = {
	[0] = {
//20100711-1, , add pinmux [START]		
		.pinmux = NvOdmSpiPinMap_Config1,
//20100711, , add pinmux [END]
		.is_slink = true,
	},
#if 0	
	[1] = {
		.is_slink = true,
	},
	[2] = {
		.is_slink = true,
	},
	[3] = {
		.is_slink = true,
	},
	[4] = {
		.is_slink = false,
	},
#endif	
};
static struct platform_device tegra_spi_devices[] = {
	[0] = {
		.name = "tegra_spi",
		.id = 0,
		.dev = {
			.platform_data = &tegra_spi_platform[0],
		},
	},
#if 0
	[1] = {
		.name = "tegra_spi",
		.id = 1,
		.dev = {
			.platform_data = &tegra_spi_platform[1],
		},
	},
	[2] = {
		.name = "tegra_spi",
		.id = 2,
		.dev = {
			.platform_data = &tegra_spi_platform[2],
		},
	},
	[3] = {
		.name = "tegra_spi",
		.id = 3,
		.dev = {
			.platform_data = &tegra_spi_platform[3],
		},
	},
	[4] = {
		.name = "tegra_spi",
		.id = 4,
		.dev = {
			.platform_data = &tegra_spi_platform[4],
		},
	},
#endif	
};
static noinline void __init tegra_setup_spi(void)
{
	const NvU32 *spi_mux;
	const NvU32 *sflash_mux;
	NvU32 spi_mux_nr;
	NvU32 sflash_mux_nr;
	int i;

	NvOdmQueryPinMux(NvOdmIoModule_Spi, &spi_mux, &spi_mux_nr);
	NvOdmQueryPinMux(NvOdmIoModule_Sflash, &sflash_mux, &sflash_mux_nr);

	for (i=0; i<ARRAY_SIZE(tegra_spi_devices); i++) {
		struct platform_device *pdev = &tegra_spi_devices[i];
		struct tegra_spi_platform_data *plat = &tegra_spi_platform[i];

		const NvOdmQuerySpiDeviceInfo *info = NULL;
		NvU32 mux = 0;
		int rc;

		if (plat->is_slink && pdev->id<spi_mux_nr)
			mux = spi_mux[pdev->id];
		else if (sflash_mux_nr && !plat->is_slink)
			mux = sflash_mux[0];

		if (!mux)
			continue;

		if (mux == NVODM_QUERY_PINMAP_MULTIPLEXED) {
			pr_err("%s: not registering multiplexed SPI master "
			       "%s.%d\n", __func__, pdev->name, pdev->id);
			WARN_ON(1);
			continue;
		}

		if (plat->is_slink) {
			info = NvOdmQuerySpiGetDeviceInfo(NvOdmIoModule_Spi,
							  pdev->id, 0);
		} else {
			info = NvOdmQuerySpiGetDeviceInfo(NvOdmIoModule_Sflash,
							  0, 0);
		}

		if (info && info->IsSlave) {
			pr_info("%s: not registering SPI slave %s.%d\n",
				__func__, pdev->name, pdev->id);
			continue;
		}

		rc = platform_device_register(pdev);
		if (rc) {
			pr_err("%s: registration of %s.%d failed\n",
			       __func__, pdev->name, pdev->id);
		}
	}

//20100711-1, , add spi_ifxn721 [START]
	tegra_register_ifxn721();
//20100711, , add spi_ifxn721 [END]
}
#else
static void tegra_setup_spi(void) { }
#endif

//20100613-1, , add spi slave [START]
#if defined(CONFIG_SPI_MDM6600)
#include <linux/spi/spi.h>

static struct spi_board_info tegra_spi_board_info[] __initdata = {
    {
        .modalias = "mdm6600",
        .bus_num = 1,
        .chip_select = 0,
        .mode = SPI_MODE_1,
        .max_speed_hz = 24000000,
//        .platform_data = NULL,//검토 
        .irq = 0,
    },
//20100809-1, , Add SPI2 for AP-CP IPC [START]
#ifdef CONFIG_DUAL_SPI
	{
		.modalias = "mdm6600",
		.bus_num = 2,
		.chip_select = 0,
		.mode = SPI_MODE_1,
		.max_speed_hz = 24000000,
//		  .platform_data = NULL,//검토 
		.irq = 0,
	},
#endif
//20100809, , Add SPI2 for AP-CP IPC [END]
};

static void __init register_mdm6600(void)
{
    NvError err;
    NvRmGpioPinHandle hPin;
    NvU32 irq;
    NvU32 instance = 0xFFFF;
    NvU32 cs = 0xFFF;
	NvU32 pin[4] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};
	NvU32 port[4] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};	//SPI_SRDY = pin[0], port[0] SPI_MRDY = pin[1],port[1]
	//NvU32 pin2 = 0xFFFF, port2 = 0xFFFF; //SPI_MRDY
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    int i =0;
    const NvOdmQuerySpiDeviceInfo *pSpiDeviceInfo;
	int id, pos = 0;

    pConnectivity = NvOdmPeripheralGetGuid(NV_ODM_GUID('s','t','a','r','-','s','p','i'));
    if (!pConnectivity)
    {
	printk("register_mdm6600 : pConnectivity is null!r\n");
        return;
    }
	
	for(id=0 ; id < ARRAY_SIZE(tegra_spi_board_info) ;id++)
	{
		printk("%s#%d : i = %d\n", __FUNCTION__, id, i);
		pos = 0;
	    for ( ; i < (id+1)*(pConnectivity->NumAddress/ARRAY_SIZE(tegra_spi_board_info)) ; i++)
    {
        switch (pConnectivity->AddressList[i].Interface)
        {
            case NvOdmIoModule_Spi:
                instance = pConnectivity->AddressList[i].Instance;
                cs = pConnectivity->AddressList[i].Address;
                break;
            case NvOdmIoModule_Gpio:
			   port[pos] = pConnectivity->AddressList[i].Instance;
			   pin[pos] = pConnectivity->AddressList[i].Address;
			   pos++;
                break;
            default:
                break;
        }
    }

    /* SPI ethernet driver needs one SPI info and a gpio for interrupt */
	    if (instance == 0xffff || cs == 0xffff || port[0] == 0xFFFF || pin[0] == 0xFFFF)
    {
		 printk("%s#%d : instance = %d, cs = %d, port2 = %d, pin2 = %dr\n", __FUNCTION__, id, instance, cs, port[0], pin[0]);
        return;
    }
	/* Check if the SPI is configured as a master for this instance
	* If it it not, don't register the device.
	* */
	pSpiDeviceInfo = NvOdmQuerySpiGetDeviceInfo(NvOdmIoModule_Spi, instance, cs);
		printk("%s#%d : instance = %d, cs = %d\n",__FUNCTION__, id, instance, cs);
//20100613-1, , add spi slave mode [START]
	if (pSpiDeviceInfo && pSpiDeviceInfo->IsSlave)
	{	//slave mode ; mrdy pin works as interrupt
#ifndef CONFIG_MACH_STAR_TMUS
			err = NvRmGpioAcquirePinHandle(s_hGpioGlobal, port[0], pin[0], &hPin);
		if (err)
		{
				printk("%s#%d : can't open pin handle\n", __FUNCTION__, id);
		    	return;
		}
		NvRmGpioConfigPins(s_hGpioGlobal, &hPin, 1, NvRmGpioPinMode_InputInterruptFallingEdge);
		NvRmGpioGetIrqs(s_hRmGlobal, &hPin, &irq, 1);
			tegra_spi_board_info[id].irq = irq; //SRDY
#endif
		/* FIXME, instance need not be same as bus number. */
			tegra_spi_board_info[id].bus_num = instance;
			tegra_spi_board_info[id].chip_select = cs;
			printk("%s#%d : irq = %d, instance = %d, cs = %d\n", __FUNCTION__, id, irq, instance, cs);
	}
	else
	{
			printk("%s#%d : spi is master\n", __FUNCTION__, id);
		return;
	}	
	}
	
	printk("Enabled SPI slave driver\n");
	spi_register_board_info(tegra_spi_board_info, ARRAY_SIZE(tegra_spi_board_info));
}
#endif /*CONFIG_SPI_MDM6600*/
//20100613-1, , add spi slave [END]


//20100613-1, , add spi slave [START]
#if !defined(CONFIG_SPI_SLAVE_TEGRA) 
#define tegra_setup_spi_slave() do {} while (0)
#else
static struct tegra_spi_platform_data tegra_spi_platform[] = {
	[0] = {
		.is_slink = true,
	},
	[1] = {
		.is_slink = true,
	},
	[2] = {
		.is_slink = true,
	},
	[3] = {
		.is_slink = true,
	},
	[4] = {
		.is_slink = false,
	},
};

static struct platform_device tegra_spi_devices[] = {
	[0] = {
		.name = "tegra_spi",
		.id = 0,
		.dev = {
			.platform_data = &tegra_spi_platform[0],
		},
	},
	[1] = {
		.name = "tegra_spi",
		.id = 1,
		.dev = {
			.platform_data = &tegra_spi_platform[1],
		},
	},
	[2] = {
		.name = "tegra_spi",
		.id = 2,
		.dev = {
			.platform_data = &tegra_spi_platform[2],
		},
	},
	[3] = {
		.name = "tegra_spi",
		.id = 3,
		.dev = {
			.platform_data = &tegra_spi_platform[3],
		},
	},
	[4] = {
		.name = "tegra_spi",
		.id = 4,
		.dev = {
			.platform_data = &tegra_spi_platform[4],
		},
	},
};

static noinline void __init tegra_setup_spi_slave(void)
{
	const NvU32 *spi_mux;
	const NvU32 *sflash_mux;
	NvU32 spi_mux_nr;
	NvU32 sflash_mux_nr;
	int i;

	NvOdmQueryPinMux(NvOdmIoModule_Spi, &spi_mux, &spi_mux_nr);
	NvOdmQueryPinMux(NvOdmIoModule_Sflash, &sflash_mux, &sflash_mux_nr);

	for (i=0; i<ARRAY_SIZE(tegra_spi_devices); i++) {
		struct platform_device *pdev = &tegra_spi_devices[i];
		struct tegra_spi_platform_data *plat = &tegra_spi_platform[i];

		const NvOdmQuerySpiDeviceInfo *info = NULL;
		NvU32 mux = 0;
		int rc;

		if (plat->is_slink && pdev->id<spi_mux_nr)
			mux = spi_mux[pdev->id];
		else if (sflash_mux_nr && !plat->is_slink)
			mux = sflash_mux[0];

		if (!mux)
			continue;

		if (mux == NVODM_QUERY_PINMAP_MULTIPLEXED) {
			pr_err("%s: not registering multiplexed SPI master "
			       "%s.%d\n", __func__, pdev->name, pdev->id);
			WARN_ON(1);
			continue;
		}

		if (plat->is_slink) {
			info = NvOdmQuerySpiGetDeviceInfo(NvOdmIoModule_Spi, pdev->id, 0);
		} else {
			info = NvOdmQuerySpiGetDeviceInfo(NvOdmIoModule_Sflash, 0, 0);
		}

		if (info && info->IsSlave!=NV_TRUE) {
			pr_info("%s: not registering SPI master %s.%d\n",
				__func__, pdev->name, pdev->id);
			continue;
		}

		rc = platform_device_register(pdev);
		if (rc) {
			pr_err("%s: registration of %s.%d failed\n",
			       __func__, pdev->name, pdev->id);
		}
	}

//20100711-1, , add spi_ifxn721 [START]
	register_mdm6600();
//20100711, , add spi_ifxn721 [END]
}
#endif	//CONFIG_SPI_SLAVE_TEGRA
//20100613, , add spi slave [END]

#ifdef CONFIG_I2C_TEGRA
#ifdef CONFIG_TEGRA_ODM_VENTANA
static struct tegra_i2c_plat_parms tegra_i2c_platform[] = {
	[0] = {
		.adapter_nr = 0,
		.bus_count = 1,
		.bus_mux = { 0, 0 },
		.bus_clk = { 100000, 0 }, /* default to 100KHz */
		.is_dvc = false,
	},
	[1] = {
		.adapter_nr = 1,
		.bus_count = 2,
		.bus_mux = { NvOdmI2cPinMap_Config1, NvOdmI2cPinMap_Config2},
		.bus_clk = { 100000, 100000 },
		.is_dvc = false,
	},
	[2] = {
		.adapter_nr = 3,
		.bus_count = 1,
		.bus_mux = { 0, 0 },
		.bus_clk = { 100000, 0 },
		.is_dvc = false,
	},
	[3] = {
		.adapter_nr = 4,
		.bus_count = 1,
		.bus_mux = { 0, 0 },
		.bus_clk = { 100000, 0 },
		.is_dvc = true,
	},
};
#else
static struct tegra_i2c_plat_parms tegra_i2c_platform[] = {
	[0] = {
		.adapter_nr = 0,
		.bus_count = 1,
		.bus_mux = { 0, 0 },
		.bus_clk = { 100000, 0 }, /* default to 100KHz */
		.is_dvc = false,
	},
	[1] = {
		.adapter_nr = 1,
		.bus_count = 1,
		.bus_mux = { 0, 0 },
		.bus_clk = { 100000, 0 },
		.is_dvc = false,
	},
	[2] = {
		.adapter_nr = 2,
		.bus_count = 1,
		.bus_mux = { 0, 0 },
		.bus_clk = { 100000, 0 },
		.is_dvc = false,
	},
	[3] = {
		.adapter_nr = 3,
		.bus_count = 1,
		.bus_mux = { 0, 0 },
		.bus_clk = { 100000, 0 },
		.is_dvc = true,
	},
};
#endif
static struct platform_device tegra_i2c_devices[] = {
	[0] = {
		.name = "tegra_i2c",
		.id = 0,
		.dev = {
			.platform_data = &tegra_i2c_platform[0],
		},
	},
	[1] = {
		.name = "tegra_i2c",
		.id = 1,
		.dev = {
			.platform_data = &tegra_i2c_platform[1],
		},
	},
	[2] = {
		.name = "tegra_i2c",
		.id = 2,
		.dev = {
			.platform_data = &tegra_i2c_platform[2],
		},
	},
	[3] = {
		.name = "tegra_i2c",
		.id = 3,
		.dev = {
			.platform_data = &tegra_i2c_platform[3],
		},
	},
};


static NvBool sensor_poweron()
{

	NvU32 mvolt_ldo7, mvolt_ldo8;
	NvU32 settletime;
	int retry = 0;
	printk("sensor poweron\n");
	NvOdmServicesPmuHandle h_pmu = NvOdmServicesPmuOpen();
	if (!h_pmu)
	{ 
		printk("sensor pmu handle fail!\n");
		return NV_FALSE;
	}

	do {

		NvOdmServicesPmuSetVoltage( h_pmu, Max8907PmuSupply_LDO7 , NVODM_VOLTAGE_OFF, &settletime);
		NvOdmOsWaitUS(settletime);

		NvOdmServicesPmuSetVoltage( h_pmu, Max8907PmuSupply_LDO8 , NVODM_VOLTAGE_OFF, &settletime);
		NvOdmOsWaitUS(settletime);

		NvOdmServicesPmuGetVoltage(h_pmu, Max8907PmuSupply_LDO7, &mvolt_ldo7);
		NvOdmServicesPmuGetVoltage(h_pmu, Max8907PmuSupply_LDO7, &mvolt_ldo8);

		printk("LDO7 = %dmV, LDO8 = %dmV\n", mvolt_ldo7, mvolt_ldo8);
		retry++;
		if (retry > 10) 
		{ 
			printk("sensor power control off fail!\n");
	                NvOdmServicesPmuClose(h_pmu);
			return NV_FALSE;
		}


	} while ( mvolt_ldo7 !=0 || mvolt_ldo8  != 0 );

	msleep(3000);
	
        do {

		NvOdmServicesPmuSetVoltage( h_pmu, Max8907PmuSupply_LDO7 , 3000, &settletime);
		NvOdmOsWaitUS(settletime);

		NvOdmServicesPmuSetVoltage( h_pmu, Max8907PmuSupply_LDO8 , 1800, &settletime);
		NvOdmOsWaitUS(settletime);

		NvOdmServicesPmuGetVoltage(h_pmu, Max8907PmuSupply_LDO7, &mvolt_ldo7);
		NvOdmServicesPmuGetVoltage(h_pmu, Max8907PmuSupply_LDO8, &mvolt_ldo8);

		printk("LDO7 = %dmV, LDO8 = %dmV\n", mvolt_ldo7, mvolt_ldo8);
		retry++;
		if (retry > 10) 
		{ 
			printk("sensor power control on fail!\n");
	                NvOdmServicesPmuClose(h_pmu);
			return NV_FALSE;
		}


	} while ( mvolt_ldo7 ==0 || mvolt_ldo8  == 0 );


	NvOdmOsWaitUS(10);

	NvOdmServicesPmuClose(h_pmu);
	return NV_TRUE;
}


static void sensor_workaround()
{

#define GEN2_I2C_SCL 't' - 'a', 5
#define GEN2_I2C_SDA 't' - 'a', 6
#define HALL_INT 'u' - 'a', 5
#define COM_INT 'r' -'a', 4
#define GYRO_INT_N 'q' - 'a' , 5
#define MOTION_INT 'i' -'a', 0
#define PROXI_OUT 'w' - 'a', 2
       
	NvOdmServicesGpioHandle hGpio;
	NvOdmGpioPinHandle hSclGpioPinHandle;
	NvOdmGpioPinHandle hSdaGpioPinHandle;
	NvOdmGpioPinHandle hHalGpioPinHandle;
	NvOdmGpioPinHandle hComGpioPinHandle;
	NvOdmGpioPinHandle hGyrGpioPinHandle;
	NvOdmGpioPinHandle hMotGpioPinHandle;
	NvOdmGpioPinHandle hProGpioPinHandle;

	hGpio = NvOdmGpioOpen();

	if (!hGpio)
	{
		printk("%s: NvOdmGpioOpen Error \n", __func__);
		goto error_open_gpio_fail;
	}

	hSclGpioPinHandle = NvOdmGpioAcquirePinHandle(hGpio, GEN2_I2C_SCL);
	hSdaGpioPinHandle = NvOdmGpioAcquirePinHandle(hGpio, GEN2_I2C_SDA);  
	hHalGpioPinHandle = NvOdmGpioAcquirePinHandle(hGpio, HALL_INT);  
	hComGpioPinHandle = NvOdmGpioAcquirePinHandle(hGpio, COM_INT);
	hGyrGpioPinHandle = NvOdmGpioAcquirePinHandle(hGpio, GYRO_INT_N);
	hMotGpioPinHandle = NvOdmGpioAcquirePinHandle(hGpio, MOTION_INT);
	hProGpioPinHandle = NvOdmGpioAcquirePinHandle(hGpio, PROXI_OUT);

         if (!hSclGpioPinHandle ||
	     !hSdaGpioPinHandle ||
	     !hHalGpioPinHandle ||
	     !hComGpioPinHandle ||
	     !hGyrGpioPinHandle ||
	     !hMotGpioPinHandle ||
	     !hProGpioPinHandle ) 
         {
                 printk("%s: Couldn't NvOdmGpioAcquirePinHandle  pin \n", __func__);
                 goto error_open_gpio_pin_acquire_fail;
         }  

	NvOdmGpioSetState(hGpio,hSclGpioPinHandle , 0x0); NvOdmGpioConfig(hGpio, hSclGpioPinHandle, 5 );
	NvOdmGpioSetState(hGpio,hSdaGpioPinHandle , 0x0); NvOdmGpioConfig(hGpio, hSdaGpioPinHandle, 5 );
	NvOdmGpioSetState(hGpio,hHalGpioPinHandle , 0x0); NvOdmGpioConfig(hGpio, hHalGpioPinHandle, 5 );
	NvOdmGpioSetState(hGpio,hComGpioPinHandle , 0x0); NvOdmGpioConfig(hGpio, hComGpioPinHandle, 5 );
	NvOdmGpioSetState(hGpio,hGyrGpioPinHandle , 0x0); NvOdmGpioConfig(hGpio, hGyrGpioPinHandle, 5 );
	NvOdmGpioSetState(hGpio,hMotGpioPinHandle , 0x0); NvOdmGpioConfig(hGpio, hMotGpioPinHandle, 5 );
	NvOdmGpioSetState(hGpio,hProGpioPinHandle , 0x0); NvOdmGpioConfig(hGpio, hProGpioPinHandle, 5 );

       if (!sensor_poweron())
       {
          printk("%s: sensor_poweron() fail \n", __func__);
       }

done:
 
error_open_gpio_pin_acquire_fail:
	NvOdmGpioReleasePinHandle(hGpio,hSclGpioPinHandle);
	NvOdmGpioReleasePinHandle(hGpio,hSdaGpioPinHandle);
	NvOdmGpioReleasePinHandle(hGpio,hHalGpioPinHandle); 
	NvOdmGpioReleasePinHandle(hGpio,hComGpioPinHandle);
	NvOdmGpioReleasePinHandle(hGpio,hGyrGpioPinHandle);
	NvOdmGpioReleasePinHandle(hGpio,hMotGpioPinHandle);
	NvOdmGpioReleasePinHandle(hGpio,hProGpioPinHandle);
error_open_gpio_fail:
        NvOdmGpioClose(hGpio);
}

static noinline void __init tegra_setup_i2c(void)
{
	const NvOdmPeripheralConnectivity *smbus;
	const NvOdmIoAddress *smbus_addr = NULL;
	const NvU32 *odm_mux_i2c = NULL;
	const NvU32 *odm_clk_i2c = NULL;
	const NvU32 *odm_mux_i2cp = NULL;
	const NvU32 *odm_clk_i2cp = NULL;
	NvU32 odm_mux_i2c_nr;
	NvU32 odm_clk_i2c_nr;
	NvU32 odm_mux_i2cp_nr;
	NvU32 odm_clk_i2cp_nr;
	int i;

        sensor_workaround();

	smbus = NvOdmPeripheralGetGuid(NV_ODM_GUID('I','2','c','S','m','B','u','s'));

	if (smbus) {
		unsigned int j;
		smbus_addr = smbus->AddressList;
		for (j=0; j<smbus->NumAddress; j++, smbus_addr++) {
			if ((smbus_addr->Interface == NvOdmIoModule_I2c) ||
			    (smbus_addr->Interface == NvOdmIoModule_I2c_Pmu))
				break;
		}
		if (j==smbus->NumAddress)
			smbus_addr = NULL;
	}

	NvOdmQueryPinMux(NvOdmIoModule_I2c, &odm_mux_i2c, &odm_mux_i2c_nr);
	NvOdmQueryPinMux(NvOdmIoModule_I2c_Pmu, &odm_mux_i2cp, &odm_mux_i2cp_nr);
	NvOdmQueryClockLimits(NvOdmIoModule_I2c, &odm_clk_i2c, &odm_clk_i2c_nr);
	NvOdmQueryClockLimits(NvOdmIoModule_I2c_Pmu, &odm_clk_i2cp, &odm_clk_i2cp_nr);

	for (i=0; i<ARRAY_SIZE(tegra_i2c_devices); i++) {

		struct platform_device *dev = &tegra_i2c_devices[i];
		struct tegra_i2c_plat_parms *plat = &tegra_i2c_platform[i];
		NvU32 mux, clk;

		if (smbus_addr) {
			if (smbus_addr->Interface == NvOdmIoModule_I2c &&
			    smbus_addr->Instance == dev->id && !plat->is_dvc) {
				pr_info("%s: skipping %s.%d (SMBUS)\n",
					__func__, dev->name, dev->id);
				continue;
			}
		}

		if (plat->is_dvc) {
			mux = (odm_mux_i2cp_nr) ? odm_mux_i2cp[0] : 0;
			clk = (odm_clk_i2cp_nr) ? odm_clk_i2cp[0] : 100;
		} else if (dev->id < odm_mux_i2c_nr) {
			mux = odm_mux_i2c[dev->id];
			clk = (dev->id < odm_clk_i2c_nr) ? odm_clk_i2c[dev->id] : 100;
		} else {
			mux = 0;
			clk = 0;
		}

		if (!mux)
			continue;

#ifndef CONFIG_TEGRA_ODM_VENTANA
		if (mux == NVODM_QUERY_PINMAP_MULTIPLEXED) {
			pr_err("%s: unable to register %s.%d (multiplexed)\n",
			       __func__, dev->name, dev->id);
			//WARN_ON(1);
			continue;
		}
#endif

		if (clk)
			plat->bus_clk[0] = clk*1000;

		if (platform_device_register(dev))
			pr_err("%s: failed to register %s.%d\n",
			       __func__, dev->name, dev->id);
	}
}
#else
static void tegra_setup_i2c(void) { }
#endif

#ifdef CONFIG_W1_MASTER_TEGRA
static struct tegra_w1_platform_data tegra_w1_platform;
static struct platform_device tegra_w1_device = {
	.name = "tegra_w1",
	.id = 0,
	.dev = {
		.platform_data = &tegra_w1_platform,
	},
};
static noinline void __init tegra_setup_w1(void)
{
	const NvU32 *pinmux;
	NvU32 nr_pinmux;

	NvOdmQueryPinMux(NvOdmIoModule_OneWire, &pinmux, &nr_pinmux);
	if (!nr_pinmux || !pinmux[0]) {
		pr_info("%s: no one-wire device\n", __func__);
		return;
	}
	tegra_w1_platform.pinmux = pinmux[0];
	if (platform_device_register(&tegra_w1_device)) {
		pr_err("%s: failed to register %s.%d\n",
		       __func__, tegra_w1_device.name, tegra_w1_device.id);
	}
}
#else
static void tegra_setup_w1(void) { }
#endif

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

//20100724  for poweroff leakage [LGE_START]
#define MAX_COUNT 10
void tegra_voltage_off( NvU32 vddId )
{
	int count=0;
	u32 settling_time;
    NvU32 millivolts=1;

	NvRmPmuGetVoltage(s_hRmGlobal, vddId, &millivolts);
	pr_info("[POWER] %s: LOD=%d : millivolts=%d (before control) !!	\n", __func__, vddId-4, millivolts);
	
	while( count < MAX_COUNT && millivolts!=0)
	{
		NvRmPmuSetVoltage(s_hRmGlobal, vddId, NVODM_VOLTAGE_OFF, &settling_time);
		udelay(settling_time);
		NvRmPmuGetVoltage(s_hRmGlobal, vddId, &millivolts);
		count=count+1;
	}
	
	if ( millivolts != 0 )
	{
		pr_info("[POWER] %s: LOD=%d (count=%d) Fail !!	\n", __func__, vddId-4, count);
	}
	else
	{
		pr_info("[POWER] %s: LOD=%d (count=%d) success !!	\n", __func__, vddId-4, count);
	}
}
//20100724  for poweroff leakage [LGE_END]

static void tegra_system_power_off(void)
{
	struct regulator *regulator = regulator_get(NULL, "soc_main");
	printk("tegra_system_power_off\n");

//20100724  for poweroff leakage [LGE_START]
    #if 0
	u32 settling_time;

	pr_info("[POWER] %s: start !!  \n",	__func__);

	NvRmPmuSetVoltage(s_hRmGlobal, Max8907PmuSupply_LDO5, 2800, &settling_time);
	udelay(settling_time);
	tegra_voltage_off(Max8907PmuSupply_LDO7);
	tegra_voltage_off(Max8907PmuSupply_LDO8);
	tegra_voltage_off(Max8907PmuSupply_LDO12);
	tegra_voltage_off(Max8907PmuSupply_LDO13);
	tegra_voltage_off(Max8907PmuSupply_LDO14);	
	tegra_voltage_off(Max8907PmuSupply_LDO18);
	tegra_voltage_off(Max8907PmuSupply_LDO3);
	tegra_voltage_off(Max8907PmuSupply_LDO4);
	tegra_voltage_off(Max8907PmuSupply_LDO5);	
    #endif
//20100724  for poweroff leakage [LGE_END]


	if (!IS_ERR(regulator)) {
		int rc;
		regulator_enable(regulator);
		rc = regulator_disable(regulator);
		pr_err("%s: regulator_disable returned %d\n", __func__, rc);
	} else {
		pr_err("%s: regulator_get returned %ld\n", __func__,
		       PTR_ERR(regulator));
	}
	local_irq_disable();
	while (1) {
		dsb();
		__asm__ ("wfi");
	}
}

static struct tegra_suspend_platform_data tegra_suspend_platform = {
	.cpu_timer = 2000,
};

static void __init tegra_setup_suspend(void)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	const int wakepad_irq[] = {
		gpio_to_irq(TEGRA_GPIO_PO5), gpio_to_irq(TEGRA_GPIO_PV3),
		gpio_to_irq(TEGRA_GPIO_PL1), gpio_to_irq(TEGRA_GPIO_PB6),
		gpio_to_irq(TEGRA_GPIO_PN7), gpio_to_irq(TEGRA_GPIO_PA0),
		gpio_to_irq(TEGRA_GPIO_PU5), gpio_to_irq(TEGRA_GPIO_PU6),
		gpio_to_irq(TEGRA_GPIO_PC7), gpio_to_irq(TEGRA_GPIO_PS2),
		gpio_to_irq(TEGRA_GPIO_PAA1), gpio_to_irq(TEGRA_GPIO_PW3),
		/* FIXME: USB/SDMMC wake pad interrupt mapping may be wrong */
		gpio_to_irq(TEGRA_GPIO_PW2), INT_SDMMC1,
		gpio_to_irq(TEGRA_GPIO_PV6), gpio_to_irq(TEGRA_GPIO_PJ7),
		INT_RTC, INT_KBC, INT_EXTERNAL_PMU,
		INT_USB, INT_USB3, INT_USB, INT_USB3,
		gpio_to_irq(TEGRA_GPIO_PI5), gpio_to_irq(TEGRA_GPIO_PV2),
		gpio_to_irq(TEGRA_GPIO_PS4), gpio_to_irq(TEGRA_GPIO_PS5),
		INT_SDMMC2, gpio_to_irq(TEGRA_GPIO_PQ6),
		gpio_to_irq(TEGRA_GPIO_PQ7), gpio_to_irq(TEGRA_GPIO_PN2),
		gpio_to_irq(TEGRA_GPIO_PG0), gpio_to_irq(TEGRA_GPIO_PG1),
	};
#endif
	const NvOdmWakeupPadInfo *w;
	const NvOdmSocPowerStateInfo *lp;
	struct tegra_suspend_platform_data *plat = &tegra_suspend_platform;
	NvOdmPmuProperty pmu;
	NvBool has_pmu;
	NvU32 nr_wake;

	lp = NvOdmQueryLowestSocPowerState();
	w = NvOdmQueryGetWakeupPadTable(&nr_wake);
	has_pmu = NvOdmQueryGetPmuProperty(&pmu);

	if (!has_pmu) {
		pr_info("%s: no PMU property, ignoring all suspend state\n",
			__func__);
		goto do_register;
	}

	if (lp->LowestPowerState==NvOdmSocPowerState_Suspend) {
		plat->dram_suspend = true;
		plat->core_off = false;
	} else if (lp->LowestPowerState==NvOdmSocPowerState_DeepSleep) {
		plat->dram_suspend = true;
		plat->core_off = true;
	}

	if (has_pmu) {
		plat->cpu_timer = pmu.CpuPowerGoodUs;
		plat->cpu_off_timer = pmu.CpuPowerOffUs;
		plat->core_timer = pmu.PowerGoodCount;
		plat->core_off_timer = pmu.PowerOffCount;

		plat->separate_req = !pmu.CombinedPowerReq;
		plat->corereq_high =
			(pmu.CorePowerReqPolarity ==
			 NvOdmCorePowerReqPolarity_High);
		plat->sysclkreq_high =
			(pmu.SysClockReqPolarity ==
			 NvOdmCorePowerReqPolarity_High);
	}

	if (!w || !nr_wake)
		goto do_register;

	plat->wake_enb = 0;
	plat->wake_low = 0;
	plat->wake_high = 0;
	plat->wake_any = 0;

	while (nr_wake--) {
		unsigned int pad = w->WakeupPadNumber;

#ifdef CONFIG_TEGRA_BATTERY_NVEC
		// pad 24 (gpio_pv2) on harmony should not be enabled as wake-up event
		// as battery charging current causes spurious events on this line and
		// thus causes un-expected wake-up from LP0
		if ((pad == 24) && !IsBoardTango())
			continue;
#endif
		if (pad < ARRAY_SIZE(wakepad_irq) && w->enable)
			enable_irq_wake(wakepad_irq[pad]);

		if (w->enable) {
			plat->wake_enb |= (1 << pad);

			if (w->Polarity == NvOdmWakeupPadPolarity_Low)
				plat->wake_low |= (1 << pad);
			else if (w->Polarity == NvOdmWakeupPadPolarity_High)
				plat->wake_high |= (1 << pad);
			else if (w->Polarity == NvOdmWakeupPadPolarity_AnyEdge)
				plat->wake_any |= (1 << pad);
		}
		w++;
	}

//20101117, cs77.ha@lge.com, gpio wakeup from LP1 [START]
#if defined(CONFIG_MACH_STAR)    
        // GPIO wakeup when entering LP1
#if defined(CONFIG_MACH_STAR_MDM_C)
	enable_irq_wake(gpio_to_irq(TEGRA_GPIO_PR2));  //proxi
#else
        enable_irq_wake(gpio_to_irq(TEGRA_GPIO_PW2));  //proxi
#endif
        enable_irq_wake(gpio_to_irq(TEGRA_GPIO_PG3));   //earjack sensor
        enable_irq_wake(gpio_to_irq(TEGRA_GPIO_PD3));   //hook detect
        enable_irq_wake(gpio_to_irq(TEGRA_GPIO_PG0));   //vol-up
        enable_irq_wake(gpio_to_irq(TEGRA_GPIO_PG1));   //vol-down
#endif
//20101117, cs77, gpio wakeup from LP1 [END]


do_register:
	tegra_init_suspend(plat);
	tegra_init_idle(plat);
}

#if defined (CONIFG_MACH_STAR)
extern void write_cmd_reserved_buffer(unsigned char *buf, size_t len);
#endif

static int tegra_reboot_notify(struct notifier_block *nb,
				unsigned long event, void *data)
{

#if defined (CONFIG_MACH_STAR)
        unsigned char tmpbuf[2];
      
	if(data)
	{
	    printk("tegra_reboot_notify : data = %s\n", (unsigned char *)data);
		memcpy(tmpbuf, (unsigned char*) data,2);
	}	
	else
	{
		tmpbuf[0] ='w';
	}	

        switch (tmpbuf[0])
        {
          case 'w':
          break;
#if defined (CONFIG_STAR_HIDDEN_RESET)
          case 'h':
          break;
#endif
          case 'p':
          break;

          default:
          tmpbuf[0] ='w';
          break;
        }
	write_cmd_reserved_buffer(tmpbuf,1);
#endif
	switch (event) {
	case SYS_RESTART:
	case SYS_HALT:
	case SYS_POWER_OFF:
		/* USB power rail must be enabled during boot */
		NvOdmEnableUsbPhyPowerRail(NV_TRUE);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block tegra_reboot_nb = {
	.notifier_call = tegra_reboot_notify,
	.next = NULL,
	.priority = 0
};


#if defined (CONFIG_MACH_STAR)
extern void write_cmd_reserved_buffer(unsigned char *buf, size_t len);

static int star_panic_event(struct notifier_block *this,
			       unsigned long event, void *ptr)
{
	return NOTIFY_DONE;
}

void star_shutdown_prepare()
{


#define MDM_POWER_ON 't' - 'a', 1
#define MDM_RESET 't' - 'a', 0

	NvOdmServicesGpioHandle hGpio;
	NvOdmGpioPinHandle hMDM_Power_On_GpioPinHandle;
	NvOdmGpioPinHandle hMDM_Reset_GpioPinHandle;

	NvOdmServicesPmuHandle h_pmu = NvOdmServicesPmuOpen();

	printk("star_shutdown_prepare : NvRMPrivDvsStop\n");
	NvRmPrivDvsStop();

	if (h_pmu)
	{

		printk("star_shutdown_prepare : pmic i2c shutdown\n");
		NvOdmServicesPmuSetVoltage( h_pmu, Max8907PmuSupply_Stop_i2c_Flag, 1UL, NULL );
	}
	else
	{
		printk("star_shutdown_prepare : sensor pmu handle fail!\n");
	}
	
	 hGpio = NvOdmGpioOpen();
     hMDM_Power_On_GpioPinHandle = NvOdmGpioAcquirePinHandle(hGpio, MDM_POWER_ON);
	 hMDM_Reset_GpioPinHandle = NvOdmGpioAcquirePinHandle(hGpio, MDM_RESET);

	 NvOdmGpioSetState(hGpio,hMDM_Power_On_GpioPinHandle , 0x0); 
	 NvOdmGpioConfig(hGpio, hMDM_Power_On_GpioPinHandle, 5 );

     NvOdmGpioSetState(hGpio,hMDM_Reset_GpioPinHandle , 0x0); 
     NvOdmGpioConfig(hGpio, hMDM_Reset_GpioPinHandle, 5 );
	 
     NvOdmGpioClose(hGpio);
	
}

EXPORT_SYMBOL_GPL(star_shutdown_prepare);


static void star_power_off_prepare()
{
    struct timespec ts;
	struct rtc_time tm;
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);

	pr_info("star_power_off_prepare: at %lld "
		"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
		ktime_to_ns(ktime_get()),
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);

}

struct notifier_block star_panic_nb = {
	.notifier_call = star_panic_event,
};
#endif

static void __init tegra_setup_reboot(void)
{
	int rc = register_reboot_notifier(&tegra_reboot_nb);
	if (rc)
		pr_err("%s: failed to regsiter platform reboot notifier\n",
			__func__);
#if defined (CONFIG_MACH_STAR)
        atomic_notifier_chain_register(&panic_notifier_list, & star_panic_nb);
#endif

}

static int __init tegra_setup_data(void)
{
	NvError e = NvSuccess;
	if (!s_hRmGlobal)
		e = NvRmOpenNew(&s_hRmGlobal);
	BUG_ON(e!=NvSuccess);
	platform_add_devices(nvodm_devices, ARRAY_SIZE(nvodm_devices));
	return 0;
}
postcore_initcall(tegra_setup_data);

void __init tegra_setup_nvodm(bool standard_i2c, bool standard_spi)
{
#if	defined(CONFIG_TOUCHSCREEN_PANJIT_I2C) && \
	defined(CONFIG_TOUCHSCREEN_ATMEL_MT_T9)
	NvOdmBoardInfo	BoardInfo;
#endif

	NvRmGpioOpen(s_hRmGlobal, &s_hGpioGlobal);
	tegra_setup_debug_uart();
	tegra_setup_hcd();
	tegra_setup_hsuart();
	tegra_setup_sdhci();
	tegra_setup_rfkill();
	tegra_setup_kbc();
	tegra_setup_gpio_key();
	if (standard_i2c)
		tegra_setup_i2c();
	if (standard_spi)
		tegra_setup_spi();
//20100613-1, , add spi slave [START]
#if defined(CONFIG_SPI_SLAVE_TEGRA) 
	tegra_setup_spi_slave();
#endif
//20100613-1, , add spi slave [END]
#if	defined(CONFIG_TOUCHSCREEN_PANJIT_I2C) && \
	defined(CONFIG_TOUCHSCREEN_ATMEL_MT_T9)
#define NVODM_ATMEL_TOUCHSCREEN    0x0A00
#define NVODM_PANJIT_TOUCHSCREEN   0x0000
#define BOARD_VENTANA              0x024B
	if (NvOdmPeripheralGetBoardInfo(BOARD_VENTANA, &BoardInfo)) {
		/* Diagnostics print messages to print Board SKU
		   printk("\n\nRRC:  Board ID:  %04X\n\n", BoardInfo.SKU);
		 */
		switch (BoardInfo.SKU & 0xFF00) {
		case NVODM_ATMEL_TOUCHSCREEN:
			ventana_touch_init_atmel();
			break;

		default:
			ventana_touch_init_panjit();
			break;
		}
	} else
		ventana_touch_init_panjit();
#elif defined(CONFIG_TOUCHSCREEN_ATMEL_MT_T9)
	ventana_touch_init_atmel();
#elif defined(CONFIG_TOUCHSCREEN_PANJIT_I2C)
	ventana_touch_init_panjit();
#endif
#ifdef CONFIG_TOUCHSCREEN_ANDROID_VIRTUALKEYS
	star_init_android_virtualkeys();
#endif
	tegra_emc_write_mrw(9);
	tegra_setup_w1();
#if defined (CONFIG_MACH_STAR)
	pm_power_off_prepare = star_power_off_prepare;
#endif
	pm_power_off = tegra_system_power_off;
	tegra_setup_suspend();
	tegra_setup_reboot();
}

void tegra_board_nvodm_suspend(void)
{
	if (console_suspend_enabled)
		tegra_debug_port_suspend();
#ifdef CONFIG_TEGRA_ODM_VENTANA
        tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDC, TEGRA_PUPD_NORMAL);
        tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDD, TEGRA_PUPD_NORMAL);
#endif
}

void tegra_board_nvodm_resume(void)
{
	if (console_suspend_enabled)
		tegra_debug_port_resume();
#ifdef CONFIG_TEGRA_ODM_VENTANA
        tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDC, TEGRA_PUPD_PULL_UP);
        tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDD, TEGRA_PUPD_PULL_UP);
#endif
}
