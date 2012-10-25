/*
 * arch/arm/mach-tegra/board-star-sdhci.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2011 NVIDIA Corporation.
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

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>

#include <mach-tegra/gpio-names.h>
#include <mach-tegra/board.h>

//#define WHISTLER_WLAN_PWR	TEGRA_GPIO_PK5
//#define WHISTLER_WLAN_RST	TEGRA_GPIO_PK6

#define STAR_WLAN_RST	TEGRA_GPIO_PW1

#define STAR_EXT_SDCARD_DETECT	TEGRA_GPIO_PI5

// LGE_CHANGE_S [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] tegra partition
const char *tegra_partition_list = NULL;
static int __init tegrapart_setup(char *options)
{
    if (options && *options && !tegra_partition_list)
        tegra_partition_list = options;
    return 0;
}

__setup("tegrapart=", tegrapart_setup);
// LGE_CHANGE_E [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] tegra partition

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
/*  sangjun.bae 20120220 wifi bringup START */
static struct clk *wifi_32k_clk;
/*  sangjun.bae 20120220 wifi bringup END */

static int star_wifi_status_register(
		void (*sdhcicallback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = sdhcicallback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int star_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int star_wifi_power(int on)
{
/*  sangjun.bae 20120220 wifi bringup START */
//	gpio_set_value(STAR_WLAN_PWR, on);
//	mdelay(100);
	if (on)
		clk_enable(wifi_32k_clk);
	else
		clk_disable(wifi_32k_clk);

/*  sangjun.bae 20120220 wifi bringup END */
	gpio_set_value(STAR_WLAN_RST, on);
	mdelay(200);

	return 0;
}

static int star_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}


static struct wifi_platform_data star_wifi_control = {
	.set_power      = star_wifi_power,
	.set_reset      = star_wifi_reset,
	.set_carddetect = star_wifi_set_carddetect,
};

static struct platform_device star_wifi_device = {
	.name           = "bcm4329_wlan",
	.id             = 1,
	.dev            = {
		.platform_data = &star_wifi_control,
	},
};

static struct resource sdhci_resource0[] = {
	[0] = {
		.start  = INT_SDMMC1,
		.end    = INT_SDMMC1,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC1_BASE,
		.end	= TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};


static struct resource sdhci_resource2[] = {
	[0] = {
		.start  = INT_SDMMC3,
		.end    = INT_SDMMC3,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start  = INT_SDMMC4,
		.end    = INT_SDMMC4,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct embedded_sdio_data embedded_sdio_data1 = {
	.cccr   = {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	},
	.cis  = {
		.vendor         = 0x02d0,
		.device         = 0x4329,
	},
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.mmc_data = {
		.register_status_notify	= star_wifi_status_register,
		.embedded_sdio = &embedded_sdio_data1,
		.built_in = 0,  //sangjun.bae 2012
	},
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.cd_gpio = STAR_EXT_SDCARD_DETECT,
	.wp_gpio = -1,
	.power_gpio = -1,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = 1,
	.mmc_data = {
		.built_in = 1,
	}
};

static struct platform_device tegra_sdhci_device0 = {
	.name		= "sdhci-tegra",
	.id		= 0,
	.resource	= sdhci_resource0,
	.num_resources	= ARRAY_SIZE(sdhci_resource0),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data0,
	},
};

static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data2,
	},
};

static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static int __init star_wifi_init(void)
{
/*  sangjun.bae 20120220 wifi bringup START */
	wifi_32k_clk = clk_get_sys(NULL, "blink");
	if (IS_ERR(wifi_32k_clk)) {
		pr_err("%s: unable to get blink clock\n", __func__);
		return PTR_ERR(wifi_32k_clk);
	}
		clk_enable(wifi_32k_clk);
	platform_device_register(&tegra_sdhci_device0);
//	gpio_request(WHISTLER_WLAN_PWR, "wlan_power");
	gpio_request(STAR_WLAN_RST, "wlan_rst");
//	gpio_request(WHISTLER_WLAN_WOW, "bcmsdh_sdmmc");

//	tegra_gpio_enable(WHISTLER_WLAN_PWR);
	tegra_gpio_enable(STAR_WLAN_RST);
//	tegra_gpio_enable(WHISTLER_WLAN_WOW);

//	gpio_direction_output(WHISTLER_WLAN_PWR, 0);
	gpio_direction_output(STAR_WLAN_RST, 0);
//	gpio_direction_input(WHISTLER_WLAN_WOW);
	tegra_sdhci_platform_data0.cd_gpio = STAR_WLAN_RST;
/*  sangjun.bae 20120220 wifi bringup END */

	platform_device_register(&star_wifi_device);
	return 0;
}

// LGE_CHANGE_S [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] tegra partition
static int Isdigit(int c)
{
    return (c>='0' && c<='9');
}

static int Isxdigit(int c)
{
    return (c>='0' && c<='9') || (c>='A' && c<='F') || (c>='a' && c<='f');
}

static int CharToXDigit(int c)
{
    return (c>='0' && c<='9') ? c - '0' :
           (c>='a' && c<='f') ? c - 'a' + 10 :
           (c>='A' && c<='F') ? c - 'A' + 10 : -1;
}

static unsigned long long int Strtoull(const char *s, char **endptr, int base)
{
    int neg = 0;
    unsigned long long int val = 0;

    if (*s == '-') {
        s++;
        neg = 1;
    }
    if (s[0]=='0' && (s[1]=='x' || s[1]=='X')) {
        if (base == 10) {
            if (endptr) {
                *endptr = (char*)s+1;
                return val;
            }
        }
        s += 2;
        base = 16;
    }

    if (base == 16) {
        while (Isxdigit(*s)) {
            val <<= 4;
            val +=  CharToXDigit(*s);
            s++;
#if DEBUG
			printk("0x%x\n", (int)val);
#endif
        }
    } else {
        while (Isdigit(*s)) {
            val *= 10;
            val += CharToXDigit(*s);
            s++;
        }
    }

    if (endptr) {
        *endptr = (char*)s;
    }
    return neg ? ((~val)+1) : val;
}

static int tegra_get_partition_info_by_name(
    const char *PartName,
    unsigned long long      *pSectorStart,
    unsigned long long      *pSectorLength,
    unsigned int	    *pSectorSize)
{
    int Len = strlen(PartName);
    const char *Ptr = tegra_partition_list;
    char *End;

    if (!Ptr)
        return -1;

    while (*Ptr && *Ptr!=' ')
    {
        if (!strncmp(Ptr, PartName, Len) && Ptr[Len]==':')
        {
            Ptr += Len + 1;
            *pSectorStart = Strtoull(Ptr, &End, 16);
            if (*End!=':')
                return -1;
            Ptr = End+1;
            *pSectorLength = Strtoull(Ptr, &End, 16);
            if (*End!=':')
                return -1;
            Ptr = End+1;
            *pSectorSize = Strtoull(Ptr, &End, 16);
			printk(KERN_DEBUG "tegra_get_partition_info_by_name: %s: pSectorStart: %d, pSectorLength: %d, pSectorSize: %d\n", PartName, (int) *pSectorStart, (int) *pSectorLength, (int) *pSectorSize);
            if (*End!=',' && *End!=' ' && *End)
                return -1;
            return 0;
        }
        else
        {
            while (*Ptr != ',' && *Ptr)
                Ptr++;
            if (!*Ptr)
                return -1;
            Ptr++;
        }
    }
    return -1;
}
// LGE_CHANGE_E [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] tegra partition


static int __init star_emmc_init(void)
{
// LGE_CHANGE_S [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] tegra partition
#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	unsigned long long start, length;
	unsigned int sector_size;
	int err;

	/*look for mbr partition*/
	err = tegra_get_partition_info_by_name("mbr", &start, &length, &sector_size);
	tegra_sdhci_platform_data3.startoffset = start * (unsigned long long)sector_size;
	printk(KERN_INFO "star_init_sdhci: MBR: err: %d, sector_start: %d, sector_length: %d, sector_size: %d, startoffset: %d\n", err, (int) start, (int) length, (int) sector_size, tegra_sdhci_platform_data3.startoffset);
#endif
// LGE_CHANGE_E [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] tegra partition

	platform_device_register(&tegra_sdhci_device3);

	return 0;
}

static int __init star_microsd_init(void)
{
	tegra_gpio_enable(STAR_EXT_SDCARD_DETECT);

	platform_device_register(&tegra_sdhci_device2);

	return 0;
}

int __init star_sdhci_init(void)
{
	int ret;
//TODO : error checking and handling

	star_emmc_init();
	star_microsd_init();
/*  sangjun.bae 20120220 wifi bringup START */
	printk("checking entering sdhci");

	star_wifi_init();
	printk("checking out of sdhci");
/*  sangjun.bae 20120220 wifi bringup END */

	return 0;
}
