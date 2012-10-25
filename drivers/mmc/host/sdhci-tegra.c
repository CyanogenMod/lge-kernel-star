/*
 * Copyright (C) 2010 Google, Inc.
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/err.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sd.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>

#include <mach/gpio.h>
#include <mach/sdhci.h>
#include <mach/io_dpd.h>

#include "sdhci.h"
#include "sdhci-pltfm.h"

#define SDHCI_VENDOR_CLOCK_CNTRL	0x100
#define SDHCI_VENDOR_CLOCK_CNTRL_SDMMC_CLK	0x1
#define SDHCI_VENDOR_CLOCK_CNTRL_PADPIPE_CLKEN_OVERRIDE	0x8
#define SDHCI_VENDOR_CLOCK_CNTRL_SPI_MODE_CLKEN_OVERRIDE	0x4
#define SDHCI_VENDOR_CLOCK_CNTRL_BASE_CLK_FREQ_SHIFT	8
#define SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_SHIFT	16
#define SDHCI_VENDOR_CLOCK_CNTRL_SDR50_TUNING		0x20

#define SDHCI_VENDOR_MISC_CNTRL		0x120
#define SDHCI_VENDOR_MISC_CNTRL_ENABLE_SDR104_SUPPORT	0x8
#define SDHCI_VENDOR_MISC_CNTRL_ENABLE_SDR50_SUPPORT	0x10
#define SDHCI_VENDOR_MISC_CNTRL_ENABLE_SD_3_0	0x20

#define SDMMC_SDMEMCOMPPADCTRL	0x1E0
#define SDMMC_SDMEMCOMPPADCTRL_VREF_SEL_MASK	0xF

#define SDMMC_AUTO_CAL_CONFIG	0x1E4
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE	0x20000000
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT	0x8
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET	0x70
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PU_OFFSET	0x62

#define SDHOST_1V8_OCR_MASK	0x8
#define SDHOST_HIGH_VOLT_MIN	2700000
#define SDHOST_HIGH_VOLT_MAX	3600000
#define SDHOST_LOW_VOLT_MIN	1800000
#define SDHOST_LOW_VOLT_MAX	1800000

// 2012-08-03 hyeondug.yeo@lge.com, Set SDIO clock for Wi-Fi.[S]
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#define TEGRA_SDHOST_MIN_FREQ	12000000 // From K36
#else
#define TEGRA_SDHOST_MIN_FREQ	50000000
#endif
// 2012-08-03 hyeondug.yeo@lge.com, Set SDIO clock for Wi-Fi.[E]
#define TEGRA2_SDHOST_STD_FREQ	50000000
#define TEGRA3_SDHOST_STD_FREQ	104000000

#define SD_SEND_TUNING_PATTERN	19
#define MAX_TAP_VALUES	256

static unsigned int tegra_sdhost_min_freq;
static unsigned int tegra_sdhost_std_freq;
static void tegra_3x_sdhci_set_card_clock(struct sdhci_host *sdhci, unsigned int clock);
static void tegra3_sdhci_post_reset_init(struct sdhci_host *sdhci);

static unsigned int tegra3_sdhost_max_clk[4] = {
	208000000,	104000000,	208000000,	104000000 };

struct tegra_sdhci_hw_ops{
	/* Set the internal clk and card clk.*/
	void	(*set_card_clock)(struct sdhci_host *sdhci, unsigned int clock);
	/* Post reset vendor registers configuration */
	void	(*sdhost_init)(struct sdhci_host *sdhci);
};

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
static struct tegra_sdhci_hw_ops tegra_2x_sdhci_ops = {
};
#else
static struct tegra_sdhci_hw_ops tegra_3x_sdhci_ops = {
	.set_card_clock = tegra_3x_sdhci_set_card_clock,
	.sdhost_init = tegra3_sdhci_post_reset_init,
};
#endif

struct tegra_sdhci_host {
	bool	clk_enabled;
// LGE_CHANGE [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] from Star
#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	unsigned int StartOffset;
#endif
	struct regulator *vdd_io_reg;
	struct regulator *vdd_slot_reg;
	/* Pointer to the chip specific HW ops */
	struct tegra_sdhci_hw_ops *hw_ops;
	/* Host controller instance */
	unsigned int instance;
	/* vddio_min */
	unsigned int vddio_min_uv;
	/* vddio_max */
	unsigned int vddio_max_uv;
	/* max clk supported by the platform */
	unsigned int max_clk_limit;
	struct tegra_io_dpd *dpd;
	bool card_present;
	bool is_rail_enabled;
};

// LGE_CHANGE [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] from Star
#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
static unsigned int tegra_sdhci_get_StartOffset(struct sdhci_host *host)
{
	struct tegra_sdhci_host *t_sdhci_host;

	t_sdhci_host = ((struct sdhci_pltfm_host *)sdhci_priv(host))->priv;

	return t_sdhci_host->StartOffset;
}
#endif

static u32 tegra_sdhci_readl(struct sdhci_host *host, int reg)
{
	u32 val;

	if (unlikely(reg == SDHCI_PRESENT_STATE)) {
		/* Use wp_gpio here instead? */
		val = readl(host->ioaddr + reg);
		return val | SDHCI_WRITE_PROTECT;
	}

	return readl(host->ioaddr + reg);
}

static u16 tegra_sdhci_readw(struct sdhci_host *host, int reg)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	if (unlikely(reg == SDHCI_HOST_VERSION)) {
		/* Erratum: Version register is invalid in HW. */
		return SDHCI_SPEC_200;
	}
#endif
	return readw(host->ioaddr + reg);
}

static void tegra_sdhci_writel(struct sdhci_host *host, u32 val, int reg)
{
	/* Seems like we're getting spurious timeout and crc errors, so
	 * disable signalling of them. In case of real errors software
	 * timers should take care of eventually detecting them.
	 */
	if (unlikely(reg == SDHCI_SIGNAL_ENABLE))
		val &= ~(SDHCI_INT_TIMEOUT|SDHCI_INT_CRC);

	writel(val, host->ioaddr + reg);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	if (unlikely(reg == SDHCI_INT_ENABLE)) {
		/* Erratum: Must enable block gap interrupt detection */
		u8 gap_ctrl = readb(host->ioaddr + SDHCI_BLOCK_GAP_CONTROL);
		if (val & SDHCI_INT_CARD_INT)
			gap_ctrl |= 0x8;
		else
			gap_ctrl &= ~0x8;
		writeb(gap_ctrl, host->ioaddr + SDHCI_BLOCK_GAP_CONTROL);
	}
#endif
}

static unsigned int tegra_sdhci_get_cd(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct tegra_sdhci_host *tegra_host = pltfm_host->priv;

	return tegra_host->card_present;
}

static unsigned int tegra_sdhci_get_ro(struct sdhci_host *sdhci)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(sdhci->mmc));
	struct tegra_sdhci_platform_data *plat;

	plat = pdev->dev.platform_data;

	if (!gpio_is_valid(plat->wp_gpio))
		return -1;

	return gpio_get_value(plat->wp_gpio);
}

static void tegra3_sdhci_post_reset_init(struct sdhci_host *sdhci)
{
	u16 misc_ctrl;
	u32 vendor_ctrl;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct tegra_sdhci_host *tegra_host = pltfm_host->priv;
	struct platform_device *pdev = to_platform_device(mmc_dev(sdhci->mmc));
	struct tegra_sdhci_platform_data *plat;

	plat = pdev->dev.platform_data;
	/* Set the base clock frequency */
	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
	vendor_ctrl &= ~(0xFF << SDHCI_VENDOR_CLOCK_CNTRL_BASE_CLK_FREQ_SHIFT);
	vendor_ctrl |= (tegra3_sdhost_max_clk[tegra_host->instance] / 1000000) <<
		SDHCI_VENDOR_CLOCK_CNTRL_BASE_CLK_FREQ_SHIFT;
	vendor_ctrl |= SDHCI_VENDOR_CLOCK_CNTRL_PADPIPE_CLKEN_OVERRIDE;
	vendor_ctrl &= ~SDHCI_VENDOR_CLOCK_CNTRL_SPI_MODE_CLKEN_OVERRIDE;

	/* Set tap delay */
	if (plat->tap_delay) {
		vendor_ctrl &= ~(0xFF <<
			SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_SHIFT);
		vendor_ctrl |= (plat->tap_delay <<
			SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_SHIFT);
	}
	/* Enable frequency tuning for SDR50 mode */
	vendor_ctrl |= SDHCI_VENDOR_CLOCK_CNTRL_SDR50_TUNING;
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VENDOR_CLOCK_CNTRL);

	/* Enable SDHOST v3.0 support */
	misc_ctrl = sdhci_readw(sdhci, SDHCI_VENDOR_MISC_CNTRL);
	misc_ctrl |= SDHCI_VENDOR_MISC_CNTRL_ENABLE_SD_3_0 |
		SDHCI_VENDOR_MISC_CNTRL_ENABLE_SDR104_SUPPORT |
		SDHCI_VENDOR_MISC_CNTRL_ENABLE_SDR50_SUPPORT;
	sdhci_writew(sdhci, misc_ctrl, SDHCI_VENDOR_MISC_CNTRL);
}

static int tegra_sdhci_set_uhs_signaling(struct sdhci_host *host,
		unsigned int uhs)
{
	u16 clk, ctrl_2;
	ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);

	/* Select Bus Speed Mode for host */
	ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
	switch (uhs) {
	case MMC_TIMING_UHS_SDR12:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR12;
		break;
	case MMC_TIMING_UHS_SDR25:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
		break;
	case MMC_TIMING_UHS_SDR50:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR50;
		break;
	case MMC_TIMING_UHS_SDR104:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR104;
		break;
	case MMC_TIMING_UHS_DDR50:
		ctrl_2 |= SDHCI_CTRL_UHS_DDR50;
		break;
	}

	sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);

	if (uhs == MMC_TIMING_UHS_DDR50) {
		clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
		clk &= ~(0xFF << SDHCI_DIVIDER_SHIFT);
		clk |= 1 << SDHCI_DIVIDER_SHIFT;
		sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
	}
	return 0;
}

static void tegra_sdhci_reset_exit(struct sdhci_host *sdhci, u8 mask)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct tegra_sdhci_host *tegra_host = pltfm_host->priv;

	if (mask & SDHCI_RESET_ALL) {
		if (tegra_host->hw_ops->sdhost_init)
			tegra_host->hw_ops->sdhost_init(sdhci);
	}
}

static void sdhci_status_notify_cb(int card_present, void *dev_id)
{
	struct sdhci_host *sdhci = (struct sdhci_host *)dev_id;
	struct platform_device *pdev = to_platform_device(mmc_dev(sdhci->mmc));
	struct tegra_sdhci_platform_data *plat;
	unsigned int status, oldstat;

	pr_debug("%s: card_present %d\n", mmc_hostname(sdhci->mmc),
		card_present);

	plat = pdev->dev.platform_data;
	if (!plat->mmc_data.status) {
		mmc_detect_change(sdhci->mmc, 0);
		return;
	}

	status = plat->mmc_data.status(mmc_dev(sdhci->mmc));

	oldstat = plat->mmc_data.card_present;
	plat->mmc_data.card_present = status;
	if (status ^ oldstat) {
		pr_debug("%s: Slot status change detected (%d -> %d)\n",
			mmc_hostname(sdhci->mmc), oldstat, status);
		if (status && !plat->mmc_data.built_in)
			mmc_detect_change(sdhci->mmc, (5 * HZ) / 2);
		else
			mmc_detect_change(sdhci->mmc, 0);
	}
}

static irqreturn_t carddetect_irq(int irq, void *data)
{
	struct sdhci_host *sdhost = (struct sdhci_host *)data;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhost);
	struct tegra_sdhci_host *tegra_host = pltfm_host->priv;
	struct platform_device *pdev = to_platform_device(mmc_dev(sdhost->mmc));
	struct tegra_sdhci_platform_data *plat;

	plat = pdev->dev.platform_data;

	tegra_host->card_present = (gpio_get_value(plat->cd_gpio) == 0);

	if (tegra_host->card_present) {
		if (!tegra_host->is_rail_enabled) {
			if (tegra_host->vdd_slot_reg)
				regulator_enable(tegra_host->vdd_slot_reg);
			if (tegra_host->vdd_io_reg)
				regulator_enable(tegra_host->vdd_io_reg);
			tegra_host->is_rail_enabled = 1;
		}
	} else {
		if (tegra_host->is_rail_enabled) {
			if (tegra_host->vdd_io_reg)
				regulator_disable(tegra_host->vdd_io_reg);
			if (tegra_host->vdd_slot_reg)
				regulator_disable(tegra_host->vdd_slot_reg);
			tegra_host->is_rail_enabled = 0;
                }
	}

	tasklet_schedule(&sdhost->card_tasklet);
	return IRQ_HANDLED;
};

static int tegra_sdhci_8bit(struct sdhci_host *host, int bus_width)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct tegra_sdhci_platform_data *plat;
	u32 ctrl;

	plat = pdev->dev.platform_data;

	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	if (plat->is_8bit && bus_width == MMC_BUS_WIDTH_8) {
		ctrl &= ~SDHCI_CTRL_4BITBUS;
		ctrl |= SDHCI_CTRL_8BITBUS;
	} else {
		ctrl &= ~SDHCI_CTRL_8BITBUS;
		if (bus_width == MMC_BUS_WIDTH_4)
			ctrl |= SDHCI_CTRL_4BITBUS;
		else
			ctrl &= ~SDHCI_CTRL_4BITBUS;
	}
	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
	return 0;
}

static void tegra_sdhci_set_clk_rate(struct sdhci_host *sdhci,
	unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct tegra_sdhci_host *tegra_host = pltfm_host->priv;
	unsigned int clk_rate;

	if (sdhci->mmc->card &&
		mmc_card_ddr_mode(sdhci->mmc->card)) {
		/*
		 * In ddr mode, tegra sdmmc controller clock frequency
		 * should be double the card clock frequency.
		 */
		 clk_rate = clock * 2;
	} else {
		if (clock <= tegra_sdhost_min_freq)
			clk_rate = tegra_sdhost_min_freq;
		else if (clock <= tegra_sdhost_std_freq)
			clk_rate = tegra_sdhost_std_freq;
		else
			clk_rate = clock;

		/*
		 * In SDR50 mode, run the sdmmc controller at 208MHz to ensure
		 * the core voltage is at 1.2V. If the core voltage is below 1.2V, CRC
		 * errors would occur during data transfers.
		 */
		if ((sdhci->mmc->ios.timing == MMC_TIMING_UHS_SDR50) &&
			(clk_rate == tegra_sdhost_std_freq))
			clk_rate <<= 1;
	}

	if (tegra_host->max_clk_limit &&
		(clk_rate > tegra_host->max_clk_limit))
		clk_rate = tegra_host->max_clk_limit;

	clk_set_rate(pltfm_host->clk, clk_rate);
	sdhci->max_clk = clk_get_rate(pltfm_host->clk);
}

static void tegra_3x_sdhci_set_card_clock(struct sdhci_host *sdhci, unsigned int clock)
{
	int div;
	u16 clk;
	unsigned long timeout;
	u8 ctrl;

	if (clock && clock == sdhci->clock)
		return;

	sdhci_writew(sdhci, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		goto out;
	if (sdhci->mmc->ios.timing == MMC_TIMING_UHS_DDR50) {
		div = 1;
		goto set_clk;
	}

	if (sdhci->version >= SDHCI_SPEC_300) {
		/* Version 3.00 divisors must be a multiple of 2. */
		if (sdhci->max_clk <= clock) {
			div = 1;
		} else {
			for (div = 2; div < SDHCI_MAX_DIV_SPEC_300; div += 2) {
				if ((sdhci->max_clk / div) <= clock)
					break;
			}
		}
	} else {
		/* Version 2.00 divisors must be a power of 2. */
		for (div = 1; div < SDHCI_MAX_DIV_SPEC_200; div *= 2) {
			if ((sdhci->max_clk / div) <= clock)
				break;
		}
	}
	div >>= 1;

	/*
	 * Tegra3 sdmmc controller internal clock will not be stabilized when
	 * we use a clock divider value greater than 4. The WAR is as follows.
	 * - Enable internal clock.
	 * - Wait for 5 usec and do a dummy write.
	 * - Poll for clk stable.
	 */
set_clk:
	clk = (div & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT;
	clk |= ((div & SDHCI_DIV_HI_MASK) >> SDHCI_DIV_MASK_LEN)
		<< SDHCI_DIVIDER_HI_SHIFT;
	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);

	/* Wait for 5 usec */
	udelay(5);

	/* Do a dummy write */
	ctrl = sdhci_readb(sdhci, SDHCI_CAPABILITIES);
	ctrl |= 1;
	sdhci_writeb(sdhci, ctrl, SDHCI_CAPABILITIES);

	/* Wait max 20 ms */
	timeout = 20;
	while (!((clk = sdhci_readw(sdhci, SDHCI_CLOCK_CONTROL))
		& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			dev_err(mmc_dev(sdhci->mmc), "Internal clock never stabilised\n");
			return;
		}
		timeout--;
		mdelay(1);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);
out:
	sdhci->clock = clock;
}

static void tegra_sdhci_set_clock(struct sdhci_host *sdhci, unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct tegra_sdhci_host *tegra_host = pltfm_host->priv;
	u8 ctrl;

	pr_debug("%s %s %u enabled=%u\n", __func__,
		mmc_hostname(sdhci->mmc), clock, tegra_host->clk_enabled);

	if (clock) {
		/* bring out sd instance from io dpd mode */
		tegra_io_dpd_disable(tegra_host->dpd);

		if (!tegra_host->clk_enabled) {
			clk_enable(pltfm_host->clk);
			ctrl = sdhci_readb(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
			ctrl |= SDHCI_VENDOR_CLOCK_CNTRL_SDMMC_CLK;
			sdhci_writeb(sdhci, ctrl, SDHCI_VENDOR_CLOCK_CNTRL);
			tegra_host->clk_enabled = true;
		}
		tegra_sdhci_set_clk_rate(sdhci, clock);
		if (tegra_host->hw_ops->set_card_clock)
			tegra_host->hw_ops->set_card_clock(sdhci, clock);
	} else if (!clock && tegra_host->clk_enabled) {
		if (tegra_host->hw_ops->set_card_clock)
			tegra_host->hw_ops->set_card_clock(sdhci, clock);
		ctrl = sdhci_readb(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
		ctrl &= ~SDHCI_VENDOR_CLOCK_CNTRL_SDMMC_CLK;
		sdhci_writeb(sdhci, ctrl, SDHCI_VENDOR_CLOCK_CNTRL);
		clk_disable(pltfm_host->clk);
		tegra_host->clk_enabled = false;
		/* io dpd enable call for sd instance */
		tegra_io_dpd_enable(tegra_host->dpd);
	}
}

static int tegra_sdhci_signal_voltage_switch(struct sdhci_host *sdhci,
	unsigned int signal_voltage)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct tegra_sdhci_host *tegra_host = pltfm_host->priv;
	unsigned int min_uV = SDHOST_HIGH_VOLT_MIN;
	unsigned int max_uV = SDHOST_HIGH_VOLT_MAX;
	unsigned int rc = 0;
	u16 clk, ctrl;
	unsigned int val;

	/* Switch OFF the card clock to prevent glitches on the clock line */
	clk = sdhci_readw(sdhci, SDHCI_CLOCK_CONTROL);
	clk &= ~SDHCI_CLOCK_CARD_EN;
	sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);

	ctrl = sdhci_readw(sdhci, SDHCI_HOST_CONTROL2);
	if (signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		ctrl |= SDHCI_CTRL_VDD_180;
		min_uV = SDHOST_LOW_VOLT_MIN;
		max_uV = SDHOST_LOW_VOLT_MAX;
	} else if (signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		if (ctrl & SDHCI_CTRL_VDD_180)
			ctrl &= ~SDHCI_CTRL_VDD_180;
	}
	sdhci_writew(sdhci, ctrl, SDHCI_HOST_CONTROL2);

	/* Switch the I/O rail voltage */
	if (tegra_host->vdd_io_reg) {
		rc = regulator_set_voltage(tegra_host->vdd_io_reg,
			min_uV, max_uV);
		if (rc) {
			dev_err(mmc_dev(sdhci->mmc), "switching to 1.8V"
			"failed . Switching back to 3.3V\n");
			regulator_set_voltage(tegra_host->vdd_io_reg,
				SDHOST_HIGH_VOLT_MIN,
				SDHOST_HIGH_VOLT_MAX);
			goto out;
		}
	}

	/* Wait for 10 msec for the voltage to be switched */
	mdelay(10);

	/* Enable the card clock */
	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);

	/* Wait for 1 msec after enabling clock */
	mdelay(1);

	if (signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		/* Do Auto Calibration for 1.8V signal voltage */
		val = sdhci_readl(sdhci, SDMMC_AUTO_CAL_CONFIG);
		val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE;
		/* Program Auto cal PD offset(bits 8:14) */
		val &= ~(0x7F <<
			SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT);
		val |= (SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET <<
			SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT);
		/* Program Auto cal PU offset(bits 0:6) */
		val &= ~0x7F;
		val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PU_OFFSET;
		sdhci_writel(sdhci, val, SDMMC_AUTO_CAL_CONFIG);

		val = sdhci_readl(sdhci, SDMMC_SDMEMCOMPPADCTRL);
		val &= ~SDMMC_SDMEMCOMPPADCTRL_VREF_SEL_MASK;
		val |= 0x7;
		sdhci_writel(sdhci, val, SDMMC_SDMEMCOMPPADCTRL);
	}

	return rc;
out:
	/* Enable the card clock */
	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);

	/* Wait for 1 msec for the clock to stabilize */
	mdelay(1);

	return rc;
}

static void tegra_sdhci_reset(struct sdhci_host *sdhci, u8 mask)
{
	unsigned long timeout;

	sdhci_writeb(sdhci, mask, SDHCI_SOFTWARE_RESET);

	/* Wait max 100 ms */
	timeout = 100;

	/* hw clears the bit when it's done */
	while (sdhci_readb(sdhci, SDHCI_SOFTWARE_RESET) & mask) {
		if (timeout == 0) {
			dev_err(mmc_dev(sdhci->mmc), "Reset 0x%x never"
				"completed.\n", (int)mask);
			return;
		}
		timeout--;
		mdelay(1);
	}
}

static void sdhci_tegra_set_tap_delay(struct sdhci_host *sdhci,
	unsigned int tap_delay)
{
	u32 vendor_ctrl;

	/* Max tap delay value is 255 */
	BUG_ON(tap_delay > MAX_TAP_VALUES);

	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
	vendor_ctrl &= ~(0xFF << SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_SHIFT);
	vendor_ctrl |= (tap_delay << SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_SHIFT);
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VENDOR_CLOCK_CNTRL);
}

static void sdhci_tegra_clear_set_irqs(struct sdhci_host *host,
	u32 clear, u32 set)
{
	u32 ier;

	ier = sdhci_readl(host, SDHCI_INT_ENABLE);
	ier &= ~clear;
	ier |= set;
	sdhci_writel(host, ier, SDHCI_INT_ENABLE);
	sdhci_writel(host, ier, SDHCI_SIGNAL_ENABLE);
}

static int sdhci_tegra_run_frequency_tuning(struct sdhci_host *sdhci)
{
	int err = 0;
	u8 ctrl;
	u32 ier;
	u32 mask;
	unsigned int timeout = 10;
	int flags;
	u32 intstatus;

	/*
	 * As per the Host Controller spec v3.00, tuning command
	 * generates Buffer Read Ready interrupt only, so enable that.
	 */
	ier = sdhci_readl(sdhci, SDHCI_INT_ENABLE);
	sdhci_tegra_clear_set_irqs(sdhci, ier, SDHCI_INT_DATA_AVAIL |
		SDHCI_INT_DATA_CRC);

	mask = SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT;
	while (sdhci_readl(sdhci, SDHCI_PRESENT_STATE) & mask) {
		if (timeout == 0) {
			dev_err(mmc_dev(sdhci->mmc), "Controller never"
				"released inhibit bit(s).\n");
			err = -ETIMEDOUT;
			goto out;
		}
		timeout--;
		mdelay(1);
	}

	ctrl = sdhci_readb(sdhci, SDHCI_HOST_CONTROL2);
	ctrl &= ~SDHCI_CTRL_TUNED_CLK;
	sdhci_writeb(sdhci, ctrl, SDHCI_HOST_CONTROL2);

	ctrl = sdhci_readb(sdhci, SDHCI_HOST_CONTROL2);
	ctrl |= SDHCI_CTRL_EXEC_TUNING;
	sdhci_writeb(sdhci, ctrl, SDHCI_HOST_CONTROL2);

	/*
	 * In response to CMD19, the card sends 64 bytes of tuning
	 * block to the Host Controller. So we set the block size
	 * to 64 here.
	 */
	sdhci_writew(sdhci, SDHCI_MAKE_BLKSZ(7, 64), SDHCI_BLOCK_SIZE);

	sdhci_writeb(sdhci, 0xE, SDHCI_TIMEOUT_CONTROL);

	sdhci_writeb(sdhci, SDHCI_TRNS_READ, SDHCI_TRANSFER_MODE);

	sdhci_writel(sdhci, 0x0, SDHCI_ARGUMENT);

	/* Set the cmd flags */
	flags = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_DATA;
	/* Issue the command */
	sdhci_writew(sdhci, SDHCI_MAKE_CMD(
		SD_SEND_TUNING_PATTERN, flags), SDHCI_COMMAND);

	timeout = 5;
	do {
		timeout--;
		mdelay(1);
		intstatus = sdhci_readl(sdhci, SDHCI_INT_STATUS);
		if (intstatus) {
			sdhci_writel(sdhci, intstatus, SDHCI_INT_STATUS);
			break;
		}
	} while(timeout);

	if ((intstatus & SDHCI_INT_DATA_AVAIL) &&
		!(intstatus & SDHCI_INT_DATA_CRC)) {
		err = 0;
		sdhci->tuning_done = 1;
	} else {
		tegra_sdhci_reset(sdhci, SDHCI_RESET_CMD);
		tegra_sdhci_reset(sdhci, SDHCI_RESET_DATA);
		err = -EIO;
	}

	if (sdhci->tuning_done) {
		sdhci->tuning_done = 0;
		ctrl = sdhci_readb(sdhci, SDHCI_HOST_CONTROL2);
		if (!(ctrl & SDHCI_CTRL_EXEC_TUNING) &&
			(ctrl & SDHCI_CTRL_TUNED_CLK))
			err = 0;
		else
			err = -EIO;
	}
	mdelay(1);
out:
	sdhci_tegra_clear_set_irqs(sdhci, SDHCI_INT_DATA_AVAIL, ier);
	return err;
}

static int sdhci_tegra_execute_tuning(struct sdhci_host *sdhci)
{
	int err;
	u16 ctrl_2;
	u8 *tap_delay_status;
	unsigned int i = 0;
	unsigned int temp_low_pass_tap = 0;
	unsigned int temp_pass_window = 0;
	unsigned int best_low_pass_tap = 0;
	unsigned int best_pass_window = 0;

	/* Tuning is valid only in SDR104 and SDR50 modes */
	ctrl_2 = sdhci_readw(sdhci, SDHCI_HOST_CONTROL2);
	if (!(((ctrl_2 & SDHCI_CTRL_UHS_MASK) == SDHCI_CTRL_UHS_SDR104) ||
		(((ctrl_2 & SDHCI_CTRL_UHS_MASK) == SDHCI_CTRL_UHS_SDR50) &&
		(sdhci->flags & SDHCI_SDR50_NEEDS_TUNING))))
			return 0;

	tap_delay_status = kzalloc(MAX_TAP_VALUES, GFP_KERNEL);
	if (tap_delay_status == NULL) {
		dev_err(mmc_dev(sdhci->mmc), "failed to allocate memory"
			"for storing tap_delay_status\n");
		err = -ENOMEM;
		goto out;
	}

	/*
	 * Set each tap delay value and run frequency tuning. After each
	 * run, update the tap delay status as working or not working.
	 */
	do {
		/* Set the tap delay */
		sdhci_tegra_set_tap_delay(sdhci, i);

		/* Run frequency tuning */
		err = sdhci_tegra_run_frequency_tuning(sdhci);

		/* Update whether the tap delay worked or not */
		tap_delay_status[i] = (err) ? 0: 1;
		i++;
	} while (i < 0xFF);

	/* Find the best possible tap range */
	for (i = 0; i < 0xFF; i++) {
		temp_pass_window = 0;

		/* Find the first passing tap in the current window */
		if (tap_delay_status[i]) {
			temp_low_pass_tap = i;

			/* Find the pass window */
			do {
				temp_pass_window++;
				i++;
				if (i > 0xFF)
					break;
			} while (tap_delay_status[i]);

			if ((temp_pass_window > best_pass_window) && (temp_pass_window > 1)){
				best_low_pass_tap = temp_low_pass_tap;
				best_pass_window = temp_pass_window;
			}
		}
	}


	pr_debug("%s: best pass tap window: start %d, end %d\n",
		mmc_hostname(sdhci->mmc), best_low_pass_tap,
		(best_low_pass_tap + best_pass_window));

	/* Set the best tap */
	sdhci_tegra_set_tap_delay(sdhci,
		(best_low_pass_tap + ((best_pass_window * 3) / 4)));

	/* Run frequency tuning */
	err = sdhci_tegra_run_frequency_tuning(sdhci);

out:
	if (tap_delay_status)
		kfree(tap_delay_status);

	return err;
}

static int tegra_sdhci_pltfm_init(struct sdhci_host *host,
				  struct sdhci_pltfm_data *pdata)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct tegra_sdhci_platform_data *plat;
	struct tegra_sdhci_host *tegra_host;
	struct clk *clk;
	int rc;

	plat = pdev->dev.platform_data;
	if (plat == NULL) {
		dev_err(mmc_dev(host->mmc), "missing platform data\n");
		return -ENXIO;
	}

	tegra_host = kzalloc(sizeof(struct tegra_sdhci_host), GFP_KERNEL);
	if (tegra_host == NULL) {
		dev_err(mmc_dev(host->mmc), "failed to allocate tegra host\n");
		return -ENOMEM;
	}

#ifdef CONFIG_MMC_EMBEDDED_SDIO
	if (plat->mmc_data.embedded_sdio)
		mmc_set_embedded_sdio_data(host->mmc,
			&plat->mmc_data.embedded_sdio->cis,
			&plat->mmc_data.embedded_sdio->cccr,
			plat->mmc_data.embedded_sdio->funcs,
			plat->mmc_data.embedded_sdio->num_funcs);
#endif

	if (gpio_is_valid(plat->power_gpio)) {
		rc = gpio_request(plat->power_gpio, "sdhci_power");
		if (rc) {
			dev_err(mmc_dev(host->mmc),
				"failed to allocate power gpio\n");
			goto out;
		}
		tegra_gpio_enable(plat->power_gpio);
		gpio_direction_output(plat->power_gpio, 1);
	}

	if (gpio_is_valid(plat->cd_gpio)) {

// LGE_CHANGE_S, [WiFi][ella.hwang@lge.com, moon-wifi@lge.com], 20120319, for X2-KDDI(KS1103) ICS
#if defined (CONFIG_MACH_BSSQ)
		if(plat->cd_gpio != 99){
#else // CONFIG_MACH_BSSQ
/*  sangjun.bae 20120220 wifi bringup START */

		if(plat->cd_gpio != 177){ // 20111028 jooin.woo@lge.com [Wi-Fi] initial work [START]
#endif // CONFIG_MACH_BSSQ
// LGE_CHANGE_E, [WiFi][ella.hwang@lge.com, moon-wifi@lge.com], 20120319, for X2-KDDI(KS1103) ICS
			rc = gpio_request(plat->cd_gpio, "sdhci_cd");
			if (rc) {
				dev_err(mmc_dev(host->mmc),
					"failed to allocate cd gpio\n");
				goto out_power;
			}
			tegra_gpio_enable(plat->cd_gpio);
			gpio_direction_input(plat->cd_gpio);
		} // 20111028 jooin.woo@lge.com [Wi-Fi] initial work [START]
		printk("[mingi.sung] sdhci_tegra probe() - before carddetect : WLAN_RST status is : %d", gpio_get_value(177));
/*		tegra_host->card_present = (gpio_get_value(plat->cd_gpio) == 0);

		rc = request_threaded_irq(gpio_to_irq(plat->cd_gpio), NULL,
				 carddetect_irq,
				 IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				 mmc_hostname(host->mmc), host);
*/
		rc = request_irq(gpio_to_irq(plat->cd_gpio), carddetect_irq,
				 IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				 mmc_hostname(host->mmc), host);

		printk("[mingi.sung] sdhci_tegra probe() - after carddetect : WLAN_RST status is : %d", gpio_get_value(177));
/*  sangjun.bae 20120220 wifi bringup END */

		if (rc)	{
			dev_err(mmc_dev(host->mmc), "request irq error\n");
			goto out_cd;
		}
		rc = enable_irq_wake(gpio_to_irq(plat->cd_gpio));
		if (rc < 0)
			dev_err(mmc_dev(host->mmc),
				"SD card wake-up event registration"
					"failed with eroor: %d\n", rc);

	} else if (plat->mmc_data.register_status_notify) {
		plat->mmc_data.register_status_notify(sdhci_status_notify_cb, host);
	}

        /*
         * If there is no card detect gpio, assume that the
         * card is always present.
         */
        if (!gpio_is_valid(plat->cd_gpio))
                tegra_host->card_present = 1;

	if (plat->mmc_data.status) {
		plat->mmc_data.card_present = plat->mmc_data.status(mmc_dev(host->mmc));
	}

	if (gpio_is_valid(plat->wp_gpio)) {
		rc = gpio_request(plat->wp_gpio, "sdhci_wp");
		if (rc) {
			dev_err(mmc_dev(host->mmc),
				"failed to allocate wp gpio\n");
			goto out_irq;
		}
		tegra_gpio_enable(plat->wp_gpio);
		gpio_direction_input(plat->wp_gpio);
	}


	if (!plat->mmc_data.built_in) {
		if (plat->mmc_data.ocr_mask & SDHOST_1V8_OCR_MASK) {
			tegra_host->vddio_min_uv = SDHOST_LOW_VOLT_MIN;
			tegra_host->vddio_max_uv = SDHOST_LOW_VOLT_MAX;
		} else {
			/*
			 * Set the minV and maxV to default
			 * voltage range of 2.7V - 3.6V
			 */
			tegra_host->vddio_min_uv = SDHOST_HIGH_VOLT_MIN;
			tegra_host->vddio_max_uv = SDHOST_HIGH_VOLT_MAX;
		}
		tegra_host->vdd_io_reg = regulator_get(mmc_dev(host->mmc), "vddio_sdmmc");
		if (IS_ERR_OR_NULL(tegra_host->vdd_io_reg)) {
			dev_err(mmc_dev(host->mmc), "%s regulator not found: %ld\n",
				"vddio_sdmmc", PTR_ERR(tegra_host->vdd_io_reg));
			tegra_host->vdd_io_reg = NULL;
		} else {
			rc = regulator_set_voltage(tegra_host->vdd_io_reg,
				tegra_host->vddio_min_uv,
				tegra_host->vddio_max_uv);
			if (rc) {
				dev_err(mmc_dev(host->mmc), "%s regulator_set_voltage failed: %d",
					"vddio_sdmmc", rc);
			}
		}

		tegra_host->vdd_slot_reg = regulator_get(mmc_dev(host->mmc), "vddio_sd_slot");
		if (IS_ERR_OR_NULL(tegra_host->vdd_slot_reg)) {
			dev_err(mmc_dev(host->mmc), "%s regulator not found: %ld\n",
				"vddio_sd_slot", PTR_ERR(tegra_host->vdd_slot_reg));
			tegra_host->vdd_slot_reg = NULL;
		}

		if (tegra_host->card_present) {
			if (tegra_host->vdd_slot_reg)
				regulator_enable(tegra_host->vdd_slot_reg);
			if (tegra_host->vdd_io_reg)
				regulator_enable(tegra_host->vdd_io_reg);
			tegra_host->is_rail_enabled = 1;
		}
	}

	// LGE_CHANGE [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] from Star
#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	tegra_host->StartOffset = plat->startoffset;
	printk(KERN_INFO "tegra_sdhci_probe: host->StartOffset: %d\n", tegra_host->StartOffset);
#endif
	
	clk = clk_get(mmc_dev(host->mmc), NULL);
	if (IS_ERR(clk)) {
		dev_err(mmc_dev(host->mmc), "clk err\n");
		rc = PTR_ERR(clk);
		goto out_wp;
	}
	rc = clk_enable(clk);
	if (rc != 0)
		goto err_clkput;
	pltfm_host->clk = clk;
	pltfm_host->priv = tegra_host;
	tegra_host->clk_enabled = true;
	tegra_host->max_clk_limit = plat->max_clk_limit;
	tegra_host->instance = pdev->id;
	tegra_host->dpd = tegra_io_dpd_get(mmc_dev(host->mmc));

	host->mmc->caps |= MMC_CAP_ERASE;
	host->mmc->caps |= MMC_CAP_DISABLE;
	/* enable 1/8V DDR capable */
	host->mmc->caps |= MMC_CAP_1_8V_DDR;
	if (plat->is_8bit)
		host->mmc->caps |= MMC_CAP_8_BIT_DATA;
	host->mmc->caps |= MMC_CAP_SDIO_IRQ;
	host->mmc->caps |= MMC_CAP_BKOPS;

	host->mmc->pm_caps = MMC_PM_KEEP_POWER | MMC_PM_IGNORE_PM_NOTIFY;
	if (plat->mmc_data.built_in) {
		host->mmc->caps |= MMC_CAP_NONREMOVABLE;
	} // 2012-08-03 hyeondug.yeo@lge.com, Set SDIO clock for Wi-Fi.
	host->mmc->pm_flags = MMC_PM_IGNORE_PM_NOTIFY;	//Nvidia internal change

	/* Do not turn OFF embedded sdio cards as it support Wake on Wireless */
	if (plat->mmc_data.embedded_sdio)
		host->mmc->pm_flags |= MMC_PM_KEEP_POWER;

	tegra_sdhost_min_freq = TEGRA_SDHOST_MIN_FREQ;
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	tegra_host->hw_ops = &tegra_2x_sdhci_ops;
	tegra_sdhost_std_freq = TEGRA2_SDHOST_STD_FREQ;
#else
	tegra_host->hw_ops = &tegra_3x_sdhci_ops;
	tegra_sdhost_std_freq = TEGRA3_SDHOST_STD_FREQ;
#endif

	return 0;

err_clkput:
	clk_put(clk);

out_wp:
	if (gpio_is_valid(plat->wp_gpio)) {
		tegra_gpio_disable(plat->wp_gpio);
		gpio_free(plat->wp_gpio);
	}

out_irq:
	if (gpio_is_valid(plat->cd_gpio))
		free_irq(gpio_to_irq(plat->cd_gpio), host);
out_cd:
	if (gpio_is_valid(plat->cd_gpio)) {
		tegra_gpio_disable(plat->cd_gpio);
		gpio_free(plat->cd_gpio);
	}

out_power:
	if (gpio_is_valid(plat->power_gpio)) {
		tegra_gpio_disable(plat->power_gpio);
		gpio_free(plat->power_gpio);
	}

out:
	kfree(tegra_host);
	return rc;
}

static void tegra_sdhci_pltfm_exit(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct tegra_sdhci_host *tegra_host = pltfm_host->priv;
	struct tegra_sdhci_platform_data *plat;

	plat = pdev->dev.platform_data;

	disable_irq_wake(gpio_to_irq(plat->cd_gpio));

	if (tegra_host->vdd_slot_reg) {
		regulator_disable(tegra_host->vdd_slot_reg);
		regulator_put(tegra_host->vdd_slot_reg);
	}

	if (tegra_host->vdd_io_reg) {
		regulator_disable(tegra_host->vdd_io_reg);
		regulator_put(tegra_host->vdd_io_reg);
	}

	if (gpio_is_valid(plat->wp_gpio)) {
		tegra_gpio_disable(plat->wp_gpio);
		gpio_free(plat->wp_gpio);
	}

	if (gpio_is_valid(plat->cd_gpio)) {
		free_irq(gpio_to_irq(plat->cd_gpio), host);
		tegra_gpio_disable(plat->cd_gpio);
		gpio_free(plat->cd_gpio);
	}

	if (gpio_is_valid(plat->power_gpio)) {
		tegra_gpio_disable(plat->power_gpio);
		gpio_free(plat->power_gpio);
	}

	if (tegra_host->clk_enabled)
		clk_disable(pltfm_host->clk);
	clk_put(pltfm_host->clk);

	kfree(tegra_host);
}

static int tegra_sdhci_suspend(struct sdhci_host *sdhci, pm_message_t state)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct tegra_sdhci_host *tegra_host = pltfm_host->priv;

// 2012-08-03 hyeondug.yeo@lge.com, Set SDIO clock for Wi-Fi.[S]
#if defined (CONFIG_MACH_STAR) // From K36 code
	if (sdhci->mmc->pm_flags & MMC_PM_KEEP_POWER) {
		int div = 0;
		u16 clk;
		unsigned int clock = 100000;

		/* reduce host controller clk and card clk to 100 KHz */
		tegra_sdhci_set_clock(sdhci, clock);
		sdhci_writew(sdhci, 0, SDHCI_CLOCK_CONTROL);

		if (sdhci->max_clk > clock) {
			div =  1 << (fls(sdhci->max_clk / clock) - 2);
			if (div > 128)
				div = 128;
		}

		clk = div << SDHCI_DIVIDER_SHIFT;
		clk |= SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_CARD_EN;
		sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);
		
		return 0;
	}
#endif
// 2012-08-03 hyeondug.yeo@lge.com, Set SDIO clock for Wi-Fi.[E]

	tegra_sdhci_set_clock(sdhci, 0);

	/* Disable the power rails if any */
	if (tegra_host->card_present) {
		if (tegra_host->is_rail_enabled) {
			if (tegra_host->vdd_io_reg)
				regulator_disable(tegra_host->vdd_io_reg);
			if (tegra_host->vdd_slot_reg)
				regulator_disable(tegra_host->vdd_slot_reg);
			tegra_host->is_rail_enabled = 0;
		}
	}

	return 0;
}

static int tegra_sdhci_resume(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct tegra_sdhci_host *tegra_host = pltfm_host->priv;

	/* Enable the power rails if any */
	if (tegra_host->card_present) {
		if (!tegra_host->is_rail_enabled) {
			if (tegra_host->vdd_slot_reg)
				regulator_enable(tegra_host->vdd_slot_reg);
			if (tegra_host->vdd_io_reg) {
                                regulator_enable(tegra_host->vdd_io_reg);
                                tegra_sdhci_signal_voltage_switch(sdhci, MMC_SIGNAL_VOLTAGE_330);
                        }
			tegra_host->is_rail_enabled = 1;
		}
	}

	/* Setting the min identification clock of freq 400KHz */
	tegra_sdhci_set_clock(sdhci, 400000);

	/* Reset the controller and power on if MMC_KEEP_POWER flag is set*/
	if (sdhci->mmc->pm_flags & MMC_PM_KEEP_POWER) {
		tegra_sdhci_reset(sdhci, SDHCI_RESET_ALL);

		sdhci_writeb(sdhci, SDHCI_POWER_ON, SDHCI_POWER_CONTROL);
		sdhci->pwr = 0;
	}

	return 0;
}

static struct sdhci_ops tegra_sdhci_ops = {
	.get_ro     = tegra_sdhci_get_ro,
	.get_cd     = tegra_sdhci_get_cd,
	.read_l     = tegra_sdhci_readl,
	.read_w     = tegra_sdhci_readw,
	.write_l    = tegra_sdhci_writel,
	.platform_8bit_width = tegra_sdhci_8bit,
	.set_clock  = tegra_sdhci_set_clock,
	.suspend    = tegra_sdhci_suspend,
	.resume     = tegra_sdhci_resume,
	.platform_reset_exit = tegra_sdhci_reset_exit,
	.set_uhs_signaling = tegra_sdhci_set_uhs_signaling,
	.switch_signal_voltage = tegra_sdhci_signal_voltage_switch,
	// LGE_CHANGE [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] from Star
#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	.get_startoffset = tegra_sdhci_get_StartOffset,
#endif

	.execute_freq_tuning = sdhci_tegra_execute_tuning,
};

struct sdhci_pltfm_data sdhci_tegra_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
		  SDHCI_QUIRK_NONSTANDARD_CLOCK |
		  SDHCI_QUIRK_NON_STD_VOLTAGE_SWITCHING |
		  SDHCI_QUIRK_NON_STANDARD_TUNING |
#endif
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_BROKEN_CARD_DETECTION,
	.ops  = &tegra_sdhci_ops,
	.init = tegra_sdhci_pltfm_init,
	.exit = tegra_sdhci_pltfm_exit,
};
