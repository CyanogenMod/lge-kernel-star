/*
 * arch/arm/mach-tegra/usb_phy.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 - 2011 NVIDIA Corporation
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
 *	Benoit Goby <benoit@android.com>
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
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>
#include <asm/mach-types.h>
#include <mach/usb_phy.h>
#include <mach/iomap.h>
#include <mach/pinmux.h>

#include "fuse.h"

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#define USB_USBCMD		0x140
#define   USB_USBCMD_RS		(1 << 0)

#define USB_USBSTS		0x144
#define   USB_USBSTS_PCI	(1 << 2)
#define   USB_USBSTS_HCH	(1 << 12)

#define USB_TXFILLTUNING        0x164
#define USB_FIFO_TXFILL_THRES(x)   (((x) & 0x1f) << 16)
#define USB_FIFO_TXFILL_MASK    0x1f0000

#define ULPI_VIEWPORT		0x170
#define   ULPI_WAKEUP		(1 << 31)
#define   ULPI_RUN		(1 << 30)
#define   ULPI_RD_WR		(1 << 29)

#define USB_PORTSC1		0x184
#define   USB_PORTSC1_PTS(x)	(((x) & 0x3) << 30)
#define   USB_PORTSC1_PSPD(x)	(((x) & 0x3) << 26)
#define   USB_PORTSC1_PHCD	(1 << 23)
#define   USB_PORTSC1_WKOC	(1 << 22)
#define   USB_PORTSC1_WKDS	(1 << 21)
#define   USB_PORTSC1_WKCN	(1 << 20)
#define   USB_PORTSC1_PTC(x)	(((x) & 0xf) << 16)
#define   USB_PORTSC1_PP	(1 << 12)
#define   USB_PORTSC1_LS(x)	(((x) & 0x3) << 10)
#define   USB_PORTSC1_SUSP	(1 << 7)
#define   USB_PORTSC1_PE	(1 << 2)
#define   USB_PORTSC1_CCS	(1 << 0)

#define USB_SUSP_CTRL		0x400
#define   USB_WAKE_ON_CNNT_EN_DEV	(1 << 3)
#define   USB_WAKE_ON_DISCON_EN_DEV	(1 << 4)
#define   USB_SUSP_CLR		(1 << 5)
#define   USB_CLKEN             (1 << 6)
#define   USB_PHY_CLK_VALID	(1 << 7)
#define   USB_PHY_CLK_VALID_INT_ENB    (1 << 9)
#define   UTMIP_RESET		(1 << 11)
#define   UHSIC_RESET		(1 << 11)
#define   UTMIP_PHY_ENABLE	(1 << 12)
#define   UHSIC_PHY_ENABLE	(1 << 12)
#define   ULPI_PHY_ENABLE	(1 << 13)
#define   USB_SUSP_SET		(1 << 14)
#define   USB_WAKEUP_DEBOUNCE_COUNT(x)	(((x) & 0x7) << 16)

#define USB1_LEGACY_CTRL	0x410
#define   USB1_NO_LEGACY_MODE			(1 << 0)
#define   USB1_VBUS_SENSE_CTL_MASK		(3 << 1)
#define   USB1_VBUS_SENSE_CTL_VBUS_WAKEUP	(0 << 1)
#define   USB1_VBUS_SENSE_CTL_AB_SESS_VLD_OR_VBUS_WAKEUP \
						(1 << 1)
#define   USB1_VBUS_SENSE_CTL_AB_SESS_VLD	(2 << 1)
#define   USB1_VBUS_SENSE_CTL_A_SESS_VLD	(3 << 1)


#define UTMIP_PLL_CFG1		0x804
#define   UTMIP_XTAL_FREQ_COUNT(x)		(((x) & 0xfff) << 0)
#define   UTMIP_PLLU_ENABLE_DLY_COUNT(x)	(((x) & 0x1f) << 27)

#define UTMIP_XCVR_CFG0		0x808
#define   UTMIP_XCVR_SETUP(x)			(((x) & 0xf) << 0)
#define   UTMIP_XCVR_LSRSLEW(x)			(((x) & 0x3) << 8)
#define   UTMIP_XCVR_LSFSLEW(x)			(((x) & 0x3) << 10)
#define   UTMIP_FORCE_PD_POWERDOWN		(1 << 14)
#define   UTMIP_FORCE_PD2_POWERDOWN		(1 << 16)
#define   UTMIP_FORCE_PDZI_POWERDOWN		(1 << 18)
#define   UTMIP_XCVR_LSBIAS_SEL			(1 << 21)
#define   UTMIP_XCVR_HSSLEW_MSB(x)		(((x) & 0x7f) << 25)

#define UTMIP_BIAS_CFG0		0x80c
#define   UTMIP_OTGPD			(1 << 11)
#define   UTMIP_BIASPD			(1 << 10)

#define UTMIP_HSRX_CFG0		0x810
#define   UTMIP_ELASTIC_LIMIT(x)	(((x) & 0x1f) << 10)
#define   UTMIP_IDLE_WAIT(x)		(((x) & 0x1f) << 15)

#define UTMIP_HSRX_CFG1		0x814
#define   UTMIP_HS_SYNC_START_DLY(x)	(((x) & 0x1f) << 1)

#define UTMIP_TX_CFG0		0x820
#define   UTMIP_FS_PREABMLE_J		(1 << 19)
#define   UTMIP_HS_DISCON_DISABLE	(1 << 8)

#define UTMIP_MISC_CFG0		0x824
#define   UTMIP_DPDM_OBSERVE		(1 << 26)
#define   UTMIP_DPDM_OBSERVE_SEL(x)	(((x) & 0xf) << 27)
#define   UTMIP_DPDM_OBSERVE_SEL_FS_J	UTMIP_DPDM_OBSERVE_SEL(0xf)
#define   UTMIP_DPDM_OBSERVE_SEL_FS_K	UTMIP_DPDM_OBSERVE_SEL(0xe)
#define   UTMIP_DPDM_OBSERVE_SEL_FS_SE1 UTMIP_DPDM_OBSERVE_SEL(0xd)
#define   UTMIP_DPDM_OBSERVE_SEL_FS_SE0 UTMIP_DPDM_OBSERVE_SEL(0xc)
#define   UTMIP_SUSPEND_EXIT_ON_EDGE	(1 << 22)

#define UTMIP_MISC_CFG1		0x828
#define   UTMIP_PLL_ACTIVE_DLY_COUNT(x)	(((x) & 0x1f) << 18)
#define   UTMIP_PLLU_STABLE_COUNT(x)	(((x) & 0xfff) << 6)

#define UTMIP_DEBOUNCE_CFG0	0x82c
#define   UTMIP_BIAS_DEBOUNCE_A(x)	(((x) & 0xffff) << 0)

#define UTMIP_BAT_CHRG_CFG0	0x830
#define   UTMIP_PD_CHRG			(1 << 0)

#define UTMIP_XCVR_CFG1		0x838
#define   UTMIP_FORCE_PDDISC_POWERDOWN	(1 << 0)
#define   UTMIP_FORCE_PDCHRP_POWERDOWN	(1 << 2)
#define   UTMIP_FORCE_PDDR_POWERDOWN	(1 << 4)
#define   UTMIP_XCVR_TERM_RANGE_ADJ(x)	(((x) & 0xf) << 18)

#define UTMIP_BIAS_CFG1		0x83c
#define   UTMIP_BIAS_PDTRK_COUNT(x)	(((x) & 0x1f) << 3)

#define UHSIC_PLL_CFG1				0x804
#define   UHSIC_XTAL_FREQ_COUNT(x)		(((x) & 0xfff) << 0)
#define   UHSIC_PLLU_ENABLE_DLY_COUNT(x)	(((x) & 0x1f) << 14)

#define UHSIC_HSRX_CFG0				0x808
#define   UHSIC_ELASTIC_UNDERRUN_LIMIT(x)	(((x) & 0x1f) << 2)
#define   UHSIC_ELASTIC_OVERRUN_LIMIT(x)	(((x) & 0x1f) << 8)
#define   UHSIC_IDLE_WAIT(x)			(((x) & 0x1f) << 13)

#define UHSIC_HSRX_CFG1				0x80c
#define   UHSIC_HS_SYNC_START_DLY(x)		(((x) & 0x1f) << 1)

#define UHSIC_MISC_CFG0				0x814
#define   UHSIC_SUSPEND_EXIT_ON_EDGE		(1 << 7)
#define   UHSIC_DETECT_SHORT_CONNECT		(1 << 8)
#define   UHSIC_FORCE_XCVR_MODE			(1 << 15)

#define UHSIC_MISC_CFG1				0X818
#define   UHSIC_PLLU_STABLE_COUNT(x)		(((x) & 0xfff) << 2)

#define UHSIC_PADS_CFG0				0x81c
#define   UHSIC_TX_RTUNEN			0xf000
#define   UHSIC_TX_RTUNE(x)			(((x) & 0xf) << 12)

#define UHSIC_PADS_CFG1				0x820
#define   UHSIC_PD_BG				(1 << 2)
#define   UHSIC_PD_TX				(1 << 3)
#define   UHSIC_PD_TRK				(1 << 4)
#define   UHSIC_PD_RX				(1 << 5)
#define   UHSIC_PD_ZI				(1 << 6)
#define   UHSIC_RX_SEL				(1 << 7)
#define   UHSIC_RPD_DATA			(1 << 9)
#define   UHSIC_RPD_STROBE			(1 << 10)
#define   UHSIC_RPU_DATA			(1 << 11)
#define   UHSIC_RPU_STROBE			(1 << 12)

#define UHSIC_STAT_CFG0				0x828
#define   UHSIC_CONNECT_DETECT			(1 << 0)


#else

#define USB_USBCMD		0x130
#define   USB_USBCMD_RS		(1 << 0)

#define USB_USBSTS		0x134
#define   USB_USBSTS_PCI	(1 << 2)
#define   USB_USBSTS_HCH	(1 << 12)

#define ULPI_VIEWPORT		0x160

#define USB_PORTSC1		0x174
#define   USB_PORTSC1_WKOC	(1 << 22)
#define   USB_PORTSC1_WKDS	(1 << 21)
#define   USB_PORTSC1_WKCN	(1 << 20)
#define   USB_PORTSC1_PTC(x)	(((x) & 0xf) << 16)
#define   USB_PORTSC1_PP	(1 << 12)
#define   USB_PORTSC1_SUSP	(1 << 7)
#define   USB_PORTSC1_PE	(1 << 2)
#define   USB_PORTSC1_CCS	(1 << 0)

#define USB_SUSP_CTRL		0x400
#define   USB_WAKE_ON_CNNT_EN_DEV	(1 << 3)
#define   USB_WAKE_ON_DISCON_EN_DEV	(1 << 4)
#define   USB_SUSP_CLR			(1 << 5)
#define   USB_PHY_CLK_VALID		(1 << 7)
#define   USB_PHY_CLK_VALID_INT_ENB    (1 << 9)


#define   UTMIP_RESET			(1 << 11)
#define   UTMIP_PHY_ENABLE		(1 << 12)
#define   ULPI_PHY_ENABLE		(1 << 13)
#define   UHSIC_RESET			(1 << 14)

#define   USB_WAKEUP_DEBOUNCE_COUNT(x)	(((x) & 0x7) << 16)
#define   UHSIC_PHY_ENABLE		(1 << 19)
#define   ULPIS2S_SLV0_RESET		(1 << 20)
#define   ULPIS2S_SLV1_RESET		(1 << 21)
#define   ULPIS2S_LINE_RESET		(1 << 22)
#define   ULPI_PADS_RESET		(1 << 23)
#define   ULPI_PADS_CLKEN_RESET		(1 << 24)

#define USB1_LEGACY_CTRL	0x410
#define   USB1_NO_LEGACY_MODE			(1 << 0)
#define   USB1_VBUS_SENSE_CTL_MASK		(3 << 1)
#define   USB1_VBUS_SENSE_CTL_VBUS_WAKEUP	(0 << 1)
#define   USB1_VBUS_SENSE_CTL_AB_SESS_VLD_OR_VBUS_WAKEUP \
						(1 << 1)
#define   USB1_VBUS_SENSE_CTL_AB_SESS_VLD	(2 << 1)
#define   USB1_VBUS_SENSE_CTL_A_SESS_VLD	(3 << 1)

#define UTMIP_PLL_CFG1		0x804
#define   UTMIP_XTAL_FREQ_COUNT(x)		(((x) & 0xfff) << 0)
#define   UTMIP_PLLU_ENABLE_DLY_COUNT(x)	(((x) & 0x1f) << 27)

#define UTMIP_XCVR_CFG0		0x808
#define   UTMIP_XCVR_SETUP(x)			(((x) & 0xf) << 0)
#define   UTMIP_XCVR_LSRSLEW(x)			(((x) & 0x3) << 8)
#define   UTMIP_XCVR_LSFSLEW(x)			(((x) & 0x3) << 10)
#define   UTMIP_FORCE_PD_POWERDOWN		(1 << 14)
#define   UTMIP_FORCE_PD2_POWERDOWN		(1 << 16)
#define   UTMIP_FORCE_PDZI_POWERDOWN		(1 << 18)
#define   UTMIP_XCVR_LSBIAS_SEL			(1 << 21)
#define   UTMIP_XCVR_HSSLEW_MSB(x)		(((x) & 0x7f) << 25)

#define UTMIP_BIAS_CFG0		0x80c
#define   UTMIP_OTGPD			(1 << 11)
#define   UTMIP_BIASPD			(1 << 10)

#define UTMIP_HSRX_CFG0		0x810
#define   UTMIP_ELASTIC_LIMIT(x)	(((x) & 0x1f) << 10)
#define   UTMIP_IDLE_WAIT(x)		(((x) & 0x1f) << 15)

#define UTMIP_HSRX_CFG1		0x814
#define   UTMIP_HS_SYNC_START_DLY(x)	(((x) & 0x1f) << 1)

#define UTMIP_TX_CFG0		0x820
#define   UTMIP_FS_PREABMLE_J		(1 << 19)
#define   UTMIP_HS_DISCON_DISABLE	(1 << 8)

#define UTMIP_MISC_CFG0		0x824
#define   UTMIP_SUSPEND_EXIT_ON_EDGE	(1 << 22)

#define UTMIP_MISC_CFG1		0x828
#define   UTMIP_PLL_ACTIVE_DLY_COUNT(x)	(((x) & 0x1f) << 18)
#define   UTMIP_PLLU_STABLE_COUNT(x)	(((x) & 0xfff) << 6)

#define UTMIP_DEBOUNCE_CFG0	0x82c
#define   UTMIP_BIAS_DEBOUNCE_A(x)	(((x) & 0xffff) << 0)

#define UTMIP_BAT_CHRG_CFG0	0x830
#define   UTMIP_PD_CHRG			(1 << 0)

#define UTMIP_XCVR_CFG1		0x838
#define   UTMIP_FORCE_PDDISC_POWERDOWN	(1 << 0)
#define   UTMIP_FORCE_PDCHRP_POWERDOWN	(1 << 2)
#define   UTMIP_FORCE_PDDR_POWERDOWN	(1 << 4)
#define   UTMIP_XCVR_TERM_RANGE_ADJ(x)	(((x) & 0xf) << 18)

#define UTMIP_BIAS_CFG1		0x83c
#define   UTMIP_BIAS_PDTRK_COUNT(x)	(((x) & 0x1f) << 3)

#define HOSTPC1_DEVLC		0x1b4
#define   HOSTPC1_DEVLC_PHCD		(1 << 22)
#define   HOSTPC1_DEVLC_PTS(x)		(((x) & 0x7) << 29)
#define   HOSTPC1_DEVLC_PTS_MASK	7
#define   HOSTPC1_DEVLC_PTS_HSIC	4
#define   HOSTPC1_DEVLC_STS 		(1 << 28)
#define   HOSTPC1_DEVLC_PSPD(x)		(((x) & 0x3) << 25)
#define   HOSTPC1_DEVLC_PSPD_MASK	3
#define   HOSTPC1_DEVLC_PSPD_HIGH_SPEED	2

#define TEGRA_USB_USBMODE_REG_OFFSET	0x1f8
#define   TEGRA_USB_USBMODE_HOST		(3 << 0)

#define TEGRA_PMC_USB_AO		0xf0
#define   TEGRA_PMC_USB_AO_VBUS_WAKEUP_PD_P0	(1 << 2)
#define   TEGRA_PMC_USB_AO_ID_PD_P0		(1 << 3)

#define ICUSB_CTRL		0x15c

#define UHSIC_PLL_CFG1				0xc04
#define   UHSIC_XTAL_FREQ_COUNT(x)		(((x) & 0xfff) << 0)
#define   UHSIC_PLLU_ENABLE_DLY_COUNT(x)	(((x) & 0x1f) << 14)

#define UHSIC_HSRX_CFG0				0xc08
#define   UHSIC_ELASTIC_UNDERRUN_LIMIT(x)	(((x) & 0x1f) << 2)
#define   UHSIC_ELASTIC_OVERRUN_LIMIT(x)	(((x) & 0x1f) << 8)
#define   UHSIC_IDLE_WAIT(x)			(((x) & 0x1f) << 13)

#define UHSIC_HSRX_CFG1				0xc0c
#define   UHSIC_HS_SYNC_START_DLY(x)		(((x) & 0x1f) << 1)

#define UHSIC_MISC_CFG0				0xc14
#define   UHSIC_SUSPEND_EXIT_ON_EDGE		(1 << 7)
#define   UHSIC_DETECT_SHORT_CONNECT		(1 << 8)
#define   UHSIC_FORCE_XCVR_MODE			(1 << 15)

#define UHSIC_MISC_CFG1				0xc18
#define   UHSIC_PLLU_STABLE_COUNT(x)		(((x) & 0xfff) << 2)

#define UHSIC_PADS_CFG0				0xc1c
#define   UHSIC_TX_RTUNEN			0xf000
#define   UHSIC_TX_RTUNE(x)			(((x) & 0xf) << 12)

#define UHSIC_PADS_CFG1				0xc20
#define   UHSIC_PD_BG				(1 << 2)
#define   UHSIC_PD_TX				(1 << 3)
#define   UHSIC_PD_TRK				(1 << 4)
#define   UHSIC_PD_RX				(1 << 5)
#define   UHSIC_PD_ZI				(1 << 6)
#define   UHSIC_RX_SEL				(1 << 7)
#define   UHSIC_RPD_DATA			(1 << 9)
#define   UHSIC_RPD_STROBE			(1 << 10)
#define   UHSIC_RPU_DATA			(1 << 11)
#define   UHSIC_RPU_STROBE			(1 << 12)

#define UHSIC_STAT_CFG0				0xc28
#define   UHSIC_CONNECT_DETECT			(1 << 0)
#endif

/* Common registers */

#define ULPIS2S_CTRL		0x418
#define   ULPIS2S_ENA			(1 << 0)
#define   ULPIS2S_SUPPORT_DISCONNECT	(1 << 2)
#define   ULPIS2S_PLLU_MASTER_BLASTER60	(1 << 3)
#define   ULPIS2S_SPARE(x)		(((x) & 0xF) << 8)
#define   ULPIS2S_FORCE_ULPI_CLK_OUT	(1 << 12)
#define   ULPIS2S_DISCON_DONT_CHECK_SE0	(1 << 13)
#define   ULPIS2S_SUPPORT_HS_KEEP_ALIVE (1 << 14)
#define   ULPIS2S_DISABLE_STP_PU	(1 << 15)
#define   ULPIS2S_SLV0_CLAMP_XMIT	(1 << 16)


#define ULPI_TIMING_CTRL_0	0x424
#define   ULPI_CLOCK_OUT_DELAY(x)	((x) & 0x1F)
#define   ULPI_OUTPUT_PINMUX_BYP	(1 << 10)
#define   ULPI_CLKOUT_PINMUX_BYP	(1 << 11)
#define   ULPI_SHADOW_CLK_LOOPBACK_EN	(1 << 12)
#define   ULPI_SHADOW_CLK_SEL		(1 << 13)
#define   ULPI_CORE_CLK_SEL		(1 << 14)
#define   ULPI_SHADOW_CLK_DELAY(x)	(((x) & 0x1F) << 16)
#define   ULPI_LBK_PAD_EN		(1 << 26)
#define   ULPI_LBK_PAD_E_INPUT_OR	(1 << 27)
#define   ULPI_CLK_OUT_ENA		(1 << 28)
#define   ULPI_CLK_PADOUT_ENA		(1 << 29)

#define ULPI_TIMING_CTRL_1	0x428
#define   ULPI_DATA_TRIMMER_LOAD	(1 << 0)
#define   ULPI_DATA_TRIMMER_SEL(x)	(((x) & 0x7) << 1)
#define   ULPI_STPDIRNXT_TRIMMER_LOAD	(1 << 16)
#define   ULPI_STPDIRNXT_TRIMMER_SEL(x)	(((x) & 0x7) << 17)
#define   ULPI_DIR_TRIMMER_LOAD		(1 << 24)
#define   ULPI_DIR_TRIMMER_SEL(x)	(((x) & 0x7) << 25)



#define UTMIP_SPARE_CFG0	0x834
#define   FUSE_SETUP_SEL		(1 << 3)

#define UHSIC_PLL_CFG0				0x800

#define UHSIC_TX_CFG0				0x810
#define   UHSIC_HS_POSTAMBLE_OUTPUT_ENABLE	(1 << 6)

#define UHSIC_CMD_CFG0				0x824
#define   UHSIC_PRETEND_CONNECT_DETECT		(1 << 5)

#define UHSIC_SPARE_CFG0			0x82c



static DEFINE_SPINLOCK(utmip_pad_lock);
static int utmip_pad_count;

struct tegra_xtal_freq {
	int freq;
	u8 enable_delay;
	u8 stable_count;
	u8 active_delay;
	u16 xtal_freq_count;
	u16 debounce;
	u8 pdtrk_count;
};

static const struct tegra_xtal_freq tegra_freq_table[] = {
	{
		.freq = 12000000,
		.enable_delay = 0x02,
		.stable_count = 0x2F,
		.active_delay = 0x04,
		.xtal_freq_count = 0x76,
		.debounce = 0x7530,
		.pdtrk_count = 5,
	},
	{
		.freq = 13000000,
		.enable_delay = 0x02,
		.stable_count = 0x33,
		.active_delay = 0x05,
		.xtal_freq_count = 0x7F,
		.debounce = 0x7EF4,
		.pdtrk_count = 5,
	},
	{
		.freq = 19200000,
		.enable_delay = 0x03,
		.stable_count = 0x4B,
		.active_delay = 0x06,
		.xtal_freq_count = 0xBB,
		.debounce = 0xBB80,
		.pdtrk_count = 7,
	},
	{
		.freq = 26000000,
		.enable_delay = 0x04,
		.stable_count = 0x66,
		.active_delay = 0x09,
		.xtal_freq_count = 0xFE,
		.debounce = 0xFDE8,
		.pdtrk_count = 9,
	},
};

static const struct tegra_xtal_freq tegra_uhsic_freq_table[] = {
	{
		.freq = 12000000,
		.enable_delay = 0x02,
		.stable_count = 0x2F,
		.active_delay = 0x0,
		.xtal_freq_count = 0x1CA,
	},
	{
		.freq = 13000000,
		.enable_delay = 0x02,
		.stable_count = 0x33,
		.active_delay = 0x0,
		.xtal_freq_count = 0x1F0,
	},
	{
		.freq = 19200000,
		.enable_delay = 0x03,
		.stable_count = 0x4B,
		.active_delay = 0x0,
		.xtal_freq_count = 0x2DD,
	},
	{
		.freq = 26000000,
		.enable_delay = 0x04,
		.stable_count = 0x66,
		.active_delay = 0x0,
		.xtal_freq_count = 0x3E0,
	},
};

static struct tegra_utmip_config utmip_default[] = {
	[0] = {
		.hssync_start_delay = 9,
		.idle_wait_delay = 17,
		.elastic_limit = 16,
		.term_range_adj = 6,
		.xcvr_setup = 9,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
	},
	[2] = {
		.hssync_start_delay = 9,
		.idle_wait_delay = 17,
		.elastic_limit = 16,
		.term_range_adj = 6,
		.xcvr_setup = 9,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
	},
};

static struct tegra_uhsic_config uhsic_default = {
	.sync_start_delay = 9,
	.idle_wait_delay = 17,
	.term_range_adj = 0,
	.elastic_underrun_limit = 16,
	.elastic_overrun_limit = 16,
};

struct usb_phy_plat_data usb_phy_data[] = {
	{ 0, 0, -1, NULL},
	{ 0, 0, -1, NULL},
	{ 0, 0, -1, NULL},
};

static int utmip_pad_open(struct tegra_usb_phy *phy)
{
	phy->pad_clk = clk_get_sys("utmip-pad", NULL);
	if (IS_ERR(phy->pad_clk)) {
		pr_err("%s: can't get utmip pad clock\n", __func__);
		return PTR_ERR(phy->pad_clk);
	}

	if (phy->instance == 0) {
		phy->pad_regs = phy->regs;
	} else {
		phy->pad_regs = ioremap(TEGRA_USB_BASE, TEGRA_USB_SIZE);
		if (!phy->pad_regs) {
			pr_err("%s: can't remap usb registers\n", __func__);
			clk_put(phy->pad_clk);
			return -ENOMEM;
		}
	}
	return 0;
}

static void utmip_pad_close(struct tegra_usb_phy *phy)
{
	if (phy->instance != 0)
		iounmap(phy->pad_regs);
	clk_put(phy->pad_clk);
}

static int utmip_pad_power_on(struct tegra_usb_phy *phy)
{
	unsigned long val, flags;
	void __iomem *base = phy->pad_regs;

	clk_enable(phy->pad_clk);

	spin_lock_irqsave(&utmip_pad_lock, flags);

	if (utmip_pad_count++ == 0) {
		val = readl(base + UTMIP_BIAS_CFG0);
		val &= ~(UTMIP_OTGPD | UTMIP_BIASPD);
		writel(val, base + UTMIP_BIAS_CFG0);
	}

	spin_unlock_irqrestore(&utmip_pad_lock, flags);

	clk_disable(phy->pad_clk);

	return 0;
}

static int utmip_pad_power_off(struct tegra_usb_phy *phy, bool is_dpd)
{
	unsigned long val, flags;
	void __iomem *base = phy->pad_regs;

	if (!utmip_pad_count) {
		pr_err("%s: utmip pad already powered off\n", __func__);
		return -EINVAL;
	}

	clk_enable(phy->pad_clk);

	spin_lock_irqsave(&utmip_pad_lock, flags);

	if (--utmip_pad_count == 0 && is_dpd) {
		val = readl(base + UTMIP_BIAS_CFG0);
		val |= UTMIP_OTGPD | UTMIP_BIASPD;
		writel(val, base + UTMIP_BIAS_CFG0);
	}

	spin_unlock_irqrestore(&utmip_pad_lock, flags);

	clk_disable(phy->pad_clk);

	return 0;
}

static int utmi_wait_register(void __iomem *reg, u32 mask, u32 result)
{
	unsigned long timeout = 2500;
	do {
		if ((readl(reg) & mask) == result)
			return 0;
		udelay(1);
		timeout--;
	} while (timeout);
	return -1;
}

static void utmi_phy_clk_disable(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	if (phy->instance == 0) {
		val = readl(base + USB_SUSP_CTRL);
		val |= USB_SUSP_SET;
		writel(val, base + USB_SUSP_CTRL);

		udelay(10);

		val = readl(base + USB_SUSP_CTRL);
		val &= ~USB_SUSP_SET;
		writel(val, base + USB_SUSP_CTRL);
	}

	if (phy->instance == 2) {
		val = readl(base + USB_PORTSC1);
		val |= USB_PORTSC1_PHCD;
		writel(val, base + USB_PORTSC1);
	}
#else
	val = readl(base + HOSTPC1_DEVLC);
	val |= HOSTPC1_DEVLC_PHCD;
	writel(val, base + HOSTPC1_DEVLC);
#endif

	if (utmi_wait_register(base + USB_SUSP_CTRL, USB_PHY_CLK_VALID, 0) < 0)
		pr_err("%s: timeout waiting for phy to stabilize\n", __func__);
}

static void utmi_phy_clk_enable(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	if (phy->instance == 0) {
		val = readl(base + USB_SUSP_CTRL);
		val |= USB_SUSP_CLR;
		writel(val, base + USB_SUSP_CTRL);

		udelay(10);

		val = readl(base + USB_SUSP_CTRL);
		val &= ~USB_SUSP_CLR;
		writel(val, base + USB_SUSP_CTRL);
	}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	if (phy->instance == 2) {
		val = readl(base + USB_PORTSC1);
		val &= ~USB_PORTSC1_PHCD;
		writel(val, base + USB_PORTSC1);
	}
#endif

	if (utmi_wait_register(base + USB_SUSP_CTRL, USB_PHY_CLK_VALID,
						     USB_PHY_CLK_VALID) < 0)
		pr_err("%s: timeout waiting for phy to stabilize\n", __func__);
}

static void vbus_enable(struct tegra_usb_phy *phy)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	int gpio_status;
	int gpio = usb_phy_data[phy->instance].vbus_gpio;

	if (gpio == -1)
		return;

	gpio_status = gpio_request(gpio,"VBUS_USB");
	if (gpio_status < 0) {
		printk("VBUS_USB request GPIO FAILED\n");
		WARN_ON(1);
		return;
	}
	tegra_gpio_enable(gpio);
	gpio_status = gpio_direction_output(gpio, 1);
	if (gpio_status < 0) {
		printk("VBUS_USB request GPIO DIRECTION FAILED \n");
		WARN_ON(1);
		return;
	}
	gpio_set_value(gpio, 1);
#else
	if (phy->reg_vbus)
		regulator_enable(phy->reg_vbus);
#endif
}

static void vbus_disable(struct tegra_usb_phy *phy)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	int gpio = usb_phy_data[phy->instance].vbus_gpio;

	if (gpio == -1)
		return;

	gpio_set_value(gpio, 0);
	gpio_free(gpio);
#else
	if (phy->reg_vbus)
		regulator_disable(phy->reg_vbus);
#endif
}

static int utmi_phy_power_on(struct tegra_usb_phy *phy, bool is_dpd)
{
	unsigned long val;
	void __iomem *base = phy->regs;
	struct tegra_utmip_config *config = phy->config;

	val = readl(base + USB_SUSP_CTRL);
	val |= UTMIP_RESET;
	writel(val, base + USB_SUSP_CTRL);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	if (phy->instance == 0) {
		val = readl(base + USB1_LEGACY_CTRL);
		val |= USB1_NO_LEGACY_MODE;
		writel(val, base + USB1_LEGACY_CTRL);
	}
#endif

	val = readl(base + UTMIP_TX_CFG0);
	val |= UTMIP_FS_PREABMLE_J;
	writel(val, base + UTMIP_TX_CFG0);

	val = readl(base + UTMIP_HSRX_CFG0);
	val &= ~(UTMIP_IDLE_WAIT(~0) | UTMIP_ELASTIC_LIMIT(~0));
	val |= UTMIP_IDLE_WAIT(config->idle_wait_delay);
	val |= UTMIP_ELASTIC_LIMIT(config->elastic_limit);
	writel(val, base + UTMIP_HSRX_CFG0);

	val = readl(base + UTMIP_HSRX_CFG1);
	val &= ~UTMIP_HS_SYNC_START_DLY(~0);
	val |= UTMIP_HS_SYNC_START_DLY(config->hssync_start_delay);
	writel(val, base + UTMIP_HSRX_CFG1);

	val = readl(base + UTMIP_DEBOUNCE_CFG0);
	val &= ~UTMIP_BIAS_DEBOUNCE_A(~0);
	val |= UTMIP_BIAS_DEBOUNCE_A(phy->freq->debounce);
	writel(val, base + UTMIP_DEBOUNCE_CFG0);

	val = readl(base + UTMIP_MISC_CFG0);
	val &= ~UTMIP_SUSPEND_EXIT_ON_EDGE;
	writel(val, base + UTMIP_MISC_CFG0);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	val = readl(base + UTMIP_MISC_CFG1);
	val &= ~(UTMIP_PLL_ACTIVE_DLY_COUNT(~0) | UTMIP_PLLU_STABLE_COUNT(~0));
	val |= UTMIP_PLL_ACTIVE_DLY_COUNT(phy->freq->active_delay) |
		UTMIP_PLLU_STABLE_COUNT(phy->freq->stable_count);
	writel(val, base + UTMIP_MISC_CFG1);

	val = readl(base + UTMIP_PLL_CFG1);
	val &= ~(UTMIP_XTAL_FREQ_COUNT(~0) | UTMIP_PLLU_ENABLE_DLY_COUNT(~0));
	val |= UTMIP_XTAL_FREQ_COUNT(phy->freq->xtal_freq_count) |
		UTMIP_PLLU_ENABLE_DLY_COUNT(phy->freq->enable_delay);
	writel(val, base + UTMIP_PLL_CFG1);
#endif

	if (phy->mode == TEGRA_USB_PHY_MODE_DEVICE) {
		val = readl(base + USB_SUSP_CTRL);
		val &= ~(USB_WAKE_ON_CNNT_EN_DEV | USB_WAKE_ON_DISCON_EN_DEV);
		writel(val, base + USB_SUSP_CTRL);
	}

	utmip_pad_power_on(phy);

	val = readl(base + UTMIP_XCVR_CFG0);
	val &= ~(UTMIP_XCVR_LSBIAS_SEL | UTMIP_FORCE_PD_POWERDOWN |
		 UTMIP_FORCE_PD2_POWERDOWN | UTMIP_FORCE_PDZI_POWERDOWN |
		 UTMIP_XCVR_SETUP(~0) | UTMIP_XCVR_LSFSLEW(~0) |
		 UTMIP_XCVR_LSRSLEW(~0) | UTMIP_XCVR_HSSLEW_MSB(~0));
	val |= UTMIP_XCVR_SETUP(config->xcvr_setup);
	val |= UTMIP_XCVR_LSFSLEW(config->xcvr_lsfslew);
	val |= UTMIP_XCVR_LSRSLEW(config->xcvr_lsrslew);
	writel(val, base + UTMIP_XCVR_CFG0);

	val = readl(base + UTMIP_XCVR_CFG1);
	val &= ~(UTMIP_FORCE_PDDISC_POWERDOWN | UTMIP_FORCE_PDCHRP_POWERDOWN |
		 UTMIP_FORCE_PDDR_POWERDOWN | UTMIP_XCVR_TERM_RANGE_ADJ(~0));
	val |= UTMIP_XCVR_TERM_RANGE_ADJ(config->term_range_adj);
	writel(val, base + UTMIP_XCVR_CFG1);

	val = readl(base + UTMIP_BAT_CHRG_CFG0);
	if (phy->mode == TEGRA_USB_PHY_MODE_HOST)
		val |= UTMIP_PD_CHRG;
	else
		val &= ~UTMIP_PD_CHRG;
	writel(val, base + UTMIP_BAT_CHRG_CFG0);

	val = readl(base + UTMIP_BIAS_CFG1);
	val &= ~UTMIP_BIAS_PDTRK_COUNT(~0);
	val |= UTMIP_BIAS_PDTRK_COUNT(phy->freq->pdtrk_count);
	writel(val, base + UTMIP_BIAS_CFG1);

	if (phy->instance == 0) {
		val = readl(base + UTMIP_SPARE_CFG0);
		if (phy->mode == TEGRA_USB_PHY_MODE_DEVICE)
			val &= ~FUSE_SETUP_SEL;
		else
			val |= FUSE_SETUP_SEL;
		writel(val, base + UTMIP_SPARE_CFG0);
	}
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	if (phy->instance == 2) {
		val = readl(base + UTMIP_SPARE_CFG0);
		val |= FUSE_SETUP_SEL;
		writel(val, base + UTMIP_SPARE_CFG0);

		val = readl(base + USB_SUSP_CTRL);
		val |= UTMIP_PHY_ENABLE;
		writel(val, base + USB_SUSP_CTRL);
	}
#else
	if ((phy->instance == 1) || (phy->instance == 2)) {
		val = readl(base + USB_SUSP_CTRL);
		val |= UTMIP_PHY_ENABLE;
		writel(val, base + USB_SUSP_CTRL);
	}
#endif

	val = readl(base + USB_SUSP_CTRL);
	val &= ~UTMIP_RESET;
	writel(val, base + USB_SUSP_CTRL);

	if (phy->instance == 0) {
		val = readl(base + USB1_LEGACY_CTRL);
		val &= ~USB1_VBUS_SENSE_CTL_MASK;
		val |= USB1_VBUS_SENSE_CTL_A_SESS_VLD;
		writel(val, base + USB1_LEGACY_CTRL);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		val = readl(base + USB_SUSP_CTRL);
		val &= ~USB_SUSP_SET;
		writel(val, base + USB_SUSP_CTRL);
#endif
	}

	utmi_phy_clk_enable(phy);
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	if (phy->instance == 2) {
		val = readl(base + USB_PORTSC1);
		val &= ~USB_PORTSC1_PTS(~0);
		writel(val, base + USB_PORTSC1);
	}
#else
	if(phy->instance == 2) {
		writel(0, base + ICUSB_CTRL);

		val = readl(base + TEGRA_USB_USBMODE_REG_OFFSET);
		writel((val | TEGRA_USB_USBMODE_HOST),
			(base + TEGRA_USB_USBMODE_REG_OFFSET));
	}
	val = readl(base + HOSTPC1_DEVLC);
	val &= ~HOSTPC1_DEVLC_PTS(~0);
	val |= HOSTPC1_DEVLC_STS;
	writel(val, base + HOSTPC1_DEVLC);
#endif

	return 0;
}

static int utmi_phy_power_off(struct tegra_usb_phy *phy, bool is_dpd)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	if (phy->mode == TEGRA_USB_PHY_MODE_DEVICE) {
		val = readl(base + USB_SUSP_CTRL);
		val &= ~USB_WAKEUP_DEBOUNCE_COUNT(~0);
		val |= USB_WAKE_ON_CNNT_EN_DEV | USB_WAKEUP_DEBOUNCE_COUNT(5);
		writel(val, base + USB_SUSP_CTRL);
	}

	if (phy->mode == TEGRA_USB_PHY_MODE_DEVICE) {
		val = readl(base + UTMIP_BAT_CHRG_CFG0);
		val |= UTMIP_PD_CHRG;
		writel(val, base + UTMIP_BAT_CHRG_CFG0);
	}

	val = readl(base + UTMIP_XCVR_CFG1);
	val |= UTMIP_FORCE_PDDISC_POWERDOWN | UTMIP_FORCE_PDCHRP_POWERDOWN |
	       UTMIP_FORCE_PDDR_POWERDOWN;
	writel(val, base + UTMIP_XCVR_CFG1);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	val = readl(base + UTMIP_BIAS_CFG1);
	val |= UTMIP_BIAS_PDTRK_COUNT(0x5);
	writel(val, base + UTMIP_BIAS_CFG1);
#endif

	if (phy->instance == 2) {
		val = readl(base + USB_PORTSC1);
		val |= USB_PORTSC1_WKCN;
		writel(val, base + USB_PORTSC1);
	}

	utmi_phy_clk_disable(phy);

	if (phy->instance == 2) {
		val = readl(base + USB_SUSP_CTRL);
		val |= USB_PHY_CLK_VALID_INT_ENB;
		writel(val, base + USB_SUSP_CTRL);
	}
	utmip_pad_power_off(phy, is_dpd);
	return 0;
}

static int utmi_phy_preresume(struct tegra_usb_phy *phy, bool is_dpd)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + UTMIP_TX_CFG0);
	val |= UTMIP_HS_DISCON_DISABLE;
	writel(val, base + UTMIP_TX_CFG0);

	return 0;
}

static int utmi_phy_postresume(struct tegra_usb_phy *phy, bool is_dpd)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + UTMIP_TX_CFG0);
	val &= ~UTMIP_HS_DISCON_DISABLE;
	writel(val, base + UTMIP_TX_CFG0);

	return 0;
}

static int uhsic_phy_postresume(struct tegra_usb_phy *phy, bool is_dpd)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + USB_TXFILLTUNING);
	if ((val & USB_FIFO_TXFILL_MASK) != USB_FIFO_TXFILL_THRES(0x10)) {
		val = USB_FIFO_TXFILL_THRES(0x10);
		writel(val, base + USB_TXFILLTUNING);
	}
#endif

	return 0;
}

static void utmi_phy_restore_start(struct tegra_usb_phy *phy,
				   enum tegra_usb_phy_port_speed port_speed)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + UTMIP_MISC_CFG0);
	val &= ~UTMIP_DPDM_OBSERVE_SEL(~0);
	if (port_speed == TEGRA_USB_PHY_PORT_SPEED_LOW)
		val |= UTMIP_DPDM_OBSERVE_SEL_FS_K;
	else
		val |= UTMIP_DPDM_OBSERVE_SEL_FS_J;
	writel(val, base + UTMIP_MISC_CFG0);
	udelay(1);

	val = readl(base + UTMIP_MISC_CFG0);
	val |= UTMIP_DPDM_OBSERVE;
	writel(val, base + UTMIP_MISC_CFG0);
	udelay(10);
#endif
}

static void utmi_phy_restore_end(struct tegra_usb_phy *phy)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + UTMIP_MISC_CFG0);
	val &= ~UTMIP_DPDM_OBSERVE;
	writel(val, base + UTMIP_MISC_CFG0);
	udelay(10);
#endif
}

static void ulpi_set_tristate(bool enable)
{
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	int tristate = (enable)? TEGRA_TRI_TRISTATE : TEGRA_TRI_NORMAL;

	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA0, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA1, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA2, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA3, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA4, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA5, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA6, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA7, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_CLK, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DIR, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_STP, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_NXT, tristate);
#endif
}

static void ulpi_phy_reset(void __iomem *base)
{
	unsigned long val;

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_CLK, TEGRA_TRI_TRISTATE);
#endif
	val = readl(base + USB_SUSP_CTRL);
	val |= UHSIC_RESET;
	writel(val, base + USB_SUSP_CTRL);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	val = readl(base + USB_SUSP_CTRL);
	val |= UTMIP_RESET;
	writel(val, base + USB_SUSP_CTRL);
#endif
}

static void ulpi_set_host(void __iomem *base)
{
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	unsigned long val;

	val = readl(base + TEGRA_USB_USBMODE_REG_OFFSET);
	val |= TEGRA_USB_USBMODE_HOST;
	writel(val, base + TEGRA_USB_USBMODE_REG_OFFSET);

	val = readl(base + HOSTPC1_DEVLC);
	val |= HOSTPC1_DEVLC_PTS(2);
	writel(val, base + HOSTPC1_DEVLC);
#endif
}

static void ulpi_set_trimmer(void __iomem *base, u8 data, u8 sdn, u8 dir)
{
	unsigned long val;

	val = ULPI_DATA_TRIMMER_SEL(data);
	val |= ULPI_STPDIRNXT_TRIMMER_SEL(sdn);
	val |= ULPI_DIR_TRIMMER_SEL(dir);
	writel(val, base + ULPI_TIMING_CTRL_1);
	udelay(10);

	val |= ULPI_DATA_TRIMMER_LOAD;
	val |= ULPI_STPDIRNXT_TRIMMER_LOAD;
	val |= ULPI_DIR_TRIMMER_LOAD;
	writel(val, base + ULPI_TIMING_CTRL_1);
}

static void ulpi_phy_restore_start(struct tegra_usb_phy *phy,
				   enum tegra_usb_phy_port_speed port_speed)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	unsigned long val;
	void __iomem *base = phy->regs;

	/*Tristate ulpi interface before USB controller resume*/
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_UAA, TEGRA_TRI_TRISTATE);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_UAB, TEGRA_TRI_TRISTATE);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_UDA, TEGRA_TRI_TRISTATE);

	val = readl(base + ULPI_TIMING_CTRL_0);
	val &= ~ULPI_OUTPUT_PINMUX_BYP;
	writel(val, base + ULPI_TIMING_CTRL_0);
#endif
}

static void ulpi_phy_restore_end(struct tegra_usb_phy *phy)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_OUTPUT_PINMUX_BYP;
	writel(val, base + ULPI_TIMING_CTRL_0);

	tegra_pinmux_set_tristate(TEGRA_PINGROUP_UAA, TEGRA_TRI_NORMAL);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_UAB, TEGRA_TRI_NORMAL);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_UDA, TEGRA_TRI_NORMAL);
#endif
}

static int ulpi_phy_power_on(struct tegra_usb_phy *phy, bool is_dpd)
{
	int ret;
	unsigned long val;
	void __iomem *base = phy->regs;
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	struct tegra_ulpi_config *config = phy->config;
#endif

	clk_enable(phy->clk);
	msleep(1);

	if (!phy->initialized) {
		phy->initialized = 1;
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		gpio_direction_output(config->reset_gpio, 0);
		msleep(5);
		gpio_direction_output(config->reset_gpio, 1);
#endif
	}

	ulpi_phy_reset(base);
	ulpi_set_host(base);

	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_OUTPUT_PINMUX_BYP | ULPI_CLKOUT_PINMUX_BYP;
	writel(val, base + ULPI_TIMING_CTRL_0);

	val = readl(base + USB_SUSP_CTRL);
	val |= ULPI_PHY_ENABLE;
	writel(val, base + USB_SUSP_CTRL);

	val = readl(base + USB_SUSP_CTRL);
	val |= USB_SUSP_CLR;
	writel(val, base + USB_SUSP_CTRL);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	if (utmi_wait_register(base + USB_SUSP_CTRL, USB_PHY_CLK_VALID,
						     USB_PHY_CLK_VALID) < 0)
		pr_err("%s: timeout waiting for phy to stabilize\n", __func__);

	if (utmi_wait_register(base + USB_SUSP_CTRL, USB_CLKEN, USB_CLKEN) < 0)
		pr_err("%s: timeout waiting for AHB clock\n", __func__);
#else
	udelay(100);
#endif

	val = readl(base + USB_SUSP_CTRL);
	val &= ~USB_SUSP_CLR;
	writel(val, base + USB_SUSP_CTRL);

	val = 0;
	writel(val, base + ULPI_TIMING_CTRL_1);

	ulpi_set_trimmer(base, 4, 4, 4);

	/* Fix VbusInvalid due to floating VBUS */
	ret = otg_io_write(phy->ulpi, 0x40, 0x08);
	if (ret) {
		pr_err("%s: ulpi write failed\n", __func__);
		return ret;
	}

	ret = otg_io_write(phy->ulpi, 0x80, 0x0B);
	if (ret) {
		pr_err("%s: ulpi write failed\n", __func__);
		return ret;
	}

	val = readl(base + USB_PORTSC1);
	val |= USB_PORTSC1_WKOC | USB_PORTSC1_WKDS | USB_PORTSC1_WKCN;
	writel(val, base + USB_PORTSC1);

	return 0;
}

static int ulpi_phy_power_off(struct tegra_usb_phy *phy, bool is_dpd)
{
	unsigned long val;
	void __iomem *base = phy->regs;
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	int ret;

	/* Disable VbusValid, SessEnd comparators */
	ret = otg_io_write(phy->ulpi, 0x00, 0x0D);
	if (ret)
		pr_err("%s: ulpi write 0x0D failed\n", __func__);

	ret = otg_io_write(phy->ulpi, 0x00, 0x10);
	if (ret)
		pr_err("%s: ulpi write 0x10 failed\n", __func__);

	/* Disable IdFloat comparator */
	ret = otg_io_write(phy->ulpi, 0x00, 0x19);
	if (ret)
		pr_err("%s: ulpi write 0x19 failed\n", __func__);

	ret = otg_io_write(phy->ulpi, 0x00, 0x1D);
	if (ret)
		pr_err("%s: ulpi write 0x1D failed\n", __func__);

	/* Clear WKCN/WKDS/WKOC wake-on events that can cause the USB
	 * Controller to immediately bring the ULPI PHY out of low power
	 */
	val = readl(base + USB_PORTSC1);
	val &= ~(USB_PORTSC1_WKOC | USB_PORTSC1_WKDS | USB_PORTSC1_WKCN);
	writel(val, base + USB_PORTSC1);

	/* Put the PHY in the low power mode */
	val = readl(base + USB_PORTSC1);
	val |= USB_PORTSC1_PHCD;
	writel(val, base + USB_PORTSC1);

	if (utmi_wait_register(base + USB_SUSP_CTRL, USB_PHY_CLK_VALID, 0) < 0)
		pr_err("%s: timeout waiting for phy to stop\n", __func__);
#else
	val = readl(base + HOSTPC1_DEVLC);
	val &= ~(HOSTPC1_DEVLC_PHCD);
	writel(val, base + HOSTPC1_DEVLC);
#endif

	clk_disable(phy->clk);

	return 0;
}

static int null_phy_power_on(struct tegra_usb_phy *phy, bool is_dpd)
{
	const struct tegra_ulpi_trimmer default_trimmer = {0, 0, 4, 4};
	unsigned long val;
	void __iomem *base = phy->regs;
	struct tegra_ulpi_config *config = phy->config;

	if (!config->trimmer)
		config->trimmer = &default_trimmer;

	ulpi_phy_reset(base);
	ulpi_set_host(base);

	if (config->preinit)
		config->preinit();

	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_OUTPUT_PINMUX_BYP | ULPI_CLKOUT_PINMUX_BYP;
	writel(val, base + ULPI_TIMING_CTRL_0);

	val = readl(base + USB_SUSP_CTRL);
	val |= ULPI_PHY_ENABLE;
	writel(val, base + USB_SUSP_CTRL);
	udelay(10);

	/* set timming parameters */
	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_SHADOW_CLK_LOOPBACK_EN;
	val |= ULPI_SHADOW_CLK_SEL;
	val &= ~ULPI_LBK_PAD_EN;
	val |= ULPI_SHADOW_CLK_DELAY(config->trimmer->shadow_clk_delay);
	val |= ULPI_CLOCK_OUT_DELAY(config->trimmer->clock_out_delay);
	val |= ULPI_LBK_PAD_E_INPUT_OR;
	writel(val, base + ULPI_TIMING_CTRL_0);

	writel(0, base + ULPI_TIMING_CTRL_1);
	udelay(10);

	/* start internal 60MHz clock */
	val = ULPIS2S_PLLU_MASTER_BLASTER60;
	writel(val, base + ULPIS2S_CTRL);

	/* select ULPI_CORE_CLK_SEL to SHADOW_CLK */
	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_CORE_CLK_SEL;
	writel(val, base + ULPI_TIMING_CTRL_0);
	udelay(10);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* remove slave0 reset */
	val = readl(base + USB_SUSP_CTRL);
	val &= ~ULPIS2S_SLV0_RESET;
	writel(val, base + USB_SUSP_CTRL);
	udelay(10);
#endif
	/* enable ULPI null phy clock - can't set the trimmers before this */
	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_CLK_OUT_ENA;
	writel(val, base + ULPI_TIMING_CTRL_0);
	udelay(10);

	/* set trimmers */
	ulpi_set_trimmer(base, config->trimmer->data_trimmer,
			 config->trimmer->stpdirnxt_trimmer, 1);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* remove various ULPIS2S resets */
	val = readl(base + USB_SUSP_CTRL);
	val &= ~ULPIS2S_LINE_RESET;
	val &= ~ULPIS2S_SLV1_RESET;
	val &= ~ULPI_PADS_RESET;
	val &= ~ULPI_PADS_CLKEN_RESET;
	writel(val, base + USB_SUSP_CTRL);
#endif

	val = readl(base + ULPIS2S_CTRL);
	val |= ULPIS2S_ENA;
	val |= ULPIS2S_SPARE((phy->mode == TEGRA_USB_PHY_MODE_HOST)? 3 : 1);
	writel(val, base + ULPIS2S_CTRL);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	val = readl(base + USB_SUSP_CTRL);
	val |= USB_SUSP_CLR;
	writel(val, base + USB_SUSP_CTRL);
	udelay(100);
	val = readl(base + USB_SUSP_CTRL);
	val &= ~USB_SUSP_CLR;
	writel(val, base + USB_SUSP_CTRL);
#endif
	if (utmi_wait_register(base + USB_SUSP_CTRL, USB_PHY_CLK_VALID,
						     USB_PHY_CLK_VALID)) {
		pr_err("%s: timeout waiting for phy to stabilize\n", __func__);
		return -ETIMEDOUT;
	}

	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_CLK_PADOUT_ENA;
	writel(val, base + ULPI_TIMING_CTRL_0);
	udelay(10);

	if (config->postinit)
		config->postinit();

	return 0;
}

static int null_phy_power_off(struct tegra_usb_phy *phy, bool is_dpd)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + ULPI_TIMING_CTRL_0);
	val &= ~ULPI_CLK_PADOUT_ENA;
	writel(val, base + ULPI_TIMING_CTRL_0);

	ulpi_set_tristate(true);
	return 0;
}

static int null_phy_post_usbcmd_reset(struct tegra_usb_phy *phy, bool is_dpd)
{
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + ULPIS2S_CTRL);
	val |=  ULPIS2S_SLV0_CLAMP_XMIT;
	writel(val, base + ULPIS2S_CTRL);

	val = readl(base + USB_SUSP_CTRL);
	val |= ULPIS2S_SLV0_RESET;
	writel(val, base + USB_SUSP_CTRL);
	udelay(10);

	/* remove slave0 reset */
	val = readl(base + USB_SUSP_CTRL);
	val &= ~ULPIS2S_SLV0_RESET;
	writel(val, base + USB_SUSP_CTRL);
	udelay(10);

	val = readl(base + ULPIS2S_CTRL);
	val &=  ~ULPIS2S_SLV0_CLAMP_XMIT;
	writel(val, base + ULPIS2S_CTRL);

	ulpi_set_host(base);
#endif
	return 0;
}

static int uhsic_phy_power_on(struct tegra_usb_phy *phy, bool is_dpd)
{
	unsigned long val;
	void __iomem *base = phy->regs;
	struct tegra_uhsic_config *config = &uhsic_default;
	struct tegra_ulpi_config *ulpi_config = phy->config;

	if (ulpi_config->preinit)
		ulpi_config->preinit();

	val = readl(base + UHSIC_PADS_CFG1);
	val &= ~(UHSIC_PD_BG | UHSIC_PD_TX | UHSIC_PD_TRK | UHSIC_PD_RX |
			UHSIC_PD_ZI | UHSIC_RPD_DATA | UHSIC_RPD_STROBE);
	val |= UHSIC_RX_SEL;
	writel(val, base + UHSIC_PADS_CFG1);
	udelay(2);

	val = readl(base + USB_SUSP_CTRL);
	val |= UHSIC_RESET;
	writel(val, base + USB_SUSP_CTRL);
	udelay(30);

	val = readl(base + USB_SUSP_CTRL);
	val |= UHSIC_PHY_ENABLE;
	writel(val, base + USB_SUSP_CTRL);

	val = readl(base + UHSIC_HSRX_CFG0);
	val |= UHSIC_IDLE_WAIT(config->idle_wait_delay);
	val |= UHSIC_ELASTIC_UNDERRUN_LIMIT(config->elastic_underrun_limit);
	val |= UHSIC_ELASTIC_OVERRUN_LIMIT(config->elastic_overrun_limit);
	writel(val, base + UHSIC_HSRX_CFG0);

	val = readl(base + UHSIC_HSRX_CFG1);
	val |= UHSIC_HS_SYNC_START_DLY(config->sync_start_delay);
	writel(val, base + UHSIC_HSRX_CFG1);

	val = readl(base + UHSIC_MISC_CFG0);
	val |= UHSIC_SUSPEND_EXIT_ON_EDGE;
	writel(val, base + UHSIC_MISC_CFG0);

	val = readl(base + UHSIC_MISC_CFG1);
	val |= UHSIC_PLLU_STABLE_COUNT(phy->freq->stable_count);
	writel(val, base + UHSIC_MISC_CFG1);

	val = readl(base + UHSIC_PLL_CFG1);
	val |= UHSIC_PLLU_ENABLE_DLY_COUNT(phy->freq->enable_delay);
	val |= UHSIC_XTAL_FREQ_COUNT(phy->freq->xtal_freq_count);
	writel(val, base + UHSIC_PLL_CFG1);

	val = readl(base + USB_SUSP_CTRL);
	val &= ~(UHSIC_RESET);
	writel(val, base + USB_SUSP_CTRL);
	udelay(2);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	val = readl(base + USB_PORTSC1);
	val &= ~USB_PORTSC1_PTS(~0);
	writel(val, base + USB_PORTSC1);

	val = readl(base + USB_TXFILLTUNING);
	if ((val & USB_FIFO_TXFILL_MASK) != USB_FIFO_TXFILL_THRES(0x10)) {
		val = USB_FIFO_TXFILL_THRES(0x10);
		writel(val, base + USB_TXFILLTUNING);
	}
#endif

	val = readl(base + USB_PORTSC1);
	val &= ~(USB_PORTSC1_WKOC | USB_PORTSC1_WKDS | USB_PORTSC1_WKCN);
	writel(val, base + USB_PORTSC1);

	val = readl(base + UHSIC_PADS_CFG0);
	val &= ~(UHSIC_TX_RTUNEN);
	/* set Rtune impedance to 40 ohm */
	val |= UHSIC_TX_RTUNE(0);
	writel(val, base + UHSIC_PADS_CFG0);

	if (utmi_wait_register(base + USB_SUSP_CTRL, USB_PHY_CLK_VALID,
							USB_PHY_CLK_VALID)) {
		pr_err("%s: timeout waiting for phy to stabilize\n", __func__);
		return -ETIMEDOUT;
	}

	return 0;
}

static int uhsic_phy_power_off(struct tegra_usb_phy *phy, bool is_dpd)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	val = readl(base + UHSIC_PADS_CFG1);
	val &= ~UHSIC_RPU_STROBE;
	val |= UHSIC_RPD_STROBE;
	writel(val, base + UHSIC_PADS_CFG1);

	val = readl(base + USB_SUSP_CTRL);
	val |= UHSIC_RESET;
	writel(val, base + USB_SUSP_CTRL);
	udelay(30);

	val = readl(base + USB_SUSP_CTRL);
	val &= ~UHSIC_PHY_ENABLE;
	writel(val, base + USB_SUSP_CTRL);

	return 0;
}

#ifdef CONFIG_USB_TEGRA_OTG
extern void tegra_otg_check_vbus_detection(void);
#endif

static irqreturn_t usb_phy_vbus_irq_thr(int irq, void *pdata)
{
	struct tegra_usb_phy *phy = pdata;

	if (!phy->regulator_on) {
		regulator_enable(phy->reg_vdd);
		phy->regulator_on = 1;
		/*
		 * Optimal time to get the regulator turned on
		 * before detecting vbus interrupt.
		 */
		mdelay(15);
	}

#ifdef CONFIG_USB_TEGRA_OTG
	tegra_otg_check_vbus_detection();
#endif

	return IRQ_HANDLED;
}

struct tegra_usb_phy *tegra_usb_phy_open(int instance, void __iomem *regs,
			void *config, enum tegra_usb_phy_mode phy_mode,
			enum tegra_usb_phy_type usb_phy_type)
{
	struct tegra_usb_phy *phy;
	struct tegra_ulpi_config *ulpi_config;
	unsigned long parent_rate;
	int i;
	int err;

	phy = kzalloc(sizeof(struct tegra_usb_phy), GFP_KERNEL);
	if (!phy)
		return ERR_PTR(-ENOMEM);

	phy->instance = instance;
	phy->regs = regs;
	phy->config = config;
	phy->mode = phy_mode;
	phy->usb_phy_type = usb_phy_type;
	phy->initialized = 0;
	phy->regulator_on = 0;

	if (!phy->config) {
		if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_LINK_ULPI ||
		    phy->usb_phy_type == TEGRA_USB_PHY_TYPE_NULL_ULPI) {
			pr_err("%s: ulpi phy configuration missing", __func__);
			err = -EINVAL;
			goto err0;
		} else {
			phy->config = &utmip_default[instance];
		}
	}

	phy->pll_u = clk_get_sys(NULL, "pll_u");
	if (IS_ERR(phy->pll_u)) {
		pr_err("Can't get pll_u clock\n");
		err = PTR_ERR(phy->pll_u);
		goto err0;
	}
	clk_enable(phy->pll_u);

	parent_rate = clk_get_rate(clk_get_parent(phy->pll_u));
	if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_HSIC) {
		for (i = 0; i < ARRAY_SIZE(tegra_uhsic_freq_table); i++) {
			if (tegra_uhsic_freq_table[i].freq == parent_rate) {
				phy->freq = &tegra_uhsic_freq_table[i];
				break;
			}
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(tegra_freq_table); i++) {
			if (tegra_freq_table[i].freq == parent_rate) {
				phy->freq = &tegra_freq_table[i];
				break;
			}
		}
	}
	if (!phy->freq) {
		pr_err("invalid pll_u parent rate %ld\n", parent_rate);
		err = -EINVAL;
		goto err1;
	}

	if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP) {
		err = utmip_pad_open(phy);
		if (err < 0)
			goto err1;
	} else if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_LINK_ULPI) {
		ulpi_config = config;

		phy->clk = clk_get_sys(NULL, ulpi_config->clk);
		if (IS_ERR(phy->clk)) {
			pr_err("%s: can't get ulpi clock\n", __func__);
			err = -ENXIO;
			goto err1;
		}
		tegra_gpio_enable(ulpi_config->reset_gpio);
		gpio_request(ulpi_config->reset_gpio, "ulpi_phy_reset_b");
		gpio_direction_output(ulpi_config->reset_gpio, 0);
		phy->ulpi = otg_ulpi_create(&ulpi_viewport_access_ops, 0);
		phy->ulpi->io_priv = regs + ULPI_VIEWPORT;
	}
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	else if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_HSIC) {
		ulpi_config = config;
		gpio_request(ulpi_config->enable_gpio,
			"uhsic_enable");
		if (ulpi_config->reset_gpio != -1)
			gpio_request(ulpi_config->reset_gpio,
				"uhsic_reset");
		/* hsic enable signal deasserted, hsic reset asserted */
		gpio_direction_output(ulpi_config->enable_gpio,
			0 /* deasserted */);
		if (ulpi_config->reset_gpio != -1)
			gpio_direction_output(ulpi_config->reset_gpio,
				0 /* asserted */);
		tegra_gpio_enable(ulpi_config->enable_gpio);
		if (ulpi_config->reset_gpio != -1)
			tegra_gpio_enable(ulpi_config->reset_gpio);
		/* keep hsic reset asserted for 1 ms */
		udelay(1000);
		/* enable (power on) hsic */
		gpio_set_value_cansleep(ulpi_config->enable_gpio, 1);
		udelay(1000);
		/* deassert reset */
		if (ulpi_config->reset_gpio != -1)
			gpio_set_value_cansleep(ulpi_config->reset_gpio, 1);
	}
#endif

	phy->reg_vdd = regulator_get(NULL, "avdd_usb");
	if (WARN_ON(IS_ERR_OR_NULL(phy->reg_vdd))) {
		pr_err("couldn't get regulator avdd_usb: %ld \n",
			 PTR_ERR(phy->reg_vdd));
		err = PTR_ERR(phy->reg_vdd);
		goto err1;
	}

	if (instance == 0 && usb_phy_data[0].vbus_irq) {
		err = request_threaded_irq(usb_phy_data[0].vbus_irq, NULL, usb_phy_vbus_irq_thr, IRQF_SHARED,
			"usb_phy_vbus", phy);
		if (err) {
			pr_err("Failed to register IRQ\n");
			goto err1;
		}
	}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* Power-up the VBUS detector for UTMIP PHY */
	if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP) {
		writel(readl((IO_ADDRESS(TEGRA_PMC_BASE) + TEGRA_PMC_USB_AO)) &
			~(TEGRA_PMC_USB_AO_VBUS_WAKEUP_PD_P0 | TEGRA_PMC_USB_AO_ID_PD_P0),
			(IO_ADDRESS(TEGRA_PMC_BASE) + TEGRA_PMC_USB_AO));

		if (usb_phy_data[phy->instance].vbus_reg_supply) {
			phy->reg_vbus = regulator_get(NULL, usb_phy_data[phy->instance].vbus_reg_supply);
			if (WARN_ON(IS_ERR_OR_NULL(phy->reg_vbus))) {
				pr_err("couldn't get regulator vdd_vbus_usb: %ld, instance : %d\n",
					PTR_ERR(phy->reg_vbus), phy->instance);
				err = PTR_ERR(phy->reg_vbus);
				goto err1;
			}
		}
	}
#endif
	if (((instance == 2) || (instance == 0)) &&
		(phy->mode == TEGRA_USB_PHY_MODE_HOST)) {
			vbus_enable(phy);
	}
	return phy;

err1:
	clk_disable(phy->pll_u);
	clk_put(phy->pll_u);
err0:
	kfree(phy);
	return ERR_PTR(err);
}

int tegra_usb_phy_power_on(struct tegra_usb_phy *phy, bool is_dpd)
{
	int ret = 0;

	const tegra_phy_fp power_on[] = {
		utmi_phy_power_on,
		ulpi_phy_power_on,
		null_phy_power_on,
		uhsic_phy_power_on,
	};

	if (phy->reg_vdd && !phy->regulator_on) {
		regulator_enable(phy->reg_vdd);
		phy->regulator_on = 1;
	}

	if (power_on[phy->usb_phy_type])
		ret = power_on[phy->usb_phy_type](phy, is_dpd);

	return ret;
}

void tegra_usb_phy_power_off(struct tegra_usb_phy *phy, bool is_dpd)
{
	const tegra_phy_fp power_off[] = {
		utmi_phy_power_off,
		ulpi_phy_power_off,
		null_phy_power_off,
		uhsic_phy_power_off,
	};

	if (power_off[phy->usb_phy_type])
		power_off[phy->usb_phy_type](phy, is_dpd);

	if (phy->reg_vdd && phy->regulator_on && is_dpd) {
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		if (tegra_get_revision() >= TEGRA_REVISION_A03)
#endif
		regulator_disable(phy->reg_vdd);
		phy->regulator_on = 0;
	}
}

void tegra_usb_phy_preresume(struct tegra_usb_phy *phy, bool is_dpd)
{
	const tegra_phy_fp preresume[] = {
		utmi_phy_preresume,
		NULL,
		NULL,
		NULL,
	};

	if (preresume[phy->usb_phy_type])
		preresume[phy->usb_phy_type](phy, is_dpd);
}

void tegra_usb_phy_postresume(struct tegra_usb_phy *phy, bool is_dpd)
{
	const tegra_phy_fp postresume[] = {
		utmi_phy_postresume,
		NULL,
		NULL,
		uhsic_phy_postresume,
	};

	if (postresume[phy->usb_phy_type])
		postresume[phy->usb_phy_type](phy, is_dpd);
}

void tegra_ehci_post_reset(struct tegra_usb_phy *phy, bool is_dpd)
{
	const tegra_phy_fp post_reset[] = {
		NULL,
		NULL,
		null_phy_post_usbcmd_reset,
		NULL,
	};

	if (post_reset[phy->usb_phy_type])
		post_reset[phy->usb_phy_type](phy, is_dpd);
}

void tegra_ehci_phy_restore_start(struct tegra_usb_phy *phy,
				 enum tegra_usb_phy_port_speed port_speed)
{
	if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP)
		utmi_phy_restore_start(phy, port_speed);
	else
		ulpi_phy_restore_start(phy, port_speed);
}

void tegra_ehci_phy_restore_end(struct tegra_usb_phy *phy)
{
	if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP)
		utmi_phy_restore_end(phy);
	else
		ulpi_phy_restore_end(phy);
}

void tegra_usb_phy_clk_disable(struct tegra_usb_phy *phy)
{
	if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP)
		utmi_phy_clk_disable(phy);
}

void tegra_usb_phy_clk_enable(struct tegra_usb_phy *phy)
{
	if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP)
		utmi_phy_clk_enable(phy);
}

void tegra_usb_phy_close(struct tegra_usb_phy *phy)
{
	if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP)
		utmip_pad_close(phy);
	else if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_LINK_ULPI)
		clk_put(phy->clk);
	if (phy->mode == TEGRA_USB_PHY_MODE_HOST) {
		vbus_disable(phy);
	}
	clk_disable(phy->pll_u);
	clk_put(phy->pll_u);
	if (phy->reg_vbus)
		regulator_put(phy->reg_vbus);
	if (phy->reg_vdd)
		regulator_put(phy->reg_vdd);
	if (phy->instance == 0 && usb_phy_data[0].vbus_irq)
		free_irq(usb_phy_data[0].vbus_irq, phy);
	kfree(phy);
}

int tegra_usb_phy_bus_connect(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_HSIC) {
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		/* Change the USB controller PHY type to HSIC */
		val = readl(base + HOSTPC1_DEVLC);
		val &= ~HOSTPC1_DEVLC_PTS(HOSTPC1_DEVLC_PTS_MASK);
		val |= HOSTPC1_DEVLC_PTS(HOSTPC1_DEVLC_PTS_HSIC);
		val &= ~HOSTPC1_DEVLC_PSPD(HOSTPC1_DEVLC_PSPD_MASK);
		val |= HOSTPC1_DEVLC_PSPD(HOSTPC1_DEVLC_PSPD_HIGH_SPEED);
		writel(val, base + HOSTPC1_DEVLC);
#endif
		val = readl(base + UHSIC_MISC_CFG0);
		val |= UHSIC_DETECT_SHORT_CONNECT;
		writel(val, base + UHSIC_MISC_CFG0);
		udelay(1);

		val = readl(base + UHSIC_MISC_CFG0);
		val |= UHSIC_FORCE_XCVR_MODE;
		writel(val, base + UHSIC_MISC_CFG0);

		val = readl(base + UHSIC_PADS_CFG1);
		val &= ~UHSIC_RPD_STROBE;
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		val |= UHSIC_RPU_STROBE;
#endif
		writel(val, base + UHSIC_PADS_CFG1);

		if (utmi_wait_register(base + UHSIC_STAT_CFG0, UHSIC_CONNECT_DETECT, UHSIC_CONNECT_DETECT) < 0) {
			pr_err("%s: timeout waiting for hsic connect detect\n", __func__);
			return -ETIMEDOUT;
		}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		if (utmi_wait_register(base + USB_PORTSC1, USB_PORTSC1_LS(2), USB_PORTSC1_LS(2)) < 0) {
			pr_err("%s: timeout waiting for dplus state\n", __func__);
			return -ETIMEDOUT;
		}
#endif
	}

	return 0;
}

int tegra_usb_phy_bus_reset(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_HSIC) {
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		val = readl(base + USB_PORTSC1);
		val |= USB_PORTSC1_PTC(5);
		writel(val, base + USB_PORTSC1);
		udelay(2);

		val = readl(base + USB_PORTSC1);
		val &= ~USB_PORTSC1_PTC(~0);
		writel(val, base + USB_PORTSC1);
		udelay(2);
#endif

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		if (utmi_wait_register(base + USB_PORTSC1, USB_PORTSC1_LS(0), 0) < 0) {
			pr_err("%s: timeout waiting for SE0\n", __func__);
			return -ETIMEDOUT;
		}
#endif
		if (utmi_wait_register(base + USB_PORTSC1, USB_PORTSC1_CCS, USB_PORTSC1_CCS) < 0) {
			pr_err("%s: timeout waiting for connection status\n", __func__);
			return -ETIMEDOUT;
		}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		if (utmi_wait_register(base + USB_PORTSC1, USB_PORTSC1_PSPD(2), USB_PORTSC1_PSPD(2)) < 0) {
			pr_err("%s: timeout waiting hsic high speed configuration\n", __func__);
			return -ETIMEDOUT;
		}
#endif
		val = readl(base + USB_USBCMD);
		val &= ~USB_USBCMD_RS;
		writel(val, base + USB_USBCMD);

		if (utmi_wait_register(base + USB_USBSTS, USB_USBSTS_HCH, USB_USBSTS_HCH) < 0) {
			pr_err("%s: timeout waiting for stopping the controller\n", __func__);
			return -ETIMEDOUT;
		}

		val = readl(base + UHSIC_PADS_CFG1);
		val &= ~UHSIC_RPU_STROBE;
		val |= UHSIC_RPD_STROBE;
		writel(val, base + UHSIC_PADS_CFG1);

		mdelay(50);

		val = readl(base + UHSIC_PADS_CFG1);
		val &= ~UHSIC_RPD_STROBE;
		val |= UHSIC_RPU_STROBE;
		writel(val, base + UHSIC_PADS_CFG1);

		val = readl(base + USB_USBCMD);
		val |= USB_USBCMD_RS;
		writel(val, base + USB_USBCMD);

		val = readl(base + UHSIC_PADS_CFG1);
		val &= ~UHSIC_RPU_STROBE;
		writel(val, base + UHSIC_PADS_CFG1);

		if (utmi_wait_register(base + USB_USBCMD, USB_USBCMD_RS, USB_USBCMD_RS) < 0) {
			pr_err("%s: timeout waiting for starting the controller\n", __func__);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

int tegra_usb_phy_bus_idle(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;

	if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_HSIC) {
		val = readl(base + UHSIC_MISC_CFG0);
		val |= UHSIC_DETECT_SHORT_CONNECT;
		writel(val, base + UHSIC_MISC_CFG0);
		udelay(1);

		val = readl(base + UHSIC_MISC_CFG0);
		val |= UHSIC_FORCE_XCVR_MODE;
		writel(val, base + UHSIC_MISC_CFG0);

		val = readl(base + UHSIC_PADS_CFG1);
		val &= ~UHSIC_RPD_STROBE;
		val |= UHSIC_RPU_STROBE;
		writel(val, base + UHSIC_PADS_CFG1);
	}
	return 0;
}

bool tegra_usb_phy_is_device_connected(struct tegra_usb_phy *phy)
{
	void __iomem *base = phy->regs;

	if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_HSIC) {
		if (!((readl(base + UHSIC_STAT_CFG0) & UHSIC_CONNECT_DETECT) == UHSIC_CONNECT_DETECT)) {
			pr_err("%s: hsic no device connection\n", __func__);
			return false;
		}
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		if (utmi_wait_register(base + USB_PORTSC1, USB_PORTSC1_LS(2), USB_PORTSC1_LS(2)) < 0) {
			pr_err("%s: timeout waiting for dplus state\n", __func__);
			return false;
		}
#endif
	}
	return true;
}

int __init tegra_usb_phy_init(struct usb_phy_plat_data *pdata, int size)
{
	if (pdata) {
		int i;

		for (i = 0; i < size; i++, pdata++) {
			usb_phy_data[pdata->instance].instance = pdata->instance;
			usb_phy_data[pdata->instance].vbus_irq = pdata->vbus_irq;
			usb_phy_data[pdata->instance].vbus_gpio = pdata->vbus_gpio;
			usb_phy_data[pdata->instance].vbus_reg_supply = pdata->vbus_reg_supply;
		}
	}

	return 0;
}
