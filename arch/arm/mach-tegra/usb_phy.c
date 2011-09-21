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

#define USB_PHY_VBUS_WAKEUP_ID	0x408
#define   VDAT_DET_INT_EN	(1 << 16)
#define   VDAT_DET_CHG_DET	(1 << 17)
#define   VDAT_DET_STS		(1 << 18)

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
#define   UTMIP_XCVR_SETUP_MSB(x)		(((x) & 0x7) << 22)
#define   UTMIP_XCVR_HSSLEW_MSB(x)		(((x) & 0x7f) << 25)

#define UTMIP_XCVR_MAX_OFFSET		2
#define UTMIP_XCVR_SETUP_MAX_VALUE	0x7f
#define XCVR_SETUP_MSB_CALIB(x)	((x) >> 4)

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

#define UTMIP_MISC_CFG1		0x828
#define   UTMIP_PLL_ACTIVE_DLY_COUNT(x)	(((x) & 0x1f) << 18)
#define   UTMIP_PLLU_STABLE_COUNT(x)	(((x) & 0xfff) << 6)

#define UTMIP_DEBOUNCE_CFG0	0x82c
#define   UTMIP_BIAS_DEBOUNCE_A(x)	(((x) & 0xffff) << 0)

#define UTMIP_BAT_CHRG_CFG0	0x830
#define   UTMIP_PD_CHRG			(1 << 0)
#define   UTMIP_ON_SINK_EN		(1 << 2)
#define   UTMIP_OP_SRC_EN		(1 << 3)

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
#define   USB_USBSTS_SRI	(1 << 7)
#define   USB_USBSTS_HCH	(1 << 12)

#define ULPI_VIEWPORT		0x160

#define USB_PORTSC1		0x174
#define   USB_PORTSC1_WKOC	(1 << 22)
#define   USB_PORTSC1_WKDS	(1 << 21)
#define   USB_PORTSC1_WKCN	(1 << 20)
#define   USB_PORTSC1_PTC(x)	(((x) & 0xf) << 16)
#define   USB_PORTSC1_PP	(1 << 12)
#define   USB_PORTSC1_SUSP	(1 << 7)
#define   USB_PORTSC1_RESUME	(1 << 6)
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

#define USB_PHY_VBUS_WAKEUP_ID	0x408
#define   VDAT_DET_INT_EN	(1 << 16)
#define   VDAT_DET_CHG_DET	(1 << 17)
#define   VDAT_DET_STS		(1 << 18)

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
#define   UTMIP_XCVR_SETUP_MSB(x)		(((x) & 0x7) << 22)
#define   UTMIP_XCVR_HSSLEW_MSB(x)		(((x) & 0x7f) << 25)

#define UTMIP_XCVR_MAX_OFFSET		5
#define UTMIP_XCVR_SETUP_MAX_VALUE	0x7f
#define XCVR_SETUP_MSB_CALIB(x)	((x) >> 4)

#define UTMIP_BIAS_CFG0		0x80c
#define   UTMIP_OTGPD			(1 << 11)
#define   UTMIP_BIASPD			(1 << 10)
#define   UTMIP_HSSQUELCH_LEVEL(x)	(((x) & 0x3) << 0)
#define   UTMIP_HSDISCON_LEVEL(x)	(((x) & 0x3) << 2)
#define   UTMIP_HSDISCON_LEVEL_MSB	(1 << 24)

#define UTMIP_HSRX_CFG0		0x810
#define   UTMIP_ELASTIC_LIMIT(x)	(((x) & 0x1f) << 10)
#define   UTMIP_IDLE_WAIT(x)		(((x) & 0x1f) << 15)

#define UTMIP_HSRX_CFG1		0x814
#define   UTMIP_HS_SYNC_START_DLY(x)	(((x) & 0x1f) << 1)

#define UTMIP_TX_CFG0		0x820
#define   UTMIP_FS_PREABMLE_J		(1 << 19)
#define   UTMIP_HS_DISCON_DISABLE	(1 << 8)

#define UTMIP_MISC_CFG1		0x828
#define   UTMIP_PLL_ACTIVE_DLY_COUNT(x)	(((x) & 0x1f) << 18)
#define   UTMIP_PLLU_STABLE_COUNT(x)	(((x) & 0xfff) << 6)

#define UTMIP_DEBOUNCE_CFG0	0x82c
#define   UTMIP_BIAS_DEBOUNCE_A(x)	(((x) & 0xffff) << 0)

#define UTMIP_BAT_CHRG_CFG0	0x830
#define   UTMIP_PD_CHRG			(1 << 0)
#define   UTMIP_ON_SINK_EN		(1 << 2)
#define   UTMIP_OP_SRC_EN		(1 << 3)

#define UTMIP_XCVR_CFG1		0x838
#define   UTMIP_FORCE_PDDISC_POWERDOWN	(1 << 0)
#define   UTMIP_FORCE_PDCHRP_POWERDOWN	(1 << 2)
#define   UTMIP_FORCE_PDDR_POWERDOWN	(1 << 4)
#define   UTMIP_XCVR_TERM_RANGE_ADJ(x)	(((x) & 0xf) << 18)

#define UTMIP_BIAS_CFG1		0x83c
#define   UTMIP_BIAS_PDTRK_COUNT(x)	(((x) & 0x1f) << 3)
#define   UTMIP_BIAS_PDTRK_POWERDOWN	(1 << 0)
#define   UTMIP_BIAS_PDTRK_POWERUP	(1 << 1)

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
#define   TEGRA_PMC_USB_AO_PD_P2		(0xf << 8)

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

#define PMC_UTMIP_MASTER_CONFIG		0x310
#define UTMIP_PWR(inst)			(1 << (inst))

#define PMC_USB_DEBOUNCE		0xec
#define UTMIP_LINE_DEB_CNT(x)		(((x) & 0xf) << 16)

#define PMC_UTMIP_UHSIC_FAKE		0x218
#define USBON_VAL(inst)		(1 << ((4*(inst))+1))
#define USBON_VAL_P2			(1 << 9)
#define USBON_VAL_P1			(1 << 5)
#define USBON_VAL_P0			(1 << 1)
#define USBOP_VAL(inst)		(1 << (4*(inst)))
#define USBOP_VAL_P2			(1 << 8)
#define USBOP_VAL_P1			(1 << 4)
#define USBOP_VAL_P0			(1 << 0)

#define PMC_SLEEPWALK_CFG		0x200
#define UTMIP_LINEVAL_WALK_EN(inst)	(1 << ((8*(inst))+7))
#define UTMIP_LINEVAL_WALK_EN_P2	(1 << 23)
#define UTMIP_LINEVAL_WALK_EN_P1	(1 << 15)
#define UTMIP_LINEVAL_WALK_EN_P0	(1 << 7)
#define UTMIP_WAKE_VAL(inst, x)	(((x) & 0xf) << ((8*(inst))+4))
#define UTMIP_WAKE_VAL_P2(x)		(((x) & 0xf) << 20)
#define UTMIP_WAKE_VAL_P1(x)		(((x) & 0xf) << 12)
#define UTMIP_WAKE_VAL_P0(x)		(((x) & 0xf) << 4)
#define WAKE_VAL_NONE			0xc
#define WAKE_VAL_FSJ			0x2
#define WAKE_VAL_FSK			0x1
#define WAKE_VAL_SE0			0x0

#define PMC_SLEEP_CFG			0x1fc
#define UTMIP_TCTRL_USE_PMC(inst)	(1 << ((8*(inst))+3))
#define UTMIP_TCTRL_USE_PMC_P2		(1 << 19)
#define UTMIP_TCTRL_USE_PMC_P1		(1 << 11)
#define UTMIP_TCTRL_USE_PMC_P0		(1 << 3)
#define UTMIP_RCTRL_USE_PMC(inst)	(1 << ((8*(inst))+2))
#define UTMIP_RCTRL_USE_PMC_P2		(1 << 18)
#define UTMIP_RCTRL_USE_PMC_P1		(1 << 10)
#define UTMIP_RCTRL_USE_PMC_P0		(1 << 2)
#define UTMIP_FSLS_USE_PMC(inst)	(1 << ((8*(inst))+1))
#define UTMIP_FSLS_USE_PMC_P2		(1 << 17)
#define UTMIP_FSLS_USE_PMC_P1		(1 << 9)
#define UTMIP_FSLS_USE_PMC_P0		(1 << 1)
#define UTMIP_MASTER_ENABLE(inst)	(1 << (8*(inst)))
#define UTMIP_MASTER_ENABLE_P2		(1 << 16)
#define UTMIP_MASTER_ENABLE_P1		(1 << 8)
#define UTMIP_MASTER_ENABLE_P0		(1 << 0)

#define PMC_USB_AO			0xf0
#define USBON_VAL_PD(inst)		(1 << ((4*(inst))+1))
#define USBON_VAL_PD_P2			(1 << 9)
#define USBON_VAL_PD_P1			(1 << 5)
#define USBON_VAL_PD_P0			(1 << 1)
#define USBOP_VAL_PD(inst)		(1 << (4*(inst)))
#define USBOP_VAL_PD_P2			(1 << 8)
#define USBOP_VAL_PD_P1			(1 << 4)
#define USBOP_VAL_PD_P0			(1 << 0)

#define PMC_TRIGGERS			0x1ec
#define UTMIP_CLR_WALK_PTR(inst)	(1 << (inst))
#define UTMIP_CLR_WALK_PTR_P2		(1 << 2)
#define UTMIP_CLR_WALK_PTR_P1		(1 << 1)
#define UTMIP_CLR_WALK_PTR_P0		(1 << 0)
#define UTMIP_CAP_CFG(inst)		(1 << ((inst)+4))
#define UTMIP_CAP_CFG_P2		(1 << 6)
#define UTMIP_CAP_CFG_P1		(1 << 5)
#define UTMIP_CAP_CFG_P0		(1 << 4)
#define UTMIP_CLR_WAKE_ALARM(inst)		(1 << ((inst)+12))
#define UTMIP_CLR_WAKE_ALARM_P2		(1 << 14)

#define PMC_PAD_CFG			(0x1f4)

#define PMC_UTMIP_BIAS_MASTER_CNTRL	0x30c
#define BIAS_MASTER_PROG_VAL		(1 << 1)

#define PMC_SLEEPWALK_REG(inst)		(0x204 + (4*(inst)))
#define PMC_SLEEPWALK_P0		0x204
#define PMC_SLEEPWALK_P1		0x208
#define PMC_SLEEPWALK_P2		0x20c
#define UTMIP_USBOP_RPD_A		(1 << 0)
#define UTMIP_USBON_RPD_A		(1 << 1)
#define UTMIP_AP_A			(1 << 4)
#define UTMIP_AN_A			(1 << 5)
#define UTMIP_HIGHZ_A			(1 << 6)
#define UTMIP_USBOP_RPD_B		(1 << 8)
#define UTMIP_USBON_RPD_B		(1 << 9)
#define UTMIP_AP_B			(1 << 12)
#define UTMIP_AN_B			(1 << 13)
#define UTMIP_HIGHZ_B			(1 << 14)
#define UTMIP_USBOP_RPD_C		(1 << 16)
#define UTMIP_USBON_RPD_C		(1 << 17)
#define UTMIP_AP_C			(1 << 20)
#define UTMIP_AN_C			(1 << 21)
#define UTMIP_HIGHZ_C			(1 << 22)
#define UTMIP_USBOP_RPD_D		(1 << 24)
#define UTMIP_USBON_RPD_D		(1 << 25)
#define UTMIP_AP_D			(1 << 28)
#define UTMIP_AN_D			(1 << 29)
#define UTMIP_HIGHZ_D			(1 << 30)

#define UTMIP_PMC_WAKEUP0		0x84c
#define  EVENT_INT_ENB			(1 << 0)

#define UTMIP_UHSIC_STATUS		0x214
#define UTMIP_WALK_PTR_VAL(inst)	(0x3 << ((inst)*2))
#define UTMIP_USBOP_VAL(inst)		(1 << ((2*(inst)) + 8))
#define UTMIP_USBOP_VAL_P2		(1 << 12)
#define UTMIP_USBOP_VAL_P1		(1 << 10)
#define UTMIP_USBOP_VAL_P0		(1 << 8)
#define UTMIP_USBON_VAL(inst)		(1 << ((2*(inst)) + 9))
#define UTMIP_USBON_VAL_P2		(1 << 13)
#define UTMIP_USBON_VAL_P1		(1 << 11)
#define UTMIP_USBON_VAL_P0		(1 << 9)
#define UTMIP_WAKE_ALARM(inst)		(1 << ((inst) + 16))
#define UTMIP_WAKE_ALARM_P2		(1 << 18)
#define UTMIP_WAKE_ALARM_P1		(1 << 17)
#define UTMIP_WAKE_ALARM_P0		(1 << 16)
#define UTMIP_WALK_PTR(inst)		(1 << ((inst)*2))
#define UTMIP_WALK_PTR_P2		(1 << 4)
#define UTMIP_WALK_PTR_P1		(1 << 2)
#define UTMIP_WALK_PTR_P0		(1 << 0)

#define UTMIP_BIAS_STS0         0x840
#define UTMIP_RCTRL_VAL(x)      (((x) & 0xffff) << 0)
#define UTMIP_TCTRL_VAL(x)      (((x) & (0xffff << 16)) >> 16)

#define PMC_UTMIP_TERM_PAD_CFG	0x1f8
#define PMC_TCTRL_VAL(x)	(((x) & 0x1f) << 5)
#define PMC_RCTRL_VAL(x)	(((x) & 0x1f) << 0)

static u32 utmip_rctrl_val, utmip_tctrl_val;

#endif

/* Common registers */
#define UTMIP_MISC_CFG0		0x824
#define   UTMIP_DPDM_OBSERVE		(1 << 26)
#define   UTMIP_DPDM_OBSERVE_SEL(x)	(((x) & 0xf) << 27)
#define   UTMIP_DPDM_OBSERVE_SEL_FS_J	UTMIP_DPDM_OBSERVE_SEL(0xf)
#define   UTMIP_DPDM_OBSERVE_SEL_FS_K	UTMIP_DPDM_OBSERVE_SEL(0xe)
#define   UTMIP_DPDM_OBSERVE_SEL_FS_SE1 UTMIP_DPDM_OBSERVE_SEL(0xd)
#define   UTMIP_DPDM_OBSERVE_SEL_FS_SE0 UTMIP_DPDM_OBSERVE_SEL(0xc)
#define   UTMIP_SUSPEND_EXIT_ON_EDGE	(1 << 22)
#define   FORCE_PULLDN_DM	(1 << 8)
#define   FORCE_PULLDN_DP	(1 << 9)
#define   COMB_TERMS		(1 << 0)
#define   ALWAYS_FREE_RUNNING_TERMS	(1 << 1)

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
#define   FUSE_ATERM_SEL		(1 << 4)

#define FUSE_USB_CALIB_0		0x1F0
#define FUSE_USB_CALIB_XCVR_SETUP(x)	(((x) & 0x7F) << 0)

#define UHSIC_PLL_CFG0		0x800

#define UHSIC_TX_CFG0				0x810
#define   UHSIC_HS_POSTAMBLE_OUTPUT_ENABLE	(1 << 6)

#define UHSIC_CMD_CFG0				0x824
#define   UHSIC_PRETEND_CONNECT_DETECT		(1 << 5)

#define UHSIC_SPARE_CFG0			0x82c

/* These values (in milli second) are taken from the battery charging spec */
#define TDP_SRC_ON_MS	 100
#define TDPSRC_CON_MS	 40

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
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
	},
	[2] = {
		.hssync_start_delay = 9,
		.idle_wait_delay = 17,
		.elastic_limit = 16,
		.term_range_adj = 6,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.xcvr_setup = 9,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
	},
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

	utmip_pad_count++;
	val = readl(base + UTMIP_BIAS_CFG0);
	val &= ~(UTMIP_OTGPD | UTMIP_BIASPD);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	val |= UTMIP_HSSQUELCH_LEVEL(0x2) | UTMIP_HSDISCON_LEVEL(0x1) |
		UTMIP_HSDISCON_LEVEL_MSB;
#endif
	writel(val, base + UTMIP_BIAS_CFG0);

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
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		val &= ~(UTMIP_HSSQUELCH_LEVEL(~0) | UTMIP_HSDISCON_LEVEL(~0) |
			UTMIP_HSDISCON_LEVEL_MSB);
#endif
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

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static void utmip_phy_enable_trking_data(struct tegra_usb_phy *phy)
{
	void __iomem *base = phy->pad_regs;
	void __iomem *pmc_base = IO_ADDRESS(TEGRA_USB_BASE);
	static bool init_done = false;
	u32 val;

	/* Should be done only once after system boot */
	if (init_done)
		return;

	clk_enable(phy->pad_clk);
	/* Bias pad MASTER_ENABLE=1 */
	val = readl(pmc_base + PMC_UTMIP_BIAS_MASTER_CNTRL);
	val |= BIAS_MASTER_PROG_VAL;
	writel(val, pmc_base + PMC_UTMIP_BIAS_MASTER_CNTRL);

	/* Setting the tracking length time */
	val = readl(base + UTMIP_BIAS_CFG1);
	val &= ~UTMIP_BIAS_PDTRK_COUNT(~0);
	val |= UTMIP_BIAS_PDTRK_COUNT(5);
	writel(val, base + UTMIP_BIAS_CFG1);

	/* Bias PDTRK is Shared and MUST be done from USB1 ONLY, PD_TRK=0 */
	val = readl(base + UTMIP_BIAS_CFG1);
	val &= ~ UTMIP_BIAS_PDTRK_POWERDOWN;
	writel(val, base + UTMIP_BIAS_CFG1);

	val = readl(base + UTMIP_BIAS_CFG1);
	val |= UTMIP_BIAS_PDTRK_POWERUP;
	writel(val, base + UTMIP_BIAS_CFG1);

	/* Wait for 25usec */
	udelay(25);

	/* Bias pad MASTER_ENABLE=0 */
	val = readl(pmc_base + PMC_UTMIP_BIAS_MASTER_CNTRL);
	val &= ~BIAS_MASTER_PROG_VAL;
	writel(val, pmc_base + PMC_UTMIP_BIAS_MASTER_CNTRL);

	/* Wait for 1usec */
	udelay(1);

	/* Bias pad MASTER_ENABLE=1 */
	val = readl(pmc_base + PMC_UTMIP_BIAS_MASTER_CNTRL);
	val |= BIAS_MASTER_PROG_VAL;
	writel(val, pmc_base + PMC_UTMIP_BIAS_MASTER_CNTRL);

	/* Read RCTRL and TCTRL from UTMIP space */
	val = readl(base + UTMIP_BIAS_STS0);
	utmip_rctrl_val = ffz(UTMIP_RCTRL_VAL(val));
	utmip_tctrl_val = ffz(UTMIP_TCTRL_VAL(val));

	/* PD_TRK=1 */
	val = readl(base + UTMIP_BIAS_CFG1);
	val |= UTMIP_BIAS_PDTRK_POWERDOWN;
	writel(val, base + UTMIP_BIAS_CFG1);

	/* Program thermally encoded RCTRL_VAL, TCTRL_VAL into PMC space */
	val = readl(pmc_base + PMC_UTMIP_TERM_PAD_CFG);
	val = PMC_TCTRL_VAL(utmip_tctrl_val) | PMC_RCTRL_VAL(utmip_rctrl_val);
	writel(val, pmc_base + PMC_UTMIP_TERM_PAD_CFG);
	clk_disable(phy->pad_clk);
	init_done = true;
}
#endif

static unsigned int tegra_phy_xcvr_setup_value(struct tegra_utmip_config *cfg)
{
	unsigned long val;

	if (cfg->xcvr_use_fuses) {
		val = FUSE_USB_CALIB_XCVR_SETUP(
				tegra_fuse_readl(FUSE_USB_CALIB_0));
		if (cfg->xcvr_setup_offset <= UTMIP_XCVR_MAX_OFFSET)
			val = val + cfg->xcvr_setup_offset;

		if (val > UTMIP_XCVR_SETUP_MAX_VALUE) {
			val = UTMIP_XCVR_SETUP_MAX_VALUE;
			pr_info("%s: reset XCVR_SETUP to max value\n",
				 __func__);
		}
	} else {
		val = cfg->xcvr_setup;
	}

	return val;
}

static int utmi_phy_power_on(struct tegra_usb_phy *phy, bool is_dpd)
{
	unsigned long val;
	void __iomem *base = phy->regs;
	unsigned int xcvr_setup_value;
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

	xcvr_setup_value = tegra_phy_xcvr_setup_value(config);

	val = readl(base + UTMIP_XCVR_CFG0);
	val &= ~(UTMIP_XCVR_LSBIAS_SEL | UTMIP_FORCE_PD_POWERDOWN |
		 UTMIP_FORCE_PD2_POWERDOWN | UTMIP_FORCE_PDZI_POWERDOWN |
		 UTMIP_XCVR_SETUP(~0) | UTMIP_XCVR_LSFSLEW(~0) |
		 UTMIP_XCVR_LSRSLEW(~0) | UTMIP_XCVR_HSSLEW_MSB(~0));
	val |= UTMIP_XCVR_SETUP(xcvr_setup_value);
	val |= UTMIP_XCVR_SETUP_MSB(XCVR_SETUP_MSB_CALIB(xcvr_setup_value));
	val |= UTMIP_XCVR_LSFSLEW(config->xcvr_lsfslew);
	val |= UTMIP_XCVR_LSRSLEW(config->xcvr_lsrslew);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	val |= UTMIP_XCVR_HSSLEW_MSB(0x8);
#endif
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

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	val = readl(base + UTMIP_SPARE_CFG0);
	val &= ~FUSE_SETUP_SEL;
	writel(val, base + UTMIP_SPARE_CFG0);

	if (phy->instance == 2) {
		val = readl(base + UTMIP_SPARE_CFG0);
		val |= FUSE_SETUP_SEL;
		writel(val, base + UTMIP_SPARE_CFG0);

		val = readl(base + USB_SUSP_CTRL);
		val |= UTMIP_PHY_ENABLE;
		writel(val, base + USB_SUSP_CTRL);
	}
#else
	val = readl(base + UTMIP_SPARE_CFG0);
	val &= ~FUSE_SETUP_SEL;
	val |= FUSE_ATERM_SEL;
	writel(val, base + UTMIP_SPARE_CFG0);

	val = readl(base + USB_SUSP_CTRL);
	val |= UTMIP_PHY_ENABLE;
	writel(val, base + USB_SUSP_CTRL);
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
	if (phy->instance == 0)
		utmip_phy_enable_trking_data(phy);

	if(phy->instance == 2) {
		writel(0, base + ICUSB_CTRL);
	}

	if (phy->mode == TEGRA_USB_PHY_MODE_HOST) {
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

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static void utmip_setup_pmc_wake_detect(struct tegra_usb_phy *phy)
{
	unsigned long val, pmc_pad_cfg_val;
	void __iomem *pmc_base = IO_ADDRESS(TEGRA_PMC_BASE);
	unsigned  int inst = phy->instance;
	void __iomem *base = phy->regs;
	bool port_connected;
	enum tegra_usb_phy_port_speed port_speed;

	/* check for port connect status */
	val = readl(base + USB_PORTSC1);
	port_connected = val & USB_PORTSC1_CCS;

	if (!port_connected)
		return;

	port_speed = (readl(base + HOSTPC1_DEVLC) >> 25) &
		HOSTPC1_DEVLC_PSPD_MASK;
	/*Set PMC MASTER bits to do the following
	* a. Take over the UTMI drivers
	* b. set up such that it will take over resume
	*    if remote wakeup is detected
	* Prepare PMC to take over suspend-wake detect-drive resume until USB
	* controller ready
	*/

	/* disable master enable in PMC */
	val = readl(pmc_base + PMC_SLEEP_CFG);
	val &= ~UTMIP_MASTER_ENABLE(inst);
	writel(val, pmc_base + PMC_SLEEP_CFG);

	/* UTMIP_PWR_PX=1 for power savings mode */
	val = readl(pmc_base + PMC_UTMIP_MASTER_CONFIG);
	val |= UTMIP_PWR(inst);
	writel(val, pmc_base + PMC_UTMIP_MASTER_CONFIG);

	/* config debouncer */
	val = readl(pmc_base + PMC_USB_DEBOUNCE);
	val &= ~UTMIP_LINE_DEB_CNT(~0);
	val |= UTMIP_LINE_DEB_CNT(1);
	writel(val, pmc_base + PMC_USB_DEBOUNCE);

	/* Make sure nothing is happening on the line with respect to PMC */
	val = readl(pmc_base + PMC_UTMIP_UHSIC_FAKE);
	val &= ~USBOP_VAL(inst);
	val &= ~USBON_VAL(inst);
	writel(val, pmc_base + PMC_UTMIP_UHSIC_FAKE);

	/* Make sure wake value for line is none */
	val = readl(pmc_base + PMC_SLEEPWALK_CFG);
	val &= ~UTMIP_LINEVAL_WALK_EN(inst);
	writel(val, pmc_base + PMC_SLEEPWALK_CFG);
	val = readl(pmc_base + PMC_SLEEP_CFG);
	val &= ~UTMIP_WAKE_VAL(inst, ~0);
	val |= UTMIP_WAKE_VAL(inst, WAKE_VAL_NONE);
	writel(val, pmc_base + PMC_SLEEP_CFG);

	/* turn off pad detectors */
	val = readl(pmc_base + PMC_USB_AO);
	val |= (USBOP_VAL_PD(inst) | USBON_VAL_PD(inst));
	writel(val, pmc_base + PMC_USB_AO);

	/* Remove fake values and make synchronizers work a bit */
	val = readl(pmc_base + PMC_UTMIP_UHSIC_FAKE);
	val &= ~USBOP_VAL(inst);
	val &= ~USBON_VAL(inst);
	writel(val, pmc_base + PMC_UTMIP_UHSIC_FAKE);

	/* Enable which type of event can trigger a walk,
	in this case usb_line_wake */
	val = readl(pmc_base + PMC_SLEEPWALK_CFG);
	val |= UTMIP_LINEVAL_WALK_EN(inst);
	writel(val, pmc_base + PMC_SLEEPWALK_CFG);

	/* Enable which type of event can trigger a walk,
	* in this case usb_line_wake */
	val = readl(pmc_base + PMC_SLEEPWALK_CFG);
	val |= UTMIP_LINEVAL_WALK_EN(inst);
	writel(val, pmc_base + PMC_SLEEPWALK_CFG);

	/* Clear the walk pointers and wake alarm */
	val = readl(pmc_base + PMC_TRIGGERS);
	val |= UTMIP_CLR_WAKE_ALARM(inst) | UTMIP_CLR_WALK_PTR(inst);
	writel(val, pmc_base + PMC_TRIGGERS);


	/* Capture FS/LS pad configurations */
	pmc_pad_cfg_val = readl(pmc_base + PMC_PAD_CFG);
	val = readl(pmc_base + PMC_TRIGGERS);
	val |= UTMIP_CAP_CFG(inst);
	writel(val, pmc_base + PMC_TRIGGERS);
	udelay(1);
	pmc_pad_cfg_val = readl(pmc_base + PMC_PAD_CFG);

	/* BIAS MASTER_ENABLE=0 */
	val = readl(pmc_base + PMC_UTMIP_BIAS_MASTER_CNTRL);
	val &= ~BIAS_MASTER_PROG_VAL;
	writel(val, pmc_base + PMC_UTMIP_BIAS_MASTER_CNTRL);

	/* program walk sequence, maintain a J, followed by a driven K
	* to signal a resume once an wake event is detected */
	val = readl(pmc_base + PMC_SLEEPWALK_REG(inst));
	val &= ~UTMIP_AP_A;
	val |= UTMIP_USBOP_RPD_A | UTMIP_USBON_RPD_A| UTMIP_AN_A | UTMIP_HIGHZ_A |
		UTMIP_USBOP_RPD_B | UTMIP_USBON_RPD_B | UTMIP_AP_B |
		UTMIP_USBOP_RPD_C | UTMIP_USBON_RPD_C | UTMIP_AP_C |
		UTMIP_USBOP_RPD_D | UTMIP_USBON_RPD_D | UTMIP_AP_D;
	writel(val, pmc_base + PMC_SLEEPWALK_REG(inst));

	if (port_speed == TEGRA_USB_PHY_PORT_SPEED_LOW) {
		val = readl(pmc_base + PMC_SLEEPWALK_REG(inst));
		val &= ~(UTMIP_AN_B | UTMIP_HIGHZ_B | UTMIP_AN_C |
			UTMIP_HIGHZ_C | UTMIP_AN_D | UTMIP_HIGHZ_D);
		writel(val, pmc_base + PMC_SLEEPWALK_REG(inst));
	} else {
		val = readl(pmc_base + PMC_SLEEPWALK_REG(inst));
		val &= ~(UTMIP_AP_B | UTMIP_HIGHZ_B | UTMIP_AP_C |
			UTMIP_HIGHZ_C | UTMIP_AP_D | UTMIP_HIGHZ_D);
		writel(val, pmc_base + PMC_SLEEPWALK_REG(inst));
	}

	/* turn on pad detectors */
	val = readl(pmc_base + PMC_USB_AO);
	val &= ~(USBOP_VAL_PD(inst) | USBON_VAL_PD(inst));
	writel(val, pmc_base + PMC_USB_AO);

	/* Add small delay before usb detectors provide stable line values */
	udelay(1);

	/* Program thermally encoded RCTRL_VAL, TCTRL_VAL into PMC space */
	val = readl(pmc_base + PMC_UTMIP_TERM_PAD_CFG);
	val = PMC_TCTRL_VAL(utmip_tctrl_val) | PMC_RCTRL_VAL(utmip_rctrl_val);
	writel(val, pmc_base + PMC_UTMIP_TERM_PAD_CFG);

	phy->remote_wakeup = false;

	/* Turn over pad configuration to PMC  for line wake events*/
	val = readl(pmc_base + PMC_SLEEP_CFG);
	val &= ~UTMIP_WAKE_VAL(inst, ~0);
	if (port_speed == TEGRA_USB_PHY_PORT_SPEED_LOW)
		val |= UTMIP_WAKE_VAL(inst, WAKE_VAL_FSJ);
	else
		val |= UTMIP_WAKE_VAL(inst, WAKE_VAL_FSK);
	val |= UTMIP_RCTRL_USE_PMC(inst) | UTMIP_TCTRL_USE_PMC(inst);
	val |= UTMIP_MASTER_ENABLE(inst) | UTMIP_FSLS_USE_PMC(inst);
	writel(val, pmc_base + PMC_SLEEP_CFG);

	val = readl(base + UTMIP_PMC_WAKEUP0);
	val |= EVENT_INT_ENB;
	writel(val, base + UTMIP_PMC_WAKEUP0);
}
#endif

static int utmi_phy_power_off(struct tegra_usb_phy *phy, bool is_dpd)
{
	unsigned long val;
	void __iomem *base = phy->regs;

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	if (phy->mode == TEGRA_USB_PHY_MODE_HOST)
		utmip_setup_pmc_wake_detect(phy);
#endif
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

	if (phy->instance != 2) {
		val = readl(base + UTMIP_XCVR_CFG0);
		val |= (UTMIP_FORCE_PD_POWERDOWN | UTMIP_FORCE_PD2_POWERDOWN |
			 UTMIP_FORCE_PDZI_POWERDOWN);
		writel(val, base + UTMIP_XCVR_CFG0);
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
	if (phy->instance != 0) {
		val = readl(base + UTMIP_BIAS_CFG0);
		val |= UTMIP_OTGPD;
		writel(val, base + UTMIP_BIAS_CFG0);
	}

	utmi_phy_clk_disable(phy);

	if (phy->instance == 2) {
		val = readl(base + USB_SUSP_CTRL);
		val |= USB_PHY_CLK_VALID_INT_ENB;
		writel(val, base + USB_SUSP_CTRL);
	} else {
		val = readl(base + USB_SUSP_CTRL);
		val |= UTMIP_RESET;
		writel(val, base + USB_SUSP_CTRL);
	}
	utmip_pad_power_off(phy, true);
	return 0;
}

static void utmip_phy_disable_pmc_bus_ctrl(struct tegra_usb_phy *phy)
{
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	unsigned long val;
	void __iomem *pmc_base = IO_ADDRESS(TEGRA_PMC_BASE);
	unsigned  int inst = phy->instance;
	void __iomem *base = phy->regs;

	val = readl(pmc_base + PMC_SLEEP_CFG);
	val &= ~UTMIP_WAKE_VAL(inst, 0x0);
	val |= UTMIP_WAKE_VAL(inst, WAKE_VAL_NONE);
	writel(val, pmc_base + PMC_SLEEP_CFG);

	val = readl(pmc_base + PMC_TRIGGERS);
	val |= UTMIP_CLR_WAKE_ALARM(inst) | UTMIP_CLR_WALK_PTR(inst);
	writel(val, pmc_base + PMC_TRIGGERS);

	val = readl(base + UTMIP_PMC_WAKEUP0);
	val &= ~EVENT_INT_ENB;
	writel(val, base + UTMIP_PMC_WAKEUP0);

	/* Disable PMC master mode by clearing MASTER_EN */
	val = readl(pmc_base + PMC_SLEEP_CFG);
	val &= ~(UTMIP_RCTRL_USE_PMC(inst) | UTMIP_TCTRL_USE_PMC(inst) |
			UTMIP_FSLS_USE_PMC(inst) | UTMIP_MASTER_ENABLE(inst));
	writel(val, pmc_base + PMC_SLEEP_CFG);

	val = readl(pmc_base + PMC_TRIGGERS);
	val &= ~UTMIP_CAP_CFG(inst);
	writel(val, pmc_base + PMC_TRIGGERS);

	/* turn off pad detectors */
	val = readl(pmc_base + PMC_USB_AO);
	val |= (USBOP_VAL_PD(inst) | USBON_VAL_PD(inst));
	writel(val, pmc_base + PMC_USB_AO);

	phy->remote_wakeup = false;
#endif
}

static int utmi_phy_preresume(struct tegra_usb_phy *phy, bool is_dpd)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	unsigned long val;
	void __iomem *base = phy->regs;
	val = readl(base + UTMIP_TX_CFG0);
	val |= UTMIP_HS_DISCON_DISABLE;
	writel(val, base + UTMIP_TX_CFG0);
#else
	utmip_phy_disable_pmc_bus_ctrl(phy);
#endif

	return 0;
}

static int utmi_phy_postresume(struct tegra_usb_phy *phy, bool is_dpd)
{
	unsigned long val;
	void __iomem *base = phy->regs;

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* check if OBS bus is already enabled */
	val = readl(base + UTMIP_MISC_CFG0);
	if (val & UTMIP_DPDM_OBSERVE) {
		/* Change the UTMIP OBS bus to drive SE0 */
		val = readl(base + UTMIP_MISC_CFG0);
		val &= ~UTMIP_DPDM_OBSERVE_SEL(~0);
		val |= UTMIP_DPDM_OBSERVE_SEL_FS_SE0;
		writel(val, base + UTMIP_MISC_CFG0);

		/* Wait for 3us(2 LS bit times) */
		udelay (3);

		/* Release UTMIP OBS bus */
		val = readl(base + UTMIP_MISC_CFG0);
		val &= ~UTMIP_DPDM_OBSERVE;
		writel(val, base + UTMIP_MISC_CFG0);

		/* Release DP/DM pulldown for Host mode */
		val = readl(base + UTMIP_MISC_CFG0);
		val &= ~(FORCE_PULLDN_DM | FORCE_PULLDN_DP |
				COMB_TERMS | ALWAYS_FREE_RUNNING_TERMS);
		writel(val, base + UTMIP_MISC_CFG0);
	}
#else
	val = readl(base + UTMIP_TX_CFG0);
	val &= ~UTMIP_HS_DISCON_DISABLE;
	writel(val, base + UTMIP_TX_CFG0);
#endif
	return 0;
}

static int uhsic_phy_postsuspend(struct tegra_usb_phy *phy, bool is_dpd)
{
	struct tegra_uhsic_config *uhsic_config = phy->config;

	if (uhsic_config->postsuspend)
		uhsic_config->postsuspend();

	return 0;
}

static int uhsic_phy_preresume(struct tegra_usb_phy *phy, bool is_dpd)
{
	struct tegra_uhsic_config *uhsic_config = phy->config;

	if (uhsic_config->preresume)
		uhsic_config->preresume();

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
#else
	unsigned long val;
	void __iomem *base = phy->regs;
	void __iomem *pmc_base = IO_ADDRESS(TEGRA_PMC_BASE);
	int inst = phy->instance;

	val = readl(pmc_base + UTMIP_UHSIC_STATUS);
	/* check whether we wake up from the remote resume */
	if (UTMIP_WALK_PTR_VAL(inst) & val) {
		phy->remote_wakeup = true;
	} else {
		if (!((UTMIP_USBON_VAL(phy->instance) |
			UTMIP_USBOP_VAL(phy->instance)) & val)) {
				utmip_phy_disable_pmc_bus_ctrl(phy);
		}
	}

	/* (2LS WAR)is not required for LS and FS devices and is only for HS */
	if ((port_speed == TEGRA_USB_PHY_PORT_SPEED_LOW) ||
		(port_speed == TEGRA_USB_PHY_PORT_SPEED_FULL)) {
		/* do not enable the OBS bus */
		val = readl(base + UTMIP_MISC_CFG0);
		val &= ~UTMIP_DPDM_OBSERVE_SEL(~0);
		writel(val, base + UTMIP_MISC_CFG0);
		return;
	}
	/* Force DP/DM pulldown active for Host mode */
	val = readl(base + UTMIP_MISC_CFG0);
	val |= FORCE_PULLDN_DM | FORCE_PULLDN_DP |
			COMB_TERMS | ALWAYS_FREE_RUNNING_TERMS;
	writel(val, base + UTMIP_MISC_CFG0);
	val = readl(base + UTMIP_MISC_CFG0);
	val &= ~UTMIP_DPDM_OBSERVE_SEL(~0);
	if (port_speed == TEGRA_USB_PHY_PORT_SPEED_LOW)
		val |= UTMIP_DPDM_OBSERVE_SEL_FS_J;
	else
		val |= UTMIP_DPDM_OBSERVE_SEL_FS_K;
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
#else
	unsigned long val;
	void __iomem *base = phy->regs;
	int wait_time_us = 3000; /* FPR should be set by this time */

	/* check whether we wake up from the remote resume */
	if (phy->remote_wakeup) {
		/* wait until FPR bit is set automatically on remote resume */
		do {
			val = readl(base + USB_PORTSC1);
			udelay(1);
			if (wait_time_us == 0)
				return;
			wait_time_us--;
		} while (!(val & USB_PORTSC1_RESUME));
		/* disable PMC master control */
		utmip_phy_disable_pmc_bus_ctrl(phy);
		/* wait for 25 ms to port resume complete */
		msleep(25);

		/* Clear PCI and SRI bits to avoid an interrupt upon resume */
		val = readl(base + USB_USBSTS);
		writel(val, base + USB_USBSTS);
		/* wait to avoid SOF if there is any */
		if (utmi_wait_register(base + USB_USBSTS,
			USB_USBSTS_SRI, USB_USBSTS_SRI) < 0) {
			pr_err("%s: timeout waiting for SOF\n", __func__);
		}
		tegra_usb_phy_postresume(phy, false);
	}
#endif
}

static void ulpi_set_tristate(bool enable)
{
	int tristate = (enable)? TEGRA_TRI_TRISTATE : TEGRA_TRI_NORMAL;

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_UAA, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_UAB, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_UDA, tristate);
#else
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
	ulpi_set_tristate(true);

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

	ulpi_set_tristate(false);
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

	if (phy->clk)
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

	if(phy->clk)
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

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* remove ULPI PADS CLKEN reset */
	val = readl(base + USB_SUSP_CTRL);
	val &= ~ULPI_PADS_CLKEN_RESET;
	writel(val, base + USB_SUSP_CTRL);
	udelay(10);
#endif
	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_OUTPUT_PINMUX_BYP | ULPI_CLKOUT_PINMUX_BYP;
	writel(val, base + ULPI_TIMING_CTRL_0);

	ulpi_set_tristate(false);

	if (config->pre_phy_on && config->pre_phy_on())
		return -EAGAIN;

	val = readl(base + USB_SUSP_CTRL);
	val |= ULPI_PHY_ENABLE;
	writel(val, base + USB_SUSP_CTRL);
	udelay(10);

	/* set timming parameters */
	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_SHADOW_CLK_LOOPBACK_EN;
	val &= ~ULPI_SHADOW_CLK_SEL;
	val &= ~ULPI_LBK_PAD_EN;
	val |= ULPI_SHADOW_CLK_DELAY(config->trimmer->shadow_clk_delay);
	val |= ULPI_CLOCK_OUT_DELAY(config->trimmer->clock_out_delay);
	val |= ULPI_LBK_PAD_E_INPUT_OR;
	writel(val, base + ULPI_TIMING_CTRL_0);

	writel(0, base + ULPI_TIMING_CTRL_1);
	udelay(10);

	/* start internal 60MHz clock */
	val = readl(base + ULPIS2S_CTRL);
	val |= ULPIS2S_ENA;
	val |= ULPIS2S_SUPPORT_DISCONNECT;
	val |= ULPIS2S_SPARE((phy->mode == TEGRA_USB_PHY_MODE_HOST) ? 3 : 1);
	val |= ULPIS2S_PLLU_MASTER_BLASTER60;
	writel(val, base + ULPIS2S_CTRL);

	/* select ULPI_CORE_CLK_SEL to SHADOW_CLK */
	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_CORE_CLK_SEL;
	writel(val, base + ULPI_TIMING_CTRL_0);
	udelay(10);

	/* enable ULPI null phy clock - can't set the trimmers before this */
	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_CLK_OUT_ENA;
	writel(val, base + ULPI_TIMING_CTRL_0);
	udelay(10);

	if (utmi_wait_register(base + USB_SUSP_CTRL, USB_PHY_CLK_VALID,
						     USB_PHY_CLK_VALID)) {
		pr_err("%s: timeout waiting for phy to stabilize\n", __func__);
		return -ETIMEDOUT;
	}

	/* set ULPI trimmers */
	ulpi_set_trimmer(base, config->trimmer->data_trimmer,
			 config->trimmer->stpdirnxt_trimmer, 1);

	ulpi_set_host(base);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* remove slave0 reset */
	val = readl(base + USB_SUSP_CTRL);
	val &= ~ULPIS2S_SLV0_RESET;
	writel(val, base + USB_SUSP_CTRL);

	/* remove slave1 and line reset */
	val = readl(base + USB_SUSP_CTRL);
	val &= ~ULPIS2S_SLV1_RESET;
	val &= ~ULPIS2S_LINE_RESET;

	/* remove ULPI PADS reset */
	val &= ~ULPI_PADS_RESET;
	writel(val, base + USB_SUSP_CTRL);
#endif
	val = readl(base + ULPI_TIMING_CTRL_0);
	val |= ULPI_CLK_PADOUT_ENA;
	writel(val, base + ULPI_TIMING_CTRL_0);

	udelay(10);

	if (config->post_phy_on && config->post_phy_on())
		return -EAGAIN;

	return 0;
}

static int null_phy_power_off(struct tegra_usb_phy *phy, bool is_dpd)
{
	unsigned long val;
	void __iomem *base = phy->regs;
	struct tegra_ulpi_config *config = phy->config;

	if (config->pre_phy_off && config->pre_phy_off())
		return -EAGAIN;

	ulpi_set_tristate(true);

	if (config->post_phy_off && config->post_phy_off())
		return -EAGAIN;

	return 0;
}

static int null_phy_pre_usbcmd_reset(struct tegra_usb_phy *phy, bool is_dpd)
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
#endif
	return 0;
}

static int null_phy_post_usbcmd_reset(struct tegra_usb_phy *phy, bool is_dpd)
{
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	unsigned long val;
	void __iomem *base = phy->regs;

	/* remove slave0 reset */
	val = readl(base + USB_SUSP_CTRL);
	val &= ~ULPIS2S_SLV0_RESET;
	writel(val, base + USB_SUSP_CTRL);

	val = readl(base + ULPIS2S_CTRL);
	val &=  ~ULPIS2S_SLV0_CLAMP_XMIT;
	writel(val, base + ULPIS2S_CTRL);
	udelay(10);

	ulpi_set_host(base);
#endif
	return 0;
}

static int uhsic_phy_power_on(struct tegra_usb_phy *phy, bool is_dpd)
{
	unsigned long val;
	void __iomem *base = phy->regs;
	struct tegra_uhsic_config *uhsic_config = phy->config;

	if (uhsic_config->enable_gpio != -1) {
		gpio_set_value_cansleep(uhsic_config->enable_gpio, 1);
		/* keep hsic reset asserted for 1 ms */
		udelay(1000);
	}

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
	val |= UHSIC_IDLE_WAIT(uhsic_config->idle_wait_delay);
	val |= UHSIC_ELASTIC_UNDERRUN_LIMIT(uhsic_config->elastic_underrun_limit);
	val |= UHSIC_ELASTIC_OVERRUN_LIMIT(uhsic_config->elastic_overrun_limit);
	writel(val, base + UHSIC_HSRX_CFG0);

	val = readl(base + UHSIC_HSRX_CFG1);
	val |= UHSIC_HS_SYNC_START_DLY(uhsic_config->sync_start_delay);
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
	struct tegra_uhsic_config *uhsic_config = phy->config;

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

	if (uhsic_config->enable_gpio != -1) {
		gpio_set_value_cansleep(uhsic_config->enable_gpio, 0);
		/* keep hsic reset de-asserted for 1 ms */
		udelay(1000);
	}

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
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct tegra_ulpi_config *uhsic_config;
	int reset_gpio, enable_gpio;
#endif

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
	phy->power_on = 0;
	phy->remote_wakeup = false;

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

		if (ulpi_config->clk) {
			phy->clk = clk_get_sys(NULL, ulpi_config->clk);
			if (IS_ERR(phy->clk)) {
				pr_err("%s: can't get ulpi clock\n", __func__);
				err = -ENXIO;
				goto err1;
			}
		} else {
			/* Some USB ULPI chips are not driven by Tegra clocks or PLL */
			phy->clk = NULL;
		}
		tegra_gpio_enable(ulpi_config->reset_gpio);
		gpio_request(ulpi_config->reset_gpio, "ulpi_phy_reset_b");
		gpio_direction_output(ulpi_config->reset_gpio, 0);
		phy->ulpi = otg_ulpi_create(&ulpi_viewport_access_ops, 0);
		phy->ulpi->io_priv = regs + ULPI_VIEWPORT;
	}
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	else if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_HSIC) {
		uhsic_config = config;
		enable_gpio = gpio_request(uhsic_config->enable_gpio,
			"uhsic_enable");
		reset_gpio = gpio_request(uhsic_config->reset_gpio,
				"uhsic_reset");
		/* hsic enable signal deasserted, hsic reset asserted */
		if (!enable_gpio)
			gpio_direction_output(uhsic_config->enable_gpio,
			0 /* deasserted */);
		if (!reset_gpio)
			gpio_direction_output(uhsic_config->reset_gpio,
				0 /* asserted */);
		if (!enable_gpio)
			tegra_gpio_enable(uhsic_config->enable_gpio);
		if (!reset_gpio)
			tegra_gpio_enable(uhsic_config->reset_gpio);
		/* keep hsic reset asserted for 1 ms */
		udelay(1000);
		/* enable (power on) hsic */
		if (!enable_gpio)
			gpio_set_value_cansleep(uhsic_config->enable_gpio, 1);
		udelay(1000);
		/* deassert reset */
		if (!reset_gpio)
			gpio_set_value_cansleep(uhsic_config->reset_gpio, 1);
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
	if (instance == 2) {
		writel(readl((IO_ADDRESS(TEGRA_PMC_BASE) + TEGRA_PMC_USB_AO)) &
			(TEGRA_PMC_USB_AO_PD_P2),
			(IO_ADDRESS(TEGRA_PMC_BASE) + TEGRA_PMC_USB_AO));
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

	if (phy->power_on)
		return ret;

	if (phy->reg_vdd && !phy->regulator_on) {
		regulator_enable(phy->reg_vdd);
		phy->regulator_on = 1;
	}

	if (power_on[phy->usb_phy_type])
		ret = power_on[phy->usb_phy_type](phy, is_dpd);

	phy->power_on = true;
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

	if (!phy->power_on)
		return;

	if (power_off[phy->usb_phy_type])
		power_off[phy->usb_phy_type](phy, is_dpd);

	if (phy->reg_vdd && phy->regulator_on && is_dpd) {
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		if (tegra_get_revision() >= TEGRA_REVISION_A03)
#endif
		regulator_disable(phy->reg_vdd);
		phy->regulator_on = 0;
	}
	phy->power_on = false;
}

void tegra_usb_phy_preresume(struct tegra_usb_phy *phy, bool is_dpd)
{
	const tegra_phy_fp preresume[] = {
		utmi_phy_preresume,
		NULL,
		NULL,
		uhsic_phy_preresume,
	};

	if (preresume[phy->usb_phy_type])
		preresume[phy->usb_phy_type](phy, is_dpd);
}

void tegra_usb_phy_postsuspend(struct tegra_usb_phy *phy, bool is_dpd)

{
	const tegra_phy_fp postsuspend[] = {
		NULL,
		NULL,
		NULL,
		uhsic_phy_postsuspend,
	};

	if (postsuspend[phy->usb_phy_type])
		postsuspend[phy->usb_phy_type](phy, is_dpd);
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

void tegra_ehci_pre_reset(struct tegra_usb_phy *phy, bool is_dpd)
{
	const tegra_phy_fp pre_reset[] = {
		NULL,
		NULL,
		null_phy_pre_usbcmd_reset,
		NULL,
	};

	if (pre_reset[phy->usb_phy_type])
		pre_reset[phy->usb_phy_type](phy, is_dpd);
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
	if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_UTMIP) {
		utmip_pad_close(phy);
		utmip_phy_disable_pmc_bus_ctrl(phy);
	}
	else if (phy->usb_phy_type == TEGRA_USB_PHY_TYPE_LINK_ULPI && phy->clk)
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

bool tegra_usb_phy_charger_detect(struct tegra_usb_phy *phy)
{
	unsigned long val;
	void __iomem *base = phy->regs;
	bool status;

	if (phy->usb_phy_type != TEGRA_USB_PHY_TYPE_UTMIP)
	{
		/* Charger detection is not there for ULPI
		 * return Charger not available */
		return false;
	}

	/* Enable charger detection logic */
	val = readl(base + UTMIP_BAT_CHRG_CFG0);
	val |= UTMIP_OP_SRC_EN | UTMIP_ON_SINK_EN;
	writel(val, base + UTMIP_BAT_CHRG_CFG0);

	/* Source should be on for 100 ms as per USB charging spec */
	msleep(TDP_SRC_ON_MS);

	val = readl(base + USB_PHY_VBUS_WAKEUP_ID);
	/* If charger is not connected disable the interrupt */
	val &= ~VDAT_DET_INT_EN;
	val |= VDAT_DET_CHG_DET;
	writel(val,base + USB_PHY_VBUS_WAKEUP_ID);

	val = readl(base + USB_PHY_VBUS_WAKEUP_ID);
	if (val & VDAT_DET_STS)
		status = true;
	else
		status = false;

	/* Disable charger detection logic */
	val = readl(base + UTMIP_BAT_CHRG_CFG0);
	val &= ~(UTMIP_OP_SRC_EN | UTMIP_ON_SINK_EN);
	writel(val, base + UTMIP_BAT_CHRG_CFG0);

	/* Delay of 40 ms before we pull the D+ as per battery charger spec */
	msleep(TDPSRC_CON_MS);

	return status;
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

/* disable walk and wake events after resume from LP0 */
bool tegra_usb_phy_is_remotewake_detected(struct tegra_usb_phy *phy)
{
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	void __iomem *pmc_base = IO_ADDRESS(TEGRA_PMC_BASE);
	void __iomem *base = phy->regs;
	unsigned  int inst = phy->instance;
	u32 val;

	val = readl(base + UTMIP_PMC_WAKEUP0);
	if (val & EVENT_INT_ENB) {
		val = readl(pmc_base + UTMIP_UHSIC_STATUS);
		if (UTMIP_WAKE_ALARM(inst) & val) {
			val = readl(pmc_base + PMC_SLEEP_CFG);
			val &= ~UTMIP_WAKE_VAL(inst, 0x0);
			val |= UTMIP_WAKE_VAL(inst, WAKE_VAL_NONE);
			writel(val, pmc_base + PMC_SLEEP_CFG);

			val = readl(pmc_base + PMC_TRIGGERS);
			val |= UTMIP_CLR_WAKE_ALARM(inst) |
				UTMIP_CLR_WALK_PTR(inst);
			writel(val, pmc_base + PMC_TRIGGERS);

			val = readl(base + UTMIP_PMC_WAKEUP0);
			val &= ~EVENT_INT_ENB;
			writel(val, base + UTMIP_PMC_WAKEUP0);
			phy->remote_wakeup = true;
			return true;
		}
	}
#endif
	return false;
}

