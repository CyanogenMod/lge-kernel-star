/*
 * linux/arch/arm/mach-tegra/pinmux-t2-tables.c
 *
 * Common pinmux configurations for Tegra 2 SoCs
 *
 * Copyright (C) 2010 NVIDIA Corporation
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
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/string.h>

#include <mach/iomap.h>
#include <mach/pinmux.h>
#include "gpio-names.h"

#define _mux(pg_name, f)				\
	{						\
		.pingroup = TEGRA_PINGROUP_ ## pg_name,	\
		.func = TEGRA_MUX_ ## f,		\
	}

struct tegra_pingroup_list {
	const struct tegra_pingroup_config *config;
	size_t len;
};

#define pinmux_func(_module, _hasreset, _tbls)			     \
static const struct tegra_pingroup_config  *pinmux_##_module ( \
	int config, size_t *len)				     \
{								     \
	_tbls							     \
	return_config((_hasreset) ? 0 : 1);			     \
}

#define _config(ent)					\
	{						\
		.config = c_ ## ent,			\
		.len = ARRAY_SIZE(c_ ## ent )		\
	}

#define return_config(_min)					\
	do {							\
		int _idx = config - (_min);			\
		if (_idx<0 || _idx>=ARRAY_SIZE(list))		\
			return NULL;				\
		if (len) {					\
			*len = list[_idx].len;			\
		} 						\
		return list[_idx].config;			\
	} while (0);

#define def_config(ent, ...)						\
	static const struct tegra_pingroup_config c_##ent [] = { \
		__VA_ARGS__ }

#define def_list(...) \
	static const struct tegra_pingroup_list list[]  = { \
		__VA_ARGS__ }

pinmux_func(uart1, true,
	def_config(off, _mux(IRRX,UARTA), _mux(IRTX,UARTA),
		_mux(UAA,UARTA), _mux(UAB,UARTA),
		_mux(SDB,UARTA), _mux(SDD,UARTA));
	def_config(1, _mux(UAA,UARTA), _mux(UAB,UARTA));
	def_config(2, _mux(GPU,UARTA));
	def_config(3, _mux(IRRX,UARTA), _mux(IRTX,UARTA),
		_mux(UAD,UARTA));
	def_config(4, _mux(IRRX,UARTA), _mux(IRTX,UARTA));
	def_config(5, _mux(SDD,UARTA), _mux(SDB,UARTA));
	def_config(6, _mux(UAA,UARTA));
	def_config(7, _mux(SDIO1,UARTA));
	def_list(_config(off), _config(1), _config(2), _config(3),
		_config(4), _config(5),	_config(6), _config(7));
)

pinmux_func(uart2, true,
	def_config(off, _mux(UAD,IRDA));
	def_config(1, _mux(IRRX,UARTB), _mux(IRTX,UARTB), _mux(UAD,IRDA));
	def_config(2,_mux(UAD,IRDA));
	def_list(_config(off), _config(1), _config(2));
)

pinmux_func(uart3, true,
	def_config(off, _mux(UCA,UARTC),_mux(UCB,UARTC));
	def_config(1, _mux(UCA,UARTC),_mux(UCB,UARTC));
	def_config(2, _mux(UCA,UARTC));
	def_list(_config(off), _config(1), _config(2));
)

pinmux_func(uart4, true,
	def_config(off, _mux(GMC,UARTD));
	def_config(1, _mux(UDA,UARTD));
	def_config(2, _mux(GMC,UARTD));
	def_list(_config(off), _config(1), _config(2));
)

pinmux_func(uart5, true,
	def_config(off, _mux(GMA,UARTE));
	def_config(1, _mux(SDIO1,UARTE));
	def_config(2, _mux(GMA,UARTE));
	def_list(_config(off), _config(1), _config(2));
)

pinmux_func(spi1, true,
	def_config(off, _mux(UDA,SPI1), _mux(SPIA,SPI1),
		_mux(SPIB,SPI1), _mux(SPIC,SPI1));
	def_config(1, _mux(UDA,SPI1));
	def_config(2, _mux(DTE,SPI1), _mux(DTB,SPI1));
	def_config(3, _mux(SPIC,SPI1), _mux(SPIB,SPI1), _mux(SPIA,SPI1));
	def_config(4, _mux(SPIE,SPI1), _mux(SPIF,SPI1), _mux(SPID,SPI1));
	def_list(_config(off), _config(1), _config(2), _config(3), _config(4));
)

pinmux_func(spi2, true,
	def_config(off, _mux(UAB,SPI2),
		_mux(SPID,SPI2), _mux(SPIE,SPI2));
	def_config(1, _mux(UAB,SPI2));
	def_config(2, _mux(SPIC,SPI2), _mux(SPIB,SPI2),
		_mux(SPIA,SPI2), _mux(SPIG,SPI2), _mux(SPIH,SPI2));
	def_config(3, _mux(SPIE,SPI2_ALT), _mux(SPIF,SPI2), _mux(SPID,SPI2_ALT),
		_mux(SPIG,SPI2_ALT), _mux(SPIH,SPI2_ALT));
	def_config(4, _mux(SPIC,SPI2), _mux(SPIB,SPI2), _mux(SPIA,SPI2));
	def_config(5,_mux(SPIE,SPI2_ALT), _mux(SPIF,SPI2), _mux(SPID,SPI2_ALT));
	def_list(_config(off), _config(1), _config(2),
		_config(3), _config(4), _config(5));
)

pinmux_func(spi3, true,
	def_config(off, _mux(UAA,SPI3), _mux(SPIF,SPI3),
		_mux(SPIG,SPI3), _mux(SPIH,SPI3));
	def_config(1, _mux(UAA,SPI3));
	def_config(2, _mux(LSC1,SPI3), _mux(LPW2,SPI3),
		_mux(LPW0,SPI3), _mux(LM0,SPI3));
	def_config(3, _mux(LSCK,SPI3), _mux(LSDI,SPI3),
		_mux(LSDA,SPI3), _mux(LCSN,SPI3));
	def_config(4, _mux(GMA,SPI3));
	def_config(5, _mux(SPIC,SPI3), _mux(SPIB,SPI3), _mux(SPIA,SPI3));
	def_config(6, _mux(SDC,SPI3), _mux(SDD,SPI3));
	def_config(7, _mux(SPIA,SPI3), _mux(SPIF,SPI3),
		_mux(SPIG,SPI3), _mux(SPIH,SPI3));
	def_list(_config(off), _config(1), _config(2), _config(3),
		_config(4), _config(5), _config(6), _config(7));
)

pinmux_func(spi4, false,
	def_config(1, _mux(UAD,SPI4), _mux(IRRX,SPI4), _mux(IRTX,SPI4));
	def_config(2, _mux(GMC,SPI4));
	def_config(3, _mux(SLXC,SPI4), _mux(SLXK,SPI4),
		_mux(SLXA,SPI4), _mux(SLXD,SPI4));
	def_list(_config(1), _config(2), _config(3));
)

pinmux_func(sflash, false,
	def_config(1, _mux(GMD,SFLASH), _mux(GMC,SFLASH));
	def_list(_config(1));
)

pinmux_func(twc, false,
	def_config(1, _mux(DAP2,TWC));
	def_config(2, _mux(SDC,TWC));
	def_list(_config(1), _config(2));
)

pinmux_func(i2c1, true,
	def_config(off, _mux(RM,I2C));
	def_config(1, _mux(RM,I2C));
	def_config(2, _mux(SPDI,I2C), _mux(SPDO,I2C));
	def_config(3, _mux(SPIG,I2C), _mux(SPIH,I2C));
	def_list(_config(off), _config(1), _config(2), _config(3));
)

pinmux_func(i2c2, true,
	def_config(off, _mux(PTA,I2C2), _mux(DDC,I2C2));
	def_config(1, _mux(DDC,I2C2));
	def_config(2, _mux(PTA,I2C2));
	def_list(_config(off), _config(1), _config(2));
)

pinmux_func(i2c3, true,
	def_config(off, _mux(DTF,I2C3));
	def_config(1, _mux(DTF,I2C3));
	def_list(_config(off), _config(1));
)

pinmux_func(i2cp, true,
	def_config(off, _mux(I2CP,I2C));
	def_config(1, _mux(I2CP,I2C));
	def_list(_config(off), _config(1));
)

pinmux_func(ulpi, false,
	def_config(1, _mux(UAA,ULPI), _mux(UAB,ULPI), _mux(UDA,ULPI));
	def_list(_config(1));
)

pinmux_func(sdio1, true,
	def_config(off, _mux(SDIO1,SDIO1));
	def_config(1, _mux(SDIO1,SDIO1));
	def_list(_config(off), _config(1));
)

pinmux_func(sdio2, false,
	def_config(1, _mux(KBCD,SDIO2), _mux(KBCB,SDIO2));
	def_config(2, _mux(KBCD,SDIO2), _mux(KBCB,SDIO2), _mux(KBCA,SDIO2));
	def_config(3, _mux(DAP1,SDIO2), _mux(SPDI,SDIO2), _mux(SPDO,SDIO2));
	def_config(4, _mux(DTA,SDIO2), _mux(DTD,SDIO2));
	def_config(5, _mux(DTA,SDIO2), _mux(DTD,SDIO2));
	def_list(_config(1), _config(2), _config(3), _config(4), _config(5));
)

pinmux_func(sdio3, false,
	def_config(1, _mux(SDD,SDIO3), _mux(SDC,SDIO3), _mux(SDB,SDIO3),
		_mux(SLXK,SDIO3), _mux(SLXC,SDIO3),
		_mux(SLXD,SDIO3), _mux(SLXA,SDIO3));
	def_config(2, _mux(SDD,SDIO3), _mux(SDC,SDIO3), _mux(SDB,SDIO3));
	def_list(_config(1), _config(2));
)

pinmux_func(sdio4, false,
	def_config(1, _mux(ATC,SDIO4), _mux(ATD,SDIO4));
	def_config(2, _mux(ATB,SDIO4), _mux(GMA,SDIO4), _mux(GME,SDIO4));
	def_config(3, _mux(ATB,SDIO4), _mux(GMA,SDIO4));
	def_list(_config(1), _config(2), _config(3));
)

pinmux_func(spdif, true,
	def_config(off, _mux(SPDO,SPDIF), _mux(SPDI,SPDIF),
		_mux(SLXD,SPDIF), _mux(SLXC,SPDIF));
	def_config(1, _mux(SPDO,SPDIF), _mux(SPDI,SPDIF));
	def_config(2, _mux(SLXD,SPDIF), _mux(SLXC,SPDIF));
	def_config(3, _mux(UAD,SPDIF));
	def_list(_config(off), _config(1), _config(2), _config(3));
)

pinmux_func(hsi, false,
	def_config(1, _mux(UAA,MIPI_HS), _mux(UAB,MIPI_HS));
	def_list(_config(1));
)

pinmux_func(hdmi, true,
	def_config(off, _mux(HDINT,HDMI));
	def_config(1, _mux(HDINT,HDMI));
	def_list(_config(off), _config(1));
)

pinmux_func(pwm, true,
	def_config(off, _mux(GPU,PWM), _mux(SDC,PWM));
	def_config(1, _mux(GPU,PWM));
	def_config(2, _mux(UCB,PWM), _mux(SDD,PWM));
	def_config(3, _mux(UCB,PWM));
	def_config(4, _mux(SDD,PWM));
	def_config(5, _mux(SDC,PWM), _mux(SDD,PWM));
	def_config(6, _mux(SDC,PWM));
	def_list(_config(off), _config(1), _config(2), _config(3),
		_config(4), _config(5), _config(6));
)

pinmux_func(ata, false,
	def_config(1, _mux(ATA,IDE), _mux(ATB,IDE), _mux(ATC,IDE),
		_mux(ATD,IDE), _mux(ATE,IDE), _mux(GMB,IDE));
	def_list(_config(1));
)

pinmux_func(nand, true,
	def_config(off, _mux(ATA,NAND), _mux(ATB,NAND), _mux(ATC,NAND),
		_mux(ATD,NAND), _mux(ATE,NAND));
	def_config(1, _mux(ATA,NAND), _mux(ATB,NAND), _mux(ATC,NAND),
		_mux(ATD,NAND), _mux(ATE,NAND));
	def_config(2, _mux(ATA,NAND), _mux(ATB,NAND), _mux(ATC,NAND));
	def_config(3, _mux(KBCA,NAND), _mux(KBCB,NAND), _mux(KBCC,NAND),
		_mux(KBCD,NAND), _mux(KBCE,NAND), _mux(KBCF,NAND));
	def_config(4, _mux(ATC,NAND));
	def_list(_config(off), _config(1), _config(2), _config(3), _config(4));
)

pinmux_func(dap1, false,
	def_config(1, _mux(DAP1,DAP1));
	def_list(_config(1));
)

pinmux_func(dap2, false,
	def_config(1, _mux(DAP2,DAP2));
	def_list(_config(1));
)

pinmux_func(dap3, false,
	def_config(1, _mux(DAP3,DAP3));
	def_list(_config(1));
)

pinmux_func(dap4, false,
	def_config(1, _mux(DAP4,DAP4));
	def_list(_config(1));
)

pinmux_func(dap5, false,
	def_config(1, _mux(GME,DAP5));
	def_list(_config(1));
)

pinmux_func(kbc, false,
	def_config(1, _mux(KBCA,KBC), _mux(KBCB,KBC), _mux(KBCC,KBC),
		_mux(KBCF,KBC),	_mux(KBCD,KBC), _mux(KBCB,KBC));
	def_config(2, _mux(KBCA,KBC), _mux(KBCB,KBC), _mux(KBCC,KBC),
		_mux(KBCF,KBC), _mux(KBCD,KBC));
	def_config(3, _mux(KBCA,KBC), _mux(KBCB,KBC),
		_mux(KBCC,KBC), _mux(KBCF,KBC));
	def_config(4, _mux(KBCA,KBC), _mux(KBCB,KBC),
		_mux(KBCC,KBC), _mux(KBCF,KBC));
	def_list(_config(1), _config(2), _config(3), _config(4));
)

pinmux_func(hdcp, false,
	def_config(1, _mux(PTA,HDMI));
	def_config(2, _mux(LSCK,HDMI), _mux(LSDA,HDMI));
	def_config(3, _mux(LPW2,HDMI), _mux(LPW0,HDMI));
	def_config(4, _mux(LSC1,HDMI), _mux(LPW0,HDMI));
	def_list(_config(1), _config(2), _config(3), _config(4));
)

#define snor_common							\
	_mux(DAP4,GMI), _mux(DAP2,GMI), _mux(SPIA,GMI), _mux(SPIB,GMI), \
	_mux(SPIC,GMI), _mux(SPID,GMI), _mux(SPIE,GMI), _mux(ATA,GMI),  \
	_mux(ATC,GMI), _mux(ATD,GMI), _mux(ATE,GMI), _mux(GMD,GMI)

pinmux_func(snor, false,
	def_config(1, snor_common, _mux(GMB,GMI), _mux(ATB,GMI),
		_mux(GMC,GMI), _mux(GMA,GMI), _mux(GME,GMI), _mux(DAP1,GMI),
		_mux(IRRX,GMI), _mux(IRTX,GMI), _mux(UCA,GMI),
		_mux(UCB,GMI), _mux(GPU,GMI));
	def_config(2, snor_common, _mux(GMB,GMI), _mux(ATB,GMI),
		_mux(GMC,GMI), _mux(GMA,GMI), _mux(GME,GMI), _mux(DAP1,GMI));
	def_config(3, snor_common, _mux(GMB,GMI), _mux(ATB,GMI));
	def_config(4,snor_common, _mux(GMA,GMI), _mux(ATB,GMI),
		_mux(IRRX,GMI), _mux(IRTX,GMI), _mux(UCA,GMI),
		_mux(UCB,GMI), _mux(GPU,GMI));
	def_config(5,snor_common, _mux(GMB,GMI_INT), _mux(GMC,SFLASH));
	def_list(_config(1), _config(2), _config(3), _config(4), _config(5));
)

pinmux_func(mio, false,
	def_config(1, _mux(KBCF,MIO), _mux(KBCD,MIO), _mux(KBCB,MIO));
	def_list(_config(1));
)

pinmux_func(ext_clock1, false,
	def_config(1, _mux(CDEV1,PLLA_OUT));
	def_config(2, _mux(CDEV1,OSC));
	def_list(_config(1), _config(2));
)

pinmux_func(ext_clock2, false,
	def_config(1, _mux(CDEV2,AHB_CLK));
	def_config(2, _mux(CDEV2,OSC));
	def_config(3, _mux(CDEV2,PLLP_OUT4));
	def_list(_config(1), _config(2), _config(3));
)

pinmux_func(ext_clock3, false,
	def_config(1, _mux(CSUS,VI_SENSOR_CLK));
	def_list(_config(1));
)

pinmux_func(vi, false,
	def_config(1, _mux(DTA,VI), _mux(DTB,VI), _mux(DTC,VI),
		_mux(DTD,VI), _mux(DTE,VI), _mux(DTF,VI));
	def_config(2, _mux(DTA,VI), _mux(DTB,VI), _mux(DTC,VI),
		_mux(DTD,VI), _mux(DTE,VI));
	def_list(_config(1), _config(2));
)

#define disp_common(_x)							\
	_mux(LD0,_x), _mux(LD1,_x), _mux(LD2,_x), _mux(LD3,_x), _mux(LD4,_x), \
	_mux(LD5,_x), _mux(LD6,_x), _mux(LD7,_x), _mux(LD8,_x), _mux(LD9,_x), \
	_mux(LD10,_x), _mux(LD11,_x), _mux(LD12,_x), _mux(LD13,_x), \
	_mux(LD14,_x), _mux(LD15,_x), _mux(LD16,_x), _mux(LD17,_x), _mux(LSC0,_x)

pinmux_func(displaya, false,
	def_config(1, disp_common(DISPLAYA), _mux(LVS,DISPLAYA),
		_mux(LHS,DISPLAYA), _mux(LSPI,DISPLAYA), _mux(LHP1,DISPLAYA),
		_mux(LHP2,DISPLAYA), _mux(LVP1,DISPLAYA), _mux(LHP0,DISPLAYA),
		_mux(LDI,DISPLAYA), _mux(LPP,DISPLAYA));
	def_config(2, disp_common(DISPLAYA), _mux(LVS,DISPLAYA),
		_mux(LHS,DISPLAYA), _mux(LSPI,DISPLAYA));
	def_config(3, _mux(LHP1,DISPLAYA), _mux(LHP2,DISPLAYA),
		_mux(LVP1,DISPLAYA), _mux(LHP0,DISPLAYA), _mux(LDI,DISPLAYA),
		_mux(LPP,DISPLAYA), _mux(LPW0,DISPLAYA), _mux(LPW1,DISPLAYA),
		_mux(LPW2,DISPLAYA), _mux(LSC1,DISPLAYA),
		_mux(LM1,DISPLAYA), _mux(LVP0,DISPLAYA));
	def_config(4, _mux(LPW0,DISPLAYA), _mux(LPW2,DISPLAYA),
		_mux(LSC1,DISPLAYA), _mux(LM0,DISPLAYA), _mux(LVP0,DISPLAYA));
	def_config(5, disp_common(DISPLAYA), _mux(LSC1,DISPLAYA),
		_mux(LM1,DISPLAYA));
	def_config(6, disp_common(DISPLAYA), _mux(LDC,DISPLAYA),
		_mux(LSPI,DISPLAYA));
	def_list(_config(1), _config(2), _config(3), _config(4),
		_config(5), _config(6));
)

pinmux_func(displayb, false,
	def_config(1, disp_common(DISPLAYB), _mux(LVS,DISPLAYB),
		_mux(LHS,DISPLAYB), _mux(LSPI,DISPLAYB), _mux(LHP1,DISPLAYB),
		_mux(LHP2,DISPLAYB), _mux(LVP1,DISPLAYB), _mux(LHP0,DISPLAYB),
		_mux(LDI,DISPLAYB), _mux(LPP,DISPLAYB));
	def_config(2, disp_common(DISPLAYB), _mux(LVS,DISPLAYB),
		_mux(LHS,DISPLAYB), _mux(LSPI,DISPLAYB));
	def_config(3, _mux(LHP1,DISPLAYB), _mux(LHP2,DISPLAYB),
		_mux(LVP1,DISPLAYB), _mux(LHP0,DISPLAYB), _mux(LDI,DISPLAYB),
		_mux(LPP,DISPLAYB), _mux(LPW0,DISPLAYB), _mux(LPW1,DISPLAYB),
		_mux(LPW2,DISPLAYB), _mux(LSC1,DISPLAYB),
		_mux(LM1,DISPLAYB), _mux(LVP0,DISPLAYB));
	def_config(4, _mux(LPW0,DISPLAYB), _mux(LPW2,DISPLAYB),
		_mux(LSC1,DISPLAYB), _mux(LM0,DISPLAYB), _mux(LVP0,DISPLAYB));
	def_config(5, disp_common(DISPLAYB), _mux(LSC1,DISPLAYB),
		_mux(LM1,DISPLAYB));
	def_config(6, disp_common(DISPLAYB), _mux(LDC,DISPLAYB),
		_mux(LSPI,DISPLAYB));
	def_list(_config(1), _config(2), _config(3), _config(4),
		_config(5), _config(6));
)

pinmux_func(crt, false,
	def_config(1, _mux(CRTP,CRT));
	def_list(_config(1));
)

pinmux_func(etm, false,
	def_config(1, _mux(KBCF,TRACE), _mux(KBCB,SDIO2), _mux(KBCC,TRACE));
	def_list(_config(1));
)

pinmux_func(owr, true,
	def_config(off, _mux(OWC,OWR), _mux(UAC,OWR), _mux(GPU,PWM));
	def_config(1, _mux(OWC,OWR), _mux(UAC,OWR));
	def_config(2, _mux(OWC,OWR), _mux(GPU,PWM));
	def_config(3, _mux(OWC,OWR), _mux(KBCE,OWR));
	def_list(_config(off), _config(1), _config(2), _config(3));
)

pinmux_func(pcie, false,
	def_config(1, _mux(GPV,PCIE), _mux(SDC,PWM),
		_mux(SLXK,PCIE), _mux(SLXA,PCIE));
	def_list(_config(1));
)

pinmux_func(blight_d1p0, false,
	def_config(1, _mux(LPW0,DISPLAYA));
	def_config(2, _mux(LPW2,DISPLAYA));
	def_config(3, _mux(LM0,DISPLAYA));
	def_list(_config(1), _config(2), _config(3));
)

pinmux_func(blight_d1p1, false,
	def_config(1, _mux(LM1,DISPLAYA));
	def_config(2, _mux(LDC,DISPLAYA));
	def_config(3, _mux(LPW1,DISPLAYA));
	def_list(_config(1), _config(2), _config(3));
)

pinmux_func(blight_d2p0, false,
	def_config(1, _mux(LPW0,DISPLAYB));
	def_config(2, _mux(LPW2,DISPLAYB));
	def_config(3, _mux(LM0,DISPLAYB));
	def_list(_config(1), _config(2), _config(3));
)


pinmux_func(blight_d2p1, false,
	def_config(1, _mux(LM1,DISPLAYB));
	def_config(2, _mux(LDC,DISPLAYB));
	def_config(3, _mux(LPW1,DISPLAYB));
	def_list(_config(1), _config(2), _config(3));
)

struct module_pinmux {
	const char *name;
	const char *dev_id;
	const struct tegra_pingroup_config *(*pin_func)(int, size_t*);
};

#define PIN_MODULE(_name, _devid) 		\
	{			  		\
		.name = #_name,			\
		.dev_id = _devid,		\
		.pin_func = pinmux_ ## _name,	\
	}

static const struct module_pinmux module_list[] = {
	PIN_MODULE(uart1, "serial8250.0"), PIN_MODULE(uart1, "tegra_uart.0"),
	PIN_MODULE(uart2, "serial8250.1"), PIN_MODULE(uart2, "tegra_uart.1"),
	PIN_MODULE(uart3, "serial8250.2"), PIN_MODULE(uart3, "tegra_uart.2"),
	PIN_MODULE(uart4, "serial8250.3"), PIN_MODULE(uart4, "tegra_uart.3"),
	PIN_MODULE(uart5, "serial8250.4"), PIN_MODULE(uart5, "tegra_uart.4"),
	PIN_MODULE(i2cp, "tegra_i2c.0"),
	PIN_MODULE(i2c1, "tegra_i2c.1"),
	PIN_MODULE(i2c2, "tegra_i2c.2"), PIN_MODULE(i2c2, "nvec"),
	PIN_MODULE(i2c3, "tegra_i2c.3"),
	PIN_MODULE(nand, "tegra_nand"),
	PIN_MODULE(owr, "tegra_w1"),
	PIN_MODULE(spi1, "tegra_spi.0"),
	PIN_MODULE(spi2, "tegra_spi.1"),
	PIN_MODULE(spi3, "tegra_spi.2"),
	PIN_MODULE(spi4, "tegra_spi.3"),
	PIN_MODULE(sflash, "tegra_spi.4"),
	PIN_MODULE(pcie, "tegra_pcie"),
	PIN_MODULE(kbc, "tegra_kbc"),
	PIN_MODULE(sdio1, "tegra-sdhci.0"),
	PIN_MODULE(sdio2, "tegra-sdhci.1"),
	PIN_MODULE(sdio3, "tegra-sdhci.2"),
	PIN_MODULE(sdio4, "tegra-sdhci.3"),
	PIN_MODULE(ulpi, "tegra-ehci.1"),
	/* no drivers exist yet for modules below this line */
	PIN_MODULE(displaya, "displaya"),
	PIN_MODULE(displayb, "displayb"),
	PIN_MODULE(hdcp, "hdcp"),
	PIN_MODULE(etm, "etm"),
	PIN_MODULE(snor, "snor"),
	PIN_MODULE(dap1, "i2s.0"),
	PIN_MODULE(dap2, "i2s.1"),
	PIN_MODULE(dap3, "i2s.2"),
	PIN_MODULE(dap4, "i2s.3"),
	PIN_MODULE(dap5, "i2s.4"),
	PIN_MODULE(vi, "vi"),
	PIN_MODULE(ata, "ide"),
	PIN_MODULE(twc, "twc"),
	PIN_MODULE(crt, "crt"),
	PIN_MODULE(hdmi, "hdmi"),
	PIN_MODULE(mio, "mio"),
	PIN_MODULE(hsi, "hsi"),
	PIN_MODULE(pwm, "pwm"),
	PIN_MODULE(spdif, "spdif"),
	PIN_MODULE(ext_clock1, "extclk.0"),
	PIN_MODULE(ext_clock2, "extclk.1"),
	PIN_MODULE(ext_clock3, "extclk.2"),
	PIN_MODULE(blight_d1p0, "blight.d1.p0"),
	PIN_MODULE(blight_d1p1, "blight.d1.p1"),
	PIN_MODULE(blight_d2p0, "blight.d2.p0"),
	PIN_MODULE(blight_d2p1, "blight.d2.p1"),
};

#define gpio_pingroup(port, pin, pingroup) \
        [TEGRA_GPIO_P##port##pin]= TEGRA_PINGROUP_##pingroup

static const int gpio_pin_pingroup[] = {
	gpio_pingroup(A, 0, DTE),
	gpio_pingroup(A, 1, UCB),
	gpio_pingroup(A, 2, DAP2),
	gpio_pingroup(A, 3, DAP2),
	gpio_pingroup(A, 4, DAP2),
	gpio_pingroup(A, 5, DAP2),
	gpio_pingroup(A, 6, SDD),
	gpio_pingroup(A, 7, SDB),

	gpio_pingroup(B, 0, GMC),
	gpio_pingroup(B, 1, GMC),
	gpio_pingroup(B, 2, LPW0),
	gpio_pingroup(B, 3, LSC0),
	gpio_pingroup(B, 4, SDC),
	gpio_pingroup(B, 5, SDC),
	gpio_pingroup(B, 6, SDC),
	gpio_pingroup(B, 7, SDC),

	gpio_pingroup(C, 0, UCB),
	gpio_pingroup(C, 1, LPW1),
	gpio_pingroup(C, 2, UAD),
	gpio_pingroup(C, 3, UAD),
	gpio_pingroup(C, 4, RM),
	gpio_pingroup(C, 5, RM),
	gpio_pingroup(C, 6, LPW2),
	gpio_pingroup(C, 7, GMB),

	gpio_pingroup(D, 0, SLXK),
	gpio_pingroup(D, 1, SLXA),
	gpio_pingroup(D, 2, DTE),
	gpio_pingroup(D, 3, SLXC),
	gpio_pingroup(D, 4, SLXD),
	gpio_pingroup(D, 5, DTA),
	gpio_pingroup(D, 6, DTC),
	gpio_pingroup(D, 7, DTC),

	gpio_pingroup(E, 0, LD0),
	gpio_pingroup(E, 1, LD1),
	gpio_pingroup(E, 2, LD2),
	gpio_pingroup(E, 3, LD3),
	gpio_pingroup(E, 4, LD4),
	gpio_pingroup(E, 5, LD5),
	gpio_pingroup(E, 6, LD6),
	gpio_pingroup(E, 7, LD7),

	gpio_pingroup(F, 0, LD8),
	gpio_pingroup(F, 1, LD9),
	gpio_pingroup(F, 2, LD10),
	gpio_pingroup(F, 3, LD11),
	gpio_pingroup(F, 4, LD12),
	gpio_pingroup(F, 5, LD13),
	gpio_pingroup(F, 6, LD14),
	gpio_pingroup(F, 7, LD15),

	gpio_pingroup(G, 0, ATC),
	gpio_pingroup(G, 1, ATC),
	gpio_pingroup(G, 2, ATC),
	gpio_pingroup(G, 3, ATC),
	gpio_pingroup(G, 4, ATC),
	gpio_pingroup(G, 5, ATC),
	gpio_pingroup(G, 6, ATC),
	gpio_pingroup(G, 7, ATC),

	gpio_pingroup(H, 0, ATD),
	gpio_pingroup(H, 1, ATD),
	gpio_pingroup(H, 2, ATD),
	gpio_pingroup(H, 3, ATD),
	gpio_pingroup(H, 4, ATE),
	gpio_pingroup(H, 5, ATE),
	gpio_pingroup(H, 6, ATE),
	gpio_pingroup(H, 7, ATE),


	gpio_pingroup(I, 0, ATC),
	gpio_pingroup(I, 1, ATC),
	gpio_pingroup(I, 2, ATB),
	gpio_pingroup(I, 3, ATA),
	gpio_pingroup(I, 4, ATA),
	gpio_pingroup(I, 5, ATB),
	gpio_pingroup(I, 6, ATA),
	gpio_pingroup(I, 7, ATC),

	gpio_pingroup(J, 0, GMD),
	gpio_pingroup(J, 1, LSPI),
	gpio_pingroup(J, 2, GMD),
	gpio_pingroup(J, 3, LHS),
	gpio_pingroup(J, 4, LVS),
	gpio_pingroup(J, 5, IRTX),
	gpio_pingroup(J, 6, IRRX),
	gpio_pingroup(J, 7, GMC),

	gpio_pingroup(K, 0, ATC),
	gpio_pingroup(K, 1, ATC),
	gpio_pingroup(K, 2, ATC),
	gpio_pingroup(K, 3, ATC),
	gpio_pingroup(K, 4, ATC),
	gpio_pingroup(K, 5, SPDO),
	gpio_pingroup(K, 6, SPDI),
	gpio_pingroup(K, 7, GMC),

	gpio_pingroup(L, 0, DTD),
	gpio_pingroup(L, 1, DTD),
	gpio_pingroup(L, 2, DTD),
	gpio_pingroup(L, 3, DTD),
	gpio_pingroup(L, 4, DTD),
	gpio_pingroup(L, 5, DTD),
	gpio_pingroup(L, 6, DTD),
	gpio_pingroup(L, 7, DTD),

	gpio_pingroup(M, 0, LD16),
	gpio_pingroup(M, 1, LD17),
	gpio_pingroup(M, 2, LHP1),
	gpio_pingroup(M, 3, LHP2),
	gpio_pingroup(M, 4, LVP1),
	gpio_pingroup(M, 5, LHP0),
	gpio_pingroup(M, 6, LDI),
	gpio_pingroup(M, 7, LPP),

	gpio_pingroup(N, 0, DAP1),
	gpio_pingroup(N, 1, DAP1),
	gpio_pingroup(N, 2, DAP1),
	gpio_pingroup(N, 3, DAP1),
	gpio_pingroup(N, 4, LCSN),
	gpio_pingroup(N, 5, LSDA),
	gpio_pingroup(N, 6, LDC),
	gpio_pingroup(N, 7, HDINT),


	gpio_pingroup(O, 0, UAB),
	gpio_pingroup(O, 1, UAA),
	gpio_pingroup(O, 2, UAA),
	gpio_pingroup(O, 3, UAA),
	gpio_pingroup(O, 4, UAA),
	gpio_pingroup(O, 5, UAB),
	gpio_pingroup(O, 6, UAB),
	gpio_pingroup(O, 7, UAB),

	gpio_pingroup(P, 0, DAP3),
	gpio_pingroup(P, 1, DAP3),
	gpio_pingroup(P, 2, DAP3),
	gpio_pingroup(P, 3, DAP3),
	gpio_pingroup(P, 4, DAP4),
	gpio_pingroup(P, 5, DAP4),
	gpio_pingroup(P, 6, DAP4),
	gpio_pingroup(P, 7, DAP4),

	gpio_pingroup(Q, 0, KBCC),
	gpio_pingroup(Q, 1, KBCC),
	gpio_pingroup(Q, 2, KBCF),
	gpio_pingroup(Q, 3, KBCF),
	gpio_pingroup(Q, 4, KBCF),
	gpio_pingroup(Q, 5, KBCF),
	gpio_pingroup(Q, 6, KBCF),
	gpio_pingroup(Q, 7, KBCE),

	gpio_pingroup(R, 0, KBCA),
	gpio_pingroup(R, 1, KBCA),
	gpio_pingroup(R, 2, KBCA),
	gpio_pingroup(R, 3, KBCD),
	gpio_pingroup(R, 4, KBCD),
	gpio_pingroup(R, 5, KBCD),
	gpio_pingroup(R, 6, KBCD),
	gpio_pingroup(R, 7, KBCB),

	gpio_pingroup(S, 0, KBCB),
	gpio_pingroup(S, 1, KBCB),
	gpio_pingroup(S, 2, KBCB),
	gpio_pingroup(S, 3, KBCB),
	gpio_pingroup(S, 4, KBCB),
	gpio_pingroup(S, 5, KBCB),
	gpio_pingroup(S, 6, KBCB),
	gpio_pingroup(S, 7, KBCB),

	gpio_pingroup(T, 0, DTD),
	gpio_pingroup(T, 1, CSUS),
	gpio_pingroup(T, 2, DTB),
	gpio_pingroup(T, 3, DTB),
	gpio_pingroup(T, 4, DTA),
	gpio_pingroup(T, 5, PTA),
	gpio_pingroup(T, 6, PTA),
	gpio_pingroup(T, 7, ATB),

	gpio_pingroup(U, 0, GPU),
	gpio_pingroup(U, 1, GPU),
	gpio_pingroup(U, 2, GPU),
	gpio_pingroup(U, 3, GPU),
	gpio_pingroup(U, 4, GPU),
	gpio_pingroup(U, 5, GPU),
	gpio_pingroup(U, 6, GPU),
	gpio_pingroup(U, 7, GPU7),

	gpio_pingroup(V, 0, UAC),
	gpio_pingroup(V, 1, UAC),
	gpio_pingroup(V, 2, UAC),
	gpio_pingroup(V, 3, UAC),
	gpio_pingroup(V, 4, GPV),
	gpio_pingroup(V, 5, GPV),
	gpio_pingroup(V, 6, GPV),
	gpio_pingroup(V, 7, LVP0),

	gpio_pingroup(W, 0, LM0),
	gpio_pingroup(W, 1, LM1),
	gpio_pingroup(W, 2, SPIG),
	gpio_pingroup(W, 3, SPIH),
	gpio_pingroup(W, 4, CDEV1),
	gpio_pingroup(W, 5, CDEV2),
	gpio_pingroup(W, 6, UCA),
	gpio_pingroup(W, 7, UCA),

	gpio_pingroup(X, 0, SPIA),
	gpio_pingroup(X, 1, SPIB),
	gpio_pingroup(X, 2, SPIC),
	gpio_pingroup(X, 3, SPIC),
	gpio_pingroup(X, 4, SPID),
	gpio_pingroup(X, 5, SPIE),
	gpio_pingroup(X, 6, SPIE),
	gpio_pingroup(X, 7, SPIF),

	gpio_pingroup(Y, 0, UDA),
	gpio_pingroup(Y, 1, UDA),
	gpio_pingroup(Y, 2, UDA),
	gpio_pingroup(Y, 3, UDA),
	gpio_pingroup(Y, 4, SDIO1),
	gpio_pingroup(Y, 5, SDIO1),
	gpio_pingroup(Y, 6, SDIO1),
	gpio_pingroup(Y, 7, SDIO1),

	gpio_pingroup(Z, 0, SDIO1),
	gpio_pingroup(Z, 1, SDIO1),
	gpio_pingroup(Z, 2, LSDI),
	gpio_pingroup(Z, 3, LSC1),
	gpio_pingroup(Z, 4, LSCK),
	gpio_pingroup(Z, 5, PMC),
	gpio_pingroup(Z, 6, I2CP),
	gpio_pingroup(Z, 7, I2CP),

	gpio_pingroup(AA, 0, GMA),
	gpio_pingroup(AA, 1, GMA),
	gpio_pingroup(AA, 2, GMA),
	gpio_pingroup(AA, 3, GMA),
	gpio_pingroup(AA, 4, GME),
	gpio_pingroup(AA, 5, GME),
	gpio_pingroup(AA, 6, GME),
	gpio_pingroup(AA, 7, GME),


	gpio_pingroup(BB, 0, PMC),
	gpio_pingroup(BB, 1, DTE),
	gpio_pingroup(BB, 2, DTF),
	gpio_pingroup(BB, 3, DTF),
	gpio_pingroup(BB, 4, DTE),
	gpio_pingroup(BB, 5, DTE)
};

const struct tegra_pingroup_config *tegra_pinmux_get(const char *dev_id,
	int config, int *len)
{
	unsigned int i;

	for (i=0; i<ARRAY_SIZE(module_list); i++) {
		if (!strncmp(dev_id, module_list[i].dev_id,
		    strlen(module_list[i].dev_id))) {
			return module_list[i].pin_func(config, len);
		}
	}
	return NULL;
}

int gpio_get_pinmux_group(int gpio_nr)
{
	WARN_ON(gpio_nr >= ARRAY_SIZE(gpio_pin_pingroup) || gpio_nr < 0);
	if (gpio_nr >= ARRAY_SIZE(gpio_pin_pingroup) || gpio_nr < 0)
		return -EINVAL;
	return gpio_pin_pingroup[gpio_nr];
}

static const struct tegra_pingroup_desc pingroups[TEGRA_MAX_PINGROUP] = {
	PINGROUP(ATA,   NAND,  IDE,       NAND,      GMI,       RSVD,          IDE,       0x14, 0,  0x80, 24, 0xA0, 0),
	PINGROUP(ATB,   NAND,  IDE,       NAND,      GMI,       SDIO4,         IDE,       0x14, 1,  0x80, 16, 0xA0, 2),
	PINGROUP(ATC,   NAND,  IDE,       NAND,      GMI,       SDIO4,         IDE,       0x14, 2,  0x80, 22, 0xA0, 4),
	PINGROUP(ATD,   NAND,  IDE,       NAND,      GMI,       SDIO4,         IDE,       0x14, 3,  0x80, 20, 0xA0, 6),
	PINGROUP(ATE,   NAND,  IDE,       NAND,      GMI,       RSVD,          IDE,       0x18, 25, 0x80, 12, 0xA0, 8),
	PINGROUP(CDEV1, AUDIO, OSC,       PLLA_OUT,  PLLM_OUT1, AUDIO_SYNC,    OSC,       0x14, 4,  0x88, 2,  0xA8, 0),
	PINGROUP(CDEV2, AUDIO, OSC,       AHB_CLK,   APB_CLK,   PLLP_OUT4,     OSC,       0x14, 5,  0x88, 4,  0xA8, 2),
	PINGROUP(CRTP,  LCD,   CRT,       RSVD,      RSVD,      RSVD,          RSVD,      0x20, 14, 0x98, 20, 0xA4, 24),
	PINGROUP(CSUS,  VI,    PLLC_OUT1, PLLP_OUT2, PLLP_OUT3, VI_SENSOR_CLK, PLLC_OUT1, 0x14, 6,  0x88, 6,  0xAC, 24),
	PINGROUP(DAP1,  AUDIO, DAP1,      RSVD,      GMI,       SDIO2,         DAP1,      0x14, 7,  0x88, 20, 0xA0, 10),
	PINGROUP(DAP2,  AUDIO, DAP2,      TWC,       RSVD,      GMI,           DAP2,      0x14, 8,  0x88, 22, 0xA0, 12),
	PINGROUP(DAP3,  BB,    DAP3,      RSVD,      RSVD,      RSVD,          DAP3,      0x14, 9,  0x88, 24, 0xA0, 14),
	PINGROUP(DAP4,  UART,  DAP4,      RSVD,      GMI,       RSVD,          DAP4,      0x14, 10, 0x88, 26, 0xA0, 16),
	PINGROUP(DDC,   LCD,   I2C2,      RSVD,      RSVD,      RSVD,          RSVD4,     0x18, 31, 0x88, 0,  0xB0, 28),
	PINGROUP(DTA,   VI,    RSVD,      SDIO2,     VI,        RSVD,          RSVD4,     0x14, 11, 0x84, 20, 0xA0, 18),
	PINGROUP(DTB,   VI,    RSVD,      RSVD,      VI,        SPI1,          RSVD1,     0x14, 12, 0x84, 22, 0xA0, 20),
	PINGROUP(DTC,   VI,    RSVD,      RSVD,      VI,        RSVD,          RSVD1,     0x14, 13, 0x84, 26, 0xA0, 22),
	PINGROUP(DTD,   VI,    RSVD,      SDIO2,     VI,        RSVD,          RSVD1,     0x14, 14, 0x84, 28, 0xA0, 24),
	PINGROUP(DTE,   VI,    RSVD,      RSVD,      VI,        SPI1,          RSVD1,     0x14, 15, 0x84, 30, 0xA0, 26),
	PINGROUP(DTF,   VI,    I2C3,      RSVD,      VI,        RSVD,          RSVD4,     0x20, 12, 0x98, 30, 0xA0, 28),
	PINGROUP(GMA,   NAND,  UARTE,     SPI3,      GMI,       SDIO4,         SPI3,      0x14, 28, 0x84, 0,  0xB0, 20),
	PINGROUP(GMB,   NAND,  IDE,       NAND,      GMI,       GMI_INT,       GMI,       0x18, 29, 0x88, 28, 0xB0, 22),
	PINGROUP(GMC,   NAND,  UARTD,     SPI4,      GMI,       SFLASH,        SPI4,      0x14, 29, 0x84, 2,  0xB0, 24),
	PINGROUP(GMD,   NAND,  RSVD,      NAND,      GMI,       SFLASH,        GMI,       0x18, 30, 0x88, 30, 0xB0, 26),
	PINGROUP(GME,   NAND,  RSVD,      DAP5,      GMI,       SDIO4,         GMI,       0x18, 0,  0x8C, 0,  0xA8, 24),
	PINGROUP(GPU,   UART,  PWM,       UARTA,     GMI,       RSVD,          RSVD4,     0x14, 16, 0x8C, 4,  0xA4, 20),
	PINGROUP(GPU7,  SYS,   RTCK,      RSVD,      RSVD,      RSVD,          RTCK,      0x20, 11, 0x98, 28, 0xA4, 6),
	PINGROUP(GPV,   SD,    PCIE,      RSVD,      RSVD,      RSVD,          PCIE,      0x14, 17, 0x8C, 2,  0xA0, 30),
	PINGROUP(HDINT, LCD,   HDMI,      RSVD,      RSVD,      RSVD,          HDMI,      0x1C, 23, 0x84, 4,  0xAC, 22),
	PINGROUP(I2CP,  SYS,   I2C,       RSVD,      RSVD,      RSVD,          RSVD4,     0x14, 18, 0x88, 8,  0xA4, 2),
	PINGROUP(IRRX,  UART,  UARTA,     UARTB,     GMI,       SPI4,          UARTB,     0x14, 20, 0x88, 18, 0xA8, 22),
	PINGROUP(IRTX,  UART,  UARTA,     UARTB,     GMI,       SPI4,          UARTB,     0x14, 19, 0x88, 16, 0xA8, 20),
	PINGROUP(KBCA,  SYS,   KBC,       NAND,      SDIO2,     EMC_TEST0_DLL, KBC,       0x14, 22, 0x88, 10, 0xA4, 8),
	PINGROUP(KBCB,  SYS,   KBC,       NAND,      SDIO2,     MIO,           KBC,       0x14, 21, 0x88, 12, 0xA4, 10),
	PINGROUP(KBCC,  SYS,   KBC,       NAND,      TRACE,     EMC_TEST1_DLL, KBC,       0x18, 26, 0x88, 14, 0xA4, 12),
	PINGROUP(KBCD,  SYS,   KBC,       NAND,      SDIO2,     MIO,           KBC,       0x20, 10, 0x98, 26, 0xA4, 14),
	PINGROUP(KBCE,  SYS,   KBC,       NAND,      OWR,       RSVD,          KBC,       0x14, 26, 0x80, 28, 0xB0, 2),
	PINGROUP(KBCF,  SYS,   KBC,       NAND,      TRACE,     MIO,           KBC,       0x14, 27, 0x80, 26, 0xB0, 0),
	PINGROUP(LCSN,  LCD,   DISPLAYA,  DISPLAYB,  SPI3,      RSVD,          RSVD4,     0x1C, 31, 0x90, 12, 0xAC, 20),
	PINGROUP(LD0,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 0,  0x94, 0,  0xAC, 12),
	PINGROUP(LD1,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 1,  0x94, 2,  0xAC, 12),
	PINGROUP(LD10,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 10, 0x94, 20, 0xAC, 12),
	PINGROUP(LD11,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 11, 0x94, 22, 0xAC, 12),
	PINGROUP(LD12,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 12, 0x94, 24, 0xAC, 12),
	PINGROUP(LD13,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 13, 0x94, 26, 0xAC, 12),
	PINGROUP(LD14,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 14, 0x94, 28, 0xAC, 12),
	PINGROUP(LD15,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 15, 0x94, 30, 0xAC, 12),
	PINGROUP(LD16,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 16, 0x98, 0,  0xAC, 12),
	PINGROUP(LD17,  LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     0x1C, 17, 0x98, 2,  0xAC, 12),
	PINGROUP(LD2,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 2,  0x94, 4,  0xAC, 12),
	PINGROUP(LD3,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 3,  0x94, 6,  0xAC, 12),
	PINGROUP(LD4,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 4,  0x94, 8,  0xAC, 12),
	PINGROUP(LD5,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 5,  0x94, 10, 0xAC, 12),
	PINGROUP(LD6,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 6,  0x94, 12, 0xAC, 12),
	PINGROUP(LD7,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 7,  0x94, 14, 0xAC, 12),
	PINGROUP(LD8,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 8,  0x94, 16, 0xAC, 12),
	PINGROUP(LD9,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 9,  0x94, 18, 0xAC, 12),
	PINGROUP(LDC,   LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     0x1C, 30, 0x90, 14, 0xAC, 20),
	PINGROUP(LDI,   LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     0x20, 6,  0x98, 16, 0xAC, 18),
	PINGROUP(LHP0,  LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     0x1C, 18, 0x98, 10, 0xAC, 16),
	PINGROUP(LHP1,  LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     0x1C, 19, 0x98, 4,  0xAC, 14),
	PINGROUP(LHP2,  LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     0x1C, 20, 0x98, 6,  0xAC, 14),
	PINGROUP(LHS,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x20, 7,  0x90, 22, 0xAC, 22),
	PINGROUP(LM0,   LCD,   DISPLAYA,  DISPLAYB,  SPI3,      RSVD,          RSVD4,     0x1C, 24, 0x90, 26, 0xAC, 22),
	PINGROUP(LM1,   LCD,   DISPLAYA,  DISPLAYB,  RSVD,      CRT,           RSVD3,     0x1C, 25, 0x90, 28, 0xAC, 22),
	PINGROUP(LPP,   LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     0x20, 8,  0x98, 14, 0xAC, 18),
	PINGROUP(LPW0,  LCD,   DISPLAYA,  DISPLAYB,  SPI3,      HDMI,          DISPLAYA,  0x20, 3,  0x90, 0,  0xAC, 20),
	PINGROUP(LPW1,  LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     0x20, 4,  0x90, 2,  0xAC, 20),
	PINGROUP(LPW2,  LCD,   DISPLAYA,  DISPLAYB,  SPI3,      HDMI,          DISPLAYA,  0x20, 5,  0x90, 4,  0xAC, 20),
	PINGROUP(LSC0,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 27, 0x90, 18, 0xAC, 22),
	PINGROUP(LSC1,  LCD,   DISPLAYA,  DISPLAYB,  SPI3,      HDMI,          DISPLAYA,  0x1C, 28, 0x90, 20, 0xAC, 20),
	PINGROUP(LSCK,  LCD,   DISPLAYA,  DISPLAYB,  SPI3,      HDMI,          DISPLAYA,  0x1C, 29, 0x90, 16, 0xAC, 20),
	PINGROUP(LSDA,  LCD,   DISPLAYA,  DISPLAYB,  SPI3,      HDMI,          DISPLAYA,  0x20, 1,  0x90, 8,  0xAC, 20),
	PINGROUP(LSDI,  LCD,   DISPLAYA,  DISPLAYB,  SPI3,      RSVD,          DISPLAYA,  0x20, 2,  0x90, 6,  0xAC, 20),
	PINGROUP(LSPI,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       HDMI,          DISPLAYA,  0x20, 0,  0x90, 10, 0xAC, 22),
	PINGROUP(LVP0,  LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     0x1C, 21, 0x90, 30, 0xAC, 22),
	PINGROUP(LVP1,  LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     0x1C, 22, 0x98, 8,  0xAC, 16),
	PINGROUP(LVS,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     0x1C, 26, 0x90, 24, 0xAC, 22),
	PINGROUP(OWC,   SYS,   OWR,       RSVD,      RSVD,      RSVD,          OWR,       0x14, 31, 0x84, 8,  0xB0, 30),
	PINGROUP(PMC,   SYS,   PWR_ON,    PWR_INTR,  RSVD,      RSVD,          PWR_ON,    0x14, 23, 0x98, 18, -1,   -1),
	PINGROUP(PTA,   NAND,  I2C2,      HDMI,      GMI,       RSVD,          RSVD4,     0x14, 24, 0x98, 22, 0xA4, 4),
	PINGROUP(RM,    UART,  I2C,       RSVD,      RSVD,      RSVD,          RSVD4,     0x14, 25, 0x80, 14, 0xA4, 0),
	PINGROUP(SDB,   SD,    UARTA,     PWM,       SDIO3,     SPI2,          PWM,       0x20, 15, 0x8C, 10, -1,   -1),
	PINGROUP(SDC,   SD,    PWM,       TWC,       SDIO3,     SPI3,          TWC,       0x18, 1,  0x8C, 12, 0xAC, 28),
	PINGROUP(SDD,   SD,    UARTA,     PWM,       SDIO3,     SPI3,          PWM,       0x18, 2,  0x8C, 14, 0xAC, 30),
	PINGROUP(SDIO1, BB,    SDIO1,     RSVD,      UARTE,     UARTA,         RSVD2,     0x14, 30, 0x80, 30, 0xB0, 18),
	PINGROUP(SLXA,  SD,    PCIE,      SPI4,      SDIO3,     SPI2,          PCIE,      0x18, 3,  0x84, 6,  0xA4, 22),
	PINGROUP(SLXC,  SD,    SPDIF,     SPI4,      SDIO3,     SPI2,          SPI4,      0x18, 5,  0x84, 10, 0xA4, 26),
	PINGROUP(SLXD,  SD,    SPDIF,     SPI4,      SDIO3,     SPI2,          SPI4,      0x18, 6,  0x84, 12, 0xA4, 28),
	PINGROUP(SLXK,  SD,    PCIE,      SPI4,      SDIO3,     SPI2,          PCIE,      0x18, 7,  0x84, 14, 0xA4, 30),
	PINGROUP(SPDI,  AUDIO, SPDIF,     RSVD,      I2C,       SDIO2,         RSVD2,     0x18, 8,  0x8C, 8,  0xA4, 16),
	PINGROUP(SPDO,  AUDIO, SPDIF,     RSVD,      I2C,       SDIO2,         RSVD2,     0x18, 9,  0x8C, 6,  0xA4, 18),
	PINGROUP(SPIA,  AUDIO, SPI1,      SPI2,      SPI3,      GMI,           GMI,       0x18, 10, 0x8C, 30, 0xA8, 4),
	PINGROUP(SPIB,  AUDIO, SPI1,      SPI2,      SPI3,      GMI,           GMI,       0x18, 11, 0x8C, 28, 0xA8, 6),
	PINGROUP(SPIC,  AUDIO, SPI1,      SPI2,      SPI3,      GMI,           GMI,       0x18, 12, 0x8C, 26, 0xA8, 8),
	PINGROUP(SPID,  AUDIO, SPI2,      SPI1,      SPI2_ALT,  GMI,           GMI,       0x18, 13, 0x8C, 24, 0xA8, 10),
	PINGROUP(SPIE,  AUDIO, SPI2,      SPI1,      SPI2_ALT,  GMI,           GMI,       0x18, 14, 0x8C, 22, 0xA8, 12),
	PINGROUP(SPIF,  AUDIO, SPI3,      SPI1,      SPI2,      RSVD,          RSVD4,     0x18, 15, 0x8C, 20, 0xA8, 14),
	PINGROUP(SPIG,  AUDIO, SPI3,      SPI2,      SPI2_ALT,  I2C,           SPI2_ALT,  0x18, 16, 0x8C, 18, 0xA8, 16),
	PINGROUP(SPIH,  AUDIO, SPI3,      SPI2,      SPI2_ALT,  I2C,           SPI2_ALT,  0x18, 17, 0x8C, 16, 0xA8, 18),
	PINGROUP(UAA,   BB,    SPI3,      MIPI_HS,   UARTA,     ULPI,          MIPI_HS,   0x18, 18, 0x80, 0,  0xAC, 0),
	PINGROUP(UAB,   BB,    SPI2,      MIPI_HS,   UARTA,     ULPI,          MIPI_HS,   0x18, 19, 0x80, 2,  0xAC, 2),
	PINGROUP(UAC,   BB,    OWR,       RSVD,      RSVD,      RSVD,          RSVD4,     0x18, 20, 0x80, 4,  0xAC, 4),
	PINGROUP(UAD,   UART,  IRDA,      SPDIF,     UARTA,     SPI4,          SPDIF,     0x18, 21, 0x80, 6,  0xAC, 6),
	PINGROUP(UCA,   UART,  UARTC,     RSVD,      GMI,       RSVD,          RSVD4,     0x18, 22, 0x84, 16, 0xAC, 8),
	PINGROUP(UCB,   UART,  UARTC,     PWM,       GMI,       RSVD,          RSVD4,     0x18, 23, 0x84, 18, 0xAC, 10),
	PINGROUP(UDA,   BB,    SPI1,      RSVD,      UARTD,     ULPI,          RSVD2,     0x20, 13, 0x80, 8,  0xB0, 16),
	/* these pin groups only have pullup and pull down control */
	PINGROUP(CK32,  SYS,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      -1,   -1, -1,   -1, 0xB0, 14),
	PINGROUP(DDRC,  DDR,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      -1,   -1, -1,   -1, 0xAC, 26),
	PINGROUP(PMCA,  SYS,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      -1,   -1, -1,   -1, 0xB0, 4),
	PINGROUP(PMCB,  SYS,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      -1,   -1, -1,   -1, 0xB0, 6),
	PINGROUP(PMCC,  SYS,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      -1,   -1, -1,   -1, 0xB0, 8),
	PINGROUP(PMCD,  SYS,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      -1,   -1, -1,   -1, 0xB0, 10),
	PINGROUP(PMCE,  SYS,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      -1,   -1, -1,   -1, 0xB0, 12),
	PINGROUP(XM2C,  DDR,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      -1,   -1, -1,   -1, 0xA8, 30),
	PINGROUP(XM2D,  DDR,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      -1,   -1, -1,   -1, 0xA8, 28),
};

const struct tegra_pingroup_desc* tegra_pinmux_get_pingroups(void) {
	return pingroups;
}

#ifdef CONFIG_PM
//20100724 byoungwoo.yoon@lge.com move to "pinmux.h"
/*
#define TRISTATE_REG_A         0x14
#define TRISTATE_REG_NUM       4
#define PIN_MUX_CTL_REG_A      0x80
#define PIN_MUX_CTL_REG_NUM    8
#define PULLUPDOWN_REG_A       0xa0
#define PULLUPDOWN_REG_NUM     5
*/

#define PADCTRL_REG            0x868
#define PADCTRL_REG_NUM        42

static u32 pinmux_reg[TRISTATE_REG_NUM + PIN_MUX_CTL_REG_NUM +
                     PULLUPDOWN_REG_NUM + PADCTRL_REG_NUM];

#if 1
u32 sleep_pinmux_reg[TRISTATE_REG_NUM + PIN_MUX_CTL_REG_NUM + PULLUPDOWN_REG_NUM] = 
{
	// TRISTATE		:   ( 0->normal,   1->tristate)
	0xffffffff,		//  TRISTATE[0]
	0xffffffff,		//  TRISTATE[1]
 	0xffffffff,		//  TRISTATE[2]
 	0xffffffff,		//  TRISTATE[3]
 	//PIN_MUX_CONTROL
 	0x02a32036,		//  PIN_MUX_CONTROL[0]
 	0xa8a01403,		//  PIN_MUX_CONTROL[1]
 	0xa00a00d5,		//  PIN_MUX_CONTROL[2]
 	0xfffaa943,		//  PIN_MUX_CONTROL[3]
 	0x00000000,		//  PIN_MUX_CONTROL[4]
 	0x00000000,		//  PIN_MUX_CONTROL[5]
 	0x00c00000,		//  PIN_MUX_CONTROL[6]
 	0x00000000,		//  PIN_MUX_CONTROL[7]
 	// PULL UP/DOWN   : ( 0->normal,  1-> pull Up,  2-> pull Down,  3-> RSVD )
 	0x215556aa,		//  PULLUPDOWN[0]
 	0x0000aa00,		//  PULLUPDOWN[1]
 	0x00aa6655,		//  PULLUPDOWN[2]
 	0xa1a55a8a,		//  PULLUPDOWN[3]
 	0xa000200a,		//  PULLUPDOWN[4]
};
#else
//20100724 byoungwoo.yoon@lge.com for gpio setting while sleep [LGE_START]
static u32 sleep_pinmux_reg[TRISTATE_REG_NUM + PIN_MUX_CTL_REG_NUM + PULLUPDOWN_REG_NUM] = 
{
	
#if defined(STAR_COUNTRY_KR) && defined(STAR_OPERATOR_SKT)	// for SU660
	// TRISTATE		:   ( 0->normal,   1->tristate)
	0xf716fff9,		//  TRISTATE[0]
	0xdfe4bfdf,		//  TRISTATE[1]
 	0xffffffff,		//  TRISTATE[2]
 	0xff7fdbff,		//  TRISTATE[3]
 	//PIN_MUX_CONTROL
 	0x02a32036,		//  PIN_MUX_CONTROL[0]
 	0xa8a01403,		//  PIN_MUX_CONTROL[1]
 	0xa00a00d5,		//  PIN_MUX_CONTROL[2]
 	0xfffaa943,		//  PIN_MUX_CONTROL[3]
 	0x00000000,		//  PIN_MUX_CONTROL[4]
 	0x00000000,		//  PIN_MUX_CONTROL[5]
 	0x00c00000,		//  PIN_MUX_CONTROL[6]
 	0x00000000,		//  PIN_MUX_CONTROL[7]
 	// PULL UP/DOWN   : ( 0->normal,  1-> pull Up,  2-> pull Down,  3-> RSVD )
 	0x215556aa,		//  PULLUPDOWN[0]
 	0x0000aa00,		//  PULLUPDOWN[1]
 	0x00aa6655,		//  PULLUPDOWN[2]
 	0xa1a55a8a,		//  PULLUPDOWN[3]
 	0xa000200a,		//  PULLUPDOWN[4]
 	
#else	// for LGP990

	// TRISTATE		:   ( 0->normal,   1->tristate)
	0xf716fff9,		//  TRISTATE[0]
	0xdfe4bfdf,		//  TRISTATE[1]
 	0xffffffff,		//  TRISTATE[2]
 	0xff7fdbff,		//  TRISTATE[3]
 	//PIN_MUX_CONTROL
 	0x02a32036,		//  PIN_MUX_CONTROL[0]
 	0xa8a01403,		//  PIN_MUX_CONTROL[1]
 	0xa00a00d5,		//  PIN_MUX_CONTROL[2]
 	0xfffaa943,		//  PIN_MUX_CONTROL[3]
 	0x00000000,		//  PIN_MUX_CONTROL[4]
 	0x00000000,		//  PIN_MUX_CONTROL[5]
 	0x00c00000,		//  PIN_MUX_CONTROL[6]
 	0x00000000,		//  PIN_MUX_CONTROL[7]
 	// PULL UP/DOWN   : ( 0->normal,  1-> pull Up,  2-> pull Down,  3-> RSVD )
 	0x215556aa,		//  PULLUPDOWN[0]
 	0x0000aa00,		//  PULLUPDOWN[1]
 	0x00aa6655,		//  PULLUPDOWN[2]
 	0xa1a55a8a,		//  PULLUPDOWN[3]
 	0xa000200a,		//  PULLUPDOWN[4]
#endif 	
 
};
#endif

#if 0 // original
{
	// TRISTATE
	0xb11affe0,		//  TRISTATE[0]
	0x00e7ade8,		//  TRISTATE[1]
 	0xffdfffff,		//  TRISTATE[2]
 	0x000041ff,		//  TRISTATE[3]
 	//PIN_MUX_CONTROL
 	0x02a32036,		//  PIN_MUX_CONTROL[0]
 	0xa8a01403,		//  PIN_MUX_CONTROL[1]
 	0xa00a00d5,		//  PIN_MUX_CONTROL[2]
 	0xfffaa943,		//  PIN_MUX_CONTROL[3]
 	0x00000000,		//  PIN_MUX_CONTROL[4]
 	0x00000000,		//  PIN_MUX_CONTROL[5]
 	0x00c00000,		//  PIN_MUX_CONTROL[6]
 	0x00000000,		//  PIN_MUX_CONTROL[7]
 	// PULL UP/DOWN
 	0x215556aa,		//  PULLUPDOWN[0]
 	0x0000aa00,		//  PULLUPDOWN[1]
 	0x00aa6655,		//  PULLUPDOWN[2]
 	0xa1a55a8a,		//  PULLUPDOWN[3]
 	0xa000200a,		//  PULLUPDOWN[4]
};
#endif

unsigned long get_reg_data( int pg, int reg )
{
	unsigned long ret = 0;

	//printk("[PINMUX] get_reg_data start :: pg=%d, reg=%d \n", pg, reg);
	
	if ( gpio_dbgfs_mode == NORMAL_MODE )  
	{
		switch( reg ) {
		case TRISTATE:
			ret = (pg_readl(pingroups[pg].tri_reg) >> pingroups[pg].tri_bit) & 0x1;
			printk("[PINMUX] TRISTATE ::  tri_reg=%d, tri_bit=%d \n", pingroups[pg].tri_reg, pingroups[pg].tri_bit);
			break;
		case PIN_MUX_CTL:
			ret = (pg_readl(pingroups[pg].mux_reg) >> pingroups[pg].mux_bit) & 0x1;
			printk("[PINMUX] PIN_MUX_CTL ::  mux_reg=%d, mux_bit=%d \n", pingroups[pg].mux_reg, pingroups[pg].mux_bit);
			break;
		case PULLUPDOWN:
			ret = (pg_readl(pingroups[pg].pupd_reg) >> pingroups[pg].pupd_bit) & 0x1;
			printk("[PINMUX] PULLUPDOWN ::  pupd_reg=%d, pupd_bit=%d \n", pingroups[pg].pupd_reg, pingroups[pg].pupd_bit);
			break;
		case REG_DATA:
			ret = pinmux_reg[pg]; 
			printk("[PINMUX] REG_DATA ::  data=%d \n", ret);
			break;
		}
	}
	else
	{
		switch( reg ) {
		case TRISTATE:
			ret = (sleep_pinmux_reg[(pingroups[pg].tri_reg-TRISTATE_REG_A)/4] 
				>> pingroups[pg].tri_bit) & 0x1;
			printk("[PINMUX] TRISTATE ::  tri_reg=%d, tri_bit=%d \n", pingroups[pg].tri_reg, pingroups[pg].tri_bit);
			break;
		case PIN_MUX_CTL:
			ret = (sleep_pinmux_reg[(pingroups[pg].mux_reg-PIN_MUX_CTL_REG_A)/4 + OFFSET_PIN_MUX_CTL] 
				>> pingroups[pg].mux_bit) & 0x1;
			printk("[PINMUX] PIN_MUX_CTL ::  mux_reg=%d, mux_bit=%d \n", pingroups[pg].mux_reg, pingroups[pg].mux_bit);
			break;
		case PULLUPDOWN:
			ret = (sleep_pinmux_reg[(pingroups[pg].pupd_reg-PULLUPDOWN_REG_A)/4 + TRISTATE_REG_NUM + PIN_MUX_CTL_REG_NUM] 
				>> pingroups[pg].pupd_bit) & 0x1;
			printk("[PINMUX] PULLUPDOWN ::  pupd_reg=%d, pupd_bit=%d \n", pingroups[pg].pupd_reg, pingroups[pg].pupd_bit);
			break;
		case REG_DATA:
			ret = sleep_pinmux_reg[pg];
		}
	}

	return ret;
}

void set_reg_data( int pg, long data, int reg )
{
	long temp;
	
	if ( gpio_dbgfs_mode == NORMAL_MODE )  
	{
		switch( reg ) {
		case TRISTATE:
			tegra_pinmux_set_tristate(pg, data);
			break;
		case PIN_MUX_CTL:
			temp = pg_readl(pingroups[pg].mux_reg);
			temp &= ~(0x3 << pingroups[pg].mux_bit);
			temp |= data << pingroups[pg].mux_bit;
			pg_writel(temp, pingroups[pg].mux_reg);
			break;
		case PULLUPDOWN:
			tegra_pinmux_set_pullupdown(pg, data);
			break;
		}
	}
	else
	{
		switch( reg ) {
		case TRISTATE:
			temp = sleep_pinmux_reg[(pingroups[pg].tri_reg-TRISTATE_REG_A)/4 + OFFSET_TRISTATE_REG];
			printk("[PINMUX] TRISTATE ::  before value=0x%x, data=0x%x, shift=%d \n", temp, data, pingroups[pg].tri_bit);
			temp &= ~(0x1 << pingroups[pg].tri_bit);
			temp |= data << pingroups[pg].tri_bit;
			printk("[PINMUX] TRISTATE ::  after value=0x%x \n", temp);
			sleep_pinmux_reg[(pingroups[pg].tri_reg-TRISTATE_REG_A)/4 + OFFSET_TRISTATE_REG] = temp;
			break;
		case PIN_MUX_CTL:
			temp = sleep_pinmux_reg[(pingroups[pg].mux_reg-PIN_MUX_CTL_REG_A)/4 + OFFSET_PIN_MUX_CTL];
			printk("[PINMUX] PIN_MUX_CTL ::  before value=0x%x, data=0x%x, shift=%d \n", temp, data, pingroups[pg].mux_bit);
			temp &= ~(0x3 << pingroups[pg].mux_bit);
			temp |= data << pingroups[pg].mux_bit;
			printk("[PINMUX] PIN_MUX_CTL ::  after value=0x%x \n", temp);
			sleep_pinmux_reg[(pingroups[pg].mux_reg-PIN_MUX_CTL_REG_A)/4 + OFFSET_PIN_MUX_CTL]  = temp;
			break;
		case PULLUPDOWN:
			temp = sleep_pinmux_reg[(pingroups[pg].pupd_reg-PULLUPDOWN_REG_A)/4 + OFFSET_PULLUPDOWN_CTL];
			printk("[PINMUX] PULLUPDOWN ::  before value=0x%x, data=0x%x, shift=%d \n", temp, data, pingroups[pg].pupd_bit);
			temp &= ~(0x3 << pingroups[pg].pupd_bit);
			temp |= data << pingroups[pg].pupd_bit;
			printk("[PINMUX] PULLUPDOWN ::  after value=0x%x \n", temp);
			sleep_pinmux_reg[(pingroups[pg].pupd_reg-PULLUPDOWN_REG_A)/4 + OFFSET_PULLUPDOWN_CTL] = temp;
			break;
		case REG_DATA:
			printk("[PINMUX] REG_DATA :: (before) sleep_pinmux_reg[%d]=0x%x \n", pg, sleep_pinmux_reg[pg]);
			sleep_pinmux_reg[pg]=data;		
			printk("[PINMUX] REG_DATA :: (after) sleep_pinmux_reg[%d]=0x%x \n", pg, sleep_pinmux_reg[pg]);
		}
	}

}

//20100724 byoungwoo.yoon@lge.com for gpio setting while sleep [LGE_END]

static inline unsigned long pg_readl(unsigned long offset)
{
	return readl(IO_TO_VIRT(TEGRA_APB_MISC_BASE + offset));
}

static inline void pg_writel(unsigned long value, unsigned long offset)
{
	writel(value, IO_TO_VIRT(TEGRA_APB_MISC_BASE + offset));
}

void tegra_pinmux_suspend(void)
{
       unsigned int i;
       u32 *ctx = pinmux_reg;

       for (i=0; i<PIN_MUX_CTL_REG_NUM; i++)
               *ctx++ = pg_readl(PIN_MUX_CTL_REG_A + i*4);

       for (i=0; i<PULLUPDOWN_REG_NUM; i++)
               *ctx++ = pg_readl(PULLUPDOWN_REG_A + i*4);

       for (i=0; i<TRISTATE_REG_NUM; i++)
               *ctx++ = pg_readl(TRISTATE_REG_A + i*4);

       for (i=0; i<PADCTRL_REG_NUM; i++)
               *ctx++ = pg_readl(PADCTRL_REG + i*4);

#if SLEEP_GPIO_LOG
		ctx = pinmux_reg;
		pr_info("[POWER-Pinmux] <<<< Suspend PinMux Setting value (Before setting) [START] >>>>>  \n");
		for (i=0; i<TRISTATE_REG_NUM; i++)
			pr_info("[POWER-Pinmux] TRISTATE_REG [%d] = 0x%x !! \n", i, pg_readl(TRISTATE_REG_A + i*4) );
		for (i=0; i<PIN_MUX_CTL_REG_NUM; i++)
			pr_info("[POWER-Pinmux] PIN_MUX_CTL [%d] = 0x%x !! \n", i, pg_readl(PIN_MUX_CTL_REG_A + i*4) );
		for (i=0; i<PULLUPDOWN_REG_NUM; i++)
			pr_info("[POWER-Pinmux] PULLUPDOWN [%d] = 0x%x !! \n", i, pg_readl(PULLUPDOWN_REG_A + i*4) );
		pr_info("[POWER-Pinmux] <<<< Suspend PinMux Setting value [END] >>>>>  \n");
#endif

//20100724 byoungwoo.yoon@lge.com for gpio setting while sleep [LGE_START]
#if APPLY_SLEEP_GPIO_TABLE
		ctx = sleep_pinmux_reg;
		for (i=0; i<TRISTATE_REG_NUM; i++)	{
				//pr_info("[POWER-Pinmux] writing TRISTATE[%d] data = 0x%x \n", i, *ctx);
				pg_writel(*ctx++, TRISTATE_REG_A + i*4);
			}
		/*for (i=0; i<PIN_MUX_CTL_REG_NUM; i++)	{
				//pr_info("[POWER-Pinmux] writing PINMUX[%d] data = 0x%x \n", i, *ctx);
				pg_writel(*ctx++, PIN_MUX_CTL_REG_A + i*4);
			}
		for (i=0; i<PULLUPDOWN_REG_NUM; i++)	{
				//pr_info("[POWER-Pinmux] writing PUPD[%d] data = 0x%x \n", i, *ctx);
				pg_writel(*ctx++, PULLUPDOWN_REG_A + i*4);
			}*/
#endif
//20100724 byoungwoo.yoon@lge.com for gpio setting while sleep [LGE_END]

#if SLEEP_GPIO_LOG
		ctx = pinmux_reg;
		pr_info("[POWER-Pinmux] <<<< Suspend PinMux Setting value (after setting) [START] >>>>>  \n");
		for (i=0; i<TRISTATE_REG_NUM; i++)
			pr_info("[POWER-Pinmux] TRISTATE_REG [%d] = 0x%x !! \n", i, pg_readl(TRISTATE_REG_A + i*4) );
		for (i=0; i<PIN_MUX_CTL_REG_NUM; i++)
			pr_info("[POWER-Pinmux] PIN_MUX_CTL [%d] = 0x%x !! \n", i, pg_readl(PIN_MUX_CTL_REG_A + i*4) );
		for (i=0; i<PULLUPDOWN_REG_NUM; i++)
			pr_info("[POWER-Pinmux] PULLUPDOWN [%d] = 0x%x !! \n", i, pg_readl(PULLUPDOWN_REG_A + i*4) );
		pr_info("[POWER-Pinmux] <<<< Suspend PinMux Setting value [END] >>>>>  \n");
#endif
}

void tegra_pinmux_resume(void)
{
       unsigned int i;
       u32 *ctx = pinmux_reg;

       for (i=0; i<PIN_MUX_CTL_REG_NUM; i++)
               pg_writel(*ctx++, PIN_MUX_CTL_REG_A + i*4);

       for (i=0; i<PULLUPDOWN_REG_NUM; i++)
               pg_writel(*ctx++, PULLUPDOWN_REG_A + i*4);

       for (i=0; i<TRISTATE_REG_NUM; i++)
               pg_writel(*ctx++, TRISTATE_REG_A + i*4);

       for (i=PADCTRL_REG; i< PADCTRL_REG + 4 * PADCTRL_REG_NUM; i+=4, ctx++) {
               /* Skip DRAM pads */
               if (i == 0x8c8 || i == 0x8cc || i == 0x8d0 || i == 0x8d4 ||
                   i == 0x8d8 || i == 0x8e4 || i == 0x8e8)
                      continue;
               pg_writel(*ctx, i);
       }
}
#endif
