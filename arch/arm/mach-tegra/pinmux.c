/*
 * linux/arch/arm/mach-tegra/pinmux.c
 *
 * Copyright (C) 2010 Google, Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/io.h>

#include <mach/iomap.h>
#include <mach/pinmux.h>


#define TEGRA_TRI_STATE(x)	(0x14 + (4 * (x)))
#define TEGRA_PP_MUX_CTL(x)	(0x80 + (4 * (x)))
#define TEGRA_PP_PU_PD(x)	(0xa0 + (4 * (x)))

#define REG_A 0
#define REG_B 1
#define REG_C 2
#define REG_D 3
#define REG_E 4
#define REG_F 5
#define REG_G 6

#define REG_N -1

struct tegra_pingroup_desc {
	const char *name;
	int funcs[4];
	int func_safe;
	int vddio;
	s8 tri_reg; 		/* offset into the TRISTATE_REG_* register bank */
	s8 tri_bit; 		/* offset into the TRISTATE_REG_* register bit */
	s8 mux_reg;		/* offset into the PIN_MUX_CTL_* register bank */
	s8 mux_bit;		/* offset into the PIN_MUX_CTL_* register bit */
	s8 pupd_reg;		/* offset into the PULL_UPDOWN_REG_* register bank */
	s8 pupd_bit;		/* offset into the PULL_UPDOWN_REG_* register bit */
};

#define PINGROUP(pg_name, vdd, f0, f1, f2, f3, f_safe,		\
		 tri_r, tri_b, mux_r, mux_b, pupd_r, pupd_b)	\
	[TEGRA_PINGROUP_ ## pg_name] = {			\
		.name = #pg_name,				\
		.vddio = TEGRA_VDDIO_ ## vdd,			\
		.funcs = {					\
			TEGRA_MUX_ ## f0,			\
			TEGRA_MUX_ ## f1,			\
			TEGRA_MUX_ ## f2,			\
			TEGRA_MUX_ ## f3,			\
		},						\
		.func_safe = TEGRA_MUX_ ## f_safe,		\
		.tri_reg = REG_ ## tri_r,			\
		.tri_bit = tri_b,				\
		.mux_reg = REG_ ## mux_r,			\
		.mux_bit = mux_b,				\
		.pupd_reg = REG_ ## pupd_r,			\
		.pupd_bit = pupd_b,				\
	}

static const struct tegra_pingroup_desc pingroups[TEGRA_MAX_PINGROUP] = {
	PINGROUP(ATA,   NAND,  IDE,       NAND,      GMI,       RSVD,          IDE,       A, 0,  A, 24, A, 0),
	PINGROUP(ATB,   NAND,  IDE,       NAND,      GMI,       SDIO4,         IDE,       A, 1,  A, 16, A, 2),
	PINGROUP(ATC,   NAND,  IDE,       NAND,      GMI,       SDIO4,         IDE,       A, 2,  A, 22, A, 4),
	PINGROUP(ATD,   NAND,  IDE,       NAND,      GMI,       SDIO4,         IDE,       A, 3,  A, 20, A, 6),
	PINGROUP(ATE,   NAND,  IDE,       NAND,      GMI,       RSVD,          IDE,       B, 25, A, 12, A, 8),
	PINGROUP(CDEV1, AUDIO, OSC,       PLLA_OUT,  PLLM_OUT1, AUDIO_SYNC,    OSC,       A, 4,  C, 2,  C, 0),
	PINGROUP(CDEV2, AUDIO, OSC,       AHB_CLK,   APB_CLK,   PLLP_OUT4,     OSC,       A, 5,  C, 4,  C, 2),
	PINGROUP(CRTP,  LCD,   CRT,       RSVD,      RSVD,      RSVD,          RSVD,      D, 14, G, 20, B, 24),
	PINGROUP(CSUS,  VI,    PLLC_OUT1, PLLP_OUT2, PLLP_OUT3, VI_SENSOR_CLK, PLLC_OUT1, A, 6,  C, 6,  D, 24),
	PINGROUP(DAP1,  AUDIO, DAP1,      RSVD,      GMI,       SDIO2,         DAP1,      A, 7,  C, 20, A, 10),
	PINGROUP(DAP2,  AUDIO, DAP2,      TWC,       RSVD,      GMI,           DAP2,      A, 8,  C, 22, A, 12),
	PINGROUP(DAP3,  BB,    DAP3,      RSVD,      RSVD,      RSVD,          DAP3,      A, 9,  C, 24, A, 14),
	PINGROUP(DAP4,  UART,  DAP4,      RSVD,      GMI,       RSVD,          DAP4,      A, 10, C, 26, A, 16),
	PINGROUP(DDC,   LCD,   I2C2,      RSVD,      RSVD,      RSVD,          RSVD4,     B, 31, C, 0,  E, 28),
	PINGROUP(DTA,   VI,    RSVD,      SDIO2,     VI,        RSVD,          RSVD4,     A, 11, B, 20, A, 18),
	PINGROUP(DTB,   VI,    RSVD,      RSVD,      VI,        SPI1,          RSVD1,     A, 12, B, 22, A, 20),
	PINGROUP(DTC,   VI,    RSVD,      RSVD,      VI,        RSVD,          RSVD1,     A, 13, B, 26, A, 22),
	PINGROUP(DTD,   VI,    RSVD,      SDIO2,     VI,        RSVD,          RSVD1,     A, 14, B, 28, A, 24),
	PINGROUP(DTE,   VI,    RSVD,      RSVD,      VI,        SPI1,          RSVD1,     A, 15, B, 30, A, 26),
	PINGROUP(DTF,   VI,    I2C3,      RSVD,      VI,        RSVD,          RSVD4,     D, 12, G, 30, A, 28),
	PINGROUP(GMA,   NAND,  UARTE,     SPI3,      GMI,       SDIO4,         SPI3,      A, 28, B, 0,  E, 20),
	PINGROUP(GMB,   NAND,  IDE,       NAND,      GMI,       GMI_INT,       GMI,       B, 29, C, 28, E, 22),
	PINGROUP(GMC,   NAND,  UARTD,     SPI4,      GMI,       SFLASH,        SPI4,      A, 29, B, 2,  E, 24),
	PINGROUP(GMD,   NAND,  RSVD,      NAND,      GMI,       SFLASH,        GMI,       B, 30, C, 30, E, 26),
	PINGROUP(GME,   NAND,  RSVD,      DAP5,      GMI,       SDIO4,         GMI,       B, 0,  D, 0,  C, 24),
	PINGROUP(GPU,   UART,  PWM,       UARTA,     GMI,       RSVD,          RSVD4,     A, 16, D, 4,  B, 20),
	PINGROUP(GPU7,  SYS,   RTCK,      RSVD,      RSVD,      RSVD,          RTCK,      D, 11, G, 28, B, 6),
	PINGROUP(GPV,   SD,    PCIE,      RSVD,      RSVD,      RSVD,          PCIE,      A, 17, D, 2,  A, 30),
	PINGROUP(HDINT, LCD,   HDMI,      RSVD,      RSVD,      RSVD,          HDMI,      C, 23, B, 4,  D, 22),
	PINGROUP(I2CP,  SYS,   I2C,       RSVD,      RSVD,      RSVD,          RSVD4,     A, 18, C, 8,  B, 2),
	PINGROUP(IRRX,  UART,  UARTA,     UARTB,     GMI,       SPI4,          UARTB,     A, 20, C, 18, C, 22),
	PINGROUP(IRTX,  UART,  UARTA,     UARTB,     GMI,       SPI4,          UARTB,     A, 19, C, 16, C, 20),
	PINGROUP(KBCA,  SYS,   KBC,       NAND,      SDIO2,     EMC_TEST0_DLL, KBC,       A, 22, C, 10, B, 8),
	PINGROUP(KBCB,  SYS,   KBC,       NAND,      SDIO2,     MIO,           KBC,       A, 21, C, 12, B, 10),
	PINGROUP(KBCC,  SYS,   KBC,       NAND,      TRACE,     EMC_TEST1_DLL, KBC,       B, 26, C, 14, B, 12),
	PINGROUP(KBCD,  SYS,   KBC,       NAND,      SDIO2,     MIO,           KBC,       D, 10, G, 26, B, 14),
	PINGROUP(KBCE,  SYS,   KBC,       NAND,      OWR,       RSVD,          KBC,       A, 26, A, 28, E, 2),
	PINGROUP(KBCF,  SYS,   KBC,       NAND,      TRACE,     MIO,           KBC,       A, 27, A, 26, E, 0),
	PINGROUP(LCSN,  LCD,   DISPLAYA,  DISPLAYB,  SPI3,      RSVD,          RSVD4,     C, 31, E, 12, D, 20),
	PINGROUP(LD0,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 0,  F, 0,  D, 12),
	PINGROUP(LD1,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 1,  F, 2,  D, 12),
	PINGROUP(LD10,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 10, F, 20, D, 12),
	PINGROUP(LD11,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 11, F, 22, D, 12),
	PINGROUP(LD12,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 12, F, 24, D, 12),
	PINGROUP(LD13,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 13, F, 26, D, 12),
	PINGROUP(LD14,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 14, F, 28, D, 12),
	PINGROUP(LD15,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 15, F, 30, D, 12),
	PINGROUP(LD16,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 16, G, 0,  D, 12),
	PINGROUP(LD17,  LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     C, 17, G, 2,  D, 12),
	PINGROUP(LD2,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 2,  F, 4,  D, 12),
	PINGROUP(LD3,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 3,  F, 6,  D, 12),
	PINGROUP(LD4,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 4,  F, 8,  D, 12),
	PINGROUP(LD5,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 5,  F, 10, D, 12),
	PINGROUP(LD6,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 6,  F, 12, D, 12),
	PINGROUP(LD7,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 7,  F, 14, D, 12),
	PINGROUP(LD8,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 8,  F, 16, D, 12),
	PINGROUP(LD9,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 9,  F, 18, D, 12),
	PINGROUP(LDC,   LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     C, 30, E, 14, D, 20),
	PINGROUP(LDI,   LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     D, 6,  G, 16, D, 18),
	PINGROUP(LHP0,  LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     C, 18, G, 10, D, 16),
	PINGROUP(LHP1,  LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     C, 19, G, 4,  D, 14),
	PINGROUP(LHP2,  LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     C, 20, G, 6,  D, 14),
	PINGROUP(LHS,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     D, 7,  E, 22, D, 22),
	PINGROUP(LM0,   LCD,   DISPLAYA,  DISPLAYB,  SPI3,      RSVD,          RSVD4,     C, 24, E, 26, D, 22),
	PINGROUP(LM1,   LCD,   DISPLAYA,  DISPLAYB,  RSVD,      CRT,           RSVD3,     C, 25, E, 28, D, 22),
	PINGROUP(LPP,   LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     D, 8,  G, 14, D, 18),
	PINGROUP(LPW0,  LCD,   DISPLAYA,  DISPLAYB,  SPI3,      HDMI,          DISPLAYA,  D, 3,  E, 0,  D, 20),
	PINGROUP(LPW1,  LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     D, 4,  E, 2,  D, 20),
	PINGROUP(LPW2,  LCD,   DISPLAYA,  DISPLAYB,  SPI3,      HDMI,          DISPLAYA,  D, 5,  E, 4,  D, 20),
	PINGROUP(LSC0,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 27, E, 18, D, 22),
	PINGROUP(LSC1,  LCD,   DISPLAYA,  DISPLAYB,  SPI3,      HDMI,          DISPLAYA,  C, 28, E, 20, D, 20),
	PINGROUP(LSCK,  LCD,   DISPLAYA,  DISPLAYB,  SPI3,      HDMI,          DISPLAYA,  C, 29, E, 16, D, 20),
	PINGROUP(LSDA,  LCD,   DISPLAYA,  DISPLAYB,  SPI3,      HDMI,          DISPLAYA,  D, 1,  E, 8,  D, 20),
	PINGROUP(LSDI,  LCD,   DISPLAYA,  DISPLAYB,  SPI3,      RSVD,          DISPLAYA,  D, 2,  E, 6,  D, 20),
	PINGROUP(LSPI,  LCD,   DISPLAYA,  DISPLAYB,  XIO,       HDMI,          DISPLAYA,  D, 0,  E, 10, D, 22),
	PINGROUP(LVP0,  LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     C, 21, E, 30, D, 22),
	PINGROUP(LVP1,  LCD,   DISPLAYA,  DISPLAYB,  RSVD,      RSVD,          RSVD4,     C, 22, G, 8,  D, 16),
	PINGROUP(LVS,   LCD,   DISPLAYA,  DISPLAYB,  XIO,       RSVD,          RSVD4,     C, 26, E, 24, D, 22),
	PINGROUP(OWC,   SYS,   OWR,       RSVD,      RSVD,      RSVD,          OWR,       A, 31, B, 8,  E, 30),
	PINGROUP(PMC,   SYS,   PWR_ON,    PWR_INTR,  RSVD,      RSVD,          PWR_ON,    A, 23, G, 18, N, -1),
	PINGROUP(PTA,   NAND,  I2C2,      HDMI,      GMI,       RSVD,          RSVD4,     A, 24, G, 22, B, 4),
	PINGROUP(RM,    UART,  I2C,       RSVD,      RSVD,      RSVD,          RSVD4,     A, 25, A, 14, B, 0),
	PINGROUP(SDB,   SD,    UARTA,     PWM,       SDIO3,     SPI2,          PWM,       D, 15, D, 10, N, -1),
	PINGROUP(SDC,   SD,    PWM,       TWC,       SDIO3,     SPI3,          TWC,       B, 1,  D, 12, D, 28),
	PINGROUP(SDD,   SD,    UARTA,     PWM,       SDIO3,     SPI3,          PWM,       B, 2,  D, 14, D, 30),
	PINGROUP(SDIO1, BB,    SDIO1,     RSVD,      UARTE,     UARTA,         RSVD2,     A, 30, A, 30, E, 18),
	PINGROUP(SLXA,  SD,    PCIE,      SPI4,      SDIO3,     SPI2,          PCIE,      B, 3,  B, 6,  B, 22),
	PINGROUP(SLXC,  SD,    SPDIF,     SPI4,      SDIO3,     SPI2,          SPI4,      B, 5,  B, 10, B, 26),
	PINGROUP(SLXD,  SD,    SPDIF,     SPI4,      SDIO3,     SPI2,          SPI4,      B, 6,  B, 12, B, 28),
	PINGROUP(SLXK,  SD,    PCIE,      SPI4,      SDIO3,     SPI2,          PCIE,      B, 7,  B, 14, B, 30),
	PINGROUP(SPDI,  AUDIO, SPDIF,     RSVD,      I2C,       SDIO2,         RSVD2,     B, 8,  D, 8,  B, 16),
	PINGROUP(SPDO,  AUDIO, SPDIF,     RSVD,      I2C,       SDIO2,         RSVD2,     B, 9,  D, 6,  B, 18),
	PINGROUP(SPIA,  AUDIO, SPI1,      SPI2,      SPI3,      GMI,           GMI,       B, 10, D, 30, C, 4),
	PINGROUP(SPIB,  AUDIO, SPI1,      SPI2,      SPI3,      GMI,           GMI,       B, 11, D, 28, C, 6),
	PINGROUP(SPIC,  AUDIO, SPI1,      SPI2,      SPI3,      GMI,           GMI,       B, 12, D, 26, C, 8),
	PINGROUP(SPID,  AUDIO, SPI2,      SPI1,      SPI2_ALT,  GMI,           GMI,       B, 13, D, 24, C, 10),
	PINGROUP(SPIE,  AUDIO, SPI2,      SPI1,      SPI2_ALT,  GMI,           GMI,       B, 14, D, 22, C, 12),
	PINGROUP(SPIF,  AUDIO, SPI3,      SPI1,      SPI2,      RSVD,          RSVD4,     B, 15, D, 20, C, 14),
	PINGROUP(SPIG,  AUDIO, SPI3,      SPI2,      SPI2_ALT,  I2C,           SPI2_ALT,  B, 16, D, 18, C, 16),
	PINGROUP(SPIH,  AUDIO, SPI3,      SPI2,      SPI2_ALT,  I2C,           SPI2_ALT,  B, 17, D, 16, C, 18),
	PINGROUP(UAA,   BB,    SPI3,      MIPI_HS,   UARTA,     ULPI,          MIPI_HS,   B, 18, A, 0,  D, 0),
	PINGROUP(UAB,   BB,    SPI2,      MIPI_HS,   UARTA,     ULPI,          MIPI_HS,   B, 19, A, 2,  D, 2),
	PINGROUP(UAC,   BB,    OWR,       RSVD,      RSVD,      RSVD,          RSVD4,     B, 20, A, 4,  D, 4),
	PINGROUP(UAD,   UART,  IRDA,      SPDIF,     UARTA,     SPI4,          SPDIF,     B, 21, A, 6,  D, 6),
	PINGROUP(UCA,   UART,  UARTC,     RSVD,      GMI,       RSVD,          RSVD4,     B, 22, B, 16, D, 8),
	PINGROUP(UCB,   UART,  UARTC,     PWM,       GMI,       RSVD,          RSVD4,     B, 23, B, 18, D, 10),
	PINGROUP(UDA,   BB,    SPI1,      RSVD,      UARTD,     ULPI,          RSVD2,     D, 13, A, 8,  E, 16),
	/* these pin groups only have pullup and pull down control */
	PINGROUP(CK32,  SYS,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      N, -1,  N, -1,  E, 14),
	PINGROUP(DDRC,  DDR,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      N, -1,  N, -1,  D, 26),
	PINGROUP(PMCA,  SYS,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      N, -1,  N, -1,  E, 4),
	PINGROUP(PMCB,  SYS,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      N, -1,  N, -1,  E, 6),
	PINGROUP(PMCC,  SYS,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      N, -1,  N, -1,  E, 8),
	PINGROUP(PMCD,  SYS,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      N, -1,  N, -1,  E, 10),
	PINGROUP(PMCE,  SYS,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      N, -1,  N, -1,  E, 12),
	PINGROUP(XM2C,  DDR,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      N, -1,  N, -1,  C, 30),
	PINGROUP(XM2D,  DDR,   RSVD,      RSVD,      RSVD,      RSVD,          RSVD,      N, -1,  N, -1,  C, 28),
};

static char *tegra_mux_names[TEGRA_MAX_MUX] = {
	[TEGRA_MUX_AHB_CLK] = "AHB_CLK",
	[TEGRA_MUX_APB_CLK] = "APB_CLK",
	[TEGRA_MUX_AUDIO_SYNC] = "AUDIO_SYNC",
	[TEGRA_MUX_CRT] = "CRT",
	[TEGRA_MUX_DAP1] = "DAP1",
	[TEGRA_MUX_DAP2] = "DAP2",
	[TEGRA_MUX_DAP3] = "DAP3",
	[TEGRA_MUX_DAP4] = "DAP4",
	[TEGRA_MUX_DAP5] = "DAP5",
	[TEGRA_MUX_DISPLAYA] = "DISPLAYA",
	[TEGRA_MUX_DISPLAYB] = "DISPLAYB",
	[TEGRA_MUX_EMC_TEST0_DLL] = "EMC_TEST0_DLL",
	[TEGRA_MUX_EMC_TEST1_DLL] = "EMC_TEST1_DLL",
	[TEGRA_MUX_GMI] = "GMI",
	[TEGRA_MUX_GMI_INT] = "GMI_INT",
	[TEGRA_MUX_HDMI] = "HDMI",
	[TEGRA_MUX_I2C] = "I2C",
	[TEGRA_MUX_I2C2] = "I2C2",
	[TEGRA_MUX_I2C3] = "I2C3",
	[TEGRA_MUX_IDE] = "IDE",
	[TEGRA_MUX_IRDA] = "IRDA",
	[TEGRA_MUX_KBC] = "KBC",
	[TEGRA_MUX_MIO] = "MIO",
	[TEGRA_MUX_MIPI_HS] = "MIPI_HS",
	[TEGRA_MUX_NAND] = "NAND",
	[TEGRA_MUX_OSC] = "OSC",
	[TEGRA_MUX_OWR] = "OWR",
	[TEGRA_MUX_PCIE] = "PCIE",
	[TEGRA_MUX_PLLA_OUT] = "PLLA_OUT",
	[TEGRA_MUX_PLLC_OUT1] = "PLLC_OUT1",
	[TEGRA_MUX_PLLM_OUT1] = "PLLM_OUT1",
	[TEGRA_MUX_PLLP_OUT2] = "PLLP_OUT2",
	[TEGRA_MUX_PLLP_OUT3] = "PLLP_OUT3",
	[TEGRA_MUX_PLLP_OUT4] = "PLLP_OUT4",
	[TEGRA_MUX_PWM] = "PWM",
	[TEGRA_MUX_PWR_INTR] = "PWR_INTR",
	[TEGRA_MUX_PWR_ON] = "PWR_ON",
	[TEGRA_MUX_RTCK] = "RTCK",
	[TEGRA_MUX_SDIO1] = "SDIO1",
	[TEGRA_MUX_SDIO2] = "SDIO2",
	[TEGRA_MUX_SDIO3] = "SDIO3",
	[TEGRA_MUX_SDIO4] = "SDIO4",
	[TEGRA_MUX_SFLASH] = "SFLASH",
	[TEGRA_MUX_SPDIF] = "SPDIF",
	[TEGRA_MUX_SPI1] = "SPI1",
	[TEGRA_MUX_SPI2] = "SPI2",
	[TEGRA_MUX_SPI2_ALT] = "SPI2_ALT",
	[TEGRA_MUX_SPI3] = "SPI3",
	[TEGRA_MUX_SPI4] = "SPI4",
	[TEGRA_MUX_TRACE] = "TRACE",
	[TEGRA_MUX_TWC] = "TWC",
	[TEGRA_MUX_UARTA] = "UARTA",
	[TEGRA_MUX_UARTB] = "UARTB",
	[TEGRA_MUX_UARTC] = "UARTC",
	[TEGRA_MUX_UARTD] = "UARTD",
	[TEGRA_MUX_UARTE] = "UARTE",
	[TEGRA_MUX_ULPI] = "ULPI",
	[TEGRA_MUX_VI] = "VI",
	[TEGRA_MUX_VI_SENSOR_CLK] = "VI_SENSOR_CLK",
	[TEGRA_MUX_XIO] = "XIO",
};

static DEFINE_SPINLOCK(mux_lock);

static int tristate_refcount[TEGRA_MAX_PINGROUP];

static const char *pingroup_name(tegra_pingroup_t pg)
{
	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return "<UNKNOWN>";

	return pingroups[pg].name;
}

static const char *func_name(tegra_mux_func_t func)
{
	if (func == TEGRA_MUX_RSVD1)
		return "RSVD1";

	if (func == TEGRA_MUX_RSVD2)
		return "RSVD2";

	if (func == TEGRA_MUX_RSVD3)
		return "RSVD3";

	if (func == TEGRA_MUX_RSVD4)
		return "RSVD4";

	if (func == TEGRA_MUX_NONE)
		return "NONE";

	if (func < 0 || func >=  TEGRA_MAX_MUX)
		return "<UNKNOWN>";

	return tegra_mux_names[func];
}


static const char *tri_name(unsigned long val)
{
	return val ? "TRISTATE" : "NORMAL";
}

static const char *pupd_name(unsigned long val)
{
	switch (val) {
	case 0:
		return "NORMAL";

	case 1:
		return "PULL_DOWN";

	case 2:
		return "PULL_UP";

	default:
		return "RSVD";
	}
}


static inline unsigned long pg_readl(unsigned long offset)
{
	return readl(IO_TO_VIRT(TEGRA_APB_MISC_BASE + offset));
}

static inline void pg_writel(unsigned long value, unsigned long offset)
{
	writel(value, IO_TO_VIRT(TEGRA_APB_MISC_BASE + offset));
}

int tegra_pinmux_cancel_func(tegra_pingroup_t pg, tegra_mux_func_t func)
{
	int mux = -1;
	int mux_safe = -1;
	int i;
	unsigned long reg;
	unsigned long flags;

	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return -ERANGE;

	if (pingroups[pg].mux_reg == REG_N)
		return -EINVAL;

	if (func < 0)
		return -ERANGE;

	if (func & TEGRA_MUX_RSVD) {
		return 0;
	}

	for (i = 0; i < 4; i++) {
		if (pingroups[pg].funcs[i] == func) {
			mux = i;
			break;
		}
	}

	if (pingroups[pg].func_safe & TEGRA_MUX_RSVD) {
		mux_safe = pingroups[pg].func_safe & 3;
	} else {
		for (i = 0; i < 4; i++) {
			if (pingroups[pg].funcs[i] == pingroups[pg].func_safe) {
				mux_safe = i;
				break;
			}
		}
	}

	if (mux < 0 || mux_safe < 0)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(TEGRA_PP_MUX_CTL(pingroups[pg].mux_reg));

	if (((reg >> pingroups[pg].mux_bit) & 0x3) == mux) {
		reg &= ~(0x3 << pingroups[pg].mux_bit);
		reg |= mux_safe << pingroups[pg].mux_bit;
		pg_writel(reg, TEGRA_PP_MUX_CTL(pingroups[pg].mux_reg));
	}

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

int tegra_pinmux_get_vddio(tegra_pingroup_t pg)
{
	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return -EINVAL;
	return pingroups[pg].vddio;
}

int tegra_pinmux_set_func(tegra_pingroup_t pg, tegra_mux_func_t func)
{
	int mux = -1;
	int i;
	unsigned long reg;
	unsigned long flags;

	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return -ERANGE;

	if (pingroups[pg].mux_reg == REG_N)
		return -EINVAL;

	if (func < 0)
		return -ERANGE;

	if (func & TEGRA_MUX_RSVD) {
		mux = func & 0x3;
	} else {
		for (i = 0; i < 4; i++) {
			if (pingroups[pg].funcs[i] == func) {
				mux = i;
				break;
			}
		}
	}

	if (mux < 0)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(TEGRA_PP_MUX_CTL(pingroups[pg].mux_reg));
	reg &= ~(0x3 << pingroups[pg].mux_bit);
	reg |= mux << pingroups[pg].mux_bit;
	pg_writel(reg, TEGRA_PP_MUX_CTL(pingroups[pg].mux_reg));

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

int tegra_pinmux_get_tristate(tegra_pingroup_t pg)
{
	unsigned long reg;

	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return 0;

	if (pingroups[pg].tri_reg == REG_N)
		return 0;

	reg = pg_readl(TEGRA_TRI_STATE(pingroups[pg].tri_reg));
	if (reg & (1 << pingroups[pg].tri_bit))
		return 0;
	else
		return 1;
}

int tegra_pinmux_set_tristate(tegra_pingroup_t pg, tegra_tristate_t tristate)
{
	unsigned long reg;
	unsigned long flags;
	bool is_update  = false;

	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return -ERANGE;

	if (pingroups[pg].tri_reg == REG_N)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	if (tristate == TEGRA_TRI_NORMAL) {
		is_update = (tristate_refcount[pg] == 0);
		tristate_refcount[pg]++;
	} else {
		is_update = (tristate_refcount[pg] == 1);
		if (tristate_refcount[pg] > 0) {
			tristate_refcount[pg]--;
		}
	}
	if (is_update) {
		reg = pg_readl(TEGRA_TRI_STATE(pingroups[pg].tri_reg));
		reg &= ~(0x1 << pingroups[pg].tri_bit);
		if (tristate)
			reg |= 1 << pingroups[pg].tri_bit;
		pg_writel(reg, TEGRA_TRI_STATE(pingroups[pg].tri_reg));
	}

	spin_unlock_irqrestore(&mux_lock, flags);
	return 0;
}

int tegra_pinmux_set_pullupdown(tegra_pingroup_t pg, tegra_pullupdown_t pupd)
{
	unsigned long reg;
	unsigned long flags;

	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return -ERANGE;

	if (pingroups[pg].pupd_reg == REG_N)
		return -EINVAL;

	if (pupd != TEGRA_PUPD_NORMAL &&
	    pupd != TEGRA_PUPD_PULL_DOWN &&
	    pupd != TEGRA_PUPD_PULL_UP)
		return -EINVAL;


	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(TEGRA_PP_PU_PD(pingroups[pg].pupd_reg));
	reg &= ~(0x3 << pingroups[pg].pupd_bit);
	reg |= pupd << pingroups[pg].pupd_bit;
	pg_writel(reg, TEGRA_PP_PU_PD(pingroups[pg].pupd_reg));

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

void tegra_pinmux_config_pingroup(tegra_pingroup_t pingroup,
				 tegra_mux_func_t func,
				 tegra_pullupdown_t pupd,
				 tegra_tristate_t tristate)
{
	int err;

	if (pingroups[pingroup].mux_reg != REG_N) {
		err = tegra_pinmux_set_func(pingroup, func);
		if (err < 0)
			pr_err("pinmux: can't set pingroup %s func to %s: %d\n",
			       pingroup_name(pingroup), func_name(func), err);
	}

	if (pingroups[pingroup].pupd_reg != REG_N) {
		err = tegra_pinmux_set_pullupdown(pingroup, pupd);
		if (err < 0)
			pr_err("pinmux: can't set pingroup %s pullupdown to %s: %d\n",
			       pingroup_name(pingroup), pupd_name(pupd), err);
	}

	if (pingroups[pingroup].tri_reg != REG_N) {
		err = tegra_pinmux_set_tristate(pingroup, tristate);
		if (err < 0)
			pr_err("pinmux: can't set pingroup %s tristate to %s: %d\n",
			       pingroup_name(pingroup), tri_name(func), err);
	}
}



void tegra_pinmux_config_table(struct tegra_pingroup_config *config, int len)
{
	int i;

	for (i = 0; i < len; i++)
		tegra_pinmux_config_pingroup(config[i].pingroup,
					     config[i].func,
					     config[i].pupd,
					     config[i].tristate);
}

void tegra_pinmux_config_pinmux_table(const struct tegra_pingroup_config *config,
				      int len, bool is_set)
{
	int i;
	int err;
	tegra_pingroup_t pingroup;
	tegra_mux_func_t func;

	for (i = 0; i < len; i++) {
		pingroup = config[i].pingroup;
		func = config[i].func;
		if (pingroups[pingroup].mux_reg != REG_N) {
			if (is_set)
				err = tegra_pinmux_set_func(pingroup, func);
			else
				err = tegra_pinmux_cancel_func(pingroup, func);
			if (err < 0)
				pr_err("pinmux: can't set pingroup %s func"
					" to %s: %d\n",	pingroup_name(pingroup),
					func_name(func), err);
		}
	}
}

void tegra_pinmux_config_tristate_table(const struct tegra_pingroup_config *config,
					int len, tegra_tristate_t tristate)
{
	int i;
	int err;
	tegra_pingroup_t pingroup;

	for (i = 0; i < len; i++) {
		pingroup = config[i].pingroup;
		if (pingroups[pingroup].tri_reg != REG_N) {
			err = tegra_pinmux_set_tristate(pingroup, tristate);
			if (err < 0)
				pr_err("pinmux: can't set pingroup %s tristate"
					" to %s: %d\n",	pingroup_name(pingroup),
					tri_name(tristate), err);
		}
	}
}

void tegra_pinmux_set_vddio_tristate(tegra_vddio_t vddio,
				     tegra_tristate_t tristate)
{
	int pg;
	for (pg = TEGRA_PINGROUP_ATA; pg < TEGRA_MAX_PINGROUP; ++pg) {
		if (pingroups[pg].vddio == vddio &&
		    pingroups[pg].tri_reg != REG_N) {
			if (tegra_pinmux_set_tristate(pg, tristate)<0)
				pr_err("pinmux: can't set pingroup %s tristate"
				       " to %s\n", pingroup_name(pg),
				       tri_name(tristate));
		}
	}
}

void tegra_pinmux_config_pullupdown_table(const struct tegra_pingroup_config *config,
					  int len, tegra_pullupdown_t pupd)
{
	int i;
	int err;
	tegra_pingroup_t pingroup;

	for (i = 0; i < len; i++) {
		pingroup = config[i].pingroup;
		if (pingroups[pingroup].pupd_reg != REG_N) {
			err = tegra_pinmux_set_pullupdown(pingroup, pupd);
			if (err < 0)
				pr_err("pinmux: can't set pingroup %s pullupdown"
					" to %s: %d\n",	pingroup_name(pingroup),
					pupd_name(pupd), err);
		}
	}
}

#ifdef CONFIG_PM
#define TRISTATE_REG_A		0x14
#define TRISTATE_REG_NUM	4
#define PIN_MUX_CTL_REG_A	0x80
#define PIN_MUX_CTL_REG_NUM	8
#define PULLUPDOWN_REG_A	0xa0
#define PULLUPDOWN_REG_NUM	5

static u32 pinmux_reg[TRISTATE_REG_NUM + PIN_MUX_CTL_REG_NUM +
		      PULLUPDOWN_REG_NUM];

void tegra_pinmux_suspend(void)
{
	unsigned int i;
	u32 *ctx = pinmux_reg;

	for (i=0; i<TRISTATE_REG_NUM; i++)
		*ctx++ = pg_readl(TRISTATE_REG_A + i*4);

	for (i=0; i<PIN_MUX_CTL_REG_NUM; i++)
		*ctx++ = pg_readl(PIN_MUX_CTL_REG_A + i*4);

	for (i=0; i<PULLUPDOWN_REG_NUM; i++)
		*ctx++ = pg_readl(PULLUPDOWN_REG_A + i*4);
}

void tegra_pinmux_resume(void)
{
	unsigned int i;
	u32 *ctx = pinmux_reg;

	for (i=0; i<TRISTATE_REG_NUM; i++)
		pg_writel(*ctx++, TRISTATE_REG_A + i*4);

	for (i=0; i<PIN_MUX_CTL_REG_NUM; i++)
		pg_writel(*ctx++, PIN_MUX_CTL_REG_A + i*4);

	for (i=0; i<PULLUPDOWN_REG_NUM; i++)
		pg_writel(*ctx++, PULLUPDOWN_REG_A + i*4);
}
#endif

#ifdef	CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>

static void dbg_pad_field(struct seq_file *s, int len)
{
	seq_putc(s, ',');

	while (len-- > -1)
		seq_putc(s, ' ');
}

static int dbg_pinmux_show(struct seq_file *s, void *unused)
{
	int i;
	int len;

	for (i = 0; i < TEGRA_MAX_PINGROUP; i++) {
		unsigned long tri;
		unsigned long mux;
		unsigned long pupd;

		seq_printf(s, "\t{TEGRA_PINGROUP_%s", pingroups[i].name);
		len = strlen(pingroups[i].name);
		dbg_pad_field(s, 5 - len);

		if (pingroups[i].mux_reg == REG_N) {
			seq_printf(s, "TEGRA_MUX_NONE");
			len = strlen("NONE");
		} else {
			mux = (pg_readl(TEGRA_PP_MUX_CTL(pingroups[i].mux_reg)) >>
			       pingroups[i].mux_bit) & 0x3;
			if (pingroups[i].funcs[mux] == TEGRA_MUX_RSVD) {
				seq_printf(s, "TEGRA_MUX_RSVD%1lu", mux+1);
				len = 5;
			} else {
				seq_printf(s, "TEGRA_MUX_%s",
					   tegra_mux_names[pingroups[i].funcs[mux]]);
				len = strlen(tegra_mux_names[pingroups[i].funcs[mux]]);
			}
		}
		dbg_pad_field(s, 13-len);

		if (pingroups[i].mux_reg == REG_N) {
			seq_printf(s, "TEGRA_PUPD_NORMAL");
			len = strlen("NORMAL");
		} else {
			pupd = (pg_readl(TEGRA_PP_PU_PD(pingroups[i].pupd_reg)) >>
				pingroups[i].pupd_bit) & 0x3;
			seq_printf(s, "TEGRA_PUPD_%s", pupd_name(pupd));
			len = strlen(pupd_name(pupd));
		}
		dbg_pad_field(s, 9 - len);

		if (pingroups[i].tri_reg == REG_N) {
			seq_printf(s, "TEGRA_TRI_NORMAL");
		} else {
			tri = (pg_readl(TEGRA_TRI_STATE(pingroups[i].tri_reg)) >>
			       pingroups[i].tri_bit) & 0x1;

			seq_printf(s, "TEGRA_TRI_%s", tri_name(tri));
		}
		seq_printf(s, "},\n");
	}
	return 0;
}

static int dbg_pinmux_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_pinmux_show, &inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_pinmux_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init tegra_pinmux_debuginit(void)
{
	(void) debugfs_create_file("tegra_pinmux", S_IRUGO,
					NULL, NULL, &debug_fops);
	return 0;
}
late_initcall(tegra_pinmux_debuginit);
#endif
