/*
 * linux/arch/arm/mach-tegra/include/mach/pinmux.h
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

#ifndef __MACH_TEGRA_PINMUX_H
#define __MACH_TEGRA_PINMUX_H

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
#include "pinmux-t2.h"
#else
#error "Undefined Tegra architecture"
#endif

typedef enum {
	TEGRA_MUX_RSVD = 0x8000,
	TEGRA_MUX_RSVD1 = 0x8000,
	TEGRA_MUX_RSVD2 = 0x8001,
	TEGRA_MUX_RSVD3 = 0x8002,
	TEGRA_MUX_RSVD4 = 0x8003,
	TEGRA_MUX_NONE = -1,
	TEGRA_MUX_AHB_CLK,
	TEGRA_MUX_APB_CLK,
	TEGRA_MUX_AUDIO_SYNC,
	TEGRA_MUX_CRT,
	TEGRA_MUX_DAP1,
	TEGRA_MUX_DAP2,
	TEGRA_MUX_DAP3,
	TEGRA_MUX_DAP4,
	TEGRA_MUX_DAP5,
	TEGRA_MUX_DISPLAYA,
	TEGRA_MUX_DISPLAYB,
	TEGRA_MUX_EMC_TEST0_DLL,
	TEGRA_MUX_EMC_TEST1_DLL,
	TEGRA_MUX_GMI,
	TEGRA_MUX_GMI_INT,
	TEGRA_MUX_HDMI,
	TEGRA_MUX_I2C,
	TEGRA_MUX_I2C2,
	TEGRA_MUX_I2C3,
	TEGRA_MUX_IDE,
	TEGRA_MUX_IRDA,
	TEGRA_MUX_KBC,
	TEGRA_MUX_MIO,
	TEGRA_MUX_MIPI_HS,
	TEGRA_MUX_NAND,
	TEGRA_MUX_OSC,
	TEGRA_MUX_OWR,
	TEGRA_MUX_PCIE,
	TEGRA_MUX_PLLA_OUT,
	TEGRA_MUX_PLLC_OUT1,
	TEGRA_MUX_PLLM_OUT1,
	TEGRA_MUX_PLLP_OUT2,
	TEGRA_MUX_PLLP_OUT3,
	TEGRA_MUX_PLLP_OUT4,
	TEGRA_MUX_PWM,
	TEGRA_MUX_PWR_INTR,
	TEGRA_MUX_PWR_ON,
	TEGRA_MUX_RTCK,
	TEGRA_MUX_SDIO1,
	TEGRA_MUX_SDIO2,
	TEGRA_MUX_SDIO3,
	TEGRA_MUX_SDIO4,
	TEGRA_MUX_SFLASH,
	TEGRA_MUX_SPDIF,
	TEGRA_MUX_SPI1,
	TEGRA_MUX_SPI2,
	TEGRA_MUX_SPI2_ALT,
	TEGRA_MUX_SPI3,
	TEGRA_MUX_SPI4,
	TEGRA_MUX_TRACE,
	TEGRA_MUX_TWC,
	TEGRA_MUX_UARTA,
	TEGRA_MUX_UARTB,
	TEGRA_MUX_UARTC,
	TEGRA_MUX_UARTD,
	TEGRA_MUX_UARTE,
	TEGRA_MUX_ULPI,
	TEGRA_MUX_VI,
	TEGRA_MUX_VI_SENSOR_CLK,
	TEGRA_MUX_XIO,
	TEGRA_MAX_MUX,
} tegra_mux_func_t;

typedef enum {
	TEGRA_PUPD_NORMAL = 0,
	TEGRA_PUPD_PULL_DOWN,
	TEGRA_PUPD_PULL_UP,
} tegra_pullupdown_t;

typedef enum {
	TEGRA_TRI_NORMAL = 0,
	TEGRA_TRI_TRISTATE = 1,
} tegra_tristate_t;

typedef enum {
	TEGRA_VDDIO_BB = 0,
	TEGRA_VDDIO_LCD,
	TEGRA_VDDIO_VI,
	TEGRA_VDDIO_UART,
	TEGRA_VDDIO_DDR,
	TEGRA_VDDIO_NAND,
	TEGRA_VDDIO_SYS,
	TEGRA_VDDIO_AUDIO,
	TEGRA_VDDIO_SD,
} tegra_vddio_t;

struct tegra_pingroup_config {
	tegra_pingroup_t	pingroup;
	tegra_mux_func_t	func;
	tegra_pullupdown_t	pupd;
	tegra_tristate_t	tristate;
};

struct tegra_pingroup_desc {
	const char *name;
	int funcs[4];
	int func_safe;
	int vddio;
	s16 tri_reg; 		/* offset into the TRISTATE_REG_* register bank */
	s16 mux_reg;		/* offset into the PIN_MUX_CTL_* register bank */
	s16 pupd_reg;		/* offset into the PULL_UPDOWN_REG_* register bank */
	s8 tri_bit; 		/* offset into the TRISTATE_REG_* register bit */
	s8 mux_bit;		/* offset into the PIN_MUX_CTL_* register bit */
	s8 pupd_bit;		/* offset into the PULL_UPDOWN_REG_* register bit */
};

//20100724 byoungwoo.yoon@lge.com for gpio setting while sleep [LGE_START]
#define APPLY_SLEEP_GPIO_TABLE	1
#define SLEEP_GPIO_LOG	0
#define APPLY_GPIO_INIT	0

typedef enum {
	TRISTATE = 0,
	PIN_MUX_CTL,
	PULLUPDOWN,
	REG_DATA,
} tegra_reg_t;

extern int gpio_dbgfs_mode;   // 0=normal, 1=sleep

#define NORMAL_MODE 		0
#define SLEEP_MODE 			1

#define TRISTATE_REG_A         0x14
#define TRISTATE_REG_NUM       4
#define PIN_MUX_CTL_REG_A      0x80
#define PIN_MUX_CTL_REG_NUM    8
#define PULLUPDOWN_REG_A       0xa0
#define PULLUPDOWN_REG_NUM     5


#define OFFSET_TRISTATE_REG	   0
#define OFFSET_PIN_MUX_CTL	   TRISTATE_REG_NUM
#define OFFSET_PULLUPDOWN_CTL  (TRISTATE_REG_NUM + PIN_MUX_CTL_REG_NUM)

unsigned long get_reg_data( int pg, int reg );
void set_reg_data( int pg, long data, int reg );
static inline unsigned long pg_readl(unsigned long offset);
static inline void pg_writel(unsigned long value, unsigned long offset);
//20100724 byoungwoo.yoon@lge.com for gpio setting while sleep [LGE_END]

int tegra_pinmux_set_tristate(tegra_pingroup_t pg, tegra_tristate_t tristate);
int tegra_pinmux_set_pullupdown(tegra_pingroup_t pg, tegra_pullupdown_t pupd);

void tegra_pinmux_init_pingroups(void);
const struct tegra_pingroup_desc* tegra_pinmux_get_pingroups(void);

void tegra_pinmux_config_table(const struct tegra_pingroup_config *config, int len);

void tegra_pinmux_config_pinmux_table(const struct tegra_pingroup_config *config,
				      int len, bool is_set);
void tegra_pinmux_config_tristate_table(const struct tegra_pingroup_config *config,
					int len, tegra_tristate_t tristate);
void tegra_pinmux_config_pullupdown_table(const struct tegra_pingroup_config *config,
					  int len, tegra_pullupdown_t pupd);
int tegra_pinmux_get_vddio(tegra_pingroup_t pg);
void tegra_pinmux_set_vddio_tristate(tegra_vddio_t vddio,
				     tegra_tristate_t tristate);
#endif
