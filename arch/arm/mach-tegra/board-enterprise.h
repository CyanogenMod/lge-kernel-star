/*
 * arch/arm/mach-tegra/board-enterprise.h
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#ifndef _MACH_TEGRA_BOARD_ENTERPRISE_H
#define _MACH_TEGRA_BOARD_ENTERPRISE_H

#include <mach/gpio.h>
#include <mach/irqs.h>

int enterprise_charge_init(void);
int enterprise_sdhci_init(void);
int enterprise_pinmux_init(void);
int enterprise_panel_init(void);
int enterprise_sensors_init(void);
int touch_init(void);
int enterprise_kbc_init(void);
int enterprise_emc_init(void);
int enterprise_regulator_init(void);
int enterprise_baseband_init(void);
int enterprise_suspend_init(void);

/* Touchscreen GPIO addresses   */
#ifdef CONFIG_TOUCHSCREEN_ATMEL_MT_T9
#define TOUCH_GPIO_IRQ_ATMEL_T9	TEGRA_GPIO_PH4
#define TOUCH_GPIO_RST_ATMEL_T9	TEGRA_GPIO_PH6
#endif

/*****************External GPIO tables ******************/
/* External peripheral gpio base. */
#define TPS80031_GPIO_BASE	TEGRA_NR_GPIOS
#define TPS80031_GPIO_REGEN1	(TPS80031_GPIO_BASE + 0)
#define TPS80031_GPIO_REGEN2	(TPS80031_GPIO_BASE + 1)
#define TPS80031_GPIO_SYSEN	(TPS80031_GPIO_BASE + 2)
#define TPS80031_GPIO_END	(TPS80031_GPIO_BASE + 3)

/*****************External Interrupt tables ******************/
/* External peripheral irq base */
#define TPS80031_IRQ_BASE	TEGRA_NR_IRQS
#define TPS80031_IRQ_END	(TPS80031_IRQ_BASE + 24)

/*****************Camera GPIOs ******************/
#define CAM_CSI_MUX_SEL_GPIO	TEGRA_GPIO_PM3
#define CAM_LDO_1V8_EN_L_GPIO	TEGRA_GPIO_PF1
#define CAM_LDO_2V8_EN_L_GPIO	TEGRA_GPIO_PM7
#define CAM1_RST_L_GPIO		TEGRA_GPIO_PM5 /*REAR RIGHT*/
#define CAM1_PWDN_GPIO		TEGRA_GPIO_PF3 /*REAR RIGHT*/
#define CAM2_RST_L_GPIO		TEGRA_GPIO_PF4 /*REAR LEFT*/
#define CAM2_PWDN_GPIO		TEGRA_GPIO_PF2 /*REAR LEFT*/
#define CAM3_RST_L_GPIO		TEGRA_GPIO_PM2 /*FRONT*/
#define CAM3_PWDN_GPIO		TEGRA_GPIO_PN4 /*FRONT*/
#define CAM_FLASH_EN_GPIO	TEGRA_GPIO_PBB3
#define CAM_FLASH_MAX_TORCH_AMP	7
#define CAM_FLASH_MAX_FLASH_AMP	7

#endif
