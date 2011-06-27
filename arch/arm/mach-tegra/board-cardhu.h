/*
 * arch/arm/mach-tegra/board-cardhu.h
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

#ifndef _MACH_TEGRA_BOARD_CARDHU_H
#define _MACH_TEGRA_BOARD_CARDHU_H

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/mfd/tps6591x.h>

/* Processor Board  ID */
#define BOARD_E1187   0x0B57
#define BOARD_E1186   0x0B56
#define BOARD_E1198   0x0B62
#define BOARD_E1291   0x0C5B
#define BOARD_PM267   0x0243
#define BOARD_PM269   0x0245

/* SKU Information */
#define SKU_DCDC_TPS62361_SUPPORT	0x1
#define SKU_SLT_ULPI_SUPPORT		0x2
#define SKU_T30S_SUPPORT		0x4

/* External peripheral act as gpio */
/* TPS6591x GPIOs */
#define TPS6591X_GPIO_BASE	TEGRA_NR_GPIOS
#define TPS6591X_GPIO_GP0	(TPS6591X_GPIO_BASE + 0)
#define TPS6591X_GPIO_GP1	(TPS6591X_GPIO_BASE + 1)
#define TPS6591X_GPIO_GP2	(TPS6591X_GPIO_BASE + 2)
#define TPS6591X_GPIO_GP3	(TPS6591X_GPIO_BASE + 3)
#define TPS6591X_GPIO_GP4	(TPS6591X_GPIO_BASE + 4)
#define TPS6591X_GPIO_GP5	(TPS6591X_GPIO_BASE + 5)
#define TPS6591X_GPIO_GP6	(TPS6591X_GPIO_BASE + 6)
#define TPS6591X_GPIO_GP7	(TPS6591X_GPIO_BASE + 7)
#define TPS6591X_GPIO_GP8	(TPS6591X_GPIO_BASE + 8)
#define TPS6591X_GPIO_END	(TPS6591X_GPIO_GP8 + 1)

/* PMU_TCA6416 GPIOs */
#define PMU_TCA6416_GPIO_BASE	(TPS6591X_GPIO_END)
#define PMU_TCA6416_GPIO_PORT00	(PMU_TCA6416_GPIO_BASE + 0)
#define PMU_TCA6416_GPIO_PORT01	(PMU_TCA6416_GPIO_BASE + 1)
#define PMU_TCA6416_GPIO_PORT02	(PMU_TCA6416_GPIO_BASE + 2)
#define PMU_TCA6416_GPIO_PORT03	(PMU_TCA6416_GPIO_BASE + 3)
#define PMU_TCA6416_GPIO_PORT04	(PMU_TCA6416_GPIO_BASE + 4)
#define PMU_TCA6416_GPIO_PORT05	(PMU_TCA6416_GPIO_BASE + 5)
#define PMU_TCA6416_GPIO_PORT06	(PMU_TCA6416_GPIO_BASE + 6)
#define PMU_TCA6416_GPIO_PORT07	(PMU_TCA6416_GPIO_BASE + 7)
#define PMU_TCA6416_GPIO_PORT10	(PMU_TCA6416_GPIO_BASE + 8)
#define PMU_TCA6416_GPIO_PORT11	(PMU_TCA6416_GPIO_BASE + 9)
#define PMU_TCA6416_GPIO_PORT12	(PMU_TCA6416_GPIO_BASE + 10)
#define PMU_TCA6416_GPIO_PORT13	(PMU_TCA6416_GPIO_BASE + 11)
#define PMU_TCA6416_GPIO_PORT14	(PMU_TCA6416_GPIO_BASE + 12)
#define PMU_TCA6416_GPIO_PORT15	(PMU_TCA6416_GPIO_BASE + 13)
#define PMU_TCA6416_GPIO_PORT16	(PMU_TCA6416_GPIO_BASE + 14)
#define PMU_TCA6416_GPIO_PORT17	(PMU_TCA6416_GPIO_BASE + 15)
#define PMU_TCA6416_GPIO_END	(PMU_TCA6416_GPIO_BASE + 16)

/* PMU_TCA6416 GPIO assignment */
#define EN_HSIC_GPIO				PMU_TCA6416_GPIO_PORT11 /* PMU_GPIO25 */
#define PM267_SMSC4640_HSIC_HUB_RESET_GPIO	PMU_TCA6416_GPIO_PORT17 /* PMU_GPIO31 */

/* CAM_TCA6416 GPIOs */
#define CAM_TCA6416_GPIO_BASE		PMU_TCA6416_GPIO_END
#define CAM1_PWR_DN_GPIO			CAM_TCA6416_GPIO_BASE + 0
#define CAM1_RST_L_GPIO				CAM_TCA6416_GPIO_BASE + 1
#define CAM1_AF_PWR_DN_L_GPIO		CAM_TCA6416_GPIO_BASE + 2
#define CAM1_LDO_SHUTDN_L_GPIO		CAM_TCA6416_GPIO_BASE + 3
#define CAM2_PWR_DN_GPIO			CAM_TCA6416_GPIO_BASE + 4
#define CAM2_RST_L_GPIO				CAM_TCA6416_GPIO_BASE + 5
#define CAM2_AF_PWR_DN_L_GPIO		CAM_TCA6416_GPIO_BASE + 6
#define CAM2_LDO_SHUTDN_L_GPIO		CAM_TCA6416_GPIO_BASE + 7
#define CAM_FRONT_PWR_DN_GPIO		CAM_TCA6416_GPIO_BASE + 8
#define CAM_FRONT_RST_L_GPIO		CAM_TCA6416_GPIO_BASE + 9
#define CAM_FRONT_AF_PWR_DN_L_GPIO	CAM_TCA6416_GPIO_BASE + 10
#define CAM_FRONT_LDO_SHUTDN_L_GPIO	CAM_TCA6416_GPIO_BASE + 11
#define CAM_FRONT_LED_EXP			CAM_TCA6416_GPIO_BASE + 12
#define CAM_SNN_LED_REAR_EXP		CAM_TCA6416_GPIO_BASE + 13
/* PIN 19 NOT USED and is reserved */
#define CAM_NOT_USED				CAM_TCA6416_GPIO_BASE + 14
#define CAM_I2C_MUX_RST_EXP			CAM_TCA6416_GPIO_BASE + 15
#define CAM_TCA6416_GPIO_END		CAM_TCA6416_GPIO_BASE + 16

/* CAMERA RELATED GPIOs on CARDHU */
#define OV5650_RESETN_GPIO			TEGRA_GPIO_PBB0
#define CAM1_POWER_DWN_GPIO			TEGRA_GPIO_PBB5
#define CAM2_POWER_DWN_GPIO			TEGRA_GPIO_PBB6
#define CAM3_POWER_DWN_GPIO			TEGRA_GPIO_PBB7
#define CAMERA_CSI_CAM_SEL_GPIO		TEGRA_GPIO_PBB4
#define CAMERA_CSI_MUX_SEL_GPIO		TEGRA_GPIO_PCC1
#define CAM1_LDO_EN_GPIO			TEGRA_GPIO_PR6
#define CAM2_LDO_EN_GPIO			TEGRA_GPIO_PR7
#define CAM3_LDO_EN_GPIO			TEGRA_GPIO_PS0

/* PCA954x I2C bus expander bus addresses */
#define PCA954x_I2C_BUS_BASE	6
#define PCA954x_I2C_BUS0	(PCA954x_I2C_BUS_BASE + 0)
#define PCA954x_I2C_BUS1	(PCA954x_I2C_BUS_BASE + 1)
#define PCA954x_I2C_BUS2	(PCA954x_I2C_BUS_BASE + 2)
#define PCA954x_I2C_BUS3	(PCA954x_I2C_BUS_BASE + 3)

#define AC_PRESENT_GPIO		TPS6591X_GPIO_GP4

/*****************Interrupt tables ******************/
/* External peripheral act as interrupt controller */
/* TPS6591x IRQs */
#define TPS6591X_IRQ_BASE	TEGRA_NR_IRQS
#define TPS6591X_IRQ_END	(TPS6591X_IRQ_BASE + 18)

#define AC_PRESENT_INT		(TPS6591X_INT_GPIO4 + TPS6591X_IRQ_BASE)

int cardhu_charge_init(void);
int cardhu_regulator_init(void);
int cardhu_suspend_init(void);
int cardhu_sdhci_init(void);
int cardhu_pinmux_init(void);
int cardhu_panel_init(void);
int cardhu_sensors_init(void);
int cardhu_kbc_init(void);
int cardhu_scroll_init(void);
int cardhu_keys_init(void);
int cardhu_gpio_switch_regulator_init(void);
int cardhu_pins_state_init(void);
int cardhu_emc_init(void);
int cardhu_power_off_init(void);
int cardhu_edp_init(void);
int cardhu_pmon_init(void);

#endif
