/*
 * arch/arm/mach-tegra/board-ventana.h
 *
 * Copyright (C) 2011 Google, Inc.
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

#ifndef _MACH_TEGRA_BOARD_VENTANA_H
#define _MACH_TEGRA_BOARD_VENTANA_H

int ventana_charge_init(void);
int ventana_regulator_init(void);
int ventana_sdhci_init(void);
int ventana_pinmux_init(void);
int ventana_panel_init(void);
int ventana_sensors_init(void);
int ventana_kbc_init(void);
int ventana_emc_init(void);
int ventana_charger_init(void);

/* external gpios */

/* TPS6586X gpios */
#define TPS6586X_GPIO_BASE	TEGRA_NR_GPIOS
#define AVDD_DSI_CSI_ENB_GPIO	(TPS6586X_GPIO_BASE + 1) /* gpio2 */

/* TCA6416 gpios */
#define TCA6416_GPIO_BASE	(TEGRA_NR_GPIOS + 4)
#define CAM1_PWR_DN_GPIO	(TCA6416_GPIO_BASE + 0) /* gpio0 */
#define CAM1_RST_L_GPIO		(TCA6416_GPIO_BASE + 1) /* gpio1 */
#define CAM1_AF_PWR_DN_L_GPIO	(TCA6416_GPIO_BASE + 2) /* gpio2 */
#define CAM1_LDO_SHUTDN_L_GPIO	(TCA6416_GPIO_BASE + 3) /* gpio3 */
#define CAM2_PWR_DN_GPIO	(TCA6416_GPIO_BASE + 4) /* gpio4 */
#define CAM2_RST_L_GPIO		(TCA6416_GPIO_BASE + 5) /* gpio5 */
#define CAM2_AF_PWR_DN_L_GPIO	(TCA6416_GPIO_BASE + 6) /* gpio6 */
#define CAM2_LDO_SHUTDN_L_GPIO	(TCA6416_GPIO_BASE + 7) /* gpio7 */
#define CAM3_PWR_DN_GPIO	(TCA6416_GPIO_BASE + 8) /* gpio8 */
#define CAM3_RST_L_GPIO		(TCA6416_GPIO_BASE + 9) /* gpio9 */
#define CAM3_AF_PWR_DN_L_GPIO	(TCA6416_GPIO_BASE + 10) /* gpio10 */
#define CAM3_LDO_SHUTDN_L_GPIO	(TCA6416_GPIO_BASE + 11) /* gpio11 */
#define CAM_I2C_MUX_RST_GPIO	(TCA6416_GPIO_BASE + 15) /* gpio15 */
#define TCA6416_GPIO_END	(TCA6416_GPIO_BASE + 31)

/* WM8903 GPIOs */
#define VENTANA_GPIO_WM8903(_x_)	(TCA6416_GPIO_END + 1 + (_x_))
#define VENTANA_GPIO_WM8903_END		VENTANA_GPIO_WM8903(4)

/* Audio-related GPIOs */
#define TEGRA_GPIO_CDC_IRQ		TEGRA_GPIO_PX3
#define TEGRA_GPIO_SPKR_EN		VENTANA_GPIO_WM8903(2)
#define TEGRA_GPIO_HP_DET		TEGRA_GPIO_PW2
#define TEGRA_GPIO_HP_DET		TEGRA_GPIO_PW2
#define TEGRA_GPIO_INT_MIC_EN		TEGRA_GPIO_PX0
#define TEGRA_GPIO_EXT_MIC_EN		TEGRA_GPIO_PX1

/* AC detect GPIO */
#define AC_PRESENT_GPIO			TEGRA_GPIO_PV3

/* Interrupt numbers from external peripherals */
#define TPS6586X_INT_BASE	TEGRA_NR_IRQS
#define TPS6586X_INT_END	(TPS6586X_INT_BASE + 32)

/* Invensense MPU Definitions */
#define MPU_GYRO_NAME		"mpu3050"
#define MPU_GYRO_IRQ_GPIO	TEGRA_GPIO_PZ4
#define MPU_GYRO_ADDR		0x68
#define MPU_GYRO_BUS_NUM	0
#define MPU_GYRO_ORIENTATION	{ 0, -1, 0, -1, 0, 0, 0, 0, -1 }
#define MPU_ACCEL_NAME		"kxtf9"
#define MPU_ACCEL_IRQ_GPIO	0 /* Disable ACCELIRQ: TEGRA_GPIO_PN4 */
#define MPU_ACCEL_ADDR		0x0F
#define MPU_ACCEL_BUS_NUM	0
#define MPU_ACCEL_ORIENTATION	{ 0, -1, 0, -1, 0, 0, 0, 0, -1 }
#define MPU_COMPASS_NAME	"ak8975"
#define MPU_COMPASS_IRQ_GPIO	TEGRA_GPIO_PN5
#define MPU_COMPASS_ADDR	0x0C
#define MPU_COMPASS_BUS_NUM	4
#define MPU_COMPASS_ORIENTATION	{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }

#endif
