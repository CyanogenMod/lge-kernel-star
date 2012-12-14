/*
 * Copyright (C) 2010-2011 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */


#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <mach-tegra/gpio-names.h>
#include <lge/board-star.h>
#include <linux/max8922l.h>

extern int device_power_control(char* reg_id, bool on);

static hw_rev start_hw_rev;

hw_rev get_hw_rev(void)
{
    //printk(KERN_DEBUG "This board is %d\n\n\n", startablet_hw_rev);
    return start_hw_rev;
}

static int __init tegra_hw_rev_setup(char *line)
{
	static char board_rev[10];
	strlcpy(board_rev, line, 10);
	if (!strncmp(board_rev, "REV.A", sizeof("REV.A")))
		start_hw_rev = REV_A;
	else if (!strncmp(board_rev, "REV.C", sizeof("REV.C")))
		start_hw_rev = REV_C;
	else if (!strncmp(board_rev, "REV.E", sizeof("REV.E")))
		start_hw_rev = REV_E;
	else if (!strncmp(board_rev, "REV.F", sizeof("REV.F")))
		start_hw_rev = REV_F;
	else if (!strncmp(board_rev, "REV.G", sizeof("REV.G")))
		start_hw_rev = REV_G;
	else if (!strncmp(board_rev, "REV.H", sizeof("REV.H")))
		start_hw_rev = REV_H;
	else if (!strncmp(board_rev, "REV.I", sizeof("REV.I")))
		start_hw_rev = REV_I;
	else if (!strncmp(board_rev, "REV.J", sizeof("REV.J")))
		start_hw_rev = REV_J;
	else if (!strncmp(board_rev, "1.0", sizeof("1.0")))
		start_hw_rev = REV_1_0;
	else
	{
		printk(KERN_ERR "FAILED!!! board_rev: %s\n",board_rev);
		start_hw_rev = REV_F;
	}
	printk(KERN_DEBUG "board_rev: %s, HW Rev: %d\n",board_rev, start_hw_rev);
	return 1;
}

#ifdef CONFIG_CM_BOOTLOADER_COMPAT
__setup("brdrev=", tegra_hw_rev_setup);
#else
__setup("hw_rev=", tegra_hw_rev_setup);
#endif

#if defined( CONFIG_STAR_VIBRATOR)
#include <mach/vibrator.h>

static struct pwm_vib_platform_data	star_vib_platform_data = {
	.max_timeout		=	15000,
	.active_low			=	0,
	.initial_vibrate	=	0,
	.pwm_id				=	0,
	.period_ns			=	50000,
	.duty_ns			=	5000,
	.enable				=	TEGRA_GPIO_PU4,
	.power				=	&device_power_control,
};

static struct platform_device star_vib_device = {
	.name   =   "star_vib_name",
	.id     =   -1,
	.dev    =   {
		.platform_data  = &star_vib_platform_data,
	},
};

int __init star_vibrator_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PU4);
	tegra_gpio_disable(TEGRA_GPIO_PU3);

	platform_device_register(&star_vib_device);

	return	0;
}
#endif

#if defined(CONFIG_STAR_HALL)
struct star_hall_platform_data {
  u32 gpio;
  int (*power)(char* reg_id, bool on);
  unsigned long irqflags;
};

static struct star_hall_platform_data star_hall_pdata = {
    .gpio   = TEGRA_GPIO_PU5,
    .power  = &device_power_control,
    .irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
};

static struct platform_device star_hall_device =
{
    .name = "star_hall",
    .id   = -1,
    .dev  = {
        .platform_data = &star_hall_pdata,
    },    
};

int __init star_hall_ic_init(void)
{
    platform_device_register(&star_hall_device);
    
    return 0;
}
#endif

#define GPIO_PGB        130
#define GPIO_EN_SET     145
#define GPIO_STATUS     146
struct charger_ic_platform_data max8922l_platform_data = {
	.gpio_en_set	= GPIO_EN_SET,
	.gpio_status	= GPIO_STATUS,
	.gpio_pgb	= GPIO_PGB,
	.irqflags	= IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
};

struct platform_device max8922l_charger_ic_device = {

   .name = "charger_ic_max8922l",
   .id   = -1,
   .dev.platform_data = &max8922l_platform_data,
};

static struct i2c_board_info __initdata star_i2c_bus6_devices_info[] ={
	{
		I2C_BOARD_INFO("max14526", STAR_I2C_DEVICE_ADDR_MUIC),
	},
};

struct platform_device star_battery_charger_device = {
        .name = "star_battery_charger",
        .id = -1,
};

void star_misc_init(void)
{
#if defined(CONFIG_STAR_VIBRATOR)
    star_vibrator_init();
#endif
#if defined(CONFIG_STAR_HALL)
    star_hall_ic_init();
#endif
    i2c_register_board_info(6 , star_i2c_bus6_devices_info, ARRAY_SIZE(star_i2c_bus6_devices_info));
}
