/*
 * arch/arm/mach-tegra/gpio.c
 *
 * Copyright (c) 2010 Google, Inc
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
 *
 * Copyright (c) 2011 NVIDIA Corporation.
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
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/syscore_ops.h>

#include <asm/mach/irq.h>

#include <mach/iomap.h>
#include "pm-irq.h"
#include "pm.h"
#include <mach/pinmux.h>

#define GPIO_BANK(x)		((x) >> 5)
#define GPIO_PORT(x)		(((x) >> 3) & 0x3)
#define GPIO_BIT(x)		((x) & 0x7)

#define GPIO_CNF(x)		(GPIO_REG(x) + 0x00)
#define GPIO_OE(x)		(GPIO_REG(x) + 0x10)
#define GPIO_OUT(x)		(GPIO_REG(x) + 0X20)
#define GPIO_IN(x)		(GPIO_REG(x) + 0x30)
#define GPIO_INT_STA(x)		(GPIO_REG(x) + 0x40)
#define GPIO_INT_ENB(x)		(GPIO_REG(x) + 0x50)
#define GPIO_INT_LVL(x)		(GPIO_REG(x) + 0x60)
#define GPIO_INT_CLR(x)		(GPIO_REG(x) + 0x70)

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
#define GPIO_REG(x)		(IO_TO_VIRT(TEGRA_GPIO_BASE) +	\
				 GPIO_BANK(x) * 0x80 +		\
				 GPIO_PORT(x) * 4)

#define GPIO_MSK_CNF(x)		(GPIO_REG(x) + 0x800)
#define GPIO_MSK_OE(x)		(GPIO_REG(x) + 0x810)
#define GPIO_MSK_OUT(x)		(GPIO_REG(x) + 0X820)
#define GPIO_MSK_INT_STA(x)	(GPIO_REG(x) + 0x840)
#define GPIO_MSK_INT_ENB(x)	(GPIO_REG(x) + 0x850)
#define GPIO_MSK_INT_LVL(x)	(GPIO_REG(x) + 0x860)
#else
#define GPIO_REG(x)		(IO_TO_VIRT(TEGRA_GPIO_BASE) +	\
				 GPIO_BANK(x) * 0x100 +		\
				 GPIO_PORT(x) * 4)

#define GPIO_MSK_CNF(x)		(GPIO_REG(x) + 0x80)
#define GPIO_MSK_OE(x)		(GPIO_REG(x) + 0x90)
#define GPIO_MSK_OUT(x)		(GPIO_REG(x) + 0XA0)
#define GPIO_MSK_INT_STA(x)	(GPIO_REG(x) + 0xC0)
#define GPIO_MSK_INT_ENB(x)	(GPIO_REG(x) + 0xD0)
#define GPIO_MSK_INT_LVL(x)	(GPIO_REG(x) + 0xE0)
#endif

#define GPIO_INT_LVL_MASK		0x010101
#define GPIO_INT_LVL_EDGE_RISING	0x000101
#define GPIO_INT_LVL_EDGE_FALLING	0x000100
#define GPIO_INT_LVL_EDGE_BOTH		0x010100
#define GPIO_INT_LVL_LEVEL_HIGH		0x000001
#define GPIO_INT_LVL_LEVEL_LOW		0x000000

#if defined(CONFIG_MACH_BSSQ) || defined(CONFIG_MACH_STAR)
#define REG_CNF			0
#define REG_OE			1
#define REG_OUT			2
#define REG_INT_ENB		3
#define REG_INT_LVL		4
#define DBG_BUF_SIZE	64
int get_gpio_reg_data(int port, int pin, int gpio, int reg);
extern int gpio_get_pinmux_group(int gpio_nr);
#endif

struct tegra_gpio_bank {
	int bank;
	int irq;
	spinlock_t lvl_lock[4];
#ifdef CONFIG_PM_SLEEP
	u32 cnf[4];
	u32 out[4];
	u32 oe[4];
	u32 int_enb[4];
	u32 int_lvl[4];
#endif
};

static struct tegra_gpio_bank tegra_gpio_banks[] = {
	{.bank = 0, .irq = INT_GPIO1},
	{.bank = 1, .irq = INT_GPIO2},
	{.bank = 2, .irq = INT_GPIO3},
	{.bank = 3, .irq = INT_GPIO4},
	{.bank = 4, .irq = INT_GPIO5},
	{.bank = 5, .irq = INT_GPIO6},
	{.bank = 6, .irq = INT_GPIO7},
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	{.bank = 7, .irq = INT_GPIO8},
#endif
};

#if defined(CONFIG_MACH_BSSQ) || defined(CONFIG_MACH_STAR)
typedef enum tegra_pin_group_conf {
	ATA = 0x00000000,
	ATB,
	ATC,
	ATD,
	CDEV1,
	CDEV2,
	CSUS,
	DAP1,
	DAP2,
	DAP3,
	DAP4,
	DTA,
	DTB,
	DTC,
	DTD,
	DTE,
	GPU,
	GPV,
	I2CP,
	IRTX,
	IRRX,
	KBCB,
	KBCA,
	PMC,
	PTA,
	RM,
	KBCE,
	KBCF,
	GMA,
	GMC,
	SDIO1,
	OWC,
	GME = 0x00010000,
	SDC,
	SDD,
	SLXA,
	UNUSED_1_4,
	SLXC,
	SLXD,
	SLXK,
	SPDI,
	SPDO,
	SPIA,
	SPIB,
	SPIC,
	SPID,
	SPIE,
	SPIF,
	SPIG,
	SPIH,
	UAA,
	UAB,
	UAC,
	UAD,
	UCA,
	UCB,
	UNUSED_1_24,
	ATE,
	KBCC,
	UNUSED_1_27,
	UNUSED_1_28,
	GMB,
	GMD,
	DDC,
	LD0 = 0x00020000,
	LD1,
	LD2,
	LD3,
	LD4,
	LD5,
	LD6,
	LD7,
	LD8,
	LD9,
	LD10,
	LD11,
	LD12,
	LD13,
	LD14,
	LD15,
	LD16,
	LD17,
	LHP0,
	LHP1,
	LHP2,
	LVP0,
	LVP1,
	HDINT,
	LM0,
	LM1,
	LVS,
	LSC0,
	LSC1,
	LSCK,
	LDC,
	LCSN,
	LSPI = 0x00030000,
	LSDA,
	LSDI,
	LPW0,
	LPW1,
	LPW2,
	LDI,
	LHS,
	LPP,
	KBCD,
	GPU7,
	DTF,
	UDA,
	CRTP,
	SDB,
	UNUSED_3_16,
	UNUSED_3_17,
	UNUSED_3_18,
	UNUSED_3_19,
	UNUSED_3_20,
	UNUSED_3_21,
	UNUSED_3_22,
	UNUSED_3_23,
	UNUSED_3_24,
	UNUSED_3_25,
	UNUSED_3_26,
	UNUSED_3_27,
	UNUSED_3_28,
	UNUSED_3_29,
	UNUSED_3_30,
	UNUSED_3_31,
	TRISTATE_SKIP = 0xffffffff,
} tegra_pin_group_conf_t;

// GPIO, SFIO configure
typedef enum tegra_gpio_sfio_confg {
	SFIO_ENABLE = 0,
	GPIO_ENABLE,
} tegra_gpio_sfio_confg_t;

// Input, output configure
typedef enum tegra_gpio_oe_confg {
	GPIO_INPUT = 0,
	GPIO_OUTPUT,
} tegra_gpio_oe_confg_t;

// out enable configure
typedef enum tegra_gpio_out_confg {
	GPIO_SLEEP_LOW = 0x00000000,
	GPIO_SLEEP_HIGH,
	GPIO_INIT_ONLY_LOW = 0x00010000,
	GPIO_INIT_ONLY_HIGH,
} tegra_gpio_out_confg_t;

struct tegra_init_gpio_info {
	u8 port;
	u8 pin;
	tegra_gpio_sfio_confg_t cnf;
	tegra_gpio_oe_confg_t   oe;
	tegra_gpio_out_confg_t  out;
	tegra_pin_group_conf_t  group;
};

#if defined (CONFIG_KS1103)
#include <lge/gpio_ks1103.h>
#elif defined(CONFIG_LU6500)
#include <lge/gpio_lu6500.h>
#elif defined(CONFIG_SU880) || defined(CONFIG_KU8800)
#include <lge/gpio_sku880.h>
#elif defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_P999)
#include "lge/star/include/lge/board_star_su660_gpio.h"
#endif

#if defined(CONFIG_MACH_STAR) && (defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_P999))
static struct tegra_gpio_bank tegra_sleep_gpio_banks[] = {
	//	PORT 0	 ,	PORT 1	 ,	 PORT2	 ,	 PORT3		  
	//	A, B, C, D		  
	{.bank = 0, .irq = INT_GPIO1, 
		.cnf		= {0x00000000, 0x00000008, 0x00000000, 0x00000001},
		.out		= {0x00000000, 0x00000008, 0x00000000, 0x00000000}, 
		.oe 		= {0x00000000, 0x00000008, 0x00000000, 0x00000001},  
		.int_enb	= {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
		.int_lvl	= {0x00000000, 0x00000000, 0x00000000, 0x00000000}}, 
	//	E, F, G, H
	{.bank = 1, .irq = INT_GPIO2, 
		.cnf		= {0x000000ff, 0x00000000, 0x0000000b, 0x00000000},
		.out		= {0x0000001f, 0x00000000, 0x00000000, 0x00000000}, 
		.oe 		= {0x000000ff, 0x00000000, 0x00000000, 0x00000000},  
		.int_enb	= {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
		.int_lvl	= {0x00000000, 0x00000000, 0x00080800, 0x00000000}},   
	//	I, J, K, L
	{.bank = 2, .irq = INT_GPIO3, 
		.cnf		= {0x000000a1, 0x00000005, 0x00000038, 0x00000000},
		.out		= {0x00000080, 0x00000001, 0x00000018, 0x00000000}, 
		.oe 		= {0x00000080, 0x00000005, 0x00000038, 0x00000000},  
		.int_enb	= {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
		.int_lvl	= {0x00202000, 0x00000000, 0x00000000, 0x00000000}},   
	//	M, N, O, P
	{.bank = 3, .irq = INT_GPIO4, 
		.cnf		= {0x00000000, 0x00000050, 0x00000021, 0x00000000},
		.out		= {0x00000000, 0x00000050, 0x00000000, 0x00000000}, 
		.oe 		= {0x00000000, 0x00000050, 0x00000001, 0x00000000},  
		.int_enb	= {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
		.int_lvl	= {0x00000000, 0x00080800, 0x00002020, 0x00000000}},   
	//	Q, R, S, T	
	{.bank = 4, .irq = INT_GPIO5, 
		.cnf		= {0x0000001b, 0x000000FB, 0x00000007, 0x00000011},
		.out		= {0x00000003, 0x00000000, 0x00000002, 0x00000000}, 
		.oe 		= {0x0000001b, 0x000000C9, 0x00000002, 0x00000011},  
		.int_enb	= {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
		.int_lvl	= {0x00000000, 0x00008000, 0x00000000, 0x00000000}},   
	//	U, V, W, X	   
	{.bank = 5, .irq = INT_GPIO6, 
		.cnf		= {0x0000003f, 0x00000081, 0x00000000, 0x00000060},
		.out		= {0x00000000, 0x00000001, 0x00000000, 0x00000040}, 
		.oe 		= {0x0000001e, 0x00000081, 0x00000000, 0x00000000},  
		.int_enb	= {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
		.int_lvl	= {0x00000100, 0x00000000, 0x00000000, 0x00004000}},   
	// Y, Z, AA, AB    
	{.bank = 6, .irq = INT_GPIO7, 
		.cnf		= {0x00000000, 0x00000000, 0x00000000, 0x00000001},
		.out		= {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
		.oe 		= {0x00000000, 0x00000000, 0x00000000, 0x00000001},  
		.int_enb	= {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
		.int_lvl	= {0x00000000, 0x00000000, 0x00000000, 0x00000000}},   
};	
#else	
static struct tegra_gpio_bank tegra_sleep_gpio_banks[] = {
	{.bank = 0, .irq = INT_GPIO1},
	{.bank = 1, .irq = INT_GPIO2},
	{.bank = 2, .irq = INT_GPIO3},
	{.bank = 3, .irq = INT_GPIO4},
	{.bank = 4, .irq = INT_GPIO5},
	{.bank = 5, .irq = INT_GPIO6},
	{.bank = 6, .irq = INT_GPIO7},
};
#endif
#endif

static int tegra_gpio_compose(int bank, int port, int bit)
{
	return (bank << 5) | ((port & 0x3) << 3) | (bit & 0x7);
}

void tegra_gpio_set_tristate(int gpio_nr, enum tegra_tristate ts)
{
	int pin_group  =  tegra_pinmux_get_pingroup(gpio_nr);
	tegra_pinmux_set_tristate(pin_group, ts);
}

static void tegra_gpio_mask_write(u32 reg, int gpio, int value)
{
	u32 val;

	val = 0x100 << GPIO_BIT(gpio);
	if (value)
		val |= 1 << GPIO_BIT(gpio);
	__raw_writel(val, reg);
}

int tegra_gpio_get_bank_int_nr(int gpio)
{
	int bank;
	int irq;
	if (gpio >= TEGRA_NR_GPIOS) {
		pr_warn("%s : Invalid gpio ID - %d\n", __func__, gpio);
		return -EINVAL;
	}
	bank = gpio >> 5;
	irq = tegra_gpio_banks[bank].irq;
	return irq;
}

#if defined(CONFIG_MACH_BSSQ)
static int tegra_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	//WARN_ON(tegra_gpio_io_power_config(offset, 1) != 0);
	tegra_gpio_enable(offset);
	//tegra_set_gpio_tristate(offset, TEGRA_TRI_NORMAL);
	return 0;
}

static void tegra_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	tegra_gpio_disable(offset);
	//tegra_set_gpio_tristate(offset, TEGRA_TRI_TRISTATE);
	//WARN_ON(tegra_gpio_io_power_config(offset, 0) != 0);
}
#endif

void tegra_gpio_enable(int gpio)
{
	if (gpio >= TEGRA_NR_GPIOS) {
		pr_warn("%s : Invalid gpio ID - %d\n", __func__, gpio);
		return;
	}
	tegra_gpio_mask_write(GPIO_MSK_CNF(gpio), gpio, 1);
}
EXPORT_SYMBOL_GPL(tegra_gpio_enable);

void tegra_gpio_disable(int gpio)
{
	if (gpio >= TEGRA_NR_GPIOS) {
		pr_warn("%s : Invalid gpio ID - %d\n", __func__, gpio);
		return;
	}
	tegra_gpio_mask_write(GPIO_MSK_CNF(gpio), gpio, 0);
}
EXPORT_SYMBOL_GPL(tegra_gpio_disable);

void tegra_gpio_init_configure(unsigned gpio, bool is_input, int value)
{
	if (gpio >= TEGRA_NR_GPIOS) {
		pr_warn("%s : Invalid gpio ID - %d\n", __func__, gpio);
		return;
	}
	if (is_input) {
		tegra_gpio_mask_write(GPIO_MSK_OE(gpio), gpio, 0);
	} else {
		tegra_gpio_mask_write(GPIO_MSK_OUT(gpio), gpio, value);
		tegra_gpio_mask_write(GPIO_MSK_OE(gpio), gpio, 1);
	}
	tegra_gpio_mask_write(GPIO_MSK_CNF(gpio), gpio, 1);
}

static void tegra_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	tegra_gpio_mask_write(GPIO_MSK_OUT(offset), offset, value);
}

static int tegra_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	if ((__raw_readl(GPIO_OE(offset)) >> GPIO_BIT(offset)) & 0x1)
		return (__raw_readl(GPIO_OUT(offset)) >>
			GPIO_BIT(offset)) & 0x1;
	return (__raw_readl(GPIO_IN(offset)) >> GPIO_BIT(offset)) & 0x1;
}

static int tegra_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	tegra_gpio_mask_write(GPIO_MSK_OE(offset), offset, 0);
	return 0;
}

static int tegra_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
					int value)
{
	tegra_gpio_set(chip, offset, value);
	tegra_gpio_mask_write(GPIO_MSK_OE(offset), offset, 1);
	return 0;
}

int tegra_gpio_to_int_pin(int gpio)
{
	if (gpio < TEGRA_NR_GPIOS)
		return tegra_gpio_banks[gpio >> 5].irq;

	return -EIO;
}


static struct gpio_chip tegra_gpio_chip = {
	.label			= "tegra-gpio",
	.direction_input	= tegra_gpio_direction_input,
	.get			= tegra_gpio_get,
	.direction_output	= tegra_gpio_direction_output,
	.set			= tegra_gpio_set,
#if defined(CONFIG_MACH_BSSQ)
	.request			= tegra_gpio_request,
	.free				= tegra_gpio_free,
#endif
	.base			= 0,
	.ngpio			= TEGRA_NR_GPIOS,
};

static void tegra_gpio_irq_ack(struct irq_data *d)
{
	int gpio = d->irq - INT_GPIO_BASE;

	__raw_writel(1 << GPIO_BIT(gpio), GPIO_INT_CLR(gpio));

#ifdef CONFIG_TEGRA_FPGA_PLATFORM
	/* FPGA platforms have a serializer between the GPIO
	   block and interrupt controller. Allow time for
	   clearing of the GPIO interrupt to propagate to the
	   interrupt controller before re-enabling the IRQ
	   to prevent double interrupts. */
	udelay(15);
#endif
}

static void tegra_gpio_irq_mask(struct irq_data *d)
{
	int gpio = d->irq - INT_GPIO_BASE;

	tegra_gpio_mask_write(GPIO_MSK_INT_ENB(gpio), gpio, 0);
}

static void tegra_gpio_irq_unmask(struct irq_data *d)
{
	int gpio = d->irq - INT_GPIO_BASE;

	tegra_gpio_mask_write(GPIO_MSK_INT_ENB(gpio), gpio, 1);
}

static int tegra_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	int gpio = d->irq - INT_GPIO_BASE;
	struct tegra_gpio_bank *bank = irq_data_get_irq_chip_data(d);
	int port = GPIO_PORT(gpio);
	int lvl_type;
	int val;
	unsigned long flags;

#if defined(CONFIG_MACH_BSSQ) || defined(CONFIG_MACH_STAR)
	if (NULL == bank) {
		printk("GPIO: %s() failed to get irq chip data\n", __func__);
		return -EINVAL;
	}
#endif


	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		lvl_type = GPIO_INT_LVL_EDGE_RISING;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		lvl_type = GPIO_INT_LVL_EDGE_FALLING;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		lvl_type = GPIO_INT_LVL_EDGE_BOTH;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		lvl_type = GPIO_INT_LVL_LEVEL_HIGH;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		lvl_type = GPIO_INT_LVL_LEVEL_LOW;
		break;

	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&bank->lvl_lock[port], flags);

	val = __raw_readl(GPIO_INT_LVL(gpio));
	val &= ~(GPIO_INT_LVL_MASK << GPIO_BIT(gpio));
	val |= lvl_type << GPIO_BIT(gpio);
	__raw_writel(val, GPIO_INT_LVL(gpio));

	spin_unlock_irqrestore(&bank->lvl_lock[port], flags);

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		__irq_set_handler_locked(d->irq, handle_level_irq);
	else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		__irq_set_handler_locked(d->irq, handle_edge_irq);

	tegra_pm_irq_set_wake_type(d->irq, type);

	return 0;
}

static void tegra_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct tegra_gpio_bank *bank;
	int port;
	int pin;
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	bank = irq_get_handler_data(irq);

	for (port = 0; port < 4; port++) {
		int gpio = tegra_gpio_compose(bank->bank, port, 0);
		unsigned long sta = __raw_readl(GPIO_INT_STA(gpio)) &
			__raw_readl(GPIO_INT_ENB(gpio));

		for_each_set_bit(pin, &sta, 8)
			generic_handle_irq(gpio_to_irq(gpio + pin));
	}

	chained_irq_exit(chip, desc);

}

#ifdef CONFIG_PM_SLEEP
static void tegra_gpio_resume(void)
{
	unsigned long flags;
	int b;
	int p;

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	if (tegra_get_current_suspend_mode() != TEGRA_SUSPEND_LP0)
		return;
#endif

	local_irq_save(flags);

	for (b = 0; b < ARRAY_SIZE(tegra_gpio_banks); b++) {
		struct tegra_gpio_bank *bank = &tegra_gpio_banks[b];

		for (p = 0; p < ARRAY_SIZE(bank->oe); p++) {
			unsigned int gpio = (b<<5) | (p<<3);
			__raw_writel(bank->cnf[p], GPIO_CNF(gpio));
			__raw_writel(bank->out[p], GPIO_OUT(gpio));
			__raw_writel(bank->oe[p], GPIO_OE(gpio));
			__raw_writel(bank->int_lvl[p], GPIO_INT_LVL(gpio));
			__raw_writel(bank->int_enb[p], GPIO_INT_ENB(gpio));
		}
	}

	local_irq_restore(flags);
}

//LGE_CHANGE_S  euikyeom.kim@lge.com
#if defined(CONFIG_MACH_STAR_DUMP_GPIO)	
extern char gpio_regs_buf_suspend[PAGE_SIZE];
#endif
//LGE_CHANGE_E  euikyeom.kim@lge.com

static int tegra_gpio_suspend(void)
{
	unsigned long flags;
	int b;
	int p;

//LGE_CHANGE_S  euikyeom.kim@lge.com
#if defined(CONFIG_MACH_STAR_DUMP_GPIO)	
        ssize_t count = 0; 
#endif
//LGE_CHANGE_E  euikyeom.kim@lge.com

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	if (tegra_get_current_suspend_mode() != TEGRA_SUSPEND_LP0)
		return 0;
#endif

	local_irq_save(flags);
	for (b = 0; b < ARRAY_SIZE(tegra_gpio_banks); b++) {
		struct tegra_gpio_bank *bank = &tegra_gpio_banks[b];

		for (p = 0; p < ARRAY_SIZE(bank->oe); p++) {
			unsigned int gpio = (b<<5) | (p<<3);
			bank->cnf[p] = __raw_readl(GPIO_CNF(gpio));
			bank->out[p] = __raw_readl(GPIO_OUT(gpio));
			bank->oe[p] = __raw_readl(GPIO_OE(gpio));
			bank->int_enb[p] = __raw_readl(GPIO_INT_ENB(gpio));
			bank->int_lvl[p] = __raw_readl(GPIO_INT_LVL(gpio));
		}

//LGE_CHANGE_S  euikyeom.kim@lge.com
#if defined(CONFIG_MACH_STAR_DUMP_GPIO)
                    count += snprintf(gpio_regs_buf_suspend + count,         
                            PAGE_SIZE - count,                       
                            "%4d:%4d %3x %2x %3x\n", b, p,           
                            bank->cnf[p], bank->oe[p], bank->out[p]);
#endif
//LGE_CHANGE_E  euikyeom.kim@lge.com
	}

#if defined(CONFIG_MACH_STAR) || defined(CONFIG_MACH_BSSQ)
	pr_info("\n[HAVE2RUN] <<< Suspend GPIO Setting value (before) [END] >>>  \n");
	for (b=0; b<ARRAY_SIZE(tegra_sleep_gpio_banks); b++) {
		struct tegra_gpio_bank *bank = &tegra_sleep_gpio_banks[b];

		for (p=0; p<ARRAY_SIZE(bank->oe); p++) {
			unsigned int gpio = (b<<5) | (p<<3);
			__raw_writel(bank->cnf[p], GPIO_CNF(gpio));
			__raw_writel(bank->oe[p], GPIO_OE(gpio));
			#if 1   // masked bit should be maintained current out status.
			if(bank->out[p] >> 16)
			{
				u32 expected_out = bank->out[p] & 0xFFFF;
				u32 current_out = __raw_readl(GPIO_OUT(gpio));
				current_out  &= (bank->out[p] >> 16);
				expected_out &= ~(bank->out[p] >> 16);
				expected_out |= current_out;
				__raw_writel(expected_out, GPIO_OUT(gpio));
			}else{
				__raw_writel((bank->out[p] & 0xFFFF), GPIO_OUT(gpio));
			}
			#else
				__raw_writel(bank->out[p], GPIO_OUT(gpio));
			#endif
			__raw_writel(bank->int_lvl[p], GPIO_INT_LVL(gpio));
			__raw_writel(bank->int_enb[p], GPIO_INT_ENB(gpio));
#if defined(SLEEP_GPIO_LOG)				
			pr_info("%d:%d %02x %02x %02x %02x %06x\n", b, p, bank->cnf[p], bank->out[p],
				bank->oe[p], bank->int_enb[p], bank->int_lvl[p]);
#endif
		}
	}

	pr_info("[HAVE2RUN] <<< Suspend GPIO Setting value (after) [START] >>>  \n");
#if defined(SLEEP_GPIO_LOG)	
	for (b=0; b<ARRAY_SIZE(tegra_gpio_banks); b++) {
		struct tegra_gpio_bank *bank = &tegra_gpio_banks[b];

		for (p=0; p<ARRAY_SIZE(bank->oe); p++) {
			unsigned int gpio = (b<<5) | (p<<3);
			pr_info("%d:%d %02x %02x %02x %02x %06x\n", b, p, __raw_readl(GPIO_CNF(gpio)), 
				__raw_readl(GPIO_OUT(gpio)), __raw_readl(GPIO_OE(gpio)), __raw_readl(GPIO_INT_ENB(gpio)),
				__raw_readl(GPIO_INT_LVL(gpio))	);
			}
	}
#endif	
	pr_info("\n[POWER] <<< Suspend GPIO Setting value (after) [END] >>>  \n");
#else
	 //LGE_CHANGE_S bae.cheolhwan@lge.com 2012-02-23. Issue that can't play sound after deep sleep.
#endif

	local_irq_restore(flags);

	return 0;
}

static int tegra_gpio_irq_set_wake(struct irq_data *d, unsigned int enable)
{
	struct tegra_gpio_bank *bank = irq_data_get_irq_chip_data(d);
	int ret = 0;

	ret = tegra_pm_irq_set_wake(d->irq, enable);

	if (ret)
		return ret;

	ret = irq_set_irq_wake(bank->irq, enable);

	if (ret)
		tegra_pm_irq_set_wake(d->irq, !enable);

	return ret;
}
#else
#define tegra_gpio_irq_set_wake NULL
#define tegra_gpio_suspend NULL
#define tegra_gpio_resume NULL
#endif

static struct syscore_ops tegra_gpio_syscore_ops = {
	.suspend = tegra_gpio_suspend,
	.resume = tegra_gpio_resume,
};

int tegra_gpio_resume_init(void)
{
	register_syscore_ops(&tegra_gpio_syscore_ops);

	return 0;
}

static struct irq_chip tegra_gpio_irq_chip = {
	.name		= "GPIO",
	.irq_ack	= tegra_gpio_irq_ack,
	.irq_mask	= tegra_gpio_irq_mask,
	.irq_unmask	= tegra_gpio_irq_unmask,
	.irq_set_type	= tegra_gpio_irq_set_type,
	.irq_set_wake	= tegra_gpio_irq_set_wake,
	.flags		= IRQCHIP_MASK_ON_SUSPEND,
};


/* This lock class tells lockdep that GPIO irqs are in a different
 * category than their parents, so it won't report false recursion.
 */
static struct lock_class_key gpio_lock_class;

static int __init tegra_gpio_init(void)
{
	struct tegra_gpio_bank *bank;
	int i;
	int j;
	int b, p;

#if defined(CONFIG_MACH_BSSQ) || defined(CONFIG_MACH_STAR)
	for (i = 0; i < ARRAY_SIZE(tegra_gpio_banks); i++) {
		for (j = 0; j < 4; j++) {
			tegra_sleep_gpio_banks[i].cnf[j] = 0;
			tegra_sleep_gpio_banks[i].oe[j] = 0;
			tegra_sleep_gpio_banks[i].out[j] = 0;
			tegra_sleep_gpio_banks[i].int_enb[j] = 0;
			tegra_sleep_gpio_banks[i].int_lvl[j] = 0;
		}
	}

	for (i=0; i<ARRAY_SIZE(tegra_sleep_gpio_info_array); i++) {
		// disable tristate group
		if( tegra_sleep_gpio_info_array[i].port == 0xFF ||
			tegra_sleep_gpio_info_array[i].oe == GPIO_OUTPUT)
		{
			u32 tristate_reg_num = tegra_sleep_gpio_info_array[i].group >> 16;
			u32 tristate_reg_bit = tegra_sleep_gpio_info_array[i].group & 0xFFFF;
			//sleep_pinmux_reg[tristate_reg_num] &= ~((u32)(0x1 << tristate_reg_bit));
		}

		// It's not GPIO pin, just set tristate.
		if(tegra_sleep_gpio_info_array[i].port != 0xFF){
			b = (tegra_sleep_gpio_info_array[i].port) >> 2;
			p = (tegra_sleep_gpio_info_array[i].port) & 0x3;

			tegra_sleep_gpio_banks[b].cnf[p] |= (tegra_sleep_gpio_info_array[i].cnf)<<(tegra_sleep_gpio_info_array[i].pin);
			tegra_sleep_gpio_banks[b].oe[p] |= (tegra_sleep_gpio_info_array[i].oe) <<(tegra_sleep_gpio_info_array[i].pin);
			tegra_sleep_gpio_banks[b].out[p] |= (tegra_sleep_gpio_info_array[i].out)<<(tegra_sleep_gpio_info_array[i].pin);
		}
	}

#if defined(CONFIG_MACH_BSSQ)

	for (i=0; i<ARRAY_SIZE(tegra_init_gpio_info_array); i++) {
		unsigned int gpio = (tegra_init_gpio_info_array[i].port<<3);
		u32 current_val;
		u32 expect_bit;
		current_val = __raw_readl(GPIO_CNF(gpio));
		expect_bit = (tegra_init_gpio_info_array[i].cnf)<<(tegra_init_gpio_info_array[i].pin);
		current_val &= ~expect_bit;
		current_val |= expect_bit;
		__raw_writel(current_val, GPIO_CNF(gpio));

		current_val = __raw_readl(GPIO_OE(gpio));
		expect_bit = (tegra_init_gpio_info_array[i].oe)<<(tegra_init_gpio_info_array[i].pin);
		current_val &= ~expect_bit;
		current_val |= expect_bit;
		__raw_writel(current_val, GPIO_OE(gpio));

		current_val = __raw_readl(GPIO_OUT(gpio));
		expect_bit = (tegra_init_gpio_info_array[i].out)<<(tegra_init_gpio_info_array[i].pin);
		current_val &= ~expect_bit;
		current_val |= expect_bit;
		__raw_writel(current_val, GPIO_OUT(gpio));
	}
#endif	
#endif

	for (i = 0; i < ARRAY_SIZE(tegra_gpio_banks); i++) {
		for (j = 0; j < 4; j++) {
			int gpio = tegra_gpio_compose(i, j, 0);
			__raw_writel(0x00, GPIO_INT_ENB(gpio));
			__raw_writel(0x00, GPIO_INT_STA(gpio));
		}
	}

	gpiochip_add(&tegra_gpio_chip);

	for (i = INT_GPIO_BASE; i < (INT_GPIO_BASE + TEGRA_NR_GPIOS); i++) {
		bank = &tegra_gpio_banks[GPIO_BANK(irq_to_gpio(i))];

		irq_set_lockdep_class(i, &gpio_lock_class);
		irq_set_chip_data(i, bank);
		irq_set_chip_and_handler(i, &tegra_gpio_irq_chip,
					 handle_simple_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	for (i = 0; i < ARRAY_SIZE(tegra_gpio_banks); i++) {
		bank = &tegra_gpio_banks[i];

		for (j = 0; j < 4; j++)
			spin_lock_init(&bank->lvl_lock[j]);

		irq_set_handler_data(bank->irq, bank);
		irq_set_chained_handler(bank->irq, tegra_gpio_irq_handler);

	}

	return 0;
}

postcore_initcall(tegra_gpio_init);

void __init tegra_gpio_config(struct tegra_gpio_table *table, int num)
{
	int i;

	for (i = 0; i < num; i++) {
		int gpio = table[i].gpio;

		if (table[i].enable)
			tegra_gpio_enable(gpio);
		else
			tegra_gpio_disable(gpio);
	}
}

#ifdef	CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int dbg_gpio_show(struct seq_file *s, void *unused)
{
	int i;
	int j;

	seq_printf(s, "Bank:Port CNF OE OUT IN INT_STA INT_ENB INT_LVL\n");
	for (i = 0; i < ARRAY_SIZE(tegra_gpio_banks); i++) {
		for (j = 0; j < 4; j++) {
			int gpio = tegra_gpio_compose(i, j, 0);
			seq_printf(s,
				"%d:%d %02x %02x %02x %02x %02x %02x %06x\n",
				i, j,
				__raw_readl(GPIO_CNF(gpio)),
				__raw_readl(GPIO_OE(gpio)),
				__raw_readl(GPIO_OUT(gpio)),
				__raw_readl(GPIO_IN(gpio)),
				__raw_readl(GPIO_INT_STA(gpio)),
				__raw_readl(GPIO_INT_ENB(gpio)),
				__raw_readl(GPIO_INT_LVL(gpio)));
		}
	}
	return 0;
}

static int dbg_gpio_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_gpio_show, &inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_gpio_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init tegra_gpio_debuginit(void)
{
	(void) debugfs_create_file("tegra_gpio", S_IRUGO,
					NULL, NULL, &debug_fops);
	return 0;
}
late_initcall(tegra_gpio_debuginit);
#endif
