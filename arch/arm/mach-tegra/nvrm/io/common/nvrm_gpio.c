/*
 * arch/arm/mach-tegra/nvrm/io/common/nvrm_gpio.c
 *
 * NvRm GPIO API implemented using gpiolib
 *
 * Copyright (c) 2008-2009, NVIDIA Corporation.
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

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/gpio.h>

#include <mach/nvrm_linux.h>

#include <nvrm_gpio.h>
#include "../../../gpio-names.h"

struct pin_data {
	bool alloc;
	bool val;
};

static struct pin_data gpio_data[ARCH_NR_GPIOS];
extern void tegra_gpio_enable(int gpio);
extern void tegra_gpio_disable(int gpio);

static inline NvRmGpioPinHandle handle_of(int gpio_nr) {
	return (NvRmGpioPinHandle)(0x80000000ul | ((unsigned)gpio_nr));
}

static inline int gpio_of(NvRmGpioPinHandle handle_) {
	unsigned int handle = (unsigned int)handle_;

	if (!(handle & 0x80000000ul))
		return -1;

	return (int)(handle & ~0x80000000ul);
}

NvError NvRmGpioOpen(NvRmDeviceHandle rm, NvRmGpioHandle *gpio)
{
	/* shove something non-NULL into gpio */
	*gpio = (NvRmGpioHandle)s_hRmGlobal;
	return NvSuccess;
}

void NvRmGpioClose(NvRmGpioHandle gpio)
{
}

static inline NvError nverror_of(int ret)
{
	switch (ret) {
	case 0: return NvSuccess;
	case -EINVAL: return NvError_BadValue;
	case -EBUSY: return NvError_Busy;
	}
	return NvError_BadParameter;
}

static void gpio_to_name(int nr_gpio, char name[12])
{
	char *b = name;
	int port = nr_gpio >> 3;
	int pin = nr_gpio & 7;

	if (nr_gpio<0 || nr_gpio>=ARCH_NR_GPIOS) {
		snprintf(name, 12, "invalid");
		return;
	}

	do {
		char c = (port) % 26;
		*b++ = 'A' + c;
		port -= 26;
	} while (port >= 26);

	*b++ = '.';
	*b++ = '0' + pin;
	*b++ = 0;
}

static bool valid_gpio(const char *caller, int nr_gpio)
{
	if (nr_gpio >=0 && nr_gpio < ARRAY_SIZE(gpio_data)) {
		if (!gpio_data[nr_gpio].alloc) {
			char name[12];
			gpio_to_name(nr_gpio, name);
			pr_err("%s: using free gpio %s (%d)\n",
			       caller, name, nr_gpio);
			return false;
		}
		return true;
	}

	pr_err("%s: invalid gpio %d\n", caller, nr_gpio);
	return false;
}

NvError NvRmGpioAcquirePinHandle(NvRmGpioHandle gpio, NvU32 nr_port,
				 NvU32 nr_pin, NvRmGpioPinHandle *pin)
{
	int nr_gpio;
	char gpio_name[12];
	int ret;

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
	if (nr_port == NVRM_GPIO_CAMERA_PORT) {
		if (nr_pin>=0 && nr_pin<=4) {
			nr_gpio = TEGRA_GPIO_PBB1 + nr_pin;
		} else if (nr_pin==5) {
			nr_gpio = TEGRA_GPIO_PD2;
		} else if (nr_pin==6) {
			nr_gpio = TEGRA_GPIO_PA0;
		} else {
			pr_err("%s: invalid cam gpio %u\n", __func__, nr_pin);
			return NvError_BadParameter;
		}
	} else
#endif
	{
		nr_gpio = nr_port*8 + nr_pin;
	}

	ret = gpio_request(nr_gpio, "nvrm_gpio");
	if (ret) {
		gpio_to_name(nr_gpio, gpio_name);
		pr_err("%s: gpio_request for %d (%s) failed (%d)\n",
		       __func__, nr_gpio, gpio_name, ret);
		return nverror_of(ret);
	}

	gpio_data[nr_gpio].alloc = true;
	gpio_data[nr_gpio].val = false;

	*pin = handle_of(nr_gpio);
	return NvSuccess;
}

void NvRmGpioReleasePinHandles(NvRmGpioHandle gpio,
			       NvRmGpioPinHandle *hpins, NvU32 nr_pins)
{
	unsigned int i;

	if (!hpins)
		return;

	for (i=0; i<nr_pins; i++) {
		int gpio = gpio_of(hpins[i]);
		if (!valid_gpio(__func__, gpio))
			continue;

		gpio_free(gpio);
		gpio_data[gpio].val = false;
		gpio_data[gpio].alloc = false;
	}
}

void NvRmGpioReadPins(NvRmGpioHandle gpio, NvRmGpioPinHandle *hpins,
		      NvRmGpioPinState *states, NvU32 nr_pins)
{
	unsigned int i;

	if (!hpins || !states)
		return;

	for (i=0; i<nr_pins; i++) {
		int gpio = gpio_of(hpins[i]);
		int v;

		if (!valid_gpio(__func__, gpio))
			continue;

		v = gpio_get_value(gpio) & 0x1;
		states[i] = (v) ? NvRmGpioPinState_High : NvRmGpioPinState_Low;
	}
}

void NvRmGpioWritePins(NvRmGpioHandle gpio, NvRmGpioPinHandle *hpins,
		       NvRmGpioPinState *states, NvU32 nr_pins)

{
	unsigned int i;

	if (!hpins || !states)
		return;

	for (i=0; i<nr_pins; i++) {
		int gpio = gpio_of(hpins[i]);
		int v;

		if (!valid_gpio(__func__, gpio))
			continue;

		v = (states[i]==NvRmGpioPinState_Low) ? 0 : 1;

		gpio_data[gpio].val = v ? true : false;
		gpio_set_value(gpio, v);
	}
}

NvError NvRmGpioConfigPins(NvRmGpioHandle gpio, NvRmGpioPinHandle *hpins,
			   NvU32 nr_pins, NvRmGpioPinMode mode)
{
	unsigned int i;

	if (!hpins)
		return NvError_BadParameter;

	for (i=0; i<nr_pins; i++) {
		int gpio = gpio_of(hpins[i]);
		if (!valid_gpio(__func__, gpio))
			continue;

		if (mode == NvRmGpioPinMode_Inactive ||
		    mode == NvRmGpioPinMode_Function) {
			tegra_gpio_disable(gpio);
		} else {
			struct irq_chip *chip;
			int irq = gpio_to_irq(gpio);

			chip = get_irq_chip(irq);
			tegra_gpio_enable(gpio);

			switch (mode) {
			case NvRmGpioPinMode_InputData:
				gpio_direction_input(gpio);
				break;
			case NvRmGpioPinMode_Output:
				gpio_direction_output(gpio,
					(gpio_data[gpio].val ? 1 : 0));
				break;
			case NvRmGpioPinMode_InputInterruptRisingEdge:
				gpio_direction_input(gpio);
				chip->set_type(irq, IRQ_TYPE_EDGE_RISING);
				break;
			case NvRmGpioPinMode_InputInterruptFallingEdge:
				gpio_direction_input(gpio);
				chip->set_type(irq, IRQ_TYPE_EDGE_FALLING);
				break;
				break;
			case NvRmGpioPinMode_InputInterruptAny:
				gpio_direction_input(gpio);
				chip->set_type(irq, IRQ_TYPE_EDGE_BOTH);
				break;
			case NvRmGpioPinMode_InputInterruptHigh:
				gpio_direction_input(gpio);
				chip->set_type(irq, IRQ_TYPE_LEVEL_HIGH);
				break;
			case NvRmGpioPinMode_InputInterruptLow:
				gpio_direction_input(gpio);
				chip->set_type(irq, IRQ_TYPE_LEVEL_LOW);
				break;
			default:
				break;
			}
		}
	}

	return NvSuccess;
}

NvError NvRmGpioGetIrqs(NvRmDeviceHandle rm, NvRmGpioPinHandle *hpins,
			NvU32 *irqs, NvU32 nr_pins)
{
	unsigned int i;

	if (!hpins || !irqs)
		return NvError_BadParameter;

	for (i=0; i<nr_pins; i++) {
		int gpio = gpio_of(hpins[i]);
		if (!valid_gpio(__func__, gpio))
			return NvError_BadParameter;

		irqs[i] = gpio_to_irq(gpio);
	}

	return NvSuccess;
}
