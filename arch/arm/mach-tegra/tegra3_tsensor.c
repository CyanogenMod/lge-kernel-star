/*
 * arch/arm/mach-tegra/tegra3_tsensor.c
 *
 * Copyright (C) 2011 NVIDIA Corporation.
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

#include <linux/kernel.h>
#include <linux/init.h>

#ifdef CONFIG_SENSORS_TEGRA_TSENSOR
#include <mach/tsensor.h>
#include <mach/tegra_fuse.h>
#include <devices.h>
#include <mach/iomap.h>
#include <mach/thermal.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/slab.h>

/* fuse revision constants used for tsensor */
#define TSENSOR_FUSE_REVISION_DECIMAL 8
#define TSENSOR_FUSE_REVISION_INTEGER 0

/* scratch register offsets needed for powering off PMU */
#define SCRATCH54_OFFSET			0x258
#define SCRATCH55_OFFSET			0x25C

/* scratch 54 register bit field offsets */
#define PMU_OFF_DATA_OFFSET			8

/* scratch 55 register bit field offsets */
#define RESET_TEGRA_OFFSET			31
#define CONTROLLER_TYPE_OFFSET			30
#define I2C_CONTROLLER_ID_OFFSET		27
#define PINMUX_OFFSET				24
#define CHECKSUM_OFFSET				16
#define PMU_16BIT_SUPPORT_OFFSET		15
/* scratch 55 register bit field masks */
#define RESET_TEGRA_MASK			0x1
#define CONTROLLER_TYPE_MASK			0x1
#define I2C_CONTROLLER_ID_MASK			0x7
#define PINMUX_MASK				0x7
#define CHECKSUM_MASK				0xff
#define PMU_16BIT_SUPPORT_MASK			0x1

#define TSENSOR_OFFSET	(4000 + 5000)

#ifdef CONFIG_TEGRA_INTERNAL_TSENSOR_EDP_SUPPORT
static int tsensor_get_temp(void *vdata, long *milli_temp)
{
	struct tegra_tsensor_data *data = vdata;
	return tsensor_thermal_get_temp(data, milli_temp);
}

static int tsensor_get_temp_low(void *vdata, long *milli_temp)
{
	struct tegra_tsensor_data *data = vdata;
	return tsensor_thermal_get_temp_low(data, milli_temp);
}

static int tsensor_set_limits(void *vdata,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct tegra_tsensor_data *data = vdata;
	return tsensor_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int tsensor_set_alert(void *vdata,
			void (*alert_func)(void *),
			void *alert_data)
{
	struct tegra_tsensor_data *data = vdata;
	return tsensor_thermal_set_alert(data, alert_func, alert_data);
}

static int tsensor_set_shutdown_temp(void *vdata, long shutdown_temp_milli)
{
	struct tegra_tsensor_data *data = vdata;
	return tsensor_thermal_set_shutdown_temp(data, shutdown_temp_milli);
}

static void tegra3_tsensor_probe_callback(struct tegra_tsensor_data *data)
{
	struct tegra_thermal_device *thermal_device;

	thermal_device = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);

	if (!thermal_device) {
		pr_err("unable to allocate thermal device\n");
		return;
	}

	thermal_device->name = "tsensor";
	thermal_device->data = data;
	thermal_device->offset = TSENSOR_OFFSET;
	thermal_device->get_temp = tsensor_get_temp;
	thermal_device->get_temp_low = tsensor_get_temp_low;
	thermal_device->set_limits = tsensor_set_limits;
	thermal_device->set_alert = tsensor_set_alert;
	thermal_device->set_shutdown_temp = tsensor_set_shutdown_temp;

	if (tegra_thermal_set_device(thermal_device)) /* This should not fail */
		BUG();
}
#endif

static struct tegra_tsensor_platform_data tsensor_data = {
#ifdef CONFIG_TEGRA_INTERNAL_TSENSOR_EDP_SUPPORT
	.probe_callback = tegra3_tsensor_probe_callback,
#endif
};

void __init tegra3_tsensor_init(struct tegra_tsensor_pmu_data *data)
{
	unsigned int reg;
	int err;
	u32 val, checksum;
	void __iomem *pMem = NULL;
	/* tsensor driver is instantiated based on fuse revision */
	err = tegra_fuse_get_revision(&reg);
	if (err)
		goto labelEnd;
	pr_info("\nTegra3 fuse revision %d ", reg);
	if (reg < TSENSOR_FUSE_REVISION_DECIMAL)
		goto labelEnd;

	if (!data)
		goto labelSkipPowerOff;

	if (!request_mem_region(TEGRA_PMC_BASE +
		SCRATCH54_OFFSET, 8, "tegra-tsensor"))
		pr_err(" [%s, line=%d]: Error mem busy\n",
			__func__, __LINE__);

	pMem = ioremap(TEGRA_PMC_BASE + SCRATCH54_OFFSET, 8);
	if (!pMem) {
		pr_err(" [%s, line=%d]: can't ioremap "
			"pmc iomem\n", __FILE__, __LINE__);
		goto labelEnd;
	}

	/*
	 * Fill scratch registers to power off the device
	 * in case if temperature crosses threshold TH3
	 */
	val = (data->poweroff_reg_data << PMU_OFF_DATA_OFFSET) |
		data->poweroff_reg_addr;
	writel(val, pMem);

	val = ((data->reset_tegra & RESET_TEGRA_MASK) << RESET_TEGRA_OFFSET) |
		((data->controller_type & CONTROLLER_TYPE_MASK) <<
		CONTROLLER_TYPE_OFFSET) |
		((data->i2c_controller_id & I2C_CONTROLLER_ID_MASK) <<
		I2C_CONTROLLER_ID_OFFSET) |
		((data->pinmux & PINMUX_MASK) << PINMUX_OFFSET) |
		((data->pmu_16bit_ops & PMU_16BIT_SUPPORT_MASK) <<
		PMU_16BIT_SUPPORT_OFFSET) | data->pmu_i2c_addr;

	checksum = data->poweroff_reg_addr +
		data->poweroff_reg_data + (val & 0xFF) +
		((val >> 8) & 0xFF) + ((val >> 24) & 0xFF);
	checksum &= 0xFF;
	checksum = 0x100 - checksum;

	val |= (checksum << CHECKSUM_OFFSET);
	writel(val, pMem + 4);

labelSkipPowerOff:
	/* set platform data for device before register */
	tegra_tsensor_device.dev.platform_data = &tsensor_data;
	platform_device_register(&tegra_tsensor_device);

labelEnd:
	return;
}

#else
void __init tegra3_tsensor_init(void) { }
#endif

