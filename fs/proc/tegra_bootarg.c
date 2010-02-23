/*
 * fs/proc/tegra_bootarg.c
 *
 * A procfs to reflect tegra ATAGs to user-space
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#define NV_DEBUG 0

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include "nvos.h"
#include "nvcommon.h"
#include "nvassert.h"
#include "nvbootargs.h"

#define BOOT_ARGUMENTS		"bootarg"
#define MODULE_DISP		"disp"

#define DISP_CONTROLLER		"Controller"
#define DISP_DEV_INDEX		"DisplayDeviceIndex"
#define DISP_BENABLED		"bEnabled"

static struct proc_dir_entry *tegra_bootarg_dir;
static struct proc_dir_entry *disp_dir;

static struct proc_dir_entry *disp_arg_controller;
static struct proc_dir_entry *disp_arg_dev_index;
static struct proc_dir_entry *disp_arg_benabled;

static int proc_read_disp_controller(char *page, char **start, off_t off,
	int count, int *eof, void *data)
{
	int len;
	NvBootArgsDisplay NvBootArgDisp;
	NvError return_v;

	return_v = NvOsBootArgGet(NvBootArgKey_Display,
		&NvBootArgDisp, sizeof(NvBootArgsDisplay));

	if (return_v!=NvSuccess) {
		pr_err("%s: fail to get boot_arg_disp\n", __func__);
		return 0;
	}
	len = snprintf(page, count, "%u\n", NvBootArgDisp.Controller);
	return len;
}

static int proc_read_dev_index(char *page, char **start, off_t off,
	int count, int *eof, void *data)
{
	int len;
	NvBootArgsDisplay NvBootArgDisp;
	NvError return_v;

	return_v = NvOsBootArgGet(NvBootArgKey_Display,
		&NvBootArgDisp, sizeof(NvBootArgsDisplay));

	if (return_v!=NvSuccess) {
		pr_err("%s: fail to get boot_arg_disp\n", __func__);
		return 0;
	}
	len = snprintf(page, count, "%u\n", NvBootArgDisp.DisplayDeviceIndex);
	return len;
}

static int proc_read_benabled(char *page, char **start, off_t off,
	int count, int *eof, void *data)
{
	int len;
	NvBootArgsDisplay NvBootArgDisp;
	NvError return_v;

	return_v = NvOsBootArgGet(NvBootArgKey_Display,
		&NvBootArgDisp, sizeof(NvBootArgsDisplay));

	if (return_v!=NvSuccess) {
		pr_err("%s: fail to get boot_arg_disp\n", __func__);
		return 0;
	}
	len = snprintf(page, count, "%u\n", (NvU32)(NvBootArgDisp.bEnabled));

	return len;
}

static int __init tegra_bootarg_init(void)
{
	int rv = 0;

	tegra_bootarg_dir = proc_mkdir(BOOT_ARGUMENTS, NULL);
	if (tegra_bootarg_dir == NULL) {
		rv = -ENOMEM;
		pr_err("%s: mkdir _proc_bootarg failure\n", __func__);
		return rv;
	}

	/*build up a display boot argument directory*/
	disp_dir = proc_mkdir(MODULE_DISP, tegra_bootarg_dir);
	if (disp_dir == NULL) {
		rv = -ENOMEM;
		pr_err("%s: mkdir _proc_bootarg_disp failure\n", __func__);
		remove_proc_entry(BOOT_ARGUMENTS, NULL);
		return rv;
	}

	/*build up each field of display boot argument as a different file*/

	/* 1. DISP_CONTROLLER*/
	/*0444 is from S_IRUSR|S_IRGRP|S_IROTH*/
	disp_arg_controller = create_proc_read_entry(DISP_CONTROLLER, 0444,
				     disp_dir, proc_read_disp_controller, NULL);
	if (disp_arg_controller == NULL) {
		rv = -ENOMEM;
		pr_err("%s: read entry failure of disp_arg_controller\n", __func__);
		remove_proc_entry(MODULE_DISP, tegra_bootarg_dir);
		remove_proc_entry(BOOT_ARGUMENTS, NULL);
		return rv;
	}

	/* 2. DISP_DEV_INDEX*/
	disp_arg_dev_index = create_proc_read_entry(DISP_DEV_INDEX, 0444,
				    disp_dir, proc_read_dev_index, NULL);
	if (disp_arg_dev_index == NULL) {
		rv = -ENOMEM;
		pr_err("%s: read entry failure of disp_arg_dev_index\n", __func__);
		remove_proc_entry(MODULE_DISP, tegra_bootarg_dir);
		remove_proc_entry(BOOT_ARGUMENTS, NULL);
		return rv;
	}

	/* 3. DISP_BENABLED*/
	disp_arg_benabled = create_proc_read_entry(DISP_BENABLED, 0444,
				   disp_dir, proc_read_benabled, NULL);
	if (disp_arg_benabled == NULL) {
		rv = -ENOMEM;
		pr_err("%s: read entry failure of disp_arg_benabled\n", __func__);
		remove_proc_entry(MODULE_DISP, tegra_bootarg_dir);
		remove_proc_entry(BOOT_ARGUMENTS, NULL);
		return rv;
	}

	/*If everything is OK, return zero*/
	return 0;

}

static void __exit tegra_bootarg_deinit(void)
{
	remove_proc_entry(MODULE_DISP, tegra_bootarg_dir);
	remove_proc_entry(BOOT_ARGUMENTS, NULL);
}

module_init(tegra_bootarg_init);
module_exit(tegra_bootarg_deinit);
