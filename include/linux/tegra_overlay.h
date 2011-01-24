/*
 * Copyright (C) 2010 NVIDIA Corporation
 * Author: Dan Willemsen <dwillemsen@nvidia.com>
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

#ifndef __LINUX_TEGRA_OVERLAY_H
#define __LINUX_TEGRA_OVERLAY_H

#include <linux/ioctl.h>
#include <linux/types.h>
#include <video/tegrafb.h>

/* set index to -1 to ignore window data */
struct tegra_overlay_windowattr {
	__s32	index;
	__u32	buff_id;
	__u32	blend;
	__u32	offset;
	__u32	offset_u;
	__u32	offset_v;
	__u32	stride;
	__u32	stride_uv;
	__u32	pixformat;
	__u32	x;
	__u32	y;
	__u32	w;
	__u32	h;
	__u32	out_x;
	__u32	out_y;
	__u32	out_w;
	__u32	out_h;
	__u32	z;
	__u32	pre_syncpt_id;
	__u32	pre_syncpt_val;
	__u32	hfilter;
	__u32	vfilter;
	__u32	tiled;
	__u32	flags;
};

struct tegra_overlay_flip_args {
	struct tegra_overlay_windowattr win[TEGRA_FB_FLIP_N_WINDOWS];
	__u32 post_syncpt_id;
	__u32 post_syncpt_val;
};

#define TEGRA_OVERLAY_IOCTL_MAGIC		'O'

#define TEGRA_OVERLAY_IOCTL_OPEN_WINDOW		_IOWR(TEGRA_OVERLAY_IOCTL_MAGIC, 0x40, __u32)
#define TEGRA_OVERLAY_IOCTL_CLOSE_WINDOW	_IOW(TEGRA_OVERLAY_IOCTL_MAGIC, 0x41, __u32)
#define TEGRA_OVERLAY_IOCTL_FLIP		_IOW(TEGRA_OVERLAY_IOCTL_MAGIC, 0x42, struct tegra_overlay_flip_args)
#define TEGRA_OVERLAY_IOCTL_SET_NVMAP_FD	_IOW(TEGRA_OVERLAY_IOCTL_MAGIC, 0x43, __u32)

#define TEGRA_OVERLAY_IOCTL_MIN_NR		_IOC_NR(TEGRA_OVERLAY_IOCTL_OPEN_WINDOW)
#define TEGRA_OVERLAY_IOCTL_MAX_NR		_IOC_NR(TEGRA_OVERLAY_IOCTL_SET_NVMAP_FD)

#endif
