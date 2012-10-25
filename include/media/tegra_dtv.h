/*
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#ifndef __TEGRA_DTV_H__
#define __TEGRA_DTV_H__

#include <linux/ioctl.h>

#define TEGRA_DTV_MAGIC 'v'

#define TEGRA_DTV_IOCTL_START           _IO(TEGRA_DTV_MAGIC, 0)
#define TEGRA_DTV_IOCTL_STOP            _IO(TEGRA_DTV_MAGIC, 1)

struct tegra_dtv_hw_config {
	int clk_edge;
	int byte_swz_enabled;
	int bit_swz_enabled;

	int protocol_sel;
	int clk_mode;
	int fec_size;
	int body_size;
	int body_valid_sel;
	int start_sel;
	int err_pol;
	int psync_pol;
	int valid_pol;
};

#define TEGRA_DTV_IOCTL_SET_HW_CONFIG  _IOW(TEGRA_DTV_MAGIC, 2,		\
					   const struct tegra_dtv_hw_config *)
#define TEGRA_DTV_IOCTL_GET_HW_CONFIG  _IOR(TEGRA_DTV_MAGIC, 3,		\
					   struct tegra_dtv_hw_config *)

/* for selecting the pin configuration for VD(valid).
 * NONE : ERROR is tied to 0, PSYNC is tied to 0
 * ERROR: ERROR is tied to VD, PSYNC is tied to 0
 * PSYNC: ERROR is tied to 0, PSYNC is tied to VD
 */
enum {
	TEGRA_DTV_PROTOCOL_NONE = 0,
	TEGRA_DTV_PROTOCOL_ERROR,
	TEGRA_DTV_PROTOCOL_PSYNC,
};

enum {
	TEGRA_DTV_CLK_DISCONTINUOUS = 0,
	TEGRA_DTV_CLK_CONTINUOUS,
};

enum {
	TEGRA_DTV_BODY_VALID_IGNORE = 0,
	TEGRA_DTV_BODY_VALID_GATE,
};

enum {
	TEGRA_DTV_START_RESERVED = 0, /* never use this */
	TEGRA_DTV_START_PSYNC,
	TEGRA_DTV_START_VALID,
	TEGRA_DTV_START_BOTH,
};

enum {
	TEGRA_DTV_ERROR_POLARITY_HIGH = 0,
	TEGRA_DTV_ERROR_POLARITY_LOW,
};

enum {
	TEGRA_DTV_PSYNC_POLARITY_HIGH = 0,
	TEGRA_DTV_PSYNC_POLARITY_LOW,
};

enum {
	TEGRA_DTV_VALID_POLARITY_HIGH = 0,
	TEGRA_DTV_VALID_POLARITY_LOW,
};

#ifdef __KERNEL__
enum {
	TEGRA_DTV_CLK_POSEDGE,
	TEGRA_DTV_CLK_NEGEDGE,
};

struct tegra_dtv_platform_data {
	unsigned int dma_buf_size;
	int clk_edge;
	bool byte_swz_enabled;
	bool bit_swz_enabled;
};
#endif /* __KERNEL__ */

#endif /* __TEGRA_DTV_H__ */
