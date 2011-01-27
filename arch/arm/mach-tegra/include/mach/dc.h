/*
 * arch/arm/mach-tegra/include/mach/dc.h
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
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

#ifndef __MACH_TEGRA_DC_H
#define __MACH_TEGRA_DC_H

#include <linux/pm.h>

#define TEGRA_MAX_DC		2
#define DC_N_WINDOWS		3


/* DSI pixel data format */
enum {
	TEGRA_DSI_PIXEL_FORMAT_16BIT_P,
	TEGRA_DSI_PIXEL_FORMAT_18BIT_P,
	TEGRA_DSI_PIXEL_FORMAT_18BIT_NP,
	TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
};

/* DSI virtual channel number */
enum {
	TEGRA_DSI_VIRTUAL_CHANNEL_0,
	TEGRA_DSI_VIRTUAL_CHANNEL_1,
	TEGRA_DSI_VIRTUAL_CHANNEL_2,
	TEGRA_DSI_VIRTUAL_CHANNEL_3,
};

/* DSI transmit method for video data */
enum {
	TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
	TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,
};

/* DSI HS clock mode */
enum {
	TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS,
	TEGRA_DSI_VIDEO_CLOCK_TX_ONLY,
};

/* DSI burst mode setting in video mode */
enum {
	TEGRA_DSI_VIDEO_NONE_BURST_MODE,
	TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END,
	TEGRA_DSI_VIDEO_BURST_MODE_LOWEST_SPEED,
	TEGRA_DSI_VIDEO_BURST_MODE_LOW_SPEED,
	TEGRA_DSI_VIDEO_BURST_MODE_MEDIUM_SPEED,
	TEGRA_DSI_VIDEO_BURST_MODE_FAST_SPEED,
	TEGRA_DSI_VIDEO_BURST_MODE_FASTEST_SPEED,
	TEGRA_DSI_VIDEO_BURST_MODE_MANUAL,
};

enum {
	TEGRA_DSI_PACKET_CMD,
	TEGRA_DSI_DELAY_MS,
};

struct tegra_dsi_cmd {
	u8	cmd_type;
	u8	data_id;
	union {
		u16 data_len;
		u16 delay_ms;
		struct{
			u8 data0;
			u8 data1;
		}sp;
	}sp_len_dly;
	u8	*pdata;
};

#define DSI_CMD_SHORT(di, p0, p1)	{ \
					.cmd_type = TEGRA_DSI_PACKET_CMD, \
					.data_id = di, \
					.sp_len_dly.sp.data0 = p0, \
					.sp_len_dly.sp.data1 = p1, \
					}
#define DSI_DLY_MS(ms)	{ \
			.cmd_type = TEGRA_DSI_DELAY_MS, \
			.sp_len_dly.delay_ms = ms, \
			}

#define DSI_CMD_LONG(di, ptr)	{ \
				.cmd_type = TEGRA_DSI_PACKET_CMD, \
				.data_id = di, \
				.sp_len_dly.data_len = ARRAY_SIZE(ptr), \
				.pdata = ptr, \
				}

struct dsi_phy_timing_ns {
	u16		t_hsdexit_ns;
	u16		t_hstrail_ns;
	u16		t_hsprepr_ns;
	u16		t_datzero_ns;

	u16		t_clktrail_ns;
	u16		t_clkpost_ns;
	u16		t_clkzero_ns;
	u16		t_tlpx_ns;
};

struct tegra_dsi_out {
	u8		n_data_lanes;			/* required*/
	u8		pixel_format;			/* required*/
	u8		refresh_rate;			/* required*/
	u8		virtual_channel;		/* required*/

	bool		panel_has_frame_buffer;	/* required*/

	struct tegra_dsi_cmd*	dsi_init_cmd;		/* required*/
	u16		n_init_cmd;			/* required*/

	u8		video_data_type;		/* required*/
	u8		video_clock_mode;
	u8		video_burst_mode;

	u16		panel_buffer_size_byte;
	u16		panel_reset_timeout_msec;

	bool		hs_cmd_mode_supported;
	bool		hs_cmd_mode_on_blank_supported;
	bool		enable_hs_clock_on_lp_cmd_mode;

	u32 		max_panel_freq_khz;
	u32 		lp_cmd_mode_freq_khz;
	u32		hs_clk_in_lp_cmd_mode_freq_khz;
	u32		burst_mode_freq_khz;

	struct dsi_phy_timing_ns phy_timing;
};

struct tegra_dc_mode {
	int	pclk;
	int	h_ref_to_sync;
	int	v_ref_to_sync;
	int	h_sync_width;
	int	v_sync_width;
	int	h_back_porch;
	int	v_back_porch;
	int	h_active;
	int	v_active;
	int	h_front_porch;
	int	v_front_porch;
	int	stereo_mode;
	u32	flags;
};

#define TEGRA_DC_MODE_FLAG_NEG_V_SYNC	(1 << 0)
#define TEGRA_DC_MODE_FLAG_NEG_H_SYNC	(1 << 1)

enum {
	TEGRA_DC_OUT_RGB,
	TEGRA_DC_OUT_HDMI,
	TEGRA_DC_OUT_DSI,
};

struct tegra_dc_out_pin {
	int     name;
	int     pol;
};

enum {
	TEGRA_DC_OUT_PIN_DATA_ENABLE,
	TEGRA_DC_OUT_PIN_H_SYNC,
	TEGRA_DC_OUT_PIN_V_SYNC,
	TEGRA_DC_OUT_PIN_PIXEL_CLOCK,
};

enum {
	TEGRA_DC_OUT_PIN_POL_LOW,
	TEGRA_DC_OUT_PIN_POL_HIGH,
};

enum {
	TEGRA_DC_DISABLE_DITHER = 1,
	TEGRA_DC_ORDERED_DITHER,
	TEGRA_DC_ERRDIFF_DITHER,
};

enum {
	TEGRA_PIN_OUT_CONFIG_SEL_LHP0_LD21,
	TEGRA_PIN_OUT_CONFIG_SEL_LHP1_LD18,
	TEGRA_PIN_OUT_CONFIG_SEL_LHP2_LD19,
	TEGRA_PIN_OUT_CONFIG_SEL_LVP0_LVP0_Out,
	TEGRA_PIN_OUT_CONFIG_SEL_LVP1_LD20,

	TEGRA_PIN_OUT_CONFIG_SEL_LM1_M1,
	TEGRA_PIN_OUT_CONFIG_SEL_LM1_LD21,
	TEGRA_PIN_OUT_CONFIG_SEL_LM1_PM1,

	TEGRA_PIN_OUT_CONFIG_SEL_LDI_LD22,
	TEGRA_PIN_OUT_CONFIG_SEL_LPP_LD23,
	TEGRA_PIN_OUT_CONFIG_SEL_LDC_SDC,
	TEGRA_PIN_OUT_CONFIG_SEL_LSPI_DE,
};

struct tegra_dc_out {
	int			type;
	unsigned		flags;

	/* size in mm */
	unsigned		h_size;
	unsigned		v_size;

	int			dcc_bus;
	int			hotplug_gpio;

	unsigned		order;
	unsigned		align;
	unsigned		depth;
	unsigned		dither;

	unsigned		height; /* mm */
	unsigned		width; /* mm */

	struct tegra_dc_mode	*modes;
	int			n_modes;

	struct tegra_dsi_out	*dsi;

	struct tegra_dc_out_pin	*out_pins;
	unsigned		n_out_pins;

	u8			*out_sel_configs;
	unsigned		n_out_sel_configs;

	int	(*enable)(void);
	int	(*postpoweron)(void);
	int	(*disable)(void);
};

/* bits for tegra_dc_out.flags */
#define TEGRA_DC_OUT_HOTPLUG_HIGH		(0 << 1)
#define TEGRA_DC_OUT_HOTPLUG_LOW		(1 << 1)
#define TEGRA_DC_OUT_HOTPLUG_MASK		(1 << 1)
#define TEGRA_DC_OUT_NVHDCP_POLICY_ALWAYS_ON	(0 << 2)
#define TEGRA_DC_OUT_NVHDCP_POLICY_ON_DEMAND	(1 << 2)
#define TEGRA_DC_OUT_NVHDCP_POLICY_MASK		(1 << 2)

#define TEGRA_DC_ALIGN_MSB		0
#define TEGRA_DC_ALIGN_LSB		1

#define TEGRA_DC_ORDER_RED_BLUE		0
#define TEGRA_DC_ORDER_BLUE_RED		1

struct tegra_dc;
struct nvmap_handle_ref;

struct tegra_dc_win {
	u8			idx;
	u8			fmt;
	u32			flags;

	void			*virt_addr;
	dma_addr_t		phys_addr;
	unsigned		offset;
	unsigned		offset_u;
	unsigned		offset_v;
	unsigned		stride;
	unsigned		stride_uv;
	unsigned		x;
	unsigned		y;
	unsigned		w;
	unsigned		h;
	unsigned		out_x;
	unsigned		out_y;
	unsigned		out_w;
	unsigned		out_h;
	unsigned		z;

	int			dirty;
	int			underflows;
	struct tegra_dc		*dc;

	struct nvmap_handle_ref	*cur_handle;
};


#define TEGRA_WIN_FLAG_ENABLED		(1 << 0)
#define TEGRA_WIN_FLAG_BLEND_PREMULT	(1 << 1)
#define TEGRA_WIN_FLAG_BLEND_COVERAGE	(1 << 2)
#define TEGRA_WIN_FLAG_TILED		(1 << 3)

#define TEGRA_WIN_BLEND_FLAGS_MASK \
	(TEGRA_WIN_FLAG_BLEND_PREMULT | TEGRA_WIN_FLAG_BLEND_COVERAGE)

/* Note: These are the actual values written to the DC_WIN_COLOR_DEPTH register
 * and may change in new tegra architectures.
 */
#define TEGRA_WIN_FMT_P1		0
#define TEGRA_WIN_FMT_P2		1
#define TEGRA_WIN_FMT_P4		2
#define TEGRA_WIN_FMT_P8		3
#define TEGRA_WIN_FMT_B4G4R4A4		4
#define TEGRA_WIN_FMT_B5G5R5A		5
#define TEGRA_WIN_FMT_B5G6R5		6
#define TEGRA_WIN_FMT_AB5G5R5		7
#define TEGRA_WIN_FMT_B8G8R8A8		12
#define TEGRA_WIN_FMT_R8G8B8A8		13
#define TEGRA_WIN_FMT_B6x2G6x2R6x2A8	14
#define TEGRA_WIN_FMT_R6x2G6x2B6x2A8	15
#define TEGRA_WIN_FMT_YCbCr422		16
#define TEGRA_WIN_FMT_YUV422		17
#define TEGRA_WIN_FMT_YCbCr420P		18
#define TEGRA_WIN_FMT_YUV420P		19
#define TEGRA_WIN_FMT_YCbCr422P		20
#define TEGRA_WIN_FMT_YUV422P		21
#define TEGRA_WIN_FMT_YCbCr422R		22
#define TEGRA_WIN_FMT_YUV422R		23
#define TEGRA_WIN_FMT_YCbCr422RA	24
#define TEGRA_WIN_FMT_YUV422RA		25

struct tegra_fb_data {
	int		win;

	int		xres;
	int		yres;
	int		bits_per_pixel; /* -1 means autodetect */

	unsigned long	flags;
};

#define TEGRA_FB_FLIP_ON_PROBE		(1 << 0)

struct tegra_dc_platform_data {
	unsigned long		flags;
	unsigned long		emc_clk_rate;
	struct tegra_dc_out	*default_out;
	struct tegra_fb_data	*fb;
};

#define TEGRA_DC_FLAG_ENABLED		(1 << 0)

struct tegra_dc *tegra_dc_get_dc(unsigned idx);
struct tegra_dc_win *tegra_dc_get_window(struct tegra_dc *dc, unsigned win);

void tegra_dc_enable(struct tegra_dc *dc);
void tegra_dc_disable(struct tegra_dc *dc);

u32 tegra_dc_get_syncpt_id(const struct tegra_dc *dc);
u32 tegra_dc_incr_syncpt_max(struct tegra_dc *dc);
void tegra_dc_incr_syncpt_min(struct tegra_dc *dc, u32 val);

/* tegra_dc_update_windows and tegra_dc_sync_windows do not support windows
 * with differenct dcs in one call
 */
int tegra_dc_update_windows(struct tegra_dc_win *windows[], int n);
int tegra_dc_sync_windows(struct tegra_dc_win *windows[], int n);

int tegra_dc_set_mode(struct tegra_dc *dc, const struct tegra_dc_mode *mode);

unsigned tegra_dc_get_out_height(struct tegra_dc *dc);
unsigned tegra_dc_get_out_width(struct tegra_dc *dc);

/* PM0 and PM1 signal control */
#define TEGRA_PWM_PM0 0
#define TEGRA_PWM_PM1 1

struct tegra_dc_pwm_params {
	int which_pwm;
	unsigned int period;
	unsigned int clk_div;
	unsigned int clk_select;
	unsigned int duty_cycle;
};

void tegra_dc_config_pwm(struct tegra_dc *dc, struct tegra_dc_pwm_params *cfg);

#endif
