/*
 * drivers/video/tegra/dc/cpu.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Erik Gilling <konkers@android.com>
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

#include <mach/dc.h>

#include "dc_reg.h"
#include "dc_priv.h"


static const u32 tegra_dc_cpu_enable_partial_pintable[] = {
	DC_COM_PIN_OUTPUT_ENABLE0,	0x00000000,
	DC_COM_PIN_OUTPUT_ENABLE1,	0x00000000,
	DC_COM_PIN_OUTPUT_ENABLE2,	0x00000000,
#ifndef CONFIG_MACH_STAR
	DC_COM_PIN_OUTPUT_ENABLE3,	0x00000000,
#endif
	DC_COM_PIN_OUTPUT_POLARITY0,	0x00000000,
	DC_COM_PIN_OUTPUT_POLARITY2,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA0,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA1,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA2,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA3,	0x00000000,
};

static const u32 tegra_dc_cpu_enable_pintable[] = {
	DC_COM_PIN_OUTPUT_ENABLE0,	0x00000000,
	DC_COM_PIN_OUTPUT_ENABLE1,	0x00000000,
	DC_COM_PIN_OUTPUT_ENABLE2,	0x00000000,
#ifndef CONFIG_MACH_STAR
	DC_COM_PIN_OUTPUT_ENABLE3,	0x00000000,
#endif
	DC_COM_PIN_OUTPUT_POLARITY0,	0x00000000,
	DC_COM_PIN_OUTPUT_POLARITY1,	0x01000000,
	DC_COM_PIN_OUTPUT_POLARITY2,	0x00000000,
	DC_COM_PIN_OUTPUT_POLARITY3,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA0,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA1,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA2,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA3,	0x00000000,
};

static const u32 tegra_dc_cpu_enable_out_sel_pintable[] = {
	DC_COM_PIN_OUTPUT_SELECT0,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT1,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT2,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT3,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT4,	0x00200222,
	DC_COM_PIN_OUTPUT_SELECT5,	0x00002200,
	DC_COM_PIN_OUTPUT_SELECT6,	0x00004000,
};

static const u32 tegra_dc_cpu_disable_pintable[] = {
	DC_COM_PIN_OUTPUT_ENABLE0,	0x55555555,
	DC_COM_PIN_OUTPUT_ENABLE1,	0x55150005,
	DC_COM_PIN_OUTPUT_ENABLE2,	0x55555555,
	DC_COM_PIN_OUTPUT_ENABLE3,	0x55555555,
	DC_COM_PIN_OUTPUT_POLARITY0,	0x00000000,
	DC_COM_PIN_OUTPUT_POLARITY1,	0x00000000,
	DC_COM_PIN_OUTPUT_POLARITY2,	0x00000000,
	DC_COM_PIN_OUTPUT_POLARITY3,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA0,	0xaaaaaaaa,
	DC_COM_PIN_OUTPUT_DATA1,	0xaaaaaaaa,
	DC_COM_PIN_OUTPUT_DATA2,	0xaaaaaaaa,
	DC_COM_PIN_OUTPUT_DATA3,	0xaaaaaaaa,
	DC_COM_PIN_OUTPUT_SELECT0,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT1,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT2,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT3,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT4,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT5,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT6,	0x00000000,
};

static void tegra_cpu_stop_dc_stream(struct tegra_dc *dc)
{
	tegra_dc_writel(dc, DISP_CTRL_MODE_STOP, DC_CMD_DISPLAY_COMMAND);
	tegra_dc_writel(dc, 0, DC_DISP_DISP_WIN_OPTIONS);
	tegra_dc_writel(dc, GENERAL_UPDATE, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
}

#define S_TO_MS(x)			(1000 * (x))
void tegra_cpu_stop_dc_stream_at_frame_end(struct tegra_dc *dc)
{
	int val;
	long timeout;
	u32 frame_period = DIV_ROUND_UP(S_TO_MS(1), 60);

	/* stop dc */
	tegra_cpu_stop_dc_stream(dc);

	/* enable frame end interrupt */
	val = tegra_dc_readl(dc, DC_CMD_INT_MASK);
	val |= FRAME_END_INT;
	tegra_dc_writel(dc, val, DC_CMD_INT_MASK);

	/* wait for frame_end completion.
	 * timeout is 2 frame duration to accomodate for
	 * internal delay.
	 */
	timeout = wait_for_completion_interruptible_timeout(
			&dc->frame_end_complete,
			msecs_to_jiffies(2 * frame_period));

	/* disable frame end interrupt */
	val = tegra_dc_readl(dc, DC_CMD_INT_MASK);
	val &= ~FRAME_END_INT;
	tegra_dc_writel(dc, val, DC_CMD_INT_MASK);

	if (timeout == 0)
		printk(KERN_WARNING "DC doesn't stop at end of frame.\n");
}

void tegra_dc_cpu_enable(struct tegra_dc *dc)
{
	int i;
	u32 out_sel_pintable[ARRAY_SIZE(tegra_dc_cpu_enable_out_sel_pintable)];
	u32 val;

	u32 h_width_pixels;
	u32 v_width_lines;
	u32 pixel_clk_hz;

	tegra_dc_io_start(dc);

	tegra_dc_writel(dc, PW0_ENABLE | PW1_ENABLE | PW2_ENABLE | PW3_ENABLE |
			PW4_ENABLE | PM0_ENABLE | PM1_ENABLE,
			DC_CMD_DISPLAY_POWER_CONTROL);

	tegra_cpu_stop_dc_stream(dc);

	tegra_dc_writel(dc, V_PULSE_1_ENABLE, DC_DISP_DISP_SIGNAL_OPTIONS0);
	tegra_dc_writel(dc, PULSE_POLARITY_LOW, DC_DISP_V_PULSE1_CONTROL);
	
	tegra_dc_writel(dc, PULSE_END(1), DC_DISP_V_PULSE1_POSITION_A);
	tegra_dc_writel(dc, 0, DC_DISP_V_PULSE1_POSITION_B);
	tegra_dc_writel(dc, 0, DC_DISP_V_PULSE1_POSITION_C);
	
	tegra_dc_writel(dc, 0x131, DC_DISP_INIT_SEQ_CONTROL);
	tegra_dc_writel(dc, 0x2c, DC_DISP_SPI_INIT_SEQ_DATA_A);
	tegra_dc_writel(dc, 0, DC_DISP_SPI_INIT_SEQ_DATA_B);
	tegra_dc_writel(dc, 0, DC_DISP_SPI_INIT_SEQ_DATA_C);
	tegra_dc_writel(dc, 0x5000, DC_DISP_SPI_INIT_SEQ_DATA_D);

	h_width_pixels = dc->mode.h_back_porch + dc->mode.h_front_porch +
			dc->mode.h_sync_width + dc->mode.h_active;
	v_width_lines = dc->mode.v_back_porch + dc->mode.v_front_porch +
			dc->mode.v_sync_width + dc->mode.v_active;

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
	{
		pixel_clk_hz = h_width_pixels * v_width_lines * 72;
	}
	else
	{
		pixel_clk_hz = h_width_pixels * v_width_lines * 60;
	}
	
	dc->mode.pclk = pixel_clk_hz;

	if (dc->out->out_pins) {
		tegra_dc_set_out_pin_polars(dc, dc->out->out_pins,
			dc->out->n_out_pins);
		tegra_dc_write_table(dc, tegra_dc_cpu_enable_partial_pintable);
	} else {
		tegra_dc_write_table(dc, tegra_dc_cpu_enable_pintable);
	}

	memcpy(out_sel_pintable, tegra_dc_cpu_enable_out_sel_pintable,
		sizeof(tegra_dc_cpu_enable_out_sel_pintable));

#if 0
	if (dc->out && dc->out->out_sel_configs) {
		u8 *out_sels = dc->out->out_sel_configs;
		for (i = 0; i < dc->out->n_out_sel_configs; i++) {
			switch (out_sels[i]) {
			case TEGRA_PIN_OUT_CONFIG_SEL_LM1_M1:
				out_sel_pintable[5*2+1] =
					(out_sel_pintable[5*2+1] &
					~PIN5_LM1_LCD_M1_OUTPUT_MASK) |
					PIN5_LM1_LCD_M1_OUTPUT_M1;
				break;
			case TEGRA_PIN_OUT_CONFIG_SEL_LM1_LD21:
				out_sel_pintable[5*2+1] =
					(out_sel_pintable[5*2+1] &
					~PIN5_LM1_LCD_M1_OUTPUT_MASK) |
					PIN5_LM1_LCD_M1_OUTPUT_LD21;
				break;
			case TEGRA_PIN_OUT_CONFIG_SEL_LM1_PM1:
				out_sel_pintable[5*2+1] =
					(out_sel_pintable[5*2+1] &
					~PIN5_LM1_LCD_M1_OUTPUT_MASK) |
					PIN5_LM1_LCD_M1_OUTPUT_PM1;
				break;
			default:
				dev_err(&dc->ndev->dev,
					"Invalid pin config[%d]: %d\n",
					 i, out_sels[i]);
				break;
			}
		}
	}
#endif

	tegra_dc_write_table(dc, out_sel_pintable);

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE) {
		/* disable LSPI/LCD_DE output */
		val = PIN_OUTPUT_LSPI_OUTPUT_DIS;
		tegra_dc_writel(dc, val, DC_COM_PIN_OUTPUT_ENABLE3);

		/* enable MSF & set MSF polarity */
		val = MSF_POLARITY_HIGH | MSF_ENABLE | MSF_LSPI;
		tegra_dc_writel(dc, val, DC_CMD_DISPLAY_COMMAND_OPTION0);

		// TE input enbale
		/* enable LSPI/LCD_DE input */
		val = PIN_INPUT_LSPI_INPUT_EN;
		tegra_dc_writel(dc, val, DC_COM_PIN_INPUT_ENABLE3);

		/* set non-continuous mode */
		tegra_dc_writel(dc, DISP_CTRL_MODE_NC_DISPLAY,
						DC_CMD_DISPLAY_COMMAND);

		tegra_dc_writel(dc, GENERAL_UPDATE, DC_CMD_STATE_CONTROL);
		tegra_dc_writel(dc, GENERAL_ACT_REQ | NC_HOST_TRIG,
						DC_CMD_STATE_CONTROL);
	}
	else {
		tegra_dc_writel(dc, DISP_CTRL_MODE_C_DISPLAY, DC_CMD_DISPLAY_COMMAND);
	}
	
	tegra_dc_io_end(dc);
}

void tegra_dc_cpu_disable(struct tegra_dc *dc)
{
	tegra_cpu_stop_dc_stream_at_frame_end(dc);
	
	tegra_dc_writel(dc, 0x00000000, DC_CMD_DISPLAY_POWER_CONTROL);

	tegra_dc_write_table(dc, tegra_dc_cpu_disable_pintable);
}

struct tegra_dc_out_ops tegra_dc_cpu_ops = {
	.enable = tegra_dc_cpu_enable,
	.disable = tegra_dc_cpu_disable,
};

