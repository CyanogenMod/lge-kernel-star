/*
 * linux/arch/arm/mach-tegra/pinmux.c
 *
 * Copyright (C) 2010 Google, Inc.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/io.h>

#include <mach/iomap.h>
#include <mach/pinmux.h>

static const struct tegra_pingroup_desc *pingroups = NULL;

static char *tegra_mux_names[TEGRA_MAX_MUX] = {
	[TEGRA_MUX_AHB_CLK] = "AHB_CLK",
	[TEGRA_MUX_APB_CLK] = "APB_CLK",
	[TEGRA_MUX_AUDIO_SYNC] = "AUDIO_SYNC",
	[TEGRA_MUX_CRT] = "CRT",
	[TEGRA_MUX_DAP1] = "DAP1",
	[TEGRA_MUX_DAP2] = "DAP2",
	[TEGRA_MUX_DAP3] = "DAP3",
	[TEGRA_MUX_DAP4] = "DAP4",
	[TEGRA_MUX_DAP5] = "DAP5",
	[TEGRA_MUX_DISPLAYA] = "DISPLAYA",
	[TEGRA_MUX_DISPLAYB] = "DISPLAYB",
	[TEGRA_MUX_EMC_TEST0_DLL] = "EMC_TEST0_DLL",
	[TEGRA_MUX_EMC_TEST1_DLL] = "EMC_TEST1_DLL",
	[TEGRA_MUX_GMI] = "GMI",
	[TEGRA_MUX_GMI_INT] = "GMI_INT",
	[TEGRA_MUX_HDMI] = "HDMI",
	[TEGRA_MUX_I2C] = "I2C",
	[TEGRA_MUX_I2C2] = "I2C2",
	[TEGRA_MUX_I2C3] = "I2C3",
	[TEGRA_MUX_IDE] = "IDE",
	[TEGRA_MUX_IRDA] = "IRDA",
	[TEGRA_MUX_KBC] = "KBC",
	[TEGRA_MUX_MIO] = "MIO",
	[TEGRA_MUX_MIPI_HS] = "MIPI_HS",
	[TEGRA_MUX_NAND] = "NAND",
	[TEGRA_MUX_OSC] = "OSC",
	[TEGRA_MUX_OWR] = "OWR",
	[TEGRA_MUX_PCIE] = "PCIE",
	[TEGRA_MUX_PLLA_OUT] = "PLLA_OUT",
	[TEGRA_MUX_PLLC_OUT1] = "PLLC_OUT1",
	[TEGRA_MUX_PLLM_OUT1] = "PLLM_OUT1",
	[TEGRA_MUX_PLLP_OUT2] = "PLLP_OUT2",
	[TEGRA_MUX_PLLP_OUT3] = "PLLP_OUT3",
	[TEGRA_MUX_PLLP_OUT4] = "PLLP_OUT4",
	[TEGRA_MUX_PWM] = "PWM",
	[TEGRA_MUX_PWR_INTR] = "PWR_INTR",
	[TEGRA_MUX_PWR_ON] = "PWR_ON",
	[TEGRA_MUX_RTCK] = "RTCK",
	[TEGRA_MUX_SDIO1] = "SDIO1",
	[TEGRA_MUX_SDIO2] = "SDIO2",
	[TEGRA_MUX_SDIO3] = "SDIO3",
	[TEGRA_MUX_SDIO4] = "SDIO4",
	[TEGRA_MUX_SFLASH] = "SFLASH",
	[TEGRA_MUX_SPDIF] = "SPDIF",
	[TEGRA_MUX_SPI1] = "SPI1",
	[TEGRA_MUX_SPI2] = "SPI2",
	[TEGRA_MUX_SPI2_ALT] = "SPI2_ALT",
	[TEGRA_MUX_SPI3] = "SPI3",
	[TEGRA_MUX_SPI4] = "SPI4",
	[TEGRA_MUX_TRACE] = "TRACE",
	[TEGRA_MUX_TWC] = "TWC",
	[TEGRA_MUX_UARTA] = "UARTA",
	[TEGRA_MUX_UARTB] = "UARTB",
	[TEGRA_MUX_UARTC] = "UARTC",
	[TEGRA_MUX_UARTD] = "UARTD",
	[TEGRA_MUX_UARTE] = "UARTE",
	[TEGRA_MUX_ULPI] = "ULPI",
	[TEGRA_MUX_VI] = "VI",
	[TEGRA_MUX_VI_SENSOR_CLK] = "VI_SENSOR_CLK",
	[TEGRA_MUX_XIO] = "XIO",
};

static DEFINE_SPINLOCK(mux_lock);

static int tristate_refcount[TEGRA_MAX_PINGROUP];

static const char *pingroup_name(tegra_pingroup_t pg)
{
	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return "<UNKNOWN>";

	return pingroups[pg].name;
}

static const char *func_name(tegra_mux_func_t func)
{
	if (func == TEGRA_MUX_RSVD1)
		return "RSVD1";

	if (func == TEGRA_MUX_RSVD2)
		return "RSVD2";

	if (func == TEGRA_MUX_RSVD3)
		return "RSVD3";

	if (func == TEGRA_MUX_RSVD4)
		return "RSVD4";

	if (func == TEGRA_MUX_NONE)
		return "NONE";

	if (func < 0 || func >=  TEGRA_MAX_MUX)
		return "<UNKNOWN>";

	return tegra_mux_names[func];
}


static const char *tri_name(unsigned long val)
{
	return val ? "TRISTATE" : "NORMAL";
}

static const char *pupd_name(unsigned long val)
{
	switch (val) {
	case 0:
		return "NORMAL";

	case 1:
		return "PULL_DOWN";

	case 2:
		return "PULL_UP";

	default:
		return "RSVD";
	}
}


static inline unsigned long pg_readl(unsigned long offset)
{
	return readl(IO_TO_VIRT(TEGRA_APB_MISC_BASE) + offset);
}

static inline void pg_writel(unsigned long value, unsigned long offset)
{
	writel(value, IO_TO_VIRT(TEGRA_APB_MISC_BASE) + offset);
}

static int tegra_pinmux_cancel_func(const struct tegra_pingroup_config *config)
{
	int mux = -1;
	int mux_safe = -1;
	int i;
	unsigned long reg;
	unsigned long flags;
	tegra_pingroup_t pg = config->pingroup;
	tegra_mux_func_t func = config->func;

	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return -ERANGE;

	if (pingroups[pg].mux_reg < 0)
		return -EINVAL;

	if (func < 0)
		return -ERANGE;

	if (func & TEGRA_MUX_RSVD) {
		return 0;
	}

	for (i = 0; i < 4; i++) {
		if (pingroups[pg].funcs[i] == func) {
			mux = i;
			break;
		}
	}

	if (pingroups[pg].func_safe & TEGRA_MUX_RSVD) {
		mux_safe = pingroups[pg].func_safe & 3;
	} else {
		for (i = 0; i < 4; i++) {
			if (pingroups[pg].funcs[i] == pingroups[pg].func_safe) {
				mux_safe = i;
				break;
			}
		}
	}

	if (mux < 0 || mux_safe < 0)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(pingroups[pg].mux_reg);

	if (((reg >> pingroups[pg].mux_bit) & 0x3) == mux) {
		reg &= ~(0x3 << pingroups[pg].mux_bit);
		reg |= mux_safe << pingroups[pg].mux_bit;
		pg_writel(reg, pingroups[pg].mux_reg);
	}

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

static int tegra_pinmux_set_func(const struct tegra_pingroup_config *config)
{
	int mux = -1;
	int i;
	unsigned long reg;
	unsigned long flags;
	tegra_pingroup_t pg = config->pingroup;
	tegra_mux_func_t func = config->func;

	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return -ERANGE;

	if (pingroups[pg].mux_reg < 0)
		return -EINVAL;

	if (func < 0)
		return -ERANGE;

	if (func & TEGRA_MUX_RSVD) {
		mux = func & 0x3;
	} else {
		for (i = 0; i < 4; i++) {
			if (pingroups[pg].funcs[i] == func) {
				mux = i;
				break;
			}
		}
	}

	if (mux < 0)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(pingroups[pg].mux_reg);
	reg &= ~(0x3 << pingroups[pg].mux_bit);
	reg |= mux << pingroups[pg].mux_bit;
	pg_writel(reg, pingroups[pg].mux_reg);

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

int tegra_pinmux_get_vddio(tegra_pingroup_t pg)
{
	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return -EINVAL;
	return pingroups[pg].vddio;
}

int tegra_pinmux_get_tristate(tegra_pingroup_t pg)
{
	unsigned long reg;

	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return 0;

	if (pingroups[pg].tri_reg < 0)
		return 0;

	reg = pg_readl(pingroups[pg].tri_reg);
	if (reg & (1 << pingroups[pg].tri_bit))
		return 0;
	else
		return 1;
}

int tegra_pinmux_set_tristate(tegra_pingroup_t pg, tegra_tristate_t tristate)
{
	unsigned long reg;
	unsigned long flags;
	bool is_update  = false;

	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return -ERANGE;

	if (pingroups[pg].tri_reg < 0)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	if (tristate == TEGRA_TRI_NORMAL) {
		is_update = (tristate_refcount[pg] == 0);
		tristate_refcount[pg]++;
	} else {
		is_update = (tristate_refcount[pg] == 1);
		if (tristate_refcount[pg] > 0) {
			tristate_refcount[pg]--;
		}
	}
	if (is_update) {
		reg = pg_readl(pingroups[pg].tri_reg);
		reg &= ~(0x1 << pingroups[pg].tri_bit);
		if (tristate)
			reg |= 1 << pingroups[pg].tri_bit;
		pg_writel(reg, pingroups[pg].tri_reg);
	}

	spin_unlock_irqrestore(&mux_lock, flags);
	return 0;
}

int tegra_pinmux_set_pullupdown(tegra_pingroup_t pg, tegra_pullupdown_t pupd)
{
	unsigned long reg;
	unsigned long flags;

	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return -ERANGE;

	if (pingroups[pg].pupd_reg < 0)
		return -EINVAL;

	if (pupd != TEGRA_PUPD_NORMAL &&
	    pupd != TEGRA_PUPD_PULL_DOWN &&
	    pupd != TEGRA_PUPD_PULL_UP)
		return -EINVAL;


	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(pingroups[pg].pupd_reg);
	reg &= ~(0x3 << pingroups[pg].pupd_bit);
	reg |= pupd << pingroups[pg].pupd_bit;
	pg_writel(reg, pingroups[pg].pupd_reg);

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

static void tegra_pinmux_config_pingroup(const struct tegra_pingroup_config *config)
{
	tegra_pingroup_t pingroup = config->pingroup;
	tegra_mux_func_t func     = config->func;
	tegra_pullupdown_t pupd   = config->pupd;
	tegra_tristate_t tristate = config->tristate;
	int err;

	if (pingroups[pingroup].mux_reg >= 0) {
		err = tegra_pinmux_set_func(config);
		if (err < 0)
			pr_err("pinmux: can't set pingroup %s func to %s: %d\n",
			       pingroup_name(pingroup), func_name(func), err);
	}

	if (pingroups[pingroup].pupd_reg >= 0) {
		err = tegra_pinmux_set_pullupdown(pingroup, pupd);
		if (err < 0)
			pr_err("pinmux: can't set pingroup %s pullupdown to %s: %d\n",
			       pingroup_name(pingroup), pupd_name(pupd), err);
	}

	if (pingroups[pingroup].tri_reg >= 0) {
		err = tegra_pinmux_set_tristate(pingroup, tristate);
		if (err < 0)
			pr_err("pinmux: can't set pingroup %s tristate to %s: %d\n",
			       pingroup_name(pingroup), tri_name(func), err);
	}
}



void tegra_pinmux_config_table(const struct tegra_pingroup_config *config, int len)
{
	int i;

	for (i = 0; i < len; i++)
		tegra_pinmux_config_pingroup(&config[i]);
}

void tegra_pinmux_config_pinmux_table(const struct tegra_pingroup_config *config,
				      int len, bool is_set)
{
	int i;
	int err;

	for (i = 0; i < len; i++) {
		if (pingroups[config[i].pingroup].mux_reg >= 0) {
			if (is_set)
				err = tegra_pinmux_set_func(&config[i]);
			else
				err = tegra_pinmux_cancel_func(&config[i]);
			if (err < 0)
				pr_err("pinmux: can't set pingroup %s func"
					" to %s: %d\n", pingroup_name(config[i].pingroup),
					func_name(config[i].func), err);
		}
	}
}

void tegra_pinmux_config_tristate_table(const struct tegra_pingroup_config *config,
					int len, tegra_tristate_t tristate)
{
	int i;
	int err;
	tegra_pingroup_t pingroup;

	for (i = 0; i < len; i++) {
		pingroup = config[i].pingroup;
		if (pingroups[pingroup].tri_reg >= 0) {
			err = tegra_pinmux_set_tristate(pingroup, tristate);
			if (err < 0)
				pr_err("pinmux: can't set pingroup %s tristate"
					" to %s: %d\n",	pingroup_name(pingroup),
					tri_name(tristate), err);
		}
	}
}

void tegra_pinmux_set_vddio_tristate(tegra_vddio_t vddio,
				     tegra_tristate_t tristate)
{
	int pg;
	for (pg = 0; pg < TEGRA_MAX_PINGROUP; ++pg) {
		if (pingroups[pg].vddio == vddio &&
		    pingroups[pg].tri_reg >= 0) {
			if (tegra_pinmux_set_tristate(pg, tristate)<0)
				pr_err("pinmux: can't set pingroup %s tristate"
				       " to %s\n", pingroup_name(pg),
				       tri_name(tristate));
		}
	}
}

void tegra_pinmux_config_pullupdown_table(const struct tegra_pingroup_config *config,
					  int len, tegra_pullupdown_t pupd)
{
	int i;
	int err;
	tegra_pingroup_t pingroup;

	for (i = 0; i < len; i++) {
		pingroup = config[i].pingroup;
		if (pingroups[pingroup].pupd_reg >= 0) {
			err = tegra_pinmux_set_pullupdown(pingroup, pupd);
			if (err < 0)
				pr_err("pinmux: can't set pingroup %s pullupdown"
					" to %s: %d\n",	pingroup_name(pingroup),
					pupd_name(pupd), err);
		}
	}
}

void tegra_pinmux_init_pingroups(void)
{
	pingroups = tegra_pinmux_get_pingroups();
}

#ifdef	CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>

static void dbg_pad_field(struct seq_file *s, int len)
{
	seq_putc(s, ',');

	while (len-- > -1)
		seq_putc(s, ' ');
}

static int dbg_pinmux_show(struct seq_file *s, void *unused)
{
	int i;
	int len;

	for (i = 0; i < TEGRA_MAX_PINGROUP; i++) {
		unsigned long tri;
		unsigned long mux;
		unsigned long pupd;

		seq_printf(s, "\t{TEGRA_PINGROUP_%s", pingroups[i].name);
		len = strlen(pingroups[i].name);
		dbg_pad_field(s, 5 - len);

		if (pingroups[i].mux_reg < 0) {
			seq_printf(s, "TEGRA_MUX_NONE");
			len = strlen("NONE");
		} 
		else 
		{
			//20100725 younghoon.lee@lge.com for pinmux setting while sleep [LGE_START]
			//mux = (pg_readl(pingroups[i].mux_reg) >>
			//       pingroups[i].mux_bit) & 0x3;
			mux = get_reg_data(i, PIN_MUX_CTL);
			//20100725 younghoon.lee@lge.com for pinmux setting while sleep [LGE_END]
			if (pingroups[i].funcs[mux] == TEGRA_MUX_RSVD) {
				seq_printf(s, "TEGRA_MUX_RSVD%1lu", mux+1);
				len = 5;
			} 
			else 
			{
				seq_printf(s, "TEGRA_MUX_%s", tegra_mux_names[pingroups[i].funcs[mux]]);
				len = strlen(tegra_mux_names[pingroups[i].funcs[mux]]);
			}
		}
		dbg_pad_field(s, 13-len);

		if (pingroups[i].pupd_reg < 0) {
			seq_printf(s, "TEGRA_PUPD_NORMAL");
			len = strlen("NORMAL");
		} else {
			//20100725 younghoon.lee@lge.com for pinmux setting while sleep [LGE_START]
			//pupd = (pg_readl(pingroups[i].pupd_reg) >>
			//	pingroups[i].pupd_bit) & 0x3;
			pupd = get_reg_data(i, PULLUPDOWN);
			//20100725 younghoon.lee@lge.com for pinmux setting while sleep [LGE_END]
			seq_printf(s, "TEGRA_PUPD_%s", pupd_name(pupd));
			len = strlen(pupd_name(pupd));
		}
		dbg_pad_field(s, 9 - len);

		if (pingroups[i].tri_reg < 0) {
			seq_printf(s, "TEGRA_TRI_NORMAL");
		} else {
			//20100725 younghoon.lee@lge.com for pinmux setting while sleep [LGE_START]
			//tri = (pg_readl(pingroups[i].tri_reg) >>
			//       pingroups[i].tri_bit) & 0x1;
			tri = get_reg_data(i, TRISTATE);
			//20100725 younghoon.lee@lge.com for pinmux setting while sleep [LGE_END]
			seq_printf(s, "TEGRA_TRI_%s", tri_name(tri));
		}
		seq_printf(s, "},\n");
	}
	return 0;
}

static int dbg_pinmux_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_pinmux_show, &inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_pinmux_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

//20100725 younghoon.lee@lge.com for pinmux setting while sleep [LGE_START]
#include <asm/uaccess.h>
#include <linux/io.h>

extern int gpio_dbgfs_mod;   // 0=normal, 1=sleep
static int gpio_pinmux_reg_num = 0;	

typedef struct  {
    tegra_pingroup_t pg_tristate;
}pg_info;

static int dbg_open_pinmux_reg_num(struct inode *inode, struct file *file)
{
        file->private_data = inode->i_private;
            return 0;
}
static ssize_t dbg_get_pinmux_reg_num(struct file *file, char __user *userbuf,
                                size_t count, loff_t *ppos)
{
        char buf[64];
        int ret;
        
        ret= snprintf(buf, sizeof(buf) - 1, "Selected pinmux_reg is %d \n", gpio_pinmux_reg_num); 

        return simple_read_from_buffer(userbuf, count, ppos, buf, ret);
}
static ssize_t dbg_set_pinmux_reg_num(struct file *file, const char __user *ubuf,
                                size_t count, loff_t *ppos)
{
        char buf[16];
        long result=0;
        int len, ret;
        memset(buf, 0, sizeof(buf));

        if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
            return -EFAULT;

		ret = strict_strtol(buf, 16, &result);
		pr_info("[Power Debugfs] %s: input = %d \n", __func__, result);
		if ( 0 <= result && result < 17 )
			gpio_pinmux_reg_num = result;
		
        return count;
}
static const struct file_operations fops_pinmux_reg_num = {
    .read = dbg_get_pinmux_reg_num,
    .write = dbg_set_pinmux_reg_num,
    .open = dbg_open_pinmux_reg_num,
};

static int dbg_open_pinmux_reg_value(struct inode *inode, struct file *file)
{
        file->private_data = inode->i_private;
            return 0;
}
static ssize_t dbg_get_pinmux_reg_value(struct file *file, char __user *userbuf,
                                size_t count, loff_t *ppos)
{
        char buf[64];
        int ret;
		unsigned long data;

		if ( gpio_dbgfs_mode == NORMAL_MODE )  
		{
			data = get_reg_data(gpio_pinmux_reg_num, REG_DATA);
			ret= snprintf(buf, sizeof(buf) - 1, " pinmux_reg[%d] 0x%x \n", gpio_pinmux_reg_num, data);
		}
		else
		{
			data = get_reg_data(gpio_pinmux_reg_num, REG_DATA);
			ret= snprintf(buf, sizeof(buf) - 1, " sleep_pinmux_reg[%d] 0x%x \n", gpio_pinmux_reg_num, data);
		}        
         

        return simple_read_from_buffer(userbuf, count, ppos, buf, ret);
}
static ssize_t dbg_set_pinmux_reg_value(struct file *file, const char __user *ubuf,
                                size_t count, loff_t *ppos)
{
        char buf[16];
        long result=0;
        int len, ret;
        memset(buf, 0, sizeof(buf));

        if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
            return -EFAULT;

		ret = strict_strtol(buf, 16, &result);
		pr_info("[Power Debugfs] %s: input = %d \n", __func__, result);

		set_reg_data( gpio_pinmux_reg_num, result, REG_DATA );
		
        return count;
}
static const struct file_operations fops_pinmux_reg_value = {
    .read = dbg_get_pinmux_reg_value,
    .write = dbg_set_pinmux_reg_value,
    .open = dbg_open_pinmux_reg_value,
};


static int dbg_open_pingroup_select(struct inode *inode, struct file *file)
{
        file->private_data = inode->i_private;
            return 0;
}

static ssize_t dbg_get_pingroup_select(struct file *file, char __user *userbuf,
                                size_t count, loff_t *ppos)
{
        pg_info *pg_info = file->private_data;
        char buf[128];
		char buf_name[32];
		char buf_tri[32];
		char buf_func[32];
		char buf_pupd[32];
        int ret;
		int i = pg_info->pg_tristate;
		
		unsigned long tri;
		unsigned long mux;
		unsigned long pupd;
		
		ret = snprintf(buf_name , sizeof(buf_name)-1,  " TEGRA_PINGROUP_%s", pingroups[i].name);
		
		if (pingroups[i].mux_reg < 0) 
		{
			ret = snprintf(buf_func,sizeof(buf_func)-1,  "TEGRA_MUX_NONE");
		} 
		else 
		{
			mux = get_reg_data(i, PIN_MUX_CTL);
			pr_info("[Power Debugfs] %s: mux=%d \n", __func__, mux);
			if (pingroups[i].funcs[mux] == TEGRA_MUX_RSVD) 
			{
				ret = snprintf(buf_func,sizeof(buf_func)-1,  "TEGRA_MUX_RSVD%1lu", mux+1);
			} else {
				ret = snprintf(buf_func,sizeof(buf_func)-1,"TEGRA_MUX_%s",
					   tegra_mux_names[pingroups[i].funcs[mux]]);
			}
		}
		
		if (pingroups[i].pupd_reg < 0) 
		{
			ret = snprintf(buf_pupd,sizeof(buf_pupd)-1, "TEGRA_PUPD_NORMAL");
		} 
		else 
		{
			pupd = get_reg_data(i, PULLUPDOWN);		
			pr_info("[Power Debugfs] %s: pupd=%d \n", __func__, pupd);
			ret = snprintf(buf_pupd,sizeof(buf_pupd)-1, "TEGRA_PUPD_%s", pupd_name(pupd));
		}
		
		if (pingroups[i].tri_reg < 0) 
		{
			ret = snprintf(buf_tri,sizeof(buf_tri), "TEGRA_TRI_NORMAL");
		} 
		else 
		{
			tri = get_reg_data(i, TRISTATE);
			pr_info("[Power Debugfs] %s: tri=%d \n", __func__, tri);
			ret = snprintf(buf_tri,sizeof(buf_tri), "TEGRA_TRI_%s", tri_name(tri));
		}
		
		ret = snprintf(buf,sizeof(buf), "%s : %s, %s, %s\n", buf_name, buf_tri, buf_func, buf_pupd);

        return simple_read_from_buffer(userbuf, count, ppos, buf, ret);
}

static ssize_t dbg_set_pingroup_select(struct file *file, const char __user *ubuf,
                                size_t count, loff_t *ppos)
{
        pg_info *pg_info = file->private_data;
        char buf[16];
        int i;
		int len;

        memset(buf, 0, sizeof(buf));
        
        if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
            return -EFAULT;

		len = strlen(buf);
		buf[len-1]=NULL;
		
		for(i=0; i < TEGRA_MAX_PINGROUP; i++)
		{
			if( strcmp(pingroups[i].name, buf)==0)
				pg_info->pg_tristate = i;
		}

        return count;
}

static const struct file_operations fops_pingroup_select = {
    .read = dbg_get_pingroup_select,
    .write = dbg_set_pingroup_select,
    .open = dbg_open_pingroup_select,
};

static int dbg_open_tristate_enable(struct inode *inode, struct file *file)
{
        file->private_data = inode->i_private;
        return 0;
}

static ssize_t dbg_get_tristate_enable(struct file *file, char __user *userbuf,
                                size_t count, loff_t *ppos)
{
        pg_info *pg_info = file->private_data;
        char buf[64];
        int ret;
		int i=pg_info->pg_tristate;
		unsigned long tri;
		
		if (pingroups[i].tri_reg < 0) {
			pr_info("[Power Debugfs] %s: tri_reg < 0 \n", __func__);
			ret = snprintf( buf,sizeof(buf) - 1, "%S : TEGRA_TRI_NORMAL\n",
				pingroups[pg_info->pg_tristate].name);
		} 
		else 
		{
			tri = get_reg_data(i, TRISTATE);
			pr_info("[Power Debugfs] %s: tri=%d \n", __func__, tri);
			ret = snprintf(buf, sizeof(buf)-1, "%s : TEGRA_TRI_%s (0->nor, 1->tri) \n",
				pingroups[pg_info->pg_tristate].name, tri_name(tri));
		}

        return simple_read_from_buffer(userbuf, count, ppos, buf, ret);
}

static ssize_t dbg_set_tristate_enable(struct file *file, const char __user *ubuf,
                                size_t count, loff_t *ppos)
{
        pg_info *pg_info = file->private_data;
        char buf[16];
		
		long result, ret;
        tegra_pingroup_t pg = pg_info->pg_tristate;		
		
        memset(buf, 0, sizeof(buf));

        if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
            return -EFAULT;
	
		ret = strict_strtol(buf, 16, &result);	
		pr_info("[Power Debugfs] %s: input = %d \n", __func__, result);

		set_reg_data( pg, result, TRISTATE );

        return count;
}

static const struct file_operations fops_tristate_enable = {
    .read = dbg_get_tristate_enable,
    .write = dbg_set_tristate_enable,
    .open = dbg_open_tristate_enable,
};


static int dbg_open_pinmux_enable(struct inode *inode, struct file *file)
{
        file->private_data = inode->i_private;
            return 0;
}

static ssize_t dbg_get_pinmux_enable(struct file *file, char __user *userbuf,
                                size_t count, loff_t *ppos)
{
        pg_info *pg_info = file->private_data;
        char buf[64];
        int ret;
		unsigned long mux;
		tegra_pingroup_t pg = pg_info->pg_tristate;
		
		if (pingroups[pg].mux_reg < 0) {
			pr_info("[Power Debugfs] %s: mux_reg < 0 \n", __func__);
			ret = snprintf( buf,sizeof(buf) - 1, "%S : TEGRA_MUX_NONE\n",
				pingroups[pg_info->pg_tristate].name);
		} 
		else 
		{
			mux = get_reg_data(pg, PIN_MUX_CTL);
			pr_info("[Power Debugfs] %s: mux=%d \n", __func__, mux);
			if (pingroups[pg].funcs[mux] == TEGRA_MUX_RSVD) 
			{
				pr_info("[Power Debugfs] %s: funcs[mux] == TEGRA_MUX_RSVD \n", __func__);
				ret = snprintf(buf,sizeof(buf)-1, "%s : TEGRA_MUX_RSVD%1lu\n",
				   pingroups[pg_info->pg_tristate].name, mux+1);
			} 
			else 
			{
				ret = snprintf(buf,sizeof(buf)-1, "%s : TEGRA_MUX_%s\n",
				   pingroups[pg_info->pg_tristate].name, tegra_mux_names[pingroups[pg].funcs[mux]]);
			}
		}

        return simple_read_from_buffer(userbuf, count, ppos, buf, ret);
}

static ssize_t dbg_set_pinmux_enable(struct file *file, const char __user *ubuf,
                                size_t count, loff_t *ppos)
{
        pg_info *pg_info = file->private_data;
        char buf[16];
		long result, ret;
		unsigned long flags;
		tegra_pingroup_t pg = pg_info->pg_tristate;	 
		
        memset(buf, 0, sizeof(buf));

        if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
            return -EFAULT;
	
		ret = strict_strtol(buf, 16, &result);	
		pr_info("[Power Debugfs] %s: input = %d \n", __func__, result);

	   	spin_lock_irqsave(&mux_lock, flags);
		set_reg_data( pg, result, PIN_MUX_CTL );
		spin_unlock_irqrestore(&mux_lock, flags);

        return count;
}

static const struct file_operations fops_pinmux_enable = {
    .read = dbg_get_pinmux_enable,
    .write = dbg_set_pinmux_enable,
    .open = dbg_open_pinmux_enable,
};


static int dbg_open_PUPD_enable(struct inode *inode, struct file *file)
{
        file->private_data = inode->i_private;
            return 0;
}

static ssize_t dbg_get_PUPD_enable(struct file *file, char __user *userbuf,
                                size_t count, loff_t *ppos)
{
        pg_info *pg_info = file->private_data;
        char buf[64];
        int ret;
        unsigned long pupd;
		tegra_pingroup_t pg = pg_info->pg_tristate;
		
		if (pingroups[pg].pupd_reg < 0) {
			pr_info("[Power Debugfs] %s: pupd_reg < 0 \n", __func__);
			ret = snprintf( buf,sizeof(buf) - 1, "%S : TEGRA_PUPD_NORMAL\n",
				pingroups[pg].name);
		} 
		else
		{
			pupd = get_reg_data(pg, PULLUPDOWN);
			pr_info("[Power Debugfs] %s: pupd=%d \n", __func__, pupd);
			ret = snprintf(buf,sizeof(buf)-1 , "%s : TEGRA_PUPD_%s (0->Nor, 1->P/U, 2->P/D, 3->RSVD) \n" ,
				pingroups[pg].name, pupd_name(pupd));
		}
	
        return simple_read_from_buffer(userbuf, count, ppos, buf, ret);
}

static ssize_t dbg_set_PUPD_enable(struct file *file, const char __user *ubuf,
                                size_t count, loff_t *ppos)
{
        pg_info *pg_info = file->private_data;
        char buf[16];
		long result, ret;
        tegra_pingroup_t pg = pg_info->pg_tristate;
		
        memset(buf, 0, sizeof(buf));

        if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
            return -EFAULT;
		
		ret = strict_strtol(buf, 16, &result);	
		pr_info("[Power Debugfs] %s: input = %d \n", __func__, result);

		set_reg_data( pg, result, PULLUPDOWN);
		
        return count;
}

static const struct file_operations fops_PUPD_enable = {
    .read = dbg_get_PUPD_enable,
    .write = dbg_set_PUPD_enable,
    .open = dbg_open_PUPD_enable,
};


//20100725 younghoon.lee@lge.com for pinmux setting while sleep [LGE_END]

static int __init tegra_pinmux_debuginit(void)
{
	(void) debugfs_create_file("tegra_pinmux", S_IRUGO,
					NULL, NULL, &debug_fops);
	
	//20100725 younghoon.lee@lge.com for pinmux setting while sleep [LGE_START]
	pg_info *buff;
	buff =(pg_info *) kmalloc(sizeof(pg_info),GFP_KERNEL);
	buff->pg_tristate = 2;

	debugfs_create_file("pinmux_reg_num", 0666, NULL, buff, &fops_pinmux_reg_num);
	debugfs_create_file("pinmux_reg_val", 0666, NULL, buff, &fops_pinmux_reg_value);
	debugfs_create_file("pingroup", 0666, NULL, buff, &fops_pingroup_select);
	debugfs_create_file("tristate", 0666, NULL, buff, &fops_tristate_enable);
	debugfs_create_file("pinmux", 0666, NULL, buff, &fops_pinmux_enable);
	debugfs_create_file("pupd", 0666, NULL, buff, &fops_PUPD_enable);
	//20100725 younghoon.lee@lge.com for pinmux setting while sleep [LGE_END]

	return 0;
}
late_initcall(tegra_pinmux_debuginit);
#endif
