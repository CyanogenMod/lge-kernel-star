/*
 * LGE HW pcb revision driver 
 *
 * Copyright (C) 2011 LGE, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <lge/lge_hw_rev.h>

#define MAX_REV_STR 	4

typedef struct lge_pcb_rev_tbl {
	char 		rev_str[MAX_REV_STR];
	hw_rev_t	rev;
}lge_bcp_rev_tbl_t;

lge_bcp_rev_tbl_t pcb_rev_tbl[REV_MAX] = {
    { "A", 		REV_A},
    { "B", 		REV_B},
    { "C", 		REV_C},
    { "D", 		REV_D},
    { "E", 		REV_E},
    { "F", 		REV_F},
    { "1.0", 	REV_10},
    { "1.1", 	REV_11},
};

static hw_rev_t lge_pcb_revision = REV_11; //default

hw_rev_t get_lge_pcb_revision(void)
{
	return lge_pcb_revision;
}

EXPORT_SYMBOL_GPL(get_lge_pcb_revision);

static int __init lge_hw_rev_setup(char *line)
{
	char line_buf[MAX_REV_STR];
	hw_rev_t i;

	memset(line_buf, 0x0, MAX_REV_STR);

	if(line[0] == '1')
	{
		strncpy(line_buf, line, MAX_REV_STR - 1);
	}
	else if (line[0] >= 'A' && line[0] <= 'F')
	{
		strncpy(line_buf, line, 1);
	}
	else 
	{
		printk("[HW_REV] Unknown HW PCB Revision\n");
	}

	for (i = REV_A ; i < REV_MAX ; i++)
	{
		if(!strcmp(line_buf, pcb_rev_tbl[i].rev_str))
		{
			lge_pcb_revision = pcb_rev_tbl[i].rev;
			printk("[HW_REV] HW PCB Revision is REV_%s\n",pcb_rev_tbl[i].rev_str);
			break;
		}
	}	

	if (i == REV_MAX)
		printk("[HW_REV] No match HW PCB Revision\n");
		

	return 1;
}

__setup("brdrev=", lge_hw_rev_setup);

/*
 * LGE MASS VERSION INFO
 * 20110622, xwolf@lge.com, we can get mass info form userdebug
 */
static int lge_massversion = 1;

int get_lge_massversion (void)
{
	return lge_massversion;
}

static int __init lge_enable_mass(char *line)
{
    if (strcmp (line, "enable") == 0) {
        lge_massversion = 0;
    }
    /*
     * 20110823, xwolf@lge.com. 
     * when the system is in usermode, the kernel should be restarted in panic
     * after 1 sec (panic_timeout = 1)
     */
    else {
        extern int panic_timeout;

        if (!panic_timeout) {
            panic_timeout = 1;
        }
    }
    return 1;
}

__setup("userdebug=", lge_enable_mass);

