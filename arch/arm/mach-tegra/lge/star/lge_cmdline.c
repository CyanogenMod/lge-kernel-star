/* 
 * arch/arm/mach-omap2/lge/lge_cmdline.c
 *
 * Copyright (C) 2011 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <asm/setup.h>

#define isspace(c)      (c == ' ' || c == '\t' || c == 10 || c == 13 || c == 0)

struct cmdline_parameter {
        char *key;
        char *value; /* If you want to remove key, set as "" */
        int new;     /* if you add new parameter, set as 1 */
};

static struct cmdline_parameter cmdline_parameters[] __initdata = {
        {"nvmem", "", 0},
        {"lpj", "", 0},
        {"mem", "512M@0M", 1},
        {"vmalloc", "128M", 1},
        {"androidboot.mode", "normal", 1},
        {"lp0_vec", "8192@0x1fe5eff0", 1}, // This *needs* to match the
                                           // OLD PMC_SCRATCH1 address (0xd7802960)
        {"tegra_fbmem", "1548288@0x1fe86000", 1},
        {"androidboot.hardware", "star", 1},
};

static char *matchstr(const char *s1, const char *s2, int getvalue)
{
        char *p, *p2;
        bool matched;

        p = s1;
        do {
                p = strstr(p, s2);
                if (p) {
                        matched = false;
                        if (p == s1)
                                matched = true;
                        else if (isspace(*(p-1)))
                                matched = true;

                        if (matched) {
                                p2 = p + strlen(s2);
                                if (isspace(*p2) || *p2 == '=')
					return getvalue ? ++p2 : p;
                        }
                        p += strlen(s2);
                }
        } while (p);

        return NULL;
}

static void __init fill_whitespace(char *s)
{
        char *p;

        for (p = s; !isspace(*p) && *p != '\0'; ++p)
                *p = ' ';
}

void __init lge_manipulate_cmdline(char *default_command_line)
{
        char *s, *p, *p2;
        int i;
        int ws;

        s = default_command_line;

        /* remove parameters */
        for (i = 0; i < ARRAY_SIZE(cmdline_parameters); i++) {
                p = s;
                do {
                        p = matchstr(p, cmdline_parameters[i].key, 0);
                        if (p)
                                fill_whitespace(p);
                } while(p);
        }

        /* trim the whitespace */
        p = p2 = s;
        ws = 0; /* whitespace */
        for (i = 0; i < COMMAND_LINE_SIZE; i++) {
                if (!isspace(p[i])) {
                        if (ws) {
                                ws = 0;
                                *p2++ = ' ';
                        }
                        *p2++ = p[i];
                }
                else {
                        ws = 1;
                }
        }
        *p2 = '\0';
        /* add parameters */
        for (i = 0; i < ARRAY_SIZE(cmdline_parameters); i++) {
                static char param[COMMAND_LINE_SIZE];
                memset(param, 0, COMMAND_LINE_SIZE);
                if (strlen(cmdline_parameters[i].value) ||
                                cmdline_parameters[i].new) {
                        strcat(param, cmdline_parameters[i].key);
                        if (strlen(cmdline_parameters[i].value)) {
                                strcat(param, "=");
                                strcat(param, cmdline_parameters[i].value);
                        }
                }
                strlcat(s, " ", COMMAND_LINE_SIZE);
                strlcat(s, param, COMMAND_LINE_SIZE);
        }
	/* Evil hack: Convert the old uniqueid into the standard serialno */
	{
                static char param[COMMAND_LINE_SIZE];
                memset(param, 0, COMMAND_LINE_SIZE);
                strcat(param, "androidboot.serialno=");
                p = matchstr(p, "uniqueid", 1);
                while (!isspace(*p) && *p) {
			sprintf(param, "%s%c", param, *p++);
		}
                strlcat(s, " ", COMMAND_LINE_SIZE);
                strlcat(s, param, COMMAND_LINE_SIZE);
	}

}

void __init manipulate_cmdline(char *default_command_line,
		const char *tag_command_line, size_t size)
{
	strlcpy(default_command_line, tag_command_line, size);
	lge_manipulate_cmdline(default_command_line);
	pr_info("bootloader  cmdline: %s\n", tag_command_line);
	pr_info("manipulated cmdline: %s\n", default_command_line);
}
