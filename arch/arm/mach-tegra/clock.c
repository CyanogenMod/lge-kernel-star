/*
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
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
#include <linux/clk.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <asm/clkdev.h>

#include "clock.h"

static int clk_debugfs_reparent(struct clk *c);

static LIST_HEAD(clocks);
static DEFINE_MUTEX(clocks_mutex);

struct clk *get_tegra_clock_by_name(const char *name)
{
	struct clk *c;
	mutex_lock(&clocks_mutex);
	list_for_each_entry(c, &clocks, node) {
		if (strcmp(c->name, name) == 0) {
			mutex_unlock(&clocks_mutex);
			return c;
		}
	}
	mutex_unlock(&clocks_mutex);
	return NULL;
}

static int clk_reparent(struct clk *c, struct clk *parent)
{
	if (c->sibling.next && c->sibling.prev)
		list_del_init(&c->sibling);
	if (parent)
		list_add(&c->sibling, &parent->children);
	clk_debugfs_reparent(c);
	return 0;
}

static void propagate_rate(struct clk *c)
{
	struct clk *clkp;
	pr_debug("%s: %s\n", __func__, c->name);
	list_for_each_entry(clkp, &c->children, sibling) {
		pr_debug("   %s\n", clkp->name);
		if (clkp->ops->recalculate_rate)
			clkp->ops->recalculate_rate(clkp);
		propagate_rate(clkp);
	}
}

static int clk_register(struct clk *c)
{
	mutex_lock(&clocks_mutex);
	list_add(&c->node, &clocks);
	mutex_unlock(&clocks_mutex);
	return 0;
}

void clk_init(struct clk *c)
{
	spin_lock_init(&c->lock);
	INIT_LIST_HEAD(&c->children);
	if (c->ops && c->ops->init)
		c->ops->init(c);
	clk_register(c);
	if (c->parent)
		list_add(&c->sibling, &c->parent->children);
}

int clk_enable(struct clk *c)
{
	int ret = 0;
	unsigned long flags;
	spin_lock_irqsave(&c->lock, flags);
	if (c->refcnt == 0) {
		if (c->parent) {
			ret = clk_enable(c->parent);
			if (ret)
				goto err;
		}

		if (c->ops && c->ops->enable) {
			ret = c->ops->enable(c);
			if (ret) {
				if (c->parent)
					clk_disable(c->parent);
				goto err;
			}
		}
	}
	c->refcnt++;
err:
	spin_unlock_irqrestore(&c->lock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *c)
{
	unsigned long flags;
	spin_lock_irqsave(&c->lock, flags);
	if (c->refcnt == 0) {
		WARN(1, "Attempting to disable clock %s with refcnt 0", c->name);
		goto err;
	}
	if (c->refcnt == 1) {
		if (c->parent)
			clk_disable(c->parent);

		if (c->ops && c->ops->disable)
			c->ops->disable(c);
	}
	c->refcnt--;
err:
	spin_unlock_irqrestore(&c->lock, flags);
}
EXPORT_SYMBOL(clk_disable);

int clk_set_parent(struct clk *c, struct clk *parent)
{
	unsigned long flags;
	int ret = 0;
	spin_lock_irqsave(&c->lock, flags);
	if (c->ops && c->ops->set_parent)
		ret = c->ops->set_parent(c, parent);
	else
		ret = -ENOSYS;
	clk_reparent(c, c->parent);
	propagate_rate(c);
	spin_unlock_irqrestore(&c->lock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *c)
{
	return c->parent;
}
EXPORT_SYMBOL(clk_get_parent);

int clk_set_rate(struct clk *c, unsigned long rate)
{
	int ret = 0;
	unsigned long flags;
	spin_lock_irqsave(&c->lock, flags);
	if (c->ops && c->ops->set_rate)
		ret = c->ops->set_rate(c, rate);
	else
		ret = -ENOSYS;
	propagate_rate(c);
	spin_unlock_irqrestore(&c->lock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

unsigned long clk_get_rate(struct clk *c)
{
	int ret = 0;
	unsigned long flags;
	spin_lock_irqsave(&c->lock, flags);
	ret = c->rate;
	spin_unlock_irqrestore(&c->lock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_get_rate);

void clk_enable_init_clocks(void)
{
	/*struct clk *clkp;

	list_for_each_entry(clkp, &clocks, node) {
		if (clkp->flags & ENABLE_ON_INIT)
			clk_enable(clkp);
	}*/
}
EXPORT_SYMBOL(clk_enable_init_clocks);

int __init tegra_init_clock(void)
{
	tegra2_init_clocks();

	clk_enable_init_clocks();
	return 0;
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *clk_debugfs_root;

static int clk_debugfs_register_one(struct clk *c)
{
	int err;
	struct dentry *d, *child, *child_tmp;
	struct clk *pa = c->parent;
	char s[255];
	char *p = s;

	p += sprintf(p, "%s", c->name);
	d = debugfs_create_dir(s, pa ? pa->dent : clk_debugfs_root);
	if (!d)
		return -ENOMEM;
	c->dent = d;

	d = debugfs_create_u8("refcnt", S_IRUGO, c->dent, (u8 *)&c->refcnt);
	if (!d) {
		err = -ENOMEM;
		goto err_out;
	}
	d = debugfs_create_u32("rate", S_IRUGO, c->dent, (u32 *)&c->rate);
	if (!d) {
		err = -ENOMEM;
		goto err_out;
	}
	d = debugfs_create_x32("flags", S_IRUGO, c->dent, (u32 *)&c->flags);
	if (!d) {
		err = -ENOMEM;
		goto err_out;
	}
	return 0;

err_out:
	d = c->dent;
	list_for_each_entry_safe(child, child_tmp, &d->d_subdirs, d_u.d_child)
		debugfs_remove(child);
	debugfs_remove(c->dent);
	return err;
}

static int clk_debugfs_register(struct clk *c)
{
	int err;
	struct clk *pa = c->parent;

	if (pa && !pa->dent) {
		err = clk_debugfs_register(pa);
		if (err)
			return err;
	}

	if (!c->dent) {
		err = clk_debugfs_register_one(c);
		if (err)
			return err;
	}
	return 0;
}

static int __init clk_debugfs_init(void)
{
	struct clk *c;
	struct dentry *d;
	int err;

	d = debugfs_create_dir("clock", NULL);
	if (!d)
		return -ENOMEM;
	clk_debugfs_root = d;

	list_for_each_entry(c, &clocks, node) {
		err = clk_debugfs_register(c);
		if (err)
			goto err_out;
	}
	return 0;
err_out:
	debugfs_remove_recursive(clk_debugfs_root);
	return err;
}
late_initcall(clk_debugfs_init);
#endif

static int clk_debugfs_reparent(struct clk *c)
{
	int ret = 0;
	pr_debug("%s: %s %p %p %s\n", __func__, c->name, c->dent, c->parent,
		c->parent ? c->parent->name : NULL);
#ifdef CONFIG_DEBUG_FS
	debugfs_remove(c->dent);
	ret = clk_debugfs_register_one(c);
#endif
	return ret;
}
