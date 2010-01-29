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
#include <asm/clkdev.h>

#include "clock.h"

static LIST_HEAD(clocks);
static DEFINE_MUTEX(clocks_mutex);

/**
 * clk_preinit - initialize any fields in the struct clk before clk init
 * @clk: struct clk * to initialize
 *
 * Initialize any struct clk fields needed before normal clk initialization
 * can run.  No return value.
 */
void clk_preinit(struct clk *c)
{
	spin_lock_init(&c->lock);
}

int clk_register(struct clk *c)
{
	mutex_lock(&clocks_mutex);
	list_add(&c->node, &clocks);
	mutex_unlock(&clocks_mutex);
	return 0;
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
	spin_unlock_irqrestore(&c->lock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

unsigned long clk_get_rate(struct clk *c)
{
	int ret = 0;
	unsigned long flags;
	spin_lock_irqsave(&c->lock, flags);
	if (c->ops && c->ops->get_rate)
		ret = c->ops->get_rate(c);
	else if (c->parent)
		ret = clk_get_rate(c->parent);
	else
		ret = -ENOSYS;
	spin_unlock_irqrestore(&c->lock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_get_rate);

void clk_enable_init_clocks(void)
{
	struct clk *clkp;

	list_for_each_entry(clkp, &clocks, node) {
		if (clkp->flags & ENABLE_ON_INIT)
			clk_enable(clkp);
	}
}
EXPORT_SYMBOL(clk_enable_init_clocks);

int __init tegra_init_clock(void)
{
	int i;
	struct clk_lookup *cl;
	struct clk *c;

	for (cl = tegra_clk_lookups; cl->clk != NULL; cl++) {
		clk_preinit(cl->clk);
		if (cl->clk->ops && cl->clk->ops->init)
			cl->clk->ops->init(cl->clk);
		clkdev_add(cl);
		clk_register(cl->clk);
	}

	for (i = 0; i < tegra_num_periph_clks; i++) {
		c = &tegra_periph_clks[i];
		cl = &tegra_periph_clk_lookups[i];
		cl->dev_id = c->dev_id;
		cl->con_id = c->con_id;
		cl->clk = c;

		clk_preinit(cl->clk);
		if (cl->clk->ops && cl->clk->ops->init)
			cl->clk->ops->init(cl->clk);
		clkdev_add(cl);
		clk_register(cl->clk);
	}

	clk_enable_init_clocks();
	return 0;
}
