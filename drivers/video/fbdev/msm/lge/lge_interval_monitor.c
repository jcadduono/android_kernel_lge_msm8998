/*
 * Frame Interval Monitor: Interval based FPS tracking.
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

#include <linux/kallsyms.h>
#include <linux/slab.h>
#include <linux/types.h>
#include "lge_interval_monitor.h"

#define SET_BIT(data, idx)	((data) |= (1 << (idx)))
#define CLR_BIT(data, idx)	((data) &= ~(1 << (idx)))
#define CHECK_BIT(data, bit)	(!((bit) & ((data)^(bit))))

#define GO_ON_NOPANEL_BIT	((BIT(INIT))|(BIT(ENABLE)))
#define GO_ON_BIT		((GO_ON_NOPANEL_BIT)|BIT(PANEL))
#define IDLE_INTERVAL		(33000)

enum status {
	INIT,
	ENABLE,
	PANEL,
	BLOCK,
};

static struct interval_monitor imon;
static struct display_info dinfo;
static struct task_struct *lge_interval_thread;
static BLOCKING_NOTIFIER_HEAD(nb_head);

void lge_interval_notifier_register(struct notifier_block *nb)
{
	blocking_notifier_chain_register(&nb_head, nb);
}

void lge_interval_notifier_unregister(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&nb_head, nb);
}

void lge_interval_notifier_call_chain(int event, void *v)
{
	blocking_notifier_call_chain(&nb_head, event, v);
}

static int lge_interval_add_idle_data(void)
{
	if (!CHECK_BIT(imon.status_bits, BIT(INIT)))
		return -EPERM;

	mutex_lock(&imon.interval_lock);
	if (!CHECK_BIT(imon.status_bits, GO_ON_BIT)) {
		pr_debug("%s: interval monitor is locked:%x\n", __func__,
			 imon.status_bits);
		mutex_unlock(&imon.interval_lock);
		return -EPERM;
	}

	imon.cur_ts = ktime_get();
	imon.interval = ktime_to_us(ktime_sub(imon.cur_ts, imon.pre_ts));
	imon.pre_ts = imon.cur_ts;
	if (imon.interval < MIN_INTERVAL)
		imon.interval = MIN_INTERVAL;
	else if (imon.interval > MAX_INTERVAL)
		imon.interval = MAX_INTERVAL;

	atomic_inc(&imon.interval_pending);
	wake_up_all(&imon.interval_wait_q);
	mutex_unlock(&imon.interval_lock);
	return 0;
}

static int __lge_interval_thread(void *data)
{
	int ret;
	unsigned long timeout;

	timeout = usecs_to_jiffies(IDLE_INTERVAL);
	while (1) {
		ret = wait_event_interruptible_timeout(imon.interval_wait_q,
			atomic_read(&imon.interval_pending) ||
			kthread_should_stop(), timeout);

		if (kthread_should_stop())
			break;

		if (!ret) {
			pr_debug("%s: no frame update\n", __func__);
			lge_interval_add_idle_data();
			timeout = usecs_to_jiffies(MAX_INTERVAL);
		} else {
			timeout = usecs_to_jiffies(IDLE_INTERVAL);
		}
		lge_interval_notifier_call_chain(INTERVAL_EVENT_NOTIFY,
						 &imon.interval);
		atomic_dec(&imon.interval_pending);
		if (!CHECK_BIT(imon.status_bits, BIT(PANEL)))
			break;
	}
	atomic_set(&imon.interval_pending, 0);
	CLR_BIT(imon.status_bits, ENABLE);
	lge_interval_thread = NULL;

	return ret;
}

static int lge_interval_start_thread(void)
{
	int ret = 0;

	pr_debug("%pS: start lge_interval thread\n",
		 __builtin_return_address(0));
	lge_interval_thread = kthread_run(__lge_interval_thread,
					  NULL, "lge_interval");

	if (IS_ERR(lge_interval_thread)) {
		pr_err("ERROR: unable to start lge_interval thread\n");
		ret = PTR_ERR(lge_interval_thread);
		lge_interval_thread = NULL;
	}

	return ret;
}

int lge_interval_notify(ktime_t cur_us)
{
	if (!CHECK_BIT(imon.status_bits, BIT(INIT)))
		return -EPERM;

	mutex_lock(&imon.interval_lock);
	if (!CHECK_BIT(imon.status_bits, GO_ON_BIT)) {
		pr_debug("%s: interval monitor is locked:%x\n", __func__,
			 imon.status_bits);
		mutex_unlock(&imon.interval_lock);
		return -EPERM;
	}

	imon.cur_ts = cur_us;
	if (imon.input_boost > 0) {
		imon.interval = MIN_INTERVAL;
		imon.input_boost = 0;
	} else {
		imon.interval = ktime_to_us(ktime_sub(imon.cur_ts, imon.pre_ts));
	}
	imon.pre_ts = imon.cur_ts;

	if (imon.interval < MIN_INTERVAL)
		imon.interval = MIN_INTERVAL;
	else if (imon.interval > MAX_INTERVAL)
		imon.interval = MAX_INTERVAL;

	atomic_inc(&imon.interval_pending);
	wake_up_all(&imon.interval_wait_q);
	mutex_unlock(&imon.interval_lock);

	return 0;
}

int lge_interval_input_notify(void)
{
	if (!CHECK_BIT(imon.status_bits, BIT(INIT)))
		return -EPERM;

	if (!CHECK_BIT(imon.status_bits, GO_ON_NOPANEL_BIT)) {
		pr_debug("%s: interval monitor is locked:%x\n", __func__,
			 imon.status_bits);
		return -EPERM;
	}

	imon.interval = BOOST_INTERVAL;
	imon.pre_ts = ktime_get();
	imon.input_boost = 1;

	atomic_inc(&imon.interval_pending);
	wake_up_all(&imon.interval_wait_q);

	return 0;
}

int lge_interval_panel_power_notify(int on)
{
	int panel_on;

	if (!CHECK_BIT(imon.status_bits, BIT(INIT)))
		return -EPERM;

	mutex_lock(&imon.interval_lock);
	panel_on = CHECK_BIT(imon.status_bits, BIT(PANEL));
	if (on != panel_on) {
		pr_debug("%s: panel power state: %d->%d\n", __func__,
			 panel_on, on);
		if (on)
			SET_BIT(imon.status_bits, PANEL);
		else
			CLR_BIT(imon.status_bits, PANEL);
	} else {
		pr_debug("%s: panel power state is already updated: %d\n",
			 __func__, on);
		mutex_unlock(&imon.interval_lock);
		return -EPERM;
	}

	if (CHECK_BIT(imon.status_bits, BIT(ENABLE)) && on) {
		mutex_unlock(&imon.interval_lock);
		if (lge_interval_thread == NULL) {
			lge_interval_start_thread();
		}
		mutex_lock(&imon.interval_lock);
	}

	if (!CHECK_BIT(imon.status_bits, GO_ON_NOPANEL_BIT)) {
		pr_debug("%s: interval monitor is locked:%x\n", __func__,
			 imon.status_bits);
		mutex_unlock(&imon.interval_lock);
		return -EPERM;
	}

	imon.interval = BOOST_INTERVAL;
	imon.pre_ts = ktime_get();

	atomic_inc(&imon.interval_pending);
	wake_up_all(&imon.interval_wait_q);
	mutex_unlock(&imon.interval_lock);

	return 0;
}

int lge_interval_enable(int enable)
{
	int ret = 0;
	int enabled;

	if (!CHECK_BIT(imon.status_bits, BIT(INIT)))
		return -EPERM;

	mutex_lock(&imon.interval_lock);
	enabled = CHECK_BIT(imon.status_bits, BIT(ENABLE));
	if (enable) {
		imon.req_cnt++;
	} else {
		if (imon.req_cnt > 0)
			imon.req_cnt--;
	}
	if (enable != enabled) {
		if (imon.req_cnt > 0) {
			SET_BIT(imon.status_bits, ENABLE);
			mutex_unlock(&imon.interval_lock);
			if (lge_interval_thread == NULL) {
				ret = lge_interval_start_thread();
				if (IS_ERR_VALUE(ret))
					return ret;
			}
		} else {
			CLR_BIT(imon.status_bits, ENABLE);
			mutex_unlock(&imon.interval_lock);
		}
	} else {
		mutex_unlock(&imon.interval_lock);
	}

	return ret;
}

void lge_interval_block(int block)
{
	int blocked;

	if (!CHECK_BIT(imon.status_bits, BIT(INIT)))
		return;

	mutex_lock(&imon.interval_lock);
	blocked = CHECK_BIT(imon.status_bits, BIT(BLOCK));
	if (block)
		SET_BIT(imon.status_bits, BLOCK);
	else
		CLR_BIT(imon.status_bits, BLOCK);
	mutex_unlock(&imon.interval_lock);

	if (block != blocked) {
		if (block) {
			lge_interval_notifier_call_chain(INTERVAL_EVENT_BLOCK,
							 &block);
		} else {
			lge_interval_notifier_call_chain(INTERVAL_EVENT_BLOCK,
							 &block);
			lge_interval_input_notify();
		}
	}
}

static int lge_interval_display_info_init(void)
{
	if (!dinfo.init) {
		dinfo.fbi_list =
			(struct fb_info **)kallsyms_lookup_name("fbi_list");
		if (dinfo.fbi_list[0] == NULL)
			return -EFAULT;

		dinfo.mfd = dinfo.fbi_list[0]->par;
		if (dinfo.mfd == NULL || dinfo.mfd->pdev == NULL)
			return -EFAULT;

		dinfo.pdata = dev_get_platdata(&dinfo.mfd->pdev->dev);
		if (dinfo.pdata == NULL)
			return -EFAULT;

		dinfo.mdp5_data = mfd_to_mdp5_data(dinfo.mfd);
		if (dinfo.mdp5_data == NULL)
			return -EFAULT;

		dinfo.ctl = dinfo.mdp5_data->ctl;
		if (dinfo.ctl == NULL)
			return -EFAULT;

		dinfo.init = 1;
	}

	return 0;
}

struct display_info *lge_interval_get_display_info(void)
{
	if (lge_interval_display_info_init() < 0) {
		pr_err("%s: failed to initialize display info\n", __func__);
		return NULL;
	}
	return &dinfo;
}

int lge_interval_init(void)
{
	int ret;

	if (CHECK_BIT(imon.status_bits, BIT(INIT))) {
		pr_debug("%s: interval monitor is already initialized.",
			 __func__);
		return 0;
	}

	ret = lge_interval_display_info_init();
	if (ret < 0) {
		pr_err("%s: failed to initialize display info\n", __func__);
		return ret;
	}

	mutex_init(&imon.interval_lock);
	init_waitqueue_head(&imon.interval_wait_q);
	imon.input_boost = 0;
	imon.req_cnt = 0;
	SET_BIT(imon.status_bits, PANEL);
	SET_BIT(imon.status_bits, INIT);

	return 0;
}
