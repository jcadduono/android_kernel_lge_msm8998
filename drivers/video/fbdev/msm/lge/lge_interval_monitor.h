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

#ifndef LGE_INTERVAL_MONITOR_H
#define LGE_INTERVAL_MONITOR_H

#include <linux/workqueue.h>
#include <linux/mutex.h>
#include "../mdss_mdp.h"

#define MIN_INTERVAL (USEC_PER_SEC/60)
#define MAX_INTERVAL (USEC_PER_SEC/1)
#define BOOST_INTERVAL (0)

enum {
	INTERVAL_EVENT_NOTIFY,
	INTERVAL_EVENT_BLOCK,
	INTERVAL_EVENT_MAX,
};

struct interval_monitor {
	ktime_t pre_ts;
	ktime_t cur_ts;
	u32 interval;
	u16 status_bits;
	atomic_t interval_pending;
	wait_queue_head_t interval_wait_q;
	int input_boost;
	int req_cnt;
	struct mutex interval_lock;
};

struct display_info {
	int init;
	struct fb_info **fbi_list;
	struct msm_fb_data_type *mfd;
	struct mdss_overlay_private *mdp5_data;
	struct mdss_mdp_ctl *ctl;
	struct mdss_panel_data *pdata;
};

struct display_info *lge_interval_get_display_info(void);
int lge_interval_enable(int enable);
int lge_interval_notify(ktime_t cur_us);
int lge_interval_input_notify(void);
int lge_interval_panel_power_notify(int on);
int lge_interval_init(void);
void lge_interval_block(int block);
void lge_interval_notifier_register(struct notifier_block *nb);
void lge_interval_notifier_unregister(struct notifier_block *nb);
void lge_interval_notifier_call_chain(int event, void *v);

#endif /* LGE_INTERVAL_MONITOR_H */
