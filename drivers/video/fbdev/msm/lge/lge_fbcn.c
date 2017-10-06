/*
 * FBCN: Frame Based Clock Normalizer.
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
#include <linux/sysfs.h>
#include <linux/device.h>
#include "lge_interval_monitor.h"
#include "lge_fbcn.h"
#include "../mdss_panel.h"

static struct fbcn_interval_stats fstats;
static struct display_info *dinfo;

static void fbcn_calc_stats(u32 interval)
{
	if (interval == BOOST_INTERVAL)
		fstats.interval = MIN_INTERVAL;
	else
		fstats.interval = (u64)interval;
}

static void lge_fbcn_notify_interval(u32 interval)
{
	struct device *dev;

	mutex_lock(&fstats.fbcn_lock);
	fbcn_calc_stats(interval);
	if (fstats.i_upper != ULONG_MAX
	    &&(fstats.interval > fstats.i_upper
	    || fstats.interval <= fstats.i_lower)) {
		mutex_unlock(&fstats.fbcn_lock);
		dev = dinfo->mfd->fbi->dev;
		if (dev) {
			sysfs_notify(&dev->kobj, NULL, "fbcn_interval");
		} else {
			pr_warn("%s: fb device is null\n", __func__);
		}
	} else {
		mutex_unlock(&fstats.fbcn_lock);
	}

	return;
}

static int lge_fbcn_callback_handler(struct notifier_block *nb,
				     unsigned long event, void *data)
{
	int ret = 0;
	u32 *val = data;

	switch (event) {
	case INTERVAL_EVENT_NOTIFY:
		lge_fbcn_notify_interval(*val);
		break;
	case INTERVAL_EVENT_BLOCK:
		break;
	default:
		break;
	};

	return ret;
}

ssize_t fbcn_en_store(struct device *dev,
		      struct device_attribute *attr, const char *buf,
		      size_t count)
{
	int enable;

	if (sscanf(buf, "%u", &enable) < 1) {
		pr_warn("%s: Failed to store enable\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&fstats.fbcn_lock);
	if (enable != fstats.enable) {
		if (enable) {
			fstats.interval = MIN_INTERVAL;
			fstats.i_upper = ULONG_MAX;
			fstats.i_lower = 0;
			fstats.enable = 1;
		} else {
			fstats.enable = 0;
		}
		mutex_unlock(&fstats.fbcn_lock);
		lge_interval_enable(fstats.enable);
		lge_interval_input_notify();
	} else {
		mutex_unlock(&fstats.fbcn_lock);
	}

	return count;
}

ssize_t fbcn_en_show(struct device *dev,
		     struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%u\n", fstats.enable);
	return ret;
}

ssize_t fbcn_i_store(struct device *dev,
		     struct device_attribute *attr, const char *buf,
		     size_t count)
{
	u64 i_upper, i_lower;

	if (sscanf(buf, "%llu %llu", &i_upper, &i_lower) < 1) {
		pr_warn("%s: Failed to store fbcn interval threshold\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&fstats.fbcn_lock);
	fstats.i_upper = i_upper;
	fstats.i_lower = i_lower;
	mutex_unlock(&fstats.fbcn_lock);
	return count;
}

ssize_t fbcn_i_show(struct device *dev,
		    struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%llu %llu\n", fstats.i_upper, fstats.i_lower);
	return ret;
}

ssize_t fbcn_interval_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%llu\n", fstats.interval);
	return ret;
}

static DEVICE_ATTR(fbcn_en, S_IRUGO | S_IWUSR, fbcn_en_show, fbcn_en_store);
static DEVICE_ATTR(fbcn_interval, S_IRUGO | S_IWUSR, fbcn_interval_show,
		   NULL);
static DEVICE_ATTR(fbcn_i, S_IRUGO | S_IWUSR, fbcn_i_show, fbcn_i_store);

static struct attribute *fbcn_fs_attrs[] = {
	&dev_attr_fbcn_en.attr,
	&dev_attr_fbcn_interval.attr,
	&dev_attr_fbcn_i.attr,
	NULL,
};

static struct attribute_group fbcn_fs_attrs_group = {
	.attrs = fbcn_fs_attrs,
};

static int __init fbcn_init(void)
{
	int ret = 0;
	struct device *dev;

	dinfo = lge_interval_get_display_info();
	if (!dinfo) {
		pr_err("%s: failed to get display info.\n", __func__);
		return -EFAULT;
	}

	dev = dinfo->mfd->fbi->dev;
	ret = sysfs_create_group(&dev->kobj, &fbcn_fs_attrs_group);
	if (ret) {
		pr_err("%s: Error fbcn sysfs creation ret=%d\n", __func__, ret);
		return ret;
	}
	mutex_init(&fstats.fbcn_lock);

	ret = lge_interval_init();
	if (ret < 0) {
		pr_err("%s: Failed to initialize interval monitor\n",
		       __func__);
		return ret;
	}
	fstats.nb.notifier_call = lge_fbcn_callback_handler;
	lge_interval_notifier_register(&fstats.nb);

	return 0;
}

late_initcall(fbcn_init);
