/*
 * Copyright(c) 2017, LG Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt)      "[Display] %s: " fmt, __func__

#include "lge_error_detect.h"
#include "lge_mdss_dsi.h"
#include "lge_mdss_dsi_panel.h"

extern struct mdss_panel_data *pdata_base;
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np,
				struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
				struct dsi_panel_cmds *pcmds, u32 flags);

void lge_mdss_dsi_panel_err_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, int link_state)
{
	if ( ctrl->err_mask) {
		ctrl->memory_err_detect_cmds.link_state = link_state;
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->memory_err_detect_cmds, CMD_REQ_COMMIT);
	} else {
		ctrl->esd_detect_cmds.link_state = link_state;
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->esd_detect_cmds, CMD_REQ_COMMIT);
	}
}

void lge_dsi_err_detect_remove(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int irq_gpio = lge_get_extra_gpio(ctrl_pdata, "err_irq");
	int irq = gpio_to_irq(irq_gpio);
	if (ctrl_pdata->err_irq_enabled)
		disable_irq(irq);
	free_irq(irq, ctrl_pdata);
	if (ctrl_pdata->err_int_workq)
		destroy_workqueue(ctrl_pdata->err_int_workq);
}

#define istate core_internal_state__do_not_mess_with_it
void lge_dsi_irq_control(struct mdss_dsi_ctrl_pdata *ctrl_pdata, bool enable)
{
	int irq_gpio = lge_get_extra_gpio(ctrl_pdata, "err_irq");
	int irq = gpio_to_irq(irq_gpio);
	struct irq_desc *desc = irq_to_desc(irq);

	if (enable) {
		if (desc) {
			if (desc->istate & IRQS_PENDING) {
				pr_info("Remove pending irq(%d)\n", irq);
				desc->istate &= ~(IRQS_PENDING);
			}
		}
		ctrl_pdata->err_result = 0;
		enable_irq(irq);
	}
	else {
		disable_irq(irq);
	}
	ctrl_pdata->err_irq_enabled = enable;
	pr_info("enable = %d\n", enable);
}

void lge_dsi_err_detect_parse_dt(struct device_node *np, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->memory_err_detect_cmds,
		"lge,memory-error-detect-command", "lge,mdss-dsi-hs-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->esd_detect_cmds,
		"lge,esd-detect-command", "lge,mdss-dsi-hs-command-state");
}

static ssize_t set_err_mask(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int param;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	if (pdata_base->panel_info.panel_power_state == 0){
		pr_err("Panel off state. Ignore err mask cmd\n");
		return -EINVAL;
	}

	ctrl =  container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	sscanf(buf, "%x", &param);
	ctrl->err_mask = param;

	lge_mdss_dsi_panel_err_cmds_send(ctrl, DSI_HS_MODE);

	pr_info("send CMD %d\n", ctrl->err_mask);
	return ret;
}

static ssize_t get_err_mask(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl =  container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	pr_info("%d\n", ctrl->err_mask);
	return sprintf(buf, "%d\n", ctrl->err_mask);
}

static ssize_t set_err_crash(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int param;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl =  container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	sscanf(buf, "%x", &param);

	ctrl->err_crash = param;
	pr_info("set crash %d\n", ctrl->err_crash);
	return ret;
}

static ssize_t get_err_crash(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl =  container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	pr_info("%d\n", ctrl->err_crash);
	return sprintf(buf, "%d\n", ctrl->err_crash);
}

static ssize_t get_mem_test(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl =  container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	pr_info("0x%x\n", ctrl->err_result);
	return sprintf(buf, "%d\n", ctrl->err_result);
}

static DEVICE_ATTR(err_mask, S_IRUGO | S_IWUSR | S_IWGRP,
					get_err_mask, set_err_mask);
static DEVICE_ATTR(err_crash, S_IRUGO | S_IWUSR | S_IWGRP,
					get_err_crash, set_err_crash);
static DEVICE_ATTR(mem_test, S_IRUGO, get_mem_test, NULL);

int lge_dsi_err_detect_create_sysfs(struct class *panel)
{
	int rc;
	static struct device *error_detect_sysfs_dev = NULL;

	if (!panel) {
		pr_err("Invalid input\n");
		return -EINVAL;;
	}

	if (!error_detect_sysfs_dev) {
		error_detect_sysfs_dev = device_create(panel, NULL, 0, NULL, "error_detect");
		if (IS_ERR(error_detect_sysfs_dev)) {
			pr_err("Failed to create dev(aod_sysfs_dev)!");
		} else {
			if ((rc = device_create_file(error_detect_sysfs_dev, &dev_attr_err_mask)) < 0)
				pr_err("add error mask node fail!");
			if ((rc = device_create_file(error_detect_sysfs_dev, &dev_attr_mem_test)) < 0)
				pr_err("add mem test node fail!");
			if ((rc = device_create_file(error_detect_sysfs_dev, &dev_attr_err_crash)) < 0)
				pr_err("add error crash node fail!");
		}
	}
	return rc;
}

void lge_dsi_err_detect_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int ret;
	int irq;
	int irq_gpio;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (ctrl_pdata == NULL) {
		pr_err("invalid input\n");
		return;
	}

	irq_gpio = lge_get_extra_gpio(ctrl_pdata, "err_irq");

	if (irq_gpio > 0 && gpio_is_valid(irq_gpio)) {
		ret = gpio_request(irq_gpio, "ddic_err_irq_gpio");
		if (ret) {
			pr_err("unable to request gpio [%d] ret=%d\n", irq_gpio, ret);
			goto err_none;
		}
		ret = gpio_direction_input(irq_gpio);
		if (ret) {
			pr_err("unable to set dir for gpio[%d]\n", irq_gpio);
			goto err_irq_gpio;
		}
	} else {
		pr_err("irq gpio not provided\n");
		goto err_none;
	}

	lge_dsi_err_detect_init_sub(ctrl_pdata);
	lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;

	ctrl_pdata->err_int_workq = create_workqueue("mdss_dsi_err_int");
	if (!ctrl_pdata->err_int_workq) {
		pr_warn(" Warning creating workqueue\n");
		goto err_none;
	} else {
		INIT_DELAYED_WORK(&ctrl_pdata->err_int_work, lge_ctrl_pdata->err_work);
	}

	irq = gpio_to_irq(irq_gpio);
	ret = request_threaded_irq(irq, NULL, lge_ctrl_pdata->err_irq_handler,
				   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				   "sw43402-irq", ctrl_pdata);
	if (ret) {
		pr_err("Failed to request irq %d: %d\n", irq, ret);
		goto err_irq_gpio;
	}
	lge_dsi_irq_control(ctrl_pdata, false);
	cancel_delayed_work(&ctrl_pdata->err_int_work);

	ctrl_pdata->err_irq_enabled = true;
	ctrl_pdata->err_result = 0;
	ctrl_pdata->err_mask = 0;
	ctrl_pdata->err_crash = 0;
	ctrl_pdata->is_first_err_mask = true;
	return;
err_irq_gpio:
	disable_irq(irq);
	free_irq(irq, ctrl_pdata);
err_none:
	return;
}
