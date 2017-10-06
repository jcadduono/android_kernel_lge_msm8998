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

#define pr_fmt(fmt)     "[Display] %s: " fmt, __func__

#include "../mdss_fb.h"
#include "lge_comfort_view.h"
#include <linux/delay.h>

static char *digital_gamma_preset_cmds[] = {
	"lge,digital-gamma-cmds-7000K",
	"lge,digital-gamma-cmds-6500K",
	"lge,digital-gamma-cmds-6000K",
	"lge,digital-gamma-cmds-5500K",
	"lge,digital-gamma-cmds-5000K",
	"lge,digital-gamma-cmds-4500K",
	"lge,digital-gamma-cmds-4000K",
	"lge,digital-gamma-cmds-3500K",
	"lge,digital-gamma-cmds-3000K",
	"lge,digital-gamma-cmds-2500K",
	"lge,digital-gamma-cmds-dummy",
};

extern struct msm_fb_data_type *mfd_primary_base;
extern struct mdss_panel_data *pdata_base;
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds, u32 flags);

static ssize_t comfort_view_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", lge_ctrl_pdata->comfort_view);
}

static ssize_t comfort_view_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (pdata_base == NULL || mfd_primary_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	lge_ctrl_pdata->comfort_view = input % 100;

	pr_info("comfort_view [%d]\n", lge_ctrl_pdata->comfort_view);

	mutex_lock(&mfd_primary_base->mdss_sysfs_lock);
	if (lge_ctrl_pdata->comfort_view == 0) { /* disable */
		if (pdata_base->panel_info.panel_power_state == 0) {
			pr_info("Panel off state. keep color_manager_status set cmd. This will be sent panel on event.\n");
			lge_set_screen_mode(ctrl, false);
		} else {
			lge_set_screen_mode(ctrl, true);
		}
	} else if (lge_ctrl_pdata->comfort_view > 0 && lge_ctrl_pdata->comfort_view <= 10) {
		lge_ctrl_pdata->dgc_status = 0x01;
		if (pdata_base->panel_info.panel_power_state == 0) {
			lge_display_control_store(ctrl, false);
		} else {
			lge_mdss_comfort_view_cmd_send(pdata_base, lge_ctrl_pdata->comfort_view);
			lge_display_control_store(ctrl, true);
		}
	}
	mutex_unlock(&mfd_primary_base->mdss_sysfs_lock);
	return ret;
}

static DEVICE_ATTR(comfort_view, S_IRUGO|S_IWUSR|S_IWGRP,
				comfort_view_get, comfort_view_set);

int lge_mdss_comfort_view_create_sysfs(struct device *panel_sysfs_dev)
{
	int rc = 0;
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_comfort_view)) < 0)
		pr_err("add comfort_view set node fail!");
	return rc;
}

void lge_mdss_comfort_view_parse_dt(struct device_node *node,struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int i;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (ctrl_pdata == NULL) {
		pr_err("Invaild input\n");
		return;
	}

	lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input data\n");
		return;
	}

	for (i = 0 ; i < DIGITAL_GAMMA_PRESET_NUMS ; i++) {
		mdss_dsi_parse_dcs_cmds(node, &lge_ctrl_pdata->dg_preset_cmds[i],
			digital_gamma_preset_cmds[i], "qcom,mode-control-dsi-state");
	}
}

int lge_mdss_comfort_view_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (ctrl_pdata == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	pr_info("[Comfort View] init start\n");
	lge_ctrl_pdata->comfort_view = 0;
	lge_ctrl_pdata->dg_preset_cmds = kzalloc(sizeof(struct dsi_panel_cmds) *
					DIGITAL_GAMMA_PRESET_NUMS, GFP_KERNEL);

	if (!lge_ctrl_pdata->dg_preset_cmds) {
		pr_err("[Comfort View] failed to get memory\n");
		return -ENOMEM;
	}
	return 0;
}

int lge_mdss_comfort_view_cmd_send(struct mdss_panel_data *pdata, int step)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo;

	if (pdata == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}
	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	pr_info("[Comfort View] comfort view step = (%d)\n", step);

	mdss_dsi_panel_cmds_send(ctrl, &lge_ctrl_pdata->dg_preset_cmds[step-1], CMD_REQ_COMMIT);

	return 0;
}
