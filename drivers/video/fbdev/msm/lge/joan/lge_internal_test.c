/*
 * Copyright(c) 2016, LG Electronics. All rights reserved.
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

#include <linux/of_platform.h>
#include "../../mdss_dsi.h"
#include "../lge_mdss_dsi_panel.h"
#include <linux/delay.h>
#include "lge_mdss_ambient_joan.h"

enum internal_test_cmds_index {
	INTERNAL_TEST_AOD,
	INTERNAL_TEST_ORBIT,
	INTERNAL_TEST_FRC,
	INTERNAL_TEST_POWER_SET1,
	INTERNAL_TEST_POWER_SET2,
	INTERNAL_TEST_POWER_SET3,
	INTERNAL_TEST_INDEX_MAX,
};

static char *internal_test_cmd_names[] = {
	"lge,internal-test-command-aod",
	"lge,internal-test-command-orbit",
	"lge,internal-test-command-frc",
	"lge,internal-test-command-power-set1",
	"lge,internal-test-command-power-set2",
	"lge,internal-test-command-power-set3",
};

extern struct mdss_panel_data *pdata_base;
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_panel_cmds *pcmds, u32 flags);

void mdss_dsi_parse_internal_dcs_cmds(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;

	lge_mdss_dsi_parse_dcs_cmds_by_name_array(np, &lge_ctrl_pdata->internal_test_cmds_list,
					internal_test_cmd_names, INTERNAL_TEST_INDEX_MAX);
}

static ssize_t aod_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;
	struct lge_dsi_cmds_entry *lge_dsi_cmds_list = NULL;
	struct mdss_panel_info *pinfo = NULL;
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	unsigned int aod_state, idx;

	pr_info("+++\n");

	if(pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	pinfo = &pdata_base->panel_info;

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;
	lge_dsi_cmds_list = lge_ctrl_pdata->internal_test_cmds_list;

	if (lge_dsi_cmds_list == NULL) {
		pr_err("Cmd list is not initialized.");
		return -EINVAL;
	}

	sscanf(buf, "%d %d", &aod_state, &idx);

	if (aod_state == 1)
		lge_dsi_cmds_list[INTERNAL_TEST_AOD].lge_dsi_cmds.cmds[0].payload[0] = 0x39;
	else
		lge_dsi_cmds_list[INTERNAL_TEST_AOD].lge_dsi_cmds.cmds[0].payload[0] = 0x38;

	if (idx == 1) idx = 720;
	else if (idx == 2) idx = 1440;
	else idx = 2880;

	if (idx > 0) {
		lge_mdss_ambient_partial_area_set(pinfo, idx);
		mdss_dsi_panel_dsc_pps_send(ctrl, &pdata_base->panel_info);
		lge_mdss_ambient_partial_area_unset(pinfo);
	}

	lge_mdss_dsi_send_dcs_cmds_by_cmd_name(ctrl, lge_dsi_cmds_list,
					INTERNAL_TEST_INDEX_MAX, internal_test_cmd_names[INTERNAL_TEST_AOD]);

	pr_info("---\n");
	return ret;
}

static ssize_t orbit_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;
	struct lge_dsi_cmds_entry *lge_dsi_cmds_list = NULL;
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	unsigned int param[9];
	int i;

	pr_info("+++\n");

	if(pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;
	lge_dsi_cmds_list = lge_ctrl_pdata->internal_test_cmds_list;

	if (lge_dsi_cmds_list == NULL) {
		pr_err("Cmd list is not initialized.");
		return -EINVAL;
	}

	sscanf(buf, "%x %x %x %x %x %x %x %x %x", &param[0], &param[1], &param[2], &param[3], &param[4], &param[5], &param[6], &param[7], &param[8]);

	for (i = 0; i < 9; i++) {
		lge_dsi_cmds_list[INTERNAL_TEST_ORBIT].lge_dsi_cmds.cmds[2].payload[i+1] = param[i];
	}

	lge_mdss_dsi_send_dcs_cmds_by_cmd_name(ctrl, lge_dsi_cmds_list,
					INTERNAL_TEST_INDEX_MAX, internal_test_cmd_names[INTERNAL_TEST_ORBIT]);

	pr_info("---\n");
	return ret;
}

static ssize_t frc_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;
	struct lge_dsi_cmds_entry *lge_dsi_cmds_list = NULL;
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	unsigned int param;

	pr_info("+++\n");

	if(pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;
	lge_dsi_cmds_list = lge_ctrl_pdata->internal_test_cmds_list;

	if (lge_dsi_cmds_list == NULL) {
		pr_err("Cmd list is not initialized.");
		return -EINVAL;
	}

	sscanf(buf, "%d", &param);

	if (param)
		lge_dsi_cmds_list[INTERNAL_TEST_FRC].lge_dsi_cmds.cmds[1].payload[1] = 0x03;
	else
		lge_dsi_cmds_list[INTERNAL_TEST_FRC].lge_dsi_cmds.cmds[1].payload[1] = 0x02;

	lge_mdss_dsi_send_dcs_cmds_by_cmd_name(ctrl, lge_dsi_cmds_list,
					INTERNAL_TEST_INDEX_MAX, internal_test_cmd_names[INTERNAL_TEST_FRC]);

	pr_info("---\n");
	return ret;
}

static ssize_t pwr_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;
	struct lge_dsi_cmds_entry *lge_dsi_cmds_list = NULL;
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	unsigned int param;

	pr_info(" +++\n");

	if(pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;
	lge_dsi_cmds_list = lge_ctrl_pdata->internal_test_cmds_list;

	if (lge_dsi_cmds_list == NULL) {
		pr_err("Cmd list is not initialized.");
		return -EINVAL;
	}

	sscanf(buf, "%d", &param);

	switch (param) {
	case 1:
		lge_mdss_dsi_send_dcs_cmds_by_cmd_name(ctrl, lge_dsi_cmds_list,
					INTERNAL_TEST_INDEX_MAX, internal_test_cmd_names[INTERNAL_TEST_POWER_SET1]);
		break;
	case 2:
		lge_mdss_dsi_send_dcs_cmds_by_cmd_name(ctrl, lge_dsi_cmds_list,
					INTERNAL_TEST_INDEX_MAX, internal_test_cmd_names[INTERNAL_TEST_POWER_SET2]);
		break;
	case 3:
		lge_mdss_dsi_send_dcs_cmds_by_cmd_name(ctrl, lge_dsi_cmds_list,
					INTERNAL_TEST_INDEX_MAX, internal_test_cmd_names[INTERNAL_TEST_POWER_SET3]);
		break;
	default:
		break;
	}

	pr_info(" ---\n");

	return ret;
}
static DEVICE_ATTR(aod, S_IWUSR | S_IWGRP, NULL, aod_set);
static DEVICE_ATTR(orbit, S_IWUSR | S_IWGRP, NULL, orbit_set);
static DEVICE_ATTR(frc, S_IWUSR | S_IWGRP, NULL, frc_set);
static DEVICE_ATTR(power_set, S_IWUSR | S_IWGRP, NULL, pwr_set);

int lge_internal_test_create_sysfs(struct class *panel)
{
	int ret = 0;
	static struct device *sysfs_dev = NULL;

	if (!panel) {
		pr_err("invalid input panel class\n");
		return -EINVAL;
	}

	if (!sysfs_dev) {
		sysfs_dev = device_create(panel, NULL, 0, NULL, "internal");
		if (IS_ERR(sysfs_dev)) {
			pr_err("Failed to create dev!\n");
		}
		else{
			if (device_create_file(sysfs_dev, &dev_attr_aod) < 0)
				pr_err("add internal aod set node fail!");
			if (device_create_file(sysfs_dev, &dev_attr_orbit) < 0)
				pr_err("add internal orbit set node fail!");
			if (device_create_file(sysfs_dev, &dev_attr_frc) < 0)
				pr_err("add internal frc set node fail!");
			if (device_create_file(sysfs_dev, &dev_attr_power_set) < 0)
				pr_err("add internal power set node fail!");
		}
	}

	return ret;
}
