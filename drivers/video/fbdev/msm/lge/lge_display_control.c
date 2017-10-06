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

#include <linux/delay.h>
#include "lge_display_control.h"

extern struct mdss_panel_data *pdata_base;
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_panel_cmds *pcmds, u32 flags);

void mdss_dsi_parse_display_control_dcs_cmds(struct device_node *np,
				struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	if (lge_ctrl_pdata->display_control_parse_dt)
		lge_ctrl_pdata->display_control_parse_dt(np, ctrl_pdata);

	mdss_dsi_parse_dcs_cmds(np, &lge_ctrl_pdata->white_d65_cmds,
		"lge,mdss-dsi-d65-command", "lge,mdss-dsi-hs-command-state");
#if defined(CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT)
	mdss_dsi_parse_dcs_cmds(np, &lge_ctrl_pdata->bc_dim_cmds,
		"lge,mdss-dsi-bc-dim-command", "lge,mdss-dsi-hs-command-state");
	mdss_dsi_parse_dcs_cmds(np, &lge_ctrl_pdata->bc_default_cmds,
		"lge,mdss-dsi-bc-default_command", "lge,mdss-dsi-hs-command-state");
#endif /* CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT */
#if defined(CONFIG_LGE_DISPLAY_COLOR_MANAGER)
	lge_color_manager_parse_dt(np, ctrl_pdata);
#endif /* CONFIG_LGE_DISPLAY_COLOR_MANAGER */
#if defined(CONFIG_LGE_DISPLAY_COMFORT_MODE)
	lge_mdss_comfort_view_parse_dt(np, ctrl_pdata);
#endif /* CONFIG_LGE_DISPLAY_COMFORT_MODE */
}

void lge_display_control_store(struct mdss_dsi_ctrl_pdata *ctrl_pdata, bool cmd_send)
{
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;
	if (ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	if (lge_ctrl_pdata->display_control_store)
		lge_ctrl_pdata->display_control_store(ctrl_pdata, cmd_send);
}

static ssize_t dgc_status_get(struct device *dev,
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
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", lge_ctrl_pdata->dgc_status);
}

static ssize_t dgc_status_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}
	if (pdata_base->panel_info.panel_power_state == 0) {
		pr_err("Panel off state. Ignore sharpness_status set cmd\n");
		return -EINVAL;
	}
	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);
	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	lge_ctrl_pdata->dgc_status = input & 0x01;
	lge_display_control_store(ctrl, true);

	pr_info("dgc_status %d \n", lge_ctrl_pdata->dgc_status);
	return ret;
}
static DEVICE_ATTR(dgc_status, S_IWUSR|S_IRUGO, dgc_status_get, dgc_status_set);

static ssize_t sharpness_status_get(struct device *dev,
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
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", lge_ctrl_pdata->sharpness_status);
}

static ssize_t sharpness_status_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}
	if (pdata_base->panel_info.panel_power_state == 0) {
		pr_err("Panel off state. Ignore sharpness_status set cmd\n");
		return -EINVAL;
	}
	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);
	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	lge_ctrl_pdata->sharpness_status = input & 0x01;
	lge_display_control_store(ctrl, true);

	pr_info("sharpness_status %d \n", lge_ctrl_pdata->sharpness_status);
	return ret;
}
static DEVICE_ATTR(sharpness_status, S_IWUSR|S_IRUGO,
					sharpness_status_get, sharpness_status_set);

static ssize_t boost_status_get(struct device *dev,
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
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", lge_ctrl_pdata->boost_status);
}

static ssize_t boost_status_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}
	if (pdata_base->panel_info.panel_power_state == 0) {
		pr_err("Panel off state. Ignore boost_status set cmd\n");
		return -EINVAL;
	}
	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);
	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	lge_ctrl_pdata->boost_status = input & 0x01;
	lge_display_control_store(ctrl, true);

	pr_info("boost status %d \n", lge_ctrl_pdata->boost_status);
	return ret;
}
static DEVICE_ATTR(boost_status, S_IRUGO | S_IWUSR | S_IWGRP,
					boost_status_get, boost_status_set);

static ssize_t contrast_status_get(struct device *dev,
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
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", lge_ctrl_pdata->contrast_status);
}

static ssize_t contrast_status_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}
	if (pdata_base->panel_info.panel_power_state == 0) {
		pr_err("Panel off state. Ignore contrast_status set cmd\n");
		return -EINVAL;
	}
	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);
	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	lge_ctrl_pdata->contrast_status = input & 0x01;
	lge_display_control_store(ctrl, true);

	pr_info("contrast_status %d \n", lge_ctrl_pdata->contrast_status);
	return ret;
}
static DEVICE_ATTR(contrast_status, S_IWUSR|S_IRUGO,
					contrast_status_get, contrast_status_set);

#if defined(CONFIG_LGE_DISPLAY_VR_MODE)
void mdss_dsi_panel_apply_settings(struct mdss_dsi_ctrl_pdata *ctrl,
	struct dsi_panel_cmds *pcmds)
{
	int vr_enable;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (ctrl == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	vr_enable = (pcmds->cmds[0].payload[1] & 0x08);
	if (vr_enable) {
		lge_ctrl_pdata->vr_status = 0x01;
		lge_display_control_store(ctrl, true);
		pr_info("vr status %d \n", lge_ctrl_pdata->vr_status);
	} else {
		lge_ctrl_pdata->vr_status = 0x00;
		lge_display_control_store(ctrl, true);
		pr_info("vr status %d \n", lge_ctrl_pdata->vr_status);
	}
}
#endif /* CONFIG_LGE_DISPLAY_VR_MODE */

static ssize_t hdr_hbm_lut_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl =  container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);
	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", lge_ctrl_pdata->hdr_hbm_lut);
}

static ssize_t hdr_hbm_lut_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}
	if (pdata_base->panel_info.panel_power_state == 0) {
		pr_err("Panel off state. Ignore hdr_hbm_lut_set cmd\n");
		return -EINVAL;
	}
	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	lge_ctrl_pdata->hdr_hbm_lut = input & 0x03;
	lge_display_control_store(ctrl, true);

	pr_info("hdr_hbm_lut status %d \n", lge_ctrl_pdata->hdr_hbm_lut);
	return ret;
}
static DEVICE_ATTR(hdr_hbm_lut, S_IWUSR|S_IRUGO,
					hdr_hbm_lut_get, hdr_hbm_lut_set);

static ssize_t hdr_mode_get(struct device *dev,
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
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", lge_ctrl_pdata->hdr_mode);
}

static ssize_t hdr_mode_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}
	if (pdata_base->panel_info.panel_power_state == 0) {
		pr_err("Panel off state. Ignore hdr_mode set cmd\n");
		return -EINVAL;
	}
	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	lge_ctrl_pdata->hdr_mode = input & 0x03;
	lge_display_control_store(ctrl, true);

	pr_info("hdr_mode status %d \n", lge_ctrl_pdata->hdr_mode);
	return ret;
}
static DEVICE_ATTR(hdr_mode, S_IWUSR|S_IRUGO, hdr_mode_get, hdr_mode_set);

static ssize_t acl_mode_get(struct device *dev,
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
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", lge_ctrl_pdata->acl_mode);
}

static ssize_t acl_mode_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}
	if (pdata_base->panel_info.panel_power_state == 0) {
		pr_err("Panel off state. Ignore acl_mode set cmd\n");
		return -EINVAL;
	}
	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	lge_ctrl_pdata->acl_mode = input & 0x03;
	lge_display_control_store(ctrl, true);

	pr_info("acl_mode status %d \n", lge_ctrl_pdata->acl_mode);
	return ret;
}
static DEVICE_ATTR(acl_mode, S_IWUSR|S_IRUGO, acl_mode_get, acl_mode_set);

static ssize_t white_point_get(struct device *dev,
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
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%s\n", (lge_ctrl_pdata->white_target ? "D65" : "7500K"));
}

static ssize_t white_point_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}
	if (pdata_base->panel_info.panel_power_state == 0) {
		pr_err("Panel off state. Ignore white point set cmd\n");
		return -EINVAL;
	}
	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	lge_ctrl_pdata->white_target = input & 0x01;
	lge_ctrl_pdata->dgc_status = lge_ctrl_pdata->white_target;

	mdss_dsi_panel_cmds_send(ctrl, &lge_ctrl_pdata->white_d65_cmds, CMD_REQ_COMMIT);
	lge_display_control_store(ctrl, true);

	pr_info("white_target %s \n", lge_ctrl_pdata->white_target ? "D65" : "7500K");
	return ret;
}
static DEVICE_ATTR(white_target, S_IWUSR|S_IRUGO,
					white_point_get, white_point_set);

#if defined(CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT)
extern struct led_classdev backlight_led;
extern void mdss_fb_set_bl_brightness(struct led_classdev *led_cdev,
				      enum led_brightness value);

void lge_bc_dim_reg_backup(struct mdss_dsi_ctrl_pdata *ctrl)
{
	char *data;
	int i;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (ctrl == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	data = &lge_ctrl_pdata->bc_default_cmds.cmds[0].payload[1];
	lge_mdss_dsi_panel_cmd_read(BC_REG, BC_REG_NUM, data);

	for (i = 0; i < BC_REG_NUM ; i++) {
		pr_info("BC_DIM_REG[%d] 0x%x \n", i,
                lge_ctrl_pdata->bc_default_cmds.cmds[0].payload[i]);
	}
	lge_ctrl_pdata->bc_reg_backup_flag = true;
}

void lge_mdss_dsi_bc_dim_work(struct work_struct *work)
{
	struct delayed_work *dw = to_delayed_work(work);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;
	struct mdss_panel_data *pdata;

	lge_ctrl_pdata = container_of(dw, struct lge_mdss_dsi_ctrl_pdata, bc_dim_work);

	if (lge_ctrl_pdata == NULL || pdata_base == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	ctrl_pdata = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	pdata = &ctrl_pdata->panel_data;
	if (!pdata) {
		pr_err("invalid panel data\n");
		return;
	}

	if (mdss_panel_is_power_on_lp(pdata->panel_info.panel_power_state)) {
		cancel_delayed_work(&lge_ctrl_pdata->bc_dim_work);
		pr_info("skip BC Ctrl\n");
	} else {
		lge_ctrl_pdata->bc_dim_cmds.cmds[0].payload[1] = BC_DIM_OFF;
		mdss_dsi_panel_cmds_send(ctrl_pdata, &lge_ctrl_pdata->bc_dim_cmds,
			CMD_REQ_COMMIT);
		pr_info("VE Mode BC: 0x%x \n",
			lge_ctrl_pdata->bc_dim_cmds.cmds[0].payload[1]);
	}
	return;
}

static ssize_t video_enhancement_get(struct device *dev,
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
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", lge_ctrl_pdata->video_enhancement);
}

static ssize_t video_enhancement_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}
	if (pdata_base->panel_info.panel_power_state == 0) {
		pr_err("Panel off state. Ignore screen_mode set cmd\n");
		return -EINVAL;
	}
	ctrl =  container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);
	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);
	lge_ctrl_pdata->video_enhancement = input;

	pdata_base->panel_info.ve_mode_on = input;

	lge_ctrl_pdata->hdr_mode = input & 0x03;
	lge_display_control_store(ctrl, true);
	pr_info("send cmds to %s the video enhancer \n",
	        (input == true) ? "enable" : "disable");

	/* VE Brightness Control*/
	if (!lge_ctrl_pdata->bc_reg_backup_flag) {
		lge_bc_dim_reg_backup(ctrl);
		lge_ctrl_pdata->bc_default_cmds.cmds[0].payload[22] = BC_DIM_FRAMES;
	}

	lge_ctrl_pdata->bc_dim_cmds.cmds[0].payload[1] = BC_DIM_ON;
	mdss_dsi_panel_cmds_send(ctrl, &lge_ctrl_pdata->bc_dim_cmds, CMD_REQ_COMMIT);
	mdelay(15);

	mdss_fb_set_bl_brightness(&backlight_led, backlight_led.brightness);

	schedule_delayed_work(&lge_ctrl_pdata->bc_dim_work, BC_DIM_TIME);

	pr_info("VE Mode bl_lvl : %d, BC DIM: 0x%x, BC : 0x%x\n",
			backlight_led.brightness, lge_ctrl_pdata->bc_dim_cmds.cmds[0].payload[1],
			lge_ctrl_pdata->bc_default_cmds.cmds[0].payload[22]);

	return ret;
}
static DEVICE_ATTR(video_enhancement, S_IRUGO | S_IWUSR | S_IWGRP,
					video_enhancement_get, video_enhancement_set);
#endif /* CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT */

static ssize_t dolby_mode_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 0);
}

static ssize_t dolby_mode_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}
static DEVICE_ATTR(dolby_mode, S_IRUGO | S_IWUSR | S_IWGRP,
				dolby_mode_get, dolby_mode_set);

int lge_display_control_create_sysfs(struct class *panel)
{
	int rc = 0;
	static struct device *panel_sysfs_dev = NULL;

	if (!panel) {
		pr_err("Invalid input panel class\n");
		return -EINVAL;
	}

	if (!panel_sysfs_dev) {
		panel_sysfs_dev = device_create(panel, NULL, 0, NULL, "img_tune");
		if (IS_ERR(panel_sysfs_dev)) {
			pr_err("Failed to create dev(panel_sysfs_dev)!");
		} else {
			if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_dgc_status)) < 0)
				pr_err("add dgc_status set node fail!");
			if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_sharpness_status)) < 0)
				pr_err("add sharpness set node fail!");
			if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_boost_status)) < 0)
				pr_err("add boost set node fail!");
			if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_contrast_status)) < 0)
				pr_err("add contrast_status set node fail!");
			if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_hdr_hbm_lut)) < 0)
				pr_err("add hdr_hbm_lut set node fail!");
			if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_hdr_mode)) < 0)
				pr_err("add hdr_mode set node fail!");
			if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_acl_mode)) < 0)
				pr_err("add scl_mode set node fail!");
			if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_white_target)) < 0)
				pr_err("add white_target set node fail!");
			if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_dolby_mode)) < 0)
				pr_err("add dolby mode node fail!");
#if defined(CONFIG_LGE_DISPLAY_COLOR_MANAGER)
			if ((rc = lge_color_manager_create_sysfs(panel_sysfs_dev)) < 0)
				pr_err("failed creating color manager sysfs\n");
#endif /* CONFIG_LGE_DISPLAY_COLOR_MANAGER */
#if defined(CONFIG_LGE_DISPLAY_COMFORT_MODE)
			if ((rc = lge_mdss_comfort_view_create_sysfs(panel_sysfs_dev)) < 0)
				pr_err("failed creating comfort view sysfs\n");
#endif /* CONFIG_LGE_DISPLAY_COMFORT_MODE */
#if defined(CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT)
			if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_video_enhancement)) < 0)
				pr_err("add video enhancement_mode set node fail!");
#endif /* CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT */
		}
	}
	return rc;
}

int lge_display_control_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;
	if (ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;

	/* This function should be defined under each model directory */
	lge_display_control_init_sub(lge_ctrl_pdata);

#if defined(CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT)
	lge_ctrl_pdata->bc_reg_backup_flag = false;
	INIT_DELAYED_WORK(&lge_ctrl_pdata->bc_dim_work, lge_mdss_dsi_bc_dim_work);
#endif /*CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT*/
#if defined(CONFIG_LGE_DISPLAY_COMFORT_MODE)
	rc = lge_mdss_comfort_view_init(ctrl_pdata);
	if (rc) {
		pr_err("[Comfort View] fail to init (rc:%d)\n", rc);
	}
#endif /* CONFIG_LGE_DISPLAY_COMFORT_MODE */
	return rc;
}
