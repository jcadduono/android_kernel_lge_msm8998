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

#include "lge_color_manager.h"
#include <linux/delay.h>

extern struct mdss_panel_data *pdata_base;
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_panel_cmds *pcmds, u32 flags);

char dg_ctrl1_values[NUM_DG_PRESET][OFFSET_DG_CTRL1] = {
	{0x03, 0x05, 0xAF},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},

	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},

	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},

	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},

	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B}
};

char dg_ctrl2_values[NUM_DG_PRESET][OFFSET_DG_CTRL2] = {
	{0x00, 0xFF, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80},
	{0x00, 0xFB, 0x7F, 0xFE, 0x7E, 0xFD, 0x7D, 0xFC, 0x7C},
	{0x00, 0xF7, 0x7E, 0xFD, 0x7C, 0xFB, 0x7A, 0xF9, 0x78},
	{0x00, 0xF3, 0x7E, 0xFC, 0x7B, 0xF9, 0x78, 0xF6, 0x75},
	{0x00, 0xEF, 0x7D, 0xFB, 0x79, 0xF7, 0x75, 0xF3, 0x71},

	{0x00, 0xEB, 0x7D, 0xFA, 0x78, 0xF5, 0x73, 0xF0, 0x6E},
	{0x00, 0xE7, 0x7C, 0xF9, 0x76, 0xF3, 0x70, 0xED, 0x6A},
	{0x00, 0xE3, 0x7C, 0xF8, 0x75, 0xF1, 0x6E, 0xEA, 0x67},
	{0x00, 0xDF, 0x7B, 0xF7, 0x73, 0xEF, 0x6B, 0xE7, 0x63},
	{0x00, 0xDB, 0x7B, 0xF6, 0x72, 0xED, 0x69, 0xE4, 0x60},

	{0x00, 0xD7, 0x7A, 0xF5, 0x70, 0xEB, 0x66, 0xE1, 0x5C},
	{0x00, 0xD3, 0x7A, 0xF4, 0x6F, 0xE9, 0x64, 0xDE, 0x59},
	{0x00, 0xCF, 0x79, 0xF3, 0x6D, 0xE7, 0x61, 0xDB, 0x55},
	{0x00, 0xCB, 0x79, 0xF2, 0x6C, 0xE5, 0x5F, 0xD8, 0x52},
	{0x00, 0xC7, 0x78, 0xF1, 0x6A, 0xE3, 0x5C, 0xD5, 0x4E},

	{0x00, 0xC3, 0x78, 0xF0, 0x69, 0xE1, 0x5A, 0xD2, 0x4B},
	{0x00, 0xBF, 0x77, 0xEF, 0x67, 0xDF, 0x57, 0xCF, 0x47},
	{0x00, 0xBB, 0x77, 0xEE, 0x66, 0xDD, 0x55, 0xCC, 0x44},
	{0x00, 0xB7, 0x76, 0xED, 0x64, 0xDB, 0x52, 0xC9, 0x40},
	{0x00, 0xB3, 0x76, 0xEC, 0x63, 0xD9, 0x50, 0xC6, 0x3D},

	{0x00, 0xAF, 0x75, 0xEB, 0x61, 0xD7, 0x4D, 0xC3, 0x39},
	{0x00, 0xAB, 0x75, 0xEA, 0x60, 0xD5, 0x4B, 0xC0, 0x36},
	{0x00, 0xA7, 0x74, 0xE9, 0x5E, 0xD3, 0x48, 0xBD, 0x32},
	{0x00, 0xA3, 0x74, 0xE8, 0x5D, 0xD1, 0x46, 0xBA, 0x2F}
};

int rgb_preset[STEP_DG_PRESET][RGB_ALL] = {
	{PRESET_SETP2_INDEX, PRESET_SETP0_INDEX, PRESET_SETP2_INDEX},
	{PRESET_SETP1_INDEX, PRESET_SETP0_INDEX, PRESET_SETP1_INDEX},
	{PRESET_SETP0_INDEX, PRESET_SETP0_INDEX, PRESET_SETP0_INDEX},
	{PRESET_SETP0_INDEX, PRESET_SETP1_INDEX, PRESET_SETP0_INDEX},
	{PRESET_SETP0_INDEX, PRESET_SETP2_INDEX, PRESET_SETP0_INDEX}
};

enum color_management_command_set_index {
	CM_DCI_P3,
	CM_SRGB,
	CM_ADOBE,
	CM_NATIVE,
	CM_MAX,
};

static char *color_management_command_set[] = {
	"lge,mdss-dsi-cm-dci-p3-command",
	"lge,mdss-dsi-cm-srgb-command",
	"lge,mdss-dsi-cm-adobe-command",
	"lge,mdss-dsi-cm-native-command"
};

static void lge_color_manager_reg_dump(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int i = 0;
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

	if (lge_ctrl_pdata->d65_default_cmds.cmd_cnt) {
		for (i = 0; i < NUM_DG_CTRL1; i++) {
			pr_debug("Reg:0x%02x [%d:0x%02x]\n",
					REG_DG_CTRL1, i, lge_ctrl_pdata->d65_default_cmds.cmds[IDX_DG_CTRL1].payload[i+1]);
		}

		for (i = 0; i < NUM_DG_CTRL2; i++) {
			pr_debug("Reg:0x%02x [%d:0x%02x]\n",
					REG_DG_CTRL2, i, lge_ctrl_pdata->d65_default_cmds.cmds[IDX_DG_CTRL2].payload[i+1]);
		}
	}
}

int lge_color_manager_reg_backup(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret = 0;
	char *data = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (!ctrl) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	lge_color_manager_reg_dump(ctrl);

	/* Backup Digital Gamma Control #1 */
	if (lge_ctrl_pdata->d65_default_cmds.cmd_cnt)
		data = &lge_ctrl_pdata->d65_default_cmds.cmds[IDX_DG_CTRL1].payload[1];
	if (data != NULL)
		lge_mdss_dsi_panel_cmd_read(REG_DG_CTRL1, NUM_DG_CTRL1, data);

	/* Backup Digital Gamma Control #2 */
	if (lge_ctrl_pdata->d65_default_cmds.cmd_cnt)
		data = &lge_ctrl_pdata->d65_default_cmds.cmds[IDX_DG_CTRL2].payload[1];
	if (data != NULL)
		lge_mdss_dsi_panel_cmd_read(REG_DG_CTRL2, NUM_DG_CTRL2, data);

	lge_color_manager_reg_dump(ctrl);

	return ret;
}

void lge_set_custom_rgb(struct mdss_dsi_ctrl_pdata *ctrl, bool send_cmd)
{
	int i = 0;
	int red_index ,green_index ,blue_index = 0;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (!ctrl) {
		pr_err("Invalid input\n");
		return;
	}

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	red_index   = rgb_preset[lge_ctrl_pdata->cm_preset_step][RED] + lge_ctrl_pdata->cm_red_step;
	green_index = rgb_preset[lge_ctrl_pdata->cm_preset_step][GREEN] + lge_ctrl_pdata->cm_green_step;
	blue_index  = rgb_preset[lge_ctrl_pdata->cm_preset_step][BLUE] + lge_ctrl_pdata->cm_blue_step;

	pr_debug("red_index=(%d) green_index=(%d) blue_index=(%d)\n", red_index, green_index, blue_index);

	for (i = 1; i < OFFSET_DG_CTRL1+1 ; i++) {
		lge_ctrl_pdata->dg_preset_cmds[10].cmds[IDX_DG_CTRL1].payload[i] = dg_ctrl1_values[red_index][i-1];
		lge_ctrl_pdata->dg_preset_cmds[10].cmds[IDX_DG_CTRL1].payload[i+OFFSET_DG_CTRL1] = dg_ctrl1_values[green_index][i-1];
		lge_ctrl_pdata->dg_preset_cmds[10].cmds[IDX_DG_CTRL1].payload[i+OFFSET_DG_CTRL1*2] = dg_ctrl1_values[blue_index][i-1];
	}

	for (i = 1; i < OFFSET_DG_CTRL2+1 ; i++) {
		lge_ctrl_pdata->dg_preset_cmds[10].cmds[IDX_DG_CTRL2].payload[i] = dg_ctrl2_values[red_index][i-1];
		lge_ctrl_pdata->dg_preset_cmds[10].cmds[IDX_DG_CTRL2].payload[i+OFFSET_DG_CTRL2] = dg_ctrl2_values[green_index][i-1];
		lge_ctrl_pdata->dg_preset_cmds[10].cmds[IDX_DG_CTRL2].payload[i+OFFSET_DG_CTRL2*2] = dg_ctrl2_values[blue_index][i-1];
	}

	for (i = 0 ; i < NUM_DG_CTRL1 ; i++) {
		pr_debug("Reg:0x%02x [%d:0x%02x]\n",
				REG_DG_CTRL1, i, lge_ctrl_pdata->dg_preset_cmds[10].cmds[IDX_DG_CTRL1].payload[i+1]);
	}

	for (i = 0; i < NUM_DG_CTRL2; i++) {
		pr_debug("Reg:0x%02x [%d:0x%02x]\n",
				REG_DG_CTRL2, i, lge_ctrl_pdata->dg_preset_cmds[10].cmds[IDX_DG_CTRL2].payload[i+1]);
	}

	if (send_cmd)
		mdss_dsi_panel_cmds_send(ctrl, &lge_ctrl_pdata->dg_preset_cmds[10], CMD_REQ_COMMIT);

	return ;
}

static ssize_t screen_mode_get(struct device *dev,
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

	return sprintf(buf, "%d\n", lge_ctrl_pdata->screen_mode);
}

void lge_set_screen_mode(struct mdss_dsi_ctrl_pdata *ctrl, bool send_cmd)
{
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (!ctrl) {
		pr_err("Invalid input\n");
		return;
	}

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	if (lge_ctrl_pdata->screen_mode == 0) {
		lge_ctrl_pdata->color_manager_mode = lge_ctrl_pdata->color_manager_table[lge_ctrl_pdata->screen_mode].color_manager_mode;
		lge_ctrl_pdata->color_manager_status = lge_ctrl_pdata->color_manager_table[lge_ctrl_pdata->screen_mode].color_manager_status;
		lge_ctrl_pdata->dgc_status = 0x00;
	} else if (lge_ctrl_pdata->screen_mode == 10) {
		pr_info("preset : %d, red = %d, green = %d, blue = %d\n",
				lge_ctrl_pdata->cm_preset_step, lge_ctrl_pdata->cm_red_step,
				lge_ctrl_pdata->cm_green_step, lge_ctrl_pdata->cm_blue_step);
		lge_ctrl_pdata->color_manager_mode = lge_ctrl_pdata->color_manager_table[0].color_manager_mode;
		lge_ctrl_pdata->color_manager_status = lge_ctrl_pdata->color_manager_table[0].color_manager_status;
		if (lge_ctrl_pdata->cm_preset_step == 2 &&
				!(lge_ctrl_pdata->cm_red_step | lge_ctrl_pdata->cm_green_step | lge_ctrl_pdata->cm_blue_step)) {
			lge_ctrl_pdata->dgc_status = 0x00;
		} else {
			lge_set_custom_rgb(ctrl, send_cmd);
			lge_ctrl_pdata->dgc_status = 0x01;
		}
	} else {
		lge_ctrl_pdata->color_manager_mode = lge_ctrl_pdata->color_manager_table[lge_ctrl_pdata->screen_mode].color_manager_mode;
		lge_ctrl_pdata->color_manager_status = lge_ctrl_pdata->color_manager_table[lge_ctrl_pdata->screen_mode].color_manager_status;
		lge_ctrl_pdata->dgc_status = 0x01;
		if (send_cmd && lge_ctrl_pdata->d65_default_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(ctrl, &lge_ctrl_pdata->d65_default_cmds, CMD_REQ_COMMIT);
	}

	lge_display_control_store(ctrl, send_cmd);

	if(send_cmd)
		lge_mdss_dsi_send_dcs_cmds_by_cmd_name(ctrl, lge_ctrl_pdata->color_modes_cmds, CM_MAX,
						color_management_command_set[lge_ctrl_pdata->color_manager_mode]);

	pr_info("color_manager_status %d color_manager_mode %d\n",
			lge_ctrl_pdata->color_manager_status, lge_ctrl_pdata->color_manager_mode);
	return;
}

static ssize_t screen_mode_set(struct device *dev,
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
	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);
	lge_ctrl_pdata->screen_mode = input;


	pr_info("ctrl->screen_mode (%d)\n", lge_ctrl_pdata->screen_mode);

	lge_set_screen_mode(ctrl, true);

	return ret;
}
static DEVICE_ATTR(screen_mode, S_IRUGO | S_IWUSR | S_IWGRP,
					screen_mode_get, screen_mode_set);

static ssize_t rgb_tune_get(struct device *dev,
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

	return sprintf(buf, "%d %d %d %d \n", lge_ctrl_pdata->cm_preset_step,
					lge_ctrl_pdata->cm_red_step,
					lge_ctrl_pdata->cm_green_step,
					lge_ctrl_pdata->cm_blue_step);
}

static ssize_t rgb_tune_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input_param[4];
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
	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);

	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d %d %d %d", &input_param[0], &input_param[1], &input_param[2], &input_param[3]);

	lge_ctrl_pdata->cm_preset_step = input_param[0];
	lge_ctrl_pdata->cm_red_step    = abs(input_param[1]);
	lge_ctrl_pdata->cm_green_step  = abs(input_param[2]);
	lge_ctrl_pdata->cm_blue_step   = abs(input_param[3]);

	pr_info("preset : %d , red = %d , green = %d , blue = %d \n",
			lge_ctrl_pdata->cm_preset_step, lge_ctrl_pdata->cm_red_step,
			lge_ctrl_pdata->cm_green_step, lge_ctrl_pdata->cm_blue_step);

	lge_ctrl_pdata->color_manager_mode = lge_ctrl_pdata->color_manager_table[0].color_manager_mode;
	lge_ctrl_pdata->color_manager_status = lge_ctrl_pdata->color_manager_table[0].color_manager_status;

	if (lge_ctrl_pdata->cm_preset_step == 2 &&
		 !(lge_ctrl_pdata->cm_red_step | lge_ctrl_pdata->cm_green_step | lge_ctrl_pdata->cm_blue_step)) {
		lge_ctrl_pdata->dgc_status = 0x00;
	} else {
		lge_set_custom_rgb(ctrl, true);
		lge_ctrl_pdata->dgc_status = 0x01;
	}

	lge_display_control_store(ctrl, true);

	return ret;
}
static DEVICE_ATTR(rgb_tune, S_IRUGO | S_IWUSR | S_IWGRP,
					rgb_tune_get, rgb_tune_set);

static ssize_t color_manager_status_get(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata, panel_data);
	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", lge_ctrl_pdata->color_manager_status);
}

static ssize_t color_manager_status_set(struct device *dev,
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
		pr_err("Panel off state. Ignore color_manager_status set cmd\n");
		return -EINVAL;
	}

	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata, panel_data);
	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	if (!lge_ctrl_pdata->color_manager_default_status) {
		pr_info("Color manager is disabled as default. Ignore color manager status control.\n");
		return ret;
	}

	sscanf(buf, "%d", &input);

	lge_ctrl_pdata->color_manager_status = input & 0x01;
	lge_display_control_store(ctrl, true);

	if (lge_ctrl_pdata->color_manager_status)
		lge_mdss_dsi_send_dcs_cmds_by_cmd_name(ctrl, lge_ctrl_pdata->color_modes_cmds, CM_MAX,
						color_management_command_set[lge_ctrl_pdata->color_manager_mode]);

	pr_info("color_manager_status %d \n", lge_ctrl_pdata->color_manager_status);
	return ret;
}
static DEVICE_ATTR(color_manager_status, S_IWUSR|S_IRUGO,
					color_manager_status_get, color_manager_status_set);

static ssize_t color_manager_mode_get(struct device *dev,
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

	return sprintf(buf, "%d\n", lge_ctrl_pdata->color_manager_mode);
}

static ssize_t color_manager_mode_set(struct device *dev,
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
		pr_err("Panel off state. Ignore color_manager_mode set cmd\n");
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

	lge_ctrl_pdata->color_manager_mode = input & 0x03;
	lge_ctrl_pdata->color_manager_status = 0x01;
	lge_display_control_store(ctrl, true);

	pr_info("color_manager_mode %d \n",	lge_ctrl_pdata->color_manager_mode);
	return ret;
}
static DEVICE_ATTR(color_manager_mode, S_IWUSR|S_IRUGO,
					color_manager_mode_get, color_manager_mode_set);

int lge_color_manager_create_sysfs(struct device *panel_sysfs_dev)
{
	int rc = 0;
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_screen_mode)) < 0)
		pr_err("add screen_mode set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_rgb_tune)) < 0)
		pr_err("add rgb_tune set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_color_manager_status)) < 0)
		pr_err("add color_manager_status set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_color_manager_mode)) < 0)
		pr_err("add color_manager_mode set node fail!");
	return rc;
}

static int mdss_dsi_parse_color_manager_modes(struct device_node *np,
				struct lge_dsi_color_manager_mode_entry color_manager_table[NUM_COLOR_MODES],
				u32 *color_manager_modes_num,
				const char *name)
{
	int num = 0;
	int i, j;
	int rc;
	struct property *data;
	u32 tmp[NUM_COLOR_MODES];
	*color_manager_modes_num = 0;
	data = of_find_property(np, name, &num);
	num /= sizeof(u32);
	if (!data || !num || num > NUM_COLOR_MODES || num % 2) {
		pr_debug("error reading %s, length found = %d\n", name, num);
	} else {
		rc = of_property_read_u32_array(np, name, tmp, num);
		if (rc)
			pr_debug("error reading %s, rc = %d\n", name, rc);
		else {
			for (i = 0, j = 0; i < num/2; i++, j++) {
				color_manager_table[i].color_manager_mode = tmp[j];
				color_manager_table[i].color_manager_status = tmp[++j];
				pr_info("index = %d, color_manager_mode = %d, color_manager_status = %d\n", i, color_manager_table[i].color_manager_mode, color_manager_table[i].color_manager_status);
			}
			*color_manager_modes_num = num/2;
		}
	}
	return 0;
}

void lge_color_manager_parse_dt(struct device_node *np, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;
	u32 tmp = 0;
	int rc = 0;

	if (ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	mdss_dsi_parse_dcs_cmds(np, &lge_ctrl_pdata->d65_default_cmds,
		"lge,mdss-dsi-wb-default-command", "lge,mdss-dsi-hs-command-state");
	mdss_dsi_parse_color_manager_modes(np, lge_ctrl_pdata->color_manager_table,
					&(lge_ctrl_pdata->color_manager_table_len), "lge,mdss-dsi-color-manager-mode-table");

	lge_mdss_dsi_parse_dcs_cmds_by_name_array(np, &lge_ctrl_pdata->color_modes_cmds,
					color_management_command_set, CM_MAX);

	rc = of_property_read_u32(np, "lge,color-manager-default-status", &tmp);
	if (rc) {
		pr_debug("fail to parse lge,color-manager-default-status\n");
		lge_ctrl_pdata->color_manager_default_status = false;
	} else {
		lge_ctrl_pdata->color_manager_default_status = (tmp > 0) ? true : false;
	}
}
