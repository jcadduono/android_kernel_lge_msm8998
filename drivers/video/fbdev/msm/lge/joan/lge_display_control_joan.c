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

#include "../lge_mdss_dsi.h"
#include "../lge_display_control.h"

enum disp_ctrl_cmds_index {
	DISPLAY_CONTROL_SET1,
	DISPLAY_CONTROL_SET2,
	DISPLAY_CONTROL_INDEX_MAX,
};

static char *disp_ctrl_cmd_names[] = {
	"lge,mdss-dsi-disp-ctrl-command-1",
	"lge,mdss-dsi-disp-ctrl-command-2",
};

extern int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_panel_cmds *pcmds, u32 flags);

static void lge_display_control_store_joan(struct mdss_dsi_ctrl_pdata *ctrl_pdata, bool send_cmd)
{
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;
	struct lge_dsi_cmds_entry *lge_dsi_cmds_list = NULL;

	if (ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;
	lge_dsi_cmds_list = lge_ctrl_pdata->disp_ctrl_cmds_list;

	if (lge_dsi_cmds_list == NULL) {
		pr_err("Cmd list is not initialized.");
		return;
	}

	lge_dsi_cmds_list[0].lge_dsi_cmds.cmds[0].payload[1] = ((lge_ctrl_pdata->dgc_status << 7) & 0x80) | \
			((lge_ctrl_pdata->sharpness_status << 6) & 0x40) | ((lge_ctrl_pdata->boost_status << 5) & 0x20) | \
			((lge_ctrl_pdata->contrast_status << 4) & 0x10) | ((lge_ctrl_pdata->vr_status << 3) & 0x08) | \
			((lge_ctrl_pdata->color_manager_status << 2) & 0x04) | (lge_ctrl_pdata->color_manager_mode & 0x03);

	lge_dsi_cmds_list[1].lge_dsi_cmds.cmds[0].payload[1] |= ((lge_ctrl_pdata->hdr_hbm_lut << 6) & 0xC0) | \
			((lge_ctrl_pdata->hdr_mode << 4) & 0x30) | (lge_ctrl_pdata->acl_mode & 0x03);

	pr_info("%s:0x%02x, %s:0x%02x\n",
			disp_ctrl_cmd_names[0],
			lge_dsi_cmds_list[0].lge_dsi_cmds.cmds[0].payload[1],
			disp_ctrl_cmd_names[1],
			lge_dsi_cmds_list[1].lge_dsi_cmds.cmds[0].payload[1]);

	if (send_cmd)
		lge_mdss_dsi_send_dcs_cmds_list(ctrl_pdata,
					lge_ctrl_pdata->disp_ctrl_cmds_list, DISPLAY_CONTROL_INDEX_MAX);
	return;
}

static void lge_display_control_parse_dt_joan(struct device_node *np, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;

	lge_mdss_dsi_parse_dcs_cmds_by_name_array(np, &lge_ctrl_pdata->disp_ctrl_cmds_list,
						disp_ctrl_cmd_names, DISPLAY_CONTROL_INDEX_MAX);
}

int lge_display_control_init_sub(struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata)
{
	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	lge_ctrl_pdata->display_control_store = lge_display_control_store_joan;
	lge_ctrl_pdata->display_control_parse_dt = lge_display_control_parse_dt_joan;

	return 0;
}
