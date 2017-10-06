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

#ifndef LGE_MDSS_AMBIENT_H
#define LGE_MDSS_AMBIENT_H

#include <linux/of_device.h>
#include <linux/module.h>
#include <soc/qcom/lge/board_lge.h>
#include "../mdss_dsi.h"
#include "../mdss_panel.h"
#include "../mdss_fb.h"

/* Enum for parse aod command set */
enum ambient_cmd_type {
	AMBIENT_PANEL_NOT_CHNAGE = -1,
	AMBIENT_PANEL_CMD_OFF = 0,
	AMBIENT_PANEL_CMD_TO_PANEL_ON,
	AMBIENT_PANEL_CMD_DOZE_SUSPEND_TO_DOZE,
	AMBIENT_PANEL_CMD_TO_DOZE_SUSPEND,
	AMBIENT_PANEL_CMD_DOZE_TO_DOZE_SUSPEND,
#if defined(CONFIG_LGE_DISPLAY_CHANGE_PARTIAL_AREA_IN_KICKOFF)
	AMBIENT_PANEL_CMD_CHANGE_PARTIAL_AREA,
#endif
	AMBIENT_PANEL_CMD_NUM,
	AMBIENT_PANEL_CMD_NONE
};

enum ambient_return_type {
	AMBIENT_RETURN_SUCCESS = 0,
	AMBIENT_RETURN_ERROR_NOT_INIT,
	AMBIENT_RETURN_ERROR_NO_SCENARIO,
	AMBIENT_RETURN_ERROR_NOT_NORMAL_BOOT,
	AMBIENT_RETURN_ERROR_UNKNOWN,
	AMBIENT_RETURN_ERROR_SEND_CMD,
	AMBIENT_RETURN_ERROR_REGISTER_BACKUP,
	AMBIENT_RETURN_ERROR_MEMORY = 12,
};

struct ambient_ctrl_info {
	int (*prepare_cmds)(struct mdss_panel_data *pdata, int cmd_index);
	int (*validate_and_set_roi)(struct ambient_ctrl_info *ainfo,
					struct mdss_panel_info *pinfo);
	int (*control_gram_area)(struct mdss_panel_info *pinfo);
	int (*set_pixel_shift)(struct mdss_panel_info *pinfo);
	struct mutex ambient_lock;
	bool is_first_doze_access;
	bool partial_area_vertical_changed;
	bool partial_area_horizontal_changed;
	u8 current_horizontal_area;
	bool lpmode;
	bool pixel_shift_on;
	u16 pixel_shift_x_unit;
	u16 pixel_shift_y_unit;
	u8 pixel_shift_interval;
	struct mdss_rect src_roi;
	struct mdss_rect dst_roi;
	bool pps_skip;
};

int lge_mdss_ambient_create_sysfs(struct class *panel);
int lge_mdss_ambient_init(struct device_node *node,
				struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_mdss_ambient_config_reset(struct mdss_panel_data *pdata);
int lge_mdss_ambient_cmd_send(struct mdss_panel_data *pdata, int enable);
int lge_mdss_ambient_change_lp_mode(struct mdss_panel_data *pdata, int req_power_state);
#if defined(CONFIG_LGE_DISPLAY_CHANGE_PARTIAL_AREA_IN_KICKOFF)
int lge_mdss_ambient_change_partial_area(struct mdss_panel_data *pdata, int cur_power_state);
int lge_mdss_ambient_post_change_partial_area(struct mdss_panel_data *pdata, int cur_power_state);
#endif
#if defined(CONFIG_LGE_DISPLAY_DYNAMIC_RESOLUTION_SWITCH)
int lge_mdss_ambient_bist_verify(struct mdss_panel_data *pdata, int cur_power_state);
#endif
int lge_mdss_ambient_backup_internal_reg(struct mdss_dsi_ctrl_pdata *ctrl_pdata);

#endif /* LGE_MDSS_AMBIENT_H */
