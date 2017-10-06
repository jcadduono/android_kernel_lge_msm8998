/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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
#ifndef LGE_COLOR_MANAGER_H
#define LGE_COLOR_MANAGER_H

#include "../mdss_dsi.h"
#include "lge_mdss_dsi_panel.h"

enum {
	RED      = 0,
	GREEN    = 1,
	BLUE     = 2,
	RGB_ALL  = 3
};

enum {
	PRESET_SETP0_INDEX = 0,
	PRESET_SETP1_INDEX = 6,
	PRESET_SETP2_INDEX = 12,
};

#define IDX_DG_CTRL1 0
#define REG_DG_CTRL1 0xCF
#define NUM_DG_CTRL1 0x09
#define OFFSET_DG_CTRL1 3

#define IDX_DG_CTRL2 1
#define REG_DG_CTRL2 0xD0
#define NUM_DG_CTRL2 0x1B
#define OFFSET_DG_CTRL2 9

#define STEP_DG_PRESET 5
#define NUM_DG_PRESET  24

int lge_color_manager_reg_backup(struct mdss_dsi_ctrl_pdata *ctrl);
void lge_set_custom_rgb(struct mdss_dsi_ctrl_pdata *ctrl, bool send_cmd);
int lge_color_manager_create_sysfs(struct device *panel_sysfs_dev);
void lge_color_manager_parse_dt(struct device_node *np,struct mdss_dsi_ctrl_pdata *ctrl_pdata);

#endif /* LGE_COLOR_MANAGER_H */
