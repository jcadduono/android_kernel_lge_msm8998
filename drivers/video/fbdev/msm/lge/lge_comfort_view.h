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

#ifndef COMFORT_VIEW_H
#define COMFORT_VIEW_H

#include "../mdss_dsi.h"
#include "lge_mdss_dsi.h"
#include "lge_mdss_dsi_panel.h"
#include "lge_mdss_fb.h"

#define DIGITAL_GAMMA_PRESET_NUMS 11

int lge_mdss_comfort_view_create_sysfs(struct device *panel_sysfs_dev);
void lge_mdss_comfort_view_parse_dt(struct device_node *node,struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_mdss_comfort_view_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_mdss_comfort_view_cmd_send(struct mdss_panel_data *pdata, int step);
#endif
