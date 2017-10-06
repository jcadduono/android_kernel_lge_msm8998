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

#ifndef LGE_MDSS_DISPLAY_CONTROL_H
#define LGE_MDSS_DISPLAY_CONTROL_H

#include <linux/of_device.h>
#include <linux/module.h>
#include <soc/qcom/lge/board_lge.h>
#include "../mdss_dsi.h"
#include "../mdss_panel.h"
#include "../mdss_fb.h"
#if defined(CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT)
#include <linux/leds.h>
#endif /* CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT */
#if defined(CONFIG_LGE_DISPLAY_COMFORT_MODE)
#include "lge_comfort_view.h"
#endif /* CONFIG_LGE_DISPLAY_COMFORT_MODE */
#if defined(CONFIG_LGE_DISPLAY_COLOR_MANAGER)
#include "lge_color_manager.h"
#endif /* CONFIG_LGE_DISPLAY_COLOR_MANAGER */

#if defined(CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT)
#define BC_DIM_TIME msecs_to_jiffies(550)
#define BC_REG 0xB8
#define BC_REG_NUM 0x18
#define BC_DIM_FRAMES 0x10
#define BC_DIM_ON 0x10
#define BC_DIM_OFF 0x00
#endif /*CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT*/

int lge_display_control_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
/* This function should be defined under each model directory */
extern int lge_display_control_init_sub(struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata);
void mdss_dsi_parse_display_control_dcs_cmds(struct device_node *np,
				struct mdss_dsi_ctrl_pdata *ctrl_pdata);
void lge_display_control_store(struct mdss_dsi_ctrl_pdata *ctrl, bool cmd_send);
int lge_display_control_create_sysfs(struct class *panel);

#endif
