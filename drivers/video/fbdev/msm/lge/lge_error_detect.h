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
#ifndef LGE_MDSS_ERR_DETECT_H
#define LGE_MDSS_ERR_DETECT_H

#include "../mdss_dsi.h"
#include "lge_mdss_dsi.h"
#include "lge_mdss_dsi_panel.h"
#include "lge_mdss_fb.h"

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include "../../../../../kernel/irq/internals.h"

void lge_dsi_err_detect_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
/* This function should be defined under each model directory */
extern void lge_dsi_err_detect_init_sub(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
void lge_dsi_err_detect_parse_dt(struct device_node *np, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_dsi_err_detect_create_sysfs(struct class *panel);
int lge_get_extra_gpio(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	const char* name);
void lge_dsi_err_detect_remove(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
void lge_dsi_irq_control(struct mdss_dsi_ctrl_pdata *ctrl_pdata, bool enable);
void lge_mdss_dsi_panel_err_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, int link_state);
#endif
