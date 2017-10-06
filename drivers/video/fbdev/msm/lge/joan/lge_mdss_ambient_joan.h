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

#ifndef LGE_MDSS_AMBIENT_JOAN_H
#define LGE_MDSS_AMBIENT_JOAN_H

#include <linux/of_device.h>
#include <linux/module.h>
#include <soc/qcom/lge/board_lge.h>
#include "../../mdss_dsi.h"
#include "../../mdss_panel.h"
#include "../../mdss_fb.h"
#include "../lge_mdss_ambient.h"
#include "../lge_mdss_dsi_panel.h"

#define DIC_UPPER_BIT(data)		(data >> 8)
#define DIC_LOWER_BIT(data)		(data & 0xFF)
#define DIC_H_SHIFT_UNIT		6
#define DIC_V_SHIFT_UNIT		1
#define MAX_GRAM_PARTITION		3
#define DEFAULT_GRAM_PARTITION		3
#define DEFAULT_FPS			30
#define TARGET_FPS			60
#define TARGET_RES_VERTICAL_QHD		2880
#define TARGET_RES_VERTICAL_FHD		2160
#define TARGET_RES_VERTICAL_HD		1440
#define TARGET_RES_HORIZONTAL_QHD	1440
#define TARGET_RES_HORIZONTAL_FHD	1080
#define TARGET_RES_HORIZONTAL_HD	720

enum {
	DIC_H_PARTIAL_AREA_STEP1 = 156,
	DIC_H_PARTIAL_AREA_STEP2 = 318,
	DIC_H_PARTIAL_AREA_STEP3 = 480,
};

enum {
	CMDS_MANUFACTURE = 0,
	CMDS_ORBIT,
	CMDS_DUTY_INIT_SET_S,
	CMDS_DUTY_BAND1_S,
	CMDS_DUTY_BAND2_S,
	CMDS_SRCCTL,
	CMDS_GIPCTL2,
	CMDS_STEP_UP_CTRL,
	CMDS_TE_OFF,
	CMDS_PTLAR,
	CMDS_PLTAC,
	CMDS_WRAOD,
	CMDS_PLTON,
	CMDS_AODCTL,
	CMDS_AMBIENT_IN,
	CMDS_TE_ON,
	CMDS_DUTY_INIT_SET_E,
	CMDS_DUTY_BAND1_E,
	CMDS_DUTY_BAND2_E,
	CMDS_MAX,
};

#define IDX_DUTY_INIT_SET 11
#define REG_DUTY_INIT_SET 0xCA
#define NUM_DUTY_INIT_SET 0x07

#define IDX_DUTY_BAND1 12
#define REG_DUTY_BAND1 0xCB
#define NUM_DUTY_BAND1 0x1E

#define IDX_DUTY_BAND2 13
#define REG_DUTY_BAND2 0xCC
#define NUM_DUTY_BAND2 0x12

int lge_mdss_sw43402_set_pixel_shift(struct mdss_panel_info *pinfo);
int lge_mdss_sw43402_validate_and_set_roi(struct ambient_ctrl_info *ainfo,
					struct mdss_panel_info *pinfo);
int lge_mdss_sw43402_prepare_cmds(struct mdss_panel_data *pdata, int cmd_index);
int lge_mdss_sw43402_update_fps(struct mdss_panel_data *pdata, int fps);
int lge_mdss_sw43402_update_partial_area_cmds(struct mdss_panel_data *pdata);
int lge_mdss_sw43402_backup_internal_reg(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_mdss_ambient_partial_area_set(struct mdss_panel_info *pinfo, int height);
int lge_mdss_ambient_partial_area_unset(struct mdss_panel_info *pinfo);
#endif /* LGE_MDSS_AMBIENT_JOAN_H */
