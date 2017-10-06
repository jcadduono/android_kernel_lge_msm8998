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

#include "lge_mdss_ambient_joan.h"

int lge_mdss_sw43402_validate_and_set_roi(struct ambient_ctrl_info *ainfo,
					struct mdss_panel_info *pinfo)
{
	struct mdss_panel_data *pdata = NULL;
	struct mdss_rect *src = &ainfo->src_roi;
	struct mdss_rect *disp = &ainfo->dst_roi;
	struct mdss_rect full_roi = {0, 0, TARGET_RES_HORIZONTAL_QHD, TARGET_RES_VERTICAL_QHD};

	pdata = container_of(pinfo, struct mdss_panel_data,
			panel_info);
	if (!pdata)
		return -EINVAL;

	if (src->w == 0 || src->h == 0 || disp->w == 0 || disp->h == 0) {
		goto full_roi;
	}

	return 0;
full_roi:
	ainfo->partial_area_horizontal_changed = true;
	ainfo->partial_area_vertical_changed = true;
	memcpy(&ainfo->src_roi, &full_roi, sizeof(struct mdss_rect));
	memcpy(&ainfo->dst_roi, &full_roi, sizeof(struct mdss_rect));
	lge_mdss_sw43402_update_partial_area_cmds(pdata);
	return 0;
}

int lge_mdss_sw43402_set_pixel_shift(struct mdss_panel_info *pinfo)
{
	struct ambient_ctrl_info *ainfo;

	if (pinfo == NULL) {
		pr_err("%s: Invalid input panel data\n", __func__);
		return -EINVAL;
	}
	ainfo = (struct ambient_ctrl_info*)pinfo->ambient_info;

	ainfo->pixel_shift_x_unit = pinfo->ambient_xsft;
	ainfo->pixel_shift_y_unit = pinfo->ambient_ysft;
	ainfo->pixel_shift_interval = pinfo->ambient_sft_interval;

	if (ainfo->pixel_shift_x_unit == 0 ||
		ainfo->pixel_shift_y_unit == 0 ||
		ainfo->pixel_shift_interval == 0) {
		ainfo->pixel_shift_on = false;
	} else {
		ainfo->pixel_shift_on = true;
	}

	return 0;
}

int lge_mdss_sw43402_backup_internal_reg(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret = 0;
	char *data;
	struct dsi_panel_cmds *cmds;

	if (!ctrl) {
		pr_err("%s: Invalid input panel data\n", __func__);
		return -EINVAL;
	}

	cmds = &ctrl->ambient_cmds[AMBIENT_PANEL_CMD_TO_PANEL_ON];

	data = &cmds->cmds[IDX_DUTY_INIT_SET].payload[1];
	lge_mdss_dsi_panel_cmd_read(REG_DUTY_INIT_SET, NUM_DUTY_INIT_SET, data);

	data = &cmds->cmds[IDX_DUTY_BAND1].payload[1];
	lge_mdss_dsi_panel_cmd_read(REG_DUTY_BAND1, NUM_DUTY_BAND1, data);

	data = &cmds->cmds[IDX_DUTY_BAND2].payload[1];
	lge_mdss_dsi_panel_cmd_read(REG_DUTY_BAND2, NUM_DUTY_BAND2, data);

	cmds = &ctrl->ambient_cmds[AMBIENT_PANEL_CMD_TO_DOZE_SUSPEND];

	data = &cmds->cmds[CMDS_DUTY_INIT_SET_E].payload[1];
	lge_mdss_dsi_panel_cmd_read(REG_DUTY_INIT_SET, NUM_DUTY_INIT_SET, data);

	data = &cmds->cmds[CMDS_DUTY_BAND1_E].payload[1];
	lge_mdss_dsi_panel_cmd_read(REG_DUTY_BAND1, NUM_DUTY_BAND1, data);

	data = &cmds->cmds[CMDS_DUTY_BAND2_E].payload[1];
	lge_mdss_dsi_panel_cmd_read(REG_DUTY_BAND2, NUM_DUTY_BAND2, data);


	pr_info("[Ambient] %s: success to store internal register info\n", __func__);

	return ret;
}

int lge_mdss_sw43402_update_partial_area_cmds(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	struct ambient_ctrl_info *ainfo;
	u16 sr, er;

	if (pdata == NULL) {
		pr_err("%s: Invalid input panel data\n", __func__);
		return -EINVAL;
	}
	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	ainfo = (struct ambient_ctrl_info*)pinfo->ambient_info;
	if (ainfo == NULL) {
		pr_err("%s: Invalid input ambient data\n", __func__);
		return -EINVAL;
	}

	if (ainfo->partial_area_vertical_changed) {
		sr = ainfo->dst_roi.y;
		er = sr + ainfo->dst_roi.h - 1;

		if (pinfo->yres == TARGET_RES_VERTICAL_FHD) {
			sr = ((sr >> 2) << 2);
			er = sr + ainfo->dst_roi.h - 1;
		}

		ctrl->ambient_cmds[AMBIENT_PANEL_CMD_TO_DOZE_SUSPEND].cmds[CMDS_PTLAR].payload[1] = DIC_UPPER_BIT(sr);
		ctrl->ambient_cmds[AMBIENT_PANEL_CMD_TO_DOZE_SUSPEND].cmds[CMDS_PTLAR].payload[2] = DIC_LOWER_BIT(sr);
		ctrl->ambient_cmds[AMBIENT_PANEL_CMD_TO_DOZE_SUSPEND].cmds[CMDS_PTLAR].payload[3] = DIC_UPPER_BIT(er);
		ctrl->ambient_cmds[AMBIENT_PANEL_CMD_TO_DOZE_SUSPEND].cmds[CMDS_PTLAR].payload[4] = DIC_LOWER_BIT(er);

#if defined(CONFIG_LGE_DISPLAY_CHANGE_PARTIAL_AREA_IN_KICKOFF)
		ctrl->ambient_cmds[AMBIENT_PANEL_CMD_CHANGE_PARTIAL_AREA].cmds[0].payload[1] = DIC_UPPER_BIT(sr);
		ctrl->ambient_cmds[AMBIENT_PANEL_CMD_CHANGE_PARTIAL_AREA].cmds[0].payload[2] = DIC_LOWER_BIT(sr);
		ctrl->ambient_cmds[AMBIENT_PANEL_CMD_CHANGE_PARTIAL_AREA].cmds[0].payload[3] = DIC_UPPER_BIT(er);
		ctrl->ambient_cmds[AMBIENT_PANEL_CMD_CHANGE_PARTIAL_AREA].cmds[0].payload[4] = DIC_LOWER_BIT(er);
#endif
	}

	/* set PLTAC */
	if (ainfo->partial_area_horizontal_changed) {
		ctrl->ambient_cmds[AMBIENT_PANEL_CMD_TO_DOZE_SUSPEND].cmds[CMDS_PLTAC].payload[1] = ainfo->current_horizontal_area;
#if defined(CONFIG_LGE_DISPLAY_CHANGE_PARTIAL_AREA_IN_KICKOFF)
		ctrl->ambient_cmds[AMBIENT_PANEL_CMD_CHANGE_PARTIAL_AREA].cmds[1].payload[1] = ainfo->current_horizontal_area;
#endif
	}

	return 0;
}

int lge_mdss_sw43402_prepare_cmds(struct mdss_panel_data *pdata, int cmd_index)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	struct ambient_ctrl_info *ainfo;
	struct dsi_panel_cmds *cmds;
	u8 data = 0x00;

	if (pdata == NULL) {
		pr_err("%s: Invalid input panel data\n", __func__);
		return -EINVAL;
	}
	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	ainfo = (struct ambient_ctrl_info*)pinfo->ambient_info;
	if (ainfo == NULL) {
		pr_err("%s: Invalid input ambient data\n", __func__);
		return -EINVAL;
	}

	cmds = &ctrl->ambient_cmds[cmd_index];

	pr_info("%s: current resolution(%d x %d)\n", __func__, pinfo->xres, pinfo->yres);

	switch(cmd_index) {
	case AMBIENT_PANEL_CMD_TO_DOZE_SUSPEND:
		ainfo->pps_skip = false;
		/* set ORBIT */
		if (ainfo->pixel_shift_on) {
			cmds->cmds[CMDS_ORBIT].payload[1] = (ainfo->dst_roi.x)/DIC_H_SHIFT_UNIT;
			cmds->cmds[CMDS_ORBIT].payload[2] = (ainfo->dst_roi.w)/DIC_H_SHIFT_UNIT;
			cmds->cmds[CMDS_ORBIT].payload[3] = DIC_UPPER_BIT(ainfo->dst_roi.h);
			cmds->cmds[CMDS_ORBIT].payload[4] = DIC_LOWER_BIT(ainfo->dst_roi.h);
			cmds->cmds[CMDS_ORBIT].payload[5] = (ainfo->pixel_shift_x_unit)/DIC_H_SHIFT_UNIT;
			cmds->cmds[CMDS_ORBIT].payload[6] = DIC_UPPER_BIT(ainfo->pixel_shift_y_unit);
			cmds->cmds[CMDS_ORBIT].payload[7] = DIC_LOWER_BIT(ainfo->pixel_shift_y_unit);
			cmds->cmds[CMDS_ORBIT].payload[8] = ainfo->pixel_shift_interval;
			cmds->cmds[CMDS_ORBIT].payload[9] = 0x10;
		}

		/* set WRAOD */
		data |= 0x0C; /* default source amp control */

		if (ainfo->pixel_shift_on)
			data |= BIT(7);

		pr_debug("%s: WRAOD : 0x%02x\n", __func__, data);
		cmds->cmds[CMDS_WRAOD].payload[1] = data;

		data = cmds->cmds[CMDS_AODCTL].payload[1];
		if (ainfo->lpmode)
			data |= BIT(1);
		else
			data &= ~BIT(1);

		data &= ~BIT(1); /* skip at this time */

		pr_info("%s: AODCTL : 0x%02x\n", __func__, data);
		cmds->cmds[CMDS_AODCTL].payload[1] = data;

		pr_info("%s : cmd_index = %d, vertical partial area (0x%02x, 0x%02x, 0x%02x, 0x%02x)\n",
							__func__, cmd_index,
							cmds->cmds[CMDS_PTLAR].payload[1],
							cmds->cmds[CMDS_PTLAR].payload[2],
							cmds->cmds[CMDS_PTLAR].payload[3],
							cmds->cmds[CMDS_PTLAR].payload[4]);
		pr_info("%s : cmd_index = %d, horizontal partial area (0x%02x)\n",
				__func__, cmd_index,
				cmds->cmds[CMDS_PTLAR].payload[1]);

		ainfo->partial_area_vertical_changed = false;
		ainfo->partial_area_horizontal_changed = false;
		break;
	case AMBIENT_PANEL_CMD_DOZE_TO_DOZE_SUSPEND:
	case AMBIENT_PANEL_CMD_DOZE_SUSPEND_TO_DOZE:
		ainfo->pps_skip = true; /* to skip sending pps */
		/*
		 * TODO : Ambient display fps is fixed 30 temporary.
		 * After brightness issue in Doze mode is clear, remove this code.
		 */
		cmds->cmds[0].payload[1] = 0x30;
		break;
#if defined(CONFIG_LGE_DISPLAY_CHANGE_PARTIAL_AREA_IN_KICKOFF)
	case AMBIENT_PANEL_CMD_CHANGE_PARTIAL_AREA:
		pr_info("%s : cmd_index = %d, vertical partial area (0x%02x, 0x%02x, 0x%02x, 0x%02x)\n",
							__func__, cmd_index,
							cmds->cmds[0].payload[1],
							cmds->cmds[0].payload[2],
							cmds->cmds[0].payload[3],
							cmds->cmds[0].payload[4]);
		pr_info("%s : cmd_index = %d, horizontal partial area (0x%02x)\n",
							__func__, cmd_index,
							cmds->cmds[1].payload[1]);
		break;
#endif
	default:
		pr_err("%s : Unexpected mode = %d", __func__, cmd_index);
		return -AMBIENT_RETURN_ERROR_NO_SCENARIO;
	}

	return 0;
}

int lge_mdss_ambient_partial_area_set(struct mdss_panel_info *pinfo, int height)
{
	if (pinfo == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo->dsc.partial_height = height;
	pr_debug("[Ambient] partial area %d\n", pinfo->dsc.partial_height);

	return 0;
}

int lge_mdss_ambient_partial_area_unset(struct mdss_panel_info *pinfo)
{
	if (pinfo == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo->dsc.partial_height = 0;

	return 0;
}
