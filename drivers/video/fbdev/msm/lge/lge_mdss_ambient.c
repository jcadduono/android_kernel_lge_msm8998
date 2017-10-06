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
#include "lge_mdss_ambient.h"
#include "lge_mdss_fb.h"
#include <linux/delay.h>
#if defined(CONFIG_LGE_DISPLAY_JOAN_COMMON)
#include "joan/lge_mdss_ambient_joan.h"
#endif

extern struct mdss_panel_data *pdata_base;
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);

extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds, u32 flags);

static char *ambient_mode_cmd_dt[] = {
	"lge,mode-change-cmds-panel-off",
	"lge,mode-change-cmds-doze-suspend-to-panel-on",
	"lge,mode-change-cmds-doze-suspend-to-doze",
	"lge,mode-change-cmds-panel-on-to-doze-suspend",
	"lge,mode-change-cmds-doze-to-doze-suspend",
#if defined(CONFIG_LGE_DISPLAY_CHANGE_PARTIAL_AREA_IN_KICKOFF)
	"lge,change-partial-area-cmds",
#endif
};

static int lge_mdss_ambient_feature_init(struct mdss_panel_info *pinfo)
{
	struct ambient_ctrl_info *ainfo;

	if (pinfo == NULL) {
		pr_err("%s: Invalid input panel data\n", __func__);
		return -EINVAL;
	}
	ainfo = pinfo->ambient_info;

	mutex_init(&ainfo->ambient_lock);
#if defined(CONFIG_LGE_DISPLAY_JOAN_COMMON)
	ainfo->prepare_cmds = lge_mdss_sw43402_prepare_cmds;
	ainfo->validate_and_set_roi = lge_mdss_sw43402_validate_and_set_roi;
	ainfo->control_gram_area = NULL;
	ainfo->set_pixel_shift = lge_mdss_sw43402_set_pixel_shift;
#else
	ainfo->prepare_cmds = NULL;
	ainfo->validate_and_set_roi = NULL;
	ainfo->control_gram_area = NULL;
	ainfo->set_pixel_shift = NULL;
	ainfo->update_fps = NULL;
#endif

	return AMBIENT_RETURN_SUCCESS;
};

int lge_mdss_ambient_backup_internal_reg(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int ret = 0;
#if defined(CONFIG_LGE_DISPLAY_JOAN_COMMON)
	if (lge_mdss_sw43402_backup_internal_reg(ctrl_pdata) < 0) {
		pr_err("[Ambient] %s: fail to backup internal register\n", __func__);
		return -AMBIENT_RETURN_ERROR_REGISTER_BACKUP;
	}
#endif
	return ret;
}

static int lge_mdss_ambient_init_control_data(struct mdss_panel_info *pinfo)
{
	if (pinfo == NULL) {
		pr_err("%s: Invalid input panel data\n", __func__);
		return -EINVAL;
	}
	pinfo->ambient_info = kzalloc(sizeof(struct ambient_ctrl_info), GFP_KERNEL);
	if (!pinfo->ambient_info) {
		pr_err("%s: failed to get memory\n", __func__);
		return -ENOMEM;
	};

	return AMBIENT_RETURN_SUCCESS;
};

int lge_mdss_ambient_init(struct device_node *node,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int i;
	struct mdss_panel_info *panel_info;

	/* Only parse ambient display command in DSI0 ctrl */
	if (ctrl_pdata->panel_data.panel_info.pdest != DISPLAY_1)
		return AMBIENT_RETURN_SUCCESS;

	pr_info("[Ambient] init start\n");
	panel_info = &(ctrl_pdata->panel_data.panel_info);
	panel_info->ambient_init_done = false;
	ctrl_pdata->ambient_cmds = kzalloc(sizeof(struct dsi_panel_cmds) *
					AMBIENT_PANEL_CMD_NUM, GFP_KERNEL);
	if (!ctrl_pdata->ambient_cmds) {
		pr_err("[Ambient] failed to get memory\n");
		return -AMBIENT_RETURN_ERROR_MEMORY;
	}

	for (i = 0 ; i < AMBIENT_PANEL_CMD_NUM ; i++) {
		mdss_dsi_parse_dcs_cmds(node, &ctrl_pdata->ambient_cmds[i],
			ambient_mode_cmd_dt[i], "qcom,mode-control-dsi-state");
	}

	if (lge_mdss_ambient_init_control_data(panel_info) < 0) {
		pr_err("[Ambient] failed to get memory(sw43402)\n");
		return -AMBIENT_RETURN_ERROR_MEMORY;
	}

	if (lge_mdss_ambient_feature_init(panel_info) < 0) {
		pr_err("[Ambient] failed to init DIC feature\n");
		goto error;
	}
	panel_info->ambient_init_done = true;

	return AMBIENT_RETURN_SUCCESS;
error:
	kfree(panel_info->ambient_info);
	panel_info->ambient_info = NULL;

	return -AMBIENT_RETURN_ERROR_NO_SCENARIO;
}

static bool lge_mdss_ambient_partial_area_chenged(struct ambient_ctrl_info *ainfo)
{
	return (ainfo->partial_area_vertical_changed ||
			ainfo->partial_area_horizontal_changed);
}

int lge_mdss_ambient_config_reset(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	struct ambient_ctrl_info *ainfo;

	pr_info("%s: start\n", __func__);
	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!pinfo->ambient_init_done) {
		pr_err("%s: ainfo is not initialized\n", __func__);
		return -AMBIENT_RETURN_ERROR_NOT_INIT;
	}
	ainfo = (struct ambient_ctrl_info*)pinfo->ambient_info;

	if (ainfo == NULL) {
		pr_err("%s: ainfo is null\n", __func__);
		return -AMBIENT_RETURN_ERROR_NOT_INIT;
	}

#if defined(CONFIG_LGE_DISPLAY_BIST_MODE)
	mutex_lock(&ctrl->bist_lock);
	lge_mdss_dsi_bist_release(ctrl);
	mutex_unlock(&ctrl->bist_lock);
	ctrl->keep_bist_on = false;
#endif
#if defined(CONFIG_LGE_DISPLAY_DYNAMIC_RESOLUTION_SWITCH)
	ctrl->requested_resolution_switch = false;
#endif
	ainfo->pixel_shift_on = false;
	ainfo->pixel_shift_x_unit = 0;
	ainfo->pixel_shift_y_unit = 0;
	ainfo->pixel_shift_interval = 0;
	ainfo->pps_skip = false;

	pr_info("%s: end\n", __func__);
	return 0;
}

static int lge_mdss_ambient_prepare_cmds(struct mdss_panel_data *pdata, int cmd_index)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	struct ambient_ctrl_info *ainfo;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: start\n", __func__);
	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!pinfo->ambient_init_done)
		return -AMBIENT_RETURN_ERROR_NOT_INIT;
	ainfo = (struct ambient_ctrl_info*)pinfo->ambient_info;

	if (ainfo->validate_and_set_roi) {
		ainfo->validate_and_set_roi(ainfo, pinfo);
	}

	mb(); /* ensure display areas are in proper state */

	if (ainfo->control_gram_area) {
		ainfo->control_gram_area(pinfo);
	}

	if (ainfo->set_pixel_shift) {
		ainfo->set_pixel_shift(pinfo);
	}

	if (ainfo->prepare_cmds)
		ainfo->prepare_cmds(pdata, cmd_index);

	return AMBIENT_RETURN_SUCCESS;
}

int lge_mdss_ambient_cmd_send(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	struct ambient_ctrl_info *ainfo;
	int cmd_index;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!pinfo->ambient_init_done)
		return -AMBIENT_RETURN_ERROR_NOT_INIT;

	ainfo = (struct ambient_ctrl_info*)pinfo->ambient_info;

	if (ainfo == NULL)
		return -AMBIENT_RETURN_ERROR_NOT_INIT;

	pr_info("[Ambient] request %s doze-suspend\n",
				(enable == true) ? "enter" : "exit");
	if (enable) {
		cmd_index = AMBIENT_PANEL_CMD_TO_DOZE_SUSPEND;
	} else {
		cmd_index = AMBIENT_PANEL_CMD_TO_PANEL_ON;
		pinfo->panel_power_state = MDSS_PANEL_POWER_ON;
		mdss_dsi_panel_dsc_pps_send(ctrl, pinfo);
		goto send_cmds;
	}

	mutex_lock(&ainfo->ambient_lock);
	lge_mdss_ambient_prepare_cmds(pdata, cmd_index);

	if (!ainfo->pps_skip && ainfo->dst_roi.h > 0) {
		lge_mdss_ambient_partial_area_set(pinfo, ainfo->dst_roi.h);
		mdss_dsi_panel_dsc_pps_send(ctrl, pinfo);
		lge_mdss_ambient_partial_area_unset(pinfo);
	}

send_cmds:
#if defined(CONFIG_LGE_DISPLAY_DYNAMIC_RESOLUTION_SWITCH)
#if defined(CONFIG_LGE_DISPLAY_BIST_MODE)
	if (enable && (ctrl->requested_resolution_switch && !ctrl->keep_bist_on)) {
		pr_info("%s: area chagned (hor:%d) (ver:%d)\n", __func__,
					ainfo->partial_area_vertical_changed,
					ainfo->partial_area_horizontal_changed);
		mutex_lock(&ctrl->bist_lock);
		if (lge_mdss_dsi_bist_ctrl(ctrl, true) < 0) {
			pr_warn("%s: fail to bist control\n", __func__);
		}
		mutex_unlock(&ctrl->bist_lock);
	}
#endif
#endif
	mdss_dsi_panel_cmds_send(ctrl, &ctrl->ambient_cmds[cmd_index], CMD_REQ_COMMIT);
	if (enable) {
		mutex_unlock(&ainfo->ambient_lock);
	}

	return AMBIENT_RETURN_SUCCESS;
}

int lge_mdss_ambient_change_lp_mode(struct mdss_panel_data *pdata, int req_power_state)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	struct ambient_ctrl_info *ainfo;
	int cmd_index = AMBIENT_PANEL_CMD_DOZE_SUSPEND_TO_DOZE;
	bool is_ulp = ((req_power_state % 2) ? true : false);

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!pinfo->ambient_init_done)
		return -AMBIENT_RETURN_ERROR_NOT_INIT;

	ainfo = (struct ambient_ctrl_info*)pinfo->ambient_info;
	if (ainfo == NULL)
		return -AMBIENT_RETURN_ERROR_NOT_INIT;

	if(is_ulp)
		cmd_index = AMBIENT_PANEL_CMD_DOZE_TO_DOZE_SUSPEND;
	else
		cmd_index = AMBIENT_PANEL_CMD_DOZE_SUSPEND_TO_DOZE;

	/* DO NOTHING in JOAN scenario */
	/* mdss_dsi_panel_cmds_send(ctrl, &ctrl->ambient_cmds[cmd_index], CMD_REQ_COMMIT); */

	return AMBIENT_RETURN_SUCCESS;
}

#if defined(CONFIG_LGE_DISPLAY_CHANGE_PARTIAL_AREA_IN_KICKOFF)
int lge_mdss_ambient_change_partial_area(struct mdss_panel_data *pdata, int cur_power_state)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	struct ambient_ctrl_info *ainfo;
	int cmd_index = AMBIENT_PANEL_CMD_CHANGE_PARTIAL_AREA;

	if (mdss_panel_is_power_on_lp(cur_power_state)) {
		if (pdata == NULL) {
			pr_err("%s: Invalid input data\n", __func__);
			return -EINVAL;
		}
		pinfo = &pdata->panel_info;
		ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

		if (!pinfo->ambient_init_done)
			return -AMBIENT_RETURN_ERROR_NOT_INIT;
		ainfo = (struct ambient_ctrl_info*)pinfo->ambient_info;

		if (ainfo == NULL)
			return -AMBIENT_RETURN_ERROR_NOT_INIT;

		mutex_lock(&ainfo->ambient_lock);
		if (lge_mdss_ambient_partial_area_chenged(ainfo)) {
#if defined(CONFIG_LGE_DISPLAY_BIST_MODE)
			lge_mdss_ambient_prepare_cmds(pdata, cmd_index);
			mutex_lock(&ctrl->bist_lock);
			if (lge_mdss_dsi_bist_ctrl(ctrl, true) < 0) {
				pr_warn("%s: fail to bist control\n", __func__);
			}
			mdelay(34);
			mutex_unlock(&ctrl->bist_lock);
#endif
			if (!ainfo->pps_skip && ainfo->dst_roi.h > 0) {
				lge_mdss_ambient_partial_area_set(pinfo, ainfo->dst_roi.h);
				mdss_dsi_panel_dsc_pps_send(ctrl, pinfo);
				lge_mdss_ambient_partial_area_unset(pinfo);
			}
			mdss_dsi_panel_cmds_send(ctrl,
					&ctrl->ambient_cmds[cmd_index],
					CMD_REQ_COMMIT);

			ainfo->partial_area_vertical_changed = false;
			ainfo->partial_area_horizontal_changed = false;
		}
		mutex_unlock(&ainfo->ambient_lock);
	}

	return AMBIENT_RETURN_SUCCESS;
}
#endif /* CONFIG_LGE_DISPLAY_CHANGE_PARTIAL_AREA_IN_KICKOFF */

#if defined(CONFIG_LGE_DISPLAY_BIST_MODE)
#if defined(CONFIG_LGE_DISPLAY_DYNAMIC_RESOLUTION_SWITCH)
int lge_mdss_ambient_bist_verify(struct mdss_panel_data *pdata, int cur_power_state)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo = NULL;
	struct ambient_ctrl_info *ainfo = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid panel data\n", __func__);
		return -EINVAL;
	}
	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);
	if (ctrl == NULL || pinfo == NULL) {
		pr_err("%s: Invalid ctrl data\n", __func__);
		return -EINVAL;
	}

	if (!pinfo->ambient_init_done) {
		pr_err("%s: ainfo is not initialized\n", __func__);
		return -EINVAL;
	}

	ainfo = (struct ambient_ctrl_info*)pinfo->ambient_info;
	if (ainfo == NULL) {
		pr_err("%s: Invalid input data : ainfo\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&ctrl->bist_lock);
	if (ctrl->bist_on == 0) {
		mutex_unlock(&ctrl->bist_lock);
		return 0;
	} else {
		if ((ctrl->requested_resolution_switch && ctrl->keep_bist_on) &&
				lge_mdss_ambient_partial_area_chenged(ainfo) &&
				(ctrl->drs_state != DYNAMIC_RESOLUTION_SWITCH_RUNNING)) {
			lge_mdss_dsi_bist_release(ctrl);
			ctrl->requested_resolution_switch = false;
			ctrl->keep_bist_on = false;
		}
	}
	mutex_unlock(&ctrl->bist_lock);

	return 0;
}
#endif /* CONFIG_LGE_DISPLAY_DYNAMIC_RESOLUTION_SWITCH */
#endif /* CONFIG_LGE_DISPLAY_BIST_MODE */

#if defined(CONFIG_LGE_DISPLAY_BIST_MODE)
int lge_mdss_ambient_post_change_partial_area(struct mdss_panel_data *pdata, int cur_power_state)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	struct ambient_ctrl_info *ainfo;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);

	if (!pinfo->ambient_init_done)
		return -AMBIENT_RETURN_ERROR_NOT_INIT;
	ainfo = (struct ambient_ctrl_info*)pinfo->ambient_info;

	mutex_lock(&ctrl->bist_lock);
#if defined(CONFIG_LGE_DISPLAY_DYNAMIC_RESOLUTION_SWITCH)
	pr_info("%s: (freeze:%d) (store:%d) (hoz:%d) (ver:%d)\n", __func__,
				ctrl->requested_resolution_switch,
				ctrl->keep_bist_on,
				ainfo->partial_area_horizontal_changed,
				ainfo->partial_area_vertical_changed);
	if (ctrl->requested_resolution_switch) {
		if ((ctrl->keep_bist_on) &&
			(!lge_mdss_ambient_partial_area_chenged(ainfo))) {
			lge_mdss_dsi_bist_release(ctrl);
			ctrl->requested_resolution_switch = false;
			ctrl->keep_bist_on = false;
		}
	} else
#endif /* CONFIG_LGE_DISPLAY_DYNAMIC_RESOLUTION_SWITCH */
	if (ctrl->bist_on > 0) {
		if (lge_mdss_dsi_bist_ctrl(ctrl, false) < 0) {
			pr_warn("%s: fail to bist control\n", __func__);
		}
	}
	mutex_unlock(&ctrl->bist_lock);

	return AMBIENT_RETURN_SUCCESS;
}
#endif /* CONFIG_LGE_DISPLAY_BIST_MODE */

static int store_aod_area(struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_rect req)
{
	int ret = 0;
	struct mdss_panel_info *pinfo = NULL;
	struct ambient_ctrl_info *ainfo = NULL;
	u8 horizontal_area = 0;
	u16 sc, ec;

	if (pdata_base == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);
	pinfo = &pdata_base->panel_info;
	if (!pinfo)
		return -EINVAL;

	ainfo = (struct ambient_ctrl_info*)pinfo->ambient_info;
	if (!ainfo)
		return -AMBIENT_RETURN_ERROR_NOT_INIT;

	mutex_lock(&ainfo->ambient_lock);

	sc = req.x;
	ec = sc + req.w;

	if (sc > DIC_H_PARTIAL_AREA_STEP1 && sc <= DIC_H_PARTIAL_AREA_STEP2) {
		horizontal_area |= BIT(4);
	} else if (sc > DIC_H_PARTIAL_AREA_STEP2 && sc <= DIC_H_PARTIAL_AREA_STEP3) {
		horizontal_area |= BIT(5);
	} else if (sc > DIC_H_PARTIAL_AREA_STEP3) {
		horizontal_area |= BIT(4) | BIT(5);
	}

	if (ec < TARGET_RES_HORIZONTAL_QHD - DIC_H_PARTIAL_AREA_STEP1 &&
			ec >= TARGET_RES_HORIZONTAL_QHD - DIC_H_PARTIAL_AREA_STEP2) {
		horizontal_area |= BIT(0);
	} else if (ec < TARGET_RES_HORIZONTAL_QHD - DIC_H_PARTIAL_AREA_STEP2 &&
			ec >= TARGET_RES_HORIZONTAL_QHD - DIC_H_PARTIAL_AREA_STEP3) {
		horizontal_area |= BIT(1);
	} else if (ec < TARGET_RES_HORIZONTAL_QHD - DIC_H_PARTIAL_AREA_STEP3) {
		horizontal_area |= BIT(0) | BIT(1);
	}

	if (horizontal_area != ainfo->current_horizontal_area) {
		ainfo->current_horizontal_area = horizontal_area;
		ainfo->partial_area_horizontal_changed = true;
	}

	if (ainfo->dst_roi.y != req.y ||
		ainfo->dst_roi.h != req.h) {
		ainfo->partial_area_vertical_changed = true;
	}

	pr_info("%s: horizontal_changed[%d], vertical_changed[%d]\n", __func__,
			ainfo->partial_area_horizontal_changed,
			ainfo->partial_area_vertical_changed);

	if (lge_mdss_ambient_partial_area_chenged(ainfo)) {
		memcpy(&ainfo->dst_roi, &req, sizeof(struct mdss_rect));
		if (req.y > 0) req.y = 0;
		memcpy(&ainfo->src_roi, &req, sizeof(struct mdss_rect));
		lge_mdss_sw43402_update_partial_area_cmds(pdata_base);
		pr_info("%s: source pos - x[%d] y[%d] w[%d] h[%d]\n", __func__,
				ainfo->src_roi.x, ainfo->src_roi.y,
				ainfo->src_roi.w, ainfo->src_roi.h);
		pr_info("%s: display pos - x[%d] y[%d] w[%d] h[%d]\n", __func__,
				ainfo->dst_roi.x, ainfo->dst_roi.y,
				ainfo->dst_roi.w, ainfo->dst_roi.h);
	}
#if defined(CONFIG_LGE_DISPLAY_DYNAMIC_RESOLUTION_SWITCH)
	if (ctrl->requested_resolution_switch) {
#if defined(CONFIG_LGE_DISPLAY_BIST_MODE)
		ctrl->keep_bist_on = true;
#endif
		if (!ainfo->partial_area_horizontal_changed &&
				!ainfo->partial_area_vertical_changed) {
			pr_info("%s: request refresh\n", __func__);
		}
	}
#endif
	mutex_unlock(&ainfo->ambient_lock);

	return ret;
}

static ssize_t aod_area_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo = NULL;
	struct ambient_ctrl_info *ainfo;
	unsigned int x, y, w, h;

	if (pdata_base == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);
	pinfo = &pdata_base->panel_info;

	if (!pinfo)
		return -EINVAL;

	ainfo = (struct ambient_ctrl_info*)pinfo->ambient_info;

	if (!ainfo)
		return -AMBIENT_RETURN_ERROR_NOT_INIT;

	x = ainfo->dst_roi.x;
	y = ainfo->dst_roi.y;
	w = ainfo->dst_roi.w;
	h = ainfo->dst_roi.h;

	return sprintf(buf, "%d %d %d %d\n", x, y, w, h);
}

static ssize_t aod_area_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo = NULL;
	struct ambient_ctrl_info *ainfo = NULL;
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int param[5];
	struct mdss_rect req_area;

	if (pdata_base == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);
	pinfo = &pdata_base->panel_info;
	if (!pinfo)
		return -EINVAL;

	ainfo = (struct ambient_ctrl_info*)pinfo->ambient_info;
	if (!ainfo)
		return -AMBIENT_RETURN_ERROR_NOT_INIT;

	sscanf(buf, "%d %d %d %d %d", &param[0], &param[1], &param[2], &param[3], &param[4]);

	ainfo->lpmode = ((param[0] > 0) ? true : false);
	req_area.x = (param[1] < 0)? 0 : param[1];
	req_area.y = (param[2] < 0)? 0 : param[2];
	req_area.w = (param[3] < 0)? 0 : param[3];
	req_area.h = (param[4] < 0)? 0 : param[4];

	pr_info("%s: lpmode(%d) x:(%d) y:(%d) w:(%d) h:(%d)\n", __func__, ainfo->lpmode,
				req_area.x, req_area.y, req_area.w, req_area.h);

	if (store_aod_area(ctrl, req_area) < 0) {
		pr_err("%s: Invalid AOD Area requested\n", __func__);
		return -EINVAL;
	}

	return ret;
}

static ssize_t aod_lpmode_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo = NULL;
	struct ambient_ctrl_info *ainfo;
	unsigned int enable = 0;

	if (pdata_base == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
			panel_data);
	pinfo = &pdata_base->panel_info;
	if (!pinfo)
		return -EINVAL;

	ainfo = (struct ambient_ctrl_info*)pinfo->ambient_info;
	if (!ainfo)
		return -AMBIENT_RETURN_ERROR_NOT_INIT;

	if (ainfo->lpmode)
		enable = 1;

	return sprintf(buf, "%d\n", enable);
}
static DEVICE_ATTR(area, S_IRUGO|S_IWUSR|S_IWGRP, aod_area_get, aod_area_set);
static DEVICE_ATTR(lpmode, S_IRUGO, aod_lpmode_get, NULL);

int lge_mdss_ambient_create_sysfs(struct class *panel)
{
	int ret = 0;
	static struct device *sysfs_dev = NULL;

	if (!panel) {
		pr_err("%s: invalid input panel class\n", __func__);
		return -EINVAL;
	}

	if (!sysfs_dev) {
		sysfs_dev = device_create(panel, NULL, 0, NULL, "aod");
		if (IS_ERR(sysfs_dev)) {
			pr_err("%s: Failed to create dev!\n", __func__);
		}
		else{
			if (device_create_file(sysfs_dev, &dev_attr_area) < 0)
				pr_err("%s: add area set node fail!", __func__);
			if (device_create_file(sysfs_dev, &dev_attr_lpmode) < 0)
				pr_err("%s: add lpmode set node fail!", __func__);
		}
	}

	return ret;
}
