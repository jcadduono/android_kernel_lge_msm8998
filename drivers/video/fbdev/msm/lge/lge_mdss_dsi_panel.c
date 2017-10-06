/*
 * Copyright(c) 2016, LG Electronics. All rights reserved.
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

#define pr_fmt(fmt)	"[Display] %s: " fmt, __func__

#include <linux/of_platform.h>
#include "../mdss_dsi.h"
#include "lge_mdss_dsi_panel.h"
#include <linux/delay.h>

#if defined(CONFIG_LGE_DISPLAY_CONTROL)
#include "lge_display_control.h"
#if defined(CONFIG_LGE_DISPLAY_COMFORT_MODE)
#include "lge_comfort_view.h"
#endif
#endif
#if defined(CONFIG_LGE_DISPLAY_AMBIENT_SUPPORTED)
#include "lge_mdss_ambient.h"
#endif /* CONFIG_LGE_DISPLAY_AMBIENT_SUPPORTED */
#if defined(CONFIG_LGE_DISPLAY_ERROR_DETECT)
#include "lge_error_detect.h"
#endif

struct mdss_panel_data *pdata_base;

char *lge_blmap_name[] = {
	"lge,blmap",
#if defined(CONFIG_LGE_HIGH_LUMINANCE_MODE)
	"lge,blmap-hl",
#endif /* CONFIG_LGE_HIGH_LUMINANCE_MODE */
#if defined(CONFIG_LGE_DISPLAY_CONTROL)
#if defined(CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT)
	"lge,blmap-ve",
#endif /* CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT */
#endif /* CONFIG_LGE_DISPLAY_CONTROL */
#if defined(CONFIG_LGE_DISPLAY_AMBIENT_SUPPORTED)
	"lge,blmap-ex",
#endif /* CONFIG_LGE_DISPLAY_AMBIENT_SUPPORTED */
};

#if defined(CONFIG_LGE_DISPLAY_COMMON)
extern int panel_not_connected;
#endif /* CONFIG_LGE_DISPLAY_COMMON*/

extern int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_panel_cmds *pcmds, u32 flags);

void lge_mdss_dsi_parse_dcs_cmds_by_name_array(struct device_node *np,
		struct lge_dsi_cmds_entry **lge_dsi_cmds_list,
		char *cmd_name_array[],
		int num_cmds)
{
	int i, rc = 0;
	char *name;
	char cmd_state[128];
	struct lge_dsi_cmds_entry *cmds_list = NULL;

	if (np == NULL || num_cmds == 0 || cmd_name_array == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	if (*lge_dsi_cmds_list == NULL) {
		cmds_list = kzalloc(sizeof(struct lge_dsi_cmds_entry) *	num_cmds, GFP_KERNEL);
		*lge_dsi_cmds_list = cmds_list;
	} else {
		pr_err("This cmd list is already allocated.\n");
		return;
	}

	if (cmds_list == NULL) {
		pr_err("no memory\n");
		return;
	}

	for (i = 0; i < num_cmds; ++i) {
		name = cmd_name_array[i];
		strlcpy(cmds_list[i].name, name,
						sizeof(cmds_list[i].name));
		strlcpy(cmd_state, name, sizeof(cmd_state));
		strcat(cmd_state, "-state");
		pr_info("name : %s, cmd state : %s\n", name, cmd_state);
		rc = mdss_dsi_parse_dcs_cmds(np,	&cmds_list[i].lge_dsi_cmds, name, cmd_state);
		if (!rc)
			pr_info("lge_dsi_cmds_list[%d].lge_dsi_cmds : 0x%02x\n", i, cmds_list[i].lge_dsi_cmds.cmds[0].payload[0]);
	}
}

void lge_mdss_dsi_send_dcs_cmds_list(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
				struct lge_dsi_cmds_entry *lge_dsi_cmds_list, int num_cmds)
{
	int i;
	for (i = 0; i < num_cmds ; i++) {
		if (lge_dsi_cmds_list[i].lge_dsi_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(ctrl_pdata,
							&lge_dsi_cmds_list[i].lge_dsi_cmds,
							CMD_REQ_COMMIT);
	}
}

void lge_mdss_dsi_send_dcs_cmds_by_cmd_name(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
				struct lge_dsi_cmds_entry *lge_dsi_cmds_list, int num_cmds, const char *cmd_name)
{
	int i, index = -1;
	for (i = 0; i < num_cmds; ++i) {
		if (!strcmp(lge_dsi_cmds_list[i].name, cmd_name)) {
			index = i;
			break;
		}
	}

	if (index != -1) {
		if (lge_dsi_cmds_list[index].lge_dsi_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(ctrl_pdata,
							&lge_dsi_cmds_list[index].lge_dsi_cmds,
							CMD_REQ_COMMIT);
	} else {
		pr_err("cmds %s not found\n", cmd_name);
	}
}

#if defined(CONFIG_LGE_DISPLAY_BIST_MODE)
int lge_mdss_dsi_bist_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, bool enable)
{
	if (enable) {
		if (ctrl->bist_on == 0) {
			mdss_dsi_panel_cmds_send(ctrl,
				&ctrl->bist_on_cmds,
				CMD_REQ_COMMIT);
		}
		ctrl->bist_on++;
	} else {
		if (ctrl->bist_on == 1) {
			mdss_dsi_panel_cmds_send(ctrl,
				&ctrl->bist_off_cmds,
				CMD_REQ_COMMIT);
		}
		ctrl->bist_on--;
		if (ctrl->bist_on < 0) {
			pr_err("count (%d) -> (0) : debugging!\n", ctrl->bist_on);
			ctrl->bist_on = 0;
		}
	}
	pr_info("count(%d)\n", ctrl->bist_on);
	return 0;
}

void lge_mdss_dsi_bist_release(struct mdss_dsi_ctrl_pdata *ctrl)
{
	if (ctrl->bist_on > 0) {
		ctrl->bist_on = 1;
		if (lge_mdss_dsi_bist_ctrl(ctrl, false) < 0) {
			pr_warn("fail to bist control\n");
		}
	}
	pr_info("%d\n", ctrl->bist_on);
}
#endif

char lge_dcs_cmd[2] = {0x00, 0x00}; /* DTYPE_DCS_READ */
struct dsi_cmd_desc lge_dcs_read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 1, sizeof(lge_dcs_cmd)},
	lge_dcs_cmd
};

int lge_mdss_dsi_panel_cmd_read(char cmd0, int cnt, char* ret_buf)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_dsi_ctrl_pdata *ctrl;
	char rx_buf[128] = {0x0};
	int i = 0;

#if defined(CONFIG_LGE_DISPLAY_COMMON)
	if(panel_not_connected) {
		pr_err("Skip Panel Cmd Read : Panel not connected.\n");
		return -EINVAL;
	}
#endif /* CONFIG_LGE_DISPLAY_COMMON*/

	if (pdata_base == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl = container_of(pdata_base, struct mdss_dsi_ctrl_pdata,
						panel_data);
	lge_dcs_cmd[0] = cmd0;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &lge_dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = cnt;
	cmdreq.cb = NULL;
	cmdreq.rbuf = rx_buf;

	if (ctrl->status_cmds.link_state == DSI_LP_MODE)
		cmdreq.flags |= CMD_REQ_LP_MODE;
	else if (ctrl->status_cmds.link_state == DSI_HS_MODE)
		cmdreq.flags |= CMD_REQ_HS_MODE;

	mdelay(1);

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	for (i = 0; i < cnt; i++)
		pr_info("Reg[0x%x], buf[%d]=0x%x\n",cmd0, i, rx_buf[i]);

	memcpy(ret_buf, rx_buf, cnt);

	return 0;
}

char *lge_get_blmapname(enum lge_bl_map_type  blmaptype)
{
	if (blmaptype >= 0 && blmaptype < LGE_BLMAPMAX)
		return lge_blmap_name[blmaptype];
	else
		return lge_blmap_name[LGE_BLDFT];
}

void lge_mdss_panel_parse_dt_blmaps(struct device_node *np,
				   struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int i, j, rc;
	u32 *array;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	pinfo->blmap_size = 256;
	array = kzalloc(sizeof(u32) * pinfo->blmap_size, GFP_KERNEL);

	if (!array)
		return;

	for (i = 0; i < LGE_BLMAPMAX; i++) {
		/* check if property exists */
		if (!of_find_property(np, lge_blmap_name[i], NULL))
			continue;

		pr_info("found %s\n", lge_blmap_name[i]);

		rc = of_property_read_u32_array(np, lge_blmap_name[i], array,
						pinfo->blmap_size);
		if (rc) {
			pr_err("%d, unable to read %s\n", __LINE__, lge_blmap_name[i]);
			goto error;
		}

		pinfo->blmap[i] = kzalloc(sizeof(int) * pinfo->blmap_size,
				GFP_KERNEL);

		if (!pinfo->blmap[i]){
			goto error;
		}

		for (j = 0; j < pinfo->blmap_size; j++)
			pinfo->blmap[i][j] = array[j];
	}

	kfree(array);
	return;

error:
	for (i = 0; i < LGE_BLMAPMAX; i++)
		if (pinfo->blmap[i])
			kfree(pinfo->blmap[i]);
	kfree(array);
}

static int lge_mdss_dsi_panel_create_sysfs(struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata)
{
	int rc = 0;
	static struct class *panel = NULL;
	if (!panel) {
		panel = class_create(THIS_MODULE, "panel");
		if (IS_ERR(panel)) {
			pr_err("Failed to create panel class\n");
			return -EINVAL;
		}
	}

	if (lge_ctrl_pdata != NULL && lge_ctrl_pdata->create_panel_sysfs) {
		rc = lge_ctrl_pdata->create_panel_sysfs(panel);
		if (rc < 0)
			pr_err("Panel-dependent sysfs creation failed\n");
	}

#if defined(CONFIG_LGE_DISPLAY_CONTROL)
	if ((rc = lge_display_control_create_sysfs(panel)) < 0) {
		pr_err("fail to create display control sysfs\n");
	}
#endif /* CONFIG_LGE_DISPLAY_CONTROL */
#if defined(CONFIG_LGE_DISPLAY_ERROR_DETECT)
	if ((rc = lge_dsi_err_detect_create_sysfs(panel)) < 0) {
		pr_err("fail to create error detect sysfs\n");
	}
#endif /* CONFIG_LGE_DISPLAY_ERROR_DETECT */
#if defined(CONFIG_LGE_DISPLAY_AMBIENT_SUPPORTED)
	if ((rc = lge_mdss_ambient_create_sysfs(panel)) < 0) {
		pr_err("fail to create ambient sysfs\n");
	}
#endif /* CONFIG_LGE_DISPLAY_AMBIENT_SUPPORTED */
	return rc;
}

static int lge_mdss_panel_parse_dt(struct device_node *np,
						struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (np == NULL || ctrl_pdata == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;

	if (lge_ctrl_pdata == NULL) {
		pr_err("Invalid pdata state\n");
		return -EINVAL;
	}

	if (lge_ctrl_pdata->parse_dt_blmaps)
		lge_ctrl_pdata->parse_dt_blmaps(np, ctrl_pdata);
	else
		lge_mdss_panel_parse_dt_blmaps(np, ctrl_pdata);

	if (lge_ctrl_pdata->parse_dt_panel_ctrl)
		lge_ctrl_pdata->parse_dt_panel_ctrl(np, ctrl_pdata);

#if defined(CONFIG_LGE_DISPLAY_CONTROL)
	mdss_dsi_parse_display_control_dcs_cmds(np, ctrl_pdata);
#endif /* CONFIG_LGE_DISPLAY_CONTROL*/
#if defined(CONFIG_LGE_DISPLAY_ERROR_DETECT)
	lge_dsi_err_detect_parse_dt(np, ctrl_pdata);
#endif /* CONFIG_LGE_DISPLAY_ERROR_DETECT */
#if defined(CONFIG_LGE_DISPLAY_BIST_MODE)
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->bist_on_cmds,
			"lge,bist-on-cmds", "qcom,mode-control-dsi-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->bist_off_cmds,
			"lge,bist-off-cmds", "qcom,mode-control-dsi-state");
#endif
	return 0;
}

int lge_mdss_dsi_panel_init(struct device_node *node, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;

	if (ctrl_pdata->panel_data.panel_info.pdest == DISPLAY_1) {
		if (pdata_base == NULL)
			pdata_base = &(ctrl_pdata->panel_data);
	}

	lge_ctrl_pdata = kzalloc(sizeof(struct lge_mdss_dsi_ctrl_pdata), GFP_KERNEL);
	if (!lge_ctrl_pdata) {
		pr_err("Unable to alloc mem for lge_crtl_pdata\n");
		rc = -ENOMEM;
		goto mem_fail;
	}

	pr_err("ctrl_pdata = %p, lge_ctrl_pdata = %p\n", ctrl_pdata, lge_ctrl_pdata);
	lge_ctrl_pdata->lge_blmap_list = lge_blmap_name;
	ctrl_pdata->lge_ctrl_pdata = lge_ctrl_pdata;

	/* This funciton should be defined under each model directory */
	lge_mdss_dsi_panel_init_sub(lge_ctrl_pdata);

#if defined(CONFIG_LGE_DISPLAY_CONTROL)
	rc = lge_display_control_init(ctrl_pdata);
	if (rc) {
		pr_err("fail to init display control (rc:%d)\n", rc);
		return rc;
	}
#endif /* CONFIG_LGE_DISPLAY_CONTROL */

#if defined(CONFIG_LGE_DISPLAY_AMBIENT_SUPPORTED)
	rc = lge_mdss_ambient_init(node, ctrl_pdata);
	if (rc) {
		pr_err("[Ambient] fail to init (rc:%d)\n", rc);
		return rc;
	}
#endif

	lge_mdss_panel_parse_dt(node, ctrl_pdata);
	rc = lge_mdss_dsi_panel_create_sysfs(lge_ctrl_pdata);
mem_fail:
	return rc;
}
