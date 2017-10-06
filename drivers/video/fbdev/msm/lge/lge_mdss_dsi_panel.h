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

#ifndef _LGE_MDSS_DSI_PANEL_H
#define _LGE_MDSS_DSI_PANEL_H

int lge_mdss_dsi_panel_cmd_read(char cmd0, int cnt, char* ret_buf);
char *lge_get_blmapname(enum lge_bl_map_type  blmaptype);
void lge_display_control_store(struct mdss_dsi_ctrl_pdata *ctrl, bool cmd_send);
void lge_set_screen_mode(struct mdss_dsi_ctrl_pdata *ctrl, bool cmd_send);
int lge_mdss_dsi_panel_init(struct device_node *node,
					struct mdss_dsi_ctrl_pdata *ctrl_pdata);

/* This function should be defined under each model directory */
extern int lge_mdss_dsi_panel_init_sub(
					struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata);

#if defined(CONFIG_LGE_MIPI_JOAN_ONCELL_QHD_CMD_PANEL)
enum lge_panel_version {
	LGE_PANEL_V0 = 0,
	LGE_PANEL_V1,
	LGE_PANEL_MAX
};
#endif /* CONFIG_LGE_MIPI_JOAN_ONCELL_QHD_CMD_PANEL */

#ifdef CONFIG_LGE_DISPLAY_VR_MODE
void mdss_dsi_panel_apply_settings(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_panel_cmds *pcmds);
#endif
#ifdef CONFIG_LGE_DISPLAY_BL_EXTENDED
int mdss_panel_parse_blex_settings(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata);
#endif
#if defined(CONFIG_LGE_DISPLAY_BIST_MODE)
int lge_mdss_dsi_bist_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, bool enable);
void lge_mdss_dsi_bist_release(struct mdss_dsi_ctrl_pdata *ctrl);
#endif
#endif
