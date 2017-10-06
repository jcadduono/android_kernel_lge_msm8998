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

#include <linux/delay.h>
#include "../../mdss_dsi.h"
#include "../lge_mdss_display.h"
#include "../../mdss_dba_utils.h"
#include <linux/input/lge_touch_notify.h>
#include <soc/qcom/lge/board_lge.h>
#include "../lge_mdss_dsi_panel.h"
#if defined(CONFIG_LGE_DISPLAY_CONTROL)
#include "../lge_display_control.h"
#endif /* CONFIG_LGE_DISPLAY_CONTROL */
#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMFORT_MODE)
#include "../lge_comfort_view.h"
#endif /* CONFIG_LGE_DISPLAY_COMFORT_MODE */
#if defined(CONFIG_LGE_DISPLAY_ERROR_DETECT)
#include "../lge_error_detect.h"
#endif /* CONFIG_LGE_DISPLAY_ERROR_DETECT */
#if defined(CONFIG_LGE_MIPI_JOAN_ONCELL_QHD_CMD_PANEL)
#include <soc/qcom/lge/power/lge_board_revision.h>
#include <soc/qcom/lge/board_lge.h>
#endif /* CONFIG_LGE_MIPI_JOAN_ONCELL_QHD_CMD_PANEL */
#if defined(CONFIG_LGE_DISPLAY_INTERNAL_TEST)
#include "lge_internal_test.h"
#endif /* CONFIG_LGE_DISPLAY_INTERNAL_TEST */

extern struct mdss_panel_data *pdata_base;
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_RESET)
int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		rc = gpio_request(ctrl_pdata->disp_en_gpio,
						"disp_enable");
		if (rc) {
			pr_err("request disp_en gpio failed, rc=%d\n",
					   rc);
			goto disp_en_gpio_err;
		}
	}
	rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
	if (rc) {
		pr_err("request reset gpio failed, rc=%d\n",
			rc);
		goto rst_gpio_err;
	}
	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
		rc = gpio_request(ctrl_pdata->bklt_en_gpio,
						"bklt_enable");
		if (rc) {
			pr_err("request bklt gpio failed, rc=%d\n",
					   rc);
			goto bklt_en_gpio_err;
		}
	}

	if (gpio_is_valid(ctrl_pdata->lcd_mode_sel_gpio)) {
		rc = gpio_request(ctrl_pdata->lcd_mode_sel_gpio, "mode_sel");
		if (rc) {
			pr_err("request dsc/dual mode gpio failed,rc=%d\n",
								rc);
			goto lcd_mode_sel_gpio_err;
		}
	}

	return rc;

lcd_mode_sel_gpio_err:
	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
		gpio_free(ctrl_pdata->bklt_en_gpio);
bklt_en_gpio_err:
	gpio_free(ctrl_pdata->rst_gpio);
rst_gpio_err:
	if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
		gpio_free(ctrl_pdata->disp_en_gpio);
disp_en_gpio_err:
	return rc;

}

int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int i, rc = 0;

	if (pdata == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &(ctrl_pdata->panel_data.panel_info);
	if ((mdss_dsi_is_right_ctrl(ctrl_pdata) &&
		mdss_dsi_is_hw_config_split(ctrl_pdata->shared_data)) ||
			pinfo->is_dba_panel) {
		pr_debug("%d, right ctrl gpio configuration not needed\n", __LINE__);
		return rc;
	}

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%d, reset line not configured\n", __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%d, reset line not configured\n", __LINE__);
		return rc;
	}

	pr_info("enable = %d\n", enable);

	if (enable) {
		rc = mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
		if (!pinfo->cont_splash_enabled) {
			if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
				rc = gpio_direction_output(
					ctrl_pdata->disp_en_gpio, 1);
				if (rc) {
					pr_err("unable to set dir for en gpio\n");
					goto exit;
				}
			}

			if (pdata->panel_info.rst_seq_len) {
				rc = gpio_direction_output(ctrl_pdata->rst_gpio,
					pdata->panel_info.rst_seq[0]);
				if (rc) {
					pr_err("unable to set dir for rst gpio\n");
					goto exit;
				}
			}

			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				gpio_set_value((ctrl_pdata->rst_gpio),
					pdata->panel_info.rst_seq[i]);
				if (pdata->panel_info.rst_seq[++i])
					usleep_range(pinfo->rst_seq[i] * 1000, pinfo->rst_seq[i] * 1000);
			}

			if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
				rc = gpio_direction_output(
					ctrl_pdata->bklt_en_gpio, 1);
				if (rc) {
					pr_err("unable to set dir for bklt gpio\n");
					goto exit;
				}
			}
		}

		if (gpio_is_valid(ctrl_pdata->lcd_mode_sel_gpio)) {
			bool out = false;

			if ((pinfo->mode_sel_state == MODE_SEL_SINGLE_PORT) ||
				(pinfo->mode_sel_state == MODE_GPIO_HIGH))
				out = true;
			else if ((pinfo->mode_sel_state == MODE_SEL_DUAL_PORT)
				|| (pinfo->mode_sel_state == MODE_GPIO_LOW))
				out = false;

			rc = gpio_direction_output(
					ctrl_pdata->lcd_mode_sel_gpio, out);
			if (rc) {
				pr_err("unable to set dir for mode gpio\n");
				goto exit;
			}
		}

		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("Panel Not properly turned OFF\n");
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("Reset panel done\n");
		}
	} else {
#if defined(CONFIG_LGE_DISPLAY_ERROR_DETECT)
		if (ctrl_pdata->err_irq_enabled) {
			lge_dsi_irq_control(ctrl_pdata, false);
		}
#endif
		if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
			gpio_set_value((ctrl_pdata->bklt_en_gpio), 0);
			gpio_free(ctrl_pdata->bklt_en_gpio);
		}
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
			gpio_free(ctrl_pdata->disp_en_gpio);
		}
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
		gpio_free(ctrl_pdata->rst_gpio);
		if (gpio_is_valid(ctrl_pdata->lcd_mode_sel_gpio)) {
			gpio_set_value(ctrl_pdata->lcd_mode_sel_gpio, 0);
			gpio_free(ctrl_pdata->lcd_mode_sel_gpio);
		}
	}

exit:
	return rc;
}
#endif

extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_panel_cmds *pcmds, u32 flags);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_ON)
int mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
#if defined(CONFIG_LGE_DISPLAY_CONTROL)
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;
#endif
	struct mdss_panel_info *pinfo;
	struct dsi_panel_cmds *on_cmds;
	int ret = 0;

	if (pdata == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
#if defined(CONFIG_LGE_DISPLAY_CONTROL)
	lge_ctrl_pdata = ctrl->lge_ctrl_pdata;
#endif

	pr_info("ndx=%d\n", ctrl->ndx);

	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			goto end;
	}

	on_cmds = &ctrl->on_cmds;

	if ((pinfo->mipi.dms_mode == DYNAMIC_MODE_SWITCH_IMMEDIATE) &&
			(pinfo->mipi.boot_mode != pinfo->mipi.mode))
		on_cmds = &ctrl->post_dms_on_cmds;

	pr_debug("ndx=%d cmd_cnt=%d\n",
				ctrl->ndx, on_cmds->cmd_cnt);

	if (on_cmds->cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, on_cmds, CMD_REQ_COMMIT);

	if (pinfo->compression_mode == COMPRESSION_DSC)
		mdss_dsi_panel_dsc_pps_send(ctrl, pinfo);

	if (ctrl->ds_registered)
		mdss_dba_utils_video_on(pinfo->dba_data, pinfo);

#if defined(CONFIG_LGE_DISPLAY_CONTROL)
#if defined(CONFIG_LGE_DISPLAY_COLOR_MANAGER)
		lge_set_screen_mode(ctrl, true);
#endif

#if defined(CONFIG_LGE_DISPLAY_COMFORT_MODE)
	if (lge_ctrl_pdata->comfort_view) {
		lge_mdss_comfort_view_cmd_send(pdata, lge_ctrl_pdata->comfort_view);
		lge_ctrl_pdata->dgc_status = 0x01;
		lge_display_control_store(ctrl, true);
	}
#endif
#endif

#if defined(CONFIG_LGE_DISPLAY_ERROR_DETECT)
	lge_mdss_dsi_panel_err_cmds_send(ctrl, DSI_LP_MODE);
	if (!ctrl->err_irq_enabled) {
		lge_dsi_irq_control(ctrl, true);
	}
#endif
end:
	pr_info("-\n");
	return ret;

}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_OFF)
int mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;

	if (pdata == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_info("ctrl=%pK ndx=%d\n", ctrl, ctrl->ndx);

	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			goto end;
	}

	if (ctrl->off_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds, CMD_REQ_COMMIT);

	if (ctrl->ds_registered && pinfo->is_pluggable) {
		mdss_dba_utils_video_off(pinfo->dba_data);
		mdss_dba_utils_hdcp_enable(pinfo->dba_data, false);
	}

end:
	pr_info("-\n");
	return 0;

}
#endif

void lge_mdss_panel_parse_dt_blmaps_joan(struct device_node *np,
				   struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int i, j, rc;
	u32 *array;
	char blmap_rev[30];
	enum lge_panel_version p_ver = LGE_PANEL_V1;

	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	pinfo->blmap_size = 512;
	array = kzalloc(sizeof(u32) * pinfo->blmap_size, GFP_KERNEL);

	if (!array)
		return;

	if(lge_get_board_rev_no() < HW_REV_1_0)
		p_ver = lge_get_panel_flag_status();

	for (i = 0; i < LGE_BLMAPMAX; i++) {
		snprintf(blmap_rev, sizeof(blmap_rev),lge_ctrl_pdata->lge_blmap_list[i]);

		if(p_ver == LGE_PANEL_V1)
			strcat(blmap_rev, "_v1");

		/* check if property exists */
		if (!of_find_property(np, blmap_rev, NULL))
			continue;

		pr_info("found %s\n", blmap_rev);

		rc = of_property_read_u32_array(np, blmap_rev, array,
						pinfo->blmap_size);
		if (rc) {
			pr_err("%d, unable to read %s\n", __LINE__, blmap_rev);
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

#if defined(CONFIG_LGE_FACTORY_BIST_MODE)
	if(lge_get_factory_boot() == true) {
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->factory_bist_on_cmds,
		"lge,bist-on-cmds", "lge,mdss-dsi-hs-command-state");
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->factory_bist_off_cmds,
		"lge,bist-off-cmds", "lge,mdss-dsi-hs-command-state");
	}
#endif /* CONFIG_LGE_FACTORY_BIST_MODE */

	kfree(array);
	return;

error:
	for (i = 0; i < LGE_BLMAPMAX; i++)
		if (pinfo->blmap[i])
			kfree(pinfo->blmap[i]);
	kfree(array);
}


void lge_mdss_panel_parse_dt_panel_ctrl_joan(struct device_node *np,
				   struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
#if defined(CONFIG_LGE_DISPLAY_INTERNAL_TEST)
	mdss_dsi_parse_internal_dcs_cmds(np, ctrl_pdata);
#endif
}

int lge_mdss_panel_create_panel_sysfs_joan(struct class *panel)
{
	int rc = 0;
#if defined(CONFIG_LGE_DISPLAY_INTERNAL_TEST)
	if (lge_internal_test_create_sysfs(panel) < 0) {
		pr_err("fail to create internal test sysfs\n");
	}
#endif /* CONFIG_LGE_DISPLAY_INTERNAL_TEST */

	return rc;
}

int lge_mdss_dsi_panel_init_sub(struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata)
{
	int rc = 0;

	lge_ctrl_pdata->parse_dt_blmaps = lge_mdss_panel_parse_dt_blmaps_joan;
	lge_ctrl_pdata->parse_dt_panel_ctrl = lge_mdss_panel_parse_dt_panel_ctrl_joan;
	lge_ctrl_pdata->create_panel_sysfs = lge_mdss_panel_create_panel_sysfs_joan;

	return rc;
}
