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

#define pr_fmt(fmt)		"[Display] %s: " fmt, __func__

#include <linux/delay.h>
#include "../../mdss_dsi.h"
#include <soc/qcom/lge/board_lge.h>
#include <linux/input/lge_touch_notify.h>
#include "../lge_mdss_dsi.h"

#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMMON)
extern int mdss_dsi_pinctrl_set_state(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
		bool active);
#endif

#ifdef CONFIG_LGE_LCD_POWER_CTRL
extern int panel_not_connected;

int lge_panel_power_off(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	int ret = 0;
	struct mdss_panel_info *pinfo;

	if (pdata == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &ctrl_pdata->panel_data.panel_info;

	pr_info("+ ndx=%d\n", ctrl_pdata->ndx);

	if (!(panel_not_connected && detect_factory_cable() &&
						!lge_get_mfts_mode())) {
		if (mdss_dsi_is_right_ctrl(ctrl_pdata)) {
			pr_err("%d, right ctrl configuration not needed\n", __LINE__);
			return ret;
		}
	}

	/* 1st : reset */
	ret = mdss_dsi_panel_reset(pdata, 0);
	if (ret) {
		pr_warn("Panel reset failed. rc=%d\n", ret);
		ret = 0;
	}

	if (mdss_dsi_pinctrl_set_state(ctrl_pdata, false))
		pr_debug("reset disable: pinctrl not enabled\n");

	ret = msm_dss_enable_vreg(
		ctrl_pdata->panel_power_data.vreg_config,
		ctrl_pdata->panel_power_data.num_vreg, 0);
	if (ret)
		pr_err("failed to disable vregs for %s\n", __mdss_dsi_pm_name(DSI_PANEL_PM));

	/* 2nd : vddio */
	lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);
	usleep_range(1000, 1000);

	/* 3rd : vpnl */
	if(pinfo->panel_type == LGD_SIW_LG43402_1440_2880_ONCELL_CMD_PANEL)
	{
		lge_extra_gpio_set_value(ctrl_pdata, "vpnl", 0);
		usleep_range(1000,1000);
	}

	pr_info("- ndx=%d\n", ctrl_pdata->ndx);

	return ret;
}

int lge_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo;

	if (pdata == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);

	pinfo = &ctrl_pdata->panel_data.panel_info;

	pr_info("+ ndx=%d\n", ctrl_pdata->ndx);

	if (!(panel_not_connected && detect_factory_cable() &&
						!lge_get_mfts_mode())) {
		if (mdss_dsi_is_right_ctrl(ctrl_pdata)) {
			pr_err("%d, right ctrl configuration not needed\n", __LINE__);
			return ret;
		}
	}
	/* 1st : VDDIO */
	lge_extra_gpio_set_value(ctrl_pdata, "vddio", 1);

	if(pinfo->panel_type == LGD_SIW_LG43402_1440_2880_ONCELL_CMD_PANEL)
	{
		usleep_range(2000,2000);

		/* 2nd : VPNL */
		lge_extra_gpio_set_value(ctrl_pdata, "vpnl", 1);

		usleep_range(2000,2000);
	}

	ret = msm_dss_enable_vreg(
		ctrl_pdata->panel_power_data.vreg_config,
		ctrl_pdata->panel_power_data.num_vreg, 1);
	if (ret) {
		pr_err("failed to enable vregs for %s\n", __mdss_dsi_pm_name(DSI_PANEL_PM));
		return ret;
	}

	/*
	 * If continuous splash screen feature is enabled, then we need to
	 * request all the GPIOs that have already been configured in the
	 * bootloader. This needs to be done irresepective of whether
	 * the lp11_init flag is set or not.
	 */
	if (pdata->panel_info.cont_splash_enabled ||
		!pdata->panel_info.mipi.lp11_init) {
		if (mdss_dsi_pinctrl_set_state(ctrl_pdata, true))
			pr_debug("reset enable: pinctrl not enabled\n");
		/* 3rd : reset */
		ret = mdss_dsi_panel_reset(pdata, 1);
		if (ret)
			pr_err("Panel reset failed. rc=%d\n", ret);
	}
	pr_info("- ndx=%d\n", ctrl_pdata->ndx);

	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
int mdss_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo;

	if (pdata == NULL) {
		pr_err("Invalid input data\n");
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &ctrl_pdata->panel_data.panel_info;

	pr_info("++\n");

	/* 1st : VDDIO */
	lge_extra_gpio_set_value(ctrl_pdata, "vddio", 1);

	if(pinfo->panel_type == LGD_SIW_LG43402_1440_2880_ONCELL_CMD_PANEL)
	{
		usleep_range(2000,2000);

		/* 2nd : VPNL */
		lge_extra_gpio_set_value(ctrl_pdata, "vpnl", 1);

		usleep_range(2000,2000);
	}

	ret = msm_dss_enable_vreg(
		ctrl_pdata->panel_power_data.vreg_config,
		ctrl_pdata->panel_power_data.num_vreg, 1);
	if (ret) {
		pr_err("failed to enable vregs for %s\n", __mdss_dsi_pm_name(DSI_PANEL_PM));
		return ret;
	}

	/*
	 * If continuous splash screen feature is enabled, then we need to
	 * request all the GPIOs that have already been configured in the
	 * bootloader. This needs to be done irresepective of whether
	 * the lp11_init flag is set or not.
	 */
	if (pdata->panel_info.cont_splash_enabled ||
		!pdata->panel_info.mipi.lp11_init) {
		if (mdss_dsi_pinctrl_set_state(ctrl_pdata, true))
			pr_debug("reset enable: pinctrl not enabled\n");
		/* 3rd : reset */
		ret = mdss_dsi_panel_reset(pdata, 1);
		if (ret)
			pr_err("Panel reset failed. rc=%d\n", ret);
	}

	pr_info("--\n");
	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_panel_power_off(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo;

	if (pdata == NULL) {
		pr_err("Invalid input data\n");
		ret = -EINVAL;
		goto end;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &ctrl_pdata->panel_data.panel_info;

	pr_info("++\n");

	/* 1st : reset */
	ret = mdss_dsi_panel_reset(pdata, 0);
	if (ret) {
		pr_warn("Panel reset failed. rc=%d\n", ret);
		ret = 0;
	}

	if (mdss_dsi_pinctrl_set_state(ctrl_pdata, false))
		pr_debug("reset disable: pinctrl not enabled\n");

	ret = msm_dss_enable_vreg(
		ctrl_pdata->panel_power_data.vreg_config,
		ctrl_pdata->panel_power_data.num_vreg, 0);
	if (ret)
		pr_err("failed to disable vregs for %s\n", __mdss_dsi_pm_name(DSI_PANEL_PM));

	/* 2nd : vddio */
	lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);
	usleep_range(1000, 1000);

	/* 3rd : vpnl */
	if(pinfo->panel_type == LGD_SIW_LG43402_1440_2880_ONCELL_CMD_PANEL)
	{
		lge_extra_gpio_set_value(ctrl_pdata, "vpnl", 0);
		usleep_range(1000,1000);
	}

	pr_info("--\n");
end:
	return ret;

}
#endif
