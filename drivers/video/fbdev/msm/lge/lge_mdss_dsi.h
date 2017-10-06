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

#ifndef LGE_MDSS_DSI_H
#define LGE_MDSS_DSI_H
#include "../mdss_dsi.h"
#include <linux/interrupt.h>
#include <linux/irq.h>

#define NUM_COLOR_MODES 	10

struct lge_supply_entry {
	char name[32];
};

struct lge_gpio_entry {
	char name[32];
	int gpio;
};

struct lge_dsi_cmds_entry {
	char name[128];
	struct dsi_panel_cmds lge_dsi_cmds;
};

#if defined(CONFIG_LGE_DISPLAY_COLOR_MANAGER)
struct lge_dsi_color_manager_mode_entry {
	u32 color_manager_mode;
	u32 color_manager_status;
};
#endif

struct lge_mdss_dsi_ctrl_pdata {
	/* gpio */
	int num_gpios;
	struct lge_gpio_entry *gpio_array;
	char **lge_blmap_list;

#if defined(CONFIG_LGE_DISPLAY_ERROR_DETECT)
	irqreturn_t (*err_irq_handler)(int irq, void *data);
	void (*err_work)(struct work_struct *work);
#endif
	void (*parse_dt_blmaps)(struct device_node *np,
					struct mdss_dsi_ctrl_pdata *ctrl_pdata);
	void (*parse_dt_panel_ctrl)(struct device_node *np,
					struct mdss_dsi_ctrl_pdata *ctrl_pdata);
	int (*create_panel_sysfs)(struct class *panel);

#if defined(CONFIG_LGE_DISPLAY_CONTROL)
	/* display control interface */
	void (*display_control_store)(struct mdss_dsi_ctrl_pdata *ctrl_pdata, bool cmd_send);
	void (*display_control_parse_dt)(struct device_node *np,
					struct mdss_dsi_ctrl_pdata *ctrl_pdata);
	struct lge_dsi_cmds_entry *disp_ctrl_cmds_list;
	struct dsi_panel_cmds white_d65_cmds;
	int dgc_status;
	int sharpness_status;
	int boost_status;
	int contrast_status;
#if defined(CONFIG_LGE_DISPLAY_VR_MODE)
	int vr_status;
#endif
	int color_manager_status;
	int color_manager_mode;
	int hdr_hbm_lut;
	int hdr_mode;
	int hbm_mode;
	int acl_mode;
	int white_target;
#if defined(CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT)
	int video_enhancement;
	struct dsi_panel_cmds bc_dim_cmds;
	struct dsi_panel_cmds bc_default_cmds;
	struct delayed_work bc_dim_work;
	bool bc_reg_backup_flag;
#endif /* CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT */
#if defined(CONFIG_LGE_DISPLAY_COLOR_MANAGER)
	bool is_backup;
	int screen_mode;
	struct dsi_panel_cmds d65_default_cmds;
	struct lge_dsi_cmds_entry *color_modes_cmds;
	int cm_preset_step;
	int cm_red_step;
	int cm_green_step;
	int cm_blue_step;
	struct lge_dsi_color_manager_mode_entry color_manager_table[NUM_COLOR_MODES];
	u32 color_manager_table_len;
	bool color_manager_default_status;
#endif
#if defined(CONFIG_LGE_DISPLAY_COMFORT_MODE)
	int comfort_view;
	struct dsi_panel_cmds *dg_preset_cmds;
#endif
#if defined(CONFIG_LGE_ENHANCE_GALLERY_SHARPNESS)
	struct dsi_panel_cmds sharpness_on_cmds;
	struct dsi_panel_cmds ce_on_cmds;
#endif
#endif /* CONFIG_LGE_DISPLAY_CONTROL*/
#if defined(CONFIG_LGE_DISPLAY_INTERNAL_TEST)
	struct lge_dsi_cmds_entry *internal_test_cmds_list;
#endif
};

#define LGE_MDELAY(m) do { if ( m > 0) usleep_range((m)*1000,(m)*1000); } while(0)
#define LGE_OVERRIDE_VALUE(x, v) do { if ((v)) (x) = (v); } while(0)

#include "lge_mdss_dsi_panel.h"

int lge_mdss_dsi_parse_extra_params(
	struct platform_device *ctrl_pdev,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_mdss_dsi_init_extra_pm(struct platform_device *ctrl_pdev,
        struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
void lge_mdss_dsi_deinit_extra_pm(struct platform_device *pdev,
        struct mdss_dsi_ctrl_pdata *ctrl_pdata);
void lge_extra_gpio_set_value(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
		const char *name, int value);

void lge_mdss_dsi_parse_dcs_cmds_by_name_array(struct device_node *np,
	struct lge_dsi_cmds_entry **lge_dsi_cmds_list,
	char *cmd_name_array[],
	int num_cmds);
void lge_mdss_dsi_send_dcs_cmds_list(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	struct lge_dsi_cmds_entry *lge_dsi_cmds_list, int num_cmds);
void lge_mdss_dsi_send_dcs_cmds_by_cmd_name(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	struct lge_dsi_cmds_entry *lge_dsi_cmds_list, int num_cmds, const char *cmd_name);

int lge_panel_power_on(struct mdss_panel_data *pdata);
int lge_panel_power_off(struct mdss_panel_data *pdata);
int detect_factory_cable(void);
int detect_qem_factory_cable(void);
#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
void mdss_dsi_ctrl_shutdown(struct platform_device *pdev);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_ON)
int mdss_dsi_panel_on(struct mdss_panel_data *pdata);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_OFF)
int mdss_dsi_panel_off(struct mdss_panel_data *pdata);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
int mdss_dsi_panel_power_on(struct mdss_panel_data *pdata);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_panel_power_off(struct mdss_panel_data *pdata);
#endif

#endif /* LGE_MDSS_DSI_H */
