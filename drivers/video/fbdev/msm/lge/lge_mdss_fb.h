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

#ifndef LGE_MDSS_FB_H
#define LGE_MDSS_FB_H

#include <soc/qcom/lge/board_lge.h>
#include "../mdss_dsi.h"
#include "lge_mdss_dsi.h"

int lge_br_to_bl(struct msm_fb_data_type *mfd, int br_lvl);
#if defined(CONFIG_LGE_DISPLAY_AMBIENT_SUPPORTED)
int lge_br_to_bl_ex(struct msm_fb_data_type *mfd, int br_lvl);
void mdss_fb_update_backlight_ex(struct msm_fb_data_type *mfd);
#endif
#if defined(CONFIG_LGE_SP_MIRRORING_CTRL_BL)
int lge_is_bl_update_blocked(int bl_lvl);
void lge_set_bl_update_blocked(bool enable);
#endif
#ifdef CONFIG_LGE_LCD_OFF_DIMMING
void lge_set_blank_called(void);
#endif
void lge_mdss_fb_init(struct msm_fb_data_type *mfd);
void lge_mdss_fb_remove_sysfs(struct msm_fb_data_type *mfd);
int lge_mdss_fb_create_sysfs(struct msm_fb_data_type *mfd);
#if defined(CONFIG_LGE_DISPLAY_AMBIENT_SUPPORTED)
void lge_ambient_brightness_register(struct msm_fb_data_type *mfd);
void lge_ambient_brightness_unregister(void);
#endif
#if defined(CONFIG_LGE_DISPLAY_DYNAMIC_RESOLUTION_SWITCH)
void mdss_fb_drs_notify(struct kobject *kobj, int state);
#endif

#endif /* LGE_MDSS_FB_H */
