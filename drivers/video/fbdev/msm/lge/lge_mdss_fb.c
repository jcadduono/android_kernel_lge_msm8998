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

#include <linux/msm_mdp.h>
#include "../mdss_fb.h"
#ifdef CONFIG_LGE_PM_LGE_POWER_CORE
#include <soc/qcom/lge/power/lge_power_class.h>
#include <soc/qcom/smem.h>
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
#include <soc/qcom/lge/power/lge_cable_detect.h>
#else
#endif
#include <linux/power/lge_battery_id.h>
#include "lge_mdss_display.h"

struct msm_fb_data_type *mfd_primary_base;

void lge_mdss_fb_init(struct msm_fb_data_type *mfd)
{
	if(mfd->index != 0)
		return;
	mfd_primary_base = mfd;

	mfd->bl_level_scaled = -1;
#if defined(CONFIG_LGE_HIGH_LUMINANCE_MODE)
	mfd->panel_info->hl_mode_on = 0;
#endif
}

/*---------------------------------------------------------------------------*/
/* LCD off & dimming                                                         */
/*---------------------------------------------------------------------------*/
#ifdef CONFIG_LGE_LCD_OFF_DIMMING
static bool fb_blank_called;
static inline bool is_blank_called(void)
{
	return fb_blank_called;
}

static inline bool is_factory_cable(void)
{
	unsigned int cable_info;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	struct lge_power *lge_cd_lpc;
	union lge_power_propval lge_val = {0,};
	int rc;
	unsigned int *p_cable_type = NULL;
	unsigned int cable_smem_size = 0;

	lge_cd_lpc = lge_power_get_by_name("lge_cable_detect");
	if (!lge_cd_lpc) {
		pr_err("lge_cd_lpc is not yet ready\n");
		p_cable_type = smem_get_entry(SMEM_ID_VENDOR1,
					&cable_smem_size, 0, 0);
		if (p_cable_type)
			cable_info = *p_cable_type;
		else
			return  false;

		pr_err("cable %d\n", cable_info);
#if !defined(CONFIG_LGE_PM_EMBEDDED_BATTERY)
		if (cable_info == LT_CABLE_56K ||
			cable_info == LT_CABLE_130K ||
			cable_info == LT_CABLE_910K)
#else
		if (cable_info == LT_CABLE_130K ||
			cable_info == LT_CABLE_910K)
#endif
			return true;
	} else {
		rc = lge_cd_lpc->get_property(lge_cd_lpc,
				LGE_POWER_PROP_CABLE_TYPE, &lge_val);
		cable_info = lge_val.intval;

#if !defined(CONFIG_LGE_PM_EMBEDDED_BATTERY)
		if (cable_info == CABLE_ADC_56K ||
			cable_info == CABLE_ADC_130K ||
			cable_info == CABLE_ADC_910K) {
#else
		if (cable_info == CABLE_ADC_130K ||
			cable_info == CABLE_ADC_910K) {
#endif

				pr_err("lge_cd_lpc is ready, cable_info = %d\n", cable_info);
				return true;
		}
	}
#elif defined (CONFIG_LGE_PM_CABLE_DETECTION)
	cable_info = lge_pm_get_cable_type();

#if !defined(CONFIG_LGE_PM_EMBEDDED_BATTERY)
	if (cable_info == CABLE_56K ||
		cable_info == CABLE_130K ||
		cable_info == CABLE_910K)
#else
	if (cable_info == CABLE_130K ||
		cable_info == CABLE_910K)
#endif
		return true;
	else
#else
	cable_info = 0;
#endif
		return false;
}

void lge_set_blank_called(void)
{
	fb_blank_called = true;
}
#else
static inline bool is_blank_called(void)
{
	return true;
}

static inline bool is_factory_cable(void)
{
	return false;
}
#endif

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_BATTERY_ID_CHECKER
static bool batt_present = false;
bool lge_battery_present(void){
	struct lge_power *lge_batt_id_lpc;
	union lge_power_propval	lge_val = {0,};
	uint *smem_batt = 0;
	uint _smem_batt_id = 0;
	int rc;
	if (batt_present == true){
		return true;
		}
	lge_batt_id_lpc = lge_power_get_by_name("lge_batt_id");
	if (lge_batt_id_lpc) {
		rc = lge_batt_id_lpc->get_property(lge_batt_id_lpc,
				LGE_POWER_PROP_PRESENT, &lge_val);
		batt_present = lge_val.intval;
	}else{
		pr_err("Failed to get batt presnet property\n");
		smem_batt = (uint *)smem_alloc(SMEM_BATT_INFO,
				sizeof(smem_batt), 0, SMEM_ANY_HOST_FLAG);
		if (smem_batt == NULL) {
			pr_err("smem_alloc returns NULL\n");
			batt_present  = false;
		} else {
			_smem_batt_id = *smem_batt;
			pr_err("Battery was read in sbl is = %d\n",
					_smem_batt_id);
			if (_smem_batt_id == BATT_NOT_PRESENT) {
				pr_err("Set batt_id as DEFAULT\n");
				batt_present = false;
			}else{
				batt_present = true;
			}
		}
	}
	return batt_present;
}
#endif

/*---------------------------------------------------------------------------*/
/* BIST Masking                                                              */
/*---------------------------------------------------------------------------*/
#if defined(CONFIG_LGE_DISPLAY_BIST_MODE)
static int lge_mdss_panel_factory_bist_ctrl(struct msm_fb_data_type *mfd, bool enable)
{
	struct mdss_panel_data *pdata = NULL;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (mfd == NULL || mfd->pdev == NULL) {
		pr_err("Invalid pdev state\n");
		return -EINVAL;
	}

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("Invalid panel data!\n");
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);
	if (!ctrl) {
		pr_err("Invalid ctrl data!\n");
		return -EINVAL;
	}

	mutex_lock(&ctrl->bist_lock);
	if (enable) {
		if (lge_mdss_dsi_bist_ctrl(ctrl, true) < 0) {
			pr_warn("fail to bist control\n");
		}
	} else {
		if (ctrl->bist_on > 0) {
			lge_mdss_dsi_bist_release(ctrl);
		}
	}
	mutex_unlock(&ctrl->bist_lock);

	return 0;
}
#endif

/*---------------------------------------------------------------------------*/
/* Brightness - Backlight mapping (main)                                     */
/*---------------------------------------------------------------------------*/
int lge_br_to_bl (struct msm_fb_data_type *mfd, int br_lvl)
{
	/* TODO: change default value more reasonablly */
	int bl_lvl = 100;
	enum lge_bl_map_type blmaptype;
	struct mdss_panel_info *pinfo;


	if(mfd->index != 0) {
		pr_err("[Ambient] fb%d is not for ambient display\n", mfd->index);
		return bl_lvl ;
	}
	pinfo = mfd->panel_info;

	if (!pinfo) {
		pr_err("no panel connected!\n");
		return -EINVAL;
	}
	/* modify brightness level */
	if (lge_get_bootreason_with_lcd_dimming() && !is_blank_called()) {
		br_lvl = 1;
		pr_info("lcd dimming mode. set value = %d\n", br_lvl);
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_BATTERY_ID_CHECKER
	} else if (is_factory_cable()
#if !defined(CONFIG_LGE_PM_EMBEDDED_BATTERY)
					&& !lge_battery_present()
#endif
#elif defined (CONFIG_LGE_PM_BATTERY_ID_CHECKER)
	} else if (is_factory_cable() && !lge_battery_check()
#else
	} else if (is_factory_cable()
#endif
			&& !is_blank_called()) {
		br_lvl = 1;
		pr_info("Detect factory cable. set value = %d\n", br_lvl);
	}

	/* map brightness level to device backlight level */
#if defined(CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT)
	blmaptype = pinfo->ve_mode_on ? LGE_BLVE : LGE_BL;
#else
#if defined(CONFIG_LGE_HIGH_LUMINANCE_MODE)
	blmaptype = pinfo->hl_mode_on ? LGE_BLHL : LGE_BL;
#else
	blmaptype = LGE_BL;
#endif /* CONFIG_LGE_HIGH_LUMINANCE_MODE */
#endif /* CONFIG_LGE_DISPLAY_VIDEO_ENHANCEMENT */

#if defined(CONFIG_LGE_DISPLAY_BIST_MODE)
	if (lge_get_factory_boot()) {
		bool enable = ((br_lvl == 0) ? true : false);
		if (lge_mdss_panel_factory_bist_ctrl(mfd, enable) < 0) {
			pr_err("factory mode, fail to run BIST\n");
		}
	}
#endif
	if (pinfo->blmap[blmaptype])
		bl_lvl = pinfo->blmap[blmaptype][br_lvl];
	pr_info("br_lvl(%d) -> bl_lvl(%d) [%s]\n", br_lvl, bl_lvl,
				lge_get_blmapname(blmaptype));
	return bl_lvl;
}

/*---------------------------------------------------------------------------*/
/* Backlight control blocking                                                */
/*---------------------------------------------------------------------------*/
#if defined(CONFIG_LGE_SP_MIRRORING_CTRL_BL)
static bool lge_block_bl_update;
static int lge_bl_lvl_unset;
static bool lge_is_bl_ready;

/* must call this function within mfd->bl_lock */
int lge_is_bl_update_blocked(int bl_lvl)
{
	lge_is_bl_ready = true;
	if(lge_block_bl_update) {
		lge_bl_lvl_unset = bl_lvl;
		pr_info("do not control backlight (bl: %d)\n", lge_bl_lvl_unset);
		return true;
	}
	return false;
}

void lge_set_bl_update_blocked(bool enable)
{
	lge_block_bl_update = enable;
}

static ssize_t mdss_fb_get_bl_off_and_block(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;

	mutex_lock(&mfd->bl_lock);
	ret = snprintf(buf, PAGE_SIZE, "sp link backlight status : %d\n",
						lge_block_bl_update);
	mutex_unlock(&mfd->bl_lock);
	return ret;
}

static ssize_t mdss_fb_set_bl_off_and_block(struct device *dev,
		struct device_attribute *attr,const char *buf,  size_t count)
{
	struct fb_info *fbi;
	struct msm_fb_data_type *mfd;
	int enable;

	if (!count || !lge_is_bl_ready ||!dev) {
		pr_warn("invalid value : %d, %d || NULL check\n",
					(int) count, lge_is_bl_ready);
		return -EINVAL;
	}

	fbi = dev_get_drvdata(dev);
	mfd = fbi->par;

	enable = simple_strtoul(buf, NULL, 10);
	if (enable) {
		pr_info("status : %d, brightness : 0 \n", lge_block_bl_update);
		mutex_lock(&mfd->bl_lock);
		lge_block_bl_update = false;
		lge_bl_lvl_unset = mfd->bl_level;
		mdss_fb_set_backlight(mfd, 0);
		lge_block_bl_update = true;
		mutex_unlock(&mfd->bl_lock);
	} else {
		pr_info("status : %d, brightness : %d \n",
					lge_block_bl_update, lge_bl_lvl_unset);
		mutex_lock(&mfd->bl_lock);
		lge_block_bl_update = false;
		mdss_fb_set_backlight(mfd, lge_bl_lvl_unset);
		mutex_unlock(&mfd->bl_lock);
	}
	return count;
}

static DEVICE_ATTR(sp_link_backlight_off, S_IRUGO | S_IWUSR,
	mdss_fb_get_bl_off_and_block, mdss_fb_set_bl_off_and_block);
#endif
/*---------------------------------------------------------------------------*/
/* High luminance function                                                   */
/*---------------------------------------------------------------------------*/
#if defined(CONFIG_LGE_HIGH_LUMINANCE_MODE)
static ssize_t hl_mode_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *pinfo;
	int ret;

	pinfo = mfd->panel_info;

	if (!pinfo) {
		pr_err("[hl_mode] no panel connected!\n");
		return -EINVAL;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", pinfo->hl_mode_on);
	return ret;
}

static ssize_t hl_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *pinfo;

	pinfo = mfd->panel_info;

	if (!pinfo) {
		pr_err("[hl_mode] no panel connected!\n");
		return len;
	}

	pinfo->hl_mode_on = simple_strtoul(buf, NULL, 10);

	if(pinfo->hl_mode_on == 1)
		pr_info("[hl_mode] hl_mode on\n");
	else
		pr_info("[hl_mode] hl_mode off\n");

	return len;
}

static DEVICE_ATTR(hl_mode, S_IRUGO | S_IWUSR | S_IWGRP, hl_mode_show, hl_mode_store);
#endif

static ssize_t mdss_fb_get_panel_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int panel_type = lge_get_panel();

	if (panel_type == LGD_SIW_LG43402_1440_2880_ONCELL_CMD_PANEL)
		ret = snprintf(buf, PAGE_SIZE, "LGD/SiW - SW43402 cmd panel\n");
	else if (panel_type == LGD_SIW_LG43401_NOTOUCH_CMD_PANEL)
		ret = snprintf(buf, PAGE_SIZE, "LGD/SiW - SW43401 cmd panel w/o touch\n");
	else if (panel_type == LGD_SIC_LG49407_1440_2720_INCELL_CMD_PANEL)
		ret = snprintf(buf, PAGE_SIZE, "LGD - SW49407 1440 x 2720 cmd\n");
	else if (panel_type == LGD_SIC_LG49408_1440_2880_INCELL_CMD_PANEL)
		ret = snprintf(buf, PAGE_SIZE, "LGD - SW49408 1440 x 2880 cmd\n");
	else
		ret = snprintf(buf, PAGE_SIZE, "Unknown LCD TYPE\n");
	return ret;

}

#ifdef CONFIG_LGE_LCD_MFTS_MODE
static ssize_t mdss_get_mfts_auto_touch(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata;
	int ret;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("[MFTS] no panel connected!\n");
		return -EINVAL;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", !pdata->panel_info.power_ctrl);

	return ret;
}

static ssize_t mdss_set_mfts_auto_touch(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata;
	int value;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("[MFTS] no panel connected!\n");
		return len;
	}

	if (sscanf(buf, "%d", &value) != 1) {
		pr_err("[MFTS] sccanf buf error!\n");
		return len;
	}

	pdata->panel_info.power_ctrl = !value;
	if (pdata->next)
		pdata->next->panel_info.power_ctrl = !value;

	pr_info("[MFTS] power_ctrl = %d\n", pdata->panel_info.power_ctrl);
	return len;
}
#endif

static DEVICE_ATTR(panel_type, S_IRUGO, mdss_fb_get_panel_type, NULL);

#ifdef CONFIG_LGE_LCD_MFTS_MODE
static DEVICE_ATTR(mfts_auto_touch_test_mode, S_IWUSR|S_IRUGO, mdss_get_mfts_auto_touch , mdss_set_mfts_auto_touch);
#endif

/*---------------------------------------------------------------------------*/
/* Ambient Display Brightness                                                */
/*---------------------------------------------------------------------------*/
#if defined(CONFIG_LGE_DISPLAY_AMBIENT_SUPPORTED)
static bool lge_ambient_brightness_registered = false;

int lge_br_to_bl_ex(struct msm_fb_data_type *mfd, int br_lvl)
{
	enum lge_bl_map_type blmaptype;
	struct mdss_panel_info *pinfo;
	int bl_lvl = 100; /* default */


	if (mfd->index != 0) {
		pr_err("[Ambient] fb%d is not for ambient display\n", mfd->index);
		return bl_lvl ;
	}
	pinfo = mfd->panel_info;

	if (!pinfo) {
		pr_err("no panel connected!\n");
		return -EINVAL;
	}

	/* modify brightness level */
	if (lge_get_bootreason_with_lcd_dimming() && !is_blank_called()) {
		br_lvl = 1;
		pr_info("lcd dimming mode. set value = %d\n", br_lvl);
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_BATTERY_ID_CHECKER
	} else if (is_factory_cable()
#if !defined(CONFIG_LGE_PM_EMBEDDED_BATTERY)
					&& !lge_battery_present()
#endif
#elif defined (CONFIG_LGE_PM_BATTERY_ID_CHECKER)
	} else if (is_factory_cable() && !lge_battery_check()
#else
	} else if (is_factory_cable()
#endif
			&& !is_blank_called()) {
		br_lvl = 1;
		pr_info("Detect factory cable. set value = %d\n", br_lvl);
	}

	/* map brightness level to device backlight level */
	blmaptype = LGE_BLEX;

	if (pinfo->blmap[blmaptype])
		bl_lvl = pinfo->blmap[blmaptype][br_lvl];
	pr_info("br_lvl(%d) -> bl_lvl(%d) [%s]\n", br_lvl, bl_lvl,
				lge_get_blmapname(blmaptype));
	return bl_lvl;
}

static void mdss_fb_set_bl_brightness_ambient(struct led_classdev *led_cdev,
				      enum led_brightness value)
{
	struct msm_fb_data_type *mfd = dev_get_drvdata(led_cdev->dev->parent);
	bool isdoze;
	int bl_lvl, panel_state;

	panel_state = mfd->panel_info->panel_power_state;

	if (value > mfd->panel_info->brightness_max)
		value = mfd->panel_info->brightness_max;

	MDSS_BRIGHT_TO_BL_EX(bl_lvl, value, mfd->panel_info->bl_max,
				mfd->panel_info->brightness_max);
	if (!bl_lvl && value) {
		bl_lvl = 1;
	}

	isdoze = (((panel_state == MDSS_PANEL_POWER_LP1) ||
			(panel_state == MDSS_PANEL_POWER_LP2)) ? true : false);

	if (!IS_CALIB_MODE_BL(mfd) &&
			(!mfd->ext_bl_ctrl || !value || !mfd->bl_level)) {
		if (isdoze) {
			mutex_lock(&mfd->bl_lock);
			mfd->unset_bl_level_ex = U32_MAX;
			mfd->allow_bl_update = true;
			mdss_fb_set_backlight(mfd, bl_lvl);
			mfd->allow_bl_update = false;
			mutex_unlock(&mfd->bl_lock);
		} else {
			mfd->unset_bl_level_ex = (value ? bl_lvl : U32_MAX);
		}
	}
}

void mdss_fb_update_backlight_ex(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	u32 temp;

	if (mfd->unset_bl_level_ex == U32_MAX)
		return;

	mutex_lock(&mfd->bl_lock);
	if (!mfd->allow_bl_update) {
		pdata = dev_get_platdata(&mfd->pdev->dev);
		if ((pdata) && (pdata->set_backlight)) {
			mfd->bl_level = mfd->unset_bl_level_ex;
			temp = mfd->bl_level;
			pdata->set_backlight(pdata, temp);
			mfd->allow_bl_update = true;
		}
	}
	mutex_unlock(&mfd->bl_lock);
}

static struct led_classdev ambient_brightness_cdev = {
	.name           = "lcd-backlight-ex",
	.brightness     = MDSS_MAX_BL_BRIGHTNESS / 2,
	.usr_brightness_req = MDSS_MAX_BL_BRIGHTNESS,
	.brightness_set = mdss_fb_set_bl_brightness_ambient,
	.max_brightness = MDSS_MAX_BL_BRIGHTNESS,
};

void lge_ambient_brightness_register(struct msm_fb_data_type *mfd)
{
	if (lge_ambient_brightness_registered)
		return;

	ambient_brightness_cdev.brightness = 0;
	ambient_brightness_cdev.max_brightness = mfd->panel_info->brightness_max;
	if (led_classdev_register(&mfd->pdev->dev, &ambient_brightness_cdev))
		pr_err("[Ambient] brightness cdev register failed\n");
	else
		lge_ambient_brightness_registered = true;
}

void lge_ambient_brightness_unregister(void)
{
	if (lge_ambient_brightness_registered) {
		lge_ambient_brightness_registered = false;
		led_classdev_unregister(&ambient_brightness_cdev);
	}
}

static ssize_t ambient_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *pinfo;
	int ret;

	pinfo = mfd->panel_info;

	if (!pinfo) {
		pr_err("no panel connected!\n");
		return -EINVAL;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", pinfo->panel_power_state);
	return ret;
}

static DEVICE_ATTR(cur_panel_mode, S_IRUGO, ambient_state_show, NULL);
#endif

#if defined(CONFIG_LGE_DISPLAY_DYNAMIC_RESOLUTION_SWITCH)
static ssize_t freeze_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata = NULL;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input = 0;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("Invalid panel data!\n");
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);
	if (!ctrl) {
		pr_err("Invalid ctrl data!\n");
		return -EINVAL;
	}
	sscanf(buf, "%d", &input);

	if (input > 0) {
		ctrl->requested_resolution_switch = true;
#if defined(CONFIG_LGE_DISPLAY_BIST_MODE)
		ctrl->keep_bist_on = false;
#endif
	} else {
		ctrl->requested_resolution_switch = false;
		ctrl->keep_bist_on = false;
		mutex_lock(&ctrl->bist_lock);
		if (ctrl->bist_on > 0) {
			pr_info("freeze request timeout! -> bist release\n");
			lge_mdss_dsi_bist_release(ctrl);
		}
		mutex_unlock(&ctrl->bist_lock);
	}
	pr_info("freezing in userspace\n");

	return ret;
}
static DEVICE_ATTR(freeze_state, S_IWUSR|S_IRUGO, NULL, freeze_set);
#endif

/*---------------------------------------------------------------------------*/
/* Dynamic Resolution Switch : uevent                                        */
/*---------------------------------------------------------------------------*/
#if defined(CONFIG_LGE_DISPLAY_DYNAMIC_RESOLUTION_SWITCH)
void mdss_fb_drs_notify(struct kobject *kobj, int state)
{
	char name_buf[20];
	char *envp[2];

	if (!kobj) {
		pr_err("fail to send uevent\n");
		return;
	}

	if (state > 0) {
		snprintf(name_buf, sizeof(name_buf),
			"RESOLUTION_SWITCH=%d", state);
		envp[0] = name_buf;
		envp[1] = NULL;

		kobject_uevent_env(kobj, KOBJ_CHANGE, envp);
	}

	pr_info("sending uevent for switching info\n");
}
#endif

/*---------------------------------------------------------------------------*/
/* Register lge sysfs attributes                                             */
/*---------------------------------------------------------------------------*/
/* TODO: implement registering method for attributes in other lge files */
static struct attribute *lge_mdss_fb_attrs[] = {
	&dev_attr_panel_type.attr,
#if defined(CONFIG_LGE_SP_MIRRORING_CTRL_BL)
	&dev_attr_sp_link_backlight_off.attr,
#endif
#if defined(CONFIG_LGE_HIGH_LUMINANCE_MODE)
	&dev_attr_hl_mode.attr,
#endif
#if defined(CONFIG_LGE_LCD_MFTS_MODE)
	&dev_attr_mfts_auto_touch_test_mode.attr,
#endif
#if defined(CONFIG_LGE_DISPLAY_AMBIENT_SUPPORTED)
	&dev_attr_cur_panel_mode.attr,
#endif
#if defined(CONFIG_LGE_DISPLAY_DYNAMIC_RESOLUTION_SWITCH)
	&dev_attr_freeze_state.attr,
#endif
	NULL,
};

static struct attribute_group lge_mdss_fb_attr_group = {
	.attrs = lge_mdss_fb_attrs,
};

int lge_mdss_fb_create_sysfs(struct msm_fb_data_type *mfd)
{
	int rc;

	if (mfd->index != 0)
		return false;

	rc = sysfs_create_group(&mfd->fbi->dev->kobj, &lge_mdss_fb_attr_group);
	if (rc)
		pr_err("lge sysfs group creation failed, rc=%d\n", rc);
	return rc;
}

void lge_mdss_fb_remove_sysfs(struct msm_fb_data_type *mfd)
{
	if (mfd->index != 0)
		return;

	sysfs_remove_group(&mfd->fbi->dev->kobj, &lge_mdss_fb_attr_group);
}
