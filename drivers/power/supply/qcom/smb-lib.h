/* Copyright (c) 2016-2017 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SMB2_CHARGER_H
#define __SMB2_CHARGER_H
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/consumer.h>
#include <linux/extcon.h>
#include "storm-watch.h"

#ifdef CONFIG_LGE_PM
#include <linux/power_supply.h>
#endif
#ifdef CONFIG_LGE_PM_USE_FAKE_BATT_TEMP_CTRL
#include <linux/of_gpio.h>
#endif

enum print_reason {
	PR_INTERRUPT	= BIT(0),
	PR_REGISTER	= BIT(1),
	PR_MISC		= BIT(2),
	PR_PARALLEL	= BIT(3),
	PR_OTG		= BIT(4),
#ifdef CONFIG_LGE_PM_DEBUG
	PR_LGE = BIT(5),
#endif
};

#define DEFAULT_VOTER			"DEFAULT_VOTER"
#define USER_VOTER			"USER_VOTER"
#define PD_VOTER			"PD_VOTER"
#define DCP_VOTER			"DCP_VOTER"
#define QC_VOTER			"QC_VOTER"
#define PL_USBIN_USBIN_VOTER		"PL_USBIN_USBIN_VOTER"
#define USB_PSY_VOTER			"USB_PSY_VOTER"
#define PL_TAPER_WORK_RUNNING_VOTER	"PL_TAPER_WORK_RUNNING_VOTER"
#define PL_QNOVO_VOTER			"PL_QNOVO_VOTER"
#define USBIN_V_VOTER			"USBIN_V_VOTER"
#define CHG_STATE_VOTER			"CHG_STATE_VOTER"
#define TYPEC_SRC_VOTER			"TYPEC_SRC_VOTER"
#define TAPER_END_VOTER			"TAPER_END_VOTER"
#define THERMAL_DAEMON_VOTER		"THERMAL_DAEMON_VOTER"
#define CC_DETACHED_VOTER		"CC_DETACHED_VOTER"
#define HVDCP_TIMEOUT_VOTER		"HVDCP_TIMEOUT_VOTER"
#define PD_DISALLOWED_INDIRECT_VOTER	"PD_DISALLOWED_INDIRECT_VOTER"
#define PD_HARD_RESET_VOTER		"PD_HARD_RESET_VOTER"
#define VBUS_CC_SHORT_VOTER		"VBUS_CC_SHORT_VOTER"
#ifdef CONFIG_LGE_PM
#define LEGACY_CABLE_VOTER		"LEGACY_CABLE_VOTER"
#define ABNORMAL_GENDER_VOTER	"ABNORMAL_GENDER_VOTER"
#define RETRY_LEGACY_VOTER			"RETRY_LEGACY_VOTER"
#define APSD_RERUN_VOTER			"APSD_RERUN_VOTER"
#define FAST_HVDCP_DETECTION_VOTER	"FAST_HVDCP_DETECTION_VOTER"
#define NO_BATTERY_VOTER	"NO_BATTERY_VOTER"
#define HW_ICL_VOTER		"HW_ICL_VOTER"
#endif
#define PD_INACTIVE_VOTER		"PD_INACTIVE_VOTER"
#define BOOST_BACK_VOTER		"BOOST_BACK_VOTER"
#ifdef CONFIG_LGE_PM_WEAK_BATT_PACK
#define WEAK_BATT_VOTER		"WEAK_BATT_VOTER"
#endif
#ifdef CONFIG_LGE_PM_DEBUG
#define CHARGING_INFORM_NORMAL_TIME     60000
#endif
#ifdef CONFIG_LGE_PM_SUPPORT_LG_POWER_CLASS
#define LGE_POWER_CLASS_FCC_VOTER "LGE_POWER_CLASS_FCC_VOTER"
#define LGE_POWER_CLASS_FV_VOTER "LGE_POWER_CLASS_FV_VOTER"
#define LGCC_EN_VOTER	"LGCC_EN_VOTER"
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_STORE_MODE
#define LGSM_EN_VOTER	"LGSM_EN_VOTER"
#endif
#ifdef CONFIG_LGE_PM_STEP_CHARGING
#define LGE_STEP_CHARGING_VOTER "LGE_STEP_CHARGING_VOTER"
#endif
#ifdef CONFIG_LGE_PM_CYCLE_BASED_CHG_VOLTAGE
#define LGE_CBC_VOTER "LGE_CBC_VOTER"
#endif
#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
#define MOISTURE_VOTER			"MOISTURE_VOTER"
#endif
#define USBIN_USBIN_BOOST_VOTER		"USBIN_USBIN_BOOST_VOTER"
#define HVDCP_INDIRECT_VOTER		"HVDCP_INDIRECT_VOTER"
#define MICRO_USB_VOTER			"MICRO_USB_VOTER"
#define DEBUG_BOARD_VOTER		"DEBUG_BOARD_VOTER"
#define PD_SUSPEND_SUPPORTED_VOTER	"PD_SUSPEND_SUPPORTED_VOTER"
#define PL_DELAY_VOTER			"PL_DELAY_VOTER"
#define CTM_VOTER			"CTM_VOTER"
#define SW_QC3_VOTER			"SW_QC3_VOTER"
#define AICL_RERUN_VOTER		"AICL_RERUN_VOTER"
#define LEGACY_UNKNOWN_VOTER		"LEGACY_UNKNOWN_VOTER"
#define CC2_WA_VOTER			"CC2_WA_VOTER"
#define QNOVO_VOTER			"QNOVO_VOTER"
#define BATT_PROFILE_VOTER		"BATT_PROFILE_VOTER"
#ifdef CONFIG_LGE_PM_CC_PROTECT
#define CC_PROTECT_VOTER			"CC_PROTECT_VOTER"
#endif

#define OTG_DELAY_VOTER			"OTG_DELAY_VOTER"
#define USBIN_I_VOTER			"USBIN_I_VOTER"
#ifdef CONFIG_LGE_PM
#define PSEUDO_USB_TYPE	1
#define PSEUDO_HVDCP	2

#define SLOW_CHARGING_THRESHOLD   100000
#define SLOW_CHARGING_DETECT_MS   15000
#endif

#define VCONN_MAX_ATTEMPTS	3
#define OTG_MAX_ATTEMPTS	3

enum smb_mode {
	PARALLEL_MASTER = 0,
	PARALLEL_SLAVE,
	NUM_MODES,
};

enum {
	QC_CHARGER_DETECTION_WA_BIT	= BIT(0),
	BOOST_BACK_WA			= BIT(1),
	TYPEC_CC2_REMOVAL_WA_BIT	= BIT(2),
	QC_AUTH_INTERRUPT_WA_BIT	= BIT(3),
	OTG_WA				= BIT(4),
};

enum smb_irq_index {
	CHG_ERROR_IRQ = 0,
	CHG_STATE_CHANGE_IRQ,
	STEP_CHG_STATE_CHANGE_IRQ,
	STEP_CHG_SOC_UPDATE_FAIL_IRQ,
	STEP_CHG_SOC_UPDATE_REQ_IRQ,
	OTG_FAIL_IRQ,
	OTG_OVERCURRENT_IRQ,
	OTG_OC_DIS_SW_STS_IRQ,
	TESTMODE_CHANGE_DET_IRQ,
	BATT_TEMP_IRQ,
	BATT_OCP_IRQ,
	BATT_OV_IRQ,
	BATT_LOW_IRQ,
	BATT_THERM_ID_MISS_IRQ,
	BATT_TERM_MISS_IRQ,
#ifdef CONFIG_IDTP9223_CHARGER
	BATT_QIPMA_ON_IRQ,
#endif
	USBIN_COLLAPSE_IRQ,
	USBIN_LT_3P6V_IRQ,
	USBIN_UV_IRQ,
	USBIN_OV_IRQ,
	USBIN_PLUGIN_IRQ,
	USBIN_SRC_CHANGE_IRQ,
	USBIN_ICL_CHANGE_IRQ,
	TYPE_C_CHANGE_IRQ,
	DCIN_COLLAPSE_IRQ,
	DCIN_LT_3P6V_IRQ,
	DCIN_UV_IRQ,
	DCIN_OV_IRQ,
	DCIN_PLUGIN_IRQ,
	DIV2_EN_DG_IRQ,
	DCIN_ICL_CHANGE_IRQ,
	WDOG_SNARL_IRQ,
	WDOG_BARK_IRQ,
	AICL_FAIL_IRQ,
	AICL_DONE_IRQ,
	HIGH_DUTY_CYCLE_IRQ,
	INPUT_CURRENT_LIMIT_IRQ,
	TEMPERATURE_CHANGE_IRQ,
	SWITCH_POWER_OK_IRQ,
	SMB_IRQ_MAX,
};

#ifdef CONFIG_LGE_PM
enum retry_legacy_index {
	WAIT_PD_HARD_RESET = 0,
	RETRY_APSD_RERUN,
	RETRY_LEGACY_CABLE,
};
#endif

#ifdef CONFIG_LGE_PM_INOV_GEN3_SYSFS_SUPPORT
enum inov_temp_index {
	CHARGER_TEMP = 0,
	CHARGER_TEMP_HOT,
	SKIN_TEMP,
	SKIN_TEMP_HOT,
	INOV_TEMP_MAX,
};
#endif

struct smb_irq_info {
	const char			*name;
	const irq_handler_t		handler;
	const bool			wake;
	const struct storm_watch	storm_data;
	struct smb_irq_data		*irq_data;
	int				irq;
};

static const unsigned int smblib_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

struct smb_regulator {
	struct regulator_dev	*rdev;
	struct regulator_desc	rdesc;
};

struct smb_irq_data {
	void			*parent_data;
	const char		*name;
	struct storm_watch	storm_data;
};

struct smb_chg_param {
	const char	*name;
	u16		reg;
	int		min_u;
	int		max_u;
	int		step_u;
	int		(*get_proc)(struct smb_chg_param *param,
				    u8 val_raw);
	int		(*set_proc)(struct smb_chg_param *param,
				    int val_u,
				    u8 *val_raw);
};

struct smb_chg_freq {
	unsigned int		freq_5V;
	unsigned int		freq_6V_8V;
	unsigned int		freq_9V;
	unsigned int		freq_12V;
	unsigned int		freq_removal;
	unsigned int		freq_below_otg_threshold;
	unsigned int		freq_above_otg_threshold;
};

struct smb_params {
	struct smb_chg_param	fcc;
	struct smb_chg_param	fv;
	struct smb_chg_param	usb_icl;
	struct smb_chg_param	icl_stat;
	struct smb_chg_param	otg_cl;
	struct smb_chg_param	dc_icl;
	struct smb_chg_param	dc_icl_pt_lv;
	struct smb_chg_param	dc_icl_pt_hv;
	struct smb_chg_param	dc_icl_div2_lv;
	struct smb_chg_param	dc_icl_div2_mid_lv;
	struct smb_chg_param	dc_icl_div2_mid_hv;
	struct smb_chg_param	dc_icl_div2_hv;
	struct smb_chg_param	jeita_cc_comp;
	struct smb_chg_param	step_soc_threshold[4];
	struct smb_chg_param	step_soc;
	struct smb_chg_param	step_cc_delta[5];
	struct smb_chg_param	freq_buck;
	struct smb_chg_param	freq_boost;
};

struct parallel_params {
	struct power_supply	*psy;
};

struct smb_iio {
	struct iio_channel	*temp_chan;
	struct iio_channel	*temp_max_chan;
	struct iio_channel	*usbin_i_chan;
	struct iio_channel	*usbin_v_chan;
	struct iio_channel	*batt_i_chan;
	struct iio_channel	*connector_temp_chan;
	struct iio_channel	*connector_temp_thr1_chan;
	struct iio_channel	*connector_temp_thr2_chan;
	struct iio_channel	*connector_temp_thr3_chan;
#ifdef CONFIG_LGE_PM_INOV_GEN3_SYSFS_SUPPORT
	struct iio_channel      *temp_hot_max_chan;
	struct iio_channel      *skin_temp_chan;
	struct iio_channel      *skin_temp_max_chan;
	struct iio_channel      *skin_temp_hot_max_chan;
#endif
};

struct reg_info {
	u16		reg;
	u8		mask;
	u8		val;
	u8		bak;
	const char	*desc;
};

struct smb_charger {
	struct device		*dev;
	char			*name;
	struct regmap		*regmap;
	struct smb_irq_info	*irq_info;
	struct smb_params	param;
	struct smb_iio		iio;
	int			*debug_mask;
	enum smb_mode		mode;
	struct smb_chg_freq	chg_freq;
	int			smb_version;
	int			otg_delay_ms;

	/* locks */
	struct mutex		lock;
	struct mutex		write_lock;
	struct mutex		ps_change_lock;
	struct mutex		otg_oc_lock;
	struct mutex		vconn_oc_lock;

	/* power supplies */
	struct power_supply		*batt_psy;
	struct power_supply		*usb_psy;
	struct power_supply		*dc_psy;
	struct power_supply		*bms_psy;
	struct power_supply_desc	usb_psy_desc;
	struct power_supply		*usb_main_psy;
	struct power_supply		*usb_port_psy;
#ifdef CONFIG_LGE_PM
	struct power_supply		*ttf_psy;
#endif
#ifdef CONFIG_LGE_PM_BATT_MANAGER
	struct power_supply		*bm_psy;
#endif
	enum power_supply_type		real_charger_type;

	/* notifiers */
	struct notifier_block	nb;

	/* parallel charging */
	struct parallel_params	pl;

	/* regulators */
	struct smb_regulator	*vbus_vreg;
	struct smb_regulator	*vconn_vreg;
	struct regulator	*dpdm_reg;

	/* votables */
	struct votable		*dc_suspend_votable;
	struct votable		*fcc_votable;
	struct votable		*fv_votable;
	struct votable		*usb_icl_votable;
	struct votable		*dc_icl_votable;
	struct votable		*pd_disallowed_votable_indirect;
	struct votable		*pd_allowed_votable;
	struct votable		*awake_votable;
	struct votable		*pl_disable_votable;
	struct votable		*chg_disable_votable;
	struct votable		*pl_enable_votable_indirect;
	struct votable		*hvdcp_disable_votable_indirect;
	struct votable		*hvdcp_enable_votable;
	struct votable		*apsd_disable_votable;
	struct votable		*hvdcp_hw_inov_dis_votable;
	struct votable		*usb_irq_enable_votable;
	struct votable		*typec_irq_disable_votable;
#ifdef CONFIG_LGE_PM
	struct votable		*pseudo_usb_type_votable;
#endif

	/* work */
	struct work_struct	bms_update_work;
#ifdef CONFIG_LGE_PM_STEP_CHARGING
	struct work_struct	batt_update_work;
	struct delayed_work step_charging_check_work;
#endif
	struct work_struct	rdstd_cc2_detach_work;
	struct delayed_work	hvdcp_detect_work;
	struct delayed_work	ps_change_timeout_work;
	struct delayed_work	step_soc_req_work;
	struct delayed_work	clear_hdc_work;
	struct work_struct	otg_oc_work;
	struct work_struct	vconn_oc_work;
	struct delayed_work	otg_ss_done_work;
	struct delayed_work	icl_change_work;
	struct delayed_work	pl_enable_work;
	struct work_struct	legacy_detection_work;
	struct delayed_work	uusb_otg_work;

	/* LGE feature */
#ifdef CONFIG_LGE_PM_DEBUG
	struct delayed_work charging_inform_work;
#endif
#ifdef CONFIG_LGE_PM_SUPPORT_LG_POWER_CLASS
	struct notifier_block   lge_nb;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	struct work_struct	cable_detect_work;
	struct lge_power		*lge_cd_lpc;
	int			is_factory_cable;
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
	struct work_struct	lge_cc_work;
	struct lge_power		*lge_cc_lpc;
	struct mutex			lge_cc_ibat_lock;
	int btm_state;
	int pseudo_chg_ui;
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PARALLEL_CONTROLLER
	struct lge_power		*lge_pl_lpc;
	struct work_struct		lge_pl_work;
	int			lge_pl_status;
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PSEUDO_BATTERY
	struct work_struct		lge_pseudo_work;
	struct lge_power		*lge_pseudo_lpc;
	bool			pseudo_batt;
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
	struct work_struct		lge_vzw_work;
	struct delayed_work		vzw_aicl_done_work;
	struct delayed_work		vzw_recheck_work;
	struct lge_power		*lge_vzw_lpc;
	int			vzw_chg;
	int			vzw_icl_change;
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_STORE_MODE
	struct work_struct      lge_sm_work;
	struct lge_power        *lge_sm_lpc;
#endif
#endif
#ifdef CONFIG_IDTP9223_CHARGER
	struct power_supply	*wlc_psy;
	struct work_struct	idtp9223_work;
	struct delayed_work	dc_lcd_current_work;
	int			dc_fastchg;
#endif
#ifdef CONFIG_LGE_PM_WEAK_BATT_PACK
	int                 batt_pack_verify_cnt;
	struct delayed_work batt_pack_check_work;
#endif
#ifdef CONFIG_LGE_PM
	int			safety_timer_en;
	int			maximum_icl_ua;
	uint32_t		smb_bat_en;
	bool			usbin_ov_sts;
	bool			no_batt_boot;
	bool			fake_hvdcp_mode;
	bool			lcd_status;
	bool			is_normal_bootmode;
	bool			checking_pd_active;
	bool			is_abnormal_gendor;
	bool			is_hvdcp_timeout;
	bool			disable_inov_for_hvdcp;
	enum retry_legacy_index		retry_legacy_detection;
	enum power_supply_type		pseudo_usb_type;
	struct delayed_work 	recovery_boost_back_work;
	struct delayed_work 	abnormal_gender_detect_work;
	struct delayed_work 	slow_charging_detect_work;
#endif
#ifdef CONFIG_LGE_PM_USE_FAKE_BATT_TEMP_CTRL
	int			fake_batt_temp_ctrl;
#endif
#ifdef CONFIG_LGE_PM_STEP_CHARGING
	int			sc_level;
	int			sc_size;
	int			*sc_cur_delta;
	int			*sc_volt_thre;
	/* preventing from overlaping between qnovo and step charging */
	int			sc_enable;
#endif
#ifdef CONFIG_LGE_PM_CYCLE_BASED_CHG_VOLTAGE
	int			batt_life_cycle_size;
	int			*batt_life_cycle_set;
	int			*batt_life_cycle_fcc_ma;
#endif
	/* cached status */
	int			voltage_min_uv;
	int			voltage_max_uv;
	int			pd_active;
	bool			system_suspend_supported;
	int			boost_threshold_ua;
	int			system_temp_level;
	int			thermal_levels;
	int			*thermal_mitigation;
	int			dcp_icl_ua;
	int			fake_capacity;
	bool			step_chg_enabled;
	bool			is_hdc;
	bool			chg_done;
	bool			micro_usb_mode;
	bool			otg_en;
	bool			vconn_en;
	bool			suspend_input_on_debug_batt;
	int			otg_attempts;
	int			vconn_attempts;
	int			default_icl_ua;
	int			otg_cl_ua;
	bool			uusb_apsd_rerun_done;
	bool			pd_hard_reset;
	bool			typec_present;
	u8			typec_status[5];
	bool			typec_legacy_valid;
	int			fake_input_current_limited;
	bool			pr_swap_in_progress;
	int			typec_mode;

	/* workaround flag */
	u32			wa_flags;
	bool			cc2_detach_wa_active;
	bool			typec_en_dis_active;
	int			boost_current_ua;
	int			temp_speed_reading_count;

	/* extcon for VBUS / ID notification to USB for uUSB */
	struct extcon_dev	*extcon;
#ifdef CONFIG_LGE_PM
	int			pre_settled_ua;
	bool			usb_ever_removed;
	bool			first_debounce_done;
#endif
#ifdef CONFIG_LGE_PM_INOV_GEN3_SYSFS_SUPPORT
	int			inov_on_temp[INOV_TEMP_MAX];
	int			inov_off_temp[INOV_TEMP_MAX];
#endif
#ifdef CONFIG_LGE_PM_CC_PROTECT
	int			cc_protect_irq;
#endif
	/* battery profile */
	int			batt_profile_fcc_ua;
	int			batt_profile_fv_uv;

	/* qnovo */
	int			usb_icl_delta_ua;
	int			pulse_cnt;
};

int smblib_read(struct smb_charger *chg, u16 addr, u8 *val);
int smblib_masked_write(struct smb_charger *chg, u16 addr, u8 mask, u8 val);
int smblib_write(struct smb_charger *chg, u16 addr, u8 val);

int smblib_get_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int *val_u);
int smblib_get_usb_suspend(struct smb_charger *chg, int *suspend);

int smblib_enable_charging(struct smb_charger *chg, bool enable);
int smblib_set_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int val_u);
int smblib_set_usb_suspend(struct smb_charger *chg, bool suspend);
int smblib_set_dc_suspend(struct smb_charger *chg, bool suspend);

int smblib_mapping_soc_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw);
int smblib_mapping_cc_delta_to_field_value(struct smb_chg_param *param,
					   u8 val_raw);
int smblib_mapping_cc_delta_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw);
int smblib_set_chg_freq(struct smb_chg_param *param,
				int val_u, u8 *val_raw);
#ifdef CONFIG_LGE_PM
int smblib_update_icl_override(struct smb_charger *chg, bool enable);
#endif

int smblib_vbus_regulator_enable(struct regulator_dev *rdev);
int smblib_vbus_regulator_disable(struct regulator_dev *rdev);
int smblib_vbus_regulator_is_enabled(struct regulator_dev *rdev);

int smblib_vconn_regulator_enable(struct regulator_dev *rdev);
int smblib_vconn_regulator_disable(struct regulator_dev *rdev);
int smblib_vconn_regulator_is_enabled(struct regulator_dev *rdev);

irqreturn_t smblib_handle_debug(int irq, void *data);
#ifdef CONFIG_LGE_PM_DEBUG
irqreturn_t smblib_handle_lge_debug(int irq, void *data);
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
irqreturn_t smblib_handle_vzw_debug(int irq, void *data);
#endif
#ifdef CONFIG_LGE_PM
irqreturn_t smblib_handle_usbin_ov(int irq, void *data);
irqreturn_t smblib_handle_aicl_fail(int irq, void *data);
#endif
irqreturn_t smblib_handle_otg_overcurrent(int irq, void *data);
irqreturn_t smblib_handle_chg_state_change(int irq, void *data);
irqreturn_t smblib_handle_step_chg_state_change(int irq, void *data);
irqreturn_t smblib_handle_step_chg_soc_update_fail(int irq, void *data);
irqreturn_t smblib_handle_step_chg_soc_update_request(int irq, void *data);
irqreturn_t smblib_handle_batt_temp_changed(int irq, void *data);
irqreturn_t smblib_handle_batt_psy_changed(int irq, void *data);
#ifdef CONFIG_IDTP9223_CHARGER
irqreturn_t smblib_handle_batt_qipma_on(int irq, void *data);
#endif
irqreturn_t smblib_handle_usb_psy_changed(int irq, void *data);
irqreturn_t smblib_handle_usbin_uv(int irq, void *data);
irqreturn_t smblib_handle_usb_plugin(int irq, void *data);
irqreturn_t smblib_handle_usb_source_change(int irq, void *data);
irqreturn_t smblib_handle_icl_change(int irq, void *data);
irqreturn_t smblib_handle_usb_typec_change(int irq, void *data);
irqreturn_t smblib_handle_dc_plugin(int irq, void *data);
irqreturn_t smblib_handle_high_duty_cycle(int irq, void *data);
irqreturn_t smblib_handle_switcher_power_ok(int irq, void *data);
irqreturn_t smblib_handle_wdog_bark(int irq, void *data);
#ifdef CONFIG_LGE_PM_CC_PROTECT
irqreturn_t smblib_handle_cc_protect(int irq, void* data);
#endif

int smblib_get_prop_input_suspend(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_present(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_capacity(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_status(struct smb_charger *chg,
				union power_supply_propval *val);
#ifdef CONFIG_LGE_PM
int smblib_get_prop_batt_status_for_ui(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_fcc_max(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_voltage_max(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_present_smem(struct smb_charger *chg,
				union power_supply_propval *val);
#endif
#ifdef CONFIG_LGE_PM_TIME_TO_FULL
int smblib_get_aicl_done(struct smb_charger *chg,
				union power_supply_propval *val);
#endif
int smblib_get_prop_batt_charge_type(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_charge_done(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_health(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_system_temp_level(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_input_current_limited(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_voltage_now(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_current_now(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_temp(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_step_chg_step(struct smb_charger *chg,
				union power_supply_propval *val);

int smblib_set_prop_input_suspend(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_batt_capacity(struct smb_charger *chg,
				const union power_supply_propval *val);
#ifdef CONFIG_LGE_PM
int smblib_set_prop_safety_timer_enabled(struct smb_charger *chg,
				bool enable);
int smblib_set_prop_parallel_batfet_en(struct smb_charger *chg,
				const union power_supply_propval *val);
#endif
#ifdef CONFIG_LGE_PM_INOV_GEN3_SYSFS_SUPPORT
int smblib_set_prop_inov_temp(struct smb_charger *chg,
				enum power_supply_property prop, const union power_supply_propval *val);
#endif
#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
int smblib_set_prop_moisture_detection(struct smb_charger *chg,
				  const union power_supply_propval *val);
#endif
int smblib_set_prop_system_temp_level(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_input_current_limited(struct smb_charger *chg,
				const union power_supply_propval *val);

int smblib_get_prop_dc_present(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_dc_online(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_dc_current_max(struct smb_charger *chg,
				union power_supply_propval *val);
#ifdef CONFIG_IDTP9223_CHARGER
int smblib_get_prop_qipma_on(struct smb_charger *chg,
				union power_supply_propval *val);
#endif
int smblib_set_prop_dc_current_max(struct smb_charger *chg,
				const union power_supply_propval *val);

#ifdef CONFIG_LGE_PM
int smblib_get_prop_fastchg_state(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_time_to_full(struct smb_charger *chg,
				union power_supply_propval *val);
#endif
int smblib_get_prop_usb_present(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_usb_online(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_usb_suspend(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_usb_voltage_now(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_pd_current_max(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_usb_current_max(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_usb_current_now(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_typec_cc_orientation(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_typec_power_role(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_pd_allowed(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_input_current_settled(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_input_voltage_settled(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_pd_in_hard_reset(struct smb_charger *chg,
			       union power_supply_propval *val);
int smblib_get_pe_start(struct smb_charger *chg,
			       union power_supply_propval *val);
int smblib_get_prop_charger_temp(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_charger_temp_max(struct smb_charger *chg,
				union power_supply_propval *val);
#ifdef CONFIG_LGE_PM_INOV_GEN3_SYSFS_SUPPORT
int smblib_get_prop_charger_temp_hot_max(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_skin_temp(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_skin_temp_max(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_skin_temp_hot_max(struct smb_charger *chg,
				union power_supply_propval *val);
#endif
int smblib_get_prop_die_health(struct smb_charger *chg,
			       union power_supply_propval *val);
int smblib_get_prop_charge_qnovo_enable(struct smb_charger *chg,
			       union power_supply_propval *val);
int smblib_set_prop_pd_current_max(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_usb_current_max(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_usb_voltage_min(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_usb_voltage_max(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_boost_current(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_typec_power_role(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_pd_active(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_pd_in_hard_reset(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_get_prop_slave_current_now(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_set_prop_ship_mode(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_charge_qnovo_enable(struct smb_charger *chg,
				const union power_supply_propval *val);
#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
int smblib_get_prop_typec_cc_disable(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_set_prop_typec_cc_disable(struct smb_charger *chg,
				const union power_supply_propval *val);
#endif
void smblib_suspend_on_debug_battery(struct smb_charger *chg);
int smblib_rerun_apsd_if_required(struct smb_charger *chg);
int smblib_get_prop_fcc_delta(struct smb_charger *chg,
			       union power_supply_propval *val);
int smblib_icl_override(struct smb_charger *chg, bool override);
int smblib_dp_dm(struct smb_charger *chg, int val);
int smblib_rerun_aicl(struct smb_charger *chg);
int smblib_set_icl_current(struct smb_charger *chg, int icl_ua);
int smblib_get_icl_current(struct smb_charger *chg, int *icl_ua);
int smblib_get_charge_current(struct smb_charger *chg, int *total_current_ua);
#ifdef CONFIG_LGE_PM_FG_AGE
int get_prop_battery_condition(struct smb_charger *chg,
		union power_supply_propval *val);
#endif
int smblib_get_prop_pr_swap_in_progress(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_set_prop_pr_swap_in_progress(struct smb_charger *chg,
				const union power_supply_propval *val);

int smblib_init(struct smb_charger *chg);
int smblib_deinit(struct smb_charger *chg);
#endif /* __SMB2_CHARGER_H */
