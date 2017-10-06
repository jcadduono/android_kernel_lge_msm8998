/* Copyright (c) 2013-2014, LG Eletronics. All rights reserved.
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

#define pr_fmt(fmt) "[LGE-CC] %s : " fmt, __func__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/power_supply.h>

#include <soc/qcom/lge/lge_charging_scenario.h>
#include <soc/qcom/lge/power/lge_power_class.h>
#include <soc/qcom/lge/board_lge.h>


#define MODULE_NAME "lge_charging_controller"
#define MONITOR_BATTEMP_POLLING_PERIOD  (60 * HZ)
#define RESTRICTED_CHG_CURRENT_800  800
#define RESTRICTED_CHG_CURRENT_700  700
#define RESTRICTED_CHG_CURRENT_500  500
#define RESTRICTED_CHG_CURRENT_400  400
#define RESTRICTED_CHG_CURRENT_300  300
#define CHG_CURRENT_MAX 3300

struct lge_charging_controller {
	struct power_supply		*batt_psy;
	struct power_supply		*usb_psy;
	struct power_supply		*bms_psy;
	struct lge_power 		lge_cc_lpc;
	struct lge_power 		*lge_cd_lpc;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PSEUDO_BATTERY
	struct lge_power		*lge_pb_lpc;
#endif
	struct delayed_work 	battemp_work;
	struct delayed_work 	uhd_record_work;
	struct delayed_work 	uhd_record_disable_work;
	struct wake_lock 		lcs_wake_lock;
	struct wake_lock 		chg_wake_lock;

	enum lge_charging_states battemp_chg_state;

	int chg_current_max;
	int otp_ibat_current;
	int before_otp_ibat_current;
	int otp_float_voltage;
	int before_otp_float_voltage;
	int pseudo_chg_ui;
	int before_battemp;
	int batt_temp;
	int btm_state;
	int start_batt_temp;
	int stop_batt_temp;
	int chg_enable;
	int before_chg_enable;
	int test_chg_scenario;
	int test_batt_therm_value;
	int is_chg_present;
	int chg_type;
	int update_type;
	int quick_chg_status;
	int lcd_status;
	bool quick_set_enable;
	int tdmb_mode_on;
	int uhd_record;
	int chg_done;
	int otp_enable;
	int fake_hvdcp_mode;
};

enum qpnp_quick_charging_status {
	HVDCP_STATUS_NONE = 0,
	HVDCP_STATUS_LCD_ON,
	HVDCP_STATUS_LCD_OFF,
	HVDCP_STATUS_CALL_ON,
	HVDCP_STATUS_CALL_OFF,
};

static enum lge_power_property lge_power_lge_cc_properties[] = {
	LGE_POWER_PROP_PSEUDO_BATT_UI,
	LGE_POWER_PROP_BTM_STATE,
	LGE_POWER_PROP_CHARGING_ENABLED,
	LGE_POWER_PROP_OTP_ENABLE,
	LGE_POWER_PROP_OTP_CURRENT,
	LGE_POWER_PROP_OTP_FLOAT_VOLTAGE,
	LGE_POWER_PROP_TEST_CHG_SCENARIO,
	LGE_POWER_PROP_TEST_BATT_THERM_VALUE,
	LGE_POWER_PROP_TEST_CHG_SCENARIO,
	LGE_POWER_PROP_TDMB_MODE_ON,
	LGE_POWER_PROP_UHD_RECORD,
	LGE_POWER_PROP_CHARGE_DONE,
	LGE_POWER_PROP_TYPE,
	LGE_POWER_PROP_HVDCP_FAKE_MODE,
	LGE_POWER_PROP_VOTE_REASON,
};

static char *lge_cc_supplied_to[] = {
	"battery",
};

static char *lge_cc_supplied_from[] = {
	"lge_cable_detect",
};

static struct lge_charging_controller *the_cc;

enum lgcc_vote_reason {
	LGCC_REASON_DEFAULT,
	LGCC_REASON_OTP,
	LGCC_REASON_LCD,
	LGCC_REASON_CALL,
	LGCC_REASON_THERMAL,
	LGCC_REASON_TDMB,
	LGCC_REASON_UHD_RECORD,
	LGCC_REASON_MAX,
};

static int lgcc_vote_fcc_table[LGCC_REASON_MAX] = {
	CHG_CURRENT_MAX,	/* max ibat current */
	-EINVAL,
	-EINVAL,
	-EINVAL,
	-EINVAL,
	-EINVAL,
	-EINVAL,
};

static int lgcc_vote_fcc_reason = -EINVAL;
static int lgcc_vote_fcc_value = -EINVAL;
static void lgcc_vote_fcc_update(void)
{
	int fcc = INT_MAX;
	int reason = -EINVAL;
	int i;

	for (i = 0; i < LGCC_REASON_MAX; i++) {
		if (lgcc_vote_fcc_table[i] == -EINVAL)
			continue;

		if (fcc > lgcc_vote_fcc_table[i]) {
			fcc = lgcc_vote_fcc_table[i];
			reason = i;
		}
	}

	if (!the_cc)
		return;

	if (reason != lgcc_vote_fcc_reason || fcc != lgcc_vote_fcc_value) {
		lgcc_vote_fcc_reason = reason;
		lgcc_vote_fcc_value = fcc;
		pr_info("lgcc_vote: vote id[%d], set cur[%d]\n",
				reason, fcc);
		lge_power_changed(&the_cc->lge_cc_lpc);
	}
}

static int lgcc_vote_fcc(int reason, int fcc)
{
	lgcc_vote_fcc_table[reason] = fcc;
	lgcc_vote_fcc_update();

	return 0;
}

static int lgcc_vote_fcc_get(void)
{
	if (lgcc_vote_fcc_reason == -EINVAL)
		return -EINVAL;

	return lgcc_vote_fcc_table[lgcc_vote_fcc_reason];
}

static int lgcc_float_vol_get(struct lge_charging_controller *cc)
{
	return cc->otp_float_voltage;
}

static int lgcc_thermal_mitigation;
module_param_call(lgcc_thermal_mitigation,
		NULL,
		param_get_int, &lgcc_thermal_mitigation, 0644);

static int lgcc_hvdcp_thermal_mitigation;
static int lgcc_set_hvdcp_thermal_chg_current(const char *val,
		struct kernel_param *kp) {

	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (!the_cc) {
		pr_err("lgcc is not ready\n");
		return 0;
	}
	if (!the_cc->otp_enable) {
		pr_err("lgcc is disabled\n");
		return 0;
	}

	if (lgcc_hvdcp_thermal_mitigation > 0 && lgcc_hvdcp_thermal_mitigation < CHG_CURRENT_MAX) {
#ifdef CONFIG_LGE_USB_FACTORY
		if (lge_get_boot_mode() == LGE_BOOT_MODE_QEM_56K ||
				lge_get_boot_mode() == LGE_BOOT_MODE_QEM_130K ||
				lge_get_boot_mode() == LGE_BOOT_MODE_QEM_910K) {
			pr_info("MiniOS boot!!!\n");
			if (lgcc_hvdcp_thermal_mitigation < 500)
				lgcc_hvdcp_thermal_mitigation = 1000;
		}
#endif
		lgcc_vote_fcc(LGCC_REASON_THERMAL, lgcc_hvdcp_thermal_mitigation);
	} else if (lgcc_hvdcp_thermal_mitigation == 0) {
		pr_info("thermal_mitigation current is 0, stop charging!\n");
		lgcc_vote_fcc(LGCC_REASON_THERMAL, lgcc_hvdcp_thermal_mitigation);
	} else {
		pr_info("Released thermal mitigation\n");
		lgcc_vote_fcc(LGCC_REASON_THERMAL, -EINVAL);
	}
	pr_err("thermal_mitigation = %d\n", lgcc_hvdcp_thermal_mitigation);
	the_cc->chg_enable = !!lgcc_vote_fcc_value;

	return 0;
}
module_param_call(lgcc_hvdcp_thermal_mitigation,
		lgcc_set_hvdcp_thermal_chg_current,
		param_get_int, &lgcc_hvdcp_thermal_mitigation, 0644);

#define RESTRICTED_CALL_STATE 500
#define RESTRICTED_LCD_STATE 1000
static int quick_charging_state;
static int set_quick_charging_state(const char *val,
		struct kernel_param *kp) {
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_info("quick_charging_state error = %d\n", ret);
		return ret;
	}

	the_cc->quick_chg_status = quick_charging_state;

	switch (quick_charging_state) {
		case HVDCP_STATUS_LCD_ON:
			lgcc_vote_fcc(LGCC_REASON_LCD, RESTRICTED_LCD_STATE);
			the_cc->lcd_status = true;
			pr_info("LCD on decreasing chg_current\n");
			break;

		case HVDCP_STATUS_LCD_OFF:
			lgcc_vote_fcc(LGCC_REASON_LCD, -EINVAL);
			the_cc->lcd_status = false;
			pr_debug("LCD off return max chg_current\n");
			break;

		case HVDCP_STATUS_CALL_ON:
			lgcc_vote_fcc(LGCC_REASON_CALL, RESTRICTED_CALL_STATE);
			pr_info("Call on  decreasing chg_current\n");
			break;

		case HVDCP_STATUS_CALL_OFF:
			lgcc_vote_fcc(LGCC_REASON_CALL, -EINVAL);
			pr_debug("Call off return max chg_current\n");
			break;

		default:
			lgcc_vote_fcc(LGCC_REASON_LCD, -EINVAL);
			lgcc_vote_fcc(LGCC_REASON_CALL, -EINVAL);
			break;
	}

	pr_info("set quick_charging_state[%d]\n", quick_charging_state);

	lge_power_changed(&the_cc->lge_cc_lpc);
	return 0;
}
module_param_call(quick_charging_state, set_quick_charging_state,
		param_get_int, &quick_charging_state, 0644);

#define DELAY_FOR_UHD_RECORD_PERIOD  (5 * 60 * HZ)
static void lge_uhd_record_work(struct work_struct *work){
	struct lge_charging_controller *cc =
		container_of(work, struct lge_charging_controller,
			uhd_record_work.work);

	if (cc->uhd_record) {
		pr_info("current of uhd record is changed.\n");
		lgcc_vote_fcc(LGCC_REASON_UHD_RECORD, RESTRICTED_CHG_CURRENT_700);
	}
}

#define DELAY_FOR_UHD_RECORD_DISABLE_PERIOD  (6 * HZ)
static void lge_uhd_record_disable_work(struct work_struct *work){
	struct lge_charging_controller *cc =
		container_of(work, struct lge_charging_controller,
			uhd_record_disable_work.work);

	if (!cc->uhd_record) {
		pr_info("Unset current of uhd record.\n");
		cancel_delayed_work_sync(&cc->uhd_record_work);
		lgcc_vote_fcc(LGCC_REASON_UHD_RECORD, -EINVAL);
	}
}

#define DECCUR_FLOAT_VOLTAGE_IN_WARM 4000000
#define OVER_VOLTAGE_DELTA 60000
static void lge_monitor_batt_temp_work(struct work_struct *work){

	struct charging_info req;
	struct charging_rsp res;
	bool is_changed = false;
	int backup_batt_temp = 0;
	union power_supply_propval ret = {0,};
	union lge_power_propval lge_val = {0,};
	struct lge_charging_controller *cc =
		container_of(work, struct lge_charging_controller,
			battemp_work.work);

	cc->usb_psy = power_supply_get_by_name("usb");
	if(!cc->usb_psy){
		pr_err("usb power_supply not found deferring probe\n");
		schedule_delayed_work(&cc->battemp_work,
			MONITOR_BATTEMP_POLLING_PERIOD);
		return;
	}

	cc->batt_psy = power_supply_get_by_name("battery");
	if(!cc->batt_psy){
		pr_err("battery power_supply not found deferring probe\n");
		schedule_delayed_work(&cc->battemp_work,
			MONITOR_BATTEMP_POLLING_PERIOD);
		return;
	}

	cc->lge_cd_lpc = lge_power_get_by_name("lge_cable_detect");
	if (!cc->lge_cd_lpc) {
		pr_err("lge_cd_lpc is not yet ready\n");
		schedule_delayed_work(&cc->battemp_work,
			MONITOR_BATTEMP_POLLING_PERIOD);
		return;
	}

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PSEUDO_BATTERY
	if (cc->test_chg_scenario) {
		if (cc->lge_pb_lpc) {
			cc->lge_pb_lpc->get_property(cc->lge_pb_lpc,
					LGE_POWER_PROPS_PSEUDO_BATT_CHARGING, &lge_val);
			cc->chg_enable = lge_val.intval;
		}
	}
#endif

	cc->batt_psy->desc->get_property(cc->batt_psy,
			POWER_SUPPLY_PROP_TEMP, &ret);
	if (cc->test_chg_scenario == 1)
		req.batt_temp = cc->test_batt_therm_value*10;
	else
		req.batt_temp = ret.intval;

	backup_batt_temp = req.batt_temp;
	req.lcs_en = cc->otp_enable;
	if (!cc->otp_enable) {
		req.batt_temp = 250;
	}
	cc->batt_temp = req.batt_temp;

	cc->batt_psy->desc->get_property(cc->batt_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
	req.batt_volt = ret.intval;

	req.max_chg_volt = -1;

	cc->batt_psy->desc->get_property(cc->batt_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
	req.current_now = ret.intval / 1000;

	cc->lge_cd_lpc->get_property(cc->lge_cd_lpc,
			LGE_POWER_PROP_BATT_CAPACITY_MIN, &lge_val);
	req.chg_current_ma = lge_val.intval;

	pr_info("chg_current_max = %d, otp_enable=%d(temp=%d)\n",
			req.chg_current_ma, cc->otp_enable, backup_batt_temp);
	req.is_charger = cc->is_chg_present;

	lge_monitor_batt_temp(req, &res);

	if (((res.change_lvl != STS_CHE_NONE) && req.is_charger) ||
			(res.force_update == true) ||
			cc->otp_float_voltage != res.float_voltage) {
		if (res.state == CHG_BATT_NORMAL_STATE) {
			pr_info("ibatmax_set CHG_BATT_NORMAL_STATE\n");
			cc->otp_ibat_current = res.chg_current;
			cc->otp_float_voltage = res.float_voltage;
			lgcc_vote_fcc(LGCC_REASON_OTP, -EINVAL);
			cc->chg_enable = !!lgcc_vote_fcc_value;
			if (wake_lock_active(&cc->lcs_wake_lock))
				wake_unlock(&cc->lcs_wake_lock);
		} else if (res.state == CHG_BATT_DECCUR_STATE) {
			pr_info("ibatmax_set CHG_BATT_DECCUR_STATE\n");
			cc->otp_ibat_current = res.chg_current;
			cc->otp_float_voltage = res.float_voltage;
			lgcc_vote_fcc(LGCC_REASON_OTP, res.chg_current);
			cc->chg_enable = !!lgcc_vote_fcc_value;

			// control wake lock in DECCUR status
			if (res.float_voltage == DECCUR_FLOAT_VOLTAGE_IN_WARM) {
				/* acquire wake_lock when vfloat is set to 4000mV (in warm status) */
				if (!wake_lock_active(&cc->lcs_wake_lock))
					wake_lock(&cc->lcs_wake_lock);
			} else {
				/* release wake_lock in cool status */
				if (wake_lock_active(&cc->lcs_wake_lock))
					wake_unlock(&cc->lcs_wake_lock);
			}
		} else if (res.state == CHG_BATT_STPCHG_STATE) {
			pr_info("ibatmax_set CHG_BATT_STPCHG_STATE\n");
			cc->otp_ibat_current = 0;
			cc->otp_float_voltage = res.float_voltage;
			lgcc_vote_fcc(LGCC_REASON_OTP, 0);
			cc->chg_enable = !!lgcc_vote_fcc_value;
			if (!wake_lock_active(&cc->lcs_wake_lock))
				wake_lock(&cc->lcs_wake_lock);
		}
	}

	pr_info("otp_ibat_current=%d\n", cc->otp_ibat_current);

	pr_debug("cc->pseudo_chg_ui = %d, res.pseudo_chg_ui = %d\n",
			cc->pseudo_chg_ui, res.pseudo_chg_ui);
	if (cc->pseudo_chg_ui ^ res.pseudo_chg_ui) {
		is_changed = true;
		cc->pseudo_chg_ui = res.pseudo_chg_ui;
	}

// This code only needs msm8998. Other device must check this code.
	if (req.batt_volt < DECCUR_FLOAT_VOLTAGE_IN_WARM + OVER_VOLTAGE_DELTA &&
			res.float_voltage == DECCUR_FLOAT_VOLTAGE_IN_WARM &&
			res.state == CHG_BATT_DECCUR_STATE) {
		is_changed = true;
	}

	pr_debug("cc->btm_state = %d, res.btm_state = %d\n",
			cc->btm_state, res.btm_state);
	if (cc->btm_state ^ res.btm_state) {
		is_changed = true;
		cc->btm_state = res.btm_state;
	}

	if (cc->before_battemp != req.batt_temp) {
		is_changed = true;
		cc->before_battemp = req.batt_temp;
	}

	if (cc->before_chg_enable != cc->chg_enable) {
		is_changed = true;
		cc->before_chg_enable = cc->chg_enable;
	}

	if (cc->before_otp_ibat_current != cc->otp_ibat_current) {
		is_changed = true;
		cc->before_otp_ibat_current = cc->otp_ibat_current;
	}

	if (cc->before_otp_float_voltage != cc->otp_float_voltage) {
		is_changed = true;
		cc->before_otp_float_voltage = cc->otp_float_voltage;
	}

	if (cc->update_type == 1) {
		is_changed = true;
		cc->update_type = 0;
	}

	cc->batt_psy->desc->get_property(cc->batt_psy,
			POWER_SUPPLY_PROP_CAPACITY, &ret);
	pr_debug("cap : %d, chg_type : %d\n",
			ret.intval, cc->chg_type);
	if (!cc->is_chg_present ||
			(cc->is_chg_present && cc->chg_done)) {
		if (wake_lock_active(&cc->chg_wake_lock)) {
			pr_info("chg_wake_unlocked\n");
			wake_unlock(&cc->chg_wake_lock);
		}
	} else if (cc->is_chg_present && !cc->chg_done) {
		if (!wake_lock_active(&cc->chg_wake_lock)) {
			pr_info("chg_wake_locked\n");
			wake_lock(&cc->chg_wake_lock);
		}
	}

	if (is_changed == true)
		lge_power_changed(&cc->lge_cc_lpc);

	pr_info("Reported Capacity : %d / voltage : %d\n",
			ret.intval, req.batt_volt/1000);

	if (cc->batt_temp <= 30)
		schedule_delayed_work(&cc->battemp_work,
				MONITOR_BATTEMP_POLLING_PERIOD / 6);
	else if (cc->batt_temp > 30 && cc->batt_temp <= 120)
		schedule_delayed_work(&cc->battemp_work,
				MONITOR_BATTEMP_POLLING_PERIOD / 3);
	else if (cc->batt_temp >= 430 && cc->batt_temp <= 550)
		schedule_delayed_work(&cc->battemp_work,
				MONITOR_BATTEMP_POLLING_PERIOD / 3);
	else if (cc->batt_temp >= 550 && cc->batt_temp <= 590)
		schedule_delayed_work(&cc->battemp_work,
				MONITOR_BATTEMP_POLLING_PERIOD / 6);
	else
		schedule_delayed_work(&cc->battemp_work,
				MONITOR_BATTEMP_POLLING_PERIOD);
}

static int lg_cc_get_pseudo_ui(struct lge_charging_controller *cc) {
	if(!(cc == NULL)) {
		return cc->pseudo_chg_ui;
	}

	return 0;
}

static int lg_cc_get_btm_state(struct lge_charging_controller *cc) {
	if(!(cc == NULL)) {
		return cc->btm_state;
	}

	return 0;
}

static void lg_cc_start_battemp_work(struct lge_charging_controller *cc,
		int delay) {
	pr_debug("start_battemp_work~!!\n");
	schedule_delayed_work(&cc->battemp_work, (delay * HZ));
}

static void lg_cc_stop_battemp_work(struct lge_charging_controller *cc) {
	pr_debug("stop_battemp_work~!!\n");
	cancel_delayed_work(&cc->battemp_work);
}

static int lge_power_lge_cc_property_is_writeable(struct lge_power *lpc,
		enum lge_power_property lpp) {
	int ret = 0;
	switch (lpp) {
		case LGE_POWER_PROP_TEST_CHG_SCENARIO:
		case LGE_POWER_PROP_TEST_BATT_THERM_VALUE:
		case LGE_POWER_PROP_OTP_ENABLE:
		case LGE_POWER_PROP_TDMB_MODE_ON:
		case LGE_POWER_PROP_UHD_RECORD:
		case LGE_POWER_PROP_HVDCP_FAKE_MODE:
			ret = 1;
			break;
		default:
			break;
	}

	return ret;
}

static int lge_power_lge_cc_set_property(struct lge_power *lpc,
		enum lge_power_property lpp,
		const union lge_power_propval *val) {
	int ret_val = 0;
	struct lge_charging_controller *cc
		= container_of(lpc,	struct lge_charging_controller,
				lge_cc_lpc);

	switch (lpp) {
		case LGE_POWER_PROP_TEST_CHG_SCENARIO:
			cc->test_chg_scenario = val->intval;
			break;

		case LGE_POWER_PROP_TEST_BATT_THERM_VALUE:
			cc->test_batt_therm_value = val->intval;
			lg_cc_stop_battemp_work(cc);
			lg_cc_start_battemp_work(cc, 2);
			break;

		case LGE_POWER_PROP_TDMB_MODE_ON:
			cc->tdmb_mode_on = val->intval;
			pr_info("tdmb mode is set to [%d]\n", cc->tdmb_mode_on);
			if (cc->tdmb_mode_on == 1)
				lgcc_vote_fcc(LGCC_REASON_TDMB, RESTRICTED_CHG_CURRENT_500);
			else if (cc->tdmb_mode_on == 2)
				lgcc_vote_fcc(LGCC_REASON_TDMB, RESTRICTED_CHG_CURRENT_300);
			else if (cc->tdmb_mode_on == 0)
				lgcc_vote_fcc(LGCC_REASON_TDMB, -EINVAL);
			break;

		case LGE_POWER_PROP_UHD_RECORD:
			cc->uhd_record = val->intval;
			pr_info("uhd record is set to [%d]\n", cc->uhd_record);
			if (cc->uhd_record == 1) {
				if (delayed_work_pending(&cc->uhd_record_work)
						|| delayed_work_pending(&cc->uhd_record_disable_work)) {
					cancel_delayed_work_sync(&cc->uhd_record_disable_work);
				} else {
					schedule_delayed_work(&cc->uhd_record_work, DELAY_FOR_UHD_RECORD_PERIOD);
					lgcc_vote_fcc(LGCC_REASON_UHD_RECORD, RESTRICTED_CHG_CURRENT_400);
				}
			} else if (cc->uhd_record == 0) {
				schedule_delayed_work(&cc->uhd_record_disable_work, DELAY_FOR_UHD_RECORD_DISABLE_PERIOD);
			}
			break;

		case LGE_POWER_PROP_CHARGE_DONE:
			cc->chg_done = val->intval;
			if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
				lg_cc_stop_battemp_work(cc);
				lg_cc_start_battemp_work(cc, 2);
			}
			break;
		case LGE_POWER_PROP_OTP_ENABLE:
			if (cc->otp_enable != val->intval) {
				cc->otp_enable = val->intval;
				cancel_delayed_work_sync(&cc->battemp_work);
				schedule_delayed_work(&cc->battemp_work, 0);
			}
			break;
		case LGE_POWER_PROP_HVDCP_FAKE_MODE:
			if (cc->fake_hvdcp_mode != val->intval)
				cc->fake_hvdcp_mode = val->intval;
			break;
		default:
			pr_err("lpp:%d is not supported!!!\n", lpp);
			ret_val = -EINVAL;
			break;
	}
	lge_power_changed(&cc->lge_cc_lpc);

	return ret_val;
}

static int lge_power_lge_cc_get_property(struct lge_power *lpc,
		enum lge_power_property lpp,
		union lge_power_propval *val) {
	int ret_val = 0;

	struct lge_charging_controller *cc
		= container_of(lpc, struct lge_charging_controller,
				lge_cc_lpc);
	switch (lpp) {
		case LGE_POWER_PROP_PSEUDO_BATT_UI:
			val->intval = lg_cc_get_pseudo_ui(cc);
			break;

		case LGE_POWER_PROP_BTM_STATE:
			val->intval = lg_cc_get_btm_state(cc);
			break;

		case LGE_POWER_PROP_CHARGING_ENABLED:
			val->intval = cc->chg_enable;
			break;

		case LGE_POWER_PROP_OTP_ENABLE:
			val->intval = cc->otp_enable;
			break;

		case LGE_POWER_PROP_OTP_CURRENT:
			val->intval = lgcc_vote_fcc_get();
			break;

		case LGE_POWER_PROP_OTP_FLOAT_VOLTAGE:
			val->intval = lgcc_float_vol_get(cc);
			break;

		case LGE_POWER_PROP_TEST_CHG_SCENARIO:
			val->intval = cc->test_chg_scenario;
			break;

		case LGE_POWER_PROP_TEST_BATT_THERM_VALUE:
			val->intval = cc->test_batt_therm_value;
			break;

		case LGE_POWER_PROP_TDMB_MODE_ON:
			val->intval = cc->tdmb_mode_on;
			break;

		case LGE_POWER_PROP_UHD_RECORD:
			val->intval = cc->uhd_record;
			break;

		case LGE_POWER_PROP_CHARGE_DONE:
			val->intval = cc->chg_done;
			break;

		case LGE_POWER_PROP_TYPE:
			val->intval = cc->chg_type;
			break;

		case LGE_POWER_PROP_HVDCP_FAKE_MODE:
			val->intval = cc->fake_hvdcp_mode;
			break;

		case LGE_POWER_PROP_LCD_STATUS:
			val->intval = cc->lcd_status;
			break;

		case LGE_POWER_PROP_VOTE_REASON:
			val->intval = lgcc_vote_fcc_reason;
			break;

		default:
			ret_val = -EINVAL;
			break;
	}

	return ret_val;
}

static void lge_cc_external_lge_power_changed(struct lge_power *lpc) {
	union lge_power_propval lge_val = {0,};

	int rc, is_changed = 0;
	struct lge_charging_controller *cc
		= container_of(lpc, struct lge_charging_controller,
				lge_cc_lpc);

	cc->lge_cd_lpc = lge_power_get_by_name("lge_cable_detect");
	if(!cc->lge_cd_lpc){
		pr_err("cable detection not found deferring probe\n");
	} else {
		rc = cc->lge_cd_lpc->get_property(cc->lge_cd_lpc,
				LGE_POWER_PROP_CHG_PRESENT, &lge_val);
		if (rc == 0) {
			if (cc->is_chg_present != lge_val.intval) {
				pr_info("chg present : %d\n",lge_val.intval);
				cc->is_chg_present = lge_val.intval;
				is_changed = 1;
			}
		}

		rc = cc->lge_cd_lpc->get_property(cc->lge_cd_lpc,
					LGE_POWER_PROP_TYPE, &lge_val);
		if (rc == 0) {
			if (cc->chg_type != lge_val.intval) {
				pr_err("chg type : %d\n",lge_val.intval);
				cc->chg_type = lge_val.intval;
				is_changed = 1;
			}
		}

		rc = cc->lge_cd_lpc->get_property(cc->lge_cd_lpc,
				LGE_POWER_PROP_CHARGING_CURRENT_MAX, &lge_val);
		pr_info("charing_current_max is %d\n", lge_val.intval/1000);

		if (rc < 0)
			pr_info("Failed to get current max!!!\n");
		else {
			if (cc->is_chg_present)
				lgcc_vote_fcc(LGCC_REASON_DEFAULT, lge_val.intval/1000);
		}

		if (is_changed) {
			lg_cc_stop_battemp_work(cc);
			lg_cc_start_battemp_work(cc,2);
			cc->update_type = 1;
		}
	}
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PSEUDO_BATTERY
	if (!cc->lge_pb_lpc) {
		cc->lge_pb_lpc = lge_power_get_by_name("pseudo_battery");
	}
	if(!cc->lge_pb_lpc){
		pr_err("pseudo battery not found deferring probe\n");
	} else {
		rc = cc->lge_pb_lpc->get_property(cc->lge_pb_lpc,
				LGE_POWER_PROP_PSEUDO_BATT, &lge_val);
		if (rc == 0)
			cc->test_chg_scenario = lge_val.intval;
		if (cc->test_chg_scenario) {
			rc = cc->lge_pb_lpc->get_property(cc->lge_pb_lpc,
				LGE_POWER_PROPS_PSEUDO_BATT_TEMP, &lge_val);
			if (rc == 0)
				cc->test_batt_therm_value = lge_val.intval;
		}
	}
#endif
}

static int lge_charging_controller_probe(struct platform_device *pdev) {
	struct lge_charging_controller *cc;
	struct lge_power *lge_power_cc;
	int ret;

	cc = kzalloc(sizeof(struct lge_charging_controller),
								GFP_KERNEL);
	if (!cc) {
		pr_err("lge_charging_controller memory alloc failed.\n");
		return -ENOMEM;
	}

	the_cc = cc;
	platform_set_drvdata(pdev, cc);

	wake_lock_init(&cc->lcs_wake_lock,
			WAKE_LOCK_SUSPEND, "lge_charging_scenario");
	wake_lock_init(&cc->chg_wake_lock,
			WAKE_LOCK_SUSPEND, "lge_charging_wake_lock");

	INIT_DELAYED_WORK(&cc->battemp_work,
			lge_monitor_batt_temp_work);
	INIT_DELAYED_WORK(&cc->uhd_record_work,
			lge_uhd_record_work);
	INIT_DELAYED_WORK(&cc->uhd_record_disable_work,
			lge_uhd_record_disable_work);

	lge_power_cc = &cc->lge_cc_lpc;
	lge_power_cc->name = "lge_cc";
	lge_power_cc->properties = lge_power_lge_cc_properties;
	lge_power_cc->num_properties =
		ARRAY_SIZE(lge_power_lge_cc_properties);
	lge_power_cc->get_property = lge_power_lge_cc_get_property;
	lge_power_cc->set_property = lge_power_lge_cc_set_property;
	lge_power_cc->property_is_writeable =
		lge_power_lge_cc_property_is_writeable;
	lge_power_cc->supplied_to = lge_cc_supplied_to;
	lge_power_cc->num_supplicants = ARRAY_SIZE(lge_cc_supplied_to);
	lge_power_cc->lge_supplied_from = lge_cc_supplied_from;
	lge_power_cc->num_lge_supplies	= ARRAY_SIZE(lge_cc_supplied_from);
	lge_power_cc->external_lge_power_changed
			= lge_cc_external_lge_power_changed;

	ret = lge_power_register(&pdev->dev, lge_power_cc);
	if (ret < 0) {
		pr_err("Failed to register lge power class: %d\n", ret);
		goto err_free;
	}

	cc->chg_current_max = -EINVAL;

	cc->otp_enable = 1;
	cc->chg_enable = -1;
	cc->before_chg_enable = -1;
	cc->otp_ibat_current = -1;
	cc->before_otp_ibat_current = -1;
	cc->otp_float_voltage = -1;
	cc->before_otp_float_voltage = -1;
	cc->fake_hvdcp_mode = 0;

	cc->start_batt_temp = 5;
	lg_cc_start_battemp_work(cc, cc->start_batt_temp);
	cc->test_chg_scenario = 0;
	cc->test_batt_therm_value = 25;
	cc->update_type = 0;
	if (lge_get_boot_mode() != LGE_BOOT_MODE_CHARGERLOGO) {
		cc->quick_chg_status = HVDCP_STATUS_LCD_ON;
		cc->lcd_status = true;
		lgcc_vote_fcc(LGCC_REASON_LCD, RESTRICTED_LCD_STATE);
	}

	pr_info("LG Charging controller probe done~!!\n");

	return 0;

err_free:
	kfree(cc);
	return ret;
}

static int lge_charging_controller_suspend(struct device *dev)
{
	struct lge_charging_controller *cc = dev_get_drvdata(dev);

	if (!the_cc) {
		pr_err("lgcc is not ready\n");
		return 0;
	}

	cancel_delayed_work_sync(&cc->battemp_work);

	return 0;
}

static int lge_charging_controller_resume(struct device *dev)
{
	struct lge_charging_controller *cc = dev_get_drvdata(dev);

	if (!the_cc) {
		pr_err("lgcc is not ready\n");
		return 0;
	}

	cancel_delayed_work_sync(&cc->battemp_work);
	schedule_delayed_work(&cc->battemp_work, 0);

	return 0;
}

static const struct dev_pm_ops lge_charging_controller_pm_ops = {
	.suspend	= lge_charging_controller_suspend,
	.resume		= lge_charging_controller_resume,
};

#ifdef CONFIG_OF
static struct of_device_id lge_charging_controller_match_table[] = {
	{.compatible = "lge,charging_controller"},
	{ },
};
#endif

static int lge_charging_controller_remove(struct platform_device *pdev) {
	lge_power_unregister(&the_cc->lge_cc_lpc);
	kfree(the_cc);

	return 0;
}

static struct platform_driver lge_charging_controller_driver = {
	.probe = lge_charging_controller_probe,
	.remove = lge_charging_controller_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = lge_charging_controller_match_table,
#endif
		.pm = &lge_charging_controller_pm_ops,
	},
};

static int __init lge_charging_controller_init(void) {
	return platform_driver_register(&lge_charging_controller_driver);
}

static void __exit lge_charging_controller_exit(void) {
	platform_driver_unregister(&lge_charging_controller_driver);
}

module_init(lge_charging_controller_init);
module_exit(lge_charging_controller_exit);

MODULE_DESCRIPTION("LGE Charging Controller driver");
MODULE_LICENSE("GPL v2");
