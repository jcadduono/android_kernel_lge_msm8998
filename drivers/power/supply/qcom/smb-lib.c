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

#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/input/qpnp-power-on.h>
#include <linux/irq.h>
#include <linux/pmic-voter.h>
#include "smb-lib.h"
#include "smb-reg.h"
#include "battery.h"
#include "storm-watch.h"
#ifdef CONFIG_LGE_PM_SUPPORT_LG_POWER_CLASS
#include <soc/qcom/lge/power/lge_power_class.h>
#endif
#ifdef CONFIG_LGE_PM
#include <soc/qcom/lge/power/lge_board_revision.h>
#include <soc/qcom/lge/board_lge.h>
#include <linux/of_gpio.h>
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
#include <soc/qcom/lge/lge_charging_scenario.h>
#endif

#define smblib_err(chg, fmt, ...)		\
	pr_err("%s: %s: " fmt, chg->name,	\
		__func__, ##__VA_ARGS__)	\

#define smblib_dbg(chg, reason, fmt, ...)			\
	do {							\
		if (*chg->debug_mask & (reason))		\
			pr_info("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
	} while (0)

static bool is_secure(struct smb_charger *chg, int addr)
{
	if (addr == SHIP_MODE_REG || addr == FREQ_CLK_DIV_REG)
		return true;
	/* assume everything above 0xA0 is secure */
	return (bool)((addr & 0xFF) >= 0xA0);
}

int smblib_read(struct smb_charger *chg, u16 addr, u8 *val)
{
	unsigned int temp;
	int rc = 0;

	rc = regmap_read(chg->regmap, addr, &temp);
	if (rc >= 0)
		*val = (u8)temp;

	return rc;
}

int smblib_multibyte_read(struct smb_charger *chg, u16 addr, u8 *val,
				int count)
{
	return regmap_bulk_read(chg->regmap, addr, val, count);
}

int smblib_masked_write(struct smb_charger *chg, u16 addr, u8 mask, u8 val)
{
	int rc = 0;

	mutex_lock(&chg->write_lock);
	if (is_secure(chg, addr)) {
		rc = regmap_write(chg->regmap, (addr & 0xFF00) | 0xD0, 0xA5);
		if (rc < 0)
			goto unlock;
	}

	rc = regmap_update_bits(chg->regmap, addr, mask, val);

unlock:
	mutex_unlock(&chg->write_lock);
	return rc;
}

int smblib_write(struct smb_charger *chg, u16 addr, u8 val)
{
	int rc = 0;

	mutex_lock(&chg->write_lock);

	if (is_secure(chg, addr)) {
		rc = regmap_write(chg->regmap, (addr & ~(0xFF)) | 0xD0, 0xA5);
		if (rc < 0)
			goto unlock;
	}

	rc = regmap_write(chg->regmap, addr, val);

unlock:
	mutex_unlock(&chg->write_lock);
	return rc;
}

static int smblib_get_step_cc_delta(struct smb_charger *chg, int *cc_delta_ua)
{
	int rc, step_state;
	u8 stat;

	if (!chg->step_chg_enabled) {
		*cc_delta_ua = 0;
		return 0;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	step_state = (stat & STEP_CHARGING_STATUS_MASK) >>
				STEP_CHARGING_STATUS_SHIFT;
	rc = smblib_get_charge_param(chg, &chg->param.step_cc_delta[step_state],
				     cc_delta_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get step cc delta rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int smblib_get_jeita_cc_delta(struct smb_charger *chg, int *cc_delta_ua)
{
	int rc, cc_minus_ua;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}

	if (!(stat & BAT_TEMP_STATUS_SOFT_LIMIT_MASK)) {
		*cc_delta_ua = 0;
		return 0;
	}

	rc = smblib_get_charge_param(chg, &chg->param.jeita_cc_comp,
				     &cc_minus_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get jeita cc minus rc=%d\n", rc);
		return rc;
	}

	*cc_delta_ua = -cc_minus_ua;
	return 0;
}

int smblib_icl_override(struct smb_charger *chg, bool override)
{
	int rc;

	rc = smblib_masked_write(chg, USBIN_LOAD_CFG_REG,
				ICL_OVERRIDE_AFTER_APSD_BIT,
				override ? ICL_OVERRIDE_AFTER_APSD_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't override ICL rc=%d\n", rc);

	return rc;
}

/********************
 * REGISTER GETTERS *
 ********************/

int smblib_get_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int *val_u)
{
	int rc = 0;
	u8 val_raw;

	rc = smblib_read(chg, param->reg, &val_raw);
	if (rc < 0) {
		smblib_err(chg, "%s: Couldn't read from 0x%04x rc=%d\n",
			param->name, param->reg, rc);
		return rc;
	}

	if (param->get_proc)
		*val_u = param->get_proc(param, val_raw);
	else
		*val_u = val_raw * param->step_u + param->min_u;
	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, *val_u, val_raw);

	return rc;
}

int smblib_get_usb_suspend(struct smb_charger *chg, int *suspend)
{
	int rc = 0;
	u8 temp;

	rc = smblib_read(chg, USBIN_CMD_IL_REG, &temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_CMD_IL rc=%d\n", rc);
		return rc;
	}
	*suspend = temp & USBIN_SUSPEND_BIT;

	return rc;
}

struct apsd_result {
	const char * const name;
	const u8 bit;
	const enum power_supply_type pst;
};

enum {
	UNKNOWN,
	SDP,
	CDP,
	DCP,
	OCP,
	FLOAT,
	HVDCP2,
	HVDCP3,
	MAX_TYPES
};

static const struct apsd_result const smblib_apsd_results[] = {
	[UNKNOWN] = {
		.name	= "UNKNOWN",
		.bit	= 0,
		.pst	= POWER_SUPPLY_TYPE_UNKNOWN
	},
	[SDP] = {
		.name	= "SDP",
		.bit	= SDP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB
	},
	[CDP] = {
		.name	= "CDP",
		.bit	= CDP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_CDP
	},
	[DCP] = {
		.name	= "DCP",
		.bit	= DCP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[OCP] = {
		.name	= "OCP",
		.bit	= OCP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[FLOAT] = {
		.name	= "FLOAT",
		.bit	= FLOAT_CHARGER_BIT,
#ifdef CONFIG_LGE_USB_COMPLIANCE_TEST
		.pst	= POWER_SUPPLY_TYPE_USB
#else
#ifdef CONFIG_LGE_PM
		.pst	= POWER_SUPPLY_TYPE_USB_FLOATED
#else
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
#endif
#endif
	},
	[HVDCP2] = {
		.name	= "HVDCP2",
		.bit	= DCP_CHARGER_BIT | QC_2P0_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_HVDCP
	},
	[HVDCP3] = {
		.name	= "HVDCP3",
		.bit	= DCP_CHARGER_BIT | QC_3P0_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_HVDCP_3,
	},
};

static const struct apsd_result *smblib_get_apsd_result(struct smb_charger *chg)
{
	int rc, i;
	u8 apsd_stat, stat;
	const struct apsd_result *result = &smblib_apsd_results[UNKNOWN];

	rc = smblib_read(chg, APSD_STATUS_REG, &apsd_stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return result;
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", apsd_stat);

	if (!(apsd_stat & APSD_DTC_STATUS_DONE_BIT))
		return result;

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_RESULT_STATUS rc=%d\n",
			rc);
		return result;
	}
	stat &= APSD_RESULT_STATUS_MASK;

	for (i = 0; i < ARRAY_SIZE(smblib_apsd_results); i++) {
		if (smblib_apsd_results[i].bit == stat)
			result = &smblib_apsd_results[i];
	}

	if (apsd_stat & QC_CHARGER_BIT) {
		/* since its a qc_charger, either return HVDCP3 or HVDCP2 */
		if (result != &smblib_apsd_results[HVDCP3])
			result = &smblib_apsd_results[HVDCP2];
	}

	return result;
}

/********************
 * REGISTER SETTERS *
 ********************/

static int chg_freq_list[] = {
	9600, 9600, 6400, 4800, 3800, 3200, 2700, 2400, 2100, 1900, 1700,
	1600, 1500, 1400, 1300, 1200,
};

int smblib_set_chg_freq(struct smb_chg_param *param,
				int val_u, u8 *val_raw)
{
	u8 i;

	if (val_u > param->max_u || val_u < param->min_u)
		return -EINVAL;

	/* Charger FSW is the configured freqency / 2 */
	val_u *= 2;
	for (i = 0; i < ARRAY_SIZE(chg_freq_list); i++) {
		if (chg_freq_list[i] == val_u)
			break;
	}
	if (i == ARRAY_SIZE(chg_freq_list)) {
		pr_err("Invalid frequency %d Hz\n", val_u / 2);
		return -EINVAL;
	}

	*val_raw = i;

	return 0;
}

static int smblib_set_opt_freq_buck(struct smb_charger *chg, int fsw_khz)
{
	union power_supply_propval pval = {0, };
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_buck, fsw_khz);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_buck rc=%d\n", rc);

	if (chg->mode == PARALLEL_MASTER && chg->pl.psy) {
		pval.intval = fsw_khz;
		/*
		 * Some parallel charging implementations may not have
		 * PROP_BUCK_FREQ property - they could be running
		 * with a fixed frequency
		 */
		power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_BUCK_FREQ, &pval);
	}

	return rc;
}

int smblib_set_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int val_u)
{
	int rc = 0;
	u8 val_raw;

	if (param->set_proc) {
		rc = param->set_proc(param, val_u, &val_raw);
		if (rc < 0)
			return -EINVAL;
	} else {
		if (val_u > param->max_u || val_u < param->min_u) {
			smblib_err(chg, "%s: %d is out of range [%d, %d]\n",
				param->name, val_u, param->min_u, param->max_u);
			return -EINVAL;
		}

		val_raw = (val_u - param->min_u) / param->step_u;
	}

	rc = smblib_write(chg, param->reg, val_raw);
	if (rc < 0) {
		smblib_err(chg, "%s: Couldn't write 0x%02x to 0x%04x rc=%d\n",
			param->name, val_raw, param->reg, rc);
		return rc;
	}

	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, val_u, val_raw);

	return rc;
}

static int step_charge_soc_update(struct smb_charger *chg, int capacity)
{
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.step_soc, capacity);
	if (rc < 0) {
		smblib_err(chg, "Error in updating soc, rc=%d\n", rc);
		return rc;
	}

	rc = smblib_write(chg, STEP_CHG_SOC_VBATT_V_UPDATE_REG,
			STEP_CHG_SOC_VBATT_V_UPDATE_BIT);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't set STEP_CHG_SOC_VBATT_V_UPDATE_REG rc=%d\n",
			rc);
		return rc;
	}

	return rc;
}

int smblib_set_usb_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT,
				 suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to USBIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

int smblib_set_dc_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DCIN_SUSPEND_BIT,
				 suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to DCIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

static int smblib_set_adapter_allowance(struct smb_charger *chg,
					u8 allowed_voltage)
{
	int rc = 0;

	switch (allowed_voltage) {
	case USBIN_ADAPTER_ALLOW_12V:
	case USBIN_ADAPTER_ALLOW_5V_OR_12V:
	case USBIN_ADAPTER_ALLOW_9V_TO_12V:
	case USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V:
	case USBIN_ADAPTER_ALLOW_5V_TO_12V:
		/* PM660 only support max. 9V */
		if (chg->smb_version == PM660_SUBTYPE) {
			smblib_dbg(chg, PR_MISC, "voltage not supported=%d\n",
					allowed_voltage);
			allowed_voltage = USBIN_ADAPTER_ALLOW_5V_OR_9V;
		}
		break;
	}

	rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG, allowed_voltage);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write 0x%02x to USBIN_ADAPTER_ALLOW_CFG rc=%d\n",
			allowed_voltage, rc);
		return rc;
	}

	return rc;
}

#define MICRO_5V	5000000
#define MICRO_9V	9000000
#define MICRO_12V	12000000
static int smblib_set_usb_pd_allowed_voltage(struct smb_charger *chg,
					int min_allowed_uv, int max_allowed_uv)
{
	int rc;
	u8 allowed_voltage;

	if (min_allowed_uv == MICRO_5V && max_allowed_uv == MICRO_5V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V;
		smblib_set_opt_freq_buck(chg, chg->chg_freq.freq_5V);
	} else if (min_allowed_uv == MICRO_9V && max_allowed_uv == MICRO_9V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_9V;
		smblib_set_opt_freq_buck(chg, chg->chg_freq.freq_9V);
	} else if (min_allowed_uv == MICRO_12V && max_allowed_uv == MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_12V;
		smblib_set_opt_freq_buck(chg, chg->chg_freq.freq_12V);
	} else if (min_allowed_uv < MICRO_9V && max_allowed_uv <= MICRO_9V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_9V;
	} else if (min_allowed_uv < MICRO_9V && max_allowed_uv <= MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_12V;
	} else if (min_allowed_uv < MICRO_12V && max_allowed_uv <= MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_9V_TO_12V;
	} else {
		smblib_err(chg, "invalid allowed voltage [%d, %d]\n",
			min_allowed_uv, max_allowed_uv);
		return -EINVAL;
	}
#ifdef CONFIG_LGE_PM
	if (chg->fake_hvdcp_mode) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_9V;
	}
#endif

	rc = smblib_set_adapter_allowance(chg, allowed_voltage);
	if (rc < 0) {
		smblib_err(chg, "Couldn't configure adapter allowance rc=%d\n",
				rc);
		return rc;
	}

	return rc;
}

/********************
 * HELPER FUNCTIONS *
 ********************/

static void smblib_rerun_apsd(struct smb_charger *chg)
{
	int rc;

	smblib_dbg(chg, PR_MISC, "re-running APSD\n");
	if (chg->wa_flags & QC_AUTH_INTERRUPT_WA_BIT) {
		rc = smblib_masked_write(chg,
				USBIN_SOURCE_CHANGE_INTRPT_ENB_REG,
				AUTH_IRQ_EN_CFG_BIT, AUTH_IRQ_EN_CFG_BIT);
		if (rc < 0)
			smblib_err(chg, "Couldn't enable HVDCP auth IRQ rc=%d\n",
									rc);
	}

	rc = smblib_masked_write(chg, CMD_APSD_REG,
				APSD_RERUN_BIT, APSD_RERUN_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't re-run APSD rc=%d\n", rc);

#ifdef CONFIG_LGE_PM
	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP ||
			chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3) {
		vote(chg->pseudo_usb_type_votable, APSD_RERUN_VOTER, true, PSEUDO_HVDCP);
		smblib_dbg(chg, PR_LGE, "pseudo_usb_type_votable set - APSD_RERUN (%d)\n", chg->pseudo_usb_type);
	}
#endif
}

static const struct apsd_result *smblib_update_usb_type(struct smb_charger *chg)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);

	/* if PD is active, APSD is disabled so won't have a valid result */
	if (chg->pd_active)
		chg->real_charger_type = POWER_SUPPLY_TYPE_USB_PD;
	else
		chg->real_charger_type = apsd_result->pst;

#ifdef CONFIG_LGE_PM
	if (chg->pd_active) {
		if (apsd_result->pst == POWER_SUPPLY_TYPE_USB
				|| apsd_result->pst == POWER_SUPPLY_TYPE_USB_CDP)
			chg->pseudo_usb_type = apsd_result->pst;
		else
			chg->pseudo_usb_type = POWER_SUPPLY_TYPE_USB_PD;
		vote(chg->pseudo_usb_type_votable, PD_VOTER, true, PSEUDO_USB_TYPE);
		vote(chg->pseudo_usb_type_votable, APSD_RERUN_VOTER, false, PSEUDO_HVDCP);
	} else {
		if (apsd_result->pst != POWER_SUPPLY_TYPE_UNKNOWN) {
			chg->pseudo_usb_type = apsd_result->pst;
		}

		if (is_client_vote_enabled_locked(chg->pseudo_usb_type_votable, PD_VOTER)) {
			vote(chg->pseudo_usb_type_votable, PD_VOTER, false, PSEUDO_USB_TYPE);
			chg->pseudo_usb_type = apsd_result->pst;
			smblib_dbg(chg, PR_LGE, "pseudo_usb_type_votable unset - PD\n");
		}
	}
#endif

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	if (chg->no_batt_boot)
		chg->real_charger_type = POWER_SUPPLY_TYPE_USB;
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
	if (chg->fake_hvdcp_mode) {
		int rc,usbin_vol;
		union power_supply_propval pval;

		rc = smblib_get_prop_usb_voltage_now(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "error get USBIN voltage rc=%d\n", rc);
			pval.intval = 0;
		}
		usbin_vol = (pval.intval) / 1000;
		if (usbin_vol >= 7000) {
			chg->real_charger_type = POWER_SUPPLY_TYPE_USB_HVDCP;
		}
	}
#endif
	smblib_dbg(chg, PR_MISC, "APSD=%s PD=%d\n",
					apsd_result->name, chg->pd_active);
	return apsd_result;
}

static int smblib_notifier_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct power_supply *psy = v;
	struct smb_charger *chg = container_of(nb, struct smb_charger, nb);

	if (!strcmp(psy->desc->name, "bms")) {
		if (!chg->bms_psy)
			chg->bms_psy = psy;
		if (ev == PSY_EVENT_PROP_CHANGED)
			schedule_work(&chg->bms_update_work);
	}

#ifdef CONFIG_LGE_PM_STEP_CHARGING
	if (!strcmp(psy->desc->name, "battery")) {
		if (!chg->batt_psy)
			chg->batt_psy = psy;
		if (ev == PSY_EVENT_PROP_CHANGED)
			schedule_work(&chg->batt_update_work);
	}
#endif
	if (!chg->pl.psy && !strcmp(psy->desc->name, "parallel"))
		chg->pl.psy = psy;

#ifdef CONFIG_IDTP9223_CHARGER
	if (!strcmp(psy->desc->name, "dc-wireless")) {
		if (!chg->wlc_psy)
			chg->wlc_psy = psy;
		if (ev == PSY_EVENT_PROP_CHANGED)
			schedule_work(&chg->idtp9223_work);
	}
#endif
	return NOTIFY_OK;
}

static int smblib_register_notifier(struct smb_charger *chg)
{
	int rc;

	chg->nb.notifier_call = smblib_notifier_call;
	rc = power_supply_reg_notifier(&chg->nb);
	if (rc < 0) {
		smblib_err(chg, "Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	return 0;
}
#ifdef CONFIG_LGE_PM_BATT_MANAGER
#define BM_MONITOR_START 1
#define BM_MONITOR_STOP 0
static void lge_set_bm_status(struct smb_charger *chg, int status){
	union power_supply_propval prop = {0, };
	chg->bm_psy = power_supply_get_by_name("battery_manager");
	if (!chg->bm_psy) {
		pr_err("cannot find battery_manager\n");
	} else {
		prop.intval = status;
		power_supply_set_property(chg->bm_psy,POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, &prop);
	}

}
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
static void lge_vzw_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			lge_vzw_work);
	union lge_power_propval lge_val = {0,};
	int rc = 0;

	if (!chg->lge_vzw_lpc)
		chg->lge_vzw_lpc = lge_power_get_by_name("lge_vzw");
	if (chg->lge_vzw_lpc) {
		rc = chg->lge_vzw_lpc->get_property(chg->lge_vzw_lpc,
				LGE_POWER_PROP_VZW_CHG, &lge_val);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get vzw_chg, rc = %d\n", rc);
			return;
		}
		if (chg->vzw_chg != lge_val.intval) {
			chg->vzw_chg = lge_val.intval;
			power_supply_changed(chg->batt_psy);
		}
	}
}
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
static void cable_detect_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						cable_detect_work);
	int rc, usb_present = 0;
	union power_supply_propval pval = {0,};
	union lge_power_propval lge_val = {0,};

	rc = smblib_get_prop_usb_present(chg, &pval);
	if (!rc)
		usb_present = pval.intval;

	if (!chg->lge_cd_lpc)
		chg->lge_cd_lpc = lge_power_get_by_name("lge_cable_detect");
	if (chg->lge_cd_lpc) {
		rc = chg->lge_cd_lpc->get_property(chg->lge_cd_lpc,
				LGE_POWER_PROP_IS_FACTORY_CABLE, &lge_val);
		chg->is_factory_cable = lge_val.intval;
		rc = chg->lge_cd_lpc->get_property(chg->lge_cd_lpc,
				LGE_POWER_PROP_IS_FACTORY_MODE_BOOT, &lge_val);
		chg->is_factory_cable |= lge_val.intval;

		rc = chg->lge_cd_lpc->get_property(chg->lge_cd_lpc,
				LGE_POWER_PROP_CURRENT_MAX, &lge_val);
		if (lge_val.intval == 0) {
			smblib_dbg(chg, PR_LGE, "Not need to current set to %d\n",
					lge_val.intval);
			power_supply_changed(chg->usb_psy);
			return;
		}
		if (rc < 0) {
			smblib_err(chg, "Couldn't get iusb current, rc = %d\n", rc);
			return;
		}
	} else {
		smblib_err(chg, "lge_cd_lpc is null!!!\n");
		return;
	}
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PARALLEL_CONTROLLER
	if (lge_val.intval <= 0) {
		lge_val.intval = 500*1000; // uA
		rc = vote(chg->usb_icl_votable, USB_PSY_VOTER,
				true, lge_val.intval);
	}
	else {
		rc = vote(chg->usb_icl_votable, USB_PSY_VOTER,
				true, lge_val.intval);
	}
#else
	if (usb_present) {
		if ((chg->is_factory_cable &&
				chg->real_charger_type != POWER_SUPPLY_TYPE_UNKNOWN) ||
				chg->real_charger_type == POWER_SUPPLY_TYPE_USB) {
			if (chg->no_batt_boot) {
				rc = vote(chg->usb_icl_votable, USB_PSY_VOTER,
						true, 1500000);
			} else {
				rc = vote(chg->usb_icl_votable, USB_PSY_VOTER,
						true, lge_val.intval);
			}
		} else if (chg->real_charger_type != POWER_SUPPLY_TYPE_UNKNOWN) {
			rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
		}
	}
#endif

	smblib_dbg(chg, PR_LGE, "present = %d, type = %d, factory cable = %d, current = %d\n",
			usb_present, chg->real_charger_type, chg->is_factory_cable, lge_val.intval);
}
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
static int lgcc_set_bat_ov_ecc(struct smb_charger *chg, bool enable)
{
	int rc;
	/* set Battery OV Ends Charge Cycle */
	rc = smblib_masked_write(chg, CHGR_CFG2_REG,
			BAT_OV_ECC_BIT, enable? BAT_OV_ECC_BIT:0);
	if (rc < 0)
		smblib_err(chg, "Couldn't enable BAT_OV_ECC rc=%d\n", rc);

	return rc;
}

#define LGCC_REASON_LCD	2
#define DC_ICL_LCD_ON_CURRENT	500
static void dc_lcd_current_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			dc_lcd_current_work.work);

	vote(chg->dc_icl_votable, THERMAL_DAEMON_VOTER,
			true, DC_ICL_LCD_ON_CURRENT*1000);
}

static int lgcc_set_ibat_current(struct smb_charger *chg,
		int chg_current) {
	int rc, chg_present, dc_present = 0;
	union power_supply_propval val = {0,};
	union lge_power_propval lge_val = {0,};

	rc = smblib_get_prop_usb_present(chg, &val);
	if (rc < 0)
		return rc;
	chg_present = val.intval;

	rc = smblib_get_prop_dc_present(chg, &val);
	if (rc < 0)
		return rc;
	dc_present = val.intval;

	if (chg_present) {
		rc = vote(chg->fcc_votable, LGE_POWER_CLASS_FCC_VOTER,
				true, chg_current*1000);
	} else if (dc_present) {
		rc = vote(chg->fcc_votable, LGE_POWER_CLASS_FCC_VOTER,
				false, chg_current*1000);
		rc = chg->lge_cc_lpc->get_property(chg->lge_cc_lpc,
				LGE_POWER_PROP_VOTE_REASON, &lge_val);
		if (lge_val.intval == LGCC_REASON_LCD) {
			if (!is_client_vote_enabled_locked(chg->dc_icl_votable, THERMAL_DAEMON_VOTER)
					&& chg->dc_fastchg)
				schedule_delayed_work(&chg->dc_lcd_current_work, msecs_to_jiffies(5000));
		} else {
			if (delayed_work_pending(&chg->dc_lcd_current_work))
				cancel_delayed_work_sync(&chg->dc_lcd_current_work);
			rc = vote(chg->dc_icl_votable, THERMAL_DAEMON_VOTER,
					false, DC_ICL_LCD_ON_CURRENT*1000);
		}
	} else {
		rc = vote(chg->fcc_votable, LGE_POWER_CLASS_FCC_VOTER,
				false, chg_current*1000);
	}
	if (rc < 0) {
		pr_err("Couldn't vote fcc setting\n");
		return rc;
	}

	return rc;
}

static int lgcc_set_charging_enable(struct smb_charger *chg,
		int enable) {
	int rc = 0;

	vote(chg->chg_disable_votable, LGCC_EN_VOTER, !enable, 0);
	if (rc < 0)
		pr_info("failed to set Charging Status \n");

	return rc;
}

static int lgcc_fv_set_charging_enable(struct smb_charger *chg,
		int enable) {
	int rc = 0;

	vote(chg->chg_disable_votable, LGE_POWER_CLASS_FV_VOTER, !enable, 0);
	if (rc < 0)
		pr_info("failed to set Charging Status \n");

	return rc;
}

static int 	lgcc_set_fake_hvdcp_mode(struct smb_charger *chg,
		int enable) {
	int rc;

	if (chg->fake_hvdcp_mode != enable) {
		chg->fake_hvdcp_mode = enable;
		rc = smblib_set_adapter_allowance(chg,
				USBIN_ADAPTER_ALLOW_5V_TO_9V);
		if (rc < 0)
			smblib_err(chg, "Couldn't set USBIN_ADAPTER_ALLOW_5V_TO_9V rc=%d\n", rc);

		smblib_update_usb_type(chg);
	}

	return 0;
}

#ifdef CONFIG_LGE_PM_INOV_GEN3_SYSFS_SUPPORT
static int 	lgcc_update_inov_temp(struct smb_charger *chg, int lcd_status) {
	union power_supply_propval val = {0,};

	if (!chg->bms_psy) {
		pr_err("don't define bms_psy\n");
		return 0;
	}

	if (chg->lcd_status != lcd_status) {
		if (lcd_status)	 {
			val.intval = chg->inov_on_temp[CHARGER_TEMP];
			power_supply_set_property(chg->bms_psy,
					POWER_SUPPLY_PROP_CHARGER_TEMP_MAX, &val);
			val.intval = chg->inov_on_temp[CHARGER_TEMP_HOT];
			power_supply_set_property(chg->bms_psy,
					POWER_SUPPLY_PROP_CHARGER_TEMP_HOT_MAX, &val);
			val.intval = chg->inov_on_temp[SKIN_TEMP];
			power_supply_set_property(chg->bms_psy,
					POWER_SUPPLY_PROP_SKIN_TEMP_MAX, &val);
			val.intval = chg->inov_on_temp[SKIN_TEMP_HOT];
			power_supply_set_property(chg->bms_psy,
					POWER_SUPPLY_PROP_SKIN_TEMP_HOT_MAX, &val);
		} else {
			val.intval = chg->inov_off_temp[CHARGER_TEMP];
			power_supply_set_property(chg->bms_psy,
					POWER_SUPPLY_PROP_CHARGER_TEMP_MAX, &val);
			val.intval = chg->inov_off_temp[CHARGER_TEMP_HOT];
			power_supply_set_property(chg->bms_psy,
					POWER_SUPPLY_PROP_CHARGER_TEMP_HOT_MAX, &val);
			val.intval = chg->inov_off_temp[SKIN_TEMP];
			power_supply_set_property(chg->bms_psy,
					POWER_SUPPLY_PROP_SKIN_TEMP_MAX, &val);
			val.intval = chg->inov_off_temp[SKIN_TEMP_HOT];
			power_supply_set_property(chg->bms_psy,
					POWER_SUPPLY_PROP_SKIN_TEMP_HOT_MAX, &val);
		}
		chg->lcd_status = lcd_status;

	}

	return 0;
}
#endif

#define CHARING_STOP_DELTA  20000
static int lgcc_set_float_voltage(struct smb_charger *chg, int float_voltage) {
	int rc, float_max, chg_present, batt_status, batt_vol = 0;
	union power_supply_propval val = {0,};
	static int is_low_float = 0;
	u8 stat;

	if (!chg->bms_psy) {
		pr_err("don't define bms_psy\n");
		return -EINVAL;
	}

	if (!chg->batt_psy) {
		pr_err("don't define batt_psy\n");
		return -EINVAL;
	}

	rc = smblib_get_prop_usb_present(chg, &val);
	if (rc < 0) {
		pr_err("Couldn't read usb present\n");
		return rc;
	}
	chg_present = val.intval;

	rc = smblib_get_prop_dc_present(chg, &val);
	if (rc < 0) {
		pr_err("Couldn't read dc present\n");
		return rc;
	}
	chg_present = (!!chg_present) || (!!val.intval);

	rc = smblib_get_prop_batt_voltage_max(chg, &val);
	if (rc < 0) {
		pr_err("Couldn't read batt voltage\n");
		return rc;
	}
	float_max = val.intval;

	if (!chg_present || float_voltage < 0) {
		rc = vote(chg->fv_votable, LGE_POWER_CLASS_FV_VOTER,
				false, float_voltage);
		if (rc < 0) {
			pr_err("Couldn't vote float setting\n");
			return rc;
		}

		if (is_low_float == 1) {
			smblib_dbg(chg, PR_LGE, "Clear low float charging mode\n");
			lgcc_set_bat_ov_ecc(chg, false);
			lgcc_fv_set_charging_enable(chg, 0);
			lgcc_fv_set_charging_enable(chg, 1);
			smblib_rerun_aicl(chg);
			is_low_float = 0;
		}
	} else {
		rc = vote(chg->fv_votable, LGE_POWER_CLASS_FV_VOTER,
				true, float_voltage);
		if (rc < 0) {
			pr_err("Couldn't vote float setting\n");
			return rc;
		}

		smblib_get_prop_batt_voltage_now(chg, &val);
		batt_vol = val.intval;
		smblib_read(chg, BATTERY_CHARGER_STATUS_6_REG, &stat);
		if (is_low_float == 0) {
			smblib_dbg(chg, PR_LGE, "Enter low float charging mode\n");
			lgcc_set_bat_ov_ecc(chg, true);
			is_low_float = 1;
			if (!(stat &GF_BATT_OV_BIT) && (batt_vol >= float_voltage - CHARING_STOP_DELTA)) {
				smblib_dbg(chg, PR_LGE, "Stop charging on low float.\n");
				lgcc_fv_set_charging_enable(chg, 0);
			}
		} else {
			smblib_get_prop_batt_status(chg, &val);
			batt_status = val.intval;
			if (batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING
					&& !(stat &GF_BATT_OV_BIT)
					&& (batt_vol <= float_voltage - CHARING_STOP_DELTA)) {
				smblib_dbg(chg, PR_LGE, "Retry charging enable on low float.\n");
				lgcc_fv_set_charging_enable(chg, 0);
				lgcc_fv_set_charging_enable(chg, 1);
				smblib_rerun_aicl(chg);
			}
		}
	}
	if (rc < 0) {
		pr_err("Couldn't set recharging voltage\n");
		return rc;
	}

	return rc;
}

static void lge_cc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						lge_cc_work);
	int rc = 0;
	int ibat, vfloat, charging_enabled, fake_hvdcp_mode = 0;

	union lge_power_propval lge_val = {0,};

	if (!chg->lge_cc_lpc)
		chg->lge_cc_lpc = lge_power_get_by_name("lge_cc");
	if (chg->lge_cc_lpc){
		rc = chg->lge_cc_lpc->get_property(chg->lge_cc_lpc,
				LGE_POWER_PROP_BTM_STATE, &lge_val);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get btm state, rc = %d\n", rc);
			return;
		}
		chg->btm_state = lge_val.intval;

		rc = chg->lge_cc_lpc->get_property(chg->lge_cc_lpc,
				LGE_POWER_PROP_OTP_CURRENT, &lge_val);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get ibat current, rc = %d\n", rc);
			return;
		}
		lgcc_set_ibat_current(chg, lge_val.intval);
		ibat = lge_val.intval;

		rc = chg->lge_cc_lpc->get_property(chg->lge_cc_lpc,
				LGE_POWER_PROP_CHARGING_ENABLED, &lge_val);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get charging enable, rc = %d\n", rc);
			return;
		}
		lgcc_set_charging_enable(chg, lge_val.intval);
		charging_enabled = lge_val.intval;

		rc = chg->lge_cc_lpc->get_property(chg->lge_cc_lpc,
				LGE_POWER_PROP_OTP_FLOAT_VOLTAGE, &lge_val);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get float voltage, rc = %d\n", rc);
			return;
		}
		lgcc_set_float_voltage(chg, lge_val.intval);
		vfloat = lge_val.intval;

		rc = chg->lge_cc_lpc->get_property(chg->lge_cc_lpc,
				LGE_POWER_PROP_PSEUDO_BATT_UI, &lge_val);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get pseudo ui, rc = %d\n", rc);
			return;
		}
		chg->pseudo_chg_ui = lge_val.intval;

		rc = chg->lge_cc_lpc->get_property(chg->lge_cc_lpc,
				LGE_POWER_PROP_HVDCP_FAKE_MODE, &lge_val);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get hvdcp fake mode, rc = %d\n", rc);
			return;
		}
		lgcc_set_fake_hvdcp_mode(chg, lge_val.intval);
		fake_hvdcp_mode = lge_val.intval;

#ifdef CONFIG_LGE_PM_INOV_GEN3_SYSFS_SUPPORT
		rc = chg->lge_cc_lpc->get_property(chg->lge_cc_lpc,
				LGE_POWER_PROP_LCD_STATUS, &lge_val);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get lcd status, rc = %d\n", rc);
			return;
		}
		lgcc_update_inov_temp(chg, lge_val.intval);
#endif

		pr_info("lge_smblib_cc_work btm[%d], ibat[%d], vfloat[%d], pseudo_chg_ui[%d], charging_enabled[%d] fake_hvdcp_mode[%d] from cc.\n",
			chg->btm_state, ibat, vfloat/1000, chg->pseudo_chg_ui, charging_enabled, fake_hvdcp_mode);
	}
}
#endif

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PARALLEL_CONTROLLER
static void lge_pl_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			lge_pl_work);
	int rc, pl_iusb, pl_ibat, usb_present = 0;
	union power_supply_propval pval = {0,};
	union lge_power_propval lge_val = {0,};

	if (!chg->lge_pl_lpc)
		chg->lge_pl_lpc = lge_power_get_by_name("lge_pl_ctrl");
	if (chg->lge_pl_lpc) {
		rc = chg->lge_pl_lpc->get_property(chg->lge_pl_lpc,
				LGE_POWER_PROP_PARALLEL_CHARGING_ENABLED, &lge_val);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get parallel status, rc = %d\n", rc);
			return;
		}
		chg->lge_pl_status = lge_val.intval;
		if (chg->lge_pl_status) {
			chg->lge_pl_lpc->get_property(chg->lge_pl_lpc,
					LGE_POWER_PROP_MASTER_CURRENT_MAX, &lge_val);
			if (rc < 0) {
				smblib_err(chg, "Couldn't get parallel current, rc = %d\n", rc);
				return;
			}
			pl_iusb = lge_val.intval;
			chg->lge_pl_lpc->get_property(chg->lge_pl_lpc,
					LGE_POWER_PROP_MASTER_CONSTANT_CHARGE_CURRENT_MAX, &lge_val);
			if (rc < 0) {
				smblib_err(chg, "Couldn't get parallel current, rc = %d\n", rc);
				return;
			}
			pl_ibat = lge_val.intval;
			smblib_err(chg, "pl iusb = %d, ibat = %d\n", pl_iusb, pl_ibat);
			usb_present = smblib_get_prop_usb_online(chg, &pval);
			if (!rc)
				usb_present = pval.intval;

			if (usb_present) {
				rc = smblib_set_charge_param(chg, &chg->param.usb_icl, pl_iusb);
				if (rc < 0)
					smblib_err(chg, "Couldn't set mater pl iusb rc=%d\n", rc);

				rc = smblib_set_charge_param(chg, &chg->param.fcc, pl_ibat);
				if (rc < 0)
					smblib_err(chg, "Couldn't set master pl ibat rc=%d\n", rc);
			}
		}
	}
}
#endif

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PSEUDO_BATTERY
static void lge_pseudo_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			lge_pseudo_work);
	union lge_power_propval lge_val = {0,};
	int rc = 0;

	if (!chg->lge_pseudo_lpc)
		chg->lge_pseudo_lpc = lge_power_get_by_name("pseudo_battery");
	if (chg->lge_pseudo_lpc) {
		rc = chg->lge_pseudo_lpc->get_property(chg->lge_pseudo_lpc,
				LGE_POWER_PROP_PSEUDO_BATT, &lge_val);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get pseudo batt status, rc = %d\n", rc);
			return;
		}
		chg->pseudo_batt = (bool)lge_val.intval;
		power_supply_changed(chg->batt_psy);
	}
}
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_STORE_MODE
static void lge_sm_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			lge_sm_work);
	union lge_power_propval lge_val = {0,};
	int rc = 0;

	if (!chg->lge_sm_lpc)
		chg->lge_sm_lpc = lge_power_get_by_name("lge_sm");
	if (chg->lge_sm_lpc) {
		rc = chg->lge_sm_lpc->get_property(chg->lge_sm_lpc,
				LGE_POWER_PROP_STORE_DEMO_ENABLED, &lge_val);
		if (rc >= 0 && lge_val.intval) {
			rc = chg->lge_sm_lpc->get_property(chg->lge_sm_lpc,
					LGE_POWER_PROP_USB_CHARGING_ENABLED, &lge_val);
			vote(chg->usb_icl_votable, LGSM_EN_VOTER,
					!lge_val.intval, 0);
			rc = chg->lge_sm_lpc->get_property(chg->lge_sm_lpc,
					LGE_POWER_PROP_CHARGING_ENABLED, &lge_val);
			vote(chg->chg_disable_votable, LGSM_EN_VOTER,
					!lge_val.intval, 0);
		} else {
			vote(chg->usb_icl_votable, LGSM_EN_VOTER, 0, 0);
			vote(chg->chg_disable_votable, LGSM_EN_VOTER, 0, 0);
		}
	}
}
#endif
#ifdef CONFIG_LGE_PM_SUPPORT_LG_POWER_CLASS
static int lge_smblib_notifier_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct lge_power *lpc = v;
	struct smb_charger *chg = container_of(nb, struct smb_charger, lge_nb);

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	if (!strcmp(lpc->name, "lge_cable_detect")) {
		pr_info("chg->fake_capacity is %d\n", chg->fake_capacity);
		schedule_work(&chg->cable_detect_work);
	}
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
	if (!strcmp(lpc->name, "lge_cc")) {
		schedule_work(&chg->lge_cc_work);
	}
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PARALLEL_CONTROLLER
	if (!strcmp(lpc->name, "lge_pl_ctrl")) {
		schedule_work(&chg->lge_pl_work);
	}
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PSEUDO_BATTERY
	if (!strcmp(lpc->name, "pseudo_battery")) {
		schedule_work(&chg->lge_pseudo_work);
	}
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
	if (!strcmp(lpc->name, "lge_vzw")) {
		schedule_work(&chg->lge_vzw_work);
	}
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_STORE_MODE
	if (!strcmp(lpc->name, "lge_sm")) {
		schedule_work(&chg->lge_sm_work);
	}
#endif
	return NOTIFY_OK;
}

static int lge_smblib_register_notifier(struct smb_charger *chg)
{
	int rc;

	chg->lge_nb.notifier_call = lge_smblib_notifier_call;
	rc = lge_power_reg_notifier(&chg->lge_nb);
	if (rc < 0) {
		smblib_err(chg, "Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	return 0;
}
#endif

int smblib_mapping_soc_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u)
		return -EINVAL;

	*val_raw = val_u << 1;

	return 0;
}

int smblib_mapping_cc_delta_to_field_value(struct smb_chg_param *param,
					   u8 val_raw)
{
	int val_u  = val_raw * param->step_u + param->min_u;

	if (val_u > param->max_u)
		val_u -= param->max_u * 2;

	return val_u;
}

int smblib_mapping_cc_delta_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u - param->max_u)
		return -EINVAL;

	val_u += param->max_u * 2 - param->min_u;
	val_u %= param->max_u * 2;
	*val_raw = val_u / param->step_u;

	return 0;
}

static void smblib_uusb_removal(struct smb_charger *chg)
{
	int rc;

	cancel_delayed_work_sync(&chg->pl_enable_work);
	vote(chg->pl_disable_votable, PL_DELAY_VOTER, true, 0);
	vote(chg->awake_votable, PL_DELAY_VOTER, false, 0);

	/* reset both usbin current and voltage votes */
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, false, 0);
	vote(chg->usb_icl_votable, SW_QC3_VOTER, false, 0);

	cancel_delayed_work_sync(&chg->hvdcp_detect_work);

	if (chg->wa_flags & QC_AUTH_INTERRUPT_WA_BIT) {
		/* re-enable AUTH_IRQ_EN_CFG_BIT */
		rc = smblib_masked_write(chg,
				USBIN_SOURCE_CHANGE_INTRPT_ENB_REG,
				AUTH_IRQ_EN_CFG_BIT, AUTH_IRQ_EN_CFG_BIT);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't enable QC auth setting rc=%d\n", rc);
	}

	/* reconfigure allowed voltage for HVDCP */
	rc = smblib_set_adapter_allowance(chg,
			USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V);
	if (rc < 0)
		smblib_err(chg, "Couldn't set USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V rc=%d\n",
			rc);

	chg->voltage_min_uv = MICRO_5V;
	chg->voltage_max_uv = MICRO_5V;
	chg->usb_icl_delta_ua = 0;
	chg->pulse_cnt = 0;
	chg->uusb_apsd_rerun_done = false;

	/* clear USB ICL vote for USB_PSY_VOTER */
	rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't un-vote for USB ICL rc=%d\n", rc);

	/* clear USB ICL vote for DCP_VOTER */
	rc = vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg,
			"Couldn't un-vote DCP from USB ICL rc=%d\n", rc);
}

void smblib_suspend_on_debug_battery(struct smb_charger *chg)
{
	int rc;
	union power_supply_propval val;

	if (!chg->suspend_input_on_debug_batt)
		return;

	rc = power_supply_get_property(chg->bms_psy,
			POWER_SUPPLY_PROP_DEBUG_BATTERY, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get debug battery prop rc=%d\n", rc);
		return;
	}

	vote(chg->usb_icl_votable, DEBUG_BOARD_VOTER, val.intval, 0);
	vote(chg->dc_suspend_votable, DEBUG_BOARD_VOTER, val.intval, 0);
	if (val.intval)
		pr_info("Input suspended: Fake battery\n");
}

int smblib_rerun_apsd_if_required(struct smb_charger *chg)
{
	union power_supply_propval val;
	int rc;

	rc = smblib_get_prop_usb_present(chg, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb present rc = %d\n", rc);
		return rc;
	}

	if (!val.intval)
		return 0;

	/* fetch the DPDM regulator */
	if (!chg->dpdm_reg && of_get_property(chg->dev->of_node,
						"dpdm-supply", NULL)) {
		chg->dpdm_reg = devm_regulator_get(chg->dev, "dpdm");
		if (IS_ERR(chg->dpdm_reg)) {
			smblib_err(chg, "Couldn't get dpdm regulator rc=%ld\n",
				PTR_ERR(chg->dpdm_reg));
			chg->dpdm_reg = NULL;
		}
	}

	if (chg->dpdm_reg && !regulator_is_enabled(chg->dpdm_reg)) {
		smblib_dbg(chg, PR_MISC, "enabling DPDM regulator\n");
		rc = regulator_enable(chg->dpdm_reg);
	if (rc < 0)
			smblib_err(chg, "Couldn't enable dpdm regulator rc=%d\n",
				rc);
	}

	chg->uusb_apsd_rerun_done = true;
	smblib_rerun_apsd(chg);

	return 0;
}

static int smblib_get_pulse_cnt(struct smb_charger *chg, int *count)
{
	int rc;
	u8 val[2];

	switch (chg->smb_version) {
	case PMI8998_SUBTYPE:
		rc = smblib_read(chg, QC_PULSE_COUNT_STATUS_REG, val);
		if (rc) {
			pr_err("failed to read QC_PULSE_COUNT_STATUS_REG rc=%d\n",
					rc);
			return rc;
		}
		*count = val[0] & QC_PULSE_COUNT_MASK;
		break;
	case PM660_SUBTYPE:
		rc = smblib_multibyte_read(chg,
				QC_PULSE_COUNT_STATUS_1_REG, val, 2);
		if (rc) {
			pr_err("failed to read QC_PULSE_COUNT_STATUS_1_REG rc=%d\n",
					rc);
			return rc;
		}
		*count = (val[1] << 8) | val[0];
		break;
	default:
		smblib_dbg(chg, PR_PARALLEL, "unknown SMB chip %d\n",
				chg->smb_version);
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_LGE_PM_STEP_CHARGING
#define RETRY_CHECK_STEP_CHARGING_MS   5000
void smblib_update_step_charging(struct smb_charger *chg)
{
	if (chg->sc_level == 0 || chg->sc_level == -1 ) {
		vote(chg->fcc_votable, LGE_STEP_CHARGING_VOTER, false, 0);
		smblib_err(chg, "disable step charging.\n");
	} else {
		vote(chg->fcc_votable, LGE_STEP_CHARGING_VOTER, true, chg->sc_cur_delta[chg->sc_level-1]);
		smblib_err(chg, "Enable step charging. step[%d] = %d.\n",
			chg->sc_level-1, chg->sc_cur_delta[chg->sc_level-1]);
	}
}

static void smblib_step_charging_check_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
					       step_charging_check_work.work);
	int rc, i, is_step_up = 0;
	union power_supply_propval val;
	int usb_present, bat_volt, chg_state, charge_type;
	int sc_size_temp = chg->sc_size;

	if (!chg->sc_enable)
		sc_size_temp = 1;

	if (!chg->bms_psy) {
		smblib_err(chg, "bms_psy don't defined\n");
		return;
	}

	rc = power_supply_get_property(chg->bms_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get debug battery prop rc=%d\n", rc);
		return;
	}
	bat_volt = val.intval;

	rc = smblib_get_prop_usb_present(chg, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb present rc=%d\n", rc);
		return;
	}
	usb_present = val.intval;

	if (is_client_vote_enabled(chg->fv_votable, LGE_POWER_CLASS_FV_VOTER)) {
		smblib_dbg(chg, PR_MISC, "Skip step charging, "	\
								 "because float voltage is limited.\n");
		return;
	}

	if (chg->sc_level == -1 && usb_present) {
		for (i =0; i < sc_size_temp; i++) {
			if (bat_volt < chg->sc_volt_thre[i]) {
				chg->sc_level = (i==0?i: i-1);
				break;
			}
		}
	}

	rc = smblib_get_prop_batt_charge_type(chg, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get charge type rc=%d\n", rc);
		return;
	}
	charge_type = val.intval;

	rc = smblib_get_prop_batt_status(chg, &val);
	if (rc < 0)
		smblib_err(chg, "Couldn't get CHG state rc=%d\n", rc);
	chg_state = val.intval;

	if (usb_present)
		is_step_up = ((bat_volt >=chg->sc_volt_thre[chg->sc_level])
				||charge_type == POWER_SUPPLY_CHARGE_TYPE_TAPER);

	smblib_err(chg, "update volt=%d, state=%d, level=%d/%d, en=%d \n",
		bat_volt, charge_type, chg->sc_level, sc_size_temp, chg->sc_enable);

	if (is_step_up && chg->sc_level < sc_size_temp) {
		chg->sc_level++;
		smblib_update_step_charging(chg);
		schedule_delayed_work(&chg->step_charging_check_work,
					  msecs_to_jiffies(RETRY_CHECK_STEP_CHARGING_MS));
	} else if (!usb_present && chg->sc_level != -1) {
		chg->sc_level = -1;
		smblib_update_step_charging(chg);
	} else if ((chg_state == POWER_SUPPLY_STATUS_DISCHARGING
				|| chg_state == POWER_SUPPLY_STATUS_NOT_CHARGING ) &&
				 chg->sc_volt_thre[0] > bat_volt && chg->sc_level != -1) {
		chg->sc_level = -1;
		smblib_update_step_charging(chg);
	}
}
#endif

#ifdef CONFIG_LGE_PM
int smblib_update_icl_override(struct smb_charger *chg) {
	int rc = 0;
	u8 stat;

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_RESULT_STATUS rc=%d\n", rc);
		return rc;
	}

	if (!(stat & ICL_OVERRIDE_LATCH_BIT)) {
		rc = smblib_masked_write(chg, CMD_APSD_REG, ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable control ICL rc=%d\n", rc);
			return rc;
		}
	}

	return rc;
}
#endif

#define USBIN_25MA	25000
#define USBIN_100MA	100000
#define USBIN_150MA	150000
#define USBIN_500MA	500000
#define USBIN_900MA	900000
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
#define USBIN_1500MA	1500000
#endif

static int set_sdp_current(struct smb_charger *chg, int icl_ua)
{
	int rc;
	u8 icl_options;

	/* power source is SDP */
	switch (icl_ua) {
	case USBIN_100MA:
		/* USB 2.0 100mA */
		icl_options = 0;
		break;
	case USBIN_150MA:
		/* USB 3.0 150mA */
		icl_options = CFG_USB3P0_SEL_BIT;
		break;
	case USBIN_500MA:
		/* USB 2.0 500mA */
		icl_options = USB51_MODE_BIT;
		break;
	case USBIN_900MA:
		/* USB 3.0 900mA */
		icl_options = CFG_USB3P0_SEL_BIT | USB51_MODE_BIT;
		break;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	case USBIN_1500MA:
		// 0x3C: USBIN_IL_1500MA
		rc = smblib_masked_write(chg, USBIN_CURRENT_LIMIT_CFG_REG,
				USBIN_CURRENT_LIMIT_MASK, (60 & USBIN_CURRENT_LIMIT_MASK));
		rc |= smblib_update_icl_override(chg);
		rc |= smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
				USBIN_MODE_CHG_BIT, USBIN_MODE_CHG_BIT);
		if (rc < 0)
			smblib_err(chg, "USBIN_1500MA masked Fail rc=%d\n", rc);
		icl_options = CFG_USB3P0_SEL_BIT | USB51_MODE_BIT;
		break;
#endif
	default:
		smblib_err(chg, "ICL %duA isn't supported for SDP\n", icl_ua);
		return -EINVAL;
	}

	rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
		CFG_USB3P0_SEL_BIT | USB51_MODE_BIT, icl_options);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL options rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int get_sdp_current(struct smb_charger *chg, int *icl_ua)
{
	int rc;
	u8 icl_options;
	bool usb3 = false;

	rc = smblib_read(chg, USBIN_ICL_OPTIONS_REG, &icl_options);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get ICL options rc=%d\n", rc);
		return rc;
	}

	usb3 = (icl_options & CFG_USB3P0_SEL_BIT);

	if (icl_options & USB51_MODE_BIT)
		*icl_ua = usb3 ? USBIN_900MA : USBIN_500MA;
	else
		*icl_ua = usb3 ? USBIN_150MA : USBIN_100MA;

	return rc;
}

int smblib_set_icl_current(struct smb_charger *chg, int icl_ua)
{
	int rc = 0;
	bool override;

#ifdef CONFIG_LGE_PM
	disable_irq_nosync(chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq);

	/* suspend and return if 25mA or less is requested */
	if (icl_ua < USBIN_25MA) {
		rc = smblib_set_usb_suspend(chg, true);
		if (rc < 0)
			goto enable_icl_changed_interrupt;
		return rc;
	}

	if (icl_ua == INT_MAX)
		goto override_suspend_config;

	smblib_err(chg, "type = %d, icl_ua = %d\n",
				chg->real_charger_type, icl_ua);

	/* To support 910K + No battery boot for factory */
	if (chg->no_batt_boot) {
		smblib_dbg(chg, PR_LGE, "icl forced set to 1500mA\n");
		icl_ua = 1500000;
	} else if (chg->is_factory_cable) {
		rc = set_sdp_current(chg, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set SDP ICL rc=%d\n", rc);
			goto enable_icl_changed_interrupt;
		}
	}
#else
	/* suspend and return if 25mA or less is requested */
	if (icl_ua < USBIN_25MA)
		return smblib_set_usb_suspend(chg, true);

	disable_irq_nosync(chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq);
	if (icl_ua == INT_MAX)
		goto override_suspend_config;
#endif

	/* configure current */
	if (chg->typec_mode == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT
		&& (chg->real_charger_type == POWER_SUPPLY_TYPE_USB)) {
		rc = set_sdp_current(chg, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set SDP ICL rc=%d\n", rc);
			goto enable_icl_changed_interrupt;
		}
	} else {
		set_sdp_current(chg, 100000);
		rc = smblib_set_charge_param(chg, &chg->param.usb_icl, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set HC ICL rc=%d\n", rc);
			goto enable_icl_changed_interrupt;
		}
	}

override_suspend_config:
	/* determine if override needs to be enforced */
	override = true;
	if (icl_ua == INT_MAX) {
		/* remove override if no voters - hw defaults is desired */
		override = false;
	} else if (chg->typec_mode == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT) {
		if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB)
			/* For std cable with type = SDP never override */
			override = false;
		else if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_CDP
			&& icl_ua == 1500000)
			/*
			 * For std cable with type = CDP override only if
			 * current is not 1500mA
			 */
			override = false;
	}

	/* enforce override */
	rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
		USBIN_MODE_CHG_BIT, override ? USBIN_MODE_CHG_BIT : 0);

	rc = smblib_icl_override(chg, override);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL override rc=%d\n", rc);
		goto enable_icl_changed_interrupt;
	}

	/* unsuspend after configuring current and override */
	rc = smblib_set_usb_suspend(chg, false);
	if (rc < 0) {
		smblib_err(chg, "Couldn't resume input rc=%d\n", rc);
		goto enable_icl_changed_interrupt;
	}

enable_icl_changed_interrupt:
	enable_irq(chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq);
	return rc;
}

int smblib_get_icl_current(struct smb_charger *chg, int *icl_ua)
{
	int rc = 0;
	u8 load_cfg;
	bool override;

	if ((chg->typec_mode == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT
		|| chg->micro_usb_mode)
#ifdef CONFIG_LGE_PM
		&& (chg->real_charger_type == POWER_SUPPLY_TYPE_USB)) {
#else
		&& (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB)) {
#endif
		rc = get_sdp_current(chg, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get SDP ICL rc=%d\n", rc);
			return rc;
		}
	} else {
		rc = smblib_read(chg, USBIN_LOAD_CFG_REG, &load_cfg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get load cfg rc=%d\n", rc);
			return rc;
		}
		override = load_cfg & ICL_OVERRIDE_AFTER_APSD_BIT;
		if (!override)
			return INT_MAX;

		/* override is set */
		rc = smblib_get_charge_param(chg, &chg->param.usb_icl, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get HC ICL rc=%d\n", rc);
			return rc;
		}
	}

	return 0;
}

/*********************
 * VOTABLE CALLBACKS *
 *********************/

static int smblib_dc_suspend_vote_callback(struct votable *votable, void *data,
			int suspend, const char *client)
{
	struct smb_charger *chg = data;

	/* resume input if suspend is invalid */
	if (suspend < 0)
		suspend = 0;

	return smblib_set_dc_suspend(chg, (bool)suspend);
}

static int smblib_dc_icl_vote_callback(struct votable *votable, void *data,
			int icl_ua, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;
	bool suspend;

	if (icl_ua < 0) {
		smblib_dbg(chg, PR_MISC, "No Voter hence suspending\n");
		icl_ua = 0;
	}

	suspend = (icl_ua < USBIN_25MA);
	if (suspend)
		goto suspend;

	rc = smblib_set_charge_param(chg, &chg->param.dc_icl, icl_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set DC input current limit rc=%d\n",
			rc);
		return rc;
	}

suspend:
	rc = vote(chg->dc_suspend_votable, USER_VOTER, suspend, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s DC rc=%d\n",
			suspend ? "suspend" : "resume", rc);
		return rc;
	}
	return rc;
}

static int smblib_pd_disallowed_votable_indirect_callback(
	struct votable *votable, void *data, int disallowed, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	rc = vote(chg->pd_allowed_votable, PD_DISALLOWED_INDIRECT_VOTER,
		!disallowed, 0);

	return rc;
}

static int smblib_awake_vote_callback(struct votable *votable, void *data,
			int awake, const char *client)
{
	struct smb_charger *chg = data;

	if (awake)
		pm_stay_awake(chg->dev);
	else
		pm_relax(chg->dev);

	return 0;
}

static int smblib_chg_disable_vote_callback(struct votable *votable, void *data,
			int chg_disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,
				 CHARGING_ENABLE_CMD_BIT,
				 chg_disable ? 0 : CHARGING_ENABLE_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s charging rc=%d\n",
			chg_disable ? "disable" : "enable", rc);
		return rc;
	}

	return 0;
}

static int smblib_hvdcp_enable_vote_callback(struct votable *votable,
			void *data,
			int hvdcp_enable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;
	u8 val = HVDCP_AUTH_ALG_EN_CFG_BIT | HVDCP_EN_BIT;
	u8 stat;

	/* vote to enable/disable HW autonomous INOV */
	vote(chg->hvdcp_hw_inov_dis_votable, client, !hvdcp_enable, 0);

	/*
	 * Disable the autonomous bit and auth bit for disabling hvdcp.
	 * This ensures only qc 2.0 detection runs but no vbus
	 * negotiation happens.
	 */
	if (!hvdcp_enable)
		val = HVDCP_EN_BIT;

	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				 HVDCP_EN_BIT | HVDCP_AUTH_ALG_EN_CFG_BIT,
				 val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s hvdcp rc=%d\n",
			hvdcp_enable ? "enable" : "disable", rc);
		return rc;
	}

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD status rc=%d\n", rc);
		return rc;
	}

	/* re-run APSD if HVDCP was detected */
	if (stat & QC_CHARGER_BIT)
		smblib_rerun_apsd(chg);

	return 0;
}

static int smblib_hvdcp_disable_indirect_vote_callback(struct votable *votable,
			void *data, int hvdcp_disable, const char *client)
{
	struct smb_charger *chg = data;

	vote(chg->hvdcp_enable_votable, HVDCP_INDIRECT_VOTER,
			!hvdcp_disable, 0);

	return 0;
}

static int smblib_apsd_disable_vote_callback(struct votable *votable,
			void *data,
			int apsd_disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	if (apsd_disable) {
		rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
							AUTO_SRC_DETECT_BIT,
							0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable APSD rc=%d\n", rc);
			return rc;
		}
	} else {
		rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
							AUTO_SRC_DETECT_BIT,
							AUTO_SRC_DETECT_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable APSD rc=%d\n", rc);
			return rc;
		}
	}

	return 0;
}

static int smblib_hvdcp_hw_inov_dis_vote_callback(struct votable *votable,
				void *data, int disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	if (disable) {
		/*
		 * the pulse count register get zeroed when autonomous mode is
		 * disabled. Track that in variables before disabling
		 */
		rc = smblib_get_pulse_cnt(chg, &chg->pulse_cnt);
		if (rc < 0) {
			pr_err("failed to read QC_PULSE_COUNT_STATUS_REG rc=%d\n",
					rc);
			return rc;
		}
	}

	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
			HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT,
			disable ? 0 : HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s hvdcp rc=%d\n",
				disable ? "disable" : "enable", rc);
		return rc;
	}

	return rc;
}

static int smblib_usb_irq_enable_vote_callback(struct votable *votable,
				void *data, int enable, const char *client)
{
	struct smb_charger *chg = data;

	if (!chg->irq_info[INPUT_CURRENT_LIMIT_IRQ].irq ||
				!chg->irq_info[HIGH_DUTY_CYCLE_IRQ].irq)
		return 0;

	if (enable) {
		enable_irq(chg->irq_info[INPUT_CURRENT_LIMIT_IRQ].irq);
		enable_irq(chg->irq_info[HIGH_DUTY_CYCLE_IRQ].irq);
	} else {
		disable_irq(chg->irq_info[INPUT_CURRENT_LIMIT_IRQ].irq);
		disable_irq(chg->irq_info[HIGH_DUTY_CYCLE_IRQ].irq);
	}

	return 0;
}

static int smblib_typec_irq_disable_vote_callback(struct votable *votable,
				void *data, int disable, const char *client)
{
	struct smb_charger *chg = data;

	if (!chg->irq_info[TYPE_C_CHANGE_IRQ].irq)
		return 0;

	if (disable)
		disable_irq_nosync(chg->irq_info[TYPE_C_CHANGE_IRQ].irq);
	else
		enable_irq(chg->irq_info[TYPE_C_CHANGE_IRQ].irq);

	return 0;
}

/*******************
 * VCONN REGULATOR *
 * *****************/

#define MAX_OTG_SS_TRIES 2
static int _smblib_vconn_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;
	u8 val;
#ifdef CONFIG_LGE_USB
	rc = smblib_masked_write(chg, OTG_CFG_REG,
		QUICKSTART_OTG_FASTROLESWAP_BIT, QUICKSTART_OTG_FASTROLESWAP_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't disable VBUS < 1V check rc=%d\n", rc);
	printk("USB [%s]: chg->vconn_en =%d\n", __func__, chg->vconn_en);
#endif

	/*
	 * When enabling VCONN using the command register the CC pin must be
	 * selected. VCONN should be supplied to the inactive CC pin hence using
	 * the opposite of the CC_ORIENTATION_BIT.
	 */
	smblib_dbg(chg, PR_OTG, "enabling VCONN\n");
	val = chg->typec_status[3] &
			CC_ORIENTATION_BIT ? 0 : VCONN_EN_ORIENTATION_BIT;
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_VALUE_BIT | VCONN_EN_ORIENTATION_BIT,
				 VCONN_EN_VALUE_BIT | val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable vconn setting rc=%d\n", rc);
		return rc;
	}

	return rc;
}

int smblib_vconn_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chg->vconn_oc_lock);
	if (chg->vconn_en)
		goto unlock;

	rc = _smblib_vconn_regulator_enable(rdev);
	if (rc >= 0)
		chg->vconn_en = true;

unlock:
	mutex_unlock(&chg->vconn_oc_lock);
	return rc;
}

static int _smblib_vconn_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	smblib_dbg(chg, PR_OTG, "disabling VCONN\n");
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_VALUE_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't disable vconn regulator rc=%d\n", rc);

	return rc;
}

int smblib_vconn_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chg->vconn_oc_lock);
	if (!chg->vconn_en)
		goto unlock;

	rc = _smblib_vconn_regulator_disable(rdev);
	if (rc >= 0)
		chg->vconn_en = false;

unlock:
	mutex_unlock(&chg->vconn_oc_lock);
	return rc;
}

int smblib_vconn_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&chg->vconn_oc_lock);
	ret = chg->vconn_en;
	mutex_unlock(&chg->vconn_oc_lock);
	return ret;
}

/*****************
 * OTG REGULATOR *
 *****************/
#define MAX_RETRY		15
#define MIN_DELAY_US		2000
#define MAX_DELAY_US		9000
static int _smblib_vbus_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc, retry_count = 0, min_delay = MIN_DELAY_US;
	u8 stat;

	smblib_dbg(chg, PR_OTG, "halt 1 in 8 mode\n");
	rc = smblib_masked_write(chg, OTG_ENG_OTG_CFG_REG,
				 ENG_BUCKBOOST_HALT1_8_MODE_BIT,
				 ENG_BUCKBOOST_HALT1_8_MODE_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set OTG_ENG_OTG_CFG_REG rc=%d\n",
			rc);
		return rc;
	}

	smblib_dbg(chg, PR_OTG, "enabling OTG\n");
	rc = smblib_write(chg, CMD_OTG_REG, OTG_EN_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable OTG regulator rc=%d\n", rc);
		return rc;
	}

	if (chg->wa_flags & OTG_WA) {
		/* check for softstart */
		do {
			usleep_range(min_delay, min_delay + 100);
			rc = smblib_read(chg, OTG_STATUS_REG, &stat);
			if (rc < 0) {
				smblib_err(chg,
					"Couldn't read OTG status rc=%d\n",
					rc);
				goto out;
			}

			if (stat & BOOST_SOFTSTART_DONE_BIT) {
				rc = smblib_set_charge_param(chg,
					&chg->param.otg_cl, chg->otg_cl_ua);
				if (rc < 0)
					smblib_err(chg,
						"Couldn't set otg limit\n");
				break;
			}

			/* increase the delay for following iterations */
			if (retry_count > 5)
				min_delay = MAX_DELAY_US;
		} while (retry_count++ < MAX_RETRY);

		if (retry_count >= MAX_RETRY) {
			smblib_dbg(chg, PR_OTG, "Boost Softstart not done\n");
			goto out;
		}
	}

	return 0;
out:
	/* disable OTG if softstart failed */
	smblib_write(chg, CMD_OTG_REG, 0);
	return rc;
}

int smblib_vbus_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chg->otg_oc_lock);
	if (chg->otg_en)
		goto unlock;

	if (!chg->usb_icl_votable) {
		chg->usb_icl_votable = find_votable("USB_ICL");

#ifdef CONFIG_LGE_USB
		if (!chg->usb_icl_votable) {
			rc = -EINVAL;
			goto unlock;
		}
#else
		if (!chg->usb_icl_votable)
			return -EINVAL;
#endif
	}
	vote(chg->usb_icl_votable, USBIN_USBIN_BOOST_VOTER, true, 0);

	rc = _smblib_vbus_regulator_enable(rdev);
	if (rc >= 0)
		chg->otg_en = true;

unlock:
	mutex_unlock(&chg->otg_oc_lock);
	return rc;
}

#ifdef CONFIG_LGE_PM
static void smblib_usb_typec_change(struct smb_charger *chg);
#endif
static int _smblib_vbus_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc;

	if (chg->wa_flags & OTG_WA) {
		/* set OTG current limit to minimum value */
		rc = smblib_set_charge_param(chg, &chg->param.otg_cl,
						chg->param.otg_cl.min_u);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't set otg current limit rc=%d\n", rc);
			return rc;
		}
	}

	smblib_dbg(chg, PR_OTG, "disabling OTG\n");
	rc = smblib_write(chg, CMD_OTG_REG, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable OTG regulator rc=%d\n", rc);
		return rc;
	}

	smblib_dbg(chg, PR_OTG, "start 1 in 8 mode\n");
	rc = smblib_masked_write(chg, OTG_ENG_OTG_CFG_REG,
				 ENG_BUCKBOOST_HALT1_8_MODE_BIT, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set OTG_ENG_OTG_CFG_REG rc=%d\n", rc);
		return rc;
	}

#ifdef CONFIG_LGE_PM
	smblib_usb_typec_change(chg);
#endif

	return 0;
}

int smblib_vbus_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chg->otg_oc_lock);
	if (!chg->otg_en)
		goto unlock;

	rc = _smblib_vbus_regulator_disable(rdev);
	if (rc >= 0)
		chg->otg_en = false;

	if (chg->usb_icl_votable)
		vote(chg->usb_icl_votable, USBIN_USBIN_BOOST_VOTER, false, 0);
unlock:
#ifdef CONFIG_LGE_PM
	if (chg->usb_icl_votable && is_client_vote_enabled_locked(
				chg->usb_icl_votable, "USBIN_USBIN_BOOST_VOTER"))
		vote(chg->usb_icl_votable, USBIN_USBIN_BOOST_VOTER, false, 0);
#endif
	mutex_unlock(&chg->otg_oc_lock);
	return rc;
}

int smblib_vbus_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&chg->otg_oc_lock);
	ret = chg->otg_en;
	mutex_unlock(&chg->otg_oc_lock);
	return ret;
}

/********************
 * BATT PSY GETTERS *
 ********************/

int smblib_get_prop_input_suspend(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	val->intval
		= (get_client_vote(chg->usb_icl_votable, USER_VOTER) == 0)
		 && get_client_vote(chg->dc_suspend_votable, USER_VOTER);
	return 0;
}

int smblib_get_prop_batt_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATIF_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATIF_INT_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = !(stat & (BAT_THERM_OR_ID_MISSING_RT_STS_BIT
					| BAT_TERMINAL_MISSING_RT_STS_BIT));

	return rc;
}

int smblib_get_prop_batt_capacity(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	int rc = -EINVAL;

	if (chg->fake_capacity >= 0) {
		val->intval = chg->fake_capacity;
		return 0;
	}

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PSEUDO_BATTERY
	if (chg->pseudo_batt && chg->lge_pseudo_lpc) {
		union lge_power_propval lge_val = {0,};
		rc = chg->lge_pseudo_lpc->get_property(chg->lge_pseudo_lpc,
				LGE_POWER_PROPS_PSEUDO_BATT_CAPACITY, &lge_val);
		if (rc >= 0) {
			val->intval = lge_val.intval;
			return rc;
		}
	}
#endif

	if (chg->bms_psy)
		rc = power_supply_get_property(chg->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, val);
	return rc;
}

#ifdef CONFIG_LGE_PM
int smblib_get_prop_batt_status_for_ui(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	bool usb_present, dc_present;
	int soc, chg_done;
	int rc = 0;
	u8 stat;

	rc = smblib_get_prop_usb_present(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb present rc=%d\n", rc);
		goto out;
	}

	usb_present = pval.intval;

	rc = smblib_get_prop_dc_present(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get dc present rc=%d\n", rc);
		goto out;
	}
	dc_present = pval.intval;

	if (usb_present || dc_present) {
		rc = smblib_get_prop_batt_capacity(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get batt capacity rc=%d\n", rc);
			goto out;
		}

		soc = pval.intval;
		if (soc == 100) {
			val->intval = POWER_SUPPLY_STATUS_FULL;
			return rc;
		}

		rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
				rc);
			goto out;
		}

		rc = smblib_get_prop_batt_charge_done(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read charge_done rc=%d\n",
				rc);
			goto out;
		}
		chg_done= pval.intval;

		if (get_effective_result_locked(chg->usb_icl_votable) != 0 &&
				chg->pseudo_chg_ui && (stat & CHARGER_ERROR_STATUS_BAT_OV_BIT || chg_done)) {
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			return rc;
		}
	}
out:
	rc = smblib_get_prop_batt_status(chg, val);

	return rc;
}
#endif

int smblib_get_prop_batt_status(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	bool usb_online, dc_online;
	u8 stat;
	int rc;

	rc = smblib_get_prop_usb_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb online property rc=%d\n",
			rc);
		return rc;
	}
	usb_online = (bool)pval.intval;

	rc = smblib_get_prop_dc_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get dc online property rc=%d\n",
			rc);
		return rc;
	}
	dc_online = (bool)pval.intval;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}
	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (!usb_online && !dc_online) {
		switch (stat) {
		case TERMINATE_CHARGE:
		case INHIBIT_CHARGE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		}
		return rc;
	}

	switch (stat) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
	case FAST_CHARGE:
	case FULLON_CHARGE:
	case TAPER_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case TERMINATE_CHARGE:
	case INHIBIT_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_FULL;
		break;
	case DISABLE_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

#ifndef CONFIG_LGE_PM
	if (val->intval != POWER_SUPPLY_STATUS_CHARGING)
		return 0;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_7_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_7 rc=%d\n",
				rc);
			return rc;
		}

	stat &= ENABLE_TRICKLE_BIT | ENABLE_PRE_CHARGING_BIT |
		 ENABLE_FAST_CHARGING_BIT | ENABLE_FULLON_MODE_BIT;
	if (!stat)
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
#endif

	return 0;
}

int smblib_get_prop_batt_charge_type(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	switch (stat & BATTERY_CHARGER_STATUS_MASK) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case FAST_CHARGE:
	case FULLON_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case TAPER_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TAPER;
		break;
	default:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	return rc;
}

int smblib_get_prop_batt_health(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
	union power_supply_propval pval2;
#endif
	int rc;
	int effective_fv_uv;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "BATTERY_CHARGER_STATUS_2 = 0x%02x\n",
		   stat);

	if (stat & CHARGER_ERROR_STATUS_BAT_OV_BIT) {
		rc = smblib_get_prop_batt_voltage_now(chg, &pval);
		if (!rc) {
			/*
			 * If Vbatt is within 40mV above Vfloat, then don't
			 * treat it as overvoltage.
			 */
			effective_fv_uv = get_effective_result(chg->fv_votable);
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
			smblib_get_prop_batt_voltage_max(chg, &pval2);
			if (pval.intval >= pval2.intval + 40000) {
#else
			if (pval.intval >= effective_fv_uv + 40000) {
#endif
				val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
				smblib_err(chg, "battery over-voltage vbat_fg = %duV, fv = %duV\n",
						pval.intval, effective_fv_uv);
				goto done;
			}
		}
	}

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
	if (chg->btm_state == BTM_HEALTH_OVERHEAT)
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chg->btm_state == BTM_HEALTH_COLD)
		val->intval = POWER_SUPPLY_HEALTH_COLD;
	else
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
#else
	if (stat & BAT_TEMP_STATUS_TOO_COLD_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & BAT_TEMP_STATUS_TOO_HOT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & BAT_TEMP_STATUS_COLD_SOFT_LIMIT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COOL;
	else if (stat & BAT_TEMP_STATUS_HOT_SOFT_LIMIT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_WARM;
	else
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
#endif

done:
	return rc;
}
#ifdef CONFIG_LGE_PM_FG_AGE
int get_prop_battery_condition(struct smb_charger *chg,
		union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
			POWER_SUPPLY_PROP_BATTERY_CONDITION, val);

	return rc;
}
#endif
int smblib_get_prop_system_temp_level(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->system_temp_level;
	return 0;
}

int smblib_get_prop_input_current_limited(struct smb_charger *chg,
				union power_supply_propval *val)
{
	u8 stat;
	int rc;

	if (chg->fake_input_current_limited >= 0) {
		val->intval = chg->fake_input_current_limited;
		return 0;
	}

	rc = smblib_read(chg, AICL_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read AICL_STATUS rc=%d\n", rc);
		return rc;
	}
	val->intval = (stat & SOFT_ILIMIT_BIT) || chg->is_hdc;
	return 0;
}

int smblib_get_prop_batt_voltage_now(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PSEUDO_BATTERY
	if (chg->pseudo_batt && chg->lge_pseudo_lpc) {
		union lge_power_propval lge_val = {0,};
		rc = chg->lge_pseudo_lpc->get_property(chg->lge_pseudo_lpc,
				LGE_POWER_PROPS_PSEUDO_BATT_VOLT, &lge_val);
		if (rc >= 0) {
			val->intval = lge_val.intval;
			return rc;
		}
	}
#endif
	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
	return rc;
}

int smblib_get_prop_batt_current_now(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_CURRENT_NOW, val);
	return rc;
}

int smblib_get_prop_batt_temp(struct smb_charger *chg,
			      union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PSEUDO_BATTERY
	if (chg->pseudo_batt && chg->lge_pseudo_lpc) {
		union lge_power_propval lge_val = {0,};
		rc = chg->lge_pseudo_lpc->get_property(chg->lge_pseudo_lpc,
				LGE_POWER_PROPS_PSEUDO_BATT_TEMP, &lge_val);
		if (rc >= 0) {
			val->intval = lge_val.intval * 10;
			return rc;
		}
	}
#endif
	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_TEMP, val);
#ifdef CONFIG_LGE_PM_USE_FAKE_BATT_TEMP_CTRL
	if (chg->fake_batt_temp_ctrl) {
		val->intval = 250;
	}
#endif

	return rc;
}

int smblib_get_prop_step_chg_step(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	if (!chg->step_chg_enabled) {
		val->intval = -1;
		return 0;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	val->intval = (stat & STEP_CHARGING_STATUS_MASK) >>
				STEP_CHARGING_STATUS_SHIFT;

	return rc;
}

int smblib_get_prop_batt_charge_done(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	val->intval = (stat == TERMINATE_CHARGE);
	return 0;
}

int smblib_get_prop_charge_qnovo_enable(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, QNOVO_PT_ENABLE_CMD_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read QNOVO_PT_ENABLE_CMD rc=%d\n",
			rc);
		return rc;
	}

	val->intval = (bool)(stat & QNOVO_PT_ENABLE_CMD_BIT);
	return 0;
}

#ifdef CONFIG_LGE_PM
int smblib_get_prop_fcc_max(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	val->intval = get_client_vote(chg->fcc_votable, BATT_PROFILE_VOTER);
	if (val->intval == -EINVAL) {
		val->intval = get_client_vote(chg->fcc_votable, QNOVO_VOTER);
	}

	return 0;
}

int smblib_get_prop_batt_voltage_max(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	val->intval = get_client_vote(chg->fv_votable, BATT_PROFILE_VOTER);
	if (val->intval == -EINVAL) {
		val->intval = get_client_vote(chg->fv_votable, QNOVO_VOTER);
	}

	return 0;
}
#endif

#ifdef CONFIG_LGE_PM
int smblib_get_aicl_done(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	u8 stat;
	int rc;

	rc = smblib_read(chg, AICL_STATUS_REG, &stat);

	if (rc < 0) {
		smblib_err(chg, "Couldn't read AICL_STATUS_REG rc=%d\n", rc);
		return rc;
	}
	val->intval = stat & AICL_DONE_BIT;

	return 0;
}

int smblib_get_aicl_fail(struct smb_charger *chg)
{
	u8 stat;
	int rc;

	rc = smblib_read(chg, AICL_STATUS_REG, &stat);

	if (rc < 0) {
		smblib_err(chg, "Couldn't read AICL_STATUS_REG rc=%d\n", rc);
		return rc;
	}
	return (stat & AICL_FAIL_BIT) >> 1;
}
#endif

/***********************
 * BATTERY PSY SETTERS *
 ***********************/

int smblib_set_prop_input_suspend(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	int rc;

	/* vote 0mA when suspended */
	rc = vote(chg->usb_icl_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s USB rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	rc = vote(chg->dc_suspend_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s DC rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	power_supply_changed(chg->batt_psy);
	return rc;
}

#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
int smblib_set_prop_moisture_detection(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	int rc;

	/* vote 0mA when suspended */
	rc = vote(chg->usb_icl_votable, MOISTURE_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s USB rc=%d\n",
			(bool)val->intval ? "enable" : "disable", rc);
		return rc;
	}

	power_supply_changed(chg->batt_psy);
	return rc;
}

#endif
int smblib_set_prop_batt_capacity(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	chg->fake_capacity = val->intval;

	power_supply_changed(chg->batt_psy);

	return 0;
}

#ifdef CONFIG_LGE_PM
int smblib_set_prop_safety_timer_enabled(struct smb_charger *chg,
				bool enable)
{
	int rc;
	u8 reg;

	if (enable == chg->safety_timer_en)
		return 0;

	if (enable)
		reg = PRE_CHARGE_SAFETY_TIMER_EN | FAST_CHARGE_SAFETY_TIMER_EN;
	else
		reg = 0;

	rc = smblib_masked_write(chg, CHGR_SAFETY_TIMER_ENABLE_CFG,
			CHGR_SAFETY_TIMER_EN, reg);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s safety timer rc=%d\n",
				enable? "enable" : "disable", rc);
		return rc;
	}
	chg->safety_timer_en = enable;

	return 0;
}

int smblib_set_prop_parallel_batfet_en(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;

	if (val->intval == 0)
		rc = gpiod_direction_output(gpio_to_desc(chg->smb_bat_en), true);
	else
		rc = gpiod_direction_output(gpio_to_desc(chg->smb_bat_en), false);

	if (rc < 0) {
		smblib_err(chg, "Couldn't set parallel batfet_en rc=%d\n", rc);
		return rc;
	}

	smblib_dbg(chg, PR_LGE, "Parallel batfet %s\n", val->intval ? "enabled" : "disabled");
	return rc;
}
#endif

int smblib_set_prop_system_temp_level(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels)
		return -EINVAL;

	chg->system_temp_level = val->intval;
	/* disable parallel charge in case of system temp level */
	vote(chg->pl_disable_votable, THERMAL_DAEMON_VOTER,
			chg->system_temp_level ? true : false, 0);

	if (chg->system_temp_level == chg->thermal_levels)
		return vote(chg->chg_disable_votable,
			THERMAL_DAEMON_VOTER, true, 0);

	vote(chg->chg_disable_votable, THERMAL_DAEMON_VOTER, false, 0);
	if (chg->system_temp_level == 0)
		return vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, false, 0);

	vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, true,
			chg->thermal_mitigation[chg->system_temp_level]);
	return 0;
}

int smblib_set_prop_charge_qnovo_enable(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_masked_write(chg, QNOVO_PT_ENABLE_CMD_REG,
			QNOVO_PT_ENABLE_CMD_BIT,
			val->intval ? QNOVO_PT_ENABLE_CMD_BIT : 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't enable qnovo rc=%d\n", rc);
		return rc;
	}

	return rc;
}

int smblib_set_prop_input_current_limited(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	chg->fake_input_current_limited = val->intval;
	return 0;
}

int smblib_rerun_aicl(struct smb_charger *chg)
{
	int rc, settled_icl_ua;
	u8 stat;

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	if (chg->is_factory_cable) // disable aicl with factory cable
		return 0;
#endif

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
								rc);
		return rc;
	}

	/* USB is suspended so skip re-running AICL */
	if (stat & USBIN_SUSPEND_STS_BIT)
		return rc;

#ifdef CONFIG_LGE_PM
	smblib_dbg(chg, PR_LGE, "re-running AICL\n");
#else
	smblib_dbg(chg, PR_MISC, "re-running AICL\n");
#endif
	rc = smblib_get_charge_param(chg, &chg->param.icl_stat,
			&settled_icl_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get settled ICL rc=%d\n", rc);
		return rc;
	}

	vote(chg->usb_icl_votable, AICL_RERUN_VOTER, true,
			max(settled_icl_ua - chg->param.usb_icl.step_u,
				chg->param.usb_icl.step_u));
	vote(chg->usb_icl_votable, AICL_RERUN_VOTER, false, 0);

	return 0;
}

static int smblib_dp_pulse(struct smb_charger *chg)
{
	int rc;

	/* QC 3.0 increment */
	rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, SINGLE_INCREMENT_BIT,
			SINGLE_INCREMENT_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
				rc);

	return rc;
}

static int smblib_dm_pulse(struct smb_charger *chg)
{
	int rc;

	/* QC 3.0 decrement */
	rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, SINGLE_DECREMENT_BIT,
			SINGLE_DECREMENT_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
				rc);

	return rc;
}

int smblib_dp_dm(struct smb_charger *chg, int val)
{
	int target_icl_ua, rc = 0;
	union power_supply_propval pval;

	switch (val) {
	case POWER_SUPPLY_DP_DM_DP_PULSE:
		rc = smblib_dp_pulse(chg);
		if (!rc)
			chg->pulse_cnt++;
		smblib_dbg(chg, PR_PARALLEL, "DP_DM_DP_PULSE rc=%d cnt=%d\n",
				rc, chg->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_DM_PULSE:
		rc = smblib_dm_pulse(chg);
		if (!rc && chg->pulse_cnt)
			chg->pulse_cnt--;
		smblib_dbg(chg, PR_PARALLEL, "DP_DM_DM_PULSE rc=%d cnt=%d\n",
				rc, chg->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_ICL_DOWN:
		target_icl_ua = get_effective_result(chg->usb_icl_votable);
		if (target_icl_ua < 0) {
			/* no client vote, get the ICL from charger */
			rc = power_supply_get_property(chg->usb_psy,
					POWER_SUPPLY_PROP_HW_CURRENT_MAX,
					&pval);
			if (rc < 0) {
				smblib_err(chg,
					"Couldn't get max current rc=%d\n",
					rc);
				return rc;
			}
			target_icl_ua = pval.intval;
		}

		/*
		 * Check if any other voter voted on USB_ICL in case of
		 * voter other than SW_QC3_VOTER reset and restart reduction
		 * again.
		 */
		if (target_icl_ua != get_client_vote(chg->usb_icl_votable,
							SW_QC3_VOTER))
			chg->usb_icl_delta_ua = 0;

		chg->usb_icl_delta_ua += 100000;
		vote(chg->usb_icl_votable, SW_QC3_VOTER, true,
						target_icl_ua - 100000);
		smblib_dbg(chg, PR_PARALLEL, "ICL DOWN ICL=%d reduction=%d\n",
				target_icl_ua, chg->usb_icl_delta_ua);
		break;
	case POWER_SUPPLY_DP_DM_ICL_UP:
	default:
		break;
	}

	return rc;
}
#ifdef CONFIG_LGE_PM_INOV_GEN3_SYSFS_SUPPORT
int smblib_set_prop_inov_temp(struct smb_charger *chg, enum power_supply_property prop, const union power_supply_propval *val) {
	int rc =0;

	if (chg->bms_psy) {
		switch (prop) {
		case POWER_SUPPLY_PROP_CHARGER_TEMP_MAX:
			chg->inov_off_temp[CHARGER_TEMP] = val->intval;
			break;
		case POWER_SUPPLY_PROP_CHARGER_TEMP_HOT_MAX:
			chg->inov_off_temp[CHARGER_TEMP_HOT] = val->intval;
			break;
		case POWER_SUPPLY_PROP_SKIN_TEMP_MAX:
			chg->inov_off_temp[SKIN_TEMP] = val->intval;
			break;
		case POWER_SUPPLY_PROP_SKIN_TEMP_HOT_MAX:
			chg->inov_off_temp[SKIN_TEMP_HOT] = val->intval;
			break;
		default:
			rc = -EINVAL;
			break;
		}
		smblib_dbg(chg, PR_LGE, "update inov lcd off temp charger[%d], charger_hot[%d], skin[%d], skin_hot[%d]\n",
			chg->inov_off_temp[CHARGER_TEMP], chg->inov_off_temp[CHARGER_TEMP_HOT],
			chg->inov_off_temp[SKIN_TEMP], chg->inov_off_temp[SKIN_TEMP_HOT]);

		rc = power_supply_set_property(chg->bms_psy, prop, val);
	}
	return rc;
}
#endif

/*******************
 * DC PSY GETTERS *
 *******************/

int smblib_get_prop_dc_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, DCIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DCIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & DCIN_PLUGIN_RT_STS_BIT);
	return 0;
}

int smblib_get_prop_dc_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	if (get_client_vote(chg->dc_suspend_votable, USER_VOTER)) {
		val->intval = false;
		return rc;
	}

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);

	val->intval = (stat & USE_DCIN_BIT) &&
		      (stat & VALID_INPUT_POWER_SOURCE_STS_BIT);

	return rc;
}

int smblib_get_prop_dc_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	val->intval = get_effective_result_locked(chg->dc_icl_votable);
	return 0;
}

#ifdef CONFIG_IDTP9223_CHARGER
int smblib_get_prop_qipma_on(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc = 0;
	u8 qipma_on_stat;

	rc = smblib_read(chg, BATIF_BASE + INT_RT_STS_OFFSET, &qipma_on_stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read qipma_on status rc=%d\n", rc);
		val->intval = -EINVAL;
		return 0;
	}
	val->intval = (bool)(qipma_on_stat & BAT_6_RT_STS_BIT);
	return rc;
}
#endif

/*******************
 * DC PSY SETTERS *
 * *****************/

int smblib_set_prop_dc_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	rc = vote(chg->dc_icl_votable, USER_VOTER, true, val->intval);
	return rc;
}

/*******************
 * USB PSY GETTERS *
 *******************/
#ifdef CONFIG_LGE_PM
enum fastchg_state_index {
	NOT_PD_HVDCP = 0,
	PD_POWER_NG,
	ERROR_GET_ICL,
	HVDCP_MODE = 10,
	PD_POWER_OK,
	PSEUDO_TYPE,
	FAKE_HVDCP_MODE,
#ifdef CONFIG_IDTP9223_CHARGER
	DC_9W_MODE,
#endif
};
#define USB_INPUT_5P0_VOLTAGE 5000000
#define FAST_CHARGING_INPUT_POWER	15000000
int smblib_get_prop_fastchg_state(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int pd_current_max, reason;
	int power = 0;
	int fastchg_state = 0;
	static int pre_power;
	static int pre_fastchg_state = -1;

#ifdef CONFIG_IDTP9223_CHARGER
	if (chg->dc_fastchg) {
		fastchg_state = 1;
		reason = DC_9W_MODE;
		goto out;
	}
#endif
	if (chg->fake_hvdcp_mode && chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP) {
		fastchg_state = 1;
		reason = FAKE_HVDCP_MODE;
		goto out;
	}

	if (get_effective_result_locked(chg->pseudo_usb_type_votable) == PSEUDO_HVDCP) {
		fastchg_state = 1;
		reason = PSEUDO_TYPE;
		goto out;
	}

	if ((chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP ||
			chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3) &&
			!get_effective_result(chg->hvdcp_disable_votable_indirect)) {
		fastchg_state = 1;
		reason = HVDCP_MODE;
		goto out;
	}

	if (!chg->pd_active || chg->real_charger_type == POWER_SUPPLY_TYPE_UNKNOWN) {
		reason = NOT_PD_HVDCP;
		goto out;
	}

	pd_current_max = get_client_vote_locked(chg->usb_icl_votable, PD_VOTER);
	if (pd_current_max < 0) {
		smblib_dbg(chg, PR_LGE,
			"Error get_client_vote_locked = %d\n", pd_current_max);
		reason = ERROR_GET_ICL;
		goto out;
	}

	power = (pd_current_max / 1000) * (chg->voltage_max_uv / 1000);

	if (pre_power != power) {
		smblib_dbg(chg, PR_LGE,
			"pd_current_max = %d uA, pd_voltage_max = %d uV\n",
			pd_current_max, chg->voltage_max_uv);
	}

	if (power >= FAST_CHARGING_INPUT_POWER) {
		fastchg_state = 1;
		reason = PD_POWER_OK;
		goto out;
	}
	reason = PD_POWER_NG;
out:
	if (pre_fastchg_state != fastchg_state) {
		smblib_dbg(chg, PR_LGE,
			"Changed fastchg_state to %d by %d\n",
			fastchg_state, reason);
		pre_fastchg_state = fastchg_state;
	}
	pre_power = power;
	val->intval = fastchg_state;

	return 0;
}

int smblib_get_time_to_full(
	struct smb_charger *chg,
	union power_supply_propval *val)
{
	int rc;

	chg->ttf_psy = power_supply_get_by_name("ttf");
	if (!chg->ttf_psy) {
		smblib_err(chg, "bms supply not found\n");
		return -1;
	}
	rc = power_supply_get_property(chg->ttf_psy,
		POWER_SUPPLY_PROP_TIME_TO_FULL_NOW, val);
	if(rc) {
		smblib_err(chg, "time to full read fail : %d\n", rc);
		return rc;
	}

	return rc;
}

int smblib_get_prop_usb_present_raw(struct smb_charger *chg,
				union power_supply_propval *val) {
	int rc;
	u8 stat;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	return 0;
}
#endif

int smblib_get_prop_usb_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

#ifdef CONFIG_LGE_PM
	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
		if (is_client_vote_enabled_locked(chg->pseudo_usb_type_votable, PD_HARD_RESET_VOTER)) {
			val->intval = true;
			return 0;
		}

		if (is_client_vote_enabled_locked(chg->pseudo_usb_type_votable, LEGACY_CABLE_VOTER)) {
			val->intval = true;
			return 0;
		}
	}
#endif

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_RT_STS rc=%d\n", rc);
		return rc;
	}

#if 0//def CONFIG_LGE_USB_FACTORY
	rc = smblib_get_prop_typec_mode(chg, val);
	if (!rc && val->intval == POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY) {
		val->intval = 1;
		return rc;
	}
#endif

	val->intval = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	return 0;
}

int smblib_get_prop_usb_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

#ifdef CONFIG_LGE_PM
	if (is_client_vote_enabled_locked(chg->pseudo_usb_type_votable, PD_HARD_RESET_VOTER)
			&& lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
		val->intval = true;
		return rc;
	}

	if (is_client_vote_enabled_locked(chg->pseudo_usb_type_votable, LEGACY_CABLE_VOTER)) {
		val->intval = true;
		return rc;
	}
#endif

	if (get_client_vote_locked(chg->usb_icl_votable, USER_VOTER) == 0) {
		val->intval = false;
		return rc;
	}

#ifdef CONFIG_LGE_PM
	if (is_client_vote_enabled_locked(chg->usb_icl_votable, BOOST_BACK_VOTER)) {
		val->intval = true;
		return rc;
	}
#endif

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);

	val->intval = (stat & USE_USBIN_BIT) &&
		      (stat & VALID_INPUT_POWER_SOURCE_STS_BIT);
	return rc;
}

int smblib_get_prop_usb_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_get_prop_usb_present(chg, val);
	if (rc < 0 || !val->intval)
		return rc;

	if (!chg->iio.usbin_v_chan ||
		PTR_ERR(chg->iio.usbin_v_chan) == -EPROBE_DEFER)
		chg->iio.usbin_v_chan = iio_channel_get(chg->dev, "usbin_v");

	if (IS_ERR(chg->iio.usbin_v_chan))
		return PTR_ERR(chg->iio.usbin_v_chan);

	return iio_read_channel_processed(chg->iio.usbin_v_chan, &val->intval);
}

int smblib_get_prop_pd_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	val->intval = get_client_vote_locked(chg->usb_icl_votable, PD_VOTER);
	return 0;
}

int smblib_get_prop_usb_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
#ifdef CONFIG_LGE_PM
	val->intval = get_effective_result(chg->usb_icl_votable);
#else
	val->intval = get_client_vote_locked(chg->usb_icl_votable,
			USB_PSY_VOTER);
#endif
	return 0;
}

int smblib_get_prop_usb_current_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_get_prop_usb_present(chg, val);
	if (rc < 0 || !val->intval)
		return rc;

	if (!chg->iio.usbin_i_chan ||
		PTR_ERR(chg->iio.usbin_i_chan) == -EPROBE_DEFER)
		chg->iio.usbin_i_chan = iio_channel_get(chg->dev, "usbin_i");

	if (IS_ERR(chg->iio.usbin_i_chan))
		return PTR_ERR(chg->iio.usbin_i_chan);

	return iio_read_channel_processed(chg->iio.usbin_i_chan, &val->intval);
}

int smblib_get_prop_charger_temp(struct smb_charger *chg,
				 union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.temp_chan ||
		PTR_ERR(chg->iio.temp_chan) == -EPROBE_DEFER)
		chg->iio.temp_chan = iio_channel_get(chg->dev, "charger_temp");

	if (IS_ERR(chg->iio.temp_chan))
		return PTR_ERR(chg->iio.temp_chan);

	rc = iio_read_channel_processed(chg->iio.temp_chan, &val->intval);
	val->intval /= 100;
	return rc;
}

int smblib_get_prop_charger_temp_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.temp_max_chan ||
		PTR_ERR(chg->iio.temp_max_chan) == -EPROBE_DEFER)
		chg->iio.temp_max_chan = iio_channel_get(chg->dev,
							 "charger_temp_max");
	if (IS_ERR(chg->iio.temp_max_chan))
		return PTR_ERR(chg->iio.temp_max_chan);

	rc = iio_read_channel_processed(chg->iio.temp_max_chan, &val->intval);
	val->intval /= 100;
	return rc;
}

#ifdef CONFIG_LGE_PM_INOV_GEN3_SYSFS_SUPPORT
int smblib_get_prop_charger_temp_hot_max(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.temp_hot_max_chan ||
			PTR_ERR(chg->iio.temp_hot_max_chan) == -EPROBE_DEFER)
		chg->iio.temp_hot_max_chan = iio_channel_get(chg->dev,
							"charger_temp_hot_max");
	if (IS_ERR(chg->iio.temp_hot_max_chan))
		return PTR_ERR(chg->iio.temp_hot_max_chan);

	rc = iio_read_channel_processed(chg->iio.temp_hot_max_chan,
					&val->intval);
	val->intval /= 100;
	return rc;
}

int smblib_get_prop_skin_temp(struct smb_charger *chg,
			     union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.skin_temp_chan ||
			PTR_ERR(chg->iio.skin_temp_chan) == -EPROBE_DEFER)
		chg->iio.skin_temp_chan = iio_channel_get(chg->dev,
								"skin_temp");

	if (IS_ERR(chg->iio.skin_temp_chan))
		return PTR_ERR(chg->iio.skin_temp_chan);

	rc = iio_read_channel_processed(chg->iio.skin_temp_chan,
					&val->intval);
	val->intval /= 100;
	return rc;
}

int smblib_get_prop_skin_temp_max(struct smb_charger *chg,
			     union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.skin_temp_max_chan ||
			PTR_ERR(chg->iio.skin_temp_max_chan) == -EPROBE_DEFER)
		chg->iio.skin_temp_max_chan = iio_channel_get(chg->dev,
								"skin_temp_max");

	if (IS_ERR(chg->iio.skin_temp_max_chan))
		return PTR_ERR(chg->iio.skin_temp_max_chan);

	rc = iio_read_channel_processed(chg->iio.skin_temp_max_chan,
					&val->intval);
	val->intval /= 100;
	return rc;
}

int smblib_get_prop_skin_temp_hot_max(struct smb_charger *chg,
				 union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.skin_temp_hot_max_chan ||
			PTR_ERR(chg->iio.skin_temp_hot_max_chan) == -EPROBE_DEFER)
		chg->iio.skin_temp_hot_max_chan = iio_channel_get(chg->dev,
							 "skin_temp_hot_max");
	if (IS_ERR(chg->iio.skin_temp_hot_max_chan))
		return PTR_ERR(chg->iio.skin_temp_hot_max_chan);

	rc = iio_read_channel_processed(chg->iio.skin_temp_hot_max_chan,
					&val->intval);
	val->intval /= 100;
	return rc;
}
#endif

int smblib_get_prop_typec_cc_orientation(struct smb_charger *chg,
					 union power_supply_propval *val)
{
	if (chg->typec_status[3] & CC_ATTACHED_BIT)
		val->intval =
			(bool)(chg->typec_status[3] & CC_ORIENTATION_BIT) + 1;
	else
		val->intval = 0;

	return 0;
}

static const char * const smblib_typec_mode_name[] = {
	[POWER_SUPPLY_TYPEC_NONE]		  = "NONE",
	[POWER_SUPPLY_TYPEC_SOURCE_DEFAULT]	  = "SOURCE_DEFAULT",
	[POWER_SUPPLY_TYPEC_SOURCE_MEDIUM]	  = "SOURCE_MEDIUM",
	[POWER_SUPPLY_TYPEC_SOURCE_HIGH]	  = "SOURCE_HIGH",
	[POWER_SUPPLY_TYPEC_NON_COMPLIANT]	  = "NON_COMPLIANT",
	[POWER_SUPPLY_TYPEC_SINK]		  = "SINK",
	[POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE]   = "SINK_POWERED_CABLE",
	[POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY] = "SINK_DEBUG_ACCESSORY",
	[POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER]   = "SINK_AUDIO_ADAPTER",
	[POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY]   = "POWERED_CABLE_ONLY",
};

static int smblib_get_prop_ufp_mode(struct smb_charger *chg)
{
	switch (chg->typec_status[0]) {
	case UFP_TYPEC_RDSTD_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
	case UFP_TYPEC_RD1P5_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_MEDIUM;
	case UFP_TYPEC_RD3P0_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_HIGH;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NONE;
}

static int smblib_get_prop_dfp_mode(struct smb_charger *chg)
{
	switch (chg->typec_status[1] & DFP_TYPEC_MASK) {
	case DFP_RA_RA_BIT:
		return POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER;
	case DFP_RD_RD_BIT:
		return POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY;
	case DFP_RD_RA_VCONN_BIT:
		return POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE;
	case DFP_RD_OPEN_BIT:
		return POWER_SUPPLY_TYPEC_SINK;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NONE;
}

static int smblib_get_prop_typec_mode(struct smb_charger *chg)
{
	if (chg->typec_status[3] & UFP_DFP_MODE_STATUS_BIT)
		return smblib_get_prop_dfp_mode(chg);
	else
		return smblib_get_prop_ufp_mode(chg);
}

int smblib_get_prop_typec_power_role(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc = 0;
	u8 ctrl;

	rc = smblib_read(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG, &ctrl);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_INTRPT_ENB_SOFTWARE_CTRL = 0x%02x\n",
		   ctrl);

	if (ctrl & TYPEC_DISABLE_CMD_BIT) {
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		return rc;
	}

	switch (ctrl & (DFP_EN_CMD_BIT | UFP_EN_CMD_BIT)) {
	case 0:
		val->intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		break;
	case DFP_EN_CMD_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SOURCE;
		break;
	case UFP_EN_CMD_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SINK;
		break;
	default:
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		smblib_err(chg, "unsupported power role 0x%02lx\n",
			ctrl & (DFP_EN_CMD_BIT | UFP_EN_CMD_BIT));
		return -EINVAL;
	}

	return rc;
}

int smblib_get_prop_pd_allowed(struct smb_charger *chg,
			       union power_supply_propval *val)
{
#ifdef CONFIG_LGE_USB_FACTORY
	int typec_mode = smblib_get_prop_typec_mode(chg);
	if (typec_mode == POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY) {
		val->intval = 0;
		return 0;
	}
#endif
	val->intval = get_effective_result(chg->pd_allowed_votable);
	return 0;
}

int smblib_get_prop_input_current_settled(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	return smblib_get_charge_param(chg, &chg->param.icl_stat, &val->intval);
}

#define HVDCP3_STEP_UV	200000
int smblib_get_prop_input_voltage_settled(struct smb_charger *chg,
						union power_supply_propval *val)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);
	int rc, pulses;
	u8 stat;

	val->intval = MICRO_5V;
	if (apsd_result == NULL) {
		smblib_err(chg, "APSD result is NULL\n");
		return 0;
	}

	switch (apsd_result->pst) {
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		rc = smblib_read(chg, QC_PULSE_COUNT_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_PULSE_COUNT rc=%d\n", rc);
			return 0;
		}
		pulses = (stat & QC_PULSE_COUNT_MASK);
		val->intval = MICRO_5V + HVDCP3_STEP_UV * pulses;
		break;
	default:
		val->intval = MICRO_5V;
		break;
	}

	return 0;
}

int smblib_get_prop_pd_in_hard_reset(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	val->intval = chg->pd_hard_reset;
	return 0;
}

int smblib_get_pe_start(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	/*
	 * hvdcp timeout voter is the last one to allow pd. Use its vote
	 * to indicate start of pe engine
	 */
	val->intval
		= !get_client_vote_locked(chg->pd_disallowed_votable_indirect,
			HVDCP_TIMEOUT_VOTER);
	return 0;
}

int smblib_get_prop_die_health(struct smb_charger *chg,
						union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TEMP_RANGE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TEMP_RANGE_STATUS_REG rc=%d\n",
									rc);
		return rc;
	}

	/* TEMP_RANGE bits are mutually exclusive */
	switch (stat & TEMP_RANGE_MASK) {
	case TEMP_BELOW_RANGE_BIT:
		val->intval = POWER_SUPPLY_HEALTH_COOL;
		break;
	case TEMP_WITHIN_RANGE_BIT:
		val->intval = POWER_SUPPLY_HEALTH_WARM;
		break;
	case TEMP_ABOVE_RANGE_BIT:
		val->intval = POWER_SUPPLY_HEALTH_HOT;
		break;
	case ALERT_LEVEL_BIT:
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	default:
		val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	return 0;
}

#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
int smblib_get_prop_typec_cc_disable(struct smb_charger *chg,
		union power_supply_propval *val)
{
	int rc = 0;
	u8 ctrl;

	rc = smblib_read(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG, &ctrl);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
				rc);
		return rc;
	}

	smblib_dbg(chg, PR_REGISTER, "TYPE_C_INTRPT_ENB_SOFTWARE_CTRL = 0x%02x\n",
			ctrl);
	val->intval = ctrl & TYPEC_DISABLE_CMD_BIT;

	return rc;
}
#endif

/*******************
 * USB PSY SETTERS *
 * *****************/

int smblib_set_prop_pd_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	if (chg->pd_active)
		rc = vote(chg->usb_icl_votable, PD_VOTER, true, val->intval);
	else
		rc = -EPERM;

	return rc;
}

int smblib_set_prop_usb_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc = 0;

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	if (chg->is_factory_cable) {
		smblib_err(chg, "factory cable is inserted!!!\n");
		return 0;
	}
#endif

	if (!chg->pd_active) {
		rc = vote(chg->usb_icl_votable, USB_PSY_VOTER,
				true, val->intval);
	} else if (chg->system_suspend_supported) {
		if (val->intval <= USBIN_25MA)
			rc = vote(chg->usb_icl_votable,
				PD_SUSPEND_SUPPORTED_VOTER, true, val->intval);
		else
			rc = vote(chg->usb_icl_votable,
				PD_SUSPEND_SUPPORTED_VOTER, false, 0);
	}
	return rc;
}

int smblib_set_prop_boost_current(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_boost,
				val->intval <= chg->boost_threshold_ua ?
				chg->chg_freq.freq_below_otg_threshold :
				chg->chg_freq.freq_above_otg_threshold);
	if (rc < 0) {
		dev_err(chg->dev, "Error in setting freq_boost rc=%d\n", rc);
		return rc;
	}

	chg->boost_current_ua = val->intval;
	return rc;
}

int smblib_set_prop_typec_power_role(struct smb_charger *chg,
				     const union power_supply_propval *val)
{
	int rc = 0;
	u8 power_role;

	switch (val->intval) {
	case POWER_SUPPLY_TYPEC_PR_NONE:
		power_role = TYPEC_DISABLE_CMD_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_DUAL:
		power_role = 0;
		break;
	case POWER_SUPPLY_TYPEC_PR_SINK:
		power_role = UFP_EN_CMD_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_SOURCE:
		power_role = DFP_EN_CMD_BIT;
		break;
	default:
		smblib_err(chg, "power role %d not supported\n", val->intval);
		return -EINVAL;
	}

	if (power_role == UFP_EN_CMD_BIT) {
		/* disable PBS workaround when forcing sink mode */
		rc = smblib_write(chg, TM_IO_DTEST4_SEL, 0x0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't write to TM_IO_DTEST4_SEL rc=%d\n",
				rc);
		}
	} else {
		/* restore it back to 0xA5 */
		rc = smblib_write(chg, TM_IO_DTEST4_SEL, 0xA5);
		if (rc < 0) {
			smblib_err(chg, "Couldn't write to TM_IO_DTEST4_SEL rc=%d\n",
				rc);
		}
	}

#ifdef CONFIG_LGE_USB
	if (power_role == DFP_EN_CMD_BIT) {
		/* disable try.SINK mode */
		rc = smblib_masked_write(chg, TYPE_C_CFG_3_REG,
					 EN_TRYSINK_MODE_BIT, 0);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't set TRYSINK_MODE rc=%d\n",
				rc);
		}
	} else if (power_role == 0 &&
		   !(chg->typec_status[3] & TYPEC_DEBOUNCE_DONE_STATUS_BIT)) {
		/* enable try.SINK mode */
		rc = smblib_masked_write(chg, TYPE_C_CFG_3_REG,
					 EN_TRYSINK_MODE_BIT, EN_TRYSINK_MODE_BIT);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't set TRYSINK_MODE rc=%d\n",
				rc);
		}
	}
#endif

#ifdef CONFIG_LGE_USB
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 TYPEC_POWER_ROLE_CMD_MASK | VCONN_EN_VALUE_BIT,
				 power_role | (chg->vconn_en ? VCONN_EN_VALUE_BIT : 0));
#else
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 TYPEC_POWER_ROLE_CMD_MASK, power_role);
#endif
	if (rc < 0) {
		smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			power_role, rc);
		return rc;
	}

	return rc;
}

int smblib_set_prop_usb_voltage_min(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, min_uv;

	min_uv = min(val->intval, chg->voltage_max_uv);
	rc = smblib_set_usb_pd_allowed_voltage(chg, min_uv,
					       chg->voltage_max_uv);
	if (rc < 0) {
		smblib_err(chg, "invalid max voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	chg->voltage_min_uv = min_uv;
	return rc;
}

int smblib_set_prop_usb_voltage_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, max_uv;

	max_uv = max(val->intval, chg->voltage_min_uv);
	rc = smblib_set_usb_pd_allowed_voltage(chg, chg->voltage_min_uv,
					       max_uv);
	if (rc < 0) {
		smblib_err(chg, "invalid min voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	chg->voltage_max_uv = max_uv;
	return rc;
}

int smblib_set_prop_pd_active(struct smb_charger *chg,
			      const union power_supply_propval *val)
{
	int rc;
	bool orientation, sink_attached, hvdcp;
	u8 stat;

#ifdef CONFIG_LGE_PM
	if (val->intval)
		chg->checking_pd_active = true;

	if (!get_effective_result(chg->pd_allowed_votable)) {
		smblib_dbg(chg, PR_LGE, "disabled pd. pd_active(%d).\n", val->intval);
		return -EINVAL;
	}
#else
	if (!get_effective_result(chg->pd_allowed_votable))
		return -EINVAL;
#endif
#ifdef CONFIG_LGE_PM
	smblib_dbg(chg, PR_LGE, "update pd_active %d -> %d.\n", chg->pd_active, val->intval);
#endif
#ifdef CONFIG_LGE_USB
	if (chg->pd_active && val->intval)
		goto skip_pd_active;
#endif

	chg->pd_active = val->intval;
	if (chg->pd_active) {
		vote(chg->apsd_disable_votable, PD_VOTER, true, 0);
		vote(chg->pd_allowed_votable, PD_VOTER, true, 0);
		vote(chg->usb_irq_enable_votable, PD_VOTER, true, 0);

		/*
		 * VCONN_EN_ORIENTATION_BIT controls whether to use CC1 or CC2
		 * line when TYPEC_SPARE_CFG_BIT (CC pin selection s/w override)
		 * is set or when VCONN_EN_VALUE_BIT is set.
		 */
		orientation = chg->typec_status[3] & CC_ORIENTATION_BIT;
		rc = smblib_masked_write(chg,
				TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				VCONN_EN_ORIENTATION_BIT,
				orientation ? 0 : VCONN_EN_ORIENTATION_BIT);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't enable vconn on CC line rc=%d\n", rc);

		/* SW controlled CC_OUT */
		rc = smblib_masked_write(chg, TAPER_TIMER_SEL_CFG_REG,
				TYPEC_SPARE_CFG_BIT, TYPEC_SPARE_CFG_BIT);
		if (rc < 0)
			smblib_err(chg, "Couldn't enable SW cc_out rc=%d\n",
									rc);

		/*
		 * Enforce 500mA for PD until the real vote comes in later.
		 * It is guaranteed that pd_active is set prior to
		 * pd_current_max
		 */
		rc = vote(chg->usb_icl_votable, PD_VOTER, true, USBIN_500MA);
		if (rc < 0)
			smblib_err(chg, "Couldn't vote for USB ICL rc=%d\n",
									rc);

		/* since PD was found the cable must be non-legacy */
		vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, false, 0);

		/* clear USB ICL vote for DCP_VOTER */
		rc = vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
		if (rc < 0)
			smblib_err(chg, "Couldn't un-vote DCP from USB ICL rc=%d\n",
									rc);

		/* remove USB_PSY_VOTER */
		rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
		if (rc < 0)
			smblib_err(chg, "Couldn't unvote USB_PSY rc=%d\n", rc);
	} else {
#ifdef CONFIG_LGE_USB
		vote(chg->usb_icl_votable, PD_VOTER, false, 0);
#endif
		rc = smblib_read(chg, APSD_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read APSD status rc=%d\n",
									rc);
			return rc;
		}

		hvdcp = stat & QC_CHARGER_BIT;
		vote(chg->apsd_disable_votable, PD_VOTER, false, 0);
#ifdef CONFIG_LGE_USB
		vote(chg->pd_allowed_votable, PD_VOTER, false, 0);
		vote(chg->usb_irq_enable_votable, PD_VOTER, false, 0);
#else
		vote(chg->pd_allowed_votable, PD_VOTER, true, 0);
		vote(chg->usb_irq_enable_votable, PD_VOTER, true, 0);
#endif
		vote(chg->hvdcp_disable_votable_indirect, PD_INACTIVE_VOTER,
								false, 0);

		/* HW controlled CC_OUT */
		rc = smblib_masked_write(chg, TAPER_TIMER_SEL_CFG_REG,
							TYPEC_SPARE_CFG_BIT, 0);
		if (rc < 0)
			smblib_err(chg, "Couldn't enable HW cc_out rc=%d\n",
									rc);

		/*
		 * This WA should only run for HVDCP. Non-legacy SDP/CDP could
		 * draw more, but this WA will remove Rd causing VBUS to drop,
		 * and data could be interrupted. Non-legacy DCP could also draw
		 * more, but it may impact compliance.
		 */
		sink_attached = chg->typec_status[3] & UFP_DFP_MODE_STATUS_BIT;
		if (!chg->typec_legacy_valid && !sink_attached && hvdcp)
			schedule_work(&chg->legacy_detection_work);
#ifdef CONFIG_LGE_PM
		if (!chg->typec_legacy_valid &&
				!sink_attached && !hvdcp && chg->retry_legacy_detection == RETRY_APSD_RERUN) {
			chg->retry_legacy_detection = RETRY_LEGACY_CABLE;
			vote(chg->pseudo_usb_type_votable, RETRY_LEGACY_VOTER, true, PSEUDO_USB_TYPE);
			smblib_rerun_apsd(chg);
		}
#endif
	}

#ifdef CONFIG_LGE_USB
skip_pd_active:
#endif
	smblib_update_usb_type(chg);
	power_supply_changed(chg->usb_psy);
	return rc;
}

int smblib_set_prop_ship_mode(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;

	smblib_dbg(chg, PR_MISC, "Set ship mode: %d!!\n", !!val->intval);

	rc = smblib_masked_write(chg, SHIP_MODE_REG, SHIP_MODE_EN_BIT,
			!!val->intval ? SHIP_MODE_EN_BIT : 0);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't %s ship mode, rc=%d\n",
				!!val->intval ? "enable" : "disable", rc);

	return rc;
}

int smblib_reg_block_update(struct smb_charger *chg,
				struct reg_info *entry)
{
	int rc = 0;

	while (entry && entry->reg) {
		rc = smblib_read(chg, entry->reg, &entry->bak);
		if (rc < 0) {
			dev_err(chg->dev, "Error in reading %s rc=%d\n",
				entry->desc, rc);
			break;
		}
		entry->bak &= entry->mask;

		rc = smblib_masked_write(chg, entry->reg,
					 entry->mask, entry->val);
		if (rc < 0) {
			dev_err(chg->dev, "Error in writing %s rc=%d\n",
				entry->desc, rc);
			break;
		}
		entry++;
	}

	return rc;
}

int smblib_reg_block_restore(struct smb_charger *chg,
				struct reg_info *entry)
{
	int rc = 0;

	while (entry && entry->reg) {
		rc = smblib_masked_write(chg, entry->reg,
					 entry->mask, entry->bak);
		if (rc < 0) {
			dev_err(chg->dev, "Error in writing %s rc=%d\n",
				entry->desc, rc);
			break;
		}
		entry++;
	}

	return rc;
}

static struct reg_info cc2_detach_settings[] = {
	{
		.reg	= TYPE_C_CFG_2_REG,
		.mask	= TYPE_C_UFP_MODE_BIT | EN_TRY_SOURCE_MODE_BIT,
		.val	= TYPE_C_UFP_MODE_BIT,
		.desc	= "TYPE_C_CFG_2_REG",
	},
	{
		.reg	= TYPE_C_CFG_3_REG,
		.mask	= EN_TRYSINK_MODE_BIT,
		.val	= 0,
		.desc	= "TYPE_C_CFG_3_REG",
	},
	{
		.reg	= TAPER_TIMER_SEL_CFG_REG,
		.mask	= TYPEC_SPARE_CFG_BIT,
		.val	= TYPEC_SPARE_CFG_BIT,
		.desc	= "TAPER_TIMER_SEL_CFG_REG",
	},
	{
		.reg	= TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
		.mask	= VCONN_EN_ORIENTATION_BIT,
		.val	= 0,
		.desc	= "TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG",
	},
	{
		.reg	= MISC_CFG_REG,
		.mask	= TCC_DEBOUNCE_20MS_BIT,
		.val	= TCC_DEBOUNCE_20MS_BIT,
		.desc	= "Tccdebounce time"
	},
	{
	},
};

static int smblib_cc2_sink_removal_enter(struct smb_charger *chg)
{
	int rc, ccout, ufp_mode;
	u8 stat;

#ifdef CONFIG_LGE_USB
	if (chg->pd_active)
		return 0;
#endif

	if ((chg->wa_flags & TYPEC_CC2_REMOVAL_WA_BIT) == 0)
		return 0;

	if (chg->cc2_detach_wa_active)
		return 0;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}

	ccout = (stat & CC_ATTACHED_BIT) ?
					(!!(stat & CC_ORIENTATION_BIT) + 1) : 0;
	ufp_mode = (stat & TYPEC_DEBOUNCE_DONE_STATUS_BIT) ?
					!(stat & UFP_DFP_MODE_STATUS_BIT) : 0;

	if (ccout != 2)
		return 0;

	if (!ufp_mode)
		return 0;

	chg->cc2_detach_wa_active = true;
	/* The CC2 removal WA will cause a type-c-change IRQ storm */
	smblib_reg_block_update(chg, cc2_detach_settings);
	schedule_work(&chg->rdstd_cc2_detach_work);
	return rc;
}

static int smblib_cc2_sink_removal_exit(struct smb_charger *chg)
{
	if ((chg->wa_flags & TYPEC_CC2_REMOVAL_WA_BIT) == 0)
		return 0;

	if (!chg->cc2_detach_wa_active)
		return 0;

	chg->cc2_detach_wa_active = false;
	cancel_work_sync(&chg->rdstd_cc2_detach_work);
	smblib_reg_block_restore(chg, cc2_detach_settings);
	return 0;
}

int smblib_set_prop_pd_in_hard_reset(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc = 0;
#ifdef CONFIG_LGE_PM
	union power_supply_propval pval = {0, };
#endif

	if (chg->pd_hard_reset == val->intval)
		return rc;

	chg->pd_hard_reset = val->intval;
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
			EXIT_SNK_BASED_ON_CC_BIT,
			(chg->pd_hard_reset) ? EXIT_SNK_BASED_ON_CC_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't set EXIT_SNK_BASED_ON_CC rc=%d\n",
				rc);

	vote(chg->apsd_disable_votable, PD_HARD_RESET_VOTER,
							chg->pd_hard_reset, 0);

#ifdef CONFIG_LGE_PM
	if (chg->pd_hard_reset) {
		chg->checking_pd_active = true;
		if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_FLOATED)
			power_supply_changed(chg->usb_psy);
	}
	vote(chg->pseudo_usb_type_votable, PD_HARD_RESET_VOTER, chg->pd_hard_reset, PSEUDO_USB_TYPE);

	rc = smblib_get_prop_usb_present(chg, &pval);
	if (!pval.intval && !chg->pd_hard_reset) {
		smblib_update_usb_type(chg);
		power_supply_changed(chg->usb_psy);
	}

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_DCP
			&& !chg->is_hvdcp_timeout && chg->pd_hard_reset ) {
		chg->retry_legacy_detection = RETRY_APSD_RERUN;
	}

	if (pval.intval && !chg->pd_hard_reset
			&& (chg->real_charger_type == POWER_SUPPLY_TYPE_UNKNOWN || smblib_get_aicl_fail(chg) == 1)) {
		smblib_rerun_apsd(chg);
	}

	smblib_dbg(chg, PR_LGE, "update pd_hard_reset %d\n", chg->pd_hard_reset);
#endif
	return rc;
}

#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
int smblib_set_prop_typec_cc_disable(struct smb_charger *chg,
		const union power_supply_propval *val)
{
	int rc = 0;

	switch (val->intval) {
	case 0:
		rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				UFP_EN_CMD_BIT | TYPEC_DISABLE_CMD_BIT, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't write 0x%x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
					val->intval, rc);
			return rc;
		}
		break;
	case 1:
#if 0
		/* Not use TYPEC_DISABLE_CMD_BIT */
		rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				TYPEC_DISABLE_CMD_BIT, val->intval);
#endif
		rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				TYPEC_DISABLE_CMD_BIT, TYPEC_DISABLE_CMD_BIT);
		usleep_range(10000, 11000);
		rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				UFP_EN_CMD_BIT | TYPEC_DISABLE_CMD_BIT, UFP_EN_CMD_BIT);

		if (rc < 0) {
			smblib_err(chg, "Couldn't write 0x%x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
					val->intval, rc);
			return rc;
		}
		break;
	default:
		smblib_err(chg, "cc disable %d not supported\n", val->intval);
		return -EINVAL;
	}

	if (rc < 0) {
		smblib_err(chg, "Couldn't write 0x%x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
				val->intval, rc);
		return rc;
	}

	return rc;
}
#endif

/***********************
* USB MAIN PSY GETTERS *
*************************/
int smblib_get_prop_fcc_delta(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc, jeita_cc_delta_ua, step_cc_delta_ua, hw_cc_delta_ua = 0;

	rc = smblib_get_step_cc_delta(chg, &step_cc_delta_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get step cc delta rc=%d\n", rc);
		step_cc_delta_ua = 0;
	} else {
		hw_cc_delta_ua = step_cc_delta_ua;
	}

	rc = smblib_get_jeita_cc_delta(chg, &jeita_cc_delta_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get jeita cc delta rc=%d\n", rc);
		jeita_cc_delta_ua = 0;
	} else if (jeita_cc_delta_ua < 0) {
		/* HW will take the min between JEITA and step charge */
		hw_cc_delta_ua = min(hw_cc_delta_ua, jeita_cc_delta_ua);
	}

	val->intval = hw_cc_delta_ua;
	return 0;
}

/***********************
* USB MAIN PSY SETTERS *
*************************/

#ifdef CONFIG_LGE_PM
#define SDP_CURRENT_UA			500000
#define CDP_CURRENT_UA			1500000
#define DCP_CURRENT_UA			1800000
#define HVDCP_CURRENT_UA		3000000
#define TYPEC_DEFAULT_CURRENT_UA	900000
#define TYPEC_MEDIUM_CURRENT_UA		1800000
#define TYPEC_HIGH_CURRENT_UA		2000000
#else
#define SDP_CURRENT_MA			500000
#define CDP_CURRENT_MA			1500000
#define DCP_CURRENT_MA			1500000
#define HVDCP_CURRENT_MA		3000000
#define TYPEC_DEFAULT_CURRENT_MA	900000
#define TYPEC_MEDIUM_CURRENT_MA		1500000
#define TYPEC_HIGH_CURRENT_MA		3000000
#endif
int smblib_get_charge_current(struct smb_charger *chg,
				int *total_current_ua)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);
	union power_supply_propval val = {0, };
	int rc = 0, typec_source_rd, current_ua;
	bool non_compliant;
	u8 stat5;

	if (chg->pd_active) {
		*total_current_ua =
			get_client_vote_locked(chg->usb_icl_votable, PD_VOTER);
		return rc;
	}

	rc = smblib_read(chg, TYPE_C_STATUS_5_REG, &stat5);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_5 rc=%d\n", rc);
		return rc;
	}
	non_compliant = stat5 & TYPEC_NONCOMP_LEGACY_CABLE_STATUS_BIT;

	/* get settled ICL */
	rc = smblib_get_prop_input_current_settled(chg, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get settled ICL rc=%d\n", rc);
		return rc;
	}

#ifdef CONFIG_LGE_USB_COMPLIANCE_TEST
	if (apsd_result->bit == FLOAT_CHARGER_BIT) {
		*total_current_ua = SDP_CURRENT_MA;
		return 0;
	}
#endif

	typec_source_rd = smblib_get_prop_ufp_mode(chg);

	/* QC 2.0/3.0 adapter */
	if (apsd_result->bit & (QC_3P0_BIT | QC_2P0_BIT)) {
#ifdef CONFIG_LGE_PM
		*total_current_ua = HVDCP_CURRENT_UA;
#else
		*total_current_ua = HVDCP_CURRENT_MA;
#endif
		return 0;
	}

	if (non_compliant) {
		switch (apsd_result->bit) {
		case CDP_CHARGER_BIT:
#ifdef CONFIG_LGE_PM
			current_ua = CDP_CURRENT_UA;
#else
			current_ua = CDP_CURRENT_MA;
#endif
			break;
		case DCP_CHARGER_BIT:
		case OCP_CHARGER_BIT:
		case FLOAT_CHARGER_BIT:
#ifdef CONFIG_LGE_PM
			current_ua = DCP_CURRENT_UA;
#else
			current_ua = DCP_CURRENT_MA;
#endif
			break;
		default:
			current_ua = 0;
			break;
		}

		*total_current_ua = max(current_ua, val.intval);
		return 0;
	}

	switch (typec_source_rd) {
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
		switch (apsd_result->bit) {
		case CDP_CHARGER_BIT:
#ifdef CONFIG_LGE_PM
			current_ua = CDP_CURRENT_UA;
#else
			current_ua = CDP_CURRENT_MA;
#endif
			break;
		case DCP_CHARGER_BIT:
		case OCP_CHARGER_BIT:
		case FLOAT_CHARGER_BIT:
			current_ua = chg->default_icl_ua;
			break;
		default:
			current_ua = 0;
			break;
		}
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
#ifdef CONFIG_LGE_PM
		current_ua = TYPEC_MEDIUM_CURRENT_UA;
#else
		current_ua = TYPEC_MEDIUM_CURRENT_MA;
#endif
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
#ifdef CONFIG_LGE_PM
		current_ua = TYPEC_HIGH_CURRENT_UA;
#else
		current_ua = TYPEC_HIGH_CURRENT_MA;
#endif
		break;
	case POWER_SUPPLY_TYPEC_NON_COMPLIANT:
	case POWER_SUPPLY_TYPEC_NONE:
	default:
		current_ua = 0;
		break;
	}

	*total_current_ua = max(current_ua, val.intval);
	return 0;
}

/************************
 * PARALLEL PSY GETTERS *
 ************************/

int smblib_get_prop_slave_current_now(struct smb_charger *chg,
		union power_supply_propval *pval)
{
	if (IS_ERR_OR_NULL(chg->iio.batt_i_chan))
		chg->iio.batt_i_chan = iio_channel_get(chg->dev, "batt_i");

	if (IS_ERR(chg->iio.batt_i_chan))
		return PTR_ERR(chg->iio.batt_i_chan);

	return iio_read_channel_processed(chg->iio.batt_i_chan, &pval->intval);
}

/**********************
 * INTERRUPT HANDLERS *
 **********************/

irqreturn_t smblib_handle_debug(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	return IRQ_HANDLED;
}

#ifdef CONFIG_LGE_PM_DEBUG
irqreturn_t smblib_handle_lge_debug(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_LGE, "IRQ: %s\n", irq_data->name);
	return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_LGE_PM
irqreturn_t smblib_handle_aicl_fail(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int apsd_disable, aicl_fail;

	apsd_disable = get_effective_result_locked(chg->apsd_disable_votable);
	aicl_fail = smblib_get_aicl_fail(chg);

	smblib_dbg(chg, PR_LGE, "IRQ: %s %s - apsd_disable(%d)\n", irq_data->name, aicl_fail ? "rising" : "falling", apsd_disable);
	if (!apsd_disable && aicl_fail)
		smblib_rerun_apsd(chg);

	return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
#define VZW_UNDER_CHARGING_CURRENT		450000
irqreturn_t smblib_handle_vzw_debug(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	union power_supply_propval pval = {0, };

	if (delayed_work_pending(&chg->vzw_aicl_done_work))
		cancel_delayed_work_sync(&chg->vzw_aicl_done_work);

	smblib_get_prop_input_current_settled(chg, &pval);
	smblib_dbg(chg, PR_LGE, "IRQ: %s(%d)\n", irq_data->name, pval.intval);
	if (pval.intval < VZW_UNDER_CHARGING_CURRENT) {
		schedule_delayed_work(&chg->vzw_aicl_done_work, msecs_to_jiffies(500));
	}

	return IRQ_HANDLED;
}
#endif

irqreturn_t smblib_handle_otg_overcurrent(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	u8 stat;

	rc = smblib_read(chg, OTG_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read OTG_INT_RT_STS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	if (chg->wa_flags & OTG_WA) {
		if (stat & OTG_OC_DIS_SW_STS_RT_STS_BIT)
			smblib_err(chg, "OTG disabled by hw\n");

		/* not handling software based hiccups for PM660 */
		return IRQ_HANDLED;
	}
#ifdef CONFIG_LGE_PM_DEBUG
	smblib_dbg(chg, PR_LGE, "IRQ: %s, OTG_INT_RT_STS = %d\n", irq_data->name, stat);
#endif

	if (stat & OTG_OVERCURRENT_RT_STS_BIT)
		schedule_work(&chg->otg_oc_work);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_chg_state_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	u8 stat;
	int rc;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
	union lge_power_propval lge_val = {0,};
#endif

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
				rc);
		return IRQ_HANDLED;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
	smblib_dbg(chg, PR_LGE, "chg_state_change to %d\n", stat);

	if (!chg->lge_cc_lpc)
		chg->lge_cc_lpc = lge_power_get_by_name("lge_cc");
	if (chg->lge_cc_lpc) {
		if (stat == TERMINATE_CHARGE)
			lge_val.intval = 1;
		else
			lge_val.intval = 0;
		rc = chg->lge_cc_lpc->set_property(chg->lge_cc_lpc,
				LGE_POWER_PROP_CHARGE_DONE, &lge_val);
	}
#endif

	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_step_chg_state_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

#ifdef CONFIG_LGE_PM_DEBUG
	smblib_dbg(chg, PR_LGE, "IRQ: %s\n", irq_data->name);
#else
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
#endif

	if (chg->step_chg_enabled)
		rerun_election(chg->fcc_votable);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_step_chg_soc_update_fail(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

#ifdef CONFIG_LGE_PM_DEBUG
	smblib_dbg(chg, PR_LGE, "IRQ: %s\n", irq_data->name);
#else
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
#endif

	if (chg->step_chg_enabled)
		rerun_election(chg->fcc_votable);

	return IRQ_HANDLED;
}

#define STEP_SOC_REQ_MS	3000
irqreturn_t smblib_handle_step_chg_soc_update_request(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	union power_supply_propval pval = {0, };

#ifdef CONFIG_LGE_PM_DEBUG
	smblib_dbg(chg, PR_LGE, "IRQ: %s\n", irq_data->name);
#else
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
#endif

	if (!chg->bms_psy) {
		schedule_delayed_work(&chg->step_soc_req_work,
				      msecs_to_jiffies(STEP_SOC_REQ_MS));
		return IRQ_HANDLED;
	}

	rc = smblib_get_prop_batt_capacity(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "Couldn't get batt capacity rc=%d\n", rc);
	else
		step_charge_soc_update(chg, pval.intval);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_batt_temp_changed(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	rerun_election(chg->fcc_votable);
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_batt_psy_changed(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

#ifdef CONFIG_LGE_PM_DEBUG
	smblib_dbg(chg, PR_LGE, "IRQ: %s\n", irq_data->name);
#else
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
#endif

	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

#ifdef CONFIG_IDTP9223_CHARGER
irqreturn_t smblib_handle_batt_qipma_on(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc = 0;
	u8 stat;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	rc = smblib_read(chg, BATIF_BASE + INT_RT_STS_OFFSET, &stat);
	smblib_dbg(chg, PR_INTERRUPT, "idtp9223 qi_pma_on pin value=%d\n", stat);

	return IRQ_HANDLED;
}
#endif

irqreturn_t smblib_handle_usb_psy_changed(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	power_supply_changed(chg->usb_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_usbin_uv(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct storm_watch *wdata;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	if (!chg->irq_info[SWITCH_POWER_OK_IRQ].irq_data)
		return IRQ_HANDLED;

	wdata = &chg->irq_info[SWITCH_POWER_OK_IRQ].irq_data->storm_data;
	reset_storm_count(wdata);
	return IRQ_HANDLED;
}

#ifdef CONFIG_LGE_PM_WEAK_BATT_PACK
#define DEFAULT_WEAK_ICL_MA 1000000
#define WEAK_CHG_DET_CNT	3
#define POWER_PATH_USB		BIT(2)
#define POWER_PATH_BATTERY	BIT(1)
static void weak_batt_pack_check_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
					       batt_pack_check_work.work);
	int rc;
	u8 stat;
	static s64 last_time;
	s64 current_time;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	current_time = ktime_to_ms(chg->irq_info[SWITCH_POWER_OK_IRQ].irq_data->storm_data.last_kt);
	if (current_time == last_time) {
		smblib_dbg(chg, PR_LGE,"time out weak batt count clear\n");
		chg->batt_pack_verify_cnt = 0;
	} else if (rc >= 0 && (stat & POWER_PATH_MASK) == POWER_PATH_USB) {
		chg->batt_pack_verify_cnt++;
		smblib_dbg(chg, PR_LGE,"weak battery pack is detected, count = %d\n",
				chg->batt_pack_verify_cnt);
		last_time = current_time;
		if (chg->batt_pack_verify_cnt >= WEAK_CHG_DET_CNT) {
			smblib_dbg(chg, PR_LGE,"weak batt pack detected, set ICL to 1A\n");
			vote(chg->usb_icl_votable,
					WEAK_BATT_VOTER, true, DEFAULT_WEAK_ICL_MA);
		}
		schedule_delayed_work(&chg->batt_pack_check_work,
					msecs_to_jiffies(5000));
	} else {
		smblib_dbg(chg, PR_LGE,"usb is removed. reset count\n");
		chg->batt_pack_verify_cnt = 0;
		vote(chg->usb_icl_votable, WEAK_BATT_VOTER, false, 0);
	}
}
#endif

static void smblib_micro_usb_plugin(struct smb_charger *chg, bool vbus_rising)
{
	if (vbus_rising) {
		/* use the typec flag even though its not typec */
		chg->typec_present = 1;
	} else {
		chg->typec_present = 0;
		smblib_update_usb_type(chg);
		extcon_set_cable_state_(chg->extcon, EXTCON_USB, false);
		smblib_uusb_removal(chg);
	}
}

#ifdef CONFIG_LGE_PM
static void smblib_lge_usb_removal(struct smb_charger *chg){
	int rc;

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
	if (delayed_work_pending(&chg->vzw_aicl_done_work))
		cancel_delayed_work_sync(&chg->vzw_aicl_done_work);
	if (delayed_work_pending(&chg->vzw_recheck_work))
		cancel_delayed_work_sync(&chg->vzw_recheck_work);
	chg->vzw_icl_change = false;
#endif
#ifdef CONFIG_LGE_PM_BATT_MANAGER
	lge_set_bm_status(chg, BM_MONITOR_STOP);
#endif
	cancel_delayed_work_sync(&chg->icl_change_work);
	vote(chg->pseudo_usb_type_votable, PD_VOTER, false, PSEUDO_USB_TYPE);
	vote(chg->pseudo_usb_type_votable, ABNORMAL_GENDER_VOTER, false, PSEUDO_USB_TYPE);
	vote(chg->pseudo_usb_type_votable, APSD_RERUN_VOTER, false, PSEUDO_HVDCP);
	vote(chg->pseudo_usb_type_votable, RETRY_LEGACY_VOTER, false, PSEUDO_USB_TYPE);
	vote(chg->pseudo_usb_type_votable, FAST_HVDCP_DETECTION_VOTER, false, PSEUDO_HVDCP);
	if (!is_client_vote_enabled_locked(chg->pseudo_usb_type_votable, LEGACY_CABLE_VOTER))
		chg->pseudo_usb_type = POWER_SUPPLY_TYPE_UNKNOWN;
	if (get_client_vote_locked(chg->pseudo_usb_type_votable, LEGACY_CABLE_VOTER) == PSEUDO_HVDCP)
		vote(chg->pseudo_usb_type_votable, LEGACY_CABLE_VOTER, false, PSEUDO_USB_TYPE);
	cancel_delayed_work_sync(&chg->recovery_boost_back_work);
	cancel_delayed_work_sync(&chg->abnormal_gender_detect_work);
	if (chg->pd_hard_reset && chg->wa_flags & BOOST_BACK_WA)
		vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);

	chg->checking_pd_active = false;
	chg->pre_settled_ua = -1;
	if (chg->is_abnormal_gendor && !chg->pd_hard_reset) {
		smblib_update_usb_type(chg);
		chg->is_abnormal_gendor = false;
	}
	if (!chg->pd_hard_reset)
		chg->retry_legacy_detection = WAIT_PD_HARD_RESET;

	rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
			USBIN_AICL_RERUN_EN_BIT, 0);
	if (rc < 0)
		smblib_err(chg,
			"Couldn't enable AICL_RERUTN rc=%d\n", rc);

	if (!chg->pd_active && !chg->pd_hard_reset &&
			chg->real_charger_type == POWER_SUPPLY_TYPE_USB_FLOATED) {
		vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER, true, 0);
		smblib_update_usb_type(chg);
	}
}

#if defined (CONFIG_LGE_TOUCH_CORE)
extern void touch_notify_connect(u32 type);
#endif
static void smblib_lge_usb_plugin(struct smb_charger *chg){
	int rc;
	int typec_mode = 0;
	bool vbus_rising;
	static bool pre_vbus_rising;
	union power_supply_propval val = {0, };

	rc = smblib_get_prop_usb_present_raw(chg, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read usb present rc=%d\n", rc);
		return;
	}
	vbus_rising = val.intval;

#if defined (CONFIG_LGE_TOUCH_CORE)
	touch_notify_connect(vbus_rising);
#endif

	if (vbus_rising) {
		if(pre_vbus_rising) {
			smblib_lge_usb_removal(chg);
			smblib_dbg(chg, PR_LGE, "smblib_lge_usb_removal\n");
		}

		if (!chg->pd_hard_reset) {
			chg->retry_legacy_detection = WAIT_PD_HARD_RESET;
			chg->is_abnormal_gendor = false;

			typec_mode = smblib_get_prop_typec_mode(chg);
			if (chg->typec_mode == POWER_SUPPLY_TYPEC_NONE) {
				cancel_delayed_work_sync(&chg->abnormal_gender_detect_work);
				schedule_delayed_work(&chg->abnormal_gender_detect_work,
							msecs_to_jiffies(1000));
			}
		}
#ifdef CONFIG_LGE_PM_BATT_MANAGER
		lge_set_bm_status(chg, BM_MONITOR_START);
#endif
	} else {
		smblib_lge_usb_removal(chg);
	}

	pre_vbus_rising = vbus_rising;
}
#endif

void smblib_usb_plugin_hard_reset_locked(struct smb_charger *chg)
{
	int rc;
	u8 stat;
	bool vbus_rising;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return;
	}

	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

	if (vbus_rising)
		smblib_cc2_sink_removal_exit(chg);
	else
		smblib_cc2_sink_removal_enter(chg);

	power_supply_changed(chg->usb_psy);
#ifdef CONFIG_LGE_PM
	smblib_dbg(chg, PR_LGE, "IRQ: usbin-plugin %s\n",
					vbus_rising ? "attached" : "detached");
#else
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: usbin-plugin %s\n",
					vbus_rising ? "attached" : "detached");
#endif
}

#define PL_DELAY_MS			30000
void smblib_usb_plugin_locked(struct smb_charger *chg)
{
	int rc;
	u8 stat;
	bool vbus_rising;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return;
	}

	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	smblib_set_opt_freq_buck(chg, vbus_rising ? chg->chg_freq.freq_5V :
						chg->chg_freq.freq_removal);

	/* fetch the DPDM regulator */
	if (!chg->dpdm_reg && of_get_property(chg->dev->of_node,
						"dpdm-supply", NULL)) {
		chg->dpdm_reg = devm_regulator_get(chg->dev, "dpdm");
		if (IS_ERR(chg->dpdm_reg)) {
			smblib_err(chg, "Couldn't get dpdm regulator rc=%ld\n",
				PTR_ERR(chg->dpdm_reg));
			chg->dpdm_reg = NULL;
		}
	}

	if (vbus_rising) {
		if (chg->dpdm_reg && !regulator_is_enabled(chg->dpdm_reg)) {
			smblib_dbg(chg, PR_MISC, "enabling DPDM regulator\n");
			rc = regulator_enable(chg->dpdm_reg);
		if (rc < 0)
				smblib_err(chg, "Couldn't enable dpdm regulator rc=%d\n",
					rc);
		}

		/* Schedule work to enable parallel charger */
		vote(chg->awake_votable, PL_DELAY_VOTER, true, 0);
		schedule_delayed_work(&chg->pl_enable_work,
					msecs_to_jiffies(PL_DELAY_MS));
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
		if (!chg->vzw_icl_change && !delayed_work_pending(&chg->vzw_recheck_work))
			schedule_delayed_work(&chg->vzw_recheck_work, msecs_to_jiffies(4000));
#endif
	} else {
		if (chg->wa_flags & BOOST_BACK_WA)
			vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);

		if (chg->dpdm_reg && regulator_is_enabled(chg->dpdm_reg)) {
			smblib_dbg(chg, PR_MISC, "disabling DPDM regulator\n");
			rc = regulator_disable(chg->dpdm_reg);
		if (rc < 0)
				smblib_err(chg, "Couldn't disable dpdm regulator rc=%d\n",
					rc);
		}
	}

	if (chg->micro_usb_mode)
		smblib_micro_usb_plugin(chg, vbus_rising);

	power_supply_changed(chg->usb_psy);
#ifdef CONFIG_LGE_PM_DEBUG
	smblib_dbg(chg, PR_LGE, "IRQ: usbin-plugin %s\n",
					vbus_rising ? "attached" : "detached");
#else
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: usbin-plugin %s\n",
					vbus_rising ? "attached" : "detached");
#endif
}

irqreturn_t smblib_handle_usb_plugin(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	mutex_lock(&chg->lock);
#ifdef CONFIG_LGE_PM
	smblib_lge_usb_plugin(chg);
#endif
	if (chg->pd_hard_reset)
		smblib_usb_plugin_hard_reset_locked(chg);
	else
		smblib_usb_plugin_locked(chg);
	mutex_unlock(&chg->lock);
	return IRQ_HANDLED;
}

#define USB_WEAK_INPUT_UA	1400000
#define ICL_CHANGE_DELAY_MS	1000
irqreturn_t smblib_handle_icl_change(int irq, void *data)
{
	u8 stat;
	int rc, settled_ua, delay = ICL_CHANGE_DELAY_MS;
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	if (chg->mode == PARALLEL_MASTER) {
		rc = smblib_read(chg, AICL_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read AICL_STATUS rc=%d\n",
					rc);
			return IRQ_HANDLED;
		}

		rc = smblib_get_charge_param(chg, &chg->param.icl_stat,
				&settled_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get ICL status rc=%d\n", rc);
			return IRQ_HANDLED;
		}

#ifdef CONFIG_LGE_PM
		/* If AICL settled then schedule work now */
		if (settled_ua == get_effective_result(chg->usb_icl_votable))
#else
		/* If AICL settled then schedule work now */
		if ((settled_ua == get_effective_result(chg->usb_icl_votable))
				|| (stat & AICL_DONE_BIT))
#endif
			delay = 0;

		cancel_delayed_work_sync(&chg->icl_change_work);
		schedule_delayed_work(&chg->icl_change_work,
						msecs_to_jiffies(delay));
	}

	return IRQ_HANDLED;
}

static void smblib_handle_slow_plugin_timeout(struct smb_charger *chg,
					      bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: slow-plugin-timeout %s\n",
		   rising ? "rising" : "falling");
}

static void smblib_handle_sdp_enumeration_done(struct smb_charger *chg,
					       bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: sdp-enumeration-done %s\n",
		   rising ? "rising" : "falling");
}

#define QC3_PULSES_FOR_6V	5
#define QC3_PULSES_FOR_9V	20
#define QC3_PULSES_FOR_12V	35
static void smblib_hvdcp_adaptive_voltage_change(struct smb_charger *chg)
{
	int rc;
	u8 stat;
	int pulses;

	power_supply_changed(chg->usb_main_psy);
	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP) {
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_CHANGE_STATUS rc=%d\n", rc);
			return;
		}

		switch (stat & QC_2P0_STATUS_MASK) {
		case QC_5V_BIT:
			smblib_set_opt_freq_buck(chg,
					chg->chg_freq.freq_5V);
			break;
		case QC_9V_BIT:
			smblib_set_opt_freq_buck(chg,
					chg->chg_freq.freq_9V);
			break;
		case QC_12V_BIT:
			smblib_set_opt_freq_buck(chg,
					chg->chg_freq.freq_12V);
			break;
		default:
			smblib_set_opt_freq_buck(chg,
					chg->chg_freq.freq_removal);
			break;
		}
	}

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3) {
		rc = smblib_read(chg, QC_PULSE_COUNT_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_PULSE_COUNT rc=%d\n", rc);
			return;
		}
		pulses = (stat & QC_PULSE_COUNT_MASK);

		if (pulses < QC3_PULSES_FOR_6V)
			smblib_set_opt_freq_buck(chg,
				chg->chg_freq.freq_5V);
		else if (pulses < QC3_PULSES_FOR_9V)
			smblib_set_opt_freq_buck(chg,
				chg->chg_freq.freq_6V_8V);
		else if (pulses < QC3_PULSES_FOR_12V)
			smblib_set_opt_freq_buck(chg,
				chg->chg_freq.freq_9V);
		else
			smblib_set_opt_freq_buck(chg,
				chg->chg_freq.freq_12V);
	}
}

/* triggers when HVDCP 3.0 authentication has finished */
static void smblib_handle_hvdcp_3p0_auth_done(struct smb_charger *chg,
					      bool rising)
{
	const struct apsd_result *apsd_result;
	int rc;

	if (!rising)
		return;

	if (chg->wa_flags & QC_AUTH_INTERRUPT_WA_BIT) {
		/*
		 * Disable AUTH_IRQ_EN_CFG_BIT to receive adapter voltage
		 * change interrupt.
		 */
		rc = smblib_masked_write(chg,
				USBIN_SOURCE_CHANGE_INTRPT_ENB_REG,
				AUTH_IRQ_EN_CFG_BIT, 0);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't enable QC auth setting rc=%d\n", rc);
	}

	if (chg->mode == PARALLEL_MASTER)
		vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, true, 0);

	/* the APSD done handler will set the USB supply type */
	apsd_result = smblib_get_apsd_result(chg);
	if (get_effective_result(chg->hvdcp_hw_inov_dis_votable)) {
		if (apsd_result->pst == POWER_SUPPLY_TYPE_USB_HVDCP) {
			/* force HVDCP2 to 9V if INOV is disabled */
			rc = smblib_masked_write(chg, CMD_HVDCP_2_REG,
					FORCE_9V_BIT, FORCE_9V_BIT);
			if (rc < 0)
				smblib_err(chg,
					"Couldn't force 9V HVDCP rc=%d\n", rc);
		}
	}

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-3p0-auth-done rising; %s detected\n",
		   apsd_result->name);
}

static void smblib_handle_hvdcp_check_timeout(struct smb_charger *chg,
					      bool rising, bool qc_charger)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);

	/* Hold off PD only until hvdcp 2.0 detection timeout */
	if (rising) {
		vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
								false, 0);

		/* enable HDC and ICL irq for QC2/3 charger */
		if (qc_charger)
			vote(chg->usb_irq_enable_votable, QC_VOTER, true, 0);

		/*
		 * HVDCP detection timeout done
		 * If adapter is not QC2.0/QC3.0 - it is a plain old DCP.
		 */
		if (!qc_charger && (apsd_result->bit & DCP_CHARGER_BIT))
#ifdef CONFIG_LGE_PM
			/* enforce DCP ICL if specified */
			vote(chg->usb_icl_votable, DCP_VOTER,
				(chg->dcp_icl_ua != -EINVAL && chg->typec_mode != POWER_SUPPLY_TYPEC_SOURCE_HIGH),
				chg->dcp_icl_ua);
		if (!qc_charger && (apsd_result->bit & DCP_CHARGER_BIT)) {
			int rc;

			rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
					USBIN_AICL_RERUN_EN_BIT, USBIN_AICL_RERUN_EN_BIT);
			if (rc < 0)
				smblib_err(chg,
					"Couldn't enable AICL_RERUTN rc=%d\n", rc);
		}
#else
			/* enforce DCP ICL if specified */
			vote(chg->usb_icl_votable, DCP_VOTER,
				chg->dcp_icl_ua != -EINVAL, chg->dcp_icl_ua);
#endif
	}
#ifdef CONFIG_LGE_PM
	chg->is_hvdcp_timeout = rising;
#endif

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: smblib_handle_hvdcp_check_timeout %s\n",
		   rising ? "rising" : "falling");
}

/* triggers when HVDCP is detected */
static void smblib_handle_hvdcp_detect_done(struct smb_charger *chg,
					    bool rising)
{
	if (!rising)
		return;

	/* the APSD done handler will set the USB supply type */
	cancel_delayed_work_sync(&chg->hvdcp_detect_work);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-detect-done %s\n",
		   rising ? "rising" : "falling");
}

#ifdef CONFIG_LGE_PM
static int get_rp_based_dcp_current(struct smb_charger *chg, int typec_mode)
{
	int rp_ua;

	switch (typec_mode) {
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		rp_ua = TYPEC_HIGH_CURRENT_UA;
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
	/* fall through */
	default:
		rp_ua = DCP_CURRENT_UA;
	}

	return rp_ua;
}
#endif

static void smblib_force_legacy_icl(struct smb_charger *chg, int pst)
{
#ifdef CONFIG_LGE_PM
	int typec_mode;
	int rp_ua;
#endif

	/* while PD is active it should have complete ICL control */
	if (chg->pd_active)
		return;

	switch (pst) {
	case POWER_SUPPLY_TYPE_USB:
		/*
		 * USB_PSY will vote to increase the current to 500/900mA once
		 * enumeration is done. Ensure that USB_PSY has at least voted
		 * for 100mA before releasing the LEGACY_UNKNOWN vote
		 */
		if (!is_client_vote_enabled(chg->usb_icl_votable,
								USB_PSY_VOTER))
			vote(chg->usb_icl_votable, USB_PSY_VOTER, true, 100000);
		vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, false, 0);
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, true, 1500000);
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
#ifdef CONFIG_LGE_PM
		if (chg->fake_hvdcp_mode && chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP) {
			vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, true, 1800000);
		} else {
			 typec_mode = smblib_get_prop_typec_mode(chg);
			 rp_ua = get_rp_based_dcp_current(chg, typec_mode);
			 vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, true, rp_ua);
		}
#else
		vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, true, 1500000);
#endif
		break;
	case POWER_SUPPLY_TYPE_USB_HVDCP:
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
#ifdef CONFIG_LGE_PM
		if (chg->typec_legacy_valid && get_effective_result(chg->hvdcp_disable_votable_indirect)) {
			typec_mode = smblib_get_prop_typec_mode(chg);
			rp_ua = get_rp_based_dcp_current(chg, typec_mode);
			vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, true, rp_ua);
		} else {
			vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, true, 1800000);
		}
#else
		vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, true, 3000000);
#endif
		break;
	default:
		smblib_err(chg, "Unknown APSD %d; forcing 500mA\n", pst);
		vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, true, 500000);
		break;
	}
}

#define HVDCP_DET_MS 2500
static void smblib_handle_apsd_done(struct smb_charger *chg, bool rising)
{
	const struct apsd_result *apsd_result;

	if (!rising)
		return;

	apsd_result = smblib_update_usb_type(chg);

	if (!chg->typec_legacy_valid)
		smblib_force_legacy_icl(chg, apsd_result->pst);

	switch (apsd_result->bit) {
	case SDP_CHARGER_BIT:
	case CDP_CHARGER_BIT:
		if (chg->micro_usb_mode)
			extcon_set_cable_state_(chg->extcon, EXTCON_USB,
					true);
	case OCP_CHARGER_BIT:
	case FLOAT_CHARGER_BIT:
		/* if not DCP then no hvdcp timeout happens, Enable pd here. */
		vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
				false, 0);
		break;
	case DCP_CHARGER_BIT:
		if (chg->wa_flags & QC_CHARGER_DETECTION_WA_BIT)
			schedule_delayed_work(&chg->hvdcp_detect_work,
					      msecs_to_jiffies(HVDCP_DET_MS));
		break;
	default:
		break;
	}

#ifdef CONFIG_LGE_PM
	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP ||
			chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3) {
		int typec_mode = 0;

		vote(chg->pseudo_usb_type_votable, APSD_RERUN_VOTER, false, PSEUDO_HVDCP);
		vote(chg->pseudo_usb_type_votable, LEGACY_CABLE_VOTER, false, PSEUDO_USB_TYPE);
		vote(chg->pseudo_usb_type_votable, ABNORMAL_GENDER_VOTER, false, PSEUDO_USB_TYPE);

		typec_mode = smblib_get_prop_typec_mode(chg);

		if (!chg->typec_legacy_valid &&
				(typec_mode == POWER_SUPPLY_TYPEC_SOURCE_MEDIUM
				|| typec_mode == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT)) {
			vote(chg->pseudo_usb_type_votable, FAST_HVDCP_DETECTION_VOTER, true, PSEUDO_HVDCP);
		}

		if (chg->retry_legacy_detection == RETRY_LEGACY_CABLE) {
			chg->retry_legacy_detection = WAIT_PD_HARD_RESET;
			schedule_work(&chg->legacy_detection_work);
		}
	}
	vote(chg->pseudo_usb_type_votable, RETRY_LEGACY_VOTER, false, PSEUDO_USB_TYPE);
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
	if (!chg->vzw_icl_change && !delayed_work_pending(&chg->vzw_recheck_work)) {
		schedule_delayed_work(&chg->vzw_recheck_work, msecs_to_jiffies(15000));
	}
#endif
#ifdef CONFIG_LGE_PM_DEBUG
	smblib_dbg(chg, PR_LGE, "IRQ: apsd-done rising; %s detected\n",
		   apsd_result->name);
#else
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: apsd-done rising; %s detected\n",
		   apsd_result->name);
#endif
}

irqreturn_t smblib_handle_usb_source_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc = 0;
	u8 stat;

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}
#ifdef CONFIG_LGE_PM_DEBUG
	smblib_dbg(chg, PR_LGE, "APSD_STATUS = 0x%02x\n", stat);
#else
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", stat);
#endif

	if (chg->micro_usb_mode && (stat & APSD_DTC_STATUS_DONE_BIT)
			&& !chg->uusb_apsd_rerun_done) {
		/*
		 * Force re-run APSD to handle slow insertion related
		 * charger-mis-detection.
		 */
		chg->uusb_apsd_rerun_done = true;
		smblib_rerun_apsd(chg);
		return IRQ_HANDLED;
	}

	smblib_handle_apsd_done(chg,
		(bool)(stat & APSD_DTC_STATUS_DONE_BIT));

	smblib_handle_hvdcp_detect_done(chg,
		(bool)(stat & QC_CHARGER_BIT));

	smblib_handle_hvdcp_check_timeout(chg,
		(bool)(stat & HVDCP_CHECK_TIMEOUT_BIT),
		(bool)(stat & QC_CHARGER_BIT));

	smblib_handle_hvdcp_3p0_auth_done(chg,
		(bool)(stat & QC_AUTH_DONE_STATUS_BIT));

	smblib_handle_sdp_enumeration_done(chg,
		(bool)(stat & ENUMERATION_DONE_BIT));

	smblib_handle_slow_plugin_timeout(chg,
		(bool)(stat & SLOW_PLUGIN_TIMEOUT_BIT));

	smblib_hvdcp_adaptive_voltage_change(chg);

	power_supply_changed(chg->usb_psy);

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", stat);

	return IRQ_HANDLED;
}

static void typec_sink_insertion(struct smb_charger *chg)
{
	/* when a sink is inserted we should not wait on hvdcp timeout to
	 * enable pd
	 */
	vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
			false, 0);
}

static void typec_sink_removal(struct smb_charger *chg)
{
	smblib_set_charge_param(chg, &chg->param.freq_boost,
			chg->chg_freq.freq_above_otg_threshold);
	chg->boost_current_ua = 0;
}

static void smblib_handle_typec_removal(struct smb_charger *chg)
{
	int rc;
#ifdef CONFIG_LGE_PM
	union power_supply_propval val = {0, };
#endif

	chg->cc2_detach_wa_active = false;

#ifndef CONFIG_LGE_USB
	/* reset APSD voters */
	vote(chg->apsd_disable_votable, PD_HARD_RESET_VOTER, false, 0);
	vote(chg->apsd_disable_votable, PD_VOTER, false, 0);
#endif

	cancel_delayed_work_sync(&chg->pl_enable_work);
	cancel_delayed_work_sync(&chg->hvdcp_detect_work);

	/* reset input current limit voters */
	vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, true, 100000);
#ifndef CONFIG_LGE_USB
	vote(chg->usb_icl_votable, PD_VOTER, false, 0);
#endif
	vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
	vote(chg->usb_icl_votable, PL_USBIN_USBIN_VOTER, false, 0);
	vote(chg->usb_icl_votable, SW_QC3_VOTER, false, 0);

	/* reset hvdcp voters */
	vote(chg->hvdcp_disable_votable_indirect, VBUS_CC_SHORT_VOTER, true, 0);
	vote(chg->hvdcp_disable_votable_indirect, PD_INACTIVE_VOTER, true, 0);
#ifdef CONFIG_LGE_PM_CC_PROTECT
	vote(chg->hvdcp_disable_votable_indirect, CC_PROTECT_VOTER, false, 0);
#endif

	/* reset power delivery voters */
#ifndef CONFIG_LGE_USB
	vote(chg->pd_allowed_votable, PD_VOTER, false, 0);
#endif
	vote(chg->pd_disallowed_votable_indirect, CC_DETACHED_VOTER, true, 0);
	vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER, true, 0);

	/* reset usb irq voters */
#ifndef CONFIG_LGE_USB
	vote(chg->usb_irq_enable_votable, PD_VOTER, false, 0);
#endif
	vote(chg->usb_irq_enable_votable, QC_VOTER, false, 0);

	/* reset parallel voters */
	vote(chg->pl_disable_votable, PL_DELAY_VOTER, true, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, false, 0);
	vote(chg->awake_votable, PL_DELAY_VOTER, false, 0);

	chg->vconn_attempts = 0;
	chg->otg_attempts = 0;
	chg->pulse_cnt = 0;
	chg->usb_icl_delta_ua = 0;
	chg->voltage_min_uv = MICRO_5V;
	chg->voltage_max_uv = MICRO_5V;
#ifndef CONFIG_LGE_USB
	chg->pd_active = 0;
	chg->pd_hard_reset = 0;
#endif
	chg->typec_legacy_valid = false;

	/* reset back to 120mS tCC debounce */
	rc = smblib_masked_write(chg, MISC_CFG_REG, TCC_DEBOUNCE_20MS_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't set 120mS tCC debounce rc=%d\n", rc);

#ifdef CONFIG_LGE_PM
	/* enable APSD CC trigger for next insertion */
	rc = smblib_masked_write(chg, TYPE_C_CFG_REG,
				APSD_START_ON_CC_BIT, chg->is_normal_bootmode ? APSD_START_ON_CC_BIT : 0);
#else
	/* enable APSD CC trigger for next insertion */
	rc = smblib_masked_write(chg, TYPE_C_CFG_REG,
				APSD_START_ON_CC_BIT, APSD_START_ON_CC_BIT);
#endif
	if (rc < 0)
		smblib_err(chg, "Couldn't enable APSD_START_ON_CC rc=%d\n", rc);

#ifdef CONFIG_LGE_PM
	rc = smblib_get_prop_usb_present_raw(chg, &val);
	if (rc >= 0 && val.intval && !chg->is_factory_cable) {
		vote(chg->pseudo_usb_type_votable, ABNORMAL_GENDER_VOTER, true, PSEUDO_USB_TYPE);
		smblib_dbg(chg, PR_LGE, "pseudo_usb_type_votable set - ABNORMAL_GENDER (%d)\n", chg->pseudo_usb_type);
	}
#endif

	if (chg->wa_flags & QC_AUTH_INTERRUPT_WA_BIT) {
		/* re-enable AUTH_IRQ_EN_CFG_BIT */
		rc = smblib_masked_write(chg,
				USBIN_SOURCE_CHANGE_INTRPT_ENB_REG,
				AUTH_IRQ_EN_CFG_BIT, AUTH_IRQ_EN_CFG_BIT);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't enable QC auth setting rc=%d\n", rc);
	}

#ifdef CONFIG_LGE_PM
	/* reconfigure allowed voltage for HVDCP */
	rc = smblib_set_adapter_allowance(chg,
			USBIN_ADAPTER_ALLOW_5V_TO_9V);
	if (rc < 0)
		smblib_err(chg, "Couldn't set USBIN_ADAPTER_ALLOW_5V_TO_9V rc=%d\n", rc);
#else
	/* reconfigure allowed voltage for HVDCP */
	rc = smblib_set_adapter_allowance(chg,
			USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V);
	if (rc < 0)
		smblib_err(chg, "Couldn't set USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V rc=%d\n",
			rc);
#endif

#ifdef CONFIG_LGE_USB
	if (chg->pd_active)
		goto skip_cc_ctrl;
#endif
	/* enable DRP */
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 TYPEC_POWER_ROLE_CMD_MASK, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't enable DRP rc=%d\n", rc);

	/* HW controlled CC_OUT */
	rc = smblib_masked_write(chg, TAPER_TIMER_SEL_CFG_REG,
							TYPEC_SPARE_CFG_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't enable HW cc_out rc=%d\n", rc);

	/* restore crude sensor */
	rc = smblib_write(chg, TM_IO_DTEST4_SEL, 0xA5);
	if (rc < 0)
		smblib_err(chg, "Couldn't restore crude sensor rc=%d\n", rc);

	mutex_lock(&chg->vconn_oc_lock);
	if (!chg->vconn_en)
		goto unlock;

	smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_VALUE_BIT, 0);
	chg->vconn_en = false;

unlock:
	mutex_unlock(&chg->vconn_oc_lock);

	/* clear exit sink based on cc */
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
						EXIT_SNK_BASED_ON_CC_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't clear exit_sink_based_on_cc rc=%d\n",
				rc);

#ifdef CONFIG_LGE_USB
skip_cc_ctrl:
#endif
	typec_sink_removal(chg);
	smblib_update_usb_type(chg);
}

static void smblib_handle_typec_insertion(struct smb_charger *chg)
{
	int rc;

	vote(chg->pd_disallowed_votable_indirect, CC_DETACHED_VOTER, false, 0);

	/* disable APSD CC trigger since CC is attached */
	rc = smblib_masked_write(chg, TYPE_C_CFG_REG, APSD_START_ON_CC_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't disable APSD_START_ON_CC rc=%d\n",
									rc);

	if (chg->typec_status[3] & UFP_DFP_MODE_STATUS_BIT)
		typec_sink_insertion(chg);
	else
		typec_sink_removal(chg);
	}

#ifdef CONFIG_LGE_PM
static void smblib_handle_rp_change(struct smb_charger *chg, int typec_mode)
{
	int rp_ua;
	const struct apsd_result *apsd = smblib_get_apsd_result(chg);

	if ((apsd->pst != POWER_SUPPLY_TYPE_USB_DCP)
		&& (apsd->pst != POWER_SUPPLY_TYPE_USB_FLOATED))
		return;

	if (chg->pd_active) {
		vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, false, 0);
		return;
	}

	/*
	 * handle Rp change for DCP/FLOAT/OCP.
	 * Update the current only if the Rp is different from
	 * the last Rp value.
	 */
	smblib_dbg(chg, PR_MISC, "CC change old_mode=%d new_mode=%d\n",
						chg->typec_mode, typec_mode);

	rp_ua = get_rp_based_dcp_current(chg, typec_mode);
	vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, true, rp_ua);
}
#endif

static void smblib_handle_typec_cc_state_change(struct smb_charger *chg)
{

#ifdef CONFIG_LGE_PM
	int typec_mode;

	cancel_delayed_work_sync(&chg->abnormal_gender_detect_work);

	if (chg->pr_swap_in_progress) {
		smblib_dbg(chg, PR_LGE, "IRQ: cc-state-change; Type-C %s skip\n",
					smblib_typec_mode_name[chg->typec_mode]);
		return;
	}

	typec_mode = smblib_get_prop_typec_mode(chg);
	if (chg->typec_present && (typec_mode != chg->typec_mode))
		smblib_handle_rp_change(chg, typec_mode);

	chg->typec_mode = typec_mode;
#else
	if (chg->pr_swap_in_progress)
		return;

	chg->typec_mode = smblib_get_prop_typec_mode(chg);
#endif

	if (!chg->typec_present && chg->typec_mode != POWER_SUPPLY_TYPEC_NONE) {
		chg->typec_present = true;
		smblib_dbg(chg, PR_MISC, "TypeC %s insertion\n",
			smblib_typec_mode_name[chg->typec_mode]);
		smblib_handle_typec_insertion(chg);
#ifdef CONFIG_LGE_PM
	} else if ((chg->typec_present
			&& chg->typec_mode == POWER_SUPPLY_TYPEC_NONE)
			|| chg->first_debounce_done) {
#else
	} else if (chg->typec_present &&
				chg->typec_mode == POWER_SUPPLY_TYPEC_NONE) {
#endif
		chg->typec_present = false;
		smblib_dbg(chg, PR_MISC, "TypeC removal\n");
		smblib_handle_typec_removal(chg);
	}

#ifdef CONFIG_LGE_PM
	chg->first_debounce_done = false;

	smblib_dbg(chg, PR_LGE, "IRQ: cc-state-change; Type-C %s detected\n",
				smblib_typec_mode_name[chg->typec_mode]);
#else
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: cc-state-change; Type-C %s detected\n",
				smblib_typec_mode_name[chg->typec_mode]);
#endif
}

static void smblib_usb_typec_change(struct smb_charger *chg)
{
	int rc;

	rc = smblib_multibyte_read(chg, TYPE_C_STATUS_1_REG,
							chg->typec_status, 5);
	if (rc < 0) {
		smblib_err(chg, "Couldn't cache USB Type-C status rc=%d\n", rc);
		return;
	}

	smblib_handle_typec_cc_state_change(chg);

	if (chg->typec_status[3] & TYPEC_VBUS_ERROR_STATUS_BIT)
#ifdef CONFIG_LGE_PM
		smblib_dbg(chg, PR_LGE, "IRQ: vbus-error\n");
#else
		smblib_dbg(chg, PR_INTERRUPT, "IRQ: vbus-error\n");
#endif

	if (chg->typec_status[3] & TYPEC_VCONN_OVERCURR_STATUS_BIT)
		schedule_work(&chg->vconn_oc_work);

	power_supply_changed(chg->usb_psy);
}

irqreturn_t smblib_handle_usb_typec_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	if (chg->micro_usb_mode) {
		cancel_delayed_work_sync(&chg->uusb_otg_work);
		vote(chg->awake_votable, OTG_DELAY_VOTER, true, 0);
		smblib_dbg(chg, PR_INTERRUPT, "Scheduling OTG work\n");
		schedule_delayed_work(&chg->uusb_otg_work,
				msecs_to_jiffies(chg->otg_delay_ms));
		return IRQ_HANDLED;
	}

	if (chg->cc2_detach_wa_active || chg->typec_en_dis_active) {
#ifdef CONFIG_LGE_PM
		smblib_dbg(chg, PR_LGE, "Ignoring since %s active\n",
			chg->cc2_detach_wa_active ?
			"cc2_detach_wa" : "typec_en_dis");
#else
		smblib_dbg(chg, PR_INTERRUPT, "Ignoring since %s active\n",
			chg->cc2_detach_wa_active ?
			"cc2_detach_wa" : "typec_en_dis");
#endif
		return IRQ_HANDLED;
	}

	mutex_lock(&chg->lock);
	smblib_usb_typec_change(chg);
	mutex_unlock(&chg->lock);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_dc_plugin(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
#ifdef CONFIG_IDTP9223_CHARGER
	union power_supply_propval pval = {0, };
	int rc = 0;

	rc = smblib_get_prop_dc_present(chg, &pval);
	smblib_dbg(chg, PR_LGE, "DC_PRESENT[%d]\n", pval.intval);
#endif
	power_supply_changed(chg->dc_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_high_duty_cycle(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	chg->is_hdc = true;
	schedule_delayed_work(&chg->clear_hdc_work, msecs_to_jiffies(60));

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_switcher_power_ok(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc, usb_icl;
	u8 stat;

	if (!(chg->wa_flags & BOOST_BACK_WA))
		return IRQ_HANDLED;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	/* skip suspending input if its already suspended by some other voter */
	usb_icl = get_effective_result(chg->usb_icl_votable);
	if ((stat & USE_USBIN_BIT) && usb_icl >= 0 && usb_icl < USBIN_25MA)
		return IRQ_HANDLED;

	if (stat & USE_DCIN_BIT)
		return IRQ_HANDLED;

#ifdef CONFIG_LGE_PM_WEAK_BATT_PACK
	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_DCP
			&& (stat & POWER_PATH_MASK) == POWER_PATH_BATTERY) {
		cancel_delayed_work_sync(&chg->batt_pack_check_work);
		schedule_delayed_work(&chg->batt_pack_check_work,
					msecs_to_jiffies(400));
	}
#endif
#ifdef CONFIG_LGE_PM_DEBUG
	smblib_dbg(chg, PR_LGE, "IRQ: %s, POWER_PATH_STS = 0x%x.\n", irq_data->name, stat);
#endif

	if (is_storming(&irq_data->storm_data)) {
		smblib_err(chg, "Reverse boost detected: voting 0mA to suspend input\n");
		vote(chg->usb_icl_votable, BOOST_BACK_VOTER, true, 0);
#ifdef CONFIG_LGE_PM
		cancel_delayed_work_sync(&chg->recovery_boost_back_work);
		schedule_delayed_work(&chg->recovery_boost_back_work,
					msecs_to_jiffies(500));
#endif
	}

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_wdog_bark(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;

	rc = smblib_write(chg, BARK_BITE_WDOG_PET_REG, BARK_BITE_WDOG_PET_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't pet the dog rc=%d\n", rc);

	return IRQ_HANDLED;
}

#ifdef CONFIG_LGE_PM_CC_PROTECT
irqreturn_t smblib_handle_cc_protect(int irq, void* data) {
	struct smb_charger *chg = data;

	smblib_dbg(chg, PR_LGE, "irq: cc protect\n");
	vote(chg->hvdcp_disable_votable_indirect, CC_PROTECT_VOTER, true, 0);

	return IRQ_HANDLED;
}
#endif

/**************
 * Additional USB PSY getters/setters
 * that call interrupt functions
***************/

int smblib_get_prop_pr_swap_in_progress(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->pr_swap_in_progress;
	return 0;
}

int smblib_set_prop_pr_swap_in_progress(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;

	chg->pr_swap_in_progress = val->intval;
	/*
	 * call the cc changed irq to handle real removals while
	 * PR_SWAP was in progress
	 */
	smblib_usb_typec_change(chg);
	rc = smblib_masked_write(chg, MISC_CFG_REG, TCC_DEBOUNCE_20MS_BIT,
			val->intval ? TCC_DEBOUNCE_20MS_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't set tCC debounce rc=%d\n", rc);
	return 0;
}

/***************
 * Work Queues *
 ***************/
static void smblib_uusb_otg_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						uusb_otg_work.work);
	int rc;
	u8 stat;
	bool otg;

	rc = smblib_read(chg, TYPE_C_STATUS_3_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_3 rc=%d\n", rc);
		goto out;
	}

	otg = !!(stat & (U_USB_GND_NOVBUS_BIT | U_USB_GND_BIT));
	extcon_set_cable_state_(chg->extcon, EXTCON_USB_HOST, otg);
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_3 = 0x%02x OTG=%d\n",
			stat, otg);
	power_supply_changed(chg->usb_psy);

out:
	vote(chg->awake_votable, OTG_DELAY_VOTER, false, 0);
}


static void smblib_hvdcp_detect_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
					       hvdcp_detect_work.work);

	vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
				false, 0);
	power_supply_changed(chg->usb_psy);
}

#ifdef CONFIG_LGE_PM_CYCLE_BASED_CHG_VOLTAGE
static int batt_life_cycle_set_fcc_ma(struct smb_charger *chg) {
	union power_supply_propval val = {0, };
	int i, rc, battery_cycle = 0;

	rc = power_supply_get_property(chg->bms_psy,
			POWER_SUPPLY_PROP_BATTERY_CYCLE, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get debug battery prop rc=%d\n", rc);
		return rc;
	}
	battery_cycle = val.intval;

	for (i = 1; i < chg->batt_life_cycle_size; i++) {
		if(battery_cycle < chg->batt_life_cycle_set[i]) {
			break;
		}
	}

	smblib_dbg(chg, PR_MISC, "cycle : %d, step : %d, fcc : %d\n",
		battery_cycle, i-1, chg->batt_life_cycle_fcc_ma[i-1]);

	rc = vote(chg->fcc_votable, LGE_CBC_VOTER, true,
		chg->batt_life_cycle_fcc_ma[i-1]);
	if (rc < 0)
		pr_err("Couldn't vote en rc %d\n", rc);

	return rc;

}
#endif

static void bms_update_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						bms_update_work);

	smblib_suspend_on_debug_battery(chg);
#ifdef CONFIG_LGE_PM_CYCLE_BASED_CHG_VOLTAGE
	batt_life_cycle_set_fcc_ma(chg);
#endif

	if (chg->batt_psy)
		power_supply_changed(chg->batt_psy);
}

#ifdef CONFIG_LGE_PM_STEP_CHARGING
static void batt_update_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						batt_update_work);

	if (!delayed_work_pending(&chg->step_charging_check_work)) {
		schedule_delayed_work(&chg->step_charging_check_work,
					  msecs_to_jiffies(0));
	} else {
		smblib_dbg(chg, PR_LGE, "skip check step charging.\n");
	}
}
#endif

#ifdef CONFIG_IDTP9223_CHARGER
static void idtp9223_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			idtp9223_work);
	union power_supply_propval pval = {0, };
	int rc = 0;

	rc = power_supply_get_property(chg->wlc_psy,
			POWER_SUPPLY_PROP_FASTCHG, &pval);
	if (rc >= 0) {
		smblib_dbg(chg, PR_LGE, "Wireless Fast-charger state=%d\n", pval.intval);
		if (chg->dc_fastchg != pval.intval) {
			chg->dc_fastchg = pval.intval;
			power_supply_changed(chg->usb_psy);
		}
	}
}
#endif

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
static void vzw_recheck_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						vzw_recheck_work.work);

	chg->vzw_icl_change = true;
	schedule_delayed_work(&chg->vzw_aicl_done_work, 0);
}

static void vzw_aicl_done_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						vzw_aicl_done_work.work);

	if (chg->usb_psy && chg->vzw_icl_change)
		power_supply_changed(chg->usb_psy);
}
#endif

static void step_soc_req_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						step_soc_req_work.work);
	union power_supply_propval pval = {0, };
	int rc;

	rc = smblib_get_prop_batt_capacity(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get batt capacity rc=%d\n", rc);
		return;
	}

	step_charge_soc_update(chg, pval.intval);
}

static void clear_hdc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						clear_hdc_work.work);

	chg->is_hdc = 0;
}

#ifdef CONFIG_LGE_PM
static void recovery_boost_back_work(struct work_struct *work){
	struct smb_charger *chg = container_of(work, struct smb_charger,
						recovery_boost_back_work.work);
	int rc;
	u8 stat;
	struct storm_watch *wdata;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n", rc);
		schedule_delayed_work(&chg->recovery_boost_back_work,
					msecs_to_jiffies(100));
		return;
	}

	if (chg->wa_flags & BOOST_BACK_WA && stat & USE_USBIN_BIT) {
		wdata = &chg->irq_info[SWITCH_POWER_OK_IRQ].irq_data->storm_data;
		reset_storm_count(wdata);
		vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);
		smblib_dbg(chg, PR_LGE, "reset boost back vote stat = 0x%x.\n", stat);
	}
}

static void abnormal_gender_detect_work(struct work_struct *work){
	struct smb_charger *chg = container_of(work, struct smb_charger,
						abnormal_gender_detect_work.work);
	int rc;

	smblib_dbg(chg, PR_LGE, "rerun apsd for abnormal gender.\n");

	rc = smblib_masked_write(chg, TYPE_C_CFG_REG, APSD_START_ON_CC_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't disable APSD_START_ON_CC rc=%d\n", rc);
	chg->is_abnormal_gendor = true;
	smblib_rerun_apsd(chg);
	smblib_rerun_aicl(chg);
}
#endif

#ifdef CONFIG_LGE_PM_DEBUG
static void charging_information(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			                        charging_inform_work.work);

	union power_supply_propval pval = {0, };
	int batt_temp, batt_soc, batt_vol, usbin_vol, fcc_ua, ibat_now;
	int usb_present, iusb_current_set, iusb_current_now;
	int chg_state_temp, dcp_icl, sdp_icl, pd_icl;
	char *chg_state, *chg_type, *usb_type, *fake_mode;
	int slave_current_now, slave_current_set, slave_voltage_set, slave_enabled, chg_status;
	int rc;
	u8 stat;
	int pmi_charger_temp, smb_charger_temp, pmi_skin_temp;

	rc = smblib_get_prop_usb_present(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "error get USB_PRESENT rc=%d\n", rc);
	usb_present = pval.intval;

	rc = smblib_get_prop_charger_temp(chg, &pval);
	if (rc < 0)
		pmi_charger_temp = -1;
	else
		pmi_charger_temp = pval.intval;

#ifdef CONFIG_LGE_PM_INOV_GEN3_SYSFS_SUPPORT
	rc = smblib_get_prop_skin_temp(chg, &pval);
	if (rc < 0)
		pmi_skin_temp = -1;
	else
		pmi_skin_temp = pval.intval;
#else
	pmi_skin_temp = -1;
#endif

	rc = smblib_get_prop_batt_temp(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "error get BATT_TEMP rc=%d\n", rc);
	batt_temp = (pval.intval) / 10;

	rc = smblib_get_prop_batt_capacity(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "error get BATT_SOC rc=%d\n", rc);
	batt_soc = pval.intval;

	rc = smblib_get_prop_batt_voltage_now(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "error get BATT_VOL rc=%d\n", rc);
	batt_vol = (pval.intval) / 1000;

	dcp_icl = get_client_vote_locked(chg->usb_icl_votable, DCP_VOTER) / 1000;
	pd_icl = get_client_vote_locked(chg->usb_icl_votable, PD_VOTER) / 1000;

	rc = smblib_get_prop_usb_current_max(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "error get IUSB current max rc=%d\n", rc);
	sdp_icl = (pval.intval) / 1000;

	rc = smblib_get_prop_input_current_settled(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "error get IUSB current setted rc=%d\n", rc);
	iusb_current_set = (pval.intval) / 1000;

	rc = smblib_get_prop_usb_current_now(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "error get IUSB current now rc=%d\n", rc);
	iusb_current_now = (pval.intval) / 1000;

	rc = smblib_get_prop_usb_voltage_now(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "error get USBIN voltage rc=%d\n", rc);
	usbin_vol = (pval.intval) / 1000;

	rc = smblib_get_charge_param(chg, &chg->param.fcc, &fcc_ua);
	if (rc < 0)
		smblib_err(chg, "error get FCC current rc=%d\n", rc);
	fcc_ua = fcc_ua / 1000;

	rc = smblib_get_prop_batt_current_now(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "error get IBAT rc=%d\n", rc);
	ibat_now = (pval.intval) / 1000;

	rc = smblib_get_prop_batt_status(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "error get CHG state rc=%d\n", rc);
	chg_state_temp = pval.intval;
	switch(chg_state_temp) {
		case POWER_SUPPLY_STATUS_CHARGING:
			chg_state = "CHARGING";
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
			chg_state = "DISCHARGING";
			break;
		case POWER_SUPPLY_STATUS_FULL:
			chg_state = "FULLCHARGING";
			break;
		case POWER_SUPPLY_STATUS_NOT_CHARGING:
			chg_state = "NOTCHARGING";
			break;
		default:
			chg_state = "UNKNOWN";
			break;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "error get BATTERY_CHARGER_STATUS_1 rc=%d\n",
				rc);
	}
	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	switch (stat) {
		case TRICKLE_CHARGE:
			chg_type = "TRICKLE";
			break;
		case PRE_CHARGE:
			chg_type = "PRE";
			break;
		case FAST_CHARGE:
			chg_type = "FAST";
			break;
		case FULLON_CHARGE:
			chg_type = "FULL_ON";
			break;
		case TAPER_CHARGE:
			chg_type = "TAPER";
			break;
		case TERMINATE_CHARGE:
			chg_type = "TERMINATE";
			break;
		case INHIBIT_CHARGE:
			chg_type = "INHIBIT";
			break;
		case DISABLE_CHARGE:
			chg_type = "DISABLE";
			break;
		default:
			chg_type = "NONE";
			break;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "error get BATTERY_CHARGER_STATUS_1 rc=%d\n",
				rc);
	}
	chg_status = stat <<8;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "error get BATTERY_CHARGER_STATUS_1 rc=%d\n",
				rc);
	}
	chg_status |= stat;

	rc = smblib_read(chg, TEMP_RANGE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "error get BATTERY_CHARGER_STATUS_1 rc=%d\n",
				rc);
	}
	chg_status |= stat <<16;

	if (chg->pl.psy) {
		rc = power_supply_get_property(chg->pl.psy,POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
		if (rc < 0) {
			slave_current_set = -1;
		} else {
			slave_current_set = (pval.intval) / 1000;
		}

		rc = power_supply_get_property(chg->pl.psy,POWER_SUPPLY_PROP_VOLTAGE_MAX, &pval);
		if (rc < 0) {
			slave_voltage_set = -1;
		} else {
			slave_voltage_set = (pval.intval) / 1000;
		}

		rc = power_supply_get_property(chg->pl.psy,POWER_SUPPLY_PROP_CURRENT_NOW, &pval);
		if (rc < 0) {
			slave_current_now = -1;
		} else {
			slave_current_now = (pval.intval) / 1000;
		}

		rc = power_supply_get_property(chg->pl.psy,POWER_SUPPLY_PROP_CHARGING_ENABLED, &pval);
		if (rc < 0) {
			slave_enabled = -1;
		} else {
			slave_enabled = pval.intval;
		}

		if (usb_present) {
			rc = power_supply_get_property(chg->pl.psy,POWER_SUPPLY_PROP_CHARGER_TEMP, &pval);
			if (rc < 0) {
				smb_charger_temp = -1;
			} else {
				smb_charger_temp = pval.intval;
			}
		} else {
			smb_charger_temp = -1;
		}
	}

	switch (chg->real_charger_type) {
		case POWER_SUPPLY_TYPE_USB:
			usb_type = "SDP";
			break;
		case POWER_SUPPLY_TYPE_USB_DCP:
			usb_type = "DCP";
			break;
		case POWER_SUPPLY_TYPE_USB_CDP:
			usb_type = "CDP";
			break;
		case POWER_SUPPLY_TYPE_USB_HVDCP:
			usb_type = "HVDCP";
			break;
		case POWER_SUPPLY_TYPE_USB_HVDCP_3:
			usb_type = "HVDCP_3";
			break;
		case POWER_SUPPLY_TYPE_USB_PD:
			usb_type = "USB_PD";
			break;
		case POWER_SUPPLY_TYPE_WIRELESS:
			usb_type = "Wireless";
			break;
		case POWER_SUPPLY_TYPE_USB_FLOATED:
			usb_type = "USB_Floated";
			break;
		default:
			rc = smblib_get_prop_dc_online(chg, &pval);
			if (rc >= 0 && pval.intval)
				usb_type = "WIRELESS";
			else
				usb_type = "UNKNOWN";
			break;
	}
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PSEUDO_BATTERY
	fake_mode = chg->pseudo_batt? "ON" : "OFF";
#else
	fake_mode = "NA";
#endif

	smblib_dbg(chg, PR_LGE, "[USB] PRESENT:%d, TYPE:%d, VOL:%d, [CHG] STATE:%d, "
			"[CUR] USB_SET:%d, USB_NOW:%d, BAT_SET:%d,\n",
			usb_present, chg->real_charger_type, usbin_vol, chg_state_temp, iusb_current_set,
			iusb_current_now, slave_enabled?fcc_ua+slave_current_set:fcc_ua);
	smblib_dbg(chg, PR_LGE, "[USB] TYPE:%s, [CHG] STATE:%s, TYPE:%s, FAKE:%s, STATUS:0x%x, "
			"[SLV] EN:%d, ISET:%d, VSET:%d, INOW:%d, [TEMP] PMI : %d, SMB : %d, SKIN : %d,\n",
			usb_type, chg_state, chg_type, fake_mode, chg_status,
			slave_enabled, slave_current_set, slave_voltage_set,slave_current_now,
			pmi_charger_temp, smb_charger_temp, pmi_skin_temp);
	smblib_dbg(chg, PR_LGE, "[VOTE] USB_ICL:%s %d, DC_ICL:%s %d, FCC:%s %d, FV:%s %d, PL_DISABLE:%s %d,\n",
			get_effective_client_locked(chg->usb_icl_votable), get_effective_result(chg->usb_icl_votable)/1000,
			get_effective_client_locked(chg->dc_icl_votable), get_effective_result(chg->dc_icl_votable)/1000,
			get_effective_client_locked(chg->fcc_votable), get_effective_result(chg->fcc_votable)/1000,
			get_effective_client_locked(chg->fv_votable), get_effective_result(chg->fv_votable)/1000,
			get_effective_client_locked(chg->pl_disable_votable), get_effective_result(chg->pl_disable_votable));

	schedule_delayed_work(&chg->charging_inform_work,
			round_jiffies_relative(msecs_to_jiffies(CHARGING_INFORM_NORMAL_TIME)));
}
#endif

static void rdstd_cc2_detach_work(struct work_struct *work)
{
	int rc;
	u8 stat4, stat5;
	struct smb_charger *chg = container_of(work, struct smb_charger,
						rdstd_cc2_detach_work);

	if (!chg->cc2_detach_wa_active)
		return;

	/*
	 * WA steps -
	 * 1. Enable both UFP and DFP, wait for 10ms.
	 * 2. Disable DFP, wait for 30ms.
	 * 3. Removal detected if both TYPEC_DEBOUNCE_DONE_STATUS
	 *    and TIMER_STAGE bits are gone, otherwise repeat all by
	 *    work rescheduling.
	 * Note, work will be cancelled when USB_PLUGIN rises.
	 */

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 UFP_EN_CMD_BIT | DFP_EN_CMD_BIT,
				 UFP_EN_CMD_BIT | DFP_EN_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write TYPE_C_CTRL_REG rc=%d\n", rc);
		return;
	}

	usleep_range(10000, 11000);

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 UFP_EN_CMD_BIT | DFP_EN_CMD_BIT,
				 UFP_EN_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write TYPE_C_CTRL_REG rc=%d\n", rc);
		return;
	}

	usleep_range(30000, 31000);

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat4);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return;
	}

	rc = smblib_read(chg, TYPE_C_STATUS_5_REG, &stat5);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't read TYPE_C_STATUS_5_REG rc=%d\n", rc);
		return;
	}

	if ((stat4 & TYPEC_DEBOUNCE_DONE_STATUS_BIT)
			|| (stat5 & TIMER_STAGE_2_BIT)) {
		smblib_dbg(chg, PR_MISC, "rerunning DD=%d TS2BIT=%d\n",
				(int)(stat4 & TYPEC_DEBOUNCE_DONE_STATUS_BIT),
				(int)(stat5 & TIMER_STAGE_2_BIT));
		goto rerun;
	}

	smblib_dbg(chg, PR_MISC, "Bingo CC2 Removal detected\n");
	chg->cc2_detach_wa_active = false;
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
						EXIT_SNK_BASED_ON_CC_BIT, 0);
	smblib_reg_block_restore(chg, cc2_detach_settings);
	mutex_lock(&chg->lock);
	smblib_usb_typec_change(chg);
	mutex_unlock(&chg->lock);
	return;

rerun:
	schedule_work(&chg->rdstd_cc2_detach_work);
}

static void smblib_otg_oc_exit(struct smb_charger *chg, bool success)
{
	int rc;

	chg->otg_attempts = 0;
	if (!success) {
		smblib_err(chg, "OTG soft start failed\n");
		chg->otg_en = false;
	}

	smblib_dbg(chg, PR_OTG, "enabling VBUS < 1V check\n");
	rc = smblib_masked_write(chg, OTG_CFG_REG,
					QUICKSTART_OTG_FASTROLESWAP_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't enable VBUS < 1V check rc=%d\n", rc);
}

#define MAX_OC_FALLING_TRIES 10
static void smblib_otg_oc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
								otg_oc_work);
	int rc, i;
	u8 stat;

	if (!chg->vbus_vreg || !chg->vbus_vreg->rdev)
		return;

	smblib_err(chg, "over-current detected on VBUS\n");
	mutex_lock(&chg->otg_oc_lock);
	if (!chg->otg_en)
		goto unlock;

	smblib_dbg(chg, PR_OTG, "disabling VBUS < 1V check\n");
	smblib_masked_write(chg, OTG_CFG_REG,
					QUICKSTART_OTG_FASTROLESWAP_BIT,
					QUICKSTART_OTG_FASTROLESWAP_BIT);

	/*
	 * If 500ms has passed and another over-current interrupt has not
	 * triggered then it is likely that the software based soft start was
	 * successful and the VBUS < 1V restriction should be re-enabled.
	 */
	schedule_delayed_work(&chg->otg_ss_done_work, msecs_to_jiffies(500));

	rc = _smblib_vbus_regulator_disable(chg->vbus_vreg->rdev);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable VBUS rc=%d\n", rc);
		goto unlock;
	}

	if (++chg->otg_attempts > OTG_MAX_ATTEMPTS) {
		cancel_delayed_work_sync(&chg->otg_ss_done_work);
		smblib_err(chg, "OTG failed to enable after %d attempts\n",
			   chg->otg_attempts - 1);
		smblib_otg_oc_exit(chg, false);
		goto unlock;
	}

	/*
	 * The real time status should go low within 10ms. Poll every 1-2ms to
	 * minimize the delay when re-enabling OTG.
	 */
	for (i = 0; i < MAX_OC_FALLING_TRIES; ++i) {
		usleep_range(1000, 2000);
		rc = smblib_read(chg, OTG_BASE + INT_RT_STS_OFFSET, &stat);
		if (rc >= 0 && !(stat & OTG_OVERCURRENT_RT_STS_BIT))
			break;
	}

	if (i >= MAX_OC_FALLING_TRIES) {
		cancel_delayed_work_sync(&chg->otg_ss_done_work);
		smblib_err(chg, "OTG OC did not fall after %dms\n",
						2 * MAX_OC_FALLING_TRIES);
		smblib_otg_oc_exit(chg, false);
		goto unlock;
	}

	smblib_dbg(chg, PR_OTG, "OTG OC fell after %dms\n", 2 * i + 1);
	rc = _smblib_vbus_regulator_enable(chg->vbus_vreg->rdev);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable VBUS rc=%d\n", rc);
		goto unlock;
	}

unlock:
	mutex_unlock(&chg->otg_oc_lock);
}

static void smblib_vconn_oc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
								vconn_oc_work);
	int rc, i;
	u8 stat;

	if (chg->micro_usb_mode)
		return;

	smblib_err(chg, "over-current detected on VCONN\n");
	if (!chg->vconn_vreg || !chg->vconn_vreg->rdev)
		return;

	mutex_lock(&chg->vconn_oc_lock);
	rc = _smblib_vconn_regulator_disable(chg->vconn_vreg->rdev);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable VCONN rc=%d\n", rc);
		goto unlock;
	}

	if (++chg->vconn_attempts > VCONN_MAX_ATTEMPTS) {
		smblib_err(chg, "VCONN failed to enable after %d attempts\n",
			   chg->otg_attempts - 1);
		chg->vconn_en = false;
		chg->vconn_attempts = 0;
		goto unlock;
	}

	/*
	 * The real time status should go low within 10ms. Poll every 1-2ms to
	 * minimize the delay when re-enabling OTG.
	 */
	for (i = 0; i < MAX_OC_FALLING_TRIES; ++i) {
		usleep_range(1000, 2000);
		rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
		if (rc >= 0 && !(stat & TYPEC_VCONN_OVERCURR_STATUS_BIT))
			break;
	}

	if (i >= MAX_OC_FALLING_TRIES) {
		smblib_err(chg, "VCONN OC did not fall after %dms\n",
						2 * MAX_OC_FALLING_TRIES);
		chg->vconn_en = false;
		chg->vconn_attempts = 0;
		goto unlock;
	}

	smblib_dbg(chg, PR_OTG, "VCONN OC fell after %dms\n", 2 * i + 1);
	if (++chg->vconn_attempts > VCONN_MAX_ATTEMPTS) {
		smblib_err(chg, "VCONN failed to enable after %d attempts\n",
			   chg->vconn_attempts - 1);
		chg->vconn_en = false;
		goto unlock;
	}

	rc = _smblib_vconn_regulator_enable(chg->vconn_vreg->rdev);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable VCONN rc=%d\n", rc);
		goto unlock;
	}

unlock:
	mutex_unlock(&chg->vconn_oc_lock);
}

static void smblib_otg_ss_done_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							otg_ss_done_work.work);
	int rc;
	bool success = false;
	u8 stat;

	mutex_lock(&chg->otg_oc_lock);
	rc = smblib_read(chg, OTG_STATUS_REG, &stat);
	if (rc < 0)
		smblib_err(chg, "Couldn't read OTG status rc=%d\n", rc);
	else if (stat & BOOST_SOFTSTART_DONE_BIT)
		success = true;

	smblib_otg_oc_exit(chg, success);
	mutex_unlock(&chg->otg_oc_lock);
}

static void smblib_icl_change_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							icl_change_work.work);
	int rc, settled_ua;

	rc = smblib_get_charge_param(chg, &chg->param.icl_stat, &settled_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get ICL status rc=%d\n", rc);
		return;
	}

#ifdef CONFIG_LGE_PM
	smblib_dbg(chg, PR_LGE, "icl_settled=%d\n", settled_ua);

	if (chg->pre_settled_ua != settled_ua) {
		chg->pre_settled_ua = settled_ua;
	power_supply_changed(chg->usb_main_psy);
	}
#else
	power_supply_changed(chg->usb_main_psy);
#endif

	smblib_dbg(chg, PR_INTERRUPT, "icl_settled=%d\n", settled_ua);
}

static void smblib_pl_enable_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							pl_enable_work.work);

	smblib_dbg(chg, PR_PARALLEL, "timer expired, enabling parallel\n");
	vote(chg->pl_disable_votable, PL_DELAY_VOTER, false, 0);
	vote(chg->awake_votable, PL_DELAY_VOTER, false, 0);
}

static void smblib_legacy_detection_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							legacy_detection_work);
	int rc;
	u8 stat;
	bool legacy, rp_high;
#ifdef CONFIG_LGE_PM
	bool enable_hvdcp = false;
	union power_supply_propval val = {0, };
#endif

	mutex_lock(&chg->lock);
	chg->typec_en_dis_active = 1;
#ifdef CONFIG_LGE_PM
	vote(chg->pseudo_usb_type_votable, LEGACY_CABLE_VOTER, true, PSEUDO_USB_TYPE);
#endif
	smblib_dbg(chg, PR_MISC, "running legacy unknown workaround\n");
	rc = smblib_masked_write(chg,
				TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				TYPEC_DISABLE_CMD_BIT,
				TYPEC_DISABLE_CMD_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't disable type-c rc=%d\n", rc);

	/* wait for the adapter to turn off VBUS */
	msleep(500);

	rc = smblib_masked_write(chg,
				TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				TYPEC_DISABLE_CMD_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't enable type-c rc=%d\n", rc);

	/* wait for type-c detection to complete */
	msleep(100);

	rc = smblib_read(chg, TYPE_C_STATUS_5_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read typec stat5 rc = %d\n", rc);
		goto unlock;
	}

	chg->typec_legacy_valid = true;
#ifndef CONFIG_LGE_PM
	vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, false, 0);
#endif
	legacy = stat & TYPEC_LEGACY_CABLE_STATUS_BIT;
	rp_high = chg->typec_mode == POWER_SUPPLY_TYPEC_SOURCE_HIGH;
#ifdef CONFIG_LGE_PM
	if ((!legacy && smblib_get_prop_ufp_mode(chg) == POWER_SUPPLY_TYPEC_SOURCE_HIGH)
			|| smblib_get_prop_ufp_mode(chg) == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT
			|| smblib_get_prop_ufp_mode(chg) == POWER_SUPPLY_TYPEC_SOURCE_MEDIUM) {
		enable_hvdcp = true;
		vote(chg->hvdcp_disable_votable_indirect, VBUS_CC_SHORT_VOTER,
								false, 0);
	} else {
		const struct apsd_result *apsd_result;

		apsd_result = smblib_update_usb_type(chg);
		smblib_force_legacy_icl(chg, apsd_result->pst);
	}
	smblib_dbg(chg, PR_LGE, "Check Vbus & CC short. hvdcp[%d], legacy[%d] rp_high[0x%x].\n", enable_hvdcp, legacy, chg->typec_status[0]);
#else
	if (!legacy || !rp_high)
		vote(chg->hvdcp_disable_votable_indirect, VBUS_CC_SHORT_VOTER,
								false, 0);
#endif
unlock:
	chg->typec_en_dis_active = 0;
#ifndef CONFIG_LGE_PM
	smblib_usb_typec_change(chg);
#endif
	mutex_unlock(&chg->lock);
#ifdef CONFIG_LGE_PM
	msleep(200);

	rc = smblib_get_prop_usb_present_raw(chg, &val);
	if (enable_hvdcp && val.intval && rc >= 0) {
		vote(chg->pseudo_usb_type_votable, LEGACY_CABLE_VOTER, true, PSEUDO_HVDCP);
		smblib_dbg(chg, PR_LGE, "pseudo_usb_type_votable set - LEGACY_CABLE (%d)\n", chg->pseudo_usb_type);
	} else {
		vote(chg->pseudo_usb_type_votable, LEGACY_CABLE_VOTER, false, PSEUDO_USB_TYPE);
		smblib_dbg(chg, PR_LGE, "pseudo_usb_type_votable unset - LEGACY_CABLE\n");
		if (chg->typec_mode != POWER_SUPPLY_TYPEC_NONE)
			smblib_usb_typec_change(chg);
		smblib_update_usb_type(chg);
		power_supply_changed(chg->usb_psy);
	}
	vote(chg->pseudo_usb_type_votable, FAST_HVDCP_DETECTION_VOTER, false, PSEUDO_HVDCP);
#endif
}

static int smblib_create_votables(struct smb_charger *chg)
{
	int rc = 0;

	chg->fcc_votable = find_votable("FCC");
	if (chg->fcc_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find FCC votable rc=%d\n", rc);
		return rc;
	}

	chg->fv_votable = find_votable("FV");
	if (chg->fv_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find FV votable rc=%d\n", rc);
		return rc;
	}

	chg->usb_icl_votable = find_votable("USB_ICL");
	if (!chg->usb_icl_votable) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find USB_ICL votable rc=%d\n", rc);
		return rc;
	}

	chg->pl_disable_votable = find_votable("PL_DISABLE");
	if (chg->pl_disable_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find votable PL_DISABLE rc=%d\n", rc);
		return rc;
	}

	chg->pl_enable_votable_indirect = find_votable("PL_ENABLE_INDIRECT");
	if (chg->pl_enable_votable_indirect == NULL) {
		rc = -EINVAL;
		smblib_err(chg,
			"Couldn't find votable PL_ENABLE_INDIRECT rc=%d\n",
			rc);
		return rc;
	}

	vote(chg->pl_disable_votable, PL_DELAY_VOTER, true, 0);

	chg->dc_suspend_votable = create_votable("DC_SUSPEND", VOTE_SET_ANY,
					smblib_dc_suspend_vote_callback,
					chg);
	if (IS_ERR(chg->dc_suspend_votable)) {
		rc = PTR_ERR(chg->dc_suspend_votable);
		return rc;
	}

	chg->dc_icl_votable = create_votable("DC_ICL", VOTE_MIN,
					smblib_dc_icl_vote_callback,
					chg);
	if (IS_ERR(chg->dc_icl_votable)) {
		rc = PTR_ERR(chg->dc_icl_votable);
		return rc;
	}

	chg->pd_disallowed_votable_indirect
		= create_votable("PD_DISALLOWED_INDIRECT", VOTE_SET_ANY,
			smblib_pd_disallowed_votable_indirect_callback, chg);
	if (IS_ERR(chg->pd_disallowed_votable_indirect)) {
		rc = PTR_ERR(chg->pd_disallowed_votable_indirect);
		return rc;
	}

	chg->pd_allowed_votable = create_votable("PD_ALLOWED",
					VOTE_SET_ANY, NULL, NULL);
	if (IS_ERR(chg->pd_allowed_votable)) {
		rc = PTR_ERR(chg->pd_allowed_votable);
		return rc;
	}

	chg->awake_votable = create_votable("AWAKE", VOTE_SET_ANY,
					smblib_awake_vote_callback,
					chg);
	if (IS_ERR(chg->awake_votable)) {
		rc = PTR_ERR(chg->awake_votable);
		return rc;
	}

	chg->chg_disable_votable = create_votable("CHG_DISABLE", VOTE_SET_ANY,
					smblib_chg_disable_vote_callback,
					chg);
	if (IS_ERR(chg->chg_disable_votable)) {
		rc = PTR_ERR(chg->chg_disable_votable);
		return rc;
	}


	chg->hvdcp_disable_votable_indirect = create_votable(
				"HVDCP_DISABLE_INDIRECT",
				VOTE_SET_ANY,
				smblib_hvdcp_disable_indirect_vote_callback,
				chg);
	if (IS_ERR(chg->hvdcp_disable_votable_indirect)) {
		rc = PTR_ERR(chg->hvdcp_disable_votable_indirect);
		return rc;
	}

	chg->hvdcp_enable_votable = create_votable("HVDCP_ENABLE",
					VOTE_SET_ANY,
					smblib_hvdcp_enable_vote_callback,
					chg);
	if (IS_ERR(chg->hvdcp_enable_votable)) {
		rc = PTR_ERR(chg->hvdcp_enable_votable);
		return rc;
	}

	chg->apsd_disable_votable = create_votable("APSD_DISABLE",
					VOTE_SET_ANY,
					smblib_apsd_disable_vote_callback,
					chg);
	if (IS_ERR(chg->apsd_disable_votable)) {
		rc = PTR_ERR(chg->apsd_disable_votable);
		return rc;
	}

	chg->hvdcp_hw_inov_dis_votable = create_votable("HVDCP_HW_INOV_DIS",
					VOTE_SET_ANY,
					smblib_hvdcp_hw_inov_dis_vote_callback,
					chg);
	if (IS_ERR(chg->hvdcp_hw_inov_dis_votable)) {
		rc = PTR_ERR(chg->hvdcp_hw_inov_dis_votable);
		return rc;
	}

	chg->usb_irq_enable_votable = create_votable("USB_IRQ_DISABLE",
					VOTE_SET_ANY,
					smblib_usb_irq_enable_vote_callback,
					chg);
	if (IS_ERR(chg->usb_irq_enable_votable)) {
		rc = PTR_ERR(chg->usb_irq_enable_votable);
		return rc;
	}

	chg->typec_irq_disable_votable = create_votable("TYPEC_IRQ_DISABLE",
					VOTE_SET_ANY,
					smblib_typec_irq_disable_vote_callback,
					chg);
	if (IS_ERR(chg->typec_irq_disable_votable)) {
		rc = PTR_ERR(chg->typec_irq_disable_votable);
		return rc;
	}

#ifdef CONFIG_LGE_PM
	chg->pseudo_usb_type_votable = create_votable("PSEUDO_USB_TYPE",
					VOTE_MAX, NULL, NULL);
	if (IS_ERR(chg->pseudo_usb_type_votable)) {
		rc = PTR_ERR(chg->pseudo_usb_type_votable);
		return rc;
	}
#endif
	return rc;
}

static void smblib_destroy_votables(struct smb_charger *chg)
{
	if (chg->dc_suspend_votable)
		destroy_votable(chg->dc_suspend_votable);
	if (chg->usb_icl_votable)
		destroy_votable(chg->usb_icl_votable);
	if (chg->dc_icl_votable)
		destroy_votable(chg->dc_icl_votable);
	if (chg->pd_disallowed_votable_indirect)
		destroy_votable(chg->pd_disallowed_votable_indirect);
	if (chg->pd_allowed_votable)
		destroy_votable(chg->pd_allowed_votable);
	if (chg->awake_votable)
		destroy_votable(chg->awake_votable);
	if (chg->chg_disable_votable)
		destroy_votable(chg->chg_disable_votable);
	if (chg->apsd_disable_votable)
		destroy_votable(chg->apsd_disable_votable);
	if (chg->hvdcp_hw_inov_dis_votable)
		destroy_votable(chg->hvdcp_hw_inov_dis_votable);
	if (chg->typec_irq_disable_votable)
		destroy_votable(chg->typec_irq_disable_votable);
#ifdef CONFIG_LGE_PM
	if (chg->pseudo_usb_type_votable)
		destroy_votable(chg->pseudo_usb_type_votable);
#endif
}

static void smblib_iio_deinit(struct smb_charger *chg)
{
	if (!IS_ERR_OR_NULL(chg->iio.temp_chan))
		iio_channel_release(chg->iio.temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.temp_max_chan))
		iio_channel_release(chg->iio.temp_max_chan);
	if (!IS_ERR_OR_NULL(chg->iio.usbin_i_chan))
		iio_channel_release(chg->iio.usbin_i_chan);
	if (!IS_ERR_OR_NULL(chg->iio.usbin_v_chan))
		iio_channel_release(chg->iio.usbin_v_chan);
	if (!IS_ERR_OR_NULL(chg->iio.batt_i_chan))
		iio_channel_release(chg->iio.batt_i_chan);
#ifdef CONFIG_LGE_PM_INOV_GEN3_SYSFS_SUPPORT
	if (!IS_ERR_OR_NULL(chg->iio.temp_hot_max_chan))
		iio_channel_release(chg->iio.temp_hot_max_chan);
	if (!IS_ERR_OR_NULL(chg->iio.skin_temp_chan))
		iio_channel_release(chg->iio.skin_temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.skin_temp_max_chan))
		iio_channel_release(chg->iio.skin_temp_max_chan);
	if (!IS_ERR_OR_NULL(chg->iio.skin_temp_hot_max_chan))
		iio_channel_release(chg->iio.skin_temp_hot_max_chan);
#endif
}

int smblib_init(struct smb_charger *chg)
{
	int rc = 0;

	mutex_init(&chg->lock);
	mutex_init(&chg->write_lock);
	mutex_init(&chg->otg_oc_lock);
	mutex_init(&chg->vconn_oc_lock);
	INIT_WORK(&chg->bms_update_work, bms_update_work);
	INIT_WORK(&chg->rdstd_cc2_detach_work, rdstd_cc2_detach_work);
#ifdef CONFIG_LGE_PM_STEP_CHARGING
	INIT_WORK(&chg->batt_update_work, batt_update_work);
	INIT_DELAYED_WORK(&chg->step_charging_check_work, smblib_step_charging_check_work);
#endif
	INIT_DELAYED_WORK(&chg->hvdcp_detect_work, smblib_hvdcp_detect_work);
	INIT_DELAYED_WORK(&chg->step_soc_req_work, step_soc_req_work);
	INIT_DELAYED_WORK(&chg->clear_hdc_work, clear_hdc_work);
	INIT_WORK(&chg->otg_oc_work, smblib_otg_oc_work);
	INIT_WORK(&chg->vconn_oc_work, smblib_vconn_oc_work);
	INIT_DELAYED_WORK(&chg->otg_ss_done_work, smblib_otg_ss_done_work);
	INIT_DELAYED_WORK(&chg->icl_change_work, smblib_icl_change_work);
#ifdef CONFIG_LGE_PM_DEBUG
	INIT_DELAYED_WORK(&chg->charging_inform_work, charging_information);
#endif
#ifdef CONFIG_LGE_PM_WEAK_BATT_PACK
	INIT_DELAYED_WORK(&chg->batt_pack_check_work, weak_batt_pack_check_work);
	chg->batt_pack_verify_cnt = 0;
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	INIT_WORK(&chg->cable_detect_work, cable_detect_work);
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
	INIT_WORK(&chg->lge_cc_work, lge_cc_work);
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PARALLEL_CONTROLLER
	INIT_WORK(&chg->lge_pl_work, lge_pl_work);
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PSEUDO_BATTERY
	INIT_WORK(&chg->lge_pseudo_work, lge_pseudo_work);
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
	INIT_WORK(&chg->lge_vzw_work, lge_vzw_work);
	INIT_DELAYED_WORK(&chg->vzw_recheck_work, vzw_recheck_work);
	INIT_DELAYED_WORK(&chg->vzw_aicl_done_work, vzw_aicl_done_work);
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_STORE_MODE
	INIT_WORK(&chg->lge_sm_work, lge_sm_work);
#endif
#ifdef CONFIG_IDTP9223_CHARGER
	INIT_WORK(&chg->idtp9223_work, idtp9223_work);
	INIT_DELAYED_WORK(&chg->dc_lcd_current_work, dc_lcd_current_work);
#endif
#ifdef CONFIG_LGE_PM
	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO || lge_get_boot_mode() == LGE_BOOT_MODE_NORMAL)
		chg->is_normal_bootmode = true;
	else
		chg->is_normal_bootmode = false;

	chg->lcd_status=-1;
	chg->pre_settled_ua = -1;
	chg->first_debounce_done = true;
	INIT_DELAYED_WORK(&chg->recovery_boost_back_work, recovery_boost_back_work);
	INIT_DELAYED_WORK(&chg->abnormal_gender_detect_work, abnormal_gender_detect_work);
#endif

	INIT_DELAYED_WORK(&chg->pl_enable_work, smblib_pl_enable_work);
	INIT_WORK(&chg->legacy_detection_work, smblib_legacy_detection_work);
	INIT_DELAYED_WORK(&chg->uusb_otg_work, smblib_uusb_otg_work);
	chg->fake_capacity = -EINVAL;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	chg->is_factory_cable = 0;
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PSEUDO_BATTERY
	chg->pseudo_batt = false;
#endif

	chg->fake_input_current_limited = -EINVAL;

	switch (chg->mode) {
	case PARALLEL_MASTER:
		rc = qcom_batt_init();
		if (rc < 0) {
			smblib_err(chg, "Couldn't init qcom_batt_init rc=%d\n",
				rc);
			return rc;
		}

		rc = smblib_create_votables(chg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't create votables rc=%d\n",
				rc);
			return rc;
		}

		rc = smblib_register_notifier(chg);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't register notifier rc=%d\n", rc);
			return rc;
		}
#ifdef CONFIG_LGE_PM_SUPPORT_LG_POWER_CLASS
		rc = lge_smblib_register_notifier(chg);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't register lge notifier rc=%d\n", rc);
			return rc;
		}
#endif
#ifdef CONFIG_LGE_PM_DEBUG
		schedule_delayed_work(&chg->charging_inform_work,
			round_jiffies_relative(msecs_to_jiffies(CHARGING_INFORM_NORMAL_TIME)));
#endif

		chg->bms_psy = power_supply_get_by_name("bms");
		chg->pl.psy = power_supply_get_by_name("parallel");
#ifdef CONFIG_IDTP9223_CHARGER
		chg->wlc_psy = power_supply_get_by_name("dc-wireless");
#endif
#ifdef CONFIG_LGE_PM
		chg->pseudo_usb_type = POWER_SUPPLY_TYPE_UNKNOWN;
#endif
		break;
	case PARALLEL_SLAVE:
		break;
	default:
		smblib_err(chg, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

	return rc;
}

int smblib_deinit(struct smb_charger *chg)
{
	switch (chg->mode) {
	case PARALLEL_MASTER:
		cancel_work_sync(&chg->bms_update_work);
		cancel_work_sync(&chg->rdstd_cc2_detach_work);
		cancel_delayed_work_sync(&chg->hvdcp_detect_work);
		cancel_delayed_work_sync(&chg->step_soc_req_work);
		cancel_delayed_work_sync(&chg->clear_hdc_work);
		cancel_work_sync(&chg->otg_oc_work);
		cancel_work_sync(&chg->vconn_oc_work);
		cancel_delayed_work_sync(&chg->otg_ss_done_work);
		cancel_delayed_work_sync(&chg->icl_change_work);
		cancel_delayed_work_sync(&chg->pl_enable_work);
		cancel_work_sync(&chg->legacy_detection_work);
		cancel_delayed_work_sync(&chg->uusb_otg_work);
		power_supply_unreg_notifier(&chg->nb);
		smblib_destroy_votables(chg);
		qcom_batt_deinit();
		break;
	case PARALLEL_SLAVE:
		break;
	default:
		smblib_err(chg, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

	smblib_iio_deinit(chg);

	return 0;
}
