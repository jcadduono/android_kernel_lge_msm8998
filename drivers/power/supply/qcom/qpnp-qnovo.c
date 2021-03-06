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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/qpnp/qpnp-revid.h>
#ifdef CONFIG_LGE_PM
#include <linux/delay.h>
#endif
#include <linux/pmic-voter.h>

#define QNOVO_REVISION1		0x00
#define QNOVO_REVISION2		0x01
#define QNOVO_PERPH_TYPE	0x04
#define QNOVO_PERPH_SUBTYPE	0x05
#define QNOVO_PTTIME_STS	0x07
#define QNOVO_PTRAIN_STS	0x08
#define QNOVO_ERROR_STS		0x09
#define QNOVO_ERROR_BIT		BIT(0)
#define QNOVO_ERROR_STS2	0x0A
#define QNOVO_ERROR_CHARGING_DISABLED	BIT(1)
#define QNOVO_INT_RT_STS	0x10
#define QNOVO_INT_SET_TYPE	0x11
#define QNOVO_INT_POLARITY_HIGH	0x12
#define QNOVO_INT_POLARITY_LOW	0x13
#define QNOVO_INT_LATCHED_CLR	0x14
#define QNOVO_INT_EN_SET	0x15
#define QNOVO_INT_EN_CLR	0x16
#define QNOVO_INT_LATCHED_STS	0x18
#define QNOVO_INT_PENDING_STS	0x19
#define QNOVO_INT_MID_SEL	0x1A
#define QNOVO_INT_PRIORITY	0x1B
#define QNOVO_PE_CTRL		0x40
#define QNOVO_PREST1_CTRL	0x41
#define QNOVO_PPULS1_LSB_CTRL	0x42
#define QNOVO_PPULS1_MSB_CTRL	0x43
#define QNOVO_NREST1_CTRL	0x44
#define QNOVO_NPULS1_CTRL	0x45
#define QNOVO_PPCNT_CTRL	0x46
#define QNOVO_VLIM1_LSB_CTRL	0x47
#define QNOVO_VLIM1_MSB_CTRL	0x48
#define QNOVO_PTRAIN_EN		0x49
#define QNOVO_PTRAIN_EN_BIT	BIT(0)
#define QNOVO_PE_CTRL2		0x4A
#define QNOVO_PREST2_LSB_CTRL	0x50
#define QNOVO_PREST2_MSB_CTRL	0x51
#define QNOVO_PPULS2_LSB_CTRL	0x52
#define QNOVO_PPULS2_MSB_CTRL	0x53
#define QNOVO_NREST2_CTRL	0x54
#define QNOVO_NPULS2_CTRL	0x55
#define QNOVO_VLIM2_LSB_CTRL	0x56
#define QNOVO_VLIM2_MSB_CTRL	0x57
#define QNOVO_PVOLT1_LSB	0x60
#define QNOVO_PVOLT1_MSB	0x61
#define QNOVO_PCUR1_LSB		0x62
#define QNOVO_PCUR1_MSB		0x63
#define QNOVO_PVOLT2_LSB	0x70
#define QNOVO_PVOLT2_MSB	0x71
#define QNOVO_RVOLT2_LSB	0x72
#define QNOVO_RVOLT2_MSB	0x73
#define QNOVO_PCUR2_LSB		0x74
#define QNOVO_PCUR2_MSB		0x75
#define QNOVO_SCNT		0x80
#define QNOVO_VMAX_LSB		0x90
#define QNOVO_VMAX_MSB		0x91
#define QNOVO_SNUM		0x92

/* Registers ending in 0 imply external rsense */
#define QNOVO_IADC_OFFSET_0	0xA0
#define QNOVO_IADC_OFFSET_1	0xA1
#define QNOVO_IADC_GAIN_0	0xA2
#define QNOVO_IADC_GAIN_1	0xA3
#define QNOVO_VADC_OFFSET	0xA4
#define QNOVO_VADC_GAIN		0xA5
#define QNOVO_IADC_GAIN_2	0xA6
#define QNOVO_SPARE		0xA7
#define QNOVO_STRM_CTRL		0xA8
#define QNOVO_IADC_OFFSET_OVR_VAL	0xA9
#define QNOVO_IADC_OFFSET_OVR		0xAA

#define QNOVO_DISABLE_CHARGING		0xAB
#define ERR_SWITCHER_DISABLED		BIT(7)
#define ERR_JEITA_SOFT_CONDITION	BIT(6)
#define ERR_BAT_OV			BIT(5)
#define ERR_CV_MODE			BIT(4)
#define ERR_BATTERY_MISSING		BIT(3)
#define ERR_SAFETY_TIMER_EXPIRED	BIT(2)
#define ERR_CHARGING_DISABLED		BIT(1)
#define ERR_JEITA_HARD_CONDITION	BIT(0)

#define DCIN_AICL_OPTIONS_CFG_REG		0x1480
#define SUSPEND_ON_COLLAPSE_DCIN_BIT		BIT(7)
#define DCIN_AICL_HDC_EN_BIT			BIT(6)
#define DCIN_AICL_START_AT_MAX_BIT		BIT(5)
#define DCIN_AICL_RERUN_EN_BIT			BIT(4)
#define DCIN_AICL_ADC_EN_BIT			BIT(3)
#define DCIN_AICL_EN_BIT			BIT(2)
#define DCIN_HV_COLLAPSE_RESPONSE_BIT		BIT(1)
#define DCIN_LV_COLLAPSE_RESPONSE_BIT		BIT(0)

#define QNOVO_TR_IADC_OFFSET_0	0xF1
#define QNOVO_TR_IADC_OFFSET_1	0xF2

#define DRV_MAJOR_VERSION	1
#define DRV_MINOR_VERSION	0

#define IADC_LSB_NA	2441400
#define VADC_LSB_NA	1220700
#define GAIN_LSB_FACTOR	976560

#ifdef CONFIG_LGE_PM
#define FG_AVAILABLE_VOTER	"FG_AVAILABLE_VOTER"
#define QNOVO_ERR_VOTER		"qnovo_err_voter"
#define INPUT_PB_VOTER		"input_pb_voter"
#define PT_RESTART_VOTER	"PT_RESTART_VOTER"
#define USER_VOTER		"user_voter"
#else
#define USER_VOTER		"user_voter"
#define OK_TO_QNOVO_VOTER	"ok_to_qnovo_voter"
#define QNOVO_VOTER		"qnovo_voter"
#endif

struct qnovo_dt_props {
	bool			external_rsense;
	struct device_node	*revid_dev_node;
};

struct qnovo {
	int			base;
	struct mutex		write_lock;
	struct regmap		*regmap;
	struct qnovo_dt_props	dt;
	struct device		*dev;
	struct votable		*disable_votable;
#ifdef CONFIG_LGE_PM
	struct votable		*qnovo_awake_votable;
#endif
	struct class		qnovo_class;
	struct pmic_revid_data	*pmic_rev_id;
	u32			wa_flags;
	s64			external_offset_nA;
	s64			internal_offset_nA;
	s64			offset_nV;
	s64			external_i_gain_mega;
	s64			internal_i_gain_mega;
	s64			v_gain_mega;
	struct notifier_block	nb;
	struct power_supply	*batt_psy;
#ifdef CONFIG_LGE_PM
	struct power_supply	*bms_psy;
	struct power_supply	*usb_psy;
	struct power_supply	*dc_psy;
#endif
	struct work_struct	status_change_work;
#ifdef CONFIG_LGE_PM
	struct work_struct	input_probation_work;
#endif
	int			fv_uV_request;
	int			fcc_uA_request;
	bool			ok_to_qnovo;
#ifdef CONFIG_LGE_PM
	bool			usb_present;
	bool			dc_present;
	bool			input_changed;
	bool			input_probation_wip;

	struct work_struct	ptrain_restart_work;
	bool			pt_restart_flag;
#endif

#ifdef CONFIG_LGE_PM_BATT_MANAGER
	struct power_supply *qnovo_psy;
#endif

};

static int debug_mask;
module_param_named(debug_mask, debug_mask, int, 0600);

#define qnovo_dbg(chip, reason, fmt, ...)				\
	do {								\
		if (debug_mask & (reason))				\
			dev_info(chip->dev, fmt, ##__VA_ARGS__);	\
		else							\
			dev_dbg(chip->dev, fmt, ##__VA_ARGS__);		\
	} while (0)

static bool is_secure(struct qnovo *chip, int addr)
{
	/* assume everything above 0x40 is secure */
	return (bool)(addr >= 0x40);
}

static int qnovo_read(struct qnovo *chip, u16 addr, u8 *buf, int len)
{
	return regmap_bulk_read(chip->regmap, chip->base + addr, buf, len);
}

static int qnovo_masked_write(struct qnovo *chip, u16 addr, u8 mask, u8 val)
{
	int rc = 0;

	mutex_lock(&chip->write_lock);
	if (is_secure(chip, addr)) {
		rc = regmap_write(chip->regmap,
				((chip->base + addr) & ~(0xFF)) | 0xD0, 0xA5);
		if (rc < 0)
			goto unlock;
	}

	rc = regmap_update_bits(chip->regmap, chip->base + addr, mask, val);

unlock:
	mutex_unlock(&chip->write_lock);
	return rc;
}

static int qnovo_write(struct qnovo *chip, u16 addr, u8 *buf, int len)
{
	int i, rc = 0;
	bool is_start_secure, is_end_secure;

	is_start_secure = is_secure(chip, addr);
	is_end_secure = is_secure(chip, addr + len);

	if (!is_start_secure && !is_end_secure) {
		mutex_lock(&chip->write_lock);
		rc = regmap_bulk_write(chip->regmap, chip->base + addr,
					buf, len);
		goto unlock;
	}

	mutex_lock(&chip->write_lock);
	for (i = addr; i < addr + len; i++) {
		if (is_secure(chip, i)) {
			rc = regmap_write(chip->regmap,
				((chip->base + i) & ~(0xFF)) | 0xD0, 0xA5);
			if (rc < 0)
				goto unlock;
		}
		rc = regmap_write(chip->regmap, chip->base + i, buf[i - addr]);
		if (rc < 0)
			goto unlock;
	}

unlock:
	mutex_unlock(&chip->write_lock);
	return rc;
}

static bool is_batt_available(struct qnovo *chip)
{
	if (!chip->batt_psy)
		chip->batt_psy = power_supply_get_by_name("battery");

	if (!chip->batt_psy)
		return false;

	return true;
}

#ifdef CONFIG_LGE_PM
static bool is_usb_available(struct qnovo *chip)
{
	if (!chip->usb_psy)
		chip->usb_psy = power_supply_get_by_name("usb");

	if (!chip->usb_psy)
		return false;

	return true;
}

static bool is_dc_available(struct qnovo *chip)
{
	if (!chip->dc_psy)
		chip->dc_psy = power_supply_get_by_name("dc");

	if (!chip->dc_psy)
		return false;

	return true;
}

static bool is_fg_available(struct qnovo *chip)
{
	if (!chip->bms_psy)
		chip->bms_psy = power_supply_get_by_name("bms");

	if (!chip->bms_psy)
		return false;

	return true;
}

static int update_input_present(struct qnovo *chip)
{
	union power_supply_propval val;
	bool input_present;
	int rc;

	rc = power_supply_get_property(chip->usb_psy,
		POWER_SUPPLY_PROP_PRESENT, &val);

	input_present =  rc < 0 ? false : val.intval;

	if (input_present ^ chip->usb_present) {
		chip->usb_present = input_present;
		chip->input_changed = true;
	}

	rc = power_supply_get_property(chip->dc_psy,
		POWER_SUPPLY_PROP_PRESENT, &val);

	input_present =  rc < 0 ? false : val.intval;

	if (input_present ^ chip->dc_present) {
		chip->dc_present = input_present;
		chip->input_changed = true;
	}

	return 0;
}

static bool is_usb_present(struct qnovo *chip)
{
	if (is_usb_available(chip) && is_dc_available(chip))
		update_input_present(chip);

	if (chip->usb_present)
		return true;
	return false;
}

static bool is_dc_present(struct qnovo *chip)
{
	if (is_usb_available(chip) && is_dc_available(chip))
		update_input_present(chip);

	if (chip->dc_present && !chip->usb_present)
		return true;
	return false;
}
#endif

static int qnovo_batt_psy_update(struct qnovo *chip, bool disable)
{
	union power_supply_propval pval = {0};
	int rc = 0;

	if (!is_batt_available(chip))
		return -EINVAL;

	if (chip->fv_uV_request != -EINVAL) {
		pval.intval = disable ? -EINVAL : chip->fv_uV_request;
		rc = power_supply_set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_VOLTAGE_QNOVO,
			&pval);
		if (rc < 0) {
			pr_err("Couldn't set prop qnovo_fv rc = %d\n", rc);
			return -EINVAL;
		}
	}

	if (chip->fcc_uA_request != -EINVAL) {
		pval.intval = disable ? -EINVAL : chip->fcc_uA_request;
		rc = power_supply_set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CURRENT_QNOVO,
			&pval);
		if (rc < 0) {
			pr_err("Couldn't set prop qnovo_fcc rc = %d\n", rc);
			return -EINVAL;
		}
	}

	return rc;
}

#ifdef CONFIG_LGE_PM
static int qnovo_ok_disable_cb(struct votable *votable, void *data, int disable,
					const char *client)
#else
static int qnovo_disable_cb(struct votable *votable, void *data, int disable,
					const char *client)
#endif
{
	struct qnovo *chip = data;
	union power_supply_propval pval = {0};
	int rc;

#ifdef CONFIG_LGE_PM
	if (!is_batt_available(chip))
		return -EINVAL;

	pval.intval = !disable;
	rc = power_supply_set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CHARGE_QNOVO_ENABLE,
			&pval);
	if (rc < 0) {
		pr_err("Couldn't set prop qnovo_enable rc = %d\n", rc);
		return -EINVAL;
	}
#endif

	rc = qnovo_batt_psy_update(chip, disable);

#ifdef CONFIG_LGE_PM
	chip->ok_to_qnovo = !disable;

	/*
	 * fg must be available for enable FG_AVAILABLE_VOTER
	 * won't enable it otherwise
	 */
	if (is_fg_available(chip) && chip->ok_to_qnovo)
		rc = power_supply_set_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CHARGE_QNOVO_ENABLE,
				&pval);
	kobject_uevent(&chip->dev->kobj, KOBJ_CHANGE);
#endif
	return rc;
}

static int qnovo_parse_dt(struct qnovo *chip)
{
	struct device_node *node = chip->dev->of_node;
	int rc;

	if (!node) {
		pr_err("device tree node missing\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "reg", &chip->base);
	if (rc < 0) {
		pr_err("Couldn't read base rc = %d\n", rc);
		return rc;
	}

	chip->dt.external_rsense = of_property_read_bool(node,
			"qcom,external-rsense");

	chip->dt.revid_dev_node = of_parse_phandle(node, "qcom,pmic-revid", 0);
	if (!chip->dt.revid_dev_node) {
		pr_err("Missing qcom,pmic-revid property - driver failed\n");
		return -EINVAL;
	}

	return 0;
}

enum {
	VER = 0,
	OK_TO_QNOVO,
	QNOVO_ENABLE,
	PT_ENABLE,
	FV_REQUEST,
	FCC_REQUEST,
	PE_CTRL_REG,
	PE_CTRL2_REG,
	PTRAIN_STS_REG,
	INT_RT_STS_REG,
	ERR_STS2_REG,
	PREST1,
	PPULS1,
	NREST1,
	NPULS1,
	PPCNT,
	VLIM1,
	PVOLT1,
	PCUR1,
	PTTIME,
	PREST2,
	PPULS2,
	NREST2,
	NPULS2,
	VLIM2,
	PVOLT2,
	RVOLT2,
	PCUR2,
	SCNT,
	VMAX,
	SNUM,
	VBATT,
	IBATT,
	BATTTEMP,
	BATTSOC,
};

struct param_info {
	char	*name;
	int	start_addr;
	int	num_regs;
	int	reg_to_unit_multiplier;
	int	reg_to_unit_divider;
	int	reg_to_unit_offset;
	int	min_val;
	int	max_val;
	char	*units_str;
};

static struct param_info params[] = {
	[PT_ENABLE] = {
		.name			= "PT_ENABLE",
		.start_addr		= QNOVO_PTRAIN_EN,
		.num_regs		= 1,
		.units_str		= "",
	},
	[FV_REQUEST] = {
		.units_str		= "uV",
	},
	[FCC_REQUEST] = {
		.units_str		= "uA",
	},
	[PE_CTRL_REG] = {
		.name			= "CTRL_REG",
		.start_addr		= QNOVO_PE_CTRL,
		.num_regs		= 1,
		.units_str		= "",
	},
	[PE_CTRL2_REG] = {
		.name			= "PE_CTRL2_REG",
		.start_addr		= QNOVO_PE_CTRL2,
		.num_regs		= 1,
		.units_str		= "",
	},
	[PTRAIN_STS_REG] = {
		.name			= "PTRAIN_STS",
		.start_addr		= QNOVO_PTRAIN_STS,
		.num_regs		= 1,
		.units_str		= "",
	},
	[INT_RT_STS_REG] = {
		.name			= "INT_RT_STS",
		.start_addr		= QNOVO_INT_RT_STS,
		.num_regs		= 1,
		.units_str		= "",
	},
	[ERR_STS2_REG] = {
		.name			= "RAW_CHGR_ERR",
		.start_addr		= QNOVO_ERROR_STS2,
		.num_regs		= 1,
		.units_str		= "",
	},
	[PREST1] = {
		.name			= "PREST1",
		.start_addr		= QNOVO_PREST1_CTRL,
		.num_regs		= 1,
		.reg_to_unit_multiplier	= 5,
		.reg_to_unit_divider	= 1,
		.min_val		= 5,
		.max_val		= 255,
		.units_str		= "mS",
	},
	[PPULS1] = {
		.name			= "PPULS1",
		.start_addr		= QNOVO_PPULS1_LSB_CTRL,
		.num_regs		= 2,
		.reg_to_unit_multiplier	= 1600, /* converts to uC */
		.reg_to_unit_divider	= 1,
		.min_val		= 30000,
		.max_val		= 65535000,
		.units_str		= "uC",
	},
	[NREST1] = {
		.name			= "NREST1",
		.start_addr		= QNOVO_NREST1_CTRL,
		.num_regs		= 1,
		.reg_to_unit_multiplier	= 5,
		.reg_to_unit_divider	= 1,
		.min_val		= 5,
		.max_val		= 255,
		.units_str		= "mS",
	},
	[NPULS1] = {
		.name			= "NPULS1",
		.start_addr		= QNOVO_NPULS1_CTRL,
		.num_regs		= 1,
		.reg_to_unit_multiplier	= 5,
		.reg_to_unit_divider	= 1,
		.min_val		= 0,
		.max_val		= 255,
		.units_str		= "mS",
	},
	[PPCNT] = {
		.name			= "PPCNT",
		.start_addr		= QNOVO_PPCNT_CTRL,
		.num_regs		= 1,
		.reg_to_unit_multiplier	= 1,
		.reg_to_unit_divider	= 1,
		.min_val		= 1,
		.max_val		= 255,
		.units_str		= "pulses",
	},
	[VLIM1] = {
		.name			= "VLIM1",
		.start_addr		= QNOVO_VLIM1_LSB_CTRL,
		.num_regs		= 2,
		.reg_to_unit_multiplier	= 610350, /* converts to nV */
		.reg_to_unit_divider	= 1,
		.min_val		= 2200000,
		.max_val		= 4500000,
		.units_str		= "uV",
	},
	[PVOLT1] = {
		.name			= "PVOLT1",
		.start_addr		= QNOVO_PVOLT1_LSB,
		.num_regs		= 2,
		.reg_to_unit_multiplier	= 610350, /* converts to nV */
		.reg_to_unit_divider	= 1,
		.units_str		= "uV",
	},
	[PCUR1] = {
		.name			= "PCUR1",
		.start_addr		= QNOVO_PCUR1_LSB,
		.num_regs		= 2,
		.reg_to_unit_multiplier	= 1220700, /* converts to nA */
		.reg_to_unit_divider	= 1,
		.units_str		= "uA",
	},
	[PTTIME] = {
		.name			= "PTTIME",
		.start_addr		= QNOVO_PTTIME_STS,
		.num_regs		= 1,
		.reg_to_unit_multiplier	= 2,
		.reg_to_unit_divider	= 1,
		.units_str		= "S",
	},
	[PREST2] = {
		.name			= "PREST2",
		.start_addr		= QNOVO_PREST2_LSB_CTRL,
		.num_regs		= 2,
		.reg_to_unit_multiplier	= 5,
		.reg_to_unit_divider	= 1,
		.min_val		= 5,
		.max_val		= 65535,
		.units_str		= "mS",
	},
	[PPULS2] = {
		.name			= "PPULS2",
		.start_addr		= QNOVO_PPULS2_LSB_CTRL,
		.num_regs		= 2,
		.reg_to_unit_multiplier	= 1600, /* converts to uC */
		.reg_to_unit_divider	= 1,
		.min_val		= 30000,
		.max_val		= 65535000,
		.units_str		= "uC",
	},
	[NREST2] = {
		.name			= "NREST2",
		.start_addr		= QNOVO_NREST2_CTRL,
		.num_regs		= 1,
		.reg_to_unit_multiplier	= 5,
		.reg_to_unit_divider	= 1,
		.reg_to_unit_offset	= -5,
		.min_val		= 5,
		.max_val		= 255,
		.units_str		= "mS",
	},
	[NPULS2] = {
		.name			= "NPULS2",
		.start_addr		= QNOVO_NPULS2_CTRL,
		.num_regs		= 1,
		.reg_to_unit_multiplier	= 5,
		.reg_to_unit_divider	= 1,
		.min_val		= 0,
		.max_val		= 255,
		.units_str		= "mS",
	},
	[VLIM2] = {
		.name			= "VLIM2",
		.start_addr		= QNOVO_VLIM2_LSB_CTRL,
		.num_regs		= 2,
		.reg_to_unit_multiplier	= 610350, /* converts to nV */
		.reg_to_unit_divider	= 1,
		.min_val		= 2200000,
		.max_val		= 4500000,
		.units_str		= "uV",
	},
	[PVOLT2] = {
		.name			= "PVOLT2",
		.start_addr		= QNOVO_PVOLT2_LSB,
		.num_regs		= 2,
		.reg_to_unit_multiplier	= 610350, /* converts to nV */
		.reg_to_unit_divider	= 1,
		.units_str		= "uV",
	},
	[RVOLT2] = {
		.name			= "RVOLT2",
		.start_addr		= QNOVO_RVOLT2_LSB,
		.num_regs		= 2,
		.reg_to_unit_multiplier	= 610350,
		.reg_to_unit_divider	= 1,
		.units_str		= "uV",
	},
	[PCUR2] = {
		.name			= "PCUR2",
		.start_addr		= QNOVO_PCUR2_LSB,
		.num_regs		= 2,
		.reg_to_unit_multiplier	= 1220700, /* converts to nA */
		.reg_to_unit_divider	= 1,
		.units_str		= "uA",
	},
	[SCNT] = {
		.name			= "SCNT",
		.start_addr		= QNOVO_SCNT,
		.num_regs		= 1,
		.reg_to_unit_multiplier	= 1,
		.reg_to_unit_divider	= 1,
		.min_val		= 0,
		.max_val		= 255,
		.units_str		= "pulses",
	},
	[VMAX] = {
		.name			= "VMAX",
		.start_addr		= QNOVO_VMAX_LSB,
		.num_regs		= 2,
		.reg_to_unit_multiplier	= 814000, /* converts to nV */
		.reg_to_unit_divider	= 1,
		.units_str		= "uV",
	},
	[SNUM] = {
		.name			= "SNUM",
		.start_addr		= QNOVO_SNUM,
		.num_regs		= 1,
		.reg_to_unit_multiplier	= 1,
		.reg_to_unit_divider	= 1,
		.units_str		= "pulses",
	},
	[VBATT]	= {
		.name			= "POWER_SUPPLY_PROP_VOLTAGE_NOW",
		.start_addr		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
		.units_str		= "uV",
	},
	[IBATT]	= {
		.name			= "POWER_SUPPLY_PROP_CURRENT_NOW",
		.start_addr		= POWER_SUPPLY_PROP_CURRENT_NOW,
		.units_str		= "uA",
	},
	[BATTTEMP] = {
		.name			= "POWER_SUPPLY_PROP_TEMP",
		.start_addr		= POWER_SUPPLY_PROP_TEMP,
		.units_str		= "uV",
	},
	[BATTSOC] = {
		.name			= "POWER_SUPPLY_PROP_CAPACITY",
		.start_addr		= POWER_SUPPLY_PROP_CAPACITY,
		.units_str		= "%",
	},
};

static struct class_attribute qnovo_attributes[];

#ifdef CONFIG_LGE_PM_BATT_MANAGER
static int current_get(struct qnovo *chip)
{
	int i = PCUR1;//current
	u8 buf[2] = {0, 0};
	int rc;
	int comp_val_uA;
	s64 regval_nA;
	s64 gain, offset_nA, comp_val_nA;

	rc = qnovo_read(chip, params[i].start_addr, buf, params[i].num_regs);
	if (rc < 0) {
		pr_err("Couldn't read %s rc = %d\n", params[i].name, rc);
		return -EINVAL;
	}

	if (buf[1] & BIT(5))
		buf[1] |= GENMASK(7, 6);

	regval_nA = (s16)(buf[1] << 8 | buf[0]);
	regval_nA = div_s64(regval_nA * params[i].reg_to_unit_multiplier,
					params[i].reg_to_unit_divider)
			- params[i].reg_to_unit_offset;

	if (chip->dt.external_rsense) {
		offset_nA = chip->external_offset_nA;
		gain = chip->external_i_gain_mega;
	} else {
		offset_nA = chip->internal_offset_nA;
		gain = chip->internal_i_gain_mega;
	}

	comp_val_nA = div_s64(regval_nA * gain, 1000000) - offset_nA;
	comp_val_uA = div_s64(comp_val_nA, 1000);
	return comp_val_uA;
}

static enum power_supply_property qnovo_psy_props[] = {
	POWER_SUPPLY_PROP_CURRENT_QNOVO,
};

static int qnovo_get_property(struct power_supply *psy, enum power_supply_property property, union power_supply_propval *val) {
	struct qnovo *chip = power_supply_get_drvdata(psy);
	int rc = 0;
	if (chip == NULL) {
		pr_err("Couldn't get qnovo class from propert\n");
		return -ENODEV;
	}
	switch (property) {
		case POWER_SUPPLY_PROP_CURRENT_QNOVO:
			val->intval = current_get(chip);
			break;
		default:
			break;
	}
	return rc;
}

static const struct power_supply_desc qnovo_desc = {
	.get_property = qnovo_get_property,
	.properties = qnovo_psy_props,
	.num_properties = ARRAY_SIZE(qnovo_psy_props),
	.name = "qcom,qnovo-driver",
};
#endif


static ssize_t version_show(struct class *c, struct class_attribute *attr,
			char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d.%d\n",
			DRV_MAJOR_VERSION, DRV_MINOR_VERSION);
}

static ssize_t ok_to_qnovo_show(struct class *c, struct class_attribute *attr,
			char *buf)
{
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ok_to_qnovo);
}

static ssize_t qnovo_enable_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
#ifdef CONFIG_LGE_PM
	union power_supply_propval pval = {0};

	int rc;

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CHARGE_QNOVO_ENABLE,
			&pval);
	if (rc < 0)
		pr_err("Couldn't set prop qnovo_enable rc = %d\n", rc);

	return snprintf(ubuf, PAGE_SIZE, "%d\n", pval.intval);
#else
	int val = get_effective_result(chip->disable_votable);
	return snprintf(ubuf, PAGE_SIZE, "%d\n", !val);
#endif
}

static ssize_t qnovo_enable_store(struct class *c, struct class_attribute *attr,
			const char *ubuf, size_t count)
{
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
#ifdef CONFIG_LGE_PM
	union power_supply_propval pval = {0};
#endif
	unsigned long val;
#ifdef CONFIG_LGE_PM
	int rc;
#endif

	if (kstrtoul(ubuf, 0, &val))
		return -EINVAL;

#ifdef CONFIG_LGE_PM
	if (!is_batt_available(chip))
		return -EINVAL;

	pval.intval = val;
	rc = power_supply_set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CHARGE_QNOVO_ENABLE,
			&pval);
	if (rc < 0) {
		pr_err("Couldn't set prop qnovo_enable rc = %d\n", rc);
		return -EINVAL;
	}
	vote(chip->disable_votable, USER_VOTER, !val, 0);
#else
	vote(chip->disable_votable, USER_VOTER, !val, 0);
#endif
	return count;
}

static ssize_t pt_enable_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	int i = attr - qnovo_attributes;
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
	u8 buf[2] = {0, 0};
	u16 regval;
	int rc;

	rc = qnovo_read(chip, params[i].start_addr, buf, params[i].num_regs);
	if (rc < 0) {
		pr_err("Couldn't read %s rc = %d\n", params[i].name, rc);
		return -EINVAL;
	}
	regval = buf[1] << 8 | buf[0];

	return snprintf(ubuf, PAGE_SIZE, "%d\n",
				(int)(regval & QNOVO_PTRAIN_EN_BIT));
}

static ssize_t pt_enable_store(struct class *c, struct class_attribute *attr,
			const char *ubuf, size_t count)
{
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
	unsigned long val;
	int rc = 0;
#ifdef CONFIG_LGE_PM
	u8 pt_en;
#endif

	if (get_effective_result(chip->disable_votable))
		return -EINVAL;

	if (kstrtoul(ubuf, 0, &val))
		return -EINVAL;

#ifdef CONFIG_LGE_PM
	if (val) {
		rc = qnovo_read(chip, QNOVO_PTRAIN_EN, &pt_en, 1);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't read QNOVO_PTRAIN_EN rc = %d\n",
				rc);
			return -EINVAL;
		}

		if (pt_en) {
			pr_info("qni:PT_RESTART enable failed - pt_enable = 1.\n");
			return -EINVAL;
		}
	}
	else {
		cancel_work_sync(&chip->ptrain_restart_work);
		vote(chip->qnovo_awake_votable, PT_RESTART_VOTER, false, 0);
	}

	if (chip->pt_restart_flag)
		msleep(100);
#endif

	rc = qnovo_masked_write(chip, QNOVO_PTRAIN_EN, QNOVO_PTRAIN_EN_BIT,
				 (bool)val ? QNOVO_PTRAIN_EN_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't %s pulse train rc=%d\n",
			(bool)val ? "enable" : "disable", rc);
		return rc;
	}

#ifdef CONFIG_LGE_PM
	if (val) {
		vote(chip->qnovo_awake_votable, PT_RESTART_VOTER, true, 0);
		schedule_work(&chip->ptrain_restart_work);
	}
#endif

	return count;
}

static ssize_t val_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
	int i = attr - qnovo_attributes;
	int val = 0;

	if (i == FV_REQUEST)
		val = chip->fv_uV_request;

	if (i == FCC_REQUEST)
		val = chip->fcc_uA_request;

	return snprintf(ubuf, PAGE_SIZE, "%d\n", val);
}

static ssize_t val_store(struct class *c, struct class_attribute *attr,
			const char *ubuf, size_t count)
{
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
	int i = attr - qnovo_attributes;
	unsigned long val;

	if (kstrtoul(ubuf, 0, &val))
		return -EINVAL;

	if (i == FV_REQUEST)
		chip->fv_uV_request = val;

	if (i == FCC_REQUEST)
		chip->fcc_uA_request = val;

#ifdef CONFIG_LGE_PM
	if (chip->ok_to_qnovo)
#else
	if (!get_effective_result(chip->disable_votable))
#endif
		qnovo_batt_psy_update(chip, false);

	return count;
}

static ssize_t reg_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	int i = attr - qnovo_attributes;
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
	u8 buf[2] = {0, 0};
	u16 regval;
	int rc;

	rc = qnovo_read(chip, params[i].start_addr, buf, params[i].num_regs);
	if (rc < 0) {
		pr_err("Couldn't read %s rc = %d\n", params[i].name, rc);
		return -EINVAL;
	}
	regval = buf[1] << 8 | buf[0];

	return snprintf(ubuf, PAGE_SIZE, "0x%04x\n", regval);
}

static ssize_t reg_store(struct class *c, struct class_attribute *attr,
			const char *ubuf, size_t count)
{
	int i = attr - qnovo_attributes;
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
	u8 buf[2] = {0, 0};
	unsigned long val;
	int rc;

	if (kstrtoul(ubuf, 0, &val))
		return -EINVAL;

	buf[0] = val & 0xFF;
	buf[1] = (val >> 8) & 0xFF;

	rc = qnovo_write(chip, params[i].start_addr, buf, params[i].num_regs);
	if (rc < 0) {
		pr_err("Couldn't write %s rc = %d\n", params[i].name, rc);
		return -EINVAL;
	}
	return count;
}

static ssize_t time_show(struct class *c, struct class_attribute *attr,
		char *ubuf)
{
	int i = attr - qnovo_attributes;
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
	u8 buf[2] = {0, 0};
	u16 regval;
	int val;
	int rc;

	rc = qnovo_read(chip, params[i].start_addr, buf, params[i].num_regs);
	if (rc < 0) {
		pr_err("Couldn't read %s rc = %d\n", params[i].name, rc);
		return -EINVAL;
	}
	regval = buf[1] << 8 | buf[0];

	val = ((regval * params[i].reg_to_unit_multiplier)
			/ params[i].reg_to_unit_divider)
		- params[i].reg_to_unit_offset;

	return snprintf(ubuf, PAGE_SIZE, "%d\n", val);
}

static ssize_t time_store(struct class *c, struct class_attribute *attr,
		       const char *ubuf, size_t count)
{
	int i = attr - qnovo_attributes;
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
	u8 buf[2] = {0, 0};
	u16 regval;
	unsigned long val;
	int rc;

	if (kstrtoul(ubuf, 0, &val))
		return -EINVAL;

	if (val < params[i].min_val || val > params[i].max_val) {
		pr_err("Out of Range %d%s for %s\n", (int)val,
				params[i].units_str,
				params[i].name);
		return -ERANGE;
	}

	regval = (((int)val + params[i].reg_to_unit_offset)
			* params[i].reg_to_unit_divider)
		/ params[i].reg_to_unit_multiplier;
	buf[0] = regval & 0xFF;
	buf[1] = (regval >> 8) & 0xFF;

	rc = qnovo_write(chip, params[i].start_addr, buf, params[i].num_regs);
	if (rc < 0) {
		pr_err("Couldn't write %s rc = %d\n", params[i].name, rc);
		return -EINVAL;
	}

	return count;
}

static ssize_t current_show(struct class *c, struct class_attribute *attr,
				char *ubuf)
{
	int i = attr - qnovo_attributes;
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
	u8 buf[2] = {0, 0};
	int rc;
	int comp_val_uA;
	s64 regval_nA;
	s64 gain, offset_nA, comp_val_nA;

	rc = qnovo_read(chip, params[i].start_addr, buf, params[i].num_regs);
	if (rc < 0) {
		pr_err("Couldn't read %s rc = %d\n", params[i].name, rc);
		return -EINVAL;
	}

	if (buf[1] & BIT(5))
		buf[1] |= GENMASK(7, 6);

	regval_nA = (s16)(buf[1] << 8 | buf[0]);
	regval_nA = div_s64(regval_nA * params[i].reg_to_unit_multiplier,
					params[i].reg_to_unit_divider)
			- params[i].reg_to_unit_offset;

	if (chip->dt.external_rsense) {
		offset_nA = chip->external_offset_nA;
		gain = chip->external_i_gain_mega;
	} else {
		offset_nA = chip->internal_offset_nA;
		gain = chip->internal_i_gain_mega;
	}

	comp_val_nA = div_s64(regval_nA * gain, 1000000) - offset_nA;
	comp_val_uA = div_s64(comp_val_nA, 1000);

	return snprintf(ubuf, PAGE_SIZE, "%d\n", comp_val_uA);
}

static ssize_t voltage_show(struct class *c, struct class_attribute *attr,
				char *ubuf)
{
	int i = attr - qnovo_attributes;
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
	u8 buf[2] = {0, 0};
	int rc;
	int comp_val_uV;
	s64 regval_nV;
	s64 gain, offset_nV, comp_val_nV;

	rc = qnovo_read(chip, params[i].start_addr, buf, params[i].num_regs);
	if (rc < 0) {
		pr_err("Couldn't read %s rc = %d\n", params[i].name, rc);
		return -EINVAL;
	}
	regval_nV = buf[1] << 8 | buf[0];
	regval_nV = div_s64(regval_nV * params[i].reg_to_unit_multiplier,
					params[i].reg_to_unit_divider)
			- params[i].reg_to_unit_offset;

	offset_nV = chip->offset_nV;
	gain = chip->v_gain_mega;

	comp_val_nV = div_s64(regval_nV * gain, 1000000) + offset_nV;
	comp_val_uV = div_s64(comp_val_nV, 1000);

	return snprintf(ubuf, PAGE_SIZE, "%d\n", comp_val_uV);
}

static ssize_t voltage_store(struct class *c, struct class_attribute *attr,
		       const char *ubuf, size_t count)
{
	int i = attr - qnovo_attributes;
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
	u8 buf[2] = {0, 0};
	int rc;
	unsigned long val_uV;
	s64 regval_nV;
	s64 gain, offset_nV;

	if (kstrtoul(ubuf, 0, &val_uV))
		return -EINVAL;

	if (val_uV < params[i].min_val || val_uV > params[i].max_val) {
		pr_err("Out of Range %d%s for %s\n", (int)val_uV,
				params[i].units_str,
				params[i].name);
		return -ERANGE;
	}

	offset_nV = chip->offset_nV;
	gain = chip->v_gain_mega;

	regval_nV = (s64)val_uV * 1000 - offset_nV;
	regval_nV = div_s64(regval_nV * 1000000, gain);

	regval_nV = div_s64((regval_nV + params[i].reg_to_unit_offset)
			* params[i].reg_to_unit_divider,
			params[i].reg_to_unit_multiplier);
	buf[0] = regval_nV & 0xFF;
	buf[1] = ((u64)regval_nV >> 8) & 0xFF;

	rc = qnovo_write(chip, params[i].start_addr, buf, params[i].num_regs);
	if (rc < 0) {
		pr_err("Couldn't write %s rc = %d\n", params[i].name, rc);
		return -EINVAL;
	}

	return count;
}

static ssize_t coulomb_show(struct class *c, struct class_attribute *attr,
				char *ubuf)
{
	int i = attr - qnovo_attributes;
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
	u8 buf[2] = {0, 0};
	int rc;
	int comp_val_uC;
	s64 regval_uC, gain;

	rc = qnovo_read(chip, params[i].start_addr, buf, params[i].num_regs);
	if (rc < 0) {
		pr_err("Couldn't read %s rc = %d\n", params[i].name, rc);
		return -EINVAL;
	}
	regval_uC = buf[1] << 8 | buf[0];
	regval_uC = div_s64(regval_uC * params[i].reg_to_unit_multiplier,
					params[i].reg_to_unit_divider)
			- params[i].reg_to_unit_offset;

	if (chip->dt.external_rsense)
		gain = chip->external_i_gain_mega;
	else
		gain = chip->internal_i_gain_mega;

	comp_val_uC = div_s64(regval_uC * gain, 1000000);
	return snprintf(ubuf, PAGE_SIZE, "%d\n", comp_val_uC);
}

static ssize_t coulomb_store(struct class *c, struct class_attribute *attr,
		       const char *ubuf, size_t count)
{
	int i = attr - qnovo_attributes;
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
	u8 buf[2] = {0, 0};
	int rc;
	unsigned long val_uC;
	s64 regval;
	s64 gain;

	if (kstrtoul(ubuf, 0, &val_uC))
		return -EINVAL;

	if (val_uC < params[i].min_val || val_uC > params[i].max_val) {
		pr_err("Out of Range %d%s for %s\n", (int)val_uC,
				params[i].units_str,
				params[i].name);
		return -ERANGE;
	}

	if (chip->dt.external_rsense)
		gain = chip->external_i_gain_mega;
	else
		gain = chip->internal_i_gain_mega;

	regval = div_s64((s64)val_uC * 1000000, gain);

	regval = div_s64((regval + params[i].reg_to_unit_offset)
			* params[i].reg_to_unit_divider,
			params[i].reg_to_unit_multiplier);

	buf[0] = regval & 0xFF;
	buf[1] = ((u64)regval >> 8) & 0xFF;

	rc = qnovo_write(chip, params[i].start_addr, buf, params[i].num_regs);
	if (rc < 0) {
		pr_err("Couldn't write %s rc = %d\n", params[i].name, rc);
		return -EINVAL;
	}

	return count;
}

static ssize_t batt_prop_show(struct class *c, struct class_attribute *attr,
				char *ubuf)
{
	int i = attr - qnovo_attributes;
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
	int rc = -EINVAL;
	int prop = params[i].start_addr;
	union power_supply_propval pval = {0};

	if (!is_batt_available(chip))
		return -EINVAL;

	rc = power_supply_get_property(chip->batt_psy, prop, &pval);
	if (rc < 0) {
		pr_err("Couldn't read battery prop %s rc = %d\n",
				params[i].name, rc);
		return -EINVAL;
	}

	return snprintf(ubuf, PAGE_SIZE, "%d\n", pval.intval);
}

static struct class_attribute qnovo_attributes[] = {
	[VER]			= __ATTR_RO(version),
	[OK_TO_QNOVO]		= __ATTR_RO(ok_to_qnovo),
	[QNOVO_ENABLE]		= __ATTR_RW(qnovo_enable),
	[PT_ENABLE]		= __ATTR_RW(pt_enable),
	[FV_REQUEST]		= __ATTR(fv_uV_request, 0644,
					val_show, val_store),
	[FCC_REQUEST]		= __ATTR(fcc_uA_request, 0644,
					val_show, val_store),
	[PE_CTRL_REG]		= __ATTR(PE_CTRL_REG, 0644,
					reg_show, reg_store),
	[PE_CTRL2_REG]		= __ATTR(PE_CTRL2_REG, 0644,
					reg_show, reg_store),
	[PTRAIN_STS_REG]	= __ATTR(PTRAIN_STS_REG, 0444,
					reg_show, NULL),
	[INT_RT_STS_REG]	= __ATTR(INT_RT_STS_REG, 0444,
					reg_show, NULL),
	[ERR_STS2_REG]		= __ATTR(ERR_STS2_REG, 0444,
					reg_show, NULL),
	[PREST1]		= __ATTR(PREST1_mS, 0644,
					time_show, time_store),
	[PPULS1]		= __ATTR(PPULS1_uC, 0644,
					coulomb_show, coulomb_store),
	[NREST1]		= __ATTR(NREST1_mS, 0644,
					time_show, time_store),
	[NPULS1]		= __ATTR(NPULS1_mS, 0644,
					time_show, time_store),
	[PPCNT]			= __ATTR(PPCNT, 0644,
					time_show, time_store),
	[VLIM1]			= __ATTR(VLIM1_uV, 0644,
					voltage_show, voltage_store),
	[PVOLT1]		= __ATTR(PVOLT1_uV, 0444,
					voltage_show, NULL),
	[PCUR1]			= __ATTR(PCUR1_uA, 0444,
					current_show, NULL),
	[PTTIME]		= __ATTR(PTTIME_S, 0444,
					time_show, NULL),
	[PREST2]		= __ATTR(PREST2_mS, 0644,
					time_show, time_store),
	[PPULS2]		= __ATTR(PPULS2_uC, 0644,
					coulomb_show, coulomb_store),
	[NREST2]		= __ATTR(NREST2_mS, 0644,
					time_show, time_store),
	[NPULS2]		= __ATTR(NPULS2_mS, 0644,
					time_show, time_store),
	[VLIM2]			= __ATTR(VLIM2_uV, 0644,
					voltage_show, voltage_store),
	[PVOLT2]		= __ATTR(PVOLT2_uV, 0444,
					voltage_show, NULL),
	[RVOLT2]		= __ATTR(RVOLT2_uV, 0444,
					voltage_show, NULL),
	[PCUR2]			= __ATTR(PCUR2_uA, 0444,
					current_show, NULL),
	[SCNT]			= __ATTR(SCNT, 0644,
					time_show, time_store),
	[VMAX]			= __ATTR(VMAX_uV, 0444,
					voltage_show, NULL),
	[SNUM]			= __ATTR(SNUM, 0444,
					time_show, NULL),
	[VBATT]			= __ATTR(VBATT_uV, 0444,
					batt_prop_show, NULL),
	[IBATT]			= __ATTR(IBATT_uA, 0444,
					batt_prop_show, NULL),
	[BATTTEMP]		= __ATTR(BATTTEMP_deciDegC, 0444,
					batt_prop_show, NULL),
	[BATTSOC]		= __ATTR(BATTSOC, 0444,
					batt_prop_show, NULL),
	__ATTR_NULL,
};

#ifdef CONFIG_LGE_PM
static void qnovo_update_status(struct qnovo *chip)
#else
static int qnovo_update_status(struct qnovo *chip)
#endif
{
	u8 val = 0;
	int rc;
#ifdef CONFIG_LGE_PM
	bool err_status;
#else
	bool ok_to_qnovo;
	bool changed = false;
#endif

	rc = qnovo_read(chip, QNOVO_ERROR_STS2, &val, 1);
	if (rc < 0) {
		pr_info("qni: <><> Couldn't read error_sts2 rc = %d\n", rc);
#ifdef CONFIG_LGE_PM
		err_status = true;
#else
		ok_to_qnovo = false;
#endif
	} else {
		/*
		 * For CV mode keep qnovo enabled, userspace is expected to
		 * disable it after few runs
		 */
#ifdef CONFIG_LGE_PM
		err_status = (val == ERR_CV_MODE || val == 0) ? false : true;

		if (is_dc_present(chip)) {
			err_status = true;
			pr_info("qni:%s dc is presented. qnovo is disabled.\n",	__func__);
		}
#else
		ok_to_qnovo = (val == ERR_CV_MODE || val == 0) ? true : false;
#endif
	}

#ifdef CONFIG_LGE_PM
	if (val == ERR_CHARGING_DISABLED) {
		msleep(100);
		rc = qnovo_read(chip, QNOVO_ERROR_STS2, &val, 1);
		if (rc < 0) {
			pr_info("qni: <><> Couldn't read error_sts2 rc = %d\n", rc);
			err_status = true;
		} else {
			err_status = (val == ERR_CV_MODE || val == 0) ? false : true;
		}
		pr_info("qni:%s ERROR_STS2=0x02 -> 0x%X (error_state=%d)\n",
						__func__, val, err_status);
	}

	vote(chip->disable_votable, QNOVO_ERR_VOTER, err_status, 0);
#else
	if (chip->ok_to_qnovo ^ ok_to_qnovo) {

		vote(chip->disable_votable, OK_TO_QNOVO_VOTER, !ok_to_qnovo, 0);
		if (!ok_to_qnovo)
			vote(chip->disable_votable, USER_VOTER, true, 0);

		chip->ok_to_qnovo = ok_to_qnovo;
		changed = true;
	}

	return changed;
#endif
}

static void status_change_work(struct work_struct *work)
{
	struct qnovo *chip = container_of(work,
			struct qnovo, status_change_work);

#ifdef CONFIG_LGE_PM
	if (is_fg_available(chip))
		vote(chip->disable_votable, FG_AVAILABLE_VOTER, false, 0);
#endif

#ifdef CONFIG_LGE_PM
	qnovo_update_status(chip);
#else
	if (qnovo_update_status(chip))
		kobject_uevent(&chip->dev->kobj, KOBJ_CHANGE);
#endif
}

#ifdef CONFIG_LGE_PM
#define POLLING_INTEVAL_MS	500
#define INPUT_PB_TIME_S		15
static void input_probation_work(struct work_struct *work)
{
	struct qnovo *chip = container_of(work,
			struct qnovo, input_probation_work);
	int t;

	pr_info("qni:%s: >>> enter (usb=%d, dc=%d)\n",
			__func__, chip->usb_present, chip->dc_present);
	vote(chip->qnovo_awake_votable, INPUT_PB_VOTER, true, 0);
	chip->input_probation_wip = true;
	vote(chip->disable_votable, INPUT_PB_VOTER, true, 0);
	vote(chip->disable_votable, USER_VOTER, false, 0);

pb_restart:
	t = 0;
	chip->input_changed = false;

	do {
		msleep(POLLING_INTEVAL_MS);
		if (chip->input_changed) {
			pr_info("qni:%s: <><><> rerun (usb=%d, dc=%d)\n",
					__func__, chip->usb_present, chip->dc_present);
			goto pb_restart;
		}
	} while (t++ < INPUT_PB_TIME_S*1000/POLLING_INTEVAL_MS);

	if (is_usb_present(chip))
		vote(chip->disable_votable, INPUT_PB_VOTER, false, 0);

	chip->input_probation_wip = false;
	vote(chip->qnovo_awake_votable, INPUT_PB_VOTER, false, 0);

	pr_info("qni:%s: <<< exit (usb=%d, dc=%d)\n",
			__func__, chip->usb_present, chip->dc_present);
}

static void ptrain_restart_work(struct work_struct *work)
{
	struct qnovo *chip = container_of(work, struct qnovo,
					ptrain_restart_work);
	u8 pt_t1, pt_t2;
	u8 pt_en;
	int i, rc;

	chip->pt_restart_flag = true;

	pr_info("qni:PT_RESTART detection >>> enter.\n");
	rc = qnovo_read(chip, QNOVO_PTRAIN_EN, &pt_en, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read QNOVO_PTRAIN_EN rc = %d\n",
			rc);
		goto clean_up;
	}

	if (!pt_en) {
		pr_info("qni:PT_RESTART pt_enable not enabled.\n");
		rc = qnovo_masked_write(chip, QNOVO_PTRAIN_EN,
					QNOVO_PTRAIN_EN_BIT,
					QNOVO_PTRAIN_EN_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't enable pulse train rc=%d\n",
					rc);
			goto clean_up;
		}
		pr_info("qni:PT_RESTART pt_enable re-enabled.\n");
	}

	msleep(20);

	rc = qnovo_read(chip, QNOVO_PTTIME_STS, &pt_t1, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read QNOVO_PTTIME_STS rc = %d\n",
			rc);
		goto clean_up;
	}

	/* pttime increments every 2 seconds */
	i = 0;
	do {
		msleep(50);
		rc = qnovo_read(chip, QNOVO_PTRAIN_EN, &pt_en, 1);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't read QNOVO_PTRAIN_EN rc = %d\n", rc);
			goto clean_up;
}
		i++;
	} while (pt_en && i < 50);

	/* ptraing not running */
	if (!pt_en) {
		pr_info("qni:PT_RESTART pt_done.\n");
		goto clean_up;
	}

	rc = qnovo_read(chip, QNOVO_PTTIME_STS, &pt_t2, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read QNOVO_PTTIME_STS rc = %d\n",
			rc);
		goto clean_up;
	}

	pr_info("qni:PT_RESTART pttime1 = %x, pptime2 = %x\n", pt_t1, pt_t2);

	if (pt_t1 != pt_t2)
		goto clean_up;

	pr_info("qni:PT_RESTART stall condition detected\n");

	/* Toggle pt enable to restart pulse train */
	rc = qnovo_masked_write(chip, QNOVO_PTRAIN_EN, QNOVO_PTRAIN_EN_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable pulse train rc=%d\n", rc);
		goto clean_up;
	}
	msleep(1000);
	rc = qnovo_masked_write(chip, QNOVO_PTRAIN_EN, QNOVO_PTRAIN_EN_BIT,
				QNOVO_PTRAIN_EN_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't enable pulse train rc=%d\n", rc);
		goto clean_up;
	}
	pr_info("qni:PT_RESTART pt restarted\n");

clean_up:
	vote(chip->qnovo_awake_votable, PT_RESTART_VOTER, false, 0);
	chip->pt_restart_flag = false;
	pr_info("qni:PT_RESTART detection <<< exit.\n");
}
#endif

static int qnovo_notifier_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct power_supply *psy = v;
	struct qnovo *chip = container_of(nb, struct qnovo, nb);

	if (ev != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

#ifdef CONFIG_LGE_PM
	if (strcmp(psy->desc->name, "battery") == 0
		|| strcmp(psy->desc->name, "bms") == 0)
#else
	if (strcmp(psy->desc->name, "battery") == 0)
#endif
		schedule_work(&chip->status_change_work);

#ifdef CONFIG_LGE_PM
	if (is_usb_available(chip) && is_dc_available(chip))
		update_input_present(chip);

	if (chip->input_changed
		&& (chip->usb_present || chip->dc_present)
		&& !chip->input_probation_wip)
		schedule_work(&chip->input_probation_work);
#endif

	return NOTIFY_OK;
}

static irqreturn_t handle_ptrain_done(int irq, void *data)
{
	struct qnovo *chip = data;
#ifdef CONFIG_LGE_PM
	int rc;
	u8 pt_en;
	union power_supply_propval pval = {0};

	rc = qnovo_read(chip, QNOVO_PTRAIN_EN, &pt_en, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read QNOVO_PTRAIN_EN rc = %d\n",
			rc);
	}

	pr_info("qni:%s ok_to_qnovo=%d, pt_en=%u\n",
			__func__, chip->ok_to_qnovo, pt_en);

	qnovo_update_status(chip);
	if (is_fg_available(chip) && chip->ok_to_qnovo && !pt_en)
		power_supply_set_property(chip->bms_psy,
				POWER_SUPPLY_PROP_RESISTANCE,
				&pval);
#else
	qnovo_update_status(chip);
#endif
	kobject_uevent(&chip->dev->kobj, KOBJ_CHANGE);
	return IRQ_HANDLED;
}

static int qnovo_hw_init(struct qnovo *chip)
{
	int rc;
	u8 iadc_offset_external, iadc_offset_internal;
	u8 iadc_gain_external, iadc_gain_internal;
	u8 vadc_offset, vadc_gain;
	u8 val;

#ifdef CONFIG_LGE_PM
	vote(chip->disable_votable, FG_AVAILABLE_VOTER, true, 0);
#else
	vote(chip->disable_votable, USER_VOTER, true, 0);
#endif

	val = 0;
	rc = qnovo_write(chip, QNOVO_STRM_CTRL, &val, 1);
	if (rc < 0) {
		pr_err("Couldn't write iadc bitstream control rc = %d\n", rc);
		return rc;
	}

	rc = qnovo_read(chip, QNOVO_IADC_OFFSET_0, &iadc_offset_external, 1);
	if (rc < 0) {
		pr_err("Couldn't read iadc exernal offset rc = %d\n", rc);
		return rc;
	}

	/* stored as an 8 bit 2's complement signed integer */
	val = -1 * iadc_offset_external;
	rc = qnovo_write(chip, QNOVO_TR_IADC_OFFSET_0, &val, 1);
	if (rc < 0) {
		pr_err("Couldn't write iadc offset rc = %d\n", rc);
		return rc;
	}

	rc = qnovo_read(chip, QNOVO_IADC_OFFSET_1, &iadc_offset_internal, 1);
	if (rc < 0) {
		pr_err("Couldn't read iadc internal offset rc = %d\n", rc);
		return rc;
	}

	/* stored as an 8 bit 2's complement signed integer */
	val = -1 * iadc_offset_internal;
	rc = qnovo_write(chip, QNOVO_TR_IADC_OFFSET_1, &val, 1);
	if (rc < 0) {
		pr_err("Couldn't write iadc offset rc = %d\n", rc);
		return rc;
	}

	rc = qnovo_read(chip, QNOVO_IADC_GAIN_0, &iadc_gain_external, 1);
	if (rc < 0) {
		pr_err("Couldn't read iadc external gain rc = %d\n", rc);
		return rc;
	}

	rc = qnovo_read(chip, QNOVO_IADC_GAIN_1, &iadc_gain_internal, 1);
	if (rc < 0) {
		pr_err("Couldn't read iadc internal gain rc = %d\n", rc);
		return rc;
	}

	rc = qnovo_read(chip, QNOVO_VADC_OFFSET, &vadc_offset, 1);
	if (rc < 0) {
		pr_err("Couldn't read vadc offset rc = %d\n", rc);
		return rc;
	}

	rc = qnovo_read(chip, QNOVO_VADC_GAIN, &vadc_gain, 1);
	if (rc < 0) {
		pr_err("Couldn't read vadc external gain rc = %d\n", rc);
		return rc;
	}

	chip->external_offset_nA = (s64)(s8)iadc_offset_external * IADC_LSB_NA;
	chip->internal_offset_nA = (s64)(s8)iadc_offset_internal * IADC_LSB_NA;
	chip->offset_nV = (s64)(s8)vadc_offset * VADC_LSB_NA;
	chip->external_i_gain_mega
		= 1000000000 + (s64)(s8)iadc_gain_external * GAIN_LSB_FACTOR;
	chip->external_i_gain_mega
		= div_s64(chip->external_i_gain_mega, 1000);
	chip->internal_i_gain_mega
		= 1000000000 + (s64)(s8)iadc_gain_internal * GAIN_LSB_FACTOR;
	chip->internal_i_gain_mega
		= div_s64(chip->internal_i_gain_mega, 1000);
	chip->v_gain_mega = 1000000000 + (s64)(s8)vadc_gain * GAIN_LSB_FACTOR;
	chip->v_gain_mega = div_s64(chip->v_gain_mega, 1000);

	/* allow charger error conditions to disable qnovo, CV mode excluded */
	val = ERR_SWITCHER_DISABLED | ERR_JEITA_SOFT_CONDITION | ERR_BAT_OV |
		ERR_BATTERY_MISSING | ERR_SAFETY_TIMER_EXPIRED |
		ERR_CHARGING_DISABLED | ERR_JEITA_HARD_CONDITION;
	rc = qnovo_write(chip, QNOVO_DISABLE_CHARGING, &val, 1);
	if (rc < 0) {
		pr_err("Couldn't write QNOVO_DISABLE_CHARGING rc = %d\n", rc);
		return rc;
	}

	return 0;
}

static int qnovo_register_notifier(struct qnovo *chip)
{
	int rc;

	chip->nb.notifier_call = qnovo_notifier_call;
	rc = power_supply_reg_notifier(&chip->nb);
	if (rc < 0) {
		pr_err("Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	return 0;
}

static int qnovo_determine_initial_status(struct qnovo *chip)
{
	status_change_work(&chip->status_change_work);
	return 0;
}

static int qnovo_request_interrupts(struct qnovo *chip)
{
	int rc = 0;
	int irq_ptrain_done = of_irq_get_byname(chip->dev->of_node,
						"ptrain-done");

	rc = devm_request_threaded_irq(chip->dev, irq_ptrain_done, NULL,
					handle_ptrain_done,
					IRQF_ONESHOT, "ptrain-done", chip);
	if (rc < 0) {
		pr_err("Couldn't request irq %d rc = %d\n",
					irq_ptrain_done, rc);
		return rc;
	}

	enable_irq_wake(irq_ptrain_done);

	return rc;
}

#ifdef CONFIG_LGE_PM
static int qnovo_awake_cb(struct votable *votable, void *data, int awake,
			const char *client)
{
	struct qnovo *chip = data;

	if (awake)
		pm_stay_awake(chip->dev);
	else
		pm_relax(chip->dev);

	return 0;
}
#endif

static int qnovo_probe(struct platform_device *pdev)
{
	struct qnovo *chip;
	int rc = 0;
#ifdef CONFIG_LGE_PM_BATT_MANAGER
	struct power_supply_config qnovo_psy_cfg;
#endif
	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->fv_uV_request = -EINVAL;
	chip->fcc_uA_request = -EINVAL;
	chip->dev = &pdev->dev;
	mutex_init(&chip->write_lock);

	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap) {
		pr_err("parent regmap is missing\n");
		return -EINVAL;
	}

	rc = qnovo_parse_dt(chip);
	if (rc < 0) {
		pr_err("Couldn't parse device tree rc=%d\n", rc);
		return rc;
	}

	/* set driver data before resources request it */
	platform_set_drvdata(pdev, chip);
#ifdef CONFIG_LGE_PM_BATT_MANAGER
	qnovo_psy_cfg.drv_data = chip;
	qnovo_psy_cfg.of_node = NULL;
	qnovo_psy_cfg.supplied_to = NULL;
	qnovo_psy_cfg.num_supplicants = 0;
	chip->qnovo_psy = power_supply_register(chip->dev, &qnovo_desc, &qnovo_psy_cfg);
#endif
	chip->disable_votable = create_votable("QNOVO_DISABLE", VOTE_SET_ANY,
#ifdef CONFIG_LGE_PM
					qnovo_ok_disable_cb, chip);
#else
					qnovo_disable_cb, chip);
#endif
	if (IS_ERR(chip->disable_votable)) {
		rc = PTR_ERR(chip->disable_votable);
		goto cleanup;
	}

#ifdef CONFIG_LGE_PM
	chip->qnovo_awake_votable = create_votable("QNOVO_WS", VOTE_SET_ANY,
					qnovo_awake_cb, chip);
	if (IS_ERR(chip->qnovo_awake_votable)) {
		rc = PTR_ERR(chip->qnovo_awake_votable);
		goto cleanup;
	}
#endif

	INIT_WORK(&chip->status_change_work, status_change_work);
#ifdef CONFIG_LGE_PM
	INIT_WORK(&chip->input_probation_work, input_probation_work);
	INIT_WORK(&chip->ptrain_restart_work, ptrain_restart_work);
#endif

	rc = qnovo_hw_init(chip);
	if (rc < 0) {
		pr_err("Couldn't initialize hardware rc=%d\n", rc);
		goto destroy_votable;
	}

	rc = qnovo_register_notifier(chip);
	if (rc < 0) {
		pr_err("Couldn't register psy notifier rc = %d\n", rc);
		goto unreg_notifier;
	}

	rc = qnovo_determine_initial_status(chip);
	if (rc < 0) {
		pr_err("Couldn't determine initial status rc=%d\n", rc);
		goto unreg_notifier;
	}

	rc = qnovo_request_interrupts(chip);
	if (rc < 0) {
		pr_err("Couldn't request interrupts rc=%d\n", rc);
		goto unreg_notifier;
	}
	chip->qnovo_class.name = "qnovo",
	chip->qnovo_class.owner = THIS_MODULE,
	chip->qnovo_class.class_attrs = qnovo_attributes;

	rc = class_register(&chip->qnovo_class);
	if (rc < 0) {
		pr_err("couldn't register qnovo sysfs class rc = %d\n", rc);
		goto unreg_notifier;
	}

	device_init_wakeup(chip->dev, true);

	return rc;

unreg_notifier:
	power_supply_unreg_notifier(&chip->nb);
destroy_votable:
	destroy_votable(chip->disable_votable);
#ifdef CONFIG_LGE_PM
	destroy_votable(chip->qnovo_awake_votable);
#endif
cleanup:
	platform_set_drvdata(pdev, NULL);
	return rc;
}

static int qnovo_remove(struct platform_device *pdev)
{
	struct qnovo *chip = platform_get_drvdata(pdev);

	class_unregister(&chip->qnovo_class);
#ifdef CONFIG_LGE_PM_BATT_MANAGER
	power_supply_unregister(chip->qnovo_psy);
#endif
	power_supply_unreg_notifier(&chip->nb);
	destroy_votable(chip->disable_votable);
#ifdef CONFIG_LGE_PM
	destroy_votable(chip->qnovo_awake_votable);
#endif
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id match_table[] = {
	{ .compatible = "qcom,qpnp-qnovo", },
	{ },
};

static struct platform_driver qnovo_driver = {
	.driver		= {
		.name		= "qcom,qnovo-driver",
		.owner		= THIS_MODULE,
		.of_match_table	= match_table,
	},
	.probe		= qnovo_probe,
	.remove		= qnovo_remove,
};
module_platform_driver(qnovo_driver);

MODULE_DESCRIPTION("QPNP Qnovo Driver");
MODULE_LICENSE("GPL v2");
