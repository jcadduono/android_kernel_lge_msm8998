/*
 * BQ2589x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include "bq25898s_reg.h"

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PARALLEL_CONTROLLER
#include <linux/notifier.h>
#include <soc/qcom/lge/power/lge_power_class.h>
#endif

#define MONITOR_WORK_TIME_SEC    15

enum bq2589x_part_no {
	BQ25898  = 0x00,
	BQ25898S = 0x01,
	BQ25898D = 0x02,
};

struct bq2589x_config {
	bool	enable_auto_dpdm;

	int		charge_voltage;
	int		charge_current;

	int		iindpm_threshold;
	int		vindpm_threshold;

	bool	enable_term;
	int		term_current;

	bool	use_absolute_vindpm;

	int     watchdog_time;
};

struct bq2589x {
	struct device *dev;
	struct i2c_client *client;
	enum   bq2589x_part_no part_no;
	int    revision;

	bool	prechg;
	struct	bq2589x_config	cfg;
	struct 	work_struct irq_work;
	struct 	delayed_work monitor_work;

	int 	rsoc;
	struct 	power_supply *batt_psy;
	struct  power_supply *bq_chg_psy;

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PARALLEL_CONTROLLER
	struct  notifier_block nb;
	struct  work_struct pl_ctrl_work;
	struct  lge_power *lge_pl_lpc;
#endif
};

static struct bq2589x *g_bq;

static DEFINE_MUTEX(bq2589x_i2c_lock);

static int bq2589x_read_byte(struct bq2589x *bq, u8 *data, u8 reg)
{
	int ret;

	mutex_lock(&bq2589x_i2c_lock);
	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		dev_err(bq->dev, "failed to read 0x%.2x\n", reg);
		mutex_unlock(&bq2589x_i2c_lock);
		return ret;
	}

	*data = (u8)ret;
	mutex_unlock(&bq2589x_i2c_lock);

	return 0;
}

static int bq2589x_write_byte(struct bq2589x *bq, u8 reg, u8 data)
{
	int ret;
	mutex_lock(&bq2589x_i2c_lock);
	ret = i2c_smbus_write_byte_data(bq->client, reg, data);
	mutex_unlock(&bq2589x_i2c_lock);
	return ret;
}

static int bq2589x_update_bits(struct bq2589x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = bq2589x_read_byte(bq, &tmp, reg);

	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;

	return bq2589x_write_byte(bq, reg, tmp);
}

static int bq2589x_enable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ25898S_CHG_ENABLE << BQ25898S_CHG_CONFIG_SHIFT;
	dev_info(bq->dev, "%s: start\n", __func__);
	ret = bq2589x_update_bits(bq, BQ25898S_REG_03, BQ25898S_CHG_CONFIG_MASK, val);
	dev_info(bq->dev, "%s: end\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_charger);

static int bq2589x_disable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ25898S_CHG_DISABLE << BQ25898S_CHG_CONFIG_SHIFT;
	dev_info(bq->dev, "%s: start\n", __func__);
	ret = bq2589x_update_bits(bq, BQ25898S_REG_03, BQ25898S_CHG_CONFIG_MASK, val);
	dev_info(bq->dev, "%s: end\n", __func__);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_disable_charger);

static int bq2589x_enable_term(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;
	dev_info(bq->dev, "%s: start\n", __func__);
	if (enable)
		val = BQ25898S_TERM_ENABLE << BQ25898S_EN_TERM_SHIFT;
	else
		val = BQ25898S_TERM_DISABLE << BQ25898S_EN_TERM_SHIFT;

	dev_info(bq->dev, "%s: enable:%d\n", __func__,enable);
	ret = bq2589x_update_bits(bq, BQ25898S_REG_07, BQ25898S_EN_TERM_MASK, val);
	dev_info(bq->dev, "%s: end\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_term);

int bq2589x_adc_start(struct bq2589x *bq, bool oneshot)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_02);
	if (ret < 0) {
		dev_err(bq->dev, "%s failed to read register 0x02:%d\n", __func__, ret);
		return ret;
	}

	if (((val & BQ25898S_CONV_RATE_MASK) >> BQ25898S_CONV_RATE_SHIFT) == BQ25898S_ADC_CONTINUE_ENABLE)
		return 0; /*is doing continuous scan*/
	if (oneshot)
		ret = bq2589x_update_bits(bq, BQ25898S_REG_02, BQ25898S_CONV_START_MASK, BQ25898S_CONV_START << BQ25898S_CONV_START_SHIFT);
	else
		ret = bq2589x_update_bits(bq, BQ25898S_REG_02, BQ25898S_CONV_RATE_MASK,  BQ25898S_ADC_CONTINUE_ENABLE << BQ25898S_CONV_RATE_SHIFT);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_adc_start);

int bq2589x_adc_stop(struct bq2589x *bq)
{
	return bq2589x_update_bits(bq, BQ25898S_REG_02, BQ25898S_CONV_RATE_MASK, BQ25898S_ADC_CONTINUE_DISABLE << BQ25898S_CONV_RATE_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_adc_stop);

int bq2589x_adc_read_battery_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_0E);
	if (ret < 0) {
		dev_err(bq->dev, "read battery voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ25898S_BATV_BASE + ((val & BQ25898S_BATV_MASK) >> BQ25898S_BATV_SHIFT) * BQ25898S_BATV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_battery_volt);

int bq2589x_adc_read_sys_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_0F);
	if (ret < 0) {
		dev_err(bq->dev, "read system voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ25898S_SYSV_BASE + ((val & BQ25898S_SYSV_MASK) >> BQ25898S_SYSV_SHIFT) * BQ25898S_SYSV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_sys_volt);

int bq2589x_adc_read_vbus_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_11);
	if (ret < 0) {
		dev_err(bq->dev, "read vbus voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ25898S_VBUSV_BASE + ((val & BQ25898S_VBUSV_MASK) >> BQ25898S_VBUSV_SHIFT) * BQ25898S_VBUSV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_vbus_volt);

int bq2589x_adc_read_charge_current(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_12);
	if (ret < 0) {
		dev_err(bq->dev, "read charge current failed :%d\n", ret);
		return ret;
	} else{
		volt = (int)(BQ25898S_ICHGR_BASE + ((val & BQ25898S_ICHGR_MASK) >> BQ25898S_ICHGR_SHIFT) * BQ25898S_ICHGR_LSB) ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_charge_current);

int bq2589x_get_chargecurrent(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_04);
	if (ret < 0) {
		dev_err(bq->dev, "read charge current failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ25898S_ICHGR_BASE + ((val & BQ25898S_ICHG_MASK) >> BQ25898S_ICHG_SHIFT) * BQ25898S_ICHG_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_get_chargecurrent);

int bq2589x_set_chargecurrent(struct bq2589x *bq, int curr)
{
	u8 ichg;

	ichg = (curr - BQ25898S_ICHG_BASE)/BQ25898S_ICHG_LSB;
	return bq2589x_update_bits(bq, BQ25898S_REG_04, BQ25898S_ICHG_MASK, ichg << BQ25898S_ICHG_SHIFT);

}
EXPORT_SYMBOL_GPL(bq2589x_set_chargecurrent);

int bq2589x_set_term_current(struct bq2589x *bq, int curr)
{
	u8 iterm;

	iterm = (curr - BQ25898S_ITERM_BASE) / BQ25898S_ITERM_LSB;

	return bq2589x_update_bits(bq, BQ25898S_REG_05, BQ25898S_ITERM_MASK, iterm << BQ25898S_ITERM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_term_current);

int bq2589x_set_prechg_current(struct bq2589x *bq, int curr)
{
	u8 iprechg;

	iprechg = (curr - BQ25898S_IPRECHG_BASE) / BQ25898S_IPRECHG_LSB;

	return bq2589x_update_bits(bq, BQ25898S_REG_05, BQ25898S_IPRECHG_MASK, iprechg << BQ25898S_IPRECHG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_prechg_current);

int bq2589x_set_chargevoltage(struct bq2589x *bq, int volt)
{
	u8 val;

	val = (volt - BQ25898S_VREG_BASE)/BQ25898S_VREG_LSB;
	return bq2589x_update_bits(bq, BQ25898S_REG_06, BQ25898S_VREG_MASK, val << BQ25898S_VREG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_chargevoltage);


int bq2589x_set_input_volt_limit(struct bq2589x *bq, int volt)
{
	u8 val;
	val = (volt - BQ25898S_VINDPM_BASE) / BQ25898S_VINDPM_LSB;
	return bq2589x_update_bits(bq, BQ25898S_REG_0D, BQ25898S_VINDPM_MASK, val << BQ25898S_VINDPM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_volt_limit);

int bq2589x_set_input_current_limit(struct bq2589x *bq, int curr)
{
	u8 val;

	val = (curr - BQ25898S_IINLIM_BASE) / BQ25898S_IINLIM_LSB;
	return bq2589x_update_bits(bq, BQ25898S_REG_00, BQ25898S_IINLIM_MASK, val << BQ25898S_IINLIM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_current_limit);

int bq2589x_get_input_current_limit(struct bq2589x *bq)
{
	uint8_t val;
	int curr;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_00);
	if (ret < 0) {
		dev_err(bq->dev, "read charge current failed :%d\n", ret);
		return ret;
	} else{
		curr = BQ25898S_IINLIM_BASE + ((val & BQ25898S_IINLIM_MASK) >> BQ25898S_IINLIM_SHIFT) * BQ25898S_IINLIM_LSB;
		return curr;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_get_input_current_limit);

int bq2589x_set_vindpm_offset(struct bq2589x *bq, int offset)
{
	u8 val;

	if (offset == 400)
		val = BQ25898S_VINDPMOS_400MV;
	else
		val = BQ25898S_VINDPMOS_600MV;

	return bq2589x_update_bits(bq, BQ25898S_REG_01, BQ25898S_VINDPMOS_MASK, val << BQ25898S_VINDPMOS_SHIFT);

}
EXPORT_SYMBOL_GPL(bq2589x_set_vindpm_offset);

int bq2589x_get_charging_status(struct bq2589x *bq)
{
	u8 val = 0;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_0B);
	if (ret < 0) {
		dev_err(bq->dev, "%s Failed to read register 0x0b:%d\n", __func__, ret);
		return ret;
	}
	val &= BQ25898S_CHRG_STAT_MASK;
	val >>= BQ25898S_CHRG_STAT_SHIFT;
	return val;
}
EXPORT_SYMBOL_GPL(bq2589x_get_charging_status);

int bq2589x_set_watchdog_timer(struct bq2589x *bq, u8 timeout)
{
	return bq2589x_update_bits(bq, BQ25898S_REG_07, BQ25898S_WDT_MASK, (u8)((timeout - BQ25898S_WDT_BASE) / BQ25898S_WDT_LSB) << BQ25898S_WDT_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_watchdog_timer);

int bq2589x_disable_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ25898S_WDT_DISABLE << BQ25898S_WDT_SHIFT;

	return bq2589x_update_bits(bq, BQ25898S_REG_07, BQ25898S_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_disable_watchdog_timer);

int bq2589x_reset_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ25898S_WDT_RESET << BQ25898S_WDT_RESET_SHIFT;

	return bq2589x_update_bits(bq, BQ25898S_REG_03, BQ25898S_WDT_RESET_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_reset_watchdog_timer);

int bq2589x_reset_chip(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ25898S_RESET << BQ25898S_RESET_SHIFT;

	ret = bq2589x_update_bits(bq, BQ25898S_REG_14, BQ25898S_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_reset_chip);

int bq2589x_enter_hiz_mode(struct bq2589x *bq)
{
	u8 val = BQ25898S_HIZ_ENABLE << BQ25898S_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ25898S_REG_00, BQ25898S_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_enter_hiz_mode);

int bq2589x_exit_hiz_mode(struct bq2589x *bq)
{

	u8 val = BQ25898S_HIZ_DISABLE << BQ25898S_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ25898S_REG_00, BQ25898S_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_exit_hiz_mode);

int bq2589x_get_hiz_mode(struct bq2589x *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_00);
	if (ret)
		return ret;
	*state = (val & BQ25898S_ENHIZ_MASK) >> BQ25898S_ENHIZ_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_get_hiz_mode);


static int bq2589x_enable_auto_dpdm(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;
	dev_info(bq->dev, "%s: start\n", __func__);

	if (enable)
		val = BQ25898S_AUTO_DPDM_ENABLE << BQ25898S_AUTO_DPDM_EN_SHIFT;
	else
		val = BQ25898S_AUTO_DPDM_DISABLE << BQ25898S_AUTO_DPDM_EN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ25898S_REG_02, BQ25898S_AUTO_DPDM_EN_MASK, val);
	dev_info(bq->dev, "%s: end\n", __func__);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enable_auto_dpdm);

static int bq2589x_set_absolute_vindpm(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ25898S_FORCE_VINDPM_ENABLE << BQ25898S_FORCE_VINDPM_SHIFT;
	else
		val = BQ25898S_FORCE_VINDPM_DISABLE << BQ25898S_FORCE_VINDPM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ25898S_REG_0D, BQ25898S_FORCE_VINDPM_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_set_absolute_vindpm);

static int bq2589x_read_idpm_limit(struct bq2589x *bq)
{
	uint8_t val;
	int curr;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_13);
	if (ret < 0) {
		dev_err(bq->dev, "read idpm limit failed :%d\n", ret);
		return ret;
	} else{
		curr = BQ25898S_IDPM_LIM_BASE + ((val & BQ25898S_IDPM_LIM_MASK) >> BQ25898S_IDPM_LIM_SHIFT) * BQ25898S_IDPM_LIM_LSB ;
		return curr;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_read_idpm_limit);

static bool bq2589x_is_charge_done(struct bq2589x *bq)
{
	int ret;
	u8 val;
	dev_info(bq->dev, "%s: start\n", __func__);
	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_0B);
	if (ret < 0) {
		dev_err(bq->dev, "%s:read REG0B failed :%d\n", __func__, ret);
		return false;
	}
	val &= BQ25898S_CHRG_STAT_MASK;
	val >>= BQ25898S_CHRG_STAT_SHIFT;
	dev_info(bq->dev, "%s: end\n", __func__);
	return (val == BQ25898S_CHRG_STAT_CHGDONE);
}
EXPORT_SYMBOL_GPL(bq2589x_is_charge_done);


static int bq2589x_init_device(struct bq2589x *bq)
{
	int ret;


	ret = bq2589x_disable_watchdog_timer(bq);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to disable watchdog timer:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_enable_auto_dpdm(bq, bq->cfg.enable_auto_dpdm); /* always disable autodpdm when acting as slave */
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to disable auto dpdm:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_enable_term(bq, bq->cfg.enable_term);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to enable/disable termination:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_absolute_vindpm(bq, bq->cfg.use_absolute_vindpm);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to enable absolute vindpm:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_disable_charger(bq);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to disable charger:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_adc_start(bq, false);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to start ADC:%d\n", __func__, ret);
	}

	return ret;
}


static ssize_t bq2589x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret ;
	dev_info(dev, "%s: start\n", __func__);
	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "Charger");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(g_bq, &val, addr);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,"Reg[0x%.2x] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}
	dev_info(dev, "%s: end\n", __func__);
	return idx;
}


static DEVICE_ATTR(registers, S_IRUGO, bq2589x_show_registers, NULL);

static struct attribute *bq2589x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2589x_attr_group = {
	.attrs = bq2589x_attributes,
};


static int bq2589x_parse_dt(struct device *dev, struct bq2589x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;

	bq->cfg.enable_auto_dpdm = of_property_read_bool(np, "ti,bq2589x,enable-auto-dpdm");
	bq->cfg.enable_term = of_property_read_bool(np, "ti,bq2589x,enable-termination");
	bq->cfg.use_absolute_vindpm = of_property_read_bool(np, "ti,bq2589x,use-absolute-vindpm");

	ret = of_property_read_u32(np, "ti,bq2589x,charge-voltage",&bq->cfg.charge_voltage);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,charge-current",&bq->cfg.charge_current);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,term-current",&bq->cfg.term_current);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,input-current-limit",&bq->cfg.iindpm_threshold);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,input-voltage-limit",&bq->cfg.vindpm_threshold);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,watchdog-time",&bq->cfg.watchdog_time);
	dev_info(bq->dev, "%s: watchdog timer bit set to %d\n", __func__, bq->cfg.watchdog_time);
	if (ret)
		return ret;

	return 0;
}

static int bq2589x_detect_device(struct bq2589x *bq)
{
	int ret;
	u8 data;
	dev_info(bq->dev, "%s: start\n", __func__);
	ret = bq2589x_read_byte(bq, &data, BQ25898S_REG_14);
	if (ret == 0) {
		bq->part_no = (data & BQ25898S_PN_MASK) >> BQ25898S_PN_SHIFT;
		bq->revision = (data & BQ25898S_DEV_REV_MASK) >> BQ25898S_DEV_REV_SHIFT;
	}
	dev_info(bq->dev, "%s: end\n", __func__);
	return ret;
}

static int bq2589x_read_batt_rsoc(struct bq2589x *bq)
{
	union power_supply_propval ret = {0,};
	dev_info(bq->dev, "%s: start\n", __func__);
	if (!bq->batt_psy)
		bq->batt_psy = power_supply_get_by_name("battery");

	if (bq->batt_psy) {
		bq->batt_psy->desc->get_property(bq->batt_psy,POWER_SUPPLY_PROP_CAPACITY,&ret);
		return ret.intval;
	}
	else {
		return 50;
	}
	dev_info(bq->dev, "%s: end\n", __func__);
}


static int bq2589x_adjust_absolute_vindpm(struct bq2589x *bq)
{
	u16 vbus_volt;
	u16 vindpm_volt;
	int ret;
	dev_info(bq->dev, "%s: start\n", __func__);
	vbus_volt = bq2589x_adc_read_vbus_volt(bq);
	dev_info(bq->dev, "%s: read vbus voltage : %d\n", __func__, vbus_volt);
	if (vbus_volt < 0){
		dev_err(bq->dev, "%s:Failed to read vbus voltage:%d\n", __func__, ret);
		return ret;
	}
	if (vbus_volt < 6000)
		vindpm_volt = vbus_volt - 600;
	else
		vindpm_volt = vbus_volt - 1200;
	ret = bq2589x_set_input_volt_limit(bq, vindpm_volt);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Set absolute vindpm threshold %d Failed:%d\n", __func__, vindpm_volt, ret);
		return ret;
	}
	else
		dev_info(bq->dev, "%s:Set absolute vindpm threshold %d successfully\n", __func__, vindpm_volt);
	dev_info(bq->dev, "%s: end\n", __func__);
	return 0;
}


int bq2589x_set_charge_profile(struct bq2589x *bq)
{
	int ret;
	dev_info(bq->dev, "%s: start\n", __func__);
	ret = bq2589x_set_chargevoltage(bq, bq->cfg.charge_voltage);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set charge voltage:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_chargecurrent(bq, bq->cfg.charge_current);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set charge current:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_term_current(bq, bq->cfg.term_current);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set termination current:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_input_current_limit(bq, bq->cfg.iindpm_threshold);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set input current limit:%d\n", __func__, ret);
		return ret;
	}
	dev_info(bq->dev, "%s: end\n", __func__);

	return 0;
}

void bq2589x_adapter_in_handler(void)
{
	struct bq2589x *bq = g_bq;
	int ret;
	int vbat;

	if (!bq) {
		printk(KERN_ERR "BQ25898S driver not loaded!");
		return;
	}
	dev_info(bq->dev, "%s: start\n", __func__);

	if (bq->cfg.use_absolute_vindpm) {
		bq2589x_adjust_absolute_vindpm(bq);
		if (ret)
			dev_err(bq->dev, "Failed to adjust absoltue vindpm");
	}

	vbat = bq2589x_adc_read_battery_volt(bq);
	if (vbat < 0){
		dev_err(bq->dev, "Failed to read battery voltage");
		return;
	}
	else if (vbat < 3500){
		bq->prechg = true;
		schedule_delayed_work(&bq->monitor_work, MONITOR_WORK_TIME_SEC * HZ);
		return;
	}

	/* check if battery is near full, if so, no need to turn on slave charge */
	bq->rsoc = bq2589x_read_batt_rsoc(bq);
	dev_info(bq->dev, "%s:RSOC=%d\n", __func__, bq->rsoc);
	if (bq->rsoc > 95 ) {
		dev_info(bq->dev, "%s:RSOC=%d, no need start slavce charger\n", __func__, bq->rsoc);
		return;
	}

	ret = bq2589x_enable_charger(bq);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to enable charging:%d\n", __func__, ret);
		return ;
	}
	else {
		dev_info(bq->dev, "%s:slave charge start charging\n", __func__);
	}

	ret = bq2589x_set_watchdog_timer(bq, bq->cfg.watchdog_time);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to enable watchdog timer:%d\n", __func__, ret);
	}
	dev_info(bq->dev, "%s: end\n", __func__);
	schedule_delayed_work(&bq->monitor_work, MONITOR_WORK_TIME_SEC * HZ);
}
EXPORT_SYMBOL_GPL(bq2589x_adapter_in_handler);

void bq2589x_adapter_out_handler(void)
{
	struct bq2589x *bq = g_bq;
	int ret;

	if (!bq) {
		printk(KERN_ERR "BQ25898S driver not loaded!");
		return;
	}
	dev_info(bq->dev, "%s: start\n", __func__);

	/* This changes for initialization */
	ret = bq2589x_set_charge_profile(bq);
	if (ret) {
		dev_err(bq->dev, "%s: set charge profile failed\n", __func__);
	}

	ret = bq2589x_disable_charger(bq);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to disable charger:%d\n", __func__, ret);
	}
	else {
		dev_info(bq->dev, "%s:slave charge stopped\n", __func__);
	}

	ret = bq2589x_disable_watchdog_timer(bq);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to disable watchdog timer:%d\n", __func__, ret);
	}

	cancel_delayed_work_sync(&bq->monitor_work);
	dev_info(bq->dev, "%s: end\n", __func__);
}
EXPORT_SYMBOL_GPL(bq2589x_adapter_out_handler);

static void bq2589x_monitor_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, monitor_work.work);
	u8 status = 0;
	int ret;
	int chg_current,vbus_volt,vbat_volt;
	dev_info(bq->dev, "%s: start\n", __func__);
	if (bq->prechg) {
		vbat_volt = bq2589x_adc_read_battery_volt(bq);

		if (vbat_volt < 0){
			dev_err(bq->dev, "Failed to read battery voltage");
			schedule_delayed_work(&bq->monitor_work, MONITOR_WORK_TIME_SEC * HZ);
			return;
		}
		else if (vbat_volt < 3500) {
			schedule_delayed_work(&bq->monitor_work, MONITOR_WORK_TIME_SEC * HZ);
			return;
		}
		else {
			ret = bq2589x_enable_charger(bq);
			if (ret < 0) {
				dev_err(bq->dev, "%s:Failed to enable charging:%d\n", __func__, ret);
				schedule_delayed_work(&bq->monitor_work, MONITOR_WORK_TIME_SEC * HZ);
				return ;
			}
			else {
				dev_info(bq->dev, "%s:slave charge start charging\n", __func__);
			}

			ret = bq2589x_set_watchdog_timer(bq, bq->cfg.watchdog_time);
			if (ret < 0) {
				dev_err(bq->dev, "%s:Failed to enable watchdog timer:%d\n", __func__, ret);
			}

			bq->prechg = false;
			schedule_delayed_work(&bq->monitor_work, MONITOR_WORK_TIME_SEC * HZ);
			return;
		}
	}
	bq2589x_reset_watchdog_timer(bq);

	vbus_volt = bq2589x_adc_read_vbus_volt(bq);
	vbat_volt = bq2589x_adc_read_battery_volt(bq);
	chg_current = bq2589x_adc_read_charge_current(bq);

	dev_info(bq->dev, "%s:vbus volt:%d,vbat volt:%d,charge current:%d\n", __func__,vbus_volt,vbat_volt,chg_current);

	ret = bq2589x_read_byte(bq, &status, BQ25898S_REG_13);
	if (ret == 0 && (status & BQ25898S_VDPM_STAT_MASK))
		dev_info(bq->dev, "%s:VINDPM occurred\n", __func__);
	if (ret == 0 && (status & BQ25898S_IDPM_STAT_MASK))
		dev_info(bq->dev, "%s:IINDPM occurred\n", __func__);

	schedule_delayed_work(&bq->monitor_work, MONITOR_WORK_TIME_SEC * HZ);
	dev_info(bq->dev, "%s: end\n", __func__);
}

static void bq2589x_charger_irq_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, irq_work);
	u8 status = 0;
	u8 fault = 0;
	u8 charge_status = 0;
	int ret;

	dev_info(bq->dev, "%s: start\n", __func__);
	msleep(5);
	/* Read STATUS and FAULT registers */
	ret = bq2589x_read_byte(bq, &status, BQ25898S_REG_0B);
	if (ret)
		return;

	ret = bq2589x_read_byte(bq, &fault, BQ25898S_REG_0C);
	if (ret)
		return;

	charge_status = (status & BQ25898S_CHRG_STAT_MASK) >> BQ25898S_CHRG_STAT_SHIFT;
	if (charge_status == BQ25898S_CHRG_STAT_IDLE)
		dev_info(bq->dev, "%s:not charging\n", __func__);
	else if (charge_status == BQ25898S_CHRG_STAT_PRECHG)
		dev_info(bq->dev, "%s:precharging\n", __func__);
	else if (charge_status == BQ25898S_CHRG_STAT_FASTCHG)
		dev_info(bq->dev, "%s:fast charging\n", __func__);
	else if (charge_status == BQ25898S_CHRG_STAT_CHGDONE){
		dev_info(bq->dev, "%s:charge done!\n", __func__);
		bq2589x_disable_charger(bq);
	}

	if (fault)
		dev_info(bq->dev, "%s:charge fault:%02x\n", __func__,fault);
	dev_info(bq->dev, "%s: end\n", __func__);


}

static irqreturn_t bq2589x_charger_interrupt(int irq, void *data)
{
	struct bq2589x *bq = data;
	dev_info(bq->dev, "%s: start\n", __func__);
	schedule_work(&bq->irq_work);
	dev_info(bq->dev, "%s: end\n", __func__);
	return IRQ_HANDLED;
}

static enum power_supply_property bq2589x_power_supply_props[] = {
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
};

static int bq2589x_power_supply_get_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct bq2589x *bq = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = bq2589x_get_input_current_limit(bq);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = bq2589x_get_chargecurrent(bq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq2589x_power_supply_set_property(struct power_supply *psy,
					enum power_supply_property prop,
					const union power_supply_propval *val)
{
	struct bq2589x *bq = power_supply_get_drvdata(psy);
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = bq2589x_set_input_current_limit(bq, val->intval);
		dev_info(bq->dev, "%s:bq2589x_set_input_current_limit to %d\n", __func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = bq2589x_set_chargecurrent(bq, val->intval);
		dev_info(bq->dev, "%s:bq2589x_set_chargecurrent to %d\n", __func__, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq2589x_power_supply_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		return true;
	default:
		return false;
	}
}

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PARALLEL_CONTROLLER
static void bq2589x_pl_ctrl_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, pl_ctrl_work);

	int rc;
	union lge_power_propval lge_val = {0,};

	dev_info(bq->dev, "%s: start\n", __func__);

	if (!bq->lge_pl_lpc)
		bq->lge_pl_lpc = lge_power_get_by_name("lge_pl_ctrl");

	if (bq->lge_pl_lpc) {
		rc = bq->lge_pl_lpc->get_property(bq->lge_pl_lpc,
						LGE_POWER_PROP_PARALLEL_CHARGING_ENABLED, &lge_val);
		if (rc < 0) {
			dev_err(bq->dev, "%s: Coudn't get slave chrger enabled condition rc = %d\n", __func__, rc);
			return;
		}
		if (lge_val.intval) {
			rc = bq->lge_pl_lpc->get_property(bq->lge_pl_lpc,
							LGE_POWER_PROP_SLAVE_CURRENT_MAX, &lge_val);
			if (rc < 0) {
				dev_err(bq->dev, "%s: Coudn't get current max value rc = %d\n", __func__, rc);
				return;
			}
			rc = bq2589x_set_input_current_limit(bq, lge_val.intval);
			dev_info(bq->dev, "%s: bq2589x_set_input_current_limit to %dmA\n", __func__, lge_val.intval);

			rc = bq->lge_pl_lpc->get_property(bq->lge_pl_lpc,
							LGE_POWER_PROP_SLAVE_CONSTANT_CHARGE_CURRENT_MAX, &lge_val);
			if (rc < 0) {
				dev_err(bq->dev, "%s: Coudn't get charge current max value rc = %d\n", __func__, rc);
				return;
			}
			rc = bq2589x_set_chargecurrent(bq, lge_val.intval);
			dev_info(bq->dev, "%s: bq2589x_set_chargecurrent to %dmA\n", __func__, lge_val.intval);

			bq2589x_adapter_in_handler();
		}
		else {
			bq2589x_adapter_out_handler();
		}
		dev_info(bq->dev, "%s:bq2589x slave charger %s\n",
			__func__, (bool)lge_val.intval ? "enabled" : "disabled");
	}
	else {
		dev_err(bq->dev, "%s: lge_pl_lpc not defined\n", __func__);
		return;
	}

	dev_info(bq->dev, "%s: end\n", __func__);
}

static int bq2589x_notifier_call(struct notifier_block *nb,
			unsigned long ev, void *v)
{
	struct lge_power *lpc = v;
	struct bq2589x *bq = container_of(nb, struct bq2589x, nb);

	if (!strcmp(lpc->name, "lge_pl_ctrl")) {
		dev_info(bq->dev, "%s: start lge parallel charging control!!\n", __func__);
		schedule_work(&bq->pl_ctrl_work);
	}

	return NOTIFY_OK;
}

static int bq2589x_register_notifier(struct bq2589x *bq)
{
	int rc;

	bq->nb.notifier_call = bq2589x_notifier_call;
	rc = lge_power_reg_notifier(&bq->nb);
	if (rc < 0) {
		dev_err(bq->dev, "%s: Coudn't register psy notifier rc = %d\n", __func__, rc);
		return rc;
	}

	return 0;
}
#endif

static const struct power_supply_desc bq2589x_power_supply_desc = {
	.name = "bq2589x-charger",
	.type = POWER_SUPPLY_TYPE_PARALLEL,
	.properties = bq2589x_power_supply_props,
	.num_properties = ARRAY_SIZE(bq2589x_power_supply_props),
	.get_property = bq2589x_power_supply_get_property,
	.set_property = bq2589x_power_supply_set_property,
	.property_is_writeable = bq2589x_power_supply_property_is_writeable,
};

static int bq2589x_power_supply_init(struct bq2589x *bq)
{
	struct power_supply_config psy_cfg = { .drv_data = bq, };

	bq->bq_chg_psy = devm_power_supply_register(bq->dev,
						 &bq2589x_power_supply_desc,
						 &psy_cfg);

	return PTR_ERR_OR_ZERO(bq->bq_chg_psy);
}

#define GPIO_IRQ    133
static int bq2589x_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bq2589x *bq;
	int irqn;
	int charge_status;
	int vbus_status;
	int ret;
	u8 status = 0;

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2589x), GFP_KERNEL);
	if (!bq) {
		dev_err(&client->dev, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->client = client;
	i2c_set_clientdata(client, bq);

	ret = bq2589x_detect_device(bq);
	dev_info(bq->dev, "%s: bq->part_no:%d\n", __func__, bq->part_no);
	if (!ret && bq->part_no == BQ25898S) {
		dev_info(bq->dev, "%s: charger device bq25898S detected, revision:%d\n", __func__, bq->revision);
	} else {
		dev_info(bq->dev, "%s: no bq25898S charger device found:%d\n", __func__, ret);
		return -ENODEV;
	}

	bq->batt_psy = power_supply_get_by_name("battery");

	g_bq = bq;

	if (client->dev.of_node)
		bq2589x_parse_dt(&client->dev, bq);

	ret = bq2589x_init_device(bq);
	if (ret) {
		dev_err(bq->dev, "device init failure: %d\n", ret);
		goto err_0;
	}

	ret = bq2589x_set_charge_profile(bq);
	if (ret) {
		dev_err(bq->dev, "%s: set charge profile failed\n", __func__);
		goto err_0;
	}

	ret = gpio_request(GPIO_IRQ, "bq2589x irq pin");
	if (ret) {
		dev_err(bq->dev, "%s: %d gpio request failed\n", __func__, GPIO_IRQ);
		goto err_0;
	}
	gpio_direction_input(GPIO_IRQ);

	irqn = gpio_to_irq(GPIO_IRQ);
	if (irqn < 0) {
		dev_err(bq->dev, "%s: %d gpio_to_irq failed\n", __func__, irqn);
		ret = irqn;
		goto err_1;
	}
	client->irq = irqn;


	INIT_WORK(&bq->irq_work, bq2589x_charger_irq_workfunc);
	INIT_DELAYED_WORK(&bq->monitor_work, bq2589x_monitor_workfunc);


	ret = sysfs_create_group(&bq->dev->kobj, &bq2589x_attr_group);
	if (ret) {
		dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);
		goto err_irq;
	}

	ret = request_irq(client->irq, bq2589x_charger_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "bq2589x_charger1_irq", bq);
	if (ret) {
		dev_err(bq->dev, "%s: Request IRQ %d failed: %d\n", __func__, client->irq, ret);
		goto err_irq;
	} else {
		dev_info(bq->dev, "%s: irq = %d\n", __func__, client->irq);
	}

	ret = bq2589x_power_supply_init(bq);
	if (ret < 0) {
		dev_err(bq->dev, "Failed to register power supply\n");
		goto err_irq;
	}

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_PARALLEL_CONTROLLER
	INIT_WORK(&bq->pl_ctrl_work, bq2589x_pl_ctrl_workfunc);
	ret = bq2589x_register_notifier(bq);
	if (ret < 0) {
		dev_err(bq->dev, "Couldn't register notifier ret = %d\n", ret);
		goto err_irq;
	}
#endif

	/* Read STATUS and FAULT registers */
	ret = bq2589x_read_byte(bq, &status, BQ25898S_REG_0B);
	if (ret)
		goto err_irq;
	charge_status = (status & BQ25898S_CHRG_STAT_MASK) >> BQ25898S_CHRG_STAT_SHIFT;
	dev_info(bq->dev, "%s: charge_status - %d\n", __func__, charge_status);

	vbus_status = (status & BQ25898S_VBUS_STAT_MASK) >> BQ25898S_VBUS_STAT_SHIFT;
	dev_info(bq->dev, "%s: vbus_status - %d\n", __func__, vbus_status);

	return 0;

err_irq:
	cancel_work_sync(&bq->irq_work);
	cancel_delayed_work_sync(&bq->monitor_work);
err_1:
	gpio_free(GPIO_IRQ);
err_0:
	g_bq = NULL;
	return ret;
}

static void bq2589x_charger_shutdown(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

	dev_info(bq->dev, "%s: shutdown\n", __func__);

	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);
	cancel_work_sync(&bq->irq_work);
	cancel_delayed_work_sync(&bq->monitor_work);

	free_irq(bq->client->irq, bq);
	gpio_free(GPIO_IRQ);
	g_bq = NULL;
}

static struct of_device_id bq2589x_charger_match_table[] = {
	{.compatible = "ti,bq25898s",},
	{},
};


static const struct i2c_device_id bq2589x_charger_id[] = {
	{ "bq25898s", BQ25898S },
	{},
};

MODULE_DEVICE_TABLE(i2c, bq2589x_charger_id);

static struct i2c_driver bq2589x_charger_driver = {
	.driver		= {
		.name	= "bq25898s",
		.of_match_table = bq2589x_charger_match_table,
	},
	.id_table	= bq2589x_charger_id,

	.probe		= bq2589x_charger_probe,
	.shutdown   = bq2589x_charger_shutdown,
};

module_i2c_driver(bq2589x_charger_driver);

MODULE_DESCRIPTION("TI BQ2589x Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
