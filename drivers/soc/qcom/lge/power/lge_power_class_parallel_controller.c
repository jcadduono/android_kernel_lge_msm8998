/* Copyright (c) 2013-2016, LG Eletronics. All rights reserved.
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

#define pr_fmt(fmt) "[LGE-PL-CTRL] %s : " fmt, __func__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <soc/qcom/lge/power/lge_power_class.h>


#define MODULE_NAME "lge_parallel_controller"

struct lge_parallel_controller {
	struct device 			*dev;
	struct lge_power 		lge_pl_ctrl_lpc;
	struct power_supply		*batt_psy;
	struct power_supply		*usb_psy;
	int total_ibat_ua;
	int total_iusb_ua;
	int master_ibat_ua;
	int master_iusb_ua;
	int slave_ibat_ua;
	int slave_iusb_ua;
	bool is_parallel_charging_enabled;
	int slave_pct;
	int ibat_threshold;
};

static enum lge_power_property lge_power_lge_pl_ctrl_properties[] = {
	LGE_POWER_PROP_CURRENT_MAX,
	LGE_POWER_PROP_CHARGING_CURRENT_MAX,
	LGE_POWER_PROP_MASTER_CURRENT_MAX,
	LGE_POWER_PROP_MASTER_CONSTANT_CHARGE_CURRENT_MAX,
	LGE_POWER_PROP_SLAVE_CURRENT_MAX,
	LGE_POWER_PROP_SLAVE_CONSTANT_CHARGE_CURRENT_MAX,
	LGE_POWER_PROP_PARALLEL_CHARGING_ENABLED,
};

static char *lge_pl_ctrl_supplied_from[] = {
	"battery",
	"usb",
};

static struct lge_parallel_controller *the_pl_ctrl;

static int lge_power_lge_pl_ctrl_property_is_writeable(struct lge_power *lpc,
		enum lge_power_property lpp) {
	int ret = 0;
	switch (lpp) {
		case LGE_POWER_PROP_CURRENT_MAX:
		case LGE_POWER_PROP_CHARGING_CURRENT_MAX:
			ret = 1;
			break;
		default:
			break;
	}
	return ret;
}

static int lge_power_lge_pl_ctrl_set_property(struct lge_power *lpc,
		enum lge_power_property lpp,
		const union lge_power_propval *val) {
	int ret_val = 0;
	struct lge_parallel_controller *pl_ctrl
		= container_of(lpc,	struct lge_parallel_controller,
				lge_pl_ctrl_lpc);

	switch (lpp) {
		case LGE_POWER_PROP_CHARGING_CURRENT_MAX:
			pl_ctrl->total_ibat_ua = val->intval;
			break;
		case LGE_POWER_PROP_CURRENT_MAX:
			pl_ctrl->total_iusb_ua = val->intval;
			break;
		default:
			pr_err("lpp:%d is not supported!!!\n", lpp);
			ret_val = -EINVAL;
			break;
	}
	lge_power_changed(&pl_ctrl->lge_pl_ctrl_lpc);

	return ret_val;
}

static int lge_power_lge_pl_ctrl_get_property(struct lge_power *lpc,
		enum lge_power_property lpp,
		union lge_power_propval *val) {
	int ret_val = 0;

	struct lge_parallel_controller *pl_ctrl
		= container_of(lpc, struct lge_parallel_controller,
				lge_pl_ctrl_lpc);
	switch (lpp) {
		case LGE_POWER_PROP_MASTER_CONSTANT_CHARGE_CURRENT_MAX:
			val->intval = pl_ctrl->master_ibat_ua;
			break;
		case LGE_POWER_PROP_MASTER_CURRENT_MAX:
			val->intval = pl_ctrl->master_iusb_ua;
			break;
		case LGE_POWER_PROP_SLAVE_CONSTANT_CHARGE_CURRENT_MAX:
			val->intval = pl_ctrl->slave_ibat_ua / 1000;
			break;
		case LGE_POWER_PROP_SLAVE_CURRENT_MAX:
			val->intval = pl_ctrl->slave_iusb_ua / 1000;
			break;
		case LGE_POWER_PROP_PARALLEL_CHARGING_ENABLED:
			val->intval = pl_ctrl->is_parallel_charging_enabled;
			break;
		default:
			ret_val = -EINVAL;
			break;
	}

	return ret_val;
}

static void lge_pl_ctrl_split_current(struct lge_parallel_controller *pl_ctrl, int total_ua,
			     int *master_ua, int *slave_ua) {
	if (pl_ctrl->is_parallel_charging_enabled) {
		*slave_ua = (total_ua * pl_ctrl->slave_pct) / 100;
		*master_ua = max(0, total_ua - *slave_ua);
	} else {
		*slave_ua = 0;
		*master_ua = total_ua;
	}
}

static bool lge_pl_ctrl_is_parallel_charging(struct lge_parallel_controller *pl_ctrl, int charge_type, enum power_supply_type usb_supply_type, int total_ibat) {
	if (charge_type == POWER_SUPPLY_CHARGE_TYPE_FAST
		&& usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP
		&& pl_ctrl->ibat_threshold <= total_ibat) {
		return true;
	} else {
		return false;
	}
}

static void lge_pl_ctrl_update_parallel_status(struct lge_parallel_controller *pl_ctrl) {

	lge_pl_ctrl_split_current(pl_ctrl,pl_ctrl->total_ibat_ua,&pl_ctrl->master_ibat_ua, &pl_ctrl->slave_ibat_ua);
	lge_pl_ctrl_split_current(pl_ctrl,pl_ctrl->total_iusb_ua,&pl_ctrl->master_iusb_ua, &pl_ctrl->slave_iusb_ua);
	pr_info("Current split total_ibat_ua : %d, master_ibat_ua : %d, slave_ibat_ua : %d \n",
		pl_ctrl->total_ibat_ua, pl_ctrl->master_ibat_ua, pl_ctrl->slave_ibat_ua);
	pr_info("Current split total_iusb_ua : %d, master_iusb_ua : %d, slave_iusb_ua : %d \n",
		pl_ctrl->total_iusb_ua, pl_ctrl->master_iusb_ua, pl_ctrl->slave_iusb_ua);
	lge_power_changed(&pl_ctrl->lge_pl_ctrl_lpc);
}

static void lge_pl_ctrl_external_power_changed(struct lge_power *lpc) {
	struct lge_parallel_controller *pl_ctrl
		= container_of(lpc, struct lge_parallel_controller,	lge_pl_ctrl_lpc);

	union power_supply_propval ret = {0,};
	int charge_type;
	int total_ibat;
	int total_iusb;
	enum power_supply_type usb_supply_type;

	pr_info("start~~~\n");
	if (pl_ctrl->batt_psy == NULL) {
		pl_ctrl->batt_psy = power_supply_get_by_name("battery");
		if(!pl_ctrl->batt_psy) {
			pr_err("battery not found deferring probe\n");
			return;
		}
	}
	pl_ctrl->batt_psy->desc->get_property(pl_ctrl->batt_psy,
			POWER_SUPPLY_PROP_CHARGE_TYPE, &ret);
	charge_type = ret.intval;

	pl_ctrl->batt_psy->desc->get_property(pl_ctrl->batt_psy,
			POWER_SUPPLY_PROP_FCC_MAX, &ret);
	total_ibat = ret.intval;

	if (pl_ctrl->usb_psy == NULL) {
		pl_ctrl->usb_psy = power_supply_get_by_name("usb");
		if(!pl_ctrl->usb_psy) {
			pr_err("USB not found deferring probe\n");
			return;
		}
	}

	pl_ctrl->usb_psy->desc->get_property(pl_ctrl->usb_psy,
			POWER_SUPPLY_PROP_TYPE, &ret);
	usb_supply_type = ret.intval;

	pl_ctrl->usb_psy->desc->get_property(pl_ctrl->usb_psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED, &ret);
	total_iusb = ret.intval;

	pr_info("Condition : charge_type : %d, total_ibat : %d, total_iusb : %d \n",charge_type, total_ibat, total_iusb);

	if(pl_ctrl->is_parallel_charging_enabled) {
		if (lge_pl_ctrl_is_parallel_charging(pl_ctrl, charge_type, usb_supply_type, total_ibat)) {
			if (pl_ctrl->total_iusb_ua != total_iusb && pl_ctrl->total_ibat_ua != total_ibat) {
				pl_ctrl->total_iusb_ua = total_iusb;
				pl_ctrl->total_ibat_ua = total_ibat;
				lge_pl_ctrl_update_parallel_status(pl_ctrl);
			}
		} else {
			pr_info("Stop parallel charging. charge_type : %d, total_ibat : %d, total_iusb : %d \n",charge_type, total_ibat, total_iusb);
			pl_ctrl->is_parallel_charging_enabled= false;
			pl_ctrl->total_iusb_ua = total_iusb;
			pl_ctrl->total_ibat_ua = total_ibat;
			lge_pl_ctrl_update_parallel_status(pl_ctrl);
		}
	} else {
		if (lge_pl_ctrl_is_parallel_charging(pl_ctrl, charge_type, usb_supply_type, total_ibat)) {
			pr_info("Start parallel charging.\n");
			pl_ctrl->is_parallel_charging_enabled= true;
			pl_ctrl->total_iusb_ua = total_iusb;
			pl_ctrl->total_ibat_ua = total_ibat;
			lge_pl_ctrl_update_parallel_status(pl_ctrl);
		}
	}
}

static int lge_parallel_controller_probe(struct platform_device *pdev) {
	struct lge_parallel_controller *pl_ctrl;
	struct lge_power *lge_power_pl_ctrl;
	int ret;

	pr_info("LG Current spliter probe start~!!\n");

	pl_ctrl = kzalloc(sizeof(struct lge_parallel_controller),
								GFP_KERNEL);
	if (!pl_ctrl) {
		pr_err("lge_parallel_controller memory alloc failed.\n");
		return -ENOMEM;
	}

	pl_ctrl->dev = &pdev->dev;
	the_pl_ctrl = pl_ctrl;

	lge_power_pl_ctrl = &pl_ctrl->lge_pl_ctrl_lpc;
	lge_power_pl_ctrl->name = "lge_pl_ctrl";
	lge_power_pl_ctrl->properties = lge_power_lge_pl_ctrl_properties;
	lge_power_pl_ctrl->num_properties =
		ARRAY_SIZE(lge_power_lge_pl_ctrl_properties);
	lge_power_pl_ctrl->get_property = lge_power_lge_pl_ctrl_get_property;
	lge_power_pl_ctrl->set_property = lge_power_lge_pl_ctrl_set_property;
	lge_power_pl_ctrl->property_is_writeable =
		lge_power_lge_pl_ctrl_property_is_writeable;
	lge_power_pl_ctrl->supplied_from = lge_pl_ctrl_supplied_from;
	lge_power_pl_ctrl->num_supplies	= ARRAY_SIZE(lge_pl_ctrl_supplied_from);
	lge_power_pl_ctrl->external_power_changed
			= lge_pl_ctrl_external_power_changed;

	ret = lge_power_register(pl_ctrl->dev, lge_power_pl_ctrl);
	if (ret < 0) {
		pr_err("Failed to register lge power class: %d\n", ret);
		goto err_free;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "lge,slave_pct",
									   &pl_ctrl->slave_pct);
	if (ret < 0) {
		pl_ctrl->slave_pct = 50;
		pr_err("Failed to get lge,slave_pct: %d\n", ret);
	}

	ret = of_property_read_u32(pdev->dev.of_node, "lge,ibat_threshold",
									   &pl_ctrl->ibat_threshold);
	if (ret < 0) {
		pl_ctrl->ibat_threshold = 1000000;
		pr_err("Failed to get lge,ibat_threshold: %d\n", ret);
	}

	pr_info("LG Current spliter probe done~!!\n");

	return 0;

err_free:
	kfree(pl_ctrl);
	return ret;
}

#ifdef CONFIG_OF
static struct of_device_id lge_parallel_controller_match_table[] = {
	{.compatible = "lge,parallel_controller"},
	{ },
};
#endif

static int lge_parallel_controller_remove(struct platform_device *pdev) {
	lge_power_unregister(&the_pl_ctrl->lge_pl_ctrl_lpc);

	kfree(the_pl_ctrl);
	return 0;
}

static struct platform_driver lge_parallel_controller_driver = {
	.probe = lge_parallel_controller_probe,
	.remove = lge_parallel_controller_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = lge_parallel_controller_match_table,
#endif
	},
};

static int __init lge_parallel_controller_init(void) {
	return platform_driver_register(&lge_parallel_controller_driver);
}

static void __exit lge_parallel_controller_exit(void) {
	platform_driver_unregister(&lge_parallel_controller_driver);
}

module_init(lge_parallel_controller_init);
module_exit(lge_parallel_controller_exit);

MODULE_DESCRIPTION("LGE Parallel Controller driver");
MODULE_LICENSE("GPL v2");
