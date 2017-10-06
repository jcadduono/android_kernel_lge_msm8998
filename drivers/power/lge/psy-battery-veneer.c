/*
 * LGE Power class
 *
 * Copyright (C) 2016 LG Electronics, Inc
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#define DEBUG
#define pr_fmt(fmt) "LBV: %s: " fmt, __func__
#define pr_lbv(fmt, ...) pr_info(fmt, ##__VA_ARGS__)

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>

#include "inc-limit-voter.h"

#define	EXTERNAL_CHANGED_PRESENT_USB		BIT(1)
#define	EXTERNAL_CHANGED_PRESENT_WIRELESS	BIT(2)
#define	EXTERNAL_CHANGED_PRESENT_BATTERY	BIT(3)

#define LBV_COMPATIBLE	"lge,battery-veneer"
#define LBV_DRIVER		"lge-battery-veneer"
#define LBV_NAME		"battery-veneer"

struct lge_battery_veneer {
	/* module descripters */
	struct device* module_dev;
	struct power_supply module_psy;

	/* shadow states */
	// present or not
	bool present_usb;
	bool present_wireless;
	bool present_battery;

	// limited charging/inputing current values by LGE scenario (unit in mA).
	int limited_iusb;
	int limited_ibat;
	int limited_idc;
};

static int limit_input_current(const union power_supply_propval *propval) {
	struct power_supply* psy_batt = power_supply_get_by_name("battery");

	if (psy_batt && psy_batt->set_property) {
		pr_lbv("bypass '%d' to psy battery\n", propval->intval);
		return psy_batt->set_property(psy_batt,
				POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, propval);
	} else {
		pr_lbv("psy battery is not ready\n");
		return -ENXIO;
	}
}

static char* psy_external_suppliers[] = { "battery", "usb", "usb-parallel",
		"usb-pd", "ac", "dc", "bms", };
static void psy_external_changed(struct power_supply *external_supplier);
static enum power_supply_property psy_property_list[] = {
		POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, };
static const char* psy_property_name(enum power_supply_property prop);
static int psy_property_set(struct power_supply *psy,
		enum power_supply_property prop, const union power_supply_propval *val);
static int psy_property_get(struct power_supply *psy,
		enum power_supply_property prop, union power_supply_propval *val);
static int psy_property_writeable(struct power_supply *psy,
		enum power_supply_property prop);

static const char* psy_property_name(enum power_supply_property prop) {
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return "POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT";
	default:
		return "NOT_SUPPORTED_PROP";
	}
}

static int psy_property_set(struct power_supply *psy,
		enum power_supply_property prop, const union power_supply_propval *val) {

	int rc = 0;
	struct lge_battery_veneer *lbv = container_of(psy,
			struct lge_battery_veneer, module_psy);

	pr_lbv("setting property %s\n", psy_property_name(prop));

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT: {
		rc = limit_input_current(val);

		if (!rc) {
			int limited_current = vote_current(val);

			switch (vote_type(val)) {
			case LIMIT_VOTER_IUSB:
				lbv->limited_iusb = limited_current;
				break;
			case LIMIT_VOTER_IBAT:
				lbv->limited_ibat = limited_current;
				break;
			case LIMIT_VOTER_IDC:
				lbv->limited_idc = limited_current;
				break;
			default:
				rc = -EINVAL;
			}
		}
	}
		break;

	default:
		rc = -EINVAL;
	}

	return rc;
}

static int psy_property_get(struct power_supply *psy,
		enum power_supply_property prop, union power_supply_propval *val) {

	int rc = 0;
	struct lge_battery_veneer *lbv = container_of(psy,
			struct lge_battery_veneer, module_psy);

	pr_lbv("getting property %s, intval = %d\n", psy_property_name(prop), val->intval);

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		dump_stack();

		switch (vote_type(val)) {
		case LIMIT_VOTER_IUSB:
			val->intval += lbv->limited_iusb;
			break;
		case LIMIT_VOTER_IBAT:
			val->intval += lbv->limited_ibat;
			break;
		case LIMIT_VOTER_IDC:
			val->intval += lbv->limited_idc;
			break;
		default:
			val->intval = 0;
		}
		break;

	default:
		rc = -EINVAL;
	}

	return rc;
}

static int psy_property_writeable(struct power_supply *psy,
		enum power_supply_property prop) {
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		rc = 1;
		break;

	default:
		rc = 0;
	}

	return rc;
}

static void psy_external_changed(struct power_supply *psy_me) {
	pr_lbv("who changed?\n");
//	union power_supply_propval value = { 0, };
//	struct power_supply* psy_batt = power_supply_get_by_name("battery");
//	struct power_supply* psy_usb = power_supply_get_by_name("usb");

//	dump_stack();

//	if (psy_batt) {
//		psy_batt->get_property(psy_batt, POWER_SUPPLY_PROP_STATUS, &value);
//		if (value.intval == POWER_SUPPLY_STATUS_FULL)
//			value.intval = 1;
//		else
//			value.intval = 0;
//		psy_me->set_property(psy_me, POWER_SUPPLY_PROP_CHARGE_FULL, &value);
//	}
//
//	if (psy_usb) {
//		psy_usb->get_property(psy_usb, POWER_SUPPLY_PROP_PRESENT, &value);
//		if (!value.intval)
//			value.intval = 1;
//		else
//			value.intval = 0;
//		psy_me->set_property(psy_me, POWER_SUPPLY_PROP_CHARGING_ENABLED,
//				&value);
//	}
}

static int lbv_probe_devicetree(struct device_node *dev_node,
		struct lge_battery_veneer *lbv) {

	int ret = 0;

//	chip->gpio_activated = of_get_named_gpio(dev_node, "idt,gpio_activated", 0);
//	if (chip->gpio_activated < 0) {
//		pr_err("Fail to get gpio_activated\n");
//		goto out;
//	} else {
//		pr_lbv("Get gpio_activated : %d\n", chip->gpio_activated);
//	}
//
//	chip->gpio_disabling = of_get_named_gpio(dev_node, "idt,gpio_disabling", 0);
//	if (chip->gpio_disabling < 0) {
//		pr_err("Fail to get gpio_disabling\n");
//		goto out;
//	} else {
//		pr_lbv("Get gpio_disabling : %d\n", chip->gpio_disabling);
//	}
//
//	return 0;

//out:
	return ret;
}

static int lbv_probe_psy(struct device *parent, struct power_supply *psy) {
	int ret = 0;

	psy->name = LBV_NAME;
	psy->type = POWER_SUPPLY_TYPE_BATTERY;
	psy->get_property = psy_property_get;
	psy->set_property = psy_property_set;
	psy->properties = psy_property_list;
	psy->property_is_writeable = psy_property_writeable;
	psy->num_properties = ARRAY_SIZE(psy_property_list);
	psy->supplied_from = psy_external_suppliers;
	psy->num_supplies = ARRAY_SIZE(psy_external_suppliers);
	psy->external_power_changed = psy_external_changed;

	ret = power_supply_register(parent, psy);
	if (ret < 0) {
		dev_err(parent, "Unable to register wlc_psy ret = %d\n", ret);
	}

	return ret;
}

static int lbv_probe(struct platform_device *pdev) {

	struct lge_battery_veneer *lbv;
	struct device_node *dev_node = pdev->dev.of_node;
	int ret;

	pr_lbv("Start\n");

	lbv = kzalloc(sizeof(struct lge_battery_veneer), GFP_KERNEL);
	if (!lbv) {
		pr_err("Failed to alloc memory\n");
		goto error;
	} else
		lbv->module_dev = &pdev->dev;

	dev_node = pdev->dev.of_node;
	if (dev_node) {
		ret = lbv_probe_devicetree(dev_node, lbv);
		if (ret < 0) {
			pr_err("Fail to read parse_dt\n");
			goto err_hw_init;
		}
	}

	ret = lbv_probe_psy(lbv->module_dev, &lbv->module_psy);
	if (ret < 0) {
		pr_err("Fail to request gpio at probe\n");
		goto err_hw_init;
	}

	platform_set_drvdata(pdev, lbv);
	pr_lbv("Complete probing LBV\n");
	return 0;

err_hw_init:
error:
	kfree(lbv);
	pr_err("Failed to probe\n");

	return ret;
}

static int lbv_remove(struct platform_device *pdev) {
	struct lge_battery_veneer *lbv = platform_get_drvdata(pdev);

	power_supply_unregister(&lbv->module_psy);
	platform_set_drvdata(pdev, NULL);
	kfree(lbv);
	return 0;
}

static const struct of_device_id lbv_devid_of[] = {
		{ .compatible = LBV_COMPATIBLE },
		{ },
};

static const struct platform_device_id lbv_devid_platform[] = {
		{ LBV_DRIVER, 0 },
		{ },
};

static struct platform_driver lbv_module_driver = {
		.driver = {
			.name = LBV_DRIVER,
			.owner = THIS_MODULE,
			.of_match_table = lbv_devid_of,
		},
		.probe = lbv_probe,
		.remove = lbv_remove,
		.id_table = lbv_devid_platform,
};

static int __init lbv_module_init(void) {
	return platform_driver_register(&lbv_module_driver);
}

static void __exit lbv_module_exit(void) {
	platform_driver_unregister(&lbv_module_driver);
}

module_init(lbv_module_init);
module_exit(lbv_module_exit);

MODULE_DESCRIPTION(LBV_DRIVER);
MODULE_LICENSE("GPL v2");
