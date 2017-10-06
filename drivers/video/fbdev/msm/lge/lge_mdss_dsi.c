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

#include <linux/of_gpio.h>
#include "../mdss_dsi.h"
#include "lge_mdss_dsi.h"
#include <soc/qcom/lge/board_lge.h>

static int lge_mdss_dsi_parse_gpio_params(struct platform_device *ctrl_pdev,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc, i;
	const char *name;
	char buf[256];
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;

	rc = of_property_count_strings(ctrl_pdev->dev.of_node,
					"lge,extra-gpio-names");
	if (rc > 0) {
		lge_ctrl_pdata->num_gpios = rc;
		pr_info("%s: num_gpios=%d\n", __func__,
				lge_ctrl_pdata->num_gpios);
		lge_ctrl_pdata->gpio_array = kmalloc(sizeof(struct lge_gpio_entry) *
				lge_ctrl_pdata->num_gpios, GFP_KERNEL);
		if (NULL == lge_ctrl_pdata->gpio_array) {
			pr_err("%s: no memory\n", __func__);
			lge_ctrl_pdata->num_gpios = 0;
			return -ENOMEM;
		}
		for (i = 0; i < lge_ctrl_pdata->num_gpios; ++i) {
			of_property_read_string_index(ctrl_pdev->dev.of_node,
					"lge,extra-gpio-names", i, &name);
			strlcpy(lge_ctrl_pdata->gpio_array[i].name, name,
			     sizeof(lge_ctrl_pdata->gpio_array[i].name));
			snprintf(buf, sizeof(buf), "lge,gpio-%s", name);
			lge_ctrl_pdata->gpio_array[i].gpio =
			      of_get_named_gpio(ctrl_pdev->dev.of_node, buf, 0);
			if (!gpio_is_valid(
				lge_ctrl_pdata->gpio_array[i].gpio))
				pr_err("%s: %s not specified\n", __func__, buf);
		}
	} else {
		lge_ctrl_pdata->num_gpios = 0;
		pr_info("%s: no lge specified gpio\n", __func__);
	}
	return 0;
}

int lge_mdss_dsi_parse_extra_params(struct platform_device *ctrl_pdev,
        struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	lge_mdss_dsi_parse_gpio_params(ctrl_pdev, ctrl_pdata);

	return 0;
}

void lge_extra_gpio_set_value(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
			const char *name, int value)
{
	int i, index = -1;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;

	for (i = 0; i < lge_ctrl_pdata->num_gpios; ++i) {
		if (!strcmp(lge_ctrl_pdata->gpio_array[i].name, name)) {
			index = i;
			break;
		}
	}

	if (index != -1) {
		gpio_set_value(lge_ctrl_pdata->gpio_array[index].gpio,
				value);
	} else {
		pr_err("%s: couldn't get gpio by name %s\n", __func__, name);
	}
}

int lge_get_extra_gpio(struct mdss_dsi_ctrl_pdata *ctrl_pdata, const char* name)
{
	int i;
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;
	for (i = 0; i < lge_ctrl_pdata->num_gpios; ++i) {
		if (!strcmp(lge_ctrl_pdata->gpio_array[i].name, name)) {
			return lge_ctrl_pdata->gpio_array[i].gpio;
		}
	}
	return -EINVAL;
}
int detect_factory_cable(void)
{
	int factory_cable = 0;

	switch (lge_get_boot_mode()) {
		case LGE_BOOT_MODE_QEM_56K:
		case LGE_BOOT_MODE_QEM_130K:
		case LGE_BOOT_MODE_QEM_910K:
		case LGE_BOOT_MODE_PIF_56K:
		case LGE_BOOT_MODE_PIF_130K:
		case LGE_BOOT_MODE_PIF_910K:
			factory_cable = 1;
			break;
		default:
			break;
	}

	return factory_cable;
}

int detect_qem_factory_cable(void)
{
	int factory_cable = 0;

	switch (lge_get_boot_mode()) {
		case LGE_BOOT_MODE_QEM_56K:
		case LGE_BOOT_MODE_QEM_130K:
		case LGE_BOOT_MODE_QEM_910K:
			factory_cable = 1;
			break;
		default:
			break;
	}

	return factory_cable;
}
