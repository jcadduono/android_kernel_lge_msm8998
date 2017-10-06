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

#ifndef _LGE_INTERNAL_TEST_H
#define _LGE_INTERNAL_TEST_H

void mdss_dsi_parse_internal_dcs_cmds(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_internal_test_create_sysfs(struct class *panel);
#endif
