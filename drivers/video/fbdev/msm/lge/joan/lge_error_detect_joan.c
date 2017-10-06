/*
 * Copyright(c) 2017, LG Electronics. All rights reserved.
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

#define pr_fmt(fmt)     "[Display] %s: " fmt, __func__

#include "../lge_mdss_dsi.h"
#include "../lge_error_detect.h"

extern struct mdss_panel_data *pdata_base;

char lge_err_dcs_cmd[2] = {0x9F, 0x00}; /* DTYPE_DCS_READ */
struct dsi_cmd_desc lge_err_dcs_read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(lge_err_dcs_cmd)},
	lge_err_dcs_cmd
};

static void sw43402_err_work(struct work_struct *work)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct delayed_work *dw = to_delayed_work(work);

	struct dcs_cmd_req cmdreq;
	char rx_buf[1] = {0x0};

	ctrl_pdata = container_of(dw, struct mdss_dsi_ctrl_pdata, err_int_work);
	if (!ctrl_pdata) {
		pr_err("invalid ctrl data\n");
		return;
	}

	if (pdata_base->panel_info.panel_power_state == 0) {
		pr_err("panel is off\n");
		return;
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &lge_err_dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 1;
	cmdreq.cb = NULL;
	cmdreq.rbuf = rx_buf;

	if (ctrl_pdata->status_cmds.link_state == DSI_LP_MODE)
		cmdreq.flags |= CMD_REQ_LP_MODE;
	else if (ctrl_pdata->status_cmds.link_state == DSI_HS_MODE)
		cmdreq.flags |= CMD_REQ_HS_MODE;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);
	ctrl_pdata->err_result = rx_buf[0];
	pr_info("Reg[0x%x]=0x%x,\n", lge_err_dcs_cmd[0], rx_buf[0]);
	if (ctrl_pdata->err_crash && ctrl_pdata->err_result) {
		pr_err("error is detected. BUG() \n");
		BUG();
	}
}

static irqreturn_t sw43402_err_irq_handler(int irq, void *data)
{
	struct mdss_dsi_ctrl_pdata *pdata = (struct mdss_dsi_ctrl_pdata *)data;

	pr_info("\n");
	queue_delayed_work(pdata->err_int_workq, &pdata->err_int_work,
		msecs_to_jiffies(50));
	return IRQ_HANDLED;
}

void lge_dsi_err_detect_init_sub(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct lge_mdss_dsi_ctrl_pdata *lge_ctrl_pdata = NULL;
	if (ctrl_pdata == NULL) {
		pr_err("invalid input\n");
		return;
	}

	lge_ctrl_pdata = ctrl_pdata->lge_ctrl_pdata;

	lge_ctrl_pdata->err_work = sw43402_err_work;
	lge_ctrl_pdata->err_irq_handler = sw43402_err_irq_handler;
}
