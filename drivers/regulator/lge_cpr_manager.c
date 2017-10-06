/*
 * drivers/soc/qcom/lge/lge_cpr_manager.c
 *
 * Copyright (C) 2016 LG Electronics, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include "cpr3-regulator.h"

#define APC0_VOL_OFFSET    0x6
#define APC1_VOL_OFFSET    0x7
#define GFX_VOL_OFFSET  0x8
#define APC0_VOL_OFFSET_ADDR  (msm_imem_cpr_margin_base + APC0_VOL_OFFSET)
#define APC1_VOL_OFFSET_ADDR  (msm_imem_cpr_margin_base + APC1_VOL_OFFSET)
#define GFX_VOL_OFFSET_ADDR  (msm_imem_cpr_margin_base + GFX_VOL_OFFSET)

static void *msm_imem_cpr_margin_base;
static s8 apc0_vol_offset;
static s8 apc1_vol_offset;
static s8 gfx_vol_offset;

s8 get_lg_cpr_margins(const char *name)
{
        if(!strncmp(name,"apc0",4))
                return apc0_vol_offset;
        if(!strncmp(name,"apc1",4))
                return apc1_vol_offset;
        if(!strncmp(name,"gfx",3))
                return gfx_vol_offset;
        return 0;
}

static int __init lge_cpr_manager_init(void)
{
        struct device_node *np;

        pr_info("lge_cpr_manager_init Start\n");
        np = of_find_compatible_node(NULL, NULL, "lge,cpr_offsets");
        if (!np) {
                pr_err("unable to find CPR Margin imem node\n");
                return -ENODEV;
        }
        msm_imem_cpr_margin_base = of_iomap(np, 0);
        if (!msm_imem_cpr_margin_base) {
                pr_err("unable to map imem\n");
                return -ENOMEM;
        }
        apc0_vol_offset = (s8)readb_relaxed(APC0_VOL_OFFSET_ADDR);
        apc1_vol_offset = (s8)readb_relaxed(APC1_VOL_OFFSET_ADDR);
        gfx_vol_offset = (s8)readb_relaxed(GFX_VOL_OFFSET_ADDR);
        pr_info("Voltage offsets APC0:%d APC1:%d\n",apc0_vol_offset,apc1_vol_offset);
        pr_info("Voltage offset GFX:%d \n",gfx_vol_offset);
        return 0;
}

static void __exit lg_cpr_manager_exit(void)
{

}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CPR VOLTAGE MANAGER");

early_initcall(lge_cpr_manager_init);
module_exit(lg_cpr_manager_exit);
