
#ifndef __BQ25898S_HEADER__
#define __BQ25898S_HEADER__

/* Register 00h */
#define BQ25898S_REG_00      		0x00
#define BQ25898S_ENHIZ_MASK		    0x80
#define BQ25898S_ENHIZ_SHIFT	    7
#define BQ25898S_HIZ_ENABLE         1
#define BQ25898S_HIZ_DISABLE        0

#define BQ25898S_IINLIM_MASK	    0x3F
#define BQ25898S_IINLIM_SHIFT		0
#define BQ25898S_IINLIM_BASE        100
#define BQ25898S_IINLIM_LSB         50

/* Register 01h */
#define BQ25898S_REG_01		    	0x01
#define BQ25898S_VINDPMOS_MASK      0x01
#define BQ25898S_VINDPMOS_SHIFT     0
#define	BQ25898S_VINDPMOS_400MV		0
#define	BQ25898S_VINDPMOS_600MV		1


/* Register 0x02 */
#define BQ25898S_REG_02              	0x02
#define BQ25898S_CONV_START_MASK     	0x80
#define BQ25898S_CONV_START_SHIFT    	7
#define BQ25898S_CONV_START          	0
#define BQ25898S_CONV_RATE_MASK       	0x40
#define BQ25898S_CONV_RATE_SHIFT      	6
#define BQ25898S_ADC_CONTINUE_ENABLE  	1
#define BQ25898S_ADC_CONTINUE_DISABLE 	0

#define BQ25898S_FORCE_DPDM_MASK     	0x02
#define BQ25898S_FORCE_DPDM_SHIFT    	1
#define BQ25898S_FORCE_DPDM          	1
#define BQ25898S_AUTO_DPDM_EN_MASK   	0x01
#define BQ25898S_AUTO_DPDM_EN_SHIFT  	0
#define BQ25898S_AUTO_DPDM_ENABLE    	1
#define BQ25898S_AUTO_DPDM_DISABLE   	0


/* Register 0x03 */
#define BQ25898S_REG_03             	0x03
#define BQ25898S_WDT_RESET_MASK		    0x40
#define BQ25898S_WDT_RESET_SHIFT     	6
#define BQ25898S_WDT_RESET           	1

#define BQ25898S_CHG_CONFIG_MASK     	0x10
#define BQ25898S_CHG_CONFIG_SHIFT    	4
#define BQ25898S_CHG_ENABLE          	1
#define BQ25898S_CHG_DISABLE         	0


/* Register 0x04*/
#define BQ25898S_REG_04              	0x04
#define BQ25898S_ICHG_MASK           	0x3F
#define BQ25898S_ICHG_SHIFT          	0
#define BQ25898S_ICHG_BASE           	0
#define BQ25898S_ICHG_LSB            	64

/* Register 0x05*/
#define BQ25898S_REG_05              0x05
#define BQ25898S_IPRECHG_MASK        0xF0
#define BQ25898S_IPRECHG_SHIFT       4
#define BQ25898S_ITERM_MASK          0x0F
#define BQ25898S_ITERM_SHIFT         0
#define BQ25898S_IPRECHG_BASE        64
#define BQ25898S_IPRECHG_LSB         64
#define BQ25898S_ITERM_BASE          64
#define BQ25898S_ITERM_LSB           64

/* Register 0x06*/
#define BQ25898S_REG_06              0x06
#define BQ25898S_VREG_MASK           0xFC
#define BQ25898S_VREG_SHIFT          2
#define BQ25898S_VREG_BASE           3840
#define BQ25898S_VREG_LSB            16
#define BQ25898S_BATLOWV_MASK        0x02
#define BQ25898S_BATLOWV_SHIFT       1
#define BQ25898S_BATLOWV_2800MV      0
#define BQ25898S_BATLOWV_3000MV      1
#define BQ25898S_VRECHG_MASK         0x01
#define BQ25898S_VRECHG_SHIFT        0
#define BQ25898S_VRECHG_100MV        0
#define BQ25898S_VRECHG_200MV        1

/* Register 0x07*/
#define BQ25898S_REG_07              0x07
#define BQ25898S_EN_TERM_MASK        0x80
#define BQ25898S_EN_TERM_SHIFT       7
#define BQ25898S_TERM_ENABLE         1
#define BQ25898S_TERM_DISABLE        0

#define BQ25898S_WDT_MASK            0x30
#define BQ25898S_WDT_SHIFT           4
#define BQ25898S_WDT_DISABLE         0
#define BQ25898S_WDT_40S             1
#define BQ25898S_WDT_80S             2
#define BQ25898S_WDT_160S            3
#define BQ25898S_WDT_BASE            0
#define BQ25898S_WDT_LSB             40

#define BQ25898S_EN_TIMER_MASK       0x08
#define BQ25898S_EN_TIMER_SHIFT      3

#define BQ25898S_CHG_TIMER_ENABLE    1
#define BQ25898S_CHG_TIMER_DISABLE   0

#define BQ25898S_CHG_TIMER_MASK      0x06
#define BQ25898S_CHG_TIMER_SHIFT     1
#define BQ25898S_CHG_TIMER_5HOURS    0
#define BQ25898S_CHG_TIMER_8HOURS    1
#define BQ25898S_CHG_TIMER_12HOURS   2
#define BQ25898S_CHG_TIMER_20HOURS   3


/* Register 0x08*/
#define BQ25898S_REG_08              0x08
#define BQ25898S_BAT_COMP_MASK       0xE0
#define BQ25898S_BAT_COMP_SHIFT      5
#define BQ25898S_VCLAMP_MASK         0x1C
#define BQ25898S_VCLAMP_SHIFT        2
#define BQ25898S_TREG_MASK           0x03
#define BQ25898S_TREG_SHIFT          0
#define BQ25898S_TREG_60C            0
#define BQ25898S_TREG_80C            1
#define BQ25898S_TREG_100C           2
#define BQ25898S_TREG_120C           3

#define BQ25898S_BAT_COMP_BASE       0
#define BQ25898S_BAT_COMP_LSB        20
#define BQ25898S_VCLAMP_BASE         0
#define BQ25898S_VCLAMP_LSB          32


/* Register 0x09*/
#define BQ25898S_REG_09              0x09
#define BQ25898S_TMR2X_EN_MASK       0x40
#define BQ25898S_TMR2X_EN_SHIFT      6
#define BQ25898S_TMR2X_ENABLE		 1
#define BQ25898S_TMR2X_DISABLE		 0

/* Register 0x0A*/
#define BQ25898S_REG_0A              0x0A


/* Register 0x0B*/
#define BQ25898S_REG_0B              0x0B
#define BQ25898S_VBUS_STAT_MASK      0xE0
#define BQ25898S_VBUS_STAT_SHIFT     5
#define BQ25898S_CHRG_STAT_MASK      0x18
#define BQ25898S_CHRG_STAT_SHIFT     3
#define BQ25898S_CHRG_STAT_IDLE      0
#define BQ25898S_CHRG_STAT_PRECHG    1
#define BQ25898S_CHRG_STAT_FASTCHG   2
#define BQ25898S_CHRG_STAT_CHGDONE   3

#define BQ25898S_PG_STAT_MASK        0x04
#define BQ25898S_PG_STAT_SHIFT       2
#define BQ25898S_VSYS_STAT_MASK      0x01
#define BQ25898S_VSYS_STAT_SHIFT     0


/* Register 0x0C*/
#define BQ25898S_REG_0C              0x0c
#define BQ25898S_FAULT_WDT_MASK      0x80
#define BQ25898S_FAULT_WDT_SHIFT     7
#define BQ25898S_FAULT_CHRG_MASK     0x30
#define BQ25898S_FAULT_CHRG_SHIFT    4
#define BQ25898S_FAULT_CHRG_NORMAL   0
#define BQ25898S_FAULT_CHRG_INPUT    1
#define BQ25898S_FAULT_CHRG_THERMAL  2
#define BQ25898S_FAULT_CHRG_TIMER    3

#define BQ25898S_FAULT_BAT_MASK      0x08
#define BQ25898S_FAULT_BAT_SHIFT     3


/* Register 0x0D*/
#define BQ25898S_REG_0D              0x0D
#define BQ25898S_FORCE_VINDPM_MASK   0x80
#define BQ25898S_FORCE_VINDPM_SHIFT  7
#define BQ25898S_FORCE_VINDPM_ENABLE 1
#define BQ25898S_FORCE_VINDPM_DISABLE 0
#define BQ25898S_VINDPM_MASK         0x7F
#define BQ25898S_VINDPM_SHIFT        0

#define BQ25898S_VINDPM_BASE         2600
#define BQ25898S_VINDPM_LSB          100


/* Register 0x0E*/
#define BQ25898S_REG_0E              0x0E
#define BQ25898S_THERM_STAT_MASK     0x80
#define BQ25898S_THERM_STAT_SHIFT    7
#define BQ25898S_BATV_MASK           0x7F
#define BQ25898S_BATV_SHIFT          0
#define BQ25898S_BATV_BASE           2304
#define BQ25898S_BATV_LSB            20


/* Register 0x0F*/
#define BQ25898S_REG_0F              0x0F
#define BQ25898S_SYSV_MASK           0x7F
#define BQ25898S_SYSV_SHIFT          0
#define BQ25898S_SYSV_BASE           2304
#define BQ25898S_SYSV_LSB            20


/* Register 0x11*/
#define BQ25898S_REG_11              0x11
#define BQ25898S_VBUS_GD_MASK        0x80
#define BQ25898S_VBUS_GD_SHIFT       7
#define BQ25898S_VBUSV_MASK          0x7F
#define BQ25898S_VBUSV_SHIFT         0
#define BQ25898S_VBUSV_BASE          2600
#define BQ25898S_VBUSV_LSB           100


/* Register 0x12*/
#define BQ25898S_REG_12              0x12
#define BQ25898S_ICHGR_MASK          0x7F
#define BQ25898S_ICHGR_SHIFT         0
#define BQ25898S_ICHGR_BASE          0
#define BQ25898S_ICHGR_LSB           50


/* Register 0x13*/
#define BQ25898S_REG_13              0x13
#define BQ25898S_VDPM_STAT_MASK      0x80
#define BQ25898S_VDPM_STAT_SHIFT     7
#define BQ25898S_IDPM_STAT_MASK      0x40
#define BQ25898S_IDPM_STAT_SHIFT     6
#define BQ25898S_IDPM_LIM_MASK       0x3F
#define BQ25898S_IDPM_LIM_SHIFT      0
#define BQ25898S_IDPM_LIM_BASE       100
#define BQ25898S_IDPM_LIM_LSB        50


/* Register 0x14*/
#define BQ25898S_REG_14              0x14
#define BQ25898S_RESET_MASK          0x80
#define BQ25898S_RESET_SHIFT         7
#define BQ25898S_RESET               1
#define BQ25898S_PN_MASK             0x38
#define BQ25898S_PN_SHIFT            3
#define BQ25898S_DEV_REV_MASK        0x03
#define BQ25898S_DEV_REV_SHIFT       0

#endif
