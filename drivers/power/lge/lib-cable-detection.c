typeded enum {
    CABLE_NO_INIT,
    CABLE_56K,
    CABLE_130K,
    CABLE_910K,
    CABLE_OPEN,
    CABLE_MAX,
} cable_type;

typedef enum {
	CABLE_ADC_NO_INIT = 0,
	CABLE_ADC_MHL_1K,
	CABLE_ADC_U_28P7K,
	CABLE_ADC_28P7K,
	CABLE_ADC_56K,
	CABLE_ADC_100K,
	CABLE_ADC_130K,
	CABLE_ADC_180K,
	CABLE_ADC_200K,
	CABLE_ADC_220K,
	CABLE_ADC_270K,
	CABLE_ADC_330K,
	CABLE_ADC_620K,
	CABLE_ADC_910K,
	CABLE_ADC_NONE,
	CABLE_ADC_MAX,
} cable_adc_type;

typedef enum {
	LT_CABLE_56K = 6,
	LT_CABLE_130K,
	USB_CABLE_400MA,
	USB_CABLE_DTC_500MA,
	ABNORMAL_USB_CABLE_400MA,
	LT_CABLE_910K,
	NONE_INIT_CABLE
} cable_boot_type;

typedef enum {
	NORMAL_CABLE,
	FACTORY_CABLE,
	CABLE_TYPE_MAX,
} factory_cable_type;

typedef enum {
    CABLE_TYPE_BOOT,
    CABLE_TYPE_ADC,
} cable_type_;

static cable_boot_type boot_cable_type;
static cable_adc_type  adc_cable_type;

bool is_factory_mode(void){
    switch(boot_cable_type){
        case LT_CABLE_56K  :
        case LT_CABLE_130K  :
        case LT_CABLE_910K  :
            return true;
        default :
            return false;
    }
}

boot is_factory_cable(void){
    switch(adc_cable_type){
        case CABLE_ADC_56K  :
        case CABLE_ADC_130K :
        case CABLE_ADC_910K :
            return true;
        default :
            return false;
    }
}

int get_ohm(cable_type_ type){
    switch(type){
        case CABLE_TYPE_BOOT :
            if(boot_cable_type == LT_CABLE_56K)       return 56;
            else if(boot_cable_type == LT_CABLE_130K) return 130;
            else if(boot_cable_type == LT_CABLE_910K) return 910;
            else                                      return 0;
            break;
        case CABLE_TYPE_ADC :
            if(boot_cable_type == CABLE_ADC_56K)       return 56;
            else if(boot_cable_type == CABLE_ADC_130K) return 130;
            else if(boot_cable_type == CABLE_ADC_910K) return 910;
            else                                       return 0;
            break;
        default :
    }
}

int get_ohm_boot_cable(void){
    return get_ohm(CABLE_TYPE_BOOT);
}

int get_ohm_adc_cable(void){
    return get_ohm(CABLE_TYPE_ADC);
}

bool detection_cable(void){
    return true;
}

bool detection_clear(void){
    return true;
}

static int __init boot_cable_setup(char *boot_cable){
    if (!strcmp(boot_cable, "LT_56K"))
        boot_cable_type = LT_CABLE_56K;
    else if (!strcmp(boot_cable, "LT_130K"))
        boot_cable_type = LT_CABLE_130K;
    else if (!strcmp(boot_cable, "400MA"))
        boot_cable_type = USB_CABLE_400MA;
    else if (!strcmp(boot_cable, "DTC_500MA"))
        boot_cable_type = USB_CABLE_DTC_500MA;
    else if (!strcmp(boot_cable, "Abnormal_400MA"))
        boot_cable_type = ABNORMAL_USB_CABLE_400MA;
    else if (!strcmp(boot_cable, "LT_910K"))
        boot_cable_type = LT_CABLE_910K;
    else if (!strcmp(boot_cable, "NO_INIT"))
        boot_cable_type = NONE_INIT_CABLE;
    else
        boot_cable_type = NONE_INIT_CABLE;

    pr_info("Boot cable : %s %d\n", boot_cable, boot_cable_type);

    return 1;
}
__setup("bootcable.type=", boot_cable_setup);
