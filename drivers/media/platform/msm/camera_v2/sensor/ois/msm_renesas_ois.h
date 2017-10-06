#ifndef MSM_RENESAS_OIS_H
#define MSM_RENESAS_OIS_H
#include "msm_ois.h"

void msm_renesas_ois_init(struct msm_ois_ctrl_t *o_ctrl);
int32_t msm_ois_fw_update(struct msm_ois_ctrl_t *o_ctrl, const char* fw_name);

int32_t msm_ois_renesas_get_info(struct msm_ois_ctrl_t *o_ctrl, struct msm_ois_info_t *ois_info);

int32_t msm_ois_hall_polarity_check(struct msm_ois_ctrl_t *o_ctrl);
int32_t msm_ois_gyro_data_check(struct msm_ois_ctrl_t *o_ctrl);
int32_t msm_ois_gyro_calibration(struct msm_ois_ctrl_t *o_ctrl);


#endif
