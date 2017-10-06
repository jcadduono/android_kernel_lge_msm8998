#include <linux/module.h>
#include <linux/firmware.h>
#include "msm_renesas_ois.h"
#include "msm_cci.h"
#include "msm_camera_i2c.h"

#define OIS_SID 0x24 //0x48

#define HALL_XY_SWAP_VER1	16701320
#define HALL_XY_SWAP_VER2	17017321

static struct msm_ois_func_tbl ois_func_tbl;
static int8_t pwm_prev_mode = -1;
static uint16_t af_pluse_timming[8]={
	0xFF00	,
	0x9600	,
	0x8200	,
	0x8200	,
	0x0E01	,
	0x9600	,
	0x6900	,
	0x9600	,
};
static uint16_t vcm_period_mode[8]={
	0xF703	,
	0x9B04	,
	0xE803	,
	0xE803	,
	0xE303	,
	0xB004	,
	0xD403	,
	0x0604	,
};

static void set_init_i2c_state(struct msm_ois_ctrl_t * o_ctrl){
	o_ctrl->i2c_client.cci_client->sid = OIS_SID;
	o_ctrl->i2c_client.cci_client->retries = 3;
	o_ctrl->i2c_client.cci_client->id_map = 0;
	o_ctrl->i2c_client.cci_client->cci_i2c_master = o_ctrl->cci_master;
	o_ctrl->i2c_client.cci_client->i2c_freq_mode = I2C_CUSTOM_MODE;
	o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
}

static void swap_16bit_data(uint16_t *data_16)
{
	uint8_t data_low, data_high;
	data_low = (uint8_t)(*data_16 &0xff);
	data_high = (uint8_t)((*data_16 >> 8) &0xff);

	*data_16 = (uint16_t)((data_low << 8) & 0xff00);
	*data_16 = *data_16 | (uint16_t)data_high;
}

int32_t msm_ois_gyro_calibration(struct msm_ois_ctrl_t *o_ctrl){
	int32_t rc = 0;
	uint16_t RcvData = 0,tmpData = 0;
	uint16_t g_offset_x = 0,g_offset_y = 0;
	uint16_t try_num = 0, max_try_num = 20;
	uint16_t read_low =0, read_high = 0;
	uint32_t fw_ver  =0;

	set_init_i2c_state(o_ctrl);

	CDBG("Enter\n");

	/* OIS Status Check */
	do
	{
		msleep(5);
		msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0001, &RcvData,1);/* OISSTS Read */
		try_num++;

		CDBG("OIS status : %d, try num : %d , max_try : %d\n", RcvData, try_num, max_try_num);
	}while(RcvData != 1 && try_num < max_try_num);

	if( RcvData != 1 ) /* OISSTS != IDLE */
	{
        CDBG (" OIS is not ready!!");
		return -EINVAL;
	}
#if 1 //FW version read
	msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x00FC, &read_low,2);	/* FW versiom [15: 0] */
	msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x00FE, &read_high,2); /* FW version [31:16] */

	swap_16bit_data (&read_low);
	swap_16bit_data (&read_high);

	fw_ver = (uint32_t)((read_high << 16) & 0xffff0000) | (uint32_t) ( read_low & 0x0000ffff);
	pr_err ("%s : fw version : %d ", __func__, fw_ver);
#endif

	msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0248, &g_offset_x,2); /* XGZERO Read */
	msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x024A, &g_offset_y,2); /* YGZERO Read */

	swap_16bit_data(&g_offset_x);
	swap_16bit_data(&g_offset_y);

	pr_err("Gyro offset before cal (%d, %d) \n", (int16_t)g_offset_x, (int16_t)g_offset_y);

	/* Gyro Calibration Start */
	msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0014, 0x01, 1); /* GCCTRL GSCEN set */
	msleep(25);

	/* Check Gyro Calibration Sequence End */
	try_num = 0;
	do
	{
		msleep(3);
		msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0014, &RcvData,1); /* GCCTRL Read */
		try_num++;

		CDBG("GCCTRL Read : %d, try num : %d \n", RcvData, try_num);
	}while(RcvData != 0 && try_num < max_try_num);

	/* Result check */
	msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0004, &RcvData,1); /* OISERR Read */

	if( (RcvData & 0xA3) == 0x0) /* OISERR register GXZEROERR & GYZEROERR & GCOMERR Bit = 0(No Error) */
	{
	CDBG("Gyro test success! \n");

		if ((fw_ver == HALL_XY_SWAP_VER1) || (fw_ver ==  HALL_XY_SWAP_VER2))//  x, y swap for gyro offset, need to check FW version
		{

			CDBG("Gyro offset cal SWAP!!! \n");
			msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0248, &g_offset_x,2); /* XGZERO Read */
			msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x024A, &g_offset_y,2); /* YGZERO Read */

			tmpData = g_offset_x;
			g_offset_x = g_offset_y;
			g_offset_y = tmpData;

			msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0248, g_offset_x,2); /* XGZERO Read */
			msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x024A, g_offset_y,2); /* YGZERO Read */

			swap_16bit_data(&g_offset_x);
			swap_16bit_data(&g_offset_y);

			pr_err("Gyro offset SWAP after cal (%d, %d) \n", (int16_t)g_offset_x, (int16_t)g_offset_y);
		}
		else
		{
			CDBG("Gyro offset cal normal!! \n");
			msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0248, &g_offset_x,2); /* XGZERO Read */
			msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x024A, &g_offset_y,2); /* YGZERO Read */

			swap_16bit_data(&g_offset_x);
			swap_16bit_data(&g_offset_y);
			pr_err("Gyro offset normal after cal (%d, %d) \n",(int16_t) g_offset_x, (int16_t)g_offset_y);
		}
	#if 1 // gyro cal data Flash write
		/* Write Gyro Calibration result to OIS DATA SECTION */
		msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0003, 0x01, 1); /* OISDATAWRITE register(0x0003) 1Byte Send */

		msleep(170); /* wait for Flash ROM Write */
		try_num = 0;
		do
		{
			msleep(3);
			try_num++;
			msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0003, &RcvData,1); /* OISDATAWRITE OIS_W read */
			CDBG("flash write (0x0003) status : %d, try num : %d \n",RcvData ,try_num);
		}while(RcvData != 0 && try_num < max_try_num);
	#endif
	}
	else
	{
		pr_err (" Gyro calibration error!!");
		return -EINVAL;
	}

	CDBG("Exit\n");
	return rc;
}

int32_t msm_ois_gyro_data_check(struct msm_ois_ctrl_t *o_ctrl){
	int32_t rc = 0;
	int16_t gyro_x = 0, gyro_y = 0;
	uint8_t i;
	uint16_t RcvData;
	uint8_t try_num, max_try_num = 30;
	uint8_t read_data[4];
	int32_t gyro_total_x = 0, gyro_total_y = 0;
	int16_t g_offset_x, g_offset_y;

	set_init_i2c_state(o_ctrl);


	CDBG("Enter\n");
	try_num = 0;
	do
	{
		msleep(5);
		msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0001, &RcvData,1);/* OISSTS Read */
		try_num++;

		CDBG("OIS status : %d, try num : %d \n", RcvData, try_num);
	}while(RcvData != 1 && try_num < max_try_num);

	msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0248, &g_offset_x,2); /* XGZERO Read */
	msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x024A, &g_offset_y,2); /* YGZERO Read */

	swap_16bit_data(&g_offset_x);
	swap_16bit_data(&g_offset_y);

	pr_err("OIS Gyro offset : (%d , %d) \n", g_offset_x, g_offset_y);


	msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0080, 0x01, 1); /* FW update enable*/
	try_num = 0;
	do
	{
		msleep(5);
		msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0080, &RcvData,1);/* Check FW update */
		try_num++;

		CDBG("Check FW update  : %d, try num : %d \n", RcvData, try_num);
	}while(RcvData != 1 && try_num < max_try_num);



	msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0000, 0x01, 1); /* OIS enable */
	msleep(5);
	try_num = 0;
	do
	{
		rc = msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0000, &RcvData,1);/* OISSTS Read */
		try_num++;
		if (rc != 1 || RcvData != 1)
		{
			msleep(5);
			CDBG("OIS enable (0x0000) rc : %d, RcvData : %d, try_num : %d \n", rc, RcvData, try_num);
		}
	}while(RcvData != 1 && try_num < max_try_num);

    for(i = 0; i < 10; i++)
    {
	msleep(5);
	try_num = 0;
	do{
		try_num++;
		rc = msm_camera_cci_i2c_read_seq(&o_ctrl->i2c_client, 0x0082, read_data, 4);/* read gyro X */
		if (rc != 0)
		{
			msleep(10);
				CDBG("I2C error, try num : %d \n", try_num);
			}
		}while (rc !=0 && try_num < max_try_num);

		gyro_x = (int16_t)((read_data[1] << 8) & 0xFF00)| (int16_t) read_data[0];
		gyro_y = (int16_t)((read_data[3] << 8) & 0xFF00)| (int16_t) read_data[2];

		gyro_total_x = gyro_total_x + gyro_x;
		gyro_total_y = gyro_total_y + gyro_y;

		CDBG("Gyro data %d, (%d, %d), total (%d, %d) \n", i, gyro_x, gyro_y, gyro_total_x, gyro_total_y);
	}

	pr_err("Average Gyro data (%d, %d) \n", (int32_t) (gyro_total_x /10) , (int32_t) (gyro_total_y / 10));

	msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0080, 0x00, 1); /* FW update disable*/

	try_num = 0;
	do
	{
		msleep(5);
		msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0080, &RcvData,1);/* Check FW update */
		try_num++;
		CDBG("Check FW update disable : %d, try num : %d \n", RcvData, try_num);
	}while(RcvData != 0 && try_num < max_try_num);

	msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0000, 0x00, 1); /* OIS disable */
	try_num = 0;
	do
	{
		msleep(5);
		msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0000, &RcvData,1);/* Check FW update */
		try_num++;
		CDBG("OIS disable : %d, try num : %d \n", RcvData, try_num);
	}while(RcvData != 0 && try_num < max_try_num);

	return rc;
}

int32_t msm_ois_hall_polarity_check(struct msm_ois_ctrl_t *o_ctrl){
	int32_t rc = 0;
	uint16_t RcvData=0;
	uint16_t try_num=0, max_try_num =10;

	set_init_i2c_state(o_ctrl);

	CDBG("Enter\n");

	/* OIS Status Check */
	do
	{
		msleep(5);
		msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0001, &RcvData,1);/* OISSTS Read */
		try_num++;

		CDBG("OIS status : %d, try num : %d \n", RcvData, try_num);
	}while(RcvData != 1 && try_num < max_try_num);

	if( RcvData != 1 ) /* OISSTS != IDLE */
	{

        pr_err ("%s : OIS is not ready!!",__func__);
		return -EINVAL;
	}

	/* Hall sensor polarity check */
	msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0012, 0x01, 1); /* HPCTRL HPEN set */

	/* Check Gyro Calibration Sequence End */
	try_num = 0;
	do
	{
		msleep(20);  // need more time
		msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0012, &RcvData,1); /* HPCTRLRead */
		try_num++;

		CDBG("HPCTRLRead : %d, try num : %d \n", RcvData, try_num);
	}while(RcvData != 0 && try_num < max_try_num);

	/* Result check */
	msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0200, &RcvData,1); /* HPOLA register POLAXERR &POLAYERR Bit == 0(No Error) */

	if( (RcvData & 0x0C) == 0x0) /* OISERR register GXZEROERR & GYZEROERR & GCOMERR Bit = 0(No Error) */
	{
	pr_err("%s : hall_polarity_check success! \n",__func__);
		#if 0  //
		/* Write Gyro Calibration result to OIS DATA SECTION */
		msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0003, 0x01, 1); /* OISDATAWRITE register(0x0003) 1Byte Send */

		msleep(170); /* wait for Flash ROM Write */
		try_num = 0;
		do
		{
			msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0003, &RcvData,1); /* OISDATAWRITE OIS_W read */
			msleep(3);
		    try_num++;

			CDBG("0x0003 try num : %d \n", try_num);
		}while(RcvData != 0 && try_num < max_try_num);
		#endif
	}
	else
	{

		pr_err ("%s : hall_polarity_check error!!\n",__func__);
		return -EINVAL;
	}

	CDBG("Exit\n");
	return rc;
}

int32_t msm_ois_renesas_get_info(struct msm_ois_ctrl_t *o_ctrl, struct msm_ois_info_t *ois_info)
{
	int32_t rc = 0;
	set_init_i2c_state(o_ctrl);

	snprintf(ois_info->ois_provider, ARRAY_SIZE(ois_info->ois_provider), "LGIT_RENESAS");

	return rc;
}

static unsigned short CalcCheckSum(unsigned short *Addr, unsigned short Size)
{
	unsigned short CheckSum;
	unsigned short i;
	CheckSum = 0;

	for( i = 0; i < (Size / 2); i++ )
	{
		CheckSum += Addr[i];
	}
	return CheckSum;
}

int32_t msm_ois_fw_update(struct msm_ois_ctrl_t *o_ctrl, const char* fw_name)
{
	uint16_t bytes_in_tx = 0;
	uint16_t total_bytes = 0;
	uint8_t *ptr = NULL;
	int32_t rc = 0;
	const struct firmware *fw = NULL;
	int read_cnt = 0;
	struct device *dev = &(o_ctrl->pdev->dev);
	uint8_t i2c_buf[32];

	unsigned char SendData[256];
	unsigned short RcvDataShort = 0;
	uint32_t fw_version;
	struct msm_camera_cci_client *cci_client = NULL;

	cci_client = o_ctrl->i2c_client.cci_client;
	set_init_i2c_state(o_ctrl);

	CDBG("Enter!!! \n");

	do{
		rc = msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0001, &RcvDataShort,1); //OISSTS
		msleep (5);
		read_cnt++;
	}while(RcvDataShort !=1 && read_cnt < 100);

	if( RcvDataShort != 1 ) /* OISSTS != IDLE */
	{
		pr_err("OISSTS != IDLE, RcvDataShort  %d read_cnt %d\n", RcvDataShort, read_cnt);
		return rc;
	}

	rc = msm_camera_cci_i2c_read_seq(&o_ctrl->i2c_client, 0x00FC, i2c_buf,4);
	if (rc < 0) {
		CDBG("Failed\n");
		return rc;
	}

	/* Load FW */
	rc = request_firmware(&fw, fw_name, dev);
	if (rc) {
		dev_err(dev, "Failed to locate %s\n", fw_name);
		return rc;
	}

	ptr = (uint8_t *)fw->data;
	fw_version = *(((uint32_t*)fw->data) + 0x6FF4/4);
	CDBG("load %s, new firmware version %d, size %d ", fw_name, fw_version,(uint16_t)fw->size);

    if( (*(ptr + 0x6FF4) == i2c_buf[0]) &&
		(*(ptr + 0x6FF5) == i2c_buf[1]) &&
		(*(ptr + 0x6FF6) == i2c_buf[2]) &&
		(*(ptr + 0x6FF7) == i2c_buf[3])){
		pr_err("Current FW version [ %d ] is the lasest version.\n",  fw_version);
		goto release_firmware;
    }

	rc = msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x000C, 0x73, 1); //0x71 : 64 bytes //0x73 : 128 bytes //0x75: 256 bytes
	if (rc < 0) {
		pr_err("Failed\n");
		goto release_firmware;
	}

	msleep (55);


	//Download FW
	total_bytes = fw->size;
	for (ptr = (uint8_t *)fw->data; total_bytes; total_bytes -= bytes_in_tx, ptr += bytes_in_tx) {

		bytes_in_tx = (total_bytes > 128) ? 128 : total_bytes;
		rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write_seq(&o_ctrl->i2c_client, 0x0100, ptr, bytes_in_tx);
		if (rc < 0) {
			pr_err("Failed:remaining bytes to be downloaded:%d\n",bytes_in_tx);
			/* abort download fw and return error*/
			goto release_firmware;
		}
		msleep(5);
	}


	//Err check
	rc = msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0006, &RcvDataShort,2); //OISSTS
	if (rc < 0) {
		pr_err("Failed\n");
		goto release_firmware;

	}

	msleep(100);
	if( RcvDataShort == 0x0000 ){
		/* CHECKSUM */
		uint16_t checkSum = CalcCheckSum((uint16_t*)fw->data, (uint16_t)fw->size/*28672*/);
		SendData [0] = (checkSum & 0x00FF);
		SendData [1] = (checkSum & 0xFF00) >> 8;
		SendData [2] = 0; /* Don't Care */
		SendData [3] = 0x80; /* Self Reset Request */
		rc = msm_camera_cci_i2c_write_seq(&o_ctrl->i2c_client, 0x0008, SendData, 4);
		if (rc < 0) {
			pr_err("Failed\n");
			goto release_firmware;
		}

		msleep(190);

		rc = msm_camera_cci_i2c_read(&o_ctrl->i2c_client, 0x0006, &RcvDataShort,2);
		if (rc < 0) {
			pr_err("Failed\n");
			goto release_firmware;
		}

		if( RcvDataShort == 0x0000 ){
			msm_camera_cci_i2c_read_seq(&o_ctrl->i2c_client, 0x00FC, i2c_buf,4);
			CDBG("NEW FW VERSION %x %x %x %x\n", i2c_buf[0],i2c_buf[1],i2c_buf[2],i2c_buf[3]);
		}else{
			pr_err("FW update failed 1 \n");
		}
	}	else {
			pr_err("FW update failed 2 \n");
	}
	CDBG("Exit!!! \n");

release_firmware:
	release_firmware(fw);
	return rc;
}


int32_t renesas_set_ois_mode(struct msm_ois_ctrl_t *o_ctrl,
					   struct msm_ois_set_info_t *set_info)
{
	int cur_mode = ois_func_tbl.ois_cur_mode;
	uint8_t mode = 0;
	int rc = OIS_SUCCESS;

	if (copy_from_user(&mode, (void *)set_info->setting, sizeof(uint8_t))) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		rc = -EFAULT;
		return rc;
	}
	pr_err("%s:Enter input mode : %d, current mode : %d \n", __func__, mode, cur_mode);

	set_init_i2c_state(o_ctrl);

	if (cur_mode == mode)
		return OIS_SUCCESS;

	switch (mode) {
	case OIS_MODE_PREVIEW_CAPTURE:
	case OIS_MODE_VIDEO:
	case OIS_MODE_CAPTURE:
		pr_err("%s:input mode : %d, current mode : %d \n", __func__, mode, cur_mode);
		rc = msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0002, 0x00, 1);

		break;
	case OIS_MODE_CENTERING_ONLY:
		pr_err("%s:%d, %d centering_only\n", __func__, mode, cur_mode);
		rc = msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0002, 0x05, 1);

		break;
	case OIS_MODE_CENTERING_OFF:
		pr_err("%s:%d, %d centering_off\n", __func__, mode, cur_mode);
		rc = msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0002, 0x00, 1);

		break;
	default:
		pr_err("%s:%d, %d default!!\n", __func__, mode, cur_mode);
	}

	ois_func_tbl.ois_cur_mode = mode;
	pr_err("%s:%d rc %d End\n", __func__, __LINE__,rc);

	return rc;
}

int32_t	renesas_ois_off(struct msm_ois_ctrl_t *o_ctrl,
					  struct msm_ois_set_info_t *set_info)
{
	int rc = OIS_SUCCESS;
	pr_err("%s:%d Enter\n", __func__, __LINE__);

	rc = msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0000, 0x00, 1);
	pr_err("%s:%d End  rc %d \n", __func__, __LINE__,rc);

	return rc;
}

int32_t	renesas_ois_on(struct msm_ois_ctrl_t *o_ctrl,
					 struct msm_ois_set_info_t *set_info)
{
	int32_t rc = OIS_SUCCESS;
	pr_err("%s:%d Enter\n", __func__, __LINE__);

	pr_err("%s:%d End\n", __func__, __LINE__);
	return rc;
}

int32_t renesas_ois_pwm_mode(struct msm_ois_ctrl_t *o_ctrl,
							struct msm_ois_set_info_t *set_info)
{
	uint8_t res_mode = 0;
	if (copy_from_user(&res_mode, (void *)set_info->setting, sizeof(uint8_t))) {
	        pr_err("%s:%d failed to get mode\n", __func__, __LINE__);
	        return OIS_FAIL;
	}
	pr_err("%s:%d Enter res mode %d, pwm_prev_mode %d\n", __func__, __LINE__,res_mode, pwm_prev_mode);
	if (pwm_prev_mode == res_mode ||
		(res_mode > 7 || res_mode < 0)) {
		return OIS_SUCCESS;
	}

	msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0000, 0x00, 1);
	msm_camera_cci_i2c_poll (&o_ctrl->i2c_client, 0x0000, 0x00, MSM_CAMERA_I2C_BYTE_DATA, 50);
	msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0060, 0x00, 1);
	msm_camera_cci_i2c_poll (&o_ctrl->i2c_client, 0x0060, 0x00, MSM_CAMERA_I2C_BYTE_DATA, 50);
	msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x00F0, 0x00, 1);
	msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x03F8, vcm_period_mode[res_mode], 2);
	msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0450, af_pluse_timming[res_mode], 2);
	msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0452, 0x0000, 2);
	msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0037, 0x01, 1);
	msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0000, 0x01, 1);
	msm_camera_cci_i2c_poll (&o_ctrl->i2c_client, 0x0000, 0x01, MSM_CAMERA_I2C_BYTE_DATA, 50);
	msm_camera_cci_i2c_write(&o_ctrl->i2c_client, 0x0060, 0x01, 1);
	msm_camera_cci_i2c_poll (&o_ctrl->i2c_client, 0x0060, 0x01, MSM_CAMERA_I2C_BYTE_DATA, 50);

	pwm_prev_mode = res_mode;

	pr_err("%s:%d End\n", __func__, __LINE__);
	return OIS_SUCCESS;
}

int32_t renesas_init_set_ois(struct msm_ois_ctrl_t *o_ctrl,
						   struct msm_ois_set_info_t *set_info)
{
	int32_t rc = OIS_SUCCESS;
	pr_err("%s:%d Enter\n", __func__, __LINE__);
	pwm_prev_mode = -1;

	set_init_i2c_state(o_ctrl);

	pr_err("%s:%d End\n", __func__, __LINE__);
	return rc;
}

void msm_renesas_ois_init(struct msm_ois_ctrl_t *o_ctrl){

	ois_func_tbl.ini_set_ois = renesas_init_set_ois;
	ois_func_tbl.enable_ois = renesas_ois_on;
	ois_func_tbl.disable_ois = renesas_ois_off;
	ois_func_tbl.ois_mode = renesas_set_ois_mode;
	ois_func_tbl.ois_stat = NULL;
	ois_func_tbl.ois_move_lens = NULL;
	ois_func_tbl.ois_pwm_mode = renesas_ois_pwm_mode;
	ois_func_tbl.ois_cur_mode = -1;

	o_ctrl->sid_ois = OIS_SID;
	o_ctrl->func_tbl = &ois_func_tbl;

	pr_err("%s",__func__);
	return;
}
