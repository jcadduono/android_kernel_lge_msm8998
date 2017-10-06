/*! \file sx9320.c
 * \brief  SX9320 Driver
 *
 * Driver for the SX9320
 * Copyright (c) 2011 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
//#define DEBUG
#define DRIVER_NAME "sx9320"

#define MAX_WRITE_ARRAY_SIZE 32

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/syscalls.h>
#include <linux/wakelock.h>
#include <linux/uaccess.h>
#include <linux/sort.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/async.h>

#include "sx9320.h" 	/* main struct, interrupt,init,pointers */

//#define SX9320_DEBUG
//#define CONFIG_LGE_SENSOR

#define IDLE			0
#define ACTIVE			1

#define SX9320_NIRQ	   62

#define MAIN_SENSOR		2
#define REF_SENSOR		0
#define ENABLE_CSX		((1 << MAIN_SENSOR) | (1<<REF_SENSOR) )

/* Sensor Enable status */
#define SX9320_ON			1
#define SX9320_OFF			0
/* Sensor Enable Input Command */
#define SX9320_INPUT_ON     1
#define SX9320_INPUT_OFF    2
/* Proximity sensor status */
#define SX9320_FAR			1
#define SX9320_NEAR			0
/* Sensor Enable Mode */
#define SX9320_MODE_SLEEP	0
#define SX9320_MODE_NORMAL	1
/* Failer Index */
#define SX9320_ID_ERROR 	1
#define SX9320_NIRQ_ERROR	2
#define SX9320_CONN_ERROR	3
#define SX9320_I2C_ERROR	4

#define CAL_RET_ERROR            -1
#define CAL_RET_NONE             2
#define CAL_RET_EXIST            1
#define CAL_RET_SUCCESS          0

#define CALIBRATION_FILE_PATH   "/capsensor_cal.dat"

#define LIMIT_PROXOFFSET        13500 /* 99.9pF */// 2550 /* 30pF */
#define LIMIT_PROXOFFSET_LOW  	1000 /* 99.9pF */// 2550 /* 30pF */
#define LIMIT_PROXUSEFUL        30000

#define PROXOFFSET_LOW			500

//#define STANDARD_CAP_MAIN       300000
//#define DEFAULT_THRESHOLD_GAP   1000

#define SX9320_ANALOG_GAIN		1
#define SX9320_DIGITAL_GAIN		1
#define SX9320_ANALOG_RANGE		2.65

//#define INIT_THRESHOLD           (STANDARD_CAP_MAIN + DEFAULT_THRESHOLD_GAP)

#define	TOUCH_CHECK_REF_AMB      0 // 44523
#define	TOUCH_CHECK_SLOPE        0 // 50
#define	TOUCH_CHECK_MAIN_AMB     0 // 151282

/*! \struct sx9320
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */
typedef struct sx9320
{
	pbuttonInformation_t pbuttonInformation;
	psx9320_platform_data_t hw;		/* specific platform data settings */
} sx9320_t, *psx9320_t;

static u8 scantime;
//startup parameter
static s32 dynamic_slope;
static s32 dynamic_ref_offset;
static s32 dynamic_startupThreshold;
static s32 dynamic_StandardCapMain;
/*! \fn static int write_register(psx93XX_t this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */
static int write_register(psx93XX_t this, u8 address, u8 value)
{
	struct i2c_client *i2c = 0;
	char buffer[2];
	int returnValue = 0;

	buffer[0] = address;
	buffer[1] = value;
	returnValue = -ENOMEM;

	if (this && this->bus) {
		i2c = this->bus;
		returnValue = i2c_master_send(i2c,buffer,2);
		#ifdef DEBUG
		dev_info(&i2c->dev,"write_register Address: 0x%x Value: 0x%x Return: %d\n",
														address,value,returnValue);
		#endif
	}
	return returnValue;
}

/*! \fn static int read_register(psx93XX_t this, u8 address, u8 *value)
* \brief Reads a register's value from the device
* \param this Pointer to main parent struct
* \param address 8-Bit address to read from
* \param value Pointer to 8-bit value to save register value to
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int read_register(psx93XX_t this, u8 address, u8 *value)
{
	struct i2c_client *i2c = 0;
	s32 returnValue = 0;

	if (this && value && this->bus) {
		i2c = this->bus;
		returnValue = i2c_smbus_read_byte_data(i2c,address);

		#ifdef DEBUG
		dev_info(&i2c->dev, "read_register Address: 0x%x Return: 0x%x\n",
														address,returnValue);
		#endif

		if (returnValue >= 0) {
			*value = returnValue;
			return 0;
		}
		else {
			return returnValue;
		}
	}
	return -ENOMEM;
}

/*! \brief Sends a write register range to the device
 * \param this Pointer to main parent struct
 * \param reg 8-bit register address (base address)
 * \param data pointer to 8-bit register values
 * \param size size of the data pointer
 * \return Value from i2c_master_send
 */
#if 0
static int write_registerEx(psx93XX_t this, unsigned char reg,
								unsigned char *data, int size)
{
	struct i2c_client *i2c = 0;
	u8 tx[MAX_WRITE_ARRAY_SIZE];
	int ret = 0;

	if (this && (i2c = this->bus) && data && (size <= MAX_WRITE_ARRAY_SIZE))
	{
		dev_info(this->pdev, "inside write_registerEx()\n");
		tx[0] = reg;
		dev_info(this->pdev, "going to call i2c_master_send(0x%p, 0x%x ",
														(void *)i2c,tx[0]);
		for (ret = 0; ret < size; ret++)
		{
			tx[ret+1] = data[ret];
			dev_info(this->pdev, "0x%x, ",tx[ret+1]);
		}

		dev_info(this->pdev, "\n");

		ret = i2c_master_send(i2c, tx, size+1 );

		if (ret < 0)
		dev_err(this->pdev, "I2C write error\n");
	}

	dev_info(this->pdev, "leaving write_registerEx()\n");


	return ret;
}
/*! \brief Reads a group of registers from the device
* \param this Pointer to main parent struct
* \param reg 8-Bit address to read from (base address)
* \param data Pointer to 8-bit value array to save registers to
* \param size size of array
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int read_registerEx(psx93XX_t this, unsigned char reg,
											unsigned char *data, int size)
{
	struct i2c_client *i2c = 0;
	int ret = 0;
	u8 tx[] = {
		reg
	};
	if (this && (i2c = this->bus) && data && (size <= MAX_WRITE_ARRAY_SIZE))
	{
		dev_info(this->pdev, "inside read_registerEx()\n");
		dev_info(this->pdev,"going to call i2c_master_send(0x%p,0x%p,1) Reg: 0x%x\n",
														(void *)i2c,(void *)tx,tx[0]);
		ret = i2c_master_send(i2c,tx,1);
		if (ret >= 0) {
			dev_info(this->pdev, "going to call i2c_master_recv(0x%p,0x%p,%x)\n",
													(void *)i2c,(void *)data,size);
			ret = i2c_master_recv(i2c, data, size);
		}
	}
	if (unlikely(ret < 0))
		dev_err(this->pdev, "I2C read error\n");

	dev_info(this->pdev, "leaving read_registerEx()\n");
	return ret;
}
#endif

//static int sx9320_set_mode(psx93XX_t this, unsigned char mode);

/*! \fn static int read_regStat(psx93XX_t this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s)
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(psx93XX_t this)
{
	u8 data = 0;
	if (this) {
		if (read_register(this,SX9320_IRQSTAT_REG,&data) == 0){
			return (data & 0x00FF);
		}else{
			return -ENOMEM;
		}
	}
	return 0;
}

/*********************************************************************/
/*! \brief Perform a manual offset calibration
* \param this Pointer to main parent struct
* \return Value return value from the write register
 */
static int manual_offset_calibration(psx93XX_t this)
{
	s32 returnValue = 0;
	returnValue = write_register(this,SX9320_STAT2_REG,0x0F);
	return returnValue;
}
/*! \brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
static ssize_t manual_offset_calibration_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0;
	psx93XX_t this = dev_get_drvdata(dev);

	dev_info(this->pdev, "Reading IRQSTAT_REG\n");
	read_register(this,SX9320_IRQSTAT_REG,&reg_value);
	return sprintf(buf, "%d\n", reg_value);
}

/*! \brief sysfs store function for manual calibration
 */
static ssize_t manual_offset_calibration_store(struct device *dev,
			struct device_attribute *attr,const char *buf, size_t count)
{
	psx93XX_t this = dev_get_drvdata(dev);
	unsigned long val;
	if (kstrtoul(buf, 0, &val))
	return -EINVAL;
	if (val) {
		dev_info( this->pdev, "Performing manual_offset_calibration()\n");
		manual_offset_calibration(this);
	}
	return count;
}
#if 0
static int sx9320_save_caldata(psx93XX_t this)
{
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;
	int ret = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
						O_CREAT | O_TRUNC | O_WRONLY | O_SYNC,
						S_IRUGO | S_IWUSR | S_IWGRP);
	if (IS_ERR(cal_filp)) {
		pr_err("[SX9320]: %s - Can't open calibration file\n",
		__func__);
		set_fs(old_fs);
		ret = PTR_ERR(cal_filp);
		return ret;
	}

	ret = cal_filp->f_op->write(cal_filp, (char *)this->calData,
								sizeof(int) * 3, &cal_filp->f_pos);
	if (ret != (sizeof(int) * 3)) {
		pr_err("[SX9320]: %s - Can't write the cal data to file\n",__func__);
		ret = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return ret;
}

static void sx9320_open_caldata(psx93XX_t this)
{
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;
	int ret;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY,
									S_IRUGO | S_IWUSR | S_IWGRP);
	if (IS_ERR(cal_filp)) {
		ret = PTR_ERR(cal_filp);
		if (ret != -ENOENT)
			pr_err("[SX9320]: %s - Can't open calibration file.\n",__func__);
		else {
			pr_info("[SX9320]: %s - There is no calibration file\n",__func__);
			/* calibration status init */
			memset(this->calData, 0, sizeof(int) * 3);
		}
		set_fs(old_fs);
		return;
	}

	ret = cal_filp->f_op->read(cal_filp, (char *)this->calData,
										sizeof(int) * 3, &cal_filp->f_pos);
	if (ret != (sizeof(int) * 3))
		pr_err("[SX9320]: %s - Can't read the cal data from file\n",__func__);

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	pr_info("[SX9320]: %s - (%d, %d, %d)\n", __func__,this->calData[0],
										this->calData[1], this->calData[2]);
}
#endif

static u8 sx9320_get_scantime(psx93XX_t this)
{
	u8 regval=0;
	const u16 scantime_list[]={0,2,4,6,8,10,14,18,22,26,30,34,38,42,46,50,56,62,68,74,80,90,100,200,300,400,600,800,1000,2000,3000,4000};
	read_register(this, SX9320_CTRL0_REG, &regval);
	regval = regval&0x1f;
	return scantime_list[regval];
}

//sensor enable and disable, en=true->enable, en=false->disable
static void sx9320_sensor_enable_disable(psx93XX_t this, u8 en)
{
	u8 regval;
	read_register(this, SX9320_CTRL1_REG, &regval);

	if(en == 1) {
		write_register(this, SX9320_CTRL1_REG, regval | ENABLE_CSX);
    }
    else {
		write_register(this, SX9320_CTRL1_REG, regval & 0xF0);
	}
}

static int sx9320_get_useful(psx93XX_t this)
{
	u8 msByte = 0;
	u8 lsByte = 0;
	s16 fullByte = 0;

	write_register(this, SX9320_CPSRD, MAIN_SENSOR);
	read_register(this, SX9320_USEMSB, &msByte);
	read_register(this, SX9320_USELSB, &lsByte);
	fullByte = (s16)((msByte << 8) | lsByte);

	pr_info("[SX9320]: %s - PROXIUSEFUL = %d\n", __func__, fullByte);

	return (int)fullByte;
}

static int sx9320_get_offset(psx93XX_t this)
{
	u8 msByte = 0;
	u8 lsByte = 0;
	u16 fullByte = 0;

	write_register(this, SX9320_CPSRD, MAIN_SENSOR);
	read_register(this, SX9320_OFFSETMSB, &msByte);
	read_register(this, SX9320_OFFSETLSB, &lsByte);
	fullByte = (u16)((msByte << 8) | lsByte);

	pr_info("[SX9320]: %s - msByte=%u lsByte=%u\n", __func__, msByte,lsByte);
	pr_info("[SX9320]: %s - PROXIOFFSET = %u\n", __func__, fullByte);

	return (int)fullByte;
}
#if 0
static int sx9320_get_diff(psx93XX_t this)
{
	u8 msByte = 0;
	u8 lsByte = 0;
	u16 fullByte = 0;

	write_register(this, SX9320_CPSRD, MAIN_SENSOR);
	read_register(this, SX9320_DIFFMSB, &msByte);
	read_register(this, SX9320_DIFFMSB, &lsByte);
	fullByte = (u16)((msByte << 8) | lsByte);

	pr_info("[SX9320]: %s - PROXIDIFF = %u\n", __func__, fullByte);

	return (int)fullByte;
}
#endif

static s32 sx9320_get_capMargin(psx93XX_t this)
{
	u8 msByte = 0;
	u8 lsByte = 0;
	u16 offset = 0;
	s32 capMain = 0, useful = 0;
	s32 capMargin = 0;
	s32 capRef = 0;
	psx9320_t pDevice = NULL;
	if (this){
		pDevice = this->pDevice;
	}else{
		return -1;
	}

	pr_info("[SX9320]: %s - slope: %ld, offset: %ld, hysteresis: %ld\n",
	__func__, (long int)dynamic_slope, (long int)dynamic_ref_offset, (long int)dynamic_startupThreshold);

	/* Calculate out the Main Cap information */
	write_register(this, SX9320_CPSRD, MAIN_SENSOR);
	read_register(this, SX9320_USEMSB, &msByte);
	read_register(this, SX9320_USELSB, &lsByte);

	useful = (s32)msByte;
	useful = (useful << 8) | ((s32)lsByte);
	if (useful > 32767)
		useful -= 65536;

	read_register(this, SX9320_OFFSETMSB, &msByte);
	read_register(this, SX9320_OFFSETLSB, &lsByte);

	offset = (u16)msByte;
	offset = (offset << 8) | ((u16)lsByte);

	msByte = (u8)((offset >> 7) & 0x7F);
	lsByte = (u8)((offset)      & 0x7F);
	/*calculate Cap main*/
	capMain = (((s32)msByte * 21200) + ((s32)lsByte * 500)) +
						(((s32)useful * 50000) / (8 * 65536));
#if 1
	/* Calculate out the Reference Cap information */
	write_register(this, SX9320_CPSRD, REF_SENSOR);
	read_register(this, SX9320_USEMSB, &msByte);
	read_register(this, SX9320_USELSB, &lsByte);
	/* Calculate out the difference between the two */
	useful = (s32)msByte;
	useful = (useful << 8) | ((s32)lsByte);
	if (useful > 32767)
		useful -= 65536;

	read_register(this, SX9320_OFFSETMSB, &msByte);
	read_register(this, SX9320_OFFSETLSB, &lsByte);

	offset = (u16)msByte;
	offset = (offset << 8) | ((u16)lsByte);

	msByte = (u8)((offset >> 7) & 0x7F);
	lsByte = (u8)((offset)      & 0x7F);
	/*calculate Cap ref*/
	capRef = (((s32)msByte * 21200) + ((s32)lsByte * 500)) +
						(((s32)useful * 50000) / (8 * 65536));

	/*calucate Dynarmic threshold*/
	capRef = (capRef - dynamic_ref_offset) * dynamic_slope + TOUCH_CHECK_MAIN_AMB;

	/*caculate Cap Marjin*/
	capMargin = capMain - capRef;
#endif
	pr_info("[SX9320]: %s - CapMargin: %ld, capMain: %ld, useful: %ld, Offset: %u\n",
	__func__, (long int)capMargin, (long int)capMain, (long int)useful, offset);

	return capMargin;
}

static int sx9320_do_calibrate(psx93XX_t this, bool do_calib)
{
	int ret = 0;
	s32 capMargin;

	if (do_calib == false) {
		pr_info("[SX9320]: %s - Erase!\n", __func__);
		goto cal_erase;
	}
	this->failStatusCode = 0;

	//Just enable the sensor
    pr_info("[SX9320]: %s - 1.Power_status : %d\n", __func__, this->enable);
	if(this->enable == SX9320_OFF){
		sx9320_sensor_enable_disable(this,1);
		msleep(scantime);
	}

	this->calData[2] = sx9320_get_offset(this);
	if ((this->calData[2] >= LIMIT_PROXOFFSET) || (this->calData[2] == 0)) {
		pr_err("[SX9320]: %s - offset fail(%d)\n", __func__,this->calData[2]);
		goto cal_fail;
	}else if(this->calData[2] < LIMIT_PROXOFFSET_LOW){
		pr_err("[SX9320]: %s - please check connection issue(%d)\n", __func__,
															this->calData[2]);
		goto cal_fail;
	}

	this->calData[1] = sx9320_get_useful(this);

	if (this->calData[1] >= LIMIT_PROXUSEFUL) {
		pr_err("[SX9320]: %s - useful fail(%d)\n", __func__,
		this->calData[1]);
		goto cal_fail;
	}

	capMargin = sx9320_get_capMargin(this);

	this->calData[0] = capMargin - dynamic_StandardCapMain;

	goto exit;

cal_fail:
	ret = -1;
	this->failStatusCode = SX9320_CONN_ERROR;
cal_erase:
	memset(this->calData, 0, sizeof(int) * 3);
exit:
	pr_info("[SX9320]: %s - (%d, %d, %d)\n", __func__,
	this->calData[0], this->calData[1], this->calData[2]);

	//Just enable the sensor
    pr_info("[SX9320]: %s - 2.Power_status : %d\n", __func__, this->enable);
	if(this->enable == SX9320_OFF){
		sx9320_sensor_enable_disable(this,0);
	}

	return ret;
}

static int sx9320_StartupCheckWithRefSensor(psx93XX_t this)
{
	s32 capMargin;
	int counter=0;

	counter = MAIN_SENSOR;

	capMargin = sx9320_get_capMargin(this);
	pr_info("[SX9320]: %s - (capMargin=%d)\n", __func__, capMargin);

	if (capMargin >= (dynamic_StandardCapMain + dynamic_startupThreshold+ this->calData[0])) {
		dev_info(this->pdev, "start up  cap button %d touched\n", counter);
		//sprintf(this->pdev, "status = close, reg = %d\n", counter);
		//input_report_key(input, pCurrentButton->keycode, 1); //should report event here
		this->startupStatus = SX9320_NEAR;
	}
	else if(capMargin < (dynamic_StandardCapMain + dynamic_startupThreshold + this->calData[0])){
		dev_info(this->pdev, "start up cap button %d released\n",counter);
		//sprintf(this->pdev, "status = far, reg = %d\n", counter);
		//input_report_key(input, pCurrentButton->keycode, 0);//should report event here
		this->startupStatus = SX9320_FAR;
		this->startup_mode = false;
	}
	else{
		dev_info(this->pdev, "cap button %d none\n",counter);
	}
	this->proxStatus = this->startupStatus;

	return 0;
}

static int sx9320_Hardware_Check(psx93XX_t this)
{
	int ret;
	u8 failcode;
	u8 loop = 0;
	this->failStatusCode = 0;

	//Check th IRQ Status
	while(this->get_nirq_low && this->get_nirq_low()){
		read_regStat(this);
		msleep(scantime);
		if(++loop >10){
			this->failStatusCode = SX9320_NIRQ_ERROR;
			break;
		}
	}

	//Just enable the sensor
    pr_info("[SX9320]: %s - 1.Power_status : %d\n", __func__, this->enable);
	if(this->enable == SX9320_OFF){
		sx9320_sensor_enable_disable(this,1);
		msleep(scantime);
	}

	//Check I2C Connection
	ret = read_register(this, SX9320_WHOAMI_REG, &failcode);
	if(ret < 0){
		this->failStatusCode = SX9320_I2C_ERROR;
	}

	if(failcode!= SX9320_WHOAMI_VALUE){
		this->failStatusCode = SX9320_ID_ERROR;
	}

	//Check Main Sensor Cap Value
	read_register(this, SX9320_STAT2_REG, &failcode);
    pr_info("[SX9320]: %s - SX9320_STAT2_REG[%u] \n", __func__, failcode);

    if(((failcode>>4)&0x0f) & (1<<MAIN_SENSOR)){
        pr_info("[SX9320]: %s - 1 \n", __func__);
		this->failStatusCode = SX9320_CONN_ERROR;
	}
	if(PROXOFFSET_LOW > sx9320_get_offset(this)){
        pr_info("[SX9320]: %s - Abnormal State DefineOffset:[%d], GetOffset[%d] \n", __func__, PROXOFFSET_LOW, sx9320_get_offset(this));
		this->failStatusCode = SX9320_CONN_ERROR;
	}

	//Disable Sensor
    pr_info("[SX9320]: %s - 2.Power_status : %d\n", __func__, this->enable);
	if(this->enable == SX9320_OFF){
		sx9320_sensor_enable_disable(this,0);
	}

	if(this->failStatusCode){
		this->proxStatus = SX9320_NEAR;
		this->enable = 0;
		sx9320_sensor_enable_disable(this,0);//into Sleep mode
	}

	pr_err("[SX9320]: %s sx9320 failcode = 0x%x\n",__func__, this->failStatusCode);
	return (int)this->failStatusCode;
}

static int sx9320_set_mode(psx93XX_t this, unsigned char mode)
{
	int ret = 0;

	if (mode == SX9320_MODE_SLEEP) {
		sx9320_sensor_enable_disable(this,0);
		disable_irq(this->irq);
		disable_irq_wake(this->irq);
		ret = write_register(this, SX9320_IRQ_ENABLE_REG, 0x60);
	} else if (mode == SX9320_MODE_NORMAL) {
		sx9320_sensor_enable_disable(this,1);
		write_register(this, SX9320_IRQ_ENABLE_REG, 0x60);

		ret = sx9320_Hardware_Check(this);
		if(ret != 0){
			pr_err("[SX9320] - Hardware Failure code=(%d)\n", ret);
			return -EINVAL;
		}

		manual_offset_calibration(this);
		msleep(scantime *4);

		if(this->cal_done){
			this->startup_mode = true;
			ret = sx9320_StartupCheckWithRefSensor(this);
		}

		enable_irq(this->irq);
        enable_irq_wake(this->irq);
		read_regStat(this);
		ret = write_register(this, SX9320_IRQ_ENABLE_REG, 0x70);
	}

	 /* make sure no interrupts are pending since enabling irq
	  * will only work on next falling edge */

	 pr_info("[SX9320]: %s - change the mode : %u\n", __func__, mode);
	 return ret;
}
/*********************************************************************/
static int sx9320_global_variable_init(psx93XX_t this)
{
	this->enable = SX9320_OFF;
	this->irq_disabled = 0;
	this->cal_done = false;
	this->failStatusCode = 0;
	this->startupStatus = SX9320_FAR;
	this->proxStatus = SX9320_FAR;
	this->startup_mode = false;
	this->reg_in_dts = true;
	memset(this->calData, 0, sizeof(int) * 3);
	return 0;
}
static ssize_t sx9320_calibration_show(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	int ret;
	psx93XX_t this = dev_get_drvdata(dev);

	if ((this->cal_done == false) && (this->calData[2] == 0))
		ret = CAL_RET_NONE;
	else if ((this->cal_done == false) && (this->calData[2] != 0))
		ret = CAL_RET_EXIST;
	else if ((this->cal_done == true) && (this->calData[2] != 0))
		ret = CAL_RET_SUCCESS;
	else
		ret = CAL_RET_ERROR;

	return sprintf(buf, "Error=%d, failStatusCode =%d, CapMain=%d, Usefult=%d, Offset=%d\n",
									ret, this->failStatusCode,
									this->calData[0],this->calData[1], this->calData[2]);
}

static ssize_t sx9320_calibration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	bool do_calib;
	int ret;
	psx93XX_t this = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "1"))
		do_calib = true;
	else if (sysfs_streq(buf, "0"))
		do_calib = false;
	else {
		pr_info("[SX9320]: %s - invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	ret = write_register(this, SX9320_IRQ_ENABLE_REG, 0x60);
	manual_offset_calibration(this);
	msleep(scantime*4);

	ret = sx9320_do_calibrate(this, do_calib);

	if (ret < 0) {
		pr_err("[SX9320]: %s - sx9320_do_calibrate fail(%d)\n",__func__, ret);
		goto exit;
	}
	ret = write_register(this, SX9320_IRQ_ENABLE_REG, 0x70);

#if 0
	ret = sx9320_save_caldata(this);
	if (ret < 0) {
		pr_err("[SX9320]: %s - sx9320_save_caldata fail(%d)\n",__func__, ret);
		memset(this->calData, 0, sizeof(int) * 3);
		goto exit;
	}
#endif

	pr_info("[SX9320]: %s - %u success!\n", __func__, do_calib);

exit:
	if ((this->calData[0] != 0) && (ret >= 0))
		this->cal_done = true;
	else
		this->cal_done = false;
		return count;
}

static ssize_t sx9320_register_write_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	int reg_address = 0, val = 0;
	psx93XX_t this = dev_get_drvdata(dev);

	if (sscanf(buf, "%x,%x", &reg_address, &val) != 2) {
		pr_err("[SX9320]: %s - The number of data are wrong\n",__func__);
		return -EINVAL;
	}

	write_register(this, (unsigned char)reg_address, (unsigned char)val);
	pr_info("[SX9320]: %s - Register(0x%x) data(0x%x)\n",__func__, reg_address, val);

	return count;
}
//read all registers not include the advanced one
static ssize_t sx9320_register_read_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	u8 val=0;
	int regist = 0;
	psx93XX_t this = dev_get_drvdata(dev);

	dev_info(this->pdev, "Reading register\n");

	if (sscanf(buf, "%x", &regist) != 1) {
		pr_err("[SX9320]: %s - The number of data are wrong\n",__func__);
		return -EINVAL;
	}

	read_register(this, regist, &val);
	pr_info("[SX9320]: %s - Register(0x%2x) data(0x%2x)\n",__func__, regist, val);

	return count;
}

static ssize_t sx9320_sw_reset_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	int ret = 0;
	psx93XX_t this = dev_get_drvdata(dev);

	ret = write_register(this, SX9320_SOFTRESET_REG, SX9320_SOFTRESET);
	msleep(scantime);

	if (this->init)
		this->init(this);

	ret = sx9320_Hardware_Check(this);
	if(ret != 0){
			pr_err("[SX9320] - Hardware Failure code=(%d)\n", ret);
			return -EINVAL;
	}

	return sprintf(buf,"%d\n", ret);
}

static ssize_t sx9320_raw_data_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	u8 msb, lsb;
	s32 useful;
	s32 average;
	s32 diff;
	u16 offset;
	psx93XX_t this = dev_get_drvdata(dev);

	write_register(this, SX9320_CPSRD, MAIN_SENSOR);
	read_register(this, SX9320_USEMSB, &msb);
	read_register(this, SX9320_USELSB, &lsb);
	useful = (s32)((msb << 8) | lsb);

	read_register(this, SX9320_AVGMSB, &msb);
	read_register(this, SX9320_AVGLSB, &lsb);
	average = (s32)((msb << 8) | lsb);

	read_register(this, SX9320_OFFSETMSB, &msb);
	read_register(this, SX9320_OFFSETLSB, &lsb);
	offset = (u16)((msb << 8) | lsb);

	read_register(this, SX9320_DIFFMSB, &msb);
	read_register(this, SX9320_DIFFLSB, &lsb);
	diff = (s32)((msb << 8) | lsb);

	if (useful > 32767)
		useful -= 65536;
	if ( diff> 32767)
		diff -= 65536;
	if (average > 32767)
		average -= 65536;
#ifdef SX9320_DEBUG
	pr_info("[SX9320]: %s - Useful : %d Average : %d, Offset : %d, DIFF : %d  \n",
            __func__, useful, average, offset, diff);
#endif
	return sprintf(buf, "Useful : %d Average : %d, Offset : %d, DIFF : %d\n",
										useful,average,offset,diff);
}

static ssize_t sx9320_show_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 val=0;
	psx93XX_t this = dev_get_drvdata(dev);

	pr_info("[SX9320]: %s -  data(0x%2x)\n", __func__, val);

	dev_info(this->pdev,"enable : %d, cal_done :%d, startup_mode : %d \n",this->enable,this->cal_done,this->startup_mode);

	if(this->startupStatus == SX9320_NEAR) {
		if(this->proxStatus == SX9320_NEAR) {
            pr_info("[SX9320]: %s - Startup status = Near || Prox Status = Near || Failercode = %d \n",__func__,this->failStatusCode);

			//return sprintf(buf, "Startup status = Near || Prox Status = Near || Failercode = %d \n",this->failStatusCode);
            return sprintf(buf,"%d\n",SX9320_NEAR); // Near = 0 , Far = 1
        }
		else {
            pr_info("[SX9320]: %s - Startup status = Near || Prox Status = Far || Failercode = %d \n",__func__,this->failStatusCode);

            //return sprintf(buf, "Startup status = Near || Prox Status = Far || Failercode = %d \n",this->failStatusCode);
            return sprintf(buf,"%d\n",SX9320_NEAR); // Near = 0 , Far = 1
        }
	}else {
		if(this->proxStatus == SX9320_NEAR) {
             pr_info("[SX9320]: %s - Startup status = Far || Prox Status = Near || Failercode = %d \n",__func__,this->failStatusCode);

            //return sprintf(buf, "Startup status = Far || Prox Status = Near || Failercode = %d \n",this->failStatusCode);
            return sprintf(buf,"%d\n",SX9320_NEAR); // Near = 0 , Far = 1
        }
		else {
            pr_info("[SX9320]: %s - Startup status = Far || Prox Status = Far || Failercode = %d \n",__func__,this->failStatusCode);

			//return sprintf(buf, "Startup status = Far || Prox Status = Far || Failercode = %d \n",this->failStatusCode);
            return sprintf(buf,"%d\n",SX9320_FAR); // Near = 0 , Far = 1
        }
	}

	//read_register(this, SX9320_STAT0_REG, &val);
/*
	if(this->startup_mode == false){
		this->proxStatus = this->startupStatus;
	}else{
		this->proxStatus = (val & 0x04)?SX9320_NEAR:SX9320_FAR;
	}
	//if((val & 0x04) || (this->startupStatus == SX9320_NEAR))
	if(this->proxStatus)
	{
		return sprintf(buf, "status = close, reg = %d\n", this->proxStatus);
	}else{
		return sprintf(buf, "status = far, reg = %d\n", this->proxStatus);
	}
	*/
}

static int sx9320_set_enable( psx93XX_t this, int enable)
{
	int ret;
	int pre_enable = this->enable;

	if (enable == SX9320_INPUT_ON) {
	    pr_info("[SX9320]: %s - Sensor on\n", __func__);
		if (pre_enable == SX9320_OFF) {
			this->enable=SX9320_ON;
			ret = sx9320_set_mode(this, SX9320_MODE_NORMAL);
		}
        else {
	        pr_info("[SX9320]: %s - Already SX9320_ON\n", __func__);
        }
	}
    else if (enable == SX9320_INPUT_OFF) {
	    pr_info("[SX9320]: %s - Sensor off\n", __func__);
		if (pre_enable == SX9320_ON) {
			this->enable=SX9320_OFF;
			ret = sx9320_set_mode(this, SX9320_MODE_SLEEP);
		}
        else {
	        pr_info("[SX9320]: %s - Already SX9320_OFF\n", __func__);
        }
	}
    else {
        ret = -1;

    }
#ifdef SX9320_DEBUG
	pr_info("[SX9320]: %s - Return Value[%d]\n", __func__, ret);
#endif

	return ret;
}
static ssize_t sx9320_enable_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long enable;
	psx93XX_t this = dev_get_drvdata(dev);

	if(kstrtoul(buf, 0, &enable)) {
		pr_err("[SX9320]: %s - 1.Invalid Argument\n", __func__);
		return -EINVAL;
    };

	pr_info("[SX9320]: %s - new_value = %ld\n", __func__, enable);
	if ((enable == SX9320_INPUT_OFF) || (enable == SX9320_INPUT_ON)){    // on = 1, off = 2
		if(sx9320_set_enable(this, (int)enable) < 0){
			pr_err("[SX9320]: %s - 2.Invalid Argument\n", __func__);
			return -EINVAL;
		}
	}
	return size;
}

static ssize_t sx9320_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	psx93XX_t this = dev_get_drvdata(dev);
	return sprintf(buf, "SX9320 Enable(1:ON 2:OFF) => %d\n", this->enable);
}

static DEVICE_ATTR(manual_calibrate, 0664, manual_offset_calibration_show,manual_offset_calibration_store);
static DEVICE_ATTR(register_write,  0664, NULL,sx9320_register_write_store);
static DEVICE_ATTR(register_read,0664, NULL,sx9320_register_read_store);
static DEVICE_ATTR(sw_reset, 0664,sx9320_sw_reset_show,NULL);
static DEVICE_ATTR(raw_data,0664,sx9320_raw_data_show,NULL);
static DEVICE_ATTR(status,0664,sx9320_show_status,NULL);
static DEVICE_ATTR(enable,0664,sx9320_enable_show,sx9320_enable_store);
static DEVICE_ATTR(calibration,0664,sx9320_calibration_show,sx9320_calibration_store);

static struct attribute *sx9320_attributes[] = {
	&dev_attr_manual_calibrate.attr,
	&dev_attr_register_write.attr,
	&dev_attr_register_read.attr,
	&dev_attr_sw_reset.attr,
	&dev_attr_raw_data.attr,
	&dev_attr_status.attr,
	&dev_attr_enable.attr,
	&dev_attr_calibration.attr,
	NULL,
};
static struct attribute_group sx9320_attr_group = {
	.attrs = sx9320_attributes,
};

/*********************************************************************/
static void read_rawData(psx93XX_t this)
{
	u8 msb=0, lsb=0;
	if(this){
		write_register(this,SX9320_CPSRD,2);//here to check the CS1, also can read other channel
		read_register(this,SX9320_USEMSB,&msb);
		read_register(this,SX9320_USELSB,&lsb);
		dev_info(this->pdev, "sx9320 raw data USEFUL msb = 0x%x, lsb = 0x%x\n",msb,lsb);

		read_register(this,SX9320_AVGMSB,&msb);
		read_register(this,SX9320_AVGLSB,&lsb);
		dev_info(this->pdev, "sx9320 raw data AVERAGE msb = 0x%x, lsb = 0x%x\n",msb,lsb);

		read_register(this,SX9320_DIFFMSB,&msb);
		read_register(this,SX9320_DIFFLSB,&lsb);
		dev_info(this->pdev, "sx9320 raw data DIFF msb = 0x%x, lsb = 0x%x\n",msb,lsb);

		read_register(this,SX9320_OFFSETMSB,&msb);
		read_register(this,SX9320_OFFSETLSB,&lsb);
		dev_info(this->pdev, "sx9320 raw data OFFSET msb = 0x%x, lsb = 0x%x\n",msb,lsb);
	}
}

/*! \brief  Initialize I2C config from platform data
 * \param this Pointer to main parent struct
 */
static void sx9320_reg_init(psx93XX_t this)
{
	psx9320_t pDevice = 0;
	psx9320_platform_data_t pdata = 0;
	int i = 0;
	/* configure device */
	pr_info("[SX9320] : %s Going to Setup I2C Registers\n", __func__);
	if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw))
	{
		/*******************************************************************************/
		// try to initialize from device tree!
		/*******************************************************************************/
		if (this->reg_in_dts == true) {
			while ( i < pdata->i2c_reg_num) {
				/* Write all registers/values contained in i2c_reg */
				dev_info(this->pdev, "Going to Write Reg from dts: 0x%x Value: 0x%x\n",
				pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
				write_register(this, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
				i++;
			}
		} else { // use static ones!!
			while ( i < ARRAY_SIZE(sx9320_i2c_reg_setup)) {
				/* Write all registers/values contained in i2c_reg */
				dev_info(this->pdev, "Going to Write Reg: 0x%x Value: 0x%x\n",
				sx9320_i2c_reg_setup[i].reg,sx9320_i2c_reg_setup[i].val);
				write_register(this, sx9320_i2c_reg_setup[i].reg,sx9320_i2c_reg_setup[i].val);
				i++;
			}
		}
	/*******************************************************************************/
	} else {
		pr_err("[SX9320] : %s ERROR! platform data\n", __func__);
	}

	scantime = sx9320_get_scantime(this);
}


/*! \fn static int initialize(psx93XX_t this)
 * \brief Performs all initialization needed to configure the device
 * \param this Pointer to main parent struct
 * \return Last used command's return value (negative if error)
 */
static int initialize(psx93XX_t this)
{
	int ret;
	if (this) {
		pr_info("SX9320 income initialize\n");
		/* prepare reset by disabling any irq handling */
		this->irq_disabled = 1;
		disable_irq(this->irq);
		/* perform a reset */
		write_register(this,SX9320_SOFTRESET_REG,SX9320_SOFTRESET);
		/* wait until the reset has finished by monitoring NIRQ */
		dev_info(this->pdev, "Sent Software Reset. Waiting until device is back from reset to continue.\n");
		/* just sleep for awhile instead of using a loop with reading irq status */
		msleep(scantime);
		//    while(this->get_nirq_low && this->get_nirq_low()) { read_regStat(this); }
		dev_info(this->pdev, "Device is back from the reset, continuing. NIRQ = %d\n",this->get_nirq_low());

		ret = sx9320_global_variable_init(this);

		sx9320_reg_init(this);
		msleep(scantime); /* make sure everything is running */
		manual_offset_calibration(this);

		/* re-enable interrupt handling */
		enable_irq(this->irq);

		#if 0
		//Open teh Cal data
		sx9320_open_caldata(this);

		if(this->calData[0]==0 && this->calData[0]==0 && this->calData[0]==0){
			this->cal_done = false;
		}
		else{
			this->cal_done = true;
		)
		#endif
		/* make sure no interrupts are pending since enabling irq will only
		* work on next falling edge */
		read_regStat(this);
		msleep(scantime);
		dev_info(this->pdev, "Exiting initialize(). NIRQ = %d\n",this->get_nirq_low());
		return 0;
	}
	return -ENOMEM;
}

static void compensationProcess(psx93XX_t this)
{
	if(this->proxStatus == SX9320_NEAR){
		this->startupStatus = SX9320_FAR;
		this->proxStatus=SX9320_FAR;
		this->startup_mode = false;
	}
	dev_info(this->pdev, "cap button %d released - compensation\n",this->proxStatus);
	sx9320_Hardware_Check(this);
	read_regStat(this);
}
/*!
 * \brief Handle what to do when a touch occurs
 * \param this Pointer to main parent struct
 */
static void touchProcess(psx93XX_t this)
{
	int counter = 0;
	u8 i = 0;
	int numberOfButtons = 0;
	psx9320_t pDevice = NULL;
	struct _buttonInfo *buttons = NULL;
	struct input_dev *input = NULL;

	struct _buttonInfo *pCurrentButton  = NULL;

	if ((this && (pDevice = this->pDevice)) & (this->startup_mode == false))
	{
	//dev_info(this->pdev, "Inside touchProcess()\n");
		read_register(this, SX9320_STAT0_REG, &i);
		if(i & (1<<MAIN_SENSOR))
		{
			this->proxStatus = SX9320_NEAR;
		}else
		{
			this->proxStatus = SX9320_FAR;
		}

		buttons = pDevice->pbuttonInformation->buttons;
		input = pDevice->pbuttonInformation->input;
		numberOfButtons = pDevice->pbuttonInformation->buttonSize;

		if (unlikely( (buttons==NULL) || (input==NULL) )) {
			dev_err(this->pdev, "ERROR!! buttons or input NULL!!!\n");
			return;
		}

		for (counter = 0; counter < numberOfButtons; counter++) {
			pCurrentButton = &buttons[counter];
			if (pCurrentButton==NULL) {
				dev_err(this->pdev,"ERROR!! current button at index: %d NULL!!!\n", counter);
				return; // ERRORR!!!!
			}
	        dev_err(this->pdev,"[SX9320]: %s - 1[%d]2[%d]state[%d]\n", __func__, i&pCurrentButton->mask, pCurrentButton->mask, pCurrentButton->state);
			switch (pCurrentButton->state) {
				case IDLE: /* Button is not being touched! */
					if (((i & pCurrentButton->mask) == pCurrentButton->mask)) {
						/* User pressed button */
#ifdef SX9320_DEBUG
						dev_err(this->pdev,"[SX9320]: %s - 1.cap button %d touched\n", __func__, counter);
						dev_err(this->pdev,"[SX9320]: %s - 2.ABS_DISTANCE %d keycode[%d]\n", __func__, ABS_DISTANCE, pCurrentButton->keycode);
#endif
                        dev_info(this->pdev, "cap button %d touched\n", counter);
						input_report_key(input, pCurrentButton->keycode, 0);

                        pCurrentButton->state = ACTIVE;
					} else {
						dev_info(this->pdev, "Button %d already released.\n",counter);
					}
					break;
				case ACTIVE: /* Button is being touched! */
					if (((i & pCurrentButton->mask) != pCurrentButton->mask)) {
						/* User released button */
						dev_err(this->pdev,"[SX9320]: %s - cap button %d released\n", __func__, counter);
						dev_err(this->pdev,"[SX9320]: %s - RELEASED ABS_DISTANCE %d keycode[%d]\n", __func__, ABS_DISTANCE, pCurrentButton->keycode);

                        input_report_key(input, pCurrentButton->keycode, 1);
						pCurrentButton->state = IDLE;
					} else {
						dev_info(this->pdev, "Button %d still touched.\n",counter);
					}
					break;
				default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
					break;
			};
		}
	    dev_err(this->pdev,"[SX9320]: %s - input_sync \n", __func__);

		input_sync(input);

   //dev_info(this->pdev, "Leaving touchProcess()\n");
	} else {
		pr_err("[SX9320] : %s ERROR!\n", __func__);
	}
}

static int sx9320_parse_dt(struct sx9320_platform_data *pdata, struct device *dev)
{
	struct device_node *dNode = dev->of_node;
	enum of_gpio_flags flags;
	int ret;
	if (dNode == NULL)
		return -ENODEV;

	pdata->irq_gpio= of_get_named_gpio_flags(dNode,
											"Semtech,nirq-gpio", 0, &flags);
	if (pdata->irq_gpio < 0) {
		pr_err("[SENSOR]: %s - get irq_gpio error\n", __func__);
		return -ENODEV;
	}
	/***********************************************************************/
/* Startup function */
	ret = of_property_read_u32(dNode, "Semtech,DynamicThres_REF_Offset", (u32*)&pdata->pStartupCheckParameters->dynamicthreshold_ref_offset);
	if (ret == 0) {
		pr_info("[SX9320]: %s - DynamicThres_REF_Offset=[%d]\n", __func__, pdata->pStartupCheckParameters->dynamicthreshold_ref_offset);
		dynamic_ref_offset=pdata->pStartupCheckParameters->dynamicthreshold_ref_offset;
	}
	ret = of_property_read_u32(dNode, "Semtech,DynamicThres_Temp_Slope", (u32*)&pdata->pStartupCheckParameters->dynamicthreshold_temp_slope);
	if (ret == 0) {
		pr_info("[SX9320]: %s - DynamicThres_Temp_Slope=[%d]\n", __func__, pdata->pStartupCheckParameters->dynamicthreshold_temp_slope);
		dynamic_slope=pdata->pStartupCheckParameters->dynamicthreshold_temp_slope;
	}
	ret = of_property_read_u32(dNode, "Semtech,DynamicThres_StartupThreshold", (u32*)&pdata->pStartupCheckParameters->dynamicthreshold_StartupThreshold);
	if (ret == 0) {
		pr_info("[SX9320]: %s - DynamicThres_StartupThreshold=[%d]\n", __func__, pdata->pStartupCheckParameters->dynamicthreshold_StartupThreshold);
		dynamic_startupThreshold = pdata->pStartupCheckParameters->dynamicthreshold_StartupThreshold;
	}
	ret = of_property_read_u32(dNode, "Semtech,DynamicThres_StandardCapMain", (u32*)&pdata->pStartupCheckParameters->dynamicthreshold_StandardCapMain);
	if (ret == 0) {
		pr_info("[SX9320]: %s - DynamicThres_StandardCapMain=[%d]\n", __func__, pdata->pStartupCheckParameters->dynamicthreshold_StandardCapMain);
		dynamic_StandardCapMain= pdata->pStartupCheckParameters->dynamicthreshold_StandardCapMain;
	}
	/***********************************************************************/
	// load in registers from device tree
	pdata->i2c_reg_num = of_property_count_u8_elems(dNode,"Semtech,reg-init");
	// layout is register, value, register, value....
	// if an extra item is after just ignore it. reading the array in will cause it to fail anyway
	pdata->i2c_reg_num /= 2;
	pr_info("[SX9320]:%s -  size of elements %d \n", __func__,pdata->i2c_reg_num);
	if (pdata->i2c_reg_num > 0) {
		 // initialize platform reg data array
		 pdata->pi2c_reg = devm_kzalloc(dev,sizeof(struct smtc_reg_data)*pdata->i2c_reg_num, GFP_KERNEL);
		 if (unlikely(pdata->pi2c_reg == NULL)) {
			return -ENOMEM;
		}

	 // initialize the array
		if (of_property_read_u8_array(dNode,"Semtech,reg-init",(u8*)&(pdata->pi2c_reg[0]),sizeof(struct smtc_reg_data)*pdata->i2c_reg_num))
		return -ENOMEM;
	}
	/***********************************************************************/
	pr_info("[SX9320]: %s -[%d] parse_dt complete\n", __func__,pdata->irq_gpio);
	return 0;
}

/* get the NIRQ state (1->NIRQ-low, 0->NIRQ-high) */
static int sx9320_init_platform_hw(struct i2c_client *client)
{
	psx93XX_t this = i2c_get_clientdata(client);
	struct sx9320 *pDevice = NULL;
	struct sx9320_platform_data *pdata = NULL;

	int rc;

	pr_info("[SX9320] : %s init_platform_hw start!",__func__);

	if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
		if (gpio_is_valid(pdata->irq_gpio)) {
			rc = gpio_request(pdata->irq_gpio, "sx9320_irq_gpio");
			if (rc < 0) {
				dev_err(this->pdev, "SX9320 Request gpio. Fail![%d]\n", rc);
				return rc;
			}
			rc = gpio_direction_input(pdata->irq_gpio);
			if (rc < 0) {
				dev_err(this->pdev, "SX9320 Set gpio direction. Fail![%d]\n", rc);
				return rc;
			}
			pr_info("[SX9320] %s pinctrl start!",__func__);
			this->irq = client->irq = gpio_to_irq(pdata->irq_gpio);
			/* -------------------------------- */
#if 1   // Wifi = 0, RF = 1
			this->pinctrl.ctrl = devm_pinctrl_get(this->pdev);
			this->pinctrl.active = pinctrl_lookup_state(this->pinctrl.ctrl, "capsensor_int_active");
			pinctrl_select_state(this->pinctrl.ctrl, this->pinctrl.active);
			pr_info("SX9320 pinctrl complete");
#endif
		}
		else {
			dev_err(this->pdev, "SX9320 Invalid irq gpio num.(init)\n");
		}
	}
	else {
		pr_err("[SX9320] : %s - Do not init platform HW", __func__);
	}

	pr_err("[SX9320]: %s - sx9320_irq_debug\n",__func__);
	return rc;
}

static void sx9320_exit_platform_hw(struct i2c_client *client)
{
	psx93XX_t this = i2c_get_clientdata(client);
	struct sx9320 *pDevice = NULL;
	struct sx9320_platform_data *pdata = NULL;

	if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
		if (gpio_is_valid(pdata->irq_gpio)) {
			gpio_free(pdata->irq_gpio);
		}
		else {
			dev_err(this->pdev, "Invalid irq gpio num.(exit)\n");
		}
	}
	return;
}

static int sx9320_get_nirq_state(void)
{
	return  !gpio_get_value(SX9320_NIRQ);
}

/*! \fn static int sx9320_probe(struct i2c_client *client, const struct i2c_device_id *id)
 * \brief Probe function
 * \param client pointer to i2c_client
 * \param id pointer to i2c_device_id
 * \return Whether probe was successful
 */
static int sx9320_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i = 0;
	int err = 0;

	psx93XX_t this = 0;
	psx9320_t pDevice = 0;
	psx9320_platform_data_t pplatData = 0;
	struct totalButtonInformation *pButtonInformationData = NULL;
	pstartupcheckparameters_t pStartupCheckParameters;
	struct input_dev *input = NULL;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	dev_info(&client->dev, "sx9320_probe()\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_WORD_DATA)) {
		dev_err(&client->dev, "Check i2c functionality.Fail!\n");
		err = -EIO;
		return err;
	}

	this = devm_kzalloc(&client->dev,sizeof(sx93XX_t), GFP_KERNEL); /* create memory for main struct */
	dev_info(&client->dev, "\t Initialized Main Memory: 0x%p\n",this);

	pButtonInformationData = devm_kzalloc(&client->dev , sizeof(struct totalButtonInformation), GFP_KERNEL);
	if (!pButtonInformationData) {
		dev_err(&client->dev, "Failed to allocate memory(totalButtonInformation)\n");
		err = -ENOMEM;
		return err;
	}

	pButtonInformationData->buttonSize = ARRAY_SIZE(psmtcButtons);
	pButtonInformationData->buttons =  psmtcButtons;
	//Startup function
	pStartupCheckParameters = devm_kzalloc(&client->dev, sizeof(struct _startupcheckparameters), GFP_KERNEL);
	if (!pStartupCheckParameters) {
		dev_err(&client->dev, "Failed to allocate memory(_startupCheckParameters)\n");
		err = -ENOMEM;
		return err;
	}
	pplatData = devm_kzalloc(&client->dev,sizeof(struct sx9320_platform_data), GFP_KERNEL);
	if (!pplatData) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}
	pplatData->get_is_nirq_low = sx9320_get_nirq_state;
	pplatData->pbuttonInformation = pButtonInformationData;
	pplatData->pStartupCheckParameters = pStartupCheckParameters;

	client->dev.platform_data = pplatData;
	err = sx9320_parse_dt(pplatData, &client->dev);
	if (err) {
		dev_err(&client->dev, "could not setup pin\n");
		//pr_err("[SX9320]: %s - could not setup pin\n", __func__);
		return ENODEV;
	}

	pplatData->init_platform_hw = sx9320_init_platform_hw;
	dev_err(&client->dev, "SX9320 init_platform_hw done!\n");

	if (this){
		dev_info(&client->dev, "SX9320 initialize start!!");
		/* In case we need to reinitialize data
		* (e.q. if suspend reset device) */
		this->init = initialize;
		/* shortcut to read status of interrupt */
		this->refreshStatus = read_regStat;
		/* pointer to function from platform data to get pendown
		* (1->NIRQ=0, 0->NIRQ=1) */
		this->get_nirq_low = pplatData->get_is_nirq_low;
		/* save irq in case we need to reference it */
		this->irq = client->irq;
		/* do we need to create an irq timer after interrupt ? */
		this->useIrqTimer = 0;

		/* Setup function to call on corresponding reg irq source bit */
		if (MAX_NUM_STATUS_BITS>= 8)
		{
			this->statusFunc[0] = 0; /* TXEN_STAT */
			this->statusFunc[1] = 0; /* UNUSED */
			this->statusFunc[2] = 0; /* UNUSED */
			this->statusFunc[3] = read_rawData; /* CONV_STAT */
			this->statusFunc[4] = compensationProcess; /* COMP_STAT */
			this->statusFunc[5] = touchProcess; /* RELEASE_STAT */
			this->statusFunc[6] = touchProcess; /* TOUCH_STAT  */
			this->statusFunc[7] = 0; /* RESET_STAT */
		}

		/* setup i2c communication */
		this->bus = client;
		i2c_set_clientdata(client, this);

		/* record device struct */
		this->pdev = &client->dev;

		/* create memory for device specific struct */
		this->pDevice = pDevice = devm_kzalloc(&client->dev,sizeof(sx9320_t), GFP_KERNEL);
		dev_info(&client->dev, "\t Initialized Device Specific Memory: 0x%p\n",pDevice);

		if (pDevice){
			/* for accessing items in user data (e.g. calibrate) */
			err = sysfs_create_group(&client->dev.kobj, &sx9320_attr_group);
			//sysfs_create_group(client, &sx9320_attr_group);

			/* Add Pointer to main platform data struct */
			pDevice->hw = pplatData;

			/* Check if we hava a platform initialization function to call*/
			if (pplatData->init_platform_hw)
			pplatData->init_platform_hw(client);

			/* Initialize the button information initialized with keycodes */
			pDevice->pbuttonInformation = pplatData->pbuttonInformation;
			/* Create the input device */
			input = input_allocate_device();
			if (!input) {
				return -ENOMEM;
			}
			/* Set all the keycodes */
			__set_bit(EV_KEY, input->evbit);
			#if 1
			for (i = 0; i < pButtonInformationData->buttonSize; i++) {
				__set_bit(pButtonInformationData->buttons[i].keycode,input->keybit);
				pButtonInformationData->buttons[i].state = IDLE;
			}
			#endif
			/* save the input pointer and finish initialization */
			pButtonInformationData->input = input;
			input->name = "sx9320";
			input->id.bustype = BUS_I2C;
            input_set_drvdata(input, this);

			if(input_register_device(input)){
				return -ENOMEM;
			}
            err = sysfs_create_group(&input->dev.kobj, &sx9320_attr_group);
            if (err)
                dev_err(this->pdev,"Failed to input create_group");
        }
		sx93XX_IRQ_init(this);
		/* call init function pointer (this should initialize all registers */
		if (this->init){
			this->init(this);
		}else{
			dev_err(this->pdev,"No init function!!!!\n");
			return -ENOMEM;
		}
	}else{
		return -1;
	}

	if(sx9320_Hardware_Check(this)!=0){
			err = -EINVAL;
			pr_err("[SX9320] - Hardware Failure code=(%d)\n", err);
			return err;
	}
	pplatData->exit_platform_hw = sx9320_exit_platform_hw;

	dev_info(&client->dev, "sx9320_probe() Done\n");

	return 0;
}

/*! \fn static int sx9320_remove(struct i2c_client *client)
 * \brief Called when device is to be removed
 * \param client Pointer to i2c_client struct
 * \return Value from sx93XX_remove()
 */
//static int __devexit sx9320_remove(struct i2c_client *client)
static int sx9320_remove(struct i2c_client *client)
{
	psx9320_platform_data_t pplatData =0;
	psx9320_t pDevice = 0;
	psx93XX_t this = i2c_get_clientdata(client);
	if (this && (pDevice = this->pDevice))
	{
		input_unregister_device(pDevice->pbuttonInformation->input);

		sysfs_remove_group(&client->dev.kobj, &sx9320_attr_group);
		pplatData = client->dev.platform_data;
		if (pplatData && pplatData->exit_platform_hw)
			pplatData->exit_platform_hw(client);
		kfree(this->pDevice);
	}
	return sx93XX_remove(this);
}
#if 1//def CONFIG_PM
/*====================================================*/
/***** Kernel Suspend *****/
static int sx9320_suspend(struct device *dev)
{
	psx93XX_t this = dev_get_drvdata(dev);
	sx93XX_suspend(this);
	return 0;
}
/***** Kernel Resume *****/
static int sx9320_resume(struct device *dev)
{
	psx93XX_t this = dev_get_drvdata(dev);
	sx93XX_resume(this);
	return 0;
}
/*====================================================*/
#else
#define sx9320_suspend		NULL
#define sx9320_resume		NULL
#endif /* CONFIG_PM */

static struct i2c_device_id sx9320_idtable[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sx9320_idtable);
#ifdef CONFIG_OF
static struct of_device_id sx9320_match_table[] = {
	{ .compatible = "Semtech,sx9320",},
	{ },
};
#else
#define sx9320_match_table NULL
#endif
static const struct dev_pm_ops sx9320_pm_ops = {
	.suspend = sx9320_suspend,
	.resume = sx9320_resume,
};
static struct i2c_driver sx9320_driver = {
	.driver = {
		.owner			= THIS_MODULE,
		.name			= DRIVER_NAME,
		.of_match_table	= sx9320_match_table,
		.pm				= &sx9320_pm_ops,
	},
	.id_table		= sx9320_idtable,
	.probe			= sx9320_probe,
	.remove			= sx9320_remove,
	//.remove    = __devexit_p(sx9320_remove),
};

static void async_sx9320_I2C_init(void *data, async_cookie_t cookie)
{
	pr_info("%s async_sx9320_I2C_init: init\n", DRIVER_NAME);
	i2c_add_driver(&sx9320_driver);
	return;
}

static int __init sx9320_I2C_init(void)
{
	async_schedule(async_sx9320_I2C_init, NULL);
	return 0;
}
static void __exit sx9320_I2C_exit(void)
{
	i2c_del_driver(&sx9320_driver);
}

module_init(sx9320_I2C_init);
module_exit(sx9320_I2C_exit);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX9320 Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

static void sx93XX_schedule_work(psx93XX_t this, unsigned long delay)
{
	unsigned long flags;
	if (this) {
		dev_info(this->pdev, "sx93XX_schedule_work()\n");
		spin_lock_irqsave(&this->lock,flags);
		/* Stop any pending penup queues */
		cancel_delayed_work(&this->dworker);
		//after waiting for a delay, this put the job in the kernel-global workqueue. so no need to create new thread in work queue.
		schedule_delayed_work(&this->dworker,delay);
		spin_unlock_irqrestore(&this->lock,flags);
	}
	else
		printk(KERN_ERR "sx93XX_schedule_work, NULL psx93XX_t\n");
}

static irqreturn_t sx93XX_irq(int irq, void *pvoid)
{
	psx93XX_t this = 0;
	if (pvoid) {
		this = (psx93XX_t)pvoid;
		//dev_info(this->pdev, "sx93XX_irq\n");
		if ((!this->get_nirq_low) || this->get_nirq_low()) {
		//dev_info(this->pdev, "sx93XX_irq - Schedule Work\n");
		sx93XX_schedule_work(this,0);
		}
		else{
			dev_err(this->pdev, "sx93XX_irq - nirq read high\n");
		}
	}
	else{
		printk(KERN_ERR "sx93XX_irq, NULL pvoid\n");
	}
	return IRQ_HANDLED;
}

static void sx93XX_worker_func(struct work_struct *work)
{
	psx93XX_t this = 0;
	int status = 0;
	int counter = 0;
	u8 nirqLow = 0;
	if (work) {
		this = container_of(work,sx93XX_t,dworker.work);

		if (!this) {
			printk(KERN_ERR "sx93XX_worker_func, NULL sx93XX_t\n");
			return;
		}
		if (unlikely(this->useIrqTimer)) {
			if ((!this->get_nirq_low) || this->get_nirq_low()) {
				nirqLow = 1;
			}
		}
		/* since we are not in an interrupt don't need to disable irq. */
		status = this->refreshStatus(this);
		counter = -1;
		dev_dbg(this->pdev, "Worker - Refresh Status %d\n",status);

		while((++counter) < MAX_NUM_STATUS_BITS) { /* counter start from MSB */
			if (((status>>counter) & 0x01) && (this->statusFunc[counter])) {
				dev_info(this->pdev, "SX9320 Function Pointer Found. Calling\n");
				this->statusFunc[counter](this);
			}
		}
		if (unlikely(this->useIrqTimer && nirqLow))
		{	/* Early models and if RATE=0 for newer models require a penup timer */
			/* Queue up the function again for checking on penup */
			sx93XX_schedule_work(this,msecs_to_jiffies(this->irqTimeout));
		}
	} else {
		printk(KERN_ERR "sx93XX_worker_func, NULL work_struct\n");
	}
}

static int sx93XX_remove(psx93XX_t this)
{
	if (this) {
		cancel_delayed_work_sync(&this->dworker); /* Cancel the Worker Func */
		/*destroy_workqueue(this->workq); */
		free_irq(this->irq, this);
		kfree(this);
		return 0;
	}
	return -ENOMEM;
}
static void sx93XX_suspend(psx93XX_t this)
{
	if (this)
		disable_irq(this->irq);

	//write_register(this,SX9320_CTRL1_REG,0x20);//make sx9320 in Sleep mode
}
static void sx93XX_resume(psx93XX_t this)
{
	if (this)
		enable_irq(this->irq);
}

static int sx93XX_IRQ_init(psx93XX_t this)
{
	int err = 0;
	if (this && this->pDevice)
	{
		/* initialize spin lock */
		spin_lock_init(&this->lock);
		/* initialize worker function */
		INIT_DELAYED_WORK(&this->dworker, sx93XX_worker_func);
		/* initailize interrupt reporting */
		this->irq_disabled = 0;
		err = request_irq(this->irq, sx93XX_irq, IRQF_TRIGGER_FALLING,
							this->pdev->driver->name, this);
		if (err) {
			dev_err(this->pdev, "irq %d busy?\n", this->irq);
			return err;
		}
		dev_info(this->pdev, "registered with irq (%d)\n", this->irq);
        disable_irq(this->irq);
	}
	return -ENOMEM;
}
