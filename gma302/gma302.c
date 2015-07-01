#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/uaccess.h>
#include "gma302.h"
#ifdef EWMA_FILTER
#include <linux/average.h>
struct ewma average[SENSOR_DATA_SIZE];
#endif

//#define DEBUG /**< if define : Enable gma->gma_client->dev debug data .*/
// device info
#define ABSMIN					-(1024*16)
#define ABSMAX					1024*16
#define FUZZ					0
#define LSG						1024//64
#define MAX_DELAY				200

struct gma_acc{
    s16    x;
    s16    y;
    s16    z;
} ;

struct gma_data {
    struct i2c_client *gma_client;
    struct input_dev *input;
    atomic_t	delay;
    atomic_t	enable;
    struct mutex enable_mutex;
    struct delayed_work work;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
    atomic_t	position;
    atomic_t	calibrated;
    struct gma_acc	offset;
    atomic_t	fuzz;
	atomic_t	addr;
#ifdef SMA_FILTER
	int sum[SENSOR_DATA_SIZE];	/* SMA_FILTER sum */
	s16	bufferave[SENSOR_DATA_SIZE][16];
	int sma_filter;
#endif
#ifdef EWMA_FILTER
	int	ewma_filter[SENSOR_DATA_SIZE];
#endif
};

// cfg data : 1-- used
#define CFG_GSENSOR_USE_CONFIG 0

// calibration file path
#define CFG_GSENSOR_CALIBFILE   "/data/data/com.actions.sensor.calib/files/gsensor_calib.txt" 
//char GMA_Offset_TXT[] = "/data/data/com.actions.sensor.calib/files/gsensor_calib.txt";	///< SAVE FILE PATH offset.txt
/*******************************************
* for xml cfg
*******************************************/
#define CFG_GSENSOR_ADAP_ID          "gsensor.i2c_adap_id"
#define CFG_GSENSOR_POSITION         "gsensor.position"
#define CFG_GSENSOR_CALIBRATION      "gsensor.calibration"

//extern int get_config(const char *key, char *buff, int len);
/*******************************************
* end for xml cfg
*******************************************/

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gma_early_suspend(struct early_suspend *h);
static void gma_early_resume(struct early_suspend *h);
#endif

static int gma_axis_remap(struct i2c_client *client, struct gma_acc *acc);

static int gma_smbus_read_byte(struct i2c_client *client,
        unsigned char reg_addr, unsigned char *data)
{
    s32 dummy;
    dummy = i2c_smbus_read_byte_data(client, reg_addr);
    if (dummy < 0)
        return -1;
    *data = dummy & 0x000000ff;

    return 0;
}

static int gma_smbus_write_byte(struct i2c_client *client,
        unsigned char reg_addr, unsigned char data)
{
    s32 dummy;
    dummy = i2c_smbus_write_byte_data(client, reg_addr, data);
    if (dummy < 0)
        return -1;
    return 0;
}

static int gma_smbus_read_byte_block(struct i2c_client *client,
        unsigned char reg_addr, unsigned char *data, unsigned char len)
{
    s32 dummy;
    dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
    if (dummy < 0)
        return -1;
    return 0;
}

static int gma_smbus_write_byte_block(struct i2c_client *client,
        unsigned char reg_addr, unsigned char *data, unsigned char len)
{
    s32 dummy;    
    dummy = i2c_smbus_write_i2c_block_data(client, reg_addr, len, data);
        if (dummy < 0)
            return -1;    
    return 0;
}
static int gma30x_hw_init(struct i2c_client *client)
{
	//struct gma_data *gma = i2c_get_clientdata(client);
    int comres = 0;    
	unsigned char stadr=0, buffer[8];
	/* 1. check GMA1302_REG_PID(0x00) , chipid = 0x33  */
        buffer[0] = 0x02;
        gma_smbus_write_byte_block(client, 0x01, buffer, 1);	
	msleep(5);
	comres += gma_smbus_read_byte(client, GMA1302_REG_PID, &stadr);
	if( stadr == GMA302_VAL_WMI)
		dev_info(&client->dev, "%s: PID = 0x%x, GMA302 accelerometer\n", __func__, stadr);
	else if( stadr == GMA303_VAL_WMI)
		dev_info(&client->dev, "%s: PID = 0x%x, GMA303 accelerometer\n", __func__, stadr);
	else if( stadr == GMA303_VAL_WMI_RD)
		dev_info(&client->dev, "%s: PID = 0x%x, GMA303_RD sample\n", __func__, stadr);
	else{
		dev_err(&client->dev, "%s: PID = 0x%x, The device is not GlobalMems accelerometer.", __func__, stadr);
		return -1;
	}
	/* 2. turn off the low-pass filter */
	buffer[0] = GMA1302_VAL_OFF;//GMA1302_VAL_LPF_ON;
	comres += gma_smbus_write_byte_block(client, GMA1302_REG_CONTR1, buffer, 1);
	/* 3. turn on the offset temperature compensation */
	buffer[0] = GMA1302_VAL_OFFSET_TC_ON;
	comres += gma_smbus_write_byte_block(client, GMA1302_REG_CONTR3, buffer, 1);
	/* 4. turn off the data ready interrupt and configure the INT pin to active high, push-pull type */
	buffer[0] = GMA1302_VAL_OFF;    //GMA1302_VAL_DATA_READY_ON;
	comres += gma_smbus_write_byte_block(client, GMA1302_REG_INTCR, buffer, 1);
	/* 5. treshold set to max */
    buffer[0] = GMA1302_VAL_TRESHOLD_MAX; 
    comres += gma_smbus_write_byte_block(client, GMA1302_REG_MTHR, buffer, 1);
	/* 7. Oversampling mode & Set Action register */
/*	buffer[0] = GMA1302_VAL_LOW_NOISE; 
    gma_smbus_write_byte_block(client, GMA1302_REG_OSM, buffer, 1);
	buffer[0] = GMA1302_VAL_ACTR_CONTINUOUS;
	buffer[1] = GMA1302_VAL_ACTR_RESET;
	buffer[2] = GMA1302_VAL_ACTR_NON_CONTINUOUS;
	buffer[3] = GMA1302_VAL_ACTR_RESET;
	gma_smbus_write_byte_block(client, GMA1302_REG_ACTR, buffer, 4);
*/	
#ifdef EWMA_FILTER
	ewma_init(&average[0], EWMA_FACTOR, EWMA_WEIGHT_X);
	ewma_init(&average[1], EWMA_FACTOR, EWMA_WEIGHT_Y);
	ewma_init(&average[2], EWMA_FACTOR, EWMA_WEIGHT_Z);
	gma->ewma_filter[0] = EWMA_WEIGHT_X;
	gma->ewma_filter[1] = EWMA_WEIGHT_Y;
	gma->ewma_filter[2] = EWMA_WEIGHT_Z;
	dev_info(&gma->gma_client->dev, "EWMA_FILTER: %d %d %d\n", gma->ewma_filter[0], gma->ewma_filter[1], gma->ewma_filter[2]);
#endif
	return comres;
}

static int gma30x_set_mode(struct i2c_client *client, unsigned char mode)
{
    int comres = 0;
    unsigned char  buffer[2]; 
	
	if(mode == GMA1302_MODE_POWERDOWN)
	{
		buffer[0] = GMA1302_MODE_POWERDOWN;
		comres += gma_smbus_write_byte_block(client, GMA1302_REG_PD, buffer, 1);				 
	}
	
	if(mode == GMA1302_MODE_RESET) 
	{
		/* 1. Powerdown -> Reset */
		buffer[0] = GMA1302_MODE_RESET;
		comres += gma_smbus_write_byte_block(client, GMA1302_REG_PD, buffer, 1);
		/* 2. hw init */
		gma30x_hw_init(client);
	}

    return comres;
}

#ifdef SMA_FILTER
/* for Simple Moving Average */
static int SMA( struct gma_data *gma, s16 *xyz){
	int i, j;
	static s8	pointer = -1;				/* last update data */

	/* init gma->sum */
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		gma->sum[i] = 0;

	pointer++;
	pointer %= gma->sma_filter;
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		gma->bufferave[i][pointer] = xyz[i];

    for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		for(j = 0; j < gma->sma_filter; ++j)
			gma->sum[i] += gma->bufferave[i][j];

	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		xyz[i] = (s16)(gma->sum[i] / gma->sma_filter);

	return 0;
}
#endif

static int gma_read_data(struct i2c_client *client, struct gma_acc *acc)
{
    int i, comres = 0;
    unsigned char data[11];
    struct gma_data *gma = i2c_get_clientdata(client);
	s16 xyzTmp[SENSOR_DATA_SIZE];
    comres += gma_smbus_read_byte_block(client, GMA1302_REG_STADR, data, sizeof(data));
    if (comres < 0)
		dev_err(&gma->gma_client->dev, "read data fail! comres=%d\n", comres);
	else{
		/* merge xyz high/low bytes(13bit) & 1g = 512*2 =1024 */
		for(i = 0; i < SENSOR_DATA_SIZE; ++i)
			xyzTmp[i] = (signed short) (((data[2*(i+2)] << 8) | data[2*(i+1)+1]) << 1);
	}
	//dev_dbg(&gma->gma_client->dev, "before xyzTmp: %3d , %3d , %3d\n", xyzTmp[0], xyzTmp[1], xyzTmp[2]);
#ifdef SMA_FILTER
	if (xyzTmp[0] != 0 && xyzTmp[1] != 0 && xyzTmp[2] != 0)
		SMA( gma, (s16 *)&xyzTmp);
#endif
	/* enable ewma filter */
#ifdef EWMA_FILTER
	ewma_add(&average[0], (unsigned long) (xyzTmp[0] + EWMA_POSITIVE));
	ewma_add(&average[1], (unsigned long) (xyzTmp[1] + EWMA_POSITIVE));
	ewma_add(&average[2], (unsigned long) (xyzTmp[2] + EWMA_POSITIVE));
	acc->x = (s16) (ewma_read(&average[0]) - EWMA_POSITIVE);
	acc->y = (s16) (ewma_read(&average[1]) - EWMA_POSITIVE);
	acc->z = (s16) (ewma_read(&average[2]) - EWMA_POSITIVE);
#else
	acc->x = xyzTmp[0];
	acc->y = xyzTmp[1];
	acc->z = xyzTmp[2];
	dev_dbg(&gma->gma_client->dev, "xyzTmp: %3d , %3d , %3d :%d\n", xyzTmp[0], xyzTmp[1], xyzTmp[2], __LINE__);
#endif
 	
    return comres;
}

static bool get_value_as_int(char const *buf, size_t size, int *value){
	long tmp;
	if (size == 0)
		return false;
	/* maybe text format value */
	if ((buf[0] == '0') && (size > 1)) {
		if ((buf[1] == 'x') || (buf[1] == 'X')) {
			/* hexadecimal format */
			if (0 != strict_strtol(buf, 16, &tmp))
				return false;
		} else {
			/* octal format */
			if (0 != strict_strtol(buf, 8, &tmp))
				return false;
		}
	} else {
		/* decimal format */
		if (0 != strict_strtol(buf, 10, &tmp))
			return false;
	}

	if (tmp > INT_MAX)
		return false;

	*value = tmp;
	return true;
}

/* sysfs SMA show & store */
#ifdef SMA_FILTER
static ssize_t gma_sma_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct gma_data *gma = input_get_drvdata(input);

	return sprintf(buf, "%d\n", gma->sma_filter);
}

static ssize_t gma_sma_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct gma_data *gma = input_get_drvdata(input);
	unsigned long filter;

	if (strict_strtoul(buf, 10, &filter) < 0)
		return count;

	if (!((filter >= 1) && (filter <= 16)))
		return -1;
	gma->sma_filter = filter;
	return count;
}
#endif
/* sysfs EWMA show & store */
#ifdef EWMA_FILTER
static ssize_t gma_ewma_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct gma_data *gma = input_get_drvdata(input);
	return sprintf(buf, "%d %d %d\n", gma->ewma_filter[0], gma->ewma_filter[1], gma->ewma_filter[2]);
}

static ssize_t gma_ewma_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct gma_data *gma = input_get_drvdata(input);

	sscanf(buf, "%d %d %d", (int *)&gma->ewma_filter[0], (int *)&gma->ewma_filter[1], (int *)&gma->ewma_filter[2]);
	if ((gma->ewma_filter[0] == 2) || (gma->ewma_filter[0] == 4) || (gma->ewma_filter[0] == 8) || (gma->ewma_filter[0] == 16))
	{
		ewma_init(&average[0], EWMA_FACTOR, gma->ewma_filter[0]);
		dev_info(&gma->gma_client->dev, "ewma_init: gma->ewma_filter[0]=%d \n", gma->ewma_filter[0]);
	}
	if ((gma->ewma_filter[1] == 2) || (gma->ewma_filter[1] == 4) || (gma->ewma_filter[1] == 8) || (gma->ewma_filter[1] == 16))
	{
		ewma_init(&average[1], EWMA_FACTOR, gma->ewma_filter[1]);
		dev_info(&gma->gma_client->dev, "ewma_init: gma->ewma_filter[1]=%d \n", gma->ewma_filter[1]);
	}
	if ((gma->ewma_filter[2] == 2) || (gma->ewma_filter[2] == 4) || (gma->ewma_filter[2] == 8) || (gma->ewma_filter[2] == 16))
	{
		ewma_init(&average[2], EWMA_FACTOR, gma->ewma_filter[2]);
		dev_info(&gma->gma_client->dev, "ewma_init: gma->ewma_filter[2]=%d \n", gma->ewma_filter[2]);
	}
	return count;
}
#endif
static ssize_t gma_register_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int address, value;
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);

    sscanf(buf, "[0x%x]=0x%x", &address, &value);
    
    if (gma_smbus_write_byte(gma->gma_client, (unsigned char)address, value) < 0)
        return -EINVAL;

    return count;
}

static ssize_t gma_register_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);    
    size_t count = 0;
    u8 reg[0x10];
    int i;
    
    // read reg: 0x00 0x01
	for (i = 0 ; i < 0x02; i++) {
		gma_smbus_read_byte(gma->gma_client, GMA1302_REG_PID+i, reg);
		count += sprintf(&buf[count], "0x%x: 0x%02x\n", 0x00+i, reg[0]);
    }
	// read reg: 0x02
	gma_smbus_read_byte(gma->gma_client, GMA1302_REG_ACTR, reg);
	count += sprintf(&buf[count], "0x%x: 0x%02x\n", 2, reg[0]);
	// read reg: 0x03
	gma_smbus_read_byte(gma->gma_client, GMA1302_REG_MTHR, reg);
	count += sprintf(&buf[count], "0x%x: 0x%02x\n", 3, reg[0]);
    // read reg:  0x04 [0x0055]
    gma_smbus_read_byte(gma->gma_client, GMA1302_REG_STADR, reg+i);
    count += sprintf(&buf[count], "0x%x: 0x%02x\n", 0x04, reg[i]);
    // read reg: 0x05 ~ 0x0c
    for (i = 0 ; i < 0x08; i++) {
        gma_smbus_read_byte(gma->gma_client, GMA1302_REG_STATUS+i, reg+i);
        count += sprintf(&buf[count], "0x%x: 0x%02x\n", 0x05+i, reg[i]);
    }
	// read reg: 0x0d
	gma_smbus_read_byte(gma->gma_client, 0x0d, reg);
	count += sprintf(&buf[count], "0x%x: 0x%02x\n", 0x0d, reg[0]);
	// read reg: 0x15 ~ 0x17
    for (i = 0 ; i < 0x03; i++) {
        gma_smbus_read_byte(gma->gma_client, GMA1302_REG_INTCR+i, reg+i);
        count += sprintf(&buf[count], "0x%x: 0x%02x\n", 0x15+i, reg[i]);
    }
	// read reg: 0x18
	gma_smbus_read_byte(gma->gma_client, GMA1302_REG_CONTR3, reg);
	count += sprintf(&buf[count], "0x%x: 0x%02x\n", 0x18, reg[0]);
	// read reg: 0x38
	gma_smbus_read_byte(gma->gma_client, 0x38, reg);
	count += sprintf(&buf[count], "0x%x: 0x%02x\n", 0x38, reg[0]);
    return count;
}

static ssize_t gma_value_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct gma_data *gma = input_get_drvdata(input);
    struct gma_acc acc;

    gma_read_data(gma->gma_client, &acc);
    gma_axis_remap(gma->gma_client, &acc);
    
    return sprintf(buf, "%d %d %d\n", acc.x, acc.y, acc.z);
}

static ssize_t gma_delay_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", atomic_read(&gma->delay));
}

static ssize_t gma_delay_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);

    error = strict_strtoul(buf, 10, &data);
    if (error)
        return error;

    atomic_set(&gma->delay, 1);
    //atomic_set(&gma->delay, (unsigned int) data);
   // dev_dbg(&gma->gma_client->dev, "gma->delay=%d , data=%d \n", gma->delay, data);
    //printk( "gma->delay=%d , data=%d \n", gma->delay, data); 

    return count;
}


static ssize_t gma_enable_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", atomic_read(&gma->enable));
}

static void gma30x_do_enable(struct device *dev, int enable)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);

    if (enable) {
        gma30x_set_mode(gma->gma_client, GMA1302_MODE_RESET);
       
        schedule_delayed_work(&gma->work,
        msecs_to_jiffies(atomic_read(&gma->delay)));
    } else {
        gma30x_set_mode(gma->gma_client, GMA1302_MODE_POWERDOWN);
        cancel_delayed_work_sync(&gma->work);
    }
}

static void gma30x_set_enable(struct device *dev, int enable)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);
    int pre_enable = atomic_read(&gma->enable);

    mutex_lock(&gma->enable_mutex);
    if (enable != pre_enable) {
            gma30x_do_enable(dev, enable);
            atomic_set(&gma->enable, enable);
    }
    mutex_unlock(&gma->enable_mutex);
}

static ssize_t gma_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;

    error = strict_strtoul(buf, 10, &data);
    if (error)
        return error;
    if ((data == 0) || (data == 1))
        gma30x_set_enable(dev, data);

    return count;
}

static ssize_t gma_fuzz_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int data;
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);

    data = atomic_read(&(gma->fuzz));

    return sprintf(buf, "%d\n", data);
}

static ssize_t gma_fuzz_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);

    error = strict_strtol(buf, 10, &data);
    if (error)
        return error;

    atomic_set(&(gma->fuzz), (int) data);
    
    if(gma->input != NULL) {
        gma->input->absinfo[ABS_X].fuzz = data;
        gma->input->absinfo[ABS_Y].fuzz = data;
        gma->input->absinfo[ABS_Z].fuzz = data;
    }
    
    return count;
}

static ssize_t gma_board_position_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int data;
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);

    data = atomic_read(&(gma->position));

    return sprintf(buf, "%d\n", data);
}

static ssize_t gma_board_position_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);

    error = strict_strtol(buf, 10, &data);
    if (error)
        return error;

    atomic_set(&(gma->position), (int) data);

    return count;
}

static ssize_t gma_calibration_run_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int cfg_calibration[3];
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);
    struct gma_acc acc;

    gma_read_data(gma->gma_client, &acc);
    
    gma->offset.x = 0 - acc.x;
    gma->offset.y = 0 - acc.y;
    //if (atomic_read(&gma->position) < 0) {
    if (atomic_read(&gma->position) > 0) {
        gma->offset.z = LSG - acc.z;
    } else {
        gma->offset.z = (-LSG) - acc.z;
    }
    dev_info(&client->dev, "fast calibration: %d %d %d\n", gma->offset.x, gma->offset.y, gma->offset.z);

    cfg_calibration[0] = gma->offset.x;
    cfg_calibration[1] = gma->offset.y;
    cfg_calibration[2] = gma->offset.z;
	dev_info(&client->dev, "run fast calibration finished\n");
    return count;
}

static ssize_t gma_calibration_reset_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int cfg_calibration[3];
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);
    
    memset(&(gma->offset), 0, sizeof(struct gma_acc));
    memset(cfg_calibration, 0, sizeof(cfg_calibration));

    printk(KERN_INFO "reset fast calibration finished\n");
    return count;
}

static ssize_t gma_calibration_value_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);

    return sprintf(buf, "%d %d %d\n", gma->offset.x, 
                                gma->offset.y, gma->offset.z);
}

static ssize_t gma_calibration_value_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int data[3];
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);

    sscanf(buf, "%d %d %d", &data[0], &data[1], &data[2]);
    gma->offset.x = (signed short) data[0];
    gma->offset.y = (signed short) data[1];
    gma->offset.z = (signed short) data[2];
    
    printk(KERN_INFO "set fast calibration finished\n");
    return count;
}
/* sysfs reg_read show & store */
static ssize_t gma_reg_read_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);
	int err;
	unsigned char i2c[6];

	i2c[0] = (unsigned char)atomic_read(&gma->addr);
	err = gma_smbus_read_byte_block(gma->gma_client, i2c[0], i2c, sizeof(i2c));
	
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", i2c[0] , i2c[1], i2c[2] , i2c[3], i2c[4] , i2c[5]);
}

static ssize_t gma_reg_read_store(struct device *dev,
						struct device_attribute *attr,
						char const *buf,
						size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);
	int addr = 0;

	if (NULL == buf)
		return -EINVAL;
	
	if (0 == count)
		return 0;

	if (false == get_value_as_int(buf, count, &addr))
		return -EINVAL;
	dev_info(&gma->gma_client->dev, "addr=%d  \n", addr);
	if (addr < 0 || 128 < addr)
		return -EINVAL;

	atomic_set(&gma->addr, addr);

	return count;
}
/* sysfs reg_write show & store */
static ssize_t gma_reg_write_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);
	int err;
	unsigned char i2c[6];

	i2c[0] = (unsigned char)atomic_read(&gma->addr);
	err = gma_smbus_read_byte_block(gma->gma_client, i2c[0], i2c, sizeof(i2c));
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", i2c[0] , i2c[1], i2c[2] , i2c[3], i2c[4] , i2c[5]);
}

static ssize_t gma_reg_write_store(struct device *dev,
						struct device_attribute *attr,
						char const *buf,
						size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *gma = i2c_get_clientdata(client);
	int value = 0;
	unsigned char buffer[2];
	if (NULL == buf)
		return -EINVAL;
	
	if (0 == count)
		return 0;

	if (false == get_value_as_int(buf, count, &value))
		return -EINVAL;
	dev_dbg(&gma->gma_client->dev, "value=%d  \n", value);
	if (value < 0 || 256 < value)
		return -EINVAL;

	/* set value to reg */
	buffer[1] = (unsigned char)atomic_read(&gma->addr);
	buffer[0] = value;
	gma_smbus_write_byte_block(gma->gma_client, buffer[1], buffer, 1);
	//dev_info(&gma->gma_client->dev, "buffer[0]=%d, buffer[1]=%d\n", buffer[0], buffer[1]);

	return count;
}
#ifdef SMA_FILTER
static DEVICE_ATTR(sma, S_IRUGO|S_IWUGO,
		gma_sma_show, gma_sma_store);
#endif
#ifdef EWMA_FILTER
static DEVICE_ATTR(ewma, S_IRUGO|S_IWUGO,
		gma_ewma_show, gma_ewma_store);
#endif
static DEVICE_ATTR(reg, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        gma_register_show, gma_register_store);
static DEVICE_ATTR(value, S_IRUGO,
        gma_value_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        gma_delay_show, gma_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        gma_enable_show, gma_enable_store);
static DEVICE_ATTR(fuzz, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        gma_fuzz_show, gma_fuzz_store);
static DEVICE_ATTR(board_position, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        gma_board_position_show, gma_board_position_store);
static DEVICE_ATTR(calibration_run, S_IWUSR|S_IWGRP|S_IWOTH,
        NULL, gma_calibration_run_store);
static DEVICE_ATTR(calibration_reset, S_IWUSR|S_IWGRP|S_IWOTH,
        NULL, gma_calibration_reset_store);
static DEVICE_ATTR(calibration_value, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        gma_calibration_value_show, gma_calibration_value_store);
static DEVICE_ATTR(reg_rx, S_IRUGO|S_IWUGO,
		gma_reg_read_show, gma_reg_read_store);
static DEVICE_ATTR(reg_tx, S_IRUGO|S_IWUGO,
		gma_reg_write_show, gma_reg_write_store);
static struct attribute *gma_attributes[] = {
#ifdef SMA_FILTER
	&dev_attr_sma.attr,
#endif /* Simple Moving Average */
#ifdef EWMA_FILTER
	&dev_attr_ewma.attr,
#endif
    &dev_attr_reg.attr,
    &dev_attr_value.attr,
    &dev_attr_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_fuzz.attr,
    &dev_attr_board_position.attr,
    &dev_attr_calibration_run.attr,
    &dev_attr_calibration_reset.attr,
    &dev_attr_calibration_value.attr,
	&dev_attr_reg_rx.attr,
	&dev_attr_reg_tx.attr,
    NULL
};

static struct attribute_group gma_attribute_group = {
    .attrs = gma_attributes
};

static int gma_read_file(char *path, char *buf, int size)
{
    struct file *filp;
    loff_t len, offset;
    int ret=0;
    mm_segment_t fs;

    filp = filp_open(path, O_RDWR, 0777);
    if (IS_ERR(filp)) {
        ret = PTR_ERR(filp);
        goto out;
    }

    len = vfs_llseek(filp, 0, SEEK_END);
    if (len > size) {
        len = size;
    }
    
    offset = vfs_llseek(filp, 0, SEEK_SET);

    fs=get_fs();
    set_fs(KERNEL_DS);

    ret = vfs_read(filp, (char __user *)buf, (size_t)len, &(filp->f_pos));

    set_fs(fs);

    filp_close(filp, NULL);    
out:
    return ret;
}

static int GMA_WriteCalibration(struct gma_data *gma, char * offset){
	char w_buf[20] = {0};
	struct file *fp;
	mm_segment_t fs;
	ssize_t ret;

	sprintf(w_buf,"%d %d %d", gma->offset.x, gma->offset.y, gma->offset.z);
	dev_err(&gma->gma_client->dev, "%d %d %d", gma->offset.x, gma->offset.y, gma->offset.z);
	/* Set segment descriptor associated to kernel space */
	fp = filp_open(offset, O_RDWR | O_CREAT, 0777);
	if(IS_ERR(fp))
		dev_err(&gma->gma_client->dev, "filp_open %s error!!.\n", offset);
	else{
		fs = get_fs();
		//set_fs(KERNEL_DS);
		set_fs(get_ds());
		dev_info(&gma->gma_client->dev, "filp_open %s SUCCESS!!.\n", offset);
 		ret = fp->f_op->write(fp,w_buf,20,&fp->f_pos);
		filp_close(fp,NULL);
		set_fs(fs);
	}

	return 0;
}

static int gma_load_user_calibration(struct i2c_client *client)
{
    char buffer[16];
    int ret = 0;
    int data[SENSOR_DATA_SIZE];
    struct gma_data *gma = i2c_get_clientdata(client);    
    int calibrated = atomic_read(&gma->calibrated);
#ifdef AutoZeroZ /* Need GlobalMems FAE support. */
	int cfg_calibration[SENSOR_DATA_SIZE];
	static int ShrinkRange = Gravity4_0; 
	static int ShrinkRangeX = LSG; 
	static int ShrinkRangeY = LSG; 
	struct gma_acc acc;
	int i, avg[SENSOR_DATA_SIZE];
	long xyz_acc[SENSOR_DATA_SIZE];
	/* initialize the accumulation buffer */
  	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		xyz_acc[i] = 0;
#endif	
    // only calibrate once
    if (calibrated) {
        goto usr_calib_end;
    } else {
        atomic_set(&gma->calibrated, 1);
    }

    ret = gma_read_file(CFG_GSENSOR_CALIBFILE, buffer, sizeof(buffer));
    if (ret < 0) {
        printk(KERN_ERR "gsensor calibration file not exist!\n");
#ifdef AutoZeroZ /* Need GlobalMems FAE support. */
	for(i = 0; i < AVG_NUM; i++) {
		gma_read_data(gma->gma_client, &acc);
		xyz_acc[0] += acc.x;
		xyz_acc[1] += acc.y;
		xyz_acc[2] += acc.z;
  	}
	/* calculate averages */
  	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		avg[i] = xyz_acc[i] / AVG_NUM;
	dev_info(&client->dev, "acc: %d %d %d\n", avg[0], avg[1], avg[2]);
	/* 1.Condition: Analyzing horizontal state */
	//if( ABS(avg[0]) < ShrinkRange && ABS(avg[1]) < ShrinkRange){
		if(1){
		/* 2.Calculate new offset */
		gma->offset.x = 0 - avg[0];
		gma->offset.y = 0 - avg[1];
		//if (atomic_read(&gma->position) < 0) {
		if (atomic_read(&gma->position) > 0) {
			gma->offset.z = LSG - avg[2];
		} else 
			gma->offset.z = (-LSG) - avg[2];
		
		dev_info(&client->dev, "fast calibration: %d %d %d\n", gma->offset.x, gma->offset.y, gma->offset.z);
	}

#endif
    cfg_calibration[0] = gma->offset.x;
    cfg_calibration[1] = gma->offset.y;
    cfg_calibration[2] = gma->offset.z;
	/* 3. Not implemented : Please do save cfg_calibration to gsensor_calib.txt */
	dev_info(&client->dev, "ShrinkRange =%d, Auto calibration finished\n", ShrinkRange);
	/* 4. Save offset */
	msleep(5000);
	GMA_WriteCalibration(gma , CFG_GSENSOR_CALIBFILE);

        goto usr_calib_end;
    }
    
    sscanf(buffer, "%d %d %d", &data[0], &data[1], &data[2]);
    gma->offset.x = (signed short) data[0];
    gma->offset.y = (signed short) data[1];
    gma->offset.z = (signed short) data[2];
    
    printk(KERN_INFO "user cfg_calibration: %d %d %d\n", data[0], data[1], data[2]);
    
usr_calib_end:
    return ret;
}

static int gma_axis_remap(struct i2c_client *client, struct gma_acc *acc)
{
    s16 swap;
    struct gma_data *gma = i2c_get_clientdata(client);
    int position = atomic_read(&gma->position);

    switch (abs(position)) {
        case 1:
            break;
        case 2:
            swap = acc->x;
            acc->x = acc->y;
            acc->y = -swap; 
            break;
        case 3:
            acc->x = -(acc->x);
            acc->y = -(acc->y);
            break;
        case 4:
            swap = acc->x;
            acc->x = -acc->y;
            acc->y = swap;
            break;
    }
    
    if (position < 0) {
        acc->z = -(acc->z);
        acc->x = -(acc->x);
    }
    
    return 0;
}

static void gma_work_func(struct work_struct *work)
{
    struct gma_data *gma = container_of((struct delayed_work *)work,
            struct gma_data, work);
    static struct gma_acc acc;
	
    int result;
    unsigned long delay = msecs_to_jiffies(atomic_read(&gma->delay));
    
    gma_load_user_calibration(gma->gma_client);
    
    result = gma_read_data(gma->gma_client, &acc);
    acc.x += gma->offset.x;
    acc.y += gma->offset.y;
    acc.z += gma->offset.z;

    if (result == 0) {
        gma_axis_remap(gma->gma_client, &acc);

/******* XYZ +- exchange fn start ******/
	//acc.x = -acc.x; //X axis
	//acc.y = -acc.y;	//Y axis
	//acc.z = -acc.z;	//Z axis
/******* XYZ +- exchange fn end ********/
	/* Analyzing horizontal state 2015-05-16 add */
	if( ABS(acc.x) < 42 && ABS(acc.y) < 42)
	{
		acc.x = 0;
		acc.y = 0;
	}
    dev_dbg(&gma->gma_client->dev, "after  %d,%d,%d !\n",acc.x,acc.y,acc.z);
    input_report_abs(gma->input, ABS_X, -acc.x);
    input_report_abs(gma->input, ABS_Y, -acc.y);
    input_report_abs(gma->input, ABS_Z, acc.z);
    input_sync(gma->input);
    }
    schedule_delayed_work(&gma->work, delay);
}

static int gma30x_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int err = 0;
#ifdef SMA_FILTER
	int j;
#endif
    struct gma_data *data;
    struct input_dev *dev;
    int cfg_position;
    int cfg_calibration[3];
	
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
        goto exit;
    }
    data = kzalloc(sizeof(struct gma_data), GFP_KERNEL);
    if (!data) {
        err = -ENOMEM;
        goto exit;
    }

    i2c_set_clientdata(client, data);
    data->gma_client = client;
    mutex_init(&data->enable_mutex);

    INIT_DELAYED_WORK(&data->work, gma_work_func);
    atomic_set(&data->delay, MAX_DELAY);
    atomic_set(&data->enable, 0);

#if CFG_GSENSOR_USE_CONFIG > 0
        /*get xml cfg*/
        err = get_config(CFG_GSENSOR_POSITION, (char *)(&cfg_position), sizeof(int));
        if (err != 0) {
			dev_err(&client->dev, "get position %d fail\n", cfg_position);
            goto kfree_exit;
        }
#else
        cfg_position = 3;
#endif
    atomic_set(&data->position, cfg_position);
    atomic_set(&data->calibrated, 0);
    atomic_set(&data->fuzz, FUZZ);
        
    //power on init regs    
    err = gma30x_hw_init(data->gma_client); 
    if (err < 0) {
		dev_err(&client->dev, "gma30x probe fail! err:%d\n", err);
        goto kfree_exit;
    }

    dev = input_allocate_device();
    if (!dev)
        return -ENOMEM;
    dev->name = SENSOR_NAME;
    dev->id.bustype = BUS_I2C;

    input_set_capability(dev, EV_ABS, ABS_MISC);
    input_set_abs_params(dev, ABS_X, ABSMIN, ABSMAX, FUZZ, 0);
    input_set_abs_params(dev, ABS_Y, ABSMIN, ABSMAX, FUZZ, 0);
    input_set_abs_params(dev, ABS_Z, ABSMIN, ABSMAX, FUZZ, 0);
    input_set_drvdata(dev, data);

    err = input_register_device(dev);
    if (err < 0) {
        input_free_device(dev);
        goto kfree_exit;
    }

    data->input = dev;

    err = sysfs_create_group(&data->input->dev.kobj,
            &gma_attribute_group);
    if (err < 0)
        goto error_sysfs;
#ifdef SMA_FILTER
	for(i = 0; i < SENSOR_DATA_SIZE; ++i){
		data->sum[i] = 0;
		for(j = 0; j < SMA_AVG; ++j)
			data->bufferave[i][j] = 0;
	}
	data->sma_filter = SMA_AVG;
	dev_info(&client->dev, "GMA30x_DEFAULT_FILTER: %d\n", data->sma_filter);
#endif	
#ifdef CONFIG_HAS_EARLYSUSPEND
    data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    data->early_suspend.suspend = gma_early_suspend;
    data->early_suspend.resume = gma_early_resume;
    register_early_suspend(&data->early_suspend);
#endif

#if CFG_GSENSOR_USE_CONFIG > 0
    /*get xml cfg*/
    err = get_config(CFG_GSENSOR_CALIBRATION, (char *)cfg_calibration, sizeof(cfg_calibration));
    if (err != 0) {
		dev_err(&client->dev, "get calibration fail\n");
        goto error_sysfs;
    }
#else
    memset(cfg_calibration, 0, sizeof(cfg_calibration));
#endif    
    
    data->offset.x = (signed short) cfg_calibration[0];
    data->offset.y = (signed short) cfg_calibration[1];
    data->offset.z = (signed short) cfg_calibration[2];
        
    return 0;

error_sysfs:
    input_unregister_device(data->input);

kfree_exit:
    kfree(data);
exit:
    return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gma_early_suspend(struct early_suspend *h)
{
    // sensor hal will disable when early suspend
}


static void gma_early_resume(struct early_suspend *h)
{
    // sensor hal will enable when early resume
}
#endif

static int  gma30x_remove(struct i2c_client *client)
{
    struct gma_data *data = i2c_get_clientdata(client);

    gma30x_set_enable(&client->dev, 0);
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&data->early_suspend);
#endif
    sysfs_remove_group(&data->input->dev.kobj, &gma_attribute_group);
    input_unregister_device(data->input);
    kfree(data);

    return 0;
}

#ifdef CONFIG_PM

static int gma30x_suspend(struct device *dev)
{
    gma30x_do_enable(dev, 0);
    
    return 0;
}

static int gma30x_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct gma_data *data = i2c_get_clientdata(client);
    
    //power on init regs    
    gma30x_hw_init(data->gma_client);     
    gma30x_do_enable(dev, atomic_read(&data->enable));
    
    return 0;
}

#else

#define gma30x_suspend        NULL
#define gma30x_resume        NULL

#endif /* CONFIG_PM */

static SIMPLE_DEV_PM_OPS(gma_pm_ops, gma30x_suspend, gma30x_resume);

static const unsigned short  gma_addresses[] = {
    SENSOR_I2C_ADDR,
    I2C_CLIENT_END,
};

static const struct i2c_device_id gma_id[] = {
    { SENSOR_NAME, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, gma_id);

static struct i2c_driver gma30x_driver = {
    .driver = {
        .owner    = THIS_MODULE,
        .name    = SENSOR_NAME,
        .pm    = &gma_pm_ops,
    },
    .class        = I2C_CLASS_HWMON,
//    .address_list    = gma_addresses,
    .id_table    = gma_id,
    .probe        = gma30x_probe,
    //.remove        = gma30x_remove,

};

static struct i2c_board_info gma_board_info={
    .type = SENSOR_NAME, 
    .addr = SENSOR_I2C_ADDR,
};

static struct i2c_client *gma_client;

static int __init gma30x_init(void)
{
    struct i2c_adapter *i2c_adap;
    unsigned int cfg_i2c_adap_id;

#if CFG_GSENSOR_USE_CONFIG > 0
    int ret;
    
    /*get xml cfg*/
    ret = get_config(CFG_GSENSOR_ADAP_ID, (char *)(&cfg_i2c_adap_id), sizeof(unsigned int));
    if (ret != 0) {
        printk(KERN_ERR"get i2c_adap_id %d fail\n", cfg_i2c_adap_id);
        return ret;
    }
#else
    cfg_i2c_adap_id = 2;//2; need check i2c bus
#endif
    
    i2c_adap = i2c_get_adapter(cfg_i2c_adap_id);  
    gma_client = i2c_new_device(i2c_adap, &gma_board_info);  
    i2c_put_adapter(i2c_adap);
    GSE_LOG("ACC driver: initialize.\n");
    return i2c_add_driver(&gma30x_driver);
}

static void __exit gma30x_exit(void)
{
    i2c_unregister_device(gma_client);
    i2c_del_driver(&gma30x_driver);
}

MODULE_AUTHOR("Zhining Song <songzhining@actions-semi.com>");
MODULE_DESCRIPTION("GMA302/GMA303 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");

module_init(gma30x_init);
module_exit(gma30x_exit);


