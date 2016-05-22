/*
 *  mag3110.c - Linux kernel modules for 3-Axis Magnetic sensor
 *  Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>

#define	SENSOR_IOCTL_BASE			'S'
#define	SENSOR_GET_MODEL_NAME		_IOR(SENSOR_IOCTL_BASE, 0, char *)
#define	SENSOR_GET_POWER_STATUS		_IOR(SENSOR_IOCTL_BASE, 2, int)
#define	SENSOR_SET_POWER_STATUS		_IOR(SENSOR_IOCTL_BASE, 3, int)
#define	SENSOR_GET_DELAY_TIME		_IOR(SENSOR_IOCTL_BASE, 4, int)
#define	SENSOR_SET_DELAY_TIME		_IOR(SENSOR_IOCTL_BASE, 5, int)
#define	SENSOR_GET_RAW_DATA			_IOR(SENSOR_IOCTL_BASE, 6, short[3])

#define MAG3110_I2C_ADDR        		0x0E
#define MAG3110_ID                      0xC4


#define MAG3110_POSITION_DEFAULT	5
#define MAG3110_DELAY_DEFAULT		200

#define MAG3110_BUF_SIZE   			6

/* register enum for mag3110 registers */
enum {
	MAG3110_DR_STATUS = 0x00,
	MAG3110_OUT_X_MSB,
	MAG3110_OUT_X_LSB,
	MAG3110_OUT_Y_MSB,
	MAG3110_OUT_Y_LSB,
	MAG3110_OUT_Z_MSB,
	MAG3110_OUT_Z_LSB,
	MAG3110_WHO_AM_I,

	MAG3110_OFF_X_MSB,
	MAG3110_OFF_X_LSB,
	MAG3110_OFF_Y_MSB,
	MAG3110_OFF_Y_LSB,
	MAG3110_OFF_Z_MSB,
	MAG3110_OFF_Z_LSB,

	MAG3110_DIE_TEMP,

	MAG3110_CTRL_REG1 = 0x10,
	MAG3110_CTRL_REG2,
};


enum {
	STANDBY = 0,
	ACTIVED,
};
struct mag3110_data_axis {
	short x;
	short y;
	short z;
};
struct mag3110_data {
	struct i2c_client *client;
	atomic_t active;
	atomic_t delay;
	atomic_t position;
	u8 chip_id;
};
static struct mag3110_data *g_mag3110_data = NULL;

static short mag3110_position_setting[8][3][3] =
{
	{{ 0, -1,  0}, { 1,  0,  0}, {0, 0, -1}},
	{{-1,  0,  0}, { 0, -1,  0}, {0, 0, -1}},
	{{ 0,  1,  0}, {-1,  0,  0}, {0, 0, -1}},
	{{ 1,  0,  0}, { 0,  1,  0}, {0, 0, -1}},
	
	{{ 0, -1,  0}, {-1,  0,  0}, {0, 0,  1}},
	{{-1,  0,  0}, { 0,  1,  0}, {0, 0,  1}},
	{{ 0,  1,  0}, { 1,  0,  0}, {0, 0,  1}},
	{{ 1,  0,  0}, { 0, -1,  0}, {0, 0,  1}},

};

static int mag3110_data_convert(struct mag3110_data *pdata,
		struct mag3110_data_axis *axis_data)
{
	short rawdata[3], data[3];
	int i, j;
	int position = atomic_read(&pdata->position);

	if (position < 0 || position > 7)
		position = 0;
	rawdata[0] = axis_data->x;
	rawdata[1] = axis_data->y;
	rawdata[2] = axis_data->z;
	for (i = 0; i < 3; i++) {
		data[i] = 0;
		for (j = 0; j < 3; j++)
			data[i] += rawdata[j] * mag3110_position_setting[position][i][j];
	}
	axis_data->x = data[0];
	axis_data->y = data[1];
	axis_data->z = data[2];
	return 0;
}
static int mag3110_device_init(struct i2c_client *client)
{
	int val, ret;
	struct mag3110_data *pdata = i2c_get_clientdata(client);
	/* enable automatic resets */
	val = 0x80;
	ret = i2c_smbus_write_byte_data(client, MAG3110_CTRL_REG2, val);

	/* set default data rate to 80HZ */
	val = i2c_smbus_read_byte_data(client, MAG3110_CTRL_REG1);
	val |= (0x0 << 5);
	ret = i2c_smbus_write_byte_data(client, MAG3110_CTRL_REG1, val);
	atomic_set(&pdata->active,STANDBY);
	return ret;
}

static int mag3110_change_mode(struct i2c_client *client, int mode)
{
	u8 val;
	int ret;
	val = i2c_smbus_read_byte_data(client, MAG3110_CTRL_REG1);
	if(mode == ACTIVED)
		val |= 0x01;		
	 else
		val &= (~0x01);
	ret = i2c_smbus_write_byte_data(client, MAG3110_CTRL_REG1,val); 
	return ret;
}
static int mag3110_set_delay(struct i2c_client *client, int delay)
{
	return 0;
}
static int mag3110_read_data(struct mag3110_data *pdata,
		struct mag3110_data_axis *data)
{
    struct i2c_client * client = pdata->client;

	u8 tmp_data[MAG3110_BUF_SIZE];
	int ret;
	ret = i2c_smbus_read_i2c_block_data(client, MAG3110_OUT_X_MSB,
									MAG3110_BUF_SIZE, tmp_data);
	if (ret < MAG3110_BUF_SIZE) {
		dev_err(&client->dev, "i2c block read failed\n");
		return -EIO;
	}
	data->x = ((tmp_data[0] << 8) & 0xff00) | tmp_data[1];
	data->y = ((tmp_data[2] << 8) & 0xff00) | tmp_data[3];
	data->z = ((tmp_data[4] << 8) & 0xff00) | tmp_data[5];
	return 0;
}

//mag3110 miscdevice
static long mag3110_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct mag3110_data *pdata = file->private_data;
	void __user *argp = (void __user *)arg;	
    long ret = 0;
	short sdata[3];
	int enable;
	int delay;
	struct mag3110_data_axis data;
	if(!pdata){
		printk(KERN_ERR "MAG3110 struct datt point is NULL.");
		return -EFAULT;
	}
	switch (cmd) {
		case SENSOR_GET_MODEL_NAME:
			if(copy_to_user(argp,"MAG3110 MAG",strlen("MAG3110 MAG") + 1))
			{
				printk(KERN_ERR "SENSOR_GET_MODEL_NAME copy_to_user failed.");
				ret = -EFAULT;
			}
			break;
		case SENSOR_GET_POWER_STATUS:
			enable = atomic_read(&pdata->active);
			if(copy_to_user(argp,&enable,sizeof(int)))
			{
				printk(KERN_ERR "SENSOR_SET_POWER_STATUS copy_to_user failed.");
				ret = -EFAULT;
			}
			break;
		case SENSOR_SET_POWER_STATUS:
			if(copy_from_user(&enable,argp,sizeof(int)))
			{
				printk(KERN_ERR "SENSOR_SET_POWER_STATUS copy_to_user failed.");
				ret = -EFAULT;
			}
			if(pdata->client){
				ret = mag3110_change_mode(pdata->client,enable? ACTIVED : STANDBY);
				if(!ret)
					atomic_set(&pdata->active,enable);
			}
			break;
		case SENSOR_GET_DELAY_TIME:
			delay = atomic_read(&pdata->delay);
			if(copy_to_user(argp, &delay, sizeof(delay)))
			{
				printk(KERN_ERR "SENSOR_GET_DELAY_TIME copy_to_user failed.");
				return -EFAULT;
			}
			break;
		case SENSOR_SET_DELAY_TIME:
			if(copy_from_user(&delay,argp,sizeof(int)));
			{
				printk(KERN_ERR "SENSOR_GET_DELAY_TIME copy_to_user failed.");
				ret = -EFAULT;
			}
			if(pdata->client && delay > 0 && delay <= 500){
				ret = mag3110_set_delay(pdata->client,delay);
				if(!ret)
					atomic_set(&pdata->delay,delay);
			}
			break;
		case SENSOR_GET_RAW_DATA:
			ret = mag3110_read_data(pdata,&data);
			if(!ret){
				mag3110_data_convert(pdata,&data);
				sdata[0] = data.x;
				sdata[1] = data.y;
				sdata[2] = data.z;
				if(copy_to_user(argp,sdata,sizeof(sdata)))
				{
					printk(KERN_ERR "SENSOR_GET_RAW_DATA copy_to_user failed.");
					ret = -EFAULT;
				}
			}
			break;
		default:
			ret = -1;
	}
	return ret;
}

static int mag3110_open(struct inode *inode, struct file *file)
{
	file->private_data = g_mag3110_data;
	return nonseekable_open(inode, file);
}

static int mag3110_release(struct inode *inode, struct file *file)
{
	/* note: releasing the wdt in NOWAYOUT-mode does not stop it */
	return 0;
}

static const struct file_operations mag3110_fops = {
	.owner = THIS_MODULE,
	.open = mag3110_open,
	.release = mag3110_release,
	.unlocked_ioctl = mag3110_ioctl,
};

static struct miscdevice mag3110_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "FreescaleMagnetometer",
	.fops = &mag3110_fops,
};

static ssize_t mag3110_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mag3110_data *pdata = g_mag3110_data;
	int enable = 0;
	enable = atomic_read(&pdata->active);
	return sprintf(buf, "%d\n", enable);
}

static ssize_t mag3110_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct mag3110_data *pdata = g_mag3110_data;
	struct i2c_client *client = pdata->client;
	int ret;
	unsigned long enable;
	enable = simple_strtoul(buf, NULL, 10);
	enable = (enable > 0) ? 1 : 0;
	ret = mag3110_change_mode(client,(enable > 0 ? ACTIVED : STANDBY));
	if (!ret) {
		atomic_set(&pdata->active,enable);
		printk(KERN_INFO"mag enable setting active \n");
	}
	return count;
}

static ssize_t mag3110_poll_delay_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mag3110_data *pdata = g_mag3110_data;
	int delay = 0;
	delay = atomic_read(&pdata->delay);
	return sprintf(buf, "%d\n", delay);
}

static ssize_t mag3110_poll_delay_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct mag3110_data *pdata = g_mag3110_data;
	struct i2c_client *client = pdata->client;
	int ret;
	int delay;
	delay = simple_strtoul(buf, NULL, 10);
	ret = mag3110_set_delay(client,delay);
	if(!ret)
		atomic_set(&pdata->delay, delay);
	return count;
}

static ssize_t mag3110_position_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct mag3110_data *pdata = g_mag3110_data;
	int position = 0;
	position = atomic_read(&pdata->position);
	return sprintf(buf, "%d\n", position);
}

static ssize_t mag3110_position_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct mag3110_data *pdata = g_mag3110_data;
	int position;
	position = simple_strtoul(buf, NULL, 10);
	atomic_set(&pdata->position,position);
	return count;
}

static ssize_t mag3110_data_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct mag3110_data *pdata = g_mag3110_data;
	int ret = 0;
	struct mag3110_data_axis data;
	ret = mag3110_read_data(pdata,&data);
	if(!ret)
		mag3110_data_convert(pdata,&data);
	return sprintf(buf, "%d,%d,%d\n",data.x,data.y,data.z);
}

static DEVICE_ATTR(enable, 0666, mag3110_enable_show, mag3110_enable_store);

static DEVICE_ATTR(poll_delay, 0666,mag3110_poll_delay_show, mag3110_poll_delay_store);

static DEVICE_ATTR(position, 0666,mag3110_position_show, mag3110_position_store);

static DEVICE_ATTR(data, 0666,mag3110_data_show, NULL);


static struct attribute *mag3110_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_position.attr,
	&dev_attr_data.attr,
	NULL
};

static const struct attribute_group mag3110_attr_group = {
	.attrs	= mag3110_attributes,
};

static int mag3110_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result, chip_id;
	struct mag3110_data *pdata;
	struct i2c_adapter *adapter;
	
	adapter = to_i2c_adapter(client->dev.parent);
	result = i2c_check_functionality(adapter,
					 I2C_FUNC_SMBUS_BYTE |
					 I2C_FUNC_SMBUS_BYTE_DATA);
	if (!result)
		goto err_out;

	chip_id = i2c_smbus_read_byte_data(client, MAG3110_WHO_AM_I);

	
	if (chip_id != MAG3110_ID) {
		dev_err(&client->dev, "read sensor who am i (0x%x)error !\n",chip_id);
		result = -EINVAL;
		goto err_out;
	}
	pdata = kzalloc(sizeof(struct mag3110_data), GFP_KERNEL);
	if (!pdata) {
		result = -ENOMEM;
		dev_err(&client->dev, "alloc data memory error!\n");
		goto err_out;
	}
	/* Initialize the MAG3110 chip */
	g_mag3110_data = pdata;
	pdata->client = client;
	pdata->chip_id = chip_id;
	atomic_set(&pdata->delay,MAG3110_DELAY_DEFAULT);
	atomic_set(&pdata->position,MAG3110_POSITION_DEFAULT);
	i2c_set_clientdata(client, pdata);
	result = misc_register(&mag3110_device);
	if (result != 0) {
		printk(KERN_ERR "register acc miscdevice error");
		goto err_regsiter_misc;
	}
	
	result = sysfs_create_group(&mag3110_device.this_device->kobj, &mag3110_attr_group);
	if (result) {
		dev_err(&client->dev, "create device file failed!\n");
		result = -EINVAL;
		goto err_create_sysfs;
	}
	mag3110_device_init(client);
	printk(KERN_INFO"mag3110 device driver probe successfully\n");
	return 0;
err_create_sysfs:
	misc_deregister(&mag3110_device);
err_regsiter_misc:
	kfree(pdata);
err_out:
	return result;
}
static int mag3110_remove(struct i2c_client *client)
{
	struct mag3110_data *pdata = i2c_get_clientdata(client);
	mag3110_change_mode(client,STANDBY);
	misc_deregister(&mag3110_device);
	if(pdata != NULL)
		kfree(pdata);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mag3110_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mag3110_data *pdata = i2c_get_clientdata(client);

	if (atomic_read(&pdata->active))		
		mag3110_change_mode(client,STANDBY);		
	return 0;
}

static int mag3110_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mag3110_data *pdata = i2c_get_clientdata(client);
	
	if (atomic_read(&pdata->active)) 
		mag3110_change_mode(client,ACTIVED);
	return 0;
}
#endif

static const struct i2c_device_id mag3110_id[] = {
	{ "mag3110", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mag3110_id);

static SIMPLE_DEV_PM_OPS(mag3110_pm_ops, mag3110_suspend, mag3110_resume);
static struct i2c_driver mag3110_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver		= {
		.name	= "mag3110",
		.owner	= THIS_MODULE,
		.pm	= &mag3110_pm_ops,
	},
	.probe		= mag3110_probe,
	.remove		= mag3110_remove,
	.id_table	= mag3110_id,
};

static int mag3110_init(void)
{
	/* register driver */
	int res;

	res = i2c_add_driver(&mag3110_driver);
	if (res < 0) {
		printk(KERN_INFO "add mag3110 i2c driver failed\n");
		return -ENODEV;
	}
	return res;
}

static void mag3110_exit(void)
{
	i2c_del_driver(&mag3110_driver);
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MAG3110 3-Axis Gyrosope Sensor driver");
MODULE_LICENSE("GPL");

module_init(mag3110_init);
module_exit(mag3110_exit);
