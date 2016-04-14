/* Sensirion SHT3x-DIS humidity and temperature sensor driver.
 * The SHT3x comes in many different versions, this driver is for the
 * I2C version only.
 *
 * Copyright (C) 2015 Sensirion AG, Switzerland
 * Author: David Frey <david.frey@sensirion.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input-polldev.h>
#include "sht3x.h"

#define ABS_TEMPTERAURE				ABS_GAS
#define ABS_HUMIDITY				ABS_MISC
#define SHT3X_ACTIVED    		 		1
#define SHT3X_STANDBY 				0
#define POLL_INTERVAL_MAX			500
#define POLL_INTERVAL				100
#define POLL_INTERVAL_MIN			1

/* commands */
static const unsigned char sht3x_cmd_measure_blocking[]    = { 0x2c, 0x06 };
static const unsigned char sht3x_cmd_measure_nonblocking[] = { 0x24, 0x00 };
static const unsigned char sht3x_cmd_read_status_reg[] = { 0xF3, 0x2D };

/* delay for non-blocking i2c command, in us */
#define SHT3X_NONBLOCKING_WAIT_TIME  15000

#define SHT3X_CMD_LENGTH      2
#define SHT3X_RESPONSE_LENGTH 6

struct sht3x_data {
	struct i2c_client *client;
	struct input_polled_dev *poll_dev;
	struct mutex update_lock;
	int active;
	bool valid;
	unsigned long last_updated; /* in jiffies */

	const unsigned char *command;

	struct sht3x_platform_data setup;

	int temperature; /* 1000 * temperature in dgr C */
	int humidity; /* 1000 * relative humidity in %RH */
};

static int sht3x_update_values(struct i2c_client *client,
			       struct sht3x_data *data,
			       char *buf, int bufsize)
{
	int ret = i2c_master_send(client, data->command, SHT3X_CMD_LENGTH);
	if (ret != SHT3X_CMD_LENGTH) {
		dev_err(&client->dev, "failed to send command: %d\n", ret);
		return ret < 0 ? ret : -EIO;
	}

	/*
	 * In blocking mode (clock stretching mode) the I2C bus
	 * is blocked for other traffic, thus the call to i2c_master_recv()
	 * will wait until the data is ready. For non blocking mode, we
	 * have to wait ourselves.
	 */
	if (!data->setup.blocking_io)
		usleep_range(SHT3X_NONBLOCKING_WAIT_TIME,
			     SHT3X_NONBLOCKING_WAIT_TIME + 1000);

	ret = i2c_master_recv(client, buf, bufsize);
	if (ret != bufsize) {
		dev_err(&client->dev, "failed to read values: %d\n", ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

/* sysfs attributes */
static struct sht3x_data *sht3x_update_client(struct sht3x_data *data)
{
	struct i2c_client *client = data->client;
	unsigned char buf[SHT3X_RESPONSE_LENGTH];
	int val;
	int ret = 0;

	mutex_lock(&data->update_lock);

	if (time_after(jiffies, data->last_updated + HZ / 10) || !data->valid) {
		ret = sht3x_update_values(client, data, buf, sizeof(buf));
		if (ret)
			goto out;

		/*
		 * From datasheet:
		 * T = -45 + 175 * ST / 2^16
		 * RH = 100 * SRH / 2^16
		 *
		 * Adapted for integer fixed point (3 digit) arithmetic.
		 */
		val = be16_to_cpup((__be16 *)buf);
		data->temperature = ((21875 * val) >> 13) - 45000;
		val = be16_to_cpup((__be16 *)(buf + 3));
		data->humidity = ((12500 * val) >> 13);

		data->last_updated = jiffies;
		data->valid = true;
	}

out:
	mutex_unlock(&data->update_lock);

	return ret == 0 ? data : ERR_PTR(ret);
}

static ssize_t sht3x_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int val;

	struct input_polled_dev *poll_dev = dev_get_drvdata(dev);
	struct sht3x_data *pdata = (struct sht3x_data *)(poll_dev->private);
	mutex_lock(&pdata->update_lock);
	val = pdata->active;
	mutex_unlock(&pdata->update_lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t sht3x_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int ret, enable;
	u8 val;
	struct input_polled_dev *poll_dev = dev_get_drvdata(dev);
	struct sht3x_data *pdata = (struct sht3x_data *)(poll_dev->private);

	enable = simple_strtoul(buf, NULL, 10);
	mutex_lock(&pdata->update_lock);
	if (enable && pdata->active == SHT3X_STANDBY) {
		pdata->active = SHT3X_ACTIVED;
		printk("sht3x set active\n");
	} else if (!enable && pdata->active == SHT3X_ACTIVED) {
		pdata->active = SHT3X_STANDBY;
		printk("sht3x set inactive\n");
	}
	mutex_unlock(&pdata->update_lock);

	return count;
}
static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO, sht3x_enable_show, sht3x_enable_store);

static struct attribute *sht3x_attrs[] = {
	&dev_attr_enable.attr,
	NULL
};

static const struct attribute_group sht3x_attr_group = {
	.attrs = sht3x_attrs,
};

static void sht3x_select_command(struct sht3x_data *data)
{
	data->command = data->setup.blocking_io ?
			sht3x_cmd_measure_blocking :
			sht3x_cmd_measure_nonblocking;
}

static void report_abs(struct sht3x_data *pdata)
{
	struct input_dev *idev;
	int humidity = 0;
	short temperature = 0;

	mutex_lock(&pdata->update_lock);
	if (pdata->active == SHT3X_STANDBY)
		goto out;
	idev = pdata->poll_dev->input;
	input_report_abs(idev, ABS_HUMIDITY, pdata->humidity);
	input_report_abs(idev, ABS_TEMPTERAURE, pdata->temperature);
	input_sync(idev);
out:
	mutex_unlock(&pdata->update_lock);
}

static void sht3x_dev_poll(struct input_polled_dev *dev)
{
	struct sht3x_data *pdata = (struct sht3x_data *)dev->private;

	sht3x_update_client(pdata);
	report_abs(pdata);
}

static int sht3x_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	int ret;
	char status_reg[2];
	struct sht3x_data *data;
	struct device *hwmon_dev;
	struct i2c_adapter *adap = client->adapter;
	struct device *dev = &client->dev;
	struct input_dev *idev;

	if (!i2c_check_functionality(adap, I2C_FUNC_I2C)) {
		dev_err(dev, "plain i2c transactions not supported\n");
		return -ENODEV;
	}

	ret = i2c_master_send(client, sht3x_cmd_read_status_reg, SHT3X_CMD_LENGTH);
	if (ret != SHT3X_CMD_LENGTH) {
		dev_err(dev, "could not send read_status_reg command: %d\n", ret);
		return ret < 0 ? ret : -ENODEV;
	}
	ret = i2c_master_recv(client, status_reg, sizeof(status_reg));
	if (ret != sizeof(status_reg)) {
		dev_err(dev, "could not read status register: %d\n", ret);
		return -ENODEV;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->setup.blocking_io = false;
	data->client = client;
	i2c_set_clientdata(client, data);

	if (client->dev.platform_data)
		data->setup = *(struct sht3x_platform_data *)dev->platform_data;
	sht3x_select_command(data);
	mutex_init(&data->update_lock);

	data->poll_dev = input_allocate_polled_device();
	if (!data->poll_dev) {
		ret = -ENOMEM;
		dev_err(&client->dev, "alloc poll device failed!\n");
		goto err_alloc_data;
	}
	data->poll_dev->poll = sht3x_dev_poll;
	data->poll_dev->private = data;
	data->poll_dev->poll_interval = POLL_INTERVAL;
	data->poll_dev->poll_interval_min = POLL_INTERVAL_MIN;
	data->poll_dev->poll_interval_max = POLL_INTERVAL_MAX;
	idev = data->poll_dev->input;
	idev->name = "sht3x";
	idev->id.bustype = BUS_I2C;
	idev->evbit[0] = BIT_MASK(EV_ABS);

	input_set_abs_params(idev, ABS_TEMPTERAURE, -0x7FFFFFFF, 0x7FFFFFFF, 0, 0);
	input_set_abs_params(idev, ABS_HUMIDITY, -0x7FFFFFFF, 0x7FFFFFFF, 0, 0);
	ret = input_register_polled_device(data->poll_dev);
	if (ret) {
		dev_err(&client->dev, "register poll device failed!\n");
		goto error_free_poll_dev;
	}
	ret = sysfs_create_group(&idev->dev.kobj, &sht3x_attr_group);
	if (ret) {
		dev_err(&client->dev, "create device file failed!\n");
		ret = -EINVAL;
		goto error_register_polled_device;
	}
	printk("sht3x device driver probe successfully");
	return 0;
error_register_polled_device:
	input_unregister_polled_device(data->poll_dev);
error_free_poll_dev:
	input_free_polled_device(data->poll_dev);
err_alloc_data:
	kfree(data);
err_out:
	return ret;
}

/* device ID table */
static const struct i2c_device_id sht3x_id[] = {
	{ "sht3x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sht3x_id);

static struct i2c_driver sht3x_i2c_driver = {
	.driver.name  = "sht3x",
	.probe        = sht3x_probe,
	.id_table     = sht3x_id,
};

module_i2c_driver(sht3x_i2c_driver);

MODULE_AUTHOR("David Frey <david.frey@sensirion.com>");
MODULE_DESCRIPTION("Sensirion SHT3x humidity and temperature sensor driver");
MODULE_LICENSE("GPL");
