/*
 * gpio-pir.c - Hwmon driver for pirs connected to GPIO lines.
 *
 * Copyright (C) 2010 LaCie
 *
 * Author: Simon Guinot <sguinot@lacie.com>
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/hwmon.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

struct gpio_pir_data {
	struct platform_device	*pdev;
	struct device		*hwmon_dev;
	struct mutex		lock; /* lock GPIOs operations. */
	bool			present_status;
	struct gpio_pir_alarm	*alarm;
	struct work_struct	alarm_work;
};

struct gpio_pir_alarm {
	unsigned	gpio;
	unsigned	active_low;
};

struct gpio_pir_platform_data {
	struct gpio_pir_alarm	*alarm;	/* alarm GPIO. */
};

/*
 * Alarm GPIO.
 */

static void pir_alarm_notify(struct work_struct *ws)
{
	struct gpio_pir_data *pir_data =
		container_of(ws, struct gpio_pir_data, alarm_work);

	sysfs_notify(&pir_data->pdev->dev.kobj, NULL, "pir1_alarm");
	kobject_uevent(&pir_data->pdev->dev.kobj, KOBJ_CHANGE);
}

static irqreturn_t pir_alarm_irq_handler(int irq, void *dev_id)
{
	struct gpio_pir_data *pir_data = dev_id;

	schedule_work(&pir_data->alarm_work);

	return IRQ_NONE;
}

static ssize_t show_pir_alarm(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct gpio_pir_data *pir_data = dev_get_drvdata(dev);
	struct gpio_pir_alarm *alarm = pir_data->alarm;
	int value = gpio_get_value(alarm->gpio);

	if (alarm->active_low)
		value = !value;

	return sprintf(buf, "%d\n", value);
}

static DEVICE_ATTR(pir_alarm, S_IRUGO, show_pir_alarm, NULL);

static int pir_alarm_init(struct gpio_pir_data *pir_data,
			  struct gpio_pir_alarm *alarm)
{
	int err;
	int alarm_irq;
	struct platform_device *pdev = pir_data->pdev;

	pir_data->alarm = alarm;

	err = devm_gpio_request(&pdev->dev, alarm->gpio, "GPIO pir alarm");
	if (err)
		return err;

	err = gpio_direction_input(alarm->gpio);
	if (err)
		return err;

	/*
	 * If the alarm GPIO don't support interrupts, just leave
	 * without initializing the fail notification support.
	 */
	alarm_irq = gpio_to_irq(alarm->gpio);
	if (alarm_irq < 0)
		return 0;

	INIT_WORK(&pir_data->alarm_work, pir_alarm_notify);
	irq_set_irq_type(alarm_irq, IRQ_TYPE_EDGE_BOTH);
	err = devm_request_irq(&pdev->dev, alarm_irq, pir_alarm_irq_handler,
			       IRQF_SHARED, "GPIO pir alarm", pir_data);
	return err;
}

static struct attribute *gpio_pir_attributes[] = {
	&dev_attr_pir_alarm.attr,
	NULL
};

static const struct attribute_group gpio_pir_group = {
	.attrs = gpio_pir_attributes,
};

static const struct attribute_group *gpio_pir_groups[] = {
	&gpio_pir_group,
	NULL
};

#ifdef CONFIG_OF_GPIO
/*
 * Translate OpenFirmware node properties into platform_data
 */
static int gpio_pir_get_of_pdata(struct device *dev,
			    struct gpio_pir_platform_data *pdata)
{
	struct device_node *node;

	node = dev->of_node;
	/* Alarm GPIO if one exists */
	if (of_gpio_named_count(node, "alarm-gpios") > 0) {
		struct gpio_pir_alarm *alarm;
		int val;
		enum of_gpio_flags flags;

		alarm = devm_kzalloc(dev, sizeof(struct gpio_pir_alarm),
					GFP_KERNEL);
		if (!alarm)
			return -ENOMEM;

		val = of_get_named_gpio_flags(node, "alarm-gpios", 0, &flags);
		if (val < 0)
			return val;
		alarm->gpio = val;
		alarm->active_low = flags & OF_GPIO_ACTIVE_LOW;

		pdata->alarm = alarm;
	}

	return 0;
}

static struct of_device_id of_gpio_pir_match[] = {
	{ .compatible = "gpio-pir", },
	{},
};
#endif /* CONFIG_OF_GPIO */

static int gpio_pir_probe(struct platform_device *pdev)
{
	int err;
	struct gpio_pir_data *pir_data;
	struct gpio_pir_platform_data *pdata = dev_get_platdata(&pdev->dev);

#ifdef CONFIG_OF_GPIO
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev,
					sizeof(struct gpio_pir_platform_data),
					GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		err = gpio_pir_get_of_pdata(&pdev->dev, pdata);
		if (err)
			return err;
	}
#else /* CONFIG_OF_GPIO */
	if (!pdata)
		return -EINVAL;
#endif /* CONFIG_OF_GPIO */

	pir_data = devm_kzalloc(&pdev->dev, sizeof(struct gpio_pir_data),
				GFP_KERNEL);
	if (!pir_data)
		return -ENOMEM;

	pir_data->pdev = pdev;
	platform_set_drvdata(pdev, pir_data);
	mutex_init(&pir_data->lock);

	/* Configure alarm GPIO if available. */
	if (pdata->alarm) {
		err = pir_alarm_init(pir_data, pdata->alarm);
		if (err)
			return err;
	}

	/* Make this driver part of hwmon class. */
	pir_data->hwmon_dev = hwmon_device_register_with_groups(&pdev->dev,
						"gpio-pir", pir_data,
						gpio_pir_groups);
	if (IS_ERR(pir_data->hwmon_dev))
		return PTR_ERR(pir_data->hwmon_dev);

	dev_info(&pdev->dev, "GPIO for PIR initialized\n");

	return 0;
}

static int gpio_pir_remove(struct platform_device *pdev)
{
	struct gpio_pir_data *pir_data = platform_get_drvdata(pdev);

	hwmon_device_unregister(pir_data->hwmon_dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_pir_suspend(struct device *dev)
{
	struct gpio_pir_data *pir_data = dev_get_drvdata(dev);

	return 0;
}

static int gpio_pir_resume(struct device *dev)
{
	struct gpio_pir_data *pir_data = dev_get_drvdata(dev);

	return 0;
}

static SIMPLE_DEV_PM_OPS(gpio_pir_pm, gpio_pir_suspend, gpio_pir_resume);
#define GPIO_PIR_PM	(&gpio_pir_pm)
#else
#define GPIO_PIR_PM	NULL
#endif

static struct platform_driver gpio_pir_driver = {
	.probe		= gpio_pir_probe,
	.remove		= gpio_pir_remove,
	.driver	= {
		.name	= "gpio-pir",
		.pm	= GPIO_PIR_PM,
#ifdef CONFIG_OF_GPIO
		.of_match_table = of_match_ptr(of_gpio_pir_match),
#endif
	},
};

module_platform_driver(gpio_pir_driver);

MODULE_AUTHOR("Robby <robbycai@gmail.com>");
MODULE_DESCRIPTION("GPIO for PIR driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gpio-pir");
