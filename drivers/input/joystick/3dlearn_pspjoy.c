/*
 * Driver for digital joysticks connected using GPIOs
 *
 * Copyright (C) 2015, Intel Corporation
 * Author: Hans Holmberg  <hans.holmberg@xxxxxxxxx>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio/consumer.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/input.h>
#include <linux/of_gpio.h>

#define DRV_NAME "gpio-joy"
#define DEFAULT_DEBOUNCE_MS 10
#define DEFAULT_DETECT_MIN	63
#define DEFAULT_DETECT_MAX	190
#define DEFAULT_DETECT_UP 10
#define DEFAULT_DETECT_DOWN 245
#define DEFAULT_DETECT_LEFT 10
#define DEFAULT_DETECT_RIGHT 245
#define ADC_TIMES 1



//extern u32 max11801_read_adc(void);
extern u32 max1307_read_adc(unsigned int channel);

enum control_pin_indices {
	BUTTON_UP,
	BUTTON_DOWN,
	BUTTON_LEFT,
	BUTTON_RIGHT,
	//X_ABS,
	//Y_ABS,
	NUM_CONTROL_PINS,
};

//static const char *gpio_ids[NUM_CONTROL_PINS] = {
//	"left", "right", "up", "down", "button1", "button2", "button3" 
//	"button4", "button5", "button6"};
/*static const char *gpio_ids[NUM_CONTROL_PINS] = {
	"A-gpio", "B-gpio", "C-gpio", "D-gpio", "Yes-gpio", "No-gpio", \
	"Up-gpio", "Down-gpio", "Left-gpio", "Right-gpio", "Center-gpio"};
*/
static const char *code_ids[NUM_CONTROL_PINS] = {
	"up-linux,code", "down-linux,code", "left-linux,code", "right-linux,code"/*, "x-code", "y-code"*/};

struct control_pin {
	int gpio;
	int irq;
	int code;
};

struct axis_detect {
	unsigned int max;
	unsigned int min;
	unsigned int up;
	unsigned int down;
	unsigned int left;
	unsigned int right;
};

struct gpio_joy_drvdata {
	struct input_dev *input;
	struct delayed_work work;
	unsigned int debounce_jiffies;
	unsigned int debounce_range;
	unsigned int X_prevalue;
	unsigned int Y_prevalue;
	struct axis_detect detect_axis;
	struct control_pin control_pins[NUM_CONTROL_PINS];
};

static void gpio_joy_report_state(struct gpio_joy_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	int X_value, Y_value;
	unsigned int dbvalue;

	dbvalue = ddata->debounce_range;
	/* ABS_X */
	X_value = max1307_read_adc(1);
	/* ABS_Y */
	Y_value = max1307_read_adc(0);

	
	// X_value or Y_value is -1, ADC convetor is not ready
	// X_value and Y_value is 0, joystick maybe not ready
	if(((X_value == 0) && (Y_value == 0)) || (X_value == -1) || (Y_value == -1)) {
		return;
	}

	// get average value of four times adc value
	X_value = 255 - X_value;	// let left adc value is zero
	Y_value = 255 - Y_value;	// let up adc value is zero
	
	//printk("Report X_value is : %d\n", X_value);
	//printk("Report Y_value is : %d\n", Y_value);

	if((Y_value > ddata->detect_axis.min) && (Y_value < ddata->detect_axis.max)) {
		if((X_value < ddata->detect_axis.left)) {
			input_event(input, EV_KEY, ddata->control_pins[BUTTON_LEFT].code, 1);
			//printk("KEY_LEFT~~~~~~~~~~~~\n");
		}
		else if ((X_value > ddata->detect_axis.left) && (X_value < ddata->detect_axis.right)) {
			input_event(input, EV_KEY, ddata->control_pins[BUTTON_LEFT].code, 0);
			input_event(input, EV_KEY, ddata->control_pins[BUTTON_RIGHT].code, 0);
		}
		else if ((X_value > ddata->detect_axis.right)) {
			input_event(input, EV_KEY, ddata->control_pins[BUTTON_RIGHT].code, 1);
			//printk("KEY_RIGHT~~~~~~~~~~~~\n");
		}
	}
/*
	if((X_value - ddata->X_prevalue) > dbvalue || (ddata->X_prevalue - X_value) > dbvalue) {
		input_report_abs(input, ddata->control_pins[X_ABS].code, (X_value * 800) / 255);
		input_event(input, EV_ABS, ddata->control_pins[X_ABS].code, (X_value * 800) / 255);
		ddata->X_prevalue = X_value;
		printk("Report ABS_X value is : %d\n", (X_value * 800) / 255);
	}
*/

	if((X_value > ddata->detect_axis.min) && (X_value < ddata->detect_axis.max)) {
		if((Y_value < ddata->detect_axis.up))  {
			input_event(input, EV_KEY, ddata->control_pins[BUTTON_UP].code, 1);
			//printk("KEY_UP~~~~~~~~~~~~\n");
		}
		else if ((Y_value > ddata->detect_axis.up) && (Y_value < ddata->detect_axis.down)) {
			input_event(input, EV_KEY, ddata->control_pins[BUTTON_UP].code, 0);
			input_event(input, EV_KEY, ddata->control_pins[BUTTON_DOWN].code, 0);
		}
		else if ((Y_value > ddata->detect_axis.down))  {
			input_event(input, EV_KEY, ddata->control_pins[BUTTON_DOWN].code, 1);
			//printk("KEY_DOWN~~~~~~~~~~~~\n");
		}
	}
/*
	if((Y_value - ddata->Y_prevalue) > dbvalue || (ddata->Y_prevalue - Y_value) > dbvalue) {
		input_report_abs(input, ddata->control_pins[Y_ABS].code, (Y_value * 600) / 255);
		input_event(input, EV_ABS, ddata->control_pins[Y_ABS].code, (Y_value * 600) / 255);
		ddata->Y_prevalue = Y_value;
		printk("Report ABS_Y value is : %d\n", (Y_value * 600) / 255);
	}
*/	
	input_sync(input);
}

static void gpio_joy_work(struct work_struct *work)
{
	struct gpio_joy_drvdata *ddata =
		container_of(work, struct gpio_joy_drvdata, work.work);

	schedule_delayed_work(&ddata->work, ddata->debounce_jiffies);
	gpio_joy_report_state(ddata);
}


static int gpio_joy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct input_dev *input;
	struct gpio_joy_drvdata *ddata;
	struct device_node *node;
	int i, err;
	//unsigned int debounce_ms;


	node = dev->of_node;
	ddata = devm_kzalloc(dev, sizeof(struct gpio_joy_drvdata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	ddata->input = input;
	ddata->X_prevalue = 0;
	ddata->X_prevalue = 0;

	input->name = pdev->name;
	input->phys = DRV_NAME"/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	set_bit(EV_KEY, input->evbit);
	//set_bit(EV_ABS, input->evbit);
	//set_bit(BTN_TRIGGER, input->keybit);

	//input_set_abs_params(input, ABS_X, 0, 800, 0, 0);
	//input_set_abs_params(input, ABS_Y, 0, 600, 0, 0);


	/* debounce interval is optional */
	//if (device_property_read_u32(dev, "debounce-interval-ms", &debounce_ms))
	if (of_property_read_u32(node, "debounce-interval-ms", &ddata->debounce_jiffies))
		ddata->debounce_jiffies = DEFAULT_DEBOUNCE_MS;
	//ddata->debounce_jiffies = msecs_to_jiffies(debounce_ms);

	if (of_property_read_u32(node, "debounce-range", &ddata->debounce_range))
		ddata->debounce_range = 5;
	
	if(of_property_read_u32(node, "detect-max", &ddata->detect_axis.max))
		ddata->detect_axis.max = DEFAULT_DETECT_MAX;	

	if(of_property_read_u32(node, "detect-min", &ddata->detect_axis.min))
		ddata->detect_axis.min = DEFAULT_DETECT_MIN;
	
	if(of_property_read_u32(node, "detect-up", &ddata->detect_axis.up))
		ddata->detect_axis.up = DEFAULT_DETECT_UP;
	
	if(of_property_read_u32(node, "detect-down", &ddata->detect_axis.down))
		ddata->detect_axis.down = DEFAULT_DETECT_DOWN;
	
	if(of_property_read_u32(node, "detect-left", &ddata->detect_axis.left))
		ddata->detect_axis.left = DEFAULT_DETECT_LEFT;
	
	if(of_property_read_u32(node, "detect-right", &ddata->detect_axis.right))
		ddata->detect_axis.right = DEFAULT_DETECT_RIGHT;

	for (i = 0; i < ARRAY_SIZE(ddata->control_pins) ; i++) {
		//if(i<4) {
			if (of_property_read_u32(node, code_ids[i], &ddata->control_pins[i].code)) {
				dev_err(dev, "Button without keycode: 0x%x\n",
					ddata->control_pins[i].code);
				err = -EINVAL;
				goto fail;
			}
			input_set_capability(input, EV_KEY, ddata->control_pins[i].code);
		//}
/*
		else if(i==4) {
			ddata->control_pins[i].code = 0x00;
			input_set_capability(input, EV_ABS, ddata->control_pins[i].code);
		}
		else if(i==5) {
			ddata->control_pins[i].code = 0x01;
			input_set_capability(input, EV_ABS, ddata->control_pins[i].code);
		}
*/
	}
	input_set_drvdata(input, ddata);

	INIT_DELAYED_WORK(&ddata->work, gpio_joy_work);
	schedule_delayed_work(&ddata->work, ddata->debounce_jiffies);

	err = input_register_device(input);
	if (err) {
		dev_err(dev, "unable to register input device\n");
		goto fail;
	}


	platform_set_drvdata(pdev, ddata);
	device_init_wakeup(&pdev->dev, 1);
	return 0;
fail:
	return err;
}

static const struct of_device_id gpio_joy_of_match[] = {
	{ .compatible = DRV_NAME, },
	{ },
};

static struct platform_driver gpio_joy_driver = {
	.probe		= gpio_joy_probe,
	.driver		= {
		.name	= DRV_NAME,
		.of_match_table = gpio_joy_of_match,
	}
};

module_platform_driver(gpio_joy_driver);

MODULE_ALIAS("platform:" DRV_NAME);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hans Holmberg <hans.holmberg@xxxxxxxxx>");
MODULE_DESCRIPTION("Driver for digital joysticks connected via GPIOs");


