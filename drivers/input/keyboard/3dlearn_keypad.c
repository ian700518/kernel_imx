/*
 * Driver for the IMX keypad port.
 * Copyright (C) 2009 Alberto Panizzo <maramaopercheseimorto@gmail.com>
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * <<Power management needs to be implemented>>.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/input/matrix_keypad.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>

/*
 * Keypad Controller registers (halfword)
 */
#define KPCR		0x00 /* Keypad Control Register */

#define KPSR		0x02 /* Keypad Status Register */
#define KBD_STAT_KPKD	(0x1 << 0) /* Key Press Interrupt Status bit (w1c) */
#define KBD_STAT_KPKR	(0x1 << 1) /* Key Release Interrupt Status bit (w1c) */
#define KBD_STAT_KDSC	(0x1 << 2) /* Key Depress Synch Chain Status bit (w1c)*/
#define KBD_STAT_KRSS	(0x1 << 3) /* Key Release Synch Status bit (w1c)*/
#define KBD_STAT_KDIE	(0x1 << 8) /* Key Depress Interrupt Enable Status bit */
#define KBD_STAT_KRIE	(0x1 << 9) /* Key Release Interrupt Enable */
#define KBD_STAT_KPPEN	(0x1 << 10) /* Keypad Clock Enable */

#define KDDR		0x04 /* Keypad Data Direction Register */
#define KPDR		0x06 /* Keypad Data Register */

#define MAX_MATRIX_KEY_ROWS	6
#define MAX_MATRIX_KEY_COLS	6
#define MATRIX_ROW_SHIFT	3

#define MAX_MATRIX_KEY_NUM	(MAX_MATRIX_KEY_ROWS * MAX_MATRIX_KEY_COLS)

#define ROWS_NUM	6

struct imx_keypad {

	struct clk *clk;
	struct input_dev *input_dev;
	void __iomem *mmio_base;

	int			irq;
	struct timer_list	check_matrix_timer;

	/*
	 * The matrix is stable only if no changes are detected after
	 * IMX_KEYPAD_SCANS_FOR_STABILITY scans
	 */
#define IMX_KEYPAD_SCANS_FOR_STABILITY 3
	int			stable_count;

	bool			enabled;

	/* Masks for enabled rows/cols */
	unsigned short		rows_en_mask;
	unsigned short		cols_en_mask;

	unsigned short		keycodes[MAX_MATRIX_KEY_NUM];

	/*
	 * Matrix states:
	 * -stable: achieved after a complete debounce process.
	 * -unstable: used in the debouncing process.
	 */
	unsigned short		matrix_stable_state[MAX_MATRIX_KEY_COLS];
	unsigned short		matrix_unstable_state[MAX_MATRIX_KEY_COLS];

	int gpio_rows[ROWS_NUM];
	int irq_flag;
	int irq_flag_prev;
	int scancode;
	bool key_released;
	int irq_gpio;
	struct mutex mutex;
	unsigned long jiffies;
};

/*
 * imx_keypad_check_for_events is the timer handler.
 */
static void imx_keypad_check_for_events(unsigned long data)
{
	struct imx_keypad *keypad = (struct imx_keypad *) data;
	struct input_dev *input_dev = keypad->input_dev;
	unsigned short matrix_volatile_state[MAX_MATRIX_KEY_COLS];
	unsigned short reg_val;
	bool state_changed, is_zero_matrix;
	int i, j;
	int val;
	int row = -1, col = -1;
	bool found_key = false;
	int code;
	static int count;

	mutex_lock(&keypad->mutex);
	disable_irq(keypad->irq);
//	gpio_direction_output(keypad->irq_gpio, 0);

	if (keypad->jiffies != 0 &&
	    time_after(keypad->jiffies + msecs_to_jiffies(60), jiffies) > 0) {
		if ((keypad->irq_flag_prev & IRQF_TRIGGER_FALLING) &&
			(keypad->irq_flag & IRQF_TRIGGER_RISING)) {

			keypad->irq_flag = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
			irq_set_irq_type(keypad->irq, keypad->irq_flag);
		//	gpio_direction_input(keypad->irq_gpio);
			udelay(2);
			keypad->irq_flag_prev = keypad->irq_flag;
			enable_irq(keypad->irq);
			mutex_unlock(&keypad->mutex);
			return;
		}
	}
	keypad->jiffies = jiffies;
	keypad->irq_flag_prev = keypad->irq_flag;
	
	if (keypad->key_released == true) {
		code = keypad->scancode;
		input_event(input_dev, EV_MSC, MSC_SCAN, code);
		input_report_key(input_dev, keypad->keycodes[code], 0);
		dev_info(&input_dev->dev, "Event code: %d, val: %d",
			keypad->keycodes[code], 0);
		input_sync(input_dev);
		for (i = 0; i < ROWS_NUM; i++)
			gpio_direction_output(keypad->gpio_rows[i], 0);
		keypad->key_released = false;
		keypad->irq_flag = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		irq_set_irq_type(keypad->irq, keypad->irq_flag);
//		gpio_direction_input(keypad->irq_gpio);
		udelay(2);
		enable_irq(keypad->irq);
		mutex_unlock(&keypad->mutex);
		return;

	}

	for (i = 0; i < ROWS_NUM; i++)
		gpio_direction_input(keypad->gpio_rows[i]);
		
	for (i = 0; i < ROWS_NUM; i++) {
		val = gpio_get_value(keypad->gpio_rows[i]);
		if (val == 0) {
			row = ROWS_NUM-1;
			col = i;
			found_key = true;
			//printk("1. row %d, col %d\n", row, col);
			break;
		}
	}

	if (found_key == false) {
		for (i = 0; i < ROWS_NUM && found_key == false; i++) {
			for (j = 0; j < MAX_MATRIX_KEY_ROWS; j++) {
				if (i == j)
					continue;
				gpio_direction_input(keypad->gpio_rows[j]);
			}
			udelay(5);
			gpio_direction_output(keypad->gpio_rows[i], 0);
			udelay(5);

			//for (j = 0; j < MAX_MATRIX_KEY_COLS; j++) {
			for (j = 0; j < MAX_MATRIX_KEY_COLS; j++) {
				if (i == j)
					continue;
//	printk("==> i=%d\n", i);

				val = gpio_get_value(keypad->gpio_rows[j]);
				if (val == 0) {
					col = j;
					row = i;
					found_key = true;
					//printk("2. row %d, col %d\n", row, col);
					break;
				}
			}
		}
		//printk("3. row %d, col %d, found_key %d\n", row, col, found_key);
	}

	if (found_key == true &&
		row >= 0 && row < MAX_MATRIX_KEY_ROWS &&
		col >= 0 && col < MAX_MATRIX_KEY_COLS) {
		keypad->scancode = code = MATRIX_SCAN_CODE(row, col, MATRIX_ROW_SHIFT);
		input_event(input_dev, EV_MSC, MSC_SCAN, code);
		input_report_key(input_dev, keypad->keycodes[code], 1);
		dev_info(&input_dev->dev, "Event code: %d, val: %d",
			keypad->keycodes[code], 1);
		input_sync(input_dev);
	}

	
#if 1
	if (found_key == false) { /* get rid of false alarm */
		for (i = 0; i < ROWS_NUM; i++)
			gpio_direction_output(keypad->gpio_rows[i], 0);
		
		udelay(5);

		keypad->irq_flag = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		irq_set_irq_type(keypad->irq, keypad->irq_flag);
//	gpio_direction_input(keypad->irq_gpio);
	udelay(2);
		enable_irq(keypad->irq);
		mutex_unlock(&keypad->mutex);
		return;
	}
#endif

	found_key = false;
//	for (i = 0; i < ROWS_NUM; i++)
//		gpio_direction_output(keypad->gpio_rows[i], 0);
//	mdelay(200);

	keypad->irq_flag = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	irq_set_irq_type(keypad->irq, keypad->irq_flag);
//	gpio_direction_input(keypad->irq_gpio);
	udelay(2);
	enable_irq(keypad->irq);
	mutex_unlock(&keypad->mutex);
}

static irqreturn_t imx_keypad_irq_handler(int irq, void *dev_id)
{
	struct imx_keypad *keypad = dev_id;
	unsigned short reg_val;

	if (keypad->enabled) {
		pm_stay_awake(keypad->input_dev->dev.parent);

		/* The matrix is supposed to be changed */
		keypad->stable_count = 0;

		/* Schedule the scanning procedure near in the future */
		mod_timer(&keypad->check_matrix_timer,
			  jiffies + msecs_to_jiffies(2));

		if (keypad->irq_flag & IRQF_TRIGGER_FALLING) {
			keypad->key_released = false;
		} else if (keypad->irq_flag & IRQF_TRIGGER_RISING) {
			keypad->key_released = true;
		}

	}

	return IRQ_HANDLED;
}

static void imx_keypad_close(struct input_dev *dev)
{
	struct imx_keypad *keypad = input_get_drvdata(dev);

	dev_dbg(&dev->dev, ">%s\n", __func__);

	/* Mark keypad as being inactive */
	keypad->enabled = false;
	synchronize_irq(keypad->irq);
	del_timer_sync(&keypad->check_matrix_timer);

}

static int imx_keypad_open(struct input_dev *dev)
{
	struct imx_keypad *keypad = input_get_drvdata(dev);
	int error;
	dev_dbg(&dev->dev, ">%s\n", __func__);

#if 1
	/* We became active from now */
	keypad->enabled = true;

	return 0;
#else
	/* Enable the kpp clock */
	error = clk_prepare_enable(keypad->clk);
	if (error)
		return error;

	/* We became active from now */
	keypad->enabled = true;

	imx_keypad_config(keypad);

	/* Sanity control, not all the rows must be actived now. */
	if ((readw(keypad->mmio_base + KPDR) & keypad->rows_en_mask) == 0) {
		dev_err(&dev->dev,
			"too many keys pressed, control pins initialisation\n");
		goto open_err;
	}

	return 0;

open_err:
	imx_keypad_close(dev);
	return -EIO;
#endif
}

#ifdef CONFIG_OF
static struct of_device_id imx_keypad_of_match[] = {
	{ .compatible = "fsl,gpio-kpp", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_keypad_of_match);
#endif

static int imx_keypad_probe(struct platform_device *pdev)
{
	const struct matrix_keymap_data *keymap_data =
			dev_get_platdata(&pdev->dev);
	struct pinctrl *pinctrl;
	struct device_node *np = pdev->dev.of_node;
	struct imx_keypad *keypad;
	struct input_dev *input_dev;
	struct resource *res;
	int irq, error, i, row, col;
	int irq_gpio;
	int gpio_rows[ROWS_NUM];
	char gpio_name[10] = "gpio-row0";
	int retval;

	if (!keymap_data && !pdev->dev.of_node) {
		dev_err(&pdev->dev, "no keymap defined\n");
		return -EINVAL;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "can't get/select pinctrl\n");
		return PTR_ERR(pinctrl);
	}

	if (of_find_property(np, "irq-gpio", NULL)) {
		irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
		if (irq_gpio == -EPROBE_DEFER) {
			dev_info(&pdev->dev, "GPIO requested is not"
				"here yet, deferring the probe\n");
			return -EPROBE_DEFER;
		}
		if (!gpio_is_valid(irq_gpio)) {
			dev_warn(&pdev->dev, "No dt property: irq-gpios\n");
		} else {

			retval = devm_gpio_request_one(&pdev->dev,
						    irq_gpio,
						    GPIOF_IN,
						    "irq_pins");
			if (retval) {
				dev_err(&pdev->dev, "failed to request gpio"
					" %d: %d\n", irq_gpio, retval);
				return -EINVAL;
			}
		}

	}
	for (i = 0; i < ROWS_NUM; i++) {
		gpio_name[8] = '0' + i;
		if (of_find_property(np, gpio_name, NULL)) {
			gpio_rows[i] = of_get_named_gpio(np, gpio_name, 0);
			//printk("gpio_rows[%d]= %d\n", i, gpio_rows[i]);
			if (gpio_rows[i] == -EPROBE_DEFER) {
				dev_info(&pdev->dev, "GPIO requested is not"
					"here yet, deferring the probe\n");
				return -EPROBE_DEFER;
			}
			if (!gpio_is_valid(gpio_rows[i])) {
				dev_warn(&pdev->dev, "No dt property: gpio-rows\n");
			} else {

				retval = devm_gpio_request_one(&pdev->dev,
							    gpio_rows[i],
							    GPIOF_OUT_INIT_LOW,
							    gpio_name);
				if (retval) {
					dev_err(&pdev->dev, "failed to request gpio"
						" %d: %d\n", gpio_rows[i], retval);
					return -EINVAL;
				}
			}
		}
	}

	irq = gpio_to_irq(irq_gpio);

	input_dev = devm_input_allocate_device(&pdev->dev);
	if (!input_dev) {
		dev_err(&pdev->dev, "failed to allocate the input device\n");
		return -ENOMEM;
	}

	keypad = devm_kzalloc(&pdev->dev, sizeof(struct imx_keypad),
			     GFP_KERNEL);
	if (!keypad) {
		dev_err(&pdev->dev, "not enough memory for driver data\n");
		return -ENOMEM;
	}

	keypad->input_dev = input_dev;
	keypad->irq = irq;
	keypad->irq_gpio = irq_gpio;
	keypad->stable_count = 0;
	mutex_init(&keypad->mutex);

	for (i = 0; i < ROWS_NUM; i++)
		keypad->gpio_rows[i] = gpio_rows[i];

	for (i = 0; i < ROWS_NUM; i++)
	//printk("keypad->gpio_rows[%d]= %d\n", i, keypad->gpio_rows[i]);
	setup_timer(&keypad->check_matrix_timer,
		    imx_keypad_check_for_events, (unsigned long) keypad);

	/* Init the Input device */
	input_dev->name = pdev->name;
	input_dev->id.bustype = BUS_HOST;
	input_dev->dev.parent = &pdev->dev;
	input_dev->open = imx_keypad_open;
	input_dev->close = imx_keypad_close;

	error = matrix_keypad_build_keymap(keymap_data, NULL,
					   MAX_MATRIX_KEY_ROWS,
					   MAX_MATRIX_KEY_COLS,
					   keypad->keycodes, input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to build keymap\n");
		return error;
	}


	__set_bit(EV_REP, input_dev->evbit);
	input_set_capability(input_dev, EV_MSC, MSC_SCAN);
	input_set_drvdata(input_dev, keypad);

	keypad->jiffies = 0;
	keypad->irq_flag = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	error = devm_request_irq(&pdev->dev, irq, imx_keypad_irq_handler, keypad->irq_flag,
			    pdev->name, keypad);
	if (error) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		return error;
	}

	/* Register the input device */
	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		return error;
	}

	platform_set_drvdata(pdev, keypad);
	device_init_wakeup(&pdev->dev, 1);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int imx_kbd_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct imx_keypad *kbd = platform_get_drvdata(pdev);
	struct input_dev *input_dev = kbd->input_dev;
	unsigned short reg_val = readw(kbd->mmio_base + KPSR);

	if (device_may_wakeup(&pdev->dev)) {
		enable_irq_wake(kbd->irq);
	}

	return 0;
}

static int imx_kbd_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct imx_keypad *kbd = platform_get_drvdata(pdev);
	struct input_dev *input_dev = kbd->input_dev;
	int ret = 0;

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(kbd->irq);


	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(imx_kbd_pm_ops, imx_kbd_suspend, imx_kbd_resume);

static struct platform_driver imx_keypad_driver = {
	.driver		= {
		.name	= "imx-keypad",
		.owner	= THIS_MODULE,
		.pm	= &imx_kbd_pm_ops,
		.of_match_table = of_match_ptr(imx_keypad_of_match),
	},
	.probe		= imx_keypad_probe,
};
module_platform_driver(imx_keypad_driver);

MODULE_AUTHOR("Alberto Panizzo <maramaopercheseimorto@gmail.com>");
MODULE_DESCRIPTION("IMX Keypad Port Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-keypad");

