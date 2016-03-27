
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>


static const struct of_device_id led_dt_ids[] = {
	{ .compatible = "leds-tvbs", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, led_dt_ids);

static int led_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;
	int led_red_gpio, led_grn_gpio;
	int batt_adc_sel_gpio;

	led_red_gpio = of_get_named_gpio(np, "led-red-gpios", 0);
	if (gpio_is_valid(led_red_gpio)) {
		ret = gpio_request_one(led_red_gpio, GPIOF_OUT_INIT_LOW,
				"led-red-gpios");
		if (ret)
			pr_warn("failed to request led-red-gpios gpio\n");
	}

	led_grn_gpio = of_get_named_gpio(np, "led-grn-gpios", 0);
	if (gpio_is_valid(led_grn_gpio)) {
		ret = gpio_request_one(led_grn_gpio, GPIOF_OUT_INIT_LOW,
				"led-grn-gpios");
		if (ret)
			pr_warn("failed to request led-grn-gpios gpio\n");
	}


	batt_adc_sel_gpio = of_get_named_gpio(np, "battadc-gpios", 0);
	if (gpio_is_valid(batt_adc_sel_gpio)) {
		ret = gpio_request_one(batt_adc_sel_gpio, GPIOF_OUT_INIT_HIGH,
					"Batt ADC sel");
		if (ret)
			pr_warn("failed to request battadc-sel-gpios gpio\n");
	} else
		dev_warn(dev, "no batt_adc_sel pin available\n");

	return 0;
}

static int led_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int led_suspend(struct device *dev)
{

	return 0;
}

static int led_resume(struct device *dev)
{

	return 0;
}
#else
#define	led_suspend	NULL
#define	led_resume	NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int led_runtime_suspend(struct device *dev)
{

	return 0;
}

static int led_runtime_resume(struct device *dev)
{

	return 0;
}
#else
#define	led_runtime_suspend	NULL
#define	led_runtime_resume	NULL
#endif

static const struct dev_pm_ops led_pm_ops = {
	SET_RUNTIME_PM_OPS(led_runtime_suspend, led_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(led_suspend, led_resume)
};

static struct platform_driver led_driver = {
	.driver = {
			.name = "led-tvbs",
			.of_match_table = of_match_ptr(led_dt_ids),
			.pm = &led_pm_ops,
		   },
	.probe = led_probe,
	.remove = led_remove,
};

static int __init led_init(void)
{
        return platform_driver_register(&led_driver);
}
late_initcall(led_init);

static void __exit led_exit(void)
{
        platform_driver_unregister(&led_driver);
}
module_exit(led_exit);


MODULE_DESCRIPTION("led power pin control driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
