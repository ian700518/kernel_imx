
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>


static const struct of_device_id gps_dt_ids[] = {
	{ .compatible = "ublox,gps", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gps_dt_ids);

static int gps_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;
	int reset_gpio;

	struct regulator *gps_regulator;
	gps_regulator = devm_regulator_get(&pdev->dev, "gps");
	if (!IS_ERR(gps_regulator)) {
		regulator_set_voltage(gps_regulator,
				      3300000, 3300000);
		ret = regulator_enable(gps_regulator);
		if (ret) {
			pr_err("%s:sensor set voltage error\n", __func__);
			return ret;
		} else {
			dev_dbg(&pdev->dev,
				"%s:sensor set voltage ok\n", __func__);
		}
	} else {
		pr_err("%s: cannot get senor voltage error\n", __func__);
		gps_regulator = NULL;
	}

	/* ublox AE suggest this pin is float, so JW suggestion to set as GPIO INPUT */
	reset_gpio = of_get_named_gpio(np, "gps-rst-gpios", 0);
	if (gpio_is_valid(reset_gpio)) {
		ret = gpio_request_one(reset_gpio, GPIOF_IN,
				"gps-rst-gpios");
		if (ret)
			pr_warn("failed to request gps-rst-gpios gpio\n");
	}

	return 0;
}

static int gps_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gps_suspend(struct device *dev)
{

	return 0;
}

static int gps_resume(struct device *dev)
{

	return 0;
}
#else
#define	gps_suspend	NULL
#define	gps_resume	NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int gps_runtime_suspend(struct device *dev)
{

	return 0;
}

static int gps_runtime_resume(struct device *dev)
{

	return 0;
}
#else
#define	gps_runtime_suspend	NULL
#define	gps_runtime_resume	NULL
#endif

static const struct dev_pm_ops gps_pm_ops = {
	SET_RUNTIME_PM_OPS(gps_runtime_suspend, gps_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(gps_suspend, gps_resume)
};

static struct platform_driver gps_driver = {
	.driver = {
			.name = "gps",
			.of_match_table = of_match_ptr(gps_dt_ids),
			.pm = &gps_pm_ops,
		   },
	.probe = gps_probe,
	.remove = gps_remove,
};

static int __init gps_init(void)
{
        return platform_driver_register(&gps_driver);
}
late_initcall(gps_init);

static void __exit gps_exit(void)
{
        platform_driver_unregister(&gps_driver);
}
module_exit(gps_exit);


MODULE_DESCRIPTION("gps power/reset pin control driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
