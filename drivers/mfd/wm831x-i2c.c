/*
 * wm831x-i2c.c  --  I2C access for Wolfson WM831x PMICs
 *
 * Copyright 2009,2010 Wolfson Microelectronics PLC.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>

#include <linux/mfd/wm831x/core.h>
#include <linux/mfd/wm831x/pdata.h>
#include <linux/mfd/wm831x/regulator.h>
#include <linux/mfd/wm831x/gpio.h>
#include <linux/regulator/machine.h>

#ifdef CONFIG_MX6_INTER_LDO_BYPASS
// LDO bypass
// 1.3, 1.3. 1.5
#define WM831X_DC1_ON_CONFIG_VAL            (0x40<<WM831X_DC1_ON_VSEL_SHIFT)
#define WM831X_DC2_ON_CONFIG_VAL            (0x40<<WM831X_DC2_ON_VSEL_SHIFT)
#define WM831X_DC3_ON_CONFIG_VAL            (0x1A<<WM831X_DC3_ON_VSEL_SHIFT)
#else
// 1.375, 1.375. 1.5

#define WM831X_DC1_ON_CONFIG_VAL            (0x44<<WM831X_DC1_ON_VSEL_SHIFT)
#define WM831X_DC2_ON_CONFIG_VAL            (0x44<<WM831X_DC2_ON_VSEL_SHIFT)
#define WM831X_DC3_ON_CONFIG_VAL            (0x1A<<WM831X_DC3_ON_VSEL_SHIFT)

#endif

#define WM831X_DC1_DVS_MODE_VAL             (0x02<<WM831X_DC1_DVS_SRC_SHIFT)
#define WM831X_DC2_DVS_MODE_VAL             (0x02<<WM831X_DC2_DVS_SRC_SHIFT)

#define WM831X_DC1_DVS_CONTROL_VAL           (0x20<<WM831X_DC1_DVS_VSEL_SHIFT)
#define WM831X_DC2_DVS_CONTROL_VAL           (0x20<<WM831X_DC2_DVS_VSEL_SHIFT)

#define WM831X_DC1_DVS_MASK                  (WM831X_DC1_DVS_SRC_MASK|WM831X_DC1_DVS_VSEL_MASK)
#define WM831X_DC2_DVS_MASK                  (WM831X_DC2_DVS_SRC_MASK|WM831X_DC1_DVS_VSEL_MASK)

#define WM831X_DC1_DVS_VAL                   (WM831X_DC1_DVS_MODE_VAL|WM831X_DC1_DVS_CONTROL_VAL)
#define WM831X_DC2_DVS_VAL                   (WM831X_DC2_DVS_MODE_VAL|WM831X_DC2_DVS_CONTROL_VAL)

#define WM831X_GPN_FN_VAL_HW_EN              (0x0A<<WM831X_GPN_FN_SHIFT)
#define WM831X_GPN_FN_VAL_HW_CTL             (0x0C<<WM831X_GPN_FN_SHIFT)
#define WM831X_GPN_FN_VAL_DVS1               (0x08<<WM831X_GPN_FN_SHIFT)

#define WM831X_GPN_DIR_VAL                   (0x1<<WM831X_GPN_DIR_SHIFT)
#define WM831X_GPN_PULL_VAL                  (0x3<<WM831X_GPN_PULL_SHIFT)
#define WM831X_GPN_INT_MODE_VAL              (0x1<<WM831X_GPN_INT_MODE_SHIFT)
#define WM831X_GPN_POL_VAL                   (0x1<<WM831X_GPN_POL_SHIFT)
#define WM831X_GPN_ENA_VAL                   (0x1<<WM831X_GPN_ENA_SHIFT)

#define  WM831X_GPIO7_8_9_MASK               (WM831X_GPN_DIR_MASK|WM831X_GPN_INT_MODE_MASK|WM831X_GPN_PULL_MASK|WM831X_GPN_POL_MASK|WM831X_GPN_FN_MASK)

#define WM831X_GPIO7_VAL                     (WM831X_GPN_DIR_VAL|WM831X_GPN_PULL_VAL|WM831X_GPN_INT_MODE_VAL|WM831X_GPN_POL_VAL|WM831X_GPN_ENA_VAL|WM831X_GPN_FN_VAL_HW_EN)
#define WM831X_GPIO8_VAL                     (WM831X_GPN_DIR_VAL|WM831X_GPN_PULL_VAL|WM831X_GPN_INT_MODE_VAL|WM831X_GPN_POL_VAL|WM831X_GPN_ENA_VAL|WM831X_GPN_FN_VAL_HW_CTL)
#define WM831X_GPIO9_VAL                     (WM831X_GPN_DIR_VAL|WM831X_GPN_PULL_VAL|WM831X_GPN_INT_MODE_VAL|WM831X_GPN_POL_VAL|WM831X_GPN_ENA_VAL|WM831X_GPN_FN_VAL_DVS1)

#define WM831X_STATUS_LED_MASK                0xC000
#define WM831X_STATUS_LED_ON                  (0x1 << 14)
#define WM831X_STATUS_LED_OFF                 (0x0 << 14)

#define WM831X_DC1_CONTROL_1_RATE_VAL        (0x3<<WM831X_DC1_RATE_SHIFT)
#define WM831X_DC2_CONTROL_1_RATE_VAL        (0x3<<WM831X_DC2_RATE_SHIFT)

static int wm8326_post_init(struct wm831x *wm831x)
{
#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	unsigned int reg;
	void __iomem *gpc_base = IO_ADDRESS(GPC_BASE_ADDR);
#endif

	wm831x_set_bits(wm831x, WM831X_DC1_ON_CONFIG, WM831X_DC1_ON_VSEL_MASK, WM831X_DC1_ON_CONFIG_VAL);
	wm831x_set_bits(wm831x, WM831X_DC2_ON_CONFIG, WM831X_DC2_ON_VSEL_MASK, WM831X_DC2_ON_CONFIG_VAL);
	wm831x_set_bits(wm831x, WM831X_DC3_ON_CONFIG, WM831X_DC3_ON_VSEL_MASK, WM831X_DC3_ON_CONFIG_VAL);

	wm831x_set_bits(wm831x, WM831X_DC1_DVS_CONTROL, WM831X_DC1_DVS_MASK, WM831X_DC1_DVS_VAL);
	wm831x_set_bits(wm831x, WM831X_DC2_DVS_CONTROL, WM831X_DC2_DVS_MASK, WM831X_DC2_DVS_VAL);

	wm831x_set_bits(wm831x, WM831X_GPIO7_CONTROL, WM831X_GPIO7_8_9_MASK, WM831X_GPIO7_VAL);
	wm831x_set_bits(wm831x, WM831X_GPIO8_CONTROL, WM831X_GPIO7_8_9_MASK, WM831X_GPIO8_VAL);
	wm831x_set_bits(wm831x, WM831X_GPIO9_CONTROL, WM831X_GPIO7_8_9_MASK, WM831X_GPIO9_VAL);

	wm831x_set_bits(wm831x, WM831X_DC1_CONTROL_1, WM831X_DC1_RATE_MASK, WM831X_DC1_CONTROL_1_RATE_VAL);
	wm831x_set_bits(wm831x, WM831X_DC2_CONTROL_1, WM831X_DC2_RATE_MASK, WM831X_DC2_CONTROL_1_RATE_VAL);

	wm831x_set_bits(wm831x, WM831X_STATUS_LED_1 , WM831X_STATUS_LED_MASK, WM831X_STATUS_LED_OFF);
	wm831x_set_bits(wm831x, WM831X_STATUS_LED_2 , WM831X_STATUS_LED_MASK, WM831X_STATUS_LED_ON);

#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	/*digital bypass VDDPU/VDDSOC/VDDARM*/
	reg = __raw_readl(ANADIG_REG_CORE);
	reg &= ~BM_ANADIG_REG_CORE_REG0_TRG;
	reg |= BF_ANADIG_REG_CORE_REG0_TRG(0x1f);
	reg &= ~BM_ANADIG_REG_CORE_REG1_TRG;
	reg |= BF_ANADIG_REG_CORE_REG1_TRG(0x1f);
	reg &= ~BM_ANADIG_REG_CORE_REG2_TRG;
	reg |= BF_ANADIG_REG_CORE_REG2_TRG(0x1f);
	__raw_writel(reg, ANADIG_REG_CORE);
	/* Mask the ANATOP brown out interrupt in the GPC. */
	reg = __raw_readl(gpc_base + 0x14);
	reg |= 0x80000000;
	__raw_writel(reg, gpc_base + 0x14);
#endif

	return 0;
}

#ifdef CONFIG_MX6_INTER_LDO_BYPASS
static struct regulator_consumer_supply tvbs_vddarm_consumers[] = {
	{
		.supply     = "VDDCORE_DCDC1",
	}
};

static struct regulator_consumer_supply tvbs_vddsoc_consumers[] = {
	{
		.supply     = "VDDSOC_DCDC2",
	}
};
#endif

#ifdef CONFIG_REGULATOR
static struct regulator_init_data tvbs_vddarm_dcdc1 = {
	.constraints = {
		.name = "vdd_arm",
		.min_uV = 100000,
		.max_uV = 1500000,
		.min_uA = 0,
		.max_uA = 4000000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = 0,
		//               .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
		//                      REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.always_on = 1,
		.boot_on = 1,
	},
#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	.num_consumer_supplies = ARRAY_SIZE(tvbs_vddarm_consumers),
	.consumer_supplies = tvbs_vddarm_consumers,
#endif
};

static struct regulator_init_data tvbs_vddsoc_dcdc2 = {
	.constraints = {
		.name = "vdd_soc",
		.min_uV = 100000,
		.max_uV = 1500000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = 0,
		.always_on = 1,
		.boot_on = 1,
	},
#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	.num_consumer_supplies = ARRAY_SIZE(tvbs_vddsoc_consumers),
	.consumer_supplies = tvbs_vddsoc_consumers,
#endif
};

static struct regulator_init_data tvbs_vddmem_1v5_dcdc3 = {
	.constraints = {
		.name = "vddmem",
		.min_uV = 1000000,
		.max_uV = 1500000,
		.valid_ops_mask = 0,//REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = 0,
		.always_on = 1,
		.boot_on = 1,
	},
#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	.num_consumer_supplies = ARRAY_SIZE(tvbs_vddsoc_consumers),
	.consumer_supplies = tvbs_vddsoc_consumers,
#endif
};

static struct regulator_init_data vldo1 = {
	.constraints = {
		.name = "VLDO1",
		.always_on = 1,
	},
};
static struct regulator_init_data vldo3 = {
	.constraints = {
		.name = "VLDO3",
		.always_on = 1,
	},
};

static struct regulator_init_data vldo7_1v8 = {
	.constraints = {
		.name = "VLDO7",
		.always_on = 1,
	},
};
static struct regulator_init_data vldo9 = {
	.constraints = {
		.name = "VLDO9",
		.always_on = 1,
	},
};
static struct regulator_init_data vldo10 = {
	.constraints = {
		.name = "VLDO10",
		.always_on = 1,
	},
};
#endif

static struct wm831x_pdata tvbs_wm8326_pdata = {
#ifdef CONFIG_REGULATOR
	.dcdc = {
		&tvbs_vddarm_dcdc1,  /* DCDC1 */
		&tvbs_vddsoc_dcdc2,  /* DCDC2 */
		&tvbs_vddmem_1v5_dcdc3, /* DCDC3 */
	},
	.ldo = {
		 &vldo1,        /* LDO1 */
		 NULL, /* LDO2 */
		 &vldo3,                /* LDO3 NC */
		 NULL,   /* LDO4 */
		 NULL,    /* LDO5 */
		 NULL,     /* LDO6 */
		 &vldo7_1v8,  /* LDO7 */
		 NULL,  /* LDO8 */
		 &vldo9,    /* LDO9 */
		 &vldo10,  /* LDO10 */
		 NULL,  /* LDO11 */
	},
#endif
	.post_init = wm8326_post_init,
};

static int wm831x_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct wm831x *wm831x;
	int ret;
	struct device_node *np = i2c->dev.of_node;

	printk("into wm831x_i2c_probe at wm831x-i2c.c");

	wm831x = devm_kzalloc(&i2c->dev, sizeof(struct wm831x), GFP_KERNEL);
	if (wm831x == NULL)
		return -ENOMEM;

	/* for tvbs */
	tvbs_wm8326_pdata.batt_adc_sel_gpio = of_get_named_gpio(np, "battadc-gpios", 0);
	if (gpio_is_valid(tvbs_wm8326_pdata.batt_adc_sel_gpio)) {
		ret = gpio_request_one(tvbs_wm8326_pdata.batt_adc_sel_gpio, GPIOF_OUT_INIT_HIGH,
					"Batt ADC sel");
		if (ret)
			pr_warn("failed to request battadc-sel-gpios gpio\n");
	} else
		dev_warn(&i2c->dev, "no batt_adc_sel pin available\n");

	tvbs_wm8326_pdata.backup_acok_gpio = of_get_named_gpio(np, "acok-gpios", 0);
	if (gpio_is_valid(tvbs_wm8326_pdata.backup_acok_gpio)) {
		ret = gpio_request_one(tvbs_wm8326_pdata.backup_acok_gpio, GPIOF_DIR_IN,
					"backup ACOK");
		if (ret)
			pr_warn("failed to request backup battery ACOK gpio\n");
	} else
		dev_warn(&i2c->dev, "no backup battery ACOK gpio pin available\n");

	tvbs_wm8326_pdata.backup_chgok_gpio = of_get_named_gpio(np, "chgok-gpios", 0);
	if (gpio_is_valid(tvbs_wm8326_pdata.backup_chgok_gpio)) {
		ret = gpio_request_one(tvbs_wm8326_pdata.backup_chgok_gpio, GPIOF_DIR_IN,
					"backup CHGOK");
		if (ret)
			pr_warn("failed to request backup battery CHGOK gpio\n");
	} else
		dev_warn(&i2c->dev, "no backup battery CHGOK gpio pin available\n");

	i2c_set_clientdata(i2c, wm831x);
	i2c->dev.platform_data = &tvbs_wm8326_pdata;
	wm831x->dev = &i2c->dev;

	wm831x->regmap = devm_regmap_init_i2c(i2c, &wm831x_regmap_config);
	if (IS_ERR(wm831x->regmap)) {
		ret = PTR_ERR(wm831x->regmap);
		dev_err(wm831x->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}
	//printk("=====i2c irq %d\n", i2c->irq);

	return wm831x_device_init(wm831x, id->driver_data, i2c->irq);
}

static int wm831x_i2c_remove(struct i2c_client *i2c)
{
	struct wm831x *wm831x = i2c_get_clientdata(i2c);

	wm831x_device_exit(wm831x);

	return 0;
}

static int wm831x_i2c_suspend(struct device *dev)
{
	struct wm831x *wm831x = dev_get_drvdata(dev);

	return wm831x_device_suspend(wm831x);
}

static int wm831x_i2c_poweroff(struct device *dev)
{
	struct wm831x *wm831x = dev_get_drvdata(dev);

	wm831x_device_shutdown(wm831x);

	return 0;
}

static const struct i2c_device_id wm831x_i2c_id[] = {
	{ "wm8310", WM8310 },
	{ "wm8311", WM8311 },
	{ "wm8312", WM8312 },
	{ "wm8320", WM8320 },
	{ "wm8321", WM8321 },
	{ "wm8325", WM8325 },
	{ "wm8326", WM8326 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wm831x_i2c_id);

static const struct of_device_id wm8623_of_match[] = {
       { .compatible = "wlf,wm8623", },
       { }
};
MODULE_DEVICE_TABLE(of, wm8623_of_match);

static const struct dev_pm_ops wm831x_pm_ops = {
	.suspend = wm831x_i2c_suspend,
	.poweroff = wm831x_i2c_poweroff,
};

static struct i2c_driver wm831x_i2c_driver = {
	.driver = {
		.name = "wm831x",
		.owner = THIS_MODULE,
		.pm = &wm831x_pm_ops,
		.of_match_table = wm8623_of_match,
	},
	.probe = wm831x_i2c_probe,
	.remove = wm831x_i2c_remove,
	.id_table = wm831x_i2c_id,
};

static int __init wm831x_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&wm831x_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register wm831x I2C driver: %d\n", ret);

	return ret;
}
subsys_initcall(wm831x_i2c_init);

static void __exit wm831x_i2c_exit(void)
{
	i2c_del_driver(&wm831x_i2c_driver);
}
module_exit(wm831x_i2c_exit);
