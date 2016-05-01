/*
 * PMU driver for Wolfson Microelectronics wm831x PMICs
 *
 * Copyright 2009 Wolfson Microelectronics PLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/irq.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include <linux/mfd/wm831x/core.h>
#include <linux/mfd/wm831x/auxadc.h>
#include <linux/mfd/wm831x/pmu.h>
#include <linux/mfd/wm831x/pdata.h>

#define	BATTERY_UPDATE_INTERVAL	5 /*seconds*/
#define ADC_SAMPLE_COUNT	6
#define PERCENT_UPDATE_THRESHOLD_COUNT	3

struct wm831x_power {
	struct wm831x *wm831x;
	struct power_supply wall;
	struct power_supply usb;
	struct power_supply battery;
	char wall_name[20];
	char usb_name[20];
	char battery_name[20];
	bool have_battery;
	struct delayed_work work;
//	struct work_struct work_acok;
	unsigned int interval;
	int voltage_uV;
	int percent;
	int old_percent;
	int first_delay_count;
	int battery_status;
#define	NR_VOLTAGE	12
	u32 saved_voltage[NR_VOLTAGE];
	int percent_minus_update_threshold;
	int percent_plus_update_threshold;
	struct mutex update_lock;
	bool acok_in;
};

typedef struct {
	u32 voltage;
	u32 percent;
} battery_capacity , *pbattery_capacity;

static battery_capacity chargingTable_backup[] = {
	{4050*2000,	99},
	{4040*2000,	98},
	{4020*2000,	97},
	{4010*2000,	96},
	{3990*2000,	95},
	{3980*2000,	94},
	{3970*2000,	93},
	{3960*2000,	92},
	{3950*2000,	91},
	{3940*2000,	90},
	{3930*2000,	85},
	{3920*2000,	81},
	{3910*2000,	77},
	{3900*2000,	73},
	{3890*2000,	70},
	{3860*2000,	65},
	{3830*2000,	60},
	{3780*2000,	55},
	{3760*2000,	50},
	{3740*2000,	45},
	{3720*2000,	40},
	{3700*2000,	35},
	{3680*2000,	30},
	{3660*2000,	25},
	{3640*2000,	20},
	{3620*2000,	17},
	{3600*2000,	14},
	{3580*2000,	13},
	{3560*2000,	12},
	{3540*2000,	11},
	{3520*2000,	10},
	{3500*2000,	9},
	{3480*2000,	8},
	{3460*2000,	7},
	{3440*2000,	6},
	{3430*2000,	5},
	{3420*2000,	4},
	{3020*2000,	0},
};

static battery_capacity dischargingTable_backup[] = {
	{4050*2000, 100},
	{4035*2000,	99},
	{4020*2000,	98},
	{4010*2000,	97},
	{4000*2000,	96},
	{3990*2000,	96},
	{3980*2000,	95},
	{3970*2000,	92},
	{3960*2000,	91},
	{3950*2000,	90},
	{3940*2000,	88},
	{3930*2000,	86},
	{3920*2000,	84},
	{3910*2000,	82},
	{3900*2000,	80},
	{3890*2000,	74},
	{3860*2000,	69},
	{3830*2000,	64},
	{3780*2000,	59},
	{3760*2000,	54},
	{3740*2000,	49},
	{3720*2000,	44},
	{3700*2000,	39},
	{3680*2000,	34},
	{3660*2000,	29},
	{3640*2000,	24},
	{3620*2000,	19},
	{3600*2000,	14},
	{3580*2000,	13},
	{3560*2000,	12},
	{3540*2000,	11},
	{3520*2000,	10},
	{3500*2000,	9},
	{3480*2000,	8},
	{3460*2000,	7},
	{3440*2000,	6},
	{3430*2000,	5},
	{3420*2000,	4},
	{3020*2000,	0},
};

static battery_capacity dischargingTable_main[] = {
	{4050*3000, 100},
	{4035*3000,	99},
	{4020*3000,	98},
	{4010*3000,	97},
	{4000*3000,	96},
	{3990*3000,	96},
	{3980*3000,	95},
	{3970*3000,	92},
	{3960*3000,	91},
	{3950*3000,	90},
	{3940*3000,	88},
	{3930*3000,	86},
	{3920*3000,	84},
	{3910*3000,	82},
	{3900*3000,	80},
	{3890*3000,	74},
	{3860*3000,	69},
	{3830*3000,	64},
	{3780*3000,	59},
	{3760*3000,	54},
	{3740*3000,	49},
	{3720*3000,	44},
	{3700*3000,	39},
	{3680*3000,	34},
	{3660*3000,	29},
	{3640*3000,	24},
	{3620*3000,	19},
	{3600*3000,	14},
	{3580*3000,	13},
	{3560*3000,	12},
	{3540*3000,	11},
	{3520*3000,	10},
	{3500*3000,	9},
	{3480*3000,	8},
	{3460*3000,	7},
	{3440*3000,	6},
	{3430*3000,	5},
	{3420*3000,	4},
	{3020*3000,	0},
};

static u32 calibration_voltage(struct wm831x_power *power);
static int wm831x_bat_check_status(struct wm831x *wm831x, int *status);

static u32 calibrate_backup_battery_capability_percent(struct wm831x_power *power)
{
	u8 i;
	pbattery_capacity pTable;
	u32 tableSize;

	wm831x_bat_check_status(power->wm831x, &power->battery_status);

	if (power->battery_status == POWER_SUPPLY_STATUS_DISCHARGING) {
		pTable = dischargingTable_backup;
		tableSize = sizeof(dischargingTable_backup)/
			sizeof(dischargingTable_backup[0]);
	} else {
		pTable = chargingTable_backup;
		tableSize = sizeof(chargingTable_backup)/
			sizeof(chargingTable_backup[0]);
	}
	for (i = 0; i < tableSize; i++) {
		if (power->voltage_uV >= pTable[i].voltage)
			return	pTable[i].percent;
	}

	return 0;
}

static u32 calibrate_battery_capability_percent(int voltage_uV)
{
	u8 i;
	pbattery_capacity pTable;
	u32 tableSize;

	/* ONLY discharging */
//	if (power->battery_status == POWER_SUPPLY_STATUS_DISCHARGING) {
		pTable = dischargingTable_main;
		tableSize = sizeof(dischargingTable_main)/
			sizeof(dischargingTable_main[0]);
//	} else {
//		pTable = chargingTable;
//		tableSize = sizeof(chargingTable)/
//			sizeof(chargingTable[0]);
//	}
	for (i = 0; i < tableSize; i++) {
		if (voltage_uV >= pTable[i].voltage)
			return	pTable[i].percent;
	}

	return 0;
}

static int main_battery_voltage(struct wm831x_power *power,
			struct wm831x_pdata *wm831x_pdata)
{
	int voltage;

	mutex_lock(&power->update_lock);
	gpio_set_value(wm831x_pdata->batt_adc_sel_gpio, 0);
	usleep_range(1000, 2000);
	voltage = calibration_voltage(power);
	//pr_info("++voltage_uV %d\n", voltage);
	gpio_set_value(wm831x_pdata->batt_adc_sel_gpio, 1);
	usleep_range(1000, 2000);
	mutex_unlock(&power->update_lock);

	return voltage;
}

static int backup_battery_is_charging(struct wm831x_pdata *wm831x_pdata)
{

	return (!gpio_get_value(wm831x_pdata->backup_acok_gpio)
			&& !gpio_get_value(wm831x_pdata->backup_chgok_gpio));
}

#if 0
static int wm831x_power_check_online(struct wm831x *wm831x, int supply,
				     union power_supply_propval *val)
{
	int ret;

	ret = wm831x_reg_read(wm831x, WM831X_SYSTEM_STATUS);
	if (ret < 0)
		return ret;

	if (ret & supply)
		val->intval = 1;
	else
		val->intval = 0;

	return 0;
}
#endif

static int wm831x_power_read_voltage(struct wm831x *wm831x,
				     enum wm831x_auxadc src,
				     union power_supply_propval *val)
{
	int ret;

	ret = wm831x_auxadc_read_uv(wm831x, src);
	if (ret >= 0)
		val->intval = ret;

	return ret;
}

/*********************************************************************
 *		WALL Power
 *********************************************************************/
static int wm831x_wall_get_prop(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct wm831x_power *wm831x_power = dev_get_drvdata(psy->dev->parent);
	struct wm831x *wm831x = wm831x_power->wm831x;
	struct wm831x_pdata *wm831x_pdata = wm831x->dev->platform_data;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
//		ret = wm831x_power_check_online(wm831x, WM831X_PWR_WALL, val);
		val->intval = !gpio_get_value(wm831x_pdata->backup_acok_gpio);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
//		ret = wm831x_power_read_voltage(wm831x, WM831X_AUX_WALL, val);
		val->intval = wm831x_power->voltage_uV;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property wm831x_wall_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

/*********************************************************************
 *		USB Power
 *********************************************************************/
#if 0
static int wm831x_usb_get_prop(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct wm831x_power *wm831x_power = dev_get_drvdata(psy->dev->parent);
	struct wm831x *wm831x = wm831x_power->wm831x;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = wm831x_power_check_online(wm831x, WM831X_PWR_USB, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = wm831x_power_read_voltage(wm831x, WM831X_AUX_USB, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property wm831x_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

#endif
/*********************************************************************
 *		Battery properties
 *********************************************************************/

struct chg_map {
	int val;
	int reg_val;
};

static struct chg_map trickle_ilims[] = {
	{  50, 0 << WM831X_CHG_TRKL_ILIM_SHIFT },
	{ 100, 1 << WM831X_CHG_TRKL_ILIM_SHIFT },
	{ 150, 2 << WM831X_CHG_TRKL_ILIM_SHIFT },
	{ 200, 3 << WM831X_CHG_TRKL_ILIM_SHIFT },
};

static struct chg_map vsels[] = {
	{ 4050, 0 << WM831X_CHG_VSEL_SHIFT },
	{ 4100, 1 << WM831X_CHG_VSEL_SHIFT },
	{ 4150, 2 << WM831X_CHG_VSEL_SHIFT },
	{ 4200, 3 << WM831X_CHG_VSEL_SHIFT },
};

static struct chg_map fast_ilims[] = {
	{    0,  0 << WM831X_CHG_FAST_ILIM_SHIFT },
	{   50,  1 << WM831X_CHG_FAST_ILIM_SHIFT },
	{  100,  2 << WM831X_CHG_FAST_ILIM_SHIFT },
	{  150,  3 << WM831X_CHG_FAST_ILIM_SHIFT },
	{  200,  4 << WM831X_CHG_FAST_ILIM_SHIFT },
	{  250,  5 << WM831X_CHG_FAST_ILIM_SHIFT },
	{  300,  6 << WM831X_CHG_FAST_ILIM_SHIFT },
	{  350,  7 << WM831X_CHG_FAST_ILIM_SHIFT },
	{  400,  8 << WM831X_CHG_FAST_ILIM_SHIFT },
	{  450,  9 << WM831X_CHG_FAST_ILIM_SHIFT },
	{  500, 10 << WM831X_CHG_FAST_ILIM_SHIFT },
	{  600, 11 << WM831X_CHG_FAST_ILIM_SHIFT },
	{  700, 12 << WM831X_CHG_FAST_ILIM_SHIFT },
	{  800, 13 << WM831X_CHG_FAST_ILIM_SHIFT },
	{  900, 14 << WM831X_CHG_FAST_ILIM_SHIFT },
	{ 1000, 15 << WM831X_CHG_FAST_ILIM_SHIFT },
};

static struct chg_map eoc_iterms[] = {
	{ 20, 0 << WM831X_CHG_ITERM_SHIFT },
	{ 30, 1 << WM831X_CHG_ITERM_SHIFT },
	{ 40, 2 << WM831X_CHG_ITERM_SHIFT },
	{ 50, 3 << WM831X_CHG_ITERM_SHIFT },
	{ 60, 4 << WM831X_CHG_ITERM_SHIFT },
	{ 70, 5 << WM831X_CHG_ITERM_SHIFT },
	{ 80, 6 << WM831X_CHG_ITERM_SHIFT },
	{ 90, 7 << WM831X_CHG_ITERM_SHIFT },
};

static struct chg_map chg_times[] = {
	{  60,  0 << WM831X_CHG_TIME_SHIFT },
	{  90,  1 << WM831X_CHG_TIME_SHIFT },
	{ 120,  2 << WM831X_CHG_TIME_SHIFT },
	{ 150,  3 << WM831X_CHG_TIME_SHIFT },
	{ 180,  4 << WM831X_CHG_TIME_SHIFT },
	{ 210,  5 << WM831X_CHG_TIME_SHIFT },
	{ 240,  6 << WM831X_CHG_TIME_SHIFT },
	{ 270,  7 << WM831X_CHG_TIME_SHIFT },
	{ 300,  8 << WM831X_CHG_TIME_SHIFT },
	{ 330,  9 << WM831X_CHG_TIME_SHIFT },
	{ 360, 10 << WM831X_CHG_TIME_SHIFT },
	{ 390, 11 << WM831X_CHG_TIME_SHIFT },
	{ 420, 12 << WM831X_CHG_TIME_SHIFT },
	{ 450, 13 << WM831X_CHG_TIME_SHIFT },
	{ 480, 14 << WM831X_CHG_TIME_SHIFT },
	{ 510, 15 << WM831X_CHG_TIME_SHIFT },
};

static void wm831x_battey_apply_config(struct wm831x *wm831x,
				       struct chg_map *map, int count, int val,
				       int *reg, const char *name,
				       const char *units)
{
	int i;

	for (i = 0; i < count; i++)
		if (val == map[i].val)
			break;
	if (i == count) {
		dev_err(wm831x->dev, "Invalid %s %d%s\n",
			name, val, units);
	} else {
		*reg |= map[i].reg_val;
		dev_dbg(wm831x->dev, "Set %s of %d%s\n", name, val, units);
	}
}

static void wm831x_config_battery(struct wm831x *wm831x)
{
	struct wm831x_pdata *wm831x_pdata = wm831x->dev->platform_data;
	struct wm831x_battery_pdata *pdata;
	int ret, reg1, reg2;

	if (!wm831x_pdata || !wm831x_pdata->battery) {
		dev_warn(wm831x->dev,
			 "No battery charger configuration\n");
		return;
	}

	pdata = wm831x_pdata->battery;

	reg1 = 0;
	reg2 = 0;

	if (!pdata->enable) {
		dev_info(wm831x->dev, "Battery charger disabled\n");
		return;
	}

	reg1 |= WM831X_CHG_ENA;
	if (pdata->off_mask)
		reg2 |= WM831X_CHG_OFF_MSK;
	if (pdata->fast_enable)
		reg1 |= WM831X_CHG_FAST;

	wm831x_battey_apply_config(wm831x, trickle_ilims,
				   ARRAY_SIZE(trickle_ilims),
				   pdata->trickle_ilim, &reg2,
				   "trickle charge current limit", "mA");

	wm831x_battey_apply_config(wm831x, vsels, ARRAY_SIZE(vsels),
				   pdata->vsel, &reg2,
				   "target voltage", "mV");

	wm831x_battey_apply_config(wm831x, fast_ilims, ARRAY_SIZE(fast_ilims),
				   pdata->fast_ilim, &reg2,
				   "fast charge current limit", "mA");

	wm831x_battey_apply_config(wm831x, eoc_iterms, ARRAY_SIZE(eoc_iterms),
				   pdata->eoc_iterm, &reg1,
				   "end of charge current threshold", "mA");

	wm831x_battey_apply_config(wm831x, chg_times, ARRAY_SIZE(chg_times),
				   pdata->timeout, &reg2,
				   "charger timeout", "min");

	ret = wm831x_reg_unlock(wm831x);
	if (ret != 0) {
		dev_err(wm831x->dev, "Failed to unlock registers: %d\n", ret);
		return;
	}

	ret = wm831x_set_bits(wm831x, WM831X_CHARGER_CONTROL_1,
			      WM831X_CHG_ENA_MASK |
			      WM831X_CHG_FAST_MASK |
			      WM831X_CHG_ITERM_MASK,
			      reg1);
	if (ret != 0)
		dev_err(wm831x->dev, "Failed to set charger control 1: %d\n",
			ret);

	ret = wm831x_set_bits(wm831x, WM831X_CHARGER_CONTROL_2,
			      WM831X_CHG_OFF_MSK |
			      WM831X_CHG_TIME_MASK |
			      WM831X_CHG_FAST_ILIM_MASK |
			      WM831X_CHG_TRKL_ILIM_MASK |
			      WM831X_CHG_VSEL_MASK,
			      reg2);
	if (ret != 0)
		dev_err(wm831x->dev, "Failed to set charger control 2: %d\n",
			ret);

	wm831x_reg_lock(wm831x);
}

static int wm831x_bat_check_status(struct wm831x *wm831x, int *status)
{
	int ret;
	struct wm831x_pdata *wm831x_pdata = wm831x->dev->platform_data;

//	ret = wm831x_reg_read(wm831x, WM831X_SYSTEM_STATUS);
//	if (ret < 0)
//		return ret;

//	if (ret & WM831X_PWR_SRC_BATT) {	// NOTE: wm8326 not enable wm831x_power by default -- no WM831X_PWR_SRC_BATT bit
//		*status = POWER_SUPPLY_STATUS_DISCHARGING;
//		return 0;
//	}

	if (backup_battery_is_charging(wm831x_pdata))
		*status = POWER_SUPPLY_STATUS_CHARGING;
	else
		*status = POWER_SUPPLY_STATUS_DISCHARGING;

	return 0;

#if 0
	ret = wm831x_reg_read(wm831x, WM831X_CHARGER_STATUS);
	if (ret < 0)
		return ret;

	switch (ret & WM831X_CHG_STATE_MASK) {
	case WM831X_CHG_STATE_OFF:
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case WM831X_CHG_STATE_TRICKLE:
	case WM831X_CHG_STATE_FAST:
		*status = POWER_SUPPLY_STATUS_CHARGING;
		break;

	default:
		*status = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return 0;
#endif
}

static int wm831x_bat_check_type(struct wm831x *wm831x, int *type)
{
	int ret;

	ret = wm831x_reg_read(wm831x, WM831X_CHARGER_STATUS);
	if (ret < 0)
		return ret;

	switch (ret & WM831X_CHG_STATE_MASK) {
	case WM831X_CHG_STATE_TRICKLE:
	case WM831X_CHG_STATE_TRICKLE_OT:
		*type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case WM831X_CHG_STATE_FAST:
	case WM831X_CHG_STATE_FAST_OT:
		*type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	default:
		*type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	}

	return 0;
}

static int wm831x_bat_check_health(struct wm831x *wm831x, int *health)
{
	int ret;

	*health = POWER_SUPPLY_HEALTH_GOOD;
	return 0;

	ret = wm831x_reg_read(wm831x, WM831X_CHARGER_STATUS);
	if (ret < 0)
		return ret;

	if (ret & WM831X_BATT_HOT_STS) {
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
		return 0;
	}

	if (ret & WM831X_BATT_COLD_STS) {
		*health = POWER_SUPPLY_HEALTH_COLD;
		return 0;
	}

	if (ret & WM831X_BATT_OV_STS) {
		*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		return 0;
	}

	switch (ret & WM831X_CHG_STATE_MASK) {
	case WM831X_CHG_STATE_TRICKLE_OT:
	case WM831X_CHG_STATE_FAST_OT:
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	case WM831X_CHG_STATE_DEFECTIVE:
		*health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;
	default:
		*health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	}

	return 0;
}

static int wm831x_bat_get_prop(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct wm831x_power *wm831x_power = dev_get_drvdata(psy->dev->parent);
	struct wm831x *wm831x = wm831x_power->wm831x;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = wm831x_bat_check_status(wm831x, &val->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
#if 0
		ret = wm831x_power_check_online(wm831x, WM831X_PWR_SRC_BATT,
						val);
#endif
		/* Assume Backup battery is always online */
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = wm831x_power_read_voltage(wm831x, WM831X_AUX_BATT, val);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = wm831x_bat_check_health(wm831x, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		//ret = wm831x_bat_check_type(wm831x, &val->intval);
		val->intval = POWER_SUPPLY_TYPE_BATTERY;	// TODO
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = wm831x_power->percent < 0 ? 0 :
				(wm831x_power->percent > 100 ? 100 : wm831x_power->percent);
#ifdef JUST_FOR_DEBUG
		val->intval = 78;
#endif
		val->intval = (val->intval > 10) ? val->intval : 11;
		//pr_info("capacity %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property wm831x_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

#if 0
static const char *wm831x_bat_irqs[] = {
	"BATT HOT",
	"BATT COLD",
	"BATT FAIL",
	"OV",
	"END",
	"TO",
	"MODE",
	"START",
};

static irqreturn_t wm831x_bat_irq(int irq, void *data)
{
	struct wm831x_power *wm831x_power = data;
	struct wm831x *wm831x = wm831x_power->wm831x;

	dev_dbg(wm831x->dev, "Battery status changed: %d\n", irq);

	/* The battery charger is autonomous so we don't need to do
	 * anything except kick user space */
	if (wm831x_power->have_battery)
		power_supply_changed(&wm831x_power->battery);

	return IRQ_HANDLED;
}
#endif


/*********************************************************************
 *		Initialisation
 *********************************************************************/

static irqreturn_t wm831x_syslo_irq(int irq, void *data)
{
	struct wm831x_power *wm831x_power = data;
	struct wm831x *wm831x = wm831x_power->wm831x;

	/* Not much we can actually *do* but tell people for
	 * posterity, we're probably about to run out of power. */
	dev_crit(wm831x->dev, "SYSVDD under voltage\n");

	return IRQ_HANDLED;
}

static irqreturn_t wm831x_pwr_src_irq(int irq, void *data)
{
	struct wm831x_power *wm831x_power = data;
	struct wm831x *wm831x = wm831x_power->wm831x;

	dev_dbg(wm831x->dev, "Power source changed\n");

	/* Just notify for everything - little harm in overnotifying. */
	if (wm831x_power->have_battery)
		power_supply_changed(&wm831x_power->battery);
	power_supply_changed(&wm831x_power->usb);
	power_supply_changed(&wm831x_power->wall);

	return IRQ_HANDLED;
}

static u32 calibration_voltage(struct wm831x_power *power)
{
	volatile u32 voltage_data = 1;
	u32 voltage_each;
	int i = 0, j = 0;
	int ret;

	/* simple average */
	for (i = 0; i < ADC_SAMPLE_COUNT; i++) {
		ret = wm831x_power_read_voltage(power->wm831x, WM831X_AUX_BATT, &voltage_each);
		if (ret < 0) {
			pr_err("%s: read voltage failure\n", __func__);
			j++;
			continue;
		}
		voltage_data += voltage_each;
	}

	if (j < ADC_SAMPLE_COUNT)
		voltage_data = voltage_data / (ADC_SAMPLE_COUNT - j);
	else
		voltage_data = 0;

	return voltage_data;
}

static void wm831x_battery_update_status(struct wm831x_power *power)
{
	u32 voltage = 0;
	int ret;

	mutex_lock(&power->update_lock);
	power->voltage_uV = calibration_voltage(power);
#ifdef	DEBUG_BATT
	pr_info("voltage_uV %d\n", power->voltage_uV);
#endif
	power->percent = calibrate_backup_battery_capability_percent(power);
	mutex_unlock(&power->update_lock);

#ifdef JUST_FOR_DEBUG
	power->percent = 78;
#endif
	power->percent = (power->percent) > 10 ? power->percent : 11;
#ifdef	DEBUG_BATT
	pr_info("percent %d\n", power->percent);
#endif

	if (power->first_delay_count < 2) {
		power->first_delay_count = power->first_delay_count + 1;
		power->old_percent = power->percent;
		power_supply_changed(&power->battery);
		power_supply_changed(&power->wall);
	}

	if (power->battery_status == POWER_SUPPLY_STATUS_CHARGING) {
#ifdef	DEBUG_BATT
		printk("charging percent %d, old_percent %d\n",
			power->percent, power->old_percent);
#endif
		if (power->percent < power->old_percent) {
			power->percent = power->old_percent;
			power->percent_plus_update_threshold = 0;
		} else if (power->percent > power->old_percent) {
			power->percent_plus_update_threshold++;
		}

		if (power->percent_plus_update_threshold >=
			PERCENT_UPDATE_THRESHOLD_COUNT) {
			power->old_percent++;// = power->percent;
			power->percent = power->old_percent;
			power_supply_changed(&power->battery);
			power_supply_changed(&power->wall);

			power->percent_plus_update_threshold = 0;
		}
	} else {
#ifdef	DEBUG_BATT
		printk("discharging percent %d, old_percent %d\n",
			power->percent, power->old_percent);
#endif
		if (power->percent > power->old_percent) {
			power->percent = power->old_percent;
			power->percent_minus_update_threshold = 0;
		} else if (power->percent < power->old_percent) {
			power->percent_minus_update_threshold++;
		}

		if (power->percent_minus_update_threshold >=
			PERCENT_UPDATE_THRESHOLD_COUNT) {
			power->old_percent--;// = power->percent;
			power->percent = power->old_percent;
			power_supply_changed(&power->battery);
			power_supply_changed(&power->wall);

			power->percent_minus_update_threshold = 0;
		}
	}

}

static void wm831x_battery_work(struct work_struct *w)
{
	struct wm831x_power *power;

	power = container_of(w, struct wm831x_power, work);
	power->interval = HZ * BATTERY_UPDATE_INTERVAL;

	wm831x_battery_update_status(power);
//	dev_dbg(data->dev, "battery voltage: %4d mV\n", data->voltage_uV);
//	dev_dbg(data->dev, "charger online status: %d\n",
//		data->charger_online);
//	dev_dbg(data->dev, "battery status : %d\n" , data->battery_status);
//	dev_dbg(data->dev, "battery capacity percent: %3d\n", data->percent);
//	dev_dbg(data->dev, "data->usb_in: %x , data->ta_in: %x\n",
//		data->usb_in, data->ta_in);
	/* reschedule for the next time */
	schedule_delayed_work(&power->work, power->interval);
}

#if 0
static void acok_work(struct work_struct *w)
{
	struct wm831x_power *power;

	power = container_of(w, struct wm831x_power, work_acok);
	mutex_lock(&power->update_lock);
	power->voltage_uV = calibration_voltage(power);
	//pr_info("voltage_uV %d\n", power->voltage_uV);
	power->old_percent = power->percent =
			calibrate_battery_capability_percent(power);
	power_supply_changed(&power->battery);
	mutex_unlock(&power->update_lock);
}
#endif

static ssize_t main_battery_capacity_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	int main_battery_capacity, voltage;
	struct wm831x_power *power = dev_get_drvdata(dev);
	struct wm831x *wm831x = dev_get_drvdata(dev->parent);
	struct wm831x_pdata *wm831x_pdata = wm831x->dev->platform_data;

	voltage = main_battery_voltage(power, wm831x_pdata);

	main_battery_capacity = calibrate_battery_capability_percent(voltage);

	return sprintf(buf, "%d\n", main_battery_capacity);
}

static DEVICE_ATTR(main_battery_capacity, 0444,
		   main_battery_capacity_show, NULL);

static ssize_t main_battery_online_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	int main_battery_online;
//	struct wm831x_power *power = dev_get_drvdata(dev);
	struct wm831x *wm831x = dev_get_drvdata(dev->parent);
	struct wm831x_pdata *wm831x_pdata = wm831x->dev->platform_data;

	main_battery_online = !gpio_get_value(wm831x_pdata->backup_acok_gpio);

	return sprintf(buf, "%d\n", main_battery_online);
}
static DEVICE_ATTR(main_battery_online, 0444,
		   main_battery_online_show, NULL);

static ssize_t backup_battery_charging_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	int backup_battery_charging;
	struct wm831x *wm831x = dev_get_drvdata(dev->parent);
	struct wm831x_pdata *wm831x_pdata = wm831x->dev->platform_data;

	backup_battery_charging = backup_battery_is_charging(wm831x_pdata);

	return sprintf(buf, "%d\n", backup_battery_charging);
}

static DEVICE_ATTR(backup_battery_charging, 0444,
		   backup_battery_charging_show, NULL);

static struct attribute *backup_attrs[] = {
	&dev_attr_main_battery_capacity.attr,
	&dev_attr_main_battery_online.attr,
	&dev_attr_backup_battery_charging.attr,
	NULL
};

static const struct attribute_group backup_attr_group = {
	.attrs = backup_attrs,
};

static irqreturn_t gpio_acok_irq_handler(int irq, void *data)
{
	struct wm831x_power *power = data;
	struct wm831x_pdata *wm831x_pdata = power->wm831x->dev->platform_data;
	bool acok_in = false;

//	schedule_work(&power->work_acok);
	acok_in = !gpio_get_value(wm831x_pdata->backup_acok_gpio);

	if (acok_in == power->acok_in)
		return IRQ_HANDLED;

	power->acok_in = acok_in;
	dev_info(power->wm831x->dev, "Charger %s.\n", acok_in ?
			"Connected" : "Disconnected");

	wm831x_bat_check_status(power->wm831x, &power->battery_status);
	power_supply_changed(&power->battery);
	power_supply_changed(&power->wall);

	return IRQ_HANDLED;
}

static int wm831x_power_probe(struct platform_device *pdev)
{
	struct wm831x *wm831x = dev_get_drvdata(pdev->dev.parent);
	struct wm831x_pdata *wm831x_pdata = wm831x->dev->platform_data;
	struct wm831x_power *power;
	struct power_supply *usb;
	struct power_supply *battery;
	struct power_supply *wall;
	int ret, irq, i;

	power = kzalloc(sizeof(struct wm831x_power), GFP_KERNEL);
	if (power == NULL)
		return -ENOMEM;

	power->wm831x = wm831x;
	platform_set_drvdata(pdev, power);

	usb = &power->usb;
	battery = &power->battery;
	wall = &power->wall;

	if (wm831x_pdata && wm831x_pdata->wm831x_num) {
		snprintf(power->wall_name, sizeof(power->wall_name),
			 "wm831x-wall.%d", wm831x_pdata->wm831x_num);
		snprintf(power->battery_name, sizeof(power->wall_name),
			 "wm831x-battery.%d", wm831x_pdata->wm831x_num);
		snprintf(power->usb_name, sizeof(power->wall_name),
			 "wm831x-usb.%d", wm831x_pdata->wm831x_num);
	} else {
		snprintf(power->wall_name, sizeof(power->wall_name),
			 "wm831x-wall");
		snprintf(power->battery_name, sizeof(power->wall_name),
			 "wm831x-battery");
		snprintf(power->usb_name, sizeof(power->wall_name),
			 "wm831x-usb");
	}

	/* We ignore configuration failures since we can still read back
	 * the status without enabling the charger.
	 */
	wm831x_config_battery(wm831x);

	wall->name = power->wall_name;
	wall->type = POWER_SUPPLY_TYPE_MAINS;
	wall->properties = wm831x_wall_props;
	wall->num_properties = ARRAY_SIZE(wm831x_wall_props);
	wall->get_property = wm831x_wall_get_prop;
	ret = power_supply_register(&pdev->dev, wall);
	if (ret)
		goto err_kmalloc;

#if 0
	usb->name = power->usb_name,
	usb->type = POWER_SUPPLY_TYPE_USB;
	usb->properties = wm831x_usb_props;
	usb->num_properties = ARRAY_SIZE(wm831x_usb_props);
	usb->get_property = wm831x_usb_get_prop;
	ret = power_supply_register(&pdev->dev, usb);
	if (ret)
		goto err_wall;
#endif
	ret = wm831x_reg_read(wm831x, WM831X_CHARGER_CONTROL_1);
	if (ret < 0)
		goto err_wall;
	power->have_battery = ret & WM831X_CHG_ENA;

	/*printk("%s %d ===\n", __func__, power->have_battery);*/
	power->have_battery = 1;	/* force to have battery, Robby */
	power->old_percent = 100;
	power->percent_minus_update_threshold = 0;
	power->percent_plus_update_threshold = 0;
	for (i = 0; i < NR_VOLTAGE; i++)
		power->saved_voltage[i] = 0;

	if (power->have_battery) {
		    battery->name = power->battery_name;
		    battery->type = POWER_SUPPLY_TYPE_BATTERY;
		    battery->properties = wm831x_bat_props;
		    battery->num_properties = ARRAY_SIZE(wm831x_bat_props);
		    battery->get_property = wm831x_bat_get_prop;
		    battery->use_for_apm = 1;
		    ret = power_supply_register(&pdev->dev, battery);
		    if (ret)
			    goto err_usb;
	}

	power->first_delay_count = 0;
	INIT_DELAYED_WORK(&power->work, wm831x_battery_work);
//	INIT_WORK(&power->work_acok, acok_work);
	schedule_delayed_work(&power->work, power->interval);

	mutex_init(&power->update_lock);
//	device_create_file(&pdev->dev, &dev_attr_main_battery_capacity);
//	device_create_file(&pdev->dev, &dev_attr_backup_battery_charging);
	sysfs_create_group(&pdev->dev.kobj, &backup_attr_group);

	if (gpio_is_valid(wm831x_pdata->backup_acok_gpio)) {
		power->acok_in = !gpio_get_value(wm831x_pdata->backup_acok_gpio);

		ret = request_threaded_irq(gpio_to_irq(wm831x_pdata->backup_acok_gpio), NULL,
			   gpio_acok_irq_handler,
			   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			   "gpio_acok_irq", power);
		if (ret) {
			dev_warn(&pdev->dev, "gpio ACOK irq handler not requested\n");
		} else {
			;//enable_irq_wake(gpio_to_irq(wm831x_pdata->backup_acok_gpio));
		}
	}
#if 0	/* no irq for pmic on tvbs */
	irq = wm831x_irq(wm831x, platform_get_irq_byname(pdev, "SYSLO"));
	ret = request_threaded_irq(irq, NULL, wm831x_syslo_irq,
				   IRQF_TRIGGER_RISING, "System power low",
				   power);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to request SYSLO IRQ %d: %d\n",
			irq, ret);
		goto err_battery;
	}

	irq = wm831x_irq(wm831x, platform_get_irq_byname(pdev, "PWR SRC"));
	ret = request_threaded_irq(irq, NULL, wm831x_pwr_src_irq,
				   IRQF_TRIGGER_RISING, "Power source",
				   power);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to request PWR SRC IRQ %d: %d\n",
			irq, ret);
		goto err_syslo;
	}

	for (i = 0; i < ARRAY_SIZE(wm831x_bat_irqs); i++) {
		irq = wm831x_irq(wm831x,
				 platform_get_irq_byname(pdev,
							 wm831x_bat_irqs[i]));
		ret = request_threaded_irq(irq, NULL, wm831x_bat_irq,
					   IRQF_TRIGGER_RISING,
					   wm831x_bat_irqs[i],
					   power);
		if (ret != 0) {
			dev_err(&pdev->dev,
				"Failed to request %s IRQ %d: %d\n",
				wm831x_bat_irqs[i], irq, ret);
			goto err_bat_irq;
		}
	}
#endif

	return ret;

#if 0
err_bat_irq:
	for (; i >= 0; i--) {
		irq = platform_get_irq_byname(pdev, wm831x_bat_irqs[i]);
		free_irq(irq, power);
	}
	irq = wm831x_irq(wm831x, platform_get_irq_byname(pdev, "PWR SRC"));
	free_irq(irq, power);
err_syslo:
	irq = wm831x_irq(wm831x, platform_get_irq_byname(pdev, "SYSLO"));
	free_irq(irq, power);
err_battery:
	if (power->have_battery)
		power_supply_unregister(battery);
#endif
err_usb:
	power_supply_unregister(usb);
err_wall:
	power_supply_unregister(wall);
err_kmalloc:
	kfree(power);
	return ret;
}

static int wm831x_power_remove(struct platform_device *pdev)
{
	struct wm831x_power *wm831x_power = platform_get_drvdata(pdev);
	struct wm831x *wm831x = wm831x_power->wm831x;
	int irq, i;

#if 0
	for (i = 0; i < ARRAY_SIZE(wm831x_bat_irqs); i++) {
		irq = wm831x_irq(wm831x, 
				 platform_get_irq_byname(pdev,
							 wm831x_bat_irqs[i]));
		free_irq(irq, wm831x_power);
	}

	irq = wm831x_irq(wm831x, platform_get_irq_byname(pdev, "PWR SRC"));
	free_irq(irq, wm831x_power);

	irq = wm831x_irq(wm831x, platform_get_irq_byname(pdev, "SYSLO"));
	free_irq(irq, wm831x_power);
#endif

	if (wm831x_power->have_battery)
		power_supply_unregister(&wm831x_power->battery);
	power_supply_unregister(&wm831x_power->wall);
	power_supply_unregister(&wm831x_power->usb);
	kfree(wm831x_power);
	return 0;
}

static int wm831x_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	struct wm831x_power *power = platform_get_drvdata(pdev);
	struct wm831x_pdata *wm831x_pdata;
	int irq;

	if (power) {
		wm831x_pdata = power->wm831x->dev->platform_data;

		if (power->acok_in==false && device_may_wakeup(&pdev->dev)) {
			irq = gpio_to_irq(wm831x_pdata->backup_acok_gpio);
			enable_irq_wake(irq);
		}
		cancel_delayed_work(&power->work);
	}

	return 0;
}

static int wm831x_resume(struct platform_device *pdev)
{
	struct wm831x_power *power = platform_get_drvdata(pdev);
	struct wm831x_pdata *wm831x_pdata;
	int irq;
	bool acok_in = false;

	if (power) {
		wm831x_pdata = power->wm831x->dev->platform_data;

		if (device_may_wakeup(&pdev->dev)) {
			irq = gpio_to_irq(wm831x_pdata->backup_acok_gpio);
			disable_irq_wake(irq);
		}

		acok_in = !gpio_get_value(wm831x_pdata->backup_acok_gpio);
		if (acok_in != power->acok_in) {
			power->acok_in = acok_in;
			dev_info(power->wm831x->dev, "Charger %s.\n", acok_in ?
					"Connected" : "Disconnected");

			wm831x_bat_check_status(power->wm831x, &power->battery_status);
			power_supply_changed(&power->battery);
			power_supply_changed(&power->wall);
		}

		schedule_delayed_work(&power->work, power->interval);
	}

	return 0;
}

static struct platform_driver wm831x_power_driver = {
	.probe = wm831x_power_probe,
	.remove = wm831x_power_remove,
	.suspend = wm831x_suspend,
	.resume = wm831x_resume,
	.driver = {
		.name = "wm831x-power",
	},
};

module_platform_driver(wm831x_power_driver);

MODULE_DESCRIPTION("Power supply driver for WM831x PMICs");
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:wm831x-power");
