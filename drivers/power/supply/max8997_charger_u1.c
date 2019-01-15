/*
 *  max8997-charger.c
 *  MAXIM 8997 charger interface driver
 *
 *  Copyright (C) 2010 Samsung Electronics
 *
 *  <ms925.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/mfd/max8997.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/mfd/max8997-private.h>
#include <linux/irqdomain.h>
#include <linux/regmap.h>


/* MAX8997_REG_STATUS4 */
#define DCINOK_SHIFT		1
#define DCINOK_MASK		(1 << DCINOK_SHIFT)
#define DETBAT_SHIFT		2
#define DETBAT_MASK		(1 << DETBAT_SHIFT)

/*  MAX8997_REG_MBCCTRL1 */
#define TFCH_SHIFT		4
#define TFCH_MASK		(7 << TFCH_SHIFT)

/*  MAX8997_REG_MBCCTRL2 */
#define MBCHOSTEN_SHIFT		6
#define VCHGR_FC_SHIFT		7
#define MBCHOSTEN_MASK		(1 << MBCHOSTEN_SHIFT)
#define VCHGR_FC_MASK		(1 << VCHGR_FC_SHIFT)

/*  MAX8997_REG_MBCCTRL3 */
#define MBCCV_SHIFT		0
#define MBCCV_MASK		(0xF << MBCCV_SHIFT)

/* MAX8997_REG_MBCCTRL4 */
#define MBCICHFC_SHIFT		0
#define MBCICHFC_MASK		(0xF << MBCICHFC_SHIFT)

/* MAX8997_REG_MBCCTRL5 */
#define ITOPOFF_SHIFT		0
#define ITOPOFF_MASK		(0xF << ITOPOFF_SHIFT)

/* MAX8997_REG_MBCCTRL6 */
#define AUTOSTOP_SHIFT		5
#define AUTOSTOP_MASK		(1 << AUTOSTOP_SHIFT)

enum {
	BAT_NOT_DETECTED,
	BAT_DETECTED
};

struct chg_data {
	struct device			*dev;
	struct max8997_dev		*max8997;
	struct power_supply		*psy_bat;
        struct mutex            lock;

};

static enum power_supply_property max8997_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

/* vf check */
static bool max8997_check_detbat(struct chg_data *chg)
{
//	struct i2c_client *i2c = chg->max8997->i2c;
	unsigned int data = 0;
	int ret;

	ret = regmap_read(chg->max8997->regmap,
                                MAX8997_REG_STATUS4, &data);

	if (ret < 0) {
		dev_err(chg->dev, "%s: max8997_read_reg error(%d)\n",
			__func__, ret);
		return ret;
	}

	if (data & DETBAT_MASK)
		dev_info(chg->dev, "%s: batt not detected(0x%x)\n",
			__func__, data);

	return data & DETBAT_MASK;
}

/* whether charging enabled or not */
static bool max8997_check_vdcin(struct chg_data *chg)
{
//	struct i2c_client *i2c = chg->max8997->i2c;
	unsigned int data = 0;
	int ret;

        ret = regmap_read(chg->max8997->regmap,
                                MAX8997_REG_STATUS4, &data);

	if (ret < 0) {
		dev_err(chg->dev, "%s: max8997_read_reg error(%d)\n", __func__,
				ret);
		return ret;
	}

	return data & DCINOK_MASK;
}

static int max8997_chg_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
        struct chg_data *chg = power_supply_get_drvdata(psy);


	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (max8997_check_vdcin(chg))
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (max8997_check_detbat(chg))
			val->intval = BAT_NOT_DETECTED;
		else
			val->intval = BAT_DETECTED;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		/* battery is always online */
		val->intval = 1;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


static int max8997_disable_charging(struct chg_data *chg)
{
//	struct i2c_client *i2c = chg->max8997->i2c;

	int ret;
	u8 mask;

	dev_info(chg->dev, "%s: disable charging\n", __func__);
	mask = MBCHOSTEN_MASK | VCHGR_FC_MASK;
//	ret = max8997_update_reg(i2c, MAX8997_REG_MBCCTRL2, 0, mask);

	ret = regmap_update_bits(chg->max8997->regmap,
                        MAX8997_REG_MBCCTRL2,
                        mask, 0);
	if (ret < 0)
		dev_err(chg->dev, "%s: fail update reg!!!\n", __func__);

	return ret;
}

static int max8997_set_charging_current(struct chg_data *chg, int chg_current)
{
//	struct i2c_client *i2c = chg->max8997->i2c;
	int ret;
	u8 val;

	if (chg_current < 200 || chg_current > 950)
		return -EINVAL;

	val = ((chg_current - 200) / 50) & 0xf;

	dev_info(chg->dev, "%s: charging current=%d", __func__, chg_current);
//	ret = max8997_update_reg(i2c, MAX8997_REG_MBCCTRL4,
//		(val << MBCICHFC_SHIFT), MBCICHFC_MASK);

	ret = regmap_update_bits(chg->max8997->regmap,
                        MAX8997_REG_MBCCTRL4,
                        MBCICHFC_MASK, (val << MBCICHFC_SHIFT));

	if (ret)
		dev_err(chg->dev, "%s: fail to write chg current(%d)\n",
				__func__, ret);

	return ret;
}

static int max8997_enable_charging(struct chg_data *chg)
{
//	struct i2c_client *i2c = chg->max8997->i2c;
	int ret;
	u8 val, mask;

	/* set auto stop disable */
//	ret = max8997_update_reg(i2c, MAX8997_REG_MBCCTRL6,
//			(0 << AUTOSTOP_SHIFT), AUTOSTOP_MASK);

        ret = regmap_update_bits(chg->max8997->regmap,
                        MAX8997_REG_MBCCTRL6,
                        AUTOSTOP_MASK, (0 << AUTOSTOP_SHIFT));


	if (ret)
		dev_err(chg->dev, "%s: failt to disable autostop(%d)\n",
				__func__, ret);

	/* set fast charging enable and main battery charging enable */
	val = (1 << MBCHOSTEN_SHIFT) | (1 << VCHGR_FC_SHIFT);
	mask = MBCHOSTEN_MASK | VCHGR_FC_MASK;
//	ret = max8997_update_reg(i2c, MAX8997_REG_MBCCTRL2, val, mask);

        ret = regmap_update_bits(chg->max8997->regmap,
                        MAX8997_REG_MBCCTRL2,
                        mask, val);

	if (ret)
		dev_err(chg->dev, "%s: failt to enable charging(%d)\n",
				__func__, ret);

	return ret;
}

/* TODO: remove this function later */
static int max8997_enable_charging_x(struct chg_data *chg, int charge_type)
{
//	struct i2c_client *i2c = chg->max8997->i2c;
	int ret;
	u8 val, mask;

	/* enable charging */
	if (charge_type == POWER_SUPPLY_CHARGE_TYPE_FAST) {
		/* ac */
		dev_info(chg->dev, "%s: TA charging", __func__);
		/* set fast charging current : 650mA */

//		ret = max8997_update_reg(i2c, MAX8997_REG_MBCCTRL4,
//			(9 << MBCICHFC_SHIFT), MBCICHFC_MASK);

		ret = regmap_update_bits(chg->max8997->regmap,
                        MAX8997_REG_MBCCTRL4,
                        MBCICHFC_MASK, (9 << MBCICHFC_SHIFT));

		if (ret)
			goto err;

	} else if (charge_type == POWER_SUPPLY_CHARGE_TYPE_TRICKLE) {
		/* usb */
		dev_info(chg->dev, "%s: USB charging", __func__);
		/* set fast charging current : 450mA */
//		ret = max8997_update_reg(i2c, MAX8997_REG_MBCCTRL4,
//				(5 << MBCICHFC_SHIFT), MBCICHFC_MASK);

                ret = regmap_update_bits(chg->max8997->regmap,
                        MAX8997_REG_MBCCTRL4,
                        MBCICHFC_MASK, (5 << MBCICHFC_SHIFT));

		if (ret)
			goto err;
	} else {
		dev_err(chg->dev, "%s: invalid arg\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	/* set auto stop disable */
//	ret = max8997_update_reg(i2c, MAX8997_REG_MBCCTRL6,
//			(0 << AUTOSTOP_SHIFT), AUTOSTOP_MASK);

	ret = regmap_update_bits(chg->max8997->regmap,
                        MAX8997_REG_MBCCTRL6,
                        AUTOSTOP_MASK, (0 << AUTOSTOP_SHIFT));

	if (ret)
		goto err;

	/* set fast charging enable and main battery charging enable */
	val = (1 << MBCHOSTEN_SHIFT) | (1 << VCHGR_FC_SHIFT);
	mask = MBCHOSTEN_MASK | VCHGR_FC_MASK;
//	ret = max8997_update_reg(i2c, MAX8997_REG_MBCCTRL2, val, mask);


        ret = regmap_update_bits(chg->max8997->regmap,
                        MAX8997_REG_MBCCTRL2,
                        mask, val);


	if (ret)
		goto err;

	return 0;
err:
	dev_err(chg->dev, "%s: max8997 update reg error!(%d)\n", __func__, ret);
	return ret;
}

static int max8997_chg_set_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val)
{
	int ret, reg_val;
//	struct chg_data *chg = container_of(psy, struct chg_data, psy_bat);
        struct chg_data *chg = power_supply_get_drvdata(psy);

	//struct i2c_client *i2c = chg->max8997->i2c;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_TYPE: /* TODO: remove this */
		if (val->intval == POWER_SUPPLY_CHARGE_TYPE_NONE)
			ret = max8997_disable_charging(chg);
		else
			ret = max8997_enable_charging_x(chg, val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW: /* Set charging current */
		ret = max8997_set_charging_current(chg, val->intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:	/* Enable/Disable charging */
		if (val->intval == POWER_SUPPLY_STATUS_CHARGING)
			ret = max8997_enable_charging(chg);
		else
			ret = max8997_disable_charging(chg);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL: /* Set recharging current */
		if (val->intval < 50 || val->intval > 200) {
			dev_err(chg->dev, "%s: invalid topoff current(%d)\n",
					__func__, val->intval);
			return -EINVAL;
		}
		reg_val = (val->intval - 50) / 10;

		dev_info(chg->dev, "%s: Set toppoff current to 0x%x\n",
				__func__, reg_val);
//		ret = max8997_update_reg(i2c, MAX8997_REG_MBCCTRL5,
//				(reg_val << ITOPOFF_SHIFT), ITOPOFF_MASK);


		ret = regmap_update_bits(chg->max8997->regmap,
                        MAX8997_REG_MBCCTRL5,
                        ITOPOFF_MASK, (reg_val << ITOPOFF_SHIFT));

		if (ret) {
			dev_err(chg->dev, "%s: max8997 update reg error(%d)\n",
					__func__, ret);
			return ret;
		}
		break;
	default:
		return -EINVAL;
	}
	return ret;
}


static const struct power_supply_desc max8997_battery_desc = {
        .name           = "max8997_pmic",
        .type           = POWER_SUPPLY_TYPE_BATTERY,
        .get_property   = max8997_chg_get_property,
	.set_property 	= max8997_chg_set_property,
        .properties     = max8997_battery_props,
        .num_properties = ARRAY_SIZE(max8997_battery_props),
};


static int max8997_battery_probe(struct platform_device *pdev)
{
	struct max8997_dev *max8997 = dev_get_drvdata(pdev->dev.parent);
	struct chg_data *chg;
	int ret = 0;
        struct power_supply_config psy_cfg = {};

	dev_info(&pdev->dev, "%s : MAX8997 Charger Driver Loading\n", __func__);

	chg = kzalloc(sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

        platform_set_drvdata(pdev, chg);

        mutex_init(&chg->lock);


	chg->dev = &pdev->dev;
	chg->max8997 = max8997;

        psy_cfg.drv_data = chg;


	/* TODO: configure by platform data*/
//	ret = max8997_update_reg(i2c, MAX8997_REG_MBCCTRL1, /* Disable */
//		(0x7 << TFCH_SHIFT), TFCH_MASK);

        ret = regmap_update_bits(chg->max8997->regmap,
                        MAX8997_REG_MBCCTRL1,
                        TFCH_MASK, (0x7 << TFCH_SHIFT));
	if (ret < 0)
		goto err_kfree;

	/* TODO: configure by platform data*/
//	ret = max8997_update_reg(i2c, MAX8997_REG_MBCCTRL3, /* 4.2V */
//		(0x0 << MBCCV_SHIFT), MBCCV_MASK);

        ret = regmap_update_bits(chg->max8997->regmap,
                        MAX8997_REG_MBCCTRL3,
                        MBCCV_MASK, (0x0 << MBCCV_SHIFT));
	if (ret < 0)
		goto err_kfree;

	/* init power supplier framework */
//	ret = power_supply_register(&pdev->dev, &chg->psy_bat);
	chg->psy_bat = power_supply_register(&pdev->dev, &max8997_battery_desc,
                                                 &psy_cfg);

        if (IS_ERR(chg->psy_bat)) {
                dev_err(&pdev->dev, "failed: power supply register\n");
                return PTR_ERR(chg->psy_bat);
        }


	return 0;

err_kfree:
	kfree(chg);
	return ret;
}


static int max8997_charger_remove(struct platform_device *pdev)
{
        struct chg_data *chg = platform_get_drvdata(pdev);

        power_supply_unregister(chg->psy_bat);

        kfree(chg);

        return 0;
}

static const struct platform_device_id max8997_battery_id[] = {
        { "max8997-battery", 0 },
        { }
};
MODULE_DEVICE_TABLE(platform, max8997_battery_id);

static struct platform_driver max8997_battery_driver = {
        .driver = {
                .name = "max8997-battery",
        },
        .probe = max8997_battery_probe,
	.remove = max8997_charger_remove,
        .id_table = max8997_battery_id,
};


module_platform_driver(max8997_battery_driver);

MODULE_DESCRIPTION("MAXIM 8997 charger control driver");
MODULE_AUTHOR("<ms925.kim@samsung.com>");
MODULE_LICENSE("GPL");
