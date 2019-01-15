// SPDX-License-Identifier: GPL-2.0+
//
// max8997_charger.c - Power supply consumer driver for the Maxim 8997/8966
//
//  Copyright (C) 2011 Samsung Electronics
//  MyungJoo Ham <myungjoo.ham@samsung.com>

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



struct charger_data {
	struct device *dev;
	struct max8997_dev *iodev;
	struct power_supply *battery;
        struct mutex            lock;
        int virq;
};

static enum power_supply_property max8997_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS, /* "FULL" or "NOT FULL" only. */
	POWER_SUPPLY_PROP_PRESENT, /* the presence of battery */
	POWER_SUPPLY_PROP_ONLINE, /* charger is active or not */
};

/* Note that the charger control is done by a current regulator "CHARGER" */
static int max8997_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct charger_data *charger = power_supply_get_drvdata(psy);
	int ret;
	unsigned int reg;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = 0;
		ret = regmap_read(charger->iodev->regmap,
				MAX8997_REG_STATUS4, &reg);
		if (ret)
			return ret;
		if ((reg & (1 << 0)) == 0x1)
			val->intval = POWER_SUPPLY_STATUS_FULL;

		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 0;
		ret = regmap_read(charger->iodev->regmap,
				MAX8997_REG_STATUS4, &reg);
		if (ret)
			return ret;
		if ((reg & (1 << 2)) == 0x0)
			val->intval = 1;

		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		ret = regmap_read(charger->iodev->regmap,
				MAX8997_REG_STATUS4, &reg);
		if (ret)
			return ret;
		/* DCINOK */
		if (reg & (1 << 1))
			val->intval = 1;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct power_supply_desc max8997_battery_desc = {
	.name		= "max8997_pmic",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= max8997_battery_get_property,
	.properties	= max8997_battery_props,
	.num_properties	= ARRAY_SIZE(max8997_battery_props),
};

static irqreturn_t max8997_chgins_irq(int irq, void *data)
{
        struct charger_data *info = data;

        dev_info(info->dev, "%s:irq(%d)\n", __func__, irq);

        //rtc_update_irq(info->rtc_dev, 1, RTC_IRQF | RTC_AF);

        return IRQ_HANDLED;
}


static int max8997_battery_probe(struct platform_device *pdev)
{
        struct max8997_dev *max8997 = dev_get_drvdata(pdev->dev.parent);
	int ret,virq;
	struct charger_data *charger;
	struct power_supply_config psy_cfg = {};

	charger = devm_kzalloc(&pdev->dev, sizeof( struct charger_data), GFP_KERNEL);
	if (!charger) {
		dev_err(&pdev->dev, "cahrger null.\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, charger);


        mutex_init(&charger->lock);

	charger->dev = &pdev->dev;
	charger->iodev = max8997;

	psy_cfg.drv_data = charger;

	charger->battery = power_supply_register(&pdev->dev,
						 &max8997_battery_desc,
						 &psy_cfg);
	if (IS_ERR(charger->battery)) {
		dev_err(&pdev->dev, "failed: power supply register\n");
		return PTR_ERR(charger->battery);
	}


        virq = regmap_irq_get_virq(max8997->irq_data, MAX8997_PMICIRQ_CHGINS);
        if (virq <= 0) {
                dev_err(&pdev->dev, "Failed to create mapping alarm IRQ\n");
                ret = -ENXIO;
                goto err_out;
        }
        charger->virq = virq;

        ret = devm_request_threaded_irq(&pdev->dev, virq, NULL,
                                max8997_chgins_irq, 0,
                                "max8997-CHGINS", charger);
        if (ret < 0)
                dev_err(&pdev->dev, "Failed to request alarm IRQ: %d: %d\n",
                        charger->virq, ret);


        virq = regmap_irq_get_virq(max8997->irq_data, MAX8997_PMICIRQ_CHGRM);
        if (virq <= 0) {
                dev_err(&pdev->dev, "Failed to create mapping alarm IRQ\n");
                ret = -ENXIO;
                goto err_out;
        }
        charger->virq = virq;

        ret = devm_request_threaded_irq(&pdev->dev, virq, NULL,
                                max8997_chgins_irq, 0,
                                "max8997-CHGRM", charger);
        if (ret < 0)
                dev_err(&pdev->dev, "Failed to request alarm IRQ: %d: %d\n",
                        charger->virq, ret);

        virq = regmap_irq_get_virq(max8997->irq_data, MAX8997_PMICIRQ_TOPOFFR);
        if (virq <= 0) {
                dev_err(&pdev->dev, "Failed to create mapping alarm IRQ\n");
                ret = -ENXIO;
                goto err_out;
        }
        charger->virq = virq;

        ret = devm_request_threaded_irq(&pdev->dev, virq, NULL,
                                max8997_chgins_irq, 0,
                                "max8997-TOPOFFR", charger);
        if (ret < 0)
                dev_err(&pdev->dev, "Failed to request alarm IRQ: %d: %d\n",
                        charger->virq, ret);

        virq = regmap_irq_get_virq(max8997->irq_data, MAX8997_PMICIRQ_DCINOVP);
        if (virq <= 0) {
                dev_err(&pdev->dev, "Failed to create mapping alarm IRQ\n");
                ret = -ENXIO;
                goto err_out;
        }
        charger->virq = virq;

        ret = devm_request_threaded_irq(&pdev->dev, virq, NULL,
                                max8997_chgins_irq, 0,
                                "max8997-DCINOVP", charger);
        if (ret < 0)
                dev_err(&pdev->dev, "Failed to request alarm IRQ: %d: %d\n",
                        charger->virq, ret);



err_out:
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
	.id_table = max8997_battery_id,
};


module_platform_driver(max8997_battery_driver);

MODULE_DESCRIPTION("MAXIM 8997/8966 battery control driver");
MODULE_AUTHOR("MyungJoo Ham <myungjoo.ham@samsung.com>");
MODULE_LICENSE("GPL");
