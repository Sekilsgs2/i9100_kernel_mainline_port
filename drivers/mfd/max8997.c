// SPDX-License-Identifier: GPL-2.0+
//
// max8997.c - mfd core driver for the Maxim 8966 and 8997
//
// Copyright (C) 2011 Samsung Electronics
// MyungJoo Ham <myungjoo.ham@samsung.com>
//
// This driver is based on max8998.c

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max8997.h>
#include <linux/mfd/max8997-private.h>
#include <linux/regmap.h>

#define I2C_ADDR_PMIC	(0xCC >> 1)
#define I2C_ADDR_MUIC	(0x4A >> 1)
#define I2C_ADDR_BATTERY	(0x6C >> 1)
#define I2C_ADDR_RTC	(0x0C >> 1)
#define I2C_ADDR_HAPTIC	(0x90 >> 1)

static const struct mfd_cell max8997_devs[] = {
	{ .name = "max8997-pmic", },
	{ .name = "max8997-rtc", },
	{ .name = "max8997-battery", },
	{ .name = "max8997-haptic", },
	{ .name = "max8997-muic", },
	{ .name = "max8997-led", .id = 1 },
	{ .name = "max8997-led", .id = 2 },
};

#ifdef CONFIG_OF
static const struct of_device_id max8997_pmic_dt_match[] = {
	{ .compatible = "maxim,max8997-pmic", .data = (void *)TYPE_MAX8997 },
	{},
};
#endif

static const struct regmap_config max8997_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX8997_REG_PMIC_END,
};

static const struct regmap_irq max8997_irqs[] = {
	/* PMIC_INT1 interrupts */
	{ .reg_offset = 0, .mask = PMIC_INT1_PWRONR_MASK, },
	{ .reg_offset = 0, .mask = PMIC_INT1_PWRONF_MASK, },
	{ .reg_offset = 0, .mask = PMIC_INT1_PWRON1SEC_MASK, },
	{ .reg_offset = 0, .mask = PMIC_INT1_JIGONR_MASK, },
	{ .reg_offset = 0, .mask = PMIC_INT1_JIGONF_MASK, },
	{ .reg_offset = 0, .mask = PMIC_INT1_LOWBAT2_MASK, },
	{ .reg_offset = 0, .mask = PMIC_INT1_LOWBAT1_MASK, },
	/* PMIC_INT2 interrupts */
	{ .reg_offset = 1, .mask = PMIC_INT2_JIGR_MASK, },
	{ .reg_offset = 1, .mask = PMIC_INT2_JIGF_MASK, },
	{ .reg_offset = 1, .mask = PMIC_INT2_MR_MASK, },
	{ .reg_offset = 1, .mask = PMIC_INT2_DVS1OK_MASK, },
	{ .reg_offset = 1, .mask = PMIC_INT2_DVS2OK_MASK, },
	{ .reg_offset = 1, .mask = PMIC_INT2_DVS3OK_MASK, },
	{ .reg_offset = 1, .mask = PMIC_INT2_DVS4OK_MASK, },
	/* PMIC_INT3 interrupts */
	{ .reg_offset = 2, .mask = PMIC_INT3_CHGINS_MASK, },
	{ .reg_offset = 2, .mask = PMIC_INT3_CHGRM_MASK, },
	{ .reg_offset = 2, .mask = PMIC_INT3_DCINOVP_MASK, },
	{ .reg_offset = 2, .mask = PMIC_INT3_TOPOFFR_MASK, },
	{ .reg_offset = 2, .mask = PMIC_INT3_CHGRSTF_MASK, },
	{ .reg_offset = 2, .mask = PMIC_INT3_MBCHGTMEXPD_MASK, },
	/* PMIC_INT4 interrupts */
	{ .reg_offset = 3, .mask = PMIC_INT4_RTC60S_MASK, },
	{ .reg_offset = 3, .mask = PMIC_INT4_RTCA1_MASK, },
	{ .reg_offset = 3, .mask = PMIC_INT4_RTCA2_MASK, },
	{ .reg_offset = 3, .mask = PMIC_INT4_SMPL_INT_MASK, },
	{ .reg_offset = 3, .mask = PMIC_INT4_RTC1S_MASK, },
	{ .reg_offset = 3, .mask = PMIC_INT4_WTSR_MASK, },
};

static const struct regmap_irq_chip max8997_irq_chip = {
	.name			= "max8997",
	.status_base		= MAX8997_REG_INT1,
	.mask_base		= MAX8997_REG_INT1MSK,
	.mask_invert		= false,
	.num_regs		= 4,
	.irqs			= max8997_irqs,
	.num_irqs		= ARRAY_SIZE(max8997_irqs),
};

static const struct regmap_config max8997_regmap_rtc_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX8997_RTC_REG_END,
};

static const struct regmap_config max8997_regmap_haptic_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX8997_HAPTIC_REG_END,
};

static const struct regmap_config max8997_regmap_muic_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX8997_MUIC_REG_END,
};

static const struct regmap_irq max8997_irqs_muic[] = {
	/* MUIC_INT1 interrupts */
	{ .reg_offset = 0, .mask = MUIC_INT1_ADC_MASK, },
	{ .reg_offset = 0, .mask = MUIC_INT1_ADCLOW_MASK, },
	{ .reg_offset = 0, .mask = MUIC_INT1_ADCERROR_MASK, },
	/* MUIC_INT2 interrupts */
	{ .reg_offset = 1, .mask = MUIC_INT2_CHGTYP_MASK, },
	{ .reg_offset = 1, .mask = MUIC_INT2_CHGDETRUN_MASK, },
	{ .reg_offset = 1, .mask = MUIC_INT2_DCDTMR_MASK, },
	{ .reg_offset = 1, .mask = MUIC_INT2_DBCHG_MASK, },
	{ .reg_offset = 1, .mask = MUIC_INT2_VBVOLT_MASK, },
	/* MUIC_INT3 interrupts */
	{ .reg_offset = 2, .mask = MUIC_INT3_OVP_MASK, },
};

static const struct regmap_irq_chip max8997_irq_chip_muic = {
	.name			= "max8997-muic",
	.status_base		= MAX8997_MUIC_REG_INT1,
	.mask_base		= MAX8997_MUIC_REG_INTMASK1,
	.mask_invert		= true,
	.num_regs		= 3,
	.irqs			= max8997_irqs_muic,
	.num_irqs		= ARRAY_SIZE(max8997_irqs_muic),
};

/*
 * Only the common platform data elements for max8997 are parsed here from the
 * device tree. Other sub-modules of max8997 such as pmic, rtc and others have
 * to parse their own platform data elements from device tree.
 *
 * The max8997 platform data structure is instantiated here and the drivers for
 * the sub-modules need not instantiate another instance while parsing their
 * platform data.
 */
static struct max8997_platform_data *max8997_i2c_parse_dt_pdata(
					struct device *dev)
{
	struct max8997_platform_data *pd;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return ERR_PTR(-ENOMEM);

	pd->ono = irq_of_parse_and_map(dev->of_node, 1);

	return pd;
}

static inline unsigned long max8997_i2c_get_driver_data(struct i2c_client *i2c,
						const struct i2c_device_id *id)
{
	if (IS_ENABLED(CONFIG_OF) && i2c->dev.of_node) {
		const struct of_device_id *match;
		match = of_match_node(max8997_pmic_dt_match, i2c->dev.of_node);
		return (unsigned long)match->data;
	}
	return id->driver_data;
}

static int max8997_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct max8997_dev *max8997;
	struct max8997_platform_data *pdata = dev_get_platdata(&i2c->dev);
	int ret = 0;

	max8997 = devm_kzalloc(&i2c->dev, sizeof(struct max8997_dev),
				GFP_KERNEL);
	if (max8997 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, max8997);
	max8997->dev = &i2c->dev;
	max8997->i2c = i2c;
	max8997->type = max8997_i2c_get_driver_data(i2c, id);
	max8997->irq = i2c->irq;

	if (IS_ENABLED(CONFIG_OF) && max8997->dev->of_node) {
		pdata = max8997_i2c_parse_dt_pdata(max8997->dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	if (!pdata)
		return ret;

	max8997->pdata = pdata;
	max8997->ono = pdata->ono;

	mutex_init(&max8997->iolock);

	max8997->rtc = i2c_new_dummy(i2c->adapter, I2C_ADDR_RTC);
	if (!max8997->rtc) {
		dev_err(max8997->dev, "Failed to allocate I2C device for RTC\n");
		return -ENODEV;
	}
	i2c_set_clientdata(max8997->rtc, max8997);

	max8997->haptic = i2c_new_dummy(i2c->adapter, I2C_ADDR_HAPTIC);
	if (!max8997->haptic) {
		dev_err(max8997->dev, "Failed to allocate I2C device for Haptic\n");
		ret = -ENODEV;
		goto err_i2c_haptic;
	}
	i2c_set_clientdata(max8997->haptic, max8997);

	max8997->muic = i2c_new_dummy(i2c->adapter, I2C_ADDR_MUIC);
	if (!max8997->muic) {
		dev_err(max8997->dev, "Failed to allocate I2C device for MUIC\n");
		ret = -ENODEV;
		goto err_i2c_muic;
	}
	i2c_set_clientdata(max8997->muic, max8997);

	max8997->regmap = devm_regmap_init_i2c(i2c, &max8997_regmap_config);
	if (IS_ERR(max8997->regmap)) {
		ret = PTR_ERR(max8997->regmap);
		dev_err(max8997->dev,
				"failed to allocate register map: %d\n", ret);
		return ret;
	}

	max8997->regmap_rtc = devm_regmap_init_i2c(max8997->rtc,
					&max8997_regmap_rtc_config);
	if (IS_ERR(max8997->regmap_rtc)) {
		ret = PTR_ERR(max8997->regmap_rtc);
		dev_err(max8997->dev,
				"failed to allocate register map: %d\n", ret);
		goto err_regmap;
	}

	max8997->regmap_haptic = devm_regmap_init_i2c(max8997->haptic,
					&max8997_regmap_haptic_config);
	if (IS_ERR(max8997->regmap_haptic)) {
		ret = PTR_ERR(max8997->regmap_haptic);
		dev_err(max8997->dev,
				"failed to allocate register map: %d\n", ret);
		goto err_regmap;
	}

	max8997->regmap_muic = devm_regmap_init_i2c(max8997->muic,
					&max8997_regmap_muic_config);
	if (IS_ERR(max8997->regmap_muic)) {
		ret = PTR_ERR(max8997->regmap_muic);
		dev_err(max8997->dev,
				"failed to allocate register map: %d\n", ret);
		goto err_regmap;
	}

	ret = regmap_add_irq_chip(max8997->regmap, max8997->irq,
				IRQF_ONESHOT | IRQF_SHARED |
				IRQF_TRIGGER_FALLING, 0,
				&max8997_irq_chip, &max8997->irq_data);
	if (ret) {
		dev_err(max8997->dev, "failed to add irq chip: %d\n", ret);
		goto err_regmap;
	}

	ret = regmap_add_irq_chip(max8997->regmap_muic, max8997->irq,
				IRQF_ONESHOT | IRQF_SHARED |
				IRQF_TRIGGER_FALLING, 0,
				&max8997_irq_chip_muic,
				&max8997->irq_data_muic);
	if (ret) {
		dev_err(max8997->dev, "failed to add irq chip: %d\n", ret);
		goto err_irq_muic;
	}

	pm_runtime_set_active(max8997->dev);

	ret = mfd_add_devices(max8997->dev, -1, max8997_devs,
			ARRAY_SIZE(max8997_devs),
			NULL, 0, NULL);
	if (ret < 0) {
		dev_err(max8997->dev, "failed to add MFD devices %d\n", ret);
		goto err_mfd;
	}

	/*
	 * TODO: enable others (flash, muic, rtc, battery, ...) and
	 * check the return value
	 */

	/* MAX8997 has a power button input. */
	device_init_wakeup(max8997->dev, true);

	return ret;

err_mfd:
	mfd_remove_devices(max8997->dev);
	regmap_del_irq_chip(max8997->irq, max8997->irq_data_muic);
err_irq_muic:
	regmap_del_irq_chip(max8997->irq, max8997->irq_data);
err_regmap:
	i2c_unregister_device(max8997->muic);
err_i2c_muic:
	i2c_unregister_device(max8997->haptic);
err_i2c_haptic:
	i2c_unregister_device(max8997->rtc);
	return ret;
}

static const struct i2c_device_id max8997_i2c_id[] = {
	{ "max8997", TYPE_MAX8997 },
	{ "max8966", TYPE_MAX8966 },
	{ }
};

static u8 max8997_dumpaddr_pmic[] = {
	MAX8997_REG_INT1MSK,
	MAX8997_REG_INT2MSK,
	MAX8997_REG_INT3MSK,
	MAX8997_REG_INT4MSK,
	MAX8997_REG_MAINCON1,
	MAX8997_REG_MAINCON2,
	MAX8997_REG_BUCKRAMP,
	MAX8997_REG_BUCK1CTRL,
	MAX8997_REG_BUCK1DVS1,
	MAX8997_REG_BUCK1DVS2,
	MAX8997_REG_BUCK1DVS3,
	MAX8997_REG_BUCK1DVS4,
	MAX8997_REG_BUCK1DVS5,
	MAX8997_REG_BUCK1DVS6,
	MAX8997_REG_BUCK1DVS7,
	MAX8997_REG_BUCK1DVS8,
	MAX8997_REG_BUCK2CTRL,
	MAX8997_REG_BUCK2DVS1,
	MAX8997_REG_BUCK2DVS2,
	MAX8997_REG_BUCK2DVS3,
	MAX8997_REG_BUCK2DVS4,
	MAX8997_REG_BUCK2DVS5,
	MAX8997_REG_BUCK2DVS6,
	MAX8997_REG_BUCK2DVS7,
	MAX8997_REG_BUCK2DVS8,
	MAX8997_REG_BUCK3CTRL,
	MAX8997_REG_BUCK3DVS,
	MAX8997_REG_BUCK4CTRL,
	MAX8997_REG_BUCK4DVS,
	MAX8997_REG_BUCK5CTRL,
	MAX8997_REG_BUCK5DVS1,
	MAX8997_REG_BUCK5DVS2,
	MAX8997_REG_BUCK5DVS3,
	MAX8997_REG_BUCK5DVS4,
	MAX8997_REG_BUCK5DVS5,
	MAX8997_REG_BUCK5DVS6,
	MAX8997_REG_BUCK5DVS7,
	MAX8997_REG_BUCK5DVS8,
	MAX8997_REG_BUCK6CTRL,
	MAX8997_REG_BUCK6BPSKIPCTRL,
	MAX8997_REG_BUCK7CTRL,
	MAX8997_REG_BUCK7DVS,
	MAX8997_REG_LDO1CTRL,
	MAX8997_REG_LDO2CTRL,
	MAX8997_REG_LDO3CTRL,
	MAX8997_REG_LDO4CTRL,
	MAX8997_REG_LDO5CTRL,
	MAX8997_REG_LDO6CTRL,
	MAX8997_REG_LDO7CTRL,
	MAX8997_REG_LDO8CTRL,
	MAX8997_REG_LDO9CTRL,
	MAX8997_REG_LDO10CTRL,
	MAX8997_REG_LDO11CTRL,
	MAX8997_REG_LDO12CTRL,
	MAX8997_REG_LDO13CTRL,
	MAX8997_REG_LDO14CTRL,
	MAX8997_REG_LDO15CTRL,
	MAX8997_REG_LDO16CTRL,
	MAX8997_REG_LDO17CTRL,
	MAX8997_REG_LDO18CTRL,
	MAX8997_REG_LDO21CTRL,
	MAX8997_REG_MBCCTRL1,
	MAX8997_REG_MBCCTRL2,
	MAX8997_REG_MBCCTRL3,
	MAX8997_REG_MBCCTRL4,
	MAX8997_REG_MBCCTRL5,
	MAX8997_REG_MBCCTRL6,
	MAX8997_REG_OTPCGHCVS,
	MAX8997_REG_SAFEOUTCTRL,
	MAX8997_REG_LBCNFG1,
	MAX8997_REG_LBCNFG2,
	MAX8997_REG_BBCCTRL,

	MAX8997_REG_FLASH1_CUR,
	MAX8997_REG_FLASH2_CUR,
	MAX8997_REG_MOVIE_CUR,
	MAX8997_REG_GSMB_CUR,
	MAX8997_REG_BOOST_CNTL,
	MAX8997_REG_LEN_CNTL,
	MAX8997_REG_FLASH_CNTL,
	MAX8997_REG_WDT_CNTL,
	MAX8997_REG_MAXFLASH1,
	MAX8997_REG_MAXFLASH2,
	MAX8997_REG_FLASHSTATUSMASK,

	MAX8997_REG_GPIOCNTL1,
	MAX8997_REG_GPIOCNTL2,
	MAX8997_REG_GPIOCNTL3,
	MAX8997_REG_GPIOCNTL4,
	MAX8997_REG_GPIOCNTL5,
	MAX8997_REG_GPIOCNTL6,
	MAX8997_REG_GPIOCNTL7,
	MAX8997_REG_GPIOCNTL8,
	MAX8997_REG_GPIOCNTL9,
	MAX8997_REG_GPIOCNTL10,
	MAX8997_REG_GPIOCNTL11,
	MAX8997_REG_GPIOCNTL12,

	MAX8997_REG_LDO1CONFIG,
	MAX8997_REG_LDO2CONFIG,
	MAX8997_REG_LDO3CONFIG,
	MAX8997_REG_LDO4CONFIG,
	MAX8997_REG_LDO5CONFIG,
	MAX8997_REG_LDO6CONFIG,
	MAX8997_REG_LDO7CONFIG,
	MAX8997_REG_LDO8CONFIG,
	MAX8997_REG_LDO9CONFIG,
	MAX8997_REG_LDO10CONFIG,
	MAX8997_REG_LDO11CONFIG,
	MAX8997_REG_LDO12CONFIG,
	MAX8997_REG_LDO13CONFIG,
	MAX8997_REG_LDO14CONFIG,
	MAX8997_REG_LDO15CONFIG,
	MAX8997_REG_LDO16CONFIG,
	MAX8997_REG_LDO17CONFIG,
	MAX8997_REG_LDO18CONFIG,
	MAX8997_REG_LDO21CONFIG,

	MAX8997_REG_DVSOKTIMER1,
	MAX8997_REG_DVSOKTIMER2,
	MAX8997_REG_DVSOKTIMER4,
	MAX8997_REG_DVSOKTIMER5,
};

static u8 max8997_dumpaddr_muic[] = {
	MAX8997_MUIC_REG_INTMASK1,
	MAX8997_MUIC_REG_INTMASK2,
	MAX8997_MUIC_REG_INTMASK3,
	MAX8997_MUIC_REG_CDETCTRL,
	MAX8997_MUIC_REG_CONTROL1,
	MAX8997_MUIC_REG_CONTROL2,
	MAX8997_MUIC_REG_CONTROL3,
};

static u8 max8997_dumpaddr_haptic[] = {
	MAX8997_HAPTIC_REG_CONF1,
	MAX8997_HAPTIC_REG_CONF2,
	MAX8997_HAPTIC_REG_DRVCONF,
	MAX8997_HAPTIC_REG_CYCLECONF1,
	MAX8997_HAPTIC_REG_CYCLECONF2,
	MAX8997_HAPTIC_REG_SIGCONF1,
	MAX8997_HAPTIC_REG_SIGCONF2,
	MAX8997_HAPTIC_REG_SIGCONF3,
	MAX8997_HAPTIC_REG_SIGCONF4,
	MAX8997_HAPTIC_REG_SIGDC1,
	MAX8997_HAPTIC_REG_SIGDC2,
	MAX8997_HAPTIC_REG_SIGPWMDC1,
	MAX8997_HAPTIC_REG_SIGPWMDC2,
	MAX8997_HAPTIC_REG_SIGPWMDC3,
	MAX8997_HAPTIC_REG_SIGPWMDC4,
};

static int max8997_freeze(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct max8997_dev *max8997 = i2c_get_clientdata(i2c);
	int i;

	for (i = 0; i < ARRAY_SIZE(max8997_dumpaddr_pmic); i++)
		regmap_read(max8997->regmap, max8997_dumpaddr_pmic[i],
				&max8997->reg_dump[i]);

	for (i = 0; i < ARRAY_SIZE(max8997_dumpaddr_muic); i++)
		regmap_read(max8997->regmap_muic, max8997_dumpaddr_muic[i],
				&max8997->reg_dump[i + MAX8997_REG_PMIC_END]);

	for (i = 0; i < ARRAY_SIZE(max8997_dumpaddr_haptic); i++)
		regmap_read(max8997->regmap_haptic, max8997_dumpaddr_haptic[i],
				&max8997->reg_dump[i + MAX8997_REG_PMIC_END +
				MAX8997_MUIC_REG_END]);

	return 0;
}

static int max8997_restore(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct max8997_dev *max8997 = i2c_get_clientdata(i2c);
	int i;

	for (i = 0; i < ARRAY_SIZE(max8997_dumpaddr_pmic); i++)
		regmap_write(max8997->regmap, max8997_dumpaddr_pmic[i],
				max8997->reg_dump[i]);

	for (i = 0; i < ARRAY_SIZE(max8997_dumpaddr_muic); i++)
		regmap_write(max8997->regmap_muic, max8997_dumpaddr_muic[i],
				max8997->reg_dump[i + MAX8997_REG_PMIC_END]);

	for (i = 0; i < ARRAY_SIZE(max8997_dumpaddr_haptic); i++)
		regmap_write(max8997->regmap_haptic, max8997_dumpaddr_haptic[i],
				max8997->reg_dump[i + MAX8997_REG_PMIC_END +
				MAX8997_MUIC_REG_END]);

	return 0;
}

static int max8997_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct max8997_dev *max8997 = i2c_get_clientdata(i2c);

       if (device_may_wakeup(dev)) {
               enable_irq_wake(max8997->irq);
               disable_irq(max8997->irq);
       }

	return 0;
}

static int max8997_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct max8997_dev *max8997 = i2c_get_clientdata(i2c);

       if (device_may_wakeup(dev)) {
               disable_irq_wake(max8997->irq);
               enable_irq(max8997->irq);
       }

       return 0;

}

static const struct dev_pm_ops max8997_pm = {
	.suspend = max8997_suspend,
	.resume = max8997_resume,
	.freeze = max8997_freeze,
	.restore = max8997_restore,
};

static struct i2c_driver max8997_i2c_driver = {
	.driver = {
		   .name = "max8997",
		   .pm = &max8997_pm,
		   .suppress_bind_attrs = true,
		   .of_match_table = of_match_ptr(max8997_pmic_dt_match),
	},
	.probe = max8997_i2c_probe,
	.id_table = max8997_i2c_id,
};

static int __init max8997_i2c_init(void)
{
	return i2c_add_driver(&max8997_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(max8997_i2c_init);
