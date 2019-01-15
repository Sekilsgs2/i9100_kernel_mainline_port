// SPDX-License-Identifier: GPL-2.0+
//
// extcon-max8997.c - MAX8997 extcon driver to support MAX8997 MUIC
//
//  Copyright (C) 2012 Samsung Electronics
//  Donggeun Kim <dg77.kim@samsung.com>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
#include <linux/mfd/max8997.h>
#include <linux/mfd/max8997-private.h>
#include <linux/extcon-provider.h>
#include <linux/irqdomain.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/power_supply.h>


#define	DEV_NAME			"max8997-muic"
#define BAT_NAME			"max8997_pmic"
#define	DELAY_MS_DEFAULT		20000		/* unit: millisecond */


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


/* MAX8997 CDETCTRL register */
#define CHGDETEN_SHIFT			0
#define CHGTYPM_SHIFT			1
#define CHGDETEN_MASK			(0x1 << CHGDETEN_SHIFT)
#define CHGTYPM_MASK			(0x1 << CHGTYPM_SHIFT)



enum max8997_muic_adc_debounce_time {
	ADC_DEBOUNCE_TIME_0_5MS = 0,	/* 0.5ms */
	ADC_DEBOUNCE_TIME_10MS,		/* 10ms */
	ADC_DEBOUNCE_TIME_25MS,		/* 25ms */
	ADC_DEBOUNCE_TIME_38_62MS,	/* 38.62ms */
};

struct max8997_muic_irq {
	unsigned int irq;
	const char *name;
	unsigned int virq;
};

static struct max8997_muic_irq muic_irqs[] = {
	{ MAX8997_MUICIRQ_ADCERROR,	"MUIC-ADCERROR" },
	{ MAX8997_MUICIRQ_ADCLOW,	"MUIC-ADCLOW" },
	{ MAX8997_MUICIRQ_ADC,		"MUIC-ADC" },
	{ MAX8997_MUICIRQ_VBVOLT,	"MUIC-VBVOLT" },
	{ MAX8997_MUICIRQ_DBCHG,	"MUIC-DBCHG" },
	{ MAX8997_MUICIRQ_DCDTMR,	"MUIC-DCDTMR" },
	{ MAX8997_MUICIRQ_CHGDETRUN,	"MUIC-CHGDETRUN" },
	{ MAX8997_MUICIRQ_CHGTYP,	"MUIC-CHGTYP" },
	{ MAX8997_MUICIRQ_OVP,		"MUIC-OVP" },
};

/* Define supported cable type */
enum max8997_muic_acc_type {
	MAX8997_MUIC_ADC_GROUND = 0x0,
	MAX8997_MUIC_ADC_MHL,			/* MHL*/
	MAX8997_MUIC_ADC_REMOTE_S1_BUTTON,
	MAX8997_MUIC_ADC_REMOTE_S2_BUTTON,
	MAX8997_MUIC_ADC_REMOTE_S3_BUTTON,
	MAX8997_MUIC_ADC_REMOTE_S4_BUTTON,
	MAX8997_MUIC_ADC_REMOTE_S5_BUTTON,
	MAX8997_MUIC_ADC_REMOTE_S6_BUTTON,
	MAX8997_MUIC_ADC_REMOTE_S7_BUTTON,
	MAX8997_MUIC_ADC_REMOTE_S8_BUTTON,
	MAX8997_MUIC_ADC_REMOTE_S9_BUTTON,
	MAX8997_MUIC_ADC_REMOTE_S10_BUTTON,
	MAX8997_MUIC_ADC_REMOTE_S11_BUTTON,
	MAX8997_MUIC_ADC_REMOTE_S12_BUTTON,
	MAX8997_MUIC_ADC_RESERVED_ACC_1,
	MAX8997_MUIC_ADC_RESERVED_ACC_2,
	MAX8997_MUIC_ADC_RESERVED_ACC_3,
	MAX8997_MUIC_ADC_RESERVED_ACC_4,
	MAX8997_MUIC_ADC_RESERVED_ACC_5,
	MAX8997_MUIC_ADC_CEA936_AUDIO,
	MAX8997_MUIC_ADC_PHONE_POWERED_DEV,
	MAX8997_MUIC_ADC_TTY_CONVERTER,
	MAX8997_MUIC_ADC_UART_CABLE,
	MAX8997_MUIC_ADC_CEA936A_TYPE1_CHG,
	MAX8997_MUIC_ADC_FACTORY_MODE_USB_OFF,	/* JIG-USB-OFF */
	MAX8997_MUIC_ADC_FACTORY_MODE_USB_ON,	/* JIG-USB-ON */
	MAX8997_MUIC_ADC_AV_CABLE_NOLOAD,	/* DESKDOCK */
	MAX8997_MUIC_ADC_CEA936A_TYPE2_CHG,
	MAX8997_MUIC_ADC_FACTORY_MODE_UART_OFF,	/* JIG-UART */
	MAX8997_MUIC_ADC_FACTORY_MODE_UART_ON,	/* CARDOCK */
	MAX8997_MUIC_ADC_AUDIO_MODE_REMOTE,
	MAX8997_MUIC_ADC_OPEN,			/* OPEN */
};

enum max8997_muic_cable_group {
	MAX8997_CABLE_GROUP_ADC = 0,
	MAX8997_CABLE_GROUP_ADC_GND,
	MAX8997_CABLE_GROUP_CHG,
	MAX8997_CABLE_GROUP_VBVOLT,
};

enum max8997_muic_usb_type {
	MAX8997_USB_HOST,
	MAX8997_USB_DEVICE,
};

enum max8997_muic_charger_type {
	MAX8997_CHARGER_TYPE_NONE = 0,
	MAX8997_CHARGER_TYPE_USB,
	MAX8997_CHARGER_TYPE_DOWNSTREAM_PORT,
	MAX8997_CHARGER_TYPE_DEDICATED_CHG,
	MAX8997_CHARGER_TYPE_500MA,
	MAX8997_CHARGER_TYPE_1A,
	MAX8997_CHARGER_TYPE_DEAD_BATTERY = 7,
};

struct max8997_muic_info {
	struct device *dev;
	struct max8997_dev *max8997;
	struct extcon_dev *edev;
	int prev_cable_type;
	int prev_chg_type;
	u8 status[2];

	int irq;
	struct work_struct irq_work;
	struct mutex mutex;

	struct max8997_muic_platform_data *muic_pdata;
	enum max8997_muic_charger_type pre_charger_type;

	/*
	 * Use delayed workqueue to detect cable state and then
	 * notify cable state to notifiee/platform through uevent.
	 * After completing the booting of platform, the extcon provider
	 * driver should notify cable state to upper layer.
	 */
	struct delayed_work wq_detcable;

	/*
	 * Default usb/uart path whether UART/USB or AUX_UART/AUX_USB
	 * h/w path of COMP2/COMN1 on CONTROL1 register.
	 */
	int path_usb;
	int path_uart;

        struct gpio_desc *otg_en_gpio;
        struct gpio_desc *usb_sel_gpio;


};

static const unsigned int max8997_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_CHG_USB_SDP,
	EXTCON_CHG_USB_DCP,
	EXTCON_CHG_USB_FAST,
	EXTCON_CHG_USB_SLOW,
	EXTCON_CHG_USB_CDP,
	EXTCON_DISP_MHL,
	EXTCON_DOCK,
	EXTCON_JIG,
	EXTCON_NONE,
};


static int max8997_endis_charging(struct max8997_muic_info *info, bool enable)
{
        struct power_supply *psy = power_supply_get_by_name(BAT_NAME);
	int ret;

	union power_supply_propval val_type;

        if (!psy) {
                dev_err(info->dev, "%s: fail to get charger ps\n", __func__);
                return -ENODEV;
        }

	if (enable) {


		val_type.intval = 450;

        	/* Set charging current */
		ret = power_supply_set_property(psy,POWER_SUPPLY_PROP_CURRENT_NOW, &val_type);
                //ret = psy->set_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW,
                //                      450);

                if (ret) {
                        dev_err(info->dev, "%s: fail to set charging cur(%d)\n",
                                __func__, ret);
                        return ret;
                }

                /* Set topoff current */
                /* mA: TODO: should change 200->160 */
                val_type.intval = 200;

                ret = power_supply_set_property(psy,POWER_SUPPLY_PROP_CHARGE_FULL, &val_type);

//                ret = psy->set_property(psy, POWER_SUPPLY_PROP_CHARGE_FULL,
//                                        &topoff);
                if (ret) {
                        dev_err(info->dev, "%s: fail to set topoff cur(%d)\n",
                                __func__, ret);
                        return ret;
                }
                val_type.intval = POWER_SUPPLY_STATUS_CHARGING;

	}
	else {
                val_type.intval = POWER_SUPPLY_STATUS_DISCHARGING;

	}

        ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_STATUS, &val_type);
        if (ret) {
                dev_err(info->dev, "%s: fail to set charging status(%d)\n",
                        __func__, ret);
                return ret;
        }



        return ret;
};



/*
 * max8997_muic_set_debounce_time - Set the debounce time of ADC
 * @info: the instance including private data of max8997 MUIC
 * @time: the debounce time of ADC
 */
static int max8997_muic_set_debounce_time(struct max8997_muic_info *info,
		enum max8997_muic_adc_debounce_time time)
{
	int ret;

	switch (time) {
	case ADC_DEBOUNCE_TIME_0_5MS:
	case ADC_DEBOUNCE_TIME_10MS:
	case ADC_DEBOUNCE_TIME_25MS:
	case ADC_DEBOUNCE_TIME_38_62MS:
		ret = regmap_update_bits(info->max8997->regmap_muic,
					  MAX8997_MUIC_REG_CONTROL3,
					  CONTROL3_ADCDBSET_MASK,
					  time << CONTROL3_ADCDBSET_SHIFT);
		if (ret) {
			dev_err(info->dev, "failed to set ADC debounce time\n");
			return ret;
		}
		break;
	default:
		dev_err(info->dev, "invalid ADC debounce time\n");
		return -EINVAL;
	}

	return 0;
};



/* MAX8997 CONTROL1 register */
#define COMN1SW_SHIFT			0
#define COMP2SW_SHIFT			3
#define MICEN_SHIFT			6
#define COMN1SW_MASK			(0x7 << COMN1SW_SHIFT)
#define COMP2SW_MASK			(0x7 << COMP2SW_SHIFT)
#define MICEN_MASK			(0x1 << MICEN_SHIFT)

enum {
	CTRL1_TO_OPEN		= 0x0,
	CTRL1_TO_USB		= 0x1,
	CTRL1_TO_AUDIO		= 0x2,
	CTRL1_TO_UART		= 0x3,
};

static int set_com_sw(struct max8997_muic_info *info)
{
	int ret = 0;
	u8 cntl1_msk = COMN1SW_MASK | COMP2SW_MASK;
	u8 temp = 0;
	u8 cntl1_val;

		cntl1_val = (CTRL1_TO_USB << COMN1SW_SHIFT) |
				(CTRL1_TO_USB << COMP2SW_SHIFT);

	/* com sw */
	dev_info(info->dev, "%s(0x%02x)\n", __func__, cntl1_val);

        ret = regmap_update_bits(info->max8997->regmap,MAX8997_MUIC_REG_CONTROL1,cntl1_msk,cntl1_val);

	if (ret < 0) {
		dev_err(info->dev, "%s: update reg err\n", __func__);
		return ret;
	}

	return ret;
}


static int set_otgtest(struct max8997_muic_info *info, int en)
{
        int ret = 0;

        /* com sw */
        dev_info(info->dev, "%s(0x%02x)\n", __func__, en);

        ret = regmap_update_bits(info->max8997->regmap,MAX8997_MUIC_REG_CDETCTRL,CHGDETEN_MASK,en << CHGDETEN_SHIFT);

        if (ret < 0) {
                dev_err(info->dev, "%s: update reg err\n", __func__);
                return ret;
        }

        return ret;
}


/* MAX8997 CONTROL2 register */
#define CTRL2_ACCDET_SHIFT              5
#define CTRL2_ACCDET_MASK               (0x1 << CTRL2_ACCDET_SHIFT)

static int set_accdet(struct max8997_muic_info *info, u8 accdet)
{
        int ret = 0;
        u8 cntl2_val = accdet << CTRL2_ACCDET_SHIFT;
        u8 temp = 0;

        /* accdet */
        dev_info(info->dev, "%s(%s) cntl2(0x%02x)\n", __func__,
                        (accdet ? "en" : "dis"), cntl2_val);

        ret = regmap_update_bits(info->max8997->regmap,MAX8997_MUIC_REG_CONTROL2,CTRL2_ACCDET_MASK,cntl2_val);
        if (ret) {
                dev_err(info->dev, "failed to MAX8997_REG_MBCCTRL4\n");
                return ret;
        }

        return ret;
};


static int max8997_muic_set_otg_sel(struct max8997_muic_info *info, int en)
{
	int ret;

	int gpio_otg, gpio_usb;

	u8 on = (u8)!!en;

	if (en == 1) {
		ret = set_accdet(info, 0x00);
		if (ret) {
			dev_err(info->dev, "%s: set_accdet err\n", __func__);
			return ret;
		}
		if (info->otg_en_gpio)
	        	gpiod_set_value_cansleep(info->otg_en_gpio, en);
	}
	else {
                ret = set_accdet(info, 0x01);
                if (ret) {
                        dev_err(info->dev, "%s: set_accdet err\n", __func__);
                        return ret;
                }
                if (info->otg_en_gpio)
                        gpiod_set_value_cansleep(info->otg_en_gpio, en);

	}

	return 0;

}

static int max8997_muic_set_safeout(struct max8997_muic_info *info, int path)
{
	struct regulator *regulator;

	if (path == 1) {
		regulator = regulator_get(NULL, "SAFEOUT1");
		if (IS_ERR(regulator)) {
			dev_err(info->dev, "cant get regulator safe1_sreg\n");
			return -ENODEV;
		}
		if (regulator_is_enabled(regulator)) {
			dev_info(info->dev, "regulator safe1_sreg enabled\n");
			regulator_force_disable(regulator);
		}

		regulator = regulator_get(NULL, "SAFEOUT2");
		if (IS_ERR(regulator)) {
                        dev_err(info->dev, "cant get regulator safe2_sreg\n");
			return -ENODEV;
		}
		if (!regulator_is_enabled(regulator)) {
			dev_info(info->dev, "regulator safe2_sreg disabled\n");
			regulator_enable(regulator);
			}
	} else {
		/* AP_USB_MODE || AUDIO_MODE */
		regulator = regulator_get(NULL, "SAFEOUT1");
                if (IS_ERR(regulator)) {
                        dev_err(info->dev, "cant get regulator safe1_sreg\n");
                        return -ENODEV;
                }
		if (!regulator_is_enabled(regulator))
			regulator_enable(regulator);

		regulator = regulator_get(NULL, "SAFEOUT2");

                if (IS_ERR(regulator)) {
                        dev_err(info->dev, "cant get regulator safe2_sreg\n");
                        return -ENODEV;
                }

		if (regulator_is_enabled(regulator))
			regulator_force_disable(regulator);
	}

	return 0;
}

/*
 * max8997_muic_set_path - Set hardware line according to attached cable
 * @info: the instance including private data of max8997 MUIC
 * @value: the path according to attached cable
 * @attached: the state of cable (true:attached, false:detached)
 *
 * The max8997 MUIC device share outside H/W line among a varity of cables,
 * so this function set internal path of H/W line according to the type of
 * attached cable.
 */
static int max8997_muic_set_path(struct max8997_muic_info *info,
		u8 val, bool attached)
{
	int ret;
	u8 ctrl1, ctrl2 = 0;

	if (attached)
		ctrl1 = val;
	else
		ctrl1 = CONTROL1_SW_OPEN;

	ret = regmap_update_bits(info->max8997->regmap_muic,
			MAX8997_MUIC_REG_CONTROL1, COMP_SW_MASK, ctrl1);
	if (ret < 0) {
		dev_err(info->dev, "failed to update MUIC register\n");
		return ret;
	}

	if (attached)
		ctrl2 |= CONTROL2_CPEN_MASK;	/* LowPwr=0, CPEn=1 */
	else
		ctrl2 |= CONTROL2_LOWPWR_MASK;	/* LowPwr=1, CPEn=0 */

	//ret = regmap_update_bits(info->max8997->regmap_muic,
	//		MAX8997_MUIC_REG_CONTROL2,
	//		CONTROL2_LOWPWR_MASK | CONTROL2_CPEN_MASK, ctrl2);
	//if (ret < 0) {
	//	dev_err(info->dev, "failed to update MUIC register\n");
	//	return ret;
	//}

	//max8997_muic_set_safeout(info,val);

	dev_info(info->dev,
		"CONTROL1 : 0x%02x, CONTROL2 : 0x%02x, state : %s\n",
		ctrl1, ctrl2, attached ? "attached" : "detached");

	return 0;
}

/*
 * max8997_muic_get_cable_type - Return cable type and check cable state
 * @info: the instance including private data of max8997 MUIC
 * @group: the path according to attached cable
 * @attached: store cable state and return
 *
 * This function check the cable state either attached or detached,
 * and then divide precise type of cable according to cable group.
 *	- MAX8997_CABLE_GROUP_ADC
 *	- MAX8997_CABLE_GROUP_CHG
 */
static int max8997_muic_get_cable_type(struct max8997_muic_info *info,
		enum max8997_muic_cable_group group, bool *attached)
{
	int cable_type = 0;
	int adc;
	int chg_type;

	switch (group) {
	case MAX8997_CABLE_GROUP_ADC:
		/*
		 * Read ADC value to check cable type and decide cable state
		 * according to cable type
		 */
		adc = info->status[0] & STATUS1_ADC_MASK;
//		adc >>= STATUS1_ADC_SHIFT;

		/*
		 * Check current cable state/cable type and store cable type
		 * (info->prev_cable_type) for handling cable when cable is
		 * detached.
		 */
		if (adc == MAX8997_MUIC_ADC_OPEN) {
			*attached = false;

			cable_type = info->prev_cable_type;
			info->prev_cable_type = MAX8997_MUIC_ADC_OPEN;
		} else {
			*attached = true;

			cable_type = info->prev_cable_type = adc;
		}
		break;
	case MAX8997_CABLE_GROUP_CHG:
		/*
		 * Read charger type to check cable type and decide cable state
		 * according to type of charger cable.
		 */
		chg_type = info->status[1] & STATUS2_CHGTYP_MASK;
		chg_type >>= STATUS2_CHGTYP_SHIFT;

		if (chg_type == MAX8997_CHARGER_TYPE_NONE) {
			*attached = false;

			cable_type = info->prev_chg_type;
			info->prev_chg_type = MAX8997_CHARGER_TYPE_NONE;
		} else {
			*attached = true;

			/*
			 * Check current cable state/cable type and store cable
			 * type(info->prev_chg_type) for handling cable when
			 * charger cable is detached.
			 */
			cable_type = info->prev_chg_type = chg_type;
		}

		break;
	default:
		dev_err(info->dev, "Unknown cable group (%d)\n", group);
		cable_type = -EINVAL;
		break;
	}

	return cable_type;
}

static int max8997_muic_handle_usb(struct max8997_muic_info *info,
			enum max8997_muic_usb_type usb_type, bool attached)
{
	int ret = 0;
	int val = 0;

        ret = max8997_muic_set_path(info, CONTROL1_SW_USB , attached);
        if (ret < 0) {
                dev_err(info->dev, "failed to update muic register\n");
                return ret;
        }

	switch (usb_type) {
	case MAX8997_USB_HOST:
		//max8997_muic_set_safeout(info,0);
		if (attached) {
			val = 1;
			max8997_muic_set_otg_sel(info,val);
			//set_com_sw(info);
		}
		else {
                	max8997_muic_set_otg_sel(info,val);

		}
		extcon_set_state_sync(info->edev, EXTCON_USB_HOST, attached);
		break;
	case MAX8997_USB_DEVICE:
                //max8997_muic_set_safeout(info,0);

		extcon_set_state_sync(info->edev, EXTCON_USB, attached);
		extcon_set_state_sync(info->edev, EXTCON_CHG_USB_SDP,
					attached);
		break;
	default:
		dev_err(info->dev, "failed to detect %s usb cable\n",
			attached ? "attached" : "detached");
		return -EINVAL;
	}

	return 0;
}

static int max8997_muic_handle_dock(struct max8997_muic_info *info,
			int cable_type, bool attached)
{
	int ret = 0;

	ret = max8997_muic_set_path(info, CONTROL1_SW_AUDIO, attached);
	if (ret) {
		dev_err(info->dev, "failed to update muic register\n");
		return ret;
	}

	switch (cable_type) {
	case MAX8997_MUIC_ADC_AV_CABLE_NOLOAD:
	case MAX8997_MUIC_ADC_FACTORY_MODE_UART_ON:
		extcon_set_state_sync(info->edev, EXTCON_DOCK, attached);
		break;
	default:
		dev_err(info->dev, "failed to detect %s dock device\n",
			attached ? "attached" : "detached");
		return -EINVAL;
	}

	return 0;
}

static int max8997_muic_handle_jig_uart(struct max8997_muic_info *info,
			bool attached)
{
	int ret = 0;

	/* switch to UART */
	ret = max8997_muic_set_path(info, info->path_uart, attached);
	if (ret) {
		dev_err(info->dev, "failed to update muic register\n");
		return ret;
	}

	extcon_set_state_sync(info->edev, EXTCON_JIG, attached);

	return 0;
}



static int max8997_muic_adc_handler(struct max8997_muic_info *info)
{
	int cable_type;
	bool attached;
	int ret = 0;

	/* Check cable state which is either detached or attached */
	cable_type = max8997_muic_get_cable_type(info,
				MAX8997_CABLE_GROUP_ADC, &attached);

	switch (cable_type) {
	case MAX8997_MUIC_ADC_GROUND:
		ret = max8997_muic_handle_usb(info, MAX8997_USB_HOST, attached);
		if (ret < 0)
			return ret;
		break;
	case MAX8997_MUIC_ADC_MHL:
		extcon_set_state_sync(info->edev, EXTCON_DISP_MHL, attached);
		break;
	case MAX8997_MUIC_ADC_FACTORY_MODE_USB_OFF:
	case MAX8997_MUIC_ADC_FACTORY_MODE_USB_ON:
		ret = max8997_muic_handle_usb(info,
					     MAX8997_USB_DEVICE, attached);
		if (ret < 0)
			return ret;
		break;
	case MAX8997_MUIC_ADC_AV_CABLE_NOLOAD:
	case MAX8997_MUIC_ADC_FACTORY_MODE_UART_ON:
		ret = max8997_muic_handle_dock(info, cable_type, attached);
		if (ret < 0)
			return ret;
		break;
	case MAX8997_MUIC_ADC_FACTORY_MODE_UART_OFF:
		ret = max8997_muic_handle_jig_uart(info, attached);
		break;
	case MAX8997_MUIC_ADC_REMOTE_S1_BUTTON:
	case MAX8997_MUIC_ADC_REMOTE_S2_BUTTON:
	case MAX8997_MUIC_ADC_REMOTE_S3_BUTTON:
	case MAX8997_MUIC_ADC_REMOTE_S4_BUTTON:
	case MAX8997_MUIC_ADC_REMOTE_S5_BUTTON:
	case MAX8997_MUIC_ADC_REMOTE_S6_BUTTON:
	case MAX8997_MUIC_ADC_REMOTE_S7_BUTTON:
	case MAX8997_MUIC_ADC_REMOTE_S8_BUTTON:
	case MAX8997_MUIC_ADC_REMOTE_S9_BUTTON:
	case MAX8997_MUIC_ADC_REMOTE_S10_BUTTON:
	case MAX8997_MUIC_ADC_REMOTE_S11_BUTTON:
	case MAX8997_MUIC_ADC_REMOTE_S12_BUTTON:
	case MAX8997_MUIC_ADC_RESERVED_ACC_1:
	case MAX8997_MUIC_ADC_RESERVED_ACC_2:
	case MAX8997_MUIC_ADC_RESERVED_ACC_3:
	case MAX8997_MUIC_ADC_RESERVED_ACC_4:
	case MAX8997_MUIC_ADC_RESERVED_ACC_5:
	case MAX8997_MUIC_ADC_CEA936_AUDIO:
	case MAX8997_MUIC_ADC_PHONE_POWERED_DEV:
	case MAX8997_MUIC_ADC_TTY_CONVERTER:
	case MAX8997_MUIC_ADC_UART_CABLE:
	case MAX8997_MUIC_ADC_CEA936A_TYPE1_CHG:
	case MAX8997_MUIC_ADC_CEA936A_TYPE2_CHG:
	case MAX8997_MUIC_ADC_AUDIO_MODE_REMOTE:
		/*
		 * This cable isn't used in general case if it is specially
		 * needed to detect additional cable, should implement
		 * proper operation when this cable is attached/detached.
		 */
		dev_info(info->dev,
			"cable is %s but it isn't used (type:0x%x)\n",
			attached ? "attached" : "detached", cable_type);
		return -EAGAIN;
	default:
		dev_err(info->dev,
			"failed to detect %s unknown cable (type:0x%x)\n",
			attached ? "attached" : "detached", cable_type);
		return -EINVAL;
	}

	return 0;
}

static int max8997_muic_chg_handler(struct max8997_muic_info *info)
{
	int chg_type;
	bool attached;
	int adc;

	chg_type = max8997_muic_get_cable_type(info,
				MAX8997_CABLE_GROUP_CHG, &attached);

        adc = info->status[0] & STATUS1_ADC_MASK;
        adc >>= STATUS1_ADC_SHIFT;

	switch (chg_type) {
	case MAX8997_CHARGER_TYPE_NONE:
		break;
	case MAX8997_CHARGER_TYPE_USB:

		if ((adc & STATUS1_ADC_MASK) == MAX8997_MUIC_ADC_OPEN) {
			max8997_muic_handle_usb(info,
					MAX8997_USB_DEVICE, attached);
		}
		break;
	case MAX8997_CHARGER_TYPE_DOWNSTREAM_PORT:
		extcon_set_state_sync(info->edev, EXTCON_CHG_USB_CDP,
					attached);
		break;
	case MAX8997_CHARGER_TYPE_DEDICATED_CHG:
		extcon_set_state_sync(info->edev, EXTCON_CHG_USB_DCP,
					attached);
		break;
	case MAX8997_CHARGER_TYPE_500MA:
		extcon_set_state_sync(info->edev, EXTCON_CHG_USB_SLOW,
					attached);
		break;
	case MAX8997_CHARGER_TYPE_1A:
		extcon_set_state_sync(info->edev, EXTCON_CHG_USB_FAST,
					attached);
		break;
	default:
		dev_err(info->dev,
			"failed to detect %s unknown chg cable (type:0x%x)\n",
			attached ? "attached" : "detached", chg_type);
		return -EINVAL;
	}

	if (attached && chg_type != MAX8997_CHARGER_TYPE_NONE && ((adc & STATUS1_ADC_MASK) != MAX8997_MUIC_ADC_GROUND)) {
		max8997_endis_charging(info,true);
		}
	else {
		max8997_endis_charging(info,false);
	}

	return 0;
}
static void max8997_muic_irq_work(struct work_struct *work)
{
	struct max8997_muic_info *info = container_of(work,
			struct max8997_muic_info, irq_work);
	int irq_type = 0;
	int i, ret;
	u8 adc, chgtyp, adcerr, chgdetrun;

	if (!info->edev)
		return;

	mutex_lock(&info->mutex);

	for (i = 0; i < ARRAY_SIZE(muic_irqs); i++)
		if (info->irq == muic_irqs[i].virq)
			irq_type = muic_irqs[i].irq;

	ret = regmap_bulk_read(info->max8997->regmap_muic,
				MAX8997_MUIC_REG_STATUS1, info->status, 2);
	if (ret) {
		dev_err(info->dev, "failed to read muic register\n");
		mutex_unlock(&info->mutex);
		return;
	}

	adc = info->status[0] & STATUS1_ADC_MASK;
	adcerr = info->status[0] & STATUS1_ADCERR_MASK;
	chgtyp = info->status[1] & STATUS2_CHGTYP_MASK;
	chgdetrun = info->status[1] & STATUS2_CHGDETRUN_MASK;



	if (chgdetrun) {
		dev_warn(info->dev, "%s: charger det run, ignore\n", __func__);
                mutex_unlock(&info->mutex);
		return ret;
	}

	switch (irq_type) {
	case MAX8997_MUICIRQ_ADCERROR:
	case MAX8997_MUICIRQ_ADCLOW:
	case MAX8997_MUICIRQ_ADC:
                dev_info(info->dev, "adc irq occured - run adc handler\n");

		/* Handle all of cable except for charger cable */
		ret = max8997_muic_adc_handler(info);
		break;
	case MAX8997_MUICIRQ_VBVOLT:
	case MAX8997_MUICIRQ_DBCHG:
	case MAX8997_MUICIRQ_DCDTMR:
	case MAX8997_MUICIRQ_CHGDETRUN:
	case MAX8997_MUICIRQ_CHGTYP:
		dev_info(info->dev, "chg irq occured - run chg handler\n");
		/* Handle charger cable */
		ret = max8997_muic_chg_handler(info);
		break;
	case MAX8997_MUICIRQ_OVP:
		break;
	default:
		dev_info(info->dev, "misc interrupt: irq %d occurred\n",
				irq_type);
		mutex_unlock(&info->mutex);
		return;
	}

	if (ret < 0)
		dev_err(info->dev, "failed to handle MUIC interrupt\n");

	mutex_unlock(&info->mutex);
}

static irqreturn_t max8997_muic_irq_handler(int irq, void *data)
{
	struct max8997_muic_info *info = data;

	dev_info(info->dev, "irq:%d\n", irq);
	info->irq = irq;

	schedule_work(&info->irq_work);

	return IRQ_HANDLED;
}

static int max8997_muic_detect_dev(struct max8997_muic_info *info)
{
	int ret = 0;
	int adc;
	int chg_type;
	bool attached;

	mutex_lock(&info->mutex);

	/* Read STATUSx register to detect accessory */
	ret = regmap_bulk_read(info->max8997->regmap_muic,
			MAX8997_MUIC_REG_STATUS1, info->status, 2);
	if (ret) {
		dev_err(info->dev, "failed to read MUIC register\n");
		mutex_unlock(&info->mutex);
		return ret;
	}

	adc = max8997_muic_get_cable_type(info, MAX8997_CABLE_GROUP_ADC,
					&attached);
	if (attached && adc != MAX8997_MUIC_ADC_OPEN) {
		ret = max8997_muic_adc_handler(info);
		if (ret < 0) {
			dev_err(info->dev, "Cannot detect ADC cable\n");
			mutex_unlock(&info->mutex);
			return ret;
		}
	}

	chg_type = max8997_muic_get_cable_type(info, MAX8997_CABLE_GROUP_CHG,
					&attached);
	if (attached && chg_type != MAX8997_CHARGER_TYPE_NONE) {
		ret = max8997_muic_chg_handler(info);
		if (ret < 0) {
			dev_err(info->dev, "Cannot detect charger cable\n");
			mutex_unlock(&info->mutex);
			return ret;
		}

	}

	mutex_unlock(&info->mutex);

	return 0;
}

static void max8997_muic_detect_cable_wq(struct work_struct *work)
{
	struct max8997_muic_info *info = container_of(to_delayed_work(work),
				struct max8997_muic_info, wq_detcable);
	int ret;

	ret = max8997_muic_detect_dev(info);
	if (ret < 0)
		dev_err(info->dev, "failed to detect cable type\n");
}

static int max8997_muic_probe(struct platform_device *pdev)
{
	struct max8997_dev *max8997 = dev_get_drvdata(pdev->dev.parent);
	struct max8997_platform_data *pdata = dev_get_platdata(max8997->dev);
	struct max8997_muic_info *info;
	int delay_jiffies;
	int ret, i;

	info = devm_kzalloc(&pdev->dev, sizeof(struct max8997_muic_info),
			    GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	info->max8997 = max8997;

	platform_set_drvdata(pdev, info);
	mutex_init(&info->mutex);

	INIT_WORK(&info->irq_work, max8997_muic_irq_work);

	for (i = 0; i < ARRAY_SIZE(muic_irqs); i++) {
		struct max8997_muic_irq *muic_irq = &muic_irqs[i];
		int virq = 0;

		virq = regmap_irq_get_virq(max8997->irq_data_muic,
					muic_irq->irq);
		if (virq <= 0) {
			ret = -EINVAL;
			goto err_irq;
		}
		muic_irq->virq = virq;

                ret = devm_request_threaded_irq(&pdev->dev, virq, NULL,
                                max8997_muic_irq_handler,
                                IRQF_NO_SUSPEND,
                                muic_irq->name, info);

		if (ret) {
			dev_err(&pdev->dev,
				"failed: irq request (IRQ: %d, error :%d)\n",
				muic_irq->irq, ret);
			goto err_irq;
		}
	}

	/* External connector */
	info->edev = devm_extcon_dev_allocate(&pdev->dev, max8997_extcon_cable);
	if (IS_ERR(info->edev)) {
		dev_err(&pdev->dev, "failed to allocate memory for extcon\n");
		ret = -ENOMEM;
		goto err_irq;
	}

	ret = devm_extcon_dev_register(&pdev->dev, info->edev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register extcon device\n");
		goto err_irq;
	}

	if (pdata && pdata->muic_pdata) {
		struct max8997_muic_platform_data *muic_pdata
			= pdata->muic_pdata;

		/* Initialize registers according to platform data */
		for (i = 0; i < muic_pdata->num_init_data; i++) {
			regmap_write(info->max8997->regmap_muic,
					muic_pdata->init_data[i].addr,
					muic_pdata->init_data[i].data);
		}

		/*
		 * Default usb/uart path whether UART/USB or AUX_UART/AUX_USB
		 * h/w path of COMP2/COMN1 on CONTROL1 register.
		 */
		if (muic_pdata->path_uart)
			info->path_uart = muic_pdata->path_uart;
		else
			info->path_uart = CONTROL1_SW_UART;

		if (muic_pdata->path_usb)
			info->path_usb = muic_pdata->path_usb;
		else
			info->path_usb = CONTROL1_SW_USB;

		/*
		 * Default delay time for detecting cable state
		 * after certain time.
		 */
		if (muic_pdata->detcable_delay_ms)
			delay_jiffies =
				msecs_to_jiffies(muic_pdata->detcable_delay_ms);
		else
			delay_jiffies = msecs_to_jiffies(DELAY_MS_DEFAULT);
	} else {
		info->path_uart = CONTROL1_SW_UART;
		info->path_usb = CONTROL1_SW_USB;
		delay_jiffies = msecs_to_jiffies(DELAY_MS_DEFAULT);
	}

        info->otg_en_gpio = gpiod_get(max8997->dev, "otg_en" , GPIOD_OUT_LOW);

        if (IS_ERR(info->otg_en_gpio)) {
                ret = PTR_ERR(info->otg_en_gpio);
                dev_err(max8997->dev,
                       "Failed to get OTG_EN gpio \n");
        }

        info->usb_sel_gpio = gpiod_get(max8997->dev, "usb_sel" , GPIOD_OUT_LOW);

        if (IS_ERR(info->usb_sel_gpio)) {
                ret = PTR_ERR(info->usb_sel_gpio);
                dev_err(max8997->dev,
                       "Failed to get USB_SEL gpio \n");
        }

	max8997_muic_detect_dev(info);

	/* Set ADC debounce time */
	max8997_muic_set_debounce_time(info, ADC_DEBOUNCE_TIME_25MS);

	/*
	 * Detect accessory after completing the initialization of platform
	 *
	 * - Use delayed workqueue to detect cable state and then
	 * notify cable state to notifiee/platform through uevent.
	 * After completing the booting of platform, the extcon provider
	 * driver should notify cable state to upper layer.
	 */
	INIT_DELAYED_WORK(&info->wq_detcable, max8997_muic_detect_cable_wq);
	queue_delayed_work(system_power_efficient_wq, &info->wq_detcable,
			delay_jiffies);

	return 0;

err_irq:
	while (--i >= 0)
		free_irq(muic_irqs[i].virq, info);
	return ret;
}

static int max8997_muic_remove(struct platform_device *pdev)
{
	struct max8997_muic_info *info = platform_get_drvdata(pdev);
	int i;


	gpiod_put(info->otg_en_gpio);
        gpiod_put(info->usb_sel_gpio);


	for (i = 0; i < ARRAY_SIZE(muic_irqs); i++)
		free_irq(muic_irqs[i].virq, info);
	cancel_work_sync(&info->irq_work);

	return 0;
}

static const struct platform_device_id max8997_extcon_table[] = {
        { .name = DEV_NAME },
        {},
};
MODULE_DEVICE_TABLE(platform, max8997_extcon_table);


static struct platform_driver max8997_muic_driver = {
	.driver		= {
		.name	= DEV_NAME,
	},
	.probe		= max8997_muic_probe,
	.remove		= max8997_muic_remove,
};

//module_platform_driver(max8997_muic_driver);



static int __init max8997_extcon_init(void)
{

        return platform_driver_register(&max8997_muic_driver);
}
module_init(max8997_extcon_init);

static void __exit max8997_extcon_exit(void)
{

        platform_driver_unregister(&max8997_muic_driver);
}
module_exit(max8997_extcon_exit);



MODULE_DESCRIPTION("Maxim MAX8997 Extcon driver");
MODULE_AUTHOR("Donggeun Kim <dg77.kim@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:extcon-max8997");

