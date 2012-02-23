/*
 * linux/drivers/star/star_vib.c
 *
 * Copyright (C) 2010 LG Electronics, Inc.
 * Author: sk.hwang@lge.com
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>

#include "nvcommon.h"
#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"

typedef struct star_vib_device_data
{
	NvOdmServicesGpioHandle h_vib_gpio;
	NvOdmGpioPinHandle  h_vib_gpio_pin;
	NvOdmServicesPmuHandle  h_vib_pmu;
	struct hrtimer timer;

	NvU32   vdd_id;
	NvU32   en_pin;
	NvU32   en_port;
	struct delayed_work delayed_work_vib;
} star_vib_device;

static star_vib_device *g_vib;
static NvBool vib_enable = false;
static volatile int vib_stay_time = 0;
static atomic_t is_enable = ATOMIC_INIT(0);
//static struct timer_list vib_timer;

#if 0
static NvU32 toggle = 0;
static void star_vib_work_func(struct work_struct *work)
{
	NvU32 gpio_state = 0;	
	pr_debug("vibrator test... value = %d\n", toggle);

	NvOdmGpioConfig(g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, NvOdmGpioPinMode_Output);
	
	NvOdmGpioSetState(g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, toggle);
	if (toggle == 0)
		toggle++;
	else	
		toggle = 0;

	NvOdmGpioGetState(g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, &gpio_state); 	
	pr_debug("[skhwang] VIB_EN = %d\n", gpio_state);

	NvOdmOsSleepMS(1000);
	schedule_delayed_work(&g_vib->delayed_work_vib,150);
}
#endif

// 20100903 taewan.kim@lge.com Power control bug fix [START]
static int star_vib_set_power_rail(NvU32 vdd_id, NvBool is_enable)
{
	NvOdmServicesPmuHandle h_pmu = NvOdmServicesPmuOpen();
	NvOdmServicesPmuVddRailCapabilities vddrailcap;
	NvU32 settletime;

	if (h_pmu) {
		NvOdmServicesPmuGetCapabilities(h_pmu, vdd_id, &vddrailcap);
		if (is_enable) {
			pr_debug("%s: vibrator PMU enable\n", __func__);
			pr_info("%s: Max millivolt=%d\n", __func__, vddrailcap.requestMilliVolts);
			NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, vddrailcap.requestMilliVolts, &settletime);
		} else {
			pr_debug("[skhwang] vibrator PMU do not enable\n");
			NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, NVODM_VOLTAGE_OFF, &settletime);
		}

		if (settletime)
			NvOdmOsWaitUS(settletime);

		NvOdmServicesPmuClose(h_pmu);
		pr_debug("[skhwang] vibrator voltage = %d or %d\n",
				vddrailcap.requestMilliVolts, vddrailcap.MinMilliVolts);

		return 0;
	}

	return -1;
}
// 20100903 taewan.kim@lge.com Power control bug fix [END]

static void star_vib_vibrating(NvBool on)
{
	NvOdmGpioConfig( g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, NvOdmGpioPinMode_Output);
	NvOdmGpioSetState( g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, !!on);
}

//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]
#ifdef CONFIG_MACH_STAR_SKT_REV_E
NvOdmServicesPwmHandle hOdmPwm = NULL;

NvBool star_vibrator_pwn_set(NvBool IsEnable)
{
	NvU32 RequestedPeriod, ReturnedPeriod;
	NvU32 gain;
	RequestedPeriod = 22930;

	gain = IsEnable << 16;

	if (!hOdmPwm) {
		pr_err("%s: failed to open NvOdmPwmOpen\n", __func__);
		return NV_FALSE;
	}

	if (IsEnable)
	{		
		pr_debug("[%s] Start Enable...duty[0x%x]\n", __func__, gain);
		NvOdmPwmConfig(hOdmPwm, NvOdmPwmOutputId_PWM3, NvOdmPwmMode_Enable, gain, &RequestedPeriod, &ReturnedPeriod);
	}
	else
	{
		pr_debug("[%s] Start Disable...duty[0x%x]\n", __func__, gain);
		NvOdmPwmConfig(hOdmPwm, NvOdmPwmOutputId_PWM3, NvOdmPwmMode_Disable, 0, &RequestedPeriod, &ReturnedPeriod);
	}

	return NV_TRUE;
}
#endif
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]

static void star_vib_enable(NvBool is_enable)
{
	if( is_enable) {
        //star_vib_set_power_rail( g_vib->h_vib_pmu, g_vib->vdd_id, is_enable);
		star_vib_vibrating(is_enable);
//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]
#ifdef CONFIG_MACH_STAR_SKT_REV_E
		star_vibrator_pwn_set(is_enable);
#endif
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]
	} else {
        //star_vib_set_power_rail( g_vib->h_vib_pmu, g_vib->vdd_id, is_enable);
		star_vib_vibrating(is_enable);
//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]
#ifdef CONFIG_MACH_STAR_SKT_REV_E
		star_vibrator_pwn_set(is_enable);
#endif
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]
	}
}
/*
static void star_vib_timeout(unsigned long arg )
{
	printk("[skhwang][%s] timeout...\n", __func__);
	star_vib_enable(NV_FALSE);
	del_timer(&vib_timer);
}
*/
static enum hrtimer_restart star_vib_timeout(struct hrtimer *timer)
{
	pr_debug("[%s] enter timer function...\n", __func__);
	pr_debug("vibrator off...\n");
//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]	
#ifdef CONFIG_MACH_STAR_SKT_REV_E
	star_vib_vibrating(NV_FALSE);
#else
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]
	atomic_set(&is_enable, 0);
	star_vib_enable(NV_FALSE);
#endif

	return HRTIMER_NORESTART;
}

static ssize_t star_vib_stay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int time = vib_stay_time;
	return sprintf(buf, "%d\n", time);	
}

static ssize_t star_vib_stay_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	int value;
	unsigned long timeout;

	sscanf(buf, "%ld", &value);
	pr_debug("%s: Timeout value = %ld ms\n", __func__, value);
	vib_stay_time = value;
	atomic_set(&is_enable, 1);
	if (atomic_read(&is_enable) == 1) {
#if 0
		timeout = (unsigned long)(value * 1000 * 1000 );
		hrtimer_start(&g_vib->timer, ktime_set(0, timeout), HRTIMER_MODE_ABS);
#else
		if (value < 1000) {
			timeout = (unsigned long)(value * 1000 * 1000);
//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]
#ifndef CONFIG_MACH_STAR_SKT_REV_E
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]
			star_vib_enable(NV_TRUE);
#endif
			hrtimer_start(&g_vib->timer, ktime_set(0, timeout), HRTIMER_MODE_REL);
		} else {
			timeout = (long)value / 1000;
//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]
#ifndef CONFIG_MACH_STAR_SKT_REV_E
			star_vib_enable(NV_TRUE);
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]
#endif
			hrtimer_start(&g_vib->timer, ktime_set((long)timeout, 0), HRTIMER_MODE_REL);
		}
#endif
	}

	return count;
}

static ssize_t star_vib_onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", vib_enable==true );	
	return (ssize_t)(strlen(buf)+1);
}
//spinlock_t vib_lock;
#define MS_TO_NS(x)	(x * 1E6L)
static ssize_t star_vib_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32 val = 0;
//	unsigned int timeout = 0;
	val = simple_strtoul(buf, NULL, 10);

//	timeout = vib_stay_time;
//	spin_lock(&vib_lock);
	if (val) {
		pr_debug("vibrator on...\n");
//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]
#ifdef CONFIG_MACH_STAR_SKT_REV_E
		star_vib_enable(val);
#endif
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]
		
//		star_vib_enable(NV_TRUE);
//		pr_debug("start timer...\n");
//		pr_debug("app's timeout = %d\n", vib_stay_time);//timeout);
		pr_debug("Vibrator enabled!!!\n");
		atomic_set(&is_enable, 1);
	/*
		if( timeout == 1 )
			hrtimer_start(&g_vib->timer, ktime_set(0, timeout*9000000), HRTIMER_MODE_REL);
		else if( timeout == 0)
			hrtimer_start(&g_vib->timer, ktime_set(1,10000000), HRTIMER_MODE_REL);
		else if( timeout >=2 && timeout <=10 )
			hrtimer_start(&g_vib->timer, ktime_set(0, timeout*900000), HRTIMER_MODE_REL);
	*/
	/*
		if( timeout == 1 )
		{
			#if 1//VIB_DEBUG
			printk("timeout == %d \n", timeout );
			#endif
			hrtimer_start(&g_vib->timer, ktime_set(0, timeout*9000000), HRTIMER_MODE_REL);
		}
		else if( timeout == 0)
		{
			#if 1//VIB_DEBUG
			printk("timeout == %d \n", timeout );
			#endif
			hrtimer_start(&g_vib->timer, ktime_set(1,10000000), HRTIMER_MODE_REL);
		}
		else
		{
			#if 1//VIB_DEBUG
			printk("timeout == %d \n", timeout );
			#endif
			hrtimer_start(&g_vib->timer, ktime_set(0, timeout*1000000), HRTIMER_MODE_REL);
		}
	*/		
	} else {
		pr_debug("vibrator off...\n");
		atomic_set(&is_enable, 0);
		star_vib_enable(NV_FALSE);
	}
//	spin_unlock(&vib_lock);
	return count;
}

static DEVICE_ATTR(onoff, 0666, star_vib_onoff_show, star_vib_onoff_store);
static DEVICE_ATTR(stay, 0666, star_vib_stay_show, star_vib_stay_store);

static struct attribute *star_vib_attributes[] = {
	&dev_attr_onoff.attr,	
	&dev_attr_stay.attr,
	NULL,
};

static const struct attribute_group star_vib_group = {
	.attrs = star_vib_attributes,
};

static int __devinit star_vib_probe(struct platform_device *pdev)
{
	const NvOdmPeripheralConnectivity *pcon;
	int err;
	NvBool found_gpio=NV_FALSE;
	int loop;
	struct device *dev = &pdev->dev;

	pr_debug("[%s] probing vibrator\n",__func__);

	g_vib = kzalloc(sizeof(*g_vib), GFP_KERNEL);
	if (g_vib == NULL) {
		err = -1;
		pr_debug("[%s] fail vib\n", __func__);
		return err;
	}

        // 20100903 taewan.kim@lge.com Power control bug fix [START]
	/*g_vib->h_vib_pmu = NvOdmServicesPmuOpen();
	if( !g_vib->h_vib_pmu )
	{err=-ENOSYS; return err;}*/
        // 20100903 taewan.kim@lge.com Power control bug fix [END]

	pcon = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(NV_ODM_GUID('v','i','b','r','a','t','o','r'));
	for (loop = 0; loop< pcon->NumAddress; loop++) {
		switch(pcon->AddressList[loop].Interface) {
		case NvOdmIoModule_Gpio:
			g_vib->en_port = pcon->AddressList[loop].Instance;
			g_vib->en_pin = pcon->AddressList[loop].Address;
			found_gpio = NV_TRUE;
			break;
		case NvOdmIoModule_Vdd:
			g_vib->vdd_id = pcon->AddressList[loop].Address;
			pr_debug("VIB POWER %d\n", g_vib->vdd_id);
			if (star_vib_set_power_rail( g_vib->vdd_id, NV_TRUE) != 0)
				return -ENOSYS;
			break;
		default:
			break;
		}
	}

	pr_debug("[skhwang][%s] : vibrator Int Port = %c, Int Pin = %d\n", __func__, (g_vib->en_port+'a'), g_vib->en_pin);
	pr_debug("vibrator.....\n");

	g_vib->h_vib_gpio = NvOdmGpioOpen();
	if (!g_vib->h_vib_gpio) {
		pr_err("[skhwang][%s] : Failed to open gpio\n",__func__);
		err = - ENOSYS;
		return err;
	}

	g_vib->h_vib_gpio_pin = NvOdmGpioAcquirePinHandle(g_vib->h_vib_gpio, g_vib->en_port, g_vib->en_pin);
	if (!g_vib->h_vib_gpio_pin) {
		pr_err("[skhwang][%s] : Failed to acquire the pin handle\n",__func__);
		err = -ENOSYS;
		return err;
	}

//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]
#ifdef CONFIG_MACH_STAR_SKT_REV_E
	hOdmPwm = NvOdmPwmOpen();
#endif
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]
	
//	NvOdmGpioConfig( g_vib->vib_gpio, g_vib->en_port, NvOdmGpioPinMode_Output);
//	INIT_DELAYED_WORK(&g_vib->delayed_work_vib, star_vib_work_func );
//	schedule_delayed_work(&g_vib->delayed_work_vib, 400);	
	hrtimer_init(&g_vib->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_vib->timer.function = star_vib_timeout;
	
	pr_debug("[vib] sysfs_create_group before....\n");

	//create sys filesystem.
	if (sysfs_create_group(&dev->kobj, &star_vib_group)) {
		pr_err("[star vib] Failed to create sys filesystem\n");
		err = -ENOSYS;
		return err;
	}

	return 0;
}

int star_vib_suspend(struct platform_device *dev, pm_message_t state)
{
	//printk("[SLEEP] %s start \n", __func__);
	star_vib_set_power_rail( g_vib->vdd_id, NV_FALSE);

	return 0;
}

int star_vib_resume(struct platform_device *dev)
{
	//printk("[SLEEP] %s start \n", __func__);
	star_vib_set_power_rail( g_vib->vdd_id, NV_TRUE);

	return 0;
}


static int __devexit star_vib_remove( struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver star_vib_driver = {
	.probe = star_vib_probe,
	.remove = __devexit_p(star_vib_remove),
	.suspend = star_vib_suspend,
	.resume = star_vib_resume,
	.driver = {
		.name = "star_vib_name",
		.owner = THIS_MODULE,
	},
};

static int __init star_vib_init(void)
{
	return platform_driver_register(&star_vib_driver);
}

static void __exit star_vib_exit(void)
{
	platform_driver_unregister(&star_vib_driver);
}

module_init(star_vib_init);
module_exit(star_vib_exit);

MODULE_AUTHOR("sk.hwang@lge.com");
MODULE_DESCRIPTION("driver of star viberator");
MODULE_LICENSE("GPL");
