/*
 * drivers/misc/bcm4329-rfkill.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/err.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/bcm4329-rfkill.h>
#include <linux/regulator/consumer.h>

static int bcm4329_rfkill_set_power(void *data, bool blocked)
{
	struct platform_device *pdev = data;
	struct bcm4329_platform_data *plat = pdev->dev.platform_data;
	struct regulator *regulator;

	regulator = regulator_get(&pdev->dev, "Vdd");

	if (IS_ERR(regulator)) {
		pr_err("%s unable to get regulator Vdd\n", __func__);
		return PTR_ERR(regulator);
	}

	if (!blocked) {
		regulator_enable(regulator);
		gpio_set_value(plat->gpio_reset, 0);
		if (plat->gpio_pwr!=-1)
			gpio_set_value(plat->gpio_pwr, 0);
		msleep(plat->delay);
		if (plat->gpio_pwr!=-1)
			gpio_set_value(plat->gpio_pwr, 1);
		gpio_set_value(plat->gpio_reset, 1);
	} else {
		gpio_set_value(plat->gpio_reset, 0);
		regulator_disable(regulator);
	}

	regulator_put(regulator);
	return 0;
}

static const struct rfkill_ops bcm4329_bt_rfkill_ops = {
	.set_block = bcm4329_rfkill_set_power,
};

static int bcm4329_rfkill_probe(struct platform_device *pdev)
{
	struct bcm4329_platform_data *plat = pdev->dev.platform_data;
	struct rfkill *rfkill;
	int rc;
	int gp_reset = -1;

	if (!plat) {
		pr_err("%s no platform data\n", __func__);
		return -ENOSYS;
	}
	if ((plat->gpio_reset == -1) && (plat->gpio_pwr == -1)) {
		pr_err("%s gpio_reset and gpio_pwr not available\n", __func__);
		pr_err("either one should be available for rfkill driver to function\n");
		return -ENOSYS;
	}
	rc = gpio_request(plat->gpio_reset, "bcm4329_reset");
	if (rc < 0)
		pr_info("%s bcm4329_reset pin not available\n", __func__);
	else
		gp_reset=1;

	if (plat->gpio_pwr!=-1)
	{
		rc = gpio_request(plat->gpio_pwr, "bcm4329_pwr");
		if (rc < 0) {
			pr_info("%s bcm4329_pwr pin not available\n", __func__);
			if (!gp_reset) {
				pr_err("%s failed to get gpio_reset and gpio_pwr, abort \n", __func__);
				goto free_bcm_res;
			}
		}
		gpio_direction_output(plat->gpio_pwr,0);
	}

	rfkill = rfkill_alloc("bcm4329-rfkill", &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &bcm4329_bt_rfkill_ops, pdev);
	if (!rfkill)
		goto free_bcm_res;

	platform_set_drvdata(pdev, rfkill);
	gpio_direction_output(plat->gpio_reset, 0);

	rc = rfkill_register(rfkill);
	if (rc < 0) {
		rfkill_destroy(rfkill);
		goto fail_alloc;
	}

	return 0;

fail_alloc:
	rfkill_destroy(rfkill);
free_bcm_res:
	gpio_free(plat->gpio_reset);
	if (plat->gpio_pwr!=-1)
		gpio_free(plat->gpio_pwr);
	return -ENODEV;
}

static int bcm4329_rfkill_remove(struct platform_device *pdev)
{
	struct bcm4329_platform_data *plat = pdev->dev.platform_data;
	struct rfkill *rfkill = platform_get_drvdata(pdev);

	rfkill_unregister(rfkill);
	rfkill_destroy(rfkill);
	gpio_free(plat->gpio_reset);
	if (plat->gpio_pwr!=-1)
		gpio_free(plat->gpio_pwr);
	return 0;
}

static struct platform_driver bcm4329_rfkill_driver = {
	.probe = bcm4329_rfkill_probe,
	.remove = bcm4329_rfkill_remove,
	.driver = {
		   .name = "bcm4329-rfkill",
		   .owner = THIS_MODULE,
	},
};

static int __init bcm4329_rfkill_init(void)
{
	return platform_driver_register(&bcm4329_rfkill_driver);
}

static void __exit bcm4329_rfkill_exit(void)
{
	platform_driver_unregister(&bcm4329_rfkill_driver);
}

module_init(bcm4329_rfkill_init);
module_exit(bcm4329_rfkill_exit);

MODULE_DESCRIPTION("BCM4329 rfkill");
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL");
