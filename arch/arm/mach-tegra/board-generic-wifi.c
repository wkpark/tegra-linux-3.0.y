#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>

#include "nvodm_query_discovery.h"
#include "gpio-names.h"
#include "board.h"

#define	BOARD_WLAN_PWR	TEGRA_GPIO_PK5
#define	BOARD_WLAN_RST	TEGRA_GPIO_PK6

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;


int board_wifi_status_register(void (*callback)(int card_present,
	void *dev_id), void *dev_id)
{
	pr_debug("%s:\n", __func__);
	if (wifi_status_cb) {
		pr_err("%s: callback already registered\n", __func__);
		return -EAGAIN;
	}
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}


static int board_wifi_power_state;
static int board_wifi_power(int on)
{
	pr_debug("%s: %d\n", __func__, on);

	mdelay(100);
	gpio_set_value(BOARD_WLAN_PWR, on);
	mdelay(100);
	gpio_set_value(BOARD_WLAN_RST, on);
	mdelay(200);

	board_wifi_power_state = on;
	return 0;
}

static int board_wifi_reset_state;
static int board_wifi_reset(int on)
{
	pr_debug("%s: %d do nothing\n", __func__, on);
	board_wifi_reset_state = on;
	return 0;
}

static int board_wifi_cd; /* WIFI virtual 'card detect' status */
static int board_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	board_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static struct wifi_platform_data board_wifi_control = {
	.set_power	= board_wifi_power,
	.set_reset	= board_wifi_reset,
	.set_carddetect	= board_wifi_set_carddetect,
};

static struct platform_device board_wifi_device = {
	.name	= "bcm4329_wlan",
	.id	= 1,
	.dev	= {
			.platform_data = &board_wifi_control,
		},
};

static void __init board_wlan_gpio(void)
{
	tegra_gpio_enable(BOARD_WLAN_PWR);
	gpio_request(BOARD_WLAN_PWR, "wlan_pwr");
	gpio_direction_output(BOARD_WLAN_PWR, 0);

	tegra_gpio_enable(BOARD_WLAN_RST);
	gpio_request(BOARD_WLAN_RST, "wlan_rst");
	gpio_direction_output(BOARD_WLAN_RST, 0);
}

int __init board_setup_wifi(void)
{
	pr_debug("%s: start\n", __func__);
	board_wlan_gpio();
	return platform_device_register(&board_wifi_device);
}
