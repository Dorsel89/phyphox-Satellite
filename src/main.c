/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <stddef.h>
#include <sys/util.h>
#include <zephyr/types.h>

#include "led.h"
#include "bmpZephyr.h"
#include "ble.h"
#include "mlxZephyr.h"
#include "icm42605.h"
#include "shtc3.h"
#include "mprls.h"
#include "ds18b20.h"
#include "ads1231.h"

//#include <nrfx_saadc.h>

//#include "waermelehre/ds18b20.h"
//#include "waermelehre/ds18b20_sensor.h"

#define SLEEP_TIME_MS	1

void main(void)
{
	printk("Hello World %s\n", CONFIG_BOARD);
	led_blink(red_led_dev,0,LED_ON_TIME_MS,LED_SLEEP_TIME_MS); 	// blink led until init done (led_off at the end)

	init_ble();
	int ret = i2c_configure(device_get_binding("i2c"),I2C_SPEED_SET(I2C_SPEED_FAST));

	init_shtc();
	init_mpr();
	init_bmp();
	init_mlx();
	init_icm(AFS_2G, GFS_15_125DPS, AODR_25Hz, GODR_25Hz);
	init_ds18b20();
	led_off(red_led_dev,0);

	//loadcell test
	init_ads1231();
	sleep_ads1231(false);
	while (true)
	{
		long weight_raw = read_loadcell();
		long weight = 433*(weight_raw-46000)/(-144100-46000);
		printk("loadcell value raw : %ld calc %ld\n ",weight_raw,weight);
		k_sleep(K_MSEC(120));
	}
	
}
