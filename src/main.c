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


#include <mgmt/mcumgr/smp_bt.h>
#include "os_mgmt/os_mgmt.h"
#include "img_mgmt/img_mgmt.h"

#include <nrfx_saadc.h>

//#include "waermelehre/ds18b20.h"
//#include "waermelehre/ds18b20_sensor.h"

#define SLEEP_TIME_MS	1

void main(void)
{
	
	int err;
	printk("build time: " __DATE__ " " __TIME__ "\n");
	os_mgmt_register_group();
	img_mgmt_register_group();
	err = smp_bt_register();

	if (err) {
	printk("SMP BT register failed (err: %d)", err);
	}
	
	
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
	init_ads1231();
	led_off(red_led_dev,0);
	led_off(blue_led_dev,0);
	/*
	init_BAS();

	while (true)
	{

		AnalogRead();
		k_sleep(K_SECONDS(1));
	}
	*/
	/*
	while (true){
		float test = AnalogRead(2);
		printk("voltage value: %f\n ",test);
		k_sleep(K_MSEC(1000));
		
	}
	*/
	//loadcell test
	/*
	sleep_ads1231(false);
	
	while (true)
	{
		float weight;
		long weight_raw;

		weight_raw = read_loadcell();
		weight = ads1231_data.calibrationWeight*(weight_raw-ads1231_data.unweighted)*1.0/(ads1231_data.weighted-ads1231_data.unweighted);

		printk("loadcell value raw : %ld calc %ld\n ",weight_raw,weight);
		k_sleep(K_MSEC(1000));
	}
	*/
	
}
