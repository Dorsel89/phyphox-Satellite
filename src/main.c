/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <stddef.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <zephyr/types.h>
#include <drivers/i2c.h>
#include "workQueue.h"
#include "bmpZephyr.h"
#include "ble.h"
#include "mlxZephyr.h"
#include "phyphox.h"
#include "icm42605.h"
#include "shtc3.h"

//#include "debug.h"
#define SLEEP_TIME_MS	1



#define I2C_DEV DT_LABEL(DT_NODELABEL(myi2c))

int8_t error =0;
void btn1_handle(struct k_work * work) {
    printk("test");
}

void main(void)
{
	printk("Hello World %s\n", CONFIG_BOARD);

	uint16_t counter =0;
	
	//k_work_init(&some_work, btn1_handle);
	

	struct device *i2c_dev;
	i2c_dev = device_get_binding("myi2c");
	if (!i2c_dev) {
		printk("I2C: Device driver not found.\n");
		return;
	}
	
	error = init_Interrupt_BMP();
	error = initBMP384(i2c_dev);

	initMLX(i2c_dev);
	init_Interrupt_MLX();
	k_sleep(K_SECONDS(1));
	//enableMLX(i2c_dev);
	
	initBLE();
	printk("start imu\n");
	initIMU(i2c_dev,AFS_2G, GFS_15_125DPS, AODR_25Hz, GODR_25Hz);
	setState(0,0); //disable 0,0 enable 1,1
	k_sleep(K_SECONDS(3));
	//printk("start shtc\n");
	//initSHTC(i2c_dev);
}
