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
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <device.h>
#include "workQueue.h"
#include "bmpZephyr.h"
#include "ble.h"
#include "mlxZephyr.h"
#include "phyphox.h"
#include "icm42605.h"
#include "shtc3.h"
#include "led.h"

//#include "debug.h"

int8_t error =0;
void btn1_handle(struct k_work * work) {
    printk("test");
}

void main(void)
{
	printk("Hello World %s\n", CONFIG_BOARD);

	//LED 
	led_blue_dev = device_get_binding(LED0_GPIO);
    if (!led_blue_dev) {
        printk("BLUE-LED: Device driver not found.\n");
        return;
    }

	led_red_dev = device_get_binding(LED1_GPIO);
    if (!led_blue_dev) {
        printk("LED1: Device driver not found.\n");
        return;
    }

	gpio_pin_configure(led_blue_dev, LED0_PIN, GPIO_OUTPUT | LED0_FLAGS);
    gpio_pin_configure(led_red_dev, LED1_PIN, GPIO_OUTPUT | LED1_FLAGS);

	//turnoff_LEDs();
	gpio_pin_set(led_blue_dev,LED0_PIN,GPIO_OUT_PIN0_High);
    gpio_pin_set(led_red_dev,LED1_PIN,GPIO_OUT_PIN1_High);

	//Supercap
	init_BAS();

	static struct device *i2c_dev;
	i2c_dev = device_get_binding("myi2c");
	if (!i2c_dev) {
		printk("I2C: Device driver not found.\n");
		return;
	}
	
	error = initBMP384(i2c_dev);
	error = init_Interrupt_BMP();

	error = initSHTC(i2c_dev);
	init_Interrupt_SHTC();

	error = initMLX(i2c_dev);
	error = init_Interrupt_MLX();

	initBLE();

	blink_LED(LED_RED_ID,3);

	//printk("start imu\n");
	//initIMU(i2c_dev,AFS_2G, GFS_15_125DPS, AODR_25Hz, GODR_25Hz);
	//setState(0,0); //disable 0,0 enable 1,1
}

bool blink_LED(uint8_t ID, uint8_t count) {
    for (uint8_t i = 0; i < count*2; i++)
    {
        if (ID == LED_BLUE_ID)
        {
            gpio_pin_toggle(led_blue_dev, LED0_PIN);
        }
        if (ID == LED_RED_ID)
        { 
            gpio_pin_toggle(led_red_dev, LED1_PIN);
        }
        
        k_msleep(300);
    }
    return true;
}