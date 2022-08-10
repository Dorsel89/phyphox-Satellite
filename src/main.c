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
#include <nrfx_saadc.h>

#include "waermelehre/ds18b20.h"
#include "waermelehre/ds18b20_sensor.h"

//#include "debug.h"
#define SLEEP_TIME_MS	1



#define I2C_DEV DT_LABEL(DT_NODELABEL(myi2c))

int8_t error =0;
void btn1_handle(struct k_work * work) {
    printk("test");
}

//ADC
nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN1, 0);

static void handle_error(nrfx_err_t error_code)
{
    if (error_code!= NRFX_SUCCESS)
    {
        // error
        while(1){};
    }
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

	//ADC
	/*
	nrfx_err_t err_code;
 
    nrf_saadc_value_t sample;
	
    err_code = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
    handle_error(err_code);
 
    err_code = nrfx_saadc_channels_config(&channel, 1);
    handle_error(err_code);
 
    err_code = nrfx_saadc_simple_mode_set((1<<0),
                                          NRF_SAADC_RESOLUTION_10BIT,
                                          NRF_SAADC_OVERSAMPLE_DISABLED,
                                          NULL);
    handle_error(err_code);
	err_code = nrfx_saadc_buffer_set(&sample, 1);
    handle_error(err_code);
	while (0)
    {
        err_code = nrfx_saadc_mode_trigger();
        handle_error(err_code);
        printk("sample %d", sample);
        NRFX_DELAY_US(1000000);
 
    }
	*/

//------------DS18B20---------------------

float fTemp1, fTemp2;
	int iTemp;

	ds18b20_setResolution(12);

	fTemp1 = ds18b20_get_temp();	// Method 1 (from sd18b20.c)
	k_sleep(K_SECONDS(1));

	//fTemp2 = ds18b20_get_temp_method_2();	// Method 2 (from sd18b20.c)
	//k_sleep(K_SECONDS(1));

	//iTemp = read_temperature();		// Metohd 3 (from sensor.c)
	
	//k_sleep(K_SECONDS(1));

	printk("Hello World! %s\nTemperature is :\n%d from method 1\n%d from method 2\n%d from method 3\n\n", CONFIG_BOARD, (int)fTemp1, (int)fTemp2, iTemp);

	while (1)
	{
		k_sleep(K_SECONDS(10));

		fTemp1 = ds18b20_get_temp();
		fTemp2 = ds18b20_get_temp();
		iTemp = read_temperature();

		printk("The temperature is now : \n%d from method 1\n%d from method 2\n%d from method 3\n\n", (int)fTemp1, (int)fTemp2, iTemp);
	}

}
