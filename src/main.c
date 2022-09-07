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

//#include <nrfx_saadc.h>

//#include "waermelehre/ds18b20.h"
//#include "waermelehre/ds18b20_sensor.h"

#define SLEEP_TIME_MS	1

void main(void)
{
	printk("Hello World %s\n", CONFIG_BOARD);
	led_blink(red_led_dev,0,LED_ON_TIME_MS,LED_SLEEP_TIME_MS); 	// blink led until init done (led_off at the end)

	init_ble();

	init_shtc();
	init_mpr();
	init_bmp();
	init_mlx();
	init_icm(AFS_2G, GFS_15_125DPS, AODR_25Hz, GODR_25Hz);

	led_off(red_led_dev,0);

	ow_init();
	k_sleep(K_SECONDS(1));
	ds18b20_measureTemperature(1);
	k_sleep(K_SECONDS(3));
	float temperatur = ds18b20_getTemperature(1);
	printk("temperatur: %f",temperatur);
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
/*
float fTemp1, fTemp2;
	int iTemp;

	ds18b20_setResolution(12);

	fTemp1 = ds18b20_get_temp();	// Method 1 (from sd18b20.c)
	k_sleep(K_SECONDS(1));
*/
	//fTemp2 = ds18b20_get_temp_method_2();	// Method 2 (from sd18b20.c)
	//k_sleep(K_SECONDS(1));

	//iTemp = read_temperature();		// Metohd 3 (from sensor.c)
	
	//k_sleep(K_SECONDS(1));
/*
	printk("Hello World! %s\nTemperature is :\n%d from method 1\n%d from method 2\n%d from method 3\n\n", CONFIG_BOARD, (int)fTemp1, (int)fTemp2, iTemp);

	while (1)
	{
		k_sleep(K_SECONDS(10));

		fTemp1 = ds18b20_get_temp();
		fTemp2 = ds18b20_get_temp();
		iTemp = read_temperature();

		printk("The temperature is now : \n%d from method 1\n%d from method 2\n%d from method 3\n\n", (int)fTemp1, (int)fTemp2, iTemp);
	}
	*/
}
