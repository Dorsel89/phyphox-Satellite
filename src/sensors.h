#ifndef SENSORS_H
#define SENSORS_H

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

#define DEBUG				true
#define PRINT_SENSOR_DATA 	false

#define SENSOR_BMP384_ID    1
#define SENSOR_IMU_ACC_ID   2
#define SENSOR_IMU_GYR_ID   3
#define SENSOR_SHTC_ID      4
#define SENSOR_MLX_ID       5

#define SENSOR_MPR_ID		6
#define SENSOR_THERMOCOUPLE_ID 7
#define SENSOR_DS18B20_ID 8


/**
 * @brief Helper function for converting struct sensor_value to float.
 *
 * @param val A pointer to a sensor_value struct.
 * @return The converted value.
 */

/*
static inline float sensor_value_to_float(const struct sensor_value *val) 
{
	return (float)val->val1 + (float)val->val2 / 1000000;
}
*/



#endif // SENSORS_H