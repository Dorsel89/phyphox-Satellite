#ifndef SENSORS_H
#define SENSORS_H

#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>

#define DEBUG				true
#define PRINT_SENSOR_DATA 	true

#define SENSOR_BMP384_ID    1
#define SENSOR_IMU_ACC_ID   2
#define SENSOR_IMU_GYR_ID   3
#define SENSOR_SHTC_ID      4
#define SENSOR_MLX_ID       5
#define SENSOR_MPR_ID		6

/**
 * @brief Helper function for converting struct sensor_value to float.
 *
 * @param val A pointer to a sensor_value struct.
 * @return The converted value.
 */
static inline float sensor_value_to_float(const struct sensor_value *val) 
{
	return (float)val->val1 + (float)val->val2 / 1000000;
}


// NOT TESTED
/**
 * @brief Helper function for converting float to struct sensor_value.
 *
 * @param val A pointer to a sensor_value struct.
 * @param inp The converted value.
 * @return 0 if successful, negative errno code if failure.
 */
static inline int sensor_value_from_float(struct sensor_value *val, float inp)
{
	if (inp < INT32_MIN || inp > INT32_MAX) {
		return -ERANGE;
	}

	float val2 = (inp - (int16_t)inp) * 1000000.0;

	if (val2 < INT32_MIN || val2 > INT32_MAX) {
		return -ERANGE;
	}

	val->val1 = (int16_t)inp;
	val->val2 = (int16_t)val2;

	return 0;
}

#endif // SENSORS_H