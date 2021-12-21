#ifndef _WORKQUEUE_H
#define _WORKQUEUE_H

#include <stdint.h>


//static struct k_work some_work;
typedef struct {
	float x;
	float y;
	float z;
	float timestamp;
}MLX;

typedef struct {
	float pressure;
	float temperature;
	float timestamp;
	uint8_t config[20];
}BMP;

typedef struct {
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
	float timestamp;
	uint8_t config[20];
}IMU;

extern BMP bmpData;
extern MLX mlxData;
extern IMU imuData;

#endif