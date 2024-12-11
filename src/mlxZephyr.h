#ifndef _MLXZEPHYR_H
#define _MLXZEPHYR_H

#include "mlx90393.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include "ble.h"
#include "sensors.h"

#define MLX_NODE DT_ALIAS(i2c)
#if DT_NODE_HAS_STATUS(MLX_NODE, okay)
const static struct device *mlx_dev = DEVICE_DT_GET(MLX_NODE);
#else
#error "Node is disabled"
#endif

#define MLX_INT1 DT_NODELABEL(button2)
#define MLX90393_MAXSAMPLES 10
static struct k_work work_mlx;
static struct k_work config_mlx;

typedef struct {
	int samplesPerPackage;
	int measureSamples;
	float x;
	float y;
	float z;
	float timestamp;
    float array[4*10];//store up to 10 datapoints before transmission
	uint8_t config[20];
}MLX;

extern MLX mlx_data;

static struct sensor_value mlx_x,mlx_y,mlx_z;

extern int8_t init_mlx();
extern uint8_t sleep_mlx(bool SLEEP);
extern void submit_config_mlx();

static void set_config_mlx();
int8_t init_interrupt_mlx();
static void mlx_int1_triggered(const struct device *dev, struct gpio_callback *cb,uint32_t pins);

static bool firstSample =false;

#endif