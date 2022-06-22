#ifndef _MLXZEPHYR_H
#define _MLXZEPHYR_H
#include "mlx90393.h"
#include "workQueue.h"
#include "zephyr.h"
#include "phyphox.h"
#include <drivers/i2c.h>
#include <drivers/gpio.h>

#include "ble.h"

#define MLX_INT1 DT_NODELABEL(button2)

static struct device *mlxdev;
static struct k_work work_mlx;

extern int8_t initMLX(struct device *i2c_dev);
extern int8_t init_Interrupt_MLX();
extern uint8_t sleepMLX(bool SLEEP);
static void setConfigMLX();
#endif