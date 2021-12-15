#ifndef _MLXZEPHYR_H
#define _MLXZEPHYR_H
#include "mlx90393.h"
#include "workQueue.h"
#include <drivers/gpio.h>
#define MLX_INT1 DT_NODELABEL(button2)

static struct device *mlxdev;
static struct k_work work_mlx;

extern void initMLX(struct device *i2c_dev);
extern void enableMLX(struct device *i2c_dev);
#endif