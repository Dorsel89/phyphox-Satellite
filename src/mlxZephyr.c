#include "mlxZephyr.h"


extern void initMLX(struct device *i2c_dev){
    	mlx_init(i2c_dev);
        mlxdev = i2c_dev;
}

extern void enableMLX(struct device *i2c_dev){
    startBurstMode(i2c_dev);
}


// GPIO DATA READY

static const struct gpio_dt_spec mlx_int1 = GPIO_DT_SPEC_GET_OR(MLX_INT1, gpios,{0});
static struct gpio_callback mlx_int1_cb_data;

static void mlx_int1_triggered(const struct device *dev, struct gpio_callback *cb,uint32_t pins)
{
	//printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
	k_work_submit(&work_mlx);

}
extern void printMLX(){
    float x,y,z;
    mlx_readMeasurement(&mlxData.x,&mlxData.y,&mlxData.z,mlxdev);
	if(PRINT_SENSOR_DATA){
    	//printk("MLX: x: %f y: %f z: %f \n",mlxData.x,mlxData.y,mlxData.z);
	}
}

int8_t init_Interrupt_MLX(){
    int8_t returnValue;

	k_work_init(&work_mlx, printMLX);

    if (!device_is_ready(mlx_int1.port)) {
		printk("Error: button device %s is not ready\n",
		       mlx_int1.port->name);
		return 1;
	}

	returnValue = gpio_pin_configure_dt(&mlx_int1, GPIO_INPUT);
	if (returnValue != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       returnValue, mlx_int1.port->name, mlx_int1.pin);
		return returnValue;
	}

	returnValue = gpio_pin_interrupt_configure_dt(&mlx_int1,GPIO_INT_EDGE_RISING);
	if (returnValue != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			returnValue, mlx_int1.port->name, mlx_int1.pin);
		return returnValue;
	}

	gpio_init_callback(&mlx_int1_cb_data, mlx_int1_triggered, BIT(mlx_int1.pin));
	gpio_add_callback(mlx_int1.port, &mlx_int1_cb_data);
	printk("Set up mlx_int1 at %s pin %d\n", mlx_int1.port->name, mlx_int1.pin);

    return returnValue;
}
