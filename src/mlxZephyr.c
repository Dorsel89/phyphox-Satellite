#include "mlxZephyr.h"


extern int8_t initMLX(struct device *i2c_dev){
	    mlxdev = i2c_dev;
    	return mlx_init(mlxdev);
}

extern int8_t mlx_enable(struct device *i2c_dev){
    return startBurstMode(i2c_dev);
}


// GPIO DATA READY

static const struct gpio_dt_spec mlx_int1 = GPIO_DT_SPEC_GET_OR(MLX_INT1, gpios,{0});
static struct gpio_callback mlx_int1_cb_data;

static void mlx_int1_triggered(const struct device *dev, struct gpio_callback *cb,uint32_t pins)
{
	//printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
	k_work_submit(&work_mlx);

}
extern void sendDataMLX(){
    mlx_readData(&mlxData.x,&mlxData.y,&mlxData.z,mlxdev);
	if(PRINT_SENSOR_DATA){
    	printk("MLX: x: %f y: %f z: %f \n",mlxData.x,mlxData.y,mlxData.z);
	}
	
	float timestamp = k_uptime_get() /1000.0;

	mlxData.array[0] = mlxData.x;
	mlxData.array[1] = mlxData.y;
	mlxData.array[2] = mlxData.z;
	mlxData.array[3] = mlxData.timestamp;

	sendData(SENSOR_MLX_ID, &mlxData.array, sizeof(mlxData.array));
}

int8_t init_Interrupt_MLX(){
    int8_t returnValue;

	k_work_init(&work_mlx, sendDataMLX);

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

extern uint8_t sleepMLX(bool SLEEP) {
	if(SLEEP){
		return mlx_exitMode(mlxdev);
	}else{
		return mlx_enable(mlxdev);
	}
}

static void setConfigMLX(){
	if (DEBUG_MODE) {printk("MLX Setting config...\n");}

	if (!mlx_setGain(mlxData.config[1], mlxdev)) {
		printk("MLX error set Gain\n");
	}
	if (!mlx_setFilter(mlxData.config[2], mlxdev)) {
		printk("MLX error set Filter\n");
    }
	if (!mlx_setOversampling(mlxData.config[3], mlxdev)) {
		printk("MLX error set Oversampling\n");
    }
	/* Set resolution. */
	if (!mlx_setResolution(MLX90393_X, mlxData.config[4], mlxdev)){
		printk("MLX error set X Resolution\n");
	}
	if (!mlx_setResolution(MLX90393_Y, mlxData.config[5], mlxdev)){
		printk("MLX error set Y Resolution\n");
	}
	if (!mlx_setResolution(MLX90393_Z, mlxData.config[6], mlxdev)){
		printk("MLX error set Z Resolution\n");
	}

	sleepMLX(!mlxData.config[0]);
}


