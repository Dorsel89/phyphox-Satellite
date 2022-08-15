#include "mlxZephyr.h"


extern int8_t init_mlx(){
    bool err = mlx_init(mlx_dev);
	init_interrupt_mlx();
	sleep_mlx(true);
	return err;
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
    mlx_readData(&mlx_data.x,&mlx_data.y,&mlx_data.z,mlx_dev);
	if(PRINT_SENSOR_DATA){
    	printk("MLX: x: %f y: %f z: %f \n",mlx_data.x,mlx_data.y,mlx_data.z);
	}
	
	float timestamp = k_uptime_get() /1000.0;
	mlx_data.timestamp = timestamp;

	mlx_data.array[0] = mlx_data.x;
	mlx_data.array[1] = mlx_data.y;
	mlx_data.array[2] = mlx_data.z;
	mlx_data.array[3] = mlx_data.timestamp;

	send_data(SENSOR_MLX_ID, &mlx_data.array, sizeof(mlx_data.array));
}

int8_t init_interrupt_mlx(){
    int8_t returnValue;

	k_work_init(&work_mlx, sendDataMLX);
	k_work_init(&config_mlx, set_config_mlx);


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

extern uint8_t sleep_mlx(bool SLEEP) {
	if(SLEEP){
		return mlx_exitMode(mlx_dev);
	}else{
		return startBurstMode(mlx_dev);
	}
}

static void set_config_mlx(){
	if (DEBUG) {printk("MLX Setting config...\n");}

	if (!mlx_setGain(mlx_data.config[1], mlx_dev)) {
		printk("MLX error set Gain\n");
	}
	if (!mlx_setFilter(mlx_data.config[2], mlx_dev)) {
		printk("MLX error set Filter\n");
    }
	if (!mlx_setOversampling(mlx_data.config[3], mlx_dev)) {
		printk("MLX error set Oversampling\n");
    }
	/* Set resolution. */
	if (!mlx_setResolution(MLX90393_X, mlx_data.config[4], mlx_dev)){
		printk("MLX error set X Resolution\n");
	}
	if (!mlx_setResolution(MLX90393_Y, mlx_data.config[5], mlx_dev)){
		printk("MLX error set Y Resolution\n");
	}
	if (!mlx_setResolution(MLX90393_Z, mlx_data.config[6], mlx_dev)){
		printk("MLX error set Z Resolution\n");
	}

	sleep_mlx(!mlx_data.config[0]);
}

extern void submit_config_mlx(){
	k_work_submit(&config_mlx);
};

