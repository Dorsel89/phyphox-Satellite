#include "bmpZephyr.h"

BMP bmp_data;

static BMP3_INTF_RET_TYPE app_i2c_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr) {
	uint8_t ret;
	ret = i2c_write(intf_ptr, &reg_addr, 1, BMP384_I2C_ADDR);
	ret = i2c_read(intf_ptr, read_data, len, BMP384_I2C_ADDR);
	return ret;
}

static BMP3_INTF_RET_TYPE app_i2c_write(uint8_t reg_addr, const uint8_t *write_data, uint32_t len, void *intf_ptr) {
	uint8_t dataBuffer[len+1];
	dataBuffer[0]=reg_addr;
	memcpy(&dataBuffer[0]+1,write_data,len);
	return i2c_write(intf_ptr, &dataBuffer, len+1, BMP384_I2C_ADDR);
}

static void app_us_delay(uint32_t period, void *intf_ptr) {
	return k_busy_wait(period);
}

extern void send_data_bmp(void){
    bmpResult = bmp3_get_sensor_data(sensor_comp, &myData, &bmp388_dev);
	if(PRINT_SENSOR_DATA){
		printk("BMP: pressure: %f temp: %f\n", myData.pressure,myData.temperature);
	}

	bmp_data.pressure = myData.pressure*0.01;
	bmp_data.temperature = myData.temperature;
	
	float timestamp = k_uptime_get() /1000.0;
	bmp_data.timestamp=timestamp;

	bmp_data.array[0] = bmp_data.pressure;
	bmp_data.array[1] = bmp_data.temperature;
	bmp_data.array[2] = bmp_data.timestamp;

	send_data(SENSOR_BMP384_ID, &bmp_data.array, 4*3);
}

static const struct gpio_dt_spec bmpInt = GPIO_DT_SPEC_GET_OR(BMP_INT, gpios,{0});
static struct gpio_callback bmpInt_cb_data;

static void bmpDataReady(const struct device *dev, struct gpio_callback *cb,uint32_t pins)
{
	k_work_submit(&work_bmp);
}

static void set_config_bmp(){
	if (DEBUG){printk("BMP: setting config BMP to:\n");};

	uint8_t oversampling = bmp_data.config[1];
	uint8_t filter = bmp_data.config[2];
	if(filter == 0){
		filter = 1;
	}
	uint8_t rate = bmp_data.config[3];
	uint16_t settings_sel;

	if (DEBUG){
		printk("Enable: %d\n",bmp_data.config[0]);
		printk("Oversampling: %d\n",oversampling);
		printk("Filter: %d\n",filter);
		printk("Rate: %d\n",rate);
	};
	
    bmp388_dev.settings.press_en = BMP3_ENABLE;
    bmp388_dev.settings.temp_en = BMP3_ENABLE;
    bmp388_dev.settings.odr_filter.press_os = oversampling;
    if(oversampling == BMP3_OVERSAMPLING_16X || oversampling == BMP3_OVERSAMPLING_32X){
        bmp388_dev.settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    }else {
        bmp388_dev.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    }
    
    bmp388_dev.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    bmp388_dev.settings.int_settings.drdy_en =BMP3_ENABLE;
    bmp388_dev.settings.odr_filter.iir_filter = filter;
    bmp388_dev.settings.odr_filter.odr =rate;//BMP3_ODR_25_HZ;
	settings_sel = BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_IIR_FILTER;
    bmpResult = bmp3_set_sensor_settings(settings_sel, &bmp388_dev);           
     if (bmpResult == BMP3_SENSOR_OK){
        bmp388_dev.settings.op_mode = BMP3_MODE_NORMAL; 
        bmpResult = bmp3_set_op_mode(&bmp388_dev);
        if (bmpResult == BMP3_SENSOR_OK)
        {			
			if (DEBUG){printk("BMP mode set successfully\n");}
            bmp388_dev.delay_us(40000, bmp388_dev.intf_ptr);
            /* Sensor component selection */
            sensor_comp = BMP3_PRESS | BMP3_TEMP;
            /* Temperature and Pressure data are read and stored in the bmp3_data instance */
        }
    }
	
	k_sleep(K_MSEC(100));
	sleep_bmp(!bmp_data.config[0]);
}

int8_t init_Interrupt_BMP(){
	if (DEBUG){printk("BMP: init interrupt\n");};

    int8_t returnValue;


    if (!device_is_ready(bmpInt.port)) {
		printk("Error: bmp interrupt %s is not ready\n",
		       bmpInt.port->name);
		return 1;
	}

	returnValue = gpio_pin_configure_dt(&bmpInt, GPIO_INPUT);
	if (returnValue != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       returnValue, bmpInt.port->name, bmpInt.pin);
		return returnValue;
	}

	returnValue = gpio_pin_interrupt_configure_dt(&bmpInt,GPIO_INT_EDGE_RISING);
	if (returnValue != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			returnValue, bmpInt.port->name, bmpInt.pin);
		return returnValue;
	}

	gpio_init_callback(&bmpInt_cb_data, bmpDataReady, BIT(bmpInt.pin));
	gpio_add_callback(bmpInt.port, &bmpInt_cb_data);

	if(DEBUG){printk("BMP: Set up BMP Interrupt button at %s pin %d\n", bmpInt.port->name, bmpInt.pin);}
	
    return returnValue;
}

extern bool init_bmp(){

	if (DEBUG){printk("BMP: init BMP\n");};
	bmp388_dev.intf = BMP3_I2C_INTF;
	bmp388_dev.intf_ptr = bmp_dev;
	bmp388_dev.intf_rslt = bmpResult;
	bmp388_dev.dummy_byte = dByte;
	bmp388_dev.read = app_i2c_read;
	bmp388_dev.write = app_i2c_write;
	bmp388_dev.delay_us = app_us_delay;
	bmpResult = bmp3_init(&bmp388_dev);

	if(bmpResult != 0){
		printk("init error: %i \n",bmpResult);
	}
	uint16_t settings_sel;
    bmp388_dev.settings.press_en = BMP3_ENABLE;
    bmp388_dev.settings.temp_en = BMP3_ENABLE;
    bmp388_dev.settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
    bmp388_dev.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    bmp388_dev.settings.int_settings.drdy_en =BMP3_ENABLE;
    bmp388_dev.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
    bmp388_dev.settings.int_settings.level = BMP3_INT_PIN_ACTIVE_HIGH;
    bmp388_dev.settings.odr_filter.odr = BMP3_ODR_1_5_HZ;
	settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |BMP3_SEL_DRDY_EN| BMP3_SEL_LEVEL |BMP3_SEL_IIR_FILTER;
	bmpResult = bmp3_set_sensor_settings(settings_sel, &bmp388_dev); 

	if(bmpResult != 0){
		printk("BMP: set settings error: %i \n",bmpResult);
	}          
	if (bmpResult == BMP3_SENSOR_OK){
        bmp388_dev.settings.op_mode = BMP3_MODE_NORMAL;
        bmpResult = bmp3_set_op_mode(&bmp388_dev);
        if (bmpResult == BMP3_SENSOR_OK)
        {
            bmp388_dev.delay_us(40000, bmp388_dev.intf_ptr);
            /* Sensor component selection */
            sensor_comp = BMP3_PRESS | BMP3_TEMP;
            /* Temperature and Pressure data are read and stored in the bmp3_data instance */
        }
	}

	bmpResult = sleep_bmp(true);
	
	k_work_init(&work_bmp, send_data_bmp);
	k_work_init(&config_work_bmp, set_config_bmp);

	init_Interrupt_BMP();
}

extern void submit_config_bmp(){
	k_work_submit(&config_work_bmp);
};

extern uint8_t sleep_bmp(bool SLEEP){
	if(SLEEP){
		if (DEBUG){printk("BMP: sleeping\n");};
		bmp388_dev.settings.op_mode = BMP3_MODE_SLEEP;
		return bmp3_set_op_mode(&bmp388_dev);
	}else{
		if (DEBUG){printk("BMP: awaking\n");};
		bmp388_dev.settings.op_mode = BMP3_MODE_NORMAL;
		return bmp3_set_op_mode(&bmp388_dev);
	}
};
