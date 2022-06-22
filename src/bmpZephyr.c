#include "bmpZephyr.h"

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

extern void sendDataBMP(void){
    bmpResult = bmp3_get_sensor_data(sensor_comp, &myData, &bmp388_dev);
	if(PRINT_SENSOR_DATA){
		printk("BMP: pressure: %f temp: %f\n", myData.pressure,myData.temperature);
	}
	float timestamp = k_uptime_get() /1000.0;

	bmpData.pressure = myData.pressure / 100.0; //Pa to hPa
	bmpData.temperature = myData.temperature;
	bmpData.timestamp=timestamp;
	
	bmpData.array[0] = myData.pressure;
	bmpData.array[1] = myData.temperature;
	bmpData.array[2] = bmpData.timestamp;

	sendData(SENSOR_BMP384_ID, &bmpData.array, 4*3);
}

void initBMP384(struct device *i2c_dev){
	if (DEBUG_MODE){printk("BMP initialising...\n");}

	bmp388_dev.intf = BMP3_I2C_INTF;
	bmp388_dev.intf_ptr = i2c_dev;
	bmp388_dev.intf_rslt = bmpResult;
	bmp388_dev.dummy_byte = dByte;
	bmp388_dev.read = app_i2c_read;
	bmp388_dev.write = app_i2c_write;
	bmp388_dev.delay_us = app_us_delay;
	bmpResult = bmp3_init(&bmp388_dev);

	if(bmpResult != 0){
		printk("init error: %i \n",bmpResult);
	}
	else if (DEBUG_MODE)
	{
		printk("BMP init successful\n");
	}

	if (DEBUG_MODE){printk("BMP applying settings...\n");}

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
		printk("BMP apply settings error: %i \n",bmpResult);
	}
	else if (DEBUG_MODE){printk("BMP applied settings successfully\n");}


	if (bmpResult == BMP3_SENSOR_OK){
		if (DEBUG_MODE){printk("BMP setting mode...\n");}
        bmp388_dev.settings.op_mode = BMP3_MODE_NORMAL;
        bmpResult = bmp3_set_op_mode(&bmp388_dev);
        if (bmpResult == BMP3_SENSOR_OK)
        {
			if (DEBUG_MODE){printk("BMP mode set successfully\n");}
            bmp388_dev.delay_us(40000, bmp388_dev.intf_ptr);
            /* Sensor component selection */
            sensor_comp = BMP3_PRESS | BMP3_TEMP;
            /* Temperature and Pressure data are read and stored in the bmp3_data instance */
        }		
		else 
		{
			printk("BMP set mode error: %i \n", bmpResult);
		}
	}
	bmpResult = sleepBMP(true);
}

static const struct gpio_dt_spec bmpInt = GPIO_DT_SPEC_GET_OR(BMP_INT, gpios,{0});
static struct gpio_callback bmpInt_cb_data;

static void bmpDataReady(const struct device *dev, struct gpio_callback *cb,uint32_t pins)
{
	k_work_submit(&work_data);
}

int8_t init_Interrupt_BMP(){
    int8_t returnValue;

	k_work_init(&work_data, sendDataBMP);
	k_work_init(&work_config, setConfigBMP);

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

	if(DEBUG_MODE){printk("Set up BMP Interrupt button at %s pin %d\n", bmpInt.port->name, bmpInt.pin);}
	
    return returnValue;
}
extern void submitConfigBMP(){
	k_work_submit(&work_config);
};
extern uint8_t sleepBMP(bool SLEEP){
	if(SLEEP){
		bmp388_dev.settings.op_mode = BMP3_MODE_SLEEP;
		return bmp3_set_op_mode(&bmp388_dev);
	}else{
		bmp388_dev.settings.op_mode = BMP3_MODE_NORMAL;
		return bmp3_set_op_mode(&bmp388_dev);
	}
};
static void setConfigBMP(){
	if (DEBUG_MODE) {printk("BMP Setting config...\n");}
	uint8_t oversampling = bmpData.config[1];
	uint8_t filter = bmpData.config[2];
	uint8_t rate = bmpData.config[3];
	uint16_t settings_sel;

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
            bmp388_dev.delay_us(40000, bmp388_dev.intf_ptr);
            /* Sensor component selection */
            sensor_comp = BMP3_PRESS | BMP3_TEMP;
            /* Temperature and Pressure data are read and stored in the bmp3_data instance */
        }
		else {printk("BMP error setting settings\n");}
    }
	else {printk("BMP error setting settings\n");}

	if (DEBUG_MODE && bmpResult == BMP3_SENSOR_OK) {
		printk("BMP settings set to:\nEnable Bit: %i\nOversampling: %i\nFilter: %i\nDatarate: %i",bmpData.config[0],oversampling,filter,rate);
	}
	else {printk("BMP error setting settings\n");}

	k_sleep(K_MSEC(100));
	sleepBMP(!bmpData.config[0]);  
}

