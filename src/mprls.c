#include "mprls.h"

MPR mpr_data = { // init values
    .timer_interval = 5
};

extern bool init_mpr() 
{   
    if(!device_is_ready(mpr_dev)){
        printk("Device not ready or not found");
        return false;
    }

    k_work_init(&work_mpr, send_data_mpr);
	k_work_init(&config_work_mpr, set_config_mpr);
    k_timer_init(&timer_mpr, mpr_data_ready, NULL);

    return true;
}

extern void sleep_mpr(bool sleep) 
{
    if (sleep) {
        k_timer_stop(&timer_mpr);
    }
    else{
        k_timer_start(&timer_mpr, K_MSEC(mpr_data.timer_interval*10), K_MSEC(mpr_data.timer_interval*10));
    }
    if(DEBUG){
        if(sleep){
            printk("MPRLS: sleep\n");
        }else{
            printk("MPRLS: enable\n");
        }
    }
}

void mpr_data_ready()
{
	k_work_submit(&work_mpr);
}

void send_data_mpr()
{
    sensor_sample_fetch(mpr_dev);
    sensor_channel_get(mpr_dev, SENSOR_CHAN_PRESS, &mpr_press);
    
    mpr_data.pressure = sensor_value_to_float(&mpr_press)*10.0;//get pressure in hPa

    float timestamp = k_uptime_get() /1000.0;
    mpr_data.timestamp = timestamp;

    mpr_data.array[0] = mpr_data.pressure;
    mpr_data.array[1] = mpr_data.timestamp;

    send_data(SENSOR_MPR_ID, &mpr_data.array, 4*2);
}

void set_config_mpr() 
{
    mpr_data.timer_interval = mpr_data.config[1];
    
    //Ensure minimum of 30ms
    if (mpr_data.timer_interval < 3) {mpr_data.timer_interval = 3;}
    if(DEBUG){
        printk("MPRLS: interval: %i \n",mpr_data.timer_interval);
    }
    sleep_mpr(!mpr_data.config[0]);
}

extern void submit_config_mpr()
{	
    k_work_submit(&config_work_mpr);
}