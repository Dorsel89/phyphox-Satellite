#include "icm42605.h"

static const struct gpio_dt_spec imuInt = GPIO_DT_SPEC_GET_OR(IMU_INT, gpios,{0});
static struct gpio_callback imuInt_cb_data;

ICM icm_data;

static void sendDataIMU(){
  if(readData() && DEBUG){
    printk("ERROR: ICM42605 data read\n");
    return;
  }
  if(PRINT_SENSOR_DATA){
    	//printk("ICM_A: x: %f y: %f z: %f \n",ax,ay,az);
      //printk("ICM_G: x: %f y: %f z: %f \n",gx,gy,gz);
      printk("ICM_A: x: %f y: %f z: %f \n",-ay,az,-ax);
      printk("ICM_G: x: %f y: %f z: %f \n",-gy,gz,-gx);
	}

  float timestamp = k_uptime_get() /1000.0;
  icm_data.timestamp = timestamp;

  //this is strange, we should rework this part
  icm_data.ax = -ay*9.807;
  icm_data.ay = az*9.807;
  icm_data.az = -ax*9.807;

  icm_data.gx = -gy;
  icm_data.gy = -gz;
  icm_data.gz = -gx;

  icm_data.a_array[0+4*icm_data.measureSamples] = icm_data.ax;
	icm_data.a_array[1+4*icm_data.measureSamples] = icm_data.ay;
	icm_data.a_array[2+4*icm_data.measureSamples] = icm_data.az;
	icm_data.a_array[3+4*icm_data.measureSamples] = icm_data.timestamp;

  icm_data.g_array[0+4*icm_data.measureSamples] = icm_data.gx;
	icm_data.g_array[1+4*icm_data.measureSamples] = icm_data.gy;
	icm_data.g_array[2+4*icm_data.measureSamples] = icm_data.gz;
	icm_data.g_array[3+4*icm_data.measureSamples] = icm_data.timestamp;

  // calling send_data with a rate >=100Hz results in crash TODO
  icm_data.measureSamples+=1;
	if(icm_data.measureSamples==icm_data.samplesPerPackage){
    send_data(SENSOR_IMU_ACC_ID, &icm_data.a_array, 16*icm_data.samplesPerPackage);
    send_data(SENSOR_IMU_GYR_ID, &icm_data.g_array, 16*icm_data.samplesPerPackage);
    icm_data.measureSamples=0;
  }

  /*  this if statement should not be here?! 
  if(timestamp > oldTime +0.01){
    send_data(SENSOR_IMU_ACC_ID, &icm_data.a_array, 4*4);
    send_data(SENSOR_IMU_GYR_ID, &icm_data.g_array, 4*4);
    oldTime=timestamp;
  }
  */
}

static void set_config_icm(){
  if (DEBUG) {
    printk("ICM Setting config to..\n");
    printk("Enable: %d\n",icm_data.config[0]);
    printk("GSCALE: %d\n",icm_data.config[1]);
    printk("ASCALE: %d\n",icm_data.config[2]);
    printk("data rate: %d\n",icm_data.config[3]);
    printk("number of samples in package: %i\n",icm_data.config[4]);
  }
  if(0<icm_data.config[4]<=ICM420605_MAXSAMPLES){
    icm_data.samplesPerPackage = icm_data.config[4];
  }else{
    icm_data.samplesPerPackage = 1;
  }
	icm_data.measureSamples=0;
  sleep_icm(1);
  _aRes = getAres(icm_data.config[2]); 
  _gRes = getGres(icm_data.config[1]);
  changeSettings(icm_data.config[3],icm_data.config[1],icm_data.config[2]);
  k_sleep(K_MSEC(1));
  sleep_icm(!icm_data.config[0]);
}

static void imuDataReady(const struct device *dev, struct gpio_callback *cb,uint32_t pins)
{
	k_work_submit(&work_icm);
}

int8_t init_Interrupt_IMU(){
    int8_t returnValue;

	k_work_init(&work_icm, sendDataIMU);
	k_work_init(&config_icm, set_config_icm);

    if (!device_is_ready(imuInt.port)) {
		printk("Error: imu interrupt %s is not ready\n",
		       imuInt.port->name);
		return 1;
	}

	returnValue = gpio_pin_configure_dt(&imuInt, GPIO_INPUT);
	if (returnValue != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       returnValue, imuInt.port->name, imuInt.pin);
		return returnValue;
	}

	returnValue = gpio_pin_interrupt_configure_dt(&imuInt,GPIO_INT_EDGE_RISING);
	if (returnValue != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			returnValue, imuInt.port->name, imuInt.pin);
		return returnValue;
	}

	gpio_init_callback(&imuInt_cb_data, imuDataReady, BIT(imuInt.pin));
	gpio_add_callback(imuInt.port, &imuInt_cb_data);
	printk("Set up IMU at %s pin %d\n", imuInt.port->name, imuInt.pin);
  return returnValue;
}

extern void submit_config_icm(){
	k_work_submit(&config_icm);
};

//TODO make standard config
extern void init_icm(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR){

    icm_data.config[1] = AFS_2G;
    icm_data.config[2] = AODR_25Hz;
    icm_data.config[3] = GFS_15_125DPS;
    icm_data.config[4] = GODR_25Hz;

    reset();
    k_sleep(K_MSEC(100));
    uint8_t temp = readByte(ICM42605_ADDRESS, ICM42605_DRIVE_CONFIG);      
    temp ^= (-1 ^ temp) & (1UL << 0);
    temp ^= (-0 ^ temp) & (1UL << 1);
    temp ^= (-0 ^ temp) & (1UL << 2);
    temp ^= (-1 ^ temp) & (1UL << 3);
    temp ^= (-0 ^ temp) & (1UL << 4);
    temp ^= (-0 ^ temp) & (1UL << 5);
    writeByte(ICM42605_ADDRESS, ICM42605_DRIVE_CONFIG, temp);// set i2c slew rate to 20-60ns

    temp = readByte(ICM42605_ADDRESS, ICM42605_INTF_CONFIG6);      
    temp ^= (-1 ^ temp) & (1UL << 4);
    temp ^= (-0 ^ temp) & (1UL << 0);
    temp ^= (-0 ^ temp) & (1UL << 1);
    writeByte(ICM42605_ADDRESS, ICM42605_INTF_CONFIG6, temp);// 

    temp = readByte(ICM42605_ADDRESS, ICM42605_INTF_CONFIG4);      
    temp ^= (-0 ^ temp) & (1UL << 6);
    writeByte(ICM42605_ADDRESS, ICM42605_INTF_CONFIG4, temp);// 

    temp = readByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG0);
    temp = temp & ~(0xEF) ; // set all to 0 
    writeByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG0, temp | GODR | Gscale << 5); // gyro full scale and data rate

    temp = readByte(ICM42605_ADDRESS, ICM42605_ACCEL_CONFIG0);
    temp = temp & ~(0xEF) ; // set all to 0 
    writeByte(ICM42605_ADDRESS, ICM42605_ACCEL_CONFIG0, temp | AODR | Ascale << 5); // set accel full scale and data rate

    temp = readByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG1);
    writeByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG1, temp | 0xD0); // set temperature sensor low pass filter to 5Hz, use first order gyro filter

    k_sleep(K_MSEC(100));
    temp = readByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG);
    //temp = temp & ~(0x3F) ; // set all to 0 
    bool activeHigh = true;
    temp ^= (-activeHigh ^ temp) & (1UL << 0);  // bit 0
    bool pushPull = true;
    temp ^= (-pushPull ^ temp) & (1UL << 1);
    bool latchedMode = false;
    temp ^= (-latchedMode ^ temp) & (1UL << 2);
    writeByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG, temp);
    
    temp = readByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG1);
    writeByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG1, temp & ~(0x10) ); // set bit 4 to zero for proper function of INT1 and INT2

    temp = readByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG0);
//    myi2c->writeByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG0, temp | 0x20 );
    bool bit0 = 0;
    bool bit1 = 0;

    bool bit2 = 0;
    bool bit3 = 0;

    bool bit4 = 0;
    bool bit5 = 0;
 
    temp ^= (-bit0 ^ temp) & (1UL << 0);
    temp ^= (-bit1 ^ temp) & (1UL << 1);  
    temp ^= (-bit2 ^ temp) & (1UL << 2);  
    temp ^= (-bit3 ^ temp) & (1UL << 3);    
    temp ^= (-bit4 ^ temp) & (1UL << 4);  
    temp ^= (-bit5 ^ temp) & (1UL << 5);       
    writeByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG0, temp);
    temp = readByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE0);
    //temp = temp & ~(0x7F);
    temp ^= (-0 ^ temp) & (1UL << 0);
    temp ^= (-0 ^ temp) & (1UL << 1);  
    temp ^= (-0 ^ temp) & (1UL << 2);  
    temp ^= (-1 ^ temp) & (1UL << 3);    
    temp ^= (-0 ^ temp) & (1UL << 4);  
    temp ^= (-0 ^ temp) & (1UL << 5); 
    temp ^= (-0 ^ temp) & (1UL << 6);
    //myi2c->writeByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE0, temp | 0x08 ); // route data ready interrupt to INT1
    writeByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE0, temp ); // route data ready interrupt to INT1

    temp = readByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE3);
    writeByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE3, temp | 0x01 ); // route AGC interrupt interrupt to INT2

    // Select Bank 4
    temp = readByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL);
    writeByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL, temp | 0x04 ); // select Bank 4

    temp = readByte(ICM42605_ADDRESS, ICM42605_APEX_CONFIG5);
    writeByte(ICM42605_ADDRESS, ICM42605_APEX_CONFIG5, temp & ~(0x07) ); // select unitary mounting matrix

    temp = readByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL);
    writeByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL, temp & ~(0x07) ); // select Bank 0

    temp = readByte(ICM42605_ADDRESS, ICM42605_PWR_MGMT0); // make sure not to disturb reserved bit values
    //myi2c->writeByte(ICM42605_ADDRESS, ICM42605_PWR_MGMT0, temp | 0x0F);  // enable gyro and accel in low noise mode 0x0F
    writeByte(ICM42605_ADDRESS, ICM42605_PWR_MGMT0, temp | 0x00);  // disable gyro and accel
    getAres(Ascale);
    getGres(Gscale);

  k_sleep(K_MSEC(10));
  init_Interrupt_IMU();
}

static uint8_t changeSettings(uint8_t ODR, uint8_t Gscale, uint8_t Ascale){
  uint8_t errorCode=0;
  uint8_t temp;
  temp = readByte(ICM42605_ADDRESS, ICM42605_ACCEL_CONFIG0);
  temp = temp & ~(0xEF) ; // set all to 0 
  errorCode = writeByte(ICM42605_ADDRESS, ICM42605_ACCEL_CONFIG0, temp | ODR | Ascale << 5);
  
  if(errorCode){
    return errorCode;
  };

  temp = readByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG0);
  temp = temp & ~(0xEF);
  errorCode = writeByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG0, temp | ODR | Gscale << 5);
  return errorCode;
}

static uint8_t readByte(uint8_t i2cAddress, uint8_t subAddress){
  uint8_t read_data;
  uint8_t ret;
    
	//ret = i2c_write(icm_dev, &subAddress, 1, i2cAddress);
	//ret = i2c_read(icm_dev, &read_data, 1, i2cAddress);

  ret = i2c_reg_read_byte(icm_dev,i2cAddress,subAddress,&read_data);
  return ret;  
}

static uint8_t writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data){
    uint8_t dataBuffer[2];
	  dataBuffer[0]=regAddr;
    dataBuffer[1]=data;
    return i2c_write(icm_dev, &dataBuffer, 2, devAddr);
}

static uint8_t readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest){
    uint8_t ret;
    ret = i2c_burst_read(icm_dev,address,subAddress,dest,count);
    if(ret){
      printk("i2c_burst_read issue: %i \n",ret);
    }
    return ret;    
}

static uint8_t getChipID()
{
  return readByte(ICM42605_ADDRESS, ICM42605_WHO_AM_I);
}

static float getAres(uint8_t Ascale) {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      _aRes = 2.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_4G:
      _aRes = 4.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_8G:
      _aRes = 8.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_16G:
      _aRes = 16.0f / 32768.0f;
      return _aRes;
      break;
    default:
        return 0.0;
  }
}

static float getGres(uint8_t Gscale) {
  switch (Gscale)
  {
    case GFS_15_125DPS:
      _gRes = 15.125f / 32768.0f;
      return _gRes;
      break;
    case GFS_31_25DPS:
      _gRes = 31.25f / 32768.0f;
      return _gRes;
      break;
    case GFS_62_5DPS:
      _gRes = 62.5f / 32768.0f;
      return _gRes;
      break;
    case GFS_125DPS:
      _gRes = 125.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_250DPS:
      _gRes = 250.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_500DPS:
      _gRes = 500.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_1000DPS:
      _gRes = 1000.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_2000DPS:
      _gRes = 2000.0f / 32768.0f;
      return _gRes;
      break;
    default:
        return 0.0;
  }
}

static void reset()
{
  // reset device
  uint8_t temp = readByte(ICM42605_ADDRESS, ICM42605_DEVICE_CONFIG);
  writeByte(ICM42605_ADDRESS, ICM42605_DEVICE_CONFIG, temp | 0x01); // Set bit 0 to 1 to reset ICM42605
  k_sleep(K_MSEC(10));
}

static uint8_t status()
{
  uint8_t temp = readByte(ICM42605_ADDRESS, ICM42605_INT_STATUS);
  return temp;
}

uint8_t setState(bool acc, bool gyr){
    uint8_t temp = readByte(ICM42605_ADDRESS, ICM42605_PWR_MGMT0);
    temp ^= (-acc ^ temp) & (1UL << 0);
    temp ^= (-acc ^ temp) & (1UL << 1);
    temp ^= (-gyr ^ temp) & (1UL << 2);
    temp ^= (-gyr ^ temp) & (1UL << 3);
    writeByte(ICM42605_ADDRESS, ICM42605_PWR_MGMT0, temp);
    return 0;    
}

extern uint8_t sleep_icm(bool SLEEP) {
	if(SLEEP){
		return setState(false, false);
	}else{
		return setState(true, true);
	}
}

static int tickerInterval(uint8_t odr){
    int myInterval = 100;
    if(odr==AODR_1000Hz){
        myInterval = 1000/1000;
    }
    if(odr==AODR_200Hz){
        myInterval = 1000/200;
    }
    if(odr==AODR_100Hz){
        myInterval = 1000/100;
    }
    if(odr==AODR_50Hz){
        myInterval = 1000/50;
    }
    if(odr==AODR_25Hz){
        myInterval = 1000/25;
    }
    if(odr==AODR_12_5Hz){
        myInterval = 1000/12.5;
    }

    return myInterval;
}
static uint8_t readData()
{
    uint8_t rawData[14];  // x/y/z accel register data stored here
    uint8_t error=0;
    error = readBytes(ICM42605_ADDRESS, ICM42605_TEMP_DATA1, 14, &rawData[0]);  // Read the 14 raw data registers into data array
    if(error){
        return error;
    }
    uint8_t rawData_COPY[14];
    error = readBytes(ICM42605_ADDRESS, ICM42605_TEMP_DATA1, 14, &rawData_COPY[0]);
    if(error){
        return error;
    }
    for(int i=0;i<14;i++){
      if(rawData_COPY[i] != rawData[i]){
        if(DEBUG){
          printk("different result on second read");
        }
        return error;
      }
    }

    int16_t destination[7];

    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value

    //accX
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
    //accY
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
    //accZ
    destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
    //Gyr X
    destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
    //Gyr y
    destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
    //Gyr z
    destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
    t= destination[0];
    ax = destination[1] *_aRes;
    ay = destination[2] *_aRes;
    az = destination[3] *_aRes;

    gx = destination[4] *_gRes;
    gy = destination[5] *_gRes;
    gz = destination[6] *_gRes;
  /*
    if(rawData[2] == 0xFF && rawData[3] == 0xFF && rawData[4] == 0xFF && rawData[5] == 0xFF && rawData[6] == 0xFF && rawData[7] == 0xFF && rawData[8] == 0xFF && rawData[9] == 0xFF && rawData[10] == 0xFF && rawData[11] == 0xFF && rawData[12] == 0xFF && rawData[13] == 0xFF){
        error = 1;      
    }
    */

    return error;
}