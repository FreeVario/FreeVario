/*
 FreeVario http://FreeVario.org

 Copyright (c), PrimalCode (http://www.primalcode.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */


#include "readsensors.h"



extern I2C_HandleTypeDef hi2c1;
extern SensorData sensors;

static uint8_t vTriggerd;
static uint32_t vtime=0;

void setupReadSensorsBMP280(BMP280_HandleTypedef *bmp280) {
	bmp280_init_default_params(&bmp280->params);
	bmp280->addr = BMP280_I2C_ADDRESS_0;
	bmp280->i2c = &hi2c1;

	bmp280->params.mode = BMP280_MODE_NORMAL;
	bmp280->params.filter = BMP280_FILTER_16;
	bmp280->params.oversampling_pressure = BMP280_ULTRA_HIGH_RES;
	bmp280->params.oversampling_temperature = BMP280_STANDARD;
	bmp280->params.oversampling_humidity = BMP280_STANDARD;
	bmp280->params.standby = BMP280_STANDBY_05;

	while (!bmp280_init(bmp280, &bmp280->params)) {
	}

	SO_setQueue(&sensors.QAltitudeMeters, 10); //vario m/s


}



void setupReadSensorsMPU6050(SD_MPU6050 *mpu1) {
	 SD_MPU6050_Init(&FV_I2C,mpu1,FV_ACCL_ADR,SD_MPU6050_Accelerometer_4G,SD_MPU6050_Gyroscope_250s );
}


void readSensorsBMP280(BMP280_HandleTypedef *bmp280){

	uint32_t pressure;
	int16_t temperature, humidity;
	while (!bmp280_read_int(bmp280, &temperature, &pressure, &humidity)) {

		}

	sensors.humidity = humidity;
	sensors.temperature = temperature;
	sensors.pressureraw = pressure;

	//low pass filter
	sensors.pressure = (conf.variosmooth * sensors.pressure + pressure) / (conf.variosmooth + 1);

}

void readSensorsMPU6050(SD_MPU6050 *mpu1){

	SD_MPU6050_ReadAll(&FV_I2C,mpu1);
	//we use the full scale so the devider is 2048

	sensors.accel_x = (ACCLSMOOTH * sensors.accel_x +  mpu1->Accelerometer_X*1000/8192) / (ACCLSMOOTH + 1);
	sensors.accel_y = (ACCLSMOOTH * sensors.accel_y +  mpu1->Accelerometer_Y*1000/8192) / (ACCLSMOOTH + 1);
	sensors.accel_z = (ACCLSMOOTH * sensors.accel_z +  mpu1->Accelerometer_Z*1000/8192) / (ACCLSMOOTH + 1);

	sensors.gyro_x = (ACCLSMOOTH * sensors.gyro_x +  mpu1->Gyroscope_X) / (ACCLSMOOTH + 1);
	sensors.gyro_y = (ACCLSMOOTH * sensors.gyro_y +  mpu1->Gyroscope_Y) / (ACCLSMOOTH + 1);
	sensors.gyro_z = (ACCLSMOOTH * sensors.gyro_z +  mpu1->Gyroscope_Z) / (ACCLSMOOTH + 1);

}

float getAltitudeFeet() { //x1000

	return getAltitudeMeters() * 145366.45;

}

float getAltitudeMeters() {

	return 44330.0f * (1.0f - pow(((float)sensors.pressure/100)  / (float)conf.qnePressure, 0.1902949f)); //because Pressure is in Pa x 100

}

//This must be called at 100ms
void calculateVario100ms() {

    float alt = getAltitudeMeters();
    sensors.AltitudeMeters = (int32_t)( alt * 1000);
    SO_enqueue(&sensors.QAltitudeMeters, alt);

    if (SO_qisFull(&sensors.QAltitudeMeters)) {
        sensors.VarioMs = (int32_t)(( SO_rear(&sensors.QAltitudeMeters) -  SO_front(&sensors.QAltitudeMeters)) * 1000);
    }else{
    	sensors.VarioMs = 0;
    }


}

void checkAdaptiveVario(int32_t vario, int8_t takeoff) { //sensors.VarioMs as parameter
#if defined(ADAPTIVEVARIO)

  int16_t triggerLevel = (conf.advTriggerLevel);

  if(takeoff) { //compensate for glider sink
    vario += -(conf.gliderSinkRate);
  }


  if (fabs(vario) > triggerLevel && !vTriggerd) { //fabs abs but can handle floats
	vtime = xTaskGetTickCount();
    vTriggerd = true;
  }

  int diff = xTaskGetTickCount() - vtime;

  //Vario level goes back to zero within TriggerTime, increase the filter
  if (vTriggerd && fabs(vario) < triggerLevel  && diff < (int)(conf.advTriggerTime))  {
    if (conf.variosmooth <= conf.advMaxSmooth ) {
      conf.variosmooth++;
      vTriggerd = false;
      vtime = xTaskGetTickCount();

    }
  }

  //overflow reset
  if (diff > conf.advTriggerTime && vTriggerd) {
    vTriggerd = false;
    vtime = xTaskGetTickCount();;
  }
  //Vario stays below trigger level for advRelaxTime, decrease the filter
  if (fabs(vario) < (float)(triggerLevel) && !vTriggerd && diff >  conf.advRelaxTime) {
    if (conf.variosmooth > conf.advMinSmooth ) {
      conf.variosmooth--;
      vtime = xTaskGetTickCount();
    }
  }
#endif
}

