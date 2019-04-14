/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */


#include "readsensors.h"
#include "kalman.h"
#include "MadgwickAHRS.h"


extern I2C_HandleTypeDef FV_I2C;
extern SensorData sensors;
extern ADC_HandleTypeDef FV_HALADC;

#define MPU6050_ACCE_SENS_2         ((float) 16384)
#define MPU6050_ACCE_SENS_4         ((float) 8192)
#define MPU6050_ACCE_SENS_8         ((float) 4096)
#define MPU6050_ACCE_SENS_16        ((float) 2048)


#define QUAT_W 0
#define QUAT_X 1
#define QUAT_Y 2
#define QUAT_Z 3

#define VEC_X 0
#define VEC_Y 1
#define VEC_Z 2



char _orientation[9] = { 0, -1, 0,
                         1, 0, 0,
                         0, 0, -1 };

static uint8_t vTriggerd;
static uint32_t vtime=0;

void setupVbatSensor() {
	HAL_ADC_Start(&FV_HALADC);
	sensors.vbat=35;
	sensors.pbat=100;

}

void readVbatSensor() {

	if (HAL_ADC_PollForConversion(&FV_HALADC,100) == HAL_OK) {

			uint32_t cnv = HAL_ADC_GetValue(&FV_HALADC);

			/*TODO: in dire need of calibration, use your scope man*/
			sensors.vbat = (sensors.vbat * 100 + (((cnv * 2 * 4800) / 0xfff)/100))/101; //lowpassfilter

		  //calculate %charge, sort of
			sensors.pbat = (uint8_t)((37 - sensors.vbat) * 16.6);

			if (sensors.pbat > 100) { sensors.pbat = 100;}

	}

}

void setupReadSensorsBMP280(BMP280_HandleTypedef *bmp280) {
	bmp280_init_default_params(&bmp280->params);
	bmp280->addr = BMP280_I2C_ADDRESS_0;
	bmp280->i2c = &FV_I2C;

	bmp280->params.mode = BMP280_MODE_NORMAL;
	bmp280->params.filter = BMP280_FILTER_2;
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


	 SD_MPU6050_SetOrientation(&FV_I2C,mpu1,Matrix2Scalar(_orientation));
}


void setupKalman(){

    KalmanFilter_Configure(Z_VARIANCE, ZACCEL_VARIANCE, ZACCELBIAS_VARIANCE, 0,0.0f,0.0f);

}


//not yet used, needs work
void calcSensorsKalman(BMP280_HandleTypedef *bmp280, SD_MPU6050 *mpu1){
    float zTrack, vTrack;


    sensors.accel_x = mpu1->Accelerometer_X*100/MPU6050_ACCE_SENS_4;
    sensors.accel_y = mpu1->Accelerometer_Y*100/MPU6050_ACCE_SENS_4;
    sensors.accel_z = ( mpu1->Accelerometer_Z*100/MPU6050_ACCE_SENS_4) - 98; //upside down

    sensors.gyro_x = (ACCLSMOOTH * sensors.gyro_x +  mpu1->Gyroscope_X) / (ACCLSMOOTH + 1);
    sensors.gyro_y = (ACCLSMOOTH * sensors.gyro_y +  mpu1->Gyroscope_Y) / (ACCLSMOOTH + 1);
    sensors.gyro_z = (ACCLSMOOTH * sensors.gyro_z +  mpu1->Gyroscope_Z) / (ACCLSMOOTH + 1);

    MadgwickAHRSupdateIMU(sensors.gyro_x,sensors.gyro_y, sensors.gyro_z,sensors.accel_x,sensors.accel_y, sensors.accel_z);

    float accelv = (9.8f*imu_GravityCompensatedAccel(sensors.accel_x,sensors.accel_y, sensors.accel_z));

    sensors.gforce = accelv/100;

    if (activity.useKalman) {
        KalmanFilter_Update((float) sensors.VarioMsRaw/10, accelv / 10, (float) SENSORREADMS / 1000,
                &zTrack, &vTrack); // values must be cm/s
        sensors.VarioMs = vTrack * 10; //from cm/s to mm/s
    } else {
        sensors.VarioMs = sensors.VarioMsRaw;
    }


}

void readSensorsBMP280(BMP280_HandleTypedef *bmp280) {

    uint32_t pressure;
    int16_t temperature, humidity;

    while (!bmp280_read_int(bmp280, &temperature, &pressure, &humidity)) {

    }

    sensors.humidity = humidity;
    sensors.temperature = temperature;
    sensors.pressureraw = pressure;


    if (activity.useKalman) {
        sensors.pressure = pressure;
    } else {
        sensors.pressure = (conf.variosmooth * sensors.pressure + pressure)
                / (conf.variosmooth + 1);
        checkAdaptiveVario(sensors.VarioMs, activity.flightstatus);
    }

}

void readSensorsMPU6050(SD_MPU6050 *mpu1){

	SD_MPU6050_ReadAll(&FV_I2C,mpu1);




}

float getAltitudeFeet() { //x1000

	return getAltitudeMeters() * 145366.45;

}

float getAltitudeMeters() {

	return 44330.0f * (1.0f - pow(((float)sensors.pressure/100)  / (float)conf.qnePressure, 0.1902949f)); //because Pressure is in Pa x 100

}


//This must be called every 50ms (value x2)
void calculateVario50ms() {

    float alt = getAltitudeMeters();
    sensors.AltitudeMeters = (int32_t)( alt * 1000);
    SO_enqueue(&sensors.QAltitudeMeters, alt);

    if (SO_qisFull(&sensors.QAltitudeMeters)) {
    	//called per 50ms, so value *2
       sensors.VarioMsRaw = (int32_t)(( SO_rear(&sensors.QAltitudeMeters) -  SO_front(&sensors.QAltitudeMeters)) * 1000) * 2;

    }else{
    	sensors.VarioMsRaw = 0;
    }

}

unsigned short Row2Scale(const char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;

    return b;
}

unsigned short Matrix2Scalar(const char *mtx)
{
    unsigned short scalar;

    scalar = Row2Scale(mtx);
    scalar |= Row2Scale(mtx + 3) << 3;
    scalar |= Row2Scale(mtx + 6) << 6;

    return scalar;
}






void checkAdaptiveVario(int32_t vario, int8_t takeoff) { //sensors.VarioMs as parameter
#if defined(ADAPTIVEVARIO)

  int16_t triggerLevel = (conf.advTriggerLevel);

  if(takeoff==FLS_FLYING) { //compensate for glider sink
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

