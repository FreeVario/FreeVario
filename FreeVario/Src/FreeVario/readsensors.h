/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#ifndef READSENSORS_H_
#define READSENSORS_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <math.h>
#include <bmp280.h>
#include <../fvconfig.h>
#include <sd_hal_mpu6050.h>



typedef struct {
    int16_t temperature;  // C x100
    uint32_t pressure;    //Pa x100
    uint32_t pressureraw;    //Pa x100
    int16_t humidity;     //% x100
    int16_t accel_x;   //x100
    int16_t accel_y;   //x100
    int16_t accel_z;   //x100
    int16_t gforce;   //x100
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int32_t AltitudeMeters; //x1000
    int32_t VarioMs; //x1000 Raw vario value from Baro
    int32_t VarioMsRaw; //x1000 Raw vario value from Baro
    uint8_t variosmooth; //Adaptive vario low pass filter value
    Queue_t QAltitudeMeters;
    uint8_t barotakeoff;
    uint32_t vbat; //bat voltage x10
    uint8_t pbat; //bat %charge

} SensorData; //owner: sensortask.c


extern SensorData sensors;



void setupVbatSensor();
void readVbatSensor();
void readInvMpuSensor();
void setupReadSensorsBMP280(BMP280_HandleTypedef *bmp280);
void setupReadSensorsMPU6050(SD_MPU6050 *mpu1);
void setupKalman();
void calcSensorsKalman(BMP280_HandleTypedef *bmp280, SD_MPU6050 *mpu1);
void readSensorsBMP280(BMP280_HandleTypedef *bmp280);
void readSensorsMPU6050(SD_MPU6050 *mpu1);
float getAltitudeFeet();
float getAltitudeMeters();
void calculateVario50ms();


unsigned short Row2Scale(const char *row);
unsigned short Matrix2Scalar(const char *mtx);
void NormalizeQuaternion(float *quat);

void checkAdaptiveVario(int32_t vario, int8_t takeoff);

#endif /* READSENSORS_H_ */
