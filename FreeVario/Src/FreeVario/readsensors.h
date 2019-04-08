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
#include <globaldata.h>
#include <../fvconfig.h>
#include <sd_hal_mpu6050.h>


 void setupVbatSensor();
 void readVbatSensor();
void setupReadSensorsBMP280(BMP280_HandleTypedef *bmp280);
void setupReadSensorsMPU6050(SD_MPU6050 *mpu1);
void readSensorsBMP280(BMP280_HandleTypedef *bmp280);
void readSensorsMPU6050(SD_MPU6050 *mpu1);
float getAltitudeFeet();
float getAltitudeMeters();
void calculateVario50ms();
void checkAdaptiveVario(int32_t vario, int8_t takeoff);


#endif /* READSENSORS_H_ */
