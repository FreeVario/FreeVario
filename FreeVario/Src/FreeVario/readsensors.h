/*
 * readsensors.h
 *
 *  Created on: Nov 18, 2018
 *      Author: marco
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



void setupReadSensorsBMP280(BMP280_HandleTypedef *bmp280);
void setupReadSensorsMPU6050(SD_MPU6050 *mpu1);
void readSensorsBMP280(BMP280_HandleTypedef *bmp280);
void readSensorsMPU6050(SD_MPU6050 *mpu1);
float getAltitudeFeet();
float getAltitudeMeters();
void calculateVario100ms();
void checkAdaptiveVario(int32_t vario, int8_t takeoff);


#endif /* READSENSORS_H_ */
