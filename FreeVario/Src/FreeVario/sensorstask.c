/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#include "sensorstask.h"
#include <readsensors.h>
#include "nmea.h"
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "freevario.h"

uint8_t nmeasendbuffer[SENDBUFFER] __attribute__((section(".ccmram")));


extern QueueHandle_t uartQueueHandle;

void StartSensorsTask(void const * argument) {
    TickType_t readbattimer = 0;

    TickType_t times;
    TickType_t startTime = xTaskGetTickCount();
    const TickType_t xDelay = SENSORREADMS; //20hz
    uint8_t timetosend = 1;
    uint8_t rundelayd = 1;
    BMP280_HandleTypedef bmp280;
    SD_MPU6050 mpu1;
    memset(&sensors, 0, sizeof(sensors));
    sensors.variosmooth = conf.variosmooth;

    sensors.humidity = 0;
    sensors.pressure = 0;
    sensors.temperature = 0;
    sensors.barotakeoff = 0;
    setupVbatSensor();
    setupReadSensorsBMP280(&bmp280);

    setupReadSensorsMPU6050(&mpu1);

    setupKalman();

    /* Infinite loop */
    for (;;) {

        readbattimer++;
        times = xTaskGetTickCount();
        //TickType_t ties = xTaskGetTickCount() ;
        timetosend++;

        readSensorsBMP280(&bmp280); //using built in filtering
        readSensorsMPU6050(&mpu1);
        calculateVario50ms();
        calcSensorsKalman(&bmp280, &mpu1);

        if ((timetosend >= 4)
                & ((xTaskGetTickCount() - startTime) > STARTDELAY)) { //every 200 ticks
            timetosend = 1;
            checkAdaptiveVario(sensors.VarioMs, activity.flightstatus);
            memset(nmeasendbuffer, 0, SENDBUFFER);
            NMEA_getPTAS1(nmeasendbuffer, sensors.VarioMs, sensors.VarioMs,
                    sensors.AltitudeMeters);
            NMEA_getnmeaShortLXWP0(nmeasendbuffer, sensors.AltitudeMeters,
                    sensors.VarioMs);
            NMEA_getNmeaLK8EX1(nmeasendbuffer, sensors.pressure,
                    sensors.AltitudeMeters, sensors.VarioMs,
                    sensors.temperature, 999);
            NMEA_getNmeaPcProbe(nmeasendbuffer, sensors.accel_x,
                    sensors.accel_y, sensors.accel_z, sensors.temperature,
                    sensors.humidity);
            xQueueSendToBack(uartQueueHandle, nmeasendbuffer, 10);

        }

        if ((int) xTaskGetTickCount()
                > (STARTDELAY + 8000)&& activity.flightstatus == FLS_GROUND) {

            if (abs(sensors.VarioMs) > TAKEOFFVARIO) {
                sensors.barotakeoff = true;
            }

        }

        if ((int) xTaskGetTickCount()
                > (STARTDELAY + 8000)&& rundelayd) { //period between startup and start delay
            sensors.VarioMsRaw = 0;
            sensors.VarioMs = 0;
            rundelayd = 0;
        }


        if (sensors.barotakeoff && activity.flightstatus > FLS_GROUND) {
            sensors.barotakeoff = 0;
        }

        if (readbattimer >= 40) { //every 2000 ticks
            readbattimer = 0;
            readVbatSensor();

        }

        vTaskDelayUntil(&times, xDelay);

//		 char buff[6];
//		 sprintf(buff,"%d\r\n",(xTaskGetTickCount()-ties) );
//
//		 CDC_Transmit_FS(buff, 6);

    }

}
