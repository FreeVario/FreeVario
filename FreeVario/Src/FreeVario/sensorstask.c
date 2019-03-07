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
#include <globaldata.h>
#include "nmea.h"
#include <stdlib.h>
#include <string.h>

uint8_t nmeasendbuffer[SENDBUFFER] __attribute__((section(".ccmram")));

extern SensorData sensors;
extern QueueHandle_t uartQueueHandle;


void StartSensorsTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorsTask */

	TickType_t times;
	TickType_t startTime = xTaskGetTickCount();
	const TickType_t xDelay = 50; //20hz
	uint8_t timetosend = 1;
	BMP280_HandleTypedef bmp280;
	SD_MPU6050 mpu1;
	memset(&sensors, 0, sizeof(sensors));


	sensors.humidity = 0;
	sensors.pressure = 0;
	sensors.temperature = 0;
	sensors.barotakeoff = 0;

	setupReadSensorsBMP280(&bmp280);
	setupReadSensorsMPU6050(&mpu1);
	osDelay(100);

	/* Infinite loop */
	for (;;) {
		times = xTaskGetTickCount();
		timetosend++;
		readSensorsBMP280(&bmp280);
	//	readSensorsMPU6050(&mpu1);

		if ((timetosend >= 2)) { //every 100 ticks
			calculateVario100ms();
			checkAdaptiveVario(sensors.VarioMs, sensors.barotakeoff); //TODO: change to activity.isflying
		}

		if ((timetosend >= 4)
				& ((xTaskGetTickCount() - startTime) > STARTDELAY)) { //every 200 ticks
			timetosend = 1;

			memset(nmeasendbuffer, 0, SENDBUFFER);
			NMEA_getPTAS1(nmeasendbuffer, sensors.VarioMs, sensors.VarioMs,
					sensors.AltitudeMeters);
			NMEA_getnmeaShortLXWP0(nmeasendbuffer, sensors.AltitudeMeters,
					sensors.VarioMs);
			NMEA_getNmeaLK8EX1(nmeasendbuffer, sensors.pressure, sensors.AltitudeMeters,
					sensors.VarioMs, sensors.temperature, 8000);
			NMEA_getNmeaPcProbe(nmeasendbuffer, sensors.accel_x, sensors.accel_y,
					sensors.accel_z, sensors.temperature, sensors.humidity, 100,
					0);
			xQueueSendToBack(uartQueueHandle, nmeasendbuffer, 10);

		}

#if defined(TAKEOFFVARIO) && !defined(TESTBUZZER)
		if ((int) xTaskGetTickCount() > (STARTDELAY + 4000)
				&& !sensors.barotakeoff) {
			if (abs(sensors.VarioMs) > TAKEOFFVARIO) {
				sensors.barotakeoff = true;

			}

		}
#else
		sensors.barotakeoff = true;
#endif

		vTaskDelayUntil(&times, xDelay);
	}
  /* USER CODE END StartSensorsTask */
}
