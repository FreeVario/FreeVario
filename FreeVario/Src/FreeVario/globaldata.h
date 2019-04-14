/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#ifndef GLOBALDATA_H_
#define GLOBALDATA_H_
#ifdef __cplusplus
 extern "C" {
#endif
#include "../fvconfig.h"

#define FLS_GROUND 		0
#define FLS_TAKEOFF 	1
#define FLS_FLYING 		2
#define FLS_LANDED 		3

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
	Queue_t  QAltitudeMeters;
	uint8_t barotakeoff;
	uint16_t vbat; //bat voltage x10
	uint8_t pbat; //bat %charge
}SensorData; //owner: sensortask.c

extern settings_t conf; //declared at fvconf.h. So yes, you must include both files

typedef struct {
    uint16_t takeoffYear;
    uint8_t takeoffMonth;
    uint8_t takeoffDate;
    uint8_t takeoffHour;
    uint8_t takeoffMinute;
    uint8_t takeoffSeconds;
    uint32_t takeoffTemp;
    uint32_t takeoffAltitude;
    uint32_t takeoffLocationLAT;
    uint32_t takeoffLocationLON;
    uint16_t landingYear;
    uint8_t landingMonth;
    uint8_t landingDate;
    uint8_t landingHour;
    uint8_t landingMinute;
    uint8_t landingSeconds;
    uint32_t landingLocationLAT;
    uint32_t landingLocationLON;
    uint32_t landingAltitude;
    uint8_t flightstatus;
    int32_t MaxAltitudeMeters;
    int32_t MaxAltitudeGainedMeters;
    int32_t MaxVarioMs;
    int32_t MaxVarioSinkMs;
    int32_t currentLogID;
    uint8_t muted;
    int32_t barognssdeveation;  //difference between baro and gnss (baro - gnss)
    uint8_t barognssavalid;    //flag if the adjusted value is usable
    uint8_t useKalman; //use kalman with accelerometer insead of a lowpass filter
}ActivityData; //owner: freevario.c

extern ActivityData activity;

typedef struct {
	uint8_t isLogging;
}DataLog;

extern DataLog datalog;


#endif /* GLOBALDATA_H_ */
