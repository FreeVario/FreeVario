/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#include "datalog.h"
#include "fatfs.h"
#include <readsensors.h>
#include "freevario.h"
#include <stdlib.h>
#include <string.h>

extern gps_t hgps;


/**
 * @brief Rewrites the Flight log summary file
 * @param None
 * @retval None
 */
void writeFlightLogSummaryFile() {
    //Make sure the mutex is set
    //filename will be <lognumber>.log
    //and <lognumber>.igc

    FIL logSumFile;
    FRESULT res;
    uint32_t byteswritten;
    uint8_t wtext[128];
    char filename[32];

    sprintf(filename, "%02u-%02u-%u-%02u%02u-Log-%06ld.log",
            activity.takeoffYear, activity.takeoffMonth, activity.takeoffDate,
            activity.takeoffHour, activity.takeoffMinute,
            activity.currentLogID);

    if (f_open(&logSumFile, filename,
    FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        /* 'STM32.TXT' file Open for write Error */
        //Error_Handler();
    } else {
        /* Write data to the text file */

        sprintf(wtext,
                "%d,%u-%02u-%02u %02u:%02u:%02u,%d,%d,%u-%02u-%02u %02u:%02u:%02u,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                activity.currentLogID, activity.takeoffDate,
                activity.takeoffMonth, activity.takeoffYear,
                activity.takeoffHour, activity.takeoffMinute,
                activity.takeoffSeconds, activity.takeoffAltitude,
                activity.takeoffTemp, activity.landingDate,
                activity.landingMonth, activity.landingYear,
                activity.landingHour, activity.landingMinute,
                activity.landingSeconds, activity.landingAltitude,
                activity.MaxAltitudeMeters, activity.MaxAltitudeGainedMeters,
                activity.MaxVarioMs, activity.MaxVarioSinkMs,
                activity.takeoffLocationLAT, activity.takeoffLocationLON,
                activity.landingLocationLAT, activity.landingLocationLON);

        res = f_write(&logSumFile, wtext, strlen(wtext),
                (void *) &byteswritten);

        if ((byteswritten == 0) || (res != FR_OK)) {
            /* 'STM32.TXT' file Write or EOF Error */
            //Error_Handler();
        } else {
            /* Close the open text file */
            f_close(&logSumFile);

        }

    }

}

int openDataLogFile(FIL* logFile) {
    char filename[32];
    uint32_t byteswritten = 0;

    sprintf(filename, "%02u-%02u-%u-%02u%02u-Log-%06ld.csv",
            activity.takeoffYear, activity.takeoffMonth, activity.takeoffDate,
            activity.takeoffHour, activity.takeoffMinute,
            activity.currentLogID);

    if (f_open(logFile, filename,
    FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
        return 0;
    }

    uint8_t header[] =
            "Date,GPS Fix, GPS Valid,Latitude,Logitude, GPS Altitude,Coarse,Speed,Sats in use, Baro Altitude, Vario,"
                    "Accel X,Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z, G-Force ,Temperature, Humidity, Pressure, Pressure Raw, vBat, Baro GNSS Valid, Baro GNSS dif, Vario Smooth, Kelman \r\n";
    f_write(logFile, header, strlen(header), (void *) &byteswritten);
    f_sync(logFile);
    return 1;

}

void writeDataLogFile(FIL *logFile) {
    uint32_t byteswritten = 0;
    uint8_t mtext[256];
    char filename[32];
//	FIL  logFile;

    sprintf(filename, "%06d.csv", activity.currentLogID);
//	if (f_open(&logFile, filename,
//			FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
//		return;
//	}

    sprintf(mtext,
            "%u-%02u-%02u %02u:%02u:%02u,%u,%u,%ld,%ld,%ld,%ld,%ld,%u,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%u,%lu,%lu,%u,%u,%ld,%ld,%d\r\n",
            hgps.date, hgps.month, hgps.year, hgps.hours, hgps.minutes,
            hgps.seconds, hgps.fix, hgps.is_valid,
            (int32_t) (hgps.latitude * 1000000),
            (int32_t) (hgps.longitude * 1000000),
            (int32_t) (hgps.altitude * 1000), (int32_t) (hgps.coarse * 1000),
            (int32_t) (hgps.speed * 1000),
            hgps.sats_in_use, sensors.AltitudeMeters, sensors.VarioMs,
            sensors.accel_x, sensors.accel_y, sensors.accel_z, sensors.gyro_x,
            sensors.gyro_y, sensors.gyro_z, sensors.gforce, sensors.temperature,
            sensors.humidity, sensors.pressure, sensors.pressureraw, sensors.vbat,
            activity.barognssavalid, activity.barognssdeveation,
            (int32_t) conf.variosmooth, activity.useKalman);

    f_write(logFile, mtext, strlen(mtext), (void *) &byteswritten);
    f_sync(logFile);
//	f_close(&logFile);
}

void closeDataLogFile(FIL *logFile) {
    f_close(logFile);
}

//FRESULT set_timestamp(char * obj) {
//	FILINFO fno;
//
//	int hour = hgps.hours;
//	int min = hgps.minutes;
//	int sec = hgps.seconds;
//	int month = hgps.month;
//	int mday = hgps.date;
//	int year = hgps.year;
//	year += 2000;
//	fno.fdate = (WORD) (((year - 1980) << 9) | month << 5 | mday);
//	fno.ftime = (WORD) (hour << 11 | min << 5 | sec / 2);
//
//	return f_utime(obj, &fno);
//}
