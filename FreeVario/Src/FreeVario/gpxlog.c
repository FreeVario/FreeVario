/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */
/*
 * gpx.c
 *
 *  Created on: May 26, 2019
 *      Author: PrimalCode
 */


#include <gpxlog.h>
#include <readsensors.h>
#include "freevario.h"
#include "gps.h"
#include "datalog.h"

extern gps_t hgps;

char  gpxheader[] = "﻿<?xml version=\"1.0\" encoding=\"utf-8\"?><gpx creator=\"FreeVario\" version=\"1.0\" xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\" xmlns=\"http://www.topografix.com/GPX/1/1\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" >\r\n <trk>\r\n<trkseg>\r\n";
char gpxtail[] = "</trkseg>\r\n</trk>\r\n</gpx>";



int openGPXLogFile(FIL* gpxFile) {
    char filename[32];
    uint32_t byteswritten = 0;

    sprintf(filename, "%02u-%02u-%u-%02u%02u-Log-%06ld.gpx",
            activity.takeoffYear, activity.takeoffMonth, activity.takeoffDate,
            activity.takeoffHour, activity.takeoffMinute,
            activity.currentLogID);

    if (f_open(gpxFile, filename,
    FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
        return 0;
    }

    f_write(gpxFile, gpxheader, strlen(gpxheader), (void *) &byteswritten);
    f_sync(gpxFile);

    return 1;


}

void writeGPXLogFile(FIL *gpxFile) {
    uint32_t byteswritten = 0;
    uint8_t gpxtrkpt[128];
    char gpxlon[11];
    char gpxlat[11];
    char alt[11];

    floa(gpxlon, hgps.longitude);
    floa(gpxlat, hgps.latitude);
    floa(alt, hgps.altitude);


    sprintf(gpxtrkpt, "<trkpt lat=\"%s\" lon=\"%s\">\r\n<ele>%s</ele>\r\n<time>20%u-%02u-%02uT%02u:%02u:%02uZ</time>\r\n</trkpt>\r\n",
            gpxlat,
            gpxlon,
            alt,
            hgps.year,
            hgps.month,
            hgps.date,
            hgps.hours,
            hgps.minutes,
            hgps.seconds);

    f_write(gpxFile, gpxtrkpt, strlen(gpxtrkpt), (void *) &byteswritten);
    f_sync(gpxFile);

}

void closeGPXLogFile(FIL *gpxFile) {
    uint32_t byteswritten = 0;
    f_write(gpxFile, gpxtail, strlen(gpxtail), (void *) &byteswritten);
    f_sync(gpxFile);

    f_close(gpxFile);
}





