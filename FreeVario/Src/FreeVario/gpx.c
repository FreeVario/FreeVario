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


#include "gpx.h"
#include <readsensors.h>

extern gps_t hgps;

char gpxheader = "GPXHEADER";
char gpxtail = "GPXTAIL";


void getGPXtrkpt(uint8_t * gpxtrkpt ) {
    char buffer[50];



    sprintf(buffer, "<trkpt lat=\"%f\" lon=\"%f\"><ele>%d</ele><time>%d-%d-%dT%d:%d:%dZ</time>");


}
