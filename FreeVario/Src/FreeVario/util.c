/*
 FreeVario http://FreeVario.org

 Copyright (c), PrimalCode (http://www.primalcode.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#include "util.h"






//only call me if gps is valid
void setRTCFromHgps(gps_t * hgps, RTC_HandleTypeDef * hrtc, int gmtoffset) {
	RTC_DateTypeDef  sdate;
	RTC_TimeTypeDef  stime;

	sdate.Year = hgps->year; //might have to convert to hex
	sdate.Month = hgps->month;
	sdate.Date = hgps->date;

	stime.Hours = hgps->hours + gmtoffset;
	stime.Minutes = hgps->minutes;
	stime.Seconds = hgps->seconds;

	HAL_RTC_SetDate(hrtc,&sdate,FORMAT_BCD);
	HAL_RTC_SetTime(hrtc,&stime,FORMAT_BCD);

}
