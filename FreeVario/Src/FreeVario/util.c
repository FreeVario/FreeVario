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


void setActivityTakeoffTime(RTC_HandleTypeDef * hrtc, ActivityData * activity) {
	RTC_DateTypeDef  sdate;
	RTC_TimeTypeDef  stime;

	HAL_RTC_GetTime(&hrtc, &stime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdate, RTC_FORMAT_BIN);

	activity->takeoffYear = sdate.Year + 2000; //year 2100 problem anyone?
	activity->takeoffMonth = sdate.Month;
	activity->takeoffDate = sdate.Date;
	activity->takeoffHour = stime.Hours;
	activity->takeoffMinute = stime.Minutes;
	activity->takeoffSeconds = stime.Seconds;

}

void setActivityLandTime(RTC_HandleTypeDef * hrtc, ActivityData * activity) {
	RTC_DateTypeDef  sdate;
	RTC_TimeTypeDef  stime;

	HAL_RTC_GetTime(&hrtc, &stime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdate, RTC_FORMAT_BIN);

	activity->landingYear = sdate.Year + 2000; //year 2100 problem anyone?
	activity->landingMonth = sdate.Month;
	activity->landingDate = sdate.Date;
	activity->landingHour = stime.Hours;
	activity->landingMinute = stime.Minutes;
	activity->landingSeconds = stime.Seconds;


}
