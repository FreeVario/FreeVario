/*
 * util.c
 *
 *  Created on: Feb 9, 2019
 *      Author: marco
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
