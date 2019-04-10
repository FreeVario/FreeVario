/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#include "util.h"
#include "settings.h"




#ifndef USEGPSDATETIME
	extern RTC_HandleTypeDef  hrtc;
#endif

void setRTCFromHgps(gps_t * hgps) {
#ifndef USEGPSDATETIME
	RTC_DateTypeDef  sdate;
	RTC_TimeTypeDef  stime;



	sdate.Year = uint2bcd(hgps->year);
	sdate.Month = uint2bcd(hgps->month);
	sdate.Date = uint2bcd(hgps->date);
	RTC_CalcDOW(&sdate);




	stime.Hours = uint2bcd(hgps->hours + conf.gmtoffset);
	stime.Minutes = uint2bcd(hgps->minutes);
	stime.Seconds = uint2bcd(hgps->seconds);
	stime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	stime.TimeFormat = RTC_HOURFORMAT_24;
	stime.StoreOperation = RTC_STOREOPERATION_RESET;


	HAL_RTC_SetTime(&hrtc,&stime,FORMAT_BCD);
	HAL_RTC_SetDate(&hrtc,&sdate,FORMAT_BCD);



#endif
}



void setActivityTakeoffTime(ActivityData * activity) {

#ifdef USEGPSDATETIME
	activity->takeoffYear = hgps.year + 2000; //year 2100 problem anyone?
	activity->takeoffMonth = hgps.month;
	activity->takeoffDate = hgps.date;
	activity->takeoffHour = hgps.hours;
	activity->takeoffMinute = hgps.minutes;
	activity->takeoffSeconds = hgps.seconds;


#else
	RTC_DateTypeDef  sdate;
	RTC_TimeTypeDef  stime;

	HAL_RTC_GetTime(&hrtc, &stime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdate, RTC_FORMAT_BIN);

	activity->takeoffYear = bcd_decimal(sdate.Year) + 2000; //year 2100 problem anyone?
	activity->takeoffMonth = bcd_decimal(sdate.Month);
	activity->takeoffDate = bcd_decimal(sdate.Date);
	activity->takeoffHour = bcd_decimal(stime.Hours);
	activity->takeoffMinute = bcd_decimal(stime.Minutes);
	activity->takeoffSeconds = bcd_decimal(stime.Seconds);
#endif

}

void setActivityLandTime(ActivityData * activity) {
#ifdef USEGPSDATETIME
	activity->landingYear = hgps.year + 2000; //year 2100 problem anyone?
	activity->landingMonth = hgps.month;
	activity->landingDate = hgps.date;
	activity->landingHour = hgps.hours;
	activity->landingMinute = hgps.minutes;
	activity->landingSeconds = hgps.seconds;

#else
	RTC_DateTypeDef  sdate;
	RTC_TimeTypeDef  stime;

	HAL_RTC_GetTime(&hrtc, &stime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdate, RTC_FORMAT_BIN);

	activity->landingYear = bcd_decimal(sdate.Year + 2000); //year 2100 problem anyone?
	activity->landingMonth = bcd_decimal(sdate.Month);
	activity->landingDate = bcd_decimal(sdate.Date);
	activity->landingHour = bcd_decimal(stime.Hours);
	activity->landingMinute = bcd_decimal(stime.Minutes);
	activity->landingSeconds = bcd_decimal(stime.Seconds);

#endif
}

uint32_t uint2bcd(uint16_t dec)
{
    uint32_t result = 0;
    int shift = 0;

    while (dec)
    {
        result +=  (dec % 10) << shift;
        dec = dec / 10;
        shift += 4;
    }
    return result;
}

inline int bcd_decimal(uint8_t hex )
{

    return ((hex & 0xF0) >> 4) * 10 + (hex & 0x0F);

}

//char array, the value, divide by amount (1000), reduce by x after 0 (100 removes 2 0's), add -sign
void intTocharFloat(char *buffer, int value, uint16_t div, uint16_t dif, uint8_t sign){

	int fpart;
	int16_t bpart;

    char *tmpSign = (value < 0) ? "-" : " ";
    fpart = abs(value)/div;
    bpart = abs(value) % div;
    if (dif >0) bpart = bpart / dif;
if (sign) {
    if ((div/dif) >= 100) {
    	sprintf(buffer,"%s%d.%02d",tmpSign,fpart,bpart);
    }else{
    	sprintf(buffer,"%s%d.%1d",tmpSign,fpart,bpart);
    }
}else{
    if ((div/dif) >= 100) {
    	sprintf(buffer,"%d.%02d",fpart,bpart);
    }else{
    	sprintf(buffer,"%d.%1d",fpart,bpart);
    }
}

}

void RTC_CalcDOW(RTC_DateTypeDef *date) {
    int16_t adjustment,mm,yy;

    // Calculate intermediate values
    adjustment = (14 - date->Month) / 12;
    mm = date->Month + (12 * adjustment) - 2;
    yy = date->Year - adjustment;

    // Calculate day of week (0 = Sunday ... 6 = Saturday)
    date->WeekDay = (date->Date + ((13 * mm - 1) / 5) + yy + (yy / 4) - (yy / 100) + (yy / 400)) % 7;

    // Sunday?
    if (!date->WeekDay) date->WeekDay = 7;
}
