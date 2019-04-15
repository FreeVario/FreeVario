/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#ifndef FREEVARIO_UTIL_H_
#define FREEVARIO_UTIL_H_
#include "../fvconfig.h"
#include "gps.h"
#include "gpstask.h"
#include <globaldata.h>

void setRTCFromHgps(gps_t * hgps);
void setActivityTakeoffTime(ActivityData * activity);
void setActivityLandTime(ActivityData * activity);
uint32_t uint2bcd(uint16_t dec);
void intTocharFloat(char *buffer, int value, uint16_t div, uint16_t dif,
        uint8_t sign);
void RTC_CalcDOW(RTC_DateTypeDef *date);
#endif /* FREEVARIO_UTIL_H_ */
