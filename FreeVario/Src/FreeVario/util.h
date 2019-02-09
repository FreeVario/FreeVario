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
#include <globaldata.h>

void setRTCFromHgps(gps_t * hgps, RTC_HandleTypeDef * hrtc, int gmtoffset);
void setActivityTakeoffTime(RTC_HandleTypeDef * hrtc, ActivityData * activity);
void setActivityLandTime(RTC_HandleTypeDef * hrtc, ActivityData * activity);

#endif /* FREEVARIO_UTIL_H_ */
