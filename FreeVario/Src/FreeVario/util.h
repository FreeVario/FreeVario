/*
 * util.h
 *
 *  Created on: Feb 9, 2019
 *      Author: marco
 */

#ifndef FREEVARIO_UTIL_H_
#define FREEVARIO_UTIL_H_
#include "gps.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

void setRTCFromHgps(gps_t * hgps, RTC_HandleTypeDef * hrtc, int gmtoffset);


#endif /* FREEVARIO_UTIL_H_ */
