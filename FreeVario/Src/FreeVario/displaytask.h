/*
 * displaytask.h
 *
 *  Created on: Nov 18, 2018
 *      Author: marco
 */

#ifndef DISPLAYTASK_H_
#define DISPLAYTASK_H_
#ifdef __cplusplus
 extern "C" {
#endif

#define COLORED      0
#define UNCOLORED    1

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "epd2in9.h"
#include "epdif.h"
#include "epdpaint.h"
#include "gps.h"
#include "../fvconfig.h"
#include <stdlib.h>
#include <globaldata.h>

extern SensorData sensors;
extern gps_t  hgps;
extern ActivityData activity;
//TODO: Fix compiler warning

void StartDisplayTask(void const * argument);
void displayTaskSetup(Paint *paint, EPD *epd, unsigned char * frame_buffer);
void displayTaskUpdate( Paint *paint,EPD *epd, unsigned char * frame_buffer);
void displayMessageShutdown(Paint *paint,EPD *epd, unsigned char * frame_buffer);
void intTocharFloat(char *buffer, int value, uint16_t div, uint16_t dif);

#endif /* DISPLAYTASK_H_ */
