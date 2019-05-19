/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
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

extern gps_t hgps;
//TODO: Fix compiler warning

void StartDisplayTask(void const * argument);
void displayTaskSetup(Paint *paint, EPD *epd, unsigned char * frame_buffer);
void displayRefreshMainScreen(Paint *paint, EPD *epd,
        unsigned char * frame_buffer);
void displayDrawmainScreen(Paint *paint, EPD *epd, unsigned char * frame_buffer);
void displayTaskUpdate(Paint *paint, EPD *epd, unsigned char * frame_buffer);
void clearScreen(Paint *paint, EPD *epd,
        unsigned char * frame_buffer);
void displayMessageShutdown(Paint *paint, EPD *epd,
        unsigned char * frame_buffer);

#endif /* DISPLAYTASK_H_ */
