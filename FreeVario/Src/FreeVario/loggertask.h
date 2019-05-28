/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#ifndef FREEVARIO_LOGGERTASK_H_
#define FREEVARIO_LOGGERTASK_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../fvconfig.h"

typedef struct {
    uint8_t isLogging;
    uint8_t gpxIsLogging;

} DataLog;

extern DataLog datalog;


void StartLoggerTask(void const * argument);

#endif /* FREEVARIO_LOGGERTASK_H_ */
