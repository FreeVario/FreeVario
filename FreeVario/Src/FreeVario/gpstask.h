/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#ifndef FREEVARIO_GPSTASK_H_
#define FREEVARIO_GPSTASK_H_
#ifdef __cplusplus
 extern "C" {
#endif
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../fvconfig.h"

 void StartGPSTask(void const * argument);


#endif /* FREEVARIO_GPSTASK_H_ */
