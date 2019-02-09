/*
 * sensorstask.h
 *
 *  Created on: Feb 1, 2019
 *      Author: marco
 */

#ifndef FREEVARIO_SENSORSTASK_H_
#define FREEVARIO_SENSORSTASK_H_
#ifdef __cplusplus
 extern "C" {
#endif
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../fvconfig.h"


 void StartSensorsTask(void const * argument);

#endif /* FREEVARIO_SENSORSTASK_H_ */
