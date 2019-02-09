/*
 * loggertask.h
 *
 *  Created on: Feb 1, 2019
 *      Author: marco
 */

#ifndef FREEVARIO_LOGGERTASK_H_
#define FREEVARIO_LOGGERTASK_H_
#ifdef __cplusplus
 extern "C" {
#endif
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../fvconfig.h"

 void StartLoggerTask(void const * argument);


#endif /* FREEVARIO_LOGGERTASK_H_ */
