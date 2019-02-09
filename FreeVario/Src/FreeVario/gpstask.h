/*
 * gpstask.h
 *
 *  Created on: Feb 1, 2019
 *      Author: marco
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
