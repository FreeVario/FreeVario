/*
 * sendatatask.h
 *
 *  Created on: Feb 1, 2019
 *      Author: marco
 */

#ifndef FREEVARIO_SENDATATASK_H_
#define FREEVARIO_SENDATATASK_H_
#ifdef __cplusplus
 extern "C" {
#endif
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../fvconfig.h"


 void StartSendDataTask(void const * argument);

#endif /* FREEVARIO_SENDATATASK_H_ */
