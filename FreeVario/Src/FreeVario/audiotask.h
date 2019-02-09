/*
 * audiotask.h
 *
 *  Created on: Feb 1, 2019
 *      Author: marco
 */

#ifndef FREEVARIO_AUDIOTASK_H_
#define FREEVARIO_AUDIOTASK_H_
#ifdef __cplusplus
 extern "C" {
#endif
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../fvconfig.h"

 void StartAudioTask(void const * argument);


#endif /* FREEVARIO_AUDIOTASK_H_ */
