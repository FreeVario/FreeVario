/*
 * freevario.h
 *
 *  Created on: Feb 1, 2019
 *      Author: marco
 */

#ifndef FREEVARIO_H_
#define FREEVARIO_H_
#ifdef __cplusplus
 extern "C" {
#endif
#include "../fvconfig.h"
 void uart_Idle_Handler_Callback(UART_HandleTypeDef *UartHandle);
 void StandbyMode(void);
 void freeVario_RTOS_Init();
 void StartDefaultTask(void const * argument);

#endif /* FREEVARIO_H_ */
