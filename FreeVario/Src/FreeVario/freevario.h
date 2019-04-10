/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */


#ifndef FREEVARIO_H_
#define FREEVARIO_H_
#ifdef __cplusplus
 extern "C" {
#endif
#include "../fvconfig.h"
 void uart_Idle_Handler_Callback(UART_HandleTypeDef *UartHandle);
 void StandbyMode(void);
 void toggleDebugLED();
 void freeVario_RTOS_Init();
 void StartDefaultTask(void const * argument);

#endif /* FREEVARIO_H_ */
