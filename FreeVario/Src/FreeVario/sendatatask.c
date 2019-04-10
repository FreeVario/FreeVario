/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */



#include "sendatatask.h"
#include "usbd_cdc_if.h"
#include <stdlib.h>
#include <string.h>

extern TaskHandle_t xSendDataNotify;
extern UART_HandleTypeDef FV_UARTBT;
extern QueueHandle_t uartQueueHandle;

void StartSendDataTask(void const * argument)
{
  /* USER CODE BEGIN StartSendDataTask */

	//configASSERT(xSendDataNotify == NULL);
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(500);
	/* Infinite loop */
	for (;;) {
		uint8_t receiveqBuffer[SENDBUFFER];

		xQueueReceive(uartQueueHandle, &receiveqBuffer, portMAX_DELAY);
		uint16_t buffsize = strlen((char *) receiveqBuffer);
//TODO: move outside loop
		xSendDataNotify = xTaskGetCurrentTaskHandle();

		HAL_UART_Transmit_DMA(&FV_UARTBT, (uint8_t *) &receiveqBuffer, buffsize); //Usart global interupt must be enabled for this to work
		CDC_Transmit_FS((uint8_t *) &receiveqBuffer, buffsize);
		toggleDebugLED();
		ulTaskNotifyTake( pdTRUE, xMaxBlockTime);

		osDelay(1);
	}
  /* USER CODE END StartSendDataTask */
}
