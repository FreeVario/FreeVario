/*
 * sendatatask.c
 *
 *  Created on: Feb 1, 2019
 *      Author: marco
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
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		ulTaskNotifyTake( pdTRUE, xMaxBlockTime);

		osDelay(1);
	}
  /* USER CODE END StartSendDataTask */
}
