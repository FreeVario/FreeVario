/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */


#include "loggertask.h"
#include "datalog.h"

DataLog datalog __attribute__((section(".ccmram")));

extern TaskHandle_t xLogDataNotify;
extern uint8_t SDcardMounted;
extern osMutexId sdCardMutexHandle;

void StartLoggerTask(void const * argument)
{
  /* USER CODE BEGIN StartLoggerTask */

	TickType_t times;
	const TickType_t xDelay = 1000;
	FIL  dataLogFile;
	uint32_t ulNotifiedValue;
	BaseType_t xResult;
	TickType_t xMaxBlockTime;
	configASSERT(xLogDataNotify == NULL);
	xLogDataNotify = xTaskGetCurrentTaskHandle();
	datalog.isLogging=0;
	TickType_t updateLogBooktime=0;
	osDelay(4000); //wait for setup of environment
	/* Infinite loop */
	for (;;) {


		times = xTaskGetTickCount();
		updateLogBooktime = xTaskGetTickCount();

		if (!SDcardMounted) { //can't continue without a SD card
			xLogDataNotify = NULL;
			vTaskSuspend( NULL);
		}

		xMaxBlockTime = pdMS_TO_TICKS(500);

		xResult = xTaskNotifyWait( pdFALSE, /* Don't clear bits on entry. */
		pdTRUE, /* Clear all bits on exit. */
		&ulNotifiedValue, /* Stores the notified value. */
		xMaxBlockTime);

		if (xResult == pdPASS) {
			/* A notification was received.  See which bits were set. */
			if (ulNotifiedValue == 1) { //started

				osDelay(200); //wait to stabilize data
				if ( xSemaphoreTake(sdCardMutexHandle,
						(TickType_t ) 600) == pdTRUE) {
					writeFlightLogSummaryFile();
					osDelay(100);
					uint8_t devnotready = 1;
					uint8_t timeout=0;
					//openDataLogFile(&dataLogFile);
					while (devnotready) {

						timeout++;
						if(openDataLogFile(&dataLogFile)) {
							datalog.isLogging = 1;
							devnotready = 0;
						}
						osDelay(2000);
						if (timeout > 2) devnotready = 0;
					}

					xSemaphoreGive(sdCardMutexHandle);
				}
			}

	//		if (ulNotifiedValue  == 2) { //flying (not used)

	//		}

			if (ulNotifiedValue == 3) { //landed
				if ( xSemaphoreTake(sdCardMutexHandle,
						(TickType_t ) 600) == pdTRUE) {
					if (datalog.isLogging) {

						datalog.isLogging = 0;
						closeDataLogFile(&dataLogFile);
					}
					writeFlightLogSummaryFile();
					xSemaphoreGive(sdCardMutexHandle);
				}
			}
		}

		if (datalog.isLogging) {

			if ( xSemaphoreTake(sdCardMutexHandle,
					(TickType_t ) 600) == pdTRUE) {
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				writeDataLogFile(&dataLogFile);
				if (xTaskGetTickCount() - updateLogBooktime > UPDATELOGFILETIME){ //update Summary log in case of program crash
					writeFlightLogSummaryFile();
				}

				xSemaphoreGive(sdCardMutexHandle);
			}
		}
		vTaskDelayUntil(&times, xDelay);
	}



  /* USER CODE END StartLoggerTask */
}
