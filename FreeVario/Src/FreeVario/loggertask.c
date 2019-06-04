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
#include "gpxlog.h"
#include "freevario.h"

DataLog datalog __attribute__((section(".ccmram")));

extern TaskHandle_t xLogDataNotify;
extern osMutexId sdCardMutexHandle;

void StartLoggerTask(void const * argument) {
    /* USER CODE BEGIN StartLoggerTask */

    FIL dataLogFile;
    FIL gpxLogFile;
    uint32_t ulNotifiedValue;
    BaseType_t xResult;
    TickType_t xMaxBlockTime;
    uint8_t twoMincounter = 0;
    configASSERT(xLogDataNotify == NULL);
    xLogDataNotify = xTaskGetCurrentTaskHandle();
    datalog.isLogging = 0;
    datalog.gpxIsLogging = 0;
    TickType_t updateLogBooktime = 0;
    osDelay(4000); //wait for setup of environment
    /* Infinite loop */
    for (;;) {

        twoMincounter++;
        updateLogBooktime = xTaskGetTickCount();

        if (!activity.SDcardMounted) {
            xLogDataNotify = NULL;
            vTaskSuspend( NULL);
        }



        xMaxBlockTime = pdMS_TO_TICKS(1000);

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

                    uint8_t devnotready = 1;
                    uint8_t timeout = 0;
                    while (devnotready) {

                        timeout++;
                        if (openDataLogFile(&dataLogFile)) { //datalog is the default logger
                            writeFlightLogSummaryFile();
                            datalog.isLogging = 1;
                            devnotready = 0;

                            if (openGPXLogFile(&gpxLogFile)) {
                                datalog.gpxIsLogging = 1;
                            }

                        } else{
                            osDelay(2000);
                        }


                        if (timeout > 4)
                            devnotready = 0;

                    }



                    xSemaphoreGive(sdCardMutexHandle);
                }
            }

            if (ulNotifiedValue  == 2) { //flying (used by gps tick)


      		}

            if (ulNotifiedValue == 3) { //landed
                if ( xSemaphoreTake(sdCardMutexHandle,
                        (TickType_t ) 600) == pdTRUE) {
                    if (datalog.isLogging) {
                        datalog.isLogging = 0;
                        closeDataLogFile(&dataLogFile);
                        writeFlightLogSummaryFile();
                    }

                    if (datalog.gpxIsLogging) {
                        closeGPXLogFile(&gpxLogFile);
                        datalog.gpxIsLogging = 0;
                    }

                    xSemaphoreGive(sdCardMutexHandle);
                }
            }
        }

        if (ulNotifiedValue == 4) { //Shutdown or reset signal, close all files and unmount
            if ( xSemaphoreTake(sdCardMutexHandle,
                    (TickType_t ) 600) == pdTRUE) {
                if (datalog.isLogging) {
                    closeDataLogFile(&dataLogFile);
                    writeFlightLogSummaryFile();
                }

                if (datalog.gpxIsLogging) {
                    closeGPXLogFile(&gpxLogFile);
                }
                xSemaphoreGive(sdCardMutexHandle);
            }

            f_mount(0, SDPath, 1); //unmount SDCARD

        }



        if (ulNotifiedValue  == 9) { //reserved gps sync signal
            if (twoMincounter >= 120) { //Bugfix: write to a file to keep SDcard alive
                //TODO: Fix this bug
                uint8_t stext[] = "kp\r\n";
                writeErrorlogFile(stext,4);
                twoMincounter = 0;

            }

        }

        if (datalog.isLogging) {

            if ( xSemaphoreTake(sdCardMutexHandle,
                    (TickType_t ) 600) == pdTRUE) {
                writeDataLogFile(&dataLogFile);

                if (datalog.gpxIsLogging) {
                    writeGPXLogFile(&gpxLogFile);
                }

                if (xTaskGetTickCount() - updateLogBooktime > UPDATELOGFILETIME) { //update Summary log in case of program crash
                    writeFlightLogSummaryFile();
                }

                xSemaphoreGive(sdCardMutexHandle);
            }
        }


    }

    /* USER CODE END StartLoggerTask */
}
