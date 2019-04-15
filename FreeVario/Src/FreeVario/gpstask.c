/*
 FreeVario http://FreeVario.org
 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#include "gpstask.h"
#include <stdlib.h>
#include <string.h>

gps_t hgps __attribute__((section(".ccmram")));

extern TaskHandle_t xReceiveNotify;
extern UART_HandleTypeDef FV_UARTGPS;
extern QueueHandle_t uartQueueHandle;

void StartGPSTask(void const * argument) {

    uint8_t sendBuffer[SENDBUFFER];
    uint8_t flag = 0;
    uint16_t y = 0;
    size_t len;

    gps_init(&hgps);
    configASSERT(xReceiveNotify == NULL);
    __HAL_UART_ENABLE_IT(&FV_UARTGPS, UART_IT_IDLE);
    uint8_t buffer[GPSRXBUFFER];
    /* Infinite loop */
    for (;;) {
        memset(buffer, 0, GPSRXBUFFER);
        __HAL_UART_CLEAR_IDLEFLAG(&FV_UARTGPS);
        __HAL_UART_ENABLE_IT(&FV_UARTGPS, UART_IT_IDLE);

        if (HAL_UART_Receive_DMA(&FV_UARTGPS, buffer, GPSRXBUFFER) != HAL_OK) {
            // error
        }
        xReceiveNotify = xTaskGetCurrentTaskHandle();
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
        HAL_UART_DMAStop(&FV_UARTGPS);

        /*
         * The sendbuffer was getting to big with the new GNSS modules
         * Something more than 700 Bytes was needed.If you send it all at that
         * size to the BT module, you start getting race conditions and
         * the data gets chopped off.
         * Below parses the NMEA in seperate lines and send it off line by line
         * TODO: merge it with the gps parser
         */
        const uint8_t* q = buffer;
        len = GPSRXBUFFER;
        while (len--) { /* Process all bytes */
            if (*q == 0x24)                                              // '$'
                    {
                y = 0;                // brute force sync on '$' to GPSbuffer[0]
                memset(sendBuffer, 0, SENDBUFFER);
            }

            if (y < SENDBUFFER)
                sendBuffer[y++] = *q;

            if (*q == 0x0d) {
                flag = 1;                       // is the character a CR for eol
                sendBuffer[y++] = '\n';   //only new-line is supported by XCsoar
                y = 0;
            }

            if (flag) {       // test for end of line and if the right GPSbuffer
                flag = 0;                                 // reset for next time

                xQueueSendToBack(uartQueueHandle, sendBuffer, 100);

            }
            q++;
        }

        gps_process(&hgps, &buffer, GPSRXBUFFER);

    }

}

