/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#include <displaytask.h>
#include "util.h"
#include "datalog.h"
#include <readsensors.h>
#include "freevario.h"

extern TaskHandle_t xDisplayNotify;

unsigned char * frame_buffer[EPD_WIDTH * EPD_HEIGHT / 8] __attribute__((section(".ccmram")));
Paint paint __attribute__((section(".ccmram")));

void StartDisplayTask(void const * argument) {

    //unsigned char * frame_buffer = (unsigned char*)malloc(EPD_WIDTH * EPD_HEIGHT / 8);
    memset(frame_buffer, 0, EPD_WIDTH * EPD_HEIGHT / 8);
    configASSERT(xDisplayNotify == NULL);
    uint32_t ulNotifiedValue;
    BaseType_t xResult;
    TickType_t xMaxBlockTime;
    TickType_t times;
    uint16_t refreshcount = 0;
    xDisplayNotify = xTaskGetCurrentTaskHandle();
    EPD epd;
    displayTaskSetup(&paint, &epd, frame_buffer);

    /* Infinite loop */
    for (;;) {
        times = xTaskGetTickCount();

        if (refreshcount > (FV_DISPLAYREFRESH * 2)) {
            refreshcount = 0;
            displayRefreshMainScreen(&paint, &epd, frame_buffer);
        }
        displayTaskUpdate(&paint, &epd, frame_buffer);

        TickType_t waittime = abs(500 - (xTaskGetTickCount() - times));
        if (waittime > 500)
            waittime = 500;
        xMaxBlockTime = pdMS_TO_TICKS(waittime);

        xResult = xTaskNotifyWait( pdFALSE, /* Don't clear bits on entry. */
        pdTRUE, /* Clear all bits on exit. */
        &ulNotifiedValue, /* Stores the notified value. */
        xMaxBlockTime);
        if (xResult == pdPASS) {
            /* A notification was received.  See which bits were set. */
            if (ulNotifiedValue == 1) {
                displayMessageShutdown(&paint, &epd, frame_buffer);
                displayMessageShutdown(&paint, &epd, frame_buffer); //also the second buffer
                osDelay(8000); //just sleep till shutdown
            }

            if (ulNotifiedValue == 2) {

            }
        }
        refreshcount++;

    }

}

void displayTaskSetup(Paint *paint, EPD *epd, unsigned char * frame_buffer) {

    if (EPD_Init(epd, lut_full_update) != 0) {
        return;
    }

    Paint_Init(paint, frame_buffer, epd->width, epd->height);
    Paint_Clear(paint, UNCOLORED);

    EPD_ClearFrameMemory(epd, 0xFF);
    EPD_DisplayFrame(epd);
    EPD_ClearFrameMemory(epd, 0xFF);
    EPD_DisplayFrame(epd);
//	  /**
//	   *  there are 2 memory areas embedded in the e-paper display
//	   *  and once the display is refreshed, the memory area will be auto-toggled,
//	   *  i.e. the next action of SetFrameMemory will set the other memory area
//	   *  therefore you have to set the frame memory and refresh the display twice.
//	   */

    displayDrawmainScreen(paint, epd, frame_buffer);

    EPD_SetFrameMemory(epd, frame_buffer, 0, 0, epd->width, epd->height);
    EPD_DisplayFrame(epd);

    EPD_SetFrameMemory(epd, frame_buffer, 0, 0, epd->width, epd->height);
    EPD_DisplayFrame(epd);

    /* EPD_or partial update */
    if (EPD_Init(epd, lut_partial_update) != 0) {
        return;
    }

}

void displayRefreshMainScreen(Paint *paint, EPD *epd,
        unsigned char * frame_buffer) {

    if (EPD_Init(epd, lut_full_update) != 0) {
        return;
    }
    Paint_Init(paint, frame_buffer, epd->width, epd->height);
    Paint_Clear(paint, UNCOLORED);

    displayDrawmainScreen(paint, epd, frame_buffer);

    EPD_SetFrameMemory(epd, frame_buffer, 0, 0, epd->width, epd->height);
    EPD_DisplayFrame(epd);

    EPD_SetFrameMemory(epd, frame_buffer, 0, 0, epd->width, epd->height);
    EPD_DisplayFrame(epd);

    if (EPD_Init(epd, lut_partial_update) != 0) {
        return;
    }

}

void displayDrawmainScreen(Paint *paint, EPD *epd, unsigned char * frame_buffer) {
    uint8_t positop;
    //draw boxes
    //Vario
    positop = 0;
    Paint_DrawStringAt(paint, 3, positop + 4, "Vario", &Font10, COLORED);
    Paint_DrawRectangle(paint, 0, 0, epd->width - 1, 74, COLORED);
    Paint_DrawStringAt(paint, epd->width - 32, positop + 4, "m/s", &Font12,
            COLORED);

    //Altitude
    positop = 74;
    Paint_DrawStringAt(paint, 3, positop + 4, "Alt", &Font10, COLORED);
    Paint_DrawRectangle(paint, 0, 74, epd->width - 1, 148, COLORED);
    Paint_DrawStringAt(paint, epd->width - 12, positop + 4, "m", &Font12,
            COLORED);

    //Ground Speed
    positop = 148;
    Paint_DrawStringAt(paint, 3, positop + 4, "Speed", &Font10, COLORED);
    Paint_DrawRectangle(paint, 0, 148, epd->width - 1, 222, COLORED);
    Paint_DrawStringAt(paint, epd->width - 42, positop + 4, "km/h", &Font12,
            COLORED);

    //Infobox
    positop = 222;
    //  Paint_DrawStringAt(paint, positop + 3, 4, "Altitude", &Font8, COLORED);
    Paint_DrawRectangle(paint, 0, 222, epd->width - 1, epd->height - 1,
            COLORED);

}

void displayTaskUpdate(Paint *paint, EPD *epd, unsigned char * frame_buffer) {
    char BmpAltitude[9];
    char BmpVario[9];
    char BmpTemp[6];
    char BmpHumid[6];
    char BmpBat[6];
    char BmpGforce[6];
    char GPSSpeed[9];

    Paint_SetWidth(paint, 112);
    Paint_SetHeight(paint, 41);

    /* --------------------Vario-----------------------------*/

    Paint_SetWidth(paint, 26);
    Paint_SetHeight(paint, 14);
    Paint_Clear(paint, UNCOLORED);

    if (activity.useKalman) {

        Paint_DrawStringAt(paint, 0, 0, "K", &Font10, COLORED);

    } else {

        Paint_DrawStringAt(paint, 0, 0, " ", &Font10, COLORED);
    }

    EPD_SetFrameMemory(epd, frame_buffer, epd->width - 65, 4,
            Paint_GetWidth(paint), Paint_GetHeight(paint));

    Paint_SetWidth(paint, 112);
    Paint_SetHeight(paint, 41);

    intTocharFloat(BmpVario, sensors.VarioMs, 1000, 100, 1);
    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 0, 4, BmpVario, &Font32, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 8, 20, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    /* ---------------------Alt----------------------------*/

    Paint_SetWidth(paint, 26);
    Paint_SetHeight(paint, 14);
    Paint_Clear(paint, UNCOLORED);

    if (activity.barognssavalid) {

        sprintf(BmpAltitude, " %d",
                (sensors.AltitudeMeters - activity.barognssdeveation) / 1000);
        Paint_DrawStringAt(paint, 0, 0, "   ", &Font10, COLORED);

    } else {
        sprintf(BmpAltitude, " %d", sensors.AltitudeMeters / 1000);
        Paint_DrawStringAt(paint, 0, 0, "QNE", &Font10, COLORED);
    }
    EPD_SetFrameMemory(epd, frame_buffer, epd->width - 74, 78,
            Paint_GetWidth(paint), Paint_GetHeight(paint));

    Paint_SetWidth(paint, 112);
    Paint_SetHeight(paint, 41);

    //intTocharFloat(BmpAltitude, sensors.AltitudeMeters,1000,1000);
    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 0, 4, BmpAltitude, &Font28, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 8, 98, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    /* --------------------Speed-----------------------------*/

    intTocharFloat(GPSSpeed, hgps.speed * 1.85, 1, 10, 1);
    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 0, 4, GPSSpeed, &Font28, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 8, 173, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    /* --------------------Status-----------------------------*/
    //status displays
    Paint_SetWidth(paint, 45);
    Paint_SetHeight(paint, 20);

    //flying
    Paint_Clear(paint, UNCOLORED);
    if (activity.flightstatus == FLS_FLYING) {
        Paint_DrawStringAt(paint, 0, 0, "PF", &Font14, COLORED);
    } else {
        Paint_DrawStringAt(paint, 0, 0, "PNF", &Font14, COLORED);
    }
    EPD_SetFrameMemory(epd, frame_buffer, 8, 230, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    //GPS Fix
    Paint_SetWidth(paint, 12);
    Paint_Clear(paint, UNCOLORED);
    if (hgps.fix > 0) {
        Paint_DrawStringAt(paint, 0, 0, "*", &Font14, COLORED);
    } else {
        Paint_DrawStringAt(paint, 0, 0, "!", &Font14, COLORED);
    }
    EPD_SetFrameMemory(epd, frame_buffer, epd->width - 74, 230,
            Paint_GetWidth(paint), Paint_GetHeight(paint));

    //datalogger
    Paint_SetWidth(paint, 50);
    Paint_Clear(paint, UNCOLORED);
    if (datalog.isLogging) {
        Paint_DrawStringAt(paint, 0, 0, " LOG", &Font14, COLORED);
    } else if (activity.SDcardMounted) {
        Paint_DrawStringAt(paint, 0, 0, " SD ", &Font14, COLORED);
    } else {
        Paint_DrawStringAt(paint, 0, 0, " ---", &Font14, COLORED);
    }
    EPD_SetFrameMemory(epd, frame_buffer, epd->width - 58, 230,
            Paint_GetWidth(paint), Paint_GetHeight(paint));

    //temp
    Paint_SetWidth(paint, 45);
    Paint_Clear(paint, UNCOLORED);
    //intTocharFloat(BmpTemp, sensors.temperature,100,1);
    sprintf(BmpTemp, "%dC", (int) sensors.temperature / 100);
    Paint_DrawStringAt(paint, 0, 0, BmpTemp, &Font14, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 8, 252, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    //Muted
    Paint_SetWidth(paint, 12);
    Paint_Clear(paint, UNCOLORED);
    if (activity.muted > 0) {
        Paint_DrawStringAt(paint, 0, 0, "m", &Font14, COLORED);
    } else {
        Paint_DrawStringAt(paint, 0, 0, ".", &Font14, COLORED);
    }
    EPD_SetFrameMemory(epd, frame_buffer, epd->width - 74, 252,
            Paint_GetWidth(paint), Paint_GetHeight(paint));

    //humid
    Paint_SetWidth(paint, 50);
    Paint_Clear(paint, UNCOLORED);
    -sprintf(BmpHumid, " %d%%", sensors.humidity / 100);
    Paint_DrawStringAt(paint, 0, 0, BmpHumid, &Font14, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, epd->width - 58, 252,
            Paint_GetWidth(paint), Paint_GetHeight(paint));

    //battery
    Paint_SetWidth(paint, 50);
    Paint_Clear(paint, UNCOLORED);
    intTocharFloat(BmpBat, sensors.vbat, 10, 1, 0);
    strncat(BmpBat, "V", 1);
    Paint_DrawStringAt(paint, 0, 0, BmpBat, &Font14, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 8, 275, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    //gforce
    Paint_SetWidth(paint, 50);
    Paint_Clear(paint, UNCOLORED);
    intTocharFloat(BmpGforce, sensors.gforce, 100, 10, 1);
    strncat(BmpGforce, "G", 1);
    Paint_DrawStringAt(paint, 0, 0, BmpGforce, &Font14, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, epd->width - 58, 275,
            Paint_GetWidth(paint), Paint_GetHeight(paint));

    EPD_DisplayFrame(epd);

}

void displayMessageShutdown(Paint *paint, EPD *epd,
        unsigned char * frame_buffer) {

    Paint_SetWidth(paint, epd->width);
    Paint_SetHeight(paint, epd->height);
    Paint_Clear(paint, UNCOLORED);

    EPD_ClearFrameMemory(epd, 0xFF);
    EPD_DisplayFrame(epd);
    EPD_ClearFrameMemory(epd, 0xFF);
    EPD_DisplayFrame(epd);

    if (EPD_Init(epd, lut_full_update) != 0) {
        return;
    }

    uint8_t sp = 10;
    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 2, 0, "Flyable?", &Font16, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 2, sp, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 2, 0, "NOTAM", &Font12, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 4, sp += 25, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 2, 0, "Wind <500", &Font12, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 4, sp += 18, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 2, 0, "Wind >500", &Font12, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 4, sp += 18, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 2, 0, "Rain/Front", &Font12, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 4, sp += 18, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 2, 0, "Thermal m/s", &Font12, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 4, sp += 18, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 2, 0, "Condition", &Font16, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 4, sp += 25, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 2, 0, "Cloud cover", &Font12, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 4, sp += 25, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 2, 0, "Temperature", &Font12, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 4, sp += 18, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 2, 0, "Cloud Base", &Font12, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 4, sp += 18, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 2, 0, "Air Pressure", &Font12, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 4, sp += 18, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 2, 0, "Over develop", &Font12, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 4, sp += 18, Paint_GetWidth(paint),
            Paint_GetHeight(paint));

    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 2, 0, "Inversion", &Font12, COLORED);

    EPD_SetFrameMemory(epd, frame_buffer, 4, sp += 18, Paint_GetWidth(paint),
            Paint_GetHeight(paint));
    EPD_DisplayFrame(epd);

}

