/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#include <displaytask.h>
extern TaskHandle_t xDisplayNotify;

unsigned char * frame_buffer[EPD_WIDTH * EPD_HEIGHT / 8] __attribute__((section(".ccmram")));
Paint  paint __attribute__((section(".ccmram")));



void StartDisplayTask(void const * argument)
{

	//unsigned char * frame_buffer = (unsigned char*)malloc(EPD_WIDTH * EPD_HEIGHT / 8);
	memset(frame_buffer , 0, EPD_WIDTH * EPD_HEIGHT / 8);
	configASSERT(xDisplayNotify == NULL);
	uint32_t ulNotifiedValue;
	BaseType_t xResult;
	TickType_t xMaxBlockTime;
	TickType_t times;
	uint16_t refreshcount=0;
	xDisplayNotify = xTaskGetCurrentTaskHandle();
	EPD epd;
	displayTaskSetup(&paint,&epd, frame_buffer);


	/* Infinite loop */
	for (;;) {
		times = xTaskGetTickCount();

		 if(refreshcount > (FV_DISPLAYREFRESH * 2)) {
			 refreshcount = 0;
			 displayRefreshMainScreen(&paint,&epd, frame_buffer);
		 }
		 displayTaskUpdate(&paint,&epd,frame_buffer);


		 TickType_t waittime = abs( 500 - (xTaskGetTickCount() - times) ) ;
		 if (waittime > 500) waittime = 500;
		 xMaxBlockTime = pdMS_TO_TICKS(waittime);

		 xResult = xTaskNotifyWait( pdFALSE,    /* Don't clear bits on entry. */
		                            pdTRUE,        /* Clear all bits on exit. */
		                            &ulNotifiedValue, /* Stores the notified value. */
		                            xMaxBlockTime );
		 if( xResult == pdPASS )
		      {
		         /* A notification was received.  See which bits were set. */
		         if(  ulNotifiedValue == 1 )
		         {
		        	 displayMessageShutdown(&paint,&epd,frame_buffer);
		        	 osDelay(4000); //just sleep till shutdown
		         }

		         if(  ulNotifiedValue == 2 )
		         {

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


	  displayDrawmainScreen (paint, epd,frame_buffer);


	  EPD_SetFrameMemory(epd, frame_buffer, 0, 0, epd->width, epd->height);
	  EPD_DisplayFrame(epd);

	  EPD_SetFrameMemory(epd, frame_buffer, 0, 0, epd->width, epd->height);
	  EPD_DisplayFrame(epd);




		  /* EPD_or partial update */
		  if (EPD_Init(epd, lut_partial_update) != 0) {
		    return;
		  }


}

void displayRefreshMainScreen(Paint *paint, EPD *epd, unsigned char * frame_buffer) {


	  if (EPD_Init(epd, lut_full_update) != 0) {
	    return;
	  }
	  Paint_Init(paint, frame_buffer, epd->width, epd->height);
	  Paint_Clear(paint, UNCOLORED);

	  displayDrawmainScreen (paint, epd,frame_buffer);

	  EPD_SetFrameMemory(epd, frame_buffer, 0, 0, epd->width, epd->height);
	  EPD_DisplayFrame(epd);

	  EPD_SetFrameMemory(epd, frame_buffer, 0, 0, epd->width, epd->height);
	  EPD_DisplayFrame(epd);

	  if (EPD_Init(epd, lut_partial_update) != 0) {
	    return;
	  }


}

void displayDrawmainScreen (Paint *paint, EPD *epd, unsigned char * frame_buffer) {
	  uint8_t positop;
	  uint8_t size=74;
	  //draw boxes
    //Vario
	  positop = 0;
	  Paint_DrawStringAt(paint,  3, positop + 4, "Vario", &Font12, COLORED);
	  Paint_DrawRectangle(paint, 0, 0, epd->width-1, 74, COLORED);
	  Paint_DrawStringAt(paint,  epd->width-24, positop + 4, "m/s", &Font12, COLORED);

	  //Altitude
	  positop = 74;
	  Paint_DrawStringAt(paint,  3, positop + 4, "Alt", &Font12, COLORED);
	  Paint_DrawRectangle(paint, 0, 74, epd->width-1, 148, COLORED);
	  Paint_DrawStringAt(paint,  epd->width-10, positop + 4, "m", &Font12, COLORED);

	  //Ground Speed
	  positop = 148;
	  Paint_DrawStringAt(paint,  3, positop + 4, "Speed", &Font12, COLORED);
	  Paint_DrawRectangle(paint, 0, 148, epd->width-1, 222, COLORED);
	  Paint_DrawStringAt(paint,  epd->width-31, positop + 4, "km/h", &Font12, COLORED);

	  //Infobox
	  positop = 222;
	//  Paint_DrawStringAt(paint, positop + 3, 4, "Altitude", &Font8, COLORED);
	  Paint_DrawRectangle(paint, 0, 222, epd->width-1, epd->height-1, COLORED);

}

void displayTaskUpdate(Paint *paint,EPD *epd, unsigned char * frame_buffer) {
	char BmpAltitude[9];
	char BmpVario[9];
	char BmpTemp[6];
	char GPSSpeed[9];

	    Paint_SetWidth(paint, 112);
	    Paint_SetHeight(paint, 41);


//    fpart = sensors.temperature/100;
//    bpart = sensors.temperature % 100;
//    sprintf(BmpTemp,"%d.%02d",fpart,bpart);

	intTocharFloat(BmpVario, sensors.VarioMs,1000,100);

    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 3, 4, BmpVario, &Font32, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 8, 20, Paint_GetWidth(paint), Paint_GetHeight(paint));


    sprintf(BmpAltitude," %d",sensors.AltitudeMeters/1000);
    //intTocharFloat(BmpAltitude, sensors.AltitudeMeters,1000,1000);
    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 0, 4, BmpAltitude, &Font28, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 8, 98, Paint_GetWidth(paint), Paint_GetHeight(paint));



    intTocharFloat(GPSSpeed, hgps.speed*1.85,1,10);
    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 0, 4, GPSSpeed, &Font28, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 8, 173, Paint_GetWidth(paint), Paint_GetHeight(paint));

    intTocharFloat(BmpTemp, sensors.temperature,100,1);
    Paint_Clear(paint, UNCOLORED);

    if (activity.flightstatus == FLS_FLYING) {
    	Paint_DrawStringAt(paint, 0, 4,"Flying", &Font16, COLORED);
    }else{
    	Paint_DrawStringAt(paint, 0, 4,"Not Flying", &Font16, COLORED);
    }


    EPD_SetFrameMemory(epd, frame_buffer, 8, 250, Paint_GetWidth(paint), Paint_GetHeight(paint));


   EPD_DisplayFrame(epd);



}

void displayMessageShutdown(Paint *paint,EPD *epd, unsigned char * frame_buffer) {

	      EPD_ClearFrameMemory(epd, 0xFF);
		  EPD_DisplayFrame(epd);
		  EPD_ClearFrameMemory(epd, 0xFF);
		  EPD_DisplayFrame(epd);

	  if (EPD_Init(epd, lut_full_update) != 0) {
	    return;
	  }


          Paint_Clear(paint, UNCOLORED);
          Paint_DrawStringAt(paint, 8, 4, "FreeVario", &Font16, COLORED);
          EPD_SetFrameMemory(epd, frame_buffer, 2, 25, Paint_GetWidth(paint), Paint_GetHeight(paint));

	      Paint_Clear(paint, UNCOLORED);
	      Paint_DrawStringAt(paint, 50, 4, "is", &Font16, COLORED);
	      EPD_SetFrameMemory(epd, frame_buffer, 2, 100, Paint_GetWidth(paint), Paint_GetHeight(paint));

	      Paint_Clear(paint, UNCOLORED);
	      Paint_DrawStringAt(paint, 8, 4, "sleeping", &Font16, COLORED);
	      EPD_SetFrameMemory(epd, frame_buffer, 2, 175, Paint_GetWidth(paint), Paint_GetHeight(paint));

	      EPD_DisplayFrame(epd);



}


//char array, the value, divide by amount (1000), reduce after 0
void intTocharFloat(char *buffer, int value, uint16_t div, uint16_t dif){

	int fpart;
	int16_t bpart;



    char *tmpSign = (value < 0) ? "-" : " ";
    fpart = abs(value)/div;
    bpart = abs(value) % div;
    if (dif >0) bpart = bpart / dif;

    if ((div/dif) >= 100) {
    	sprintf(buffer,"%s%d.%02d",tmpSign,fpart,bpart);
    }else{
    	sprintf(buffer,"%s%d.%1d",tmpSign,fpart,bpart);
    }


}





