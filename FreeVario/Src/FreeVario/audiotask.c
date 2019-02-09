/*
 FreeVario http://FreeVario.org

 Copyright (c), PrimalCode (http://www.primalcode.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */


#include "audiotask.h"
#include <stdlib.h>
#include <string.h>
#include "audio.h"

extern SensorData sensors;

void StartAudioTask(void const * argument)
{
  /* USER CODE BEGIN StartAudioTask */

#ifdef TESTBUZZER

    int step=100;
	int t_vario=1000;
	uint32_t times=0;

#endif
	audio_t audiorun;
//	audiorun.multiplier = 100000000;
	uint8_t running = 0;

	setupAudio(&audiorun);
	/* Infinite loop */
	for (;;) {

		if (xTaskGetTickCount() > STARTDELAY) {
			running = 1;
		}
#ifdef TESTBUZZER
		 uint32_t i =xTaskGetTickCount() - times;

	  if (i > 1000) {
	    times = xTaskGetTickCount();
	   // if(t_vario <= -5000) step = 100;
		//if(t_vario >= 9000) step = -100;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		t_vario += step;
		sensors.barotakeoff = 1;
	  }
	  //toneconstant(&audiorun, 1000);
	  makeVarioAudio(&audiorun, t_vario);
#else

		if (running) {
		if (sensors.barotakeoff) {
				makeVarioAudio(&audiorun, sensors.VarioMs); //flying

			}
		}
#endif
		osDelay(10);
	}
  /* USER CODE END StartAudioTask */
}
