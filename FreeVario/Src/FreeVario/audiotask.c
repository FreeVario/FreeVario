/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
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
extern ActivityData activity;
extern WWDG_HandleTypeDef hwwdg;

    TickType_t times;
    const TickType_t xDelay = 10; //do not change, need to be precise for the wwdt

void StartAudioTask(void const * argument) {

    uint8_t audioon = 0;

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
        times = xTaskGetTickCount();

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

        }
        //toneconstant(&audiorun, 1000);
        makeVarioAudio(&audiorun, t_vario);
#else

        if (running) {

            if (activity.flightstatus == FLS_FLYING && !activity.muted) {

                makeVarioAudio(&audiorun, sensors.VarioMs); //flying

                audioon = 1;

            }
            if ((activity.flightstatus == FLS_GROUND && audioon)
                    || (activity.muted && audioon)) {
                audioon = 0;
                noTone();
            }
        }
#endif

        vTaskDelayUntil(&times, xDelay);
        HAL_WWDG_Refresh(&hwwdg);


    }

}

