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


#define FLS_GROUND      0
#define FLS_TAKEOFF     1
#define FLS_FLYING      2
#define FLS_LANDED      3

#define PWRBTNDSPSIGNAL    0x01
#define OKBTNDSPSIGNAL     0x0A


typedef struct {
    uint16_t takeoffYear;
    uint8_t takeoffMonth;
    uint8_t takeoffDate;
    uint8_t takeoffHour;
    uint8_t takeoffMinute;
    uint8_t takeoffSeconds;
    uint32_t takeoffTemp;
    uint32_t takeoffAltitude;
    float takeoffLocationLAT;
    float takeoffLocationLON;
    uint16_t landingYear;
    uint8_t landingMonth;
    uint8_t landingDate;
    uint8_t landingHour;
    uint8_t landingMinute;
    uint8_t landingSeconds;
    float landingLocationLAT;
    float landingLocationLON;
    uint32_t landingAltitude;
    uint8_t flightstatus;
    int32_t MaxAltitudeMeters;
    int32_t MaxAltitudeGainedMeters;
    int32_t MaxVarioMs;
    int32_t MaxVarioSinkMs;
    int32_t currentLogID;
    uint8_t muted;
    int32_t barognssdeveation;  //difference between baro and gnss (baro - gnss)
    uint8_t barognssavalid;    //flag if the adjusted value is usable
    uint8_t useKalman; //use kalman with accelerometer insead of a lowpass filter
    uint8_t SDcardMounted;
} ActivityData; //owner: freevario.c

extern ActivityData activity;



#endif /* FREEVARIO_H_ */
