/*
 FreeVario http://FreeVario.org

  Copyright (c), FreeVario (http://freevario.org)
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  any later version. see <http://www.gnu.org/licenses/>
*/


#ifndef FVCONFIG_H_
#define FVCONFIG_H_
#ifdef __cplusplus
 extern "C" {
#endif
#include "math.h"
#include "stdbool.h"

#include "fvconfig_hal.h"
#include "settings.h"
#include "stackops.h"



//#define SETUPBT

#define DEBUGUSBOUT //use the usb for print. It disable sending data out

#define ACCLSMOOTH 10 //Lowpass filter level
#define UPDATELOGFILETIME 30000 //time in ms to update de Log summary file
#define LANDEDSPEED 5 //Ground Speed in knots to detect landing
#define TAKEOFFSPEED 10 //Ground Speed in knots to detect takeoff
#define LANDEDLOWSPEEDTIME 20000 //Time in ms groundspeed lower than LANDEDSPEED to trigger landed detection
#define STARTDELAY 8000 //the time delay before the process starts
#define TAKEOFFVARIO 800 //0.4 //abs vario level to detect takeoff
#define BUZZERZEROCLIMB -0.2 // Normal sink rate for glider is -0.9 m/s. At this value up to 0 m/s a "blip" sound is made
//#define BUZZERSTARTUPSOUND // beep when starting up
#define BUZZER //enble the buzzer function
#define BUZZSINKALERT -2 //Alert if sinking harder than normal (not sink alarm)
#define BUZZSINKALERTPAUSE 8000 //pause length in between alerts
#define BUZZERVARIOSTOP 10000 //time vario STOP making noise when climbrate 0 m/s
//#define OUTOFTHERMALBUZZT 3000 //time buzzer goes buuhhhhh
#define SOARDETECTION 30000 // if climbrate is constant for set milliseconds at 0 m/s the buzzer is muted
#define BUZZERCLIMBING 0.1 // vario level to start giving climbing signal
#define AUDIOSMOOTH 10 //smooth out audio changes
//#define VARIOLOWPASSFILTER  // smmooth out the vario (disable the BMP280 sensor filter if used)
//#define ADAPTIVEVARIO //Adapts the vario low pass filter
//#define USEKALMANFILTER  //still under development
#define SENSORREADMS    50  //ms sensors are read
//#define TESTBUZZER

#define SENDBUFFER 164 //global size of the send buffer
#define GPSRXBUFFER 1024
#define CONFIGFILENAME "settings.cfg"

#define PWRBUTTONDELAY 3000//Time powerbutton must be pressed


 // Z_VARIANCE is the measured sensor altitude cm noise variance, with sensor at rest, normal sampling rate, and 1-2 seconds max samples
 // I personally use a somewhat smaller value than the measured variance as I favour a faster response to step inputs and am willing to
 // tolerate a bit of jitter with the unit at rest.
 #define Z_VARIANCE          300.0f

 // The filter models acceleration as an external environmental disturbance to the system, ZACCEL_VARIANCE is the estimated
 // variance of the perturbations. For a paragliding application, this would be the expected acceleration variance due to thermal
 // activity, how sharp the thermal edges are, etc.  This is NOT the accelerometer sensor noise variance. Increase this value and the
 // filter will respond faster to acceleration inputs.
 #define ZACCEL_VARIANCE     200.0f

 // The accelerometer bias (offset between true acceleration and measured value) is not likely to change rapidly, so a low value
 // of ZACCELBIAS_VARIANCE will enforce that. But too low a value, and the filter will take longer on reset to settle to the estimated
 // value. For an audio variometer, the symptom would be the variometer beeping for several seconds after reset, with the unit at rest.
 #define ZACCELBIAS_VARIANCE 1.0f


#endif /* FVCONFIG_H_ */
