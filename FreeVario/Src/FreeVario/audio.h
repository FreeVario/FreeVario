/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */


#ifndef AUDIO_H_
#define AUDIO_H_

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../fvconfig.h"
#include <stdlib.h>
#include <bmp280.h>
#include <globaldata.h>


#define AUDIOSINKALARM 0
#define AUDIOSINKALERT 1
#define AUDIOZEROCLIMB 2
#define AUDIOCLIMBING  3

typedef struct{
	int tm;
	int rm;
	int stime;
	uint8_t toneOn;
	uint8_t muted;
	int pause;
	int tcount;
	int notonetimer;
	int multiplier;
	float variof;
	uint8_t transition;
}audio_t;

void setupAudio(audio_t * audio);
void makeVarioAudio(audio_t * audio,float vario);
void tone(audio_t * audio, float freq, int period);
void toneconstant(audio_t * audio, float freq);
void noTone();
void switchTone(audio_t * audio);
int millis();
void playTonePause(audio_t * audio,int freq, int nbeeps, int tpause);
void playToneInterval(audio_t * audio, int freq, int period, int tinterval);
void playTwoToneInterval(audio_t * audio,int freq,int freq2, int period, int intervald);
void noToneTimer(audio_t * audio);
void dynaTone(audio_t * audio, float freq);
void testtone(audio_t * audio, float freq);


#endif /* AUDIO_H_ */
