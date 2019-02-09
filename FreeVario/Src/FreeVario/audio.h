/*
 * audio.h
 *
 *  Created on: Nov 18, 2018
 *      Author: marco
 */

#ifndef AUDIO_H_
#define AUDIO_H_

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <bmp280.h>
#include <globaldata.h>


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
}audio_t;

void setupAudio(audio_t * audio);
void makeVarioAudio(audio_t * audio,float vario);
void tone(audio_t * audio, float freq, int period);
void toneconstant(audio_t * audio, float freq);
void noTone();
int millis();
void playTonePause(audio_t * audio,int freq, int nbeeps, int tpause);
void playToneInterval(audio_t * audio, int freq, int period, int tinterval);
void playTwoToneInterval(audio_t * audio,int freq,int freq2, int period, int intervald);
void noToneTimer(audio_t * audio);
void dynaTone(audio_t * audio, float freq);
void testtone(audio_t * audio, float freq);


#endif /* AUDIO_H_ */
