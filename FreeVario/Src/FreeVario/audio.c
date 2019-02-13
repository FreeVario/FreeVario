/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */



#include "audio.h"


extern TIM_HandleTypeDef FV_TONETMR;


void setupAudio(audio_t * audio) {


	 HAL_TIM_PWM_Start(&FV_TONETMR, FV_TONECHN);
	 HAL_TIMEx_PWMN_Start(&FV_TONETMR, FV_TONECHN);
	 FV_TONEHALTMR->PSC  = SystemCoreClock/10000000;
	 toneconstant(audio, 1000);
	 osDelay(10);
	 noTone();
}

#define PWMTMRMULTIPLIER 10000000


void noTone() {

	FV_TONEHALTMR->CR1 &= ~TIM_CR1_CEN;
	FV_TONEHALTMR->CNT = 0;


}
#define BASEPULSE 200
#define TOPPULSE  1000

 void tone(audio_t * audio, float freq, int period) {

	    audio->notonetimer = period + millis();
	    uint16_t fv_tone_t = 1/(float)freq * PWMTMRMULTIPLIER;
	    FV_TONEHALTMR->CR1 |= TIM_CR1_CEN;
	    FV_TONEHALTMR->ARR  = fv_tone_t;
		FV_TONEHALTMR->CCR1 = fv_tone_t/2;
}

void toneconstant(audio_t * audio, float freq) {
    uint16_t fv_tone_t = 1/(float)freq * PWMTMRMULTIPLIER;
    FV_TONEHALTMR->CR1 |= TIM_CR1_CEN;
    FV_TONEHALTMR->ARR  = fv_tone_t;
	FV_TONEHALTMR->CCR1 = fv_tone_t/2;
}

//changes tone while beeping
 void dynaTone(audio_t * audio, float freq) {
		uint16_t fv_tone_t = 1/(float)freq * PWMTMRMULTIPLIER;
		FV_TONEHALTMR->ARR  = fv_tone_t;
		FV_TONEHALTMR->CCR1 = fv_tone_t/2;
}


 int millis() {

	//return HAL_GetTick();
	return xTaskGetTickCount();
}



 void noToneTimer(audio_t * audio) {
   if(millis() >= audio->notonetimer && audio->notonetimer > 0) {
     noTone();
     audio->notonetimer = 0;
   }

}


// Non-Blocking beep blob beep
 void playTwoToneInterval(audio_t * audio, int freq,int freq2, int period, int intervald) {


  if (audio->toneOn) {
    int wait = period + audio->tm;


    if ( wait < millis()) {
    	audio->toneOn = false;
      //noTone();
      toneconstant(audio, freq2);
      audio->rm = millis();
    }

  } else {
    int ndwait = intervald + audio->rm;

    if(ndwait < millis()) {

    toneconstant(audio, freq);
    audio->toneOn = true;
    audio->tm = millis();
    }
  }

}


// Non-Blocking beep beep beep
 void playToneInterval(audio_t * audio, int freq, int period, int tinterval) {

  if (audio->toneOn) {
    int wait = period + tinterval + audio->tm;

    if ( wait < millis()) {
    	audio->toneOn = false;
      noTone();
      audio->tcount++; // count the amount of beeps for playTonePause
      if (audio->tcount > 1000) { // prevent overflow
    	  audio->tcount = 0;
      }
    }

  } else {
	  tone(audio, freq, period);
	  audio->toneOn = true;
	  audio->tm = millis();
  }

}


// Plays nbeeps then pause

 void playTonePause(audio_t * audio, int freq, int nbeeps, int tpause) {

   if (audio->pause < millis()) {

      if (audio->tcount < nbeeps) {
        playToneInterval(audio,freq, 500, 200);

      }else{
    	  audio->pause = millis() + tpause;
    	  audio->tcount=0;

      }


   }


}

//main task to poll
void makeVarioAudio(audio_t * audio, float vario) {
  int pulse;
  vario = vario /1000; //TODO: change to int32
  float varioorg = vario;
  float variofr;
   noToneTimer(audio);


  if (vario > 20) {
    vario = 20;

  }

  if (vario < -20) {
    vario = -20;

  }

#if defined(SOARDETECTION) && !defined(TESTBUZZER)


  if (varioorg > -0.2 && varioorg < 0.2) { //TODO: add to conf
    int diffe = millis() - audio->stime;
    if (diffe >  (int)(conf.SoarDeadBandTime)) {
    	audio->muted = true;
    }
  } else {
	  audio->stime = millis();
	  audio->muted = false;
  }

#endif

  variofr = ((float)(fabs(vario + 1)) * 150 ) + FV_TONEBASE;

  audio->variof = (AUDIOSMOOTH * audio->variof + variofr )/(AUDIOSMOOTH + 1);

    if (vario <= BUZZERCLIMBING && vario >= BUZZERZEROCLIMB) { // prethermal audio bip bip bip

      if (!audio->muted) {
         playToneInterval(audio, audio->variof, 50, 400);
      }

    }

   if (vario <= (double)(conf.sinkAlarmLevel)/1000 ) { //sink alarm
      if (!audio->muted) {
         playTwoToneInterval(audio,1400, 1800, 40, 40);
      }

   }

#if defined(BUZZSINKALERT) //sink alert beh beh beh (-3)
    if (vario <=  BUZZSINKALERT && vario > (double)(conf.sinkAlarmLevel)/1000 ) {
       playTonePause(audio, 300, fabs(vario), BUZZSINKALERTPAUSE);
    }

#endif


  if (vario > BUZZERCLIMBING) {
	  pulse = TOPPULSE / (vario * 10) + 100;

    if (!audio->muted) {
      dynaTone(audio, audio->variof);
      playToneInterval(audio, audio->variof, pulse, pulse / 2);
    }
   // climbing = true;
  }



  //else {
  //  if (climbing ) { //dropped out of the thermal
  //    tone( 100, OUTOFTHERMALBUZZT);
  //    climbing = false;
  //  }
  //}

}

void testtone(audio_t * audio, float freq) {

	FV_TONEHALTMR->ARR = 1/(float)freq * audio->multiplier;
	FV_TONEHALTMR->CR1 |= TIM_CR1_CEN;

}
