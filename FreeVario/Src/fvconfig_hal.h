/*
 FreeVario http://FreeVario.org

  Copyright (c), FreeVario (http://freevario.org)
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  any later version. see <http://www.gnu.org/licenses/>

 * config_hal.h
 *
 * Processor, channel and pin mappings
 * Only this file should needed to be configured when changing processors
 * Better use FV_ preamble to avoid conflicts
 */

#ifndef FVCONFIG_HAL_H_
#define FVCONFIG_HAL_H_
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"

//Save config to SDCard
#define SDCONFIGSAVE
#define CONFIGFILENAME "settings.cfg"
#define CONFIGVERSION 3

//Board LED
#define FV_LED PB9_Pin
#define FV_LED_GPIO	GPIOB

//Audio baseline for buzzer
#define FV_TONEBASE 600

// Master I2C channel
/* Use this for the Baro and ACCL sensor
 * Configure: Fast mode 400
 * Disable the Analog filter if there are a lot of read errors
 * Setting digital filter to 2 (or higher) will help a lot
 */
#define FV_I2C hi2c1


//ACCL address
#define FV_ACCL_ADR SD_MPU6050_Device_0

//GPS
/*
 * Speed: set for correct speed GPS
 * DMA: Enable DMA Channel with circular buffer
 * and set Global interrupt
 * Set polarity low and idle state to set
 */
#define FV_UARTGPS huart1


//BT
/*
 * Set Correct speed
 * DMA not needed
 */
#define FV_UARTBT huart3


//HW output for the speaker
#define FV_TONETMR htim1
#define FV_TONECHN TIM_CHANNEL_1
#define FV_TONEHALTMR TIM1


//Old gpio input format for DHT sensors, not implemented yet (might never be)
//#define FV_DHTPIN GPIO_PIN_1
//#define FV_DHTPORT GPIOB



//Watchdog timer
/*
 * Set prescaler to 32
 */
#define FV_IWDG hiwdg

/*
 * Power management
 *
 */


//ADC for reading internal vBat Channel
//ADC1 needs to be enabled for it
//Continuous conversion mode set to enabled
#define FV_HALADC hadc1

/*
 * Buttons
 * Configure all buttons as GPIO_EXTI
 * to use the interrupt handler.
 * Use the same port
 */

#define FV_BTNOPTION GPIO_PIN_3
#define FV_BTNNEXT	GPIO_PIN_5 //B
#define FV_BTNPREV  GPIO_PIN_4 //B
#define FV_BRNPRT 	GPIOB




#endif /* FVCONFIG_HAL_H_ */
