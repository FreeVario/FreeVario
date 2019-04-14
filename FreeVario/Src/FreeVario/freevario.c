/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

//////////////////////////////////////////////////
/*
 * CUBE MX Build Notes
 * Checkout GIT
 * Load the IOC file
 * Check the clock frequencies (underclock to save power)
 * Re-Check the xtal frequency
 * Generate project
 * Add sub directories to include paths
 * Compile and test
 */

/////////////////////////////////////////////////

#include "freevario.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"
#include <displaytask.h>
#include <globaldata.h>
#include <stdlib.h>
#include <string.h>
#include "sensorstask.h"
#include "gpstask.h"
#include "sendatatask.h"
#include "audiotask.h"
#include "loggertask.h"
#include "fanettask.h"
#include "util.h"


/* FV CCM memory allocation-----------------------------------------------------*/

SensorData sensors __attribute__((section(".ccmram")));
ActivityData activity __attribute__((section(".ccmram")));
settings_t conf __attribute__((section(".ccmram")));


/*  Standard Vars---------------------------------------------------------------*/

//TODO: Move all extern declerations to apropriate header files see:
// https://sogilis.com/blog/wrong-usage-of-extern-keyword-in-c/
extern char SDPath[4]; /* SD logical drive path */
extern FATFS SDFatFS; /* File system object for SD logical drive */
extern gps_t  hgps;
extern RTC_HandleTypeDef hrtc;


__IO uint8_t UserPowerButton = 0;
__IO uint8_t UserOkButton = 0;
__IO uint8_t UserCancelButton = 0;
__IO uint8_t UserNextButton = 0;
__IO uint8_t UserPrevButton = 0;
uint8_t SDcardMounted = 0;
uint8_t HasSetTime = 0;
TickType_t pwrBTtimePressed = 0;
TickType_t OkButtonTimePressed = 0;
TickType_t CancelButtonTimePressed = 0;
TickType_t NextButtonTimePressed = 0;
TickType_t PrevButtonTimePressed = 0;



/*  FreeRtos-------------------------------------------------------------------*/

osThreadId fvdefaultTaskHandle;
osThreadId displayTaskHandle;
osThreadId sensorsTaskHandle;
osThreadId gpsTaskHandle;
osThreadId sendDataTaskHandle;
osThreadId audioTaskHandle;
osThreadId fanetTaskHandle;
osThreadId loggerTaskHandle;
osMutexId confMutexHandle;
osMutexId sdCardMutexHandle;


/* FV Task allocation----------------------------------------------------------*/

TaskHandle_t xReceiveNotify = NULL;
TaskHandle_t xSendDataNotify = NULL;
TaskHandle_t xDisplayNotify = NULL;
TaskHandle_t xLogDataNotify = NULL;

/* FV Queues  -----------------------------------------------------------------*/

QueueHandle_t uartQueueHandle;

/* FV Semaphores and Mutexes---------------------------------------------------*/



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {

	  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	  	vTaskNotifyGiveFromISR(xReceiveNotify, &xHigherPriorityTaskWoken);
	  	xReceiveNotify = NULL;
	  	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);


}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {

	if (UartHandle->Instance == FV_USARTBT) {

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(xSendDataNotify, &xHigherPriorityTaskWoken);
		xSendDataNotify = NULL;
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

}

void uart_Idle_Handler_Callback(UART_HandleTypeDef *UartHandle){
	  uint32_t tmp_flag = 0, tmp_it_source = 0;
	  tmp_flag = __HAL_UART_GET_FLAG(UartHandle, UART_FLAG_IDLE);
	  tmp_it_source = __HAL_UART_GET_IT_SOURCE(UartHandle, UART_IT_IDLE);


	  if((tmp_flag != RESET) && (tmp_it_source != RESET))
	  {
	    __HAL_UART_CLEAR_IDLEFLAG(UartHandle);
	    __HAL_UART_DISABLE_IT(UartHandle, UART_IT_IDLE);

	    //check for correct DMA
	    DMA2_Stream2->CR &= ~DMA_SxCR_EN; //trigger tx completecallback
	  }



}

/**
 * @brief EXTI line detection callbacks
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == PWRBUTTON_Pin) {
		/* Set the variable: button pressed */
		if (HAL_GetTick() > 5000) {
			pwrBTtimePressed = xTaskGetTickCount();
			UserPowerButton = 1;
		}
	}else if (GPIO_Pin == BTN_OK_Pin) {

		UserOkButton = 1;
		OkButtonTimePressed = xTaskGetTickCount();

	}else if (GPIO_Pin == BTN_CANCEL_Pin) {
		UserCancelButton = 1;
		CancelButtonTimePressed = xTaskGetTickCount();

	}else if (GPIO_Pin == BTN_NEXT_Pin) {
		UserNextButton = 1;
		NextButtonTimePressed = xTaskGetTickCount();

	}else if (GPIO_Pin == BTN_PREV_Pin) {
		UserPrevButton =1 ;
		PrevButtonTimePressed = xTaskGetTickCount();
	}


}



/**
  * @brief  This function configures the system to enter Standby mode for
  *         current consumption measurement purpose.
  *         STANDBY Mode
  *         ============
  *           - Backup SRAM and RTC OFF
  *           - IWDG and LSI OFF
  *           - Wakeup using WakeUp Pin (PA.00)
  * @param  None
  * @retval None
  */
void StandbyMode(void)
{
  /* Enable Power Clock*/
  __HAL_RCC_PWR_CLK_ENABLE();


  /* Write data to backup register to indicate valid shutdown */
  HAL_RTCEx_BKUPWrite(&hrtc, 0, 3);

  /* Disable all used wakeup sources: Pin1(PA.0) */
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);

  /* Clear all related wakeup flags */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);



  /* Re-enable all used wakeup sources: Pin1(PA.0) */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);


  /* Request to enter STANDBY mode  */
  HAL_PWR_EnterSTANDBYMode();
}

void toggleDebugLED() {
#ifdef DEBUG_MODE
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#endif
}



void freeVario_RTOS_Init()  {

	  /* definition and creation of confMutex */
	  osMutexDef(confMutex);
	  confMutexHandle = osMutexCreate(osMutex(confMutex));

	  /* definition and creation of sdCardMutex */
	  osMutexDef(sdCardMutex);
	  sdCardMutexHandle = osMutexCreate(osMutex(sdCardMutex));

	  /* add queues, ... */
	  uartQueueHandle = xQueueCreate(8, SENDBUFFER);

#ifdef FV_DISPLAY
	  /* definition and creation of displayTask */
	  osThreadDef(displayTask, StartDisplayTask, osPriorityBelowNormal, 0, 2048);
	  displayTaskHandle = osThreadCreate(osThread(displayTask), NULL);
#endif

#ifdef FV_SENSORS
	  /* definition and creation of sensorsTask */
	  osThreadDef(sensorsTask, StartSensorsTask, osPriorityNormal, 0, 2048);
	  sensorsTaskHandle = osThreadCreate(osThread(sensorsTask), NULL);
#endif

#ifdef FV_GPS
	  /* definition and creation of gpsTask */
	  osThreadDef(gpsTask, StartGPSTask, osPriorityNormal, 0, 2048);
	  gpsTaskHandle = osThreadCreate(osThread(gpsTask), NULL);
#endif

#ifdef FV_SENDATA
	  /* definition and creation of sendDataTask */
	  osThreadDef(sendDataTask, StartSendDataTask, osPriorityAboveNormal, 0, 2048);
	  sendDataTaskHandle = osThreadCreate(osThread(sendDataTask), NULL);
#endif

#ifdef FV_AUDIO
	  /* definition and creation of audioTask */
	  osThreadDef(audioTask, StartAudioTask, osPriorityNormal, 0, 1024);
	  audioTaskHandle = osThreadCreate(osThread(audioTask), NULL);
#endif

#ifdef FV_FANET
	  /* definition and creation of FanetTask */
	  osThreadDef(fanetTask, StartFanetTask, osPriorityNormal, 0, 1024);
	  fanetTaskHandle = osThreadCreate(osThread(fanetTask), NULL);
#endif

#ifdef FV_LOGGER
	  /* definition and creation of loggerTask */
	  osThreadDef(loggerTask, StartLoggerTask, osPriorityNormal, 0, 2048);
	  loggerTaskHandle = osThreadCreate(osThread(loggerTask), NULL);
#endif

}


void StartDefaultTask(void const * argument)
{

  /* init code for FATFS */
  MX_FATFS_Init();

  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();


  xSemaphoreGive(confMutexHandle);
  xSemaphoreGive(sdCardMutexHandle);

  TickType_t landedcheck=0;
  TickType_t maintenanceloop=0; //loop every 1 minute

  activity.muted = 0;
  activity.flightstatus = FLS_GROUND;
  activity.useKalman = 0;

  memset(&activity, 0, sizeof(activity));

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,SET);

  if ( xSemaphoreTake( sdCardMutexHandle, ( TickType_t ) 500 ) == pdTRUE) {
		if ( xSemaphoreTake( confMutexHandle, ( TickType_t ) 100 ) == pdTRUE) {
			if (f_mount(&SDFatFS, SDPath, 0) == FR_OK) { //Mount SD card
				SDcardMounted =1;
				osDelay(2000); //give some time to load it up
			}

			loadConfigFromSD();
			xSemaphoreGive(confMutexHandle);
		}
		xSemaphoreGive(sdCardMutexHandle);
  }

	/* Infinite loop */
	for (;;) {

		maintenanceloop++;

		if  (UserPowerButton) {
			osDelay(20);

			if (((xTaskGetTickCount() - pwrBTtimePressed) > PWRBUTTONDELAY) & (HAL_GPIO_ReadPin(PWRBUTTON_GPIO_Port,PWRBUTTON_Pin) == GPIO_PIN_SET)) {
				UserPowerButton = 0;
				xTaskNotify(xDisplayNotify,0x01,eSetValueWithOverwrite);


				if (SDcardMounted) {
					f_mount(0, "0:", 1); //unmount SDCARD
				}
				osDelay(5000);
				while(HAL_GPIO_ReadPin(PWRBUTTON_GPIO_Port,PWRBUTTON_Pin) == GPIO_PIN_SET) { //wait for button to be released
				}

				StandbyMode();
			}
			if (HAL_GPIO_ReadPin(PWRBUTTON_GPIO_Port,PWRBUTTON_Pin) == GPIO_PIN_RESET) {
				UserPowerButton = 0;
			}

		}
		if (UserOkButton) {

			osDelay(20); //debouncer
			if (((xTaskGetTickCount() - OkButtonTimePressed) > 2000) & (HAL_GPIO_ReadPin(BTN_OK_GPIO_Port,BTN_OK_Pin) == GPIO_PIN_SET)) {
				UserOkButton = 0;
				if (activity.flightstatus == FLS_FLYING) {
					activity.flightstatus = FLS_LANDED;
				} else {
					activity.flightstatus = FLS_TAKEOFF;
				}
			}

			if (HAL_GPIO_ReadPin(BTN_OK_GPIO_Port,BTN_OK_Pin) == GPIO_PIN_RESET) {
				UserOkButton = 0;
			}

		}

		if (UserCancelButton) {
			osDelay(20);

			if (((xTaskGetTickCount() - CancelButtonTimePressed) > 4000) & (HAL_GPIO_ReadPin(BTN_CANCEL_GPIO_Port,BTN_CANCEL_Pin) == GPIO_PIN_SET)) {
				__disable_irq();
				NVIC_SystemReset();
				while( 1 ) {
				        __nop();
				    }
			}

			if (HAL_GPIO_ReadPin(BTN_CANCEL_GPIO_Port,BTN_CANCEL_Pin) == GPIO_PIN_RESET) {
				UserCancelButton = 0;
			}

		}

		if (UserNextButton) {
			osDelay(20);

			if (((xTaskGetTickCount() - NextButtonTimePressed) > 2000) & (HAL_GPIO_ReadPin(BTN_NEXT_GPIO_Port,BTN_NEXT_Pin) == GPIO_PIN_SET)) {
				UserNextButton = 0;
				activity.muted = (activity.muted == 0) ? 1 : 0;
			}

			if (HAL_GPIO_ReadPin(BTN_NEXT_GPIO_Port,BTN_NEXT_Pin) == GPIO_PIN_RESET) {
				UserNextButton = 0;
			}
		}

		if (UserPrevButton) {
			osDelay(20);
			if (((xTaskGetTickCount() - PrevButtonTimePressed) > 2000) & (HAL_GPIO_ReadPin(BTN_PREV_GPIO_Port,BTN_PREV_Pin) == GPIO_PIN_SET)) {
				UserPrevButton = 0;
				activity.useKalman = (activity.useKalman == 0) ? 1 : 0;
			}

			if (HAL_GPIO_ReadPin(BTN_PREV_GPIO_Port,BTN_PREV_Pin) == GPIO_PIN_RESET) {
				UserPrevButton = 0;
			}
		}

#ifdef SETRTCBYGPS
		if (hgps.fix > 0 && !HasSetTime ) {
		    if ( !HasSetTime ) {

			setRTCFromHgps(&hgps); //util.c
			HasSetTime=1;
		    }
		}
#endif
		if (activity.flightstatus == FLS_GROUND) {
			if (hgps.fix > 0 && hgps.is_valid) {
				if (hgps.speed > TAKEOFFSPEED && sensors.barotakeoff) {
					activity.flightstatus = FLS_TAKEOFF;
				}
			}else {
				//Baro takeoff detected
				if(sensors.barotakeoff){
					activity.flightstatus = FLS_TAKEOFF;
				}
			}
		}


		//Flight Operations

		switch (activity.flightstatus) {
		case FLS_TAKEOFF:
			activity.currentLogID = conf.lastLogNumber + 1;
			activity.takeoffLocationLAT = (int32_t) (hgps.latitude * 1000000);
			activity.takeoffLocationLON = (int32_t) (hgps.longitude * 1000000);
			activity.takeoffTemp = sensors.temperature;
			setActivityTakeoffTime(&activity); //util.c
			activity.flightstatus = FLS_FLYING;
			xTaskNotify(xLogDataNotify, 0x01, eSetValueWithOverwrite);
			landedcheck = xTaskGetTickCount();
			break;

		case FLS_FLYING:
			if (sensors.AltitudeMeters > activity.MaxAltitudeMeters)
				activity.MaxAltitudeMeters = sensors.AltitudeMeters;
			if (sensors.VarioMs > activity.MaxVarioMs)
				activity.MaxVarioMs = sensors.VarioMs;
			if (sensors.VarioMs < activity.MaxVarioSinkMs)
				activity.MaxVarioSinkMs = sensors.VarioMs;

			//Landed detection
			if (hgps.fix > 0) {
				if (hgps.speed > LANDEDSPEED) {
					landedcheck = xTaskGetTickCount();
				}
			} else {
				landedcheck = xTaskGetTickCount();
			}

			if (xTaskGetTickCount() - landedcheck > LANDEDLOWSPEEDTIME) {
				activity.flightstatus = FLS_LANDED;
			}

			break;

		case FLS_LANDED:
			activity.landingAltitude = sensors.AltitudeMeters;
			activity.landingLocationLAT = (int32_t) (hgps.latitude * 1000000);
			activity.landingLocationLON = (int32_t) (hgps.longitude * 1000000);
			activity.MaxAltitudeGainedMeters = activity.MaxAltitudeMeters
					- activity.takeoffAltitude;
			setActivityLandTime(&activity); //util.c
			activity.flightstatus = FLS_GROUND;
			conf.lastLogNumber = activity.currentLogID;
			saveConfigtoSD();
			xTaskNotify(xLogDataNotify, 0x03, eSetValueWithOverwrite);
			break;

		}

		if (maintenanceloop > 600) { // = 1 minute
			maintenanceloop = 0;

			if (hgps.fix > 0 && hgps.is_valid && hgps.sats_in_use > 8) { //have accurate fix
			    activity.barognssdeveation = sensors.AltitudeMeters - (hgps.altitude *1000) ;
			    activity.barognssavalid = 1;

			}else{
			     activity.barognssavalid = 0;
			}

		}

		osDelay(100);
	}

}

