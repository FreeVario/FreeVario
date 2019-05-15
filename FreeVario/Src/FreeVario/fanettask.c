/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#include "fanettask.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../fvconfig.h"
#include <stdlib.h>
#include "SX1278.h"

extern SPI_HandleTypeDef FV_LoRa_SPI;
SX1278_hw_t SX1278_hw;
SX1278_t SX1278;

/*
 * NOT YET FANET IMPLEMENTATION
 * This is a test implementation
 * to simulate a FLARM radio with
 * a private data structure. The fanet
 * data structure will be implemented
 * at a later stage.
 */

void StartFanetTask(void const * argument) {

    int ret;

    char buffer[64];

    int message = 0;
    int message_length;

    //initialize hardware for LoRa module
    SX1278_hw.dio0.port = SX_INT_GPIO_Port;
    SX1278_hw.dio0.pin = SX_INT_Pin;
    SX1278_hw.nss.port = SX_NSS_GPIO_Port;
    SX1278_hw.nss.pin = SX_NSS_Pin;
    SX1278_hw.reset.port = SX_RST_GPIO_Port;
    SX1278_hw.reset.pin = SX_RST_Pin;
    SX1278_hw.spi = &FV_LoRa_SPI;

    //initialize logic for LoRa module
    SX1278.hw = &SX1278_hw;
    //SX1278_sleep(&SX1278);
    //configure module
    SX1278_begin(&SX1278, SX1278_433MHZ, SX1278_POWER_20DBM, SX1278_LORA_SF_8,
    SX1278_LORA_BW_20_8KHZ, 10);

    SX1278_LoRaEntryTx(&SX1278, 16, 2000);
    SX1278_LoRaEntryRx(&SX1278, 16, 2000);

    /* Infinite loop */
    for (;;) {

        osDelay(3000);

        message_length = sprintf(buffer, "Hello %d", message);
        ret = SX1278_LoRaEntryTx(&SX1278, message_length, 2000);
       // printf("Entry: %d\r\n", ret);
        osDelay(100);
        //printf("Sending %s\r\n", buffer);
        ret = SX1278_LoRaTxPacket(&SX1278, (uint8_t *) buffer, message_length, 2000);
        message += 1;



/*


        ret = SX1278_LoRaRxPacket(&SX1278);
        printf("Received: %d\r\n", ret);
        if (ret > 0) {
            SX1278_read(&SX1278, (uint8_t *) buffer, ret);
            CDC_Transmit_FS((uint8_t *) &buffer, 64);

        }

*/
    }

}
