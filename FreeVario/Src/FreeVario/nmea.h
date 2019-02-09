/*
 FreeVario http://FreeVario.org

  Copyright (c), PrimalCode (http://www.primalcode.org)
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  any later version. see <http://www.gnu.org/licenses/>
*/

#ifndef NMEA_H_
#define NMEA_H_
#include "../fvconfig.h"

void NMEA_getPTAS1(uint8_t * buffer, int32_t vario, int32_t varioAv, int32_t altitude);
void NMEA_getnmeaShortLXWP0(uint8_t * buffer, int32_t varioAlt, int32_t varioMts);
void  NMEA_getNmeaLK8EX1(uint8_t * buffer, int32_t rawPressure, int32_t varioAlt, int32_t climbRate, int32_t temperature, int32_t pbat);
void NMEA_getNmeaPcProbe(uint8_t * buffer,int16_t aax, int16_t aay, int16_t aaz, int16_t temperature, int16_t humidity, uint8_t batPers, uint8_t charging);
void getCRC(char * buff);
void appendCharNum(char * buff, int32_t value, uint16_t div, uint8_t type);



#endif /* NMEA_H_ */
