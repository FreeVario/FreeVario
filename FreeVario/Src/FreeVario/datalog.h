/*
 FreeVario http://FreeVario.org

 Copyright (c), FreeVario (http://freevario.org)
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#ifndef DATALOG_H_
#define DATALOG_H_
#include "../fvconfig.h"
#include "gps.h"

void writeFlightLogSummaryFile();
int openDataLogFile(FIL * logFile);
void writeDataLogFile(FIL * logFile);
void closeDataLogFile(FIL * logFile);
//FRESULT set_timestamp(char * obj);





#endif /* DATALOG_H_ */
