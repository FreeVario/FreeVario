/*
 * datalog.h
 *
 *  Created on: Dec 9, 2018
 *      Author: marco
 */

#ifndef DATALOG_H_
#define DATALOG_H_
#include "../fvconfig.h"
#include "gps.h"
#include <globaldata.h>

extern ActivityData activity;
extern gps_t  hgps;
extern SensorData sensors;
void writeFlightLogSummaryFile();
int openDataLogFile(FIL * logFile);
void writeDataLogFile(FIL  * logFile);
void closeDataLogFile(FIL  * logFile);

#endif /* DATALOG_H_ */
