/*
 * gpx.h
 *
 *  Created on: May 26, 2019
 *      Author: marco
 */

#ifndef FREEVARIO_GPX_H_
#define FREEVARIO_GPX_H_

#include "fatfs.h"


int openGPXLogFile(FIL* gpxFile);
void writeGPXLogFile(FIL *gpxFile);
void closeGPXLogFile(FIL *gpxFile);

#endif /* FREEVARIO_GPX_H_ */


