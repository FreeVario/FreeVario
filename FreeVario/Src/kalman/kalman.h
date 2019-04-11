/*
 * kalman.h
 *
 *  Created on: Apr 11, 2019
 *      Author: marco
 */

#ifndef KALMAN_KALMAN_H_
#define KALMAN_KALMAN_H_



void KalmanFilter_Configure(float zVariance, float zAccelVariance, float zAccelBiasVariance, float zInitial, float vInitial, float aBiasInitial);
void KalmanFilter_Update(float z, float a, float dt, float* pZ, float* pV);


#endif /* KALMAN_KALMAN_H_ */
