/*
 * Kalman.h
 *
 *  Created on: 25/feb/2016
 *      Author: pc asus
 */

#ifndef SRC_DUCTED_DRIVERS_KALMAN_H_
#define SRC_DUCTED_DRIVERS_KALMAN_H_

typedef struct{
    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    float K[2]; // Kalman gain - This is a 2x1 vector
    float y; // Angle difference
    float S; // Estimate error
} KALMAN;

void init_Kalman(void);
float getAngle(float newAngle, float newRate, float dt, KALMAN* Kalman);
float getRate(KALMAN Kalman);
void setAngle(float newAngle, KALMAN* Kalman);

#endif /* SRC_DUCTED_DRIVERS_KALMAN_H_ */
