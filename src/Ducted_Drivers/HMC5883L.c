/*
 * HMC5883L.c
 *
 *  Created on: 29/feb/2016
 *      Author: pc asus
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "r_riic_rx600.h"
#include "r_riic_rx600_master.h"
#include "IMU_write_read.h"
#include "CMT.h"
#include "I2C.h"
#include "support_functions.h"
#include "HMC5883L.h"


void Get_Mag_Value(int* MAG_XOUT, int* MAG_YOUT, int* MAG_ZOUT)
{
	uint8_t MAG_XOUT_H = 0, MAG_XOUT_L = 0;
	uint8_t MAG_YOUT_H = 0, MAG_YOUT_L = 0;
	uint8_t MAG_ZOUT_H = 0, MAG_ZOUT_L = 0;

	IMU_read(CHANNEL, MPU6050_ADDRESS, HMC5883L_DO_X_MSB_REG, &MAG_XOUT_H, NUM_BYTES);
	IMU_read(CHANNEL, MPU6050_ADDRESS, HMC5883L_DO_X_LSB_REG, &MAG_XOUT_L, NUM_BYTES);
	IMU_read(CHANNEL, MPU6050_ADDRESS, HMC5883L_DO_Y_MSB_REG, &MAG_YOUT_H, NUM_BYTES);
	IMU_read(CHANNEL, MPU6050_ADDRESS, HMC5883L_DO_Y_LSB_REG, &MAG_YOUT_L, NUM_BYTES);
	IMU_read(CHANNEL, MPU6050_ADDRESS, HMC5883L_DO_Z_MSB_REG, &MAG_ZOUT_H, NUM_BYTES);
	IMU_read(CHANNEL, MPU6050_ADDRESS, HMC5883L_DO_Z_LSB_REG, &MAG_ZOUT_L, NUM_BYTES);

	*MAG_XOUT = (((MAG_XOUT_H<<8)|MAG_XOUT_L) - MAG_XOUT_OFFSET);//*HMC5883L_MG_PER_DIGIT;
	*MAG_YOUT = (((MAG_YOUT_H<<8)|MAG_YOUT_L) - MAG_YOUT_OFFSET);//*HMC5883L_MG_PER_DIGIT;
	*MAG_ZOUT = (((MAG_ZOUT_H<<8)|MAG_ZOUT_L) - MAG_ZOUT_OFFSET);//*HMC5883L_MG_PER_DIGIT;

}

void Get_Mag_Value_Normalized(float* MAG_X_NORM, float* MAG_Y_NORM, float* MAG_Z_NORM)
{
	int MAG_X_OUT;
	int MAG_Y_OUT;
	int MAG_Z_OUT;

	Get_Mag_Value(&MAG_X_OUT, &MAG_Y_OUT, &MAG_Z_OUT);

	*MAG_X_NORM = map(C2toD(MAG_X_OUT), -QL/2, QL/2, -HMC5883L_FULL_SCALE_RANGE, HMC5883L_FULL_SCALE_RANGE);
	*MAG_Y_NORM = map(C2toD(MAG_Y_OUT), -QL/2, QL/2, -HMC5883L_FULL_SCALE_RANGE, HMC5883L_FULL_SCALE_RANGE);
	*MAG_Z_NORM = map(C2toD(MAG_Z_OUT), -QL/2, QL/2, -HMC5883L_FULL_SCALE_RANGE, HMC5883L_FULL_SCALE_RANGE);

}
