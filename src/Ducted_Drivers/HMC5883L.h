/*
 * HMC5883L.h
 *
 *  Created on: 29/feb/2016
 *      Author: pc asus
 */

#include "MPU6050.h"

#ifndef SRC_DUCTED_DRIVERS_HMC5883L_H_
#define SRC_DUCTED_DRIVERS_HMC5883L_H_

// Define Costants/Registers
#define HMC5883L_ADDRESS 			 	0x1E

#ifdef MPU6050_I2C_AUX_MASTER
//This registers are referred to MPU6050 (use the MPU6050_address)
#define HMC5883L_DO_X_MSB_REG			0x49
#define HMC5883L_DO_X_LSB_REG			0x4A
#define HMC5883L_DO_Z_MSB_REG			0x4B//DATASHEET DICE Z... MA SEMO SICURI?
#define HMC5883L_DO_Z_LSB_REG			0x4C
#define HMC5883L_DO_Y_MSB_REG			0x4D
#define HMC5883L_DO_Y_LSB_REG			0x4E

#else

#define HMC5883L_CFG_REG_A				0x00
#define HMC5883L_CFG_REG_B				0x01
#define HMC5883L_MODE_REG				0x02
#define HMC5883L_DO_X_MSB_REG			0x03
#define HMC5883L_DO_X_LSB_REG			0x04
#define HMC5883L_DO_Z_MSB_REG			0x05//DATASHEET DICE Z... MA SEMO SICURI?
#define HMC5883L_DO_Z_LSB_REG			0x06
#define HMC5883L_DO_Y_MSB_REG			0x07
#define HMC5883L_DO_Y_LSB_REG			0x08
#define HMC5883L_STATUS_REG				0x09
#define HMC5883L_ID_REG_A				0x0A
#define HMC5883L_ID_REG_B				0x0B
#define HMC5883L_ID_REG_C				0x0C
#endif /*MPU6050_I2C_AUXMASTER*/

#define HMC5883L_FULL_SCALE_RANGE		8 //gauss
#define HMC5883L_MG_PER_DIGIT		  	0.92 // Digital resolution???
//QL CHE ROBA E'?


// Define Variable
static float MAG_XOUT_OFFSET = 0;
static float MAG_YOUT_OFFSET = 0;
static float MAG_ZOUT_OFFSET = 0;

// Define Prototype
void Get_Mag_Value(int* MAG_XOUT, int* MAG_YOUT, int* MAG_ZOUT);
void Get_Mag_Value_Normalized(float* MAG_X_NORM, float* MAG_Y_NORM, float* MAG_Z_NORM);
//void Calibrate_Mag(void);//da vedere se si può fare prima


#endif /* SRC_DUCTED_DRIVERS_HMC5883L_H_ */
