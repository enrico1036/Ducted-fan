#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <machine.h>
#include <math.h>
#include "r_riic_rx600.h"
#include "r_riic_rx600_master.h"
#include "MPU6050.h"
#include "IMU_write_read.h"
#include "CMT.h"
#include "I2C.h"
#include "support_functions.h"

uint8_t DISABLE = 0x00; 			//DISABLE FUNCTION
uint8_t INITIAL = 0x00;		 		//INITIALIZE TO ZERO
uint8_t ENABLE = 0x00; 				//ENABLE FUNCTION
uint8_t RATE_1000 = 0x07; 			//Sets sample rate to 8000/1+7 = 1000Hz
uint8_t D_GYRO_SELF_TEST = 0x08; 	//Disable gyro self test
uint8_t GYRO_CLOCK = 0x02; 			//Gyro clock reference

// Magnetometer register
uint8_t CONFIG_6A = 0x20;
uint8_t CONFIG_MST_CTR = 0x0D;
uint8_t CONFIG_SLV0_ADDR_R = 0x80 | 0x1E;
uint8_t CONFIG_SLV0_ADDR_W = 0x1E;
uint8_t CONFIG_SLV0_REG_DATA = 0x03; // per ora è il registro di configurazione A, poi sarà solo il 0x03 (primo data output register)
uint8_t CONFIG_SLV0_REG_Mode_Register = 0x02;
uint8_t CONFIG_SLV0_CTRL_1 = 0x01;//len=1
uint8_t CONFIG_SLV0_CTRL_2 = 0x8D;//len=13
uint8_t CONFIG_SLV0_CTRL_Enable = 0x81;
uint8_t CONFIG_INT_PIN_CFG = 0x02;


//Function to test correct communication
void MPU6050_Test_I2C()
{
	unsigned char Data = 0x00;
	int iteration=0;
	riic_ret_t ret = IMU_read(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, &Data, NUM_BYTES);

    while (Data != 0x68){
    	riic_ret_t iic_ret = RIIC_OK;
    	iic_ret |= riic_master_init();
    	while (RIIC_OK != iic_ret)
    	{
    		nop(); //Failure to initialize: demo cannot proceed.
    	}
    	ret = IMU_read(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, &Data, NUM_BYTES);
    	lcd_display(LCD_LINE3, "  I2C_INIT  ");
    	char a[20];
    	sprintf(a, "Add: 0x%2.0f", Data);
    	lcd_display(LCD_LINE4, (uint8_t*)a);
    	sprintf(a, "Iter: %d", iteration);
    	lcd_display(LCD_LINE5, (uint8_t*)a);
    	iteration++;
    }
/*
    if(Data == 0x68)
    {
        printf("\nI2C Read Test Passed, MPU6050 Address: 0x%x", Data);

    }
    else
    {
        printf("ERROR: I2C Read Test Failed, Stopping 0x%x", Data);
        while(1){
        	LED4=~LED4;
        }
    }
*/
}




void Setup_MPU6050()
{
    //Sets sample rate to 8000/1+7 = 1000Hz
	IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, &RATE_1000, NUM_BYTES);
    //Disable FSync, 256Hz DLPF
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_CONFIG, &DISABLE, NUM_BYTES);
    //Disable gyro self tests, scale of 500 degrees/s
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, &D_GYRO_SELF_TEST, NUM_BYTES);
    //Disable accel self tests, scale of +-2g, no DHPF
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, &DISABLE, NUM_BYTES);
    //Freefall threshold of |0mg|
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_FF_THR, &INITIAL, NUM_BYTES);
    //Freefall duration limit of 0
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_FF_DUR, &INITIAL, NUM_BYTES);
    //Motion threshold of 0mg
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_MOT_THR, &INITIAL, NUM_BYTES);
    //Motion duration of 0s
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_MOT_DUR, &INITIAL, NUM_BYTES);
    //Zero motion threshold
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_ZRMOT_THR, &INITIAL, NUM_BYTES);
    //Zero motion duration threshold
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_ZRMOT_DUR, &INITIAL, NUM_BYTES);
    //Disable sensor output to FIFO buffer
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, &DISABLE, NUM_BYTES);

    //AUX I2C setup
    //Sets AUX I2C to single master control, plus other config
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, &INITIAL, NUM_BYTES);
    //Setup AUX I2C slaves
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_REG, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_ADDR, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_REG, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_CTRL, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_ADDR, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_REG, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_CTRL, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_ADDR, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_REG, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_CTRL, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_ADDR, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_REG, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DO, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_CTRL, &INITIAL, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DI, &INITIAL, NUM_BYTES);

   //Setup magnetometer HMC5883L as external sensor of MPU6050
   //Set register mode in continous-measurement

   //I2C_MST_EN = 1 	I2C master mode enabled (MPU6050 Register Map pag. 38)
   IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, &CONFIG_6A, NUM_BYTES);
   //Set the MPU6050 as the master I2C rate (400kHz)
   IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, &CONFIG_MST_CTR, NUM_BYTES);
   //Configure the address used to specify the I2C slave address of Slave 0 (magnetometer): write mode
   IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, &CONFIG_SLV0_ADDR_W, NUM_BYTES);
   //Data transfer starts from this register within Slave 0
   IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_REG , &CONFIG_SLV0_REG_Mode_Register, NUM_BYTES);
   //This register holds the output data written into Slave 0 when Slave 0 is set to write mode
   IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_DO, &DISABLE, NUM_BYTES);
   //Set the data operation number
   IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, &CONFIG_SLV0_CTRL_1, NUM_BYTES);
   //Enable sub equipment operation
   IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, &CONFIG_SLV0_CTRL_Enable, NUM_BYTES);

   //Set the read mode
   //Configure the address used to specify the I2C slave address of Slave 0 (magnetometer): read mode
   IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, &CONFIG_SLV0_ADDR_R, NUM_BYTES);
   //Data transfer starts from this register within Slave 0
   IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_REG , &CONFIG_SLV0_REG_DATA, NUM_BYTES);
   //Enable sub equipment operation
   IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, &CONFIG_SLV0_CTRL_2, NUM_BYTES);
   //Enable data ready interrupt
   IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, &ENABLE, NUM_BYTES);


    //MPU6050_RA_I2C_MST_STATUS //Read-only
    //Setup INT pin and AUX I2C pass through
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, &INITIAL, NUM_BYTES);
    //Enable data ready interrupt
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, &ENABLE, NUM_BYTES);

    //MPU6050_RA_DMP_INT_STATUS        //Read-only
    //MPU6050_RA_INT_STATUS 3A        //Read-only
    //MPU6050_RA_ACCEL_XOUT_H         //Read-only
    //MPU6050_RA_ACCEL_XOUT_L         //Read-only
    //MPU6050_RA_ACCEL_YOUT_H         //Read-only
    //MPU6050_RA_ACCEL_YOUT_L         //Read-only
    //MPU6050_RA_ACCEL_ZOUT_H         //Read-only
    //MPU6050_RA_ACCEL_ZOUT_L         //Read-only
    //MPU6050_RA_TEMP_OUT_H         //Read-only
    //MPU6050_RA_TEMP_OUT_L         //Read-only
    //MPU6050_RA_GYRO_XOUT_H         //Read-only
    //MPU6050_RA_GYRO_XOUT_L         //Read-only
    //MPU6050_RA_GYRO_YOUT_H         //Read-only
    //MPU6050_RA_GYRO_YOUT_L         //Read-only
    //MPU6050_RA_GYRO_ZOUT_H         //Read-only
    //MPU6050_RA_GYRO_ZOUT_L         //Read-only
    //MPU6050_RA_EXT_SENS_DATA_00     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_01     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_02     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_03     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_04     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_05     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_06     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_07     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_08     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_09     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_10     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_11     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_12     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_13     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_14     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_15     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_16     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_17     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_18     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_19     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_20     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_21     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_22     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_23     //Read-only
    //MPU6050_RA_MOT_DETECT_STATUS     //Read-only

    //Slave out, don't care
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_DO, &DISABLE, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_DO, &DISABLE, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_DO, &DISABLE, NUM_BYTES);
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_DO, &DISABLE, NUM_BYTES);
    //More slave config
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_I2C_MST_DELAY_CTRL, &INITIAL, NUM_BYTES);
    //Reset sensor signal paths
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, &INITIAL, NUM_BYTES);
    //Motion detection control
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL, &ENABLE, NUM_BYTES);
    //Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, &DISABLE, NUM_BYTES);
    //Sets clock source to gyro reference w/ PLL
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, &GYRO_CLOCK, NUM_BYTES);
    //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, &INITIAL, NUM_BYTES);
    //MPU6050_RA_BANK_SEL              //Not in datasheet
    //MPU6050_RA_MEM_START_ADDR        //Not in datasheet
    //MPU6050_RA_MEM_R_W               //Not in datasheet
    //MPU6050_RA_DMP_CFG_1             //Not in datasheet
    //MPU6050_RA_DMP_CFG_2             //Not in datasheet
    //MPU6050_RA_FIFO_COUNTH           //Read-only
    //MPU6050_RA_FIFO_COUNTL           //Read-only

    //Data transfer to and from the FIFO buffer
    IMU_write(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, &ENABLE, NUM_BYTES);
    //MPU6050_RA_WHO_AM_I              //Read-only, I2C address

    printf("\nMPU6050 Setup Complete");
}

void Calibrate_Acc(){
	uint32_t ACC_YOUT_OFFSET_ACC_MEASURES_SUM = 0;

	float ACC_XOUT_init;
	float ACC_YOUT_init;
	float ACC_ZOUT_init;

	float ACC_YOUT_OFFSET_MAP;

	for(int x = 0; x<ACC_MEASURES; x++)
	{
		Get_Accel_Gravity_Power(&ACC_XOUT_init, &ACC_YOUT_init, &ACC_ZOUT_init);
		ACC_YOUT_OFFSET_ACC_MEASURES_SUM += ACC_YOUT_init;
		ms_delay(1);
	}

	ACC_YOUT_OFFSET = (float)ACC_YOUT_OFFSET_ACC_MEASURES_SUM/ACC_MEASURES;
	ACC_YOUT_OFFSET_MAP = map(C2toD(ACC_YOUT_OFFSET), -QL/2, QL/2, -MPU6050_ACC_FULL_SCALE_RANGE, MPU6050_ACC_FULL_SCALE_RANGE);
	if (abs(ACC_YOUT_OFFSET_MAP)-1<0.1)
	{
		lcd_display(LCD_LINE7, "   DRITTO   ");
		char a[20];
		sprintf(a, "ACCY: %4.2f", ACC_YOUT_OFFSET_MAP);
		lcd_display(LCD_LINE8, (uint8_t*)a);
	}
	else
	{
		char a[20];
		sprintf(a, "ACCY: %4.2f", ACC_YOUT_OFFSET_MAP);
		lcd_display(LCD_LINE8, (uint8_t*)a);
	}
}


//Gets raw accelerometer data, performs no processing
void Get_Accel_Values(int* ACCEL_XOUT, int* ACCEL_YOUT, int* ACCEL_ZOUT)
{
	uint8_t ACCEL_XOUT_H = 0, ACCEL_XOUT_L = 0;
	uint8_t ACCEL_YOUT_H = 0, ACCEL_YOUT_L = 0;
	uint8_t ACCEL_ZOUT_H = 0, ACCEL_ZOUT_L = 0;

	IMU_read(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, &ACCEL_XOUT_H, NUM_BYTES);
	IMU_read(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_L, &ACCEL_XOUT_L, NUM_BYTES);
	IMU_read(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, &ACCEL_YOUT_H, NUM_BYTES);
	IMU_read(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_L, &ACCEL_YOUT_L, NUM_BYTES);
	IMU_read(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, &ACCEL_ZOUT_H, NUM_BYTES);
	IMU_read(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &ACCEL_ZOUT_L, NUM_BYTES);

	*ACCEL_XOUT = ((ACCEL_XOUT_H<<8)|ACCEL_XOUT_L);
	*ACCEL_YOUT = ((ACCEL_YOUT_H<<8)|ACCEL_YOUT_L);
	*ACCEL_ZOUT = ((ACCEL_ZOUT_H<<8)|ACCEL_ZOUT_L);
}
//Accelerometer data in g
void Get_Accel_Gravity_Power(float* ACCEL_XPOWER, float* ACCEL_YPOWER, float* ACCEL_ZPOWER)
{
	int ACCEL_XOUT;
	int	ACCEL_YOUT;
	int	ACCEL_ZOUT;

	Get_Accel_Values(&ACCEL_XOUT, &ACCEL_YOUT, &ACCEL_ZOUT);

	*ACCEL_XPOWER = map(C2toD(ACCEL_XOUT), -QL/2, QL/2, -MPU6050_ACC_FULL_SCALE_RANGE, MPU6050_ACC_FULL_SCALE_RANGE) - ACC_XOUT_OFFSET;
	*ACCEL_YPOWER = map(C2toD(ACCEL_YOUT), -QL/2, QL/2, -MPU6050_ACC_FULL_SCALE_RANGE, MPU6050_ACC_FULL_SCALE_RANGE) - ACC_YOUT_OFFSET;
	*ACCEL_ZPOWER = map(C2toD(ACCEL_ZOUT), -QL/2, QL/2, -MPU6050_ACC_FULL_SCALE_RANGE, MPU6050_ACC_FULL_SCALE_RANGE) - ACC_ZOUT_OFFSET;
}

//Converts the already acquired accelerometer data into 3D euler angles
void Get_Accel_Angles(float* ACCEL_XANGLE, float* ACCEL_YANGLE)
{
	int ACCEL_XOUT;
	int	ACCEL_YOUT;
	int	ACCEL_ZOUT;

	Get_Accel_Values(&ACCEL_XOUT, &ACCEL_YOUT, &ACCEL_ZOUT);

//	*ACCEL_XANGLE = 57.295*atan((float)C2toD(ACCEL_YOUT)/sqrt(pow((float)C2toD(ACCEL_ZOUT),2)+pow((float)C2toD(ACCEL_XOUT),2)));
//	*ACCEL_YANGLE = 57.295*atan((float)-C2toD(ACCEL_XOUT)/sqrt(pow((float)C2toD(ACCEL_ZOUT),2)+pow((float)C2toD(ACCEL_YOUT),2)));
	*ACCEL_XANGLE = 57.295*atan2(-(float)C2toD(ACCEL_YOUT), (float)C2toD(ACCEL_ZOUT));
	*ACCEL_YANGLE = 57.295*atan2((float)C2toD(ACCEL_XOUT), sqrt(pow((float)C2toD(ACCEL_YOUT),2)+pow((float)C2toD(ACCEL_ZOUT),2)));
}

// calibration of the gyroscope with the average of the first GYRO_MEASURES readings
void Calibrate_Gyros()
{
	uint32_t GYRO_XOUT_OFFSET_GYRO_MEASURES_SUM = 0;
	uint32_t GYRO_YOUT_OFFSET_GYRO_MEASURES_SUM = 0;
	uint32_t GYRO_ZOUT_OFFSET_GYRO_MEASURES_SUM = 0;

	int GYRO_XOUT_init;
	int GYRO_YOUT_init;
	int GYRO_ZOUT_init;

	float GYRO_XOUT_OFFSET0 = 0;
	float GYRO_YOUT_OFFSET0 = 0;
	float GYRO_ZOUT_OFFSET0 = 0;


	float GYRO_XOUT_OFFSET_MAP;
	float GYRO_YOUT_OFFSET_MAP;
	float GYRO_ZOUT_OFFSET_MAP;

	// int iteration = 1;
	do
		{
			GYRO_XOUT_OFFSET0 = GYRO_XOUT_OFFSET_MAP;
			GYRO_YOUT_OFFSET0 = GYRO_YOUT_OFFSET_MAP;
			GYRO_ZOUT_OFFSET0 = GYRO_ZOUT_OFFSET_MAP;

			// lcd_display (LCD_LINE3, " TestOffset ");

			for(int x = 0; x<GYRO_MEASURES; x++)
			{
				Get_Gyro_Value(&GYRO_XOUT_init, &GYRO_YOUT_init, &GYRO_ZOUT_init);

				GYRO_XOUT_OFFSET_GYRO_MEASURES_SUM += GYRO_XOUT_init;
				GYRO_YOUT_OFFSET_GYRO_MEASURES_SUM += GYRO_YOUT_init;
				GYRO_ZOUT_OFFSET_GYRO_MEASURES_SUM += GYRO_ZOUT_init;

				ms_delay(1);
			}

		GYRO_XOUT_OFFSET = (float)GYRO_XOUT_OFFSET_GYRO_MEASURES_SUM/GYRO_MEASURES;
		GYRO_YOUT_OFFSET = (float)GYRO_YOUT_OFFSET_GYRO_MEASURES_SUM/GYRO_MEASURES;
		GYRO_ZOUT_OFFSET = (float)GYRO_ZOUT_OFFSET_GYRO_MEASURES_SUM/GYRO_MEASURES;

		GYRO_XOUT_OFFSET_MAP = map(C2toD(GYRO_XOUT_OFFSET), -QL/2, QL/2, -MPU6050_GYROC_FULL_SCALE_RANGE, MPU6050_GYROC_FULL_SCALE_RANGE);
		GYRO_YOUT_OFFSET_MAP = map(C2toD(GYRO_YOUT_OFFSET), -QL/2, QL/2, -MPU6050_GYROC_FULL_SCALE_RANGE, MPU6050_GYROC_FULL_SCALE_RANGE);
		GYRO_ZOUT_OFFSET_MAP = map(C2toD(GYRO_ZOUT_OFFSET), -QL/2, QL/2, -MPU6050_GYROC_FULL_SCALE_RANGE, MPU6050_GYROC_FULL_SCALE_RANGE);


/*
		char a[20];
		sprintf(a, "X: %4.2f", GYRO_XOUT_OFFSET_MAP-GYRO_XOUT_OFFSET);
		lcd_display(LCD_LINE4, (uint32_t*)a);
		sprintf(a, "Y: %4.2f", GYRO_YOUT_OFFSET_MAP-GYRO_YOUT_OFFSET);
		lcd_display(LCD_LINE5, (uint32_t*)a);
		sprintf(a, "Z: %4.2f", GYRO_ZOUT_OFFSET_MAP-GYRO_ZOUT_OFFSET);
		lcd_display(LCD_LINE6, (uint32_t*)a);
		sprintf(a, "iter: %2d", iteration);
		lcd_display(LCD_LINE7, (uint32_t*)a);

		ms_delay(1000);

		iteration++;
*/

	} while((abs(GYRO_XOUT_OFFSET0 - GYRO_XOUT_OFFSET_MAP) > OFFSET_ERROR) || (abs(GYRO_YOUT_OFFSET0 - GYRO_YOUT_OFFSET_MAP) > OFFSET_ERROR) || (abs(GYRO_ZOUT_OFFSET0 - GYRO_ZOUT_OFFSET_MAP) > OFFSET_ERROR));
}

//Function to read the gyroscope rate data
void Get_Gyro_Value(int* GYRO_XOUT, int* GYRO_YOUT, int* GYRO_ZOUT)
{
	uint8_t GYRO_XOUT_H = 0, GYRO_XOUT_L = 0;
	uint8_t GYRO_YOUT_H = 0, GYRO_YOUT_L = 0;
	uint8_t GYRO_ZOUT_H = 0, GYRO_ZOUT_L = 0;

	IMU_read(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, &GYRO_XOUT_H, NUM_BYTES);
	IMU_read(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_L, &GYRO_XOUT_L, NUM_BYTES);
	IMU_read(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H, &GYRO_YOUT_H, NUM_BYTES);
	IMU_read(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_L, &GYRO_YOUT_L, NUM_BYTES);
	IMU_read(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, &GYRO_ZOUT_H, NUM_BYTES);
	IMU_read(CHANNEL, MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &GYRO_ZOUT_L, NUM_BYTES);

	*GYRO_XOUT = ((GYRO_XOUT_H<<8)|GYRO_XOUT_L) - GYRO_XOUT_OFFSET;
	*GYRO_YOUT = ((GYRO_YOUT_H<<8)|GYRO_YOUT_L) - GYRO_YOUT_OFFSET;
	*GYRO_ZOUT = ((GYRO_ZOUT_H<<8)|GYRO_ZOUT_L) - GYRO_ZOUT_OFFSET;
}

//Function to convert the gyroscope rate data read into degrees/s
void Get_Gyro_Rates(float* GYRO_XRATE, float* GYRO_YRATE, float* GYRO_ZRATE)
{
	int GYRO_XOUT;
	int GYRO_YOUT;
	int GYRO_ZOUT;

	Get_Gyro_Value(&GYRO_XOUT, &GYRO_YOUT, &GYRO_ZOUT);

	*GYRO_XRATE = map(C2toD(GYRO_XOUT), -QL/2, QL/2, -MPU6050_GYROC_FULL_SCALE_RANGE, MPU6050_GYROC_FULL_SCALE_RANGE);
	*GYRO_YRATE = map(C2toD(GYRO_YOUT), -QL/2, QL/2, -MPU6050_GYROC_FULL_SCALE_RANGE, MPU6050_GYROC_FULL_SCALE_RANGE);
	*GYRO_ZRATE = map(C2toD(GYRO_ZOUT), -QL/2, QL/2, -MPU6050_GYROC_FULL_SCALE_RANGE, MPU6050_GYROC_FULL_SCALE_RANGE);
}

