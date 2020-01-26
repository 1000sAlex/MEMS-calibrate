/*
 * mpu6050.c
 *
 *  Created on: Jan 25, 2020
 *      Author: u
 */

#include "mpu6050.h"
#include "lsm303dlhc.h"
u8 startup_flag = 0;

MPU_DATA MPU_data;

void MPU_get_data(void)
    {
//    static float avg_pitch = 0;
//    static float avg_roll = 0;
//    AccX = ((float) ((s16) ((i2c_data_get_buf[0] << 8) | i2c_data_get_buf[1])))
//	    * (2.0 / 32768.0);
//    AccY = ((float) ((s16) ((i2c_data_get_buf[2] << 8) | i2c_data_get_buf[3])))
//	    * (2.0 / 32768.0);
//    AccZ = ((float) ((s16) ((i2c_data_get_buf[4] << 8) | i2c_data_get_buf[5])))
//	    * (2.0 / 32768.0);
//    GyroX = ((float) ((s16) ((i2c_data_get_buf[8] << 8) | i2c_data_get_buf[9])))
//	    * (500.0 / 32768.0);
//    GyroY = ((float) ((s16) ((i2c_data_get_buf[10] << 8) | i2c_data_get_buf[11])))
//		    * (500.0 / 32768.0);
//    GyroZ =
//	    ((float) ((s16) ((i2c_data_get_buf[12] << 8) | i2c_data_get_buf[13])))
//		    * (500.0 / 32768.0);
//    MadgwickAHRSupdateIMU(AccX, AccY, AccZ, GyroX * M_PI / 180.0f,
//	    GyroY * M_PI / 180.0f, GyroZ * M_PI / 180.0f); //,mag_data[0],mag_data[1],-mag_data[2]);
//    Convert();
//    avg_pitch = (avg_pitch * (1 - K)) + (pitch * K);
//    avg_roll = (avg_roll * (1 - K)) + (roll * K);
//    //avg_yaw = (avg_yaw * 0.9) + (yaw * 0.1);

    }

void MPU6050_1_get_raw(void)
    {
    if (startup_flag)
	{
	u8 data[14];
	Accel_IO_Read_byf(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, data, 14);
	MPU_data.accX1 = (data[0] << 8) | data[1];
	MPU_data.accY1 = (data[2] << 8) | data[3];
	MPU_data.accZ1 = (data[4] << 8) | data[5];
	MPU_data.Temp1 = (data[6] << 8) | data[7];
	MPU_data.gX1 = (data[8] << 8) | data[9];
	MPU_data.gY1 = (data[10] << 8) | data[11];
	MPU_data.gZ1 = (data[12] << 8) | data[13];
	for (u8 i = 0; i < 14; i++)
	    {
	    MPU_data.raw_data[i] = data[i];
	    }
	SetTimerTask(MPU6050_1_get_raw, 110);
	}
    }

void MPU6050_2_get_raw(void)
    {
    if (startup_flag)
	{
	u8 data[14];
	Accel_IO_Read_byf(MPU6050_ADDRESS2, MPU6050_RA_ACCEL_XOUT_H, data, 14);
	MPU_data.accX2 = (data[0] << 8) | data[1];
	MPU_data.accY2 = (data[2] << 8) | data[3];
	MPU_data.accZ2 = (data[4] << 8) | data[5];
	MPU_data.Temp2 = (data[6] << 8) | data[7];
	MPU_data.gX2 = (data[8] << 8) | data[9];
	MPU_data.gY2 = (data[10] << 8) | data[11];
	MPU_data.gZ2 = (data[12] << 8) | data[13];
	for (u8 i = 0; i < 14; i++)
	    {
	    MPU_data.raw_data[14 + i] = data[i];
	    }
	SetTimerTask(MPU6050_2_get_raw, 110);
	}
    }

void MPU_init(void)
    {
    //сбрасываем mpu
    HAL_Delay(100);
    if (Accel_IO_Read(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I) == 0x68)
	{
	Accel_IO_Write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x80);
	HAL_Delay(100);
	Accel_IO_Write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01);
	HAL_Delay(100);
	LED_G1_GPIO_Port->ODR |= LED_G1_Pin;
	startup_flag = 1;
	SetTimerTask(MPU6050_1_get_raw, 100);
	//500 гц частота работы
	Accel_IO_Write(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 9); //100гц
	//включаем фильтр и делаем частоту тактирования гироскопа 1кгц
	Accel_IO_Write(MPU6050_ADDRESS, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42);
	//задаем диапазон гироскопа +-500 градусов в сек
	Accel_IO_Write(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG,
	MPU6050_GYRO_FS_500 << 3);
	//задаем диапазон акселерометра +-2g в сек
	Accel_IO_Write(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG,
	MPU6050_ACCEL_FS_2 << 3);
	//включим прерывание
	Accel_IO_Write(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG,
		1 << MPU6050_INTCFG_LATCH_INT_EN_BIT
			| 1 << MPU6050_INTCFG_INT_RD_CLEAR_BIT);
	Accel_IO_Write(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE,
		1 << MPU6050_INTERRUPT_DATA_RDY_BIT);
	}
    else
	{
	LED_R1_GPIO_Port->ODR |= LED_R1_Pin;
	startup_flag = 0;
	}
    //мпу 2

    if (Accel_IO_Read(MPU6050_ADDRESS2, MPU6050_RA_WHO_AM_I) == 0x68)
	{
	Accel_IO_Write(MPU6050_ADDRESS2, MPU6050_RA_PWR_MGMT_1, 0x80);
	HAL_Delay(100);
	Accel_IO_Write(MPU6050_ADDRESS2, MPU6050_RA_PWR_MGMT_1, 0x01);
	HAL_Delay(100);
	LED_G2_GPIO_Port->ODR |= LED_G2_Pin;
	startup_flag = 1;
	SetTimerTask(MPU6050_2_get_raw, 100);
	//500 гц частота работы
	Accel_IO_Write(MPU6050_ADDRESS2, MPU6050_RA_SMPLRT_DIV, 9); //100гц
	//включаем фильтр и делаем частоту тактирования гироскопа 1кгц
	Accel_IO_Write(MPU6050_ADDRESS2, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42);
	//задаем диапазон гироскопа +-500 градусов в сек
	Accel_IO_Write(MPU6050_ADDRESS2, MPU6050_RA_GYRO_CONFIG,
	MPU6050_GYRO_FS_500 << 3);
	//задаем диапазон акселерометра +-2g в сек
	Accel_IO_Write(MPU6050_ADDRESS2, MPU6050_RA_ACCEL_CONFIG,
	MPU6050_ACCEL_FS_2 << 3);
	//включим прерывание
	Accel_IO_Write(MPU6050_ADDRESS2, MPU6050_RA_INT_PIN_CFG,
		1 << MPU6050_INTCFG_LATCH_INT_EN_BIT
			| 1 << MPU6050_INTCFG_INT_RD_CLEAR_BIT);
	Accel_IO_Write(MPU6050_ADDRESS2, MPU6050_RA_INT_ENABLE,
		1 << MPU6050_INTERRUPT_DATA_RDY_BIT);
	}
    else
	{
	LED_R2_GPIO_Port->ODR |= LED_R2_Pin;
	startup_flag = 0;
	}
    }
