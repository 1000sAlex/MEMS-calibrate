/*
 * mpu6050.c
 *
 *  Created on: Jan 25, 2020
 *      Author: u
 */

#include "mpu6050.h"
#include "lsm303dlhc.h"
u8 startup_flag = 0;

MPU_DATA mpu;

static u8 colibrate_init = 0;

void MPU6050_colibrate_start(void)
    {
    if (colib_number == 0)
	{
	colib_number = COLIB_NUMBER;
	}
    if (BitIsClear(colibrate_init, COLIB_MPU_1_START) &&
    BitIsClear(colibrate_init, COLIB_MPU_1_READY))
	{
	MPU6050_1_colibrate();
	}
    if (BitIsClear(colibrate_init, COLIB_MPU_2_START) &&
    BitIsClear(colibrate_init, COLIB_MPU_2_READY))
	{
	MPU6050_2_colibrate();
	}
    if (BitIsSet(colibrate_init, COLIB_MPU_1_READY) &&
    BitIsSet(colibrate_init,COLIB_MPU_2_READY))
	{
#if (DEBUG == 1)
	printf("COLIB_READY\n");
	printf("gX1=%d gY1=%d gZ1=%d\n", mpu.bias_gX1, mpu.bias_gY1,
		mpu.bias_gZ1);
	printf("gX2=%d gY2=%d gZ2=%d\n", mpu.bias_gX2, mpu.bias_gY2,
		mpu.bias_gZ2);
#endif
	mpu.q0_1 = 1.0f;
	mpu.q0_2 = 1.0f;
	}
    }

void MPU6050_1_colibrate(void)
    {
    static u16 colib_count = 0;
    static float x, y, z;
    if (BitIsClear(colibrate_init, COLIB_MPU_1_START))
	{
	x = mpu.gX1;
	y = mpu.gY1;
	z = mpu.gZ1;
	colibrate_init |= (1 << COLIB_MPU_1_START);
	}
    else
	{
	if (colib_count < colib_number)
	    {
	    x = x * (1 - K) + mpu.gX1 * K;
	    y = y * (1 - K) + mpu.gY1 * K;
	    z = z * (1 - K) + mpu.gZ1 * K;
	    colib_count++;
	    }
	else
	    {
	    mpu.bias_gX1 = (s16) x;
	    mpu.bias_gY1 = (s16) y;
	    mpu.bias_gZ1 = (s16) z;
	    colibrate_init &= ~(1 << COLIB_MPU_1_START);
	    colibrate_init |= (1 << COLIB_MPU_1_READY);
	    SetTask(MPU6050_colibrate_start);
	    colib_count = 0;
	    }
	}
    }

void MPU6050_2_colibrate(void)
    {
    static u16 colib_count = 0;
    static float x, y, z;
    if (BitIsClear(colibrate_init, COLIB_MPU_2_START))
	{
	x = mpu.gX2;
	y = mpu.gY2;
	z = mpu.gZ2;
	colibrate_init |= (1 << COLIB_MPU_2_START);
	}
    else
	{
	if (colib_count < colib_number)
	    {
	    x = x * (1 - K) + mpu.gX2 * K;
	    y = y * (1 - K) + mpu.gY2 * K;
	    z = z * (1 - K) + mpu.gZ2 * K;
	    colib_count++;
	    }
	else
	    {
	    mpu.bias_gX2 = (s16) x;
	    mpu.bias_gY2 = (s16) y;
	    mpu.bias_gZ2 = (s16) z;
	    colibrate_init &= ~(1 << COLIB_MPU_2_START);
	    colibrate_init |= (1 << COLIB_MPU_2_READY);
	    SetTask(MPU6050_colibrate_start);
	    colib_count = 0;
	    }
	}
    }

void MPU6050_1_get_raw(void)
    {
    if (startup_flag)
	{
	u8 data[14];
	Accel_IO_Read_byf(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, data, 14);
	mpu.accX1 = (data[0] << 8) | data[1];
	mpu.accY1 = (data[2] << 8) | data[3];
	mpu.accZ1 = (data[4] << 8) | data[5];
	mpu.Temp1 = (data[6] << 8) | data[7];
	mpu.gX1 = (data[8] << 8) | data[9];
	mpu.gY1 = (data[10] << 8) | data[11];
	mpu.gZ1 = (data[12] << 8) | data[13];
	SetTimerTask(MPU6050_1_get_raw, 110);
	if (BitIsSet(colibrate_init, COLIB_MPU_1_START))
	    {
	    SetTask(MPU6050_1_colibrate);
	    }
	else if (BitIsSet(colibrate_init, COLIB_MPU_1_READY))
	    {
	    SetTask(position_1_calc);
	    }
	}
    }

void MPU6050_2_get_raw(void)
    {
    if (startup_flag)
	{
	u8 data[14];
	Accel_IO_Read_byf(MPU6050_ADDRESS2, MPU6050_RA_ACCEL_XOUT_H, data, 14);
	mpu.accX2 = (data[0] << 8) | data[1];
	mpu.accY2 = (data[2] << 8) | data[3];
	mpu.accZ2 = (data[4] << 8) | data[5];
	mpu.Temp2 = (data[6] << 8) | data[7];
	mpu.gX2 = (data[8] << 8) | data[9];
	mpu.gY2 = (data[10] << 8) | data[11];
	mpu.gZ2 = (data[12] << 8) | data[13];
	SetTimerTask(MPU6050_2_get_raw, 110);
	if (BitIsSet(colibrate_init, COLIB_MPU_2_START))
	    {
	    SetTask(MPU6050_2_colibrate);
	    }
	else if (BitIsSet(colibrate_init, COLIB_MPU_1_READY))
	    {
		{
		SetTask(position_2_calc);
		}
	    }
	}
    }

u8 MPU_init(void)
    {
    //сбрасываем mpu
    HAL_Delay(100);
    if (Accel_IO_Read(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I) == 0x68)
	{
	Accel_IO_Write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x80);
	HAL_Delay(100);
	Accel_IO_Write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01);
	HAL_Delay(100);
	startup_flag = 1;
	SetTimerTask(MPU6050_1_get_raw, 100);
	//частота работы
	Accel_IO_Write(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 9); //100гц
	//включаем фильтр и делаем частоту тактирования гироскопа 1кгц
	Accel_IO_Write(MPU6050_ADDRESS, MPU6050_RA_CONFIG,
	MPU6050_DLPF_BW_42);
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
	startup_flag = 0;
	return MPU_1_ERR;
	}
    //мпу 2

    if (Accel_IO_Read(MPU6050_ADDRESS2, MPU6050_RA_WHO_AM_I) == 0x68)
	{
	Accel_IO_Write(MPU6050_ADDRESS2, MPU6050_RA_PWR_MGMT_1, 0x80);
	HAL_Delay(100);
	Accel_IO_Write(MPU6050_ADDRESS2, MPU6050_RA_PWR_MGMT_1, 0x01);
	HAL_Delay(100);
	startup_flag = 1;
	SetTimerTask(MPU6050_2_get_raw, 100);
	//частота работы
	Accel_IO_Write(MPU6050_ADDRESS2, MPU6050_RA_SMPLRT_DIV, 9); //100гц
	//включаем фильтр и делаем частоту тактирования гироскопа 1кгц
	Accel_IO_Write(MPU6050_ADDRESS2, MPU6050_RA_CONFIG,
	MPU6050_DLPF_BW_42);
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
	startup_flag = 0;
	return MPU_2_ERR;
	}
    return MPU_OK;
    }
