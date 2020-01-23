/*
 * lsm303dlhc.c
 *
 *  Created on: Jan 23, 2020
 *      Author: u
 */

#include "lsm303dlhc.h"
#include "main.h"

u8 I2Cx_ReadData(u16 Addr, u8 Reg);
void Accel_IO_Write(u16 DeviceAddr, u8 RegisterAddr, u8 Value);
static void I2Cx_WriteData(u16 Addr, u8 Reg, u8 Value);
void Accel_IO_Read_byf(u16 DeviceAddr, u8 RegisterAddr, u8 *data, u8 len);
static void Mag_Get_raw(void);
static void Accel_Get_raw(void);
u8 Accel_ReadID(void);
static void Error(void);

extern I2C_HandleTypeDef hi2c1;
LSM_DATA LSM303_data;

void Accel_Init(void)
    {
    HAL_Delay(50);
    if (Accel_ReadID() == 0x33)
	{
	LED_G2_GPIO_Port->ODR |= LED_G2_Pin;
	}
    else
	{
	Error();
	}
    Accel_IO_Write(0x32, LSM303DLHC_CTRL_REG1_A, LSM303DLHC_NORMAL_MODE |
    LSM303DLHC_ODR_100_HZ | LSM303DLHC_AXES_ENABLE);
    Accel_IO_Write(0x32, LSM303DLHC_CTRL_REG4_A, LSM303DLHC_HR_ENABLE);
    Accel_IO_Write(0x32, LSM303DLHC_CTRL_REG3_A, 0x10);
    Accel_IO_Write(0x3C, 0, 0x80 | 0x18);
    Accel_IO_Write(0x3C, 0x02, 0);
    HAL_Delay(200);
    Accel_Get_raw();
    Mag_Get_raw();
    }

inline static void Accel_Get_raw(void)
    {
    /* Read output register X, Y & Z acceleration */
    u8 temp[6];
    Accel_IO_Read_byf(0x32, LSM303DLHC_OUT_X_L_A | 0x80, temp, 6);
    LSM303_data.accX = temp[0];
    LSM303_data.accX = (temp[1] << 8);
    LSM303_data.accY = temp[2];
    LSM303_data.accY = (temp[3] << 8);
    LSM303_data.accZ = temp[4];
    LSM303_data.accZ = (temp[5] << 8);
    }

inline static void Mag_Get_raw(void)
    {
    /* Read output register X, Y & Z acceleration */
    u8 temp[6];
    Accel_IO_Read_byf(0x3D, 0x03 | 0x80, temp, 6);
    LSM303_data.magX = temp[0];
    LSM303_data.magX = (temp[1] << 8);
    LSM303_data.magY = temp[2];
    LSM303_data.magY = (temp[3] << 8);
    LSM303_data.magZ = temp[4];
    LSM303_data.magZ = (temp[5] << 8);
    }


u8 Accel_ReadID(void)
    {
    u8 ctrl = 0x00;
    ctrl = Accel_IO_Read(0x32, 0x0F);
    return ctrl;
    }

void Accel_IO_Read_byf(u16 DeviceAddr, u8 RegisterAddr, u8 *data, u8 len)
    {
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_I2C_Mem_Read(&hi2c1, DeviceAddr, RegisterAddr,
    I2C_MEMADD_SIZE_8BIT, data, len, 0x100);
    if (status != HAL_OK)
	{
	Error();
	}
    }

u8 Accel_IO_Read(u16 DeviceAddr, u8 RegisterAddr)
    {
    return I2Cx_ReadData(DeviceAddr, RegisterAddr);
    }

void Accel_IO_Write(u16 DeviceAddr, u8 RegisterAddr, u8 Value)
    {
    I2Cx_WriteData(DeviceAddr, RegisterAddr, Value);
    }

static void Error(void)
    {
    LED_R1_GPIO_Port->ODR |= LED_R1_Pin;
    }

inline u8 I2Cx_ReadData(u16 Addr, u8 Reg)
    {
    HAL_StatusTypeDef status = HAL_OK;
    u8 value = 0;
    status = HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value,
	    1, 0x100);
    if (status != HAL_OK)
	{
	Error();
	}
    return value;
    }

inline static void I2Cx_WriteData(u16 Addr, u8 Reg, u8 Value)
    {
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_I2C_Mem_Write(&hi2c1, Addr, (u16) Reg,
    I2C_MEMADD_SIZE_8BIT, &Value, 1, 0x100);
    if (status != HAL_OK)
	{
	Error();
	}
    }

void Mag_int(void)
    {
    LED_O1_GPIO_Port->ODR ^= LED_O1_Pin;
    Mag_Get_raw();
    SetTimerTask(Mag_int,99);
    }

void Acc_int(void)
    {
    LED_O2_GPIO_Port->ODR ^= LED_O2_Pin;
    Accel_Get_raw();
    SetTimerTask(Acc_int,99);
    }
