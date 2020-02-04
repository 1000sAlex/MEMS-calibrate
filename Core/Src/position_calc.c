/*
 * position_calc.c
 *
 *  Created on: Feb 3, 2020
 *      Author: u
 */
#include "position_calc.h"

#define GYRO_K (500.0f / 32768.0f) * M_PI / 180.0f
extern MPU_DATA mpu;

const float gyro_k = GYRO_K;
void position_1_calc(void)
    {
    LED_B1_GPIO_Port->ODR |= LED_B1_Pin;
    MadgwickAHRSupdateIMU(mpu.accX1, mpu.accY1, mpu.accZ1,
	    (mpu.gX1 - mpu.bias_gX1) * gyro_k,
	    (mpu.gY1 - mpu.bias_gY1) * gyro_k,
	    (mpu.gZ1 - mpu.bias_gZ1) * gyro_k, &mpu.q0_1, &mpu.q1_1, &mpu.q2_1,
	    &mpu.q3_1);
    mpu.yaw_1 = atan2f(2.0f * mpu.q1_1 * mpu.q2_1 - 2.0f * mpu.q0_1 * mpu.q3_1,
	    2.0f * mpu.q0_1 * mpu.q0_1 + 2.0f * mpu.q1_1 * mpu.q1_1 - 1.0f);
    mpu.pitch_1 = -1.0f
	    * asinf(2.0f * mpu.q1_1 * mpu.q3_1 + 2.0f * mpu.q0_1 * mpu.q2_1);
    mpu.roll_1 = atan2f(2.0f * mpu.q2_1 * mpu.q3_1 - 2.0f * mpu.q0_1 * mpu.q1_1,
	    2.0f * mpu.q0_1 * mpu.q0_1 + 2.0f * mpu.q3_1 * mpu.q3_1 - 1.0f);
#if (DEBUG == 1)
    printf(" pry1 %f;%f;%f\n", mpu.pitch_1, mpu.roll_1, mpu.yaw_1);
    LED_B1_GPIO_Port->ODR &= ~LED_B1_Pin;
#endif
    }
void position_2_calc(void)
    {
    LED_B1_GPIO_Port->ODR |= LED_B1_Pin;
    MadgwickAHRSupdateIMU(mpu.accX2, mpu.accY2, mpu.accZ2,
	    (mpu.gX2 - mpu.bias_gX2) * gyro_k,
	    (mpu.gY2 - mpu.bias_gY2) * gyro_k,
	    (mpu.gZ2 - mpu.bias_gZ2) * gyro_k, &mpu.q0_2, &mpu.q1_2, &mpu.q2_2,
	    &mpu.q3_2);
    mpu.yaw_2 = atan2f(2.0f * mpu.q1_2 * mpu.q2_2 - 2.0f * mpu.q0_2 * mpu.q3_2,
	    2.0f * mpu.q0_2 * mpu.q0_2 + 2.0f * mpu.q1_2 * mpu.q1_2 - 1.0f);
    mpu.pitch_2 = -1.0f
	    * asinf(2.0f * mpu.q1_2 * mpu.q3_2 + 2.0f * mpu.q0_2 * mpu.q2_2);
    mpu.roll_2 = atan2f(2.0f * mpu.q2_2 * mpu.q3_2 - 2.0f * mpu.q0_2 * mpu.q1_2,
	    2.0f * mpu.q0_2 * mpu.q0_2 + 2.0f * mpu.q3_2 * mpu.q3_2 - 1.0f);
#if (DEBUG == 1)
    printf(" pry2 %f;%f;%f\n", mpu.pitch_2, mpu.roll_2, mpu.yaw_2);
    LED_B1_GPIO_Port->ODR &= ~LED_B1_Pin;
#endif
    }

void position_init(void)
    {
    u8 status = MPU_init();
    if (status == MPU_OK)
	{
	LED_G1_GPIO_Port->ODR |= LED_G1_Pin;
	SetTimerTask(MPU6050_colibrate_start, 100);
#if (DEBUG == 1)
	printf("MPU_OK\n");
#endif
	}
    else
	{
	LED_R1_GPIO_Port->ODR |= LED_R1_Pin;
#if (DEBUG == 1)
	if (status == MPU_1_ERR)
	    {
	    printf("MPU_1_ERR\n");
	    }
	if (status == MPU_2_ERR)
	    {
	    printf("MPU_2_ERR\n");
	    }
#endif
	}
    }

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	100.0f		// sample frequency in Hz

#define gyroMeasError 3.14159265358979f * (2.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define Test_beta 0.866f * gyroMeasError // compute beta
#define GyroMeasDrift 3.14159265358979f * (1.1f / 180.0f)    // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
#define Zeta 0.866f * GyroMeasDrift  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

//---------------------------------------------------------------------------------------------------
// Variable definitions

// parameters for 6 DoF sensor fusion calculations
float beta = Test_beta; // 2 * proportional gain (Kp)
float zeta = Zeta; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 1.0f / sampleFreq; // integration interval for both filter schemes
//float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations
#define INVSQRT 1
#if INVSQRT == 1
float invSqrt(float x)
    {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
    }
#else
#define invSqrt(x) 1.0f/sqrtf(x)
#endif

void MadgwickAHRSupdateIMU(float ax, float ay, float az, float gx, float gy,
	float gz, float *pq0, float *pq1, float *pq2, float *pq3)
    {
    float recipNorm;
    float q0, q1, q2, q3;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1,
	    q2q2, q3q3;
    q0 = (*pq0);
    q1 = (*pq1);
    q2 = (*pq2);
    q3 = (*pq3);
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    //if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
//	{

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1
	    + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
	    + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
//	}

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * deltat;
    q1 += qDot2 * deltat;
    q2 += qDot3 * deltat;
    q3 += qDot4 * deltat;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    (*pq0) = q0;
    (*pq1) = q1;
    (*pq2) = q2;
    (*pq3) = q3;
    }
