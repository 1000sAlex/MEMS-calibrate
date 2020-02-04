/*
 * position_calc.h
 *
 *  Created on: Feb 3, 2020
 *      Author: u
 */

#ifndef INC_POSITION_CALC_H_
#define INC_POSITION_CALC_H_

#include "main.h"

void position_init(void);
void position_1_calc(void);
void position_2_calc(void);
void MadgwickAHRSupdateIMU(float ax, float ay, float az, float gx, float gy,
	float gz, float *q0, float *q1, float *q2, float *q3);

#endif /* INC_POSITION_CALC_H_ */
