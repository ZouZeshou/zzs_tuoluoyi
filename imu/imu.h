#ifndef _IMU_H_
#define _IMU_H_

#include "stm32f1xx_hal.h"
#include "MPU6500.h"
#include <math.h>

void init_quaternion(void);
void imu_AHRS_update(void);
void imu_cal_update(void);
void AngleUpdate(void);

extern int imu_init_ok;
extern float q0,q1,q2,q3;
extern float angletrans,angleroll,anglepitch;

#endif



