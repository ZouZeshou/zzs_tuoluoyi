#include "imu.h"
#include <string.h>
#include "tim.h"
#include "mpu6500_reg.h"
#include "can.h"

float q0 = 1.0,q1 = 0.0,q2 = 0.0,q3 = 0.0;
static volatile uint32_t last_update, now_update;
static volatile float exInt, eyInt, ezInt;
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;   //
float angletrans;
float angleroll,anglepitch;

int imu_init_ok = 0;

#define BOARD_DOWN 1   //

void init_quaternion(void)
{
  int16_t hx, hy;
  float temp;
	uint8_t buff[4];
	/*MPU6500_Read_Regs(MPU6500_EXT_SENS_DATA_00,buff,4);
  imu_data_forcal.mx = buff[0]<<8 | buff[1];
	imu_data_forcal.my = buff[2]<<8 | buff[3];*/
	/*imu_data_forcal.mx = MPU9250_iic_Mag_read(AK8963_ASAX);
	imu_data_forcal.my = MPU9250_iic_Mag_read(AK8963_ASAY);
  hx = imu_data_forcal.mx;
  hy = imu_data_forcal.my;
	printf("hx:%d hy:%d\r\n",hx,hy);*/
	
  if (hy != 0)
    temp = hx/hy;
  else
    return ;

  #ifdef BOARD_DOWN
  if(hx<0 && hy <0)   //OK
  {
    if(fabs(temp) >= 1)
    {
      q0 = -0.005;
      q1 = -0.199;
      q2 = 0.979;
      q3 = -0.0089;
    }
    else
    {
      q0 = -0.008;
      q1 = -0.555;
      q2 = 0.83;
      q3 = -0.002;
    }
    
  }
  else if (hx<0 && hy > 0) //OK
  {
    if(fabs(temp) >= 1)   
    {
      q0 = 0.005;
      q1 = -0.199;
      q2 = -0.978;
      q3 = 0.012;
    }
    else
    {
      q0 = 0.005;
      q1 = -0.553;
      q2 = -0.83;
      q3 = -0.0023;
    }
    
  }
  else if (hx > 0 && hy > 0)   //OK
  {
    if(fabs(temp) >= 1)
    {
      q0 = 0.0012;
      q1 = -0.978;
      q2 = -0.199;
      q3 = -0.005;
    }
    else
    {
      q0 = 0.0023;
      q1 = -0.83;
      q2 = -0.553;
      q3 = 0.0023;
    }
    
  }
  else if (hx > 0 && hy < 0)     //OK
  {
    if(fabs(temp) >= 1)
    {
      q0 = 0.0025;
      q1 = 0.978;
      q2 = -0.199;
      q3 = 0.008;
    }
    else
    {
      q0 = 0.0025;
      q1 = 0.83;
      q2 = -0.56;
      q3 = 0.0045;
    }
  }
  #else
    if(hx<0 && hy <0)
  {
    if(fabs(temp) >= 1)
    {
      q0 = 0.195;
      q1 = -0.015;
      q2 = 0.0043;
      q3 = 0.979;
    }
    else
    {
      q0 = 0.555;
      q1 = -0.015;
      q2 = 0.006;
      q3 = 0.829;
    }
    
  }
  else if (hx<0 && hy > 0)
  {
    if(fabs(temp) >= 1)
    {
      q0 = -0.193;
      q1 = -0.009;
      q2 = -0.006;
      q3 = 0.979;
    }
    else
    {
      q0 = -0.552;
      q1 = -0.0048;
      q2 = -0.0115;
      q3 = 0.8313;
    }
    
  }
  else if (hx>0 && hy > 0)
  {
    if(fabs(temp) >= 1)
    {
      q0 = -0.9785;
      q1 = 0.008;
      q2 = -0.02;
      q3 = 0.195;
    }
    else
    {
      q0 = -0.9828;
      q1 = 0.002;
      q2 = -0.0167;
      q3 = 0.5557;
    }
    
  }
  else if (hx > 0 && hy < 0)
  {
    if(fabs(temp) >= 1)
    {
      q0 = -0.979;
      q1 = 0.0116;
      q2 = -0.0167;
      q3 = -0.195;
    }
    else
    {
      q0 = -0.83;
      q1 = 0.014;
      q2 = -0.012;
      q3 = -0.556;
    }
  }
  #endif
   
}

float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

float halfT;
float Kp  = 2.0, Ki = 0.01;


void imu_AHRS_update(void) 
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;//, halfT;
  float tempq0,tempq1,tempq2,tempq3;

  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;

  gx = imu_data_forcal.gx / 16.384f / 57.2957795f; // dps->rad/s
  gy = imu_data_forcal.gy / 16.384f / 57.2957795f;
  gz = imu_data_forcal.gz / 16.384f / 57.2957795f;
  ax = imu_data_forcal.ax;
  ay = imu_data_forcal.ay;
  az = imu_data_forcal.az;
  mx = 0;//imu_data_forcal.mx;
  my = 0;//imu_data_forcal.my;
  mz = 0;//imu_data_forcal.mz;

  now_update = HAL_GetTick(); //ms
  halfT =  ((float)(now_update - last_update) / 2000.0f);
  last_update = now_update;


  //Fast inverse square-root
  norm = invSqrt(ax*ax + ay*ay + az*az);
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;

  /*norm = invSqrt(mx*mx + my*my + mz*mz);
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  // compute reference direction of flux
  hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
  hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
  hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz; */

  // estimated direction of gravity and flux (v and w)
  vx = 2.0f*(q1q3 - q0q2);
  vy = 2.0f*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
  wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
  wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy);
  ey = (az*vx - ax*vz);
  ez = (ax*vy - ay*vx);

  if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
  {
      exInt = exInt + ex * Ki * halfT;
      eyInt = eyInt + ey * Ki * halfT;
      ezInt = ezInt + ez * Ki * halfT;
      // PI
      gx = gx + Kp*ex + exInt;
      gy = gy + Kp*ey + eyInt;
      gz = gz + Kp*ez + ezInt;
  }
  // 
  tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

  //normalise quaternion
  norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
  q0 = tempq0 * norm;
  q1 = tempq1 * norm;
  q2 = tempq2 * norm;
  q3 = tempq3 * norm;

}

void Angle2Update(void)
{
	angleroll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * 57.2957795f; // roll       -pi----pi
	anglepitch = asin(-2*q1*q3 + 2*q0*q2)* 57.2957795f;  
}

void AngleUpdate(void)
{
	static int cnt, pitch_cnt;
	static float imulast,imunow,imu_first, imupitch_now, imupitch_last;
	static int inittic;
	/* yaw angle update */
	imunow = (atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1))* 57.2957795f;
	if(imunow - imulast > 330) cnt--;
	if(imunow - imulast < -330) cnt++;
	imulast = imunow;
	if(!imu_init_ok )
	{
		imu_first = imunow + cnt*360.0;
		inittic++;
		if(inittic > 3000) imu_init_ok  = 1; 
	}
	angletrans = imunow + cnt*360.0 - imu_first;
	
	/* pitch angle update */
	anglepitch = asin(-2*q1*q3 + 2*q0*q2)* 57.2957795f;
//	if(imupitch_now - imupitch_last > 330) pitch_cnt--;
//	if(imupitch_now - imupitch_last < -330) pitch_cnt++;
//	anglepitch = pitch_cnt * 180.0f + imupitch_now; 
//	imupitch_last = imupitch_now;

	/* roll angle update */
	angleroll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * 57.2957795f; // roll       -pi----pi
}

void imu_cal_update(void)
{ 
	int counter;
	IMU_Get_Data();
	imu_AHRS_update();
	AngleUpdate();
	//pitch
	//CAN_Send(0x405,imu_data_forcal.gy,imu_data_forcal.gz,angletrans);
	//yaw 
	//CAN_Send(0x401,imu_data_forcal.gy,imu_data_forcal.gz,angletrans);
  //chassis
	CAN_Send(0x402 ,imu_data_forcal.gy,imu_data_forcal.gz,angletrans);
//	CAN_Send(0x405,0,0,angletrans);
//	counter++;
//	if(counter == 30)
//	{
//		counter = 0;
//	}
	//CAN_SendI(0x401,imu_data.gx,imu_data.gy,imu_data.gz);
	//CAN_SendI(0x401,imu_data_forcal.gx,imu_data_forcal.gy,imu_data_forcal.gz);
	//CAN_SendF(0x402,angleroll,angletrans);
}
