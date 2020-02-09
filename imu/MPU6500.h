#ifndef _MPU6500_H_
#define _MPU6500_H_

#include "stm32f1xx_hal.h"
#include "main.h"

#define MPU6500_NSS_Low() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define MPU6500_NSS_High() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  
  int16_t temp;
  
  int16_t gx;
  int16_t gy;
  int16_t gz;
  
  int16_t mx;
  int16_t my;
  int16_t mz;
	
}IMUDataTypedef;

extern uint8_t MPU_id;
extern uint8_t IMUhealth;
extern IMUDataTypedef imu_data;
extern IMUDataTypedef imu_data_forcal;
extern int offset_gx ,offset_gy ,offset_gz ;
extern IMUDataTypedef imu_data ;
extern IMUDataTypedef imu_data_forcal ;
extern IMUDataTypedef imu_data_offset ;

uint8_t MPU6500_Init(void);
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data);
uint8_t MPU6500_Read_Reg(uint8_t const reg);
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len);
uint8_t MPU9250_iic_Mag_read(uint8_t reg);
void IMU_Get_Data(void);

#endif

