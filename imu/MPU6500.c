#include "MPU6500.h"
#include "mpu6500_reg.h"
#include "IST8310_reg.h"
#include "spi.h"
#include "imu.h"

// imu x pitch z yaw
// x: up + down -
// z left + right -

uint8_t MPU_id = 0;
uint8_t IMUhealth = 0;

IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};
IMUDataTypedef imu_data_forcal = {0,0,0,0,0,0,0,0,0,0};
IMUDataTypedef imu_data_offset = {0,0,0,0,0,0,0,0,0,0};

int offset_gx = 0,offset_gy = 0,offset_gz = 0;
//Write a register to MPU6500
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg&0x7f;
  HAL_SPI_TransmitReceive(&hspi1, &MPU_Tx, &MPU_Rx, 1, 55);
  MPU_Tx = data;
  HAL_SPI_TransmitReceive(&hspi1, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();

  return 0;
}

//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg|0x80;
  HAL_SPI_TransmitReceive(&hspi1, &MPU_Tx, &MPU_Rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi1, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();
  return MPU_Rx;
}

//Read registers from MPU6500,address begin with regAddr
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
  static uint8_t MPU_Rx, MPU_Tx, MPU_Tx_buff[14] = {0xff};
  MPU6500_NSS_Low();
  
  MPU_Tx = regAddr|0x80;
  MPU_Tx_buff[0] = MPU_Tx;
  HAL_SPI_TransmitReceive(&hspi1, &MPU_Tx, &MPU_Rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi1, MPU_Tx_buff, pData, len, 55);
  
  MPU6500_NSS_High();
  return 0;
}


//Set the accelerated velocity resolution
uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr<<3);
}

//Set the angular velocity resolution
uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr<<3);
}

//Get 6 axis data from MPU6500

static void MPU9250_Mag_Write(uint8_t reg,uint8_t value){
	MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR,AK8963_I2C_ADDR);
	MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG,reg);
	MPU6500_Write_Reg(MPU6500_I2C_SLV0_DO,value);
	HAL_Delay(10);
}

uint8_t MPU9250_iic_Mag_read(uint8_t reg){
	MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR,0x0C|0x80);
	MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG,reg);
	MPU6500_Write_Reg(MPU6500_I2C_SLV0_DO,0xff);
	HAL_Delay(10);
	uint8_t data;
	MPU6500_Read_Regs(MPU6500_EXT_SENS_DATA_00,&data,1);
	return data;
}

void IMU_Get_Data()
{
    uint8_t mpu_buff[14];
    MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H , mpu_buff, 14);//14
	imu_data_forcal.ax = mpu_buff[0]<<8 |mpu_buff[1];
	imu_data_forcal.ay = mpu_buff[2]<<8 |mpu_buff[3];
	imu_data_forcal.az = mpu_buff[4]<<8 |mpu_buff[5];
	//imu_data_forcal.temp = 21 + (mpu_buff[6]<<8 |mpu_buff[7])/333.87f;	
	
	imu_data.gx = mpu_buff[8]<<8 |mpu_buff[9];
	imu_data.gy = mpu_buff[10]<<8 |mpu_buff[11];
	imu_data.gz = mpu_buff[12]<<8 |mpu_buff[13];
	
	imu_data_forcal.gx = imu_data.gx - imu_data_offset.gx;
	imu_data_forcal.gy = imu_data.gy - imu_data_offset.gy;
	imu_data_forcal.gz = imu_data.gz - imu_data_offset.gz;
	
	/************ÐÂÔö**************/
	if(abs(imu_data_forcal.gx) <= 10) imu_data_forcal.gx = 0;
	if(abs(imu_data_forcal.gy) <= 10) imu_data_forcal.gy = 0;
	if(abs(imu_data_forcal.gz) <= 10) imu_data_forcal.gz = 0;
	/*imu_data_forcal.mx = mpu_buff[14]<<8 |mpu_buff[15];
	imu_data_forcal.my = mpu_buff[16]<<8 |mpu_buff[17];
	imu_data_forcal.mz = mpu_buff[18]<<8 |mpu_buff[19];*/

}


//Initialize the MPU6500

static void mpu_offset_init(void)
{
	uint8_t mpu_buff[6];
	
	for(int i=0;i<300;i++)
	{
		MPU6500_Read_Regs(MPU6500_GYRO_XOUT_H, mpu_buff, 6);//14
		imu_data_offset.gx = mpu_buff[0]<<8 |mpu_buff[1];
		imu_data_offset.gy = mpu_buff[2]<<8 |mpu_buff[3];
		imu_data_offset.gz = mpu_buff[4]<<8 |mpu_buff[5];	
		offset_gx += imu_data_offset.gx;
		offset_gy += imu_data_offset.gy;
		offset_gz += imu_data_offset.gz;
		HAL_Delay(5);		
	}
	imu_data_offset.gx = offset_gx / 300;
	imu_data_offset.gy = offset_gy / 300;
	imu_data_offset.gz = offset_gz / 300;
}
uint8_t MPU6500_Init(void)
{
  MPU6500_Write_Reg(MPU6500_PWR_MGMT_1, 0x80);
  HAL_Delay(100);
  MPU6500_Write_Reg(MPU6500_SIGNAL_PATH_RESET, 0x07);
  HAL_Delay(100);
  
  IMUhealth = 1;
  uint8_t index = 0;
  if (MPU6500_ID != MPU6500_Read_Reg(MPU6500_WHO_AM_I))
	{
	  IMUhealth = 0;
	  printf("imu offline!\r\n");
	  return 1;
  }
  uint8_t MPU6500_Init_Data[7][2] = {
    { MPU6500_PWR_MGMT_1,     0x03 }, // Auto selects Clock Source
    { MPU6500_PWR_MGMT_2,     0x00 }, // all enable
    { MPU6500_CONFIG,         0x04 }, // gyro bandwidth 184Hz 01
    { MPU6500_GYRO_CONFIG,    0x18 }, // +-2000dps
    { MPU6500_ACCEL_CONFIG,   0x10 }, // +-8G
    { MPU6500_ACCEL_CONFIG_2, 0x04 }, // acc bandwidth 20Hz
    { MPU6500_USER_CTRL,      0x20 }, // Enable the I2C Master I/F module
                                      // pins ES_DA and ES_SCL are isolated from 
                                      // pins SDA/SDI and SCL/SCLK.
  };
  for(index = 0; index < 7; index++)
  {
    MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
    HAL_Delay(1);
  }
  mpu_offset_init();
  return 0;
}


