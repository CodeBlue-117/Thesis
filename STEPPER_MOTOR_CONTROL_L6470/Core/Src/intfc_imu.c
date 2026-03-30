/*
 * intfc_imu.c
 *
 *  Created on: Mar 24, 2026
 *      Author: jake-
 */

#include "intfc_imu.h"

HAL_StatusTypeDef IMU_Write(uint16_t reg, uint8_t data)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(&hi2c1, MPU6000_ADDR, reg, 1, &data, 1, HAL_MAX_DELAY);
	return status;

}


HAL_StatusTypeDef IMU_Read(uint16_t reg, uint8_t *buf, uint8_t len)
{

	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(&hi2c1, MPU6000_ADDR, reg, 1, buf, len, HAL_MAX_DELAY);
	return status;
}


uint8_t initializeIMU(void)
{
	  HAL_StatusTypeDef status;

	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  // Read WhoAmI and verify it is 0x68
	  uint8_t reg = 0x75;
	  uint8_t receiveData = 0;

	  status = IMU_Read(reg, &receiveData, 1);
	  if(status != HAL_OK)
	  {
		  printf("Error reading WhoAmI register\n\r");
		  return 1;
	  }

	  HAL_Delay(10);

	  if(receiveData != 0x68)
	  {
		  printf("Error reading WhoAmI register\n\r");
		  return 1;
	  }

	  HAL_Delay(10);

	  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  // Device Reset
	  status = IMU_Write(PWR_MGMT_REG_1, 0x80); // 1000-0000
	  if(status != HAL_OK)
	  {
		  printf("Error in Device Reset\n\r");
		  return 1;
	  }

	  HAL_Delay(100);

	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      //Signal Path Reset
	  status = IMU_Write(SIGNAL_PATH_REG, 0x07); // Reset GYRO, ACCEL and TEMP 0000-0111 = 0x07
	  if(status != HAL_OK)
	  {
		  printf("Error resetting accel and gyro\n\r");
		  return 1;
	  }

	  HAL_Delay(100);

	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  // Wakeup and Clock Source
	  status = IMU_Write(PWR_MGMT_REG_1, 0x01); // Clock Source PLL from x-axis
	  if(status != HAL_OK)
	  {
		  printf("Error setting clock source\n\r");
		  return 1;
	  }

	  HAL_Delay(10);

	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  // Configure DLPF
	  status = IMU_Write(CONFIG_REG, 0x02); // DLPF in CONFIG REG set to 94Hz bandwidth and 3ms delay (try 0x01 for 184Hz BW and 2ms delay)
	  if(status != HAL_OK)
	  {
		  printf("Error setting DLPF\n\r");
		  return 1;
	  }
	  HAL_Delay(10);

	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  // Configure Accel Full Scale Range
	  status = IMU_Write(ACCEL_CONFIG_REG, 0x00); // Configure Accel for full scale range +2g
	  if(status != HAL_OK)
	  {
		  printf("Error setting Accel Config/Full Scale Range\n\r");
		  return 1;
	  }
	  HAL_Delay(10);

	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  // Configure Gyro Full Scale Range
	  status = IMU_Write(GYRO_CONFIG_REG, 0x00); // Configure Accel for full scale range +2g
	  if(status != HAL_OK)
	  {
		  printf("Error setting Gyro Config/Full Scale Range\n\r");
		  return 1;
	  }
	  HAL_Delay(10);

	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  return status;
}

uint8_t IMU_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
	uint8_t buf[6];

	if(HAL_I2C_Mem_Read(&hi2c1, MPU6000_ADDR, ACCEL_OUT_REG_START, 1, buf, 6, HAL_MAX_DELAY) != HAL_OK)
	{
		return 1;
	}

	*ax = (int16_t)((buf[0] << 8) | buf[1]);
	*ay = (int16_t)((buf[2] << 8) | buf[3]);
	*az = (int16_t)((buf[4] << 8) | buf[5]);

	return 0;

}

uint8_t IMU_ReadGyro(int16_t *wx, int16_t *wy, int16_t *wz)
{
	uint8_t buf[6];

	if(HAL_I2C_Mem_Read(&hi2c1, MPU6000_ADDR, GYRO_OUT_REG_START, 1, buf, 6, HAL_MAX_DELAY) != HAL_OK)
	{
		return 1;
	}

	*wx = (int16_t)((buf[0] << 8) | buf[1]);
	*wy = (int16_t)((buf[2] << 8) | buf[3]);
	*wz = (int16_t)((buf[4] << 8) | buf[5]);

	return 0;

}
