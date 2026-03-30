/*
 * intfc_imu.h
 *
 *  Created on: Mar 24, 2026
 *      Author: jake-
 */
#include "stdint.h"
#include "math.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"
#include <stdio.h>

#ifndef INC_INTFC_IMU_H_
#define INC_INTFC_IMU_H_

// Definitions
#define MPU6000_ADDR 		(0x68 << 1) // 0xD0
#define PWR_MGMT_REG_1		(0x6B)
#define SIGNAL_PATH_REG 	(0x68)
#define CONFIG_REG			(0x1A)
#define ACCEL_CONFIG_REG	(0x1C)
#define GYRO_CONFIG_REG		(0x1B)
#define DEBOUNCE_DELAY 		200  // 50ms debounce time
#define DEFAULT_DT	  		0.003f

#define ACCEL_OUT_REG_START 0x3B
#define GYRO_OUT_REG_START  0x43

// Function Prototypes
HAL_StatusTypeDef IMU_Write(uint16_t reg, uint8_t data);
HAL_StatusTypeDef IMU_Read(uint16_t reg, uint8_t *buf, uint8_t len);
uint8_t initializeIMU(void);
uint8_t IMU_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az);
uint8_t IMU_ReadGyro(int16_t *wx, int16_t *wy, int16_t *wz);

#endif /* INC_INTFC_IMU_H_ */
