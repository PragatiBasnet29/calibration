/*
 * MPU6500.h
 *
 *  Created on: May 26, 2026
 *      Author: PRAGATI BASNET
 */

#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_

#include "main.h"

#define ACCEL_XOUT_H  0x3B //accelerometer X-axis HIGH byte register is located at address 0x3B
#define GYRO_XOUT_H   0x43 //gyroscope X-axis HIGH byte register is located at address 0x43


#define WHO_AM_I_MPU6500_ADDR				0x75
#define WHO_AM_I_MPU6500_REG_DEFAULT		0x70

extern SPI_HandleTypeDef hspi1;
extern float accel_g[3];
extern float gyro_dps[3];
extern float accel_offset[3];
extern float gyro_offset[3];

void MPU6500_Calibrate(void);

uint8_t MPU6500_TEST();
void enable_mpu6500();
void disable_mpu6500();

void MPU6500_Read_Accel(int16_t *Ax, int16_t *Ay, int16_t *Az);

void MPU6500_Read_Gyro(int16_t *Gx, int16_t *Gy, int16_t *Gz);

#endif /* INC_MPU6500_H_ */
