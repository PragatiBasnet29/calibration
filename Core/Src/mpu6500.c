/*
 * MPU6500.c
 *
 *  Created on: May 26, 2026
 *      Author: PRAGATI
 *
 */

#include "MPU6500.h"

float accel_offset[3] = {0};
float gyro_offset[3] = {0};

uint8_t MPU6500_TEST() {

	uint8_t addr = WHO_AM_I_MPU6500_ADDR | 0x80;
	uint8_t val;

	disable_mpu6500();
	enable_mpu6500();

	HAL_SPI_Transmit(SPI_SENSOR, &addr, 1, 1000);
	HAL_SPI_Receive(SPI_SENSOR, &val, 1, 1000);

	disable_mpu6500();

	return val;
}

/* MPU6500 CS LOW */
void enable_mpu6500(void)
{
    HAL_GPIO_WritePin(mpusensor_NSS_CS_GPIO_Port,
                      mpusensor_NSS_CS_Pin,
                      GPIO_PIN_RESET);
}

/* MPU6500 CS HIGH */
void disable_mpu6500(void)
{
    HAL_GPIO_WritePin(mpusensor_NSS_CS_GPIO_Port,
                      mpusensor_NSS_CS_Pin,
                      GPIO_PIN_SET);
}

void MPU6500_Read_Accel(int16_t *Ax,
                        int16_t *Ay,
                        int16_t *Az)
{
    uint8_t addr = ACCEL_XOUT_H | 0x80;
    uint8_t rx[6];

    disable_mpu6500();
    enable_mpu6500();

    HAL_SPI_Transmit(SPI_SENSOR,
                     &addr,
                     1,
                     1000);

    HAL_SPI_Receive(SPI_SENSOR,
                    rx,
                    6,
                    1000);

    disable_mpu6500();

    *Ax = (int16_t)(rx[0] << 8 | rx[1]);
    *Ay = (int16_t)(rx[2] << 8 | rx[3]);
    *Az = (int16_t)(rx[4] << 8 | rx[5]);

    accel_g[0] = ((*Ax) / 16384.0f) - accel_offset[0];
    accel_g[1] = ((*Ay) / 16384.0f) - accel_offset[1];
    accel_g[2] = ((*Az) / 16384.0f) - accel_offset[2];

}

void MPU6500_Read_Gyro(int16_t *Gx,
                       int16_t *Gy,
                       int16_t *Gz)
{
    uint8_t addr = GYRO_XOUT_H | 0x80;
    uint8_t rx[6];

    disable_mpu6500();
    enable_mpu6500();

    HAL_SPI_Transmit(SPI_SENSOR,
                     &addr,
                     1,
                     1000);

    HAL_SPI_Receive(SPI_SENSOR,
                    rx,
                    6,
                    1000);

    disable_mpu6500();

    *Gx = (int16_t)(rx[0] << 8 | rx[1]);
    *Gy = (int16_t)(rx[2] << 8 | rx[3]);
    *Gz = (int16_t)(rx[4] << 8 | rx[5]);

    gyro_dps[0] = ((*Gx) / 131.0f) - gyro_offset[0];
    gyro_dps[1] = ((*Gy) / 131.0f) - gyro_offset[1];
    gyro_dps[2] = ((*Gz) / 131.0f) - gyro_offset[2];
}

void MPU6500_Calibrate(void)
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    float ax_sum = 0;
    float ay_sum = 0;
    float az_sum = 0;

    float gx_sum = 0;
    float gy_sum = 0;
    float gz_sum = 0;

    int samples = 1000;

    for(int i = 0; i < samples; i++)
    {
        MPU6500_Read_Accel(&ax, &ay, &az);
        MPU6500_Read_Gyro(&gx, &gy, &gz);

        ax_sum += ax / 16384.0f;
        ay_sum += ay / 16384.0f;
        az_sum += az / 16384.0f;

        gx_sum += gx / 131.0f;
        gy_sum += gy / 131.0f;
        gz_sum += gz / 131.0f;

        HAL_Delay(10);
    }

    accel_offset[0] = ax_sum / samples;
    accel_offset[1] = ay_sum / samples;

    /*
       Z axis should be +1g while flat
       so remove extra error only
    */
    accel_offset[2] = (az_sum / samples) - 1.0f;

    gyro_offset[0] = gx_sum / samples;
    gyro_offset[1] = gy_sum / samples;
    gyro_offset[2] = gz_sum / samples;
}
