#include "mpu6050.h"

#include "delay.h"

#include <stddef.h>

#define MPU6050_GRAVITY_LSB_2G 16384.0f

static MPU6050_Calib_t g_calib = {0};

uint8_t MPU6050_Calibrate(uint16_t sample_count, uint16_t sample_delay_ms)
{
    uint32_t i;
    float sum_ax = 0.0f;
    float sum_ay = 0.0f;
    float sum_az = 0.0f;
    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;
    MPU6050_RawData_t raw;

    if (sample_count == 0U) {
        return 1U;
    }

    g_calib.accel_x_offset = 0.0f;
    g_calib.accel_y_offset = 0.0f;
    g_calib.accel_z_offset = 0.0f;
    g_calib.gyro_x_offset = 0.0f;
    g_calib.gyro_y_offset = 0.0f;
    g_calib.gyro_z_offset = 0.0f;

    for (i = 0U; i < sample_count; i++) {
        if (MPU6050_ReadRaw(&raw) != 0U) {
            return 2U;
        }

        sum_ax += (float)raw.accel_x;
        sum_ay += (float)raw.accel_y;
        sum_az += (float)raw.accel_z;
        sum_gx += (float)raw.gyro_x;
        sum_gy += (float)raw.gyro_y;
        sum_gz += (float)raw.gyro_z;

        if (sample_delay_ms > 0U) {
            Delay_Ms(sample_delay_ms);
        }
    }

    g_calib.accel_x_offset = sum_ax / (float)sample_count;
    g_calib.accel_y_offset = sum_ay / (float)sample_count;
    g_calib.accel_z_offset = (sum_az / (float)sample_count) - MPU6050_GRAVITY_LSB_2G;
    g_calib.gyro_x_offset = sum_gx / (float)sample_count;
    g_calib.gyro_y_offset = sum_gy / (float)sample_count;
    g_calib.gyro_z_offset = sum_gz / (float)sample_count;

    return 0U;
}

void MPU6050_GetCalibration(MPU6050_Calib_t *calib)
{
    if (calib == NULL) {
        return;
    }
    *calib = g_calib;
}

void MPU6050_ApplyCalibration(MPU6050_RawData_t *raw)
{
    if (raw == NULL) {
        return;
    }

    raw->accel_x = (int16_t)((float)raw->accel_x - g_calib.accel_x_offset);
    raw->accel_y = (int16_t)((float)raw->accel_y - g_calib.accel_y_offset);
    raw->accel_z = (int16_t)((float)raw->accel_z - g_calib.accel_z_offset);
    raw->gyro_x = (int16_t)((float)raw->gyro_x - g_calib.gyro_x_offset);
    raw->gyro_y = (int16_t)((float)raw->gyro_y - g_calib.gyro_y_offset);
    raw->gyro_z = (int16_t)((float)raw->gyro_z - g_calib.gyro_z_offset);
}
