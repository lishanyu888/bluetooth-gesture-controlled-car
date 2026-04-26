#ifndef CAR_MPU6050_H
#define CAR_MPU6050_H

#include <stdint.h>

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU6050_RawData_t;

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temperature_c;
} MPU6050_ScaledData_t;

typedef struct {
    float pitch;
    float roll;
} MPU6050_Angle_t;

typedef struct {
    float accel_x_offset;
    float accel_y_offset;
    float accel_z_offset;
    float gyro_x_offset;
    float gyro_y_offset;
    float gyro_z_offset;
} MPU6050_Calib_t;

uint8_t MPU6050_Init(void);
uint8_t MPU6050_TestConnection(uint8_t *who_am_i);
uint8_t MPU6050_ReadRaw(MPU6050_RawData_t *raw);
uint8_t MPU6050_ReadScaled(MPU6050_ScaledData_t *scaled);
uint8_t MPU6050_CalcAccelAngle(const MPU6050_RawData_t *raw, MPU6050_Angle_t *angle);
uint8_t MPU6050_UpdateAngles(float dt_s, MPU6050_Angle_t *angle);
uint8_t MPU6050_Calibrate(uint16_t sample_count, uint16_t sample_delay_ms);
void MPU6050_GetCalibration(MPU6050_Calib_t *calib);
void MPU6050_ApplyCalibration(MPU6050_RawData_t *raw);

#endif /* CAR_MPU6050_H */
