#include "mpu6050.h"

#include "i2c_soft.h"

#include <math.h>
#include <stddef.h>

#define MPU6050_ADDR             0x68U
#define MPU6050_SMPLRT_DIV       0x19U
#define MPU6050_CONFIG           0x1AU
#define MPU6050_GYRO_CONFIG      0x1BU
#define MPU6050_ACCEL_CONFIG     0x1CU
#define MPU6050_ACCEL_XOUT_H     0x3BU
#define MPU6050_PWR_MGMT_1       0x6BU
#define MPU6050_WHO_AM_I         0x75U

#define MPU6050_ACCEL_SCALE      16384.0f
#define MPU6050_GYRO_SCALE       131.0f
#define MPU6050_TEMP_SCALE       340.0f
#define MPU6050_TEMP_OFFSET      36.53f
#define MPU6050_RAD_TO_DEG       57.2957795f
#define MPU6050_ALPHA            0.98f

static uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    return SoftI2C_WriteByte(MPU6050_ADDR, reg, data);
}

static uint8_t MPU6050_ReadReg(uint8_t reg, uint8_t *data)
{
    return SoftI2C_ReadByte(MPU6050_ADDR, reg, data);
}

static int16_t MPU6050_MakeWord(uint8_t high, uint8_t low)
{
    return (int16_t)((high << 8) | low);
}

uint8_t MPU6050_Init(void)
{
    uint8_t who_am_i;
    uint8_t status;

    SoftI2C_Init();

    status = MPU6050_TestConnection(&who_am_i);
    if ((status != 0U) || (who_am_i != MPU6050_ADDR)) {
        return 1U;
    }

    if (MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00U) != 0U) {
        return 2U;
    }
    if (MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07U) != 0U) {
        return 3U;
    }
    if (MPU6050_WriteReg(MPU6050_CONFIG, 0x06U) != 0U) {
        return 4U;
    }
    if (MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00U) != 0U) {
        return 5U;
    }
    if (MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00U) != 0U) {
        return 6U;
    }

    return 0U;
}

uint8_t MPU6050_TestConnection(uint8_t *who_am_i)
{
    if (who_am_i == NULL) {
        return 1U;
    }

    return MPU6050_ReadReg(MPU6050_WHO_AM_I, who_am_i);
}

uint8_t MPU6050_ReadRaw(MPU6050_RawData_t *raw)
{
    uint8_t buf[14];

    if (raw == NULL) {
        return 1U;
    }

    if (SoftI2C_ReadBytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buf, 14U) != 0U) {
        return 2U;
    }

    raw->accel_x = MPU6050_MakeWord(buf[0], buf[1]);
    raw->accel_y = MPU6050_MakeWord(buf[2], buf[3]);
    raw->accel_z = MPU6050_MakeWord(buf[4], buf[5]);
    raw->temp = MPU6050_MakeWord(buf[6], buf[7]);
    raw->gyro_x = MPU6050_MakeWord(buf[8], buf[9]);
    raw->gyro_y = MPU6050_MakeWord(buf[10], buf[11]);
    raw->gyro_z = MPU6050_MakeWord(buf[12], buf[13]);

    return 0U;
}

uint8_t MPU6050_ReadScaled(MPU6050_ScaledData_t *scaled)
{
    MPU6050_RawData_t raw;

    if (scaled == NULL) {
        return 1U;
    }

    if (MPU6050_ReadRaw(&raw) != 0U) {
        return 2U;
    }

    scaled->accel_x_g = (float)raw.accel_x / MPU6050_ACCEL_SCALE;
    scaled->accel_y_g = (float)raw.accel_y / MPU6050_ACCEL_SCALE;
    scaled->accel_z_g = (float)raw.accel_z / MPU6050_ACCEL_SCALE;
    scaled->gyro_x_dps = (float)raw.gyro_x / MPU6050_GYRO_SCALE;
    scaled->gyro_y_dps = (float)raw.gyro_y / MPU6050_GYRO_SCALE;
    scaled->gyro_z_dps = (float)raw.gyro_z / MPU6050_GYRO_SCALE;
    scaled->temperature_c = ((float)raw.temp / MPU6050_TEMP_SCALE) + MPU6050_TEMP_OFFSET;

    return 0U;
}

uint8_t MPU6050_CalcAccelAngle(const MPU6050_RawData_t *raw, MPU6050_Angle_t *angle)
{
    float accel_x;
    float accel_y;
    float accel_z;

    if ((raw == NULL) || (angle == NULL)) {
        return 1U;
    }

    accel_x = (float)raw->accel_x;
    accel_y = (float)raw->accel_y;
    accel_z = (float)raw->accel_z;

    angle->pitch = atan2f(accel_y, sqrtf((accel_x * accel_x) + (accel_z * accel_z))) * MPU6050_RAD_TO_DEG;
    angle->roll = atan2f(-accel_x, sqrtf((accel_y * accel_y) + (accel_z * accel_z))) * MPU6050_RAD_TO_DEG;

    return 0U;
}

uint8_t MPU6050_UpdateAngles(float dt_s, MPU6050_Angle_t *angle)
{
    MPU6050_RawData_t raw;
    MPU6050_Angle_t accel_angle;
    uint8_t status;

    if ((angle == NULL) || (dt_s <= 0.0f)) {
        return 1U;
    }

    status = MPU6050_ReadRaw(&raw);
    if (status != 0U) {
        return 2U;
    }

    status = MPU6050_CalcAccelAngle(&raw, &accel_angle);
    if (status != 0U) {
        return 3U;
    }

    angle->pitch = MPU6050_ALPHA * (angle->pitch + ((float)raw.gyro_x / MPU6050_GYRO_SCALE) * dt_s)
                 + (1.0f - MPU6050_ALPHA) * accel_angle.pitch;
    angle->roll = MPU6050_ALPHA * (angle->roll + ((float)raw.gyro_y / MPU6050_GYRO_SCALE) * dt_s)
                + (1.0f - MPU6050_ALPHA) * accel_angle.roll;

    return 0U;
}
