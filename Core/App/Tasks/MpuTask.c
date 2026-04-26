#include "cmsis_os2.h"
#include "i2c_soft.h"
#include "mpu6050.h"
#include "telemetry.h"

#define MPU6050_GYRO_SCALE_250DPS 131.0f

static float wrap_angle_deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static void put_latest_yaw(const YawData_t *data)
{
    YawData_t old_data;

    if (osMessageQueuePut(YawQueueHandle, data, 0U, 0U) != osOK) {
        (void)osMessageQueueGet(YawQueueHandle, &old_data, NULL, 0U);
        (void)osMessageQueuePut(YawQueueHandle, data, 0U, 0U);
    }
}

//
// Created by 30714 on 2026/4/16.
//
void StartMPUTask(void *argument) {
    MPU6050_RawData_t raw = {0};
    YawData_t yaw_data = {0};
    float yaw = 0.0f;
    uint32_t last_tick;
    uint8_t status;
    uint8_t retry;
    uint8_t read_fail_count = 0U;
    const uint16_t cali_samples = 200U;
    const uint16_t cali_delay_ms = 2U;
    const uint32_t period_ms = 20U;

    status = MPU6050_Init();
    if (status == 0U) {
        osDelay(100);
        for (retry = 0U; retry < 3U; retry++) {
            status = MPU6050_Calibrate(cali_samples, cali_delay_ms);
            if (status == 0U) {
                break;
            }
            osDelay(100);
        }
        if (status != 0U) {
            status = (uint8_t)(20U + status);
        }
    } else {
        status = (uint8_t)(10U + status);
    }

    yaw_data.status = status;
    yaw_data.yaw_deg = 0.0f;
    put_latest_yaw(&yaw_data);

    last_tick = osKernelGetTickCount();

    for (;;) {
        uint32_t now_tick = osKernelGetTickCount();
        float dt_s = (float)(now_tick - last_tick) / 1000.0f;

        if (dt_s <= 0.0f) {
            dt_s = (float)period_ms / 1000.0f;
        }
        last_tick = now_tick;

        if ((status == 0U) || (status >= 30U)) {
            uint8_t read_status = MPU6050_ReadRaw(&raw);
            if (read_status == 0U) {
                status = 0U;
                read_fail_count = 0U;
                MPU6050_ApplyCalibration(&raw);
                yaw += ((float)raw.gyro_z / MPU6050_GYRO_SCALE_250DPS) * dt_s;
                yaw = wrap_angle_deg(yaw);
            } else {
                read_fail_count++;
                SoftI2C_Recover();
                if (read_fail_count >= 3U) {
                    status = (uint8_t)(30U + read_status);
                }
            }
        }

        yaw_data.status = status;
        yaw_data.yaw_deg = yaw;
        put_latest_yaw(&yaw_data);

        osDelay(period_ms);
    }
}
