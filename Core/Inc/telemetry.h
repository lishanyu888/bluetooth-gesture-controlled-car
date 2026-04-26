#ifndef CAR_TELEMETRY_H
#define CAR_TELEMETRY_H

#include "cmsis_os2.h"
#include <stdint.h>

typedef struct {
    float yaw_deg;
    uint8_t status;
} YawData_t;

typedef struct {
    float yaw_deg;
    int32_t delta_l;
    int32_t delta_r;
    int16_t pwm_l;
    int16_t pwm_r;
    uint8_t bt_action;
    int32_t bt_seconds;
    uint8_t status;
} ShowData_t;

typedef struct {
    uint8_t action;
    int16_t angle_deg;
    uint8_t speed_pct;
    uint32_t duration_ms;
} BtCmd_t;

extern osMessageQueueId_t YawQueueHandle;
extern osMessageQueueId_t ShowQueueHandle;
extern osMessageQueueId_t BtCmdQueueHandle;

#endif /* CAR_TELEMETRY_H */
