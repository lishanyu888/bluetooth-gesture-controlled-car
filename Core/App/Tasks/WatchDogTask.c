#include "cmsis_os2.h"
#include "iwdg.h"

extern osThreadId_t MPUTaskHandle;
extern osThreadId_t MotorTaskHandle;
extern osThreadId_t OLEDTaskHandle;
extern osThreadId_t BT24TaskHandle;

static uint8_t thread_is_healthy(osThreadId_t thread)
{
    osThreadState_t state;

    if (thread == NULL) {
        return 0U;
    }

    state = osThreadGetState(thread);
    if ((state == osThreadInactive) || (state == osThreadTerminated) || (state == osThreadError)) {
        return 0U;
    }
    return 1U;
}

static uint8_t system_is_healthy(void)
{
    if (thread_is_healthy(MPUTaskHandle) == 0U) {
        return 0U;
    }
    if (thread_is_healthy(MotorTaskHandle) == 0U) {
        return 0U;
    }
    if (thread_is_healthy(OLEDTaskHandle) == 0U) {
        return 0U;
    }
    if (thread_is_healthy(BT24TaskHandle) == 0U) {
        return 0U;
    }
    return 1U;
}
//
// Created by 30714 on 2026/4/21.
//
void StartWatchDogTask(void *argument) {
    const uint32_t feed_period_ms = 500U;

    for (;;) {
        if (system_is_healthy() != 0U) {
            (void)HAL_IWDG_Refresh(&hiwdg);
        }
        osDelay(feed_period_ms);
    }
}
