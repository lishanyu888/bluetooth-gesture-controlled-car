#include "cmsis_os2.h"
#include "oled.h"
#include "telemetry.h"
#include <stdio.h>

//
// Created by 30714 on 2026/4/8.
//
void StartOLEDTask(void *argument) {
    char line1[24];
    char line2[24];
    char line3[24];
    char line4[24];
    ShowData_t show_data = {0};
    const uint32_t refresh_ms = 50U;

    osDelay(20);
    OLED_Init();

    for (;;) {
        (void)osMessageQueueGet(ShowQueueHandle, &show_data, NULL, refresh_ms);

        snprintf(line1, sizeof(line1), "dL:%5ld dR:%5ld", show_data.delta_l, show_data.delta_r);
        snprintf(line2, sizeof(line2), "Yaw:%8.2f", show_data.yaw_deg);
        snprintf(line3, sizeof(line3), "L:%4d R:%4d", show_data.pwm_l, show_data.pwm_r);
        snprintf(line4, sizeof(line4), "MPU:%3u BT:%c,%ld", show_data.status, (char)show_data.bt_action, show_data.bt_seconds);

        OLED_NewFrame();
        OLED_PrintASCIIString(0, 0, line1, (ASCIIFont *)&afont12x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(0, 16, line2, (ASCIIFont *)&afont12x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(0, 32, line3, (ASCIIFont *)&afont12x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(0, 48, line4, (ASCIIFont *)&afont12x6, OLED_COLOR_NORMAL);
        OLED_ShowFrame();

        osDelay(refresh_ms);
    }
}
