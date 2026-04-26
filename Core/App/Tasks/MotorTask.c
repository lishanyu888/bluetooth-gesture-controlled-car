#include "cmsis_os2.h"
#include <stdbool.h>
#include <math.h>
#include "telemetry.h"
#include "tim.h"
#include "tb6612.h"

#define PID_DT_S 0.02f

typedef struct {
    float kp;
    float ki;
    float kd;
    float i_term;
    float prev_error;
    float i_limit;
    float out_limit;
} PID_t;

static int16_t speed_pct_to_pwm(uint8_t speed_pct, int16_t trim)
{
    const int16_t move_pwm_min = 22;
    const int16_t move_pwm_max = 78;
    int16_t pwm;

    if (speed_pct == 0U) {
        return 0;
    }

    pwm = (int16_t)(move_pwm_min + ((move_pwm_max - move_pwm_min) * (int16_t)speed_pct) / 100);
    pwm += trim;
    if (pwm < 0) {
        pwm = 0;
    }
    if (pwm > 100) {
        pwm = 100;
    }
    return pwm;
}

static float clampf(float value, float min_v, float max_v)
{
    if (value > max_v) {
        return max_v;
    }
    if (value < min_v) {
        return min_v;
    }
    return value;
}

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

static float pid_update(PID_t *pid, float error, float dt_s)
{
    float d_term;
    float out;

    pid->i_term += error * dt_s;
    pid->i_term = clampf(pid->i_term, -pid->i_limit, pid->i_limit);

    d_term = (error - pid->prev_error) / dt_s;
    pid->prev_error = error;

    out = pid->kp * error + pid->ki * pid->i_term + pid->kd * d_term;
    return clampf(out, -pid->out_limit, pid->out_limit);
}

static void pid_reset(PID_t *pid)
{
    pid->i_term = 0.0f;
    pid->prev_error = 0.0f;
}

static void set_motor_pwm_signed(int16_t pwm_l, int16_t pwm_r)
{
    uint8_t duty_l = (uint8_t)clampf((float)(pwm_l >= 0 ? pwm_l : -pwm_l), 0.0f, 100.0f);
    uint8_t duty_r = (uint8_t)clampf((float)(pwm_r >= 0 ? pwm_r : -pwm_r), 0.0f, 100.0f);

    if (pwm_l >= 0) {
        TB6612_Forward(ONLY_A, duty_l);
    } else {
        TB6612_Backward(ONLY_A, duty_l);
    }

    if (pwm_r >= 0) {
        TB6612_Forward(ONLY_B, duty_r);
    } else {
        TB6612_Backward(ONLY_B, duty_r);
    }
}

typedef enum {
    DRIVE_IDLE = 0,
    DRIVE_TURNING,
    DRIVE_MOVING
} DriveState_t;

static void put_latest_show(const ShowData_t *data)
{
    ShowData_t old_data;

    if (osMessageQueuePut(ShowQueueHandle, data, 0U, 0U) != osOK) {
        (void)osMessageQueueGet(ShowQueueHandle, &old_data, NULL, 0U);
        (void)osMessageQueuePut(ShowQueueHandle, data, 0U, 0U);
    }
}

//
// Created by 30714 on 2026/4/6.
//
void StartMotorTask(void *argument) {
    int32_t last_cnt_l;
    int32_t last_cnt_r;
    YawData_t yaw_data = {
            .yaw_deg = 0.0f,
            .status = 1U
    };
    ShowData_t show_data = {0};
    BtCmd_t bt_cmd = {0};
    uint8_t last_bt_action = 'N';
    int32_t last_bt_seconds = 0;
    int16_t cmd_angle_deg = 0;
    uint8_t cmd_speed_pct = 0;
    uint8_t has_new_cmd;
    DriveState_t drive_state = DRIVE_IDLE;
    int8_t move_dir = 1;
    int8_t turn_dir = 0;
    uint32_t move_end_tick = 0U;
    uint32_t post_turn_forward_ms = 0U;
    float turn_target_yaw = 0.0f;
    float yaw_ref = 0.0f;
    uint8_t yaw_ref_ready = 0U;
    PID_t yaw_pid = {
            .kp = 1.2f,
            .ki = 0.0f,
            .kd = 0.03f,
            .i_term = 0.0f,
            .prev_error = 0.0f,
            .i_limit = 30.0f,
            .out_limit = 20.0f
    };
    PID_t enc_pid = {
            .kp = 1.2f,
            .ki = 0.05f,
            .kd = 0.0f,
            .i_term = 0.0f,
            .prev_error = 0.0f,
            .i_limit = 50.0f,
            .out_limit = 20.0f
    };
    const int16_t pwm_trim_l = -1;
    const int16_t pwm_trim_r = 0;
    const int16_t turn_pwm_min = 20;
    const int16_t turn_pwm_max = 42;
    const uint32_t refresh_ms = 20U;

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    last_cnt_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    last_cnt_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    TB6612_Init(ALL);
    osDelay(100);

    for (;;) {
        int32_t cnt_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        int32_t cnt_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        int32_t dL = cnt_l - last_cnt_l;
        int32_t dR = cnt_r - last_cnt_r;
        int16_t pwm_l;
        int16_t pwm_r;
        int16_t move_base_l;
        int16_t move_base_r;
        int16_t turn_pwm;
        float yaw_error;
        float yaw_u = 0.0f;
        float enc_error;
        float enc_u;
        float total_u;

        while (osMessageQueueGet(YawQueueHandle, &yaw_data, NULL, 0U) == osOK) {
        }
        has_new_cmd = (uint8_t)(osMessageQueueGet(BtCmdQueueHandle, &bt_cmd, NULL, 0U) == osOK);

        if ((yaw_data.status == 0U) && (yaw_ref_ready == 0U)) {
            yaw_ref = yaw_data.yaw_deg;
            yaw_ref_ready = 1U;
        }

        if (has_new_cmd != 0U) {
            char action = (char)bt_cmd.action;
            last_bt_action = (uint8_t)action;
            last_bt_seconds = (int32_t)(bt_cmd.duration_ms / 1000U);
            cmd_speed_pct = bt_cmd.speed_pct;
            cmd_angle_deg = bt_cmd.angle_deg;
            if ((action == 'L') || (action == 'R')) {
                if (yaw_data.status == 0U) {
                    /* Left turn is positive yaw target, right turn is negative yaw target. */
                    turn_dir = (action == 'L') ? 1 : -1;
                    turn_target_yaw = wrap_angle_deg(yaw_data.yaw_deg + (float)(turn_dir * cmd_angle_deg));
                    post_turn_forward_ms = bt_cmd.duration_ms;
                    drive_state = DRIVE_TURNING;
                    yaw_ref_ready = 0U;
                } else {
                    drive_state = DRIVE_IDLE;
                }
            } else if ((action == 'S') || (action == 'B')) {
                move_dir = (action == 'S') ? 1 : -1;
                yaw_ref = yaw_data.yaw_deg;
                yaw_ref_ready = (uint8_t)(yaw_data.status == 0U);
                move_end_tick = osKernelGetTickCount() + bt_cmd.duration_ms;
                pid_reset(&yaw_pid);
                pid_reset(&enc_pid);
                drive_state = (bt_cmd.duration_ms > 0U) ? DRIVE_MOVING : DRIVE_IDLE;
            } else {
                drive_state = DRIVE_IDLE;
            }
        }

        last_cnt_l = cnt_l;
        last_cnt_r = cnt_r;

        pwm_l = 0;
        pwm_r = 0;
        move_base_l = speed_pct_to_pwm(cmd_speed_pct, pwm_trim_l);
        move_base_r = speed_pct_to_pwm(cmd_speed_pct, pwm_trim_r);
        turn_pwm = (int16_t)((turn_pwm_min * (100 - (int16_t)cmd_speed_pct) + turn_pwm_max * (int16_t)cmd_speed_pct) / 100);
        turn_pwm = (int16_t)clampf((float)turn_pwm, (float)turn_pwm_min, (float)turn_pwm_max);

        if (drive_state == DRIVE_TURNING) {
            float turn_err = wrap_angle_deg(turn_target_yaw - yaw_data.yaw_deg);
            if ((yaw_data.status == 0U) && (fabsf(turn_err) > 4.0f)) {
                pwm_l = (int16_t)(-turn_dir * turn_pwm);
                pwm_r = (int16_t)(turn_dir * turn_pwm);
            } else {
                set_motor_pwm_signed(0, 0);
                if (post_turn_forward_ms > 0U) {
                    move_dir = 1;
                    yaw_ref = yaw_data.yaw_deg;
                    yaw_ref_ready = (uint8_t)(yaw_data.status == 0U);
                    move_end_tick = osKernelGetTickCount() + post_turn_forward_ms;
                    pid_reset(&yaw_pid);
                    pid_reset(&enc_pid);
                    drive_state = DRIVE_MOVING;
                } else {
                    drive_state = DRIVE_IDLE;
                }
            }
        } else if (drive_state == DRIVE_MOVING) {
            if (((int32_t)(move_end_tick - osKernelGetTickCount())) <= 0) {
                drive_state = DRIVE_IDLE;
            } else {
                if (move_dir < 0) {
                    /* Reverse command: move straight backward without steering correction. */
                    pwm_l = (int16_t)(-move_base_l);
                    pwm_r = (int16_t)(-move_base_r);
                } else {
                    if ((yaw_data.status == 0U) && (yaw_ref_ready != 0U)) {
                        yaw_error = wrap_angle_deg(yaw_ref - yaw_data.yaw_deg);
                        yaw_u = pid_update(&yaw_pid, yaw_error, PID_DT_S);
                    } else {
                        yaw_u = 0.0f;
                    }

                    enc_error = (float)(dL - dR);
                    enc_u = pid_update(&enc_pid, enc_error, PID_DT_S);
                    total_u = yaw_u + enc_u;
                    {
                        float max_corr = 0.6f * ((float)(move_base_l + move_base_r) * 0.5f);
                        if (max_corr < 8.0f) {
                            max_corr = 8.0f;
                        }
                        total_u = clampf(total_u, -max_corr, max_corr);
                    }

                    pwm_l = (int16_t)clampf((float)move_dir * ((float)move_base_l - total_u), -100.0f, 100.0f);
                    pwm_r = (int16_t)clampf((float)move_dir * ((float)move_base_r + total_u), -100.0f, 100.0f);
                }
            }
        }
        set_motor_pwm_signed(pwm_l, pwm_r);

        show_data.delta_l = dL;
        show_data.delta_r = dR;
        show_data.yaw_deg = yaw_data.yaw_deg;
        show_data.pwm_l = pwm_l;
        show_data.pwm_r = pwm_r;
        show_data.bt_action = last_bt_action;
        show_data.bt_seconds = last_bt_seconds;
        show_data.status = yaw_data.status;
        put_latest_show(&show_data);

        osDelay(refresh_ms);
    }
}
