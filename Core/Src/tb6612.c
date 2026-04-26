/**
 * @file   tb6612.c
 * @brief  TB6612双H桥电机驱动器控制实现
 * @author myqfeng
 */

#include "tb6612.h"
#include "tim.h"

/** @brief 速度最值 */
#define MAX_SPEED 100
#define STOP_SPEED 0

/**
 * @brief 设置PWM占空比
 * @param duty 占空比值
 */
static inline void _SetPWM(const UseMotor motor, uint8_t duty)
{
    // 限制占空比范围
    if (duty > MAX_SPEED) duty = MAX_SPEED;

    switch (motor)
    {
    case ALL:
        __HAL_TIM_SET_COMPARE(PWMA_TIM, PWMA_TIM_CHANNEL, duty);
        __HAL_TIM_SET_COMPARE(PWMB_TIM, PWMB_TIM_CHANNEL, duty);
        break;
    case ONLY_A:
        __HAL_TIM_SET_COMPARE(PWMA_TIM, PWMA_TIM_CHANNEL, duty);
        break;
    case ONLY_B:
        __HAL_TIM_SET_COMPARE(PWMB_TIM, PWMB_TIM_CHANNEL, duty);
        break;
    default:
        break;
    }
}

/**
 * @brief 初始化TB6612
 * @param motor 要设置的电机接口
 */
void TB6612_Init(const UseMotor motor)
{
    // 初始化电机的 PWM 定时器
    if (motor == ONLY_A || motor == ALL)
    {
        // 先停止定时器，配置参数后再启动
        HAL_TIM_PWM_Stop(PWMA_TIM, PWMA_TIM_CHANNEL);
        __HAL_TIM_SET_PRESCALER(PWMA_TIM, 72-1);
        __HAL_TIM_SET_AUTORELOAD(PWMA_TIM, 100-1);
        __HAL_TIM_SET_COMPARE(PWMA_TIM, PWMA_TIM_CHANNEL, 0);
        HAL_TIM_PWM_Start(PWMA_TIM, PWMA_TIM_CHANNEL);
    }
    if (motor == ONLY_B || motor == ALL)
    {
        HAL_TIM_PWM_Stop(PWMB_TIM, PWMB_TIM_CHANNEL);
        __HAL_TIM_SET_PRESCALER(PWMB_TIM, 72-1);
        __HAL_TIM_SET_AUTORELOAD(PWMB_TIM, 100-1);
        __HAL_TIM_SET_COMPARE(PWMB_TIM, PWMB_TIM_CHANNEL, 0);
        HAL_TIM_PWM_Start(PWMB_TIM, PWMB_TIM_CHANNEL);
    }

    // 设置默认状态为停止(滑行)
    TB6612_Coast(motor);

}

/**
 * @brief 控制电机正转
 * @param motor 要设置的电机接口
 * @param speed 速度值（0-100）
 */
void TB6612_Forward(const UseMotor motor, uint8_t speed)
{
    switch (motor)
    {
    case ALL:
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        break;
    case ONLY_A:
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        break;
    case ONLY_B:
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        break;
    default:
        break;
    }
    _SetPWM(motor, speed);
}

/**
 * @brief 控制电机反转
 * @param motor 要设置的电机接口
 * @param speed 速度值（0-100）
 */
void TB6612_Backward(const UseMotor motor, uint8_t speed)
{
    switch (motor)
    {
    case ALL:
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        break;
    case ONLY_A:
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        break;
    case ONLY_B:
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        break;
    default:
        break;
    }
    _SetPWM(motor, speed);
}

/**
 * @brief 电机刹车
 * @param motor 要设置的电机接口
 */
void TB6612_Brake(const UseMotor motor)
{
    switch (motor)
    {
    case ALL:
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        break;
    case ONLY_A:
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        break;
    case ONLY_B:
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        break;
    default:
        break;
    }
    _SetPWM(motor, STOP_SPEED);//注意检查这里是0还是100
}

/**
 * @brief 电机滑行
 * @param motor 要设置的电机接口
 */
void TB6612_Coast(const UseMotor motor)
{
    switch (motor)
    {
    case ALL:
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        break;
    case ONLY_A:
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        break;
    case ONLY_B:
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        break;
    default:
        break;
    }
    _SetPWM(motor, STOP_SPEED);
}
