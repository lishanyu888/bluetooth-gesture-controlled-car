/**
 * @brief 适用于 STM32 HAL 库的 TB6612 电机驱动
 * @author Myqfeng
 * @date 2026-03-06
 * @note 参考了 Keysking 的 DRV8833 电机驱动库以及 wzl713 的 TB6612 电机驱动库，实现了更加完善的功能
 ******************************************
 * @attention 使用方法：
 *   1. 包含 tb6612.h 头文件
 *   2. 修改 tb6612.h 文件，根据实际情况更改 GPIO 引脚定义和定时器通道定义
 *   3. 调用 TB6612_Init() 函数初始化 TB6612 电机驱动
 *   4. 初始化完成后，即可调用 TB6612_Forward()、TB6612_Backward()、TB6612_Brake()、TB6612_Coast() 等函数控制电机
 *
 * @warning 特别提示：
 *   1. 需确保在HAL库中完成定时器和 GPIO 的初始化，定时器使用的是PWM模式，GPIO 引脚需要配置为推挽输出
 *   2. 初始化哪个电机接口就使用哪个电机接口，否则可能出现错误。
 */


#ifndef TB6612_H_
#define TB6612_H_

#include <stdint.h>
#include "main.h"

// A通道 GPIO 引脚定义 - 需要根据实际情况修改
#define AIN1_GPIO_Port TB6612_AIN1_GPIO_Port
#define AIN2_GPIO_Port TB6612_AIN2_GPIO_Port
#define AIN1_Pin TB6612_AIN1_Pin
#define AIN2_Pin TB6612_AIN2_Pin

// B通道 GPIO 引脚定义 - 需要根据实际情况修改
#define BIN1_GPIO_Port TB6612_BIN1_GPIO_Port
#define BIN2_GPIO_Port TB6612_BIN2_GPIO_Port
#define BIN1_Pin TB6612_BIN1_Pin
#define BIN2_Pin TB6612_BIN2_Pin

// PWMA/B 使用的定时器和通道定义 - 需要根据实际情况修改
#define PWMA_TIM &htim1
#define PWMA_TIM_CHANNEL TIM_CHANNEL_4
#define PWMB_TIM &htim1
#define PWMB_TIM_CHANNEL TIM_CHANNEL_1

/**
 * @brief 用于选择需要设置的电机接口
 * @param ONLY_A 仅设置A电机接口
 * @param ONLY_B 仅设置B电机接口
 * @param ALL 设置A电机接口和B电机接口
 */
typedef enum
{
    ONLY_A,
    ONLY_B,
    ALL,
} UseMotor;

/**
 * @brief 初始化TB6612
 * @param motor 要设置的电机接口
 */
void TB6612_Init(UseMotor motor);

/**
 * @brief 控制电机正转
 * @param motor 要设置的电机接口
 * @param speed 速度值（0-100）
 */
void TB6612_Forward(UseMotor motor, uint8_t speed);

/**
 * @brief 控制电机反转
 * @param motor 要设置的电机接口
 * @param speed 速度值（0-100）
 */
void TB6612_Backward(UseMotor motor, uint8_t speed);

/**
 * @brief 电机刹车
 * @param motor 要设置的电机接口
 */
void TB6612_Brake(UseMotor motor);

/**
 * @brief 电机滑行
 * @param motor 要设置的电机接口
 */
void TB6612_Coast(UseMotor motor);

#endif /* TB6612_H_ */
