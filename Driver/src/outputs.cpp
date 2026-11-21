/**
 ******************************************************************************
 * @file           : outputs.cpp
 * @brief          : 电机/舵机硬件输出实现（Driver 层）
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "outputs.hpp"
#include "hal_handles.h"
#include "main.h"
#include "motor.hpp"
#include <algorithm>

/* Private typedef -----------------------------------------------------------*/
namespace {
/**
 * @brief 一路电机的引脚与定时器通道
 */
struct Motor {
    GPIO_TypeDef *a_port;
    uint16_t a_pin;
    GPIO_TypeDef *b_port;
    uint16_t b_pin;
    uint32_t channel;
};

/* Private variables ---------------------------------------------------------*/
const Motor kMotors[] = {
    {motor1_A_GPIO_Port, motor1_A_Pin, motor1_B_GPIO_Port, motor1_B_Pin, TIM_CHANNEL_1},
    {motor2_A_GPIO_Port, motor2_A_Pin, motor2_B_GPIO_Port, motor2_B_Pin, TIM_CHANNEL_2},
    {motor3_A_GPIO_Port, motor3_A_Pin, motor3_B_GPIO_Port, motor3_B_Pin, TIM_CHANNEL_3},
    {motor4_A_GPIO_Port, motor4_A_Pin, motor4_B_GPIO_Port, motor4_B_Pin, TIM_CHANNEL_4}};

constexpr uint32_t servo_channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

int previous_motor_duty[4] = {}; // 上一帧占空比，用于换向检测

/** @brief 夹爪这一路 PWM 是否已启动；false = 上电不给脉冲、舵机不受力 */
bool g_gripper_pwm_on = false;

/** @brief 夹爪在 servo[] 里的下标（与 arm_config::kGripperServo 一致） */
constexpr unsigned kGripperIndex = 2;
} // namespace

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 下发四路舵机行程百分比
 * @param command 四路行程百分比（-100..100）
 */
void apply_servos(ArmCommand command) {
    // TIM8 每 1 微秒计数一次（72 MHz / 72），因此比较值就是脉宽的微秒数。
    constexpr int kNeutralUs = 1500;
    constexpr int kSpanUs = 1000;
    for (unsigned i = 0; i < 4; ++i) {
        const int percent =
            command.servo[i] < -100 ? -100 : (command.servo[i] > 100 ? 100 : command.servo[i]);
        const int pulse_us = kNeutralUs + kSpanUs * percent / 100;
        __HAL_TIM_SET_COMPARE(&htim8, servo_channels[i], static_cast<uint32_t>(pulse_us));
        // 夹爪上电不给脉冲：舵机不上力就不会自己顶在“抓紧”位堵转。
        // 首次收到非零夹爪指令才启动这一路，之后常开（回中位 0 也照常给脉冲）。
        if (i == kGripperIndex && !g_gripper_pwm_on && percent != 0) {
            if (HAL_TIM_PWM_Start(&htim8, servo_channels[i]) != HAL_OK) {
                Error_Handler();
            }
            g_gripper_pwm_on = true;
        }
    }
}

/**
 * @brief 下发四路电机指令
 * @param command 四路目标占空比（-100..100）
 */
void apply_motors(DriveCommand command) {
    command = motor_update(command, HAL_GetTick());
    int requested[4];
    bool reversed[4];
    bool any_reversing = false;
    for (unsigned i = 0; i < 4; ++i) {
        requested[i] = std::max(-100, std::min(100, command.motor[i]));
        // 上一拍与这一拍都非零且异号才算换向；从停车起步（上一拍 0）不算。
        reversed[i] = requested[i] * previous_motor_duty[i] < 0;
        any_reversing |= reversed[i];
    }
    if (any_reversing) {
        // 只关需要换向的那几路：换向必须先断开 H 桥 2 ms，否则反电动势会灌进驱动。
        // 其余通道保持原占空比继续输出，避免一路换向把整车拖得顿一下。
        for (unsigned i = 0; i < 4; ++i) {
            if (!reversed[i]) {
                continue;
            }
            const Motor &motor = kMotors[i];
            __HAL_TIM_SET_COMPARE(&htim1, motor.channel, 0);
            HAL_GPIO_WritePin(motor.a_port, motor.a_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor.b_port, motor.b_pin, GPIO_PIN_RESET);
        }
        HAL_Delay(2);
    }
    for (unsigned i = 0; i < 4; ++i) {
        const Motor &motor = kMotors[i];
        const int duty = requested[i];
        if (!reversed[i] && duty == previous_motor_duty[i]) {
            continue;
        }
        HAL_GPIO_WritePin(motor.a_port, motor.a_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor.b_port, motor.b_pin, GPIO_PIN_RESET);
        if (duty != 0) {
            HAL_GPIO_WritePin(duty > 0 ? motor.a_port : motor.b_port,
                              duty > 0 ? motor.a_pin : motor.b_pin, GPIO_PIN_SET);
        }
        const uint32_t magnitude = static_cast<uint32_t>(duty < 0 ? -duty : duty);
        const uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);
        const uint32_t scaled = ((period + 1U) * magnitude) / 100U;
        const uint32_t compare = scaled > period ? period : scaled;
        __HAL_TIM_SET_COMPARE(&htim1, motor.channel, compare);
        previous_motor_duty[i] = duty;
    }
}

/**
 * @brief 初始化所有电机/舵机输出
 */
void init_outputs() {
    motor_init();
    // 先写入停止状态，再启动 PWM
    for (unsigned i = 0; i < 4; ++i) {
        const auto &motor = kMotors[i];
        __HAL_TIM_SET_COMPARE(&htim1, motor.channel, 0);
        HAL_GPIO_WritePin(motor.a_port, motor.a_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor.b_port, motor.b_pin, GPIO_PIN_RESET);
        previous_motor_duty[i] = 0;
    }
    for (const Motor &motor : kMotors) {
        if (HAL_TIM_PWM_Start(&htim1, motor.channel) != HAL_OK) {
            Error_Handler();
        }
    }
    apply_servos({});
    for (unsigned i = 0; i < 4; ++i) {
        if (i == kGripperIndex) {
            continue; // 夹爪上电不启动：见 apply_servos 的惰性启动
        }
        if (HAL_TIM_PWM_Start(&htim8, servo_channels[i]) != HAL_OK) {
            Error_Handler();
        }
    }
}
