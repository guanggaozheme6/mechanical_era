/**
 ******************************************************************************
 * @file           : motor.cpp
 * @brief          : 四路电机速度闭环实现（Device 层）
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"
#include "encoder.hpp"
#include "pid.hpp"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
namespace {

constexpr uint32_t kUpdateMs = 10;        // 闭环周期
constexpr uint32_t kEncoderStallMs = 300; // 无计数多久判堵转
// 100% 目标速度对应每个 10 ms 控制周期的编码器计数。实车可在此处统一调速。
constexpr float kTicksAtFullSpeed = 20.0F;

// PID 参数属于电机本体，不属于算法层：换电机/换底盘只改这四个数。
constexpr float kProportionalGain = 3.0F;
constexpr float kIntegralGain = 8.0F;
constexpr float kIntegralLimit = 12.5F;
constexpr float kOutputMaxPercent = 100.0F;

/**
 * @brief 单路电机的闭环状态
 */
struct Motor {
    Pid pid;                  // Algorithm 层的纯 PID
    uint32_t last_motion_ms = 0; // 最近一次编码器有变化的时刻（堵转判定）
    int previous_target = 0;     // 上一拍目标（起步/换向时清积分）
    int target = 0;
    int encoder_ticks = 0;
    bool stalled = false;
    int output = 0;
};

Motor g_motor[4];
uint32_t g_last_update_ms = 0;

/**
 * @brief 取绝对值
 * @param value 输入
 * @return |value|
 */
int magnitude(int value) { return value < 0 ? -value : value; }

/**
 * @brief 推进单路电机一个控制周期
 * @details 停车 → 清积分；起步/换向 → 重新计堵转与积分；堵转 → 输出 0 并置标志；
 *          正常 → PID 推一拍并按目标符号输出。
 * @param index 电机序号 0..3
 * @param target 目标占空比（-100..100）
 * @param now_ms 当前毫秒时基
 * @return 该路实际输出占空比（-100..100）
 */
int update_one(unsigned index, int target, uint32_t now_ms) {
    Motor &motor = g_motor[index];
    const int ticks = magnitude(encoder::delta_ticks(index));
    motor.target = target;
    motor.encoder_ticks = ticks;

    if (target == 0) {
        pid_reset(motor.pid);
        motor.previous_target = 0;
        motor.output = 0;
        motor.stalled = false;
        motor.last_motion_ms = now_ms;
        return 0;
    }
    if (motor.previous_target == 0 || (target < 0) != (motor.previous_target < 0)) {
        // 从停车起步或换向时，重新开始堵转计时和积分。
        pid_reset(motor.pid);
        motor.last_motion_ms = now_ms;
        motor.stalled = false;
    }
    motor.previous_target = target;
    if (ticks != 0) {
        motor.last_motion_ms = now_ms;
    }
    if (now_ms - motor.last_motion_ms >= kEncoderStallMs) {
        pid_reset(motor.pid);
        motor.output = 0;
        motor.stalled = true;
        return 0;
    }

    const float desired_ticks = magnitude(target) * kTicksAtFullSpeed / 100.0F;
    const float error = desired_ticks - static_cast<float>(ticks);
    const float output = pid_update(motor.pid, error, static_cast<float>(kUpdateMs) / 1000.0F);
    motor.output = static_cast<int>(output + 0.5F);
    motor.stalled = false;
    return target < 0 ? -motor.output : motor.output;
}

} // namespace

/* Exported functions --------------------------------------------------------*/

void motor_init() {
    encoder::init();
    const uint32_t now_ms = HAL_GetTick();
    for (auto &motor : g_motor) {
        pid_configure(motor.pid, kProportionalGain, kIntegralGain, kIntegralLimit, 0.0F,
                      kOutputMaxPercent);
        motor.last_motion_ms = now_ms;
    }
    g_last_update_ms = now_ms - kUpdateMs;
}

DriveCommand motor_update(const DriveCommand &target, uint32_t now_ms) {
    DriveCommand output{};
    if (now_ms - g_last_update_ms < kUpdateMs) {
        // 周期间隔内沿用上一次的输出，方向跟随当前目标。
        for (unsigned i = 0; i < 4; ++i) {
            output.motor[i] = target.motor[i] == 0
                                  ? 0
                                  : (target.motor[i] < 0 ? -g_motor[i].output
                                                         : g_motor[i].output);
        }
        return output;
    }
    g_last_update_ms = now_ms;
    for (unsigned i = 0; i < 4; ++i) {
        output.motor[i] = update_one(i, target.motor[i], now_ms);
    }
    return output;
}

MotorStatus motor_status(unsigned motor) {
    if (motor >= 4) {
        return {};
    }
    const Motor &state = g_motor[motor];
    return {state.target, state.encoder_ticks, state.output, state.stalled};
}
