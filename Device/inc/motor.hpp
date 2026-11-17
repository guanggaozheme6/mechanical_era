/**
 ******************************************************************************
 * @file           : motor.hpp
 * @brief          : 四路电机速度闭环（Device 层）
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "drive_control.hpp"
#include <cstdint>

// 对外只暴露「给我 -100..100 的目标，我还你 -100..100 的 PWM 输出」。
// 内部：读编码器增量（Driver/encoder）→ 算误差 → pid（Algorithm）→ 抗饱和 →
//       堵转判定 → 10 ms 调度。上层不认识编码器、PID 或任何定时器。

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 单路电机的闭环状态（只读，给串口诊断用）
 */
struct MotorStatus {
    int target = 0;        // 目标占空比（-100..100）
    int encoder_ticks = 0; // 本控制周期的编码器计数（绝对值）
    int output = 0;        // PID 输出（0..100，方向另算）
    bool stalled = false;  // 有目标但编码器 300 ms 无变化
};

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化四路编码器与 PID 参数
 * @note 由 init_outputs() 调用，必须先于任何 motor_update()。
 */
void motor_init();

/**
 * @brief 推一拍闭环
 * @details 10 ms 一个控制周期；周期间隔内沿用上一次的输出（方向不变、幅值保持）。
 * @param target 四路目标占空比（-100..100，0 = 停车并清积分）
 * @param now_ms 当前毫秒时基（由调用方给，内部不读 HAL_GetTick）
 * @return 四路实际要写进 PWM 的占空比
 */
DriveCommand motor_update(const DriveCommand &target, uint32_t now_ms);

/**
 * @brief 查询某路电机的闭环状态
 * @param motor 电机序号 0..3
 * @return 状态快照；序号越界返回默认值
 */
MotorStatus motor_status(unsigned motor);
