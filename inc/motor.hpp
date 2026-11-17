#pragma once

#include "drive_control.hpp"
#include <cstdint>

// ============================================================================
//  motor —— Device 层：四路电机的速度闭环
// ----------------------------------------------------------------------------
//  对外只暴露「给我 -100..100 的目标，我还你 -100..100 的 PWM 输出」。
//  内部：读编码器增量（encoder 驱动）→ 算误差 → pid（算法）→ 抗饱和 →
//        堵转判定 → 10 ms 调度。上层不认识编码器、PID 或任何定时器。
// ============================================================================

void motor_init();

// 10 ms 一个控制周期；周期间隔内沿用上一次的输出（方向不变、幅值保持）。
DriveCommand motor_update(const DriveCommand &target, uint32_t now_ms);

struct MotorStatus {
    int target = 0;
    int encoder_ticks = 0;
    int output = 0;
    bool stalled = false;
};

MotorStatus motor_status(unsigned motor);
