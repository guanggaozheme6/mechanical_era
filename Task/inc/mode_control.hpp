#pragma once

#include "arm_control.hpp"
#include "arm_pose.hpp"
#include "drive_control.hpp"
#include "ps2_protocol.hpp"
#include <cstdint>

// 手柄操作模式。两个直接入口（Start = 一、Select = 二），不是 toggle。
enum class OperationMode : uint8_t { Drive = 0, Arm };

struct ControlOutput {
    DriveCommand drive;
    ArmCommand arm;
    OperationMode mode = OperationMode::Drive;
    bool stopped = false; // 急停 latch 或手柄断线
    ArmFeedback arm_fb;   // 机械臂反馈（生产者是 arm_pose，这里只搬运）
};

void mode_control_init(uint32_t now_ms);

// 每轮主循环调用一次：边沿检测、急停组合、模式分发。
// 断线时返回停止状态并保持机械臂当前姿态（不回中）。
ControlOutput mode_control_update(const Ps2Frame &frame, uint32_t now_ms);
