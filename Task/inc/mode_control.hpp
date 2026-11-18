/**
 ******************************************************************************
 * @file           : mode_control.hpp
 * @brief          : 模式状态机与按键分发（Task 层）
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "arm_control.hpp"
#include "arm_pose.hpp"
#include "drive_control.hpp"
#include "ps2_protocol.hpp"
#include <cstdint>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 手柄操作模式
 * @note 两个直接入口（Start = 一、Select = 二），不是 toggle。
 */
enum class OperationMode : uint8_t { Drive = 0, Arm };

/**
 * @brief 一帧的控制输出
 */
struct ControlOutput {
    DriveCommand drive;
    ArmCommand arm;
    OperationMode mode = OperationMode::Drive;
    bool stopped = false; // 急停 latch 或手柄断线
    ArmFeedback arm_fb;   // 机械臂反馈
};

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化模式状态机与机械臂
 * @param now_ms 当前毫秒时基
 */
void mode_control_init(uint32_t now_ms);

/**
 * @brief 每轮主循环调用一次：边沿检测、急停组合、模式分发
 * @details 只产出"意图"（速度 / 预设索引 / 模式）；平方律、限位、积分都在 arm_pose。
 * @param frame 本帧手柄数据
 * @param now_ms 当前毫秒时基
 * @return 本帧的底盘与机械臂指令；断线时返回停止状态并保持机械臂当前姿态（不回中）
 */
ControlOutput mode_control_update(const Ps2Frame &frame, uint32_t now_ms);
