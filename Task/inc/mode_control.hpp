/**
 ******************************************************************************
 * @file           : mode_control.hpp
 * @brief          : 按键分发与急停组合（Task 层，单模式）
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

// v0.4 起只有一套操作：底盘与机械臂**同时**受手柄控制，没有模式切换。
//   左摇杆 → 底盘平移          L1/R1 → 底盘原地左/右转
//   右摇杆 → 机械臂末端速度    L2/R2 → 夹爪 松/紧     ←/→ → 腕 roll 左/右转
//   Start  → 回中立位          L3+R3 → 急停锁定
// 所以这里没有"当前模式"这种状态，也就没有切换时"机械臂该不该保持姿态"的问题。

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 一帧的控制输出
 */
struct ControlOutput {
    DriveCommand drive;
    ArmCommand arm;
    bool stopped = false; // 急停 latch 或手柄断线
    ArmFeedback arm_fb;   // 机械臂反馈
};

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化按键状态机与机械臂
 * @param now_ms 当前毫秒时基
 */
void mode_control_init(uint32_t now_ms);

/**
 * @brief 每轮主循环调用一次：边沿检测、急停组合、按键分发
 * @details 只产出"意图"（速度 / jog / 中立位）；平方律、限位、积分都在 arm_pose。
 * @param frame 本帧手柄数据
 * @param now_ms 当前毫秒时基
 * @return 本帧的底盘与机械臂指令；断线时返回停止状态并保持机械臂当前姿态（不回中）
 */
ControlOutput mode_control_update(const Ps2Frame &frame, uint32_t now_ms);
