/**
 ******************************************************************************
 * @file           : diagnostics.hpp
 * @brief          : 串口状态上报（Task 层）
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
#include "arm_control.hpp"
#include "arm_pose.hpp"
#include "drive_control.hpp"

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 上报一条 PS2 + 底盘状态（500 ms 一次）
 * @details 顺手把 M4 的 PID 与 TIM1/GPIO 实配也打出来，方便逐项对照"请求的占空比 vs
 *          实际寄存器"。mode= 是 PS2 协议模式字，op= 才是操作模式。
 * @param frame 本帧手柄数据
 * @param command 本帧实际下发的四路电机指令
 * @param emergency_stop 板载 Key 的急停状态
 * @param key_pressed 板载 Key 当前电平（去抖后）
 * @param arm_mode true 打 op=ARM，false 打 op=DRV
 */
void report_status(const Ps2Frame &frame, DriveCommand command, bool emergency_stop,
                   bool key_pressed, bool arm_mode);

/**
 * @brief 上报一条机械臂状态（500 ms 一次）
 * @details 前 4 个字段 S1..S4 的位置一个字节都不动（标定手法依赖它），后面追加
 *          xy / tgt / th / ok / lim / wall / poses / bu。
 * @param command 本帧实际下发的四路舵机指令
 * @param feedback 解算与状态反馈
 */
void report_arm(const ArmCommand &command, const ArmFeedback &feedback);
