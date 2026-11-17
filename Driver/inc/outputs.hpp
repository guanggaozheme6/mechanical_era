/**
 ******************************************************************************
 * @file           : outputs.hpp
 * @brief          : 电机/舵机硬件输出与 PWM 启动（Driver 层）
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
#include "arm_control.hpp"

// 只做寄存器层面的动作：方向脚、TIM1 比较值、TIM8 舵机脉宽。
// 不认识手柄、不认识模式、不做闭环计算（闭环在 Device/motor）。

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化所有输出
 * @details 先初始化电机闭环与编码器，再把四路电机 PWM 与两路定时器启动到停止状态，
 *          最后把四路舵机写中位并启动。
 * @warning 启动失败直接进 Error_Handler，不要绕过。
 */
void init_outputs();

/**
 * @brief 下发四路电机指令
 * @details 先走 Device/motor 的速度闭环，再按换向需要先清零再换方向（H 桥保护），
 *          最后写 TIM1 比较值。
 * @param command 四路目标占空比（-100..100）
 */
void apply_motors(DriveCommand command);

/**
 * @brief 下发四路舵机指令
 * @details 行程百分比 → 脉宽（中位 1500 us，满行程 ±1000 us），带最后一道 ±100 钳位。
 * @param command 四路行程百分比
 */
void apply_servos(ArmCommand command);
