/**
 ******************************************************************************
 * @file           : outputs.hpp
 * @brief          : 电机/舵机硬件输出与 PWM 启动（Driver 层）
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "drive_control.hpp"
#include "arm_control.hpp"

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化所有电机/舵机输出
 * @details 先初始化电机闭环与编码器，再把四路电机 PWM 与两路定时器启动到停止状态，
 *          最后把四路舵机写中位并启动。
 */
void init_outputs();

/**
 * @brief 下发四路电机指令
 * @param command 四路目标占空比（-100..100）
 */
void apply_motors(DriveCommand command);

/**
 * @brief 下发四路舵机指令
 * @param command 四路行程百分比
 */
void apply_servos(ArmCommand command);
