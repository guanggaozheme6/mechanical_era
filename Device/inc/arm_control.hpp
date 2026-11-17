/**
 ******************************************************************************
 * @file           : arm_control.hpp
 * @brief          : 舵机指令类型与机械臂摇杆换算（Device 层）
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
#include <cstdint>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 四路舵机的行程百分比指令
 * @note 每路取值 -100..100，0 为中位；硬件层（Driver/outputs）换算成脉宽。
 * @note 这是 Task 与 Device 之间唯一的指令类型，不含任何几何信息。
 */
struct ArmCommand {
    int servo[4] = {};
};

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 把 PS2 摇杆原始值 0..255 换算成 -100..100，并扣除死区
 * @details 机械臂和底盘用不同的死区（见 car_config 的 kArmDeadzone / kAxisDeadzone），
 *          因此单独实现一份，不复用 drive_control 里的 axis()。
 * @param axis 摇杆原始值（0..255，128 为中位）
 * @param invert 是否反转极性（left_y 推上 = 0x00，所以要反转才能得到"向上为正"）
 * @return -100..100 的行程百分比
 */
int arm_axis_percent(uint8_t axis, bool invert);
