/**
 ******************************************************************************
 * @file           : drive_control.hpp
 * @brief          : 麦轮底盘混控与按键平移/自转（Device 层）
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "ps2_protocol.hpp"
#include "car_config.hpp"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 四路电机的占空比指令
 * @note 按实际电机 M1～M4 排列，已包含轮位映射和方向校正；取值 -100..100。
 */
struct DriveCommand {
    int motor[4] = {};
};

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 麦轮逆运动学混控
 * @details 前进、向右平移、顺时针旋转分别为对应轴的正方向。采用开环归一化 PWM 混合；
 *          旋转项是差速量，单位不是弧度每秒。
 * @param forward 前进分量（-100..100）
 * @param strafe 向右平移分量（-100..100）
 * @param rotation 顺时针自转分量（-100..100）
 * @param layout 麦轮布局，决定横向符号
 * @return 四路电机占空比
 */
DriveCommand mix_drive(int forward, int strafe, int rotation,
                       car_config::MecanumLayout layout = car_config::kMecanumLayout);

/**
 * @brief 手柄 → 底盘指令（模式一）
 * @details 左摇杆平移、L2/R2 自转、方向键 80% 覆盖平移。只挡断线；
 *          停车判定的唯一真值在 mode_control（急停 latch + 断线）。
 * @param frame 解码后的手柄帧
 * @return 四路电机占空比；断线时全零
 */
DriveCommand drive_from_ps2(const Ps2Frame &frame);
