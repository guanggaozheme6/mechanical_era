/**
 ******************************************************************************
 * @file           : arm_control.cpp
 * @brief          : 机械臂摇杆换算实现（Device 层）
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "arm_control.hpp"
#include "car_config.hpp"

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 机械臂摇杆换算：0..255 → -100..100
 * @details 先扣 kArmDeadzone，再按半轴满量程（127 / 128）归一化；invert 决定极性。
 * @param axis 摇杆原始值（0..255，128 为中位）
 * @param invert 是否反转极性（left_y 推上 = 0x00，所以要反转）
 * @return -100..100 的行程百分比
 */
int arm_axis_percent(uint8_t axis, bool invert) {
    const int delta = static_cast<int>(axis) - 128;
    const int distance = delta < 0 ? -delta : delta;
    if (distance <= car_config::kArmDeadzone) {
        // 摇杆中位通常有 ±5..10 的漂移；不去掉会让舵机持续抖动发热。
        return 0;
    }
    // 负半轴和正半轴的满量程不同（0..127 与 128..255），分别归一化。
    const int full_scale = delta < 0 ? 128 : 127;
    const int usable = full_scale - car_config::kArmDeadzone;
    int scaled = (distance - car_config::kArmDeadzone) * 100 / usable;
    if (delta < 0) {
        scaled = -scaled;
    }
    return invert ? -scaled : scaled;
}
