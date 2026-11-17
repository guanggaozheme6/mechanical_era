/**
 ******************************************************************************
 * @file           : drive_control.cpp
 * @brief          : 麦轮混控与模式一按键映射实现（Device 层）
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drive_control.hpp"

/* Private functions ---------------------------------------------------------*/
namespace {

/**
 * @brief 取绝对值
 * @param value 输入
 * @return |value|
 */
int magnitude(int value) { return value < 0 ? -value : value; }

/**
 * @brief 编译期校验轮位与极性配置
 * @details 每个轮位只能出现一次，极性只能是 ±1。
 * @return 配置合法返回 true
 */
constexpr bool valid_configuration() {
    unsigned seen = 0;
    for (unsigned i = 0; i < 4; ++i) {
        const unsigned wheel = static_cast<unsigned>(car_config::kMotorWheel[i]);
        if (wheel >= 4 || (seen & (1U << wheel)) ||
            (car_config::kMotorPolarity[i] != 1 && car_config::kMotorPolarity[i] != -1)) {
            return false;
        }
        seen |= 1U << wheel;
    }
    return seen == 15U;
}
static_assert(valid_configuration(), "Assign every wheel once and use polarity +1/-1");

/**
 * @brief 底盘摇杆换算：0..255 → -100..100，含死区与半轴归一化
 * @param value 摇杆原始值
 * @param invert 是否反转极性
 * @return -100..100 的占空比
 */
int axis(uint8_t value, bool invert) {
    const int delta = static_cast<int>(value) - 128;
    const int distance = magnitude(delta);
    if (distance <= car_config::kAxisDeadzone) {
        return 0;
    }
    const int full_scale = delta < 0 ? 128 : 127;
    int scaled = (distance - car_config::kAxisDeadzone) * car_config::kMaxDutyPercent /
                 (full_scale - car_config::kAxisDeadzone);
    if (delta < 0) {
        scaled = -scaled;
    }
    return invert ? -scaled : scaled;
}
} // namespace

/* Exported functions --------------------------------------------------------*/

DriveCommand mix_drive(int forward, int strafe, int rotation, car_config::MecanumLayout layout) {
    // O 型与 X 型的横向滚子方向相反。轮序为左前、右前、左后、右后。
    // 旋转指令仍表示归一化的左右差速量，不是角速度。
    if (layout == car_config::MecanumLayout::O) {
        strafe = -strafe;
    }
    int wheel[4] = {forward + strafe + rotation, forward - strafe - rotation,
                    forward - strafe + rotation, forward + strafe - rotation};
    int peak = car_config::kMaxDutyPercent;
    for (int speed : wheel) {
        if (magnitude(speed) > peak) {
            peak = magnitude(speed);
        }
    }
    DriveCommand result{};
    for (unsigned i = 0; i < 4; ++i) {
        result.motor[i] = wheel[static_cast<unsigned>(car_config::kMotorWheel[i])] *
                          car_config::kMaxDutyPercent / peak * car_config::kMotorPolarity[i];
    }
    return result;
}

DriveCommand drive_from_ps2(const Ps2Frame &frame) {
    // 这里只挡断线：停车判定的唯一真值在 mode_control（急停 latch + 断线）。
    // Cross 现在是预设 P6、Select 是模式键，按下的那一刻不能再把底盘清零，
    // 否则「按 Select 进机械臂模式」时底盘会抽搐一下。
    if (!frame.connected) {
        return {};
    }
    const bool up = ps2_pressed(frame, Ps2Button::Up);
    const bool down = ps2_pressed(frame, Ps2Button::Down);
    const bool left = ps2_pressed(frame, Ps2Button::Left);
    const bool right = ps2_pressed(frame, Ps2Button::Right);

    int forward = frame.analog ? axis(frame.left_y, true) : 0;
    int strafe = frame.analog ? axis(frame.left_x, false) : 0;
    // L2 逆时针 / R2 顺时针自转；两键同按相抵为 0。右摇杆整条通道已删。
    const int rotation =
        (static_cast<int>(ps2_pressed(frame, Ps2Button::R2)) -
         static_cast<int>(ps2_pressed(frame, Ps2Button::L2))) *
        car_config::kRotateDutyPercent;

    if (up || down || left || right) {
        forward = (static_cast<int>(up) - static_cast<int>(down)) * car_config::kButtonDutyPercent;
        strafe =
            (static_cast<int>(right) - static_cast<int>(left)) * car_config::kButtonDutyPercent;
    }
    return mix_drive(forward, strafe, rotation);
}
