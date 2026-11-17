#pragma once

#include <cstdint>

// 四路舵机的行程百分比：-100..100，0 为中位；硬件层换算成脉宽。
struct ArmCommand {
    int servo[4] = {};
};

// 把 PS2 摇杆原始值 0..255 换算成 -100..100，并扣除死区。
// 机械臂和底盘用不同的死区（见 car_config），因此单独实现一份。
int arm_axis_percent(uint8_t axis, bool invert);
