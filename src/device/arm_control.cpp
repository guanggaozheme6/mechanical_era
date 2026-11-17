#include "device/arm_control.hpp"
#include "config/car_config.hpp"

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
