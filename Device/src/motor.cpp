#include "motor.hpp"
#include "encoder.hpp"
#include "pid.hpp"
#include "main.h"

namespace {

constexpr uint32_t kUpdateMs = 10;
constexpr uint32_t kEncoderStallMs = 300;
// 100% 目标速度对应每个 10 ms 控制周期的编码器计数。实车可在此处统一调速。
constexpr float kTicksAtFullSpeed = 20.0F;

// PID 参数属于电机本体，不属于算法层：换电机/换底盘只改这四个数。
constexpr float kProportionalGain = 3.0F;
constexpr float kIntegralGain = 8.0F;
constexpr float kIntegralLimit = 12.5F;
constexpr float kOutputMaxPercent = 100.0F;

struct Motor {
    Pid pid;
    uint32_t last_motion_ms = 0;
    int previous_target = 0;
    int target = 0;
    int encoder_ticks = 0;
    bool stalled = false;
    int output = 0;
};

Motor g_motor[4];
uint32_t g_last_update_ms = 0;

int magnitude(int value) { return value < 0 ? -value : value; }

int update_one(unsigned index, int target, uint32_t now_ms) {
    Motor &motor = g_motor[index];
    const int ticks = magnitude(encoder::delta_ticks(index));
    motor.target = target;
    motor.encoder_ticks = ticks;

    if (target == 0) {
        pid_reset(motor.pid);
        motor.previous_target = 0;
        motor.output = 0;
        motor.stalled = false;
        motor.last_motion_ms = now_ms;
        return 0;
    }
    if (motor.previous_target == 0 || (target < 0) != (motor.previous_target < 0)) {
        // 从停车起步或换向时，重新开始堵转计时和积分。
        pid_reset(motor.pid);
        motor.last_motion_ms = now_ms;
        motor.stalled = false;
    }
    motor.previous_target = target;
    if (ticks != 0) {
        motor.last_motion_ms = now_ms;
    }
    if (now_ms - motor.last_motion_ms >= kEncoderStallMs) {
        pid_reset(motor.pid);
        motor.output = 0;
        motor.stalled = true;
        return 0;
    }

    const float desired_ticks = magnitude(target) * kTicksAtFullSpeed / 100.0F;
    const float error = desired_ticks - static_cast<float>(ticks);
    const float output = pid_update(motor.pid, error, static_cast<float>(kUpdateMs) / 1000.0F);
    motor.output = static_cast<int>(output + 0.5F);
    motor.stalled = false;
    return target < 0 ? -motor.output : motor.output;
}

} // namespace

void motor_init() {
    encoder::init();
    const uint32_t now_ms = HAL_GetTick();
    for (auto &motor : g_motor) {
        pid_configure(motor.pid, kProportionalGain, kIntegralGain, kIntegralLimit, 0.0F,
                      kOutputMaxPercent);
        motor.last_motion_ms = now_ms;
    }
    g_last_update_ms = now_ms - kUpdateMs;
}

DriveCommand motor_update(const DriveCommand &target, uint32_t now_ms) {
    DriveCommand output{};
    if (now_ms - g_last_update_ms < kUpdateMs) {
        // 周期间隔内沿用上一次的输出，方向跟随当前目标。
        for (unsigned i = 0; i < 4; ++i) {
            output.motor[i] = target.motor[i] == 0
                                  ? 0
                                  : (target.motor[i] < 0 ? -g_motor[i].output
                                                         : g_motor[i].output);
        }
        return output;
    }
    g_last_update_ms = now_ms;
    for (unsigned i = 0; i < 4; ++i) {
        output.motor[i] = update_one(i, target.motor[i], now_ms);
    }
    return output;
}

MotorStatus motor_status(unsigned motor) {
    if (motor >= 4) {
        return {};
    }
    const Motor &state = g_motor[motor];
    return {state.target, state.encoder_ticks, state.output, state.stalled};
}
