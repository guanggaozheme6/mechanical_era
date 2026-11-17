#include "algorithm/pid.hpp"

namespace {

float clamp(float value, float lower, float upper) {
    return value < lower ? lower : (value > upper ? upper : value);
}

} // namespace

void pid_configure(Pid &pid, float kp, float ki, float integral_limit, float output_min,
                   float output_max) {
    pid.kp = kp;
    pid.ki = ki;
    pid.integral_limit = integral_limit;
    pid.output_min = output_min;
    pid.output_max = output_max;
    pid.integral = 0.0F;
}

void pid_reset(Pid &pid) { pid.integral = 0.0F; }

float pid_update(Pid &pid, float error, float dt_s) {
    const float candidate_integral =
        clamp(pid.integral + error * dt_s, -pid.integral_limit, pid.integral_limit);
    const float requested = pid.kp * error + pid.ki * candidate_integral;
    const float output = clamp(requested, pid.output_min, pid.output_max);
    // 只有没饱和才采纳这次积分，避免积分项把输出顶在限幅上退不回来。
    if (requested == output) {
        pid.integral = candidate_integral;
    }
    return output;
}
