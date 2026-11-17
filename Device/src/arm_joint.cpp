/**
 ******************************************************************************
 * @file           : arm_joint.cpp
 * @brief          : 关节行程状态机实现（Device 层）
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "arm_joint.hpp"

#include <cmath>

namespace arm_joint {

/* Private functions ---------------------------------------------------------*/
namespace {

/**
 * @brief 按限位钳位，非法值回退
 * @details 非法目标回退到上一个合法 target，而不是 current —— 回到 current 会让
 *          "目标突然缩回原位"，回到旧 target 才是"忽略这次输入"。
 * @param joint 关节
 * @param percent 待钳位的目标
 * @param fallback 非法时使用的值
 * @return 钳位后的目标
 */
float clamp_to_limits(const Joint &joint, float percent, float fallback) {
    if (!std::isfinite(percent)) {
        return fallback; // 非法目标一律丢弃，回退到上一个合法值
    }
    if (percent < joint.limits.min_percent) {
        return joint.limits.min_percent;
    }
    if (percent > joint.limits.max_percent) {
        return joint.limits.max_percent;
    }
    return percent;
}

} // namespace

/* Exported functions --------------------------------------------------------*/

void configure(Joint &joint, Limits limits, float percent) {
    joint.limits = limits;
    joint.target = clamp_to_limits(joint, std::isfinite(percent) ? percent : 0.0F, 0.0F);
    joint.current = joint.target;
    joint.jog_percent_per_second = 0.0F;
}

void set_target(Joint &joint, float percent) {
    joint.target = clamp_to_limits(joint, percent, joint.target);
}

void set_jog(Joint &joint, float percent_per_second) {
    joint.jog_percent_per_second = std::isfinite(percent_per_second) ? percent_per_second : 0.0F;
}

void hold(Joint &joint) { joint.jog_percent_per_second = 0.0F; }

void update(Joint &joint, float dt_s, float slew_percent_per_second) {
    // dt 非法（0 / 负 / NaN / Inf）时本帧不推进，但 jog 仍然消费掉，
    // 与"jog 是一次性"的语义一致。
    if (!(dt_s > 0.0F) || !std::isfinite(dt_s)) {
        dt_s = 0.0F;
    }

    if (joint.jog_percent_per_second != 0.0F && dt_s > 0.0F) {
        joint.target = clamp_to_limits(joint, joint.target + joint.jog_percent_per_second * dt_s,
                                       joint.target);
    }
    joint.jog_percent_per_second = 0.0F;

    if (!(slew_percent_per_second > 0.0F) || !std::isfinite(slew_percent_per_second)) {
        return; // 不限速参数视为"冻结"，不会一步跳到目标
    }

    const float max_step = slew_percent_per_second * dt_s;
    const float diff = joint.target - joint.current;
    if (diff > max_step) {
        joint.current += max_step;
    } else if (diff < -max_step) {
        joint.current -= max_step;
    } else {
        joint.current = joint.target;
    }
}

bool at_target(const Joint &joint, float epsilon_percent) {
    const float diff = joint.target - joint.current;
    return (diff <= epsilon_percent) && (diff >= -epsilon_percent);
}

int output_percent(const Joint &joint) {
    const float value = joint.current;
    if (!std::isfinite(value)) {
        return 0;
    }
    const float clamped = value < -100.0F ? -100.0F : (value > 100.0F ? 100.0F : value);
    return static_cast<int>(clamped >= 0.0F ? clamped + 0.5F : clamped - 0.5F);
}

void bank_configure(Bank &bank, const Limits (&limits)[kChannelCount],
                    const float (&percent)[kChannelCount]) {
    for (unsigned i = 0; i < kChannelCount; ++i) {
        configure(bank.channel[i], limits[i], percent[i]);
    }
}

void bank_update(Bank &bank, float dt_s, float slew_percent_per_second) {
    for (unsigned i = 0; i < kChannelCount; ++i) {
        update(bank.channel[i], dt_s, slew_percent_per_second);
    }
}

void bank_output(const Bank &bank, int (&out)[kChannelCount]) {
    for (unsigned i = 0; i < kChannelCount; ++i) {
        out[i] = output_percent(bank.channel[i]);
    }
}

bool bank_at_target(const Bank &bank, float epsilon_percent) {
    for (unsigned i = 0; i < kChannelCount; ++i) {
        if (!at_target(bank.channel[i], epsilon_percent)) {
            return false;
        }
    }
    return true;
}

void bank_hold(Bank &bank) {
    for (unsigned i = 0; i < kChannelCount; ++i) {
        hold(bank.channel[i]);
    }
}

} // namespace arm_joint
