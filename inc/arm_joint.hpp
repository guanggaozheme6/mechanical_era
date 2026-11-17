#pragma once

// ============================================================================
//  arm_joint —— 关节行程层（每路舵机一套状态机）
// ----------------------------------------------------------------------------
//  四路舵机（大臂 / 小臂 / 夹爪 / 腕 roll）走完全相同的代码：
//    target → 限位钳位 → jog 积分 → slew 限速逼近 → current
//  夹爪和腕不是特殊通道，不给它们另写一份 jog/限速（这是低冗余的关键）。
//
//  本模块吃 dt（秒），不读 HAL_GetTick、不 include 任何配置、不认识 ArmCommand，
//  所以 slew 时序、限位、hold、at_target 全部可以在 PC 上直接跑时序测试。
// ============================================================================

namespace arm_joint {

// 物理通道数（四路舵机）。机械事实，不是标定值；arm_config 用 static_assert 对齐。
constexpr unsigned kChannelCount = 4U;

// 行程限位（百分比）。两个解算关节由角度限位派生，夹爪/腕为 ±100。
struct Limits {
    float min_percent;
    float max_percent;
};

struct Joint {
    float target;                  // 期望行程 %，任何时刻都在 limits 内
    float current;                 // 实际下发行程 %（经 slew 限速）
    float jog_percent_per_second;  // 一次性速度，update 后自动清零
    Limits limits;
};

// 配置并落到某个初始值（上电用 current = target = percent）。
void configure(Joint &joint, Limits limits, float percent);

// 直接改目标（绝对行程 %），内部按 limits 钳位。
void set_target(Joint &joint, float percent);

// 给一帧速度（%/s）。不每帧重给就自动停 —— 松手即停的语义靠这个成立。
void set_jog(Joint &joint, float percent_per_second);

// 冻结：清掉 jog，target 保持不动。急停/断线时用。
void hold(Joint &joint);

// 推进一帧。dt_s 非正或非有限时只清 jog，不移动。
void update(Joint &joint, float dt_s, float slew_percent_per_second);

bool at_target(const Joint &joint, float epsilon_percent);

// 给 ArmCommand.servo[] 用的整数百分比（四舍五入；非有限值回落 0）。
int output_percent(const Joint &joint);

// ---- 四路一起处理 ----------------------------------------------------------
struct Bank {
    Joint channel[kChannelCount];
};

void bank_configure(Bank &bank, const Limits (&limits)[kChannelCount],
                    const float (&percent)[kChannelCount]);
void bank_update(Bank &bank, float dt_s, float slew_percent_per_second);
void bank_output(const Bank &bank, int (&out)[kChannelCount]);
bool bank_at_target(const Bank &bank, float epsilon_percent);
void bank_hold(Bank &bank);

} // namespace arm_joint
