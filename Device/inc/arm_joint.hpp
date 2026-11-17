/**
 ******************************************************************************
 * @file           : arm_joint.hpp
 * @brief          : 关节行程层：四路舵机的状态机（Device 层）
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

// 四路舵机（大臂 / 小臂 / 夹爪 / 腕 roll）走完全相同的代码：
//   target → 限位钳位 → jog 积分 → slew 限速逼近 → current
// 夹爪和腕不是特殊通道，不给它们另写一份 jog/限速。
//
// 本模块吃 dt（秒），不读 HAL_GetTick、不 include 任何配置、不认识 ArmCommand，
// 所以 slew 时序、限位、hold、at_target 全部可以在 PC 上直接跑时序测试。

namespace arm_joint {

/* Exported constants --------------------------------------------------------*/

/**
 * @brief 物理通道数（四路舵机）
 * @note 这是机械事实，不是标定值；arm_config 用 static_assert 对齐。
 */
constexpr unsigned kChannelCount = 4U;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 行程限位（百分比）
 * @note 两个解算关节由角度限位派生，夹爪/腕为 ±100。
 */
struct Limits {
    float min_percent;
    float max_percent;
};

/**
 * @brief 单路关节的状态
 * @note jog_percent_per_second 是一次性速度，update 后自动清零 —— 松手即停靠这个成立。
 */
struct Joint {
    float target;                 // 期望行程 %，任何时刻都在 limits 内
    float current;                // 实际下发行程 %（经 slew 限速）
    float jog_percent_per_second; // 一次性速度，update 后自动清零
    Limits limits;
};

/**
 * @brief 四路关节打包
 * @note 一个 Bank 就处理全部四路，避免三组平行数组带来的越界风险。
 */
struct Bank {
    Joint channel[kChannelCount];
};

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 配置并落到某个初始值
 * @param joint 关节
 * @param limits 行程限位
 * @param percent 初始行程百分比（会被钳进限位）
 */
void configure(Joint &joint, Limits limits, float percent);

/**
 * @brief 直接改目标（绝对行程 %）
 * @details 内部按 limits 钳位；非法目标回退到上一个合法 target。
 * @param joint 关节
 * @param percent 目标行程百分比
 */
void set_target(Joint &joint, float percent);

/**
 * @brief 给一帧速度（%/s）
 * @note 不每帧重给就自动停 —— 松手即停的语义靠这个成立。
 * @param joint 关节
 * @param percent_per_second 速度，单位 %/s
 */
void set_jog(Joint &joint, float percent_per_second);

/**
 * @brief 冻结：清掉 jog，target 保持不动
 * @details 急停 / 断线时用；解除后 update 会继续走向原目标。
 * @param joint 关节
 */
void hold(Joint &joint);

/**
 * @brief 推进一帧
 * @param joint 关节
 * @param dt_s 本帧时长（秒）；非正或非有限时只清 jog，不移动
 * @param slew_percent_per_second 限速（%/s）；≤0 或非有限视为冻结
 */
void update(Joint &joint, float dt_s, float slew_percent_per_second);

/**
 * @brief 是否已到目标
 * @param joint 关节
 * @param epsilon_percent 容差（行程百分比）
 * @return true 表示 |target - current| <= epsilon
 */
bool at_target(const Joint &joint, float epsilon_percent);

/**
 * @brief 取本帧要下发的整数行程百分比
 * @details 四舍五入；非有限值回落 0。
 * @param joint 关节
 * @return -100..100，供 ArmCommand.servo[] 用
 */
int output_percent(const Joint &joint);

/**
 * @brief 四路一起配置
 * @param bank 四路关节
 * @param limits 四路限位
 * @param percent 四路初始行程百分比
 */
void bank_configure(Bank &bank, const Limits (&limits)[kChannelCount],
                    const float (&percent)[kChannelCount]);

/**
 * @brief 四路一起推进一帧
 * @param bank 四路关节
 * @param dt_s 本帧时长（秒）
 * @param slew_percent_per_second 统一限速（%/s）
 */
void bank_update(Bank &bank, float dt_s, float slew_percent_per_second);

/**
 * @brief 四路一起取整数行程百分比
 * @param bank 四路关节
 * @param out 输出数组（4 项）
 */
void bank_output(const Bank &bank, int (&out)[kChannelCount]);

/**
 * @brief 四路是否都到目标
 * @param bank 四路关节
 * @param epsilon_percent 容差
 * @return true 表示四路都到位
 */
bool bank_at_target(const Bank &bank, float epsilon_percent);

/**
 * @brief 四路一起冻结
 * @param bank 四路关节
 */
void bank_hold(Bank &bank);

} // namespace arm_joint
