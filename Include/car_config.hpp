/**
 ******************************************************************************
 * @file           : car_config.hpp
 * @brief          : 底盘与摇杆参数（唯一真值）
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

namespace car_config {

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 麦轮布局
 * @details O 型与 X 型的横向滚子方向相反，混控里两者平移符号相反。
 */
enum class MecanumLayout { X, O };

/**
 * @brief 当前麦轮布局
 * @note 俯视时四个麦轮的滚子组成 O 型。
 */
constexpr MecanumLayout kMecanumLayout = MecanumLayout::O;

/**
 * @brief 轮位枚举
 */
enum class Wheel : unsigned { FrontLeft = 0, FrontRight, RearLeft, RearRight };

/**
 * @brief M1～M4 四路电机各自对应的轮位
 */
constexpr Wheel kMotorWheel[4] = {Wheel::FrontLeft, Wheel::FrontRight, Wheel::RearRight,
                                  Wheel::RearLeft};

/**
 * @brief 每路电机的方向校正系数
 * @details +1：A=1、B=0 时车轮朝车头方向滚动；-1：反之。
 */
constexpr int kMotorPolarity[4] = {1, 1, 1, 1};

/**
 * @brief 摇杆满偏时的 PWM 占空比上限（%）
 */
constexpr int kMaxDutyPercent = 100;

/**
 * @brief 底盘摇杆死区（原始 0..255 计数）
 */
constexpr int kAxisDeadzone = 16;

/**
 * @brief 肩键 L1/R1 原地自转的占空比（%）
 * @note 两键同按相抵为 0。扳机 L2/R2 已改为夹爪，不再参与底盘。
 */
constexpr int kRotateDutyPercent = 80;

/**
 * @brief 急停组合 L3+R3 需要同按的时长（ms）
 * @details PS2 20 ms 一帧，两键很少落在同一帧，因此不能用瞬时判定。
 */
constexpr int kEstopHoldMs = 200;

/**
 * @brief 机械臂摇杆死区
 * @note 比底盘小，因为机械臂需要更精细的微调。
 * @note 关节速度、插值速度、限位、预设表都在 arm_config.hpp，不在本文件。
 */
constexpr int kArmDeadzone = 8;

/* Exported macro ------------------------------------------------------------*/

static_assert(kMaxDutyPercent > 0 && kMaxDutyPercent <= 100, "Invalid PWM limit");
static_assert(kAxisDeadzone >= 0 && kAxisDeadzone < 127, "Invalid stick deadzone");
static_assert(kRotateDutyPercent > 0 && kRotateDutyPercent <= kMaxDutyPercent,
              "Invalid rotate duty");
static_assert(kEstopHoldMs >= 20, "Estop hold window must cover at least one PS2 frame");
static_assert(kArmDeadzone >= 0 && kArmDeadzone < 127, "Invalid arm deadzone");

} // namespace car_config
