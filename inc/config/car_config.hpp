#pragma once

namespace car_config {
enum class MecanumLayout { X, O };
// 俯视时四个麦轮的滚子组成 O 型。
constexpr MecanumLayout kMecanumLayout = MecanumLayout::O;
enum class Wheel : unsigned { FrontLeft = 0, FrontRight, RearLeft, RearRight };
// M1～M4 的轮位已经通过单电机校准确认。
constexpr Wheel kMotorWheel[4] = {Wheel::FrontLeft, Wheel::FrontRight, Wheel::RearRight,
                                  Wheel::RearLeft};
// +1 表示 A=1、B=0 时车轮朝车头方向滚动，否则使用 -1。
// 已按实车测试反转 M1、M2；所有电机的正方向现使用 +1。
constexpr int kMotorPolarity[4] = {1, 1, 1, 1};
constexpr int kMaxDutyPercent = 100;
constexpr int kButtonDutyPercent = 80;
constexpr int kAxisDeadzone = 16;
// L2/R2 原地自转的占空比；两键同按 = 相抵为 0。实测太猛就往下调。
constexpr int kRotateDutyPercent = 80;
// 急停组合 L3+R3 需要同按的时长（ms）。PS2 20 ms 一帧，两键很少落在同一帧，
// 瞬时判定会时灵时不灵；200 ms 是本工程唯一的安全妥协。
constexpr int kEstopHoldMs = 200;

// ---- 机械臂操作参数 ----------------------------------------------------------
// 舵机摇杆死区。比底盘小，因为机械臂需要更精细的微调。
// 关节速度、插值速度、限位、预设表都在 inc/arm_config.hpp，不在本文件。
constexpr int kArmDeadzone = 8;

static_assert(kMaxDutyPercent > 0 && kMaxDutyPercent <= 100, "Invalid PWM limit");
static_assert(kButtonDutyPercent > 0 && kButtonDutyPercent <= kMaxDutyPercent,
              "Invalid button speed");
static_assert(kAxisDeadzone >= 0 && kAxisDeadzone < 127, "Invalid stick deadzone");
static_assert(kRotateDutyPercent > 0 && kRotateDutyPercent <= kMaxDutyPercent,
              "Invalid rotate duty");
static_assert(kEstopHoldMs >= 20, "Estop hold window must cover at least one PS2 frame");
static_assert(kArmDeadzone >= 0 && kArmDeadzone < 127, "Invalid arm deadzone");

}
