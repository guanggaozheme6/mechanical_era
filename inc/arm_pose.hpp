#pragma once

#include "arm_control.hpp"
#include <cstdint>

// ============================================================================
//  arm_pose —— 编排层：预设姿态 + 笛卡尔会话 + 反馈聚合
// ----------------------------------------------------------------------------
//  这一层只做"谁调谁"：
//    预设/坐标请求 → arm_kinematics::solve_xy → arm_joint::set_target
//    左摇杆速度   → FK(当前) → 矩形钳位 → IK → arm_joint::set_target
//    夹爪/腕速度  → arm_joint::set_jog（不经 IK）
//    每帧          → arm_joint::bank_update → ArmCommand
//  不含三角函数公式、不含积分公式、不含限位表。
//
//  **没有"单关节 jog 模式"**：轴 0/1 由笛卡尔解算独占驱动，只留下夹爪(2) 和
//  腕 roll(3) 的手动 jog 入口。标定期改为编译开关 ARM_BRINGUP（见 arm_pose.cpp）。
//
//  时间由外部传入（now_ms），内部不读 HAL_GetTick —— 所以整层可以在 PC 上跑。
// ============================================================================

// 机械臂反馈，给 mode_control 汇总、给串口打印。
// 放在 arm_pose.hpp（生产者一侧）而不是 mode_control.hpp：mode_control 只是搬运工。
struct ArmFeedback {
    float x_mm = 0.0F;          // 当前末端（FK 回算，基于 current）
    float y_mm = 0.0F;
    float target_x_mm = 0.0F;   // 最近一次解算用的笛卡尔目标（已含下垂补偿）
    float target_y_mm = 0.0F;
    float shoulder_deg = 0.0F;  // 当前连杆角（标定反推用）
    float elbow_deg = 0.0F;
    bool reachable = true;      // 圆环内（没被径向钳位）
    bool limited = false;       // 关节角被结构限位裁剪
    bool at_wall = false;       // 目标被工作区矩形钳位 / 被拒绝
    bool bringup = false;       // 当前固件是 ARM_BRINGUP 标定版
    unsigned bad_poses = 0U;    // 上电自检失败的预设位掩码
};

// 记录时基，四路归零到中位，并对 8 条预设各做一次 IK 校验。
void arm_pose_init(uint32_t now_ms);

// 请求第 index 条预设（0..7，见 arm_config::Pose）。表驱动，不再有 8 个 if。
// 内部先清笛卡尔速度，否则"按了预设先动一下再被拉回"。
// 索引越界或该条上电自检失败时不做任何动作，失败原因进 ArmFeedback。
void arm_pose_request(unsigned index);

// 绝对坐标请求（动作序列 / 上位机）。返回 false = 被钳位或超限，未下发。
bool arm_pose_request_xy(float x_mm, float y_mm);

// 每帧给一次末端速度（mm/s），松手传 0。
void arm_pose_set_cart_velocity(float vx_mm_per_s, float vy_mm_per_s);

// 摇杆百分比 → 末端速度：平方律 + 双轴满偏归一化。
// 纯函数，给 mode_control 一行调用，也给 PC 单测直接验证。
void arm_pose_stick_velocity(int forward_percent, int lift_percent, float &vx_mm_per_s,
                             float &vy_mm_per_s);

// 单关节速度，单位是"摇杆百分比"（-100..100），机械臂层乘该路满杆速度。
// **只接受 axis 2（夹爪）和 3（腕 roll）**；0/1 返回 false 且不产生任何动作，
// 因为两个解算关节只能由笛卡尔入口驱动（甲方要求删掉所有单关节手动入口）。
// 与 arm_pose_set_cart_velocity 可以在同一帧共存（左手摆位 + 右手开合夹爪）。
bool arm_pose_joint_jog(unsigned axis, int stick_percent);

ArmCommand arm_pose_update(uint32_t now_ms);
ArmCommand arm_pose_hold(uint32_t now_ms);

int arm_pose_value(unsigned joint);
bool arm_pose_at_target();
unsigned arm_pose_gripper_servo();
ArmFeedback arm_pose_feedback();

// 上电自检结果：bit i = 第 i 条预设不可用。
unsigned arm_pose_bad_pose_mask();
