/**
 ******************************************************************************
 * @file           : arm_pose.hpp
 * @brief          : 机械臂编排层：预设姿态、笛卡尔会话与反馈聚合（Device 层）
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "arm_control.hpp"
#include <cstdint>

// 这一层只做"谁调谁"：
//   预设/坐标请求 → arm_kinematics::solve_xy → arm_joint::set_target
//   左摇杆速度   → FK(当前) → 矩形钳位 → IK → arm_joint::set_target
//   夹爪/腕速度  → arm_joint::set_jog（不经 IK）
//   每帧          → arm_joint::bank_update → ArmCommand
// 不含三角函数公式、不含积分公式、不含限位表。
//
// 没有"单关节 jog 模式"：轴 0/1 由笛卡尔解算独占驱动，只留下夹爪(2) 和腕 roll(3)
// 的手动 jog 入口。标定期改为编译开关 ARM_BRINGUP（见 arm_pose.cpp）。
//
// 时间由外部传入（now_ms），内部不读 HAL_GetTick —— 所以整层可以在 PC 上跑。

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 机械臂反馈，给 mode_control 汇总、给串口打印
 * @note 放在 arm_pose.hpp（生产者一侧）而不是 mode_control.hpp：mode_control 只是搬运工。
 */
struct ArmFeedback {
    float x_mm = 0.0F;         // 当前末端（FK 回算，基于 current）
    float y_mm = 0.0F;         //
    float target_x_mm = 0.0F;  // 最近一次解算用的笛卡尔目标（已含下垂补偿）
    float target_y_mm = 0.0F;  //
    float shoulder_deg = 0.0F; // 当前连杆角（标定反推用）
    float elbow_deg = 0.0F;    //
    bool reachable = true;     // 圆环内（没被径向钳位 = 精确命中）
    bool limited = false;      // θ1 被结构限位裁剪（该方位够不到，未下发）
    bool at_wall = false;      // 目标被钳到边界（已下发）；与 limited 互斥
    bool bringup = false;      // 当前固件是 ARM_BRINGUP 标定版
    unsigned bad_poses = 0U;   // 上电自检失败的预设位掩码
};

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化：记时基、四路归零到中位、对 8 条预设各做一次 IK 校验
 * @param now_ms 当前毫秒时基
 */
void arm_pose_init(uint32_t now_ms);

/**
 * @brief 请求第 index 条预设（0..7，见 arm_config::Pose）
 * @details 表驱动，不再有 8 个 if。进门先清笛卡尔速度，否则"按了预设先动一下再被拉回"。
 *          坐标越界时会被钳到边界照发；只有该方位 θ1 越限、或该条上电自检没过时才不动。
 * @note 索引越界、或该条上电自检失败时不做任何动作，失败原因进 ArmFeedback。
 * @param index 预设索引 0..7
 */
void arm_pose_request(unsigned index);

/**
 * @brief 绝对坐标请求（动作序列 / 上位机）
 * @details 越界不拒绝：先钳到工作区矩形、再沿方位缩到可控圆环边界，然后照样下发。
 * @param x_mm 目标 x（mm）
 * @param y_mm 目标 y（mm）
 * @return true 表示精确命中；false 表示已钳到边界后下发，或该方位越限够不到（未下发）
 */
bool arm_pose_request_xy(float x_mm, float y_mm);

/**
 * @brief 每帧给一次末端速度（mm/s），松手传 0
 * @param vx_mm_per_s 末端 x 方向速度
 * @param vy_mm_per_s 末端 y 方向速度
 */
void arm_pose_set_cart_velocity(float vx_mm_per_s, float vy_mm_per_s);

/**
 * @brief 摇杆百分比 → 末端速度：平方律 + 双轴满偏归一化
 * @details 纯函数，给 mode_control 一行调用，也给 PC 单测直接验证。
 * @param forward_percent 前伸方向摇杆百分比（-100..100）
 * @param lift_percent 抬升方向摇杆百分比（-100..100）
 * @param vx_mm_per_s 输出：末端 x 速度
 * @param vy_mm_per_s 输出：末端 y 速度
 */
void arm_pose_stick_velocity(int forward_percent, int lift_percent, float &vx_mm_per_s,
                             float &vy_mm_per_s);

/**
 * @brief 单关节速度，单位是"摇杆百分比"（-100..100）
 * @details 机械臂层乘该路满杆速度。只接受 axis 2（夹爪）和 3（腕 roll）；
 *          与 arm_pose_set_cart_velocity 可以在同一帧共存（左手摆位 + 右手开合夹爪）。
 * @param axis 通道号，只允许 2 / 3
 * @param stick_percent 摇杆百分比（-100..100）
 * @return 0/1 返回 false 且不产生任何动作，因为两个解算关节只能由笛卡尔入口驱动
 */
bool arm_pose_joint_jog(unsigned axis, int stick_percent);

/**
 * @brief 推进一帧：解算 + 关节插值 + 刷新反馈
 * @param now_ms 当前毫秒时基
 * @return 本帧要下发的四路行程百分比
 */
ArmCommand arm_pose_update(uint32_t now_ms);

/**
 * @brief 冻结一帧：保持当前姿态（急停 / 断线用）
 * @param now_ms 当前毫秒时基（仍然要推进，否则解除急停第一帧会跳）
 * @return 当前四路行程百分比
 */
ArmCommand arm_pose_hold(uint32_t now_ms);

/**
 * @brief 取某一路当前的整数行程百分比
 * @param joint 关节序号 0..3
 * @return -100..100；越界返回 0
 */
int arm_pose_value(unsigned joint);

/**
 * @brief 四路是否都已到达目标
 * @return true 表示四路都在 settle 容差内
 */
bool arm_pose_at_target();

/**
 * @brief 夹爪接在哪一路舵机上
 * @return 通道号（arm_config::kGripperServo）
 */
unsigned arm_pose_gripper_servo();

/**
 * @brief 取最新反馈快照
 * @return ArmFeedback 副本
 */
ArmFeedback arm_pose_feedback();

/**
 * @brief 上电自检结果
 * @return bit i = 第 i 条预设不可用（0 = 八条全好）
 */
unsigned arm_pose_bad_pose_mask();
