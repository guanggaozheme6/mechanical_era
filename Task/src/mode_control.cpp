/**
 ******************************************************************************
 * @file           : mode_control.cpp
 * @brief          : 按键分发与急停组合实现（Task 层，单模式）
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "mode_control.hpp"
#include "arm_pose.hpp"
#include "car_config.hpp"

// v0.4：没有模式切换，底盘和机械臂同时受控。按键分工只在这一个文件里定：
//   左摇杆 (left_x/left_y)      → drive_from_ps2（底盘平移）
//   L1 / R1                     → drive_from_ps2（底盘原地左/右转）
//   右摇杆 (right_x/right_y)    → arm_pose_set_cart_velocity（末端速度）
//   L2 / R2                     → arm_pose_joint_jog(2)：夹爪 松/紧
//   ← / →                       → arm_pose_joint_jog(3)：腕 roll 正=左转 / 负=右转
//   Start                       → arm_pose_request_neutral（回中立位）
//   L3 + R3                     → 急停锁定 / 解除
//   ↑ ↓ Select △ ○ ╳ □          → 无功能（v0.4 删掉了预设与模式键）

/* Private constants ---------------------------------------------------------*/
namespace {

/** @brief 夹爪 / 腕 roll 在关节层里的通道号（与 arm_config::k*Servo 一致） */
constexpr unsigned kGripperAxis = 2U;
constexpr unsigned kWristAxis = 3U;

/** @brief 腕 roll 方向约定：**正输出 = 左转**（实物反了就把这两个符号对调） */
constexpr int kWristLeftPercent = 100;
constexpr int kWristRightPercent = -100;

bool g_latched_stop = false; // L3+R3 锁定的急停，再按一次解除

// 边沿检测只用「上一帧按键位图」一个变量
uint16_t g_prev_buttons = 0;

// 急停组合的两段状态：arming = 同按计时中，fired = 已触发、等两键都松开。
bool g_combo_arming = false;
bool g_combo_fired = false;
uint32_t g_combo_since_ms = 0;

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 查询按键是否按下
 * @param frame 手柄帧
 * @param button 按键
 * @return true 表示按下
 */
bool pressed(const Ps2Frame &frame, Ps2Button button) { return ps2_pressed(frame, button); }

/**
 * @brief 上升沿检测（对上一帧按键位图）
 * @details PS2 20 ms 才刷新一次
 * @param frame 手柄帧
 * @param button 按键
 * @return true 表示本帧刚按下
 */
bool edge(const Ps2Frame &frame, Ps2Button button) {
    const auto bit = 1U << static_cast<unsigned>(button);
    return ps2_pressed(frame, button) && (g_prev_buttons & bit) == 0U;
}

/**
 * @brief 急停组合 L3+R3
 * @details 同按 >= kEstopHoldMs 才翻转 latch；触发后必须先都松开才能再触发。
 *          200 ms 而不是瞬时，是因为 PS2 20 ms 一帧、两键很少落在同一帧。
 * @param frame 手柄帧
 * @param now_ms 当前毫秒时基
 */
void update_estop_combo(const Ps2Frame &frame, uint32_t now_ms) {
    const bool both = pressed(frame, Ps2Button::L3) && pressed(frame, Ps2Button::R3);
    if (!both) {
        g_combo_arming = false;
        g_combo_fired = false;
        return;
    }
    if (g_combo_fired) {
        return;
    }
    if (!g_combo_arming) {
        g_combo_arming = true;
        g_combo_since_ms = now_ms;
        return;
    }
    if (now_ms - g_combo_since_ms >= static_cast<uint32_t>(car_config::kEstopHoldMs)) {
        g_latched_stop = !g_latched_stop;
        g_combo_fired = true;
    }
}

/**
 * @brief 机械臂输入：右摇杆给末端速度，扳机给夹爪，左右方向键给腕 roll
 * @details 右摇杆推右 = 前伸（+x），推上 = 抬升（+y）；平方律与合速度归一化在 arm_pose。
 * @param frame 手柄帧
 * @param hold_velocity true 表示本帧压住摇杆速度（按了 Start，或手柄退回数字模式）
 */
void update_arm_input(const Ps2Frame &frame, bool hold_velocity) {
    if (hold_velocity || !frame.analog) {
        arm_pose_set_cart_velocity(0.0F, 0.0F);
    } else {
        const int forward = arm_axis_percent(frame.right_x, false);
        const int lift = arm_axis_percent(frame.right_y, true);
        float vx = 0.0F;
        float vy = 0.0F;
        arm_pose_stick_velocity(forward, lift, vx, vy);
        arm_pose_set_cart_velocity(vx, vy);
    }

    // 夹爪 servo[2]：扳机 L2 松（正 = 张开）/ R2 紧；两键同按相抵。
    int gripper = 0;
    if (pressed(frame, Ps2Button::L2)) {
        gripper += 100;
    }
    if (pressed(frame, Ps2Button::R2)) {
        gripper -= 100;
    }
    if (gripper != 0) {
        arm_pose_joint_jog(kGripperAxis, gripper);
    }

    // 腕 roll servo[3]：← 左转 / → 右转；两键同按相抵。
    int wrist = 0;
    if (pressed(frame, Ps2Button::Left)) {
        wrist += kWristLeftPercent;
    }
    if (pressed(frame, Ps2Button::Right)) {
        wrist += kWristRightPercent;
    }
    if (wrist != 0) {
        arm_pose_joint_jog(kWristAxis, wrist);
    }
}

} // namespace

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化按键状态机与机械臂
 * @param now_ms 当前毫秒时基
 */
void mode_control_init(uint32_t now_ms) {
    g_latched_stop = false;
    g_prev_buttons = 0;
    g_combo_arming = false;
    g_combo_fired = false;
    g_combo_since_ms = 0;
    arm_pose_init(now_ms);
}

/**
 * @brief 每轮主循环调用一次：边沿检测、急停组合、按键分发
 * @details 只产出"意图"；平方律、限位、积分都在 arm_pose。
 *          急停 / 断线时底盘停、机械臂冻结在当前姿态（不回中、不松爪）。
 * @param frame 本帧手柄数据
 * @param now_ms 当前毫秒时基
 * @return 本帧的底盘与机械臂指令
 */
ControlOutput mode_control_update(const Ps2Frame &frame, uint32_t now_ms) {
    ControlOutput output;

    // 1. 全局：急停组合 + 断线。
    update_estop_combo(frame, now_ms);
    const bool stopped = g_latched_stop || !frame.connected;

    // 2. Start 回中立位：唯一保留的姿态动作（冻结时不响应）。
    const bool neutral_edge = !stopped && edge(frame, Ps2Button::Start);

    // 3. 底盘与机械臂各走各的，同一帧可以同时动（左手开车、右手摆臂）。
    if (stopped) {
        output.drive = DriveCommand{};
        output.arm = arm_pose_hold(now_ms);
    } else {
        output.drive = drive_from_ps2(frame);
        if (neutral_edge) {
            arm_pose_request_neutral();
        }
        update_arm_input(frame, neutral_edge);
        output.arm = arm_pose_update(now_ms);
    }
    output.arm_fb = arm_pose_feedback();

    output.stopped = stopped;
    g_prev_buttons = frame.buttons;
    return output;
}
