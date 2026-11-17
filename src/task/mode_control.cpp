#include "task/mode_control.hpp"
#include "device/arm_pose.hpp"
#include "config/car_config.hpp"

namespace {

OperationMode g_mode = OperationMode::Drive;
bool g_latched_stop = false; // L3+R3 锁定的急停，再按一次解除

// 边沿检测只用「上一帧按键位图」一个变量。旧版为每个键维护 g_xxx_prev，
// 加一个键要改四处，还会漏同步（resync_arm_edges）。
uint16_t g_prev_buttons = 0;

// 急停组合的两段状态：arming = 同按计时中，fired = 已触发、等两键都松开。
bool g_combo_arming = false;
bool g_combo_fired = false;
uint32_t g_combo_since_ms = 0;

bool pressed(const Ps2Frame &frame, Ps2Button button) { return ps2_pressed(frame, button); }

bool edge(const Ps2Frame &frame, Ps2Button button) {
    const auto bit = 1U << static_cast<unsigned>(button);
    return ps2_pressed(frame, button) && (g_prev_buttons & bit) == 0U;
}

// 预设绑定表：pose 字段就是 arm_config::Pose 的索引（0..7）。
// 表驱动，加第 9 条只加一行，不在 mode_control 里写分支。
struct PresetBind {
    Ps2Button button;
    unsigned pose;
};

constexpr PresetBind kPresetBinds[] = {
    {Ps2Button::Up, 0},       // ↑ P0 Home
    {Ps2Button::Down, 1},     // ↓ P1 Ready
    {Ps2Button::Left, 2},     // ← P2 Reach
    {Ps2Button::Right, 3},    // → P3 Retract
    {Ps2Button::Triangle, 4}, // △ P4 Lift
    {Ps2Button::Circle, 5},   // ○ P5 Carry
    {Ps2Button::Cross, 6},    // ╳ P6 Place
    {Ps2Button::Square, 7},   // □ P7 Drop
};

constexpr unsigned kPresetCount = sizeof(kPresetBinds) / sizeof(kPresetBinds[0]);
static_assert(kPresetCount == 8U, "预设绑定表与 8 个按键不一致");

// 急停：L3+R3 同按 >= kEstopHoldMs 才翻转 latch；触发后必须先都松开才能再触发。
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

// 机械臂模式：只把按键换成「意图」（速度 / 预设索引），
// 平方律、限位、积分全在 arm_pose 里，这里不认识任何速度常数。
// R3 单按没有分支：它只是 L3+R3 急停组合的另一半（Q-1 定案）。
void update_arm_mode(const Ps2Frame &frame) {
    // 预设 > 摇杆：命中预设的那一帧丢弃摇杆速度。arm_pose_request 内部也会清，
    // 这里显式清一次是为了让同一帧的两条入口语义一致（不回弹）。
    bool requested = false;
    for (unsigned i = 0; i < kPresetCount; ++i) {
        if (edge(frame, kPresetBinds[i].button)) {
            arm_pose_request(kPresetBinds[i].pose);
            requested = true;
        }
    }

    if (requested || !frame.analog) {
        // 绿灯数字模式没有模拟量：显式传 0，不依赖解码默认值 128。
        arm_pose_set_cart_velocity(0.0F, 0.0F);
    } else {
        const int forward = arm_axis_percent(frame.left_x, false);
        const int lift = arm_axis_percent(frame.left_y, true);
        float vx = 0.0F;
        float vy = 0.0F;
        arm_pose_stick_velocity(forward, lift, vx, vy);
        arm_pose_set_cart_velocity(vx, vy);
    }

    // 夹爪 servo[2]：L2 松 / R2 紧；腕 roll servo[3]：L1 逆 / R1 顺。
    // 方向约定以 arm_pose_v0.3 §7.4 为准：gripper 正 = 张开(松)、负 = 夹紧。
    // 轴号 2/3 与 arm_config 的通道角色一致；0/1 会被 arm_pose_joint_jog 拒绝，
    // 所以这里不需要再判一次模式 —— 笛卡尔与 jog 本来就允许同帧共存。
    int gripper = 0;
    if (pressed(frame, Ps2Button::L2)) {
        gripper += 100;
    }
    if (pressed(frame, Ps2Button::R2)) {
        gripper -= 100;
    }
    if (gripper != 0) {
        arm_pose_joint_jog(2U, gripper);
    }

    int wrist = 0;
    if (pressed(frame, Ps2Button::L1)) {
        wrist -= 100;
    }
    if (pressed(frame, Ps2Button::R1)) {
        wrist += 100;
    }
    if (wrist != 0) {
        arm_pose_joint_jog(3U, wrist);
    }
}

} // namespace

void mode_control_init(uint32_t now_ms) {
    g_mode = OperationMode::Drive;
    g_latched_stop = false;
    g_prev_buttons = 0;
    g_combo_arming = false;
    g_combo_fired = false;
    g_combo_since_ms = 0;
    arm_pose_init(now_ms);
}

ControlOutput mode_control_update(const Ps2Frame &frame, uint32_t now_ms) {
    ControlOutput output;

    // 1. 全局：急停组合 + 断线。
    update_estop_combo(frame, now_ms);
    const bool stopped = g_latched_stop || !frame.connected;

    // 2. 模式入口。两个键各自幂等；急停锁定时不响应，避免停机期间误切。
    //    切换瞬间清笛卡尔速度并同步边沿位图：底盘模式下按住不放的键
    //    不会在切进机械臂模式的第一帧被当成一次上升沿。
    const bool start_edge = edge(frame, Ps2Button::Start);
    const bool select_edge = edge(frame, Ps2Button::Select);
    if (!stopped && start_edge && g_mode != OperationMode::Drive) {
        g_mode = OperationMode::Drive;
        arm_pose_set_cart_velocity(0.0F, 0.0F); // 机械臂保持姿态，只是不再接受新速度
        g_prev_buttons = frame.buttons;
    } else if (!stopped && select_edge && g_mode != OperationMode::Arm) {
        g_mode = OperationMode::Arm;
        arm_pose_set_cart_velocity(0.0F, 0.0F);
        g_prev_buttons = frame.buttons;
    }

    // 3. 按模式分发。
    if (g_mode == OperationMode::Drive) {
        output.drive = stopped ? DriveCommand{} : drive_from_ps2(frame);
    } else {
        output.drive = DriveCommand{};
        if (!stopped) {
            update_arm_mode(frame);
        }
    }

    // 4. 机械臂输出：急停/断线时保持当前位置，避免突然甩动或掉落。
    output.arm = stopped ? arm_pose_hold(now_ms) : arm_pose_update(now_ms);
    output.arm_fb = arm_pose_feedback();

    output.mode = g_mode;
    output.stopped = stopped;
    g_prev_buttons = frame.buttons;
    return output;
}
