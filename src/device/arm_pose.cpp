#include "device/arm_pose.hpp"
#include "config/arm_config.hpp"
#include "device/arm_joint.hpp"
#include "algorithm/arm_kinematics.hpp"

#include <cmath>

// ============================================================================
//  标定期编译开关（甲方要求：删掉所有单关节手动入口之后，结构限位只能这样量）
// ----------------------------------------------------------------------------
//  定义 ARM_BRINGUP 后：
//    * 上电直接把 kBringupPercent 四路值当目标，绕过 IK / 工作区 / 角度限位；
//    * 串口仍然打 S1..S4 与反馈，用它记录"顶到结构那一刻"的读数；
//    * 不接任何手柄键 —— 只能改常量、重烧、上电，标完把宏关掉再烧一版。
//  打开方式：CMakeLists.txt 的 target_compile_options 里加 -DARM_BRINGUP，
//  或临时在 arm_config.hpp 顶部 #define。正常固件绝不定义它。
// ============================================================================
#if defined(ARM_BRINGUP)
constexpr bool kBringup = true;
#else
constexpr bool kBringup = false;
#endif

namespace {

using arm_config::kGeom;
using arm_config::kMotion;
using arm_config::kPoseCount;
using arm_config::kPoses;
using arm_config::kServoCount;
using arm_config::kWorkspace;

constexpr float kZeroPercent[kServoCount] = {0.0F, 0.0F, 0.0F, 0.0F};

// 标定期初值（行程百分比，-100..100）。只有 ARM_BRINGUP 打开时才生效。
// 改一个数 → 重烧 → 看串口：这是删掉关节 jog 之后唯一的单路手动手段。
constexpr int kBringupPercent[kServoCount] = {0, 0, 0, 0};

// 标定版必须放开到 ±100 全行程。用 kJointLimits 的话，"用限位去测限位"的循环
// 还在，只不过程序从 solve_xy 换成了 bank_configure —— 量不出真实边界。
constexpr arm_joint::Limits kBringupLimits[kServoCount] = {
    {-100.0F, 100.0F}, {-100.0F, 100.0F}, {-100.0F, 100.0F}, {-100.0F, 100.0F}};

arm_joint::Bank g_bank;
float g_cart_vx = 0.0F; // 一次性，update 后清零（与 jog 同语义）
float g_cart_vy = 0.0F;
uint32_t g_last_ms = 0U;
ArmFeedback g_fb;
unsigned g_bad_pose_mask = 0U;

bool finite(float value) { return std::isfinite(value); }

float clampf(float value, float lower, float upper) {
    return value < lower ? lower : (value > upper ? upper : value);
}

// 水平位下垂预补偿：线性按"前伸距离"缩放。默认系数 0 = 完全关闭。
float sag_compensation(float x_mm) { return kMotion.sag_mm_per_100mm * x_mm / 100.0F; }

void refresh_feedback() {
    const float s1 = g_bank.channel[arm_config::kShoulderServo].current;
    const float s2 = g_bank.channel[arm_config::kElbowServo].current;
    arm_kinematics::solve_forward(kGeom, s1, s2, g_fb.x_mm, g_fb.y_mm);
    g_fb.shoulder_deg = arm_kinematics::percent_to_deg(kGeom.shoulder, s1);
    g_fb.elbow_deg = arm_kinematics::percent_to_deg(kGeom.elbow, s2);
    g_fb.bringup = kBringup;
    g_fb.bad_poses = g_bad_pose_mask;
}

// 两级钳位 + IK + 写入关节层。返回 false 表示没下发（原因已在 g_fb 里）。
// 钳位顺序不能反：先矩形（穿肩轴 / 力矩 / 折叠死区），再圆环（够不着）。
bool apply_xy(float x_mm, float y_mm, float &applied_x, float &applied_y) {
    if (!finite(x_mm) || !finite(y_mm)) {
        g_fb.at_wall = true;
        g_fb.reachable = false;
        return false;
    }

    float tx = x_mm;
    float ty = y_mm + sag_compensation(x_mm);
    bool wall = false;
    if (tx < kWorkspace.x_min_mm) {
        tx = kWorkspace.x_min_mm;
        wall = true;
    } else if (tx > kWorkspace.x_max_mm) {
        tx = kWorkspace.x_max_mm;
        wall = true;
    }
    if (ty < kWorkspace.y_min_mm) {
        ty = kWorkspace.y_min_mm;
        wall = true;
    } else if (ty > kWorkspace.y_max_mm) {
        ty = kWorkspace.y_max_mm;
        wall = true;
    }

    const arm_kinematics::Solution solution = arm_kinematics::solve_xy(kGeom, tx, ty);
    applied_x = tx;
    applied_y = ty;
    g_fb.reachable = solution.reachable;
    g_fb.limited = solution.limited;
    g_fb.at_wall = wall;

    // 命中任一级钳位就不改目标：边界点往往贴着奇异区，照着走会抖。
    if (wall || !solution.reachable || solution.limited) {
        return false;
    }

    arm_joint::set_target(g_bank.channel[arm_config::kShoulderServo], solution.shoulder_percent);
    arm_joint::set_target(g_bank.channel[arm_config::kElbowServo], solution.elbow_percent);
    return true;
}

// 闭环于"当前实际末端"的积分：每帧只走一个步长，不累积虚假距离。
void cartesian_step(float dt_s) {
    const float s1 = g_bank.channel[arm_config::kShoulderServo].current;
    const float s2 = g_bank.channel[arm_config::kElbowServo].current;
    float fx = 0.0F;
    float fy = 0.0F;
    arm_kinematics::solve_forward(kGeom, s1, s2, fx, fy);

    float applied_x = 0.0F;
    float applied_y = 0.0F;
    apply_xy(fx + g_cart_vx * dt_s, fy + g_cart_vy * dt_s, applied_x, applied_y);
    g_fb.target_x_mm = applied_x;
    g_fb.target_y_mm = applied_y;
}

ArmCommand make_command() {
    ArmCommand command;
    arm_joint::bank_output(g_bank, command.servo);
    return command;
}

} // namespace

void arm_pose_init(uint32_t now_ms) {
    g_cart_vx = 0.0F;
    g_cart_vy = 0.0F;
    g_last_ms = now_ms;
    g_fb = ArmFeedback{};
    g_bad_pose_mask = 0U;

    if (kBringup) {
        // 标定版：直接给固定百分比，跳过解算与全部钳位（限位本身就是要量出来的）。
        float start[kServoCount] = {};
        for (unsigned i = 0; i < kServoCount; ++i) {
            start[i] = static_cast<float>(kBringupPercent[i]);
        }
        arm_joint::bank_configure(g_bank, kBringupLimits, start);
        refresh_feedback();
        return;
    }

    arm_joint::bank_configure(g_bank, arm_config::kJointLimits, kZeroPercent);

    // 上电全表校验：预设坐标填错时立刻可见，而不是等按到那一条才发现不动。
    for (unsigned i = 0; i < kPoseCount; ++i) {
        const arm_kinematics::Solution solution = arm_kinematics::solve_xy(
            kGeom, kPoses[i].x_mm, kPoses[i].y_mm + sag_compensation(kPoses[i].x_mm));
        if (!solution.reachable || solution.limited) {
            g_bad_pose_mask |= 1U << i;
        }
    }
    refresh_feedback();
}

void arm_pose_request(unsigned index) {
    if (kBringup) {
        return;
    }
    // 先清掉本帧的笛卡尔速度：预设是绝对目标，不能先按摇杆方向挪一格再被拉回。
    g_cart_vx = 0.0F;
    g_cart_vy = 0.0F;

    if (index >= kPoseCount) {
        g_fb.at_wall = true;
        return;
    }
    if ((g_bad_pose_mask & (1U << index)) != 0U) {
        g_fb.at_wall = true; // 该条上电自检就没过，拒绝下发
        return;
    }

    const arm_config::Preset &pose = kPoses[index];
    float applied_x = 0.0F;
    float applied_y = 0.0F;
    if (!apply_xy(pose.x_mm, pose.y_mm, applied_x, applied_y)) {
        return;
    }
    g_fb.target_x_mm = applied_x;
    g_fb.target_y_mm = applied_y;
    arm_joint::set_target(g_bank.channel[arm_config::kGripperServo], pose.gripper_percent);
    arm_joint::set_target(g_bank.channel[arm_config::kWristServo], pose.wrist_percent);
}

bool arm_pose_request_xy(float x_mm, float y_mm) {
    if (kBringup) {
        return false;
    }
    g_cart_vx = 0.0F;
    g_cart_vy = 0.0F;
    float applied_x = 0.0F;
    float applied_y = 0.0F;
    const bool ok = apply_xy(x_mm, y_mm, applied_x, applied_y);
    g_fb.target_x_mm = applied_x;
    g_fb.target_y_mm = applied_y;
    return ok;
}

void arm_pose_set_cart_velocity(float vx_mm_per_s, float vy_mm_per_s) {
    if (kBringup) {
        return; // 标定版不接受任何运动输入
    }
    g_cart_vx = finite(vx_mm_per_s) ? vx_mm_per_s : 0.0F;
    g_cart_vy = finite(vy_mm_per_s) ? vy_mm_per_s : 0.0F;
}

void arm_pose_stick_velocity(int forward_percent, int lift_percent, float &vx_mm_per_s,
                             float &vy_mm_per_s) {
    const float fx = clampf(static_cast<float>(forward_percent), -100.0F, 100.0F) / 100.0F;
    const float fy = clampf(static_cast<float>(lift_percent), -100.0F, 100.0F) / 100.0F;
    const float speed = kMotion.cart_speed_mm_per_second;

    // 保号平方律：轻推落在 0.5~2 mm/s（精调），推到底才是 50 mm/s。
    vx_mm_per_s = speed * fx * std::fabs(fx);
    vy_mm_per_s = speed * fy * std::fabs(fy);

    // 双轴满偏若不归一化会得到 √2×speed（70 mm/s），等比缩到 speed。
    const float magnitude = std::sqrt(vx_mm_per_s * vx_mm_per_s + vy_mm_per_s * vy_mm_per_s);
    if (magnitude > speed) {
        const float scale = speed / magnitude;
        vx_mm_per_s *= scale;
        vy_mm_per_s *= scale;
    }
}

bool arm_pose_joint_jog(unsigned axis, int stick_percent) {
    if (kBringup) {
        return false; // 标定版只认 kBringupPercent，不吃手柄
    }
    if (axis != arm_config::kGripperServo && axis != arm_config::kWristServo) {
        return false; // 0/1 只能由笛卡尔入口驱动，没有手动 jog
    }
    const float rate = static_cast<float>(stick_percent) *
                       static_cast<float>(kMotion.jog_percent_per_second[axis]) / 100.0F;
    arm_joint::set_jog(g_bank.channel[axis], rate);
    return true;
}

ArmCommand arm_pose_update(uint32_t now_ms) {
    uint32_t elapsed = now_ms - g_last_ms; // 无符号减法，时间回绕也正确
    if (elapsed > kMotion.max_elapsed_ms) {
        elapsed = kMotion.max_elapsed_ms;
    }
    g_last_ms = now_ms;
    const float dt = static_cast<float>(elapsed) / 1000.0F;

    // 标定版跳过解算，只按 slew 走向 kBringupPercent。
    if (!kBringup && (g_cart_vx != 0.0F || g_cart_vy != 0.0F)) {
        cartesian_step(dt);
    }
    g_cart_vx = 0.0F;
    g_cart_vy = 0.0F;

    arm_joint::bank_update(g_bank, dt, kMotion.slew_percent_per_second);
    refresh_feedback();
    return make_command();
}

ArmCommand arm_pose_hold(uint32_t now_ms) {
    g_last_ms = now_ms;
    g_cart_vx = 0.0F;
    g_cart_vy = 0.0F;
    arm_joint::bank_hold(g_bank);
    refresh_feedback();
    return make_command();
}

int arm_pose_value(unsigned joint) {
    if (joint >= kServoCount) {
        return 0;
    }
    return arm_joint::output_percent(g_bank.channel[joint]);
}

bool arm_pose_at_target() { return arm_joint::bank_at_target(g_bank, kMotion.settle_percent); }

unsigned arm_pose_gripper_servo() { return arm_config::kGripperServo; }

ArmFeedback arm_pose_feedback() { return g_fb; }

unsigned arm_pose_bad_pose_mask() { return g_bad_pose_mask; }
