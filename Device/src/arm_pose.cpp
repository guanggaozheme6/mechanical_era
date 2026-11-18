/**
 ******************************************************************************
 * @file           : arm_pose.cpp
 * @brief          : 机械臂编排层实现：预设/笛卡尔入口 → 解算 → 关节层（Device 层）
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "arm_pose.hpp"
#include "arm_config.hpp"
#include "arm_joint.hpp"
#include "arm_kinematics.hpp"

#include <cmath>

/* Private macro -------------------------------------------------------------*/

/**
 * @brief 标定期编译开关
 * @details 定义 ARM_BRINGUP 后：上电直接把 kBringupPercent 四路值当目标，绕过 IK /
 *          工作区 / 角度限位；串口仍然打 S1..S4 与反馈，用它记录"顶到结构那一刻"
 *          的读数；不接任何手柄键 —— 只能改常量、重烧、上电，标完把宏关掉再烧一版。
 * @note 打开方式：CMakeLists.txt 的 target_compile_options 里加 -DARM_BRINGUP，
 *       或临时在 include 处 #define。正常固件绝不定义它。
 */
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

/* Private constants ---------------------------------------------------------*/

/**
 * @brief 上电时四路的目标行程（必须落在 kWorkspace 内）
 * @details B 方案下 percent=0 意味着大臂竖直 + 小臂半折(45°)，末端绕在肩上方，
 *          不在工作区里；上电点直接对齐 kPoses[Home]=(105.2, 38.3)。
 */
constexpr float kZeroPercent[kServoCount] = {-66.0F, -86.0F, 0.0F, 0.0F};

/**
 * @brief 标定期初值（行程百分比，-100..100）
 * @note 只有 ARM_BRINGUP 打开时才生效。改一个数 → 重烧 → 看串口：这是删掉关节 jog
 *       之后唯一的单路手动手段。
 */
constexpr int kBringupPercent[kServoCount] = {0, 0, 0, 0};

/**
 * @brief 标定版用的全行程限位
 * @details 必须放开到 ±100。用 kJointLimits 的话，"用限位去测限位"的循环还在，
 *          只不过程序从 solve_xy 换成了 bank_configure —— 量不出真实边界。
 * @warning 标定版没有任何软件限位，必须人工盯住，听到堵转立刻拔电。
 */
constexpr arm_joint::Limits kBringupLimits[kServoCount] = {
    {-100.0F, 100.0F}, {-100.0F, 100.0F}, {-100.0F, 100.0F}, {-100.0F, 100.0F}};

/* Private variables ---------------------------------------------------------*/

arm_joint::Bank g_bank;
float g_cart_vx = 0.0F; // 一次性，update 后清零（与 jog 同语义）
float g_cart_vy = 0.0F;
uint32_t g_last_ms = 0U;
ArmFeedback g_fb;
unsigned g_bad_pose_mask = 0U;

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 是否有限值
 * @param value 输入
 * @return true 表示既非 NaN 也非 Inf
 */
bool finite(float value) { return std::isfinite(value); }

/**
 * @brief 标量限幅
 * @param value 输入值
 * @param lower 下限
 * @param upper 上限
 * @return 限幅后的值
 */
float clampf(float value, float lower, float upper) {
    return value < lower ? lower : (value > upper ? upper : value);
}

/**
 * @brief 水平位下垂预补偿
 * @details 线性按"前伸距离"缩放，默认系数 0 = 完全关闭。
 * @param x_mm 前伸距离（mm）
 * @return 需要额外抬高的量（mm）
 */
float sag_compensation(float x_mm) { return kMotion.sag_mm_per_100mm * x_mm / 100.0F; }

/**
 * @brief 用当前关节值刷新反馈快照
 * @details FK 回算末端 + 两路连杆角；没有编码器时这是"命令值"，不是实测角。
 */
void refresh_feedback() {
    const float s1 = g_bank.channel[arm_config::kShoulderServo].current;
    const float s2 = g_bank.channel[arm_config::kElbowServo].current;
    arm_kinematics::solve_forward(kGeom, s1, s2, g_fb.x_mm, g_fb.y_mm);
    g_fb.shoulder_deg = arm_kinematics::percent_to_deg(kGeom.shoulder, s1);
    g_fb.elbow_deg = arm_kinematics::percent_to_deg(kGeom.elbow, s2);
    g_fb.bringup = kBringup;
    g_fb.bad_poses = g_bad_pose_mask;
}

/**
 * @brief 两级钳位 + IK + 写入关节层
 * @details 钳位顺序不能反：先矩形（穿肩轴 / 力矩 / 折叠死区），再圆环（够不着）。
 *          越界**不拒绝**：矩形直接夹到边，半径沿方位缩到"限位允许的可控边界"，
 *          然后把钳位结果照样下发 —— 摇杆推到头就是顶住边界，手感连续。
 *          只有 θ1 越出结构限位才真的不动作（那个方位本身够不到，硬掰会让臂甩到别处）。
 * @param x_mm 请求 x（mm）
 * @param y_mm 请求 y（mm）
 * @param applied_x 输出：实际用于解算的 x（钳位后、含下垂补偿）
 * @param applied_y 输出：实际用于解算的 y
 * @return true 表示精确命中；false 表示已钳到边界后下发，或完全够不到（原因在 g_fb 里）
 */
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
    g_fb.reachable = solution.reachable; // 没被径向钳位 ⇒ 精确命中
    g_fb.limited = solution.limited;     // θ1 越限 ⇒ 这个方位本身够不到
    g_fb.at_wall = wall || !solution.reachable;
    g_fb.target_x_mm = applied_x;
    g_fb.target_y_mm = applied_y;

    // 越界的点已经在 solve_xy 里沿方位钳到圆环边界（太远 → L1+L2，太近 → 限位允许的内半径），
    // 所以这里照发：摇杆推到头就是顶住边界，手感连续、不会突然拒绝。
    // 只有 θ1 越过结构限位才真的不动 —— 那种情况是"这个方位本身够不到"（比如正下方死角），
    // 硬掰只会让臂甩到别处去。
    if (solution.limited) {
        return false;
    }

    arm_joint::set_target(g_bank.channel[arm_config::kShoulderServo], solution.shoulder_percent);
    arm_joint::set_target(g_bank.channel[arm_config::kElbowServo], solution.elbow_percent);
    return !wall && solution.reachable;
}

/**
 * @brief 闭环于"当前实际末端"的积分
 * @details 每帧只走一个步长，不累积虚假距离：松手不追尾、到边界不蠕动、扳回立即恢复。
 * @param dt_s 本帧时长（秒）
 */
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

/**
 * @brief 打包本帧的四路指令
 * @return ArmCommand（四舍五入后的整数行程百分比）
 */
ArmCommand make_command() {
    ArmCommand command;
    arm_joint::bank_output(g_bank, command.servo);
    return command;
}

} // namespace

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化：记时基、四路归零、8 条预设各做一次 IK 自检
 * @details 标定版（ARM_BRINGUP）走另一条路：直接给 kBringupPercent，跳过全部钳位。
 * @param now_ms 当前毫秒时基
 */
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

/**
 * @brief 请求第 index 条预设
 * @details 进门先清笛卡尔速度；越界或该条自检没过则拒绝下发并置 at_wall。
 * @param index 预设索引 0..7
 */
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

/**
 * @brief 绝对坐标请求（动作序列 / 上位机）
 * @param x_mm 目标 x（mm）
 * @param y_mm 目标 y（mm）
 * @return true 已下发；false 被钳位或超限
 */
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

/**
 * @brief 每帧给一次末端速度（mm/s）
 * @note 一次性：update 后清零，松手必须传 0。
 * @param vx_mm_per_s 末端 x 速度
 * @param vy_mm_per_s 末端 y 速度
 */
void arm_pose_set_cart_velocity(float vx_mm_per_s, float vy_mm_per_s) {
    if (kBringup) {
        return; // 标定版不接受任何运动输入
    }
    g_cart_vx = finite(vx_mm_per_s) ? vx_mm_per_s : 0.0F;
    g_cart_vy = finite(vy_mm_per_s) ? vy_mm_per_s : 0.0F;
}

/**
 * @brief 摇杆百分比 → 末端速度（保号平方律 + 合速度归一化）
 * @param forward_percent 前伸方向摇杆百分比（-100..100）
 * @param lift_percent 抬升方向摇杆百分比（-100..100）
 * @param vx_mm_per_s 输出：末端 x 速度
 * @param vy_mm_per_s 输出：末端 y 速度
 */
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

/**
 * @brief 单关节速度（单位是摇杆百分比）
 * @details 白名单只放 2（夹爪）/ 3（腕 roll）；0/1 返回 false，保证"解算关节没有手动入口"。
 * @param axis 通道号
 * @param stick_percent 摇杆百分比（-100..100）
 * @return true 已接受；false 被拒绝（标定版 / 轴 0、1）
 */
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

/**
 * @brief 推进一帧：积分 + 关节插值 + 刷新反馈
 * @param now_ms 当前毫秒时基
 * @return 本帧四路行程百分比
 */
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

/**
 * @brief 冻结一帧：保持当前姿态
 * @note 仍然推进时基，否则解除急停的第一帧会因为累积时间而跳一大段。
 * @param now_ms 当前毫秒时基
 * @return 当前四路行程百分比
 */
ArmCommand arm_pose_hold(uint32_t now_ms) {
    g_last_ms = now_ms;
    g_cart_vx = 0.0F;
    g_cart_vy = 0.0F;
    arm_joint::bank_hold(g_bank);
    refresh_feedback();
    return make_command();
}

/**
 * @brief 取某一路当前的整数行程百分比
 * @param joint 关节序号 0..3
 * @return -100..100；越界返回 0
 */
int arm_pose_value(unsigned joint) {
    if (joint >= kServoCount) {
        return 0;
    }
    return arm_joint::output_percent(g_bank.channel[joint]);
}

/**
 * @brief 四路是否都已到达目标
 * @return true 表示四路都在 settle 容差内
 */
bool arm_pose_at_target() { return arm_joint::bank_at_target(g_bank, kMotion.settle_percent); }

/**
 * @brief 夹爪接在哪一路舵机上
 * @return 通道号（arm_config::kGripperServo）
 */
unsigned arm_pose_gripper_servo() { return arm_config::kGripperServo; }

/**
 * @brief 取最新反馈快照
 * @return ArmFeedback 副本
 */
ArmFeedback arm_pose_feedback() { return g_fb; }

/**
 * @brief 上电自检结果
 * @return bit i = 第 i 条预设不可用（0 = 八条全好）
 */
unsigned arm_pose_bad_pose_mask() { return g_bad_pose_mask; }
