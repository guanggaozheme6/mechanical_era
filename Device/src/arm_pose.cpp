/**
 ******************************************************************************
 * @file           : arm_pose.cpp
 * @brief          : 机械臂编排层实现：中立位/笛卡尔入口 → 解算 → 关节层（Device 层）
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "arm_pose.hpp"
#include "arm_config.hpp"
#include "arm_joint.hpp"
#include "arm_kinematics.hpp"
#include "arm_limit.hpp"

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
using arm_config::kNeutralPose;
using arm_config::kServoCount;
using arm_config::kWorkspace;

/* Private constants ---------------------------------------------------------*/

/**
 * @brief 上电时四路的目标行程（必须落在 kWorkspace 内）
 * @details percent=0 意味着大臂竖直 + 小臂半折(45°)，末端绕在肩上方，
 *          不在工作区里；上电点直接对齐 kNeutralPose=(105.2, 38.3)。
 */
constexpr float kZeroPercent[kServoCount] = {-66.0F, -86.0F, 0.0F, 0.0F};

/**
 * @brief 标定期初值（行程百分比，-100..100）
 * @note 只有 ARM_BRINGUP 打开时才生效。改一个数 → 重烧 → 看串口。
 */
constexpr int kBringupPercent[kServoCount] = {0, 0, 0, 0};

/**
 * @brief 标定版用的全行程限位
 * @details 必须放开到 ±100：用 kJointLimits 就量不出真实边界。
 * @warning 标定版没有任何软件限位，必须人工盯住，听到堵转立刻拔电。
 */
constexpr arm_joint::Limits kBringupLimits[kServoCount] = {
    {-100.0F, 100.0F}, {-100.0F, 100.0F}, {-100.0F, 100.0F}, {-100.0F, 100.0F}};

/**
 * @brief 摇杆限位（arm_limit）要用的三张表
 * @details 算法层不认识 arm_config，所以限位/矩形/调参由这一层从配置派生后传进去，
 *          形态是 arm_limit 自己的 POD。改 kWorkspace / kJointLimits / kMotion 不用动算法。
 */
constexpr arm_limit::Span kLimitSpans[2] = {
    {arm_config::kJointLimits[arm_config::kShoulderServo].min_percent,
     arm_config::kJointLimits[arm_config::kShoulderServo].max_percent},
    {arm_config::kJointLimits[arm_config::kElbowServo].min_percent,
     arm_config::kJointLimits[arm_config::kElbowServo].max_percent}};

/** @brief 工作区矩形（摇杆命令的允许范围，不是可达性证明） */
constexpr arm_limit::Box kStepBox = {kWorkspace.x_min_mm, kWorkspace.x_max_mm,
                                     kWorkspace.y_min_mm, kWorkspace.y_max_mm};

/** @brief 限位策略调参：阻尼 λ / 下垂补偿 / 矩形残差容差 */
constexpr arm_limit::Params kStepParams = {kMotion.stick_damping_mm_per_percent,
                                           kMotion.sag_mm_per_100mm,
                                           kMotion.stick_box_tol_mm};

/* Private variables ---------------------------------------------------------*/

arm_joint::Bank g_bank;
float g_cart_vx = 0.0F; // 一次性，update 后清零（与 jog 同语义）
float g_cart_vy = 0.0F;
uint32_t g_last_ms = 0U;
ArmFeedback g_fb;
bool g_neutral_ok = true; // 上电自检：中立位可达且不越限

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
    g_fb.neutral_ok = g_neutral_ok;
}

/**
 * @brief 绝对值入口：两级钳位 + IK + 写入关节层（中立位 / 上位机用的那条路）
 * @details 钳位顺序不能反：先矩形（穿肩轴 / 力矩 / 折叠死区），再圆环（够不着）。
 *          越界**不拒绝**：矩形直接夹到边，半径沿方位缩到"限位允许的可控边界"，
 *          然后把钳位结果照样下发 —— 回中立位 / 给坐标是「去一个点」，不是「沿某个方向走」，
 *          所以这里保留钳位照发；摇杆那条路才是贴边滑（见 cartesian_step + arm_limit）。
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
 * @brief 摇杆一步：位置法优先，关节域兜底（贴边滑）
 * @details 「走不通」不解释成「沿半径缩到弧带上照发」，而是「位置法沿摇杆方向缩步，
 *          缩到底就交给关节域沿边界切向滑」。
 *          闭环于「当前实际末端」（current，不是 target）：每帧只走一个步长，松手不追尾、
 *          到边界不蠕动、扳回立即恢复。
 * @param dt_s 本帧时长（秒）
 */
void cartesian_step(float dt_s) {
    const float s1 = g_bank.channel[arm_config::kShoulderServo].current;
    const float s2 = g_bank.channel[arm_config::kElbowServo].current;
    const arm_limit::Step step = arm_limit::tangent_step(kGeom, kLimitSpans, kStepBox, kStepParams,
                                                        s1, s2, g_cart_vx, g_cart_vy, dt_s);

    if (!step.idle && !step.stuck) {
        // 关节层里还有一道限位；percent 全程 float，取整只发生在 ArmCommand 收口处
        // （贴边滑每帧可能只增 0.004%，中间取整会把增量抹成 0，现象是「推不动」）。
        arm_joint::set_target(g_bank.channel[arm_config::kShoulderServo], step.p1_percent);
        arm_joint::set_target(g_bank.channel[arm_config::kElbowServo], step.p2_percent);
    }

    // 反馈口径：三个独立事实，不许合成一个 ok。
    //   ok=1 精确命中（没缩步、没裁角）；wall=1 顶住边界但还在走（缩步 / 切向滑，已下发）；
    //   lim=1 一步都走不动（正对墙 / 墙角），整帧不下发。
    g_fb.reachable = !step.slide && !step.stuck;
    g_fb.at_wall = step.slide || step.stuck;
    g_fb.limited = step.stuck;

    float target_x = 0.0F;
    float target_y = 0.0F;
    arm_kinematics::solve_forward(kGeom, step.p1_percent, step.p2_percent, target_x, target_y);
    g_fb.target_x_mm = target_x;
    g_fb.target_y_mm = target_y;
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
 * @brief 初始化：记时基、四路归零、中立位做一次 IK 自检
 * @details 标定版（ARM_BRINGUP）走另一条路：直接给 kBringupPercent，跳过全部钳位。
 * @param now_ms 当前毫秒时基
 */
void arm_pose_init(uint32_t now_ms) {
    g_cart_vx = 0.0F;
    g_cart_vy = 0.0F;
    g_last_ms = now_ms;
    g_fb = ArmFeedback{};
    g_neutral_ok = true;

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

    // 中立位自检：坐标填错时立刻可见（串口 neutral_ok=0），而不是等按 Start 才发现不动。
    const arm_kinematics::Solution solution = arm_kinematics::solve_xy(
        kGeom, kNeutralPose.x_mm, kNeutralPose.y_mm + sag_compensation(kNeutralPose.x_mm));
    g_neutral_ok = solution.reachable && !solution.limited;
    refresh_feedback();
}

/**
 * @brief 回到中立位（Start 键）
 * @details 先清笛卡尔速度（绝对目标是「去一个点」，不能先按摇杆方向挪一格再被拉回），
 *          再走和绝对坐标入口同一条钳位链路。**只写 S1/S2**：夹爪与腕保持用户当前值，
 *          否则夹着东西按 Start 会松手或甩腕。
 */
void arm_pose_request_neutral() {
    if (kBringup) {
        return;
    }
    g_cart_vx = 0.0F;
    g_cart_vy = 0.0F;

    if (!g_neutral_ok) {
        g_fb.at_wall = true; // 上电自检就没过（中立位够不到），拒绝下发
        return;
    }

    float applied_x = 0.0F;
    float applied_y = 0.0F;
    if (!apply_xy(kNeutralPose.x_mm, kNeutralPose.y_mm, applied_x, applied_y)) {
        return;
    }
    g_fb.target_x_mm = applied_x;
    g_fb.target_y_mm = applied_y;
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
 * @brief 四路是否都已到达目标
 * @return true 表示四路都在 settle 容差内
 */
bool arm_pose_at_target() { return arm_joint::bank_at_target(g_bank, kMotion.settle_percent); }

/**
 * @brief 取最新反馈快照
 * @return ArmFeedback 副本
 */
ArmFeedback arm_pose_feedback() { return g_fb; }
