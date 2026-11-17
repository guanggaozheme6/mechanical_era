#pragma once

// ============================================================================
//  arm_config —— 机械臂参数的唯一真值
// ----------------------------------------------------------------------------
//  这里的每个数字都是"标定后该改的地方"，算法文件里不再出现任何硬编码数值。
//
//  ⚠️ C++14（CMakeLists.txt:7 + CXX_EXTENSIONS OFF）：不能用 designated
//  initializers（`.field = x` 是 C++20）。只能按声明顺序位置初始化，
//  所以**字段顺序就是初始化顺序，以后只准在末尾追加**（D1）。
//
//  本文件只装数值 + 编译期自检，不放任何逻辑。
// ============================================================================

#include "arm_joint.hpp"
#include "arm_kinematics.hpp"

namespace arm_config {

// ---- 1. 几何模型（L1 / L2 / 每关节零点、比例、角度限位）--------------------
//  shoulder: 大臂，zero 0° = 水平；elbow: 小臂，zero 90° = 垂直于大臂（v0.1 §12.3）
constexpr arm_kinematics::Model kGeom = {
    110.0F, 95.0F,                          // upper_mm, fore_mm           TBD-1
    {0.0F, 0.9F, -90.0F, 90.0F},            // shoulder zero,deg/%,min,max TBD-2/3/4
    {90.0F, 0.9F, 0.0F, 180.0F},            // elbow    zero 必须 90°      TBD-2/3/4
    +1,                                     // elbow_sign = 小臂向上兜
};

static_assert(kGeom.upper_mm > 0.0F && kGeom.fore_mm > 0.0F, "连杆长度必须为正");
static_assert(kGeom.elbow_sign == 1 || kGeom.elbow_sign == -1, "elbow_sign 只能是 ±1");

// 内半径 |L1-L2| 太小会让折叠区数值病态（r→0 时方向不可分辨）。10 mm 是下限：
// 本车 15 mm（θ2=180° 的折叠点）刚好通过，再小就要求连杆长度明显不对了。
constexpr bool inner_radius_ok() {
    const float diff = kGeom.upper_mm > kGeom.fore_mm ? kGeom.upper_mm - kGeom.fore_mm
                                                      : kGeom.fore_mm - kGeom.upper_mm;
    return diff > 10.0F;
}
static_assert(inner_radius_ok(), "|L1-L2| 太小，折叠区数值病态，检查连杆长度");

// ---- 2. 通道角色（D5：全工程唯一一处，删掉 arm_pose.cpp:40 的 kGripperServo）----
constexpr unsigned kServoCount = arm_joint::kChannelCount;
constexpr unsigned kShoulderServo = 0U;  // 大臂，参与解算
constexpr unsigned kElbowServo = 1U;     // 小臂，参与解算
constexpr unsigned kGripperServo = 2U;   // 夹爪松紧，不参与解算
constexpr unsigned kWristServo = 3U;     // 腕 roll，不参与解算
static_assert(arm_joint::kChannelCount == 4U, "关节层通道数与四路舵机不一致");
static_assert(kShoulderServo < kServoCount && kElbowServo < kServoCount &&
                  kGripperServo < kServoCount && kWristServo < kServoCount,
              "通道编号越界");

// ---- 3. 行程限位：只存角度，百分比全部派生（P3，一套真值）-------------------
constexpr float percent_of(const arm_kinematics::JointCalib &calib, float deg) {
    // C++14 constexpr 里只能用四则运算（不能调 std::），所以这里自己防零除。
    return calib.deg_per_percent == 0.0F ? 0.0F : (deg - calib.zero_deg) / calib.deg_per_percent;
}

constexpr arm_joint::Limits kJointLimits[kServoCount] = {
    // 1% ≈ 0.9°：角度限位 → 行程百分比，改角度就不用再改百分比
    {percent_of(kGeom.shoulder, kGeom.shoulder.range_min_deg),
     percent_of(kGeom.shoulder, kGeom.shoulder.range_max_deg)},
    {percent_of(kGeom.elbow, kGeom.elbow.range_min_deg),
     percent_of(kGeom.elbow, kGeom.elbow.range_max_deg)},
    {-100.0F, 100.0F}, // 夹爪：无几何约束，全行程
    {-100.0F, 100.0F}, // 腕 roll：全行程
};

static_assert(kJointLimits[kShoulderServo].min_percent >= -100.0F &&
                  kJointLimits[kShoulderServo].max_percent <= 100.0F,
              "大臂角度限位超出舵机行程，检查 deg_per_percent");
static_assert(kJointLimits[kElbowServo].min_percent >= -100.0F &&
                  kJointLimits[kElbowServo].max_percent <= 100.0F,
              "小臂角度限位超出舵机行程，zero_deg 是否取了 90°？");
static_assert(kJointLimits[kShoulderServo].min_percent < kJointLimits[kShoulderServo].max_percent &&
                  kJointLimits[kElbowServo].min_percent < kJointLimits[kElbowServo].max_percent,
              "角度限位上下界颠倒");

// ---- 4. 笛卡尔工作区（矩形，第一级钳位）------------------------------------
struct Workspace {
    float x_min_mm;
    float x_max_mm;
    float y_min_mm;
    float y_max_mm;
};

// x_min 由小臂最大折叠角决定（v0.1 §12.2）：θ2max=143° → 70 mm；y_max 避开高力矩区。
constexpr Workspace kWorkspace = {70.0F, 200.0F, -20.0F, 120.0F};

static_assert(kWorkspace.x_min_mm > 0.0F, "工作区 x_min 必须为正，否则末端会穿过肩轴");
static_assert(kWorkspace.x_min_mm < kWorkspace.x_max_mm, "工作区 x 上下界颠倒");
static_assert(kWorkspace.y_min_mm < kWorkspace.y_max_mm, "工作区 y 上下界颠倒");
static_assert(kWorkspace.x_max_mm <= kGeom.upper_mm + kGeom.fore_mm,
              "工作区 x_max 超出 L1+L2，永远走不到");

// ---- 5. 速度与手感 ---------------------------------------------------------
struct Motion {
    float cart_speed_mm_per_second;  // 左摇杆满偏时的末端速度
    float slew_percent_per_second;   // 关节插值限速（姿态切换 / 追目标）
    float settle_percent;            // at_target 容差
    float sag_mm_per_100mm;          // 水平位下垂预补偿，0 = 关闭
    unsigned max_elapsed_ms;         // 单帧最大积分步长（主循环卡顿保护）
    int jog_percent_per_second[kServoCount]; // 夹爪/腕 roll 的满杆 jog 速度（轴 0/1 白名单禁止，值为占位）
};

constexpr Motion kMotion = {
    50.0F,                              // cart_speed_mm_per_second
    300.0F,                             // slew_percent_per_second（car_config 原值）
    1.0F,                               // settle_percent
    0.0F,                               // sag：默认关闭，见 v0.3 风险表
    100U,                               // max_elapsed_ms（arm_pose.cpp 原 kMaxElapsedMs）
    {150, 150, 200, 150},               // jog: 肩/肘/夹爪/腕
};

static_assert(kMotion.cart_speed_mm_per_second > 0.0F, "满杆末端速度必须为正");
static_assert(kMotion.slew_percent_per_second > 0.0F, "slew 必须为正");
static_assert(kMotion.sag_mm_per_100mm >= 0.0F, "下垂补偿不允许为负");
static_assert(kMotion.jog_percent_per_second[kGripperServo] > 0, "夹爪 jog 速度必须为正");

// 最坏情况 slew 余量自检（v0.2 §2.3 实算）：r=60mm、末端 50mm/s 需要 ≈53 %/s。
constexpr bool slew_ok() {
    const float needed = kMotion.cart_speed_mm_per_second / 60.0F * arm_kinematics::kRadToDeg /
                         kGeom.shoulder.deg_per_percent;
    return kMotion.slew_percent_per_second > needed * 4.0F;
}
static_assert(slew_ok(), "slew 余量不足 4 倍，末端精调会跟不上摇杆");

// ---- 6. 八条预设姿态（表驱动，索引 = 按键绑定表的 pose 字段）-----------------
//  语义：x/y 是腕点绝对坐标（mm，原点在肩轴）；gripper/wrist 是行程百分比。
//  gripper：正 = 张开，负 = 夹紧（机械方向未标定时以此约定为准）。
struct Preset {
    float x_mm;
    float y_mm;
    float gripper_percent;
    float wrist_percent;
};

enum class Pose : unsigned {
    Home = 0,   // ↑ 收纳 / 上电初始位
    Ready,      // ↓ 准备抓取
    Reach,      // ← 前伸
    Retract,    // → 回收
    Lift,       // △ 抬起
    Carry,      // ○ 搬运
    Place,      // ╳ 放置
    Drop,       // □ 释放
    Count
};

constexpr Preset kPoses[8] = {
    //  x_mm   y_mm  grip  wrist  说明
    {110.0F, 95.0F, 0.0F, 0.0F},   // Home   = FK(0,0,0,0)，上电中位就是这个点，切换不跳
    {150.0F, 60.0F, -20.0F, 0.0F}, // Ready  准备抓取
    {190.0F, 10.0F, -40.0F, 0.0F}, // Reach  前伸到边界附近
    {75.0F, 15.0F, 0.0F, 0.0F},    // Retract 收到最近
    {120.0F, 110.0F, -40.0F, 0.0F},// Lift   抬到高位（y_max 内）
    {110.0F, 90.0F, -40.0F, 0.0F}, // Carry  搬运中间态
    {160.0F, 40.0F, -30.0F, 0.0F}, // Place  放置
    {170.0F, 70.0F, 20.0F, 0.0F},  // Drop   松开
};

constexpr unsigned kPoseCount = static_cast<unsigned>(Pose::Count);
static_assert(kPoseCount == 8U, "预设姿态数量与 8 个按键不一致");
static_assert(sizeof(kPoses) / sizeof(kPoses[0]) == kPoseCount, "kPoses 行数与 Pose::Count 不一致");

// 上电自检：每条预设必须在工作区矩形内（IK 另有运行期校验）
constexpr bool poses_in_area() {
    for (unsigned i = 0; i < kPoseCount; ++i) {
        if (kPoses[i].x_mm < kWorkspace.x_min_mm || kPoses[i].x_mm > kWorkspace.x_max_mm ||
            kPoses[i].y_mm < kWorkspace.y_min_mm || kPoses[i].y_mm > kWorkspace.y_max_mm) {
            return false;
        }
    }
    return true;
}
static_assert(poses_in_area(), "有预设姿态落在工作区矩形之外");

constexpr bool poses_in_travel() {
    for (unsigned i = 0; i < kPoseCount; ++i) {
        if (kPoses[i].gripper_percent < -100.0F || kPoses[i].gripper_percent > 100.0F ||
            kPoses[i].wrist_percent < -100.0F || kPoses[i].wrist_percent > 100.0F) {
            return false;
        }
    }
    return true;
}
static_assert(poses_in_travel(), "预设姿态的夹爪/腕百分比越界");

} // namespace arm_config
