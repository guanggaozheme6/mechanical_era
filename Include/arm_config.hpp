/**
 ******************************************************************************
 * @file           : arm_config.hpp
 * @brief          : 机械臂参数的唯一真值（几何、限位、工作区、速度、中立位）
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

// 这里的每个数字都是"标定后该改的地方"，算法文件里不再出现任何硬编码数值。
//
// 注意：C++14（CMakeLists.txt 的 CXX_STANDARD 14 + CXX_EXTENSIONS OFF）不能用
// designated initializers（.field = x 是 C++20）。只能按声明顺序位置初始化，
// 所以字段顺序就是初始化顺序，以后只准在末尾追加。
//
// 本文件只装数值 + 编译期自检，不放任何逻辑。
//
// v0.4 起固件里**没有预设姿态表**：只剩一个中立位（kNeutralPose），按键不再绑坐标。
// 那 8 条规划坐标留在 sim/index.html 里当挑点工具（见 sim/README §9），不进固件。

/* Includes ------------------------------------------------------------------*/
#include "arm_joint.hpp"
#include "arm_kinematics.hpp"

namespace arm_config {

/* Exported constants --------------------------------------------------------*/

/**
 * @brief 大臂贴底盘的结构接触角（度）
 * @details 臂按线段、底盘按实体建模（判据本体在 `sim/collide.js`，页面与探针共用）：底盘上表面留
 *          1 mm 余隙，肩轴 14 mm 半径内是底座、不算干涉。大臂上离轴心 r 的点高度是
 *          r·sinθ1，r 一过 14 mm 就必须高于 1 mm ⇒ θ1 ≥ asin(1/14) = 4.096°。
 *          0° 与 180° 都是「大臂躺在底盘上」，两端对称，所以 θ1 的可用段是 [4.10°, 175.90°]。
 * @note 这就是 D-2「结构限位」的取值依据：θ1 的 0°/180° 根本到不了，挡住它的是底盘不是舵机，
 *       所以「两端零余量」这条担心只对 θ2 成立。线模型没有臂厚，实物只会比 4.10° 更靠内。
 * @note 换底座高度 / 臂厚之后只需改这一个数，再跑 node sim/verify/struct_limit.js 重算派生量。
 */
constexpr float kShoulderContactDeg = 4.10F;

/**
 * @brief 两端各退的安全余量（度）
 * @details 「顶住结构后回退 3°」是 v0.2 起定下的标定约定；这里把它提前写进配置，
 *          重烧固件、换舵机都不会再退回到「一格余量都没有」的老值。
 * @note 改它之前跑 node sim/verify/struct_limit.js：会重算内半径、弧带厚度与中立位余量。
 */
constexpr float kStopBackoffDeg = 3.0F;

/**
 * @brief 几何模型
 * @details 大臂 θ1 ∈ [7.10°, 172.90°]，zero 90° = 竖直；小臂 θ2 ∈ [−42°, +132°]，zero 45° = 半折。
 *          余量 3° 的来源：θ1 是底盘接触角（kShoulderContactDeg），θ2 是舵机自身 180° 行程（D-1）。
 *          小臂行程不对称：以伸直为基准，前（超伸侧）只有 42°，后（折叠侧）132°。
 *          两路跨度 165.8° / 174°（= 184.2% / 193.3% 行程），一格行程 0.9°。
 * @note zero_deg 必须等于 (min+max)/2。余量是对称退的，所以 percent=0 的物理位置没变；
 *       文件末尾的 static_assert 会替你把这条卡住。
 */
constexpr arm_kinematics::Model kGeom = {
    76.0F, 40.08F, // L1 大臂 / L2 小臂（轴心距，实测）
    {90.0F, 0.9F, kShoulderContactDeg + kStopBackoffDeg, // shoulder: zero / deg% / min = 7.10
     180.0F - kShoulderContactDeg - kStopBackoffDeg},    //           max = 172.90
    {45.0F, 0.9F, -45.0F + kStopBackoffDeg, // elbow: zero / deg% / min = −42.00
     135.0F - kStopBackoffDeg},             //        max = +132.00
    +1,                                     // elbow_sign = +1 高肘（上吊）构型
};

/**
 * @brief zero_deg 是否正好落在行程中点
 * @details 行程百分比是对称的（±100），所以只有 zero 在中点时「退余量」才不改变 percent=0
 *          的物理位置；重心不在这条上的话，上电中位、中立位百分比、摇杆零速全都要重算。
 * @param calib 该关节的标定
 * @return true 表示中点与 zero 的差小于 1e-3°
 */
constexpr bool zero_at_mid(const arm_kinematics::JointCalib &calib) {
    const float mid = 0.5F * (calib.range_min_deg + calib.range_max_deg);
    const float diff = mid - calib.zero_deg;
    return diff < 1.0e-3F && diff > -1.0e-3F;
}

/**
 * @brief 角度跨度是否还在舵机行程以内
 * @param calib 该关节的标定
 * @return true 表示 (max−min)/deg_per_percent ≤ 200%
 */
constexpr bool span_within_travel(const arm_kinematics::JointCalib &calib) {
    return calib.deg_per_percent == 0.0F
               ? false
               : (calib.range_max_deg - calib.range_min_deg) / calib.deg_per_percent <= 200.0F;
}

/**
 * @brief 内半径是否合理
 * @details 内半径 |L1-L2| 太小会让折叠区数值病态（r→0 时方向不可分辨）。
 * @note 下限 10 mm：|L1−L2| 小于此值时折叠区的方向不可分辨。
 */
constexpr bool inner_radius_ok() {
    const float diff = kGeom.upper_mm > kGeom.fore_mm ? kGeom.upper_mm - kGeom.fore_mm
                                                      : kGeom.fore_mm - kGeom.upper_mm;
    return diff > 10.0F;
}

/**
 * @brief 四路舵机的通道角色
 * @note 全工程唯一一处定义；0/1 参与解算，2/3 不参与。
 */
constexpr unsigned kServoCount = arm_joint::kChannelCount;
constexpr unsigned kShoulderServo = 0U; // 大臂，参与解算
constexpr unsigned kElbowServo = 1U;    // 小臂，参与解算
constexpr unsigned kGripperServo = 2U;  // 夹爪松紧，不参与解算
constexpr unsigned kWristServo = 3U;    // 腕 roll，不参与解算

/**
 * @brief 角度 → 行程百分比
 * @details 限位只存角度，百分比由本函数在编译期派生，避免两套限位漂移。
 * @param calib 该关节的标定
 * @param deg 连杆角（度）
 * @return 行程百分比；deg_per_percent 为 0 时返回 0
 */
constexpr float percent_of(const arm_kinematics::JointCalib &calib, float deg) {
    // C++14 constexpr 里只能用四则运算（不能调 std::），所以这里自己防零除。
    return calib.deg_per_percent == 0.0F ? 0.0F : (deg - calib.zero_deg) / calib.deg_per_percent;
}

/**
 * @brief 四路行程限位（由角度限位派生）
 * @note 改角度限位就不用再改百分比；1% 约 0.9°。
 */
constexpr arm_joint::Limits kJointLimits[kServoCount] = {
    // 1% ≈ 0.9°：角度限位 → 行程百分比，改角度就不用再改百分比
    {percent_of(kGeom.shoulder, kGeom.shoulder.range_min_deg),
     percent_of(kGeom.shoulder, kGeom.shoulder.range_max_deg)},
    {percent_of(kGeom.elbow, kGeom.elbow.range_min_deg),
     percent_of(kGeom.elbow, kGeom.elbow.range_max_deg)},
    {-100.0F, 100.0F}, // 夹爪：无几何约束，全行程
    {-100.0F, 100.0F}, // 腕 roll：全行程
};

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 笛卡尔工作区（矩形，第一级钳位）
 * @details 穿肩轴、力矩过大、折叠死区都靠这四个边界挡掉；够不着由 IK 的圆环钳位负责。
 */
struct Workspace {
    float x_min_mm;
    float x_max_mm;
    float y_min_mm;
    float y_max_mm;
};

/**
 * @brief 笛卡尔工作区实例
 * @details 前半平面可达域是一条**弧带**，不是一个矩形：
 *          θ2 在右半平面只能用 [−42°,0]（超伸侧只有 42°，含 3° 余量），所以半径只能落在
 *          109.13 ~ 116.08 mm（6.95 mm 厚），围成一条从 β≈7° 扫到近乎竖直的带子；
 *          再往正前方（β→0°）大臂就要贴底盘了（kShoulderContactDeg），所以带子不是从 (116,0) 起的。
 *          域宽（前后）不随 x 变，随 β 变 —— 矩形只能保守包住，靠 solve_xy 的圆环钳位
 *          把框内偏心的命令点投影到弧带上，所以这里是"软边界"：
 *          框内不等于逐点可达，但任何点都会被钳到最近的弧带位置。
 * @note 要更大的平面工作区只能放开小臂前侧行程或换连杆，加工作区没用。
 *       TBD-11：实物工位摆好后，按实际抓取点再往里缩一次。
 */
constexpr Workspace kWorkspace = {70.0F, 116.0F, -20.0F, 85.0F};

/**
 * @brief 速度与手感参数
 * @note 字段顺序 = 位置初始化顺序，只在末尾追加。
 */
struct Motion {
    float cart_speed_mm_per_second;     // 右摇杆满偏时的末端速度
    float slew_percent_per_second;      // 关节插值限速（回中立位 / 追目标）
    float settle_percent;               // at_target 容差
    float sag_mm_per_100mm;             // 水平位下垂预补偿，0 = 关闭
    unsigned max_elapsed_ms;            // 单帧最大积分步长（主循环卡顿保护）
    int jog_percent_per_second[4];      // 夹爪/腕的满杆 jog 速度（轴 0/1 白名单禁止，值为占位）
    float stick_damping_mm_per_percent; // 摇杆关节兜底的阻尼 λ（mm/%）：越小越跟手，奇异区越抖
    float stick_box_tol_mm;             // 矩形当墙的线性化残差容差（mm）：法向剔除会残留这个量
};

/**
 * @brief 速度与手感实例
 * @note slew 沿用 car_config 原值 300 %/s；sag 默认 0 = 关闭（见风险表 R-1）。
 */
constexpr Motion kMotion = {
    50.0F,                // cart_speed_mm_per_second
    300.0F,               // slew_percent_per_second（car_config 原值）
    1.0F,                 // settle_percent
    0.0F,                 // sag：默认关闭，见风险表
    100U,                 // max_elapsed_ms
    {150, 150, 200, 150}, // jog: 肩/肘（占位）/夹爪/腕
    0.2F,                 // stick_damping：仿真实测 0.2 → 抖动 0.061 mm/帧²（0.02 时 3.08）
    0.02F,                // stick_box_tol：舵机一个步进 2.6 mm 的 1%，刚好盖住线性化残差
};

/**
 * @brief slew 余量是否够
 * @details 最坏情况（r=60 mm、末端 50 mm/s）实算需要约 53 %/s，这里要求 4 倍余量，
 *          否则末端精调会跟不上摇杆。
 */
constexpr bool slew_ok() {
    const float needed = kMotion.cart_speed_mm_per_second / 60.0F * arm_kinematics::kRadToDeg /
                         kGeom.shoulder.deg_per_percent;
    return kMotion.slew_percent_per_second > needed * 4.0F;
}

/**
 * @brief 一个腕点坐标（mm，原点在肩轴）
 * @note 固件里唯一的「目标姿态」就是中立位；v0.4 起不再有带 gripper/wrist 的预设表。
 */
struct Point {
    float x_mm;
    float y_mm;
};

/**
 * @brief 中立位（Start 的目标，也是上电点）
 * @details 必须等于 `arm_pose.cpp` 的 `kZeroPercent{-66, -86}` 的 FK，否则按 Start 第一帧会跳。
 * @note 坐标是 TBD-8 占位值：验过可达且在工作区内（β=20°、r=112），没对过实际工位，标定后替换。
 */
constexpr Point kNeutralPose = {105.2F, 38.3F};

/**
 * @brief 中立位是否落在工作区矩形内
 * @details 上电自检的一半；IK 的另一半（可达 / 限位）在运行期做。
 */
constexpr bool neutral_in_area() {
    return kNeutralPose.x_mm >= kWorkspace.x_min_mm && kNeutralPose.x_mm <= kWorkspace.x_max_mm &&
           kNeutralPose.y_mm >= kWorkspace.y_min_mm && kNeutralPose.y_mm <= kWorkspace.y_max_mm;
}

/* Exported macro ------------------------------------------------------------*/
/* 编译期自检：任何一条不过都编不过 ------------------------------------------*/

static_assert(kGeom.upper_mm > 0.0F && kGeom.fore_mm > 0.0F, "连杆长度必须为正");
static_assert(kGeom.elbow_sign == 1 || kGeom.elbow_sign == -1, "elbow_sign 只能是 ±1");
static_assert(inner_radius_ok(), "|L1-L2| 太小，折叠区数值病态，检查连杆长度");
static_assert(zero_at_mid(kGeom.shoulder) && zero_at_mid(kGeom.elbow),
              "zero_deg 必须等于行程中点：否则退余量会挪动 percent=0 的物理位置");
static_assert(span_within_travel(kGeom.shoulder) && span_within_travel(kGeom.elbow),
              "角度跨度超过舵机行程（200%），检查 range_*_deg 与 deg_per_percent");
static_assert(kStopBackoffDeg > 0.0F, "两端余量必须为正：0 就是回到「零余量顶死」的老问题");
static_assert(kShoulderContactDeg > 0.0F, "底盘接触角必须为正（asin(1/14) = 4.10）");
static_assert(arm_joint::kChannelCount == 4U, "关节层通道数与四路舵机不一致");
static_assert(kShoulderServo < kServoCount && kElbowServo < kServoCount &&
                  kGripperServo < kServoCount && kWristServo < kServoCount,
              "通道编号越界");
static_assert(kJointLimits[kShoulderServo].min_percent >= -100.0F &&
                  kJointLimits[kShoulderServo].max_percent <= 100.0F,
              "大臂角度限位超出舵机行程，检查 deg_per_percent");
static_assert(kJointLimits[kElbowServo].min_percent >= -100.0F &&
                  kJointLimits[kElbowServo].max_percent <= 100.0F,
              "小臂角度限位超出舵机行程，检查 zero_deg 与 deg_per_percent");
static_assert(kJointLimits[kShoulderServo].min_percent < kJointLimits[kShoulderServo].max_percent &&
                  kJointLimits[kElbowServo].min_percent < kJointLimits[kElbowServo].max_percent,
              "角度限位上下界颠倒");
static_assert(kWorkspace.x_min_mm > 0.0F, "工作区 x_min 必须为正，否则末端会穿过肩轴");
static_assert(kWorkspace.x_min_mm < kWorkspace.x_max_mm, "工作区 x 上下界颠倒");
static_assert(kWorkspace.y_min_mm < kWorkspace.y_max_mm, "工作区 y 上下界颠倒");
static_assert(kWorkspace.x_max_mm <= kGeom.upper_mm + kGeom.fore_mm,
              "工作区 x_max 超出 L1+L2，永远走不到");
static_assert(kMotion.cart_speed_mm_per_second > 0.0F, "满杆末端速度必须为正");
static_assert(kMotion.slew_percent_per_second > 0.0F, "slew 必须为正");
static_assert(kMotion.sag_mm_per_100mm >= 0.0F, "下垂补偿不允许为负");
static_assert(kMotion.jog_percent_per_second[kGripperServo] > 0, "夹爪 jog 速度必须为正");
static_assert(kMotion.stick_damping_mm_per_percent > 0.0F,
              "贴边滑阻尼必须为正：0 会让奇异区（θ2→0）来回抖");
static_assert(kMotion.stick_box_tol_mm > 0.0F,
              "矩形当墙的容差必须为正：法向剔除是线性化投影，容差 0 会被自己的残差判死");
static_assert(slew_ok(), "slew 余量不足 4 倍，末端精调会跟不上摇杆");
static_assert(neutral_in_area(), "中立位落在工作区矩形之外，按 Start 会被钳到边界");

} // namespace arm_config
