#pragma once

// ============================================================================
//  arm_kinematics —— 平面 2R 纯几何模块
// ----------------------------------------------------------------------------
//  约束（故意做成硬约束，改本文件时必须保持）：
//    * 只包含 <cmath> / <cstdint>，不 include HAL、不 include 任何工程配置；
//    * 没有任何全局可变状态（全部函数是纯函数）；
//    * 不读 HAL_GetTick、不认识 ArmCommand、不认识 Ps2Frame；
//    * Model 由调用方以 const& 传入 —— 参数与算法分离，PC / 上位机 / 另一
//      个工程可以把这四个文件和自己的配置一起编，不需要改一行代码。
//  因此本模块可以在 PC 上用 g++ 直接单测，也可以被 Cortex-M3 交叉编译。
// ============================================================================

namespace arm_kinematics {

// 单个关节的机械标定 + 角度限位（角度限位的唯一真值；百分比限位由它派生）。
// 字段顺序 = 位置初始化顺序，只在末尾追加。
struct JointCalib {
    float zero_deg;         // percent == 0 时连杆相对 +x 的角度
    float deg_per_percent;  // 带符号：1% 行程对应的角度（0.9 表示 180° 舵机）
    float range_min_deg;    // 结构允许的最小连杆角
    float range_max_deg;    // 结构允许的最大连杆角
};

// 只装几何量。工作区、速度、姿态表属于 arm_config，不进这里。
struct Model {
    float upper_mm;    // L1 肩轴心 → 肘轴心
    float fore_mm;     // L2 肘轴心 → 腕点
    JointCalib shoulder;
    JointCalib elbow;
    int elbow_sign;    // +1 = 小臂向上兜（唯一分支，运行期无歧义）
};

// 一次逆解的完整结果。角度字段是 D4 要求的（串口标定要打印）。
struct Solution {
    bool reachable;          // 目标在圆环内（未被径向钳位）
    bool limited;            // 关节角被 range_*_deg 裁剪
    float x_mm;              // FK 回算的实际末端（钳位 + 取整后）
    float y_mm;
    float shoulder_deg;      // 钳位后的连杆角
    float elbow_deg;
    float shoulder_percent;  // float，精调必须（int 会卡顿跳变）
    float elbow_percent;
};

// 限位比较容差：边界点（θ 正好等于 range_*）不误报 limited（D3）。
constexpr float kLimitEpsDeg = 1.0e-3F;
// r 小于此值视为"目标落在肩轴上"，方向未定义，禁止做除法（D2）。
constexpr float kMinRadiusMm = 1.0e-3F;
constexpr float kRadToDeg = 57.29577951308232F;
constexpr float kDegToRad = 0.017453292519943295F;

// 平面 2R 逆解。输入腕点坐标，输出两路行程百分比。
// 不可达 / 超限 / 非法输入一律返回有限数值，绝不返回 NaN（D2）。
Solution solve_xy(const Model &model, float x_mm, float y_mm);

// 正解。float 版本给闭环积分用，也是标定核对的唯一手段。
void solve_forward(const Model &model, float shoulder_percent, float elbow_percent, float &x_mm,
                   float &y_mm);

// 角度 ↔ 百分比。非有限输入返回 0，避免 INT_MIN 脉宽（D2 兜底）。
float percent_to_deg(const JointCalib &calib, float percent);
float deg_to_percent(const JointCalib &calib, float deg);

// -100..100 钳位（下游 outputs.cpp 还有一道，双重保险）。
float clamp_percent(float percent);

} // namespace arm_kinematics
