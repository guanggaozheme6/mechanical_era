/**
 ******************************************************************************
 * @file           : arm_kinematics.hpp
 * @brief          : 平面 2R 逆解/正解与角度换算（Algorithm 层）
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

// 约束（故意做成硬约束，改本文件时必须保持）：
//   * 只包含 <cmath> / <cstdint>，不 include HAL、不 include 任何工程配置；
//   * 没有任何全局可变状态（全部函数是纯函数）；
//   * 不读 HAL_GetTick、不认识 ArmCommand、不认识 Ps2Frame；
//   * Model 由调用方以 const& 传入 —— 参数与算法分离，PC / 上位机 / 另一
//     个工程可以把这几个文件和自己的配置一起编，不需要改一行代码。

namespace arm_kinematics {

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 单关节的机械标定 + 角度限位
 * @details 角度限位是唯一真值；行程百分比的限位由它派生（见 arm_config.hpp）。
 * @note 字段顺序 = 位置初始化顺序，只在末尾追加（C++14 没有 designated initializer）。
 */
struct JointCalib {
    float zero_deg;        // percent == 0 时连杆相对 +x 的角度
    float deg_per_percent; // 带符号：1% 行程对应的角度（0.9 表示 180° 舵机）
    float range_min_deg;   // 结构允许的最小连杆角
    float range_max_deg;   // 结构允许的最大连杆角
};

/**
 * @brief 平面 2R 几何模型
 * @note 只装几何量；工作区、速度、姿态表属于 arm_config，不进这里。
 */
struct Model {
    float upper_mm; // L1 肩轴心 → 肘轴心
    float fore_mm;  // L2 肘轴心 → 腕点
    JointCalib shoulder;
    JointCalib elbow;
    int elbow_sign; // +1 = 小臂向上兜（唯一分支，运行期无歧义）
};

/**
 * @brief 一次逆解的完整结果
 * @details 角度字段是串口标定要打印的量；百分比用 float 是因为精调时整数量化
 *          会让末端出现可见跳变。
 */
struct Solution {
    bool reachable;         // 目标在圆环内（未被径向钳位）
    bool limited;           // 关节角被 range_*_deg 裁剪
    float x_mm;             // FK 回算的实际末端（钳位 + 取整后）
    float y_mm;             //
    float shoulder_deg;     // 钳位后的连杆角
    float elbow_deg;        //
    float shoulder_percent; // float，精调必须（int 会卡顿跳变）
    float elbow_percent;    //
};

/* Exported constants --------------------------------------------------------*/

/**
 * @brief 限位比较容差（度）
 * @note 边界点（θ 正好等于 range_*）不误报 limited。
 */
constexpr float kLimitEpsDeg = 1.0e-3F;

/**
 * @brief 认为目标落在肩轴上的半径阈值（mm）
 * @note r 小于此值时方向未定义，禁止做 r_min / r 的除法。
 */
constexpr float kMinRadiusMm = 1.0e-3F;

constexpr float kRadToDeg = 57.29577951308232F;      // 弧度 → 度
constexpr float kDegToRad = 0.017453292519943295F;  // 度 → 弧度

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 平面 2R 逆解
 * @details 输入腕点坐标，输出两路行程百分比；内部做圆环钳位、角度限位钳位与 FK 自检。
 * @param model 几何模型
 * @param x_mm 目标腕点 x（mm，原点在肩轴，+x 指向前方）
 * @param y_mm 目标腕点 y（mm，+y 指向上方）
 * @return 逆解结果；不可达 / 超限 / 非法输入一律返回有限数值，绝不返回 NaN
 */
Solution solve_xy(const Model &model, float x_mm, float y_mm);

/**
 * @brief 正解
 * @details float 版本给闭环积分用，也是标定核对的唯一手段。
 * @param model 几何模型
 * @param shoulder_percent 大臂行程百分比
 * @param elbow_percent 小臂行程百分比
 * @param x_mm 输出：末端 x（mm）
 * @param y_mm 输出：末端 y（mm）
 */
void solve_forward(const Model &model, float shoulder_percent, float elbow_percent, float &x_mm,
                   float &y_mm);

/**
 * @brief 行程百分比 → 连杆角
 * @param calib 该关节的标定
 * @param percent 行程百分比
 * @return 连杆角（度）；非有限输入返回 zero_deg
 */
float percent_to_deg(const JointCalib &calib, float percent);

/**
 * @brief 连杆角 → 行程百分比
 * @param calib 该关节的标定
 * @param deg 连杆角（度）
 * @return 行程百分比；非有限输入或 deg_per_percent == 0 时返回 0
 */
float deg_to_percent(const JointCalib &calib, float deg);

/**
 * @brief 行程百分比限幅到 -100..100
 * @details 下游 outputs 还有一道，双重保险。
 * @param percent 输入百分比
 * @return 限幅结果
 */
float clamp_percent(float percent);

} // namespace arm_kinematics
