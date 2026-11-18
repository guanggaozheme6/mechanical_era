/**
 ******************************************************************************
 * @file           : arm_kinematics.cpp
 * @brief          : 平面 2R 逆解/正解与角度换算实现（Algorithm 层）
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "arm_kinematics.hpp"

#include <cmath>

namespace arm_kinematics {

/* Private functions ---------------------------------------------------------*/
namespace {

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
 * @brief 是否有限值
 * @param value 输入
 * @return true 表示既非 NaN 也非 Inf
 */
bool finite(float value) { return std::isfinite(value); }

} // namespace

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 行程百分比限幅到 -100..100
 * @param percent 输入百分比
 * @return 限幅结果
 */
float clamp_percent(float percent) { return clampf(percent, -100.0F, 100.0F); }

/**
 * @brief 某一侧的最小可达半径
 * @details 右半平面只能用 θ2 的负支（吃 range_min 的绝对值），左半平面用正支（吃 range_max）。
 *          超过 180° 的部分按镜像折算（转过死点等价于 360°−θ2）。
 * @param model 几何模型
 * @param wrist_right true = 目标在右半平面
 * @return 内半径（mm），不小于 |L1−L2|
 */
float min_reachable_radius(const Model &model, bool wrist_right) {
    float t = wrist_right ? std::fabs(model.elbow.range_min_deg) : model.elbow.range_max_deg;
    if (t < 0.0F) {
        t = -t;
    }
    if (t > 180.0F) {
        t = 360.0F - t;
    }
    const float c = std::cos(t * kDegToRad);
    float r2 = model.upper_mm * model.upper_mm + model.fore_mm * model.fore_mm +
               2.0F * model.upper_mm * model.fore_mm * c;
    if (!finite(r2) || r2 < 0.0F) {
        r2 = 0.0F;
    }
    const float r = std::sqrt(r2);
    const float geo = std::fabs(model.upper_mm - model.fore_mm);
    return r > geo ? r : geo;
}

/**
 * @brief 行程百分比 → 连杆角
 * @param calib 该关节的标定
 * @param percent 行程百分比
 * @return 连杆角（度）；非有限输入返回 zero_deg
 */
float percent_to_deg(const JointCalib &calib, float percent) {
    if (!finite(percent) || !finite(calib.zero_deg) || !finite(calib.deg_per_percent)) {
        return calib.zero_deg;
    }
    const float deg = calib.zero_deg + percent * calib.deg_per_percent;
    return finite(deg) ? deg : calib.zero_deg;
}

/**
 * @brief 连杆角 → 行程百分比
 * @param calib 该关节的标定
 * @param deg 连杆角（度）
 * @return 行程百分比；非有限输入或 deg_per_percent == 0 时返回 0
 */
float deg_to_percent(const JointCalib &calib, float deg) {
    // 双保险：上游任何一环出 NaN，这里都不允许把 NaN 转成 int 送进 TIM8。
    if (!finite(deg) || !finite(calib.zero_deg) || !finite(calib.deg_per_percent) ||
        calib.deg_per_percent == 0.0F) {
        return 0.0F;
    }
    const float percent = (deg - calib.zero_deg) / calib.deg_per_percent;
    return finite(percent) ? percent : 0.0F;
}

/**
 * @brief 正解：两路行程百分比 → 末端坐标
 * @details 逐级 percent_to_deg 再按两段连杆求和，全程 float，供闭环积分用。
 * @param model 几何模型
 * @param shoulder_percent 大臂行程百分比
 * @param elbow_percent 小臂行程百分比
 * @param x_mm 输出：末端 x（mm）
 * @param y_mm 输出：末端 y（mm）
 */
void solve_forward(const Model &model, float shoulder_percent, float elbow_percent, float &x_mm,
                   float &y_mm) {
    const float t1 = percent_to_deg(model.shoulder, shoulder_percent) * kDegToRad;
    const float t2 = percent_to_deg(model.elbow, elbow_percent) * kDegToRad;
    const float sum = t1 + t2;
    x_mm = model.upper_mm * std::cos(t1) + model.fore_mm * std::cos(sum);
    y_mm = model.upper_mm * std::sin(t1) + model.fore_mm * std::sin(sum);
}

/**
 * @brief 平面 2R 逆解（含五道数值防护）
 * @details 顺序：非法输入兜底 → 圆环钳位 → 余弦定理求肘/肩角 → 角度限位钳位 → FK 自检。
 * @param model 几何模型
 * @param x_mm 目标腕点 x（mm）
 * @param y_mm 目标腕点 y（mm）
 * @return 逆解结果；不可达 / 超限 / 非法输入一律返回有限数值，绝不返回 NaN
 */
Solution solve_xy(const Model &model, float x_mm, float y_mm) {
    Solution solution{};
    solution.reachable = true;
    solution.limited = false;

    const float l1 = model.upper_mm;
    const float l2 = model.fore_mm;

    // ---- 1. 非法输入兜底（D2 的扩展：不只 r==0，NaN/Inf 也不能进来）----
    float x = x_mm;
    float y = y_mm;
    if (!finite(x) || !finite(y)) {
        x = 0.0F;
        y = 0.0F;
        solution.reachable = false;
    }

    // ---- 2. 圆环钳位：内边界取「角度限位允许的最小半径」，外边界 L1+L2 ----
    //      内边界不能用 |L1−L2|：那要求 θ2=180°；B 方案右半平面只到 −45°、左半平面到 +135°。
    //      越界一律沿目标方位缩/放到边界上，保证"推到头就顶住"，不出现跳空。
    const float r = std::sqrt(x * x + y * y);
    const float r_max = l1 + l2;
    float r_min = min_reachable_radius(model, x >= 0.0F);
    if (!(r_min > kMinRadiusMm)) {
        r_min = kMinRadiusMm; // 退化结构（L1==L2）时避免零半径
    }

    float tx = x;
    float ty = y;
    if (r > r_max) {
        // 太远：沿目标方位缩到最大伸展半径。
        const float scale = r_max / r;
        tx = x * scale;
        ty = y * scale;
        solution.reachable = false;
    } else if (r < r_min) {
        // 太近：缩到折叠半径。r 接近 0 时方向未定义，绝不做 r_min / r。
        if (r < kMinRadiusMm) {
            tx = r_min;
            ty = 0.0F;
        } else {
            const float scale = r_min / r;
            tx = x * scale;
            ty = y * scale;
        }
        solution.reachable = false;
    }

    // ---- 3. 余弦定理求肘角，再求肩角 ----
    const float rr = std::sqrt(tx * tx + ty * ty);
    float cos_elbow = (rr * rr - l1 * l1 - l2 * l2) / (2.0F * l1 * l2);
    if (!finite(cos_elbow)) {
        cos_elbow = -1.0F;
    }
    cos_elbow = clampf(cos_elbow, -1.0F, 1.0F); // 必须钳，否则 acos 出 NaN

    // 两支镜像解共用同一个 |θ2| = a 和同一个修正角 α，差别只在符号：
    //   θ2 = +a ⇒ θ1 = β − α      θ2 = −a ⇒ θ1 = β + α
    // 所以先把 a、α、β 各自算一遍，选支就只是一个符号，三角函数次数与只算一支时相同。
    const float a = std::acos(cos_elbow);
    const float alpha = std::atan2(l2 * std::sin(a), l1 + l2 * std::cos(a));
    const float beta = std::atan2(ty, tx);

    // 选支：永远取"肘点更高"的那一支（上吊构型：大臂竖起、小臂从高处往下伸）。
    // 两支的肘高差 = L1·[sin(β+α) − sin(β−α)] = 2·L1·cosβ·sinα，cosβ 与 tx 同号，
    // 所以"取高肘"等价于「θ2 的符号与目标 x 的符号相反」—— 一个乘法。
    // 效果：左半平面和右半平面解出来的构型关于 +y 轴镜像、形状完全一致；
    // 右下方的近点还会自动避开 θ1 < 0 那个倒退支。
    // elbow_sign 保留为整体镜像开关：+1 = 高肘（默认），-1 = 低肘。
    // 这条规则要求小臂限位跨 0（B 方案的 −45°~+135° 满足）；若限位退化成 [0,180]、
    // 只剩 +θ2 一支，右半平面会被整片判成超限 —— 换限位时必须一起看这条。
    // 注意：目标正好落在 +y 轴上时两支等高，跨过 x=0 会翻支（θ1 从 β+α 跳到 β−α），
    // 折叠角大时这一次跳变幅度可观，标定时别沿着肩轴竖线走。
    const float side = (tx >= 0.0F) ? -1.0F : 1.0F;
    const float t2 = static_cast<float>(model.elbow_sign) * side * a;
    const float t1 = beta + (t2 >= 0.0F ? -alpha : alpha);

    float d1 = t1 * kRadToDeg;
    float d2 = t2 * kRadToDeg;
    if (!finite(d1)) {
        d1 = model.shoulder.zero_deg;
    }
    if (!finite(d2)) {
        d2 = model.elbow.zero_deg;
    }
    // θ1 归一化到 (-180, 180]：目标在第三象限时 β−α 会掉到 −180° 以下（例如 −188°），
    // 那只是绕了一圈，物理角应当是 +172°。不归一化会让左下角的点被误判超限。
    if (d1 <= -180.0F) {
        d1 += 360.0F;
    } else if (d1 > 180.0F) {
        d1 -= 360.0F;
    }

    // ---- 4. 角度限位。带容差，边界点不误报（D3）----
    const float d1_clamped = clampf(d1, model.shoulder.range_min_deg, model.shoulder.range_max_deg);
    const float d2_clamped = clampf(d2, model.elbow.range_min_deg, model.elbow.range_max_deg);
    solution.limited = (std::fabs(d1_clamped - d1) > kLimitEpsDeg) ||
                       (std::fabs(d2_clamped - d2) > kLimitEpsDeg);

    solution.shoulder_deg = d1_clamped;
    solution.elbow_deg = d2_clamped;
    solution.shoulder_percent = clamp_percent(deg_to_percent(model.shoulder, d1_clamped));
    solution.elbow_percent = clamp_percent(deg_to_percent(model.elbow, d2_clamped));

    // ---- 5. FK 回算：没有编码器的开环系统里唯一能发现"标定填错"的手段 ----
    solve_forward(model, solution.shoulder_percent, solution.elbow_percent, solution.x_mm,
                  solution.y_mm);
    if (!finite(solution.x_mm) || !finite(solution.y_mm)) {
        solution.x_mm = 0.0F;
        solution.y_mm = 0.0F;
    }
    return solution;
}

} // namespace arm_kinematics
