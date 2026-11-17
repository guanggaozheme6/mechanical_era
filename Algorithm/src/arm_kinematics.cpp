#include "arm_kinematics.hpp"

#include <cmath>

namespace arm_kinematics {
namespace {

float clampf(float value, float lower, float upper) {
    return value < lower ? lower : (value > upper ? upper : value);
}

bool finite(float value) { return std::isfinite(value); }

} // namespace

float clamp_percent(float percent) { return clampf(percent, -100.0F, 100.0F); }

float percent_to_deg(const JointCalib &calib, float percent) {
    if (!finite(percent) || !finite(calib.zero_deg) || !finite(calib.deg_per_percent)) {
        return calib.zero_deg;
    }
    const float deg = calib.zero_deg + percent * calib.deg_per_percent;
    return finite(deg) ? deg : calib.zero_deg;
}

float deg_to_percent(const JointCalib &calib, float deg) {
    // 双保险：上游任何一环出 NaN，这里都不允许把 NaN 转成 int 送进 TIM8。
    if (!finite(deg) || !finite(calib.zero_deg) || !finite(calib.deg_per_percent) ||
        calib.deg_per_percent == 0.0F) {
        return 0.0F;
    }
    const float percent = (deg - calib.zero_deg) / calib.deg_per_percent;
    return finite(percent) ? percent : 0.0F;
}

void solve_forward(const Model &model, float shoulder_percent, float elbow_percent, float &x_mm,
                   float &y_mm) {
    const float t1 = percent_to_deg(model.shoulder, shoulder_percent) * kDegToRad;
    const float t2 = percent_to_deg(model.elbow, elbow_percent) * kDegToRad;
    const float sum = t1 + t2;
    x_mm = model.upper_mm * std::cos(t1) + model.fore_mm * std::cos(sum);
    y_mm = model.upper_mm * std::sin(t1) + model.fore_mm * std::sin(sum);
}

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

    // ---- 2. 圆环钳位：|L1-L2| <= r <= L1+L2 ----
    const float r = std::sqrt(x * x + y * y);
    const float r_max = l1 + l2;
    float r_min = std::fabs(l1 - l2);
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

    const float t2 = static_cast<float>(model.elbow_sign) * std::acos(cos_elbow);
    const float t1 = std::atan2(ty, tx) -
                     std::atan2(l2 * std::sin(t2), l1 + l2 * std::cos(t2));

    float d1 = t1 * kRadToDeg;
    float d2 = t2 * kRadToDeg;
    if (!finite(d1)) {
        d1 = model.shoulder.zero_deg;
    }
    if (!finite(d2)) {
        d2 = model.elbow.zero_deg;
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
