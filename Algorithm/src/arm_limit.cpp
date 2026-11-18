/**
 ******************************************************************************
 * @file           : arm_limit.cpp
 * @brief          : 摇杆软件限位：两段式（位置法 + 关节域贴边滑）实现
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "arm_limit.hpp"

#include <cmath>

namespace arm_limit {

/* Private constants ---------------------------------------------------------*/

namespace {

/** @brief 比较容差：float 下比这更小的差一律视为「没动 / 没越界」 */
constexpr float kEps = 1.0e-6F;

/** @brief 平方量的比较容差（如 |J_j|²） */
constexpr float kEpsSq = 1.0e-12F;

/** @brief λ 缺省值（mm/%）：只在调用方给了非法 damping 时兜底 */
constexpr float kDefaultDamping = 1.0F;

/** @brief δt 缺省（秒）：调用方给了非法 dt 时按「不动」处理 */
constexpr float kZeroDt = 0.0F;

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

/**
 * @brief 候选姿态是否可用（只判工作区矩形）
 * @details 矩形当「墙」而不是「投影面」：越界就判不可行，让上层的候选去试别的方向，
 *          而不是把末端拽到边上（拽到边上正是末端域的毛病）。
 *          唯一的例外是「不比起点更越界」：顶在边界上时，任何关节运动都会让末端在法向上
 *          多出零点零几毫米（法向剔除是线性化投影，有残差），若严格按「必须进框」判，
 *          沿边切向滑会全被判死。所以判据是「越界量 ≤ max(起始越界量, tol)」。
 * @param model 几何模型
 * @param p1 大臂行程百分比
 * @param p2 小臂行程百分比
 * @param box 工作区矩形
 * @param base_over 起点本来就越界多少（mm）
 * @param tol 线性化残差容差（mm）
 * @return true 表示可下发
 * @note 本层没有「撞底盘 / 钻地面」的判据：真要做干涉，就在这里加一段线段采样，
 *       或由调用方在拿到 Step 之后再 gate 一次。
 */
bool feasible_at(const arm_kinematics::Model &model, float p1, float p2, const Box &box,
                 float base_over, float tol) {
    const float allowed = base_over > tol ? base_over : tol;
    return overflow(model, p1, p2, box) <= allowed + kEps;
}

} // namespace

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 2R 末端雅可比（解析式）
 * @param model 几何模型
 * @param p1 大臂行程百分比
 * @param p2 小臂行程百分比
 * @return 雅可比（mm/%）
 */
Jacobian jacobian(const arm_kinematics::Model &model, float p1, float p2) {
    const float t1 = arm_kinematics::percent_to_deg(model.shoulder, p1) * arm_kinematics::kDegToRad;
    const float t2 = arm_kinematics::percent_to_deg(model.elbow, p2) * arm_kinematics::kDegToRad;
    const float s = t1 + t2;
    const float l1 = model.upper_mm;
    const float l2 = model.fore_mm; // 末端 = 腕点，与 solve_forward 同口径
    const float k1 = model.shoulder.deg_per_percent * arm_kinematics::kDegToRad;
    const float k2 = model.elbow.deg_per_percent * arm_kinematics::kDegToRad;

    Jacobian J;
    J.j11 = (-l1 * std::sin(t1) - l2 * std::sin(s)) * k1;
    J.j12 = (-l2 * std::sin(s)) * k2;
    J.j21 = (l1 * std::cos(t1) + l2 * std::cos(s)) * k1;
    J.j22 = (l2 * std::cos(s)) * k2;
    return J;
}

/**
 * @brief 2×2 雅可比的奇异性指标（σmax/σmin）
 * @details 用奇异值比值的闭式解，不调 SVD：2×2 的 σ 由 trace 与行列式直接给出。
 * @param J 雅可比
 * @return 条件数；完全奇异时返回 HUGE_VALF
 */
float condition(const Jacobian &J) {
    const float a = J.j11;
    const float b = J.j12;
    const float c = J.j21;
    const float d = J.j22;
    const float sum = a * a + b * b + c * c + d * d;
    const float cross = a * c + b * d;
    const float dif = a * a + b * b - c * c - d * d;
    const float disc = std::sqrt(std::fmax(0.0F, dif * dif + 4.0F * cross * cross));
    const float s1 = std::sqrt(std::fmax(0.0F, (sum + disc) / 2.0F));
    const float s2 = std::sqrt(std::fmax(0.0F, (sum - disc) / 2.0F));
    if (!(s2 > kEpsSq)) {
        return HUGE_VALF;
    }
    return s1 / s2;
}

/**
 * @brief 阻尼最小二乘
 * @param J 雅可比（mm/%）
 * @param wx 期望末端位移 x（mm）
 * @param wy 期望末端位移 y（mm）
 * @param damping 阻尼 λ（mm/%）
 * @return 关节增量（%）
 */
JointStep dls(const Jacobian &J, float wx, float wy, float damping) {
    JointStep out{0.0F, 0.0F, false, false};
    const float lam2 = damping * damping;
    const float a11 = J.j11 * J.j11 + J.j12 * J.j12 + lam2;
    const float a12 = J.j11 * J.j21 + J.j12 * J.j22;
    const float a22 = J.j21 * J.j21 + J.j22 * J.j22 + lam2;
    const float det = a11 * a22 - a12 * a12;
    if (!(std::fabs(det) > kEpsSq)) {
        return out; // 奇异到无解：这一候选直接放弃，交给后面的单关节兜底
    }
    const float i11 = a22 / det;
    const float i12 = -a12 / det;
    const float i22 = a11 / det;
    const float u1 = i11 * wx + i12 * wy;
    const float u2 = i12 * wx + i22 * wy;
    out.dp1 = J.j11 * u1 + J.j21 * u2;
    out.dp2 = J.j12 * u1 + J.j22 * u2;
    return out;
}

/**
 * @brief 带饱和重解的关节步
 * @param J 雅可比（mm/%）
 * @param wx 期望末端位移 x（mm）
 * @param wy 期望末端位移 y（mm）
 * @param p1 当前大臂行程百分比
 * @param p2 当前小臂行程百分比
 * @param limits 两路行程限位
 * @param damping 阻尼 λ（mm/%）
 * @return 关节增量 + 哪两轴被固定
 */
JointStep solve_joint_step(const Jacobian &J, float wx, float wy, float p1, float p2,
                           const Span (&limits)[2], float damping) {
    const float lam2 = damping * damping;
    const JointStep dq = dls(J, wx, wy, damping);
    const float raw1 = p1 + dq.dp1;
    const float raw2 = p2 + dq.dp2;
    const bool over1 = (raw1 < limits[0].min_percent - kEps) || (raw1 > limits[0].max_percent + kEps);
    const bool over2 = (raw2 < limits[1].min_percent - kEps) || (raw2 > limits[1].max_percent + kEps);
    if (!over1 && !over2) {
        return dq;
    }

    const float d1 = clampf(raw1, limits[0].min_percent, limits[0].max_percent) - p1;
    const float d2 = clampf(raw2, limits[1].min_percent, limits[1].max_percent) - p2;

    JointStep out{0.0F, 0.0F, false, false};
    if (over2 && !over1) {
        // 小臂固定：扣掉它已经贡献的位移，剩下的交给大臂（单列 DLS）
        const float rx = wx - J.j12 * d2;
        const float ry = wy - J.j22 * d2;
        const float den = J.j11 * J.j11 + J.j21 * J.j21 + lam2;
        const float k = den > kEpsSq ? (J.j11 * rx + J.j21 * ry) / den : 0.0F;
        out.dp1 = clampf(p1 + k, limits[0].min_percent, limits[0].max_percent) - p1;
        out.dp2 = d2;
        out.fixed2 = true;
        return out;
    }
    if (over1 && !over2) {
        // 大臂固定：对称处理
        const float rx = wx - J.j11 * d1;
        const float ry = wy - J.j21 * d1;
        const float den = J.j12 * J.j12 + J.j22 * J.j22 + lam2;
        const float k = den > kEpsSq ? (J.j12 * rx + J.j22 * ry) / den : 0.0F;
        out.dp1 = d1;
        out.dp2 = clampf(p2 + k, limits[1].min_percent, limits[1].max_percent) - p2;
        out.fixed1 = true;
        return out;
    }
    // 两轴都饱和：只剩被裁掉的那点位移，交给上层判 STUCK / 滑停
    out.dp1 = d1;
    out.dp2 = d2;
    out.fixed1 = true;
    out.fixed2 = true;
    return out;
}

/**
 * @brief 剔掉一步里的法向分量（贴矩形边时的切向滑）
 * @param J 雅可比（mm/%）
 * @param step 待投影的关节步
 * @param from_x 步进前末端 x（mm）
 * @param from_y 步进前末端 y（mm）
 * @param box 工作区矩形
 * @param damping 阻尼 λ（mm/%）
 * @return 投影后的关节步
 */
JointStep flatten_to_rect(const Jacobian &J, JointStep step, float from_x, float from_y,
                          const Box &box, float damping) {
    const float pred_x = from_x + J.j11 * step.dp1 + J.j12 * step.dp2;
    const float pred_y = from_y + J.j21 * step.dp1 + J.j22 * step.dp2;
    const float over0 =
        std::fmax(0.0F, std::fmax(std::fmax(box.x_min_mm - from_x, from_x - box.x_max_mm),
                                  std::fmax(box.y_min_mm - from_y, from_y - box.y_max_mm)));
    const float over1 =
        std::fmax(0.0F, std::fmax(std::fmax(box.x_min_mm - pred_x, pred_x - box.x_max_mm),
                                  std::fmax(box.y_min_mm - pred_y, pred_y - box.y_max_mm)));
    if (over1 <= over0 + kEps) {
        return step; // 没有越界得更多，不管
    }

    float nx = 0.0F;
    float ny = 0.0F;
    if (pred_x > box.x_max_mm) {
        nx = 1.0F;
    } else if (pred_x < box.x_min_mm) {
        nx = -1.0F;
    }
    if (pred_y > box.y_max_mm) {
        ny = 1.0F;
    } else if (pred_y < box.y_min_mm) {
        ny = -1.0F;
    }
    if (nx == 0.0F && ny == 0.0F) {
        return step;
    }

    const float a1 = nx * J.j11 + ny * J.j21;
    const float a2 = nx * J.j12 + ny * J.j22;
    const float den = a1 * a1 + a2 * a2 + damping * damping;
    if (!(den > kEpsSq)) {
        return step;
    }
    const float k = (a1 * step.dp1 + a2 * step.dp2) / den;
    step.dp1 -= a1 * k;
    step.dp2 -= a2 * k;
    return step;
}

/**
 * @brief 末端越出工作区矩形多少（mm）
 * @param model 几何模型
 * @param p1 大臂行程百分比
 * @param p2 小臂行程百分比
 * @param box 工作区矩形
 * @return 越界量（0 = 在框内）
 */
float overflow(const arm_kinematics::Model &model, float p1, float p2, const Box &box) {
    float x = 0.0F;
    float y = 0.0F;
    arm_kinematics::solve_forward(model, p1, p2, x, y);
    return std::fmax(0.0F, std::fmax(std::fmax(box.x_min_mm - x, x - box.x_max_mm),
                                     std::fmax(box.y_min_mm - y, y - box.y_max_mm)));
}

/**
 * @brief 一步：位置法优先，关节域兜底
 * @param model 几何模型
 * @param limits 两路行程限位
 * @param box 工作区矩形
 * @param params 调参
 * @param p1 当前大臂行程百分比
 * @param p2 当前小臂行程百分比
 * @param vx_mm_per_s 摇杆给的末端 x 速度
 * @param vy_mm_per_s 摇杆给的末端 y 速度
 * @param dt_s 本帧时长（秒）
 * @return 本帧结果
 */
Step tangent_step(const arm_kinematics::Model &model, const Span (&limits)[2], const Box &box,
                  const Params &params, float p1, float p2, float vx_mm_per_s, float vy_mm_per_s,
                  float dt_s) {
    // 关节状态全程 float：贴边滑每帧可能只增 0.004%，任何中间取整都会把增量抹成 0。
    const float q1 = arm_kinematics::clamp_percent(p1);
    const float q2 = arm_kinematics::clamp_percent(p2);

    float fx = 0.0F;
    float fy = 0.0F;
    arm_kinematics::solve_forward(model, q1, q2, fx, fy); // 闭环于「当前实际末端」

    Step out{q1, q2, false, false, false, false, false};

    const float dt = (finite(dt_s) && dt_s > kZeroDt) ? dt_s : kZeroDt;
    const float want_x = (finite(vx_mm_per_s) ? vx_mm_per_s : 0.0F) * dt;
    const float want_y = (finite(vy_mm_per_s) ? vy_mm_per_s : 0.0F) * dt;
    const float mag = std::sqrt(want_x * want_x + want_y * want_y);
    if (!finite(mag) || !(mag > kEps)) {
        out.idle = true; // 摇杆没给位移：原样返回，别浪费一次逆解
        return out;
    }

    // base_over：起点本来就越界多少。判据是「不许越界得更多」，所以起点被钳出框时
    // 也不会整帧卡死（它会先往回走，越界量单调下降）。
    const float base_over = overflow(model, q1, q2, box);
    const float tol = finite(params.tol_mm) ? params.tol_mm : 0.0F;
    const float sag = finite(params.sag_mm_per_100mm) ? params.sag_mm_per_100mm : 0.0F;
    const float damping = (finite(params.damping) && params.damping > 0.0F) ? params.damping
                                                                           : kDefaultDamping;

    // ---- ① 位置法：沿摇杆方向直线走，解不出就缩步 ----
    for (unsigned i = 0U; i < kScaleCount; ++i) {
        const float s = kScales[i];
        const float tx = fx + want_x * s;
        const float ty = fy + want_y * s + sag * tx / 100.0F; // 下垂预补偿（0 = 关闭）
        const arm_kinematics::Solution sol = arm_kinematics::solve_xy(model, tx, ty);
        // 圆环钳位（reachable=false）和角度越限（limited=true）都不接受：
        // 前者就是末端域的「钳位照发」，后者是固件的整帧拒发 —— 两个都不要。
        if (!sol.reachable || sol.limited) {
            continue;
        }
        const float n1 = arm_kinematics::clamp_percent(sol.shoulder_percent);
        const float n2 = arm_kinematics::clamp_percent(sol.elbow_percent);
        if (!feasible_at(model, n1, n2, box, base_over, tol)) {
            continue;
        }
        out.p1_percent = n1;
        out.p2_percent = n2;
        out.slide = (i > 0U); // 缩了步就是贴边滑（位置法自己没被裁剪，cut 恒 false）
        return out;
    }

    // ---- ② 关节兜底：位置法一步都走不了，说明正对着一堵墙 ----
    const Jacobian J = jacobian(model, q1, q2);

    // 候选关节步，按优先级排：
    //   ①②③ 摇杆原方向 / x 单轴 / y 单轴（轴对齐的矩形边）
    //   ④⑤   单关节兜底（只动大臂 / 只动小臂）
    // ④⑤ 不是可选项：干涉墙的法向不在 J 的像空间里 —— 末端要往车前方地面伸时，
    // 「大臂几乎放平」这条路会压到车体上表面，而唯一走得通的「只折叠小臂」方向
    // 在 DLS 的最小范数解里根本不会出现（DLS 总是两个关节一起动）。没有这两条，
    // 摇杆推到那个位置就整帧 STUCK，看着像卡死。
    JointStep cand[5];
    unsigned count = 0U;

    const float wx[3] = {want_x, want_x, 0.0F};
    const float wy[3] = {want_y, 0.0F, want_y};
    for (unsigned i = 0U; i < 3U; ++i) {
        if (!(std::sqrt(wx[i] * wx[i] + wy[i] * wy[i]) > kEps)) {
            continue;
        }
        cand[count] = flatten_to_rect(
            J, solve_joint_step(J, wx[i], wy[i], q1, q2, limits, damping), fx, fy, box, damping);
        ++count;
    }

    // 单关节：把摇杆方向投影到该关节那一列（k = Jⱼ·w / |Jⱼ|²，保证位移与摇杆方向同向
    // 且不超过 |w|）。投影后再过法向剔除，剩下的就是「贴着边能走的那一点」。
    for (unsigned j = 0U; j < 2U; ++j) {
        const float cx = (j == 0U) ? J.j11 : J.j12;
        const float cy = (j == 0U) ? J.j21 : J.j22;
        const float n2 = cx * cx + cy * cy;
        if (!(n2 > kEpsSq)) {
            continue;
        }
        const float k = (cx * want_x + cy * want_y) / n2;
        if (!(std::fabs(k) > kEps)) {
            continue;
        }
        JointStep raw{0.0F, 0.0F, false, false};
        if (j == 0U) {
            raw.dp1 = k;
        } else {
            raw.dp2 = k;
        }
        cand[count] = flatten_to_rect(J, raw, fx, fy, box, damping);
        ++count;
    }

    for (unsigned c = 0U; c < count; ++c) {
        const JointStep step = cand[c];
        if (!(std::fabs(step.dp1) > kEps) && !(std::fabs(step.dp2) > kEps)) {
            continue;
        }
        if (step.fixed1 && step.fixed2) {
            continue; // 两轴都在边界上，这个方向上彻底走不动
        }
        for (unsigned i = 0U; i < kScaleCount; ++i) {
            const float s = kScales[i];
            const float raw1 = q1 + step.dp1 * s;
            const float raw2 = q2 + step.dp2 * s;
            const float n1 = clampf(raw1, limits[0].min_percent, limits[0].max_percent);
            const float n2 = clampf(raw2, limits[1].min_percent, limits[1].max_percent);
            const bool cut1 = step.fixed1 || (std::fabs(n1 - raw1) > kEps);
            const bool cut2 = step.fixed2 || (std::fabs(n2 - raw2) > kEps);
            if (!(std::fabs(n1 - q1) > kEps) && !(std::fabs(n2 - q2) > kEps)) {
                continue; // 这一步什么也没动，再小的步长也没用
            }
            if (!feasible_at(model, n1, n2, box, base_over, tol)) {
                continue;
            }
            out.p1_percent = n1;
            out.p2_percent = n2;
            out.cut1 = cut1;
            out.cut2 = cut2;
            out.slide = cut1 || cut2 || (c > 0U) || (i > 0U);
            return out;
        }
    }

    out.stuck = true; // 所有候选都走不通：正对墙 / 墙角 / 奇异点
    return out;
}

} // namespace arm_limit
