/**
 ******************************************************************************
 * @file           : stick_probe.cpp
 * @brief          : 对拍用探针：编译真实的 Algorithm/src/arm_limit.cpp，
 *                   从 stdin 读「配置 + 一串摇杆步」，把结果按行打到 stdout
 ****************************************************************************** *
 * 用法：stick_probe < config_and_steps.txt
 *   首行 17 个数：L1 L2 dpp1 zero1 min1 max1 dpp2 zero2 min2 max2 sign
 *                 damping tol xmin xmax ymin ymax
 *   之后每行 5 个数：p1 p2 vx vy dt
 *   每行输出：p1 p2 slide cut1 cut2 stuck idle
 *
 * 只为 sim/verify/stick_port_test.js 服务，不进固件构建。
 * 轴序固定：1 = 大臂（shoulder），2 = 小臂（elbow）。
 */
#include "arm_limit.hpp"

#include <cstdio>
#include <cstdlib>

namespace {

/**
 * @brief 角度 → 行程百分比（与 arm_config::percent_of 同式）
 * @param deg 连杆角（度）
 * @param zero_deg 行程 0% 对应的连杆角
 * @param deg_per_percent 1% 行程对应的角度
 * @return 行程百分比；deg_per_percent 为 0 时返回 0
 */
float percent_of(float deg, float zero_deg, float deg_per_percent) {
    return deg_per_percent == 0.0F ? 0.0F : (deg - zero_deg) / deg_per_percent;
}

} // namespace

int main() {
    float l1 = 0.0F;
    float l2 = 0.0F;
    float dpp1 = 0.0F;
    float zero1 = 0.0F;
    float min1 = 0.0F;
    float max1 = 0.0F;
    float dpp2 = 0.0F;
    float zero2 = 0.0F;
    float min2 = 0.0F;
    float max2 = 0.0F;
    int sign = 1;
    float damping = 0.0F;
    float tol = 0.0F;
    float xmin = 0.0F;
    float xmax = 0.0F;
    float ymin = 0.0F;
    float ymax = 0.0F;

    if (std::scanf("%f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f", &l1, &l2, &dpp1, &zero1,
                   &min1, &max1, &dpp2, &zero2, &min2, &max2, &sign, &damping, &tol, &xmin, &xmax,
                   &ymin, &ymax) != 17) {
        std::fprintf(stderr, "stick_probe: 首行需要 17 个数（几何 + 限位 + 调参 + 矩形）\n");
        return 2;
    }

    arm_kinematics::Model model{};
    model.upper_mm = l1;
    model.fore_mm = l2;
    model.shoulder = {zero1, dpp1, min1, max1};
    model.elbow = {zero2, dpp2, min2, max2};
    model.elbow_sign = sign;

    const arm_limit::Span limits[2] = {{percent_of(min1, zero1, dpp1), percent_of(max1, zero1, dpp1)},
                                       {percent_of(min2, zero2, dpp2), percent_of(max2, zero2, dpp2)}};
    const arm_limit::Box box{xmin, xmax, ymin, ymax};
    const arm_limit::Params params{damping, 0.0F, tol}; // sag 固定 0：JS 侧没有这条

    float p1 = 0.0F;
    float p2 = 0.0F;
    float vx = 0.0F;
    float vy = 0.0F;
    float dt = 0.0F;
    while (std::scanf("%f %f %f %f %f", &p1, &p2, &vx, &vy, &dt) == 5) {
        const arm_limit::Step step = arm_limit::tangent_step(model, limits, box, params, p1, p2, vx,
                                                             vy, dt);
        std::printf("%.6f %.6f %d %d %d %d %d\n", static_cast<double>(step.p1_percent),
                    static_cast<double>(step.p2_percent), step.slide ? 1 : 0, step.cut1 ? 1 : 0,
                    step.cut2 ? 1 : 0, step.stuck ? 1 : 0, step.idle ? 1 : 0);
    }
    return 0;
}
