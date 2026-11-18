/**
 ******************************************************************************
 * @file           : probe.cpp
 * @brief          : 对拍用探针：编译真实的 Algorithm/src/arm_kinematics.cpp，
 *                   从 stdin 读点，把解算结果按行打到 stdout
 ******************************************************************************
 *
 * 用法：probe <ik|fk> <L1> <L2> <elbow_sign>
 *   ik 模式：每行 "x y"   → 输出 "x y θ1 θ2 %1 %2 reachable limited"
 *   fk 模式：每行 "p1 p2" → 输出 "x y"
 *
 * 只为 sim/verify/compare.js 服务，不进固件构建。
 */
#include "arm_kinematics.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

int main(int argc, char **argv) {
    if (argc < 5) {
        std::fprintf(stderr, "usage: probe <ik|fk> <L1> <L2> <elbow_sign>\n");
        return 2;
    }
    const bool fkMode = (std::strcmp(argv[1], "fk") == 0);

    arm_kinematics::Model model{};
    model.upper_mm = static_cast<float>(std::atof(argv[2]));
    model.fore_mm = static_cast<float>(std::atof(argv[3]));
    model.shoulder = {90.0F, 0.9F, 0.0F, 180.0F};
    model.elbow = {45.0F, 0.9F, -45.0F, 135.0F};
    model.elbow_sign = std::atoi(argv[4]);

    float a = 0.0F;
    float b = 0.0F;
    while (std::scanf("%f %f", &a, &b) == 2) {
        if (fkMode) {
            float x = 0.0F;
            float y = 0.0F;
            arm_kinematics::solve_forward(model, a, b, x, y);
            std::printf("%.6f %.6f\n", static_cast<double>(x), static_cast<double>(y));
        } else {
            const arm_kinematics::Solution s = arm_kinematics::solve_xy(model, a, b);
            std::printf("%.6f %.6f %.6f %.6f %.6f %.6f %d %d\n",
                        static_cast<double>(s.x_mm), static_cast<double>(s.y_mm),
                        static_cast<double>(s.shoulder_deg), static_cast<double>(s.elbow_deg),
                        static_cast<double>(s.shoulder_percent), static_cast<double>(s.elbow_percent),
                        s.reachable ? 1 : 0, s.limited ? 1 : 0);
        }
    }
    return 0;
}
