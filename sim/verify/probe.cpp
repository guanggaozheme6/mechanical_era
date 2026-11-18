/**
 ******************************************************************************
 * @file           : probe.cpp
 * @brief          : 对拍用探针：编译真实的 Algorithm/src/arm_kinematics.cpp，
 *                   从 stdin 读点，把解算结果按行打到 stdout
 ******************************************************************************
 *
 * 用法：probe <ik|fk> <L1> <L2> <elbow_sign> [dpp1 zero1 min1 max1 dpp2 zero2 min2 max2]
 *   ik 模式：每行 "x y"   → 输出 "x y θ1 θ2 %1 %2 reachable limited"
 *   fk 模式：每行 "p1 p2" → 输出 "x y"
 *
 * 限位不再写死在本文件：sim/verify/compare.js 从 arm_core.js 的默认几何（= arm_config.hpp 的
 * kGeom）抄 8 个数传进来。以前这里硬编码 0/180 与 −45/135，arm_config 一改限位，
 * 对拍就悄悄跑在旧几何上 —— 那种漂移比不测更糟。
 *
 * 只为 sim/verify/compare.js 服务，不进固件构建。
 */
#include "arm_kinematics.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

int main(int argc, char **argv) {
    if (argc < 5) {
        std::fprintf(stderr,
                     "usage: probe <ik|fk> <L1> <L2> <elbow_sign> "
                     "[dpp1 zero1 min1 max1 dpp2 zero2 min2 max2]\n");
        return 2;
    }
    const bool fkMode = (std::strcmp(argv[1], "fk") == 0);

    arm_kinematics::Model model{};
    model.upper_mm = static_cast<float>(std::atof(argv[2]));
    model.fore_mm = static_cast<float>(std::atof(argv[3]));
    model.shoulder = {90.0F, 0.9F, 7.1F, 172.9F};  // 缺省 = 2026-11-18 的 kGeom 限位
    model.elbow = {45.0F, 0.9F, -42.0F, 132.0F};
    model.elbow_sign = std::atoi(argv[4]);
    if (argc >= 13) {
        model.shoulder = {static_cast<float>(std::atof(argv[6])),
                          static_cast<float>(std::atof(argv[5])),
                          static_cast<float>(std::atof(argv[7])),
                          static_cast<float>(std::atof(argv[8]))};
        model.elbow = {static_cast<float>(std::atof(argv[10])),
                       static_cast<float>(std::atof(argv[9])),
                       static_cast<float>(std::atof(argv[11])),
                       static_cast<float>(std::atof(argv[12]))};
    }

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
