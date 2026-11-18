/**
 ******************************************************************************
 * @file           : struct_probe.cpp
 * @brief          : 结构限位探针：把 Include/arm_config.hpp 的真值 + 固件 solve_xy
 *                   的真实结论打出来，供 sim/verify/struct_limit.js 判 PASS/FAIL
 ******************************************************************************
 *
 * 用法（在仓库根目录，一行）：
 *   g++ -std=c++14 -O2 -I Include -I Algorithm/inc -I Device/inc
 *       Algorithm/src/arm_kinematics.cpp sim/verify/struct_probe.cpp -o sim/verify/struct_probe.exe
 *   sim/verify/struct_probe.exe
 *
 * 为什么用探针而不是在 JS 里再抄一份限位：kGeom / kWorkspace / kPoses 只有 arm_config.hpp
 * 一个真值。抄到 JS 里迟早漂移（这条流水线已经踩过：固件改了限位、仿真还挂着老值）。
 *
 * 输出（每行一条，空白分隔）：
 *   geom   L1 L2 zero1 dpp1 min1 max1 zero2 dpp2 min2 max2 elbow_sign
 *   limits pct_min1 pct_max1 pct_min2 pct_max2
 *   ws     x_min x_max y_min y_max
 *   pose   i x y reachable limited s1_pct s2_pct th1_deg th2_deg
 *   selftest bad_mask
 */
#include "arm_config.hpp"
#include "arm_kinematics.hpp"

#include <cstdio>

int main() {
    const arm_kinematics::Model &g = arm_config::kGeom;
    std::printf("geom %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %d\n",
                static_cast<double>(g.upper_mm), static_cast<double>(g.fore_mm),
                static_cast<double>(g.shoulder.zero_deg),
                static_cast<double>(g.shoulder.deg_per_percent),
                static_cast<double>(g.shoulder.range_min_deg),
                static_cast<double>(g.shoulder.range_max_deg),
                static_cast<double>(g.elbow.zero_deg), static_cast<double>(g.elbow.deg_per_percent),
                static_cast<double>(g.elbow.range_min_deg),
                static_cast<double>(g.elbow.range_max_deg), g.elbow_sign);

    std::printf("limits %.6f %.6f %.6f %.6f\n",
                static_cast<double>(arm_config::kJointLimits[arm_config::kShoulderServo].min_percent),
                static_cast<double>(arm_config::kJointLimits[arm_config::kShoulderServo].max_percent),
                static_cast<double>(arm_config::kJointLimits[arm_config::kElbowServo].min_percent),
                static_cast<double>(arm_config::kJointLimits[arm_config::kElbowServo].max_percent));

    std::printf("ws %.6f %.6f %.6f %.6f\n", static_cast<double>(arm_config::kWorkspace.x_min_mm),
                static_cast<double>(arm_config::kWorkspace.x_max_mm),
                static_cast<double>(arm_config::kWorkspace.y_min_mm),
                static_cast<double>(arm_config::kWorkspace.y_max_mm));

    unsigned bad = 0U;
    for (unsigned i = 0U; i < arm_config::kPoseCount; ++i) {
        const arm_config::Preset &p = arm_config::kPoses[i];
        const arm_kinematics::Solution s = arm_kinematics::solve_xy(g, p.x_mm, p.y_mm);
        if (!s.reachable || s.limited) {
            bad |= 1U << i;
        }
        std::printf("pose %u %.6f %.6f %d %d %.6f %.6f %.6f %.6f\n", i,
                    static_cast<double>(p.x_mm), static_cast<double>(p.y_mm),
                    s.reachable ? 1 : 0, s.limited ? 1 : 0,
                    static_cast<double>(s.shoulder_percent),
                    static_cast<double>(s.elbow_percent), static_cast<double>(s.shoulder_deg),
                    static_cast<double>(s.elbow_deg));
    }

    std::printf("selftest %u\n", bad);
    return 0;
}
